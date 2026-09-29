#!/usr/bin/env python3
"""Identify the aircraft's rate response from real flight logs, and close
the loop around it in simulation.

WHY A PLANT MODEL IS NOT OPTIONAL
---------------------------------
The obvious way to "replay a flight at new gains" is to push the logged
gyro back through the PID and score the tracking error. That is OPEN
LOOP and it is worthless: at different gains the aircraft would have
moved differently, so the logged gyro is no longer the gyro that would
have been measured. The score then reduces to "bigger gains produce
bigger commands", and it recommends kp = infinity.

To predict anything you need a model of how a motor command becomes an
angular rate. That is what this module fits, from the same logs.

THE MODEL
---------
Per axis, a first-order rigid body behind a first-order actuator, with a
pure transport delay:

    u_eff[k] = u_eff[k-1] + (dt/tau_m) * (u[k-n] - u_eff[k-1])
    w[k+1]   = w[k] + dt * (b * u_eff[k] - a * w[k] + c)

  a      aerodynamic + rotor damping, 1/s
  b      control effectiveness, dps/s per us of motor differential
  c      constant disturbance (trim bias), dps/s
  tau_m  motor + ESC spin-up lag, s
  n      transport delay, whole samples

FITTED BY OUTPUT ERROR, NOT BY REGRESSION
-----------------------------------------
The obvious fit is one-step least squares: regress w[k+1]-w[k] on
w[k] and u[k]. It does not work here, and the way it fails is worth
recording because the numbers look like data.

w[k] appears on BOTH sides - inside the increment on the left and as
the damping regressor on the right - so gyro noise correlates the
regressor with the residual and biases `a` downwards. On this
airframe's logs it drove `a` NEGATIVE, i.e. a model of an aircraft that
accelerates itself, with R2 = 0.18 and a free-run VAF of -209 %. The
loop being CLOSED makes it worse: u is mostly -kp*w plus a term in
dw/dt, so the two regressors are near-collinear and the split between
`a` and `b` is barely identified at all.

So the fit here minimises the FREE-RUN simulation error directly -
output error - over the same short windows the validation uses. It is
nonlinear in (a, b, c, tau_m), which costs a Levenberg-Marquardt solve
instead of a lstsq, and it is not biased by output noise because no
measured w is ever used as a regressor. The delay stays on a grid
because it is an integer.

WHAT `w` MEANS HERE
-------------------
The logged gyro is POST-filter - fusion.c applies the 60 Hz LPF and the
78 Hz notch before the controller ever sees it. So this identifies the
plant AS THE CONTROLLER SEES IT, filters included. That is the right
thing for tuning the loop that closes around it, and it means the model
is only valid for the filter settings that flew. Change gyro_lpf_hz or
notch_hz and the model must be refitted.

WHY THE RESIDUAL IS KEPT
------------------------
Whatever the model cannot explain - vibration, prop wash, gusts, the
gyro's own noise - is measured, per frame, and handed back as
`residual`. The sweep replays it as an additive disturbance, so a
candidate gain set is judged against the real noise floor of this
airframe. Without it a simulation is noiseless, kd looks free, and the
tuner recommends a differential gain that would scream on the bench.
"""

import math

import numpy as np

# Free-run window used for FITTING, in samples at 250 Hz.
FREERUN_WINDOW = 50

# Horizons the fit is REPORTED against, in samples at 250 Hz. One number
# is not enough and picking the wrong one condemns a usable model: this
# airframe fits 93 % at 20 ms and 41 % at 200 ms, and both are true. The
# horizon that decides whether a model can be tuned on is the one the
# feedback acts over - roughly 1/(2*pi*fc), so 10-40 ms for a rate loop
# crossing anywhere near 5-15 Hz. Longer horizons mostly measure how
# gusty the air was.
VAF_HORIZONS = (5, 10, 25, 50)
VAF_GATE_SAMPLES = 10           # 40 ms: the horizon the gate is read at
VAF_GATE_PCT = 75.0

# A model whose VAF barely moves with horizon never tracked at all - a
# good model must DEGRADE as unmodelled disturbance accumulates. Flatness
# is the signature of a fit that has latched onto the mean.
VAF_FLATNESS_PCT = 15.0


class PlantModel:
    """One axis of identified rate dynamics."""

    def __init__(self, axis, a, b, c, tau_m, delay, dt, r2, vaf, n):
        self.axis = axis
        self.a = a              # damping, 1/s
        self.b = b              # effectiveness, dps/s per us
        self.c = c              # constant disturbance, dps/s
        self.tau_m = tau_m      # actuator lag, s
        self.delay = delay      # transport delay, samples
        self.dt = dt            # median cycle, s (for margins)
        self.r2 = r2            # one-step coefficient of determination
        self.vaf = vaf          # {samples: VAF %} by free-run horizon
        self.n = n              # frames used

    def __repr__(self):
        return (f"<{self.axis}: a={self.a:.2f}/s b={self.b:.3f} dps/s/us "
                f"c={self.c:+.1f} tau_m={self.tau_m * 1000:.1f} ms "
                f"delay={self.delay} VAF@40ms={self.vaf_gate:.1f}%>")

    def as_dict(self):
        d = {k: getattr(self, k) for k in
             ("axis", "a", "b", "c", "tau_m", "delay", "dt", "r2", "n")}
        d["vaf"] = {str(k): v for k, v in self.vaf.items()}
        return d

    @property
    def tau_body(self):
        """Open-loop time constant of the airframe alone, s."""
        return 1.0 / self.a if self.a > 0 else float("inf")

    @property
    def vaf_gate(self):
        return self.vaf.get(VAF_GATE_SAMPLES, float("nan"))

    @property
    def trim_us(self):
        """Motor differential the loop must hold just to sit still, us.

        Equilibrium of b*u - a*w + c = 0 at w = 0. It is worth printing
        because it is independently measurable: it must equal the mean
        inverse-mixed command from the same flights, and if it does not,
        the fit is describing something other than this aircraft.
        """
        return -self.c / self.b if self.b else float("nan")

    def verdict(self):
        """-> (grade, why). The gate the sweep refuses to run without."""
        if self.b <= 0:
            return ("REJECTED", "control effectiveness came out negative - "
                    "the model says the command fights itself, which is "
                    "either a sign error or an axis with no real "
                    "excitation in this log")
        gate = self.vaf_gate
        if not math.isfinite(gate):
            return ("REJECTED", "no VAF at the gate horizon")
        span = max(self.vaf.values()) - min(self.vaf.values())
        if span < VAF_FLATNESS_PCT:
            return ("REJECTED", f"VAF is flat across horizons ({span:.1f} "
                    "points) - a model that does not degrade as it runs "
                    "never tracked in the first place")
        if gate >= VAF_GATE_PCT:
            return ("USABLE", f"{gate:.1f}% at 40 ms")
        if gate >= 50.0:
            return ("MARGINAL", f"{gate:.1f}% at 40 ms; treat the ranking "
                    "as a shortlist, not a recommendation")
        return ("REJECTED", f"{gate:.1f}% at 40 ms")


def _apply_actuator(u, dt, tau_m, delay):
    """u -> u_eff: whole-sample delay, then a first-order lag."""
    if delay:
        u = np.concatenate([np.full(delay, u[0]), u[:-delay]])
    if tau_m <= 0:
        return u
    out = np.empty_like(u)
    state = u[0]
    for k in range(len(u)):
        # alpha > 1 is not a faster filter, it is an unstable one. The
        # solver walks tau_m towards zero looking for less lag, so clamp
        # rather than let it explore a region where the model diverges
        # and the cost surface is nonsense.
        alpha = dt[k] / tau_m
        state += (alpha if alpha < 1.0 else 1.0) * (u[k] - state)
        out[k] = state
    return out


def _segments(link, window):
    """Window starts and stops that never cross a break in `link`.

    `link[k]` says frame k+1 really is the cycle after frame k. Frames
    dropped as un-invertible (mixer clamped, scale pinned at the idle
    floor) leave holes, and running the model across one would present a
    jump the aircraft never made as a single cycle's response.
    """
    spans, n, k = [], len(link), 0
    while k < n - 1:
        stop = k
        while stop < n - 1 and link[stop] and stop - k < window:
            stop += 1
        if stop > k:
            spans.append((k, stop))
        k = stop + (0 if link[stop] else 1)
        if stop == k:
            k += 1
    return spans


def _freerun(w, u_eff, dt, spans, a, b, c):
    """Simulate each window from the truth at its start. -> sim (NaN
    where nothing ran)."""
    sim = np.full(len(w), np.nan)
    for start, stop in spans:
        x = w[start]
        for k in range(start, stop):
            sim[k] = x
            x += dt[k] * (b * u_eff[k] - a * x + c)
        sim[stop] = x
    return sim


def _vaf(w, sim):
    """Variance accounted for, %. The honest score: one-step prediction
    is dominated by `w[k+1] ~ w[k]` and looks superb for any model at
    all, while a free run exposes a wrong damping or a wrong gain."""
    ok = ~np.isnan(sim)
    if not ok.any():
        return float("nan")
    var = float(np.var(w[ok]))
    if var <= 0:
        return float("nan")
    return 100.0 * (1.0 - float(np.var(w[ok] - sim[ok])) / var)


def identify(w, u, dt, link=None, axis="?", tau_grid=None,
             delay_grid=(0, 1, 2, 3)):
    """Fit one axis. w in dps, u in us, dt in s, all same length.

    tau_m and the delay are searched on a grid because they enter
    nonlinearly; (a, b, c) are solved exactly at each grid point. The
    grid is coarse on purpose - the point is to stop the linear fit from
    absorbing actuator lag into a fake damping term, not to measure the
    ESC.
    """
    from scipy.optimize import least_squares

    w = np.asarray(w, dtype=float)
    u = np.asarray(u, dtype=float)
    dt = np.asarray(dt, dtype=float)
    if link is None:
        link = np.ones(len(w), dtype=bool)
        link[-1] = False
    link = np.asarray(link, dtype=bool)
    spans = _segments(link, FREERUN_WINDOW)
    if not spans:
        raise ValueError("no contiguous windows to fit")

    # Seed from physics rather than from zero: the body time constant of
    # a small quad is a few hundred ms, and b follows from the rate the
    # aircraft actually reached per us of differential in this log.
    scale_u = float(np.std(u)) or 1.0
    b0 = float(np.std(w)) * 10.0 / scale_u
    seed = [5.0, b0, 0.0, 0.015]

    def residuals(theta, delay):
        a, b, c, tau_m = theta
        u_eff = _apply_actuator(u, dt, max(tau_m, 0.0), delay)
        sim = _freerun(w, u_eff, dt, spans, a, b, c)
        ok = ~np.isnan(sim)
        r = np.zeros(len(w))
        r[ok] = w[ok] - sim[ok]
        # A parameter set that blows the simulation up is a legitimate
        # answer of "not this one", but inf/nan stops the solver dead.
        return np.nan_to_num(r, nan=0.0, posinf=1e6, neginf=-1e6)

    best = None
    for delay in delay_grid:
        try:
            sol = least_squares(residuals, seed, args=(delay,),
                                bounds=([0.0, -np.inf, -np.inf, 0.0],
                                        [200.0, np.inf, np.inf, 0.200]),
                                x_scale=[10.0, max(abs(b0), 1.0), 10.0,
                                         0.02],
                                max_nfev=400)
        except ValueError:
            continue
        if best is None or sol.cost < best[0]:
            best = (sol.cost, sol.x, delay)

    if best is None:
        raise ValueError("output-error fit did not converge")
    _, theta, delay = best
    a, b, c, tau_m = (float(v) for v in theta)

    u_eff = _apply_actuator(u, dt, tau_m, delay)
    vaf = {}
    for horizon in VAF_HORIZONS:
        sim = _freerun(w, u_eff, dt, _segments(link, horizon), a, b, c)
        vaf[horizon] = _vaf(w, sim)

    # One-step residual, kept for the sweep to replay as this airframe's
    # own disturbance. It is the increment the model could not account
    # for, in dps, so it adds straight onto the simulated state.
    pred = np.zeros(len(w))
    pred[link] = dt[link] * (b * u_eff[link] - a * w[link] + c)
    actual = np.zeros(len(w))
    actual[:-1] = w[1:] - w[:-1]
    actual[~link] = 0.0
    resid = actual - pred
    resid[~link] = 0.0

    # One-step R2 on the same quantity, reported only as a sanity check.
    # It is NOT what the fit optimised and it flatters every model.
    ss_tot = float(np.var(actual[link])) * int(link.sum())
    r2 = (1.0 - float(np.sum(resid[link] ** 2)) / ss_tot
          if ss_tot > 0 else float("nan"))

    model = PlantModel(axis, a, b, c, tau_m, delay,
                       float(np.median(dt)), r2, vaf, int(link.sum()))
    return model, resid


def simulate(model, setp, resid, dt, cfg, pid_step, PidState, pid_init,
             out_max=400.0):
    """Close the real firmware PID around the identified plant.

    `setp` is the rate command the pilot actually produced, replayed as
    an exogenous reference; `resid` is the measured, per-frame
    disturbance the model could not explain, replayed so that a
    candidate gain set meets this airframe's own noise rather than a
    clean one.

    Returns (w, u) - simulated rate and the int16-truncated command the
    firmware would have issued.
    """
    n = len(setp)
    w = np.zeros(n)
    u_out = np.zeros(n)
    st = PidState()
    pid_init(st)
    u_eff = 0.0
    hist = [0.0] * (model.delay + 1)
    x = 0.0
    for k in range(n):
        w[k] = x
        # The firmware truncates to int16 before the mixer; a sweep that
        # skips that is scoring a controller with finer resolution than
        # the aircraft has.
        cmd = float(int(pid_step(cfg, st, float(setp[k]), float(x),
                                 float(dt[k]))))
        u_out[k] = cmd
        hist.append(cmd)
        delayed = hist.pop(0) if model.delay else cmd
        if model.tau_m > 0:
            u_eff += (dt[k] / model.tau_m) * (delayed - u_eff)
        else:
            u_eff = delayed
        x += dt[k] * (model.b * u_eff - model.a * x + model.c)
        x += resid[k] if k < len(resid) else 0.0
        # A diverging candidate is information, not a crash. Stop the
        # integration before it overflows and let the caller score it.
        if not math.isfinite(x) or abs(x) > 5000.0:
            w[k:] = math.copysign(5000.0, x if math.isfinite(x) else 1.0)
            u_out[k:] = math.copysign(out_max, cmd)
            break
    return w, u_out


def margins(model, kp, ki, kd, tau=0.015, dt=None, n_pts=4000):
    """Gain and phase margin of the discrete loop, from the identified
    plant and the firmware's exact controller structure.

    The controller is 2-DOF (D acts on the MEASUREMENT, not the error),
    so the transfer that decides stability is measurement -> command:

        C(z) = kp + ki*dt*z/(z-1) + kd*alpha*(1-1/z)/(dt*(1-(1-alpha)/z))

    with alpha = dt/(dt+tau), exactly as pid.c computes it. The plant is
    the delay, the actuator lag and the body:

        P(z) = z^-n * (1-pm)/(z-pm) * b*dt/(z-(1-a*dt))

    Evaluated on the unit circle. Returns (gain_margin_dB,
    phase_margin_deg, crossover_Hz); NaN where no crossing exists.
    """
    dt = dt or model.dt
    alpha = dt / (dt + tau) if tau > 0 else 1.0
    pm_pole = math.exp(-dt / model.tau_m) if model.tau_m > 0 else 0.0

    wn = np.linspace(1e-4, math.pi, n_pts)          # rad/sample
    z = np.exp(1j * wn)

    C = kp + ki * dt * z / (z - 1.0)
    if kd:
        C = C + kd * alpha * (1.0 - 1.0 / z) / (dt * (1.0 - (1.0 - alpha) / z))

    P = model.b * dt / (z - (1.0 - model.a * dt))
    if model.tau_m > 0:
        P = P * (1.0 - pm_pole) / (z - pm_pole)
    if model.delay:
        P = P * z ** (-model.delay)

    L = C * P
    mag = np.abs(L)
    phase = np.unwrap(np.angle(L))

    gm_db = float("nan")
    idx = np.where(np.diff(np.sign(phase + math.pi)))[0]
    if len(idx):
        i = idx[0]
        gm_db = -20.0 * math.log10(max(mag[i], 1e-12))

    pm_deg = float("nan")
    wc_hz = float("nan")
    idx = np.where(np.diff(np.sign(mag - 1.0)))[0]
    if len(idx):
        i = idx[0]
        pm_deg = math.degrees(phase[i] + math.pi)
        wc_hz = wn[i] / dt / (2.0 * math.pi)

    return gm_db, pm_deg, wc_hz
