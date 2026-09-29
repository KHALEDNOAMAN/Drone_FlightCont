#!/usr/bin/env python3
"""Closed-loop simulation of P7 stage 2 - the REAL flight C, in a loop
with a plant model.

    python tools/alt_hold_sim.py
    python tools/alt_hold_sim.py --plot

WHAT THIS IS FOR
----------------
Log B has not been flown. Everything about the alt-hold controller that
matters - is the sign right, is the loop stable, does the ToF handover
at 2 m throw it, does the voltage feedforward actually cancel the sag -
is unanswerable from Log A, which is 103 s of an aircraft sitting still.

So the aircraft is simulated and the CONTROLLER IS NOT. alt_hold.c and
vert_filter.c are compiled from the real sources and called through
ctypes, exactly as the flight loop calls them. A sign error, a windup
bug or an unstable gain shows up here rather than at 2 m over concrete.

WHAT IT IS NOT
--------------
Not a substitute for Log B. The plant below is a first-order motor lag
and a quadratic thrust curve; it has no ground effect, no prop wash off
the floor, no airframe flex, and its vibration is white noise rather
than the rotor-order comb the real accelerometer sees. It can prove the
loop is WRONG. It cannot prove the gains are right.

The numbers it needs from Log B are listed at the bottom of the output.
"""

import argparse
import ctypes
import math
import random
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from host_c import HostC, CVertFilter, CAltHoldCfg, CAltHoldState  # noqa: E402

G = 9.80665
DT = 1.0 / 250.0
BARO_DIV = 5          # 50 Hz, matches BARO_MAG_DIVIDER
# The ToF DELIVERS at 1000/tof_budget_ms Hz - 5 Hz at the 200 ms this
# airframe boots with - NOT at the ~9 Hz the bus owner POLLS it at. The
# fusion loop is seq-gated (fusion.c), so a delivered sample is what the
# filter actually sees. This was 28 (9 Hz), which meant every check in
# this file, including the crash regression, certified the loop against
# nearly twice the ground fixes the aircraft gets.
TOF_DIV = 50          # 5 Hz at tof_budget_ms = 200
TOF_MAX_RANGE = 2.0   # VL53L0X indoor ceiling

# --- measured on this airframe -------------------------------------------
MASS_KG = 1.2              # PLACEHOLDER - not weighed. Only sets how fast
                           # the plant responds, not the loop's stability.
ACC_BIAS = 0.194           # Log A, measured
ACC_SIGMA_IDLE = 0.294     # Log A, measured (idle - hover is worse)
# MEASURED in powered flight, replacing the old 2x-idle guess
# of 0.588: 1.774 (ceiling_s35), 1.819 (s30), 2.831 (s33) m/s^2. The sim
# must run at the noise the aircraft actually sees, or it certifies gains
# against a quieter world than the one they fly in. See
# tools/vert_filter.py ACC_SIGMA_HOVER_MULT.
ACC_SIGMA_HOVER = 2.0
BARO_SIGMA = 0.073         # Log A, measured
BARO_DRIFT = -0.0018       # Log A: -0.18 m over 103 s
TOF_SIGMA = 0.0014         # Log A, measured at 0.14 m
HOVER_THR = 420.0          # flight4 s19-s22, measured
MOTOR_TAU = 0.08           # PLACEHOLDER - ESC + prop inertia


class Plant:
    """Vertical axis only. Thrust is quadratic in both throttle and pack
    voltage, which is the physics that makes the voltage feedforward
    necessary in the first place: RPM follows voltage, thrust follows
    RPM squared, so a sagging pack loses thrust at a fixed throttle."""

    def __init__(self, v0=16.8, seed=1):
        self.h = 0.0
        self.w = 0.0
        self.thr = 1000.0        # motor state, lags the command
        self.v = v0
        self.v0 = v0
        self.t = 0.0
        self.rng = random.Random(seed)
        self.accz = 0.0

    def step(self, cmd_us, dt, sag_rate=0.0):
        # First-order motor/prop lag
        self.thr += (cmd_us - self.thr) * (dt / MOTOR_TAU)

        # Thrust normalised so (HOVER_THR, v_ref) gives exactly 1 g.
        span = max(self.thr - 1000.0, 0.0)
        rpm = (span / HOVER_THR) * (self.v / 16.0)
        acc = G * rpm * rpm - G

        # A little aerodynamic damping so the sim is not a pure double
        # integrator (a real quad has plenty).
        acc -= 0.35 * self.w * abs(self.w)

        self.w += acc * dt
        self.h += self.w * dt
        if self.h < 0.0:                 # the floor
            self.h = 0.0
            if self.w < 0.0:
                self.w = 0.0
        self.accz = acc
        self.t += dt

        # Pack: resting decay plus load-proportional sag.
        self.v = self.v0 - sag_rate * self.t - 1.2 * (span / 1000.0)
        return acc


class Sensors:
    def __init__(self, seed=2):
        self.rng = random.Random(seed)

    def accz(self, true_acc, sigma):
        return true_acc + ACC_BIAS + self.rng.gauss(0.0, sigma)

    def baro(self, h, t):
        # +84 m of Istanbul, so the estimator's baro-bias state has a
        # realistically large offset to swallow rather than a token one.
        return 84.0 + h + BARO_DRIFT * t + self.rng.gauss(0.0, BARO_SIGMA)

    def tof(self, h):
        if h > TOF_MAX_RANGE:
            return None                  # NO_TARGET
        return max(0.0, h + self.rng.gauss(0.0, max(TOF_SIGMA, 0.02 * h)))


def run(hc, profile, seconds, acc_sigma, sag_rate=0.0, v0=16.8,
        cfg_over=None, tof_dead_at=None):
    """tof_dead_at: seconds after which the ToF STOPS DELIVERING SAMPLES
    ENTIRELY - no update, and no tof_lost either.

    That distinction is the whole crash. A ToF that reports
    NO_TARGET is a live sensor saying "nothing in range", and the fusion
    loop hears it. A ToF whose power wire has fallen out still ACKs, so
    the driver's sample counter simply stops moving, the fusion loop
    never enters its fresh-sample branch, and NOTHING is ever called.
    Modelling it as tof_lost() would test the case that already worked."""
    L = hc.lib
    vf = CVertFilter()
    ah = CAltHoldState()
    cfg = CAltHoldCfg()
    L.vf_init(ctypes.byref(vf), ctypes.c_float(acc_sigma), ctypes.c_float(0.0))
    L.ah_init(ctypes.byref(ah))
    L.ah_defcfg(ctypes.byref(cfg))
    if cfg_over:
        for k, v in cfg_over.items():
            setattr(cfg, k, v)

    plant = Plant(v0=v0)
    sens = Sensors()
    n = int(seconds / DT)
    log = []

    for i in range(n):
        t = i * DT
        want, stick = profile(t, plant)

        # --- estimator, exactly as the fusion loop drives it ---
        L.vf_predict(ctypes.byref(vf),
                     ctypes.c_float(sens.accz(plant.accz, acc_sigma)),
                     ctypes.c_float(DT))
        if i % BARO_DIV == 0:
            L.vf_baro(ctypes.byref(vf), ctypes.c_float(sens.baro(plant.h, t)))
        tof_alive = (tof_dead_at is None) or (t < tof_dead_at)
        if i % TOF_DIV == 0 and tof_alive:
            m = sens.tof(plant.h)
            if m is None:
                L.vf_toflost(ctypes.byref(vf))
            else:
                # tilt_cos = 1: this sim has no attitude axis.
                L.vf_tof(ctypes.byref(vf), ctypes.c_float(m),
                         ctypes.c_float(1.0))

        shift = L.vf_takeshift(ctypes.byref(vf))

        # --- controller ---
        # The firmware's alt_want gate (fc_tasks.c) includes vert_valid,
        # so the sim must too. Without this the sim would happily certify
        # a hold the aircraft will refuse to perform - and, worse, would
        # keep flying an estimate the firmware has already disowned.
        want = want and bool(vf.valid)

        cmd = L.ah_update(ctypes.byref(ah), ctypes.byref(cfg),
                          1 if want else 0, ctypes.c_uint16(int(stick)),
                          ctypes.c_float(vf.z), ctypes.c_float(vf.vz),
                          ctypes.c_float(shift), ctypes.c_float(plant.v),
                          ctypes.c_float(DT))

        plant.step(cmd, DT, sag_rate)
        log.append((t, plant.h, plant.w, vf.z, vf.vz, cmd, plant.v,
                    ah.z_sp, ah.integ, 1 if ah.engaged else 0,
                    1 if vf.valid else 0))
    return log


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def pilot(target):
    """A crude human holding altitude on the throttle stick, used only
    for the MANUAL phase before alt-hold engages. Without it a fixed
    stick position just accelerates: the first version of these tests
    said "climb to 1.2 m" and actually parked the aircraft at 12 m, on
    baro alone, and then blamed the resulting +-0.2 m on the
    controller."""
    def f(plant):
        u = 1000.0 + HOVER_THR + 60.0 * (target - plant.h) - 90.0 * plant.w
        return int(max(1100.0, min(1600.0, u)))
    return f


def _hold_case(hc, acc_sigma, target, name, err_lim, est_lim, note):
    fly = pilot(target)

    held = {}

    def prof(t, plant):
        if t < 10.0:
            return False, fly(plant)
        # The pilot flips CH6 and does NOT move the throttle stick. That
        # is the whole point of capturing neutral at engage.
        held.setdefault("us", fly(plant))
        return True, held["us"]
    log = run(hc, prof, 50.0, acc_sigma)
    eng = [r for r in log if r[9]]
    settle = [r for r in eng if r[0] > eng[0][0] + 5.0]
    target_z = settle[0][7]
    err = [abs(r[1] - target_z) for r in settle]
    est = [abs(r[1] - r[3]) for r in settle]
    return {
        "name": name,
        "note": note,
        "checks": [
            ("stays within %.2f m of the setpoint" % err_lim,
             max(err) < err_lim, "worst %.3f m" % max(err)),
            ("no vertical oscillation (|w| < 0.35 m/s)",
             max(abs(r[2]) for r in settle) < 0.35,
             "peak %.3f m/s" % max(abs(r[2]) for r in settle)),
            ("estimate tracks truth within %.2f m" % est_lim,
             max(est) < est_lim, "worst %.3f m" % max(est)),
        ],
        "log": log,
    }


def sc_hold_tof(hc, acc_sigma):
    """Hold at 1.2 m - inside the ToF's range, which is where the first
    real alt-hold test flight will happen.

    LIMITS WIDENED, and this is a measured degradation, not a
    relaxed standard. The old 0.10/0.05 m limits were met at an accel
    sigma of 0.588 m/s^2 - an idle bench figure doubled as a guess. At
    the sigma real flight actually measures (2.0), the identical code
    and gains give 0.100 m of hold error and 0.059 m of estimate error.
    Re-run with `--acc-sigma 0.588` to see the old numbers reproduce
    exactly: nothing regressed, the quiet world was never real.

    So ~0.10 m is what a ToF-referenced hold is worth on this airframe,
    not the ~0.05 m previously advertised. Tightening this needs less
    VIBRATION - balanced props, soft-mounted IMU - not different gains."""
    return _hold_case(
        hc, acc_sigma, 1.2, "hold at 1.2 m (ToF in range)", 0.15, 0.08,
        "the ToF is the altitude reference here, so this is the tight case")


def sc_tof_death(hc, acc_sigma):
    """THE CRASH, as a regression test.

    The aircraft is holding at 1.2 m on a healthy ToF when the sensor's
    power wire falls out. It keeps ACKing, so the sample counter simply
    stops and the fusion loop never hears anything again - no update, no
    tof_lost. Before the fix, `valid` stayed true (the BAROMETER was
    setting it), the estimate dead-reckoned into a false dive, and the
    controller answered -3.4 m/s with +264 us of thrust.

    What must happen now: `valid` expires within the coast window, the
    firmware's alt_want gate drops, and the throttle ramps back to the
    pilot's stick WITHOUT a thrust excursion on the way out."""
    fly = pilot(1.2)
    held = {}

    def prof(t, plant):
        if t < 10.0:
            return False, fly(plant)
        held.setdefault("us", fly(plant))
        return True, held["us"]

    dead_at = 20.0
    log = run(hc, prof, 40.0, acc_sigma, tof_dead_at=dead_at)

    after = [r for r in log if r[0] >= dead_at]
    lost = [r for r in after if not r[10]]
    drop_s = (lost[0][0] - dead_at) if lost else float("inf")

    # Thrust excursion between the sensor dying and the mode releasing:
    # this is the number that hit the ceiling.
    window = [r for r in after if r[0] <= dead_at + 6.0]
    peak_cmd = max(r[5] for r in window)
    hover_cmd = 1000.0 + HOVER_THR

    # Once released, the throttle belongs to the stick again.
    tail = [r for r in log if r[0] > dead_at + 6.0]
    still_engaged = any(r[9] for r in tail)

    return {
        "name": "ToF DIES SILENTLY mid-hold (regression)",
        "note": "the sensor stops answering entirely - no NO_TARGET, no "
                "error, just silence. This is the crash.",
        "checks": [
            ("estimator drops `valid` within 4 s",
             drop_s <= 4.0,
             "%.2f s" % drop_s if lost else "NEVER - THIS IS THE CRASH"),
            ("alt-hold releases the throttle",
             not still_engaged,
             "released" if not still_engaged else "STILL ENGAGED"),
            ("no thrust excursion on the way out (< +150 us over hover)",
             peak_cmd - hover_cmd < 150.0,
             "peak %+.0f us over hover" % (peak_cmd - hover_cmd)),
        ],
        "log": log,
    }


def sc_above_tof_range(hc, acc_sigma):
    """Climbing out of ToF range now ENDS the hold rather than coasting
    on the barometer indefinitely.

    That capability was designed in and is deliberately withdrawn. It
    rested on Log A's BENCH baro numbers (0.073 m sigma, 0.11 m pk-pk).
    Real flight measures 0.66 m pk-pk per second and 1.10 m at worst -
    six times worse, from prop wash and cabin pressure. A 0.1-0.2 m hold
    on that signal was never available; and "the ToF is out of range" is
    indistinguishable from "the ToF is dead", which is what killed the
    aircraft. Below 2 m the ToF is a real reference and the hold is real.
    Above it, the pilot flies."""
    fly = pilot(1.2)
    held = {}

    def prof(t, plant):
        if t < 10.0:
            return False, fly(plant)
        held.setdefault("us", fly(plant))
        if t < 20.0:
            return True, held["us"]
        if t < 26.0:
            return True, 1700      # climb out through the ToF ceiling
        return True, held["us"]    # then back to a hover-ish stick, so
                                   # the tail measures the handback and
                                   # not an unconstrained sim runaway

    log = run(hc, prof, 45.0, acc_sigma)
    climbed = max(r[1] for r in log)
    tail = [r for r in log if r[0] > 32.0]
    released = tail and not any(r[9] for r in tail)

    return {
        "name": "climb out of ToF range -> hold ends, pilot flies",
        "note": "WITHDRAWN CAPABILITY, on purpose - see the docstring. "
                "Baro-only hold is not supported by the measured noise.",
        "checks": [
            ("actually left ToF range (> 2 m)", climbed > TOF_MAX_RANGE,
             "reached %.2f m" % climbed),
            ("hold released rather than coasting on the baro",
             bool(released), "released" if released else "STILL ENGAGED"),
        ],
        "log": log,
    }


def sc_bumpless(hc, acc_sigma):
    """The throttle command must not jump when the switch is flipped.

    Engage from a SETTLED hover on purpose. The first version of this
    test engaged mid-climb and called the resulting 57 us step a
    failure, but that step was the P term correctly reacting to 2.3 m/s
    of real climb rate - the controller doing its job, not a transfer
    bug. Bumpless means the FEEDFORWARD and INTEGRATOR hand over without
    a step; it does not mean the loop ignores the aircraft's motion."""
    fly = pilot(1.2)

    held = {}

    def prof(t, plant):
        if t < 10.0:
            return False, fly(plant)
        held.setdefault("us", fly(plant))
        return True, held["us"]
    log = run(hc, prof, 16.0, acc_sigma)
    i = next(k for k, r in enumerate(log) if r[9])
    jump = abs(log[i][5] - log[i - 1][5])
    vz_at = log[i][2]
    return {
        "name": "bumpless engage",
        "checks": [("throttle step at engage < 20 us",
                    jump < 20.0,
                    "%.1f us  (vz was %.3f m/s)" % (jump, vz_at))],
        "log": log,
    }


def sc_handover(hc, acc_sigma):
    """Climb THROUGH the ToF's 2 m ceiling. This is the moment the
    estimator hands altitude from the ToF to the barometer, and the
    whole reason the filter carries a baro-bias state.

    ESTIMATOR-ONLY. The frozen-bias handover is still
    correct and still worth testing - the estimate must not step when
    the reference changes. What is no longer claimed is that the
    CONTROLLER keeps holding afterwards: `valid` expires a few seconds
    above ToF range and the hold is handed back. sc_above_tof_range
    covers that side, and the docstring there says why."""
    fly = pilot(1.0)

    held = {}

    def prof(t, plant):
        if t < 8.0:
            return False, fly(plant)
        held.setdefault("us", fly(plant))
        if t < 20.0:
            return True, 1900          # stick up = commanded climb
        return True, held["us"]        # back to neutral = hold
    log = run(hc, prof, 40.0, acc_sigma)
    cross = [k for k in range(1, len(log))
             if log[k - 1][1] <= TOF_MAX_RANGE < log[k][1]]
    if not cross:
        return {"name": "ToF -> baro handover",
                "checks": [("aircraft reached 2 m", False,
                            "peak %.2f m" % max(r[1] for r in log))],
                "log": log}
    k = cross[0]
    # Estimator error either side of the ceiling.
    before = max(abs(r[1] - r[3]) for r in log[max(0, k - 250):k])
    after = max(abs(r[1] - r[3]) for r in log[k:k + 500])
    return {
        "name": "ToF -> baro handover (estimator only)",
        "checks": [
            ("no estimator step crossing 2 m",
             after < before + 0.15,
             "err %.3f -> %.3f m" % (before, after)),
        ],
        "log": log,
    }


def sc_sag(hc, acc_sigma):
    """Hold altitude while the pack sags 16.8 -> ~14 V. Without the
    voltage feedforward this is the flight-4 sink."""
    fly = pilot(1.5)

    held = {}

    def prof(t, plant):
        if t < 10.0:
            return False, fly(plant)
        held.setdefault("us", fly(plant))
        return True, held["us"]
    log = run(hc, prof, 70.0, acc_sigma, sag_rate=0.035)
    eng = [r for r in log if r[9]]
    settle = [r for r in eng if r[0] > eng[0][0] + 5.0]
    target = settle[0][7]
    err = [abs(r[1] - target) for r in settle]
    return {
        "name": "battery sag 16.8 -> %.1f V" % log[-1][6],
        "checks": [
            ("holds within 0.25 m through the sag",
             max(err) < 0.25, "worst %.3f m" % max(err)),
            ("no runaway (final within 0.25 m)",
             abs(log[-1][1] - target) < 0.25,
             "%.3f m" % abs(log[-1][1] - target)),
        ],
        "log": log,
    }


def sc_step(hc, acc_sigma):
    """A commanded climb, then hold. Tracking, not just regulation.

    Deliberately kept INSIDE ToF range (0.7 -> ~1.4 m): the climb used
    to run to 1900 us and end up above 2 m, where the hold no longer
    exists. A tracking test that leaves the envelope
    stops measuring tracking and starts measuring the handback."""
    fly = pilot(0.7)

    held = {}

    def prof(t, plant):
        if t < 10.0:
            return False, fly(plant)
        held.setdefault("us", fly(plant))
        if t < 16.0:
            return True, held["us"]
        if t < 22.0:
            return True, held["us"] + 120   # stick up = gentle climb
        return True, held["us"]
    log = run(hc, prof, 45.0, acc_sigma)
    climbed = max(r[1] for r in log) - log[int(15.5 / DT)][1]
    tail = log[-1250:]
    ripple = max(r[1] for r in tail) - min(r[1] for r in tail)
    return {
        "name": "commanded climb then hold",
        "checks": [
            ("climbs on stick (> 0.4 m)", climbed > 0.4,
             "%.2f m" % climbed),
            ("settles after the climb (ripple < 0.2 m)",
             ripple < 0.2, "%.3f m" % ripple),
        ],
        "log": log,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--acc-sigma", type=float, default=ACC_SIGMA_HOVER,
                    help="accel noise in the SIM (default: the MEASURED "
                         "in-flight hover sigma, 2.0 m/s^2)")
    args = ap.parse_args()

    print("P7 stage 2 closed-loop sim - real alt_hold.c + vert_filter.c\n"
          "accel sigma %.3f m/s^2, hover FF %.0f us, 250 Hz\n"
          % (args.acc_sigma, HOVER_THR))

    ok = True
    with HostC() as hc:
        results = [f(hc, args.acc_sigma) for f in
                   (sc_bumpless, sc_hold_tof, sc_tof_death,
                    sc_above_tof_range, sc_step, sc_handover, sc_sag)]

    for r in results:
        print("=== %s ===" % r["name"])
        if r.get("note"):
            print("  (%s)" % r["note"])
        for name, good, detail in r["checks"]:
            print("  [%s] %-45s %s"
                  % ("PASS" if good else "FAIL", name, detail))
            ok &= good
        print()

    if args.plot:
        plot(results)

    print("MEASURED since this sim was written:")
    print("  * hover accz sigma = 2.0 m/s^2 (ceiling_s30/s33/s35,")
    print("). Re-measure from a CLEAN hover once the")
    print("    airframe is repaired and raise it if that hover says so.")
    print()
    print("STILL NEEDS A REAL FLIGHT - the sim cannot supply these:")
    print("  * the real hover throttle at a KNOWN pack voltage, which is")
    print("    what alt_vref should be set to (flight4 predates vbat_mv)")
    print("  * ground effect below ~1 rotor diameter")
    print("  * that the LINEAR voltage term is right on the real ESC")
    print("    curve (hover at fixed altitude, watch integ vs vbat - flat")
    print("    integrator while the pack sags = the linear term is correct)")
    print("  * kp_vz / ki_vz: this sim shows them STABLE, not CORRECT")
    return 0 if ok else 1


def plot(results):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed - skipping plots)")
        return
    fig, axes = plt.subplots(len(results), 1, figsize=(11, 3 * len(results)))
    for ax, r in zip(axes, results):
        log = r["log"]
        t = [x[0] for x in log]
        ax.plot(t, [x[1] for x in log], lw=1.0, label="true altitude")
        ax.plot(t, [x[3] for x in log], lw=0.8, label="estimate")
        ax.plot(t, [x[7] if x[9] else float("nan") for x in log], lw=0.8,
                ls="--", label="setpoint")
        ax2 = ax.twinx()
        ax2.plot(t, [x[5] for x in log], lw=0.5, color="gray", alpha=0.6)
        ax2.set_ylabel("thr us", color="gray")
        ax.set_title(r["name"]); ax.set_ylabel("m"); ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=7)
    fig.tight_layout()
    fig.savefig("alt_hold_sim.png", dpi=110)
    print("plot -> alt_hold_sim.png")


if __name__ == "__main__":
    sys.exit(main())
