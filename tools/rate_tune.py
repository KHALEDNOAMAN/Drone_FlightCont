#!/usr/bin/env python3
"""Rate-PID tuner: identify this airframe from real flights, then close
the real firmware PID around it and sweep gains.

    python tools/rate_tune.py sysid s121.csv
    python tools/rate_tune.py sysid 's*.csv' --axis pitch
    python tools/rate_tune.py sweep 's*.csv' --axis pitch

RUN `rate_replay.py verify` FIRST. If the replay cannot reproduce the
flight it came from, nothing here means anything - the same PID code and
the same log plumbing feed both tools, and verify is the cheaper place
to find out they are wrong.

WHAT IS BEING TUNED, EXACTLY
----------------------------
The INNER rate loop, with the logged `setp_*` replayed as an exogenous
reference. That setpoint is the outer angle loop's output, so it already
contains the pilot's command and the attitude error of the flight it came
from; the sweep does not re-simulate attitude, and it does not re-tune
angle_kp. Rate first is the right order anyway - the angle loop is a pure
gain sitting on top, and tuning it against an unsettled inner loop tunes
it against noise.

THREE THINGS MAKE THIS HONEST
-----------------------------
1. The controller is the firmware's own pid.c, through pidcore - not a
   port, and the output is int16-truncated exactly as fc_tasks.c does.
2. The plant is identified from this aircraft's own flights, so a gain
   change actually changes the simulated response. See plant.py for why
   an open-loop replay cannot do this.
3. The disturbance replayed into the loop is the MEASURED residual - the
   part of the rate the model could not explain, which is this airframe's
   real vibration and gust noise. A noiseless simulation makes kd look
   free and will happily recommend a differential gain that screams.

WHAT IT STILL CANNOT SEE
------------------------
The identified model is linear, single-axis and valid only for the gyro
filter settings that flew (the logged gyro is post-LPF and post-notch).
It does not know about cross-axis coupling, prop damage, or thrust
nonlinearity with collective. Treat the ranking as "which candidates are
worth a flight", never as a number to type in and go fly.
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "pid_replay"))
sys.path.insert(0, HERE)

from pidcore import (PidState, pid_init, pid_step,        # noqa: E402
                     rate_cfg, RATE_OUT_MAX, RATE_TAU)
from plant import identify, margins, simulate             # noqa: E402
from rate_replay import (DT_CLAMP_HI, DT_CLAMP_LO, NOMINAL_DT,  # noqa: E402
                         T_WRAP, floor_pinned, inverse_mix)
from rc_trim_check import FLY_THR_US, read_params, scale_of    # noqa: E402

AXES = {"roll": ("gyro_roll", "setp_roll", 1),
        "pitch": ("gyro_pitch", "setp_pitch", 2),
        "yaw": ("gyro_yaw", "setp_yaw", 3)}

MAX_US = 2000.0          # mixer.c MAX_THROTTLE

# Below this ratio of commanded to measured rate, the axis was mostly
# just reacting and the closed-loop fit is not identifiable - see
# excitation(). Measured on the flights: roll 1.05, pitch
# 1.13, yaw 0.44, and yaw is exactly the axis whose fit came out with a
# negative control effectiveness. 0.7 separates them with room to spare.
EXCITATION_MIN = 0.7

# Acceptance thresholds for the sweep. Classical margins - a discrete
# loop with less than this has no room for the model error that is
# certainly present, and the first thing that eats it is the prop and
# battery state changing between flights.
MIN_GAIN_MARGIN_DB = 6.0
MIN_PHASE_MARGIN_DEG = 30.0


def load(path, axis):
    """One session -> dict of arrays over the VALID airborne frames.

    A frame is dropped when the inverse mix cannot be trusted:
      - the mixer's final clamp bit, which is detectable from the log
        alone because the clamp leaves the de-scaled span sitting exactly
        at (2000 - idle); shifts preserve the span, only the clamp caps it
      - the per-motor scale was pinned at the idle floor (see
        rate_replay.floor_pinned)
    `link` marks where frame k+1 really is the next cycle, so nothing
    downstream differences across a hole.
    """
    gyro_col, setp_col, _ = AXES[axis]
    params = read_params(path)
    scale = scale_of(params)
    idle = params.get("min_idle_us", 1050.0)
    span_limit = (MAX_US - idle) - 0.5

    w, u, sp, dts, keep_idx = [], [], [], [], []
    prev_t = None
    with open(path, newline="") as fh:
        for i, row in enumerate(csv.DictReader(fh)):
            t = float(row["t_us"])
            dt = NOMINAL_DT
            if prev_t is not None:
                d = t - prev_t
                if d < 0:
                    d += T_WRAP
                dt = d * 1e-6
            prev_t = t
            dt = min(max(dt, DT_CLAMP_LO), DT_CLAMP_HI)

            if float(row["throttle"]) <= FLY_THR_US:
                continue
            motors = [int(row[f"m{j}"]) for j in (1, 2, 3, 4)]
            if floor_pinned(motors, scale, idle):
                continue
            raw = [1000.0 + (motors[j] - 1000.0) / scale[j] for j in range(4)]
            if max(raw) - min(raw) >= span_limit:
                continue                      # the final clamp bit
            _, u_r, u_p, u_y = inverse_mix(motors, scale)
            u.append({"roll": u_r, "pitch": u_p, "yaw": u_y}[axis])
            w.append(float(row[gyro_col]))
            sp.append(float(row[setp_col]))
            dts.append(dt)
            keep_idx.append(i)

    if len(w) < 2:
        return None
    keep_idx = np.asarray(keep_idx)
    link = np.zeros(len(w), dtype=bool)
    link[:-1] = np.diff(keep_idx) == 1
    return {"w": np.asarray(w), "u": np.asarray(u), "setp": np.asarray(sp),
            "dt": np.asarray(dts), "link": link, "params": params,
            "path": path}


def load_many(paths, axis):
    """Concatenate sessions, keeping `link` False across the joins so no
    fit or simulation ever steps from the end of one flight into the
    start of another."""
    out = None
    for p in paths:
        d = load(p, axis)
        if d is None:
            continue
        if out is None:
            out = {k: d[k] for k in ("w", "u", "setp", "dt", "link")}
            out["params"] = d["params"]
            out["sessions"] = [d["path"]]
            continue
        for k in ("w", "u", "setp", "dt", "link"):
            out[k] = np.concatenate([out[k], d[k]])
        out["sessions"].append(d["path"])
    if out is not None:
        out["link"][-1] = False
    return out


def dedupe(paths):
    """The ring buffer re-pulls one flight into several .bb files, so the
    same s<N> decodes under several stems. Fitting on both would just
    weight that flight twice."""
    seen, uniq = set(), []
    for pat in paths:
        for p in sorted(glob.glob(pat)) or [pat]:
            sid = os.path.basename(p).rsplit("_s", 1)[-1]
            if sid not in seen:
                seen.add(sid)
                uniq.append(p)
    return uniq


def excitation(data):
    """How much of the axis's motion was COMMANDED rather than reactive.

    Closed-loop identification only works when something outside the loop
    moves it. With the sticks centred, u is very nearly -kp*w by
    construction, u and w are collinear, and a fit cannot tell the plant
    (w responds to u) from the controller (u responds to w) - it will
    happily return the controller's inverse, complete with a negative
    control effectiveness. The ratio below is the warning sign; a
    deliberate stick sweep on the axis is the cure.
    """
    setp = float(np.sqrt(np.mean(data["setp"] ** 2)))
    gyro = float(np.sqrt(np.mean(data["w"] ** 2)))
    return {"setp": setp, "gyro": gyro,
            "ratio": setp / gyro if gyro > 0 else float("nan")}


def gains_of(params, axis):
    if axis == "yaw":
        return (params.get("yaw_kp", 3.0), params.get("yaw_ki", 0.1),
                params.get("yaw_kd", 0.0))
    return (params.get("rate_kp", 1.2), params.get("rate_ki", 0.05),
            params.get("rate_kd", 0.03))


def cmd_sysid(args):
    paths = dedupe(args.csv)
    for axis in args.axes:
        data = load_many(paths, axis)
        if data is None:
            print(f"{axis}: no valid airborne frames")
            continue
        model, resid = identify(data["w"], data["u"], data["dt"],
                                data["link"], axis=axis)
        n_link = int(data["link"].sum())
        print(f"\n=== {axis} ===")
        print(f"  {len(data['w'])} valid frames from "
              f"{len(data['sessions'])} session(s), {n_link} usable "
              f"transitions ({n_link / 250.0:.1f} s)")
        print(f"  control effectiveness  b = {model.b:8.4f} dps/s per us")
        print(f"  damping                a = {model.a:8.3f} 1/s   "
              f"(open-loop tau {model.tau_body * 1000:.0f} ms)")
        print(f"  constant disturbance   c = {model.c:+8.2f} dps/s")
        print(f"  actuator lag       tau_m = {model.tau_m * 1000:8.1f} ms")
        print(f"  transport delay        n = {model.delay:8d} samples "
              f"({model.delay * model.dt * 1000:.1f} ms)")
        print(f"  implied steady trim      = {model.trim_us:+8.2f} us "
              "(must match the measured mean command)")
        print("  free-run VAF by horizon  = " + "  ".join(
            f"{h * model.dt * 1000:.0f}ms {v:.1f}%"
            for h, v in sorted(model.vaf.items())))
        exc = excitation(data)
        print(f"  exogenous excitation     = {exc['ratio']:8.2f} "
              f"(setp RMS {exc['setp']:.1f} dps vs gyro RMS "
              f"{exc['gyro']:.1f} dps)")
        grade, why = model.verdict()
        if model.b <= 0 and exc["ratio"] < EXCITATION_MIN:
            why = ("the pilot barely commanded this axis, so the only "
                   "strong relationship in the data is the FEEDBACK path "
                   f"u = -kp*w. With setp RMS only {exc['ratio']:.0%} of "
                   "gyro RMS the fit recovers the controller's inverse "
                   "instead of the plant, and reports b < 0. Fly a "
                   "deliberate stick sweep on this axis to fix it")
        print(f"  -> {grade}: {why}")
        if args.save:
            with open(args.save, "a") as fh:
                fh.write(json.dumps(model.as_dict()) + "\n")
            print(f"  appended to {args.save}")


def score(model, setp, resid, dt, kp, ki, kd, tau=RATE_TAU):
    """Simulate one candidate; -> dict of metrics."""
    cfg = rate_cfg(kp, ki, kd)
    w, u = simulate(model, setp, resid, dt, cfg, pid_step, PidState,
                    pid_init, out_max=RATE_OUT_MAX)
    err = setp - w
    track = float(np.sqrt(np.mean(err ** 2)))
    # Command slew, us per second: what the ESCs are actually asked to
    # chase. This is where excess kd shows up as heat and noise long
    # before it shows up as instability.
    slew = float(np.sqrt(np.mean((np.diff(u) / dt[:-1]) ** 2)))
    railed = float(np.mean(np.abs(u) >= RATE_OUT_MAX - 0.5))
    gm, pm, wc = margins(model, kp, ki, kd, tau=tau)
    ok = (np.isfinite(gm) and gm >= MIN_GAIN_MARGIN_DB
          and np.isfinite(pm) and pm >= MIN_PHASE_MARGIN_DEG
          and np.all(np.isfinite(w)) and float(np.max(np.abs(w))) < 4999.0)
    return {"kp": kp, "ki": ki, "kd": kd, "track": track, "slew": slew,
            "railed": railed, "gm": gm, "pm": pm, "wc": wc, "ok": bool(ok)}


def frange(spec):
    """'lo:hi:step' or a comma list -> list of floats."""
    if ":" in spec:
        lo, hi, step = (float(x) for x in spec.split(":"))
        n = int(round((hi - lo) / step)) + 1
        return [round(lo + i * step, 6) for i in range(n)]
    return [float(x) for x in spec.split(",")]


def cmd_sweep(args):
    paths = dedupe(args.csv)
    for axis in args.axes:
        data = load_many(paths, axis)
        if data is None:
            print(f"{axis}: no valid airborne frames")
            continue
        model, resid = identify(data["w"], data["u"], data["dt"],
                                data["link"], axis=axis)
        print(f"\n=== {axis} === plant: {model!r}")
        grade, why = model.verdict()
        if grade == "REJECTED":
            print(f"  REJECTED: {why}.")
            print("  Refusing to rank gains against a model that does not "
                  "describe this axis - a ranking would still print, and "
                  "it would still be wrong.")
            continue
        if grade == "MARGINAL":
            print(f"  MARGINAL: {why}")

        # The reference the sweep chases is the flight's own rate
        # setpoint, carried through the same valid-frame filter as the fit.
        setp = data["setp"]
        dt = data["dt"]
        n = min(len(setp), len(dt), len(resid), len(data["w"]))
        setp, dt, res = setp[:n], dt[:n], resid[:n]

        flown = gains_of(data["params"], axis)
        rows = [score(model, setp, res, dt, *flown)]
        rows[0]["flown"] = True
        for kp in frange(args.kp):
            for ki in frange(args.ki):
                for kd in frange(args.kd):
                    if (kp, ki, kd) == flown:
                        continue
                    rows.append(score(model, setp, res, dt, kp, ki, kd))

        base = rows[0]

        # The sweep's own gate. The simulation at the FLOWN gains has to
        # land near the tracking error the aircraft actually had; if it
        # does not, every "-18 %" below is a percentage of the wrong
        # number. Not exact by construction - the sim replays the
        # setpoint but re-derives the rate - so this is a calibration,
        # not a pass/fail on the plumbing (rate_replay verify is that).
        measured = float(np.sqrt(np.mean((setp - data["w"][:n]) ** 2)))
        off = 100.0 * (base["track"] / measured - 1.0)
        print(f"  simulated at the flown gains: {base['track']:.2f} dps vs "
              f"{measured:.2f} dps actually measured ({off:+.0f}%)")
        if abs(off) > 30.0:
            print("  WARNING: the simulation does not reproduce the flight's "
                  "own tracking error. Read the ranking as ordinal at best.")

        rows_ok = [r for r in rows if r["ok"]]
        rows_ok.sort(key=lambda r: r["track"])
        print(f"  {len(rows)} candidates, {len(rows_ok)} with GM >= "
              f"{MIN_GAIN_MARGIN_DB:g} dB and PM >= "
              f"{MIN_PHASE_MARGIN_DEG:g} deg")
        print("  Ranked by tracking error ALONE. Read `slew` next to it: "
              "that is what the ESCs have to chase, and it is where extra "
              "kd is paid for.")
        print(f"\n  {'kp':>6}{'ki':>7}{'kd':>7}{'track RMS':>12}"
              f"{'vs flown':>10}{'slew':>11}{'vs':>6}{'railed':>9}"
              f"{'GM dB':>8}{'PM deg':>8}{'fc Hz':>8}")

        def show(r, tag=""):
            rel = 100.0 * (r["track"] / base["track"] - 1.0)
            print(f"  {r['kp']:>6.2f}{r['ki']:>7.3f}{r['kd']:>7.3f}"
                  f"{r['track']:>10.2f} dps{rel:>+9.1f}%"
                  f"{r['slew']:>9.0f} us/s"
                  f"{r['slew'] / base['slew']:>5.1f}x{100 * r['railed']:>8.1f}%"
                  f"{r['gm']:>8.1f}{r['pm']:>8.1f}{r['wc']:>8.1f}  {tag}")

        show(base, "<- FLOWN" + ("" if base["ok"] else "  (below margin!)"))
        for r in rows_ok[:args.top]:
            show(r)
        if not rows_ok:
            print("  nothing in the grid met the margins - widen it or "
                  "accept that this airframe is the limit.")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("csv", nargs="+", help="session CSV(s), globs allowed")
        p.add_argument("--axis", dest="axes", action="append",
                       choices=list(AXES),
                       help="repeatable; default all three")

    p = sub.add_parser("sysid", help="fit the plant and report fit quality")
    common(p)
    p.add_argument("--save", help="append the fitted models to this "
                                  "JSON-lines file")
    p.set_defaults(func=cmd_sysid)

    p = sub.add_parser("sweep", help="rank candidate gains")
    common(p)
    p.add_argument("--kp", default="0.6:3.0:0.2")
    p.add_argument("--ki", default="0,0.02,0.05,0.1,0.2")
    p.add_argument("--kd", default="0,0.01,0.02,0.03,0.05,0.08")
    p.add_argument("--top", type=int, default=12)
    p.set_defaults(func=cmd_sweep)

    args = ap.parse_args()
    if not args.axes:
        args.axes = list(AXES)
    args.func(args)


if __name__ == "__main__":
    main()
