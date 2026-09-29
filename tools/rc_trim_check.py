#!/usr/bin/env python3
"""Recover what the PILOT actually commanded, from a decoded blackbox CSV.

    python tools/rc_trim_check.py s121.csv
    python tools/rc_trim_check.py 's*.csv' --summary

WHY THIS EXISTS
---------------
The blackbox does not log the roll/pitch sticks - only throttle (`.throttle
= packet.channels[CH_THROTTLE]` in fc_tasks.c). So "was the author holding a
trim?" looks unanswerable from a log, and the `setp_roll` / `setp_pitch`
columns get read as if they were the stick. THEY ARE NOT. What is logged is

    setp = target_rate = angle_kp * (des - att)          [fc_tasks.c]

the OUTPUT of the outer angle loop, which mixes the pilot's command with
the aircraft's own attitude error. A negative `setp_pitch` mean therefore
proves nothing about the pilot on its own: it is equally well an aircraft
sitting nose-down with the stick centred.

But the angle loop is a pure gain (ki = 0, kd = 0, i_max = 0), and `att` is
logged next to `setp`. So the pilot's command inverts exactly:

    des = setp / angle_kp + att

VALIDATION - the command must land on the RC-count grid
-------------------------------------------------------
des can only ever be an integer number of RC counts:
`des = stick * angle_max_deg / 500`, stick integer. So des/(angle_max/500)
must come out near-integer for every airborne frame. That is a hard,
falsifiable check on the whole chain - angle_kp, the log's scaling, the
column order - and it is printed as `off-grid`. Pure noise gives 0.25-0.5;
a correct inversion gives < 0.15. If it does not land on the grid, STOP:
something upstream is wrong and every number below is fiction.

Frames where the angle loop clipped at out_max (200 dps) are dropped - the
setpoint is no longer angle_kp * error there, so it cannot be inverted.

SIGN CONVENTIONS (verified in source, do not re-derive)
------------------------------------------------------------------
fusion.c seeds `pitch = atan2(-ax, ...)`, and the accelerometer of an
x-forward / z-up chip reads ax = +g*sin(theta) when the nose is UP. So:

    +pitch attitude = nose DOWN        -pitch = nose UP
    +roll  attitude = rolled RIGHT     -roll  = rolled LEFT
    +yaw   rate     = nose LEFT

and mixer.c agrees: +pitch raises the REAR pair (nose down), +roll raises
the LEFT pair (rolls right), +yaw raises the CW pair (nose left). The loop
is negative feedback in every axis. This is the SAME convention for the
recovered `des` here, because des is compared against att directly.

Consequences for reading the output:

    des_pitch < 0   pilot asking NOSE UP    -> fighting a FORWARD drift
    des_pitch > 0   pilot asking NOSE DOWN  -> fighting a BACKWARD drift
    des_roll  < 0   pilot asking roll LEFT  -> fighting a RIGHT drift
    des_roll  > 0   pilot asking roll RIGHT -> fighting a LEFT drift

WHAT A SUSTAINED des MEANS, AND WHAT IT DOES NOT
------------------------------------------------
A steady non-zero des is a TRANSLATION complaint, not a torque complaint.
A torque imbalance - a nicked prop, a hot ESC, a per-motor scale - is
absorbed by the rate loop's integrator and shows up as a steady MOTOR
asymmetry (the `torque` block below), while the aircraft still holds the
attitude it was asked for. It takes an attitude REFERENCE error
(level_roll_deg / level_pitch_deg) or an outside force to make a level-
looking aircraft drift, and that is what the pilot answers with stick.

So the two blocks below answer two different questions and must not be
conflated: `stick` = where is level wrong, `torque` = which corner is weak.
"""

import argparse
import csv
import glob
import math
import os
import statistics as st
import sys

# fc_tasks.c: the ANGLE loops are built with these, and they are
# compile-time, not params - only .kp comes from the params table.
ANGLE_OUT_MAX_DPS = 200.0

# Defaults if no <stem>.params.txt sits next to the CSV. Both are params,
# so a log flown at other values will be read wrong - hence the warning.
DEFAULT_ANGLE_KP = 4.0
DEFAULT_ANGLE_MAX_DEG = 30.0

# Airborne gate. alt_hover_thr is ~420 (= 1420 us), so 1350 is under the
# hover point but well clear of the idle and spool-up bands.
FLY_THR_US = 1350
MIN_FLY_FRAMES = 250            # 1 s at 250 Hz; below that a mean is noise

MOTOR_NAMES = ("m1 FR", "m2 FL", "m3 RR", "m4 RL")


def read_params(csv_path):
    """<stem>.params.txt -> dict of floats, or {} if it isn't there."""
    p = csv_path
    for suffix in (".csv.gz", ".csv"):
        if p.endswith(suffix):
            p = p[: -len(suffix)]
            break
    p += ".params.txt"
    if not os.path.exists(p):
        return {}
    out = {}
    with open(p) as fh:
        for line in fh:
            if line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            try:
                out[k.strip()] = float(v)
            except ValueError:
                pass
    return out


def inverse_mix(m, scale):
    """Post-scale motor us -> (thrust, roll, pitch, yaw) command, us.

    Undoes the per-motor trim first (fc_tasks.c block 4b scales the span
    above 1000, so the inverse is symmetric), then inverts mixer_quad_x's
    table. Airmode's collective shift lands entirely in `thrust` and
    leaves the three differentials untouched, which is exactly why they
    are worth reading; the final per-motor CLAMP does corrupt them, but
    it only bites at extreme demand.
    """
    raw = [1000.0 + (m[i] - 1000.0) / scale[i] for i in range(4)]
    fr, fl, rr, rl = raw
    return ((fr + fl + rr + rl) / 4.0,
            (-fr + fl - rr + rl) / 4.0,
            (-fr - fl + rr + rl) / 4.0,
            (fr - fl - rr + rl) / 4.0)


def load(path, angle_kp, angle_max_deg, scale):
    """CSV -> list of per-frame dicts for the airborne frames only."""
    step = angle_max_deg / 500.0          # one RC count, in degrees
    opener = open
    if path.endswith(".gz"):
        import gzip
        opener = gzip.open
    frames = []
    n_total = n_clip = 0
    with opener(path, "rt", newline="") as fh:
        for r in csv.DictReader(fh):
            if int(r["throttle"]) <= FLY_THR_US:
                continue
            n_total += 1
            sr, sp = float(r["setp_roll"]), float(r["setp_pitch"])
            if abs(sr) >= ANGLE_OUT_MAX_DPS - 1.0 or \
               abs(sp) >= ANGLE_OUT_MAX_DPS - 1.0:
                n_clip += 1               # not invertible, see docstring
                continue
            att_r, att_p = float(r["roll"]), float(r["pitch"])
            m = [int(r["m1"]), int(r["m2"]), int(r["m3"]), int(r["m4"])]
            thr, u_r, u_p, u_y = inverse_mix(m, scale)
            frames.append({
                "des_r": sr / angle_kp + att_r,
                "des_p": sp / angle_kp + att_p,
                "att_r": att_r, "att_p": att_p,
                "g_r": float(r["gyro_roll"]), "g_p": float(r["gyro_pitch"]),
                "setp_r": sr, "setp_p": sp,
                "u_thr": thr, "u_roll": u_r, "u_pitch": u_p, "u_yaw": u_y,
                "m": m,
            })
    # Grid residual: |des/step - round(des/step)|, the falsifiable check.
    off = []
    for f in frames:
        for key in ("des_r", "des_p"):
            n = f[key] / step
            off.append(abs(n - round(n)))
    return frames, n_total, n_clip, (st.median(off) if off else float("nan"))


def stick_block(frames, key, step, neg_label, pos_label):
    """One axis of the pilot-command summary."""
    v = [f[key] for f in frames]
    held = [x for x in v if abs(x) > 0.5 * step]      # outside the deadband
    neg = [x for x in held if x < 0]
    pos = [x for x in held if x > 0]
    n = len(v)
    return {
        "mean": sum(v) / n,
        "centred_pct": 100.0 * (n - len(held)) / n,
        "neg_pct": 100.0 * len(neg) / n,
        "pos_pct": 100.0 * len(pos) / n,
        "neg_mean": (sum(neg) / len(neg)) if neg else 0.0,
        "pos_mean": (sum(pos) / len(pos)) if pos else 0.0,
        "neg_worst": min(neg) if neg else 0.0,
        "pos_worst": max(pos) if pos else 0.0,
        "neg_label": neg_label, "pos_label": pos_label,
    }


def report(path, frames, n_total, n_clip, off_grid, params, quiet=False):
    n = len(frames)
    angle_max = params.get("angle_max_deg", DEFAULT_ANGLE_MAX_DEG)
    step = angle_max / 500.0
    mean = lambda k: sum(f[k] for f in frames) / n
    rms = lambda k: math.sqrt(sum(f[k] ** 2 for f in frames) / n)

    roll = stick_block(frames, "des_r", step, "LEFT", "RIGHT")
    pitch = stick_block(frames, "des_p", step, "NOSE UP", "NOSE DOWN")

    if not quiet:
        print(f"\n=== {os.path.basename(path)} ===")
        print(f"  {n} airborne frames ({n / 250.0:.1f} s above {FLY_THR_US} us)"
              + (f", {n_clip} dropped as angle-loop clip" if n_clip else ""))
        grade = ("OK" if off_grid < 0.15 else
                 "SUSPECT" if off_grid < 0.25 else "FAILED")
        print(f"  off-grid residual {off_grid:.3f}  [{grade}]"
              "   (< 0.15 = the inversion is sound; ~0.25+ = do not trust "
              "anything below)")

        print("\n  stick  - what the PILOT asked for (deg of commanded tilt)")
        for name, b in (("roll ", roll), ("pitch", pitch)):
            print(f"    {name}  mean {b['mean']:+6.2f} deg   "
                  f"centred {b['centred_pct']:5.1f}%   "
                  f"{b['neg_label']:>8} {b['neg_pct']:5.1f}% "
                  f"(avg {b['neg_mean']:+6.2f}, peak {b['neg_worst']:+7.2f})   "
                  f"{b['pos_label']:>10} {b['pos_pct']:5.1f}% "
                  f"(avg {b['pos_mean']:+6.2f}, peak {b['pos_worst']:+7.2f})")

        print("\n  attitude - what the aircraft DID (deg), and rate error")
        print(f"    roll   mean {mean('att_r'):+6.2f}   "
              f"tracking err RMS {rms_err(frames, 'setp_r', 'g_r'):6.2f} dps")
        print(f"    pitch  mean {mean('att_p'):+6.2f}   "
              f"tracking err RMS {rms_err(frames, 'setp_p', 'g_p'):6.2f} dps")

        print("\n  torque - inverse-mixed motor command (us), scales undone")
        print(f"    collective {mean('u_thr'):7.1f}   "
              f"roll {mean('u_roll'):+6.2f} (+ = rolls right)   "
              f"pitch {mean('u_pitch'):+6.2f} (+ = nose down)   "
              f"yaw {mean('u_yaw'):+6.2f} (+ = nose left)")
        raw = [1000.0 + (sum(f["m"][i] for f in frames) / n - 1000.0)
               / scale_of(params)[i] for i in range(4)]
        print("    de-scaled motor means: "
              + "  ".join(f"{MOTOR_NAMES[i]} {raw[i]:6.1f}" for i in range(4)))
    return {"path": path, "n": n, "off_grid": off_grid,
            "roll": roll, "pitch": pitch,
            "u_roll": mean("u_roll"), "u_pitch": mean("u_pitch"),
            "u_yaw": mean("u_yaw"),
            "err_r": rms_err(frames, "setp_r", "g_r"),
            "err_p": rms_err(frames, "setp_p", "g_p")}


def rms_err(frames, setp_key, gyro_key):
    n = len(frames)
    return math.sqrt(sum((f[setp_key] - f[gyro_key]) ** 2
                         for f in frames) / n)


def scale_of(params):
    return [params.get(f"m{i + 1}_scale", 1.0) or 1.0 for i in range(4)]


def wls(xs, ys, ws):
    """Frame-count-weighted least squares -> (slope, intercept, r)."""
    sw = sum(ws)
    mx = sum(w * x for w, x in zip(ws, xs)) / sw
    my = sum(w * y for w, y in zip(ws, ys)) / sw
    sxy = sum(w * (x - mx) * (y - my) for w, x, y in zip(ws, xs, ys))
    sxx = sum(w * (x - mx) ** 2 for w, x in zip(ws, xs))
    syy = sum(w * (y - my) ** 2 for w, y in zip(ws, ys))
    if not sxx or not syy:
        return float("nan"), my, float("nan")
    slope = sxy / sxx
    return slope, my - slope * mx, sxy / math.sqrt(sxx * syy)


def vs_level(rows):
    """Is the pilot's steady command just cancelling the level trim?

    If level_<axis>_deg were CORRECT the aircraft would fly straight with
    the stick centred, so the pilot's mean command would sit at zero
    whatever the trim is. A slope of -1 says the opposite: every degree
    put into the trim comes straight back out of the pilot's thumb, so
    the trim is not correcting a mounting offset, it is CREATING one.
    The intercept is then the drift that is genuinely left over, and the
    best trim value for that axis is the x where the fit crosses zero.

    Weighted by frame count, and useless without at least two distinct
    trim values - grouping is printed so leverage from a single outlying
    value is visible rather than hidden inside r.
    """
    rows = [r for r in rows if r["lev_r"] is not None]
    if not rows:
        sys.exit("no .params.txt had level_*_deg - nothing to regress")

    print(f"\n{'session':<28}{'n':>6}{'levR':>9}{'levP':>9}"
          f"{'desR':>8}{'desP':>8}{'uRoll':>8}{'uPitch':>8}")
    for r in sorted(rows, key=lambda r: r["lev_r"]):
        print(f"{os.path.basename(r['path']):<28}{r['n']:>6}"
              f"{r['lev_r']:>9.3f}{r['lev_p']:>9.3f}"
              f"{r['roll']['mean']:>8.2f}{r['pitch']['mean']:>8.2f}"
              f"{r['u_roll']:>8.2f}{r['u_pitch']:>8.2f}")

    w = [r["n"] for r in rows]
    for axis, lev_key, cmd_key in (("roll", "lev_r", "roll"),
                                   ("pitch", "lev_p", "pitch")):
        xs = [r[lev_key] for r in rows]
        ys = [r[cmd_key]["mean"] for r in rows]
        distinct = sorted(set(round(x, 4) for x in xs))
        s, b, r = wls(xs, ys, w)
        print(f"\n  {axis}: des = {s:+.2f} * level {b:+.2f}   r = {r:+.3f}   "
              f"over {len(distinct)} distinct trim value(s)")
        if len(distinct) < 2:
            print("    -> only one trim value flown; says nothing yet.")
            continue
        # Leverage warning: with a handful of distinct values, one far-out
        # group can set the slope on its own. Name it instead of hiding it.
        spread = distinct[-1] - distinct[0]
        biggest = max(distinct, key=lambda v: abs(v - sum(distinct)
                                                  / len(distinct)))
        # 4 dp, not 3: a hand-typed -0.261 and a LEVELCAL-produced
        # -0.260822 are genuinely different flights, and printing both as
        # "-0.261" makes the count look like a bug.
        print(f"    trim values flown: "
              + ", ".join(f"{v:+.4f}" for v in distinct)
              + f"  (span {spread:.2f} deg, {biggest:+.3f} carries the "
                "leverage)")
        if s < -0.6:
            best = -b / s if s else 0.0
            print(f"    -> slope near -1: the pilot cancels this trim "
                  f"1:1. Zero-command trim would be {best:+.3f} deg.")
        elif abs(s) < 0.3:
            print(f"    -> no relationship. The {b:+.2f} deg residual "
                  "command is NOT this trim; look elsewhere.")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("csv", nargs="+", help="session CSV(s), globs allowed")
    ap.add_argument("--summary", action="store_true",
                    help="one line per session plus a pooled verdict")
    ap.add_argument("--vs-level", action="store_true",
                    help="regress the pilot's command against level_*_deg "
                         "across the given sessions (needs >= 2 distinct "
                         "trim values to say anything)")
    ap.add_argument("--angle-kp", type=float, default=None,
                    help="override angle_kp (default: from .params.txt)")
    args = ap.parse_args()

    paths = []
    for pat in args.csv:
        hits = sorted(glob.glob(pat))
        paths.extend(hits if hits else [pat])
    # The ring buffer means one session is re-pulled into several .bb
    # files, so the same s<N> decodes to identical CSVs under different
    # stems. Dedupe on the session id or every mean gets weighted by how
    # many times that flight happened to be pulled.
    seen, uniq = set(), []
    for p in paths:
        sid = os.path.basename(p).rsplit("_s", 1)[-1]
        if sid in seen:
            continue
        seen.add(sid)
        uniq.append(p)
    if len(uniq) < len(paths):
        print(f"({len(paths) - len(uniq)} duplicate session re-pulls skipped)")

    rows = []
    for path in uniq:
        params = read_params(path)
        if not params:
            print(f"{path}: no .params.txt - assuming angle_kp="
                  f"{DEFAULT_ANGLE_KP}, angle_max_deg={DEFAULT_ANGLE_MAX_DEG}",
                  file=sys.stderr)
        kp = args.angle_kp or params.get("angle_kp", DEFAULT_ANGLE_KP)
        amax = params.get("angle_max_deg", DEFAULT_ANGLE_MAX_DEG)
        try:
            frames, n_total, n_clip, off = load(path, kp, amax,
                                                scale_of(params))
        except (OSError, KeyError) as e:
            print(f"{path}: {e}", file=sys.stderr)
            continue
        if len(frames) < MIN_FLY_FRAMES:
            continue
        row = report(path, frames, n_total, n_clip, off, params,
                     quiet=args.summary or args.vs_level)
        row["lev_r"] = params.get("level_roll_deg")
        row["lev_p"] = params.get("level_pitch_deg")
        rows.append(row)

    if not rows:
        sys.exit(f"no session had {MIN_FLY_FRAMES}+ frames above "
                 f"{FLY_THR_US} us - these are bench logs, not flights")

    if args.vs_level:
        vs_level(rows)
        return

    if args.summary:
        print(f"\n{'session':<28}{'n':>6}{'grid':>6}"
              f"{'desR':>7}{'desP':>7}{'uRoll':>7}{'uPitch':>7}{'uYaw':>7}"
              f"{'errR':>7}{'errP':>7}")
        for r in rows:
            print(f"{os.path.basename(r['path']):<28}{r['n']:>6}"
                  f"{r['off_grid']:>6.2f}"
                  f"{r['roll']['mean']:>7.2f}{r['pitch']['mean']:>7.2f}"
                  f"{r['u_roll']:>7.2f}{r['u_pitch']:>7.2f}{r['u_yaw']:>7.2f}"
                  f"{r['err_r']:>7.1f}{r['err_p']:>7.1f}")

    # Pooled verdict. A trim demand is only real if it is CONSISTENT: the
    # sign has to survive across independent flights, which is the one
    # thing a single session's mean can never show.
    print("\n--- verdict over "
          f"{len(rows)} independent flight session(s) ---")
    for axis, neg, pos in (("des_r", "roll LEFT (fighting a RIGHT drift)",
                            "roll RIGHT (fighting a LEFT drift)"),
                           ("des_p", "NOSE UP (fighting a FORWARD drift)",
                            "NOSE DOWN (fighting a BACKWARD drift)")):
        key = "roll" if axis == "des_r" else "pitch"
        means = [r[key]["mean"] for r in rows]
        n_neg = sum(1 for m in means if m < 0)
        pooled = sum(means) / len(means)
        label = neg if pooled < 0 else pos
        consistent = max(n_neg, len(means) - n_neg)
        print(f"  {key:<6} pooled mean {pooled:+.2f} deg, "
              f"same sign in {consistent}/{len(means)} sessions "
              f"-> pilot held {label}")
    print("  A consistent sign is an attitude-REFERENCE error "
          "(level_roll_deg / level_pitch_deg) or an outside force.")
    print("  It is NOT a weak motor: the rate integrator absorbs torque "
          "bias and it shows in `uRoll`/`uPitch` instead.")


if __name__ == "__main__":
    main()
