#!/usr/bin/env python3
"""Analyse a strapped-down throttle-ladder run.

    python tools/ladder.py <session>.csv

THE RUN THIS EXPECTS: aircraft STRAPPED DOWN, props on,
throttle stepped 1050 -> 1550 in 50 us increments, ~10 s held at each step.
Targets: |accz mean| < 0.5 m/s^2 and sd < 2.5 at every step.

WHY IT EXISTS. Vertical acceleration reads -3.06 m/s^2 above roughly
1400 us against +0.20 at rest (incident 19), and that one number is what blocks the
P7 acceptance hover. A ladder is the controlled version of the flight that
found it: throttle is the only thing changing, it is held long enough to
average, and nothing is manoeuvring. That makes it the one run that can
separate the candidates.

The steps are found automatically from the throttle column - no need to
note timestamps while flying it. Frames during a transition, and the first
seconds of each plateau, are discarded so the numbers describe a settled
aircraft rather than a spin-up.

ONE RUN, THREE OPEN QUESTIONS. Because it is a clean throttle sweep it also
answers, in the same log:
  * does the DPS310 die above ~1400 us, and does the MS5611 beside it
    survive (if fitted)
  * does the magnetometer's gating track throttle - the mid-range peak
    seen in incident 28
  * where the vibration actually is

WHAT THIS CANNOT SEE. The accelerometer RAIL COUNTER is not in the
blackbox; it lives in the `STATS` console command (debug_stream.c). Run
STATS before and after the ladder and read `clipped ... %` - that is the
honest discriminator for the clipping hypothesis and this script cannot
substitute for it.

Blackbox gyro is post-LPF, post-notch and folded about 125 Hz, so
the per-step gyro spread here is a RELATIVE vibration comparison between
steps and never a spectrum. Use FFT_LOG=1 for real spectra.
"""

import csv
import glob
import math
import sys

IMU_OK, BARO_OK, MAG_OK, MAG_GATED, BARO2_OK = 1, 2, 4, 8, 16

SETTLE_S = 2.0        # discarded at the head of each plateau
MIN_STEP_S = 5.0      # a deliberate 10 s dwell, with room for a short one.
                      # 3 s was too loose: a steady HOVER passed as a
                      # plateau and flight data was analysed as a ladder.
THR_TOL = 8           # us of wobble still counted as the same step

ACCZ_MEAN_LIMIT = 0.5
ACCZ_SD_LIMIT = 2.5


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def sd(xs):
    if len(xs) < 2:
        return float("nan")
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def find_steps(rows, rate_hz):
    """Contiguous runs where throttle holds within THR_TOL."""
    steps, start = [], 0
    for i in range(1, len(rows) + 1):
        if i < len(rows):
            lo = min(int(r["throttle"]) for r in rows[start:i + 1])
            hi = max(int(r["throttle"]) for r in rows[start:i + 1])
            if hi - lo <= THR_TOL:
                continue
        seg = rows[start:i]
        if len(seg) / rate_hz >= MIN_STEP_S:
            steps.append(seg)
        start = i
    return steps


def looks_like_ladder(table):
    """Is this actually a ladder, or just a flight with some steady bits?

    Getting this wrong is worse than useless: the script happily produced a
    per-step table for a 50 s hover and then announced the baro failure
    "reproduces on the bench, with no flying involved". It was flight data.
    A ladder is deliberate and has a shape - several steps, climbing, evenly
    spaced, each held. Demand that shape before drawing ladder conclusions."""
    if len(table) < 5:
        return False, f"only {len(table)} settled step(s); a ladder has ~11"
    thrs = [r["thr"] for r in table]
    if any(b <= a for a, b in zip(thrs, thrs[1:])):
        return False, "throttle does not climb monotonically"
    if thrs[-1] - thrs[0] < 300:
        return False, (f"only {thrs[-1] - thrs[0]} us of range; "
                       f"the ladder spans 1050-1550")
    gaps = [b - a for a, b in zip(thrs, thrs[1:])]
    if max(gaps) > 3 * min(gaps):
        return False, "steps are unevenly spaced"
    dwells = sorted(r["dwell"] for r in table)
    med = dwells[len(dwells) // 2]
    if med < 7.0:
        return False, f"median dwell only {med:.1f} s; the ladder holds 10 s"
    return True, ""


def main(argv):
    paths = []
    for a in argv[1:]:
        paths.extend(sorted(glob.glob(a)))
    if not paths:
        print(__doc__)
        return 1

    for path in paths:
        with open(path, newline="") as fh:
            rows = list(csv.DictReader(fh))
        if len(rows) < 100:
            print(f"\n{path}: only {len(rows)} frames, nothing to analyse")
            continue

        span_s = (int(rows[-1]["t_us"]) - int(rows[0]["t_us"])) / 1e6
        rate = len(rows) / span_s if span_s > 0 else 250.0

        steps = find_steps(rows, rate)
        thrs = sorted({int(r["throttle"]) for r in rows})

        print(f"\n=== {path} ===")
        print(f"{len(rows)} frames, {span_s:.1f} s, {rate:.0f} Hz, "
              f"throttle {thrs[0]}-{thrs[-1]} us")

        if not steps:
            print(f"\n  No throttle plateau held for {MIN_STEP_S:.0f} s.")
            print("  Not a ladder run. Strapped down, 1050 -> 1550 in 50 us")
            print("  steps, 10 s each.")
            continue

        print(f"\n  {len(steps)} plateau(s) of >= {MIN_STEP_S:.0f} s. First "
              f"{SETTLE_S:.0f} s of each discarded as settling.\n")
        hdr = (f"  {'thr':>5} {'dwell':>6} {'n':>6} | {'accz mean':>10} "
               f"{'sd':>6} {'min':>7} {'max':>7} | {'gyro sd':>8} | "
               f"{'IMU':>5} {'BARO':>5} {'MAG':>5} {'9DOF':>5}")
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))

        table = []
        any_baro2 = "baro2_m" in rows[0]
        for seg in steps:
            drop = int(SETTLE_S * rate)
            body = seg[drop:] if len(seg) > drop + int(rate) else seg
            if not body:
                continue
            thr = int(round(mean([int(r["throttle"]) for r in body])))
            az = [float(r["accz_ms2"]) for r in body]
            gy = [sd([float(r[c]) for r in body])
                  for c in ("gyro_roll", "gyro_pitch", "gyro_yaw")]
            h = [int(r["sensor_health"]) for r in body]
            n = len(body)
            pc = lambda bit: 100.0 * sum(1 for x in h if x & bit) / n
            row = dict(thr=thr, dwell=len(seg) / rate, n=n,
                       mean=mean(az), sd=sd(az), lo=min(az), hi=max(az),
                       gyro=mean(gy), imu=pc(IMU_OK), baro=pc(BARO_OK),
                       mag=pc(MAG_OK), dof=pc(MAG_GATED),
                       baro2=pc(BARO2_OK) if any_baro2 else None)
            table.append(row)

            flag = ""
            if abs(row["mean"]) > ACCZ_MEAN_LIMIT:
                flag += " MEAN"
            if row["sd"] > ACCZ_SD_LIMIT:
                flag += " SD"
            print(f"  {thr:5d} {row['dwell']:5.1f}s {n:6d} | "
                  f"{row['mean']:+10.2f} {row['sd']:6.2f} {row['lo']:+7.1f} "
                  f"{row['hi']:+7.1f} | {row['gyro']:8.2f} | "
                  f"{row['imu']:4.0f}% {row['baro']:4.0f}% {row['mag']:4.0f}% "
                  f"{row['dof']:4.0f}%{flag}")

        baro2_fitted = any_baro2 and any(r["baro2"] > 0 for r in table)
        if any_baro2 and not baro2_fitted:
            print("\n  second barometer (MS5611): not fitted in this run "
                  "(bit 4 never set) - expected until the part is installed.")
        elif baro2_fitted:
            print("\n  second barometer (MS5611) health per step:")
            print("     " + "  ".join(f"{r['thr']}:{r['baro2']:.0f}%"
                                      for r in table))

        is_ladder, why = looks_like_ladder(table)
        if not is_ladder:
            print(f"\n  *** NOT A LADDER RUN: {why}. ***")
            print("  The table above is still valid as a description of what")
            print("  the aircraft did at those throttles, and the accz and")
            print("  health columns are real. But it is NOT a controlled")
            print("  sweep: throttle was not the only thing changing, so")
            print("  nothing here separates throttle from manoeuvring, and")
            print("  the ladder verdicts below are withheld.")
            print("\n  For the real thing: strapped down, props on,")
            print("  1050 -> 1550 in 50 us steps, 10 s held at each.")
            continue

        # ---- accz: is the bias throttle-dependent, and where is the knee?
        print("\n  === accz ===")
        bad = [r for r in table if abs(r["mean"]) > ACCZ_MEAN_LIMIT]
        noisy = [r for r in table if r["sd"] > ACCZ_SD_LIMIT]
        if not bad and not noisy:
            print(f"  Every step is inside target (|mean| < "
                  f"{ACCZ_MEAN_LIMIT}, sd < {ACCZ_SD_LIMIT}).")
            print("  The -3.06 m/s^2 does NOT reproduce. Before calling it")
            print("  fixed, name what changed: the +-8 g range, the airframe")
            print("  being tightened, or the pack being strapped. Tightening")
            print("  counts as a change (incident 26).")
        else:
            lo_t = [r for r in table if r["thr"] < 1400]
            hi_t = [r for r in table if r["thr"] >= 1400]
            if lo_t and hi_t:
                ml, mh = mean([r["mean"] for r in lo_t]), mean([r["mean"] for r in hi_t])
                print(f"  below 1400 us: mean {ml:+.2f} m/s^2   "
                      f"at/above 1400: {mh:+.2f}   step {mh - ml:+.2f}")
            first_bad = min((r["thr"] for r in bad), default=None)
            if first_bad:
                print(f"  first step outside the mean target: {first_bad} us")
            if noisy:
                print(f"  {len(noisy)} step(s) exceed sd {ACCZ_SD_LIMIT}: "
                      + ", ".join(str(r['thr']) for r in noisy))
                print("  Expected after the +-8 g change - half the counts per")
                print("  m/s^2 is half the resolution. If the MEAN is now")
                print("  clean and only the sd grew, raise `alt_acc_sigma`")
                print("  rather than treating it as a fault.")
            print("\n  Read `STATS` for the rail counter. It is not in this")
            print("  log and it is the honest discriminator:")
            print("    clipped > 0    -> clipping was real; +-8 g addressed it")
            print("    clipped 0.0000 and the bias is still there")
            print("                   -> clipping is dead. Move to g-sensitivity,")
            print("                      thermal, or a sustained projection error.")

        # ---- vibration: where is the resonance?
        print("\n  === vibration (relative only - post-filter, folded) ===")
        peak = max(table, key=lambda r: r["gyro"])
        base = min(table, key=lambda r: r["gyro"])
        print(f"  quietest step {base['thr']} us (gyro sd {base['gyro']:.2f}), "
              f"worst {peak['thr']} us ({peak['gyro']:.2f})")
        if peak["gyro"] > 3 * base["gyro"] and base["gyro"] > 0:
            span = table[-1]["thr"] - table[0]["thr"]
            frac = (peak["thr"] - table[0]["thr"]) / span if span else 0.0
            if 0.2 < frac < 0.8:
                print(f"  A {peak['gyro']/base['gyro']:.1f}x peak in the MIDDLE "
                      f"of the range is a structural resonance at that RPM.")
                print("  Worth comparing against the mag gating below - incident 28")
                print("  found mag rejection peaking at 1200-1400 us, and a")
                print("  resonance is one thing that would explain both.")
            else:
                print("  The worst step is at an end of the range, so this is")
                print("  a trend rather than a resonance.")
        else:
            print("  No strong peak - vibration is roughly flat across the")
            print("  range, which argues against a structural resonance.")

        # ---- the other two open questions, free from the same run
        print("\n  === baro and mag vs throttle ===")
        b_lo = [r["baro"] for r in table if r["thr"] < 1400]
        b_hi = [r["baro"] for r in table if r["thr"] >= 1400]
        if b_lo and b_hi:
            print(f"  DPS310 health: {mean(b_lo):.0f} % below 1400 us, "
                  f"{mean(b_hi):.0f} % at/above")
            if mean(b_lo) > 90 and mean(b_hi) < 50:
                print("  -> the 27 Aug failure REPRODUCES in a strapped-down")
                print("     sweep. That is the good outcome: it becomes a")
                print("     repeatable bench test instead of a flight anecdote,")
                print("     and the MS5611 column above says whether it is the")
                print("     part or the environment.")
            elif mean(b_hi) > 90:
                print("  -> it does NOT reproduce. Say what changed before")
                print("     calling it fixed - re-seating bus A counts (incident 28).")

            # With a control fitted, the experiment resolves here rather
            # than needing a separate pass through baro_compare.py.
            if baro2_fitted:
                m2_lo = mean([r["baro2"] for r in table if r["thr"] < 1400])
                m2_hi = mean([r["baro2"] for r in table if r["thr"] >= 1400])
                print(f"\n  MS5611 health: {m2_lo:.0f} % below 1400 us, "
                      f"{m2_hi:.0f} % at/above")
                if mean(b_hi) < 50 and m2_hi > 75:
                    print("  *** VERDICT: the MS5611 SURVIVES where the DPS310")
                    print("      dies, in the same run, on the same bus, at the")
                    print("      same throttles. It is the DPS310. Swap it.")
                elif mean(b_hi) < 50 and m2_hi < 50:
                    print("  *** VERDICT: BOTH barometers die above 1400 us.")
                    print("      A shared cause. The mag on the same rail and")
                    print("      bus is unaffected, so this is VIBRATION, not")
                    print("      supply - and the vibration section above says")
                    print("      at which throttle. Soft-mount before")
                    print("      replacing anything.")
                elif mean(b_hi) > 90 and m2_hi > 90:
                    print("  *** VERDICT: both fine. The fault did not")
                    print("      reproduce; name what changed before closing.")
                else:
                    print("  *** Mixed - read the per-step tables rather than")
                    print("      trusting a one-line summary.")
        gated = [(r["thr"], r["mag"] - r["dof"]) for r in table]
        worst = max(gated, key=lambda x: x[1])
        print(f"  mag gated most at {worst[0]} us ({worst[1]:.0f} points "
              f"between 'replied' and 'believed')")
        print("  incident 28 predicts a mid-range peak, not a rise with throttle.")
        print("  If it rises monotonically here instead, incident 28 is wrong and it")
        print("  really is motor current.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
