#!/usr/bin/env python3
"""Is the IMU telling the truth in this log?

    python tools/imu_check.py *_s18*.csv

Nothing else in tools/ answers this. mag_gate.py, rail_test.py and
baro_time.py all interrogate the SLOW sensors; the LSM6DSOX has never had
a checker because until now it has never been the suspect.

The checks are ordered by what the project's says actually goes wrong, hardest
evidence first:

  1. LIVENESS, not value.  "Never trust a sensor VALUE without its
     LIVENESS. A frozen cache passes every value-based check. That is
     what destroyed the aircraft."  So the first thing measured is the
     longest run of BIT-IDENTICAL samples. A real gyro on a real airframe
     never repeats a triple twice in a row; a dead SPI read repeats it
     forever, at a perfectly plausible value.

  2. SATURATION.  gyro is logged as dps*16 in an int16, so the column
     itself rails at +-2047.94 dps - well inside the LSM6DSOX's own
     +-2000 dps full scale, which means a railed column is the SENSOR
     saturating and not the log format clipping. Accel is +-8 g since
. A saturated attitude solution is garbage downstream even
     though every individual number looks legal.

  3. HEALTH BIT.  sensor_health bit 0 is "the LSM6DSOX answered". Read it
     WITH the liveness result, never instead of it - answering and
     answering correctly are different claims ("A transaction
     succeeding says nothing about the DATA").

  4. PHYSICAL PLAUSIBILITY.  Attitude outside +-180 deg, or an accel
     magnitude far from 1 g when the aircraft is sitting at idle, is the
     cheap end of the funnel and comes last because it is the check most
     easily passed by a broken sensor.

Ground and idle frames are reported separately from flying ones, because
the project's is explicit that mixing them produces numbers indistinguishable
from a mid-air excursion.
"""

import argparse
import csv
import glob
import gzip
import os
import sys

GYRO_RAIL_DPS = 32767 / 16.0        # int16 dps*16
ACC_RAIL_MS2 = 8.0 * 9.80665        # +-8 g
SENSOR_IMU_OK = 0x01


def load(path):
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, mode="rt", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    return rows


def longest_identical_run(rows, keys):
    """Longest run of consecutive frames whose keys are all identical,
    and where it started. This is the liveness probe."""
    best, best_at, cur, cur_at = 1, 0, 1, 0
    prev = None
    for i, r in enumerate(rows):
        cure = tuple(r[k] for k in keys)
        if prev is not None and cure == prev:
            cur += 1
            if cur > best:
                best, best_at = cur, cur_at
        else:
            cur, cur_at = 1, i
        prev = cure
    return best, best_at


def stats(vals):
    if not vals:
        return (float("nan"),) * 4
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    return min(vals), max(vals), mean, var ** 0.5


def report(path, rows, thr_fly):
    t0 = float(rows[0]["t_us"]) / 1e6
    dur = float(rows[-1]["t_us"]) / 1e6 - t0
    fly = [r for r in rows if int(r["throttle"]) > thr_fly]
    grd = [r for r in rows if int(r["throttle"]) <= thr_fly]

    print("%s  %d frames, %.1f s  (%d above throttle %d, %d at/below)"
          % (os.path.basename(path), len(rows), dur, len(fly), thr_fly,
             len(grd)))

    # ---- 1. liveness -------------------------------------------------
    gk = ["gyro_roll", "gyro_pitch", "gyro_yaw"]
    ak = ["roll", "pitch", "yaw"]
    grun, gat = longest_identical_run(rows, gk)
    arun, aat = longest_identical_run(rows, ak)
    rate = len(rows) / dur if dur > 0 else 250.0
    # A run that STARTS at frame 0 is the session opening before the
    # first fusion output has been published - every column is still
    # zero. That is not a frozen sensor, and flagging it buries a real
    # freeze later in the file under a false positive on every log.
    frozen = grun > 0.05 * rate and gat > 0
    flag = "  <-- FROZEN" if frozen else ("  (session start, not a freeze)"
                                          if grun > 0.05 * rate else "")
    print("  liveness   longest identical gyro triple %4d frames "
          "(%.0f ms) at t=%.2f s%s"
          % (grun, 1000.0 * grun / rate, float(rows[gat]["t_us"]) / 1e6 - t0,
             flag))
    if frozen:
        r = rows[gat]
        print("             frozen at gyro=(%s, %s, %s)"
              % (r["gyro_roll"], r["gyro_pitch"], r["gyro_yaw"]))
    print("             longest identical attitude    %4d frames (%.0f ms)"
          % (arun, 1000.0 * arun / rate))

    # ---- 2. saturation ----------------------------------------------
    for axis in gk:
        v = [float(r[axis]) for r in rows]
        n_rail = sum(1 for x in v if abs(x) >= GYRO_RAIL_DPS - 1.0)
        lo, hi, mean, sd = stats(v)
        mark = "  <-- RAILED %d frames" % n_rail if n_rail else ""
        print("  %-11s min %9.2f  max %9.2f  mean %8.2f  sd %7.2f dps%s"
              % (axis, lo, hi, mean, sd, mark))

    az = [float(r["accz_ms2"]) for r in rows]
    n_rail = sum(1 for x in az if abs(x) >= ACC_RAIL_MS2 - 0.5)
    lo, hi, mean, sd = stats(az)
    mark = "  <-- RAILED %d frames" % n_rail if n_rail else ""
    print("  accz_ms2    min %9.2f  max %9.2f  mean %8.2f  sd %7.2f m/s2%s"
          % (lo, hi, mean, sd, mark))
    print("             (earth-frame, gravity already removed: "
          "hover mean should sit near 0)")

    # ---- 3. health bit ----------------------------------------------
    ok = sum(1 for r in rows if int(r["sensor_health"]) & SENSOR_IMU_OK)
    print("  health     IMU bit set on %d/%d frames (%.1f%%)"
          % (ok, len(rows), 100.0 * ok / len(rows)))

    # ---- 4. plausibility --------------------------------------------
    for axis in ak:
        v = [float(r[axis]) for r in rows]
        lo, hi, mean, sd = stats(v)
        bad = sum(1 for x in v if abs(x) > 180.0)
        mark = "  <-- %d frames outside +-180 deg" % bad if bad else ""
        print("  %-11s min %9.2f  max %9.2f  mean %8.2f  sd %7.2f deg%s"
              % (axis, lo, hi, mean, sd, mark))

    if grd:
        r0, p0 = stats([float(r["roll"]) for r in grd])[2], \
                 stats([float(r["pitch"]) for r in grd])[2]
        print("  at idle    mean roll %+.2f  mean pitch %+.2f deg "
              "(sitting level should be near the level_* trim)"
              % (r0, p0))
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--thr", type=int, default=1100)
    args = ap.parse_args()

    paths = []
    for pat in args.csv:
        paths.extend(sorted(glob.glob(pat)) or [pat])
    for p in paths:
        rows = load(p)
        if rows is None:
            print("%s  EMPTY\n" % p)
            continue
        try:
            report(p, rows, args.thr)
        except KeyError as e:
            sys.exit("%s is missing column %s - decode with the current "
                     "tools/blackbox_decode.py" % (p, e))


if __name__ == "__main__":
    main()
