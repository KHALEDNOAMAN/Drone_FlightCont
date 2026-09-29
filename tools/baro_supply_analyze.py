#!/usr/bin/env python3
"""Why did a sensor die in flight: supply, EMI, or a wedged chip?

Point it at decoded blackbox CSVs:

    python tools/baro_supply_analyze.py aug27_s86.csv aug27_s88.csv ...

Written after SENSOR_BARO_OK was measured at 88.3% idle and 17.1%
flying, and two plausible-sounding explanations turned out to be wrong. The
point of this file is that the three candidates make DIFFERENT predictions,
and the log can separate them:

  a WEDGED CHIP        - one clean step from healthy to dead, and it never
                         comes back, because a DPS310 that stops ACKing
                         cannot be re-initialised (see dps310.c). Health does
                         not care about throttle or voltage.
  RADIATED EMI         - health tracks THROTTLE. Drop the throttle and it
                         recovers, whatever the pack voltage or the time.
  SUPPLY / BROWNOUT    - health tracks PACK VOLTAGE, and collapses at the SAME
                         voltage in every flight, whatever the throttle or the
                         duration.

The third is what the flights actually showed: a cliff at ~15.2 V
reproduced across three flights of 35, 51 and 64 s.

CONFOUND, and it is why `--by-voltage` pools flights: within ONE flight the
pack drains monotonically, so voltage and elapsed time are almost perfectly
correlated and CANNOT be separated. They only come apart ACROSS flights that
start at different voltages and run for different lengths. Never conclude
"it is the supply" from a single log.
"""
import argparse
import csv
import statistics
import sys

SENSOR_BITS = {"imu": 1 << 0, "baro": 1 << 1, "mag_ok": 1 << 2,
               "mag_gated": 1 << 3}


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({
                "t": int(r["t_us"]),
                "health": int(r["sensor_health"]),
                "thr": int(r["throttle"]),
                "vbat": int(r["vbat_mv"]),
                "accz": float(r["accz_ms2"]),
            })
    return rows


def ok_of(rows, bit):
    return [1 if (r["health"] & bit) else 0 for r in rows]


def timeline(rows, bit, path):
    ok = ok_of(rows, bit)
    n = len(ok)
    if not n:
        return
    dur = (rows[-1]["t"] - rows[0]["t"]) / 1e6
    trans = sum(1 for i in range(1, n) if ok[i] != ok[i - 1])
    print("  %-22s %d frames, %.1f s, OK %.1f%%, %d transitions"
          % (path, n, dur, 100.0 * sum(ok) / n, trans))
    if dur <= 0:
        return
    line = []
    for s in range(int(dur) + 1):
        lo = int(s * n / dur)
        hi = min(int((s + 1) * n / dur), n)
        if hi <= lo:
            continue
        frac = sum(ok[lo:hi]) / float(hi - lo)
        line.append("." if frac < 0.05 else str(min(9, int(frac * 10))))
    print("    each char = 1 s, 9 = 90-100% OK, . = dead:")
    print("    " + "".join(line))


def by_key(rows, bit, key, width, label, minn):
    ok = ok_of(rows, bit)
    bins = {}
    for r, o in zip(rows, ok):
        v = r[key]
        if v <= 0:
            continue
        b = (v // width) * width
        d = bins.setdefault(b, [0, 0])
        d[0] += o
        d[1] += 1
    print("  %s" % label)
    for b in sorted(bins, reverse=True):
        okc, n = bins[b]
        if n < minn:
            continue
        bar = "#" * int(round(40.0 * okc / n))
        print("    %6d  n=%6d  %5.1f%%  %s" % (b, n, 100.0 * okc / n, bar))


def decouple(rows, bit, name):
    """Same throttle, different time -> is it time or throttle?"""
    ok = ok_of(rows, bit)
    if not rows:
        return
    t0 = rows[0]["t"]
    dur = (rows[-1]["t"] - t0) / 1e6
    if dur <= 0:
        return

    def sel(early, hi_thr):
        out = []
        for r, o in zip(rows, ok):
            e = (r["t"] - t0) / 1e6
            is_early = e < dur * 0.25
            if is_early != early:
                continue
            if hi_thr and r["thr"] < 1300:
                continue
            if (not hi_thr) and r["thr"] >= 1100:
                continue
            out.append(o)
        return out

    for early in (True, False):
        for hi in (True, False):
            s = sel(early, hi)
            if len(s) < 50:
                continue
            print("    %-5s flight, throttle %-5s : n=%6d  OK %5.1f%%"
                  % ("EARLY" if early else "LATE",
                     ">=1300" if hi else "<1100",
                     len(s), 100.0 * sum(s) / len(s)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+")
    ap.add_argument("--sensor", default="baro", choices=sorted(SENSOR_BITS))
    ap.add_argument("--accz", action="store_true",
                    help="also bin accz by voltage at fixed throttle")
    a = ap.parse_args()
    bit = SENSOR_BITS[a.sensor]

    loaded = []
    for p in a.csvs:
        rows = load(p)
        if rows:
            loaded.append((p, rows))
        else:
            print("%s: empty" % p)
    if not loaded:
        return 1

    print("=" * 74)
    print("TIMELINE  (a single clean step = wedged chip)")
    for p, rows in loaded:
        timeline(rows, bit, p)

    print("=" * 74)
    print("THROTTLE vs TIME  (if it recovers at low throttle LATE, it is EMI)")
    for p, rows in loaded:
        print("  %s" % p)
        decouple(rows, bit, p)

    print("=" * 74)
    print("POOLED BY PACK VOLTAGE, mV  (a repeatable cliff = supply)")
    print("  Pooling is the point: within one flight, voltage and time are")
    print("  the same variable. Only across flights do they separate.")
    allrows = [r for _, rows in loaded for r in rows]
    by_key(allrows, bit, "vbat", 100, "%s_OK vs pack mV" % a.sensor.upper(), 100)

    print("=" * 74)
    print("PER FLIGHT BY PACK VOLTAGE  (the cliff must land in the same bin)")
    for p, rows in loaded:
        print("  %s" % p)
        by_key(rows, bit, "vbat", 100, "", 40)

    if a.accz:
        print("=" * 74)
        print("accz vs pack voltage, THROTTLE HELD >=1400 us")
        print("  NOTE: this does NOT hold thrust constant - as the pack sags")
        print("  the same throttle makes less thrust, so vibration and")
        print("  voltage stay partly confounded. Direction only.")
        for p, rows in loaded:
            hi = [r["accz"] for r in rows
                  if r["vbat"] >= 15200 and r["thr"] >= 1400]
            lo = [r["accz"] for r in rows
                  if 0 < r["vbat"] < 15200 and r["thr"] >= 1400]
            print("  %s" % p)
            for nm, s in ((">=15.20 V", hi), ("< 15.20 V", lo)):
                if len(s) > 50:
                    print("    %s n=%6d  accz mean %+.2f  sd %.2f"
                          % (nm, len(s), statistics.mean(s),
                             statistics.pstdev(s)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
