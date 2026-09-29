#!/usr/bin/env python3
"""Compare the two barometers in one flight - the DPS310 control experiment.

    python tools/baro_compare.py aug30_s12.csv [more.csv ...]

THE QUESTION THIS ANSWERS. Since the DPS310 has read 0 % health
above roughly 1400 us of throttle and does not recover when throttle drops.
The supply theory died: the magnetometer shares bus A and the LDO2
rail and holds 89-100 % in every voltage bin, including the bins where the
baro reads 0. So the DPS310 is either the culprit or the victim, and one
sensor cannot tell us which.

Fitting a second barometer of a DIFFERENT vendor and process (MS5611,
TE/MEAS, at 0x76) and flying both in one log makes it a two-line answer:

    MS5611 survives while the DPS310 dies -> it is the DPS310. Swap it.
    both die together                     -> shared cause. The mag on the
                                             same rail is fine, so that
                                             means VIBRATION.

Health comes from the sensor_health bitmask, bit 1 = DPS310, bit 4 =
MS5611. Bit 4 is 0 for every frame in every log recorded before the part
was fitted, and this script says "not fitted" rather than "dead" in that
case - a hole in the data is not a measurement.

Companion to tools/rail_test.py (health binned by pack voltage) and
tools/baro_time.py (health binned by time since throttle-up).
"""

import csv
import sys

BARO_BIT = 1 << 1
BARO2_BIT = 1 << 4

# Throttle bins. The interesting edge is ~1400 us, so the bins are placed
# to straddle it rather than to be round numbers.
BINS = [(1000, 1100), (1100, 1200), (1200, 1300), (1300, 1400),
        (1400, 1500), (1500, 1600), (1600, 1750), (1750, 2100)]


def pct(n, d):
    return 100.0 * n / d if d else float("nan")


def analyse(path):
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print(f"{path}: empty")
        return None

    if "baro2_m" not in rows[0]:
        print(f"{path}: decoded by an OLD blackbox_decode.py - no baro2_m "
              f"column. Re-decode with the current tool.")
        return None

    n = len(rows)
    health = [int(r["sensor_health"]) for r in rows]
    thr = [int(r["throttle"]) for r in rows]

    b1 = [bool(h & BARO_BIT) for h in health]
    b2 = [bool(h & BARO2_BIT) for h in health]

    fitted = any(b2)
    dur = (int(rows[-1]["t_us"]) - int(rows[0]["t_us"])) / 1e6

    print(f"\n=== {path} ===")
    print(f"{n} frames, {dur:.1f} s, throttle {min(thr)}-{max(thr)} us")

    if not fitted:
        print("\n  MS5611: NOT FITTED in this log (bit 4 never set).")
        print("  Nothing to compare. This is the expected result for every")
        print("  log recorded before the part was installed - it is not a")
        print("  failure of the second barometer.")
        print(f"\n  DPS310 health over the whole session: "
              f"{pct(sum(b1), n):.1f} %")
        return None

    print(f"\n  {'throttle':>12}  {'frames':>7}  {'DPS310':>8}  {'MS5611':>8}")
    print(f"  {'-'*12}  {'-'*7}  {'-'*8}  {'-'*8}")
    rowsout = []
    for lo, hi in BINS:
        idx = [i for i in range(n) if lo <= thr[i] < hi]
        if not idx:
            continue
        h1 = pct(sum(b1[i] for i in idx), len(idx))
        h2 = pct(sum(b2[i] for i in idx), len(idx))
        rowsout.append((lo, hi, len(idx), h1, h2))
        print(f"  {lo:5d}-{hi:<6d}  {len(idx):7d}  {h1:7.1f}%  {h2:7.1f}%")

    # Where both are alive, how far apart are they? A steady offset is
    # nothing (two parts, two QNH errors); a GROWING gap is one of them
    # drifting, which is a different fault from one of them dying.
    both = [i for i in range(n) if b1[i] and b2[i]]
    if len(both) > 20:
        d = [float(rows[i]["baro2_m"]) - float(rows[i]["alt_m"]) for i in both]
        half = len(d) // 2
        first, second = sum(d[:half]) / half, sum(d[half:]) / (len(d) - half)
        print(f"\n  Both alive in {len(both)} frames "
              f"({pct(len(both), n):.0f} % of the session).")
        print(f"  baro2 - baro: {first:+.2f} m in the first half, "
              f"{second:+.2f} m in the second")
        print(f"  drift over the session: {second - first:+.2f} m")
    else:
        print(f"\n  The two are alive together in only {len(both)} frames - "
              f"too few to compare traces.")

    # The verdict, stated in the terms the experiment was set up in.
    high = [r for r in rowsout if r[0] >= 1400]
    if not high:
        print("\n  VERDICT: inconclusive - this session never went above "
              "1400 us, which is where the DPS310 fails.")
        return rowsout

    d_high = sum(r[3] * r[2] for r in high) / sum(r[2] for r in high)
    m_high = sum(r[4] * r[2] for r in high) / sum(r[2] for r in high)
    print(f"\n  Above 1400 us:  DPS310 {d_high:.1f} %   MS5611 {m_high:.1f} %")

    if d_high < 25 and m_high > 75:
        print("  VERDICT: the MS5611 survives where the DPS310 dies.")
        print("           It is the DPS310. Swap the part.")
    elif d_high < 25 and m_high < 25:
        print("  VERDICT: BOTH barometers die above 1400 us.")
        print("           A shared cause. The mag on the same rail and bus")
        print("           is unaffected, so this points at VIBRATION, not")
        print("           supply - see PHYS-1.md 5.1.")
    elif d_high > 75 and m_high > 75:
        print("  VERDICT: both survive. Whatever was wrong on 27 August is")
        print("           not reproducing - check what else changed before")
        print("           calling it fixed. Tightening counts as a change.")
    else:
        print("  VERDICT: mixed. Neither clean survival nor clean shared")
        print("           failure - read the per-bin table above rather")
        print("           than trusting a one-line summary.")
    return rowsout


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    for path in argv[1:]:
        analyse(path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
