#!/usr/bin/env python3
"""Does the whole sensor RAIL brown out, or only the DPS310?

sensor_health bits: 0 IMU, 1 BARO, 2 MAG_OK, 3 MAG_GATED.
The mag sits on the SAME I2C bus and the SAME LDO2 rail as the baro.
If LDO2 browns out, EVERY bit must fall together in the low-voltage bins.
If only bit 1 falls, the fault is in the DPS310's own branch or the chip.
"""
import csv, sys
from collections import defaultdict

BITS = [("IMU", 0), ("BARO", 1), ("MAG_OK", 2), ("MAG_GATED", 3)]

for path in sys.argv[1:]:
    bins = defaultdict(lambda: [0, 0, 0, 0, 0])   # n, then one counter per bit
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                v = int(row["vbat_mv"]); h = int(row["sensor_health"])
            except (KeyError, ValueError):
                continue
            if v < 10000:            # blanked cache / not yet read
                continue
            b = round(v / 100) * 100 # 0.1 V bins
            e = bins[b]
            e[0] += 1
            for i, (_, bit) in enumerate(BITS):
                e[1 + i] += (h >> bit) & 1

    print(f"\n=== {path} ===")
    print(f"{'V':>6} {'n':>7}  " + "  ".join(f"{n:>9}" for n, _ in BITS))
    for b in sorted(bins, reverse=True):
        e = bins[b]
        n = e[0]
        if n < 100:
            continue
        pct = "  ".join(f"{100.0*e[1+i]/n:>8.1f}%" for i in range(len(BITS)))
        print(f"{b/1000:>6.1f} {n:>7}  {pct}")
