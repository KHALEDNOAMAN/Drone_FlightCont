#!/usr/bin/env python3
"""Voltage or TIME? Inside one flight they are the same variable.
Bin BARO_OK by seconds since arm, 5 s buckets, and print vbat beside it."""
import csv, sys

for path in sys.argv[1:]:
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                rows.append((int(row["t_us"]), int(row["sensor_health"]),
                             int(row["vbat_mv"]), int(row["throttle"])))
            except (KeyError, ValueError):
                pass
    if not rows:
        continue
    t0 = rows[0][0]
    bins = {}
    for t, h, v, thr in rows:
        b = int((t - t0) / 1e6 // 5) * 5
        e = bins.setdefault(b, [0, 0, 0, 0])
        e[0] += 1
        e[1] += (h >> 1) & 1
        e[2] += v
        e[3] += thr
    print(f"\n=== {path} ===")
    print(f"{'t(s)':>6} {'n':>6} {'BARO':>8} {'vbat':>8} {'thr':>7}")
    for b in sorted(bins):
        n, ok, v, thr = bins[b]
        if n < 50:
            continue
        print(f"{b:>6} {n:>6} {100.0*ok/n:>7.1f}% {v/n/1000:>7.2f}V {thr/n:>7.0f}")
