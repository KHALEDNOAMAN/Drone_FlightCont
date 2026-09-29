#!/usr/bin/env python3
"""Which motor puts the DC offset on accel Y?  One motor at a time.

The props-off roll lean is the accelerometer (6-DOF A/B: mag out of
the filter, lean stays) and it is not clipping (16 g did not move it:
+0.041 g vs +0.046 g at 8 g). This spins each motor alone with the
firmware's disarmed bench test (MOT,<n>,<us>,<ms>, capped 1700 us / 10 s)
and prints, per motor, the accel shift against that motor's own idle
baseline just before it.

Read it like this:
  one motor far above the rest  -> that motor/prop-adapter/bearing;
                                   balance or swap it, re-test
  all four about the same, same sign -> a sensor response to vibration
                                   (not one bad part): isolation, or the
                                   filter trusting the accel less
  sign follows the motor's side -> the frame, not the chip

    python tools/lean_per_motor.py COM8 --save captures/bench/lean_motor.csv
    python tools/lean_per_motor.py --file captures/bench/lean_motor.csv

DISARMED, PROPS OFF. Records without resetting the S3 (DTR/RTS low).
"""
import argparse
import math
import sys
import time

IDLE_S, SPIN_S, SETTLE_S, REST_S = 4.0, 10.0, 4.0, 6.0


def parse(line):
    """D-line -> (ax, ay, az, roll) or None. Same columns as tilt_vs_throttle."""
    p = line.strip().split(",")
    try:
        v = [float(x) for x in p[1:24]]
    except ValueError:
        return None
    if len(v) < 23:
        return None
    return v[1], v[2], v[3], v[14]


def run(port, us, save):
    from fc_common import open_port
    s = open_port(port, timeout=0.05)
    s.write(b"STREAM,1\n")
    fh = open(save, "w") if save else None
    t0 = time.time()

    def grab(seconds, tag):
        """Record D-lines for `seconds`, tagging each with the phase name."""
        out, end = [], time.time() + seconds
        while time.time() < end:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore")
            if not line.startswith("D,"):
                if "MOT" in line:
                    print("  S3:", line.strip())
                continue
            out.append(line)
            if fh:
                fh.write(f"{tag},{time.time() - t0:.3f},{line}")
        return out

    phases = []
    try:
        for n in (1, 2, 3, 4):
            print(f"M{n}: idle {IDLE_S:.0f} s", flush=True)
            base = grab(IDLE_S, f"M{n}_idle")
            print(f"M{n}: spin {us} us {SPIN_S:.0f} s", flush=True)
            s.write(f"MOT,{n},{us},{int(SPIN_S * 1000) + 500}\n".encode())
            grab(SETTLE_S, f"M{n}_settle")
            spin = grab(SPIN_S - SETTLE_S, f"M{n}_spin")
            s.write(b"MOT,0\n")
            grab(REST_S, f"M{n}_rest")
            phases.append((n, base, spin))
    finally:
        s.write(b"MOT,0\n")
        s.close()
        if fh:
            fh.close()
    return phases


def load(path):
    """Saved capture -> the same (n, base, spin) phases."""
    groups = {}
    with open(path, errors="ignore") as fh:
        for row in fh:
            tag, _, line = row.split(",", 2)
            groups.setdefault(tag, []).append(line)
    return [(n, groups.get(f"M{n}_idle", []), groups.get(f"M{n}_spin", []))
            for n in (1, 2, 3, 4)]


def stats(lines):
    rows = [r for r in map(parse, lines) if r]
    if len(rows) < 20:
        return None
    m = [sum(r[i] for r in rows) / len(rows) for i in range(4)]
    sd_y = math.sqrt(sum((r[1] - m[1]) ** 2 for r in rows) / len(rows))
    return m, sd_y, len(rows)


def report(phases):
    print("\n       dAx g     dAy g     dAz g   sd y   d roll(accel) d roll(fw)   n")
    for n, base, spin in phases:
        b, sp = stats(base), stats(spin)
        if not b or not sp:
            print(f"M{n}: too few D-lines (idle {len(base)}, spin {len(spin)})")
            continue
        (bx, by, bz, br), _, _ = b
        (sx, sy, sz, sr), sd_y, cnt = sp
        acc_roll = math.degrees(math.atan2(sy, sz) - math.atan2(by, bz))
        print(f"M{n}  {sx - bx:+.4f}   {sy - by:+.4f}   {sz - bz:+.4f}  "
              f"{sd_y:.3f}      {acc_roll:+.2f}        {sr - br:+.2f}   {cnt}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port", nargs="?")
    ap.add_argument("--file")
    ap.add_argument("--us", type=int, default=1700)
    ap.add_argument("--save")
    a = ap.parse_args()
    if a.file:
        report(load(a.file))
    elif a.port:
        report(run(a.port, a.us, a.save))
    else:
        sys.exit("give a port or --file")


if __name__ == "__main__":
    main()
