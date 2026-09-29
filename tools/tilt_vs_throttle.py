#!/usr/bin/env python3
"""Why does roll lean at high throttle on a table? Accel or magnetometer.

The 3.5 deg roll lean (props off, max throttle, pitch untouched) is the attitude filter being pulled to a different reference, not
a gyro bias: the logged roll rate stays at ~0 while roll climbs at Madgwick's
correction rate and falls back when the throttle drops. Two references can
pull it: the ACCELEROMETER (vibration rectified into a DC offset on one axis)
or the MAGNETOMETER (motor/ESC current bending the field; this firmware's
9-DOF Madgwick lets the mag correct roll and pitch, not only heading).

This bins the USB D-stream (50 Hz: raw accel, mag, attitude, motor PWM) by
mean motor output and prints, per bin:
  - accel mean and the roll/pitch that gravity ALONE implies,
  - the mag vector, its magnitude and how far its direction turned,
  - the firmware roll/pitch/yaw.
Read it like this:
  accel-only roll moves with the firmware roll  -> the accelerometer
  accel-only roll flat, mag direction turns      -> the magnetometer
Confirm with the 6-DOF A/B: an all-zero MAGSET matrix drops the
mag out of the filter; if the lean stays, it is the accelerometer.

    python tools/tilt_vs_throttle.py capture.csv
    python tools/tilt_vs_throttle.py --port COM8 --seconds 90 --save ramp.csv

--port records without resetting the S3 (DTR/RTS low). Arm, props OFF, and
ramp slowly: idle, then a few seconds at each step, then 10+ s at max.
"""
import argparse
import math
import sys
import time

BINS = [(999, 1001), (1001, 1150), (1150, 1300), (1300, 1450),
        (1450, 1650), (1650, 1850), (1850, 2001)]


def parse(lines):
    """D-lines -> list of (ax,ay,az, mx,my,mz, roll,pitch,yaw, pwm_mean)."""
    out = []
    for line in lines:
        if not line.startswith("D,"):
            continue
        p = line.strip().split(",")
        try:
            v = [float(x) for x in p[1:24]]
        except ValueError:
            continue
        if len(v) < 23:
            continue
        ax, ay, az = v[1:4]
        mx, my, mz = v[7:10]
        roll, pitch, yaw = v[14:17]
        pwm = sum(v[19:23]) / 4.0
        out.append((ax, ay, az, mx, my, mz, roll, pitch, yaw, pwm))
    return out


def record(port, seconds, save):
    from fc_common import open_port
    s = open_port(port)
    s.write(b"STREAM,1\n")
    lines, end = [], time.time() + seconds
    fh = open(save, "w") if save else None
    try:
        while time.time() < end:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore")
            if line.startswith("D,"):
                lines.append(line)
                if fh:
                    fh.write(line)
    finally:
        s.close()
        if fh:
            fh.close()
    return lines


def mean(xs):
    return sum(xs) / len(xs)


def report(rows):
    print(f"{len(rows)} D-lines")
    ref_mag = None
    ref = None
    for lo, hi in BINS:
        b = [r for r in rows if lo <= r[9] < hi]
        if len(b) < 20:
            continue
        ax, ay, az = (mean([r[i] for r in b]) for i in range(3))
        mx, my, mz = (mean([r[i] for r in b]) for i in range(3, 6))
        roll, pitch = mean([r[6] for r in b]), mean([r[7] for r in b])
        yaw = math.degrees(math.atan2(mean([math.sin(math.radians(r[8])) for r in b]),
                                      mean([math.cos(math.radians(r[8])) for r in b])))
        acc_roll = math.degrees(math.atan2(ay, az))
        acc_pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        m = (mx, my, mz)
        mn = math.sqrt(sum(c * c for c in m))
        if ref_mag is None and mn > 0:
            ref_mag = m
        turn = 0.0
        if ref_mag and mn > 0:
            rn = math.sqrt(sum(c * c for c in ref_mag))
            cosang = sum(a * c for a, c in zip(m, ref_mag)) / (mn * rn)
            turn = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        if ref is None:
            ref = (acc_roll, acc_pitch, roll, pitch)
        sd_y = math.sqrt(mean([(r[1] - ay) ** 2 for r in b]))
        print(f"\nmotors {lo:4d}-{hi:4d} us  n={len(b)}")
        print(f"  accel {ax:+.4f} {ay:+.4f} {az:+.4f} g (sd y {sd_y:.3f})  "
              f"gravity-only roll {acc_roll:+.2f} ({acc_roll - ref[0]:+.2f})  "
              f"pitch {acc_pitch:+.2f} ({acc_pitch - ref[1]:+.2f})")
        print(f"  mag   {mx:+.2f} {my:+.2f} {mz:+.2f} uT  |m| {mn:.2f}  "
              f"turned {turn:.2f} deg")
        print(f"  firmware roll {roll:+.2f} ({roll - ref[2]:+.2f})  "
              f"pitch {pitch:+.2f} ({pitch - ref[3]:+.2f})  yaw {yaw:+.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("file", nargs="?")
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=90)
    ap.add_argument("--save")
    a = ap.parse_args()
    if a.port:
        lines = record(a.port, a.seconds, a.save)
    elif a.file:
        with open(a.file, errors="ignore") as fh:
            lines = fh.readlines()
    else:
        sys.exit("give a capture file or --port")
    report(parse(lines))


if __name__ == "__main__":
    main()
