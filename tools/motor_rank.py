#!/usr/bin/env python3
"""Weak-motor hunt: per-motor RPM ranking from a MOT-command capture.

The ear cannot rank four motors that differ by 5-15 %, and amplitude in
the gyro cannot either (a motor near the IMU is always louder than one
far from it). FREQUENCY can: at the same commanded pulse width a weak
motor settles at a lower RPM under prop load, and the rotation line in
the gyro spectrum gives that RPM to a fraction of a percent.

Feed it one capture.py file (or several) recorded while stepping through
the motors with MOT. It self-segments: the D, stream carries m1-m4 pwm,
so every window where exactly one motor is above idle becomes a labelled
segment - no need to keep files-per-motor straight by hand.

    python tools/motor_rank.py hunt.csv
    MOT,1,1400,8000   ... wait for stop ...
    MOT,2,1400,8000   ... etc, all four, then Ctrl-C the capture.

Needs the FFT_LOG=1 build (G, lines at ~423 Hz). Pair each segment with
the wattmeter current you wrote down - RPM alone can miss a
demagnetised motor (lower torque per amp can hide at similar RPM while
current climbs). Weak = lower RPM at same us, or same RPM at more amps.

Amplitude is deliberately NOT compared across motors; only frequency is.
"""
import argparse

import numpy as np
from scipy import signal

# D, line: 0-indexed fields after split(',')
D_T = 1
D_M1 = 20            # m1..m4 pwm at 20-23 (see debug_stream.c printf)
IDLE_THRESH = 1050   # above this = MOT test running on that motor


def load(path):
    """One pass -> (d_t, d_pwm[n,4]) and (g_t, g[n,3]). Both share the
    esp_timer clock, so D segments can window the G samples directly."""
    d_rows, g_rows = [], []
    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            f = line.strip().split(",")
            try:
                if line.startswith("G,") and len(f) >= 5:
                    g_rows.append([float(f[1]), float(f[2]),
                                   float(f[3]), float(f[4])])
                elif line.startswith("D,") and len(f) >= D_M1 + 4:
                    d_rows.append([float(f[D_T]),
                                   float(f[D_M1]), float(f[D_M1 + 1]),
                                   float(f[D_M1 + 2]), float(f[D_M1 + 3])])
            except ValueError:
                continue
    d = np.asarray(d_rows) if d_rows else None
    g = np.asarray(g_rows) if g_rows else None
    return d, g


def segments(d, min_s=2.0):
    """Windows where exactly ONE motor is above idle, split on motor or
    commanded-us changes. Returns (motor_1based, us, t0, t1)."""
    t = d[:, 0] * 1e-6
    pwm = d[:, 1:5]
    active = np.where(pwm > IDLE_THRESH, pwm, 0)
    n_on = (active > 0).sum(axis=1)
    which = active.argmax(axis=1)

    out = []
    i = 0
    while i < len(t):
        if n_on[i] != 1:
            i += 1
            continue
        j = i
        while (j < len(t) and n_on[j] == 1 and which[j] == which[i]
               and abs(active[j, which[i]] - active[i, which[i]]) <= 10):
            j += 1
        t0, t1 = t[i], t[j - 1]
        if t1 - t0 >= min_s:
            out.append((int(which[i]) + 1,
                        int(round(active[i:j, which[i]].mean())), t0, t1))
        i = j
    return out


def dominant_lines(g, sel, fs, fmax, n_top=3):
    """Top spectral lines of the 3-axis PSD sum inside one segment.

    The sum, not one axis: a single motor shakes all three axes and
    which one is loudest depends on geometry, not health. Also flags a
    half-frequency line - if f0/2 is lit, f0 is the 2nd harmonic and
    the ROTATION is f0/2. At 423 Hz sampling a high 2/rev can fold back
    into band, so the top-3 list is printed rather than trusting any
    single line; ranking compares like with like at the same us anyway.
    """
    x = g[sel]
    if len(x) < 512:
        return None, []
    nper = min(2048, len(x))
    pxx = None
    for ax in range(3):
        f, p = signal.welch(x[:, ax] - x[:, ax].mean(), fs=fs, nperseg=nper)
        pxx = p if pxx is None else pxx + p
    band = (f >= 20.0) & (f <= fmax)
    fb, pb = f[band], pxx[band]
    med = np.median(pb)
    peaks, props = signal.find_peaks(10 * np.log10(pb + 1e-30),
                                     prominence=6.0, distance=4)
    if len(peaks) == 0:
        return None, []
    order = np.argsort(props["prominences"])[::-1][:n_top]
    lines = [(float(fb[peaks[k]]), float(pb[peaks[k]] / med)) for k in order]

    f0 = lines[0][0]
    half = (fb > 0.47 * f0) & (fb < 0.53 * f0)
    sub = half.any() and float(pb[half].max() / med) > 10.0
    return (f0 / 2.0 if sub else f0), lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--fmax", type=float, default=200.0)
    ap.add_argument("--min-s", type=float, default=2.0)
    a = ap.parse_args()

    rows = []
    for path in a.files:
        d, g = load(path)
        if d is None or g is None:
            print(f"{path}: needs BOTH D, lines (motor pwm) and G, lines "
                  "(FFT_LOG=1 build) - got "
                  f"{'no D' if d is None else 'D ok'}, "
                  f"{'no G' if g is None else 'G ok'}")
            continue
        gt = g[:, 0] * 1e-6
        fs = 1.0 / np.median(np.diff(gt))
        print(f"{path}: fs = {fs:.1f} Hz, {len(gt)} gyro samples")

        for motor, us, t0, t1 in segments(d, a.min_s):
            sel = (gt >= t0 + 0.5) & (gt <= t1 - 0.2)   # skip spin-up
            rot, lines = dominant_lines(g[:, 1:4], sel, fs, a.fmax)
            if rot is None:
                print(f"  M{motor} @ {us} us: segment too short/quiet")
                continue
            rows.append((us, motor, t1 - t0, rot, lines))

    if not rows:
        print("no usable segments - was MOT run while capture.py recorded?")
        return

    print(f"\n{'us':>6} {'motor':>6} {'dur':>6} {'rot Hz':>8} "
          f"{'RPM(1/rev)':>11} {'vs best':>8}   top lines (Hz x floor)")
    for us in sorted({r[0] for r in rows}):
        grp = [r for r in rows if r[0] == us]
        best = max(r[3] for r in grp)
        for _, motor, dur, rot, lines in sorted(grp, key=lambda r: r[1]):
            dpc = 100.0 * (rot - best) / best
            ls = "  ".join(f"{f0:.1f}x{rel:.0f}" for f0, rel in lines)
            print(f"{us:>6} {'M%d' % motor:>6} {dur:5.1f}s {rot:8.1f} "
                  f"{rot*60:11.0f} {dpc:+7.1f}%   {ls}")
        print()

    print("Read it with the wattmeter numbers next to it:")
    print("  lower RPM at the same us            -> weak (or that ESC's")
    print("  range differs - swap props/ESCs to split motor vs ESC)")
    print("  same RPM but clearly more amps      -> demagnetising motor")
    print("  all four within ~2-3 %              -> motors are NOT the")
    print("  problem; look at ESC calibration and props instead")


if __name__ == "__main__":
    main()
