#!/usr/bin/env python3
"""Frame resonance hunt: Welch PSD + spectrogram of the G, gyro stream.

Reads the "G,t_us,gx,gy,gz" lines a FFT_LOG=1 build emits at the DRDY loop
rate (~423 Hz) and answers the only question the sweep exists to answer:
which peaks are STRUCTURAL (fixed frequency -> notch them) and which are
RPM harmonics (frequency slides with throttle -> a static notch drifts
straight off them and is worse than useless).

The discriminator is the spectrogram, not the PSD. A motor harmonic and a
frame mode look identical in an averaged spectrum; they only separate when
you watch the peak move - or not move - as the throttle ramps. So for each
candidate peak this tracks the local argmax frequency slice by slice and
reports how far it wanders.

Usage:
    python fft_analyze.py sweep1.csv [sweep2.csv ...] [--fmax 250] [--no-plot]

Multiple files are analysed separately then compared: a real resonance
repeats across runs, a one-off does not. That is why BENCH_FFT.md asks for
2-3 sweeps.
"""
import argparse
import sys

import numpy as np
from scipy import signal


def load(path):
    """G, lines -> (t_seconds, gx, gy, gz). Ignores M,/D,/CSV/'#' lines."""
    rows = []
    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            if not line.startswith("G,"):
                continue
            f = line.strip().split(",")
            if len(f) < 5:
                continue
            try:
                rows.append([float(f[1]), float(f[2]), float(f[3]), float(f[4])])
            except ValueError:
                continue
    if not rows:
        return None
    d = np.asarray(rows)
    return d[:, 0] * 1e-6, d[:, 1], d[:, 2], d[:, 3]


def check_timebase(t):
    """fs measured, never assumed - the LSM6DSOX oscillator runs ~1.7% fast.

    Also counts gaps: a dropped block is a step in the series, and a step
    is broadband energy that shows up as a fake noise floor across the
    whole spectrum. Better to know than to notch a phantom.
    """
    dt = np.diff(t)
    med = np.median(dt)
    fs = 1.0 / med
    gaps = int(np.sum(dt > 3 * med))
    # uint32 microsecond timestamps wrap every ~71.6 min
    wraps = int(np.sum(dt < 0))
    return fs, med, gaps, wraps


def peak_persistence(x, fs, f0, nperseg=512, band=0.20):
    """Is there energy at EXACTLY f0 the whole run, or only in passing?

    Returns (occupancy, drift_hz).

    The obvious test - track the loudest frequency in a band and see if it
    moves - fails on real hardware, and failed on the synthetic check that
    built this: when an RPM harmonic sweeps THROUGH a frame mode, argmax
    follows whichever is louder at that instant, so a genuine structural
    peak gets reported as sliding. Frames really do have both near the same
    frequency, so that is not a corner case.

    Persistence separates them properly. A structural mode is excited at
    its own fixed frequency for the whole sweep - amplitude rises and falls
    with throttle, but the FREQUENCY bin is always lit. A sweeping harmonic
    lights that bin only while passing through it, a small fraction of the
    run. Each time slice is normalised by its own band median first, so
    throttle-driven amplitude changes cannot be mistaken for presence.
    """
    f, tt, Sxx = signal.spectrogram(x - x.mean(), fs=fs, nperseg=nperseg)
    i0 = int(np.argmin(np.abs(f - f0)))
    lo, hi = f0 * (1 - band), f0 * (1 + band)
    sel = (f >= lo) & (f <= hi)
    if sel.sum() < 5 or Sxx.shape[1] < 6:
        return 1.0, 0.0
    fb, Sb = f[sel], Sxx[sel, :]

    # Ignore slices where the whole band is quiet (motors near idle).
    tot = Sb.sum(axis=0)
    live = tot > np.median(tot) * 0.15
    if live.sum() < 5:
        return 1.0, 0.0

    ratio = Sxx[i0, :] / (np.median(Sb, axis=0) + 1e-30)
    occupancy = float(np.mean(ratio[live] > 3.0))
    track = fb[np.argmax(Sb[:, live], axis=0)]
    return occupancy, float(track.max() - track.min())


def analyse_axis(name, g, fs, fmax, results):
    f, pxx = signal.welch(g - g.mean(), fs=fs, nperseg=2048)
    band = f <= fmax
    f, pxx = f[band], pxx[band]

    # Ignore the DC/handling hump; real frame modes live well above this.
    floor_i = np.searchsorted(f, 8.0)
    med = np.median(pxx[floor_i:])
    peaks, props = signal.find_peaks(10 * np.log10(pxx[floor_i:]),
                                     prominence=6.0, distance=4)
    peaks = peaks + floor_i

    order = np.argsort(props["prominences"])[::-1][:5]
    print(f"\n  {name}:")
    if len(peaks) == 0:
        print("    no peaks above 6 dB prominence - broadband only.")
        print("    That is general vibration, not a resonance. Fix it")
        print("    mechanically (balance props, check arm screws, soft-mount")
        print("    the FC) before reaching for a filter.")
        return

    print(f"    {'freq':>8} {'prom':>7} {'x floor':>8} {'lit':>6}")
    for i in order:
        pi = peaks[i]
        f0 = float(f[pi])
        prom = float(props["prominences"][i])
        rel = float(pxx[pi] / med)
        occ, drift = peak_persistence(g, fs, f0)
        # No verdict here any more. A single run cannot tell a frame mode
        # from an RPM harmonic reliably - measured, the same
        # 80 Hz peak scored 74%/57%/76% across three sweeps and flipped
        # verdict on the middle one purely from where a threshold sat.
        # The call is made across runs instead, in cross_run_report.
        results.append((name, f0, prom, rel, occ))
        print(f"    {f0:7.1f}H {prom:6.1f}dB {rel:7.0f}x {occ*100:5.0f}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--fmax", type=float, default=250.0)
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    all_struct = {}
    for path in a.files:
        got = load(path)
        if got is None:
            print(f"{path}: no G, lines - was FFT_LOG=1 flashed?")
            continue
        t, gx, gy, gz = got
        fs, med, gaps, wraps = check_timebase(t)
        dur = t[-1] - t[0]

        print(f"\n=== {path} ===")
        print(f"  {len(t)} samples, {dur:.1f} s, fs = {fs:.1f} Hz measured "
              f"(usable band DC-{fs/2:.0f} Hz)")
        if wraps:
            print(f"  WARNING {wraps} timestamp wraps - uint32 us rolls over "
                  "every ~71 min; fs may be wrong")
        if gaps:
            pct = 100.0 * gaps / len(t)
            print(f"  WARNING {gaps} gaps ({pct:.2f}%) - dropped samples add a "
                  "fake broadband floor")
        if dur < 20:
            print("  WARNING run is short; a slow 25 s up + 25 s down sweep is "
                  "what lets a mode ring up")

        results = []
        for nm, g in (("gx (roll)", gx), ("gy (pitch)", gy), ("gz (yaw)", gz)):
            analyse_axis(nm, g, fs, a.fmax, results)

        for nm, f0, prom, rel, occ in results:
            all_struct.setdefault(nm, []).append((path, f0, prom, rel, occ))

        if not a.no_plot:
            plot(path, t, gx, gy, gz, fs, a.fmax)

    if len(a.files) > 1:
        cross_run_report(all_struct, len(a.files))


def cluster(peaks, tol=0.08):
    """Group peaks from different runs that are the same physical mode.

    Fixed-width bins do not work: a real mode wanders a few Hz run to run,
    so round(f/5)*5 split one 75-83 Hz peak across three buckets and made
    it look like three unconfirmed peaks. Relative tolerance tracks the
    fact that a 6 % wander is 1 Hz at 13 Hz and 12 Hz at 175 Hz.
    """
    out = []
    for p in sorted(peaks, key=lambda p: p[1]):
        if out and abs(p[1] - np.mean([q[1] for q in out[-1]])) <= tol * p[1]:
            out[-1].append(p)
        else:
            out.append([p])
    return out


def cross_run_report(by_axis, n_runs):
    """Decide structural vs RPM ACROSS runs, not within one.

    A structural mode sits at its own fixed frequency every time the frame
    is excited, so independent sweeps reproduce it tightly - the pitch mode
    on this airframe landed at 60.1/60.1/60.3 Hz. An RPM harmonic lands
    wherever the throttle happened to be when it was loudest, so it will
    not reproduce to a fraction of a percent three times running.
    """
    print("\n" + "=" * 62)
    print("CROSS-RUN VERDICT  (this is the one to act on)")
    print("=" * 62)
    print("A mode that reproduces its FREQUENCY across independent sweeps is")
    print("structural. Frequency scatter is the test; a single run cannot")
    print("make this call.\n")

    notch = []
    for axis in ("gx (roll)", "gy (pitch)", "gz (yaw)"):
        peaks = by_axis.get(axis, [])
        if not peaks:
            continue
        print(f"  {axis}")
        for grp in cluster(peaks):
            runs = len({g[0] for g in grp})
            fs_ = np.array([g[1] for g in grp])
            rel = np.array([g[3] for g in grp])
            occ = np.array([g[4] for g in grp])
            scatter = (fs_.max() - fs_.min()) / fs_.mean() if len(fs_) > 1 else 0.0

            # The decisive question is not "what causes it" but "can a
            # STATIC notch hold it". A biquad notch at Q=15 is about 5 Hz
            # wide at -3 dB, so a peak that wanders 9 % run-to-run walks
            # straight out of it no matter what the physics is. That makes
            # the answer actionable even where the cause stays ambiguous.
            if runs < n_runs:
                tag = f"only {runs}/{n_runs} runs - ignore"
            elif scatter < 0.02:
                tag = f"STRUCTURAL, scatter {scatter*100:.1f}% - NOTCH THIS"
                notch.append((axis, float(fs_.mean()), float(rel.max())))
            elif scatter < 0.06:
                tag = (f"probably structural, scatter {scatter*100:.1f}% - "
                       "check the spectrogram first")
            else:
                tag = (f"WANDERS {scatter*100:.0f}% - do NOT notch, a static "
                       "notch cannot hold it")
            print(f"    {fs_.mean():6.1f} Hz  x{rel.max():5.0f} floor  "
                  f"[{fs_.min():.1f}-{fs_.max():.1f}]  lit {occ.mean()*100:.0f}%"
                  f"  {tag}")
        print()

    if notch:
        print("  Candidates to notch, strongest first:")
        for axis, f0, rel in sorted(notch, key=lambda x: -x[2]):
            print(f"    {f0:6.1f} Hz  ({axis}, {rel:.0f}x noise floor)")
        print()
    print("  A wandering peak is NOT a filter problem. Either it follows RPM")
    print("  (balance the props, check motor bells) or the mount tension")
    print("  changed between runs. Both are mechanical. Look at the")
    print("  spectrogram in the PNG: a DIAGONAL streak is RPM, a HORIZONTAL")
    print("  band is a mode. That picture settles it faster than any metric")
    print("  here can.")
    print()
    print("  CAVEAT: measured with the airframe RESTRAINED, so every mode")
    print("  here belongs to drone + mount together. To tell them apart,")
    print("  re-run one sweep with the strapping moved or changed - mount")
    print("  modes shift, airframe modes do not.")


def plot(path, t, gx, gy, gz, fs, fmax):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib not installed - numbers only, no plots)")
        return

    fig, ax = plt.subplots(2, 1, figsize=(11, 9))
    for nm, g in (("gx", gx), ("gy", gy), ("gz", gz)):
        f, pxx = signal.welch(g - g.mean(), fs=fs, nperseg=2048)
        sel = f <= fmax
        ax[0].semilogy(f[sel], pxx[sel], label=nm, lw=0.9)
    ax[0].set_xlabel("Hz"); ax[0].set_ylabel("power")
    ax[0].set_title(f"{path} - Welch PSD"); ax[0].legend(); ax[0].grid(alpha=.3)

    f, tt, Sxx = signal.spectrogram(gz - gz.mean(), fs=fs, nperseg=512)
    sel = f <= fmax
    ax[1].pcolormesh(tt, f[sel], 10 * np.log10(Sxx[sel] + 1e-20), shading="auto")
    ax[1].set_xlabel("s"); ax[1].set_ylabel("Hz")
    ax[1].set_title("gz spectrogram - horizontal line = structural, "
                    "diagonal = RPM harmonic")

    out = path.rsplit(".", 1)[0] + "_fft.png"
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f"  plot -> {out}")


if __name__ == "__main__":
    main()
