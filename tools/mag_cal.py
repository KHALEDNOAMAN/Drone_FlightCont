#!/usr/bin/env python3
"""Magnetometer calibration: ellipsoid fit -> MAGSET line.

The firmware applies   corrected = S * (raw - off)   (mmc5603.c:204-212),
so this has to produce a 3-vector `off` and a 3x3 matrix `S`.

WHAT IT ASSUMES ABOUT THE INPUT
The capture must be of RAW mag, i.e. taken AFTER sending

    MAGSET,0,0,0

which resets the stored calibration to zero offset and identity S. If you
fit a capture that still has an old calibration applied, you are fitting a
correction on top of a correction and the result is meaningless.

THE MATH
An uncalibrated magnetometer traces an ELLIPSOID as you rotate it, not a
sphere. Two distortions cause that:
  hard iron - permanent magnets nearby (and, on this board, the sensor's
              own magnetization) shift the whole ellipsoid off origin.
              That offset is `off`.
  soft iron - ferrous material distorts the field direction-dependently,
              squashing the sphere into an ellipsoid. `S` un-squashes it.

Fit the general quadric
    v^T A v + 2 b^T v = 1
then
    centre c = -A^-1 b
    k       = 1 + c^T A c
    M       = A / k          so that (v-c)^T M (v-c) = 1
    S       = |B| * sqrtm(M) so that |S (v-c)| = |B| in every direction

WHICH LINES IT READS
Two capture formats, auto-detected:
  "M,t_us,mx,my,mz"   the MAGCAP,1 stream - 50 Hz, over the C6 WiFi bridge
                      (`ncat <c6-ip> 5555 > mag.csv`). Use this one: a
                      frame-mounted sensor has to be tumbled by hand and a
                      USB cable will not reach.
  "D,..."             the 50 Hz USB debug line, mx at column 8.
M, wins if both are present in the file, and an FFT "G," flood mixed into
the same capture is ignored either way.

Usage:
    python mag_cal.py capture.csv [--field 47.0] [--col 8]

--field is the local geomagnetic TOTAL intensity in uT. Istanbul is ~47.
Look yours up on the NOAA/BGS calculator if you move.
--col is the 0-indexed column of mx in the D, line (mx,my,mz follow).
Ignored for M, lines, where mx is always column 2.
"""
import sys
import numpy as np
from scipy.linalg import sqrtm


def fit_ellipsoid(xyz, field_ut):
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

    # [x^2 y^2 z^2 2xy 2xz 2yz 2x 2y 2z] . p = 1
    D = np.column_stack([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z])
    p, *_ = np.linalg.lstsq(D, np.ones(len(x)), rcond=None)

    A = np.array([[p[0], p[3], p[4]],
                  [p[3], p[1], p[5]],
                  [p[4], p[5], p[2]]])
    b = p[6:9]

    centre = -np.linalg.solve(A, b)
    k = 1.0 + centre @ A @ centre
    M = A / k

    # Symmetric PSD -> real square root. Discard the imaginary residue
    # sqrtm leaves behind on a symmetric input.
    S = field_ut * np.real(sqrtm(M))
    return centre, S


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(__doc__)
        sys.exit(1)

    field = 47.0
    col = 8
    for a in sys.argv[1:]:
        if a.startswith("--field"):
            field = float(a.split("=", 1)[1]) if "=" in a else field
        if a.startswith("--col"):
            col = int(a.split("=", 1)[1]) if "=" in a else col
    if "--field" in sys.argv:
        field = float(sys.argv[sys.argv.index("--field") + 1])
    if "--col" in sys.argv:
        col = int(sys.argv[sys.argv.index("--col") + 1])

    # Collect both formats in one pass, then prefer M, if it produced
    # anything. A capture taken during an FFT build has "G," lines
    # interleaved at 423 Hz; prefix matching drops them.
    m_rows, d_rows = [], []
    with open(args[0], "r", errors="ignore") as fh:
        for line in fh:
            if line.startswith("M,"):
                dst, first = m_rows, 2
            elif line.startswith("D,"):
                dst, first = d_rows, col
            else:
                continue
            f = line.strip().split(",")
            if len(f) <= first + 2:
                continue
            try:
                dst.append([float(f[first]), float(f[first+1]), float(f[first+2])])
            except ValueError:
                continue

    if m_rows:
        rows, src = m_rows, "M, (MAGCAP stream)"
    else:
        rows, src = d_rows, f"D, (column {col})"
    if not rows:
        print("no M, or D, lines found in that file")
        sys.exit(1)
    print(f"source           : {src}")

    m = np.asarray(rows)
    if len(m) < 500:
        print(f"only {len(m)} samples - tumble longer, want a few thousand")
        if len(m) < 50:
            sys.exit(1)

    # Drop the corrupt reads the I2C bus occasionally produces. They are
    # wildly out of range and would drag the fit badly.
    norm = np.linalg.norm(m, axis=1)
    keep = (norm > 5.0) & (norm < 500.0)
    dropped = len(m) - keep.sum()
    m = m[keep]

    centre, S = fit_ellipsoid(m, field)

    corrected = (S @ (m - centre).T).T
    cn = np.linalg.norm(corrected, axis=1)
    raw_n = np.linalg.norm(m, axis=1)

    print(f"samples used     : {len(m)}  (dropped {dropped} out-of-range)")
    print(f"target field     : {field:.1f} uT")
    print()
    print(f"RAW   |B|  mean {raw_n.mean():7.2f}  std {raw_n.std():6.2f}"
          f"  min {raw_n.min():7.2f}  max {raw_n.max():7.2f}")
    print(f"FIXED |B|  mean {cn.mean():7.2f}  std {cn.std():6.2f}"
          f"  min {cn.min():7.2f}  max {cn.max():7.2f}")
    print()
    print(f"offset (uT)      : {centre[0]:8.3f} {centre[1]:8.3f} {centre[2]:8.3f}")
    print("S matrix         :")
    for r in S:
        print(f"                   {r[0]:8.4f} {r[1]:8.4f} {r[2]:8.4f}")
    diag = np.array([S[0, 0], S[1, 1], S[2, 2]])
    offdiag = max(abs(S[0, 1]), abs(S[0, 2]), abs(S[1, 2]))
    print(f"S diagonal       : {diag[0]:.3f} {diag[1]:.3f} {diag[2]:.3f}")
    print()

    vals = list(centre) + list(S.flatten())
    print("MAGSET," + ",".join(f"{v:.5f}" for v in vals))
    print()

    # ---- Verdict ----
    #
    # This used to fail any fit whose S diagonal was not near 1.0, which was
    # the wrong test and rejected two good calibrations of this board in a
    # row (and, S diag ~2.4 both times). S near 1.0 only
    # holds if the driver's raw-to-microtesla conversion matches the part's
    # actual sensitivity. On a sensor that reads half scale, S absorbs the
    # deficit - and that costs nothing, because madgwick_update NORMALISES
    # the mag vector before use. Absolute scale cancels completely; only
    # DIRECTION reaches the attitude filter.
    #
    # So pass/fail is decided on sphere quality alone, which is exactly what
    # direction accuracy depends on. The S matrix is reported as diagnosis,
    # not as a gate.
    spread = cn.max() - cn.min()
    ok = True

    if cn.std() > 1.5 or spread > 5.0:
        print(f"FAIL  corrected |B| still varies with orientation "
              f"(std {cn.std():.2f} uT, spread {spread:.2f} uT).")
        print("      The ellipsoid did not map to a sphere, so heading will be")
        print("      wrong in some attitudes. Redo the tumble covering all six")
        print("      faces - this is almost always missed orientations.")
        ok = False

    # Diagnosis on S. None of these fail the fit; they say what the hardware
    # is doing, which is what you actually want to know across sessions.
    gain = diag.mean()
    aniso = diag.max() / diag.min()
    if offdiag > 0.05 * gain:
        print(f"NOTE  cross-axis terms are significant (max off-diagonal "
              f"{offdiag:.3f} vs diagonal ~{gain:.2f}).")
        print("      That is real soft iron - ferrous material near the sensor")
        print("      bending the field. Worth locating before you trust yaw in")
        print("      unusual attitudes.")
    else:
        print(f"NOTE  S is essentially diagonal (max off-diagonal {offdiag:.3f})"
              " - negligible soft")
        print("      iron. The correction is a pure per-axis gain.")

    if not 0.8 < gain < 1.25:
        print(f"NOTE  mean gain {gain:.2f}x - the sensor reads "
              f"{100.0/gain:.0f}% of true scale.")
        print("      Harmless for attitude (Madgwick normalises the vector), but")
        print(f"      it also multiplies sensor NOISE by {gain:.1f}x, so heading")
        print("      is noisier than a healthy part would give.")
    if aniso > 1.2:
        print(f"NOTE  per-axis sensitivity differs by {aniso:.2f}x "
              f"({diag[0]:.2f}/{diag[1]:.2f}/{diag[2]:.2f}).")
        print("      Compare against previous fits: a value that REPEATS across")
        print("      sessions and environments is a property of the part and is")
        print("      fine. One that moves means something in the frame changed.")

    if abs(cn.mean() - field) > 2.0:
        print(f"NOTE  corrected mean {cn.mean():.1f} vs target {field:.1f} - the")
        print("      fit is scaled to whatever --field you gave. Cosmetic only.")

    if ok:
        print()
        print("PASS  send the MAGSET line above, then reboot and confirm |B|")
        print(f"      holds near {cn.mean():.0f} uT while you rotate the board.")


if __name__ == "__main__":
    main()
