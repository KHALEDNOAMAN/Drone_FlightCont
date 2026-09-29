#!/usr/bin/env python3
"""Regression: does the vertical filter still sawtooth on the real logs
that made it obvious?

    python tools/vert_runaway_test.py

WHAT HAPPENED
-------------
s179 and s180  were flown at a low hover with the ToF live on
99-100% of frames. The estimator reached +14.07 m and +31.62 m while the
ToF read ~0.10 m throughout, and the recorded trace is a clean ~1.6 Hz
sawtooth: z ramps away at whatever vz says, the runaway detector fires,
vf_redatum snaps z back onto the ToF, and the phantom velocity throws it
straight out again. 47 recoveries in 29.8 s on s179, 51 in 30.3 s on
s180, with vz sitting at +32.18 m/s across every one of them.

The recovery was correct and useless, because it reset the SYMPTOM (z)
and left the CAUSE (vz, and the accel bias generating it) untouched.

WHY A REPLAY AND NOT A SYNTHETIC
--------------------------------
incident 40, applied. The synthetic scenarios in vert_gating_test.py reach the
gating branches but cannot reproduce this: the thing under test is a
real accel bias interacting with a real 5 Hz ToF over half a minute, and
three attempts at synthesising the neighbouring terrain discriminator
each failed differently. These two flights ARE the fixture.

They are gitignored (*_s*.csv), so this SKIPS cleanly when they are
absent - same contract as setpoint_safety_test.py. Do not "fix" that.
"""

import ctypes
import glob
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

sys.path.insert(0, HERE)
from vert_ctest import CVertFilter, build                # noqa: E402
from vert_replay import load, TOF_NO_TARGET_M            # noqa: E402

# Measured on the shipped firmware, before the fix. These are the numbers
# the change has to move, quoted so a future reader can tell whether this
# test still means anything.
BASELINE = {
    "s179": {"edges": 47, "worst": 13.96},
    "s180": {"edges": 51, "worst": 31.33},
}
# A recovery that fires more than a couple of times a flight is the
# sawtooth by another name.
MAX_EDGES = 5
# The ToF is ground truth below 2 m. A metre is generous; the sawtooth
# was thirty.
MAX_ERR_M = 1.0
MAX_BAD_S = 0.75
# The filter is allowed to be wrong while it is SAYING it is wrong -
# valid=False means alt-hold has been handed back and nothing is acting
# on the number. Bounded anyway, because unbounded dead reckoning is how
# the estimate reached +31.6 m in the first place.
MAX_EXCURSION_M = 3.0
# How long the estimate may exceed MAX_ERR_M while still claiming valid.
# s180 measures 0.32 s in a 30 s flight, all of it in one window at
# t = 28.8-29.1 s during a recovery; 98.9% of its valid frames are
# within 1 m. This is a RATCHET at roughly twice the measured value, not
# a target - the goal is zero, and it is the number to push on next.


def find(stem):
    """Decode output lives under captures/blackbox/ (moved there).
    The repo root is still searched so a fresh decode dropped there is
    picked up without ceremony."""
    hits = []
    for d in (os.path.join(ROOT, "captures", "blackbox"), ROOT):
        hits += sorted(glob.glob(os.path.join(d, "*_%s.csv" % stem)))
    return hits[-1] if hits else None


def replay(lib, rows):
    """Drive the real C with the logged sensors. The ToF is fed only when
    the CACHED value CHANGES - it is repeated across ~50 frames at 250 Hz
    and feeding it every frame would let the filter mistake one
    measurement for fifty."""
    f = CVertFilter()
    lib.vf_init(ctypes.byref(f), ctypes.c_float(2.0), ctypes.c_float(0.0))

    t0 = float(rows[0]["t_us"]) / 1e6
    prev_t = None
    prev_tof = None
    prev_dv = None
    prev_valid = False
    edges = 0
    worst = 0.0
    worst_valid = 0.0
    bad_s = 0.0
    n = 0
    have_dv = ("accz_dv_ms" in rows[0]
               and any(float(r["accz_dv_ms"]) != 0.0 for r in rows))

    for r in rows:
        t = float(r["t_us"]) / 1e6 - t0
        dt = 0.0 if prev_t is None else t - prev_t
        prev_t = t
        if not (0.0 < dt < 0.05):
            continue

        # The integrated dv is the honest input: accz_cms2 is ONE 250 Hz
        # sample of a signal the filter consumes at 416 Hz, and hover
        # accz carries several m/s^2 of noise.
        if have_dv:
            dv = float(r["accz_dv_ms"])
            accz = 0.0 if prev_dv is None else (dv - prev_dv) / dt
            prev_dv = dv
        else:
            accz = float(r["accz_ms2"])
        lib.vf_predict(ctypes.byref(f), ctypes.c_float(accz),
                       ctypes.c_float(dt))

        tof = float(r["tof_m"])
        live = 0.0 < tof < TOF_NO_TARGET_M
        if live and tof != prev_tof:
            lib.vf_tof(ctypes.byref(f), ctypes.c_float(tof),
                       ctypes.c_float(1.0))
            prev_tof = tof
        elif not live:
            lib.vf_toflost(ctypes.byref(f))
            prev_tof = None

        if bool(f.valid) and not prev_valid:
            edges += 1
        prev_valid = bool(f.valid)

        # Score only where there is ground truth and the aircraft is
        # actually flying - the project's is explicit that ground/idle frames
        # produce numbers indistinguishable from a real excursion.
        if live and int(r["throttle"]) > 1100:
            err = abs(f.z - tof)
            worst = max(worst, err)
            # Error WHILE THE FILTER CLAIMS TO BE RIGHT is the number a
            # controller would act on. When valid is false alt-hold has
            # already been handed back, so a large error there is
            # bounded dead-reckoning that nothing consumes - undesirable
            # but not dangerous. s180's residual 1.62 m excursion sits
            # entirely inside a valid=False window, which is the filter
            # behaving correctly, and asserting on the raw maximum would
            # have punished exactly that.
            if bool(f.valid):
                worst_valid = max(worst_valid, err)
                if err > MAX_ERR_M:
                    bad_s += dt
            n += 1

    return edges, worst, worst_valid, bad_s, n


def main():
    fails = 0
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        lib = ctypes.CDLL(build(tmp))
        lib.vf_size.restype = ctypes.c_int
        got, want = lib.vf_size(), ctypes.sizeof(CVertFilter)
        if got != want:
            sys.exit("struct layout mismatch: C says %d, ctypes says %d - "
                     "fix CVertFilter in tools/vert_ctest.py" % (got, want))

        print("Replaying the two sawtooth flights through the REAL C.\n")
        ran = 0
        for stem, base in sorted(BASELINE.items()):
            path = find(stem)
            if path is None:
                print("  [SKIP] %s - log not on disk (gitignored)" % stem)
                continue
            ran += 1
            edges, worst, worst_valid, bad_s, n = replay(lib, load(path))
            print("  %s  (%s)" % (stem, os.path.basename(path)))
            print("    recoveries      %3d   was %3d"
                  % (edges, base["edges"]))
            print("    worst |z-tof| %6.2f m was %6.2f m   (%d scored "
                  "frames)" % (worst, base["worst"], n))
            print("      of which, while VALID  %6.2f m, and %.2f s "
                  "spent above %.1f m" % (worst_valid, bad_s, MAX_ERR_M))
            for label, ok, detail in (
                ("the sawtooth is gone", edges <= MAX_EDGES,
                 "%d recoveries (limit %d, baseline %d)"
                 % (edges, MAX_EDGES, base["edges"])),
                # A single brief transient during a recovery is not the
                # same failure as a sustained lie, and only the second
                # one can fly an aircraft into something. s180 peaks at
                # 1.55 m while valid but spends 0.32 s above 1 m out of
                # 30 - measured, and the reason this asserts on DURATION
                # rather than on the maximum alone.
                ("not wrong for LONG while claiming to be right",
                 bad_s <= MAX_BAD_S,
                 "%.2f s above %.1f m while valid (limit %.1f s), peak "
                 "%.2f m" % (bad_s, MAX_ERR_M, MAX_BAD_S, worst_valid)),
                ("and the excursion itself is bounded",
                 worst <= MAX_EXCURSION_M,
                 "worst %.2f m overall (limit %.1f, baseline %.2f)"
                 % (worst, MAX_EXCURSION_M, base["worst"])),
            ):
                print("    [%s] %-34s %s"
                      % ("PASS" if ok else "FAIL", label, detail))
                if not ok:
                    fails += 1
            print()

        if not ran:
            print("No fixture logs on disk - nothing asserted. This is the "
                  "documented skip, not a pass.")
            return 0

    print("OK" if not fails else "%d FAILURE(S)" % fails)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
