#!/usr/bin/env python3
"""Host test for components/common/alt_ref.c.

    python tools/alt_ref_test.py

WHY IT EXISTS
-------------
alt_ref_replay.py scores the module against real flights, but it PRINTS;
it does not assert. And the replay cannot reach the branch that matters
most: across all four flights replayed so far there were ZERO
re-reference events, so ALT_REF_AGREE_S - the five seconds of sustained
disagreement that the author insisted on, and the load-bearing part of the
whole scheme - has never once fired on real data.

Unlike the vertical filter's terrain discriminator (incident 40), this branch IS
synthetically reachable with no competing paths in the way: hold the ToF
at a fixed range, drift the barometer, and the clock either expires at
5 s or it does not. So it gets a synthetic test, and the fact that the
real logs cannot reach it is the reason rather than an excuse.

Also covers the liveness rule added baro_ok used to gate
only the initial baro0 latch, so a dead barometer coasted silently -
the frozen-cache failure that destroyed the aircraft, inside the module
proposed to replace the filter's altitude.

Compiles the REAL alt_ref.c freestanding, the same way
alt_ref_replay.py does. Set ALT_REF_SRC to a doctored copy for mutation
runs.
"""

import ctypes
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "components", "common")
ALT_REF_SRC = os.environ.get("ALT_REF_SRC",
                             os.path.join(SRC, "alt_ref.c"))

SHIM = r"""
int _fltused = 0;
#include "alt_ref_under_test.c"
#define EXPORT __declspec(dllexport)
EXPORT void ar_init(alt_ref_t *a){ alt_ref_init(a); }
EXPORT void ar_update(alt_ref_t *a, float b, int ok, float tof, float dt){
    alt_ref_update(a, b, ok ? 1 : 0, tof, dt); }
EXPORT int ar_size(void){ return (int)sizeof(alt_ref_t); }
"""


class CAltRef(ctypes.Structure):
    _fields_ = [
        ("z", ctypes.c_float),
        ("baro0", ctypes.c_float),
        ("offset", ctypes.c_float),
        ("disagree_s", ctypes.c_float),
        ("baro_stale_s", ctypes.c_float),
        ("have_baro0", ctypes.c_bool),
        ("have_z", ctypes.c_bool),
        ("tof_live", ctypes.c_bool),
        ("valid", ctypes.c_bool),
    ]


def build(tmp):
    with open(ALT_REF_SRC, encoding="utf-8") as fh:
        src = fh.read()
    with open(os.path.join(tmp, "alt_ref_under_test.c"), "w",
              encoding="utf-8") as fh:
        fh.write(src)
    shim = os.path.join(tmp, "shim.c")
    with open(shim, "w") as fh:
        fh.write(SHIM)
    dll = os.path.join(tmp, "ar.dll")
    cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry", "-O1",
           "-fno-math-errno", "-I", tmp, "-I", SRC, shim, "-o", dll]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        sys.exit("clang failed - see above.")
    return dll


DT = 1.0 / 416.0        # the rate alt_ref actually runs at in fusion.c


class Rig:
    def __init__(self, lib):
        self.lib = lib
        self.a = CAltRef()
        lib.ar_init(ctypes.byref(self.a))

    def run(self, seconds, baro, tof, baro_ok=lambda t: True, t0=0.0):
        t = t0
        end = t0 + seconds
        while t < end:
            self.lib.ar_update(ctypes.byref(self.a),
                               ctypes.c_float(baro(t)),
                               1 if baro_ok(t) else 0,
                               ctypes.c_float(tof(t)),
                               ctypes.c_float(DT))
            t += DT
        return t


def main():
    n = bad = 0

    def ok(cond, what, detail=""):
        nonlocal n, bad
        n += 1
        if cond:
            print("  [PASS] %-46s %s" % (what, detail))
        else:
            bad += 1
            print("  [FAIL] %-46s %s" % (what, detail))

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        lib = ctypes.CDLL(build(tmp))
        lib.ar_size.restype = ctypes.c_int
        got, want = lib.ar_size(), ctypes.sizeof(CAltRef)
        if got != want:
            sys.exit("struct layout mismatch: C says %d, ctypes says %d - "
                     "fix CAltRef to mirror alt_ref.h." % (got, want))
        lib.ar_update.argtypes = [ctypes.POINTER(CAltRef), ctypes.c_float,
                                  ctypes.c_int, ctypes.c_float,
                                  ctypes.c_float]

        print("alt_ref.c, real C, compiled freestanding\n")

        # ---- 1. the ToF is the altitude while it can see the ground ---
        r = Rig(lib)
        r.run(3.0, baro=lambda t: 100.0, tof=lambda t: 1.0)
        ok(abs(r.a.z - 1.0) < 0.01, "ToF in range: z tracks the ToF",
           "z = %.3f" % r.a.z)
        ok(r.a.valid, "and it is valid")

        # ---- 2. THE 5 SECOND CLOCK. Never reached on real data. -------
        # Start in AGREEMENT - sitting on the ground, ToF 0.05 m and the
        # barometer freshly latched, so brel = 0. Then drift the
        # barometer up 1 m while the ToF stays where it is: the baro's
        # implied altitude walks away from a floor that has not moved,
        # which is exactly the "genuinely drifted barometer" this branch
        # exists for. The offset must move ONCE, and not before 5 s of
        # SUSTAINED disagreement.
        r = Rig(lib)
        r.run(1.0, baro=lambda t: 100.0, tof=lambda t: 0.05)
        ok(r.a.offset == 0.0, "starts in agreement, no re-reference",
           "offset %+.4f" % r.a.offset)

        # |brel - tof| passes ALT_REF_AGREE_TOL_M (0.30) at t = 1.70 s.
        def drift(t):
            return 100.0 + (0.0 if t < 1.0 else min(1.0, (t - 1.0) / 2.0))
        cross = 1.0 + 2.0 * 0.35      # brel must exceed 0.05 + 0.30
        moves = []
        t = 1.0
        while t < 10.0:
            prev = r.a.offset
            lib.ar_update(ctypes.byref(r.a), ctypes.c_float(drift(t)), 1,
                          ctypes.c_float(0.05), ctypes.c_float(DT))
            if r.a.offset != prev:
                moves.append(t)
            t += DT
        ok(len(moves) == 1, "sustained drift re-references EXACTLY once",
           "%d offset move(s)" % len(moves))
        if moves:
            waited = moves[0] - cross
            ok(4.8 <= waited <= 5.4,
               "after ALT_REF_AGREE_S of disagreement, not before",
               "waited %.2f s (disagreement began t=%.2f, fired t=%.2f)"
               % (waited, cross, moves[0]))
            ok(abs(r.a.z - 0.05) < 0.02,
               "and z is still the ToF, not the drifted baro",
               "z = %.3f" % r.a.z)

        # ---- 3. a PASSING OBJECT must not redefine the ground ---------
        # the author's actual reason for the 5 s. Climb to ~1 m with the baro
        # agreeing, then slide a box under for 2 s and take it away.
        r = Rig(lib)
        r.run(1.0, baro=lambda t: 100.0, tof=lambda t: 0.05)
        r.run(3.0, baro=lambda t: 101.0, tof=lambda t: 1.05, t0=1.0)
        off_before = r.a.offset
        ok(off_before == 0.0, "climbing with a baro that agrees: no move",
           "offset %+.4f" % off_before)
        r.run(2.0, baro=lambda t: 101.0, tof=lambda t: 0.40, t0=4.0)
        r.run(3.0, baro=lambda t: 101.0, tof=lambda t: 1.05, t0=6.0)
        ok(r.a.offset == off_before,
           "a 2 s passing object does NOT re-reference",
           "offset %+.4f, unchanged" % r.a.offset)

        # ---- 4. LIVENESS: a dead baro out of ToF range --------------
        r = Rig(lib)
        r.run(3.0, baro=lambda t: 100.0, tof=lambda t: 1.0)
        ok(r.a.valid, "valid with ToF in range and baro alive")
        # ToF out of range, baro still answering: coasting, but honestly.
        r.run(2.0, baro=lambda t: 100.0, tof=lambda t: -1.0, t0=3.0)
        ok(r.a.valid, "still valid coasting on a LIVE baro",
           "baro_stale %.2f s" % r.a.baro_stale_s)
        # Baro stops answering while still out of ToF range.
        r.run(2.0, baro=lambda t: 100.0, tof=lambda t: -1.0,
              baro_ok=lambda t: False, t0=5.0)
        ok(not r.a.valid,
           "INVALID once the baro stops answering  <-- the regression",
           "baro_stale %.2f s" % r.a.baro_stale_s)
        # The ToF coming back makes it valid again immediately - below
        # 2 m the barometer is irrelevant.
        r.run(0.5, baro=lambda t: 100.0, tof=lambda t: 0.8,
              baro_ok=lambda t: False, t0=7.0)
        ok(r.a.valid, "ToF returning re-validates even with a dead baro",
           "z = %.3f" % r.a.z)

        # ---- 5. the slew is bounded ----------------------------------
        r = Rig(lib)
        r.run(2.0, baro=lambda t: 100.0, tof=lambda t: 0.1)
        z_before = r.a.z
        lib.ar_update(ctypes.byref(r.a), ctypes.c_float(100.0), 1,
                      ctypes.c_float(1.9), ctypes.c_float(DT))
        step = abs(r.a.z - z_before)
        ok(step <= 2.0 * DT + 1e-6,
           "a 1.8 m ToF jump is slewed, not snapped",
           "step %.4f m (limit %.4f = 2 m/s x dt)" % (step, 2.0 * DT))

    print("\n%d checks, %d failed" % (n, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
