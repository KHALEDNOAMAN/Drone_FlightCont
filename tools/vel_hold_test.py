#!/usr/bin/env python3
"""Does velocity hold command the RIGHT WAY? Tested against a real walk.

    python tools/vel_hold_test.py [log]   (default tools/fixtures/s164_walk.csv.gz)

WHY THIS TEST, AND WHY IT IS NOT A UNIT TEST
--------------------------------------------
The two sign constants in vel_hold.c decide whether this loop brakes or
accelerates. Get one wrong and the aircraft does not wobble - it leaves,
faster and faster, because the controller is now positive feedback on its
own drift. That is the single most dangerous thing in the file and it
cannot be checked by reading it: this project has been wrong about a
control sign before (the yaw stick mapping, backwards for weeks) and
reasoning lost to measurement every time.

So the test uses REAL data with a KNOWN truth. s164 is the author walking a
GPS-logged path carrying the board. Velocity hold asks for zero ground
speed, so at every instant where he was measurably moving, the controller
must command a tilt that would DECELERATE him. Feed the log's real
velocity and real heading in, and check the sign of what comes out.

WHAT "CORRECT" LOOKS LIKE
------------------------
Walking forward (relative to where the board is pointing) must command
nose UP - negative pitch, because +pitch is nose down on this airframe.
Walking left must command roll RIGHT - positive roll, because +roll lifts
the left side and pushes the aircraft right.

Equivalently and more robustly: the commanded acceleration vector must
point OPPOSITE the measured velocity. That is one dot product, it does
not care about the sign conventions being right in any particular
notation, and it is what actually keeps an aircraft from flying away.

This is a FLIGHT-SAFETY gate, not a nicety. It runs on the C, through
tools/host_c.py, because the C is what flies.
"""

import argparse
import ctypes
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "components", "common")

sys.path.insert(0, HERE)
from vert_replay import load                              # noqa: E402

SHIM = r"""
int _fltused = 0;
#include "vel_hold.c"
#define EXPORT __declspec(dllexport)
EXPORT void vh_init(vel_hold_state_t *s){ vel_hold_init(s); }
EXPORT void vh_defcfg(vel_hold_cfg_t *c){ vel_hold_default_cfg(c); }
EXPORT int vh_update(vel_hold_state_t *s, const vel_hold_cfg_t *c,
                     int want, short sr, short sp, float vn, float ve,
                     float cy, float sy, int ok, float dt,
                     float *dr, float *dp){
    return vel_hold_update(s,c,want?1:0,sr,sp,vn,ve,cy,sy,ok?1:0,dt,dr,dp)?1:0;
}
EXPORT int vh_state_size(void){ return (int)sizeof(vel_hold_state_t); }
EXPORT int vh_cfg_size(void){ return (int)sizeof(vel_hold_cfg_t); }
"""


class CCfg(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ("kp", "ki", "i_max", "tilt_max", "vel_max")]


class CState(ctypes.Structure):
    _fields_ = [("engaged", ctypes.c_bool),
                ("integ_fwd", ctypes.c_float), ("integ_left", ctypes.c_float),
                ("out_roll", ctypes.c_float), ("out_pitch", ctypes.c_float)]


def build(tmp):
    shim = os.path.join(tmp, "vhshim.c")
    with open(shim, "w") as fh:
        fh.write(SHIM)
    dll = os.path.join(tmp, "vh.dll")
    r = subprocess.run(["clang", "-shared", "-nostdlib", "-Wl,-noentry",
                        "-O1", "-fno-math-errno", "-I", SRC, shim, "-o", dll],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        sys.exit("clang failed - see above.")
    return dll


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?",
                    default=os.path.join("tools", "fixtures", "s164_walk.csv.gz"))
    args = ap.parse_args()
    path = args.log
    if not os.path.isabs(path) and not os.path.exists(path):
        path = os.path.join(ROOT, args.log)
    rows = load(path)

    tmp = tempfile.mkdtemp()
    lib = ctypes.CDLL(build(tmp))
    ps, pc, pf = (ctypes.POINTER(CState), ctypes.POINTER(CCfg),
                  ctypes.POINTER(ctypes.c_float))
    lib.vh_init.argtypes = [ps]
    lib.vh_defcfg.argtypes = [pc]
    lib.vh_update.argtypes = [ps, pc, ctypes.c_int, ctypes.c_short,
                              ctypes.c_short, ctypes.c_float, ctypes.c_float,
                              ctypes.c_float, ctypes.c_float, ctypes.c_int,
                              ctypes.c_float, pf, pf]
    lib.vh_update.restype = ctypes.c_int
    lib.vh_state_size.restype = ctypes.c_int
    lib.vh_cfg_size.restype = ctypes.c_int
    for nm, got, want in (("vel_hold_state_t", lib.vh_state_size(),
                           ctypes.sizeof(CState)),
                          ("vel_hold_cfg_t", lib.vh_cfg_size(),
                           ctypes.sizeof(CCfg))):
        if got != want:
            sys.exit("%s layout mismatch: C %d, ctypes %d" % (nm, got, want))

    st, cfg = CState(), CCfg()
    lib.vh_init(ctypes.byref(st))
    lib.vh_defcfg(ctypes.byref(cfg))
    # KI IS ZEROED FOR THIS TEST, deliberately, and the reason matters.
    #
    # This is an OPEN-LOOP replay: the controller commands a braking
    # tilt, but the log is a recording and the author keeps walking, so the
    # error never goes away. In a closed loop the aircraft would slow and
    # the error would vanish; here the integrator sees a permanent error
    # and winds to its limit, after which the command is dominated by a
    # constant trim rather than by the current velocity. Measured with
    # ki = 1.0 that dragged the mean braking cosine from -1.0 to -0.893
    # and pinned the tilt at the 12 deg clamp.
    #
    # That is the REPLAY being unphysical, not the controller being
    # wrong, and it would mask the thing this test exists to catch. The
    # proportional path is what encodes the signs, so the signs are what
    # is tested. The integrator's behaviour cannot be judged without a
    # plant that responds, and pretending otherwise would be a test that
    # passes for the wrong reason.
    cfg.ki = 0.0
    dr, dp = ctypes.c_float(0.0), ctypes.c_float(0.0)

    t0 = float(rows[0]["t_us"]) / 1e6
    prev = None
    samples = []          # (speed, brake_dot, pitch, roll, v_fwd, v_left)
    for r in rows:
        t = float(r["t_us"]) / 1e6 - t0
        if prev is None:
            prev = t
            continue
        dt = t - prev
        prev = t
        if not (0.0 < dt < 0.05):
            continue
        vn = float(r["vel_n_ms"])
        ve = float(r["vel_e_ms"])
        yaw = float(r["yaw"])
        ok = bool(int(r["gps_flags"]) & 0x08)
        # Sticks CENTRED: the whole point is that this means zero speed.
        yr_ = math.radians(yaw)
        lib.vh_update(ctypes.byref(st), ctypes.byref(cfg), 1, 0, 0,
                      ctypes.c_float(vn), ctypes.c_float(ve),
                      ctypes.c_float(math.cos(yr_)),
                      ctypes.c_float(math.sin(yr_)), 1 if ok else 0,
                      ctypes.c_float(dt), ctypes.byref(dr), ctypes.byref(dp))
        sp = math.hypot(vn, ve)
        if sp < 0.8 or not ok:
            continue
        yr = math.radians(yaw)
        v_fwd = vn * math.cos(yr) + ve * math.sin(yr)
        v_left = vn * math.sin(yr) - ve * math.cos(yr)
        # Commanded acceleration in the body frame, from the commanded
        # tilt: +pitch (nose down) accelerates FORWARD, +roll (left side
        # up) accelerates RIGHT i.e. -left.
        a_fwd = dp.value
        a_left = -dr.value
        # Braking means the commanded acceleration opposes the velocity.
        dot = a_fwd * v_fwd + a_left * v_left
        norm = math.hypot(a_fwd, a_left) * math.hypot(v_fwd, v_left)
        samples.append((sp, dot / norm if norm > 1e-9 else 0.0,
                        dp.value, dr.value, v_fwd, v_left))

    if not samples:
        sys.exit("no moving samples in this log - nothing to test")

    fail = 0
    n = len(samples)
    braking = sum(1 for s in samples if s[1] < 0.0)
    frac = braking / n
    mean_dot = sum(s[1] for s in samples) / n

    print("=== %s ===" % os.path.basename(path))
    print("moving samples (>0.8 m/s, fix ok): %d\n" % n)

    ok1 = frac > 0.95
    fail += 0 if ok1 else 1
    print("  [%s] commanded tilt OPPOSES the motion  %.1f%% of samples "
          "(want >95%%)" % ("PASS" if ok1 else "FAIL", 100 * frac))
    ok2 = mean_dot < -0.9
    fail += 0 if ok2 else 1
    print("  [%s] and squarely opposite, not sideways  mean cos = %+.3f "
          "(want < -0.9; -1 is perfect braking)"
          % ("PASS" if ok2 else "FAIL", mean_dot))

    # The two axes separately, so a single flipped sign is named rather
    # than hidden inside an aggregate that the other axis rescues.
    fw = [s for s in samples if abs(s[4]) > 0.5]
    lf = [s for s in samples if abs(s[5]) > 0.5]
    if fw:
        bad = sum(1 for s in fw if s[2] * s[4] > 0)
        ok3 = bad < 0.05 * len(fw)
        fail += 0 if ok3 else 1
        print("  [%s] PITCH sign: moving forward commands nose UP  "
              "(%d of %d wrong)"
              % ("PASS" if ok3 else "FAIL", bad, len(fw)))
    if lf:
        bad = sum(1 for s in lf if s[3] * s[5] < 0)
        ok4 = bad < 0.05 * len(lf)
        fail += 0 if ok4 else 1
        print("  [%s] ROLL sign: moving left commands roll RIGHT  "
              "(%d of %d wrong)"
              % ("PASS" if ok4 else "FAIL", bad, len(lf)))

    mx = max(max(abs(s[2]), abs(s[3])) for s in samples)
    ok5 = mx <= cfg.tilt_max + 1e-3
    fail += 0 if ok5 else 1
    print("  [%s] tilt stayed inside the %.0f deg safety bound  (max %.2f)"
          % ("PASS" if ok5 else "FAIL", cfg.tilt_max, mx))

    # ---- SYNTHETIC: the rotation, at headings the log never visited ----
    #
    # THE GAP THIS CLOSES, found by mutation M3 . Corrupting
    # the body-frame rotation to `v_left = vn*sin + ve*cos` (the sign of
    # the second term flipped) passed every check above. Not because the
    # checks are weak, but because of the DATA: the author held the board at
    # yaw ~88 deg for the whole walk, so cos_yaw ~ 0.035 and the term the
    # mutation corrupts is multiplied by almost nothing. A single-heading
    # dataset cannot exercise a rotation, however much of it there is.
    #
    # So the rotation is tested where the log cannot: drive a known
    # velocity due NORTH at eight headings and require the commanded tilt
    # to oppose it at every one. At yaw 0 that is a pure pitch response,
    # at yaw 90 a pure roll response, and in between it must split
    # between them correctly - which is exactly what a broken rotation
    # gets wrong.
    print("")
    print("  synthetic rotation check (headings the log never visited):")
    rot_fail = 0
    for hdg in (0, 45, 90, 135, 180, 225, 270, 315):
        st2, dr2, dp2 = CState(), ctypes.c_float(0.0), ctypes.c_float(0.0)
        lib.vh_init(ctypes.byref(st2))
        yr = math.radians(hdg)
        # 1 m/s at bearing 53 deg - BOTH components non-zero, which the
        # first version of this check got wrong. Driving it due north
        # left ve = 0, so the very term mutation M3 corrupts (ve*cos_yaw)
        # was multiplied by zero and the mutant survived a second time.
        # A rotation test needs the input to have something in every axis
        # it rotates.
        VN, VE = 0.6, 0.8
        for _ in range(20):
            lib.vh_update(ctypes.byref(st2), ctypes.byref(cfg), 1, 0, 0,
                          ctypes.c_float(VN), ctypes.c_float(VE),
                          ctypes.c_float(math.cos(yr)),
                          ctypes.c_float(math.sin(yr)), 1,
                          ctypes.c_float(0.01),
                          ctypes.byref(dr2), ctypes.byref(dp2))
        v_fwd = VN * math.cos(yr) + VE * math.sin(yr)
        v_left = VN * math.sin(yr) - VE * math.cos(yr)
        a_fwd, a_left = dp2.value, -dr2.value
        dot = a_fwd * v_fwd + a_left * v_left
        norm = math.hypot(a_fwd, a_left) * math.hypot(v_fwd, v_left)
        cosang = dot / norm if norm > 1e-9 else 0.0
        ok = cosang < -0.99
        rot_fail += 0 if ok else 1
        print("    [%s] heading %3d deg  pitch %+6.2f roll %+6.2f  "
              "cos %+.3f" % ("PASS" if ok else "FAIL", hdg,
                             dp2.value, dr2.value, cosang))
    fail += rot_fail

    if os.name == "nt":
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(lib._handle))
    print("\n%s" % ("ALL PASS" if not fail else "%d FAILURE(S)" % fail))
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
