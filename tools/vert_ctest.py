#!/usr/bin/env python3
"""Host test: does components/common/vert_filter.c actually match the
design in tools/vert_filter.py?

    python tools/vert_ctest.py                       # the checked-in Log A fixture
    python tools/vert_ctest.py some_other_s12.csv

Compiles the REAL flight source into a DLL, replays a real blackbox log
through both it and the Python reference, and diffs the state histories
sample by sample. A transliteration that "looks right" is how a sign
error survives to a flight; this makes the two provably identical on
25789 frames of real data.

WHY THIS CAN WORK ON THIS MACHINE
---------------------------------
There is no host C runtime here - the only compilers are the xtensa and
riscv cross-compilers plus a clang with no libc headers (the same
constraint that made tools/test_level_rotation.py parse fusion.c rather
than compile it). vert_filter.c is written to need no libc at all: no
math.h, no stdio, only __builtin_sqrtf and __builtin_fabsf. So it links
freestanding, with -nostdlib, and the two symbols Windows still insists
on are supplied by the shim below.

Two flags matter and both are about not needing libm:
  -fno-math-errno   lets __builtin_sqrtf become the sqrtss instruction
                    instead of a call into libm to set errno
  -nostdlib         no CRT, hence the _fltused stub and -Wl,-noentry
"""

import argparse
import ctypes
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "components", "common")

sys.path.insert(0, HERE)
from vert_filter import VertFilter, ACC_SIGMA_IDLE      # noqa: E402
from vert_replay import load, TOF_NO_TARGET_M           # noqa: E402

SHIM = r"""
int _fltused = 0;
#include "vert_filter.c"
#define EXPORT __declspec(dllexport)
EXPORT void vf_init(vert_filter_t *f, float a, float b){ vert_filter_init(f,a,b); }
EXPORT void vf_predict(vert_filter_t *f, float a, float dt){ vert_filter_predict(f,a,dt); }
EXPORT int  vf_baro(vert_filter_t *f, float m){ return vert_filter_update_baro(f,m)?1:0; }
EXPORT int  vf_tof(vert_filter_t *f, float m, float c){ return vert_filter_update_tof(f,m,c)?1:0; }
EXPORT void vf_toflost(vert_filter_t *f){ vert_filter_tof_lost(f); }
EXPORT int  vf_size(void){ return (int)sizeof(vert_filter_t); }
"""


class CVertFilter(ctypes.Structure):
    _fields_ = [
        ("z", ctypes.c_float), ("vz", ctypes.c_float),
        ("ba", ctypes.c_float), ("bb", ctypes.c_float),
        ("P", (ctypes.c_float * 4) * 4),
        ("acc_sigma", ctypes.c_float), ("baro_sigma", ctypes.c_float),
        ("have_baro", ctypes.c_bool), ("have_tof", ctypes.c_bool),
        ("valid", ctypes.c_bool),
        ("tof_reject_streak", ctypes.c_uint16),
        ("baro_reject_streak", ctypes.c_uint16),
        ("dead_reckon_frames", ctypes.c_uint32),
        ("tof_coast_s", ctypes.c_float),
        ("tof_reject_s", ctypes.c_float),
        ("z_at_last_tof", ctypes.c_float),
        ("datum_shift", ctypes.c_float),
        ("baro_reject_lo", ctypes.c_float),
        ("baro_reject_hi", ctypes.c_float),
        # Consistency monitoring and covariance inflation.
        ("baro_reject_s", ctypes.c_float),
        ("baro_inflation", ctypes.c_float),
        # Sensor trust, separate from per-sample plausibility.
        ("baro_trusted", ctypes.c_bool),
        ("baro_distrust_s", ctypes.c_float),
        ("baro_coherent_s", ctypes.c_float),
        ("baro_sample_dt", ctypes.c_float),
        ("baro_dt_acc", ctypes.c_float),
        # Latest raw baro reading; a lost re-acquire re-derives bb
        # from it instead of adjusting bb by dz.
        ("last_baro_m", ctypes.c_float),
    ]


def build(tmp):
    shim = os.path.join(tmp, "vfshim.c")
    with open(shim, "w") as fh:
        fh.write(SHIM)
    dll = os.path.join(tmp, "vf.dll")
    cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry",
           "-O1", "-fno-math-errno", "-I", SRC, shim, "-o", dll]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        sys.exit("clang failed - see above. (Expected on a machine with "
                 "no clang at all; this test needs one, the IDF build "
                 "does not.)")
    return dll


def run_c(dll, rows, acc_sigma):
    lib = ctypes.CDLL(dll)
    lib.vf_size.restype = ctypes.c_int
    got, want = lib.vf_size(), ctypes.sizeof(CVertFilter)
    if got != want:
        sys.exit("struct layout mismatch: C says %d bytes, ctypes says %d. "
                 "Fix CVertFilter in this file to mirror vert_filter.h."
                 % (got, want))

    for fn, args in ((lib.vf_init, [ctypes.POINTER(CVertFilter),
                                    ctypes.c_float, ctypes.c_float]),
                     (lib.vf_predict, [ctypes.POINTER(CVertFilter),
                                       ctypes.c_float, ctypes.c_float]),
                     (lib.vf_baro, [ctypes.POINTER(CVertFilter), ctypes.c_float]),
                     (lib.vf_tof, [ctypes.POINTER(CVertFilter), ctypes.c_float,
                                   ctypes.c_float]),
                     (lib.vf_toflost, [ctypes.POINTER(CVertFilter)])):
        fn.argtypes = args

    f = CVertFilter()
    lib.vf_init(ctypes.byref(f), ctypes.c_float(acc_sigma), ctypes.c_float(0.0))

    out = []
    last_alt = last_tof = None
    prev_t = None
    for r in rows:
        t = int(r["t_us"]) * 1e-6
        dt = 0.004 if prev_t is None else (t - prev_t)
        if dt <= 0.0 or dt > 0.1:
            dt = 0.004
        prev_t = t

        health = int(r["sensor_health"])
        if health & 0x01:
            lib.vf_predict(ctypes.byref(f), ctypes.c_float(float(r["accz_ms2"])),
                           ctypes.c_float(dt))

        alt = float(r["alt_m"])
        if (health & 0x02) and alt != last_alt:
            last_alt = alt
            lib.vf_baro(ctypes.byref(f), ctypes.c_float(alt))

        tof = float(r["tof_m"])
        if tof != last_tof:
            last_tof = tof
            if tof <= 0.0 or tof >= TOF_NO_TARGET_M:
                lib.vf_toflost(ctypes.byref(f))
            else:
                lib.vf_tof(ctypes.byref(f), ctypes.c_float(tof),
                           ctypes.c_float(tilt_cos(r)))
        out.append((f.z, f.vz, f.ba, f.bb, f.datum_shift))

    # Windows keeps a loaded DLL locked, so the temp directory cannot be
    # removed until this handle is dropped.
    if os.name == "nt":
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(lib._handle))
    return out


def tilt_cos(row):
    """The C takes cos(tilt) directly (fusion passes the quaternion's r22).
    The Python reference takes roll/pitch and does the trig itself, so
    this recreates the same number for the C side from the same columns.
    cos(roll)*cos(pitch) is exactly what VertFilter.update_tof computes."""
    import math
    return (math.cos(math.radians(float(row["roll"]))) *
            math.cos(math.radians(float(row["pitch"]))))


def run_py(rows, acc_sigma):
    f = VertFilter(acc_sigma=acc_sigma)
    out = []
    last_alt = last_tof = None
    prev_t = None
    for r in rows:
        t = int(r["t_us"]) * 1e-6
        dt = 0.004 if prev_t is None else (t - prev_t)
        if dt <= 0.0 or dt > 0.1:
            dt = 0.004
        prev_t = t

        health = int(r["sensor_health"])
        if health & 0x01:
            f.predict(float(r["accz_ms2"]), dt)

        alt = float(r["alt_m"])
        if (health & 0x02) and alt != last_alt:
            last_alt = alt
            f.update_baro(alt)

        tof = float(r["tof_m"])
        if tof != last_tof:
            last_tof = tof
            if tof <= 0.0 or tof >= TOF_NO_TARGET_M:
                f.tof_lost()
            else:
                f.update_tof(tof, float(r["roll"]), float(r["pitch"]))
        out.append((f.z, f.vz, f.ba, f.bb, f.datum_shift))
    return out


def main():
    ap = argparse.ArgumentParser()
    # Default to the fixture that is IN the repo. This used to point at a loose
    # decode copy at the repo root, which meant the suite only ran on a machine
    # that happened to have one - never on a clean checkout, never in CI.
    ap.add_argument("csv", nargs="?",
                    default=os.path.join(ROOT, "tools", "fixtures", "logA_s23.csv.gz"))
    # The C runs in float and the Python in double, so they diverge by
    # rounding alone. 5 mm over 100 s of a filter that integrates
    # acceleration is rounding; 5 cm is a transcription error.
    ap.add_argument("--tol-z", type=float, default=0.005)
    ap.add_argument("--tol-v", type=float, default=0.005)
    args = ap.parse_args()

    rows = load(args.csv)
    with tempfile.TemporaryDirectory() as tmp:
        dll = build(tmp)
        c = run_c(dll, rows, ACC_SIGMA_IDLE)
    p = run_py(rows, ACC_SIGMA_IDLE)

    names = ("altitude", "climb rate", "accel bias", "baro bias",
             "datum shift")
    tols = (args.tol_z, args.tol_v, args.tol_z, args.tol_z, args.tol_z)
    print("C (components/common/vert_filter.c) vs Python reference, "
          "%d frames of %s\n" % (len(rows), os.path.basename(args.csv)))
    ok = True
    for i, (name, tol) in enumerate(zip(names, tols)):
        worst = max(abs(a[i] - b[i]) for a, b in zip(c, p))
        good = worst <= tol
        print("  [%s] %-12s max |C - Python| = %.3e   (tol %.0e)"
              % ("PASS" if good else "FAIL", name, worst, tol))
        ok &= good
    print("\n  C final state:      z %+.4f  vz %+.4f  ba %+.4f  bb %+.4f"
          % c[-1][:4])
    print("  Python final state: z %+.4f  vz %+.4f  ba %+.4f  bb %+.4f"
          % p[-1][:4])
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
