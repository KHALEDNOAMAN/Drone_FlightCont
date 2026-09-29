#!/usr/bin/env python3
"""Compile the portable flight C in components/common and load it into
Python, so the code that actually flies can be tested on the host.

There is no host C runtime on this machine - only the xtensa and riscv
cross-compilers and a clang with no libc headers. vert_filter.c and
alt_hold.c are written to need no libc at all (no math.h, only
__builtin_* intrinsics), which is exactly what makes this possible:
they link freestanding with -nostdlib.

Two flags carry the trick:
  -fno-math-errno   lets __builtin_sqrtf compile to the sqrtss
                    instruction instead of a call into libm to set errno
  -nostdlib         no CRT, hence the _fltused stub and -Wl,-noentry

Used by tools/vert_ctest.py, tools/vert_gating_test.py and
tools/alt_hold_sim.py.
"""

import ctypes
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
# HOST_C_SRC points the compile at a DOCTORED copy of components/common,
# for mutation runs. Same convention as PARAMS_SRC / UBX_SRC / GEO_SRC /
# MS5611_SRC, and it exists for the same reason: without it a mutation
# harness silently tests the real source and every mutant "survives".
SRC = os.environ.get("HOST_C_SRC") or os.path.join(ROOT, "components",
                                                   "common")

SHIM = r"""
int _fltused = 0;
#include "vert_filter.c"
#include "alt_hold.c"
#define EXPORT __declspec(dllexport)

EXPORT void vf_init(vert_filter_t *f, float a, float b){ vert_filter_init(f,a,b); }
EXPORT void vf_predict(vert_filter_t *f, float a, float dt){ vert_filter_predict(f,a,dt); }
EXPORT int  vf_baro(vert_filter_t *f, float m){ return vert_filter_update_baro(f,m)?1:0; }
EXPORT int  vf_tof(vert_filter_t *f, float m, float c){ return vert_filter_update_tof(f,m,c)?1:0; }
EXPORT void vf_toflost(vert_filter_t *f){ vert_filter_tof_lost(f); }
EXPORT float vf_takeshift(vert_filter_t *f){ return vert_filter_take_datum_shift(f); }
EXPORT int  vf_size(void){ return (int)sizeof(vert_filter_t); }

EXPORT void ah_init(alt_hold_state_t *s){ alt_hold_init(s); }
EXPORT void ah_defcfg(alt_hold_cfg_t *c){ alt_hold_default_cfg(c); }
EXPORT float ah_update(alt_hold_state_t *s, const alt_hold_cfg_t *c,
                       int want, unsigned short stick, float z, float vz,
                       float shift, float vbat, float dt){
    return alt_hold_update(s, c, want ? 1 : 0, stick, z, vz, shift, vbat, dt);
}
EXPORT int  ah_state_size(void){ return (int)sizeof(alt_hold_state_t); }
EXPORT int  ah_cfg_size(void){ return (int)sizeof(alt_hold_cfg_t); }

EXPORT float ah_vscale(float v_ref, float vbat){
    return alt_hold_vscale(v_ref, vbat);
}
EXPORT float ah_vcomp(float stick_us, float v_ref, float vbat){
    return thr_vcomp_us(stick_us, v_ref, vbat);
}
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


class CAltHoldCfg(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ("hover_thr", "v_ref", "vz_max", "kp_z", "kp_vz", "ki_vz",
                 "i_max", "thr_deadband", "exit_ramp_s")]


class CAltHoldState(ctypes.Structure):
    _fields_ = [
        ("engaged", ctypes.c_bool),
        ("z_sp", ctypes.c_float), ("integ", ctypes.c_float),
        ("out_us", ctypes.c_float), ("exit_ramp", ctypes.c_float),
        ("exit_from_us", ctypes.c_float), ("thr_neutral", ctypes.c_float),
    ]


class HostC:
    """Owns the temp dir and the loaded DLL. Use as a context manager -
    Windows keeps a loaded DLL locked, so the handle must be freed before
    the directory can go away."""

    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        shim = os.path.join(self._tmp.name, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(self._tmp.name, "fc.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry",
               "-O1", "-fno-math-errno", "-I", SRC, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit(
                "clang failed - see above. This host test needs clang on "
                "PATH; the ESP-IDF build does not.")
        self.lib = ctypes.CDLL(dll)
        self._decl()
        self._check_layout()

    def _decl(self):
        L = self.lib
        pf = ctypes.POINTER(CVertFilter)
        ps = ctypes.POINTER(CAltHoldState)
        pc = ctypes.POINTER(CAltHoldCfg)
        f = ctypes.c_float
        L.vf_init.argtypes = [pf, f, f]
        L.vf_predict.argtypes = [pf, f, f]
        L.vf_baro.argtypes = [pf, f]
        L.vf_tof.argtypes = [pf, f, f]
        L.vf_toflost.argtypes = [pf]
        L.vf_takeshift.argtypes = [pf]
        L.vf_takeshift.restype = f
        L.vf_size.restype = ctypes.c_int
        L.ah_init.argtypes = [ps]
        L.ah_defcfg.argtypes = [pc]
        L.ah_update.argtypes = [ps, pc, ctypes.c_int, ctypes.c_uint16,
                                f, f, f, f, f]
        L.ah_update.restype = f
        L.ah_state_size.restype = ctypes.c_int
        L.ah_cfg_size.restype = ctypes.c_int
        L.ah_vscale.argtypes = [f, f]
        L.ah_vscale.restype = f
        L.ah_vcomp.argtypes = [f, f, f]
        L.ah_vcomp.restype = f

    def _check_layout(self):
        """A silently mismatched struct layout would make every number in
        every test wrong in a way that looks like a tuning problem."""
        for name, got, want in (
                ("vert_filter_t", self.lib.vf_size(), ctypes.sizeof(CVertFilter)),
                ("alt_hold_state_t", self.lib.ah_state_size(),
                 ctypes.sizeof(CAltHoldState)),
                ("alt_hold_cfg_t", self.lib.ah_cfg_size(),
                 ctypes.sizeof(CAltHoldCfg))):
            if got != want:
                raise SystemExit(
                    "%s layout mismatch: C says %d bytes, ctypes says %d. "
                    "Update the mirror in tools/host_c.py." % (name, got, want))

    def close(self):
        if os.name == "nt":
            ctypes.windll.kernel32.FreeLibrary(
                ctypes.c_void_p(self.lib._handle))
        self._tmp.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
        return False


if __name__ == "__main__":
    with HostC() as h:
        print("built and loaded OK")
        print("  vert_filter_t   %d B" % h.lib.vf_size())
        print("  alt_hold_state_t %d B" % h.lib.ah_state_size())
        print("  alt_hold_cfg_t   %d B" % h.lib.ah_cfg_size())
    sys.exit(0)
