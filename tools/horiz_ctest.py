#!/usr/bin/env python3
"""Host test: does components/common/horiz_filter.c match the design in
tools/horiz_filter.py?

    python tools/horiz_ctest.py [log]   (default tools/fixtures/s164_walk.csv.gz)

The same idea, the same machinery and the same justification as
tools/vert_ctest.py: compile the REAL flight source into a DLL, replay a
real blackbox log through both it and the Python reference, and diff the
state histories sample by sample. A transliteration that "looks right" is
how a sign error survives to a flight.

s164 is the right log to default to: 247 s, 1225 GPS fixes, HOME_VALID on
every frame, and validated earth-frame horizontal acceleration - so both
the prediction and both corrections are exercised on real data rather
than on a synthetic that could flatter either implementation.
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
from horiz_filter import HorizFilter                      # noqa: E402
from vert_replay import load                              # noqa: E402

SHIM = r"""
int _fltused = 0;

/* A host-link stub, the same category as _fltused above and NOT a
 * relaxation of the no-libc rule the estimator is written to.
 *
 * horiz_filter.c copies 6x6 float matrices with ordinary nested loops.
 * Clang recognises those as block copies and emits a call to memcpy,
 * which does not exist under -nostdlib - so the freestanding link fails
 * even though the source contains no library call at all. On the
 * aircraft this is a non-issue: the IDF build links a real libc and
 * memcpy is a compiler builtin on every target this will see. The
 * property that actually matters, and that vert_filter.c states, is no
 * math.h and hence no libm; that still holds.
 *
 * vert_filter.c never hit this because its matrices are 4x4 and small
 * enough for clang to copy inline. */
typedef __SIZE_TYPE__ hf_size_t;
void *memcpy(void *d, const void *s, hf_size_t n)
{
    unsigned char *dd = (unsigned char *)d;
    const unsigned char *ss = (const unsigned char *)s;
    while (n--) *dd++ = *ss++;
    return d;
}

#include "horiz_filter.c"
#define EXPORT __declspec(dllexport)
EXPORT void hf_init(horiz_filter_t *f, float a){ horiz_filter_init(f,a); }
EXPORT void hf_predict(horiz_filter_t *f, float an, float ae, float dt){
    horiz_filter_predict(f,an,ae,dt); }
EXPORT int hf_pos(horiz_filter_t *f, float n, float e, float h){
    return horiz_filter_update_gps_pos(f,n,e,h)?1:0; }
EXPORT int hf_vel(horiz_filter_t *f, float n, float e, float s){
    return horiz_filter_update_gps_vel(f,n,e,s)?1:0; }
EXPORT int hf_size(void){ return (int)sizeof(horiz_filter_t); }
"""


class CHorizFilter(ctypes.Structure):
    _fields_ = [
        ("pn", ctypes.c_float), ("pe", ctypes.c_float),
        ("vn", ctypes.c_float), ("ve", ctypes.c_float),
        ("bn", ctypes.c_float), ("be", ctypes.c_float),
        ("P", (ctypes.c_float * 6) * 6),
        ("acc_sigma", ctypes.c_float),
        ("have_fix", ctypes.c_bool),
        ("n_pos_accept", ctypes.c_uint32), ("n_pos_reject", ctypes.c_uint32),
        ("n_vel_accept", ctypes.c_uint32), ("n_vel_reject", ctypes.c_uint32),
    ]


def build(tmp):
    shim = os.path.join(tmp, "hfshim.c")
    with open(shim, "w") as fh:
        fh.write(SHIM)
    dll = os.path.join(tmp, "hf.dll")
    cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry",
           "-O1", "-fno-math-errno", "-I", SRC, shim, "-o", dll]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        sys.exit("clang failed - see above. This host test needs clang on "
                 "PATH; the ESP-IDF build does not.")
    return dll


def events(rows):
    """The measurement sequence, extracted ONCE and replayed into both
    implementations. Building it separately for each would let a
    difference in the harness masquerade as a difference in the filter."""
    out = []
    t0 = float(rows[0]["t_us"]) / 1e6
    prev_t = None
    last = None
    pdn = pde = None
    have_acc = "accn_dv_ms" in rows[0]
    for r in rows:
        t = float(r["t_us"]) / 1e6 - t0
        if prev_t is not None:
            dt = t - prev_t
            if 0.0 < dt < 0.05:
                an = ae = 0.0
                if have_acc:
                    dn = float(r["accn_dv_ms"])
                    de = float(r["acce_dv_ms"])
                    if pdn is not None:
                        an = (dn - pdn) / dt
                        ae = (de - pde) / dt
                    pdn, pde = dn, de
                out.append(("p", an, ae, dt))
        prev_t = t
        fl = int(r["gps_flags"])
        key = (r["pos_n_m"], r["pos_e_m"], r["vel_n_ms"], r["vel_e_ms"])
        if key != last and (fl & 0x08) and (fl & 0x10):
            last = key
            out.append(("v", float(r["vel_n_ms"]), float(r["vel_e_ms"]), 0.0))
            out.append(("x", float(r["pos_n_m"]), float(r["pos_e_m"]),
                        float(r["h_acc_m"])))
    return out


def run_c(dll, ev):
    lib = ctypes.CDLL(dll)
    lib.hf_size.restype = ctypes.c_int
    got, want = lib.hf_size(), ctypes.sizeof(CHorizFilter)
    if got != want:
        sys.exit("struct layout mismatch: C says %d bytes, ctypes says %d. "
                 "Fix CHorizFilter in this file to mirror horiz_filter.h."
                 % (got, want))
    p = ctypes.POINTER(CHorizFilter)
    f32 = ctypes.c_float
    lib.hf_init.argtypes = [p, f32]
    lib.hf_predict.argtypes = [p, f32, f32, f32]
    lib.hf_pos.argtypes = [p, f32, f32, f32]
    lib.hf_vel.argtypes = [p, f32, f32, f32]

    f = CHorizFilter()
    lib.hf_init(ctypes.byref(f), f32(0.0))
    out = []
    for kind, a, b, c in ev:
        if kind == "p":
            lib.hf_predict(ctypes.byref(f), f32(a), f32(b), f32(c))
        elif kind == "v":
            lib.hf_vel(ctypes.byref(f), f32(a), f32(b), f32(0.0))
        else:
            lib.hf_pos(ctypes.byref(f), f32(a), f32(b), f32(c))
        out.append((f.pn, f.pe, f.vn, f.ve, f.bn, f.be))
    counts = (f.n_pos_accept, f.n_pos_reject, f.n_vel_accept, f.n_vel_reject)
    if os.name == "nt":
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(lib._handle))
    return out, counts


def run_py(ev):
    f = HorizFilter()
    out = []
    for kind, a, b, c in ev:
        if kind == "p":
            f.predict(a, b, c)
        elif kind == "v":
            f.update_gps_vel(a, b, 0.0)
        else:
            f.update_gps_pos(a, b, c)
        out.append((f.pn, f.pe, f.vn, f.ve, f.bn, f.be))
    return out, (f.n_pos_accept, f.n_pos_reject,
                 f.n_vel_accept, f.n_vel_reject)


def main():
    ap = argparse.ArgumentParser()
    # s164 is checked in gzipped (2.1 MB), shared with
    # vel_hold_test.py, so this suite also runs on CI.
    ap.add_argument("log", nargs="?",
                    default=os.path.join("tools", "fixtures",
                                         "s164_walk.csv.gz"))
    args = ap.parse_args()

    path = args.log
    if not os.path.isabs(path) and not os.path.exists(path):
        path = os.path.join(ROOT, args.log)
    rows = load(path)
    ev = events(rows)
    n_pred = sum(1 for e in ev if e[0] == "p")
    n_fix = sum(1 for e in ev if e[0] == "v")
    print("%s: %d events (%d predicts, %d GPS fixes)"
          % (os.path.basename(path), len(ev), n_pred, n_fix))
    if n_fix == 0:
        sys.exit("no usable GPS fixes in this log (HOME_VALID never set?) - "
                 "the corrections would go untested, which is most of the "
                 "filter. Use a log with a latched home point.")

    tmp = tempfile.mkdtemp()
    c, cc = run_c(build(tmp), ev)
    p, pc = run_py(ev)

    fail = 0
    names = ("pos_n", "pos_e", "vel_n", "vel_e", "bias_n", "bias_e")
    # C is float32 and the Python reference is double, so they drift apart
    # by rounding alone. The same 5 mm / 5 mm-per-second rule vert_ctest
    # uses, and the bias states get it too - they are small numbers whose
    # absolute error is what matters, not their ratio.
    TOL = 5e-3
    print()
    for k, nm in enumerate(names):
        worst = max(abs(a[k] - b[k]) for a, b in zip(c, p))
        ok = worst <= TOL
        fail += 0 if ok else 1
        print("  [%s] %-7s max |C - Python| = %.3e   (tol %.0e)"
              % ("PASS" if ok else "FAIL", nm, worst, TOL))

    ok = cc == pc
    fail += 0 if ok else 1
    print("  [%s] gate decisions identical  C pos %d/%d vel %d/%d  "
          "Py pos %d/%d vel %d/%d"
          % ("PASS" if ok else "FAIL", cc[0], cc[1], cc[2], cc[3],
             pc[0], pc[1], pc[2], pc[3]))

    print("\n  C final:      pn %+.4f pe %+.4f vn %+.4f ve %+.4f "
          "bn %+.4f be %+.4f" % c[-1])
    print("  Python final: pn %+.4f pe %+.4f vn %+.4f ve %+.4f "
          "bn %+.4f be %+.4f" % p[-1])
    print("\n%s" % ("ALL PASS" if not fail else "%d FAILURE(S)" % fail))
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
