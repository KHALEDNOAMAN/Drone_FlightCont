#!/usr/bin/env python3
"""Host test for components/gnss_guard/gnss_guard.c - the GNSS jamming and
spoofing guard.

Same trick as tools/ubx_test.py and tools/host_c.py: gnss_guard.c is
written freestanding (stdint and float, no libc, no libm, no ESP-IDF), so
clang compiles the EXACT file that flies into a DLL and ctypes drives it.

Unlike ubx_test.py this one does NOT mirror the main struct in ctypes.
gg_t is a pile of mixed-width fields whose padding would make a sizeof
assertion a test of the compiler rather than of the code, so the struct is
held as an opaque buffer and read through accessors.

The point of most of these cases is not that the guard fires. It is that
the guard does NOT fire: a gate that trips on a clean sky costs a flight
just as surely as one that never trips at all.

GNSS_GUARD_SRC lets a mutation run point the compiler at a doctored copy.

Run:  python tools/gnss_guard_test.py
"""

import ctypes
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.environ.get("GNSS_GUARD_SRC") or os.path.join(
    ROOT, "components", "gnss_guard")
UBX_SRC = os.path.join(ROOT, "components", "tbsM10")

SHIM = r"""
int _fltused = 0;
#include "gnss_guard.c"
#define EXPORT __declspec(dllexport)

EXPORT int  g_size(void){ return (int)sizeof(gg_t); }
EXPORT void g_init(gg_t *g){ gg_init(g, 0); }
EXPORT void g_defcfg(gg_cfg_t *c){ gg_default_cfg(c); }
EXPORT void g_setcfg(gg_t *g, const gg_cfg_t *c){ g->cfg = *c; }

EXPORT void g_rf(gg_t *g, unsigned int t, unsigned char mj,
                 unsigned short agc, unsigned short noise, unsigned char ji,
                 unsigned char hs, unsigned char sj, unsigned char ss){
    gg_feed_rf(g, t, mj, agc, noise, ji, hs, sj, ss);
}
EXPORT void g_cno(gg_t *g, unsigned int t, unsigned short n,
                  unsigned int s, unsigned int sq){
    gg_feed_cno(g, t, n, s, sq);
}
EXPORT void g_sol(gg_t *g, unsigned int t, unsigned char ns,
                  unsigned int gs, unsigned char hi, unsigned int is_){
    gg_feed_solution(g, t, ns, gs, hi, is_);
}
EXPORT void g_update(gg_t *g, unsigned int t){ gg_update(g, t); }
EXPORT void g_clear(gg_t *g, unsigned int t){ gg_force_clear(g, t); }

EXPORT int   g_pos(const gg_t *g){ return (int)g->st.pos; }
EXPORT int   g_vel(const gg_t *g){ return (int)g->st.vel; }
EXPORT int   g_reasons(const gg_t *g){ return (int)g->st.reasons; }
EXPORT int   g_latchedreasons(const gg_t *g){ return (int)g->st.reasons_latched; }
EXPORT float g_mean(const gg_t *g){ return g->st.cno_mean; }
EXPORT float g_sd(const gg_t *g){ return g->st.cno_sd; }
EXPORT int   g_latched(const gg_t *g){ return (int)g->latched; }
EXPORT int   g_baseready(const gg_t *g){ return (int)g->base_ready; }
EXPORT int   g_agcbase(const gg_t *g){ return (int)g->st.agc_baseline; }
EXPORT int   g_esc(const gg_t *g){ return (int)g->st.n_escalations; }
EXPORT float g_rscale(const gg_t *g, int lvl){ return gg_r_scale(g, (gg_level_t)lvl); }
EXPORT float g_sqrt(float x){ return gg_sqrtf(x); }

/* The guard mirrors u-blox's state numbering instead of including ubx.h.
 * This exports its copy so the test can assert the two still agree. */
EXPORT int g_jamwarn(void){ return UBX_JAM_WARNING; }
EXPORT int g_jamcrit(void){ return UBX_JAM_CRITICAL; }
EXPORT int g_spfind(void){ return UBX_SPF_INDICATED; }
EXPORT int g_spfmult(void){ return UBX_SPF_MULTIPLE; }
"""

CLEAN, SUSPECT, DENIED = 0, 1, 2

R = {
    "MOD_JAM_WARN":   0x0001,
    "MOD_JAM_CRIT":   0x0002,
    "MOD_SPOOF":      0x0004,
    "MOD_SPOOF_MULT": 0x0008,
    "CNO_UNIFORM":    0x0010,
    "CNO_COLLAPSE":   0x0020,
    "AGC_SHIFT":      0x0040,
    "NOISE_HIGH":     0x0080,
    "VEL_DIVERGE":    0x0100,
    "SAT_JUMP":       0x0200,
    "STALE":          0x0400,
}


class CCfg(ctypes.Structure):
    _fields_ = [
        ("cno_sd_min_x10", ctypes.c_uint16),
        ("cno_min_used", ctypes.c_uint16),
        ("cno_mean_floor", ctypes.c_uint8),
        ("agc_delta", ctypes.c_uint16),
        ("noise_delta", ctypes.c_uint16),
        ("vel_diverge_mms", ctypes.c_uint32),
        ("vel_diverge_ms", ctypes.c_uint32),
        ("sat_jump", ctypes.c_uint8),
        ("stale_ms", ctypes.c_uint32),
        ("clear_ms", ctypes.c_uint32),
        ("confirm_ms", ctypes.c_uint32),
    ]


class Host:
    def __init__(self, src_dir=SRC):
        self._tmp = tempfile.TemporaryDirectory()
        shim = os.path.join(self._tmp.name, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(self._tmp.name, "gg.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry",
               "-O1", "-I", src_dir, "-I", UBX_SRC, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L = self.lib
        vp = ctypes.c_void_p
        for name in ("g_pos", "g_vel", "g_reasons", "g_latchedreasons",
                     "g_latched", "g_baseready", "g_agcbase", "g_esc"):
            getattr(L, name).argtypes = [vp]
            getattr(L, name).restype = ctypes.c_int
        for name in ("g_mean", "g_sd"):
            getattr(L, name).argtypes = [vp]
            getattr(L, name).restype = ctypes.c_float
        L.g_size.restype = ctypes.c_int
        L.g_init.argtypes = [vp]
        L.g_defcfg.argtypes = [ctypes.POINTER(CCfg)]
        L.g_setcfg.argtypes = [vp, ctypes.POINTER(CCfg)]
        L.g_rf.argtypes = [vp, ctypes.c_uint32, ctypes.c_uint8,
                           ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint8,
                           ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8]
        L.g_cno.argtypes = [vp, ctypes.c_uint32, ctypes.c_uint16,
                            ctypes.c_uint32, ctypes.c_uint32]
        L.g_sol.argtypes = [vp, ctypes.c_uint32, ctypes.c_uint8,
                            ctypes.c_uint32, ctypes.c_uint8, ctypes.c_uint32]
        L.g_update.argtypes = [vp, ctypes.c_uint32]
        L.g_clear.argtypes = [vp, ctypes.c_uint32]
        L.g_rscale.argtypes = [vp, ctypes.c_int]
        L.g_rscale.restype = ctypes.c_float
        L.g_sqrt.argtypes = [ctypes.c_float]
        L.g_sqrt.restype = ctypes.c_float
        for n in ("g_jamwarn", "g_jamcrit", "g_spfind", "g_spfmult"):
            getattr(L, n).restype = ctypes.c_int

    def __enter__(self):
        return self

    def __exit__(self, *a):
        h = self.lib._handle
        del self.lib
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
        self._tmp.cleanup()

    def new(self, **cfg):
        buf = ctypes.create_string_buffer(self.lib.g_size())
        self.lib.g_init(buf)
        if cfg:
            c = CCfg()
            self.lib.g_defcfg(ctypes.byref(c))
            for k, v in cfg.items():
                setattr(c, k, v)
            self.lib.g_setcfg(buf, ctypes.byref(c))
        return buf

    # --- convenience: a quiet sky -------------------------------------
    def quiet(self, g, t, agc=3000, noise=100, n_used=12, mean=42, spread=9):
        """One second of a clean, well-spread constellation."""
        self.lib.g_rf(g, t, 1, agc, noise, 0, 1, 1, 1)
        s, sq = spread_stats(n_used, mean, spread)
        self.lib.g_cno(g, t, n_used, s, sq)
        self.lib.g_sol(g, t, n_used, 500, 1, 500)
        self.lib.g_update(g, t)

    def settle(self, g, t0, seconds, step=1000, **kw):
        t = t0
        for _ in range(seconds):
            self.quiet(g, t, **kw)
            t += step
        return t


def spread_stats(n, mean, spread):
    """Build sum/sumsq for n signals spread evenly over mean +/- spread.
    Computed here in Python so the C is never checked against itself."""
    vals = []
    for i in range(n):
        frac = (i / (n - 1)) if n > 1 else 0.5
        vals.append(int(round(mean - spread + 2 * spread * frac)))
    return sum(vals), sum(v * v for v in vals)


def sd_of(n, mean, spread):
    vals = []
    for i in range(n):
        frac = (i / (n - 1)) if n > 1 else 0.5
        vals.append(mean - spread + 2 * spread * frac)
    m = sum(vals) / len(vals)
    return (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5


FAIL = 0


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got  {got!r}\n         want {want!r}")


def close(name, got, want, tol):
    global FAIL
    ok = abs(got - want) <= tol
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got  {got!r}\n         want {want!r} +/- {tol}")


def main():
    with Host() as H:
        L = H.lib

        print("\nconstants still agree with ubx.h")
        ubxh = open(os.path.join(UBX_SRC, "ubx.h")).read()
        for name, fn in (("UBX_JAM_WARNING", L.g_jamwarn),
                         ("UBX_JAM_CRITICAL", L.g_jamcrit),
                         ("UBX_SPF_INDICATED", L.g_spfind),
                         ("UBX_SPF_MULTIPLE", L.g_spfmult)):
            want = int([ln.split()[-1] for ln in ubxh.splitlines()
                        if ln.startswith(f"#define {name} ")][0])
            check(f"     {name}", fn(), want)

        print("\nfreestanding sqrt")
        for x in (0.0, 1.0, 2.0, 16.0, 81.0, 1234.5, 0.25):
            close(f"     sqrt({x})", L.g_sqrt(x), x ** 0.5, 1e-3)
        check("     sqrt of a negative is 0, not a trap", L.g_sqrt(-4.0), 0.0)

        print("\na clean sky stays CLEAN")
        g = H.new()
        t = H.settle(g, 1000, 60)
        check("     position", L.g_pos(g), CLEAN)
        check("     velocity", L.g_vel(g), CLEAN)
        check("     no reasons", L.g_reasons(g), 0)
        check("     no escalations", L.g_esc(g), 0)
        check("     baseline learned", L.g_baseready(g), 1)
        close("     baseline is the quiet AGC", L.g_agcbase(g), 3000, 2)
        close("     reported C/N0 mean", L.g_mean(g), 42.0, 0.6)
        close("     reported C/N0 sd", L.g_sd(g), sd_of(12, 42, 9), 0.6)

        print("\nthe guard must go SUSPECT when it goes blind")
        L.g_update(g, t + 20000)          # no RF feed for 20 s
        check("     stale flagged", L.g_reasons(g) & R["STALE"], R["STALE"])
        check("     position SUSPECT, not CLEAN", L.g_pos(g), SUSPECT)
        check("     velocity SUSPECT", L.g_vel(g), SUSPECT)

        print("\nmodule jamming verdicts")
        g = H.new()
        t = H.settle(g, 1000, 40)
        L.g_rf(g, t, 2, 3000, 100, 0, 1, 2, 1)      # warning
        L.g_update(g, t)
        check("     warning -> position SUSPECT", L.g_pos(g), SUSPECT)
        check("     warning -> velocity still CLEAN", L.g_vel(g), CLEAN)

        g = H.new()
        t = H.settle(g, 1000, 40)
        for k in range(4):                          # critical, held 3 s
            L.g_rf(g, t + k * 1000, 3, 3000, 100, 0, 1, 3, 1)
            L.g_update(g, t + k * 1000)
        check("     critical -> position DENIED", L.g_pos(g), DENIED)
        check("     critical -> velocity only SUSPECT", L.g_vel(g), SUSPECT)
        check("     reason recorded", L.g_reasons(g) & R["MOD_JAM_CRIT"],
              R["MOD_JAM_CRIT"])

        print("\nmodule spoofing verdict condemns BOTH halves")
        g = H.new()
        t = H.settle(g, 1000, 40)
        for k in range(4):
            L.g_rf(g, t + k * 1000, 1, 3000, 100, 0, 1, 1, 3)   # multiple
            L.g_update(g, t + k * 1000)
        check("     position DENIED", L.g_pos(g), DENIED)
        check("     velocity DENIED too", L.g_vel(g), DENIED)

        print("\nconfirmation: one bad frame must not drop the fix")
        g = H.new()
        t = H.settle(g, 1000, 40)
        L.g_rf(g, t, 3, 3000, 100, 0, 1, 3, 1)      # critical, first sight
        L.g_update(g, t)
        check("     held at SUSPECT inside confirm_ms", L.g_pos(g), SUSPECT)
        L.g_rf(g, t + 1200, 3, 3000, 100, 0, 1, 3, 1)
        L.g_update(g, t + 1200)
        check("     DENIED once it has stood for confirm_ms", L.g_pos(g), DENIED)

        print("\none of our own heuristics alone can only reach SUSPECT")
        # C/N0 spread collapses - a single indicator, nothing corroborating.
        g = H.new()
        t = H.settle(g, 1000, 40)
        s, sq = spread_stats(12, 42, 0)             # every signal identical
        for k in range(6):
            L.g_rf(g, t + k * 1000, 1, 3000, 100, 0, 1, 1, 1)
            L.g_cno(g, t + k * 1000, 12, s, sq)
            L.g_sol(g, t + k * 1000, 12, 500, 1, 500)
            L.g_update(g, t + k * 1000)
        check("     uniformity detected", L.g_reasons(g) & R["CNO_UNIFORM"],
              R["CNO_UNIFORM"])
        check("     but alone it is only SUSPECT", L.g_pos(g), SUSPECT)
        check("     velocity SUSPECT (spoof family)", L.g_vel(g), SUSPECT)
        check("     never latched", L.g_latched(g), 0)

        print("\ntwo agreeing heuristics DO reach DENIED")
        g = H.new()
        t = H.settle(g, 1000, 40)
        for k in range(6):
            tk = t + k * 1000
            L.g_rf(g, tk, 1, 3000, 100, 0, 1, 1, 1)
            L.g_cno(g, tk, 12, s, sq)               # uniform C/N0
            L.g_sol(g, tk, 12, 9000, 1, 500)        # and velocity disagrees
            L.g_update(g, tk)
        r = L.g_reasons(g)
        check("     uniformity", r & R["CNO_UNIFORM"], R["CNO_UNIFORM"])
        check("     divergence", r & R["VEL_DIVERGE"], R["VEL_DIVERGE"])
        check("     position DENIED", L.g_pos(g), DENIED)
        check("     velocity DENIED", L.g_vel(g), DENIED)

        print("\nC/N0 spread test needs enough satellites to mean anything")
        g = H.new()
        t = H.settle(g, 1000, 40)
        s3, sq3 = spread_stats(3, 42, 0)            # 3 identical signals
        L.g_cno(g, t, 3, s3, sq3)
        L.g_update(g, t)
        check("     3 identical signals do NOT trip it",
              L.g_reasons(g) & R["CNO_UNIFORM"], 0)

        print("\nAGC is tested in one direction only")
        g = H.new()
        t = H.settle(g, 1000, 40)                   # baseline 3000
        L.g_rf(g, t, 1, 3000 - 900, 100, 0, 1, 1, 1)   # gain backed off
        L.g_update(g, t)
        check("     a large DROP trips it", L.g_reasons(g) & R["AGC_SHIFT"],
              R["AGC_SHIFT"])
        g = H.new()
        t = H.settle(g, 1000, 40)
        L.g_rf(g, t, 1, 3000 + 900, 100, 0, 1, 1, 1)   # quieter band
        L.g_update(g, t)
        check("     an equally large RISE does not",
              L.g_reasons(g) & R["AGC_SHIFT"], 0)

        print("\nthe baseline must not be learned while the band is dirty")
        g = H.new()
        for k in range(60):                         # jammed from boot
            tk = 1000 + k * 1000
            L.g_rf(g, tk, 3, 1200, 900, 200, 1, 3, 1)
            L.g_update(g, tk)
        check("     no baseline learned under jamming", L.g_baseready(g), 0)

        print("\nvelocity divergence must persist, not blink")
        g = H.new()
        t = H.settle(g, 1000, 40)
        L.g_sol(g, t, 12, 9000, 1, 500)             # one disagreeing sample
        L.g_update(g, t)
        check("     a single sample does not trip it",
              L.g_reasons(g) & R["VEL_DIVERGE"], 0)
        L.g_sol(g, t + 2500, 12, 9000, 1, 500)
        L.g_update(g, t + 2500)
        check("     sustained divergence does",
              L.g_reasons(g) & R["VEL_DIVERGE"], R["VEL_DIVERGE"])

        print("\nno inertial reference means the check is SKIPPED, not passed")
        g = H.new()
        t = H.settle(g, 1000, 40)
        for k in range(5):
            L.g_sol(g, t + k * 1000, 12, 9000, 0, 0)   # have_inertial = 0
            L.g_update(g, t + k * 1000)
        check("     no divergence invented",
              L.g_reasons(g) & R["VEL_DIVERGE"], 0)

        print("\nsatellite-count jumps")
        g = H.new()
        t = H.settle(g, 1000, 40)
        L.g_sol(g, t, 30, 500, 1, 500)              # 12 -> 30
        L.g_update(g, t)
        check("     a big jump is flagged", L.g_reasons(g) & R["SAT_JUMP"],
              R["SAT_JUMP"])

        # A jump is an EVENT, and NAV-PVT arrives at 10 Hz. The first
        # solution after a capture sees the step and the nine behind it see
        # a steady count, so a flag that cleared on a steady count would be
        # erased before gg_update() ever ran. This is the shape the real
        # driver feeds, and it is how the bug got through the version of
        # this test that fed one solution per update.
        g = H.new()
        t = H.settle(g, 1000, 40)
        for _ in range(10):                          # one second, 10 Hz
            L.g_sol(g, t, 30, 500, 1, 500)
        L.g_update(g, t)
        check("     survives the nine steady solutions behind it",
              L.g_reasons(g) & R["SAT_JUMP"], R["SAT_JUMP"])

        # ...and is released by its hold, not left standing forever.
        for k in range(1, 5):
            for _ in range(10):
                L.g_sol(g, t + k * 1000, 30, 500, 1, 500)
            L.g_update(g, t + k * 1000)
        check("     released once the hold expires",
              L.g_reasons(g) & R["SAT_JUMP"], 0)

        print("\nthe latch: a flickering interferer must not make it flap")
        g = H.new()
        t = H.settle(g, 1000, 40)
        for k in range(4):
            L.g_rf(g, t + k * 1000, 3, 3000, 100, 0, 1, 3, 1)
            L.g_update(g, t + k * 1000)
        check("     denied and latched", L.g_latched(g), 1)
        t2 = t + 4000
        t2 = H.settle(g, t2, 3)                     # 3 s of quiet
        check("     still denied after 3 s clean", L.g_pos(g), DENIED)
        check("     still latched", L.g_latched(g), 1)
        t2 = H.settle(g, t2, 9)                     # past clear_ms = 10 s
        check("     released after clear_ms", L.g_latched(g), 0)
        check("     position CLEAN again", L.g_pos(g), CLEAN)
        check("     but the history survives",
              L.g_latchedreasons(g) & R["MOD_JAM_CRIT"], R["MOD_JAM_CRIT"])

        print("\na latch does not release if the sky keeps flickering")
        g = H.new()
        t = H.settle(g, 1000, 40)
        for k in range(4):
            L.g_rf(g, t + k * 1000, 3, 3000, 100, 0, 1, 3, 1)
            L.g_update(g, t + k * 1000)
        t = t + 4000
        for _ in range(6):                          # 5 s quiet, 1 s bad, x6
            t = H.settle(g, t, 5)
            L.g_rf(g, t, 3, 3000, 100, 0, 1, 3, 1)
            L.g_update(g, t)
            t += 1000
        check("     never released", L.g_latched(g), 1)
        check("     still denied", L.g_pos(g), DENIED)

        print("\nmanual clear overrides the latch, not reality")
        g = H.new()
        t = H.settle(g, 1000, 40)
        for k in range(4):
            L.g_rf(g, t + k * 1000, 3, 3000, 100, 0, 1, 3, 1)
            L.g_update(g, t + k * 1000)
        L.g_clear(g, t + 4000)
        check("     cleared", L.g_latched(g), 0)
        check("     evidence kept for the log",
              L.g_latchedreasons(g) & R["MOD_JAM_CRIT"], R["MOD_JAM_CRIT"])
        for k in range(4):                          # interferer still there
            L.g_rf(g, t + 5000 + k * 1000, 3, 3000, 100, 0, 1, 3, 1)
            L.g_update(g, t + 5000 + k * 1000)
        check("     goes straight back to DENIED", L.g_pos(g), DENIED)

        print("\nR scaling")
        g = H.new()
        check("     clean is 1.0", L.g_rscale(g, CLEAN), 1.0)
        check("     suspect inflates", L.g_rscale(g, SUSPECT), 16.0)
        check("     denied is not a scale factor",
              L.g_rscale(g, DENIED) > 1000.0, True)

        print("\nescalation counter")
        g = H.new()
        t = H.settle(g, 1000, 40)
        check("     zero on a clean sky", L.g_esc(g), 0)
        L.g_rf(g, t, 2, 3000, 100, 0, 1, 2, 1)
        L.g_update(g, t)
        check("     one after the first escalation", L.g_esc(g), 1)

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
