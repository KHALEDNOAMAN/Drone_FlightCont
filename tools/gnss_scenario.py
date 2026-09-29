#!/usr/bin/env python3
"""End-to-end GNSS interference scenarios, on the host, with no hardware.

ubx_test.py proves the parser. gnss_guard_test.py proves the policy. This
runs BOTH REAL FILES TOGETHER against a synthetic byte stream, through the
same sequence tbsM10.c uses, and checks what the aircraft would actually
have concluded second by second.

That matters because the two halves can each be right and still not meet:
a decoder that reports a field the guard never reads, a message enabled at
a rate the guard treats as stale, a scenario where the evidence arrives in
an order the latch mishandles. None of that is visible from either unit
test.

It also does the job a real jammer would do, without a transmitter.
Transmitting on a GNSS band is illegal in essentially every jurisdiction
and GPS is a safety-of-life service, so the interference here is SYNTHETIC
- frames that say what a jammed or spoofed module would say. That is also
the better experiment: it is exactly repeatable, which a real jammer never
is, and it can produce a spoofer far more convincing than anything you
could legally build.

Run:  python tools/gnss_scenario.py
      python tools/gnss_scenario.py --verbose
"""

import ctypes
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
UBX_SRC = os.path.join(ROOT, "components", "tbsM10")
GG_SRC = os.path.join(ROOT, "components", "gnss_guard")

sys.path.insert(0, HERE)
from ubx_test import (frame, pvt_payload, mon_rf_payload, sec_sig_v1,  # noqa: E402
                      nav_sig_payload)

# The pipeline mirrors the dispatch in tbsM10.c's gps_task: parse a byte,
# and on a completed frame try NAV-PVT, then MON-RF, then SEC-SIG, then
# take any C/N0 snapshot. Kept here rather than in the test so the order
# under test is the order that flies.
SHIM = r"""
int _fltused = 0;
#include "ubx.c"
#include "gnss_guard.c"
#define EXPORT __declspec(dllexport)

typedef struct {
    ubx_parser_t  parser;
    gg_t          guard;
    ubx_mon_rf_t  rf;
    ubx_sec_sig_t sec;
    unsigned char have_rf, have_sec;
    unsigned int  n_pvt, n_rf, n_sec, n_sig;
    /* Set by the harness before feeding, exactly as tbsM10.c reads it
     * from nav_state.horiz_*. */
    unsigned char have_inertial;
    unsigned int  inertial_speed_mms;
} pipe_t;

EXPORT int  pipe_size(void){ return (int)sizeof(pipe_t); }
EXPORT void pipe_init(pipe_t *p){
    ubx_parser_init(&p->parser);
    gg_init(&p->guard, 0);
    p->have_rf = p->have_sec = 0;
    p->n_pvt = p->n_rf = p->n_sec = p->n_sig = 0;
    p->have_inertial = 0;
    p->inertial_speed_mms = 0;
}
EXPORT void pipe_inertial(pipe_t *p, unsigned char have, unsigned int mms){
    p->have_inertial = have; p->inertial_speed_mms = mms;
}

static unsigned int hspeed(const ubx_nav_pvt_t *v){
    float n = (float)v->vel_n_mms, e = (float)v->vel_e_mms;
    return (unsigned int)gg_sqrtf(n*n + e*e);
}

EXPORT void pipe_feed(pipe_t *p, const unsigned char *b, int n, unsigned int t){
    for (int i = 0; i < n; i++) {
        if (!ubx_parse_byte(&p->parser, b[i])) continue;
        ubx_nav_pvt_t pvt;
        if (ubx_decode_nav_pvt(&p->parser, &pvt)) {
            p->n_pvt++;
            unsigned char usable = (pvt.fix_type >= 3 && pvt.gnss_fix_ok);
            gg_feed_solution(&p->guard, t, usable ? pvt.num_sv : 0,
                             hspeed(&pvt),
                             usable ? p->have_inertial : 0,
                             p->inertial_speed_mms);
        } else if (ubx_decode_mon_rf(&p->parser, &p->rf)) {
            p->have_rf = 1; p->n_rf++;
            gg_feed_rf(&p->guard, t, p->rf.jamming_state, p->rf.agc_cnt,
                       p->rf.noise_per_ms, p->rf.jam_ind, p->have_sec,
                       p->have_sec ? p->sec.jamming_state : 0,
                       p->have_sec ? p->sec.spoofing_state : 0);
        } else if (ubx_decode_sec_sig(&p->parser, &p->sec)) {
            p->have_sec = 1; p->n_sec++;
            if (p->have_rf)
                gg_feed_rf(&p->guard, t, p->rf.jamming_state, p->rf.agc_cnt,
                           p->rf.noise_per_ms, p->rf.jam_ind, 1,
                           p->sec.jamming_state, p->sec.spoofing_state);
        }
        ubx_cno_stats_t c;
        if (ubx_take_cno_stats(&p->parser, &c)) {
            p->n_sig++;
            gg_feed_cno(&p->guard, t, c.n_used, c.sum_cno, c.sumsq_cno);
        }
    }
}
EXPORT void pipe_tick(pipe_t *p, unsigned int t){ gg_update(&p->guard, t); }
EXPORT int  pipe_pos(const pipe_t *p){ return (int)p->guard.st.pos; }
EXPORT int  pipe_vel(const pipe_t *p){ return (int)p->guard.st.vel; }
EXPORT int  pipe_reasons(const pipe_t *p){ return (int)p->guard.st.reasons; }
EXPORT float pipe_sd(const pipe_t *p){ return p->guard.st.cno_sd; }
EXPORT float pipe_mean(const pipe_t *p){ return p->guard.st.cno_mean; }
EXPORT int  pipe_counts(const pipe_t *p, int which){
    switch (which) { case 0: return (int)p->n_pvt; case 1: return (int)p->n_rf;
                     case 2: return (int)p->n_sec; default: return (int)p->n_sig; }
}
"""

LVL = {0: "CLEAN", 1: "SUSPECT", 2: "DENIED"}
R_NAMES = [
    (0x0001, "jam-warn"), (0x0002, "jam-crit"), (0x0004, "spoof"),
    (0x0008, "spoof-mult"), (0x0010, "cno-uniform"), (0x0020, "cno-collapse"),
    (0x0040, "agc-shift"), (0x0080, "noise-high"), (0x0100, "vel-diverge"),
    (0x0200, "sat-jump"), (0x0400, "STALE"),
]


def reasons_str(r):
    s = [n for b, n in R_NAMES if r & b]
    return ",".join(s) if s else "-"


class Pipe:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        shim = os.path.join(self._tmp.name, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(self._tmp.name, "pipe.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry", "-O1",
               "-I", UBX_SRC, "-I", GG_SRC, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this harness needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L = self.lib
        vp = ctypes.c_void_p
        L.pipe_size.restype = ctypes.c_int
        L.pipe_init.argtypes = [vp]
        L.pipe_inertial.argtypes = [vp, ctypes.c_uint8, ctypes.c_uint32]
        L.pipe_feed.argtypes = [vp, ctypes.POINTER(ctypes.c_uint8),
                                ctypes.c_int, ctypes.c_uint32]
        L.pipe_tick.argtypes = [vp, ctypes.c_uint32]
        for n in ("pipe_pos", "pipe_vel", "pipe_reasons"):
            getattr(L, n).argtypes = [vp]
            getattr(L, n).restype = ctypes.c_int
        for n in ("pipe_sd", "pipe_mean"):
            getattr(L, n).argtypes = [vp]
            getattr(L, n).restype = ctypes.c_float
        L.pipe_counts.argtypes = [vp, ctypes.c_int]
        L.pipe_counts.restype = ctypes.c_int
        self.p = ctypes.create_string_buffer(L.pipe_size())
        L.pipe_init(self.p)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        h = self.lib._handle
        del self.lib
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
        self._tmp.cleanup()

    def inertial(self, have, mms):
        self.lib.pipe_inertial(self.p, 1 if have else 0, mms)

    def feed(self, data, t_ms):
        arr = (ctypes.c_uint8 * len(data))(*data)
        self.lib.pipe_feed(self.p, arr, len(data), t_ms)

    def tick(self, t_ms):
        self.lib.pipe_tick(self.p, t_ms)

    @property
    def pos(self):
        return self.lib.pipe_pos(self.p)

    @property
    def vel(self):
        return self.lib.pipe_vel(self.p)

    @property
    def reasons(self):
        return self.lib.pipe_reasons(self.p)


def second(sats=12, cno_mean=42, cno_spread=9, jam=1, spoof=1,
           agc=3000, noise=100, jam_ind=0, gps_speed_mms=0, n_used=None):
    """One second of module output: 10 NAV-PVT, 1 each of the other three."""
    n_used = sats if n_used is None else n_used
    out = b""
    vn = gps_speed_mms
    for _ in range(10):
        out += frame(0x01, 0x07, pvt_payload(fix=3, flags=1, nsv=sats,
                                             lat=401885000, lon=290610000,
                                             vn=vn, ve=0, vd=0))
    sigs = []
    for i in range(n_used):
        frac = (i / (n_used - 1)) if n_used > 1 else 0.5
        c = int(round(cno_mean - cno_spread + 2 * cno_spread * frac))
        sigs.append((max(c, 1), 1))
    out += frame(0x01, 0x43, nav_sig_payload(sigs))
    out += frame(0x0A, 0x38, mon_rf_payload([(jam, 2, noise, agc, jam_ind)]))
    out += frame(0x27, 0x09, sec_sig_v1(1, jam, 1, spoof))
    return out


FAIL = 0


def expect(name, got, want):
    global FAIL
    if got != want:
        FAIL += 1
        print(f"  [FAIL] {name}\n         got  {got}\n         want {want}")
    else:
        print(f"  [PASS] {name}")


def run(title, steps, verbose=False):
    """steps: list of (label, kwargs_for_second, inertial_mms_or_None)."""
    print(f"\n=== {title} ===")
    with Pipe() as P:
        t = 1000
        trace = []
        for label, kw, inertial in steps:
            if inertial is None:
                P.inertial(False, 0)
            else:
                P.inertial(True, inertial)
            P.feed(second(**kw), t)
            P.tick(t)
            trace.append((label, t, P.pos, P.vel, P.reasons))
            if verbose:
                print(f"    t={t/1000:5.1f}s {label:<16} "
                      f"pos={LVL[P.pos]:<7} vel={LVL[P.vel]:<7} "
                      f"{reasons_str(P.reasons)}")
            t += 1000
        return trace


def main():
    verbose = "--verbose" in sys.argv

    # ---------------------------------------------------------------- 1
    steps = [("clean", {}, 0) for _ in range(40)]
    tr = run("a clean sky, 40 s - the guard must stay out of the way",
             steps, verbose)
    expect("never left CLEAN", {(p, v) for _, _, p, v, _ in tr}, {(0, 0)})
    expect("never raised a reason", {r for *_, r in tr}, {0})

    # ---------------------------------------------------------------- 2
    # A jammer ramping in: noise and AGC move first, the module's own
    # detector catches up, and the verdict escalates rather than jumping.
    steps = [("clean", {}, 0) for _ in range(40)]
    # Genuinely warm: the module notices, but the AGC has only eased off
    # 300 counts and the noise floor has risen 200 - both inside their
    # thresholds. One indicator standing alone, which must mean SUSPECT.
    # (An earlier draft of this scenario put the AGC and noise past their
    # thresholds here too and then expected SUSPECT; three agreeing
    # indicators earning DENIED was the guard being right.)
    steps += [("jammer warm", dict(jam=2, agc=2700, noise=300,
                                   jam_ind=40, cno_mean=36), 0)
              for _ in range(3)]
    steps += [("jammer full", dict(jam=3, agc=1400, noise=900, jam_ind=210,
                                   cno_mean=19, sats=4, n_used=4), 0)
              for _ in range(5)]
    tr = run("a jammer ramping in", steps, verbose)
    expect("clean while the sky was clean", tr[39][2], 0)
    expect("SUSPECT as the jammer warms up", tr[41][2], 1)
    expect("position DENIED at full jamming", tr[-1][2], 2)
    expect("velocity only SUSPECT - it degrades later", tr[-1][3], 1)
    expect("the module's critical verdict is in the reasons",
           bool(tr[-1][4] & 0x0002), True)

    # ---------------------------------------------------------------- 3
    # A spoofer capturing the receiver. The tell is NOT a bad-looking fix:
    # C/N0 is HIGH and the satellite count is HEALTHY. What gives it away
    # is that every signal arrives at the same strength, and the position
    # starts moving in a way the accelerometers never felt.
    steps = [("clean", {}, 500) for _ in range(40)]
    steps += [("capture", dict(sats=26, n_used=26, cno_mean=47,
                               cno_spread=0, gps_speed_mms=9000), 500)
              for _ in range(6)]
    tr = run("a spoofer capturing the receiver", steps, verbose)
    expect("clean before the capture", tr[39][2], 0)
    expect("the fix still LOOKS healthy - module says no jamming",
           bool(tr[-1][4] & 0x0003), False)
    expect("C/N0 spread collapse seen", bool(tr[-1][4] & 0x0010), True)
    # The count jump is an EVENT with a 3 s hold, so it is asserted at the
    # instant of capture rather than six seconds later. Its absence at the
    # end is the hold working, not the detector failing - which is why the
    # release is asserted too.
    expect("satellite-count jump seen at the capture",
           bool(tr[40][4] & 0x0200), True)
    expect("and released once its hold expired",
           bool(tr[-1][4] & 0x0200), False)
    expect("inertial disagreement seen", bool(tr[-1][4] & 0x0100), True)
    expect("position DENIED", tr[-1][2], 2)
    expect("velocity DENIED too - a spoofer owns the Doppler", tr[-1][3], 2)

    # ---------------------------------------------------------------- 4
    # The case that must NOT fire: flying under a bridge. C/N0 collapses
    # and satellites drop, but nothing else corroborates, and the aircraft
    # is not being attacked.
    steps = [("clean", {}, 500) for _ in range(40)]
    steps += [("under a bridge", dict(sats=6, n_used=6, cno_mean=21,
                                      cno_spread=6, agc=3050), 500)
              for _ in range(4)]
    steps += [("clear again", {}, 500) for _ in range(3)]
    tr = run("flying under a bridge - degradation, not an attack", steps,
             verbose)
    expect("never DENIED", max(p for _, _, p, _, _ in tr), 1)
    expect("recovered to CLEAN afterwards", tr[-1][2], 0)

    # ---------------------------------------------------------------- 5
    # The module stops talking entirely. The guard must say it is blind.
    with Pipe() as P:
        print("\n=== the module goes silent ===")
        t = 1000
        for _ in range(40):
            P.inertial(True, 0)
            P.feed(second(), t)
            P.tick(t)
            t += 1000
        expect("clean while it was talking", P.pos, 0)
        P.tick(t + 20000)
        expect("STALE once it stops", bool(P.reasons & 0x0400), True)
        expect("position SUSPECT, never CLEAN", P.pos, 1)
        expect("velocity SUSPECT", P.vel, 1)

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL SCENARIOS PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
