#!/usr/bin/env python3
"""Host test for the rate I-term ground hold, components/common/iterm_gate.c.

    python tools/iterm_gate_test.py

Compiles the REAL iterm_gate.c freestanding, the same way
pid_windup_test.py compiles pid.c, and checks it three ways:

  1. One scenario per clause of the rule in iterm_gate.h, boundaries
     included - the 3 cm band, the 1250 us throttle, the 500 ms freshness
     window, the fallback latch, the blackbox bits.
  2. s207 (a real 91 s flight, props on): the integrators are
     released AT THE LIFTOFF FRAME - not at the 1250 us throttle crossing
     9.8 s earlier - and never held again while airborne, through a low
     hover that skimmed the ground at 1431 us.
  3. s19 (bench, props off, throttle ramp to 1998 us plus full
     stick inputs): replayed through the gate the rate integrators stay at
     zero; without it they end at the -20.7 / +20.2 us the motors showed.

Fixtures (tools/fixtures/iterm_*.csv.gz) are those two decoded sessions cut
to the columns this needs: t_us relative to the first frame, throttle,
tof_mm (= tof_m * 1000, sentinels 0 and 65535 kept) and, for s19, the rate
setpoints and gyro in the frame's native units (dps * 16). No GPS columns.
The logs carry no ToF sample counter, so the replays advance it every
200 ms (the ToF timing budget); freshness itself is the scenarios' job.

ITERM_SRC points the compiler at a doctored copy for mutation runs
(python tools/mutate.py iterm_gate).
"""

import csv
import ctypes
import gzip
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.environ.get("ITERM_SRC") or os.path.join(ROOT, "components",
                                                   "common")
FIX = os.path.join(HERE, "fixtures")

# The rule as agreed. Asserted against the C constants, so a
# changed number fails here instead of quietly changing what flies.
THR_US, NEAR_MM, FRESH_US = 1250, 30, 500000
F_HELD, F_TOF_REF, F_LIFTED, F_FRESH, F_THR = 1, 2, 4, 8, 16
NONE, NO_TARGET = 0, 65535

SHIM = r"""
int _fltused = 0;
#include "iterm_gate.c"
#define EXPORT __declspec(dllexport)
EXPORT void g_init(iterm_gate_t *g){ iterm_gate_init(g); }
EXPORT int  g_step(iterm_gate_t *g, int armed, unsigned thr, unsigned tof,
                   unsigned seq, long long now){
    return iterm_gate_step(g, armed != 0, (uint16_t)thr, (uint16_t)tof,
                           (uint32_t)seq, (int64_t)now) ? 1 : 0;
}
EXPORT unsigned g_flags(const iterm_gate_t *g){ return iterm_gate_flags(g); }
EXPORT int g_size(void){ return (int)sizeof(iterm_gate_t); }
EXPORT unsigned g_thr(void){ return ITERM_GATE_THR_US; }
EXPORT int g_near(void){ return ITERM_GATE_NEAR_MM; }
EXPORT long long g_fresh_us(void){ return ITERM_GATE_FRESH_US; }
EXPORT unsigned g_fbits(void){
    return ITERM_GATE_F_HELD | ITERM_GATE_F_TOF_REF << 5 |
           ITERM_GATE_F_LIFTED << 10 | ITERM_GATE_F_FRESH << 15 |
           ITERM_GATE_F_THR << 20;
}
"""


class CGate(ctypes.Structure):
    _fields_ = [("was_armed", ctypes.c_bool), ("ref_ok", ctypes.c_bool),
                ("ref_mm", ctypes.c_uint16), ("lifted", ctypes.c_bool),
                ("thr_seen", ctypes.c_bool), ("have_seq", ctypes.c_bool),
                ("seq_moved", ctypes.c_bool), ("seq", ctypes.c_uint32),
                ("seq_us", ctypes.c_int64), ("fresh", ctypes.c_bool),
                ("hold", ctypes.c_bool)]


class Host:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        shim = os.path.join(self._tmp.name, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(self._tmp.name, "itg.dll")
        r = subprocess.run(["clang", "-shared", "-nostdlib", "-Wl,-noentry",
                            "-O1", "-I", SRC, shim, "-o", dll],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L = self.lib
        pg = ctypes.POINTER(CGate)
        L.g_init.argtypes = [pg]
        L.g_step.argtypes = [pg, ctypes.c_int, ctypes.c_uint, ctypes.c_uint,
                             ctypes.c_uint, ctypes.c_longlong]
        L.g_step.restype = ctypes.c_int
        L.g_flags.argtypes = [pg]
        L.g_flags.restype = ctypes.c_uint
        L.g_size.restype = ctypes.c_int
        L.g_thr.restype = ctypes.c_uint
        L.g_near.restype = ctypes.c_int
        L.g_fresh_us.restype = ctypes.c_longlong
        L.g_fbits.restype = ctypes.c_uint
        if L.g_size() != ctypes.sizeof(CGate):
            raise SystemExit(f"iterm_gate_t layout mismatch: C {L.g_size()} B, "
                             f"ctypes {ctypes.sizeof(CGate)} B")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        import _ctypes
        _ctypes.FreeLibrary(self.lib._handle)
        self._tmp.cleanup()


class Sim:
    """One aircraft: a 250 Hz loop clock, and a ToF that completes a sample
    every 200 ms while it is alive (its counter stops when it is not)."""
    CYCLE = 4000

    def __init__(self, lib, tof=110):
        self.L = lib
        self.g = CGate()
        lib.g_init(ctypes.byref(self.g))
        self.t = 5_000_000
        self.seq = 7
        self.last = -10 ** 12
        self.tof = tof

    def run(self, seconds=None, cycles=None, armed=True, thr=1000, tof=None,
            alive=True):
        if tof is not None:
            self.tof = tof
        n = cycles if cycles is not None else round(seconds * 1e6 / self.CYCLE)
        out = []
        for _ in range(n):
            if alive and self.t - self.last >= 200_000:
                self.seq += 1
                self.last = self.t
            out.append(self.L.g_step(ctypes.byref(self.g), 1 if armed else 0,
                                     thr, self.tof, self.seq, self.t))
            self.t += self.CYCLE
        return out

    def flags(self):
        return self.L.g_flags(ctypes.byref(self.g))


FAILS = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail and not ok else ""))
    if not ok:
        FAILS.append(name)


def booted(lib, tof=110, alive=True):
    """Powered up on the table and disarmed for 1 s, as before any arm."""
    s = Sim(lib, tof)
    s.run(1.0, armed=False, tof=tof, alive=alive)
    return s


def scenarios(lib):
    print("rule, clause by clause")
    check("constants are the agreed ones (1250 us, 30 mm, 500 ms, bits)",
          (lib.g_thr(), lib.g_near(), lib.g_fresh_us(), lib.g_fbits())
          == (THR_US, NEAR_MM, FRESH_US,
              F_HELD | F_TOF_REF << 5 | F_LIFTED << 10 | F_FRESH << 15 | F_THR << 20),
          f"{lib.g_thr()} {lib.g_near()} {lib.g_fresh_us()} {lib.g_fbits():#x}")

    s = Sim(lib)
    h = s.run(1.0, armed=False)
    check("disarmed never holds (the disarmed path resets the PIDs itself)",
          not any(h))

    s = booted(lib)
    h = s.run(5.0, thr=1000)
    check("idle on the table: held every cycle", all(h))
    check("idle on the table: flags HELD|TOF_REF|FRESH",
          s.flags() == F_HELD | F_TOF_REF | F_FRESH, f"{s.flags():#x}")

    s = booted(lib)
    h = []
    n = 1250
    for i in range(n):
        h += s.run(cycles=1, thr=1000 + i * 1000 // (n - 1),
                   tof=108 if i % 2 else 114)
    check("props-off bench, ramp to 2000 us on the table: held throughout",
          all(h))
    check("props-off bench: never counted as lifted",
          not s.flags() & F_LIFTED, f"{s.flags():#x}")

    s = booted(lib)
    h = []
    for i in range(1500):                         # 6 s spool to hover
        h += s.run(cycles=1, thr=1000 + i * 430 // 1499, tof=110)
    check("spool-up to 1430 us still on its legs: held (even past 1250)",
          all(h))
    h = s.run(cycles=1, thr=1430, tof=145)
    check("liftoff (+35 mm): released on the first fresh reading", h == [0])
    check("after liftoff: flags TOF_REF|LIFTED|FRESH",
          s.flags() == F_TOF_REF | F_LIFTED | F_FRESH, f"{s.flags():#x}")
    h = s.run(5.0, thr=1430, tof=120) + s.run(5.0, thr=1430, tof=111)
    check("low hover skimming within 3 cm at 1430 us: never held", not any(h))
    check("after liftoff, 1250 us near the ground is still flying",
          s.run(cycles=5, thr=1250, tof=111) == [0] * 5)
    check("landing, throttle cut to 1249 us near the ground: held",
          s.run(cycles=5, thr=1249, tof=111) == [1] * 5)
    check("landed at idle: held", all(s.run(2.0, thr=1000, tof=111)))
    check("above ~1.7 m (65535) after liftoff: never held",
          not any(s.run(2.0, thr=1000, tof=NO_TARGET)))

    s.run(0.5, armed=False, tof=110)
    h = s.run(1.0, thr=1000, tof=110)
    check("disarm and re-arm starts over: held again on the table", all(h))
    check("re-arm clears LIFTED", not s.flags() & F_LIFTED, f"{s.flags():#x}")

    s = booted(lib)
    s.run(0.2, thr=1000, tof=110)                # armed, reference taken
    check("65535 on the ground before liftoff: never held",
          not any(s.run(1.0, thr=1000, tof=NO_TARGET)))
    s = booted(lib)
    s.run(0.2, thr=1000, tof=110)
    check("0 (reads failing) before liftoff: never held",
          not any(s.run(1.0, thr=1000, tof=NONE)))

    s = booted(lib)
    s.run(0.2, thr=1000, tof=110)                # reference = 110 mm
    check("3 cm band, +30 mm: still on its legs",
          s.run(cycles=3, tof=140) == [1] * 3 and not s.flags() & F_LIFTED)
    check("3 cm band, -30 mm: still on its legs",
          s.run(cycles=3, tof=80) == [1] * 3)
    check("3 cm band, -31 mm: not on its legs, not lifted either",
          s.run(cycles=3, tof=79) == [0] * 3 and not s.flags() & F_LIFTED)
    check("3 cm band, +31 mm: lifted",
          s.run(cycles=1, tof=141) == [0] and bool(s.flags() & F_LIFTED))

    s = booted(lib)
    while True:                                  # stop right after a sample
        before = s.seq
        s.run(cycles=1)
        if s.seq != before:
            break
    h = s.run(cycles=140, alive=False, tof=110)
    check("stale reading: held for the 500 ms freshness window",
          all(h[:120]), f"first release at cycle {h.index(0) if 0 in h else None}")
    check("stale reading: released once the counter stops for >500 ms",
          not any(h[130:]))

    for label, tof, alive in (("no reading at arm (0)", NONE, True),
                              ("nothing in range at arm (65535)", NO_TARGET, True),
                              ("counter never moved since boot", 110, False)):
        s = booted(lib, tof=tof, alive=alive)
        h = s.run(1.0, thr=1000, alive=alive)
        check(f"{label}: fallback holds at idle", all(h))
        check(f"{label}: fallback logged (TOF_REF clear)",
              not s.flags() & F_TOF_REF, f"{s.flags():#x}")
        check(f"{label}: 1249 us still held",
              s.run(cycles=3, thr=1249, alive=alive) == [1] * 3)
        check(f"{label}: 1250 us releases",
              s.run(cycles=3, thr=1250, alive=alive) == [0] * 3)
        check(f"{label}: stays released back at idle (latched)",
              not any(s.run(1.0, thr=1000, alive=alive)))
        check(f"{label}: flags show the latch (THR)",
              s.flags() & (F_THR | F_HELD | F_TOF_REF) == F_THR,
              f"{s.flags():#x}")


def load(name):
    with gzip.open(os.path.join(FIX, name), "rt", newline="") as fh:
        return [{k: int(v) for k, v in r.items()} for r in csv.DictReader(fh)]


def replay(lib, rows):
    """Step the gate through a logged armed session. Returns (holds, flags)."""
    g = CGate()
    lib.g_init(ctypes.byref(g))
    seq, last = 0, -10 ** 12
    t = rows[0]["t_us"] - 1_000_000               # 1 s disarmed first
    while t < rows[0]["t_us"]:
        if t - last >= 200_000:
            seq, last = seq + 1, t
        lib.g_step(ctypes.byref(g), 0, 1000, rows[0]["tof_mm"], seq, t)
        t += 4000
    holds, flags = [], []
    for r in rows:
        if r["t_us"] - last >= 200_000:
            seq, last = seq + 1, r["t_us"]
        holds.append(lib.g_step(ctypes.byref(g), 1, r["throttle"],
                                r["tof_mm"], seq, r["t_us"]))
        flags.append(lib.g_flags(ctypes.byref(g)))
    return holds, flags


def flight_s207(lib):
    print("s207, real flight (props on)")
    rows = load("iterm_s207_flight.csv.gz")
    holds, _ = replay(lib, rows)
    ref = rows[0]["tof_mm"]
    up = [i for i, r in enumerate(rows)
          if r["tof_mm"] not in (NONE, NO_TARGET) and r["tof_mm"] > ref + NEAR_MM]
    lift, last_air = up[0], up[-1]
    t = lambda i: (rows[i]["t_us"] - rows[0]["t_us"]) / 1e6
    first_live = holds.index(0)
    check(f"released exactly at liftoff (frame {lift}, t={t(lift):.2f} s)",
          first_live == lift, f"first release frame {first_live}, t={t(first_live):.2f} s")
    check("never held while airborne (liftoff to last airborne frame)",
          not any(holds[lift:last_air + 1]),
          f"{sum(holds[lift:last_air + 1])} held frames")
    cross = next(i for i, r in enumerate(rows) if r["throttle"] >= THR_US)
    check("a plain 1250 us rule would have released >9 s on the ground",
          t(lift) - t(cross) > 9.0, f"{t(lift) - t(cross):.1f} s")


def bench_s19(lib):
    print("s19, bench (props off, ramp + full sticks)")
    rows = load("iterm_s19_bench.csv.gz")
    holds, flags = replay(lib, rows)
    KI, LIM = 0.05, 100 / 0.05                    # rate_ki, i_max / ki
    res = {}
    for gated in (False, True):
        ir = ip = 0.0
        worst = 0.0
        prev = rows[0]["t_us"] - 4000
        for r, h in zip(rows, holds):
            dt = min(max((r["t_us"] - prev) / 1e6, 0.002), 0.006)
            prev = r["t_us"]
            if gated and h:
                ir = ip = 0.0                     # as fc_tasks.c, before the PID
            ir = min(max(ir + (r["setp_roll_x16"] - r["gyro_roll_x16"]) / 16 * dt, -LIM), LIM)
            ip = min(max(ip + (r["setp_pitch_x16"] - r["gyro_pitch_x16"]) / 16 * dt, -LIM), LIM)
            worst = max(worst, abs(ir * KI), abs(ip * KI))
        res[gated] = (ir * KI, ip * KI, worst)
    r0, p0, _ = res[False]
    check("fixture is the real case: throttle reached 1998 us",
          max(r["throttle"] for r in rows) >= 1998)
    check(f"without the gate: ends at roll {r0:+.1f} / pitch {p0:+.1f} us (logged -20.7 / +20.2)",
          abs(r0 + 20.7) < 1.0 and abs(p0 - 20.2) < 1.0)
    check(f"through the gate: |I| never above {res[True][2]:.2f} us",
          res[True][2] < 0.5)
    check("bench frame never counted as lifted",
          not any(f & F_LIFTED for f in flags))


def main():
    with Host() as host:
        scenarios(host.lib)
        flight_s207(host.lib)
        bench_s19(host.lib)
    print("\nOK" if not FAILS else f"\n{len(FAILS)} FAILURE(S)")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
