#!/usr/bin/env python3
"""Host test for components/pmw3901/pmw3901_calc.c - motion-burst decoding,
the per-sample checks, the running-sum bookkeeping, the identity verdicts and
the vendor init tables.

Same trick as tools/ms5611_test.py: the file is freestanding, so clang
compiles the exact code that flies and ctypes drives it. The shim keeps the
tracking state in C and exposes it through accessors, so no Python mirror of
pmw3901_track_t can silently drift from the real layout.

Where a reference exists it is external, never the C's own output:
  * byte order of the burst: Crazyflie's motionBurst_t (deltas little-endian,
    shutter byte-swapped by hand) and ArduPilot's MotionBurst (shutter_upper
    before shutter_lower), both independent of this driver
  * the init table: the four entries where PixArt's demo kit V3.20 (PX4)
    differs from the older table (Bitcraze Arduino, ArduPilot) - so a
    "fix" back to the old values fails here instead of passing review
  * the prelude arithmetic: PX4's sensorInit(), re-implemented below

PMW3901_SRC points the compiler at a doctored copy for mutation runs.

Run:  python tools/pmw3901_test.py
"""

import ctypes
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.environ.get("PMW3901_SRC") or os.path.join(ROOT, "components", "pmw3901")

SHIM = r"""
int _fltused = 0;
typedef unsigned long long sz_t;
/* clang may lower the byte loops to these; -nostdlib has no libc. */
void *memcpy(void *d, const void *s, sz_t n)
{ unsigned char *a = d; const unsigned char *b = s; while (n--) *a++ = *b++; return d; }
void *memset(void *d, int c, sz_t n)
{ unsigned char *a = d; while (n--) *a++ = (unsigned char)c; return d; }

#include "pmw3901_calc.c"
#define EXPORT __declspec(dllexport)

static pmw3901_track_t T;

EXPORT void b_decode(const unsigned char *raw, int *o)
{
    pmw3901_burst_t b;
    pmw3901_burst_decode(raw, &b);
    o[0] = b.motion; o[1] = b.observation; o[2] = b.dx; o[3] = b.dy;
    o[4] = b.squal; o[5] = b.raw_sum; o[6] = b.raw_max; o[7] = b.raw_min;
    o[8] = b.shutter;
}
EXPORT int  b_size(void) { return (int)sizeof(pmw3901_burst_t); }

EXPORT void t_init(void)      { pmw3901_track_init(&T); }
EXPORT void t_restart(void)   { pmw3901_track_restart(&T); }
EXPORT void t_note_lost(void) { pmw3901_track_note_lost(&T); }
EXPORT int  t_update(const unsigned char *raw, unsigned int dt_us)
{
    pmw3901_burst_t b;
    pmw3901_burst_decode(raw, &b);
    return pmw3901_track_update(&T, raw, &b, dt_us);
}
EXPORT int t_get(int k)
{
    switch (k) {
    case 0:  return T.sum_dx;
    case 1:  return T.sum_dy;
    case 2:  return (int)T.n_bursts;
    case 3:  return (int)T.n_valid;
    case 4:  return (int)T.n_lost;
    case 5:  return (int)T.n_stuck;
    case 6:  return (int)T.n_inconsistent;
    case 7:  return (int)T.n_saturated;
    case 8:  return (int)T.n_rate;
    case 9:  return (int)T.n_bad_dt;
    case 10: return (int)T.same_run;
    case 11: return (int)T.same_run_max;
    }
    return -999;
}

EXPORT int id_check(int id, int inv) { return (int)pmw3901_id_check((uint8_t)id, (uint8_t)inv); }

EXPORT int tbl_len(int which) { return (int)(which ? pmw3901_init_b_len : pmw3901_init_a_len); }
EXPORT int tbl_reg(int which, int i) { return (which ? pmw3901_init_b : pmw3901_init_a)[i].reg; }
EXPORT int tbl_val(int which, int i) { return (which ? pmw3901_init_b : pmw3901_init_a)[i].val; }
EXPORT int bank_after(int which, int bank_in)
{
    return pmw3901_bank_after(which ? pmw3901_init_b : pmw3901_init_a,
                              which ? pmw3901_init_b_len : pmw3901_init_a_len,
                              (uint8_t)bank_in);
}
EXPORT int pre_c1(int raw) { return pmw3901_prelude_c1((uint8_t)raw); }
EXPORT int pre_c2(int raw) { return pmw3901_prelude_c2((uint8_t)raw); }

EXPORT int flag(int k)
{
    switch (k) {
    case 0: return PMW_F_VALID;       case 1: return PMW_F_MOTION;
    case 2: return PMW_F_FIRST;       case 3: return PMW_F_BUS_STUCK;
    case 4: return PMW_F_INCONSISTENT; case 5: return PMW_F_SATURATED;
    case 6: return PMW_F_RATE;        case 7: return PMW_F_AFTER_LOST;
    case 8: return PMW_F_BAD_DT;
    }
    return 0;
}
"""

# pmw3901_id_verdict_t, in declaration order
ID_OK, ID_NONE_00, ID_NONE_FF, ID_SHIFTED, ID_INV_BAD, ID_WRONG = range(6)

SUM_DX, SUM_DY, N_BURSTS, N_VALID, N_LOST, N_STUCK, N_INCONS, N_SAT, \
    N_RATE, N_BAD_DT, SAME_RUN, SAME_RUN_MAX = range(12)


class Host:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        shim = os.path.join(self._tmp.name, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(self._tmp.name, "pmw.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry", "-O1",
               "-I", SRC, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L = self.lib
        u8p = ctypes.POINTER(ctypes.c_uint8)
        L.b_decode.argtypes = [u8p, ctypes.POINTER(ctypes.c_int)]
        L.t_update.argtypes = [u8p, ctypes.c_uint32]
        for f in ("b_size", "t_update", "t_get", "id_check", "tbl_len",
                  "tbl_reg", "tbl_val", "bank_after", "pre_c1", "pre_c2", "flag"):
            getattr(L, f).restype = ctypes.c_int
        F = [L.flag(k) for k in range(9)]
        (self.VALID, self.MOTION, self.FIRST, self.STUCK, self.INCONS,
         self.SAT, self.RATE, self.AFTER_LOST, self.BAD_DT) = F

    def __enter__(self):
        return self

    def __exit__(self, *a):
        h = self.lib._handle
        del self.lib
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
        self._tmp.cleanup()

    @staticmethod
    def raw(b):
        return (ctypes.c_uint8 * 12)(*b)

    def decode(self, b):
        o = (ctypes.c_int * 9)()
        self.lib.b_decode(self.raw(b), o)
        return list(o)

    def update(self, b, dt_us):
        return self.lib.t_update(self.raw(b), dt_us)

    def get(self, k):
        return self.lib.t_get(k)


def burst(dx=0, dy=0, motion=0xB0, obs=0x3F, squal=80, rsum=60, rmax=150,
          rmin=20, shutter=300):
    """A burst in WIRE order, built from the datasheet register order - not
    from anything in the C."""
    dx &= 0xFFFF
    dy &= 0xFFFF
    return [motion, obs, dx & 0xFF, dx >> 8, dy & 0xFF, dy >> 8, squal, rsum,
            rmax, rmin, shutter >> 8, shutter & 0xFF]


FAIL = 0


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got {got!r}  want {want!r}")


def main():
    with Host() as H:
        print("\nburst decoding - two byte orders in one read")
        check("pmw3901_burst_t is 12 bytes", H.lib.b_size(), 12)
        d = H.decode([0xB0, 0x3F, 0xFD, 0xFF, 0x02, 0x01, 0x55, 0x40, 0x90,
                      0x10, 0x01, 0x23])
        check("motion byte", d[0], 0xB0)
        check("observation byte", d[1], 0x3F)
        check("dx little-endian: FD FF -> -3", d[2], -3)
        check("dy little-endian: 02 01 -> +258", d[3], 258)
        check("squal / raw_sum / raw_max / raw_min in order",
              d[4:8], [0x55, 0x40, 0x90, 0x10])
        check("shutter BIG-endian: 01 23 -> 0x0123 (Upper first)", d[8], 0x0123)
        check("dx 00 80 -> INT16_MIN", H.decode(burst(dx=-32768))[2], -32768)

        print("\nidentity - the failure pattern names the fault")
        check("49/B6 ok", H.lib.id_check(0x49, 0xB6), ID_OK)
        check("00/00: MISO low or unpowered", H.lib.id_check(0x00, 0x00), ID_NONE_00)
        check("FF/FF: nothing drove MISO", H.lib.id_check(0xFF, 0xFF), ID_NONE_FF)
        check("one bit late (24/5B)", H.lib.id_check(0x24, 0x5B), ID_SHIFTED)
        check("one bit late, stray MSB (A4/DB)", H.lib.id_check(0xA4, 0xDB), ID_SHIFTED)
        check("one bit early (92/6C)", H.lib.id_check(0x92, 0x6C), ID_SHIFTED)
        check("one bit early, stray LSB (93/6D)", H.lib.id_check(0x93, 0x6D), ID_SHIFTED)
        check("mixed shifts are NOT a shift (24/6C)", H.lib.id_check(0x24, 0x6C), ID_WRONG)
        check("only one byte shifted is not a shift (24/B6)",
              H.lib.id_check(0x24, 0xB6), ID_WRONG)
        check("ID right, inverse wrong (49/00)", H.lib.id_check(0x49, 0x00), ID_INV_BAD)
        check("the IMU's WHO_AM_I (6C) is not this chip", H.lib.id_check(0x6C, 0x93), ID_WRONG)

        print("\nfirst window after an init is discarded, and is not a hole")
        H.lib.t_init()
        f = H.update(burst(dx=500, dy=500), 0)
        check("FIRST set", bool(f & H.FIRST), True)
        check("     and not VALID", bool(f & H.VALID), False)
        check("     sums untouched", (H.get(SUM_DX), H.get(SUM_DY)), (0, 0))
        check("     not counted lost", H.get(N_LOST), 0)
        f = H.update(burst(dx=5, dy=-7), 10000)
        check("next sample VALID", bool(f & H.VALID), True)
        check("     and NOT after-lost", bool(f & H.AFTER_LOST), False)
        check("     sums (5, -7)", (H.get(SUM_DX), H.get(SUM_DY)), (5, -7))
        check("     motion bit passed through", bool(f & H.MOTION), True)
        f = H.update(burst(dx=0, dy=0, motion=0x30), 10000)
        check("no-motion burst still VALID (zero counts are data)",
              bool(f & H.VALID), True)
        check("     motion bit clear", bool(f & H.MOTION), False)

        print("\nrate bound - scaled by the REAL window, rejected not clamped")
        H.lib.t_init()
        H.update(burst(), 0)
        # 12000 counts/s * 10 ms + 2 slack = 122
        check("10 ms window: 122 counts accepted",
              bool(H.update(burst(dx=122, obs=1), 10000) & H.VALID), True)
        f = H.update(burst(dx=123, obs=2), 10000)
        check("10 ms window: 123 counts rejected (RATE)", bool(f & H.RATE), True)
        check("     not VALID", bool(f & H.VALID), False)
        check("     not added - sum still 122", H.get(SUM_DX), 122)
        check("     counted lost", H.get(N_LOST), 1)
        f = H.update(burst(dx=1, obs=3), 10000)
        check("the next good sample is flagged AFTER_LOST", bool(f & H.AFTER_LOST), True)
        check("     and is itself VALID", bool(f & H.VALID), True)
        f = H.update(burst(dx=1, obs=4), 10000)
        check("and the one after that is not", bool(f & H.AFTER_LOST), False)
        check("negative counts bounded the same way",
              bool(H.update(burst(dy=-123, obs=5), 10000) & H.RATE), True)
        # An SD card stall: 300 ms window. A fixed-10ms assumption would
        # call 3000 counts a 300000 count/s glitch.
        f = H.update(burst(dx=3000, obs=6), 300000)
        check("300 ms SD-stall window: 3000 counts VALID", bool(f & H.VALID), True)
        # 12000 * 1.9e6 = 2.28e10 overflows 32 bits; the limit must not wrap.
        f = H.update(burst(dx=20000, obs=7), 1900000)
        check("1.9 s window: 20000 counts VALID (64-bit limit)", bool(f & H.VALID), True)
        f = H.update(burst(dx=23000, obs=8), 1900000)
        check("1.9 s window: 23000 counts RATE", bool(f & H.RATE), True)

        print("\nwindow length itself")
        f = H.update(burst(obs=9), 0)
        check("dt 0 -> BAD_DT", bool(f & H.BAD_DT), True)
        check("     not VALID", bool(f & H.VALID), False)
        f = H.update(burst(obs=10), 2000001)
        check("dt past 2 s -> BAD_DT", bool(f & H.BAD_DT), True)
        f = H.update(burst(obs=11), 2000000)
        check("dt exactly 2 s accepted", bool(f & H.VALID), True)

        print("\na dead bus reads as a plausible sensor - and is caught")
        H.lib.t_init()
        H.update(burst(), 0)
        f = H.update([0x00] * 12, 10000)
        check("all 0x00 ('perfectly still') -> BUS_STUCK", bool(f & H.STUCK), True)
        check("     not VALID", bool(f & H.VALID), False)
        f = H.update([0xFF] * 12, 10000)
        check("all 0xFF (dx=dy=-1, squal 255) -> BUS_STUCK", bool(f & H.STUCK), True)
        check("     not VALID", bool(f & H.VALID), False)
        check("     sums untouched", (H.get(SUM_DX), H.get(SUM_DY)), (0, 0))

        print("\nstructural checks")
        f = H.update(burst(rmin=151, rmax=150), 10000)
        check("raw_min > raw_max -> INCONSISTENT", bool(f & H.INCONS), True)
        check("raw_min == raw_max is fine (flat scene)",
              bool(H.update(burst(rmin=90, rmax=90), 10000) & H.VALID), True)
        f = H.update(burst(dx=32767), 3000000 // 2)
        check("dx +32767 -> SATURATED", bool(f & H.SAT), True)
        f = H.update(burst(dy=-32768), 3000000 // 2)
        check("dy -32768 -> SATURATED", bool(f & H.SAT), True)

        print("\nscene quality is NOT a data fault")
        f = H.update(burst(squal=0, shutter=0x1FFF, rsum=1, rmax=3, rmin=0), 10000)
        check("squal 0, huge shutter: still VALID (logged, not judged)",
              bool(f & H.VALID), True)

        print("\nidentical-run diagnostic")
        H.lib.t_init()
        H.update(burst(obs=1), 0)
        for _ in range(3):
            H.update(burst(obs=2), 10000)
        check("3 identical in a row -> run 2", H.get(SAME_RUN), 2)
        H.update(burst(obs=3), 10000)
        check("a different burst resets the run", H.get(SAME_RUN), 0)
        check("     the max is kept", H.get(SAME_RUN_MAX), 2)

        print("\nre-init keeps the integral")
        H.lib.t_init()
        H.update(burst(), 0)
        H.update(burst(dx=10, dy=20, obs=1), 10000)
        H.lib.t_restart()
        f = H.update(burst(dx=999, dy=999, obs=2), 10000)
        check("first burst after restart is FIRST", bool(f & H.FIRST), True)
        check("     sums survive the restart", (H.get(SUM_DX), H.get(SUM_DY)), (10, 20))
        H.update(burst(dx=1, dy=2, obs=3), 10000)
        check("and keep accumulating", (H.get(SUM_DX), H.get(SUM_DY)), (11, 22))

        print("\na latched-but-unread burst")
        H.lib.t_init()
        H.lib.t_note_lost()
        check("on the FIRST burst: nothing lost", H.get(N_LOST), 0)
        f = H.update(burst(dx=4), 10000)
        check("     next sample is ordinary and VALID",
              (bool(f & H.VALID), bool(f & H.FIRST), bool(f & H.AFTER_LOST)),
              (True, False, False))
        H.lib.t_note_lost()
        check("later: one window lost", H.get(N_LOST), 1)
        f = H.update(burst(dx=4, obs=1), 10000)
        check("     and the next sample says so", bool(f & H.AFTER_LOST), True)

        print("\ninit tables - PixArt demo kit V3.20, not the older table")
        check("table A has 59 writes", H.lib.tbl_len(0), 59)
        check("table B has 14 writes", H.lib.tbl_len(1), 14)
        check("table A starts by selecting bank 0 (the prelude leaves 0x0E)",
              (H.lib.tbl_reg(0, 0), H.lib.tbl_val(0, 0)), (0x7F, 0x00))
        for idx, reg, new, old in ((33, 0x74, 0x21, 0x1F), (42, 0x65, 0x67, 0x60),
                                   (44, 0x63, 0x70, 0x78), (46, 0x48, 0x48, 0x58)):
            check(f"A[{idx}] 0x{reg:02X}=0x{new:02X} (old table: 0x{old:02X})",
                  (H.lib.tbl_reg(0, idx), H.lib.tbl_val(0, idx)), (reg, new))
        check("table A ends in bank 0x07", H.lib.bank_after(0, 0x0E), 0x07)
        check("table B ends in bank 0 - the burst and ID reads need it",
              H.lib.bank_after(1, 0x07), 0x00)
        tail = []
        for i in range(H.lib.tbl_len(1) - 1, -1, -1):
            if H.lib.tbl_reg(1, i) == 0x7F:
                break
            tail.append(H.lib.tbl_reg(1, i))
        check("at least 2 bank-0 writes after B's last bank select (sentinels)",
              len(tail) >= 2, True)
        regs_a = [H.lib.tbl_reg(0, i) for i in range(H.lib.tbl_len(0))]
        regs_b = [H.lib.tbl_reg(1, i) for i in range(H.lib.tbl_len(1))]
        check("no table writes Power_Up_Reset (0x3A) or Shutdown (0x3B)",
              any(r in (0x3A, 0x3B) for r in regs_a + regs_b), False)

        print("\nprelude arithmetic - PX4's sensorInit(), minus its uint8 wrap")

        def px4_c1(raw):
            c1 = raw + 14 if raw <= 28 else raw + 11
            return min(c1, 0x3F)

        agree = sum(H.lib.pre_c1(r) == px4_c1(r) for r in range(256))
        check("c1 matches min(raw + 14|11, 0x3F) for all 256 inputs", agree, 256)
        check("c1(28) = 42 (the +14 branch)", H.lib.pre_c1(28), 42)
        check("c1(29) = 40 (the +11 branch)", H.lib.pre_c1(29), 40)
        check("c1(250) = 0x3F, where uint8 maths would wrap to 5", H.lib.pre_c1(250), 0x3F)
        agree = sum(H.lib.pre_c2(r) == (r * 45) // 100 for r in range(256))
        check("c2 = raw * 45 / 100 for all 256 inputs", agree, 256)

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
