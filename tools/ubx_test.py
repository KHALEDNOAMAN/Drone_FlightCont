#!/usr/bin/env python3
"""Host test for components/tbsM10/ubx.c - the UBX frame parser, NAV-PVT
decoder and CFG-VALSET builder.

Same trick as tools/host_c.py: ubx.c is written freestanding (stdint only,
no libc, no ESP-IDF), so clang can compile the EXACT file that flies into
a DLL and ctypes can drive it. There is no host C runtime on this machine,
hence -nostdlib.

Every expected byte string below is built independently in Python with
struct.pack. Nothing is compared against the C's own idea of the answer,
which is the only way this is evidence rather than a tautology.

Run:  python tools/ubx_test.py
"""

import ctypes
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# UBX_SRC lets a mutation run point the compiler at a doctored copy of
# ubx.c/ubx.h. Without it this test would silently compile the real source
# no matter what a mutant harness wrote, and every mutant would "survive" -
# which is exactly what happened the first time it was run.
SRC = os.environ.get("UBX_SRC") or os.path.join(ROOT, "components", "tbsM10")

SHIM = r"""
int _fltused = 0;
#include "ubx.c"
#define EXPORT __declspec(dllexport)

EXPORT void  p_init(ubx_parser_t *p){ ubx_parser_init(p); }
EXPORT int   p_byte(ubx_parser_t *p, unsigned char b){ return ubx_parse_byte(p, b); }
EXPORT int   p_pvt(const ubx_parser_t *p, ubx_nav_pvt_t *o){ return ubx_decode_nav_pvt(p, o); }
EXPORT unsigned short p_build(unsigned char *o, unsigned short cap,
                              unsigned char c, unsigned char i,
                              const unsigned char *pl, unsigned short n){
    return ubx_build(o, cap, c, i, pl, n);
}
EXPORT unsigned short p_vbegin(unsigned char *pl, unsigned short cap, unsigned char l){
    return ubx_valset_begin(pl, cap, l);
}
EXPORT unsigned short p_vadd(unsigned char *pl, unsigned short cap, unsigned short n,
                             unsigned int key, unsigned int val, unsigned char vl){
    return ubx_valset_add(pl, cap, n, key, val, vl);
}
EXPORT void  p_cksum(const unsigned char *b, unsigned short n,
                     unsigned char *a, unsigned char *c){ ubx_checksum(b, n, a, c); }
EXPORT int   p_size(void){ return (int)sizeof(ubx_parser_t); }
EXPORT int   p_pvtsize(void){ return (int)sizeof(ubx_nav_pvt_t); }
EXPORT int   p_rf(const ubx_parser_t *p, ubx_mon_rf_t *o){ return ubx_decode_mon_rf(p, o); }
EXPORT int   p_sec(const ubx_parser_t *p, ubx_sec_sig_t *o){ return ubx_decode_sec_sig(p, o); }
EXPORT int   p_ack(const ubx_parser_t *p, unsigned char *a, unsigned char *c,
                   unsigned char *i){ return ubx_decode_ack(p, a, c, i); }
EXPORT int   p_cno(ubx_parser_t *p, ubx_cno_stats_t *o){ return ubx_take_cno_stats(p, o); }
EXPORT int   p_rfsize(void){ return (int)sizeof(ubx_mon_rf_t); }
EXPORT int   p_secsize(void){ return (int)sizeof(ubx_sec_sig_t); }
EXPORT int   p_cnosize(void){ return (int)sizeof(ubx_cno_stats_t); }
EXPORT int   p_streamed(const ubx_parser_t *p){ return (int)p->streamed; }
EXPORT unsigned int p_nstreamed(const ubx_parser_t *p){ return p->n_streamed; }
"""

UBX_MAX_PAYLOAD = 128


class CCno(ctypes.Structure):
    _fields_ = [
        ("itow_ms", ctypes.c_uint32),
        ("sum_cno", ctypes.c_uint32), ("sumsq_cno", ctypes.c_uint32),
        ("n_tracked", ctypes.c_uint16), ("n_used", ctypes.c_uint16),
        ("num_sigs", ctypes.c_uint8),
        ("min_cno", ctypes.c_uint8), ("max_cno", ctypes.c_uint8),
    ]


class CMonRf(ctypes.Structure):
    _fields_ = [
        ("n_blocks", ctypes.c_uint8), ("jamming_state", ctypes.c_uint8),
        ("ant_status", ctypes.c_uint8), ("jam_ind", ctypes.c_uint8),
        ("noise_per_ms", ctypes.c_uint16), ("agc_cnt", ctypes.c_uint16),
    ]


class CSecSig(ctypes.Structure):
    _fields_ = [
        ("version", ctypes.c_uint8),
        ("jam_det_enabled", ctypes.c_uint8), ("jamming_state", ctypes.c_uint8),
        ("spf_det_enabled", ctypes.c_uint8), ("spoofing_state", ctypes.c_uint8),
    ]


class CParser(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_uint8), ("cls", ctypes.c_uint8), ("id", ctypes.c_uint8),
        ("len", ctypes.c_uint16), ("idx", ctypes.c_uint16),
        ("ck_a", ctypes.c_uint8), ("ck_b", ctypes.c_uint8),
        ("skip", ctypes.c_uint16),
        ("buf", ctypes.c_uint8 * UBX_MAX_PAYLOAD),
        ("n_frames", ctypes.c_uint32), ("n_badsum", ctypes.c_uint32),
        ("n_oversize", ctypes.c_uint32), ("n_streamed", ctypes.c_uint32),
        ("streamed", ctypes.c_uint8),
        ("cno_acc", CCno), ("cno_stats", CCno),
        ("cno_active", ctypes.c_uint8), ("cno_ready", ctypes.c_uint8),
        ("cno_cur_cno", ctypes.c_uint8), ("cno_cur_flags", ctypes.c_uint8),
    ]


class CPvt(ctypes.Structure):
    _fields_ = [
        ("itow_ms", ctypes.c_uint32),
        ("lat_1e7", ctypes.c_int32), ("lon_1e7", ctypes.c_int32),
        ("h_msl_mm", ctypes.c_int32),
        ("h_acc_mm", ctypes.c_uint32), ("v_acc_mm", ctypes.c_uint32),
        ("vel_n_mms", ctypes.c_int32), ("vel_e_mms", ctypes.c_int32),
        ("vel_d_mms", ctypes.c_int32),
        ("s_acc_mms", ctypes.c_uint32),
        ("fix_type", ctypes.c_uint8), ("num_sv", ctypes.c_uint8),
        ("gnss_fix_ok", ctypes.c_uint8),
    ]


class HostUbx:
    def __init__(self, src_dir=SRC):
        self._tmp = tempfile.TemporaryDirectory()
        shim = os.path.join(self._tmp.name, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(self._tmp.name, "ubx.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry",
               "-O1", "-I", src_dir, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L = self.lib
        pp, po = ctypes.POINTER(CParser), ctypes.POINTER(CPvt)
        u8p = ctypes.POINTER(ctypes.c_uint8)
        L.p_init.argtypes = [pp]
        L.p_byte.argtypes = [pp, ctypes.c_uint8]
        L.p_byte.restype = ctypes.c_int
        L.p_pvt.argtypes = [pp, po]
        L.p_pvt.restype = ctypes.c_int
        L.p_build.argtypes = [u8p, ctypes.c_uint16, ctypes.c_uint8,
                              ctypes.c_uint8, u8p, ctypes.c_uint16]
        L.p_build.restype = ctypes.c_uint16
        L.p_vbegin.argtypes = [u8p, ctypes.c_uint16, ctypes.c_uint8]
        L.p_vbegin.restype = ctypes.c_uint16
        L.p_vadd.argtypes = [u8p, ctypes.c_uint16, ctypes.c_uint16,
                             ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint8]
        L.p_vadd.restype = ctypes.c_uint16
        L.p_cksum.argtypes = [u8p, ctypes.c_uint16, u8p, u8p]
        L.p_size.restype = ctypes.c_int
        L.p_pvtsize.restype = ctypes.c_int
        prf, psec, pcno = (ctypes.POINTER(CMonRf), ctypes.POINTER(CSecSig),
                           ctypes.POINTER(CCno))
        L.p_rf.argtypes = [pp, prf]
        L.p_rf.restype = ctypes.c_int
        L.p_sec.argtypes = [pp, psec]
        L.p_sec.restype = ctypes.c_int
        L.p_ack.argtypes = [pp, u8p, u8p, u8p]
        L.p_ack.restype = ctypes.c_int
        L.p_cno.argtypes = [pp, pcno]
        L.p_cno.restype = ctypes.c_int
        L.p_streamed.argtypes = [pp]
        L.p_streamed.restype = ctypes.c_int
        L.p_nstreamed.argtypes = [pp]
        L.p_nstreamed.restype = ctypes.c_uint32
        for f in ("p_rfsize", "p_secsize", "p_cnosize"):
            getattr(L, f).restype = ctypes.c_int
        # A silently mismatched layout makes every number below wrong in a
        # way that looks like a protocol bug.
        for name, got, want in (("ubx_parser_t", L.p_size(), ctypes.sizeof(CParser)),
                                ("ubx_nav_pvt_t", L.p_pvtsize(), ctypes.sizeof(CPvt)),
                                ("ubx_mon_rf_t", L.p_rfsize(), ctypes.sizeof(CMonRf)),
                                ("ubx_sec_sig_t", L.p_secsize(), ctypes.sizeof(CSecSig)),
                                ("ubx_cno_stats_t", L.p_cnosize(), ctypes.sizeof(CCno))):
            if got != want:
                raise SystemExit(f"{name} layout mismatch: C {got} B, ctypes {want} B")

    def __enter__(self):
        return self

    def __exit__(self, *a):
        h = self.lib._handle
        del self.lib
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
        self._tmp.cleanup()

    def feed(self, data, p=None):
        """Push bytes through the parser. Returns (parser, [frames]) where
        each frame is (cls, id, payload bytes) captured at the moment the
        parser reported it."""
        if p is None:
            p = CParser()
            self.lib.p_init(ctypes.byref(p))
        got = []
        for b in data:
            if self.lib.p_byte(ctypes.byref(p), b):
                got.append((p.cls, p.id, bytes(p.buf[:p.len])))
        return p, got

    def decode(self, p):
        o = CPvt()
        ok = self.lib.p_pvt(ctypes.byref(p), ctypes.byref(o))
        return ok, o


# ---------------------------------------------------------------- helpers
def cksum(body):
    """Fletcher-8, written here from the protocol spec - NOT from ubx.c."""
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def frame(cls, id_, payload):
    body = bytes([cls, id_]) + struct.pack("<H", len(payload)) + payload
    return b"\xb5\x62" + body + bytes(cksum(body))


PVT_FMT = "<IHBBBBBBIiBBBBiiiiIIiiiiiIIHH4xihH"
assert struct.calcsize(PVT_FMT) == 92


def pvt_payload(itow=0, fix=3, flags=1, nsv=0, lon=0, lat=0, hmsl=0,
                hacc=0, vacc=0, vn=0, ve=0, vd=0, sacc=0):
    return struct.pack(
        PVT_FMT,
        itow, 2026, 8, 30, 12, 0, 0, 0x37,   # iTOW, date, valid
        50, 0,                                # tAcc, nano
        fix, flags, 0, nsv,
        lon, lat, 0, hmsl,                    # lon, lat, height, hMSL
        hacc, vacc,
        vn, ve, vd, 0, 0,                     # velN/E/D, gSpeed, headMot
        sacc, 0,                              # sAcc, headAcc
        0, 0,                                 # pDOP, flags3
        0, 0, 0,                              # headVeh, magDec, magAcc
    )


MON_RF_HDR = "<BB2x"
MON_RF_BLK = "<BBBBIIHHBbBbB3x"
assert struct.calcsize(MON_RF_HDR) == 4
assert struct.calcsize(MON_RF_BLK) == 24


def mon_rf_payload(blocks):
    """blocks: list of (jamming_state, ant_status, noise, agc, jam_ind)."""
    out = struct.pack(MON_RF_HDR, 0, len(blocks))
    for i, (jam, ant, noise, agc, ji) in enumerate(blocks):
        out += struct.pack(MON_RF_BLK, i, jam & 0x03, ant, 0, 0, 0,
                           noise, agc, ji, 0, 0, 0, 0)
    return out


def sec_sig_v1(jam_en=1, jam=1, spf_en=1, spf=1):
    return struct.pack("<B3xB3xB3x", 1,
                       (jam_en & 1) | ((jam & 3) << 1),
                       (spf_en & 1) | ((spf & 7) << 1))


def sec_sig_v2(jam_en=1, jam=1, spf_en=1, spf=1, freqs=()):
    flags = ((jam_en & 1) | ((jam & 3) << 1) |
             ((spf_en & 1) << 3) | ((spf & 7) << 4))
    out = struct.pack("<BBBB", 2, flags, 0, len(freqs))
    for f in freqs:
        out += struct.pack("<I", f & 0x00FFFFFF)
    return out


SIG_HDR = "<IBB2x"
SIG_REC = "<BBBBhBBBBHI"
assert struct.calcsize(SIG_HDR) == 8
assert struct.calcsize(SIG_REC) == 16


def nav_sig_payload(sigs, itow=0):
    """sigs: list of (cno, pr_used). sigFlags bit 3 is prUsed."""
    out = struct.pack(SIG_HDR, itow, 0, len(sigs))
    for cno, used in sigs:
        out += struct.pack(SIG_REC, 0, 1, 0, 0, 0, cno, 4, 0, 0,
                           0x08 if used else 0x00, 0)
    return out


def cno_expect(sigs):
    """What the accumulator SHOULD produce - computed here, in Python, from
    the same list the frame was built from. Never read back from C."""
    used = [c for c, u in sigs if u and c > 0]
    return {
        "n_tracked": sum(1 for c, _ in sigs if c > 0),
        "n_used": len(used),
        "sum_cno": sum(used),
        "sumsq_cno": sum(c * c for c in used),
        "min_cno": min(used) if used else 0,
        "max_cno": max(used) if used else 0,
    }


FAIL = 0


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got  {got!r}\n         want {want!r}")


def main():
    with HostUbx() as H:
        print("\nchecksum + frame building")
        # Known-good frame from the u-blox spec: CFG-RATE poll, empty payload.
        f = H_build(H, 0x06, 0x08, b"")
        check("empty frame is 8 bytes", len(f), 8)
        check("built frame matches an independently-packed one",
              f, frame(0x06, 0x08, b""))
        pl = bytes(range(20))
        check("20-byte payload frame matches", H_build(H, 0x01, 0x07, pl),
              frame(0x01, 0x07, pl))

        print("\nparsing a clean stream")
        p, got = H.feed(frame(0x05, 0x01, b"\x06\x8a"))
        check("ACK-ACK parsed", got, [(0x05, 0x01, b"\x06\x8a")])
        check("frame counter", p.n_frames, 1)
        check("no bad checksums", p.n_badsum, 0)

        print("\nNAV-PVT decode - Bursa, moving NE and descending")
        # 40.1885 N, 29.0610 E; hMSL 155.2 m; 1.4 m/s N, 0.6 m/s E, 0.2 down
        payload = pvt_payload(itow=123456, fix=3, flags=1, nsv=12,
                              lat=401885000, lon=290610000, hmsl=155200,
                              hacc=1500, vacc=2400,
                              vn=1400, ve=600, vd=200, sacc=90)
        p, got = H.feed(frame(0x01, 0x07, payload))
        check("one frame surfaced", len(got), 1)
        ok, o = H.decode(p)
        check("decode accepted", ok, 1)
        check("lat", o.lat_1e7, 401885000)
        check("lon", o.lon_1e7, 290610000)
        check("hMSL mm", o.h_msl_mm, 155200)
        check("hAcc mm", o.h_acc_mm, 1500)
        check("vAcc mm", o.v_acc_mm, 2400)
        check("velN", o.vel_n_mms, 1400)
        check("velE", o.vel_e_mms, 600)
        check("velD", o.vel_d_mms, 200)
        check("sAcc", o.s_acc_mms, 90)
        check("iTOW", o.itow_ms, 123456)
        check("numSV", o.num_sv, 12)
        check("fixType", o.fix_type, 3)
        check("gnssFixOK", o.gnss_fix_ok, 1)

        print("\nsigned fields - southern/western hemisphere, climbing")
        payload = pvt_payload(lat=-338675000, lon=-1514500000, hmsl=-1500,
                              vn=-1400, ve=-600, vd=-250)
        p, _ = H.feed(frame(0x01, 0x07, payload))
        ok, o = H.decode(p)
        check("negative lat", o.lat_1e7, -338675000)
        check("negative lon", o.lon_1e7, -1514500000)
        check("below sea level", o.h_msl_mm, -1500)
        check("velD negative = climbing", o.vel_d_mms, -250)
        check("velN negative", o.vel_n_mms, -1400)

        print("\ngnssFixOK is separate from fixType")
        p, _ = H.feed(frame(0x01, 0x07, pvt_payload(fix=3, flags=0)))
        ok, o = H.decode(p)
        check("fixType 3 but flags bit0 clear -> not OK", o.gnss_fix_ok, 0)
        check("     fixType still reported", o.fix_type, 3)

        print("\nrobustness")
        good = frame(0x01, 0x07, pvt_payload(nsv=7))
        # split across two reads, mid-payload
        p, got = H.feed(good[:40])
        check("half a frame yields nothing", got, [])
        p, got = H.feed(good[40:], p)
        check("second half completes it", len(got), 1)

        bad = bytearray(good)
        bad[-1] ^= 0xFF
        p, got = H.feed(bytes(bad))
        check("corrupt checksum rejected", got, [])
        check("     counted as badsum", p.n_badsum, 1)
        check("     not counted as a frame", p.n_frames, 0)

        p, got = H.feed(b"\x00\xff\x12garbage\xb5\x00" + good)
        check("leading garbage then a good frame", len(got), 1)

        p, got = H.feed(b"\xb5\xb5\x62" + good[2:])
        check("B5 B5 62 - the resync trap - still parses", len(got), 1)

        nmea = b"$GNGGA,120000.00,4011.310,N,02903.660,E,1,12,0.8,155.2,M,,,,*5A\r\n"
        p, got = H.feed(nmea + good)
        check("NMEA text ignored, frame after it parsed", len(got), 1)

        print("\noversize length must not stall or overrun")
        big = b"\xb5\x62\x01\x07\x00\x08" + b"\xAA" * 4     # declares 2048 B
        p, got = H.feed(big)
        check("oversize not emitted", got, [])
        check("     counted", p.n_oversize, 1)
        # it should skip exactly 2048+2 bytes, then find the next frame
        p, got = H.feed(b"\xAA" * (2048 + 2 - 4) + good, p)
        check("     parser recovers and finds the next frame", len(got), 1)

        print("\nshort NAV-PVT is refused, not read past the end")
        p, got = H.feed(frame(0x01, 0x07, b"\x00" * 60))
        check("frame itself is valid", len(got), 1)
        ok, o = H.decode(p)
        check("but decode refuses it", ok, 0)

        p, got = H.feed(frame(0x01, 0x21, pvt_payload()))   # NAV-TIMEUTC
        ok, o = H.decode(p)
        check("wrong message id refused", ok, 0)

        print("\nCFG-VALSET builder")
        buf = (ctypes.c_uint8 * 256)()
        n = H.lib.p_vbegin(buf, 256, 0x01)
        check("header is 4 bytes", n, 4)
        check("     version 0, layer RAM", bytes(buf[:4]), b"\x00\x01\x00\x00")
        n = H.lib.p_vadd(buf, 256, n, 0x30210001, 100, 2)      # 100 ms = 10 Hz
        check("U2 key adds 6 bytes", n, 10)
        check("     key LE then value LE", bytes(buf[4:10]),
              struct.pack("<IH", 0x30210001, 100))
        n = H.lib.p_vadd(buf, 256, n, 0x20110021, 8, 1)        # airborne <4g
        check("U1 key adds 5 bytes", n, 15)
        check("     value byte", bytes(buf[10:15]),
              struct.pack("<IB", 0x20110021, 8))
        check("bad value width refused", H.lib.p_vadd(buf, 256, n, 0x1234, 1, 3), 0)
        check("overflow refused", H.lib.p_vadd(buf, 16, n, 0x1234, 1, 4), 0)
        check("add without begin refused", H.lib.p_vadd(buf, 256, 0, 0x1234, 1, 1), 0)

        print("\nMON-RF decode")
        p, _ = H.feed(frame(0x0A, 0x38,
                            mon_rf_payload([(2, 2, 310, 2900, 17)])))
        o = CMonRf()
        check("MON-RF accepted", H.lib.p_rf(ctypes.byref(p), ctypes.byref(o)), 1)
        check("     nBlocks", o.n_blocks, 1)
        check("     jammingState", o.jamming_state, 2)
        check("     antStatus", o.ant_status, 2)
        check("     noisePerMS", o.noise_per_ms, 310)
        check("     agcCnt", o.agc_cnt, 2900)
        check("     jamInd", o.jam_ind, 17)

        # A dual-band part must not be able to report a clean L1 while a
        # second band is jammed - the WORST state is what the guard sees.
        p, _ = H.feed(frame(0x0A, 0x38,
                            mon_rf_payload([(1, 2, 100, 3000, 0),
                                            (3, 2, 900, 1200, 200)])))
        H.lib.p_rf(ctypes.byref(p), ctypes.byref(o))
        check("worst jammingState across blocks", o.jamming_state, 3)
        check("     block 0's own agc still reported", o.agc_cnt, 3000)

        # nBlocks that the length cannot support must be refused, not walked.
        bad = struct.pack(MON_RF_HDR, 0, 4) + struct.pack(MON_RF_BLK,
                                                          0, 1, 2, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0, 0)
        p, _ = H.feed(frame(0x0A, 0x38, bad))
        check("nBlocks past the payload refused",
              H.lib.p_rf(ctypes.byref(p), ctypes.byref(o)), 0)
        p, _ = H.feed(frame(0x0A, 0x38, struct.pack(MON_RF_HDR, 0, 0)))
        check("zero blocks refused",
              H.lib.p_rf(ctypes.byref(p), ctypes.byref(o)), 0)
        # ...and refused for the RIGHT reason. Above, the payload is only 4
        # bytes, so the length check rejects it before nBlocks is ever read.
        # This one is long enough to pass every length test and still
        # declares zero blocks - the value the decoder would otherwise
        # report is a whole reading assembled from a block nobody sent.
        nb0 = struct.pack(MON_RF_HDR, 0, 0) + struct.pack(
            MON_RF_BLK, 0, 3, 2, 0, 0, 0, 900, 1200, 200, 0, 0, 0, 0)
        check("     and the payload was long enough to be read", len(nb0), 28)
        p, _ = H.feed(frame(0x0A, 0x38, nb0))
        check("     zero blocks refused on a full-length frame",
              H.lib.p_rf(ctypes.byref(p), ctypes.byref(o)), 0)

        print("\nSEC-SIG - two layouts behind one message id")
        s = CSecSig()
        p, _ = H.feed(frame(0x27, 0x09, sec_sig_v1(1, 3, 1, 2)))
        check("V1 accepted", H.lib.p_sec(ctypes.byref(p), ctypes.byref(s)), 1)
        check("     version", s.version, 1)
        check("     jamDetEnabled", s.jam_det_enabled, 1)
        check("     jammingState critical", s.jamming_state, 3)
        check("     spfDetEnabled", s.spf_det_enabled, 1)
        check("     spoofingState indicated", s.spoofing_state, 2)

        # jamDetEnabled and jammingState share a byte, so a decoder that
        # forgets the shift still gets the right answer whenever the two
        # happen to agree. This case makes them disagree: the enable bit is
        # CLEAR and the state is 2, so an unshifted read returns 0.
        p, _ = H.feed(frame(0x27, 0x09, sec_sig_v1(jam_en=0, jam=2)))
        H.lib.p_sec(ctypes.byref(p), ctypes.byref(s))
        check("     enable clear, state 2 - shift really applied",
              (s.jam_det_enabled, s.jamming_state), (0, 2))

        # spoofingState is a THREE-bit field. u-blox documents 0-3 today,
        # but a two-bit mask would silently fold a future 4-7 into a
        # confident wrong answer, so the decoder reports what was sent.
        p, _ = H.feed(frame(0x27, 0x09, sec_sig_v1(spf_en=1, spf=5)))
        H.lib.p_sec(ctypes.byref(p), ctypes.byref(s))
        check("     an out-of-range spoofingState is reported, not folded",
              s.spoofing_state, 5)

        p, _ = H.feed(frame(0x27, 0x09, sec_sig_v2(1, 2, 1, 3, freqs=(1575420,))))
        check("V2 accepted", H.lib.p_sec(ctypes.byref(p), ctypes.byref(s)), 1)
        check("     version", s.version, 2)
        check("     jammingState warning", s.jamming_state, 2)
        check("     spoofingState multiple", s.spoofing_state, 3)

        # The point of the version dispatch: V2's flags live at offset 1,
        # where V1 has a RESERVED byte. Decoding a V2 frame with V1's
        # offsets reads reserved zeros and reports "all clear" on a frame
        # that says it is being spoofed. This asserts that did not happen.
        v2 = sec_sig_v2(1, 3, 1, 3)
        check("     V1 offsets would have read zero here", v2[4] if len(v2) > 4 else 0, 0)
        p, _ = H.feed(frame(0x27, 0x09, v2))
        H.lib.p_sec(ctypes.byref(p), ctypes.byref(s))
        check("     but V2 dispatch reports critical", s.jamming_state, 3)

        p, _ = H.feed(frame(0x27, 0x09, b"\x09" + b"\x00" * 11))
        check("unknown version refused",
              H.lib.p_sec(ctypes.byref(p), ctypes.byref(s)), 0)
        p, _ = H.feed(frame(0x27, 0x09, sec_sig_v1()[:8]))
        check("truncated V1 refused",
              H.lib.p_sec(ctypes.byref(p), ctypes.byref(s)), 0)

        print("\nNAV-SIG C/N0 - buffered path (small frame)")
        small = [(44, 1), (39, 1), (51, 1), (0, 0), (22, 0)]
        pl = nav_sig_payload(small, itow=98765)
        check("     small frame really does fit the buffer", len(pl) <= 128, True)
        p, _ = H.feed(frame(0x01, 0x43, pl))
        c = CCno()
        check("stats published", H.lib.p_cno(ctypes.byref(p), ctypes.byref(c)), 1)
        exp = cno_expect(small)
        check("     iTOW", c.itow_ms, 98765)
        check("     numSigs", c.num_sigs, len(small))
        check("     n_tracked", c.n_tracked, exp["n_tracked"])
        check("     n_used (prUsed only)", c.n_used, exp["n_used"])
        check("     sum", c.sum_cno, exp["sum_cno"])
        check("     sumsq", c.sumsq_cno, exp["sumsq_cno"])
        check("     min", c.min_cno, exp["min_cno"])
        check("     max", c.max_cno, exp["max_cno"])
        check("taking it again returns nothing",
              H.lib.p_cno(ctypes.byref(p), ctypes.byref(c)), 0)

        print("\nNAV-SIG C/N0 - streamed path (frame far past the buffer)")
        big = [(30 + (i % 20), 1 if i % 4 else 0) for i in range(34)]
        pl = nav_sig_payload(big, itow=4242)
        check("     frame is genuinely oversize", len(pl) > 128, True)
        p, got = H.feed(frame(0x01, 0x43, pl))
        check("     counted as streamed", H.lib.p_nstreamed(ctypes.byref(p)), 1)
        check("     NOT counted as an oversize drop", p.n_oversize, 0)
        check("     streamed flag set", H.lib.p_streamed(ctypes.byref(p)), 1)
        check("     still reported as a valid frame", len(got), 1)
        check("stats published", H.lib.p_cno(ctypes.byref(p), ctypes.byref(c)), 1)
        exp = cno_expect(big)
        check("     iTOW", c.itow_ms, 4242)
        check("     numSigs", c.num_sigs, len(big))
        check("     n_tracked", c.n_tracked, exp["n_tracked"])
        check("     n_used", c.n_used, exp["n_used"])
        check("     sum", c.sum_cno, exp["sum_cno"])
        check("     sumsq", c.sumsq_cno, exp["sumsq_cno"])
        check("     min", c.min_cno, exp["min_cno"])
        check("     max", c.max_cno, exp["max_cno"])

        # The whole reason the streamed path is not UBX_ST_SKIP.
        print("\nstreamed frame with a corrupt checksum publishes NOTHING")
        good = frame(0x01, 0x43, nav_sig_payload(big, itow=777))
        bad = good[:-1] + bytes([good[-1] ^ 0xFF])
        p, got = H.feed(bad)
        check("     frame rejected", len(got), 0)
        check("     counted as badsum", p.n_badsum, 1)
        check("     no stats published",
              H.lib.p_cno(ctypes.byref(p), ctypes.byref(c)), 0)

        print("\na streamed frame must not let a stale NAV-PVT re-decode")
        pvt = frame(0x01, 0x07, pvt_payload(itow=1, fix=3, flags=1, nsv=9,
                                            lat=401885000, lon=290610000))
        p, _ = H.feed(pvt)
        ok, o1 = H.decode(p)
        check("     NAV-PVT decodes normally", ok, 1)
        p, _ = H.feed(frame(0x01, 0x43, nav_sig_payload(big)), p=p)
        ok, _ = H.decode(p)
        check("     after a streamed frame, PVT decode refuses", ok, 0)

        # White-box, and deliberately so. Today NAV-SIG is the only streamed
        # message and its id already fails the NAV-PVT check, which means a
        # black-box test cannot tell whether the `streamed` guard is doing
        # anything. The guard exists for the NEXT message that gets streamed
        # - at which point a buffered decoder reading buf[] would hand back
        # the previous frame's contents as fresh data. This asserts the
        # invariant directly: streamed set means buffered decode refuses,
        # whatever the class and id happen to say.
        p, _ = H.feed(pvt)
        ok, _ = H.decode(p)
        check("     a real NAV-PVT decodes with streamed clear", ok, 1)
        p.streamed = 1
        ok, _ = H.decode(p)
        check("     the SAME frame is refused with streamed set", ok, 0)
        o = CMonRf()
        check("     MON-RF decode honours it too",
              H.lib.p_rf(ctypes.byref(p), ctypes.byref(o)), 0)
        s = CSecSig()
        check("     SEC-SIG decode honours it too",
              H.lib.p_sec(ctypes.byref(p), ctypes.byref(s)), 0)
        p.streamed = 0

        print("\nparser recovers after a streamed frame")
        stream = frame(0x01, 0x43, nav_sig_payload(big))
        p, got = H.feed(stream + pvt)
        check("     both frames seen", len(got), 2)
        ok, o2 = H.decode(p)
        check("     the NAV-PVT after it still decodes", ok, 1)
        check("     with the right latitude", o2.lat_1e7, 401885000)

        # The answer to every CFG-VALSET. The driver sends the
        # core config and each guard input as separate VALSETs and waits for
        # this per message, so a NAK read as an ACK - or any other frame read
        # as either - would report the wrong group as working.
        print("\nACK / NAK - the module's answer to a CFG-VALSET")

        def ack(fr, streamed=False):
            p, _ = H.feed(fr)
            if streamed:
                p.streamed = 1
            a, c, i = ctypes.c_uint8(9), ctypes.c_uint8(9), ctypes.c_uint8(9)
            ok = H.lib.p_ack(ctypes.byref(p), ctypes.byref(a),
                             ctypes.byref(c), ctypes.byref(i))
            return ok, a.value, c.value, i.value

        check("ACK-ACK for CFG-VALSET: acked, answers 06 8A",
              ack(frame(0x05, 0x01, b"\x06\x8a")), (1, 1, 0x06, 0x8A))
        check("ACK-NAK for CFG-VALSET: a NAK, answers 06 8A",
              ack(frame(0x05, 0x00, b"\x06\x8a")), (1, 0, 0x06, 0x8A))
        check("a NAK names the message it answers (06 00, not 06 8A)",
              ack(frame(0x05, 0x00, b"\x06\x00")), (1, 0, 0x06, 0x00))
        check("a NAV-PVT is not an ACK",
              ack(frame(0x01, 0x07, pvt_payload()))[0], 0)
        check("class 06 id 01 with an ACK-shaped payload is not an ACK",
              ack(frame(0x06, 0x01, b"\x06\x8a"))[0], 0)
        check("an unknown ACK-class id (05 02) is refused",
              ack(frame(0x05, 0x02, b"\x06\x8a"))[0], 0)
        check("a 3-byte ACK is refused", ack(frame(0x05, 0x01, b"\x06\x8a\x00"))[0], 0)
        check("a 1-byte ACK is refused", ack(frame(0x05, 0x01, b"\x06"))[0], 0)
        check("a streamed frame is never an ACK",
              ack(frame(0x05, 0x01, b"\x06\x8a"), streamed=True)[0], 0)

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


def H_build(H, cls, id_, payload):
    out = (ctypes.c_uint8 * 256)()
    pl = (ctypes.c_uint8 * max(1, len(payload)))(*payload)
    n = H.lib.p_build(out, 256, cls, id_, pl, len(payload))
    return bytes(out[:n])


if __name__ == "__main__":
    sys.exit(main())
