#!/usr/bin/env python3
"""Host test for the C6 -> S3 command-link line assembler
(components/telemetry/telem_line.c), added.

The real file is compiled - not a model of it - freestanding with clang,
like the other host tests. The byte streams are the ones the dashboard
actually produced: the S3 answered "unknown command: " followed by 0xFF
bytes and then the real command ("...MOT,1,1050,3000"), because line noise
arriving while the link was idle had no newline and sat in the buffer
until the next real command was glued onto it.

TELEM_LINE_SRC points the compiler at a doctored copy for mutation runs.

Run:  python tools/telem_line_test.py
"""

import ctypes
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.environ.get("TELEM_LINE_SRC") or os.path.join(
    ROOT, "components", "telemetry")

STUBS = {
    "string.h": r"""
#pragma once
#include <stddef.h>
void *memcpy(void *d, const void *s, size_t n);
void *memset(void *d, int c, size_t n);
""",
}

SHIM = r"""
int _fltused = 0;
#include <stddef.h>
#include <stdint.h>
void *memcpy(void *d, const void *s, size_t n)
{ unsigned char *a = d; const unsigned char *b = s; while (n--) *a++ = *b++; return d; }
void *memset(void *d, int c, size_t n)
{ unsigned char *a = d; while (n--) *a++ = (unsigned char)c; return d; }

#include "telem_line.c"
#define EXPORT __declspec(dllexport)

static telem_line_t S;
static char out[16][TELEM_LINE_MAX];
static int n_out;

EXPORT void t_init(void) { telem_line_init(&S); n_out = 0; }
EXPORT void t_feed(const unsigned char *b, int n)
{
    for (int i = 0; i < n; i++) {
        char *l = telem_line_feed(&S, b[i]);
        if (l && n_out < 16) {
            int k = 0;
            while (l[k] && k < TELEM_LINE_MAX - 1) { out[n_out][k] = l[k]; k++; }
            out[n_out][k] = 0;
            n_out++;
        }
    }
}
EXPORT void t_silence(void) { telem_line_silence(&S); }
EXPORT int  t_n_out(void) { return n_out; }
EXPORT const char *t_out(int i) { return out[i]; }
EXPORT void t_clear_out(void) { n_out = 0; }
EXPORT void t_stats(unsigned int *o)
{
    o[0] = S.lines; o[1] = S.junk_bytes; o[2] = (unsigned int)S.junk_head_n;
    for (int i = 0; i < TELEM_JUNK_HEAD; i++) o[3 + i] = S.junk_head[i];
}
EXPORT int t_max(void) { return TELEM_LINE_MAX; }
"""


class Host:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        d = self._tmp.name
        for rel, text in STUBS.items():
            p = os.path.join(d, rel)
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w") as fh:
                fh.write(text)
        shim = os.path.join(d, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(d, "tl.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry", "-O1", "-w",
               "-I", d, "-I", SRC, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L = self.lib
        L.t_feed.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
        L.t_out.argtypes = [ctypes.c_int]
        L.t_out.restype = ctypes.c_char_p
        L.t_stats.argtypes = [ctypes.POINTER(ctypes.c_uint)]
        for f in ("t_n_out", "t_max"):
            getattr(L, f).restype = ctypes.c_int

    def __enter__(self):
        return self

    def __exit__(self, *a):
        h = self.lib._handle
        del self.lib
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
        self._tmp.cleanup()

    def feed(self, data):
        b = bytes(data)
        self.lib.t_feed((ctypes.c_uint8 * len(b))(*b), len(b))

    def lines(self):
        """-> the lines handed out since the last call, then forgets them."""
        got = [self.lib.t_out(i).decode("latin-1")
               for i in range(self.lib.t_n_out())]
        self.lib.t_clear_out()
        return got

    def stats(self):
        o = (ctypes.c_uint * 9)()
        self.lib.t_stats(o)
        o = list(o)
        return {"lines": o[0], "junk": o[1], "head": o[3:3 + o[2]]}


FAIL = 0


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got {got!r}  want {want!r}")


FF7 = b"\xff" * 7


def main():
    with Host() as H:
        MAX = H.lib.t_max()

        print("\nplain commands still arrive")
        H.lib.t_init()
        H.feed(b"STATS\n")
        check("STATS\\n -> one line", H.lines(), ["STATS"])
        H.feed(b"HELP\r\n")
        check("CRLF -> one line, no empty second line", H.lines(), ["HELP"])
        H.feed(b"ST")
        H.feed(b"ATS\n")
        check("split across two reads with no silence between: joined",
              H.lines(), ["STATS"])
        H.feed(b"SET rate_kp 0.5\n")
        check("spaces are printable - SET with spaces passes",
              H.lines(), ["SET rate_kp 0.5"])
        H.feed(b"~}|{\n")
        check("0x7E and the rest of printable ASCII pass", H.lines(), ["~}|{"])
        check("lines counted, nothing dropped",
              (H.stats()["lines"], H.stats()["junk"]), (5, 0))

        print("\nTHE BUG: line noise glued onto the next command")
        H.lib.t_init()
        H.feed(FF7)                               # the idle-time glitches
        H.lib.t_silence()                         # ...then the link is quiet
        H.feed(b"MOT,1,1050,3000\n")              # the command the author typed
        check("junk, silence, command -> the command runs FIRST time",
              H.lines(), ["MOT,1,1050,3000"])
        s = H.stats()
        check("...the 7 junk bytes counted and sampled",
              (s["junk"], s["head"]), (7, [0xFF] * 6))
        H.lib.t_init()
        H.feed(FF7 + b"MOT,1,1050,3000\n")        # no silence between at all
        check("junk glued on with NO silence: the line is refused, not run",
              H.lines(), [])
        check("...all 22 bytes counted as junk", H.stats()["junk"], 22)
        H.feed(b"MOT,1,1050,3000\n")
        check("...and the retyped command works", H.lines(),
              ["MOT,1,1050,3000"])

        print("\nanything outside 0x20..0x7E is noise")
        for bad, name in ((b"\x00", "NUL"), (b"\x7f", "DEL"),
                          (b"\x1b", "ESC"), (b"\x80", "0x80"),
                          (b"\t", "TAB")):
            H.lib.t_init()
            H.feed(b"VB" + bad + b"AT\n")
            check(f"{name} inside a line: refused", H.lines(), [])

        print("\nsilence")
        H.lib.t_init()
        H.lib.t_silence()
        check("silence with nothing pending: no junk", H.stats()["junk"], 0)
        H.feed(b"GYRO")
        H.lib.t_silence()
        H.feed(b"CAL\n")
        check("a printable partial line is dropped by silence too "
              "(a real command is one write)", H.lines(), ["CAL"])
        check("...and counted", H.stats()["junk"], 4)

        print(f"\noverlong lines (buffer {MAX})")
        H.lib.t_init()
        H.feed(b"A" * (MAX + 40) + b"\n")
        check("an overlong printable line is dropped whole - its tail is "
              "NOT run as a command", H.lines(), [])
        check("...every byte counted", H.stats()["junk"], MAX + 40)
        H.feed(b"VBAT\n")
        check("...and the next command works", H.lines(), ["VBAT"])
        H.lib.t_init()
        H.feed(b"A" * (MAX - 1) + b"\n")
        check(f"{MAX - 1} chars is still a line (the longest that fits)",
              len(H.lines()), 1)
        H.lib.t_init()
        H.feed(b"A" * (MAX + 5))
        H.lib.t_silence()
        H.feed(b"TOF\n")
        check("overlong then silence: skipping ends, next command works",
              H.lines(), ["TOF"])

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
