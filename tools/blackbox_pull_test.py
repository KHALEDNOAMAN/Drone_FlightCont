#!/usr/bin/env python3
"""Host test for tools/blackbox_pull.py's reply loop and port opening.

THE BUG : read_until_eof ended only on BBEOF or a serial timeout,
but the S3 prints IBUS/GPS/TELEMETRY log lines every second, so the timeout
never fired. One BBEOF lost to the USB console's dropped bytes and the loop
spun forever - a range pull sat at sector ~10358 for 26 minutes with the S3
perfectly healthy. Now it gives up after stall_s seconds without a dump line.

The fake port below replays exactly that: dump lines, then log lines with no
end. It raises after 5000 reads so the OLD loop fails instead of hanging.

BBPULL_SRC points the import at a doctored copy for mutation runs.

Run:  python tools/blackbox_pull_test.py
"""

import base64
import importlib.util
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.environ.get("BBPULL_SRC") or HERE

spec = importlib.util.spec_from_file_location(
    "blackbox_pull", os.path.join(SRC, "blackbox_pull.py"))
bp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bp)

LOG = b"I (2524339) IBUS: ibus link: rx=4160 B, frames=130, badsum=0 (GPIO 8)\n"


def bb(seq, off):
    n = 256 if off == 3840 else 768
    b64 = base64.b64encode(bytes([seq & 0xFF]) * n).decode()
    return f"BB,{seq},{off},{b64}\n".encode()


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakePort:
    """Serves `lines`, then `tail` forever (b"" = serial timeout). Every
    readline costs `step` seconds on the fake clock."""

    def __init__(self, lines, tail, clock, step=0.1):
        self.lines, self.tail, self.clock, self.step = list(lines), tail, clock, step
        self.reads = 0

    def readline(self):
        self.reads += 1
        self.clock.t += self.step
        if self.reads > 5000:
            raise RuntimeError("read_until_eof never returned")
        return self.lines.pop(0) if self.lines else self.tail


FAIL = 0


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got {got!r}  want {want!r}")


def run(lines, tail, stall=5.0, step=0.1):
    clk = Clock()
    port = FakePort(lines, tail, clk, step)
    chunks = {}
    try:
        r = bp.read_until_eof(port, chunks, stall_s=stall, clock=clk)
    except RuntimeError as e:
        r = f"HUNG ({e})"
    except TypeError as e:                    # an old signature
        r = f"HUNG (no stall rule: {e})"
    return r, chunks, port.reads


def main():
    sector = [bb(7, o) for o in bp.OFFSETS]

    print("\nnormal replies")
    r, ch, _ = run(sector + [b"BBEOF,1\n"], LOG)
    check("six chunks + BBEOF -> the count", r, 1)
    check("...all six chunks kept", len(ch), 6)
    mixed = []
    for line in sector:
        mixed += [LOG, LOG, line]             # logs interleaved, not a stall
    r, ch, _ = run(mixed + [b"BBEOF,1\n"], LOG)
    check("log lines between dump lines do not end the reply", (r, len(ch)), (1, 6))
    r, _, reads = run([], b"")
    check("serial timeout -> None at once", (r, reads), (None, 1))

    print("\nTHE BUG: BBEOF lost, the S3 keeps logging")
    r, ch, reads = run(sector, LOG, stall=5.0, step=0.1)
    check("returns None instead of looping forever", r, None)
    check("...after about stall_s of log lines (50 +- 2 reads)",
          48 <= reads - len(sector) <= 52, True)
    check("...keeping the chunks that did arrive", len(ch), 6)
    r, _, reads = run([], LOG, stall=2.0, step=0.1)
    check("no reply at all, logs only: None after ~2 s", (r, 18 <= reads <= 22),
          (None, True))

    print("\n--no-reset opens with DTR/RTS already low")
    events = []

    class FakeSerial:
        def __init__(self, *a, **k):
            events.append(("init", a, tuple(sorted(k))))

        def __setattr__(self, name, value):
            events.append((name, value))
            object.__setattr__(self, name, value)

        def open(self):
            events.append(("open",))

    had = bp.serial
    if bp.serial is None:                     # CI is stdlib-only: no pyserial
        bp.serial = types.SimpleNamespace(Serial=None)
    real = bp.serial.Serial
    bp.serial.Serial = FakeSerial
    try:
        opener = getattr(bp, "open_port", None)
        if opener is None:
            check("open_port exists", False, True)
        else:
            events.clear()
            opener("COM8", True)
            names = [e[0] for e in events]
            ok = ("open" in names and "dtr" in names and "rts" in names and
                  names.index("dtr") < names.index("open") and
                  names.index("rts") < names.index("open") and
                  ("dtr", False) in events and ("rts", False) in events)
            check("dtr=False and rts=False set BEFORE open()", ok, True)
            check("...and the port is not opened by the constructor",
                  events[0], ("init", (), ()))
            events.clear()
            opener("COM8", False)
            check("default still opens the old way (constructor with port)",
                  events[0][0] == "init" and events[0][1][:1] == ("COM8",), True)
    finally:
        bp.serial.Serial = real
        bp.serial = had

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
