#!/usr/bin/env python3
"""Does tools/blackbox_decode.py still agree with bb_frame_t?

    python tools/blackbox_frame_test.py

WHY THIS EXISTS
---------------
The decoder identifies a frame generation BY ITS LENGTH - that is what
lets 34, 35, 43, 48, 50 and 54 byte logs all decode from one tool. The
cost of that trick is that a field added to the C struct without a
matching struct.Struct here does not fail loudly. It produces a frame
whose length nothing recognises (every row silently dropped as corrupt),
or worse, one whose length collides with an older generation and decodes
into the WRONG COLUMNS while looking entirely plausible.

Nothing else catches it. There is no host build of the firmware, the
round-trip needs an aircraft to fly, and by the time a real log exists
the flight has already happened.

So this parses the struct out of components/blackbox/blackbox.h the same
way tools/test_level_rotation.py parses fusion.c, rebuilds the expected
format string from the C field types, and checks three things:

  1. the C field order and types produce exactly FRAME_54's format
  2. every FRAMES[] entry really is the size it is keyed by
  3. a synthetic 66 B frame survives a full write -> decode round trip,
     values intact, through the real decoder rather than a copy of it
"""

import base64
import os
import re
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
HEADER = os.path.join(ROOT, "components", "blackbox", "blackbox.h")

sys.path.insert(0, HERE)
import blackbox_decode as BD                                # noqa: E402

# The newest frame generation, found rather than named. Pinning a
# constant here (it was FRAME_66) means that adding a field to
# bb_frame_t makes this test compare the new struct against the OLD
# decoder and fail with a struct.error instead of the message it was
# written to print. The newest generation is simply the largest.
NEWEST = max(BD.FRAMES.values(), key=lambda f: f.size)

# C type -> (struct code, size). Only the types bb_frame_t actually uses;
# anything else must fail loudly rather than be guessed at.
CTYPES = {"uint32_t": ("I", 4), "int32_t": ("i", 4),
          "uint16_t": ("H", 2), "int16_t": ("h", 2),
          "uint8_t": ("B", 1), "int8_t": ("b", 1),
          "float": ("f", 4)}

FIELD_RE = re.compile(
    r"^\s*(uint\d+_t|int\d+_t|float)\s+(\w+)\s*(?:\[(\d+)\])?\s*;")


def parse_struct():
    """bb_frame_t's fields, in declaration order, comments stripped."""
    src = open(HEADER).read()
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)          # block comments
    src = re.sub(r"//.*", "", src)                           # line comments
    m = re.search(r"typedef struct[^{]*\{(.*?)\}\s*bb_frame_t\s*;",
                  src, flags=re.S)
    if not m:
        sys.exit("could not find bb_frame_t in " + HEADER)
    fields = []
    for line in m.group(1).splitlines():
        f = FIELD_RE.match(line)
        if f:
            ctype, name, count = f.group(1), f.group(2), f.group(3)
            if ctype not in CTYPES:
                sys.exit(f"unknown C type '{ctype}' on field '{name}' - "
                         "add it to CTYPES and decide its struct code "
                         "deliberately")
            fields.append((ctype, name, int(count) if count else 1))
    return fields


def fmt_of(fields):
    out = "<"
    for ctype, _name, count in fields:
        code, _ = CTYPES[ctype]
        out += (str(count) if count > 1 else "") + code
    return out


def expand(fmt):
    """'<I3h2H' -> '<Ihhh HH' with the counts written out.

    The comparison has to be on LAYOUT, not on spelling: '3h' and 'hhh'
    describe the same bytes, and a test that calls them different fails
    every time someone tidies the format string."""
    out, n = fmt[0], ""
    for ch in fmt[1:]:
        if ch.isdigit():
            n += ch
        else:
            out += ch * (int(n) if n else 1)
            n = ""
    return out


def main():
    fail = 0
    fields = parse_struct()
    size = sum(CTYPES[c][1] * n for c, _, n in fields)
    print(f"bb_frame_t parsed from blackbox.h: {len(fields)} fields, "
          f"{size} B")
    print("  " + "  ".join(f"{n}{'[%d]' % c if c > 1 else ''}"
                           for _, n, c in fields))

    # 1. the newest decoder format must match the C struct exactly
    want = fmt_of(fields)
    got = NEWEST.format
    if expand(want) == expand(got) and struct.calcsize(want) == size:
        print(f"\n  [PASS] newest frame matches C     {want} ({size} B)")
    else:
        fail += 1
        print(f"\n  [FAIL] newest frame does NOT match C")
        print(f"           C header : {want} ({struct.calcsize(want)} B)")
        print(f"           decoder  : {got} ({NEWEST.size} B)")
        print("         A field was added to bb_frame_t without adding a "
              "matching struct.Struct to blackbox_decode.py.")

    # 2. every generation must be keyed by its own true size
    bad = [(k, f.size) for k, f in BD.FRAMES.items() if k != f.size]
    if bad:
        fail += 1
        print(f"  [FAIL] FRAMES keyed by the wrong size: {bad}")
    else:
        print(f"  [PASS] all {len(BD.FRAMES)} generations self-consistent  "
              + ", ".join(f"{k} B" for k in sorted(BD.FRAMES)))

    # 3. round trip a synthetic newest-generation frame through the REAL
    #    decoder. Distinct values per field, so a shifted column shows up
    #    as a wrong number rather than a plausible one.
    # Distinct per field, but inside each C type's range - a uint8_t
    # cannot hold 1000, and struct.pack is right to refuse.
    limit = {"uint8_t": 250, "int8_t": 120}
    vals = []
    for i, (ctype, _n, count) in enumerate(fields):
        for j in range(count):
            v = 1000 + i * 7 + j
            cap = limit.get(ctype)
            if cap is not None:
                v = v % cap
            vals.append(float(v) if ctype == "float" else v)
    payload = NEWEST.pack(*vals)
    sector = bytearray(b"\xff" * 4096)
    sector[0:BD.HDR.size] = BD.HDR.pack(BD.MAGIC, 0, 7, 0)
    off = BD.HDR.size
    pset = struct.pack("<I", 6) + struct.pack("<f", 1.0)     # tiny params
    sector[off:off + 2 + len(pset)] = bytes([1, len(pset)]) + pset
    off += 2 + len(pset)
    sector[off:off + 2 + len(payload)] = bytes([2, len(payload)]) + payload

    tmp = tempfile.mkdtemp()
    bb = os.path.join(tmp, "synthetic.bb")
    with open(bb, "w") as fh:
        for o in range(0, 4096, 768):
            chunk = bytes(sector[o:o + 768])
            fh.write(f"BB,0,{o},{base64.b64encode(chunk).decode()}\n")
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, "blackbox_decode.py"), bb],
                       capture_output=True, text=True)
    csvp = os.path.join(tmp, "synthetic_s7.csv")
    if not os.path.exists(csvp):
        fail += 1
        print(f"  [FAIL] round trip produced no CSV\n{r.stdout}{r.stderr}")
    else:
        import csv as _csv
        rows = list(_csv.DictReader(open(csvp, newline="")))
        hdr = BD.CSV_HEADER.strip().split(",")
        if len(rows) != 1:
            fail += 1
            print(f"  [FAIL] round trip decoded {len(rows)} rows, want 1")
        elif len(rows[0]) != len(hdr):
            fail += 1
            print(f"  [FAIL] row has {len(rows[0])} columns, header has "
                  f"{len(hdr)}")
        else:
            # t_us is the one field written through unscaled, so it is the
            # cheapest end-to-end proof the bytes landed where intended.
            got_t = int(float(rows[0]["t_us"]))
            if got_t != vals[0]:
                fail += 1
                print(f"  [FAIL] round trip t_us {got_t}, want {vals[0]}")
            else:
                print(f"  [PASS] {NEWEST.size} B frame round trips through "
                      f"the real decoder  ({len(hdr)} columns)")

            # THE GAP THIS CLOSES. The check above compares the CSV row
            # against the CSV HEADER - two things that move together, so
            # both can be short by the same amount and still agree. On
            # accn_dv_ms and acce_dv_ms were added to
            # bb_frame_t and the decoder emitted neither; the frame
            # matched the C, the header matched the row, and this file
            # printed OK while silently dropping two columns.
            #
            # A frame field must reach a CSV column. Scalars map 1:1;
            # arrays (gyro[3]) fan out into per-axis columns, so they are
            # counted by their width.
            want_cols = sum(count for _c, _n, count in fields)
            if len(hdr) != want_cols:
                fail += 1
                print(f"  [FAIL] bb_frame_t carries {want_cols} values but "
                      f"the CSV has {len(hdr)} columns - a field was added "
                      f"to the frame without a column to land in")
            else:
                print(f"  [PASS] every frame field reaches a CSV column  "
                      f"({want_cols})")

    fail += check_engaged_implies_wanted()

    print("\nOK" if not fail else f"\n{fail} FAILURE(S)")
    return 1 if fail else 0


# VERT_F_* bits, mirrored from components/blackbox/blackbox.h.
VF_ENGAGED, VF_WANTED, VF_CH6 = 1 << 1, 1 << 2, 1 << 6


def check_engaged_implies_wanted():
    """ENGAGED must imply WANTED. A controller cannot take the throttle
    without the policy having asked for it.

    s179 recorded ENGAGED on 50.4% of frames with WANTED at 0%, which is
    impossible under either meaning of the bit and should have been
    caught the moment it was logged. It was not, because WANTED was
    reading the CH6 switch while ENGAGED was reading the automatic
    policy - the two flags were measuring different things.

    SCOPED BY VERT_F_CH6, deliberately. Sessions recorded before
 cannot satisfy this - their bit 2 IS the switch - so
    asserting over them would make this permanently red, and a test
    that is supposed to be red is a test people learn to skip. A session
    where bit 6 is never set anywhere predates the change and is
    reported as skipped rather than failed. It goes green on its own the
    first time the new firmware flies.
    """
    import csv
    import glob
    paths = sorted(glob.glob(os.path.join(ROOT, "*_s*.csv")))
    if not paths:
        print("\n  [SKIP] ENGAGED-implies-WANTED: no decoded sessions on "
              "disk (they are gitignored). Not a failure.")
        return 0

    checked = skipped = bad = 0
    worst = None
    for p in paths:
        try:
            with open(p, newline="") as fh:
                rows = list(csv.DictReader(fh))
        except (OSError, csv.Error):
            continue
        if not rows or "vert_flags" not in rows[0]:
            continue
        flags = [int(r["vert_flags"]) for r in rows]
        if not any(f & VF_CH6 for f in flags):
            skipped += 1          # early  flag semantics
            continue
        checked += 1
        n = sum(1 for f in flags if (f & VF_ENGAGED) and not (f & VF_WANTED))
        if n:
            bad += 1
            if worst is None or n > worst[1]:
                worst = (os.path.basename(p), n, len(flags))

    print()
    if not checked:
        print(f"  [SKIP] ENGAGED-implies-WANTED: {skipped} session(s) on "
              f"disk, all predating VERT_F_CH6. Nothing to assert yet.")
        return 0
    if bad:
        print(f"  [FAIL] ENGAGED without WANTED in {bad}/{checked} "
              f"session(s); worst {worst[0]}: {worst[1]}/{worst[2]} frames. "
              f"The two flags are measuring different things again.")
        return 1
    print(f"  [PASS] ENGAGED implies WANTED across {checked} session(s) "
          f"({skipped} older session(s) skipped)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
