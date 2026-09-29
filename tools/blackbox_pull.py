#!/usr/bin/env python3
"""Pull the blackbox over USB with transport verification.

    python tools/blackbox_pull.py COM4 flight.bb
    python tools/blackbox_pull.py COM4          # -> 1742.bb
    python tools/blackbox_pull.py COM8 --no-reset --from 10158

Omit the filename and the pull is stamped with the current date and
time, so the newest .bb on disk is always the newest flight - no more
guessing which of two hand-picked names (flight.bb, ladder.bb) is
more recent.

--no-reset opens the port with DTR/RTS held low, so the S3 keeps running
(and a frozen IMU stays frozen for measurement; a warm reset is also what
locks the DPS310 out). --from SEQ also fetches every sector from SEQ up to
the start of BBDUMP's window, one BBDUMP,<seq> at a time (~15 sectors/s):
the device only streams its newest 3000 sectors, and a session that began
before them does not decode ("no sessions found"). SEQ
is usually the first sector after the previous pull.

Full BBDUMP first, then every sector is verified: all six chunk
offsets present and each base64 payload the exact expected length.
Anything missing or mangled is re-requested per sector with
BBDUMP,<seq>, up to 3 rounds. Holes in the seq window (a sector whose
six lines were ALL lost leaves only a gap between its neighbours) are
re-requested the same way, and the device's BBEOF sector count is used
to probe for lost sectors at the old end of the ring.

Why: the USB console drops bytes under load. Found the
same flight pulled twice gave different frame counts, and lost chunks
decoded as ghost frames (thr=0, motors=0xFFFF) plus ~0.5 s gaps.

Needs pyserial (pip install pyserial). The D, stream is silenced first
because BBDUMP prints from a different task than the 50 Hz D, stream,
and interleaved printf output could split a dump line.

Decode the result with tools/blackbox_decode.py.
"""
import argparse
import base64
import sys
import time

try:
    import serial
except ImportError:
    # Refused in main(), not here: tools/blackbox_pull_test.py imports this
    # file to test the reply loop on CI, which is stdlib-only (same fix as
    # viewer.py, 68838f5). Exiting at import failed that job.
    serial = None

SECTOR = 4096
CHUNK = 768
OFFSETS = tuple(range(0, SECTOR, CHUNK))     # 0,768,...,3840
MAX_ROUNDS = 3
STALL_S = 5.0          # no dump line for this long = the reply is lost


def chunk_len(off):
    return min(CHUNK, SECTOR - off)          # 768 except 256 at 3840


def parse_line(line, chunks):
    """BB,<seq>,<off>,<b64> -> keep only if the payload decodes to the
    exact expected length. Returns True if stored."""
    if not line.startswith("BB,"):
        return False
    try:
        _, seq, off, b64 = line.split(",", 3)
        seq, off = int(seq), int(off)
        payload = base64.b64decode(b64, validate=True)
    except (ValueError, base64.binascii.Error):
        return False
    if off not in OFFSETS or len(payload) != chunk_len(off):
        return False
    chunks[(seq, off)] = b64
    return True


def read_until_eof(ser, chunks, progress=False, t0=0.0, stall_s=STALL_S,
                   clock=time.monotonic):
    """Consume lines into chunks until BBEOF (-> its count), serial timeout
    (-> None), or stall_s seconds without a single dump line (-> None).

    The stall rule exists because the serial timeout alone never fires:
    the S3 logs IBUS/GPS/TELEMETRY lines every second, so readline always
    returns SOMETHING. One BBEOF lost to the USB console's dropped bytes
    then spun this loop forever -, a range pull hung for 26
    minutes at sector ~10358 while the S3 was fine."""
    n_seen = 0
    last = clock()
    while True:
        raw = ser.readline()
        if not raw:
            return None
        line = raw.decode(errors="ignore").strip()
        if line.startswith("BBEOF"):
            try:
                return int(line.split(",", 1)[1])
            except (IndexError, ValueError):
                return -1
        if parse_line(line, chunks):
            last = clock()
            if progress:
                n_seen += 1
                if n_seen % 500 == 0:
                    print(f"  {n_seen} lines, {time.time()-t0:.0f} s")
        elif clock() - last > stall_s:
            return None


def missing_work(chunks):
    """-> (seqs with missing/bad chunks, whole-sector holes in the seq
    window)."""
    seqs = sorted({s for s, _ in chunks})
    if not seqs:
        return [], []
    have = set(seqs)
    incomplete = [s for s in seqs
                  if any((s, o) not in chunks for o in OFFSETS)]
    holes = [s for s in range(seqs[0], seqs[-1] + 1) if s not in have]
    return incomplete, holes


def open_port(port, no_reset):
    """pyserial asserts DTR/RTS on open, which resets the S3. Setting both
    low on an UNOPENED port first keeps it running."""
    from fc_common import open_port as _open
    return _open(port, 115200, 2, no_reset, serial_cls=serial.Serial)


def main():
    if serial is None:
        sys.exit("pyserial missing: pip install pyserial")
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port")
    ap.add_argument("out", nargs="?", default=time.strftime("%Y%m%d_%H%M.bb"))
    ap.add_argument("--no-reset", action="store_true",
                    help="open without resetting the S3")
    ap.add_argument("--from", dest="from_seq", type=int, default=None,
                    help="also fetch sectors from this seq up to the window")
    args = ap.parse_args()
    out_path = args.out

    ser = open_port(args.port, args.no_reset)
    time.sleep(0.3)

    def cmd(s):
        ser.write((s + "\n").encode())

    chunks = {}
    t0 = time.time()
    try:
        cmd("STREAM,0")
        time.sleep(0.3)
        ser.reset_input_buffer()
        # SD blackbox : opening the port reboots the S3, and the
        # card is probed in the background a moment after boot. A BBDUMP
        # sent before that answers BBEOF,0 and the pull reports "no data
        # captured" with a perfectly good card. Wait for the card first.
        state, deadline = "", time.time() + 15
        while time.time() < deadline:
            cmd("BBINFO")
            end = time.time() + 0.5
            while time.time() < end:
                raw = ser.readline()
                line = raw.decode(errors="ignore").strip()
                if line.startswith("blackbox:"):
                    state = line
            if any(s in state for s in ("READY", "FULL", "WRITE FAULT")):
                break
        print(f"  {state or 'no BBINFO reply'}")
        if not any(s in state for s in ("READY", "FULL", "WRITE FAULT")):
            print("card not ready - nothing to dump (NO CARD = wiring/5 V, "
                  "UNMARKED = run SDFORMAT)")
        ser.reset_input_buffer()
        cmd("BBDUMP")
        eof_count = read_until_eof(ser, chunks, progress=True, t0=t0)
        if eof_count is None:
            print("timeout waiting for data - is the FC on this port, "
                  "disarmed, with the P4 build? verifying what arrived...")

        # Older than the window: one sector per request. The retry rounds
        # below then cover the whole span, window and all.
        if args.from_seq is not None and chunks:
            window_lo = min(s for s, _ in chunks)
            older = list(range(args.from_seq, window_lo))
            print(f"fetching {len(older)} sector(s) older than the window "
                  f"({args.from_seq}..{window_lo - 1}), ~15/s")
            for i, s in enumerate(older):
                cmd(f"BBDUMP,{s}")
                read_until_eof(ser, chunks, stall_s=2.0)
                if i % 500 == 0:
                    print(f"  {i}/{len(older)}, {time.time()-t0:.0f} s")

        for round_no in range(1, MAX_ROUNDS + 1):
            incomplete, holes = missing_work(chunks)
            bad = sorted(set(incomplete) | set(holes))
            # A sector lost at the OLD end of the ring leaves no seq gap;
            # only the device's BBEOF count reveals it. Probe downward.
            if eof_count and eof_count > 0:
                have = {s for s, _ in chunks}
                short = eof_count - (len(have) + len(holes))
                probe = min(have) - 1 if have else 0
                while short > 0 and probe >= 1:
                    bad.append(probe)
                    probe -= 1
                    short -= 1
            if not bad:
                break
            print(f"retry round {round_no}: {len(bad)} sector(s) "
                  f"missing or damaged")
            for s in bad:
                cmd(f"BBDUMP,{s}")
                if read_until_eof(ser, chunks) is None:
                    print(f"  no reply for sector {s}")
    finally:
        cmd("STREAM,1")
        ser.close()

    if not chunks:
        sys.exit("no data captured")

    incomplete, holes = missing_work(chunks)
    n_sect = len({s for s, _ in chunks})
    with open(out_path, "w") as fh:
        for (s, o) in sorted(chunks):
            fh.write(f"BB,{s},{o},{chunks[(s, o)]}\n")
        fh.write(f"BBEOF,{n_sect}\n")

    if incomplete or holes:
        print(f"WARNING: still incomplete after {MAX_ROUNDS} rounds: "
              f"{len(incomplete)} damaged, {len(holes)} lost sector(s) - "
              f"the decoder will skip those regions")
    verdict = "verified COMPLETE" if not (incomplete or holes) else "INCOMPLETE"
    print(f"done: {n_sect} sectors, {verdict}, {time.time()-t0:.0f} s "
          f"-> {out_path}")
    print(f"next: python tools/blackbox_decode.py {out_path}")


if __name__ == "__main__":
    main()
