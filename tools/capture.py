#!/usr/bin/env python3
"""Capture the FC telemetry stream off the C6 WiFi bridge to a file.

Replaces `ncat <ip> 5555 > file.csv`, which works but tells you nothing
while it runs. During a mag tumble you are holding the airframe with both
hands and cannot watch a console, so this prints a once-a-second line
counting what actually arrived, per line type. If M, stays at 0 you know
within a second that MAGCAP is off, instead of after a two-minute tumble.

Usage:
    python capture.py 192.168.65.7 mag.csv
    python capture.py 192.168.65.7 sweep.csv --quiet

Ctrl-C to stop. The file is flushed continuously, so a Ctrl-C (or a
yanked battery) never loses more than the last fraction of a second.

Line types it counts:
    M,  mag capture      (MAGCAP,1)
    G,  gyro at loop rate (FFT_LOG=1 build)
    D,  50 Hz USB debug line, if it is being mirrored
    #   command replies from the FC
    dig telemetry CSV, 4 Hz (starts with the armed flag)
"""
import argparse
import socket
import sys
import time


# nav_state.h sensor_health bits, CSV field 52 (0-based). One shared copy.
from fc_common import health_names as decode_health  # noqa: E402


def classify(line):
    if not line:
        return None
    if line.startswith(b"M,"):
        return "M"
    if line.startswith(b"G,"):
        return "G"
    if line.startswith(b"D,"):
        return "D"
    if line.startswith(b"P,"):
        return "P"
    if line.startswith(b"#"):
        return "#"
    if line[0:1].isdigit():
        return "csv"
    return "other"


class SerialSource:
    """A COM port dressed up as a socket, so the capture loop below does
    not care which way the data arrived.

    Why bother: the WiFi bridge is lossy under the FFT_LOG stream. The
 sweeps lost 8-20 % of samples, not evenly but as ~700
    bursts of 3-9 consecutive samples, which leaves the longest uniform
    run at ~74 samples. That is too short for a windowed spectrum and too
    long to interpolate across (6 missing samples is more than a whole
    cycle at 84 Hz), so it costs the resonance hunt its answer. The UART
    is not the bottleneck - 921600 baud is 92 kB/s against the stream's
    15 kB/s - so the loss is in the C6's forwarding, and USB sidesteps it
    entirely.

    The trade is a tether, which is fine for exactly the case that needs
    the clean data: TASK 3 restrains the airframe anyway.
    """

    def __init__(self, port, baud=115200):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial missing: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=1.0)

    def recv(self, n):
        # An idle serial port returns b"" on timeout, but the capture loop
        # reads b"" as "the peer hung up" and stops. Raise the same
        # exception a quiet socket would, so a pause in the stream is a
        # pause and not the end of the run.
        data = self.ser.read(max(1, min(n, self.ser.in_waiting or 1)))
        if not data:
            raise socket.timeout
        return data

    def close(self):
        self.ser.close()


def looks_like_port(s):
    return s.upper().startswith("COM") or s.startswith("/dev/")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", help="C6 IP address, or a serial port "
                                 "(COM4, /dev/ttyUSB0) to read USB directly")
    ap.add_argument("outfile")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--baud", type=int, default=115200,
                    help="serial only; the USB console runs at 115200")
    ap.add_argument("--quiet", action="store_true",
                    help="no per-second progress line")
    a = ap.parse_args()

    if looks_like_port(a.host):
        print(f"opening {a.host} at {a.baud} ...", flush=True)
        try:
            sk = SerialSource(a.host, a.baud)
        except Exception as e:                       # noqa: BLE001
            print(f"FAILED: {e}")
            print("  - is anything else holding the port (a serial monitor,"
                  " idf.py monitor, another capture)?")
            sys.exit(1)
    else:
        print(f"connecting to {a.host}:{a.port} ...", flush=True)
        try:
            sk = socket.create_connection((a.host, a.port), timeout=10)
        except OSError as e:
            print(f"FAILED: {e}")
            print("  - is the C6 powered and on the WiFi?")
            print("  - only ONE client at a time: close any other capture "
                  "first")
            sys.exit(1)
        sk.settimeout(1.0)
    print(f"connected. writing to {a.outfile}   (Ctrl-C to stop)\n", flush=True)

    counts, totals = {}, {}
    health = set()
    pending = b""
    total_bytes = 0
    t_start = time.time()
    t_tick = t_start

    try:
        with open(a.outfile, "wb") as fh:
            while True:
                try:
                    chunk = sk.recv(8192)
                except socket.timeout:
                    chunk = b""
                else:
                    if not chunk:
                        print("\nconnection closed by the C6")
                        break
                    fh.write(chunk)
                    fh.flush()
                    total_bytes += len(chunk)

                    pending += chunk
                    *lines, pending = pending.split(b"\n")
                    for ln in lines:
                        ln = ln.strip()
                        k = classify(ln)
                        if k:
                            counts[k] = counts.get(k, 0) + 1
                            totals[k] = totals.get(k, 0) + 1
                        # Track what the FC says about its own sensors, so
                        # "no M, lines" can be told apart from "mag is dead"
                        # without guessing.
                        if k == "csv":
                            f = ln.split(b",")
                            if len(f) > 52:
                                try:
                                    health.add(int(f[52]))
                                except ValueError:
                                    pass

                now = time.time()
                if not a.quiet and now - t_tick >= 1.0:
                    rate = " ".join(f"{k}={v}" for k, v in sorted(counts.items()))
                    print(f"\r[{now - t_start:5.0f}s] {total_bytes/1024:7.0f} kB  "
                          f"{rate or 'NOTHING ARRIVING'}          ",
                          end="", flush=True)
                    counts = {}
                    t_tick = now
    except KeyboardInterrupt:
        pass
    finally:
        sk.close()

    dur = time.time() - t_start
    print(f"\n\nstopped after {dur:.0f} s, {total_bytes/1024:.0f} kB -> {a.outfile}")
    if totals:
        for k, v in sorted(totals.items()):
            print(f"  {k:>5} : {v:7d} lines  ({v/max(dur,1):.0f}/s)")
    else:
        print("  NOTHING was captured.")

    mag_alive = None
    if health:
        print()
        for v in sorted(health):
            ok, bad = decode_health(v)
            print(f"  sensor_health {v:2d} : ok={'+'.join(ok) or 'NONE'}"
                  f"{'   MISSING=' + '+'.join(bad) if bad else ''}")
        mag_alive = any(v & 4 for v in health)

    m = totals.get("M", 0)
    if m == 0:
        print("\nno M, lines.")
        if mag_alive is False:
            print("  The FC reports the MAG bit CLEAR - the magnetometer is not")
            print("  reading. Fix that first; MAGCAP cannot emit what it cannot")
            print("  read. Check I2C wiring and the boot log.")
        elif mag_alive:
            print("  The FC reports the mag ALIVE, so this is just the capture")
            print("  flag: send MAGCAP,1. It is off after every FC reboot or")
            print("  battery cycle - it does not persist.")
        else:
            print("  No telemetry CSV seen either, so sensor state is unknown.")
    elif m < 500:
        print(f"\nonly {m} mag samples - the fit wants 500+, tumble longer")
    else:
        print(f"\n{m} mag samples, enough. Next: python tools/mag_cal.py {a.outfile}")


if __name__ == "__main__":
    main()
