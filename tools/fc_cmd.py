#!/usr/bin/env python3
"""Send console commands to the S3 over USB and print what it answers.

    python tools/fc_cmd.py COM8 BBINFO
    python tools/fc_cmd.py COM8 GET,m1_scale "MAGLOG,90,BOOT"

Opens the port WITHOUT resetting the S3 (DTR/RTS low), turns the 50 Hz
D-stream off so the replies are readable, sends each command, and prints
every non-D line that comes back within --wait seconds of it.
"""
import argparse
import time

from fc_common import open_port


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port")
    ap.add_argument("cmds", nargs="+")
    ap.add_argument("--wait", type=float, default=1.5)
    a = ap.parse_args()

    s = open_port(a.port, timeout=0.1)
    try:
        s.write(b"STREAM,0\n")
        time.sleep(0.3)
        s.reset_input_buffer()
        for cmd in a.cmds:
            print(f"> {cmd}")
            s.write(cmd.encode() + b"\n")
            end = time.time() + a.wait
            while time.time() < end:
                line = s.readline().decode(errors="ignore").rstrip()
                if line and not line.startswith("D,"):
                    print(f"  {line}")
    finally:
        s.close()


if __name__ == "__main__":
    main()
