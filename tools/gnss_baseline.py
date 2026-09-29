#!/usr/bin/env python3
"""Derive the guard's MEASURE thresholds from a real clean-sky log.

Three numbers in gg_default_cfg() are marked MEASURE because they are
properties of THIS antenna, THIS cable and THIS airframe's own switching
noise, not of the protocol: cno_sd_min_x10, agc_delta and noise_delta. A
guessed threshold either never fires or fires on a clean sky, and the
second one costs a flight.

This reads the driver's 1 Hz guard line, either live from the board or
from a saved capture, and prints the thresholds the observed sky actually
supports.

    python tools/gnss_baseline.py --port COM4 --minutes 10
    python tools/gnss_baseline.py --file bench/gnss_clean_.log

RUN IT ON A CLEAN SKY, OUTDOORS, WITH THE MOTORS BOTH OFF AND RUNNING.
The motors matter: the ESCs are the loudest RF source on the airframe, and
a baseline taken with them stopped will call a normal hover "interference"
the first time it flies. Capture both, and this prints the union.
"""

import argparse
import re
import statistics
import sys
import time

LINE = re.compile(
    r"guard pos=(?P<pos>\w+) vel=(?P<vel>\w+) reasons=0x(?P<reasons>[0-9A-Fa-f]+)"
    r".*?jam=(?P<jam>\d+) spf=(?P<spf>\d+) "
    r"agc=(?P<agc>\d+)/(?P<agcb>\d+) noise=(?P<noise>\d+)/(?P<noiseb>\d+) "
    r"cno=(?P<cno>[\d.]+) sd=(?P<sd>[\d.]+) n=(?P<n>\d+)")


def parse(lines):
    rows = []
    for ln in lines:
        m = LINE.search(ln)
        if m:
            d = m.groupdict()
            rows.append({
                "pos": d["pos"], "reasons": int(d["reasons"], 16),
                "jam": int(d["jam"]), "spf": int(d["spf"]),
                "agc": int(d["agc"]), "agc_base": int(d["agcb"]),
                "noise": int(d["noise"]), "noise_base": int(d["noiseb"]),
                "cno": float(d["cno"]), "sd": float(d["sd"]),
                "n": int(d["n"]),
            })
    return rows


def report(rows):
    if len(rows) < 30:
        print(f"only {len(rows)} guard lines - need at least 30 (30 s) to say "
              f"anything. Is the GPS configured and outdoors?")
        return 1

    # Only samples the guard itself considered clean describe a clean sky.
    clean = [r for r in rows if r["reasons"] == 0 and r["n"] > 0]
    print(f"\n{len(rows)} samples, {len(clean)} with no standing reason")
    if len(clean) < 20:
        print("\nTOO FEW CLEAN SAMPLES. Either the sky is genuinely not clean,")
        print("or a threshold is already too tight - check which reasons are")
        print("standing with GSEC before trusting anything below.")
        seen = {}
        for r in rows:
            seen[r["reasons"]] = seen.get(r["reasons"], 0) + 1
        for k, v in sorted(seen.items(), key=lambda kv: -kv[1])[:5]:
            print(f"   reasons=0x{k:03X}  {v} samples")
        return 1

    sd = [r["sd"] for r in clean]
    cno = [r["cno"] for r in clean]
    n = [r["n"] for r in clean]
    agc = [r["agc"] for r in clean]
    noise = [r["noise"] for r in clean]
    agc_base = statistics.median(r["agc_base"] for r in clean) or 0
    noise_base = statistics.median(r["noise_base"] for r in clean) or 0

    def stat(name, v, unit=""):
        print(f"  {name:<14} min {min(v):8.1f}  median {statistics.median(v):8.1f}"
              f"  max {max(v):8.1f}{unit}")

    print("\nclean-sky observations")
    stat("C/N0 sd", sd, " dB-Hz")
    stat("C/N0 mean", cno, " dB-Hz")
    stat("used sigs", [float(x) for x in n])
    stat("AGC", [float(x) for x in agc])
    stat("noisePerMS", [float(x) for x in noise])
    print(f"  learned baselines: agc {agc_base:.0f}  noise {noise_base:.0f}")

    # cno_sd_min_x10: sit clearly UNDER the tightest spread a clean sky
    # produced, or the guard calls a good constellation a spoofer.
    sd_min = min(sd)
    rec_sd = int(round(sd_min * 10 * 0.70))

    # agc_delta: the deepest DROP below baseline seen while clean, with
    # margin. Only drops matter - a rise is a quieter band.
    agc_drop = max(0.0, agc_base - min(agc))
    rec_agc = int(round(max(agc_drop * 1.8, 150)))

    # noise_delta: the largest RISE above baseline seen while clean.
    noise_rise = max(0.0, max(noise) - noise_base)
    rec_noise = int(round(max(noise_rise * 1.8, 100)))

    rec_minused = max(5, int(min(n) * 0.8))

    print("\nrecommended, for components/gnss_guard/gnss_guard.c "
          "gg_default_cfg():")
    print(f"    cfg->cno_sd_min_x10  = {rec_sd};"
          f"    /* {rec_sd/10:.1f} dB-Hz; tightest clean spread was {sd_min:.1f} */")
    print(f"    cfg->cno_min_used    = {rec_minused};"
          f"      /* fewest used signals seen was {min(n)} */")
    print(f"    cfg->agc_delta       = {rec_agc};"
          f"    /* deepest clean drop below baseline was {agc_drop:.0f} */")
    print(f"    cfg->noise_delta     = {rec_noise};"
          f"    /* largest clean rise above baseline was {noise_rise:.0f} */")

    margin = sd_min * 10 - rec_sd
    if margin < 8:
        print("\nWARNING: the clean sky's tightest C/N0 spread is close to the")
        print("recommended floor. This antenna may not separate a spoofer from")
        print("a good fix on spread alone - lean on the inertial cross-check")
        print("and the module's own verdict, and do NOT let cno-uniform reach")
        print("DENIED by itself (it cannot today, by design).")

    print("\nAfter editing, re-run:  python tools/gnss_guard_test.py"
          "  and  python tools/gnss_scenario.py")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. COM4")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--file", help="read a saved capture instead")
    ap.add_argument("--save", help="also write the raw lines here")
    a = ap.parse_args()

    if a.file:
        with open(a.file, errors="replace") as fh:
            return report(parse(fh))

    if not a.port:
        ap.error("give --port or --file")

    try:
        import serial
    except ImportError:
        print("pyserial not installed:  pip install pyserial")
        return 2

    lines, out = [], open(a.save, "w") if a.save else None
    deadline = time.time() + a.minutes * 60
    print(f"capturing {a.minutes:.0f} min from {a.port} - Ctrl-C to stop early")
    try:
        with serial.Serial(a.port, a.baud, timeout=1) as s:
            while time.time() < deadline:
                ln = s.readline().decode(errors="replace").rstrip()
                if not ln:
                    continue
                if "guard pos=" in ln:
                    lines.append(ln)
                    if out:
                        out.write(ln + "\n")
                    left = int(deadline - time.time())
                    print(f"\r  {len(lines)} samples, {left}s left ", end="")
    except KeyboardInterrupt:
        print("\n  stopped early")
    finally:
        if out:
            out.close()
    print()
    return report(parse(lines))


if __name__ == "__main__":
    sys.exit(main())
