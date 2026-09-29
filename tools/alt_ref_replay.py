#!/usr/bin/env python3
"""Replay components/common/alt_ref.c against a real blackbox session and
score it against the vertical filter, with the ToF as ground truth.

    python tools/alt_ref_replay.py LATEST.csv

WHY THIS EXISTS AS A REPLAY AND NOT A LOG COLUMN
------------------------------------------------
alt_ref.z is published to nav_state.alt_ref_m, telemetry field 70 and a
dashboard row - but it is NOT in bb_frame_t. The blackbox therefore
cannot answer "did the reference beat the filter" directly. It does not
need to: alt_ref_update's entire input is (baro altitude, baro_ok, ToF
range, dt), and all four are in the log. Compiling the REAL alt_ref.c and
driving it with the logged sensors reproduces exactly what the board
displayed, rather than a Python paraphrase of it.

Two honest differences from the aircraft, neither of which moves a number
that matters:

  - The board runs alt_ref at 416 Hz inside the fusion task; the log is
    250 Hz. Every constant in alt_ref.c is PER SECOND (the 2 m/s slew,
    the 5 s disagreement clock), and dt comes from the log's own t_us, so
    the scheme is rate-independent by construction. The sensors it reads
    are slower than either rate anyway - baro 50 Hz, ToF ~5 Hz.
  - alt_m is int16 centimetres in the frame, so the baro input is
    quantised to 1 cm. Against a barometer whose 1-second spread is
    0.95-1.43 m that is not a rounding error worth naming.
  - The board latches baro0 at BOOT; the log starts at the ARM edge, so
    the replay latches it there instead. That makes the replay's brel
    differ from the board's by the boot-to-arm baro drift. It does not
    survive into z: the aircraft spends that whole window on the ground
    with the ToF live, where z IS the ToF and the 5 s clock re-references
    offset onto it, so both arrive at the arm edge with brel+offset == the
    ToF. The state that could differ is pinned by the same ground time
    that creates it. Logging alt_ref_m in bb_frame_t would retire this
    paragraph and is the right fix if the scheme earns authority.

WHAT IT SCORES
--------------
The ToF is the only ground truth this aircraft has ("THE ToF DOES
NOT LIE" - it returns a range under ~2 m or the NO_TARGET sentinel, never
a plausible wrong number). So the test is: over the frames where the ToF
was live, how far was each estimate from it? The filter has no excuse on
those frames - it was being fed the same ToF.
"""

import argparse
import csv
import ctypes
import gzip
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "components", "common")

SHIM = r"""
int _fltused = 0;
#include "alt_ref.c"
#define EXPORT __declspec(dllexport)
EXPORT void ar_init(alt_ref_t *a){ alt_ref_init(a); }
EXPORT void ar_update(alt_ref_t *a, float b, int ok, float tof, float dt){
    alt_ref_update(a, b, ok ? 1 : 0, tof, dt); }
EXPORT int ar_size(void){ return (int)sizeof(alt_ref_t); }
"""


class CAltRef(ctypes.Structure):
    _fields_ = [
        ("z", ctypes.c_float),
        ("baro0", ctypes.c_float),
        ("offset", ctypes.c_float),
        ("disagree_s", ctypes.c_float),
        ("have_baro0", ctypes.c_bool),
        ("have_z", ctypes.c_bool),
        ("tof_live", ctypes.c_bool),
    ]


def build(tmp):
    shim = os.path.join(tmp, "arshim.c")
    with open(shim, "w") as fh:
        fh.write(SHIM)
    dll = os.path.join(tmp, "ar.dll")
    cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry",
           "-O1", "-fno-math-errno", "-I", SRC, shim, "-o", dll]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        sys.exit("clang failed - see above.")
    return dll


def load(path):
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, mode="rt", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("empty CSV: %s" % path)
    need = ("t_us", "alt_m", "tof_m", "alt_est_m", "sensor_health")
    missing = [c for c in need if c not in rows[0]]
    if missing:
        sys.exit("CSV missing %s - decode a 48 B or later session"
                 % ", ".join(missing))
    return rows


TOF_NO_TARGET = 65.535   # the sentinel, /1000 by the decoder
SENSOR_BARO_OK = 0x02


def replay(rows, dll):
    lib = ctypes.CDLL(dll)
    lib.ar_size.restype = ctypes.c_int
    got, want = lib.ar_size(), ctypes.sizeof(CAltRef)
    if got != want:
        sys.exit("struct layout mismatch: C says %d bytes, ctypes says %d. "
                 "Fix CAltRef to mirror alt_ref.h." % (got, want))
    p = ctypes.POINTER(CAltRef)
    f32 = ctypes.c_float
    lib.ar_init.argtypes = [p]
    lib.ar_update.argtypes = [p, f32, ctypes.c_int, f32, f32]

    a = CAltRef()
    lib.ar_init(ctypes.byref(a))

    t0 = float(rows[0]["t_us"]) / 1e6
    prev_t = None
    out = []
    prev_live = None
    handovers = []          # (t, step the estimate had to make)
    rerefs = []
    prev_offset = 0.0

    for r in rows:
        t = float(r["t_us"]) / 1e6 - t0
        dt = 0.0 if prev_t is None else (t - prev_t)
        prev_t = t
        if not (0.0 < dt < 0.05):
            continue

        baro = float(r["alt_m"])
        health = int(r["sensor_health"])
        baro_ok = bool(health & SENSOR_BARO_OK)
        tof_raw = float(r["tof_m"])
        # 0 = absent/failing, 65.535 = alive but no target. Both mean
        # "no usable range", which alt_ref_update takes as <= 0.
        live = (tof_raw > 0.0) and (tof_raw < TOF_NO_TARGET)
        tof_in = tof_raw if live else -1.0

        z_before = a.z
        lib.ar_update(ctypes.byref(a), baro, 1 if baro_ok else 0, tof_in, dt)

        if prev_live is False and live:
            handovers.append((t, abs(tof_raw - z_before)))
        prev_live = live
        if a.offset != prev_offset:
            rerefs.append((t, prev_offset, a.offset))
            prev_offset = a.offset

        out.append({
            "t": t,
            "ref": a.z,
            "est": float(r["alt_est_m"]),
            "baro": baro,
            "tof": tof_raw if live else None,
            "thr": int(r["throttle"]),
            "vf": int(r["vert_flags"]),
            "disagree": a.disagree_s,
            "offset": a.offset,
        })
    return out, handovers, rerefs


def pct(vals, q):
    if not vals:
        return float("nan")
    s = sorted(vals)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


def report(name, out, handovers, rerefs, thr_min):
    fly = [r for r in out if r["thr"] > thr_min]
    if not fly:
        print("  %-22s no frames above throttle %d - skipped" % (name, thr_min))
        return None
    dur = fly[-1]["t"] - fly[0]["t"]
    tof = [r for r in fly if r["tof"] is not None]

    print("  %s" % name)
    print("    %.1f s above throttle %d, %d frames, ToF live on %d (%.0f%%)"
          % (dur, thr_min, len(fly), len(tof),
             100.0 * len(tof) / len(fly)))

    row = None
    if tof:
        er = [abs(r["ref"] - r["tof"]) for r in tof]
        ee = [abs(r["est"] - r["tof"]) for r in tof]
        print("    error vs ToF        median      p90       max")
        print("      alt_ref         %7.3f  %7.3f  %7.3f"
              % (pct(er, .5), pct(er, .9), max(er)))
        print("      alt_est (filter)%7.3f  %7.3f  %7.3f"
              % (pct(ee, .5), pct(ee, .9), max(ee)))
        row = (pct(er, .5), max(er), pct(ee, .5), max(ee))

    # Which loops actually had authority. Reading an altitude trace
    # without this is how a runaway with the hold OFF gets blamed on the
    # controller.
    bits = (("valid", 1 << 0), ("hold ENGAGED", 1 << 1),
            ("hold wanted", 1 << 2), ("ToF ok", 1 << 3),
            ("vel hold ENGAGED", 1 << 4), ("vel hold wanted", 1 << 5))
    print("    authority   " + "  ".join(
        "%s %.0f%%" % (n, 100.0 * sum(1 for r in fly if r["vf"] & m) / len(fly))
        for n, m in bits))

    est = [r["est"] for r in fly]
    ref = [r["ref"] for r in fly]
    print("    range               min      max     final")
    print("      alt_ref         %7.3f  %7.3f  %7.3f"
          % (min(ref), max(ref), ref[-1]))
    print("      alt_est         %7.3f  %7.3f  %7.3f"
          % (min(est), max(est), est[-1]))
    print("      raw baro        %7.3f  %7.3f  %7.3f"
          % (min(r["baro"] for r in fly), max(r["baro"] for r in fly),
             fly[-1]["baro"]))

    if handovers:
        steps = [s for _, s in handovers]
        print("    %d ToF handovers, worst pending correction %.3f m"
              % (len(handovers), max(steps)))
    if rerefs:
        print("    %d re-reference events:" % len(rerefs))
        for t, o0, o1 in rerefs[:6]:
            print("      t=%7.2f s  offset %+.3f -> %+.3f" % (t, o0, o1))
        if len(rerefs) > 6:
            print("      ... %d more" % (len(rerefs) - 6))
    else:
        print("    no re-reference events (the 5 s clock never expired)")
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--thr", type=int, default=1100,
                    help="throttle floor for in-flight frames "
                         "")
    args = ap.parse_args()

    import tempfile
    # ignore_cleanup_errors: ctypes.CDLL keeps the .dll mapped for the
    # life of the process and Windows refuses to unlink a mapped file.
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        dll = build(tmp)
        rows_by_file = [(p, load(p)) for p in args.csv]
        print("alt_ref.c replayed against %d session(s)\n" % len(rows_by_file))
        summary = []
        for path, rows in rows_by_file:
            out, ho, rr = replay(rows, dll)
            r = report(os.path.basename(path), out, ho, rr, args.thr)
            if r:
                summary.append((os.path.basename(path),) + r)
            print()

        if summary:
            print("VERDICT - median / max error against the ToF, in metres")
            print("  %-28s %8s %8s   %8s %8s"
                  % ("session", "ref med", "ref max", "est med", "est max"))
            for n, rm, rx, em, ex in summary:
                print("  %-28s %8.3f %8.3f   %8.3f %8.3f"
                      % (n, rm, rx, em, ex))


if __name__ == "__main__":
    main()
