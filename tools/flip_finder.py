#!/usr/bin/env python3
"""Which session flipped, and what failed FIRST?

    python tools/flip_finder.py *_s*.csv

Scans decoded sessions for a loss-of-control event and, where it finds
one, answers the only question that separates the two candidate causes:

  SENSORS DIED FIRST  -> the estimate went garbage and the controller
                         flew it into the ground. Rail / wiring / a
                         chip. Look at sensor_health, not at gains.
  ATTITUDE WENT FIRST -> the sensors were fine and the aircraft was
                         genuinely commanded or trimmed into a flip.
                         Look at m*_scale, level_*, and the mixer.

Getting that ordering backwards is how a hardware fault gets debugged as
a control problem for a week, which is a thing that has already happened
on this aircraft more than once.

A "flip" here is |roll| or |pitch| past FLIP_DEG while the throttle is up
- not a tipped-over airframe sitting on the bench afterwards, which
the project's is explicit produces numbers indistinguishable from a real
excursion.
"""

import csv
import glob
import os
import sys

FLIP_DEG = 60.0          # past this it is not a manoeuvre
THR_FLY = 1100           # below this it is ground/idle, not flight

from fc_common import HEALTH_BITS  # noqa: E402  (nav_state.h SENSOR_*)


def bits(v):
    return "+".join(n for b, n in HEALTH_BITS if v & b) or "NONE"


def scan(path):
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows or "roll" not in rows[0]:
        return None
    t0 = float(rows[0]["t_us"]) / 1e6

    ev = []
    for r in rows:
        t = float(r["t_us"]) / 1e6 - t0
        thr = int(r["throttle"])
        att = max(abs(float(r["roll"])), abs(float(r["pitch"])))
        ev.append((t, thr, att, int(r["sensor_health"])))

    dur = ev[-1][0]
    fly = [e for e in ev if e[1] > THR_FLY]
    if not fly:
        return dict(path=path, dur=dur, n=len(ev), flew=False)

    # First moment the attitude leaves control, while flying.
    t_flip = next((e[0] for e in fly if e[2] > FLIP_DEG), None)

    # First moment the IMU bit drops after the aircraft is up.
    t0_fly = fly[0][0]
    t_imu = next((e[0] for e in ev
                  if e[0] >= t0_fly and not (e[3] & 1)), None)
    # First moment ANY health bit that was set at takeoff goes away.
    h0 = fly[0][3]
    t_health = next((e[0] for e in ev
                     if e[0] >= t0_fly and (e[3] & h0) != h0), None)

    return dict(path=path, dur=dur, n=len(ev), flew=True,
                t_fly=t0_fly, max_att=max(e[2] for e in fly),
                max_thr=max(e[1] for e in fly),
                h_takeoff=h0, h_end=ev[-1][3],
                t_flip=t_flip, t_imu=t_imu, t_health=t_health)


def main():
    pats = sys.argv[1:] or ["*_s*.csv"]
    paths = []
    for p in pats:
        paths.extend(sorted(glob.glob(p)))
    if not paths:
        sys.exit("no CSVs matched")

    flips = []
    print("%-34s %6s %7s %8s %-18s" %
          ("session", "dur s", "maxAtt", "maxThr", "health takeoff->end"))
    print("-" * 84)
    for p in paths:
        s = scan(p)
        if s is None:
            continue
        if not s["flew"]:
            print("%-34s %6.1f   (never above throttle %d)"
                  % (os.path.basename(p), s["dur"], THR_FLY))
            continue
        mark = "  <-- FLIP" if s["t_flip"] is not None else ""
        print("%-34s %6.1f %7.1f %8d %-18s%s"
              % (os.path.basename(p), s["dur"], s["max_att"], s["max_thr"],
                 "%s -> %s" % (bits(s["h_takeoff"]), bits(s["h_end"])),
                 mark))
        if s["t_flip"] is not None:
            flips.append(s)

    if not flips:
        print("\nNo session exceeded %.0f deg while flying." % FLIP_DEG)
        return 0

    print("\n" + "=" * 84)
    print("ORDERING - what failed first")
    print("=" * 84)
    for s in flips:
        print("\n%s" % os.path.basename(s["path"]))
        print("  took off        t = %6.2f s" % s["t_fly"])
        print("  attitude > %.0f  t = %6.2f s  (peak %.1f deg)"
              % (FLIP_DEG, s["t_flip"], s["max_att"]))
        if s["t_imu"] is None and s["t_health"] is None:
            print("  sensor health   never dropped - every bit that was set "
                  "at takeoff was still set at the end")
            print("  => VERDICT: the sensors were FINE. The aircraft was "
                  "flown or trimmed into this.")
            continue
        if s["t_imu"] is not None:
            print("  IMU bit lost    t = %6.2f s" % s["t_imu"])
        if s["t_health"] is not None:
            print("  any bit lost    t = %6.2f s" % s["t_health"])
        first = min(x for x in (s["t_imu"], s["t_health"]) if x is not None)
        lead = s["t_flip"] - first
        if lead > 0.05:
            print("  => VERDICT: SENSORS DIED %.2f s BEFORE the attitude "
                  "went. Hardware first, control second." % lead)
        elif lead < -0.05:
            print("  => VERDICT: the attitude went %.2f s BEFORE any sensor "
                  "dropped. The sensors were fine when it started."
                  % (-lead))
        else:
            print("  => VERDICT: simultaneous within one frame - cannot "
                  "order them from this log.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
