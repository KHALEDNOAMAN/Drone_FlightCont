#!/usr/bin/env python3
"""The altitude SETPOINT must never be driven below the ground.

    python tools/setpoint_safety_test.py

WHY THIS IS A REAL-LOG TEST AND NOT A SYNTHETIC ONE
---------------------------------------------------
tools/vert_gating_test.py covers the gating branches with synthetic
scenarios, and that is the right tool for most of them. It cannot reach
THIS one, and the reason is worth writing down rather than pretending
otherwise.

A re-datum only consults the terrain-vs-error discriminator
(VF_TOF_TERRAIN_HOLD_M) when the estimate has walked between 0.25 m and
1.0 m during the disagreement window. Below 0.25 m it is credited to the
ground; above 1.0 m the runaway detector fires first, clears `valid`, and
the immediate-recovery path handles it instead. That leaves a narrow
band, and the dynamics cross it fast: measured on synthetic drifts, any
accelerometer bias large enough to get the ToF rejected at all drives the
estimate straight past 1.0 m inside one window. Every synthetic profile
tried either never rejected a sample or blew through the band.

Flight s170 landed in it. Real accelerometer bias, real ToF, real
timing - and the consequence was a re-datum that pushed -1.04 m into the
altitude setpoint, taking it to -0.74 m, BELOW THE GROUND. The hold then
saw itself 2.2 m too high and cut 236 us of throttle. the author felt the drop
and disarmed.

So the regression is the flight itself. Three logs, replayed through the
REAL C, asserting the property that matters: datum_shift is what lands on
the controller's altitude setpoint, and it must never carry the estimate's
own error into it.

MEASURED, with and without the discriminator:

    flight   with        without
    s169     -0.16 m     -0.74 m
    s170      0.00 m     -0.68 m
    s171      0.00 m     -0.66 m
"""

import csv
import ctypes
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from host_c import HostC, CVertFilter                     # noqa: E402

# The three flights flown with hold_auto=1. s170 is the one
# that produced the incident; the other two are the same firmware and the
# same conditions, so a fix that only helps s170 is a fix aimed at one
# dataset.
FLIGHTS = ("s169.csv",
           "s170.csv",
           "s171.csv")

# How far below the ground datum the setpoint may be pushed. The ground
# is 0 by construction - it is where the ToF said the floor was - so any
# meaningful negative number is the filter telling the controller to fly
# into it. 0.30 m is slack for genuine downward terrain (a step down off
# a kerb is real and must still be followed); s170 shipped -0.74.
SETPOINT_FLOOR_M = -0.30


def replay(lib, path):
    rows = list(csv.DictReader(open(path, newline="")))
    f = CVertFilter()
    F = ctypes.c_float
    lib.vf_init(ctypes.byref(f), F(2.0), F(0.0))

    prev_t = last_alt = last_tof = prev_dv = None
    lo = hi = 0.0
    worst_gap = 0.0
    shift = 0.0
    for r in rows:
        t = float(r["t_us"]) / 1e6
        if prev_t is not None:
            dt = t - prev_t
            if 0.0 < dt < 0.05:
                accz = float(r["accz_ms2"])
                dv = float(r["accz_dv_ms"])
                if prev_dv is not None:
                    accz = (dv - prev_dv) / dt
                prev_dv = dv
                lib.vf_predict(ctypes.byref(f), F(accz), F(dt))
        prev_t = t

        alt = float(r["alt_m"])
        if (int(r["sensor_health"]) & 0x02) and alt != last_alt:
            last_alt = alt
            lib.vf_baro(ctypes.byref(f), F(alt))

        tof = float(r["tof_m"])
        if tof != last_tof:
            last_tof = tof
            if tof <= 0.0 or tof >= 65.535:
                lib.vf_toflost(ctypes.byref(f))
            else:
                import math
                tc = (math.cos(math.radians(float(r["roll"]))) *
                      math.cos(math.radians(float(r["pitch"]))))
                lib.vf_tof(ctypes.byref(f), F(tof), F(tc))
        # take_datum_shift is read-and-clear, exactly as fc_tasks does it,
        # so this accumulates the same total the controller would see.
        shift += lib.vf_takeshift(ctypes.byref(f))
        lo = min(lo, shift)
        hi = max(hi, shift)
        if 0.0 < tof < 65.0:
            worst_gap = max(worst_gap, abs(f.z - tof))
    return lo, hi, worst_gap


def main():
    fail = 0
    print("Replaying the three hold_auto flights through the REAL C.\n")
    print("  %-7s %11s %11s %12s"
          % ("flight", "setpoint lo", "setpoint hi", "|z - tof| max"))
    with HostC() as hc:
        for name in FLIGHTS:
            path = os.path.join(ROOT, name)
            if not os.path.exists(path):
                print("  %-7s  SKIPPED - log not on disk" % name[-9:-4])
                continue
            lo, hi, gap = replay(hc.lib, path)
            ok = lo >= SETPOINT_FLOOR_M
            fail += 0 if ok else 1
            print("  %-7s %+11.2f %+11.2f %12.2f   [%s]"
                  % (name[-9:-4], lo, hi, gap, "PASS" if ok else "FAIL"))

    print("\n  setpoint floor %.2f m; s170 shipped -0.74 m and cut 236 us "
          "of throttle." % SETPOINT_FLOOR_M)
    print("\n%s" % ("ALL PASS" if not fail else "%d FAILURE(S)" % fail))
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
