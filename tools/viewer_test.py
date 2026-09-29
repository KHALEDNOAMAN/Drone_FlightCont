#!/usr/bin/env python3
"""Host test for tools/viewer.py - the D-line decode and the flow maths.

The viewer is where optical-flow counts become a velocity and a track, so
it is where a sign error or a missed window turns into a confident wrong
picture. Everything here is checked against numbers worked out by hand,
never against the viewer's own output.

Pure Python: no clang, no board, no serial port. Run:

    python tools/viewer_test.py
"""

import importlib.util
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.environ.get("VIEWER_SRC") or os.path.join(HERE, "viewer.py")

spec = importlib.util.spec_from_file_location("viewer_under_test", SRC)
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)

FAIL = 0


def check(name, got, want, tol=None):
    global FAIL
    ok = (abs(got - want) <= tol) if tol is not None else (got == want)
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got {got!r}  want {want!r}")


def dline(t_us, sumdx=0, sumdy=0, valid=0, lost=0, tof=1000, gyro=(0, 0, 0),
          yaw=0.0, squal=80, shutter=300, flags=3, seq=1, nfields=41):
    """One D line, built from the field list in main/debug_stream.c - not
    from anything in viewer.py."""
    f = ["D", str(t_us), "0", "0", "9.81",
         str(gyro[0]), str(gyro[1]), str(gyro[2]), "0", "0", "0",
         "1", "0", "0", "0", "0", "0", str(yaw), "0.0024", "0.004",
         "1000", "1000", "1000", "1000", "0", "1234", "0.50",
         "0", "0", "0", "0", "15"]
    if nfields >= 41:
        f += [str(tof), str(seq), str(sumdx), str(sumdy), str(valid),
              str(lost), str(squal), str(shutter), str(flags)]
    return ",".join(f)


def fresh(**over):
    cfg = {"scale": 1e-3, "manual_height_m": 0.0, "use_tof": True,
           "gyro_comp": True, "gyro_sign": 1.0, "swap_xy": False,
           "invert_x": False, "invert_y": False, "rotate_by_yaw": True,
           "yaw_offset_deg": 0.0, "invert_pitch": False}
    cfg.update(over)
    st = v.State(cfg)
    st.pending = []
    return st


def main():
    print("\nD line decode")
    st = fresh()
    v.decode(dline(1_000_000), st)
    check("health word", st.d["health"], 15)
    check("health names", st.d["health_names"],
          ["IMU", "BARO", "MAG", "MAG_GATED"])
    check("squal", st.d["flow"]["squal"], 80)
    check("tof", st.d["tof_mm"], 1000)
    check("flag names", st.d["flow"]["flag_names"], ["VALID", "MOTION"])

    print("\na firmware without the flow fields degrades, never crashes")
    st = fresh()
    v.decode(dline(1_000_000, nfields=32), st)
    check("the rest still parses", st.d["armed"], 0)
    check("flow absent", st.d["flow"], None)
    check("no integration", st.nav["integrating"], False)

    print("\nvelocity: 20 counts in 20 ms at 1.000 m, 1e-3 rad/count = 1 m/s")
    st = fresh()
    v.decode(dline(1_000_000, 0, 0, 0), st)
    v.decode(dline(1_020_000, 20, 0, 2), st)
    check("integrating", st.nav["integrating"], True)
    check("vx", st.nav["vx"], 1.0, 1e-9)
    check("x after one window", st.nav["x"], 0.02, 1e-9)
    check("height", st.nav["height_m"], 1.0, 1e-9)
    check("height source", st.nav["height_src"], "ToF")
    check("height scales velocity", fly(fresh(), tof=2000)["vx"], 2.0, 1e-9)
    check("scale scales velocity", fly(fresh(scale=2e-3))["vx"], 2.0, 1e-9)

    print("\ngyro compensation - rotation is not translation")
    one_rad = math.degrees(1.0)
    check("pure pitch rate cancels the flow it causes",
          fly(fresh(), gyro=(0, one_rad, 0))["vx"], 0.0, 1e-9)
    check("roll rate acts on the other axis",
          fly(fresh(), gyro=(one_rad, 0, 0), sumdy=20)["vy"], 0.0, 1e-9)
    check("compensation off leaves it in",
          fly(fresh(gyro_comp=False), gyro=(0, one_rad, 0))["vx"], 1.0, 1e-9)
    check("flipped sign doubles it instead of cancelling",
          fly(fresh(gyro_sign=-1.0), gyro=(0, one_rad, 0))["vx"], 2.0, 1e-9)

    print("\nmounting toggles")
    check("invert X", fly(fresh(invert_x=True))["vx"], -1.0, 1e-9)
    check("swap X/Y moves it to the other axis",
          fly(fresh(swap_xy=True))["vy"], 1.0, 1e-9)
    check("  and leaves x at zero", fly(fresh(swap_xy=True))["vx"], 0.0, 1e-9)

    print("\nyaw: a body-forward move at 90 deg of heading goes world +y")
    n = fly(fresh(), yaw=90.0)
    check("vx", n["vx"], 0.0, 1e-9)
    check("vy", n["vy"], 1.0, 1e-9)
    check("rotate_by_yaw off keeps it in the body frame",
          fly(fresh(rotate_by_yaw=False), yaw=90.0)["vx"], 1.0, 1e-9)

    print("\nrefuses rather than inventing (and says why)")
    n = fly(fresh(), valid=0)
    check("no valid window", n["integrating"], False)
    check("  why", "no valid flow window" in n["why"], True)
    check("  x untouched", n["x"], 0.0, 1e-12)
    n = fly(fresh(), tof=v.TOF_NO_TARGET)
    check("ToF has no target and no manual height", n["integrating"], False)
    check("  why", "no height" in n["why"], True)
    n = fly(fresh(manual_height_m=1.5, use_tof=False), tof=v.TOF_NO_TARGET)
    check("manual height takes over", n["height_src"], "manual")
    check("  and scales", n["vx"], 1.5, 1e-9)
    n = fly(fresh(), tof=50)
    check("50 mm is inside the 80 mm focus limit", n["integrating"], False)
    check("  why", "focus limit" in n["why"], True)
    n = fly(fresh(), dt_us=700_000)
    check("a 0.7 s window means the link stalled", n["integrating"], False)
    n = fly(fresh(), tof=0)
    check("ToF absent (0) is not a height", n["integrating"], False)

    print("\nattaching mid-session: the sums are already large")
    # The real case. The flow task has been running since boot, so the
    # first line the viewer ever sees carries a big sum. Only the CHANGE
    # between two lines is motion; the absolute value is not.
    st = fresh()
    v.decode(dline(1_000_000, 5000, -3000, 900), st)
    v.decode(dline(1_020_000, 5020, -3000, 902), st)
    check("vx from the change alone", st.nav["vx"], 1.0, 1e-9)
    check("  vy stays at zero", st.nav["vy"], 0.0, 1e-9)
    check("  one window of travel", st.nav["x"], 0.02, 1e-9)

    print("\ndropped D lines cost one window, not the answer")
    st = fresh()
    v.decode(dline(1_000_000, 0, 0, 0), st)
    v.decode(dline(1_100_000, 100, 0, 10), st)   # five lines missed
    check("x is exact across the gap", st.nav["x"], 0.1, 1e-9)

    print("\nsnapshot")
    st = fresh()
    v.decode(dline(1_000_000, 0, 0, 0), st)
    v.decode(dline(1_020_000, 20, 0, 2), st)
    snap = v.snapshot(st, 0)
    json.dumps(snap)
    check("serialises", True, True)
    check("track points", len(snap["track"]["pts"]) >= 2, True)
    check("generation starts at 1", snap["track"]["gen"], 1)
    check("fresh right after a line", snap["link"]["stale"], False)
    st.last_line_at = v.now() - (v.STALE_S + 0.2)
    check("stale once the lines stop", v.snapshot(st, 0)["link"]["stale"], True)
    nxt = snap["track"]["next"]
    check("incremental fetch adds nothing",
          len(v.snapshot(st, nxt)["track"]["pts"]), 0)
    st.reset_track()
    snap = v.snapshot(st, 0)
    check("reset bumps the generation", snap["track"]["gen"], 2)
    check("reset clears the position", snap["nav"]["x"], 0.0)

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


def fly(st, sumdx=20, sumdy=0, valid=2, dt_us=20_000, **kw):
    """Two lines one window apart; returns the nav state after."""
    v.decode(dline(1_000_000, 0, 0, 0, **kw), st)
    v.decode(dline(1_000_000 + dt_us, sumdx, sumdy, valid, **kw), st)
    return st.nav


if __name__ == "__main__":
    sys.exit(main())
