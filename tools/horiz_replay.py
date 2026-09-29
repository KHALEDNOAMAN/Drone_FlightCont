#!/usr/bin/env python3
"""Replay a decoded blackbox CSV through the 6-state horizontal filter.

    python tools/horiz_replay.py s161.csv

WHAT "IT WORKS" MEANS ON A WALK LOG
-----------------------------------
A walk is a strong test for the same reason Log A is a strong test for
the vertical filter: large parts of the truth are known in advance.

  1. While PARKED the aircraft did not move, so every metre of scatter in
     the estimate is error, and the filter must be quieter than the raw
     GPS it was fed. If it is not, it is adding nothing.
  2. While WALKING it must still TRACK. A filter can always win test 1 by
     ignoring its measurements; the walking legs are what stops that
     being a passing grade.
  3. Velocity innovations must mostly be ACCEPTED. This is the sharp one,
     and it is really a test of the PREDICTION, not the correction - see
     below.

THE ACCELEROMETER, AND WHY THIS TOOL CHECKS FOR IT FIRST
--------------------------------------------------------
The filter's predict() integrates earth-frame horizontal acceleration.
Logs written earlier have no such column - the blackbox carried
GPS but no horizontal accel - so this tool falls back to driving predict()
with ZERO acceleration, i.e. a constant-velocity model.

That fallback is honest but it is NOT the filter. With no acceleration
input the estimate cannot follow a start or a stop, so every one of them
produces a large velocity innovation and a chunk of them get gated out.
MEASURED on s161, which has no accel columns: 227 of 1426 velocity fixes
rejected, 16%, and a walking leg tracked 8.6 m against the GPS's 11.8 m.

So a high velocity-reject rate on an accel-less log is the EXPECTED
result and says nothing bad about the filter. On a log that HAS the
columns it is a real measurement, and the number to watch: if it is still
near 16% with real acceleration going in, something is wrong with the
accel path - a sign, an axis, or the frame convention.
"""

import argparse
import csv
import gzip
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from horiz_filter import HorizFilter                     # noqa: E402

GPS_F_FIX_OK = 0x08
GPS_F_HOME_VALID = 0x10


def load(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", newline="") as fh:
        return list(csv.DictReader(fh))


def still_blocks(rows, t0, min_s=20.0, thresh=0.45):
    """Find the stretches where the aircraft was not moving, from GPS
    speed. These are where the truth is known: zero."""
    out, start, prev = [], None, None
    for r in rows:
        t = float(r["t_us"]) / 1e6 - t0
        sp = math.hypot(float(r["vel_n_ms"]), float(r["vel_e_ms"]))
        if sp <= thresh:
            if start is None:
                start = t
        else:
            if start is not None and prev - start >= min_s:
                out.append((start, prev))
            start = None
        prev = t
    if start is not None and prev - start >= min_s:
        out.append((start, prev))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    args = ap.parse_args()

    rows = load(args.log)
    t0 = float(rows[0]["t_us"]) / 1e6
    dur = float(rows[-1]["t_us"]) / 1e6 - t0

    have_acc = "accn_dv_ms" in rows[0]
    if have_acc:
        # Present but all-zero means firmware that predates the field,
        # decoded by a decoder that knows about it - which looks exactly
        # like a stationary aircraft unless it is checked.
        nz = any(float(r["accn_dv_ms"]) or float(r["acce_dv_ms"])
                 for r in rows)
        if not nz:
            have_acc = False
            print("!! accn_dv_ms/acce_dv_ms present but ZERO for the whole\n"
                  "   session - firmware older than. Falling back\n"
                  "   to a constant-velocity prediction.\n")
    else:
        print("!! no accn_dv_ms/acce_dv_ms columns - log predates.\n"
              "   Falling back to a constant-velocity prediction; expect a\n"
              "   high velocity-reject rate. See the header of this file.\n")

    f = HorizFilter()
    tp = None
    last = None
    prev_dn = prev_de = None
    est, raw = [], []
    n_fix = 0

    for r in rows:
        t = float(r["t_us"]) / 1e6 - t0
        if tp is not None:
            dt = t - tp
            if 0.0 < dt < 0.05:
                an = ae = 0.0
                if have_acc:
                    dn = float(r["accn_dv_ms"])
                    de = float(r["acce_dv_ms"])
                    if prev_dn is not None:
                        # The columns are ABSOLUTE running sums of a*dt,
                        # so differencing two rows and dividing by the
                        # interval recovers the MEAN acceleration over
                        # it - which is what the filter consumed, not an
                        # approximation of it.
                        an = (dn - prev_dn) / dt
                        ae = (de - prev_de) / dt
                    prev_dn, prev_de = dn, de
                f.predict(an, ae, dt)
        tp = t

        fl = int(r["gps_flags"])
        key = (r["pos_n_m"], r["pos_e_m"], r["vel_n_ms"], r["vel_e_ms"])
        if key != last and (fl & GPS_F_FIX_OK) and (fl & GPS_F_HOME_VALID):
            last = key
            n_fix += 1
            pn, pe = float(r["pos_n_m"]), float(r["pos_e_m"])
            # Velocity FIRST, deliberately. It is the better measurement
            # (MEASURED 0.1 m/s against 1-3 m), so letting it correct the
            # state before position is judged means position is gated
            # against a filter that has already had the good news.
            f.update_gps_vel(float(r["vel_n_ms"]), float(r["vel_e_ms"]))
            f.update_gps_pos(pn, pe, float(r["h_acc_m"]))
            raw.append((t, pn, pe))
        est.append((t, f.pn, f.pe, f.vn, f.ve))

    print("=== %s ===" % os.path.basename(args.log))
    print("frames %d, %.1f s, %d GPS fixes (%.2f Hz), accel %s"
          % (len(rows), dur, n_fix, n_fix / dur,
             "YES" if have_acc else "NO (constant-velocity fallback)"))
    pr = 100.0 * f.n_pos_reject / max(1, f.n_pos_accept + f.n_pos_reject)
    vr = 100.0 * f.n_vel_reject / max(1, f.n_vel_accept + f.n_vel_reject)
    print("  position  %d accepted / %d rejected  (%.1f%% gated)"
          % (f.n_pos_accept, f.n_pos_reject, pr))
    print("  velocity  %d accepted / %d rejected  (%.1f%% gated)"
          % (f.n_vel_accept, f.n_vel_reject, vr))
    if have_acc:
        print("  -> with real acceleration, a velocity reject rate near the "
              "16%% measured\n     WITHOUT it would mean the accel path is "
              "wrong (sign, axis or frame).")

    fail = 0
    print("\nacceptance:")
    blocks = still_blocks(rows, t0)
    if not blocks:
        print("  [SKIP] no stationary block of 20 s+ - cannot measure error")
    for i, (a, b) in enumerate(blocks, 1):
        e = [x for x in est if a <= x[0] <= b]
        rr = [x for x in raw if a <= x[0] <= b]
        if len(e) < 50 or len(rr) < 20:
            continue
        mn, me = st.mean(x[1] for x in e), st.mean(x[2] for x in e)
        fsd = (st.pstdev([x[1] - mn for x in e]) +
               st.pstdev([x[2] - me for x in e]))
        rn, re_ = st.mean(x[1] for x in rr), st.mean(x[2] for x in rr)
        rsd = (st.pstdev([x[1] - rn for x in rr]) +
               st.pstdev([x[2] - re_ for x in rr]))
        good = fsd < rsd
        fail += 0 if good else 1
        print("  [%s] parked block %d (%.0f-%.0f s): raw sd %.2f m -> "
              "filtered %.2f m  (%.1fx)"
              % ("PASS" if good else "FAIL", i, a, b, rsd / 2, fsd / 2,
                 rsd / max(1e-9, fsd)))

    # Tracking: the filter must not win the test above by ignoring the GPS.
    #
    # NOT measured as net displacement over a leg, which was the first
    # thing tried and is wrong: an out-and-back walk has a net
    # displacement near zero, so that number compares two small noisy
    # quantities and fails a perfectly good filter. It reported 39% on
    # s161 for exactly that reason.
    #
    # What tracking actually means is that the estimate STAYS WITH the
    # measurement while moving. So: the distance between the filtered
    # position and the raw fix, at each fix, during motion. A filter that
    # ignores its GPS to win the parked test above walks away from it and
    # this number grows without bound.
    moving = []
    for k in range(1, len(blocks)):
        moving.append((blocks[k - 1][1], blocks[k][0]))
    if moving:
        errs = []
        for a, b in moving:
            for (t, pn, pe) in raw:
                if not (a <= t <= b):
                    continue
                # the estimate at that same instant
                near = min((x for x in est if abs(x[0] - t) < 0.01),
                           key=lambda x: abs(x[0] - t), default=None)
                if near:
                    errs.append(math.hypot(near[1] - pn, near[2] - pe))
        if errs:
            errs.sort()
            rms = math.sqrt(sum(e * e for e in errs) / len(errs))
            p95 = errs[int(len(errs) * 0.95)]
            # 5 m: comfortably inside the raw GPS's own measured scatter
            # (per-axis sd up to 3.2 m, radial p95 9.9 m), so a filter
            # that is genuinely following its measurement clears it, and
            # one that has stopped listening does not.
            good = rms < 5.0
            fail += 0 if good else 1
            print("  [%s] still TRACKS while moving: filtered position is "
                  "%.2f m rms from the raw fix (p95 %.2f, want rms < 5.0)"
                  % ("PASS" if good else "FAIL", rms, p95))

    print("\n%s" % ("ALL PASS" if not fail else "%d FAILURE(S)" % fail))
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
