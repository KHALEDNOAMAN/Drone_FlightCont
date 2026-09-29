#!/usr/bin/env python3
"""Replay a decoded blackbox CSV through the vertical filter (P7 stage 1).

    python tools/vert_replay.py tools/fixtures/logA_s23.csv.gz
    python tools/vert_replay.py tools/fixtures/logA_s23.csv.gz --plot

Takes .csv or .csv.gz. The checked-in Log A fixture is gzipped because it
compresses to 11% (3.5 MB -> 388 KB) and a regression test whose input is
not in the repo is a test that rots.

Needs the four vertical columns that frame 43 added (048ef55): alt_m,
accz_ms2, tof_m, plus roll/pitch for the ToF tilt compensation. Older 34 B
and 35 B sessions decode with zeros in those columns and are refused here
rather than silently replayed against nothing.

ACCEPTANCE - what "the filter works" means on a STATIC log
----------------------------------------------------------
Log A is 103 s of an armed, strapped, motionless aircraft. That makes it a
STRONG test, not a weak one, because every correct answer is known in
advance:

  1. altitude must stay FLAT. The aircraft did not move (the ToF proves it
     to 1.4 mm), so any altitude ramp is the filter inventing motion.
  2. climb rate must stay at ZERO, same reason.
  3. the accel bias state must converge to +0.194 m/s^2, the value
     measured directly from the log's own accz mean.
  4. the baro bias must ABSORB the barometer's 18 cm drift. If the drift
     shows up in altitude instead, the fourth state is not doing its job.

A filter that drifts on this log is broken, full stop. Log B (the flight)
tests tracking and lag, which this cannot.
"""

import argparse
import csv
import gzip
import math
import statistics as st
import sys

sys.path.insert(0, __import__("os").path.dirname(__file__))
from vert_filter import (VertFilter, ACC_SIGMA_IDLE,   # noqa: E402
                         TOF_GATE_MAX_M)

TOF_NO_TARGET_M = 65.535        # the 65535 mm sentinel, in metres


def load(path):
    # .gz transparently, so a freshly decoded session and the checked-in
    # fixture are both just "the path you have".
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, mode="rt", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("empty CSV: %s" % path)
    need = ("t_us", "alt_m", "accz_ms2", "tof_m", "roll", "pitch",
            "sensor_health")
    missing = [c for c in need if c not in rows[0]]
    if missing:
        sys.exit("CSV is missing %s - decode a 43 B session (048ef55 or "
                 "later), not a legacy 34/35 B one" % ", ".join(missing))
    return rows


def replay(rows, acc_sigma, verbose=False, tof_cut=None, tof_step=None):
    """tof_cut: seconds after which the ToF is forced to NO_TARGET.
    tof_step: (seconds, metres) - adds a step to the ToF reading, i.e.
    the ground under the aircraft moved. Both are SYNTHETIC INJECTIONS
    used to test the two behaviours Log A cannot contain, because Log A
    never left the ground and the ToF never dropped out."""
    f = VertFilter(acc_sigma=acc_sigma)
    # A session logged by firmware without the field decodes as a column
    # of zeros, which is indistinguishable from "the aircraft never
    # accelerated" - so require a NON-ZERO value somewhere before
    # believing it, rather than just checking the column exists.
    have_dv = ("accz_dv_ms" in rows[0]
               and any(float(r["accz_dv_ms"]) != 0.0 for r in rows))
    prev_dv = None
    out = []

    # The log carries CACHED sensor values: the baro updates at 50 Hz and
    # the ToF at 1000/tof_budget_ms (~5 Hz at the 200 ms this airframe
    # boots with - the ~9 Hz seen elsewhere is the POLL rate, not the
    # delivery rate), both repeated across many 250 Hz frames. Feeding
    # the same measurement ~50 times in a row would make the filter
    # believe it has 50 independent observations and shrink its
    # covariance to nothing. Update only when the cached value CHANGES.
    #
    # The firmware does this properly with sample counters (vl53l0x_seq /
    # the slow-sample seqlock); change-detection is the offline stand-in
    # and is exact except for the rare case of two consecutive samples
    # landing on the same value.
    last_alt = None
    last_tof = None
    prev_t = None
    t_start = None
    baro_updates = tof_updates = tof_gated = 0

    for r in rows:
        t = int(r["t_us"]) * 1e-6
        if t_start is None:
            t_start = t
        if prev_t is None:
            dt = 0.004
        else:
            dt = t - prev_t
            if dt <= 0.0 or dt > 0.1:     # t_us wraps every 71 min
                dt = 0.004
        prev_t = t

        health = int(r["sensor_health"])
        accz = float(r["accz_ms2"])
        roll = float(r["roll"])
        pitch = float(r["pitch"])

        # SENSOR_IMU_OK. Without a fresh IMU read accz holds its last
        # value, so predicting on it would integrate a stale number.
        if health & 0x01:
            # Prefer the INTEGRAL the aircraft actually accumulated.
            #
            # accz_ms2 is one 250 Hz sample of a signal the filter
            # consumes at 416 Hz, and hover vibration is sd 3.67 m/s^2,
            # so replaying it integrates something the aircraft never
            # did. s139  is the proof: the filter ran to
            # -23.46 m in flight and the same C code on this same CSV
            # stayed inside 1.6 m. accz_dv_ms carries the running sum
            # instead, so the mean acceleration over this exact frame is
            # a subtraction - and being a mean over the interval, it
            # reproduces the filter's velocity update exactly rather
            # than approximately.
            #
            # Absent (0 for the whole session) on any log written before
            # the field existed, and those fall back to the old
            # behaviour so every fixture still replays.
            if have_dv:
                dv = float(r["accz_dv_ms"])
                if prev_dv is not None and dt > 0.0:
                    accz = (dv - prev_dv) / dt
                prev_dv = dv
            f.predict(accz, dt)

        alt = float(r["alt_m"])
        if (health & 0x02) and alt != last_alt:      # SENSOR_BARO_OK
            last_alt = alt
            f.update_baro(alt)
            baro_updates += 1

        tof = float(r["tof_m"])
        elapsed = t - t_start
        if tof_cut is not None and elapsed >= tof_cut:
            tof = TOF_NO_TARGET_M
        elif tof_step is not None and elapsed >= tof_step[0]:
            tof += tof_step[1]
        if tof != last_tof:
            last_tof = tof
            if tof <= 0.0 or tof >= TOF_NO_TARGET_M:
                f.tof_lost()
            else:
                before = f.tof_reject_streak
                if not f.update_tof(tof, roll, pitch):
                    if f.tof_reject_streak != before:
                        tof_gated += 1
                else:
                    tof_updates += 1

        z, vz, ba, bb = f.state()
        # Column 8 is the CUMULATIVE datum shift. z minus this is the
        # aircraft's true movement: it is what an alt-hold loop sees as
        # error, because the setpoint moves with the datum.
        out.append((t, z, vz, ba, bb, alt, tof, accz, f.datum_shift))

    if verbose:
        print("  baro updates %d, tof updates %d, tof gated %d"
              % (baro_updates, tof_updates, tof_gated))
    return out


def report(path, res, static):
    t0 = res[0][0]
    dur = res[-1][0] - t0
    z = [r[1] for r in res]
    vz = [r[2] for r in res]
    ba = [r[3] for r in res]
    bb = [r[4] for r in res]
    alt = [r[5] for r in res]
    accz = [r[7] for r in res]

    # Judge the settled part. The first 5 s is the filter finding the
    # accel bias from a cold start, which is convergence, not drift.
    settle_i = next((i for i, r in enumerate(res) if r[0] - t0 >= 5.0), 0)
    zs, vzs, bas = z[settle_i:], vz[settle_i:], ba[settle_i:]

    print("\n=== %s ===" % path)
    print("frames %d, %.1f s, %.1f Hz"
          % (len(res), dur, (len(res) - 1) / dur if dur else 0))
    print("\nfilter output (after 5 s settling):")
    print("  altitude   mean %+.4f m   sd %.4f   drift %+.4f m over %.0f s"
          % (st.mean(zs), st.pstdev(zs), zs[-1] - zs[0], res[-1][0] - res[settle_i][0]))
    print("  climb rate mean %+.4f m/s sd %.4f   max |vz| %.4f"
          % (st.mean(vzs), st.pstdev(vzs), max(abs(v) for v in vzs)))
    print("  accel bias final %+.4f m/s^2  (log accz mean %+.4f)"
          % (ba[-1], st.mean(accz)))
    print("  baro bias  first %+.4f m  final %+.4f m  moved %+.4f m"
          % (bb[0], bb[-1], bb[-1] - bb[0]))
    print("  raw baro   first %+.4f m  final %+.4f m  moved %+.4f m"
          % (alt[0], alt[-1], alt[-1] - alt[0]))

    if not static:
        print("\n(dynamic log - acceptance below is written for a static "
              "log and is NOT the right test here)")
        return True

    print("\nacceptance (static log):")
    ok = True

    def check(name, cond, detail):
        nonlocal ok
        print("  [%s] %-22s %s" % ("PASS" if cond else "FAIL", name, detail))
        if not cond:
            ok = False

    drift = abs(zs[-1] - zs[0])
    check("altitude flat", drift < 0.05,
          "drift %.4f m (limit 0.05)" % drift)
    check("altitude quiet", st.pstdev(zs) < 0.03,
          "sd %.4f m (limit 0.03)" % st.pstdev(zs))
    vz_rms = math.sqrt(sum(v * v for v in vzs) / len(vzs))
    check("climb rate zero", vz_rms < 0.05,
          "rms %.4f m/s (limit 0.05)" % vz_rms)
    target = st.mean(accz)
    check("accel bias converged", abs(ba[-1] - target) < 0.05,
          "%.4f vs %.4f m/s^2 (limit 0.05)" % (ba[-1], target))
    baro_moved = abs(alt[-1] - alt[0])
    absorbed = abs(bb[-1] - bb[0])
    check("baro drift absorbed", absorbed > 0.5 * baro_moved,
          "bias moved %.4f m of the baro's %.4f m" % (absorbed, baro_moved))
    return ok


def synthetic(path, rows, sigma, args):
    """The two behaviours the fourth state exists for, neither of which
    Log A contains. Both are injected into a REAL log so the noise,
    timing and sample rates stay real - only the ToF is doctored."""
    dur = (int(rows[-1]["t_us"]) - int(rows[0]["t_us"])) * 1e-6
    cut = dur * 0.5
    ok = True

    if args.handover:
        res = replay(rows, sigma, tof_cut=cut)
        # Altitude either side of the moment the ToF disappears. A
        # three-state filter jumps here by the whole baro offset; the
        # point of the baro-bias state is that this number is ~0.
        i = next(i for i, r in enumerate(res) if r[0] - res[0][0] >= cut)
        before = st.mean([r[1] for r in res[i - 250:i]])
        after = st.mean([r[1] for r in res[i + 25:i + 275]])
        step = after - before
        # After the handover the aircraft still has not moved, so the only
        # thing that may move altitude is the baro's own drift.
        tail = [r[1] for r in res[i:]]
        print("\n=== %s  [SYNTHETIC: ToF cut at %.0f s] ===" % (path, cut))
        print("  altitude step at handover  %+.4f m" % step)
        print("  altitude drift after       %+.4f m over %.0f s (baro-only, "
              "so it now inherits the baro's drift - expected)"
              % (tail[-1] - tail[0], dur - cut))
        good = abs(step) < 0.02
        print("  [%s] handover is seamless (limit 0.02 m)"
              % ("PASS" if good else "FAIL"))
        ok &= good

    if args.ground_step is not None:
        d = args.ground_step
        # DIFFERENTIAL test. Comparing raw altitude before and after would
        # mostly measure the barometer's own drift over the same window,
        # which has nothing to do with the injected step. Replay the same
        # log twice - once clean, once with the ground moved - and diff
        # them. Everything common cancels and what is left is exactly the
        # filter's response to the lie.
        base = replay(rows, sigma)
        res = replay(rows, sigma, tof_step=(cut, d))
        n = min(len(base), len(res))

        # What the CONTROLLER sees: altitude with datum changes removed.
        # A datum shift moves the setpoint too, so it is not error.
        err = [(res[k][1] - res[k][8]) - (base[k][1] - base[k][8])
               for k in range(n)]
        i = next(k for k, r in enumerate(res) if r[0] - res[0][0] >= cut)

        print("\n=== %s  [SYNTHETIC: ground moves %+.2f m at %.0f s] ==="
              % (path, d, cut))

        # A step that drives the reading outside TOF_MIN/MAX is not a
        # moved floor, it is a dropout - a different test (--handover).
        lo = min(r[6] for r in res[i:])
        hi = max(r[6] for r in res[i:] if r[6] < TOF_NO_TARGET_M)
        if lo < 0.03 or hi > 2.00:
            print("  NOTE: the step puts the ToF outside its %.2f-%.2f m "
                  "valid window (%.3f..%.3f), so this is a DROPOUT, not a "
                  "moved floor. Use --handover for that case." % (0.03, 2.0, lo, hi))

        shift = res[-1][8] - base[-1][8]
        worst = max(abs(e) for e in err[i:])
        settled = st.mean(err[-250:])
        print("  datum shift absorbed        %+.4f m of the %+.4f m step"
              % (shift, d))
        print("  worst commanded-height excursion %+.4f m" % worst)
        print("  settled commanded-height offset  %+.4f m" % settled)

        # Two regimes, split by TOF_GATE_MAX_M, and each has its own
        # correct answer. Judging both by one rule is what made the first
        # version of this test lie.
        if abs(d) <= TOF_GATE_MAX_M:
            # Inside the gate: the filter is SUPPOSED to follow it. That
            # is terrain following at small scale and it is the intended
            # meaning of a ground-referenced hold.
            good = abs(settled - d) < 0.03 and abs(shift) < 0.01
            print("  [%s] step is inside the %.0f cm gate, so the filter "
                  "follows it as terrain (expect settled ~= %+.3f, no "
                  "datum shift)"
                  % ("PASS" if good else "FAIL", TOF_GATE_MAX_M * 100, d))
        else:
            # Outside the gate: the step must land in the DATUM, never in
            # the command. Limits are set by the BAROMETER, not by
            # ambition - while the ToF is gated out the baro is the only
            # absolute sensor and it carries 73 mm of noise (Log A), so a
            # couple of sigma of commanded wander during the blind window
            # is physics. What must not happen is the aircraft ending up
            # somewhere else once it settles.
            absorbed = abs(shift - d) < 0.06
            good = absorbed and worst < 0.15 and abs(settled) < 0.05
            print("  [%s] step is outside the gate, so it must go into "
                  "the DATUM and not the command (limits: datum within "
                  "0.06 m of the step, 0.15 m peak, 0.05 m settled - the "
                  "last two set by the baro's 73 mm noise)"
                  % ("PASS" if good else "FAIL"))
        ok &= good

    return ok


def plot(res, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not installed - skipping the plot)")
        return
    t = [r[0] - res[0][0] for r in res]
    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    ax[0].plot(t, [r[5] for r in res], lw=0.6, label="baro (raw)")
    ax[0].plot(t, [r[6] if 0 < r[6] < 10 else float("nan") for r in res],
               lw=0.8, label="ToF")
    ax[0].plot(t, [r[1] for r in res], lw=1.2, label="filter altitude")
    ax[0].set_ylabel("m"); ax[0].legend(loc="upper right", fontsize=8)
    ax[0].set_title("P7 stage 1 vertical filter - %s" % path)
    ax[1].plot(t, [r[2] for r in res], lw=0.8)
    ax[1].set_ylabel("vz  m/s"); ax[1].axhline(0, color="k", lw=0.4)
    ax[2].plot(t, [r[3] for r in res], lw=0.8)
    ax[2].set_ylabel("accel bias\nm/s^2")
    ax[3].plot(t, [r[4] for r in res], lw=0.8)
    ax[3].set_ylabel("baro bias  m"); ax[3].set_xlabel("t  s")
    for a in ax:
        a.grid(alpha=0.3)
    out = path.rsplit(".", 1)[0] + "_vert.png"
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print("\nplot -> %s" % out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--acc-sigma", type=float, default=None,
                    help="process noise override, m/s^2 (default: the "
                         "hover placeholder from vert_filter.py)")
    ap.add_argument("--dynamic", action="store_true",
                    help="the log contains real motion - skip the "
                         "static-log acceptance checks")
    ap.add_argument("--handover", action="store_true",
                    help="synthetic test: kill the ToF halfway and check "
                         "the baro takes over without a step")
    ap.add_argument("--ground-step", type=float, metavar="M",
                    help="synthetic test: move the ground by M metres "
                         "halfway through and check the gate holds")
    args = ap.parse_args()

    all_ok = True
    for path in args.csv:
        rows = load(path)
        # A static log is best judged with the noise the log actually has.
        sigma = args.acc_sigma
        if sigma is None:
            sigma = ACC_SIGMA_IDLE if not args.dynamic else None

        if args.handover or args.ground_step is not None:
            all_ok &= synthetic(path, rows, sigma, args)
            continue

        res = replay(rows, sigma, verbose=True)
        if not report(path, res, static=not args.dynamic):
            all_ok = False
        if args.plot:
            plot(res, path)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
