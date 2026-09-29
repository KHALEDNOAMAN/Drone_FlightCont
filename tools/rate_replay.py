#!/usr/bin/env python3
"""Replay a flight's rate loop through the REAL firmware PID (pidcore).

    python tools/rate_replay.py verify s121.csv
    python tools/rate_replay.py verify 's*.csv'

WHY VERIFY COMES FIRST
----------------------
A tuner that cannot reproduce a flight has no business predicting one.
Before any gain sweep is worth a single number, the replay has to show it
computes what the aircraft actually computed, on that aircraft's own
data, at the gains that actually flew.

The log stores the rate loop's INPUTS (`setp_*`, `gyro_*`) and the motor
outputs, but not the PID output in between. That middle value is
recoverable, because mixer_quad_x is invertible - undo the per-motor
scales (fc_tasks.c block 4b) and the table inverts to

    u_roll  = (-FR + FL - RR + RL) / 4
    u_pitch = (-FR - FL + RR + RL) / 4
    u_yaw   = ( FR - FL - RR + RL) / 4

Airmode's collective shift is common-mode and cancels out of all three,
which is exactly why the differentials are the trustworthy part of a
motor record; only the final per-motor clamp corrupts them, and the
mixer reports when that bites.

So verify feeds logged setp and gyro through the real PID and checks the
result against the inverse-mixed motors. This is NOT circular: the
integrator carries history the log never stored, so agreement over a
whole flight tests ki, i_max, the anti-windup freeze, the D filter and
the dt handling at once, against 250 Hz of reality.

READING THE RESULT
------------------
Residual RMS under ~0.5 us is quantisation - setp is logged at 1/16 dps,
motors at 1 us, and the de-scale divides by 0.88. Larger than that means
the replay and the aircraft disagree, and the first suspect is always the
gains: `.params.txt` records what was saved AT ARM, which is not
necessarily what a mid-flight SET left running.

A large `bias` with a near-1.0 `corr` is the interesting failure: the
shape is right and the level is not, which is what a wrong ki or a
different i_max looks like.
"""

import argparse
import csv
import glob
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "pid_replay"))
from pidcore import (PidState, mix, pid_init, pid_step,   # noqa: E402
                     rate_cfg)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rc_trim_check import read_params, scale_of          # noqa: E402

NOMINAL_DT = 1.0 / 250.0        # blackbox is fed once per control cycle
T_WRAP = 2 ** 32                # t_us is the firmware's uint32 microseconds

# fc_tasks.c:409 clamps the measured loop period into this band before any
# controller sees it. The replay MUST clamp identically: a 7 ms cycle the
# aircraft integrated as 6 ms is a different flight, and on a loop with
# this much jitter (p10 3.4 ms, p90 4.4 ms on s121) the difference is not
# academic - leaving it out cost 6 us RMS of residual.
DT_CLAMP_LO = 0.002
DT_CLAMP_HI = 0.006

# Defaults are the gains that flew. Only used when a session
# has no .params.txt beside it.
DEFAULTS = {"rate_kp": 1.2, "rate_ki": 0.05, "rate_kd": 0.03,
            "yaw_kp": 3.0, "yaw_ki": 0.1, "yaw_kd": 0.0,
            "min_idle_us": 1050.0}


class Acc:
    """Running stats for one axis: logged vs replayed."""

    def __init__(self):
        self.n = 0
        self.sx = self.sxx = 0.0        # logged
        self.sy = self.syy = 0.0        # replayed
        self.sxy = 0.0
        self.sd = self.sdd = 0.0        # residual
        self.n_big = 0

    def add(self, logged, replayed):
        d = replayed - logged
        self.n += 1
        self.sx += logged
        self.sxx += logged * logged
        self.sy += replayed
        self.syy += replayed * replayed
        self.sxy += logged * replayed
        self.sd += d
        self.sdd += d * d
        if abs(d) > 1.0:
            self.n_big += 1

    @property
    def rms_logged(self):
        return math.sqrt(self.sxx / self.n) if self.n else float("nan")

    @property
    def rms_resid(self):
        return math.sqrt(self.sdd / self.n) if self.n else float("nan")

    @property
    def bias(self):
        return self.sd / self.n if self.n else float("nan")

    @property
    def corr(self):
        if self.n < 2:
            return float("nan")
        mx, my = self.sx / self.n, self.sy / self.n
        cxy = self.sxy / self.n - mx * my
        cxx = self.sxx / self.n - mx * mx
        cyy = self.syy / self.n - my * my
        if cxx <= 0 or cyy <= 0:
            return float("nan")
        return cxy / math.sqrt(cxx * cyy)


def inverse_mix(motors, scale):
    """Post-scale motor us -> (collective, roll, pitch, yaw) command."""
    raw = [1000.0 + (motors[i] - 1000.0) / scale[i] for i in range(4)]
    fr, fl, rr, rl = raw
    return ((fr + fl + rr + rl) / 4.0,
            (-fr + fl - rr + rl) / 4.0,
            (-fr - fl + rr + rl) / 4.0,
            (fr - fl - rr + rl) / 4.0)


def floor_pinned(motors, scale, idle_us):
    """Is the per-motor scale un-invertible on this frame?

    fc_tasks.c block 4b scales the span and then RE-APPLIES the idle
    floor: `s = 1000 + (out-1000)*k; out = (s < idle) ? idle : s`. Once
    that floor bites, the scaling is destroyed rather than merely
    applied, and dividing by k afterwards recovers a value the mixer
    never produced. On the bench at idle this is the normal case for
    m1 (k = 0.88 pins below 1000 + 50/0.88 = 1057), and it shows up as a
    steady ~1.2 us of fake bias on every static log.

    A motor with k == 1 is unaffected: scaling it is the identity, so
    sitting on the floor is just airmode doing its job.
    """
    return any(motors[i] <= idle_us and scale[i] != 1.0 for i in range(4))


def verify(path, gains, scale, idle_us):
    """Replay one session; -> (per-axis Acc, diagnostics dict)."""
    cfg_r = rate_cfg(gains["rate_kp"], gains["rate_ki"], gains["rate_kd"])
    cfg_p = rate_cfg(gains["rate_kp"], gains["rate_ki"], gains["rate_kd"])
    cfg_y = rate_cfg(gains["yaw_kp"], gains["yaw_ki"], gains["yaw_kd"])
    st_r, st_p, st_y = PidState(), PidState(), PidState()
    for s in (st_r, st_p, st_y):
        pid_init(s)

    acc = {"roll": Acc(), "pitch": Acc(), "yaw": Acc()}
    clean = {"roll": Acc(), "pitch": Acc(), "yaw": Acc()}
    saturated = 0
    n_rows = n_gap = n_sat = n_pinned = 0
    prev_t = None

    opener = open
    if path.endswith(".gz"):
        import gzip
        opener = gzip.open

    with opener(path, "rt", newline="") as fh:
        for row in csv.DictReader(fh):
            # dt from the log, then the firmware's own clamp. A negative
            # delta is the uint32 wrap at ~71 min, not time running
            # backwards, so fold it first.
            t = float(row["t_us"])
            dt = NOMINAL_DT
            if prev_t is not None:
                d = t - prev_t
                if d < 0:
                    d += T_WRAP
                dt = d * 1e-6
            prev_t = t
            if dt < DT_CLAMP_LO or dt > DT_CLAMP_HI:
                n_gap += 1
                dt = min(max(dt, DT_CLAMP_LO), DT_CLAMP_HI)
            n_rows += 1

            motors = [int(row["m1"]), int(row["m2"]),
                      int(row["m3"]), int(row["m4"])]
            collective, u_r, u_p, u_y = inverse_mix(motors, scale)

            mix_r = pid_step(cfg_r, st_r, float(row["setp_roll"]),
                             float(row["gyro_roll"]), dt, saturated)
            mix_p = pid_step(cfg_p, st_p, float(row["setp_pitch"]),
                             float(row["gyro_pitch"]), dt, saturated)
            mix_y = pid_step(cfg_y, st_y, float(row["setp_yaw"]),
                             float(row["gyro_yaw"]), dt, saturated)

            # Run the real mixer so the NEXT cycle's anti-windup flag is
            # the one the aircraft had. Collective comes from the log
            # because alt-hold can replace the throttle stick and the
            # result is never recorded - but a common-mode value cannot
            # change the differentials, only whether the mixer saturates.
            saturated, _ = mix(collective, mix_r, mix_p, mix_y, idle_us)
            n_sat += saturated

            # fc_tasks.c casts to int16 before the mixer, so what the
            # motors encode is the TRUNCATED command. Compare like for
            # like or every axis picks up half a count of fake bias.
            #
            # `clean` drops the frames where the mixer's final clamp bit.
            # That is not tidying an inconvenient number away: when the
            # clamp moves one motor without its siblings the differential
            # in the log is NOT the command that was issued, so the
            # inverse mix has nothing valid to compare against. Those
            # frames are counted and reported instead.
            invertible = not saturated and not floor_pinned(motors, scale,
                                                            idle_us)
            n_pinned += floor_pinned(motors, scale, idle_us)
            for name, logged, replayed in (("roll", u_r, mix_r),
                                           ("pitch", u_p, mix_p),
                                           ("yaw", u_y, mix_y)):
                acc[name].add(logged, float(int(replayed)))
                if invertible:
                    clean[name].add(logged, float(int(replayed)))

    return acc, clean, {"n": n_rows, "gaps": n_gap, "sat": n_sat,
                        "pinned": n_pinned}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("mode", choices=["verify"])
    ap.add_argument("csv", nargs="+", help="session CSV(s), globs allowed")
    for k in DEFAULTS:
        ap.add_argument(f"--{k.replace('_', '-')}", type=float, default=None,
                        help=f"override {k} (default: session .params.txt)")
    args = ap.parse_args()

    paths = []
    for pat in args.csv:
        hits = sorted(glob.glob(pat))
        paths.extend(hits if hits else [pat])
    # One flight is re-pulled into several .bb files by the ring buffer,
    # so the same s<N> decodes under several stems. Dedupe or the pooled
    # numbers get weighted by how often a flight happened to be pulled.
    seen, uniq = set(), []
    for p in paths:
        sid = os.path.basename(p).rsplit("_s", 1)[-1]
        if sid not in seen:
            seen.add(sid)
            uniq.append(p)

    worst = 0.0
    for path in uniq:
        params = read_params(path)
        gains = {}
        for k, dflt in DEFAULTS.items():
            override = getattr(args, k)
            gains[k] = (override if override is not None
                        else params.get(k, dflt))
        if not params:
            print(f"{path}: no .params.txt - replaying at defaults, so a "
                  "large residual may just mean wrong gains", file=sys.stderr)
        try:
            acc, clean, d = verify(path, gains, scale_of(params),
                                   gains["min_idle_us"])
        except (OSError, KeyError) as e:
            print(f"{path}: {e}", file=sys.stderr)
            continue
        if not d["n"]:
            continue

        print(f"\n=== {os.path.basename(path)} ===")
        line = f"  {d['n']} frames ({d['n'] / 250.0:.1f} s)"
        if d["gaps"]:
            line += f", {d['gaps']} dt clamped"
        if d["sat"]:
            line += (f", mixer clamped {d['sat']} "
                     f"({100.0 * d['sat'] / d['n']:.1f}%)")
        if d["pinned"]:
            line += (f", {d['pinned']} scale-pinned at the idle floor "
                     f"({100.0 * d['pinned'] / d['n']:.1f}%)")
        print(line)
        print(f"  gains rate {gains['rate_kp']:g}/{gains['rate_ki']:g}/"
              f"{gains['rate_kd']:g}  yaw {gains['yaw_kp']:g}/"
              f"{gains['yaw_ki']:g}/{gains['yaw_kd']:g}  "
              f"scales {'/'.join(f'{s:g}' for s in scale_of(params))}")
        print(f"  {'axis':<7}{'logged RMS':>12}{'residual RMS':>14}"
              f"{'bias':>10}{'corr':>9}{'|err|>1us':>11}"
              f"{'incl. clamped':>15}")
        for name in ("roll", "pitch", "yaw"):
            a, cl = acc[name], clean[name]
            if not cl.n:
                continue
            worst = max(worst, cl.rms_resid)
            print(f"  {name:<7}{cl.rms_logged:>10.2f} us{cl.rms_resid:>11.3f} us"
                  f"{cl.bias:>8.3f} us{cl.corr:>9.4f}"
                  f"{100.0 * cl.n_big / cl.n:>10.1f}%"
                  f"{a.rms_resid:>12.3f} us")

    if not uniq:
        sys.exit("no sessions matched")
    verdict = ("EXACT - the replay is this flight, sweeps are trustworthy"
               if worst < 0.5 else
               "CLOSE - shape matches, check the gains against .params.txt"
               if worst < 2.0 else
               "MISMATCH - do NOT tune on this until it is explained")
    print(f"\nworst residual RMS across all sessions: {worst:.3f} us  "
          f"[{verdict}]")


if __name__ == "__main__":
    main()
