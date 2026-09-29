#!/usr/bin/env python3
"""Battery-sag throttle compensation - host test of the real flight C.

    python tools/thr_vcomp_test.py

WHAT IS BEING TESTED
--------------------
`thr_vcomp_us()` holds thrust constant as the pack sags, by holding
`duty x V` constant:

    us_out = 1000 + (us_in - 1000) * (V_ref / V_now)

The law is PARAMETER-FREE, which is the whole reason it is worth having.
Thrust follows ELECTRICAL POWER, not bus voltage: from the SunnySky
X2212 980 kV + APC9045 table, at equal thrust the power differs by only
3-4 % between 11.1 V and 14.8 V, and T = a*P^b fits with b = 0.782 and
0.755 at the two voltages (r2 = 0.9996). So there is no thrust exponent
to measure and get wrong - holding the effective voltage the motor sees
holds the thrust.

WHY THE FLIGHT LOG CANNOT CHECK THIS (incident 46)
------------------------------------------
Regressing collective motor command against pack voltage inside a flight
gives -235 us/V on s207 and up to -700 us/V elsewhere, against a physics
value of about -29. Voltage falls with TIME, so every slow drift in the
flight is billed to it and the fit looks beautiful while being wrong.
The law comes from physics and is validated in the air on ALTITUDE, not
on motor command. This file is what checks the arithmetic and every
refusal path; the aircraft checks the effect.

SIZED ON s207
--------------------------
Hover throttle measured at 1460 us at 16.2 V from the steady windows of
s207, NOT the older 1420: that number came from flight4 and s207's
liftoff was misread for the first 38 s, during which the aircraft was
still skittering on the ground at 1430 us. 1460 is 460 us of span.
"""

import ctypes
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from host_c import HostC, CAltHoldCfg, CAltHoldState  # noqa: E402

V_REF = 16.0
HOVER_US = 1460.0          # s207, measured
SPAN = HOVER_US - 1000.0   # 460 us

# alt_hold.c's own rails, reused deliberately - one policy, not two.
VSCALE_MIN, VSCALE_MAX = 0.8, 1.3
VBAT_MIN_PLAUSIBLE, VBAT_MAX_PLAUSIBLE = 9.0, 26.0

fails = []
checks = [0]


def check(name, got, want, tol=1e-3):
    checks[0] += 1
    if abs(got - want) > tol:
        fails.append("%-52s got %.4f want %.4f" % (name, got, want))


def check_true(name, cond):
    checks[0] += 1
    if not cond:
        fails.append("%-52s FALSE" % name)


def main():
    with HostC() as h:
        vcomp = h.lib.ah_vcomp
        vscale = h.lib.ah_vscale

        # ---- the scale itself -------------------------------------
        check("scale at v_ref is exactly 1", vscale(V_REF, 16.0), 1.0)
        check("scale at 15.0 V", vscale(V_REF, 15.0), 16.0 / 15.0)
        check("scale at 16.8 V", vscale(V_REF, 16.8), 16.0 / 16.8)

        # ---- nominal band, sized on the real airframe -------------
        # 4S usable range, hover span 460 us.
        check("16.8 V full pack", vcomp(HOVER_US, V_REF, 16.8),
              1000.0 + SPAN * (16.0 / 16.8))
        check("16.0 V at reference", vcomp(HOVER_US, V_REF, 16.0), HOVER_US)
        check("15.5 V", vcomp(HOVER_US, V_REF, 15.5),
              1000.0 + SPAN * (16.0 / 15.5))
        check("15.0 V, the author's floor", vcomp(HOVER_US, V_REF, 15.0),
              1000.0 + SPAN * (16.0 / 15.0))

        # The whole point: the swing over the usable pack, and the slope
        # at hover. Physics says about 29 us/V here; anything near the
        # -235 us/V the flight log regresses to would be the incident 46 trap
        # baked into the firmware.
        swing = vcomp(HOVER_US, V_REF, 15.0) - vcomp(HOVER_US, V_REF, 16.8)
        check("total swing 16.8 -> 15.0 V, us", swing, 52.57, 0.05)
        slope = swing / (16.8 - 15.0)
        check_true("slope is 25-35 us/V (physics, not the log's -235)",
                   25.0 < slope < 35.0)

        # ---- THROTTLE OFF STAYS OFF -------------------------------
        # The compensation scales the SPAN above 1000, never the floor.
        # If it scaled the absolute value, a sagging pack would spin the
        # motors up with the stick at the bottom.
        for v in (9.0, 12.0, 15.0, 16.0, 16.8, 26.0):
            check("idle stick unmoved at %.1f V" % v,
                  vcomp(1000.0, V_REF, v), 1000.0)

        # ---- refusals: an implausible reading must CHANGE NOTHING --
        # Never clamp a nonsense sensor value into a legal range; reject
        # it. A dead INA226 reports 0 mV, and an unclamped v_ref/0 would
        # be infinite.
        check("dead INA226 (0 V) -> unchanged", vcomp(HOVER_US, V_REF, 0.0),
              HOVER_US)
        check("negative voltage -> unchanged", vcomp(HOVER_US, V_REF, -5.0),
              HOVER_US)
        # 33.41 V is not hypothetical - this aircraft's INA226 produced
        # exactly that sample once (ffe5be8), exactly 2x real.
        check("the real 33.41 V glitch -> unchanged",
              vcomp(HOVER_US, V_REF, 33.41), HOVER_US)
        check("just below the plausible floor -> unchanged",
              vcomp(HOVER_US, V_REF, VBAT_MIN_PLAUSIBLE - 0.01), HOVER_US)
        check("just above the plausible ceiling -> unchanged",
              vcomp(HOVER_US, V_REF, VBAT_MAX_PLAUSIBLE + 0.01), HOVER_US)
        check("v_ref of 0 -> unchanged", vcomp(HOVER_US, 0.0, 15.0),
              HOVER_US)
        check("v_ref of 1 or less -> unchanged", vcomp(HOVER_US, 1.0, 15.0),
              HOVER_US)

        # ---- both clamp rails -------------------------------------
        # Inside the plausible window but past the scale rails: clamped,
        # not refused. A gate that only scales with sigma eventually
        # swallows anything - every adaptive gate needs an absolute cap.
        check("scale clamped at MIN on a 26 V pack",
              vcomp(HOVER_US, V_REF, VBAT_MAX_PLAUSIBLE),
              1000.0 + SPAN * VSCALE_MIN)
        check("scale clamped at MAX on a 9 V pack",
              vcomp(HOVER_US, V_REF, VBAT_MIN_PLAUSIBLE),
              1000.0 + SPAN * VSCALE_MAX)
        check("vscale() itself clamps low", vscale(V_REF, 26.0), VSCALE_MIN)
        check("vscale() itself clamps high", vscale(V_REF, 9.0), VSCALE_MAX)

        # ---- output rail ------------------------------------------
        # Above 15.5 V full stick asks for more than 2000. That is not
        # lost authority: the ESC cannot exceed 100 % duty and max thrust
        # genuinely IS lower at lower voltage. Clamp, do not wrap.
        check("full stick cannot exceed 2000", vcomp(2000.0, V_REF, 12.0),
              2000.0)
        check("near-full stick clamps rather than overflowing",
              vcomp(1950.0, V_REF, 12.0), 2000.0)
        check_true("output never below 1000",
                   vcomp(1000.0, V_REF, 26.0) >= 1000.0)

        # ---- monotonic --------------------------------------------
        # More sag must never mean less throttle, at any stick position.
        for stick in (1100.0, 1300.0, 1460.0, 1700.0):
            prev = -1.0
            ok = True
            v = 16.8
            while v >= 9.0:
                cur = vcomp(stick, V_REF, v)
                if cur < prev - 1e-4:
                    ok = False
                prev = cur
                v -= 0.1
            check_true("monotonic in falling voltage at stick %d" % stick, ok)

        # ---- ONE POLICY, NOT TWO ----------------------------------
        # alt_hold's own hover feedforward must use the SAME scale. With
        # the integrator disabled and the aircraft exactly on setpoint,
        # alt_hold's output IS the feedforward: 1000 + hover_thr*scale.
        # If these two ever diverge the throttle steps at the engage
        # edge, which is the failure this shares one function to avoid.
        cfg = CAltHoldCfg()
        cfg.hover_thr = SPAN
        cfg.v_ref = V_REF
        cfg.i_max = 0.0          # no integrator: out == ff exactly
        cfg.kp_vz = 0.0          # and no rate term
        cfg.kp_z = 0.0
        cfg.vz_max = 1.0
        cfg.thr_deadband = 80.0
        cfg.exit_ramp_s = 0.5
        pcfg = ctypes.byref(cfg)
        for v in (16.8, 16.0, 15.5, 15.0, 12.0, 26.0):
            st = CAltHoldState()
            pst = ctypes.byref(st)
            h.lib.ah_init(pst)
            # engage edge, then a steady cycle on setpoint
            h.lib.ah_update(pst, pcfg, 1, int(HOVER_US), 1.0, 0.0, 0.0, v,
                            0.004)
            out = h.lib.ah_update(pst, pcfg, 1, int(HOVER_US), 1.0, 0.0, 0.0,
                                  v, 0.004)
            check("alt_hold FF uses the same scale at %.1f V" % v,
                  out, 1000.0 + SPAN * vscale(V_REF, v), 0.01)

        # ---- continuity across the engage edge --------------------
        # fc_tasks compensates the stick GOING IN to alt_hold_update
        # rather than the throttle coming out. That is what makes this
        # continuous: the disengaged branch returns the compensated
        # stick, and the engage edge seeds its integrator from the same
        # number. Compensating the OUTPUT instead would both scale the
        # feedforward twice and step the throttle the instant the hold
        # engaged.
        for v in (15.0, 16.8):
            st = CAltHoldState()
            pst = ctypes.byref(st)
            h.lib.ah_init(pst)
            comp = vcomp(HOVER_US, V_REF, v)
            last = h.lib.ah_update(pst, pcfg, 0, int(comp), 1.0, 0.0, 0.0,
                                   v, 0.004)
            check("disengaged output IS the compensated stick at %.1f V" % v,
                  last, comp, 1.01)      # the uint16 round trip
            first = h.lib.ah_update(pst, pcfg, 1, int(comp), 1.0, 0.0, 0.0,
                                    v, 0.004)
            check_true("no step at engage at %.1f V (%.2f -> %.2f)"
                       % (v, last, first), abs(first - last) < 1.0)

    print("thr_vcomp: %d checks, %d failed" % (checks[0], len(fails)))
    for f in fails:
        print("  FAIL " + f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
