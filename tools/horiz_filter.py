#!/usr/bin/env python3
"""Stage C step 2 - the 6-state HORIZONTAL filter, offline reference.

This file is the DESIGN ARTEFACT, exactly as tools/vert_filter.py is for
the vertical channel. A components/common/horiz_filter.c will be a
line-for-line transliteration of it, and a host test will diff the two on
real data. If they ever disagree, THIS one is right and the C is the bug.

    x = [pos_n, pos_e, vel_n, vel_e, bias_n, bias_e]

--------------------------------------------------------------------------
WHY THIS EXISTS AND WHY IT IS NOT THE 15-STATE
--------------------------------------------------------------------------
The GPS-fusion design lists it as step 2, with step 3 (the full 15-state ESKF)
subsuming it. That ordering is deliberate and it is not busywork.

The 15-state estimates ATTITUDE as well, so an attitude error and a
position error feed each other through the rotation that turns body
acceleration into NED. That coupling is where 15-state filters actually
fail, and it is invisible while the easy parts are still being debugged.

Here attitude is taken as given, from Madgwick, via the earth-frame
acceleration fusion.c already publishes. So when this filter misbehaves
the fault is in the GPS ingestion, the NED conventions, the timing or the
R matrix - and every one of those is a thing the 15-state also needs to
get right. Prove them here, where there is one fewer way to be wrong, and
the 15-state's remaining unknown is the coupling alone.

--------------------------------------------------------------------------
THE MEASUREMENT THAT SHAPED THIS FILTER
--------------------------------------------------------------------------
Walk s161, 6 min 15 s, two long blocks with the receiver
PHYSICALLY MOTIONLESS. Scatter about each block's own mean:

    block          pos sd_n   pos sd_e   vel sd_n   vel sd_e   hAcc
    first (109 s)    3.21 m     2.94 m    0.102      0.069     1.01 m
    last   (90 s)    2.22 m     1.07 m    0.106      0.077     0.63 m

Two things follow, and both are load-bearing:

1. hAcc LIES, by a factor of 3.6-4.0. The receiver's own accuracy estimate
   is not a usable R. Building R from hAcc as reported - which is the
   obvious thing to do, and what the plan sketched - would tell this
   filter the position is 4x better than it is, and it would chase
   multipath. R is scaled, and floored, on the numbers above.

2. VELOCITY IS ~30x MORE INFORMATIVE THAN POSITION. 0.1 m/s against
   2-3 m. That is not a small difference in weighting, it is the whole
   character of the filter: position anchors the slow drift, velocity
   does the real work. It is also the measured justification for the
   project's own ladder - velocity hold BEFORE position hold.

--------------------------------------------------------------------------
POSITION AND VELOCITY ARE GATED SEPARATELY. THIS MATTERS.
--------------------------------------------------------------------------
NAV-PVT delivers both in one message, so the obvious implementation is
one 4-DOF update. This filter deliberately does two 2-DOF updates.

The reason is the table above: the two halves of that message have
wildly different quality AND different failure modes. Multipath throws
POSITION by metres while leaving the Doppler VELOCITY almost untouched -
the carrier phase is not fooled the way the code solution is. Gated
together, one bad position excursion rejects the whole message and
throws away a perfectly good velocity, which is the half this filter
most needs.

Separately gated, a multipath fix loses its position and keeps its
velocity. That is the correct outcome and a joint gate cannot express it.
"""

import math

# ---------------------------------------------------------------------------
# Tuning. MEASURED (cited) or a labelled PLACEHOLDER. Nothing unlabelled.
# ---------------------------------------------------------------------------

# GPS position noise, metres per axis, 1 sigma.
#
# MEASURED from s161's two motionless blocks: per-axis sd 1.07, 2.22,
# 2.94, 3.21 m. 3.0 is at the pessimistic end of that, on purpose -
# erring high costs tracking lag, erring low makes the filter chase
# multipath, and the ceiling crash is what the second failure looks like.
GPS_POS_SIGMA_M = 3.0

# hAcc is used, but never at face value. MEASURED understatement 3.6x and
# 4.0x in the two blocks; 4.0 is the honest multiplier. The receiver's
# hAcc still carries real information about RELATIVE quality between one
# fix and the next - it just gets the scale wrong - so R follows it and
# is floored at the measured noise.
GPS_HACC_MULT = 4.0

# GPS Doppler velocity noise, m/s per axis, 1 sigma.
#
# Two measurements, and the larger one wins for a reason.
#
# PARKED, s161: 0.069, 0.077, 0.102, 0.106 m/s. That is the receiver's
# own noise with nothing else going on, and it is what a datasheet would
# call the velocity accuracy.
#
# IN USE, s164: the filter's own velocity innovations have median 0.142
# (north) and 0.124 (east) with a heavy tail out to p90 1.07 and 1.48.
# Against a consistent filter's expected 0.674 sigma that implies an
# effective sigma near 0.21 m/s - 2x the parked figure.
#
# The gap is NOT receiver noise. It is everything the prediction fails to
# anticipate, and on a HAND-CARRIED board that is mostly the board
# swinging in a hand while the GPS antenna travels a different path from
# the accelerometer. Sizing R at the parked 0.12 gated 36% of velocity
# fixes; the filter was rejecting real data because it believed its own
# prediction more than it had earned.
#
# 0.21 is therefore an honest EFFECTIVE noise for this configuration, not
# a fudge - but it is specific to a board carried by hand. RE-MEASURE IT
# BOLTED TO THE AIRFRAME, where the antenna and the IMU share one rigid
# body and this term should fall back toward the parked 0.12.
GPS_VEL_SIGMA_MS = 0.21

# Horizontal accelerometer process noise, m/s^2.
#
# MEASURED from walk s164, the first log carrying validated
# earth-frame horizontal acceleration, by differencing the delta-v
# columns back into per-frame acceleration:
#
#     parked  (41582 samples)  accN mean +0.006 sd 0.694
#                              accE mean +0.008 sd 0.590
#     walking (12900 samples)  accN sd 0.848   accE sd 1.013
#
# 1.0 is the walking figure, which is the regime this filter runs in.
#
# It replaces a placeholder of 2.0 copied from vert_filter.ACC_SIGMA, and
# that number was wrong here for a reason worth keeping: the vertical
# 2.0 is dominated by prop wash and thrust vibration coupling into body
# Z, which is simply not the disturbance the horizontal axes see. Half
# the process noise means the filter trusts its own prediction twice as
# much, which is the whole point of having an accelerometer at all.
#
# The parked means are also the cleanest evidence on record that the
# body->earth rotation is correct: 41582 samples of a stationary board
# average +0.006 and +0.008 m/s^2. Gravity is 9.81 and vertical; if any
# of it were leaking into the horizontal pair through a bad rotation,
# these means could not be seven thousandths.
ACC_SIGMA_MS2 = 1.0

# Accelerometer bias random walk, m/s^2 per sqrt(s). Same value and same
# reasoning as vert_filter.BA_RW_SIGMA: large enough to converge in a few
# seconds, small enough that it cannot chase vibration.
#
# The lesson that constant carries is worth repeating here, because this
# filter has two more bias states to get it wrong in: a state cannot
# learn what its process noise forbids. The vertical filter sat at -0.15
# against a real -3.06 because 3 m/s^2 was a 20-sigma move under its Q.
# Before concluding "the bias never converged", check whether the value
# is inside the sd this number allows.
BIAS_RW_SIGMA = 0.02

# Chi-square gate thresholds, TWO degrees of freedom (north and east
# together), as a probability rather than as "sigmas" - sigma is 1-DOF
# language and does not carry over.
#
# 13.82 is the 99.9% point of chi-square with 2 DOF. One fix in a
# thousand is rejected when everything is healthy, which at ~4 Hz of
# unique fixes is one every four minutes: rare enough not to throw away
# good data, tight enough to catch the multipath excursions measured in
# s161 (radial p95 9.92 m, max 10.48 m, against a 3 m sigma).
GATE_CHI2_2DOF = 13.82

# Initial uncertainty. Position starts unknown-but-bounded rather than
# infinite: home is latched at arm and the aircraft is sitting on it, so
# the truth is metres away at worst, and a huge P here would let the
# first fix take an enormous step.
P0_POS = 25.0          # (5 m)^2
P0_VEL = 1.0           # (1 m/s)^2
P0_BIAS = 1.0          # (1 m/s^2)^2, same as the vertical's accel bias


class HorizFilter(object):
    """Six states, written as one 6x6 rather than two independent 3x3s.

    In THIS model the north and east channels never mix: the dynamics are
    per-axis and the measurements are per-axis, so the off-diagonal 3x3
    blocks of P start at zero and stay exactly zero. Two 3-state filters
    would be mathematically identical and roughly half the arithmetic.

    It is written as a 6x6 anyway, on purpose. The 15-state that subsumes
    this one has GENUINE cross-axis coupling - through the attitude error
    states, an error in yaw rotates north error into east - and the
    machinery for that is the machinery here. Exploiting the block
    structure now would mean tearing it out later, and the saving is
    nothing on a filter that runs at the GPS rate.
    """

    def __init__(self, acc_sigma=ACC_SIGMA_MS2):
        self.pn = self.pe = 0.0
        self.vn = self.ve = 0.0
        self.bn = self.be = 0.0
        self.acc_sigma = acc_sigma

        self.P = [[0.0] * 6 for _ in range(6)]
        self.P[0][0] = self.P[1][1] = P0_POS
        self.P[2][2] = self.P[3][3] = P0_VEL
        self.P[4][4] = self.P[5][5] = P0_BIAS

        self.have_fix = False
        # Diagnostics, and only diagnostics - nothing thresholds these,
        # which is the only reason they may stay counts. Give one a
        # threshold and it has to become seconds.
        self.n_pos_accept = self.n_pos_reject = 0
        self.n_vel_accept = self.n_vel_reject = 0

    # -- state access -----------------------------------------------------
    def state(self):
        return (self.pn, self.pe, self.vn, self.ve, self.bn, self.be)

    # -- prediction -------------------------------------------------------
    def predict(self, accn_ms2, acce_ms2, dt):
        """accn/acce: EARTH-frame horizontal acceleration, m/s^2, +north
        and +east, gravity-free by construction (gravity is vertical in
        the earth frame, so it lands entirely in Z and contributes
        nothing here). This is nav_state.accn_ms2 / acce_ms2, computed in
        fusion.c from the same quaternion that produces accz_ms2.

        Call only on a good IMU read. Integrating a stale acceleration is
        how dead reckoning invents motion - the same rule as the vertical
        filter's predict()."""
        if dt <= 0.0:
            return

        an = accn_ms2 - self.bn
        ae = acce_ms2 - self.be

        self.pn += self.vn * dt + 0.5 * an * dt * dt
        self.pe += self.ve * dt + 0.5 * ae * dt * dt
        self.vn += an * dt
        self.ve += ae * dt
        # biases are random walks: unchanged here, driven only by Q

        # F is identity except for two entries per axis:
        #   d(pos)/d(vel)  = dt
        #   d(pos)/d(bias) = -dt^2/2
        #   d(vel)/d(bias) = -dt
        f_pv = dt
        f_pb = -0.5 * dt * dt
        f_vb = -dt

        P = self.P
        # FP = F*P. Rows 4,5 (the biases) are identity rows, so only the
        # position and velocity rows change.
        FP = [row[:] for row in P]
        for ax in (0, 1):                 # 0 = north, 1 = east
            ip, iv, ib = ax, 2 + ax, 4 + ax
            for j in range(6):
                FP[ip][j] = P[ip][j] + f_pv * P[iv][j] + f_pb * P[ib][j]
                FP[iv][j] = P[iv][j] + f_vb * P[ib][j]

        # (FP)F' - the same combination applied to the columns.
        newP = [row[:] for row in FP]
        for ax in (0, 1):
            ip, iv, ib = ax, 2 + ax, 4 + ax
            for i in range(6):
                newP[i][ip] = FP[i][ip] + f_pv * FP[i][iv] + f_pb * FP[i][ib]
                newP[i][iv] = FP[i][iv] + f_vb * FP[i][ib]

        # Q. Acceleration noise enters position and velocity the way an
        # unknown constant acceleration over dt would; the bias states get
        # their own random walk.
        #
        # The p-v cross term is kept for CORRECTNESS, not for effect.
        # MEASURED (mutation M3): deleting it changes no state
        # by more than 5e-3 over 64322 events on s164, because at a 416 Hz
        # predict dt^3 is ~1e-8. An earlier version of this comment said
        # the term was "large at this dt" - that was wrong, and the
        # mutation test is what caught it.
        sa2 = self.acc_sigma * self.acc_sigma
        q_pp = 0.25 * sa2 * dt ** 4
        q_pv = 0.5 * sa2 * dt ** 3
        q_vv = sa2 * dt * dt
        q_bb = BIAS_RW_SIGMA * BIAS_RW_SIGMA * dt
        for ax in (0, 1):
            ip, iv, ib = ax, 2 + ax, 4 + ax
            newP[ip][ip] += q_pp
            newP[ip][iv] += q_pv
            newP[iv][ip] += q_pv
            newP[iv][iv] += q_vv
            newP[ib][ib] += q_bb

        # Force symmetry. Single precision on the aircraft loses it
        # otherwise, and a non-symmetric P is the first step toward a
        # non-positive-definite one - which in the vertical filter turned
        # a 0.15 m ground step into 1246 m of divergence.
        for i in range(6):
            for j in range(i, 6):
                v = 0.5 * (newP[i][j] + newP[j][i])
                newP[i][j] = v
                newP[j][i] = v
        self.P = newP

    # -- generic 2-DOF update ---------------------------------------------
    def _update2(self, i0, i1, meas0, meas1, r0, r1, gate_chi2):
        """Kalman update for a 2-vector measurement that observes states
        i0 and i1 directly (H is two rows of the identity). Both GPS
        updates have that shape, which is why the innovation covariance
        inverse below can be the closed-form 2x2 rather than a solver.

        Returns (accepted, d2) where d2 is the normalised innovation
        squared - the quantity the chi-square gate tests, and the one
        worth logging when asking why a fix was refused."""
        P = self.P
        x = self.state()

        # S = H P H' + R, a 2x2 built from four elements of P.
        s00 = P[i0][i0] + r0
        s01 = P[i0][i1]
        s11 = P[i1][i1] + r1
        det = s00 * s11 - s01 * s01
        if det <= 0.0:
            return (False, float("inf"))

        y0 = meas0 - x[i0]
        y1 = meas1 - x[i1]

        # d2 = y' S^-1 y, with the 2x2 inverse written out.
        inv00 = s11 / det
        inv01 = -s01 / det
        inv11 = s00 / det
        d2 = y0 * (inv00 * y0 + inv01 * y1) + y1 * (inv01 * y0 + inv11 * y1)
        if gate_chi2 > 0.0 and d2 > gate_chi2:
            return (False, d2)

        # K = P H' S^-1, a 6x2.
        K = []
        for i in range(6):
            a = P[i][i0]
            b = P[i][i1]
            K.append((a * inv00 + b * inv01, a * inv01 + b * inv11))

        upd = [K[i][0] * y0 + K[i][1] * y1 for i in range(6)]
        self.pn += upd[0]
        self.pe += upd[1]
        self.vn += upd[2]
        self.ve += upd[3]
        self.bn += upd[4]
        self.be += upd[5]

        # P = (I - K H) P, where H picks rows i0 and i1.
        newP = [[P[i][j] - (K[i][0] * P[i0][j] + K[i][1] * P[i1][j])
                 for j in range(6)] for i in range(6)]
        for i in range(6):
            for j in range(i, 6):
                v = 0.5 * (newP[i][j] + newP[j][i])
                newP[i][j] = v
                newP[j][i] = v
        self.P = newP
        return (True, d2)

    # -- measurement: GPS position ----------------------------------------
    def update_gps_pos(self, pos_n_m, pos_e_m, h_acc_m):
        """NED position relative to the home point latched at arm, and the
        module's own hAcc.

        h_acc_m is used but NOT believed: MEASURED on s161 it understates
        the real scatter by 3.6-4.0x, so it is multiplied by
        GPS_HACC_MULT and floored at the measured noise. It still carries
        real information about which fixes are better than which - the
        receiver just gets the scale wrong."""
        if not self.have_fix:
            # First fix defines position outright rather than letting P0
            # take one large step into it. Velocity and bias are left
            # alone: a position fix says nothing about either.
            self.pn, self.pe = pos_n_m, pos_e_m
            self.P[0][0] = self.P[1][1] = GPS_POS_SIGMA_M ** 2
            self.have_fix = True
            self.n_pos_accept += 1
            return True

        sigma = max(GPS_POS_SIGMA_M, h_acc_m * GPS_HACC_MULT)
        ok, _d2 = self._update2(0, 1, pos_n_m, pos_e_m,
                                sigma * sigma, sigma * sigma,
                                GATE_CHI2_2DOF)
        if ok:
            self.n_pos_accept += 1
        else:
            self.n_pos_reject += 1
        return ok

    # -- measurement: GPS velocity ----------------------------------------
    def update_gps_vel(self, vel_n_ms, vel_e_ms, s_acc_ms=0.0):
        """NED ground velocity from NAV-PVT's Doppler solution.

        This is the update that does the work. MEASURED noise 0.07-0.11
        m/s against 1-3 m on position, so a velocity fix is worth roughly
        thirty position fixes to this filter - and it is the half that
        survives multipath, which is why it is gated separately."""
        r = max(GPS_VEL_SIGMA_MS, s_acc_ms) ** 2
        ok, _d2 = self._update2(2, 3, vel_n_ms, vel_e_ms, r, r,
                                GATE_CHI2_2DOF)
        if ok:
            self.n_vel_accept += 1
        else:
            self.n_vel_reject += 1
        return ok
