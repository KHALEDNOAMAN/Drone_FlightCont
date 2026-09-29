#!/usr/bin/env python3
"""P7 stage 1 - vertical state estimator, offline reference implementation.

This file is the DESIGN ARTEFACT. components/common/vert_filter.c is a
line-for-line transliteration of it, and tools/vert_replay.py checks both
against a real blackbox log. If the two ever disagree, THIS one is right
and the C is the bug - the whole point of prototyping offline is that the
maths is settled somewhere it can be plotted.

The log cannot check everything: it gates zero ToF samples, so the
re-datum, the datum bound and the runaway detector are exercised only by
tools/vert_gating_test.py, which runs synthetic scenarios through this
file and the C together and requires the same events at the same times.
Edit the gating logic here and you must edit it there too.

--------------------------------------------------------------------------
WHY FOUR STATES AND NOT THREE
--------------------------------------------------------------------------
The plan said three: altitude, climb rate, accelerometer bias. Log A says
four. The fourth is a BARO BIAS, and it is not a refinement - without it
the estimator breaks at exactly the moment alt-hold is most useful.

Log A, 103 s sitting still:
  - ToF held 0.1402 m with sigma = 1.4 mm.
  - The barometer reported an 18 cm descent over the same window.

Two sensors, one aircraft, and they disagree by 18 cm and growing. A
three-state filter has nowhere to put that disagreement: whichever sensor
is trusted less gets its residual dumped into ALTITUDE, which is the one
number the controller closes a loop around. Worse, the baro's absolute
offset (it reports metres above an inferred sea level, the ToF reports
metres above the carpet) is metres, not centimetres - so on the ToF's
first update the estimate would jump by that whole offset.

The fourth state absorbs it:

    baro_reading = altitude + baro_bias

baro_bias soaks up BOTH the arbitrary MSL offset AND the slow drift. It
is observable whenever the ToF is valid, so near the ground the filter
learns "the baro currently reads 84.31 m when we are 0.14 m up" and
locks that in. When the ToF drops out above ~2 m, baro_bias is frozen at
its last learned value and altitude rides the baro from there - no jump,
no step, the handover is invisible. That is the whole trick.

--------------------------------------------------------------------------
STATE VECTOR
--------------------------------------------------------------------------
    x[0] = z    altitude above the ToF ground datum, m, POSITIVE UP
    x[1] = vz   climb rate, m/s, POSITIVE UP
    x[2] = ba   accelerometer Z bias, m/s^2 (measured +0.194 in Log A)
    x[3] = bb   baro bias, m  (baro_reading - true altitude)

Process model, driven by the earth-frame vertical acceleration `a` that
fusion.c already publishes as nav_state.accz_ms2:

    z  += vz*dt + 0.5*(a - ba)*dt^2
    vz += (a - ba)*dt
    ba += 0        (random walk in Q)
    bb += 0        (random walk in Q)

--------------------------------------------------------------------------
WHY THE ACCEL BIAS STATE IS NOT OPTIONAL
--------------------------------------------------------------------------
Measured in Log A: +0.194 m/s^2, which is 1.98% of 1 g - an accelerometer
SCALE error, not a level error (a 1 deg tilt would give 0.002 m/s^2, a
hundred times less). Integrated blind that is 1.9 m/s of phantom climb
rate after 10 s and 9.7 m of altitude error. There is still no accel
calibration on this airframe, so the filter has to find it at runtime.
"""

import math

# ---------------------------------------------------------------------------
# Tuning. Every number here is either MEASURED (Log A, cited) or a stated
# PLACEHOLDER waiting on Log B. Nothing is a guess with no label.
# ---------------------------------------------------------------------------

# Accelerometer process noise, m/s^2. Log A measured sigma = 0.294 m/s^2 at
# IDLE (motors 1050-1136 us, props turning, airframe strapped).
ACC_SIGMA_IDLE = 0.294

# NO LONGER A PLACEHOLDER - measured from real flight logs,
# and the old guess was the reason "too small makes the filter trust a
# vibrating accelerometer and diverge" stopped being a warning and became
# a wrecked aircraft.
#
# The guess was 2x idle = 0.588. Measured, powered flight, accz sigma:
#   ceiling_s30  1.819 m/s^2   (motors ~1107 mean)
#   ceiling_s35  1.774 m/s^2   (motors ~1257 mean, peaks +-8)
#   ceiling_s33  2.831 m/s^2   (motors ~1206 mean, peaks -20/+12)
# i.e. 6x the idle figure and 3x the guess. Vibration does not scale with
# thrust nearly as gently as the reasoning above assumed.
#
# 6.8x lands on 2.0, inside the measured spread and nearer the quiet end
# of it. Deliberately not the 2.83 worst case: that sample is from the
# session where the ToF was already failing and the airframe was being
# flown hard. Re-measure from a clean hover and raise it if that hover
# says so - erring high only costs tracking lag.
ACC_SIGMA_HOVER_MULT = 6.8
ACC_SIGMA = ACC_SIGMA_IDLE * ACC_SIGMA_HOVER_MULT      # 2.0 m/s^2

# Accel-bias random walk, m/s^2 per sqrt(s). Sized so the bias converges
# from 0 to its true value in a few seconds at startup but cannot chase
# vibration. Log A converges in ~4 s at this value.
BA_RW_SIGMA = 0.02

# Baro-bias random walk, m per sqrt(s). Log A: 0.18 m of drift in 103 s
# = 1.8 mm/s. 0.02 gives the state room to track ~10x that without
# letting it race the real altitude signal.
BB_RW_SIGMA = 0.02

# Covariance vz and ba are reset to when the filter re-acquires from a
# LOST state - the same two numbers __init__ uses, for the same reason:
# after a runaway the filter knows as little about them as it did at
# boot. Keep in step with VF_P0_VZ / VF_P0_BA in vert_filter.c.
P0_VZ = 0.01
P0_BA = 1.0

# Barometer measurement noise, m. Log A: sigma = 0.073 m, 0.39 m pk-pk.
BARO_SIGMA = 0.073

# ToF measurement noise. Log A measured sigma = 1.4 mm - but at 0.14 m,
# on a static bench, against one surface. The VL53L0X gets worse with
# range and with dark or angled targets, so R grows with distance:
#     sigma = max(TOF_SIGMA_FLOOR, TOF_SIGMA_FRAC * range)
# The floor is deliberately ~4x the measured 1.4 mm. Trusting a single
# sensor to 1.4 mm would let one bad reflection yank the altitude.
TOF_SIGMA_FLOOR = 0.006     # m
TOF_SIGMA_FRAC = 0.03       # 3% of range

# ToF validity window. Below the floor the module is reading the landing
# gear or a stray reflection; above the ceiling it is at the edge of its
# indoor range and about to report NO_TARGET anyway.
TOF_MIN_M = 0.03
TOF_MAX_M = 2.00

# Reject a ToF update whose innovation exceeds this many sigmas of the
# predicted innovation covariance. This is what keeps the aircraft level
# when it flies over a table: the ToF says the ground jumped up 0.8 m,
# the baro and the accelerometer both say nothing happened, so the ToF
# loses. See TOF_REJECT_S below for the escape hatch.
TOF_GATE_SIGMA = 4.0

# ...but the sigma gate ALONE is not enough, and the replay is what
# showed it. A purely adaptive gate widens while it rejects: every
# rejected sample lets P grow, so after a second or two the gate is wide
# enough to swallow the very step it was built to catch. A 10 cm ground
# change was being accepted outright, never reaching the reject streak,
# and the aircraft followed it - with no datum shift to protect the
# setpoint, because as far as the filter knew the AIRCRAFT had moved.
#
# So the gate is the TIGHTER of the two rules:
#     |innovation| <= min(TOF_GATE_SIGMA * sigma, TOF_GATE_MAX_M)
#
# which splits the behaviour cleanly and predictably in two:
#   step below 10 cm -> followed. This is terrain following at small
#                       scale and it is the intended meaning of a
#                       ground-referenced hold.
#   step above 10 cm -> can never be accepted as aircraft motion. It
#                       goes through the reject streak to a re-datum,
#                       which moves the SETPOINT with it, so the
#                       aircraft is never commanded to fly the step.
TOF_GATE_MAX_M = 0.10

# If the ToF keeps getting gated out for this long, the ground really did
# move (or we really did fly over something) and refusing forever is worse
# than accepting. Re-datum: snap the ground reference, keep vz and both
# biases.
#
# SECONDS, accumulated from dt. Not samples, and no longer frames either
# - this constant has been wrong twice for the same reason. As SAMPLES:
# the ToF's rate IS tof_budget_ms, a live 20-400 ms param, so 20 samples
# written for a 9 Hz poll (~2.2 s) silently became 4.0 s when the budget
# moved to 200 ms. As FRAMES: 500 predicts labelled "250 Hz" is 1.20 s,
# because 250 Hz is the FLIGHT loop and predict() is called from the
# 416 Hz fusion task (fusion.c FUSION_RATE_HZ). dt is already an
# argument; accumulating it is the only count that cannot be wrong.
TOF_REJECT_S = 2.0

# The clock is OPEN when it is > 0, so opening it must not leave it at
# zero. Arbitrary, and deliberately far below one predict: it marks the
# window open and predict() does the timing.
TOF_REJECT_OPEN_S = 1e-6

# How long the estimate stays trustworthy with no accepted ToF fix.
# 3 SECONDS, same accumulation and for the same reason (750 frames was
# 1.80 s on the real 416 Hz fusion task). See the C constant
# VF_TOF_COAST_S for the crash this exists to prevent and for why the
# barometer no longer sets `valid` at all.
TOF_COAST_S = 3.0

# The largest step this filter will move the DATUM under a live estimate.
# Real floors step by a kerb, a stair, a table; 2 m covers all of them.
# Anything larger is not terrain, it is the filter being wrong about
# where it is - and a re-datum is not a local correction, it leaves
# through datum_shift and lands on the controller's altitude SETPOINT.
# s89 : 16 re-datums, 2387 m of commanded setpoint in one
# flight, because nothing asked whether the number was possible. The
# step is not clamped - a clamped bad number is still a bad number - the
# filter clears `valid` and says it is lost. See VF_REDATUM_MAX_M.
REDATUM_MAX_M = 2.0

# How far z may walk away from the ToF, while the ToF is right there
# disagreeing with it, before the ESTIMATE is what is at fault. The two
# failures are identical in one sample and opposite across a window:
# the floor moving steps the ToF and leaves z alone; a runaway holds the
# ToF still and walks z off it. s89 was the second - a -3.06 m/s^2 accz
# bias pushed every innovation outside the 0.10 m gate, 82-88% of ToF
# samples were rejected, and z left while the ToF went on reporting the
# ground correctly at 5 Hz. See VF_TOF_RUNAWAY_M.
TOF_RUNAWAY_M = 1.0

# How still the ESTIMATE must have held during the disagreement window
# for a re-datum to be credited to the GROUND rather than to the filter.
#
# THE QUESTION THIS ANSWERS. A re-datum means "the ToF and I disagree and
# the ToF wins". But WHY they disagree decides what to do with the step:
#
#   the floor moved      -> z held still, the ToF stepped. The aircraft
#                           really is a different height above the ground
#                           now, so the controller's SETPOINT must move
#                           with it or it will climb over the table.
#   the estimate drifted -> the ToF held still, z walked off it. The
#                           aircraft has not moved at all. Correcting z
#                           is right; moving the setpoint is a lie about
#                           where the aircraft is, and it commands a
#                           climb or a dive that nothing asked for.
#
# The old code could not tell these apart and treated every re-datum as
# terrain. MEASURED, flight s170, the 2.5 s before a
# re-datum fired: the ToF moved 0.237 m and the estimate moved 0.960 m -
# four times further. The floor plainly had not moved. The re-datum
# pushed -1.04 m into the altitude setpoint anyway, taking it to -0.74 m,
# BELOW THE GROUND. The hold then saw itself 2.2 m too high and cut up to
# 236 us of throttle. the author felt the drop and disarmed.
#
# 0.25 m: comfortably above the few centimetres a hovering aircraft moves
# inside one window, comfortably below the 0.96 m the estimate walked in
# s170, and well under the smallest terrain step worth following.
TOF_TERRAIN_HOLD_M = 0.25

# Baro innovation gate, sigmas. Looser than the ToF gate - the baro is
# the fallback sensor and gating it out leaves dead reckoning only.
BARO_GATE_SIGMA = 6.0

# The baro needs the same absolute innovation ceiling the ToF has, for the
# identical reason - see TOF_GATE_MAX_M, whose comment describes this
# failure and then only fixes it for the ToF.
#
# MEASURED, blackbox s89 : the DPS310 loses the bus under
# motor load (SENSOR_BARO_OK set in 17% of flying frames) and the reads
# that DO succeed return garbage - consecutive raw altitudes 655 m apart.
# A 6-sigma gate cannot hold that off, because sigma is not a constant:
# acc_sigma 2.0 against a real in-flight accz sigma of 5.2 with a -3.6
# m/s^2 bias grows P fast between baro samples, and a wide-enough gate
# swallows anything. The estimate reached the +-327.67 m log saturation
# and stayed there.
BARO_GATE_MAX_M = 3.0

# How closely the rejected baro samples must agree with EACH OTHER before
# the streak is believed over the filter. Persistence alone is not
# evidence - a stuck-but-healthy baro and a broken one are both
# persistent, and only the healthy one is self-consistent.
BARO_RESYNC_SPREAD_M = 1.0

# ---- Consistency monitoring and covariance inflation ------------------
#
# WHY THIS EXISTS, and what it replaces. Flight logs recorded the vertical
# filter's failure as "absolute innovation ceilings cap how far a HEALTHY
# sensor may correct - the system is bistable". MEASURED on s152
#  that is NOT what happens, and the measurement is easy to
# repeat: replay s152 with BARO_GATE_MAX_M removed entirely and the drift
# is -14.095 m, identical to three decimals to the shipped -14.095 m. The
# cap never bit once in 619 rejections.
#
# What actually rejects the barometer is the SIGMA gate, and the reason is
# that the filter is over-confident:
#
#     median rejected |innovation|   1.210 m
#     the gate it had to clear       0.494 m   (6 * sqrt(S), sqrt(S)=0.082)
#     the absolute cap               3.000 m   - never reached
#
# 83.7% of the rejections sit BETWEEN the gate and the cap. The filter
# claims 8 cm of altitude uncertainty while its true error is metres, so
# every honest correction looks like an outlier. A filter whose innovations
# are 15x larger than the sigma it predicts is not gating noise out - it is
# announcing that its own covariance is wrong.
#
# THE FIX, and why it is not "use a looser gate". Widening the gate to 30
# sigma does fix s152 (drift -1.032 m, 99.0% accepted) but it would also
# swallow the s89 garbage the gate exists to stop - a DPS310 that lost the
# bus and returned altitudes 655 m apart. A constant cannot separate those
# two: both are large innovations.
#
# What separates them is SELF-CONSISTENCY, which is this project's own
# standing rule ("PERSISTENCE IS NOT EVIDENCE; SELF-CONSISTENCY
# IS"). A healthy baro that disagrees with a drifted filter disagrees
# CONSISTENTLY - its rejected samples cluster. A broken one disagrees with
# itself as much as with the filter. So:
#
#   while rejected samples keep agreeing with each other, inflate P.
#
# Inflating P grows S, which widens the gate, which lets the honest sensor
# back in - and it is not a trick, it is the filter correcting the thing
# that was actually wrong. Garbage never triggers it, because garbage
# fails the spread test and resets the window.
#
# The inflation is per SECOND, not per sample. Every constant in this
# filter that means an amount of time accumulates dt, for the reasons
# written at length above VF_TOF_REJECT_S - and BARO_FORCE_STREAK's 50
# samples was the last one still counting the wrong thing.

# How fast P inflates while a consistent disagreement stands, per second.
# 2.0 doubles the covariance every second, so sqrt(S) grows 1.41x/s and
# the gate reaches a 1.2 m median innovation from 0.494 m in ~1.3 s. Fast
# enough that a real climb is not fought for long; slow enough that a
# one-off glitch (which resets the window anyway) cannot open the gate.
BARO_INFLATE_PER_S = 2.0

# Ceiling on the inflation, as a multiple of the covariance the filter had
# when the disagreement opened. THIS is the job the absolute cap was
# supposed to do and never did: bound how far self-doubt may go before the
# filter admits it is lost rather than quietly accepting anything.
# 400x on P is 20x on sigma - 0.494 m of gate becomes 9.9 m, which covers
# every real disagreement measured (max 24.8 m innovation is s89 garbage
# and fails the spread test long before this).
BARO_INFLATE_MAX = 400.0

# Marks the disagreement window OPEN. Same trick and same reason as
# TOF_REJECT_OPEN_S: the window is open when the clock is > 0, so opening
# it must not leave it sitting at zero. predict() does the timing.
BARO_REJECT_OPEN_S = 1e-6

# ---- Sensor TRUST, as distinct from sample plausibility ---------------
#
# WHY A SECOND LAYER EXISTS. Everything above judges ONE SAMPLE against
# the filter. That is necessary and it is not sufficient, and the gating
# fixture proves it: drive this filter with barometer readings scattered
# across +-40 m and 97% are correctly rejected, but the few that happen to
# land near the prediction pass the chi-square test HONESTLY, and each one
# kicks vz through K[1]. Between them the filter dead-reckons, P[1][1]
# grows, so K[1] is large by the time a sample finally lands. MEASURED on
# a STATIONARY aircraft, 8 s:
#
#     git HEAD (before any of this work):  z -162.6 m at vz -40.1 m/s
#
# Every gate did its job and the estimate still ran away. The defect
# predates the chi-square and inflation work - the fixture is simply the
# first thing that ever drove this filter with SCATTERED garbage rather
# than CONSISTENT garbage. It is also the mechanism by which a barometer
# that dies mid-flight into noise can run altitude away with alt-hold
# engaged, which is close to what destroyed the aircraft.
#
# WHAT FIXES IT. Not a tighter per-sample gate - the samples that get
# through are, individually, plausible. What is implausible is the
# SENSOR: it has failed its own self-consistency test over and over. So
# trust is tracked separately from plausibility, exactly as the project's
# standing rule says - check a sensor's LIVENESS, never only its value.
#
# An untrusted barometer is not merely gated: it is not applied at all.
# Its innovations are still COMPUTED, so the filter can watch for it
# coming back, but nothing it says reaches the state.

# Seconds of incoherence that cost the barometer its trust. Incoherence
# accrues only when a rejected sample disagrees with the OTHER REJECTED
# SAMPLES (the spread test resetting), which is the one thing a healthy
# baro disagreeing with a drifted filter never does - its rejections
# cluster. 1.0 s is ~25 samples at the rate the DPS310 delivers here,
# far more than the one-off resets a real sensor produces, far less than
# the 8 s the runaway needed.
#
# MEASURED, sweeping this constant against the runaway fixture: at 1.0 s
# the sensor does lose trust but two garbage samples are accepted first
# and the estimate still reaches -138 m. At 0.5 s only one is accepted
# and the runaway collapses to -2.29 m. Tightening further to 0.06 s
# changes NOTHING on the fixture and only costs trust on real data, so
# 0.5 s is the loosest setting that actually works - which is where a
# safety threshold belongs.
BARO_DISTRUST_S = 0.5

# How fast trust returns, in seconds of samples that both agree with each
# other AND would have passed the gate. Recovery is deliberately slower
# than loss: losing trust costs an altitude reference the ToF can supply,
# regaining it wrongly costs the aircraft.
BARO_RETRUST_S = 2.0

# Trust decays toward zero while the sensor behaves, so that isolated
# resets spread over minutes never accumulate into a distrust. Per second.
BARO_DISTRUST_DECAY_S = 0.25

# Chi-square gating, 1 degree of freedom. d2 = y*y/S is the normalized
# innovation squared; gating on d2 > threshold is EXACTLY the sigma gate
# (d2 > k^2 <=> |y| > k*sqrt(S)) and switching to it changes no numbers.
# It is written this way because the 6-state horizontal filter and the
# 15-state ESKF gate VECTOR innovations, where a per-axis sigma has no
# meaning and d2 = y' S^-1 y is the only form that generalises. Keeping
# one gate shape across all three filters means one thing to validate.
BARO_CHI2 = BARO_GATE_SIGMA * BARO_GATE_SIGMA     # 36.0, = 6 sigma
TOF_CHI2 = TOF_GATE_SIGMA * TOF_GATE_SIGMA        # 16.0, = 4 sigma

# Tilt past this angle makes the ToF slant range meaningless (the cone
# is looking sideways at a wall, not down at the floor).
TOF_MAX_TILT_DEG = 25.0


class VertFilter:
    """4-state vertical estimator. Mirrors components/common/vert_filter.c."""

    def __init__(self, acc_sigma=ACC_SIGMA):
        self.acc_sigma = acc_sigma
        self.reset()

    def reset(self):
        # State
        self.z = 0.0
        self.vz = 0.0
        self.ba = 0.0
        self.bb = 0.0

        # Covariance, row-major 4x4. Initial values are HONEST ignorance:
        # altitude and baro bias are completely unknown (the baro offset is
        # metres), climb rate is known to be zero because we start on the
        # ground, accel bias is known to be small but nonzero.
        self.P = [[0.0] * 4 for _ in range(4)]
        self.P[0][0] = 100.0      # altitude: no idea, metres
        self.P[1][1] = P0_VZ      # vz: we are sitting still, 0.1 m/s
        self.P[2][2] = P0_BA      # accel bias: within +-1 m/s^2
        self.P[3][3] = 10000.0    # baro bias: no idea at all, 100 m

        self.have_baro = False
        # Latest RAW baro reading, stored whether or not the sample was
        # accepted. A lost re-acquire re-derives bb from it - see _redatum.
        self.last_baro_m = 0.0
        self.have_tof = False
        self.tof_reject_streak = 0
        self.baro_reject_streak = 0
        # Range of the baro altitudes rejected during the CURRENT streak;
        # meaningless when baro_reject_streak is 0. See update_baro.
        self.baro_reject_lo = 0.0
        self.baro_reject_hi = 0.0
        # SECONDS the current consistent baro disagreement has stood, and
        # the inflation applied to P because of it. Both are zero whenever
        # no disagreement is open; see BARO_INFLATE_PER_S. Seconds and not
        # samples, because the baro's delivery rate is a live divider and
        # a sample count would mean a different amount of time at each.
        self.baro_reject_s = 0.0
        self.baro_inflation = 1.0
        # Sensor trust, separate from per-sample plausibility. See
        # BARO_DISTRUST_S. `baro_trusted` False means nothing this sensor
        # says reaches the state, however individually plausible it looks.
        self.baro_trusted = True
        self.baro_distrust_s = 0.0
        self.baro_coherent_s = 0.0
        # Seconds between baro samples, measured rather than assumed: the
        # delivery rate is a live divider, so charging distrust per SAMPLE
        # would mean a different amount of time at every setting.
        self.baro_sample_dt = 0.0
        self.baro_dt_acc = 0.0
        self.valid = False
        # Predict calls since the last accepted absolute (baro or ToF)
        # update. A DIAGNOSTIC, and the only reason it may stay a count
        # while the two clocks below may not: nothing thresholds it, so
        # it never has to mean an amount of time.
        self.dead_reckon_frames = 0
        # SECONDS since the last accepted ToF fix. Drives `valid`, which
        # the barometer deliberately does NOT set - see TOF_COAST_S.
        self.tof_coast_s = 0.0
        # SECONDS since the ToF STARTED disagreeing; 0 = none open.
        # Drives the re-datum - see TOF_REJECT_S.
        self.tof_reject_s = 0.0
        # z as of the last moment the ToF and this estimate were tied to
        # each other: an accepted fix, a re-datum, a NO_TARGET, or the
        # opening of the current disagreement window. The ORIGIN of the
        # window whose AGE is tof_reject_s - every place that starts one
        # starts the other. See TOF_RUNAWAY_M.
        self.z_at_last_tof = 0.0

        # Metres of altitude that were added by a DATUM CHANGE rather than
        # by the aircraft moving. Read-and-cleared by the controller.
        #
        # This exists because a re-datum is a coordinate change, not a
        # measurement, and the controller must not react to one. Fly over
        # a table at 1.5 m: the ToF says the ground came up 0.8 m, the
        # gate holds it off for a couple of seconds, then gives up and
        # accepts that the surface really did move. If altitude alone
        # jumped by -0.8 m, an alt-hold loop would see 0.8 m of error and
        # CLIMB OVER THE TABLE. Shifting the setpoint by the same amount
        # makes the aircraft stay exactly where it is, which is the only
        # answer that is right whether the surface is a table or a hill.
        self.datum_shift = 0.0

    # -- prediction -------------------------------------------------------
    def predict(self, accz_ms2, dt):
        """accz_ms2: earth-frame vertical acceleration, gravity removed,
        POSITIVE UP (this is nav_state.accz_ms2). dt in seconds."""
        if dt <= 0.0:
            return
        a = accz_ms2 - self.ba

        # State
        self.z += self.vz * dt + 0.5 * a * dt * dt
        self.vz += a * dt
        # ba, bb unchanged (random walk lives in Q)

        # F, the state Jacobian. Only the nonzero off-diagonals matter.
        #   [1  dt  -dt^2/2  0]
        #   [0   1  -dt      0]
        #   [0   0   1       0]
        #   [0   0   0       1]
        f02 = -0.5 * dt * dt
        f12 = -dt

        P = self.P
        # P = F P F' + Q, written out rather than looped: rows 2 and 3 of F
        # are identity, so most of a generic 4x4x2 multiply is multiplying
        # by 1 and 0. This runs at 416 Hz on an ESP32 next to Madgwick.
        #
        # Step 1: FP = F*P. Only rows 0 and 1 change.
        FP = [row[:] for row in P]
        for j in range(4):
            FP[0][j] = P[0][j] + dt * P[1][j] + f02 * P[2][j]
            FP[1][j] = P[1][j] + f12 * P[2][j]
        # Step 2: P = FP*F'. (FP*F')[i][j] = sum_k FP[i][k]*F[j][k], so
        # only columns 0 and 1 change - by the same coefficients, because
        # F' is F transposed.
        for i in range(4):
            f0, f1, f2, f3 = FP[i]
            P[i][0] = f0 + f1 * dt + f2 * f02
            P[i][1] = f1 + f2 * f12
            P[i][2] = f2
            P[i][3] = f3

        # Q. The accelerometer noise enters through the same channel the
        # acceleration does, so its contribution to (z, vz) is the classic
        # continuous-white-noise-acceleration block. The two biases are
        # independent random walks.
        sa2 = self.acc_sigma * self.acc_sigma
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        P[0][0] += sa2 * dt4 * 0.25
        P[0][1] += sa2 * dt3 * 0.5
        P[1][0] += sa2 * dt3 * 0.5
        P[1][1] += sa2 * dt2
        P[2][2] += BA_RW_SIGMA * BA_RW_SIGMA * dt
        P[3][3] += BB_RW_SIGMA * BB_RW_SIGMA * dt

        self.dead_reckon_frames += 1

        # See TOF_COAST_S. Accumulated on PREDICT from the dt it is
        # handed, so the clock means the same number of SECONDS here, on
        # the aircraft, and at whatever rate the fusion task runs.
        if self.tof_coast_s < TOF_COAST_S:
            self.tof_coast_s += dt
            if self.tof_coast_s >= TOF_COAST_S:
                self.valid = False

        # The disagreement clock, same tick and same reason - see
        # TOF_REJECT_S. 0 means nothing is open, so it only runs once a
        # sample has actually been gated out. Saturates rather than
        # growing: the re-datum needs a distance to re-datum onto, so it
        # can only fire on the next gated sample after this.
        if 0.0 < self.tof_reject_s < TOF_REJECT_S:
            self.tof_reject_s += dt

        # Time since the last baro sample, for the distrust accrual.
        self.baro_dt_acc += dt

        # Re-trust clock. An untrusted barometer is still being evaluated
        # in shadow (see update_baro), and every sample that both agrees
        # with its neighbours and would have passed the gate lets this
        # run. Incoherence zeroes it there. Recovery is deliberately
        # slower than loss.
        if not self.baro_trusted:
            self.baro_coherent_s += dt
            if self.baro_coherent_s >= BARO_RETRUST_S:
                self.baro_trusted = True
                self.baro_distrust_s = 0.0
                self.baro_coherent_s = 0.0

        # Covariance inflation while a CONSISTENT baro disagreement stands.
        #
        # This is the recovery path the filter did not have. Once the
        # estimate has drifted past the gate, every honest baro sample is
        # an outlier and the filter has no way back - measured on s152 as
        # 619 rejections and -14.095 m of drift with the barometer alive
        # and reading correctly the whole time.
        #
        # Inflating P is not a fudge to force the sample in: the filter's
        # covariance is DEMONSTRABLY too small (innovations 15x the sigma
        # it predicts), and this is it admitting that. S grows with P, the
        # gate widens with S, and the sensor is readmitted on the ordinary
        # test rather than through an exception to it.
        #
        # It runs on the predict tick, in seconds, so it means the same
        # thing at every baro divider and every fusion rate. update_baro
        # opens the window and clears it; nothing else touches it.
        if self.baro_reject_s > 0.0 and self.baro_inflation < BARO_INFLATE_MAX:
            self.baro_reject_s += dt
            grow = 1.0 + BARO_INFLATE_PER_S * dt
            if self.baro_inflation * grow > BARO_INFLATE_MAX:
                grow = BARO_INFLATE_MAX / self.baro_inflation
            self.baro_inflation *= grow
            # Scaling the WHOLE matrix by a positive scalar keeps it
            # symmetric and positive-definite, which a per-element nudge
            # does not - and non-positive-definite P is what turned a
            # 0.15 m ground step into 1246 m of divergence once already
            # (see the re-datum comment on leaving P alone).
            P = self.P
            for i in range(4):
                for j in range(4):
                    P[i][j] *= grow

    # -- generic scalar update -------------------------------------------
    def _update(self, H, meas, R, gate_sigma, gate_max=0.0, dry_run=False):
        """Scalar Kalman update. H is a length-4 list. gate_max, if > 0,
        is a hard innovation ceiling applied on top of the sigma gate -
        see TOF_GATE_MAX_M for why the sigma gate alone is not enough.
        Returns (accepted, innovation, innovation_sigma)."""
        P = self.P
        x = [self.z, self.vz, self.ba, self.bb]

        # PH' (column vector)
        PHt = [sum(P[i][j] * H[j] for j in range(4)) for i in range(4)]
        S = sum(H[i] * PHt[i] for i in range(4)) + R
        if S <= 0.0:
            return (False, 0.0, 0.0)

        y = meas - sum(H[i] * x[i] for i in range(4))
        s_sigma = math.sqrt(S)
        if gate_sigma > 0.0:
            # Chi-square form: d2 = y^2/S against the 1-DOF threshold.
            # Identical arithmetic to |y| > k*sqrt(S) - it is the same
            # inequality squared - but it is the form that generalises to
            # the vector innovations of the 6-state and 15-state filters,
            # where d2 = y' S^-1 y and a per-axis sigma means nothing.
            d2 = (y * y) / S
            if d2 > gate_sigma * gate_sigma:
                return (False, y, s_sigma)
            # The ceiling stays, and stays SECOND, because it is the
            # backstop for what the chi-square test cannot see: an
            # inflated P whose gate has grown wide enough to swallow
            # genuine corruption.
            #
            # gate_max is whatever the CALLER passed. That matters: the
            # baro path scales it by sqrt(inflation) (see update_baro) so
            # that a filter which has legitimately grown uncertain can
            # still be corrected, while the ToF path passes TOF_GATE_MAX_M
            # flat, because nothing inflates there and a ground-referenced
            # hold must keep following the floor rather than the estimate.
            # This function must NOT reach for self.baro_inflation itself -
            # it is the one update both sensors share, and a rule that
            # applied to both would silently widen the ToF gate too.
            if gate_max > 0.0 and abs(y) > gate_max:
                return (False, y, s_sigma)

        # dry_run: the sample PASSED, and the caller only wanted to know.
        # Used by the untrusted-sensor shadow path, so that "would this
        # have been accepted" is answered by the real gate rather than by
        # a second copy of it that could drift out of step.
        if dry_run:
            return (True, y, s_sigma)

        K = [PHt[i] / S for i in range(4)]
        self.z += K[0] * y
        self.vz += K[1] * y
        self.ba += K[2] * y
        self.bb += K[3] * y

        # Joseph form would be more numerically polite, but this filter
        # runs in float on a 250 Hz loop and the simple form is what the C
        # can afford. Symmetry is forced below instead, which is the part
        # that actually goes wrong in single precision.
        newP = [[P[i][j] - K[i] * sum(H[k] * P[k][j] for k in range(4))
                 for j in range(4)] for i in range(4)]
        for i in range(4):
            for j in range(i, 4):
                v = 0.5 * (newP[i][j] + newP[j][i])
                P[i][j] = v
                P[j][i] = v
        return (True, y, s_sigma)

    # -- measurement: barometer ------------------------------------------
    def update_baro(self, baro_alt_m):
        """baro_alt_m: whatever the pressure-to-altitude conversion says.
        Absolute value, offset and all - the bb state exists to eat it.

        Nothing here sets `valid`, on purpose: the barometer constrains
        (z + bb), never z alone, so it can only ever adopt a bias that
        agrees with whatever z already is. See TOF_COAST_S."""
        # Recorded before any gate below can reject the sample: a
        # rejected reading is still the best statement the baro has made.
        self.last_baro_m = baro_alt_m

        self.baro_sample_dt = self.baro_dt_acc
        self.baro_dt_acc = 0.0

        if not self.have_baro:
            # First baro sample defines the bias outright rather than
            # letting a 10000 m^2 covariance take a huge first step.
            self.bb = baro_alt_m - self.z
            self.P[3][3] = BARO_SIGMA * BARO_SIGMA * 4.0
            self.have_baro = True
            self.dead_reckon_frames = 0
            return True

        # The ceiling SCALES WITH THE INFLATION, and without that the
        # inflation is decorative. MEASURED before this line existed: a
        # filter 5 m from a healthy, self-consistent barometer inflated to
        # the full 400x ceiling - sqrt(S) 670.850 m, chi-square gate
        # 4025.101 m, so the statistical test passed by three orders of
        # magnitude - and the sample was STILL refused, because 5 m > the
        # flat 3 m cap. Permanent lockout with no recovery path: exactly
        # the bistability seen in the flight logs, reached by a different route
        # than the one it named.
        #
        # The cap asks "is an innovation this large physically possible?".
        # That question has no fixed answer - it depends on how wrong the
        # filter admits it might be. sqrt() because the inflation is on P
        # and the cap is a distance: 400x on the covariance is 20x on
        # sigma, so 3.0 m becomes at most 60 m.
        #
        # This cannot become a licence to accept garbage, and the reason
        # is upstream: inflation only accumulates while the rejected
        # samples agree with EACH OTHER. A sensor whose samples disagree
        # among themselves resets the window and slams baro_inflation back
        # to 1.0, so its ceiling stays at a flat 3.0 m - which is what
        # s89's DPS310 (consecutive altitudes 655 m apart) would meet.
        # An UNTRUSTED barometer runs in shadow: the innovation is still
        # computed, so the filter can see it coming back, but the update
        # is never applied. Passing gate_sigma = -1 asks _update for the
        # verdict without the state change - anything else here would be
        # a second copy of the gate arithmetic, and two copies of a gate
        # is how they drift apart.
        if not self.baro_trusted:
            would, y, s = self._update([1.0, 0.0, 0.0, 1.0], baro_alt_m,
                                       BARO_SIGMA * BARO_SIGMA,
                                       BARO_GATE_SIGMA, BARO_GATE_MAX_M,
                                       dry_run=True)
            # Coherence is measured the same way trust was lost: does this
            # sample agree with the ones around it? The window bookkeeping
            # below is reused verbatim, so a sensor earns its way back by
            # the same test that condemned it.
            lo = min(self.baro_reject_lo, baro_alt_m)
            hi = max(self.baro_reject_hi, baro_alt_m)
            if would and (hi - lo) <= BARO_RESYNC_SPREAD_M:
                self.baro_reject_lo, self.baro_reject_hi = lo, hi
            else:
                self.baro_reject_lo = self.baro_reject_hi = baro_alt_m
                self.baro_coherent_s = 0.0
            return False

        ok, y, s = self._update([1.0, 0.0, 0.0, 1.0], baro_alt_m,
                                BARO_SIGMA * BARO_SIGMA, BARO_GATE_SIGMA,
                                BARO_GATE_MAX_M
                                * math.sqrt(self.baro_inflation))
        if ok:
            self.baro_reject_streak = 0
            self.dead_reckon_frames = 0
            # The disagreement is over: close the window and stop
            # inflating. P is NOT deflated back - the update above already
            # shrank it by the correct amount for the information the
            # sample actually carried, and winding it down by hand on top
            # of that would throw away the filter's own arithmetic.
            self.baro_reject_s = 0.0
            self.baro_inflation = 1.0
            # An accepted sample is the sensor behaving. Decay the charge
            # so isolated resets spread over minutes never accumulate into
            # a distrust - only a sustained run of incoherence does.
            self.baro_distrust_s -= BARO_DISTRUST_DECAY_S * self.baro_sample_dt
            if self.baro_distrust_s < 0.0:
                self.baro_distrust_s = 0.0
        else:
            # Track what the rejected samples say about each other, not
            # just how many there have been. The window RESTARTS the
            # moment a sample lands outside it, so the streak counts a
            # run of samples that AGREE - without that, a baro that spat
            # garbage once could never re-sync again however healthy it
            # went on to be.
            lo, hi = self.baro_reject_lo, self.baro_reject_hi
            lo = min(lo, baro_alt_m)
            hi = max(hi, baro_alt_m)
            if self.baro_reject_streak == 0 or (hi - lo) > BARO_RESYNC_SPREAD_M:
                self.baro_reject_lo = self.baro_reject_hi = baro_alt_m
                self.baro_reject_streak = 1
                # The samples stopped agreeing with each other, so this is
                # not a filter that has drifted - it is a sensor that is
                # lying. Slam the inflation shut and start over. THIS is
                # what keeps s89's +-327 m garbage from opening the gate:
                # it fails the spread test on nearly every sample and can
                # never accumulate the seconds inflation needs.
                self.baro_reject_s = BARO_REJECT_OPEN_S
                # The sensor just contradicted its OWN recent rejections.
                # A healthy baro arguing with a drifted filter never does
                # this - its rejections cluster. Charge it, in seconds of
                # the interval since the last sample, so the accrual means
                # the same thing at every baro divider.
                self.baro_distrust_s += self.baro_sample_dt
                if self.baro_distrust_s >= BARO_DISTRUST_S:
                    self.baro_trusted = False
                    self.baro_coherent_s = 0.0
                    # Losing trust must also CLOSE the inflation window.
                    # An untrusted sensor is not applied, so it has no
                    # business driving the filter's covariance either -
                    # and left open, predict() would keep inflating P
                    # unboundedly the whole time the sensor is ignored,
                    # so the gate would be wide open the moment trust
                    # came back. Undo the inflation, do not merely stop.
                    if self.baro_inflation > 1.0:
                        P = self.P
                        shrink = 1.0 / self.baro_inflation
                        for i in range(4):
                            for j in range(4):
                                P[i][j] *= shrink
                    self.baro_inflation = 1.0
                    self.baro_reject_s = 0.0
                if self.baro_inflation > 1.0:
                    P = self.P
                    shrink = 1.0 / self.baro_inflation
                    for i in range(4):
                        for j in range(4):
                            P[i][j] *= shrink
                    self.baro_inflation = 1.0
            else:
                self.baro_reject_lo, self.baro_reject_hi = lo, hi
                self.baro_reject_streak += 1
                # Consistent disagreement. Open the window if it is not
                # already open; predict() does the timing and the growing.
                if self.baro_reject_s <= 0.0:
                    self.baro_reject_s = BARO_REJECT_OPEN_S

            # WHAT USED TO BE HERE: `if baro_reject_streak > 50: accept it
            # UNGATED`. Both halves of that were wrong.
            #
            # It counted SAMPLES, so "a full second" was whatever the live
            # baro divider made it - the last constant in this filter still
            # measuring time in the wrong unit.
            #
            # And it forced the sample in with BOTH gates disabled, which
            # is the project's own warning written as code: persistence is
            # not evidence, and an ungated accept hands the state to
            # whichever sensor is most stubbornly wrong. The self-agreement
            # test in front of it helped, but the accept behind it was
            # still unconditional.
            #
            # The inflation window replaces it and is strictly stronger: a
            # consistently-disagreeing baro gets back in by the ORDINARY
            # chi-square test, once the filter has widened its own gate to
            # reflect the uncertainty it demonstrably has - and it must
            # still clear the absolute ceiling on the way. Nothing is ever
            # accepted ungated any more.
        return ok

    # -- measurement: ToF -------------------------------------------------
    def update_tof(self, tof_m, roll_deg, pitch_deg):
        """tof_m: raw VL53L0X slant range in metres, already screened for
        the 65535 NO_TARGET sentinel. NOT screened by device range status
        - that gate was removed, see the C driver's
        vl53l0x_cached_mm(). roll/pitch in deg
        from the AHRS - the ToF points along body -Z, so a tilted aircraft
        measures a longer slant range than its true height."""
        if not (TOF_MIN_M <= tof_m <= TOF_MAX_M):
            return False

        cr = math.cos(math.radians(roll_deg))
        cp = math.cos(math.radians(pitch_deg))
        tilt_cos = cr * cp
        # cos of the total tilt angle; below this the cone is looking at a
        # wall and the range is not a height at all.
        if tilt_cos < math.cos(math.radians(TOF_MAX_TILT_DEG)):
            return False

        agl = tof_m * tilt_cos

        sigma = max(TOF_SIGMA_FLOOR, TOF_SIGMA_FRAC * agl)
        # ---- LOST: the ToF wins, immediately ----------------------
        #
        # THE PRINCIPLE, and it should have been here from the start: a
        # VL53L0X does not lie. Its failure mode is SILENCE - a range
        # under ~2 m, or the 65535 NO_TARGET sentinel, and nothing in
        # between. That is the opposite of a dying barometer, which
        # returns confident nonsense, and the two must not be treated
        # with the same "the filter might be right" logic.
        #
        # So once `valid` is false the filter has ALREADY ADMITTED it
        # does not know where the ground is, and at that moment a ToF
        # reading is by construction more trustworthy than the dead
        # reckoning that replaced it. There is nothing left to protect:
        # alt-hold has been handed back, there is no setpoint to yank.
        # Waiting is pure downside.
        #
        # MEASURED, s171, which is what forced this. z ran
        # from -1.1 m to -5.97 m over 8.5 s while the ToF sat at 0.11-
        # 0.33 m and every sample was rejected. The cause was a LIVELOCK
        # between two individually sensible rules:
        #
        #   the re-datum needs TOF_REJECT_S (2.0 s) of steady disagreement
        #   the runaway detector RESETS that clock whenever z walks
        #   TOF_RUNAWAY_M (1.0 m) from the window origin
        #
        # z was drifting at 0.6 m/s, so it covered 1 m every 1.7 s and
        # reset the clock before it could ever reach 2 s. Measured over
        # the window: the runaway detector fired 48 times and the
        # re-datum clock never exceeded 0.000 s. The safety mechanism
        # starved the recovery mechanism.
        #
        # This branch breaks the loop by removing the wait entirely in
        # the only state where the wait was never buying anything.
        if self.have_tof and not self.valid:
            self._redatum(agl, sigma, emit_shift=False,
                          lost_reacquire=True)
            self.tof_reject_streak = 0
            return True

        if not self.have_tof:
            # First valid ToF DEFINES the ground datum, and is exempt
            # from REDATUM_MAX_M explicitly rather than by luck: there is
            # no previous datum for it to be a step away from, and its dz
            # is only the distance from an arbitrary z = 0 start to the
            # floor. Bounding it would mean booting with a ground
            # reference the filter already knows to be wrong.
            self._redatum(agl, sigma)
            self.have_tof = True
            self.valid = True
            return True

        ok, y, s = self._update([1.0, 0.0, 0.0, 0.0], agl,
                                sigma * sigma, TOF_GATE_SIGMA, TOF_GATE_MAX_M)
        if ok:
            self.tof_reject_streak = 0
            self.tof_reject_s = 0.0
            self.dead_reckon_frames = 0
            self.tof_coast_s = 0.0
            self.z_at_last_tof = self.z
            self.valid = True
            return ok

        self.tof_reject_streak += 1
        # Start the clock on the FIRST gated sample; predict() runs it
        # from there. A count of samples cannot time this. The window's
        # ORIGIN is set with its age: z as it stands now is what the next
        # two seconds of drift are measured against.
        if self.tof_reject_s <= 0.0:
            self.tof_reject_s = TOF_REJECT_OPEN_S
            self.z_at_last_tof = self.z

        # Did the ESTIMATE run away, rather than the floor moving? Asked
        # on every gated sample and not just at expiry, because waiting
        # out the full two seconds to notice hands a diverging estimate
        # the 1.6 s that put an aircraft through a ceiling.
        #
        # Nothing is re-datumed while this holds, and the window RESTARTS
        # instead, so the question stays "how far has z walked in the
        # last couple of seconds" and never becomes "since some fix
        # minutes ago" - a distance that can only grow is a verdict the
        # filter could never argue its way out of.
        if abs(self.z - self.z_at_last_tof) > TOF_RUNAWAY_M:
            self.valid = False
            self.tof_reject_s = TOF_REJECT_OPEN_S
            self.z_at_last_tof = self.z
            return ok

        if self.tof_reject_s >= TOF_REJECT_S:
            # The ground moved, it has kept saying so for two seconds,
            # and the estimate held still throughout while it said so.
            # Accept the new surface as the datum. (_redatum clears the
            # clock.)
            #
            # Unless the step is larger than any floor. Then the honest
            # answer is that we do not know where we are, so say it:
            # clearing `valid` costs the controller the hold instead of
            # costing it a 2387 m setpoint.
            #
            # The `valid` half of the test is what keeps this ESCAPABLE,
            # and it is a statement about the CONTROLLER, not the filter.
            # The bound exists because a re-datum walks straight into the
            # altitude setpoint, so it only has to hold while something
            # can still be closing a loop on this estimate. Once `valid`
            # is false, alt-hold has already been handed back and there
            # is no setpoint left to protect - a big step is then just
            # this filter re-acquiring a datum it has admitted losing,
            # exactly like the first fix above. Refusing it forever would
            # strand the estimator on a ground reference it has already
            # disowned, which is the one failure worse than the jump.
            #
            # So a large step costs one extra window, never permission.
            if self.valid and abs(agl - self.z) > REDATUM_MAX_M:
                self.valid = False
                self.tof_reject_s = TOF_REJECT_OPEN_S
                self.z_at_last_tof = self.z
                return ok
            # WHO MOVED? See TOF_TERRAIN_HOLD_M. The step only reaches
            # the controller's setpoint when the estimate held still and
            # the ground is what changed.
            walked = abs(self.z - self.z_at_last_tof)
            self._redatum(agl, sigma,
                          emit_shift=(walked < TOF_TERRAIN_HOLD_M))
            self.tof_reject_streak = 0
        return ok

    def _redatum(self, agl, sigma, emit_shift=True,
                 lost_reacquire=False):
        """Move the ground datum to whatever the ToF is now measuring.

        This is a CHANGE OF ORIGIN, not new information about where the
        aircraft is, and getting that distinction wrong breaks the filter
        two separate ways - the replay caught both:

        1. The baro bias moves the OPPOSITE way to the altitude. The
           measurement model is `baro = z + bb`, and the barometer did
           not change when the floor did, so z += dz forces bb -= dz.
           Adding dz to both moved the predicted baro reading by 2*dz and
           the next baro update spent seconds dragging it back.

        2. The covariance must be left ALONE. Slamming P[0][0] down to
           the ToF's variance while every off-diagonal term stayed sized
           for the old uncertainty made P non-positive-definite, and a
           +0.15 m ground step diverged to 1246 m in a few seconds. A
           translation of the state vector does not change its
           covariance; the ToF's precision enters through the ordinary
           update below, whose innovation is now exactly zero, so it
           shrinks P correctly and moves the state not at all.
        """
        dz = agl - self.z
        self.z = agl
        if self.have_baro:
            if lost_reacquire:
                # RE-DERIVE bb, do not adjust it by dz.
                #
                # `bb -= dz` is the right rule for a GROUND-MOVED
                # re-datum and only there: it preserves `baro = z + bb`,
                # which is worth preserving exactly when the state
                # already agreed with the barometer. On a lost
                # re-acquire it did not - the baro is gated out
                # PRECISELY BECAUSE z + bb stopped matching the reading
                # - so adjusting by dz carries the whole excursion into
                # bb instead of ending it.
                #
                # MEASURED on s180 before this: bb moved +2.328 m across
                # the flight while the barometer moved +0.060 m. Setting
                # bb from the reading restores `baro = z + bb` in one
                # step, and is the rule the first baro sample uses.
                self.bb = self.last_baro_m - self.z
            else:
                self.bb -= dz
        # The aircraft did not move, so the controller's setpoint must
        # move with the datum. This is what stops it climbing over a
        # table. Read and cleared by take_datum_shift().
        # emit_shift=False when re-acquiring from a LOST state.
        #
        # datum_shift exists for ONE purpose: to keep a controller's
        # altitude setpoint aligned when the GROUND moves under a live
        # hold. If `valid` was false there is no live hold - alt-hold has
        # already been handed back - so there is no setpoint to keep
        # aligned, and emitting the step would push a number straight
        # into a controller that is not using it and will re-engage from
        # scratch anyway.
        #
        # This is not a detail. The gating test caught it immediately:
        # emitting the shift on a lost re-acquire put +103.3 m into
        # datum_shift during the runaway scenario, which is the same path
        # that shipped 2387 m of commanded setpoint in s89. A recovery
        # that resurrects the worst bug in the file is not a recovery.
        if emit_shift:
            self.datum_shift += dz

        # LOST RE-ACQUIRE: the velocity that walked the estimate off a
        # steady ToF is exactly as wrong as the position, so it goes too,
        # along with everything that claimed to know about it. The cross
        # terms matter - a fresh P[1][1] beside a stale P[0][1] is the
        # covariance shape that turned a 0.15 m step into 1246 m.
        #
        # ba is re-opened because ba is what GENERATES the phantom
        # velocity, and a confident P[2][2] means the ToF samples that
        # follow get absorbed by z instead of moving ba.
        #
        # The zero-innovation update below is KEPT on both paths, and
        # that was measured rather than reasoned. Skipping it here - on
        # the argument that shrinking P right after admitting you are
        # lost is overconfidence - leaves ba at -0.752 after 10 s of the
        # s89 scenario with z wandering 2.25 m. Keeping it, ba converges
        # to -3.057 against a true -3.06 and z tracks the ToF to 0.000 m
        # from t = 4 s. The inflation says "ba is unknown"; this update
        # is what re-establishes how ba and z RELATE given the ToF's
        # precision, and without that the innovations that follow are
        # absorbed entirely by z.
        if lost_reacquire:
            self.vz = 0.0
            for i in range(4):
                self.P[1][i] = 0.0
                self.P[i][1] = 0.0
            self.P[1][1] = P0_VZ
            if self.P[2][2] < P0_BA:
                self.P[2][2] = P0_BA
        self._update([1.0, 0.0, 0.0, 0.0], agl, sigma * sigma, 0.0)
        self.dead_reckon_frames = 0
        self.tof_coast_s = 0.0
        self.tof_reject_s = 0.0
        # Datum and estimate are the same number again, so this is where
        # the next disagreement window gets measured from.
        self.z_at_last_tof = self.z
        self.valid = True

    def take_datum_shift(self):
        """Read and clear the accumulated datum change. The controller
        calls this every cycle and adds the result to its setpoint."""
        d = self.datum_shift
        self.datum_shift = 0.0
        return d

    def tof_lost(self):
        """Call when the ToF reports NO_TARGET or goes out of range. The
        baro bias FREEZES at its learned value - that frozen number is
        what makes the handover to baro-only seamless.

        The sensor is SAYING it cannot see the floor, so there is no
        disagreement to measure and nothing to measure it from: the next
        window starts wherever z is when the ToF comes back. Drift piled
        up across a dropout the sensor ANNOUNCED is not evidence of a
        runaway - the aircraft may simply have climbed out of range and
        flown back down. A SILENT death is the opposite case: nothing
        calls this, samples keep arriving with a stale value, and the
        window keeps running. That asymmetry is deliberate."""
        self.tof_reject_streak = 0
        self.tof_reject_s = 0.0
        self.z_at_last_tof = self.z

    def state(self):
        return (self.z, self.vz, self.ba, self.bb)
