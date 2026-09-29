/* P7 stage 1 - vertical state estimator. See vert_filter.h for the API
 * and tools/vert_filter.py for the design, the derivation, and the
 * reason there are four states and not three.
 *
 * No math.h on purpose. __builtin_sqrtf and __builtin_fabsf are provided
 * by both GCC and Clang, compile to a single instruction on every target
 * this will ever see, and leave this file with NO libc dependency at all
 * - which is what lets tools/vert_ctest.py compile and run it on the
 * host and diff it against the Python, on a machine whose only C
 * compilers are cross-compilers and a headerless clang. The gating
 * branches below are reached by NO log fixture; tools/vert_gating_test.py
 * is the only thing that tests them, in C and Python together. */

#include "vert_filter.h"

#define VF_SQRTF __builtin_sqrtf
#define VF_FABSF __builtin_fabsf

/* ---- Tuning. Every value is MEASURED (Log A) or a labelled PLACEHOLDER
 * waiting on Log B. Keep in step with tools/vert_filter.py. ---- */

/* MEASURED from powered flight, replacing the 2x-idle guess.
 * Log A's 0.294 m/s^2 is the IDLE figure (motors 1050-1136 us) and the
 * old multiplier of 2 turned it into 0.588. Real flight, accz sigma:
 * 1.774 (ceiling_s35), 1.819 (s30), 2.831 (s33) - 6x idle, 3x the guess.
 *
 * The old comment said "too small makes the filter trust a vibrating
 * accelerometer and diverge". It was right, and that is
 * exactly what happened. Keep this at or above the measured value.
 * Erring high only costs tracking lag. Keep in step with
 * tools/vert_filter.py. */
#define VF_ACC_SIGMA_DEFAULT   2.0f
#define VF_BARO_SIGMA_DEFAULT  0.073f      /* Log A: sigma 0.073 m       */

#define VF_BA_RW_SIGMA         0.02f       /* accel-bias random walk     */
#define VF_BB_RW_SIGMA         0.02f       /* baro-bias random walk      */

/* Covariance the velocity and accel-bias states are reset to when the
 * filter re-acquires from a LOST state. Named rather than inline because
 * vert_filter_init uses the same two numbers for the same reason: after
 * a runaway the filter knows as little about vz and ba as it did at
 * boot, and pretending otherwise is what let the next runaway start
 * immediately. Keep in step with tools/vert_filter.py. */
#define VF_P0_VZ               0.01f       /* +-0.1 m/s                  */
#define VF_P0_BA               1.0f        /* +-1 m/s^2                  */

#define VF_TOF_SIGMA_FLOOR     0.006f      /* 4x the 1.4 mm Log A sigma  */
#define VF_TOF_SIGMA_FRAC      0.03f       /* + 3% of range              */
#define VF_TOF_MIN_M           0.03f
#define VF_TOF_MAX_M           2.00f

#define VF_TOF_GATE_SIGMA      4.0f
/* The sigma gate alone is NOT enough, and the replay is what showed it:
 * a purely adaptive gate widens while it rejects, so after a second or
 * two it swallows the very step it exists to catch. A 10 cm ground
 * change was being accepted as aircraft motion with no datum shift to
 * protect the setpoint. The gate is the TIGHTER of the two rules, which
 * splits the behaviour predictably: below 10 cm the filter follows the
 * ground (terrain following, which is what a ground-referenced hold
 * MEANS), above it the step can only reach the datum, never the
 * command. */
#define VF_TOF_GATE_MAX_M      0.10f

/* How long the ToF may keep disagreeing before the SURFACE is believed
 * instead of the datum.
 *
 * SECONDS, accumulated from dt. Not samples, and no longer frames
 * either - this one constant has now been wrong TWICE for the same
 * reason, which is why it is finally measured in the unit it is
 * expressed in:
 *
 *   1. SAMPLES. The ToF's sample rate IS `tof_budget_ms`, a live
 *      20-400 ms param, so a sample count means anywhere from 0.4 s to
 *      8 s depending on a value that can be changed from the console.
 *      The original 20 samples were written for the 9 Hz this sensor
 *      used to be polled at (~2.2 s) and silently became 4.0 s the day
 *      the budget moved to 200 ms - nothing in the code changed, and
 *      nothing announced it.
 *   2. FRAMES. The fix for (1) counted PREDICTS and labelled them
 *      "250 Hz". But 250 Hz is the FLIGHT loop (fc_tasks.c); predict()
 *      is called from the fusion task, which is paced by the LSM6DSOX
 *      INT1 line at 416 Hz (fusion.c FUSION_RATE_HZ). So 500 frames was
 *      1.20 s, not the 2.0 s written next to it - and nothing announced
 *      that either.
 *
 * dt is already an argument to predict(). Accumulating it means two
 * seconds at every budget AND at every loop rate, on the aircraft and
 * in the host replay alike. tof_reject_streak still counts samples, but
 * only as a diagnostic; it no longer triggers anything. */
#define VF_TOF_REJECT_S        2.0f        /* seconds of disagreement    */

/* The clock is OPEN when it is > 0, so opening it must not leave it
 * sitting at zero. The value is arbitrary and deliberately far below one
 * predict: it marks the window open, and predict() does the timing. */
#define VF_TOF_REJECT_OPEN_S   1e-6f

/* Below this the ToF cone is looking at a wall, not the floor. 25 deg. */
#define VF_TOF_MIN_TILT_COS    0.9063f

#define VF_BARO_GATE_SIGMA     6.0f

/* The baro needs the same absolute innovation ceiling the ToF has, and
 * for the identical reason - see VF_TOF_GATE_MAX_M above, whose comment
 * describes this failure and then only fixes it for the ToF.
 *
 * MEASURED, blackbox s89 : the DPS310 loses the bus under
 * motor load (SENSOR_BARO_OK set in 17% of flying frames) and the reads
 * that DO succeed return garbage - consecutive raw altitudes 655 m
 * apart. A 6-sigma gate cannot hold that off, because sigma is not a
 * constant: acc_sigma 2.0 against a real in-flight accz sigma of 5.2
 * with a -3.6 m/s^2 bias grows P fast between the ~2 Hz the baro
 * actually delivered at, and a wide-enough gate swallows anything. The
 * estimate reached the +-327.67 m log saturation and stayed there.
 *
 * 3 m is far outside any innovation this filter should ever see once bb
 * is learned - baro noise is 0.073 m and the drift is 18 cm/100 s - and
 * far inside the corruption it exists to stop. A genuine fast climb the
 * accelerometer under-reported reaches it only at ~3 m of error, which
 * is what the re-sync below is for. */
#define VF_BARO_GATE_MAX_M     3.0f

/* ---- Consistency monitoring and covariance inflation ----
 *
 * The full derivation, the measurements and the reason the OLD diagnosis
 * was wrong are in tools/vert_filter.py. In short, MEASURED on s152
 * 
 *
 *   - VF_BARO_GATE_MAX_M never bit once in 619 rejections. Replaying with
 *     the cap removed entirely gives -14.095 m of drift, identical to
 *     three decimals to the shipped filter. The absolute ceiling was NOT
 *     what locked the barometer out.
 *   - the SIGMA gate did all of it: median rejected innovation 1.210 m
 *     against a gate of 0.494 m (6*sqrt(S), sqrt(S) = 0.082). 83.7% of
 *     the rejections sat between the gate and the 3 m cap.
 *
 * A filter whose innovations are 15x the sigma it predicts is not
 * rejecting outliers, it is announcing that its covariance is wrong. So
 * while rejected samples keep AGREEING WITH EACH OTHER, P is inflated
 * until the gate reflects the uncertainty the filter demonstrably has,
 * and the sensor is readmitted by the ordinary test. Samples that
 * disagree with each other reset it, which is what keeps s89's +-327 m
 * garbage out. Persistence is not evidence; self-consistency is. */
#define VF_BARO_INFLATE_PER_S  2.0f     /* doubles P per second        */
#define VF_BARO_INFLATE_MAX    400.0f   /* 400x on P = 20x on sigma    */
#define VF_BARO_REJECT_OPEN_S  1e-6f    /* marks the window open       */

/* ---- Sensor TRUST, as distinct from sample plausibility ----
 *
 * Everything above judges ONE SAMPLE against the filter. Necessary, and
 * NOT sufficient. Drive this filter with barometer readings scattered
 * across +-40 m and 97% are correctly rejected - but the few that land
 * near the prediction pass the chi-square test HONESTLY, and each kicks
 * vz through K[1], which is large because P[1][1] grew while the filter
 * dead-reckoned between them. MEASURED on a STATIONARY aircraft, 8 s:
 *
 *     git HEAD (before any of this work):  z -162.6 m at vz -40.1 m/s
 *
 * Every gate did its job and the estimate still ran away. The defect
 * PREDATES the chi-square and inflation work - that fixture is simply
 * the first thing ever to drive this filter with SCATTERED garbage
 * instead of CONSISTENT garbage. It is also the mechanism by which a
 * barometer dying mid-flight into noise runs altitude away under
 * alt-hold, which is close to what destroyed the aircraft.
 *
 * The fix is not a tighter per-sample gate: the samples that get through
 * are individually plausible. What is implausible is the SENSOR, which
 * has failed its own self-consistency test over and over. So trust is
 * tracked separately - the project's standing rule, check a sensor's
 * LIVENESS and never only its value. An untrusted barometer is not
 * merely gated: it is not applied at all. Its innovations are still
 * COMPUTED so the filter can see it recover, but nothing it says reaches
 * the state.
 *
 * 0.5 s is MEASURED, not chosen: at 1.0 s trust is lost but two garbage
 * samples land first and the estimate still reaches -138 m; at 0.5 s one
 * lands and the runaway collapses to -2.29 m; tightening to 0.06 s
 * changes nothing on the fixture and only costs trust on real data. Log A
 * never loses trust at this setting (0 frames of 25789, charge 0.0000). */
#define VF_BARO_DISTRUST_S      0.5f    /* incoherence that costs trust */
#define VF_BARO_RETRUST_S       2.0f    /* coherence that earns it back */
#define VF_BARO_DISTRUST_DECAY  0.25f   /* per second, while behaving   */

/* How closely the rejected baro samples must agree with EACH OTHER
 * before the streak is believed over the filter. See baro_reject_lo/hi
 * in the header for why persistence alone is not evidence.
 *
 * 1.0 m: comfortably above the noise and drift a genuinely healthy baro
 * shows across one streak, and 655x below the spread s89's broken one
 * produced. A sensor whose samples disagree with each other by metres
 * is not telling the truth about anything. */
#define VF_BARO_RESYNC_SPREAD_M 1.0f

/* How long the estimate stays trustworthy with no accepted ToF fix.
 *
 * this number did not exist and an aircraft was destroyed.
 * The ToF's power wire had fallen out; the chip still ACKed, so the
 * driver served its last reading forever and the fusion loop never saw
 * a fresh sample to reject. `valid` stayed true because the BAROMETER
 * was setting it, alt-hold engaged on an estimator with no absolute
 * reference, the estimate dived through the floor, and the controller
 * answered a false -3.4 m/s with +264 us of thrust.
 *
 * Two changes came out of that, and they are separate ideas:
 *
 *   1. The baro no longer sets `valid` at all. In the model
 *      `baro = z + bb`, altitude and baro bias are NOT separately
 *      observable from the barometer alone - only the ToF splits them.
 *      A sensor that cannot tell you where the ground is has no
 *      business certifying that you know where the ground is. This is
 *      also just the project's own standing rule ("never treat baro
 *      altitude as an absolute short-term reference") applied to the
 *      one place that was quietly breaking it.
 *
 *   2. `valid` EXPIRES. Freezing bb and coasting on the baro is the
 *      designed behaviour above 2 m where the ToF cannot reach, so
 *      coasting must be allowed - but not forever, because "the ToF is
 *      out of range" and "the ToF is dead" look identical from here.
 *      3 s is roughly 15 missed fixes at the ~5 Hz this sensor DELIVERS
 *      at the 200 ms budget (it is POLLED at ~9 Hz; the two are not the
 *      same number and confusing them is what made the reject constant
 *      above wrong): far past any plausible dropout, far short of the
 *      ~10 s a diverging accel-driven estimate needs to invent metres.
 *      It was frame-based, which did stop it drifting with the BUDGET
 *      - and left it drifting with the LOOP RATE instead: 750 frames on
 *      the 416 Hz fusion task is 1.80 s, not 3 s. Accumulating dt is
 *      the only count that cannot be wrong about either.
 *
 * MEASURED, and the reason 3 s is not more generous: in the crash log
 * the estimate went from correct to 1.6 m below the floor in 1.6 s. */
#define VF_TOF_COAST_S         3.0f       /* seconds without a ToF fix  */

/* The largest step this filter will move the DATUM under a live
 * estimate.
 *
 * A re-datum says "the floor is somewhere else". Real floors step by a
 * kerb, a stair, a table, and 2 m covers all of them with room to
 * spare. Anything larger is not terrain - it is this filter being wrong
 * about where it is. The difference matters because a re-datum is not a
 * local correction: it leaves through datum_shift and lands on the
 * controller's altitude SETPOINT (alt_hold.c). In s89  that
 * path took 16 re-datums and 2387 m of commanded setpoint in a single
 * flight, because nothing here ever asked whether the number was
 * physically possible.
 *
 * What happens instead of the jump is the whole point: `valid` is
 * cleared. The step is NOT clamped to the bound - a clamped bad number
 * is still a bad number - and the old datum is not kept as if nothing
 * had happened either. The filter says it is LOST, and downstream an
 * invalid estimate already means alt-hold will not engage and hands the
 * throttle back (fc_tasks.c alt_want, and `valid` in the header). */
#define VF_REDATUM_MAX_M       2.0f

/* How far z may walk away from the ToF, while the ToF is right there
 * disagreeing with it, before the ESTIMATE is what is at fault.
 *
 * The two failures that reach the re-datum are indistinguishable in one
 * sample - filter and ToF differ by a lot - and opposite across a
 * window:
 *     the floor moved:      the ToF steps once, z stays where it was.
 *     the estimate ran off: the ToF holds still, z walks away from it.
 * s89 was the second. A -3.06 m/s^2 accz bias above 1400 us pushed every
 * innovation outside the 0.10 m gate, 82-88% of ToF samples were
 * rejected, and z left the building while the only sensor that can see
 * the ground went on reporting it correctly at 5 Hz.
 *
 * 1 m across one 2 s window: far above anything a hold does between
 * fixes (a healthy flight never opens a window at all - those samples
 * are accepted), far below the metres a diverging accel-driven estimate
 * covers in the same time. It is checked on every gated sample rather
 * than at the end of the window, because the crash log went 1.6 m wrong
 * in 1.6 s and two seconds of patience is not free. */
/* How still the ESTIMATE must have held during the disagreement window
 * for a re-datum to be credited to the GROUND rather than to the filter.
 *
 * A re-datum means "the ToF and I disagree and the ToF wins". WHY they
 * disagree decides what to do with the step:
 *
 *   the floor moved      -> z held still, the ToF stepped. The aircraft
 *                           really is a different height now, so the
 *                           SETPOINT must move with it or the hold will
 *                           climb over the table.
 *   the estimate drifted -> the ToF held still, z walked off it. The
 *                           aircraft has not moved. Correcting z is
 *                           right; moving the setpoint is a lie about
 *                           where the aircraft is and commands a climb
 *                           or a dive nothing asked for.
 *
 * The old code could not tell these apart and treated every re-datum as
 * terrain. MEASURED, flight s170, the 2.5 s before a
 * re-datum: the ToF moved 0.237 m and the estimate moved 0.960 m, four
 * times further. The floor had not moved. The re-datum pushed -1.04 m
 * into the altitude setpoint anyway, taking it to -0.74 m - BELOW THE
 * GROUND - and the hold then saw itself 2.2 m too high and cut up to
 * 236 us of throttle.
 *
 * 0.25 m: above the few centimetres a hovering aircraft moves inside one
 * window, below the 0.96 m the estimate walked in s170, and under the
 * smallest terrain step worth following. */
#define VF_TOF_TERRAIN_HOLD_M  0.25f

#define VF_TOF_RUNAWAY_M       1.0f

void vert_filter_init(vert_filter_t *f, float acc_sigma, float baro_sigma)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) f->P[i][j] = 0.0f;
    }
    f->z = f->vz = f->ba = f->bb = 0.0f;
    f->last_baro_m = 0.0f;

    /* Honest ignorance. Altitude and the baro bias are genuinely unknown
     * (the baro's offset is METRES - it reports height above a sea level
     * it inferred from today's weather). Climb rate is known to be zero
     * because we start on the ground, and the accel bias is known to be
     * small but nonzero. */
    f->P[0][0] = 100.0f;      /* altitude:   +-10 m                     */
    f->P[1][1] = VF_P0_VZ;    /* vz:         +-0.1 m/s, we are parked   */
    f->P[2][2] = VF_P0_BA;    /* accel bias: +-1 m/s^2                  */
    f->P[3][3] = 10000.0f;    /* baro bias:  +-100 m, no idea at all    */

    f->acc_sigma  = (acc_sigma  > 0.0f) ? acc_sigma  : VF_ACC_SIGMA_DEFAULT;
    f->baro_sigma = (baro_sigma > 0.0f) ? baro_sigma : VF_BARO_SIGMA_DEFAULT;

    f->have_baro = false;
    f->have_tof  = false;
    f->valid     = false;
    f->tof_reject_streak  = 0;
    f->baro_reject_streak = 0;
    f->dead_reckon_frames = 0;
    f->tof_coast_s        = 0.0f;
    f->tof_reject_s       = 0.0f;
    f->z_at_last_tof      = 0.0f;
    f->datum_shift = 0.0f;
    f->baro_reject_s = 0.0f;
    f->baro_inflation = 1.0f;
    f->baro_trusted = true;
    f->baro_distrust_s = 0.0f;
    f->baro_coherent_s = 0.0f;
    f->baro_sample_dt = 0.0f;
    f->baro_dt_acc = 0.0f;
    f->baro_reject_lo = 0.0f;
    f->baro_reject_hi = 0.0f;
}

void vert_filter_predict(vert_filter_t *f, float accz_ms2, float dt)
{
    if (dt <= 0.0f) return;

    const float a = accz_ms2 - f->ba;

    f->z  += f->vz * dt + 0.5f * a * dt * dt;
    f->vz += a * dt;
    /* ba and bb are constants in the model; their motion lives in Q. */

    /* F, the state Jacobian:
     *     [1  dt  -dt^2/2  0]
     *     [0   1  -dt      0]
     *     [0   0   1       0]
     *     [0   0   0       1]
     * Rows 2 and 3 are identity, so most of a generic 4x4 double product
     * would be multiplying by 1 and 0. Written out instead - this runs
     * at 416 Hz on an ESP32 next to Madgwick. */
    const float f02 = -0.5f * dt * dt;
    const float f12 = -dt;

    /* Step 1: FP = F*P. Only rows 0 and 1 change. */
    float FP[4][4];
    for (int j = 0; j < 4; j++) {
        FP[0][j] = f->P[0][j] + dt * f->P[1][j] + f02 * f->P[2][j];
        FP[1][j] = f->P[1][j] + f12 * f->P[2][j];
        FP[2][j] = f->P[2][j];
        FP[3][j] = f->P[3][j];
    }
    /* Step 2: P = FP*F'. (FP*F')[i][j] = sum_k FP[i][k]*F[j][k], so only
     * columns 0 and 1 change - by the same coefficients, F' being F
     * transposed. */
    for (int i = 0; i < 4; i++) {
        const float c0 = FP[i][0], c1 = FP[i][1];
        const float c2 = FP[i][2], c3 = FP[i][3];
        f->P[i][0] = c0 + c1 * dt + c2 * f02;
        f->P[i][1] = c1 + c2 * f12;
        f->P[i][2] = c2;
        f->P[i][3] = c3;
    }

    /* Q. The accelerometer noise enters through the same channel the
     * acceleration does, giving the classic continuous-white-noise
     * acceleration block on (z, vz). The two biases are independent
     * random walks. */
    const float sa2 = f->acc_sigma * f->acc_sigma;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    f->P[0][0] += sa2 * dt4 * 0.25f;
    f->P[0][1] += sa2 * dt3 * 0.5f;
    f->P[1][0] += sa2 * dt3 * 0.5f;
    f->P[1][1] += sa2 * dt2;
    f->P[2][2] += VF_BA_RW_SIGMA * VF_BA_RW_SIGMA * dt;
    f->P[3][3] += VF_BB_RW_SIGMA * VF_BB_RW_SIGMA * dt;

    f->dead_reckon_frames++;

    /* The coast clock runs on PREDICT and accumulates the dt it is
     * handed, so it means three SECONDS on the aircraft, in the host
     * replay, and at whatever rate the fusion task ends up running. See
     * VF_TOF_COAST_S: once the last accepted ToF fix is far enough
     * back, this estimate is dead reckoning against an accelerometer
     * with a known bias and a barometer with an unobservable one, and
     * nothing may close a loop around it. */
    if (f->tof_coast_s < VF_TOF_COAST_S) {
        f->tof_coast_s += dt;
        if (f->tof_coast_s >= VF_TOF_COAST_S) f->valid = false;
    }

    /* The disagreement clock, on the same PREDICT tick and for the same
     * reason - see VF_TOF_REJECT_S. Zero means no disagreement is open,
     * so it only runs once a sample has actually been gated out. It
     * SATURATES rather than growing: the re-datum needs a distance to
     * re-datum ONTO, so it can only fire on the next gated sample after
     * this expires, and that sample may be up to a budget away. */
    if (f->tof_reject_s > 0.0f && f->tof_reject_s < VF_TOF_REJECT_S) {
        f->tof_reject_s += dt;
    }

    /* Time since the last baro sample, for the distrust accrual. */
    f->baro_dt_acc += dt;

    /* Re-trust clock. An untrusted barometer is still evaluated in shadow
     * (see vert_filter_update_baro); every sample that both agrees with
     * its neighbours and would have passed the gate lets this run, and
     * incoherence zeroes it there. Recovery is deliberately slower than
     * loss: losing trust costs a reference the ToF can supply, regaining
     * it wrongly costs the aircraft. */
    if (!f->baro_trusted) {
        f->baro_coherent_s += dt;
        if (f->baro_coherent_s >= VF_BARO_RETRUST_S) {
            f->baro_trusted = true;
            f->baro_distrust_s = 0.0f;
            f->baro_coherent_s = 0.0f;
        }
    }

    /* Covariance inflation while a CONSISTENT baro disagreement stands -
     * the recovery path the filter did not have. On s152 that gap was
     * 619 rejections and -14.095 m of drift with the barometer alive
     * throughout. Runs on the predict tick, in SECONDS, so it means the
     * same thing at every baro divider and every fusion rate.
     * update_baro opens and clears the window; nothing else touches it. */
    if (f->baro_reject_s > 0.0f && f->baro_inflation < VF_BARO_INFLATE_MAX) {
        f->baro_reject_s += dt;
        float grow = 1.0f + VF_BARO_INFLATE_PER_S * dt;
        if (f->baro_inflation * grow > VF_BARO_INFLATE_MAX) {
            grow = VF_BARO_INFLATE_MAX / f->baro_inflation;
        }
        f->baro_inflation *= grow;
        /* Scaling the WHOLE matrix by a positive scalar keeps it
         * symmetric and positive-definite; a per-element nudge does not,
         * and a non-PD P is what turned a 0.15 m ground step into 1246 m
         * of divergence once already. */
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) f->P[i][j] *= grow;
        }
    }
}

/* Scalar Kalman update. H is sparse in both uses (baro is [1,0,0,1],
 * ToF is [1,0,0,0]) but kept general - the cost is 16 multiplies at
 * 50 Hz, and a hand-specialised version of each is exactly where a
 * transcription error would hide.
 *
 * gate_max, when > 0, is a hard innovation ceiling applied on top of the
 * sigma gate. See VF_TOF_GATE_MAX_M. */
static bool vf_update(vert_filter_t *f, const float H[4], float meas,
                      float R, float gate_sigma, float gate_max,
                      bool dry_run)
{
    float PHt[4];
    float S = R;
    for (int i = 0; i < 4; i++) {
        float s = 0.0f;
        for (int j = 0; j < 4; j++) s += f->P[i][j] * H[j];
        PHt[i] = s;
    }
    for (int i = 0; i < 4; i++) S += H[i] * PHt[i];
    if (S <= 0.0f) return false;

    const float x[4] = { f->z, f->vz, f->ba, f->bb };
    float pred = 0.0f;
    for (int i = 0; i < 4; i++) pred += H[i] * x[i];
    const float y = meas - pred;

    if (gate_sigma > 0.0f) {
        /* Chi-square form: d2 = y^2/S against the 1-DOF threshold. Same
         * inequality as |y| > k*sqrt(S), squared - no number changes -
         * but it is the form that generalises to the VECTOR innovations
         * of the 6-state and 15-state filters, where d2 = y' S^-1 y and
         * a per-axis sigma means nothing. One gate shape across all
         * three filters is one thing to validate. */
        const float d2 = (y * y) / S;
        if (d2 > gate_sigma * gate_sigma) return false;
        /* The ceiling stays, and stays SECOND: it is the backstop for
         * what chi-square cannot see, an inflated P whose gate has grown
         * wide enough to swallow real corruption.
         *
         * gate_max is whatever the CALLER passed. That matters: the baro
         * path scales it by sqrt(inflation) (see vert_filter_update_baro)
         * so a filter that has legitimately grown uncertain can still be
         * corrected, while the ToF path passes VF_TOF_GATE_MAX_M flat,
         * because nothing inflates there and a ground-referenced hold
         * must keep following the floor rather than the estimate. This
         * function must NOT reach into f->baro_inflation itself - it is
         * the one update both sensors share, and a rule applied here
         * would silently widen the ToF gate too. */
        if (gate_max > 0.0f && VF_FABSF(y) > gate_max) return false;
    }

    /* The sample PASSED and the caller only wanted the verdict. Used by
     * the untrusted-sensor shadow path, so "would this have been
     * accepted" is answered by the real gate rather than by a second
     * copy of it that could drift out of step. */
    if (dry_run) return true;

    float K[4];
    for (int i = 0; i < 4; i++) K[i] = PHt[i] / S;

    f->z  += K[0] * y;
    f->vz += K[1] * y;
    f->ba += K[2] * y;
    f->bb += K[3] * y;

    /* P = (I - K H) P. Joseph form would be more numerically polite, but
     * this is float on a 416 Hz loop; forcing symmetry afterwards fixes
     * the part that actually goes wrong in single precision. */
    float HP[4];
    for (int j = 0; j < 4; j++) {
        float s = 0.0f;
        for (int k = 0; k < 4; k++) s += H[k] * f->P[k][j];
        HP[j] = s;
    }
    float newP[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) newP[i][j] = f->P[i][j] - K[i] * HP[j];
    }
    for (int i = 0; i < 4; i++) {
        for (int j = i; j < 4; j++) {
            const float v = 0.5f * (newP[i][j] + newP[j][i]);
            f->P[i][j] = v;
            f->P[j][i] = v;
        }
    }
    return true;
}

static const float VF_H_BARO[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
static const float VF_H_TOF[4]  = { 1.0f, 0.0f, 0.0f, 0.0f };

bool vert_filter_update_baro(vert_filter_t *f, float baro_alt_m)
{
    /* NOTE: nothing in this function sets f->valid, and that is the
     * whole point - see VF_TOF_COAST_S. The barometer constrains
     * (z + bb), never z on its own, so on the first sample below it can
     * only ADOPT a bias consistent with whatever z already is. If z is
     * wrong, bb silently absorbs the error and the baro then confirms
     * the wrong altitude forever. That is not an absolute reference and
     * must not be advertised as one. */
    /* Recorded before any gate below can reject the sample: a rejected
     * reading is still the best statement the barometer has made. */
    f->last_baro_m = baro_alt_m;

    f->baro_sample_dt = f->baro_dt_acc;
    f->baro_dt_acc = 0.0f;

    if (!f->have_baro) {
        /* Define the bias outright rather than letting a 10000 m^2
         * covariance take one enormous first step. */
        f->bb = baro_alt_m - f->z;
        f->P[3][3] = f->baro_sigma * f->baro_sigma * 4.0f;
        f->have_baro = true;
        f->dead_reckon_frames = 0;
        return true;
    }

    const float R = f->baro_sigma * f->baro_sigma;
    /* The ceiling SCALES WITH THE INFLATION, and without that the
     * inflation is decorative. MEASURED before this existed: a filter 5 m
     * from a healthy, self-consistent barometer inflated to the full 400x
     * ceiling - sqrt(S) 670.850 m, chi-square gate 4025.101 m, so the
     * statistical test passed by three orders of magnitude - and the
     * sample was STILL refused, because 5 m > the flat 3 m cap. Permanent
     * lockout with no recovery path: exactly the bistability observed in the flight logs
     * described, reached by a different route than the one it named.
     *
     * The cap asks "is an innovation this large physically possible?",
     * and that has no fixed answer - it depends on how wrong the filter
     * admits it might be. sqrt() because the inflation is on P and the
     * cap is a distance: 400x on the covariance is 20x on sigma, so 3.0 m
     * becomes at most 60 m.
     *
     * It cannot become a licence to accept garbage, and the reason is
     * upstream: inflation only accumulates while the rejected samples
     * agree with EACH OTHER. A sensor whose samples disagree among
     * themselves resets the window and slams baro_inflation back to 1.0,
     * so its ceiling stays a flat 3.0 m - which is what s89's DPS310
     * (consecutive altitudes 655 m apart) would meet. */
    /* An UNTRUSTED barometer runs in shadow: the innovation is still
     * computed so the filter can watch it come back, but the update is
     * never applied. It earns its way back by the same self-consistency
     * test that condemned it. */
    if (!f->baro_trusted) {
        const bool would = vf_update(f, VF_H_BARO, baro_alt_m, R,
                                     VF_BARO_GATE_SIGMA,
                                     VF_BARO_GATE_MAX_M, true);
        float slo = f->baro_reject_lo, shi = f->baro_reject_hi;
        if (baro_alt_m < slo) slo = baro_alt_m;
        if (baro_alt_m > shi) shi = baro_alt_m;
        if (would && (shi - slo) <= VF_BARO_RESYNC_SPREAD_M) {
            f->baro_reject_lo = slo;
            f->baro_reject_hi = shi;
        } else {
            f->baro_reject_lo = f->baro_reject_hi = baro_alt_m;
            f->baro_coherent_s = 0.0f;
        }
        return false;
    }

    const float baro_gate_max = VF_BARO_GATE_MAX_M
                              * VF_SQRTF(f->baro_inflation);
    if (vf_update(f, VF_H_BARO, baro_alt_m, R,
                  VF_BARO_GATE_SIGMA, baro_gate_max, false)) {
        f->baro_reject_streak = 0;
        f->dead_reckon_frames = 0;
        /* Disagreement over: close the window, stop inflating. P is NOT
         * deflated - the update just shrank it by exactly the amount the
         * sample's information warrants, and winding that back by hand
         * would throw away the filter's own arithmetic. */
        f->baro_reject_s = 0.0f;
        f->baro_inflation = 1.0f;
        /* An accepted sample is the sensor behaving. Decay the charge so
         * isolated resets spread over minutes never accumulate into a
         * distrust - only a sustained run of incoherence does. */
        f->baro_distrust_s -= VF_BARO_DISTRUST_DECAY * f->baro_sample_dt;
        if (f->baro_distrust_s < 0.0f) f->baro_distrust_s = 0.0f;
        return true;
    }

    /* Track what the rejected samples say about each other, not just how
     * many there have been.
     *
     * The window RESTARTS the moment a sample lands outside it, so what
     * the streak counts is a run of samples that agree - not every
     * rejection since the sensor first misbehaved. Without the restart a
     * baro that spat garbage once could never re-sync again, however
     * healthy it went on to be, because lo/hi would remember the
     * garbage forever. */
    float lo = f->baro_reject_lo, hi = f->baro_reject_hi;
    if (baro_alt_m < lo) lo = baro_alt_m;
    if (baro_alt_m > hi) hi = baro_alt_m;
    if (f->baro_reject_streak == 0 || (hi - lo) > VF_BARO_RESYNC_SPREAD_M) {
        f->baro_reject_lo = f->baro_reject_hi = baro_alt_m;
        f->baro_reject_streak = 1;
        /* The samples stopped agreeing with EACH OTHER, so this is not a
         * drifted filter - it is a lying sensor. Slam the inflation shut
         * and start over. This is what keeps s89's +-327 m garbage from
         * ever opening the gate: it fails the spread test on nearly
         * every sample and can never accumulate the seconds inflation
         * needs. */
        f->baro_reject_s = VF_BARO_REJECT_OPEN_S;
        /* The sensor just contradicted its OWN recent rejections. A
         * healthy baro arguing with a drifted filter never does this -
         * its rejections cluster. Charge it in SECONDS of the measured
         * interval since the last sample, so the accrual means the same
         * thing at every baro divider. */
        f->baro_distrust_s += f->baro_sample_dt;
        if (f->baro_distrust_s >= VF_BARO_DISTRUST_S) {
            f->baro_trusted = false;
            f->baro_coherent_s = 0.0f;
            /* Losing trust must also CLOSE the inflation window. An
             * untrusted sensor is not applied, so it has no business
             * driving the covariance either - and left open, predict()
             * keeps inflating P the whole time the sensor is ignored, so
             * the gate would be wide open the moment trust returned.
             * Undo the inflation, do not merely stop it. */
            if (f->baro_inflation > 1.0f) {
                const float back = 1.0f / f->baro_inflation;
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) f->P[i][j] *= back;
                }
            }
            f->baro_inflation = 1.0f;
            f->baro_reject_s = 0.0f;
        }
        if (f->baro_inflation > 1.0f) {
            const float shrink = 1.0f / f->baro_inflation;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) f->P[i][j] *= shrink;
            }
            f->baro_inflation = 1.0f;
        }
    } else {
        f->baro_reject_lo = lo;
        f->baro_reject_hi = hi;
        f->baro_reject_streak++;
        /* Consistent disagreement: open the window if it is not already
         * open. predict() does the timing and the growing. */
        if (f->baro_reject_s <= 0.0f) f->baro_reject_s = VF_BARO_REJECT_OPEN_S;
    }

    /* WHAT USED TO BE HERE: `if (baro_reject_streak > 50) accept it
     * UNGATED`. Both halves were wrong.
     *
     * It counted SAMPLES, so "a full second" was whatever the live baro
     * divider made it - the last constant in this filter still measuring
     * time in the wrong unit.
     *
     * And it forced the sample in with BOTH gates disabled, which is the
     * project's own warning written as code: persistence is not
     * evidence, and an ungated accept hands the state to whichever
     * sensor is most stubbornly wrong.
     *
     * The inflation window above replaces it and is strictly stronger: a
     * consistently-disagreeing baro gets back in through the ORDINARY
     * chi-square test, once the filter has widened its own gate to match
     * the uncertainty it demonstrably has - and it must still clear the
     * absolute ceiling on the way. Nothing is accepted ungated now. */
    return false;
}

/* Move the ground datum to whatever the ToF is now measuring.
 *
 * A CHANGE OF ORIGIN, not new information about where the aircraft is,
 * and getting that distinction wrong breaks the filter two separate
 * ways - the replay caught both:
 *
 *   1. The baro bias moves the OPPOSITE way to the altitude. The model
 *      is `baro = z + bb`, and the barometer did not change when the
 *      floor did, so z += dz forces bb -= dz. Adding dz to both moved
 *      the predicted baro reading by 2*dz.
 *
 *   2. The covariance must be left ALONE. Slamming P[0][0] down to the
 *      ToF's variance while every off-diagonal term stayed sized for the
 *      old uncertainty made P non-positive-definite, and a 0.15 m ground
 *      step diverged to 1246 m in seconds. A translation of the state
 *      does not change its covariance; the ToF's precision enters
 *      through the ordinary update below, whose innovation is now
 *      exactly zero, so it shrinks P correctly and moves the state not
 *      at all. */
/* lost_reacquire: this re-datum is the filter coming back from having
 * ADMITTED IT WAS LOST, as opposed to the ground having moved under a
 * healthy estimate. The two need different treatment and conflating them
 * is what kept s179 and s180 sawtoothing.
 *
 * When the ground moves, the aircraft's real velocity has not changed -
 * a stepped floor says nothing about vz - so vz must be left alone.
 * When the ESTIMATE walked off a steady ToF, the velocity that walked it
 * is exactly as wrong as the position, and carrying it through the
 * recovery guarantees the next runaway. Measured: s180 sat at
 * vz = +32.18 m/s across all 51 of its recoveries, so z was snapped home
 * and immediately flung out again, 1.68 times a second, for the whole
 * flight. */
static void vf_redatum(vert_filter_t *f, float agl, float sigma,
                       bool emit_shift, bool lost_reacquire)
{
    const float dz = agl - f->z;
    f->z = agl;
    if (f->have_baro) {
        if (lost_reacquire) {
            /* RE-DERIVE bb, do not adjust it by dz.
             *
             * `bb -= dz` is the right rule for a GROUND-MOVED re-datum,
             * and only there: it preserves `baro = z + bb`, which is
             * worth preserving exactly when the state already agreed
             * with the barometer. On a lost re-acquire it did not - the
             * baro is being gated out PRECISELY BECAUSE z + bb stopped
             * matching the reading - so adjusting by dz carries the
             * whole excursion into bb instead of ending it.
             *
             * MEASURED on s180 before this: bb moved +2.328 m across the
             * flight while the barometer itself moved +0.060 m, so the
             * bias was absorbing the estimator's error and then
             * confirming it. Setting bb from the reading restores
             * `baro = z + bb` in one step, and is the same rule the
             * first-ever baro sample uses above. */
            f->bb = f->last_baro_m - f->z;
        } else {
            f->bb -= dz;
        }
    }

    if (lost_reacquire) {
        /* The velocity state, and everything that claimed to know
         * something about it.
         *
         * The cross terms go too. A fresh P[1][1] beside a stale
         * P[0][1] tells the filter it has a precise, strongly
         * correlated relationship between an altitude it just forced
         * and a velocity it just erased, and covariance nonsense of
         * that shape is how the 0.15 m step became 1246 m. */
        f->vz = 0.0f;
        for (int i = 0; i < 4; i++) { f->P[1][i] = 0.0f; f->P[i][1] = 0.0f; }
        f->P[1][1] = VF_P0_VZ;

        /* Re-open the accel bias.
         *
         * ba is what GENERATES the phantom velocity - s180 needed about
         * 1.2 m/s^2 of unmodelled bias to reach +31.6 m in 26 s - and
         * the one thing that could correct it is a real ToF innovation.
         * A confident P[2][2] means the ToF samples that follow get
         * absorbed by z instead of moving ba, so the filter re-learns
         * nothing and does this again. Widening it says what is
         * actually true after a runaway: the bias estimate is the least
         * trustworthy number in the filter. */
        if (f->P[2][2] < VF_P0_BA) f->P[2][2] = VF_P0_BA;
    }
    /* emit_shift is false when re-acquiring from a LOST state.
     *
     * datum_shift exists for ONE purpose: keeping a controller's
     * altitude setpoint aligned when the GROUND moves under a live
     * hold. If `valid` was false there is no live hold - alt-hold has
     * already been handed back - so there is no setpoint to align, and
     * emitting the step pushes a number into a controller that is not
     * using it and will re-engage from scratch anyway.
     *
     * Not a detail: emitting it on a lost re-acquire put +103.3 m into
     * datum_shift in the runaway scenario, the same path that shipped
     * 2387 m of commanded setpoint in s89. A recovery that resurrects
     * the worst bug in the file is not a recovery. */
    if (emit_shift) f->datum_shift += dz;

    /* This update has an innovation of exactly zero - z was just forced
     * to agl - so it moves no state and only shapes P. It looks
     * pointless on the lost path and it is not: it is what makes the
     * covariance inflation above USEFUL.
     *
     * Measured, because the reasoning went the wrong way first. Skipping
     * it on a lost re-acquire, on the argument that shrinking P right
     * after admitting you are lost is overconfidence, was TRIED: the
     * s89 scenario (constant -3.06 m/s^2 with the ToF pinned at 0.15 m)
     * then left ba at -0.752 after 10 s and z wandering 2.25 m. Keeping
     * it, ba converges to -3.057 against the true -3.06 and z tracks the
     * ToF to 0.000 m from t = 4 s.
     *
     * The reason is the CROSS terms. The inflation says "ba is unknown";
     * this update is what re-establishes how ba and z are related given
     * the ToF's precision, and without that relationship the innovations
     * that follow are absorbed entirely by z and ba never moves. That is
     * the mechanism behind the project rule "a state cannot learn what its process
     * noise forbids" - and this is the first time ba has been able to
     * learn a 3 m/s^2 bias at all. */
    vf_update(f, VF_H_TOF, agl, sigma * sigma, 0.0f, 0.0f, false);
    f->dead_reckon_frames = 0;
    f->tof_coast_s        = 0.0f;
    f->tof_reject_s       = 0.0f;
    /* Datum and estimate are the same number again, so this is where the
     * next disagreement window will be measured from. */
    f->z_at_last_tof      = f->z;
    f->valid              = true;
}

bool vert_filter_update_tof(vert_filter_t *f, float tof_m, float tilt_cos)
{
    if (tof_m < VF_TOF_MIN_M || tof_m > VF_TOF_MAX_M) return false;
    if (tilt_cos < VF_TOF_MIN_TILT_COS) return false;

    const float agl = tof_m * tilt_cos;

    float sigma = VF_TOF_SIGMA_FRAC * agl;
    if (sigma < VF_TOF_SIGMA_FLOOR) sigma = VF_TOF_SIGMA_FLOOR;

    /* ---- LOST: the ToF wins, immediately ----
     *
     * THE PRINCIPLE, and it should have been here from the start: a
     * VL53L0X does not lie. Its failure mode is SILENCE - a range under
     * ~2 m or the 65535 NO_TARGET sentinel, nothing in between. That is
     * the opposite of a dying barometer, which returns confident
     * nonsense, and the two must not share the same "the filter might
     * be right" logic.
     *
     * Once `valid` is false the filter has ALREADY ADMITTED it does not
     * know where the ground is, and at that moment a ToF reading is by
     * construction more trustworthy than the dead reckoning that
     * replaced it. Nothing is left to protect - alt-hold has been handed
     * back, there is no setpoint to yank - so waiting is pure downside.
     *
     * MEASURED, s171, the flight that forced this: z ran
     * from -1.1 m to -5.97 m over 8.5 s while the ToF sat at 0.11-0.33 m
     * and every sample was rejected. The cause was a LIVELOCK between
     * two individually sensible rules:
     *
     *   the re-datum needs VF_TOF_REJECT_S (2.0 s) of steady disagreement
     *   the runaway detector RESETS that clock whenever z walks
     *   VF_TOF_RUNAWAY_M (1.0 m) from the window origin
     *
     * z drifted at 0.6 m/s, covering 1 m every 1.7 s, and reset the
     * clock before it could reach 2 s. Over that window the runaway
     * detector fired 48 times and the re-datum clock never exceeded
     * 0.000 s. The safety mechanism starved the recovery mechanism.
     *
     * This branch breaks the loop by removing the wait in the one state
     * where the wait was never buying anything. Replaying s171 through
     * it, the worst |alt_est - tof| falls from 6.15 m to 0.79 m. */
    if (f->have_tof && !f->valid) {
        vf_redatum(f, agl, sigma, false, true);   /* LOST: reset vz too */
        f->tof_reject_streak = 0;
        return true;
    }

    if (!f->have_tof) {
        /* The first fix DEFINES the datum and is exempt from
         * VF_REDATUM_MAX_M by construction, explicitly and not by luck:
         * there is no previous datum for it to be a step away from, and
         * its dz is only the distance between an arbitrary z = 0 start
         * and the floor. Bounding it would mean booting with a ground
         * reference the filter already knows to be wrong. */
        vf_redatum(f, agl, sigma, true, false); /* first fix defines the datum */
        f->have_tof = true;
        f->valid = true;
        return true;
    }

    if (vf_update(f, VF_H_TOF, agl, sigma * sigma,
                  VF_TOF_GATE_SIGMA, VF_TOF_GATE_MAX_M, false)) {
        f->tof_reject_streak  = 0;
        f->tof_reject_s       = 0.0f;
        f->dead_reckon_frames = 0;
        f->tof_coast_s        = 0.0f;
        f->z_at_last_tof      = f->z;
        f->valid = true;
        return true;
    }

    f->tof_reject_streak++;
    /* Start the clock on the FIRST gated sample; predict() runs it from
     * there. A count of samples cannot time this - see the constant.
     * The window's ORIGIN is set with its age: z as it stands now is
     * what the next two seconds of drift are measured against. */
    if (f->tof_reject_s <= 0.0f) {
        f->tof_reject_s  = VF_TOF_REJECT_OPEN_S;
        f->z_at_last_tof = f->z;
    }

    /* Did the ESTIMATE run away, rather than the floor moving? Asked on
     * every gated sample and not just when the clock expires, because
     * waiting out the full two seconds to notice hands a diverging
     * estimate the 1.6 s that put an aircraft through a ceiling. See
     * VF_TOF_RUNAWAY_M.
     *
     * Nothing is re-datumed while this holds. The window RESTARTS
     * instead, so the question stays "how far has z walked in the last
     * couple of seconds" and never becomes "since some fix minutes
     * ago": a distance that can only grow is a verdict the filter could
     * never argue its way out of, and an estimator with no way back is
     * not a safety feature. */
    if (VF_FABSF(f->z - f->z_at_last_tof) > VF_TOF_RUNAWAY_M) {
        f->valid         = false;
        f->tof_reject_s  = VF_TOF_REJECT_OPEN_S;
        f->z_at_last_tof = f->z;
        return false;
    }

    if (f->tof_reject_s >= VF_TOF_REJECT_S) {
        /* The ground has kept saying it moved for two seconds, and the
         * estimate held still throughout while it said so. Believe the
         * surface, not the old datum. (vf_redatum clears the clock.)
         *
         * Unless the step is larger than any floor. Then the honest
         * answer is that we do not know where we are, so say it -
         * clearing `valid` is the admission, and it costs the controller
         * the hold rather than costing it a 2387 m setpoint.
         *
         * The `valid` half of the test is what keeps this ESCAPABLE, and
         * it is a statement about the controller, not about the filter.
         * The bound exists because a re-datum walks straight into the
         * altitude setpoint, so it only has to hold while something can
         * still be closing a loop on this estimate. Once `valid` is
         * false, alt-hold has already been handed back and there is no
         * setpoint left to protect - a big step is then just this filter
         * re-acquiring a datum it has admitted losing, exactly like the
         * first fix above. Refusing it forever would strand the
         * estimator on a ground reference it has already disowned, which
         * is the one failure worse than the jump.
         *
         * So a large step costs one extra window, never permission: the
         * first one is refused and takes `valid` with it, and the ToF
         * has to still be saying the same thing two seconds later, with
         * z holding still throughout, before the datum actually moves. */
        if (f->valid && VF_FABSF(agl - f->z) > VF_REDATUM_MAX_M) {
            f->valid         = false;
            f->tof_reject_s  = VF_TOF_REJECT_OPEN_S;
            f->z_at_last_tof = f->z;
            return false;
        }
        /* WHO MOVED? See VF_TOF_TERRAIN_HOLD_M. The step only reaches
         * the controller's setpoint when the estimate held still and the
         * ground is what changed. */
        const bool ground_moved =
            VF_FABSF(f->z - f->z_at_last_tof) < VF_TOF_TERRAIN_HOLD_M;
        vf_redatum(f, agl, sigma, ground_moved, false);
        f->tof_reject_streak = 0;
    }
    return false;
}

void vert_filter_tof_lost(vert_filter_t *f)
{
    f->tof_reject_streak = 0;
    f->tof_reject_s      = 0.0f;
    /* The sensor is SAYING it cannot see the floor, so there is no
     * disagreement to measure and nothing to measure it from. The next
     * window starts wherever z is when the ToF comes back: drift piled
     * up across a dropout the sensor announced is not evidence of a
     * runaway, because the aircraft may simply have climbed out of range
     * and flown back down. A SILENT death is the opposite case - nothing
     * calls this, samples keep arriving with a stale value, and the
     * window keeps running. That asymmetry is the whole discrimination
     * and it is deliberate. */
    f->z_at_last_tof     = f->z;
}

float vert_filter_take_datum_shift(vert_filter_t *f)
{
    const float d = f->datum_shift;
    f->datum_shift = 0.0f;
    return d;
}
