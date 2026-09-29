#ifndef VERT_FILTER_H
#define VERT_FILTER_H

/* P7 stage 1 - vertical state estimator (altitude, climb rate, and the
 * two biases that make them believable).
 *
 * Lives in components/common, and depends on NOTHING: no ESP-IDF, no
 * FreeRTOS, no math.h, no allocation, no globals. That is deliberate.
 * The STM32F722 core split is a planned direction, and an estimator that
 * only compiles against one vendor's SDK has to be rewritten - which
 * means re-validated - the day the port happens. It also means this file
 * can be compiled and RUN on the host by tools/vert_ctest.py, which is
 * what proves it matches the Python design in tools/vert_filter.py
 * instead of merely looking like it - and by
 * tools/vert_gating_test.py, which proves the same thing on the gating
 * branches no log reaches.
 *
 * The design, the derivation and the reason there are four states and
 * not three all live in tools/vert_filter.py. Read that first; this is
 * the transliteration, and any disagreement between the two is a bug
 * HERE.
 *
 * STATE
 *   z   altitude above the ToF ground datum, m, POSITIVE UP
 *   vz  climb rate, m/s, POSITIVE UP
 *   ba  accelerometer Z bias, m/s^2  (Log A measured +0.194 = 2% of 1 g)
 *   bb  baro bias, m  (baro_reading - true altitude; eats both the
 *       arbitrary sea-level offset and the 18 cm/100 s drift)
 *
 * USAGE, per fusion cycle:
 *   vert_filter_predict(f, accz_ms2, dt);              // every cycle
 *   if (new baro sample)  vert_filter_update_baro(f, alt_m);
 *   if (new tof sample)   vert_filter_update_tof(f, m, tilt_cos);
 *   else if (tof says NO_TARGET) vert_filter_tof_lost(f);
 *
 * Feed each MEASUREMENT ONCE. Both sensors are read far slower than the
 * fusion loop and their values are cached, so re-applying a cached
 * sample every cycle would tell the filter it had ~80 independent
 * observations of the same thing and collapse its covariance to zero.
 * The caller detects fresh samples with the driver's sample counters.
 *
 * ~80 because the ToF DELIVERS at 1000/tof_budget_ms Hz (~5 Hz at the
 * 200 ms this airframe runs), not at the ~9 Hz it is POLLED at, and
 * predict() is called from the fusion task - 416 Hz, paced by the
 * LSM6DSOX INT1 line (fusion.c FUSION_RATE_HZ), NOT the 250 Hz flight
 * loop in fc_tasks.c.
 *
 * Anything in this filter that has to mean a fixed amount of TIME
 * ACCUMULATES dt. Not samples, whose rate is a live console param; and
 * not frames either, which is the same mistake one level up - every
 * frame count in here was written "at 250 Hz" and was therefore 1.67x
 * short of the duration printed beside it. A constant that counts
 * anything but time lies about time. */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* --- state --- */
    float z, vz, ba, bb;
    float P[4][4];

    /* --- tuning, live-settable so params can drive it --- */
    float acc_sigma;    /* process noise, m/s^2. Default 2.0 (flight 1.77-
                           2.83; Log A's 0.294 was at IDLE). */
    float baro_sigma;   /* baro measurement noise, m. Log A: 0.073.    */

    /* --- bookkeeping --- */
    bool     have_baro;
    bool     have_tof;
    /* Safe to close a control loop around this estimate RIGHT NOW.
     *
     * Set ONLY by an accepted ToF fix and cleared once the coast clock
     * runs out - the barometer never sets it, because it constrains
     * (z + bb) and not z. Read this every cycle, not once: it goes false
     * in flight the moment the ground reference goes stale. */
    bool     valid;
    /* Consecutive GATED ToF samples. A diagnostic only (tools/
     * vert_replay.py counts gated samples with it); it used to drive the
     * re-datum and no longer does, because a sample count is a different
     * amount of TIME at every tof_budget_ms. See VF_TOF_REJECT_S. */
    uint16_t tof_reject_streak;
    uint16_t baro_reject_streak;
    /* Predict calls since the last accepted fix. A DIAGNOSTIC, and the
     * only reason it is allowed to stay a count while the two clocks
     * below are not: nothing thresholds it, so it never has to mean an
     * amount of time. Give it a threshold and it must become seconds. */
    uint32_t dead_reckon_frames;
    float    tof_coast_s;        /* SECONDS since the last ToF fix      */
    float    tof_reject_s;       /* SECONDS since the ToF started
                                  * disagreeing; 0 = none open. Drives
                                  * the re-datum.                       */
    /* z as of the last moment the ToF and this estimate were tied to
     * each other: an accepted fix, a re-datum, a NO_TARGET, or the
     * opening of the current disagreement window. It is the ORIGIN of
     * the window whose AGE is tof_reject_s, and every place that starts
     * one starts the other.
     *
     * What it is for: while the ToF is alive and every sample is being
     * gated out, it is the only ground truth left, and how far z has
     * walked from it is what separates "the floor moved" (z still, ToF
     * stepped) from "the estimate ran away" (ToF still, z walking).
     * See VF_TOF_RUNAWAY_M. */
    float    z_at_last_tof;
    float    datum_shift;        /* see vert_filter_take_datum_shift    */
    /* Range of the baro altitudes rejected during the CURRENT reject
     * streak. Meaningless when baro_reject_streak is 0.
     *
     * The re-sync below used to fire on PERSISTENCE alone - "disagreed
     * for a full second, must be telling the truth". That is exactly
     * backwards for a sensor whose failure mode is confident garbage,
     * and s89  is the log that proves it: a DPS310 dying
     * under motor load returned altitudes flipping +-327 m and the
     * estimator was dragged along with them. A stuck-but-HEALTHY baro
     * and a broken one are both persistent; only the healthy one is
     * self-CONSISTENT. So the re-sync now asks for agreement across the
     * streak, not merely stubbornness. */
    float    baro_reject_lo;
    float    baro_reject_hi;
    /* SECONDS the current CONSISTENT baro disagreement has stood, and the
     * factor P has been inflated by because of it. Both are 0/1.0 when no
     * disagreement is open.
     *
     * This is the filter's recovery path. Once the estimate drifts past
     * the gate every honest baro sample looks like an outlier and there
     * is no way back - measured on s152 as 619 rejections and -14.095 m
     * of drift with the barometer alive and reading the whole time.
     * Inflating P grows S, which widens the gate, which readmits the
     * sensor through the ORDINARY test rather than an exception to it.
     *
     * Seconds, not samples: the baro's delivery rate is a live divider,
     * so a sample count would mean a different amount of time at each
     * one. This was the last constant in the filter still getting that
     * wrong (the old 50-sample force-accept). */
    float    baro_reject_s;
    float    baro_inflation;
    /* SENSOR TRUST, which is a different question from whether any one
     * sample is plausible.
     *
     * A barometer scattering readings across tens of metres has ~97% of
     * them correctly rejected - and the few that land near the prediction
     * pass the gate HONESTLY and kick vz through a K[1] that grew large
     * while the filter dead-reckoned between them. Measured on a
     * STATIONARY aircraft: z -162.6 m at vz -40.1 m/s, on the code as it
     * shipped before any of this. Every gate worked; the estimate ran
     * away anyway.
     *
     * `baro_trusted` false means nothing this sensor says reaches the
     * state at all. Its innovations are still computed (the shadow path
     * in update_baro) so recovery can be seen, but they are not applied.
     * Trust is charged by INCOHERENCE - a rejected sample disagreeing
     * with the other rejected samples - which is the one thing a healthy
     * baro arguing with a drifted filter never does. */
    bool     baro_trusted;
    float    baro_distrust_s;
    float    baro_coherent_s;
    /* Measured seconds between baro samples. The delivery rate is a live
     * divider, so charging distrust per SAMPLE would mean a different
     * amount of time at every setting - the mistake this filter has now
     * made three times. */
    float    baro_sample_dt;
    float    baro_dt_acc;
    /* The latest RAW barometer reading, stored whether or not the sample
     * was accepted. A lost re-acquire re-derives bb from it instead of
     * adjusting bb by dz - see vf_redatum. Kept even for a gated sample
     * on purpose: the case this exists for is the one where the baro is
     * fine and the ESTIMATE is what walked off, which is exactly when
     * the baro is being gated. */
    float    last_baro_m;
} vert_filter_t;

/* acc_sigma / baro_sigma: pass 0 for the measured Log A defaults. */
void vert_filter_init(vert_filter_t *f, float acc_sigma, float baro_sigma);

/* accz_ms2 is nav_state.accz_ms2: earth-frame vertical acceleration with
 * gravity already removed, POSITIVE UP. Call only on a good IMU read -
 * accz holds its last value on a failed one, and integrating a stale
 * acceleration is how dead reckoning invents climb. */
void vert_filter_predict(vert_filter_t *f, float accz_ms2, float dt);

/* baro_alt_m: the raw pressure-derived altitude, offset and all. The bb
 * state exists precisely so the caller does not have to reference it to
 * anything. Returns true if the sample was accepted. */
bool vert_filter_update_baro(vert_filter_t *f, float baro_alt_m);

/* tof_m: VL53L0X slant range in metres, already screened for the 65535 mm
 * NO_TARGET sentinel. NOT screened by device range status - see
 * vl53l0x_cached_mm() for why that gate was removed. The
 * gating that matters happens here instead, in TOF_MIN_M/TOF_MAX_M and
 * the filter's own innovation handling.
 *
 * tilt_cos: cosine of the tilt from vertical. The ToF looks along body
 * -Z, so a tilted aircraft measures a longer slant range than its true
 * height. Pass the quaternion's r22 = 1 - 2*(q1*q1 + q2*q2), which
 * fusion.c already computes for the earth-frame accel - it IS the
 * cosine, exactly, with no trig anywhere in this file.
 *
 * Returns true if the sample was accepted into the state. */
bool vert_filter_update_tof(vert_filter_t *f, float tof_m, float tilt_cos);

/* Call when the ToF reports NO_TARGET or goes out of range. The baro
 * bias FREEZES at its learned value, and that frozen number is what
 * makes the handover to baro-only seamless (measured on Log A with a
 * synthetic cut: 9 mm step). */
void vert_filter_tof_lost(vert_filter_t *f);

/* Metres of altitude added by a DATUM CHANGE rather than by the aircraft
 * moving; read-and-cleared. The controller MUST add this to its altitude
 * setpoint every cycle.
 *
 * Fly over a table at 1.5 m: the ToF says the ground rose 0.8 m, the
 * innovation gate holds that off for two seconds, then accepts that the
 * surface really did move and re-datums. Altitude jumps by -0.8 m with
 * the aircraft perfectly still. A controller that did not shift its
 * setpoint by the same amount would see 0.8 m of error and CLIMB OVER
 * THE TABLE. */
float vert_filter_take_datum_shift(vert_filter_t *f);

#endif /* VERT_FILTER_H */
