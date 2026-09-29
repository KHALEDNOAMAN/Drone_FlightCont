#ifndef NAV_STATE_H
#define NAV_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* nav_state.fix_type. The UBX path (default) stores NAV-PVT fixType as-is:
 * 0 no fix, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS+DR, 5 time only. The NMEA
 * path (TBSM10_USE_NMEA) stores GSA mode: 1 no fix, 2 2D, 3 3D. Only
 * GPS_FIX_3D is compared against (>= 3D), which means the same in both. */
#define GPS_FIX_NONE  1      /* NMEA GSA "no fix"; UBX reports 0 */
#define GPS_FIX_2D    2
#define GPS_FIX_3D    3

/* Pre-arm blocker bits (nav_state.arm_blocked). 0 = clear to arm.
 * Each bit names the crash it prevents. */
#define PREARM_GYRO_MOVING    (1u << 0)  /* craft moving - attitude/cal untrustworthy */
#define PREARM_NOT_LEVEL      (1u << 1)  /* angle mode would snap toward level on arm */
#define PREARM_STICKS         (1u << 2)  /* roll/pitch/yaw stick deflected - instant flip */
#define PREARM_THROTTLE_HIGH  (1u << 3)  /* props would spin the moment arming completes */
#define PREARM_FUSION_WARMUP  (1u << 4)  /* estimator still converging (high-beta warmup) */
/* A sensor the flight code depends on is not answering. Added
 * nothing checked sensor health at arm, so an aircraft could
 * lift off with a dead sensor and only find out in the air. Finding it on
 * the ground is free; finding it at 1.5 m is not. */
#define PREARM_SENSOR_DEAD    (1u << 5)
/* The alt-hold switch is UP but the vertical estimator has no absolute
 * reference. THE CRASH ARMED IN EXACTLY THIS STATE: CH6 was
 * up before takeoff and the ToF had been dead for two flights. This bit
 * is the check that refuses that flight on the pad. */
#define PREARM_ALTHOLD_UNSAFE (1u << 6)
/* A saved params blob existed and was REFUSED, so the aircraft is on
 * compiled-in defaults: m*_scale 1.0, no level trim, notch back to its
 * default. It will fly, badly, and the only tell was a boot log line.
 *
 * Added with the names digest. The digest stops the wrong
 * values being loaded; this stops the RECOVERY from being silent too.
 * Refusing a blob and flying anyway would be the same class of failure
 * as the one being fixed - a real problem reported only where nobody
 * looks. Clears on SAVE, which is exactly the action that resolves it:
 * check the values, then persist them. */
#define PREARM_PARAMS_DEFAULTED (1u << 7)
/* The blackbox cannot record: no SD card, an unmarked card, or a write
 * failed. Added with the move to SD. the author's rule is no flight
 * without a log - s208 and s11 were both diagnosed ONLY from the blackbox -
 * so a missing recorder refuses the flight rather than degrading silently.
 * BBINFO says which. Needed a 9th bit, so arm_blocked is uint16_t now. */
#define PREARM_BLACKBOX_NOT_READY (1u << 8)
/* The arm switch has not been seen OFF since the last arm or the last
 * refused arm attempt. Cycle it OFF, then ON. Added (review
 * item 1): arming used to need only "CH5 high + no other blocker", so a
 * switch left ON armed the aircraft BY ITSELF - at boot the moment warmup
 * finished, when the SD card came ready, after an RC dropout, and after a
 * crash-disarm as soon as someone set the aircraft upright and still.
 * Now only an OFF -> ON the pilot makes while nothing else blocks arms it. */
#define PREARM_ARM_SWITCH (1u << 9)

/* Per-cycle sensor health (nav_state.sensor_health).
 * Exists because a FAILING sensor and a genuinely CONSTANT one look
 * identical in the stream: on a failed read the driver leaves the caller's
 * sample struct untouched, so fusion publishes the last good value forever.
 * Diagnosing that from the data alone means decoding raw counts by hand.
 * These bits say it directly - read them BEFORE believing any sensor value. */
#define SENSOR_IMU_OK    (1u << 0)  /* IMU read returned ESP_OK this cycle */
#define SENSOR_BARO_OK   (1u << 1)  /* baro read OK on the last divider tick */
#define SENSOR_MAG_OK    (1u << 2)  /* mag read OK on the last divider tick */
#define SENSOR_MAG_GATED (1u << 3)  /* mag passed the magnitude gate -> 9-DOF */
/* The SECOND barometer (MS5611 at 0x76), added. It is a CONTROL
 * for the DPS310 investigation, not a flight sensor: nothing reads it, no
 * filter consumes it, and it is deliberately absent from
 * PREARM_SENSORS_REQUIRED below so a missing or failing control part can
 * never refuse a flight. Its whole job is to sit in the same log as the
 * DPS310 and answer one question - is the DPS310 the culprit or the
 * victim. See components/ms5611/ms5611.h. */
#define SENSOR_BARO2_OK  (1u << 4)

/* Sensors that must be answering before the props may spin
 * (PREARM_SENSOR_DEAD). Lives here, not in config.h, because it is built
 * out of the bits directly above it.
 *
 * Deliberately NOT including SENSOR_MAG_GATED: that bit says "the field
 * looks like Earth's right now", which is a property of where the
 * aircraft is parked - near a car, a desk leg, a speaker - not of
 * whether the chip is alive. Gating arming on it would refuse to arm on
 * perfectly good hardware.
 *
 * KNOWN INTERACTION: after a WARM reset the DPS310 can come up without its
 * coefficients loaded. Until init read them anyway and garbage
 * passed as healthy; since then dps310_init refuses (COEF_RDY/SENSOR_RDY
 * check) and fusion's baro_retry_task re-inits it every 10 s while
 * disarmed, so the blocker normally clears on its own within seconds. A
 * baro that stays dead needs a power cycle. See dps310.c. */
#define PREARM_SENSORS_REQUIRED \
    (SENSOR_IMU_OK | SENSOR_BARO_OK | SENSOR_MAG_OK)

/* ===================== HIL MODE =====================
 * 0 = normal flight firmware (real sensors, real RC).
 * 1 = Hardware-In-the-Loop bench mode:
 *       - IMU + mag samples are INJECTED over USB serial ("S,..." lines)
 *       - RC channels are INJECTED over USB serial ("R,..." lines)
 *       - real iBUS RX task is not started
 *     NEVER fly with HIL_MODE=1.
 * Lives here (component "common") so fusion, fc_tasks and
 * debug_stream all see the same value. */
#define HIL_MODE 0

typedef struct __attribute__((packed)) {
    float roll;
    float pitch;
    float yaw;
    float gyro_z;
    
    // Rate PID needs these
    float gyro_x;
    float gyro_y;
    
    uint8_t fix_type;
    uint8_t sats;
    
    uint8_t reserved[2]; 

    /* Latitude/longitude in 1e-7 degrees - u-blox's native NAV-PVT unit,
     * ~1.1 cm. These were `float` previously, which quantised
     * longitude to about 0.42 m at this latitude: a 32-bit float holds ~7
     * significant digits and 29.0610000 needs 9. The NMEA driver had been
     * computing exact doubles and then throwing the precision away on the
     * store. int32 is the same 4 bytes, so the struct layout is unchanged.
     *
     * TELEMETRY STILL SENDS DEGREES. CSV fields 17 and 18 are part of the
     * frozen 0-38 layout that MATLAB and the C6 dashboard index, so
     * telemetry.c scales by 1e-7 on the way out. Changing the wire format
     * here would have needed a C6 reflash to avoid breaking the map. */
    int32_t lat;        // 1e-7 deg, + north
    int32_t lon;        // 1e-7 deg, + east
    float gps_alt_m;    // GPS altitude MSL (GGA field 9 / NAV-PVT hMSL)
    float altitude_m;   // barometric altitude
    float pressure_pa;
    float temp_c;

    float mag_x;
    float mag_y;
    float mag_z;
    float quat[4];

    uint16_t m1_pwm;
    uint16_t m2_pwm;
    uint16_t m3_pwm;
    uint16_t m4_pwm;

    uint32_t val_heartbeat;

    // --- NEW: Telemetry Debug Data ---
    float des_roll;   // What you asked for (Stick input)
    float des_pitch;
    float des_yaw;    // Desired Rate
    float loop_dt;    // Real loop time
    uint8_t is_armed; // 0 = Disarmed, 1 = Armed

    // --- Bench / debug-stream data (appended at END so the C6 CSV
    //     telemetry above is unaffected) ---
    float acc_x;      // raw accel [g], body frame
    float acc_y;
    float acc_z;
    float fusion_dt;  // real fusion-loop dt [s] (loop timing audit)

    // --- HIL injection buffers (only used when HIL_MODE=1) ---
    struct {
        float ax, ay, az;      // [g]
        float gx, gy, gz;      // [dps]
        float mx, my, mz;      // [uT]
        bool  fresh;           // set by serial parser, cleared by fusion
    } hil_sens;

    // Pre-arm blocker bitmask (PREARM_* above). 0 = clear to arm.
    // Live while disarmed; the dashboard can say WHY arming is refused.
    uint16_t arm_blocked;

    // Per-cycle sensor health bitmask (SENSOR_* above). Appended at the
    // END so existing C6 CSV / MATLAB column indices are unaffected.
    uint8_t sensor_health;

    // --- Telemetry dashboard additions (appended at END) ---
    uint16_t rc_throttle;    // raw RC throttle channel, us (1000-2000)
    uint16_t rc_roll, rc_pitch, rc_yaw, rc_arm;  // raw channels, us
    // Consecutive received frames whose four GIMBAL channels were
    // byte-identical. The FS-iA6B keeps transmitting the LAST values
    // when the TX dies, so the 500 ms frame timeout never fires; this
    // is how that condition becomes visible. DIAGNOSTIC ONLY - nothing
    // disarms on it (a TX-side failsafe is the real fix).
    uint32_t rc_frozen_frames;
    uint8_t  rc_link_ok;     // 1 = a frame arrived within RC_TIMEOUT_US
    struct {                 // gains the flight task is ACTUALLY flying on
        float kp_rate, ki_rate, kd_rate;   // roll+pitch rate loop
        float kp_yaw,  ki_yaw,  kd_yaw;    // yaw rate loop
    } pid_active;

    // --- Vertical channel (P7 stage 0, appended at END) ---
    // Earth-frame Z acceleration with gravity removed, m/s^2, POSITIVE = UP.
    // Body accel rotated by the fusion quaternion, minus 1 g.
    //
    // Exists because neither vertical sensor is usable alone: altitude_m
    // above comes off a 32 Hz barometer and is noisy and slow, the
    // accelerometer runs at ~416 Hz and drifts, and NEITHER gives climb
    // rate. This is the high-rate half a vertical filter needs.
    //
    // Written only on a good IMU read - otherwise it HOLDS ITS LAST VALUE,
    // exactly like acc_x/y/z. Check SENSOR_IMU_OK before believing it.
    float accz_ms2;
    /* Earth-frame HORIZONTAL acceleration, m/s^2, +north / +east. The
     * horizontal counterpart of accz_ms2 and computed from the same
     * quaternion in the same place (fusion.c), so the 6-state horizontal
     * filter and the 15-state ESKF never have to re-derive attitude.
     * NO gravity term: gravity is vertical in the earth frame and lands
     * entirely in Z. Check SENSOR_IMU_OK before believing them. */
    float accn_ms2;
    float acce_ms2;

    // --- Vertical filter outputs (P7 stage 1, appended at END) ---
    // From components/common/vert_filter.c, running in the fusion task.
    //
    // Published EVERY cycle even when the IMU read failed, unlike
    // accz_ms2 above: the filter predicts through a bad sample rather
    // than holding a stale value, so these are always current.
    float alt_est_m;    // altitude above the ToF ground datum, m, +up
    float vz_ms;        // climb rate, m/s, POSITIVE UP
    float accz_bias;    // learned accelerometer Z bias, m/s^2 (Log A: 0.194)
    uint8_t vert_valid; // 1 = an absolute reference (baro or ToF) has been
                        // seen. A controller must NOT close a loop on pure
                        // dead reckoning - blind, the 2% accel scale error
                        // invents 1.9 m/s of climb in 10 s.

    // Metres of altitude added by a DATUM CHANGE rather than by the
    // aircraft moving, accumulated. The alt-hold controller tracks the
    // delta and adds it to its setpoint, so a re-datum never commands
    // motion. See vert_filter_take_datum_shift() for why this exists -
    // short version: without it the aircraft climbs over a table.
    float vert_datum_shift;

    // --- Alt-hold controller outputs (P7 stage 2, appended at END) ---
    float alt_sp_m;      // altitude setpoint, same datum as alt_est_m
    uint8_t vert_flags;  // VERT_F_* from blackbox.h - the SAME bit
                         // layout as the blackbox frame's, deliberately,
                         // so the dashboard and the log cannot drift
                         // apart on what bit 1 means.

    /* --- Second barometer, MS5611 at 0x76 (appended at END) ---
     * A CONTROL for the DPS310 investigation, never a flight input.
     * Nothing reads these to fly on; they exist to sit in the same log as
     * the DPS310. Left at zero when the part is absent, which is the
     * normal state until it is ordered and fitted.
     * Check SENSOR_BARO2_OK before believing any of them.
     *
     * baro2_alt_m uses the IDENTICAL formula and the identical hard-coded
     * SEA_LEVEL_PRESSURE as altitude_m above. That matters: the absolute
     * value of both is meaningless (see the QNH note), but the
     * DIFFERENCE between them is exactly the comparison being run, and it
     * is only honest if both were computed the same way. */
    float baro2_press_pa;
    float baro2_temp_c;
    float baro2_alt_m;

    /* --- GPS, Stage A (appended at END) ---------------------
     * Populated by the UBX driver from NAV-PVT. All zero, and gps_fix_ok
     * clear, when TBSM10_USE_NMEA is built instead - the NMEA sentences
     * this project parses carry none of it.
     *
     * Velocities are stored in the module's own mm/s rather than the cm/s
     * the plan sketched. nav_state is a RAM struct with no size pressure,
     * so dividing by ten would only round the source data away and add a
     * unit to get wrong. The name carries the unit. */
    int32_t vel_n_mms;      // NED velocity, mm/s, + north
    int32_t vel_e_mms;      // + east
    int32_t vel_d_mms;      // + DOWN. Negative means climbing.
    uint32_t gps_h_acc_mm;  // horizontal accuracy estimate, mm -> R matrix
    uint32_t gps_v_acc_mm;  // vertical accuracy estimate, mm
    uint32_t gps_s_acc_mms; // speed accuracy estimate, mm/s
    uint32_t gps_itow_ms;   // module time of week - identical value twice
                            // means the solution did not update
    /* Local esp_timer stamp of the last accepted NAV-PVT. int64 because a
     * uint32 of microseconds wraps every 71 minutes, and a wrapped
     * freshness check reads as "fresh" at exactly the wrong moment. */
    int64_t gps_time_us;
    /* NAV-PVT flags bit 0. A fixType of 3 with this bit CLEAR is the
     * module still settling - not a position anything may use. */
    uint8_t gps_fix_ok;

    /* Position in the local tangent plane, metres from the home point
     * latched at arm. Only meaningful while home_valid is 1. See
     * components/common/geo.h. */
    uint8_t home_valid;
    float pos_n_m;
    float pos_e_m;
    float pos_d_m;      // + DOWN, so a climb is negative

    /* Accumulated velocity increment the vertical filter has actually
     * integrated: sum of accz_ms2 * dt over every predict, m/s.
     *
     * WHY AN INTEGRAL AND NOT A SAMPLE. accz_ms2 above is an
     * INSTANTANEOUS value, published at the 416 Hz fusion rate and
     * sampled by the 250 Hz flight loop for the blackbox. That
     * decimation is lossy in exactly the way that matters: hover accz
     * carries sd 3.67 m/s^2 of vibration (s139), while a
     * sustained error of only 0.24 m/s^2 integrates to 23 m in 14 s. So
     * the log could not reconstruct what the filter integrated, and the
     * -23.46 m estimator runaway that flight recorded was not
     * reproducible offline - the same C code on the same logged inputs
     * stayed inside 1.6 m.
     *
     * Integrating in the PRODUCER fixes that: a sum over every predict
     * is immune to how often the consumer looks at it. The blackbox logs
     * this ABSOLUTE sum and the READER differences two frames, which gives
     * the exact velocity change the filter applied over that interval -
     * and stays exact across dropped frames, which a logged delta would
     * not.
     *
     * Advanced only on a good IMU read, inside the same `if` that calls
     * vert_filter_predict() - so a frame the filter skipped contributes
     * nothing here either, and the replay skips it too. */
    float accz_dv_ms;
    /* The same running integral for north and east - see accz_dv_ms for
     * why these are ABSOLUTE sums the reader differences, and not
     * per-frame deltas. m/s. */
    /* ---- Horizontal estimator output (Stage C step 2) ----
     * Filtered NED ground velocity, m/s, from horiz_filter. NOT the raw
     * vel_n_mms above: this one is fused with the accelerometer, runs at
     * the fusion rate rather than the module's 10 Hz, and survives a
     * dropped fix. It is what a velocity hold closes its loop around. */
    /* the author's altitude reference (alt_ref.c),. DISPLAY AND
     * LOG ONLY - nothing closes a loop on it yet, by his instruction:
     * prove it on the dashboard against real flights before it gets any
     * authority. It runs beside vert_filter on the same sensors so the
     * two can be compared honestly. */
    float alt_ref_m;
    /* Is alt_ref_m meaningful right now? 1 when the ToF can see the
     * ground, or when it cannot but the barometer is still answering.
     * 0 when the estimate is coasting on a barometer that has gone
     * quiet - published so the dashboard can dim the row instead of
     * showing a frozen number that looks live. Added with
     * the liveness clock in alt_ref.c. */
    uint8_t alt_ref_valid;

    float horiz_vn_ms;
    float horiz_ve_ms;
    /* Position estimate in the same local tangent plane as pos_n_m. */
    float horiz_pn_m;
    float horiz_pe_m;
    /* Safe to close a control loop around the two velocities RIGHT NOW.
     *
     * Read it EVERY cycle, never once at engage. The rule this encodes
     * is the one the ceiling crash bought: alt-hold engaged
     * on an estimator whose ToF had died, because `valid` was being set
     * by a sensor that could not see the ground. A controller must never
     * close a loop around an estimate whose source has stopped
     * answering.
     *
     * Set only while a home point exists, the module reports a good fix,
     * and a fix has actually ARRIVED recently - the third is the part
     * that catches a receiver which has stopped talking while its last
     * cached values still look perfectly reasonable. */
    uint8_t horiz_valid;

    float accn_dv_ms;
    float acce_dv_ms;
} nav_state_t;

extern nav_state_t global_nav_state;
extern SemaphoreHandle_t state_mutex;

void nav_state_init(void);

#endif