#ifndef PARAMS_H
#define PARAMS_H

/* Runtime parameter table (P4). One struct of floats, a name table for
 * the CLI, an NVS blob with version + CRC for persistence.
 *
 * Concurrency model, deliberately simple: writers are the two command
 * paths (USB console task and the dashboard's telemetry_rx task, both
 * through params_set) plus params_init at boot; readers are the flight and
 * fusion loops doing plain float loads. A 32-bit aligned float store is
 * atomic on Xtensa, so a reader sees either the old or the new value of
 * each field, never a torn one. No locks in the hot path.
 *
 * LIVE params take effect on the next loop iteration. BOOT params are
 * read once at task startup (gyro filters are initialised before the
 * fusion loop runs) - change, SAVE, power-cycle. */

#include <stdbool.h>
#include <stddef.h>

/* Bump whenever params_t changes shape or meaning. A blob with a bad CRC,
 * a NEWER version, or a names digest that does not match is REFUSED
 * (defaults win, PREARM_PARAMS_DEFAULTED). An OLDER v7+ blob whose names are
 * a prefix of this table is MIGRATED (params_load_blob). Blobs older than
 * v7 have no names digest and are refused.
 *
 * Lives in the header, not params.c, because it is not only an NVS concern:
 * blackbox.c stamps it into every session's params snapshot so a decoded log
 * says which table shape it was recorded against. */
#define PARAMS_VERSION  7   /* v7: blob carries a names digest (see params.c) */

typedef struct {
    /* rate loop, roll + pitch share gains (LIVE) */
    float rate_kp, rate_ki, rate_kd;
    /* yaw rate loop (LIVE) */
    float yaw_kp, yaw_ki, yaw_kd;
    /* angle outer loop P (LIVE) */
    float angle_kp;
    /* limits (LIVE) */
    float angle_max_deg;     /* stick full-scale tilt, deg */
    float yaw_rate_max;      /* stick full-scale yaw rate, dps */
    float rc_deadband;       /* stick counts ignored around centre */
    float min_idle_us;       /* armed motor floor, us */
    /* gyro filters (BOOT - fusion task reads them once at start) */
    float gyro_lpf_hz;
    float notch_hz;
    float notch_q;
    /* per-motor output scaling (LIVE) - trims a hot corner by scaling
     * the span above 1000 us. Scale-DOWN only (max 1.0): software can
     * equalise a strong motor downward, never add thrust to a weak
     * one. Motive: M1's ESC delivers ~20-24% more current per us than
     * its three matched brothers (ladder), the chronic
     * outlier of the ESC saga. MOT bench tests bypass these on
     * purpose so ladders always measure raw. NOTE: fields are
     * APPENDED here - blackbox snapshots store params_t verbatim and
     * the decoder maps names by length, so order is ABI. */
    float m1_scale, m2_scale, m3_scale, m4_scale;
    /* IMU mount level reference (LIVE) - the roll/pitch the board reads
     * when it is PHYSICALLY level, in degrees. Fusion rotates accel,
     * gyro and mag by the inverse of this before Madgwick sees them, so
     * "level" means level to the airframe rather than to however the
     * sensor happens to be soldered and screwed down.
     *
     * Measured by the LEVELCAL console command, applied immediately,
     * persisted only by an explicit SAVE - so a cal taken on a surface
     * that turned out not to be flat is undone by a power cycle.
     *
     * Motive: ~2 deg of level bias made the aircraft drift backwards in
     * every hover, which was chosen over a trim_pitch fudge because the
     * bias is in the SENSOR, not in the pilot's stick. Fixing the input
     * fixes attitude, the vertical projection of thrust, and alt-hold
     * all at once. */
    float level_roll_deg, level_pitch_deg;

    /* --- P7 alt-hold (LIVE), appended in params v4 ---
     * Read every cycle into alt_hold_cfg_t / vert_filter_t, so SET takes
     * effect on the next control loop and tuning does not need a
     * reboot. Sources for each number are in components/common/
     * alt_hold.c and vert_filter.c; the short version:
     *
     * alt_hover_thr  us above 1000 that hovers. MEASURED at ~420 from
     *                flight4 sessions 19-22, the four real hovers
     *                already in the blackbox ring.
     * alt_vref       pack voltage alt_hover_thr corresponds to. Those
     *                flights predate vbat_mv in the frame (834b1fb), so
     *                this is nominal 4S until Log B pins it down. The
     *                feedforward scales by alt_vref/vbat, which is the
     *                EXACT correction for constant thrust, not an
     *                approximation - see alt_hold.h.
     * alt_acc_sigma  vertical accelerometer noise, m/s^2 (the field sits
     *                after vel_max - see params.c). Log A measured 0.294
     *                at IDLE; real flight measured 1.77-2.83 on
     *, so the default is 2.0.
     * alt_kp_vz      us per m/s of climb-rate error. Derived from the
     *                hover throttle: d(accel)/d(span) = 2g/span_hover,
     *                mass cancels.
     * alt_kp_z       1/s. The OUTER loop, and it must stay several times
     *                slower than the inner one - 1.0 against an inner
     *                loop of 1.17 oscillated in the sim. */
    float alt_hover_thr;
    float alt_vref;
    float alt_vz_max;
    float alt_kp_z;
    float alt_kp_vz;
    float alt_ki_vz;
    float alt_i_max;
    float alt_thr_db;
    /* AUTOMATIC ALTITUDE HOLD. 1 = altitude hold engages itself whenever
     * the vertical estimate is healthy and the aircraft is airborne;
     * 0 = manual only.
     *
     * A PARAMETER and deliberately NOT a stick switch. the author's point,
     *, is right: a mode switch on the transmitter is one more
     * thing to get wrong with your thumbs at the worst moment, and mode
     * confusion is a real cause of accidents. A parameter is set once on
     * the bench and never touched in the air.
     *
     * DEFAULT 0, and that is not timidity. The vertical filter was
     * rewritten (chi-square gating, covariance inflation,
     * sensor trust) and has NEVER FLOWN. Shipping an untested control
     * loop enabled-by-default onto an aircraft is how the
     * ceiling happened. Set it to 1 from the console when you are ready
     * to test it: SET,alt_hold_auto,1 then SAVE.
     *
     * WAS `hold_auto`, renamed. One switch used to gate BOTH
     * holds, which made the documented test plan self-contradicting:
     * turning on altitude hold to test it also turned on velocity hold,
     * a loop that has never flown, on the same flight. See
     * vel_hold_auto at the end of this struct.
     *
     * MARKED no_migrate IN k_table. A mode enable must never be
     * inherited across a layout change: losing the setting costs a
     * console command, inheriting one costs a flight. This field is
     * exactly where alt_acc_sigma used to sit, and it
     * inherited that field's 2.0, clamped to this field's max of 1.0,
     * and switched both holds on for flights nobody chose that for. */
    float alt_hold_auto;

    /* Velocity hold (Stage C step 2). Defaults are deliberately timid -
     * this loop has never flown, and too much gain here produces a
     * pilot-induced oscillation that reads like a rate-loop tuning
     * problem and sends you debugging the wrong thing. */
    float vel_kp;        /* deg of tilt per m/s of error       */
    float vel_ki;        /* deg per m/s per second             */
    float vel_i_max;     /* bound on the integral CONTRIBUTION, deg */
    float vel_tilt_max;  /* SAFETY bound on commanded tilt, deg */
    float vel_max;       /* ground speed at full stick, m/s    */
    float alt_acc_sigma;

    /* --- VL53L0X range profile (BOOT), appended in params v5 ---
     * Read once by sys_ctrl at boot and handed to vl53l0x_init; a reprobe
     * reuses the same profile. SET + SAVE + power-cycle.
     *
     * ST's stock settings (0.25 MCPS, VCSEL 14/10) capped this airframe's
     * TOF200C at ~24 cm: every return past that was too
     * weak to pass the limit, and the chip fell back to reporting a
     * ~50 mm internal reference as a status-11 "valid" measurement -
     * pointed at an empty room. The SAME module reached 1700 mm under
     * Pololu's long-range profile, which is why the defaults below are
     * that profile and not ST's.
     *
     * The cost of reaching further is believing weaker returns, so these
     * are parameters rather than a silent lowering. If the sensor starts
     * locking onto walls or foliage instead of the ground, raise
     * tof_sig_lim first - it is the one that trades range for confidence.
     *
     * tof_sig_lim   final-range minimum count rate, MCPS. ST 0.25, ST's
     *               own long-range preset 0.10, MEASURED WORKING 0.05.
     * tof_pre_pclk  pre-range VCSEL pulse period. Legal 12/14/16/18.
     * tof_fin_pclk  final-range VCSEL pulse period. Legal 8/10/12/14.
     *               Off-list values snap to the nearest legal one and say
     *               so in the boot log.
     * tof_budget_ms measurement timing budget - integration time, and the
     *               biggest single lever on range. ST stock ~33, MEASURED
     *               WORKING 200.
     *
     * tof_budget_ms is ALSO the sample rate, and that is its cost: the
     * chip ranges back-to-back, so 200 ms means ~5 readings a second where
     * stock gave ~9. The vertical filter gets half as many ground fixes.
     * That was the trade accepted to get a rangefinder that can see the
     * ground at all from hover height.
     *
     * Bounded at 400 for a second reason: the dead-sensor detector in
     * vl53l0x.c counts consecutive polls with no new measurement, and a
     * long enough budget would make a HEALTHY sensor look stopped. Read
     * TOF_EMPTY_BLANK there before raising this bound. */
    float tof_sig_lim;
    float tof_pre_pclk;
    float tof_fin_pclk;
    float tof_budget_ms;

    /* AUTOMATIC VELOCITY HOLD, split out of hold_auto.
     *
     * Appended at the END rather than placed beside alt_hold_auto, and
     * that is the whole discipline this file now enforces: fields are
     * APPEND-ONLY so an older blob's floats stay a valid prefix of the
     * current struct. Tidiness in the struct is worth nothing against
     * one silently reinterpreted slot.
     *
     * DEFAULT 0. Velocity hold has never flown. It gets the same
     * logging-only first pass altitude is getting, and it cannot be
     * dragged into that flight by the altitude switch any more.
     *
     * MARKED no_migrate, same reasoning as alt_hold_auto. */
    float vel_hold_auto;

    /* BATTERY-SAG THROTTLE COMPENSATION, appended.
     *
     * 1 = the manual throttle stick is scaled by alt_vref/vbat before it
     * reaches the mixer, so the same stick keeps producing the same
     * thrust as the pack drains. 0 = raw stick, the behaviour every
     * flight so far has had.
     *
     * REUSES alt_vref rather than adding a second reference voltage.
     * Two references that could drift apart would make the throttle step
     * the instant altitude hold engaged or dropped, by exactly the
     * difference between them.
     *
     * COLLECTIVE ONLY, applied to the throttle before the mixer. All
     * four motors share the bus, so a per-motor version would inject a
     * roll/pitch/yaw disturbance out of a common-mode measurement.
     *
     * DEFAULT 0. This changes the throttle law the pilot has learned:
     * with it on, the aircraft stops sinking as the pack drains, which
     * is exactly the low-battery cue the author has been flying by ("sink =
     * battery sag"). Do not turn it on without a VBAT warning to replace
     * that cue.
     *
     * MARKED no_migrate, same reasoning as the two hold enables:
     * inheriting this one across a layout change would silently rewrite
     * the throttle response. */
    float thr_vcomp;
} params_t;

extern params_t g_params;

/* Defaults -> NVS load (if a valid blob exists). Call once in app_main,
 * BEFORE sys_ctrl/fc_tasks start - fusion reads filter params at task
 * start and the flight loop reads gains every cycle. */
void params_init(void);

/* Set by name. Returns 0 on success, -1 unknown name, -2 out of range.
 * LIVE fields apply immediately; BOOT fields apply next power-cycle. */
int params_set(const char *name, float value);

/* Get by name. Returns 0 and writes *out, or -1 if unknown. */
int params_get(const char *name, float *out);

/* Persist the current table to NVS. Returns 0 or negative esp error. */
int params_save(void);

/* Put every field back to its compiled-in default, IN RAM ONLY. The
 * caller must SAVE to make it stick, so a mistyped reset costs a reboot
 * and not a tuning session.
 *
 * Exists because there was no way back from a bad blob except retyping
 * every SET line by hand, and a recovery path that tedious is one that
 * does not get used. */
void params_reset_defaults(void);

/* True when a saved blob EXISTED and was REFUSED, so the aircraft is
 * flying compiled-in defaults rather than its tuning.
 *
 * Distinguished from "no blob at all", which is the normal state of a
 * fresh board and is not a fault. This one means m*_scale, the level
 * trim and every gain silently reverted, which is the same class of
 * failure as the one the names digest was added to catch - so it is
 * surfaced as a pre-arm blocker (PREARM_PARAMS_DEFAULTED) and not left
 * in a boot log nobody reads. Cleared by a successful SAVE. */
bool params_were_defaulted(void);

/* CLI support: iterate the table. Returns the number of params. */
int params_count(void);
/* Fills the caller's pointers for table entry i (0-based). Returns 0,
 * or -1 if i is out of range. boot_only: applies at next boot. */
int params_describe(int i, const char **name, float *value,
                    float *min, float *max, bool *boot_only);

#endif /* PARAMS_H */
