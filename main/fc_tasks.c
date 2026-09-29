#include "fc_tasks.h"
#include "fc_cores.h"
#include "config.h"
#include "alt_hold.h"
#include "vel_hold.h"
#include "iterm_gate.h"
#include "ibus.h"
#include "mixer.h"
#include "motors.h"
#include "nav_state.h"
#include "pid.h"
#include "params.h"
#include "blackbox.h"
#include "tbsM10.h"
#include "ina226.h"
#include "vl53l0x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

static const char *TAG = "FC";

/* NOTE: BB_LOG_RATE_HZ in components/blackbox/blackbox.h must match this.
 * It cannot include this file - the constant lives in a .c - and it needs
 * the rate to report ring capacity in seconds. */
#define FLIGHT_LOOP_RATE_HZ 250
#define MAX_STALE_LOOPS 50
#define RC_TIMEOUT_US   500000   /* 0.5 s without a valid RC frame = link lost */
/* How long alt-hold tolerates a silent barometer before handing the
 * throttle back. 1 s at the 250 Hz flight loop. Long enough to ride out
 * a bus glitch on a chip with a history of them, short enough that a
 * genuinely dead baro does not leave the aircraft holding altitude on a
 * sensor that no longer exists. */
#define ALT_BARO_GRACE_CYCLES  (FLIGHT_LOOP_RATE_HZ)

/* Alt-hold may only engage while the aircraft is actually FLYING.
 *
 * it engaged on the ground at 1081 us because the only
 * throttle gate was THROTTLE_LOW (1050), which exists to detect "stick
 * down for arming" and has nothing to do with flight. alt_hold captures
 * the stick position at the engage edge as its zero-climb neutral, so
 * neutral was captured 339 us below the 1420 us hover. Every stick
 * position the pilot used afterwards therefore read as "climb at
 * maximum rate", and the pilot spent nine seconds pushing a throttle
 * that could not do what they were asking of it.
 *
 * It also bumped: alt_hold seeds its integrator with clamp(stick - ff,
 * +-i_max), and 1081 - 1407 = -326 clamps to -200, so the output stepped
 * +126 us at the instant of engage instead of matching what was already
 * being flown. Engaging near hover keeps that term inside the clamp and
 * the transfer genuinely bumpless.
 *
 * Two independent conditions, because neither alone is enough. A stick
 * near hover does not prove the aircraft left the ground (it could be
 * held down), and an altitude reading does not prove the pilot is ready
 * to hand over the throttle. The altitude gate is only meaningful
 * because vert_filter's `valid` now requires a ToF fix within the last
 * 3 s - it is a real height above a real floor, not a baro guess. */
#define ALT_ENGAGE_THR_FRAC    0.70f    /* of hover_thr above 1000 us   */
#define ALT_ENGAGE_MIN_ALT_M   0.30f    /* above the ToF ground datum   */

/* Internal Data Structure */
typedef struct {
    uint16_t channels[IBUS_CHANNELS];
    bool failsafe;
} rc_packet_t;

static QueueHandle_t rc_queue;

/* ============ Bench motor test (MOT serial command) ============
 * Spins ONE motor at a fixed pulse width while DISARMED, for finding
 * weak motors (per-motor current / RPM / thrust comparison). The
 * flight task overwrites every motor every cycle, so the test has to
 * be applied inside its disarmed path - a write from another task
 * would be overwritten 4 ms later by the flight task's own motors_set().
 *
 * Shared as three volatiles written by the command task, read by the
 * flight task. loops is the enable and is written LAST; a torn read
 * is impossible (all <=32-bit) and the worst race costs one cycle. */
#define MOT_TEST_MAX_US  1700   /* ~70 % throttle cap for a restrained test */
#define MOT_TEST_MAX_MS  10000
static volatile uint8_t  mot_test_idx;      /* 0-based motor index */
static volatile uint16_t mot_test_us;
static volatile uint32_t mot_test_loops;    /* remaining flight-loop cycles */

void fc_motor_test(uint8_t motor_1based, uint16_t us, uint32_t ms)
{
    if (motor_1based < 1 || motor_1based > 4) {   /* MOT,0 = stop */
        mot_test_loops = 0;
        return;
    }
    if (us < 1000) us = 1000;
    if (us > MOT_TEST_MAX_US) us = MOT_TEST_MAX_US;
    if (ms > MOT_TEST_MAX_MS) ms = MOT_TEST_MAX_MS;

    mot_test_idx   = motor_1based - 1;
    mot_test_us    = us;
    mot_test_loops = ms * FLIGHT_LOOP_RATE_HZ / 1000;  /* enable last */
}

/* ============ ESC range calibration (ESCCAL serial command) ============
 * Teaches all four ESCs the SAME 1000-2000 us range at once: max first
 * (2000 us held while the user plugs the pack in), then min (1000 us).
 * Unequal learned ranges are the prime suspect for M1 pulling 3-4x the
 * current of its peers at an identical commanded pulse width - the four
 * ESCs were never calibrated as a set. A bare blocking max-then-min helper
 * cannot do this job: flight_task overwrites every motor every cycle, so
 * the sequence has to run as flight_task state, which is what this is.
 *
 * Stage protocol is deliberate friction - stage 2 drives FULL THROTTLE
 * on all four outputs:
 *   ESCCAL,HIGH -> 10 s confirm window, outputs unchanged
 *   ESCCAL,YES  -> 2000 us on all four, 60 s auto-abort
 *   ESCCAL,LOW  -> 1000 us, calibration stored, idle
 * Any arm attempt aborts to 1000 immediately. */
#define ESCCAL_CONFIRM_LOOPS (10 * FLIGHT_LOOP_RATE_HZ)
#define ESCCAL_HIGH_LOOPS    (60 * FLIGHT_LOOP_RATE_HZ)
static volatile int      esccal_state;   /* 0 idle, 1 confirm window, 2 HIGH */
static volatile uint32_t esccal_loops;

int fc_esccal(int stage)
{
    switch (stage) {
    case 1:                              /* request: open confirm window */
        esccal_loops = ESCCAL_CONFIRM_LOOPS;
        esccal_state = 1;                /* enable last */
        return 1;
    case 2:                              /* confirm: go HIGH */
        if (esccal_state != 1) return -1;
        esccal_loops = ESCCAL_HIGH_LOOPS;
        esccal_state = 2;
        return 2;
    default:                             /* LOW / abort */
        esccal_state = 0;
        return 0;
    }
}

/* ============ Stick passthrough (ESCPASS serial command) ============
 * Routes the RAW throttle channel to all four outputs while disarmed,
 * making the FC a wire so the CLASSIC TX-based ESC calibration ritual
 * works exactly as ESCs expect it: stick max -> plug pack -> entry
 * beeps -> stick min -> confirm beeps. Exists because the console-
 * driven ESCCAL failed on the author's ESCs the likely reason
 * is timing (these ESCs want the low promptly after the entry beeps,
 * and a dashboard round-trip is slower than a thumb).
 * 120 s auto-off; arm attempt cancels; no RC link = outputs held at
 * 1000 so stale packet data can never drive motors. */
#define ESCPASS_TIMEOUT_LOOPS (120 * FLIGHT_LOOP_RATE_HZ)
static volatile uint32_t escpass_loops;   /* remaining cycles; 0 = off */

void fc_esc_passthrough(bool on)
{
    escpass_loops = on ? ESCPASS_TIMEOUT_LOOPS : 0;
}

#if HIL_MODE
/* Bench RC injection: debug_stream parsed an "R,..." line over USB.
 * Takes the exact same path as a real iBUS frame (rc_queue overwrite),
 * so failsafe / RC-timeout / arming logic all run unmodified. */
void fc_hil_inject_rc(const uint16_t ch[IBUS_CHANNELS], bool failsafe) {
    if (rc_queue == NULL) return;
    rc_packet_t p;
    memcpy(p.channels, ch, sizeof(p.channels));
    p.failsafe = failsafe;
    xQueueOverwrite(rc_queue, &p);
}
#endif

/* ================= TASK: RX (Producer) ================= */
void rx_task(void *pvParameters) {
    ibus_init(IBUS_GPIO);
    rc_packet_t packet;

    while (1) {
        if (ibus_read(packet.channels, &packet.failsafe)) {
            xQueueOverwrite(rc_queue, &packet);
        }
    }
}

/* dt clamp watch. DIAGNOSTIC ONLY - the clamp itself is unchanged.
 *
 * The loop runs at 250 Hz (4 ms) and dt is clamped to 2..6 ms, so the
 * bounds should never be reached. If they are, the PID integrator is
 * counting the wrong amount of real time and every gain tuned in that
 * session was tuned against a lie. The  audit proposed widening
 * the clamp to 1..20 ms; that changes integration on a flying aircraft, so
 * the safe half is to first find out whether it is ever hit at all.
 * Counting costs nothing and settles the question with evidence.
 *
 * Written only by the flight task, read only for display. 32-bit aligned
 * words on Xtensa, so a torn read is impossible and no lock is needed. */
static volatile uint32_t s_dt_samples  = 0;
static volatile uint32_t s_dt_clamp_lo = 0;
static volatile uint32_t s_dt_clamp_hi = 0;
static volatile uint32_t s_dt_max_us   = 0;

void fc_dt_clamp_stats(uint32_t *samples, uint32_t *lo, uint32_t *hi,
                       uint32_t *max_us)
{
    if (samples) *samples = s_dt_samples;
    if (lo)      *lo      = s_dt_clamp_lo;
    if (hi)      *hi      = s_dt_clamp_hi;
    if (max_us)  *max_us  = s_dt_max_us;
}

/* NaN/Inf caught on the motor path and replaced (review, item
 * 5). A float -> integer conversion of a NaN is undefined in C, and the
 * trunc.s/utrunc.s this core compiles it to saturate instead of trapping,
 * so an unguarded NaN became a RAIL value: full or zero throttle, never a
 * controlled one. Must stay 0 - any count means an estimator or controller
 * produced a non-finite number and the loop substituted a safe one.
 * Written only by the flight task, read for STATS. */
static volatile uint32_t s_nonfinite = 0;

uint32_t fc_nonfinite_count(void) { return s_nonfinite; }

/* Pre-arm blockers, evaluated while disarmed. Arming is allowed only
 * when the returned mask is 0. Bits are defined in nav_state.h so the
 * dashboard can decode WHY arming is refused. */
static uint16_t prearm_blockers(const rc_packet_t *p,
                                float roll, float pitch,
                                float gx, float gy, float gz,
                                uint32_t heartbeat,
                                uint8_t sensor_health, bool vert_valid)
{
    uint16_t block = 0;

    if (fabsf(gx) > PREARM_GYRO_MAX_DPS ||
        fabsf(gy) > PREARM_GYRO_MAX_DPS ||
        fabsf(gz) > PREARM_GYRO_MAX_DPS)            block |= PREARM_GYRO_MOVING;

    if (fabsf(roll)  > PREARM_TILT_MAX_DEG ||
        fabsf(pitch) > PREARM_TILT_MAX_DEG)         block |= PREARM_NOT_LEVEL;

    int16_t sr = rc_normalize(p->channels[CH_ROLL]);
    int16_t sp = rc_normalize(p->channels[CH_PITCH]);
    int16_t sy = rc_normalize(p->channels[CH_YAW]);
    if (sr > PREARM_STICK_MAX || sr < -PREARM_STICK_MAX ||
        sp > PREARM_STICK_MAX || sp < -PREARM_STICK_MAX ||
        sy > PREARM_STICK_MAX || sy < -PREARM_STICK_MAX)
                                                    block |= PREARM_STICKS;

    if (p->channels[CH_THROTTLE] >= THROTTLE_LOW)   block |= PREARM_THROTTLE_HIGH;

    if (heartbeat < PREARM_MIN_HEARTBEAT)           block |= PREARM_FUSION_WARMUP;

    /* A sensor the flight code depends on is not answering. Nothing
     * checked this earlier, so the aircraft would lift off and
     * find out in the air. See PREARM_SENSORS_REQUIRED for what is in
     * the mask and why the mag's magnitude gate is not. */
    if ((sensor_health & PREARM_SENSORS_REQUIRED) != PREARM_SENSORS_REQUIRED)
        block |= PREARM_SENSOR_DEAD;

    /* Asking for alt-hold with no absolute reference to hold against.
     *
     * THIS IS THE CHECK THAT REFUSED THE FLIGHT. CH6 was up
     * before takeoff and the ToF had been frozen for two flights, so
     * vert_valid would be false here and the props would never have
     * spun.
     *
     * CH6 NO LONGER ENGAGES ANYTHING (alt_hold_auto) - the hold
     * decides for itself. This check is kept anyway, for two reasons
     * that both survive the change: an old habit of flicking CH6 up
     * before takeoff should not silently mean nothing, and a pilot who
     * does it is telling us he WANTS a hold, which is worth honouring
     * with the same refusal as before.
     *
     * It is NOT extended to cover alt_hold_auto, and that is
     * deliberate.
     * With automatic engagement a missing vert_valid simply means the
     * hold never engages and the aircraft flies manually - it degrades,
     * it does not misbehave. Blocking arming on it would ground every
     * flight where the ToF cannot see the ground, which is most outdoor
     * flying above 2 m. The refusal belongs where the hazard is, and
     * with auto-engage the hazard is gone. */
    if ((p->channels[CH_ALTHOLD] > ALTHOLD_ON) && !vert_valid)
        block |= PREARM_ALTHOLD_UNSAFE;

    /* The saved tuning was refused and this aircraft is on compiled
     * defaults - untrimmed motors, no level calibration.
     *
     * Blocking is the point. The failure this replaces was silent: a
     * blob whose slots had been reinterpreted loaded, got clamped into
     * legal-looking values, and flew, with one line in a boot log as the
     * only warning. Refusing that blob and then flying anyway on
     * defaults would move the silence rather than remove it.
     *
     * Cleared by SAVE, which is the right gesture: look at what GET
     * reports, fix what is wrong, persist it. */
    if (params_were_defaulted())
        block |= PREARM_PARAMS_DEFAULTED;

    /* No recorder, no flight. The two crashes worth learning from (s208,
     * and the s11 ground rocking that exposed the flash-erase stall) were
     * understood only through the blackbox. */
    if (!blackbox_ready())
        block |= PREARM_BLACKBOX_NOT_READY;

    return block;
}

/* Saturating float -> int16 for blackbox fields, for the derived values
 * (altitude, climb rate, earth-frame accel) rather than the raw sensor
 * ones. A wrapped int16 in a log is actively misleading - full-scale
 * positive reads as full-scale negative and looks like a real event -
 * while a flat line pinned at the rail is obviously "off the scale".
 *
 * This is saturation FOR DISPLAY, not the forbidden kind: nothing in the
 * control path reads it back, so it can never clamp a nonsense sensor
 * value into a plausible-looking range the way the INA226 cell count did
 * (ffe5be8). The NaN case lands on 0 - every comparison against NaN is
 * false, so both branches fall through. */
static inline int16_t bb_sat16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

/* ================= TASK: FLIGHT (Consumer) ================= */
void flight_task(void *pvParameters) {
    motors_init();
    
    // Gains come from the params table (P4): defaults in params.c, live
    // SET applies next cycle via the per-loop refresh below. Structure
    // (i_max/out_max/tau) stays compile-time - only gains are params.
    pid_cfg_t angle_roll_cfg  = { .kp = g_params.angle_kp, .ki = 0.0f, .kd = 0.0f, .i_max = 0, .out_max = 200.0f, .tau = 0.0f };
    pid_cfg_t angle_pitch_cfg = { .kp = g_params.angle_kp, .ki = 0.0f, .kd = 0.0f, .i_max = 0, .out_max = 200.0f, .tau = 0.0f };

    pid_cfg_t rate_roll_cfg   = { .kp = g_params.rate_kp, .ki = g_params.rate_ki, .kd = g_params.rate_kd, .i_max = 100, .out_max = 400.0f, .tau = 0.015f };
    pid_cfg_t rate_pitch_cfg  = { .kp = g_params.rate_kp, .ki = g_params.rate_ki, .kd = g_params.rate_kd, .i_max = 100, .out_max = 400.0f, .tau = 0.015f };
    pid_cfg_t rate_yaw_cfg    = { .kp = g_params.yaw_kp, .ki = g_params.yaw_ki, .kd = g_params.yaw_kd, .i_max = 100, .out_max = 400.0f, .tau = 0.015f };

    pid_state_t p_ang_r, p_ang_p;
    pid_state_t p_rate_r, p_rate_p, p_rate_y;

    /* Last gains the integrators were accumulating under, so a live SET
     * can be carried into them. Roll and pitch share rate_ki; yaw has its
     * own. The ANGLE loops need nothing - ki = 0 and i_max = 0, so they
     * never accumulate at all. */
    float prev_rate_ki = g_params.rate_ki;
    float prev_yaw_ki  = g_params.yaw_ki;

    /* Mixer saturation from the PREVIOUS cycle, for anti-windup.
     *
     * It has to be the previous one: the rate PIDs run before the mixer,
     * so this cycle's saturation does not exist yet when the integrators
     * need it. At 250 Hz that is 4 ms of lag on a condition that lasts as
     * long as the manoeuvre causing it, which is many cycles - the delay
     * costs one cycle of extra accumulation at the start and one of extra
     * freeze at the end. Restructuring the loop to avoid it would buy
     * nothing and risk the ordering of everything else. */
    int mixer_saturated = 0;

    pid_init(&p_ang_r); pid_init(&p_ang_p);
    pid_init(&p_rate_r); pid_init(&p_rate_p); pid_init(&p_rate_y);

    /* Rate I-term ground hold: on its legs the frame cannot rotate, so the
     * rate integrators are held at zero until the ToF shows it has lifted.
     * The rule and the logs behind it: components/common/iterm_gate.h. */
    iterm_gate_t itg;
    iterm_gate_init(&itg);

    // Vars
    rc_packet_t packet = {0}; 
    packet.failsafe = true; 
    packet.channels[CH_THROTTLE] = 1000;
    packet.channels[CH_ARM] = 1000;

    uint16_t motor_out[4];
    bool armed = false;
    /* Arm-switch interlock (PREARM_ARM_SWITCH). Set only by a LIVE frame
     * showing CH5 off while disarmed; cleared by every live frame showing
     * CH5 on. So the only way to arm is an off -> on the pilot makes while
     * nothing else blocks - never a blocker clearing under a switch that
     * was left on. Starts false: a switch that is already on at boot has
     * to be cycled. */
    bool arm_switch_seen_low = false;

    // Sensor Local Copies
    float c_roll = 0, c_pitch = 0, c_yaw = 0;
    float c_gyro_x = 0, c_gyro_y = 0, c_gyro_z = 0;
    float c_altitude_m = 0, c_accz_ms2 = 0;
    float c_baro2_alt_m = 0;          /* MS5611 control baro, see ms5611.h */
    uint8_t c_sensor_health = 0;

    // P7 alt-hold: estimator outputs and the controller's own state.
    float c_alt_est_m = 0, c_vz_ms = 0, c_datum_shift = 0;
    /* Accumulated velocity increment from the fusion task. Logged as-is
     * and differenced by the reader - see nav_state.accz_dv_ms. */
    float c_accz_dv_ms = 0;
    float c_accn_dv_ms = 0;
    float c_acce_dv_ms = 0;
    /* GPS, for the blackbox only - the flight loop does not steer on any
     * of it yet. Cached in the same critical section as everything else
     * so one frame cannot mix a position from one solution with an
     * accuracy from the next. */
    float c_pos_n_m = 0, c_pos_e_m = 0;
    int32_t c_vel_n_mms = 0, c_vel_e_mms = 0;
    uint32_t c_gps_h_acc_mm = 0;
    uint8_t c_sats = 0, c_fix_type = 0, c_gps_fix_ok = 0, c_home_valid = 0;
    uint8_t c_vert_valid = 0;
    float prev_datum_shift = 0;
    alt_hold_state_t alt = {0};
    alt_hold_cfg_t   alt_cfg = {0};
    alt_hold_init(&alt);
    alt_hold_default_cfg(&alt_cfg);
    vel_hold_state_t vel = {0};
    vel_hold_cfg_t   vel_cfg = {0};
    vel_hold_init(&vel);
    vel_hold_default_cfg(&vel_cfg);
    bool vel_engaged = false;
    /* AIRBORNE LATCH, shared by both holds and evaluated BEFORE either.
     *
     * Set once the aircraft is demonstrably flying, and never cleared
     * until disarm. Two reasons it latches rather than being re-tested:
     *
     *  - Engaging on the GROUND is one of the things that made the
     * ceiling crash possible, and a velocity hold on a
     *    stationary aircraft would fight ground friction with tilt and
     *    try to tip itself over.
     *  - Re-testing it every cycle would drop the mode mid-descent or
     *    mid-manoeuvre, in the flight regime with the least room to
     *    recover. A commanded descent looks exactly like "not airborne
     *    any more" to any instantaneous test.
     *
     * The altitude term uses alt_est, which is only meaningful while the
     * vertical estimate is valid - so this cannot latch on a filter that
     * has no reference, which is the same protection alt-hold's own
     * engage gate applies. */
    bool airborne_latch = false;
    float c_horiz_vn = 0, c_horiz_ve = 0;
    uint8_t c_horiz_valid = 0;

    uint16_t baro_bad_cycles = 0;  // consecutive cycles without SENSOR_BARO_OK
    bool alt_engaged = false;      // what the controller ACTUALLY did
    /* What the automatic policy ASKED for. Hoisted out of the alt-hold
     * block so the blackbox can log the verdict itself
     * instead of re-deriving something close to it: the frame build is
     * in a sibling scope and the re-derivation there had drifted into
     * being the CH6 switch position. Reset with alt_engaged below. */
    bool alt_want = false;
    float alt_sp_m = 0;
    uint16_t thr_cmd = 1000;       // throttle the mixer is handed
    /* Battery-sag compensation actually APPLIED this cycle - the param
     * is on AND the voltage was plausible. Declared out here for the
     * same reason alt_want is: the blackbox frame is built in a sibling
     * scope, and re-deriving the verdict there is how VERT_F_WANTED
     * ended up reporting a switch nobody was touching (incident 42). */
    bool thr_vcomp_on = false;

    // Blackbox arm-edge tracking (P4): session per armed period.
    bool prev_armed = false;

    // Barometric altitude at the arm edge - the zero the logged alt_cm is
    // measured from (P7 stage 0). Captured once per session so a log's
    // vertical axis reads "metres above where it took off" instead of
    // "metres above a sea level the DPS310 inferred from today's weather".
    // Whatever bias the baro has is in this value too, so it subtracts out.
    float alt_ref_m = 0.0f;
    /* The control barometer gets its OWN arm-edge zero rather than sharing
     * alt_ref_m. The two parts sit at different absolute offsets - a
     * different QNH error, a different mounting height - and forcing them
     * onto one reference would show a constant gap that looks like a
     * disagreement and is not. Zeroing each at arm means the plot shows
     * only what actually matters: which one stops tracking. */
    float baro2_ref_m = 0.0f;
    
    // Zombie check vars
    uint32_t last_heartbeat = 0;
    uint32_t current_heartbeat = 0;
    int stale_loop_counter = 0;

    // RC link-loss detection: time of last VALID frame from rx_task.
    // 0 = never received -> failsafe until the first frame arrives.
    int64_t last_rc_us = 0;

    // Frozen-RC detector state (see the block in the RC read below)
    uint16_t prev_gimbals[4] = {0};
    uint32_t rc_frozen_frames = 0;
    bool     have_prev_rc = false;

    // Timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FLIGHT_LOOP_RATE_HZ);
    
    int64_t last_time_us = esp_timer_get_time();
    int64_t last_dt_warn_us = 0;
    float dt_sec = 0.004f;

    // Watchdog: a hung flight loop = motors locked at last PWM.
    // Missing 50 cycles (200 ms) panics and reboots to motors-off.
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        esp_task_wdt_reset();

        // --- 0. CALCULATE REAL DT ---
        int64_t current_time_us = esp_timer_get_time();
        int64_t dt_us = current_time_us - last_time_us;
        last_time_us = current_time_us;
        
        dt_sec = (float)dt_us / 1000000.0f;

        /* Behaviour below is IDENTICAL to before - the same two bounds,
         * applied the same way. The only addition is counting, so STATS
         * can answer whether the clamp is ever actually reached. */
        s_dt_samples++;
        if (dt_us > 0 && (uint32_t)dt_us > s_dt_max_us) {
            s_dt_max_us = (uint32_t)dt_us;
        }
        if (dt_sec < 0.002f) { s_dt_clamp_lo++; dt_sec = 0.002f; }
        if (dt_sec > 0.006f) { s_dt_clamp_hi++; dt_sec = 0.006f; }

        /* Loud once a second while it is happening. A starved loop is not
         * a tuning curiosity: the integrator under-counts time for as long
         * as it lasts. Rate-limited on wall clock rather than on a cycle
         * divider, so a rare event is still reported promptly instead of
         * waiting for the 250th occurrence. */
        if (dt_us > 8000) {
            if (current_time_us - last_dt_warn_us > 1000000) {
                last_dt_warn_us = current_time_us;
                ESP_LOGW(TAG, "flight loop starved: dt %ld us (clamped to "
                              "6000). PID integration under-counts real "
                              "time while this lasts.", (long)dt_us);
            }
        }

        // --- A. READ RC ---
        rc_packet_t temp_packet;
        if (xQueueReceive(rc_queue, &temp_packet, 0) == pdTRUE) {
            // Frozen-frame detector. Turning the transmitter off does NOT
            // stop an FS-iA6B: it keeps sending the last channel values it
            // received, so frames keep arriving, rc_link_ok stays true, and
            // an armed aircraft stays armed on stale sticks. The real fix
            // is failsafe configured in the TX (CH5 below ARM_ON); this
            // counter exists so the condition is VISIBLE either way.
            //
            // Gimbal channels come off analog pots and dither by a count or
            // two whenever a human is holding them, so byte-identical
            // gimbals frame after frame means the receiver is repeating
            // itself. Switch channels are excluded - they are legitimately
            // constant.
            //
            // Deliberately NOT wired to disarm: a false positive here
            // stops the motors in flight. It is reported, not acted on.
            bool same = have_prev_rc;
            for (int i = 0; i < 4 && same; i++) {
                if (temp_packet.channels[i] != prev_gimbals[i]) same = false;
            }
            if (same) {
                rc_frozen_frames++;
            } else {
                rc_frozen_frames = 0;
                for (int i = 0; i < 4; i++) prev_gimbals[i] = temp_packet.channels[i];
                have_prev_rc = true;
            }

            packet = temp_packet;
            last_rc_us = current_time_us;
        }

        // RC link lost? The receiver may simply STOP sending frames on
        // signal loss - without this check we would keep flying on the
        // last received stick values forever.
        bool rc_link_ok = (last_rc_us != 0) &&
                          ((current_time_us - last_rc_us) < RC_TIMEOUT_US);

        // --- B. READ SENSORS & UPDATES ---
        if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
            c_roll  = global_nav_state.roll;
            c_pitch = global_nav_state.pitch;
            c_yaw   = global_nav_state.yaw;
            c_gyro_x = global_nav_state.gyro_x;
            c_gyro_y = global_nav_state.gyro_y;
            c_gyro_z = global_nav_state.gyro_z;
            c_altitude_m = global_nav_state.altitude_m;
            c_baro2_alt_m = global_nav_state.baro2_alt_m;
            c_accz_ms2   = global_nav_state.accz_ms2;
            c_alt_est_m  = global_nav_state.alt_est_m;
            c_vz_ms      = global_nav_state.vz_ms;
            c_vert_valid = global_nav_state.vert_valid;
            c_datum_shift = global_nav_state.vert_datum_shift;
            c_accz_dv_ms  = global_nav_state.accz_dv_ms;
            c_accn_dv_ms  = global_nav_state.accn_dv_ms;
            c_acce_dv_ms  = global_nav_state.acce_dv_ms;
            c_pos_n_m       = global_nav_state.pos_n_m;
            c_pos_e_m       = global_nav_state.pos_e_m;
            c_vel_n_mms     = global_nav_state.vel_n_mms;
            c_vel_e_mms     = global_nav_state.vel_e_mms;
            c_gps_h_acc_mm  = global_nav_state.gps_h_acc_mm;
            c_sats          = global_nav_state.sats;
            c_fix_type      = global_nav_state.fix_type;
            c_gps_fix_ok    = global_nav_state.gps_fix_ok;
            c_home_valid    = global_nav_state.home_valid;
            c_horiz_vn      = global_nav_state.horiz_vn_ms;
            c_horiz_ve      = global_nav_state.horiz_ve_ms;
            c_horiz_valid   = global_nav_state.horiz_valid;
            c_sensor_health   = global_nav_state.sensor_health;
            current_heartbeat = global_nav_state.val_heartbeat;
            xSemaphoreGive(state_mutex);
        }

        // Refresh gains from the params table every cycle (P4). Replaces
        // the old live_tuning handshake: SET/PID commands write g_params,
        // this picks the change up on the next loop. Plain float copies -
        // aligned 32-bit stores/loads cannot tear, whichever command task
        // (USB console or dashboard link) wrote them.
        angle_roll_cfg.kp  = g_params.angle_kp;
        angle_pitch_cfg.kp = g_params.angle_kp;
        rate_roll_cfg.kp   = g_params.rate_kp;
        rate_roll_cfg.ki   = g_params.rate_ki;
        rate_roll_cfg.kd   = g_params.rate_kd;
        rate_pitch_cfg.kp  = g_params.rate_kp;
        rate_pitch_cfg.ki  = g_params.rate_ki;
        rate_pitch_cfg.kd  = g_params.rate_kd;
        rate_yaw_cfg.kp    = g_params.yaw_kp;
        rate_yaw_cfg.ki    = g_params.yaw_ki;
        rate_yaw_cfg.kd    = g_params.yaw_kd;

        /* A ki change still has to reach the integrator. The integral is
         * RAW (ki applied at the output), so history is no
         * longer reinterpreted - but i_max bounds the OUTPUT contribution,
         * so raising ki can leave a legal integral producing more than
         * i_max. pid_retune_ki RE-CLAMPS it on the spot (no rescale any
         * more; see pid.h). The guards make this free on every cycle where
         * nothing changed. */
        if (rate_roll_cfg.ki != prev_rate_ki) {
            pid_retune_ki(&p_rate_r, prev_rate_ki, rate_roll_cfg.ki,
                          rate_roll_cfg.i_max);
            pid_retune_ki(&p_rate_p, prev_rate_ki, rate_pitch_cfg.ki,
                          rate_pitch_cfg.i_max);
            ESP_LOGI(TAG, "rate_ki %.4f -> %.4f, integrators rescaled",
                     (double)prev_rate_ki, (double)rate_roll_cfg.ki);
            prev_rate_ki = rate_roll_cfg.ki;
        }
        if (rate_yaw_cfg.ki != prev_yaw_ki) {
            pid_retune_ki(&p_rate_y, prev_yaw_ki, rate_yaw_cfg.ki,
                          rate_yaw_cfg.i_max);
            ESP_LOGI(TAG, "yaw_ki %.4f -> %.4f, integrator rescaled",
                     (double)prev_yaw_ki, (double)rate_yaw_cfg.ki);
            prev_yaw_ki = rate_yaw_cfg.ki;
        }

        // --- ZOMBIE CHECK ---
        if (current_heartbeat == last_heartbeat) {
            stale_loop_counter++;
        } else {
            stale_loop_counter = 0;
            last_heartbeat = current_heartbeat;
        }

        // ==========================================================
        // --- C. ARMING LOGIC (RC ONLY) ---
        // ==========================================================
        
        // 1. Define Conditions
        bool rc_arm = (packet.channels[CH_ARM] > ARM_ON);
        bool crash_detected = (fabsf(c_roll) > 80.0f) || (fabsf(c_pitch) > 80.0f);
        
        bool no_failsafe = (!packet.failsafe) &&
                           rc_link_ok &&
                           (stale_loop_counter < MAX_STALE_LOOPS) &&
                           (!crash_detected);

        /* Arm-switch interlock, part 1: note the switch OFF. Only from a
         * LIVE frame - a stale packet held over a dropout says nothing
         * about where the pilot's switch is now, and a frame the iBUS
         * decoder flagged as failsafe is not the pilot either. */
        const bool rc_live = rc_link_ok && !packet.failsafe;
        if (!armed && rc_live && !rc_arm) arm_switch_seen_low = true;

        // Pre-arm gate: evaluated EVERY cycle while disarmed (not just
        // on the arm attempt) so the blocker mask is continuously live
        // in telemetry. The old throttle-low check is one of its bits.
        uint16_t prearm = 0;
        if (!armed) {
            prearm = prearm_blockers(&packet, c_roll, c_pitch,
                                     c_gyro_x, c_gyro_y, c_gyro_z,
                                     current_heartbeat,
                                     c_sensor_health, c_vert_valid != 0);
            /* rc_live only: with no link there is no switch to talk about,
             * and no_failsafe (which needs the same link) refuses the arm
             * anyway - "cycle the switch" would send the pilot the wrong way. */
            if (rc_live && !arm_switch_seen_low) prearm |= PREARM_ARM_SWITCH;
        }

        // 2. Logic Implementation
        if (rc_arm && no_failsafe) {
            if (!armed) {
                if (prearm == 0) {
                    armed = true;
                } else {
                    // Arm attempt refused - say why, once per second
                    static int blocked_log_div = 0;
                    if (++blocked_log_div >= FLIGHT_LOOP_RATE_HZ) {
                        blocked_log_div = 0;
                        ESP_LOGW(TAG, "Arming blocked (0x%03X):%s%s%s%s%s%s%s%s%s%s",
                                 prearm,
                                 (prearm & PREARM_GYRO_MOVING)   ? " gyro-moving"   : "",
                                 (prearm & PREARM_NOT_LEVEL)     ? " not-level"     : "",
                                 (prearm & PREARM_STICKS)        ? " sticks"        : "",
                                 (prearm & PREARM_THROTTLE_HIGH) ? " throttle-high" : "",
                                 (prearm & PREARM_FUSION_WARMUP) ? " fusion-warmup" : "",
                                 (prearm & PREARM_SENSOR_DEAD)   ? " sensor-dead"   : "",
                                 (prearm & PREARM_ALTHOLD_UNSAFE)
                                                    ? " althold-no-reference" : "",
                                 (prearm & PREARM_PARAMS_DEFAULTED)
                                                    ? " params-defaulted(SAVE to clear)" : "",
                                 (prearm & PREARM_BLACKBOX_NOT_READY)
                                                    ? " blackbox-not-ready(BBINFO)" : "",
                                 (prearm & PREARM_ARM_SWITCH)
                                                    ? " arm-switch(cycle OFF then ON)" : "");
                    }
                }
            }
            // Already armed: stay armed (no re-checks mid-air; failsafe
            // conditions above are the in-flight guards)
        } else {
            armed = false;
        }

        /* Arm-switch interlock, part 2: every live frame with the switch
         * ON uses up the OFF that was seen before it - whether this cycle
         * armed, was refused by a blocker, or is flying. So a refused
         * attempt cannot turn into an arm later when the blocker clears
         * under a switch that stayed on, and after ANY disarm (failsafe,
         * crash, stale fusion) the switch has to come down before the
         * aircraft can arm again. */
        if (rc_live && rc_arm) arm_switch_seen_low = false;

        /* Rate I-term ground hold (iterm_gate.h). Stepped EVERY cycle, armed
         * or not: it has to watch the ToF sample counter move before the arm
         * edge to know whether the reading it takes there is fresh. One cache
         * read per cycle, shared with the blackbox frame, so the log shows
         * the exact number the gate decided on. */
        const uint16_t tof_mm_cycle = vl53l0x_cached_mm();
        const bool iterm_hold = iterm_gate_step(&itg, armed,
                                                packet.channels[CH_THROTTLE],
                                                tof_mm_cycle,
                                                vl53l0x_sample_seq(),
                                                current_time_us);

        // Blackbox session tracking (P4): one session per armed period.
        // Start BEFORE the control loop logs its first frame, stop on
        // the disarm edge so the writer flushes the partial sector.
        if (armed && !prev_armed) {
            blackbox_session_start();
            // Zero the logged altitude here, at the arm edge, on the
            // ground. Every alt_cm in this session is relative to this
            // one number. If the baro is dead c_altitude_m is 0, the
            // reference is 0, and alt_cm stays 0 all session - which is
            // why SENSOR_BARO_OK is logged next to it.
            alt_ref_m = c_altitude_m;
            baro2_ref_m = c_baro2_alt_m;
            /* Latch the GPS home point for the local tangent plane (Stage
             * A). Fails quietly and leaves home_valid at 0 when there is
             * no good fix - nothing is wired to control, so a missing home
             * costs a diagnostic and not a flight. */
            tbsm10_capture_home();
            /* Not silent: a flight on the fallback is logged as such too
             * (ctrl_flags bit 1 = 0), but the console should say it now. */
            if (!itg.ref_ok) {
                ESP_LOGW(TAG, "I-term ground hold: no fresh ToF reading at "
                              "arm - holding until throttle %u us instead",
                         (unsigned)ITERM_GATE_THR_US);
            }
        }
        if (!armed && prev_armed) {
            blackbox_session_stop();
            /* Drop home on disarm so the next arm cannot silently reuse an
             * origin from a previous flight in a different place. */
            tbsm10_clear_home();
        }
        prev_armed = armed;
        // ==========================================================


        // --- D. CONTROL LOOP ---
        if (!armed) {
            // Bench motor test (MOT command). An arm attempt cancels it
            // even if arming itself is refused by the pre-arm gate - the
            // test vibration can trip PREARM_GYRO_MOVING, and a spinning
            // test motor must never survive into an arm sequence.
            // Bench modes, mutually exclusive, priority ESCCAL > ESCPASS
            // > MOT. out_pwm is what actually reaches the ESCs this cycle
            // and what m_pwm reports below.
            uint16_t out_pwm[4] = {1000, 1000, 1000, 1000};
            if (rc_arm) {
                mot_test_loops = 0;
                esccal_state   = 0;
                escpass_loops  = 0;
            }
            if (esccal_state != 0) {
                // State 1 = YES-confirm window (outputs stay low),
                // state 2 = HIGH phase; either expires back to idle.
                mot_test_loops = 0;
                escpass_loops  = 0;
                if (esccal_loops > 0) esccal_loops--;
                else                  esccal_state = 0;
                if (esccal_state == 2)
                    for (int i = 0; i < 4; i++) out_pwm[i] = 2000;
            } else if (escpass_loops > 0) {
                // Stick passthrough: raw throttle straight to all four,
                // ONLY while the RC link is live - stale packet data must
                // never drive motors.
                escpass_loops--;
                mot_test_loops = 0;
                if (rc_link_ok) {
                    uint16_t thr = packet.channels[CH_THROTTLE];
                    if (thr < 1000) thr = 1000;
                    if (thr > 2000) thr = 2000;
                    for (int i = 0; i < 4; i++) out_pwm[i] = thr;
                }
            } else if (mot_test_loops > 0) {
                mot_test_loops--;
                out_pwm[mot_test_idx] = mot_test_us;
            }
            for (int i = 0; i < 4; i++) motors_set(i, out_pwm[i]);
            pid_init(&p_ang_r); pid_init(&p_ang_p);
            pid_init(&p_rate_r); pid_init(&p_rate_p); pid_init(&p_rate_y);
            /* And the saturation flag with them - the mixer is not being
             * run on this path, so a stale 1 from the last armed cycle
             * would otherwise freeze the integrators on the next arm. */
            mixer_saturated = 0;
            // Same rule as the PIDs: a disarm clears the integrator and
            // the engaged flag, so re-arming never starts inside a mode
            // the pilot did not just ask for, on an integrator wound up
            // from the previous flight.
            alt_hold_init(&alt);
            vel_hold_init(&vel);
            airborne_latch = false;
            baro_bad_cycles = 0;
            alt_engaged = false;
            alt_want = false;
            alt_sp_m = 0;
            /* Disarmed the mixer is not being run, so the compensation
             * is not being applied and the flag must not claim it was. */
            thr_vcomp_on = false;
            thr_cmd = packet.channels[CH_THROTTLE];
            // Swallow whatever the estimator re-datumed while disarmed
            // (it re-acquires the ground on the bench) so the first
            // armed cycle does not see a metre of accumulated shift.
            prev_datum_shift = c_datum_shift;

            if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
                // m_pwm reflects what actually reached the ESCs this
                // cycle (idle, MOT, ESCCAL or ESCPASS) for the D, stream.
                global_nav_state.m1_pwm = out_pwm[0];
                global_nav_state.m2_pwm = out_pwm[1];
                global_nav_state.m3_pwm = out_pwm[2];
                global_nav_state.m4_pwm = out_pwm[3];
                
                global_nav_state.des_roll = 0;
                global_nav_state.des_pitch = 0;
                global_nav_state.des_yaw = 0;
                global_nav_state.loop_dt = dt_sec;
                global_nav_state.alt_sp_m = 0;
                /* Disarmed the estimator still runs, so its validity and
                 * the ToF are live on the dashboard - that is how the
                 * pre-flight check "will alt-hold even engage?" gets
                 * answered on the bench instead of in the air. */
                /* WANTED is deliberately 0 here: disarmed, the policy is
                 * asking for nothing, whatever the switch says. CH6 is
                 * still reported so the pre-flight habit stays visible
                 * on the bench. */
                global_nav_state.vert_flags = (uint8_t)(
                      (c_vert_valid ? VERT_F_VALID : 0)
                    | ((packet.channels[CH_ALTHOLD] > ALTHOLD_ON)
                                    ? VERT_F_CH6 : 0)
                    | ((vl53l0x_cached_mm() != 0 &&
                        vl53l0x_cached_mm() != 65535) ? VERT_F_TOF_OK : 0));

                // IMPORTANT: Tell C6 that we are Disarmed
                global_nav_state.is_armed = 0;
                global_nav_state.arm_blocked = prearm;

                // Dashboard: raw RC + the gains actually in use
                global_nav_state.rc_throttle = packet.channels[CH_THROTTLE];
                global_nav_state.rc_roll     = packet.channels[CH_ROLL];
                global_nav_state.rc_pitch    = packet.channels[CH_PITCH];
                global_nav_state.rc_yaw      = packet.channels[CH_YAW];
                global_nav_state.rc_arm      = packet.channels[CH_ARM];
                global_nav_state.rc_frozen_frames = rc_frozen_frames;
                global_nav_state.rc_link_ok  = rc_link_ok ? 1 : 0;
                global_nav_state.pid_active.kp_rate = rate_roll_cfg.kp;
                global_nav_state.pid_active.ki_rate = rate_roll_cfg.ki;
                global_nav_state.pid_active.kd_rate = rate_roll_cfg.kd;
                global_nav_state.pid_active.kp_yaw  = rate_yaw_cfg.kp;
                global_nav_state.pid_active.ki_yaw  = rate_yaw_cfg.ki;
                global_nav_state.pid_active.kd_yaw  = rate_yaw_cfg.kd;

                xSemaphoreGive(state_mutex);
            }
        }
        else {
            // A completed arm always kills any bench mode. Without this a
            // MOT countdown paused by an arm/disarm cycle would RESUME
            // spinning its motor the moment the aircraft disarms (the
            // rc_arm cancel above only runs while the disarmed path does).
            mot_test_loops = 0;
            esccal_state   = 0;
            escpass_loops  = 0;

            // 1. Map Stick. Deadband is applied to the STICK around its
            // center: gimbal jitter must not become setpoint jitter.
            // (It used to be applied to |setpoint - measurement|, which
            // zeroed the angle error within 2 deg - P term dead near
            // hover, free drift, step input at the band edge.)
            int16_t stick_roll  = rc_normalize(packet.channels[CH_ROLL]);
            int16_t stick_pitch = rc_normalize(packet.channels[CH_PITCH]);
            int16_t stick_yaw   = rc_normalize(packet.channels[CH_YAW]);
            int16_t deadband    = (int16_t)g_params.rc_deadband;
            if (stick_roll  > -deadband && stick_roll  < deadband) stick_roll  = 0;
            if (stick_pitch > -deadband && stick_pitch < deadband) stick_pitch = 0;
            if (stick_yaw   > -deadband && stick_yaw   < deadband) stick_yaw   = 0;

            if (!airborne_latch && c_vert_valid &&
                (float)packet.channels[CH_THROTTLE] >=
                    1000.0f + ALT_ENGAGE_THR_FRAC * g_params.alt_hover_thr &&
                c_alt_est_m >= ALT_ENGAGE_MIN_ALT_M) {
                airborne_latch = true;
            }

            float des_roll     = stick_roll  * g_params.angle_max_deg / 500.0f;
            float des_pitch    = stick_pitch * g_params.angle_max_deg / 500.0f;

            /* ---- VELOCITY HOLD (Stage C step 2) ----
             *
             * Overrides the stick-derived tilt above when it engages, so
             * STICKS CENTRED MEANS ZERO GROUND SPEED rather than zero
             * tilt. Placed here, right after the stick mapping and
             * before the angle loop, because that is exactly what it
             * replaces - everything downstream is untouched and still
             * sees a tilt command in degrees.
             *
             * AUTOMATIC, on the same policy as alt-hold: no switch, and
             * the three conditions that matter are machine-checked every
             * cycle. horiz_valid already carries the GPS freshness test,
             * so a receiver that has gone quiet drops the mode by itself
             * rather than holding a stale velocity.
             *
             * The airborne gate is deliberately the SAME one alt-hold
             * uses. Engaging on the ground is one of the things that
             * made possible, and a velocity hold on a
             * stationary aircraft would fight ground friction with tilt
             * and try to tip itself over. Once flying it stops being
             * re-tested (alt.engaged keeps it latched), for the same
             * reason as there: dropping the mode mid-manoeuvre is worse
             * than staying in it. */
            const bool vel_want =
                (g_params.vel_hold_auto > 0.5f) &&
                c_horiz_valid &&
                airborne_latch &&
                (packet.channels[CH_THROTTLE] > THROTTLE_LOW);

            /* cos/sin of heading, computed HERE and not inside the
             * controller: __builtin_sinf/cosf lower to libm calls, and
             * vel_hold.c is written to link freestanding so its host
             * test can run it. This is the one place that already has
             * libm linked anyway. */
            const float yaw_rad = c_yaw * 0.017453292519943295f;
            const float cos_yaw = cosf(yaw_rad);
            const float sin_yaw = sinf(yaw_rad);

            vel_cfg.kp       = g_params.vel_kp;
            vel_cfg.ki       = g_params.vel_ki;
            vel_cfg.i_max    = g_params.vel_i_max;
            vel_cfg.tilt_max = g_params.vel_tilt_max;
            vel_cfg.vel_max  = g_params.vel_max;

            vel_engaged = vel_hold_update(
                &vel, &vel_cfg, vel_want, stick_roll, stick_pitch,
                c_horiz_vn, c_horiz_ve, cos_yaw, sin_yaw,
                c_horiz_valid != 0, dt_sec, &des_roll, &des_pitch);
            /* NaN guard (s_nonfinite). A non-finite tilt command means the
             * horizontal estimate behind it is broken: drop the hold and
             * fly the sticks, so the angle loop keeps a real setpoint
             * instead of losing stabilisation on both axes. */
            if (!isfinite(des_roll) || !isfinite(des_pitch)) {
                s_nonfinite++;
                vel_hold_init(&vel);
                vel_engaged = false;
                des_roll  = stick_roll  * g_params.angle_max_deg / 500.0f;
                des_pitch = stick_pitch * g_params.angle_max_deg / 500.0f;
            }

            // Yaw stick is NEGATED. Not a typo - bench-measured:
            // yaw stick right (CW from above) made the airframe turn LEFT.
            //
            // The body frame here is right-handed with z UP, so gyro_z > 0
            // means nose LEFT, and mixer_quad_x is built to match: +yaw
            // raises the CW pair (M1 FR, M4 RL), whose CCW reaction torque
            // turns the nose LEFT. That convention is CORRECT and the rate
            // loop depends on it - with the stick centred, a nose-left
            // disturbance gives error = -gyro_z < 0 -> -yaw -> the CCW pair
            // (M2, M3) lifts -> nose pushed back right. Negative feedback.
            //
            // So the ONLY broken link was the pilot mapping: stick right
            // (rc_normalize > 0) was asking for +yaw = nose left. Fix it
            // here and nowhere else. Flipping the mixer table instead would
            // invert that damping term into POSITIVE feedback and the yaw
            // axis would run away on the first disturbance.
            //
            // Consequence: nav_state.des_yaw (CSV field 6) is a body-frame
            // yaw RATE, so it reads negative while you yaw right. Correct,
            // and it matches the sign of gyro_z next to it.
            float des_yaw_rate = -stick_yaw  * g_params.yaw_rate_max / 500.0f;

            // 2. Outer Loop: Angle -> Rate Target
            float target_rate_roll  = pid_calculate(&angle_roll_cfg,  &p_ang_r, des_roll,  c_roll,  dt_sec);
            float target_rate_pitch = pid_calculate(&angle_pitch_cfg, &p_ang_p, des_pitch, c_pitch, dt_sec);

            // 3. Inner Loop: Rate Target -> Motor Mix
            /* Ground hold: each cycle on its legs starts from an empty
             * integrator, so this cycle's I is one error*dt*ki - under
             * 0.1 us at today's gains with the frame still - and nothing a
             * stick did on the ground survives into the air. P and D are
             * untouched: the aircraft answers sticks and gyro as before. */
            if (iterm_hold) {
                p_rate_r.integral = 0.0f;
                p_rate_p.integral = 0.0f;
                p_rate_y.integral = 0.0f;
            }
            /* Rate loops carry the anti-windup flag; the ANGLE loops
             * above do not, and cannot - they have ki = 0 and i_max = 0,
             * so there is no integrator to protect. */
            float mix_roll  = pid_calculate_sat(&rate_roll_cfg,  &p_rate_r, target_rate_roll,  c_gyro_x,  dt_sec, mixer_saturated);
            float mix_pitch = pid_calculate_sat(&rate_pitch_cfg, &p_rate_p, target_rate_pitch, c_gyro_y,  dt_sec, mixer_saturated);
            float mix_yaw   = pid_calculate_sat(&rate_yaw_cfg,   &p_rate_y, des_yaw_rate,      c_gyro_z,  dt_sec, mixer_saturated);

            // 3b. Altitude hold (P7 stage 2). Replaces the raw throttle
            // stick with a commanded throttle; when disengaged it hands
            // the stick straight back, so the mixer call below is
            // unchanged in the manual case.
            //
            // The gates are ALL here rather than inside alt_hold.c so
            // the controller stays a pure function and every reason it
            // can be refused is readable in one place:
            //
            //   vert_valid   the estimator has seen an absolute
            //                reference. Blind, the measured 2% accel
            //                scale error invents 1.9 m/s of climb in
            //                10 s - an alt-hold running on dead
            //                reckoning would fly that into the ceiling.
            //   BARO_OK      a dead barometer leaves only the ToF, which
            //                is blind above 2 m. Refuse rather than hold
            //                altitude on a sensor that stops existing
            //                partway up.
            //   throttle     below the arming floor the pilot is asking
            //                to descend or has given up. Alt-hold must
            //                never re-apply throttle to an aircraft
            //                whose pilot just closed it.
            //
            // Failsafe and disarm are handled structurally: this whole
            // branch only runs while armed, and `armed` already folds in
            // no_failsafe.
            {
                // SENSOR_BARO_OK is a PER-CYCLE bit - it reports the last
                // read, so one failed I2C transaction clears it for ~20 ms.
                // Gating alt-hold on it directly would drop the pilot out
                // of a hover on a single glitch, and this barometer has a
                // history of dropouts (and the 100 kHz bus runs on
                // internal pull-ups). The
                // filter coasts on the accelerometer and the ToF for a
                // second without trouble, so ride out a blip and only
                // hand back if the baro is genuinely gone.
                //
                // One-sided on purpose: the grace window lets an ENGAGED
                // hold survive a glitch, and because engaging also tests
                // this flag, a baro that has been dead for more than a
                // second cannot be engaged on at all.
                if (c_sensor_health & SENSOR_BARO_OK) baro_bad_cycles = 0;
                else if (baro_bad_cycles < ALT_BARO_GRACE_CYCLES)
                    baro_bad_cycles++;

                /* isfinite: a NaN estimate is not a reference either. Refusing
                 * it HERE hands back through alt_hold's 0.5 s exit ramp; the
                 * guard after alt_hold_update is only the backstop. */
                const bool alt_sensors_ok =
                    c_vert_valid && (baro_bad_cycles < ALT_BARO_GRACE_CYCLES) &&
                    isfinite(c_alt_est_m) && isfinite(c_vz_ms);

                /* See ALT_ENGAGE_THR_FRAC. Only gates the ENGAGE edge -
                 * once holding, the aircraft is allowed to descend below
                 * this altitude and the pilot is allowed to pull the
                 * stick below this throttle, which is exactly what a
                 * commanded descent looks like. Re-testing them every
                 * cycle would drop the mode mid-descent, in the flight
                 * regime with the least room to recover. */
                const bool alt_airborne =
                    alt.engaged ||
                    ((float)packet.channels[CH_THROTTLE] >=
                         1000.0f + ALT_ENGAGE_THR_FRAC * g_params.alt_hover_thr
                     && c_alt_est_m >= ALT_ENGAGE_MIN_ALT_M);

                /* AUTOMATIC ENGAGEMENT (alt_hold_auto),.
                 * Split from the single hold_auto so
                 * testing altitude hold no longer drags velocity hold
                 * into the same flight.
                 *
                 * The hold now engages ITSELF whenever it is safe to,
                 * instead of waiting for CH6. Nothing about the safety
                 * gates changed - alt_sensors_ok, alt_airborne and the
                 * throttle floor are the same three tests that were
                 * already here and they still all have to pass. What
                 * went away is the pilot having to ask.
                 *
                 * Why that is SAFER and not laxer: a switch is a fourth
                 * condition the pilot has to get right with his thumbs
                 * while flying, and mode confusion is a real cause of
                 * accidents. The three conditions that actually matter
                 * are machine-checkable and are checked every cycle. If
                 * the estimate goes bad the hold drops itself and hands
                 * the sticks back - which is what alt_hold's exit ramp
                 * is for - and the pilot flies on exactly as before
                 * without having to notice or reach for anything.
                 *
                 * CH6 is deliberately NOT read here any more. Leaving it
                 * as a second way in would recreate the confusion this
                 * removes, and a switch that sometimes matters is worse
                 * than one that never does. It still gates ARMING (see
                 * the pre-arm check) so an old habit cannot bite. */
                alt_want =
                    (g_params.alt_hold_auto > 0.5f) &&
                    alt_sensors_ok &&
                    alt_airborne &&
                    (packet.channels[CH_THROTTLE] > THROTTLE_LOW);

                // Gains live, straight from params, every cycle - the
                // whole loop is tunable over the console mid-bench.
                alt_cfg.hover_thr    = g_params.alt_hover_thr;
                alt_cfg.v_ref        = g_params.alt_vref;
                alt_cfg.vz_max       = g_params.alt_vz_max;
                alt_cfg.kp_z         = g_params.alt_kp_z;
                alt_cfg.kp_vz        = g_params.alt_kp_vz;
                alt_cfg.ki_vz        = g_params.alt_ki_vz;
                alt_cfg.i_max        = g_params.alt_i_max;
                alt_cfg.thr_deadband = g_params.alt_thr_db;

                // Only the CHANGE since last cycle: the filter's
                // accumulator is monotonic, the controller wants a delta.
                const float d_datum = c_datum_shift - prev_datum_shift;
                prev_datum_shift = c_datum_shift;

                const float vbat_v = (float)ina226_cached_mv() * 0.001f;

                /* BATTERY-SAG THROTTLE COMPENSATION (thr_vcomp).
                 *
                 * Thrust follows the ELECTRICAL POWER the motor gets,
                 * and a PWM ESC gives it duty * V_bus, so holding
                 * duty * V constant holds thrust constant as the pack
                 * drains. Full derivation and the datasheet numbers that
                 * justify it are on thr_vcomp_us() in alt_hold.h.
                 *
                 * APPLIED TO THE STICK GOING IN, not to thr_cmd coming
                 * out, and the difference is not cosmetic:
                 *
                 *  - alt_hold's feedforward ALREADY scales hover_thr by
                 *    the same ratio. Scaling its output too would apply
                 *    the correction twice to the feedforward.
                 *  - alt_hold seeds its bumpless transfer from `stick`
                 *    and captures thr_neutral from it. Feeding it the
                 *    raw stick while the manual path flew a compensated
                 *    one would step the throttle by (scale-1)*span the
                 *    instant the hold engaged - about 28 us at 15.0 V.
                 *
                 * Compensating the input makes every branch consistent:
                 * disengaged it returns the compensated stick, engaged
                 * its feedforward carries the same scale, and the exit
                 * ramp interpolates between two compensated values.
                 *
                 * The GATES above are deliberately left on the RAW
                 * channel. THROTTLE_LOW and the engage floor are
                 * questions about where the pilot's thumb is, not about
                 * what the mixer was handed. */
                uint16_t stick_thr = packet.channels[CH_THROTTLE];
                thr_vcomp_on = false;
                if (g_params.thr_vcomp > 0.5f) {
                    const float vs = alt_hold_vscale(g_params.alt_vref,
                                                     vbat_v);
                    /* A refused voltage returns exactly 1.0, so the flag
                     * reports what was APPLIED rather than what was
                     * asked for. "On but the INA226 was reading rubbish"
                     * and "off" are different throttle laws and the log
                     * has to tell them apart. */
                    thr_vcomp_on = (vs != 1.0f);
                    stick_thr = (uint16_t)thr_vcomp_us((float)stick_thr,
                                                       g_params.alt_vref,
                                                       vbat_v);
                }
                const float thr_f = alt_hold_update(
                    &alt, &alt_cfg, alt_want,
                    stick_thr,
                    c_alt_est_m, c_vz_ms, d_datum, vbat_v, dt_sec);
                /* NaN guard (s_nonfinite). ah_clamp passes a NaN straight
                 * through - every compare with NaN is false - so this is the
                 * only thing between a NaN and the uint16 cast. Drop the
                 * hold's poisoned state and fly the pilot's stick, which is
                 * always finite. */
                if (isfinite(thr_f)) {
                    thr_cmd = (uint16_t)thr_f;
                } else {
                    s_nonfinite++;
                    alt_hold_init(&alt);
                    thr_cmd = stick_thr;
                }
                alt_engaged = alt.engaged;
                alt_sp_m    = alt.z_sp;
            }

            /* NaN guard (s_nonfinite), the last line before the int16 casts
             * into the mixer. A NaN that gets into a PID never leaves it -
             * D-on-measurement and the integrator carry it every cycle, and
             * NaN * kd is NaN even at kd = 0 - so the axis would stay dead
             * until disarm. Zero that axis's correction for this cycle and
             * restart its PIDs clean. */
            if (!isfinite(mix_roll)) {
                s_nonfinite++;
                mix_roll = 0.0f;
                pid_init(&p_ang_r); pid_init(&p_rate_r);
            }
            if (!isfinite(mix_pitch)) {
                s_nonfinite++;
                mix_pitch = 0.0f;
                pid_init(&p_ang_p); pid_init(&p_rate_p);
            }
            if (!isfinite(mix_yaw)) {
                s_nonfinite++;
                mix_yaw = 0.0f;
                pid_init(&p_rate_y);
            }

            // 4. Mixing
            mixer_saturated = mixer_quad_x(
                thr_cmd,
                (int16_t)mix_roll,
                (int16_t)mix_pitch,
                (int16_t)mix_yaw,
                (uint16_t)g_params.min_idle_us,
                motor_out
            );
            
            // 4b. Per-motor output scaling (params m1..m4_scale): trims
            // a hot corner by scaling the span above 1000 us, so the
            // fleet is symmetric before the PID has to trim it (M1's
            // ESC delivers ~20-24% more per us - ladder).
            // The idle floor is re-applied after scaling so airmode
            // never stalls a motor. MOT bench tests bypass this on
            // purpose - current ladders must measure the raw response.
            {
                const float mscale[4] = {
                    g_params.m1_scale, g_params.m2_scale,
                    g_params.m3_scale, g_params.m4_scale,
                };
                uint16_t idle_us = (uint16_t)g_params.min_idle_us;
                for (int i = 0; i < 4; i++) {
                    uint16_t s = 1000 + (uint16_t)
                        ((float)(motor_out[i] - 1000) * mscale[i]);
                    motor_out[i] = (s < idle_us) ? idle_us : s;
                }
            }

            // 5. Output
            for (int i = 0; i < 4; i++) motors_set(i, motor_out[i]);

            const uint16_t tof_now = tof_mm_cycle;

            // 5b. Blackbox frame (P4): what the controller saw and did
            // this cycle. Non-blocking; drops are counted, never waited.
            bb_frame_t bbf = {
                .t_us    = (uint32_t)current_time_us,
                .gyro    = { (int16_t)(c_gyro_x * 16.0f),
                             (int16_t)(c_gyro_y * 16.0f),
                             (int16_t)(c_gyro_z * 16.0f) },
                .setp    = { (int16_t)(target_rate_roll  * 16.0f),
                             (int16_t)(target_rate_pitch * 16.0f),
                             (int16_t)(des_yaw_rate      * 16.0f) },
                .att     = { (int16_t)(c_roll  * 100.0f),
                             (int16_t)(c_pitch * 100.0f),
                             (int16_t)(c_yaw   * 100.0f) },
                .motor   = { motor_out[0], motor_out[1],
                             motor_out[2], motor_out[3] },
                .throttle = packet.channels[CH_THROTTLE],
                .sensor_health = c_sensor_health,
                /* Cache read, no I2C: the bus owner samples the INA226
                 * at 5 Hz, so this repeats across ~50 frames. That is
                 * the point - the log needs voltage on the SAME
                 * timebase as the collective, not a faster one. */
                .vbat_mv = ina226_cached_mv(),
                /* Vertical channel (P7 stage 0). Logged before any
                 * estimator or controller exists, because the filter is
                 * designed offline against THIS data and there is no
                 * other way to see a 250 Hz vertical signal - the
                 * telemetry CSV runs at 4 Hz. */
                .alt_cm    = bb_sat16((c_altitude_m - alt_ref_m) * 100.0f),
                .vz_cms    = bb_sat16(c_vz_ms * 100.0f),
                .accz_cms2 = bb_sat16(c_accz_ms2 * 100.0f),
                /* Cache read, no I2C: the bus owner POLLS the ToF at
                 * ~9 Hz but the chip DELIVERS at 1000/tof_budget_ms
                 * (~5 Hz at 200 ms), so this repeats across ~50 frames.
                 * Ground truth below 2 m, sentinel 65535 above it. */
                .tof_mm    = tof_now,
                /* Alt-hold channel (P7 stage 2). alt_est_cm shares the
                 * FILTER's ground datum, not alt_cm's arm-edge baro
                 * zero - two different zeros in one frame, on purpose:
                 * the gap between them IS the baro bias the filter
                 * learned, visible without re-running anything. */
                .alt_est_cm = bb_sat16(c_alt_est_m * 100.0f),
                .alt_sp_cm  = bb_sat16(alt_sp_m * 100.0f),
                /* WANTED and VEL_WANT are the POLICY VERDICTS - the very
                 * booleans the controllers were handed this cycle, not a
                 * re-derivation of them. Both used to be rebuilt here
                 * from a subset of their real gates, and WANTED had
                 * drifted all the way back to the CH6 switch position,
                 * so the log could not separate "the policy refused"
                 * from "nothing ever asked". s179 recorded ENGAGED on
                 * 50.4% of frames with WANTED at 0% - impossible under
                 * either meaning, and nothing flagged it.
                 *
                 * CH6 keeps its own bit rather than sharing one. */
                .vert_flags = (uint8_t)(
                      (c_vert_valid ? VERT_F_VALID : 0)
                    | (alt_engaged  ? VERT_F_ENGAGED : 0)
                    | (alt_want     ? VERT_F_WANTED : 0)
                    | ((tof_now != 0 && tof_now != 65535)
                                    ? VERT_F_TOF_OK : 0)
                    | (vel_engaged  ? VERT_F_VEL_ENG : 0)
                    | (vel_want     ? VERT_F_VEL_WANT : 0)
                    | ((packet.channels[CH_ALTHOLD] > ALTHOLD_ON)
                                    ? VERT_F_CH6 : 0)
                    | (thr_vcomp_on ? VERT_F_VCOMP : 0)),
                /* Control barometer, same arm-edge convention as alt_cm
                 * but against its own zero. 0 all session = no MS5611
                 * fitted. Read bit 4 of sensor_health to tell a held
                 * sample from a fresh one. */
                .baro2_cm = bb_sat16((c_baro2_alt_m - baro2_ref_m) * 100.0f),
                /* The fusion task's running velocity integral, passed
                 * through as-is. Logged ABSOLUTE so the reader can
                 * difference it - a delta would be wrong across every
                 * dropped or transport-lost frame. See blackbox.h. */
                .accz_dv_ms = c_accz_dv_ms,
                .accn_dv_ms = c_accn_dv_ms,
                .acce_dv_ms = c_acce_dv_ms,
                /* GPS channel. Decimetres and cm/s - see blackbox.h for
                 * why position is dm and not the cm used elsewhere. */
                .pos_n_dm = bb_sat16(c_pos_n_m * 10.0f),
                .pos_e_dm = bb_sat16(c_pos_e_m * 10.0f),
                .vel_n_cms = bb_sat16((float)c_vel_n_mms * 0.1f),
                .vel_e_cms = bb_sat16((float)c_vel_e_mms * 0.1f),
                /* Saturate rather than wrap: 65535 reads as "worse than
                 * 65 m, or no estimate", which is true either way. */
                .gps_h_acc_mm = (c_gps_h_acc_mm > 65535u)
                                ? 65535u : (uint16_t)c_gps_h_acc_mm,
                .sats = c_sats,
                .gps_flags = (uint8_t)((c_fix_type & GPS_F_FIX_MASK)
                    | (c_gps_fix_ok ? GPS_F_FIX_OK : 0)
                    | (c_home_valid ? GPS_F_HOME_VALID : 0)),
                .ctrl_flags = iterm_gate_flags(&itg),
            };
            blackbox_log(&bbf);

            // 6. Update Telemetry
            if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
                global_nav_state.m1_pwm = motor_out[0];
                global_nav_state.m2_pwm = motor_out[1];
                global_nav_state.m3_pwm = motor_out[2];
                global_nav_state.m4_pwm = motor_out[3];
                
                global_nav_state.des_roll = des_roll;
                global_nav_state.des_pitch = des_pitch;
                global_nav_state.des_yaw = des_yaw_rate;
                global_nav_state.loop_dt = dt_sec;
                global_nav_state.alt_sp_m   = alt_sp_m;
                global_nav_state.vert_flags = bbf.vert_flags;

                // IMPORTANT: Tell C6 that we are Armed
                global_nav_state.is_armed = 1;
                global_nav_state.arm_blocked = 0;

                // Dashboard: raw RC + the gains actually in use
                global_nav_state.rc_throttle = packet.channels[CH_THROTTLE];
                global_nav_state.rc_roll     = packet.channels[CH_ROLL];
                global_nav_state.rc_pitch    = packet.channels[CH_PITCH];
                global_nav_state.rc_yaw      = packet.channels[CH_YAW];
                global_nav_state.rc_arm      = packet.channels[CH_ARM];
                global_nav_state.rc_frozen_frames = rc_frozen_frames;
                global_nav_state.rc_link_ok  = rc_link_ok ? 1 : 0;
                global_nav_state.pid_active.kp_rate = rate_roll_cfg.kp;
                global_nav_state.pid_active.ki_rate = rate_roll_cfg.ki;
                global_nav_state.pid_active.kd_rate = rate_roll_cfg.kd;
                global_nav_state.pid_active.kp_yaw  = rate_yaw_cfg.kp;
                global_nav_state.pid_active.ki_yaw  = rate_yaw_cfg.ki;
                global_nav_state.pid_active.kd_yaw  = rate_yaw_cfg.kd;

                xSemaphoreGive(state_mutex);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void fc_tasks_init(void) {
    rc_queue = xQueueCreate(1, sizeof(rc_packet_t));
#if !HIL_MODE
    xTaskCreatePinnedToCore(rx_task,     "rx_task",     4096, NULL, 5, NULL,
                            FC_CORE_SUPPORT);
#endif
    xTaskCreatePinnedToCore(flight_task, "flight_task", 4096, NULL, 4, NULL,
                            FC_CORE_REALTIME);
}