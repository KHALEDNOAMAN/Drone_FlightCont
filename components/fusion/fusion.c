#include <math.h>
#include <stdio.h>
#include <string.h>
#include "fusion.h"
#include "fc_cores.h"
#include "biquad.h"
#include "vert_filter.h"
#include "horiz_filter.h"
#include "alt_ref.h"
#include "params.h"
#include "driver/gpio.h"
#include "telemetry.h"
#include "nav_state.h"
#include "perf_stats.h"
#include "lsm6dsox.h"
#include "dps310.h"
#include "ms5611.h"
#include "mmc5603.h"
#include "ina226.h"
#include "vl53l0x.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

static const char *TAG = "FUSION";

/* ---- Configuration ---- */
/* Loop rate. Now PACED BY THE IMU, not by a timer: the LSM6DSOX runs at
 * 416 Hz (lsm6dsox.c CTRL1_XL/CTRL2_G) and raises INT1 on every new gyro
 * sample. This must match that ODR - it sets the biquad sample rate and
 * the warmup length, and a wrong value silently detunes both. */
#define FUSION_RATE_HZ       416
#define IMU_DRDY_GPIO        15      /* LSM6DSOX INT1; moved 14 -> 15 in the
                                      * harness rebuild */
#define DRDY_MIN_INTERVAL_US 1800    /* reject physically impossible edges */
#define DRDY_TIMEOUT_MS      10      /* self-heal if an edge is ever missed */
#define BARO_MAG_DIVIDER     5       /* read baro+mag every 5th cycle = 50 Hz */
#define SEA_LEVEL_PRESSURE   101325.0f
#define MADGWICK_BETA        0.04f

/* ---- Gyro low-pass (P3) ----
 * 2nd-order Butterworth on each gyro axis before it reaches Madgwick,
 * the PID, or the log.
 *
 * Set to 0 to bypass entirely - REQUIRED for the FFT session, which must
 * see raw gyro to locate the frame resonance.
 *
 * 60 Hz is the MEASURED choice from the four-run sweep set
 * (sweep_a-d, cross-run analysis in tools/fft_analyze.py): the airframe
 * has a resonance cluster at 70-90 Hz (pitch arm mode 74.5 Hz, dominant
 * roll mode wandering 77-83 Hz at 800-1100x the noise floor). The old
 * 80 Hz placeholder sat its corner exactly ON that cluster - the one
 * placement the sweep data forbids. 60 gets under the cluster and takes
 * the 150-200 Hz 2/rev band down ~15-20 dB for free.
 *
 * Group delay is part of control latency (~0.225/fc at DC for a 2nd-
 * order Butterworth: 60 Hz -> ~3.7 ms) - do not go lower than the data
 * justifies. Aliasing is no longer a concern here: DRDY pacing runs the
 * loop at the sensor's own ~416 Hz, so the band is clean to ~208 Hz.
 *
 * P4: the VALUE lives in the params table (gyro_lpf_hz, default 60 in
 * params.c) - SET + SAVE + power-cycle to change it, no reflash. The
 * biquads are initialised once at task start, which is why filter
 * params are boot-apply. */
#define GYRO_LPF_HZ          (FFT_LOG ? 0.0f : g_params.gyro_lpf_hz)

/* ---- Gyro notch (P3, set from the same sweep set) ----
 * One WIDE band-stop on all three axes, centred in the 70-90 Hz mode
 * cluster: the 74.1-75.2 Hz pitch mode (tight across four runs incl. a
 * 90 deg remount) and the roll resonance whose centre wandered
 * 77.4-82.8 Hz run to run (strap tension / session). Wide and low-Q on
 * purpose: a Q15 notch is ~5 Hz wide and the roll peak moves more than
 * that, so it would walk straight out. Q5 at 78 Hz puts the -3 dB span
 * at roughly 70-87 Hz, covering both modes at every position observed.
 * PROVISIONAL: these are drone-plus-jig numbers. Re-centre from free-
 * flight blackbox spectra once P4 flies. Bypassed in FFT builds.
 * P4: values live in the params table (notch_hz / notch_q, defaults 78
 * and 5 in params.c) - boot-apply, same as the LPF. */
#define GYRO_NOTCH_HZ        (FFT_LOG ? 0.0f : g_params.notch_hz)
#define GYRO_NOTCH_Q         (g_params.notch_q)

/* ---- FFT capture build (P3) ----
 * Set to 1, rebuild, flash, run the motor sweep, then set back to 0.
 *
 * Does two things, and BOTH are required:
 *   1. Bypasses the gyro LPF - it would erase the very peaks the sweep
 *      is hunting.
 *   2. Streams gyro at the FULL loop rate on a "G," line.
 *
 * (2) is the one that is easy to miss. The normal D, stream runs at
 * 50 Hz (debug_stream.c STREAM_RATE_HZ), so its Nyquist is 25 Hz, while
 * frame resonances live at 100-500 Hz. Doing an FFT on D, data would
 * alias everything and produce notch frequencies for peaks that do not
 * exist. At ~416 Hz the usable band is DC-208 Hz, which is what the DRDY
 * work bought us.
 *
 * The fusion loop must never printf: a USB CDC write blocks when the
 * host is not reading, and that would blow the 2364 us budget. So it
 * pushes into a lock-free ring and a low-priority task on core 0 drains
 * it. Drops are counted and reported rather than silently losing
 * samples - a gap in the series is an artefact in the spectrum. */
#define FFT_LOG              0

/* 1 = send the G, stream out the telemetry UART to the C6 WiFi bridge
 *     (TCP <C6 ip>:5555; 192.168.4.1 only in the C6's AP mode). Needed for the sweep: props spinning means
 *     you want distance from the airframe, not a USB tether.
 * 0 = printf to the USB console instead.
 * Not both: a USB write blocks when no host is reading, which would
 * stall the drain task and cost samples. */
#define FFT_LOG_TO_UART      1

/* ---- Sensor handles ---- */
static lsm6dsox_handle_t *imu_dev  = NULL;
static mmc5603_handle_t  *mag_dev  = NULL;

/* Written by baro_retry_task, read by fusion_task -> volatile so the read
 * in the hot loop is not hoisted out. A 32-bit aligned pointer store is
 * atomic on Xtensa, and it only ever goes NULL -> valid, never back, so no
 * lock is needed: fusion either sees the old NULL and skips the baro for
 * one more cycle, or sees a fully-initialised handle. */
static dps310_handle_t *volatile baro_dev = NULL;

/* The MS5611 control barometer at 0x76. Plain, not volatile, and with no
 * retry task: it is init-once at boot and NULL forever if the part is not
 * fitted, which is the normal state until it is ordered. Nothing depends
 * on it, so a missing control must cost exactly nothing. */
static ms5611_handle_t *baro2_dev = NULL;

/* ---- Quaternion state ---- */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* ---- Madgwick gain (runtime) ----
 * At beta 0.04 a large initial attitude error takes MINUTES to converge -
 * 156 deg of apparent "drift" was once measured over 2 min on the bench
 * that was really just slow convergence from q = identity.
 *
 * Two fixes, both P3:
 *   1. seed_attitude_from_sensors() starts the quaternion AT the measured
 *      attitude instead of identity, so there is no large error to burn
 *      off in the first place;
 *   2. beta decays GEOMETRICALLY from 2.5 to 0.04 over 2 s instead of
 *      sitting high for 4 s and then stepping. A step leaves the filter
 *      abruptly 60x less responsive mid-flight-prep; a smooth ramp does
 *      not, and it reaches trust-the-gyro sooner.
 *
 * The per-cycle multiplier is computed once at task start:
 *   ratio = (BETA_NORMAL / BETA_WARMUP) ^ (1 / WARMUP_CYCLES)
 * so g_beta *= ratio each cycle costs one multiply. */
#define MADGWICK_BETA_WARMUP   2.5f
#define WARMUP_CYCLES          (FUSION_RATE_HZ * 2)   /* 2 s, whatever the rate */
static float g_beta = MADGWICK_BETA_WARMUP;

/* ---- Madgwick AHRS (9-DOF) ---- */
static float inv_sqrt(float x)
{
    /* Fast inverse square root — good enough for AHRS, avoids sqrtf + divf */
    float half = 0.5f * x;
    int32_t i;
    memcpy(&i, &x, 4);
    i = 0x5F3759DF - (i >> 1);
    memcpy(&x, &i, 4);
    x *= (1.5f - half * x * x);
    return x;
}

static void madgwick_update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float mx, float my, float mz,
                            float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float beta = g_beta;

    /* Convert gyro from dps to rad/s */
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* Compute feedback only if accelerometer measurement valid */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Normalise magnetometer measurement */
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        /* Auxiliary variables */
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        /* Reference direction of Earth's magnetic field */
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
           + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2
           - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3
             - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent corrective step */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
           + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
           - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
           + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
           - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
           + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
           + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
           - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
           + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
           + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
           + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        /* Normalise step magnitude */
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply feedback step */
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    /* Integrate rate of change of quaternion */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* Normalise quaternion */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/* 6-DOF fallback (no mag) */
static void madgwick_update_imu(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;
    float beta = g_beta;

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0; qDot2 -= beta * s1;
        qDot3 -= beta * s2; qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt; q1 += qDot2 * dt;
    q2 += qDot3 * dt; q3 += qDot4 * dt;

    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

/* ---- Quaternion → Euler (ZYX convention, degrees) ---- */
static void quat_to_euler(float *roll, float *pitch, float *yaw)
{
    /* Roll (X) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 57.2957795f;

    /* Pitch (Y) — clamp to avoid NaN */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * 57.2957795f;

    /* Yaw (Z) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;
}

/* ---- Baro runtime recovery ----
 *
 * The DPS310 is the one sensor that can be alive at power-on and not
 * ready after a warm reboot: its coefficients may not be loaded yet, and
 * dps310_init REFUSES it then (COEF_RDY check) instead of
 * accepting garbage. So keep re-initialising until it answers properly -
 * normally within one or two retries; a part that never recovers needs a
 * power cycle.
 *
 * Runs only while DISARMED. A failed dps310_init takes ~350 ms of
 * retries and interleaves ~17 extra I2C transactions with the fusion
 * loop's own 50 Hz baro+mag block; that contention is fine on the bench
 * and unwanted in the air. The delays inside dps310_init are vTaskDelay,
 * so the bus mutex is released between transactions - the fusion loop
 * sees brief contention, not a 350 ms stall.
 *
 * Self-deletes once the baro is back; there is nothing to retry after. */
#define BARO_RETRY_PERIOD_MS  10000
#define BARO_RETRY_TASK_PRIO  1        /* below flight(4)/fusion, above idle */

static i2c_master_bus_handle_t s_bus_handle = NULL;

static void baro_retry_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(BARO_RETRY_PERIOD_MS));

        if (global_nav_state.is_armed) continue;   /* never poke the bus in flight */

        dps310_handle_t *dev = dps310_init(s_bus_handle);
        if (dev) {
            baro_dev = dev;
            ESP_LOGW(TAG, "DPS310 recovered on retry - altitude is live again");
            break;
        }
    }
    vTaskDelete(NULL);
}


#if FFT_LOG
/* Single producer (fusion), single consumer (fft_log_task) -> the head
 * and tail indices need no lock, only ordering. */
#define FFT_RING_N   2048

typedef struct { uint32_t t_us; float gx, gy, gz; } fft_sample_t;

static fft_sample_t     s_fft_ring[FFT_RING_N];
static volatile uint32_t s_fft_head = 0;   /* written by fusion */
static volatile uint32_t s_fft_tail = 0;   /* written by drain task */
static volatile uint32_t s_fft_drops = 0;

static inline void fft_push(int64_t t_us, float gx, float gy, float gz)
{
    uint32_t head = s_fft_head;
    uint32_t next = (head + 1) % FFT_RING_N;
    if (next == s_fft_tail) {          /* full - never block the loop */
        s_fft_drops++;
        return;
    }
    s_fft_ring[head].t_us = (uint32_t)t_us;
    s_fft_ring[head].gx   = gx;
    s_fft_ring[head].gy   = gy;
    s_fft_ring[head].gz   = gz;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    s_fft_head = next;
}

static void fft_log_task(void *arg)
{
    (void)arg;
    uint32_t last_report = 0;
    for (;;) {
        while (s_fft_tail != s_fft_head) {
            uint32_t tail = s_fft_tail;
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            const fft_sample_t *e = &s_fft_ring[tail];
            char line[64];
            int n = snprintf(line, sizeof(line), "G,%lu,%.3f,%.3f,%.3f\n",
                             (unsigned long)e->t_us,
                             (double)e->gx, (double)e->gy, (double)e->gz);
#if FFT_LOG_TO_UART
            telemetry_write_raw(line, n);
#else
            fwrite(line, 1, n, stdout);
#endif
            s_fft_tail = (tail + 1) % FFT_RING_N;
        }
        /* Report drops once a second - a gap in the series shows up as a
         * spurious broadband floor in the spectrum, so it must not pass
         * unnoticed. */
        uint32_t d = s_fft_drops;
        if (d != last_report) {
            ESP_LOGW(TAG, "FFT log dropped %lu samples", (unsigned long)d);
            last_report = d;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif /* FFT_LOG */

/* ---- IMU data-ready interrupt (P3) ----
 *
 * The loop used to run on vTaskDelayUntil at 250 Hz while the sensor
 * produced data at 416 Hz. Two bugs in one:
 *
 *   Aliasing. Our Nyquist was 125 Hz, the sensor emits to 208 Hz, so
 *   125-208 Hz folded down into the band at the wrong frequency and no
 *   filter could remove it. Sampling at the true ODR eliminates it.
 *
 *   Jitter. dt came from the scheduler, and each sample was 0-4 ms stale
 *   by an unpredictable amount. Both errors went straight into the
 *   integration. Now dt is the interval between two hardware edges, from
 *   the sensor's own clock.
 *
 * The ISR does the minimum: stamp the time, wake the task. */
static TaskHandle_t     s_fusion_task = NULL;
static volatile int64_t s_drdy_us     = 0;
static portMUX_TYPE     s_drdy_mux    = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_drdy_rejects = 0;   /* impossible-interval edges */

static void IRAM_ATTR imu_drdy_isr(void *arg)
{
    (void)arg;
    /* A 416 Hz sensor cannot produce data every 10 us. Edges closer
     * together than DRDY_MIN_INTERVAL_US are electrical noise on the
     * INT1 line, not data-ready, and acting on them makes the loop
     * re-read the same sample several times and overrun its period.
     * Observed with a marginal INT1 joint: 1500+ Hz of edges,
     * 75% of consecutive samples byte-identical. Counted, not hidden. */
    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&s_drdy_mux);
    bool too_soon = (now - s_drdy_us) < DRDY_MIN_INTERVAL_US;
    if (too_soon) s_drdy_rejects++;
    else          s_drdy_us = now;
    portEXIT_CRITICAL_ISR(&s_drdy_mux);

    if (too_soon) return;

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(s_fusion_task, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}

/* 64-bit read is not atomic on a 32-bit core, and the ISR can land
 * mid-read - so take it under the same lock the ISR uses. */
static inline int64_t drdy_stamp(void)
{
    int64_t t;
    portENTER_CRITICAL(&s_drdy_mux);
    t = s_drdy_us;
    portEXIT_CRITICAL(&s_drdy_mux);
    return t;
}

static void imu_drdy_init(void)
{
    s_fusion_task = xTaskGetCurrentTaskHandle();

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << IMU_DRDY_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,   /* push-pull output, no pull needed */
        /* Pull-down, not floating: an open INT1 wire otherwise picks up
         * noise and fabricates interrupts. Held low it makes none, and
         * the DRDY timeout reports the fault honestly. */
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* Another component may already have installed the GPIO ISR service,
     * in which case this returns ESP_ERR_INVALID_STATE - which is success
     * for us, not a failure. */
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(IMU_DRDY_GPIO, imu_drdy_isr, NULL));

    ESP_LOGI(TAG, "IMU DRDY on GPIO %d, loop paced at %d Hz",
             IMU_DRDY_GPIO, FUSION_RATE_HZ);
}

/* ---- Attitude seeding (P3) ----
 *
 * Start the quaternion at the attitude the sensors actually report, not
 * at identity. Gravity fixes roll and pitch outright; the mag, tilt-
 * compensated by that roll/pitch, fixes yaw. One sample is enough - the
 * board is stationary on the bench at this point, and Madgwick refines
 * from here anyway.
 *
 * Without this the filter boots believing it is perfectly level and
 * north-facing, and with beta at flight value it needs minutes to walk
 * to the truth. That was misread as gyro drift once already.
 *
 * Falls back gracefully: no mag -> seed roll/pitch only, leave yaw 0; no
 * accel -> leave the identity quaternion and let warmup do its old job. */
static void seed_attitude_from_sensors(void)
{
    lsm6dsox_data_t imu;
    if (!imu_dev || lsm6dsox_read(imu_dev, &imu) != ESP_OK) {
        ESP_LOGW(TAG, "Attitude seed skipped - no IMU sample");
        return;
    }

    /* Accelerometer -> roll, pitch. Stationary, so the measured specific
     * force IS gravity. */
    float roll  = atan2f(imu.ay, imu.az);
    float pitch = atan2f(-imu.ax, sqrtf(imu.ay * imu.ay + imu.az * imu.az));
    float yaw   = 0.0f;

    /* Magnetometer -> yaw, de-tilted into the horizontal plane. Reuses
     * the same 10..120 uT sanity gate the runtime path applies. */
    mmc5603_data_t m;
    if (mag_dev && mmc5603_read(mag_dev, &m) == ESP_OK) {
        float m2 = m.mx * m.mx + m.my * m.my + m.mz * m.mz;
        if (m2 > 100.0f && m2 < 14400.0f) {
            float sr = sinf(roll),  cr = cosf(roll);
            float sp = sinf(pitch), cp = cosf(pitch);
            float mx_h = m.mx * cp + m.my * sr * sp + m.mz * cr * sp;
            float my_h = m.my * cr - m.mz * sr;
            yaw = atan2f(-my_h, mx_h);
        } else {
            ESP_LOGW(TAG, "Attitude seed: mag outside gate, yaw starts at 0");
        }
    }

    /* ZYX Euler -> quaternion. Inverse of quat_to_euler() above; keep the
     * two in step if either convention ever changes. */
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
    float cp2 = cosf(pitch * 0.5f), sp2 = sinf(pitch * 0.5f);
    float cr2 = cosf(roll  * 0.5f), sr2 = sinf(roll  * 0.5f);

    q0 = cr2 * cp2 * cy + sr2 * sp2 * sy;
    q1 = sr2 * cp2 * cy - cr2 * sp2 * sy;
    q2 = cr2 * sp2 * cy + sr2 * cp2 * sy;
    q3 = cr2 * cp2 * sy - sr2 * sp2 * cy;

    ESP_LOGI(TAG, "Attitude seeded: roll %.2f pitch %.2f yaw %.2f deg",
             (double)(roll  * 57.2957795f),
             (double)(pitch * 57.2957795f),
             (double)(yaw   * 57.2957795f));
}

/* ---- Slow sensor snapshot: baro + mag, read at 50 Hz on core 0 ----
 *
 * These two live on I2C at 100 kHz and cost 2383 us per read - 81% of the
 * fusion loop's 2944 us worst tick, against a 4000 us budget, while the
 * IMU read and the Madgwick update together are 82 us. Doing them inside
 * the 250 Hz loop meant every 5th cycle nearly blew the budget for data
 * that only changes at 50 Hz anyway.
 *
 * So slow_sensor_task owns both devices, reads them on its own 50 Hz
 * cadence on core 0, and publishes a snapshot. The fusion loop just picks
 * up the latest one - no I2C, no blocking, no mutex.
 *
 * Handover is a seqlock rather than a mutex: the fusion task must never
 * block on a lower-priority task holding a sensor, and a ~50-byte struct
 * copy is far cheaper than a semaphore round trip. Writer bumps the
 * counter to odd, writes, bumps it to even; reader retries while the
 * counter is odd or changed underneath it. Single writer, so the
 * non-atomic increment is safe.
 *
 * The reader gives up after SLOW_SNAP_RETRIES and keeps the values it
 * already had. That bounds the hot loop: a writer preempted mid-publish
 * on core 0 can never spin core 1. Losing one 20 ms sample is invisible
 * to Madgwick at beta 0.04.
 *
 * Staleness: fusion now sees data up to one 20 ms period old. Irrelevant
 * for altitude, and the mag only trims yaw slowly - it was already being
 * sampled at 1/5 the loop rate before this change. */
typedef struct {
    mmc5603_data_t mag;
    float baro_temp_c;
    float baro_press_pa;
    bool  mag_read_ok;
    bool  mag_valid;
    bool  baro_ok;
    /* Second barometer - a control, carried alongside and never blended.
     * Blending it would destroy the experiment it exists for: if the two
     * disagree we must be able to see WHICH one drifted. */
    float baro2_temp_c;
    float baro2_press_pa;
    bool  baro2_ok;
    /* Bumped on every SUCCESSFUL baro read. Same reason as
     * vl53l0x_sample_seq(): the vertical filter runs at the 416 Hz fusion
     * rate against a 50 Hz barometer, and applying one pressure sample
     * eight times over would tell the Kalman update it had eight
     * independent observations of it. Comparing pressure values instead would not work -
     * a genuinely unchanged pressure is the sitting-still case. */
    uint32_t baro_seq;
} slow_sample_t;

#define SLOW_SENSOR_RATE_HZ  50
#define SLOW_SENSOR_PRIO     3     /* below fusion(5)/flight(4), above gps(2) */
#define SLOW_SNAP_RETRIES    4

/* Pack voltage rides this task because it is the same 100 kHz bus, and
 * this task is its owner. 5 Hz: the INA226 averages 16 conversions and
 * only produces a new number every ~35 ms, and the 4 Hz telemetry CSV is
 * the fastest consumer - sampling faster would spend bus time on
 * duplicate values. */
#define VBAT_DIVIDER         (SLOW_SENSOR_RATE_HZ / 5)

/* Ground distance rides the same bus and the same owner. Every 5th tick,
 * minus the ticks the INA226 takes (see the staggering below), so ~9 Hz.
 *
 * That is the POLL rate and it is deliberately faster than the sensor:
 * the VL53L0X finishes a measurement every tof_budget_ms (200 ms on this
 * airframe = ~5 Hz DELIVERED), so most polls return "nothing new yet"
 * and the driver's seq counter does not move. Over-polling costs one
 * 1-byte status read plus one 12-byte result read on the 100 kHz bus and
 * buys minimum latency on the samples that do arrive.
 *
 * Do not read ~9 Hz as the rate anything downstream RECEIVES at - that
 * confusion is what left VF_TOF_REJECT_STREAK timing 4 s while its
 * comment claimed 2.2. */
#define TOF_DIVIDER          (SLOW_SENSOR_RATE_HZ / 10)

/* Mag calibration capture ("M," line out the telemetry UART). Written by
 * the command interpreter, read by slow_sensor_task -> volatile. See
 * fusion_set_mag_capture in fusion.h for what it is for. */
static volatile bool s_mag_capture = false;

static volatile uint32_t s_slow_seq = 0;   /* even = stable, odd = mid-write */
static slow_sample_t     s_slow_pub;

/* Returns false if the writer kept moving under us - caller keeps its
 * previous values rather than spinning. */
static bool slow_sample_get(slow_sample_t *out)
{
    for (int i = 0; i < SLOW_SNAP_RETRIES; i++) {
        uint32_t before = s_slow_seq;
        if (before & 1u) continue;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        *out = s_slow_pub;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (s_slow_seq == before) return true;
    }
    return false;
}

/* ---- Raw-pressure plausibility gate ----
 *
 * A successful I2C transaction says NOTHING about the data, and this
 * barometer is the project's own proof: blackbox s89  has
 * the DPS310 returning ESP_OK with pressures whose implied altitudes sit
 * 655 m apart on consecutive samples, while the aircraft hovered at
 * 15 cm. The vertical filter has its own gates, but a filter can only
 * weigh a sample against what it already believes - it cannot know that
 * a 24-bit two's-complement read came back with a flipped sign bit.
 * That is knowable HERE, from physics, without any state to be wrong.
 *
 * Deliberately at the single choke point that feeds both the filter and
 * SENSOR_BARO_OK, so a corrupt sample also shows up as an unhealthy
 * baro - in the blackbox, in telemetry, in the pre-arm check and in
 * alt-hold's ALT_BARO_GRACE_CYCLES. A sensor returning garbage IS an
 * unhealthy sensor, and every consumer already knows what to do with
 * one. Nothing here silently substitutes a value; a rejected sample
 * leaves the published pressure exactly as a failed read does.
 *
 * An ABSOLUTE band cannot do this job alone and it is worth saying why,
 * because it was the obvious first idea: s89's corrupt altitudes of
 * +-327 m correspond to ~97 and ~105 kPa, both of them perfectly
 * ordinary pressures. Only the RATE gives it away - 8 kPa between
 * samples 20 ms apart is roughly 700 m of altitude, which nothing this
 * aircraft can do. */
#define BARO_MIN_PA          30000.0f   /* ~9000 m. Above this is space  */
#define BARO_MAX_PA         110000.0f   /* below sea level               */
/* ~12 Pa per metre near sea level, times 20 m/s - far past anything
 * this airframe climbs or dives at, so a genuine flight never trips it. */
#define BARO_MAX_RATE_PA_S     240.0f
#define BARO_JUMP_MARGIN_PA    200.0f   /* ~16 m of slack over the rate  */
/* After this long with nothing accepted, re-seed the reference so a
 * genuine step - or the chip coming back via baro_retry_task with the
 * weather since moved on - is not locked out for the rest of the
 * flight. The re-seeded sample is NOT published: re-seeding restores
 * the ability to judge, it does not vouch for the sample that did it. */
#define BARO_RESEED_PASSES     (SLOW_SENSOR_RATE_HZ * 2)

/* gap_passes: slow-task passes since the last ACCEPTED sample, counted
 * on every pass including failed reads - the elapsed TIME is what sets
 * how far the pressure is allowed to have moved, and a read that failed
 * still took 20 ms. */
static bool baro_sample_plausible(float p_pa, uint32_t gap_passes)
{
    static float last_pa = 0.0f;

    if (!(p_pa > BARO_MIN_PA && p_pa < BARO_MAX_PA)) return false;

    if (last_pa <= 0.0f) {          /* first sample: nothing to judge against */
        last_pa = p_pa;
        return true;
    }

    const float dt = (float)gap_passes / (float)SLOW_SENSOR_RATE_HZ;
    const float allowed = BARO_MAX_RATE_PA_S * dt + BARO_JUMP_MARGIN_PA;
    const float d = (p_pa > last_pa) ? (p_pa - last_pa) : (last_pa - p_pa);

    if (d <= allowed) {
        last_pa = p_pa;
        return true;
    }
    if (gap_passes > BARO_RESEED_PASSES) last_pa = p_pa;
    return false;
}

static void slow_sensor_task(void *arg)
{
    (void)arg;
    slow_sample_t s = {0};
    s.baro_temp_c = 25.0f;
    uint32_t vbat_div = 0;
    uint32_t tof_div  = 0;
    uint32_t baro_gap_passes = 0;   /* see baro_sample_plausible */

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1000 / SLOW_SENSOR_RATE_HZ));

        /* baro_dev is volatile and may be filled in later by
         * baro_retry_task - re-check it every pass, never cache it. */
        if (baro_dev) {
            /* Read into locals, not into the published sample: a read
             * that fails OR comes back implausible must leave the last
             * good pressure standing, exactly as a failed read always
             * has. baro_ok is what tells the consumer which it is. */
            float t_c = s.baro_temp_c, p_pa = 0.0f;
            const bool read_ok =
                (dps310_read(baro_dev, &t_c, &p_pa) == ESP_OK);
            baro_gap_passes++;
            if (read_ok && baro_sample_plausible(p_pa, baro_gap_passes)) {
                s.baro_temp_c   = t_c;
                s.baro_press_pa = p_pa;
                s.baro_ok       = true;
                s.baro_seq++;
                baro_gap_passes = 0;
            } else {
                s.baro_ok = false;
            }
        }

        /* Second barometer, read straight through with no gating. It is
         * deliberately NOT put through baro_sample_plausible(): that gate
         * exists to protect the vertical filter, nothing consumes this
         * part, and a gate here would hide exactly the dropouts the
         * experiment is looking for. ms5611_read() already refuses a
         * reading outside the part's own specified range. */
        if (baro2_dev) {
            float t2 = 0.0f, p2 = 0.0f;
            const esp_err_t e2 = ms5611_poll(baro2_dev, &t2, &p2);
            if (e2 == ESP_OK) {
                s.baro2_temp_c   = t2;
                s.baro2_press_pa = p2;
                s.baro2_ok       = true;
            } else if (e2 != ESP_ERR_NOT_FINISHED) {
                /* A conversion still in flight is the normal case on most
                 * passes and is NOT a failure - only a real error clears
                 * the health bit, or the chip would read as dead for half
                 * of every second. */
                s.baro2_ok = false;
            }
        }

        /* On a failed read mmc5603_read leaves the struct untouched, so
         * `s.mag` keeps the previous sample - which is exactly why
         * mag_read_ok is published separately. Without it a dead mag is
         * indistinguishable from a mag reading a constant field. */
        s.mag_read_ok = mag_dev && (mmc5603_read(mag_dev, &s.mag) == ESP_OK);

        /* Calibration capture. Emitted before the validity gate below on
         * purpose: an uncalibrated mag inside a frame full of hard iron
         * routinely reads outside the 10-120 uT window, and those are the
         * samples the ellipsoid fit needs most. Only a failed READ is
         * skipped - mmc5603_read leaves the struct untouched on error, so
         * emitting there would silently duplicate the previous sample and
         * bias the fit toward whatever orientation was held during the
         * glitch.
         *
         * Safe to call from this task: uart_write_bytes serialises on the
         * driver's tx_mux, so it cannot interleave with the 4 Hz CSV, the
         * "#" command replies, or an FFT "G," flood, and the fusion loop
         * reads through a seqlock so it never waits on this task anyway. */
        if (s_mag_capture && s.mag_read_ok) {
            char line[64];
            int n = snprintf(line, sizeof(line), "M,%lld,%.2f,%.2f,%.2f\n",
                             (long long)esp_timer_get_time(),
                             (double)s.mag.mx, (double)s.mag.my,
                             (double)s.mag.mz);
            if (n > 0) telemetry_write_raw(line, n);
        }

        if (s.mag_read_ok) {
            /* Magnitude gate: Earth's field is ~25-65 uT. Reject
             * zero/stuck/disturbed samples (motor current, hard iron
             * nearby) so they cannot poison the 9-DOF update - the norm of
             * a zero vector would divide by zero and NaN the quaternion.
             * Gate: 10..120 uT (squared: 100..14400). */
            float m2 = s.mag.mx * s.mag.mx + s.mag.my * s.mag.my
                     + s.mag.mz * s.mag.mz;
            s.mag_valid = (m2 > 100.0f) && (m2 < 14400.0f);
        } else {
            s.mag_valid = false;
        }

        s_slow_seq++;                                  /* -> odd */
        __atomic_thread_fence(__ATOMIC_RELEASE);
        s_slow_pub = s;
        __atomic_thread_fence(__ATOMIC_RELEASE);
        s_slow_seq++;                                  /* -> even */

        /* AFTER the publish, never before: pack voltage and ground
         * distance are dashboard numbers, the seqlock feeds the 416 Hz
         * attitude loop, and an extra I2C transaction ahead of it would
         * delay real flight data. Both are no-ops when the chip is
         * absent. Staggered onto different ticks so the two never land
         * in the same 20 ms slot as each other. */
        if (++vbat_div >= VBAT_DIVIDER) {
            vbat_div = 0;
            ina226_sample();
        } else if (++tof_div >= TOF_DIVIDER) {
            tof_div = 0;
            vl53l0x_sample();
        }
    }
}

/* ---- Public API ---- */

esp_err_t fusion_init_sensors(i2c_master_bus_handle_t bus_handle,
                              i2c_master_bus_handle_t aux_handle)
{
    ESP_LOGI(TAG, "Initializing sensors...");

    /* LSM6DSOX — SPI (self-contained, no bus_handle needed) */
    imu_dev = lsm6dsox_init();
    if (!imu_dev) {
        ESP_LOGE(TAG, "LSM6DSOX init failed");
        return ESP_FAIL;
    }
    /* The driver's refusal dump reads this pin's level: HIGH = the chip
     * is still sampling and data is waiting, LOW = it stopped. */
    imu_dev->int1_gpio = IMU_DRDY_GPIO;

    /* DPS310 — I2C. Optional for BOOTING: attitude (Madgwick) only needs
     * IMU+mag; altitude reads 0 without it, so fusion runs. NOT optional
     * for FLYING: SENSOR_BARO_OK is in PREARM_SENSORS_REQUIRED, so a
     * missing baro blocks arming. */
    s_bus_handle = bus_handle;          /* kept for baro_retry_task */

    /* Second barometer, the control for the DPS310 investigation. Absent
     * until the part is ordered and fitted, and its absence is logged at
     * INFO rather than WARN - a control that is not installed is not a
     * fault. It lives on bus B, away from the DPS310 on bus A, so either of
     * its addresses (0x76 CSB high / 0x77 CSB low) is free there and
     * ms5611_init probes both - see ms5611.h. */
    baro2_dev = ms5611_init(aux_handle, MS5611_OSR_4096);
    if (!baro2_dev) {
        ESP_LOGI(TAG, "MS5611 not present on bus B - control baro absent, "
                      "flight unaffected");
    }

    baro_dev = dps310_init(bus_handle);
    if (!baro_dev) {
        ESP_LOGW(TAG, "DPS310 init failed - continuing WITHOUT baro (altitude = 0)");
        /* Keep trying in the background. After a warm S3 reset (watchdog
         * panic, brownout, host reset) the DPS310 can come up with its
         * coefficients not yet loaded; dps310_init refuses it then and this
         * retry picks it up once it is ready (dps310.c, fixed).
         * Without the retry a reboot would leave the aircraft with no
         * altitude until someone noticed. */
        xTaskCreatePinnedToCore(baro_retry_task, "baro_retry", 3072, NULL,
                                BARO_RETRY_TASK_PRIO, NULL, FC_CORE_SUPPORT);
    }

    /* MMC5603 — I2C. Optional for BOOTING: on failure fusion falls back to
     * 6-DOF (roll/pitch fine, yaw drifts). NOT optional for FLYING:
     * SENSOR_MAG_OK is in PREARM_SENSORS_REQUIRED, so a dead mag blocks
     * arming. */

    /* BUS B (0x30, no collision with 0x40 / 0x76-0x77). */
    mag_dev = mmc5603_init(aux_handle);
    if (!mag_dev) {
        ESP_LOGW(TAG, "MMC5603 init failed - continuing WITHOUT mag (6-DOF, yaw drifts)");
    } else {
        /* Load magnetometer calibration from NVS */
        mmc5603_load_cal(mag_dev);

        /* Field-health check: log |B| once so per-boot sensor state is
         * visible. Earth is ~25-65 uT; far outside that = distorted
         * environment or degraded sensing film. */
        vTaskDelay(pdMS_TO_TICKS(30));      /* first continuous sample */
        mmc5603_data_t m0;
        if (mmc5603_read(mag_dev, &m0) == ESP_OK) {
            float b = sqrtf(m0.mx * m0.mx + m0.my * m0.my + m0.mz * m0.mz);
            if (b < 25.0f || b > 65.0f) {
                ESP_LOGW(TAG, "Mag field |B| = %.1f uT (Earth ~25-65) - "
                         "distorted or degraded, yaw accuracy reduced", (double)b);
            } else {
                ESP_LOGI(TAG, "Mag field |B| = %.1f uT (healthy)", (double)b);
            }
        }
    }

    /* Seed BEFORE slow_sensor_task exists so this is the only owner of
     * the mag right now - no seqlock dance needed for one sample. */
    seed_attitude_from_sensors();

#if !HIL_MODE
    /* Owns baro+mag from here on. Started even if both failed: baro_dev
     * can appear later via baro_retry_task, and the task simply publishes
     * baro_ok/mag_read_ok = false until then. */
    xTaskCreatePinnedToCore(slow_sensor_task, "slow_sensor", 4096, NULL,
                            SLOW_SENSOR_PRIO, NULL, FC_CORE_SUPPORT);
#endif

    ESP_LOGI(TAG, "All sensors initialized");
    return ESP_OK;
}

esp_err_t fusion_set_mag_cal(const float off[3], const float S[9])
{
    if (!mag_dev) return ESP_ERR_INVALID_STATE;
    return mmc5603_save_cal(mag_dev, off, S);
}

static volatile bool gyro_recal_requested = false;

void fusion_request_gyro_recal(void)
{
    gyro_recal_requested = true;
}

/* ---- Level calibration (LEVELCAL) ----
 *
 * Requested from the command task, executed in fusion_task so the IMU
 * keeps a single owner (same rule as GYROCAL). The result travels back
 * through a small struct + a done flag, because the console reply is the
 * ONLY way the user sees it: the sensor rail is battery-fed, so on the
 * bench where this runs the USB log shows nothing at all. */
#define LEVELCAL_SAMPLES     500      /* ~1 s at the 2 ms sample delay */
#define LEVELCAL_STILL_STD_G 0.05f    /* still board is ~0.005 g */
#define LEVELCAL_MAG_TOL_G   0.10f    /* |a| must be 1 g +- this */

static volatile bool      level_cal_requested = false;
static volatile bool      level_cal_done      = false;
static fusion_levelcal_t  level_cal_result;

void fusion_request_level_cal(void)
{
    level_cal_done      = false;
    level_cal_requested = true;      /* enable last */
}

bool fusion_level_cal_done(fusion_levelcal_t *out)
{
    if (!level_cal_done) return false;
    if (out) *out = level_cal_result;
    return true;
}

/* Runs inside fusion_task with the watchdog detached. Averages accel,
 * gates on motion and on |a|, and writes the two params on success. */
static void run_level_cal(void)
{
    fusion_levelcal_t r = {0};
    double sum[3] = {0}, sumsq[3] = {0};
    int n = 0;

    for (int i = 0; i < LEVELCAL_SAMPLES; i++) {
        lsm6dsox_data_t s;
        if (imu_dev && lsm6dsox_read(imu_dev, &s) == ESP_OK) {
            const float a[3] = { s.ax, s.ay, s.az };
            for (int k = 0; k < 3; k++) {
                /* Accumulate in double, explicitly. Not caution: the
                 * variance below is the cancellation-prone
                 * E[x^2] - E[x]^2 form, and on the Z axis both terms are
                 * ~1.0 while their difference is ~2.5e-5. Float carries
                 * about two significant digits into that subtraction.
                 * The gyro cal gets away with float only because its
                 * samples sit near zero. */
                sum[k]   += (double)a[k];
                sumsq[k] += (double)a[k] * (double)a[k];
            }
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (n < LEVELCAL_SAMPLES / 2) {
        r.why = "IMU reads failing";
        goto finish;
    }

    double dmean[3];
    float  mean[3];
    for (int k = 0; k < 3; k++) {
        dmean[k] = sum[k] / n;
        mean[k]  = (float)dmean[k];
    }
    for (int k = 0; k < 3; k++) {
        double var = sumsq[k] / n - dmean[k] * dmean[k];
        float  sd  = var > 0.0 ? sqrtf((float)var) : 0.0f;
        if (sd > r.worst_std_g) r.worst_std_g = sd;
    }
    r.mag_g = sqrtf(mean[0]*mean[0] + mean[1]*mean[1] + mean[2]*mean[2]);

    /* Gate 1: the board moved. An offset captured while someone is
     * holding the frame is a lie that then defines "level" forever. */
    if (r.worst_std_g > LEVELCAL_STILL_STD_G) {
        r.why = "board moving";
        goto finish;
    }
    /* Gate 2: the average is not gravity. Catches a dead or half-read
     * accel before its garbage becomes the level reference. */
    if (fabsf(r.mag_g - 1.0f) > LEVELCAL_MAG_TOL_G) {
        r.why = "|a| not 1 g";
        goto finish;
    }

    /* Same convention as seed_attitude_from_sensors - keep them in step. */
    r.roll_deg  = atan2f(mean[1], mean[2]) * 57.2957795f;
    r.pitch_deg = atan2f(-mean[0], sqrtf(mean[1]*mean[1] + mean[2]*mean[2]))
                * 57.2957795f;

    /* Gate 3: params_set would reject these anyway, but a named reason
     * beats a silent no-op. Past 10 deg the board is not slightly
     * crooked - it is mounted wrong, or the surface was not level. */
    if (params_set("level_roll_deg",  r.roll_deg)  != 0 ||
        params_set("level_pitch_deg", r.pitch_deg) != 0) {
        r.why = "offset > 10 deg - check the mount, not the number";
        goto finish;
    }
    r.applied = true;

finish:
    level_cal_result = r;
    level_cal_done   = true;
}

/* ---- Applying the level reference ----
 *
 * v_level = R(roll0, pitch0) * v_sensor, where R is the ZYX body->earth
 * matrix at yaw 0. That is exactly right and not an approximation: at
 * rest the measured accel IS the third row of R, and R times its own
 * third row is (0,0,1) identically. So a physically level board reads
 * dead level after this, by construction.
 *
 * Rebuilt only when the params change - four transcendentals are cheap
 * against a ~2.4 ms loop, but recomputing an unchanged constant 416
 * times a second is still silly. Both floats are written atomically by
 * the single params writer, so the compare is safe without a lock. */
typedef struct {
    float m[9];          /* row-major R */
    bool  identity;      /* both offsets zero - skip the rotation */
    float last_roll_deg, last_pitch_deg;
} level_rot_t;

static void level_rot_update(level_rot_t *L)
{
    float rd = g_params.level_roll_deg, pd = g_params.level_pitch_deg;
    if (rd == L->last_roll_deg && pd == L->last_pitch_deg) return;
    L->last_roll_deg  = rd;
    L->last_pitch_deg = pd;
    L->identity = (rd == 0.0f && pd == 0.0f);
    if (L->identity) return;

    float r = rd * 0.01745329252f, p = pd * 0.01745329252f;
    float sr = sinf(r), cr = cosf(r), sp = sinf(p), cp = cosf(p);
    L->m[0] =  cp;  L->m[1] = sp * sr;  L->m[2] =  sp * cr;
    L->m[3] = 0.0f; L->m[4] = cr;       L->m[5] = -sr;
    L->m[6] = -sp;  L->m[7] = cp * sr;  L->m[8] =  cp * cr;
}

static inline void level_rot_apply(const level_rot_t *L,
                                   float *x, float *y, float *z)
{
    if (L->identity) return;
    float vx = *x, vy = *y, vz = *z;
    *x = L->m[0]*vx + L->m[1]*vy + L->m[2]*vz;
    *y = L->m[3]*vx + L->m[4]*vy + L->m[5]*vz;
    *z = L->m[6]*vx + L->m[7]*vy + L->m[8]*vz;
}

void fusion_set_mag_capture(bool on)
{
    s_mag_capture = on;
}

void fusion_task(void *pvParameters)
{
    lsm6dsox_data_t imu = {0};
    mmc5603_data_t  mag = {0};
    float baro_temp_c   = 25.0f;
    float baro_press_pa = 0.0f;

    uint32_t loop_counter   = 0;
    uint32_t heartbeat      = 0;
    bool     mag_valid      = false;   /* passed the magnitude gate -> 9-DOF */
    bool     mag_read_ok    = false;   /* last mag read actually returned data */
    bool     baro_ok        = false;   /* last baro read actually returned data */
    bool     baro2_ok       = false;   /* MS5611 control baro, see ms5611.h */
    float    baro2_temp_c   = 0.0f;
    float    baro2_press_pa = 0.0f;
    uint32_t baro_seq       = 0;       /* fresh-sample marker, see slow_sample_t */
    uint32_t last_baro_seq  = 0;
    uint32_t last_tof_seq   = 0;
    uint32_t warmup_left    = WARMUP_CYCLES;
    const float beta_decay  = powf(MADGWICK_BETA / MADGWICK_BETA_WARMUP,
                                   1.0f / (float)WARMUP_CYCLES);

    /* One LPF + one notch section per axis. Seeded from the first good
     * sample rather than from zero, so the filters do not spend their
     * first samples ramping up - on a gyro that reads as a transient the
     * controller reacts to. */
    /* NAN, not 0: the first level_rot_update MUST fall through its
     * "unchanged" early-out and actually build the matrix. Seeded at 0 it
     * would match a zeroed param, skip the build, and leave an all-zero
     * matrix with identity == false - which multiplies every sensor
     * reading by nothing at all. */
    level_rot_t level_rot = { .last_roll_deg = NAN, .last_pitch_deg = NAN };

    /* Vertical filter (P7 stage 1). Task-local, like the biquads: one
     * owner, no locking, and the published outputs go through the same
     * state_mutex as everything else. */
    vert_filter_t vert;
    vert_filter_init(&vert, g_params.alt_acc_sigma, 0.0f);
    /* Velocity increment the filter has integrated, m/s. Task-local and
     * accumulated forever: the blackbox logs the ABSOLUTE sum and the
     * reader differences it, so only differences matter, never the
     * absolute value. float32
     * holds this easily - even a 0.19 m/s^2 mean for ten minutes reaches
     * only ~114 m/s, where the mantissa still resolves 1e-5. */
    float s_accz_dv_ms = 0.0f;
    float s_accn_dv_ms = 0.0f;
    float s_acce_dv_ms = 0.0f;

    /* Horizontal filter (Stage C step 2). Same ownership rule as the
     * vertical one above: task-local, one owner, no locking, outputs
     * published through state_mutex with everything else. */
    horiz_filter_t horiz;
    horiz_filter_init(&horiz, 0.0f);
    /* the author's altitude reference. Display only - see alt_ref.h. */
    alt_ref_t altref;
    alt_ref_init(&altref);
    /* NAV-PVT arrives at 10 Hz and its values are CACHED in nav_state,
     * so the fix must be fed to the filter exactly ONCE. Feeding the
     * cached value every 416 Hz cycle would tell the Kalman update it
     * had forty independent observations of one reading and collapse the
     * covariance to nothing - the identical trap vert_filter.h warns
     * about for the baro and the ToF. iTOW is the module's own time of
     * week: the same value twice means the solution did not update. */
    uint32_t last_gps_itow = 0xFFFFFFFFu;
    /* Home is latched on the arm edge and dropped on disarm. When it
     * moves, the filter's position states refer to an origin that no
     * longer exists, so it must start again rather than fly the old
     * one home. */
    uint8_t last_home_valid = 0;

    biquad_t gyro_lpf[3], gyro_notch[3];
    bool     gyro_lpf_primed = false;
    for (int i = 0; i < 3; i++) {
        biquad_init_lpf(&gyro_lpf[i], GYRO_LPF_HZ, (float)FUSION_RATE_HZ,
                        BIQUAD_Q_BUTTERWORTH);
        biquad_init_notch(&gyro_notch[i], GYRO_NOTCH_HZ,
                          (float)FUSION_RATE_HZ, GYRO_NOTCH_Q);
    }

    g_beta = MADGWICK_BETA_WARMUP;

    ESP_LOGI(TAG, "Fusion task started at %d Hz (Madgwick AHRS)", FUSION_RATE_HZ);

    imu_drdy_init();

#if FFT_LOG
    xTaskCreatePinnedToCore(fft_log_task, "fft_log", 4096, NULL,
                            2, NULL, FC_CORE_SUPPORT);
    ESP_LOGW(TAG, "FFT CAPTURE BUILD: gyro LPF bypassed, G, stream at loop rate");
#endif

    int64_t  last_us       = esp_timer_get_time();
    uint32_t drdy_timeouts = 0;

    /* Watchdog: fusion feeding stale attitude is caught by the flight
     * task's heartbeat check, but fusion HANGING (wedged bus, deadlock)
     * still needs the reboot path. 200 ms budget; I2C worst case with
     * bounded timeouts is ~40 ms. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        /* ---- 0. Wait for the IMU to say it has a new sample ----
         * Timeout is a safety net, not the normal path: if the wire is
         * broken, the pin is wrong, or an edge is ever missed while the
         * line sits high, this still fires and the read below clears
         * DRDY, which restarts the edges on its own. */
        uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DRDY_TIMEOUT_MS));

        esp_task_wdt_reset();
        perf_begin(PERF_LOOP);

        /* ---- 0b. Timing, from the interrupt edge ---- */
        int64_t now_us;
        if (got) {
            now_us = drdy_stamp();
        } else {
            now_us = esp_timer_get_time();
            if (++drdy_timeouts == 1 || drdy_timeouts % 500 == 0) {
                ESP_LOGW(TAG, "DRDY timeout x%lu, noise rejects %lu - INT1 on GPIO %d "
                         "is open or intermittent",
                         (unsigned long)drdy_timeouts,
                         (unsigned long)s_drdy_rejects, IMU_DRDY_GPIO);
            }
        }
        float dt = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.010f) dt = 0.010f;

#if HIL_MODE
        /* ---- 1+2 (HIL). IMU + mag come from the serial injector.
         * Values are held between "S" lines (MATLAB sends ~50-100 Hz; the
         * loop's DRDY timeout paces it at ~100 Hz without an IMU - same as a
         * slow sensor, fine for Madgwick).
         * Baro stays real (harmless, attitude unaffected). ---- */
        esp_err_t err_imu = ESP_OK;
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            imu.ax = global_nav_state.hil_sens.ax;
            imu.ay = global_nav_state.hil_sens.ay;
            imu.az = global_nav_state.hil_sens.az;
            imu.gx = global_nav_state.hil_sens.gx;
            imu.gy = global_nav_state.hil_sens.gy;
            imu.gz = global_nav_state.hil_sens.gz;
            mag.mx = global_nav_state.hil_sens.mx;
            mag.my = global_nav_state.hil_sens.my;
            mag.mz = global_nav_state.hil_sens.mz;
            xSemaphoreGive(state_mutex);
        }
        {
            float m2 = mag.mx * mag.mx + mag.my * mag.my + mag.mz * mag.mz;
            mag_valid = (m2 > 100.0f) && (m2 < 14400.0f);
            mag_read_ok = true;   /* injected data is fresh by definition */
        }
        if (baro_dev && loop_counter % BARO_MAG_DIVIDER == 0) {
            baro_ok = (dps310_read(baro_dev, &baro_temp_c, &baro_press_pa) == ESP_OK);
        }
#else
        /* ---- 0b. On-demand gyro bias recal (GYROCAL serial command).
         * Runs in THIS task so the SPI device keeps a single owner.
         * Blocks the loop ~2-6 s; board must be still. ---- */
        if (gyro_recal_requested && imu_dev) {
            gyro_recal_requested = false;
            /* Recal blocks this loop 2-6 s - legitimately. Unsubscribe
             * from the 200 ms watchdog for the duration. */
            esp_task_wdt_delete(NULL);
            lsm6dsox_calibrate_bias(imu_dev);
            ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
            last_us = esp_timer_get_time();   /* don't count cal time as dt */
            warmup_left = WARMUP_CYCLES / 2;  /* fast re-lock after pause */
            g_beta = MADGWICK_BETA_WARMUP;
        }

        /* ---- 0c. On-demand level calibration (LEVELCAL serial command).
         * Same contract as GYROCAL above: this task owns the IMU, the
         * capture blocks the loop (~1 s here), so detach the 200 ms
         * watchdog for the duration and do not charge the pause to dt. */
        if (level_cal_requested && imu_dev) {
            level_cal_requested = false;
            esp_task_wdt_delete(NULL);
            run_level_cal();
            ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
            last_us = esp_timer_get_time();
            /* The quaternion still holds the OLD frame's attitude, off by
             * however much just got corrected. Re-lock fast rather than
             * letting beta 0.04 walk there over minutes. */
            warmup_left = WARMUP_CYCLES / 2;
            g_beta = MADGWICK_BETA_WARMUP;
        }

        /* ---- 1. Read IMU (every cycle) ---- */
        perf_begin(PERF_IMU_READ);
        esp_err_t err_imu = lsm6dsox_read(imu_dev, &imu);
        perf_end(PERF_IMU_READ);

        /* Level reference (LEVELCAL): rotate the SENSOR frame onto the
         * AIRFRAME frame before anything reads it, so every consumer -
         * Madgwick, the rate PID, the blackbox, the earth-frame accel
         * below - agrees on where level is. Applied ahead of the gyro
         * biquads; identical per-axis LTI filters commute with a fixed
         * rotation, so the order is free and this way "the driver reports
         * the level frame" holds everywhere downstream.
         *
         * No-op (one branch) until LEVELCAL has actually run. */
        level_rot_update(&level_rot);
        if (err_imu == ESP_OK) {
            level_rot_apply(&level_rot, &imu.ax, &imu.ay, &imu.az);
            level_rot_apply(&level_rot, &imu.gx, &imu.gy, &imu.gz);
        }

        /* Gyro filters, in place, so Madgwick, the PID and the logged
         * stream all see the same signal the controller acts on. LPF
         * first, then the wide 78 Hz notch. fc = 0 (FFT builds) makes
         * both inits produce pass-throughs. */
        if (err_imu == ESP_OK &&
            (GYRO_LPF_HZ > 0.0f || GYRO_NOTCH_HZ > 0.0f)) {
            if (!gyro_lpf_primed) {
                biquad_reset(&gyro_lpf[0],   imu.gx);
                biquad_reset(&gyro_lpf[1],   imu.gy);
                biquad_reset(&gyro_lpf[2],   imu.gz);
                biquad_reset(&gyro_notch[0], imu.gx);
                biquad_reset(&gyro_notch[1], imu.gy);
                biquad_reset(&gyro_notch[2], imu.gz);
                gyro_lpf_primed = true;
            }
            imu.gx = biquad_apply(&gyro_notch[0],
                                  biquad_apply(&gyro_lpf[0], imu.gx));
            imu.gy = biquad_apply(&gyro_notch[1],
                                  biquad_apply(&gyro_lpf[1], imu.gy));
            imu.gz = biquad_apply(&gyro_notch[2],
                                  biquad_apply(&gyro_lpf[2], imu.gz));
        }

#if FFT_LOG
        /* After the (bypassed) filter, so this logs exactly what Madgwick
         * and the PID see. Timestamp is the DRDY edge, not the print. */
        if (err_imu == ESP_OK) fft_push(now_us, imu.gx, imu.gy, imu.gz);
#endif

        /* ---- 2. Pick up the latest baro + mag snapshot ----
         * No I2C here any more; slow_sensor_task publishes at 50 Hz on
         * core 0. Kept under PERF_I2C so the P, line shows this collapse
         * directly against the old numbers. */
        perf_begin(PERF_I2C);
        {
            slow_sample_t slow;
            if (slow_sample_get(&slow)) {
                mag           = slow.mag;
                mag_read_ok   = slow.mag_read_ok;
                mag_valid     = slow.mag_valid;
                baro_ok       = slow.baro_ok;
                baro_temp_c   = slow.baro_temp_c;
                baro_press_pa = slow.baro_press_pa;
                baro_seq      = slow.baro_seq;
                baro2_ok       = slow.baro2_ok;
                baro2_temp_c   = slow.baro2_temp_c;
                baro2_press_pa = slow.baro2_press_pa;

                /* Same level reference as the IMU. The mag's own hard/
                 * soft iron cal was fitted in the SENSOR frame, so the
                 * driver applies that first and this rotates the
                 * corrected vector - cal in its own frame, then reframe.
                 *
                 * The 10-120 uT gate upstream is unaffected: it tests
                 * magnitude, and rotations preserve magnitude.
                 *
                 * INSIDE the snapshot branch, not after it. `mag` is
                 * loop-scoped and keeps its previous contents when
                 * slow_sample_get gives up on a seqlock retry; rotating
                 * unconditionally would rotate an already-rotated vector
                 * again and again, which reads as yaw that wanders for
                 * no reason. Rotate only what was just copied in. */
                level_rot_apply(&level_rot, &mag.mx, &mag.my, &mag.mz);
            }
        }
        perf_end(PERF_I2C);
        
#endif /* HIL_MODE */
        loop_counter++;

        if (warmup_left) {
            g_beta *= beta_decay;
            if (--warmup_left == 0) {
                g_beta = MADGWICK_BETA;   /* land exactly on it */
                ESP_LOGI(TAG, "Madgwick warmup done, beta -> %.2f",
                         (double)MADGWICK_BETA);
            }
        }

        /* ---- 3. Run Madgwick AHRS ---- */
        if (err_imu == ESP_OK) {
            perf_begin(PERF_MADGWICK);
            if (mag_valid) {
                madgwick_update(imu.gx, imu.gy, imu.gz,
                                imu.ax, imu.ay, imu.az,
                                mag.mx, mag.my, mag.mz, dt);
            } else {
                madgwick_update_imu(imu.gx, imu.gy, imu.gz,
                                    imu.ax, imu.ay, imu.az, dt);
            }
            perf_end(PERF_MADGWICK);
        }

        /* ---- 4. Extract Euler angles ---- */
        float roll, pitch, yaw;
        quat_to_euler(&roll, &pitch, &yaw);

        /* ---- 4b. Earth-frame vertical acceleration (P7 stage 0) ----
         *
         * The vertical filter needs an acceleration that points at the
         * SKY, not at the top of the airframe: a tilted quad reads less
         * than 1 g on body Z while accelerating nowhere, and feeding that
         * to an integrator invents a climb rate out of a banked turn.
         *
         * Madgwick's convention here (see the gradient terms above, and
         * quat_to_euler): the earth "up" unit vector expressed in BODY
         * coordinates is
         *     [ 2(q1q3 - q0q2), 2(q0q1 + q2q3), 1 - 2(q1q1 + q2q2) ]
         * which is the third ROW of the body->earth rotation matrix. So
         * dotting it with the body accel gives the earth-frame Z
         * component directly - no matrix, no trig, 6 multiplies.
         *
         * Sanity check that must hold: level and at rest, body accel is
         * (0,0,+1) g and q is identity, so the dot product is +1 g and
         * this reports 0. Sitting still is not accelerating.
         *
         * Sign: POSITIVE = UP, matching the z-up body frame the mixer and
         * gyro_z already use. 1 g is subtracted BEFORE the m/s^2 scale so
         * the constant stays exact in the units it is defined in. */
        float accz_ms2 = 0.0f;
        /* ---- and the HORIZONTAL pair, for the 6-state horizontal
         * filter and the 15-state ESKF.
         *
         * The Z component above uses the THIRD row of the body->earth
         * rotation matrix. North and East are the first two rows of the
         * same matrix, from the same quaternion - 12 more multiplies, no
         * trig, no matrix object. Adding them here rather than in the
         * filter is the point: fusion already owns the quaternion, and
         * an estimator that had to re-derive attitude would be a second
         * place for the convention to be wrong.
         *
         * NO GRAVITY TERM, unlike Z. Gravity is a purely vertical vector
         * in the earth frame, so it lands entirely in the Z row and
         * contributes exactly nothing to north or east. Subtracting
         * anything here would be subtracting it twice.
         *
         * Sign: +north, +east, matching nav_state's NED position and the
         * module's own NAV-PVT velocity, so every horizontal quantity in
         * this project points the same way. (NED's third axis is DOWN
         * while accz_ms2 is UP - that inconsistency is older than this
         * change and deliberately not touched here.)
         *
         * Sanity check that must hold: level and at rest, body accel is
         * (0,0,+1) g and q is identity, so rows 0 and 1 dot to 0. */
        float accn_ms2 = 0.0f;
        float acce_ms2 = 0.0f;
        if (err_imu == ESP_OK) {
            float r20 = 2.0f * (q1 * q3 - q0 * q2);
            float r21 = 2.0f * (q0 * q1 + q2 * q3);
            float r22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
            float az_g = r20 * imu.ax + r21 * imu.ay + r22 * imu.az;
            accz_ms2 = (az_g - 1.0f) * 9.80665f;

            float r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
            float r01 = 2.0f * (q1 * q2 - q0 * q3);
            float r02 = 2.0f * (q1 * q3 + q0 * q2);
            float r10 = 2.0f * (q1 * q2 + q0 * q3);
            float r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
            float r12 = 2.0f * (q2 * q3 - q0 * q1);
            accn_ms2 = (r00 * imu.ax + r01 * imu.ay + r02 * imu.az) * 9.80665f;
            /* NOT negated, and the reason is written here because I
             * negated it once and had to put it back.
             *
             * Walk s162 said east was inverted: against GPS Doppler over
             * 27 clean walking transitions, r_north +0.763 / r_east
             * -0.733. Walk s164, two hours later on the same firmware,
             * said the opposite: r_north +0.550 / r_east +0.837. A sign
             * error in this line cannot change between two logs from the
             * same binary, so whatever flipped is NOT here. Leave the
             * matrix textbook-correct and find the real cause before
             * touching this again - the prime suspect is the YAW used to
             * build the rotation, which is magnetometer-driven and has
             * already been measured wandering tens of degrees while
             * walking. */
            acce_ms2 = (r10 * imu.ax + r11 * imu.ay + r12 * imu.az) * 9.80665f;
        }

        /* ---- 5. Compute altitude from pressure ---- */
        float alt_m = 0.0f;
        if (baro_press_pa > 0.0f) {
            float temp_k = baro_temp_c + 273.15f;
            alt_m = (powf(SEA_LEVEL_PRESSURE / baro_press_pa, 1.0f / 5.257f) - 1.0f)
                  * temp_k / 0.0065f;
        }

        /* Same conversion for the control barometer, deliberately using
         * the identical formula and constant. Nothing consumes this - it
         * exists so the two baros can be plotted on one axis. */
        float baro2_alt_m = 0.0f;
        if (baro2_press_pa > 0.0f) {
            float t2_k = baro2_temp_c + 273.15f;
            baro2_alt_m = (powf(SEA_LEVEL_PRESSURE / baro2_press_pa,
                                1.0f / 5.257f) - 1.0f) * t2_k / 0.0065f;
        }

        /* ---- 5b. Vertical filter (P7 stage 1) ----
         * Altitude and climb rate, from the accelerometer at 416 Hz, the
         * barometer at 50 Hz and the ToF at ~5 Hz. Design, derivation
         * and the reason for four states: tools/vert_filter.py. The C is
         * diffed against that Python over a real 25789-frame log by
         * tools/vert_ctest.py.
         *
         * Runs HERE, in the estimator, not in the flight loop: this task
         * owns accz and the baro, it is the only 416 Hz consumer of
         * both, and keeping estimation out of the controller is what
         * lets the STM32 split cut cleanly later.
         *
         * acc_sigma is re-read every cycle so it can be tuned live over
         * the console during Log B - it is the single most important
         * unmeasured number in the filter (hover vibration). */
        vert.acc_sigma = g_params.alt_acc_sigma;
        if (err_imu == ESP_OK) {
            vert_filter_predict(&vert, accz_ms2, dt);
            /* The SAME accz and the SAME dt the filter just integrated,
             * accumulated here so the 250 Hz blackbox can record the
             * integral rather than a decimated sample of the integrand.
             * See nav_state.accz_dv_ms. Kept inside this branch on
             * purpose: a skipped predict must contribute nothing. */
            s_accz_dv_ms += accz_ms2 * dt;
            /* Same integral, same reason, for the horizontal pair: the
             * blackbox runs at 250 Hz and this loop at 416, so a logged
             * SAMPLE cannot reconstruct what the filter consumed. The
             * reader differences two rows. See nav_state.accz_dv_ms. */
            s_accn_dv_ms += accn_ms2 * dt;
            s_acce_dv_ms += acce_ms2 * dt;
            /* Horizontal prediction, on the same good-IMU condition and
             * for the same reason: integrating a stale acceleration is
             * how dead reckoning invents motion. */
            horiz_filter_predict(&horiz, accn_ms2, acce_ms2, dt);
        }

        /* ---- Horizontal correction, once per NAV-PVT ----
         *
         * Home moving is a change of ORIGIN, not new information: the
         * position states refer to a point that no longer exists, so the
         * filter restarts rather than flying the old origin home. Both
         * edges matter - latching at arm and dropping at disarm. */
        const uint8_t home_now = global_nav_state.home_valid;
        if (home_now != last_home_valid) {
            horiz_filter_init(&horiz, 0.0f);
            last_gps_itow = 0xFFFFFFFFu;
            last_home_valid = home_now;
        }
        const uint32_t itow = global_nav_state.gps_itow_ms;
        if (home_now && global_nav_state.gps_fix_ok && itow != last_gps_itow) {
            last_gps_itow = itow;
            /* VELOCITY FIRST, deliberately. It is the better measurement
             * by a wide margin (MEASURED: 0.07-0.11 m/s parked against
             * 1.07-3.21 m on position), so correcting with it before
             * position is judged means position is gated against a
             * filter that has already had the good news. */
            horiz_filter_update_gps_vel(
                &horiz,
                (float)global_nav_state.vel_n_mms * 0.001f,
                (float)global_nav_state.vel_e_mms * 0.001f,
                (float)global_nav_state.gps_s_acc_mms * 0.001f);
            horiz_filter_update_gps_pos(
                &horiz,
                global_nav_state.pos_n_m,
                global_nav_state.pos_e_m,
                (float)global_nav_state.gps_h_acc_mm * 0.001f);
        }

        /* Each measurement EXACTLY ONCE. Both sensors are far slower
         * than this loop and their values are cached, so applying them
         * every cycle would tell the Kalman update it had dozens of
         * independent observations of one reading and collapse the
         * covariance. The sequence counters are what make "fresh" a
         * fact rather than a guess - comparing values would misread a
         * genuinely unchanged reading (i.e. hovering still) as stale. */
        if (baro_ok && baro_seq != last_baro_seq && baro_press_pa > 0.0f) {
            last_baro_seq = baro_seq;
            vert_filter_update_baro(&vert, alt_m);
        }

        uint32_t tof_seq = vl53l0x_sample_seq();
        if (tof_seq != last_tof_seq) {
            last_tof_seq = tof_seq;
            uint16_t mm = vl53l0x_cached_mm();
            if (mm == 0 || mm >= VL53L0X_MM_NO_TARGET) {
                vert_filter_tof_lost(&vert);
            } else {
                /* r22 IS the cosine of the tilt from vertical - it is
                 * the third component of earth-up expressed in body
                 * coordinates, already computed above for accz. The ToF
                 * looks along body -Z, so a tilted aircraft measures a
                 * longer slant range than its true height. No trig. */
                float r22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
                vert_filter_update_tof(&vert, (float)mm * 0.001f, r22);
            }
        }

        /* the author's altitude reference, run on EVERY cycle rather than on
         * the ToF sample edge, because it is a time-based scheme: its
         * disagreement clock and its slew rate are both per-second, and
         * both would be wrong if they only advanced when the sensor
         * happened to answer. It reads the CACHED ToF for the same
         * reason the driver caches it - the value is valid between
         * samples. Display only; nothing closes a loop on this. */
        {
            uint16_t ar_mm = vl53l0x_cached_mm();
            const bool ar_live =
                (ar_mm != 0 && ar_mm < VL53L0X_MM_NO_TARGET);
            /* baro_ok, not `baro_press_pa > 0.0f`. The latter is TRUE
             * FOREVER after the first successful read - the cached
             * pressure simply stops changing when the chip dies - so
             * alt_ref was being told the barometer was fine no matter
             * what. baro_ok is the same bit that feeds SENSOR_BARO_OK
             * and it means "answered this cycle". */
            alt_ref_update(&altref, alt_m, baro_ok,
                           ar_live ? (float)ar_mm * 0.001f : -1.0f,
                           dt);
        }

        /* ---- 6. Update global state ---- */
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (err_imu == ESP_OK) {
                global_nav_state.roll  = roll;
                global_nav_state.pitch = pitch;
                global_nav_state.yaw   = yaw;
                global_nav_state.gyro_x = imu.gx;
                global_nav_state.gyro_y = imu.gy;
                global_nav_state.gyro_z = imu.gz;
                global_nav_state.acc_x = imu.ax;
                global_nav_state.acc_y = imu.ay;
                global_nav_state.acc_z = imu.az;
                global_nav_state.accz_ms2 = accz_ms2;
                global_nav_state.accn_ms2 = accn_ms2;
                global_nav_state.acce_ms2 = acce_ms2;
            }
            /* Published unconditionally, unlike accz above: the filter
             * PREDICTS through a bad IMU read rather than holding a
             * stale value, so its output is fresh every cycle even when
             * the sensor is not. vert_valid says whether an absolute
             * reference has ever been seen - a controller must not close
             * a loop on pure dead reckoning. */
            global_nav_state.alt_est_m  = vert.z;
            global_nav_state.vz_ms      = vert.vz;
            global_nav_state.accz_bias  = vert.ba;
            global_nav_state.vert_valid = vert.valid ? 1 : 0;
            global_nav_state.vert_datum_shift += vert_filter_take_datum_shift(&vert);
            global_nav_state.alt_ref_m  = altref.z;
            global_nav_state.alt_ref_valid = altref.valid ? 1 : 0;
            global_nav_state.accz_dv_ms = s_accz_dv_ms;
            global_nav_state.accn_dv_ms = s_accn_dv_ms;

            /* Horizontal estimator, published on the same terms. */
            global_nav_state.horiz_vn_ms = horiz.vn;
            global_nav_state.horiz_ve_ms = horiz.ve;
            global_nav_state.horiz_pn_m  = horiz.pn;
            global_nav_state.horiz_pe_m  = horiz.pe;
            /* THREE conditions, and the third is the one that matters.
             *
             * home_valid and gps_fix_ok are the module's own opinion of
             * itself, and a receiver that has stopped talking keeps
             * reporting both from its last good message - which is
             * exactly the failure that let alt-hold engage on a frozen
             * ToF: a dead sensor whose cached value still
             * looked reasonable. So freshness is tested too, against the
             * LOCAL clock rather than the module's.
             *
             * 1 s is ~10 missed NAV-PVT messages at 10 Hz. Far past any
             * plausible dropout, far short of the time a filter coasting
             * on accelerometer alone needs to invent metres - the
             * horizontal channel has no ToF equivalent to catch it, so
             * this window is the only thing that does.
             *
             * gps_time_us is int64 on purpose: a uint32 of microseconds
             * wraps every 71 minutes and a wrapped freshness check reads
             * as "fresh" at exactly the wrong moment. */
            const int64_t gps_age_us =
                esp_timer_get_time() - global_nav_state.gps_time_us;
            global_nav_state.horiz_valid =
                (global_nav_state.home_valid &&
                 global_nav_state.gps_fix_ok &&
                 horiz.have_fix &&
                 gps_age_us < 1000000) ? 1 : 0;
            global_nav_state.acce_dv_ms = s_acce_dv_ms;
            global_nav_state.fusion_dt = dt;

            global_nav_state.baro2_press_pa = baro2_press_pa;
            global_nav_state.baro2_temp_c   = baro2_temp_c;
            global_nav_state.baro2_alt_m    = baro2_alt_m;
            global_nav_state.pressure_pa = baro_press_pa;
            global_nav_state.altitude_m  = alt_m;
            global_nav_state.temp_c      = baro_temp_c;

            global_nav_state.mag_x = mag.mx;
            global_nav_state.mag_y = mag.my;
            global_nav_state.mag_z = mag.mz;

            global_nav_state.quat[0] = q0;
            global_nav_state.quat[1] = q1;
            global_nav_state.quat[2] = q2;
            global_nav_state.quat[3] = q3;

            /* Sensor health. The mag/baro values above may be REPUBLISHED
             * STALE data when their read failed - these bits are the only
             * way a consumer can tell. */
            global_nav_state.sensor_health =
                  (err_imu == ESP_OK ? SENSOR_IMU_OK    : 0)
                | (baro_ok           ? SENSOR_BARO_OK   : 0)
                | (mag_read_ok       ? SENSOR_MAG_OK    : 0)
                | (mag_valid         ? SENSOR_MAG_GATED : 0)
                | (baro2_ok          ? SENSOR_BARO2_OK  : 0);

            /* The heartbeat is the flight task's proof that fusion is
             * alive AND FED (MAX_STALE_LOOPS). It advances only on a sample
             * the IMU actually delivered. It used to advance on every loop,
             * so a dead or garbage IMU kept it ticking and the flight loop
             * flew on the last good gyro reading indefinitely (
             * review, item 2). Now 50 flight cycles without a real sample
             * disarm, same as a hung fusion task. */
            if (err_imu == ESP_OK) heartbeat++;
            global_nav_state.val_heartbeat = heartbeat;

            xSemaphoreGive(state_mutex);
        }

        perf_end(PERF_LOOP);
        /* No delay here - the loop is paced by DRDY at the top. */
    }
}
