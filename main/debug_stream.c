#include "debug_stream.h"
#include "fc_cores.h"
#include "nav_state.h"
#include "fusion.h"
#include "fc_tasks.h"
#include "config.h"
#include "perf_stats.h"
#include "dps310.h"
#include "mmc5603.h"
#include "lsm6dsox.h"
#include "pid.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "ibus.h"
#include "telemetry.h"
#include "params.h"
#include "blackbox.h"
#include "ina226.h"
#include "vl53l0x.h"
#include "i2c_all.h"
#include "pmw3901.h"
#include "tbsM10.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

static const char *TAG = "DBG_STREAM";

#define STREAM_RATE_HZ 50
#define RX_LINE_MAX    160

static volatile bool stream_enabled = true;

/* ================= P, 1 Hz perf stats =================
 * P,t_us,loop_n,loop_min,loop_mean,loop_max,imu_mean,imu_max,
 *   mad_mean,mad_max,i2c_mean,i2c_max,
 *   hwm_fusion,hwm_flight,heap_free,discards,baro_errs,mag_errs
 * Times in us over the last 1 s window (P1 measurement kit).
 * HWM = uxTaskGetStackHighWaterMark, BYTES on Xtensa (min ever free).
 * discards = perf samples dropped due to core migration (unpinned
 * tasks, until P2).
 * baro_errs/mag_errs = CUMULATIVE I2C failures since boot (not per
 * window). Rising counters + a large i2c_max means the bus is timing
 * out; pair them with the SENSOR_* bits on the D, line. */
static void emit_perf_line(void)
{
    static TaskHandle_t h_fusion = NULL, h_flight = NULL;
    if (!h_fusion) h_fusion = xTaskGetHandle("fusion_task");
    if (!h_flight) h_flight = xTaskGetHandle("flight_task");

    perf_stats_t st[PERF_SECTION_COUNT];
    perf_harvest(st);

    uint32_t discards = 0;
    for (int i = 0; i < PERF_SECTION_COUNT; i++) discards += st[i].discarded;

    printf("P,%lld,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
           (long long)esp_timer_get_time(),
           (unsigned long)st[PERF_LOOP].count,
           (unsigned long)st[PERF_LOOP].min_us,
           (unsigned long)st[PERF_LOOP].mean_us,
           (unsigned long)st[PERF_LOOP].max_us,
           (unsigned long)st[PERF_IMU_READ].mean_us,
           (unsigned long)st[PERF_IMU_READ].max_us,
           (unsigned long)st[PERF_MADGWICK].mean_us,
           (unsigned long)st[PERF_MADGWICK].max_us,
           (unsigned long)st[PERF_I2C].mean_us,
           (unsigned long)st[PERF_I2C].max_us,
           (unsigned long)(h_fusion ? uxTaskGetStackHighWaterMark(h_fusion) : 0),
           (unsigned long)(h_flight ? uxTaskGetStackHighWaterMark(h_flight) : 0),
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
           (unsigned long)discards,
           (unsigned long)dps310_i2c_err_count(),
           (unsigned long)mmc5603_i2c_err_count());
}

/* ================= TX: 50 Hz CSV =================
 *
 * Fields 0-31 are the frozen original layout (0 is the "D" tag, 31 is
 * sensor_health). APPEND ONLY at the tail, never renumber - the MATLAB
 * bench scripts and tools/capture.py index by position.
 *
 * 32-40, added for the live viewer (tools/viewer.py):
 *   32 tof_mm        VL53L0X cache: 0 = absent/failing, 65535 = no target
 *   33 flow_seq      bursts since boot; frozen = the flow task is not running
 *   34 flow_sum_dx   ABSOLUTE running sums of VALID windows, counts
 *   35 flow_sum_dy
 *   36 flow_valid    windows integrated into those sums
 *   37 flow_lost     windows whose counts are gone (a hole in the sums)
 *   38 flow_squal    newest sample: surface quality
 *   39 flow_shutter  newest sample: exposure
 *   40 flow_flags    newest sample: PMW_F_* bits
 *
 * WHY SUMS AND NOT dx/dy: the flow sensor runs at 100 Hz and this line at
 * 50, so a "newest dx" field would throw away every second window and any
 * integration built on it would read half the real motion. Differencing
 * two absolute sums is exact no matter how many lines are missed - the
 * same rule as the blackbox's accz_dv_ms. */
static void stream_task(void *pv)
{
    nav_state_t s;
    uint32_t tick = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / STREAM_RATE_HZ);

    while (1) {
        if (stream_enabled &&
            xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s = global_nav_state;
            xSemaphoreGive(state_mutex);

            pmw3901_totals_t ft;
            pmw3901_sample_t fs;
            pmw3901_totals(&ft);
            const bool fhave = pmw3901_latest(&fs);

            printf("D,%lld,"
                   "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
                   "%.5f,%.5f,%.5f,%.5f,"
                   "%.2f,%.2f,%.2f,"
                   "%.5f,%.5f,"
                   "%u,%u,%u,%u,%u,%lu,%.2f,"
                   "%.6f,%.6f,%u,%u,%u,"
                   "%u,%lu,%ld,%ld,%lu,%lu,%u,%u,%u\n",
                   (long long)esp_timer_get_time(),
                   (double)s.acc_x, (double)s.acc_y, (double)s.acc_z,
                   (double)s.gyro_x, (double)s.gyro_y, (double)s.gyro_z,
                   (double)s.mag_x, (double)s.mag_y, (double)s.mag_z,
                   (double)s.quat[0], (double)s.quat[1],
                   (double)s.quat[2], (double)s.quat[3],
                   (double)s.roll, (double)s.pitch, (double)s.yaw,
                   (double)s.fusion_dt, (double)s.loop_dt,
                   s.m1_pwm, s.m2_pwm, s.m3_pwm, s.m4_pwm,
                   s.is_armed, (unsigned long)s.val_heartbeat,
                   (double)s.altitude_m,
                   (double)s.lat * 1.0e-7, (double)s.lon * 1.0e-7,
                   s.sats, s.fix_type,
                   s.sensor_health,
                   vl53l0x_cached_mm(),
                   (unsigned long)ft.seq,
                   (long)ft.sum_dx, (long)ft.sum_dy,
                   (unsigned long)ft.n_valid, (unsigned long)ft.n_lost,
                   fhave ? fs.b.squal : 0,
                   fhave ? fs.b.shutter : 0,
                   fhave ? fs.flags : 0);
        }

        /* 1 Hz perf stats ride the same stream */
        if (stream_enabled && ++tick >= STREAM_RATE_HZ) {
            tick = 0;
            emit_perf_line();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* ================= Serial command interpreter =================
 *
 * Reachable from two places: the USB console (dbg_rx task) and the
 * telemetry link from the C6 dashboard. The dashboard route exists
 * because typing a command into a console that is printing 50 CSV lines
 * a second does not work in practice - you cannot see what you typed and
 * the reply scrolls past instantly.
 *
 * Replies go back the way the command came in, so a dashboard command
 * lands in the dashboard's console pane, not on the USB port nobody is
 * watching. */
typedef enum { CMD_OUT_USB, CMD_OUT_TELEM } cmd_out_t;

/* BBDUMP sink: raw base64 lines to the USB console. Not cmd_reply - a
 * dump line is ~1 KB and needs no "#" framing; the puller script
 * filters on the "BB," prefix. */
static void bbdump_sink_usb(const char *line)
{
    printf("%s\n", line);
}

static void cmd_reply(cmd_out_t out, const char *fmt, ...)
{
    char buf[224];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;

    if (out == CMD_OUT_USB) printf("%s\n", buf);
    else                    telemetry_write_console(buf);
}

/* Commands that stall the flight loop or write flash must not run in
 * the air. GYROCAL blocks the fusion task 2-6 s (no attitude updates);
 * MAGSET commits to NVS. Both were bench-only while the USB console was
 * the only way in - the dashboard makes them reachable over WiFi, so
 * the guard has to be explicit now. Unsynchronised read of a uint8_t is
 * fine here: it is a coarse safety gate, not a data handoff. */
static bool refuse_if_armed(cmd_out_t out, const char *what)
{
    if (global_nav_state.is_armed) {
        cmd_reply(out, "%s refused: ARMED (disarm first)", what);
        ESP_LOGW(TAG, "%s refused while armed", what);
        return true;
    }
    return false;
}

static void exec_command(char *line, cmd_out_t out)
{
    if (strncmp(line, "STREAM,", 7) == 0) {
        stream_enabled = (line[7] == '1');
        cmd_reply(out, "USB D,/P, stream %s", stream_enabled ? "ON" : "OFF");
        return;
    }

    /* GSEC: what the GNSS interference guard currently believes.
     *   GSEC        one verdict, with every standing reason spelled out
     *   GSEC,CLEAR  release a latched denial (bench only)
     *
     * The reasons are printed by NAME rather than as a bitmask, because a
     * hex word in a log is something nobody decodes at the field at
     * midnight. See components/gnss_guard/gnss_guard.h. */
    if (strncmp(line, "GSEC", 4) == 0) {
        if (line[4] == ',' && strncmp(line + 5, "CLEAR", 5) == 0) {
            tbsm10_guard_clear();
            cmd_reply(out, "gsec: latch released. This overrides the LATCH, "
                           "not the evidence - if the interferer is still "
                           "there the guard is back within a second.");
            return;
        }
        /* First, and outside the mutex: the module's answer to every config
         * message. Whether NAV-PVT and the guard's inputs exist at all
         * decides what the rest of this output can mean. */
        char cfg[160];
        tbsm10_cfg_report(cfg, sizeof(cfg));
        cmd_reply(out, "gsec: %s", cfg);

        gg_status_t gs;
        if (!tbsm10_guard_status(&gs)) {
            cmd_reply(out, "gsec: state mutex busy, try again");
            return;
        }
        static const char *lvl[] = { "CLEAN", "SUSPECT", "DENIED" };
        cmd_reply(out, "gsec: pos=%s vel=%s  escalations=%lu%s",
                  lvl[gs.pos <= GG_DENIED ? gs.pos : 0],
                  lvl[gs.vel <= GG_DENIED ? gs.vel : 0],
                  (unsigned long)gs.n_escalations,
                  gs.denied_since_ms ? "  [LATCHED]" : "");
        cmd_reply(out, "gsec: module jam=%u spoof=%u | agc=%u base=%u | "
                       "noise=%u base=%u | cno mean=%.1f sd=%.1f over %u used",
                  gs.jamming_state, gs.spoofing_state,
                  gs.agc_cnt, gs.agc_baseline,
                  gs.noise_per_ms, gs.noise_baseline,
                  (double)gs.cno_mean, (double)gs.cno_sd, gs.n_used);

        static const struct { uint16_t bit; const char *name; } names[] = {
            { GG_R_MOD_JAM_WARN,   "module jamming WARNING" },
            { GG_R_MOD_JAM_CRIT,   "module jamming CRITICAL" },
            { GG_R_MOD_SPOOF,      "module spoofing INDICATED" },
            { GG_R_MOD_SPOOF_MULT, "module spoofing MULTIPLE" },
            { GG_R_CNO_UNIFORM,    "C/N0 spread collapsed (one source?)" },
            { GG_R_CNO_COLLAPSE,   "C/N0 mean below floor" },
            { GG_R_AGC_SHIFT,      "AGC backed off from baseline" },
            { GG_R_NOISE_HIGH,     "noisePerMS above baseline" },
            { GG_R_VEL_DIVERGE,    "GPS velocity disagrees with inertial" },
            { GG_R_SAT_JUMP,       "used-satellite count jumped" },
            { GG_R_STALE,          "NO MON-RF/SEC-SIG - the guard is BLIND" },
        };
        if (gs.reasons == 0) {
            cmd_reply(out, "gsec: no standing reasons");
        }
        for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
            const char *tag = (gs.reasons & names[i].bit) ? "NOW"
                            : (gs.reasons_latched & names[i].bit) ? "was" : 0;
            if (tag) cmd_reply(out, "gsec:   [%s] %s", tag, names[i].name);
        }
        return;
    }

    /* STATS: one-shot FreeRTOS run-time stats + task list dump
     * (P1 measurement kit). Needs FREERTOS_GENERATE_RUN_TIME_STATS +
     * FREERTOS_USE_STATS_FORMATTING_FUNCTIONS in sdkconfig. */
    if (strncmp(line, "STATS", 5) == 0) {
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
        /* 1.5 KB is too much for these tasks' stacks, so the buffer is
         * static - which means two callers must not be inside it at
         * once now that there are two command sources. */
        static char buf[1536];
        static volatile bool busy = false;
        if (busy) { cmd_reply(out, "STATS busy, retry"); return; }
        busy = true;

        vTaskGetRunTimeStats(buf);
        cmd_reply(out, "=== run-time stats (task, abs time, %%CPU since boot) ===");
        if (out == CMD_OUT_USB) printf("%s", buf);
        else                    telemetry_write_console_block(buf);

        vTaskList(buf);
        cmd_reply(out, "=== tasks (name state prio stack-HWM num) ===");
        if (out == CMD_OUT_USB) printf("%s", buf);
        else                    telemetry_write_console_block(buf);

        busy = false;
#else
        cmd_reply(out, "STATS disabled: enable FREERTOS_GENERATE_RUN_TIME_STATS "
                       "and FREERTOS_USE_STATS_FORMATTING_FUNCTIONS in sdkconfig");
#endif
        /* Accelerometer rail watch. Outside the #if on purpose: it has
         * nothing to do with FreeRTOS run-time stats and must still be
         * readable on a build where those are off.
         *
         * Printed as a RATE, because a raw count answers nothing - 5000
         * clipped samples is a dead craft over one flight and noise over
         * an hour on the bench. Anything above ~0 while hovering means
         * the accel is spending time pinned at full scale, and the
         * one-sided cut rectifies into the DC bias on earth-frame Z
         * (-3.06 m/s^2 above 1400 us). Per-axis counts are
         * CHIP frame, so they point at the physical axis, not the
         * remapped body one. */
        {
            uint32_t n = lsm6dsox_accel_sample_count();
            uint32_t c = lsm6dsox_clip_count();
            uint32_t ca[3];
            lsm6dsox_clip_axis_counts(ca);
            cmd_reply(out, "=== accel rail watch (raw >= %d of 32767) ===",
                      LSM6DSOX_CLIP_RAW);
            cmd_reply(out, "samples %lu, clipped %lu (%.4f %%); "
                           "per-axis chip x %lu, y %lu, z %lu",
                      (unsigned long)n, (unsigned long)c,
                      n ? 100.0 * (double)c / (double)n : 0.0,
                      (unsigned long)ca[0], (unsigned long)ca[1],
                      (unsigned long)ca[2]);
        }
        /* IMU liveness (lsm6dsox.h). Bursts refused as not data. Both
         * must read 0: while either is refusing, the fusion heartbeat is
         * stopped and the aircraft cannot arm - or disarms if armed. */
        {
            uint32_t u = 0, fz = 0;
            lsm6dsox_liveness_counts(&u, &fz);
            cmd_reply(out, "=== IMU liveness (refused bursts, must be 0) ===");
            cmd_reply(out, "uniform (all 12 bytes equal) %lu, frozen (>%d "
                           "identical in a row) %lu",
                      (unsigned long)u, LSM6DSOX_FROZEN_READS,
                      (unsigned long)fz);

            /* Episodes and the chip's registers at the moment it refused
             * (LSM6DSOX_STICKY_READS in lsm6dsox.h says how to read them).
             * Short lines: the C6 console keeps 180 columns. */
            lsm6dsox_diag_t dg;
            lsm6dsox_diag_get(&dg);
            cmd_reply(out, "episodes %lu, longest %lu reads, now %lu | init "
                           "read-back %s (mask 0x%02X)",
                      (unsigned long)dg.episodes, (unsigned long)dg.longest,
                      (unsigned long)dg.current,
                      dg.cfg_bad ? "FAILED" : "ok", dg.cfg_bad);
            const lsm6dsox_dump_t *dd[2] = {
                dg.have_first  ? &dg.first  : NULL,
                dg.have_sticky ? &dg.sticky : NULL,
            };
            for (int k = 0; k < 2; k++) {
                const lsm6dsox_dump_t *d = dd[k];
                if (!d) continue;
                cmd_reply(out, "#%lu %s %.2f s: %s %02X, INT1 %s | WHO %02X I1 "
                               "%02X C1 %02X C2 %02X C3 %02X C4 %02X C9 %02X "
                               "ST %02X%s",
                          (unsigned long)d->episode,
                          k ? "+100 at" : "at", (double)d->t_us / 1e6,
                          d->kind == 1 ? "uniform" : "frozen", d->stuck,
                          d->int1 < 0 ? "?" : (d->int1 ? "HIGH" : "LOW"),
                          d->reg[0], d->reg[1], d->reg[2], d->reg[3],
                          d->reg[4], d->reg[5], d->reg[6], d->reg[7],
                          d->spi_err == ESP_OK ? "" : " READ FAILED");
            }
            if (dg.have_first && !dg.have_sticky && dg.current == 0)
                cmd_reply(out, "(it recovered before %d refusals - no second "
                               "dump)", LSM6DSOX_STICKY_READS);
        }
        /* dt clamp watch. The flight loop clamps dt to 2..6 ms while
         * running at 250 Hz (4 ms), so these should read 0. A non-zero
         * `hi` means the loop was starved and the PID integrator
         * under-counted real time for those cycles - which makes every
         * gain tuned in that session suspect. This is the evidence that
         * decides whether widening the clamp is worth the risk of changing
         * integration on a flying aircraft. */
        {
            uint32_t n = 0, lo = 0, hi = 0, mx = 0;
            fc_dt_clamp_stats(&n, &lo, &hi, &mx);
            cmd_reply(out, "=== anti-windup ===");
            cmd_reply(out, "integrator frozen %lu cycle(s) by mixer "
                           "saturation", pid_windup_freeze_count());
            cmd_reply(out, "=== non-finite guard (must be 0) ===");
            cmd_reply(out, "NaN/Inf replaced on the motor path: %lu",
                      (unsigned long)fc_nonfinite_count());
            cmd_reply(out, "=== dt clamp watch (loop clamps to 2000-6000 us) ===");
            cmd_reply(out, "cycles %lu, clamped low %lu (%.4f %%), high %lu "
                           "(%.4f %%), longest dt %lu us",
                      (unsigned long)n, (unsigned long)lo,
                      n ? 100.0 * (double)lo / (double)n : 0.0,
                      (unsigned long)hi,
                      n ? 100.0 * (double)hi / (double)n : 0.0,
                      (unsigned long)mx);
        }
        return;
    }

    /* GYROCAL: re-run gyro bias calibration (board must be STILL).
     * Executed by the fusion task on its next cycle. */
    if (strncmp(line, "GYROCAL", 7) == 0) {
        if (refuse_if_armed(out, "GYROCAL")) return;
        fusion_request_gyro_recal();
        cmd_reply(out, "gyro recal requested - keep the board still ~3 s");
        return;
    }

    /* LEVELCAL: measure what the board reads while it is PHYSICALLY
     * level and store that as the level reference (params
     * level_roll_deg / level_pitch_deg).
     *
     * Unlike GYROCAL this WAITS for the answer instead of firing and
     * forgetting. It has to: the sensor rail is battery-fed, so every
     * session where this command is usable is a session where USB is
     * unplugged and the ESP_LOG output goes nowhere. The console reply
     * is the only channel the numbers can come back on. */
    if (strncmp(line, "LEVELCAL", 8) == 0) {
        if (refuse_if_armed(out, "LEVELCAL")) return;
        fusion_request_level_cal();

        /* Capture is ~1 s; allow 5 s before giving up so a busy fusion
         * task or a retry does not look like a failure. */
        fusion_levelcal_t r;
        bool done = false;
        for (int i = 0; i < 50 && !done; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            done = fusion_level_cal_done(&r);
        }
        if (!done) {
            cmd_reply(out, "LEVELCAL: no result in 5 s - is fusion running?");
            return;
        }

        char buf[192];
        if (r.applied) {
            snprintf(buf, sizeof(buf),
                     "LEVELCAL: roll %+.2f pitch %+.2f deg applied "
                     "(|a| %.3f g, motion %.4f g). SEND SAVE TO KEEP IT - "
                     "without SAVE this is gone at the next power cycle.",
                     (double)r.roll_deg, (double)r.pitch_deg,
                     (double)r.mag_g, (double)r.worst_std_g);
        } else {
            snprintf(buf, sizeof(buf),
                     "LEVELCAL REFUSED: %s (|a| %.3f g, motion %.4f g) - "
                     "params unchanged. Board must be still and on a "
                     "level surface.",
                     r.why ? r.why : "unknown",
                     (double)r.mag_g, (double)r.worst_std_g);
        }
        cmd_reply(out, buf);
        return;
    }

    /* MAGSET,ox,oy,oz,s11,s12,s13,s21,s22,s23,s31,s32,s33
     * Sets the FULL mag calibration absolutely (not additive):
     * corrected = S * (raw - off). MAGSET with 3 values resets to
     * offset-only with identity S. */
    if (strncmp(line, "MAGSET,", 7) == 0) {
        if (refuse_if_armed(out, "MAGSET")) return;
        float v[12];
        int n = sscanf(line + 7,
                       "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                       &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                       &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]);
        if (n == 3 || n == 12) {
            static const float I9[9] = {1,0,0, 0,1,0, 0,0,1};
            esp_err_t err = fusion_set_mag_cal(v, n == 12 ? &v[3] : I9);
            cmd_reply(out, err == ESP_OK
                      ? "MAGSET stored (%d values)"
                      : "MAGSET failed (%d values) - mag not initialised?", n);
        } else {
            cmd_reply(out, "MAGSET: expected 3 (offset) or 12 (offset+matrix) values");
        }
        return;
    }

    /* MAGCAP,0|1: 50 Hz "M,t_us,mx,my,mz" capture stream out the telemetry
     * UART, for fitting a new mag calibration over the C6 WiFi bridge
     * instead of a USB tether. The whole tumble now has to move the frame,
     * not a loose board, so a cable is not an option.
     *
     * Refused while armed: it is not dangerous, but it puts 2.3 kB/s of
     * extra traffic on the link that carries the telemetry you would be
     * flying on, and there is no reason to capture mag data in the air. */
    if (strncmp(line, "MAGCAP,", 7) == 0) {
        if (refuse_if_armed(out, "MAGCAP")) return;
        bool on = (line[7] == '1');
        fusion_set_mag_capture(on);
        cmd_reply(out, on
                  ? "mag capture ON (M, at 50 Hz) - send MAGSET,0,0,0 FIRST "
                    "or you are fitting a correction on top of a correction"
                  : "mag capture OFF");
        return;
    }

    /* MOT,<1-4>,<us>[,<ms>]: bench motor test - spin ONE motor at a
     * fixed pulse width while disarmed, to hunt weak motors by
     * per-motor current / RPM / thrust. MOT,0 stops early; it also
     * auto-stops after ms (default 3000, max 10000) and on any arm
     * attempt. us capped at 1700. PROPS OFF, or airframe restrained. */
    if (strncmp(line, "MOT,", 4) == 0) {
        if (refuse_if_armed(out, "MOT")) return;
        int n = -1, us = 0, ms = 3000;
        int cnt = sscanf(line + 4, "%d,%d,%d", &n, &us, &ms);
        if (cnt >= 1 && n == 0) {
            fc_motor_test(0, 0, 0);
            cmd_reply(out, "MOT stopped");
            return;
        }
        if (cnt < 2 || n < 1 || n > 4 || us < 1000) {
            cmd_reply(out, "MOT,<1-4>,<1000-1700 us>[,<ms max 10000>] | MOT,0 stops");
            return;
        }
        if (ms < 100) ms = 100;
        fc_motor_test((uint8_t)n, (uint16_t)us, (uint32_t)ms);
        cmd_reply(out, "MOT: M%d -> %d us, %d ms max. PROPS OFF or RESTRAINED. "
                       "MOT,0 stops", n, us > 1700 ? 1700 : us, ms > 10000 ? 10000 : ms);
        return;
    }

    /* ESCPASS,0|1: throttle-stick passthrough to ALL FOUR outputs while
     * disarmed - the FC becomes a wire, so the classic TX-based ESC cal
     * ritual works with the user's own timing and ears. The console
     * ESCCAL failed on these ESCs, most plausibly because
     * they want the low right after the entry beeps and a dashboard
     * round-trip is slower than a thumb. */
    if (strncmp(line, "ESCPASS,", 8) == 0) {
        if (refuse_if_armed(out, "ESCPASS")) return;
        bool on = (line[8] == '1');
        fc_esc_passthrough(on);
        if (on) {
            cmd_reply(out, "ESC passthrough ON for 120 s - throttle stick drives "
                           "ALL 4 outputs directly. PROPS OFF. Ritual: pack "
                           "unplugged -> stick MAX -> plug pack -> entry beeps -> "
                           "stick MIN -> confirm beeps -> ESCPASS,0.");
            if (!global_nav_state.rc_link_ok)
                cmd_reply(out, "WARNING: no RC link right now - outputs stay at "
                               "1000 until the receiver delivers frames (TX on?)");
        } else {
            cmd_reply(out, "ESC passthrough OFF");
        }
        return;
    }

    /* ESCCAL: teach all four ESCs the same 1000-2000 range at once.
     * Three-stage on purpose: the HIGH phase is FULL THROTTLE on every
     * output, so it demands an explicit YES inside a 10 s window, and
     * the pack must be unplugged until the high is already being driven
     * (classic ESC cal entry: sees max at power-on, then learns min). */
    if (strncmp(line, "ESCCAL", 6) == 0) {
        if (refuse_if_armed(out, "ESCCAL")) return;
        const char *arg = (line[6] == ',') ? line + 7 : "";
        if (strncmp(arg, "HIGH", 4) == 0) {
            fc_esccal(1);
            cmd_reply(out, "ESCCAL: will drive 2000 us (FULL THROTTLE) on ALL 4 "
                           "outputs. PROPS OFF. PACK UNPLUGGED. FC on USB power. "
                           "Send ESCCAL,YES within 10 s to start.");
        } else if (strncmp(arg, "YES", 3) == 0) {
            if (fc_esccal(2) == 2) {
                cmd_reply(out, "ESCCAL: 2000 us on all 4 NOW. Plug the pack in, "
                               "wait for the cal-entry beeps, then send "
                               "ESCCAL,LOW. Auto-aborts to 1000 us in 60 s.");
            } else {
                cmd_reply(out, "ESCCAL: no HIGH request pending - send "
                               "ESCCAL,HIGH first");
            }
        } else if (strncmp(arg, "LOW", 3) == 0) {
            fc_esccal(0);
            cmd_reply(out, "ESCCAL: 1000 us on all 4 - min being stored, wait "
                           "for the confirm beeps. Unplug the pack before props "
                           "go back on.");
        } else {
            cmd_reply(out, "ESCCAL,HIGH -> ESCCAL,YES -> plug pack, beeps -> "
                           "ESCCAL,LOW. Props OFF.");
        }
        return;
    }

    /* ===== P4 params CLI: GET[,name] | SET,name,value | SAVE =====
     * SET applies live params on the next flight-loop cycle; (boot)
     * params need SAVE + power-cycle. SET is allowed while armed on
     * purpose - live tuning is the point - but SAVE writes flash and
     * is not. */
    /* Separator is a comma OR a space, for both GET and SET. It was
     * comma-only, which is fine once you know it and a wall if you do
     * not: "GET alt_thr_db" fell through to "unknown command", which
     * reads as "this firmware cannot fetch one param" rather than "wrong
     * punctuation". Both spellings now work; the HELP line shows the
     * comma form because it is what the rest of the console uses. */
    if (strncmp(line, "GET", 3) == 0 &&
        (line[3] == '\0' || line[3] == ',' || line[3] == ' ')) {
        const char *arg = line + 3;
        while (*arg == ',' || *arg == ' ' || *arg == '\t') arg++;
        char name[32];
        if (*arg && sscanf(arg, "%31[^, \t]", name) == 1) {
            float v;
            if (params_get(name, &v) == 0) {
                cmd_reply(out, "%s = %.4f", name, (double)v);
            } else {
                cmd_reply(out, "GET: unknown param '%s'", name);
            }
        } else {
            int n = params_count();
            for (int i = 0; i < n; i++) {
                const char *nm; float v, lo, hi; bool boot;
                params_describe(i, &nm, &v, &lo, &hi, &boot);
                cmd_reply(out, "%-14s = %9.4f  [%g..%g]%s", nm, (double)v,
                          (double)lo, (double)hi, boot ? "  (boot)" : "");
            }
        }
        return;
    }

    if (strncmp(line, "SET,", 4) == 0 || strncmp(line, "SET ", 4) == 0) {
        char name[32];
        float v;
        /* " " skips leading blanks, %[^, \t] takes the name, %*[, \t]
         * eats whichever separator was used (one or more), %f the value.
         * Accepts SET,name,value / SET name value / SET, name , value. */
        if (sscanf(line + 4, " %31[^, \t]%*[, \t]%f", name, &v) != 2) {
            cmd_reply(out, "SET,<name>,<value>  (a space works too; "
                           "names: GET)");
            return;
        }
        int r = params_set(name, v);
        if (r == -1) {
            cmd_reply(out, "SET: unknown param '%s' (names: GET)", name);
        } else if (r == -2) {
            cmd_reply(out, "SET: %s value out of range (bounds: GET)", name);
        } else {
            bool boot = false;
            for (int i = 0; i < params_count(); i++) {
                const char *nm;
                bool b;
                params_describe(i, &nm, NULL, NULL, NULL, &b);
                if (strcmp(nm, name) == 0) { boot = b; break; }
            }
            cmd_reply(out, "%s = %.4f %s", name, (double)v,
                      boot ? "(SAVE + power-cycle to apply)"
                           : "(live, unsaved - SAVE to persist)");
        }
        return;
    }

    if (strncmp(line, "SAVE", 4) == 0) {
        if (refuse_if_armed(out, "SAVE")) return;
        int r = params_save();
        cmd_reply(out, r == 0 ? "params saved to NVS"
                              : "SAVE failed (err %d)", -r);
        return;
    }

    /* PARAMS,DEFAULT - put the whole table back to compiled defaults,
     * IN RAM. Deliberately not persisted: a mistyped reset should cost a
     * reboot, not a tuning session.
     *
     * Exists because the only way back from a bad blob was retyping every
     * SET line by hand, and a recovery that tedious does not get used -
     * which is how a board keeps flying on values nobody chose. */
    if (strncmp(line, "PARAMS", 6) == 0) {
        const char *arg = line + 6;
        while (*arg == ',' || *arg == ' ') arg++;
        if (strncmp(arg, "DEFAULT", 7) != 0) {
            cmd_reply(out, "usage: PARAMS,DEFAULT "
                           "(resets all params IN RAM; SAVE to persist)");
            return;
        }
        if (refuse_if_armed(out, "PARAMS,DEFAULT")) return;
        params_reset_defaults();
        cmd_reply(out, "all %d params reset to defaults IN RAM. "
                       "GET to review, SAVE to persist, or reboot to "
                       "discard.", params_count());
        return;
    }

    /* ===== INA226 pack voltage (voltage-only install) ===== */
    if (strncmp(line, "VBAT", 4) == 0) {
        if (!ina226_present()) {
            cmd_reply(out, "ina226: not detected at boot on bus B or A "
                           "(check wiring, then power-cycle)");
            return;
        }
        uint32_t mv = 0;
        esp_err_t err = ina226_read_bus_mv(&mv);
        if (err == ESP_OK) {
            cmd_reply(out, "vbat: %u.%03u V (ina226 @0x40 bus %s; 0 = VBUS "
                           "not wired)",
                      (unsigned)(mv / 1000), (unsigned)(mv % 1000),
                      ina226_bus());
        } else {
            cmd_reply(out, "ina226: read failed (%s)", esp_err_to_name(err));
        }
        return;
    }

    /* ===== ACC: watch the earth-frame acceleration live =====
     *
     * WHY THIS EXISTS. accn_ms2/acce_ms2 (fusion.c) rotate body accel
     * into the earth frame for the 6-state horizontal filter. Two walk
     * logs disagreed about the sign of the EAST component - s162 gave
     * r_east -0.733 against GPS Doppler, s164 gave +0.837, on the SAME
     * binary and with the board held the same way (yaw 88.5 vs 88.2 deg
     * while walking, a 0.3 deg difference). A sign error in a line of C
     * cannot change between two logs, so the cause is elsewhere and
     * neither log can settle it: both are hand-carried walks where
     * footfall and hand rotation swamp the signal, and GPS velocity is
     * a noisy reference.
     *
     * THE TEST THIS ENABLES, and it needs no GPS, no props and no
     * walking: a STATIONARY board is not accelerating, whatever
     * direction it points. So while the board sits still - level, or
     * tilted to any angle, or rotated to any heading - accn and acce
     * must both stay near ZERO. Gravity is vertical in the earth frame
     * and lands entirely in accz, so it must never leak into the
     * horizontal pair. If it does, the rotation is wrong, and HOW it
     * leaks says what is wrong: leaking with ROLL/PITCH means the tilt
     * rotation is broken, leaking only as YAW changes means the heading
     * feeding it is.
     *
     * Prints at ~5 Hz for a fixed number of seconds so it cannot be
     * left running into a flight. */
    if (strncmp(line, "ACC", 3) == 0) {
        int secs = 20;
        const char *arg = strchr(line, ',');
        if (arg) {
            int v = atoi(arg + 1);
            if (v >= 1 && v <= 120) secs = v;
        }
        cmd_reply(out, "acc: %d s. Hold the board STILL and tilt it "
                       "every which way, then turn it slowly all the way "
                       "round. Nothing prints until the end.", secs);

        /* NOTHING is printed during the run, on purpose. The first
         * version of this dumped 5 rows a second of seven columns and
         * expected a human to watch them, which is not a test anyone can
         * actually perform. The board has the numbers; the board should
         * do the arithmetic.
         *
         * A still board is not accelerating, so accn/acce must stay near
         * zero at EVERY attitude - gravity is vertical in the earth
         * frame and belongs entirely to accz. If it leaks, WHAT it
         * tracks says what is broken, so both correlations are
         * accumulated and the larger one names the culprit:
         *   leaks with TILT    -> the roll/pitch rotation is wrong
         *   leaks with HEADING -> the yaw feeding it is wrong (the
         *                         magnetometer), which is the standing
         *                         suspect for the s162/s164 sign
         *                         disagreement. */
        float worst_n = 0.0f, worst_e = 0.0f;
        double s_h = 0, s_t = 0, s_y = 0, s_ht = 0, s_hy = 0;
        double s_tt = 0, s_yy = 0, s_hh = 0;
        int n = 0;
        float yaw0 = global_nav_state.yaw;
        for (int i = 0; i < secs * 20; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
            float an = global_nav_state.accn_ms2;
            float ae = global_nav_state.acce_ms2;
            if (fabsf(an) > fabsf(worst_n)) worst_n = an;
            if (fabsf(ae) > fabsf(worst_e)) worst_e = ae;

            /* h = how much horizontal accel is leaking right now.
             * t = how far from level. y = how far the heading has
             * turned from where it started, unwrapped to +-180. */
            double h = sqrt((double)an * (double)an + (double)ae * (double)ae);
            double t = (double)fabsf(global_nav_state.roll)
                     + (double)fabsf(global_nav_state.pitch);
            double y = (double)global_nav_state.yaw - (double)yaw0;
            while (y > 180.0) y -= 360.0;
            while (y < -180.0) y += 360.0;
            y = fabs(y);

            s_h += h; s_t += t; s_y += y;
            s_ht += h * t; s_hy += h * y;
            s_tt += t * t; s_yy += y * y; s_hh += h * h;
            n++;
        }

        double r_t = 0.0, r_y = 0.0;
        if (n > 10) {
            double dh = s_hh - s_h * s_h / n;
            double dt = s_tt - s_t * s_t / n;
            double dy = s_yy - s_y * s_y / n;
            if (dh > 1e-9 && dt > 1e-9)
                r_t = (s_ht - s_h * s_t / n) / sqrt(dh * dt);
            if (dh > 1e-9 && dy > 1e-9)
                r_y = (s_hy - s_h * s_y / n) / sqrt(dh * dy);
        }

        /* ---- PHASE 2: the SIGN, which phase 1 cannot see ----
         *
         * A still board reads ~0 on both horizontal axes whether the
         * second earth axis is east or west, so the tilt test above is
         * blind to a sign convention error. Only MOTION reveals it.
         *
         * The board knows its own heading, so no compass and no GPS are
         * needed: push it along its own NOSE, and the earth-frame accel
         * must point where the nose points. With yaw t measured
         * clockwise from magnetic north, a nose-forward push should give
         *      NED  (east +ve):  accn ~ +cos(t),  acce ~ +sin(t)
         *      NWU  (west +ve):  accn ~ +cos(t),  acce ~ -sin(t)
         * The north component agrees either way - which is exactly why
         * both walk logs agreed on north and fought about east.
         *
         * Samples are taken only while the push is strong enough to
         * dominate hand tremor, and only the east projection is scored:
         * a positive score means east, negative means west. */
        double east_score = 0.0;
        int pushes = 0;
        cmd_reply(out, "acc: now PUSH the board along its own nose, "
                       "sharply, ~10 times. Turn to a different heading "
                       "every few pushes. %d s.", secs);
        for (int i = 0; i < secs * 20; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
            float an = global_nav_state.accn_ms2;
            float ae = global_nav_state.acce_ms2;
            double mag = sqrt((double)an * (double)an + (double)ae * (double)ae);
            if (mag < 2.0) continue;              /* not a deliberate push */
            double t = (double)global_nav_state.yaw * (M_PI / 180.0);
            double cn = cos(t), ce = sin(t);
            /* Only score pushes that are genuinely along the nose, so a
             * sideways shove cannot vote. */
            double along = ((double)an * cn + (double)ae * ce) / mag;
            if (along < 0.5) continue;
            east_score += ((double)ae * ce) / mag;
            pushes++;
        }

        bool pass = fabsf(worst_n) < 1.0f && fabsf(worst_e) < 1.0f;
        cmd_reply(out, "");
        cmd_reply(out, "ACC RESULT: %s", pass ? "PASS" : "FAIL");
        cmd_reply(out, "  worst leak   north %+.2f   east %+.2f m/s^2  "
                       "(want both under 1.00)", (double)worst_n, (double)worst_e);
        if (pass) {
            cmd_reply(out, "  gravity is not leaking into north/east - the "
                           "rotation is correct.");
        } else {
            cmd_reply(out, "  leak vs TILT r=%+.2f   leak vs HEADING r=%+.2f",
                      r_t, r_y);
            cmd_reply(out, "  -> blame the %s",
                      (fabs(r_t) >= fabs(r_y))
                      ? "TILT rotation (roll/pitch handling in fusion.c)"
                      : "HEADING (magnetometer yaw feeding the rotation)");
        }
        if (pushes < 5) {
            cmd_reply(out, "  SIGN: not enough pushes (%d) - inconclusive.",
                      pushes);
        } else {
            cmd_reply(out, "  SIGN: %d pushes, east score %+.2f -> the "
                           "second axis is %s",
                      pushes, east_score / pushes,
                      (east_score > 0.0) ? "EAST (NED, as the code has it)"
                                         : "WEST (NED needs it NEGATED)");
        }
        return;
    }

    /* ===== I2C bus scan =====
     * The boot scan prints to the USB console, and on USB the sensor 3V3
     * rail is unpowered, so that scan can never answer a question about a
     * real chip. This one comes back over whichever console asked.
     * Reports BOTH buses - A (DPS310) and B (mag, ToF, INA226, MS5611) -
     * hence the wider buffer. */
    if (strncmp(line, "I2CSCAN", 7) == 0) {
        if (refuse_if_armed(out, "I2CSCAN")) return;
        char buf[224];   /* matches cmd_reply(), which truncates above this */
        i2c_all_scan_str(buf, sizeof(buf));
        cmd_reply(out, "%s", buf);
        return;
    }

    /* ===== VL53L0X ground distance ===== */
    if (strncmp(line, "TOF", 3) == 0) {
        /* Not found at boot? Retry NOW rather than making a wiring change
         * cost a power cycle each time. */
        if (!vl53l0x_present() && !vl53l0x_reprobe()) {
            cmd_reply(out, "vl53l0x: no answer at 0x29 (re-probed just now). "
                           "Run I2CSCAN - if 0x29 is missing there it is "
                           "wiring, not firmware.");
            return;
        }

        /* "TOF,<n>": n DISTINCT measurements, then their spread.
         *
         * The spread is what decides whether this sensor can be an
         * altitude reference at all, and one-shot TOF cannot show it. A
         * mean that is correct while individual reads swing 100 mm is
         * not a height, it is noise the controller would chase. Sigma
         * grows as the return weakens, so it is also the number that
         * says how much of any new range is real and how much is the
         * signal-rate limit having been lowered past what the optics
         * can actually support.
         *
         * Every sample waits for vl53l0x_sample_seq() to MOVE first.
         * vl53l0x_read_raw() re-reads the result registers WITHOUT
         * clearing the interrupt, so back-to-back calls hand back the
         * same measurement - a tight loop would report sigma 0 off one
         * reading counted n times, which is the most convincing wrong
         * answer this command could give. The seq counter is the only
         * thing that says a new measurement landed; it is bumped by
         * vl53l0x_sample() on the fusion side, including on the blanking
         * paths, so a chip that stops ranging shows up as a timeout here
         * rather than as a frozen number.
         *
         * The chip DELIVERS at 1000/tof_budget_ms (~5 Hz at the 200 ms
         * default), not at the ~9 Hz poll rate, so n=100 takes ~20 s.
         * That blocks rx_line_task for the duration - the 50 Hz stream
         * runs on stream_task and is unaffected - and it is refused
         * while armed, which is the guard that matters. */
        if (line[3] == ',') {
            if (refuse_if_armed(out, "TOF,n")) return;

            long want = strtol(line + 4, NULL, 10);
            if (want < 2)   want = 2;
            if (want > 200) want = 200;

            uint32_t n = 0, valid11 = 0, notarget = 0;
            uint16_t lo = 0xFFFF, hi = 0;
            double sum = 0.0, sumsq = 0.0, sig_sum = 0.0, amb_sum = 0.0;
            uint32_t last_seq = vl53l0x_sample_seq();

            for (long i = 0; i < want; i++) {
                /* 2 s is several measurement periods even at the 400 ms
                 * budget ceiling, so a timeout here means the chip
                 * stopped ranging rather than that it was slow. Report
                 * what was collected instead of burning the rest. */
                int64_t t0 = esp_timer_get_time();
                while (vl53l0x_sample_seq() == last_seq &&
                       esp_timer_get_time() - t0 < 2000000)
                    vTaskDelay(pdMS_TO_TICKS(5));
                if (vl53l0x_sample_seq() == last_seq) break;
                last_seq = vl53l0x_sample_seq();

                uint16_t mm = 0;
                uint8_t  st = 255;
                float    sg = 0.0f, ab = 0.0f;
                if (vl53l0x_read_raw(&mm, &st, &sg, &ab) != ESP_OK) continue;

                n++;
                if (st == 11) valid11++;
                sig_sum += (double)sg;
                amb_sum += (double)ab;

                /* 8190/8191 is the chip saying nothing came back. It is
                 * COUNTED, never averaged: folding an 8 m sentinel into
                 * a mean would invent a distance nothing measured. */
                if (mm >= VL53L0X_NO_RETURN_MM) { notarget++; continue; }

                if (mm < lo) lo = mm;
                if (mm > hi) hi = mm;
                sum   += (double)mm;
                sumsq += (double)mm * (double)mm;
            }

            uint32_t ranged = n - notarget;
            if (ranged == 0) {
                cmd_reply(out, "tof x%lu: NONE ranged (%lu no-target, "
                               "%lu status-11). %s",
                          (unsigned long)n, (unsigned long)notarget,
                          (unsigned long)valid11,
                          n == 0 ? "seq never moved - the chip has stopped "
                                   "ranging" : "chip is alive, nothing in "
                                   "range");
                return;
            }
            double mean = sum / (double)ranged;
            double var  = sumsq / (double)ranged - mean * mean;
            if (var < 0.0) var = 0.0;    /* rounding only, at n this small */
            cmd_reply(out, "tof x%lu: mean %.1f mm, sigma %.1f, min %u, "
                           "max %u, spread %u; %lu no-target, %lu status-11; "
                           "signal %.3f MCPS, ambient %.3f MCPS",
                      (unsigned long)ranged, mean, sqrt(var),
                      (unsigned)lo, (unsigned)hi, (unsigned)(hi - lo),
                      (unsigned long)notarget, (unsigned long)valid11,
                      sig_sum / (double)n, amb_sum / (double)n);
            return;
        }

        /* Raw, unfiltered: the dashboard shows the filtered cache, so
         * when the card says "no target" this is what says WHY - status
         * 11 is a valid measurement, anything else is the chip refusing
         * to stand behind the number next to it.
         *
         * signal/ambient are printed next to the distance because the
         * distance on its own cannot be sanity-checked. Return power
         * goes as 1/d^2, so a genuinely short target is BRIGHT; a short
         * number carrying a weak signal is the chip localising stray
         * light. Reading them side by side is what tells the two apart. */
        uint16_t mm = 0;
        uint8_t  st = 255;
        float sig = 0.0f, amb = 0.0f;
        esp_err_t err = vl53l0x_read_raw(&mm, &st, &sig, &amb);
        if (err == ESP_OK) {
            cmd_reply(out, "tof: %u mm raw, range status %u (11 = the chip "
                           "vouches for it; ADVISORY, it no longer gates "
                           "the reading); signal %.3f MCPS, ambient %.3f "
                           "MCPS; cached %u mm",
                      mm, st, (double)sig, (double)amb, vl53l0x_cached_mm());
        } else {
            cmd_reply(out, "vl53l0x: read failed (%s)", esp_err_to_name(err));
        }
        return;
    }

    /* ===== PMW3901 optical flow (passive) =====
     *   FLOW        status: state, identity, MISO check, init, counters,
     *               the newest sample WITH its age
     *   FLOW,<s>    s seconds (1-30) of 100 ms windows - the hand-wave
     *               bench test. dx/dy are summed from the driver's running
     *               totals, so a window is exact however the samples fell
     *   FLOW,FORCE  skip the MISO release check once (see pmw3901.h)
     * FLOW,<s> blocks this task like TOF,n does, and is refused while armed
     * for the same reason. */
    if (strncmp(line, "FLOW", 4) == 0) {
        char buf[224];   /* matches cmd_reply(), which truncates above this */
        if (line[4] == ',' && strncmp(line + 5, "FORCE", 5) == 0) {
            if (refuse_if_armed(out, "FLOW,FORCE")) return;
            pmw3901_force_next_init();
            cmd_reply(out, "flow: the next init SKIPS the MISO release check, "
                           "once. Only safe if the SD module tri-states MISO.");
            return;
        }
        if (line[4] == ',') {
            if (refuse_if_armed(out, "FLOW,n")) return;
            long secs = strtol(line + 5, NULL, 10);
            if (secs < 1)  secs = 1;
            if (secs > 30) secs = 30;
            pmw3901_totals_t a, b;
            pmw3901_totals(&a);
            if (!a.running) {
                pmw3901_status_line(0, buf, sizeof(buf));
                cmd_reply(out, "%s", buf);
                return;
            }
            cmd_reply(out, "flow: %ld s of 100 ms windows. dx/dy = counts in "
                           "the window, SENSOR frame, not remapped", secs);
            for (long i = 1; i <= secs * 10; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                pmw3901_totals(&b);
                pmw3901_sample_t s;
                const bool have = pmw3901_latest(&s);
                if (b.seq == a.seq || !have) {
                    /* Silence is reported as silence, never as zero motion. */
                    cmd_reply(out, "flow %ld.%lds: NO NEW SAMPLES (%s)",
                              i / 10, i % 10, b.running ? "stalled" : "stopped");
                } else {
                    cmd_reply(out, "flow %ld.%lds: dx %+5ld dy %+5ld | %lu valid "
                                   "%lu lost | squal %3u shutter %5u raw %u..%u "
                                   "| flags 0x%03X",
                              i / 10, i % 10,
                              (long)(b.sum_dx - a.sum_dx),
                              (long)(b.sum_dy - a.sum_dy),
                              (unsigned long)(b.n_valid - a.n_valid),
                              (unsigned long)(b.n_lost - a.n_lost),
                              s.b.squal, s.b.shutter, s.b.raw_min, s.b.raw_max,
                              s.flags);
                }
                a = b;
            }
            return;
        }
        for (int part = 0; pmw3901_status_line(part, buf, sizeof(buf)); part++)
            cmd_reply(out, "%s", buf);
        return;
    }

    /* ===== P4 blackbox: BBINFO | BBDUMP | BBERASE ===== */
    if (strncmp(line, "BBINFO", 6) == 0) {
        char buf[200];
        blackbox_info(buf, sizeof(buf));
        cmd_reply(out, "%s", buf);
        return;
    }

    /* SD blackbox. SDFORMAT marks a card so the recorder will
     * use it - a card without the mark is never written, so a card that
     * still holds something else cannot be overwritten by accident. On a
     * card already marked it starts a fresh log area past the old one;
     * BBERASE is the same operation under its old name. Both instant. */
    if (strncmp(line, "SDFORMAT", 8) == 0 || strncmp(line, "BBERASE", 7) == 0) {
        if (refuse_if_armed(out, line[0] == 'S' ? "SDFORMAT" : "BBERASE")) return;
        int r = blackbox_format();
        if (r == 0)       cmd_reply(out, "blackbox: SD card marked, log starts fresh");
        else if (r == -3) cmd_reply(out, "blackbox: no SD card answering - check wiring/5V");
        else              cmd_reply(out, "blackbox: format failed (%d)", r);
        return;
    }

    if (strncmp(line, "BBDUMP", 6) == 0) {
        if (refuse_if_armed(out, "BBDUMP")) return;
        if (out != CMD_OUT_USB) {
            /* A dump line is ~1 KB; the telemetry console path truncates
             * at 256. WiFi dump is the planned C6-side follow-up. */
            cmd_reply(out, "BBDUMP: USB console only (for now)");
            return;
        }
        /* "BBDUMP,<seq>": re-dump one sector - the pull tool's retry
         * path for lines the USB console dropped. No start/done chatter
         * so the reply is exactly 6 BB lines + BBEOF. */
        if (line[6] == ',') {
            blackbox_dump_seq((uint32_t)strtoul(line + 7, NULL, 10),
                              bbdump_sink_usb);
            return;
        }
        cmd_reply(out, "BBDUMP start - capture with tools/blackbox_pull.py");
        blackbox_dump(bbdump_sink_usb);
        cmd_reply(out, "BBDUMP done");
        return;
    }

    /* HELP: the dashboard has no man page, so the firmware is it. */
    if (strncmp(line, "HELP", 4) == 0) {
        /* Two replies, not one: cmd_reply's buffer is 224 bytes and this
         * list passed that a while ago - everything from I2CSCAN on was
         * being silently vsnprintf'd away, so the commands least likely
         * to be remembered were exactly the ones HELP had stopped
         * mentioning. */
        cmd_reply(out, "commands: STATS | GYROCAL | LEVELCAL | "
                       "MAGSET,<3 or 12 floats> | "
                       "MAGCAP,0|1 | STREAM,0|1 | PID,<6> (dashboard only) | "
                       "MOT,<1-4>,<us>[,ms] | MOT,0 | "
                       "ESCPASS,0|1 | ESCCAL,HIGH|YES|LOW");
        cmd_reply(out, "     ... GET[,name] | SET,name,val | SAVE | "
                       "PARAMS,DEFAULT | "
                       "VBAT | TOF[,n] | FLOW[,s|,FORCE] | I2CSCAN | "
                       "BBINFO | BBDUMP[,seq] | BBERASE | SDFORMAT | "
                       "GSEC[,CLEAR] | ACC[,s] | HELP");
        return;
    }

    cmd_reply(out, "unknown command: %.32s (try HELP)", line);
}

/* Bridge for commands arriving over the telemetry UART from the C6. */
static void telem_cmd_handler(const char *line)
{
    char buf[RX_LINE_MAX];
    strlcpy(buf, line, sizeof(buf));
    exec_command(buf, CMD_OUT_TELEM);
}

/* ================= RX: USB console line parser ================= */
static void handle_line(char *line)
{
#if HIL_MODE
    if (line[0] == 'S' && line[1] == ',') {
        float v[9];
        if (sscanf(line, "S,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &v[0], &v[1], &v[2], &v[3], &v[4],
                   &v[5], &v[6], &v[7], &v[8]) == 9) {
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                global_nav_state.hil_sens.ax = v[0];
                global_nav_state.hil_sens.ay = v[1];
                global_nav_state.hil_sens.az = v[2];
                global_nav_state.hil_sens.gx = v[3];
                global_nav_state.hil_sens.gy = v[4];
                global_nav_state.hil_sens.gz = v[5];
                global_nav_state.hil_sens.mx = v[6];
                global_nav_state.hil_sens.my = v[7];
                global_nav_state.hil_sens.mz = v[8];
                global_nav_state.hil_sens.fresh = true;
                xSemaphoreGive(state_mutex);
            }
        }
        return;
    }
    if (line[0] == 'R' && line[1] == ',') {
        int roll, pitch, thr, yaw, arm;
        if (sscanf(line, "R,%d,%d,%d,%d,%d",
                   &roll, &pitch, &thr, &yaw, &arm) == 5) {
            uint16_t ch[IBUS_CHANNELS];
            for (int i = 0; i < IBUS_CHANNELS; i++) ch[i] = 1500;
            ch[CH_ROLL]     = (uint16_t)roll;
            ch[CH_PITCH]    = (uint16_t)pitch;
            ch[CH_THROTTLE] = (uint16_t)thr;
            ch[CH_YAW]      = (uint16_t)yaw;
            ch[CH_ARM]      = (uint16_t)arm;
            fc_hil_inject_rc(ch, false);
        }
        return;
    }
#endif

    exec_command(line, CMD_OUT_USB);
}

static void rx_line_task(void *pv)
{
    static char line[RX_LINE_MAX];
    int pos = 0;
    uint8_t buf[64];

    while (1) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf),
                                           pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (pos > 0) {
                    line[pos] = 0;
                    handle_line(line);
                    pos = 0;
                }
            } else if (pos < RX_LINE_MAX - 1) {
                line[pos++] = c;
            } else {
                pos = 0;   /* overlong line: drop it */
            }
        }
    }
}

void debug_stream_init(void)
{
    /* RX driver for reading commands. Console TX (printf) keeps using
     * the non-driver USB-Serial-JTAG path; only reads go through this. */
    usb_serial_jtag_driver_config_t cfg =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "usb_serial_jtag driver install failed: %d", err);
    }
    /* Commands from the C6 dashboard land in the same interpreter the
     * USB console uses, with replies routed back over the telemetry
     * link instead of to the USB port. */
    telemetry_set_cmd_handler(telem_cmd_handler);

    /* Route stdio through the driver so printf and reads share one path */
    usb_serial_jtag_vfs_use_driver();

#if HIL_MODE
    ESP_LOGW(TAG, "=== HIL MODE ACTIVE - DO NOT FLY ===");
#endif

    xTaskCreatePinnedToCore(stream_task,  "dbg_stream", 4096, NULL, 2, NULL,
                            FC_CORE_SUPPORT);
    xTaskCreatePinnedToCore(rx_line_task, "dbg_rx",     4096, NULL, 3, NULL,
                            FC_CORE_SUPPORT);
    ESP_LOGI(TAG, "debug stream started (%d Hz)", STREAM_RATE_HZ);
}
