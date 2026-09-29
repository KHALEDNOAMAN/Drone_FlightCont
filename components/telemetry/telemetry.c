#include "telemetry.h"
#include "telem_line.h"
#include "fc_cores.h"
#include "params.h"
#include "ina226.h"
#include "vl53l0x.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "TELEMETRY";
#define RX_BUF_SIZE 1024

static telemetry_cmd_fn s_cmd_handler = NULL;
/* Bytes the UART driver actually accepted - the tx= half of the
 * "telem link" health line in telemetry_rx_task. */
static volatile uint32_t s_tx_bytes = 0;

void telemetry_set_cmd_handler(telemetry_cmd_fn fn) { s_cmd_handler = fn; }

void telemetry_write_console(const char *line)
{
    char out[256];
    int n = snprintf(out, sizeof(out), "#%s\n", line);
    if (n > 0) {
        if (n >= (int)sizeof(out)) n = sizeof(out) - 1;   /* truncated */
        uart_write_bytes(TELEMETRY_UART_PORT_NUM, out, n);
    }
}

void telemetry_write_console_block(const char *text)
{
    char line[200];
    int  pos = 0;
    for (const char *p = text; *p; p++) {
        if (*p == '\n' || pos >= (int)sizeof(line) - 1) {
            line[pos] = 0;
            if (pos > 0) telemetry_write_console(line);
            pos = 0;
            if (*p != '\n') p--;      /* re-handle the char that overflowed */
        } else if (*p != '\r') {
            line[pos++] = *p;
        }
    }
    if (pos > 0) { line[pos] = 0; telemetry_write_console(line); }
}

/* CSV telemetry line, 4 Hz. Fields 0-38 are FROZEN (historical layout the
 * MATLAB scripts and old dashboards indexed); everything new is APPENDED.
 *
 * Index map (0-based):
 *   0 armed  1-3 roll,pitch,yaw  4-6 des_r,des_p,des_yawrate
 *   7-12 zeros (legacy err/pid slots)
 *   13 temp_c  14 hum(0)  15 pressure_pa  16 alt_m
 *   17 lat  18 lon  19 sats  20 time(0)
 *   21-24 m1..m4 pwm  25-37 zeros (legacy)  38 loop_dt
 *   --- appended for the C6 dashboard ---
 *   39-41 gyro x,y,z dps   42-44 accel x,y,z g
 *   45 rc_throttle us
 *   46-48 rate PID kp,ki,kd   49-51 yaw PID kp,ki,kd  (ACTIVE gains)
 *   52 sensor_health bitmask  53 gps fix_type  54 arm_blocked bitmask
 *   55-58 raw RC roll,pitch,yaw,arm us
 *   59 rc_frozen_frames  60 rc_link_ok
 *   --- appended ---
 *   61 vbat_mv (INA226 pack voltage; 0 = chip absent / read failing)
 *   --- appended ---
 *   62 tof_mm  (VL53L0X ground distance; 0 = chip absent / read failing,
 *               65535 = alive but nothing in range)
 *   --- appended (P7 stages 1+2) ---
 *   63 alt_est_m   vertical filter altitude, m above its ground datum.
 *                  NOT the same zero as field 16 (baro altitude MSL).
 *   64 vz_ms       climb rate, m/s, positive = UP
 *   65 accz_bias   accelerometer Z bias the filter learned, m/s^2
 *   66 alt_sp_m    alt-hold setpoint, same datum as 63 (0 when off)
 *   67 vert_flags  bit0 estimator valid, bit1 alt-hold engaged,
 *                  bit2 the AUTOMATIC POLICY asked for it (alt_hold_auto
 *                  on, estimate valid, airborne, throttle up - NOT the
 *                  CH6 switch, which is bit6),
 *                  bit3 ToF ok, bit4/5 velocity hold engaged/wanted,
 *                  bit6 CH6 switch position, diagnostic only.
 *                  Engaged=0 while wanted=1 is a REFUSED engage - the
 *                  only way to see that a gate blocked it. Engaged=1
 *                  while wanted=0 is impossible and means the flags are
 *                  measuring different things again.
 *   --- appended (MS5611 control barometer) ---
 *   68 baro2_pa    MS5611 pressure, Pa. 0 = part not fitted.
 *   70 alt_ref_m  the author's altitude reference (alt_ref.c): ToF when it
 *                 can see the ground, baro-relative when it cannot,
 *                 re-referenced onto the ToF only after 5 s of
 *                 sustained disagreement. DISPLAY ONLY - it drives
 *                 nothing. Compare it against alt_est_m (field 63) to
 *                 see whether it deserves to.
 *   --- appended ---
 *   71 alt_ref_valid  1 = field 70 is meaningful right now; 0 = it is
 *                 coasting on a barometer that has stopped answering.
 *                 A display-only number that shows a stale value
 *                 without saying so is worse than showing nothing -
 *                 read this before reading 70.
 *   69 baro2_alt_m MS5611 altitude by the SAME formula and the same
 *                  hard-coded sea-level constant as field 16, so the two
 *                  are comparable to each other even though neither is a
 *                  true MSL altitude. Liveness is bit 4 of field 52 -
 *                  a held value and a fresh one look identical here.
 *
 * NOTE on 17/18: lat/lon are sent as DEGREES at %.7f. nav_state holds
 * int32 1e-7 deg and telemetry.c scales on the way out,
 * because 17 and 18 are inside the frozen 0-38 layout. */
static void telemetry_send_packet(const nav_state_t *state) {
    if (state == NULL) return;

    char buffer[768];

    int len = snprintf(buffer, sizeof(buffer),
        "%d,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "0.00,0.00,0.00,"
        "0.00,0.00,0.00,"
        "%.1f,%.1f,%.1f,%.1f,"
        "%.7f,%.7f,%d,0,"
        "%d,%d,%d,%d,"
        "0.00,0.00,0.00,"
        "0.00,0.00,0.00,"
        "0,0,0,0,0,0,0,"
        "%.4f,"
        "%.3f,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,"
        "%u,"
        "%.3f,%.4f,%.4f,"
        "%.3f,%.4f,%.4f,"
        "%u,%u,%u,"
        "%u,%u,%u,%u,%lu,%u,"
        "%u,%u,"
        /* P7 stages 1+2: fields 63-67 */
        "%.3f,%.3f,%.3f,%.3f,%u,"
        /* MS5611 control barometer: fields 68-69 */
        "%.1f,%.3f,"
        /* the author's altitude reference (alt_ref.c): field 70. Appended at
         * the TAIL so every existing index is untouched - the 0-38 block
         * is frozen for MATLAB and the C6 index. DISPLAY ONLY.
         *
         * Field 71 is its LIVENESS. Appended, because 70 on
         * its own is a value with no way to tell a fresh number from
         * one coasting on a dead barometer. */
        "%.3f,%u\n",

        state->is_armed,
        (double)state->roll,
        (double)state->pitch,
        (double)state->yaw,

        (double)state->des_roll,
        (double)state->des_pitch,
        (double)state->des_yaw,

        (double)state->temp_c,
        0.0,   /* humidity: not measured */
        (double)state->pressure_pa,
        (double)state->altitude_m,
        /* nav_state holds 1e-7 deg; CSV fields 17 and 18
         * are part of the FROZEN 0-38 layout that MATLAB and the C6
         * dashboard index, so they keep carrying degrees. Scaling here
         * rather than changing the wire format is what lets this change
         * ship without a C6 reflash. */
        (double)state->lat * 1.0e-7,
        (double)state->lon * 1.0e-7,
        state->sats,

        state->m1_pwm,
        state->m2_pwm,
        state->m3_pwm,
        state->m4_pwm,

        (double)state->loop_dt,

        (double)state->gyro_x,
        (double)state->gyro_y,
        (double)state->gyro_z,
        (double)state->acc_x,
        (double)state->acc_y,
        (double)state->acc_z,
        state->rc_throttle,
        (double)state->pid_active.kp_rate,
        (double)state->pid_active.ki_rate,
        (double)state->pid_active.kd_rate,
        (double)state->pid_active.kp_yaw,
        (double)state->pid_active.ki_yaw,
        (double)state->pid_active.kd_yaw,
        state->sensor_health,
        state->fix_type,
        state->arm_blocked,
        state->rc_roll,
        state->rc_pitch,
        state->rc_yaw,
        state->rc_arm,
        (unsigned long)state->rc_frozen_frames,
        state->rc_link_ok,
        /* Not from nav_state: the INA226 and VL53L0X caches are their own
         * singletons, so pack voltage and ground distance need no field in
         * nav_state_t. */
        ina226_cached_mv(),
        vl53l0x_cached_mm(),
        (double)state->alt_est_m,
        (double)state->vz_ms,
        (double)state->accz_bias,
        (double)state->alt_sp_m,
        state->vert_flags,
        /* MS5611 control barometer. Zero when the part is not fitted,
         * which is the normal state until it is ordered. */
        (double)state->baro2_press_pa,
        (double)state->baro2_alt_m,
        (double)state->alt_ref_m,
        state->alt_ref_valid
    );

    if (len > 0 && len < sizeof(buffer)) {
        { int w = uart_write_bytes(TELEMETRY_UART_PORT_NUM, buffer, len); if (w > 0) s_tx_bytes += (uint32_t)w; }
    }
}

/* --- RX: commands from the C6 (dashboard console + PID uplink) ---
 * Line-accumulating parser: a command split across two uart_read_bytes
 * calls used to be lost by the old strtok-per-chunk approach. Bytes are
 * buffered until a newline, then the whole line is handled. */
static void handle_telem_line(char *line) {
    // "PID,pl,il,dl,py,iy,dy" -> params_set on the six rate/yaw gains;
    // flight_task picks g_params up on its next cycle.
    if (strncmp(line, "PID,", 4) == 0) {
        float v[6];
        if (sscanf(line + 4, "%f,%f,%f,%f,%f,%f",
                   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
            ESP_LOGW(TAG, "PID: expected 6 values, got: %s", line);
            return;
        }
        /* This is a network-facing input that ends up multiplying motor
         * commands. sscanf happily parses "nan"/"inf", and a NaN gain
         * NaNs the mixer. params_set bounds-checks NaN-safely, but the
         * explicit finite check keeps the reject reason readable. */
        for (int i = 0; i < 6; i++) {
            if (!isfinite(v[i])) {
                ESP_LOGW(TAG, "PID rejected: value %d not finite", i);
                return;
            }
        }
        /* P4: gains live in the params table now. The flight task
         * refreshes its pid_cfg from g_params every cycle, so this
         * applies on the next loop. SAVE (console) persists. */
        static const char *k_names[6] = {
            "rate_kp", "rate_ki", "rate_kd",
            "yaw_kp",  "yaw_ki",  "yaw_kd",
        };
        for (int i = 0; i < 6; i++) {
            if (params_set(k_names[i], v[i]) != 0) {
                ESP_LOGW(TAG, "PID rejected: %s = %f out of range",
                         k_names[i], (double)v[i]);
                return;
            }
        }
        ESP_LOGI(TAG, "PID updated via telemetry link: "
                 "rate %.3f/%.4f/%.4f yaw %.3f/%.4f/%.4f",
                 (double)v[0], (double)v[1], (double)v[2],
                 (double)v[3], (double)v[4], (double)v[5]);
    }
    /* Note: the old "ARM,x" command was removed it set a
     * flag (web_arm_request) that the flight task deliberately ignores.
     * Arming stays RC-only. */
    else if (s_cmd_handler) {
        /* Everything else (STATS, GYROCAL, MAGSET, STREAM) goes to the
         * FC's serial command interpreter, so the dashboard can drive
         * the same commands the USB console accepts. */
        s_cmd_handler(line);
    }
}

// RX Task to listen to C6
static void telemetry_rx_task(void *pvParameters) {
    uint8_t buf[128];
    /* Line hygiene lives in telem_line.c (host-tested): why the link needs
     * it is written there. */
    static telem_line_t rx;
    telem_line_init(&rx);
    /* Link health on the USB console. Added the C6 prints its
     * own "rx=N B/s" counter, so the S3 having no equivalent meant a dead
     * link could only be measured from one end - and with both ends alive
     * and the wires ohm-checked, one end is not enough to find it. Logs at
     * 1 Hz while bytes are arriving, every 10 s while silent, so it is
     * quiet enough to leave in permanently. Since it also says
     * how many bytes were dropped as noise, and what the latest ones were. */
    uint32_t rx_bytes = 0;
    TickType_t last_log = xTaskGetTickCount();

    while (1) {
        /* Returns after 20 ms with nothing more arriving, so an empty read
         * means the link has been quiet that long: a line still partial
         * then is noise (telem_line_silence). */
        int n = uart_read_bytes(TELEMETRY_UART_PORT_NUM, buf, sizeof(buf),
                                pdMS_TO_TICKS(20));
        if (n > 0) rx_bytes += (uint32_t)n;
        else       telem_line_silence(&rx);

        TickType_t now = xTaskGetTickCount();
        TickType_t period = rx_bytes ? pdMS_TO_TICKS(1000) : pdMS_TO_TICKS(10000);
        if (now - last_log >= period) {
            char hex[3 * TELEM_JUNK_HEAD + 1] = "";
            for (int k = 0; k < rx.junk_head_n; k++)
                snprintf(hex + 3 * k, sizeof(hex) - 3 * k, " %02X", rx.junk_head[k]);
            ESP_LOGI(TAG, "telem link: tx=%lu B, rx=%lu B, lines=%lu, junk=%lu B%s%s%s "
                     "(TX %d / RX %d)", (unsigned long)s_tx_bytes,
                     (unsigned long)rx_bytes, (unsigned long)rx.lines,
                     (unsigned long)rx.junk_bytes, rx.junk_head_n ? " [" : "",
                     rx.junk_head_n ? hex + 1 : "", rx.junk_head_n ? "]" : "",
                     TELEMETRY_TX_PIN, TELEMETRY_RX_PIN);
            rx_bytes = 0;
            rx.lines = 0;
            rx.junk_bytes = 0;
            rx.junk_head_n = 0;
            last_log = now;
        }

        for (int i = 0; i < n; i++) {
            char *line = telem_line_feed(&rx, buf[i]);
            if (line) handle_telem_line(line);
        }
    }
}

static void telemetry_task(void *pvParameters) {
    nav_state_t snapshot;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(250); // 4Hz

    while (1) {
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            snapshot = global_nav_state;
            xSemaphoreGive(state_mutex);
            telemetry_send_packet(&snapshot);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

int telemetry_write_raw(const char *buf, int len) {
    return uart_write_bytes(TELEMETRY_UART_PORT_NUM, buf, len);
}

void telemetry_init_and_start(void) {
    const uart_config_t uart_config = {
        .baud_rate = TELEMETRY_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(TELEMETRY_UART_PORT_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(TELEMETRY_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(TELEMETRY_UART_PORT_NUM, TELEMETRY_TX_PIN, TELEMETRY_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "Telemetry Started (CSV Mode). TX:%d RX:%d", TELEMETRY_TX_PIN, TELEMETRY_RX_PIN);


    /* 6144, not 4096: STATS measured telemetry_tx at 840 B
     * high-water on a 4096 stack - the thinnest margin of any task on the
     * board, and every task above it had >1400 B. The CSV buffer (640 B
     * then, 768 B now) plus
     * a 40-argument snprintf in telemetry_send_packet is what eats it, so
     * the margin shrinks again every time a field is appended to the CSV. */
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_tx", 6144, NULL, 3, NULL,
                            FC_CORE_SUPPORT);
    xTaskCreatePinnedToCore(telemetry_rx_task, "telemetry_rx", 4096, NULL, 3, NULL,
                            FC_CORE_SUPPORT);
}

