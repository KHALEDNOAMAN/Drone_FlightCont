#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tbsM10.h"
#include "fc_cores.h"
#include "esp_timer.h"
#include "nav_state.h" // Access to global_nav_state and state_mutex
#include "ubx.h"
#include "geo.h"
#include "gnss_guard.h"

static const char *TAG = "GPS";

/* --- the interference guard ---------------------------------------------
 *
 * Owned by gps_task, which is the only writer. The console reads a COPY
 * taken under state_mutex rather than the live struct, so a `GSEC` typed
 * mid-update cannot print half of one verdict and half of the next.
 *
 * OBSERVER ONLY for now, and that is deliberate. The guard's thresholds
 * are documented starting points, not measurements - the C/N0 spread
 * floor, the AGC delta and the noise delta all have to be re-derived from
 * a real bench log of THIS antenna before anything is allowed to act on
 * them. Publishing a verdict costs nothing if it is wrong; dropping GPS in
 * flight on a guessed threshold costs a flight. Flip
 * TBSM10_GUARD_GATES_FUSION to 1 once the numbers. */
#define TBSM10_GUARD_GATES_FUSION 0

static gg_t s_guard;
static gg_status_t s_guard_pub;   /* snapshot for the console */

/* The home point for the local tangent plane. Owned here because this is
 * the only task that writes position, so no extra lock is needed for the
 * scale factors - only the nav_state publish takes the mutex. */
static geo_home_t g_home;

/* A fix has to be at least this good before it may become the origin.
 * A home point latched off a settling first fix is worse than no home at
 * all: every later position inherits its error silently, and nothing
 * downstream can tell. 3D + gnssFixOK + hAcc under 10 m. */
#define HOME_MAX_HACC_MM 10000u

void tbsm10_clear_home(void)
{
    geo_home_clear(&g_home);
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        global_nav_state.home_valid = 0;
        global_nav_state.pos_n_m = 0.0f;
        global_nav_state.pos_e_m = 0.0f;
        global_nav_state.pos_d_m = 0.0f;
        xSemaphoreGive(state_mutex);
    }
}

/* Latch home. Called from the 250 Hz flight loop on the arm edge, which
 * sets two rules for this function:
 *   - ONE critical section, not two. The loop period is 4 ms, so a pair of
 *     20 ms waits could cost ten cycles at the exact moment the motors
 *     spin up.
 *   - NO LOGGING while the mutex is held. The result is carried out in
 *     locals and printed afterwards.
 * geo_set_home() runs inside the lock, which is fine - it is a few dozen
 * flops with no allocation and no blocking. */
bool tbsm10_capture_home(void)
{
    int32_t lat = 0, lon = 0;
    uint32_t hacc = 0;
    uint8_t fix = 0, ok = 0;
    float alt_m = 0.0f;
    bool took = false, good = false;

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        took  = true;
        lat   = global_nav_state.lat;
        lon   = global_nav_state.lon;
        hacc  = global_nav_state.gps_h_acc_mm;
        fix   = global_nav_state.fix_type;
        ok    = global_nav_state.gps_fix_ok;
        alt_m = global_nav_state.gps_alt_m;

        /* A home point latched off a settling first fix is worse than no
         * home at all: every position after it inherits the error, and
         * nothing downstream can tell. Require 3D, the module's own
         * gnssFixOK, and a real hAcc inside HOME_MAX_HACC_MM.
         *
         * NMEA builds carry neither gps_fix_ok nor hAcc, so they can never
         * latch home. That is deliberate, not an oversight - with no
         * accuracy figure there is no way to know the fix is good enough,
         * and guessing is the thing this check exists to prevent. */
        good = (fix >= GPS_FIX_3D) && ok && hacc > 0 &&
               hacc <= HOME_MAX_HACC_MM;

        if (good) {
            geo_set_home(&g_home, lat, lon, (int32_t)(alt_m * 1000.0f));
            global_nav_state.home_valid = 1;
            global_nav_state.pos_n_m = 0.0f;
            global_nav_state.pos_e_m = 0.0f;
            global_nav_state.pos_d_m = 0.0f;
        }
        xSemaphoreGive(state_mutex);
    }

    if (!took) {
        ESP_LOGW(TAG, "home NOT captured: state_mutex busy");
    } else if (!good) {
        ESP_LOGW(TAG, "home NOT captured: fix=%u ok=%u hAcc=%lu mm",
                 fix, ok, (unsigned long)hacc);
    } else {
        /* Printed as scaled integers rather than %f - lat/lon need nine
         * significant digits and the ESP_LOG float path would round the
         * last two away, which is the precision this whole change was
         * about. */
        ESP_LOGI(TAG, "home captured: lat=%ld lon=%ld (1e-7 deg) "
                      "alt=%.1f m hAcc=%lu mm",
                 (long)lat, (long)lon, (double)alt_m, (unsigned long)hacc);
    }
    return good;
}

/* Copy into the 1 Hz report sample, replacing anything non-printable
 * with '.'. A baud mismatch delivers raw framing noise, which would
 * otherwise scramble the terminal. Shared by both protocol paths. */
static void sanitize_sample(char *dst, size_t dst_sz,
                            const char *src, int src_len) {
    size_t n = 0;
    for (int i = 0; i < src_len && n < dst_sz - 1; i++) {
        char c = src[i];
        dst[n++] = (c >= 32 && c < 127) ? c : '.';
    }
    dst[n] = 0;
}

#if TBSM10_USE_NMEA
// --- NMEA HELPERS ---

// Verify "$...*HH" checksum: XOR of all chars between '$' and '*'
// must equal the two hex digits after '*'. Rejects corrupted lines.
static bool nmea_checksum_ok(const char *line) {
    if (line[0] != '$') return false;

    uint8_t cs = 0;
    const char *p = line + 1;
    while (*p && *p != '*') {
        cs ^= (uint8_t)(*p++);
    }
    if (*p != '*' || !p[1] || !p[2]) return false; // no checksum present

    return cs == (uint8_t)strtol(p + 1, NULL, 16);
}

// Split line at commas IN PLACE, keeping EMPTY fields.
// (strtok collapses consecutive commas — "$GNGGA,time,,,,,0,00" would
// shift every later field into the wrong slot.)
// Returns number of fields.
static int nmea_split(char *line, char *fields[], int max_fields) {
    int n = 0;
    fields[n++] = line;
    for (char *p = line; *p && n < max_fields; p++) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    return n;
}

// --- NMEA PARSER ---
// Takes one raw sentence (e.g. "$GNGGA,...*4F") and updates nav state.
// Returns true if the sentence carried a valid checksum — i.e. this really
// was NMEA and not line noise. The caller uses that to tell a wiring fault
// (no bytes at all) apart from a baud mismatch (bytes, but none parse).
static bool parse_nmea_line(char *line) {
    if (!nmea_checksum_ok(line)) return false;

    char *f[24];
    int nf = nmea_split(line, f, 24);

    // 1. Position ($xxGGA)
    // f[2]=lat, f[3]=N/S, f[4]=lon, f[5]=E/W, f[6]=fix quality,
    // f[7]=sats, f[9]=altitude MSL (meters)
    if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
        if (nf < 10) return true;

        int quality = atoi(f[6]);
        int sats    = atoi(f[7]);
        float alt   = (f[9][0] != '\0') ? (float)atof(f[9]) : 0.0f;

        // Only accept position when there is a fix and lat/lon non-empty
        bool has_pos = (quality > 0) && (f[2][0] != '\0') && (f[4][0] != '\0');

        double final_lat = 0.0, final_lon = 0.0;
        if (has_pos) {
            // Convert NMEA DDMM.MMMM -> decimal degrees
            double raw_lat = atof(f[2]);
            double raw_lon = atof(f[4]);

            int lat_deg = (int)(raw_lat / 100);
            final_lat = lat_deg + (raw_lat - lat_deg * 100) / 60.0;
            if (f[3][0] == 'S') final_lat = -final_lat;

            int lon_deg = (int)(raw_lon / 100);
            final_lon = lon_deg + (raw_lon - lon_deg * 100) / 60.0;
            if (f[5][0] == 'W') final_lon = -final_lon;
        }

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (has_pos) {
                /* nav_state holds 1e-7 deg. The exact
                 * doubles computed above used to be truncated to float
                 * here, throwing away ~0.42 m of the longitude the module
                 * had already resolved. */
                global_nav_state.lat = (int32_t)(final_lat * 1.0e7);
                global_nav_state.lon = (int32_t)(final_lon * 1.0e7);
                global_nav_state.gps_alt_m = alt;
            }
            global_nav_state.sats = sats;
            xSemaphoreGive(state_mutex);
        }
    }
    // 2. Fix Type ($xxGSA): f[2] = mode (1=No Fix, 2=2D, 3=3D)
    else if (strncmp(line, "$GNGSA", 6) == 0 || strncmp(line, "$GPGSA", 6) == 0) {
        if (nf < 3) return true;

        int fix_mode = atoi(f[2]);
        if (fix_mode < GPS_FIX_NONE || fix_mode > GPS_FIX_3D) fix_mode = GPS_FIX_NONE;

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            global_nav_state.fix_type = fix_mode;
            xSemaphoreGive(state_mutex);
        }
    }
    return true;
}

// --- GPS LISTENER TASK ---
static void gps_task(void *pvParameters) {
    // Allocate buffer for raw bytes
    uint8_t *data = (uint8_t *) malloc(TBSM10_BUF_SIZE);
    if (!data) {
        ESP_LOGE(TAG, "RX buffer alloc failed - GPS task not starting");
        vTaskDelete(NULL);
        return;
    }

    // Line buffer to build NMEA strings
    char line_buf[128];
    int line_pos = 0;

    /* Bring-up counters. "No fix" has three very different causes and the
     * parser alone cannot tell them apart, because a bad checksum is
     * dropped silently:
     *   bytes == 0             -> nothing on the wire: TX/RX swapped, GPS
     *                             unpowered, or RX not landing on GPIO 44
     *   bytes > 0, ok == 0     -> framing noise: wrong baud rate
     *   ok > 0, still no fix   -> real NMEA, genuinely no satellites yet
     * Reported once a second, which is also the GPS update rate. */
    uint32_t n_bytes = 0, n_ok = 0, n_bad = 0;
    char sample[80] = "";
    bool sample_is_line = false;
    TickType_t next_report = xTaskGetTickCount() + pdMS_TO_TICKS(1000);

    ESP_LOGI(TAG, "GPS task listening on TX=%d RX=%d @ %d baud",
             TBSM10_TX_PIN, TBSM10_RX_PIN, TBSM10_BAUD_RATE);

    while (1) {
        // Read raw bytes from UART (Blocking for 100ms max)
        int len = uart_read_bytes(TBSM10_UART_NUM, data, TBSM10_BUF_SIZE, pdMS_TO_TICKS(100));

        if (len > 0) {
            n_bytes += (uint32_t)len;

            /* Hold a raw sample until a full line shows up. At the wrong
             * baud no line ever completes, so this is the only look we get
             * at what is actually on the wire. */
            if (!sample_is_line) {
                sanitize_sample(sample, sizeof(sample), (const char *)data, len);
            }

            // Process every byte received
            for (int i=0; i<len; i++) {
                char c = (char)data[i];

                // If end of line, parse the complete sentence
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line_buf[line_pos] = 0; // Null-terminate string
                        /* Snapshot first: parse_nmea_line splits in place. */
                        sanitize_sample(sample, sizeof(sample), line_buf, line_pos);
                        sample_is_line = true;
                        if (parse_nmea_line(line_buf)) n_ok++; else n_bad++;
                        line_pos = 0; // Reset buffer
                    }
                }
                // Accumulate characters (prevent buffer overflow)
                else if (line_pos < 127) {
                    line_buf[line_pos++] = c;
                }
            }
        }

        if (xTaskGetTickCount() >= next_report) {
            next_report += pdMS_TO_TICKS(1000);

            uint8_t sats = 0, fix = 0;
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                sats = global_nav_state.sats;
                fix  = global_nav_state.fix_type;
                xSemaphoreGive(state_mutex);
            }

            ESP_LOGI(TAG, "rx=%lu B/s nmea=%lu badsum=%lu sats=%u fix=%u | %s",
                     (unsigned long)n_bytes, (unsigned long)n_ok,
                     (unsigned long)n_bad, sats, fix, sample);

            n_bytes = n_ok = n_bad = 0;
            sample_is_line = false;
        }
    }
}

#endif /* TBSM10_USE_NMEA */

#if !TBSM10_USE_NMEA
// =====================  UBX PATH (Stage A)  =========================

/* Baud rates to try when the module is not talking to us, in the order
 * they are worth trying. TBSM10_BAUD_RATE first because a working system
 * must not pay a scan on every boot; 9600 second because it is the
 * factory rate a module reverts to after a cold start or a config wipe,
 * and the one that produced this bug.
 *
 * A mismatch does NOT look like silence, which is what made this hard to
 * see: bytes keep arriving at roughly (their_baud/our_baud) times the
 * real rate, framed wrongly. Measured at 840 B/s with zero
 * '$' and zero UBX sync - neither protocol, because every byte boundary
 * was in the wrong place. */
static const int k_baud_scan[] = { TBSM10_BAUD_RATE, 9600, 115200,
                                   57600, 19200 };

/* Does this look like a GPS talking, rather than a wrong-baud shred?
 * Either protocol will do - NMEA means the module is alive at its default
 * settings and simply has not been configured yet. */
static bool gps_traffic_looks_framed(const uint8_t *b, int n)
{
    for (int i = 0; i < n; i++) {
        if (b[i] == '$') return true;                       /* NMEA */
        if (i + 1 < n && b[i] == 0xB5 && b[i + 1] == 0x62)  /* UBX sync */
            return true;
    }
    return false;
}

/* Find the baud the module is ACTUALLY using and switch our UART to it.
 * Returns the rate found, or 0 if nothing framed was seen anywhere - in
 * which case the caller leaves the UART at the default and says so. */
static int gps_detect_baud(uint8_t *buf, size_t cap)
{
    for (unsigned i = 0; i < sizeof(k_baud_scan) / sizeof(k_baud_scan[0]);
         i++) {
        const int baud = k_baud_scan[i];
        if (uart_set_baudrate(TBSM10_UART_NUM, baud) != ESP_OK) continue;
        uart_flush_input(TBSM10_UART_NUM);
        /* 400 ms: a 1 Hz NMEA module emits its whole sentence set inside
         * this, and a 10 Hz UBX one several solutions. */
        int len = uart_read_bytes(TBSM10_UART_NUM, buf, cap,
                                  pdMS_TO_TICKS(400));
        if (len > 0 && gps_traffic_looks_framed(buf, len)) {
            ESP_LOGI(TAG, "module found at %d baud (%d B in 400 ms)",
                     baud, len);
            return baud;
        }
        ESP_LOGD(TAG, "  %d baud: %d B, not framed", baud, len);
    }
    uart_set_baudrate(TBSM10_UART_NUM, TBSM10_BAUD_RATE);
    return 0;
}

/* Baud the module is believed to be using right now. 0 = not yet
 * detected. Set by gps_detect_baud(), followed by ubx_send_config(). */
static int s_gps_baud = 0;

/* --- configuration: one CFG-VALSET per group, every answer read back ---
 *
 * Until all of this went out as ONE VALSET, and a VALSET is
 * all-or-nothing: a single key the module does not know NAKs every key in
 * the message. When the interference guard's six keys joined it (f8091ca,
 *) the GPS fell back to factory NMEA with zero NAV-PVT, and one of
 * those keys being unsupported by this module's firmware is the prime
 * suspect. So the core group now goes out alone - the same
 * keys and values as when NAV-PVT last worked  - each guard input is
 * a message of its own, and the module's ACK or NAK for every one is kept
 * for GSEC. Whatever this firmware rejects now costs only that input. */
typedef enum { CFG_PENDING = 0, CFG_ACK, CFG_NAK, CFG_SILENT } cfg_result_t;
enum { CFG_CORE, CFG_NAV_SIG, CFG_MON_RF, CFG_SEC_SIG, CFG_ITFM, CFG_N };
static const char *const k_cfg_group[CFG_N] = {
    "core", "NAV-SIG", "MON-RF", "SEC-SIG", "ITFM",
};
static const char *const k_cfg_result[] = { "-", "ACK", "NAK", "no-reply" };
static volatile uint8_t  s_cfg[CFG_N];     /* cfg_result_t per group, for GSEC */
static volatile uint32_t s_cfg_rounds;

/* How long to wait for the module's answer to one VALSET. u-blox answers
 * within tens of ms; the rest is margin on a busy 38400-baud link. */
#define CFG_REPLY_US 500000

/* One group's VALSET payload. Returns its length, or 0 if it overflowed
 * (ubx_valset_add propagates a 0). */
static uint16_t cfg_payload(int group, uint8_t *pl, uint16_t cap)
{
    uint16_t n = ubx_valset_begin(pl, cap, UBX_LAYER_RAM);
    switch (group) {
    case CFG_CORE: {
        /* FIRST, because everything after it depends on the link
         * surviving: move the module to the rate this driver wants to run
         * at. Harmless when it is already there. The module applies it
         * after acking this frame, so the caller must follow it there. */
        n = ubx_valset_add(pl, cap, n, UBX_KEY_UART1_BAUDRATE,
                           TBSM10_BAUD_RATE, 4);
        /* 100 ms between solutions, one output per solution -> 10 Hz. */
        n = ubx_valset_add(pl, cap, n, UBX_KEY_RATE_MEAS, 100, 2);
        n = ubx_valset_add(pl, cap, n, UBX_KEY_RATE_NAV, 1, 2);
        /* Airborne <4g: the receiver stops assuming a ground vehicle,
         * which is what keeps a fix through the accelerations of flight. */
        n = ubx_valset_add(pl, cap, n, UBX_KEY_NAVSPG_DYNMODEL,
                           UBX_DYNMODEL_AIRBORNE_4G, 1);
        /* NAV-PVT on, every NMEA sentence off. Silencing NMEA is not
         * tidiness: at 38400 baud the default sentence set plus 10 Hz
         * NAV-PVT does not fit in the link, and the overflow surfaces as
         * dropped frames. */
        n = ubx_valset_add(pl, cap, n, UBX_KEY_MSGOUT_NAV_PVT_U1, 1, 1);
        static const uint32_t nmea_off[] = {
            UBX_KEY_MSGOUT_NMEA_GGA_U1, UBX_KEY_MSGOUT_NMEA_GLL_U1,
            UBX_KEY_MSGOUT_NMEA_GSA_U1, UBX_KEY_MSGOUT_NMEA_GSV_U1,
            UBX_KEY_MSGOUT_NMEA_RMC_U1, UBX_KEY_MSGOUT_NMEA_VTG_U1,
        };
        for (unsigned i = 0; i < sizeof(nmea_off) / sizeof(nmea_off[0]); i++)
            n = ubx_valset_add(pl, cap, n, nmea_off[i], 0, 1);
        break;
    }

    /* --- the interference guard's inputs, at ONE TENTH the solution rate
     *
     * The MSGOUT value is "emit every Nth navigation solution", so 10 at a
     * 10 Hz solution rate is 1 Hz. That divisor is a hard link-budget
     * requirement, not a preference. At 38400 baud, 8N1, the link carries
     * 3840 B/s:
     *
     *   NAV-PVT   100 B x 10 Hz = 1000 B/s   (already there)
     *   NAV-SIG  ~658 B x  1 Hz =  658 B/s   8 + 16*numSigs, ~40 signals
     *   MON-RF     36 B x  1 Hz =   36 B/s
     *   SEC-SIG    20 B x  1 Hz =   20 B/s
     *                            ---------
     *                             1714 B/s   45% of the link
     *
     * NAV-SIG at the full 10 Hz would be 6580 B/s ON ITS OWN - nearly
     * twice the entire link - and the overflow would surface as dropped
     * NAV-PVT frames, i.e. as a navigation fault a long way from here.
     * 1 Hz is also plenty: jamming and spoofing are not fast dynamics, and
     * the guard's confirmation window is a second. */
    case CFG_NAV_SIG:
        n = ubx_valset_add(pl, cap, n, UBX_KEY_MSGOUT_NAV_SIG_U1, 10, 1);
        break;
    case CFG_MON_RF:
        n = ubx_valset_add(pl, cap, n, UBX_KEY_MSGOUT_MON_RF_U1, 10, 1);
        break;
    case CFG_SEC_SIG:
        n = ubx_valset_add(pl, cap, n, UBX_KEY_MSGOUT_SEC_SIG_U1, 10, 1);
        break;
    case CFG_ITFM:
        /* The interference monitor itself. Without it the MON-RF
         * jammingState and jamInd fields sit at 0 = "unknown" forever and
         * the jamming half of the guard silently reports nothing at all.
         * The thresholds are the u-blox defaults, restated so a module that
         * booted with something else is put back to a known state. */
        n = ubx_valset_add(pl, cap, n, UBX_KEY_ITFM_ENABLE, 1, 1);
        n = ubx_valset_add(pl, cap, n, UBX_KEY_ITFM_BBTHRESHOLD, 3, 1);
        n = ubx_valset_add(pl, cap, n, UBX_KEY_ITFM_CWTHRESHOLD, 15, 1);
        break;
    default:
        return 0;
    }
    return n;
}

/* Send one VALSET and wait for the module's ACK or NAK. Reads through the
 * task's own parser and buffer, so bytes read here are consumed and a few
 * NMEA or NAV-PVT frames around a config round are lost - config only runs
 * at boot and, while no NAV-PVT arrives, every 5 s. */
static cfg_result_t cfg_exchange(ubx_parser_t *p, uint8_t *buf, size_t cap,
                                 const uint8_t *frame, uint16_t len)
{
    uart_write_bytes(TBSM10_UART_NUM, (const char *)frame, len);
    uart_wait_tx_done(TBSM10_UART_NUM, pdMS_TO_TICKS(200));
    const int64_t until = esp_timer_get_time() + CFG_REPLY_US;
    while (esp_timer_get_time() < until) {
        const int got = uart_read_bytes(TBSM10_UART_NUM, buf, cap,
                                        pdMS_TO_TICKS(20));
        for (int i = 0; i < got; i++) {
            uint8_t acked, cls, id;
            if (ubx_parse_byte(p, buf[i]) &&
                ubx_decode_ack(p, &acked, &cls, &id) &&
                cls == UBX_CLASS_CFG && id == UBX_ID_CFG_VALSET) {
                return acked ? CFG_ACK : CFG_NAK;
            }
        }
    }
    return CFG_SILENT;
}

/* Send the RAM-layer configuration, group by group. RAM only, deliberately:
 * a power cycle reverts the module to factory NMEA, so a wrong key or a bad
 * value can never leave the GPS in a state the firmware cannot talk to. The
 * price is re-sending on every boot, which is what this function is for. */
static void ubx_send_config(ubx_parser_t *p, uint8_t *buf, size_t cap)
{
    uint8_t pl[64];      /* the core group is the largest, 64 B */
    uint8_t frame[80];

    s_cfg_rounds++;
    for (int g = 0; g < CFG_N; g++) {
        const uint16_t n = cfg_payload(g, pl, sizeof(pl));
        const uint16_t len = n ? ubx_build(frame, sizeof(frame), UBX_CLASS_CFG,
                                           UBX_ID_CFG_VALSET, pl, n) : 0;
        if (len == 0) {
            ESP_LOGE(TAG, "CFG-VALSET %s: build failed - NOT sent",
                     k_cfg_group[g]);
            s_cfg[g] = CFG_SILENT;
            continue;
        }
        /* Speak at whatever rate the module is listening at, not the rate
         * we wish it were using - a config sent at the wrong baud is the bug
         * this whole path exists to fix, and it fails silently in both
         * directions at once. Only the core group moves the rate. */
        const bool rate_move = (g == CFG_CORE) && s_gps_baud &&
                               s_gps_baud != TBSM10_BAUD_RATE;
        if (rate_move) uart_set_baudrate(TBSM10_UART_NUM, s_gps_baud);
        const cfg_result_t r = cfg_exchange(p, buf, cap, frame, len);
        s_cfg[g] = (uint8_t)r;
        /* The module moves to the new rate after acking the core group. A
         * NAK moved nothing, so stay put; no reply at all is ambiguous, so
         * follow anyway and let the 5 s re-detect correct a wrong guess. */
        if (rate_move && r != CFG_NAK) {
            uart_set_baudrate(TBSM10_UART_NUM, TBSM10_BAUD_RATE);
            uart_flush_input(TBSM10_UART_NUM);   /* drop the mid-switch shred */
            ESP_LOGI(TAG, "core config sent at %d baud (%s); both ends now "
                          "at %d", s_gps_baud, k_cfg_result[r],
                     TBSM10_BAUD_RATE);
            s_gps_baud = TBSM10_BAUD_RATE;
        }
    }
    ESP_LOGI(TAG, "config: core %s | NAV-SIG %s | MON-RF %s | SEC-SIG %s | "
                  "ITFM %s", k_cfg_result[s_cfg[CFG_CORE]],
             k_cfg_result[s_cfg[CFG_NAV_SIG]], k_cfg_result[s_cfg[CFG_MON_RF]],
             k_cfg_result[s_cfg[CFG_SEC_SIG]], k_cfg_result[s_cfg[CFG_ITFM]]);
    if (s_cfg[CFG_CORE] != CFG_ACK) {
        ESP_LOGW(TAG, "core config %s - NAV-PVT will not start",
                 k_cfg_result[s_cfg[CFG_CORE]]);
    }
}

void tbsm10_cfg_report(char *out, size_t n)
{
    snprintf(out, n, "config round %lu: core %s, NAV-SIG %s, MON-RF %s, "
                     "SEC-SIG %s, ITFM %s", (unsigned long)s_cfg_rounds,
             k_cfg_result[s_cfg[CFG_CORE]], k_cfg_result[s_cfg[CFG_NAV_SIG]],
             k_cfg_result[s_cfg[CFG_MON_RF]], k_cfg_result[s_cfg[CFG_SEC_SIG]],
             k_cfg_result[s_cfg[CFG_ITFM]]);
}

/* Copy one decoded solution into nav_state, and update NED if home is set.
 *
 * Position is written ONLY when the module says the fix is usable. A
 * NAV-PVT arrives at 10 Hz whether or not there is a fix, and its lat/lon
 * are zero or stale when there is not; writing those unconditionally would
 * park the aircraft off the coast of Africa at 0,0. Velocity, accuracy and
 * the counters are always written, because "no fix, 3 sats, hAcc huge" is
 * exactly what the health log needs to say. */
static void ubx_publish(const ubx_nav_pvt_t *p)
{
    const bool usable = (p->fix_type >= GPS_FIX_3D) && p->gnss_fix_ok;

    float n_m = 0.0f, e_m = 0.0f, d_m = 0.0f;
    const bool have_ned = usable && g_home.valid &&
        geo_to_ned(&g_home, p->lat_1e7, p->lon_1e7,
                   p->h_msl_mm, &n_m, &e_m, &d_m);

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    if (usable) {
        global_nav_state.lat = p->lat_1e7;
        global_nav_state.lon = p->lon_1e7;
        global_nav_state.gps_alt_m = (float)p->h_msl_mm * 0.001f;
    }
    global_nav_state.sats = p->num_sv;
    global_nav_state.fix_type = p->fix_type;
    global_nav_state.gps_fix_ok = p->gnss_fix_ok;
    global_nav_state.vel_n_mms = p->vel_n_mms;
    global_nav_state.vel_e_mms = p->vel_e_mms;
    global_nav_state.vel_d_mms = p->vel_d_mms;
    global_nav_state.gps_h_acc_mm = p->h_acc_mm;
    global_nav_state.gps_v_acc_mm = p->v_acc_mm;
    global_nav_state.gps_s_acc_mms = p->s_acc_mms;
    global_nav_state.gps_itow_ms = p->itow_ms;
    global_nav_state.gps_time_us = esp_timer_get_time();

    if (have_ned) {
        global_nav_state.home_valid = 1;
        global_nav_state.pos_n_m = n_m;
        global_nav_state.pos_e_m = e_m;
        global_nav_state.pos_d_m = d_m;
    }
    xSemaphoreGive(state_mutex);
}

/* Hand MON-RF (and the last SEC-SIG, if one has arrived) to the guard. */
static void guard_feed_rf(const ubx_mon_rf_t *rf, const ubx_sec_sig_t *sec,
                          uint8_t have_sec, uint32_t now_ms)
{
    gg_feed_rf(&s_guard, now_ms,
               rf->jamming_state, rf->agc_cnt, rf->noise_per_ms, rf->jam_ind,
               have_sec,
               have_sec ? sec->jamming_state : 0,
               have_sec ? sec->spoofing_state : 0);
}

/* Hand the solution to the guard, with the horizontal filter's own speed
 * as the cross-check.
 *
 * The inertial speed is read from horiz_vn_ms/horiz_ve_ms and used ONLY
 * when horiz_valid says the filter means it. Comparing against an estimate
 * the filter does not trust would invent a disagreement and condemn a
 * perfectly good fix - so a filter with nothing to say makes this check
 * sit out rather than vote. */
static void guard_feed_solution(const ubx_nav_pvt_t *p, uint32_t now_ms)
{
    float gn = (float)p->vel_n_mms, ge = (float)p->vel_e_mms;
    uint32_t gps_speed = (uint32_t)gg_sqrtf(gn * gn + ge * ge);

    uint8_t have_inertial = 0;
    uint32_t inertial_speed = 0;
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (global_nav_state.horiz_valid) {
            float vn = global_nav_state.horiz_vn_ms;
            float ve = global_nav_state.horiz_ve_ms;
            inertial_speed = (uint32_t)(gg_sqrtf(vn * vn + ve * ve) * 1000.0f);
            have_inertial = 1;
        }
        xSemaphoreGive(state_mutex);
    }

    /* Only a fix the module itself vouches for is worth cross-checking;
     * a settling or absent solution disagrees with everything. */
    uint8_t usable = (p->fix_type >= 3 && p->gnss_fix_ok);
    gg_feed_solution(&s_guard, now_ms, usable ? p->num_sv : 0,
                     gps_speed, usable ? have_inertial : 0, inertial_speed);
}

/* Snapshot of the guard for the console. Returns 0 if the mutex was busy,
 * so a caller prints "busy" rather than a half-copied verdict. */
int tbsm10_guard_status(gg_status_t *out)
{
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return 0;
    *out = s_guard_pub;
    xSemaphoreGive(state_mutex);
    return 1;
}

void tbsm10_guard_clear(void)
{
    gg_force_clear(&s_guard, (uint32_t)(esp_timer_get_time() / 1000));
}

static void gps_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(TBSM10_BUF_SIZE);
    if (!data) {
        ESP_LOGE(TAG, "RX buffer alloc failed - GPS task not starting");
        vTaskDelete(NULL);
        return;
    }

    geo_home_clear(&g_home);

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_nav_pvt_t pvt;

    /* Guard inputs. MON-RF and SEC-SIG arrive as separate frames, so the
     * last of each is held here and the pair is fed together. */
    ubx_mon_rf_t rf;
    ubx_sec_sig_t sec;
    ubx_cno_stats_t cno;
    uint8_t have_rf = 0, have_sec = 0;
    uint32_t n_rf = 0, n_sec = 0, n_sig = 0;
    gg_init(&s_guard, 0);

    /* The same three-way bring-up split the NMEA driver reports, plus a
     * fourth case that only exists for UBX:
     *   bytes == 0             -> nothing on the wire
     *   bytes > 0, pvt == 0,
     *     and NMEA in them     -> the module never got the config. TX has
     *                             never been exercised, so check it first
     *   pvt > 0, still no fix  -> real solutions, genuinely no satellites */
    uint32_t n_bytes = 0, n_pvt = 0, n_nmea = 0;
    uint32_t last_frames = 0, last_badsum = 0;
    char sample[80] = "";
    TickType_t next_report = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    int reconfig_ticks = 0;

    ESP_LOGI(TAG, "GPS task listening on TX=%d RX=%d @ %d baud (UBX)",
             TBSM10_TX_PIN, TBSM10_RX_PIN, TBSM10_BAUD_RATE);

    /* Find the module before talking to it. Measured 840 B/s
     * arriving with ubx=0, pvt=0, badsum=0 AND n_nmea=0 - bytes that were
     * neither protocol, because the module was not at the rate we were
     * reading. Config sent into that goes unheard, so the receiver stays
     * unconfigured forever and the retry every 5 s can never win. */
    s_gps_baud = gps_detect_baud(data, TBSM10_BUF_SIZE);
    if (s_gps_baud == 0) {
        ESP_LOGW(TAG, "no framed GPS traffic at any of %d rates - staying "
                      "at %d. Check GPIO %d (RX) <- GPS TX and that the "
                      "module has power.",
                 (int)(sizeof(k_baud_scan) / sizeof(k_baud_scan[0])),
                 TBSM10_BAUD_RATE, TBSM10_RX_PIN);
        s_gps_baud = TBSM10_BAUD_RATE;
    }
    ubx_send_config(&parser, data, TBSM10_BUF_SIZE);

    while (1) {
        int len = uart_read_bytes(TBSM10_UART_NUM, data, TBSM10_BUF_SIZE,
                                  pdMS_TO_TICKS(100));
        if (len > 0) {
            n_bytes += (uint32_t)len;
            if (sample[0] == 0) {
                sanitize_sample(sample, sizeof(sample), (const char *)data, len);
            }
            for (int i = 0; i < len; i++) {
                if (data[i] == 0x24) n_nmea++;      /* '$' - an NMEA sentence */
                if (ubx_parse_byte(&parser, data[i])) {
                    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

                    if (ubx_decode_nav_pvt(&parser, &pvt)) {
                        ubx_publish(&pvt);
                        n_pvt++;
                        guard_feed_solution(&pvt, now_ms);
                    } else if (ubx_decode_mon_rf(&parser, &rf)) {
                        have_rf = 1;
                        n_rf++;
                        guard_feed_rf(&rf, &sec, have_sec, now_ms);
                    } else if (ubx_decode_sec_sig(&parser, &sec)) {
                        have_sec = 1;
                        n_sec++;
                        /* SEC-SIG carries no AGC or noise count, so it is
                         * only worth feeding once a MON-RF has supplied
                         * those. Before that the guard would be learning a
                         * baseline of zero. */
                        if (have_rf) guard_feed_rf(&rf, &sec, 1, now_ms);
                    }

                    /* Independent of the frame that just completed: the
                     * accumulator publishes on its own schedule and a
                     * NAV-SIG is never a "decoded" frame in the sense
                     * above, because nothing of it was buffered. */
                    if (ubx_take_cno_stats(&parser, &cno)) {
                        n_sig++;
                        gg_feed_cno(&s_guard, now_ms, cno.n_used,
                                    cno.sum_cno, cno.sumsq_cno);
                    }
                }
            }
        }

        /* Timers, every loop rather than once a second: gg_update() is
         * what applies the staleness, confirmation and latch windows, and
         * a guard updated at 1 Hz can only resolve them to the nearest
         * second. This costs a few microseconds per 100 ms. */
        {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            gg_update(&s_guard, now_ms);
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                s_guard_pub = *gg_status(&s_guard);
                xSemaphoreGive(state_mutex);
            }
        }

        if (xTaskGetTickCount() >= next_report) {
            next_report += pdMS_TO_TICKS(1000);

            uint8_t sats = 0, fix = 0, ok = 0, hv = 0;
            int32_t vn = 0, ve = 0, vd = 0;
            uint32_t hacc = 0;
            float pn = 0.0f, pe = 0.0f;
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                sats = global_nav_state.sats;
                fix  = global_nav_state.fix_type;
                ok   = global_nav_state.gps_fix_ok;
                hv   = global_nav_state.home_valid;
                vn   = global_nav_state.vel_n_mms;
                ve   = global_nav_state.vel_e_mms;
                vd   = global_nav_state.vel_d_mms;
                hacc = global_nav_state.gps_h_acc_mm;
                pn   = global_nav_state.pos_n_m;
                pe   = global_nav_state.pos_e_m;
                xSemaphoreGive(state_mutex);
            }

            const uint32_t frames = parser.n_frames - last_frames;
            const uint32_t badsum = parser.n_badsum - last_badsum;
            last_frames = parser.n_frames;
            last_badsum = parser.n_badsum;

            ESP_LOGI(TAG,
                     "rx=%lu B/s ubx=%lu pvt=%lu badsum=%lu | fix=%u ok=%u "
                     "sats=%u hAcc=%lu mm vel=%ld/%ld/%ld mm/s | home=%u "
                     "N=%.1f E=%.1f",
                     (unsigned long)n_bytes, (unsigned long)frames,
                     (unsigned long)n_pvt, (unsigned long)badsum,
                     fix, ok, sats, (unsigned long)hacc,
                     (long)vn, (long)ve, (long)vd, hv, (double)pn, (double)pe);

            /* Guard line, once a second. Printed even when everything is
             * clean, because "the guard is running and sees nothing" and
             * "the guard is not running" look identical otherwise - and
             * the second one is what a silent security feature always
             * turns out to be. sig/rf/sec are the frame counts: all three
             * at zero means the CFG-VALSET for them never took. */
            {
                const gg_status_t *gs = gg_status(&s_guard);
                static const char *lvl[] = { "CLEAN", "SUSPECT", "DENIED" };
                ESP_LOGI(TAG,
                         "guard pos=%s vel=%s reasons=0x%03X | sig=%lu rf=%lu "
                         "sec=%lu | jam=%u spf=%u agc=%u/%u noise=%u/%u "
                         "cno=%.1f sd=%.1f n=%u",
                         lvl[gs->pos <= GG_DENIED ? gs->pos : 0],
                         lvl[gs->vel <= GG_DENIED ? gs->vel : 0],
                         (unsigned)gs->reasons,
                         (unsigned long)n_sig, (unsigned long)n_rf,
                         (unsigned long)n_sec,
                         gs->jamming_state, gs->spoofing_state,
                         gs->agc_cnt, gs->agc_baseline,
                         gs->noise_per_ms, gs->noise_baseline,
                         (double)gs->cno_mean, (double)gs->cno_sd,
                         gs->n_used);
            }

            /* Bytes are arriving, they contain NMEA, and no NAV-PVT ever
             * does. The module never received the configuration, so say the
             * likely reason out loud rather than leaving a silent GPS to be
             * rediscovered later. Re-sent every 5 s in case the module
             * finished booting after we did. */
            if (n_bytes > 0 && n_pvt == 0) {
                if (n_nmea > 0) {
                    /* The module's answer to the core VALSET says which half
                     * is broken: a NAK is the module refusing a key or a
                     * value, no reply at all is the wire or the baud. */
                    const uint8_t c = s_cfg[CFG_CORE];
                    if (c == CFG_NAK) {
                        ESP_LOGW(TAG, "NMEA on the wire, zero NAV-PVT: the "
                                      "module NAKed the core config - a key "
                                      "or value it rejects. rx: %s", sample);
                    } else {
                        ESP_LOGW(TAG, "NMEA on the wire, zero NAV-PVT (core "
                                      "config %s). Check GPIO %d (TX) -> GPS "
                                      "RX, then baud. rx: %s",
                                 k_cfg_result[c], TBSM10_TX_PIN, sample);
                    }
                } else {
                    /* Neither protocol. Not a TX fault and not a dead
                     * module - the bytes are arriving, they just are not
                     * bytes. Print them: this line is what turns "pvt=0"
                     * from a mystery into a baud mismatch at a glance. */
                    ESP_LOGW(TAG, "%lu B/s that is NEITHER NMEA NOR UBX at %d "
                                  "baud - framing is wrong, not the wiring. "
                                  "rx: %s",
                             (unsigned long)n_bytes, s_gps_baud, sample);
                }
                if (++reconfig_ticks >= 5) {
                    reconfig_ticks = 0;
                    /* RE-DETECT before retrying. Resending at a rate we
                     * already know produced nothing is the loop that ran
                     * unnoticed for weeks. Only reachable while pvt == 0,
                     * so a working link never pays for the scan. */
                    int b = gps_detect_baud(data, TBSM10_BUF_SIZE);
                    s_gps_baud = b ? b : TBSM10_BAUD_RATE;
                    ubx_send_config(&parser, data, TBSM10_BUF_SIZE);
                }
            } else {
                reconfig_ticks = 0;
            }

            n_bytes = n_pvt = n_nmea = 0;
            sample[0] = 0;
        }
    }
}
#endif /* !TBSM10_USE_NMEA */

// --- INIT FUNCTION ---
void tbsm10_init_and_start_task(void) {
    uart_config_t uart_config = {
        .baud_rate = TBSM10_BAUD_RATE, // 38400
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 1. Install Driver
    ESP_ERROR_CHECK(uart_driver_install(TBSM10_UART_NUM, TBSM10_BUF_SIZE * 2, 0, 0, NULL, 0));
    
    // 2. Configure Parameters
    ESP_ERROR_CHECK(uart_param_config(TBSM10_UART_NUM, &uart_config));
    
    // 3. Set Pins. TX is only used to CONFIGURE the module (UBX path);
    //    see the wiring note in tbsM10.h before trusting it.
    ESP_ERROR_CHECK(uart_set_pin(TBSM10_UART_NUM, TBSM10_TX_PIN, TBSM10_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 4. Start Task
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 2, NULL,
                            FC_CORE_SUPPORT);
}