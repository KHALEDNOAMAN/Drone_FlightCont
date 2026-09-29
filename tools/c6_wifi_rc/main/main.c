/* ============================================================
 * ESP32-C6 telemetry bridge + live dashboard for the S3 FC.
 *
 * The WiFi-RC joystick emulator that used to live here was REMOVED
 * the real FS-iA6B receiver is on the FC (GPIO 8 since
 *), so this board's only job is telemetry.
 *
 * Three services:
 *   1. Raw TCP bridge   FC UART -> TCP :5555. The FFT capture path,
 *      contract unchanged: `ncat <ip> 5555 > sweep.csv`.
 *   2. Dashboard        http://<ip>/  - live telemetry page, ~4 Hz,
 *      rendered from the FC's CSV line (GET /telem).
 *   3. PID uplink       POST /pid "kp,ki,kd,kpy,kiy,kdy" -> validated ->
 *      "PID,..." line on the UART -> the FC's params (params_set). The FC
 *      echoes the ACTIVE gains back inside the CSV (fields 46-51), so a
 *      successful set is visible in the stream - the loop closes end to
 *      end.
 *   4. Console          POST /cmd forwards one console command, GET
 *      /console returns the FC's '#' replies.
 *
 * Wiring :
 *   C6 D5 / GPIO23 (UART0 RX) <- S3 GPIO47 (telemetry TX)   921600 baud
 *   C6 D6 / GPIO16 (UART0 TX) -> S3 GPIO21 (telemetry RX)   commands, PID
 *   GND common.
 *
 * CSV field map (0-based; authoritative copy in telemetry.c):
 *   0 armed 1-3 r,p,y 4-6 des 13 temp 15 press 16 alt 17-19 lat,lon,sats
 *   21-24 m1..m4 38 dt 39-41 gyro 42-44 accel 45 thr
 *   46-51 active PID (rate kp,ki,kd, yaw kp,ki,kd)
 *   52 sensor_health 53 fix_type 54 arm_blocked
 *   55-58 raw RC r,p,y,arm 59 rc_frozen_frames 60 rc_link_ok
 *   61 vbat_mv (INA226 pack voltage; 0 = no reading) 62 tof_mm
 *   63-67 alt_est, vz, accz_bias, alt_sp, vert_flags
 *   68-69 MS5611 pressure, altitude  70-71 alt_ref_m, alt_ref_valid
 *
 * The bridge itself does not parse any of this - it forwards the newest
 * CSV line verbatim and the page splits it. Appending a field therefore
 * needs no bridge change, only a dashboard one, and the dashboard is
 * embedded in THIS binary: a new field means reflashing the C6 too.
 * ============================================================ */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <errno.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/tcp.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "mdns.h"
#include "wifi_creds.h"

static const char *TAG = "C6_BRIDGE";

/* ---- Telemetry link: FC <-> C6 UART ---- */
#define TELEM_UART        UART_NUM_0
/* GPIO23 = D5, NOT D7/GPIO17. THE GPIO17 PAD'S INPUT IS DEAD on this
 * board, proven with the S3 electrically off the wires, the
 * C6 could not receive its OWN transmitter looped onto GPIO17 through
 * the GPIO matrix (rx=0 B/s), and the identical test on GPIO23 read
 * rx=5 B/s immediately. That dead input is why the dashboard showed no
 * data through five S3 pin assignments and two rewires. */
#define TELEM_RX_GPIO     23         /* D5 <- S3 GPIO47 (FC TX) */
#define TELEM_TX_GPIO     16         /* D6 -> S3 GPIO21 (FC RX): commands, PID */
#define TELEM_BAUD        921600
#define TELEM_TCP_PORT    5555
/* 32 KB = ~2.2 s of the 14.8 KB/s FFT gyro stream. 8192 gave only
 * 0.55 s and a single 837 ms WiFi stall overflowed it, losing 12% of a
 * capture. The loss is here, not in TCP - TCP retransmits, so anything
 * that reaches the socket arrives. */
#define TELEM_RX_BUF      32768
#define CSV_LINE_MAX      768

/* 1 = join the house WiFi (bench: keeps the PC's internet alive).
 * 0 = be an AP (field use, no router). */
#define USE_WIFI_STA      1

/* AP_SSID and AP_PASS come from wifi_creds.h, which is gitignored - the AP
 * password is a credential like the STA ones and does not belong in a file
 * that gets pushed. Copy wifi_creds.h.example if this fails to compile. */
#if !defined(AP_SSID) || !defined(AP_PASS)
#error "AP_SSID / AP_PASS missing from wifi_creds.h - see wifi_creds.h.example"
#endif

/* mDNS name: the dashboard is http://drone.local/ on every network.
 * Worth the managed component because the bench router and the phone
 * hotspot hand out different subnets (192.168.1.x vs 192.168.65.x), so
 * the numeric address changes depending on where you are. */
#define MDNS_HOSTNAME     "drone"

/* ---- Shared state ---- */
static volatile int      s_client_fd = -1;    /* raw TCP bridge client */
static SemaphoreHandle_t s_csv_mtx;
static char              s_last_csv[CSV_LINE_MAX];  /* latest FC CSV line */

/* Console ring: '#'-prefixed lines the FC sends as command replies.
 * This is what replaces "type STATS into a console that is printing 50
 * CSV lines a second" - the reply lands here and the dashboard shows
 * it. Sized for a full STATS dump (task list is ~15 lines) plus room. */
#define CONSOLE_LINES  48
#define CONSOLE_COLS   180
static char     s_console[CONSOLE_LINES][CONSOLE_COLS];
static uint32_t s_console_head = 0;   /* next slot to write */
static uint32_t s_console_count = 0;  /* lines ever written */

static void console_push(const char *line)
{
    if (xSemaphoreTake(s_csv_mtx, pdMS_TO_TICKS(20)) != pdTRUE) return;
    strlcpy(s_console[s_console_head], line, CONSOLE_COLS);
    s_console_head = (s_console_head + 1) % CONSOLE_LINES;
    s_console_count++;
    xSemaphoreGive(s_csv_mtx);
}

/* ---- UART reader: the single owner of the RX stream ----
 *
 * Drains the UART continuously (the old code only read while a TCP
 * client was attached), forwards raw bytes to the TCP client when one
 * exists, and keeps the newest telemetry CSV line for the dashboard.
 *
 * The TCP send is deliberately BLOCKING: during a WiFi stall the reader
 * parks and the 32 KB UART buffer absorbs the stream, which is exactly
 * the behaviour that got capture loss to 0.05%. The dashboard just
 * serves a slightly stale line during a stall - correct trade for the
 * FFT capture path.
 *
 * Line classification: FC telemetry CSV starts with a digit (the armed
 * flag); command replies start with '#'. Everything else - notably the
 * "G," flood of an FFT capture - goes to the TCP bridge only and is
 * never buffered here. */
static void uart_reader_task(void *arg)
{
    (void)arg;
    static uint8_t buf[1024];
    static char    line[CSV_LINE_MAX];
    int pos = 0;
    uint32_t bytes = 0, csv_lines = 0;
    TickType_t last_log = xTaskGetTickCount();

    for (;;) {
        int n = uart_read_bytes(TELEM_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0) {
            bytes += (uint32_t)n;

            int fd = s_client_fd;
            if (fd >= 0 && send(fd, buf, n, 0) < 0) {
                /* Only tear down the fd we actually failed on - the
                 * accept task may have swapped in a new client. */
                if (s_client_fd == fd) {
                    s_client_fd = -1;
                    close(fd);
                    ESP_LOGI(TAG, "TCP client gone");
                }
            }

            for (int i = 0; i < n; i++) {
                char c = (char)buf[i];
                if (c == '\n' || c == '\r') {
                    if (pos > 0) {
                        line[pos] = 0;
                        if (isdigit((unsigned char)line[0])) {
                            if (xSemaphoreTake(s_csv_mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
                                memcpy(s_last_csv, line, pos + 1);
                                xSemaphoreGive(s_csv_mtx);
                            }
                            csv_lines++;
                        } else if (line[0] == '#') {
                            console_push(line + 1);
                        }
                        pos = 0;
                    }
                } else if (pos < CSV_LINE_MAX - 1) {
                    line[pos++] = c;
                } else {
                    pos = 0;   /* overlong: drop */
                }
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_log >= pdMS_TO_TICKS(1000)) {
            last_log = now;
            ESP_LOGI(TAG, "rx=%lu B/s csv=%lu/s tcp=%s",
                     (unsigned long)bytes, (unsigned long)csv_lines,
                     s_client_fd >= 0 ? "client" : "idle");
            bytes = csv_lines = 0;
        }
    }
}

/* ---- Raw TCP bridge: accept loop (one client at a time) ---- */
static void tcp_accept_task(void *arg)
{
    (void)arg;

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "telem socket() failed");
        vTaskDelete(NULL);
        return;
    }
    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(TELEM_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "telem bind/listen failed");
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "raw bridge listening on %d", TELEM_TCP_PORT);

    for (;;) {
        struct sockaddr_in peer;
        socklen_t peerlen = sizeof(peer);
        int fd = accept(listen_fd, (struct sockaddr *)&peer, &peerlen);
        if (fd < 0) {
            /* This used to retry silently, which is exactly why a socket
             * exhaustion bug looked like "the bridge hangs up on me": the
             * client saw a completed handshake (lwIP does that in the
             * backlog) and then a reset, while the only component that
             * knew why said nothing. ENFILE/EMFILE here means the pool is
             * full - see the note in httpd_start_all. */
            ESP_LOGE(TAG, "accept() failed, errno %d (%s) - retrying",
                     errno, strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* No Nagle: 40 ms of coalescing on a stream we timestamp to the
         * microsecond is pure downside. */
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        int old = s_client_fd;
        s_client_fd = -1;          /* park the reader off the old fd */
        if (old >= 0) close(old);
        s_client_fd = fd;
        ESP_LOGI(TAG, "TCP client connected from %s (fd %d)",
                 inet_ntoa(peer.sin_addr), fd);
        /* No uart_flush_input here: the reader drains continuously, so
         * there is no stale backlog to skip any more. */
    }
}

/* ---- HTTP: dashboard ---- */
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    /* no-store, for the same reason the telemetry endpoints below set it,
     * and it is not a micro-optimisation to skip: this page ships INSIDE
     * the C6 binary, so the only way to update it is a reflash. Served
     * with no cache directive the browser is free to cache it heuristically
     * (phones do, aggressively), and the user then sees a stale dashboard
     * after a perfectly good flash - indistinguishable from a broken build,
     * and it cost a debugging round to find.
     *
     * This is the same hazard the project's already names as "a C6 that was not
     * reflashed after an S3 telemetry change" wearing a different hat: the
     * page and the data it parses drift apart, and the page loses. The
     * whole document is ~60 KB over LAN; there is nothing to save here. */
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, index_html_start,
                           index_html_end - index_html_start - 1);
}

/* Browsers request /favicon.ico unprompted; without a route the server
 * logged a 404 pair on every page load. 204 answers it once and the
 * browser stops asking, rather than shipping an icon nobody needs. */
static esp_err_t favicon_get(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

/* GET /telem -> latest FC CSV line, text/plain. Empty body = nothing
 * received yet (FC off / wire loose); the page shows LINK DOWN. */
static esp_err_t telem_get(httpd_req_t *req)
{
    char out[CSV_LINE_MAX] = "";
    if (xSemaphoreTake(s_csv_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        strlcpy(out, s_last_csv, sizeof(out));
        xSemaphoreGive(s_csv_mtx);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out, strlen(out));
}

/* POST /pid, body "kp,ki,kd,kp_yaw,ki_yaw,kd_yaw" (6 floats).
 * Validated here (finite, [0,100] - a coarse fence) AND on the FC, where
 * params_set applies each gain's own, TIGHTER bounds (e.g. rate_kd <= 1,
 * yaw_ki <= 5; GET lists them). A value that passes here can still be
 * refused there - "OK" from this handler only means "sent". */
static esp_err_t pid_post(httpd_req_t *req)
{
    char body[128];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "empty body", HTTPD_RESP_USE_STRLEN);
    }
    body[len] = 0;

    float v[6];
    if (sscanf(body, "%f,%f,%f,%f,%f,%f",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "expected 6 comma-separated numbers",
                               HTTPD_RESP_USE_STRLEN);
    }
    for (int i = 0; i < 6; i++) {
        if (!isfinite(v[i]) || v[i] < 0.0f || v[i] > 100.0f) {
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req, "values must be finite, 0..100",
                                   HTTPD_RESP_USE_STRLEN);
        }
    }

    /* Leading newline: see cmd_post. */
    char out[160];
    int n = snprintf(out, sizeof(out), "\nPID,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                     (double)v[0], (double)v[1], (double)v[2],
                     (double)v[3], (double)v[4], (double)v[5]);
    uart_write_bytes(TELEM_UART, out, n);
    ESP_LOGI(TAG, "PID sent to FC: %s", out);

    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

/* GET /console -> the buffered FC command replies, oldest first, plus a
 * sequence number so the page can tell "nothing new" from "empty". */
static esp_err_t console_get(httpd_req_t *req)
{
    static char out[CONSOLE_LINES * CONSOLE_COLS + 64];
    int len = 0;

    if (xSemaphoreTake(s_csv_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        len = snprintf(out, sizeof(out), "%lu\n", (unsigned long)s_console_count);
        uint32_t n = s_console_count < CONSOLE_LINES ? s_console_count : CONSOLE_LINES;
        uint32_t start = (s_console_head + CONSOLE_LINES - n) % CONSOLE_LINES;
        for (uint32_t i = 0; i < n && len < (int)sizeof(out) - 2; i++) {
            const char *l = s_console[(start + i) % CONSOLE_LINES];
            len += snprintf(out + len, sizeof(out) - len, "%s\n", l);
        }
        xSemaphoreGive(s_csv_mtx);
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out, len);
}

/* POST /cmd, body = one command line (STATS, GYROCAL, STREAM,0 ...).
 * Forwarded verbatim to the FC, which owns the interpreter and its own
 * safety gates (it refuses GYROCAL/MAGSET while armed). Validated here
 * only for framing: printable ASCII, bounded length, no embedded
 * newlines that could inject a second command. */
static esp_err_t cmd_post(httpd_req_t *req)
{
    char body[160];
    int len = httpd_req_recv(req, body, sizeof(body) - 2);
    if (len <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "empty command", HTTPD_RESP_USE_STRLEN);
    }
    body[len] = 0;

    /* strip trailing whitespace the browser may have added */
    while (len > 0 && (body[len-1] == '\n' || body[len-1] == '\r' ||
                       body[len-1] == ' ')) {
        body[--len] = 0;
    }
    if (len == 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "empty command", HTTPD_RESP_USE_STRLEN);
    }
    for (int i = 0; i < len; i++) {
        if (body[i] < 0x20 || body[i] > 0x7E) {
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req, "command must be printable ASCII, one line",
                                   HTTPD_RESP_USE_STRLEN);
        }
    }

    /* Leading newline : the wire to the S3 picks up glitches
     * that decode as 0xFF, and the S3 used to glue them onto the front of
     * the next command ("unknown command: \xFF\xFF...", then the retyped
     * command worked). The S3 now drops a partial line after 20 ms of
     * silence (components/telemetry/telem_line.c); this newline covers
     * noise that lands inside those 20 ms - it ends the noise as a line of
     * its own, which the S3 refuses as non-printable. One write, so the
     * two newlines and the command leave back to back. */
    char msg[sizeof(body) + 1];
    msg[0] = '\n';
    memcpy(msg + 1, body, (size_t)len);
    msg[len + 1] = '\n';
    uart_write_bytes(TELEM_UART, msg, len + 2);
    ESP_LOGI(TAG, "cmd -> FC: %.*s", len, body);

    return httpd_resp_send(req, "sent", HTTPD_RESP_USE_STRLEN);
}

static void httpd_start_all(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();

    /* The dashboard must not be able to starve the TCP capture bridge.
     *
     * Measured with the dashboard open in a browser, a client
     * connecting to :5555 completed the TCP handshake and was then reset
     * with zero bytes delivered. Cause is the socket pool, not the bridge.
     * HTTPD_DEFAULT_CONFIG asks for max_open_sockets = 7 with LRU purge
     * DISABLED; add httpd's listener and control socket, mDNS, and the
     * bridge's own listener, and that overruns CONFIG_LWIP_MAX_SOCKETS.
     * accept() then fails, and because lwIP completes the handshake in the
     * listen backlog before the app ever calls accept(), the client has
     * already printed "connected" by the time its connection is dropped -
     * which is why this looked like the bridge hanging up rather than a
     * resource limit.
     *
     * Two changes, both needed. Capping httpd at 4 leaves headroom for the
     * bridge no matter how many browser tabs are open, and lru_purge_enable
     * makes httpd evict its own oldest idle connection instead of consuming
     * sockets the rest of the system needs. LWIP_MAX_SOCKETS is also raised
     * 10 -> 16 in sdkconfig.defaults; the cap alone would be enough today,
     * but not once anything else on this board wants a socket.
     *
     * This matters beyond the mag capture: the FFT sweep wants the
     * dashboard open to watch throttle and motors WHILE the capture runs. */
    cfg.max_open_sockets  = 4;
    cfg.lru_purge_enable  = true;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));

    const httpd_uri_t root  = { .uri = "/",        .method = HTTP_GET,  .handler = root_get    };
    const httpd_uri_t fav   = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get };
    const httpd_uri_t telem = { .uri = "/telem",   .method = HTTP_GET,  .handler = telem_get   };
    const httpd_uri_t cons  = { .uri = "/console", .method = HTTP_GET,  .handler = console_get };
    const httpd_uri_t pid   = { .uri = "/pid",     .method = HTTP_POST, .handler = pid_post    };
    const httpd_uri_t cmd   = { .uri = "/cmd",     .method = HTTP_POST, .handler = cmd_post    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &fav));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &telem));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &cons));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &pid));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &cmd));
}

/* ---- WiFi: join whichever configured network is actually in range ----
 *
 * Two networks are configured (see wifi_creds.h): the house router for
 * bench work and a phone hotspot for the field. Rather than hardcoding
 * one and retrying blindly, the manager task SCANS first and connects to
 * whichever configured SSID is present with the strongest signal. An
 * absent network therefore costs one scan, not a chain of association
 * timeouts, and no rebuild is needed to move between them.
 *
 * Note the two networks hand out different subnets, so the dashboard's
 * IP changes with the network. The address is logged on every got-IP,
 * and it is also visible in the router's / phone's client list. */
typedef struct { const char *ssid; const char *pass; } sta_net_t;
static const sta_net_t s_nets[] = STA_NETWORKS;
#define N_STA_NETS (sizeof(s_nets) / sizeof(s_nets[0]))

#define WIFI_GOT_IP_BIT      BIT0
#define WIFI_DISCONNECT_BIT  BIT1
static EventGroupHandle_t s_wifi_events;
static volatile uint8_t   s_disc_reason = 0;

/* The driver's reason code is the difference between "that password is
 * wrong" and "the AP hiccuped, try again" - guessing wastes bench time,
 * so name the ones that actually show up. */
static const char *disc_reason_str(uint8_t r)
{
    switch (r) {
    case WIFI_REASON_AUTH_EXPIRE:            return "auth expired";
    case WIFI_REASON_ASSOC_EXPIRE:           return "assoc expired";
    case WIFI_REASON_ASSOC_TOOMANY:          return "AP full (too many clients)";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "4-way handshake timed out";
    case WIFI_REASON_BEACON_TIMEOUT:         return "beacon timeout (out of range)";
    case WIFI_REASON_NO_AP_FOUND:            return "AP not found";
    case WIFI_REASON_AUTH_FAIL:              return "auth failed (wrong password)";
    case WIFI_REASON_ASSOC_FAIL:             return "assoc failed";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:      return "handshake timeout";
    case WIFI_REASON_CONNECTION_FAIL:        return "connection failed";
    default:                                 return "see esp_wifi_types.h";
    }
}

/* A first attempt that dies in the 4-way handshake is normal with phone
 * hotspots and costs nothing to retry; only these mean "stop, go look
 * somewhere else". */
static bool reason_is_fatal(uint8_t r)
{
    return r == WIFI_REASON_NO_AP_FOUND ||
           r == WIFI_REASON_AUTH_FAIL   ||
           r == WIFI_REASON_ASSOC_TOOMANY;
}

static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    /* The handler only reports; wifi_manager_task owns every state
     * change. Calling esp_wifi_connect() from here (the old code) fights
     * the task for control and makes a scan impossible. */
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        s_disc_reason = d ? d->reason : 0;
        xEventGroupSetBits(s_wifi_events, WIFI_DISCONNECT_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "DASHBOARD  http://" MDNS_HOSTNAME ".local/  or  http://"
                 IPSTR "/", IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "RAW BRIDGE " IPSTR ":%d", IP2STR(&e->ip_info.ip),
                 TELEM_TCP_PORT);
        xEventGroupSetBits(s_wifi_events, WIFI_GOT_IP_BIT);
    }
}

/* Blocking scan -> index into s_nets of the best present network, or -1.
 * Must be called only while disconnected. */
static int scan_pick_network(void)
{
    wifi_scan_config_t sc = { .show_hidden = false };
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) return -1;

    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n == 0) { esp_wifi_clear_ap_list(); return -1; }
    if (n > 32) n = 32;

    wifi_ap_record_t *recs = calloc(n, sizeof(*recs));
    if (!recs) { esp_wifi_clear_ap_list(); return -1; }
    /* This call is also what frees the driver's internal AP list, so it
     * runs even when the result is ignored. */
    esp_wifi_scan_get_ap_records(&n, recs);

    int best = -1, best_rssi = -127;
    for (uint16_t i = 0; i < n; i++) {
        for (size_t k = 0; k < N_STA_NETS; k++) {
            if (strcmp((const char *)recs[i].ssid, s_nets[k].ssid) == 0 &&
                recs[i].rssi > best_rssi) {
                best      = (int)k;
                best_rssi = recs[i].rssi;
            }
        }
    }
    if (best >= 0) {
        ESP_LOGI(TAG, "scan: joining \"%s\" (%d dBm), %u APs seen",
                 s_nets[best].ssid, best_rssi, n);
    } else {
        ESP_LOGW(TAG, "scan: none of the %u configured networks in range "
                 "(%u APs seen) - check the SSID is 2.4 GHz and spelled exactly",
                 (unsigned)N_STA_NETS, n);
    }
    free(recs);
    return best;
}

#define JOIN_ATTEMPTS  3

static void wifi_manager_task(void *arg)
{
    (void)arg;
    for (;;) {
        int idx = scan_pick_network();
        if (idx < 0) {
            vTaskDelay(pdMS_TO_TICKS(5000));    /* nothing here yet */
            continue;
        }

        wifi_config_t sta = { 0 };
        strlcpy((char *)sta.sta.ssid,     s_nets[idx].ssid, sizeof(sta.sta.ssid));
        strlcpy((char *)sta.sta.password, s_nets[idx].pass, sizeof(sta.sta.password));
        sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));

        /* Retry the SAME AP before scanning again. A hotspot dropping the
         * first 4-way handshake is routine - the old code treated that as
         * "wrong password", threw the result away and spent ~3.5 s on a
         * rescan that picked the same network anyway. */
        for (int attempt = 1; attempt <= JOIN_ATTEMPTS; attempt++) {
            xEventGroupClearBits(s_wifi_events,
                                 WIFI_GOT_IP_BIT | WIFI_DISCONNECT_BIT);
            esp_wifi_connect();

            EventBits_t b = xEventGroupWaitBits(
                s_wifi_events, WIFI_GOT_IP_BIT | WIFI_DISCONNECT_BIT,
                pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

            if (b & WIFI_GOT_IP_BIT) {
                /* Park until the link actually drops, then start over
                 * with a fresh scan - which is also how the board moves
                 * between the router and the hotspot without a reflash. */
                xEventGroupWaitBits(s_wifi_events, WIFI_DISCONNECT_BIT,
                                    pdTRUE, pdFALSE, portMAX_DELAY);
                ESP_LOGW(TAG, "\"%s\" dropped (%s), re-selecting",
                         s_nets[idx].ssid, disc_reason_str(s_disc_reason));
                break;
            }

            uint8_t r = (b & WIFI_DISCONNECT_BIT) ? s_disc_reason : 0;
            esp_wifi_disconnect();

            if (!(b & WIFI_DISCONNECT_BIT)) {
                ESP_LOGW(TAG, "\"%s\": no result in 15 s, rescanning",
                         s_nets[idx].ssid);
                break;
            }
            if (reason_is_fatal(r)) {
                ESP_LOGE(TAG, "\"%s\": %s (reason %u) - not retrying",
                         s_nets[idx].ssid, disc_reason_str(r), r);
                break;
            }
            ESP_LOGW(TAG, "\"%s\": %s (reason %u), attempt %d/%d",
                     s_nets[idx].ssid, disc_reason_str(r), r,
                     attempt, JOIN_ATTEMPTS);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void wifi_sta_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_evt, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_evt, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "%u networks configured, scanning (2.4 GHz only - the C6 "
             "has no 5 GHz radio)", (unsigned)N_STA_NETS);
    xTaskCreate(wifi_manager_task, "wifi_mgr", 4096, NULL, 3, NULL);
}

#if !USE_WIFI_STA
static void wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .password = AP_PASS,
            .channel = 1,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP \"%s\" up, dashboard at http://192.168.4.1", AP_SSID);
}
#endif /* !USE_WIFI_STA */

/* Publish http://drone.local/. Non-fatal: a failure here costs the
 * friendly name, not the dashboard, which stays reachable by IP. */
static void mdns_start(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed (%s) - use the IP address",
                 esp_err_to_name(err));
        return;
    }
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set("Drone telemetry bridge");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS up: http://" MDNS_HOSTNAME ".local/");
}

static void telem_uart_init(void)
{
    const uart_config_t uc = {
        .baud_rate  = TELEM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(TELEM_UART, TELEM_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(TELEM_UART, &uc));
    /* Bidirectional: RX carries telemetry down, TX
     * carries console commands and PID lines up to FC GPIO 21. */
    ESP_ERROR_CHECK(uart_set_pin(TELEM_UART, TELEM_TX_GPIO, TELEM_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();               /* wifi needs NVS */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    s_csv_mtx = xSemaphoreCreateMutex();
    telem_uart_init();

#if USE_WIFI_STA
    wifi_sta_start();
#else
    wifi_ap_start();
#endif
    mdns_start();
    httpd_start_all();

    /* AFTER wifi start: that is what brings up the LWIP TCP/IP stack.
     * Creating socket tasks earlier trips "Invalid mbox". */
    xTaskCreate(tcp_accept_task,  "tcp_accept",  4096, NULL, 4, NULL);
    xTaskCreate(uart_reader_task, "uart_reader", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "telemetry bridge + dashboard up (joystick RC removed)");
}


