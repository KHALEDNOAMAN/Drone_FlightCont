#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "nav_state.h" 

#define TELEMETRY_UART_PORT_NUM       UART_NUM_2 
/* Pins (since 2c519be):
 *   S3 GPIO 47 (TX, S3 talks)    -> C6 D5 / GPIO23 (its RX)
 *   S3 GPIO 21 (RX, S3 listens)  <- C6 D6 / GPIO16 (its TX)
 * 47 is the SPICLK_P pad, used only by a differential-clock external
 * flash, so it is a plain GPIO here. UART2 reaches both pads through the
 * GPIO matrix. History (41/42 previously, a failed 21/47 attempt the
 * same day, the SD card taking 39-42) is.md. */
#define TELEMETRY_TX_PIN              47
#define TELEMETRY_RX_PIN              21
/* 921600, not 115200. The FFT capture build streams gyro at the full
 * ~416 Hz loop rate over this link (~15 KB/s of text); 115200 tops out
 * near 11.5 KB/s and would drop samples. The 4 Hz CSV telemetry is
 * unaffected either way. The C6 bridge must match. */
#define TELEMETRY_BAUD_RATE           921600

// Initializes UART and starts the FreeRTOS transmission task
void telemetry_init_and_start(void);

/* Raw passthrough on the telemetry UART, for the FFT capture stream.
 * Returns bytes written, or -1. */
int telemetry_write_raw(const char *buf, int len);

/* Send one console/response line to the C6, '#'-prefixed and newline
 * terminated. The '#' is what lets the C6 tell a command response apart
 * from telemetry CSV (starts with a digit) and FFT gyro lines ("G,").
 * Exists so serial commands can be driven from the dashboard instead of
 * typed into a USB console that is printing 50 CSV lines a second. */
void telemetry_write_console(const char *line);

/* Multi-line variant: splits on '\n' and prefixes every line. */
void telemetry_write_console_block(const char *text);

/* Commands arriving over the telemetry link that telemetry.c does not
 * handle itself are passed to this handler. debug_stream registers the
 * FC's serial command interpreter here at boot, which is what makes
 * STATS/GYROCAL/MAGSET/STREAM reachable from the dashboard.
 * Registered once at startup; NULL means "telemetry-link commands are
 * ignored", which is the state before debug_stream_init runs. */
typedef void (*telemetry_cmd_fn)(const char *line);
void telemetry_set_cmd_handler(telemetry_cmd_fn fn);

#endif // TELEMETRY_H
