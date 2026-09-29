#ifndef TBSM10_H
#define TBSM10_H

#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "gnss_guard.h"

// --- PIN CONFIGURATION ---
// MOVED OFF 43/44 ON, and this one is not cosmetic. On the
// replacement S3 board, 43/44 (U0TXD/U0RXD) are ALSO wired to the second
// USB-C port's USB-serial chip. That chip drives GPIO44 whenever it has
// power, fighting the GPS's own TX on the same wire, and its receive LED
// blinking ~10x a second was it watching GPS traffic. Wiring now:
//   GPIO 1 = TX (ESP32 talks to GPS RX)
//   GPIO 2 = RX (ESP32 listens to GPS TX)
// 1 and 2 are plain GPIOs (ADC1_CH0/CH1); neither is a strapping pin.
// Side benefit: the ROM bootloader still prints its reset banner on the
// UART0 DEFAULT pad (43) before uart_set_pin runs, so the GPS no longer
// receives boot text at all.
#define TBSM10_TX_PIN       1
#define TBSM10_RX_PIN       2

// UART0 stays the GPS's: the console runs on USB-Serial-JTAG
// (see sdkconfig.defaults). UART1 = iBUS, UART2 = telemetry.
#define TBSM10_UART_NUM     UART_NUM_0
#define TBSM10_BAUD_RATE    38400 
#define TBSM10_BUF_SIZE     1024

/* Protocol selection - Stage A,.
 *
 * 0 = UBX. UBX-NAV-PVT at 10 Hz carrying NED velocity and hAcc/vAcc, which
 *     is what a filter actually needs and what NMEA cannot give.
 * 1 = the NMEA driver that flew previously. Verified working: 3D fix,
 *     12 sats, position hand-checked against the raw sentence. Kept whole
 *     so this one #define walks the change back if UBX misbehaves in the
 *     field, which is the entire point of a transition flag.
 *
 * NOTE: the module is configured over TBSM10_TX_PIN (GPIO 1 since the
 * rebuild), and UBX configuration + NAV-PVT have worked over
 * it since. If NAV-PVT ever stops while bytes keep coming, TX (or the
 * baud) is still the first suspect - NMEA output needs no command, so
 * working lat/lon alone never proves TX. The driver reports exactly that
 * case once a second rather than going quiet, and GSEC prints the
 * module's ACK/NAK per config group. */
#define TBSM10_USE_NMEA 0

// Initialize and start the FreeRTOS task
void tbsm10_init_and_start_task(void);

/* Latch the current fix as the home point for the local tangent plane.
 * Called on the arm edge. Returns true if a fix good enough to trust was
 * available; false leaves the previous home (or none) in place, so a bad
 * fix at arm can never silently move the origin. */
bool tbsm10_capture_home(void);

/* Forget home. nav_state.home_valid goes to 0 and NED output stops. */
void tbsm10_clear_home(void);

/* Copy the GNSS interference guard's current verdict. Returns 0 if the
 * state mutex was busy, so the caller reports that rather than printing a
 * half-copied verdict. See components/gnss_guard/gnss_guard.h. */
int tbsm10_guard_status(gg_status_t *out);

/* Release a latched GNSS denial by hand. Bench use - it overrides the
 * latch, not the evidence: if the interferer is still there the guard is
 * back in the same state within a second. */
void tbsm10_guard_clear(void);

/* One line for the console: the module's answer (ACK / NAK / no-reply) to
 * each configuration message of the last round - core, then the guard's
 * NAV-SIG, MON-RF, SEC-SIG and ITFM. "core NAK" means NAV-PVT cannot start;
 * a guard NAK costs only that input. */
void tbsm10_cfg_report(char *out, size_t n);

#endif // TBSM10_H