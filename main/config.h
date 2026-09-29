#pragma once
#include <stdint.h>

/* ================= HARDWARE PINS ================= */
/* Moved 7 -> 8 and unchanged by the rebuild
 * (7 is now the IMU's SPI CS).
 *
 * CORRECTED the comment here used to claim the FS-iA6B's iBUS
 * output is ~5.7 V and to recommend a series resistor. the author measured it:
 * the receiver FEEDS from 5 V but its iBUS signal is 3.3 V, safe for the
 * S3 directly. No resistor needed.
 *
 * The lead must sit in the receiver's `i-BUS SERVO` port, never `SENS` -
 * SENS is a telemetry-sensor input, carries no channel data, and sits
 * right beside it. */
#define IBUS_GPIO       8

/* ================= RC MAPPING ================= */
#define CH_ROLL         0
#define CH_PITCH        1
#define CH_THROTTLE     2
#define CH_YAW          3
#define CH_ARM          4
/* CH6 (0-based index 5), formerly the alt-hold switch - see ALTHOLD_ON. The receiver
 * carries 14 and only five were in use. Kept OFF the gimbal channels on
 * purpose: the frozen-RC detector watches indices 0-3 for byte-identical
 * repeats, and a switch channel is legitimately constant. */
#define CH_ALTHOLD      5

/* ================= SETTINGS ================= */
#define ARM_ON          1600
/* CH6 "up" threshold. CH6 ENGAGES NOTHING - alt-hold is
 * automatic (param alt_hold_auto). CH6 only gates ARMING (PREARM_ALTHOLD_
 * UNSAFE: CH6 up with no ground reference) and is logged as VERT_F_CH6. A
 * channel that is absent reads 0, i.e. "down". */
#define ALTHOLD_ON      1600
#define THROTTLE_LOW    1050
/* Stick deadband is the runtime param rc_deadband (params.c), not a
 * compile-time constant. */

/* ================= PRE-ARM GATES ================= */
#define PREARM_GYRO_MAX_DPS   3.0f   /* "still" threshold, all axes */
#define PREARM_TILT_MAX_DEG   20.0f  /* max |roll|/|pitch| to allow arming */
#define PREARM_STICK_MAX      50     /* max stick counts off center (10%) */
#define PREARM_MIN_HEARTBEAT  1100   /* fusion cycles; ~2.6 s at the ~416 Hz
                                        DRDY-paced loop - past the 2 s beta
                                        ramp with margin */
/* Tilt and yaw-rate limits are runtime params, not compile-time constants:
 * angle_max_deg / yaw_rate_max in components/params/params.c. The #defines
 * that used to sit here had no references left and had already drifted from
 * the values the board actually flies. */

/* ================= HELPER MATH ================= */
/* 1000-2000 -> -500 to +500 */
static inline int16_t rc_normalize(uint16_t rc) {
    return (int16_t)rc - 1500;
}