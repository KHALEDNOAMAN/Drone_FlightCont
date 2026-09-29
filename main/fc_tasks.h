#pragma once

#include "nav_state.h"   /* HIL_MODE */
#include "ibus.h"        /* IBUS_CHANNELS */
#include <stdint.h>
#include <stdbool.h>

void fc_tasks_init(void);

/* Bench motor test (MOT command): spin ONE motor while DISARMED.
 * motor_1based 1-4 selects M1-M4; anything else stops the test.
 * us is clamped to [1000, MOT_TEST_MAX_US], ms to MOT_TEST_MAX_MS.
 * The flight task applies it only in the disarmed path and any arm
 * attempt cancels it immediately. */
void fc_motor_test(uint8_t motor_1based, uint16_t us, uint32_t ms);

/* ESC range calibration (ESCCAL command): teach all four ESCs the same
 * 1000-2000 us range simultaneously. stage 1 = open 10 s confirm window,
 * stage 2 = drive 2000 us on all four (only from stage 1; 60 s auto-
 * abort), stage 0 = drive 1000 us / abort. Returns the resulting state,
 * -1 if the transition is refused. DRIVES FULL THROTTLE - props off. */
int fc_esccal(int stage);

/* Stick passthrough (ESCPASS command): raw throttle channel to all four
 * outputs while disarmed, for the classic TX-based ESC calibration
 * ritual. 120 s auto-off; arm attempt cancels; without a live RC link
 * the outputs are held at 1000. DRIVES WHATEVER THE STICK SAYS - props
 * off. */
void fc_esc_passthrough(bool on);

/* dt clamp watch counters, for STATS. The flight loop clamps dt to
 * 2..6 ms while running at 250 Hz, so `lo` and `hi` should both be 0; a
 * non-zero `hi` means the loop was starved and the PID integrator
 * under-counted real time for those cycles. Any pointer may be NULL. */
void fc_dt_clamp_stats(uint32_t *samples, uint32_t *lo, uint32_t *hi,
                       uint32_t *max_us);

/* NaN/Inf values caught on the motor path and replaced with safe ones
 * (see s_nonfinite in fc_tasks.c). Must be 0; STATS prints it. */
uint32_t fc_nonfinite_count(void);

#if HIL_MODE
/* Inject a fake RC frame from the serial bench link (debug_stream). */
void fc_hil_inject_rc(const uint16_t ch[IBUS_CHANNELS], bool failsafe);

#endif
