#ifndef ITERM_GATE_H
#define ITERM_GATE_H

/* Rate-loop I-term ground hold.
 *
 * WHY IT EXISTS. On its legs the frame cannot rotate, so a rate error never
 * closes and the rate integrators keep every stick input for the rest of the
 * armed period, up to their 100 us clamp. Measured on the bench
 * logs (props off): s25 held pitch at -98.5 us, leaving the motors at
 * 1225/1244/1062/1050 with the sticks centred; s26 held roll at +42.5 us.
 * Carried into the air, -98.5 us needs ~82 dps of P to cancel, i.e. a ~20 deg
 * lean that bleeds off with a 24 s time constant (rate kp/ki).
 *
 * THE RULE. The ToF, measured against its own reading at the arm edge,
 * decides whether the aircraft is sitting on its legs:
 *   - before the first liftoff: hold while the ToF is within 3 cm of the arm
 *     reading, AT ANY THROTTLE - so a slow spool-up and a props-off bench run
 *     both stay at zero;
 *   - liftoff = a fresh reading MORE than 3 cm above the arm reading;
 *   - after liftoff: hold only while within 3 cm AND the throttle stick is
 *     below 1250 us. Throttle has to be part of it here because a low hover
 *     skims the ground: s207 (a real flight) spent 9.4 s at
 *     1431 us with the ToF within 3 cm of its arm reading;
 *   - a reading that is not fresh - chip absent, nothing in range above
 *     ~1.7 m (65535), sample counter not advancing - is NEVER "on the
 *     ground": the integrators run, exactly as before this gate existed;
 *   - no fresh reading AT THE ARM EDGE means no reference, and the gate
 *     falls back to Betaflight's airmode latch (core.c: hold until the
 *     throttle stick first reaches 1250 us, then never again until disarm).
 *
 * Replayed through this file (tools/iterm_gate_test.py): on s207 the I-term
 * goes live at liftoff, 22.48 s - 9.8 s later than a plain 1250 us throttle
 * rule would have let it - and is never held while airborne; on the s19
 * bench log it stays at zero through a full-throttle ramp with the sticks
 * being moved.
 *
 * Same rules as vel_hold.c: no libc, no FreeRTOS, no driver headers, so the
 * host test compiles THIS file and replays real logs through it. */

#include <stdbool.h>
#include <stdint.h>

#define ITERM_GATE_THR_US    1250u    /* throttle stick below this: not asking to fly */
#define ITERM_GATE_NEAR_MM   30       /* within this of the arm reading: on its legs */
#define ITERM_GATE_FRESH_US  500000   /* the ToF sample counter must have moved this recently */

/* The ToF driver's sentinels (VL53L0X_MM_NONE / VL53L0X_MM_NO_TARGET in
 * vl53l0x.h), repeated here so this file needs no driver header. */
#define ITERM_GATE_TOF_NONE       0u
#define ITERM_GATE_TOF_NO_TARGET  65535u

/* Blackbox bits (bb_frame_t.ctrl_flags). */
#define ITERM_GATE_F_HELD    (1u << 0)  /* rate integrators zeroed this cycle */
#define ITERM_GATE_F_TOF_REF (1u << 1)  /* reference captured at arm; 0 = throttle-latch fallback */
#define ITERM_GATE_F_LIFTED  (1u << 2)  /* the ToF has shown liftoff since arm */
#define ITERM_GATE_F_FRESH   (1u << 3)  /* this cycle's ToF reading was fresh */
#define ITERM_GATE_F_THR     (1u << 4)  /* fallback latch: throttle has reached 1250 us */

typedef struct {
    bool     was_armed;
    bool     ref_ok;      /* a fresh ToF reading existed at the arm edge */
    uint16_t ref_mm;
    bool     lifted;      /* the ToF has shown liftoff since the arm edge */
    bool     thr_seen;    /* fallback latch: throttle reached ITERM_GATE_THR_US */
    bool     have_seq;    /* the sample counter has been seen at least once */
    bool     seq_moved;   /* ...and has advanced since, so seq_us is real */
    uint32_t seq;         /* the ToF driver's completed-sample counter */
    int64_t  seq_us;      /* when that counter last advanced */
    bool     fresh;       /* this cycle's verdicts, kept for the blackbox */
    bool     hold;
} iterm_gate_t;

void iterm_gate_init(iterm_gate_t *g);

/* One flight-loop cycle, armed or not. Returns true when the rate integrators
 * must be zeroed this cycle.
 *
 * Call it every cycle, disarmed included: it has to watch the ToF sample
 * counter move BEFORE the arm edge to know whether the reading it takes as
 * the reference is fresh. Disarmed it returns false - the disarmed path
 * already resets every PID.
 *
 * throttle_us is the raw throttle STICK, not the mixer's throttle: the
 * question is whether the pilot is asking to fly. tof_mm and tof_seq are
 * vl53l0x_cached_mm() and vl53l0x_sample_seq(). */
bool iterm_gate_step(iterm_gate_t *g, bool armed, uint16_t throttle_us,
                     uint16_t tof_mm, uint32_t tof_seq, int64_t now_us);

/* ITERM_GATE_F_* for the blackbox, describing the last step. */
uint8_t iterm_gate_flags(const iterm_gate_t *g);

#endif /* ITERM_GATE_H */
