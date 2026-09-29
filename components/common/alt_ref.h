#ifndef ALT_REF_H
#define ALT_REF_H

/* Altitude reference - the author's scheme,.
 *
 * DISPLAY ONLY. Nothing in this file may drive a control loop yet, and
 * that is his instruction, not a limitation: "just displayed on the
 * dashboard for starters.. if this worked then we can implement it in
 * the vert filter". It runs beside vert_filter, sees the same sensors,
 * and is logged so the two can be compared on real flights before either
 * gets authority.
 *
 * ---------------------------------------------------------------------
 * WHY IT EXISTS - what vert_filter got wrong
 * ---------------------------------------------------------------------
 * vert_filter fuses the barometer as an ABSOLUTE altitude, splitting it
 * into z and a baro bias bb. That model needs a barometer whose noise is
 * near BARO_SIGMA (0.073 m). MEASURED across the flights, the
 * DPS310's 1-second spread was 0.95 to 1.43 m - THIRTEEN TO TWENTY TIMES
 * the model. With a barometer that noisy, z and bb are not separately
 * observable and z wanders:
 *
 *     s166   estimate reached -3.46 m   hold NOT engaged
 *     s171   estimate reached -6.00 m   ToF reading 0.18 m throughout
 *
 * s166 matters most: the estimate broke with alt-hold completely off, so
 * this was never a controller bug. It is the sensor model.
 *
 * ---------------------------------------------------------------------
 * THE SCHEME
 * ---------------------------------------------------------------------
 * Stop asking the barometer what altitude it is. It does not know, and
 * on this airframe it is wrong by metres. Ask it only what has CHANGED,
 * which is the one thing a barometer is good at.
 *
 *   1. At start, latch the pressure altitude. Everything after is
 *      RELATIVE to that - no sea level, no QNH, no 8.3 m per hPa of
 *      weather error.
 *   2. ToF alive (below ~2 m): the ToF IS the altitude. It does not lie -
 *      it returns a range or the NO_TARGET sentinel, never a plausible
 *      wrong number.
 *   3. ToF out of range: coast on baro_relative + a stored offset.
 *   4. ToF back, and the baro's implied altitude disagreeing for
 *      ALT_REF_AGREE_S seconds: re-reference the offset onto the ToF.
 *
 * Step 4's delay is the load-bearing part and it is the author's: without it,
 * a person or a box passing under the aircraft would instantly redefine
 * where the ground is. Five seconds of SUSTAINED disagreement is not
 * something a passing object produces.
 *
 * MEASURED refinement: correcting toward the ToF at a bounded RATE
 * rather than snapping. Snapping tracks the ToF perfectly but hands the
 * controller a step - across 129 real handovers, median 0.201 m and
 * worst 1.221 m. Slewing at 2 m/s cuts the worst step to 0.011 m, a
 * hundredfold, and finishes correcting in about 0.6 s.
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float z;            /* the estimate, m above the start datum   */
    float baro0;        /* pressure altitude latched at start, m   */
    float offset;       /* added to baro-relative when coasting, m */
    float disagree_s;   /* seconds the baro has been disagreeing   */
    /* Seconds since a FRESH baro sample. Added.
     *
     * baro_ok used to gate only the baro0 latch; after that brel was
     * computed from baro_alt_m unconditionally, and the call site was
     * passing `baro_press_pa > 0.0f` - true forever after the first
     * successful read. So a DEAD barometer coasted silently, which is
     * the frozen-cache failure that destroyed the aircraft on
     *, inside the module being proposed to replace the
     * filter's altitude. A clock, not a bool: the same shape
     * vert_filter already uses, because "it answered once" and "it is
     * answering now" are different claims. */
    float baro_stale_s;
    bool  have_baro0;
    bool  have_z;
    bool  tof_live;     /* was the ToF usable on the last update   */
    /* Is z meaningful RIGHT NOW?
     *
     * True whenever the ToF can see the ground - below 2 m the ToF
     * alone is sufficient and a dead barometer does not matter. False
     * when the ToF is out of range AND the barometer has stopped
     * answering, because then z is coasting on a number that is no
     * longer being measured. A display-only module that shows a stale
     * value without saying so is worse than showing nothing. */
    bool  valid;
} alt_ref_t;

void alt_ref_init(alt_ref_t *a);

/* One update.
 *
 * baro_alt_m  pressure-derived altitude, absolute and arbitrary - the
 *             offset exists precisely so the caller need not care.
 * baro_ok     the barometer answered this cycle.
 * tof_m       ToF range in metres, already screened for the NO_TARGET
 *             sentinel; pass <= 0 when there is no usable range.
 * dt          seconds. */
void alt_ref_update(alt_ref_t *a, float baro_alt_m, bool baro_ok,
                    float tof_m, float dt);

#endif /* ALT_REF_H */
