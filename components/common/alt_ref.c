/* Altitude reference. See alt_ref.h for the scheme and why it exists.
 *
 * No math.h, no libc, same rule as vert_filter.c and horiz_filter.c, so
 * the host test can compile and run it. */

#include "alt_ref.h"

#define AR_FABSF __builtin_fabsf

/* Above this the VL53L0X is at the edge of its range and about to report
 * NO_TARGET anyway. Matches VF_TOF_MAX_M. */
#define ALT_REF_TOF_MAX_M    2.00f

/* How long the barometer's implied altitude must disagree with the ToF
 * before the offset is moved onto the ToF.
 *
 * USER'S NUMBER and the load-bearing part of the design. Without it, a
 * person or a box passing under the aircraft redefines where the ground
 * is, instantly, and the altitude hold then flies to match. Five seconds
 * of SUSTAINED disagreement is not something a passing object produces -
 * it is what a genuinely drifted barometer produces. */
#define ALT_REF_AGREE_S      5.0f

/* How far apart they must be to count as disagreeing at all. Below this
 * the barometer is tracking well enough and nothing needs correcting;
 * measured baro drift on this airframe is ~18 cm per 100 s sitting
 * still, so 0.30 m is a few minutes of honest drift. */
#define ALT_REF_AGREE_TOL_M  0.30f

/* Rate limit on correcting the estimate toward the ToF, m/s.
 *
 * MEASURED across 129 real ToF handovers in the flights:
 * snapping straight onto the ToF gives a median step of 0.201 m and a
 * worst of 1.221 m, because the barometer has drifted while the ToF was
 * out of range and the whole error arrives in one frame. At 2 m/s the
 * worst step becomes 0.011 m - a hundredfold better - and the correction
 * completes in about 0.6 s.
 *
 * The cost, stated honestly: for those 0.6 s the estimate LAGS the ToF
 * by up to 1.2 m. A gradual error a controller can track beats a step it
 * cannot, but this is the number to revisit if the lag ever matters more
 * than the step. */
#define ALT_REF_SLEW_MS      2.0f

/* How long the barometer may go without answering before a COASTING
 * estimate stops being believable.
 *
 * Only matters out of ToF range - below 2 m the ToF is the altitude and
 * the barometer is irrelevant. One second matches the flight loop's
 * existing ALT_BARO_GRACE_CYCLES: long enough to ride out a bus glitch
 * on a chip with a documented history of them, short enough that a
 * genuinely dead baro cannot keep a coasted number on the dashboard
 * looking live. */
#define ALT_REF_BARO_STALE_S 1.0f

void alt_ref_init(alt_ref_t *a)
{
    a->z = 0.0f;
    a->baro0 = 0.0f;
    a->offset = 0.0f;
    a->disagree_s = 0.0f;
    a->baro_stale_s = 0.0f;
    a->have_baro0 = false;
    a->have_z = false;
    a->tof_live = false;
    a->valid = false;
}

void alt_ref_update(alt_ref_t *a, float baro_alt_m, bool baro_ok,
                    float tof_m, float dt)
{
    if (dt <= 0.0f) return;

    /* LIVENESS, not just a value. baro_ok says the barometer answered
     * THIS cycle; the clock says how long ago that last was. Everything
     * below that consumes brel has to know the difference. */
    if (baro_ok) a->baro_stale_s = 0.0f;
    else         a->baro_stale_s += dt;
    const bool baro_fresh = (a->baro_stale_s < ALT_REF_BARO_STALE_S);

    /* Latch the start pressure once. Everything downstream is RELATIVE
     * to it, which is what removes the sea-level constant and the 8.3 m
     * per hPa of weather error along with it. */
    if (!a->have_baro0 && baro_ok) {
        a->baro0 = baro_alt_m;
        a->have_baro0 = true;
    }
    const float brel = a->have_baro0 ? (baro_alt_m - a->baro0) : 0.0f;

    const bool live = (tof_m > 0.0f) && (tof_m < ALT_REF_TOF_MAX_M);

    /* Below 2 m the ToF IS the altitude, so a dead barometer costs
     * nothing. Out of range the estimate is coasting on the barometer's
     * change, and if the barometer has stopped answering it is coasting
     * on a number nobody is measuring any more. */
    a->valid = live || (a->have_baro0 && baro_fresh);

    if (!a->have_z) {
        a->z = live ? tof_m : (brel + a->offset);
        a->have_z = true;
        a->tof_live = live;
        return;
    }

    if (live) {
        /* The ToF is the altitude. Approached at a bounded rate rather
         * than jumped to - see ALT_REF_SLEW_MS. */
        const float d = tof_m - a->z;
        const float step = ALT_REF_SLEW_MS * dt;
        if (d > step)       a->z += step;
        else if (d < -step) a->z -= step;
        else                a->z = tof_m;

        /* Has the barometer's own idea of altitude drifted away from the
         * ToF, and STAYED away? Only then is the offset moved. */
        if (AR_FABSF((brel + a->offset) - tof_m) > ALT_REF_AGREE_TOL_M) {
            a->disagree_s += dt;
            if (a->disagree_s >= ALT_REF_AGREE_S) {
                a->offset = tof_m - brel;
                a->disagree_s = 0.0f;
            }
        } else {
            a->disagree_s = 0.0f;
        }
    } else {
        /* Out of ToF range: coast on the barometer's CHANGE plus the
         * offset it was last referenced with. The disagreement clock is
         * cleared because there is nothing to disagree with - letting it
         * run here would let a long climb bank credit toward a
         * re-reference that never got tested against the ground. */
        a->z = brel + a->offset;
        a->disagree_s = 0.0f;
    }
    a->tof_live = live;
}
