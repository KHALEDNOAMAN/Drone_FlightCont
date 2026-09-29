/* P7 stage 2 - altitude-hold controller. See alt_hold.h for the shape of
 * the loop and why the feedforward carries the hover throttle. */

#include "alt_hold.h"

#define AH_FABSF __builtin_fabsf

/* Hover throttle, MEASURED from flight4 sessions 19-22 (the four real
 * hovers already in the ring). Collective motor output during the hover
 * portions of those flights averaged 1383, 1401, 1425 and 1425 us, so
 * ~1420 us total, i.e. 420 us above idle. Taking the higher end on
 * purpose: an FF that is slightly low leaves the integrator to make up
 * the difference upward, which it does gracefully. An FF that is high
 * makes the aircraft climb on engage, which is the frightening
 * direction.
 *
 * NOT a placeholder - this is measured. What IS unmeasured is the pack
 * voltage those hovers ran at (flight4's frames predate vbat_mv being
 * logged, 834b1fb), so v_ref below is the nominal 4S resting voltage and
 * Log B is what pins it down. */
#define AH_HOVER_THR_DEFAULT   420.0f
#define AH_V_REF_DEFAULT       16.0f

/* Gains. DERIVED, then checked in the closed-loop sim - not guessed.
 *
 * The throttle-to-acceleration sensitivity near hover falls out of the
 * measured hover throttle alone, and MASS CANCELS, which is what makes
 * it trustworthy on an airframe that has never been weighed:
 *
 *     thrust = m*g * (span / span_hover)^2
 *     accel  = thrust/m - g = g*((span/span_hover)^2 - 1)
 *     d(accel)/d(span) at span = span_hover  =  2g / span_hover
 *                                            =  2*9.81/420
 *                                            =  0.0467 m/s^2 per us
 *
 * So an inner climb-rate loop with bandwidth w needs kp_vz = w/0.0467.
 * w = 2.8 rad/s (a ~0.35 s response, comfortably inside the ~0.08 s
 * motor lag) gives kp_vz = 60.
 *
 * kp_z is the OUTER loop and must be several times slower than the
 * inner one or the cascade rings. The sim demonstrated exactly that:
 * the first attempt at kp_z = 1.0 against an inner loop of only
 * 1.17 rad/s put the two within a factor of 1.2 of each other and the
 * aircraft oscillated +-0.3 m with 0.48 m/s of vertical velocity while
 * supposedly holding station. kp_z = 0.8 against 2.8 is a factor of
 * 3.5, which is the usual rule and which settles.
 *
 * ki_vz = kp_vz/2 puts the integral time constant at ~2 s: fast enough
 * to absorb a wrong hover-throttle guess within a couple of seconds,
 * slow enough not to chase vibration.
 *
 * STILL PLACEHOLDERS in the sense that matters: the sim shows these are
 * STABLE, not that they are RIGHT. It has no ground effect, no prop
 * wash off the floor, and white-noise vibration instead of the real
 * rotor-order comb. Log B is what confirms them. */
#define AH_KP_VZ_DEFAULT       60.0f    /* us per (m/s)               */
#define AH_KI_VZ_DEFAULT       30.0f    /* us per (m/s)/s             */
#define AH_KP_Z_DEFAULT         0.8f    /* 1/s                        */
#define AH_VZ_MAX_DEFAULT       1.0f    /* m/s at full stick          */
#define AH_I_MAX_DEFAULT      200.0f    /* us                         */
#define AH_THR_DEADBAND        80.0f    /* stick counts               */
#define AH_EXIT_RAMP_S          0.5f

/* Output rails. 1700 is the standing bench cap (STANDING HAZARDS); the
 * controller must not be the thing that discovers what 1900 does. */
#define AH_OUT_MIN           1100.0f
#define AH_OUT_MAX           1700.0f

/* The FF scaling is clamped, hard. A glitching INA226 already produced
 * one 33.41 V sample on this aircraft (exactly 2x real, ffe5be8), and
 * an unclamped v_ref/v_now would turn that into HALF the hover throttle
 * - the aircraft would drop out of the sky on one bad I2C read. Same
 * rule as the battery card: never let one implausible sample through. */
#define AH_VSCALE_MIN           0.8f
#define AH_VSCALE_MAX           1.3f
#define AH_VBAT_MIN_PLAUSIBLE   9.0f
#define AH_VBAT_MAX_PLAUSIBLE  26.0f

/* The largest DATUM shift this controller will move its setpoint by in
 * one cycle. The same number as vert_filter.c's VF_REDATUM_MAX_M, and it
 * has to be: the estimator bounds the step at its source and this is the
 * second wall behind that one. It is NOT shared through a header -
 * alt_hold.c and vert_filter.c deliberately depend on nothing, including
 * each other - so if one number moves, move the other. */
#define AH_DATUM_MAX_M          2.0f

void alt_hold_default_cfg(alt_hold_cfg_t *c)
{
    c->hover_thr    = AH_HOVER_THR_DEFAULT;
    c->v_ref        = AH_V_REF_DEFAULT;
    c->vz_max       = AH_VZ_MAX_DEFAULT;
    c->kp_z         = AH_KP_Z_DEFAULT;
    c->kp_vz        = AH_KP_VZ_DEFAULT;
    c->ki_vz        = AH_KI_VZ_DEFAULT;
    c->i_max        = AH_I_MAX_DEFAULT;
    c->thr_deadband = AH_THR_DEADBAND;
    c->exit_ramp_s  = AH_EXIT_RAMP_S;
}

void alt_hold_init(alt_hold_state_t *s)
{
    s->engaged      = false;
    s->z_sp         = 0.0f;
    s->integ        = 0.0f;
    s->out_us       = 1000.0f;
    s->exit_ramp    = 0.0f;
    s->exit_from_us = 1000.0f;
    s->thr_neutral  = 1500.0f;
}

static float ah_clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float alt_hold_vscale(float v_ref, float vbat_v)
{
    /* An implausible reading is REFUSED, not clamped: 1.0 changes
     * nothing, which is the only safe answer when the voltage is not
     * known. A dead INA226 reports 0 mV and v_ref/0 is infinite. */
    if (vbat_v < AH_VBAT_MIN_PLAUSIBLE || vbat_v > AH_VBAT_MAX_PLAUSIBLE ||
        v_ref <= 1.0f) {
        return 1.0f;
    }
    return ah_clamp(v_ref / vbat_v, AH_VSCALE_MIN, AH_VSCALE_MAX);
}

float thr_vcomp_us(float stick_us, float v_ref, float vbat_v)
{
    /* Scales the SPAN above 1000, never the absolute value, so a stick
     * at the bottom stays at the bottom however far the pack has
     * sagged. Scaling the absolute number would spin the motors up on a
     * low battery with the throttle closed. */
    const float out = 1000.0f + (stick_us - 1000.0f)
                              * alt_hold_vscale(v_ref, vbat_v);
    return ah_clamp(out, 1000.0f, 2000.0f);
}

float alt_hold_update(alt_hold_state_t *s, const alt_hold_cfg_t *c,
                      bool want, uint16_t stick_thr,
                      float z, float vz, float datum_shift,
                      float vbat_v, float dt)
{
    const float stick = (float)stick_thr;

    if (dt <= 0.0f) dt = 0.004f;

    /* ---- disengaged ------------------------------------------------ */
    if (!want) {
        if (s->engaged) {
            /* Hand back to the stick over a ramp instead of instantly.
             * The pilot's throttle stick is wherever they left it, and
             * on this airframe that is usually the bottom - a hard
             * handback would cut from ~1420 us to 1000 us in one cycle
             * and drop the aircraft. The ramp gives them half a second
             * to catch it - engagement is automatic, so the handback
             * must never be a surprise. */
            s->engaged      = false;
            s->exit_ramp    = 1.0f;
            s->exit_from_us = s->out_us;
        }
        if (s->exit_ramp > 0.0f) {
            s->exit_ramp -= dt / (c->exit_ramp_s > 0.0f ? c->exit_ramp_s
                                                        : AH_EXIT_RAMP_S);
            if (s->exit_ramp < 0.0f) s->exit_ramp = 0.0f;
            s->out_us = stick + (s->exit_from_us - stick) * s->exit_ramp;
        } else {
            s->out_us = stick;
        }
        s->integ = 0.0f;
        return s->out_us;
    }

    /* ---- voltage feedforward ---------------------------------------
     * Through alt_hold_vscale, which is also what the manual path uses
     * (thr_vcomp_us). ONE policy: if these two ever computed a
     * different scale the throttle would step at the engage edge, by
     * exactly the difference between them. */
    const float ff = 1000.0f + c->hover_thr * alt_hold_vscale(c->v_ref,
                                                              vbat_v);

    /* ---- engage edge: bumpless transfer ---------------------------- */
    if (!s->engaged) {
        s->engaged = true;
        s->z_sp    = z;
        s->exit_ramp = 0.0f;
        /* Wherever the stick is right now becomes zero climb rate. See
         * alt_hold_state_t.thr_neutral for why this is not 1500. */
        s->thr_neutral = stick;
        /* Seed the integrator so the FIRST output equals what the pilot
         * was already flying. Without this the aircraft jumps to the FF
         * value the instant the switch is flipped - and the FF is a
         * measurement from a different battery on a different day.
         * Clamped, so engaging at full or zero stick cannot preload the
         * integrator with a number it then has to unwind. */
        s->integ = ah_clamp(stick - ff, -c->i_max, c->i_max);
        s->out_us = stick;
    }

    /* A datum change is a coordinate change, not motion: the ToF decided
     * the floor is somewhere else. Move the setpoint with it or the loop
     * sees the jump as error and flies it. This is what stops the
     * aircraft climbing over a table.
     *
     * But only as far as a floor can move. This number arrives from the
     * estimator unchecked, and in s89  it arrived as 2387 m
     * across 16 re-datums in one flight. vert_filter.c now bounds it at
     * the source (VF_REDATUM_MAX_M) and clears `valid` instead of
     * emitting a bigger one; this is the wall behind that wall, because
     * a controller that will fly any number it is handed only needs to
     * be handed one bad number once.
     *
     * NOT clamped to the bound: a clamped bad number is still a bad
     * number, and laundering one into the legal range is exactly how a
     * fault becomes invisible. The setpoint is re-anchored to the
     * current estimate instead, which is the one answer that commands NO
     * motion whatever the datum did - the error term goes to zero, which
     * is what applying an honest shift would also have produced. The
     * altitude the pilot was holding is lost, and it deserves to be: it
     * was measured against a datum the estimator has just admitted it
     * cannot locate.
     *
     * Disengaging is not this function's call. `valid` is already false
     * by the time a shift this big can exist, and fc_tasks.c gates
     * alt_want on it, so the hold is being handed back through the exit
     * ramp on the same cycle - and a self-disengage here would only be
     * undone by the next cycle's engage edge. The event is visible in
     * the log either way: alt_sp_m is the setpoint this line writes. */
    if (AH_FABSF(datum_shift) > AH_DATUM_MAX_M) {
        s->z_sp = z;
    } else {
        s->z_sp += datum_shift;
    }

    /* ---- outer: stick -> climb-rate setpoint -----------------------
     * Referenced to the stick position captured at engage, not to 1500.
     * The span each way is whatever is left to the rails, so full
     * deflection always means vz_max regardless of where neutral sits. */
    const float off = stick - s->thr_neutral;
    const float span_up = (2000.0f - s->thr_neutral) - c->thr_deadband;
    const float span_dn = (s->thr_neutral - 1000.0f) - c->thr_deadband;
    float vz_sp;
    if (off > c->thr_deadband && span_up > 1.0f) {
        vz_sp = (off - c->thr_deadband) / span_up * c->vz_max;
        s->z_sp = z;                 /* climbing: setpoint follows us */
    } else if (off < -c->thr_deadband && span_dn > 1.0f) {
        vz_sp = (off + c->thr_deadband) / span_dn * c->vz_max;
        s->z_sp = z;                 /* descending: same */
    } else {
        /* In the detent: hold the captured altitude. The altitude P term
         * generates the climb rate needed to get back to it. */
        vz_sp = ah_clamp(c->kp_z * (s->z_sp - z), -c->vz_max, c->vz_max);
    }

    /* ---- inner: climb-rate PI + feedforward ------------------------ */
    const float err = vz_sp - vz;
    float out = ff + c->kp_vz * err + s->integ;

    /* Integrate only when it can do something. Winding up against a rail
     * is how a controller that was merely slow becomes a controller that
     * fights the pilot for a second after the rail clears. */
    const bool at_rail = (out >= AH_OUT_MAX && err > 0.0f) ||
                         (out <= AH_OUT_MIN && err < 0.0f);
    if (!at_rail) {
        s->integ = ah_clamp(s->integ + c->ki_vz * err * dt,
                            -c->i_max, c->i_max);
        out = ff + c->kp_vz * err + s->integ;
    }

    s->out_us = ah_clamp(out, AH_OUT_MIN, AH_OUT_MAX);
    return s->out_us;
}
