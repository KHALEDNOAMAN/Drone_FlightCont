#ifndef ALT_HOLD_H
#define ALT_HOLD_H

/* P7 stage 2 - altitude-hold controller.
 *
 * Same rules as vert_filter.c: no ESP-IDF, no FreeRTOS, no math.h, no
 * globals, so it survives the STM32 core split and can be run on the
 * host by tools/host_c.py (compiles this file) and exercised closed-loop
 * by tools/alt_hold_sim.py.
 *
 * SHAPE OF THE LOOP
 *   throttle stick -> climb-rate setpoint   (centre detent = hold)
 *   altitude error -> climb-rate setpoint   (only while holding)
 *   climb-rate error -> PI -> throttle correction
 *   plus a hover-throttle FEEDFORWARD scaled by pack voltage
 *
 * The feedforward is what makes this work at all. A PI loop asked to
 * find 420 us of hover throttle from zero would either take seconds or
 * need gains that scream. Instead the FF supplies the known hover
 * throttle and the PI only trims the difference - so the integrator
 * stays small, and small integrators recover from saturation.
 *
 * WHY THE VOLTAGE TERM: a 4S pack sags from 16.8 V to about 14 V under
 * load. Motor RPM at a fixed duty follows voltage, so the SAME throttle
 * produces less thrust as the flight goes on - which is exactly the
 * "sink" seen in flight 4. Scaling the hover FF by v_ref/v_now feeds
 * that forward instead of waiting for the integrator to discover it.
 *
 * The LINEAR form is not an approximation - it is the exact answer for
 * this plant, and it is worth writing the two lines down because the
 * "thrust goes as RPM squared, so surely the correction is squared too"
 * reflex gets it wrong:
 *
 *     thrust  ~ RPM^2,   RPM ~ duty * V,   duty ~ span
 *     =>  thrust ~ (span * V)^2
 *     =>  constant thrust needs span * V = const
 *     =>  span_new = span_ref * (V_ref / V_now)          <- linear
 *
 * The square is already spent turning voltage into RPM; correcting it
 * again would double-count. Log B still has to CONFIRM this on the real
 * airframe (hover at a fixed altitude and watch the integrator against
 * vbat - if the linear term is right, the integrator stays flat while
 * the pack sags), because a real ESC's duty-to-RPM curve is only
 * approximately linear.
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* --- gains and limits (from params, refreshed every cycle) --- */
    float hover_thr;    /* us ABOVE 1000 that hovers at v_ref            */
    float v_ref;        /* pack voltage hover_thr was measured at, V     */
    float vz_max;       /* climb rate at full stick, m/s                 */
    float kp_z;         /* altitude error -> climb-rate setpoint, 1/s    */
    float kp_vz;        /* climb-rate error -> us                        */
    float ki_vz;        /* climb-rate integral -> us/s                   */
    float i_max;        /* integrator clamp, us                          */
    float thr_deadband; /* stick counts around centre that mean "hold"   */
    float exit_ramp_s;  /* manual handback ramp, s                       */
} alt_hold_cfg_t;

typedef struct {
    bool  engaged;
    float z_sp;         /* altitude setpoint, filter datum, m            */
    float integ;        /* PI integrator, us                             */
    float out_us;       /* last throttle command, us (1000-2000)         */
    float exit_ramp;    /* 1 -> 0 while handing back to the stick        */
    float exit_from_us; /* output at the moment of disengage             */
    /* Throttle stick position captured at the engage edge, and the
     * zero-climb-rate reference from then on.
     *
     * NOT stick centre (1500), which is what most flight controllers
     * use and what the first version of this did. Two reasons, and the
     * sim found the first one as an 89 us lurch:
     *
     *  - the hover throttle on this airframe is ~1420 us, so a
     *    centre-referenced stick reads 80 counts LOW the instant
     *    alt-hold engages and immediately commands a descent. On a
     *    lighter setup hovering at 1300 that is a 0.3 m/s descent
     *    starting the moment the pilot flips the switch.
     *  - this TX's throttle gimbal is RATCHETED and does not spring
     *    back to centre. A centre-referenced climb-rate stick would
     *    therefore be commanding something almost all the time, and
     *    "let go and it holds" - the property you actually want the
     *    first time you test alt-hold - would be false.
     *
     * Capturing the stick means the engage edge itself does nothing, and
     * pushing UP from wherever the stick already is climbs. The cost is
     * that engaging with the stick already high leaves less climb
     * authority, which is the safe direction to lose it in. */
    float thr_neutral;
} alt_hold_state_t;

void alt_hold_init(alt_hold_state_t *s);

/* Defaults are the MEASURED hover from flight4 s19-s22 plus placeholder
 * gains - see alt_hold.c. Call once, then override from params. */
void alt_hold_default_cfg(alt_hold_cfg_t *c);

/* One control cycle.
 *
 *   want       the AUTOMATIC policy's verdict (alt_hold_auto on,
 *              estimate valid, airborne, throttle up - fc_tasks.c). CH6
 *              engages nothing.
 *   stick_thr  raw RC throttle, us (1000-2000)
 *   z, vz      from the vertical filter
 *   datum_shift  vert_filter_take_datum_shift() - metres the DATUM
 *              moved this cycle. Added straight to the setpoint so a
 *              re-datum never commands motion. Pass 0 if unused.
 *   vbat_v     pack voltage, V. Pass 0 to disable the FF scaling (a
 *              dead INA226 must not scale the hover throttle to zero).
 *
 * Returns the throttle in us to hand the mixer. When not engaged this
 * returns the stick (or a ramp from the held value back to it), so the
 * caller can use the result unconditionally. */
float alt_hold_update(alt_hold_state_t *s, const alt_hold_cfg_t *c,
                      bool want, uint16_t stick_thr,
                      float z, float vz, float datum_shift,
                      float vbat_v, float dt);

/* The throttle scale that holds thrust constant as the pack sags,
 * v_ref/vbat_v, clamped to [0.8, 1.3] and REFUSED (1.0) outside the
 * 9-26 V plausibility window or with a nonsense v_ref.
 *
 * Public because the manual path needs the SAME number the hover
 * feedforward above uses. Two copies of this policy would step the
 * throttle at the engage edge by the difference between them. */
float alt_hold_vscale(float v_ref, float vbat_v);

/* BATTERY-SAG THROTTLE COMPENSATION for the manual path.
 *
 *     us_out = 1000 + (us_in - 1000) * alt_hold_vscale(v_ref, vbat_v)
 *
 * Result clamped to [1000, 2000]. Above about 15.5 V full stick clamps
 * at 2000 and that is not lost authority: the ESC cannot exceed 100 %
 * duty, and max thrust genuinely IS lower on a sagged pack.
 *
 * WHY LINEAR, AND WHY THERE IS NO EXPONENT TO TUNE. Thrust follows the
 * ELECTRICAL POWER the motor gets, not the bus voltage. From this
 * airframe's own datasheet (SunnySky X2212 980 kV + APC9045), at equal
 * thrust the power differs by only 3-4 % between 11.1 V and 14.8 V, and
 * T = a*P^b fits with b = 0.782 and 0.755 at those two voltages
 * (r2 = 0.9996 on both). With a PWM ESC the motor sees duty * V_bus, so
 * holding duty * V constant holds thrust constant whatever b is. The
 * law is parameter-free, which is the point - there is nothing here to
 * measure wrong.
 *
 * DO NOT TRY TO MEASURE THE SLOPE FROM A FLIGHT LOG. Regressing
 * collective motor command against pack voltage gives -235 us/V on s207
 * and up to -700 us/V elsewhere against a physics value near -29,
 * because voltage falls with TIME and every slow drift in the flight
 * gets billed to it. That fit reaches r2 = 0.94 while being entirely
 * confounded. Validate this on ALTITUDE, not on motor command.
 *
 * Applied to the stick BEFORE alt_hold_update, never to its output:
 * this function scales a stick position, and alt_hold's feedforward
 * already carries its own scaling. Compensating the output would apply
 * the scale twice to the feedforward and step the throttle whenever the
 * hold engaged or dropped. */
float thr_vcomp_us(float stick_us, float v_ref, float vbat_v);

#endif /* ALT_HOLD_H */
