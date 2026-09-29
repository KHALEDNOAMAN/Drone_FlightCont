#ifndef VEL_HOLD_H
#define VEL_HOLD_H

/* Velocity hold - sticks centred means ZERO GROUND SPEED.
 *
 * The rung below position hold on this project's own ladder, and it is
 * there for a reason: one integrator fewer, and far less able to run
 * away. A position hold that is wrong flies off toward a wrong
 * coordinate; a velocity hold that is wrong holds a wrong speed, which
 * the pilot can see and out-fly.
 *
 * Same rules as vert_filter.c and horiz_filter.c: no ESP-IDF, no
 * FreeRTOS, no math.h, no globals, so it runs on the host under a test.
 *
 * WHAT IT PRODUCES. Tilt, in degrees, to be used INSTEAD of the pilot's
 * stick-derived des_roll/des_pitch while engaged (the VELOCITY HOLD block
 * in fc_tasks.c flight_task). It
 * does not touch throttle, yaw, or the rate loops - a quad accelerates
 * horizontally by tilting, so a horizontal controller's only output is
 * an attitude command.
 *
 * WHAT IT NEEDS. Earth-frame NED ground velocity from horiz_filter, and
 * the current heading. Nothing else: no position, no home point, no GPS
 * fix quality. That independence is the point - velocity hold keeps
 * working through the multipath excursions that make position useless
 * (MEASURED on s161: position scattered 9.3 m while parked, velocity
 * stayed inside 0.11 m/s).
 *
 * ---------------------------------------------------------------------
 * SIGNS ARE DERIVED, NOT YET FLIGHT-VERIFIED. READ THIS BEFORE FLYING.
 * ---------------------------------------------------------------------
 * The two constants VEL_HOLD_PITCH_SIGN and VEL_HOLD_ROLL_SIGN encode
 * which way to tilt to accelerate which way. They are derived from the
 * conventions this project has already established and measured:
 *
 *   - body frame is x FORWARD, y LEFT, z UP (lsm6dsox.h IMU_REMAP)
 *   - +pitch is nose DOWN (project convention, recorded)
 *   - therefore +pitch accelerates FORWARD  -> pitch sign +1
 *   - +roll about +x (forward) carries +y (left) toward +z (up), so the
 *     LEFT side rises and the aircraft accelerates RIGHT
 *   - therefore accelerating LEFT needs NEGATIVE roll -> roll sign -1
 *
 * That chain is sound but it is reasoning, and this project's own
 * history says reasoning about signs loses to measurement: the yaw stick
 * mapping was backwards for weeks, and a "roll trim" fix once made
 * things worse. They are isolated into two constants precisely so a
 * disagreement is a one-character fix rather than a rewrite.
 *
 * VERIFY BEFORE PROPS, then again in the air:
 *   1. Offline, against a walk log: tools/vel_hold_test.py replays real
 *      GPS velocity and checks the commanded tilt OPPOSES the measured
 *      motion. A sign error shows up as a command that would accelerate
 *      INTO the drift, which is the failure that flies away.
 *   2. On the bench, armed, PROPS OFF: push the aircraft forward by hand
 *      and watch des_pitch go NEGATIVE (nose up, braking). Push it left
 *      and watch des_roll go POSITIVE (right side down, braking).
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* Tilt commanded per m/s of velocity error, deg/(m/s). The whole
     * loop gain: too low and it drifts, too high and it oscillates in a
     * pilot-induced way that looks like a tuning problem in the rate
     * loop. Start low. */
    float kp;
    /* Integral, deg/(m/s)/s. Trims out a steady disturbance - wind, or
     * the CoG offset this airframe has a documented history of. */
    float ki;
    /* Bound on the integral's CONTRIBUTION in degrees, not on the raw
     * sum. Same convention as pid.c after the anti-windup
     * change, so the two read alike. */
    float i_max;
    /* Hard limit on commanded tilt, degrees. This is a safety bound, not
     * a tuning knob: it caps how hard the loop may ever pitch the
     * aircraft over when its velocity estimate is wrong. */
    float tilt_max;
    /* Full stick asks for this ground speed, m/s. */
    float vel_max;
} vel_hold_cfg_t;

typedef struct {
    bool  engaged;
    float integ_fwd;    /* raw integral, m/s*s - ki applied at the output */
    float integ_left;
    float out_roll;     /* last commanded tilt, degrees */
    float out_pitch;
} vel_hold_state_t;

void vel_hold_init(vel_hold_state_t *s);
void vel_hold_default_cfg(vel_hold_cfg_t *c);

/* One control step.
 *
 * want      the AUTOMATIC policy's verdict (vel_hold_auto on,
 *           horiz_valid, airborne latch, throttle up - fc_tasks.c)
 * stick_r   roll stick, -500..+500, +ve = right
 * stick_p   pitch stick, -500..+500, +ve = forward
 * vel_n/e   EARTH-frame NED ground velocity, m/s, from horiz_filter
 * cos_yaw,  cosine and sine of the heading (degrees clockwise from
 * sin_yaw   north). Passed IN rather than computed here, for the same
 *           reason vert_filter takes tilt_cos: __builtin_sinf and
 *           __builtin_cosf are NOT single instructions - they fall back
 *           to libm calls, which breaks the freestanding link the host
 *           test depends on and pulls libm into the flight image for two
 *           numbers the caller can produce more cheaply. fusion.c
 *           already owns the quaternion these come from.
 * vel_ok    the velocity estimate is trustworthy RIGHT NOW. False
 *           disengages: the same rule the ceiling crash taught, that a
 *           controller must never close a loop around an estimate whose
 *           source has stopped answering.
 * dt        seconds
 *
 * Writes the commanded tilt to *des_roll_deg / *des_pitch_deg and
 * returns true when it is driving. Returns false and leaves them
 * untouched when disengaged, so the caller keeps the pilot's sticks. */
bool vel_hold_update(vel_hold_state_t *s, const vel_hold_cfg_t *c,
                     bool want, int16_t stick_r, int16_t stick_p,
                     float vel_n, float vel_e,
                     float cos_yaw, float sin_yaw,
                     bool vel_ok, float dt,
                     float *des_roll_deg, float *des_pitch_deg);

#endif /* VEL_HOLD_H */
