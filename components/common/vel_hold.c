/* Velocity hold. See vel_hold.h for the API, the sign derivation, and
 * the verification steps that must happen before this ever drives an
 * aircraft.
 *
 * NO TRIG IN THIS FILE, and that is not stylistic. __builtin_sinf and
 * __builtin_cosf are not single instructions the way __builtin_sqrtf is
 * - they lower to libm calls, which fails a freestanding link and drags
 * libm into the flight image. The caller passes cos and sin of the
 * heading instead, exactly as vert_filter takes tilt_cos rather than an
 * angle. fusion.c already holds the quaternion they come from. */

#include "vel_hold.h"

/* See the long note in vel_hold.h. DERIVED from this project's
 * established frames, NOT yet flight-verified, and isolated here so a
 * disagreement costs one character.
 *
 *   +pitch is nose DOWN, so +pitch accelerates FORWARD.
 *   +roll lifts the LEFT side (body y is left, z is up), so +roll
 *   accelerates RIGHT - hence accelerating left needs negative roll. */
#define VEL_HOLD_PITCH_SIGN  (+1.0f)
#define VEL_HOLD_ROLL_SIGN   (-1.0f)

/* Defaults are DELIBERATELY TIMID. This loop has never flown, and the
 * failure mode of too much gain here is a pilot-induced oscillation that
 * reads like a rate-loop tuning problem and sends you debugging the
 * wrong thing.
 *
 * kp 4 deg per m/s: a 1 m/s drift asks for 4 degrees of tilt, which on
 * this airframe is a gentle correction that takes a few seconds. Raise it
 * once the loop is proven to settle rather than hunt.
 *
 * tilt_max 12 deg is a SAFETY bound, not a tuning knob: it is the most
 * this loop may ever tip the aircraft when its velocity estimate is
 * wrong. With a GPS velocity noise of 0.11 m/s (MEASURED) the noise
 * alone asks for well under a degree, so 12 leaves room for real
 * corrections while bounding a runaway. */
void vel_hold_default_cfg(vel_hold_cfg_t *c)
{
    c->kp = 4.0f;
    c->ki = 1.0f;
    c->i_max = 5.0f;
    c->tilt_max = 12.0f;
    c->vel_max = 3.0f;
}

void vel_hold_init(vel_hold_state_t *s)
{
    s->engaged = false;
    s->integ_fwd = 0.0f;
    s->integ_left = 0.0f;
    s->out_roll = 0.0f;
    s->out_pitch = 0.0f;
}

bool vel_hold_update(vel_hold_state_t *s, const vel_hold_cfg_t *c,
                     bool want, int16_t stick_r, int16_t stick_p,
                     float vel_n, float vel_e,
                     float cos_yaw, float sin_yaw,
                     bool vel_ok, float dt,
                     float *des_roll_deg, float *des_pitch_deg)
{
    /* Disengage on the pilot's word OR on a velocity estimate that has
     * stopped being trustworthy. The second half is the lesson from the
     * ceiling crash written as code: alt-hold engaged on an
     * estimator whose ToF had died, because `valid` was being set by a
     * sensor that could not see the ground. A controller must never
     * close a loop around an estimate whose source has stopped
     * answering, and it must re-check that EVERY cycle, not once at
     * engage. */
    if (!want || !vel_ok || dt <= 0.0f) {
        if (s->engaged) {
            /* Drop the integrators on the way out. Carrying them across
             * a disengage means the next engage starts with a trim
             * learned under conditions that no longer apply. */
            s->integ_fwd = 0.0f;
            s->integ_left = 0.0f;
            s->engaged = false;
        }
        return false;
    }

    if (!s->engaged) {
        s->integ_fwd = 0.0f;
        s->integ_left = 0.0f;
        s->engaged = true;
    }

    /* Pilot demand, in the BODY frame: pitch stick forward asks to move
     * forward, roll stick right asks to move right. Sticks centred asks
     * for zero, which is the whole feature. */
    const float cmd_fwd = (float)stick_p * c->vel_max / 500.0f;
    const float cmd_left = -(float)stick_r * c->vel_max / 500.0f;

    /* Rotate the measured EARTH-frame velocity into the body frame.
     *
     * yaw is degrees clockwise from north, so the forward unit vector in
     * (north, east) is (cos y, sin y). LEFT is that turned 90 degrees
     * anticlockwise, i.e. bearing y-90, giving (sin y, -cos y) - which
     * is the sanity check worth keeping: facing north (y = 0), left is
     * (0, -1) = west. It is. */
    const float v_fwd = vel_n * cos_yaw + vel_e * sin_yaw;
    const float v_left = vel_n * sin_yaw - vel_e * cos_yaw;

    const float err_fwd = cmd_fwd - v_fwd;
    const float err_left = cmd_left - v_left;

    /* Raw integrals; ki is applied at the OUTPUT and i_max bounds that
     * contribution in degrees. Same convention as pid.c since the
     * anti-windup change, so the two read alike. */
    s->integ_fwd += err_fwd * dt;
    s->integ_left += err_left * dt;

    float i_fwd = c->ki * s->integ_fwd;
    float i_left = c->ki * s->integ_left;
    if (i_fwd > c->i_max) { i_fwd = c->i_max; s->integ_fwd = c->i_max / c->ki; }
    if (i_fwd < -c->i_max) { i_fwd = -c->i_max; s->integ_fwd = -c->i_max / c->ki; }
    if (i_left > c->i_max) { i_left = c->i_max; s->integ_left = c->i_max / c->ki; }
    if (i_left < -c->i_max) { i_left = -c->i_max; s->integ_left = -c->i_max / c->ki; }

    float pitch = VEL_HOLD_PITCH_SIGN * (c->kp * err_fwd + i_fwd);
    float roll = VEL_HOLD_ROLL_SIGN * (c->kp * err_left + i_left);

    /* Clamp tilt, and freeze the integrator that is pushing INTO the
     * clamp while leaving the other free. Clamping the output while the
     * integral keeps growing is the windup that put 2387 m of setpoint
     * into a single flight through the altitude path. */
    if (pitch > c->tilt_max) {
        pitch = c->tilt_max;
        s->integ_fwd -= err_fwd * dt;
    } else if (pitch < -c->tilt_max) {
        pitch = -c->tilt_max;
        s->integ_fwd -= err_fwd * dt;
    }
    if (roll > c->tilt_max) {
        roll = c->tilt_max;
        s->integ_left -= err_left * dt;
    } else if (roll < -c->tilt_max) {
        roll = -c->tilt_max;
        s->integ_left -= err_left * dt;
    }

    s->out_pitch = pitch;
    s->out_roll = roll;
    *des_pitch_deg = pitch;
    *des_roll_deg = roll;
    return true;
}
