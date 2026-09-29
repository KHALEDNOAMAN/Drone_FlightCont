#include "mixer.h"

/*
 * Quad X mixer with Saturation Handling (AirMode logic)
 *
 * Motor order:
 * M1 = Front Right (FR) - CW
 * M2 = Front Left  (FL) - CCW
 * M3 = Rear  Right (RR) - CCW
 * M4 = Rear  Left  (RL) - CW
 *
 * Logic:
 * 1. Calculate raw mix.
 * 2. Check for saturation (overshoot/undershoot).
 * 3. Shift ALL motors equally to fit back in range (idle_us..2000).
 * This preserves the PID/Stick ratios (flight stability)
 * at the cost of total thrust.
 */

/* Armed output floor (idle_us argument). 1000 us = ESC OFF: a stopped
 * prop gives that corner zero torque authority and restarting a
 * sensorless BLDC from 0 RPM takes 50-100+ ms (or desyncs). The mixer
 * only runs while armed (disarmed path writes 1000 directly), so its
 * floor is the idle speed, never full stop. The VALUE became the P4
 * param `min_idle_us` (default 1050, measured all four
 * motors start from a dead stop at 1024-1028 us; worst + sag margin). */
#define MAX_THROTTLE 2000

int mixer_quad_x(
    uint16_t throttle,
    int16_t roll,
    int16_t pitch,
    int16_t yaw,
    uint16_t idle_us,
    uint16_t motors[4]
)
{
    int32_t mix[4];

    // 1. Calculate Raw Mix
    //
    // Signs derived for OUR sensor convention (bench-verified):
    // IMU chip is mounted x-forward / z-UP, so the PID loops measure
    //   roll rate  = +p   (body roll, same sign)
    //   pitch rate = -q   (chip pitch: + = nose DOWN)
    //   yaw rate   = -r   (chip yaw:   + = nose LEFT)
    // Negative feedback therefore requires:
    //   +roll  -> left motors up            (tau_x > 0)
    //   +pitch -> REAR motors up            (tau_y < 0, nose down)
    //   +yaw   -> CW motors (FR, RL) up     (tau_z < 0, nose left)
    // Old table (+pitch -> front up, +yaw -> CCW up) gave positive
    // feedback on pitch/yaw: SIL diverged, hand-test confirmed mounting.

    mix[MOTOR_FR] = throttle - roll - pitch + yaw;
    mix[MOTOR_FL] = throttle + roll - pitch - yaw;
    mix[MOTOR_RR] = throttle - roll + pitch - yaw;
    mix[MOTOR_RL] = throttle + roll + pitch + yaw;

    // 2. Find Max and Min values in the current mix
    int32_t max_motor = mix[0];
    int32_t min_motor = mix[0];

    for (int i = 1; i < 4; i++) {
        if (mix[i] > max_motor) max_motor = mix[i];
        if (mix[i] < min_motor) min_motor = mix[i];
    }

    // 3. Upper Saturation Handling (Thrust Ceiling)
    // If a motor wants > 2000, lower ALL motors so the highest one hits exactly 2000.
    if (max_motor > MAX_THROTTLE) {
        int32_t overshoot = max_motor - MAX_THROTTLE;
        for (int i = 0; i < 4; i++) {
            mix[i] -= overshoot;
        }
    }

    // 4. Lower Saturation Handling (AirMode / Zero Throttle Authority)
    // If a motor wants < idle, raise ALL motors so the lowest one hits
    // exactly idle. Steering stays available at zero throttle and no
    // motor ever stops while armed.
    if (min_motor < (int32_t)idle_us) {
        int32_t undershoot = (int32_t)idle_us - min_motor;
        for (int i = 0; i < 4; i++) {
            mix[i] += undershoot;
        }
    }

    // 5. Final Safety Clamp.
    //
    // Steps 3 and 4 shift ALL FOUR motors together, so between them they
    // can leave a value out of range at the other end - ask for more
    // differential than the 1050..2000 band can hold and the shift that
    // fixes the top breaks the bottom. When that happens this clamp moves
    // ONE motor without its siblings, so the torque that comes out is not
    // the torque that was asked for.
    //
    // That, and only that, is saturation worth reporting: it is exactly
    // the condition under which an integrator would keep accumulating
    // against authority the aircraft does not have. See mixer.h.
    int saturated = 0;
    for (int i = 0; i < 4; i++) {
        if (mix[i] < (int32_t)idle_us) { mix[i] = (int32_t)idle_us; saturated = 1; }
        if (mix[i] > MAX_THROTTLE)     { mix[i] = MAX_THROTTLE;     saturated = 1; }

        motors[i] = (uint16_t)mix[i];
    }
    return saturated;
}
