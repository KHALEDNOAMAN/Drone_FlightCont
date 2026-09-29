#pragma once
#include <stdint.h>

/* Motor order */
#define MOTOR_FR 0   // m1
#define MOTOR_FL 1   // m2
#define MOTOR_RR 2   // m3
#define MOTOR_RL 3   // m4

/* idle_us: armed output floor (P4 param min_idle_us). Bench-measured
 * all four motors start from a dead stop at 1024-1028 us,
 * so 1050 = worst + sag margin. Passed in rather than read from the
 * params table so the mixer stays a pure function. */
/* Returns 1 when the mixer COULD NOT deliver the differential it was asked
 * for, 0 otherwise. Feed it to pid_calculate_sat() as the anti-windup
 * signal.
 *
 * The definition is deliberately narrow. Shifting all four motors together
 * - steps 3 and 4 below - is NOT saturation: it preserves every
 * differential and spends only collective thrust, which is the whole point
 * of airmode. Authority is lost only when the FINAL clamp in step 5 has to
 * bite, because that is the one place a motor's value changes without its
 * siblings moving with it, and the commanded torque is then not what comes
 * out. Reporting the shifts as saturation would freeze the integrators
 * during ordinary hard manoeuvres, which is worse than not having the
 * feature.
 *
 * Return type changed from void; existing callers that ignore
 * it are unaffected. */
int mixer_quad_x(
    uint16_t throttle,
    int16_t roll,
    int16_t pitch,
    int16_t yaw,
    uint16_t idle_us,
    uint16_t motors[4]
);
