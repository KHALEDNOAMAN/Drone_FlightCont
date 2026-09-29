#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

// Configuration Structure (The Tuning Knobs)
typedef struct {
    float kp;           // Proportional Gain
    float ki;           // Integral Gain
    float kd;           // Derivative Gain
    float i_max;        // Integral Limit (Anti-Windup)
    float out_max;      // Output Limit (e.g., Motor Max PWM)
    
    // NEW: Derivative Filter Time Constant (Seconds)
    // Typical value: 0.01s to 0.05s
    // Cutoff Freq (Hz) = 1 / (2 * PI * tau)
    // tau = 0.015 (~10Hz) is a good starting point.
    float tau;          
} pid_cfg_t;

// State Structure (The Memory)
typedef struct {
    /* RAW accumulated error, in error-units * seconds. ki is applied at
     * the OUTPUT, not folded in here.
     *
     * It used to hold `ki * sum(e*dt)`, which had two consequences. A live
     * ki change silently reinterpreted the stored history (worked around
     * by pid_retune_ki), and the i_max clamp was applied to a quantity
     * whose meaning moved with the gain. Storing the raw sum makes the
     * state independent of tuning; i_max still bounds the OUTPUT
     * contribution, so the flown behaviour is unchanged. */
    float integral;     // Accumulated Error (I-Term), RAW
    float prev_meas;    // Last Measurement (For D-on-Meas)
    
    // NEW: The filtered derivative value from the previous loop
    float d_filtered;   
} pid_state_t;

/**
 * @brief Initialize or Reset a PID controller state
 */
void pid_init(pid_state_t *state);

/**
 * @brief Calculate the PID output
 */
float pid_calculate(const pid_cfg_t *cfg, pid_state_t *state, float setpoint, float measured, float dt_sec);

/**
 * @brief PID with anti-windup, given the mixer's saturation state.
 *
 * `saturated` is 1 when the mixer could NOT deliver the torque it was
 * asked for - see mixer_quad_x(), which reports whether its FINAL clamp
 * had to bite. Note that a mixer shifting all four motors together is not
 * saturation: that preserves the differential and only spends collective.
 * Authority is lost only when the clamp itself bites.
 *
 * While saturated, the integrator is frozen ONLY IN THE DIRECTION THAT
 * WOULD MAKE IT WORSE - `error * integral > 0`. It is always allowed to
 * shrink. A blanket freeze would trap the integrator at whatever value it
 * held when authority ran out, and the aircraft would never recover it.
 *
 * Without this the integrator keeps accumulating against a motor that is
 * already railed, so when the aircraft finally has authority again it
 * carries a large command nothing asked for - the classic windup
 * overshoot, and it is worst in exactly the situation that caused it.
 *
 * pid_calculate() is this with saturated = 0.
 */
float pid_calculate_sat(const pid_cfg_t *cfg, pid_state_t *state,
                        float setpoint, float measured, float dt_sec,
                        int saturated);

/**
 * @brief Cycles in which the integrator was frozen by saturation.
 *
 * A diagnostic, and deliberately global rather than per-controller: the
 * question it answers is "is this feature doing anything at all on this
 * airframe", which STATS asks once. Zero means the mixer never ran out of
 * authority, in which case anti-windup is costing nothing and proving
 * nothing.
 */
unsigned long pid_windup_freeze_count(void);

/**
 * @brief Carry a live ki change into the integrator.
 *
 * Previously, pid_calculate accumulated `integral += error*dt*ki`, so
 * ki was baked into the stored value and a live ki change needed a
 * rescale. SINCE THE INTEGRAL IS RAW, and this is vestigial for
 * correctness: with
 * ki applied at the output, changing ki no longer reinterprets stored
 * history, so nothing needs rescaling. It is kept because the i_max clamp
 * is expressed on the OUTPUT contribution, so a ki increase can leave a
 * previously legal integral producing more than i_max; this re-clamps it
 * on the spot rather than waiting for the next pid_calculate to notice.
 * The rescale itself is now a no-op by construction and the function
 * returns whether the gain moved.
 *
 * @return 1 if the gain changed and the state was touched, 0 otherwise.
 */
int pid_retune_ki(pid_state_t *state, float ki_old, float ki_new, float i_max);

#endif // PID_H