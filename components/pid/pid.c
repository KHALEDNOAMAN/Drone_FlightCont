#include "pid.h"

void pid_init(pid_state_t *state) {
    state->integral = 0.0f;
    state->prev_meas = 0.0f;
    state->d_filtered = 0.0f; // Init filter
}

static unsigned long s_windup_freezes = 0;

unsigned long pid_windup_freeze_count(void) { return s_windup_freezes; }

float pid_calculate(const pid_cfg_t *cfg, pid_state_t *state, float setpoint, float measured, float dt_sec) {
    return pid_calculate_sat(cfg, state, setpoint, measured, dt_sec, 0);
}

float pid_calculate_sat(const pid_cfg_t *cfg, pid_state_t *state,
                        float setpoint, float measured, float dt_sec,
                        int saturated) {

    // 1. Calculate Error
    float error = setpoint - measured;

    // 2. Proportional Term
    float P = error * cfg->kp;

    // 3. Integral Term (raw accumulation, ki applied at the output)
    //
    // The stored integral is the RAW sum of error*dt. ki multiplies it
    // below, which is what makes a live gain change harmless: the stored
    // history means the same thing whatever ki is doing.
    //
    // ki == 0 holds the accumulator at zero rather than letting it run.
    // With ki folded in, `integral += e*dt*0` kept it at 0 for free; on a
    // raw sum it would grow unbounded while contributing nothing, then
    // appear as a large step the moment ki was raised.
    if (dt_sec > 0.0f && cfg->ki != 0.0f) {
        // ANTI-WINDUP. Freeze only in the direction that makes it worse:
        // while the mixer cannot deliver what it was asked for, growing
        // |I| adds command nothing can act on, and it is still there when
        // authority returns. Shrinking is always allowed - a blanket
        // freeze would trap the integrator at whatever it held when
        // authority ran out and the aircraft could never recover it.
        const float grows = error * state->integral;
        if (saturated && grows > 0.0f) {
            s_windup_freezes++;
        } else {
            state->integral += error * dt_sec;
        }

        // i_max bounds the OUTPUT CONTRIBUTION - the same quantity it
        // bounded before this change, so the flown behaviour is identical.
        // Expressed against the raw sum that limit is i_max/ki.
        const float lim = cfg->i_max / cfg->ki;
        const float lo = (lim < 0.0f) ? lim : -lim;   // ki may be negative
        const float hi = (lim < 0.0f) ? -lim : lim;
        if (state->integral > hi) state->integral = hi;
        else if (state->integral < lo) state->integral = lo;
    }
    float I = state->integral * cfg->ki;

    // 4. Derivative Term (Filtered D-on-Measurement)
    float D = 0.0f;
    
    if (dt_sec > 0.0f) {
        // Calculate the raw spikey derivative
        float raw_d = -(measured - state->prev_meas) / dt_sec;

        // Calculate Filter Alpha (Strength)
        // alpha = dt / (dt + tau)
        // If tau is 0, alpha is 1 (No Filtering)
        float alpha = 1.0f;
        if (cfg->tau > 0.0f) {
            alpha = dt_sec / (dt_sec + cfg->tau);
        }

        // Apply Low Pass Filter
        state->d_filtered += alpha * (raw_d - state->d_filtered);

        // Calculate final D term using the smooth filtered value
        D = state->d_filtered * cfg->kd;
    }

    // 5. Store state for next loop
    state->prev_meas = measured;

    // 6. Calculate Total Output
    float output = P + I + D;

    // 7. Clamp Output
    if (output > cfg->out_max) {
        output = cfg->out_max;
    } else if (output < -cfg->out_max) {
        output = -cfg->out_max;
    }

    return output;
}
int pid_retune_ki(pid_state_t *state, float ki_old, float ki_new, float i_max)
{
    if (ki_new == ki_old) return 0;

    /* No rescale any more. Since the integral became RAW and ki is applied
     * at the output, a gain change no longer reinterprets stored history -
     * the problem this function was written for stopped existing when the
     * accumulator did. What is still needed is the CLAMP: i_max bounds the
     * output contribution, so raising ki can leave a previously legal
     * integral producing more than i_max, and waiting for the next
     * pid_calculate to notice would let one oversized command out first. */
    if (ki_new != 0.0f) {
        const float lim = i_max / ki_new;
        const float lo = (lim < 0.0f) ? lim : -lim;
        const float hi = (lim < 0.0f) ? -lim : lim;
        if (state->integral > hi) state->integral = hi;
        else if (state->integral < lo) state->integral = lo;
    } else {
        /* ki switched off; the accumulator is held at zero while it is
         * off, matching pid_calculate_sat. */
        state->integral = 0.0f;
    }
    return 1;
}
