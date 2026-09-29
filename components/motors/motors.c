#include "motors.h"
#include "driver/ledc.h"

/* ================= PWM CONFIG ================= */

#define PWM_FREQ    400                     // ESC PWM frequency
#define PWM_RES     LEDC_TIMER_14_BIT       // ESP32-S3 max
#define PWM_MODE    LEDC_LOW_SPEED_MODE
#define PWM_TIMER   LEDC_TIMER_0

/* The ESC pulse range - the clamp in motors_set() enforces both ends. */
#define PWM_MIN_US  1000
#define PWM_MAX_US  2000

/* Index = motor number - 1, value = the GPIO its ESC signal is on.
 *   M1 Front Right CW    M2 Front Left CCW
 *   M3 Rear  Right CCW   M4 Rear  Left CW
 *
 * M3 AND M4 SWAPPED they were {9, 10, 12, 11} (M3 on GPIO12,
 * M4 on GPIO11), verified with MOT,1..4, and the author found them
 * reversed on the bench today - commanding M3 spun the rear LEFT motor.
 * The leads moved at some point between those two dates.
 *
 * RE-VERIFY AFTER ANY CHANGE HERE, PROPS OFF, BEFORE ARMING:
 *   MOT,1 -> front right     MOT,2 -> front left
 *   MOT,3 -> rear right      MOT,4 -> rear left
 * A wrong entry is not a degraded flight, it is an instant flip: the mixer
 * sends roll and yaw to a corner that is not where it thinks it is. */
static const int motor_pins[4] = {9, 10, 11, 12};

/* ================= INTERNAL ================= */

/* Convert microseconds to LEDC duty
 * 400 Hz -> 2.5 ms period -> 2500 us full scale
 */
static uint32_t us_to_duty(uint16_t us)
{
    uint32_t max_duty = (1 << 14) - 1;
    return (us * max_duty) / (1000000 / PWM_FREQ);
}

/* ================= PUBLIC API ================= */

void motors_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = motor_pins[i],
            .speed_mode = PWM_MODE,
            .channel    = i,
            .timer_sel  = PWM_TIMER,
            .duty       = us_to_duty(PWM_MIN_US),
            .hpoint     = 0
        };
        ledc_channel_config(&ch);
    }
}

/* The clamp is the LAST thing between the controller and the ESCs, and it
 * is deliberately unconditional rather than an assert or an error return:
 * there is no useful way to fail here, and a motor output is not a place
 * to find out at 250 Hz that a caller was wrong.
 *
 * Nothing reaches it today. Both live callers in fc_tasks.c flight_task
 * are already bounded - the disarmed/MOT/ESCCAL path (hard 1000/2000 and
 * a 1700 MOT cap) and the armed mixer output (clamped to [idle_us, 2000]
 * inside mixer_quad_x, then only ever scaled DOWN by m*_scale). That is the
 * point: those bounds live in the CALLERS, and every one of them is a
 * place a future edit could get it wrong. us_to_duty() is a bare
 * multiply, so an out-of-range us silently becomes an out-of-range duty
 * and LEDC writes whatever that is. One clamp here makes the invariant
 * a property of the OUTPUT STAGE instead of a property of everyone who
 * happens to call it.
 *
 * The channel guard is the same argument: motor indexes an LEDC channel,
 * so a bad index writes some other peripheral's channel config. */
void motors_set(uint8_t motor, uint16_t us)
{
    if (motor >= 4)         return;
    if (us < PWM_MIN_US)    us = PWM_MIN_US;
    else if (us > PWM_MAX_US) us = PWM_MAX_US;

    ledc_set_duty(PWM_MODE, motor, us_to_duty(us));
    ledc_update_duty(PWM_MODE, motor);
}
