/* RC -> motors passthrough. A wire with firmware in it, nothing more.
 *
 * iBUS in on GPIO 7 (the existing receiver wiring, nothing replugged),
 * raw CH3 throttle out as 50 Hz PWM on all four motor pins 9/10/11/12.
 * No arming, no idle floor, no failsafe, no console, no sequencing:
 * the throttle stick IS the motor signal, the human does everything.
 *
 * Built for the classic by-hand ESC calibration on THIS airframe,
 * where the receiver is powered from the ESC BEC and the board runs
 * from ONE power source at a time - so nothing here may depend on USB
 * being present after flashing:
 *
 *   flash (USB) -> unplug USB -> TX on, stick MAX -> plug pack ->
 *   entry beeps -> stick MIN -> confirm beeps -> done.
 *
 * After calibration the stick directly drives all four motors (PROPS
 * OFF) so each one can be spun up and down by hand to check they now
 * respond identically. The learned range is in the ESC's own EEPROM -
 * one-time. Then unplug and reflash the flight firmware from the repo
 * root; identical partition table, NVS/mag cal untouched either way.
 *
 * The motor pins stay silent until the first valid iBUS frame -
 * exactly what an ESC sees from a directly connected receiver that is
 * still booting - and from then on they mirror the stick frame by
 * frame. 50 Hz on purpose: every past successful calibration of these
 * ESCs came from a receiver's 50 Hz servo output, and cal entry on
 * cheap ESCs can be picky about frame rate even though 400 Hz flies
 * fine.
 */
#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"
#include "ibus.h"

#define IBUS_GPIO   7
#define CH_THROTTLE 2                  /* CH3 - same as main/config.h */
#define PWM_FREQ_HZ 50                 /* what a receiver PWM pin outputs */
#define PWM_RES     LEDC_TIMER_14_BIT
#define PWM_MODE    LEDC_LOW_SPEED_MODE

/* Same pins, same order as components/motors/motors.c: M1 FR .. M4 RL */
static const int motor_pins[4] = {9, 10, 11, 12};

static uint32_t us_to_duty(uint32_t us)
{
    /* 50 Hz -> 20000 us period across 14 bits */
    return us * ((1 << 14) - 1) / (1000000 / PWM_FREQ_HZ);
}

void app_main(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = PWM_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ibus_init(IBUS_GPIO);

    uint16_t ch[IBUS_CHANNELS];
    bool failsafe;
    bool driving = false;

    while (1) {
        if (!ibus_read(ch, &failsafe)) continue;   /* blocks on UART events */

        if (!driving) {
            /* First valid frame: only now start driving the pins, so a
             * booting ESC sees silence (like a still-binding receiver),
             * never a value the firmware made up. */
            for (int i = 0; i < 4; i++) {
                ledc_channel_config_t c = {
                    .gpio_num   = motor_pins[i],
                    .speed_mode = PWM_MODE,
                    .channel    = i,
                    .timer_sel  = LEDC_TIMER_0,
                    .duty       = us_to_duty(ch[CH_THROTTLE]),
                    .hpoint     = 0,
                };
                ledc_channel_config(&c);
            }
            driving = true;
            continue;
        }

        for (int i = 0; i < 4; i++) {
            ledc_set_duty(PWM_MODE, i, us_to_duty(ch[CH_THROTTLE]));
            ledc_update_duty(PWM_MODE, i);
        }
    }
}
