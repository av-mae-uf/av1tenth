#include "servo-pwm.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

/* -------------------------------------------------------------------------- */
/*                                  Functions                                 */
/* -------------------------------------------------------------------------- */


void SERVO_PWM_INIT(const int pin) {
    pwm_config cfg = pwm_get_default_config();

    // The ESC on the car needs a PWM signal. Most servos and ESCs
    // run at 50 Hz (20 ms) and need a pulse of around 1.0 - 2.0 ms.

    // The formula for pwm frequency is f = clock_freq / (divider * (TOP + 1))
    // divider is the clock divider. TOP is the count value before overflow.

    // The pico has a clock of about 125 MHz and if we use a divider of 125,
    // we can set top to be 19999 for a resolution of 1us so that the PWM values
    // we need are 1000 (1 ms) and 2000 (2 ms) at 50 Hz.
    pwm_config_set_clkdiv(&cfg, 125.0f);
    pwm_config_set_wrap(&cfg, 19999);

    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint8_t slice = pwm_gpio_to_slice_num(pin);
    pwm_init(slice, &cfg, true);
    pwm_set_enabled(slice, true);
}


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setAngle(int angle, const int pin) {
    if (angle > MAX_FWR_DEG)
        angle = MAX_FWR_DEG;
    if (angle < MAX_REV_DEG)
        angle = MAX_REV_DEG;

    int pulse = map(angle, MAX_REV_DEG, MAX_FWR_DEG, ARDUINO_LOW, ARDUINO_HIGH);
    pwm_set_gpio_level(pin, pulse);
}
