#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

#include "hardware/pwm.h"

#include "pico/stdlib.h"

#define STOP_VAL 1500 // us, pulse width for neutral position of motor
#define MAX_FWR_VAL 2000 // us, assumed max forward speed pulse width
#define MAX_REV_VAL 1000 // us, assumed max reverse speed pulse width

void MOTOR_PWM_INIT(const int pin);

long map(long x, long in_min, long in_max, long out_min, long out_max);

void setAngle(int angle, const int pin);

typedef struct crc16_cfg {
  uint16_t initial;
  uint16_t polynome;
} crc16_config;

uint16_t crc16_ccitt(const uint8_t* data, size_t length, const crc16_config* cfg);

int main () {
  stdio_init_all();
  
  while (1) {
    tight_loop_contents();
  }
}

void MOTOR_PWM_INIT(const int pin) {
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
  if (angle > 180)
    angle = 180;
  if (angle < 0)
    angle = 0;

  int pulse = map(angle, 0, 180, MAX_REV_VAL, MAX_FWR_VAL);
  pwm_set_gpio_level(pin, pulse);
}

uint16_t crc16_ccitt(const uint8_t* data, size_t length, const crc16_config* cfg) {
  // TODO: Needs to be tested...

  /* uint16_t crc = 0xFFFF; */
  /* const uint16_t polynome = 0x1021; */

  uint16_t crc = cfg->initial;
  const uint16_t polynome = cfg->polynome;

  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t) (data[i] << 8);

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ polynome;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}
