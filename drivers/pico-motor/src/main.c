#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

#include "hardware/pwm.h"

#include "pico/stdlib.h"

// Local includes
#include "motor_pwm.h"
#include "crc16.h"


/* -------------------------------------------------------------------------- */
/*                                   MACROS                                   */
/* -------------------------------------------------------------------------- */

#define LED_PIN 16 // GPIO connected to on-board LED (if it has one, double check)

#define MOTOR_PIN 1 // GPIO pin we are using to control the motor

/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */

int main () {
    stdio_init_all();
  
    gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    MOTOR_PWM_INIT(MOTOR_PIN);
    setAngle(STOP_VAL, MOTOR_PIN);

  while (1) {

    // printf("Stop value\n");
    // setAngle(90, MOTOR_PIN);
    // sleep_ms(1000);

    // printf("Forward value\n");
    // setAngle(180, MOTOR_PIN);
    // sleep_ms(1000);

    // printf("Reverse value\n");
    // setAngle(0, MOTOR_PIN);
    // sleep_ms(1000);

    for (int i = 0; i <= 180; i += 5) {
        setAngle(i, MOTOR_PIN);
        printf("Angle %d\n", i);
        sleep_ms(500);
    }

    tight_loop_contents();
  }
}


