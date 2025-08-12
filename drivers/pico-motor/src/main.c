#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

#include "hardware/pwm.h"

#include "pico/stdlib.h"

// Local includes
#include "servo-pwm.h"
#include "crc16.h"
#include "serial.h"
#include "motor-control.h"


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
  
    // TODO figure out how to use NeoPixel on RP2040-Tiny
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    SERVO_PWM_INIT(MOTOR_PIN);
    setAngle(STOP_VAL, MOTOR_PIN);

    msg_t message;

    while (1) {

        if (read_motor_message(&message) == MSG_TIMEOUT)
            continue;

        if (parse_received_message(&message)) {
            setAngle(message.speed, MOTOR_PIN);
            send_response(true);
        } else {
            send_response(false);
        }

        tight_loop_contents();
    }
}


