#ifndef SERVO_PWM_H
#define SERVO_PWM_H

/* -------------------------------------------------------------------------- */
/*                                   MACROS                                   */
/* -------------------------------------------------------------------------- */

#define STOP_VAL 1500 // us, pulse width for neutral position of motor
#define MAX_FWR_VAL 2000 // us, assumed max forward speed pulse width
#define MAX_REV_VAL 1000 // us, assumed max reverse speed pulse 

// To get similar values as the Arduino, we need a different range
#define ARDUINO_LOW 595
#define ARDUINO_HIGH 2425

#define STOP_DEG 90
#define MAX_FWR_DEG 180
#define MAX_REV_DEG 0

/* -------------------------------------------------------------------------- */
/*                                 PROTOTYPES                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initializes PWM for the input pin to be 50 Hz with 1us resolution
 * 
 * @param pin (const int) Pin number (typically 1-30, check with what your 2040 board allows)
 */
void SERVO_PWM_INIT(const int pin);

/**
 * @brief Arduino's map function, "Re-maps a number from one range to another. That is, a value of in_min would get mapped to out_min, a value of in_max to out_max, values in-between to values in-between, etc."
 * 
 * @param x: (long) number to map
 * @param in_min: (long) lower bound of x's current range
 * @param in_max: (long) upper bound of x's current range 
 * @param out_min: (long) lower bound of new range
 * @param out_max: (long) upper bound of new range
 * @return `[long]`: a long int number inbetween out_min (inclusive) and out_max (inclusive)
 */
long map(long x, long in_min, long in_max, long out_min, long out_max);

/**
 * @brief Changes the pulse width of the passed pin based on the angle passed [0-180]
 * 
 * @param angle: (int) Angle, in degrees, you want to change the pulse to [0-180] | 90 is STOP_VAL | 180 is MAX_FWR_VAL | 0 is MAX_REV_VAL
 * @param pin: (const int) Pin number (typically 1-30, check with what your 2040 board allows)
 */
void setAngle(int angle, const int pin);

#endif // SERVO_PWM_H