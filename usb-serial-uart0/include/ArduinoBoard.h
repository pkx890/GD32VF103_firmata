#ifndef ARDUINOBOARD_H
#define ARDUINOBOARD_H



#define TOTAL_ANALOG_PINS 6
#define TOTAL_PINS 48
#define TOTAL_PORTS 3

#define TONE_TONE 0
#define TONE_NO_TONE 1

#define VERSION_BLINK_PIN 13

#define IS_PIN_ANALOG(p) ((p) >= 14 && (p) < 14 + TOTAL_ANALOG_PINS)
#define IS_PIN_I2C(p) ((p) == 18 || (p) == 19)

#define IS_PIN_PWM(p) IS_PIN_DIGITAL(p)

#define PIN_TO_DIGITAL(p) (p)
#define PIN_TO_ANALOG(p) ((p)-14)

#endif
