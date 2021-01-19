#ifndef FIRMATA_TO_BOARD_H
#define FIRMATA_TO_BOARD_H

#include "stdint.h"
#include "gd32vf103.h"
#include "ArduinoBoard.h"
#include "gd32vf103_gpio.h"
#include "timer.h"

#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER1C 5

#define TIMER2A 7
#define TIMER2B 8

#define TIMER3A 9
#define TIMER3B 10
#define TIMER3C 11
#define TIMER4A 12
#define TIMER4B 13
#define TIMER4C 14
#define TIMER4D 15
#define TIMER5A 16
#define TIMER5B 17
#define TIMER5C 18

#define PIN_MODE_INPUT 0x00   // same as INPUT defined in Arduino.h
#define PIN_MODE_OUTPUT 0x01  // same as OUTPUT defined in Arduino.h
#define PIN_MODE_ANALOG 0x02  // analog pin in analogInput mode
#define PIN_MODE_PWM 0x03     // digital pin in PWM output mode
#define PIN_MODE_SERVO 0x04   // digital pin in Servo output mode
#define PIN_MODE_SHIFT 0x05   // shiftIn/shiftOut mode
#define PIN_MODE_I2C 0x06     // pin included in I2C setup
#define PIN_MODE_ONEWIRE 0x07 // pin configured for 1-wire
#define PIN_MODE_STEPPER 0x08 // pin configured for stepper motor
#define PIN_MODE_ENCODER 0x09 // pin configured for rotary encoders
#define PIN_MODE_SERIAL 0x0A  // pin configured for serial communication
#define PIN_MODE_PULLUP 0x0B  // enable internal pull-up resistor for pin
#define PIN_MODE_SONAR 0x0C   // pin configured for HC-SR04
#define PIN_MODE_TONE 0x0D    // pin configured for tone
#define PIN_MODE_PIXY 0x0E    // pin configured for pixy spi
#define PIN_MODE_DHT 0x0F     // pin configured for DHT
#define PIN_MODE_IGNORE 0x7F  // pin configured to be ignored by digitalWrite and capabilityResponse

#define TOTAL_PIN_MODES 17

#define NOT_A_PIN 0
#define NOT_A_PORT 0
#define PA 0
#define PB 1
#define PC 2
#define PD 3
#define PE 4
#define PF 5
#define _BV(bit) (1 << (bit))

extern const uint8_t digital_pin_to_port_PGM[];
extern const uint16_t port_to_output_PGM[];
extern const uint8_t digital_pin_to_bit_mask_PGM[];

uint8_t getPinMode(uint16_t pin);
void setPinMode(uint16_t pin, int mode);
void digitalWrite(uint8_t pin, uint8_t value);
bool digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void set_PWM(uint8_t pin, uint8_t duty, int freq);

#endif
