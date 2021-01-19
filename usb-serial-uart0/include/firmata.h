#ifndef FIRMATA_H
#define FIRMATA_H

#include "stdint.h"
#include "gd32vf103.h"
#include "stddef.h"
#include "firmata_to_board.h"
#include "twi.h"
#include "ArduinoBoard.h"
#include "cdc_acm_core.h"
#include "usart0.h"
#include "timer.h"

#include "dht.h"
#include "stdlib.h"


#define MINIMUM_SAMPLING_INTERVAL 1

#define INTER_PING_INTERVAL 40 // 40 ms.

#define FIRMATA_MAJOR 0x02
#define FIRMATA_MINOR 0x06

#define DIGITAL_MESSAGE 0x90
#define ANALOG_MESSAGE 0xE0
#define REPORT_ANALOG 0xC0
#define REPORT_DIGITAL 0xD0

#define SET_PIN_MODE 0xF4
#define SET_DIGITAL_PIN_VALUE 0xF5

#define REPORT_VERSION 0xF9
#define SYSTEM_RESET 0xFF

#define START_SYSEX 0xF0
#define END_SYSEX 0xF7
#define SYSEX_I2C_REPLY 0x77 // same as I2C_REPLY

#define KEEP_ALIVE 0x50   //keep alive message
#define RU_THERE 0x51     // Poll Request For Boards Presence
#define I_AM_HERE 0x52    // Response to RU_THERE
#define TONE_DATA 0x5F    // request to play a tone
#define SERIAL_DATA 0x60  // communicate with serial devices, including other boards
#define ENCODER_DATA 0x61 // reply with encoders current positions
#define SONAR_CONFIG 0x62 // sonar configuration request
#define SONAR_DATA 0x63   // sonar data reply
#define DHT_CONFIG 0x64
#define DHT_DATA 0x65
#define SERVO_CONFIG 0x70            // set max angle, minPulse, maxPulse, freq
#define STRING_DATA 0x71             // a string message with 14-bits per char
#define STEPPER_DATA 0x72            // control a stepper motor
#define ONEWIRE_DATA 0x73            // send an OneWire read/write/reset/select/skip/search request
#define SHIFT_DATA 0x75              // a bitstream to/from a shift register
#define I2C_REQUEST 0x76             // send an I2C read/write request

#define SPI_REQUEST 0x46

#define I2C_REPLY 0x77               // a reply to an I2C read request
#define I2C_CONFIG 0x78              // config I2C settings such as delay times and power pins
#define SPI_CONFIG 0x48

#define REPORT_FIRMWARE 0x79         // report name and version of the firmware
#define EXTENDED_ANALOG 0x6F         // analog write (PWM, Servo, etc) to any pin
#define PIN_STATE_QUERY 0x6D         // ask for a pin's current mode and value
#define PIN_STATE_RESPONSE 0x6E      // reply with pin's current mode and value
#define CAPABILITY_QUERY 0x6B        // ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE 0x6C     // reply with supported modes and resolution
#define ANALOG_MAPPING_QUERY 0x69    // ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE 0x6A // reply with mapping info
#define SAMPLING_INTERVAL 0x7A       // set the poll rate of the main loop
#define SCHEDULER_DATA 0x7B          // send a createtask/deletetask/addtotask/schedule/querytasks/querytask request to the scheduler
#define SYSEX_NON_REALTIME 0x7E      // MIDI Reserved for non-realtime messages
#define SYSEX_REALTIME 0x7F          // MIDI Reserved for realtime messages
//DFRobot˽��Э��
#define DFROBOT_MESSAGE 0x0D
#define SUB_MESSAGE_DFROBOT_REPORTS 0x0A

#define SUB_MESSAGE_IR 0x00
#define SUB_MESSAGE_SONAR 0x01
#define SUB_MESSAGE_TONE 0x02
#define SUB_MESSAGE_PULSE 0x03
#define SUB_MESSAGE_MILLIS 0x04
#define SUB_MESSAGE_DHT 0x0D
#define SUB_MESSAGE_I2CSCAN 0x0E
#define SUB_MESSAGE_DS18B20 0x11
#define SUB_MESSAGE_VIBRATION 0x15
#define SUB_MESSAGE_LED 0x17

//�ƴ�APIЭ��
#define LED_begin 0x00
#define LED_setRangeColor 0x01
#define LED_setBrightness 0x02
#define LED_showRainbow 0x03
#define LED_shift 0x04
#define LED_rotate 0x05
#define LED_showBarGraph 0x06
#define LED_clear 0x07

#define SUB_MESSAGE_RESET 0x7f

#define MAX_DATA_BYTES 64

#define ARDUINO_INSTANCE_ID 1

#define TOTAL_ANALOG_PINS 6
#define TOTAL_PINS 48
#define TOTAL_PORTS 3

#define I2C0_OWN_ADDRESS7 0x72
#define I2C0_SLAVE_ADDRESS7 0x82

// static uint8_t nvicflag2 = 0;
// static uint8_t nvicflag3 = 0;

// extern volatile unsigned long timer0_overflow_count;
// extern unsigned int samplingInterval;

void firmata_init(void);
void uart_write(uint8_t data[],uint8_t len);
void firmata_checkDigitalInputs();
// uint8_t getPinMode(uint8_t pin);
void firmata_sendAnalog(uint8_t pin, uint16_t value);
void readAndReportData(uint8_t address, int theRegister, uint8_t numBytes, uint8_t stopTX);
void enableI2CPins();
void firmata_reportDHT();
void firmata_reportSonar();
bool firmata_bufferDataAtPosition(const uint8_t data, const size_t pos);

void firmata_CDC_init(usb_core_driver *dev);
void firmata_parse(uint8_t inputData);

void i2c_config(void);

#endif
