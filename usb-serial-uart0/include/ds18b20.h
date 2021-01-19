#if 0
#include "stdint.h"

uint8_t ds18b20_init(uint8_t pin);
int getTemp(uint8_t *p);
void reset_search(void);
uint8_t search(uint8_t *newAddr);
void Select(const uint8_t rom[8]);
void ds18b20_write(uint8_t v, uint8_t power);
uint8_t ds18b20_read(void);
uint8_t reset(void);
uint8_t read_bit(void);
void write_bit(uint8_t v);
uint8_t crc8(const uint8_t *addr, uint8_t len);

extern void delayMicroseconds(uint32_t time_us);
extern void uart_write(uint8_t data[], uint8_t len);

#endif

#ifndef _TEMP_SENSOR_H
#define _TEMP_SENSOR_H

#include "stdint.h"

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_4

uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);
int DS18B20_GetCelsiusTemp(void);

#endif