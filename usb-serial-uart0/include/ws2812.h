#include "firmata_to_board.h"
#include "String.h"
#include "stdio.h"
#include "stdlib.h"

typedef uint8_t neoPixelType;

bool begin(uint8_t p, uint16_t n, uint8_t bright, neoPixelType t);
void updateType(neoPixelType t, uint8_t pin);
bool updateLength(uint16_t n, uint8_t pin);
void setPin(uint8_t p);
void pinMode(uint8_t pin, uint8_t Mode);
void setBrightness(uint8_t pin, uint8_t b);
void setRangeColor(uint8_t pin, int16_t start, int16_t end, uint32_t c);
void setPixelColor(uint8_t pin, uint16_t n, uint32_t c, bool user);
uint32_t hsl(uint32_t h, uint32_t s, uint32_t l);
void clear(uint8_t pin);
uint32_t rgbToColor(uint8_t r, uint8_t g, uint8_t b);
void showRainbow(uint8_t pin, uint16_t start, uint16_t end, uint32_t startHue, uint32_t endHue);
void shift(uint8_t pin, int8_t offset);
void rotate(uint8_t pin, int8_t offset);
uint32_t getPixelColor(uint8_t pin, uint16_t n);
void showBarGraph(uint8_t pin, uint16_t start, uint16_t end, int16_t value, int16_t high);
void show(uint8_t pin);