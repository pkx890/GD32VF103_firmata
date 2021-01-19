#include "dht.h"
#include "firmata_to_board.h"

#include "sys_time.h"

uint8_t _bits[5]; // buffer to receive data
int dhtNumLoops = 0;
int dhtLoopCounter = 0;
int numActiveDHTs = 0;
uint8_t DHT_PinNumbers[MAX_DHTS];
uint8_t DHT_WakeUpDelay[MAX_DHTS];
uint8_t DHT_TYPE[MAX_DHTS];
uint8_t nextDHT = 0;
uint8_t currentDHT = 0; // Keeps track of which sensor is active.

extern uint32_t tick;

#define OUTPUT TRUE
#define INPUT FALSE

void change_gpio_mode(uint8_t pin,bool mode){
    uint8_t PORT = pin / 16;
    uint8_t Pin = pin % 16;
    switch (PORT){
        case 0:
            if(mode==OUTPUT)
                gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
            else
                gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, BIT(Pin));
            break;
        case 1:
            if (mode == OUTPUT)
                gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
            else
                gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, BIT(Pin));
            break;
        case 2:
            if (mode == OUTPUT)
                gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
            else
                gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, BIT(Pin));
            break;
    }
}

extern void delayMicroseconds(uint32_t time_us);

int get_dht11(int index)
{
    uint8_t mask = 128;
    uint8_t idx = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        _bits[i] = 0;
    }
    uint8_t temp;
    uint16_t loopCnt;

    uint8_t pin = DHT_PinNumbers[index];
    uint8_t wakeupDelay = DHT_WakeUpDelay[index];

    if (dhtLoopCounter++ > 100)
    {
    dhtLoopCounter = 0;

    uint8_t PORT = pin / 16;
    uint8_t Pin = pin % 16;

    change_gpio_mode(pin,OUTPUT);

    // gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

    gpio_bit_write(GPIOA, BIT(pin), 0);
    delay_ms(18);
    gpio_bit_write(GPIOA, BIT(pin), 1);

    change_gpio_mode(pin, INPUT);

    // gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    delayMicroseconds(40);
    loopCnt = DHTLIB_TIMEOUT;

    while (digitalRead(BIT(pin)) == 0)
    {
        if (--loopCnt == 0)
        {
            return DHTLIB_ERROR_TIMEOUT;
        }
    }

    loopCnt = DHTLIB_TIMEOUT;
    while (digitalRead(BIT(pin)) == 1)
    {
        if (--loopCnt == 0)
        {
            return DHTLIB_ERROR_TIMEOUT;
        }
    }

    for (uint8_t i = 40; i != 0; i--)
    {
        loopCnt = DHTLIB_TIMEOUT;
        while (digitalRead(BIT(pin)) == 0)
        {
            if (--loopCnt == 0)
            {
                return DHTLIB_ERROR_TIMEOUT;
            }
        }
        uint32_t t = micro();
        loopCnt = DHTLIB_TIMEOUT;
        while (digitalRead(BIT(pin)) == 1)
        {
            if (--loopCnt == 0)
            {
                return DHTLIB_ERROR_TIMEOUT;
            }
        }
        if ((micro() - t) > 20)
        {
            _bits[idx] |= mask;
        }

        tick = 0;

        mask >>= 1;
        if (mask == 0)
        {
            mask = 128;
            idx++;
        }
    }
    }
    return DHTLIB_OK;
}