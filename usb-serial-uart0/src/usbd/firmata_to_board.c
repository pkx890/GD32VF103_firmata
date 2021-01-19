#include "firmata_to_board.h"
uint8_t pinConfig[TOTAL_PINS];

uint8_t getPinMode(uint16_t pin){
    return pinConfig[pin];
}

void setPinMode(uint16_t pin, int mode){
    uint8_t PORT = pin / 16;
    uint8_t Pin = pin % 16;
    if (pinConfig[pin] == PIN_MODE_IGNORE){
        return;
    }
    pinConfig[pin] = mode;
    switch (mode)
    {
        case PIN_MODE_ANALOG:
            break;
        case PIN_MODE_INPUT:
            // switch (PORT){
            //     case 0:
            //         gpio_init(GPIOA, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, BIT(Pin));
            //         break;
            //     case 1:
            //         gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, BIT(Pin));
            //         break;
            //     case 2:
            //         gpio_init(GPIOC, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, BIT(Pin));
            //         break;
            //     default:
            //         break;
            // }
            break;
        case PIN_MODE_PULLUP:
            break;
        case PIN_MODE_OUTPUT:
            switch (PORT){
                case 0:
                    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
                    break;
                case 1:
                    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
                    break;
                case 2:
                    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
                    break;
                default:
                    break;
            }
            break;
        case PIN_MODE_PWM:
            switch (PORT)
            {
            case 0:
                rcu_periph_clock_enable(RCU_GPIOA);
                rcu_periph_clock_enable(RCU_AF);
                gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
                break;
            case 1:
                rcu_periph_clock_enable(RCU_GPIOB);
                rcu_periph_clock_enable(RCU_AF);
                gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
                break;
            case 2:
                rcu_periph_clock_enable(RCU_GPIOC);
                rcu_periph_clock_enable(RCU_AF);
                gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BIT(Pin));
                break;
            default:
                break;
            }
            break;
        case PIN_MODE_SERVO:
            break;
        case PIN_MODE_I2C:
            break;
        case PIN_MODE_SERIAL:
            break;
        case PIN_MODE_TONE:
            break;
        case PIN_MODE_SONAR:
            break;
        case PIN_MODE_DHT:
            break;
        case PIN_MODE_STEPPER:
            break;
        default:
            break;
    }
}

void digitalWrite(uint8_t pin, uint8_t value)
{
    uint8_t GPIO_PORT=pin/16;
    uint8_t Pin = pin % 16;
    switch(GPIO_PORT){
        case 0:
            gpio_bit_write(GPIOA, BIT(Pin), value);
            break;
        case 1:
            gpio_bit_write(GPIOB, BIT(Pin), value);
            break;
        case 2:
            gpio_bit_write(GPIOC, BIT(Pin), value);
            break;
        default:
            break;
    }
}

bool digitalRead(uint8_t pin){
    uint8_t GPIO_PORT = pin / 16;
    uint8_t Pin = pin % 16;
    uint8_t val=0;
    switch (GPIO_PORT){
        case 0:
            val = gpio_input_bit_get(GPIOA, BIT(Pin));
            break;
        case 1:
            val = gpio_input_bit_get(GPIOB, BIT(Pin));
            break;
        case 2:
            val = gpio_input_bit_get(GPIOC, BIT(Pin));
            break;
        default:
            break;
    }
    if (val==0)
        return FALSE;
    else if(val==1)
        return TRUE;
    return 0;
}

void set_PWM(uint8_t pin, uint8_t duty, int freq)
{
    timer_config(pin, duty, freq);
}