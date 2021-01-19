#include <gd32vf103.h>
#include <lcd.h>
#include "usart0.h"
#include "cdc_acm_core.h"

__IO uint8_t data_buffer[64];
__IO uint8_t data_length = 0;

int usart0_init()
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    // eclic_irq_enable(USART0_IRQn, 1, 0);

    return 0;
}

int usart0_irq()
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
        /* receive data */
        data_buffer[data_length++] = usart_data_receive(USART0);

        if (data_length >= sizeof(data_buffer))
            data_length = 0;
    }

    return 0;
}

int usart0_update_config(uint32_t baudrate, uint8_t databits, uint8_t stopbits, uint8_t parity)
{
    /* baudrate configure */
    usart_baudrate_set(USART0, baudrate);

    /* databits configure: either 9 bits or 8 bits, default 8 */
    if (databits == 9)
        usart_word_length_set(USART0, USART_WL_9BIT);
    else
        usart_word_length_set(USART0, USART_WL_8BIT);
    
    /* stopbits configure: 1 1.5 2, defalut 1 */
    switch (stopbits) {
        case 1: /* 1.5 bit */
            usart_stop_bit_set(USART0, USART_STB_1_5BIT);
            break;
        case 2: /* 2 bit */
            usart_stop_bit_set(USART0, USART_STB_2BIT);
            break;
        default: /* default set to 1 bit */
            usart_stop_bit_set(USART0, USART_STB_1BIT);
            break;
    }

    /* parity configure, default none */
    switch (parity) {
        case 1: /* odd */
            usart_parity_config(USART0, USART_PM_ODD);
            break;
        case 2: /* even */
            usart_parity_config(USART0, USART_PM_EVEN);
            break;
        default:
            usart_parity_config(USART0, USART_PM_NONE);
            break;
    }

    return 0;
}
