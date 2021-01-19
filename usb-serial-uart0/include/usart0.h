#ifndef _USART0_H
#define _USART0_H

int usart0_init(void);
int usart0_irq(void);

int usart0_update_config(uint32_t baudrate, uint8_t databits, uint8_t stopbits, uint8_t parity);


#endif