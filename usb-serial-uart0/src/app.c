/*!
    \file  main.c
    \brief USB CDC ACM device

    \version 2019-6-5, V1.0.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/


#include "drv_usb_hw.h"
#include "cdc_acm_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
#include "usart0.h"
#include "dbgprint.h"
#include "firmata.h"

//测试PWM
#include "timer.h"

//测试adc
#include "adc.h"

//测试IIC
#include "twi.h"

//测试spi
#include "spi.h"

//测试dht
#include "dht.h"

//测试获取系统时间
#include "sys_time.h"

//测试DS18B20
#include "ds18b20.h"

//测试灯带
#include "ws2812.h"

usb_core_driver USB_OTG_dev = 
{
    .dev = {
        .desc = {
            .dev_desc       = (uint8_t *)&device_descriptor,
            .config_desc    = (uint8_t *)&configuration_descriptor,
            .strings        = usbd_strings,
        }
    }
};

void send(char *buf)
{
    int len = strlen(buf);
    for (int i = 0; i < len; i++) {
        usart_data_transmit(USART0, buf[i]);
        while (usart_flag_get(USART0, USART_FLAG_TC) == RESET) {}
    }
}

/*!
    \brief      main routine will construct a USB keyboard
    \param[in]  none
    \param[out] none
    \retval     none
*/

void digital_test()
{
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    while(1){
    for(int i=0;i<2;i++){
        gpio_bit_write(GPIOA, BIT(4), !digitalRead(4));
        delay_ms(1000);
    }
    }
}

void PWM_test(){
}

extern uint16_t adc_value[4];

void adc_test(){
    /* system clocks configuration */
    rcu_config();
    /* GPIO configuration */
    gpio_config();
    /* DMA configuration */
    dma_config();
    /* ADC configuration */
    adc_config();
    /* configure COM port */
    gd_eval_com_init(EVAL_COM0);

    while (1)
    {
        delay_1ms(1000);
        // printf("\r\n *******************************");
        printf("\r\n ADC0 regular channel data = 0x%04X", adc_value[0]);
        printf("\r\n ADC0 regular channel data = 0x%04X", adc_value[1]);
        printf("\r\n ADC0 regular channel data = 0x%04X", adc_value[2]);
        printf("\r\n ADC0 regular channel data = 0x%04X\r\n", adc_value[3]);
    }
}

void iic_test(void)
{
    while(1)
    test_recive();
}

void spi_test(){
    test1();
}

void delay_ms(uint32_t time_ms){
    usb_mdelay(time_ms);
}

void delayMicroseconds(uint32_t time_us){
    usb_udelay(time_us);
}

extern uint32_t tick;
void test_dht11(void){
}

void millis_test(){
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    uint32_t t = micros();
    while(1){
        if ((micros() - t) > 1000000)
        {
            t = micros();
            gpio_bit_write(GPIOA, BIT(4), !gpio_input_bit_get(GPIOA, GPIO_PIN_4));
        }
    }
}

void test_ds18b20(){
    int i=0;
    uint8_t state;
    while(1){
        state = DS18B20_GetCelsiusTemp();
        uart_write(&state,4);
        usb_mdelay(1000);
    }
}

volatile int data = 0;
volatile int da = 2;

void test_HZ(){
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    while(1){
        // gpio_bit_write(GPIOA, BIT(4),1);
        // gpio_bit_write(GPIOA, BIT(4), 0);
        data = da;
        while (data--)
            ;
        
        GPIO_BOP(GPIOA) = BIT(4);
        
        // usb_udelay(10);
        data = da;
        while (data--)
            ;
       
        GPIO_BC(GPIOA) = BIT(4);
        
        // usb_udelay(10);
    }
}

volatile uint32_t systick;
// extern void sendBuffer(uint32_t pin, uint8_t* data_address, uint16_t num_leds);
void test_ws281(){
    // uint8_t data[30]={1};
    // sendBuffer(4, data, 1);
    begin(4,1,100,50);
    setRangeColor(4,0, 4, 0x0000FF);
}

int main(void)
{
    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL2_PRIO2);

    //串口0初始化
    usart0_init();

    //配置系统主频
    usb_rcu_config();

    //初始化需要时钟（包括timer2）
    usb_timer_init();

    //初始化USB中断
    usb_intr_config();

    //将USB的结构体传到firmata文件中便于上报数据
    firmata_CDC_init(&USB_OTG_dev);
    
    //对firmata的全局变量进行初始化
    firmata_init();
    
    //USB_CDC 初始化
    usbd_init (&USB_OTG_dev, USB_CORE_ENUM_FS, &usbd_cdc_cb);

    time3_init();

    /* check if USB device is enumerated successfully */
    while (USBD_CONFIGURED != USB_OTG_dev.dev.cur_status) {
    }

    while (1) {
        if(USBD_CONFIGURED == USB_OTG_dev.dev.cur_status){
            extern uint8_t packet_receive;
            if (packet_receive) {
                cdc_acm_data_receive(&USB_OTG_dev);
            }
            // TODO
            extern uint8_t data_buffer[64];
            extern uint8_t data_length;
            if (data_length) {
                for (uint8_t i = 0; i < data_length; i++)
                {
                    firmata_parse(data_buffer[i]);
                    usb_udelay(50);
                }
                data_length = 0;
            }
        }
    }
}