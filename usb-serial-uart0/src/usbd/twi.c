/*!
    \file    main.c
    \brief   master transmitter
    
    \version 2019-06-05, V1.0.0, firmware for GD32VF103
    \version 2020-08-04, V1.1.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

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

#include "gd32vf103.h"

#define I2C0_OWN_ADDRESS7 0x74 
#define I2C0_SLAVE_ADDRESS7 (0x74<<1)

uint8_t i2c_transmitter[16];

void rcu1_config(void);
void gpio1_config(void);
void i2c1_config(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
extern void digital_test();
extern void uart_write(uint8_t data[], uint8_t len);

int test(void)
{   
    int i;
    i2c1_config();

    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);

    for (i = 0; i < 16; i++)
    {
        i2c_transmitter[i] = i + 0x80;
    }

    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
        ;
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
        ;
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
        ;
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while (!i2c_flag_get(I2C0, I2C_FLAG_TBE))
        ;
    for (i = 0; i < 16; i++)
    {
        /* data transmission */
        i2c_data_transmit(I2C0, i2c_transmitter[i]);
        /* wait until the TBE bit is set */
        while (!i2c_flag_get(I2C0, I2C_FLAG_TBE))
            ;
        digital_test();
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while (I2C_CTL0(I2C0) & 0x0200);
    /* infinite loop */

}

/*!
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu1_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);
}

/*!
    \brief      cofigure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio1_config(void)
{
    /* I2C0 and I2C1 GPIO ports */
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
}

/*!
    \brief      cofigure the I2C0 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c1_config(void)
{
    /* RCU config */
    rcu1_config();

    /* GPIO config */
    gpio1_config();

    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);

    /* I2C address configure */
    // i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);

    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

void sendmessage(uint8_t i2caddr, uint8_t* sendbuf,uint8_t len){
    int i;

    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, (i2caddr<<1));

    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
        ;

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
        ;
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
        ;
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while (!i2c_flag_get(I2C0, I2C_FLAG_TBE))
        ;
    for (i = 0; i < len; i++)
    {
        /* data transmission */
        i2c_data_transmit(I2C0, sendbuf[i]);
        /* wait until the TBE bit is set */
        while (!i2c_flag_get(I2C0, I2C_FLAG_TBE))
            ;
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while (I2C_CTL0(I2C0) & 0x0200)
        ;
    /* infinite loop */
}


//测试iic接收
uint8_t i2c_receiver[16];

void rcu2_config(void);
void gpio2_config(void);
void i2c2_config(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int test_recive(void)
{
    int i;

    i2c1_config();
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);

    i = 0;
    /* send a NACK for the next data byte which will be received into the shift register */
    i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);

    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
        ;
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
        ;
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_RECEIVER);
    /* disable ACK before clearing ADDSEND bit */
    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
        ;
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* Wait until the last data byte is received into the shift register */
    while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
        ;
    /* wait until the RBNE bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE))
        ;
    /* read a data from I2C_DATA */
    i2c_receiver[i++] = i2c_data_receive(I2C0);
    /* wait until the RBNE bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE))
        ;
    /* read a data from I2C_DATA */
    i2c_receiver[i++] = i2c_data_receive(I2C0);
    /* send a stop condition */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while (I2C_CTL0(I2C0) & 0x0200)
        ;
    i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

void receivemessage(uint8_t address, int theRegister, uint8_t numBytes){
    int i;
    if(theRegister != -1){
        sendmessage(address,&theRegister,1);
    }else{
        i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, (address<<1));
    }
    i = 0;
    /* send a NACK for the next data byte which will be received into the shift register */
    i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);

    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
        ;
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
        ;
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_RECEIVER);
    /* disable ACK before clearing ADDSEND bit */
    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
        ;
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* Wait until the last data byte is received into the shift register */
    while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
        ;
    // uart_write(&numBytes,1);
    for(i=0;i<numBytes;i++){
        /* wait until the RBNE bit is set */
        while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE))
            ;
        /* read a data from I2C_DATA */
        i2c_receiver[i++] = i2c_data_receive(I2C0);

        digital_test();
    }
    /* send a stop condition */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while (I2C_CTL0(I2C0) & 0x0200)
        ;
    i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}