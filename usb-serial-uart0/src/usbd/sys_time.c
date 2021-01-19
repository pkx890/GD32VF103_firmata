#include "gd32vf103.h"
#include "systick.h"
#include "sys_time.h"

#define LED_PIN GPIO_PIN_7
#define LED_GPIO_PORT GPIOA
#define LED_GPIO_CLK RCU_GPIOA

uint32_t tick;

void led_config(void)
{
    rcu_periph_clock_enable(LED_GPIO_CLK); //enable the peripherals clock
    gpio_init(LED_GPIO_PORT, GPIO_MODE_OUT_PP,
              GPIO_OSPEED_50MHZ, LED_PIN); //GPIO output with push-pull
    GPIO_BC(LED_GPIO_PORT) = LED_PIN;      //bit clear
}

void led_toggle(void)
{
    gpio_bit_write(LED_GPIO_PORT, LED_PIN,
                   (bit_status)(1 - gpio_input_bit_get(LED_GPIO_PORT, LED_PIN)));
}

void timer3_config(uint32_t timer_periph, uint32_t time_interval_ms)
{
    timer_parameter_struct timer_initpara;

    switch (timer_periph)
    {
    case TIMER0:
        rcu_periph_clock_enable(RCU_TIMER0);
        break;
    case TIMER1:
        rcu_periph_clock_enable(RCU_TIMER1);
        break;
    case TIMER2:
        rcu_periph_clock_enable(RCU_TIMER2);
        break;
    case TIMER3:
        rcu_periph_clock_enable(RCU_TIMER3);
        break;
    case TIMER4:
        rcu_periph_clock_enable(RCU_TIMER4);
        break;
    case TIMER5:
        rcu_periph_clock_enable(RCU_TIMER5);
        break;
    case TIMER6:
        rcu_periph_clock_enable(RCU_TIMER6);
        break;
    default:
        break;
    }

    timer_deinit(timer_periph);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler = 99; //108M/10800 = 10K Hz
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    // timer_initpara.period = (uint32_t)10 * time_interval_ms; //(uint32_t)1000000U/time_interval_us;

    timer_initpara.period =time_interval_ms;

    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(timer_periph, &timer_initpara);

    timer_interrupt_enable(timer_periph, TIMER_INT_UP); //update interrupt
    timer_enable(timer_periph);
}

void time3_init(void)
{
    led_config();
    eclic_global_interrupt_enable();
    eclic_set_nlbits(ECLIC_GROUP_LEVEL3_PRIO1);
    eclic_irq_enable(TIMER3_IRQn, 1, 0);
    timer3_config(TIMER3, 1); //2s
    return ;
}

void TIMER3_IRQHandler(void)
{
    if (SET == timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP))
    {
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
        tick++;
    }
    
}

uint32_t micro(void){
    return tick;
}

unsigned long millis(void)
{
    return (unsigned long)(get_timer_value() * (108000000 / 4000));
}

unsigned long micros(void)
{
    return (unsigned long)(get_timer_value() * (108000000 / 4000000));
}
