#include "gd32vf103.h"
#include <stdio.h>
#include "gd32vf103v_eval.h"

void gpio_config(void);
void timer_config(uint8_t pin, uint8_t duty, int freq);