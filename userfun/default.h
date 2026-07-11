#ifndef DEFAULT_H
#define DEFAULT_H

//duty范围为0~4000，4000为最大占空比

#include "ti_msp_dl_config.h"

void delay_ms(uint32_t ms);
void PWM_duty(int duty,int motor_id);
#endif // DEFAULT_H