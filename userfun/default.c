#include "ti_msp_dl_config.h"
#include "default.h"

//详细信息在default.h中


//---------------------------------------------延时函数(毫秒)--------------------------------------------//
void delay_ms(uint32_t ms)

{
    while(ms--)
    {
        delay_cycles(CPUCLK_FREQ/1000);
    }
}

//-----------------------------------------------PWM占空比设置-------------------------------------------//
void PWM_duty(int duty,int motor_id)
{
    if (duty > PWM_0_INST_CLK_FREQ/10000) duty = PWM_0_INST_CLK_FREQ/10000;  // 限制最大占空比
    if(motor_id == 1)
    {
        DL_Timer_setCaptureCompareValue(PWM_0_INST,duty,GPIO_PWM_0_C0_IDX);
    }
    else if(motor_id == 2)
    {
        DL_Timer_setCaptureCompareValue(PWM_0_INST,duty,GPIO_PWM_0_C1_IDX);
    }
}