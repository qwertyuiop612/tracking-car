#include "ti_msp_dl_config.h"
#include "default.h"

//---------------------------------------------延时函数(毫秒)--------------------------------------------//
void delay_ms(uint32_t ms)

{
    while(ms--)
    {
        delay_cycles(CPUCLK_FREQ/1000);
    }
}

//-----------------------------------------------PWM占空比设置-------------------------------------------//
void PWM_duty(float duty,uint8_t channel)
{
    uint32_t compare;
    if (duty > 0)                 //大于0 正转
    { 
        compare = PWM_0_INST_CLK_FREQ/1000 - PWM_0_INST_CLK_FREQ/1000 * duty/1000;
        if(channel == 0)
        {
            DL_TimerG_setCaptureCompareValue(PWM_0_INST,compare,DL_TIMER_CC_0_INDEX);
        }
        else if(channel == 1)
        {
            DL_TimerG_setCaptureCompareValue(PWM_0_INST,compare,DL_TIMER_CC_1_INDEX);
        }
        else if(channel == 2)
        {
            DL_TimerG_setCaptureCompareValue(PWM_1_INST,compare,DL_TIMER_CC_0_INDEX);
        }
        else if(channel == 3)
        {
            DL_TimerG_setCaptureCompareValue(PWM_1_INST,compare,DL_TIMER_CC_1_INDEX);
        }
    }
    else if(duty < 0)            //小于0 反转
    {
        compare = PWM_0_INST_CLK_FREQ/1000 + PWM_0_INST_CLK_FREQ/1000 * -duty/1000;
        if(channel == 0)
        {
            DL_TimerG_setCaptureCompareValue(PWM_0_INST,compare,DL_TIMER_CC_1_INDEX);
        }
        else if(channel == 1)
        {
            DL_TimerG_setCaptureCompareValue(PWM_0_INST,compare,DL_TIMER_CC_0_INDEX);
        }
        else if(channel == 2)
        {
            DL_TimerG_setCaptureCompareValue(PWM_1_INST,compare,DL_TIMER_CC_1_INDEX);
        }
        else if(channel == 3)
        {
            DL_TimerG_setCaptureCompareValue(PWM_1_INST,compare,DL_TIMER_CC_0_INDEX);
        }
    }
}