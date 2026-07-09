#include "ti_msp_dl_config.h"
#include "default.h"


void delay_ms(uint32_t ms)

{
    while(ms--)
    {
        delay_cycles(CPUCLK_FREQ/1000);
    }
}

void PWM_duty(float duty,uint8_t channel)
{
    uint32_t compare;
    compare = PWM_0_INST_CLK_FREQ/1000 - PWM_0_INST_CLK_FREQ/1000 * duty;
    if(channel == 0)
    {
        DL_TimerG_setCaptureCompareValue(PWM_0_INST,compare,DL_TIMER_CC_0_INDEX);
    }
    else if(channel == 1)
    {
        DL_TimerG_setCaptureCompareValue(PWM_0_INST,compare,DL_TIMER_CC_1_INDEX);
    }
}