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