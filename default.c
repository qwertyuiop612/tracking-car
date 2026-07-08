#include "ti_msp_dl_config.h"
#include "default.h"


void delay_ms(uint32_t ms)

{
    while(ms--)
    {
        delay_cycles(CPUCLK_FREQ/1000);
    }
}