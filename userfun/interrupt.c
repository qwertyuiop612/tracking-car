#include "ti_msp_dl_config.h"
#include "interrupt.h"

extern int status;

uint8_t get_key_state(uint32_t key)
{
    uint32_t high_pins = DL_GPIO_readPins(SWITCH_GRP_PORT, key);
    if((high_pins & key) != 0)
        return 1;
    else
        return 0;
}

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case SWITCH_GRP_SWITCH_0_IIDX:
        status = (status + 1) % 3;
        break;
    case SWITCH_GRP_SWITCH_1_IIDX:
        status = (status + 2) % 3;
        break;
    default:
        break;
    }
}