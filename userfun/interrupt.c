#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

uint32_t tmp_a = 0;
uint32_t tmp_b = 0;
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
    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
        case TB6612_ENCD_LA_IIDX:
            tmp_a++;
            break;

        default:
            break;
    }
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        case TB6612_ENCD_RA_IIDX:
            tmp_b++;
            break;
        case SWITCH_GRP_SWITCH_0_IIDX:          //状态目前定3个，要的另外加入
            status = (status + 1) % 3;
            break;
        case SWITCH_GRP_SWITCH_1_IIDX:
            status = (status + 2) % 3;
            break;
        default:
            break;
    }
}