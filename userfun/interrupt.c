#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

uint32_t tmp_a = 0;
uint32_t tmp_b = 0;

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

        default:
            break;
    }
}
