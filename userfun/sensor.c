#include "sensor.h"
#include "ti_msp_dl_config.h"

uint8_t sensor[7] = {0,0,0,0,0,0,0};

//----------------------------------------------GPIO 状态读取-------------------------------------------//
uint8_t get_gpio_state(GPIO_Regs *gpio_port,uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    if ((high_bits & gpio) != 0 ) return 1;
    else return 0;
}

//-----------------------------------------------传感器-----------------------------------------------//
void sensor_detect()
{
    sensor[0] = get_gpio_state(SENSOR_GRP_SNESOR_0_PORT,SENSOR_GRP_SNESOR_0_PIN);
    sensor[1] = get_gpio_state(SENSOR_GRP_SNESOR_1_PORT,SENSOR_GRP_SNESOR_1_PIN);
    sensor[2] = get_gpio_state(SENSOR_GRP_SNESOR_2_PORT,SENSOR_GRP_SNESOR_2_PIN);
    sensor[3] = get_gpio_state(SENSOR_GRP_SNESOR_3_PORT,SENSOR_GRP_SNESOR_3_PIN);
    sensor[4] = get_gpio_state(SENSOR_GRP_SNESOR_4_PORT,SENSOR_GRP_SNESOR_4_PIN);
    sensor[5] = get_gpio_state(SENSOR_GRP_SNESOR_5_PORT,SENSOR_GRP_SNESOR_5_PIN);
    sensor[6] = get_gpio_state(SENSOR_GRP_SNESOR_6_PORT,SENSOR_GRP_SNESOR_6_PIN);
}
//------------------------------------------从0到6分别为从左到右---------------------------------------//
