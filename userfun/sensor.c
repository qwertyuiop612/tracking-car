#include "sensor.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

int sensor[7] = {0,0,0,0,0,0,0};
uint8_t idx = 0;
float trace = 0;
float last_trace = 0;

//----------------------------------------------GPIO 状态读取-------------------------------------------//
uint8_t get_gpio_state(GPIO_Regs *gpio_port,uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    if ((high_bits & gpio) != 0 ) return 1;
    else return 0;
}

//-----------------------------------------------传感器-----------------------------------------------//
float sensor_detect()
{
    int i = 0;
    int sum = 0;
    if (get_gpio_state(SENSOR_GRP_SNESOR_0_PORT,SENSOR_GRP_SNESOR_0_PIN)) {sensor[0] = 1 ; i++;} else sensor[0] = 0;
    if (get_gpio_state(SENSOR_GRP_SNESOR_1_PORT,SENSOR_GRP_SNESOR_1_PIN)) {sensor[1] = 2 ; i++;} else sensor[1] = 0;
    if (get_gpio_state(SENSOR_GRP_SNESOR_2_PORT,SENSOR_GRP_SNESOR_2_PIN)) {sensor[2] = 3 ; i++;} else sensor[2] = 0;
    if (get_gpio_state(SENSOR_GRP_SNESOR_3_PORT,SENSOR_GRP_SNESOR_3_PIN)) {sensor[3] = 4 ; i++;} else sensor[3] = 0;
    if (get_gpio_state(SENSOR_GRP_SNESOR_4_PORT,SENSOR_GRP_SNESOR_4_PIN)) {sensor[4] = 5 ; i++;} else sensor[4] = 0;
    if (get_gpio_state(SENSOR_GRP_SNESOR_5_PORT,SENSOR_GRP_SNESOR_5_PIN)) {sensor[5] = 6 ; i++;} else sensor[5] = 0;
    if (get_gpio_state(SENSOR_GRP_SNESOR_6_PORT,SENSOR_GRP_SNESOR_6_PIN)) {sensor[6] = 7 ; i++;} else sensor[6] = 0;
    sum = sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6];
    if (i == 0)
        trace = 0;
    else
        trace = (float)sum / i;
    if (trace != 0)
        last_trace = trace;
    else if (trace == 0)
        trace = last_trace;
    idx = (trace * 2 + 0.5);
    return trace;
}
//------------------------------------------从0到6分别为从左到右---------------------------------------//