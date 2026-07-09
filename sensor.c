#include "sensor.h"
#include "ti_msp_dl_config.h"
//------------通过设定每个传感器为一个特定数值，确定一个权重，越靠近中心权重越低，最后使得数值为4-------------//
uint8_t sensor_detect(void)
{
    uint8_t sensor[7];
    uint16_t i = 0;
    uint16_t sum = 0;
    if(DL_GPIO_readPin(SENSOR_GRP_SNESOR_0_PORT, SENSOR_GRP_SNESOR_0_PIN)) {i++; sensor[0] = 1;}else sensor[0] = 0;
    if(DL_GPIO_readPin(SENSOR_GRP_SNESOR_1_PORT, SENSOR_GRP_SNESOR_1_PIN)) {i++; sensor[1] = 2;}else sensor[1] = 0;
    if(DL_GPIO_readPin(SENSOR_GRP_SNESOR_2_PORT, SENSOR_GRP_SNESOR_2_PIN)) {i++; sensor[2] = 3;}else sensor[2] = 0;
    if(DL_GPIO_readPin(SENSOR_GRP_SNESOR_3_PORT, SENSOR_GRP_SNESOR_3_PIN)) {i++; sensor[3] = 4;}else sensor[3] = 0;
    if(DL_GPIO_readPin(SENSOR_GRP_SNESOR_4_PORT, SENSOR_GRP_SNESOR_4_PIN)) {i++; sensor[4] = 5;}else sensor[4] = 0;
    if(DL_GPIO_readPin(SENSOR_GRP_SNESOR_5_PORT, SENSOR_GRP_SNESOR_5_PIN)) {i++; sensor[5] = 6;}else sensor[5] = 0;
    if(DL_GPIO_readPin(SENSOR_GRP_SNESOR_6_PORT, SENSOR_GRP_SNESOR_6_PIN)) {i++; sensor[6] = 7;}else sensor[6] = 0;
    sum = sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6];
}
//------------------------------------------从0到6分别为从左到右---------------------------------------//