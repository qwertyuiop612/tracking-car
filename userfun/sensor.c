#include "sensor.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

uint8_t sensor[7] = {0,0,0,0,0,0,0};
int index;
float ALL_num = 0;

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
    float sum =0.0f;
    sensor[0] = get_gpio_state(SENSOR_GRP_SNESOR_0_PORT,SENSOR_GRP_SNESOR_0_PIN) ? (1 && i++) : 0;
    sensor[1] = get_gpio_state(SENSOR_GRP_SNESOR_1_PORT,SENSOR_GRP_SNESOR_1_PIN) ? (2 && i++) : 0;
    sensor[2] = get_gpio_state(SENSOR_GRP_SNESOR_2_PORT,SENSOR_GRP_SNESOR_2_PIN) ? (3 && i++) : 0;
    sensor[3] = get_gpio_state(SENSOR_GRP_SNESOR_3_PORT,SENSOR_GRP_SNESOR_3_PIN) ? (4 && i++) : 0;
    sensor[4] = get_gpio_state(SENSOR_GRP_SNESOR_4_PORT,SENSOR_GRP_SNESOR_4_PIN) ? (5 && i++) : 0;
    sensor[5] = get_gpio_state(SENSOR_GRP_SNESOR_5_PORT,SENSOR_GRP_SNESOR_5_PIN) ? (6 && i++) : 0;
    sensor[6] = get_gpio_state(SENSOR_GRP_SNESOR_6_PORT,SENSOR_GRP_SNESOR_6_PIN) ? (7 && i++) : 0;
    sum = sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6];
    if (i == 0) ALL_num = 0;
    else ALL_num = sum / i;
    index = (int) (ALL_num * 2 + 0.5);
    return ALL_num;
}
//------------------------------------------从0到6分别为从左到右---------------------------------------//

//-------------------------------------------速度对应表-----------------------------------------------//
float Difspeed(){
    switch (index)
    {
        case 2: //ALL_num = 1
            return 400;
            break;
        case 3: //ALL_num = 2
            return 250;
            break;
        case 4: //ALL_num = 3
            return 100;
            break;
        case 5: //ALL_num = 4
            return 0;
            break;
        case 6: //ALL_num = 5
            return -100;
            break;
        case 7: //ALL_num = 6
            return -250;
            break;
        case 8: //ALL_num = 7
            return -400;
            break;
        default:
            break;
    }
}