#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

extern int sensor[7];

//----------------------------------------------循迹----------------------------------------------//
void track()
{
    static float last_left = 500;
    static float last_right = 500;
    float detect_val = sensor_detect(); // 只读一次，避免 idx/sensor[] 被覆写
    int avrspeed = 600 - fabs(detect_val - 4) * 150;

    // 用整数判断是否丢线，避免浮点 == 0 不可靠
    int sensor_sum = sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6];
    if (sensor_sum == 0) // 丢线，使用上一次有效目标速度
    {
        LEFT.target_speed = last_left;
        RIGHT.target_speed = last_right;
    }
    else if (sensor[0] && sensor[1] && sensor[2] && sensor[3] && sensor[4] && sensor[5] && sensor[6]) // 起点、终点
    {
        LEFT.target_speed = 200;
        RIGHT.target_speed = 200;
        last_left = 200;
        last_right = 200;
    }
    else
    {
        LEFT.target_speed = avrspeed - Difspeed();
        RIGHT.target_speed = avrspeed + Difspeed();
        last_left = LEFT.target_speed;
        last_right = RIGHT.target_speed;
    }
}