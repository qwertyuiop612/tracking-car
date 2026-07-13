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
    int avrspeed = 600 - fabs(sensor_detect() - 4) * 100;

    if (sensor_detect() == 0)   //丢线
    {
        direction(1,1);
        direction(2,1);
    }
    else if (sensor[0] && sensor[1] && sensor[2] && sensor[3] && sensor[4] && sensor[5] && sensor[6]) // 起点、终点停止
    {
        WHEEL.target_speed = 0;
    }
    else
    {
        LEFT.target_speed = avrspeed - Difspeed();
        RIGHT.target_speed = avrspeed + Difspeed();
    }
}