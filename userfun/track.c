#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

extern int sensor[7];

//----------------------------------------------丢线检测----------------------------------------------//
void flag()
{
    sensor_detect();
    if (sensor[0] && sensor[1] && sensor[2] && sensor[3] && sensor[4] && sensor[5] && sensor[6] == 0)
    {
        direction(1,1);
        direction(2,1);
    }
}