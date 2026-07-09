#include "track.h"
#include "motor.h"
#include "sensor.h"
#include "ti_msp_dl_config.h"
#include "default.h"

//---------------------------------------------轨迹检测------------------------------------------------//
uint8_t flag(void)
{
    if(sensor_detect() == 0) return 0;
    else return 1;
}

//---------------------------------------------循迹函数------------------------------------------------//
void track(void)
{
    int avrPWM = 500 - fabs(sensor_detect() - 4) * 100; //根据传感器检测到的值计算平均PWM
    int leftPWM = avrPWM - DifPWM(); //左轮PWM
    int rightPWM = avrPWM + DifPWM(); //右轮PWM
    //速度限制
    if (leftPWM > 800) leftPWM = 800; else if (leftPWM < -800) leftPWM = -800;
    if (rightPWM > 800) rightPWM = 800; else if (rightPWM < -800) rightPWM = -800;
    motor_PWM(leftPWM,rightPWM);
}

//---------------------------------------------丢线逻辑------------------------------------------------//
void track_lost(void)
{
    
}