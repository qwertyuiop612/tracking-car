#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

// 文件级共享状态：last_state 与 丢线标志 DX
static float flag_last_state = 0.0f;
static int flag_DX = 0;

//--------------------------------------------轨迹检测-----------------------------------------//
float flag(void)
{
    int state = sensor_detect();
    if (!state && flag_DX == 0)
    {
        flag_DX = 1;  // 检测到丢线，切换到丢线状态
    }
    if (flag_DX == 0)
    {
        flag_last_state = (float)state;  // 未丢线时，记录上一次的状态
    }
    return (float)flag_DX;
}

//--------------------------------------------循迹函数----------------------------------------//
void track(void)
{
    int avrPWM = 500 - fabs(sensor_detect() - 4) * 100; //根据传感器检测到的值计算平均PWM
    int leftPWM = avrPWM - DifPWM(); //左轮PWM
    int rightPWM = avrPWM + DifPWM(); //右轮PWM
    //----------------------------------------速度限制-----------------------------------------//
    if (leftPWM > 800) leftPWM = 800; else if (leftPWM < -800) leftPWM = -800;
    if (rightPWM > 800) rightPWM = 800; else if (rightPWM < -800) rightPWM = -800;
    //---------------------------------------电机PWM设置---------------------------------------//
    motor_PWM(leftPWM,rightPWM);
}

//--------------------------------------------丢线逻辑-----------------------------------------//
void track_lost(void)
{
    float turn_state; 
    static uint8_t i = 0;
    if (i < 1)
    {
        i++;
        turn_state = flag_last_state; // 使用 flag 中记录的 last_state
    }
    if (turn_state > 4) // 如果 last_state 大于 4，说明上一次检测到的线路在右侧
    {
        motor_PWM(400,-100); // 右转
    }
    else if (turn_state < 4) // 如果 last_state 小于 4，说明上一次检测到的线路在左侧
    {
        motor_PWM(-100,400); // 左转
    }
    else // 如果 last_state 等于 4，说明上一次检测到的线路在中间
    {
        motor_PWM(400,400); // 直行
    }
    // 若重新检测到线路（不再丢线），清除 flag 中的丢线标志 DX
    if (sensor_detect() != 0)
    {
        flag_DX = 0;
    }
}