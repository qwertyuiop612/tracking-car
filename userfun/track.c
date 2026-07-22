#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

extern int sensor[7];
float p_kp = 90 * 0.6; // 经验值，待测量
float p_ki = 0.4;
float p_kd = 290 * 0.3;
float p_err;              // 当前误差
float p_last_err;         // 上次误差
float p_err_sum;          // 累计误差
float p_err_dif;          // 误差差值
float p_err_filtered = 0; // 低通滤波后的误差

//---------------------------------------------防止过调-------------------------------------------//
void I_lim(float limit)
{
    if (p_err_sum >= limit)
        p_err_sum = limit;
    else if (p_err_sum <= -limit)
        p_err_sum = -limit;
}

//----------------------------------------------pid----------------------------------------------//
int PLACE_pid(float measure, float cal)
{
    p_err = measure - cal;

    // 低通滤波平滑误差突变，防止边缘检测时剧烈摆动
    p_err_filtered = p_err_filtered * 0.7f + p_err * 0.3f;

    // 积分分离：误差较大时清除积分，防止饱和
    if (p_err_filtered > 2.0f || p_err_filtered < -2.0f)
        p_err_sum = 0;
    else
        p_err_sum += p_err_filtered;
    I_lim(200.0);

    p_err_dif = p_err_filtered - p_last_err;
    p_last_err = p_err_filtered;

    return p_kp * p_err_filtered + p_ki * p_err_sum + p_kd * p_err_dif;
}

//--------------------------------------------循迹------------------------------------------------//
void track()
{
    float turn = PLACE_pid(4, sensor_detect());
    // 限制转向幅度，防止左右轮速度差过大导致剧烈摆动
    if (turn > 300.0f)
        turn = 300.0f;
    if (turn < -300.0f)
        turn = -300.0f;
    LEFT.target_speed = 400 - turn;
    RIGHT.target_speed = 400 + turn;
}
