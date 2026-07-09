#include "motor.h"
#include "sensor.h"
#include "ti_msp_dl_config.h"

//---------------------------------------------电机初始化设置为静止--------------------------------------------//
void motor_init(void)
{
    SYSCFG_DL_PWM_0_init();
    PWM_duty(1000,0);
    PWM_duty(1000,1);
}

//-----------------------------------------------电机转动设置------------------------------------------------//
void motor_LT(int speed)  //左转
{
    PWM_duty(-speed,0);
    PWM_duty(speed,1);
    PWM_duty(speed,2);
    PWM_duty(-speed,3);
}
void motor_RT(int speed)  //右转
{
    PWM_duty(speed,0);
    PWM_duty(-speed,1);
    PWM_duty(-speed,2);
    PWM_duty(speed,3);
}
void motor_ST(int speed)  //直行
{
    PWM_duty(speed,0);
    PWM_duty(speed,1);
    PWM_duty(speed,2);
    PWM_duty(speed,3);
}
void motor_BK(int speed)  //后退
{
    PWM_duty(-speed,0);
    PWM_duty(-speed,1);
    PWM_duty(-speed,2);
    PWM_duty(-speed,3);
}
void motor_STOP(void)  //停止
{
    PWM_duty(1000,0);
    PWM_duty(1000,1);
    PWM_duty(1000,2);
    PWM_duty(1000,3);
}
