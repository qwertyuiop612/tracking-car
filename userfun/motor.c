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
void motor_LT(int speed)  //原地左转
{
    motor_PWM(-speed,speed);
}
void motor_RT(int speed)  //原地右转
{
    motor_PWM(speed,-speed);
}
void motor_ST(int speed)  //直行
{
    motor_PWM(speed,speed);
}
void motor_BK(int speed)  //后退
{
    motor_PWM(-speed,-speed);
}
void motor_STOP(void)  //停止
{
    motor_PWM(0,0);
}

//-----------------------------------------------电机PWM直接设置------------------------------------------------//
void motor_PWM(int L,int R)  //直接设置PWM
{
    if(L>0)
    {
        PWM_duty(1000-L,0);
        PWM_duty(1000,1);
    }
    else
    {
        PWM_duty(1000,0);
        PWM_duty(1000+L,1);
    }
    if(R>0)
    {
        PWM_duty(1000-R,2);
        PWM_duty(1000,3);
    }
    else
    {
        PWM_duty(1000,2);
        PWM_duty(1000+R,3);
    }
}