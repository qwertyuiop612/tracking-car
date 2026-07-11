#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

extern uint32_t tmp_a;
float speed_1 = 0.0;
float speed_2 = 0.0;

//---------------------------------------------电机初始化设置为静止--------------------------------------------//
void motor_init(void)
{
    DL_GPIO_setPins(TB6612_STBY_PORT,TB6612_STBY_PIN);  //使能电机驱动芯片
    DL_Timer_startCounter(PWM_0_INST);
    DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);  //设置电机方向为静止
    DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST,0,GPIO_PWM_0_C0_IDX);
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

//-----------------------------------------------电机转动设置------------------------------------------------//
void motor_PWM(int leftPWM,int rightPWM)
{
    leftPWM = leftPWM * 4;  //将PWM值扩大4倍，增加电机转速
    rightPWM = rightPWM * 4;
    if(leftPWM > 0)  //左轮正转
    {
        DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);
        DL_GPIO_clearPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
        PWM_duty(leftPWM,0);
    }
    else if(leftPWM < 0)  //左轮反转
    {
        DL_GPIO_clearPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);
        DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
        leftPWM = -leftPWM;
        PWM_duty(leftPWM,0);
    }
    else  //左轮静止
    {
        DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);
        DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
        PWM_duty(0,0);

    }
    /*if(rightPWM > 0)  //右轮正转
    {
        DL_GPIO_setPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
        DL_GPIO_clearPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
    }
    else if(rightPWM < 0)  //右轮反转
    {
        DL_GPIO_clearPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
        rightPWM = -rightPWM;
    }
    else  //右轮静止
    {
        DL_GPIO_setPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
    }*/
}

//------------------------------------------------速度计算------------------------------------------------//
float cal_speed(uint8_t motor_id)
{
    float speed = 0.0;
    if (motor_id == 1)
    {
        speed_1 = (float)tmp_a / ENCODE * PI * WHEEL_DIAMETER * 200;  //1速度 mm/s  200为频率
        tmp_a = 0;
    }
    /*if (motor_id == 2)
    {
        speed_2 = (float)tmp_a / ENCODE * PI * WHEEL_DIAMETER * 200;  //2速度 mm/s
        tmp_b = 0;
    }*/
    return speed;
}

//-------------------------------------中断函数(电机PID)计算------------------------------------------------//
