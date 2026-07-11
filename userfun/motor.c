#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"


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

//-------------------------------------中断函数(电机PID)计算------------------------------------------------//
void MOTOR_PID_INST_IRQHandler(void)
{
    sensor_Get_Right_speed();
    switch( DL_TimerG_getPendingInterrupt(MOTOR_PID_INST ) )
    {
        case DL_TIMER_IIDX_ZERO:
            PID_Calculate();
            break;
        default:
            break;
    }
}