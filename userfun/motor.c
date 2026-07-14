#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

extern uint32_t tmp_a;
extern uint32_t tmp_b;
uint16_t PWM_1_duty = 0;
uint16_t PWM_2_duty = 0;

struct WHEEL LEFT = {0.0f, 0.0f, 0.0f, 0.0f};
struct WHEEL RIGHT = {0.0f, 0.0f, 0.0f, 0.0f};
struct WHEEL WHEEL = {0.0f, 0.0f, 0.0f, 0.0f};

float kp = 0.5;
float ki = 0.1; 
float kd = 0.1; 
float integral = 0;

//---------------------------------------------电机初始化设置为静止--------------------------------------------//
void MOTOR_Init(void)
{
    DL_GPIO_setPins(TB6612_STBY_PORT,TB6612_STBY_PIN);  //使能电机驱动芯片
    DL_Timer_startCounter(PWM_0_INST);
    DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);  //设置电机方向为静止
    DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST,0,GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST,0,GPIO_PWM_0_C1_IDX);
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

//-----------------------------------------------电机转动设置------------------------------------------------//
void motor_PWM(int leftPWM,int rightPWM)
{
    leftPWM =  leftPWM * 4;  //将PWM值扩大4倍，增加电机转速
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
    if(rightPWM > 0)  //右轮正转
    {
        DL_GPIO_setPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
        DL_GPIO_clearPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
        PWM_duty(rightPWM,1);
    }
    else if(rightPWM < 0)  //右轮反转
    {
        DL_GPIO_clearPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
        rightPWM = -rightPWM;
        PWM_duty(rightPWM,1);
    }
    else  //右轮静止
    {
        DL_GPIO_setPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
        PWM_duty(0,1);
    }
}

//------------------------------------------------方向设置------------------------------------------------//
void direction(uint8_t motor_id , uint8_t dir)
{
    if(motor_id == 1)  //左轮方向
    {
        if(dir == 1)
        {
            DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);
            DL_GPIO_clearPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
        }
        else if(dir == 0)
        {
            DL_GPIO_clearPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);
            DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
        }
        else if(dir == 2)   //静止
        {
            DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);
        }
    }
    else if(motor_id == 2)  //右轮方向
    {
        if(dir == 1)
        {
            DL_GPIO_setPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
            DL_GPIO_clearPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
        }
        else if(dir == 0)
        {
            DL_GPIO_clearPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
            DL_GPIO_setPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
        }
        else if(dir == 2)   //静止
        {
            DL_GPIO_setPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
        }
    }
}

//------------------------------------------------速度计算------------------------------------------------//
float cal_speed(uint8_t motor_id)
{
    if (motor_id == 1)
    {
        LEFT.speed = (float)tmp_a / ENCODE * PI * WHEEL_DIAMETER * 200;  //1速度 mm/s  200为频率
        tmp_a = 0;
        return LEFT.speed;
    }
    if (motor_id == 2)
    {
        RIGHT.speed = (float)tmp_b / ENCODE * PI * WHEEL_DIAMETER * 200;  //2速度 mm/s
        tmp_b = 0;
        return RIGHT.speed;
    }
}

//------------------------------------------PID(仅使用PI速度环控制)---------------------------------------------------//

void MOTOR_PID(uint8_t motor_id)
{
    // LEFT.target_speed = WHEEL.target_speed;
    // RIGHT.target_speed = WHEEL.target_speed;

    if (motor_id == 1)
    {
        if (PWM_1_duty > 4000)
            PWM_1_duty = 4000;
        float error = LEFT.target_speed - LEFT.speed;
        LEFT.current_error = error;
        PWM_1_duty += (uint16_t)(kp * (LEFT.current_error - LEFT.last_error) + ki * LEFT.current_error);     //增量的PI控制
        LEFT.last_error = LEFT.current_error;
        PWM_duty(PWM_1_duty,motor_id);
    }
    else if (motor_id == 2)
    {
        if (PWM_2_duty > 4000)
            PWM_2_duty = 4000;
        float error = RIGHT.target_speed - RIGHT.speed;
        RIGHT.current_error = error;
        PWM_2_duty += (uint16_t)(kp * (RIGHT.current_error - RIGHT.last_error) + ki * RIGHT.current_error);
        RIGHT.last_error = RIGHT.current_error;
        PWM_duty(PWM_2_duty,motor_id);
    }
}

//-------------------------------------中断函数(电机PID)计算------------------------------------------------//
void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
        case DL_TIMER_IIDX_LOAD:
            cal_speed(1);
            MOTOR_PID(1);
            cal_speed(2);
            MOTOR_PID(2);
            break;
        default:
            break;
    }
}