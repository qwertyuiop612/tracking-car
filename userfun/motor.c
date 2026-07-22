#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"

extern uint32_t tmp_a;
extern uint32_t tmp_b;
volatile uint16_t PWM_1_duty = 0;
volatile uint16_t PWM_2_duty = 0;

struct WHEEL LEFT = {0.0f, 0.0f, 0.0f, 0.0f};
struct WHEEL RIGHT = {0.0f, 0.0f, 0.0f, 0.0f};
struct WHEEL WHEEL = {0.0f, 0.0f, 0.0f, 0.0f};

float v_kp = 2.0;
float v_ki = 0.5;
float v_kd = 0.1;
float integral = 0;

//---------------------------------------------电机初始化设置为静止--------------------------------------------//
void MOTOR_Init(void)
{
    DL_Timer_startCounter(PWM_0_INST);
    DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);  //设置电机方向为静止
    DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 100, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 100, GPIO_PWM_0_C1_IDX);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN); // NVIC使能，定时器在main中启动
}

// 在方向和PWM配置完成后使能电机驱动
void MOTOR_Enable(void)
{
    DL_GPIO_setPins(TB6612_STBY_PORT, TB6612_STBY_PIN);
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
        PWM_1_duty = leftPWM;
        PWM_duty(leftPWM, 1); // 左轮 → C0 (PA12)
    }
    else if(leftPWM < 0)  //左轮反转
    {
        DL_GPIO_clearPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);
        DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
        leftPWM = -leftPWM;
        PWM_1_duty = leftPWM;
        PWM_duty(leftPWM, 1); // 左轮 → C0 (PA12)
    }
    else  //左轮静止
    {
        DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN);
        DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN);
        PWM_1_duty = 0;
        PWM_duty(0, 1); // 左轮 → C0 (PA12)
    }
    if(rightPWM > 0)  //右轮正转
    {
        DL_GPIO_clearPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
        PWM_2_duty = rightPWM;
        PWM_duty(rightPWM, 2); // 右轮 → C1 (PA13)
    }
    else if(rightPWM < 0)  //右轮反转
    {
        DL_GPIO_setPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
        DL_GPIO_clearPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
        rightPWM = -rightPWM;
        PWM_2_duty = rightPWM;
        PWM_duty(rightPWM, 2); // 右轮 → C1 (PA13)
    }
    else  //右轮静止
    {
        DL_GPIO_setPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN);
        PWM_2_duty = 0;
        PWM_duty(0, 2); // 右轮 → C1 (PA13)
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
            DL_GPIO_setPins(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
        }
    }
    else if(motor_id == 2)  //右轮方向
    {
        if(dir == 1)
        {
            DL_GPIO_clearPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
            DL_GPIO_setPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
        }
        else if(dir == 0)
        {
            DL_GPIO_setPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
            DL_GPIO_clearPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
        }
        else if(dir == 2)   //静止
        {
            DL_GPIO_setPins(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
            DL_GPIO_setPins(TB6612_BIN2_PORT, TB6612_BIN2_PIN);
        }
    }
}

//------------------------------------------------速度计算------------------------------------------------//
float cal_speed(uint8_t motor_id)
{
    if (motor_id == 1)
    {
        LEFT.speed = (float)tmp_a / ENCODE * PI * WHEEL_DIAMETER * 100; // 1速度 mm/s  200为频率
        tmp_a = 0;
        return LEFT.speed;
    }
    if (motor_id == 2)
    {
        RIGHT.speed = (float)tmp_b / ENCODE * PI * WHEEL_DIAMETER * 100; // 2速度 mm/s
        tmp_b = 0;
        return RIGHT.speed;
    }
    return 0;
}

//------------------------------------------PID(仅使用PI速度环控制)---------------------------------------------------//
// error = target - speed, new_duty = old_duty + pid_out
// 当速度偏低时 error>0 → pid_out>0 → CC增大 → 占空比增大 → 加速
// 当速度偏高时 error<0 → pid_out<0 → CC减小 → 占空比减小 → 减速

void MOTOR_PID(uint8_t motor_id)
{
    if (motor_id == 1)
    {
        // 根据目标速度符号设置方向，PID 用绝对值计算
        if (LEFT.target_speed > 0)
            direction(1, 1); // 正转
        else if (LEFT.target_speed < 0)
            direction(1, 0); // 反转
        else
            direction(1, 2); // 静止

        float target = LEFT.target_speed > 0 ? LEFT.target_speed : -LEFT.target_speed;
        float error = target - LEFT.speed;
        LEFT.current_error = error;
        int16_t pid_out = (int16_t)(v_kp * (LEFT.current_error - LEFT.last_error) + v_ki * LEFT.current_error);
        int32_t new_duty = (int32_t)PWM_1_duty + pid_out;
        if (new_duty <= 0)
            PWM_1_duty = 0;
        else if (new_duty >= 4000)
            PWM_1_duty = 4000;
        else
            PWM_1_duty = new_duty;
        LEFT.last_error = LEFT.current_error;
        PWM_duty(PWM_1_duty, motor_id);
    }
    else if (motor_id == 2)
    {
        // 根据目标速度符号设置方向，PID 用绝对值计算
        if (RIGHT.target_speed > 0)
            direction(2, 1); // 正转
        else if (RIGHT.target_speed < 0)
            direction(2, 0); // 反转
        else
            direction(2, 2); // 静止

        float target = RIGHT.target_speed > 0 ? RIGHT.target_speed : -RIGHT.target_speed;
        float error = target - RIGHT.speed;
        RIGHT.current_error = error;
        int16_t pid_out = (int16_t)(v_kp * (RIGHT.current_error - RIGHT.last_error) + v_ki * RIGHT.current_error);
        int32_t new_duty = (int32_t)PWM_2_duty + pid_out;
        if (new_duty <= 0)
            PWM_2_duty = 0;
        else if (new_duty >= 4000)
            PWM_2_duty = 4000;
        else
            PWM_2_duty = new_duty;
        RIGHT.last_error = RIGHT.current_error;
        PWM_duty(PWM_2_duty, motor_id);
    }
}

//-------------------------------------中断函数(电机PID)计算------------------------------------------------//
void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        track(); // 位置 PID，与速度 PID 同频 100Hz
        cal_speed(1);
        MOTOR_PID(1);
        cal_speed(2);
        MOTOR_PID(2);
        break;
    default:
        break;
    }
}