#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"
#include "gyro.h"

extern uint32_t tmp_a;
extern uint32_t tmp_b;
volatile uint16_t PWM_1_duty = 0;
volatile uint16_t PWM_2_duty = 0;
volatile uint8_t motor_brake_flag = 0;

struct WHEEL LEFT = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
struct WHEEL RIGHT = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
struct WHEEL WHEEL = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 速度 PID 参数（可通过 UART "SETV P:x I:y D:z" 命令修改）
float v_kp = 14.0f;
float v_ki = 1.7f;
float v_kd = 11.0f;

// SysConfig 生成: MOTOR PWM (TIMA0), timerCount=8000, 10kHz
#define MOTOR_MAX_DUTY (MOTOR_INST_CLK_FREQ / 10000) // =8000

/*
 * DRV8870 控制逻辑（无独立 PWM 引脚，PWM 直接加在 IN 上）:
 *   正转: IN1=PWM, IN2=0
 *   反转: IN1=0, IN2=PWM
 *   刹车: IN1=HIGH, IN2=HIGH
 *
 *   左电机: IN1=CCP0(PA8), IN2=CCP1(PA9)
 *   右电机: IN1=CCP3(PA12), IN2=CCP2(PA7)
 */
static void drv8870_set(uint8_t motor_id, int16_t pwm)
{
    uint32_t duty = (pwm > 0) ? (uint32_t)pwm : (uint32_t)(-pwm);
    if (duty > MOTOR_MAX_DUTY)
        duty = MOTOR_MAX_DUTY;

    if (motor_id == 1)
    {
        if (pwm > 0)
        { // 正转
            DL_Timer_setCaptureCompareValue(MOTOR_INST, duty, GPIO_MOTOR_C0_IDX);
            DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C1_IDX);
        }
        else if (pwm < 0)
        { // 反转
            DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C0_IDX);
            DL_Timer_setCaptureCompareValue(MOTOR_INST, duty, GPIO_MOTOR_C1_IDX);
        }
        else
        { // 刹车
            DL_Timer_setCaptureCompareValue(MOTOR_INST, MOTOR_MAX_DUTY, GPIO_MOTOR_C0_IDX);
            DL_Timer_setCaptureCompareValue(MOTOR_INST, MOTOR_MAX_DUTY, GPIO_MOTOR_C1_IDX);
        }
    }
    else
    {
        if (pwm > 0)
        { // 正转（C3=IN1=0, C2=IN2=PWM）
            DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C3_IDX);
            DL_Timer_setCaptureCompareValue(MOTOR_INST, duty, GPIO_MOTOR_C2_IDX);
        }
        else if (pwm < 0)
        { // 反转（C3=IN1=PWM, C2=IN2=0）
            DL_Timer_setCaptureCompareValue(MOTOR_INST, duty, GPIO_MOTOR_C3_IDX);
            DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C2_IDX);
        }
        else
        { // 刹车
            DL_Timer_setCaptureCompareValue(MOTOR_INST, MOTOR_MAX_DUTY, GPIO_MOTOR_C3_IDX);
            DL_Timer_setCaptureCompareValue(MOTOR_INST, MOTOR_MAX_DUTY, GPIO_MOTOR_C2_IDX);
        }
    }
}

//---------------------------------------------电机初始化设置为静止--------------------------------------------//
void MOTOR_Init(void)
{
    // SysConfig 已自动启动 MOTOR PWM (timerStartTimer=true)
    // 四通道初始占空比设为 0（惰行/静止）
    DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C0_IDX);
    DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C1_IDX);
    DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C2_IDX);
    DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C3_IDX);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

//-----------------------------------------------电机转动设置------------------------------------------------//
void motor_PWM(int leftPWM, int rightPWM)
{
    leftPWM = leftPWM * 4;
    rightPWM = rightPWM * 4;

    PWM_1_duty = (leftPWM > 0) ? leftPWM : -leftPWM;
    PWM_2_duty = (rightPWM > 0) ? rightPWM : -rightPWM;

    drv8870_set(1, leftPWM);
    drv8870_set(2, rightPWM);
}

//------------------------------------------------速度计算------------------------------------------------//
float cal_speed(uint8_t motor_id)
{
    if (motor_id == 1)
    {
        LEFT.speed = (float)tmp_a / ENCODE * PI * WHEEL_DIAMETER * 100;
        tmp_a = 0;
        return LEFT.speed;
    }
    if (motor_id == 2)
    {
        RIGHT.speed = (float)tmp_b / ENCODE * PI * WHEEL_DIAMETER * 100;
        tmp_b = 0;
        return RIGHT.speed;
    }
    return 0;
}

//---------------------------------------速度 PID（位置式）----------------------------------------//
static void V_I_lim(float limit, struct WHEEL *w)
{
    if (w->err_sum >= limit)
        w->err_sum = limit;
    else if (w->err_sum <= -limit)
        w->err_sum = -limit;
}

static int16_t SPEED_pid(float target, float actual, struct WHEEL *w)
{
    // 防暴冲：实际速度异常偏高时钳位（如模式切换后编码器堆积）
    if (actual > target * 3.0f)
        actual = target * 3.0f;
    else if (actual < -target * 3.0f)
        actual = -target * 3.0f;

    float err = target - actual;
    w->current_error = err;

    w->err_sum += err;
    V_I_lim(2000.0f, w);

    float err_dif = w->current_error - w->last_error;
    w->last_error = w->current_error;

    float out = v_kp * err + v_ki * w->err_sum + v_kd * err_dif;
    if (out > MOTOR_MAX_DUTY)
        return MOTOR_MAX_DUTY;
    if (out < -MOTOR_MAX_DUTY)
        return -MOTOR_MAX_DUTY;
    return (int16_t)out;
}

void MOTOR_PID(uint8_t motor_id)
{
    if (motor_brake_flag)
    {
        drv8870_set(motor_id, 0);
        return;
    }
    if (motor_id == 1)
    {
        float target = LEFT.target_speed > 0 ? LEFT.target_speed : -LEFT.target_speed;
        int16_t pwm_out = SPEED_pid(target, LEFT.speed, &LEFT);
        if (LEFT.target_speed < 0)
            pwm_out = -pwm_out;
        drv8870_set(1, pwm_out);
    }
    else if (motor_id == 2)
    {
        float target = RIGHT.target_speed > 0 ? RIGHT.target_speed : -RIGHT.target_speed;
        int16_t pwm_out = SPEED_pid(target, RIGHT.speed, &RIGHT);
        if (RIGHT.target_speed < 0)
            pwm_out = -pwm_out;
        drv8870_set(2, pwm_out);
    }
}

//-------------------------------------中断函数(电机PID)计算------------------------------------------------//
void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        static uint8_t pos_div = 0;
        // 位置环降频到 20Hz（每5次中断执行一次），速度环保持 100Hz
        if (++pos_div >= 2)
        {
            pos_div = 0;
            GYRO_StateMachine();
        }
        cal_speed(1);
        MOTOR_PID(1);
        cal_speed(2);
        MOTOR_PID(2);
        break;
    default:
        break;
    }
}