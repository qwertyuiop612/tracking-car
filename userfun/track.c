#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"
#include "uart.h"

extern int sensor[7];
extern volatile uint32_t sys_tick_ms;

// LLM-PID-Tuner 上报开关，默认开启（按钮未焊接，直接上报）
volatile uint8_t tuner_report_enable = 1;

float p_kp = 60.0f;
float p_ki = 0.2f;
float p_kd = 1000.0f;     // 阻尼比 32/√17≈7.8，小幅加速 8.82522
float p_err;              // 当前误差
float p_last_err;         // 上次误差
float p_err_sum;          // 累计误差
float p_err_dif;          // 误差差值

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

    // 积分分离：仅丢线时清零（|误差|>3意味着传感器接近边缘）
    if (p_err > 3.0f || p_err < -3.0f)
        p_err_sum = 0;
    else
        p_err_sum += p_err;
    I_lim(80.0);

    p_err_dif = p_err - p_last_err;
    p_last_err = p_err;

    return p_kp * p_err + p_ki * p_err_sum + p_kd * p_err_dif;
}

//--------------------------------------------循迹------------------------------------------------//
// LLM-PID-Tuner 串口上报间隔：每 TUNER_REPORT_DIV 次 track() 发一帧
#define TUNER_REPORT_DIV 5
#define PIVOT_SPEED 150  // 原地转向速度（降低防过冲）
#define PIVOT_COOLDOWN 5 // 退出后冷却周期数（50Hz下=100ms）

void track()
{
    float trace_val = sensor_detect();

    // ====== 直角检测：原地转向 ======
    // 仅用两侧各 2 个最外侧传感器（0/1 和 5/6），中间传感器 2/3/4 不参与
    uint8_t left_far = (sensor[0] != 0);                      // 最左
    uint8_t right_far = (sensor[6] != 0);                     // 最右
    uint8_t left_side = (sensor[0] != 0) + (sensor[1] != 0);  // 左侧 2 个
    uint8_t right_side = (sensor[5] != 0) + (sensor[6] != 0); // 右侧 2 个
    uint8_t center_ok = (sensor[3] != 0);

    static int8_t pivot_dir = 0;    // 0=无, 1=左转, -1=右转
    static uint8_t cooldown = 0;    // 退出后冷却计数
    static uint8_t just_exited = 0; // 刚退出 pivot，首帧跳过 EMA
    static uint8_t pend_cnt = 0;    // 持续判定计数
    static int8_t pend_dir = 0;     // 待判定方向

    if (cooldown > 0)
    {
        cooldown--;
    }
    else if (pivot_dir != 0)
    {
        if (center_ok)
        {
            pivot_dir = 0;
            cooldown = PIVOT_COOLDOWN;
            just_exited = 1;
            p_last_err = 0;
            p_err_sum = 0;
        }
    }
    else if (left_far && right_side == 0)
    {
        if (pend_dir == 1)
        {
            if (++pend_cnt >= 2)
            {
                pivot_dir = 1;
                pend_cnt = 0;
                pend_dir = 0;
            }
        }
        else
        {
            pend_dir = 1;
            pend_cnt = 1;
        }
    }
    else if (right_far && left_side == 0)
    {
        if (pend_dir == -1)
        {
            if (++pend_cnt >= 2)
            {
                pivot_dir = -1;
                pend_cnt = 0;
                pend_dir = 0;
            }
        }
        else
        {
            pend_dir = -1;
            pend_cnt = 1;
        }
    }
    else
    {
        pend_cnt = 0; // 条件不满足，重置判定
        pend_dir = 0;
    }

    float turn;
    if (pivot_dir != 0)
    {
        // 原地转向：直接设定相反目标速度，不走 PID
        if (pivot_dir == 1)
        {
            LEFT.target_speed = -PIVOT_SPEED;
            RIGHT.target_speed = PIVOT_SPEED;
        }
        else
        {
            LEFT.target_speed = PIVOT_SPEED;
            RIGHT.target_speed = -PIVOT_SPEED;
        }
        turn = (pivot_dir == 1) ? 300.0f : -300.0f; // 上报用
    }
    else
    {
        // 正常 PID 循迹
        turn = PLACE_pid(4, trace_val);

        // 限制最大转向幅度
        if (turn > 300.0f)
            turn = 300.0f;
        if (turn < -300.0f)
            turn = -300.0f;

        // EMA 平滑目标速度，防止 turn 突变导致转速跳变
        // 刚退出 pivot 时直接赋值，不清残留的 ±150 旧值
        float target_L = 400.0f - turn;
        float target_R = 400.0f + turn;
        if (just_exited)
        {
            LEFT.target_speed = target_L;
            RIGHT.target_speed = target_R;
            just_exited = 0;
        }
        else
        {
            LEFT.target_speed = LEFT.target_speed * 0.6f + target_L * 0.4f;
            RIGHT.target_speed = RIGHT.target_speed * 0.6f + target_R * 0.4f;
        }
    }

    // LLM-PID-Tuner CSV 上报: timestamp_ms,setpoint,input,pwm,error,p,i,d
    if (tuner_report_enable)
    {
        static uint8_t report_cnt = 0;
        if (++report_cnt >= TUNER_REPORT_DIV)
        {
            report_cnt = 0;
            uart_printf("%lu,4.0,%.1f,%.0f,%.1f,%.2f,%.2f,%.2f\r\n",
                        sys_tick_ms,      // timestamp
                        raw_trace,        // input（原始读数，出线=0）
                        turn,             // pwm（PID 输出）
                        4.0f - raw_trace, // error（基于原始值）
                        p_kp, p_ki, p_kd);
        }
    }
}
