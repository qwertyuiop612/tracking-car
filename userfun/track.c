#include "ti_msp_dl_config.h"
#include "default.h"
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"
#include "uart.h"
#include "oled.h"
#include "stdio.h"

extern volatile uint32_t sys_tick_ms;
extern int flag;

// 循迹持续时间追踪
volatile uint32_t track_start_ms = 0;
volatile uint32_t track_stop_ms  = 0;  // 停车时刻，用于冻结计时

// 标识停车状态机
volatile uint8_t marker_stopped = 0;
extern uint8_t braking; // 急刹中（GYRO_SetMode 重置）

// LLM-PID-Tuner 上报开关，默认开启（按钮未焊接，直接上报）
volatile uint8_t tuner_report_enable = 1;

float p_kp = 70.0f;
float p_ki = 1.3f;
float p_kd = 485.0f;      // 实测需此值压制直道微偏和出弯摆动
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

    // 积分分离：仅丢线时清零（|误差|>3.5意味着传感器接近边缘）
    if (p_err > 3.5f || p_err < -3.5f)
        p_err_sum = 0;
    else
        p_err_sum += p_err;
    I_lim(30.0);

    p_err_dif = p_err - p_last_err;
    p_last_err = p_err;

    // 微小误差时半 Kd，防直道抽动；大误差全 Kd，保弯道响应
    float kd_scaled = (p_err > -1.0f && p_err < 1.0f) ? p_kd * 0.75f : p_kd;
    return p_kp * p_err + p_ki * p_err_sum + kd_scaled * p_err_dif;
}

// 急刹状态（GYRO_SetMode 中重置）
uint8_t  braking = 0;
uint32_t brake_start_ms = 0;

//--------------------------------------------循迹------------------------------------------------//
#define BASE_SPEED 400.0f
#define CRAWL_SPEED 100.0f
#define RAMP_UP_MS 2000     // 加速段
#define SLOW_START_MS 15000 // 开始减速时间点
#define SLOW_MS 1000        // 减速持续时间
#define STOP_START_MS 18000 // 开始停车时间点
#define STOP_MS 500         // 停车斜坡
#define BRAKE_MS 150        // 检测到停车线后的急刹时间
#define TUNER_REPORT_DIV 5

void track()
{
    float trace_val = sensor_detect();
    float turn = PLACE_pid(4.5f, trace_val);

    if (turn > 400.0f) turn = 400.0f;
    if (turn < -400.0f) turn = -400.0f;

    uint32_t elapsed = sys_tick_ms - track_start_ms;
    float base;

    // 检测停车线 → 触发急刹（防超线）
    if (flag && !braking && !marker_stopped)
    {
        braking = 1;
        brake_start_ms = sys_tick_ms;
    }

    if (marker_stopped)
    {
        base = 0.0f;
    }
    else if (braking)
    {
        uint32_t bt = sys_tick_ms - brake_start_ms;
        if (bt >= BRAKE_MS)
        {
            base = 0.0f;
            marker_stopped = 1;
            track_stop_ms = sys_tick_ms;  // 冻结计时
        }
        else
        {
            // 按当前阶段取速度，线性急刹到 0
            float cur;
            if (elapsed < RAMP_UP_MS)
                cur = 150.0f + (BASE_SPEED - 150.0f) * elapsed / RAMP_UP_MS;
            else if (elapsed < SLOW_START_MS)
                cur = BASE_SPEED;
            else if (elapsed < SLOW_START_MS + SLOW_MS)
                cur = BASE_SPEED + (CRAWL_SPEED - BASE_SPEED) * (elapsed - SLOW_START_MS) / SLOW_MS;
            else if (elapsed < STOP_START_MS)
                cur = CRAWL_SPEED;
            else
                cur = CRAWL_SPEED * (STOP_MS - (elapsed - STOP_START_MS)) / STOP_MS;
            base = cur * (BRAKE_MS - bt) / BRAKE_MS;
        }
    }
    else if (elapsed < RAMP_UP_MS)
    {
        base = BASE_SPEED * elapsed / RAMP_UP_MS;
        if (base < 150.0f)
            base = 150.0f;
    }
    else if (elapsed < SLOW_START_MS)
    {
        base = BASE_SPEED;
    }
    else if (elapsed < SLOW_START_MS + SLOW_MS)
    {
        uint32_t t = elapsed - SLOW_START_MS;
        base = BASE_SPEED + (CRAWL_SPEED - BASE_SPEED) * t / SLOW_MS;
    }
    else if (elapsed < STOP_START_MS)
    {
        base = CRAWL_SPEED;
    }
    else if (elapsed < STOP_START_MS + STOP_MS)
    {
        uint32_t t = elapsed - STOP_START_MS;
        base = CRAWL_SPEED * (STOP_MS - t) / STOP_MS;
    }
    else
    {
        base = 0.0f;
        marker_stopped = 1;
        track_stop_ms = sys_tick_ms;      // 冻结计时
    }

    LEFT.target_speed = base - turn;
    RIGHT.target_speed = base + turn;

    // LLM-PID-Tuner CSV 上报
    if (tuner_report_enable)
    {
        static uint8_t report_cnt = 0;
        if (++report_cnt >= TUNER_REPORT_DIV)
        {
            report_cnt = 0;
            uart_printf("%lu,4.5,%.1f,%.0f,%.1f,%.2f,%.2f,%.2f\r\n",
                        sys_tick_ms,      // timestamp
                        raw_trace,        // input
                        turn,             // pwm
                        4.5f - raw_trace, // error
                        p_kp, p_ki, p_kd);
        }
    }
}
