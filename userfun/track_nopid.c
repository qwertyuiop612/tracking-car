#include "ti_msp_dl_config.h"
#include "sensor.h"
#include "track_nopid.h"
#include "motor.h"
#include "uart.h"

extern volatile uint32_t sys_tick_ms;
extern volatile uint8_t tuner_report_enable;
extern int sensor[8];
extern float raw_trace;

//===========================================
// 开环循迹（无 PID，枚举传感器组合直接配差速）
// sensor[0]~[7] 从左到右
//===========================================

void track_nopid()
{
    sensor_detect(); // 更新 sensor[] 和 raw_trace

    int turn;

    // ====== 情况1：直道（中间 2~4 个传感器检测到） ======
    if ((sensor[3] || sensor[4]) && !sensor[0] && !sensor[1] && !sensor[6] && !sensor[7])
    {
        // 子情况：微偏修正（仅用相邻传感器微调）
        if (sensor[3] && sensor[4])
            turn = 0;                       // 正中直行
        else if (sensor[3] && sensor[2])
            turn = 60;                      // 偏左微调
        else if (sensor[4] && sensor[5])
            turn = -60;                     // 偏右微调
        else if (sensor[3] && !sensor[4])
            turn = 100;                     // 左偏
        else if (sensor[4] && !sensor[3])
            turn = -100;                    // 右偏
        else
            turn = 0;
    }
    // ====== 情况2：弯道（仅单侧传感器检测到） ======
    // 距中心偏移量 → 差速递增
    else if (sensor[0] || sensor[1] || sensor[2])
    {
        // 左侧有信号 → 右转
        if      (sensor[0])                 turn = 400;
        else if (sensor[1] && sensor[2])    turn = 320;
        else if (sensor[1])                 turn = 260;
        else if (sensor[2] && sensor[3])    turn = 180;
        else if (sensor[2])                 turn = 140;
        else                                turn = 400; // 兜底
    }
    else if (sensor[5] || sensor[6] || sensor[7])
    {
        // 右侧有信号 → 左转
        if      (sensor[7])                 turn = -400;
        else if (sensor[6] && sensor[5])    turn = -320;
        else if (sensor[6])                 turn = -260;
        else if (sensor[5] && sensor[4])    turn = -180;
        else if (sensor[5])                 turn = -140;
        else                                turn = -400; // 兜底
    }
    else
    {
        // 完全丢线：保持最后状态（raw_trace 已由 sensor_detect 设好）
        turn = (raw_trace > 4.5f) ? -400 : 400;
    }

    LEFT.target_speed  = 400 - turn;
    RIGHT.target_speed = 400 + turn;

    // LLM-PID-Tuner 上报
    if (tuner_report_enable)
    {
        static uint8_t report_cnt = 0;
        if (++report_cnt >= 5)
        {
            report_cnt = 0;
            uart_printf("%lu,4.5,%.1f,%d,%.1f,0.00,0.00,0.00\r\n",
                        sys_tick_ms, raw_trace, turn, 4.5f - raw_trace);
        }
    }
}
