#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>

// 循迹 PID 参数（可被上位机修改）
extern float p_kp;
extern float p_ki;
extern float p_kd;

// LLM-PID-Tuner 串口上报开关
extern volatile uint8_t tuner_report_enable;

// 循迹持续时间追踪（进入循迹模式时的 sys_tick_ms）
extern volatile uint32_t track_start_ms;
extern volatile uint32_t track_stop_ms;

// 标识停车状态，1=停车中，仅模式切换（切出再切回循迹）时清除
extern volatile uint8_t marker_stopped;

void track(void);

#endif // TRACK_H