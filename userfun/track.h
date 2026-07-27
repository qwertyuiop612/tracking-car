#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>

// 循迹 PID 参数（可被上位机修改）
extern float p_kp;
extern float p_ki;
extern float p_kd;

// LLM-PID-Tuner 串口上报开关
extern volatile uint8_t tuner_report_enable;

void track(void);

#endif // TRACK_H