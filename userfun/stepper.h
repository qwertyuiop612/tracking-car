#ifndef STEPPER_H
#define STEPPER_H

#include "ti_msp_dl_config.h"

// 步进电机控制 — 通过 UART3 (GYRO_INST) 发送指令
void STEPPER_Init(void);
void STEPPER_SetSpeed(int rpm);
void STEPPER_Move(int steps);     // 正=前进, 负=后退
void STEPPER_Stop(void);

#endif
