#ifndef GYRO_H
#define GYRO_H

#include "ti_msp_dl_config.h"
#include "uart.h"
#include <stdint.h>

// 帧结构: [0x0A, 0x03, 0x04, AngleH, AngleL, DPS_H, DPS_L, CRC_L, CRC_H]
#define GYRO_SLAVE_ADDR 0x0A
#define GYRO_FRAME_LEN 9
#define GYRO_DATA_LEN 7 // CRC 覆盖前 7 字节

#define GYRO_ANGLE_SCALE 9.77f // raw / 9.77 = 度
#define GYRO_DPS_SCALE 100.0f  // raw / 100  = 度/秒

// 运行模式
#define MODE_ANGLE_TUNE 0 // AI 辅助调整角度 PID
#define MODE_TRACK 1      // 循迹模式

typedef struct {
    int16_t angle_raw;
    int16_t dps_raw;
    float angle_deg;
    float dps;
} GyroData_t;

// ----- 角度 PID 参数（可通过 UART SET 命令调整）-----
extern float a_kp;
extern float a_ki;
extern float a_kd;

// 角度目标值（可通过 UART "angle.target=x" 设置）
extern float angle_target;

// LLM-PID-Tuner 上报开关
extern volatile uint8_t tuner_report_enable;

extern volatile uint8_t gyro_rx_done;
extern volatile int16_t gyro_angle_raw;
extern volatile int16_t gyro_dps_raw;

void GYRO_Init(void);
void GYRO_SendQuery(void);
int GYRO_GetData(GyroData_t *data);

// 状态机：在 PID 定时器 ISR 中调用，替代原 track() 的位置
void GYRO_StateMachine(void);

// 设置运行模式（供 UART 命令 mode:x 调用）
void GYRO_SetMode(int new_mode);
int GYRO_GetMode(void);

#endif