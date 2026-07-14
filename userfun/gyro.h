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

typedef struct {
    int16_t angle_raw;
    int16_t dps_raw;
    float angle_deg;
    float dps;
} GyroData_t;

extern volatile uint8_t gyro_rx_done;
extern volatile int16_t gyro_angle_raw;
extern volatile int16_t gyro_dps_raw;

void GYRO_Init(void);
void GYRO_SendQuery(void); // 发送 Modbus 查询命令（可选，模块自动发送时不需要）
int GYRO_GetData(GyroData_t *data);

#endif