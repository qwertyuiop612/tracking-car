#ifndef GYRO_H
#define GYRO_H

#include "ti_msp_dl_config.h"
#include "default.h"
#include "uart.h"
#include <stdint.h>

// Modbus RTU 帧结构: [0x0A, 0x03, 0x04, AngleH, AngleL, DPS_H, DPS_L, CRC_L, CRC_H]
#define GYRO_SLAVE_ADDR     0x0A        // 陀螺仪从机地址
#define GYRO_FRAME_LEN      9           // 完整帧长度
#define GYRO_DATA_LEN       7           // CRC 计算的数据长度(前7字节)

// 角度比例因子: 实测旋转90°对应原始值变化879，比例因子=879/90≈9.77
#define GYRO_ANGLE_SCALE    9.77f
// 角速度比例因子: 原始值÷100得到实际dps
#define GYRO_DPS_SCALE      100.0f

// Modbus 寄存器地址（根据实际模块确认）
#define GYRO_REG_ANGLE      0x003D      // 角度寄存器起始地址

// 解析后的陀螺仪数据结构
typedef struct {
    int16_t angle_raw;      // 原始角度值
    int16_t dps_raw;        // 原始角速度值
    float angle_deg;        // 角度(度)，归一化到±180°
    float dps;              // 角速度(度/秒)
} GyroData_t;

extern volatile uint8_t  gyro_rx_done;      // 新数据标志
extern volatile int16_t  gyro_angle_raw;    // 最新原始角度
extern volatile int16_t  gyro_dps_raw;      // 最新原始角速度

void GYRO_Init(void);
void GYRO_SendQuery(void);                  // 发送一次查询命令
int  GYRO_GetData(GyroData_t *data);        // 获取解析后的数据

#endif