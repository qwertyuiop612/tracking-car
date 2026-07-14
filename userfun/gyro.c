#include "gyro.h"

volatile uint8_t  gyro_rx_done   = 0;
volatile int16_t  gyro_angle_raw = 0;
volatile int16_t  gyro_dps_raw   = 0;
volatile uint32_t gyro_frame_count = 0;

//-------------------------- Modbus CRC16 计算 ---------------------------//
// 多项式 0xA001，初始值 0xFFFF
static uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

//--------------------------发送 Modbus 查询命令---------------------------//
// 发送读保持寄存器命令: [Addr, 0x03, RegH, RegL, CountH, CountL, CRC_L, CRC_H]
// 读取 2 个寄存器（角度 + 角速度）
void GYRO_SendQuery(void)
{
    uint8_t cmd[6] = {
        GYRO_SLAVE_ADDR,            // 从机地址 0x0A
        0x03,                       // 功能码：读保持寄存器
        (GYRO_REG_ANGLE >> 8),      // 起始寄存器地址高字节
        (GYRO_REG_ANGLE & 0xFF),    // 起始寄存器地址低字节
        0x00,                       // 寄存器数量高字节
        0x02                        // 寄存器数量低字节（读2个寄存器=4字节）
    };
    uint16_t crc = CRC16(cmd, 6);

    uint8_t tx_buf[8];
    for (int i = 0; i < 6; i++) tx_buf[i] = cmd[i];
    tx_buf[6] = crc & 0xFF;         // CRC 低字节在前
    tx_buf[7] = crc >> 8;           // CRC 高字节在后

    UART_send_buffer(GYRO_INST, tx_buf, 8);
}

//-----------------------------状态机解析接收帧----------------------------//
// 帧格式: [0x0A, 0x03, 0x04, AngleH, AngleL, DPS_H, DPS_L, CRC_L, CRC_H]
static uint8_t  rx_state = 0;
static uint8_t  rx_buf[GYRO_FRAME_LEN];
static uint8_t  rx_idx  = 0;

static void GYRO_ParseFrame(uint8_t byte)
{
    switch (rx_state) {
    case 0:     // 等待帧头 0x0A
        if (byte == GYRO_SLAVE_ADDR) {
            rx_buf[0] = byte;
            rx_idx = 1;
            rx_state = 1;
        }
        break;
    case 1:     // 等待功能码 0x03
        if (byte == 0x03) {
            rx_buf[1] = byte;
            rx_state = 2;
        } else {
            rx_state = 0;
        }
        break;
    case 2:     // 等待字节数 0x04
        if (byte == 0x04) {
            rx_buf[2] = byte;
            rx_state = 3;
        } else {
            rx_state = 0;
        }
        break;
    case 3:     // 接收剩余 6 字节（数据 + CRC），存满 9 字节后校验
        rx_buf[rx_idx++] = byte;
        if (rx_idx >= GYRO_FRAME_LEN) {
            // 对前 7 字节计算 CRC
            uint16_t crc_calc = CRC16(rx_buf, GYRO_DATA_LEN);
            uint16_t crc_recv = rx_buf[7] | (rx_buf[8] << 8);
            if (crc_calc == crc_recv) {
                // 大端模式提取角度和角速度
                gyro_angle_raw = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
                gyro_dps_raw   = (int16_t)((rx_buf[5] << 8) | rx_buf[6]);
                gyro_rx_done = 1;
                gyro_frame_count++;
            }
            rx_state = 0;
        }
        break;
    default:
        rx_state = 0;
        break;
    }
}

//----------------------------GYRO UART 中断处理----------------------------//
void GYRO_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(GYRO_INST)) {
    case DL_UART_IIDX_RX: {
        uint8_t rec = DL_UART_receiveData(GYRO_INST);
        GYRO_ParseFrame(rec);
        break;
    }
    default:
        break;
    }
}

//----------------------------------初始化---------------------------------//
void GYRO_Init(void)
{
    // 发送 Modbus 查询命令，触发陀螺仪返回数据
    GYRO_SendQuery();
}

//------------------------------角度归一化 ±180°---------------------------//
static float WrapAngle180(float angle)
{
    while (angle >=  180.0f) angle -= 360.0f;
    while (angle <  -180.0f) angle += 360.0f;
    return angle;
}

//-----------------------------获取解析后的数据----------------------------//
int GYRO_GetData(GyroData_t *data)
{
    if (!gyro_rx_done || !data) return 0;

    // 关中断拷贝原始值，防止被中断更新
    __disable_irq();
    data->angle_raw = gyro_angle_raw;
    data->dps_raw   = gyro_dps_raw;
    __enable_irq();

    // 转换为浮点值
    data->angle_deg = WrapAngle180(data->angle_raw / GYRO_ANGLE_SCALE);
    data->dps       = data->dps_raw / GYRO_DPS_SCALE;

    gyro_rx_done = 0;   // 清除标志
    return 1;
}