#include "gyro.h"
#include "motor.h"
#include "track.h"
#include "uart.h"

extern volatile uint32_t sys_tick_ms;
extern float p_last_err;
extern float p_err_sum;
extern uint8_t braking;

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
            rx_buf[rx_idx++] = byte;
            rx_state = 2;
        } else {
            rx_state = 0;
        }
        break;
    case 2:     // 等待字节数 0x04
        if (byte == 0x04) {
            rx_buf[rx_idx++] = byte;
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
    case DL_UART_IIDX_OVERRUN_ERROR:
    {
        DL_UART_clearInterruptStatus(GYRO_INST, DL_UART_IIDX_OVERRUN_ERROR);
        (void)DL_UART_receiveData(GYRO_INST);
        break;
    }
    default:
        break;
    }
}

//---------------------------发送 Modbus 查询命令(可选)---------------------//
// 发送: [Addr, 0x03, RegH, RegL, 0x00, 0x02, CRC_L, CRC_H]
// 0x003D 为常用角度寄存器起始地址，需根据实际模块手册确认
#define GYRO_REG_ANGLE_START 0x003D

void GYRO_SendQuery(void)
{
    uint8_t cmd[6] = {
        GYRO_SLAVE_ADDR,
        0x03,
        (GYRO_REG_ANGLE_START >> 8),
        (GYRO_REG_ANGLE_START & 0xFF),
        0x00,
        0x02};
    uint16_t crc = CRC16(cmd, 6);

    uint8_t tx_buf[8];
    for (int i = 0; i < 6; i++)
        tx_buf[i] = cmd[i];
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = crc >> 8;

    UART_send_buffer(GYRO_INST, tx_buf, 8);
}

//----------------------------------初始化---------------------------------//
// SYSCFG_DL_init 已配置好 UART3 + RX 中断，这里只需开 NVIC
void GYRO_Init(void)
{
    NVIC_ClearPendingIRQ(GYRO_INST_INT_IRQN);
    NVIC_EnableIRQ(GYRO_INST_INT_IRQN);
}

//------------------------------角度归一化 0~360°---------------------------//
// M0+ 无硬件 FPU，用整数运算归一化更可靠
// GYRO_ANGLE_SCALE = 9.77, 180° ≈ 1759 raw, 360° ≈ 3517 raw
#define RAW_HALF_CIRCLE 1759
#define RAW_FULL_CIRCLE 3517

static int16_t WrapRaw360(int16_t raw)
{
    while (raw >= RAW_FULL_CIRCLE)
        raw -= RAW_FULL_CIRCLE;
    while (raw < 0)
        raw += RAW_FULL_CIRCLE;
    return raw;
}

// 整数最短路径差：返回 t - c，范围 (-half, half]
static int16_t RawShortestDiff(int16_t t, int16_t c)
{
    int16_t d = t - c;
    while (d > RAW_HALF_CIRCLE)
        d -= RAW_FULL_CIRCLE;
    while (d < -RAW_HALF_CIRCLE)
        d += RAW_FULL_CIRCLE;
    return d;
}

//-----------------------------获取解析后的数据----------------------------//
int GYRO_GetData(GyroData_t *data)
{
    if (!gyro_rx_done || !data) return 0;

    // 关中断拷贝原始值，防止被中断更新
    __disable_irq();
    data->angle_raw = gyro_angle_raw;
    data->dps_raw   = gyro_dps_raw;
    gyro_rx_done = 0; // 先清标志再开中断，避免竞态

    // 转换为浮点值（M0+ 无硬件 FPU，必须在关中断下完成，防止 PID ISR 重入破坏 FP 上下文）
    data->angle_deg = (float)WrapRaw360(data->angle_raw) / GYRO_ANGLE_SCALE;
    data->dps       = data->dps_raw / GYRO_DPS_SCALE;
    __enable_irq();

    return 1;
}

//------------------------------角度 PID ---------------------------------//
// 参数可通过 UART "SET P:x I:y D:z" 命令调整
float a_kp = 3.0f;
float a_ki = 0.01f;
float a_kd = 18.0f;
float angle_target = 0.0f; // 目标角度，通过 UART "angle.target=x" 设置

static float a_err = 0.0f;
static float a_last_err = 0.0f;
static float a_err_sum = 0.0f;

// 积分限幅
static void A_IntegralLimit(float limit)
{
    if (a_err_sum > limit)
        a_err_sum = limit;
    if (a_err_sum < -limit)
        a_err_sum = -limit;
}

// 角度 PID（M0+ 可靠版：所有角度差用整数计算，避免浮点比较失败）
// 返回转向量，正值=角度增大方向
static float AnglePID(float target_deg, float current_deg)
{
    // 转为 raw 整数做可靠比较
    int16_t t_raw = (int16_t)(target_deg * GYRO_ANGLE_SCALE + 0.5f);
    int16_t c_raw = (int16_t)(current_deg * GYRO_ANGLE_SCALE + 0.5f);

    // 整数最短路径误差
    int16_t err_raw = RawShortestDiff(t_raw, c_raw);
    a_err = (float)err_raw / GYRO_ANGLE_SCALE;

    // 整数积分分离（90° = 879 raw）
    if (err_raw > 879 || err_raw < -879)
        a_err_sum = 0.0f;
    else
        a_err_sum += a_err;
    A_IntegralLimit(100.0f);

    // D 项：整数差分，避免 +179↔-179 跨越导致浮点 D 爆表
    int16_t last_raw = (int16_t)(a_last_err * GYRO_ANGLE_SCALE + 0.5f);
    int16_t diff_raw = RawShortestDiff(err_raw, last_raw);
    float diff = (float)diff_raw / GYRO_ANGLE_SCALE;
    a_last_err = a_err;

    return a_kp * a_err + a_ki * a_err_sum + a_kd * diff;
}

//------------------------------运行模式管理-----------------------------//
static volatile int g_mode = 0; // 默认角度调参模式，开机显示 Start

void GYRO_SetMode(int new_mode)
{
    if (new_mode == MODE_ANGLE_TUNE || new_mode == MODE_TRACK)
    {
        g_mode = new_mode;
        // 切换模式时清零 PID 历史，防突变
        a_err = 0.0f;
        a_last_err = 0.0f;
        a_err_sum = 0.0f;
        p_last_err = 0.0f;
        p_err_sum = 0.0f;

        // 模式 0：停车
        if (new_mode == MODE_ANGLE_TUNE)
        {
            LEFT.target_speed = 0.0f;
            RIGHT.target_speed = 0.0f;
        }
        // 模式 1：记录循迹开始时间，清除标识停车标志
        else if (new_mode == MODE_TRACK)
        {
            LEFT.target_speed = 0.0f;
            RIGHT.target_speed = 0.0f;
            track_start_ms = sys_tick_ms;
            marker_stopped = 0;
            braking = 0;
            extern int flag;
            flag = 0;
        }
        // 清零电机 PID 状态，防止旧积分/微分残留
        LEFT.err_sum = 0.0f;
        LEFT.last_error = 0.0f;
        LEFT.speed = 0.0f;
        RIGHT.err_sum = 0.0f;
        RIGHT.last_error = 0.0f;
        RIGHT.speed = 0.0f;
        // 清零编码器累计，防止模式切换期间脉冲堆积
        extern uint32_t tmp_a;
        extern uint32_t tmp_b;
        tmp_a = 0;
        tmp_b = 0;
    }
}

int GYRO_GetMode(void)
{
    return g_mode;
}

//------------------------------状态机（在 PID 定时器 ISR 中调用）------------------//
// 替代原 track() 的位置调用：motor.c MOTOR_PID_INST_IRQHandler → 位置环 → 这里
#define ANGLE_TUNE_REPORT_DIV 5 // 上报分频（50Hz 位置环 → 10Hz 上报）

void GYRO_StateMachine(void)
{
    static GyroData_t data;
    static uint8_t report_cnt = 0;

    if (g_mode == MODE_ANGLE_TUNE)
    {
        // ---- 模式 0：原地角度 PID 调参（速度=0，仅转向）----
        if (GYRO_GetData(&data))
        {
            float turn = AnglePID(angle_target, data.angle_deg);

            // 限幅
            if (turn > 400.0f)
                turn = 400.0f;
            if (turn < -400.0f)
                turn = -400.0f;

            // 差速原地转向（角度增大方向 = turn 正值方向）
            LEFT.target_speed = turn;
            RIGHT.target_speed = -turn;

            // CSV 上报
            if (tuner_report_enable)
            {
                if (++report_cnt >= ANGLE_TUNE_REPORT_DIV)
                {
                    report_cnt = 0;
                    // 整数最短路径误差（M0+ 浮点比较不可靠）
                    int16_t t_raw = (int16_t)(angle_target * GYRO_ANGLE_SCALE + 0.5f);
                    int16_t c_raw = (int16_t)(data.angle_deg * GYRO_ANGLE_SCALE + 0.5f);
                    float csv_err = (float)RawShortestDiff(t_raw, c_raw) / GYRO_ANGLE_SCALE;

                    uart_printf("%lu,%.1f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f\r\n",
                                sys_tick_ms,
                                angle_target,   // setpoint
                                data.angle_deg, // input
                                turn,           // pwm
                                csv_err,        // error
                                a_kp, a_ki, a_kd);
                }
            }
        }
    }
    else // MODE_TRACK
    {
        // ---- 模式 1：循迹（原 track.c 逻辑）----
        track();
    }
}