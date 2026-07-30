#include "stepper.h"
#include "uart.h"
#include "stdio.h"

// 步进电机通过 UART3 (GYRO_INST) 控制
// 协议格式（通用步进驱动）: 单行 ASCII，\r\n 结尾

void STEPPER_Init(void)
{
    // UART3 由 SysConfig 初始化，此处无需额外配置
}

void STEPPER_SetSpeed(int rpm)
{
    char buf[16];
    sprintf(buf, "S%d\r\n", rpm);
    UART_send_string(GYRO_INST, buf);
}

void STEPPER_Move(int steps)
{
    char buf[16];
    sprintf(buf, "M%d\r\n", steps);
    UART_send_string(GYRO_INST, buf);
}

void STEPPER_Stop(void)
{
    UART_send_string(GYRO_INST, "STOP\r\n");
}
