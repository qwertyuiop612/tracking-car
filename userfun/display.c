#include "display.h"
#include "uart.h"
#include "stdio.h"

// UART 显示协议 — 命令通过 UART0 发送到上层
// 上层接收后驱动实际 OLED（I2C）

static void disp_send(const char *cmd)
{
    UART_send_string(UART_0_INST, cmd);
    UART_send_char(UART_0_INST, '\n');
}

void DISP_Init(void)
{
    disp_send("@I");
}

void DISP_Clear(void)
{
    disp_send("@C");
}

void DISP_ColorTurn(uint8_t mode)
{
    char buf[8];
    sprintf(buf, "@T,%u", mode);
    disp_send(buf);
}

void DISP_DisplayTurn(uint8_t mode)
{
    char buf[8];
    sprintf(buf, "@D,%u", mode);
    disp_send(buf);
}

void DISP_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size)
{
    // 格式: @S,x,y,size,text
    // 文本中不能含换行符
    char buf[96];
    int len = sprintf(buf, "@S,%u,%u,%u,", x, y, size);
    // 直接拼接字符串，避免 vsnprintf 额外开销
    const char *src = str;
    char *dst = buf + len;
    int remain = sizeof(buf) - len - 1;
    while (*src && remain > 0) {
        if (*src == '\n' || *src == '\r') {
            *dst++ = ' ';
            remain--;
            src++;
        } else {
            *dst++ = *src++;
            remain--;
        }
    }
    *dst = '\0';
    disp_send(buf);
}

void DISP_Refresh(void)
{
    disp_send("@R");
}
