#include "uart.h"
#include "stdio.h"
#include "stdarg.h"

//----------------------------------------格式化打印（用于VOFA+调试）-------------------------//
void uart_printf(const char *fmt, ...)
{
	static char buf[128];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	UART_send_string(UART_0_INST, buf);
}

//------------------------------------------发送单个字符------------------------------------//
void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
	// 等待 TX FIFO 非满（带超时，防止死锁）
	volatile uint32_t timeout = 100000;
	while (DL_UART_isTXFIFOFull(uart) && --timeout)
		;
	DL_UART_Main_transmitData(uart, chr);
}

//--------------------------------------------发送字符串------------------------------------//
void UART_send_string(UART_Regs *uart, const char *str)
{
	while (*str)
	{
		if (*str == '\n')
			UART_send_char(uart, '\r'); // VOFA+ 需要 \r\n
		UART_send_char(uart, (uint8_t)*str);
		str++;
	}
}

//----------------------------------------发送字节数组（陀螺仪通信）-------------------------//
void UART_send_buffer(UART_Regs *uart, const uint8_t *buf, const uint8_t len)
{
	for (int i = 0; i < len; i++)
	{
		UART_send_char(uart, buf[i]);
	}
}

//------------------------------------------中断接收字符------------------------------------//
void UART_0_INST_IRQHandler()
{
	switch(DL_UART_getPendingInterrupt(UART_0_INST))
	{
		case DL_UART_IIDX_RX:
			uint8_t rec = DL_UART_Main_receiveData(UART_0_INST);
			UART_send_char(UART_0_INST, rec);
			break;
		default:
		break;
	}
}