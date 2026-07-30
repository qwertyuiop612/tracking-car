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
#define CMD_BUF_SIZE 48

void UART_0_INST_IRQHandler()
{
	switch(DL_UART_getPendingInterrupt(UART_0_INST))
	{
		case DL_UART_IIDX_RX:
		{
			static char cmd_buf[CMD_BUF_SIZE];
			static uint8_t cmd_idx = 0;
			uint8_t rec = DL_UART_Main_receiveData(UART_0_INST);

			if (rec == '\n' || rec == '\r')
			{
				if (cmd_idx > 0)
				{
					cmd_buf[cmd_idx] = '\0';

					// @M：下层 car2 停车，同步冻结计时
					if (cmd_buf[0] == '@' && cmd_buf[1] == 'M')
					{
						extern volatile uint8_t marker_stopped;
						extern volatile uint32_t track_stop_ms;
						extern volatile uint32_t sys_tick_ms;
						if (!marker_stopped)
						{
							marker_stopped = 1;
							track_stop_ms = sys_tick_ms;
						}
					}
					// mode:x：接收来自 car2 的模式切换
					else if (cmd_buf[0] == 'm' && cmd_buf[1] == 'o' && cmd_buf[2] == 'd' &&
							 cmd_buf[3] == 'e' && cmd_buf[4] == ':')
					{
						extern volatile int status;
						char m = cmd_buf[5];
						if (m >= '0' && m <= '1')
							status = m - '0';
					}
					cmd_idx = 0;
				}
			}
			else if (cmd_idx < CMD_BUF_SIZE - 1)
			{
				cmd_buf[cmd_idx++] = (char)rec;
			}
			break;
		}
		default:
		break;
		}
}