#include "uart.h"
#include "stdio.h"
#include "stdarg.h"
#include "track.h"
#include "gyro.h"

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
// LLM-PID-Tuner 命令解析（轻量级手动解析，不依赖 sscanf/strtof）
#define CMD_BUF_SIZE 48

// 手动解析 float，支持 "-?[0-9]+\.?[0-9]*"，返回解析后指针
static const char *parse_float(const char *s, float *out)
{
	int sign = 1;
	if (*s == '-')
	{
		sign = -1;
		s++;
	}
	float int_part = 0.0f;
	while (*s >= '0' && *s <= '9')
	{
		int_part = int_part * 10.0f + (float)(*s - '0');
		s++;
	}
	if (*s == '.')
	{
		s++;
		float frac = 0.0f, div = 1.0f;
		while (*s >= '0' && *s <= '9')
		{
			frac = frac * 10.0f + (float)(*s - '0');
			div *= 10.0f;
			s++;
		}
		int_part += frac / div;
	}
	*out = sign * int_part;
	return s;
}

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
					const char *s = cmd_buf;

					// ---- 命令：mode:x（切换运行模式）----
					if (s[0] == 'm' && s[1] == 'o' && s[2] == 'd' && s[3] == 'e' && s[4] == ':')
					{
						s += 5;
						if (s[0] >= '0' && s[0] <= '9' && s[1] == '\0')
						{
							GYRO_SetMode(s[0] - '0');
						}
					}
					// ---- 命令：angle.target=x（设置角度 PID 目标值）----
					else if (s[0] == 'a' && s[1] == 'n' && s[2] == 'g' && s[3] == 'l' &&
							 s[4] == 'e' && s[5] == '.' && s[6] == 't')
					{
						float val;
						s = parse_float(s + 7, &val);
						angle_target = val;
					}
					// ---- 命令：SET P:x I:y D:z（根据当前模式设置对应 PID）----
					else if (s[0] == 'S' && s[1] == 'E' && s[2] == 'T' && s[3] == ' ')
					{
						float new_p, new_i, new_d;
						s += 4;
						if (s[0] == 'P' && s[1] == ':')
						{
							s = parse_float(s + 2, &new_p);
							if (s[0] == ' ' && s[1] == 'I' && s[2] == ':')
							{
								s = parse_float(s + 3, &new_i);
								if (s[0] == ' ' && s[1] == 'D' && s[2] == ':')
								{
									s = parse_float(s + 3, &new_d);
									if (GYRO_GetMode() == MODE_ANGLE_TUNE)
									{
										a_kp = new_p;
										a_ki = new_i;
										a_kd = new_d;
									}
									else
									{
										p_kp = new_p;
										p_ki = new_i;
										p_kd = new_d;
									}
								}
							}
						}
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