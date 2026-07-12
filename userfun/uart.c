#include "uart.h"

//------------------------------------------发送单个字符------------------------------------//
void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
	DL_UART_transmitDataBlocking(uart, chr);
}

//--------------------------------------------发送字符串------------------------------------//
void UART_send_string(UART_Regs *uart, const char *str)
{
	while(*str)
	{
		UART_send_char(uart , (uint8_t) *str);
		str++;
	}
}

//------------------------------------------中断接收字符------------------------------------//
void UART_0_INST_IRQHandler()
{
	switch(DL_UART_getPendingInterrupt(UART_0_INST))
	{
		case DL_UART_IIDX_RX:
			uint8_t rec = DL_UART_receiveData(UART_0_INST);
			UART_send_char(UART_0_INST, rec);
			break;
		default:
			break;
	}
}