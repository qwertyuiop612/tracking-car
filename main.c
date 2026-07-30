/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "default.h"
#include "interrupt.h"
#include "oled.h"
#include "uart.h"
#include "stepper.h"
#include "stdio.h"

volatile int status = 0;
volatile uint32_t sys_tick_ms = 0;

// 运行模式
#define MODE_ANGLE_TUNE 0
#define MODE_TRACK 1
static volatile int g_mode = MODE_ANGLE_TUNE;

// 计时相关（由 uart.c 的 @M 处理更新）
volatile uint32_t track_start_ms = 0;
volatile uint32_t track_stop_ms = 0;
volatile uint8_t marker_stopped = 0;

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    STEPPER_Init();

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    __enable_irq();

    while (1)
    {
        static int last_status = -1;
        if (status != last_status)
        {
            last_status = status;
            g_mode = status % 2;
            if (g_mode == MODE_TRACK)
            {
                track_start_ms = sys_tick_ms;
                marker_stopped = 0;
            }
            uart_printf("mode:%d\r\n", g_mode); // 同步到下层 car2
        }

        if (g_mode == MODE_ANGLE_TUNE)
        {
            OLED_ShowString(0, 0, (u8 *)"Start                                                      ", 16);
        }
        else
        {
            char str[20];
            uint32_t disp_ms = marker_stopped ? (track_stop_ms - track_start_ms)
                                              : (sys_tick_ms - track_start_ms);
            sprintf(str, "T:%4.1f s", disp_ms / 1000.0f);
            OLED_ShowString(0, 0, (u8 *)str, 16);
        }
        OLED_Refresh();
        delay_ms(50);
    }
}