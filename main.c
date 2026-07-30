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
#include "sensor.h"
#include "track.h"
#include "interrupt.h"
#include "motor.h"
#include "oled.h"
#include "servo.h"
#include "gyro.h"
#include "uart.h"
#include "mgnt.h"
#include "stdio.h"

volatile int status = 0;
volatile uint32_t sys_tick_ms = 0;
GyroData_t gyro_data;
char oled_str1[50];
char oled_str2[50];

/*// VOFA+ 调试用（用简单计数器，不依赖SysTick）
static uint32_t vofa_counter = 0;
#define VOFA_INTERVAL 20000 // 主循环每20k次发一帧*/

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

int main(void)
{
    SYSCFG_DL_init();
    MGNT_Init();                                // 电磁铁初始化
    GYRO_Init();                                // 陀螺仪初始化
    SERVO_Init();                               // 舵机初始化
    MOTOR_Init();                               // 电机初始化
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    // NVIC
    NVIC_EnableIRQ(DRV8870_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    // 所有配置就绪后，启动 PID 定时器
    DL_Timer_startCounter(MOTOR_PID_INST);
    __enable_irq(); // 全局中断使能，PID 接管

    while (1)
    {
        // 同步物理按键状态到运行模式（按键切换 0/1/2，取模到 0/1）
        static int last_status = -1;
        if (status != last_status)
        {
            last_status = status;
            GYRO_SetMode(status % 2);
        }

        if (GYRO_GetMode() == MODE_ANGLE_TUNE)
        {
            OLED_ShowString(0, 0, (u8 *)"Start                                                      ", 16);
            OLED_Refresh();
        }
        else
        {
            char str[20];
            uint32_t disp_ms = marker_stopped ? (track_stop_ms - track_start_ms)
                                              : (sys_tick_ms - track_start_ms);
            sprintf(str, "T:%4.1f s", disp_ms / 1000.0f);
            OLED_ShowString(0, 0, (u8 *)str, 16);
            OLED_Refresh();
        }
        OLED_Refresh();
    }
}