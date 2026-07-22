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
#include "mpu_port.h"
#include "gyro.h"
#include "uart.h"
#include "stdio.h"

volatile int status = 0;
extern volatile uint32_t sys_tick_ms;
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
    GYRO_Init();
    SERVO_Init();                               // 舵机初始化
    MOTOR_Init();                               // 电机初始化
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    // NVIC
    NVIC_EnableIRQ(TB6612_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    // 方向 + STBY + 启动PWM（为PID提供初始基准值）
    direction(1, 1);                                    // 左轮正转
    direction(2, 1);                                    // 右轮正转
    DL_GPIO_setPins(TB6612_STBY_PORT, TB6612_STBY_PIN); // 使能驱动
    motor_PWM(100, 100);                                // PID基准值，防止从0起步
    LEFT.target_speed = 500;
    RIGHT.target_speed = 500;

    // 所有配置就绪后，启动PID定时器
    DL_Timer_startCounter(MOTOR_PID_INST);
    __enable_irq(); // 全局中断使能，PID 接管

    while (1)
    {
    }
}