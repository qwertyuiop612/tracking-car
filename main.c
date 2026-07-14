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
#include "stdio.h"

volatile int status = 0;
extern volatile uint32_t sys_tick_ms;
GyroData_t gyro_data;
char oled_str1[50];
char oled_str2[50];

void sysTick_Handler(void)
{
    sys_tick_ms++;
}

int main(void)
{
    //Init
    SYSCFG_DL_init();
    DL_ADC12_enableConversions(ADC12_0_INST);   //adc初始化
    GYRO_Init();
    SERVO_Init();                               // 舵机初始化
    MOTOR_Init();                               // 电机初始化
    OLED_Init();                                // oled初始化
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    while (DMP_Init()) // 陀螺仪初始化
        ;

    // NVIC
    NVIC_EnableIRQ(TB6612_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    while (1)
    {
        if (GYRO_GetData(&gyro_data))
        {
            sprintf(oled_str1, "angle deg = %.2f", gyro_data.angle_deg);
            OLED_ShowString(0, 0, (u8 *)oled_str1, 127);
            sprintf(oled_str2, "dps = %.2f", gyro_data.dps);
            OLED_ShowString(0, 23, (u8 *)oled_str2, 127);
            OLED_Refresh();
        }
        track();
    }
}