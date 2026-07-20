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

// ===== 弱函数覆盖：替代 SYSCFG_DL_SYSCTL_init，增加 HFXT 等待和 PLL 超时 =====
// 因为 SysConfig 会覆盖 Debug/ti_msp_dl_config.c，所以在此定义强符号版本
static const DL_SYSCTL_SYSPLLConfig mySYSPLLConfig = {
    .inputFreq = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
    .rDivClk2x = 1,
    .rDivClk1 = 0,
    .rDivClk0 = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef = DL_SYSCTL_SYSPLL_REF_HFCLK,
    .qDiv = 3,
    .pDiv = DL_SYSCTL_SYSPLL_PDIV_1};

/* 自定义 SYSCTL 初始化：HFXT 等待 + PLL 超时，替代生成版本 */
static void my_SYSCTL_init(void)
{
    volatile uint32_t to;
    uint32_t pll_retry;

    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ, 255, false);

    /* 等待 HFXT 晶振起振稳定 */
    to = 800000;
    while (!(DL_SYSCTL_getClockStatus() & DL_SYSCTL_CLK_STATUS_HFCLK_GOOD))
    {
        if (--to == 0)
            break;
    }

    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *)&mySYSPLLConfig);

    /* SYSPLL_ERR_01 绕回，最多重试 10 次 */
    pll_retry = 0;
    while (SYSCFG_DL_SYSCTL_SYSPLL_init() == false)
    {
        if (++pll_retry >= 10)
            break;
        DL_SYSCTL_disableSYSPLL();
        DL_SYSCTL_enableSYSPLL();

        to = 500000;
        while ((DL_SYSCTL_getClockStatus() & SYSCTL_CLKSTATUS_SYSPLLGOOD_MASK) != DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD)
        {
            if (--to == 0)
                break;
        }
    }

    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
}

/*// VOFA+ 调试用（用简单计数器，不依赖SysTick）
static uint32_t vofa_counter = 0;
#define VOFA_INTERVAL 20000 // 主循环每20k次发一帧*/

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

int main(void)
{
    // 等待供电稳定（约 10ms @ 32MHz SYSOSC）
    delay_cycles(320000);

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
        /*if (GYRO_GetData(&gyro_data))
        {
            sprintf(oled_str1, "deg = %.2f", gyro_data.angle_deg);
            OLED_ShowString(0, 0, (u8 *)oled_str1, 16);
            sprintf(oled_str2, "dps = %.2f", gyro_data.dps);
            OLED_ShowString(0, 40, (u8 *)oled_str2, 16);
            OLED_Refresh();
        }
        else
        {
            OLED_ShowString(0, 0, (u8 *)"error", 16);
            OLED_Refresh();
        }*/

        /*// VOFA+ 调试输出
        if (++vofa_counter >= VOFA_INTERVAL)
        {
            vofa_counter = 0;
            uart_printf("%.1f,%.1f,%d,%.1f,%.1f,%d\n",
                        LEFT.target_speed, LEFT.speed, PWM_1_duty,
                        RIGHT.target_speed, RIGHT.speed, PWM_2_duty);
        }*/
    }
}