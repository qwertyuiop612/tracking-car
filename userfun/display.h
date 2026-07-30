#ifndef DISPLAY_H
#define DISPLAY_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

// UART 显示协议 — 将 OLED 数据通过 UART0 发送到上层
// 命令格式: 单行 ASCII，'\n' 结尾
//   @I        Init（清屏+初始化）
//   @C        Clear（清空显存）
//   @T,n      ColorTurn（0=正常, 1=反色）
//   @D,n      DisplayTurn（0=正常, 1=旋转180）
//   @S,x,y,n,text   ShowString（n=字号, text不含逗号换行）
//   @R        Refresh（刷新显存到屏幕）

void DISP_Init(void);
void DISP_Clear(void);
void DISP_ColorTurn(uint8_t mode);
void DISP_DisplayTurn(uint8_t mode);
void DISP_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size);
void DISP_Refresh(void);

#endif
