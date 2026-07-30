#include "receiver.h"
#include "oled.h"
#include <stdlib.h>

// disp_dirty 定义在 main.c，此处声明
extern volatile uint8_t disp_dirty;

// 手动 atoi，跳过 strtol 开销
static int atoi_simple(const char *s, const char **end)
{
    int v = 0;
    while (*s >= '0' && *s <= '9') {
        v = v * 10 + (*s - '0');
        s++;
    }
    if (end) *end = s;
    return v;
}

// 跳过逗号，返回下一字符指针
static const char *skip_comma(const char *s)
{
    if (*s == ',') s++;
    return s;
}

int receiver_parse(const char *s)
{
    if (s[0] != '@') return 0;

    switch (s[1]) {
    case 'I':   // @I — Init（清屏，由主循环执行 I2C 刷新）
        extern u8 OLED_GRAM[144][8];
        for (int i = 0; i < 8; i++)
            for (int n = 0; n < 128; n++)
                OLED_GRAM[n][i] = 0;
        disp_dirty = 1;
        return 1;

    case 'C':   // @C — Clear（同上）
        {
            extern u8 OLED_GRAM[144][8];
            for (int i = 0; i < 8; i++)
                for (int n = 0; n < 128; n++)
                    OLED_GRAM[n][i] = 0;
        }
        disp_dirty = 1;
        return 1;

    case 'R':   // @R — Refresh（由主循环执行）
        disp_dirty = 1;
        return 1;

    case 'T':   // @T,n — ColorTurn（单字节 I2C，ISR 中可执行）
        if (s[2] == ',') {
            int mode = atoi_simple(s + 3, &s);
            OLED_ColorTurn((u8)mode);
        }
        return 1;

    case 'D':   // @D,n — DisplayTurn
        if (s[2] == ',') {
            int mode = atoi_simple(s + 3, &s);
            OLED_DisplayTurn((u8)mode);
        }
        return 1;

    case 'S':   // @S,x,y,size,text（仅写显存缓冲区，安全）
        if (s[2] == ',') {
            const char *p = s + 3;
            int x = atoi_simple(p, &p);
            p = skip_comma(p);
            int y = atoi_simple(p, &p);
            p = skip_comma(p);
            int size = atoi_simple(p, &p);
            p = skip_comma(p);
            OLED_ShowString((u8)x, (u8)y, (u8 *)p, (u8)size);
            disp_dirty = 1;
        }
        return 1;

    default:
        return 0;
    }
}
