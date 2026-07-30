#ifndef RECEIVER_H
#define RECEIVER_H

// 解析从下层(car2)通过 UART0 发来的 @ 显示命令，驱动本地 OLED
// 命令格式: 单行 ASCII，'\n' 结尾
//   @I         Init + Clear
//   @C         Clear
//   @T,n       ColorTurn (0/1)
//   @D,n       DisplayTurn (0/1)
//   @S,x,y,n,text   ShowString
//   @R         Refresh
// 返回 0=非显示命令, 1=已处理

int receiver_parse(const char *cmd);

#endif
