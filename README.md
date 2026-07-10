# tracking-car

本工程基于 MSPM0 DriverLib 的 empty 示例，扩展了循迹小车相关模块。包含封装好的循迹/传感/电机接口与 SysConfig 引脚映射说明，便于在 `main` / `empty.c` 中直接调用 `track()`、`flag()`、`track_lost()` 等函数实现循迹与丢线处理。

## 概要

- 扩展在 `userfun` 目录下增加自定义模块。
- 通过 `track.c` 实现传感器状态判断、`last_state` 记录、丢线标志 `DX` 控制、PWM 计算与电机驱动调用。
- 通过 `flag()` 接口判断丢线、通过 `track_lost()` 接口处理脱线恢复。
- 通过 `track.h` 对外暴露接口，`flag_last_state` 与 `flag_DX` 为文件级 `static`，外部不能直接访问。

## 已封装模块

- `userfun/track.c`
  - 函数：`float flag(void);`、`void track(void);`、`void track_lost(void);`
  - 职责：记录 `last_state`、维护丢线标志 `DX`、主循迹控制逻辑（PWM 计算、驱动电机）。
- `userfun/motor.c` / `userfun/motor.h`
  - 典型接口：`motor_PWM(left, right);`、`DifPWM();`
  - 职责：电机 PWM 输出与差速封装。
- `userfun/sensor.c` / `userfun/sensor.h`
  - 典型接口：`int sensor_detect(void);`
  - 职责：读取传感器输入并返回位置/state 值。
- `userfun/track.h`
  - 对外声明：`float flag(void);`、`void track(void);`、`void track_lost(void);`

> 如果 `flag()` 名称与系统库冲突，建议重命名为 `track_flag()` 或类似更明确的函数名。

## SysConfig 引脚映射（来自 `empty.syscfg`）

| 外设 / 变量 | 代码符号 / 名称 | 物理引脚 |
| --- | ---: | --- |
| 调试 SWCLK | `DEBUGSS.swclkPin` | PA20 |
| 调试 SWDIO | `DEBUGSS.swdioPin` | PA19 |
| 传感器组 - 通道 0 | `SENSOR_GRP.SNESOR_0` | PA2 |
| 传感器组 - 通道 1 | `SENSOR_GRP.SNESOR_1` | PA24 |
| 传感器组 - 通道 2 | `SENSOR_GRP.SNESOR_2` | PB9 |
| 传感器组 - 通道 3 | `SENSOR_GRP.SNESOR_3` | PB8 |
| 传感器组 - 通道 4 | `SENSOR_GRP.SNESOR_4` | PA16 |
| 传感器组 - 通道 5 | `SENSOR_GRP.SNESOR_5` | PA15 |
| 传感器组 - 通道 6 | `SENSOR_GRP.SNESOR_6` | PA14 |
| PWM 组 0 CCP0（建议） | `PWM_0.peripheral.ccp0Pin` | PA12 |
| PWM 组 0 CCP1（建议） | `PWM_0.peripheral.ccp1Pin` | PA13 |
| PWM 组 1 CCP0 | `PWM_1.peripheral.ccp0Pin` | PA8 |
| PWM 组 1 CCP1 | `PWM_1.peripheral.ccp1Pin` | PA9 |

请根据实际硬件在 SysConfig 或代码中的宏定义（如 `MOTOR_LEFT_PWM_PIN`、`SENSOR_0_PIN` 等）同步上述映射。

## 使用说明

1. 在 `main/empty.c` 中包含头文件：
   ```c
   #include "userfun/track.h"
   ```

2. 在主循环或定时回调中调用：
   - 正常循迹：`track();`
   - 丢线检测：`flag();`
   - 脱线恢复：`track_lost();`

3. 确保工程中编译并链接 `userfun` 下的源文件。

## 预期行为说明

- `flag()`：读取传感器状态；如果检测到脱线则设置 `DX=1`，否则保留上一次的 `last_state`。
- `track_lost()`：从 `flag.c` 维护的 `last_state` 获取上一次状态；如果重新检测到线路，则将 `DX` 清零。
- `track()`：根据传感器检测值计算 PWM，调整左右电机输出，保持循迹。

## 开发提示

- 修改引脚后请同步 `SysConfig` 并更新代码中的宏或 pin 映射。
- `track.c` 中使用的是文件级 `static` 变量，外部只能通过接口访问。
- 若需要我把具体宏定义写入头文件或生成 `track.h`，请贴出你的宏名称或现有头文件片段。

## 编译与运行

按原 empty 项目流程编译、烧录并运行。调试时注意 SWD 引脚 `PA19/PA20` 可能被占用。
