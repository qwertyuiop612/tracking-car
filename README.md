# tracking-car

本工程基于 TI MSPM0G3507 empty 示例，扩展为一个简易循迹小车项目。项目把传感器、循迹、丢线处理和电机 PWM 控制封装在 `userfun` 目录下，`empty.c` 中只需通过 `flag()` 判断状态后调用 `track()` 或 `track_lost()`。

## 目录说明

- `empty.c`：主程序，初始化 SysConfig 并在主循环中切换循迹/丢线逻辑。
- `userfun/track.c` / `userfun/track.h`：循迹核心逻辑。
- `userfun/sensor.c` / `userfun/sensor.h`：传感器读取接口。
- `userfun/motor.c` / `userfun/motor.h`：电机 PWM 输出与差速控制。
- `ti_msp_dl_config.c` / `ti_msp_dl_config.h`：SysConfig 初始化。

## 核心接口

- `float flag(void);`
  - 读取传感器状态。
  - 若当前检测为丢线，则设置丢线标志并返回 `1`。
  - 若未丢线，则记录当前 `state` 作为最后一次有效线路位置，并返回 `0`。
- `void track(void);`
  - 正常循迹函数。
  - 调用 `sensor_detect()` 获得线路位置，计算 `avrPWM`、`leftPWM` 和 `rightPWM`，并通过 `motor_PWM()` 输出。
- `void track_lost(void);`
  - 丢线恢复函数。
  - 根据上一次记录的 `last_state` 判定方向，执行转向或直行，直到重新检测到线路后清除丢线标志。

## 运行流程

`empty.c` 中的主循环逻辑如下：

```c
SYSCFG_DL_init();
motor_Init();
while (1)
{
    if (flag() == 0)
    {
        track();
    }
    else
    {
        track_lost();
    }
}
```

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
| 电机驱动 TB6612 AIN1 | `TB6612_AIN1` | PA8 |
| 电机驱动 TB6612 AIN2 | `TB6612_AIN2` | PA9 |
| 电机驱动 TB6612 STBY | `TB6612_STBY` | PB24 |
| PWM 输出 | `PWM_0.peripheral.ccp0Pin` | PA12 |
| OLED SCL | `OLED_GRP.SCL_PIN` | PB2 |
| OLED SDA | `OLED_GRP.SDA_PIN` | PB3 |
| 开关 1 | `SWITCH_GRP.SWITCH_1` | PB7 |
| 开关 0 | `SWITCH_GRP.SWITCH_0` | 未分配 |

请根据实际硬件在 SysConfig 或代码中的宏定义（如 `SENSOR_GRP_SNESOR_0_PIN`、`TB6612_AIN1_PIN` 等）同步上述映射。

## 使用说明

1. 在 `empty.c` 中包含头文件：
   ```c
   #include "track.h"
   ```
2. 初始化 SysConfig 与电机：
   ```c
   SYSCFG_DL_init();
   motor_Init();
   ```
3. 主循环中调用：
   - 正常循迹：`track();`
   - 丢线检测：`flag();`
   - 丢线恢复：`track_lost();`

## 设计说明

- `track.c` 中的 `flag_last_state` 与 `flag_DX` 为文件级 `static` 变量，外部不可直接访问。
- `flag()` 用于检测是否脱离线路，并在未脱线时记录最后一次有效线路位置。
- `track()` 通过 `DifPWM()` 计算差速，结合传感器位置计算左右电机 PWM。
- `track_lost()` 根据最后一次线路位置决定转向策略，直到重新检测到线路后恢复循迹。

## 编译与调试

- 按照原 empty 示例工程流程编译、烧录并运行。
- 注意 SWD 引脚 `PA19/PA20` 在调试时可能被占用，若需要复用请先断开调试连接。
- 修改引脚配置后，请同步 `empty.syscfg` 并更新相应代码。
