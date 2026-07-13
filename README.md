# tracking-car

基于 TI MSPM0G3507 的智能循迹小车项目。采用 7 路灰度传感器检测线路，双路编码器电机配合 PID 速度环实现精确控制，支持 OLED 显示、舵机控制、串口通信及 ADC 电压检测，并在主循环中不断执行循迹逻辑。

## 硬件平台

- **MCU**：TI MSPM0G3507（Cortex-M0+）
- **电机驱动**：TB6612（双路直流电机驱动）
- **传感器**：7 路灰度循迹传感器
- **显示**：0.96 寸 OLED（I2C，SSD1306）
- **舵机**：1 路舵机
- **编码器**：2 路编码器（左右轮速度反馈）
- **系统时钟**：外部 40 MHz 晶振 + PLL 倍频

## 目录结构

```
├── main.c                          # 主程序：初始化各模块，主循环调用 track()
├── empty.syscfg                    # SysConfig 配置文件
├── userfun/
│   ├── default.c / default.h       # 通用函数：延时 delay_ms()、PWM 占空比设置
│   ├── sensor.c / sensor.h         # 传感器：7 路灰度读取 & 差速查表
│   ├── track.c / track.h           # 循迹：综合传感器与电机执行循迹逻辑
│   ├── motor.c / motor.h           # 电机：TB6612 驱动、编码器测速、PID 速度环
│   ├── servo.c / servo.h           # 舵机：角度初始化与角度设置
│   ├── oled.c / oled.h             # OLED：SSD1306 I2C 驱动、字符/图形显示
│   ├── oledfont.h                  # OLED 字库文件
│   ├── interrupt.c / interrupt.h   # 中断：编码器计数、按键状态切换
│   ├── adc.c / adc.h               # ADC：电压采集
│   └── uart.c / uart.h             # UART：串口收发
├── Debug/                          # 编译输出目录
├── ti_msp_dl_config.c              # SysConfig 自动生成的初始化代码
└── ti_msp_dl_config.h              # SysConfig 自动生成的头文件
```

## 模块说明

### 主程序（main.c）

```c
SYSCFG_DL_init();
DL_ADC12_enableConversions(ADC12_0_INST);
SERVO_init();
MOTOR_init();
OLED_init();
NVIC_EnableIRQ(TB6612_GPIOA_INT_IRQN);
NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

while (1) { track(); }
```

初始化顺序：SysConfig → ADC → 舵机 → 电机（含 PID 定时器） → OLED → 中断使能 → 进入循迹循环。

### 循迹（track.c）

- `track()`：根据 `sensor_detect()` 返回的线路中心位置计算基础速度，结合 `Difspeed()` 查表得到的差速值，分别设定左右轮目标速度。检测到 7 路全黑（起点/终点）时停车，丢线时两轮正向保持。

### 传感器（sensor.c）

- `sensor_detect()`：读取 7 路 GPIO 状态，计算加权平均位置（1~7），返回线路中心值。
- `Difspeed()`：根据离散化位置 `idx`（0~8）查表返回差速值，中间为 0，越偏离中心差速越大。

### 电机（motor.c）

- 结构体 `WHEEL`：记录速度 `speed`、目标速度 `target_speed`、误差历史。
- `MOTOR_init()`：使能 TB6612，配置 PWM 及 PID 定时器。
- `motor_PWM(leftPWM, rightPWM)`：根据正负值控制方向与占空比，输出到 TB6612。
- `cal_speed(motor_id)`：通过编码器脉冲增量计算实际速度（mm/s）。
- `MOTOR_PID(motor_id)`：增量式 PI 速度环控制，在定时器中断中周期性执行。

### 舵机（servo.c）

- `SERVO_init()`：启动 TIMG7 定时器，设置初始占空比（7.5% ≈ 90°）。
- `SERVO_set(deg)`：将 0~180° 角度转换为 PWM 比较值。

### OLED（oled.c）

- 基于 I2C 的 SSD1306 驱动，支持字符显示（12/16/24 字号）、汉字、图形、数字及图片显示。
- `OLED_WR_Byte()` 使用 DriverLib I2C FIFO API 进行传输。

### 中断（interrupt.c）

- `GROUP1_IRQHandler()`：
  - GPIOA 中断 → 编码器 A（左轮）计数累加 `tmp_a`
  - GPIOB 中断 → 编码器 B（右轮）计数累加 `tmp_b`、按键 SWITCH_0/1 切换 `status` 状态（0~2 循环）

### ADC（adc.c）

- `ADC_get()`：启动 ADC12 转换，返回通道 2（PA17）的电压值。

### UART（uart.c）

- `UART_send_char()` / `UART_send_string()`：阻塞式发送。
- `UART_0_INST_IRQHandler()`：接收中断中回显收到的字符。

## SysConfig 引脚映射

| 外设 | 名称 | 引脚 | 功能 |
|------|------|------|------|
| DEBUGSS | SWCLK | PA20 | 调试时钟 |
| DEBUGSS | SWDIO | PA19 | 调试数据 |
| SENSOR_GRP | SNESOR_0 | PA2 | 灰度传感器 0（最左） |
| SENSOR_GRP | SNESOR_1 | PA24 | 灰度传感器 1 |
| SENSOR_GRP | SNESOR_2 | PB9 | 灰度传感器 2 |
| SENSOR_GRP | SNESOR_3 | PB8 | 灰度传感器 3（中间） |
| SENSOR_GRP | SNESOR_4 | PA16 | 灰度传感器 4 |
| SENSOR_GRP | SNESOR_5 | PA15 | 灰度传感器 5 |
| SENSOR_GRP | SNESOR_6 | PA14 | 灰度传感器 6（最右） |
| SWITCH_GRP | SWITCH_0 | PB6 | 按键 0（状态切换） |
| SWITCH_GRP | SWITCH_1 | PB7 | 按键 1（状态切换） |
| TB6612 | AIN1 | PA8 | 左电机方向 1 |
| TB6612 | AIN2 | PA9 | 左电机方向 2 |
| TB6612 | STBY | PB24 | 电机驱动使能 |
| TB6612 | ENCD_LA | PA22 | 左轮编码器 A 相 |
| TB6612 | ENCD_LB | PA21 | 左轮编码器 B 相 |
| TB6612 | ENCD_RA | PB19 | 右轮编码器 A 相 |
| TB6612 | ENCD_RB | PB20 | 右轮编码器 B 相 |
| TB6612 | BIN1 | PB18 | 右电机方向 1 |
| TB6612 | BIN2 | PA7 | 右电机方向 2 |
| PWM_0 | CCP0 | PA12 | 左电机 PWM 输出（TIMG0） |
| PWM_0 | CCP1 | PA13 | 右电机 PWM 输出（TIMG0） |
| SERVO | CCP1 | PA27 | 舵机 PWM 输出（TIMG7） |
| OLED | SCL | PB2 | OLED I2C 时钟 |
| OLED | SDA | PB3 | OLED I2C 数据 |
| ADC12_0 | ADC_IN2 | PA17 | ADC 电压采集 |
| VREF | VREF+ | PA23 | ADC 参考电压输出（2.5V） |
| UART_0 | TX | PA28 | 串口发送 |
| UART_0 | RX | PA31 | 串口接收 |
| HFXT | HFXIN | PA5 | 外部 40MHz 晶振输入 |
| HFXT | HFXOUT | PA6 | 外部 40MHz 晶振输出 |

## 编译与烧录

1. 使用 Code Composer Studio（Theia）打开工程。
2. 通过 SysConfig 修改引脚配置后，重新生成 `ti_msp_dl_config.c/h`。
3. 执行 build 任务编译工程。
4. 通过 XDS-110 调试器烧录到 MSPM0G3507 LaunchPad。

> 调试时 J101 的 13:14 和 15:16 需短接；若要在应用中复用 PA19/PA20，需断开对应跳线帽。

## 状态机说明

- `status` 变量通过 `SWITCH_0`（+1）和 `SWITCH_1`（+2）按键在 0、1、2 之间循环切换。
- 可在 `track()` 中根据 `status` 值实现不同运行模式（如循迹模式、调试模式、蓝牙遥控模式等）。
