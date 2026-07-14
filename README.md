# tracking-car

基于 TI MSPM0G3507 的智能循迹小车项目。采用 7 路灰度传感器检测线路，双路编码器电机配合 PID 速度环实现精确控制，集成 MPU6050 六轴陀螺仪（DMP 姿态解算）、OLED 显示、舵机控制、串口通信及 ADC 电压检测。

## 硬件平台

- **MCU**：TI MSPM0G3507（Cortex-M0+，80 MHz）
- **电机驱动**：TB6612（双路直流电机驱动）
- **传感器**：7 路灰度循迹传感器 + MPU6050 六轴陀螺仪
- **显示**：0.96 寸 OLED（I2C，SSD1306）
- **舵机**：1 路舵机
- **编码器**：2 路编码器（左右轮速度反馈）
- **系统时钟**：外部 40 MHz 晶振 + PLL 倍频

## 目录结构

```
├── main.c                          # 主程序：初始化各模块，主循环调用 track()
├── empty.syscfg                    # SysConfig 配置文件
├── userfun/
│   ├── default.c / default.h       # 通用函数：delay_ms()、PWM_duty()
│   ├── sensor.c / sensor.h         # 传感器：7 路灰度读取 & 差速查表
│   ├── track.c / track.h           # 循迹：综合传感器与电机执行循迹逻辑
│   ├── motor.c / motor.h           # 电机：TB6612 驱动、编码器测速、PID 速度环
│   ├── servo.c / servo.h           # 舵机：角度初始化与设置
│   ├── interrupt.c / interrupt.h   # 中断：编码器计数、按键状态切换
│   ├── adc.c / adc.h               # ADC：电压采集
│   ├── uart.c / uart.h             # UART：串口收发
│   ├── OLED/
│   │   ├── oled.c / oled.h         # OLED：SSD1306 硬件 I2C 驱动
│   │   └── oledfont.h              # 字库（12/16/24 点阵 ASCII + 汉字）
│   └── mpu6050/
│       ├── mpu_port.c / mpu_port.h # MSPM0 平台移植层（I2C 读写、时基）
│       ├── inv_mpu.c / inv_mpu.h   # InvenSense 官方 MPU 驱动库
│       ├── inv_mpu_dmp_motion_driver.c / .h  # DMP 运动驱动
│       ├── dmpKey.h                # DMP 固件数组
│       └── dmpmap.h                # DMP 寄存器映射
├── Debug/                          # 编译输出目录
├── ti_msp_dl_config.c              # SysConfig 自动生成的初始化代码
└── ti_msp_dl_config.h              # SysConfig 自动生成的头文件
```

## 模块说明

### 主程序（main.c）

```c
SYSCFG_DL_init();
DL_ADC12_enableConversions(ADC12_0_INST);
SERVO_Init();
MOTOR_Init();
OLED_Init();
OLED_ColorTurn(0);
OLED_DisplayTurn(0);
OLED_Clear();
while (DMP_Init());                   // 阻塞等待 MPU6050 DMP 就绪
NVIC_EnableIRQ(TB6612_GPIOA_INT_IRQN);
NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
while (1) { track(); }
```

初始化顺序：SysConfig → ADC → 舵机 → 电机（含 PID 定时器） → OLED → MPU6050/DMP → 中断使能 → 循迹循环。

`status` 变量（`volatile int`）在中断中由按键修改，可在 `track()` 中根据状态值实现多模式切换。

### 循迹（track.c）

- `track()`：调用 `sensor_detect()` 获取线路中心位置，计算基础速度，结合 `Difspeed()` 查表得到差速值，设定左右轮目标速度。
  - 丢线（无传感器触发）→ 两轮正向直行
  - 起点/终点（7 路全触发）→ 停车（`WHEEL.target_speed = 0`）
  - 正常 → 差速转向

### 传感器（sensor.c）

- `sensor_detect()`：读取 7 路 GPIO，计算加权平均位置（1~7），返回线路中心值并更新 `idx` 离散位置。
- `Difspeed()`：根据 `idx`（2~8）查表返回差速值，中心为 0，越偏离差速越大。

### 电机（motor.c）

- `struct WHEEL { speed, target_speed, last_error, current_error }` — 左右轮 + 公共目标
- `MOTOR_Init()`：使能 TB6612，配置 TIMG0 PWM 及 TIMA0 PID 定时器（5ms 周期）
- `motor_PWM(leftPWM, rightPWM)`：正负值控制方向与占空比
- `cal_speed(motor_id)`：编码器脉冲 → 实际速度（mm/s）
- `MOTOR_PID(motor_id)`：增量式 PI 速度环，在定时器中断中执行

### 舵机（servo.c）

- `SERVO_Init()`：启动 TIMG7，初始占空比 7.5%（≈ 90°）
- `SERVO_set(deg)`：0~180° 角度 → PWM 比较值

### OLED（OLED/oled.c）

- 硬件 I2C（I2C1）驱动的 SSD1306，128×64 分辨率
- 支持 12/16/24 点阵字符、汉字、图形、数字、图片显示
- `OLED_WR_Byte()` 使用 DriverLib I2C FIFO API

### 中断（interrupt.c）

- `GROUP1_IRQHandler()`：
  - GPIOA → 左轮编码器 A 相脉冲计数 `tmp_a++`
  - GPIOB → 右轮编码器 A 相脉冲计数 `tmp_b++`、按键 SWITCH_0/1 修改 `status`
- `sysTick_Handler()`（main.c）：`sys_tick_ms++`，为 MPU6050 DMP 提供毫秒时基

### MPU6050 陀螺仪（mpu6050/）

- **mpu_port.c/h**：MSPM0 平台移植层
  - `MPU_Write_Len()` / `MPU_Read_Len()`：基于硬件 I2C0 的多字节读写，带超时检测
  - `mget_ms()`：返回系统毫秒计数
  - `DMP_Init()`：加载 DMP 固件并初始化
  - `DMP_Read_Data()`：读取姿态角（pitch / roll / yaw）
- **inv_mpu.c/h**：InvenSense 官方 MPU 驱动库
- **inv_mpu_dmp_motion_driver.c/h**：DMP 数字运动处理器驱动
- **dmpKey.h / dmpmap.h**：DMP 固件镜像与寄存器映射

### ADC（adc.c）

- `ADC_get()`：启动 ADC12 转换，返回通道 2（PA17）电压值

### UART（uart.c）

- `UART_send_char()` / `UART_send_string()`：阻塞式发送
- `UART_0_INST_IRQHandler()`：接收中断回显

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
| SWITCH_GRP | SWITCH_0 | PB6 | 按键 0（status +1） |
| SWITCH_GRP | SWITCH_1 | PB7 | 按键 1（status +2） |
| TB6612 | AIN1 | PA8 | 左电机方向 1 |
| TB6612 | AIN2 | PA9 | 左电机方向 2 |
| TB6612 | STBY | PB24 | 电机驱动使能 |
| TB6612 | ENCD_LA | PA22 | 左轮编码器 A 相 |
| TB6612 | ENCD_LB | PA21 | 左轮编码器 B 相 |
| TB6612 | ENCD_RA | PB19 | 右轮编码器 A 相 |
| TB6612 | ENCD_RB | PB20 | 右轮编码器 B 相 |
| TB6612 | BIN1 | PB18 | 右电机方向 1 |
| TB6612 | BIN2 | PA7 | 右电机方向 2 |
| PWM_0 | CCP0 | PA12 | 左电机 PWM（TIMG0） |
| PWM_0 | CCP1 | PA13 | 右电机 PWM（TIMG0） |
| SERVO | CCP1 | PA27 | 舵机 PWM（TIMG7） |
| OLED | SCL | PB2 | OLED I2C1 时钟 |
| OLED | SDA | PB3 | OLED I2C1 数据 |
| MPU | SCL | PA1 | MPU6050 I2C0 时钟 |
| MPU | SDA | PA0 | MPU6050 I2C0 数据 |
| ADC12_0 | ADC_IN2 | PA17 | ADC 电压采集 |
| VREF | VREF+ | PA23 | ADC 参考电压 2.5V |
| UART_0 | TX | PA28 | 串口发送 |
| UART_0 | RX | PA31 | 串口接收 |
| HFXT | HFXIN | PA5 | 外部 40MHz 晶振 |
| HFXT | HFXOUT | PA6 | 外部 40MHz 晶振 |

## 编译与烧录

1. 使用 Code Composer Studio（Theia）打开工程。
2. 通过 SysConfig 修改引脚配置后，重新生成 `ti_msp_dl_config.c/h`。
3. 执行 build 任务编译工程。
4. 通过 XDS-110 调试器烧录到 MSPM0G3507 LaunchPad。

> 调试时 J101 的 13:14（SWDIO）和 15:16（SWCLK）需短接；若要在应用中复用 PA19/PA20，需断开对应跳线帽。

## 状态机

`status`（volatile int）通过 SWITCH_0（+1 % 3）和 SWITCH_1（+2 % 3）按键在 0、1、2 间循环，可用于在 `track()` 中切换运行模式（如循迹、调试、遥控等）。
