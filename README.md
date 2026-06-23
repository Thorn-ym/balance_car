# STM32F103C8T6 Balance Car

这是一个基于 `STM32F103C8T6` 的两轮自平衡小车控制程序，工程由 STM32CubeMX 生成 Makefile 项目后继续开发，适合使用 GCC 工具链编译，并通过 Ozone + J-Link 进行变量观察和在线调参。

项目主要功能包括：

- MPU6050 姿态采样
- 互补滤波计算俯仰角
- 角度环 PID
- 速度环 PID
- 转向环 PID
- TB6612 电机驱动
- 双编码器测速
- HC-05/HC-06 蓝牙串口遥控
- Android 蓝牙遥控 APP
- PC13 运行指示灯
- SSD1306 OLED 状态显示
- DHT11 温湿度采集
- HX711 称重传感器读取与重量换算
- Ozone + J-Link 调试变量入口

调试阶段可以通过 Ozone 直接观察和修改全局变量，完成姿态校准、电机方向确认、编码器方向确认和 PID 参数调节。

## 1. 硬件连接

主控为 `STM32F103C8T6`。

### MPU6050

| 功能 | STM32 引脚 | MPU6050 |
| --- | --- | --- |
| I2C1_SCL | PB8 | SCL |
| I2C1_SDA | PB9 | SDA |
| 3.3V | 3.3V | VCC |
| GND | GND | GND |

代码默认 MPU6050 地址为 `0x68`，即 AD0 接 GND 或悬空。

MPU6050 单独使用 I2C1 的 PB8/PB9。

### TB6612 电机驱动

| 功能 | STM32 引脚 | TB6612 |
| --- | --- | --- |
| STBY | PA3 | STBY |
| AIN1 | PA4 | AIN1 |
| AIN2 | PA5 | AIN2 |
| PWMA | PA6 / TIM3_CH1 | PWMA |
| BIN1 | PB1 | BIN1 |
| BIN2 | PB0 | BIN2 |
| PWMB | PA7 / TIM3_CH2 | PWMB |
| GND | GND | GND |

注意：

- TB6612 的 `VM` 接电机电源，不要接 STM32 3.3V。
- TB6612 的逻辑电源 `VCC` 接 3.3V。
- STM32、TB6612、电机电源必须共地。
- 程序停机时会拉低 `STBY`，并把 PWM 清零。

### 编码器

| 功能 | STM32 引脚 | 定时器 |
| --- | --- | --- |
| 左编码器 A | PA8 | TIM1_CH1 |
| 左编码器 B | PA9 | TIM1_CH2 |
| 右编码器 A | PA0 | TIM2_CH1 |
| 右编码器 B | PA1 | TIM2_CH2 |

如果编码器方向反了，可以交换 A/B 相，也可以在代码中给对应 delta 加负号。

### 运行指示灯

| 功能 | STM32 引脚 | 说明 |
| --- | --- | --- |
| 运行指示灯 | PC13 | 最小系统板常见板载 LED，运行允许时点亮 |

PB6/PB7 已用于蓝牙串口，不再作为实体按键。启停通过 Android APP 的 `START` / `EMERGENCY STOP`，或 Ozone 修改 `g_balance_debug.run_enable` 完成。PC13 指示灯跟随 `run_enable` 亮灭。

### OLED 显示屏

默认使用 0.96 寸 I2C SSD1306 128x64 OLED，地址 `0x3C`。

| 功能 | STM32 引脚 | OLED |
| --- | --- | --- |
| I2C2_SCL | PB10 | SCL |
| I2C2_SDA | PB11 | SDA |
| 3.3V | 3.3V | VCC |
| GND | GND | GND |

OLED 使用 I2C2 的 PB10/PB11，不与 MPU6050 共用 I2C1。

### HC-05/HC-06 蓝牙模块

蓝牙模块使用重映射后的 USART1 与 STM32 通信。手机 APP 只发送遥控命令，平衡控制仍由 STM32 完成。

| STM32F103C8T6 | HC-05/HC-06 | 说明 |
| --- | --- | --- |
| PB6 / USART1_TX | RXD | STM32 发给蓝牙模块 |
| PB7 / USART1_RX | TXD | 蓝牙模块发给 STM32 |
| GND | GND | 必须与 STM32、电机电源共地 |
| 3.3V 或 5V | VCC | 按模块板标注供电 |

注意：

- HC-05/HC-06 常见默认串口参数是 `9600 8N1`，当前代码也按 `9600` 配置。
- 如果你的蓝牙模块已经改成 `115200`，需要把 `remote_control.c` 里的 `REMOTE_UART_BAUDRATE` 改成 `115200U`。
- 很多 HC-05/HC-06 模块板的 `VCC` 可以接 5V，但串口电平仍建议按 3.3V 逻辑使用；如果模块 RXD 不耐 5V，需要确认电平安全。
- 手机需要先在系统蓝牙设置里配对模块，常见配对码是 `1234` 或 `0000`。

### DHT11 温湿度模块

| 功能 | STM32 引脚 | DHT11 |
| --- | --- | --- |
| DATA | PC14 | DATA |
| 3.3V | 3.3V | VCC |
| GND | GND | GND |

DHT11 数据脚需要上拉电阻；多数模块板已自带上拉。

### HX711 称重模块

| 功能 | STM32 引脚 | HX711 |
| --- | --- | --- |
| 数据输出 | PA2 | DT / DOUT |
| 时钟输入 | PB12 | SCK / SCLK |
| 3.3V | 3.3V | VCC |
| GND | GND | GND |

注意：

- HX711 常见模块可接 3.3V 或 5V；若接 5V，请确认 `DT/DOUT` 高电平不会超过 STM32 输入允许范围。
- 当前代码使用 HX711 通道 A、增益 128，每次读取后额外发送 1 个 SCK 脉冲。
- PB12 当前只用于 HX711 SCK；PA3 已用于 TB6612 STBY，不能作为 HX711 SCK。

## 2. 工程结构

核心代码在：

```text
Core/Inc/balance_car/
Core/Src/balance_car/
```

主要文件：

| 文件 | 作用 |
| --- | --- |
| `balance_control.c/.h` | 平衡车主控制逻辑、调试变量、控制节拍、PID 串级关系 |
| `pid.c/.h` | 位置式 PID，带积分限幅、微分先行、输出死区补偿 |
| `mpu6050_hal.c/.h` | HAL I2C 版 MPU6050 初始化和原始数据读取 |
| `motor_tb6612.c/.h` | TB6612 电机方向和 PWM 输出 |
| `encoder_hal.c/.h` | TIM1/TIM2 编码器模式读取左右轮速度 |
| `i2c_bus.c/.h` | I2C1/I2C2 总线初始化，MPU6050 使用 I2C1，OLED 使用 I2C2 |
| `oled_ssd1306.c/.h` | SSD1306 128x64 I2C OLED 分页刷新 |
| `remote_control.c/.h` | USART1 重映射 PB6/PB7 蓝牙遥控命令接收、解析、限幅和超时保护 |
| `dht11.c/.h` | PC14 单总线读取 DHT11 温湿度 |
| `hx711.c/.h` | PA2/PB12 读取 HX711 24 位称重原始计数 |
| `app_sensors.c/.h` | 温湿度、HX711 原始值、重量换算和 Ozone 状态变量 |
| `display_ui.c/.h` | OLED 四行数据显示和后台刷新 |

Android APP 工程在：

```text
android_bluetooth_remote/
```

CubeMX 生成的主入口在：

```text
Core/Src/main.c
```

其中调用：

```c
(void)BalanceCar_Init();

while (1)
{
    BalanceCar_Background();
}
```

## 3. 编译方法

需要安装 `arm-none-eabi-gcc`，并确保 `make` 可以在终端中使用。

编译：

```bash
make clean
make
```

生成文件：

```text
build/balance_car.elf
build/balance_car.hex
build/balance_car.bin
```

Ozone 调试时建议加载 `build/balance_car.elf`，因为 ELF 中带有调试符号，可以直接看到全局变量名。

## 4. 控制逻辑解析

控制系统由姿态解算、角度环、速度环和转向环组成：

```text
MPU6050
  -> 加速度计角度 angle_acc
  -> 陀螺仪积分角度 angle_gyro
  -> 互补滤波 angle
  -> 角度环 PID
  -> 平均 PWM

编码器
  -> left_speed / right_speed
  -> ave_speed / dif_speed
  -> 速度环 PID 输出角度目标
  -> 转向环 PID 输出差分 PWM
```

### 4.1 10ms 角度环

TIM4 每 1ms 产生节拍，后台每 10ms 执行一次角度环：

```c
Mpu6050_ReadRaw(&raw);
angle_acc = -atan2f(raw.ax, raw.az) / PI * 180;
angle_gyro = s_angle + gy_calibrated / 32768 * 2000 * 0.01;
angle = 0.01 * angle_acc + 0.99 * angle_gyro;
PID_Update(&g_angle_pid);
```

当前代码中的最终控制输出为：

```c
float ave_pwm = g_angle_pid.Out;
```

如果你的车方向相反，可以根据后面的“方向反了怎么办”调整这里的正负号。

### 4.2 50ms 速度环和转向环

后台每 50ms 读取编码器：

```c
left_speed = left_delta / 44.0 / 0.05 / 9.27666;
right_speed = right_delta / 44.0 / 0.05 / 9.27666;
ave_speed = (left_speed + right_speed) / 2.0;
dif_speed = left_speed - right_speed;
```

速度环输出给角度环目标：

```c
g_speed_pid.Actual = ave_speed;
PID_Update(&g_speed_pid);
g_angle_pid.Target = g_speed_pid.Out;
```

转向环输出给差分 PWM：

```c
g_turn_pid.Actual = dif_speed;
PID_Update(&g_turn_pid);
s_dif_pwm = g_turn_pid.Out;
```

最终左右轮 PWM：

```c
left_pwm = ave_pwm + dif_pwm / 2;
right_pwm = ave_pwm - dif_pwm / 2;
```

## 5. Ozone 调试变量

主要看这几个全局变量：

```c
g_balance_debug
g_balance_state
g_sensor_state
g_remote_state
g_remote_debug
g_angle_pid
g_speed_pid
g_turn_pid
```

### 5.1 g_balance_debug

这是 Ozone 中最重要的调试入口，用来启动、停机、设定目标和校准传感器。

| 变量 | 含义 | 调试建议 |
| --- | --- | --- |
| `run_enable` | 0=停机，1=允许输出 PWM | 第一次必须保持 0，确认安全后再改 1 |
| `reset_pid_request` | 写 1 后清空 PID 历史量 | 每次重新调参前可以写 1 |
| `clear_fault_request` | 写 1 后清除故障位 | 硬件故障未解决会再次置位 |
| `speed_target` | 目标前后速度 | 初期保持 0 |
| `turn_target` | 目标转向速度差 | 初期保持 0 |
| `gyro_y_offset` | 陀螺仪 Y 轴零漂 | 静止时观察 `g_balance_state.gy`，把静止平均值填进去 |
| `angle_offset` | 机械竖直角度偏移 | 扶正车后观察 `angle_acc`，按公式修正到接近 0 |
| `fall_angle_limit` | 倒车保护角度 | 默认 50 度 |

`gyro_y_offset` 和 `angle_offset` 都不是通用固定值。不同 MPU6050 模块、不同安装角度、不同车架机械中心都会不一样，必须按自己的车实测后写回 `Core/Src/balance_car/balance_control.c`。

### 5.2 g_balance_state

这是 Ozone 中主要观察的状态变量。

| 变量 | 含义 |
| --- | --- |
| `control_ms` | 1ms 递增，确认控制节拍在运行 |
| `fault_flags` | 故障标志位 |
| `run_active` | 1=当前正在输出控制，0=停机 |
| `imu_ready` | 1=MPU6050 可用 |
| `mpu_id` | MPU6050 ID，正常应为 `0x68` |
| `ax/ay/az` | 加速度计 X/Y/Z 原始值 |
| `gx/gy/gz` | 陀螺仪 X/Y/Z 原始值 |
| `angle_acc` | 只由加速度计算出的俯仰角 |
| `angle_gyro` | 由陀螺仪积分得到的角度 |
| `angle` | 互补滤波后的最终俯仰角，调平衡主要看它 |
| `left_speed/right_speed` | 左右轮速度 |
| `ave_speed` | 左右轮平均速度，速度环实际值 |
| `dif_speed` | 左右轮速度差，转向环实际值 |
| `left_pwm/right_pwm` | 左右电机最终 PWM，范围 -100 到 100 |
| `ave_pwm` | 平均 PWM，主要来自角度环 |
| `dif_pwm` | 差分 PWM，主要来自转向环 |

### 5.3 g_sensor_state

这是 OLED、DHT11 和 HX711 的观察与校准入口。

| 变量 | 含义 |
| --- | --- |
| `temperature_c` | DHT11 温度，单位摄氏度 |
| `humidity_percent` | DHT11 湿度，单位百分比 |
| `hx711_raw` | HX711 24 位有符号原始计数 |
| `weight_g` | 按校准系数换算出的重量，单位 g |
| `hx711_zero_raw` | 空载零点原始计数，可在 Ozone 中手动修正 |
| `hx711_g_per_count` | 每个 HX711 count 对应多少克，可在 Ozone 中手动校准 |
| `sensor_fault_flags` | 传感器和 OLED 故障位 |
| `dht_valid` | 1=DHT11 最近一次读取成功 |
| `hx711_valid` | 1=HX711 最近一次读取成功 |
| `oled_ready` | 1=OLED 初始化成功且正在刷新 |
| `oled_addr_7bit` | OLED 实际使用的 7 位 I2C 地址，正常通常是 `0x3C` 或 `0x3D` |
| `oled_probe_mask` | OLED 地址探测结果，bit0=`0x3C` 有应答，bit1=`0x3D` 有应答 |
| `oled_fail_step` | OLED 初始化失败步骤，1=I2C2 初始化失败，2=地址无应答，3=初始化命令失败，4=首次刷新失败 |

重量换算公式：

```text
weight_g = max(0, (hx711_raw - hx711_zero_raw) * hx711_g_per_count)
```

容易混淆的一点：

```c
g_balance_state.ax
g_balance_state.ay
g_balance_state.az
```

它们不是俯仰角，而是加速度计三个轴的原始值。真正的俯仰角是：

```c
g_balance_state.angle
```

### 5.4 g_remote_state

这是蓝牙遥控的接收状态。调 HC-05/HC-06、串口和 Android APP 时主要看它。

| 变量 | 含义 |
| --- | --- |
| `link_active` | 1=最近 `timeout_ms` 内收到过有效遥控命令 |
| `command_ready` | 1=收到完整命令行并等待后台解析，通常只会短暂出现 |
| `parser_error` | 1=最近一次命令格式错误 |
| `rx_count` | USART1 重映射 PB7 收到的字节数 |
| `valid_cmd_count` | 有效命令计数 |
| `invalid_cmd_count` | 无效命令计数 |
| `timeout_count` | 遥控超时次数 |
| `last_rx_ms` | 最近一次有效命令的系统毫秒时间 |
| `fault_flags` | 遥控模块故障位 |
| `speed_cmd` | 最近一次遥控速度目标，已经过限幅 |
| `turn_cmd` | 最近一次遥控转向目标，已经过限幅 |
| `last_command` | 最近一次完整命令字符串 |

正常现象：

- APP 点按钮或串口助手发命令时，`rx_count` 应该增加。
- 命令格式正确时，`valid_cmd_count` 应该增加。
- 发送 `SPD 0.5` 后，`speed_cmd` 和 `g_balance_debug.speed_target` 应变为 `0.5`。
- 发送 `TURN 0.4` 后，`turn_cmd` 和 `g_balance_debug.turn_target` 应变为 `0.4`。
- 超过 500ms 没收到有效命令时，`link_active` 变 0，`speed_target/turn_target` 自动清零。

### 5.5 g_remote_debug

这是蓝牙遥控功能的调试配置，可以在 Ozone 中临时修改。

| 变量 | 含义 | 默认值 |
| --- | --- | --- |
| `enable` | 1=允许遥控命令改变目标，0=忽略遥控命令 | 1 |
| `allow_run_command` | 1=允许 APP 的 `RUN 1/RUN 0` 控制启停 | 1 |
| `timeout_stop_enable` | 1=遥控超时后自动清零速度和转向目标 | 1 |
| `speed_limit` | 遥控速度目标绝对值限幅 | 1.0 |
| `turn_limit` | 遥控转向目标绝对值限幅 | 0.8 |
| `timeout_ms` | 遥控超时时间，单位 ms | 500 |

调试建议：

- 第一次联调时可以先保持 `run_enable = 0`，只看 `speed_target/turn_target` 是否会跟随 APP 变化。
- 如果你只想用 Ozone 启动，不想让 APP 启动小车，可以把 `allow_run_command = 0`。
- 如果松开 APP 方向键后车还继续走，优先看 `timeout_stop_enable` 是否为 1，以及 `timeout_count` 是否会增加。

### 5.6 三个 PID

| PID | 作用 | 调试顺序 |
| --- | --- | --- |
| `g_angle_pid` | 角度环，让车直立 | 第一个调 |
| `g_speed_pid` | 速度环，让车不乱跑 | 第二个调 |
| `g_turn_pid` | 转向环，控制左右差速 | 最后调 |

PID 字段含义：

| 字段 | 含义 |
| --- | --- |
| `Target` | 目标值 |
| `Actual` | 实际值 |
| `Out` | PID 输出 |
| `Kp` | 比例系数 |
| `Ki` | 积分系数 |
| `Kd` | 微分系数 |
| `Error0` | 当前误差 |
| `ErrorInt` | 误差积分 |
| `OutMax/OutMin` | 输出限幅 |
| `OutOffset` | 输出死区补偿 |

## 6. 第一次调试顺序

不要一开始就让车落地运行。推荐严格按下面顺序来。

### 6.1 只接 STM32 和 MPU6050

先不要接电机电源 `VM`。

在 Ozone 中运行程序，看：

```c
g_balance_state.mpu_id
g_balance_state.imu_ready
g_balance_state.fault_flags
g_balance_state.angle
```

如果使用 Ozone Data Graph，建议先画：

```c
g_balance_state.angle
g_balance_state.angle_acc
g_balance_state.angle_gyro
g_balance_state.gy
```

调好标准：

- `mpu_id == 0x68`
- `imu_ready == 1`
- `fault_flags` 没有 `BALANCE_FAULT_MPU_INIT` 或 `BALANCE_FAULT_MPU_READ`
- 手动前后倾斜车架，`angle` 连续变化，不乱跳
- 车竖直时，`angle` 接近 0
- Data Graph 中 `angle` 曲线应该跟随车体前后倾斜平滑变化，不应突然跳变。
- 静止不动时，`angle_gyro` 不应快速单方向漂移；如果漂移明显，优先调 `gyro_y_offset`。

#### 6.1.1 校准陀螺仪零漂 `gyro_y_offset`

`gyro_y_offset` 调的是 MPU6050 陀螺仪 Y 轴的静态零漂。代码里俯仰角积分使用的是：

```c
gy_calibrated = raw.gy - g_balance_debug.gyro_y_offset;
```

所以校准目标是：车完全静止时，让 `raw.gy - gyro_y_offset` 尽量接近 0。

调试步骤：

1. 保持停机，不让电机输出。

```c
g_balance_debug.run_enable = 0
```

2. 把小车或 MPU6050 固定住，保持完全静止。此时不一定非要直立，关键是不要动、不要振动。

3. 在 Ozone 里观察：

```c
g_balance_state.gy
```

4. 看 `gy` 静止时大概稳定在多少。如果看到它在 `-13, -12, -12, -11, -12` 附近跳动，平均值大概就是 `-12`。

5. 把这个平均值填到：

```c
g_balance_debug.gyro_y_offset = -12.0f
```

也就是说：静止时 `g_balance_state.gy` 平均是多少，`g_balance_debug.gyro_y_offset` 就填多少。

例子：

```text
静止时 gy 大约是 -11.6，gyro_y_offset 就设为 -11.6
静止时 gy 大约是 25，gyro_y_offset 就设为 25
静止时 gy 大约是 -38，gyro_y_offset 就设为 -38
```

验证标准：

- `g_balance_state.gy` 是原始值，调 `gyro_y_offset` 后它不会变，这是正常的。
- `gyro_y_offset` 影响的是 `angle_gyro` 和 `angle` 的长期漂移，不会让角度立刻跳变。
- 静止不动时，看 Ozone Data Graph 里的 `g_balance_state.angle_gyro` 和 `g_balance_state.angle`，它们不应该几秒钟内快速单方向漂移很多度。
- 允许非常慢的小漂移；如果几秒钟就明显跑偏，继续微调 `gyro_y_offset`。

调好后，把最终值写回：

```c
.gyro_y_offset = 你的实测平均值,
```

位置在 `Core/Src/balance_car/balance_control.c` 的 `g_balance_debug` 初始化处。改完后重新编译并烧录，否则单独上电还是旧参数。

#### 6.1.2 校准机械零点 `angle_offset`

`angle_offset` 调的是小车的机械直立零点。目标是：小车真正直立时，程序算出来的俯仰角应该接近 0 度。

代码里加速度角度的计算是：

```c
float angle_acc = -atan2f((float)raw.ax, (float)raw.az) / PI_F * 180.0f;
angle_acc += g_balance_debug.angle_offset;
```

所以 `angle_offset` 的作用是把 `angle_acc` 整体平移。它最直接影响的是：

```c
g_balance_state.angle_acc
```

然后通过互补滤波慢慢影响：

```c
g_balance_state.angle
```

当前滤波系数是 `0.01`，所以改 `angle_offset` 后，`angle_acc` 会立刻变，最终的 `angle` 会慢慢跟过去。调这个参数时，优先看 `angle_acc`，不要只看 `angle`。

调试步骤：

1. 保持停机，不让电机输出。

```c
g_balance_debug.run_enable = 0
```

2. 用手把小车扶到你认为的机械直立点，也就是两个轮子着地、车身刚好能平衡的位置。

3. 在 Ozone 里观察：

```c
g_balance_state.angle_acc
```

4. 使用这个公式修正：

```text
新的 angle_offset = 当前 angle_offset - 当前直立时的 angle_acc
```

例子 1：

```text
当前 angle_offset = 0.5
直立时 angle_acc = +3.0
新的 angle_offset = 0.5 - 3.0 = -2.5
```

例子 2：

```text
当前 angle_offset = 0.5
直立时 angle_acc = -2.0
新的 angle_offset = 0.5 - (-2.0) = 2.5
```

5. 把新的值写到 Ozone 里的：

```c
g_balance_debug.angle_offset
```

6. 再扶正车，继续观察 `g_balance_state.angle_acc`。调好后，直立时它应该接近 0。

验证标准：

- 直立时 `g_balance_state.angle_acc` 最好在 `-0.5 ~ +0.5` 度附近。
- 直立时 `g_balance_state.angle` 应该慢慢靠近 0，通常在 `-1 ~ +1` 度附近比较理想。
- 如果车放倒，`angle` 在接近 `+90` 或 `-90` 度是正常现象。
- 如果车直立时 `angle` 仍然接近 `+90` 或 `-90` 度，说明 MPU6050 安装方向和代码里的角度轴不匹配，应该先检查 `ax/ay/az` 或调整角度计算公式，不建议单纯用 `angle_offset` 硬补 90 度。

调好后，把最终值写回：

```c
.angle_offset = 你的实测修正值,
```

位置同样在 `Core/Src/balance_car/balance_control.c` 的 `g_balance_debug` 初始化处。写回代码后要重新编译并烧录。

### 6.2 架空测试电机

接电机电源前必须把车架空，让轮子离地。

先在 Ozone 中保持：

```c
g_balance_debug.run_enable = 0
```

然后把 PID 临时调小：

```c
g_angle_pid.Kp = 1.0
g_angle_pid.Ki = 0.0
g_angle_pid.Kd = 0.0
g_angle_pid.OutOffset = 0.0
g_speed_pid.Kp = 0.0
g_speed_pid.Ki = 0.0
g_turn_pid.Kp = 0.0
g_turn_pid.Ki = 0.0
```

再把：

```c
g_balance_debug.run_enable = 1
```

调好标准：

- 车往前倒，轮子应该往前追
- 车往后倒，轮子应该往后追
- `left_pwm/right_pwm` 不应突然打满
- 如果方向反了，不要继续加 PID，先修方向

Data Graph 建议观察：

```c
g_balance_state.angle
g_angle_pid.Out
g_balance_state.left_pwm
g_balance_state.right_pwm
```

调好的曲线现象：

- 前后轻轻倾斜车身时，`left_pwm/right_pwm` 会跟着变化。
- PWM 不应该一启动就长期贴着 `+100` 或 `-100`。
- 如果车身倾斜很小但 PWM 立刻打满，先检查角度方向、电机方向或把 `Kp` 降低。

### 6.3 调角度环

只调角度环，速度环和转向环先关掉。

建议顺序：

1. `Ki = 0`
2. `Kd = 0`
3. 慢慢增加 `Kp`
4. 方向正确、有扶正力后增加 `Kd`
5. 最后再加少量 `Ki`
6. 电机小 PWM 不动时再加 `OutOffset`

调好标准：

- 架空时，前后倾斜轮子追车方向正确
- 落地短时间测试时，车能明显抵抗倾倒
- 不会一启动就打满 PWM
- 不会疯狂高频抖动

Data Graph 建议观察：

```c
g_balance_state.angle
g_angle_pid.Target
g_angle_pid.Out
g_balance_state.left_pwm
g_balance_state.right_pwm
```

调好的曲线现象：

- `angle` 能围绕 `g_angle_pid.Target` 附近小幅波动。
- 轻推车身后，`angle` 会偏离，然后能回到目标附近。
- `g_angle_pid.Out` 不长期顶到 `OutMax/OutMin`。
- `left_pwm/right_pwm` 不长期打满 `+100` 或 `-100`。
- 曲线不会越振越大。

异常曲线：

- `angle` 振荡越来越大：方向可能反了，或 `Kp/Kd` 不合适。
- `g_angle_pid.Out` 长期打满：角度环输出饱和，参数太大或方向错误。
- PWM 长期满输出：不要继续加 PID，先检查角度方向和电机方向。

### 6.4 调编码器

用手转轮子，观察：

```c
g_balance_state.left_speed
g_balance_state.right_speed
```

调好标准：

- 左轮往车前进方向转，`left_speed` 为正
- 右轮往车前进方向转，`right_speed` 为正
- 两边速度数值变化连续，不乱跳

如果某边反了，可以交换编码器 A/B 相，或者在代码中给该侧 delta 加负号。

### 6.5 调速度环

角度环能基本直立后，再打开速度环。

先保持：

```c
g_balance_debug.speed_target = 0
```

从小参数开始：

```c
g_speed_pid.Kp = 0.5
g_speed_pid.Ki = 0.0
g_speed_pid.Kd = 0.0
```

观察：

```c
g_balance_state.ave_speed
g_speed_pid.Out
g_angle_pid.Target
```

调好标准：

- `speed_target = 0` 时，车不会持续向一个方向越跑越快
- 轻推车后，速度环会通过改变 `g_angle_pid.Target` 把速度拉回来
- `g_speed_pid.Out` 不应长期打到 `OutMax/OutMin`

Data Graph 建议观察：

```c
g_speed_pid.Target
g_balance_state.ave_speed
g_speed_pid.Out
g_angle_pid.Target
g_balance_state.angle
```

如果图像窗口放得下，也可以加：

```c
g_balance_state.left_speed
g_balance_state.right_speed
```

速度环的数据关系是：

```text
g_speed_pid.Target        -> 目标速度
g_balance_state.ave_speed -> 实际平均速度
g_speed_pid.Out           -> 速度环输出
g_angle_pid.Target        -> 速度环给角度环的目标倾角
```

`speed_target = 0` 时，调好的曲线现象：

- `ave_speed` 围绕 0 上下波动，不会一直偏正或一直偏负。
- `g_speed_pid.Out` 不会一直单方向增大。
- `g_angle_pid.Target` 不会长期顶到速度环输出限幅。
- 车不会自己越跑越远。

给一个小速度目标，例如：

```c
g_balance_debug.speed_target = 0.3f;
```

或：

```c
g_balance_debug.speed_target = -0.3f;
```

调好的曲线现象：

- `ave_speed` 会朝 `g_speed_pid.Target` 的方向靠近。
- `ave_speed` 不一定完全等于目标值，但不能反方向跑。
- `g_speed_pid.Out` 会先变化，然后逐渐收敛。
- `g_angle_pid.Target` 是小幅变化，不应该突然变得很大。
- 车能平稳前进或后退。

异常曲线：

- 目标速度为正，但 `ave_speed` 长期往负方向走：编码器方向、速度环方向或电机方向可能反了。
- `ave_speed` 追目标时振荡越来越大：`Kp` 可能过大，或 `Ki` 加得太早。
- `g_speed_pid.Out` 长期顶到 `+20` 或 `-20`：速度环输出饱和，先减小参数或检查方向。
- `g_angle_pid.Target` 突然很大：速度环给角度环的目标太猛。

### 6.6 调转向环

最后调转向环。

先保持：

```c
g_balance_debug.turn_target = 0
```

从小参数开始：

```c
g_turn_pid.Kp = 1.0
g_turn_pid.Ki = 0.0
g_turn_pid.Kd = 0.0
```

观察：

```c
g_balance_state.dif_speed
g_turn_pid.Out
g_balance_state.dif_pwm
```

调好标准：

- `turn_target = 0` 时，左右轮不应长期有很大的差分输出
- 小幅给 `turn_target` 后，左右轮能产生可控差速
- 车不会因为转向环介入而破坏直立

Data Graph 建议观察：

```c
g_turn_pid.Target
g_balance_state.dif_speed
g_turn_pid.Out
g_balance_state.dif_pwm
g_balance_state.left_speed
g_balance_state.right_speed
g_balance_state.angle
```

转向环的数据关系是：

```text
g_turn_pid.Target         -> 目标左右速度差
g_balance_state.dif_speed -> 实际左右速度差
g_turn_pid.Out            -> 转向环输出
g_balance_state.dif_pwm   -> 最终差分PWM
```

`turn_target = 0` 时，调好的曲线现象：

- `dif_speed` 围绕 0 附近波动。
- `left_speed/right_speed` 不会明显一正一负互相打架。
- `g_turn_pid.Out` 不会长期顶到 `+50` 或 `-50`。

给一个小转向目标，例如：

```c
g_balance_debug.turn_target = 0.3f;
```

调好的曲线现象：

- `dif_speed` 会朝 `g_turn_pid.Target` 的方向变化。
- `left_speed` 和 `right_speed` 会拉开差值。
- 车会产生可控转向。
- 转向时 `angle` 不会明显失控。

异常曲线：

- `turn_target` 为正，但 `dif_speed` 长期往负方向走：转向方向可能反了。
- `g_turn_pid.Out` 长期打满 `+50` 或 `-50`：转向环参数太大或方向错误。
- 转向一介入，`angle` 大幅震荡：转向环太猛，先减小 `Kp/Ki`。

### 6.7 后续循迹调试

当前代码已撤掉循迹模块，只保留平衡车本体控制。后续如果重新加入循迹，先确认模块类型：

- 数字输出循迹模块：输出只有高低电平，可接普通 GPIO。
- 模拟输出循迹模块：输出是连续电压，必须接 ADC 引脚或外部 ADC。

加入循迹前，建议先让角度环、速度环和转向环稳定。循迹代码只负责把循迹偏差转换成：

```c
g_balance_debug.turn_target
```

调试时应观察：

```c
g_balance_debug.turn_target
g_balance_state.dif_speed
g_turn_pid.Out
g_balance_state.angle
g_balance_state.ave_speed
```

### 6.8 OLED、温湿度和重量显示调试

先不要接电机电源，只接 STM32、MPU6050、OLED、DHT11 和 HX711。

在 Ozone 中观察：

```c
g_sensor_state.oled_ready
g_sensor_state.temperature_c
g_sensor_state.humidity_percent
g_sensor_state.hx711_raw
g_sensor_state.hx711_zero_raw
g_sensor_state.hx711_g_per_count
g_sensor_state.weight_g
g_sensor_state.sensor_fault_flags
g_sensor_state.oled_addr_7bit
g_sensor_state.oled_probe_mask
g_sensor_state.oled_fail_step
```

调好标准：

- OLED 每隔约 500ms 更新一次显示。
- DHT11 温湿度每隔约 2s 更新一次。
- 放置或移除砝码时，`hx711_raw` 会连续变化。
- 空载时 `weight_g` 接近 0。

HX711 重量校准步骤：

1. 空载时观察 `g_sensor_state.hx711_raw`，把稳定值写入：

```c
g_sensor_state.hx711_zero_raw
```

2. 放一个已知重量的物体，观察新的 HX711 原始计数。

3. 按下面公式计算：

```text
hx711_g_per_count = 已知重量g / (当前hx711_raw - hx711_zero_raw)
```

4. 把结果写入：

```c
g_sensor_state.hx711_g_per_count
```

如果放上砝码后 `weight_g` 变成 0 或方向相反，说明传感器受力方向或接线让计数反向，可以把 `hx711_g_per_count` 写成负值。

### 6.9 蓝牙遥控调试

蓝牙遥控建议分三步调：先确认 STM32 串口能收命令，再确认手机 APP 能连蓝牙，最后再接电机电源实车测试。

#### 6.9.1 先用 USB-TTL 测 STM32 串口

先不要接电机电源，用 USB-TTL 临时代替蓝牙模块。

| USB-TTL | STM32 |
| --- | --- |
| TXD | PB7 / USART1_RX |
| RXD | PB6 / USART1_TX |
| GND | GND |

串口助手设置：

```text
9600 8N1
```

发送命令时每条后面都要带换行 `\n`：

```text
RUN 1
SPD 0.5
TURN 0.4
STOP
RUN 0
```

Ozone 观察：

```c
g_remote_state.rx_count
g_remote_state.valid_cmd_count
g_remote_state.invalid_cmd_count
g_remote_state.last_command
g_remote_state.speed_cmd
g_remote_state.turn_cmd
g_remote_state.link_active
g_balance_debug.run_enable
g_balance_debug.speed_target
g_balance_debug.turn_target
```

调好标准：

- 串口每发一个字符，`rx_count` 增加。
- 每发一条正确命令，`valid_cmd_count` 增加。
- `last_command` 能看到最近一条命令。
- `RUN 1` 能让 `run_enable` 变 1。
- `RUN 0` 能让 `run_enable` 变 0，并清零速度和转向。
- `SPD 0.5` 能让 `speed_target` 变 0.5。
- `TURN 0.4` 能让 `turn_target` 变 0.4。

如果 `rx_count` 不动，优先检查 PB6/PB7 是否接反、USB-TTL 是否共地、波特率是否是 9600。

#### 6.9.2 手机配对 HC-05/HC-06

先在手机系统蓝牙设置里搜索并配对蓝牙模块。常见名称是 `HC-05` 或 `HC-06`，常见配对码是：

```text
1234
0000
```

调好标准：

- 手机系统蓝牙列表中能看到模块。
- 输入密码后显示已配对。
- 蓝牙模块指示灯通常会从快速闪烁变为慢闪或连接状态闪烁，具体看模块型号。

如果搜不到模块，先只给蓝牙模块供电测试；如果能搜到但配对失败，换 `1234/0000`，并确认模块没有进入 AT 模式。

#### 6.9.3 编译并安装 Android APP

APP 源码在：

```text
android_bluetooth_remote/
```

用 Android Studio 打开这个目录，等待 Gradle Sync 完成，然后连接 Android 手机，点击 Run 安装。

APP 使用流程：

1. 先在手机系统蓝牙里配对 HC-05/HC-06。
2. 打开 APP，界面会固定为横屏实体遥控器面板风格。
3. 点击 `刷新设备`。
4. 选择已配对的 HC-05/HC-06。
5. 点击 `连接`。
6. 点击中间下方的 `START` 发送 `RUN 1`。
7. 左侧摇杆上下控制前进/后退，松开后速度自动归零。
8. 右侧摇杆左右控制左转/右转，松开后转向自动归零。
9. 中间区域会同步显示车上 OLED 的四行内容。
10. 点击右上角 `调试` 切换到 PID 调试界面。
11. 调试界面上半屏显示当前控制环的目标值和实际值双波形，下半屏通过 `速度环`、`角度环`、`转向环` 三个按钮切换 Kp/Ki/Kd 滑杆。
12. 点击中间下方的 `EMERGENCY STOP` 发送 `RUN 0`。

APP 控制对应命令：

| APP 操作 | 发送给 STM32 |
| --- | --- |
| 启动 | `RUN 1` |
| 急停 | `RUN 0` |
| 左摇杆向上 | `SPD 正值` |
| 左摇杆向下 | `SPD 负值` |
| 左摇杆松开 | `SPD 0` |
| 右摇杆向左 | `TURN 正值` |
| 右摇杆向右 | `TURN 负值` |
| 右摇杆松开 | `TURN 0` |
| 速度归零 | `STOP` |
| 调试界面速度环滑杆 | `PID SPD Kp Ki Kd` |
| 调试界面角度环滑杆 | `PID ANG Kp Ki Kd` |
| 调试界面转向环滑杆 | `PID TURN Kp Ki Kd` |
| 清 PID 历史 | `PIDRST` |

STM32 每隔约 500ms 会通过蓝牙回传一行 OLED 数据：

```text
OLED Temp:25.0 C|Humi:60 %|Weight:120 g|HX:123456
```

同时会回传三行 PID 调试数据，APP 调试界面的波形图用这些数据绘制目标值和实际值：

```text
DBG SPD 0.500 0.420
DBG ANG 1.250 1.100
DBG TURN 0.200 0.180
```

APP 收到后会拆成四行显示，尽量和车上 OLED 内容保持一致：

```text
Temp: 25.0 C
Humi: 60 %
Weight: 120 g
HX: 123456
```

如果车的前进后退方向反了，优先改 APP 里发送的 `SPD` 正负号；如果左右转向反了，优先改 APP 里发送的 `TURN` 正负号。不要再动已经调好的角度环、电机方向和编码器方向。

#### 6.9.4 蓝牙和 STM32 联调

接线：

| HC-05/HC-06 | STM32 |
| --- | --- |
| TXD | PB7 / USART1_RX |
| RXD | PB6 / USART1_TX |
| GND | GND |
| VCC | 按模块板标注接 3.3V 或 5V |

Ozone 观察：

```c
g_remote_state.rx_count
g_remote_state.valid_cmd_count
g_remote_state.last_command
g_remote_state.fault_flags
g_balance_debug.speed_target
g_balance_debug.turn_target
g_balance_debug.run_enable
```

调好标准：

- APP 点击启动，`last_command` 显示 `RUN 1`，`run_enable` 变 1。
- 左摇杆向上，`last_command` 显示 `SPD 正值`，`speed_target` 变正。
- 左摇杆向下，`last_command` 显示 `SPD 负值`，`speed_target` 变负。
- 松开左摇杆，`speed_target` 回到 0。
- 右摇杆左右移动，`turn_target` 正负变化。
- 超过 500ms 没有新命令时，`speed_target/turn_target` 自动回 0，但 `run_enable` 不会被强制关掉。

如果 APP 显示已连接但 `rx_count` 不增加，基本就是硬件链路问题：检查 `蓝牙 TXD -> PB7`、`蓝牙 RXD -> PB6`、共地、波特率是否一致。

#### 6.9.5 架空和落地测试

先把车轮架空，再接电机电源。

架空时观察：

```c
g_balance_state.left_pwm
g_balance_state.right_pwm
g_balance_state.left_speed
g_balance_state.right_speed
g_balance_debug.speed_target
g_balance_debug.turn_target
```

调好标准：

- 前进时两个轮子方向一致。
- 后退时两个轮子方向一致且与前进相反。
- 左转/右转时左右轮出现可控差速。
- 松开摇杆后 `speed_target/turn_target` 回到 0。
- PWM 不长期打满。

落地测试时先用手扶住车，只给很小的遥控动作。只要出现越跑越快、方向明显反、角度大幅振荡，立即急停，再回到 Ozone 看变量。

## 7. 方向反了怎么办

方向问题不要同时改多个地方。一次只改一处。

### 7.1 角度方向反了

如果 `angle` 的正负方向和你的车体安装方向相反，可以改：

```c
float angle_acc = -atan2f((float)raw.ax, (float)raw.az) / PI_F * 180.0f;
```

改成：

```c
float angle_acc = atan2f((float)raw.ax, (float)raw.az) / PI_F * 180.0f;
```

如果动态角度又不对，还需要把陀螺仪积分方向一起反：

```c
float angle_gyro = s_angle + gy_calibrated / 32768.0f * 2000.0f * ANGLE_LOOP_PERIOD_S;
```

改成：

```c
float angle_gyro = s_angle - gy_calibrated / 32768.0f * 2000.0f * ANGLE_LOOP_PERIOD_S;
```

### 7.2 两个电机整体反了

如果 `angle` 看起来对，但车往前倒时两个轮子都往后跑，改平均 PWM 符号。

当前代码是：

```c
float ave_pwm = g_angle_pid.Out;
```

可以改成：

```c
float ave_pwm = -g_angle_pid.Out;
```

或者反过来，根据你的车实际方向决定。

### 7.3 只有一个电机反了

如果一个轮子往前、一个轮子往后，改 `motor_tb6612.c` 中对应电机的方向脚逻辑，或者直接交换该侧电机线。

左电机方向由 `AIN1/AIN2` 决定，右电机方向由 `BIN1/BIN2` 决定。

### 7.4 编码器方向反了

如果手动往前转轮子，但速度变量是负的，可以在 `encoder_hal.c` 中把对应返回值取负。

例如左编码器反了：

```c
return delta;
```

改成：

```c
return -delta;
```

## 8. 故障标志

`g_balance_state.fault_flags` 是按位组合的。

| 值 | 含义 | 排查方向 |
| --- | --- | --- |
| `0x00000000` | 无故障 | 正常 |
| `0x00000001` | MPU6050 初始化失败 | 检查 PB8/PB9、供电、地址 |
| `0x00000002` | MPU6050 读取失败 | 检查 I2C 接触、供电干扰 |
| `0x00000004` | 倾角超过保护阈值 | 车倒了，程序自动停机 |
| `0x00000008` | 控制节拍堆积 | 主循环太慢或卡住 |
| `0x00000010` | 电机初始化失败 | 检查 TIM3 PWM/GPIO 初始化 |
| `0x00000020` | 编码器初始化失败 | 检查 TIM1/TIM2 初始化 |
| `0x00000040` | TIM4 初始化失败 | 检查控制节拍定时器 |

清除故障：

```c
g_balance_debug.clear_fault_request = 1;
```

如果硬件问题还在，故障会再次出现。

`g_sensor_state.sensor_fault_flags` 是新增显示和传感器故障位。

| 值 | 含义 | 排查方向 |
| --- | --- | --- |
| `0x00000000` | 无故障 | 正常 |
| `0x00000001` | DHT11 初始化失败 | 检查 PC14、供电和上拉 |
| `0x00000002` | DHT11 读取失败 | 检查数据线、上拉、电源稳定性 |
| `0x00000004` | HX711 初始化失败 | 检查 PA2/PB12 配置 |
| `0x00000008` | HX711 读取失败 | 检查 DT/DOUT、SCK、供电和共地 |
| `0x00000010` | OLED 初始化失败 | 检查 PB10/PB11、地址 0x3C、供电 |
| `0x00000020` | OLED 刷新失败 | 检查 I2C 总线和 OLED 接触 |

OLED 不亮时，优先看：

```c
g_sensor_state.oled_probe_mask
g_sensor_state.oled_fail_step
```

如果 `oled_fail_step == 2` 且 `oled_probe_mask == 0`，说明 PB10/PB11 的 I2C2 总线上没有探测到 `0x3C` 或 `0x3D` OLED，应优先检查 SCL/SDA 是否接反、供电/GND、模块是否真的是 I2C 版本。

`g_remote_state.fault_flags` 是蓝牙遥控模块故障位。

| 值 | 含义 | 排查方向 |
| --- | --- | --- |
| `0x00000000` | 无故障 | 正常 |
| `0x00000001` | USART1 初始化失败 | 检查 HAL UART 是否启用、PB6/PB7 是否被其他外设占用 |
| `0x00000002` | UART 接收中断重启失败 | 检查 USART1 中断和 HAL UART 状态 |
| `0x00000004` | 命令行过长溢出 | APP 或串口助手发送的单条命令超过 31 字节 |
| `0x00000008` | 命令格式错误 | 检查命令是否是 `RUN/SPD/TURN/STOP/PING`，并且是否带换行 |

如果 `rx_count` 增加但 `valid_cmd_count` 不增加，通常是命令格式不对或没有发送 `\n`。

## 9. 脱离 Ozone 后自启动

调试完成前，不建议自启动。默认代码：

```c
.run_enable = 0U
```

如果已经确认：

- MPU6050 方向正确
- 电机方向正确
- 编码器方向正确
- 角度环能稳定直立
- 速度环不会把车越推越快

可以改成：

```c
.run_enable = 1U
```

位置在：

```text
Core/Src/balance_car/balance_control.c
```

这样上电后程序会自动进入平衡控制。

建议自启动时仍保持：

```c
.speed_target = 0.0f
.turn_target = 0.0f
```

## 10. 常见问题

### AX、AY、AZ 哪个是俯仰角？

都不是。

```c
g_balance_state.ax
g_balance_state.ay
g_balance_state.az
```

它们是加速度计三个轴的原始值。

真正的俯仰角是：

```c
g_balance_state.angle
```

其中：

```c
g_balance_state.angle_acc   // 加速度计算出的角度
g_balance_state.angle_gyro  // 陀螺仪积分角度
g_balance_state.angle       // 互补滤波后的最终角度
```

### 为什么找不到 ax/ay/az 的单独定义？

因为它们不是单独的全局变量，而是结构体成员。

结构体定义在：

```text
Core/Inc/balance_car/balance_control.h
```

变量定义在：

```text
Core/Src/balance_car/balance_control.c
```

完整变量名是：

```c
g_balance_state.ax
g_balance_state.ay
g_balance_state.az
```

### 一上电电机不动是不是坏了？

不是。默认：

```c
g_balance_debug.run_enable = 0
```

这是安全设计。需要在 Ozone 中改成：

```c
g_balance_debug.run_enable = 1
```

才允许输出 PWM。

### 为什么 Ozone 要用 ELF？

因为 ELF 里有调试符号，Ozone 可以直接识别：

```c
g_balance_debug
g_balance_state
g_angle_pid
g_speed_pid
g_turn_pid
```

HEX/BIN 只适合烧录，不适合看变量。

### CubeMX 重新生成后要注意什么？

本项目的 I2C、PWM、编码器、TIM4 控制节拍由 `Core/Src/balance_car/` 中的 HAL 初始化代码配置。如果重新用 CubeMX 生成代码，重点检查：

- `main.c` 中是否还调用 `BalanceCar_Init()` 和 `BalanceCar_Background()`
- `stm32f1xx_hal_msp.c` 中是否仍保留 SWD，不要禁用 SWD
- `Makefile` 是否仍包含 `Core/Src/balance_car/*.c`
- `Makefile` 是否仍包含 `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c`
- `stm32f1xx_hal_conf.h` 是否启用了 `HAL_I2C_MODULE_ENABLED`、`HAL_TIM_MODULE_ENABLED` 和 `HAL_UART_MODULE_ENABLED`
- OLED 是否仍接在 PB10/PB11 的 I2C2
- PB6/PB7 是否仍留给 USART1 重映射蓝牙遥控，不要再接实体按键

## 11. 安全建议

- 第一次调试不要接电机电源。
- 接电机电源时必须把车架空。
- 第一次运行时先把 PID 调小。
- 任何异常先把 `g_balance_debug.run_enable` 改为 `0`。
- 不要在方向没确认前加大 `Kp/Kd`。
- 不要同时修改角度方向、电机方向和编码器方向。
- 车能短时间直立后，再逐步调速度环和转向环。
