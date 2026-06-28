# STM32F103C8T6 Balance Car

这是一个基于 `STM32F103C8T6` 的两轮自平衡小车控制程序，工程由 STM32CubeMX 生成 Makefile 项目后继续开发，适合使用 GCC 工具链编译，并通过 Ozone + J-Link 进行变量观察和在线调参。

项目主要功能包括：

- MPU6050 姿态采样
- 互补滤波计算俯仰角
- 角度环 PID
- 速度环 PID
- 基于 MPU6050 Z 轴角速度反馈的转向环 PID
- TB6612 电机驱动
- 双编码器测速
- HC-05/HC-06 蓝牙串口遥控
- Android 蓝牙遥控 APP
- PC13 运行指示灯
- DHT11 温湿度采集
- XFW-XH711/HX711 称重模块
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

PB6/PB7 当前用于树莓派 UART 通信，不再作为实体按键使用。启停通过 Android APP 的 `START` / `EMERGENCY STOP`、树莓派 `$BB` 控制帧，或 Ozone 修改 `g_balance_debug.run_enable` 完成。PC13 指示灯跟随 `run_enable` 亮灭。

### OLED 显示屏

OLED 已从当前固件中移除，不再初始化或刷新。PB10/PB11 已改给蓝牙 USART3 使用。

### HC-05/HC-06 蓝牙模块

蓝牙模块使用默认 USART3 与 STM32 通信。手机 APP 只发送遥控命令，平衡控制仍由 STM32 完成。

| STM32F103C8T6 | HC-05/HC-06 | 说明 |
| --- | --- | --- |
| PB10 / USART3_TX | RXD | STM32 发给蓝牙模块 |
| PB11 / USART3_RX | TXD | 蓝牙模块发给 STM32 |
| GND | GND | 必须与 STM32、电机电源共地 |
| 3.3V 或 5V | VCC | 按模块板标注供电 |

注意：

- HC-05/HC-06 常见默认串口参数是 `9600 8N1`，当前代码也按 `9600` 配置。
- 如果你的蓝牙模块已经改成 `115200`，需要把 `remote_control.c` 里的 `REMOTE_UART_BAUDRATE` 改成 `115200U`。
- 很多 HC-05/HC-06 模块板的 `VCC` 可以接 5V，但串口电平仍建议按 3.3V 逻辑使用；如果模块 RXD 不耐 5V，需要确认电平安全。
- 手机需要先在系统蓝牙设置里配对模块，常见配对码是 `1234` 或 `0000`。

### 树莓派 UART 通信

树莓派使用 USART1 重映射后的 PB6/PB7 与 STM32 通信。蓝牙仍保留在 USART3 PB10/PB11，二者互不复用。

| STM32F103C8T6 | 树莓派 GPIO | 说明 |
| --- | --- | --- |
| PB6 / USART1_TX | GPIO15 / RXD，物理引脚 10 | STM32 发给树莓派 |
| PB7 / USART1_RX | GPIO14 / TXD，物理引脚 8 | 树莓派发给 STM32 |
| GND | GND | 必须共地 |

串口参数为 `115200 8N1`。树莓派和 STM32 都是 3.3V 串口电平，只连接 TX/RX/GND，不要把树莓派 5V 接到 STM32 串口脚。

Pi 下发文本帧格式：

```text
$BB,<seq>,<enable_motion>,<obstacle_stop>,<linear_x>,<angular_z>*<checksum>
```

`checksum` 为 `$` 和 `*` 之间所有 ASCII 字节的异或值，使用两位十六进制表示。合法帧会把 `linear_x` 按轮半径换算成内部轮速目标，再更新 `g_balance_debug.speed_target`；`angular_z` 会更新 `g_balance_debug.turn_target`。`enable_motion=0` 或 `obstacle_stop=1` 会立即停机并清零目标。超过 500ms 没收到合法帧时，STM32 会自动停机，防止保持旧命令。

STM32 还会周期性回传里程计帧给树莓派：

```text
$BO,<seq>,<stamp_ms>,<x_m>,<y_m>,<yaw_rad>,<linear_x_mps>,<angular_z_rps>*<checksum>
```

当前实现默认每 50ms 发一次，浮点字段保留 4 位小数。`checksum` 只计算 `BO,...` 这一段 payload 的 ASCII 异或，不包括 `$`、`*` 和换行。字段含义是：

- `x/y/yaw`：STM32 内部积分得到的位姿估计
- `vx`：左右轮平均速度换算出的线速度
- `wz`：MPU6050 Z 轴角速度换算值

### DHT11 温湿度模块

| 功能 | STM32 引脚 | DHT11 |
| --- | --- | --- |
| DATA | PC14 | DATA |
| 3.3V | 3.3V | VCC |
| GND | GND | GND |

DHT11 数据脚需要上拉电阻；多数模块板已自带上拉。

### XFW-XH711 称重模块

XFW-XH711 按 HX711 两线数字接口读取，默认使用 A 通道、128 倍增益。

| 功能 | STM32 引脚 | XFW-XH711 |
| --- | --- | --- |
| 数据输出 | PB12 | DT / DOUT |
| 时钟输入 | PB13 | SCK / PD_SCK |
| 3.3V | 3.3V | VCC |
| GND | GND | GND |

称重传感器接 XFW-XH711：

| XFW-XH711 | 称重传感器 |
| --- | --- |
| E+ | E+ |
| E- | E- |
| A+ | S+ / A+ |
| A- | S- / A- |

PA2 / ADC1_IN2 不再作为重量来源，可以不接原压力传感器。

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
| `i2c_bus.c/.h` | I2C1 总线初始化，MPU6050 使用 I2C1 |
| `oled_ssd1306.c/.h` | OLED 驱动源码保留，但当前不参与编译 |
| `remote_control.c/.h` | USART3 PB10/PB11 蓝牙遥控命令接收、解析、限幅和超时保护 |
| `raspi_link.c/.h` | USART1 PB6/PB7 树莓派 `$BB` 控制帧接收、校验、解析和失联停机 |
| `uart_dispatch.c` | HAL UART 全局回调分发，避免蓝牙和树莓派串口互相覆盖 |
| `dht11.c/.h` | PC14 单总线读取 DHT11 温湿度 |
| `hx711.c/.h` | PB12/PB13 读取 XFW-XH711/HX711 称重模块 |
| `app_sensors.c/.h` | 温湿度、XH711 重量计算和 Ozone 状态变量 |
| `display_ui.c/.h` | OLED UI 源码保留，但当前不参与编译 |
| `app_tasks.c/.h` | FreeRTOS 任务调度层，集中创建任务、信号量、队列和任务函数 |
| `FreeRTOSConfig.h` | FreeRTOS 内核配置，包含任务优先级数量、堆大小、栈溢出检测和中断优先级阈值 |

Android APP 工程在：

```text
bluetooth_app/
```

CubeMX 生成的主入口在：

```text
Core/Src/main.c
```

启动流程：

```c
(void)BalanceCar_Init();
AppTasks_Init();
osKernelStart();
```

`main.c` 不再用裸机 `while(1)` 轮询所有模块。外设初始化仍在 `BalanceCar_Init()` 内保持原有模块调用，调度逻辑集中在 `app_tasks.c`。

FreeRTOS 任务划分：

| 任务 | 优先级 | 触发/周期 | 职责 |
| --- | --- | --- | --- |
| `Task_BalanceControl` | `osPriorityRealtime` | TIM4 每 10ms 释放 `Sem_BalanceControl` 唤醒 | 读取 MPU6050、姿态解算、角度环、每 50ms 速度/转向环、电机 PWM 输出和安全停机 |
| `Task_UartRaspberry` | `osPriorityHigh` | 20ms | 处理 USART1 PB6/PB7 树莓派 `$BB/$BO` 通讯 |
| `Task_BluetoothApp` | `osPriorityNormal` | 50ms | 处理 USART3 PB10/PB11 蓝牙 APP 命令和遥测 |
| `Task_SensorCollect` | `osPriorityLow` | 空闲 500ms；DHT11 起始等待期间 20ms 短服务 | 运行 DHT11、XH711 低速采集后台 |

树莓派和蓝牙解析出的控制目标会写入 `Queue_ControlCommand`，PID 参数更新会写入 `Queue_PidUpdate`，都由最高优先级的平衡控制任务统一取出后更新 `g_balance_debug` 和三组 PID。这样通讯和传感器任务不会排在 MPU6050 采样之前。

串口接收采用“中断搬运、任务解析”的方式：

- 蓝牙完整命令行进入 `Queue_BluetoothRxLine`，`Task_BluetoothApp` 逐条解析，避免 50ms 后台周期内多条 APP 命令互相覆盖。
- 树莓派完整帧进入 `Queue_RaspberryRxLine`，`Task_UartRaspberry` 逐条解析。
- 蓝牙和树莓派各自使用独立 TX 互斥量保护发送缓冲区；发送仍使用 `HAL_UART_Transmit_IT()` 非阻塞接口。
- 通讯任务读取姿态、传感器、PID 遥测时通过 `AppTasks_CopyBalanceState()`、`AppTasks_CopySensorState()`、`AppTasks_CopyPidSnapshot()` 获取受保护快照，不直接跨任务读写控制结构体。
- `g_app_task_monitor` 提供四个任务的循环/唤醒计数和最近运行毫秒时间，便于在 Ozone 里确认任务调度仍在运行。
- 蓝牙和树莓派接收都有半帧超时保护，超时未收到换行会丢弃当前缓冲并增加对应 `partial_timeout_count`。

实时性边界：

- 主循环轮询已经移除，通讯、低速传感器和遥测不会排在 10ms 平衡控制前面。
- 为了保持 MPU6050 驱动内部逻辑不重写，当前 `Mpu6050_ReadRaw()` 仍复用 HAL 同步 I2C 读，单次读取带 5ms 超时保护。正常 I2C 通信只占用很短时间；若 I2C 硬件异常导致超时，平衡任务会停机并置 `BALANCE_FAULT_MPU_READ`。
- 如果需要严格做到平衡任务内所有外设访问完全非阻塞，需要进一步允许改造 MPU6050 驱动为 I2C 中断/DMA 状态机；这会超出当前“驱动层内部逻辑不重写”的约束。

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
  -> ave_speed
  -> 速度环 PID 输出角度目标

MPU6050 Z轴陀螺仪
  -> gyro_z_rate
  -> 转向环 PID 输出差分 PWM
```

### 4.1 10ms 角度环

TIM4 每 1ms 产生节拍，每 10ms 在中断中释放 `Sem_BalanceControl`。`Task_BalanceControl` 被唤醒后立即执行角度环：

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

`Task_BalanceControl` 每 5 次 10ms 控制步读取一次编码器，即 50ms 周期：

```c
left_speed = left_delta / 13.0 / 0.05 / 30.0;
right_speed = right_delta / 13.0 / 0.05 / 30.0;
ave_speed = (left_speed + right_speed) / 2.0;
dif_speed = left_speed - right_speed;
```

速度环输出给角度环目标：

```c
g_speed_pid.Actual = ave_speed;
PID_Update(&g_speed_pid);
g_angle_pid.Target = g_speed_pid.Out;
```

转向环使用 MPU6050 的 Z 轴角速度做闭环。`turn_target` 先通过 `turn_gyro_scale` 换算成目标旋转角速度：

```c
turn_rate_target = -g_balance_debug.turn_target * g_balance_debug.turn_gyro_scale;
g_turn_pid.Target = turn_rate_target;
g_turn_pid.Actual = g_balance_state.gyro_z_rate;
PID_Update(&g_turn_pid);
s_dif_pwm = g_turn_pid.Out * 50.0f;
```

`dif_speed` 仍然会计算并保存，用来观察左右轮速度差，但当前转向环的反馈量不是 `dif_speed`，而是 `gyro_z_rate`。

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
| `turn_target` | 目标转向输入，内部乘 `turn_gyro_scale` 变成目标 Z 轴角速度 | 初期保持 0 |
| `gyro_y_offset` | 陀螺仪 Y 轴零漂 | 静止时观察 `g_balance_state.gy`，把静止平均值填进去 |
| `gyro_z_offset` | 陀螺仪 Z 轴零漂 | 静止时观察 `g_balance_state.gz`，把静止平均值填进去 |
| `turn_gyro_scale` | `turn_target` 到 Z 轴目标角速度的比例 | 默认 60，手感太猛就减小，转向太弱就增大 |
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
| `gyro_z_rate` | Z 轴角速度，单位约 deg/s，转向环实际值 |
| `turn_rate_target` | 转向环目标 Z 轴角速度，由 `turn_target * turn_gyro_scale` 换算得到 |
| `left_speed/right_speed` | 左右轮速度 |
| `ave_speed` | 左右轮平均速度，速度环实际值 |
| `dif_speed` | 左右轮速度差，用来辅助观察左右轮差速 |
| `left_pwm/right_pwm` | 左右电机最终 PWM，范围 -100 到 100 |
| `ave_pwm` | 平均 PWM，主要来自角度环 |
| `dif_pwm` | 差分 PWM，主要来自转向环 |

### 5.3 g_sensor_state

这是 DHT11 和 XFW-XH711 的观察与校准入口。OLED 当前已从固件中移除，相关源码保留但不参与编译。

| 变量 | 含义 |
| --- | --- |
| `temperature_c` | DHT11 温度，单位摄氏度 |
| `humidity_percent` | DHT11 湿度，单位百分比 |
| `weight_g` | 最终显示重量，等于 `hx711_weight_g` |
| `hx711_raw` | XH711 原始 24 位有符号读数 |
| `hx711_zero_raw` | 空载零点原始值，可在 Ozone 中手动修正 |
| `hx711_g_per_count` | 每个 XH711 count 对应多少克，可在 Ozone 中手动校准 |
| `hx711_weight_g` | XH711 计算出的重量，单位 g |
| `sensor_fault_flags` | DHT11 和 XH711 故障位 |
| `dht_valid` | 1=DHT11 最近一次读取成功 |
| `dht_fail_step` | DHT11 最近失败阶段，0=正常，1~3=响应阶段失败，4~5=数据位超时，6=校验失败 |
| `hx711_valid` | 1=XH711 最近一次读取成功 |
| `oled_*` | 历史保留字段，当前固件不初始化 OLED，因此不会更新 |

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
| `rx_count` | USART3 PB11 收到的字节数 |
| `valid_cmd_count` | 有效命令计数 |
| `invalid_cmd_count` | 无效命令计数 |
| `timeout_count` | 遥控超时次数 |
| `partial_timeout_count` | 半条命令超过行超时时间后被丢弃次数 |
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
- APP 摇杆连续遥控时发送 `CTL speed turn`，`speed_cmd/turn_cmd` 会同时更新。
- 超过 500ms 没收到有效命令时，`link_active` 变 0，`speed_target/turn_target` 自动清零。

### 5.5 g_app_task_monitor

这是 FreeRTOS 任务运行监测入口，用来确认调度层没有卡死。

| 变量 | 含义 |
| --- | --- |
| `balance_wake_count` | 平衡任务被 TIM4 信号量唤醒的次数 |
| `balance_timeout_count` | 平衡任务 30ms 内未收到控制信号量的次数，非 0 说明控制节拍异常 |
| `balance_last_ms` | 平衡任务最近运行的系统毫秒时间 |
| `raspi_loop_count` / `raspi_last_ms` | 树莓派通信任务循环次数和最近运行时间 |
| `bluetooth_loop_count` / `bluetooth_last_ms` | 蓝牙 APP 任务循环次数和最近运行时间 |
| `sensor_loop_count` / `sensor_last_ms` | 低速传感器任务循环次数和最近运行时间 |

### 5.6 g_remote_debug

这是蓝牙遥控功能的调试配置，可以在 Ozone 中临时修改。

| 变量 | 含义 | 默认值 |
| --- | --- | --- |
| `enable` | 1=允许遥控命令改变目标，0=忽略遥控命令 | 1 |
| `allow_run_command` | 1=允许 APP 的 `RUN 1/RUN 0` 控制启停 | 1 |
| `timeout_stop_enable` | 1=遥控超时后自动清零速度和转向目标 | 1 |
| `speed_limit` | 遥控速度目标绝对值限幅 | 5.0 |
| `turn_limit` | 遥控转向目标绝对值限幅 | 0.8 |
| `timeout_ms` | 遥控超时时间，单位 ms | 500 |

调试建议：

- 第一次联调时可以先保持 `run_enable = 0`，只看 `speed_target/turn_target` 是否会跟随 APP 变化。
- APP 右摇杆界面最大可输出到 `2.0`，STM32 默认 `turn_limit` 为 `0.8`，所以实际进入 `g_balance_debug.turn_target` 的值会先被限制在 `-0.8~+0.8`。需要更大的转向输入时，可以在 Ozone 中调大 `g_remote_debug.turn_limit`，确认稳定后再写回 `remote_control.c`。
- 如果你只想用 Ozone 启动，不想让 APP 启动小车，可以把 `allow_run_command = 0`。
- 如果松开 APP 方向键后车还继续走，优先看 `timeout_stop_enable` 是否为 1，以及 `timeout_count` 是否会增加。

### 5.7 三个 PID

### 5.8 g_raspi_link_state

这是树莓派 UART 链路的接收状态。调 Pi 到 STM32 的 `$BB` 控制帧时主要看它。

| 变量 | 含义 |
| --- | --- |
| `link_active` | 1=最近 `timeout_ms` 内收到过有效 Pi 控制帧 |
| `frame_ready` | 1=收到完整帧并等待后台解析，通常只会短暂出现 |
| `parser_error` | 1=最近一次帧格式或校验错误 |
| `obstacle_stop` | 最近一次有效帧的障碍急停标志 |
| `rx_count` | USART1 PB7 收到的字节数 |
| `valid_frame_count` | 有效 `$BB` 帧计数 |
| `invalid_frame_count` | 无效帧计数 |
| `checksum_error_count` | 校验失败计数 |
| `timeout_count` | Pi 链路失联超时次数 |
| `partial_timeout_count` | 半帧超过行超时时间后被丢弃次数 |
| `last_rx_ms` | 最近一次有效帧的系统毫秒时间 |
| `last_tx_ms` | 最近一次成功发送 `$BO` 的系统毫秒时间 |
| `last_seq` | 最近一次有效帧的序号 |
| `odom_seq` | 最近一次发送的里程计序号 |
| `fault_flags` | Pi 链路故障位 |
| `linear_x_cmd` | 最近一次 Pi 线速度目标，单位 m/s，已经过限幅 |
| `angular_z_cmd` | 最近一次 Pi 角速度目标，已经过限幅 |
| `speed_target_rps` | `linear_x_cmd` 换算后的内部速度环目标，单位轮子输出轴转/秒 |
| `speed_actual_rps` | 编码器测得的内部实际速度，单位轮子输出轴转/秒 |
| `speed_actual_mps` | 编码器测得的实际线速度，单位 m/s |
| `left_pwm_snapshot` | 最近一次记录的左电机 PWM |
| `right_pwm_snapshot` | 最近一次记录的右电机 PWM |
| `odom_x_m` | 回传给树莓派的里程计 X |
| `odom_y_m` | 回传给树莓派的里程计 Y |
| `odom_yaw_rad` | 回传给树莓派的里程计 yaw |
| `odom_linear_x_mps` | 回传给树莓派的线速度 |
| `odom_angular_z_rps` | 回传给树莓派的角速度 |
| `last_frame` | 最近一次完整帧字符串 |
| `last_tx_frame` | 最近一次发送给树莓派的 `$BO` 帧 |

正常现象：

- 串口助手或 Pi 发送 `$BB` 帧并带 `\n` 时，`rx_count` 应该增加。
- 校验正确时，`valid_frame_count` 增加，`linear_x_cmd/angular_z_cmd` 更新。
- `tx_count` 应该持续增长，`last_tx_frame` 会显示 `$BO,...`。
- `enable_motion=0` 或 `obstacle_stop=1` 时，`g_balance_debug.run_enable` 变 0，速度和转向目标清零。
- 超过 500ms 没收到有效帧时，`timeout_count` 增加，平衡车停机。

### 5.9 g_raspi_link_debug

这是树莓派 UART 链路的调试配置，可以在 Ozone 中临时修改。

| 变量 | 含义 | 默认值 |
| --- | --- | --- |
| `enable` | 1=允许 Pi 控制帧改变目标，0=只接收不执行 | 1 |
| `timeout_stop_enable` | 1=Pi 失联超时后自动停机 | 1 |
| `allow_run_enable` | 1=合法 Pi 帧允许把 `run_enable` 置 1 | 1 |
| `speed_limit` | Pi 线速度目标绝对值限幅 | 3.0 |
| `turn_limit` | Pi 角速度目标绝对值限幅 | 2.0 |
| `wheel_radius_m` | 用于把轮速换成线速度的轮半径，65mm 直径对应 0.0325m | 0.0325 |
| `odom_angular_deadband_rps` | 里程计 yaw 积分角速度死区，小于该值按 0 处理 | 0.02 |
| `timeout_ms` | Pi 链路失联超时时间，单位 ms | 500 |
| `odom_period_ms` | `$BO` 里程计回传周期，单位 ms | 50 |

第一次联调时可以把 `allow_run_enable` 设为 0，只观察 `linear_x_cmd`、`speed_target_rps`、`speed_actual_mps` 和 `left_pwm_snapshot/right_pwm_snapshot` 是否合理，确认车架空和控制方向正确后再允许 Pi 启动。65mm 轮径下，`linear_x=0.02m/s` 对应内部速度目标约 `0.098rps`。

如果车完全静止但 `/odom` 的 yaw 缓慢漂移，先校准 `g_balance_debug.gyro_z_offset`，让 `g_balance_state.gyro_z_rate` 静止时接近 0；仍有轻微漂移时，可以适当调大 `g_raspi_link_debug.odom_angular_deadband_rps`。写 `g_raspi_link_debug.odom_reset_request = 1` 可以清零当前 `x/y/yaw`。

### 5.8 三个 PID

| PID | 作用 | 调试顺序 |
| --- | --- | --- |
| `g_angle_pid` | 角度环，让车直立 | 第一个调 |
| `g_speed_pid` | 速度环，让车不乱跑 | 第二个调 |
| `g_turn_pid` | 转向环，控制 Z 轴旋转角速度 | 最后调 |

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

#### 6.1.2 校准 Z 轴陀螺仪零漂 `gyro_z_offset`

`gyro_z_offset` 调的是 MPU6050 陀螺仪 Z 轴的静态零漂。当前转向环使用 Z 轴角速度闭环，代码里使用的是：

```c
gz_calibrated = raw.gz - g_balance_debug.gyro_z_offset;
gyro_z_rate = gz_calibrated / 32768.0f * 2000.0f;
```

校准目标是：车完全静止时，让 `gyro_z_rate` 尽量接近 0。

调试步骤：

1. 保持停机，不让电机输出。

```c
g_balance_debug.run_enable = 0
```

2. 把小车或 MPU6050 固定住，保持完全静止，不要用手晃动车体。

3. 在 Ozone 里观察：

```c
g_balance_state.gz
```

4. 看 `gz` 静止时大概稳定在多少，把这个平均值填到：

```c
g_balance_debug.gyro_z_offset
```

例子：

```text
静止时 gz 大约是 95，gyro_z_offset 就设为 95
静止时 gz 大约是 -140，gyro_z_offset 就设为 -140
```

验证标准：

- `g_balance_state.gz` 是原始值，调 `gyro_z_offset` 后它不会变，这是正常的。
- 真正应该变化的是 `g_balance_state.gyro_z_rate`。
- 静止时 `g_balance_state.gyro_z_rate` 应该接近 0。
- 原地转动车体时，`gyro_z_rate` 应明显正负变化，方向要和 `turn_rate_target` 的调试方向一致。

调好后，把最终值写回：

```c
.gyro_z_offset = 你的实测平均值,
```

位置在 `Core/Src/balance_car/balance_control.c` 的 `g_balance_debug` 初始化处。改完后重新编译并烧录。

#### 6.1.3 校准机械零点 `angle_offset`

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

最后调转向环。当前转向环用 MPU6050 的 Z 轴陀螺仪角速度做闭环，目标是让车的实际旋转角速度 `gyro_z_rate` 跟随目标旋转角速度 `turn_rate_target`。

先校准 Z 轴陀螺仪零漂。车完全静止时观察：

```c
g_balance_state.gz
```

把静止平均值填入：

```c
g_balance_debug.gyro_z_offset
```

注意：

```c
g_balance_state.gz
```

是 MPU6050 的 Z 轴原始值，填了 `gyro_z_offset` 后它不会变。真正受零漂修正影响的是：

```c
g_balance_state.gyro_z_rate
```

静止时 `gyro_z_rate` 越接近 0 越好。

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
g_balance_state.gyro_z_rate
g_balance_state.turn_rate_target
g_turn_pid.Target
g_turn_pid.Actual
g_balance_state.dif_speed
g_turn_pid.Out
g_balance_state.dif_pwm
```

调好标准：

- `turn_target = 0` 时，`gyro_z_rate` 应围绕 0 附近小幅波动
- 小幅给 `turn_target` 后，`gyro_z_rate` 应朝 `turn_rate_target` 的方向变化
- 左右轮能产生可控差速，`dif_pwm` 不长期打满
- 车不会因为转向环介入而破坏直立

Data Graph 建议观察：

```c
g_turn_pid.Target
g_turn_pid.Actual
g_balance_state.gyro_z_rate
g_balance_state.turn_rate_target
g_turn_pid.Out
g_balance_state.dif_pwm
g_balance_state.dif_speed
g_balance_state.left_speed
g_balance_state.right_speed
g_balance_state.angle
```

转向环的数据关系是：

```text
g_balance_debug.turn_target      -> APP/Ozone给的转向输入
g_balance_debug.turn_gyro_scale  -> 转向输入到Z轴目标角速度的比例
g_balance_state.turn_rate_target -> 目标Z轴角速度
g_balance_state.gyro_z_rate      -> 实际Z轴角速度
g_turn_pid.Out                   -> 转向环输出
g_balance_state.dif_pwm          -> 最终差分PWM
```

`turn_target = 0` 时，调好的曲线现象：

- `turn_rate_target` 为 0。
- `gyro_z_rate` 围绕 0 附近小幅波动。
- `g_turn_pid.Out` 不会长期顶到 `OutMax/OutMin`。
- `left_speed/right_speed` 不会明显一正一负互相打架。

给一个小转向目标，例如：

```c
g_balance_debug.turn_target = 0.3f;
```

调好的曲线现象：

- `turn_rate_target` 会变成 `turn_target * turn_gyro_scale` 对应的目标角速度。
- `gyro_z_rate` 会朝 `turn_rate_target` 的方向变化。
- `left_speed` 和 `right_speed` 会拉开差值。
- 车会产生可控转向。
- 转向时 `angle` 不会明显失控。

异常曲线：

- `turn_target` 为正，但 `gyro_z_rate` 长期往反方向走：转向方向可能反了。
- `g_turn_pid.Out` 长期打满 `OutMax/OutMin`：转向环参数太大、方向错误，或 `turn_gyro_scale` 太大。
- 转向一介入，`angle` 大幅震荡：转向环太猛，先减小 `Kp/Ki`。

转向方向反时，优先检查 `balance_control.c` 里的这一行：

```c
float turn_rate_target = -g_balance_debug.turn_target * g_balance_debug.turn_gyro_scale;
```

如果正负号和实车相反，只改这里的负号，不要同时改电机方向、编码器方向和 APP 方向。

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

### 6.8 温湿度和重量采集调试

先不要接电机电源，只接 STM32、MPU6050、DHT11 和 XFW-XH711。OLED 当前不参与编译，PB10/PB11 留给 USART3 蓝牙。

在 Ozone 中观察：

```c
g_sensor_state.temperature_c
g_sensor_state.humidity_percent
g_sensor_state.dht_valid
g_sensor_state.dht_fail_step
g_sensor_state.hx711_valid
g_sensor_state.hx711_raw
g_sensor_state.hx711_zero_raw
g_sensor_state.hx711_g_per_count
g_sensor_state.hx711_weight_g
g_sensor_state.weight_g
g_sensor_state.sensor_fault_flags
```

调好标准：

- DHT11 温湿度每隔约 2s 更新一次。
- 按压称重传感器时，`hx711_raw` 连续变化。
- 模块接好并正常出数时，`hx711_valid == 1`。
- 空载时 `weight_g` 接近 0。

XH711 重量校准步骤：

1. 空载时观察 `g_sensor_state.hx711_raw`，把稳定值写入：

```c
g_sensor_state.hx711_zero_raw
```

2. 放一个已知重量的物体，观察新的 `hx711_raw`。

3. 按下面公式计算：

```text
hx711_g_per_count = 已知重量g / (当前hx711_raw - hx711_zero_raw)
```

4. 把结果写入：

```c
g_sensor_state.hx711_g_per_count
```

5. 校准稳定后，把 `hx711_zero_raw` 和 `hx711_g_per_count` 写回 `Core/Src/balance_car/app_sensors.c` 的 `g_sensor_state` 默认值。

如果放上重量后 `hx711_raw` 变小，说明传感器方向或接线极性相反，可以交换 A+/A-，也可以使用负的 `hx711_g_per_count`。

### 6.9 蓝牙遥控调试

蓝牙遥控建议分三步调：先确认 STM32 串口能收命令，再确认手机 APP 能连蓝牙，最后再接电机电源实车测试。

#### 6.9.1 先用 USB-TTL 测 STM32 串口

先不要接电机电源，用 USB-TTL 临时代替蓝牙模块。

| USB-TTL | STM32 |
| --- | --- |
| TXD | PB11 / USART3_RX |
| RXD | PB10 / USART3_TX |
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
CTL 0.5 0.4
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
- `CTL 0.5 0.4` 能同时让 `speed_target` 变 0.5、`turn_target` 变 0.4。

如果 `rx_count` 不动，优先检查 PB10/PB11 是否接反、USB-TTL 是否共地、波特率是否是 9600。

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
bluetooth_app/
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
9. 中间区域会同步显示 STM32 回传的传感器四行遥测。
10. 点击中间下方的 `EMERGENCY STOP` 发送 `RUN 0`。

APP 遥控输出：

```text
左侧速度摇杆范围: -3.0 ~ +3.0
右侧转向摇杆范围: -2.0 ~ +2.0
遥控发送周期: 50ms
连续遥控命令: CTL speed turn
```

APP 控制对应命令：

| APP 操作 | 发送给 STM32 |
| --- | --- |
| 启动 | `RUN 1` |
| 急停 | `RUN 0` |
| 左摇杆向上 | `CTL 正速度 当前转向` |
| 左摇杆向下 | `CTL 负速度 当前转向` |
| 左摇杆松开 | `CTL 0 当前转向` |
| 右摇杆向左/向右 | `CTL 当前速度 当前转向` |
| 右摇杆松开 | `CTL 当前速度 0` |

STM32 每隔约 500ms 会通过蓝牙回传一行传感器数据。行首仍使用 `OLED`，只是为了兼容 APP 现有解析协议，不代表当前固件还驱动车上 OLED：

```text
OLED Temp:25.0 C|Humi:60 %|Weight:120 g|Raw:123456
```

APP 收到后会拆成四行显示：

```text
Temp: 25.0 C
Humi: 60 %
Weight: 120 g
Raw: 123456
```

如果车的前进后退方向反了，优先改 APP 里速度摇杆的正负号；如果左右转向反了，优先改 `balance_control.c` 中 `turn_rate_target` 那一行的正负号。不要同时改角度环、电机方向、编码器方向和 APP 方向。

#### 6.9.4 蓝牙和 STM32 联调

接线：

| HC-05/HC-06 | STM32 |
| --- | --- |
| TXD | PB11 / USART3_RX |
| RXD | PB10 / USART3_TX |
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
- 左摇杆向上，`last_command` 显示 `CTL`，`speed_target` 变正。
- 左摇杆向下，`last_command` 显示 `CTL`，`speed_target` 变负。
- 松开左摇杆，`speed_target` 回到 0。
- 右摇杆左右移动，`turn_target` 正负变化。
- 超过 500ms 没有新命令时，`speed_target/turn_target` 自动回 0，但 `run_enable` 不会被强制关掉。

如果 APP 显示已连接但 `rx_count` 不增加，基本就是硬件链路问题：检查 `蓝牙 TXD -> PB11`、`蓝牙 RXD -> PB10`、共地、波特率是否一致。

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
| `0x00000008` | 控制节拍堆积 | 平衡任务未及时消耗 TIM4 控制信号，检查中断、任务优先级或高优先级阻塞 |
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
| `0x00000004` | XH711 初始化失败 | 检查 PB12/PB13 GPIO 配置 |
| `0x00000008` | XH711 读取失败 | 检查 DT/SCK、供电、GND、称重传感器接线 |
| `0x00000010` | OLED 历史保留位 | 当前固件未编译 OLED 驱动，不会主动置位 |
| `0x00000020` | OLED 历史保留位 | 当前固件未编译 OLED 驱动，不会主动置位 |

`g_remote_state.fault_flags` 是蓝牙遥控模块故障位。

| 值 | 含义 | 排查方向 |
| --- | --- | --- |
| `0x00000000` | 无故障 | 正常 |
| `0x00000001` | USART3 初始化失败 | 检查 HAL UART 是否启用、PB10/PB11 是否被其他外设占用 |
| `0x00000002` | UART 接收中断重启失败 | 检查 USART3 中断和 HAL UART 状态 |
| `0x00000004` | 命令行过长溢出 | APP 或串口助手发送的单条命令超过 31 字节 |
| `0x00000008` | 命令格式错误 | 检查命令是否是 `RUN/SPD/TURN/CTL/STOP/PING`，并且是否带换行 |

如果 `rx_count` 增加但 `valid_cmd_count` 不增加，通常是命令格式不对或没有发送 `\n`。

`g_raspi_link_state.fault_flags` 是树莓派 UART 链路故障位。

| 值 | 含义 | 排查方向 |
| --- | --- | --- |
| `0x00000000` | 无故障 | 正常 |
| `0x00000001` | USART1 初始化失败 | 检查 HAL UART、PB6/PB7 remap 和时钟配置 |
| `0x00000002` | UART 接收中断重启失败 | 检查 USART1 中断和 HAL UART 状态 |
| `0x00000004` | 帧过长溢出 | Pi 单帧超过 79 字节或没有及时换行 |
| `0x00000008` | 帧格式错误 | 检查是否为 `$BB,<seq>,<enable>,<stop>,<linear>,<angular>*CS` |
| `0x00000010` | 校验错误 | 检查 checksum 是否为 `$` 和 `*` 之间 ASCII 字节异或 |
| `0x00000020` | Pi 链路失联超时 | 检查 Pi 是否持续发送有效帧、波特率是否为 115200 |

如果 `rx_count` 增加但 `valid_frame_count` 不增加，优先检查是否带 `\n`、checksum 是否正确、TX/RX 是否接反。

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

本项目的 I2C、PWM、编码器、TIM4 控制节拍由 `Core/Src/balance_car/` 中的 HAL 初始化代码配置，FreeRTOS 调度层由 `app_tasks.c/.h` 和 `FreeRTOSConfig.h` 管理。如果重新用 CubeMX 生成代码，重点检查：

- `main.c` 中是否还调用 `BalanceCar_Init()`、`AppTasks_Init()` 和 `osKernelStart()`
- `Makefile` 是否仍包含 `Core/Src/balance_car/app_tasks.c`
- `Makefile` 是否仍包含 `Middlewares/Third_Party/FreeRTOS/Source/*.c`、`CMSIS_RTOS/cmsis_os.c`、`portable/GCC/ARM_CM3/port.c` 和 `portable/MemMang/heap_4.c`
- `Makefile` 是否仍包含 FreeRTOS include 路径
- `stm32f1xx_it.c` 中不要重新生成空的 `SVC_Handler`、`PendSV_Handler`、`SysTick_Handler` 覆盖 FreeRTOS 端口
- `stm32f1xx_hal_msp.c` 中是否仍保留 SWD，不要禁用 SWD
- `Makefile` 是否仍包含 `Core/Src/balance_car/*.c`
- `Makefile` 是否仍包含 `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c`
- `stm32f1xx_hal_conf.h` 是否启用了 `HAL_I2C_MODULE_ENABLED`、`HAL_TIM_MODULE_ENABLED` 和 `HAL_UART_MODULE_ENABLED`
- OLED 是否仍保持禁用，不参与编译
- PB10/PB11 是否仍留给 USART3 蓝牙，不要再接 OLED I2C2
- PB12/PB13 是否仍留给 XFW-XH711，不要被其它外设占用

## 11. 安全建议

- 第一次调试不要接电机电源。
- 接电机电源时必须把车架空。
- 第一次运行时先把 PID 调小。
- 任何异常先把 `g_balance_debug.run_enable` 改为 `0`。
- 不要在方向没确认前加大 `Kp/Kd`。
- 不要同时修改角度方向、电机方向和编码器方向。
- 车能短时间直立后，再逐步调速度环和转向环。
