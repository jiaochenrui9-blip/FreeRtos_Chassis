# FreeRTOS Chassis

基于 STM32F407、FreeRTOS 和 STM32 HAL 的四轮麦克纳姆底盘工程。工程通过 ELRS/CRSF 接收遥控指令，完成四轮运动学解算，并使用速度 PID 控制 4 个 M3508 电机；同时集成 BMI088、IST8310 和姿态解算模块。

## 主要功能

- ELRS/CRSF 遥控数据接收与 16 通道解析
- USART1 空闲中断 + 循环 DMA 接收
- 四轮麦克纳姆底盘运动学解算
- 4 个 M3508 电机注册、反馈解析与在线管理
- M3508 速度 PID 和 CAN 电流指令发送
- BMI088 SPI DMA 采样
- IST8310 磁力计读取
- IMU 姿态更新
- USART6 调试接口
- 文本及二进制调试协议模块

## 控制流程

```text
ELRS 接收机
    │ CRSF
    ▼
USART1 RX DMA
    ▼
ELRS 解析任务
    ▼
底盘指令队列
    ▼
麦克纳姆运动学
    ▼
4 路目标转速
    ▼
M3508 速度 PID
    ▼
CAN1 电流控制帧
```

底盘控制任务每 2 ms 获取最新遥控指令和电机反馈，计算目标转速并发送电流控制帧。停止状态会将目标、电流和 PID 历史状态清零。

## FreeRTOS 任务

| 任务 | 作用 |
| --- | --- |
| `ELRS` | 等待 DMA 接收信号量，解析 CRSF 数据并生成底盘指令 |
| `BMI088_Parse` | 处理 BMI088 DMA 采样、IST8310 数据和姿态更新 |
| `Chassis_Remote` | 读取最新指令，执行运动学、速度 PID 和 CAN 电流发送 |

任务之间使用信号量和消息队列传递接收事件及底盘命令。

## 外设配置

| 外设 | 配置 | 引脚 |
| --- | --- | --- |
| USART1 | ELRS/CRSF，420000 baud，RX 循环 DMA | PB6 TX、PB7 RX |
| USART6 | 调试串口，115200 baud | PG14 TX、PG9 RX |
| CAN1 | M3508，1 Mbps | PD1 TX、PD0 RX |
| SPI1 | BMI088，DMA | PB3 SCK、PB4 MISO、PA7 MOSI |
| BMI088 ACC CS | GPIO | PA4 |
| BMI088 GYRO CS | GPIO | PB0 |
| I2C3 | IST8310 | PA8 SCL、PC9 SDA |

ELRS 接收机 TX 接 PB7；需要遥测时将接收机 RX 接 PB6，并确保共地。

## 代码结构

```text
Core/
├─ Src/                         CubeMX 生成代码及 FreeRTOS 任务
└─ UserModules/
   ├─ Remote/                   ELRS、CRSF、DMA、底盘指令
   ├─ IMU/                      BMI088、IST8310、DMA、姿态
   ├─ Motor/                    CAN、M3508、速度 PID
   ├─ Control/                  通用 PID
   └─ Debug/                    文本和二进制协议
```

## 构建

需要安装 Arm GNU Toolchain、CMake 和 Ninja。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

## 使用前检查

- 确认 ELRS 接收机串口波特率为 420000。
- 确认 CAN 收发器、终端电阻及 M3508 电调 ID 为 1～4。
- 首次上电应架空车轮，并保留可靠的急停方式。
- `CHASSIS_MOTOR_MAX_RPM`、PID 参数和轮序需要根据实际底盘调整。
- 编译通过只验证软件能够构建，不能代替接线、方向、传感器和整车实测。
