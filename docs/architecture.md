# 底盘架构与开发约定

2026-11-21 确认。目标硬件为四轮灵风底盘和 STM32F103RCT6 运动控制板。本文约定每项控制和标定工作由哪一端负责。主机接入细节见[底盘接口约定](base-interface.md)。

## 控制职责

| 归属 | 负责的事情 | 参数放在哪里 |
| --- | --- | --- |
| STM32 固件 | 同周期采集四路编码器，各轮独立闭环调速。执行使能、停机和命令超时保护，采集电源电压及 MPU6050 六轴数据。 | 板端保存插座与车轮的映射、方向极性、输出轴每圈计数、各轮调速参数和输出上限。按 IMU 实际配置的量程换算物理单位。 |
| 上位机通信与联调工具 | 通过板载 USB 转串口下发轮端目标，显示与记录四轮状态、电压和带采样时间的六轴数据。首版所有运动指令均从这条链路进入。 | 主机分别维护底盘状态和 IMU 数据的新鲜度。手柄等输入设备由上位机接收。 |
| 上位机整车控制 | 将前进速度和转向速度换算为左右轮组的目标，根据实际轮子转角计算里程计。 | 有效滚动半径、滑移转向的有效轮距及里程计修正量由整车控制使用。 |
| 未来的 ROS 2 接入 | 底盘作为一个硬件组件暴露四个轮关节，机械臂使用独立组件。上位机负责 IMU 安装变换与姿态融合。 | 本期提供轮关节及六轴消息映射。采用 `ros2_control` 后实现 `SystemInterface`，复用底盘通信代码。 |

差速控制器支持同侧多个车轮，输入车体速度，输出轮关节速度，可用关节位置反馈计算里程计。Husarion Panther 的四驱实车按左右两组配置四个关节，并单独校正有效轮距。这是本项目选择轮端接口的依据。[ROS 2 差速控制器](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) · [Panther 四轮配置](https://github.com/husarion/husarion_ugv_ros/blob/ros2/husarion_ugv_controller/config/WH01_controller.yaml)

## 板级资源与实车核对

### 四轮接线

按装车照片看，两块红色电机驱动板在上，双层 USB 座在下。表中“照片位置”指白色六针插座在图中的位置，“车轮”按车头朝前判定。车轮来自照片标注，插座编号和 MCU 引脚来自图纸。实车逐轮核对后，在标定记录中填写转向和计数符号。[装车接线原图](assets/chassis-wiring.png) · [随车说明，第 7～8 页](四轮灵风小车资料/四轮小车资料简介.pdf#page=7) · [运动控制板原理图](四轮灵风小车资料/原理图.pdf)

| 协议下标 / 车轮 | 照片位置 | 六针插座 | PWM | 方向 GPIO / 原理图网络 | 编码器 CH1 / CH2 |
| --- | --- | --- | --- | --- | --- |
| 0 左前 | 右上 | M1 / JP2 | TIM8_CH4 · PC9 | PC4 · `M4_ENB` | TIM4 · PB6 / PB7 |
| 1 右前 | 右下 | M2 / JP5 | TIM8_CH3 · PC8 | PC13 · `M3_ENB` | TIM2 · PA15 / PB3 |
| 2 左后 | 左上 | M3 / JP4 | TIM8_CH2 · PC7 | PC14 · `M2_ENB` | TIM5 · PA0 / PA1 |
| 3 右后 | 左下 | M4 / JP3 | TIM8_CH1 · PC6 | PC15 · `M1_ENB` | TIM3 · PA6 / PA7 |

JP5 的 3、4 脚依次接 CH2、CH1，其余三个插座依次接 CH1、CH2。固件按表分配通道。TIM2 选择部分重映射 1，调试端口选择 Serial Wire，释放 PA15、PB3 并保留 PA13/SWDIO、PA14/SWCLK。PC13～PC15 作为低速方向输出，PC14、PC15 已用于电机，LSE 保持关闭。[STM32F103xC 数据手册](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf) · [STM32F1 参考手册](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101-stm32f102-stm32f103-stm32f105-and-stm32f107-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

旧工程的 TIM8 采用高有效 PWM1，`PSC=2`、`ARR=1999`，初始化和四轮零目标都写 `CCR=2000`。按 72 MHz 定时器时钟计算，PWM 频率为 12 kHz，写入该值后已使能的输出脚为持续高电平。新固件接入驱动后，在四轮架空、电机接线保留的状态下核对复位、未使能和停机时的 PWM 与方向组合。观察未使能时车轮能否保持静止，以及运动中停机时的滑行或制动表现。[厂商 TIM8 初始化](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/tim8.c) · [PWM 通道和上限](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Inc/MotorControl.h) · [厂商零目标处理](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/VelocityControl.c)

### 电源采样与 USB

板上外部晶振为 8 MHz，设计采用 PLL ×9，HCLK 和 PCLK2 为 72 MHz，PCLK1 为 36 MHz。ADC1 选 PCLK2 ÷6，得到 12 MHz。旧工程的 ÷4 在同一时钟树下会得到 18 MHz，超过该芯片 14 MHz 的 ADC 上限。[厂商时钟初始化](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/main.c) · [STM32F103xC 数据手册](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)

电源经 R2（68 kΩ）和 R5（10 kΩ）分压到 PA4/ADC1_IN4。固件启动时校准 ADC，每次采 PA4 后自动采内部参考 VREFINT。输入电压估算为 `1200 mV × 7.8 × PA4 读数 / VREFINT 读数`，其中 1200 mV 为参考标称值。绝对精度待输入端万用表同步比对。[电压采样实现](../firmware/Core/Src/base_feedback.c) · [运动控制板原理图](四轮灵风小车资料/原理图.pdf)

USART2 经 PA2/TX、PA3/RX 接 CH340N。双层 USB 座的上层将 D+、D− 和电脑提供的 `USB_5V` 接至 CH340N，下层给外设输出 5 V。用适配本板的 USB-A 公对公数据线将电脑 USB-A 口接到上层，枚举出串口后以 115200 bit/s 联调。SWD 使用单独的调试引脚。[随车说明，第 2、5 页](四轮灵风小车资料/四轮小车资料简介.pdf) · [运动控制板原理图](四轮灵风小车资料/原理图.pdf)

### 已安装的 MPU6050

照片中央蓝色模块标为 GY-521。按板图，MPU6050 使用 U7，针 1～4 依次为 3.3 V、GND、SCL、SDA，针 8 为 INT。I2C1 重映射到 PB8/SCL、PB9/SDA，数据就绪信号接 PA5。相邻 U6 是针序不同的 GY-85 插座。首次上电前核对模块 VCC/GND 与 U7 针 1/2 的方向。[装车接线原图](assets/chassis-wiring.png) · [随车说明，第 6 页](四轮灵风小车资料/四轮小车资料简介.pdf#page=6) · [运动控制板原理图](四轮灵风小车资料/原理图.pdf)

上电后在 7 位地址 `0x68`、`0x69` 中确认 AD0 选中的地址，读寄存器 `0x75`（WHO_AM_I），两种地址的预期返回值都是 `0x68`。旧 HAL 代码中的 `0xD0` 是 `0x68` 左移一位后的参数。[厂商驱动地址定义](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Inc/mpu6050.h)

用数据就绪事件记录板端时间，一次连续读取 `0x3B～0x48` 的 14 字节。每个有符号 16 位数高字节在前，跳过中间的两字节温度。按实际量程换算六轴读数，再将加速度从 g 乘 9.80665 转为 m/s²，角速度从 °/s 乘 π/180 转为 rad/s。联调时记录量程、实际采样率和安装朝向。[MPU6050 寄存器手册](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf)

六轴数据保留传感器自身的右手坐标轴。板端只发送有效新样本。I²C 读取有时限，读取失败跳过该样本，串口发送拥塞时优先保留轮状态反馈。上位机记录传感器相对车身的安装变换，姿态解算可以与轮式里程计在主机上融合。[底盘接口约定](base-interface.md) · [ROS IMU 坐标约定](https://reps.openrobotics.org/rep-0145/)

## 固件组织

- 板级代码集中管理时钟、引脚、定时器、ADC 和串口。CubeMX 生成初始化代码。手写的控制和编解码逻辑使用 C，将输入、输出与状态写清楚，使同一份源码能在电脑和目标板上测试。
- 裸机按固定周期采样四轮并更新控制输出。四个轮子共用一份调速函数，各自保留参数和积分状态。初始控制方案采用带输出限幅和抗积分饱和的 PI，依据实测曲线调参。
- 编码器连续累计正反向位移，并处理硬件计数器回绕。四个轮子的角度和速度使用同一个采样时刻。串口收发和遥测不阻塞轮速更新。
- 板端用自身时钟判断运动指令是否过期。停机时清除旧目标和调速器积累的输出。通信工具和未来的 ROS 插件复用一份协议编解码实现。

松灵 Scout 将底盘通信与 ROS 封装分开，支持独立联调。Linorobot2 则在单片机里接收 ROS 车速指令并计算里程计。本项目选择独立 USB 接口，把 ROS 依赖和整车几何留在上位机。[Scout SDK](https://github.com/agilexrobotics/ugv_sdk) · [Scout ROS 2 驱动](https://github.com/agilexrobotics/scout_ros2) · [Linorobot2 固件](https://github.com/linorobot/linorobot2_hardware)

## 开发环境与版本

主开发环境选 Windows 原生。用经典 STM32CubeMX 配置 F1 系列外设，使用 STM32CubeF1 HAL、Arm GCC 和 CMake/Ninja 构建。VS Code 与终端共用 CMake 预设，烧录和断点调试走 SWD。将 `.ioc`、生成的初始化代码、CMake 预设和工具版本锁定文件纳入版本管理，普通构建直接使用仓库中的源文件。[ST 配置工具](https://www.st.com/content/st_com/en/stm32cubemx.html) · [STM32CubeF1](https://github.com/STMicroelectronics/STM32CubeF1) · [ST 可复现构建示例](https://dev.st.com/stm32cube-docs/stm32cubecli/latest/en/docs/markup/STM32CubeCLI_Getting_Started/STM32CubeCLI_First_Project.html)

Keil µVision 支持命令行构建，旧工程可用于查阅原厂实现。新固件只维护一套 CMake 构建配置，板上交叉编译和电脑端逻辑测试共用项目中的手写源码。[µVision 命令行](https://www.keil.com/support/man/docs/uv4cl/uv4cl_commandline.htm)

## 标定与验收依据

随车图纸标注轮径约 152.4 mm，旧代码中轮周长为 0.496371 m，折合轮径约 158.0 mm，两者相差约 3.7%。整车里程计使用实际行驶测得的有效滚动周长。四轮滑移转向的有效轮距通过实际转角标定，并记录地面与载荷条件。[底盘尺寸，资料第 1 页](四轮灵风小车资料/四轮小车资料简介.pdf) · [原厂运动学参数](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Inc/Kinematics_4WD.h)

编码器沿用厂商默认值，输出轴每圈 144000 计数，M1～M4 的原始计数依次乘以 −1、−1、−1、+1 换算为前进为正的轮角。原始累计计数保留，轮角按累计值换算到最近的 mrad。四个定时器均采用双路编码器模式，两个输入的数字滤波值均为 15。轮序和方向在首次架空单轮点动时核对，发现比例偏差再用独立观察的转数复核每圈计数。[固件换算参数](../firmware/control/baize_encoders.c) · [厂商计数参数](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Inc/VelocityControl.h) · [厂商反馈符号](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/VelocityControl.c) · [厂商编码器输入配置](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/tim4.c)

厂商调速周期为 5 ms，参考程序的转速限幅为 170 rpm。PI 增益取决于反馈单位、控制周期和输出定义，接入闭环后依据响应调整。命令超时、实际速度上限和停稳距离随验收版本记录。机械臂尚未装车，先形成空载记录，安装后按实际姿态和载荷复测。[厂商调速配置](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Inc/VelocityControl.h) · [环境准备与实车验收](bringup-and-acceptance.md)
