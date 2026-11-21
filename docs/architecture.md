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

TIM8 沿用厂商高有效 PWM1，`PSC=2`、`ARR=1999`，频率 12 kHz。四路在引脚切入定时器复用前先锁存 `CCR=2000`，板上已读到四脚持续为高、编码器静止。非零输出将 CCR 降低相应计数，正向 DIR 按左前、右前、左后、右后依次为高、低、高、低。换向前先保持一个控制周期的停止输出，停机时同时清零积分。架空点动时已测得输出归中后的编码器余动时间，落地停稳距离待测。[固件电机输出](../firmware/Core/Src/base_motion.c) · [架空实测](bringup-and-acceptance.md#架空四轮实测) · [厂商 TIM8 初始化](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/tim8.c) · [厂商方向逻辑](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/MotorControl.c)

### 电源采样与 USB

板上外部晶振为 8 MHz，设计采用 PLL ×9，HCLK 和 PCLK2 为 72 MHz，PCLK1 为 36 MHz。ADC1 选 PCLK2 ÷6，得到 12 MHz。旧工程的 ÷4 在同一时钟树下会得到 18 MHz，超过该芯片 14 MHz 的 ADC 上限。[厂商时钟初始化](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/main.c) · [STM32F103xC 数据手册](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)

电源经 R2（68 kΩ）和 R5（10 kΩ）分压到 PA4/ADC1_IN4。固件启动时校准 ADC，每次采 PA4 后自动采内部参考 VREFINT。输入电压估算为 `1200 mV × 7.8 × PA4 读数 / VREFINT 读数`，其中 1200 mV 为参考标称值。绝对精度待输入端万用表同步比对。[电压采样实现](../firmware/Core/Src/base_feedback.c) · [运动控制板原理图](四轮灵风小车资料/原理图.pdf)

USART2 经 PA2/TX、PA3/RX 接 CH340N。双层 USB 座的上层将 D+、D− 和电脑提供的 `USB_5V` 接至 CH340N，下层给外设输出 5 V。用适配本板的 USB-A 公对公数据线将电脑 USB-A 口接到上层，枚举出串口后以 115200 bit/s 联调。SWD 使用单独的调试引脚。[随车说明，第 2、5 页](四轮灵风小车资料/四轮小车资料简介.pdf) · [运动控制板原理图](四轮灵风小车资料/原理图.pdf)

### 已安装的 MPU6050

照片中央蓝色模块标为 GY-521。按板图，MPU6050 使用 U7，针 1～4 依次为 3.3 V、GND、SCL、SDA，针 8 为 INT。I2C1 重映射到 PB8/SCL、PB9/SDA，数据就绪信号接 PA5。相邻 U6 是针序不同的 GY-85 插座。首次上电前核对模块 VCC/GND 与 U7 针 1/2 的方向。[装车接线原图](assets/chassis-wiring.png) · [随车说明，第 6 页](四轮灵风小车资料/四轮小车资料简介.pdf#page=6) · [运动控制板原理图](四轮灵风小车资料/原理图.pdf)

上电后在 7 位地址 `0x68`、`0x69` 中确认 AD0 选中的地址，读寄存器 `0x75`（WHO_AM_I），两种地址的预期返回值都是 `0x68`。旧 HAL 代码中的 `0xD0` 是 `0x68` 左移一位后的参数。[厂商驱动地址定义](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Inc/mpu6050.h)

实车在 `0x68` 通过 WHO_AM_I 检查。量程为 ±2 g 和 ±250 °/s，启用 DLPF（档位 4），将 1 kHz 基频除以 20。PA5 上升沿记录板端时间，I²C 中断一次读出 `0x3A～0x48` 的就绪标志和 14 字节数据。跳过温度，按 16384 LSB/g 和 131 LSB/(°/s) 换算，保留重力与零偏。该固件将单次传输时限设为 5 ms，连续 100 ms 无就绪事件时重新探测，探测间隔 250 ms。[固件采样实现](../firmware/Core/Src/mpu6050.c) · [TDK InvenSense 寄存器手册](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf)

六轴数据保留传感器自身的右手坐标轴。板端只发送有效新样本。I²C 读取有时限，读取失败跳过该样本，串口发送拥塞时优先保留轮状态反馈。上位机记录传感器相对车身的安装变换，姿态解算可以与轮式里程计在主机上融合。[底盘接口约定](base-interface.md) · [ROS IMU 坐标约定](https://reps.openrobotics.org/rep-0145/)

## 固件组织

- 板级代码集中管理时钟、引脚、定时器、ADC 和串口。CubeMX 生成初始化代码。手写的控制和编解码逻辑使用 C，将输入、输出与状态写清楚，使同一份源码能在电脑和目标板上测试。
- 裸机按固定周期采样四轮并更新控制输出。四个轮子共用带前馈的 PI 调速函数，各自保留参数和积分状态。
- 编码器连续累计正反向位移，并处理硬件计数器回绕。四个轮子的角度和速度使用同一个采样时刻。串口收发和遥测不阻塞轮速更新。
- 板端用自身时钟判断运动指令是否过期。停机时清除旧目标和调速器积累的输出。通信工具和未来的 ROS 插件复用一份协议编解码实现。

松灵 Scout 将底盘通信与 ROS 封装分开，支持独立联调。Linorobot2 则在单片机里接收 ROS 车速指令并计算里程计。本项目选择独立 USB 接口，把 ROS 依赖和整车几何留在上位机。[Scout SDK](https://github.com/agilexrobotics/ugv_sdk) · [Scout ROS 2 驱动](https://github.com/agilexrobotics/scout_ros2) · [Linorobot2 固件](https://github.com/linorobot/linorobot2_hardware)

## 开发环境与版本

主开发环境选 Windows 原生。用经典 STM32CubeMX 配置 F1 系列外设，使用 STM32CubeF1 HAL、Arm GCC 和 CMake/Ninja 构建。VS Code 与终端共用 CMake 预设，烧录和断点调试走 SWD。将 `.ioc`、生成的初始化代码、CMake 预设和工具版本锁定文件纳入版本管理，普通构建直接使用仓库中的源文件。[ST 配置工具](https://www.st.com/content/st_com/en/stm32cubemx.html) · [STM32CubeF1](https://github.com/STMicroelectronics/STM32CubeF1) · [ST 可复现构建示例](https://dev.st.com/stm32cube-docs/stm32cubecli/latest/en/docs/markup/STM32CubeCLI_Getting_Started/STM32CubeCLI_First_Project.html)

Keil µVision 支持命令行构建，旧工程可用于查阅原厂实现。新固件只维护一套 CMake 构建配置，板上交叉编译和电脑端逻辑测试共用项目中的手写源码。[µVision 命令行](https://www.keil.com/support/man/docs/uv4cl/uv4cl_commandline.htm)

## 标定与验收依据

随车图纸标注轮径约 152.4 mm，旧代码中轮周长为 0.496371 m，折合轮径约 158.0 mm，两者相差约 3.7%。整车里程计使用实际行驶测得的有效滚动周长。四轮滑移转向的有效轮距通过实际转角标定，并记录地面与载荷条件。[底盘尺寸，资料第 1 页](四轮灵风小车资料/四轮小车资料简介.pdf) · [原厂运动学参数](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Inc/Kinematics_4WD.h)

编码器沿用厂商默认值，输出轴每圈 144000 计数，M1～M4 的原始计数依次乘以 −1、−1、−1、+1 换算为前进为正的轮角。原始累计计数保留，轮角按累计值换算到最近的 mrad。四个定时器均采用双路编码器模式，两个输入的数字滤波值均为 15。架空单轮实测已核对四组电机与编码器的配对及反馈正负。还需从车身上独立核对实际轮位和朝前的滚动方向，出现比例偏差时再独立复核每圈计数。[固件换算参数](../firmware/control/baize_encoders.c) · [厂商计数参数](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Inc/VelocityControl.h) · [厂商反馈符号](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/VelocityControl.c) · [厂商编码器输入配置](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/tim4.c)

当前固件每 5 ms 更新一次**轮速 PI 加前馈**。目标轮速 `r` 非零时，先用 `Ks × sign(r) + Kv × r` 估计该方向和速度所需的 PWM，再用编码器误差 `e = r − 实测轮速` 修正。比例项为 `Kp × e`，积分按实际秒数累加 `Ki × e × Δt`。三项相加后限幅并取整。总输出饱和时暂停会加深饱和的积分更新。零目标或停机会清积分并将输出归中。微分增益为零。轮速单位为 mrad/s，输出单位为 PWM 计数。[板端控制函数](../firmware/control/baize_drive.c) · [四路参数](../firmware/Core/Src/base_motion.c) · [WPILib 电机前馈原理](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html)

架空标定值为四轮共用 `Kp=0.28`、`Ki=4.0`、`Kv=0.18`。按左前、右前、左后、右后顺序，`Ks` 分别为 125、125、125、90 计数。右后通道维持相同低速所需的输出较小。板端接受每轮 ±2000 mrad/s，命令时限 150 ms，PWM 上限为 ±700/2000（35%）。原版 PI 的起始增益取自厂商常速档换算，`Kp=0.067265`、`Ki=3.2392`。两版均已用同一辆车测量，带载参数需在相应工况复核。[架空调参与复验](bringup-and-acceptance.md#四轮架空调参复验) · [厂商调速实现](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/VelocityControl.c)

### 速度环为什么选 PI

起转取决于轮端转矩能否克服当时的静摩擦和负载。轮子一旦转动，维持同一低速所需的输出可能更小。现有前馈先给出基础 PWM，P 立即响应轮速误差，I 在误差持续时补足输出。轮子卡住且目标恒定时，测速微分趋于零。D 的用途在于速度变化时提供阻尼。随车范例使用分档增量式 PID。ODrive 的速度环采用 PI 加前馈，maxon 的单环速度整定也从 P、I 和测速滤波入手。[厂商范例](四轮灵风小车资料/DoubleQ_FourRoundsCar_AllCode/Src/VelocityControl.c) · [ODrive 速度环](https://docs.odriverobotics.com/v/latest/manual/control.html#velocity-control-loop) · [maxon 速度环整定](https://support.maxongroup.com/hc/en-us/articles/4411824545682-EPOS4-IDX-Manual-control-parameter-tuning-hints)

按本车 144000 计数/圈、5 ms 测速窗，相邻两窗的增量相差 1 计数，速度就相差约 0.00873 rad/s，直接差分会得到约 1.75 rad/s²。D 会同时响应真实加速度、量化跳变和机械速度波动。实车对照采用测速微分和一阶低通，避免目标阶跃本身产生微分冲击。[微分与高频噪声](https://www.mathworks.com/videos/understanding-pid-control-part-3-expanding-beyond-a-simple-derivative-1531120808026.html) · [本车低速与 D 项对照](bringup-and-acceptance.md#架空低速下限与微分对照)

若任务需要持续爬行，一种已在本车验证的办法是将测速和 PI 调节周期缩短至 1 ms，并按该周期重新调增益和摩擦前馈。轮子减速时可更早加力，重新加速时也能更早收回 PWM。架空试验在 0.10 rad/s 实现连续跟踪，在 0.20 rad/s 实现平稳跟踪。按同样的 5 ms 间隔记录，试验组的 PWM 变化幅度更大，当前作为地面带载对照的备选。[参数、代价与实测曲线](bringup-and-acceptance.md#低速-pi-的-1-ms-实车对照)

| 备选方法 | 适用条件与本车取舍 |
| --- | --- |
| 正反向摩擦辨识与电压前馈 | 如果落地后起转仍是瓶颈，可缓慢递增驱动电压测起转，再从转动状态降压测维持转动所需的输出。分别记录正反向、载荷和电池电压，用独立数据检查分方向或按电压补偿是否能降低平滑轮速下限。现有 `Ks` 是闭环标定的 PWM 参数。[WPILib 电机电压模型](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html) · [双向辨识试验](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html) |
| 外环控速度、内环控电流 | 电流内环能在有电流测量的驱动上直接调节产生转矩的电流，适合频繁负载变化。当前 STM32 采集了母线电压，现有接线和固件尚无逐轮电流反馈。[ODrive 级联控制](https://docs.odriverobotics.com/v/latest/manual/control.html) |
| 状态观测器与 LQR | 适合已有可信电机模型并需要估计速度的场景。WPILib 的单状态飞轮例子中，LQR 反馈项等价于比例增益，滤波来自模型与观测器。低速黏滑还需要单独辨识摩擦。现阶段优先实测载荷变化。[WPILib 状态空间实例](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-flywheel-walkthrough.html) |
