# 底盘接口约定

**串口协议 v1 随空载 v1.0.0 发布（2026-11-22）。** USB 上位机是运动命令来源。停机、使能、四轮目标、底盘状态和可选的 MPU6050 六轴帧均已通过板载 CH340 验证。[现场互通与断线验收](bringup-and-acceptance.md#空载正式版实车验收)

固件维护另用 `0x04` 请求和 `0x83` 回执。控制程序在维护前关闭串口，升级结束后按原有启停顺序重连。版本查询、停稳条件、镜像确认和 SMP 传输见[固件升级说明](firmware-update.md#地址与维护帧)。

## 轮序、单位与 ROS 2 映射

以车辆前进方向判定左右。四轮使车辆前进的旋转方向统一为正，累计转角跨整圈连续。每条轮速目标命令同时更新四轮，每条底盘状态报告同一采样时刻的四轮测量值。

| 数组下标 | 物理车轮 | 约定的 ROS 2 关节名 | 差速控制器分组 |
| --- | --- | --- | --- |
| 0 | 左前 | `front_left_wheel_joint` | 左侧 |
| 1 | 右前 | `front_right_wheel_joint` | 右侧 |
| 2 | 左后 | `rear_left_wheel_joint` | 左侧 |
| 3 | 右后 | `rear_right_wheel_joint` | 右侧 |

ROS 2 轮关节使用 `rad` 和 `rad/s`。未来的 `SystemInterface` 为每个关节提供一个 `velocity` 命令接口和 `position`、`velocity` 两个状态接口。差速控制器用累计转角计算里程计。同一次连接中漏收单帧时，可以用下一帧的累计角度补上这段位移。建立新连接或识别到板端复位时，先建立新的角度基准，再累加后续差值。[ROS 2 差速控制器接口](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)

插件的 `write()` 将四个关节目标交给主机库，`read()` 将库提供的最新有效轮状态填入关节接口。主机库负责将线路上的 mrad 和 mrad/s 换算为 rad 和 rad/s。插件在控制器更新周期内读取已有反馈，轮反馈过期时上报通信异常。[主机库读写接口](#主机库读写接口)

本车轮式里程计的初始参数取厂商标称轮周长 0.496371 m（轮半径约 0.0790 m）和左右轮距 0.265 m。一次直行的估计距离等于四轮平均转角增量乘以标称轮半径。[几何参数来源](architecture.md#标定与验收依据)

六轴数据对应 ROS 2 的 `imu/data_raw`（`sensor_msgs/msg/Imu`），`frame_id` 为 `imu_link`。插件将主机库提供的两组三轴值分别填入 `linear_acceleration`（m/s²）和 `angular_velocity`（rad/s）。姿态由上位机融合，原始消息填 `orientation_covariance[0] = -1`。另两项协方差未知时填全零。用静态变换描述 `imu_link` 在 `base_link` 中的位置和朝向，车身坐标取 x 向前、y 向左、z 向上。时间戳用板端采样时间映射到主机时钟，重连或板端复位后重建映射，日志另存主机接收时间。[ROS IMU 坐标与话题约定](https://reps.openrobotics.org/rep-0145/) · [Jazzy Imu 消息定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Imu.msg)

本车安装方向经水平、抬车头和左右转复核。按车身 x 向前、y 向左、z 向上的约定，三个传感器轴分别同向对应三个车身轴。对于这一轴向近似，`imu_link` 到 `base_link` 的旋转四元数按 ROS 的 `x,y,z,w` 顺序为 `[0, 0, 0, 1]`。模块相对车身原点的平移和小安装角未测。此次水平静止加速度模长约 9.28 m/s²，陀螺有静止偏置。[安装与转向实测](bringup-and-acceptance.md#空载正式版实车验收)

## 主机库读写接口

应用从仓库根目录引入 CMake 目标 `baize_host`。C++11 包含 `<baize/chassis.hpp>`，C11 包含 `<baize/chassis.h>`。两者使用同一份串口读写、握手和单位换算实现。每个连接由一个 I/O 循环持有。[C++ 头文件](../include/baize/chassis.hpp) · [C 头文件](../include/baize/chassis.h)

```cmake
add_subdirectory(3rdparty/baize baize EXCLUDE_FROM_ALL)
target_link_libraries(controller PRIVATE baize_host)
```

打开端口并使能后，应用可在每个控制周期执行下列函数。调用者根据返回值处理中断，结束运动时调用 `disable()` 确认停机。

```cpp
bool update(baize::Chassis &chassis, double forward_m_s, double yaw_rate_rad_s)
{
    baize::ChassisState state{};
    if (!chassis.read(state) || !state.drive.valid) return false;
    const auto wheels = baize::to_wheel_speeds(baize::nominal_geometry(), forward_m_s, yaw_rate_rad_s);
    return chassis.write(wheels);
}
```

| 操作 | 语义 |
| --- | --- |
| `open(port)` | 清空旧输入，发送 STOP，最多等待约 1 s。通常以已确认的未使能状态打开。遇到板端锁存故障也可打开，读取 `state.drive.state` 和 `stop_reason` 诊断。C 函数返回句柄或空指针。 |
| `enable()` | 以新鲜的主动停机回执为前提。收到随后已使能的轮反馈才返回成功。 |
| `read(state)` | 处理调用时串口里已有的字节，取最近快照。检查 `state.drive.valid` 和 `state.imu.valid`。两路各有板端 `sample_ms`，连续 250 ms 没有新样本时相应标志失效，重复样本不能延长有效期。I/O 错误返回失败，两路均失效，需要关闭并重开端口。 |
| `write(wheels)` | 一次提交四个 `WheelSpeeds.rad_s`，按左前、右前、左后、右后排列。每项必须为有限数且在 ±2 rad/s 内，越界整组拒绝。成功表示目标帧已交给串口，跟踪结果从下一批轮反馈读取。 |
| `disable()`、`close()` | 停用发送 STOP 并等待新回执，已缓存或只收到半帧的旧回执不能确认本次请求。关闭端口时尽力发送 STOP 再释放资源。C++ 对象析构也会关闭。重连后须重新使能并给新目标。 |

轮端数据在 `state.drive` 中，累计角度 `angle_rad[4]`、角速度 `velocity_rad_s[4]`、电压 `supply_v`（ADC 尚无读数时为 NaN）、驱动状态和停机原因来自同一帧。`state.imu` 单独提供三轴加速度 `acceleration_m_s2` 和三轴角速度 `angular_velocity_rad_s`。两路分别接受向前推进的 32 位板端时间，支持自然回绕。重复或乱序样本不刷新主机单调时钟上的 250 ms 有效期。

持续运动建议每 20 ms 读一次、提交一次最新目标。写入零速仍保持使能。停止提交后，板端按 150 ms 命令时限撤销使能。轮反馈过期或底盘复位时，库请求停机，旧目标随即失效。IMU 缺样只影响六轴数据的有效性。

差速函数只做换算。设前进速度为 `v`（m/s，朝前为正）、转向角速度为 `ω`（rad/s，朝左为正）、轮半径为 `r`、左右轮距为 `b`，左右两侧的轮角速度分别为 `(v − ωb/2)/r` 和 `(v + ωb/2)/r`。使用本车标称几何，0.079 m/s 直行对应四轮各约 +1 rad/s。已有四个关节目标的上层直接写轮速。[本车几何参数](architecture.md#标定与验收依据) · [WPILib 差速运动学](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html)

两个示例工程都能复制到仓库外，用 `cmake -S <示例目录> -B <构建目录> -DBAIZE_SOURCE_DIR=<本仓库绝对路径>` 配置，再执行 `cmake --build <构建目录>`。四轮架空时运行 `chassis-example PORT` 或 `chassis-c-example PORT`，会以约 +0.5 rad/s 短时直行、停机和重连。加 `--turn` 进行同幅值左转。Windows 口名如 `COM17`，Linux 如 `/dev/ttyUSB0`。[C++ 示例](../examples/chassis/main.cpp) · [C 示例](../examples/c/main.c)

## 上位机 C 库

v1.0.0 应用可以沿用原来的构建入口、头文件和线路单位。这一入口通过 CMake 目标 `baize_host` 使用串口、v1 编解码和启停会话。假设应用已有 `controller` 目标，并将本仓库放在 `third_party/baize`，可在应用的 CMakeLists.txt 中加入下列两行。Windows 和 Linux 的接入方式相同。

```cmake
add_subdirectory(third_party/baize/host baize-host EXCLUDE_FROM_ALL)
target_link_libraries(controller PRIVATE baize_host)
```

在一个 I/O 循环中持有串口和会话。所有 `now_ms` 使用同一主机单调时钟，传入 `(uint32_t)monotonic_ms()` 即可。C 结构体保留线路单位，上层以 rad 或 rad/s 工作时在应用边界换算。

原版示例自行收发每个字节并将 mrad 换算为 rad。已用标签上的源文件和 CMake 工程原样编译并复验。[v1.0.0 原示例](https://github.com/1229282331/baize/tree/v1.0.0/examples/chassis)

| 操作 | 调用与判断 |
| --- | --- |
| 打开或重连 | `serial_open` 配置并清空串口，失败返回 `SERIAL_INVALID`。每次成功打开后调用 `baize_session_open(&session, 250)`，立即调用 `baize_session_next` 取出停机帧，用 `serial_write_frame` 发出。250 ms 是本工具的轮反馈过期时限。 |
| 接收反馈 | 将 `serial_read` 得到的每个字节依次交给 `baize_session_receive`。返回 true 时按类型读取 `frame.status` 或 `frame.imu`。两路时间独立，相同的板端毫秒可各有一帧。 |
| 使能和给速 | 应用明确请求使能时调用 `baize_session_enable`，返回 true 就立即发送它写出的使能帧。收到已使能反馈后，用 `baize_session_target` 提交四个 mrad/s 值（+1 rad/s 写 1000）。约每 20 ms 调用 `baize_session_next`，即使没有新目标也检查一次，返回 true 就发送该帧。一次提交最多发送一次，持续运动由控制程序持续提交新目标。 |
| 停机和退出 | `baize_session_stop` 丢弃待发目标，随即用 `baize_session_next` 取出并发送停机帧。继续处理收发，直至会话进入 `BAIZE_READY` 或等待超时。串口读写失败时向调用者报告并关闭串口，重开后重新握手。 |

轮状态的板端时间自然越过 `UINT32_MAX` 时继续采样。若轮时钟倒退，或同一采样毫秒报告了不同驱动状态或停机原因，会话丢弃待发目标并请求停机。识别到板端重启后，等待新时间轴上向前推进的主动停机回执，再允许使能。上层在重启或重连时重建角度和时钟基准。[会话接口](../src/baize_session.h) · [串口接口](../src/serial_port.h)

## 启停与异常行为

| 事件 | 底盘行为 |
| --- | --- |
| 上电或复位 | 默认未使能，四轮目标清零，周期发送状态。 |
| 轮控停止运行 | IWDG 复位主控。下一次状态报告使用已有的启动原因 1。主机丢弃旧目标，重新做停机、使能握手，收到已使能反馈后才提交新目标。 |
| 主机请求使能 | 无活动故障时进入已使能状态，从零目标开始计时。收到后续有效轮速命令才执行运动。重复使能不延长指令有效期。 |
| 收到轮速命令 | 仅在已使能时接受整组有效目标并刷新超时计时。任一轮目标超出该版本标定上限时整帧丢弃，上一有效命令仍受原有时限约束。 |
| 收到停机命令或运动命令超时 | 清零目标和控制器积累的输出，至迟在下一个轮速控制周期切入停止输出，撤销使能。上位机用停机原因、实测轮速和累计轮角核对停机及车轮停稳。 |
| 单轮持续给力且缺少编码器进展 | 四轮输出归中、清零目标和积分，锁存故障，状态为 2、原因为 4。检查机械传动和编码器接线后复位控制板，再做停机握手、重新使能并提交新目标。 |

每个 5 ms 轮控周期检查上一窗口实际施加的 PWM 和累计轮角。某轮沿非零目标方向的输出达到 250/2000，连续 350 ms 朝该方向的净进展不足 2 mrad，即触发全车停机。达到 2 mrad 后从新的位置重新计时。在同一位置附近往复抖动不能延长窗口。输出低于门槛、换向、零目标和新的启停过程分别重新计时。主机持续续发、STOP、重新打开串口和 ENABLE 均保持故障锁存。故障码描述板端见到的反馈异常，机械卡滞和编码器失效均可能产生该现象。

主机每次打开或重开端口都丢弃旧目标，发送停机。收到本次连接中驱动未使能、停机原因为主动停机的反馈后，再按调用者意图请求使能。上电、复位和板端超时反馈均不能代替主动停机回执。收到故障反馈则报告原因并保持停机。运行中若轮反馈过期，主机停止刷新目标，并在链路可用时发送停机。串口字节流中断由板端命令超时检出，只有已使能时收到的有效轮速目标能延长运动指令的有效期。

协议接受每轮 −2 至 +2 rad/s 的目标，主机点动约每 20 ms 刷新一次。板端每 5 ms 调速，指令有效期为 150 ms，轮状态周期为 20 ms。空载平地直行通过的轮速幅值为 0.35、0.50、1.00、1.50 rad/s，各档双向点动 1.6 秒。上层可按 1.50 rad/s 取此次工况的规划上限。2.00 rad/s 时个别轮的输出达到固件限幅，左右转实测的轮速幅值为 1.00 rad/s。[空载原始记录与判据](bringup-and-acceptance.md#空载正式版实车验收)

四轮架空、未装机械臂时，常规固件持续平稳转动的每轮建议下限为 **0.35 rad/s**。规划持续运动时宜避开 0.10 rad/s 这样的极低速。起停过渡和转弯内侧可以给更小的非零目标，固件按给定值调速。零目标清该轮积分并将输出归中。若把转弯内侧的小目标单独抬高，会改变左右轮速比和转弯曲率。对接 Nav2 时，`deadband_velocity` 按车身各轴速度配置，具体数值需结合底盘运动学和地面实测确定。[低速实测](bringup-and-acceptance.md#架空低速下限与微分对照) · [Nav2 速度平滑器](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/#deadband_velocity)

## 串口线协议 v1

固件和主机工具默认使用 **115200 bit/s、8 数据位、无校验、1 停止位（8N1）、无硬件流控**。选择这一常用档位是为了便于跨平台联调，现有帧长也留有带宽余量。[pySerial 波特率兼容性说明](https://pyserial.readthedocs.io/en/latest/pyserial_api.html#serial.Serial)

按 8N1 计算，每字节占 10 个线路位，全双工串口每个方向的容量为 11,520 字节/秒。若轮速目标、底盘状态和 IMU 都按 50 Hz 发送，主机每秒发 `22 × 50 = 1100` 字节（9.55%）。底盘每秒发 `(62 + 34) × 50 = 4800` 字节（41.67%），其中四轮状态占 26.91%。50 Hz 用于估算线路占用。目标刷新周期、状态周期、IMU 周期与命令超时时限经实测后随标定参数发布。板内轮速闭环独立运行。

每帧先顺序拼接一字节版本 `01`、一字节类型、该类型的定长载荷和两字节 CRC。下表的帧类型均用十六进制表示。CRC 覆盖版本至载荷末尾，采用 CRC-16/IBM-3740（CCITT-FALSE），多项式 `0x1021`，初值 `0xFFFF`，输入输出均不反射，最终异或 `0x0000`。CRC 与多字节数值均用小端序，有符号数用二进制补码。对整帧做 COBS 编码，最后追加一个 `00` 分隔符。[COBS 原始论文](https://www.stuartcheshire.org/papers/cobsforton.pdf)

| 类型 | 方向 | 载荷 |
| --- | --- | --- |
| `01` 停机 | 主机 → 底盘 | 空。停机请求在所有状态下有效，已有驱动故障仍保留。 |
| `02` 使能 | 主机 → 底盘 | 空。 |
| `03` 轮速目标 | 主机 → 底盘 | 四个 `int32`，按约定轮序排列，单位 mrad/s，共 16 字节。 |
| `81` 底盘状态 | 底盘 → 主机 | 同一次四轮采样的 56 字节载荷。 |
| `82` IMU 遥测（可选） | 底盘 → 主机 | 同一次六轴采样的 28 字节载荷。传感器就绪后自动上报，主机可按需读取。 |

| 状态载荷字节偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0～3 | `uint32` | 从本次启动开始的采样时间，单位 ms。连续运行时按 32 位无符号数处理回绕。 |
| 4 | `uint8` | 驱动状态。0 未使能，1 已使能，2 故障。 |
| 5 | `uint8` | 停机原因。0 正常使能，1 上电或复位，2 主动停机，3 指令超时，4 板端故障。 |
| 6～7 | `uint16` | 电源电压，单位 mV。尚无有效采样时为 `0xFFFF`。 |
| 8～39 | `int64[4]` | 本次启动以来的累计转角，单位 mrad，按累计编码器计数换算。停机和重新使能均保留累计值。 |
| 40～55 | `int32[4]` | 实测角速度，单位 mrad/s。 |

| IMU 载荷字节偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0～3 | `uint32` | `board_sample_ms`，六轴样本的板端采样时间，单位 ms。与 `81` 共用启动时钟，独立采样，按 32 位无符号数处理回绕。 |
| 4～15 | `int32[3]` | `acceleration_mm_s2`，传感器 x、y、z 三轴加速度（比力），单位 mm/s²。保留静止时的重力响应。 |
| 16～27 | `int32[3]` | `gyro_mrad_s`，同一传感器坐标系的 x、y、z 三轴角速度，单位 mrad/s，正向遵循右手定则。 |

`82` 在驱动未使能时也可发送。遇到 IMU 缺失、读错或样本未更新，板端跳过该帧，有新样本后恢复。主机分别用 `81` 和 `82` 判断两路反馈是否新鲜，缺少六轴样本时把 IMU 标为不可用或过期。四轮闭环及状态反馈持续运行，只有有效轮速目标能刷新运动命令时限。

接收方以 `00` 找帧边界。v1 最长帧含分隔符为 62 字节。遇到超长、COBS 解码失败、未知版本或类型、方向或长度不符、CRC 错误时，丢弃该帧，在下一帧边界恢复接收。仅实现四类核心帧的主机可按此规则跳过 `82`。坏帧对驱动状态和命令时限均无影响。

正式发布后，轮序、单位、字段布局和启停语义构成 v1 的兼容约定。改变这些约定时递增协议版本，并附上新版本的测试向量。控制器调参可沿用同一协议版本。

## 收发测试向量

以下是构造的线路字节，已含 COBS、CRC 和末尾分隔符。空格和换行仅供阅读。

**停机**

```text
05 01 01 1F 3E 00
```

**使能**

```text
05 01 02 7C 0E 00
```

**轮速目标**　左前、右前、左后、右后分别为 −1、+1、−1、+1 rad/s。

```text
09 01 03 18 FC FF FF E8 03 01 07 18 FC FF FF E8
03 01 03 CC 7C 00
```

**状态反馈**　采样时间 1000 ms，已使能，停机原因 0，电源 14.800 V。累计转角为 −1、+1、−1、+1 rad，实测角速度为 −0.9、+0.9、−0.9、+0.9 rad/s。

```text
05 01 81 E8 03 01 02 01 0D D0 39 18 FC FF FF FF
FF FF FF E8 03 01 01 01 01 01 0B 18 FC FF FF FF
FF FF FF E8 03 01 01 01 01 01 07 7C FC FF FF 84
03 01 07 7C FC FF FF 84 03 01 03 FA 19 00
```

**IMU 遥测**　构造样本时间 1020 ms。主机解码后的加速度为（+0.100、−0.200、+9.807）m/s²，角速度为（0、−0.250、+0.500）rad/s。此帧在线路上共 34 字节。

```text
05 01 82 FC 03 01 02 64 01 01 07 38 FF FF FF 4F
26 01 01 01 01 01 07 06 FF FF FF F4 01 01 03 3F
E6 00
```
