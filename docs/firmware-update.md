# 通过 RDK X5 更新底盘固件

## 日常操作

底盘上层 USB 座通过 A–A 数据线接 X5。将串口升级包完整复制到 X5，退出占用该串口的控制程序，在 X5 上执行：

```sh
baize-update info
sudo baize-update install ./baize-1.1.1.bin
```

默认端口为 `/dev/ttyUSB0`。两条命令均可加 `--port /dev/serial/by-id/…` 指定设备。安装命令打印 systemd 单元名。SSH 断开后，已启动的本机服务继续运行。重新登录可用 `journalctl -u <单元名>` 查看结果，用 `baize-update info` 核对版本、确认状态和镜像摘要。安装相同镜像会先检查停车，再报告已安装。回到指定旧版时，按同一条命令安装其串口包。

| 文件 | 用途 |
| --- | --- |
| `baize-<版本>.bin` | 含 MCUboot 头、型号和摘要的紧凑应用镜像，交给 X5 的 `install`。 |
| `baize-factory.bin` | 从 `0x08000000` 开始的 SWD 首装镜像，包含引导程序、已确认应用和擦除态的候选区。 |
| `chassis.bin`、`baize_base.elf` | 开发电脑的应用构建产物，分别用于打包和调试符号。 |

维护串口的访问权即安装权限。MCUboot 与 X5 分别检查 SHA-256 和受摘要覆盖的底盘型号 UUID。SHA-256 用于检测损坏。开发者可安装自己构建的同型号固件，也可以安装较早版本。发送控制指令和维护时都应独占串口，新版 Linux 主机库和升级工具通过同一设备上的 `flock` 仲裁。旧工具或第三方串口程序需先退出。

## 从停机到确认

1. X5 加锁，向应用发送 STOP，检查主动停机原因、连续的新轮反馈、零速和转角稳定。板端另查至少 20 个轮控周期的未使能、零轮速和四路中性 PWM，回送进入维护回执后软件复位。
2. MCUboot 将 PC6～PC9 置高中性，使用 USART2、115200 bit/s、8N1 提供 SMP 串口服务。普通启动有约一秒的维护窗口。收到维护报文后，有可启动应用时，连续约十秒无有效报文会复位。有可用旧版时，传输中途退出后可以从头重传。没有可启动应用时，引导程序保持停车等候恢复。
3. X5 通过 MCUboot 的 direct upload ID 2 写候选区，按候选镜像摘要申请 **test** 并复位。引导程序校验应用，按 2 KiB 页执行 offset swap。交换进度记在 Flash，复位后继续。试运行时板端忽略使能和速度目标。X5 复核版本、摘要、主动停机和新鲜轮反馈。升级前有 IMU 时还要收到新的六轴样本。
4. 检查通过后，应用只为与自身摘要匹配的确认请求写入运行区尾部的 `image_ok` 标志。正常执行但十秒未确认会软件复位，进入应用早期即卡死由引导程序已经启动的 IWDG 复位。下次启动由 MCUboot 自动恢复上一份已确认镜像。控制程序重新连接并使能、写入新目标后才能运动。已确认后的故障由原有轮控看门狗和停机语义处理。

v1.1.1 的升级器要求板端与主机同时观察到至少 300 ms 连续停车，轮样本间隔最多 60 ms。已装 IMU 要有两笔递增的近期采样，且与最后一笔轮反馈相距小于 250 ms。初次检查最多观察 1.5 s，给启动稍晚的可选 IMU 留出时间。中断上传后直接重试时，工具先让引导程序退出维护，重新读取旧版的能力。若有可启动的运行区却读不到该应用，工具在写入前报错。运行区确实为空时允许首次恢复，此时没有可回退的旧版；恢复后仍须验停车和轮反馈。

试运行依靠新应用遵守上述确认约定和保持看门狗有效。维护口有权装入自行构建的裸机程序，因此这些机制用于开发及运行故障恢复，不能约束主动改写 Flash 或持续错误喂狗的程序。[MCUboot 交换与回退](https://docs.mcuboot.com/design.html) · [串口恢复](https://docs.mcuboot.com/serial_recovery.html) · [smpclient](https://github.com/intercreate/smpclient)

### 地址与维护帧

| 区域 | 起始地址 | 大小 |
| --- | --- | ---: |
| MCUboot | `0x08000000` | 64 KiB |
| 运行区，镜像头 / 应用向量 | `0x08010000` / `0x08010200` | 94 KiB |
| 候选区，交换工作页 / 上传头 | `0x08027800` / `0x08028000` | 96 KiB |
| 预留页 | `0x0803F800` | 2 KiB |

部署按图纸保证的 256 KiB Flash、48 KiB SRAM。运行区末页专供 MCUboot 交换状态，串口包上限为 94,208 字节，包含 512 字节头、应用与 TLV。镜像型号为 `fe431dd9-ebe8-5fa2-93ea-0790b4a0472d`。采用 Zephyr 4.4.0 清单中的 MCUboot 2.4.0，复用镜像格式、串口 SMP 和掉电恢复的交换算法。项目补丁将串口写目标限定为候选区、限制试运行请求，并处理维护空闲退出。[板级分区](../firmware/boot/boards/baize/baize_chassis/baize_chassis.dts) · [串口策略](../firmware/boot/serial-policy.patch) · [依赖清单](https://github.com/zephyrproject-rtos/zephyr/blob/v4.4.0/west.yml)

应用运行时沿用 v1 的 COBS、CRC-16 与字节序。请求 `0x04` 的负载为一字节操作码加 32 字节镜像摘要；操作 1 查询、2 进入维护、3 确认。前两项的摘要填零。回执 `0x83` 依次为操作码、结果、试运行标记各一字节，接 MCUboot 版本（major、minor、LE16 revision、LE32 build）和 32 字节镜像摘要。结果 0 成功、1 尚未停稳或正在进入、2 镜像或操作不匹配、3 确认写入失败。`info` 显示的是 MCUboot 对头、应用及受保护元数据的摘要，文件清单中的 SHA-256 覆盖完整文件。上层日常读写仍使用原五类帧。[底盘接口](base-interface.md)

## 构建与安装工具

在 Windows 上构建 HAL 应用：

```powershell
cd firmware
cube cmake --preset Release
cube cmake --build --preset Release
```

引导程序和打包使用开发电脑上的 Linux / WSL。下列命令从仓库根目录执行。west 工作区放在仓库外供后续复用。此版本使用 Ubuntu 24.04 的 Arm GNU GCC 13.2.1。

```sh
repo=$PWD
work=$HOME/.cache/baize-boot
sudo apt install git python3-venv ninja-build device-tree-compiler gcc-arm-none-eabi libnewlib-arm-none-eabi
mkdir -p "$work"
python3 -m venv "$work/venv"
. "$work/venv/bin/activate"
pip install 'west==1.5.0' 'cmake==4.4.5'
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.4.0 "$work/src"
cd "$work/src"
west update mcuboot cmsis cmsis_6 hal_stm32 zcbor
pip install -r zephyr/scripts/requirements-base.txt ./bootloader/mcuboot/scripts
export GNUARMEMB_TOOLCHAIN_PATH=/usr ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
cd "$repo"
python firmware/boot/build.py "$work/src" "$work/build"
python scripts/package_firmware.py --version 1.1.1 --app firmware/build/Release/chassis.bin \
  --boot "$work/build/zephyr/zephyr.bin" --output build/update
```

日常只改 HAL 应用时，可省略 `--boot` 来单独制作串口包。为新构建指定相应的镜像版本。在 X5 上安装运行工具：

```sh
sudo python3 -m venv /opt/baize-update/venv
sudo /opt/baize-update/venv/bin/pip install ./tools/update
sudo ln -sfn /opt/baize-update/venv/bin/baize-update /usr/local/bin/baize-update
```

使用发布包时，安装命令可改为 `sudo /opt/baize-update/venv/bin/pip install -c x5-requirements.txt ./baize_update-1.1.1-py3-none-any.whl`。运行工具与开发电脑上的 imgtool 分用各自的 Python 环境。

本车的 X5 已安装带串口锁的 `baize-monitor`。用 `baize-monitor /dev/ttyUSB0 100` 可在维护前后采集轮状态。v1.1.1 交付包在 `linux-arm64/` 提供 X5 工具。

在仓库根目录可以从当前提交重建发布用 wheel。下面的源码归档、解包和输出都在构建目录内。

```sh
mkdir -p build/update
git archive --format=tar.gz --prefix=baize-update/ -o build/update/updater-src.tar.gz HEAD:tools/update
SOURCE_DATE_EPOCH=$(git log -1 --format=%ct -- tools/update) \
  python3 -m pip wheel --no-deps -w build/update build/update/updater-src.tar.gz
```

发布时在安装了升级依赖的 Linux Python 环境运行 `python tests/test_update_health.py`。许可证与署名从同一次构建的编译清单、头文件和工具链提取。运行 `python scripts/package_notices.py --help` 查看输入路径，输出目录随固件和主机工具一起分发。

### 复做故障试验

在开发电脑已激活 imgtool 的环境运行 `python tests/make_update_fixtures.py firmware/build/Release/chassis.bin build/update-test`，试验目录会得到正常的 `+1`、在复位入口停住的 `+2` 及填满 94,208 字节上限的 `+3`。这些文件仅用于台架验收。先用正式包建立已确认的基线，再在 X5 上用升级工具的 Python 运行以下脚本：

```sh
py=/opt/baize-update/venv/bin/python
$py tests/qualify_update.py interrupt baize-1.1.1+1.bin
$py tests/qualify_update.py no-confirm baize-1.1.1+1.bin
$py tests/qualify_update.py early-hang baize-1.1.1+2.bin
$py tests/qualify_update.py reject baize-1.1.1+1.bin
```

每项从一份摘要不同的已确认镜像开始，结束时核对同一基线、四轮停车和原有 IMU。串口断开观察所用的等待时间是复查时刻。上游擦写边界试验可在固定版本的 MCUboot `sim/` 下执行 `cargo test --no-default-features --features swap-offset,validate-primary-slot,bootstrap`，需检出该版本的 `ext/mbedtls` 子模块。[实车故障试验](../tests/qualify_update.py) · [上游模拟器](https://github.com/mcu-tools/mcuboot/tree/ee39e2d694bd827ffd1bebbce2f571a9154e6ec2/sim)

## ST-LINK 首装与原版恢复

在车架稳固、四轮架空、控制程序已停机时进行。X5 同时连接 ST-LINK 和底盘上层 CH340，车板独立供电。通过 OpenOCD 两次读取原 Flash，比较文件并另存一份到开发电脑。本车报告 512 KiB，首装镜像仅写前 254 KiB。以下 Bash 数组使用四线 SWD、软件复位与本机探针，工作目录中放待写文件：

```sh
swd=(openocd -f interface/stlink.cfg -c 'transport select hla_swd' -f target/stm32f1x.cfg
     -c 'gdb_port disabled; tcl_port disabled; telnet_port disabled; reset_config none; adapter speed 480')
"${swd[@]}" -c 'init; dump_image original-1.bin 0x08000000 0x80000; shutdown'
"${swd[@]}" -c 'init; dump_image original-2.bin 0x08000000 0x80000; shutdown'
cmp original-1.bin original-2.bin && sha256sum original-1.bin
"${swd[@]}" -c 'init; reset halt; flash write_image erase baize-factory.bin 0x08000000 bin; verify_image baize-factory.bin 0x08000000 bin; reset run; shutdown'
```

恢复首装前整片内容时，先核对本机独立备份的 SHA-256，使用同一条写入与校验命令，将两个 `baize-factory.bin` 均换为 `original-1.bin`。恢复到 v1.0.0 后用原监视工具读轮状态和 IMU。再次使用串口升级前须重新执行 SWD 首装。

## v1.1.0 实车记录

2026-11-24，RDK X5、板载 CH340 和 STM32F103 底盘，四轮架空。正式应用 24,248 字节，引导程序 30,668 字节。源码提交 `fc97a22` 的串口包、SWD 首装镜像与 X5 工具均逐字节重建一致。首装经 SWD 独立读回 260,096 字节，与发布的首装文件完全相同。[交付包、依赖版本和原始记录](measurements/baize-v1.1.0.zip)

整个 ZIP 的 SHA-256 为 `c5930971e1e7124e43eb48015ba88ca49c28d6b62ef9ecd984194487ef2f3c78`。解压后运行 `sha256sum -c SHA256SUMS` 可核对每个交付文件。

| 项目 | 实测结果 |
| --- | --- |
| 日常升级与 SSH 断开 | `1.1.0+0 → +1` 的 systemd 服务耗时 12.167 s。反向安装时，在服务启动后关闭实际 SSH 终端，重新登录看到服务成功，底盘回报 `+0` 已确认。两次均经 A–A 线传输、校验停机状态和 IMU。 |
| 传输中断与错误输入 | 24,824 字节包只发到 2,224 字节就关闭维护串口。静候后读到原确认版本和两路反馈。X5 在打开串口前拒绝错型号、损坏、截断和首装文件。通过底层 SMP 直接提交时，STM32 也拒绝前三种候选的试运行、向运行区上传和永久升级请求。上位机与升级工具双向占用均被拦截。 |
| 超时与看门狗回退 | 试运行期间关闭维护串口、不给确认，老版本自行恢复。另一次以最大镜像作为后备，候选轮反馈最后的板端时间为 9,981 ms，恢复镜像首帧为 21 ms，此次跨复位与整槽交换的反馈间隔为 5.736 s，全段轮状态均未使能且零速。复位入口即停住的专用候选也自动退回，试验前清除的 RCC 复位标志从 `0x00000003` 变为 `0x34000003`，其中 IWDGRSTF 已置位。 |
| 分区上限和停车 | 94,208 字节的完整包在 31.627 s 内试运行并通过确认。多一个字节的构建被拒绝。单独进入维护时非停机读取 GPIO，PC6～PC9 均为推挽输出，输入采样与输出锁存均为高。 |
| 兼容性和最终状态 | 首装前编译的原 C 示例直行、C++ 示例原地转向，各点动约 300 ms。两者均读到 IMU、主动停机并以未使能状态重连。故障试验后再次通过串口装回正式包。最终静置采集末 12 帧覆盖 220 ms，四轮未使能、零速且转角不变。去除连接初段的 USB 缓冲数据，按不同的板端采样时间计算，轮反馈 50.00 Hz、IMU 49.91 Hz。 |

Windows 9 项、Linux 开发机与 X5 各 14 项回归通过。固定版本的 MCUboot 模拟器启用 `swap-offset,validate-primary-slot,bootstrap`，25 项测试通过，覆盖交换、回退和擦写操作间注入复位。真实整车断电未实测。SWD 首装镜像 SHA-256 为 `d4d00c3fd879805f31898d405b3c55936968d04841f09dd51c69674bab54f98c`，串口包文件 SHA-256 为 `371553a3d4cc61f7c27ca1099cf83b5bbb949dddd9350b80f70722b825fc8e4d`。板上 `info` 报告的镜像摘要为 `a9df3e7ad41252a63da94b66eb9d7b74fbd6564fcc42cdbb8c442ff96225a9ea`。

首装前的独立全片备份保存在本机 `%LOCALAPPDATA%\Baize\Backups\20261124-before-serial-update\original-flash.bin` 和车载 X5 的 `/var/backups/baize/v1.0.0-before-mcuboot.bin`。两份均为 524,288 字节，SHA-256 为 `9f33c03b3c079ff9b43f89e45b8831d86e1571f1ec006c195e74fc96cdb84af4`。镜像前 22,480 字节与已发布的 v1.0.0 应用完全一致。
