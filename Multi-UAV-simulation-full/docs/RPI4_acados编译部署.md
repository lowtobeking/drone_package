# RPi 4B acados 编译部署（真机机载电脑）

前提：RPi 4B（建议 4GB+ 内存）、Ubuntu 22.04 Server **arm64**、已联网、风扇散热已装。
全程 SSH 操作，约 30–50 分钟（含编译）。

## 0. 系统准备

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git python3-pip libblas-dev liblapack-dev

# CPU 锁性能模式（避免编译降频 + 控制循环 50Hz 抖动）
sudo apt install -y cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

2GB 内存版必须先加 swap（4GB/8GB 跳过）：

```bash
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## 1. 克隆 acados

```bash
cd ~
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
```

> 版本一致性：建议与 x86 仿真机同版本。在仿真机 `cd ~/acados && git log -1 --format=%H`
> 拿到哈希，RPi 上 `git checkout <hash> && git submodule update --recursive --init`。

## 2. CMake 配置（关键：BLASFEO target）

RPi 4 是 Cortex-A72，BLASFEO 没有 A72 专属 target，**选 `ARMV8A_ARM_CORTEX_A57`**（同代架构，社区验证最优）：

```bash
mkdir -p build && cd build
cmake .. \
  -DACADOS_WITH_QPOASES=ON \
  -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57 \
  -DHPIPM_TARGET=GENERIC \
  -DACADOS_INSTALL_DIR=$HOME/acados
```

若后续编译/运行有非法指令(SIGILL)等异常，退回保守 target 重配：
`-DBLASFEO_TARGET=GENERIC`（慢 2–3 倍但必稳，solve 预算内仍够 trio3）。

## 3. 编译安装（RPi4 约 10–20 分钟）

```bash
make install -j4        # 2GB 内存用 -j2，防 OOM
```

## 4. Python 接口

```bash
pip3 install -e ~/acados/interfaces/acados_template
pip3 install numpy casadi pyyaml matplotlib    # casadi 有 aarch64 wheel，直接装
```

## 5. t_renderer（⚠️ ARM64 最大的坑）

acados 生成 C 代码依赖 `t_renderer` 二进制。x86 上会自动下载，但下载的是
**x86_64 版**，在 RPi 上报 `Exec format error`。必须手动放 arm64 版：

```bash
# 先去 https://github.com/acados/tera_renderer/releases 确认最新版号
wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux-arm64 \
     -O ~/acados/bin/t_renderer
chmod +x ~/acados/bin/t_renderer
~/acados/bin/t_renderer --help   # 能打印用法 = 架构对了
```

若该版本没有 arm64 资产，用 Rust 源码编译（~5 分钟）：

```bash
sudo apt install -y cargo
git clone https://github.com/acados/tera_renderer ~/tera_renderer
cd ~/tera_renderer && cargo build --release
cp target/release/t_renderer ~/acados/bin/
```

## 6. 环境变量

```bash
cat >> ~/.bashrc <<'EOF'
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib
EOF
source ~/.bashrc
```

## 7. 自检（acados 官方例子）

```bash
cd ~/acados/examples/acados_python/getting_started
python3 minimal_example_ocp.py
```

首跑会现编 OCP C 代码（RPi 上比 x86 慢，耐心等）。正常 = 打印求解状态无报错。

## 8. 项目 benchmark（部署门槛）

```bash
cd ~ && git clone https://github.com/Aaron6099/Multi-UAV-simulation.git
cd Multi-UAV-simulation
python3 report/verify_mpc_step.py      # 单机阶跃：看 solve mean/max
python3 report/verify_formation.py     # pair2/trio3 编队：更接近实机负载
```

**判定：solve max < 15 ms 才允许上机**（50Hz 周期 20ms，留 25% 余量）。
参考：x86 SITL 实测 0.13–0.32ms；RPi 4 预估 trio3 约 2–5ms，应能过。

## 9. 常见坑速查

| 症状 | 原因 | 处理 |
|---|---|---|
| `Exec format error` | t_renderer 是 x86 版 | 步骤 5 换 arm64 版 |
| 编译中途被 killed | OOM | `-j4`→`-j2` 或加 swap |
| `import acados_template` 失败 | pip 装错解释器 / 环境变量缺 | 确认 `pip3` 与 `python3` 同源；重 source bashrc |
| 运行时找不到 `.so` | LD_LIBRARY_PATH 未生效 | 重登 SSH 再试 |
| 非法指令 SIGILL | BLASFEO target 不兼容 | 步骤 2 退 GENERIC 重编 |
| solve 偶发尖刺 | CPU 降频 | 锁 performance + 查温度（见下） |

温度监控（Ubuntu 上没有 vcgencmd，用 sysfs）：

```bash
watch -n2 'echo $(($(cat /sys/class/thermal/thermal_zone0/temp)/1000))°C'
# 持续 >80°C 会降频 → 检查风扇
```

## 10. 下一步（acados 通过后）

1. **ROS2 Humble**：按官方 apt 源装 `ros-humble-ros-base`（Server 版不需要 desktop）
2. **px4_msgs + 本包**：
   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone https://github.com/PX4/px4_msgs.git -b release/1.14
   git clone https://github.com/Aaron6099/Multi-UAV-simulation.git mpc_control
   cd ~/ros2_ws && colcon build --symlink-install
   ```
3. **MicroXRCEAgent**（real_hardware_launch.py 依赖，可执行名必须是 `MicroXRCEAgent`）：
   ```bash
   git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/xrce_agent
   cd ~/xrce_agent && mkdir build && cd build
   cmake .. && make -j4 && sudo make install && sudo ldconfig
   ```
4. **DDS 中间件 + 域号 + 时钟**：

   > ### 🚨 勘误（2026-07-21 实测）：**不要用 CycloneDDS**
   >
   > 本节原写 `sudo apt install ros-humble-rmw-cyclonedds-cpp` +
   > `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`——**这是错的，照做会直接搞断飞控链路**。
   >
   > **原因**：`MicroXRCEAgent` 是**基于 Fast-DDS 编译**的，ROS2 不同 RMW 实现之间
   > 官方不保证互通。实测（Orin NX + 6C，链路其余部分均正常）：
   >
   > | RMW | domain | `/fmu/` 话题数 |
   > |---|---|---|
   > | `rmw_fastrtps_cpp` | 42 | **22** ✅ |
   > | `rmw_cyclonedds_cpp` | 42 | **0** ❌ |
   >
   > **且仿真机从未装过 CycloneDDS**（`dpkg -l \| grep -c cyclonedds` = 0，
   > 默认 RMW 就是 `rmw_fastrtps_cpp`）——41 场景回归、论文全部实验都跑在 FastRTPS 上。
   > 即 **FastRTPS 才是本项目被验证过的配置**，CycloneDDS 属于未经验证的误写。

   ```bash
   sudo apt install -y chrony
   echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc   # 与仿真机/Agent 一致
   echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc                      # 全员一致
   ```

   ### ⚠️ 域号必须【两侧都设】——只设 companion 侧无效

   `ROS_DOMAIN_ID` 只管 companion 上的 ROS2 节点。**Agent 的 DDS participant 用哪个域，
   是由飞控参数 `UXRCE_DDS_DOM_ID` 决定的**（PX4 源码 `uxrce_dds_client.cpp:229`
   `uint16_t domain_id = _param_xrce_dds_dom_id.get();` → 传入
   `uxr_buffer_create_participant_*()`），Agent 自身的环境变量不起作用。

   **该参数默认为 0**。若只在 companion 设 `ROS_DOMAIN_ID=42` 而不改飞控，
   Agent 会在 domain 0 发布、ROS2 节点在 domain 42 收听 →
   **Agent 日志显示会话正常、话题全部创建、零报错，但 ROS2 里一个话题都看不到**。

   **每架飞控都要设**（QGC 参数页，或用 MAVLink 写入）：
   ```
   UXRCE_DDS_DOM_ID = 42     # 与全员 ROS_DOMAIN_ID 一致；reboot_required
   ```
   实测验证（2026-07-21）：设为 42 并重启飞控后，domain 42 可见 22 个话题、
   domain 0 可见 0 个——**域隔离生效**。这有实际安全价值：仿真机在同一局域网
   跑 SITL（默认 domain 0）不会与真机话题串台。

   ### 验证方法（⚠️ 必须加 `--no-daemon`）
   ```bash
   bash -ic 'printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID'   # 注意 -ic 而非 -lc，原因见上条
   ros2 topic list --no-daemon | grep fmu                 # 必须 --no-daemon
   ros2 topic hz /fmu/out/vehicle_status
   ```
   > **`ros2 daemon` 缓存会造成大量假阴性**（2026-07-21 踩坑，浪费了数轮排查）：
   > daemon 会缓存拓扑发现结果。Agent 启动【之前】若跑过 `ros2 topic list`，
   > daemon 缓存的"空拓扑"会一直返回 0 个话题，即使链路完全正常。
   > `ros2 daemon stop` 后立刻查询同样不可靠——daemon 重启后需要时间重新发现。
   > **验证链路时一律加 `--no-daemon`**，或用 `ros2 topic hz`（自建节点，不走 daemon）。

   > ### ⚠️ 非交互 shell 不读 `.bashrc` —— 真机上会导致编队**静默失效**
   >
   > `.bashrc` **只对交互式 shell 生效**。实测（2026-07-21，Orin NX）：
   > `bash -lc 'printenv ROS_DOMAIN_ID'` 读不到，`bash -ic` 才读得到。
   >
   > **后果**：若用 systemd unit / `nohup` / 非交互 SSH 脚本拉起
   > `real_hardware_launch.py`，`ROS_DOMAIN_ID` 和 `RMW_IMPLEMENTATION` 都不会生效，
   > 该机静默退回 `ROS_DOMAIN_ID=0` + 默认 FastRTPS ——
   > **节点起得来、日志无任何报错，但它跟编队里其他机根本发现不了对方**。
   > 这类故障在外场极难定位（看起来"一切正常"，只是队形不成型）。
   >
   > **对策**：凡是非交互方式启动，必须在脚本/unit 里**显式 export**，别依赖 `.bashrc`：
   > ```bash
   > export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   > export ROS_DOMAIN_ID=42
   > export ACADOS_SOURCE_DIR=$HOME/acados
   > export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib
   > source /opt/ros/humble/setup.bash
   > source ~/ros2_ws/install/setup.bash
   > ```
   > systemd 用 `Environment=` 或 `EnvironmentFile=`。
   > 同类坑仿真机上已踩过（`mpc_node` 因 `LD_LIBRARY_PATH` 缺失找不到 `libhpipm.so`
   > 而静默崩溃），见 `report/run_one_trial.sh` 注释。**起飞前务必在目标启动方式下
   > 实跑一次并核对这几个变量。**
5. **串口方案 —— ⚠️ Orin NX 与 RPi4 在这里分道扬镳**

   ### 🚨 CH340 在 Jetson/L4T 上【不能用】（2026-07-21 实测）

   L4T 36.4.3（JetPack 6.2）内核**不含 `ch341.ko` 驱动**。现象具有迷惑性：
   ```bash
   lsusb          # 能看到 1a86:7523 QinHeng CH340 —— USB 层枚举成功
   ls /dev/ttyUSB*   # 但没有任何设备节点生成
   modinfo ch341     # Module ch341 not found  ← 根因：模块文件根本不存在
   ```
   `/lib/modules/$(uname -r)/kernel/drivers/usb/serial/` 里只有
   `cp210x.ko`、`ftdi_sio.ko`、`option.ko`、`usbserial.ko`、`usb_wwan.ko`。
   **不是没加载，是内核就没编这个驱动**，`modprobe` 无解。
   （RPi4 跑标准 Ubuntu 内核自带 ch341，所以"复用 CH340 最省事"这个判断
   **只对 RPi4 成立**，迁到 Orin NX 后失效。）

   #### ❓"刷成 Ubuntu 22.04 能解决吗？" —— 不能，问题在内核不在发行版

   **板子本来就已经是 Ubuntu 22.04。** 同版本发行版实测对比（2026-07-21）：

   | | 仿真机 (x86) | Orin NX |
   |---|---|---|
   | 发行版 | Ubuntu 22.04.5 LTS | Ubuntu 22.04.5 LTS ← **相同** |
   | 内核 | `6.8.0-134-generic` | `5.15.148-tegra` |
   | 内核来源 | Canonical `linux-image-*-generic` | NVIDIA `nvidia-l4t-kernel` |
   | `ch341.ko` | ✅ 有 | ❌ 无 |
   | usb/serial 驱动数 | **53 个** | **5 个** |

   "Ubuntu 22.04" 描述的是**用户态**（apt/glibc/命令行工具），**内核是独立一层**。
   Canonical 通用内核"能编的驱动全编上"；NVIDIA 按 Tegra 需求裁剪，不含 CH340。

   **Jetson 无法换用 Canonical 通用内核**——Tegra234 的启动代码、设备树、GPU/显示/
   摄像头/USB 控制器/电源管理驱动都不在主线内核里。即 **"Ubuntu 22.04 + generic 内核"
   这个组合在 Jetson 上不存在**，刷机只会再得到一次 "Ubuntu 22.04 + L4T tegra 内核"，
   模块集完全相同。

   已核实**所有**内核相关包均已安装且都不含 ch341：`nvidia-l4t-kernel`、`-dtbs`、
   `-headers`、`-oot-headers`、`-oot-modules`（含 NVIDIA 树外驱动包）。

   > ⚠️ **刷机的代价**：会抹掉整机，本文档 Step 0–7（acados 全套）、MicroXRCEAgent、
   > colcon 工作区、dialout/DDS 环境变量等全部要重做，**换来零收益**。不要为这个问题刷机。

   若确实必须用 CH340，唯一出路是动内核（取 NVIDIA 内核源码开
   `CONFIG_USB_SERIAL_CH341=m` 重编，或单独把 `ch341.c` 编成树外模块）——
   代价是每次内核升级重做 × 9 台机器 × 外场多一个自制组件，**不推荐**。

   ### ✅ Orin NX 正解：走片上原生 UART `/dev/ttyTHS1`

   官方载板 P3768（`cat /proc/device-tree/model` 确认）的 **J12 是 40 针 2×20
   双排排针**，全部信号 3.3V，与 Pixhawk TELEM2 电平天然匹配，无需电平转换。

   **接线（3 线，VCC 不接）**：

   | Jetson J12 40针 | 方向 | Pixhawk 6C TELEM2 |
   |---|---|---|
   | Pin 8 (UART1_TX) | → | Pin 3 (RX) |
   | Pin 10 (UART1_RX) | ← | Pin 2 (TX) |
   | Pin 6 (GND) | — | Pin 6 (GND) |
   | — | ✗ | Pin 1 (VCC) **悬空** |

   > 偶数排从 Pin 2 端数：`2→4→6→8→10`，Pin8/Pin10 是第 4、5 个且**相邻**。
   > ⚠️ Pin 2/4 是 5V、Pin 6 是 GND，数错桥到 5V-GND 之间即短路，建议断电接线。

   **`agent_dev:=/dev/ttyTHS1`**（不再是 `/dev/ttyFC`）

   ### 引脚映射验证：回环测试（接飞控【前】做，排掉一个未知数）

   用一根杜邦线短接 **Pin 8 ↔ Pin 10**（此时飞控勿接，否则两个发送端打架），
   然后跑 `loopback_test.sh`（写入标记看能否原样读回，同时测 ttyTHS1/ttyTHS2）：
   ```bash
   stty -F /dev/ttyTHS1 921600 raw -echo
   timeout 3 cat /dev/ttyTHS1 > /tmp/lb.txt &
   sleep 0.5; printf 'LOOPBACK_OK\n' > /dev/ttyTHS1; wait
   grep LOOPBACK_OK /tmp/lb.txt && echo "Pin8/10 => ttyTHS1 确认"
   ```
   **2026-07-21 实测结果**：`ttyTHS1` ✅ 收到标记，`ttyTHS2` ❌ 0 字节 ——
   40 针 Pin8/Pin10 确定映射到 **ttyTHS1**（与 NVIDIA 官方 UART1 文档一致）。

   ### 串口权限（两种方案都要）
   ```bash
   sudo usermod -a -G dialout $USER   # ttyTHS*/ttyUSB* 均为 root:dialout 660
   # 需重新登录才生效；新开 SSH 会话自动带上
   ```
   验证：`id -nG | grep dialout`、`ls -la /dev/ttyTHS1`

   ### 若坚持用 USB 转串口
   换 **CP2102 或 FTDI** 芯片的模块（`cp210x.ko`/`ftdi_sio.ko` L4T 内核已自带），
   电平选 3.3V，udev 规则里的 `idVendor` 要改成对应厂商 ID
   （CP2102=`10c4`，FTDI=`0403`；CH340 的 `1a86` 在 Jetson 上写了也没用）。
   ~~CH340 udev 规则（仅 RPi4 适用）~~：
   ```bash
   # 仅 RPi4：echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="ttyFC"' \
   #   | sudo tee /etc/udev/rules.d/99-ch340.rules
   ```
6. **FC 侧释放 TELEM2 给 uXRCE-DDS**（否则 Agent 永远连不上）：

   > **⚠️ 勘误（2026-07-21，查 PX4 源码核实）**：本节早先写的
   > "TELEM2 出厂默认被第二个 MAVLink 实例占用（`MAV_1_CONFIG` 默认即 TELEM2）"
   > **是错的**（当时据博客排障表推断，未核实）。查 PX4 **v1.14.3** 源码
   > `src/modules/mavlink/module.yaml`：
   > ```yaml
   > port_config_param:
   >   name: MAV_${i}_CONFIG
   >   # 0: Telem1 Port (Telemetry Link)
   >   # 1: Telem2 Port (Companion Link). Disabled by default to reduce RAM usage
   >   default: [TEL1, "", ""]
   > ```
   > 即 `MAV_0_CONFIG`=TEL1、**`MAV_1_CONFIG` 默认为空 = Disabled**。
   > 且 `boards/px4/fmu-v6c/` 板级配置**未覆盖**该默认（全仓库仅 ModalAI fc-v1/fc-v2、
   > sky-drones、NXP mr-canhubk3 在板级默认里设了 `MAV_1_CONFIG`）。
   >
   > **实际影响**：全新 Pixhawk 6C 上 `MAV_1_CONFIG=0` 是**空操作**，TELEM2 出厂
   > 无人占用。因此**不能用"TELEM2 上读不到数据"来判断接线好坏**——出厂状态下
   > 本来就没有任何东西在发，0 字节是预期行为。
   >
   > **但这一步仍应保留为防御性检查**：①飞控可能被前手/自己配置过；
   > ②部分厂商板级默认确实把 TELEM2 给了 MAVLink；③改前先记录原值便于回溯。

   若 `MAV_1_CONFIG` 当前**不是** Disabled，它会与 `UXRCE_DDS_CFG=TELEM2`
   抢同一物理串口 → `uxrce_dds_client` 起不来。QGC 参数页设：
   ```
   MAV_1_CONFIG  = 0 (Disabled)   # 先把 TELEM2 从 MAVLink 手里放出来
   UXRCE_DDS_CFG = TELEM2
   SER_TEL2_BAUD = 921600
   ```
   不重启 FC 的临时释放（NSH 里）：`mavlink stop -d <TELEM2设备>`

   > **台架首跑用 Pixhawk 6C**（2026-07-21 定）：`/dev/ttyS3` = TELEM2，
   > 与 duduuu 博客同款板子已验证，固件目标 `px4_fmu-v6c`。
   > 选它是为**隔离变量**——本轮同时引入 Orin NX、CH340 串口、新编的 Agent 三个新环节，
   > 不宜再叠一个设备路径未知的飞控。
   >
   > ⚠️ **换到 CUAV Nora+（或任何其他板）时必须重验**：TELEM2 内部设备路径
   > **不一定**是 `/dev/ttyS3`，用 `uxrce_dds_client status` 核对实际路径，固件目标也不同。
   > 台架在 6C 上跑通只证明了 companion 侧这条链，**不等于 Nora+ 上也通**；
   > 真正上天那块板必须把本节配置重做并验证一遍。

   **症状对照**（Agent 报 not connected / `ros2 topic list` 无 `/fmu/...` 时）：
   | 症状 | 原因 | 处理 |
   |---|---|---|
   | Client 起不来 | MAVLink 仍占 TELEM2 | `MAV_1_CONFIG=0` + `mavlink stop -d <dev>` |
   | Agent not connected | `UXRCE_DDS_CFG` 没设 | 设成 TELEM2 后重启 FC |
   | 串口无任何数据 | 接线/波特率错 | TX↔RX 交叉、两侧同为 921600 |
7. **接线前先验物理链路**（把故障域切开：先证明"线+波特率"没问题，再往上查 DDS/ROS）：
   ```bash
   stty -F /dev/ttyFC 921600 raw -echo
   cat /dev/ttyFC        # 出现乱码 = 线通了、波特率对了；毫无输出 = 还没通
   ```
   ⚠️ 接线注意：**不要接 VCC**——飞控与机载电脑各自独立供电，只接 TX/RX/GND，
   且 TX↔RX 必须交叉。另：**飞行中反复热插拔串口可能导致后续连接失败**，
   外场不要靠插拔串口来"复位"。
8. 台架首跑：`ros2 launch mpc_control real_hardware_launch.py drone_id:=0 scenario:=S2_pair2_hover`
   （首次现编 mpc_node 的 OCP，数分钟；**务必台架预编译，别留到外场**）

   验证链路已通：`ros2 topic list | grep fmu`

---
> 第 5–7 条的 `dialout` 组、`MAV_1_CONFIG=0`、串口验证阶梯补充自
> [duduuu.xyz uXRCE-DDS 真机部署](https://duduuu.xyz/zh/posts/px4-ros2-uxrce-dds)（2026-07-21 核对）。
> 该文走 Jetson **原生 UART** `/dev/ttyTHS1`（40 针 Pin8/10/6），本项目走 **CH340 USB 转串口**，
> 引脚表不适用；其单机方案也无多机命名空间（`-n px4_<id>`）和 `ROS_DOMAIN_ID=42`，勿照抄。
> 原生 UART 可作 CH340 出问题时的备选路径。
