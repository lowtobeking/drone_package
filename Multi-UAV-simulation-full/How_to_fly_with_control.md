# 新手飞行Demo教程

> 🔴🔴 **废弃警告（2026-07-31）——这不是本项目当前 MPC 编队栈的飞场流程，别照此放飞。**
> 本文是飞场随附的**单机正弦 Demo** 教程，与本项目实际部署多处不符，照做会连不上/解不了锁/偏航报错：
> - **网段**写的是 `10.41.10.x`、`AG_IP` 等，本项目台架用 `192.168.77.x`，飞场按分配网段（见 `report/飞场对接问题清单.md`）；
> - **控制节点**是 `px4_ros_com offboard_velocity_control`（单机正弦），不是本项目的 `mpc_control` 编队栈；
> - **桥**写的是 `px4_mocap_bridge`，本项目用自研 `mocap_bridge_node`（ENU→NED，已实测）；
> - **缺 `EKF2_MAG_TYPE=5`** —— 动捕方案不设它会 `Preflight Fail: Yaw estimate error` 解不了锁。
>
> ✅ **当前飞场流程以这两份为准**：`report/飞场当天流程.md`（按序 runbook）+ `report/飞场对接问题清单.md`。
> 本文仅留作参考（安全须知/上电顺序等通用部分仍可看），**技术步骤一律以上面两份为准**。

---

## ⚠️ 重要安全与操作注意事项
1. **安全优先**：飞行区域务必选择场地中央位置，场内有人时禁止飞行。
2. **硬件检查**：飞行前严格检查电机、螺旋桨、电池电压、遥控器信号强度，确保设备无松动或损坏，禁止电池低电压时飞行。
3. **环境确认**：确保动捕系统无遮挡、WiFi信号稳定。
4. **参数备份**：修改飞控参数前，通过QGroundControl备份全参数，异常时可一键恢复。
5. **应急操作**：飞行异常时优先用 `Ctrl+C` 终止控制节点（飞控自动降落），非紧急避免直接拨 `Kill` 开关（可能导致设备硬着陆）。
6. **操作提醒**：通常ros2环境加载用的是`source install/setup.bash`，当前所用机载电脑Jetson开发板的默认shell是`zsh`，加载需选用`source install/setup.zsh`，加载环境是请根据所用shell，正确选择。

---

## 1. PX4飞控配置
### 1.1 外部位置控制启用
1. 下载并安装[QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)，连接飞控后进入**参数设置**界面。

### 1.2 ESP32无线数传配置（DroneBridge）
1. **固件刷写**：访问[DroneBridge在线刷写工具](https://drone-bridge.com/flasher/)，选择对应ESP32型号的固件，版本使用`v2.1.0(stable)`。

2. **基础网络配置**：
   - 连接WiFi热点 `DroneBridge for ESP32`，密码 `dronebridge`。
   - 浏览器访问 `http://192.168.2.1` 进入配置页（若无法访问，检查WiFi连接或重启ESP32）。
   - 根据实际焊接引脚配置**UART参数**（波特率推荐 `115200`，与飞控一致），保存并重启。

3. **地面端通信配置**：
   - 在配置页添加**UDP客户端**：
     - `IP地址`：填写地面端电脑的实际IP（需与ESP32同网段）。
     - `端口`：默认 `14550`，多设备时需**递增**（如 `14551`、`14552`，避免冲突）。
     - 协议选择 `UDP`，保存并重启。
   - 打开QGroundControl → **通信设置** → 添加**UDP连接**，端口与上一步一致，保存。

4. **MAVLink连接检查**：
   - 确认 `MAVLink_0` 配置为使用 `TELEM1` 口（通常无需修改）。
   - 检查 `TELEM1` 波特率与ESP32 UART一致（推荐 `115200`）。
   - 重启飞控，拔下USB线，等待无线连接，确认QGroundControl显示飞控在线。

### 1.3 飞控核心参数修改（附解释）
在QGroundControl参数设置中搜索并修改：

| 参数名            | 推荐值         | 参数解释                                                                 |
|-------------------|----------------|--------------------------------------------------------------------------|
| `EKF2_EV_CTRL`    | `11`           | 外部视觉融合控制：`11`（二进制`1011`）表示同时融合位置+姿态数据。     |
| `EKF2_HGT_REF`    | `vision`       | 高度参考源：设为 `vision`，完全依赖动捕/视觉数据。                     |
| `EKF2_GPS_CTRL`   | `0`            | GPS控制：`0` 禁用GPS（纯外部定位场景）。                               |
| `EKF2_BARO_CTRL`  | `disabled`     | 气压计控制：`disabled` 禁用（避免与视觉高度冲突）。                    |
| `EKF2_RNG_CTRL`   | `0`            | 测距仪控制：`0` 禁用（无硬件时关闭）。                                 |
| `UXRCE_DDS_CFG`   | `Ethernet`     | XRCE-DDS传输方式：`Ethernet`（通过UDP与机载电脑通信）。                |
| `UXRCE_DDS_AG_IP` | `170461796`    | XRCE-DDS Agent IP（整数格式）：`10.41.10.100` 转换为整数是 `170461796`。 |
| `UXRCE_DDS_PRT`   | `8888`         | XRCE-DDS通信端口：与机载电脑DDS Agent保持一致。                         |
| `UXRCE_DDS_DOM_ID`| `0`            | DDS域ID：同一网络内设备需一致，避免与其他DDS系统冲突。                  |
| `COM_OF_LOSS_T`   | `0.2`          | 外部定位丢失超时（秒）：超过 `0.2s` 未收到数据触发保护。               |
| `COM_OBL_RC_ACT`  | `Land mode`    | 遥控器异常动作：设为 `Land mode`（自动降落）。                          |

### 1.4 遥控器配置
1. 连接接收机与飞控，确保绑定成功。
2. 在QGroundControl**遥控器设置**中配置：
   - `Kill`开关：紧急停机（非紧急避免使用）。
   - `Arm`开关：解锁/上锁飞控。
   - `Flight mode`开关：切换飞行模式（如自稳、位置、Offboard）。
3. 保存并重启飞控，测试各开关功能正常。

---

## 2. 动捕系统连接
### 2.1 动捕软件配置
1. 开启动捕软件，创建刚体并调整重心。
2. 确认刚体跟踪稳定后，开启**VRPN协议传输**。

### 2.2 VRPN客户端启动（ROS2）
1. 打开Windows Terminal，切换至 `Ubuntu22-04 (vrpn_mocap)` 选项卡。
2. （可选）验证VRPN安装：
   ```bash
   ls /usr/local/include/vrpn_Tracker.h  # 检查头文件
   ls /usr/local/lib/libvrpn.so           # 检查库文件
   # 若不存在，需重新编译安装VRPN
   ```
3. 启动VRPN客户端（修改 `server` 为动捕系统IP）：
   ```bash
   ros2 launch vrpn_mocap client.launch.yaml server:=127.0.0.1 port:=3883
   ```
4. 验证连接：
   ```bash
   ros2 topic list
   # 看到 `/vrpn_mocap/<刚体名>/pose` 话题说明成功
   ```

---

## 3. 动捕数据转换
### 3.1 转换节点编译与启动
1. 确保 `px4_mocap_bridge` 仓库已克隆到 `~/ros2_ws/src`。
2. 打开Ubuntu22-04终端，编译工作空间：
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install  # 方便代码修改后无需重编译
   source install/setup.bash         # 刷新环境变量
   ```
3. 启动转换节点（话题名等配置详见仓库README）：
   ```bash
   ros2 run px4_mocap_bridge mocap_to_px4_node
   ```

### 3.2 转换数据验证
```bash
ros2 topic list
ros2 topic echo /fmu/in/vehicle_visual_odometry  # 查看数据内容
```

---

## 4. 机载电脑连接配置
### 4.1 SSH连接机载电脑
1. 打开PowerShell，运行：
   ```bash
   ssh nvidia@192.168.1.X -i ~/.ssh/id_ed25519
   # 替换：
   # - 192.168.1.X → 机载电脑实际IP（需在路由器设为静态）
   # - ~/.ssh/id_ed25519 → 私钥路径（权限报错时运行 chmod 600 ~/.ssh/id_ed25519）
   ```

### 4.2 Micro XRCE-DDS Agent安装与启动
#### 方式一：源码安装（推荐）
```bash
# 克隆仓库
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)  # 多线程编译加速
sudo make install
sudo ldconfig /usr/local/lib/  # 刷新库缓存
```

#### 方式二：Snap安装（快速）
```bash
sudo snap install microxrce-ds-agent --edge
```

#### 启动DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888 -v6
# 参数说明：
# - udp4：IPv4 UDP协议
# - -p 8888：监听端口（与飞控一致）
# - -v6：详细日志（调试用）
```

### 4.3 航标灯节点启动（可选）
打开新终端连接机载电脑，运行：
```zsh
cd ~/ros2_ws
source install/setup.zsh
ros2 run relay_control hardware_node
# 首次运行需按README安装依赖
```

---

## 5. 飞行测试
### 5.1 地面检查（关键！）
1. **遥控器测试**：
   - 拨 `Kill` 开关，确认飞控锁定。
   - 拨 `Arm` 开关，确认能正常解锁/上锁（解锁后电机怠速，注意安全）。
   - 拨 `Flight mode` 开关，确认QGroundControl模式切换正常。

2. **数据链路检查**：
   - 开启动捕、VRPN客户端、转换节点，确认话题数据稳定。
   - 启动DDS Agent，在QGroundControl **MAVLink控制台** 运行：
     ```bash
     listener vehicle_visual_odometry    # 查看外部视觉数据
     listener estimator_status_flags      # 确保cs_ev_开头标志位均为True
     listener vehicle_odometry            # 查看飞控自身位置
     ```

### 5.2 飞行前准备
1. 打开Windows Terminal，新建4个选项卡并SSH连接机载电脑：
   - **选项卡1（DDS）**：
     ```bash
     MicroXRCEAgent udp4 -p 8888 -v6
     ```
   - **选项卡2（航标灯）**：
     ```bash
     cd ~/ros2_ws && source install/setup.zsh
     ros2 run relay_control hardware_node
     ```
   - **选项卡3（数据记录）**：
     ```bash
     cd ~/ros2_ws && source install/setup.zsh
     ros2 bag record -o run_001 \
       /offboard/pos_ned /offboard/ref_ned /offboard/payload_ned \
       /offboard/origin_ned /offboard/state
     ```
   - **选项卡4（飞行控制）**：
     ```bash
     cd ~/ros2_ws && source install/setup.zsh
     ros2 run px4_ros_com offboard_velocity_control --ros-args \
       -p loop_hz:=100.0 \          # 控制频率（Hz）
       -p warmup_sec:=1.0 \          # 解锁后预热时间（s）
       -p takeoff_up:=0.8 \          # 起飞高度（m）
       -p takeoff_ramp_sec:=2.0 \    # 爬升时间（s）
       -p yaw_fixed_rad:=0.0 \       # 固定偏航角（rad）
       -p yaw_ramp_sec:=1.5 \        # 偏航调整时间（s）
       -p traj_amp_x:=0.6 \          # X轴轨迹振幅（m）
       -p traj_amp_y:=0.4 \          # Y轴轨迹振幅（m）
       -p traj_period:=20.0 \         # 轨迹周期（s）
       -p loops:=2 \                  # 循环次数
       -p z_tol:=0.08 \               # Z轴误差容忍度（m）
       -p xy_tol:=0.10 \              # X/Y轴误差容忍度（m）
       -p mission_mode:=1             # 任务模式（1=轨迹飞行，0=悬停）
     ```
   - **选项卡5/6/7（ros2话题查看）**：
     ```bash
     cd ~/ros2_ws && source install/setup.zsh
     ros2 topic list
     ros2 topic echo /offboard/state  # 查看数据内容 ⑤
     ros2 topic echo /fmu/out/estimator_flags  # 查看数据内容 ⑥
     ```


2. 开启相机记录实验视频。

### 5.3 开始飞行
1. 确认地面检查通过，区域安全。
2. 依次启动：DDS → 航标灯 → 数据记录 → 飞行控制节点。
3. **关键**：保持飞行控制终端聚焦，紧急时按 `Ctrl+C` 终止。
4. 解除`kill`，飞机将在`arm`后，自动执行起飞和轨迹。

### 5.4 飞行结束与数据处理
1. 飞行完成后，立即 `Ctrl+C` 终止飞行控制节点。
2. 拨 `Kill` 开关锁定飞控，终止其他节点和相机。
3. 切换飞控至自稳模式。
4. 导出数据（需提前准备脚本）：
   ```bash
   cd ~/ws_sensor_combined/tools
   python3 export_offboard_bag_to_csv.py ~/ros2_ws/run_001 ~/ros2_ws/run_001.csv
   ```

---

## 常见问题排查
1. **ESP32连不上QGC**：检查WiFi连接、UDP端口、防火墙。
2. **飞控收不到动捕数据**：确认 `EKF2` 参数、DDS Agent状态、IP/端口配置。
3. **DDS Agent失联**：/fmu/out/vehicle_odometry 话题不更新，/fmu/out/estimator_status_flags 话题cs_ev_开头标志位均为False，即DDS Agent未连接，重启命令：
   ```nsh
   uxrce_dds_client stop
   uxrce_dds_client start
   ```
