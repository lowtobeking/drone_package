# FC 安全参数配置单（QGC 逐项勾选版）

> **本单适用硬件**（2026-07-22 更新，据飞场文档 `How_to_fly_with_control.md` + 用户确认）：
>
> | 部件 | 型号 |
> |---|---|
> | 飞控 | **ZeroOne X6 Air+**（FMUv6X 架构 / STM32H7 / 双冗余 IMU / **带网口**） |
> | 机载电脑 | Nvidia Orin NX |
> | 数传 | **3DR 433MHz**（SiK 电台，串口） |
> | 电源模块 | **One PMU** 电流计 |
> | 定位 | **飞场动捕系统**（VRPN → `px4_mocap_bridge` → `/fmu/in/vehicle_visual_odometry`） |
>
> **操作方式**：QGC → 车辆设置 → 参数 → 搜索参数名 → 改值 → 保存 → 重启复核。
> **每架 FC 都要独立配一遍。**

---

## ⚠️ 本单已按新硬件重写 —— 与 2026-07-22 上午版的差异

上午那版是按 **Pixhawk 6C + 串口 XRCE + LQ-3** 写的，硬件换了之后大量条目失效：

| 项 | 旧版（6C，已作废） | 本版（X6 Air+） | 依据 |
|---|---|---|---|
| XRCE 传输 | `UXRCE_DDS_CFG=102`（TELEM2 串口） | **`Ethernet`** | 飞场固定要求；X6 Air+ 官网标明"支持网口通信"，PX4 `boards/px4/fmu-v6x/default.px4board` 有 `CONFIG_BOARD_ETHERNET=y` |
| DDS 域号 | 42 | **0** | 飞场固定要求 |
| `COM_OF_LOSS_T` | 0.5 | **0.2** | 飞场固定要求 |
| `COM_OBL_RC_ACT` | Hold | **Land** | 飞场固定要求 |
| 串口设备路径 | `/dev/ttyS3`（6C 实机确认） | **未知，须实机确认** | 板型不同，**严禁照抄** |
| 数传 | LQ-3 | 3DR 433MHz | 用户确认 |
| 高度基准 | 气压计 | **vision（气压计禁用）** | 飞场固定要求 |

> ⚠️ **凡标「6C 实测」的结论对本飞控一律无效，须重验。** 固件级参数（安全/失效保护）
> 与板型无关，可沿用。

---

## 📊 2026-07-22 实测：这块 X6 Air+ 的当前状态

USB 接仿真机经 MAVLink 读取（只读，未改任何参数）。**固件 PX4 v1.16.0**，
`SYS_AUTOSTART=4019`（Holybro X500 V2，四旋翼 X）。

> ⚠️ **读参数的坑**：PX4 的 INT32 参数经 MAVLink `param_value`(float 字段)**按字节透传**，
> 直接当 float 读会得到 `1.4013e-45` 这类垃圾值，必须按 `param_type` 用
> `struct.unpack('<i', struct.pack('<f', v))` 重新解释。

### ✅ 已就绪（不用动）
| 参数 | 当前值 | 说明 |
|---|---|---|
| `CAL_ACC0/GYRO0/MAG0_ID` | 均非 0 | 传感器**已校准**（但装机后须重做，安装方向变了） |
| `RC_MAP_ROLL/PITCH/THROTTLE/YAW` | 1/2/3/4 | 摇杆已映射（标准 AETR） |
| `RC1_MIN/MAX/TRIM` | 999/1999/1497 | RC **已校准**（非默认值） |
| `RC_MAP_FLTMODE` | 5 | 模式开关在通道 5 |
| `UXRCE_DDS_DOM_ID` | **0** | 已是飞场要求值 |
| `UXRCE_DDS_PRT` | 8888 | ✓ |
| `SER_TEL1_BAUD` | 57600 | ✓ 适合 3DR 433 |
| `COM_ARM_WO_GPS` | 1 | ✓ **保持不动** |
| `MAV_0_CONFIG` | 101 (TELEM1) | ✓ |

### ⚠️ 必须改（当前值 → 目标值）
| 参数 | 当前 | 目标 | 后果 |
|---|---|---|---|
| `UXRCE_DDS_CFG` | **0 (Disabled)** | Ethernet | XRCE 没启用，ROS2 完全不通 |
| `UXRCE_DDS_AG_IP` | **2130706433**(127.0.0.1) | 170461796 (10.41.10.100) | 指向本机，连不上 Agent |
| `EKF2_EV_CTRL` | 0 | **11** | 不融合外部视觉 → 动捕白接 |
| `EKF2_HGT_REF` | 1 | **vision** | 高度源不对 |
| `EKF2_GPS_CTRL` | 7 | **0** | 仍在用 GPS |
| `EKF2_BARO_CTRL` | 1 | **0** | 气压计与视觉高度冲突 |
| `EKF2_RNG_CTRL` | 1 | **0** | 无测距仪 |
| **`RC_MAP_KILL_SW`** | **0（未映射）** | 分配拨杆 | **没有 kill 开关，安全底线缺失** |
| **`RC_MAP_ARM_SW`** | **0（未映射）** | 分配开关 | 无独立 Arm 开关 |
| `COM_OBL_RC_ACT` | **0 (Position mode)** | **4 (Land mode)** | offboard 丢失后切 Position——室内无 GPS 时极危险 |
| `NAV_RCL_ACT` | 2 (Return) | **3 (Land)** | 室内 RTL 比就近降落危险 |
| `COM_OF_LOSS_T` | 1.0 | **0.2** | 飞场要求 |
| `MPC_XY_VEL_MAX` | **12.0** | 2.0 | 默认值，5×5 场地里等于没有限制 |
| `MPC_Z_VEL_MAX_UP` | 3.0 | 1.5 | 同上 |
| `MPC_TILTMAX_AIR` | 45.0 | 25–30 | 同上 |
| `GF_MAX_HOR_DIST` | **0（围栏未启用）** | 2.0 | 围栏形同虚设 |
| `GF_MAX_VER_DIST` | **0（未启用）** | 3.0 | 同上 |
| `BAT_CRIT_THR` | 0.05 | 0.07 | — |
| `BAT_EMERGEN_THR` | 0.03 | 0.05 | — |

### 🔴 电池监控**完全未配置**（One PMU 接上后必须做）
| 参数 | 当前值 | 含义 |
|---|---|---|
| `BAT1_SOURCE` | **-1** | 电池源未选 |
| `BAT1_N_CELLS` | **0** | 节数未设 |
| `BAT1_V_DIV` | **-1** | **电压分压未标定** |
| `BAT1_A_PER_V` | **-1** | **电流系数未标定** |
| `BAT1_CAPACITY` | **-1** | 容量未设 |
| `COM_LOW_BAT_ACT` | **0 (Warning only)** | **低电量只警告、不采取任何动作** |

→ 现状等于**完全没有电池保护**。`BAT_LOW_THR=0.15` 看着配好了，但没有电压标定和
动作设置，它不会起任何作用。One PMU 接上后必须走完整标定流程。

---

## 🔵 第零批：网络与 XRCE —— 飞场文档**漏掉**的一步在这里

### N0. 飞控自身 IP（⚠️ 飞场文档没写，不配则 XRCE 根本连不上）

PX4 以太网配置在 **SD 卡的 `/fs/microsd/net.cfg`**，默认 IP 是 **192.168.0.3**
（源码 `src/systemcmds/netman/netman.cpp` 确认），而飞场 Agent 在 **10.41.10.100**
——**网段不匹配，必须先改**。

- [ ] SD 卡根目录建/改 `net.cfg`，网段对齐飞场：
      ```
      DEVICE=eth0
      BOOTPROTO=static
      IPADDR=10.41.10.<飞场分配给飞控的地址>
      NETMASK=255.255.255.0
      ROUTER=10.41.10.1
      DNS=10.41.10.1
      ```
- [ ] 插卡重启，NSH 里 `netman show` 复核实际生效地址
- [ ] 从 Orin NX `ping` 通飞控 IP

> **须向飞场确认**：飞控该用哪个 IP、网关是否 `10.41.10.1`、是否有 DHCP。

### N0b. ✅ 以太网已实测可用（2026-07-22 确认，好消息）

读到以下**只在以太网启用时才存在**的参数，说明这块板子以太网硬件、固件支持、
配置三者齐备：

| 参数 | 当前值 | 含义 |
|---|---|---|
| `MAV_2_CONFIG` | 1000 | MAVLink 实例 2 走以太网 |
| `MAV_2_UDP_PRT` | 14550 | 已配 UDP 端口 |
| `MAV_2_REMOTE_PRT` | 14550 | |
| `MAV_2_BROADCAST` | **1** | **已开广播** |
| `MAV_2_RATE` | 100000 | 100KB/s（远高于串口） |

**⇒ 接上网线，QGC 通过网络就能自动发现飞控**，台架阶段连 USB 和数传都不需要。
比飞场文档的 ESP32 方案更直接（那套是为没有网口的飞控准备的）。

> `uxrce_dds_client` 的 `module.yaml` 确认 `supports_networking: true`，
> 选 ethernet 时客户端以 `-t udp` 启动，端口取 `UXRCE_DDS_PRT`（当前已是 8888 ✓）。
> QGC 里 `UXRCE_DDS_CFG` 是下拉菜单，直接选 **Ethernet** 即可，不用管背后的数字。

### N1. XRCE-DDS（飞场固定要求）

- [ ] `UXRCE_DDS_CFG` = **Ethernet**
- [ ] `UXRCE_DDS_AG_IP` = **170461796**（即 `10.41.10.100`，机载电脑/Agent 地址）
- [ ] `UXRCE_DDS_PRT` = **8888**
- [ ] `UXRCE_DDS_DOM_ID` = **0**　← 注意不是 42
- [ ] **重启飞控**（`UXRCE_DDS_*` 多为 `reboot_required`）

> ⚠️ **本项目所有 companion 侧也要一并改成 `ROS_DOMAIN_ID=0`**（原为 42）：
> 域号只设一侧会【静默失效】——Agent 会话正常、22 话题全建、零报错，
> 但 ROS2 里一个话题都看不到（2026-07-21 实测踩过整轮）。

---

## 🟣 第一批：EKF2 外部视觉（飞场固定要求，动捕的前提）

- [ ] `EKF2_EV_CTRL` = **11**（二进制 1011：融合位置 + 姿态）
- [ ] `EKF2_HGT_REF` = **vision**
- [ ] `EKF2_GPS_CTRL` = **0**（禁用 GPS）
- [ ] `EKF2_BARO_CTRL` = **disabled**（禁用气压计，避免与视觉高度冲突）
- [ ] `EKF2_RNG_CTRL` = **0**（无测距仪）
- [ ] 重启后 NSH 验证：
      ```
      listener vehicle_visual_odometry     # 有数据
      listener estimator_status_flags      # cs_ev_ 开头标志位应全为 True
      listener vehicle_odometry            # 飞控自身位置解算正常
      ```

### ⛔ `COM_ARM_WO_GPS` 必须保持 **1**

`EKF2_GPS_CTRL=0` 已经把 GPS 彻底禁用。若把 `COM_ARM_WO_GPS` 设成 0（要求 GPS lock），
**将永远无法解锁**。上午那版清单里我标的"等定位方案定了再配 0"——现在方案定了，
结论是**这项就该保持默认 1，不要改**。

> ⚠️ 连带作废：气压计被禁用后，"无定位也能飞 Altitude 模式"（上午基于
> `local_altitude_invalid=false` 得出）在本配置下**不再成立**，高度完全依赖动捕。

---

## 🟢 第二批：安全失效保护（固件级，与板型无关）

### A. OFFBOARD 失联 — companion 方案的命脉
- [ ] `COM_OF_LOSS_T` = **0.2** s（飞场值）
- [ ] `COM_OBL_RC_ACT` = **Land**（飞场值）
- [ ] `COM_RCL_EXCEPT` → **不勾选 Offboard**（保留 RC 作为安全兜底）

### B. RC 失联
- [ ] `COM_RC_LOSS_T` = 0.5 s
- [ ] `NAV_RCL_ACT` = **Land**（室内首飞就近落，勿用 Return）

### C. Kill 开关 / 解锁落地
- [ ] `RC_MAP_KILL_SW` = 分配一个**拨杆**通道
- [ ] `COM_KILL_DISARM` = 5 s
- [ ] `COM_DISARM_LAND` = 2 s
- [ ] **无桨实拨测试**：解锁后拨 kill，确认电机瞬间全停

> ⚠️ **外场处置顺序以飞场规程为准**：飞场文档明确「飞行异常时**优先 `Ctrl+C` 终止
> 控制节点**（飞控自动降落），非紧急避免直接拨 Kill（可能硬着陆）」。
> Kill 仍是最后手段、地面必须测通，但**不是空中异常时的第一动作**。

### D. 速度/姿态硬帽 — 须比 companion 限幅**宽**
> companion `conservative` 限 v≤1.5 / climb≤1.0 / a≤2.0。FC 这层更宽，
> 正常由 companion 控、失常才由 FC 兜底。**设反了会让 FC 天天限制正常控制。**
> ⚠️ 室内飞场尺度极小（飞场 demo：起飞高度 0.8m、轨迹振幅 0.6×0.4m），
> 这组值可能仍需按实际场地进一步下调。

- [ ] `MPC_XY_VEL_MAX` = 2.0 m/s
- [ ] `MPC_Z_VEL_MAX_UP` = 1.5 m/s
- [ ] `MPC_Z_VEL_MAX_DN` = 1.0 m/s
- [ ] `MPC_TILTMAX_AIR` = 25–30°

### E. 电池失效保护（One PMU）
- [ ] **先标定，再设阈值**——否则保护形同虚设：
      - [ ] 万用表实测电池电压，与 QGC 显示对比；偏差 >0.2V 则标定 `BAT1_V_DIV`
      - [ ] 电流读数与已知负载对比，偏差大则标定 `BAT1_A_PER_V`
      - [ ] 若 One PMU 是 I2C/CAN 数字模块，则无需分压标定，但要确认驱动已识别
        （QGC 电源页能读到电压电流即可）
      > One PMU 的分压/分流系数官网未公开，**以实测标定为准**，别抄任何现成数值。
- [ ] `BAT_LOW_THR` = 0.15
- [ ] `BAT_CRIT_THR` = 0.07
- [ ] `BAT_EMERGEN_THR` = 0.05
- [ ] `COM_LOW_BAT_ACT` = Return at low, Land at critical
- [ ] 确认电池节数 `BAT1_N_CELLS` 与实际一致

---

## 🟡 第三批：数传（3DR 433MHz，与飞场文档的 ESP32 方案不同）

飞场文档写的是 ESP32 DroneBridge（WiFi→UDP 14550），**你用的是 3DR 433 SiK 电台**，
连接方式完全不同：

- [ ] `MAV_0_CONFIG` = TELEM1（PX4 默认即为 TEL1，源码 `module.yaml` 确认，通常无需改）
- [ ] `SER_TEL1_BAUD` 与 3DR 电台配置**一致**（SiK 默认常为 57600）
      　　_不一致的表现是 QGC 连不上或乱码，很容易误判成"数传坏了"_
- [ ] 地面端 3DR 接笔记本 USB → QGC 选**串口**连接（不是 UDP）
- [ ] 433MHz 与飞场其他设备是否冲突 —— **须向飞场确认**

> **台架用 USB 直连、放飞用数传**：USB 带宽高，配参数/校准/固件升级都该用它；
> 飞起来后 USB 够不着，必须靠数传看遥测。**不是两条同时接着用。**

---

## 配完之后

- [ ] QGC 点保存 → **重启飞控** → 回来逐项复核没回滚
- [ ] `reboot_required` 的参数（`UXRCE_DDS_*` 等）必须重启才生效
- [ ] QGC → 参数 → 工具 → **保存到文件**，做一份全参数备份
      　　_飞场文档也明确要求：改参数前先备份，异常可一键恢复_
- [ ] 链路复验（在 Orin NX 上）：
      ```
      ros2 topic hz /fmu/out/vehicle_status      # 首选判据
      ros2 topic list --no-daemon | grep -c fmu  # 仅参考：有发现竞态，实测连查三次得 22/22/0
      ```

---

## 📋 仍须向飞场确认的清单

1. **飞控分配哪个 IP**、网关是否 `10.41.10.1`、有无 DHCP（决定 `net.cfg` 怎么写）
2. **Orin NX 怎么接入飞场网络**（有线/WiFi、分配哪个 IP）
3. **动捕/VRPN server 跑在哪台机器**、刚体命名规则、输出频率
4. **动捕标定体积**的实际尺寸（≠ 场地尺寸，决定可飞范围与围栏值）
5. **3DR 433MHz 数传飞场是否认可**（文档写的是 ESP32，频段是否与场内设备冲突）
6. `px4_mocap_bridge` 仓库地址与话题配置（文档说"详见仓库 README"，我们还没有）

---
**关联**：`How_to_fly_with_control.md`（飞场原始文档）、`无桨台架验证清单.md`、
`真机安全配置清单_FC.md`（分层原理/中止判据/渐进放飞协议）。
