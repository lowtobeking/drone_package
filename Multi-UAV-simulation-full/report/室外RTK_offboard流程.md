# 室外 RTK — offboard 控制操作流程（单机 / 双机）

> 适用：ZeroOne X6 Air+（PX4 v1.16）+ Jetson Orin NX + C-RTK 9PS，**室外 GPS/RTK 定位**。
> 室内动捕流程见 `真机安全参数配置单_QGC.md` / `飞场当天流程.md`——**两套定位参数集互斥，别混**。
> 本文所有参数名、默认值、判据均取自当前代码（`mpc_control/`、`launch/real_hardware_launch.py`），
> 2026-08-18 核实。

---

## 0. 机身与链路现状（2026-08-18）

| | A10 | A08 |
|---|---|---|
| SSH | `ssh A10` | `ssh A08` |
| RTK | ✅ C-RTK 9PS 接 **GPS1 串口** | ❌ 未装 |
| 飞控 IP | `10.41.10.2` | `192.168.77.2` |
| Jetson eno1 | `10.41.10.100` | `192.168.77.100` |
| XRCE | ✅ 通（51 话题） | ✅ 通 |
| 磁罗盘 | ✅ 已启用 | ✅ 已启用 |
| EKF2 定位源 | ✅ GPS（`GPS_CTRL=7`） | ⚠️ **仍是动捕**（`GPS_CTRL=0`/`EV_CTRL=11`），室外前必须切 |

⚠️ 两机飞控网段不一致，建议最终统一到 `192.168.77.x`。

---

## 1. 前置条件（不满足 PX4 会直接拒绝进 OFFBOARD）

在机载电脑上逐条验，**任何一条不过就不要装桨**：

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

| # | 条件 | 命令 | 期望 |
|---|---|---|---|
| 1 | XRCE 链路 | `ros2 topic hz /fmu/out/vehicle_status_v1` | ≈2 Hz |
| 2 | **GPS 定位有效** | `ros2 topic echo --once /fmu/out/vehicle_local_position` | `xy_valid: true`、`xy_global: true`、`ref_lat` 非 nan |
| 3 | RTK 质量 | `ros2 topic echo --once /fmu/out/vehicle_gps_position` | `fix_type≥3`（**6=RTK Fixed** 最佳）、`satellites_used` 足够、`hdop` 小 |
| 4 | **航向有来源** | `ros2 topic echo --once /fmu/out/estimator_status_flags` | `cs_yaw_align: true`、`cs_mag_fault: false` |
| 5 | 无阻塞失效 | 同上 topic 的 `failsafe_flags` | `global_position_invalid: false` |

> 🔴 **为什么 GPS 是硬门槛**：offboard 下发的是 **velocity setpoint**，PX4 的 `offboardCheck.cpp`
> 对 velocity / position / acceleration 三种模式**都要查 `local_position` 有效性**。室内无定位
> 必被拒，与链路好坏无关。（室内唯一能验的是 `body_rate` 心跳，见 `tools/uplink_test.py`。）

---

## 2. 出发前必须在台架做完

1. **室外重新标定指南针**。磁罗盘是 2026-08-18 才从 `SYS_HAS_MAG=0` 恢复的，历史标定是换 RTK
   模块之前做的，已作废。QGC 里做，**开阔地、远离车辆/钢筋/大功率线缆**。
2. **首次 launch 会现编 acados OCP（数分钟）**——代码注释明确要求"外场前在台架跑通一次"。
   产物缓存在 `~/.cache/mpc_control/`，跨重启命中（已从 `/tmp` 迁出，`/tmp` 每次开机被清）。
3. **A08 若要用，先切 RTK 参数集**（见 §9），并**先导出当前配置存档**。
4. **拆桨跑一遍完整流程**（§3/§4 全走一遍，只是不装桨）。能把参数错、话题名错、定位不满足
   全部暴露，代价为零。
5. 🔴 **做完 §10 的 A 级检查**（**A2 油门磁干扰** 为主 / A3 低电量动作；A1 板子朝向已于
   2026-08-19 降级为读一个参数）——都不用起飞，
   但它们挡的是最可能的炸机方式。
6. **想好数据怎么记**（§8）：SD 卡在不在、`diag_monitor --log` 的队形名对不对、
   要不要顺带录 τ 辨识的 rosbag。**这些飞后补不了。**

---

## 3. 单机（solo1）

> 🔴 **单机也要起 leader 节点**。轨迹参考（悬停点 / 直线 / 圆周）全部由 `leader_node`
> 通过 `/leader/state` 提供，`solo1` 不例外；而且**第 2 层失联降落正是靠"收不到 leader"
> 触发的**，不起 leader 等于放弃那一层保护。

### 3.1 三种轨迹

| 轨迹 | 场景名 | leader 参数 | 说明 |
|---|---|---|---|
| **悬停** | `IN_solo1_hover` | `mode: hover` | 定点悬停。**首飞必须从这个开始** |
| **直线** | `IN_solo1_line` | `mode: line`、`speed: 0.4`、`max_distance: 2.2`、`yaw_mode: tangent` | 往返直线，到端点梯形减速（`line_decel=0.5 m/s²`）防过冲 |
| **匀速圆周** | `IN_solo1_circle` | `mode: circle`、`speed: 0.5`、`radius: 1.2`、`yaw_mode: center` | 匀速圆周，机头始终朝圆心 |

三者共同限幅（`IN_*` 系列）：`target_alt −1.2 m`、`max_speed 0.5`、`max_climb 0.5`、
`safety_max_alt 2.5`、`safety_min_alt 0.3`、`safety_max_track_dist 1.5`。

**`yaw_mode` 三选一**（`leader_node` 实现）：
- `fixed` — 固定起飞朝向（默认，最保守）
- `center` — 机头朝圆心（圆周用）
- `tangent` — 机头跟随飞行方向（直线/展示用）

> 🔴 **首飞建议一律先用 `yaw_mode:=fixed`**。`center`/`tangent` 会让飞机在飞行中**主动转向**，
> 而板载罗盘的**动态**表现（推油门时的电流干扰，§10 A2）尚未验证过。转向过程中航向估计若变差，
> 会把误差直接变成位置发散。**先用 fixed 建立基线，A2 通过后再放开转向。**
>
> ⚠️ **但别把 `fixed` 当成航向保险**（2026-08-19 补正）：offboard 下发的速度指令是**世界系 NED**，
> 飞控要用**估计的航向**把它旋转到机体系才能算推力方向。**航向估计错 θ，飞机就朝偏 θ 的方向飞
> ——与 `yaw_mode` 无关**；位置控制器随后朝错误方向"修正"，形成正反馈（画圈/发散）。
> `yaw_mode` 只决定要不要**主动指令**转向，挡不住估计本身是错的。
> ⇒ **A2 是 offboard 的前置条件，不能因为用了 `fixed` 就跳过。**

> ⚠️ `IN_*` 是**室内尺寸**（`target_alt −1.2 m`、`radius 1.2 m`）。室外照搬**不是更保守而是更危险**
> ——见 §4.4 的说明。

### ✅ 室外场景已建：`OUT_solo1_*`（2026-08-19，按场地半径 15 m）

| 场景 | leader | 说明 |
|---|---|---|
| `OUT_solo1_hover` | `mode: hover` | **首飞从这个开始** |
| `OUT_solo1_line` | `speed 0.8`、`max_distance 5.0`、`yaw_mode fixed` | 最远离原点 5 m |
| `OUT_solo1_circle` | `speed 0.8`、`radius 4.0`、`yaw_mode fixed` | 向心加速度仅 0.16 m/s² |

三者共同限幅：`target_alt −3.0`、`safety_max_alt 8.0`、`safety_min_alt 0.3`、
`safety_max_track_dist 3.0`、`max_speed 1.0`、`max_climb 0.8`。

> 📌 **2026-08-20 两处改动**（均已写进 `scenarios.yaml` 的注释）：
> · `target_alt` −4.0 → **−3.0**：首飞实测 4 m 偏高；3 m 仍远离地效，离 8 m 上限余量 5 m。
> · `safety_min_alt` 1.0 → **0.3**：1.0 时飞机停在地面（读数 0.6~1.0 m）就持续违规
>   ⇒ 升级到 RELINQUISH ⇒ 节点停发 setpoint 并每秒重发 Hold，现场表现为
>   「arming denied, change to manual first」、飞手切挡还被拽回。

**安全分层（顺序不能反）**：
```
场地半径 15m
  └ 飞控 GF_MAX_HOR_DIST = 12m      ← 留 3m 反应余量（相对 home/起飞点）
      └ 轨迹最远离原点 4~5m
          └ 最坏 = 轨迹 5 + 跟踪误差 3 = 8m < 12m ✅
```
⚠️ `safety_max_track_dist` 限的是 **|pos − ref|（偏离参考点）**，不是距出生点，与围栏
不同量纲、不能直接相加比较；上面的"最坏"才是二者叠加后的实际包络。

**围栏下发**（`tools/fc_configure.py` 新增 `geofence_out15` 组，默认 dry-run）：
```bash
python3 tools/fc_configure.py -g geofence_out15            # 先看
python3 tools/fc_configure.py -g geofence_out15 --apply    # 再写
```
⚠️ 与室内 `geofence` 组（3.0 m）**互斥**，别同时下发。

🔴 **三个场景 `yaw_mode` 一律 `fixed`**：航向来源是 2026-08-18 才恢复、08-19 才标定的板载
磁罗盘，**尚未在飞行中验证**（§10 A1/A2 未做）。确认航向可信后再放开 `center`/`tangent`。

### 3.2 启动（两个终端）

```bash
# ── 终端 1：机载电脑（drone 0）──
ssh A10
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch mpc_control real_hardware_launch.py \
    role:=drone drone_id:=0 \
    scenario:=IN_solo1_hover \
    px4_version:=1.16 \
    start_agent:=false \
    mocap:=false \
    calib_shared_origin:=false \
    conservative:=true \
    alt_sync:=true \
    comms_loss_land_s:=5.0

# ── 终端 2：leader 参考节点（可跑在同一台机载电脑或地面 PC，只要同 DDS 域）──
ros2 launch mpc_control real_hardware_launch.py \
    role:=leader \
    scenario:=IN_solo1_hover \
    yaw_mode:=fixed \
    mission_duration:=60.0        # 飞 60 s 后自动降落；0=禁用（⚠️见下方计时基准）

# ── 终端 3（可选但强烈建议）：数据记录，见 §8 ──
python3 ~/Multi-UAV-simulation/diag_monitor.py --formation solo1 --log
```

**换轨迹只改 `scenario`**（两个终端要改成同一个）：
`IN_solo1_hover` → `IN_solo1_line` → `IN_solo1_circle`。

**每个参数为什么这么给（错了会怎样）**

| 参数 | 原因 |
|---|---|
| `start_agent:=false` | 🔴 Agent 已是 systemd 自启。不传则 launch 再起一个**抢同一端口** |
| `mocap:=false` | 关 VRPN 桥。不关会起一个永远收不到数据的节点 |
| `calib_shared_origin:=false` | 🔴 该项是**动捕专用**（全场共享原点）。GPS 下必须关，否则 `world_birth` 校准判据用错 |
| `px4_version:=1.16` | 🔴 v1.16 话题名带 `_v1` 后缀。写错**不报错**，只是永远收不到 `vehicle_status` → 卡在 "retry ARM+OFFBOARD" |
| `conservative:=true` | 硬帽 `max_speed≤1.5 / max_climb≤1.0 / max_accel≤2.0 / d_safe≥2.5` |
| `alt_sync:=true` | 外场各机 home 海拔真不同，代码注释建议外场显式开 |
| `comms_loss_land_s:=5.0` | 第 2 层失联降落（§6.2），默认值就是 5.0 |
| `mission_duration:=60.0` | 第 1 层任务完成自动降落（§6.1）。**首飞建议给个值**，别只依赖手动 |

### 3.3 操作顺序

`auto_arm` 真机默认 **false** —— 节点只发 setpoint 流并等待，**不会自己解锁**。

```
1) 起 drone 节点 → 等日志 "waiting RC ARM+OFFBOARD"
   （此时节点已在发 setpoint，满足 PX4「切 OFFBOARD 前 setpoint 流须 >2Hz」）
2) 起 leader 节点（+ diag_monitor）
3) 飞手 RC 解锁
4) 飞手模式开关切 OFFBOARD
5) 节点确认 nav_state=14 + armed → 日志 "OFFBOARD + ARMED confirmed" → 自动爬升到 target_alt
6) leader 的 start_delay 到点后开始走轨迹
7) mission_duration 到点 → 自动降落；或飞手随时切模式夺回控制
```

⚠️ **别用 `auto_arm:=true`** —— 那是台架无桨用的，会让节点自己发 ARM。

**建议推进顺序**：`hover`（确认能悬停住、看 `poserr`）→ `line`（确认能跟直线、端点不过冲）
→ `circle`（最吃跟随性能——持续变向，论文里那个速度环滞后正是在圆周上最先暴露）。
**每一步都先拆桨走一遍**。

---

## 4. 双机（pair2）

双机比单机多三件事：**leader 节点**、**机间通信**、**摆放位置**。

### 4.1 机间通信前提

两台 Jetson 必须在同一 DDS 域且互通。当前走 **LQ-MESH**：**A10 = `192.168.1.101`、A08 = `192.168.1.102`**。
起飞前验：

```bash
# 在 A10 上
ping -c3 192.168.1.102
# 两边都能看到对方的 mpc_node 话题
ros2 topic list | grep -c px4_
```

> 🔴 **2026-08-19 现场改编址：机载电脑一律 `.10x`，不要再用 `.20x`。**
> LQ 电台自己占着 `192.168.1.200` / `.202` / `.203`（三台都开 80+23 配置页），
> 而 A08 原本也配在 `.202` ⇒ **IP 冲突**。现场表现极具迷惑性：同一个 `.202`，
> 从 A10 解析到的是 Jetson（SSH 正常），从地面 PC 解析到的是电台（TCP 被 RST，
> 报"积极拒绝"），且 ARP 归属会随免费 ARP 来回翻。**飞行中一旦 A10 的 ARP 翻到
> 电台，机间 DDS 立刻断链**。判定方法：`arp -a` 看同一 IP 在不同机器上的 MAC 是否一致。

⚠️ FastDDS 单播发现 profile（`deploy/mesh_fastdds_profile.xml`）**尚未在两机验证**。
如果 mesh 上 DDS 发现不稳，先退回同一 WiFi 验证编队逻辑，再解决 mesh。

### 4.2 摆放

按 `config/scenarios.yaml` 的 `formations.pair2_in.birth`（或所选 formation）摆到标记点，
**误差 < `calib_max_origin_offset`（2 m）**。超出会被校准门控拦下（打红字 `calib STUCK`，
守在出生点不动，属于保护不是故障）。

> 🔴 **GPS 编队还有一个未验证项**：`xy_global_align_enable` 那套"用各机 EKF 原点经纬度对齐
> world_birth"的代码（commit `5452692`）**从未在 SITL 验证通过**（探针跑过但没出数据）。
> 不开它 ⇒ 代码盲信配置里的出生点，摆放误差会变成**系统性队形偏移**；开它 ⇒ 逻辑未验证。
> **建议：首次双机先用 `hover`、把间距放大、不开该项**，先确认基础闭环，再单独攻这一项。

### 4.3 启动（三个终端）

```bash
# ── 终端 1：A10（drone 0）──
ssh A10
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mpc_control real_hardware_launch.py \
    role:=drone drone_id:=0 scenario:=<见 §4.4> \
    px4_version:=1.16 start_agent:=false mocap:=false \
    calib_shared_origin:=false conservative:=true alt_sync:=true \
    comms_loss_land_s:=5.0

# ── 终端 2：A08（drone 1）── 只有 drone_id 不同
ssh A08
... 同上 ... role:=drone drone_id:=1 scenario:=<同一个> ...

# ── 终端 3：地面站（leader 参考节点）──
# 可以跑在任一台 Jetson 或地面 PC，只要在同一 DDS 域
ros2 launch mpc_control real_hardware_launch.py \
    role:=leader scenario:=<同一个> \
    yaw_mode:=fixed \
    mission_duration:=90.0        # 飞 90 s 后自动降落；0=禁用

# ── 终端 4（可选但强烈建议）：数据记录，见 §8 ──
python3 ~/Multi-UAV-simulation/diag_monitor.py --formation pair2 --log
```

**顺序**：先两架 drone（等各自 "waiting RC ARM+OFFBOARD"）→ 再起 leader →
飞手**逐机**解锁+切 OFFBOARD → 两机爬升成队 → leader 的就绪门控
（全员 `pos_err < ready_pos_err` 保持 `ready_hold` 秒）通过后才开始运动。

> 就绪门控是设计好的：leader **不会**在飞机还没到位时就开动。日志 `formation ready — starting`。

### 4.4 三种轨迹

现成场景都是**室内尺寸**（`IN_*`：`target_alt=-1.2m`、`radius=0.6m`）。

> 🔴 **不要直接把 `IN_*` 拿到室外用**。`target_alt=-1.2 m` 在室内是安全值，
> 在室外**反而更危险**：①离地 1.2 m 处于强地效区，姿态与高度都更难稳；
> ②一旦出问题**没有任何恢复高度**，从发现异常到触地不足 1 秒；
> ③室外地面不平、有草木，`safety_min_alt=0.3` 的余量不够。
> **室外首飞建议 `target_alt` 取 −3 ~ −5 m**（既留恢复余量，又低于多数场地限高）。

| 轨迹 | 现成场景（室内尺寸） | 关键参数 |
|---|---|---|
| **悬停** | `IN_pair2_hover` | `mode: hover`，`ready_hold: 10`、`ready_pos_err: 0.3` |
| **直线** | `IN_pair2_line` | `mode: line`，`speed: 0.3`、`max_distance: 1.4`、`yaw_mode: fixed` |
| **匀速圆周** | `IN_pair2_circle` | `mode: circle`，`speed: 0.3`、`radius: 0.6`、`yaw_mode: center` |

共同限幅：`max_speed 0.4`、`max_climb 0.4`、`safety_max_alt 2.5`、`safety_min_alt 0.3`、
`safety_max_track_dist 1.2`。

> 🔴 **首飞一律先 `yaw_mode:=fixed` 覆盖掉场景里的 `center`**（同 §3.1 的理由）：
> 航向来源是刚恢复的板载罗盘，尚未在飞行中验证（§10 A1/A2）。`center`/`tangent` 会让飞机
> **主动转向**，航向一错就把误差直接放大成位置发散。**双机比单机更怕这个**——两机同时转错
> 方向会一起朝同一侧漂，`d_safe` 保护的是相对间距、拦不住整队跑偏。

**室外要放大时**，在 `config/scenarios.yaml` 里加 `OUT_pair2_*`，只改这几项：
`target_alt`（建议 −3 ~ −5 m）、`radius` / `max_distance`、`speed`、
`safety_max_alt` / `safety_max_track_dist`。
🔴 **同时要把飞控的 `GF_MAX_HOR_DIST` 从 0 改成实际场地半径**——现在围栏是关的。

**建议推进顺序**：`hover` → `line` → `circle`。圆周是三者里最吃 offboard 跟随性能的
（持续变向 + 论文里那个速度环滞后正是在圆周上最先暴露）。

---

## 5. 自主解锁：从手动首飞过渡到全自主编队

### 5.1 代码**能**自主解锁+起飞，默认关掉是安全策略不是技术限制

`mpc_node` 里早就实现了完整的自主进入：

```python
if self.auto_arm_enable:
    if self._nav_state != 14:
        self._send_vehicle_command(VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)      # 切 OFFBOARD
    if self._arming_state != 2:
        self._send_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)  # 解锁
```

每 2 s 重试直到 `nav_state=14 && arming_state=2` 确认。**SITL 的 41 场景回归全部是这么跑的。**

`real_hardware_launch.py` 把 `auto_arm` 默认设成 **false**，理由是：

1. **自主解锁的瞬间飞机就会起飞**，没有缓冲。航向错、GPS 掉到单点、场景参数写错——
   任何一项都会立刻变成动作，**没有"先看看再决定"的机会**。
2. **首飞阶段未知项太多**（罗盘刚恢复未标定、RTK 未接基站、双机从未飞过）。
   此时把"是否起飞"交给代码 = 把所有未验证项一次性押上。
3. 这个能力是真实有力的：SITL 默认 `auto_arm_enable=true`，而 SITL 的 drone0 与真机
   共用 `/fmu` 话题名 ⇒ **真机在线时跑 SITL，SITL 会真的去解锁真机**（已知风险，见
   `jetson-orin-nx-bench` 知识库条目）。

### 5.2 分阶段把控制权交给代码

研究目标本就是全自主编队，所以路径不是"永远手动"，而是分阶段移交：

| 阶段 | `auto_arm` | 解锁 | 切 OFFBOARD | 目的 |
|---|---|---|---|---|
| **① 拆桨台架** | `true` | 代码 | 代码 | 验证自动解锁链路本身跑得通（**电机不转，零风险**） |
| **② 装桨首飞** | `false` | 飞手 | 飞手 | 人在环，确认航向/定位/轨迹都对 |
| **③ 参数确认后** | `false` | 飞手 | 飞手 | 重复 ②，把 hover→line→circle 走完 |
| **④ 全自主** | `true` | 代码 | 代码 | 飞手只握 kill 开关待命 |

**阶段 ① 现在就能做**（拆桨 + `auto_arm:=true`）。它验证的正是"代码发的
`ARM`/`DO_SET_MODE` 飞控认不认"，把这条链路的问题在地面暴露掉，成本为零。

### 5.3 全自主时的额外注意

- **解锁前 PX4 预检必须全过**（GPS fix / EKF 收敛 / 罗盘 / 水平）。任一项不过，
  代码会一直重试 ARM 而飞控一直拒绝 —— 表现为**"卡住不动"**，要看飞控日志才知道原因，
  不要误判成代码故障。
- **代码已有一道保护**：`self._ocp_ready = False  # acados 编译完成前不尝试 ARM`。
  首次启动要编译数分钟，这期间不会误解锁。
- 🔴 **遥控器无论如何都要开着并绑定**。它是 §7.4 那层"最高优先级退出路径"，
  CH8 kill 是最后一道闸。**全自主 ≠ 不要遥控。**
- 🔴 **`COM_RCL_EXCEPT` 现阶段不要动**。它是让 PX4 在 offboard 下不因 RC 丢失触发失效保护的
  参数——将来完全无 RC 飞行才需要，现在动它等于**拆掉最后一道闸**。

---

## 6. 降落流程（三种，都已实现并验证）

### 6.1 第 1 层：任务完成自动降落 ✅

**机制**：leader 在运动开始 `mission_duration` 秒后向 `/swarm/mission` 广播 `LAND`（latched
QoS + 1 Hz 兜底重播）。各机收到后**各自独立**：零速悬停 `land_settle_s`（默认 1.5 s）把编队
速度刹掉 → 命令本机 PX4 切 `AUTO.LAND` → 停 offboard 流，交给 PX4 自主下降 + 触地自动上锁。

- **执行完全各机独立**，不依赖机间通信。
- **无碰撞是构造上的**：各机原地垂直下降，水平间距在飞行段已由 `d_safe` 拉开。
- SITL 1→9 机全部验证通过（全部触地自动上锁、垂直下降无碰撞）。

**用法**：leader 传 `mission_duration:=<秒>`（`0` = 禁用），或 `trigger_land:=true` 立即触发。

> 🔴 **计时基准不是「起飞」，是「leader 运动开始」——hover 场景下这两者差很远**
> （2026-08-20 复盘首飞时查出，尚未被实际咬到，因为那次 `mission_duration` 用的默认 0）。
> `leader_node.py:244` 对 **hover 模式直接跳过就绪门控**：`return t >= self._start_delay`，
> 也就是说 leader 节点起来 `start_delay` 秒后 `_motion_started` 就置位了，
> **跟飞机是否解锁、是否离地毫无关系**。
>
> 首飞实测的时间线：leader 节点 t=0 起、飞手 **t=271 s** 才切进 OFFBOARD。
> 若当时按本文档自己的建议传了 `mission_duration:=60`，LAND 会在 **t≈60 s 广播**，
> 即**飞机起飞前 211 秒**；而 `_land_triggered` 是 sticky 的、LAND 用 latched QoS 重播
> ⇒ 飞机一进 OFFBOARD 就会被立刻要求降落。
>
> **在改掉之前**，hover 场景要么把 `mission_duration` 留 0（靠飞手或 `trigger_land:=true`），
> 要么把它设成「预计从起节点到起飞的时间 + 想飞的时长」。line/circle 走的是真正的
> 就绪门控（各机 pos_err 收敛才开动），基准接近起飞时刻，问题小得多但仍偏早。

> ## ✅ 2026-08-21 已实飞验证并修正（hover / line / circle 三场景全部成功）
>
> **计时基准问题已修**：`mission_duration` 改用独立任务时钟，基准 = `_all_formed_up(require_alt=True)`
> 连续保持 `ready_hold`（即飞机真正到达目标高度），**刻意不给超时兜底**。实飞验证：
> hover 起表于飞机到位后、circle 起表于起 leader 后 **t=5.1 s**。
>
> ### 🔴 但同类问题在 `ready_timeout` 上又咬了一次（line 实飞）
> `_should_start_motion` 的超时兜底 `ready_timeout` **默认 90 s，同样从 leader 节点启动算起**。
> 而「起节点 → 飞手手动起飞 → 切 OFFBOARD」这套流程很容易超过 90 s。line 实飞就撞上了：
> ```
> [ERROR] readiness TIMEOUT at t=90.0s — not all drones formed up; starting motion anyway
> ```
> 此时**飞机还在地上**，leader 兜底触发、**把 3 m 直线跑完并冻结在终点**；飞手切进 OFFBOARD
> 时参考已在 3 m 外，飞机**直奔终点**，`poserr` 冲到 **3.007**，恰好顶满 `safety_max_track_dist=3.0`
> （再多一点就 RELINQUISH 交还控制权）。落点 3.00 m @ 51.6° 看着对，但**根本没走直线**。
>
> ### ✅ 两条修正，已落地
> **① 操作顺序：leader 最后起。**
> ```
> ① 起 drone 节点 + diag_monitor            ← 飞机在地面
> ② 飞手解锁、手动起飞 1–2 m、切 OFFBOARD    ← 飞机自己爬到目标高度并就地悬停
> ③ 这时才起 leader 节点                     ← ready_timeout 从此刻才开始算
> ④ 飞机已就位 ⇒ 数秒内满足就绪 ⇒ 正常开跑，兜底永远够不着
> ```
> circle 实飞用此顺序：起 leader 后 **t=5.1 s 就绪**，对比 line 那次的 t=90 s 兜底。
> 附带好处：`line_along_heading` 锁的是**飞机在空中稳住时的实际朝向**，比地面摆放更贴近真实机头方向。
> ⚠️ 第②步尽量**在起飞点正上方**切 OFFBOARD —— 飞机在哪悬停，起 leader 时那里就是轨迹起点。
>
> **② `ready_timeout` 现在可由 scenario 覆盖**（此前两个 launch 都没透传，现场想改都改不了）。
> `OUT_solo1_line` / `OUT_solo1_circle` 已设 `ready_timeout: 600.0` 作为第二道保险。
>
> ### 🔴 起飞期锁 XY（`takeoff_xy_lock_enable`）有 bug，真机一律显式关闭
> `_ground_z`（地面高度基准）不是在解锁边沿捕获，而是**拖到 mpc_node 控制回路第一次执行时**才捕获，
> 而该回路只在进 OFFBOARD 后才跑 ⇒ 按本文档推荐的「手动起飞后切 OFFBOARD」流程，
> **基准必然被记成空中值**（实飞记成 −2.85 m），解除条件（相对上升 0.5 m）永远满足不了
> ⇒ **XY 速度指令全程为零**，飞机只能随风漂（实飞漂到 2.9 m、`SAFETY DEGRADED`、任务时钟不起表）。
> **修好并在 SITL 复验前，真机一律传 `takeoff_xy_lock_enable:=false`。**
>
> ### 🔴 每次飞行前必须重建 EKF 原点（气压计漂移 0.12 m/min）
> 实测：飞机静置地面、RTK 海拔恒定不动，而 EKF `z` 以 **0.12 m/min** 单向漂移
> （与既有记录的「室外气压计漂移 0.152 m/min」同量级）。飞控启动 12 分钟后高度基准已偏 **1.0 m**
> ⇒ `target_alt=-3.0` 实际只飞到 **2.0 m**。
> ⇒ **原点不是「一次校准管一天」**，而是**每次飞行前**用不停桥重启重建（RTCM 不中断）。
> 实测重建后 `|xy|` 可到 1.7–5.7 cm。

> ⚠️ **2026-08-18 补的缺口**：`real_hardware_launch.py` 此前**没有透传** `mission_duration`
> （只有 SITL 的 `swarm_launch.py` 暴露了），所以第 1 层降落在真机上一直够不到，
> 只能靠第 2 层失联降落或飞手手动。现已补上，**用前需在机载电脑 `colcon build`**。

### 6.2 第 2 层：通信失联自动降落 ✅

**机制**（companion 侧看门狗）：OFFBOARD 飞行中超过 `comms_loss_land_s`（真机默认 **5.0 s**）
收不到 `/leader/state`，就触发第 1 层的降落流程。中间还有个"先冻结"阶段：leader 停
`comms_loss_hold_s`（默认 **1.0 s**）先就地悬停、不追陈旧参考。

**为什么必须有这一层**：飞控层的失效保护看的是"offboard 心跳还在不在"。如果
**leader 挂了但 mpc_node 还活着**，飞控视角一切正常、永不触发失效保护 ⇒ 飞机会照旧参考
悬停/飞行**直到电池耗尽**。这个看门狗补的正是这个洞。solo1 同样适用（solo1 也有 leader）。

> 约束：`comms_loss_hold_s` 必须 < `comms_loss_land_s`，否则冻结阶段自动禁用（会打日志）。

### 6.3 手动降落（飞手随时可用）

飞手把模式开关从 OFFBOARD 切走（Position / Altitude / Land）即可**立即夺回控制**。
mpc_node 检测到 `nav_state != 14` 会停止接管。**这始终是最高优先级的退出路径。**

---

## 7. 失控保护（四层，从里到外）

### 7.1 companion 侧安全滤波器（`safety_filter.py`）

在 MPC 速度指令下发**之前**统一过一道硬保护，纯函数式、无 ROS 依赖。五个缺口：

| 缺口 | 保护 |
|---|---|
| 飞散围栏 | 限 `|pos − ref|`（**偏离参考点**距离，不是距出生点——line/circle 会远离出生点但不该远离"它该在的位置"）+ 绝对高度。临界刹停、越界升级 |
| 硬碰撞地板 | 用**实测邻居位置**独立硬判（绕过 MPC 的软代价），太近 → 横向刹停 |
| 估计健康门 | 自身状态估计中途变脏 → 失效保护（不拿坏状态飞） |
| 失效状态机 | `NORMAL → DEGRADED → HOLD → RELINQUISH`（最终**交还 PX4 失效保护**） |
| leader 限跳 | 速度硬帽 + 转换率(jerk)限制，吸收参考巨跳 |

关键参数：`max_track_dist`、`max_alt`、`min_alt`(0.3)、`d_emergency`(真机建议 ~1.8)、
`escalate_frames`(15 帧=0.3 s 升一级)、`grace_frames`(150 帧=3 s，arm 后冷启动窗口，
期间只激活碰撞+NaN 检查)。

### 7.2 mpc_node 降级

任何异常（**求解失败 / 偏差 >5 m / NaN**）→ `_hover_setpoint_world()` 就地悬停。

### 7.3 飞控层失效保护（PX4）

| 参数 | A10 现值 | 室外建议 |
|---|---|---|
| `NAV_RCL_ACT` | **2 = RTL 返航** | ✅ 室外合适（室内是 Land） |
| `GF_MAX_HOR_DIST` | **0 = 围栏关闭** | 🔴 **必须设成场地半径** |
| `COM_ARM_WO_GPS` | **1 = 允许无 GPS 解锁** | 🔴 建议改 **0**，防 RTK 未锁定就起飞 |
| `COM_OF_LOSS_T` | ~1 s | offboard 心跳丢失判定时间 |

### 7.4 飞手 RC（最高优先级）

- 切模式 → 立即夺回控制
- **CH8 = kill 开关** → 立即停桨（`RC_MAP_KILL_SW=8`）
- ⚠️ **装桨前必测 kill 通不通**

---

## 8. 实验数据记录（三条链，飞前想好，飞后补不了）

真机飞行有三份可记录的数据，**用途不同、缺一不可**：

| 数据 | 内容 | 频率 | 用途 |
|---|---|---|---|
| **① `diag_monitor` CSV** | 各机位置/误差/间距/solve_ms/降级次数 + leader 参考 | **1 Hz** | 复盘、体检报告、俯视轨迹图 |
| **② PX4 `.ulg`** | 飞控内部全量（EKF/传感器/指令/执行器） | 高频 | 事故分析、**τ 辨识** |
| **③ rosbag** | ROS2 话题原始流 | 话题原生 | offboard 指令链复现、τ 辨识 |

### 8.1 ① diag_monitor CSV —— 复盘主力

```bash
# 在任一台机载电脑（或地面 PC，只要同 DDS 域）另开一个终端
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

python3 ~/Multi-UAV-simulation/diag_monitor.py --formation pair2 --log
# 省略路径 = 自动命名 flight_<formation>_<时间戳>.csv
# 或指定：--log ~/flights/20260819_pair2_hover.csv
```

**列结构**（每机一组，纯增列、向后兼容）：
`t` / `d{i}_x,y,z,zerr,velxy,arm,nav,mpc,solve_ms,fallback,hover,poserr,planratio,clipn,dhat{x,y,z}`
/ `min_spacing, formation_max_err, safety_violations, total_fallbacks, max_solve_ms,
leader_x, leader_y, leader_vx, leader_vy`

🔴 **真机三个必须注意的点**

1. **`--formation` 必须与 launch 用的队形一致**。它决定记几架机的列、以及出生点表。
   写错会静默记错列数（SITL 的 grid9 就这么被记成 5 机，整组数据作废）。
2. **`--shared-origin` 在 GPS 下不要加**。该开关是给**动捕/全场共享原点**用的
   （local 已是世界坐标、不再叠出生点）。GPS 下各机 EKF 原点独立，加了会算错。
   ⚠️ 反过来漏加也危险：动捕下漏加会把间距算成 2 倍且**数值很稳定、看着像真的**
   （实测 pair2_in 间距被算成 5.600 m，真值 2.800 m）。
3. **1 Hz 太粗，不能用于 τ 辨识**。τ≈0.5 s 的时间常数需要 ≥50 Hz，见 §8.3。

**跑完出体检报告**：
```bash
python3 ~/Multi-UAV-simulation/analyze_flight.py <csv> --plot
# 输出 pos_err/高度误差、最小间距+时刻、违规、solve 耗时、编队成型时间 + 俯视轨迹图
```

### 8.2 ② PX4 `.ulg` 日志

`SDLOG_MODE=0`（当前值）= **解锁后开始记、上锁停止**，正是飞行段，够用。
`SDLOG_PROFILE=1` = 默认剖面。

- 🔴 **飞前确认 SD 卡在飞控里且有空间**——没卡就没日志，事后无法补。
- 导出：飞控 USB 直连电脑，或 QGC 的 Log Download。
- 历史 26 个架次存在 `交接/飞行日志/`。

### 8.3 ③ rosbag —— **τ 辨识专用，论文要的就是这个**

论文当前唯一硬伤是 SITL-only。τ 辨识需要的就是**指令速度 vs 实际速度**的高频对照，
而室外 offboard 本来就要飞，**加录这一项边际成本≈0**。

```bash
# 与 launch 同时起，在机载电脑上
ros2 bag record -o ~/flights/tau_$(date +%Y%m%d_%H%M%S) \
    /fmu/in/trajectory_setpoint \
    /fmu/out/vehicle_local_position \
    /fmu/out/vehicle_attitude \
    /fmu/out/vehicle_status_v1
```

- `/fmu/in/trajectory_setpoint` = **我们下发的速度指令**（`velocity` 字段）
- `/fmu/out/vehicle_local_position` = **实际实现的速度**（`vx/vy/vz`）
- 两者做一阶滞后拟合 `v̇=(v_cmd−v)/τ` 即得 τ，方法与 `report/tau_goodness.py` 一致

🔴 **注意**：`.ulg` 里其实也有这两路（更高频），**两条都录**——rosbag 便于用现成脚本处理，
`.ulg` 是高频兜底。

**采集建议**：τ 辨识最好在**静风、直线或悬停加阶跃**的段落做。论文用的是静风窗口拟合、
风扰段不再辨识（风扰下发散会污染辨识，自由拟合会病态）。

### 8.4 归档约定

飞完立刻把三份数据归档，**别留在机载电脑上**（Jetson 非 git 仓库、断电有风险）：

```
flights-仓库 或 交接/飞行日志/
  20260819_A10_solo1_hover/
    flight_solo1_20260819_143012.csv     ← diag_monitor
    log_XX_2026-8-19-14-30-00.ulg        ← PX4
    tau_20260819_143012/                 ← rosbag
    NOTES.md   ← 🔴 手写：风况、场地、fix_type、改过什么参数、主观感受
```

**`NOTES.md` 是最容易漏又最值钱的一份**——参数和风况事后回忆不出来，
而论文/复盘要的正是"这次和上次差在哪"。

---

## 9. A08 切 RTK 参数集（若要用 A08）

A08 现在是**室内动捕**配置。切换前**先存档**（当前存档：
`交接/真机参数与部署/A08_室内动捕_20260818.params`，1112 项全量，QGC 可直接加载）。

| 参数 | 动捕（现值） | 室外 RTK |
|---|---|---|
| `EKF2_GPS_CTRL` | 0（禁） | 7（位置+速度+高度） |
| `EKF2_HGT_REF` | 0 | 1（GPS，RTK 垂直 cm） |
| `EKF2_EV_CTRL` | 11 | 0（关外部视觉） |
| `EKF2_BARO_CTRL` | 1 | 1（可保持，GPS 主 / baro 辅） |
| `COM_ARM_WO_GPS` | 1 | **0** |
| `GF_MAX_HOR_DIST` | 2（室内） | 场地半径 |

工具：`tools/set_param.py`（单参数写 + 回读校验）、`tools/dump_params2.py`（存档）、
`tools/fc_reboot.py`。用法见 `tools/README_飞控工具.md`。

---

## 10. 风险复审（研究员视角，按"能不能炸机 / 会不会让实验白做"排序）

> 2026-08-18 对全套室外方案做的独立复审。**A 级三条建议在装桨前全部处理掉**——
> 都不需要起飞，半小时能做完，但能挡住最可能的炸机方式。

### 🔴 A 级：可能直接失控

#### A1. 板子朝向 —— 🟢 **2026-08-19 降级：既有证据其实已经足够**

> ⚠️ **本节原先的论证是错的，保留原文见下方引用块，避免下一个接手的人重蹈。**

**结论**：`SENS_BOARD_ROT=0` 的正确性**已由 Stabilized 的飞行表现证明**，不需要额外验证。

推理：`SENS_BOARD_ROT` 是**板级**旋转，同时作用于 IMU 与板载磁罗盘。Stabilized 虽不使用绝对
航向，**却直接检验 roll/pitch 轴**——打杆向右若飞机向右滚转，就说明 IMU 的 X/Y 轴与机架对齐；
板子若相对机架转了 90°，打右滚会变成俯仰，飞手第一时间就会发现。故：

```
Stabilized 飞得跟手 ⇒ IMU X/Y 轴与机架对齐 ⇒ SENS_BOARD_ROT=0 正确
                   ⇒ 同一块板上的内置磁罗盘随之对齐 ⇒ 航向轴向无问题
```
（这与知识库 `uav-maiden-manual-flight` 的既有结论一致：「箭头朝左但飞正常 = **IMU 已对齐**」。）

> **原论证（已废，错在偷换范围）**：~~"那个'飞得好'是在 Stabilized/Altitude 下得到的——这两个
> 模式不使用绝对航向，所以不能证明板子朝向正确"~~。该说法对 **yaw 本身**成立（偏航确是陀螺积分
> 的相对量），**但板子朝向错误会同时污染 roll/pitch，而那恰恰是 Stabilized 检验得最充分的**。

**唯一残留的可查项（几秒钟，不需物理操作）**：读 **`CAL_MAG0_ROT`**。板级旋转管的是"板子相对
机架"，而磁罗盘另有自己的旋转参数；内置罗盘正常应为 **`-1`（跟随板级旋转）**。若被设成别的值，
磁罗盘与 IMU 会各转各的 —— **这是唯一能造成"roll/pitch 对、航向偏"的机制**。

⚠️ **别用这条去覆盖 A2**：板子装得再正，也挡不住电流干扰（见下）。两者性质完全不同。

#### A2. 板载罗盘 + **零电流补偿** = 油门相关航向漂移

`CAL_MAG0_XCOMP / YCOMP / ZCOMP` 全为 **0**（无电流补偿），而这是**飞控板载**罗盘——
位置紧邻电源线与电调，是磁力计能放的**最差位置**。业界把罗盘装在 GPS 支架上远离电流，
正是为了躲这个。

失效方式隐蔽：**静止时航向正常，一推油门电流上来航向就偏**，偏移随油门变化，
**地面静态测试永远发现不了**。

**验证（拆桨、机身固定）**：QGC 看航向读数 → actuator test 把电机拉到 50% / 80% →
看航向漂多少。原判据：**漂 >10° 就不能靠它做位置控制**。

> ## ✅ 2026-08-21 已实测 —— 结论是「效应确实存在，但本测试给不出能飞/不能飞」
>
> 工具 `tools/a2_mag_test.py`（**只读，不发任何指令**；电机由飞手在 QGC 控制）。
> 实测（A10，室外，RTK-Fixed）：
> ```
> 干净基线 30-120s   航向 44.112°   峰峰 0.065°     ← 极稳
> 电机段1 121-144s   电流 1.3-4.6A   航向 -5.91°(中位) / -7.31°(极值)
> 电机段2 157-174s   电流 1.2-5.9A   航向 -4.28°(中位) / -6.12°(极值)
> 关机后恢复到 45.55°
> ```
> **两段独立复现、方向一致、关机后恢复 ⇒ 真的电流相关航向偏移，不是噪声。**
>
> ### 🔴 但原来那个 >10° 的二值判据，在拆桨条件下根本触发不了
> 1. 拆桨电流只有 **2.5–5.9 A**，而悬停要 **20–30 A**；
> 2. **更要命：拆桨时电流几乎不随油门变化** —— 实测推 50% 与 80%，电流都是 2.5–2.6 A。
>    props-off 电流由摩擦/风阻决定，**根本不是飞行电流的代理**；
> 3. 因此对测得的窄带（2.4–5.9 A）做线性回归再外推到 20–40 A **不可信**（是拿噪声外推），
>    本次刻意不采信脚本给出的外推值。
>
> ⇒ **修订判据**：本测试只能证明「效应是否存在」，**不能**用来放行。
>   测到偏移即说明板载罗盘受电流影响；能否飞要靠下面的证据链判断。
>
> ### 🔑 反向证据（分量不轻）
> **2026-08-20 首飞就是用这个罗盘飞的**：offboard 悬停 125 s，水平误差均值 **1.8 cm**、
> 最大 5.8 cm。若悬停电流下航向偏了几十度，位置环虽仍靠 GPS 闭合，但会出现明显的
> 画圈/漂移特征 —— **实测没有**。⇒ 效应存在，但悬停电流下的量级大概率仍可接受。
>
> ### 🔴 测试本身的一个前置条件（v1 因此报废过一次）
> **EKF 航向收敛期的漂移实测约 11°（33°→44°，耗时约 90 s），本身就超过 10° 判据。**
> 在收敛完成前推电机，电流效应与收敛漂移完全混在一起、无法解读。
> ⇒ `a2_mag_test.py` v2 会先等航向稳定（10 s 峰峰 <0.15°）才提示可以推电机；
>   **任何 A2 测试都必须先确认航向已稳定**。
>
> ### 后续选项（按代价，未决）
> | | 做法 | 代价 | 得到什么 |
> |---|---|---|---|
> | A | 接受现状按 `hover`→`line`→`circle` 飞 | 零 | 首飞已证明够用；`yaw_mode=fixed` 不主动转向，风险最小 |
> | B | 系留测试（装桨、飞机绑死、推到悬停油门） | 中，**有风险** | 唯一能在飞行电流下测到真值的办法 |
> | C | 加外置罗盘（装 GPS 支架上远离电流） | 低 | 一劳永逸，业界标准做法 |
>
> 建议 **A + C**：今天正常飞，同时把外置罗盘列入采购 —— `circle` 最吃航向
> （切向速度指令要靠航向转到世界系），双机编队对航向更敏感。
> **B 不建议**：系留失败的后果远大于测试收益。


> 🔴 **A2 必须先做，因为它决定要不要花钱**（2026-08-19 查证后重排）：
> | 顺序 | 方案 | 成本 |
> |---|---|---|
> | ① | **先做本测试**（拆桨 actuator test） | 零。**漂 <10° ⇒ 现有板载罗盘够用，下面两条都不用做** |
> | ② | 外置罗盘（装 GPS 支架上，远离电流） | 低 |
> | ③ | GPS 航向（moving baseline） | **高** |
>
> ⚠️ **关于 ③ 的一个常见误解**：**C-RTK 9Ps 是单天线模块**（每模块仅一个 MMCX 口）。
> PX4 官方文档原文 *"It also supports RTK GPS Heading using **dual modules**"* ——
> 要 GPS 航向必须**再买一整个 C-RTK 9Ps 模块**，不是"补一根天线"；还要接 GPS2 口、
> 在机架上拉开足够基线距离固定两根天线、配 moving-baseline（`GPS_2_CONFIG` / `EKF2_GPS_YAW`）。
> 参考：<https://docs.px4.io/main/en/gps_compass/rtk_gps_cuav_c-rtk-9ps.html>

#### A3. `COM_LOW_BAT_ACT = 0` —— 低电量**只警告、不动作**

室内 7×8.5 m 随时能接手，室外不行。配合 `BAT1_N_CELLS=4` 与**尚未标定的 One PMU**，
意味着电池耗尽时**没有任何自动保护**。室外建议至少设 RTL 或 Land。
（历史决定是"等 One PMU 标定后再配"，室外飞之前必须了结这件事。）

### 🟠 B 级：会让实验白做

#### B1. RTCM 差分链路未定 —— 现在**不是真 RTK**

无基站差分时 9PS 只是单点定位，**CEP ≈1.5 m**（规格书值）。而 `d_safe=2.5 m`、编队间距 3 m
—— 1.5 m 的位置误差意味着两机"真实间距"可能只剩 1.5 m，**避碰约束建立在错误的位置估计上**。

判据：起飞前看 `fix_type`。**3 = 3D（单点）不能飞编队**，要 **5 (Float)** 或 **6 (Fixed)**。
RTCM 怎么进飞控（基站 → 地面站 → 数传 → FC）**是双机上天前必须解决的**。

#### B2. 顺带把论文缺的 τ 辨识数据采了（性价比最高）

论文目前唯一硬伤是 SITL-only。而 τ 辨识只需要：单机 offboard 发速度指令、同时记录
**指令速度 vs 实际速度**。室外飞 offboard 本来就要做，**加这项采集边际成本≈0**，
却能把 §4 那段从"论证仿真的合理性"变成"有真机 τ 支撑"。

准备：确认 PX4 日志在记（`SDLOG_MODE=0` = 解锁后记录，够用），并在机载电脑同时录 rosbag
（`/fmu/in/trajectory_setpoint` + `/fmu/out/vehicle_local_position`）。
**飞前想好，飞后补不了。**

#### B3. 高度参考会跳

A10 是 `EKF2_HGT_REF=1`（GPS 高度）。RTK **Fixed** 时垂直 cm 级，但掉到 **Float / 3D** 时
垂直误差瞬间变几米 ⇒ 飞行中 fix 质量变化会造成**高度估计跳变**，飞机会突然爬升/下降去"修正"。
要么确保全程 Fixed，要么考虑气压计为主。

### 🟡 C 级：值得注意

- **水平校准可能已作废**：`SENS_BOARD_X_OFF≈3.2° / Y_OFF≈0.7°` 是**装机前单板状态**标的。
  装了 RTK 模块、改了走线后建议重新校平地平线——3° 静差会让飞机朝固定方向持续漂。
- **`MPC_XY_VEL_MAX = 12` m/s**：companion 侧 `conservative` 把 MPC 输出限到 1.5，
  但**飞手切回 Position 模式接管时 12 m/s 是生效的**。场地够大问题不大，心里要有数。
- **真实湍流 ≠ Gazebo 恒定侧向力**：论文 §4 已如实声明这条边界。真机数据若用于对照，
  要注意这个差异。
- **EKF 收敛时间**：9PS 规格 RTK 收敛 <60 s。上电后别急着解锁，等 `fix_type` 稳定在 6。

### ✅ 若只做三件事

**A2 油门磁干扰测试 → 读 `CAL_MAG0_ROT`（应为 −1）→ B1 确认 RTK Fixed。**
前两项不用起飞、不用装桨。
（原列的"A1 航向朝向验证"已于 2026-08-19 降级——Stabilized 的飞行表现已证明板子朝向正确，见 §10 A1。）

---

## 11. 已知未验证项（诚实清单）

| 项 | 状态 |
|---|---|
| **两机 uXRCE 话题命名空间冲突** | 🔴 **双机飞行的硬阻塞，2026-08-19 实测发现**，详见下方 |
| `xy_global_align_enable`（GPS 编队全局对齐） | 🔴 **从未在 SITL 验证通过**。单机不需要，编队才需要 |
| FastDDS mesh 单播发现 profile | 🔴 起草了未验证（`deploy/mesh_fastdds_profile.xml`） |
| 室外磁罗盘标定 | 🔴 罗盘刚恢复，未标定 |
| 双机 offboard | 从未做过 |
| RTCM 差分注入链路 | 未定（基站 → 地面站 → 数传 → 飞控） |

### 🔴 `MAV_SYS_ID` 必须逐机不同（2026-08-19 现场查出的既有配置错误，已修）

**代码早就按 `drone_id + 1` 定址**（`mpc_control/mpc_node.py:1407`、`arming_node.py:108`）：
```python
msg.target_system = self.drone_id + 1     # drone0 → 1, drone1 → 2
```
而实测**两架飞控的 `MAV_SYS_ID` 都是 1**。PX4 会**丢弃 `target_system` 不匹配的
`VehicleCommand`** ⇒ **A08(drone1) 的 ARM / 切 OFFBOARD 指令根本到不了飞控，永远解不了锁**，
而且不会报错、只表现为"一直 retry"。

**已修**：A08 `MAV_SYS_ID` 1→2（写入并回读校验，重启飞控后心跳 sysid 确认为 2）。A10 保持 1。
⚠️ 新增机器时务必同步设置：**drone_id N 的飞控 `MAV_SYS_ID` 必须 = N+1**。

### ✅ 多机同时显示在一个 QGC 里（走 LQ mesh，不需要 3DR）

前提：①各机 `MAV_SYS_ID` 不同（见上）；②每台 Jetson 各跑一份 `tools/mav_relay.py`，
都指向同一个地面站 IP。QGC 在 14550 上按 sysid 自动分辨出多架飞机。

```bash
# A10（飞控 10.41.10.2）
MAV_RELAY_FC=10.41.10.2   setsid nohup python3 ~/Multi-UAV-simulation/tools/mav_relay.py 192.168.1.123 >/tmp/mav_relay.log 2>&1 &
# A08（飞控 192.168.77.2）
MAV_RELAY_FC=192.168.77.2 setsid nohup python3 ~/Multi-UAV-simulation/tools/mav_relay.py 192.168.1.123 >/tmp/mav_relay.log 2>&1 &
```
🔴 `MAV_RELAY_FC` 是 2026-08-19 新加的环境变量——**原脚本把飞控 IP 写死成 `192.168.77.2`（A08 的）**，
在 A10 上会连不到飞控且**不报错、只是静默无数据**。

**排障**：桥的日志每 2000 包打印 `fwd FC->gcs=...`，有增长就说明桥这侧正常。
若地面站仍收不到，查 **Windows 防火墙**——裸 socket 监听会被拦（实测收 0 包），
而 QGC 自带入站放行规则（需确认规则里的 exe 路径与实际启动的一致、且网络类别与规则的
Profile 匹配；本机 `以太网`=Public、规则也是 Public，故可用）。
⚠️ 桥占用 Jetson 的 14550，**用 pymavlink 读写参数前要先停桥**，否则端口被占。
⚠️ 停桥别在同一条 ssh 里 `pkill -f mav_relay`——整段脚本会被当成单个参数传给远端 shell，
cmdline 命中自身 pattern **把自己杀掉**（本次实测复现）。拆成独立 ssh 调用。

### 🔴 两机 uXRCE 话题命名空间冲突（2026-08-19 现场实测，双机硬阻塞）

**现象**：两台飞控的 `MAV_SYS_ID` / `UXRCE_DDS_KEY` / `UXRCE_DDS_DOM_ID` 全是 `1/1/0`，
两台 Jetson 的 Agent 都是 `MicroXRCEAgent udp4 -p 8888`（**无命名空间参数**）
⇒ **两架飞控的话题都落在 `/fmu/...` 上**（各 51 个，`/px4_*` 为 0）。
XRCE 那一段本身是隔离的（各 Agent 只连本机飞控，走各自的 `10.41.10.x` / `192.168.77.x`），
**冲突发生在 ROS2/DDS 层**：两个 Agent 在同一个 domain 0、同一个 mesh 上建了同名话题。

**后果**（`mpc_control/mpc_node.py:95` 的 `topic_for_drone` 用的是**绝对话题名**，
launch 的节点 namespace 不影响话题名）：
| | 后果 |
|---|---|
| A08 的 mpc_node(drone1) | 找 `/px4_1/fmu/...`，不存在 ⇒ 一直等待不解锁（**失效在安全侧**） |
| A10 的 mpc_node(drone0) | 订阅 `/fmu/out/vehicle_local_position`，**两个 Agent 都在发** ⇒ 收到两架交替的位置 |
| **A10 发的指令** | `/fmu/in/vehicle_command`、`/fmu/in/trajectory_setpoint` **两架飞控都收** ⇒ **一条 ARM 解锁两架、一条速度指令驱动两架** |

**判定命令**：`ros2 topic info /fmu/out/vehicle_local_position` 看 **Publisher count**。
`1` = 正常；`2` = 两架撞在一起。

**临时缓解（单机作业时）**：`ssh A08 "sudo systemctl stop microxrce-agent"`，
停掉不用的那台，Publisher count 回到 1。

**正式修法（双机飞行前必须做完，未做）**：飞控侧没有 `UXRCE_DDS_NS` 参数（已实测确认不存在），
只能按本文 §3 顶部注释的方式在飞控启动客户端时带 `-n`——改 A08 的 SD 卡 `/fs/microsd/etc/extras.txt`：
```
uxrce_dds_client stop
uxrce_dds_client start -t udp4 -p 8888 -h 192.168.77.100 -n px4_1
```
改完 A08 话题变成 `/px4_1/fmu/...`，与 `topic_for_drone(1, ...)` 对上。
⚠️ 这也是记忆里那条「真机在线时跑 SITL，SITL 会真的去解锁真机」的**同一个根因**。

---

## 12. 快速排障

| 症状 | 先查 |
|---|---|
| **QGC「无法检索参数集」/ 概况整页打不开** | **不是故障**，见 §12.1。飞行界面不受影响 |
| 卡在 "retry ARM+OFFBOARD" | `px4_version` 是否 1.16（话题名 `_v1` 后缀）；`ros2 topic hz /fmu/out/vehicle_status_v1` |
| 切不进 OFFBOARD | `local_position_invalid`（多半没 GPS fix）；`cs_yaw_align` |
| 一个 fmu 话题都看不到 | 飞控 `UXRCE_DDS_DOM_ID` 是否为 0（只设 companion 侧无效，静默失效） |
| Agent 冲突 | 是否漏了 `start_agent:=false` |
| `calib STUCK` 红字 | 摆放误差 > 2 m，不是故障是保护 |
| 编队不开动 | leader 就绪门控没过（`pos_err`/`ready_hold`），看 leader 日志 |

### 12.1 QGC「无法检索参数集」/ 概况整页打不开 —— **不是故障**（2026-08-20 定位）

**根因**：QGC 要求**所有 MAVLink 组件**的参数都凑齐才放行 Vehicle Setup。而飞控上有两个组件：
- `(N, 1)` 飞控本体 —— **1127/1127 全齐** ✅
- `(N, 125)` **One PMU 的 DroneCAN 节点** —— 只回 10~13/25 ❌

PX4 的 DroneCAN 参数桥一次只向 CAN 节点转发一个请求，QGC 连敲 75 次大部分被丢弃
（实测缺失 index 呈 `3,5,6,7,8,10,11,...` 的稀疏分布 ≈ 三个请求才回一个）。
节点 125 的身份由**参数名**坐实：`BATT_FUL_VOLTAGE`/`BATT_MAX_AMPS`/`CAN_NODE`/
`NTF_LED_BRIGHT`/`NTF_BUZZ_ON_LVL` —— **全是电源模块自己的配置，没有一个是飞行参数**。

**✅ 飞行不受影响**：`UAVCAN=2` 下 **QGC 飞行界面完全正常**（实测移动飞机数据随动）——
姿态 / 高度 / GPS 星数 / 电池电压 / 模式全都有。**被挡的只有台架才用的 Setup 与参数编辑页。**

**⚖️ 取舍（两者不可兼得）**：
| | `UAVCAN_ENABLE=2` | `UAVCAN_ENABLE=0` |
|---|---|---|
| 电池遥测 + `COM_LOW_BAT_ACT`(Land) 保护 | ✅ | ❌ **失去数据源** |
| QGC 参数页 / 概况 / Setup | ❌ 被挡 | ✅ |

**🔴 定案：飞行时保持 `UAVCAN_ENABLE=2`。** 电池保护是飞行中的安全功能；QGC Setup 是台架便利，
而改参数本就该用 `tools/fc_configure.py`（dry-run + 自动备份 + 写后回读校验，比 QGC 手改更安全）。

**需要用 QGC 做传感器校准时**，临时切换：
```bash
python3 /tmp/setint.py UAVCAN_ENABLE 0   # 或 QGC 里改
python3 /tmp/fcreboot.py                 # 重启飞控 → QGC Setup 立刻可用
#   … 在 QGC 里做校准 …
python3 /tmp/setint.py UAVCAN_ENABLE 2
python3 /tmp/fcreboot.py                 # 🔴 必做：恢复电池遥测
```
🔴 **改回 2 之后必须验证电池电压重新出现**（应为 15.x V / 4S），别只看参数值。
📎 同类先例：校准电调时也要「先 `UAVCAN_SUB_BAT=0`+重启绕过 PX4"配了电池拒校准"、校完改回 1」。

> 💡 **排查这类问题的通用手法**：`Documents\QGroundControl\Telemetry\*.tlog` 里记着每一个
> MAVLink 包。用 pymavlink 按 `(sysid, compid)` 统计 `PARAM_VALUE` 的唯一 index 数与
> `param_count`，**"谁没凑齐"一目了然**——比任何猜测都快。本次正是靠这一步定位的，
> 在此之前的三个假设（飞控不响应 `PARAM_REQUEST_LIST` / 广播双份包压垮 mesh /
> QGC 走了电台串口透传）全部被数据否掉。辅助工具见 `tools/param_download_test.py`。

---

## 13. 提高实验成功率（按投入产出比排）

> 外场实验最大的成本不是飞坏，是**跑一趟什么都没得到**。下面按"每小时准备换回多少
> 有效数据"排序。

### 13.1 把"能不能飞"和"飞得好不好"分开验

**外场时间最贵，别用来查配置错误。** 配置类问题（话题名、参数、队形名、定位不满足）
**全部能在台架暴露**，成本为零：

- `tools/sensor_check.py` —— 传感器在位
- `tools/uplink_test.py` —— 上行通路（不解锁）
- 拆桨跑完整 launch —— 参数/队形/acados 编译
- `auto_arm:=true` 拆桨 —— 自动解锁链路（§5.2 阶段①）

**到了场地只解决"飞得好不好"**：定位精度、航向、跟随性能、风。

### 13.2 一次只改一个变量

每架次只动一项（轨迹 / 高度 / 速度 / 参数）。同时改两项，出了问题**无法归因**，
这一趟就白跑。历史教训：§10 的 A1（`SENS_BOARD_ROT`）之所以到今天还是未知项，
正是因为当初"飞得好"和"参数对"被混在一起判断了。

### 13.3 每次都留可复现的记录

- 三份数据（§8）**每次都录**，不要"这次先试试不录了"——出问题的那次往往就是没录的那次。
- **`NOTES.md` 必写**：风况、场地、`fix_type`、改过什么、主观感受。
  参数和风况事后回忆不出来，而复盘要的正是"这次和上次差在哪"。
- 每架次的场景名/参数**原样贴进 NOTES**，别写"用的默认值"。

### 13.4 先建立"已知好"的基线，再往外走

推进顺序固定：**单机 hover → 单机 line → 单机 circle → 双机 hover → 双机 line → 双机 circle**。
每上一档，**先复现上一档**确认没退化。一旦某档失败，回到上一个已知好的状态，
而不是继续往前试。

### 13.5 起飞前的固定检查清单（每架次都走）

```
[ ] fix_type ≥ 5（Float）/ 6（Fixed）   ← 单点 3D 不飞编队（§10 B1）
[ ] cs_yaw_align = true, cs_mag_fault = false
[ ] xy_valid / xy_global = true
[ ] ros2 topic hz /fmu/out/vehicle_status_v1 ≈ 2 Hz
[ ] 电池电压 / 剩余容量
[ ] SD 卡在飞控里（否则没 .ulg）
[ ] diag_monitor --formation <与 launch 一致> 已起
[ ] rosbag 已起（要 τ 数据时）
[ ] 遥控器开机、已绑定、kill 开关测过
[ ] 场地无人、无障碍、风况记录
```

**做成纸质或手机清单，逐项打勾**。外场容易漏，而漏的那一项往往就是失败原因。

### 13.6 风况与时段

- 论文的风扰实验是 Gazebo 恒定侧向力；**真实湍流是变化的**，同一架次前后风况可能不同。
- **记录风速风向**（手持风速计最好，没有就记天气预报 + 主观描述）。
- 尽量选**风小且稳定的时段**（清晨通常最稳）。做对照实验时，风况变化会直接污染结论。

### 13.7 备件与恢复能力

- **备用电池**：外场最常见的中断原因是电量耗尽，而不是故障。
- **备用桨**：便宜、易损、换起来快。
- **能在场地改参数**：带上能连飞控的笔记本（`tools/set_param.py` 走以太网，不需要 QGC）。
  发现参数不对却改不了 = 白跑一趟。
- **Jetson 关机纪律**：`sudo shutdown -h now`，别硬断电（NVMe 有损坏风险，
  上面是 acados 全套 + ROS2 工作区，重装代价很大）。

### 13.8 两人以上

- **飞手专注握遥控**，不看屏幕、不操作电脑。
- **另一人跑命令、读遥测、记 NOTES**。
- 单人同时做这两件事，是外场事故的常见成因。

### 13.9 论文导向的额外提醒

如果这趟飞行要为论文取数（§8.3 的 τ 辨识）：

- **静风段**做辨识，风扰段不辨识（发散会污染拟合、自由拟合病态——论文就是这么做的）。
- 需要**速度指令有变化**才能辨识出时间常数：纯悬停信息量不足，
  **直线往返或阶跃**更好。
- **多来几个架次**。论文里 SITL 用的是 n≥5 重复统计；真机哪怕只有 2–3 个架次，
  也比单次可信得多。
- 记录当次的 `MPC_XY_VEL_P_ACC`（现在两机都是默认 1.8）——它是 τ 的直接决定因素。

---

**相关**：`tools/README_飞控工具.md`、`report/真机安全参数配置单_QGC.md`、
`report/自动降落第1层_SITL验证.md`、`report/失联降落第2层_SITL验证.md`、
`交接/硬件安装与接口对应.md`。
