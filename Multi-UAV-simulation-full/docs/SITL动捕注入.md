# SITL 动捕位姿注入（打通记录，2026-07-23）

在没有真动捕硬件的情况下，用 Gazebo 真值冒充动捕，把飞场那条位姿注入链路
在 SITL 上完整跑通，提前暴露坐标系/EKF2 配置/失效行为的坑。

## 1. 架构

```
【飞场】 动捕软件 → VRPN → vrpn_mocap 客户端 ┐
                                            ├→ /vrpn_mocap/<刚体>/pose (PoseStamped, ENU)
【SITL】 Gazebo 真值 → sitl_mocap_source_node ┘
                                                        ↓
                                            mocap_bridge_node   ← **两种场景共用同一份代码**
                                                        ↓
                                     /fmu/in/vehicle_visual_odometry (VehicleOdometry, NED)
                                                        ↓ uXRCE-DDS
                                                  PX4 EKF2（EV 融合）
```

飞场与 SITL 的**唯一差别是数据源**：真机跑 `vrpn_mocap` 客户端，SITL 跑
`sitl_mocap_source_node`。bridge 及其下游完全相同，因此 SITL 里验过的东西
在飞场上是同一条代码路径。

| 文件 | 作用 | 真机是否用 |
|---|---|---|
| `mpc_control/frame_convert.py` | ENU/FLU ↔ NED/FRD 纯数学（无 ROS 依赖） | ✅ |
| `mpc_control/mocap_bridge_node.py` | PoseStamped → VehicleOdometry | ✅ |
| `mpc_control/sitl_mocap_source_node.py` | Gazebo 真值 → PoseStamped（假 VRPN） | ❌ 仅 SITL |
| `tools/test_frame_convert.py` | 坐标变换单元自检（任何机器可跑） | ✅ 改代码后必跑 |
| `tools/verify_mocap_injection.py` | 链路三点同步核验 | ✅（真机只能查后半段） |

## 2. PX4 侧改动（SITL 专用）

真值话题默认不导出到 ROS2，需给 `dds_topics.yaml` 的 `publications:` 段加两条：

```yaml
  - topic: /fmu/out/vehicle_local_position_groundtruth
    type: px4_msgs::msg::VehicleLocalPosition

  - topic: /fmu/out/vehicle_attitude_groundtruth
    type: px4_msgs::msg::VehicleAttitude
```

两条都是**现有消息类型的话题别名**（PX4 用 `# TOPICS` 机制），`px4_msgs` 无需改动。
改完 `make px4_sitl_default` 增量重编（实测仅数秒，只重生成 uxrce_dds_client）。

真值由 `src/modules/simulation/gz_bridge/GZBridge.cpp` 发布，取自 **Gazebo 物理状态、
不经 EKF**，所以用它验证 EKF 不构成循环论证。

⚠️ **真值是 Gazebo 世界系**（`pose_position` 是模型世界位姿，所有机共享一个原点），
与飞场动捕形态一致，而非"各机自己的出生点系"。

## 3. 运行

```bash
# 1) gz + PX4 + Agent（gz sim 必须带 -s，否则无 DISPLAY 下会 abort 并被静默换成替身世界）
gz sim -s -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf &
FORMATION=solo1 bash start_1_px4.sh &
MicroXRCEAgent udp4 -p 8888 &

# 2) 配 EKF2（SITL 的 MAVLink 在 udp:14540，fc_configure.py 同样可用）
#    ⚠️ SITL 参数会持久化到 build/px4_sitl_default/rootfs/<N>/parameters.bson，
#       41 场景回归用的是同一套 —— **测完必须还原**（见 §6）
cp ~/PX4-Autopilot-1.14/build/px4_sitl_default/rootfs/0/parameters.bson /tmp/params_before.bson
python3 tools/fc_configure.py -d udp:127.0.0.1:14540 -g ekf2_mocap --apply
# 改完重启 PX4 让 EKF2 重新初始化

# 3) 注入链路
ros2 run mpc_control sitl_mocap_source_node --ros-args -p drone_id:=0 -p rigid_body:=drone0 &
ros2 run mpc_control mocap_bridge_node      --ros-args -p drone_id:=0 -p rigid_body:=drone0 &

# 4) 闭环
ros2 launch mpc_control swarm_launch.py scenario:=IN_solo1_hover
```

## 4. 验证结果（2026-07-23 实测）

**坐标变换单元自检**（`tools/test_frame_convert.py`）9 项全过，含 ENU 朝东→NED +90°、
朝北→0°、朝西→−90°、2000 个随机四元数往返（最大误差 3.5e-16）、俯仰不被混淆。

**独立交叉验证**：Gazebo 里 x500 以单位姿态出生（ENU 朝东），PX4 自己算出的
NED heading = 90° —— 与上面自检的第一条从两条独立路径对上。

**闭环飞行**（`IN_solo1_hover`，GPS 与气压计均已禁用，位置**只来自注入的动捕**）：

| 量 | EKF 估计 | Gazebo 真值 | 差 |
|---|---|---|---|
| x | −0.0005813 | −0.0005167 | 6.5e-5 m |
| z | −1.1999916 | −1.2000006 | 9.0e-6 m |
| heading | 6.61e-5 | 9.15e-5 | 2.5e-5 rad |

`arming_state=2`(ARMED) / `nav_state=14`(OFFBOARD) / `failsafe=false`，
MPC `solve 0.25ms`、`cost=0`、`fallbacks=0`。

**运动中三点核验**（`IN_solo1_circle`，800 样本 / 40 s，最大水平位移 3.005 m）：

| 检查 | 最大误差 |
|---|---|
| ①真值 → ②中间跳的 ENU 关系 | 8.2 mm |
| ①真值 → ③EKF 端到端 | 11.7 mm |

毫米级残差是三个话题非同步采样的时间错位（0.4 m/s × ~20 ms ≈ 8 mm），非系统偏差。

> ⚠️ **为什么必须单独查 ①→②**：源与 bridge 用的是同一个对合函数正反各调一次，
> `F∘F ≡ 恒等` —— **即使 F 写错，往返也会自动抵消**，①③ 照样完美吻合而中间跳是错的。
> 同理，**悬停在原点的测试查不出 x/y 互换**（两者都≈0），必须在运动中验。

**动捕断流**（杀掉源节点模拟遮挡）：bridge 看门狗 0.35 s 报警 → EKF `xy_valid`/`z_valid`
约 1 s 内转 false → 飞控 `Failsafe activated` + `mc_pos_control: Failsafe: blind land`
（盲降，无位置控制）→ `nav_state` 14(OFFBOARD) → **13 = TERMINATION**
（查 `msg/VehicleStatus.msg` 确认）。

## 5. 打通过程中发现的三个问题

### 5.1 🔴 飞场文档的 EKF2 参数不完整，缺 `EKF2_MAG_TYPE`

只按飞场文档配那五个参数（`EV_CTRL=11 / HGT_REF=vision / GPS_CTRL=0 /
BARO_CTRL=disabled / RNG_CTRL=0`）**解不了锁**：

```
WARN [health_and_arming_checks] Preflight Fail: Yaw estimate error
```

`EKF2_MAG_TYPE` 默认 0(Automatic)，EKF 会**同时融合磁力计偏航与视觉偏航**，两者
互相打架。设 `EKF2_MAG_TYPE=5`(None) 后立即解锁起飞。已加进
`tools/fc_configure.py` 的 `ekf2_mocap` 组。

室内金属结构 + 电机磁干扰下磁力计本就不可信，动捕方案理应把偏航完全交给视觉。

### 5.2 🟡 `IN_solo1_circle` 的实际活动半径是配置值的两倍

`leader_node.py:232` 是 `cx = self._x0 - self._radius` —— **圆心偏置在起飞点旁边**，
起飞点落在圆周上，因此离起飞点最远处是 **2×radius**。

`IN_solo1_circle` 配 `radius: 1.5`，实测最大水平位移 **3.005 m**，而：
- 飞场 5×5 m ⇒ 可用半径约 2.5 m — **超了**
- 今日已写入飞控的 `GF_MAX_HOR_DIST=2.0` — **会触发围栏**

`scenarios.yaml` 里"`leader_radius=1.5` —— 场地半径 2.5m 减去约 1m 安全边距"这句
注释基于"圆心在起飞点"的错误假设。**按 2×radius 折算，室内圆周场景的 radius
应 ≤ 1.0**（且要等动捕标定体积尺寸确认后再定死）。

`line` 模式无此问题（`leader_node.py:278` 起，距离由 0 单调走到 `max_distance`），
但 `IN_solo1_line` 的 `max_distance: 2.0` **正好等于**围栏值，零余量。

各 `IN_*` 场景的活动范围与今日已写入飞控的 `GF_MAX_HOR_DIST=2.0` 对照
（⚠️ PX4 围栏是**相对各机自己的 home/起飞点**，不是场地中心）：

| 场景 | 离**自身起飞点**最远 | vs 围栏 2.0 | 离**场地中心**最远 | vs 可用半径 2.5 |
|---|---|---|---|---|
| `IN_solo1_hover` | 0 | ✅ | 0 | ✅ |
| `IN_solo1_line` | 2.0 | ⚠️ 零余量 | 2.0 | ✅ |
| `IN_solo1_circle` | **3.0** | ❌ 超 | **3.0** | ❌ 超 |
| `IN_pair2_hover` | 0 | ✅ | 1.4 | ✅ |
| `IN_pair2_line` | 1.0 | ✅ | 2.4 | ⚠️ 贴墙 |
| `IN_trio3_hover` | 0 | ✅ | 1.5 | ✅ |
| `IN_trio3_line` | 0.8 | ✅ | 2.3 | ⚠️ 贴墙 |

两个约束性质不同：**围栏**触发 `GF_ACTION`（当前 Hold）会中断试验；**场地半径**
超了是撞墙。动捕标定体积尺寸拿到后两列都要重算。

**已处理（2026-07-23）**：`IN_solo1_circle` 的 `radius` 1.5 → **0.8**（2×0.8=1.6，
离围栏留 0.4 m）、`IN_solo1_line` 的 `max_distance` 2.0 → **1.5**，并更正了
`scenarios.yaml` 里那句基于错误假设的注释。

**没有改 `leader_node`**：`cx = x0 - radius` 的偏置圆心是有意设计（起飞点落在
圆周上，避免 t=0 位置阶跃，与 `circle_ramp_time` 缓启动配套），且改它会改变**所有**
圆周场景的轨迹语义——41 场景回归、论文 S33/S34 与 topo_sweep 四拓扑实验全部
建立在这个约定上。该缩的是轨迹，不是判据。

同理**没有放大 `GF_MAX_HOR_DIST`** 去迁就轨迹：围栏是 5×5 房间里的最后一道保护。

### 5.3 🟡 `world_birth` 校准的门控语义在动捕下不再成立

`mpc_node.py:1029` 的 `near_origin` 门控：

```python
# EKF 收敛后静止于自身 local 原点，|local_xy| 必接近 0
near_origin = norm(first_local[:2]) < calib_max_origin_offset   # 2.0 m
```

这条假定**每架机的 EKF 原点在自己的出生点**。动捕是**共享原点**，`first_local`
就是该机在场地系里的真实位置：

| 队形 | 各机离场地原点 | 是否 < 2.0 |
|---|---|---|
| solo1 | 0 m | ✅ |
| pair2_in | 1.4 m | ✅ |
| trio3(scale 0.5) | 外接半径 1.5 m | ✅ |

**目前撞大运通过**，但这条门控的语义已经错了：动捕下一架合法停在离原点 3 m 处的
飞机会被误判成"EKF 未 fix"而拒绝校准（打 `calib STUCK` 红字守出生点）。

校准公式本身 `world_birth = birth − first_local` 是**自洽的**（共享原点下
`world_birth` 自动归零，世界坐标仍正确，本次实测 `wbirth=(0,0,0)` 印证），
问题只在门控。

**已处理（2026-07-23）**：新增参数 `calib_shared_origin`（默认 **false** = 原行为
完全不变，41 场景回归与冻结的论文基线零风险）+ `calib_shared_origin_tol`（默认 0.5 m）。

```python
if calib_shared_origin:            # 动捕：共享原点
    gate = |first_local - birth_i| < calib_shared_origin_tol
else:                              # GPS/SITL：各机独立原点（原行为）
    gate = |first_local|          < calib_max_origin_offset
```

经 `swarm_launch.py` 的 `calib_shared_origin:=true` 传入。**做成 launch 参数
而不写进 `IN_*` 场景**：同一场景在普通 SITL(GPS) 下也要能跑，焊死会让 GPS 模式用错判据。

容差取 0.5 m 而非沿用 2.0 m：动捕给的就是真实位置，偏差是厘米级；2.0 m 是留给
GPS 收敛暂态的，在动捕下会放过 trio3 尺度（边长 2.60 m）的刚体错接。

**这条改动的真正价值是把静默故障变成响亮故障**。以 `pair2_in`（各机距中心 ±1.4 m）
两机刚体名接反为例：

| | 原判据 | 新判据 |
|---|---|---|
| drone0 收到的 local | (−1.4, 0)（实为 drone1 的位置） | 同左 |
| 门控 | \|local\|=1.4 < 2.0 → **通过** | \|local−birth\|=2.8 > 0.5 → **拒绝** |
| 后果 | `world_birth` 烤成 (2.8,0)，两机都以为自己在出生点、实际飞向同一物理点 → **撞机，全程零报错** | 报错并保持出生点悬停 |

#### 判据必须**闩锁**——只看瞬时值挡不住（实测两次才收敛到这个结论）

用 `sitl_mocap_source_node` 的 `offset_enu:=[1.0,0,0]` 单机模拟刚体错接，实测：

| 轮次 | 实现 | 结果 |
|---|---|---|
| 1 | 仅瞬时判据 `\|local−birth\| < tol` | 先正确拒绝(dev=0.93m)，**随后仍被放行** —— dev 一路降到 0.50m，走 `timeout` 兜底锁定 `world_birth=(0,−0.50,0)` |
| 2 | 加"共享原点下必须 `stable`、不接受 timeout" | **仍被放行** —— 飞机稳定后 dev 只剩 0.07m 且 spread 收敛，`stable` 条件真的满足了 |
| 3 | **闩锁**：一旦检出即永久拒绝该机校准 | ✅ 0 次校准，持续报 `calib LATCHED: 曾检出 \|local_xy-birth\|=1.00m > 0.5m` |

**为什么瞬时判据必然失效**：未校准时飞机已被允许飞行去"守出生点"，而 EKF 认为
自己偏了 1 m，于是**物理上朝反方向飞 1 m** 把这个不存在的偏差消除掉。等它稳定，
偏差是**真的**小了 —— 注入的配准错误被转化成了**物理位移**而非被检出。任何基于
当前 `local` 的判据都会在这之后放行。

闩锁的合法性：动捕是绝对定位，从第一帧起就该对得上，不存在"先偏后正"的合法暂态
（那是 GPS fix 收敛才有的）。闩锁只在 EKF 已连续有效 `calib_settle_frames` 帧后
才生效，避开"EV 尚未融合、`local` 还是 0 而 `birth` 非 0"的初始化暂态（多机必踩）。

⚠️ 报错文案存的是**闩锁瞬间**的 dev：飞机随后会漂走，每帧重算会打印出
"0.04m > 0.5m"这种自相矛盾的话（第 3 轮实测踩到，已修）。

**验证**（`IN_solo1_hover` + `calib_shared_origin:=true`）：

| 用例 | `offset_enu` | 期望 | 实测 |
|---|---|---|---|
| 接受 | [0,0,0] | 正常校准 | ✅ `calibrated [STABLE]`，`world_birth=(0,0,0)` |
| 拒绝 | [1.0,0,0] | 永久拒绝 | ✅ 0 次校准，`LATCHED`，报出 dev=1.00m |

**仍未验证**：真正的多机刚体错接（需 2 架 + 2 个动捕源），单机静态偏移只是它的等价
模拟。这是多机注入那一步的验收用例。

## 6. 还原 SITL 参数（测完必做）

```bash
cp /tmp/params_before.bson ~/PX4-Autopilot-1.14/build/px4_sitl_default/rootfs/0/parameters.bson
# 验证：备份里不应含被改过的 EKF2 项（不在文件里 = 用默认值）
strings .../parameters.bson | grep -cE 'EKF2_(EV_CTRL|HGT_REF|GPS_CTRL|BARO_CTRL|RNG_CTRL|MAG_TYPE)'   # 应为 0
```

2026-07-23 本次测试已还原并核验（md5 一致、EKF2 改动项计数为 0）。

## 7. 尚未做的

- **多机**（pair2/trio3）：共享原点下的多机注入未测，`5.3` 的门控问题在多机上
  才真正有风险；且需要每架机一个刚体名 → `drone_id` 的映射约定
- **真 VRPN 客户端**：`vrpn_mocap` 包尚未在仿真机/Orin NX 上安装
- **`EKF2_EV_DELAY` 标定**：本次 SITL 注入延迟≈0，真动捕有网络+处理延迟，
  需实测后配（配错会在机动时表现为位置滞后/振荡）
- **噪声/延迟压测**：`sitl_mocap_source_node` 已支持 `noise_pos_m`/`latency_ms`/
  `dropout_*` 参数，本次只测了完全断流
