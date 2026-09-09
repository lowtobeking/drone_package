# 真机安全配置清单（FC 侧 / QGC）— PX4 v1.14

> **适用飞控**：CUAV Nora+ / Pixhawk 6C 及其他 PX4 v1.14 板卡。本清单参数均为
> **PX4 固件级**，与板型无关；**仅"TELEM2 对应的内部设备路径"因板而异**
> （6C = `/dev/ttyS3`，2026-07-21 由启动日志实机确认；Nora+ 需用
> `uxrce_dds_client status` 自行核对，勿照抄）。
> 台架首跑用 **Pixhawk 6C**（2026-07-21 定），换板上天前须重验 FC 侧配置。
>
> 适用：companion(Orin NX) 经 XRCE-DDS(TELEM2) 发 OFFBOARD 速度 setpoint 的多机编队。
> **本清单是第一道、也是最关键的防线**——跑在 FC 上、独立于 companion/ROS，companion 死机或 ROS 崩了它照样保你。companion 侧 `safety_filter`（Step 2）是第二道网。
> ⚠️ 参数名以 QGC 参数搜索框实际存在为准（跨 PX4 版本可能改名/改枚举）；下表给「QGC 下拉标签」而非魔数，更稳。**每架 FC 独立配一遍。**

## 0. 安全分层（心里要有这张图）
1. **FC 失效保护**（本清单）——围栏/失联/电池，FC 自主触发 RTL/Land/Hold。
2. **companion safety_filter**（Step 2）——下发前拦住飞散/贴近/坏估计。
3. **飞手 RC kill 开关**——终极人工中止，任何时刻可瞬间锁桨。三层缺一不可。

## 1. 必配参数

### 1.1 电子围栏（防飞散——最高优先）
| 参数 | 建议值 | 说明 |
|---|---|---|
| `GF_ACTION` | **Return** 或 **Land** | 越界动作。首飞建议 Land（就近落），熟悉后用 Return(RTL) |
| `GF_MAX_HOR_DIST` | **首飞 30–50 m** | 距 home 最大水平距离。按场地缩到最小够用。**须 > 任务半径 + companion 软围栏 6 m**（Phase 3 标定 `safety_max_track_dist=6.0`，companion 先刹、FC 兜底；如 circle R=10 → 至少 16 m + 裕度） |
| `GF_MAX_VER_DIST` | **首飞 15–20 m** | 最大高度。target_alt=5m，留足裕度即可 |
| `GF_SOURCE` | GPS | 围栏判据用 GPS |

### 1.2 OFFBOARD 失联（companion 方案的命脉）
companion/ROS 一旦停发 setpoint（崩溃、断电、线松），**FC 必须自己接管**。`safety_filter` 的"交还 PX4"也靠这个生效。
| 参数 | 建议值 | 说明 |
|---|---|---|
| `COM_OF_LOSS_T` | **0.5 s** | offboard 信号超时阈值。setpoint 流断超过它即判失联 |
| `COM_OBL_RC_ACT` | **Hold** 或 **Return** | 失联后动作（RC 可用时）。首飞 Hold(悬停等飞手)，远场用 Return |
| `COM_RCL_EXCEPT` | **不勾 Offboard** | 不要把 offboard 设成"无 RC 也允许"——保留 RC 作安全 |

### 1.3 RC 失联
| 参数 | 建议值 | 说明 |
|---|---|---|
| `COM_RC_LOSS_T` | 0.5 s | RC 信号超时 |
| `NAV_RCL_ACT` | **Return** 或 **Land** | RC 丢了的动作 |

### 1.4 数传/地面站失联
| 参数 | 建议值 | 说明 |
|---|---|---|
| `NAV_DLL_ACT` | **Hold** 或 Return | 数据链(LQ-3)丢失动作。注意：leader 经地面站中继时，链路质量直接影响编队 |
| `COM_DL_LOSS_T` | 10 s | 数传超时 |

### 1.5 电池失效保护
| 参数 | 建议值 | 说明 |
|---|---|---|
| `BAT_LOW_THR` | 0.15 | 低电量阈值(15%) |
| `BAT_CRIT_THR` | 0.07 | 临界(7%) |
| `BAT_EMERGEN_THR` | 0.05 | 紧急(5%) |
| `COM_LOW_BAT_ACT` | **Return at low, Land at critical** | 注意 BEC≥5V/3A，电压跌落会误触发 |

### 1.6 速度/姿态硬帽（FC 侧兜底，设在 companion 限幅之下）
companion `conservative` 限 v≤1.5/climb≤1.0/a≤2.0；FC 再设一层更高的硬上限，防 companion 发疯。
| 参数 | 建议值 | 说明 |
|---|---|---|
| `MPC_XY_VEL_MAX` | 2.0 m/s | 水平速度硬上限（> companion 1.5，作兜底） |
| `MPC_Z_VEL_MAX_UP` / `_DN` | 1.5 / 1.0 | 升/降速上限 |
| `MPC_TILTMAX_AIR` | 25–30° | 最大倾角，防速度环发疯后翻 |

### 1.7 Kill 开关 / 解锁落地
| 参数 | 建议值 | 说明 |
|---|---|---|
| `RC_MAP_KILL_SW` | 分配一个 RC 通道 | **必配**。飞手拇指随时能锁桨 |
| `COM_KILL_DISARM` | 5 s | kill 后自动 disarm 时间 |
| `COM_DISARM_LAND` | 2 s | 落地后自动锁桨 |

### 1.8 解锁前检查（保持默认开）
- EKF/GPS 健康预检 `COM_ARM_EKF_*`、`COM_ARM_WO_GPS=0`（要求 GPS）——**不要为图省事关掉**，这正是"别带病起飞"的 FC 侧体现。

## 2. 多机注意
- 每架 FC 独立：`MAV_SYS_ID = drone_id+1`；围栏/失联/电池**逐机各配一遍**。
- 各机 home 点不同 → 围栏以**各自 home** 为心；地面摆位间距务必 ≥ d_safe(真机 2.5m)。
- `drone_id≥1` 的 FC：SD 卡 `etc/extras.txt` 设 XRCE 命名空间 `-n px4_<id>`（见 real_hardware_launch.py 文件头）。

## 3. 起飞前逐项检查（接 real_hardware_launch 起飞顺序）
- [ ] 上表 1.1–1.8 每架 FC 配完并写入（QGC 改完点"保存"，重启 FC 复核没回滚）
- [ ] 地面摆位到 births 标记点，误差 < `calib_max_origin_offset`=2m，机间 ≥ d_safe
- [ ] **桨先不装**，台架跑通一次：launch → 等 "waiting RC ARM+OFFBOARD" → RC 解锁 → 看 setpoint 流/姿态响应正常 → 锁桨装桨
- [ ] 各 companion `chrony` 时钟同步
- [ ] **DDS 链路三项一致**（2026-07-21 实机验证，任一不符都会【静默失效】——节点起得来、日志零报错、但话题互相看不见）：
  - [ ] `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 全员一致 —— **勿用 CycloneDDS**（MicroXRCEAgent 基于 Fast-DDS 编译，实测 cyclonedds 下 `/fmu/` 话题为 0；仿真机亦从未装过，FastRTPS 才是本项目验证过的配置）
  - [ ] `ROS_DOMAIN_ID=42` 全员一致（companion + 地面站）
  - [ ] **每架 FC 的 `UXRCE_DDS_DOM_ID` = 42**（默认 0，`reboot_required`）—— Agent 的 DDS 域由此飞控参数决定，只设 companion 侧无效
  - [ ] 验证：**首选 `ros2 topic hz /fmu/out/vehicle_status`**（应见 ~1.97Hz）。
    `ros2 topic list --no-daemon | grep fmu`（应见 22 个）只作参考——**它本身就有发现竞态**：
    2026-07-22 连查三次得到 22/22/**0**，每次都是新建节点、DDS 拓扑发现没完成就返回快照。
    `topic hz` 自建节点持续等待，不受此影响，才是可靠判据。
    （`--no-daemon` 仍然必须加，否则 `ros2 daemon` 缓存会给另一种假阴性）
- [ ] **非交互方式启动须显式 export 环境变量**（systemd/nohup 不读 `.bashrc`，否则静默退回 domain 0 + 默认 RMW，编队无提示失效）
- [ ] 首次 launch 会现编 acados OCP，**外场前务必台架编译过一次**
      （Orin NX 实测：首次 7.5s，之后指纹命中 1.0s；缓存在 `~/.cache/mpc_control/`，
      跨重启保留。改代码或改限幅参数会让指纹失效、重新编译，属预期行为）
- [ ] MicroXRCEAgent 已配 systemd 开机自启（`microxrce-agent.service`），上电即通，
      不需再 ssh 进去手动起；复核用 `systemctl is-active microxrce-agent`
- [ ] 飞手 kill 开关测过（解锁后拨一下确认真锁桨）
- [ ] 围栏边界 < 场地边界，且 < 任何障碍/人群距离

## 4. 中止判据（满足任一，飞手立即 kill 或切手动）
- 任一机水平偏出预期 > ~5m 且仍在扩大
- 两机肉眼可见快速接近 / 间距明显小于队形间距
- 任一机高度异常（冲高 / 掉高）
- 日志狂刷 `fallback` / `calib STUCK` / `acados status≠0`
- LQ-3 链路质量骤降、leader 卡顿

## 5. 渐进放飞协议（别一上来就 trio3）
1. **单机** solo1 hover，低空(2–3m)、围栏收到最小 → 确认悬停/落点/失联动作
2. 单机 line/circle 低速 → 确认轨迹跟踪 + 端点行为
3. **pair2** hover → line：重点看机间间距与 world_birth 一致性（你历史上 xy 塌缩的真凶）
4. **trio3**：最后做，间距裕度留够
- 每升一级先复盘上一级日志（`analyze_flight.py`），无异常再升。

---
**关联**：companion 侧加固见 `safety_filter`（模块自测 15/15 + SITL 回归 S23–S33 全部通过，含本清单 1.2 节 `COM_OF_LOSS_T`/`COM_OBL_RC_ACT` 配置对应的 RELINQUISH→PX4 接管链实测 S31，见 `阶段性报告.md` §5.D）；本清单随固件升级复核参数名。
**跨架次解闩（2026-07-14）**：RELINQUISH 闩锁在 disarmed→armed 新架次边沿自动清零（`mpc_node._on_vehicle_status`）——外场若某机触发 RELINQUISH 交还 PX4，落地后**重新 RC 解锁即可恢复 companion 控制，无需重启 companion/ROS 进程**；同一架次内闩锁仍 sticky 不自恢复。
**队友失能行为（2026-07-14）**：编队中某机 mpc 静默（节点崩溃/失联）而其 FC 仍在发位置时，其余机的编队项对它自动失效化（不再被"拽向"趴窝机造成恒定跟踪偏差），碰撞规避地板不受影响照常生效——SITL S23/S24 实测验证。外场含义：单机故障退出不影响幸存机继续正常跟队。
