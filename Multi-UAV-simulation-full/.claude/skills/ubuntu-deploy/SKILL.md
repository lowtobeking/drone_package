---
name: ubuntu-deploy
description: 输出在 Ubuntu 仿真主机上拉取最新代码、编译并启动仿真的完整命令序列。在 Windows 端改完代码 push 后，用此 skill 生成 Ubuntu 端需要运行的命令。
disable-model-invocation: false
---

用户运行 /ubuntu-deploy 时，询问：
1. 需要哪种队形？(solo1 / pair2 / trio3 / cross5 / star5 / grid9)
2. 领队模式？(hover / circle / line)
3. 是否需要清理 acados 缓存？（修改了 mpc_node.py 或 MPC 结构时选是，默认选是）

然后输出以下命令块，用户可直接复制到 Ubuntu 终端执行：

---

**步骤 0：清理残留进程（每次启动前必须执行；逐个明确进程名，pkill -9 -f 不会误伤 claude）**
```bash
for p in px4 'gz sim' gzserver MicroXRCEAgent mpc_node leader_node swarm_launch 'ros2 launch'; do pkill -9 -f "$p"; done
ros2 daemon stop 2>/dev/null; pkill -9 -f 'ros2-daemon'
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*            # FastDDS 共享内存残留（kill -9 攒出来的，会卡死 Agent 致 DDS 反复断连）
find ~/PX4-Autopilot-1.14 -name "parameters*.bson" -delete   # 旧参数含 backup（只删主文件 PX4 会读 backup 兜底）
# 必做验证：下面这条应「零输出」才算清干净（gz sim server 会忽略信号、最易残留，需按 PID kill -9）
ps aux | grep -E "px4|gz sim|gzserver|MicroXRCE|mpc_node|leader_node|swarm_launch|ros2-daemon" | grep -v grep
```

**⚠️ 步骤 0.5：DDS 隔离（2026-07-27 grid9 九机集体断连教训 + 2026-07-28 纠偏）**
- **拔掉 Orin NX / 机载电脑的 USB 线**（其网口上的 ROS2/DDS 会把 Agent 拖到 timesync RTT 1000ms+，全员 ping 超时断连；用户实证拔线即恢复）
- **⚠️ 不要设置 `ROS_LOCALHOST_ONLY=1`**（2026-07-28 实测坐实：这会导致 MicroXRCEAgent 的内部 DDS
  participant 完全无法被 ROS2 发现——`ros2 topic list` 永远零 fmu 话题，drone 卡死在 `arm=0`
  解锁不了。根因：Agent 走 XRCE 协议创建 participant（`create_by_bin`/`create_by_ref`），
  不经过 `FASTRTPS_DEFAULT_PROFILES_FILE` 的默认 profile 查找，实测怎么配 `config/agent_localhost.xml`
  都不生效，Agent 的 participant 因此没能带上 127.0.0.1 locator，被 `ROS_LOCALHOST_ONLY` 的
  严格过滤整体拒收。`~/.bashrc` 里这行已改成注释状态，勿再打开）
  - 之前"修复已固化"的结论是误判：只验证了 Agent↔PX4 客户端层不再断连，没端到端验证过
    `ros2 topic list`/实际订阅，两层问题被混在一起了
  - `config/agent_localhost.xml` 目前对本项目使用的 MicroXRCEAgent 版本无效，终端2 不需要再带
    `FASTRTPS_DEFAULT_PROFILES_FILE`，直接 `MicroXRCEAgent udp4 -p 8888` 即可
  - 外部 WiFi 设备的组播干扰风险：目前观测影响有限（不同于 07-27 那次 USB 直连的严重 RTT 问题），
    如果将来又出现全员断连，优先怀疑"新连了什么设备"，而不是重新开 ROS_LOCALHOST_ONLY
- **启动顺序铁律：Agent 必须先于 PX4**——PX4 首次握手失败后不再重试（日志 `uxr_create_session failed` 即此）

**步骤 1：拉取最新代码**
```bash
cd ~/ros2_control_mpc_ws
git pull origin main
mkdir -p ~/flights          # CSV 记录目录（首次建一次即可，供终端5 的 --log 使用）
```

**步骤 2：赋予启动脚本执行权限（首次拉取后执行一次即可）**
```bash
chmod +x src/mpc_control/start_*_px4.sh   # 覆盖 1/2/3/5/9 全部脚本
```

**步骤 3：清理 acados 缓存（仅当改了 MPC「OCP 结构」：horizon N / 状态维 / 约束 / 邻居数）**
```bash
rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*
```
> ⚠️ 纯 Python / 参数改动（标定、leader、就绪门控、记录器等）**不必清缓存**，清了只是白等几分钟重编译。只有动了 acados OCP 结构才需清。

**步骤 4：编译**
```bash
cd ~/ros2_control_mpc_ws
colcon build --packages-select mpc_control
source install/setup.bash
```

---

根据队形选择对应的启动命令：

**solo1（单机诊断，Phase 0）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2（等 Gazebo 就绪后，先开 Agent；不要带 ROS_LOCALHOST_ONLY=1）
MicroXRCEAgent udp4 -p 8888
# 终端3（等 Agent 起来后再开 PX4）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_1_px4.sh
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=solo1 [leader_mode:=<hover|circle|line>] [leader_speed:=1.5] [leader_radius:=10.0]
# 终端5（诊断监控）
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation solo1 --log ~/flights/flight_solo1_<traj>.csv
```

**pair2（双机，Phase 1）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2（等 Gazebo 就绪后，先开 Agent；不要带 ROS_LOCALHOST_ONLY=1）
MicroXRCEAgent udp4 -p 8888
# 终端3（等 Agent 起来后再开 PX4）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_2_px4.sh
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=pair2 [leader_mode:=hover]
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation pair2 --log ~/flights/flight_pair2_<traj>.csv
```

**trio3（三机，Phase 2）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2（等 Gazebo 就绪后，先开 Agent；不要带 ROS_LOCALHOST_ONLY=1）
MicroXRCEAgent udp4 -p 8888
# 终端3（等 Agent 起来后再开 PX4）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_3_px4.sh
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=trio3 [leader_mode:=hover]
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation trio3 --log ~/flights/flight_trio3_<traj>.csv
```

**cross5 / star5（5机，Phase 3）**
⚠️ cross5 的 circle/line **必须用 `scenario:=`**（S5_cross5_circle / S4_cross5_line），不要手拼
leader_mode/leader_speed/leader_radius——S5 带的 `w_collision:=500`（CLI 传不进去）是修复过
0.03m 中心-臂近撞险情的调优，S4 带的 `ready_hold=30s`（同样传不进去）防"边追边成型"瞬态畸变。
star5 circle 没有安全滤波开着的验证场景（唯一的 `S_star5_circle_nosafety` 是刻意关安全滤波的
研究对比用途），手拼参数是唯一选项；star5 line 用 `scenario:=S9_star5_line`。
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2（等 Gazebo 就绪后，先开 Agent；不要带 ROS_LOCALHOST_ONLY=1）
MicroXRCEAgent udp4 -p 8888
# 终端3（等 Agent 起来后再开 PX4；5机务必用 start_5_px4.sh，误用 start_9 会让 drone5-8 停在地面）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_5_px4.sh
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=cross5   # 悬停
ros2 launch mpc_control swarm_launch.py scenario:=S5_cross5_circle   # cross5 圆周
ros2 launch mpc_control swarm_launch.py scenario:=S4_cross5_line     # cross5 直线
ros2 launch mpc_control swarm_launch.py formation:=star5 [leader_mode:=<模式>] [leader_speed:=1.5] [leader_radius:=10.0]   # star5
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation <cross5|star5> --log ~/flights/flight_<cross5|star5>_<traj>.csv
```

**grid9（9机，Phase 3）**
⚠️ line/circle **必须用 `scenario:=`**，不要手拼 `leader_mode:=circle leader_speed:=... leader_radius:=...`——
S7/S8 带了 `ready_hold=30s`，circle 还带 `alt_resync_enable`/`w_collision`/`vel_xy_kp`/`vel_xy_cap`/
`alt_trim_enable`/`safety_max_track_dist` 等**只能从 scenario.yaml 传入、CLI 参数传不进去**的防撞调优；
手拼参数是未验证组合（cross5 circle 曾因同类问题把中心-臂间距压到 0.03m 险些相撞）。
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2（等 Gazebo 就绪后，先开 Agent；不要带 ROS_LOCALHOST_ONLY=1）
MicroXRCEAgent udp4 -p 8888
# 终端3（等 Agent 起来后再开 PX4）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_9_px4.sh
# 终端4（三选一，都用 scenario:=）
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py scenario:=S7_grid9_hover    # 悬停
ros2 launch mpc_control swarm_launch.py scenario:=S7_grid9_line     # 直线(0.5m/s)
ros2 launch mpc_control swarm_launch.py scenario:=S8_grid9_circle   # 圆周(0.5m/s R10)
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation grid9 --log ~/flights/flight_grid9_<traj>.csv
```

---

**就绪门控（af59b66）**：leader 不再死等固定 `start_delay`，而是**等所有机进入编队（pos_err<0.5m 保持 2s）才自动开始运动**，日志打 `formation ready — starting`；90s 超时兜底打红字。想退回旧固定延时：launch 加 `ready_gate_enable:=false`。

**飞行记录（3e1b3e9）**：上面终端5 已带 `--log`，每秒写一行 CSV 到 `~/flights/`（`<traj>` 换成所选 hover/line/circle）。跑完用 analyze_flight 出体检报告：
```bash
python3 ~/ros2_control_mpc_ws/src/mpc_control/analyze_flight.py ~/flights/flight_<队形>_<traj>.csv [--plot]
```
> 完整逐场景「控制器+记录」命令见 `report/CORE_run_commands.md`（CORE S1–S8）与 `report/RUN_PLAN_仿真运行清单.md`。

---

**任务完成自动降落（第1层）**：收场别用 Ctrl+C（会把整个仿真栈一起端掉，飞机直接掉=炸机）。
- 运行中手动一键：另开终端 `ros2 param set /leader_node trigger_land true`
- 或启动即定时：launch 命令末尾加 `mission_duration:=N`（运动开始 N 秒后自动降；默认0=禁用）
- 原理：leader 广播 LAND → 各机零速悬停 `land_settle_s`(默认1.5s) → 各自 `AUTO.LAND` 垂直下降、
  触地自动上锁。判据：PX4 日志 `Landing detected`+`Disarmed by landing`。

**通信失联自动降落（第2层，兜底）**：跟第1层不同，这层是**各机 mpc_node 自带看门狗**，不靠
leader 广播——OFFBOARD 中若超过 `comms_loss_land_s` 秒没收到 `/leader/state` 自动本机降落。
- 启用：launch 命令末尾加 `comms_loss_land_s:=5.0 [comms_loss_hold_s:=1.0]`
  （`comms_loss_hold_s` 是可选的"先冻结"中间态，须 < `comms_loss_land_s`，到点先原地悬停
  不追陈旧参考，避免冲向失效前的旧目标点）
- 默认全关（0.0），需显式加参数；已验证单机/pair2双机同时触发+3.02m无碰撞
- 详细测试步骤见 `report/失联降落第2层_SITL验证.md`

⚠️ 本 skill 只覆盖 **SITL 仿真**。飞场真机只用 `real_hardware_launch.py` + `IN_*` 场景（已按
7×8.5m 场地缩放），严禁把这里的 `leader_radius:=10.0` 等仿真尺度参数照搬到真机。

---

正常启动标志：
- Gazebo 中出现对应数量的无人机模型
- MicroXRCEAgent 显示 `[CREATE  CLIENT]` session 建立
- 各 mpc_node 输出 `acados OCP ready`
- 约 2 s 后输出 `OFFBOARD + ARMED confirmed`
- diag_monitor 显示所有机 ARM=ARMED, NAV=OFFBOARD
