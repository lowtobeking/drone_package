# CORE 场景运行命令速查 / CORE Run Commands

> 配套 `report/RUN_PLAN_仿真运行清单.md`（完整说明 + EXT 场景 + 进度勾选表）与 `report/阶段性报告.md`。
> 本文件是 **Ubuntu 端照着贴** 的精简版：CORE 8 个场景（S1–S8），按机数分批，每个场景两条命令（**终端4** 控制器+领队 / **终端5** 诊断记 CSV）。
> 工作流：Windows 编辑 → `/commit-push` → Ubuntu `git pull` 后使用。

---

## 一次性准备（Ubuntu）

```bash
mkdir -p ~/flights
cd ~/ros2_control_mpc_ws/src/mpc_control && git pull origin main   # 拉最新 diag_monitor（含 x,y 轨迹列）
```

> `diag_monitor.py` 是直接 `python3` 跑的，`git pull` 即生效，**无需 colcon build**。只有改了 `mpc_node.py` 的 MPC OCP 结构（horizon/状态/邻居数/约束）才需 `rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*` 后重编。

## 节奏要点

- 每个场景两条命令：**终端4** 起控制器+领队；**终端5** 起诊断并 `--log` 记 CSV。
- 两条尽量同时起；想抓「起飞→成型→机动」全过程，**先起终端5 的 `--log`，再起终端4 控制器**最稳。
- 跑够时长：**hover/line ≥ 120 s；circle ≥ 1–2 圈或 ≥ 120 s**。完事 **终端4、终端5 都 `Ctrl+C`**。
- 起飞后先看面板：全员 `ARM=ARMED`、`NAV=OFFBOARD`、`Min spacing > 1.8 m`、`fallback=0`，再开始计时。
- **同队形换轨迹（机数不变）：终端1/2/3 不动，只 `Ctrl+C` 重起 终端4+终端5。** → cross5 的 line/circle/hover 共用一次起飞；grid9 的 line/circle 共用一次。
- **换队形（机数变）：先清理，再重起 终端1-3。**
- 建议顺序由简到繁：**solo1 → pair2 → trio3 → cross5 → grid9**。

## 清理 + 底座（换队形/机数时执行；`<N>` 按下表选）

| 队形 formation | start 脚本 | diag `--formation` |
|---|---|---|
| solo1 | `start_1_px4.sh` | solo1 |
| pair2 | `start_2_px4.sh` | pair2 |
| trio3 | `start_3_px4.sh` | trio3 |
| cross5 / star5 | `start_5_px4.sh` | cross5 / star5 |
| grid9 | `start_9_px4.sh` | grid9 |

```bash
# 清理（gz server 最易残留）
for p in px4 'gz sim' gzserver MicroXRCEAgent mpc_node leader_node swarm_launch 'ros2 launch'; do pkill -9 -f "$p"; done
ros2 daemon stop 2>/dev/null; pkill -9 -f 'ros2-daemon'
# 终端1：Gazebo（窗口出现后再等 ~10s）
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2：PX4 实例（按队形选 start_<N>）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_<N>_px4.sh
# 终端3：DDS 桥
MicroXRCEAgent udp4 -p 8888
# 终端4：编译并 source（首次 / 改过被编译的 .py 后）
cd ~/ros2_control_mpc_ws && colcon build --packages-select mpc_control && source install/setup.bash
# 终端5：也要 source（diag 用 px4_msgs）；若报找不到 px4_msgs 先 source /opt/ros/humble/setup.bash
source ~/ros2_control_mpc_ws/install/setup.bash
```

---

## 批次 1 · solo1（start_1）

### S1 · solo1 · line
```bash
# 终端4
ros2 launch mpc_control swarm_launch.py formation:=solo1 leader_mode:=line leader_speed:=1.0 max_distance:=20.0
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation solo1 --log ~/flights/flight_solo1_line.csv
```

## 批次 2 · pair2（start_2）

### S2 · pair2 · line
```bash
# 终端4
ros2 launch mpc_control swarm_launch.py formation:=pair2 leader_mode:=line leader_speed:=0.5 max_distance:=20.0
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation pair2 --log ~/flights/flight_pair2_line.csv
```

> 悬停基线（可选）：终端4 改 `leader_mode:=hover`，终端5 改 `--log ~/flights/flight_pair2_hover.csv`。

## 批次 3 · trio3（start_3）

### S3 · trio3 · circle
```bash
# 终端4
ros2 launch mpc_control swarm_launch.py formation:=trio3 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation trio3 --log ~/flights/flight_trio3_circle.csv
```

## 批次 4 · cross5（start_5）— 一次起飞跑完 S4/S5/S6，只 `Ctrl+C` 重起 终端4+终端5

### S4 · cross5 · line
```bash
# 终端4
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=line leader_speed:=1.0 max_distance:=20.0
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_line.csv
```

### S5 · cross5 · circle
```bash
# 终端4
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_circle.csv
```

### S6 · cross5 · hover
```bash
# 终端4
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=hover
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_hover.csv
```

## 批次 5 · grid9（start_9）— 一次起飞跑完 S7/S8

### S7 · grid9 · line
```bash
# 终端4
ros2 launch mpc_control swarm_launch.py formation:=grid9 leader_mode:=line leader_speed:=1.0 max_distance:=20.0
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation grid9 --log ~/flights/flight_grid9_line.csv
```

### S8 · grid9 · circle
```bash
# 终端4
ros2 launch mpc_control swarm_launch.py formation:=grid9 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation grid9 --log ~/flights/flight_grid9_circle.csv
```

---

## 每个场景跑完顺手体检（可选，Ubuntu）

```bash
python3 ~/ros2_control_mpc_ws/src/mpc_control/analyze_flight.py ~/flights/flight_<…>.csv
```

## 回传与出图（Windows 仓库根目录）

```powershell
# 1) 把 ~/flights/*.csv 拷到仓库 report/data/（scp / 共享目录 / U 盘 / git 均可）
# 2) 全部出图：
py report/make_figures.py report/data/flight_*.csv --out report/figures
# 3) 或挑 line 系列做跨场景对比：
py report/make_figures.py report/data/flight_*_line.csv --out report/figures --labels solo1,pair2,cross5,grid9
```

产出 → `report/figures/`：`<stem>_panels.png`（六联面板）、`<stem>_traj.png`（俯视轨迹）、`compare_metrics.png`、`compare_spacing.png`、`metrics_table.md`。

> 进度勾选表见 `report/RUN_PLAN_仿真运行清单.md` §5。跑完 CORE（S1–S8）即可支撑报告主体。
