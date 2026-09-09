# 仿真运行清单 / Simulation Run Plan & Checklist

> 配套报告 `report/阶段性报告.md` §4。目标：在 **Ubuntu 主机**按本清单跑多工况、用 `--log` 记 CSV，并采集视频/截图；CSV 回传 **Windows** 后用 `report/make_figures.py` 出图。
> Run scenarios on the **Ubuntu host** with `--log`, capture video/screenshots, then bring CSVs back to **Windows** and plot with `report/make_figures.py`.

约定 / conventions
- 路径以 [F11] 诊断文档为准：包在 `~/ros2_control_mpc_ws/src/mpc_control/`。
- 每个 run 的 CSV 命名：**`flight_<formation>_<traj>[_<变体>].csv`**（变体如 `v2.5`、`wide`、`perturbed`），便于 `make_figures.py` 自动取名。
- 视频/截图命名对应：`cross5_line.mp4` / `cross5_line_gazebo.png`。

---

## 0. 一次性准备 / One‑time prep (Ubuntu)

```bash
# Gazebo 资源路径（建议写进 ~/.bashrc 一劳永逸）
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot-1.14/Tools/simulation/gz/models:~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds

# 存放 CSV 的目录
mkdir -p ~/flights

# 确认拿到最新代码（含本次 diag_monitor 的 x,y 记录列）
cd ~/ros2_control_mpc_ws/src/mpc_control && git pull origin main && git log --oneline -1
```

> ⚠️ 本次改了 `diag_monitor.py`（新增 CSV 列）。`diag_monitor.py` 是**直接 `python3` 运行**，`git pull` 后即生效，**无需 colcon build**。只有改了 `mpc_node.py` 的 **MPC OCP 结构**（horizon/状态/邻居数/约束）才需 `rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*` 后重编。本次没改 OCP 结构。

---

## 1. 每个队形的底座启动 / Per‑formation bring‑up

> 同一队形下做不同轨迹时，**Gazebo / PX4 / DDS 桥可保持不动**，只重启第 5、6 步（控制器 + 诊断）。换队形（机数变化）才需从头清理重启。

```bash
# 1.1 清理残留（务必清干净；gz server 最易残留）
for p in px4 'gz sim' gzserver MicroXRCEAgent mpc_node leader_node swarm_launch 'ros2 launch'; do pkill -9 -f "$p"; done
ros2 daemon stop 2>/dev/null; pkill -9 -f 'ros2-daemon'
ps aux | grep -E "px4|gz sim|gzserver|MicroXRCE|mpc_node|leader_node|swarm_launch" | grep -v grep   # 应零输出

# 1.2 终端1：Gazebo（窗口出现后再等 ~10s）
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

# 1.3 终端2：PX4 实例（按队形选脚本：见下表）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_<N>_px4.sh

# 1.4 终端3：DDS 桥
MicroXRCEAgent udp4 -p 8888

# 1.5 终端4：编译并 source（首次或改过被编译的 .py 后）
cd ~/ros2_control_mpc_ws && colcon build --packages-select mpc_control && source install/setup.bash
```

| 队形 formation | PX4 脚本 start_\<N\> | diag `--formation` |
|---|---|---|
| solo1 | `start_1_px4.sh` | solo1 |
| pair2 | `start_2_px4.sh` | pair2 |
| trio3 | `start_3_px4.sh` | trio3 |
| cross5 / star5 | `start_5_px4.sh` | cross5 / star5 |
| grid9 | `start_9_px4.sh` | grid9 |

---

## 2. 逐场景命令 / Per‑scenario commands

> 每个场景两条命令：**终端4** 起控制器 + 领队；**终端5** 起诊断并记 CSV。
> 领队默认**闭环就绪门控**：全员进编队后自动开动，无需手数延时。
> 跑够时长（瞬态收敛 + 稳态观察；hover/line ≥ 120 s，circle ≥ 1–2 圈或 ≥ 120 s）后，两个终端都 `Ctrl+C`。

### CORE（建议必跑）

**S1 · solo1 · line**
```bash
# T4
ros2 launch mpc_control swarm_launch.py formation:=solo1 leader_mode:=line leader_speed:=1.0 max_distance:=20.0
# T5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation solo1 --log ~/flights/flight_solo1_line.csv
```

**S2 · pair2 · line**
```bash
ros2 launch mpc_control swarm_launch.py formation:=pair2 leader_mode:=line leader_speed:=0.5 max_distance:=20.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation pair2 --log ~/flights/flight_pair2_line.csv
```

**S3 · trio3 · circle**
```bash
ros2 launch mpc_control swarm_launch.py formation:=trio3 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation trio3 --log ~/flights/flight_trio3_circle.csv
```

**S4 · cross5 · line**
```bash
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=line leader_speed:=1.0 max_distance:=20.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_line.csv
```

**S5 · cross5 · circle**
```bash
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_circle.csv
```

**S6 · cross5 · hover**
```bash
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=hover
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_hover.csv
```

**S7 · grid9 · line**（先 `start_9_px4.sh`）
```bash
ros2 launch mpc_control swarm_launch.py formation:=grid9 leader_mode:=line leader_speed:=1.0 max_distance:=20.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation grid9 --log ~/flights/flight_grid9_line.csv
```

**S8 · grid9 · circle**
```bash
ros2 launch mpc_control swarm_launch.py formation:=grid9 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation grid9 --log ~/flights/flight_grid9_circle.csv
```

### EXT（可选加强）

**S9 · star5 · line**（`start_5_px4.sh`）
```bash
ros2 launch mpc_control swarm_launch.py formation:=star5 leader_mode:=line leader_speed:=1.0 max_distance:=20.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation star5 --log ~/flights/flight_star5_line.csv
```

**S10 · cross5 · line · 高速应力 v=2.5**
```bash
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=line leader_speed:=2.5 max_distance:=30.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_line_v2.5.csv
```

**S13 · cross5 · circle · 长时 10 min**（记录后期是否漂移）
```bash
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_circle_10min.csv
# 跑满 ~600 s 再停
```

**S14 · cross5 · line · 失联降级**（仿真专用：稳定后 kill 一台僚机的 mpc_node）
```bash
# 起 S4 同款；稳定后在另一终端：
pkill -9 -f 'mpc_node.*px4_1'    # 杀 drone1 控制器，观察 drone0 是否在 2s 内降级 hover、最小间距是否保持
# 记录到同一 CSV：flight_cross5_line_killnode.csv（启动 diag 时用此名）
```

### EXT · 需改代码的场景（Windows 改 → push → Ubuntu pull）

> 这两类改源码（出生位置 / 间距），按工作流在 **Windows 改 `launch/swarm_launch.py` → `/commit-push` → Ubuntu `git pull` → colcon build**。

**S11 · cross5 · 扰动出生位置**（看从打乱初值收敛到队形）
- 改 [F3] `BIRTH_5`（把各机起点打乱，仍保证两两 > d_safe），并**同步**改 `start_5_px4.sh` 的 `POSES`（ENU，需与 NED 对应：`NED_x=ENU_y, NED_y=ENU_x`）。`OFFSETS_CROSS5` 保持十字不变 → 各机需从扰动起点飞到十字位。
- CSV 名：`flight_cross5_line_perturbed.csv`。

**S12 · cross5 · 加宽间距 ~5 m**
- 改 [F3] `OFFSETS_CROSS5` 把臂长 3 → 5（如 `[0,5],[0,-5],[5,0],[-5,0]`）。`desired_distances` 会自动跟着变；出生 `BIRTH_5`/POSES 也相应放大或保持（保持则看从 3 m 扩到 5 m 的收敛）。
- CSV 名：`flight_cross5_line_wide.csv`。

---

## 3. 每个场景要采集什么 / What to capture per scenario

- [ ] **CSV**（必须）：`~/flights/flight_<…>.csv`（diag `--log` 自动写）。
- [ ] **视频**（你来）：Gazebo 视角录屏，命名 `<formation>_<traj>.mp4`。建议含起飞→成型→机动→（到点/绕圈）全过程。
- [ ] **截图**（你来）：编队成型瞬间 + 机动中各一张，`<formation>_<traj>_gazebo.png`。
- [ ] **诊断面板截图**（可选）：`diag_monitor` 终端稳态一屏，佐证 status/间距/求解。

> 跑完可顺手出一份文本体检：`python3 …/analyze_flight.py ~/flights/flight_<…>.csv`（[F5]）。

---

## 4. 数据回传与出图 / Bring data back & make figures

```bash
# 4.1 Ubuntu → Windows：把 ~/flights/*.csv 拷到仓库 report/data/
#     （scp / 共享目录 / U 盘 / git 均可。CSV 不大，git 也行。）

# 4.2 Windows（仓库根目录）：单个场景出图
py report/make_figures.py report/data/flight_cross5_line.csv --out report/figures

# 4.3 多场景对比出图（一次传多个 CSV）
py report/make_figures.py report/data/flight_*line*.csv --out report/figures --labels solo1,pair2,cross5,grid9
```

产出（→ `report/figures/`）：`<stem>_panels.png`、`<stem>_traj.png`、`compare_metrics.png`、`compare_spacing.png`、`metrics_table.md`。
然后把图按 `report/阶段性报告.md` §5.B 的占位插入，分析文字随后补。

---

## 5. 进度勾选表 / Progress checklist

| ID | 场景 | 已跑 | CSV | 视频 | 截图 | 出图 |
|---|---|:--:|:--:|:--:|:--:|:--:|
| S1 | solo1 · line | ☐ | ☐ | ☐ | ☐ | ☐ |
| S2 | pair2 · line | ☐ | ☐ | ☐ | ☐ | ☐ |
| S3 | trio3 · circle | ☐ | ☐ | ☐ | ☐ | ☐ |
| S4 | cross5 · line | ☐ | ☐ | ☐ | ☐ | ☐ |
| S5 | cross5 · circle | ☐ | ☐ | ☐ | ☐ | ☐ |
| S6 | cross5 · hover | ☐ | ☐ | ☐ | ☐ | ☐ |
| S7 | grid9 · line | ☐ | ☐ | ☐ | ☐ | ☐ |
| S8 | grid9 · circle | ☐ | ☐ | ☐ | ☐ | ☐ |
| S9 | star5 · line | ☐ | ☐ | ☐ | ☐ | ☐ |
| S10 | cross5 · line v2.5 | ☐ | ☐ | ☐ | ☐ | ☐ |
| S11 | cross5 · 扰动出生 | ☐ | ☐ | ☐ | ☐ | ☐ |
| S12 | cross5 · 加宽间距 | ☐ | ☐ | ☐ | ☐ | ☐ |
| S13 | cross5 · circle 10min | ☐ | ☐ | ☐ | ☐ | ☐ |
| S14 | cross5 · 失联降级 | ☐ | ☐ | ☐ | ☐ | ☐ |

> 跑完 CORE（S1–S8）即可支撑报告主体；EXT 用于加强稳定性论证。
