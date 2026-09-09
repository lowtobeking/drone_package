# τ 敏感性 + n≥5 重复实验 运行方案

> 配套 `report/paper_skeleton.md` §6/§9 待办：补 τ 敏感性曲线、补 n≥5 统计。目标机器：**Ubuntu 仿真主机**（`ssh sim`）。
> 对应论文里要填的两个 headline 效应量：无风圆周 track_err **0.368→0.035 m**（trio3-circle）、风扰峰峰 **33.14→0.13 m**（cross5-circle）。

---

## 0. 方法论前提（写实验设置一节时必须照此措辞，不要简化）

1. **本仿真栈没有可控随机种子**。`tools/gen_spawn.py` 生成的"扰动出生"是 `scenarios.yaml` 里手写的固定坐标，不是逐次重采样；代码里唯一的 `random.random()` 只用于 `comms_dropout` 场景（S16/S25），与本实验无关。因此重复试验的方差来源是 **EKF 收敛时机、ROS2/DDS 调度抖动、Gazebo 物理引擎的浮点时序**——这是真实系统噪声，不是种子扫描。论文实验设置一节要写"repeated trials under the SITL stack's inherent timing nondeterminism（no fixed RNG seed control）"，不能写"N seeded Monte Carlo runs"。
2. **重复试验必须每次完整重启**（对照 `RUN_PLAN_仿真运行清单.md` §1.1 的清理流程：`pkill -9` 全套 + 重开 Gazebo/PX4/DDS）。"同队形换轨迹只重启控制器"那条捷径在这里不适用——上一条航迹的终止位置/速度会污染下一条的初值，两次跑就不再是独立样本。这会显著拉长实跑时间，但没有更便宜的正确做法。
3. **改 `vel_lag_tau` 会改变 OCP 结构**，每次换值必须 `rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*` 后 `colcon build --packages-select mpc_control` 重新生成 acados C 代码——τ 敏感性扫描里这是主要耗时步骤，编译时长现场看，本文档不预估具体分钟数。

---

## Part A — n≥5 重复实验（4 组 × 5 次 = 20 跑）

复用已有场景定义，不新增 yaml 条目。

| 组 | 场景 ID | τ | 风 | safety | 说明 |
|---|---|---|---|---|---|
| A1 | `S3_trio3_circle_rh` + `vel_lag_tau:=0.0`（显式钉，默认已转 0.5） | 0（legacy） | 无 | 开 | 无风圆周对照"前" |
| A2 | `S37_trio3_circle_rh_lag` | 0.5 | 无 | 开 | 无风圆周对照"后"（yaml 已钉 0.5） |
| B1 | `S34_cross5_circle_nosafety` | 0（yaml 已钉） | 2N | 关 | 风扰对照"前" |
| B2 | `S36_cross5_circle_nosafety_lag` | 0.5（yaml 已钉） | 2N | 关 | 风扰对照"后" |

### A1/A2（trio3，无风，用 `start_3_px4.sh`，默认 `default.sdf`）

每组跑 5 次，**每次都按 RUN_PLAN §1.1 完整清理重启**：

```bash
# ── 每次重复都从这里开始 ──
for p in px4 'gz sim' gzserver MicroXRCEAgent mpc_node leader_node swarm_launch 'ros2 launch'; do pkill -9 -f "$p"; done
ros2 daemon stop 2>/dev/null; pkill -9 -f 'ros2-daemon'

# T1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf &
sleep 10
# T2
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_3_px4.sh &
# T3
MicroXRCEAgent udp4 -p 8888 &
sleep 5
cd ~/ros2_control_mpc_ws && colcon build --packages-select mpc_control && source install/setup.bash

# T4：A1 用 vel_lag_tau:=0.0；A2 去掉这个覆盖（走 yaml 里已钉的 0.5）
ros2 launch mpc_control swarm_launch.py scenario:=S3_trio3_circle_rh vel_lag_tau:=0.0   &   # A1
# ros2 launch mpc_control swarm_launch.py scenario:=S37_trio3_circle_rh_lag              &   # A2（二选一）

# T5：ready_hold=20s，稳态窗建议 --steady-offset 45（脚本里已按此默认）
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation trio3 \
    --log ~/flights/tau_n5/flight_A1_run{1..5}.csv     # 每次手改 run 号
```

CSV 命名约定：`~/flights/tau_n5/flight_{A1,A2,B1,B2}_run{1..5}.csv`。跑够 ready_hold(20s)+120s 稳态观察即可 Ctrl+C 两个终端。

### B1/B2（cross5，风扰，用 `start_5_px4.sh` + `wind_world.sdf`）

Gazebo 起始命令替换为报告 §5.F 的复现命令：

```bash
gz sim -s -r ~/wind_world.sdf &
sleep 10
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_5_px4.sh &
MicroXRCEAgent udp4 -p 8888 &
sleep 5
cd ~/ros2_control_mpc_ws && colcon build --packages-select mpc_control && source install/setup.bash

ros2 launch mpc_control swarm_launch.py scenario:=S34_cross5_circle_nosafety &          # B1
# ros2 launch mpc_control swarm_launch.py scenario:=S36_cross5_circle_nosafety_lag &     # B2

python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 \
    --log ~/flights/tau_n5/flight_B1_run{1..5}.csv

# 起飞入轨 100s 后（等 T4 日志出现稳态），对每架 x500 施加恒定风力：
for d in x500_N x500_N_1 x500_N_2 x500_N_3 x500_N_4; do   # 5 机模型名按 start_5_px4.sh 里的实际命名核对
  gz topic -t /world/default/wrench/persistent -m gz.msgs.EntityWrench \
    -p "entity: {name: \"${d}::base_link\", type: LINK}, wrench: {force: {x: 2.0}}"
done
```

> ⚠️ 报告 §5.F 原复现命令只写了单机 `x500_N::base_link`；cross5 是 5 机，需要对**每架**模型施加 wrench，模型命名请在 `start_5_px4.sh` 里核对实际值（不同 drone 的模型名后缀可能是 `_1..._4` 或按 drone id 命名，脚本里没有现成的批量施加逻辑，需要照此手动确认后再固化成脚本）。
> 录满 260s（风前 100s + 风后 160s）后 Ctrl+C。

### 数据处理

```bash
python3 tools/repeat_stats.py --group A1 ~/flights/tau_n5/flight_A1_run*.csv --scenario S3_trio3_circle_rh
python3 tools/repeat_stats.py --group A2 ~/flights/tau_n5/flight_A2_run*.csv --scenario S37_trio3_circle_rh_lag
python3 tools/repeat_stats.py --group B1 ~/flights/tau_n5/flight_B1_run*.csv --wind-window 110 258
python3 tools/repeat_stats.py --group B2 ~/flights/tau_n5/flight_B2_run*.csv --wind-window 110 258
```

见 `tools/repeat_stats.py`（本次一并新增），输出每组的 `pos_err` 稳态均值±标准差，以及风扰组额外的窗口内峰峰值±标准差——直接对应论文 §6.2 表格占位。

---

## Part B — τ 敏感性扫描（单跑，基于 A2 场景族）

基础场景固定为 **`S37_trio3_circle_rh_lag`**（无风圆周，headline 数字所在场景），只改 `vel_lag_tau`。网格围绕辨识值 τ≈0.48s（风下 0.61s）与当前默认 0.5 展开，两端各加一个"明显跑偏"参照点：

| τ (s) | 定位 | 说明 |
|---|---|---|
| 0.0 | 复用 A1 数据 | legacy，无需重跑 |
| 0.2 | 欠估计 | |
| 0.3 | 欠估计 | |
| 0.4 | 略欠估计 | |
| 0.48 | 辨识值 | 130Hz 旁路辨识出的实测值本身 |
| 0.5 | 复用 A2 数据 | 当前生产默认，无需重跑 |
| 0.6 | 略高估 | 对应风下辨识值 0.61 |
| 0.8 | 高估 | |

即需新跑 **6 个 τ 值**（0.2/0.3/0.4/0.48/0.6/0.8），每个单跑一次（n=1；若曲线在某点出现明显拐点/非单调，再对该点补 n=3 复核，不必对全网格加密）：

```bash
for TAU in 0.2 0.3 0.4 0.48 0.6 0.8; do
  for p in px4 'gz sim' gzserver MicroXRCEAgent mpc_node leader_node swarm_launch 'ros2 launch'; do pkill -9 -f "$p"; done
  rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*
  gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf &
  sleep 10
  START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_3_px4.sh &
  MicroXRCEAgent udp4 -p 8888 &
  sleep 5
  cd ~/ros2_control_mpc_ws && colcon build --packages-select mpc_control && source install/setup.bash

  ros2 launch mpc_control swarm_launch.py scenario:=S37_trio3_circle_rh_lag vel_lag_tau:=${TAU} &
  python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation trio3 \
      --log ~/flights/tau_sweep/flight_tau${TAU//./p}.csv
  # 跑够 ready_hold(20s)+120s 后手动 Ctrl+C 两个终端，再进入下一个 τ
done
```

### 数据处理

```bash
python3 tools/verdict.py --batch <(for f in ~/flights/tau_sweep/*.csv; do echo -e "$f\tS37_trio3_circle_rh_lag"; done)
```

verdict.py 输出的 `pos_err`（稳态窗均值）即敏感性曲线的 y 轴；连同 Part A 复用的 τ=0/0.5 两点（来自 A1/A2 的 5 次均值），共 8 点，可直接画成论文 §5.G/§6 的 τ-track_err 曲线图。

---

## 预算提示

- Part A：20 次完整重启+跑，每次含清理+Gazebo/PX4/DDS 就绪等待+ready_hold+录制，实际耗时以现场为准，不在此处编造具体分钟数——建议先跑 1 组 5 次摸清单次真实耗时，再决定是否分批。
- Part B：6 次，每次多一道 acados 重新编译，通常比 Part A 单次更慢。
- 建议顺序：先做 Part A（数据直接支撑 headline 效应量的统计显著性，论文里优先级更高），Part B 作为补充曲线，时间不够可先只做 τ∈{0.3, 0.48, 0.6} 三点 + 复用 A1/A2 端点，够画一条"单调收敛于识别值附近"的示意曲线。
