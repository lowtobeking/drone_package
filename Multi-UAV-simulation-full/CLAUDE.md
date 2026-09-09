# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

PX4 v1.14 多机编队仿真，5/9 架无人机，ROS2 + acados MPC 控制。
代码在 **Windows 桌面编辑**，通过 GitHub 同步到 **Ubuntu 主机**进行仿真。

## 代码结构（唯一源头，勿再产生副本）

标准 ROS2 ament_python 包，仓库根目录即 Ubuntu 上的 `src/mpc_control/`：

- `mpc_control/`（Python 模块）：`mpc_node.py` / `leader_node.py` / `virtual_leader_node.py` / `arming_node.py` —— **`ros2 launch` 实际运行的就是这里**
- `launch/swarm_launch.py`：启动入口（**yaml 驱动**，唯一 launch 文件）。队形几何 / 公共参数 / 测试工况均来自 `config/scenarios.yaml`，自 P1 起已删硬编码的 `COMMON`/`OFFSETS_*`/`NBR_*`
- `config/scenarios.yaml`：**单一真值源**（defaults 公共参数 / formations 队形几何 / scenarios 工况 S1–S26）；`config/swarm_config.yaml` 为节点参数文件
- 根目录：`start_{1,2,3,5,9}_px4.sh`（bash 直接跑）、`diag_monitor.py`（python3 直接跑）、`setup.py`/`package.xml`

> 编辑代码只改 `mpc_control/` 模块和 `launch/`；**绝不在根目录另存 `mpc_node.py`/`leader_node.py`/`swarm_launch.py` 副本**。2026-05-29 已清理一批重构前的过时副本——根目录那些不会被 ROS2 编译，曾导致"改了没生效"。提交一律用 `/commit-push`（`git add -A`）。

## 坐标系（极易出错）

| 系统 | 坐标轴 | 说明 |
|------|--------|------|
| PX4 NED | x=北, y=东, z=下 | 所有 setpoint、position 话题均用此系 |
| Gazebo ENU | x=东, y=北, z=上 | 换算：NED_x=ENU_y, NED_y=ENU_x |

内部统一用 **世界系 NED**：`ds.pos = local_pos + world_birth[drone_id]`
发给 PX4 前须转回本地系：`pos_local = pos_world - world_birth[drone_id]`

## 控制架构

> ⚠️ 当前为**速度控制模式**（自 commit 32759f6 / 3f7bbe8）。早期文档写的 position 闭环已废弃，**勿回退**（诊断文档已修复 bug#6）。

- **控制模式**：`OffboardControlMode.velocity=True`（速度闭环）
- **Setpoint**：`TrajectorySetpoint.velocity`；**XY = MPC 预测速度（已拆外层无阻尼 P 环，1c4bc5a，消除圆周震荡）**，Z 保留纯 P 保持高度；`.position=NaN`
- **MPC 输出**：取 `x_pred[1, 3:6]`（预测速度）作为速度设定点基准
- **高度**：纯 P 控制器保持 `target_alt`
- **降级策略**：任何异常（解失败 / 偏差 >5m / NaN）→ `_hover_setpoint_world()` 悬停

## 关键参数位置

所有运行参数集中在 `config/scenarios.yaml` 的 `defaults`（旧 `swarm_launch.py` 的 `COMMON` 字典已迁移至此），修改后 `colcon build` 即可，无需改 `mpc_node.py`。

重要参数：
- `max_speed`: 3.0 m/s（速度控制模式下的速度设定点限幅）
- `neighbour_timeout`: 2.0 s（邻居通信超时容忍）
- `target_alt`: -5.0（NED，负值=向上，离地 5 米）
- `d_safe`: 1.5 m（碰撞软约束距离）

新增参数（2026-06-03，均在 `swarm_launch.py`/节点默认值）：
- **标定硬化**（Tier1，30915f2）：`calib_max_origin_offset=2.0`（校准锁定前近原点门控；未收敛不硬锁，打红字 `calib STUCK` 守出生点）
- **高度全员基准 re-sync**（Tier2，978ab9e）：`alt_resync_enable=True`/`alt_resync_rate=0.05`/`alt_ref_filter_alpha=0.05`/`alt_resync_max=3.0`（drone0 广播当前 ref_alt 到 `/swarm/alt_datum`，各机限速纠 `world_birth_z`，治 ref_alt 连续温漂）
- **直线终点减速**（9f6bed6）：`line_decel=0.5` m/s²（梯形速度曲线缓停，防到点过冲）
- **就绪门控**（af59b66）：`ready_gate_enable=True`/`ready_pos_err=0.5`/`ready_hold=2.0`/`ready_timeout=90.0`/`health_timeout=2.0`（leader 等全员进编队才开动，替代固定 `start_delay`）；`num_drones` 由 launch 传入

## 话题命名规则

- drone 0：`/fmu/in/...`、`/fmu/out/...`
- drone 1-8：`/px4_{id}/fmu/in/...`、`/px4_{id}/fmu/out/...`
- 领队：`/leader/state`（**Float64MultiArray**，格式 `[t, x, y, z, vx, vy, vz, yaw]`；曾用 Float32 精度不足致漂移，已修复 bug#4，勿回退）
- MPC 预测轨迹：`/mpc/predicted_trajectory`（drone 0）或 `/px4_{id}/mpc/predicted_trajectory`

## 队形配置

| 队形 | 无人机数 | 拓扑 | 间距 |
|------|----------|------|------|
| cross5 | 5 | 十字，中心连四臂 | 3 m |
| star5 | 5 | 正五边形，环形邻居 | R=3 m |
| grid9 | 9 | 3×3 方阵 | 3 m |

队形偏移 / 邻居定义在 `config/scenarios.yaml` 的 `formations.<name>`（`offsets` / `neighbours`）；旧 `swarm_launch.py` 的 `OFFSETS_*`/`NBR_*` 已迁移至此。

## Ubuntu 端构建与启动顺序

```bash
# 1. 清理残留（用 -9 强制杀死，避免 MPC 节点残留造成命令冲突）
pkill -9 -f px4; pkill -9 -f gz; pkill -9 -f MicroXRCEAgent; pkill -9 -f ros2

# 2. 终端1：Gazebo（先设 GZ_SIM_RESOURCE_PATH，见诊断文档步骤0）
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

# 3. 终端2：PX4（等 Gazebo 就绪后；按队形选脚本：5机用 start_5，9机用 start_9）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_5_px4.sh

# 4. 终端3：DDS 桥
MicroXRCEAgent udp4 -p 8888

# 5. 终端4：编译并启动（从 GitHub 拉取新代码后执行）
cd ~/ros2_control_mpc_ws/src/mpc_control && git pull origin main  # 实际代码在 src/mpc_control
cd ~/ros2_control_mpc_ws
rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*     # 改了 MPC 结构时必须清缓存
colcon build --packages-select mpc_control
source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=cross5
```

> 完整分阶段启动命令（solo1→grid9）见 `/ubuntu-deploy` skill 或诊断文档「启动顺序」。

**就绪门控**：leader 不再死等固定 `start_delay`，等全员进编队（pos_err<0.5m 保持 2s）才自动开始运动，日志 `formation ready — starting`，90s 超时兜底（`ready_gate_enable:=false` 退回固定延时）。

**诊断与飞行记录**：`python3 diag_monitor.py --formation <队形>` 看实时面板；加 `--log` 每秒写 CSV，跑完 `python3 analyze_flight.py <csv> [--plot]` 出体检报告（pos_err/高度误差、最小间距+时刻、违规、solve、编队成型时间）。CSV 含各机 `x,y`(世界系 NED) 与 `leader_x,y,vx,vy`，支持俯视轨迹图（纯增列、向后兼容）。

**阶段性报告与出图**：`report/` 下有 `阶段性报告.md`（原理/架构/设计/结果，中英双语）、`RUN_PLAN_仿真运行清单.md`（逐场景命令）、`make_figures.py`（读 CSV 出面板图/轨迹图/多 run 对比/`metrics_table.md`）。出图：`py report/make_figures.py report/data/<csv> --out report/figures`。

## acados 编译缓存

每架无人机首次运行会编译 C 代码到 `~/.cache/mpc_control/acados_di_mpc_{tag}_v{drone_id}_m{neighbours_count}/`
（路径由 `config/scenarios.yaml` 的 `acados_build_dir` 配置，`mpc_node` 会做 expanduser）。

> **2026-07-21 从 `/tmp` 迁到 home**：Jetson 的 systemd-tmpfiles 配置为 `D /tmp`，
> **每次开机清空 `/tmp`** → 台架预编译的生成码撑不过一次断电，飞场每次上电都要现编数分钟
> （且每个 drone_id×邻居数组合各一份）。放在 home 下可跨重启命中缓存。
> 下面的清缓存命令同时清新旧两个路径，兼顾仍有 `/tmp` 残留的机器。

**修改 MPC 结构（horizon N、状态维度、邻居数）后必须删除该目录**，否则使用旧缓存导致崩溃：
```bash
rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*
```

## 已知陷阱

- **EKF 重置**：`world_birth` 数组动态补偿 EKF 跳变；直接用 `ds.pos` 已是世界系，勿再加偏移
- **5 机模式**：drone 5-8 在 Gazebo 中可见但停在地面，属正常现象
- **MPC 求解器**：`SQP_RTI` 单次迭代，`nlp_solver_max_iter=30` 不影响此模式；修改结构需重新编译 acados
- **启动顺序**：Gazebo 必须先于 PX4 实例启动，否则无人机模型无法加载

## 开发工作流

```
Windows 桌面编辑代码
    → /commit-push 提交推送到 GitHub
    → Ubuntu: git pull → colcon build → 仿真验证
```
