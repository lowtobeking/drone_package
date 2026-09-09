# Multi-UAV Formation Simulation · 多机编队仿真

PX4 v1.14 + ROS 2 + acados MPC 的多无人机编队控制仿真。渐进式验证 **单机 → 2/3 机 → 5 机(十字/星形) → 9 机(3×3 方阵)**：领队驱动 `hover / line / circle` 三种机动，僚机用**分布式 MPC** 跟踪编队并避碰。

> 开发流：Windows 编辑 → GitHub 同步 → Ubuntu(Gazebo SITL) 仿真。仓库根目录即 ROS 2 包 `src/mpc_control/`。

## 亮点

- **分布式 MPC**：每机一个 acados OCP（SQP-RTI 单次迭代，solve 0.13–0.32 ms），交换邻居预测轨迹实现编队 + 软碰撞约束；邻居稀疏化（每机 ≤4），队形扩展不增状态维度。
- **速度控制模式**：下发 MPC 预测速度作 setpoint（已拆外层无阻尼 P 环、消除圆周震荡），高度纯 P 保持。
- **世界系 NED 统一 + EKF 跳变补偿**：`world_birth` 动态补偿 PX4 EKF 重置，解决多机世界系不一致导致的塌缩/碰撞。
- **配置即真值**：`config/scenarios.yaml` 单一真值源驱动队形几何 / 参数 / 测试工况（S1–S26）；新增队形或工况改 yaml、不动控制器。
- **双栈交叉验证**：同一 MPC 移植进 Simulink（本机无优化工具箱→自写 ADMM 替 HPIPM）独立复算，与 SITL 同判，作为代码准确性的独立证据。
- **companion 安全层**：`safety_filter.py` 飞散围栏 / 硬碰撞地板 / 估计健康门 / 失效状态机（异常停发 setpoint 交还 PX4），下发前拦截。

## 仓库结构

| 路径 | 说明 |
|---|---|
| `mpc_control/` | ROS 2 节点：`mpc_node`（分布式 MPC）/ `leader_node` / `virtual_leader_node` / `arming_node` / `safety_filter` |
| `launch/` | `swarm_launch.py`（yaml 驱动）、`real_hardware_launch.py`（真机部署） |
| `config/scenarios.yaml` | **单一真值源**：队形几何 + 公共参数 + 测试工况 |
| `tools/` | `gen_spawn.py`（yaml → PX4 出生点）、`spawn_px4.sh` |
| `start_{1,2,3,5,9}_px4.sh` | 按机数启动 PX4 实例 |
| `diag_monitor.py` / `analyze_flight.py` | 实时诊断面板 / 飞行体检报告 |
| `report/` | 阶段性报告（中英双语）、运行清单、出图脚本、Simulink 移植与验证 |

## 快速开始（Ubuntu）

完整分阶段命令见 `report/CORE_run_commands.md`（照着贴的精简版）与 `CLAUDE.md`「Ubuntu 端构建与启动顺序」。最小回路（5 机十字·直线）：

```bash
# 终端1 Gazebo
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2 PX4 实例
START_DELAY=5 bash start_5_px4.sh
# 终端3 DDS 桥
MicroXRCEAgent udp4 -p 8888
# 终端4 控制器（colcon build 后）
ros2 launch mpc_control swarm_launch.py scenario:=S4_cross5_line
# 终端5 诊断 + 记 CSV
python3 diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_line.csv
```

## 结果与报告

原理推导 / 架构 / 仿真设计 / 结果见 `report/阶段性报告.md`。Simulink-MPC 双栈交叉验证见 `report/simulink_formation/`；真机安全配置见 `report/真机安全配置清单_FC.md`；飞行 CSV 数据仓 → [flights-](https://github.com/Aaron6099/flights-)。

## 开发约定

- 代码唯一源头 = `mpc_control/` + `launch/`；**勿在根目录另存节点副本**。
- 新增队形/工况只改 `config/scenarios.yaml`；改后 Ubuntu 端需 `colcon build`（launch 读 install 安装副本）。
- 改 MPC 结构（horizon / 状态 / 邻居数）后必须 `rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*`。
- 更多细节见 `CLAUDE.md`。
