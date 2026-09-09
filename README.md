# drone_package

多无人机 PX4 SITL + MPC 工程与编排（`低空仿真的可复现工程主体`）。

> 本仓库只收**工程本身的代码 / 编排脚本 / 自建 world / 文档**。
> PX4-1.16 固件、Micro-XRCE-DDS-Agent、acados、ros2_ws 等**第三方依赖不在此仓库内**（见 §依赖）。

## 目录结构
```
drone_package
├─ Multi-UAV-simulation-full/   # MPC/swarm 业务工程(含 report 文档, 已去 .git/.bak/report数据附件)
├─ gz_overrides/worlds/         # 自建"全系统"world(传感器/NavSat/GUI)
├─ SITL仿真调试记忆_20260908.md # 调试记忆(开跑前先读!)
├─ run_env.sh                   # 环境: 选 PX4-1.16 + 全系统 world + px4_msgs v1.16.2
├─ sitl_{solo,pair,trio3}_check.sh   # 单/双/三机 headless 验收
├─ sitl_solo_gui.sh / sitl_pair_gui.sh / launch_trio3_gui.sh  # GUI 可视化演示
├─ 其他编排/探测脚本                # gzstep/probe_topics/run_scenario_sitl...
└─ mesh_dds.xml / mesh_dds.env   # mesh DDS 单播配置样例
```

## 它是什么
这套代码跑 **PX4 SITL（Gazebo/gz）多机 + MPC 编队**：含单机(solo1)、双机(pair2, 间隔3m)、三机(trio3, 等边三角~5.2m)以及更多队形定义。
改动都在 `Multi-UAV-simulation-full` 内；**仿真与真机共用同一份 `mpc_control`**（真机用 `real_hardware_launch.py`, 仿真用 `swarm_launch.py`）。

## 依赖（不在此仓库，需自行提供/安装）
| 组件 | 版本 | 说明 |
|---|---|---|
| Ubuntu | 24.04 (WSL2) | 4 核 / ~7.8 GB（越大越好） |
| ROS | jazzy | `/opt/ros/jazzy`，RMW=fastrtps，`ROS_DOMAIN_ID=0` |
| Gazebo / gz-sim | Harmonic 8.x | 全系统 world 需要 sensor/navsat 插件 |
| PX4 | **1.16**（`PX4-Autopilot-1.16`, release/1.16） | 默认用 1.16；**不要用 ~/PX4-Autopilot main** |
| PX4 msgs ROS | `px4_msgs` @ **v1.16.2** | 必须与固件 1.16 匹配，否则话题静默不可见 |
| Micro-XRCE-DDS-Agent | — | udp4 :8888 |
| acados | — | 放 `$BASE/acados`（MPC OCP 求解） |

> run_env.sh 会把路径指到相对 `$BASE` 的 `PX4-Autopilot-1.16`、`gz_overrides/worlds/default.sdf` 等。

## 复现入口（先 source）
```bash
BASE=$(pwd)                 # = drone_package 根
source $BASE/run_env.sh     # 打印 PX4_DIR / SITL_WORLD 确认指向正确

bash $BASE/sitl_solo_check.sh    # 单机 headless 验收(Ready+MPC悬停)
bash $BASE/sitl_pair_check.sh    # 双机 headless（本库新增, headless 版 pair）
bash $BASE/sitl_trio3_check.sh   # 三机 headless（本库新增）

bash $BASE/sitl_solo_gui.sh [s]  # 单机 GUI(Windows 桌面看窗口)
bash $BASE/sitl_pair_gui.sh [s]  # 双机 GUI
bash $BASE/launch_trio3_gui.sh   # 三机 GUI（本库新增封装）
```

## 文档地图（更细）
- `SITL仿真调试记忆_20260908.md` —— 开跑/排障第一手，含三级(headless+GUI)验收结论（§8）。
- `Multi-UAV-simulation-full/report/*.md` —— 34+ 篇流程/待办/论文/真机 checklist。
- `链路连接手册.md`（若在顶层）—— mesh/DDS 链路。

## 分支/提交约定
- 本仓库只收工程代码/脚本/文档，**不含固件与第三方依赖**。
- 提交前无需处理子仓库 `.git` —— 全都以普通文件收录（`git rm -r --cached` 排除 `Multi-UAV-simulation-full` 里若出现的嵌套 `.git`）。
