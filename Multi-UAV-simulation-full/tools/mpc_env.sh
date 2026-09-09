#!/usr/bin/env bash
# 机载电脑上跑 mpc_control 所需的完整环境。**任何非交互场景都必须先 source 它。**
#
#     source ~/Multi-UAV-simulation/tools/mpc_env.sh
#     ros2 launch mpc_control real_hardware_launch.py ...
#
# ── 为什么需要这个文件（2026-08-20 台架空跑实测踩到）───────────────────────────
# acados 的环境变量定义在 `~/.bashrc` 第 119+ 行，而 Ubuntu 默认 `.bashrc` 开头有：
#     # If not running interactively, don't do anything
#     case $- in  *i*) ;;  *) return;;  esac        # ← 第 5-9 行
# `ssh host '命令'` / systemd / 任何脚本都是**非交互 shell**，读到这里就 return，
# 根本走不到第 119 行 ⇒ 节点启动即崩：
#     OSError: libqpOASES_e.so: cannot open shared object file: No such file or directory
#     Warning: Did not find environment variable ACADOS_SOURCE_DIR
# 实测对照：`bash -ic` 拿得到变量、`bash -lc` 全空。
#
# ⚠️ **不要**把 export 挪到 `.bashrc` guard 之前来"修"这个问题——那样每个非交互 shell
#    （含 scp / git-over-ssh）都会 source ROS，输出污染会直接弄坏 scp/sftp 传输。
# ⚠️ systemd unit 里同样读不到 `.bashrc`，要么 `ExecStart=/bin/bash -c 'source 本文件 && ...'`，
#    要么用 `Environment=` 逐条显式写（见 deploy/microxrce-agent.service 的做法）。

# ROS 2 基础环境
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

# acados（与 ~/.bashrc 119-124 行保持一致）
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}:$HOME/acados/lib"
export PYTHONPATH="/usr/local/lib/python3.10/site-packages/:${PYTHONPATH}"

# 多机编队 DDS：全员必须一致（域号两侧都要设，只设 companion 侧会静默失效）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 自检：缺任何一项都直接报出来，别等节点崩了才发现
if [ ! -f "$ACADOS_SOURCE_DIR/lib/libqpOASES_e.so" ]; then
    echo "⚠️  找不到 $ACADOS_SOURCE_DIR/lib/libqpOASES_e.so —— acados 未安装或路径不对" >&2
fi
