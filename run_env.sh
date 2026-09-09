#!/usr/bin/env bash
# run_env.sh — source this to run the MPC multi-UAV SITL on this host (jazzy + Harmonic + PX4-1.16, px4_msgs main)
# 默认 PX4 树 = BASE/PX4-Autopilot-1.16（其 x500 模型 merge 了带 imu 噪声 + navsat 的
# x500_base，gz_bridge 自带 navsat→sensor_gps 路径，能干净 boot 到 Ready）。
# 可用 PX4_DIR=/path 覆盖（例如切回 $HOME/PX4-Autopilot main，此时需自行保证模型可用）。
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -d "$BASE/PX4-Autopilot-1.16" ]; then
  export PX4_DIR="${PX4_DIR:-$BASE/PX4-Autopilot-1.16}"
else
  export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export ACADOS_SOURCE_DIR="$BASE/acados"
export ACADOS_INSTALL_DIR="$BASE/acados"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}:$BASE/acados/lib"

# 模型/world 一律以当前 PX4 树为准（顺序：PX4_DIR 优先）。
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
# 带全系统插件(Sensors/Imu/NavSat/…)的 world —— SITL 启动 gz 统一用这个，
# 不要直接用 PX4 自带的 default.sdf（1.16 的裸 world 不含这些 system，传感器不发布）。
export SITL_WORLD="$BASE/gz_overrides/worlds/default.sdf"
export GZ_IP=127.0.0.1

# ament/colcon 的 *_setup.sh 内部会引用大量环境变量；当入口脚本开了 `set -u` 时，
# source ROS 会因“未设变量”而直接中断。解决办法：source 期间临时关闭 -u，完后再恢复。
_u_flag="$(case "$-" in *u*) echo u;; *) echo '';; esac)"
set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$BASE/ros2_ws/install/setup.bash" ]; then
  source "$BASE/ros2_ws/install/setup.bash"
fi
if [ "$_u_flag" = "u" ]; then set -u; fi

echo "[run_env] PX4_DIR=$PX4_DIR"
echo "[run_env] SITL_WORLD=$SITL_WORLD"
echo "[run_env] RMW=$RMW_IMPLEMENTATION ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[run_env] ACADOS=$ACADOS_SOURCE_DIR"
