#!/bin/bash
# 无(真)定位 solo1 台架 dry-run —— 注入合成位置跑完整 companion 栈(不解锁)。
#
# 用途：飞场前在 companion(Orin NX / 仿真机)上验证整条链在【真飞控】上工作：
#   fake_vrpn_pose(合成 ENU 位姿) → mocap_bridge → EKF2 → mpc_node(calib→OFFBOARD 就绪)
#   —— 不依赖真动捕、auto_arm:=false 全程不解锁、电机不转。
#
# 前置：EKF2 动捕 5 参数已配 + 飞控已重启；XRCE Agent 在线(systemd)。
#
# 用法：  bash tools/start_dryrun.sh [scenario]     # scenario 默认 IN_solo1_hover
# 停止：  pkill -f fake_vrpn_pose; pkill -f real_hardware_launch
#
# 验证(另开一个 shell，需 source 同样环境)：
#   ros2 topic echo --once /fmu/out/failsafe_flags \
#       --qos-reliability best_effort --qos-durability transient_local | grep offboard_control_signal_lost
#     → 期望 **false**（= setpoint 流达 FC + 位置有效 + OFFBOARD 就绪；无定位时恒 true）
#   grep -E 'calibrated|waiting' /tmp/drone.log
#     → 应见 "calibrated [STABLE] ... world_birth=(0,0,0)" 和 "waiting RC ARM+OFFBOARD"
#
# 2026-07-27 真机(X6 Air+ v1.16 / Orin NX)实测通过。
#
# ⚠️ 注入位置必须在场景出生点 calib_shared_origin_tol(0.5m)内 —— solo1 出生点=(0,0,0)，
#    故注入 ENU(0,0,0)。换场景要相应改注入点。
# ⚠️ 只验证到"OFFBOARD 就绪"；解锁后 OFFBOARD+MPC 闭环跟踪本脚本【不做】
#    (静态开环注入会让 MPC 持续命令爬升、且解锁=电机转)。闭环跟踪走 SITL 或飞场真动捕。
set -u
SCEN="${1:-IN_solo1_hover}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null \
    || source "$HOME/ros2_control_mpc_ws/install/setup.bash" 2>/dev/null
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
# acados 运行时环境：非交互 shell(setsid/systemd/nohup)不读 .bashrc，必须显式导出，
# 否则 mpc_node 起不来 —— OSError: libqpOASES_e.so cannot open shared object file。
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}:$HOME/acados/lib"

# 清理旧进程(脚本文件方式，pattern 不会匹配本脚本 runner，避免 pkill 自杀)
pkill -f 'fake_vrpn_pose'
pkill -f 'real_hardware_launch'
pkill -f 'leader_node'
sleep 2

# setsid 让子进程脱离 ssh 会话，连接断开不会被带走
# 1) 假 VRPN：注入 solo1 出生点 ENU(0,0,0)
setsid python3 "$SCRIPT_DIR/fake_vrpn_pose.py" 0 0 0 > /tmp/fake.log 2>&1 < /dev/null &
sleep 2

# 2) drone 侧：mocap_bridge + mpc_node（不解锁；Agent 走 systemd 故 start_agent:=false）
setsid ros2 launch mpc_control real_hardware_launch.py \
    drone_id:=0 scenario:="$SCEN" \
    start_agent:=false auto_arm:=false \
    > /tmp/drone.log 2>&1 < /dev/null &

echo "launched: fake + drone (scenario=$SCEN)。日志：/tmp/fake.log /tmp/drone.log"
echo "停止：pkill -f fake_vrpn_pose; pkill -f real_hardware_launch"
