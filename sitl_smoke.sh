#!/bin/bash
# sim2 SITL 四件套冒烟：gz(headless) → PX4 → XRCE Agent → 话题核实
# 全程无 DISPLAY，gz 用 -s 只起服务端。日志都在 /tmp/smoke_*.log
PX4_DIR="$HOME/PX4-Autopilot-1.14"
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "=== [1/4] gz sim 服务端(headless) ==="
nohup gz sim -r -s "$PX4_DIR/Tools/simulation/gz/worlds/default.sdf" > /tmp/smoke_gz.log 2>&1 &
echo "  gz PID=$!"; sleep 12
pgrep -f "gz sim" >/dev/null && echo "  gz 在跑" || { echo "  ❌ gz 没起来"; tail -5 /tmp/smoke_gz.log; }

echo "=== [2/4] MicroXRCEAgent ==="
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/smoke_agent.log 2>&1 &
echo "  agent PID=$!"; sleep 3

echo "=== [3/4] PX4 SITL (solo1) ==="
cd "$HOME/Multi-UAV-simulation"
FORMATION=solo1 nohup bash start_1_px4.sh > /tmp/smoke_px4_spawn.log 2>&1 &
sleep 25
echo "--- spawn 日志 ---"; tail -4 /tmp/smoke_px4_spawn.log
echo "--- px4 进程 ---"; pgrep -af "bin/px4" | head -3
echo "--- px4 日志尾 ---"; tail -6 "$HOME/px4_logs/px4_0.log" 2>/dev/null

echo "=== [4/4] ROS2 话题核实 ==="
sleep 5
echo "话题总数: $(ros2 topic list 2>/dev/null | wc -l)"
ros2 topic list 2>/dev/null | grep -E "vehicle_local_position|vehicle_status|vehicle_odometry" | head -5
echo "--- 位置话题是否真在发 ---"
timeout 10 ros2 topic echo /fmu/out/vehicle_local_position --once 2>/dev/null | grep -E "^(x|y|z):" | head -3 || echo "(没收到)"
echo "=== 冒烟结束 ==="
