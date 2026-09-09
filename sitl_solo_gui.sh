#!/usr/bin/env bash
# sitl_solo_gui.sh — PX4-1.16 可视化演示：Gazebo GUI + agent + px4(solo1) + MPC 起飞悬停。
# 打开后请到 Windows 桌面看 Gazebo 窗口（WSLg）。跑完自动清理。
# 用法: bash sitl_solo_gui.sh [keep_s]
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO=$BASE/Multi-UAV-simulation-full
KEEP="${1:-45}"        # MPC 悬停保持秒数
D="$HOME/sitl_gui"
mkdir -p "$D"
export GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"  # WSLg 兼容兜底
source "$BASE/run_env.sh"
HP=$PX4_DIR
LOG="$HOME/px4_logs/px4_0.log"

echo "### [0] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
sleep 3
rm -f /dev/shm/fastrtps_*; rm -rf "$D"/*; rm -f "$LOG"

echo "### [1] gz GUI 启动: $SITL_WORLD"
gz sim -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 18
echo "  gz log 头: "; head -5 "$D/gz.log" 2>/dev/null

echo "### [2] agent + px4(solo1)"
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3
cd "$HP" || exit 1
PX4_DIR=$HP FORMATION=solo1 bash "$REPO/tools/spawn_px4.sh" > "$D/spawn.log" 2>&1
for i in $(seq 1 30); do
  timeout 8 gz model --list 2>/dev/null | grep -q x500_0 && { echo "  x500_0 已生成 ~$((i*2))s"; break; }
  sleep 2
done

echo "### [3] 等 Ready"
R=""
for i in $(seq 1 12); do
  sleep 5
  if grep -aq "Ready for takeoff" "$LOG" 2>/dev/null; then R=$((i*5)); echo "  READY at ~${R}s"; break; fi
done
[ -z "$R" ] && grep -aE "Preflight|missing|Gyro|Ready" "$LOG" | tail -6

echo "### [4] MPC solo hover（约 ${KEEP}s）—— 看窗口里起飞"
timeout $((KEEP+15)) ros2 launch mpc_control swarm_launch.py scenario:=S0_solo1_hover > "$D/mpc.log" 2>&1 &
for tt in 18 30; do
  sleep $tt
  echo "  t+$tt s ---"
  timeout 4 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE "z: |xy_valid|vx:" | head -6
  grep -aE "acados OCP ready|已离地|OFFBOARD confirmed|retry ARM" "$D/mpc.log" | tail -2
done
sleep $((KEEP-30))
echo "  t+$KEEP s 终值:"
timeout 4 ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort --once 2>/dev/null | grep -aE "arming_state|nav_state:" | head -4
timeout 4 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE "z: |xy_valid" | head -4
echo "### px4 事件:"
grep -aE "Ready for takeoff|Armed by|Takeoff detected|Preflight Fail|Disarmed" "$LOG" | tail -8
echo "### [5] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
echo "GUI-DEMO-DONE logs: $D"
