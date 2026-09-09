#!/usr/bin/env bash
# sitl_solo_check.sh — PX4-1.16 SITL 单机验收：干净启动→Ready→MPC solo hover 解锁/起飞。
# 依赖：BASE/gz_overrides/worlds/default.sdf（全系统 world），PX4_DIR=PX4-Autopilot-1.16。
# 用法: bash sitl_solo_check.sh
set -u
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO=$BASE/Multi-UAV-simulation-full
D="$HOME/sitl_solo_check"
mkdir -p "$D"
export GZ_IP=127.0.0.1
source "$BASE/run_env.sh"
HP=$PX4_DIR
LOG="$HOME/px4_logs/px4_0.log"

echo "### [0] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
sleep 3
rm -f /dev/shm/fastrtps_*; rm -rf "$D"/*; rm -f "$LOG"

echo "### [1] gz($SITL_WORLD)+agent+px4(solo1)"
gz sim -s -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 15
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3
cd "$HP" || exit 1
PX4_DIR=$HP FORMATION=solo1 bash "$REPO/tools/spawn_px4.sh" > "$D/spawn.log" 2>&1
for i in $(seq 1 30); do
  timeout 8 gz model --list 2>/dev/null | grep -q x500_0 && break
  sleep 2
done

echo "### [2] wait Ready (<=60s)"
R=""
for i in $(seq 1 12); do
  sleep 5
  if grep -aq "Ready for takeoff" "$LOG" 2>/dev/null; then R=$((i*5)); echo "READY at ~${R}s"; break; fi
done
[ -z "$R" ] && { echo "NO READY:"; grep -aE "Preflight Fail|missing|Gyro|Ready" "$LOG" | tail -8; }

echo "### [3] MPC solo hover (30s) —— 期望 Armed→Takeoff→~5m 悬停"
timeout 40 ros2 launch mpc_control swarm_launch.py scenario:=S0_solo1_hover > "$D/mpc.log" 2>&1 &
sleep 30
echo "--- local pos ---"
timeout 5 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE "xy_valid|z: |vx:|vz:" | head -8
echo "--- px4 events ---"
grep -aE "Ready for takeoff|Armed by|Arming denied|Takeoff detected|Preflight Fail" "$LOG" | tail -8
echo "--- mpc tail ---"
tail -6 "$D/mpc.log"
echo "### [4] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
echo "SITL-SOLO-CHECK-DONE logs: $D"
