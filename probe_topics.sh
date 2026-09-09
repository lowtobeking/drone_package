#!/usr/bin/env bash
# probe_topics.sh — start gz+agent+1xPX4, echo key fmu topics, teardown.
set +e
BASE=/home/caolihao/drone_package_20260908
source "$BASE/run_env.sh"
REPO=$BASE/Multi-UAV-simulation-full

pkill -9 -f 'MicroXRCEAgent' 2>/dev/null
pkill -9 -f 'px4 -d -i' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* 2>/dev/null

gz sim -s -r "$PX4_DIR/Tools/simulation/gz/worlds/default.sdf" > /tmp/probe_gz.log 2>&1 &
sleep 8
MicroXRCEAgent udp4 -p 8888 > /tmp/probe_agent.log 2>&1 &
sleep 3
PX4_DIR=$PX4_DIR FORMATION=solo1 bash "$REPO/start_1_px4.sh" > /tmp/probe_spawn.log 2>&1
sleep 25

echo "### OUT topics:"
timeout 15 ros2 topic list 2>/dev/null | grep -E "^/fmu/out" | head -30

echo "### vehicle_status echo (3s):"
timeout 5 ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --once 2>&1 | head -25
echo "### vehicle_control_mode echo (3s):"
timeout 5 ros2 topic echo /fmu/out/vehicle_control_mode --qos-reliability best_effort --once 2>&1 | head -25
echo "### vehicle_local_position (position) echo:"
timeout 5 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>&1 | grep -E "x:|y:|z:|timestamp" | head -8

echo "### px4_0.log tail 20:"
tail -20 "$BASE/px4_logs/px4_0.log"

pkill -9 -f 'px4 -d -i' 2>/dev/null
pkill -9 -f MicroXRCEAgent 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
echo "### probe done"
