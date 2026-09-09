#!/usr/bin/env bash
set +e
BASE=/home/caolihao/drone_package_20260908
export PX4_DIR=$BASE/PX4-Autopilot-1.16
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
export GZ_IP=127.0.0.1
pkill -9 -f 'gz sim' 2>/dev/null
sleep 2; rm -f /dev/shm/fastrtps_* 2>/dev/null
gz sim -s -r "$PX4_DIR/Tools/simulation/gz/worlds/default.sdf" > /tmp/gzstep.log 2>&1 &
sleep 8
echo "### clock sample 1:"; timeout 5 gz topic -e -t /world/default/clock 2>/dev/null | head -6
sleep 3
echo "### clock sample 2 (should advance ~3s if stepping):"; timeout 5 gz topic -e -t /world/default/clock 2>/dev/null | head -6
echo "### sim/real fields full sample:"; timeout 5 gz topic -e -t /world/default/clock 2>/dev/null | grep -A0 "sim {" | head -4
echo "### world stats (iterations) sample:"; timeout 5 gz topic -e -t /world/default/world_stats 2>/dev/null | grep -iE "iterations|sim_time|real_time|paused" | head -8
pkill -9 -f 'gz sim' 2>/dev/null
echo done
