#!/usr/bin/env bash
set +e
BASE=/home/caolihao/drone_package_20260908
export PX4_DIR=$BASE/PX4-Autopilot-1.16
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
export GZ_IP=127.0.0.1
export PX4_GZ_WORLD=default
pkill -9 -f 'px4|gz sim' 2>/dev/null
sleep 3; rm -f /dev/shm/fastrtps_* 2>/dev/null
cd "$PX4_DIR"
echo "### run make px4_sitl gz_x500 (85s)"
timeout 85 make px4_sitl gz_x500 > /tmp/make_gz.log 2>&1 &
sleep 75
echo "### gz topics imu?"; timeout 8 gz topic -l 2>/dev/null | grep -cE "sensor/imu_sensor/imu"
echo "### health in log:"; grep -aE "Preflight Fail|sensor_imu|sensor_mag|ekf2|Accel|Ready for takeoff|armed|Arming" /tmp/make_gz.log | head -20
echo "### px4 bin running?"; ps aux | grep "[b]in/px4" | head -2
echo "### tail make log:"; tail -15 /tmp/make_gz.log
pkill -9 -f 'px4|gz sim' 2>/dev/null
echo done
