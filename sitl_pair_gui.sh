#!/usr/bin/env bash
# sitl_pair_gui.sh — PX4-1.16 pair2 双机可视化演示：Gazebo GUI + 2×px4 + MPC 编队起飞悬停。
# 用法: bash sitl_pair_gui.sh [keep_s]
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO=$BASE/Multi-UAV-simulation-full
KEEP="${1:-60}"        # 双机悬停保持秒数
D="$HOME/sitl_pair"
mkdir -p "$D"
export GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export START_DELAY="${START_DELAY:-12}"   # 两架 PX4 spawn 间隔
source "$BASE/run_env.sh"
HP=$PX4_DIR
L0="$HOME/px4_logs/px4_0.log"
L1="$HOME/px4_logs/px4_1.log"

echo "### [0] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
sleep 3
rm -f /dev/shm/fastrtps_*; rm -rf "$D"/*; rm -f "$L0" "$L1"

echo "### [1] gz GUI 启动: $SITL_WORLD"
gz sim -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 18

echo "### [2] agent + px4 x2 (pair2)"
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3
cd "$HP" || exit 1
PX4_DIR=$HP FORMATION=pair2 bash "$REPO/tools/spawn_px4.sh" > "$D/spawn.log" 2>&1
for i in $(seq 1 40); do
  n=$(timeout 8 gz model --list 2>/dev/null | grep -c x500_)
  [ "$n" -ge 2 ] && { echo "  x500_0/x500_1 已生成 ~$((i*2))s"; break; }
  sleep 2
done

echo "### [3] 等双机 Ready (<=150s)"
for i in $(seq 1 30); do
  sleep 5
  r0=$(grep -ac "Ready for takeoff" "$L0" 2>/dev/null)
  r1=$(grep -ac "Ready for takeoff" "$L1" 2>/dev/null)
  if [ "$r0" -ge 1 ] && [ "$r1" -ge 1 ]; then echo "  双机 READY at ~$((i*5))s"; break; fi
done
echo "  drone0 marks: $(grep -aE 'Ready|missing|Gyro|Preflight' "$L0" | tail -2)"
echo "  drone1 marks: $(grep -aE 'Ready|missing|Gyro|Preflight' "$L1" | tail -2)"

echo "### [4] MPC pair2 hover（约 ${KEEP}s；drone1 首次需现场编 acados）—— 看窗口双机起飞"
timeout $((KEEP+150)) ros2 launch mpc_control swarm_launch.py scenario:=S2_pair2_hover > "$D/mpc.log" 2>&1 &
for tt in 30 60 90 120; do
  sleep 30
  echo "  --- t+$tt ---"
  echo "   acados: $(grep -ac 'acados OCP ready' "$D/mpc.log") ; 已离地: $(grep -ac '已离地' "$D/mpc.log") ; retry: $(grep -ac 'retry ARM' "$D/mpc.log")"
  timeout 4 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE "x: |y: |z: |xy_valid" | head -8
  timeout 4 ros2 topic echo /px4_1/fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE "x: |y: |z: |xy_valid" | head -8
  a=$(grep -ac '已离地' "$D/mpc.log"); [ "$a" -ge 2 ] && [ $tt -ge 60 ] && break
done
sleep 4
echo "### 终值 ---"
echo " d0 z/xy: $(timeout 4 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE 'z: |xy_valid' | tr '\n' ' ')"
echo " d1 z/xy: $(timeout 4 ros2 topic echo /px4_1/fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE 'z: |xy_valid' | tr '\n' ' ')"
echo " d1 status: $(timeout 4 ros2 topic echo /px4_1/fmu/out/vehicle_status_v1 --qos-reliability best_effort --once 2>/dev/null | grep -aE 'arming_state|nav_state:' | tr '\n' ' ')"
echo "### px4 事件 ---"
echo " [drone0]"; grep -aE "Ready for takeoff|Armed by|Takeoff detected|Arming denied|Preflight Fail" "$L0" | tail -5
echo " [drone1]"; grep -aE "Ready for takeoff|Armed by|Takeoff detected|Arming denied|Preflight Fail" "$L1" | tail -5
echo "### mpc markers ---"
grep -aE "acados OCP ready|已离地|retry ARM" "$D/mpc.log" | tail -6
echo "### [5] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
echo "PAIR-GUI-DEMO-DONE logs: $D"
