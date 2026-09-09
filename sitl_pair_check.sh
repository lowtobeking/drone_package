#!/usr/bin/env bash
# sitl_pair_check.sh — PX4-1.16 SITL pair2 双机 headless 验收：
# gz -s（无GUI）→ agent → 2×px4(pair2) → MPC pair2 hover 起飞/悬停。
# 关注 doc §6.1 的 drone1（mag/battery）健康告警是否仍在，是否影响起飞。
# 用法: bash sitl_pair_check.sh
set -u
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO=$BASE/Multi-UAV-simulation-full
D="$HOME/sitl_pair_check"
mkdir -p "$D"
export GZ_IP=127.0.0.1
export START_DELAY="${START_DELAY:-12}"
source "$BASE/run_env.sh"            # 内部已处理 set -u 下的 ROS source
HP=$PX4_DIR
L0="$HOME/px4_logs/px4_0.log"
L1="$HOME/px4_logs/px4_1.log"

echo "### [0] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
sleep 3
rm -f /dev/shm/fastrtps_*; rm -rf "$D"/*; rm -f "$L0" "$L1"

echo "### [1] gz headless($SITL_WORLD)+agent+px4 x2(pair2)"
gz sim -s -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 15
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3
cd "$HP" || exit 1
PX4_DIR=$HP FORMATION=pair2 bash "$REPO/tools/spawn_px4.sh" > "$D/spawn.log" 2>&1
for i in $(seq 1 40); do
  n=$(timeout 8 gz model --list 2>/dev/null | grep -c x500_)
  [ "$n" -ge 2 ] && { echo "  x500_0/x500_1 已生成 ~$((i*2))s"; break; }
  sleep 2
done

echo "### [2] 等双机 Ready (<=150s)"
for i in $(seq 1 30); do
  sleep 5
  r0=$(grep -ac "Ready for takeoff" "$L0" 2>/dev/null)
  r1=$(grep -ac "Ready for takeoff" "$L1" 2>/dev/null)
  if [ "$r0" -ge 1 ] && [ "$r1" -ge 1 ]; then echo "  双机 READY at ~$((i*5))s"; break; fi
done
echo "  [drone0] $(grep -aE 'Ready for takeoff|Preflight Fail|Gyro #0 fail' "$L0" | tail -3 | tr '\n' ' | ')"
echo "  [drone1] $(grep -aE 'Ready for takeoff|Preflight Fail|Gyro #0 fail' "$L1" | tail -3 | tr '\n' ' | ')"

echo "### [3] MPC pair2 hover（drone1 可能需现场编 acados，最多等 ~150s）"
timeout 200 ros2 launch mpc_control swarm_launch.py scenario:=S2_pair2_hover > "$D/mpc.log" 2>&1 &
for tt in 40 80 120 170; do
  sleep $(( tt==40?40:tt>120?50:40 ))
  a=$(grep -ac '已离地' "$D/mpc.log")
  oc=$(grep -ac 'acados OCP ready' "$D/mpc.log")
  echo "  --- t+$tt --- acados=$oc 已离地=$a"
  echo "   d0: $(timeout 4 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE 'z: |xy_valid: |x: ' | tr '\n' ' ')"
  echo "   d1: $(timeout 4 ros2 topic echo /px4_1/fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE 'z: |xy_valid: |x: ' | tr '\n' ' ')"
  if [ "$a" -ge 2 ]; then echo "  双机均已离地，收集稳定数据后结束"; sleep 10; break; fi
done
echo
echo "### [4] 终值 ---"
echo "  d0 z/xy: $(timeout 4 ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE 'z: |xy_valid:' | tr '\n' ' ')"
echo "  d1 z/xy: $(timeout 4 ros2 topic echo /px4_1/fmu/out/vehicle_local_position --qos-reliability best_effort --once 2>/dev/null | grep -aE 'z: |xy_valid:' | tr '\n' ' ')"
echo "  d1 io/arm: $(timeout 4 ros2 topic echo /px4_1/fmu/out/vehicle_status_v1 --qos-reliability best_effort --once 2>/dev/null | grep -aE 'arming_state:|nav_state:' | tr '\n' ' ')"
echo "  队形间距(d0-d1): 差值应≈3m"
echo "### px4 事件 ---"
echo "  [drone0]"; grep -aE "Ready for takeoff|Armed by|Takeoff detected|Arming denied|Preflight Fail" "$L0" | tail -6
echo "  [drone1]"; grep -aE "Ready for takeoff|Armed by|Takeoff detected|Arming denied|Preflight Fail" "$L1" | tail -6
echo "### mpc ---"
grep -aE "acados OCP ready|已离地|retry ARM|\[d[01]\]" "$D/mpc.log" | tail -8
echo "### [5] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
echo "SITL-PAIR-CHECK-DONE logs: $D"
