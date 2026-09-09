#!/usr/bin/env bash
# sitl_trio3_check.sh — PX4-1.16 SITL trio3 三机 headless 验收：
# gz -s(无GUI) → agent → 3×px4(trio3) → MPC S3_trio3_hover 起飞/等边悬停。
# trio3 每机 2 邻居 → 首次需现场编 acados m2(分钟级)，此脚本给出较长容忍窗口。
# 用法: bash sitl_trio3_check.sh
set -u
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO=$BASE/Multi-UAV-simulation-full
D="$HOME/sitl_trio3_check"
mkdir -p "$D"
export GZ_IP=127.0.0.1
export START_DELAY="${START_DELAY:-15}"
source "$BASE/run_env.sh"            # 内部已处理 set -u 下的 ROS source
HP=$PX4_DIR

echo "### [0] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
sleep 3
rm -f /dev/shm/fastrtps_*
for i in 0 1 2; do rm -f "$HOME/px4_logs/px4_$i.log"; done
rm -rf "$D"/*

echo "### [1] gz headless + agent + px4 x3(trio3)"
gz sim -s -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 15
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3
cd "$HP" || exit 1
PX4_DIR=$HP FORMATION=trio3 bash "$REPO/tools/spawn_px4.sh" > "$D/spawn.log" 2>&1
for i in $(seq 1 50); do
  n=$(timeout 8 gz model --list 2>/dev/null | grep -c x500_)
  [ "$n" -ge 3 ] && { echo "  x500_0..2 已生成 ~$((i*2))s"; break; }
  sleep 2
done
echo "  spawn 尾: $(tail -1 "$D/spawn.log")"

echo "### [2] 等三机 Ready (<=200s)"
for i in $(seq 1 40); do
  sleep 5
  r=0
  for j in 0 1 2; do grep -aq "Ready for takeoff" "$HOME/px4_logs/px4_$j.log" 2>/dev/null && r=$((r+1)); done
  if [ "$r" -ge 3 ]; then echo "  三机 READY at ~$((i*5))s"; break; fi
done
for j in 0 1 2; do
  echo "  [drone$j] $(grep -aE 'Ready for takeoff|Preflight Fail|Gyro #0 fail|Compass' "$HOME/px4_logs/px4_$j.log" | tail -2 | tr '\n' ' | ')"
done

echo "### [3] MPC S3_trio3_hover（每机m2现场编acados，容忍 <=210s）"
timeout 260 ros2 launch mpc_control swarm_launch.py scenario:=S3_trio3_hover > "$D/mpc.log" 2>&1 &
for tt in 45 90 150 210; do
  [ $tt -eq 45 ] && sl=45 || sl=60
  sleep $sl
  a=$(grep -ac '已离地' "$D/mpc.log")
  oc=$(grep -ac 'acados OCP ready' "$D/mpc.log")
  echo "  --- t+$tt --- acados=$oc 已离地=$a"
  for j in 0 1 2; do
    if [ $j -eq 0 ]; then tp=/fmu; else tp=/px4_$j/fmu; fi
    v=$(timeout 4 ros2 topic echo "$tp/out/vehicle_local_position" --qos-reliability best_effort --once 2>/dev/null | grep -aE 'z: |xy_valid:' | tr '\n' ' ')
    echo "      d$j: $v"
  done
  if [ "$a" -ge 3 ]; then echo "  三机均已离地"; sleep 8; break; fi
done
echo
echo "### [4] 终值 ---"
for j in 0 1 2; do
  if [ $j -eq 0 ]; then tp=/fmu; else tp=/px4_$j/fmu; fi
  echo "  d$j xy/z: $(timeout 4 ros2 topic echo "$tp/out/vehicle_local_position" --qos-reliability best_effort --once 2>/dev/null | grep -aE 'xy_valid:|z: |x: |y: ' | tr '\n' ' ')"
  echo "  d$j io: $(timeout 4 ros2 topic echo "$tp/out/vehicle_status_v1" --qos-reliability best_effort --once 2>/dev/null | grep -aE 'arming_state:|nav_state:' | tr '\n' ' ')"
done
echo "### [5] 本地ENU内 x500_0..2 间距(应≈等边三角形边5.2m, 中心为原点) ---"
timeout 10 gz topic -e -t /world/default/pose/info -n 3 2>/dev/null | python3 -c "
import sys,re
d=sys.stdin.read()
for nm in ['x500_0','x500_1','x500_2']:
    i=d.find('name: \"%s\"'%nm)
    if i<0: continue
    seg=d[i:i+400]
    m=re.search(r'position \\{.*?x:\\s*([-\\d.]+).*?y:\\s*([-\\d.]+).*?z:\\s*([-\\d.]+)',seg,re.S)
    if m: print('  %s: (%s, %s, %s)m'%(nm,m.group(1),m.group(2),m.group(3)))
"
echo "### px4 事件(是否启动/armed/takeoff) ---"
for j in 0 1 2; do
  echo "  [drone$j]"; grep -aE "Ready for takeoff|Armed by external|Takeoff detected|Arming denied" "$HOME/px4_logs/px4_$j.log" | tail -3
done
echo "### [6] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
echo "SITL-TRIO3-CHECK-DONE logs: $D"
