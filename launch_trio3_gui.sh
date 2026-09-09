#!/usr/bin/env bash
# trio3 三机 GUI 可视化后台启动器
cd /home/caolihao/drone_package_20260908 || exit 1
BASE="$(pwd)"
REPO=$BASE/Multi-UAV-simulation-full
D="$HOME/sitl_trio3_gui"
mkdir -p "$D"
export GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export START_DELAY="${START_DELAY:-15}"
source "$BASE/run_env.sh"
HP=$PX4_DIR

echo "### [0] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
sleep 3
rm -f /dev/shm/fastrtps_*
for i in 0 1 2; do rm -f "$HOME/px4_logs/px4_$i.log"; done
rm -rf "$D"/*

echo "### [1] gz GUI"
gz sim -r "$SITL_WORLD" > "$D/gz.log" 2>&1 &
sleep 18

echo "### [2] agent + px4 x3(trio3)"
MicroXRCEAgent udp4 -p 8888 > "$D/agent.log" 2>&1 &
sleep 3
cd "$HP" || exit 1
PX4_DIR=$HP FORMATION=trio3 bash "$REPO/tools/spawn_px4.sh" > "$D/spawn.log" 2>&1
for i in $(seq 1 45); do
  n=$(timeout 8 gz model --list 2>/dev/null | grep -c x500_)
  [ "$n" -ge 3 ] && { echo "  x500_0..2 已生成 ~$((i*2))s"; break; }
  sleep 2
done

echo "### [3] 等三机 Ready"
for i in $(seq 1 40); do
  sleep 5
  r=0
  for j in 0 1 2; do grep -aq "Ready for takeoff" "$HOME/px4_logs/px4_$j.log" 2>/dev/null && r=$((r+1)); done
  if [ "$r" -ge 3 ]; then echo "  三机 READY at ~$((i*5))s"; break; fi
done

echo "### [4] MPC trio3 hover —— 看窗口三架等边起飞"
KEEP=60
timeout $((KEEP+220)) ros2 launch mpc_control swarm_launch.py scenario:=S3_trio3_hover > "$D/mpc.log" 2>&1 &
for tt in 45 90 150; do
  sl=60
  [ $tt -eq 45 ] && sl=45
  sleep $sl
  a=$(grep -ac '已离地' "$D/mpc.log")
  oc=$(grep -ac 'acados OCP ready' "$D/mpc.log")
  echo "  --- t+$tt --- acados=$oc 已离地=$a"
  for j in 0 1 2; do
    [ $j -eq 0 ] && tp=/fmu || tp=/px4_$j/fmu
    echo "      d$j: $(timeout 4 ros2 topic echo "$tp/out/vehicle_local_position" --qos-reliability best_effort --once 2>/dev/null | grep -aE 'x: |y: |z: ' | tr '\n' ' ')"
  done
  if [ "$a" -ge 3 ] && [ $tt -ge 90 ]; then echo "  三机均已离地,再保持供观看"; sleep 35; break; fi
done
echo "### [5] 终值 px4 事件 ---"
for j in 0 1 2; do
  echo "  [drone$j] $(grep -aE 'Ready for takeoff|Armed by external|Takeoff detected' "$HOME/px4_logs/px4_$j.log" | tail -2 | tr '\n' ' | ')"
done
echo "### [6] cleanup"
pkill -9 -f 'px4 -d -i'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'swarm_launch|mpc_node|leader_node'
echo "TRIO3-GUI-DONE logs: $D"
