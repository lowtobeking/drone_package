#!/bin/bash
# run_scenario.sh <scenario_id> <formation> <duration_s>
# e.g. run_scenario.sh S27_solo1_fence_line solo1 150
set -e
export SCENARIO=$1   # 须 export：start_N_px4.sh → spawn_px4.sh 靠继承环境变量取 --scenario
export FORMATION=$2  # 同上，SCENARIO 为空时 spawn_px4.sh 落回 --formation 也要对
DURATION=${3:-150}
NUM_DRONES=${4:-1}   # 1 for solo1/pair2, 3 for trio3
LOG_DIR=~/flights

# nohup 非交互 shell 不 source ~/.bashrc，显式设置，勿依赖登录 shell 环境
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-$HOME/PX4-Autopilot-1.14/Tools/simulation/gz/models:$HOME/PX4-Autopilot-1.14/Tools/simulation/gz/worlds}"

echo "=== START $SCENARIO (${DURATION}s) ==="

# 0. cleanup
for p in px4 'gz sim' gzserver MicroXRCEAgent mpc_node leader_node swarm_launch 'ros2 launch'; do
  pkill -9 -f "$p" 2>/dev/null || true; done
sleep 2

# 1. clear stale parameters
for i in $(seq 0 $((NUM_DRONES-1))); do
  rm -f ~/PX4-Autopilot-1.14/build/px4_sitl_default/rootfs/$i/parameters.bson
  rm -f ~/PX4-Autopilot-1.14/build/px4_sitl_default/rootfs/$i/parameters_backup.bson
done
echo "[OK] parameters cleared"

# 2. gazebo
DISPLAY=:1 gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf \
  &> /tmp/gz_${SCENARIO}.log &
GZ_PID=$!
echo "[..] Gazebo pid=$GZ_PID, waiting..."
for i in $(seq 1 20); do
  sleep 2
  if gz model --list 2>&1 | grep -q ground_plane; then
    echo "[OK] Gazebo ready"; break
  fi
  [ $i -eq 20 ] && echo "[ERR] Gazebo timeout" && exit 1
done

# 3. PX4 + Agent（同步启动，Agent 在 PX4 之前就绪）
MicroXRCEAgent udp4 -p 8888 &> /tmp/agent_${SCENARIO}.log &
sleep 1
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_${NUM_DRONES}_px4.sh \
  &> /tmp/px4_${SCENARIO}.log &
echo "[..] PX4+Agent starting, waiting for all $NUM_DRONES drones..."
LAST_IDX=$((NUM_DRONES - 1))
for i in $(seq 1 60); do
  sleep 2
  MODELS=$(gz model --list 2>&1)
  ALL_OK=true
  for d in $(seq 0 $LAST_IDX); do
    echo "$MODELS" | grep -q "x500_${d}" || { ALL_OK=false; break; }
  done
  $ALL_OK && echo "[OK] all x500_0..${LAST_IDX} in Gazebo" && break
  [ $i -eq 60 ] && echo "[ERR] PX4/model timeout" && exit 1
done
# 等 Agent 收到所有 session
echo "[..] Waiting for $NUM_DRONES Agent sessions..."
for i in $(seq 1 30); do
  sleep 2
  SESSIONS=$(grep -c "session established" /tmp/agent_${SCENARIO}.log 2>/dev/null || echo 0)
  [ "$SESSIONS" -ge "$NUM_DRONES" ] && echo "[OK] $SESSIONS Agent sessions" && break
  [ $i -eq 30 ] && echo "[WARN] only $SESSIONS/${NUM_DRONES} sessions — continuing anyway"  && break
done

# 4.5 可选：launch 前钩子（如 S31 需先给 PX4 设 failsafe 参数）
if [ -n "${PRE_LAUNCH_HOOK:-}" ]; then
  echo "[..] PRE_LAUNCH_HOOK: $PRE_LAUNCH_HOOK"
  eval "$PRE_LAUNCH_HOOK" || { echo "[ERR] PRE_LAUNCH_HOOK failed"; exit 1; }
  echo "[OK] PRE_LAUNCH_HOOK done"
fi

# 5. ros2 launch
cd ~/ros2_control_mpc_ws && source install/setup.bash
# EXTRA_LAUNCH_ARGS：整批扫候选配置用，如 EXTRA_LAUNCH_ARGS="vel_lag_tau:=0.5 soc_norm_enable:=true"
DISPLAY=:1 ros2 launch mpc_control swarm_launch.py scenario:=${SCENARIO} ${EXTRA_LAUNCH_ARGS:-} \
  &> /tmp/launch_${SCENARIO}.log &
echo "[..] ros2 launch started, waiting for ARM..."
# 须全部 N 架确认（旧版 grep -q 只等第一架：某机初始握手失败趴窝时照样开录，
# 幸存机被编队项拽出恒定偏差 → 录出废数据还判 FAIL，S2 20260713 即此因）
for i in $(seq 1 40); do
  sleep 2
  # grep -c 无匹配时打印 0 且 exit 1——不能接 || echo 0（会追加第二个 0 破坏整数比较）
  CONFIRMED=$(grep -c "OFFBOARD + ARMED confirmed" /tmp/launch_${SCENARIO}.log 2>/dev/null || true)
  CONFIRMED=${CONFIRMED:-0}
  if [ "$CONFIRMED" -ge "$NUM_DRONES" ]; then
    echo "[OK] ARMED+OFFBOARD ($CONFIRMED/$NUM_DRONES)"; break
  fi
  [ $i -eq 40 ] && echo "[ERR] ARM timeout ($CONFIRMED/$NUM_DRONES confirmed)" && exit 1
done

# 6. diag_monitor + log
CSV=${LOG_DIR}/flight_${SCENARIO}.csv
echo "[..] Recording ${DURATION}s → $CSV"
timeout ${DURATION} python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py \
  --formation ${FORMATION} --log ${CSV} &> /tmp/diag_${SCENARIO}.log || true

echo "[OK] Recording done. Lines in CSV: $(wc -l < $CSV)"

# 7. cleanup
for p in px4 'gz sim' gzserver MicroXRCEAgent mpc_node leader_node swarm_launch 'ros2 launch'; do
  pkill -9 -f "$p" 2>/dev/null || true; done
sleep 2
echo "=== DONE $SCENARIO ==="
