#!/bin/bash
# 一轮 SITL A/B：$1 = takeoff_xy_lock_enable (true/false)，$2 = 标签
LOCK="$1"; TAG="$2"
PX4_DIR="$HOME/PX4-Autopilot-1.14"
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
source "$HOME/Multi-UAV-simulation/tools/mpc_env.sh" >/dev/null 2>&1

nohup gz sim -r -s "$PX4_DIR/Tools/simulation/gz/worlds/default.sdf" > /tmp/ab_gz.log 2>&1 &
GZ=$!; sleep 12
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/ab_agent.log 2>&1 &
AG=$!; sleep 3
cd "$HOME/Multi-UAV-simulation"
# 🔑 复现首飞条件：飞机物理位置偏离编队槽位 OFFSET 米（首飞实测 0.964m）。
#    start_1_px4.sh 会按 scenarios.yaml 的 birth 出生 ⇒ 误差恒为 0，复现不出来。
OFFSET="${OFFSET:-1.0}"
mkdir -p "$HOME/px4_logs"
( export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
  export PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500
  export PX4_GZ_MODEL_POSE="$OFFSET,0,0,0,0,0"
  cd "$PX4_DIR"
  ./build/px4_sitl_default/bin/px4 -d -i 0 < /dev/null > "$HOME/px4_logs/px4_0.log" 2>&1 ) &
sleep 22
PX4=$(pgrep -f "bin/px4 -d -i 0" | head -1)

# leader 立刻横向平移：飞机还在地上就被命令横向速度 = 首飞那个机理
# 🔑 探针必须**先于** launch 启动：晚 2 秒起飞机已爬到 ~1m，探针会把 1m 当成地面基准，
#    离地判定整体偏掉（A2 轮实测 起始 z=-1.04 就是这么来的）。
echo "[probe] 先起探针"
python3 ~/windup_probe.py 75 "$TAG" > /tmp/ab_probe_$TAG.log 2>&1 &
PR2=$!
# 同时起 diag_monitor 落 CSV，验证 d0_airborne 列
( cd /home/idt/Multi-UAV-simulation && python3 diag_monitor.py --scenario OUT_solo1_hover --csv /tmp/diag_.csv > /tmp/diag_.log 2>&1 ) &
PR=$!
sleep 2
nohup ros2 launch mpc_control swarm_launch.py formation:=solo1       leader_mode:=hover leader_start_delay:=0 calib_shared_origin:=false       takeoff_xy_lock_enable:="$LOCK" > "/tmp/ab_mpc_$TAG.log" 2>&1 &
LA=$!
wait $PR
echo "[probe] exit=$?"
cat /tmp/ab_probe_$TAG.log

for p in $LA $PX4 $AG $GZ; do kill -INT $p 2>/dev/null; done
sleep 6
for p in $LA $PX4 $AG $GZ; do kill -TERM $p 2>/dev/null; done
sleep 3
echo "--- $TAG 结束 ---"
