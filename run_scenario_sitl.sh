#!/bin/bash
# 完整跑一个 OUT_* 场景的 SITL 端到端：gz → PX4(出生偏航 45°) → Agent → diag_monitor → launch
# 用法: bash run_scenario_sitl.sh <scenario> <总时长s>
# 🔑 出生偏航设 45°：line_along_heading 若失效会朝正北(0°)飞，落点差 2.1m，一眼可辨。
SC="$1"; DUR="${2:-120}"
PX4_DIR="$HOME/PX4-Autopilot-1.14"
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
source "$HOME/Multi-UAV-simulation/tools/mpc_env.sh" >/dev/null 2>&1
rm -f "/tmp/sc_${SC}.csv"

nohup gz sim -r -s "$PX4_DIR/Tools/simulation/gz/worlds/default.sdf" > /tmp/sc_gz.log 2>&1 &
GZ=$!; sleep 12
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/sc_agent.log 2>&1 &
AG=$!; sleep 3
mkdir -p "$HOME/px4_logs"
( export PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500
  export PX4_GZ_MODEL_POSE="0,0,0,0,0,0.7854"        # yaw = 45°
  cd "$PX4_DIR"
  ./build/px4_sitl_default/bin/px4 -d -i 0 < /dev/null > "$HOME/px4_logs/px4_0.log" 2>&1 ) &
sleep 24
PX4=$(pgrep -f "bin/px4 -d -i 0" | head -1)

cd "$HOME/Multi-UAV-simulation"
nohup python3 diag_monitor.py --scenario "$SC" --log "/tmp/sc_${SC}.csv" > /tmp/sc_diag.log 2>&1 &
DG=$!; sleep 2
nohup ros2 launch mpc_control swarm_launch.py scenario:="$SC" > "/tmp/sc_mpc_${SC}.log" 2>&1 &
LA=$!
echo "[run] 已启动，跑 ${DUR}s"
sleep "$DUR"

echo "[run] 收尾"
# 🔴 只 kill `ros2 launch` **不可靠**：子节点(leader_node/mpc_node)会活下来，
#    下一轮 leader 会读到上一轮残留节点的 /mpc/health ⇒ 任务时钟秒起表、结果作废
#    （2026-08-21 circle 第一轮就是这么废掉的）。必须按可执行路径显式清并核实到 0。
PAT='mpc_control/lib/mpc_control|diag_monitor.py|ros2 launch|bin/px4|gz sim|MicroXRCEAgent'
for sig in INT TERM KILL; do
  for pid in $(ps -eo pid,args --no-headers | grep -E "$PAT" | grep -v grep | awk '{print $1}'); do
    kill -$sig "$pid" 2>/dev/null
  done
  sleep 4
done
LEFT=$(ps -eo pid,args --no-headers | grep -E "$PAT" | grep -v grep | wc -l)
echo "[run] 残留进程数=$LEFT （必须为 0）"
echo "[run] 结束 CSV=/tmp/sc_${SC}.csv"
