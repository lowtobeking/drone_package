#!/bin/bash
# start_1_px4.sh — solo1 单机（薄包装 → tools/spawn_px4.sh）
# 出生点来自 config/scenarios.yaml（改 birth 去那里改，勿手改本文件）。
#   终端1: gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
#   终端2: bash start_1_px4.sh
#   终端3: MicroXRCEAgent udp4 -p 8888
#   终端4: ros2 launch mpc_control swarm_launch.py formation:=solo1
export FORMATION="${FORMATION:-solo1}"
exec "$(dirname "$(readlink -f "$0")")/tools/spawn_px4.sh" "$@"
