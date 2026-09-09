#!/bin/bash
# start_2_px4.sh — pair2 双机（薄包装 → tools/spawn_px4.sh）
# 出生点来自 config/scenarios.yaml（改 birth 去那里改，勿手改本文件）。
#   终端2: START_DELAY=5 bash start_2_px4.sh
#   终端4: ros2 launch mpc_control swarm_launch.py formation:=pair2
export FORMATION="${FORMATION:-pair2}"
exec "$(dirname "$(readlink -f "$0")")/tools/spawn_px4.sh" "$@"
