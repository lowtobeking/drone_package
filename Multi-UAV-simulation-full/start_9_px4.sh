#!/bin/bash
# start_9_px4.sh — grid9 九机 3×3（薄包装 → tools/spawn_px4.sh）
# 出生点来自 config/scenarios.yaml（改 birth 去那里改，勿手改本文件）。
#   终端2: START_DELAY=3 bash start_9_px4.sh
#   终端4: ros2 launch mpc_control swarm_launch.py formation:=grid9
export FORMATION="${FORMATION:-grid9}"
export START_DELAY="${START_DELAY:-3}"
exec "$(dirname "$(readlink -f "$0")")/tools/spawn_px4.sh" "$@"
