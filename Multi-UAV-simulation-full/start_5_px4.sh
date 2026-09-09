#!/bin/bash
# start_5_px4.sh — cross5 / star5 五机（薄包装 → tools/spawn_px4.sh）
# 出生点来自 config/scenarios.yaml（改 birth 去那里改，勿手改本文件）。
#   终端2: START_DELAY=5 bash start_5_px4.sh
#          star5(出生同十字):  SCENARIO=S9_star5_line       bash start_5_px4.sh
#          扰动出生:           SCENARIO=S11_cross5_perturbed bash start_5_px4.sh
#   终端4: ros2 launch mpc_control swarm_launch.py formation:=cross5   (或 scenario:=...)
export FORMATION="${FORMATION:-cross5}"
exec "$(dirname "$(readlink -f "$0")")/tools/spawn_px4.sh" "$@"
