#!/bin/bash
# spawn_px4.sh — 按 config/scenarios.yaml 的出生点启动 N 个 PX4 SITL 实例。
#
# 出生 POSES 由 tools/gen_spawn.py 从 scenarios.yaml 生成（单一真值源；与
# swarm_launch.py 读同一份 birth，杜绝 BIRTH_* ↔ POSES 漂移）。
#
# 用法:
#   FORMATION=cross5 bash tools/spawn_px4.sh               # 用队形标准出生点
#   SCENARIO=S11_cross5_perturbed bash tools/spawn_px4.sh  # 用工况(可扰动出生)
#   （start_N_px4.sh 是设好 FORMATION 的薄包装）
#
# Env: PX4_DIR(默认 ~/PX4-Autopilot-1.14), START_DELAY(默认 5),
#      FORMATION(默认 cross5) | SCENARIO(设了则优先)
set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14}"
START_DELAY="${START_DELAY:-5}"
FORMATION="${FORMATION:-cross5}"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4_DIR=$PX4_DIR 不存在"
    exit 1
fi

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
cd "$PX4_DIR"

# ── 出生 POSES：从 scenarios.yaml 生成（gen_spawn 把警告打到 stderr，stdout 仅 POSE 串）──
GEN="$(dirname "$(readlink -f "$0")")/gen_spawn.py"
if [ -n "${SCENARIO:-}" ]; then SEL=(--scenario "$SCENARIO"); TAG="$SCENARIO"
else                            SEL=(--formation "$FORMATION"); TAG="$FORMATION"; fi
mapfile -t POSES < <(python3 "$GEN" "${SEL[@]}" --format lines)
if [ "${#POSES[@]}" -eq 0 ]; then
    echo "ERROR: gen_spawn 未产出 POSES（检查 $GEN 与 config/scenarios.yaml）"
    exit 1
fi
echo "=== 出生点: $TAG (${#POSES[@]} 架)，来自 config/scenarios.yaml ==="

for i in "${!POSES[@]}"; do
    POSE="${POSES[$i]}"
    echo "启动 drone $i | ENU pose: $POSE"

    # 有 gnome-terminal 还不够：非交互 SSH 下 DISPLAY 未设，开终端必然失败
    # （"无法打开显示"），且 run_scenario.sh 只给 gz / ros2 launch 传了 DISPLAY=:1。
    # 没有可用显示时一律走下面的后台分支。
    if command -v gnome-terminal &> /dev/null && [ -n "${DISPLAY:-}" ]; then
        gnome-terminal --tab --title="px4_${i}_${TAG}" -- bash -c "
            export GZ_SIM_RESOURCE_PATH='$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds'
            export PX4_GZ_STANDALONE=1
            export PX4_SYS_AUTOSTART=4001
            export PX4_GZ_MODEL=x500
            export PX4_GZ_MODEL_POSE='$POSE'
            cd '$PX4_DIR'
            mkdir -p '$HOME/px4_logs'
            ./build/px4_sitl_default/bin/px4 -i $i 2>&1 | tee '$HOME/px4_logs/px4_$i.log'
            st=\${PIPESTATUS[0]}
            # 不再 exec bash：px4 退出/被杀后标签自动关闭，杜绝僵留空壳标签。
            # 仅异常退出时停留几秒供查看（输出已同时落盘到日志，崩溃信息不丢）。
            [ \"\$st\" -ne 0 ] && echo \"[px4 -i $i 异常退出 st=\$st，5s 后关闭；日志 ~/px4_logs/px4_$i.log]\" && sleep 5
        "
    else
        LOG_DIR="$HOME/px4_logs"
        mkdir -p "$LOG_DIR"
        (
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

# gz_overrides/models 若存在（本机 SITL 修正模型时用），置于 PX4 模型路径之前。
# 默认只保留 gz_overrides/worlds（SITL_WORLD 用），models 目录可留空。
_OVR="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)/gz_overrides"
if [ -d "$_OVR/models" ]; then
    export GZ_SIM_RESOURCE_PATH="$_OVR/models:$GZ_SIM_RESOURCE_PATH"
fi
            export PX4_GZ_STANDALONE=1
            export PX4_SYS_AUTOSTART=4001
            export PX4_GZ_MODEL=x500
            export PX4_GZ_MODEL_POSE="$POSE"
            cd "$PX4_DIR"
            # -d 守护模式关掉 pxh 交互控制台：无 tty 时 stdin EOF 会让控制台
            # 死循环重绘提示符，单实例日志几分钟灌到 ~20GB（2026-07-08 SSH 实测）
            ./build/px4_sitl_default/bin/px4 -d -i $i < /dev/null > "$LOG_DIR/px4_$i.log" 2>&1
        ) &
        echo "  后台，日志: $LOG_DIR/px4_$i.log (PID $!)"
    fi

    if [ "$i" -lt "$(( ${#POSES[@]} - 1 ))" ]; then
        echo "  等待 ${START_DELAY}s ..."
        sleep "$START_DELAY"
    fi
done

echo "=== $TAG: ${#POSES[@]} 架 PX4 已启动 ==="
