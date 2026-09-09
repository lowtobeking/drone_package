#!/usr/bin/env bash
# 把当前 HEAD 的**飞行相关路径**部署到机载电脑，并在机上 git 仓库留下可追溯记录。
#
# 用法（在 Windows Git Bash 或任意能 ssh 到机载电脑的机器上，于仓库根目录执行）：
#     bash tools/deploy_to_jetson.sh A10 A08
#     bash tools/deploy_to_jetson.sh A10            # 只部署一台
#
# 为什么不直接 `git push`：
#   仓库 .git 有 291MB、工作区 206MB，其中 `report/` 独占 205MB（论文图表与数据），
#   而飞行真正需要的 mpc_control/launch/config/tools/deploy 加起来 <1MB。
#   经无线 mesh 推整仓既慢又容易断（现场实测网络会掉）。
#   故用 `git archive` 只打飞行相关路径（~570KB），到机上再提交进本地 git 仓库。
#
# 机载电脑侧前提（一次性，2026-08-20 已对 A10/A08 做过）：
#   cd ~/Multi-UAV-simulation && git init -b main && git add -A \
#     && git commit -m 'rsync 副本快照' && git config receive.denyCurrentBranch updateInstead
#
# 部署后机上可查：
#   git -C ~/Multi-UAV-simulation log --oneline -1     # commit 信息含上游 hash
#   cat ~/Multi-UAV-simulation/DEPLOYED_VERSION        # 完整 hash / 短 hash / 时间
#
# ⚠️ 部署后必须在机载电脑上重新 build，否则装的还是旧的：
#   ssh <host> 'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --packages-select mpc_control'

set -euo pipefail

HOSTS=("$@")
if [ ${#HOSTS[@]} -eq 0 ]; then
    echo "用法: bash tools/deploy_to_jetson.sh <host> [host...]   例: bash tools/deploy_to_jetson.sh A10 A08" >&2
    exit 1
fi

REPO_ROOT=$(git rev-parse --show-toplevel)
cd "$REPO_ROOT"

if [ -n "$(git status --porcelain)" ]; then
    echo "⚠️  工作区有未提交改动 —— 部署的是 HEAD，这些改动不会包含在内："
    git status --short | sed 's/^/     /'
    echo
fi

FULL=$(git rev-parse HEAD)
SHORT=$(git rev-parse --short HEAD)
WHEN=$(date '+%Y-%m-%d %H:%M:%S')
REMOTE_DIR='$HOME/Multi-UAV-simulation'

# 飞行相关路径（刻意不含 report/：205MB 论文素材，机载电脑用不到）
PATHS=(
    mpc_control launch config tools deploy px4_overrides resource docs
    package.xml setup.py setup.cfg CLAUDE.md README.md
    analyze_flight.py diag_monitor.py clean.sh
    start_1_px4.sh start_2_px4.sh start_3_px4.sh start_5_px4.sh start_9_px4.sh
)

TD=$(mktemp -d)
trap 'rm -rf "$TD"' EXIT

git archive HEAD -o "$TD/deploy.tar" -- "${PATHS[@]}"
printf '%s\n%s\n%s\n' "$FULL" "$SHORT" "$WHEN" > "$TD/DEPLOYED_VERSION"
echo "打包完成：$(du -h "$TD/deploy.tar" | cut -f1)  上游 $SHORT"
echo

for H in "${HOSTS[@]}"; do
    echo "===== $H ====="
    scp -o ConnectTimeout=20 -o BatchMode=yes "$TD/deploy.tar" "$TD/DEPLOYED_VERSION" "$H:/tmp/" >/dev/null
    ssh -o ConnectTimeout=25 -o BatchMode=yes "$H" "
        set -e
        cd $REMOTE_DIR
        if [ ! -d .git ]; then
            echo '  🔴 机上不是 git 仓库，先做一次性初始化（见本脚本头部注释）'; exit 1
        fi
        tar -xf /tmp/deploy.tar
        cp /tmp/DEPLOYED_VERSION ./DEPLOYED_VERSION
        git add -A
        if git commit -q -m '部署上游 $SHORT（飞行相关路径）' 2>/dev/null; then
            echo -n '  已提交  '; git show --stat --oneline HEAD | tail -1
        else
            echo '  无变更（已是该版本）'
        fi
        echo -n '  机上版本: '; git log --oneline -1
    "
    echo
done

echo "⚠️  别忘了在机载电脑上 build："
for H in "${HOSTS[@]}"; do
    echo "    ssh $H 'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --packages-select mpc_control'"
done
