#!/usr/bin/env bash
# pair2_sitl_check.sh —— 双机 SITL 验收：把 2026-08-30 的四项改动在仿真里跑成断言。
#
# 与 report/run_one_trial.sh 的区别：那个是为论文录数据的，这个是**验收**——每一步都
# 给 PASS/FAIL，不产出统计量。沿用它趟出来的启动模式（gz -s / 按 px4_logs 数 Ready /
# mpc_node 存活校验 / set +u 包 ROS setup），因为那些都是踩出来的。
#
# 用法（在仿真机上，非交互 ssh 也可）：
#   tools/pair2_sitl_check.sh all          # 全部（约 12 分钟）
#   tools/pair2_sitl_check.sh reflat       # 只做 ref_lat 探针（回答 XY 对齐的塌缩前提）
#   tools/pair2_sitl_check.sh xyalign      # XY 全局对齐真的对齐了吗（须 reflat 先 PASS）
#   tools/pair2_sitl_check.sh hover        # pair2 hover + 起飞锁回归
#   tools/pair2_sitl_check.sh semiauto     # 半自主档 offboard_on_arm
#   tools/pair2_sitl_check.sh land         # LAND 广播双机
#
# ⚠️ 每个子命令都自带清场与拉起，互不依赖，可单独重跑。
# ⚠️ 结果汇总在末尾；任一 FAIL 则退出码非 0。
set -uo pipefail

WS=${WS:-~/ros2_control_mpc_ws}
MPC=$WS/src/mpc_control
PX4_DIR=${PX4_DIR:-~/PX4-Autopilot-1.14}
OUTDIR=${OUTDIR:-~/pair2_check/$(date +%Y%m%d_%H%M%S)}
N=2
mkdir -p "$OUTDIR"

export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}

PASS=0; FAIL=0; SKIP=0
log()  { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUTDIR/run.log"; }
ok()   { PASS=$((PASS+1)); echo "  [PASS] $*" | tee -a "$OUTDIR/run.log"; }
bad()  { FAIL=$((FAIL+1)); echo "  [FAIL] $*" | tee -a "$OUTDIR/run.log"; }
skip() { SKIP=$((SKIP+1)); echo "  [SKIP] $*" | tee -a "$OUTDIR/run.log"; }

# 清场用独立脚本，避免命令串里含匹配模式把自己杀掉（tools/cleanup_sim.sh 同理）
cleanup() {
    bash "$MPC/tools/cleanup_sim.sh" >/dev/null 2>&1 || true
    ros2 daemon stop >/dev/null 2>&1 || true
    sleep 2
}

# ── 拉起 gz + N 架 PX4 + DDS agent + build，成功返回 0 ────────────────────────
bringup() {
    local tag=$1
    cleanup
    log "[$tag] gazebo…"
    nohup gz sim -s -r "$PX4_DIR/Tools/simulation/gz/worlds/default.sdf" \
        > "$OUTDIR/$tag.gz.log" 2>&1 &
    for _ in $(seq 1 30); do
        grep -qi "Gazebo Sim Server" "$OUTDIR/$tag.gz.log" 2>/dev/null && break
        sleep 1
    done
    sleep 5

    log "[$tag] px4 x$N…"
    rm -f ~/px4_logs/px4_*.log 2>/dev/null
    START_DELAY=5 nohup bash "$MPC/start_${N}_px4.sh" > "$OUTDIR/$tag.px4.log" 2>&1 &
    local got=0
    for _ in $(seq 1 90); do
        got=0
        for i in $(seq 0 $((N-1))); do
            [ -f ~/px4_logs/px4_${i}.log ] && \
                grep -q "Ready for takeoff" ~/px4_logs/px4_${i}.log 2>/dev/null && got=$((got+1))
        done
        [ "$got" -ge "$N" ] && break
        sleep 1
    done
    if [ "$got" -lt "$N" ]; then
        bad "[$tag] 只有 $got/$N 架 PX4 就绪（90s）——本项无法执行"
        return 1
    fi

    nohup MicroXRCEAgent udp4 -p 8888 > "$OUTDIR/$tag.dds.log" 2>&1 &
    sleep 3

    cd "$WS"
    set +u; source /opt/ros/humble/setup.bash; set -u
    if ! colcon build --packages-select mpc_control > "$OUTDIR/$tag.build.log" 2>&1; then
        bad "[$tag] colcon build 失败，见 $tag.build.log"
        return 1
    fi
    set +u; source install/setup.bash; set -u
    return 0
}

# 启动 launch 并确认两个 mpc_node 都活着
start_launch() {
    local tag=$1; shift
    log "[$tag] ros2 launch mpc_control swarm_launch.py $*"
    nohup ros2 launch mpc_control swarm_launch.py "$@" > "$OUTDIR/$tag.launch.log" 2>&1 &
    echo $! > "$OUTDIR/$tag.launch.pid"
    sleep 8
    local alive; alive=$(pgrep -c -f '[m]pc_control/lib/mpc_control/mpc_node' 2>/dev/null); alive=${alive:-0}
    if [ "$alive" -lt "$N" ]; then
        bad "[$tag] 只有 $alive/$N 个 mpc_node 存活"
        grep -iE 'Traceback|Error|cannot open shared' "$OUTDIR/$tag.launch.log" | tail -5
        return 1
    fi
    ok "[$tag] $alive/$N 个 mpc_node 存活"
    return 0
}

stop_launch() {
    local tag=$1
    [ -f "$OUTDIR/$tag.launch.pid" ] && kill -INT "$(cat "$OUTDIR/$tag.launch.pid")" 2>/dev/null
    sleep 3
    cleanup
}

# 等日志里出现某模式，超时返回 1
wait_for() {   # wait_for <file> <pattern> <timeout_s>
    for _ in $(seq 1 "$3"); do
        grep -qE "$2" "$1" 2>/dev/null && return 0
        sleep 1
    done
    return 1
}

# ── 断言：两机都稳定停在各自参考点上 ─────────────────────────────────────────
# hover 模式的 leader **绕过就绪门**（leader_node.py:327 对 hover 直接退回固定
# start_delay），永远不打印 "formation ready" ⇒ 不能靠等那条日志来判就绪。
# 改为读各机最后一条 solve：高度落在 target_alt±0.4m 且代价已收敛，
# 这才是"就绪"本来要表达的东西。
assert_hold() {   # assert_hold <tag> <launch_log> <target_alt>
    local tag=$1 L=$2 alt=$3
    local d last z cost
    for d in 0 1; do
        last=$(grep -oE "\[d$d\] solve:.*" "$L" 2>/dev/null | tail -1)
        if [ -z "$last" ]; then
            bad "[$tag] d$d 全程没有任何 solve 记录（没收到数据或没进 OFFBOARD）"
            continue
        fi
        z=$(echo "$last"    | sed -nE 's/.*pos=\([^,]*,[^,]*,(-?[0-9.]+)\).*/\1/p')
        cost=$(echo "$last" | sed -nE 's/.*cost=([0-9.]+).*/\1/p')
        if [ -z "$z" ] || [ -z "$cost" ]; then
            bad "[$tag] d$d solve 行解析失败: $last"
            continue
        fi
        if awk -v z="$z" -v a="$alt" 'BEGIN{exit !(z-a<0.4 && z-a>-0.4)}' \
           && awk -v c="$cost" 'BEGIN{exit !(c<5.0)}'; then
            ok "[$tag] d$d 稳定在目标高度（z=$z，目标 $alt，cost=$cost）"
        else
            bad "[$tag] d$d 未稳定（z=$z，目标 $alt，cost=$cost）"
        fi
    done
}

# ── ① ref_lat 探针：XY 全局对齐在 SITL 里到底能不能验 ─────────────────────────
# 背景：xy_global_align 用各机 EKF 原点的经纬度相对 drone0 投影。若 SITL 各实例共享
# Gazebo 世界原点、ref_lat/ref_lon 完全相同，则 dN=dE=0、**所有机的 world_birth 会塌缩
# 到同一点**——那样在 SITL 里开这个开关是有害的，且该功能在 SITL 根本无法验证。
# 这就是 2026-08-14 那个未完成探针要回答的问题，先答它再谈别的。
stage_reflat() {
    log "=== ① ref_lat 探针（XY 全局对齐的塌缩前提）==="
    bringup reflat || return
    start_launch reflat scenario:=S2_pair2_hover || { stop_launch reflat; return; }
    sleep 15
    local f="$OUTDIR/reflat.txt"; : > "$f"
    # 🔑 QoS（2026-08-31 加）：PX4 out 话题是 BEST_EFFORT（见 mpc_node.py make_px4_qos），
    #    而 ros2 topic echo 默认按 RELIABLE 订阅 ⇒ reliability 不兼容、一条都收不到，
    #    症状与「压根没数据」完全一样。必须显式指定 best_effort。
    #    （durability 方向是兼容的：发布端 TRANSIENT_LOCAL ⊇ 订阅端 VOLATILE，无需改。）
    # 🔑 同时把 topic info 落盘：万一仍读不到，用它区分「话题不存在」/「无发布者」/
    #    「QoS 仍不匹配」，避免再花一整轮去猜。
    timeout 20 ros2 topic list > "$OUTDIR/topic_list.txt" 2>&1 || true
    for i in 0 1; do
        local topic="/fmu/out/vehicle_local_position"
        [ "$i" -ne 0 ] && topic="/px4_${i}${topic}"
        echo "########## drone$i $topic" >> "$OUTDIR/topic_info.txt"
        timeout 20 ros2 topic info -v "$topic" >> "$OUTDIR/topic_info.txt" 2>&1 || true
        timeout 15 ros2 topic echo --once --qos-reliability best_effort "$topic" 2>/dev/null \
            | grep -E "^ *ref_(lat|lon):" | sed "s/^/drone$i /" >> "$f"
    done
    cat "$f" | tee -a "$OUTDIR/run.log"
    local nuniq; nuniq=$(grep "ref_lat" "$f" | awk '{print $3}' | sort -u | wc -l)
    if [ "$(grep -c ref_lat "$f")" -lt 2 ]; then
        bad "① 没读到两机的 ref_lat（话题名/Agent 有问题）"
    elif [ "$nuniq" -ge 2 ]; then
        ok "① 两机 ref_lat 不同 ⇒ XY 全局对齐在 SITL 可验，可用 xy_global_align_enable:=true 跑"
    else
        skip "① 两机 ref_lat 相同 ⇒ SITL 各实例共享 EKF 原点：该功能在 SITL 无法验证，"
        echo "         且开启会把两机 world_birth 塌缩到同一点。**SITL 一律保持 false**，"
        echo "         真机地面验（前置清单第 23 条）。数学已由 tools/test_xy_datum_align.py 覆盖。"
    fi
    stop_launch reflat
}

# ── ② pair2 hover + 起飞锁回归 ────────────────────────────────────────────────
# 注意：起飞锁的原 bug 在 SITL 里**本来就不会出现**——SITL 用 auto_arm，解锁与切
# OFFBOARD 都发生在地面，所以旧实现捕到的"地面基准"也是对的。这里验的是修复后
# 闩锁仍能正常武装并解除（回归），不是验它能挡住地面积分饱和（Gazebo 复刻不出）。
stage_hover() {
    log "=== ② pair2 hover + 起飞锁回归 ==="
    bringup hover || return
    start_launch hover scenario:=OUT_pair2_hover takeoff_xy_lock_enable:=true \
        || { stop_launch hover; return; }
    local L="$OUTDIR/hover.launch.log"
    if wait_for "$L" "起飞期：横向速度指令锁零" 40; then ok "② 起飞锁已武装"
    else bad "② 40s 内未见起飞锁武装日志"; fi
    if wait_for "$L" "已离地.*横向速度指令解锁" 60; then ok "② 起飞锁已解除（离地判定生效）"
    else bad "② 60s 内闩锁未解除——这正是真机上那个 bug 的症状，查 $L"; fi
    if wait_for "$L" "起飞锁看门狗" 5; then
        bad "② 看门狗被触发 = 正常判据失灵（不该发生）"
    else ok "② 看门狗未触发（正常路径生效）"; fi
    # 🔑 hover 模式**不会**打印 "formation ready"：leader_node.py:327 对 hover 直接
    #    退回固定 start_delay、绕过就绪门。原断言在等一条永远不出现的日志 ⇒ 必然假 FAIL
    #    （2026-08-31 实跑咬到）。改为直接验"两机是否稳定在各自参考点上"，
    #    这才是就绪门本来想表达的意思。
    sleep 30
    assert_hold hover "$L" -3.0
    # 🔑 只判**真故障**。原 grep 有两个坑（同上，实跑咬到）：
    #   ① "fallback" 命中每条 solve 里的 "fallbacks=0" —— 那是**零**回退、是好事；
    #   ② "SAFETY" 命中起飞爬升穿 min_alt 的 SAFETY DEGRADED['fence_alt_low']——
    #      已知且良性，真机 2026-08-20 首飞同样触发过，5s 后自行消失。
    if grep -qE "OFFBOARD LOST|SAFETY HOLD|RELINQUISH|fallbacks=[1-9]" "$L"; then
        bad "② 悬停期出现真故障（HOLD/RELINQUISH/OFFBOARD LOST/非零 fallback），查 $L"
        grep -oE "OFFBOARD LOST|SAFETY HOLD \[[^]]*\]|RELINQUISH \[[^]]*\]" "$L" | sort | uniq -c | head -5
    else
        local ndeg; ndeg=$(grep -c "SAFETY DEGRADED" "$L" 2>/dev/null || echo 0)
        ok "② 悬停无真故障（起飞期 DEGRADED $ndeg 次，属已知良性）"
    fi
    stop_launch hover
}

# ── ③ 半自主档 offboard_on_arm ────────────────────────────────────────────────
# SITL 没有 RC，用外部发一条 ARM 指令模拟"飞手解锁"。要验的是：
#   · 节点自己**不发** ARM（auto_arm:=false），飞机在外部解锁前不解锁；
#   · 外部解锁后节点自动切 OFFBOARD 并起飞。
stage_semiauto() {
    log "=== ③ 半自主档 offboard_on_arm ==="
    bringup semi || return
    start_launch semi scenario:=OUT_pair2_hover auto_arm:=false offboard_on_arm:=true \
        || { stop_launch semi; return; }
    local L="$OUTDIR/semi.launch.log"
    if wait_for "$L" "半自主档 — 等待飞手 RC 解锁" 30; then ok "③ 节点进入半自主档等待状态"
    else bad "③ 未见半自主档启动日志（参数没透传？）"; fi
    sleep 20
    if grep -qE "半自主档：已解锁" "$L"; then
        bad "③ 外部还没解锁，节点就报"已解锁" —— 状态判定有误"
    else ok "③ 外部解锁前节点未请求切 OFFBOARD"; fi
    log "③ 外部发 ARM（模拟飞手解锁）…"
    for i in 0 1; do
        local topic="/fmu/in/vehicle_command"
        [ "$i" -ne 0 ] && topic="/px4_${i}${topic}"
        ros2 topic pub -1 "$topic" px4_msgs/msg/VehicleCommand \
            "{command: 400, param1: 1.0, target_system: $((i+1)), target_component: 1,
              source_system: 255, source_component: 190, from_external: true}" \
            >/dev/null 2>&1
    done
    if wait_for "$L" "半自主档：已解锁，请求切 OFFBOARD" 30; then ok "③ 解锁后自动请求切 OFFBOARD"
    else bad "③ 解锁后 30s 未请求切 OFFBOARD"; fi
    if wait_for "$L" "OFFBOARD \+ ARMED confirmed" 40; then ok "③ 已进入 OFFBOARD 并起飞"
    else bad "③ 40s 内未确认 OFFBOARD+ARMED"; fi
    stop_launch semi
}

# ── ④ LAND 广播双机 ───────────────────────────────────────────────────────────
stage_land() {
    log "=== ④ LAND 广播双机（第 1 层自动降落）==="
    bringup land || return
    start_launch land scenario:=OUT_pair2_hover mission_duration:=45.0 \
        || { stop_launch land; return; }
    local L="$OUTDIR/land.launch.log"
    # 同 ②：OUT_pair2_hover 是 hover 模式，leader 绕过就绪门、不打印 "formation ready"。
    sleep 30
    assert_hold land "$L" -3.0
    # 🔑 别用裸 "LAND" 做判据（2026-08-31 实跑咬到）：leader 在**起表时**就会打印
    #    「…45s 后广播 LAND 自动降落」这条**预告**，裸 LAND 会立刻命中 ⇒ 假 PASS，
    #    而真正的广播还要等 mission_duration 秒。必须匹配广播那一刻的专有串。
    #    时间线：全员到高度→任务时钟起表→+mission_duration→MISSION COMPLETE。
    if wait_for "$L" "MISSION COMPLETE" 180; then ok "④ LAND 广播已发出（leader 侧）"
    else bad "④ 180s 内未见 MISSION COMPLETE 广播（mission_duration 没透传？）"; fi
    # 三级链路各自独立断言：收到指令 → settle → 命令 AUTO.LAND
    sleep 8
    local n
    # 🔑 grep -c 命中 0 次时**退出码为 1**，写成 `$(grep -c ... || echo 0)` 会把
    #    grep 的 "0" 和 echo 的 "0" 一起捕获成两行，后面 [ "$n" -ge 2 ] 直接报错。
    #    用 `|| true` 而非 `|| echo 0`。
    n=$(grep -c "收到任务完成 LAND 指令" "$L" 2>/dev/null || true); n=${n:-0}
    if [ "$n" -ge 2 ]; then ok "④ 两机都收到 LAND 指令（$n 条）"
    else bad "④ 只有 $n 机收到 LAND 指令，应 >= 2"; fi
    n=$(grep -c "命令 AUTO.LAND" "$L" 2>/dev/null || true); n=${n:-0}
    if [ "$n" -ge 2 ]; then ok "④ 两机都已命令 AUTO.LAND（$n 条）"
    else bad "④ 只有 $n 机命令了 AUTO.LAND，应 >= 2"; fi
    stop_launch land
}

# ── ⑤ XY 全局对齐真的对齐了吗（2026-08-31 新增）──────────────────────────────
# 前提：① 已答"两机 ref_lat 不同"⇒ SITL 里开这个开关有意义、不会塌缩。
# 本项验的是**投影结果对不对**：drone0 广播 datum，各机据自身 EKF 原点经纬度投影出
# world_birth_xy。判据 = 两机 world_birth 之差应等于真实出生间距（pair2 = 3m 南北），
# 而**不是**塌缩到同一点。
# ⚠️ 这只验数学与链路在 SITL 内自洽；真机还要验 datum 跨机到达（前置清单第 23 条）。
stage_xyalign() {
    log "=== ⑤ XY 全局对齐（xy_global_align_enable:=true）==="
    bringup xyalign || return
    start_launch xyalign scenario:=S2_pair2_hover xy_global_align_enable:=true \
        || { stop_launch xyalign; return; }
    local L="$OUTDIR/xyalign.launch.log"
    if wait_for "$L" "XY 全局对齐" 60; then ok "⑤ 已执行 XY 全局对齐（datum 已到达）"
    else bad "⑤ 60s 内未见 XY 全局对齐日志（datum 没跨机到达？）"; fi
    sleep 20
    grep -oE "\[veh [0-9]\] XY 全局对齐.*" "$L" | tail -4 | tee -a "$OUTDIR/run.log"
    # 取 drone1 的 dN/dE：pair2 出生点相距 3m（正北 -3），故 dN 应 ≈ -3、dE ≈ 0
    local dn de
    dn=$(grep -oE "\[veh 1\] XY 全局对齐.*dN=([-+0-9.]+)" "$L" | tail -1 | sed -nE 's/.*dN=([-+0-9.]+).*/\1/p')
    de=$(grep -oE "\[veh 1\] XY 全局对齐.*dE=([-+0-9.]+)" "$L" | tail -1 | sed -nE 's/.*dE=([-+0-9.]+).*/\1/p')
    if [ -z "$dn" ] || [ -z "$de" ]; then
        bad "⑤ 没解析到 drone1 的 dN/dE"
    elif awk -v n="$dn" 'BEGIN{exit !(n<-2.5 && n>-3.5)}' \
         && awk -v e="$de" 'BEGIN{exit !(e<0.5 && e>-0.5)}'; then
        ok "⑤ 投影正确：dN=$dn dE=$de（期望 ≈-3.0 / ≈0，即真实出生间距）"
    elif awk -v n="$dn" -v e="$de" 'BEGIN{exit !(n<0.5 && n>-0.5 && e<0.5 && e>-0.5)}'; then
        bad "⑤ **塌缩了**：dN=$dn dE=$de 都≈0 ⇒ 两机 world_birth 落到同一点"
    else
        bad "⑤ 投影异常：dN=$dn dE=$de（期望 ≈-3.0 / ≈0）"
    fi
    assert_hold xyalign "$L" -5.0
    stop_launch xyalign
}

# ⚠️ 刻意**不给默认子命令**：本脚本第一件事就是 pkill 清场并拉起 Gazebo/PX4，
#    误敲一次空参数就会把别人正在跑的仿真清掉。必须显式指定要做哪一项。
case "${1:-}" in
    xyalign)  stage_xyalign ;;
    reflat)   stage_reflat ;;
    hover)    stage_hover ;;
    semiauto) stage_semiauto ;;
    land)     stage_land ;;
    all)      stage_reflat; stage_xyalign; stage_hover; stage_semiauto; stage_land ;;
    *) echo "用法: $0 all|reflat|xyalign|hover|semiauto|land   （无默认值：会清场并拉起仿真，须显式指定）"
       exit 64 ;;
esac

log "================ 汇总: PASS=$PASS FAIL=$FAIL SKIP=$SKIP ================"
log "日志目录: $OUTDIR"
exit $([ "$FAIL" -eq 0 ] && echo 0 || echo 1)
