#!/bin/bash
# 彻底清理仿真进程（放在脚本文件里执行，调用方命令串不含匹配模式 → 不自杀）
for r in 1 2 3; do
  pids=$(pgrep -f "run_scenario.sh|build/px4_sitl_default/bin/px4|gz sim|gzserver|MicroXRCEAgent|mpc_control/lib|leader_node|spawn_px4|diag_monitor|swarm_launch" | grep -vw $$ || true)
  rubies=$(pgrep -x ruby || true)
  [ -z "$pids$rubies" ] && break
  for p in $pids $rubies; do kill -9 "$p" 2>/dev/null || true; done
  sleep 2
done
left=$(pgrep -f "build/px4_sitl_default/bin/px4|gz sim|gzserver|MicroXRCEAgent|mpc_control/lib" | grep -vw $$ || true)
if [ -n "$left" ]; then echo "REMAIN: $left"; ps -o pid,cmd -p $left; else echo "CLEAN"; fi

# 🔑 陈旧锁清理（2026-08-31 加）：kill -9 杀得掉进程，但 PX4 SITL 的
#    /tmp/px4_lock-<i> 与 /tmp/px4-sock-<i> 会留下来，下次启动**静默**报
#    "PX4 server already running for instance N" 后直接退出（返回值 256），
#    表现为 bringup 报「只有 0/2 架 PX4 就绪」，极易误判成别的问题。
#    ⚠️ 只在确认无 px4 进程存活时才删——有进程时那些文件是活的，删了会搞乱正在跑的实例。
if [ -z "$(pgrep -f 'build/px4_sitl_default/bin/px4' | grep -vw $$ || true)" ]; then
  n=$(ls /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null | wc -l)
  if [ "$n" -gt 0 ]; then
    rm -rf /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null || true
    echo "STALE-LOCKS-REMOVED: $n"
  fi
else
  echo "STALE-LOCKS-KEPT (px4 仍在跑)"
fi

# 🔑 /dev/shm DDS 孤儿段清理（2026-09-01 加，与 report/run_one_trial.sh 的 cleanup() 同源）：
#    kill -9 掉的 Fast-DDS 参与者不回收共享内存段。09-01 实测一天攒到 **340 个（83 MB）**，
#    最老的来自前一天的 pair2 验收；DDS 发现随之变慢，症状是 start_N_px4.sh 起到一半卡住
#    （后几架 px4 日志一个字都没有）、90s 门限里只就绪 2/5。
#    ⚠️ 同样只在确认无任何仿真参与者（px4/gz/Agent/mpc 节点）存活时才清。
if [ -z "$(pgrep -f 'build/px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|mpc_control/lib' | grep -vw $$ || true)" ]; then
  nshm=$(ls /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
  if [ "$nshm" -gt 0 ]; then
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    echo "STALE-SHM-REMOVED: $nshm"
  fi
else
  echo "STALE-SHM-KEPT (仿真参与者仍在跑)"
fi
