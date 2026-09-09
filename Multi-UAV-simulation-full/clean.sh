#!/bin/bash
# clean.sh — 一键彻底清理仿真残留进程 (PX4 / Gazebo / DDS / ROS2 节点)。
#
# 为什么需要它 / why your pkill 删不干净:
#   1) 不加 -9：gz sim server、px4 会忽略 SIGTERM → 残留。必须 SIGKILL。
#   2) `pkill -f ros2` 杀不到节点：mpc_node/leader_node 实际是
#      `python3 .../install/.../mpc_node` 进程,命令行里没有 "ros2" 字样,
#      所以匹配不到、一直活着。必须按 mpc_node / leader_node 等名字杀。
#   3) ros2 daemon 会留守 → 需 `ros2 daemon stop`。
#
# 用法: bash clean.sh
set +e

echo "清理仿真进程 (SIGKILL)..."
for p in px4 'gz sim' gzserver MicroXRCEAgent \
         mpc_node leader_node virtual_leader arming_node diag_monitor \
         swarm_launch 'ros2 launch'; do
    pkill -9 -f "$p"
done
ros2 daemon stop 2>/dev/null
pkill -9 -f 'ros2-daemon'
sleep 1

echo "=== 残留检查 (下面应为空) ==="
if ps aux | grep -E "px4|gz sim|gzserver|MicroXRCE|mpc_node|leader_node|diag_monitor|swarm_launch|ros2-daemon" | grep -v grep; then
    echo "⚠️ 仍有上面这些残留;个别 gz server 可能要按 PID 再 kill -9 <pid>"
else
    echo "✓ 干净,可以重新启动了"
fi
