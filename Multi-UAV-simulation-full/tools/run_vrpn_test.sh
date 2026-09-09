#!/bin/bash
# 假 VRPN 服务器(NULL tracker) → vrpn_mocap，验证客户端产出 pose/twist/accel + QoS。
# 隔离域7，不碰飞控。对应 PDF 第四节的台架验证。
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 假服务器是 C++ 源(仓库只存 .cpp)；首次运行按当前架构编译出二进制
BIN="$SCRIPT_DIR/null_vrpn_server"
if [ ! -x "$BIN" ]; then
  echo "编译 null_vrpn_server ..."
  g++ "$SCRIPT_DIR/null_vrpn_server.cpp" -I/opt/ros/humble/include \
      -L/opt/ros/humble/lib -lvrpn -lquat -lpthread -o "$BIN" \
      || { echo "编译失败(检查 ros-humble-vrpn 是否安装)"; exit 1; }
fi

"$BIN" Tracker0 > /tmp/vrpn_srv.log 2>&1 &
SRV=$!
sleep 1
ros2 launch vrpn_mocap client.launch.yaml server:=localhost port:=3883 > /tmp/vrpn_mocap.log 2>&1 &
sleep 5

echo "=== ros2 topic list (PDF: 查看发布节点) ==="
ros2 topic list --no-daemon 2>/dev/null | grep vrpn_mocap

for t in pose twist accel; do
  echo "=== echo /vrpn_mocap/Tracker0/$t (一帧) ==="
  timeout 4 ros2 topic echo /vrpn_mocap/Tracker0/$t --once --qos-reliability best_effort 2>&1 | head -16
done

# 清理：括号技巧防 pkill 自杀；连 launch 的孤儿 client_node 一起杀
kill $SRV 2>/dev/null
pkill -9 -f "vrpn_mocap/[c]lient_node" 2>/dev/null
pkill -9 -f "[n]ull_vrpn_server" 2>/dev/null
sleep 1
echo "=== 残留检查 ==="
pgrep -f "vrpn_mocap/[c]lient_node" >/dev/null && echo "❌ client_node 残留" || echo "✅ client_node 清干净"
pgrep -f "[n]ull_vrpn_server" >/dev/null && echo "❌ server 残留" || echo "✅ server 清干净"
