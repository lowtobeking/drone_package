#!/bin/bash
# 阶段C：静态假位姿注入真机 FC，观察 EKF 收敛。无桨台架、不解锁、只喂 EKF。domain 0。
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   # 依赖同目录的 ekf_watch.py
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "########## 1) 注入前基线 (4s，期望 xy/z_valid=False, local_position_invalid=True) ##########"
python3 "$SCRIPT_DIR/ekf_watch.py" 4 BEFORE

echo "########## 2) 起注入 static_pose_source(原点) + bridge ##########"
ros2 run mpc_control static_pose_source_node > /tmp/stagec_src.log 2>&1 &
SRC=$!
sleep 1
ros2 run mpc_control mocap_bridge_node --ros-args -p drone_id:=0 -p rigid_body:=drone0 > /tmp/stagec_bridge.log 2>&1 &
BRG=$!
echo "SRC=$SRC BRG=$BRG"

echo "########## 3) 注入中看 EKF 收敛 (18s，期望 xy/z_valid 翻 True) ##########"
python3 "$SCRIPT_DIR/ekf_watch.py" 18 INJECT

echo "########## 注入链路健康 ##########"
echo "--- bridge 日志尾 ---"; tail -4 /tmp/stagec_bridge.log
echo "--- /fmu/in/vehicle_visual_odometry 速率 ---"
timeout 4 ros2 topic hz /fmu/in/vehicle_visual_odometry 2>&1 | grep -m1 "average rate" || echo "(未算出)"

echo "########## 4) 停注入, 看是否回落 invalid (证明 valid 是注入所致) ##########"
kill $SRC $BRG 2>/dev/null
sleep 1
kill -9 $SRC $BRG 2>/dev/null
pkill -9 -f "[s]tatic_pose_source_node" 2>/dev/null
pkill -9 -f "[m]ocap_bridge_node" 2>/dev/null
sleep 3
python3 "$SCRIPT_DIR/ekf_watch.py" 10 AFTER

echo "########## 残留检查 ##########"
pgrep -f "[s]tatic_pose_source_node" >/dev/null && echo "❌ src 残留" || echo "✅ src 清"
pgrep -f "[m]ocap_bridge_node" >/dev/null && echo "❌ bridge 残留" || echo "✅ bridge 清"
