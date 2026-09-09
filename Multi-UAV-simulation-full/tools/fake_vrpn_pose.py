#!/usr/bin/env python3
"""真机台架用：从零合成一个静态 ENU 位姿，冒充 VRPN 发布 PoseStamped。

无动捕时，用它验证 `mocap_bridge_node → vehicle_visual_odometry → 真机 EKF2`
整条注入链路 + 坐标系变换 + EKF2 外部视觉融合配置是否正确（不依赖 Gazebo，
故区别于 `mpc_control/sitl_mocap_source_node.py`——后者的真值取自 Gazebo
groundtruth 话题，真机上没有）。

**静态位姿**与台架静止的 IMU 自洽（EV 说静止、加速度计也说静止），
不会引发 EKF 拒绝；且不解锁、不动电机，台架安全。

验证方法（EKF2 动捕参数已配 + 飞控已重启后）：
    # 1) 起桥
    ros2 run mpc_control mocap_bridge_node --ros-args -p drone_id:=0
    # 2) 起本节点（另一终端）
    python3 tools/fake_vrpn_pose.py 1.0 2.0 1.5
    # 3) 看 EKF 是否收敛（注入 ENU(x,y,z) 应得 EKF NED(y, x, -z)）
    ros2 topic echo --once /fmu/out/vehicle_local_position \
        --qos-reliability best_effort --qos-durability transient_local
    ros2 topic echo --once /fmu/out/estimator_status_flags \
        --qos-reliability best_effort --qos-durability transient_local
    # 判据：xy_valid/z_valid=true、位置≈NED(y,x,-z)、cs_ev_pos/yaw/hgt=true

2026-07-27 真机（X6 Air+ v1.16 / Orin NX）实测通过：注入 ENU(1,2,1.5) →
EKF NED(1.9997,1.0003,-1.4998)、heading=90°、cs_ev_* 全 True、cs_ev_yaw_fault=false。

用法: python3 fake_vrpn_pose.py [x_enu y_enu z_enu [topic]]
默认: 1.0 2.0 1.5 /vrpn_mocap/drone0/pose
"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class FakeVrpn(Node):
    def __init__(self, topic, x, y, z):
        super().__init__('fake_vrpn_pose')
        self.x, self.y, self.z = x, y, z
        # depth 10、默认 RELIABLE：mocap_bridge 订阅是 BEST_EFFORT，RELIABLE 发布
        # 可喂 BEST_EFFORT 订阅（反之不行），兼容。
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.n = 0
        self.create_timer(0.01, self.tick)   # 100 Hz（飞场要求 ≥50Hz）
        self.create_timer(2.0, self.stat)
        self.get_logger().info(
            f'[fake_vrpn] 静态 ENU 位姿 ({x},{y},{z}) → {topic} @100Hz')

    def tick(self):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.pose.position.x = self.x
        m.pose.position.y = self.y
        m.pose.position.z = self.z
        m.pose.orientation.w = 1.0   # ENU 单位四元数（机头朝东）
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        self.pub.publish(m)
        self.n += 1

    def stat(self):
        self.get_logger().info(f'[fake_vrpn] pub n={self.n}')


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    z = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
    topic = sys.argv[4] if len(sys.argv) > 4 else '/vrpn_mocap/drone0/pose'
    rclpy.init()
    node = FakeVrpn(topic, x, y, z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
