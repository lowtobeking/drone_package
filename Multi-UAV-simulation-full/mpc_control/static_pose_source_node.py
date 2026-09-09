#!/usr/bin/env python3
"""台架专用：合成一个**静态**假位姿冒充 VRPN，打通无定位台架的注入链路。

    本节点（发布 PoseStamped, ENU, 恒定）
        → /vrpn_mocap/<刚体>/pose
        → mocap_bridge_node（真机同一份代码，不改）
        → /fmu/in/vehicle_visual_odometry → EKF2（EV 融合）

用途（**仅无桨台架**，与 [[mocap-injection-pipeline]] 第一步实验对应）
------------------------------------------------------------------
在没有真动捕、也没有 GPS 的台架上，注入"飞机静止在原点"的假位姿，验证：
  ① 注入管线在真机上是否连通（bridge in/out 计数、EKF 是否收到 EV）
  ② `ekf2_mocap` 参数是否正确、坐标系是否对得上
  ③ EKF2 能否收敛出 valid 的 local position（xy_valid / z_valid）

**为什么静态、为什么是原点**：飞机实际静止在桌上，真实 IMU 报告"没动"。
注入"静止在原点"的位姿与 IMU 一致 → EKF 不会因视觉/IMU 矛盾而拒绝融合或发散。
这一步**只验链路与收敛**，不涉及运动、不解锁、不转电机。

🔴🔴 安全红线
--------------
* **仅限无桨台架**。假位姿不反映飞机真实位置。
* **绝对不能在装桨状态下用它解锁飞行** —— 飞机会按假位置反馈乱冲。
* 装桨前务必停掉本节点、换成真 vrpn_mocap 客户端。

发布的是**恒定值**：位置默认 (0,0,0) ENU，姿态默认单位四元数（机头朝东、水平）。
可用参数微调，但台架第一步建议保持默认。
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


def vrpn_pub_qos():
    """匹配 mocap_bridge_node.mocap_sub_qos()：BEST_EFFORT / VOLATILE。

    真 vrpn_mocap 客户端也是 BEST_EFFORT 发布，这里保持一致，
    以免 QoS 不兼容导致 bridge 收不到（真机多机静默失配的经典坑）。
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.VOLATILE,
    )


class StaticPoseSource(Node):

    def __init__(self):
        super().__init__('static_pose_source')

        # 刚体名：与 mocap_bridge_node 默认一致（drone_id=0 → 'drone0'）。
        self.declare_parameter('rigid_body', 'drone0')
        # 恒定位置（ENU，米）与姿态（四元数 x,y,z,w）。默认原点 + 单位姿态。
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)
        # 发布频率（Hz）。VRPN 常见 100~200Hz；EKF EV 融合 ≥ 30Hz 即可。
        # [[mocap-injection-pipeline]] 实测 50Hz 足够、省一半带宽，故默认 50。
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('log_period_s', 5.0)

        rigid = str(self.get_parameter('rigid_body').value)
        self.topic = f'/vrpn_mocap/{rigid}/pose'
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.qx = float(self.get_parameter('qx').value)
        self.qy = float(self.get_parameter('qy').value)
        self.qz = float(self.get_parameter('qz').value)
        self.qw = float(self.get_parameter('qw').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.log_period = float(self.get_parameter('log_period_s').value)

        # 归一化四元数，避免 bridge 因非单位四元数丢帧
        n = (self.qx**2 + self.qy**2 + self.qz**2 + self.qw**2) ** 0.5
        if n < 1e-6:
            self.get_logger().error(
                '[static_pose] 四元数全零，回退到单位四元数 (0,0,0,1)')
            self.qx, self.qy, self.qz, self.qw = 0.0, 0.0, 0.0, 1.0
        else:
            self.qx, self.qy, self.qz, self.qw = (
                self.qx / n, self.qy / n, self.qz / n, self.qw / n)

        self.pub = self.create_publisher(PoseStamped, self.topic, vrpn_pub_qos())
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self._n = 0
        self._last_log = self.get_clock().now()

        self.get_logger().warn(
            '🔴 台架专用【静态假位姿】源已启动 —— 仅限无桨台架，'
            '绝不可用于装桨飞行！')
        self.get_logger().info(
            f'[static_pose] 发布 {self.topic} @ {rate:.0f}Hz  '
            f'ENU pos=({self.x:.2f},{self.y:.2f},{self.z:.2f}) '
            f'quat=({self.qx:.3f},{self.qy:.3f},{self.qz:.3f},{self.qw:.3f})')

    def on_timer(self):
        msg = PoseStamped()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'  # ENU 世界系（仅供追溯，bridge 不读）
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        msg.pose.orientation.x = self.qx
        msg.pose.orientation.y = self.qy
        msg.pose.orientation.z = self.qz
        msg.pose.orientation.w = self.qw
        self.pub.publish(msg)

        self._n += 1
        dt = (now - self._last_log).nanoseconds / 1e9
        if dt >= self.log_period:
            self.get_logger().info(
                f'[static_pose] 已发布 {self._n} 帧（{self._n / dt:.0f}Hz）')
            self._n = 0
            self._last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = StaticPoseSource()
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
