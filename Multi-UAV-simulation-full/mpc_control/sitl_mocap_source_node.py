#!/usr/bin/env python3
"""SITL 专用：用 Gazebo 真值冒充动捕，发布 VRPN 格式的 PoseStamped。

    /px4_N/fmu/out/vehicle_local_position_groundtruth  (NED, Gazebo 世界系)
    /px4_N/fmu/out/vehicle_attitude_groundtruth        (q: FRD→NED)
                        ↓ 本节点（NED→ENU，冒充 VRPN）
    /vrpn_mocap/<刚体>/pose  (PoseStamped, ENU)
                        ↓ mocap_bridge_node（与真机同一份代码）
    /fmu/in/vehicle_visual_odometry

**真机上本节点不启动**，换成 `ros2 launch vrpn_mocap client.launch.yaml`。

⚠️ 依赖 PX4 侧改动：`vehicle_*_groundtruth` 两条话题默认不在
`dds_topics.yaml` 里，需加入 publications 段后重编 px4_sitl_default
（见 `docs/SITL动捕注入.md`）。真值由 `GZBridge.cpp` 发布，取自 Gazebo
物理状态、**不经 EKF**，因此用它验证 EKF 不构成循环论证。

⚠️ **真值是 Gazebo 世界系（所有机共享一个原点）**，与飞场动捕形态一致，
而非"各机自己的出生点系"——这正是本管线要提前暴露的坐标系差异。

噪声/延迟/丢失注入默认全关，打开可用于压测 EKF 与 bridge 看门狗。
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from mpc_control.frame_convert import swap_enu_ned_pos, swap_enu_ned_quat
from mpc_control.mocap_bridge_node import topic_for_drone


def px4_sub_qos():
    """订阅 PX4 out 话题：PX4 DataWriter 是 TRANSIENT_LOCAL。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class SitlMocapSource(Node):

    def __init__(self):
        super().__init__('sitl_mocap_source')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('rigid_body', '')
        self.declare_parameter('pose_topic', '')
        self.declare_parameter('rate_hz', 100.0)      # 飞场要求 ≥50Hz
        self.declare_parameter('frame_id', 'world')
        # --- 可选的真实性注入（默认全关）---
        self.declare_parameter('noise_pos_m', 0.0)    # 位置高斯噪声 std
        self.declare_parameter('noise_yaw_deg', 0.0)  # 偏航高斯噪声 std
        self.declare_parameter('latency_ms', 0.0)     # 传输延迟
        self.declare_parameter('dropout_start_s', -1.0)   # <0 = 不注入
        self.declare_parameter('dropout_duration_s', 0.0)
        # 静态位置偏移（ENU，米）。用于**单机模拟"刚体名↔drone_id 接错"或
        # "动捕原点未对齐"**——这两种故障在真机上表现为"位姿完全正常、只是整体
        # 平移了"，是 calib_shared_origin 门控要抓的对象。默认 0 = 不偏移。
        self.declare_parameter('offset_enu', [0.0, 0.0, 0.0])
        self.declare_parameter('stats_period_s', 5.0)

        self.drone_id = int(self.get_parameter('drone_id').value)
        rigid = str(self.get_parameter('rigid_body').value)
        topic = str(self.get_parameter('pose_topic').value)
        if not topic:
            if not rigid:
                rigid = f'drone{self.drone_id}'
            topic = f'/vrpn_mocap/{rigid}/pose'
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.noise_pos = float(self.get_parameter('noise_pos_m').value)
        self.noise_yaw = math.radians(float(self.get_parameter('noise_yaw_deg').value))
        self.latency_s = float(self.get_parameter('latency_ms').value) / 1000.0
        self.drop_start = float(self.get_parameter('dropout_start_s').value)
        self.drop_dur = float(self.get_parameter('dropout_duration_s').value)
        self.offset_enu = np.array(
            [float(v) for v in self.get_parameter('offset_enu').value], dtype=float)

        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.create_subscription(
            VehicleLocalPosition,
            topic_for_drone(self.drone_id, 'out/vehicle_local_position_groundtruth'),
            self.on_lpos, px4_sub_qos())
        self.create_subscription(
            VehicleAttitude,
            topic_for_drone(self.drone_id, 'out/vehicle_attitude_groundtruth'),
            self.on_att, px4_sub_qos())

        self._pos_ned = None
        self._q_ned = None
        self._buf = deque()          # 延迟注入用：(可发布时刻, pos, q)
        self._rng = np.random.default_rng(12345 + self.drone_id)
        self._t0 = self.get_clock().now()
        self._n_pub = 0
        self._n_drop = 0
        self._last_stats = self._t0

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self.on_tick)
        self.create_timer(float(self.get_parameter('stats_period_s').value),
                          self.on_stats)

        self.get_logger().info(
            f'[sitl_mocap] veh {self.drone_id}: 真值 → {topic} @{rate:.0f}Hz'
            + (f' 噪声{self.noise_pos}m' if self.noise_pos > 0 else '')
            + (f' 延迟{self.latency_s * 1000:.0f}ms' if self.latency_s > 0 else '')
            + (f' 断流@{self.drop_start}s×{self.drop_dur}s' if self.drop_start >= 0 else '')
            + (f' **静态偏移{self.offset_enu.tolist()}(ENU)**'
               if float(np.linalg.norm(self.offset_enu)) > 1e-9 else '')
        )

    # ------------------------------------------------------------------
    def on_lpos(self, msg: VehicleLocalPosition):
        if math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z):
            self._pos_ned = np.array([msg.x, msg.y, msg.z], dtype=float)

    def on_att(self, msg: VehicleAttitude):
        q = np.array(msg.q, dtype=float)     # [w,x,y,z]，FRD→NED
        if np.all(np.isfinite(q)) and float(np.linalg.norm(q)) > 1e-6:
            self._q_ned = q / float(np.linalg.norm(q))

    # ------------------------------------------------------------------
    def _in_dropout(self, t_s):
        return self.drop_start >= 0 and self.drop_start <= t_s < self.drop_start + self.drop_dur

    def on_tick(self):
        if self._pos_ned is None or self._q_ned is None:
            return
        now = self.get_clock().now()
        t_s = (now - self._t0).nanoseconds / 1e9

        if self._in_dropout(t_s):
            self._n_drop += 1
            return

        pos = self._pos_ned.copy()
        q = self._q_ned.copy()

        if self.noise_pos > 0:
            pos = pos + self._rng.normal(0.0, self.noise_pos, 3)
        if self.noise_yaw > 0:
            # 只扰偏航：绕 NED 的 z 轴叠加一个小旋转
            d = self._rng.normal(0.0, self.noise_yaw)
            qz = np.array([math.cos(d / 2), 0.0, 0.0, math.sin(d / 2)])
            w1, x1, y1, z1 = qz
            w2, x2, y2, z2 = q
            q = np.array([
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ])

        # NED→ENU：与 bridge 用的是同一对合函数（反向复用）
        pos_enu = swap_enu_ned_pos(pos) + self.offset_enu
        q_enu = swap_enu_ned_quat(q)

        release = now.nanoseconds / 1e9 + self.latency_s
        self._buf.append((release, pos_enu, q_enu))

        tnow = now.nanoseconds / 1e9
        while self._buf and self._buf[0][0] <= tnow:
            _, p, qq = self._buf.popleft()
            m = PoseStamped()
            m.header.stamp = now.to_msg()
            m.header.frame_id = self.frame_id
            m.pose.position.x = float(p[0])
            m.pose.position.y = float(p[1])
            m.pose.position.z = float(p[2])
            # 内部 [w,x,y,z] → ROS (x,y,z,w)
            m.pose.orientation.w = float(qq[0])
            m.pose.orientation.x = float(qq[1])
            m.pose.orientation.y = float(qq[2])
            m.pose.orientation.z = float(qq[3])
            self.pub.publish(m)
            self._n_pub += 1

    def on_stats(self):
        now = self.get_clock().now()
        dt = (now - self._last_stats).nanoseconds / 1e9
        self._last_stats = now
        if dt <= 0:
            return
        src = 'OK' if (self._pos_ned is not None and self._q_ned is not None) else '**无真值**'
        self.get_logger().info(
            f'[sitl_mocap] veh {self.drone_id}: 真值{src} pub={self._n_pub} '
            f'({self._n_pub / dt:.1f}Hz) 断流丢弃={self._n_drop}'
        )
        self._n_pub = self._n_drop = 0


def main(args=None):
    rclpy.init(args=args)
    node = SitlMocapSource()
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
