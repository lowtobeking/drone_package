#!/usr/bin/env python3
"""
Virtual leader node.

Publishes the leader's NED state on /leader/state at a fixed rate.
The leader follows a rectangular path (4 waypoints) at constant speed and altitude.

State message: geometry_msgs/PoseStamped + TwistStamped is awkward;
we pack everything into a Float32MultiArray of length 8:
  [t, north, east, down, vN, vE, vD, yaw]

This avoids needing a custom .msg definition, keeping the package pure-Python.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray


class VirtualLeaderNode(Node):
    def __init__(self):
        super().__init__('virtual_leader_node')

        # Parameters
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('altitude', -5.0)
        self.declare_parameter('publish_hz', 20.0)
        # Waypoints come as flat list [n0,e0, n1,e1, n2,e2, n3,e3]
        self.declare_parameter(
            'waypoints_flat',
            [0.0, 0.0, 0.0, 50.0, 50.0, 50.0, 50.0, 0.0]
        )

        self.speed = float(self.get_parameter('speed').value)
        self.altitude = float(self.get_parameter('altitude').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        wp_flat = list(self.get_parameter('waypoints_flat').value)
        if len(wp_flat) % 2 != 0 or len(wp_flat) < 4:
            self.get_logger().error(
                f'waypoints_flat must have even length >= 4, got {len(wp_flat)}'
            )
            raise RuntimeError('bad waypoints')
        self.waypoints = [(wp_flat[i], wp_flat[i+1]) for i in range(0, len(wp_flat), 2)]
        self.num_wp = len(self.waypoints)

        # Pre-compute leg lengths and total perimeter
        self.leg_lengths = []
        for i in range(self.num_wp):
            n0, e0 = self.waypoints[i]
            n1, e1 = self.waypoints[(i + 1) % self.num_wp]
            d = math.hypot(n1 - n0, e1 - e0)
            self.leg_lengths.append(d)
        self.perimeter = sum(self.leg_lengths)
        self.cum_length = [0.0]
        for d in self.leg_lengths:
            self.cum_length.append(self.cum_length[-1] + d)

        # QoS - leader state should be reliable enough but recent
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher = self.create_publisher(
            Float32MultiArray, '/leader/state', qos
        )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(
            1.0 / self.publish_hz, self.publish_leader_state
        )

        self.get_logger().info(
            f'Virtual leader started. Perimeter={self.perimeter:.1f}m, '
            f'speed={self.speed}m/s, alt={self.altitude}m, '
            f'lap_time={self.perimeter/self.speed:.1f}s'
        )

    def publish_leader_state(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        # Distance traveled, wrapped to perimeter
        dist = (self.speed * t) % self.perimeter

        # Find which leg we are on
        leg = 0
        for i in range(self.num_wp):
            if dist <= self.cum_length[i + 1]:
                leg = i
                break

        n0, e0 = self.waypoints[leg]
        n1, e1 = self.waypoints[(leg + 1) % self.num_wp]
        leg_dist = dist - self.cum_length[leg]
        leg_len = self.leg_lengths[leg]

        if leg_len < 1e-6:
            ratio = 0.0
        else:
            ratio = leg_dist / leg_len

        north = n0 + ratio * (n1 - n0)
        east = e0 + ratio * (e1 - e0)
        down = self.altitude

        # Velocity vector along this leg
        if leg_len < 1e-6:
            vN = 0.0
            vE = 0.0
        else:
            vN = self.speed * (n1 - n0) / leg_len
            vE = self.speed * (e1 - e0) / leg_len
        vD = 0.0

        # Yaw in NED: yaw = 0 means pointing north, positive means rotating toward east
        yaw = math.atan2(vE, vN)

        msg = Float32MultiArray()
        msg.data = [
            float(t),
            float(north),
            float(east),
            float(down),
            float(vN),
            float(vE),
            float(vD),
            float(yaw),
        ]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualLeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
