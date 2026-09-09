#!/usr/bin/env python3
"""动捕位姿注入桥：geometry_msgs/PoseStamped → px4_msgs/VehicleOdometry。

飞场管线（`How_to_fly_with_control.md` §2-3）：

    动捕软件 → VRPN → vrpn_mocap 客户端 → /vrpn_mocap/<刚体>/pose (PoseStamped)
                                              ↓  **本节点**
                                    /fmu/in/vehicle_visual_odometry
                                              ↓  uXRCE-DDS
                                          PX4 EKF2 (EV 融合)

**同一份代码在 SITL 与真机上跑**：SITL 用 `sitl_mocap_source_node` 冒充 VRPN
发布 PoseStamped，真机换成 vrpn_mocap 客户端，本节点不需要任何改动。

坐标系（最容易错的一环）
------------------------
ROS/VRPN 用 **ENU + FLU**，PX4 用 **NED + FRD**：

  位置  N = E_enu(y)   E = E_enu(x)   D = -z_enu
  姿态  q_ned_frd = Q_ENU2NED ⊗ q_enu_flu ⊗ Q_FLU2FRD

其中 Q_ENU2NED = (0, √2/2, √2/2, 0)（绕 (1,1,0)/√2 转 180°），
     Q_FLU2FRD = (0, 1, 0, 0)（绕 X 转 180°）。
两者 w 均为 0（纯 180° 旋转）⇒ 各自是自逆的 ⇒ **该变换是对合**，
同一个函数正反两个方向都能用（`sitl_mocap_source_node` 反向用的就是它）。

自检：ENU 下机头朝东(q=单位) → NED 应得 yaw=+90°；机头朝北(yaw_enu=90°)
→ NED 应得 yaw=0。两例均已验证（见 `tools/test_frame_convert.py`）。

不融合速度
----------
飞场要求 `EKF2_EV_CTRL=11` = 0b1011 = 水平位置 + 垂直位置 + 偏航，**不含速度**
（bit2）。VRPN 本身也只给位姿。故 `velocity`/`angular_velocity` 一律填 NaN
——PX4 对这两个字段的约定是"NaN = 无效/未知"，填 0 会被当成"实测速度为零"。
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# 坐标变换在独立的纯 numpy 模块里，便于脱离 ROS 单独测（tools/test_frame_convert.py）
from mpc_control.frame_convert import swap_enu_ned_pos, swap_enu_ned_quat  # noqa: F401


def px4_pub_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.VOLATILE,
    )


def mocap_sub_qos():
    """订阅动捕位姿。

    用 BEST_EFFORT 是**故意的宽松选择**：RELIABLE 发布者可以喂 BEST_EFFORT
    订阅者，反过来不行。vrpn_mocap 的 QoS 我们没有实物确认过，选宽松的一侧
    可避免"节点正常、零报错、就是收不到"这种静默不匹配。
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def topic_for_drone(drone_id, suffix):
    """与 mpc_node.topic_for_drone 保持一致（drone0 不带命名空间）。"""
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'


class MocapBridge(Node):

    def __init__(self):
        super().__init__('mocap_bridge')

        self.declare_parameter('drone_id', 0)
        # 刚体名 → 话题。飞场刚体命名规则尚未确认，故留成可配。
        self.declare_parameter('rigid_body', '')
        self.declare_parameter('pose_topic', '')
        # 输入是否已是 NED（少数动捕软件可直接输出 NED；VRPN 默认 ENU）
        self.declare_parameter('input_ned', False)
        # 动捕失联判定：超过此时长没新位姿就告警（飞场遮挡是真实风险）
        self.declare_parameter('timeout_s', 0.3)
        # 注入方差。EKF2 另有 EKF2_EVP_NOISE/EKF2_EVA_NOISE，填 NaN 则用那两个参数。
        self.declare_parameter('pos_var', 1.0e-4)     # m²，动捕毫米级 → 1e-4 偏保守
        self.declare_parameter('orient_var', 1.0e-4)  # rad²
        self.declare_parameter('stats_period_s', 5.0)

        self.drone_id = int(self.get_parameter('drone_id').value)
        rigid = str(self.get_parameter('rigid_body').value)
        topic = str(self.get_parameter('pose_topic').value)
        if not topic:
            if not rigid:
                rigid = f'drone{self.drone_id}'
            topic = f'/vrpn_mocap/{rigid}/pose'
        self.input_ned = bool(self.get_parameter('input_ned').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.pos_var = float(self.get_parameter('pos_var').value)
        self.orient_var = float(self.get_parameter('orient_var').value)

        out_topic = topic_for_drone(self.drone_id, 'in/vehicle_visual_odometry')
        self.pub = self.create_publisher(VehicleOdometry, out_topic, px4_pub_qos())
        self.create_subscription(PoseStamped, topic, self.on_pose, mocap_sub_qos())

        self._topic = topic
        self._t_start = self.get_clock().now()
        self._never_warn_after = 5.0   # s，超此仍一帧未收到就开始报错
        self._n_in = 0
        self._n_out = 0
        self._n_bad = 0
        self._last_rx = None
        self._warned_stale = False
        self._last_stats = self.get_clock().now()

        period = float(self.get_parameter('stats_period_s').value)
        # 周期性打印：收尾用 SIGKILL 时 finally 不执行，退出汇总打不出来（踩过）
        self.create_timer(period, self.on_stats)
        self.create_timer(0.1, self.on_watchdog)

        self.get_logger().info(
            f'[mocap_bridge] veh {self.drone_id}: {topic} → {out_topic} '
            f'({"NED 直通" if self.input_ned else "ENU→NED 转换"})'
        )

    # ------------------------------------------------------------------
    def on_pose(self, msg: PoseStamped):
        self._n_in += 1
        self._last_rx = self.get_clock().now()
        self._warned_stale = False

        p = msg.pose.position
        o = msg.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        # ROS 的 Quaternion 是 (x,y,z,w)，本模块内部一律 [w,x,y,z]
        q = np.array([o.w, o.x, o.y, o.z], dtype=float)

        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(q))):
            self._n_bad += 1
            return
        n = float(np.linalg.norm(q))
        if n < 1e-6:
            self._n_bad += 1   # 全零四元数：刚体丢失时某些动捕软件会这么发
            return
        q = q / n

        if not self.input_ned:
            pos = swap_enu_ned_pos(pos)
            q = swap_enu_ned_quat(q)

        out = VehicleOdometry()
        # timestamp_sample = 测量时刻（EKF2 靠它做延迟补偿，EKF2_EV_DELAY 在此之上叠加）
        # timestamp        = 发送时刻。uXRCE-DDS 客户端负责与飞控时钟对齐。
        stamp_us = (int(msg.header.stamp.sec) * 1_000_000
                    + int(msg.header.stamp.nanosec) // 1000)
        now_us = int(self.get_clock().now().nanoseconds // 1000)
        out.timestamp_sample = stamp_us if stamp_us > 0 else now_us
        out.timestamp = now_us

        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = [float(v) for v in pos]
        out.q = [float(v) for v in q]

        # 不融合速度（EKF2_EV_CTRL=11 不含 bit2）；NaN = 无效，勿填 0
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [float('nan')] * 3
        out.angular_velocity = [float('nan')] * 3

        out.position_variance = [self.pos_var] * 3
        out.orientation_variance = [self.orient_var] * 3
        out.velocity_variance = [float('nan')] * 3
        out.reset_counter = 0
        out.quality = 0

        self.pub.publish(out)
        self._n_out += 1

    # ------------------------------------------------------------------
    def on_watchdog(self):
        """动捕断流检测。停止注入时 EKF 会在数秒内失去位置有效性，
        飞控按 `COM_OBL_RC_ACT=4` 降落 —— 这里只负责把原因喊出来。"""
        if self._last_rx is None:
            # **一帧都没收到过**：与"收到过又断了"是两种不同故障，必须分开喊。
            # 飞场上最常见的原因是 VRPN 客户端没起、或刚体名写错 —— 此时若只打
            # in=0 的统计行，很容易被当成"还没开始"而漏掉。
            dt = (self.get_clock().now() - self._t_start).nanoseconds / 1e9
            if dt > self._never_warn_after and int(dt) % 5 == 0:
                self.get_logger().error(
                    f'[mocap_bridge] veh {self.drone_id}: 启动 {dt:.0f}s 以来'
                    f'**一帧位姿都没收到** —— 订阅的是 {self._topic}。'
                    f'检查：vrpn_mocap 客户端起了吗？刚体名对吗？'
                    f'（`ros2 topic list | grep vrpn`）'
                )
            return
        dt = (self.get_clock().now() - self._last_rx).nanoseconds / 1e9
        if dt > self.timeout_s and not self._warned_stale:
            self._warned_stale = True
            self.get_logger().error(
                f'[mocap_bridge] veh {self.drone_id}: 动捕位姿已 {dt:.2f}s 无更新'
                f'（阈值 {self.timeout_s}s）—— 遮挡/刚体丢失/VRPN 断线？'
            )

    def on_stats(self):
        now = self.get_clock().now()
        dt = (now - self._last_stats).nanoseconds / 1e9
        self._last_stats = now
        if dt <= 0:
            return
        self.get_logger().info(
            f'[mocap_bridge] veh {self.drone_id}: in={self._n_in} out={self._n_out} '
            f'bad={self._n_bad} 出口速率≈{self._n_out / dt:.1f}Hz'
        )
        self._n_in = self._n_out = self._n_bad = 0


def main(args=None):
    rclpy.init(args=args)
    node = MocapBridge()
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
