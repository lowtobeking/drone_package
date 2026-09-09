#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上行通路验证：只发 OffboardControlMode(body_rate=True) 心跳，看飞控的
failsafe_flags.offboard_control_signal_lost 是否 true→false。

🔴 安全边界（本脚本严格遵守）：
   · 不发 ARM / 任何 VehicleCommand
   · 不发 TrajectorySetpoint / 任何 setpoint
   · 不请求切模式
   ⇒ 电机不会转，飞机不会解锁。纯粹验证"Jetson 发的包飞控收到了并作出反应"。

为什么用 body_rate 而不是 velocity：PX4 的 offboardCheck 对 velocity/position/
acceleration 都要查 local_position 有效性，室内无定位必判 false，与链路好坏无关
（此坑记忆里记过，曾差点误判成链路故障）。body_rate 不查定位。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, FailsafeFlags

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 history=HistoryPolicy.KEEP_LAST, depth=1)


class UplinkTest(Node):
    def __init__(self):
        super().__init__('uplink_test')
        self.pub = self.create_publisher(OffboardControlMode,
                                         '/fmu/in/offboard_control_mode', QOS)
        self.create_subscription(FailsafeFlags, '/fmu/out/failsafe_flags',
                                 self.on_flags, QOS)
        self.flag = None
        self.history = []
        self.sending = False
        self.create_timer(0.1, self.tick)          # 10 Hz 心跳
        self.t0 = time.time()
        self.phase = 'idle'

    def on_flags(self, msg):
        v = bool(msg.offboard_control_signal_lost)
        if v != self.flag:
            self.get_logger().info(
                '  [%5.1fs] offboard_control_signal_lost: %s → %s   (阶段: %s)'
                % (time.time() - self.t0, self.flag, v, self.phase))
            self.history.append((round(time.time() - self.t0, 1), self.flag, v, self.phase))
        self.flag = v

    def tick(self):
        t = time.time() - self.t0
        # 0-3s 静默基线 → 3-13s 发心跳 → 13-20s 停发（看是否翻回）
        if t < 3:
            self.phase = '静默基线(应为 true)'
            self.sending = False
        elif t < 13:
            self.phase = '发心跳(应翻 false)'
            self.sending = True
        elif t < 20:
            self.phase = '停发(应翻回 true)'
            self.sending = False
        else:
            raise SystemExit(0)

        if self.sending:
            m = OffboardControlMode()
            m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            m.position = False
            m.velocity = False
            m.acceleration = False
            m.attitude = False
            m.body_rate = True          # ← 唯一置位项，且不查定位
            m.thrust_and_torque = False
            m.direct_actuator = False
            self.pub.publish(m)


def main():
    rclpy.init()
    n = UplinkTest()
    print('=== 上行通路验证（不解锁、不发 setpoint、电机不会转）===')
    print('    0-3s 静默 → 3-13s 发 body_rate 心跳 → 13-20s 停发\n')
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    print('\n=== 翻转记录 ===')
    if not n.history:
        print('  ❌ 全程无变化（当前值 %s）—— 上行没通' % n.flag)
    for t, old, new, ph in n.history:
        print('  %5.1fs  %s → %s   [%s]' % (t, old, new, ph))
    ok = any(o is True and nw is False for _, o, nw, _ in n.history)
    print('\n结论：%s' % ('✅ 上行通路正常（飞控收到了 Jetson 发的包并作出反应）'
                          if ok else '❌ 未见 true→false 翻转，上行可能不通'))
    n.destroy_node()
    rclpy.shutdown()


main()
