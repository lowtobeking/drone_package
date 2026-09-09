#!/usr/bin/env python3
"""验证 Jetson→飞控 上行(XRCE)：发 OffboardControlMode(body_rate) 心跳，
看 failsafe_flags.offboard_control_signal_lost 是否 true->false->true。
判据用 body_rate（非 velocity/position，后者无定位恒 false 与链路无关）。全程只发心跳，不解锁。"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, FailsafeFlags

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=10,
                 durability=DurabilityPolicy.VOLATILE)


class V(Node):
    def __init__(self):
        super().__init__('verify_uplink')
        self.pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', QOS)
        self.sub = self.create_subscription(FailsafeFlags, '/fmu/out/failsafe_flags',
                                             self.on_ff, QOS)
        self.flag = None      # 最新 offboard_control_signal_lost
        self.n_ff = 0
        self.sending = False
        self.create_timer(0.05, self.tick)  # 20Hz

    def on_ff(self, m):
        self.n_ff += 1
        v = bool(m.offboard_control_signal_lost)
        if v != self.flag:
            self.get_logger().info(f'  offboard_control_signal_lost -> {v}')
        self.flag = v

    def tick(self):
        if not self.sending:
            return
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    v = V()

    def spin(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(v, timeout_sec=0.05)

    print('=== 阶段1: 不发心跳 2.5s（期望 lost=True，即上行"没信号"）===')
    spin(2.5)
    s1 = v.flag
    print(f'   -> flag={s1}  (收到 {v.n_ff} 条 failsafe_flags)')

    print('=== 阶段2: 20Hz 发 body_rate 心跳 4s（期望翻成 lost=False = 上行到达飞控）===')
    v.sending = True
    spin(4.0)
    s2 = v.flag

    print('=== 阶段3: 停发 3s（期望 ~1s 后翻回 lost=True）===')
    v.sending = False
    spin(3.0)
    s3 = v.flag

    print(f'\n结果: 阶段1={s1}  阶段2={s2}  阶段3={s3}')
    ok = (s1 is True and s2 is False and s3 is True)
    if ok:
        print('✅ 上行确认: True->False->True，Jetson 发的指令确实到达飞控并被处理')
    elif v.n_ff == 0:
        print('❌ 一条 failsafe_flags 都没收到 —— 下行订阅/QoS 有问题')
    else:
        print('⚠️ 未出现预期跳变，见上方各阶段值（若阶段2仍 True=上行没到）')
    v.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
