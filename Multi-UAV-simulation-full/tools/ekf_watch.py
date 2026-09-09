#!/usr/bin/env python3
"""只读监视真机 EKF 状态：vehicle_local_position 的 xy_valid/z_valid/pos +
failsafe_flags.local_position_invalid。用法: ekf_watch.py <秒> <标签>。纯订阅，不发任何指令。"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, FailsafeFlags

# 匹配 mpc_node.make_px4_qos()：PX4 /fmu/out DataWriter 用 TRANSIENT_LOCAL，
# 订阅端必须匹配（实测 VOLATILE 收不到——本会话踩到、mpc_node 注释亦明写）。
QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=5,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)


class W(Node):
    def __init__(self):
        super().__init__('ekf_watch')
        self.lp = None
        self.ff = None
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position', self._lp, QOS)
        self.create_subscription(FailsafeFlags,
                                 '/fmu/out/failsafe_flags', self._ff, QOS)

    def _lp(self, m): self.lp = m
    def _ff(self, m): self.ff = m


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    tag = sys.argv[2] if len(sys.argv) > 2 else ''
    rclpy.init()
    w = W()
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < dur:
        rclpy.spin_once(w, timeout_sec=0.1)
        if time.time() - last >= 1.0:
            last = time.time()
            lp, ff = w.lp, w.ff
            xy = lp.xy_valid if lp else None
            z = lp.z_valid if lp else None
            pos = (round(lp.x, 3), round(lp.y, 3), round(lp.z, 3)) if lp else None
            lpi = ff.local_position_invalid if ff else None
            print(f'[{tag}] xy_valid={xy} z_valid={z} pos={pos} '
                  f'local_position_invalid={lpi}', flush=True)
    w.destroy_node()
    rclpy.shutdown()


main()
