#!/usr/bin/env python3
"""动捕注入链路三点同步核验。

    ros2 run 环境下：python3 tools/verify_mocap_injection.py [--drone-id 0] [--secs 30]

同时订阅链路上的三个点，持续检查它们之间**应当成立的代数关系**：

    ①真值(NED)  ──F──▶ ②/vrpn_mocap/<刚体>/pose(ENU) ──F──▶ ③EKF 估计(NED)

- ①→② 检查 ENU 关系：x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
- ①→③ 检查端到端：EKF 估计应重现真值

⚠️ **为什么不能只查 ①→③**：源与 bridge 用的是同一个对合函数 F 正反各调一次，
F∘F ≡ 恒等 —— **即使 F 本身写错，往返也会自动抵消**，①③ 照样完美吻合，而
中间那一跳（真机上要喂给真 VRPN 消费者、也是飞场排障时唯一能看的东西）
是错的。所以 ①→② 这一段必须单独查。

真机上没有 ①（没有真值），只能查 ②→③ 的一致性，但 ② 本身的正确性届时由
动捕软件保证，性质不同。
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


def px4_sub_qos():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Verifier(Node):

    def __init__(self, drone_id, rigid, secs):
        super().__init__('verify_mocap_injection')
        ns = '' if drone_id == 0 else f'/px4_{drone_id}'
        self.secs = secs

        self.truth = None
        self.pose = None
        self.ekf = None
        # 各项误差的运行最大值
        self.max_enu = 0.0      # ①→② 关系误差
        self.max_e2e = 0.0      # ①→③ 端到端误差
        self.n = 0
        self.max_excursion = 0.0   # 真值离原点最远距离（判断样本是否含运动）

        self.create_subscription(
            VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position_groundtruth',
            self.on_truth, px4_sub_qos())
        self.create_subscription(
            VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position',
            self.on_ekf, px4_sub_qos())
        self.create_subscription(
            PoseStamped, f'/vrpn_mocap/{rigid}/pose', self.on_pose,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10,
                       durability=DurabilityPolicy.VOLATILE))

        self.create_timer(0.05, self.on_tick)
        self.create_timer(1.0, self.on_report)
        self._elapsed = 0.0

    def on_truth(self, m):
        if all(math.isfinite(v) for v in (m.x, m.y, m.z)):
            self.truth = (m.x, m.y, m.z)

    def on_ekf(self, m):
        if m.xy_valid and m.z_valid:
            self.ekf = (m.x, m.y, m.z)

    def on_pose(self, m):
        p = m.pose.position
        self.pose = (p.x, p.y, p.z)

    def on_tick(self):
        if self.truth is None or self.pose is None or self.ekf is None:
            return
        tx, ty, tz = self.truth
        px, py, pz = self.pose
        ex, ey, ez = self.ekf

        # ①→②：ENU 关系
        e_enu = max(abs(px - ty), abs(py - tx), abs(pz + tz))
        # ①→③：端到端
        e_e2e = max(abs(ex - tx), abs(ey - ty), abs(ez - tz))

        self.max_enu = max(self.max_enu, e_enu)
        self.max_e2e = max(self.max_e2e, e_e2e)
        self.max_excursion = max(self.max_excursion, math.hypot(tx, ty))
        self.n += 1

    def on_report(self):
        self._elapsed += 1.0
        have = all(x is not None for x in (self.truth, self.pose, self.ekf))
        if not have:
            miss = [n for n, v in (('真值', self.truth), ('vrpn位姿', self.pose),
                                   ('EKF', self.ekf)) if v is None]
            self.get_logger().warn(f'等待话题: {", ".join(miss)}')
            return
        self.get_logger().info(
            f'[{int(self._elapsed)}s] n={self.n} '
            f'①→②ENU关系 max={self.max_enu:.6f}m  '
            f'①→③端到端 max={self.max_e2e:.6f}m  '
            f'真值最大水平位移={self.max_excursion:.3f}m'
        )
        if self._elapsed >= self.secs:
            self.finish()

    def finish(self):
        print('\n' + '=' * 58)
        print(f'样本 {self.n}')
        print(f'①→② ENU 关系   最大误差 {self.max_enu:.6f} m')
        print(f'①→③ 端到端     最大误差 {self.max_e2e:.6f} m')
        print(f'真值最大水平位移 {self.max_excursion:.3f} m')
        ok = True
        if self.max_enu > 0.05:
            print('❌ ENU 关系不成立 —— 中间跳的坐标系是错的'); ok = False
        if self.max_e2e > 0.10:
            print('❌ 端到端偏差过大 —— EKF 未正确跟随注入'); ok = False
        if self.max_excursion < 0.5:
            print('⚠️ 全程近乎静止：x/y 互换类错误在原点附近不可见，')
            print('   本次结果**不足以**判定 x/y 映射正确，需在运动中复测。')
            ok = False
        print('✅ 通过' if ok else '未通过')
        print('=' * 58)
        rclpy.shutdown()
        sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drone-id', type=int, default=0)
    ap.add_argument('--rigid-body', default='')
    ap.add_argument('--secs', type=float, default=30.0)
    a = ap.parse_args()
    rigid = a.rigid_body or f'drone{a.drone_id}'

    rclpy.init()
    node = Verifier(a.drone_id, rigid, a.secs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()


if __name__ == '__main__':
    main()
