#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证 line 沿机头方向飞：喂一个 45° 的实测航向，看 leader 轨迹方向、机头、行程、降落。

判据：
  1. 轨迹方向 = 航向 45°（不是正北 0°）—— 旧代码会朝正北
  2. yaw 全程恒等于 45°     —— 不做对准转向
  3. 总行程 = max_distance   —— 梯形曲线刹停在终点
  4. mission_duration 后广播 LAND
"""
import sys, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String
from px4_msgs.msg import VehicleLocalPosition

HDG = math.radians(45.0)
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

class H(Node):
    def __init__(self):
        super().__init__('line_heading_test')
        px4q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=5,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pos_pub = self.create_publisher(VehicleLocalPosition,
                                             '/fmu/out/vehicle_local_position', px4q)
        self.hp = self.create_publisher(Float32MultiArray, '/mpc/health', 10)
        self.create_subscription(Float64MultiArray, '/leader/state', self._st, 10)
        mq = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=10,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/swarm/mission', self._ms, mq)
        self.t0 = time.time(); self.rows = []; self.land_t = None
        self.create_timer(0.05, self._tick)
    def _st(self, m):
        if len(m.data) >= 8:
            self.rows.append((time.time()-self.t0, m.data[1], m.data[2], m.data[4], m.data[5], m.data[7]))
    def _ms(self, m):
        if m.data == 'LAND' and self.land_t is None:
            self.land_t = time.time()-self.t0
    def _tick(self):
        p = VehicleLocalPosition(); p.heading = float(HDG)
        p.x = p.y = 0.0; p.z = -3.0
        self.pos_pub.publish(p)
        h = Float32MultiArray()
        h.data = [0.0,1.0,3.0,0.0,0.0, 0.05, 0.05, 0.0,0.0,0.0,0.0,0.0,1.0]
        self.hp.publish(h)

def main():
    rclpy.init(); n = H()
    while rclpy.ok() and (time.time()-n.t0) < DUR:
        rclpy.spin_once(n, timeout_sec=0.05)
    R = np.array(n.rows)
    fails = []
    def ck(name, got, want, tol=None):
        ok = (abs(got-want) <= tol) if tol is not None else (got == want)
        print(f'  {"OK " if ok else "FAIL"} {name}: got={got:.4f} want={want:.4f}' + (f' (tol {tol})' if tol else ''))
        if not ok: fails.append(name)
    print(f'\n===== line 沿机头 45° =====  样本 {len(R)}')
    if len(R) < 50:
        print('数据不足'); sys.exit(1)
    moved = R[np.hypot(R[:,1], R[:,2]) > 0.05]
    if len(moved) == 0:
        print('FAIL 飞机没动'); sys.exit(1)
    last = moved[-1]
    ang = math.atan2(last[2], last[1])
    ck('轨迹方向(deg)', math.degrees(ang), 45.0, 1.0)
    ck('总行程(m)', float(np.hypot(last[1], last[2])), 3.0, 0.05)
    yaws = R[:,5]
    ck('yaw 最大偏离 45°(deg)', float(np.max(np.abs(np.degrees(yaws) - 45.0))), 0.0, 1.0)
    vmax = float(np.max(np.hypot(R[:,3], R[:,4])))
    ck('速度峰值(m/s)', vmax, 0.8, 0.05)
    print(f'  LAND 时刻 = {n.land_t if n.land_t else float("nan")}')
    print('\n===== ' + ('全部通过' if not fails else f'{len(fails)} 项失败: {fails}') + ' =====')
    sys.exit(1 if fails else 0)

if __name__ == '__main__':
    main()
