#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""§10 A2 油门磁干扰测量 v2 —— 只读，不发任何指令（电机由飞手在 QGC 控制）。

v1 的教训：**EKF 航向收敛期的漂移(实测约 11°)本身就超过 10° 判据**，
在收敛完成前推电机，电流效应与收敛漂移完全混在一起、无法解读。
⇒ v2 先等航向稳定（10s 内峰峰 < 0.15°）才允许开始，并落盘原始数据做回归。
"""
import os, sys, time, math
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, BatteryStatus

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
# 🔴 默认不要落 /tmp：Jetson 无 RTC、**每次断电 /tmp 被清空**，2026-08-21 的原始数据
#    就是这么丢的（飞机搬回室内避雨、断电后没了）。落 $HOME 并尽快 scp 回仓库。
CSV = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser('~/a2_raw.csv')

def q():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)

class A2(Node):
    def __init__(self):
        super().__init__('a2_mag_test')
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._p, q())
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self._b, q())
        self.t0 = time.time(); self.hdg = None; self.cur = 0.0
        self.rows = []; self.settled = False; self.settle_t = None
        self.create_timer(0.1, self._tick)
        self.create_timer(5.0, self._report)
    def _p(self, m):
        if math.isfinite(m.heading): self.hdg = float(m.heading)
    def _b(self, m):
        if math.isfinite(m.current_a): self.cur = float(m.current_a)
    def _tick(self):
        if self.hdg is not None:
            self.rows.append((time.time()-self.t0, math.degrees(self.hdg), self.cur))
    def _report(self):
        if len(self.rows) < 100: return
        R = np.array(self.rows[-100:])          # 近 10s
        pp = R[:,1].max() - R[:,1].min()
        if not self.settled:
            if pp < 0.15 and R[:,2].max() < 1.0:
                self.settled = True; self.settle_t = R[-1,0]
                print(f'  ✅ [{R[-1,0]:5.0f}s] 航向已稳定（10s 峰峰 {pp:.3f}°）—— **现在可以开始推电机**', flush=True)
            else:
                print(f'  ⏳ [{R[-1,0]:5.0f}s] 等航向稳定… 峰峰 {pp:5.2f}° （需 <0.15°）  航向 {R[-1,1]:7.2f}°', flush=True)
        else:
            print(f'  [{R[-1,0]:5.0f}s] 航向 {R[-1,1]:7.2f}°  电流 {R[-1,2]:6.2f} A  (近10s 峰峰 {pp:5.2f}°)', flush=True)

def main():
    rclpy.init(); n = A2()
    print(f'A2 v2 —— {DUR:.0f}s，只读。先等航向稳定，我会提示何时开始推电机。')
    while rclpy.ok() and (time.time()-n.t0) < DUR:
        rclpy.spin_once(n, timeout_sec=0.05)
    R = np.array(n.rows)
    with open(CSV, 'w') as f:
        f.write('t,heading_deg,current_a\n')
        for r in R: f.write('%.2f,%.4f,%.4f\n' % tuple(r))
    print(f'\n原始数据已存 {CSV}（{len(R)} 行）')
    if n.settle_t is None:
        print('🔴 航向始终未稳定，本轮无效。'); return
    R = R[R[:,0] >= n.settle_t]              # 只用稳定之后的数据
    h = np.degrees(np.unwrap(np.radians(R[:,1]))); cur = R[:,2]
    base = h[cur < 1.0]
    print(f'\n===== 结果（仅用稳定后 {len(R)} 样本）=====')
    print(f'静止基线: 中位 {np.median(base):7.2f}°  峰峰 {base.max()-base.min():5.3f}°  样本 {len(base)}')
    h0 = float(np.median(base)); worst = 0.0
    print(f'{"电流档":>12} {"样本":>6} {"航向中位":>10} {"相对基线":>10} {"最大偏移":>10}')
    for lo,hi in [(1,3),(3,6),(6,10),(10,20),(20,999)]:
        sel = (cur>=lo)&(cur<hi)
        if sel.sum() < 10: continue
        hh = h[sel]; mx = float(np.max(np.abs(hh-h0))); worst = max(worst,mx)
        print(f'{lo:5.0f}-{hi:<5.0f}A {sel.sum():6d} {np.median(hh):10.2f} {np.median(hh)-h0:+10.2f} {mx:10.2f}')
    print(f'\n最大电流 {cur.max():.1f} A   最大航向偏移 {worst:.2f}°')
    on = cur >= 1.0
    if on.sum() > 30:
        k, b = np.polyfit(cur[on], h[on]-h0, 1)
        print(f'回归: 航向偏移 = {k:+.4f}°/A × 电流 {b:+.3f}°')
        for I in (20, 30, 40):
            print(f'  外推到悬停电流 {I} A ⇒ 偏移 {k*I+b:+.2f}°')
        print('⚠️ 外推假设线性且干扰源不变，仅供量级判断；拆桨电流远低于悬停，实测覆盖不到飞行工况。')
    else:
        print('🔴 电机段样本不足，无法回归。')

if __name__ == '__main__':
    main()
