#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GPS 定位验收测试 —— 室外飞行的**第一道门**，不过就不要往下走。

为什么必须持续测而不是看一眼（2026-08-19 血的教训）：
    当天 A10 的 eph 在一个上午里是 8.5 → 1.5 → 2.8 → 4.0 → 6.7 m 反复横跳。
    任何一次快照都可能骗过你。**1.5m 只维持了几分钟。**
    而 OUT_solo1_* 的 safety_max_track_dist=3.0m 只在 eph ≤1.5m 时才是合理的 2σ 门槛
    —— 门槛落到 1σ 就会频繁误触发，或反过来漏掉真实飞散。

判据（全部满足才算通过）：
    eph  ≤ 1.5 m   全程（允许极少数瞬时尖峰，见 TOL）
    epv  ≤ 2.5 m   全程
    satellites_used ≥ 20
    xy_valid 持续 true（不允许反复跳变）

🔑 诊断口诀：**hdop 好而 eph 差 = 几何没问题、信号质量有问题**（遮挡/多径/增益不足）。
   hdop 只反映卫星几何分布，eph 才含信号质量。只看 hdop 或只看卫星数都会漏掉遮挡问题。
   2026-08-19 的根因正是「RTK 天线上盖了一块海绵板」——卫星数 16→26、eph 8.5→1.5。

用法（在机载电脑上）：
    source ~/Multi-UAV-simulation/tools/mpc_env.sh
    python3 tools/gps_acceptance.py            # 默认 300 秒
    python3 tools/gps_acceptance.py 120

⚠️ 走 ROS2 话题读，**不占用 14550**，可与 mav-relay / QGC 并存。
⚠️ 两台 Agent 同时在跑时 /fmu 话题会串台 ⇒ 先停掉另一台的 Agent。
⚠️ **不要用 `ros2 topic echo --once` 逐次采样**（本脚本初版的错误）：每次都要重启进程 +
   重做 ROS2 发现，约 3–5 秒一个样本，30 秒只采到 6 个、达不到判定下限。改用 rclpy 常驻订阅。
"""
import sys
import time
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import SensorGps, VehicleLocalPosition

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
TOL = 0.05                      # 允许 5% 采样超标
EPH_MAX, EPV_MAX, SATS_MIN = 1.5, 2.5, 20


def px4_qos():
    """PX4 的 DataWriter 用 TRANSIENT_LOCAL + BEST_EFFORT，订阅端必须匹配否则收不到。"""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Collector(Node):
    def __init__(self):
        super().__init__('gps_acceptance')
        self.eph, self.epv, self.sats, self.hdop = [], [], [], []
        self.xy_true = self.xy_total = 0
        self.fix = []
        self.rtcm = []
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position',
                                 self._on_gps, px4_qos())
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self._on_pos, px4_qos())
        self._last_gps = 0.0

    def _on_gps(self, m):
        now = time.time()
        if now - self._last_gps < 1.0:       # 降采样到 1 Hz，够用且省内存
            return
        self._last_gps = now
        self.eph.append(m.eph); self.epv.append(m.epv)
        self.sats.append(m.satellites_used); self.hdop.append(m.hdop)
        self.fix.append(m.fix_type)
        self.rtcm.append(getattr(m, 'rtcm_injection_rate', 0.0))

    def _on_pos(self, m):
        self.xy_total += 1
        if m.xy_valid:
            self.xy_true += 1


def report(name, vals, limit, unit='m'):
    bad = sum(1 for v in vals if not (v <= limit))
    frac = bad / len(vals)
    ok = frac <= TOL
    print(f'  {name:6s} 中位 {statistics.median(vals):7.2f}{unit}  '
          f'最大 {max(vals):7.2f}{unit}  超标 {bad}/{len(vals)} ({100*frac:4.1f}%)  '
          f'{"✅" if ok else "🔴"}')
    return ok


def main():
    rclpy.init()
    node = Collector()
    print(f'GPS 验收测试 —— 时长 {DUR:.0f}s')
    print(f'判据: eph≤{EPH_MAX}m  epv≤{EPV_MAX}m  sats≥{SATS_MIN}  xy_valid 持续 true\n')

    FIXN = {0: 'no-fix', 1: 'dead-reck', 2: '2D', 3: '3D', 4: 'DGPS',
            5: 'RTK-Float', 6: 'RTK-Fixed'}
    t0 = time.time()
    nxt = t0 + 15
    while time.time() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() >= nxt and node.eph:
            nxt += 15
            xy = 100 * node.xy_true / max(node.xy_total, 1)
            print(f'  [{time.time()-t0:5.0f}s] n={len(node.eph):3d}  '
                  f'eph={node.eph[-1]:5.2f}  epv={node.epv[-1]:5.2f}  '
                  f'sats={node.sats[-1]:3d}  fix={FIXN.get(node.fix[-1], "?")}  '
                  f'xy_valid={xy:3.0f}%', flush=True)

    n = len(node.eph)
    if n == 0:
        print('🔴 一个 GPS 样本都没收到 —— 检查 XRCE Agent 是否在跑、话题名是否正确')
        sys.exit(2)
    if node.sats and max(node.sats) == 0:
        print('🔴 全程卫星数为 0 —— 室内或天线无信号，验收无法进行。')
        print('   （室内 fix_type=0 / hdop=99.99 是正常现象，不是故障）')
        sys.exit(2)
    if n < 10:
        print(f'🔴 只采到 {n} 个样本，无法判定 —— 延长时长或检查话题频率')
        sys.exit(2)

    print('\n===== 结果 =====')
    ok_eph = report('eph', node.eph, EPH_MAX)
    ok_epv = report('epv', node.epv, EPV_MAX)
    sats_ok = sum(1 for s in node.sats if s >= SATS_MIN) / n >= 1 - TOL
    print(f'  sats   中位 {statistics.median(node.sats):7.0f}    '
          f'最小 {min(node.sats):7.0f}     {"✅" if sats_ok else "🔴"}')
    print(f'  hdop   中位 {statistics.median(node.hdop):7.2f}    最大 {max(node.hdop):7.2f}')
    fixes = {FIXN.get(f, str(f)): node.fix.count(f) for f in set(node.fix)}
    print(f'  fix_type 分布: {fixes}')
    if max(node.rtcm) > 0:
        print(f'  RTCM 注入率 中位 {statistics.median(node.rtcm):.2f} Hz')
    else:
        print('  RTCM 注入率 0 —— 未接差分（单点定位）')
    xy_frac = node.xy_true / max(node.xy_total, 1)
    xy_ok = xy_frac >= 1 - TOL
    print(f'  xy_valid 为 true 的比例 {100*xy_frac:5.1f}%  {"✅" if xy_ok else "🔴"}')

    if statistics.median(node.hdop) < 1.5 and statistics.median(node.eph) > 3.0:
        print('\n⚠️  hdop 好而 eph 差 ⇒ 卫星几何没问题，**信号质量有问题**')
        print('    查：天线是否被遮挡/压住、是否朝天水平、下方有无金属地板、')
        print('        接头是否拧紧、是否远离电调与动力线')

    print()
    if ok_eph and ok_epv and sats_ok and xy_ok:
        print('✅ 验收通过 —— 可以进入下一项')
        code = 0
    else:
        print('🔴 验收未通过 —— 不要往下走。')
        print('   OUT_* 场景的 safety_max_track_dist=3.0m 只在 eph≤1.5m 时才是合理的 2σ 门槛')
        code = 1
    node.destroy_node(); rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
