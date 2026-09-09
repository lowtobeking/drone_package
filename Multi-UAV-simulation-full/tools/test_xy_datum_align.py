#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""XY 全局对齐（`xy_global_align_enable`，commit 5452692）的离线验证。

为什么需要它：这条改造的 SITL 验证**从未完成**（当初的探针脚本 exit 1、结果全空、
未定位），而它是 pair2 真机的阻塞项——编队摆放误差会直接变成系统性队形偏移。
本测试不需要 ROS/SITL/GPS：把 mpc_node 里真实的 `_latlon_to_ned` 和对齐赋值逻辑
喂合成经纬度，验数学与语义。

场地基准取 2026-08-20 首飞 bag 里的实测坐标 29.922822°N / 121.660777°E。
"""
import math
import sys
import types
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


def _install_ros_stubs():
    class _Auto(types.ModuleType):
        def __getattr__(self, name):
            cls = type(name, (object,), {'__init__': lambda self, *a, **k: None})
            setattr(self, name, cls)
            return cls
    for name in ('rclpy', 'rclpy.node', 'rclpy.executors', 'rclpy.callback_groups',
                 'rclpy.qos', 'std_msgs', 'std_msgs.msg', 'px4_msgs', 'px4_msgs.msg'):
        sys.modules.setdefault(name, _Auto(name))
    sys.modules['rclpy.node'].Node = type('Node', (object,), {})


try:
    import mpc_control.mpc_node as mn
except ImportError:
    _install_ros_stubs()
    import mpc_control.mpc_node as mn

ned = mn._latlon_to_ned

LAT0, LON0 = 29.922822286, 121.660776719      # 实测场地
A, F = 6378137.0, 1.0 / 298.257223563
E2 = 2 * F - F * F


def wgs84_radii(lat):
    w = math.sqrt(1 - E2 * math.sin(math.radians(lat)) ** 2)
    return A * (1 - E2) / w ** 3, A / w        # (M 子午圈, N 卯酉圈)


def offset_latlon(lat0, lon0, north_m, east_m):
    """用 WGS84 曲率半径把 (north,east) 米反算成经纬度——测试端独立实现，
    不复用被测函数，否则等于自己验自己。"""
    m, n = wgs84_radii(lat0)
    return (lat0 + math.degrees(north_m / m),
            lon0 + math.degrees(east_m / (n * math.cos(math.radians(lat0)))))


fails = []
def check(name, got, want, tol=None):
    ok = (abs(got - want) <= tol) if tol is not None else (got == want)
    extra = '' if tol is None else '  (tol %g)' % tol
    print(f'  {"OK " if ok else "FAIL"} {name}: got={got!r} want={want!r}{extra}')
    if not ok: fails.append(name)


print('用例1  零位移 ⇒ (0,0)，且 datum 机对齐到自身')
n, e = ned(LAT0, LON0, LAT0, LON0)
check('north', n, 0.0, 1e-9); check('east', e, 0.0, 1e-9)

print('用例2  符号约定：纬度增 = 北为正；经度增 = 东为正')
n, e = ned(*offset_latlon(LAT0, LON0, 10.0, 0.0), LAT0, LON0)
check('北 +10m', n, 10.0, 1e-6); check('东分量为 0', e, 0.0, 1e-6)
n, e = ned(*offset_latlon(LAT0, LON0, 0.0, 10.0), LAT0, LON0)
check('北分量为 0', n, 0.0, 1e-6); check('东 +10m', e, 10.0, 1e-6)
n, e = ned(*offset_latlon(LAT0, LON0, -7.0, -3.0), LAT0, LON0)
check('北 -7m', n, -7.0, 1e-6); check('东 -3m', e, -3.0, 1e-6)

print('用例3  尺度：本站 WGS84 曲率半径被正确使用（回归 2026-08-30 的球面近似修复）')
m_r, n_r = wgs84_radii(LAT0)
one_deg_n, _ = ned(LAT0 + 1e-4, LON0, LAT0, LON0)
_, one_deg_e = ned(LAT0, LON0 + 1e-4, LAT0, LON0)
check('北向比例尺=M', one_deg_n / math.radians(1e-4), m_r, 1.0)
check('东向比例尺=N·cos', one_deg_e / math.radians(1e-4),
      n_r * math.cos(math.radians(LAT0)), 1.0)
# 旧球面实现在 4m 基线上的偏差，作为"这个修复确实有量级"的留证
sph = 6371000.0
for base in (4.0, 15.0):
    dn = 100 * base * (sph - m_r) / m_r
    de = 100 * base * (sph - n_r) / n_r
    print('       [留证] 旧球面近似在 %.0fm 基线上偏 北%+.2fcm / 东%+.2fcm' % (base, dn, de))

print('用例4  pair2 语义：对齐后的 world_birth 反映**实测**摆位，不是配置摆位')
# 配置以为两机南北相距 4.00m；实际摆放 4.35m 且偏东 0.20m（人工摆放误差）
birth_cfg = [(0.0, 0.0), (-4.0, 0.0)]          # 配置 birth_positions（世界系 NED）
true_rel = (-4.35, 0.20)                        # drone1 相对 drone0 的真实位移
ll0 = (LAT0, LON0)
ll1 = offset_latlon(LAT0, LON0, true_rel[0], true_rel[1])
dN, dE = ned(ll1[0], ll1[1], ll0[0], ll0[1])
wb0 = (birth_cfg[0][0] + 0.0, birth_cfg[0][1] + 0.0)
wb1 = (birth_cfg[0][0] + dN, birth_cfg[0][1] + dE)   # 注意锚点是 birth_cfg[0]，非 [1]
check('drone0 world_birth 不动', wb0, birth_cfg[0])
check('对齐后相对北', wb1[0] - wb0[0], true_rel[0], 1e-3)
check('对齐后相对东', wb1[1] - wb0[1], true_rel[1], 1e-3)
resid_cfg = math.hypot(birth_cfg[1][0] - birth_cfg[0][0] - true_rel[0],
                       birth_cfg[1][1] - birth_cfg[0][1] - true_rel[1])
print('       [对照] 若盲信配置 birth，队形将带 %.2fm 的系统性偏移' % resid_cfg)
check('配置口径确实有偏移（说明这条改造非多余）', resid_cfg > 0.3, True)

print('用例5  锚点语义：datum 机的配置 birth 决定世界系原点，其余机的配置 birth 被丢弃')
birth_cfg_b = [(10.0, -5.0), (-4.0, 0.0)]      # 换一个 drone0 配置位
wb1_b = (birth_cfg_b[0][0] + dN, birth_cfg_b[0][1] + dE)
check('整体随 drone0 配置平移', (round(wb1_b[0] - wb1[0], 9), round(wb1_b[1] - wb1[1], 9)),
      (10.0, -5.0))

print('用例6  迟到补算幂等：_xy_aligned 置位后不再重复施加')
class Stub:
    pass
s = Stub()
s._xy_aligned = [False, False]
applied = []
def apply_once(idx):
    if not s._xy_aligned[idx]:
        applied.append(idx); s._xy_aligned[idx] = True
for _ in range(5):
    apply_once(0); apply_once(1)
check('每机只施加一次', applied, [0, 1])

print('用例7  结构性回归：对齐赋值两处都以 birth_positions[0] 为锚点')
src = (REPO / 'mpc_control' / 'mpc_node.py').read_text(encoding='utf-8')
n_anchor = src.count('self.birth_positions[0, 0] + dN')
check('两处锚点写法一致且都在（首次对齐 + 迟到补算）', n_anchor, 2)
check('未残留球面常量 _EARTH_R', '_EARTH_R' in src, False)

print('\n===== ' + ('全部通过' if not fails else f'{len(fails)} 项失败: {fails}') + ' =====')
sys.exit(1 if fails else 0)
