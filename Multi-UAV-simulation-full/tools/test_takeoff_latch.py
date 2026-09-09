#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离地闩锁状态机的离线单元测试：直接调用 mpc_node 里真实的 _update_airborne，
用桩对象喂 z 序列。不需要 ROS 运行时、不需要 SITL —— 验的是代码本身。

2026-08-30 起也能在 Windows 开发机上跑：若 rclpy/px4_msgs 不可用（本仓库的工作流是
Windows 编辑 → Ubuntu 构建），自动注入极简桩模块再导入。Ubuntu 上有真模块时走真导入，
行为不变。桩只为让 `import mpc_control.mpc_node` 通过；被测函数本身是纯状态机。
"""
import os, sys, types
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


def _install_ros_stubs():
    """给 rclpy / std_msgs / px4_msgs 装上按需生成属性的桩模块。"""
    class _Auto(types.ModuleType):
        def __getattr__(self, name):                      # 任何名字都给一个可调用的占位类
            cls = type(name, (object,), {'__init__': lambda self, *a, **k: None})
            setattr(self, name, cls)
            return cls

    for name in ('rclpy', 'rclpy.node', 'rclpy.executors', 'rclpy.callback_groups',
                 'rclpy.qos', 'std_msgs', 'std_msgs.msg', 'px4_msgs', 'px4_msgs.msg'):
        sys.modules.setdefault(name, _Auto(name))
    # `class MpcControllerNode(Node)` 需要 Node 是个真能继承的类
    sys.modules['rclpy.node'].Node = type('Node', (object,), {})


try:
    from mpc_control.mpc_node import MpcControllerNode as C
except ImportError:
    _install_ros_stubs()
    from mpc_control.mpc_node import MpcControllerNode as C


def stub(rise=0.5, airborne=False, ground_z=None, enable=True, arm_time=0.0, now=0.0):
    s = types.SimpleNamespace()
    s._airborne = airborne; s._ground_z = ground_z
    s._takeoff_rise = rise; s.drone_id = 0
    s._takeoff_lock_enable = enable
    s._arm_time = arm_time
    s._takeoff_lock_max_s = 20.0
    s.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=int(now * 1e9)))
    s.get_logger = lambda: types.SimpleNamespace(info=lambda m: None, warn=lambda m: None)
    return s


fails = []
def check(name, got, want):
    ok = got == want
    print(f'  {"OK " if ok else "FAIL"} {name}: got={got} want={want}')
    if not ok: fails.append(name)


print('用例1  地面 z=0 起飞：升 0.4m 不解锁、升 0.5m 解锁')
s = stub()
for z in (0.0, -0.2, -0.4):
    C._update_airborne(s, z)
check('升0.4m 仍锁定', s._airborne, False)
C._update_airborne(s, -0.5)
check('升0.5m 解锁', s._airborne, True)

print('用例2  基准漂移免疫：地面 z=+0.85（首飞气压计漂移那种），阈值仍是"相对上升 0.5m"')
s = stub()
C._update_airborne(s, 0.85)          # 解锁瞬间的地面基准
C._update_airborne(s, 0.45)          # 升了 0.40m
check('升0.4m 仍锁定', s._airborne, False)
C._update_airborne(s, 0.35)          # 升了 0.50m
check('升0.5m 解锁', s._airborne, True)

print('用例3  闩锁不回退：解锁后即便掉回地面高度也不再锁')
s = stub()
C._update_airborne(s, 0.0); C._update_airborne(s, -1.0)
check('已解锁', s._airborne, True)
C._update_airborne(s, 0.0)
check('掉回地面仍解锁', s._airborne, True)

print('用例4  fail-open：没见过解锁边沿(_airborne 初值 True) ⇒ 永不锁 XY')
s = stub(airborne=True)
C._update_airborne(s, 0.0); C._update_airborne(s, 0.0)
check('保持解锁', s._airborne, True)
check('不误设地面基准', s._ground_z, None)

print('用例5  地面基准取第一拍有效 z，不是硬编码 0')
s = stub()
C._update_airborne(s, 3.7)
check('基准=3.7', s._ground_z, 3.7)

print('用例6  2026-08-21 实测 bug 的症状复现：基准若在空中才捕获，闩锁永不解除')
# 飞手手动把飞机带到接近目标高度(-2.9m)才切 OFFBOARD；旧实现在控制回路里首次捕获
# ⇒ 假"地面"=-2.9，而此后飞机只在目标高度附近浮动、再也升不了 takeoff_rise。
s = stub()
C._update_airborne(s, -2.9)                  # 空中捕到的假"地面"
check('假基准=-2.9', s._ground_z, -2.9)
for z in (-2.95, -3.05, -3.0, -2.88, -3.02):
    C._update_airborne(s, z)
check('闩锁卡死（这正是必须由位置回调捕获的理由）', s._airborne, False)

print('用例7  看门狗：锁超过 _takeoff_lock_max_s 强制 fail-open')
s = stub(arm_time=0.0, now=19.9)
C._update_airborne(s, -2.0)                  # 同上，卡死状态
C._check_takeoff_lock_watchdog(s)
check('19.9s 未到时限，仍锁定', s._airborne, False)
s.get_clock = lambda: types.SimpleNamespace(
    now=lambda: types.SimpleNamespace(nanoseconds=int(20.1e9)))
C._check_takeoff_lock_watchdog(s)
check('20.1s 超时，强制解锁', s._airborne, True)

print('用例8  看门狗不误伤：功能关闭 / 未解锁 / 已离地 三种情况都不动状态')
s = stub(enable=False, now=999.0)
C._check_takeoff_lock_watchdog(s); check('功能关闭时不改状态', s._airborne, False)
s = stub(arm_time=None, now=999.0)
C._check_takeoff_lock_watchdog(s); check('未见解锁边沿时不改状态', s._airborne, False)
s = stub(airborne=True, now=999.0)
C._check_takeoff_lock_watchdog(s); check('已离地时保持', s._airborne, True)

print('用例9  结构性回归：_update_airborne 必须由位置回调推进，不能只在控制回路里')
src = (REPO / 'mpc_control' / 'mpc_node.py').read_text(encoding='utf-8')
i = src.find('def _make_pos_callback')
j = src.find('\n    def ', i + 10)            # 该方法体的结束
body = src[i:j]
check('_make_pos_callback 内调用了 _update_airborne', '_update_airborne' in body, True)
check('自机门控存在（不能给邻居推进闩锁）', 'drone_idx == self.drone_id' in body, True)
others = src[:i] + src[j:]
check('别处不再调用 _update_airborne', '_update_airborne(' in others.replace('def _update_airborne(', ''), False)

print('\n===== ' + ('全部通过' if not fails else f'{len(fails)} 项失败: {fails}') + ' =====')
sys.exit(1 if fails else 0)
