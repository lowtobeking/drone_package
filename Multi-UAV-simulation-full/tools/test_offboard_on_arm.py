#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""半自主档（offboard_on_arm，「解锁=授权起飞」）的离线单元测试。

直接调用 mpc_node 里真实的 _arm_and_engage_offboard，桩掉 _send_vehicle_command
记录发了什么。不需要 ROS 运行时。核心要验的是**防抢杆**语义：
  · 节点绝不发 ARM；
  · 每个解锁架次只自动进一次 OFFBOARD——飞手切走就是接管，节点不许切回去；
  · 首次进入前的重试有上限（15 次），不无限期跟飞手的挡位开关对抗。
"""
import sys, types
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

C = mn.MpcControllerNode
VC = mn.VehicleCommand
# 桩环境下补上用到的两个命令号（真 px4_msgs 时已存在，不覆盖）
if not hasattr(VC, 'VEHICLE_CMD_DO_SET_MODE') or isinstance(
        getattr(VC, 'VEHICLE_CMD_DO_SET_MODE', None), type):
    VC.VEHICLE_CMD_DO_SET_MODE = 176
    VC.VEHICLE_CMD_COMPONENT_ARM_DISARM = 400


def stub(auto=False, semi=True, armed=2, nav=2, engaged_once=False, req=0, confirmed=False):
    s = types.SimpleNamespace()
    s.auto_arm_enable = auto
    s.offboard_on_arm = semi
    s._arming_state = armed
    s._nav_state = nav
    s._arm_offboard_confirmed = confirmed
    s._offboard_engaged_once = engaged_once
    s._offboard_req_count = req
    s.drone_id = 0
    s.sent = []
    s._send_vehicle_command = lambda cmd, p1=0.0, p2=0.0, p3=0.0: s.sent.append((cmd, p1, p2))
    s.get_logger = lambda: types.SimpleNamespace(info=lambda m: None, warn=lambda m: None)
    return s


fails = []
def check(name, got, want):
    ok = got == want
    print(f'  {"OK " if ok else "FAIL"} {name}: got={got} want={want}')
    if not ok: fails.append(name)


ARM = VC.VEHICLE_CMD_COMPONENT_ARM_DISARM
MODE = VC.VEHICLE_CMD_DO_SET_MODE

print('用例1  纯手动档（auto=false, semi=false）：已解锁也什么都不发')
s = stub(semi=False)
C._arm_and_engage_offboard(s)
check('不发任何指令', s.sent, [])

print('用例2  半自主：未解锁 ⇒ 不发（解锁权在飞手）')
s = stub(armed=1)
C._arm_and_engage_offboard(s)
check('不发任何指令', s.sent, [])

print('用例3  半自主：已解锁 + 未进 OFFBOARD ⇒ 只发 DO_SET_MODE(OFFBOARD)，绝不发 ARM')
s = stub()
C._arm_and_engage_offboard(s)
check('恰好一条指令', len(s.sent), 1)
check('是 DO_SET_MODE(custom=6)', s.sent[0], (MODE, 1.0, 6.0))
check('没有 ARM', any(c == ARM for c, _, _ in s.sent), False)
check('请求计数=1', s._offboard_req_count, 1)

print('用例4  防抢杆①：本架次已自动进过 OFFBOARD，飞手切走后不再切回')
s = stub(engaged_once=True, nav=2)          # 飞手已切回 POSCTL
C._arm_and_engage_offboard(s)
check('不发任何指令', s.sent, [])

print('用例5  防抢杆②：重试上限 15 次后放弃')
s = stub()
for _ in range(40):                          # 模拟 40 个重试节拍，PX4 一直没进 OFFBOARD
    C._arm_and_engage_offboard(s)
check('总共只发了 15 条', len(s.sent), 15)
check('计数停在 16（放弃已告警）', s._offboard_req_count, 16)

print('用例6  确认路径：进入 OFFBOARD 后 confirmed + engaged_once 都置位')
s = stub(nav=14)
C._arm_and_engage_offboard(s)
check('confirmed', s._arm_offboard_confirmed, True)
check('engaged_once（授权用掉）', s._offboard_engaged_once, True)
check('确认过程不发指令', s.sent, [])

print('用例7  全自主档回归：行为不变（OFFBOARD+ARM 都发）')
s = stub(auto=True, semi=False, armed=1, nav=2)
C._arm_and_engage_offboard(s)
check('发两条', len(s.sent), 2)
check('含 DO_SET_MODE', any(c == MODE for c, _, _ in s.sent), True)
check('含 ARM', any(c == ARM for c, _, _ in s.sent), True)

print('用例8  结构性回归：解锁边沿会重置一次性授权（新架次=新授权）')
src = (REPO / 'mpc_control' / 'mpc_node.py').read_text(encoding='utf-8')
i = src.find('def _on_vehicle_status')
j = src.find('\n    def ', i + 10)
body = src[i:j]
check('arm 边沿重置 engaged_once', '_offboard_engaged_once = False' in body, True)
check('arm 边沿重置请求计数', '_offboard_req_count = 0' in body, True)

print('用例9  结构性回归：两个 launch 都透传了 offboard_on_arm（防"声明了不透传"坑）')
for lf in ('launch/real_hardware_launch.py', 'launch/swarm_launch.py'):
    lt = (REPO / lf).read_text(encoding='utf-8')
    check('%s 透传' % lf, "offboard_on_arm" in lt
          and "DeclareLaunchArgument('offboard_on_arm'" in lt, True)

print('\n===== ' + ('全部通过' if not fails else f'{len(fails)} 项失败: {fails}') + ' =====')
sys.exit(1 if fails else 0)
