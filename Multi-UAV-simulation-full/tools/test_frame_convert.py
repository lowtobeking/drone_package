#!/usr/bin/env python3
"""动捕注入的坐标变换自检（纯数学，不需要 ROS 运行时/飞控/仿真）。

    python3 tools/test_frame_convert.py

坐标系错误是位姿注入最容易翻车、且**在仿真里看起来"能飞"但方向是反的**
那类问题，所以在接任何链路之前先把变换本身钉死。
"""
import math
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from mpc_control.frame_convert import (quat_mul, swap_enu_ned_pos,
                                       swap_enu_ned_quat)

FAIL = []


def check(name, cond, detail=''):
    print(f'  {"✅" if cond else "❌"} {name}' + (f'  {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


def yaw_of(q):
    """[w,x,y,z] → 绕 z 的偏航角（弧度）。"""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def q_yaw(a):
    return np.array([math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)])


def norm_q(q):
    """把 q 与 -q 归一到同一代表元（两者是同一个旋转）。"""
    q = np.asarray(q, dtype=float)
    return q if q[0] >= 0 else -q


print('=== 位置 ENU↔NED ===')
p_enu = np.array([1.0, 2.0, 3.0])          # E=1, N=2, U=3
p_ned = swap_enu_ned_pos(p_enu)
check('ENU(1,2,3) → NED(2,1,-3)',
      np.allclose(p_ned, [2.0, 1.0, -3.0]), f'得到 {p_ned}')
check('位置变换是对合（连用两次回到原值）',
      np.allclose(swap_enu_ned_pos(p_ned), p_enu))

print('\n=== 姿态 ENU/FLU ↔ NED/FRD ===')
# ENU 里 yaw=0 表示机头朝**东**；NED 里机头朝东 = yaw +90°
q_east = swap_enu_ned_quat(np.array([1.0, 0.0, 0.0, 0.0]))
check('ENU 机头朝东(单位四元数) → NED yaw = +90°',
      abs(yaw_of(q_east) - math.pi / 2) < 1e-9,
      f'得到 {math.degrees(yaw_of(q_east)):.3f}°')

# ENU 里 yaw=+90° 表示机头朝**北**；NED 里机头朝北 = yaw 0°
q_north = swap_enu_ned_quat(q_yaw(math.pi / 2))
check('ENU 机头朝北(yaw=90°) → NED yaw = 0°',
      abs(yaw_of(q_north)) < 1e-9,
      f'得到 {math.degrees(yaw_of(q_north)):.3f}°')

# ENU yaw=180°(朝西) → NED 应为 -90°(即 270°)
q_west = swap_enu_ned_quat(q_yaw(math.pi))
check('ENU 机头朝西(yaw=180°) → NED yaw = -90°',
      abs(yaw_of(q_west) + math.pi / 2) < 1e-9,
      f'得到 {math.degrees(yaw_of(q_west)):.3f}°')

print('\n=== 对合性（随机四元数往返）===')
rng = np.random.default_rng(0)
worst = 0.0
for _ in range(2000):
    q = rng.normal(size=4)
    q = q / np.linalg.norm(q)
    back = swap_enu_ned_quat(swap_enu_ned_quat(q))
    worst = max(worst, float(np.linalg.norm(norm_q(back) - norm_q(q))))
check('2000 个随机四元数往返误差 < 1e-12', worst < 1e-12, f'最大误差 {worst:.2e}')

print('\n=== 四元数乘法本身 ===')
check('单位元', np.allclose(quat_mul(np.array([1.0, 0, 0, 0]),
                                     np.array([0.0, 1, 0, 0])),
                            [0.0, 1, 0, 0]))
# 绕 z 转 90° 两次 = 转 180°
check('绕z 90°×2 = 180°',
      np.allclose(norm_q(quat_mul(q_yaw(math.pi / 2), q_yaw(math.pi / 2))),
                  norm_q(q_yaw(math.pi))))

print('\n=== 滚转/俯仰不被混淆（非纯偏航姿态）===')
# ENU/FLU 下抬头(pitch up, 绕 FLU 的 y 轴负向)在 NED/FRD 下应为 pitch 负值
def q_pitch(a):
    return np.array([math.cos(a / 2), 0.0, math.sin(a / 2), 0.0])


def pitch_of(q):
    w, x, y, z = q
    s = 2.0 * (w * y - z * x)
    return math.asin(max(-1.0, min(1.0, s)))


# FLU 下绕 y 转 +10° = 机头**下压**；转到 FRD 后绕 y 应为 -10°
q_in = q_pitch(math.radians(10))
q_out = swap_enu_ned_quat(q_in)
check('FLU pitch +10° → FRD pitch -10°',
      abs(math.degrees(pitch_of(q_out)) + 10.0) < 1e-6,
      f'得到 {math.degrees(pitch_of(q_out)):.4f}°')

print()
if FAIL:
    print(f'❌ {len(FAIL)} 项失败: {", ".join(FAIL)}')
    sys.exit(1)
print('✅ 全部通过')
