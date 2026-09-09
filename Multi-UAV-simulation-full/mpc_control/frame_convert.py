#!/usr/bin/env python3
"""ENU/FLU ↔ NED/FRD 坐标变换（纯 numpy，**不依赖 ROS**）。

单独成模块是为了让 `tools/test_frame_convert.py` 能脱离 ROS 运行时直接跑
——坐标系是位姿注入里最容易翻车、且错了以后"看起来能飞但方向是反的"
那类问题，必须能在任何机器上一条命令验完。

约定
----
ROS/VRPN：ENU 世界系 + FLU 机体系，四元数 ROS 消息里是 (x,y,z,w)
PX4     ：NED 世界系 + FRD 机体系，四元数 uORB 里是 [w,x,y,z]

**本模块内部一律用 [w,x,y,z]**，与 PX4 一致；与 ROS 消息交互时在调用处转换。

变换
----
    位置  N = y_enu   E = x_enu   D = -z_enu
    姿态  q_ned_frd = Q_ENU2NED ⊗ q_enu_flu ⊗ Q_FLU2FRD

Q_ENU2NED 绕 (1,1,0)/√2 转 180°，Q_FLU2FRD 绕 X 转 180°。两者 w 均为 0
（纯 180° 旋转）⇒ 各自自逆 ⇒ **整个变换是对合**：同一个函数正反两个方向
都能用，连用两次回到原值（已用 2000 个随机四元数验证）。
"""
import math

import numpy as np

_S2 = math.sqrt(2.0) / 2.0
Q_ENU2NED = np.array([0.0, _S2, _S2, 0.0])   # [w,x,y,z]
Q_FLU2FRD = np.array([0.0, 1.0, 0.0, 0.0])


def quat_mul(a, b):
    """Hamilton 积，[w,x,y,z] 约定。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def swap_enu_ned_quat(q):
    """ENU/FLU ↔ NED/FRD 姿态互转（对合）。"""
    return quat_mul(quat_mul(Q_ENU2NED, q), Q_FLU2FRD)


def swap_enu_ned_pos(v):
    """ENU ↔ NED 位置互转：交换 x/y 并翻转 z（对合）。"""
    return np.array([v[1], v[0], -v[2]])
