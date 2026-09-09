#!/usr/bin/env python3
"""
diag_monitor.py — 实时编队健康诊断监控

订阅所有无人机的位置、状态和 MPC 健康话题，每秒刷新显示：
  - 每机：高度误差、ARM/OFFBOARD 状态、速度
  - 编队：机间距（最小值高亮）、队形偏差
  - MPC：求解时间、降级次数、是否当前在 hover 降级
  - 安全违规计数（间距 < 阈值的事件数）

用法:
  ros2 run mpc_control diag_monitor --ros-args -p formation:=solo1
  ros2 run mpc_control diag_monitor --ros-args -p formation:=pair2
  ros2 run mpc_control diag_monitor --ros-args -p formation:=trio3
  ros2 run mpc_control diag_monitor --ros-args -p formation:=cross5
  ros2 run mpc_control diag_monitor --ros-args -p formation:=grid9

  # 也可直接运行（不在 ROS2 包里）：
  python3 diag_monitor.py --formation pair2
"""

import argparse
import itertools
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

# ── 队形配置（与 swarm_launch.py 保持一致）──────────────────────────────────
#
# ⚠️ 曾有 'pairs' 字段手工列出"要监控的机对"，已于 2026-07-10 删除。
#    min_spacing / safety_violations 现一律对 **C(num,2) 全对** 枚举（_all_pairs）。
#    旧写法的三个实证问题：
#      · cross5 只列了中心-臂 4 对，臂↔臂对角(3√2=4.24m)从不参与判定
#      · star5 只列五边形环 5 对；grid9 只列 12 条晶格边（对角 4.24m 漏掉）
#      · star5 的槽位表还曾未随 66027a7 重分配同步 → 把正确落位算成 ~5.71m 假误差
#        （即 S9 假 FAIL 的根因）
#    碰撞代价只对邻居生效 ⇒ 非邻居对本就不在 OCP 视野内，监控端更不能再漏。
def _all_pairs(n):
    return list(itertools.combinations(range(n), 2))


def _resolve_target_alt(formation, scenario=None, override=None):
    """决定 zerr 用哪个参考高度。优先级：显式 override > scenarios.yaml > 本文件兜底。

    🔴 存在的理由：FORMATION_CFG 里的 target_alt 是 scenarios.yaml 的**重复真值源**，
       2026-08-20 复盘真机首飞时发现它漂了：`OUT_solo1_hover` 的 limits.target_alt
       是 −4.0（后改 −3.0），而 FORMATION_CFG['solo1'] 写着 −5.0 ⇒ 所有 OUT_* 架次的
       `d0_zerr` 列整体偏 1.000 m。这种偏差**看不出来**（列里全是"像模像样"的数），
       只有拿 bag 的 z 逐行对才暴露 —— 所以宁可多这几行也要走单一真值源。
    """
    if override is not None:
        return float(override), f'--target-alt {override}'
    if scenario:
        try:
            import yaml
            p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'config', 'scenarios.yaml')
            with open(p, encoding='utf-8') as f:
                y = yaml.safe_load(f)
            lim = (y.get('scenarios') or {}).get(scenario, {}).get('limits') or {}
            if 'target_alt' in lim:
                return float(lim['target_alt']), f'scenarios.yaml::{scenario}'
            raise KeyError(f'scenarios.{scenario}.limits.target_alt 不存在')
        except Exception as e:
            # 不静默回落：读不到就说清楚，否则又变成一个查不出来的偏差
            print(f'[diag_monitor] ⚠️ 无法从 scenarios.yaml 读取 "{scenario}" 的 '
                  f'target_alt（{e}），回落到 FORMATION_CFG["{formation}"]', file=sys.stderr)
    return float(FORMATION_CFG[formation]['target_alt']), f'FORMATION_CFG[{formation}] 兜底'


# 4-连接晶格（间距 3 m，居中于原点），逐字对照 config/scenarios.yaml 的 formations.grid16/25。
# grid16 = 4×4 行序；grid25 = 5×5 同心环序（前 9 机 == grid9，故 grid9 ⊂ grid25）。
_GRID16 = np.array([
    [-4.5, -4.5, 0.0], [-4.5, -1.5, 0.0], [-4.5,  1.5, 0.0], [-4.5,  4.5, 0.0],
    [-1.5, -4.5, 0.0], [-1.5, -1.5, 0.0], [-1.5,  1.5, 0.0], [-1.5,  4.5, 0.0],
    [ 1.5, -4.5, 0.0], [ 1.5, -1.5, 0.0], [ 1.5,  1.5, 0.0], [ 1.5,  4.5, 0.0],
    [ 4.5, -4.5, 0.0], [ 4.5, -1.5, 0.0], [ 4.5,  1.5, 0.0], [ 4.5,  4.5, 0.0],
])
_GRID25 = np.array([
    [ 0.0,  0.0, 0.0],
    [ 0.0,  3.0, 0.0], [ 0.0, -3.0, 0.0], [ 3.0,  0.0, 0.0], [-3.0,  0.0, 0.0],
    [ 3.0,  3.0, 0.0], [-3.0,  3.0, 0.0], [ 3.0, -3.0, 0.0], [-3.0, -3.0, 0.0],
    [ 0.0,  6.0, 0.0], [ 0.0, -6.0, 0.0], [ 6.0,  0.0, 0.0], [-6.0,  0.0, 0.0],
    [ 3.0,  6.0, 0.0], [ 6.0,  3.0, 0.0], [ 6.0,  6.0, 0.0], [-6.0,  3.0, 0.0],
    [-6.0,  6.0, 0.0], [-3.0,  6.0, 0.0],
    [ 3.0, -6.0, 0.0], [ 6.0, -6.0, 0.0], [ 6.0, -3.0, 0.0], [-6.0, -6.0, 0.0],
    [-6.0, -3.0, 0.0], [-3.0, -6.0, 0.0],
])


# ⚠️ 本表是 config/scenarios.yaml 里 formations 的**重复真值源**，已经漂移过一次：
# 2026-07-22 新增的室内队形 pair2_in 没同步进来，2026-07-23 跑动捕遮挡对照实验时
# argparse 直接以 "invalid choice" 退出、而外层脚本只看到"录制 0 行"，绕了一圈才定位。
# **新增/修改 formations 时务必同步这里**；长期应改成直接从 scenarios.yaml 读。
FORMATION_CFG = {
    'solo1': {
        'num': 1,
        'offsets_ned': np.array([[0.0, 0.0, 0.0]]),
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'pair2': {
        'num': 2,
        'offsets_ned': np.array([[0.0, 0.0, 0.0], [-3.0, 0.0, 0.0]]),
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    # 室内 5×5/7×8.5 飞场专用：关于原点对称（原 pair2 的 offsets 偏心，
    # 起飞点设在场地中心时 drone1 会出界）。间距 2.8m，target_alt 对齐 IN_* 场景。
    'pair2_in': {
        'num': 2,
        'offsets_ned': np.array([[1.4, 0.0, 0.0], [-1.4, 0.0, 0.0]]),
        'target_alt': -1.2,
        'd_safe': 1.5,
    },
    'trio3': {
        'num': 3,
        'offsets_ned': np.array([
            [ 3.0,    0.0,   0.0],
            [-1.5,    2.598, 0.0],
            [-1.5,   -2.598, 0.0],
        ]),
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'cross5': {
        'num': 5,
        'offsets_ned': np.array([
            [ 0.0,  0.0, 0.0],
            [ 0.0,  3.0, 0.0],
            [ 0.0, -3.0, 0.0],
            [ 3.0,  0.0, 0.0],
            [-3.0,  0.0, 0.0],
        ]),
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'star5': {
        'num': 5,
        # 与 scenarios.yaml 的槽位最近邻重分配（66027a7）对齐：d0→SW d1→NE
        # d2→NW d3→N d4→SE。旧表是重分配前的槽序，会把正确落位的机算出
        # ~5.71m 的假队形误差（= 五边形隔位顶点弦长 2·3·sin72°）。
        'offsets_ned': np.array([
            [-2.427, -1.763,  0.0],
            [ 0.927,  2.853,  0.0],
            [ 0.927, -2.853,  0.0],
            [ 3.0,    0.0,    0.0],
            [-2.427,  1.763,  0.0],
        ]),
        # 五边形环序 d3-d1-d4-d0-d2-d3（与 yaml neighbours 同源）
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'grid9': {
        'num': 9,
        'offsets_ned': np.array([
            [ 0.0,  0.0, 0.0],
            [ 0.0,  3.0, 0.0],
            [ 0.0, -3.0, 0.0],
            [ 3.0,  0.0, 0.0],
            [-3.0,  0.0, 0.0],
            [ 3.0,  3.0, 0.0],
            [-3.0,  3.0, 0.0],
            [ 3.0, -3.0, 0.0],
            [-3.0, -3.0, 0.0],
        ]),
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    # ── 规模上限研究（优化③）：4-连接晶格，出生即队形 ⇒ offsets == birth ──────
    'grid16': {
        'num': 16,
        'offsets_ned': _GRID16,
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'grid25': {
        'num': 25,
        'offsets_ned': _GRID25,
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
}

# ── 各机出生点（世界系 NED），与 swarm_launch.py 的 BIRTH_* / mpc_node 的 world_birth 一致 ──
# diag 订阅的 vehicle_local_position 是各机相对【自身出生点】的本地系；要在统一世界系里
# 比较机间距/队形偏差，必须加回出生点：world = local + birth。否则非原点出生的机（如 pair2
# 的 drone1 出生在 (-3,0)）本地读数≈0 会被误判成与中心机重叠 → 假 CRIT、队形误差虚高。
# ★ star5 出生在十字布局(BIRTH_5)、目标才是五边形(offsets)，故 birth ≠ offsets，不能用 offsets 顶替。
_BIRTH5 = np.array([
    [ 0.0,  0.0, 0.0],   # 0 中心
    [ 0.0,  3.0, 0.0],   # 1 东
    [ 0.0, -3.0, 0.0],   # 2 西
    [ 3.0,  0.0, 0.0],   # 3 北
    [-3.0,  0.0, 0.0],   # 4 南
])
# ⚠️ 与 FORMATION_CFG 一样是 scenarios.yaml 的重复真值源，**加队形要两处都改**
# （2026-07-23 只补了 FORMATION_CFG，这里漏了，直接 KeyError 崩在启动）。
BIRTH_NED = {
    'solo1':  np.array([[0.0, 0.0, 0.0]]),
    'pair2':  np.array([[0.0, 0.0, 0.0], [-3.0, 0.0, 0.0]]),
    'pair2_in': np.array([[1.4, 0.0, 0.0], [-1.4, 0.0, 0.0]]),  # 室内对称版
    'trio3':  np.array([[3.0, 0.0, 0.0], [-1.5, 2.598, 0.0], [-1.5, -2.598, 0.0]]),
    'cross5': _BIRTH5,
    'star5':  _BIRTH5,   # ★ 出生=十字，目标=五边形；birth ≠ offsets
    'grid9':  np.concatenate([_BIRTH5, np.array([
        [ 3.0,  3.0, 0.0],   # 5 东北
        [-3.0,  3.0, 0.0],   # 6 东南
        [ 3.0, -3.0, 0.0],   # 7 西北
        [-3.0, -3.0, 0.0],   # 8 西南
    ])]),
    'grid16': _GRID16,   # 出生即队形（birth == offsets）
    'grid25': _GRID25,   # 出生即队形（birth == offsets）
}

# 安全间距告警阈值（比 d_safe 多 0.3m 余量，给提前预警）
SPACING_WARN  = 1.8   # 黄色警告
SPACING_CRIT  = 1.5   # 红色危险（等于 d_safe）


def qos_sub():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def topic_for(drone_id, suffix):
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'


def mpc_topic_for(drone_id, suffix):
    if drone_id == 0:
        return f'/mpc/{suffix}'
    return f'/px4_{drone_id}/mpc/{suffix}'


class DroneData:
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.received = False
        self.last_stamp = 0.0
        self.arm_state  = 0   # 1=disarmed, 2=armed
        self.nav_state  = 0   # 14=offboard
        self.status_recv = False
        # MPC health
        self.mpc_status    = -1
        self.solve_ms      = 0.0
        self.fallback_count = 0
        self.hover_active  = False
        self.pos_err       = 0.0
        self.health_recv   = False
        # §5.G③ 取证（老节点不发这两项，保持 0）
        self.plan_ratio_max = 0.0
        self.clip_count     = 0
        # §6.5 DOB baseline 的 d̂（老节点不发 index 9-11，保持 0）
        self.dhat = np.zeros(3)
        # 起飞期横向锁定的闩锁（health index 12）：0=锁定中/未离地，1=已离地。
        # 老节点不发这一列 ⇒ 保持 None，CSV 写 nan 而不是骗人的 0。
        self.airborne = None


class DiagMonitor(Node):
    def __init__(self, formation: str, log_path: str = None,
                 shared_origin: bool = False, scenario: str = None,
                 target_alt: float = None):
        super().__init__('diag_monitor')
        # 动捕/外部定位=全场共享原点，local 已是世界坐标（详见 _make_pos_cb）
        self.shared_origin = shared_origin

        if formation not in FORMATION_CFG:
            self.get_logger().error(
                f'未知队形 "{formation}"，可选: {list(FORMATION_CFG.keys())}')
            raise ValueError(formation)

        self.cfg = FORMATION_CFG[formation]
        self.formation = formation
        # 🔴 target_alt 是**第二个真值源**（见文件头注释），2026-08-20 复盘真机首飞时
        #    发现它又漂了一次：`OUT_solo1_hover` 的 limits.target_alt 是 −4.0，而本表里
        #    solo1 写着 −5.0 ⇒ 所有 OUT_* 架次的 `d0_zerr` 列**整体偏 1.000 m**
        #    （逐行核对 z + 5.0 完全吻合）。列本身"看着很正常"，所以能一直不被发现。
        # ✅ 现在优先用 scenarios.yaml 里该场景的 limits.target_alt；本表只作兜底。
        self.target_alt, self._alt_src = _resolve_target_alt(
            formation, scenario, target_alt)
        self.num = self.cfg['num']
        self.birth = BIRTH_NED[formation]   # (num,3) 世界系出生点；world = local + birth
        self.drones = [DroneData() for _ in range(self.num)]
        # min_spacing / safety_violations 一律走全对枚举（见文件头 FORMATION_CFG 注释）。
        # solo1 → 空列表 → 显示 N/A。grid25 = 300 对，逐帧枚举开销可忽略。
        self.all_pairs = _all_pairs(self.num)

        # 统计
        self._safety_violations = 0   # 间距 < SPACING_CRIT 的事件次数
        self._start_time = time.time()
        self._leader_pos = np.zeros(3)
        self._leader_vel = np.zeros(3)
        self._leader_recv = False

        q = qos_sub()
        for i in range(self.num):
            # local_position 话题名随固件版本化：PX4 main → `_v1`；v1.16/v1.14 → 无后缀版。
            # 下面两个后缀都订阅：哪个存在哪个生效，跨版本通用；v1.16 命中无后缀版。
            for _lp_t in ('out/vehicle_local_position_v1', 'out/vehicle_local_position'):
                self.create_subscription(
                    VehicleLocalPosition,
                    topic_for(i, _lp_t),
                    self._make_pos_cb(i), q,
                )
            # 🔴 PX4 v1.16 起启用**消息版本化**：话题改名为 `vehicle_status_v1`
            #    （消息类型仍是 px4_msgs/msg/VehicleStatus，只有话题名带后缀）。
            #    v1.14 上不存在 _v1、v1.16 上不存在无后缀版 —— 而**订阅错了不会报错，
            #    只是永远收不到**，`arm`/`nav` 两列就一直是初始值 0。
            #    2026-08-20 现场踩到：真机首飞的 CSV 里 nav/arm 全 0，而那正是复盘时
            #    用来切出飞行段的关键列 ⇒ 整份记录的状态信息作废。
            #    mpc_node 早就用 px4_version 参数处理了，本文件一直没跟上。
            # ✅ 两个名字都订阅：哪个存在哪个生效，跨版本通用、无需加参数。
            for _st_topic in ('out/vehicle_status_v4', 'out/vehicle_status_v1', 'out/vehicle_status'):
                self.create_subscription(
                    VehicleStatus,
                    topic_for(i, _st_topic),
                    self._make_status_cb(i), q,
                )
            self.create_subscription(
                Float32MultiArray,
                mpc_topic_for(i, 'health'),
                self._make_health_cb(i), 10,
            )

        self.create_subscription(
            Float64MultiArray, '/leader/state', self._on_leader, 10)

        self.create_timer(1.0, self._print_status)
        self.get_logger().info(
            f'diag_monitor 已启动，监控队形={formation}，{self.num} 架无人机')

        # ── 可选：结构化飞行记录 (CSV)，每秒一行，供 analyze_flight.py 复盘 ──
        self._csv = None
        if log_path:
            import os
            d = os.path.dirname(os.path.abspath(log_path))
            if d:
                os.makedirs(d, exist_ok=True)
            self._csv = open(log_path, 'w', buffering=1)
            hdr = ['t']
            for i in range(self.num):
                # d{i}_x / d{i}_y 为世界系 NED 位置(北/东)，用于俯视轨迹图；
                # 追加在每机分组最前，纯增列、向后兼容 analyze_flight.py。
                hdr += [f'd{i}_x', f'd{i}_y', f'd{i}_z', f'd{i}_zerr',
                        f'd{i}_velxy', f'd{i}_arm',
                        f'd{i}_nav', f'd{i}_mpc', f'd{i}_solve_ms',
                        f'd{i}_fallback', f'd{i}_hover', f'd{i}_poserr',
                        f'd{i}_planratio', f'd{i}_clipn',
                        f'd{i}_dhatx', f'd{i}_dhaty', f'd{i}_dhatz',
                        f'd{i}_airborne']
            # ⚠️ 列名 `max_solve_ms` 容易误读：它是**当前这一秒跨机取最大**，
            #    不是"全程最大"。solo1 下它与 d0_solve_ms 逐行完全相同；
            #    2026-08-20 复盘首飞时就因为末行读到 4.202 而全程实际最大是 10.611
            #    白查了一轮。想要全程最大请自己对整列取 max()——
            #    analyze_flight.py / tools/verdict.py 本来就是这么做的，语义没问题。
            #    刻意不改名：改了会同时弄坏那两个脚本和所有已归档 CSV，收益只是名字好听。
            hdr += ['min_spacing', 'formation_max_err', 'safety_violations',
                    'total_fallbacks', 'max_solve_ms',
                    'leader_x', 'leader_y', 'leader_vx', 'leader_vy']
            self._csv.write(','.join(hdr) + '\n')
            self.get_logger().info(f'flight log → {log_path}')

    def _make_pos_cb(self, idx):
        def cb(msg):
            d = self.drones[idx]
            # local→world：加回出生点，与 mpc_node 的 ds.pos = local + world_birth 一致。
            # 用静态出生点：稳态正确；EKF 偶发 xy reset 后会有短暂偏差，不影响报告稳态指标。
            #
            # ⚠️ 动捕/外部定位是**全场共享原点**，msg.x/y 已经就是世界坐标，
            # 再加 birth 会**重复叠加**：pair2_in 实测间距被算成 5.600m（真值 2.800）、
            # 编队误差 1.400m（真值≈0），且数值稳定得很像真的。用 --shared-origin 关掉。
            # mpc_node 那侧靠校准 world_birth = birth − first_local 自动抵消，没有这个问题。
            base = 0.0 if self.shared_origin else self.birth[idx]
            d.pos = np.array([msg.x, msg.y, msg.z]) + base
            d.vel = np.array([msg.vx, msg.vy, msg.vz])
            d.received = True
            d.last_stamp = self.get_clock().now().nanoseconds * 1e-9
        return cb

    def _make_status_cb(self, idx):
        def cb(msg):
            d = self.drones[idx]
            d.arm_state   = msg.arming_state
            d.nav_state   = msg.nav_state
            d.status_recv = True
        return cb

    def _make_health_cb(self, idx):
        def cb(msg):
            if len(msg.data) < 6:
                return
            d = self.drones[idx]
            d.mpc_status     = int(msg.data[1])
            d.solve_ms       = float(msg.data[2])
            d.fallback_count = int(msg.data[3])
            d.hover_active   = bool(msg.data[4])
            d.pos_err        = float(msg.data[5])
            if len(msg.data) >= 9:          # 老节点只发 7 项，不覆盖默认 0
                d.plan_ratio_max = float(msg.data[7])
                d.clip_count     = int(msg.data[8])
            if len(msg.data) >= 12:         # §6.5 DOB d̂（老节点不发，保持 0）
                d.dhat = np.array([float(msg.data[9]), float(msg.data[10]),
                                   float(msg.data[11])])
            if len(msg.data) >= 13:         # 离地闩锁（老节点不发，保持 None→nan）
                d.airborne = int(msg.data[12])
            d.health_recv    = True
        return cb

    def _on_leader(self, msg):
        if len(msg.data) < 8:
            return
        self._leader_pos = np.array([msg.data[1], msg.data[2], msg.data[3]])
        self._leader_vel = np.array([msg.data[4], msg.data[5], msg.data[6]])
        self._leader_recv = True

    # ── 格式化辅助 ──────────────────────────────────────────────────────────
    @staticmethod
    def _arm_str(s):
        if s == 2: return '\033[32mARMED  \033[0m'
        if s == 1: return '\033[33mDISARMD\033[0m'
        return '\033[90mUNKNOWN\033[0m'

    @staticmethod
    def _nav_str(s):
        if s == 14: return '\033[32mOFFBOARD\033[0m'
        return f'\033[33mNAV={s:2d}  \033[0m'

    @staticmethod
    def _spacing_str(d):
        if d < SPACING_CRIT:
            return f'\033[31m{d:.2f}m<!CRIT\033[0m'
        if d < SPACING_WARN:
            return f'\033[33m{d:.2f}m<!WARN\033[0m'
        return f'\033[32m{d:.2f}m\033[0m'

    @staticmethod
    def _mpc_status_str(s):
        if s == 0: return '\033[32m OK \033[0m'
        if s == 2: return '\033[33mITER\033[0m'
        if s == -1: return '\033[90m--- \033[0m'
        return f'\033[31m ERR{s}\033[0m'

    def _print_status(self):
        now = time.time()
        elapsed = now - self._start_time
        h, rem = divmod(int(elapsed), 3600)
        m, s   = divmod(rem, 60)
        uptime = f'{h:02d}:{m:02d}:{s:02d}'
        ts = datetime.now().strftime('%H:%M:%S')

        lines = []
        lines.append(
            f'\033[2J\033[H'   # 清屏
            f'╔══ SWARM DIAG [{ts}] formation={self.formation} '
            f'uptime={uptime} ══╗'
        )

        # ── 领队 ──────────────────────────────────────────────────────────
        if self._leader_recv:
            lp = self._leader_pos
            lv = self._leader_vel
            lines.append(
                f'  Leader: pos=({lp[0]:+.2f}, {lp[1]:+.2f}, {lp[2]:+.2f})  '
                f'vel=({lv[0]:+.2f}, {lv[1]:+.2f})  \033[32m[RECV]\033[0m'
            )
        else:
            lines.append('  Leader: \033[31m[NO SIGNAL]\033[0m')

        lines.append('')

        # ── 每机状态 ──────────────────────────────────────────────────────
        target_alt = self.target_alt
        for i, d in enumerate(self.drones):
            if not d.received:
                lines.append(f'  Drone {i}: \033[31m[NO POSITION]\033[0m')
                continue

            z_err = d.pos[2] - target_alt
            vel_xy = float(np.linalg.norm(d.vel[:2]))
            age = now - d.last_stamp

            # MPC
            if d.health_recv:
                mpc_info = (
                    f'MPC={self._mpc_status_str(d.mpc_status)} '
                    f'solve={d.solve_ms:.1f}ms '
                    f'fallback={d.fallback_count}'
                    + ('\033[33m[HOVER]\033[0m' if d.hover_active else '')
                )
            else:
                mpc_info = '\033[90mMPC=[no diag topic]\033[0m'

            lines.append(
                f'  Drone {i}: '
                f'z={d.pos[2]:+.2f}m(err={z_err:+.2f}m)  '
                f'velXY={vel_xy:.2f}m/s  '
                f'{self._arm_str(d.arm_state)}  '
                f'{self._nav_str(d.nav_state)}  '
                f'age={age:.1f}s  '
                f'{mpc_info}'
            )

        lines.append('')

        # ── 机间距（全对枚举；违规计数与 min 均覆盖非邻居对）────────────────────
        if self.all_pairs:
            min_spacing = float('inf')
            argmin = None
            n_warn = 0
            spacing_strs = []
            for (i, j) in self.all_pairs:
                di, dj = self.drones[i], self.drones[j]
                if di.received and dj.received:
                    dist = float(np.linalg.norm(di.pos - dj.pos))
                    if dist < SPACING_CRIT:
                        self._safety_violations += 1
                    if dist < SPACING_WARN:
                        n_warn += 1
                    if dist < min_spacing:
                        min_spacing, argmin = dist, (i, j)
                    if self.num <= 5:
                        spacing_strs.append(f'{i}↔{j}: {self._spacing_str(dist)}')
                elif self.num <= 5:
                    spacing_strs.append(f'{i}↔{j}: \033[90m---\033[0m')

            # 大阵列不逐对打印（grid25 有 300 对），只给最小值 + 触发对 + 预警计数
            if self.num <= 5:
                lines.append('  Spacing: ' + '  '.join(spacing_strs))
            else:
                lines.append(
                    f'  Spacing: {len(self.all_pairs)} 对全枚举'
                    f'（逐对省略；<{SPACING_WARN}m 的对数={n_warn}）')
            if math.isfinite(min_spacing):
                who = f'  [d{argmin[0]}↔d{argmin[1]}]' if argmin else ''
                lines.append(
                    f'  Min spacing: {self._spacing_str(min_spacing)}{who}'
                    f'  (warn<{SPACING_WARN}m, crit<{SPACING_CRIT}m)'
                    f'  safety_violations={self._safety_violations}'
                )
        else:
            lines.append('  Spacing: N/A (solo1)')

        # ── 队形偏差 ──────────────────────────────────────────────────────
        offsets = self.cfg['offsets_ned']
        leader_xy = self._leader_pos[:2] if self._leader_recv else np.zeros(2)
        all_received = all(d.received for d in self.drones)
        if all_received and self._leader_recv:
            form_errs = []
            for i, d in enumerate(self.drones):
                expected_xy = leader_xy + offsets[i, :2]
                err = float(np.linalg.norm(d.pos[:2] - expected_xy))
                form_errs.append(err)
            max_err = max(form_errs)
            err_str = '  '.join(f'd{i}:{e:.2f}m' for i, e in enumerate(form_errs))
            color = '\033[32m' if max_err < 0.5 else ('\033[33m' if max_err < 1.0 else '\033[31m')
            lines.append(f'  Formation err: {err_str}  {color}max={max_err:.2f}m\033[0m')

        lines.append('')

        # ── Gate 状态摘要 ─────────────────────────────────────────────────
        all_armed    = all(d.arm_state == 2 for d in self.drones if d.status_recv)
        all_offboard = all(d.nav_state == 14 for d in self.drones if d.status_recv)
        any_hover    = any(d.hover_active for d in self.drones if d.health_recv)
        total_fb     = sum(d.fallback_count for d in self.drones)
        max_solve_ms = max((d.solve_ms for d in self.drones if d.health_recv), default=0.0)

        g_arm  = '\033[32m✓\033[0m' if all_armed    else '\033[31m✗\033[0m'
        g_ofb  = '\033[32m✓\033[0m' if all_offboard else '\033[31m✗\033[0m'
        g_mpc  = '\033[32m✓\033[0m' if not any_hover else '\033[33m!\033[0m'
        g_safe = '\033[32m✓\033[0m' if self._safety_violations == 0 else '\033[31m✗\033[0m'

        lines.append(
            f'  Gate checks: ARM={g_arm}  OFFBOARD={g_ofb}  '
            f'MPC_ok={g_mpc}(fallbacks={total_fb})  '
            f'SAFE={g_safe}(violations={self._safety_violations})  '
            f'max_solve={max_solve_ms:.1f}ms'
        )
        lines.append('╚' + '═' * 70 + '╝')

        print('\n'.join(lines), flush=True)
        self._log_row(elapsed)


    def _log_row(self, elapsed):
        if self._csv is None:
            return
        nan = float('nan')
        row = [f'{elapsed:.1f}']
        for d in self.drones:
            if d.received:
                x = float(d.pos[0])
                y = float(d.pos[1])
                z = float(d.pos[2])
                zerr = z - self.target_alt
                velxy = float(np.linalg.norm(d.vel[:2]))
            else:
                x = y = z = zerr = velxy = nan
            row += [f'{x:.3f}', f'{y:.3f}', f'{z:.3f}', f'{zerr:.3f}',
                    f'{velxy:.3f}',
                    str(d.arm_state), str(d.nav_state), str(d.mpc_status),
                    f'{d.solve_ms:.3f}', str(d.fallback_count),
                    str(int(d.hover_active)), f'{d.pos_err:.3f}',
                    f'{d.plan_ratio_max:.4f}', str(d.clip_count),
                    f'{d.dhat[0]:.4f}', f'{d.dhat[1]:.4f}', f'{d.dhat[2]:.4f}',
                    ('nan' if d.airborne is None else str(d.airborne))]
        min_sp = nan
        if self.all_pairs:
            dmin = float('inf')
            for (i, j) in self.all_pairs:
                di, dj = self.drones[i], self.drones[j]
                if di.received and dj.received:
                    dmin = min(dmin, float(np.linalg.norm(di.pos - dj.pos)))
            if math.isfinite(dmin):
                min_sp = dmin
        max_ferr = nan
        if self._leader_recv and all(d.received for d in self.drones):
            offs = self.cfg['offsets_ned']
            lxy = self._leader_pos[:2]
            max_ferr = max(
                float(np.linalg.norm(self.drones[i].pos[:2] - (lxy + offs[i, :2])))
                for i in range(self.num))
        total_fb  = sum(d.fallback_count for d in self.drones)
        max_solve = max((d.solve_ms for d in self.drones if d.health_recv), default=0.0)
        if self._leader_recv:
            lx, ly   = float(self._leader_pos[0]), float(self._leader_pos[1])
            lvx, lvy = float(self._leader_vel[0]), float(self._leader_vel[1])
        else:
            lx = ly = lvx = lvy = nan
        row += [f'{min_sp:.3f}', f'{max_ferr:.3f}', str(self._safety_violations),
                str(total_fb), f'{max_solve:.3f}',
                f'{lx:.3f}', f'{ly:.3f}', f'{lvx:.3f}', f'{lvy:.3f}']
        self._csv.write(','.join(row) + '\n')


def main():
    parser = argparse.ArgumentParser(description='编队健康诊断监控')
    parser.add_argument('--formation', '-f', default='pair2',
                        choices=list(FORMATION_CFG.keys()),
                        help='队形名称')
    parser.add_argument('--log', nargs='?', const='', default=None,
                        help='把每秒指标写入 CSV：给路径用之，省略路径则自动命名 '
                             'flight_<formation>_<时间戳>.csv')
    parser.add_argument('--scenario', default=None,
                        help='场景名（如 OUT_solo1_hover）。给了就从 config/scenarios.yaml '
                             '读该场景的 limits.target_alt 当 zerr 基准 —— **强烈建议给**，'
                             '否则用本文件里那份容易漂的副本')
    parser.add_argument('--target-alt', type=float, default=None,
                        help='直接指定 zerr 的参考高度（NED，负=向上），优先级最高')
    parser.add_argument('--shared-origin', action='store_true',
                        help='定位源为全场共享原点（动捕/外部定位）：local 已是世界'
                             '坐标，不再加出生点。不加会重复叠加 —— pair2_in 实测'
                             '间距被算成 5.600m(真值 2.800)、编队误差 1.400m(真值≈0)，'
                             '而且数值很稳定、看着像真的')
    args, ros_args = parser.parse_known_args()

    log_path = None
    if args.log is not None:
        log_path = args.log or \
            f'flight_{args.formation}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

    rclpy.init(args=ros_args)
    try:
        node = DiagMonitor(args.formation, log_path, args.shared_origin,
                           args.scenario, args.target_alt)
        node.get_logger().info(
            f'zerr 参考高度 target_alt = {node.target_alt:+.2f} m'
            f'（来源：{node._alt_src}）')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError:
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
