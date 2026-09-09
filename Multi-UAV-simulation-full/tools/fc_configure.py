#!/usr/bin/env python3
"""X6 Air+ 真机参数批量配置（MAVLink）。

默认 **dry-run 只显示不写入**；确认后加 --apply 才真正写。
写入前自动备份被改参数原值，写入后逐项读回验证。

用法:
    python3 fc_configure.py --list                    # 看有哪些组
    python3 fc_configure.py -g limits,failsafe        # dry-run 预览
    python3 fc_configure.py -g limits,failsafe --apply
    python3 fc_configure.py -g ekf2_mocap --apply     # 到飞场前再执行

⚠️ 需要交互的三项**不在本脚本内**，必须在 QGC 图形界面做：
   传感器校准 / 电调校准与电机测试 / RC 通道映射（含 kill 开关）
"""
import argparse
import csv
import os
import struct
import sys
import time
from datetime import datetime

from pymavlink import mavutil

INT_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8, mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32, mavutil.mavlink.MAV_PARAM_TYPE_INT32,
}

# (参数, 目标值, 是否整型, 理由)
GROUPS = {
    'failsafe': ('失效保护 —— 修危险默认值', [
        ('COM_OBL_RC_ACT', 4, True,
         'offboard 丢失动作 0(Position)→4(Land)。Position 在室内无 GPS 时无源可依'),
        ('COM_OF_LOSS_T', 0.2, False, 'offboard 超时 1.0→0.2s（飞场固定要求）'),
        ('NAV_RCL_ACT', 3, True, 'RC 失联 2(Return)→3(Land)，室内 RTL 比就近落危险'),
        ('COM_RC_LOSS_T', 0.5, False, 'RC 超时（已是 0.5，确认用）'),
        ('COM_RCL_EXCEPT', 0, True,
         'RC 失联失效保护例外位掩码。0=所有模式（含 OFFBOARD）都对 RC 失联做失效保护 → '
         '编队 OFFBOARD 飞行中遥控掉线也会 Land。⚠️ 设 4 会让 OFFBOARD 忽略 RC 失联（危险，别设）。'
         '出厂即 0，纳入管理仅为每次体检显式核验、防被误改'),
    ]),
    'limits': ('速度/姿态硬帽 —— 当前是室外默认值，7×8.5m 飞场等于没限制', [
        ('MPC_XY_VEL_MAX', 2.0, False, '水平速度 12.0→2.0（须 > companion 限幅 1.5）'),
        ('MPC_Z_VEL_MAX_UP', 1.5, False, '升速 3.0→1.5'),
        ('MPC_Z_VEL_MAX_DN', 1.0, False, '降速 1.5→1.0'),
        ('MPC_TILTMAX_AIR', 25.0, False, '最大倾角 45→25 度'),
        ('MPC_XY_CRUISE', 1.0, False, '巡航速度 5.0→1.0'),
        ('MPC_TKO_SPEED', 0.5, False, '起飞速度 1.5→0.5'),
        ('MPC_LAND_SPEED', 0.3, False, '降落速度 0.7→0.3'),
        ('MIS_TAKEOFF_ALT', 1.2, False, '自动起飞高度 2.5→1.2（对齐 IN_* 场景）'),
    ]),
    'geofence': ('电子围栏 —— 当前 GF_MAX_*=0 即未启用', [
        # 相对各机 home。3.0m 覆盖 IN_* 最大离 home 位移(单机圆周 2×1.2=2.4m)+余量；
        # 更紧的飞散保护由 companion safety_max_track_dist(1.0~1.5m)负责，此为外层兜底。
        # (2026-07-30 随 IN_* 轨迹按 7×8.5m 场地放大，从 2.0→3.0；旧 2.0 会拦住新圆周)
        ('GF_MAX_HOR_DIST', 3.0, False, '水平围栏 0→3.0m（离 home；配合 IN_* 放大后的轨迹）'),
        ('GF_MAX_VER_DIST', 3.0, False, '垂直围栏 0→3.0m（任务高度 1.2m，足够）'),
        # GF_ACTION 当前=2(Hold)。室内 Hold(原地悬停等飞手)比 Land 可控，暂不改。
    ]),
    'failsafe_out15': ('失控保护·室外 15m 场地（2026-08-20 建，两机统一）', [
        # 2026-08-20 实测发现两机失控配置**实质性不一致**，编队时会各飞各的：
        #   COM_OBL_RC_ACT  A10=0(Position 危险默认值) / A08=4(Land)
        #   COM_OF_LOSS_T   A10=1.0s / A08=0.2s（差 5 倍）
        #   NAV_RCL_ACT     两机都是 2(Return) —— 与本项目已确立的原则矛盾（见下）
        #
        # 🔑 贯穿原则：**航向与位置精度在飞行中验证之前，不启用任何让飞机自主飞行一段距离的动作。**
        #    据此 GF_ACTION 保持 Hold、COM_LOW_BAT_ACT 选 Land 而非 Return；本组把 RC 失联也对齐。
        ('NAV_RCL_ACT', 3, True,
         'RC 失联 2(Return)→3(Land)。Return 会自主飞回并降落，用的正是尚未在飞行中验证过的'
         '航向；15m 场地就地降落更可控，且两名飞手在场'),
        ('COM_OBL_RC_ACT', 4, True,
         'offboard 丢失 0(Position)→4(Land)。A10 原为危险默认值 0；Position 依赖位置估计，'
         'GPS 退化时无源可依。统一到 A08 已有的 4，也与飞场文档一致'),
        ('COM_OF_LOSS_T', 0.2, False,
         'offboard 超时 1.0→0.2s（飞场固定要求）。⚠️ 0.2s 安全的前提是 offboard 流走'
         '**有线以太网**(FC↔Jetson eno1)而非无线 mesh —— 无线链路上这个值会太激进'),
        ('MPC_XY_VEL_MAX', 3.0, False,
         '水平速度硬帽 12.0→3.0。offboard 下 companion 已限到 ≤1.5 不受此约束，但**飞手切回'
         'Position 接管时 12m/s 是生效的** —— 15m 场地 1.25 秒穿场。须 > companion 限幅 1.5'),
        ('MPC_TILTMAX_AIR', 25.0, False, '最大倾角 45→25 度，首飞限制姿态激烈程度'),
        # 🔴 纵深防御：即使 NAV_RCL_ACT 已改 Land，**飞手仍可手动选 RTL 模式**，那时下面这个
        #    高度照样生效。原值 30m > 垂直围栏 GF_MAX_VER_DIST=15m ⇒ **RTL 爬升会撞破围栏、
        #    触发围栏动作(Hold)，两个失效保护互相打架**，飞机行为不可预测。必须降到围栏以内。
        #    （PX4 RTL 锥形逻辑：距 home < RTL_MIN_DIST(10m) 时按 RTL_CONE_ANG(45°) 取较低
        #     返航高度；我们轨迹最远离 home 仅 5m，正常情况本就只爬约 1m——30m 只在飞机
        #     已经跑出 10m 时才生效，而那正是失控保护该起作用的时刻。）
        ('RTL_RETURN_ALT', 10.0, False, '返航高度 30→10m，压到垂直围栏 15m 以内避免两个失效保护冲突'),
        # ⚠️ RC_MAP_KILL_SW **不在本组**：A10=9 / A08=8，两机不同。这取决于各自遥控器的
        #    物理配置（双机方案是两个遥控器、两名飞手），**不能盲目统一**，须逐台对着实物核对。
        # ⚠️ COM_RCL_EXCEPT 保持 0：设 4 会让 OFFBOARD 忽略 RC 失联 = 拆掉最后一道闸。
    ]),
    'ekf2_rtk': ('EKF2 室外 RTK/GPS 定位 —— ⚠️ 与 ekf2_mocap 互斥，切换后须重启飞控', [
        # 2026-08-20 建。与 'ekf2_mocap' 组正好相反，别同时下发。
        ('EKF2_GPS_CTRL', 7, True, '采信 GPS 位置+速度+高度（动捕下是 0=禁用）'),
        ('EKF2_EV_CTRL', 0, True, '关闭外部视觉/动捕（动捕下是 11）'),
        ('EKF2_BARO_CTRL', 1, True, '开气压计（动捕下被禁用；室外作高度源或辅助）'),
        ('EKF2_MAG_TYPE', 0, True, 'Automatic —— 室外靠磁罗盘定航向（动捕下是 5=None）'),
        # ⚠️ EKF2_HGT_REF 不在本组：0(气压计) vs 1(GPS 高度) 是**需要按现场决定**的取舍——
        #    RTK 未到 Fixed 时 GPS 垂直误差 2~5m 且 fix 质量变化会跳变(§10 B3)，
        #    此时气压计在悬停尺度上更稳；RTK 稳定 Fixed 后再切 1 拿厘米级垂直。
        # ⚠️ COM_ARM_WO_GPS 也不在本组：室外建议改 0(强制有 GPS 才解锁)，但 GPS 未达标前
        #    改 0 会导致连台架解锁都做不了，把无桨验证一起堵死。等 GPS 验收通过再单独设。
    ]),
    # ── 高度参考源：两组互斥，按 RTK 状态二选一（2026-08-20 建）──────────────────
    # 🔑 取舍是**有条件的**，不存在"总是更好"的一方：
    #   · RTK 稳定 Fixed  → 用 hgt_gps：垂直厘米级，明显优于气压计（气压计受风/温漂影响大）
    #   · 无 RTK / 只有单点 → 用 hgt_baro：GPS 垂直误差 2~5m 且 **fix 质量变化时会跳变**，
    #                        而气压计的漂移是缓变的、不会阶跃
    # ⚠️ 外部经验佐证（CSDN「小小洋洋」2022-12 多机 RTK 实测）：把高度源改成 GPS 后，
    #    指令 2m 的飞机冲到 8m —— 机理是**测得的高度不断下降（实际没降）→ 飞机为保持高度
    #    持续加油门**。其前提是他们 RTK 已达 cm 级仍出事，我们连 Fixed 都没进过，风险更高。
    # 🔴 这类"估计错误"型故障，**`safety_max_alt` 拦不住**——安全滤波器用的是同一个错误估计。
    'hgt_baro': ('高度参考=气压计 —— RTK 未稳定 Fixed 时用这组（首飞默认）', [
        ('EKF2_HGT_REF', 0, True, '高度参考 → 0(气压计)。GPS 垂直误差大且会跳变时更安全'),
        ('EKF2_BARO_CTRL', 1, True, '确保气压计已启用'),
    ]),
    'hgt_gps': ('高度参考=GPS —— **仅在 RTK 稳定 Fixed(fix_type=6) 后**才切这组', [
        ('EKF2_HGT_REF', 1, True, '高度参考 → 1(GPS)。RTK Fixed 下垂直厘米级'),
        ('EKF2_GPS_CTRL', 7, True, '确保 GPS 位置+速度+高度全开'),
    ]),
    'geofence_out15': ('电子围栏·室外 15m 半径场地（2026-08-19 建，配合 OUT_solo1_*）', [
        # 安全分层（顺序不能反：companion 正常管，飞控围栏兜底，围栏再留场地余量）：
        #   场地半径 15m
        #     └ GF_MAX_HOR_DIST = 12m       ← 留 3m 反应余量（相对各机 home/起飞点）
        #         └ OUT_solo1_* 轨迹最远离原点 4~5m
        #             └ 最坏 = 轨迹 5 + safety_max_track_dist 3 = 8m < 12m ✅
        # ⚠️ 与室内 'geofence' 组互斥，别同时下发（那组是 3.0m，室外会把飞机拦死在起飞点附近）。
        ('GF_MAX_HOR_DIST', 12.0, False, '水平围栏 0→12.0m（场地半径 15m，留 3m 余量）'),
        ('GF_MAX_VER_DIST', 15.0, False, '垂直围栏 0→15.0m（任务高度 4m / safety_max_alt 8m，足够）'),
        # GF_ACTION 当前=2(Hold)：触发后原地悬停等飞手接管。室外首飞保持 Hold，
        # 不改成 RTL —— RTL 会自主飞回并降落，而航向/位置精度都还没在飞行中验证过。
        # 一次只改一个变量，确认飞行正常后再考虑。
    ]),
    'battery_pmu': ('电池·One PMU（DroneCAN）—— 2026-08-20 万用表验证通过后启用低电量动作', [
        # 🔑 One PMU 走 DroneCAN（UAVCAN_ENABLE=2 / UAVCAN_SUB_BAT=1），**电压电流由模块
        #    自己标定后上报**，PX4 不做分压换算 ⇒ `BAT1_V_DIV` / `BAT1_A_PER_V` 恒为 -1
        #    是**正常的**，不代表"未标定"。旧注释里"须等 One PMU 标定"针对的是模拟分压模块。
        # ✅ 2026-08-20 用万用表对比验证：A10 实测 16.09V / 上报 16.14V（差 0.05V）；
        #    A08 实测 16.27V / 上报 16.27V（差 0.00V）。均在 0.1V 判据内 ⇒ 读数可信。
        ('BAT1_N_CELLS', 4, True, '4S 锂电（A10 原为 0=未设）'),
        ('BAT_CRIT_THR', 0.07, False, '临界阈值，两机统一到 0.07'),
        ('BAT_EMERGEN_THR', 0.05, False, '紧急阈值，两机统一到 0.05'),
        # 🔴 COM_LOW_BAT_ACT=2(Land)：在 BAT_LOW_THR(15%) 触发**就地降落**。
        #    不用 1(Return)/3(含 Return) —— RTL 会自主飞回并降落，用的正是**尚未在飞行中
        #    验证过的航向与位置精度**。与 GF_ACTION 保持 Hold 不改 RTL 是同一条原则：
        #    航向/定位经飞行验证前，不启用任何让飞机自主飞行一段距离的动作。
        # ⚠️ 15% 触发对长航时测试偏早，确认可靠后可考虑改 3(分级)。
        ('COM_LOW_BAT_ACT', 2, True, '低电量动作 0(仅警告)→2(Land)，室外必须有兜底'),
        # ⚠️ BAT1_CAPACITY 不在本组：**逐机不同**（A10=4000mAh / A08=5300mAh），单独设。
        # ⚠️ BAT1_SOURCE 不在本组：两机不一致(A10=-1/A08=0)但 UAVCAN 模式下遥测均正常，
        #    一次只改一个变量，暂不动。
    ]),
    'battery_thr': ('电池阈值（旧组，已被 battery_pmu 取代）—— 仅阈值，不含动作', [
        ('BAT_CRIT_THR', 0.07, False, '临界 0.05→0.07'),
        ('BAT_EMERGEN_THR', 0.05, False, '紧急 0.03→0.05'),
        # COM_LOW_BAT_ACT 不在此处：电压未标定就设动作会误触发。
        # BAT1_SOURCE/V_DIV/A_PER_V/N_CELLS 须按实际电池与 PMU 在 QGC 电源页标定。
    ]),
    'ekf2_mocap': ('EKF2 外部视觉（动捕）—— ⚠️ 执行后气压计被禁用，Altitude 手动试飞将不可用', [
        ('EKF2_EV_CTRL', 11, True, '融合外部视觉位置+姿态（飞场固定要求）'),
        ('EKF2_HGT_REF', 3, True, '高度参考=vision（3=Vision，执行前请在 QGC 确认枚举）'),
        ('EKF2_GPS_CTRL', 0, True, '禁用 GPS'),
        ('EKF2_BARO_CTRL', 0, True, '禁用气压计（避免与视觉高度冲突）'),
        ('EKF2_RNG_CTRL', 0, True, '禁用测距仪'),
        # ⚠️ 飞场文档没写这条，但缺了它解不了锁：MAG_TYPE=0(Automatic) 时 EKF 会
        # 同时融合磁力计偏航与视觉偏航，两者互相打架 → "Preflight Fail: Yaw
        # estimate error"，arming_state 卡在 INIT。2026-07-23 SITL 实测确认。
        # 室内金属/电机磁干扰下磁力计本就不可信，动捕方案应完全交给视觉偏航。
        ('EKF2_MAG_TYPE', 5, True, '磁力计类型=None(5)，偏航完全交给视觉'),
    ]),
    'xrce': ('XRCE-DDS 以太网（第1步）—— 执行后**必须重启飞控**', [
        ('UXRCE_DDS_CFG', 1000, True, '传输方式=Ethernet（1000，与 MAV_2_CONFIG 同枚举）'),
        # UXRCE_DDS_AG_IP 不在本组：它是 PX4 **条件参数**，要等 UXRCE_DDS_CFG 启用
        # 并重启后才出现（重启前读不到，本脚本会拒绝写入）。见下面的 xrce_ag_ip 组。
    ]),
    'xrce_ag_ip': ('XRCE-DDS Agent 地址（第2步，**重启后**才能执行）', [
        # 飞控↔Orin NX 走**隔离的点对点网段** 192.168.77.0/24（飞控 .2 / Orin NX .100）。
        # ⚠️ 为什么不用飞场网段 10.41.10.x：飞控出厂 netman 配的就是 10.41.10.2 +
        #    网关 10.41.10.254，说明**飞场自己的网络就是 10.41.10.0/24**。而 Orin NX
        #    还要连飞场 WiFi 取动捕(VRPN)数据 ⇒ 两个接口同网段会造成路由歧义，
        #    发往 VRPN 的包可能被塞进只通飞控的网线里黑洞掉，且症状极像"动捕没数据"。
        #    换成私有段后，飞场 WiFi 分到什么地址都不冲突。(2026-07-23)
        # 整数换算：a×2²⁴+b×2¹⁶+c×2⁸+d
        #          = 192×16777216+168×65536+77×256+100 = 3232255332
        # 默认值是 127.0.0.1(2130706433)，不改则 Agent 永远连不上。
        ('UXRCE_DDS_AG_IP', 3232255332, True, 'Agent 地址=192.168.77.100（Orin NX）'),
    ]),
}


def as_int32(v):
    """把 uint32 语义的值折成有符号 int32。

    PX4 的 IP 类参数（UXRCE_DDS_AG_IP 等）声明为 INT32，但 IP 的自然整数
    形式在首字节 ≥128 时会超出 int32 上限：
        10.41.10.100  →  170461796      ✅ 装得下
        192.168.77.100 → 3232255332     ❌ > 2147483647，struct.pack('<i') 直接抛异常
    读回时 decode() 得到的也是有符号值（-1062711964），不折算则比对永远不等、
    每次都判定"需要修改"且验证必失败。(2026-07-23 实测踩到)
    """
    v = int(v)
    return v - 0x100000000 if v > 0x7FFFFFFF else v


def decode(r):
    if r.param_type in INT_TYPES:
        return struct.unpack('<i', struct.pack('<f', r.param_value))[0]
    return round(r.param_value, 6)


def read_param(m, name, timeout=3.0, retries=3):
    """读参数。MAVLink 无重传，首次连接时请求偶发丢失（实测过），故重试。"""
    for attempt in range(retries):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if r is None:
                continue
            if r.param_id.strip('\x00') == name:
                if attempt:
                    print(f'      （第 {attempt + 1} 次尝试才读到）')
                return decode(r), r.param_type
    return None, None


def write_param(m, name, value, is_int, ptype):
    if is_int:
        pv = struct.unpack('<f', struct.pack('<i', int(value)))[0]
        t = ptype if ptype in INT_TYPES else mavutil.mavlink.MAV_PARAM_TYPE_INT32
    else:
        pv = float(value)
        t = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), pv, t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-d', '--dev', default='/dev/ttyACM0',
                    help='串口设备（USB 链路）。用 --udp 时忽略')
    ap.add_argument('--udp', default='',
                    help='以太网链路：飞控 MAVLink 目标 IP:端口，如 192.168.77.2:14550。'
                         '给了就走 UDP，不用串口。')
    ap.add_argument('-g', '--groups', default='')
    ap.add_argument('--apply', action='store_true', help='真正写入（默认只预览）')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()

    if a.list:
        for k, (desc, items) in GROUPS.items():
            print(f'  {k:14s} {desc}  ({len(items)} 项)')
        return

    sel = [g.strip() for g in a.groups.split(',') if g.strip()]
    if not sel:
        print('未指定 -g，用 --list 查看可选组'); sys.exit(1)
    for g in sel:
        if g not in GROUPS:
            print(f'未知组: {g}'); sys.exit(1)

    if a.udp:
        # 以太网广播模式(MAV_2_BROADCAST=1)：飞控广播 HEARTBEAT 到 x.x.x.255:port。
        # 早先用 udpout（临时本地端口）收不到广播、要靠飞控学到我们后转单播，时序不稳
        # （2026-07-27 台架实测：dry-run 偶通、--apply 连续两次失败）。改用 udpin 绑定
        # 本地 port 直接收广播，稳；收到后发一拍心跳让飞控也学到我们、把 PARAM_VALUE
        # 单播回本端口。8 台机同样走以太网，此接法通用。
        port = a.udp.split(':')[-1]
        conn = f'udpin:0.0.0.0:{port}'
        m = mavutil.mavlink_connection(conn, source_system=250, source_component=190)
        print(f'连接 {conn} ...（绑定本地 {port} 直接收飞控广播）')
        if m.wait_heartbeat(timeout=15) is None:
            print('❌ 没等到 heartbeat（UDP）。确认飞控在线、MAV_2 在以太网广播')
            sys.exit(1)
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)
    else:
        m = mavutil.mavlink_connection(a.dev, baud=57600)
        print(f'连接 {a.dev} ...')
        if m.wait_heartbeat(timeout=15) is None:
            print('❌ 没等到 heartbeat'); sys.exit(1)
    print(f'✅ sysid={m.target_system}\n')

    plan, backup, unread = [], [], []
    for g in sel:
        desc, items = GROUPS[g]
        print(f'=== [{g}] {desc} ===')
        for name, target, is_int, why in items:
            # 整型目标值统一折成有符号 int32，与读回值同口径（见 as_int32 注释）
            if is_int:
                target = as_int32(target)
            cur, ptype = read_param(m, name)
            if cur is None:
                print(f'  {name:18s} ❌ 重试 3 次仍读不到')
                unread.append(name); continue
            same = abs(float(cur) - float(target)) < 1e-6
            mark = '（已是目标值，跳过）' if same else f'{cur} → {target}'
            print(f'  {name:18s} {mark}')
            print(f'      {why}')
            if not same:
                plan.append((name, cur, target, is_int, ptype))
                backup.append((name, cur))
        print()

    if unread:
        print(f'⚠️ {len(unread)} 项读不到：{", ".join(unread)}')
        if a.apply:
            # 读不到 = 无法确认当前值，也无法备份。安全参数上静默漏配比报错危险得多。
            print('❌ 拒绝写入。请检查连接后重跑（MAVLink 首次连接偶发丢包）。')
            sys.exit(2)
        print()

    if not plan:
        print('没有需要修改的项。'); return

    if not a.apply:
        print(f'*** DRY-RUN：以上 {len(plan)} 项**未写入**。确认无误后加 --apply 执行。***')
        return

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    bak = os.path.expanduser(f'~/fc_param_backup_{ts}.csv')
    with open(bak, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['name', 'old_value'])
        w.writerows(backup)
    print(f'原值已备份 → {bak}\n')

    print(f'开始写入 {len(plan)} 项...')
    for name, cur, target, is_int, ptype in plan:
        write_param(m, name, target, is_int, ptype)
        time.sleep(0.25)

    print('\n=== 读回验证 ===')
    ok = fail = 0
    for name, cur, target, is_int, ptype in plan:
        got, _ = read_param(m, name)
        good = got is not None and abs(float(got) - float(target)) < 1e-6
        print(f'  {name:18s} {cur} → {got}  {"✅" if good else "❌ 未生效"}')
        ok, fail = (ok + 1, fail) if good else (ok, fail + 1)
    print(f'\n成功 {ok} / 失败 {fail}')
    if fail:
        print('⚠️ 失败项请在 QGC 里手动确认（部分参数可能被条件隐藏或需重启）')
    print('⚠️ 带 reboot_required 的参数（UXRCE_DDS_*）需重启飞控后生效')


if __name__ == '__main__':
    main()
