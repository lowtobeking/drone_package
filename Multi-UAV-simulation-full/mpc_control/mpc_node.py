#!/usr/bin/env python3
import hashlib
import json
import math
import os
import random
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension, Float64, String
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

from mpc_control.safety_filter import SafetyFilter, NORMAL as SAFETY_NORMAL


# ------------------------------------------------------------------ helpers
_WGS84_A = 6378137.0                 # m，WGS84 长半轴
_WGS84_F = 1.0 / 298.257223563       # WGS84 扁率
_WGS84_E2 = 2.0 * _WGS84_F - _WGS84_F * _WGS84_F


def _latlon_to_ned(lat, lon, lat0, lon0):
    """小范围经纬度(deg)→本地 NED(m，相对 lat0/lon0)。等距圆柱一阶近似，
    但**按 lat0 处的 WGS84 局部曲率半径**取比例尺，而非球面平均半径。
    返回 (north, east)。

    🔴 2026-08-30 修：原实现南北、东西两个方向共用球面 R=6371km。地球是扁的，
       在本项目场地(实测 29.92°N)子午圈曲率半径 M=6351.3km、卯酉圈 N=6383.5km，
       ⇒ 原式北向偏大 +0.310%、东向偏小 -0.195%。折算成距离：
         4m 基线  → 北 +1.24cm / 东 -0.78cm
        15m 场地  → 北 +4.65cm / 东 -2.93cm
       100m       → 北 +31cm  / 东 -20cm （原 docstring 声称"百米量级内厘米级"，不成立）
       这是**系统性尺度误差**，不是噪声：它把整个队形按方向各向异性地缩放。
       4m 间距下约 1cm、勉强可忍，但本函数存在的理由正是"让 RTK 的 cm 精度替代
       盲信配置摆位"——留一个 cm 级的系统偏差与该目的自相矛盾，且修它是免费的。
       曲率半径公式无外部依赖，只多两次乘除。
    """
    s = math.sin(math.radians(lat0))
    w = math.sqrt(1.0 - _WGS84_E2 * s * s)
    r_north = _WGS84_A * (1.0 - _WGS84_E2) / (w * w * w)   # 子午圈曲率半径 M
    r_east = _WGS84_A / w                                  # 卯酉圈曲率半径 N
    n = math.radians(lat - lat0) * r_north
    e = math.radians(lon - lon0) * r_east * math.cos(math.radians(lat0))
    return n, e


def make_px4_qos():
    """订阅 PX4 'out' 话题：PX4 DataWriter 用 TRANSIENT_LOCAL，ROS2 订阅者匹配。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def make_swarm_qos():
    """机间广播（邻居预测轨迹）：BEST_EFFORT。

    这类数据 50Hz 周期发布、**过期即废** —— 重传到达时早被新帧取代，毫无价值。
    默认的 RELIABLE 在 localhost 上零成本，但真机跨机走 2.4G 无线时，丢一包就
    触发重传，白占本就紧张的带宽、反过来加剧拥塞（估算 grid9 约 9.6 Mbps）。

    ⚠️ **发布端与订阅端必须同时用本 QoS**：BEST_EFFORT 发布 + RELIABLE 订阅
    互不兼容，会【静默】收不到（节点正常、零报错、邻居数据永远为空）。
    ⚠️ 真机多机部署时**所有机必须同步更新代码**，新旧版本混跑同样不匹配。
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def make_px4_pub_qos():
    """发布到 PX4 'in' 话题：PX4 DataReader 用 VOLATILE，ROS2 发布者必须匹配。
    用 TRANSIENT_LOCAL 会导致多机时消息丢失、OFFBOARD 丢失、ARM 失败。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.VOLATILE,
    )


def quaternion_to_yaw(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def safe_finite(x, default=0.0):
    return float(x) if math.isfinite(x) else float(default)


def topic_for_drone(drone_id, suffix):
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'


def mpc_topic_for_drone(drone_id, suffix='predicted_trajectory'):
    if drone_id == 0:
        return f'/mpc/{suffix}'
    return f'/px4_{drone_id}/mpc/{suffix}'


# ------------------------------------------------------------------ MPC core
class DoubleIntegratorMPC:
    """两种模型，由 `vel_lag_tau` 单参数切换（其余逐字相同，便于 A/B 对照）：

    * `vel_lag_tau <= 0` —— **legacy**：`v̇ = u`（u=加速度），下发 `x_pred[1]` 的速度分量，
      即模型假设速度设定点瞬时实现（τ=0）。
    * `vel_lag_tau > 0`  —— **接口一致（2026-07-14 起为默认，τ=0.5）**：`v̇ = (u − v)/τ`，
      输入 `u := v_sp` 正是实际下发给 PX4 的量，故直接下发 `u0`。

    依据（报告 §5.G，2026-07-09 实测）：PX4 速度环有一阶滞后 τ≈0.48 s（风下 0.61 s；
    旁证 1/MPC_XY_VEL_P_ACC=0.55 s）。τ=0 的模型使 MPC 高估自身加速度权限约 10 倍
    （OCP 写 a_max=4，实得 0.13–0.53 m/s²），于是每拍规划一个物理上不可能的恢复动作
    → 持续饱和 → 过冲 → 被速度限幅兜成极限环（幅值与周期均 ∝ max_speed）。
    消融：把对象滞后拿掉即稳（非整定问题）；无风、仅踢 5 m 也照样极限环（风只是把状态
    推过 u0 饱和阈的触发器，不是持续扰动源，故 d̂ 前馈无效——且下发状态时它本就是空操作）。
    """

    # SOC 范数约束的多面体面数 = 2*K（K 个方向、双边）。
    # 内接圆 ⇒ 严格 ‖·‖<=R，轴向权限损失 1-cos(π/2K)：K=8 → 1.92%，K=16 → 0.48%。
    # 增大 K 线性增加约束行数（求解耗时），K=8 已把 √2=41.4% 的泄漏压到 0。
    _SOC_FACET_DIRS = 8

    def __init__(self, N=20, dt=0.05, max_speed=5.0, max_climb=1.5,
                 max_accel=5.0, max_neighbours=4, d_safe=1.2,
                 w_collision=200.0, w_formation=0.5,
                 q_pos=4.0, q_vel=1.0, r_acc=0.1,
                 q_pos_terminal_scale=2.0, vel_lag_tau=0.0,
                 soc_norm_enable=False, dob_enable=False,
                 build_dir='~/.cache/mpc_control/acados_di_mpc', instance_id=0):
        self.N = N; self.dt = dt
        self.max_speed = max_speed; self.max_climb = max_climb
        self.max_accel = max_accel
        self.max_neighbours = max(1, int(max_neighbours))
        self.d_safe = d_safe
        self.w_collision = w_collision; self.w_formation = w_formation
        self.q_pos = q_pos; self.q_vel = q_vel; self.r_acc = r_acc
        self.q_pos_terminal_scale = q_pos_terminal_scale
        self.vel_lag_tau = float(vel_lag_tau)
        self.lag_model = self.vel_lag_tau > 0.0
        self.soc_norm = bool(soc_norm_enable)
        # 论文 §6.5 DOB-MPC 对照 baseline（审稿意见 M2）：在**legacy 零滞后模型**
        # (v̇=u) 上加一个加性集总扰动前馈 d̂（FxTDO 一族，[Xu24]），模型改为 v̇=u+d̂，
        # d̂ 由在线固定时间观测器估计（见 mpc_node._dob_update）、每拍作为常量参数注入、
        # horizon 内保持不变。**只在 legacy 上加**（lag 模型已把滞后建进 v̇=(u−v)/τ，
        # 再叠 DOB 无意义）；与 lag_model 互斥。用于经验验证否定性结果：加性 d̂ 无法
        # 复现输入通道滞后的快速动态（§3.3/§5.3）。
        self._dob = bool(dob_enable) and not self.lag_model
        self._dist = np.zeros(3)   # 当拍 d̂，solve() 每次刷新；_pack_params 注入

        _m = max(1, int(max_neighbours))
        # 两种模型的生成码不可混用 → 缓存目录与 model.name 均带标记
        _tag = ('lag' + f'{self.vel_lag_tau:.3f}'.replace('.', 'p')) if self.lag_model else 'di'
        if self.soc_norm:
            _tag += 'soc'          # 约束结构变了 → 生成码不可复用，必须换缓存目录
        if self._dob:
            _tag += 'dob'          # 模型多一个扰动参数(p 维度变) → 生成码不可复用，必须换缓存目录
        _tag += 'sb'               # 速度状态箱软化(idxsbx)；同上，旧生成码不可复用
        # expanduser：允许 yaml 里写 ~/... 的持久路径（各机器用户名不同：
        # Jetson=nvidia / 仿真机=shirui / RPi=pi）。默认的 /tmp 在 Jetson 上会被
        # systemd-tmpfiles(`D /tmp`) 每次开机清空，导致预编译成果撑不过断电。
        self._build_dir = os.path.expanduser(
            f'{build_dir}_{_tag}_v{instance_id}_m{_m}')
        os.makedirs(self._build_dir, exist_ok=True)
        self._instance_id = instance_id

        self.solver = None
        self._setup_ocp()

    def _setup_ocp(self):
        import casadi as ca
        from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

        nx, nu = 6, 3
        x = ca.SX.sym('x', nx)
        u = ca.SX.sym('u', nu)

        M = self.max_neighbours
        p_pos    = ca.SX.sym('p_pos',  3 * M)
        p_active = ca.SX.sym('p_act',      M)
        p_dstar  = ca.SX.sym('p_dstar',    M)
        if self._dob:
            p_dist = ca.SX.sym('p_dist', 3)          # 加性集总扰动 d̂（观测器每拍注入，horizon 内常量）
            p_full = ca.vertcat(p_pos, p_active, p_dstar, p_dist)
        else:
            p_full = ca.vertcat(p_pos, p_active, p_dstar)

        if self.lag_model:
            # 接口一致：u := v_sp（实际下发量），PX4 闭环速度环建模为一阶滞后
            tau = self.vel_lag_tau
            acc = ca.vertcat((u[0]-x[3])/tau, (u[1]-x[4])/tau, (u[2]-x[5])/tau)
            acc_dyn = acc
        else:
            acc = ca.vertcat(u[0], u[1], u[2])       # 现状：u 就是加速度（代价/约束作用于此）
            if self._dob:
                # DOB baseline：动力学 v̇=u+d̂（d̂ 前馈进预测模型）；代价/约束仍作用于
                # 决策变量 u（命令加速度），不含 d̂——d̂ 是已知常量、非决策量。
                acc_dyn = ca.vertcat(u[0]+p_dist[0], u[1]+p_dist[1], u[2]+p_dist[2])
            else:
                acc_dyn = acc
        f_expl = ca.vertcat(x[3], x[4], x[5], acc_dyn[0], acc_dyn[1], acc_dyn[2])

        model = AcadosModel()
        _mtag = ('lag' + f'{self.vel_lag_tau:.3f}'.replace('.', 'p')) if self.lag_model else 'di'
        if self.soc_norm:
            _mtag += 'soc'
        model.name = f'{_mtag}_mpc_i{self._instance_id}_m{M}'
        model.x = x; model.u = u; model.p = p_full
        model.f_expl_expr = f_expl
        xdot = ca.SX.sym('xdot', nx)
        model.xdot = xdot
        model.f_impl_expr = xdot - f_expl

        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N

        coll_residuals = []
        form_residuals = []
        for i in range(M):
            ni = p_pos[3*i : 3*(i+1)]
            diff_xy = x[0:2] - ni[0:2]
            d2 = ca.sumsqr(diff_xy) + 1e-6
            d_i = ca.sqrt(d2)
            active = p_active[i]
            d_star = p_dstar[i]
            coll_residuals.append(
                ca.sqrt(self.w_collision) * active * ca.fmax(0.0, self.d_safe - d_i)
            )
            form_residuals.append(
                ca.sqrt(self.w_formation) * active * (d_i - d_star)
            )

        # 第三段必须是**加速度**而非 u：接口一致模型里 u 是速度设定点，
        # 直接惩罚 u 等于惩罚"飞得快"。用 acc 后 r_acc 的物理含义两模型一致。
        y_track = ca.vertcat(x[0:3], x[3:6], acc)
        y_coll  = ca.vertcat(*coll_residuals) if M > 0 else ca.SX.zeros(0, 1)
        y_form  = ca.vertcat(*form_residuals) if M > 0 else ca.SX.zeros(0, 1)
        y_expr  = ca.vertcat(y_track, y_coll, y_form)
        y_expr_e = ca.vertcat(x[0:3], x[3:6])

        ny = 9 + 2 * M

        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.model.cost_y_expr = y_expr
        ocp.model.cost_y_expr_e = y_expr_e

        w_diag = (
            [self.q_pos]*3 + [self.q_vel]*3 + [self.r_acc]*3 +
            [1.0]*M + [1.0]*M
        )
        ocp.cost.W = np.diag(w_diag)
        ocp.cost.W_e = np.diag(
            [self.q_pos * self.q_pos_terminal_scale]*3 + [self.q_vel]*3
        )
        ocp.cost.yref   = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(6)

        # ── 约束 ────────────────────────────────────────────────────────────────
        # ⚠ acados 语义（两处都踩过坑，勿改）：
        #   · con_h_expr   只作用于 stage 1..N-1
        #   · con_h_expr_0 单独作用于 stage 0 —— 而 u0 恰恰是唯一真正下发的量。
        #     漏掉 con_h_expr_0 则该约束对实际指令完全失效
        #     （实测：u0=3, v0=0, τ=0.5 → 隐含 a=6 m/s²，超 a_max=4 的 50%）。
        #   · stage 0 的 x 被 x0 钉死。把**纯状态**约束放进 con_h_expr_0，
        #     一旦 x0 越界即 QP 不可行 → stage 0 只放含 u 的约束。
        if self.soc_norm:
            # ── §5.G 行动项③：xy 用**范数**约束，与下发端的范数限幅口径统一 ──────
            # 逐轴箱允许 ‖·‖ 达 √2 倍上界（3 → 4.24 m/s），而下发端按范数缩放，
            # 二者不一致：OCP 规划出的解可能一出 solver 就被限幅改写。
            # z 轴是标量，无范数问题，仍用逐轴箱。
            #
            # ⚠⚠ 实现方式：**多面体（线性半空间）**近似，不是二次型 ‖u‖²<=R²。
            # 二次型在 SQP_RTI 下不可用（2026-07-10 实测）：
            #   · ∇(u_x²+u_y²) 在 u=0 处为零 ⇒ 冷启动时线性化成 0<=0<=R²，约束**空的**，
            #     实测 u0 冲到 144 m/s（‖acc0‖=409，a_max=4）。
            #   · 即便 warm start，牛顿步在 u≫R 时只是折半收敛（204→102→51→…），
            #     逼近可行域后还会震荡发散（6.6→40→354）。
            # 线性半空间的梯度恒定非零 ⇒ 一次 QP 精确满足，RTI-safe。
            #
            # K 个方向 d_k = (cos kπ/K, sin kπ/K)，双边界 |d_k·w| <= c ⇒ 2K 面多边形。
            # 取 c = R·cos(π/2K) 使多边形**内接**于半径 R 的圆 ⇒ 严格保证 ‖w‖ <= R
            # （代价：面法向方向的权限损失 1-cos(π/2K)，K=8 时 1.92%）。
            K = self._SOC_FACET_DIRS
            ck = math.cos(math.pi / (2 * K))
            dirs = [(math.cos(math.pi * k / K), math.sin(math.pi * k / K))
                    for k in range(K)]

            def _facets(wx, wy, R):
                """把 ‖(wx,wy)‖ <= R 表成 K 条双边线性约束，返回 (expr_list, bound)。"""
                return [d[0] * wx + d[1] * wy for d in dirs], R * ck

            if self.lag_model:
                # u = v_sp。规划量 = u0 ⇒ 约束必须覆盖 stage 0。
                e_u,   b_u   = _facets(u[0],   u[1],   self.max_speed)
                e_acc, b_acc = _facets(acc[0], acc[1], self.max_accel)
                _h = ca.vertcat(*e_u, *e_acc, acc[2])
                _lh = np.concatenate([np.full(K, -b_u), np.full(K, -b_acc),
                                      [-self.max_accel]])
                _uh = np.concatenate([np.full(K, +b_u), np.full(K, +b_acc),
                                      [+self.max_accel]])
                # 两处都设：con_h_expr 只管 stage 1..N-1，u0 靠 con_h_expr_0
                ocp.model.con_h_expr_0 = _h
                ocp.constraints.lh_0, ocp.constraints.uh_0 = _lh, _uh
                ocp.model.con_h_expr = _h
                ocp.constraints.lh, ocp.constraints.uh = _lh, _uh
                # u_z = v_sp_z：逐轴箱
                ocp.constraints.lbu = np.array([-self.max_climb])
                ocp.constraints.ubu = np.array([+self.max_climb])
                ocp.constraints.idxbu = np.array([2])
                # v_xy 的范数界由 ‖u_xy‖<=v_max 蕴含：一阶滞后下 v[k+1] 是
                # v[k] 与 u[k] 的凸组合 ⇒ ‖v‖ <= max(‖v0‖, v_max)，无需再约束。
            else:
                # u = 加速度。规划量 = x_pred[1] 的速度分量（stage 1 的状态）。
                # stage 0 的 x 被 x0 钉死 ⇒ 纯状态约束不能进 con_h_expr_0
                #（x0 一旦越界即 QP 不可行）。速度范数放 con_h(stage 1..N-1)，
                # 覆盖 stage 1 即覆盖了真正下发的那个量。
                e_a, b_a = _facets(u[0], u[1], self.max_accel)
                e_v, b_v = _facets(x[3], x[4], self.max_speed)
                ocp.model.con_h_expr_0 = ca.vertcat(*e_a)
                ocp.constraints.lh_0 = np.full(K, -b_a)
                ocp.constraints.uh_0 = np.full(K, +b_a)
                ocp.model.con_h_expr = ca.vertcat(*e_a, *e_v)
                ocp.constraints.lh = np.concatenate([np.full(K, -b_a), np.full(K, -b_v)])
                ocp.constraints.uh = np.concatenate([np.full(K, +b_a), np.full(K, +b_v)])
                # u_z = acc_z：逐轴箱
                ocp.constraints.lbu = np.array([-self.max_accel])
                ocp.constraints.ubu = np.array([+self.max_accel])
                ocp.constraints.idxbu = np.array([2])
            # 状态箱只剩 v_z（xy 已由上面的范数约束覆盖）
            ocp.constraints.lbx = np.array([-self.max_climb])
            ocp.constraints.ubx = np.array([+self.max_climb])
            ocp.constraints.idxbx = np.array([5])
        else:
            # ── 现状：逐轴 box（与输出端范数限幅不一致，见上）──────────────────
            if self.lag_model:
                ocp.constraints.lbu = np.array([-self.max_speed, -self.max_speed, -self.max_climb])
                ocp.constraints.ubu = np.array([+self.max_speed, +self.max_speed, +self.max_climb])
                _lh = np.array([-self.max_accel]*3)
                _uh = np.array([+self.max_accel]*3)
                ocp.model.con_h_expr = acc
                ocp.constraints.lh, ocp.constraints.uh = _lh, _uh
                ocp.model.con_h_expr_0 = acc
                ocp.constraints.lh_0, ocp.constraints.uh_0 = _lh, _uh
            else:
                ocp.constraints.lbu = np.array([-self.max_accel]*3)
                ocp.constraints.ubu = np.array([+self.max_accel]*3)
            ocp.constraints.idxbu = np.arange(nu)
            ocp.constraints.lbx = np.array([-self.max_speed, -self.max_speed, -self.max_climb])
            ocp.constraints.ubx = np.array([+self.max_speed, +self.max_speed, +self.max_climb])
            ocp.constraints.idxbx = np.array([3, 4, 5])

        # ── 速度状态箱必须是**软**的（2026-07-10 S9 实测定位）─────────────────────
        # x0 由实测状态钉死，而状态箱作用于 stage 1..N-1：只要实际速度冲出箱外，
        # 一步之内就拉不回来 ⇒ QP 直接不可行(status=4) ⇒ 整拍 MPC 输出被丢弃。
        # 实证：τ=0.5 起飞时 lag 模型的 xy 设定点可瞬跳 ±a_max·τ=2 m/s（legacy 每帧
        # 只能变 a_max·dt=0.2），飞机猛倾→PX4 补推力→回平后 v_z 过冲到 2.64 m/s，
        # 而 max_climb=1.5。stage-1 要拉回需 24 m/s²（a_max=4）⇒ 必然不可行：
        # S9 单机 2 秒内 56 次 fallback。legacy 三次跑峰值 1.22~1.43 只是没踩中。
        # 输入侧约束（u、acc）不软化：它们是决策变量，恒可行。
        _nsbx = len(ocp.constraints.idxbx)
        ocp.constraints.idxsbx = np.arange(_nsbx)
        # 线性惩罚够大 ⇒ 不违反时解与硬约束一致；二次项保证 QP 良态。
        ocp.cost.zl = np.full(_nsbx, 1e4)
        ocp.cost.zu = np.full(_nsbx, 1e4)
        ocp.cost.Zl = np.full(_nsbx, 1e2)
        ocp.cost.Zu = np.full(_nsbx, 1e2)

        ocp.constraints.x0 = np.zeros(nx)
        ocp.parameter_values = np.zeros(p_full.shape[0])

        ocp.solver_options.tf = self.N * self.dt
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.print_level = 0
        ocp.solver_options.qp_solver_iter_max = 100
        ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.levenberg_marquardt = 1e-4
        ocp.solver_options.nlp_solver_max_iter = 30

        ocp.code_export_directory = os.path.join(self._build_dir, 'c_generated_code')
        json_file = os.path.join(self._build_dir, 'acados_ocp.json')
        self.solver = self._build_or_load_solver(ocp, json_file, model.name)
        self._nx, self._nu = nx, nu

    def _config_fingerprint(self):
        """所有影响生成码/求解器配置的量的哈希，用于判断缓存能否安全复用。

        ⚠ 宁可多算不可漏算：漏一项 = 真机带着错误配置起飞。
        两类都必须覆盖——
          · 烧进 C 代码的：vel_lag_tau(动力学)、d_safe/w_collision/w_formation(代价表达式)
          · 只存在 acados_ocp.json 里的：max_speed/max_accel/max_climb(约束边界)、
            q_*/r_acc(权重矩阵)。load-only 时 acados 从旧 json 读这些值，
            漏算会让真机 conservative:=true 的保守限幅被**静默忽略**、用回 SITL 宽松值。
        """
        params = {
            'N': self.N, 'dt': self.dt,
            'max_speed': self.max_speed, 'max_climb': self.max_climb,
            'max_accel': self.max_accel, 'max_neighbours': self.max_neighbours,
            'd_safe': self.d_safe,
            'w_collision': self.w_collision, 'w_formation': self.w_formation,
            'q_pos': self.q_pos, 'q_vel': self.q_vel, 'r_acc': self.r_acc,
            'q_pos_terminal_scale': self.q_pos_terminal_scale,
            'vel_lag_tau': self.vel_lag_tau, 'soc_norm': self.soc_norm,
            'dob': self._dob,
            'soc_facet_dirs': self._SOC_FACET_DIRS,
            'instance_id': self._instance_id,
        }
        h = hashlib.sha256()
        h.update(json.dumps(params, sort_keys=True).encode())
        # 本文件源码：软约束罚项(zl/Zl)等硬编码常量、约束结构改动都不在上面的参数里，
        # 用源码哈希兜住 —— 任何改动都让缓存失效，代价只是改完首次启动慢几秒。
        try:
            with open(os.path.abspath(__file__), 'rb') as f:
                h.update(f.read())
        except OSError:
            h.update(b'src-unavailable')
        # acados 运行库换版本/重编译后，旧生成码不保证 ABI 兼容
        try:
            lib = os.path.join(
                os.environ.get('ACADOS_SOURCE_DIR', os.path.expanduser('~/acados')),
                'lib', 'libacados.so')
            st = os.stat(lib)
            h.update(f'{st.st_size}:{int(st.st_mtime)}'.encode())
        except OSError:
            h.update(b'acados-unavailable')
        return h.hexdigest()

    def _build_or_load_solver(self, ocp, json_file, model_name):
        """指纹命中则直接加载已编译的 .so，否则重新生成+编译。

        实测(Orin NX)：命中 0.02s / 未命中 ~7s。默认每次都 generate+build，
        而生成结果逐字节相同 —— 纯浪费，飞场上电每架机都要多等 7 秒。
        任何异常一律回退到"重新生成"，绝不带病启动。
        """
        from acados_template import AcadosOcpSolver

        fp = self._config_fingerprint()
        fp_file = os.path.join(self._build_dir, 'config_fingerprint.txt')
        so_file = os.path.join(self._build_dir, 'c_generated_code',
                               f'libacados_ocp_solver_{model_name}.so')

        hit = False
        if all(os.path.exists(p) for p in (fp_file, so_file, json_file)):
            try:
                with open(fp_file) as f:
                    hit = f.read().strip() == fp
            except OSError:
                hit = False

        if hit:
            try:
                solver = AcadosOcpSolver(ocp, json_file=json_file,
                                         generate=False, build=False)
                self._cache_state = 'hit'
                print(f'[mpc] acados 缓存命中，直接加载: {self._build_dir}', flush=True)
                return solver
            except Exception as e:      # 缓存损坏/不兼容 → 回退重新生成
                print(f'[mpc] acados 缓存加载失败({type(e).__name__})，重新生成', flush=True)
                hit = False

        self._cache_state = 'hit-failed' if os.path.exists(fp_file) else 'miss'
        print(f'[mpc] acados 重新生成+编译中（{self._cache_state}）: {self._build_dir}',
              flush=True)
        solver = AcadosOcpSolver(ocp, json_file=json_file)
        try:
            with open(fp_file, 'w') as f:
                f.write(fp)
        except OSError:
            pass
        return solver

    def solve(self, x0, x_ref, neighbour_traj=None, desired_distances=None, dist=None):
        N = self.N; M = self.max_neighbours
        # DOB baseline：把观测器估计的加性扰动 d̂ 作为常量参数注入（horizon 内不变）。
        if self._dob:
            self._dist = (np.asarray(dist, dtype=float).reshape(3)
                          if dist is not None else np.zeros(3))
        self.solver.set(0, 'lbx', x0)
        self.solver.set(0, 'ubx', x0)
        for k in range(N):
            yref_k = np.concatenate([
                x_ref[k, 0:3], x_ref[k, 3:6], np.zeros(3),
                np.zeros(M), np.zeros(M),
            ])
            self.solver.set(k, 'yref', yref_k)
            self.solver.set(k, 'p', self._pack_params(k, neighbour_traj, desired_distances))
        self.solver.set(N, 'yref', x_ref[N, 0:6])
        self.solver.set(N, 'p', self._pack_params(N, neighbour_traj, desired_distances))
        status = self.solver.solve()
        u0 = self.solver.get(0, 'u')
        x_pred = np.zeros((N + 1, self._nx))
        for k in range(N + 1):
            x_pred[k, :] = self.solver.get(k, 'x')
        return u0, x_pred, {
            'status': int(status),
            'cost': float(self.solver.get_cost()),
            'solve_time_s': float(self.solver.get_stats('time_tot')),
        }

    def _pack_params(self, k, neighbour_traj, desired_distances):
        M = self.max_neighbours
        p = np.zeros(3 * M + M + M)
        if neighbour_traj is not None:
            nt = np.asarray(neighbour_traj)
            if nt.ndim == 3 and nt.shape[2] == 3:
                M_real = min(nt.shape[0], M)
                K = max(0, nt.shape[1] - 1)
                kc = min(k, K)
                dd = (np.asarray(desired_distances) if desired_distances is not None
                      else np.zeros(M_real))
                for i in range(M_real):
                    p[3*i : 3*(i+1)] = nt[i, kc, :]
                    p[3*M + i]       = 1.0
                    if i < len(dd):
                        p[4*M + i]   = float(dd[i])
        # DOB baseline：模型 p 末尾 3 位是加性扰动 d̂（当拍常量，所有 stage 相同）
        if self._dob:
            p = np.concatenate([p, self._dist])
        return p


# -- state
class DroneState:
    def __init__(self):
        self.received = False
        self.last_stamp = 0.0
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.xy_valid = False   # EKF 水平估计健康（safety_filter 估计门用）
        self.z_valid = False    # EKF 垂直估计健康


# -- node
class MpcControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('num_drones', 9)

        # 默认出生位置（NED: x=北, y=东），与 swarm_launch.py 的 BIRTH_9 一致
        default_births = [
            0.0,  0.0, 0.0,   # 0 中心
            0.0,  3.0, 0.0,   # 1 东
            0.0, -3.0, 0.0,   # 2 西
            3.0,  0.0, 0.0,   # 3 北
           -3.0,  0.0, 0.0,   # 4 南
            3.0,  3.0, 0.0,   # 5 东北
           -3.0,  3.0, 0.0,   # 6 东南
            3.0, -3.0, 0.0,   # 7 西北
           -3.0, -3.0, 0.0,   # 8 西南
        ]
        self.declare_parameter('birth_positions_flat', default_births)
        self.declare_parameter('formation_offsets_flat', default_births)
        self.declare_parameter('neighbours', [0])

        self.declare_parameter('target_alt', -5.0)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_climb', 1.5)
        self.declare_parameter('max_accel', 5.0)
        # PX4 闭环速度环的一阶滞后 τ（s）。>0 启用接口一致 MPC（模型 v̇=(v_sp−v)/τ，
        # 直接下发 u0=v_sp）；<=0 退回现状（v̇=u，下发 x_pred[1]）。
        # 实测 τ=0.48s(无风)/0.61s(2N 风)；旁证 1/MPC_XY_VEL_P_ACC=0.55s。见报告 §5.G。
        # 改动此值会改变 OCP 结构 → 必须 `rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*` 重新生成。
        self.declare_parameter('vel_lag_tau', 0.5)  # 2026-07-14 转正（复归 40/41，§5.H）；<=0 退回 legacy
        # §5.G 行动项③：把 OCP 的**逐轴** box 统一成 xy **范数**约束，与下发端的
        # 范数限幅口径一致（逐轴箱允许 ‖·‖ 达 √2 倍上界，OCP 解可能一出 solver 就被改写）。
        # false = 现状逐轴箱（所有 S1–S34 基线均在此口径下取得）。
        # 改动此值会改变 OCP 结构 → 必须 `rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*` 重新生成。
        self.declare_parameter('soc_norm_enable', False)
        # 论文 §6.5 DOB-MPC 对照 baseline（审稿意见 M2）：true=在 legacy 零滞后模型上
        # 叠加固定时间扰动观测器(FxTDO,[Xu24])估计的加性集总扰动 d̂ 前馈，模型 v̇=u+d̂。
        # 只对 legacy 生效(vel_lag_tau<=0)；改动此值会改变 OCP 结构(p 多 3 维)→ 缓存自动重建。
        # 用于经验验证否定性结果：加性 d̂ 无法复现输入通道滞后的快速动态。
        self.declare_parameter('dob_enable', False)
        # Q-filter 扰动观测器截止带宽 L（rad/s）：d̂ = LPF_L[v̇_meas − u]，**无积分器**
        # （标准 Ohishi Q-filter DOB）。低带宽=准静态，符合 Xu24 Assumption 1。
        self.declare_parameter('dob_bandwidth', 1.0)
        # d̂ 幅值上限（m/s²）。设为 0.5，即 §5.3 实测可估外源扰动(~0.017)的约 25 倍——
        # 远高于真实外源扰动(PX4 速度环积分已抑掉 DC 风扰)，故非"阉割"；同时压住"命令相关
        # 伪扰动"(v̇≈0.1u ⇒ 残差≈−0.9u)造成的 ~9× 正反馈,使 DOB 保守稳定、提供的修正微乎其微。
        # ⚠️ 更大上限(≥2)会让 d̂ 被正反馈顶到饱和、反而破坏稳定跟踪(smoke 实测)——两个方向
        # 都够不到接口一致修复的 0.6m,印证 §3.3:加性项无法表示输入通道滞后。
        self.declare_parameter('dob_dmax', 0.5)
        # 论文 §6.5 分布式一致性编队算法对照（审稿要求）：'mpc'=本文 MPC；'consensus'=标准
        # 分布式 leader-follower 位移一致性律（二阶一致性/consensus 编队,Ren-Beard/Olfati-Saber 式）：
        #   v_cmd_i = v_leader + kL·((p_leader+off_i)−p_i) + kC·Σ_{j∈N_i}((p_j+off_i−off_j)−p_i)
        # 只用本机+邻居信息(分布式)，输出速度设定点(与 MPC 同一 PX4 速度环接口)、过同一安全滤波。
        # 无滞后补偿(名义) ⇒ 用于检验"速度环滞后致失效"是否 MPC 专属、还是一般分布式编队通病。
        self.declare_parameter('controller', 'mpc')
        self.declare_parameter('consensus_kL', 1.5)   # leader/队形位置跟踪增益
        self.declare_parameter('consensus_kC', 0.8)   # 邻居位移一致性增益
        # DOB 仅在编队**成型后**(leader 开始绕圈)才接入：成型阶段 DOB=off，编队与 legacy
        # 一样干净成型、过就绪门；leader 起动(绕圈)进入任务段才启用扰动补偿。避免 DOB 偏置
        # 卡住"pos_err<门限"的就绪判定导致 trial 被判 startup race 弃用。
        # warmup 只是 hover(leader 永不动)场景的**远端兜底**，设得比任何 trial 都长 ⇒ 绕圈
        # 场景永远靠 leader_moving 触发、warmup 不会提前误触发（20s 太短会在成型中途接入）。
        self.declare_parameter('dob_warmup_s', 600.0)
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('neighbour_timeout', 1.0)
        self.declare_parameter('startup_zero_vel_frames', 30)
        # 校准 world_birth 前,要求 EKF 连续 valid 的帧数(等 GPS fix 收敛)
        self.declare_parameter('calib_settle_frames', 25)
        # 校准窗口：只在 local 连续 calib_stable_window 帧"极差 < tol"时才锁定，
        # 不看绝对值(对恒定偏置也成立)，x/y/z 统一生效
        self.declare_parameter('calib_stable_window', 40)     # 稳定性判定窗口(帧)
        self.declare_parameter('calib_stable_tol',    0.05)   # m，窗口三轴极差上限
        self.declare_parameter('calib_timeout_frames', 600)   # 兜底，防永不收敛死锁
        # EKF 收敛后静止于自身 local 原点，|local_xy| 必接近 0；仍几十米=GPS fix 前暂态，拒绝锁定
        self.declare_parameter('calib_max_origin_offset', 2.0)  # m，校准锁定时 |local_xy| 上限
        # 动捕/外部定位：所有机共享一个场地原点，local 本就等于自身出生点（不是 0）
        # ⇒ 判据换成"与期望出生点的偏差"。默认 false = 保持 GPS/SITL 原行为不变。
        # PX4 v1.16 消息版本化：vehicle_status → vehicle_status_v1（见订阅处注释）。
        # 留空=按固件版本自动选；也可显式写 'out/vehicle_status' / 'out/vehicle_status_v1'。
        self.declare_parameter('px4_version', 'main')     # '1.16' | 'main'   # SITL on home PX4 main (harmonic)
        self.declare_parameter('vehicle_status_topic', '')
        self.declare_parameter('calib_shared_origin', False)
        # 共享原点下容差应远小于 calib_max_origin_offset：动捕给的就是真实位置，
        # 偏差是厘米级；2.0m 是给 GPS 收敛暂态留的，在动捕下会放过 trio3 尺度
        # (边长 2.60m)的刚体错接。
        self.declare_parameter('calib_shared_origin_tol', 0.5)  # m
        # XY 全局对齐（RTK 真机编队）：用各机 EKF 原点的 GPS 经纬度相对 drone0 datum 投影，
        # 替代"盲信配置 birth_xy"。各机 EKF local 系彼此独立（原点在各自上电点），本参数
        # 让 RTK cm 精度自动把各机对齐进统一世界系，等价于动捕共享原点。
        # 默认 False = 旧行为（SITL spawn 即 birth、动捕走 shared_origin）不变；真机 RTK 开。
        self.declare_parameter('xy_global_align_enable', False)
        # Tier2: ref_alt 连续温漂(不走 z_reset)→ 持续把 world_birth_z 拉回全员基准
        self.declare_parameter('alt_sync_enable', True)       # 初始用 ref_alt 差校准 world_birth_z(真机各机home海拔不同需要)；SITL 三机同地面应关(ref_alt 差是EKF噪声,补偿会让各机飞到不同物理高度)
        self.declare_parameter('alt_resync_enable', True)
        self.declare_parameter('alt_resync_rate', 0.05)       # m/s，world_birth_z 逼近限速(防 MPC z 跳)
        self.declare_parameter('alt_ref_filter_alpha', 0.05)  # 当前 ref_alt 的 EMA 系数
        self.declare_parameter('alt_resync_max', 3.0)         # m，单机 z 偏移安全上限
        # 悬停期一次性高度 trim：各机 home ref_alt 不同 → 同控 -5 真高散；用 ref_alt 与
        # drone0 datum 之差偏调 target_alt(而非 world_birth_z，避开 alt_resync 撞机根因)，
        # 使各机收敛到同一绝对海拔=同真高；起飞前(leader 起动)冻结，圆周中不再变。
        self.declare_parameter('alt_trim_enable', False)
        self.declare_parameter('alt_trim_max', 2.0)           # m，trim 安全上限

        self.declare_parameter('mpc_horizon', 20)
        self.declare_parameter('mpc_dt', 0.05)
        self.declare_parameter('q_pos', 4.0)
        self.declare_parameter('q_vel', 1.0)
        self.declare_parameter('r_acc', 0.1)
        self.declare_parameter('q_pos_terminal_scale', 2.0)
        self.declare_parameter('d_safe', 1.2)
        self.declare_parameter('w_collision', 200.0)
        self.declare_parameter('w_formation', 0.5)
        # 候选①：XY 位置反馈（前馈 + 有界 P）。kp=0 → 纯前馈(现状/1c4bc5a 行为)；
        # >0 时叠加饱和限幅的位置纠偏，抓住运动时碰撞约束推离槽位的漂移、防发散。
        # cap 限幅避免无阻尼比例环的圆周震荡。非 acados 烤死参数，改后无需清缓存。
        self.declare_parameter('vel_xy_kp', 0.0)
        self.declare_parameter('vel_xy_cap', 0.8)
        self.declare_parameter('acados_build_dir',
                               '~/.cache/mpc_control/acados_di_mpc')
        # P2 故障注入：邻居预测轨迹的通信劣化（S16；0=关闭）。
        # delay 注入后走既有时间对齐路径（latency 平移），dropout 直接丢弃消息——
        # 超过 neighbour_timeout 自动落入"常值外推/队形推断"降级，与真失联同路径。
        self.declare_parameter('comms_delay_ms', 0.0)
        self.declare_parameter('comms_dropout', 0.0)
        # P2 注入: 周期性突发中断（模拟无线遮挡/干扰，0=关闭）。见 _in_comms_blackout。
        self.declare_parameter('comms_blackout_duration_s', 0.0)  # 每次中断时长
        self.declare_parameter('comms_blackout_period_s', 0.0)    # 周期(含中断段)
        self.declare_parameter('comms_blackout_start_s', 0.0)     # 首次中断起始(节点启动后)
        # 中断范围：'all' = 预测+位置+health 全断（链路完全中断）
        #           'pred' = 仅断预测（大包选择性丢失：预测 ~800-1000B 比位置 ~100-200B
        #                    更易丢，是无线劣化的典型表现，且会触发趴窝误判路径）
        self.declare_parameter('comms_blackout_scope', 'all')
        # 真机安全门控：false = 节点不发 ARM/OFFBOARD 指令，只持续发 setpoint 流并
        # 等待飞手用 RC 解锁+切 OFFBOARD（PX4 要求切换前 setpoint 流已 >2Hz，满足）。
        # SITL 无 RC，保持默认 true 自动解锁。
        self.declare_parameter('auto_arm_enable', True)
        # 半自主档（2026-08-30）：「手动解锁 = 授权起飞」。仅在 auto_arm_enable=false 时
        # 有意义——节点**绝不发 ARM**，但检测到飞手已解锁(arming_state=2)后自动切
        # OFFBOARD，随即照常自动爬升到 target_alt。语义上把解锁动作变成起飞授权：
        # 飞手不解锁飞机绝不动；一解锁就会自己升空，飞手须知情并全程握 kill。
        # ⚠️ 地面接入 OFFBOARD 正是 §6.6 的憋劲/过冲场景（0.8s 憋劲→109%、6.6s→340%），
        #    启用前提：takeoff_xy_lock 已过台架验证。默认 false = 现行为完全不变。
        self.declare_parameter('offboard_on_arm', False)

        # 任务完成自动降落（第 1 层，2026-07-29）：订阅 /swarm/mission，收到 'LAND' 后
        # 先零速悬停 land_settle_s 秒稳住（把编队速度刹掉），再命令本机 PX4 切 AUTO.LAND，
        # 随后停 offboard 流交给 PX4 自主降落 + 触地自动上锁。执行完全各机独立，不依赖机间
        # 通信（触发是 leader 单点广播，执行不需协同）；不发 mission 命令则永不触发，行为无变化。
        # ⚠️ 各机原地垂直下降 → 水平间距(d_safe)已在飞行段拉开，降落不收敛=构造上无碰撞。
        self.declare_parameter('land_settle_s', 1.5)   # 收到 LAND 后零速悬停稳住的时长(s)

        # 通信失联自动降落（第 2 层，companion 侧看门狗，2026-07-31）：飞控层只兜得住
        # "companion 停发 setpoint"(COM_OF_LOSS_T→COM_OBL_RC_ACT) 和 "RC 失联"(NAV_RCL_ACT)；
        # 但当 companion 健康、仍在发 setpoint，只是 WiFi 丢了收不到 leader 时，飞控看到的是
        # "offboard 一切正常"、永不失联降落 → 飞机会照旧参考悬停/飞行直到电池耗尽。此看门狗补这个洞：
        # OFFBOARD 飞行中超过 comms_loss_land_s 秒没收到 leader 参考(/leader/state)即触发第 1 层
        # 降落(settle→AUTO.LAND)。solo1 亦适用（solo1 也有 leader 提供轨迹）。
        # 0 = 关闭（SITL 41 场景回归/通信故障注入实验 S25/S42/S44 默认关，行为不变）；
        # 真机由 real_hardware_launch 打开（默认 5s）。
        self.declare_parameter('comms_loss_land_s', 0.0)
        # 失联降落的中间"先冻结"阶段：leader 失联超过 comms_loss_hold_s（但未到 land 阈值）时，
        # **就地悬停**而不是继续朝最后一次的陈旧参考飞（circle/line 下会朝 leader 旧位置漂）。
        # leader 在 land 阈值前恢复则退出冻结、恢复编队。0=关（退回旧行为：整个窗口都追陈旧参考）。
        # 仅在 comms_loss_land_s>0 时生效（是失联降落的前置阶段），须 < comms_loss_land_s。真机默认 1s。
        self.declare_parameter('comms_loss_hold_s', 0.0)

        # companion 安全滤波层（缺省保守；真机经 launch 覆盖，d_emergency 应 < d_safe）
        self.declare_parameter('safety_filter_enable', True)
        self.declare_parameter('safety_max_track_dist', 5.0)   # m，偏离参考点上限=飞散阈值
        self.declare_parameter('safety_max_alt', 8.0)          # m，最大离地绝对高度
        self.declare_parameter('safety_min_alt', 0.3)          # m，最小离地绝对高度
        self.declare_parameter('safety_d_emergency', 1.2)      # m，硬碰撞地板（< d_safe）
        self.declare_parameter('safety_self_timeout', 0.3)     # s，自机 EKF 话题新鲜度门限（独立于邻居超时）
        # 起飞期横向锁定（2026-08-21，修首飞实测的地面积分饱和；见 _update_airborne）
        self.declare_parameter('takeoff_xy_lock_enable', True)
        self.declare_parameter('takeoff_rise_m', 0.5)          # m，相对解锁时高度上升多少算离地
        # 🔴 2026-09-01 台架 ⑤-21 击穿后加的第二条件：解锁怠速下 EKF z 会 ±0.5–0.6 m
        #    随机漂（一轮 24 s 假"离地"、一轮反向永不触发），单靠位移判据挡不住。
        #    真起飞的爬升率 ≥0.3–1 m/s，而怠速漂移速率只有 ~0.02–0.05 m/s ⇒ 用
        #    「爬升速度持续超阈」做第二条件，两条件同时满足才判离地。
        self.declare_parameter('takeoff_climb_ms', 0.2)        # m/s，持续爬升速度阈值
        self.declare_parameter('takeoff_climb_hold_s', 1.0)    # s，爬升须连续保持时长

        self.drone_id   = int(self.get_parameter('drone_id').value)
        self.num_drones = int(self.get_parameter('num_drones').value)

        births = list(self.get_parameter('birth_positions_flat').value)
        if len(births) != 3 * self.num_drones:
            raise RuntimeError(
                f'birth_positions_flat must have {3*self.num_drones} elements')
        self.birth_positions = np.array(births, dtype=float).reshape(self.num_drones, 3)
        # DYNAMIC version — mutated on each EKF reset
        self.world_birth = self.birth_positions.copy()

        offsets = list(self.get_parameter('formation_offsets_flat').value)
        if len(offsets) != 3 * self.num_drones:
            raise RuntimeError(
                f'formation_offsets_flat must have {3*self.num_drones} elements')
        self.formation_offsets = np.array(offsets, dtype=float).reshape(self.num_drones, 3)
        self.my_offset = self.formation_offsets[self.drone_id]

        neighbours_raw = list(self.get_parameter('neighbours').value)
        self.neighbours = sorted(set(
            int(j) for j in neighbours_raw
            if 0 <= int(j) < self.num_drones and int(j) != self.drone_id
        ))

        self.desired_distances = np.array([
            float(np.linalg.norm(
                self.my_offset[:2] - self.formation_offsets[j][:2]
            ))
            for j in self.neighbours
        ])

        self.target_alt = float(self.get_parameter('target_alt').value)
        self.max_speed  = float(self.get_parameter('max_speed').value)
        self.max_climb  = float(self.get_parameter('max_climb').value)
        self.vel_xy_kp  = float(self.get_parameter('vel_xy_kp').value)
        self.vel_xy_cap = float(self.get_parameter('vel_xy_cap').value)
        self.max_accel  = float(self.get_parameter('max_accel').value)
        self.vel_lag_tau = float(self.get_parameter('vel_lag_tau').value)
        self.soc_norm_enable = bool(self.get_parameter('soc_norm_enable').value)
        self.dob_enable = bool(self.get_parameter('dob_enable').value)
        self._dob_L     = float(self.get_parameter('dob_bandwidth').value)
        self._dob_warmup_s = float(self.get_parameter('dob_warmup_s').value)
        self._dob_vprev = None         # 上一拍实测速度（Q-filter 用有限差分算 v̇）
        self._dob_dhat  = np.zeros(3)  # 观测器扰动估计 d̂（LPF 输出）
        self._last_u0   = np.zeros(3)  # 上一拍 MPC 命令加速度（观测器的名义输入 u_nom）
        self._dob_dmax  = float(self.get_parameter('dob_dmax').value)   # d̂ 幅值上限
        self._dob_engaged = False      # 接入门闩：编队成型(leader 起动)或 warmup 后置 True
        self._dob_first_ctrl_t = None  # 首次进入 solve 段的时刻（warmup 兜底计时起点）
        self._last_dhat = np.zeros(3)  # 最近一拍 d̂（供 health/CSV 记录，dob 关时恒 0）
        self.controller  = str(self.get_parameter('controller').value).strip().lower()
        self.consensus_kL = float(self.get_parameter('consensus_kL').value)
        self.consensus_kC = float(self.get_parameter('consensus_kC').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.neighbour_timeout = float(self.get_parameter('neighbour_timeout').value)
        self.startup_zero_vel_frames = int(self.get_parameter('startup_zero_vel_frames').value)
        self.calib_settle_frames = max(1, int(self.get_parameter('calib_settle_frames').value))
        self.calib_stable_window = max(2, int(self.get_parameter('calib_stable_window').value))
        self.calib_stable_tol    = float(self.get_parameter('calib_stable_tol').value)
        self.calib_timeout_frames = max(self.calib_settle_frames,
                                        int(self.get_parameter('calib_timeout_frames').value))
        self.calib_max_origin_offset = float(self.get_parameter('calib_max_origin_offset').value)
        # vehicle_status 话题名：显式指定优先，否则按 px4_version 推导
        _vst = str(self.get_parameter('vehicle_status_topic').value).strip()
        if not _vst:
            _pxv = str(self.get_parameter('px4_version').value).strip()
            if _pxv.startswith('main'):
                _vst = 'out/vehicle_status_v4'
            elif _pxv.startswith('1.16'):
                _vst = 'out/vehicle_status_v1'
            else:
                _vst = 'out/vehicle_status'
        self.vehicle_status_topic = _vst
        # local_position 话题名同样随固件版本化：PX4 main 发 `_v1` 后缀；
        # v1.16 / v1.14 发无后缀版 `vehicle_local_position`。
        if _pxv.startswith('main'):
            self.local_position_topic = 'out/vehicle_local_position_v1'
        else:
            self.local_position_topic = 'out/vehicle_local_position'
        self.calib_shared_origin = bool(self.get_parameter('calib_shared_origin').value)
        self.calib_shared_origin_tol = float(
            self.get_parameter('calib_shared_origin_tol').value)

        N        = int(self.get_parameter('mpc_horizon').value)
        mpc_dt   = float(self.get_parameter('mpc_dt').value)
        q_pos    = float(self.get_parameter('q_pos').value)
        q_vel    = float(self.get_parameter('q_vel').value)
        r_acc    = float(self.get_parameter('r_acc').value)
        q_term_s = float(self.get_parameter('q_pos_terminal_scale').value)
        d_safe   = float(self.get_parameter('d_safe').value)
        w_coll   = float(self.get_parameter('w_collision').value)
        w_form   = float(self.get_parameter('w_formation').value)
        build_dir = str(self.get_parameter('acados_build_dir').value)
        self.comms_delay_s = max(0.0, float(self.get_parameter('comms_delay_ms').value)) * 1e-3
        self.comms_dropout = min(1.0, max(0.0, float(self.get_parameter('comms_dropout').value)))
        self.auto_arm_enable = bool(self.get_parameter('auto_arm_enable').value)
        self.offboard_on_arm = bool(self.get_parameter('offboard_on_arm').value)
        if self.auto_arm_enable and self.offboard_on_arm:
            # 全自主档本就包含切 OFFBOARD，半自主开关在此无意义；置回 False 防歧义。
            self.offboard_on_arm = False
        if not self.auto_arm_enable:
            if self.offboard_on_arm:
                self.get_logger().warn(
                    f'drone {self.drone_id}: 半自主档 — 等待飞手 RC 解锁；'
                    f'解锁即自动切 OFFBOARD 并起飞（解锁=授权起飞，飞手须握 kill）')
            else:
                self.get_logger().warn(
                    f'drone {self.drone_id}: auto_arm DISABLED — 等待飞手 RC 解锁并切 OFFBOARD')
        self.comms_blackout_dur    = max(0.0, float(self.get_parameter('comms_blackout_duration_s').value))
        self.comms_blackout_period = max(0.0, float(self.get_parameter('comms_blackout_period_s').value))
        self.comms_blackout_start  = max(0.0, float(self.get_parameter('comms_blackout_start_s').value))
        self._node_t0 = self.get_clock().now().nanoseconds * 1e-9
        self._blackout_was_active = False
        self._blackout_scope = str(self.get_parameter('comms_blackout_scope').value).lower()
        self.peer_health_stamps = {}     # peer_id -> 最近 health 到达时刻
        self._peer_degrade_lvl = {}      # peer_id -> 当前降级级别(1/2/3)
        self._peer_degrade_frames = {}   # level -> 累计帧数
        self._degrade_last_report = 0.0  # 上次汇总时刻（周期打印，见 _log_degrade_summary）
        if self.comms_delay_s > 0.0 or self.comms_dropout > 0.0:
            self.get_logger().warn(
                f'[P2 FAULT INJECTION] comms_delay={self.comms_delay_s*1e3:.0f}ms '
                f'dropout={self.comms_dropout:.0%} — 仅用于降级测试，真机部署必须为 0')
        if self.comms_blackout_dur > 0.0 and self.comms_blackout_period > 0.0:
            self.get_logger().warn(
                f'[P2 FAULT INJECTION] comms_blackout={self.comms_blackout_dur:.1f}s '
                f'每 {self.comms_blackout_period:.1f}s 一次，'
                f'始于 t+{self.comms_blackout_start:.0f}s — 真机部署必须为 0')

        # State
        self.drone_states = [DroneState() for _ in range(self.num_drones)]
        self.leader_received = False
        self._last_leader_rx = None   # 最近一次收到 /leader/state 的时刻(s)；None=从未收到
        self.leader_pos = np.zeros(3)
        self.leader_vel = np.zeros(3)
        self.leader_acc = np.zeros(3)   # 向心加速度（圆周运动）
        self.leader_yaw = 0.0
        self.attitude_yaw = 0.0
        self.attitude_received = False
        self.last_control_time = self.get_clock().now()
        self._startup_counter = 0
        self.peer_predictions = {}
        self.peer_prediction_stamps = {}   # drone_id -> float (seconds)
        self._pred_delay_buf = {}          # P2 注入延迟: drone_id -> [(arrival_t, traj), ...]
        self._dbg_counter = 0

        self.last_valid_yaw = 0.0

        # 健康诊断计数器
        self._fallback_count = 0           # 累计 hover 降级次数
        self._hover_active = False         # 当前帧是否在 hover 降级
        self._last_mpc_status = 0
        self._last_solve_ms = 0.0
        self._last_pos_err = 0.0
        self._last_z_err   = 0.0

        # §5.G 行动项③ 取证：OCP 的输入/速度箱是**逐轴** |·|≤max_speed，而下发前是
        # **范数**限幅 → 规划的可达范数上界是 max_speed·√2(≈1.414)，执行的是 max_speed。
        # 先量再修：若比值恒 ≤1 则该不一致从不绑定，属纸面问题。
        self._plan_ratio_max = 0.0   # max‖v_plan_xy‖/max_speed（P 纠偏**之前**，干净）
        self._clip_count     = 0     # 最终下发前范数限幅实际触发的帧数

        self._ocp_ready = False  # acados 编译完成前不尝试 ARM

        # EKF reset trackers (one per known drone)
        self._prev_xy_reset = [0] * self.num_drones
        self._prev_z_reset  = [0] * self.num_drones
        self._pos_calibrated = [False] * self.num_drones
        # 共享原点门控的闩锁：一旦检出偏差就永久拒绝该机校准（详见校准处注释）
        self._calib_gate_latched = [False] * self.num_drones
        self._calib_latch_desc = [''] * self.num_drones   # 闩锁瞬间的判据描述
        self._ref_alt = [None] * self.num_drones   # 各机 EKF 参考海拔 (ref_alt from local_pos)
        # Tier2: 当前(滤波)ref_alt，每帧更新；与上面"标定时冻结的 _ref_alt"区分
        self._ref_alt_now   = [None] * self.num_drones
        self._datum_ref_alt = None   # drone0 广播的当前基准 ref_alt
        # XY 全局对齐：各机 EKF 原点经纬度(标定时冻结) + drone0 广播的公共原点(lat0,lon0)
        self._ref_lat = [None] * self.num_drones
        self._ref_lon = [None] * self.num_drones
        self._datum_ref_latlon = None   # drone0 广播的公共原点 (ref_lat0, ref_lon0)
        self._xy_aligned = [False] * self.num_drones  # 该机 world_birth XY 是否已全局对齐
        self._xy_global_align_enable = bool(
            self.get_parameter('xy_global_align_enable').value)
        self._alt_sync_enable = bool(self.get_parameter('alt_sync_enable').value)
        self._alt_resync_enable = bool(self.get_parameter('alt_resync_enable').value)
        self._alt_resync_rate   = float(self.get_parameter('alt_resync_rate').value)
        self._alt_ref_alpha     = float(self.get_parameter('alt_ref_filter_alpha').value)
        self._alt_resync_max    = float(self.get_parameter('alt_resync_max').value)
        # 悬停期高度 trim 状态
        self._alt_trim_enable = bool(self.get_parameter('alt_trim_enable').value)
        self._alt_trim_max    = float(self.get_parameter('alt_trim_max').value)
        self._alt_trim        = 0.0     # m，加到 target_alt 的世界系 z 偏调（NED）
        self._alt_trim_frozen = False   # leader 起动后冻结，圆周中不再变
        # 连续 EKF-valid 帧计数,用于 world_birth 校准的收敛门控
        self._valid_streak = [0] * self.num_drones
        # 校准滚动窗口：最近若干帧的 local (x,y,z)，用于稳定性门控
        self._calib_win = [[] for _ in range(self.num_drones)]

        # MPC
        self.N = N
        self.mpc_dt = mpc_dt
        self.get_logger().info(
            f'building acados OCP for drone {self.drone_id} '
            f'(N={N}, dt={mpc_dt}, neighbours={self.neighbours}, '
            f'd_star={self.desired_distances.tolist()})...'
        )
        self.mpc = DoubleIntegratorMPC(
            N=N, dt=mpc_dt,
            max_speed=self.max_speed,
            max_climb=self.max_climb,
            max_accel=self.max_accel,
            max_neighbours=max(1, len(self.neighbours)),
            d_safe=d_safe,
            w_collision=w_coll,
            w_formation=w_form,
            q_pos=q_pos, q_vel=q_vel, r_acc=r_acc,
            q_pos_terminal_scale=q_term_s,
            vel_lag_tau=self.vel_lag_tau,
            soc_norm_enable=self.soc_norm_enable,
            dob_enable=self.dob_enable,
            build_dir=build_dir,
            instance_id=self.drone_id,
        )
        _mm = (f'interface-consistent (v_dot=(v_sp-v)/tau, tau={self.vel_lag_tau:.3f}s, '
               f'publish u0)' if self.mpc.lag_model
               else 'double-integrator (v_dot=u, publish x_pred[1]) [legacy]')
        _cc = ('SOC xy-norm (‖·‖<=bound, 与下发端限幅同口径)' if self.mpc.soc_norm
               else 'per-axis box (允许 ‖·‖ 达 √2 倍上界) [legacy]')
        self.get_logger().info(f'acados OCP ready. model = {_mm} | constraints = {_cc}')
        self._ocp_ready = True

        # companion 安全滤波层（独立于 MPC，下发前过一道硬保护；不动 OCP→不清缓存）
        self._relinquished = False
        self._relinquish_next_cmd = 0.0  # Hold 未确认前重发 DO_SET_MODE 的节拍（1s）
        # 任务完成降落状态机（第 1 层）：requested=收到 LAND；settle_start=开始零速悬停时刻；
        # commanded=已切 AUTO.LAND 进入停流终态；next_cmd=Land 未确认前重发 DO_SET_MODE 的节拍。
        self.land_settle_s = max(0.0, float(self.get_parameter('land_settle_s').value))
        self.comms_loss_land_s = max(0.0, float(self.get_parameter('comms_loss_land_s').value))
        self.comms_loss_hold_s = max(0.0, float(self.get_parameter('comms_loss_hold_s').value))
        self._comms_loss_landed = False   # 只触发一次
        self._comms_hold_active = False   # 就地冻结阶段的边沿日志标志
        # hold 必须严格小于 land，否则冻结阶段永远进不去（land 先触发）→ 视为误配，禁用冻结
        if self.comms_loss_hold_s > 0.0 and self.comms_loss_hold_s >= self.comms_loss_land_s:
            self.get_logger().warn(
                f'[d{self.drone_id}] comms_loss_hold_s({self.comms_loss_hold_s:.1f}) '
                f'>= comms_loss_land_s({self.comms_loss_land_s:.1f}) — 冻结阶段禁用（须 hold<land）')
            self.comms_loss_hold_s = 0.0
        if self.comms_loss_land_s > 0.0:
            _hold_txt = (f'；leader 停 {self.comms_loss_hold_s:.1f}s 先就地冻结'
                         if self.comms_loss_hold_s > 0.0 else '')
            self.get_logger().info(
                f'[d{self.drone_id}] 第2层失联降落已武装：OFFBOARD 中 leader 停 '
                f'{self.comms_loss_land_s:.1f}s → 自动降落{_hold_txt}')
        self._landing_requested = False
        self._land_settle_start = None
        self._land_commanded = False
        self._land_next_cmd = 0.0
        self._safety_self_timeout = 0.3   # 默认值；safety ON 时由参数覆盖
        if bool(self.get_parameter('safety_filter_enable').value):
            s_track = float(self.get_parameter('safety_max_track_dist').value)
            s_maxa  = float(self.get_parameter('safety_max_alt').value)
            s_mina  = float(self.get_parameter('safety_min_alt').value)
            s_demg  = float(self.get_parameter('safety_d_emergency').value)
            self._safety_self_timeout = float(self.get_parameter('safety_self_timeout').value)
            s_dwarn = max(s_demg + 0.5, d_safe + 0.5)
            self.safety = SafetyFilter(
                max_track_dist=s_track, max_alt=s_maxa, min_alt=s_mina,
                d_emergency=s_demg, d_warn=s_dwarn,
                max_speed=self.max_speed, max_climb=self.max_climb,
                max_accel=self.max_accel, drone_id=self.drone_id)
            self.get_logger().info(
                f'safety_filter ON: track<{s_track}m alt[{s_mina},{s_maxa}]m '
                f'd_emerg={s_demg}m d_warn={s_dwarn}m self_timeout={self._safety_self_timeout}s')
        else:
            self.safety = None
            self.get_logger().warn('safety_filter OFF（仅调试用，真机务必开）')

        # ROS 2 IO
        qos     = make_px4_qos()      # 订阅 PX4 "out" 话题（TRANSIENT_LOCAL）
        pub_qos = make_px4_pub_qos()  # 发布到 PX4 "in" 话题（VOLATILE，必须匹配 PX4 DataReader）
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode,
            topic_for_drone(self.drone_id, 'in/offboard_control_mode'), pub_qos,
        )
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint,
            topic_for_drone(self.drone_id, 'in/trajectory_setpoint'), pub_qos,
        )
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand,
            topic_for_drone(self.drone_id, 'in/vehicle_command'), pub_qos,
        )
        self._arming_state = 0   # 0=unknown,1=disarmed,2=armed
        self._nav_state   = 0
        self._prev_arming_state = 0   # 边沿检测：disarmed→armed = 新架次，安全闩锁在此清零
        # ── 起飞期横向锁定（2026-08-21）────────────────────────────────────────
        # 🔴 首飞 bag 实测：接管 OFFBOARD 时飞机**还在地上**且离 world_birth 目标 0.964m
        #    ⇒ 节点立刻下发满幅 |vxy|=1.000；被地面顶住整整 1 秒（实测速度 −0.01m/s），
        #    PX4 速度环积分累积 ⇒ 离地瞬间实测峰值 **1.370 m/s = 指令上限的 137%**
        #    （一阶滞后不可能超调，只能是积分饱和），y 过冲 −0.403m，8.5s 才收进 10cm。
        #    单机无害，但 pair2_out 只有 4m 间距 ⇒ 双机前必修。
        # ✅ 离地前把 XY 速度指令置零（垂直不动，照常爬升），离地后解锁。
        # 🔑 判据用**相对解锁时刻的上升量**，不用绝对高度：首飞实测气压计漂移把"地面"
        #    推到 z=+0.85m，绝对高度阈值会被基准漂移带偏；相对量把漂移直接减掉。
        # 🔑 基准在**解锁边沿之后的第一帧位置回调**里捕获——解锁必然发生在地面，那一刻的
        #    z 就是可信地面。⚠️ 必须是位置回调、不能是控制回路：控制回路只在进 OFFBOARD
        #    之后才跑，而人在环流程是"手动起飞到 1–2m 再切 OFFBOARD"，在那里捕获会把
        #    空中高度当成地面 ⇒ 闩锁永不解除、XY 全程锁零（2026-08-21 两个架次实测）。
        # 🔑 另有 _check_takeoff_lock_watchdog 兜底：锁超过 _takeoff_lock_max_s 强制解除。
        # ⚠️ 没见过解锁边沿（节点在飞行中重启）⇒ 默认"已离地"、不锁 = **故意 fail-open**：
        #    在飞行中锁死 XY 比不修这个 bug 危险得多。
        self._takeoff_lock_enable = bool(self.get_parameter('takeoff_xy_lock_enable').value)
        self._takeoff_rise = max(0.0, float(self.get_parameter('takeoff_rise_m').value))
        self._takeoff_climb_ms = max(0.0, float(self.get_parameter('takeoff_climb_ms').value))
        self._takeoff_climb_hold_s = max(0.0, float(self.get_parameter('takeoff_climb_hold_s').value))
        self._airborne = True         # 初值 True = 未见解锁边沿则不锁（fail-open）
        self._ground_z = None         # 解锁瞬间的 z（NED，向下为正）
        self._climb_since = None      # 爬升速度连续超阈的起始时刻（秒）；None=当前没在爬
        self._takeoff_lock_logged = False
        self._arm_time = None         # 解锁边沿的时刻（秒），供起飞锁看门狗计时
        # 起飞期横向锁定的兜底时限（秒，从解锁边沿算起）：锁生效期间飞机完全没有横向
        # 纠正能力，判据一旦失灵表现就是随风漂走——比不锁危险得多，所以给硬时限兜底。
        # 刻意不做成 ROS 参数：本仓库反复出现"节点声明了参数但 launch 不透传"，而这个值
        # 不需要按场景调（正常起飞几秒内就升过 takeoff_rise）。
        # 2026-09-01 从 20 提到 45：离地判据升级为「位移+持续爬升」双条件后不再会被怠速
        # 漂移假触发，看门狗只兜"传感器/位置流失灵"这一种情况；而外场解锁后怠速检查
        # 超过 20 s 是常态，20 s 会让保护恰好在最需要的窗口（长怠速+地面切 OFFBOARD）
        # 提前失效。45 s 仍然有界，fail-open 语义不变。
        self._takeoff_lock_max_s = 45.0
        self._arm_offboard_confirmed = False
        self._offboard_engaged_once = False   # 半自主档：本架次是否已自动进过一次 OFFBOARD
        self._offboard_req_count = 0          # 半自主档：本架次自动切模式的请求计数
        self._cmd_retry_counter = 0   # frames since startup finished
        # 控制定时器与所有订阅分属不同回调组 + MultiThreadedExecutor：
        # 否则单线程 executor 下 50Hz 控制定时器会被 9 邻居+leader+位置订阅的回调洪流
        # 偶发拖过 0.5s → offboard setpoint 断流 → PX4 报 offboard_control_signal_lost → Hold。
        # 共享态(ds.pos/vel、leader_pos、peer_predictions)均为原子 rebind，跨线程读最多取到旧一帧，安全。
        self._cb_control = MutuallyExclusiveCallbackGroup()
        self._cb_subs = MutuallyExclusiveCallbackGroup()
        # ⚠️ PX4 v1.16 起启用**消息版本化**：话题改叫 `vehicle_status_v1`
        # （消息类型仍是 px4_msgs/msg/VehicleStatus，只有话题名带后缀）。
        # v1.14 上不存在 _v1，v1.16 上不存在无后缀版 —— 名字错了不会报错，
        # 只是**永远收不到**，表现为一直 "retry ARM+OFFBOARD" 卡死。
        # 2026-07-23 在 X6 Air+(v1.16) 实测确认：`ros2 topic list` 里只有 _v1。
        # 仿真机仍是 v1.14（论文基线冻结），故做成参数而非写死。
        self.create_subscription(
            VehicleStatus,
            topic_for_drone(self.drone_id, self.vehicle_status_topic),
            self._on_vehicle_status, qos, callback_group=self._cb_subs,
        )
        self.create_subscription(
            VehicleAttitude,
            topic_for_drone(self.drone_id, 'out/vehicle_attitude'),
            self.on_self_attitude, qos, callback_group=self._cb_subs,
        )
        self.create_subscription(
            VehicleLocalPosition,
            topic_for_drone(self.drone_id, self.local_position_topic),
            self._make_pos_callback(self.drone_id), qos, callback_group=self._cb_subs,
        )
        for j in self.neighbours:
            self.create_subscription(
                VehicleLocalPosition,
                topic_for_drone(j, self.local_position_topic),
                self._make_pos_callback(j), qos, callback_group=self._cb_subs,
            )
        leader_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Float64MultiArray, '/leader/state', self.on_leader_state, leader_qos,
            callback_group=self._cb_subs,
        )
        # Tier2: 全员高度基准 —— drone0 广播当前 ref_alt，各机据此 re-sync world_birth_z
        datum_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_alt_datum = (
            self.create_publisher(Float64, '/swarm/alt_datum', datum_qos)
            if self.drone_id == 0 else None
        )
        self.create_subscription(
            Float64, '/swarm/alt_datum', self._on_alt_datum, datum_qos,
            callback_group=self._cb_subs,
        )
        # XY 全局对齐 datum：drone0 广播其 EKF 原点经纬度 [ref_lat0, ref_lon0]，
        # 各机据此把自身 EKF 原点投影进公共世界系（latched，晚加入/掉线也能收到）。
        self.pub_xy_datum = (
            self.create_publisher(Float64MultiArray, '/swarm/xy_datum', datum_qos)
            if self.drone_id == 0 else None
        )
        self.create_subscription(
            Float64MultiArray, '/swarm/xy_datum', self._on_xy_datum, datum_qos,
            callback_group=self._cb_subs,
        )
        # 任务完成降落指令（第 1 层）：leader 单点广播 'LAND'。用 latched(TRANSIENT_LOCAL)
        # +RELIABLE，短暂掉线/晚加入的机也能收到最后一条指令；执行仍各机独立。
        self.create_subscription(
            String, '/swarm/mission', self._on_mission_cmd, datum_qos,
            callback_group=self._cb_subs,
        )
        # 机间广播：BEST_EFFORT（见 make_swarm_qos —— 发布/订阅两端必须一致）
        _swarm_qos = make_swarm_qos()
        self.pub_predicted = self.create_publisher(
            Float64MultiArray,
            mpc_topic_for_drone(self.drone_id, 'predicted_trajectory'), _swarm_qos,
        )
        # 健康诊断话题（供 diag_monitor.py 订阅）
        # 格式: [drone_id, mpc_status, solve_ms, fallback_count, hover_active, pos_err_m]
        # 保持默认 RELIABLE：消息小、且丢包会在 CSV 里留下数据空洞（论文用）。
        self.pub_health = self.create_publisher(
            Float32MultiArray,
            mpc_topic_for_drone(self.drone_id, 'health'), 10,
        )
        for j in self.neighbours:
            self.create_subscription(
                Float64MultiArray,
                mpc_topic_for_drone(j, 'predicted_trajectory'),
                self._make_pred_callback(j), _swarm_qos,
                callback_group=self._cb_subs,
            )
            # 邻居 health：作为"该机 mpc 是否存活"的**独立**判据。
            # 不能只用"预测过期+位置新鲜"判趴窝 —— 那分不清"mpc 崩了"和"预测包丢了"，
            # 而预测包(~800-1000B)比位置包(~100-200B)在丢包链路上更容易丢，真机上
            # 极易误判（2026-07-22 SITL 实测：只断预测时误触发 81 次）。
            # health 仅 9 个 float(~36B) 且走 RELIABLE，可靠性高得多。
            self.create_subscription(
                Float32MultiArray,
                mpc_topic_for_drone(j, 'health'),
                self._make_health_callback(j), 10,
                callback_group=self._cb_subs,
            )
        # 控制定时器单独成组 → 与订阅并行、永不被回调洪流饿死
        self.timer = self.create_timer(
            1.0 / self.control_hz, self.control_loop,
            callback_group=self._cb_control,
        )

        self.get_logger().info(
            f'mpc_controller drone {self.drone_id} ready. '
            f'birth={self.birth_positions[self.drone_id]}, '
            f'r_i0={self.my_offset}, '
            f'neighbours={self.neighbours}'
        )

    # =================================================================
    # callbacks
    # =================================================================
    def _make_pos_callback(self, drone_idx):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            # P2 注入: 突发中断 —— **邻居**位置在真机上同样跨机走无线
            # (/px4_j/fmu/out/vehicle_local_position 由 j 机自己的 Agent 发布)，
            # 链路断时与 predicted_trajectory 一起断。只屏蔽预测而留着位置，
            # 会让降级停在 2 级(常值外推)、永远测不到 3 级(队形推断)。
            # 自机位置是本地 FC 直连、不走无线，**不屏蔽**。
            # scope='pred' 时只断预测（大包选择性丢失），位置照常可达。
            if (drone_idx != self.drone_id and self._blackout_scope == 'all'
                    and self._in_comms_blackout(now)):
                return
            ds = self.drone_states[drone_idx]

            # Tier2: 持续跟踪当前(滤波)ref_alt（标定后 ref_alt 仍会温漂）
            if math.isfinite(msg.ref_alt) and msg.ref_alt > 0.0:
                if self._ref_alt_now[drone_idx] is None:
                    self._ref_alt_now[drone_idx] = float(msg.ref_alt)
                else:
                    a = self._alt_ref_alpha
                    self._ref_alt_now[drone_idx] = (
                        (1.0 - a) * self._ref_alt_now[drone_idx] + a * float(msg.ref_alt))

            # --- detect EKF reset; update dynamic birth offset ---
            # Skip until first position calibrated world_birth — stale resets
            # from before MPC startup would corrupt the offset.
            _EKF_RESET_CAP = 5.0  # m — ignore implausibly large SITL startup glitches
            if self._pos_calibrated[drone_idx]:
                if msg.xy_reset_counter > self._prev_xy_reset[drone_idx]:
                    dx, dy = float(msg.delta_xy[0]), float(msg.delta_xy[1])
                    reset_mag = math.hypot(dx, dy)
                    if reset_mag > _EKF_RESET_CAP:
                        self.get_logger().error(
                            f'[veh {drone_idx}] xy reset #{msg.xy_reset_counter} '
                            f'IGNORED (mag={reset_mag:.1f}m > cap={_EKF_RESET_CAP}m); '
                            f'world_birth unchanged'
                        )
                    else:
                        self.world_birth[drone_idx, 0] -= dx
                        self.world_birth[drone_idx, 1] -= dy
                        self.get_logger().warn(
                            f'[veh {drone_idx}] xy reset #{msg.xy_reset_counter}, '
                            f'delta=({dx:+.2f}, {dy:+.2f}); '
                            f'world_birth -> ({self.world_birth[drone_idx,0]:+.2f}, '
                            f'{self.world_birth[drone_idx,1]:+.2f})'
                        )
                    self._prev_xy_reset[drone_idx] = msg.xy_reset_counter
                # Z reset：补偿 world_birth_z。
                # z_reset 改变 local_z 与 GPS 海拔的对应关系。由于 world_birth_z
                # 已用 GPS 海拔差校准，z_reset 后必须同步补偿，否则 MPC 世界坐标系
                # 中 ds.pos 会跳变，导致各机高度不一致。
                if msg.z_reset_counter > self._prev_z_reset[drone_idx]:
                    # 起飞期(离地<1m)的 z_reset 多为 EKF 沉降/气压计settle，把跳变补进
                    # world_birth_z 会将一次性偏移永久烤成真高误差(d5 起飞期 reset#3
                    # delta=-0.76 → 真高永久偏 +0.76m)。与 alt_resync 离地门控(:843)同
                    # 思路：近地 reset 只推进计数器、不补偿；飞行中 reset 仍补偿以保世界系
                    # 连续、不扰编队几何。msg 为 drone_idx 本机 local_position，-z=离地高度。
                    alt_agl = -float(msg.z)
                    if alt_agl < 1.0:
                        self.get_logger().warn(
                            f'[veh {drone_idx}] z reset #{msg.z_reset_counter}, '
                            f'delta={msg.delta_z:+.2f}; 离地{alt_agl:.2f}m<1m(起飞期)'
                            f'→ 不补偿 world_birth_z(保持 {self.world_birth[drone_idx,2]:+.2f})'
                        )
                    else:
                        self.world_birth[drone_idx, 2] -= float(msg.delta_z)
                        self.get_logger().warn(
                            f'[veh {drone_idx}] z reset #{msg.z_reset_counter}, '
                            f'delta={msg.delta_z:+.2f}; '
                            f'world_birth_z → {self.world_birth[drone_idx,2]:+.2f}'
                        )
                    self._prev_z_reset[drone_idx] = msg.z_reset_counter

            # --- one-time world_birth calibration, GATED on EKF convergence ---
            # 上电瞬间 PX4 就会发 vehicle_local_position,但此时 EKF 还没 GPS fix,
            # x/y 是几十米级瞬态值。若直接拿首帧校准,会把垃圾烤进 world_birth,
            # 造成机间坐标系不一致(悬停被各机"守出生点"掩盖,line 模式才暴露)。
            # 因此必须等 xy_valid/z_valid 且连续稳定 calib_settle_frames 帧再锁定。
            if not self._pos_calibrated[drone_idx]:
                pos_ok = (msg.xy_valid and msg.z_valid
                          and math.isfinite(msg.x)
                          and math.isfinite(msg.y)
                          and math.isfinite(msg.z))
                if not pos_ok:
                    self._valid_streak[drone_idx] = 0
                    self._calib_win[drone_idx].clear()
                    return
                self._valid_streak[drone_idx] += 1

                # 滚动窗口：不数绝对值，只看 local 是否已"停止漂移"
                win = self._calib_win[drone_idx]
                win.append((msg.x, msg.y, msg.z))
                if len(win) > self.calib_stable_window:
                    win.pop(0)

                arr = np.array(win)
                spread = (arr.max(0) - arr.min(0)) if len(win) >= 2 else np.full(3, np.inf)
                # 用收敛后的窗口均值当基准(抹平单帧噪声),而非抖动单帧
                first_local = arr.mean(0)
                stable = (self._valid_streak[drone_idx] >= self.calib_settle_frames
                          and len(win) >= self.calib_stable_window
                          and float(spread.max()) < self.calib_stable_tol)
                timed_out = self._valid_streak[drone_idx] >= self.calib_timeout_frames

                # 绝对门控：拒绝把可疑的 local 烤进 world_birth(否则机间世界系不一致
                # →编队塌缩)。timeout 也不放行，只升级为告警；未校准时各机保持出生点
                # 悬停(天然间距)，安全可查。
                #
                # 判据取决于 EKF 的 local 原点在哪，两种定位方案根本不同：
                #  · GPS/SITL(默认)：原点≈各机自身出生点 ⇒ 收敛后静止时 |local_xy|≈0，
                #    仍几十米 = GPS fix 前暂态。
                #  · 动捕/外部定位(calib_shared_origin=true)：**所有机共享一个场地原点**
                #    ⇒ local 就是该机在场地系里的真实位置，本就等于其出生点，与 0 无关。
                #    此时应查"与期望出生点的偏差"。这不只是语义修正——它能抓出
                #    **刚体名↔drone_id 接错**这个动捕多机高发故障：pair2_in 两机刚体
                #    接反时 |local_xy|=1.4m 会通过原判据，随后 world_birth 被烤成
                #    (2.8,0)，两机认为各自在出生点、实际飞向同一物理点 → 撞机，全程零报错。
                if self.calib_shared_origin:
                    dev = float(np.linalg.norm(
                        (first_local - self.birth_positions[drone_idx])[:2]))
                    gate_ok = dev < self.calib_shared_origin_tol
                    gate_desc = (f'|local_xy-birth|={dev:.2f}m > '
                                 f'{self.calib_shared_origin_tol}m '
                                 f'(定位源与出生点不符？刚体名↔drone_id 接错？)')
                else:
                    dev = float(np.linalg.norm(first_local[:2]))
                    gate_ok = dev < self.calib_max_origin_offset
                    gate_desc = (f'|local_xy|={dev:.1f}m 远离原点(EKF 未 fix?)')
                # 共享原点模式下**闩锁**：一旦检出偏差就永久拒绝该机校准，直到重启。
                # 2026-07-23 SITL 实测（注入 1.0m 静态偏移模拟刚体错接）：仅靠瞬时判据
                # 挡不住 —— 未校准时飞机已被允许飞行去"守出生点"，而 EKF 认为自己偏了
                # 1m，于是**物理上朝反方向飞 1m** 把这个不存在的偏差"消除"掉；等它稳定，
                # dev 真的降到 0.07m，任何瞬时判据都会放行（实测两轮：先靠 timeout 放行，
                # 加了 stable 要求后仍靠"飞过去再稳定"放行，world_birth 都是错的）。
                # ⇒ 注入偏移被转化成了**物理位移**而非被检出。必须一旦检出就闩死。
                # 合法性：动捕是绝对定位，从第一帧起就该对得上，不存在"先偏后正"的
                # 合法暂态（那是 GPS fix 收敛才有的）。
                # 闩锁只在 EKF 已连续有效 calib_settle_frames 帧后才生效，避开
                # "EV 尚未融合、local 还是 0 而 birth 非 0"的初始化暂态（多机必踩）。
                if (self.calib_shared_origin and not gate_ok
                        and self._valid_streak[drone_idx] >= self.calib_settle_frames
                        and not self._calib_gate_latched[drone_idx]):
                    self._calib_gate_latched[drone_idx] = True
                    # 必须存**闩锁瞬间**的描述：飞机随后会漂走，此后每帧重算出来的
                    # dev 会变小，直接打印当前值会得到"0.04m > 0.5m"这种自相矛盾的话。
                    self._calib_latch_desc[drone_idx] = gate_desc
                if self._calib_gate_latched[drone_idx]:
                    if self._valid_streak[drone_idx] % 50 == 0:
                        self.get_logger().error(
                            f'[veh {drone_idx}] calib LATCHED: 曾检出 '
                            f'{self._calib_latch_desc[drone_idx]}'
                            f'，本次运行不再校准→保持出生点悬停。'
                            f'请检查刚体名↔drone_id 映射与动捕原点后重启。'
                        )
                    return
                if not gate_ok:
                    if timed_out and self._valid_streak[drone_idx] % 50 == 0:
                        self.get_logger().error(
                            f'[veh {drone_idx}] calib STUCK: {gate_desc}'
                            f'，拒绝校准→保持出生点悬停'
                        )
                    return
                # 共享原点模式下**不接受 timeout 兜底**，必须 stable。
                # 2026-07-23 SITL 实测：给注入位姿加 1.0m 静态偏移(模拟刚体错接)后，
                # 门控先正确拒绝(dev=0.93m)，但未校准时飞机按"出生点悬停"控制、而 EKF
                # 认为自己偏了 1m，于是物理上朝反方向跑去消除这个不存在的偏差；跑着跑着
                # dev 掉进容差(0.93→0.68→0.50)，被 timeout 分支放行，烤进错误的
                # world_birth=(0,-0.50,0)。**门控挡得住初始条件，挡不住系统漂移进容差。**
                # 该次锁定的 spread=(0.000,0.291,0.058) 远超 calib_stable_tol，
                # 故要求 stable 即可挡死。动捕位姿是绝对量且噪声极低(实测 spread=0.000)，
                # 本就不该需要 timeout 兜底——那是给 GPS 收敛暂态留的。
                if self.calib_shared_origin:
                    if not stable:
                        if timed_out and self._valid_streak[drone_idx] % 50 == 0:
                            self.get_logger().error(
                                f'[veh {drone_idx}] calib STUCK: 共享原点模式要求 local '
                                f'稳定(窗口极差<{self.calib_stable_tol}m)，当前 '
                                f'spread={tuple(round(float(v), 3) for v in spread)}'
                                f'，拒绝校准→保持出生点悬停'
                            )
                        return
                elif not (stable or timed_out):
                    return

                # 锁定时抓 ref_alt(此刻 EKF 已收敛/GPS fix,避免冻结 fix 前暂态值)
                if hasattr(msg, 'ref_alt') and math.isfinite(msg.ref_alt) and msg.ref_alt > 0:
                    self._ref_alt[drone_idx] = float(msg.ref_alt)
                # 锁定时抓 EKF 原点经纬度(仅 xy_global 有效=已有 GPS/RTK 全局参考时)
                if (getattr(msg, 'xy_global', False)
                        and math.isfinite(msg.ref_lat) and math.isfinite(msg.ref_lon)):
                    self._ref_lat[drone_idx] = float(msg.ref_lat)
                    self._ref_lon[drone_idx] = float(msg.ref_lon)
                # 基线：world_birth = 配置出生点 − first_local（SITL/动捕沿用；退化兜底）
                self.world_birth[drone_idx] = (
                    self.birth_positions[drone_idx] - first_local
                )
                # XY 全局对齐（RTK 真机）：用各机 EKF 原点 GPS 经纬度相对 drone0 datum 投影，
                # 覆盖 world_birth 的 XY 两轴（Z 仍由下方 alt_sync 处理）。各机 EKF local 系
                # 独立、first_local≈0，故盲信 birth_xy 会把摆放误差变成系统性队形偏移；
                # 这里让 RTK cm 精度自动把各机对齐进以 drone0 EKF 原点(+birth0)为原点的世界系。
                # 公共原点/自身经纬度任一不可用（未收到 datum / 无全局 fix）→ 保持上面的基线值。
                if (self._xy_global_align_enable
                        and self._datum_ref_latlon is not None
                        and self._ref_lat[drone_idx] is not None
                        and self._ref_lon[drone_idx] is not None):
                    lat0, lon0 = self._datum_ref_latlon
                    dN, dE = _latlon_to_ned(
                        self._ref_lat[drone_idx], self._ref_lon[drone_idx], lat0, lon0)
                    self.world_birth[drone_idx, 0] = float(self.birth_positions[0, 0] + dN)
                    self.world_birth[drone_idx, 1] = float(self.birth_positions[0, 1] + dE)
                    self.get_logger().info(
                        f'[veh {drone_idx}] XY 全局对齐: EKF原点相对drone0 datum '
                        f'(dN={dN:+.2f} dE={dE:+.2f})m → world_birth_xy=('
                        f'{self.world_birth[drone_idx,0]:+.2f},'
                        f'{self.world_birth[drone_idx,1]:+.2f}) '
                        f'[配置 birth_xy=({self.birth_positions[drone_idx,0]:+.2f},'
                        f'{self.birth_positions[drone_idx,1]:+.2f})]'
                    )
                    self._xy_aligned[drone_idx] = True
                # Z 校准：用 EKF 参考海拔差异补偿各机 home 海拔不同。
                # 各机 EKF 启动时气压计读数不同 → home 海拔(ref_alt)不同
                # → 同一 local_z 对应不同实际海拔。
                # 用 ref_alt 差作为 world_birth_z 偏移量，
                # 使 MPC 世界坐标系中所有机共享同一海拔基准。
                ref_0   = self._ref_alt[0]
                ref_i   = self._ref_alt[drone_idx]
                if self._alt_sync_enable and ref_0 is not None and ref_i is not None:
                    alt_offset = ref_0 - ref_i   # 正值 = 我比基准(home)低
                    self.world_birth[drone_idx, 2] = (
                        self.birth_positions[drone_idx, 2] + alt_offset
                    )
                    self.get_logger().info(
                        f'[veh {drone_idx}] alt sync: '
                        f'my_ref_alt={ref_i:.2f} ref_alt_0={ref_0:.2f} '
                        f'offset={alt_offset:+.2f}m → '
                        f'world_birth_z={self.world_birth[drone_idx,2]:.2f}'
                    )
                else:
                    # alt_sync 关闭(SITL同地面)或 ref_alt 不可用 → 用 birth_z，各机各控离地 target_alt
                    self.world_birth[drone_idx, 2] = self.birth_positions[drone_idx, 2]
                    self.get_logger().info(
                        f'[veh {drone_idx}] alt_sync {"disabled" if not self._alt_sync_enable else "no ref_alt"}, '
                        f'using birth_z={self.birth_positions[drone_idx,2]:.2f}'
                    )
                self._prev_xy_reset[drone_idx] = msg.xy_reset_counter
                self._prev_z_reset[drone_idx] = msg.z_reset_counter
                self._pos_calibrated[drone_idx] = True
                tag = 'STABLE' if stable else 'TIMEOUT(not converged)'
                self.get_logger().info(
                    f'first position from drone {drone_idx} '
                    f'(calibrated [{tag}] after {self._valid_streak[drone_idx]} frames, '
                    f'spread=({spread[0]:.3f},{spread[1]:.3f},{spread[2]:.3f})): '
                    f'local_mean=({first_local[0]:.2f}, {first_local[1]:.2f}, {first_local[2]:.2f}) '
                    f'world_birth=({self.world_birth[drone_idx,0]:.2f}, '
                    f'{self.world_birth[drone_idx,1]:.2f}, '
                    f'{self.world_birth[drone_idx,2]:.2f})'
                )

            ds.received = True
            ds.last_stamp = now
            ds.xy_valid = bool(msg.xy_valid)   # EKF 健康（safety_filter 估计门）
            ds.z_valid = bool(msg.z_valid)
            # Use DYNAMIC world_birth (compensates for EKF resets)
            ds.pos = np.array([msg.x, msg.y, msg.z]) + self.world_birth[drone_idx]
            ds.vel = np.array([msg.vx, msg.vy, msg.vz])
            ds.yaw = float(msg.heading) if math.isfinite(msg.heading) else 0.0

            # 🔴 起飞期离地闩锁必须在**位置回调**里推进，不能在控制回路里（2026-08-21 实测 bug）：
            #    控制回路只在进入 OFFBOARD 之后才跑，而推荐流程是"手动起飞到 1–2m 再切 OFFBOARD"
            #    ⇒ 首次捕获到的"地面基准"必然是空中的 z ⇒ 相对上升量永远到不了 takeoff_rise
            #    ⇒ 闩锁永不解除、XY 速度指令全程为零，飞机随风漂到 2.9m 才被安全滤波拦下。
            #    两个架次实测如此（d0_airborne 列全程 0 即证据）。
            #    位置话题从节点启动就在流、且与 vehicle_status 同属 _cb_subs（串行，无竞态），
            #    解锁边沿之后的第一帧必然还在地面，在这里捕获才是对的。
            if drone_idx == self.drone_id:
                self._update_airborne(ds.pos[2], ds.vel[2])
        return cb

    def _update_airborne(self, z_now, vz_now):
        """推进离地闩锁。z_now/vz_now = 自机当前 z 与 z 速度（NED，向下为正）。

        地面基准在解锁边沿武装、在此处用第一拍有效 z 填充（回调时 self_ds 可能还没数据）。

        🔴 2026-09-01 判据升级为**双条件**（台架 ⑤-21 击穿后）：
          ① 位移：相对解锁基准上升 ≥ takeoff_rise（原判据，保留）
          ② 爬升：vz ≤ -takeoff_climb_ms **连续保持** takeoff_climb_hold_s
        怠速下 EKF z 随机漂 ±0.5–0.6 m 能凑够①（实测 24 s 假离地），但漂移速率
        ~0.02–0.05 m/s 远低于②的 0.2 m/s、且撑不满 1 s 连续——两条件同时成立
        实际上只有真起飞做得到。反向漂移（①永不满足）由看门狗兜底（fail-open）。
        满足即闩锁"已离地"，本架次不再回退（防地面反弹/EKF 抖动来回切）。

        🔑 看门狗也在此处驱动（2026-09-01 从控制回路挪来）：控制回路依赖 leader 才跑，
        台架无 leader 时看门狗从未执行过（⑤-21 第二缺陷，与当年闩锁 bug 同根）；
        位置回调从节点启动就在流，与 leader 无关。
        """
        self._check_takeoff_lock_watchdog()
        if self._airborne:
            return
        if self._ground_z is None:
            self._ground_z = float(z_now)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        # 条件②：爬升速度连续超阈计时（NED 向下为正 ⇒ 爬升 = vz 为负）
        if math.isfinite(vz_now) and float(vz_now) <= -self._takeoff_climb_ms:
            if self._climb_since is None:
                self._climb_since = now
        else:
            self._climb_since = None
        climb_held = (self._climb_since is not None
                      and (now - self._climb_since) >= self._takeoff_climb_hold_s)
        # 条件①：NED：z 向下为正 ⇒ 上升量 = ground_z - z_now
        rise = self._ground_z - float(z_now)
        if rise >= self._takeoff_rise and climb_held:
            self._airborne = True
            self.get_logger().info(
                f'[d{self.drone_id}] 已离地（上升 {rise:.2f}m 且持续爬升 '
                f'{now - self._climb_since:.1f}s）—— 横向速度指令解锁')

    def _check_takeoff_lock_watchdog(self):
        """起飞锁兜底：解锁后超过 _takeoff_lock_max_s 仍未判定离地 ⇒ 强制解锁并告警。

        闩锁正常由位置回调推进；此看门狗只处理判据本身失灵的情况（位置流中断、
        takeoff_rise 配得过大、气压计异常……）。方向刻意是 fail-open：锁着比不锁危险。
        """
        if self._airborne or not self._takeoff_lock_enable or self._arm_time is None:
            return
        held = self.get_clock().now().nanoseconds * 1e-9 - self._arm_time
        if held > self._takeoff_lock_max_s:
            self._airborne = True
            self.get_logger().warn(
                f'[d{self.drone_id}] 起飞锁看门狗：解锁后 {held:.1f}s 仍未判定离地 '
                f'(ground_z={self._ground_z}, rise 阈值 {self._takeoff_rise:.2f}m) '
                f'—— 强制解除横向锁定（fail-open）')

    def _on_vehicle_status(self, msg):
        # 新架次边沿检测（disarmed/unknown → armed）：RELINQUISH 闩锁本就设计为
        # sticky（防止故障闪烁间歇性抢回控制权），但仅限"同一架次内"；跨架次若不清零，
        # 一次 RELINQUISH 会让 companion 侧对这架机永久失能，直到重启 ROS2 进程。
        # 重新解锁武装是飞手/FC 的显式动作（经过 prearm 检查），在此时机清零是安全的。
        if msg.arming_state == 2 and self._prev_arming_state != 2:
            if self.safety is not None and self.safety.state != SAFETY_NORMAL:
                self.get_logger().warn(
                    f'[d{self.drone_id}] 新架次 ARMED：安全闩锁 {self.safety.state}→NORMAL 已清零')
            if self.safety is not None:
                self.safety.reset()
            self._relinquished = False
            self._relinquish_next_cmd = 0.0
            # 任务降落闩锁同样按架次清零：重新解锁=飞手/FC 显式动作，给新架次干净状态。
            self._landing_requested = False
            self._land_settle_start = None
            self._land_commanded = False
            self._land_next_cmd = 0.0
            self._comms_loss_landed = False   # 第2层失联降落看门狗按架次重新武装
            self._comms_hold_active = False   # 就地冻结边沿标志同样按架次清零
            # 起飞期横向锁定按架次重新武装：此刻飞机必然在地面，捕获 z 作地面基准。
            self._airborne = False
            self._ground_z = None     # 下一帧位置回调用当时的 z 填（此处 self_ds 可能还没数据）
            self._climb_since = None  # 爬升连续计时按架次清零（双条件判据之②）
            self._takeoff_lock_logged = False
            self._arm_time = self.get_clock().now().nanoseconds * 1e-9
            # 半自主档的一次性授权按架次重新武装：新解锁 = 新的起飞授权。
            self._offboard_engaged_once = False
            self._offboard_req_count = 0
        self._prev_arming_state = msg.arming_state
        self._arming_state = msg.arming_state
        self._nav_state    = msg.nav_state

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.param3           = float(param3)
        msg.target_system    = self.drone_id + 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_cmd.publish(msg)

    def _arm_and_engage_offboard(self):
        """Retry ARM + OFFBOARD until confirmed. Call every ~100 frames (2 s at 50 Hz).
        auto_arm_enable=false（真机）时不发指令，只被动等待飞手 RC 操作后的状态确认；
        offboard_on_arm=true（半自主档）时不发 ARM，仅在检测到飞手已解锁后补发 OFFBOARD。"""
        if self._arm_offboard_confirmed:
            return
        if self.auto_arm_enable:
            if self._nav_state != 14:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self._arming_state != 2:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif self.offboard_on_arm:
            # 半自主：ARM 永远不发（解锁权在飞手）；只在已解锁且尚未进 OFFBOARD 时切模式。
            # setpoint 流从节点启动就在（PX4 要求切换前 >2Hz，满足）。
            # 🔴 防抢杆双保险（授权是一次性的，不是持续性的）：
            #   ① 每个解锁架次只自动进一次 OFFBOARD（_offboard_engaged_once）：进过之后
            #      飞手切走 = 接管，节点绝不切回去；要恢复须飞手自己再切 OFFBOARD。
            #      没有这一条，OFFBOARD LOST 复位 confirmed 后本函数会每 2s 抢一次模式。
            #   ② 首次进入前的重试也设上限（15 次≈30s）：若 PX4/飞手持续拒绝，放弃并留告警,
            #      而不是无限期跟飞手的挡位开关对抗。
            if (self._arming_state == 2 and self._nav_state != 14
                    and not self._offboard_engaged_once):
                if self._offboard_req_count < 15:
                    self._offboard_req_count += 1
                    self._send_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.get_logger().info(
                        f'[d{self.drone_id}] 半自主档：已解锁，请求切 OFFBOARD '
                        f'({self._offboard_req_count}/15, nav={self._nav_state})')
                elif self._offboard_req_count == 15:
                    self._offboard_req_count += 1   # 只告警一次
                    self.get_logger().warn(
                        f'[d{self.drone_id}] 半自主档：15 次请求均未进入 OFFBOARD，放弃。'
                        f'本架次不再自动切换；飞手可手动切 OFFBOARD 接续')
        if self._nav_state == 14 and self._arming_state == 2:
            self._arm_offboard_confirmed = True
            self._offboard_engaged_once = True   # 半自主档：本架次授权已用掉，之后不再自动切
            self.get_logger().info(
                f'drone {self.drone_id}: OFFBOARD + ARMED confirmed')

    def on_self_attitude(self, msg):
        self.attitude_yaw = quaternion_to_yaw(msg.q)
        self.attitude_received = True

    def on_leader_state(self, msg):
        if len(msg.data) < 8:
            return
        self.leader_received = True
        self._last_leader_rx = self.get_clock().now().nanoseconds * 1e-9
        self.leader_pos = np.array([msg.data[1], msg.data[2], msg.data[3]])
        self.leader_vel = np.array([msg.data[4], msg.data[5], msg.data[6]])
        self.leader_yaw = float(msg.data[7]) if math.isfinite(msg.data[7]) else 0.0
        # 加速度（圆周运动向心加速度），向后兼容旧版 leader_node
        if len(msg.data) >= 10:
            ax = float(msg.data[8]) if math.isfinite(msg.data[8]) else 0.0
            ay = float(msg.data[9]) if math.isfinite(msg.data[9]) else 0.0
            self.leader_acc = np.array([ax, ay, 0.0])
        else:
            self.leader_acc = np.zeros(3)

    def _on_mission_cmd(self, msg):
        """任务完成降落指令（第 1 层）。收到 'LAND' 置位，由 control_loop 走 settle→AUTO.LAND。"""
        cmd = str(msg.data).strip().upper()
        if cmd == 'LAND' and not self._landing_requested and not self._land_commanded:
            self._landing_requested = True
            self.get_logger().warn(
                f'[d{self.drone_id}] 收到任务完成 LAND 指令 — 进入 settle→自主降落')

    def _make_health_callback(self, drone_idx):
        """记录邻居 health 到达时刻 —— 用于区分"mpc 崩了"与"预测包丢了"。"""
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            # 链路完全中断(scope='all')时 health 同样收不到；scope='pred' 模拟的是
            # 大包选择性丢失，health/位置照常可达。
            if self._blackout_scope == 'all' and self._in_comms_blackout(now):
                return
            self.peer_health_stamps[drone_idx] = now
        return cb

    def _in_comms_blackout(self, now):
        """P2 注入: 周期性通信中断窗口判定（0=关闭）。

        与 comms_dropout 的区别 —— 这是本类注入里唯一能真正触发
        neighbour_timeout 降级的：
          · dropout 是 i.i.d. 伯努利，20% 下连续丢满 timeout(2s@50Hz=100条)
            的概率约 1e-70，**降级路径事实上从未被触发**（S25 的实际情况）。
          · 真实 2.4G 无线的典型失效是遮挡/干扰造成的**突发中断**：连续断
            数百 ms ~ 数秒后恢复。本注入模拟的正是这个，用于验证
            "降级进得去、恢复出得来"。
        时间基准为节点启动时刻，故场景里 start_s 需覆盖 leader start_delay + 起飞段。
        """
        if self.comms_blackout_dur <= 0.0 or self.comms_blackout_period <= 0.0:
            return False
        t = now - self._node_t0 - self.comms_blackout_start
        if t < 0.0:
            return False
        return (t % self.comms_blackout_period) < self.comms_blackout_dur

    def _make_pred_callback(self, drone_idx):
        def cb(msg):
            data = np.asarray(msg.data, dtype=float)
            if data.size % 3 != 0 or data.size == 0:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            # P2 注入: 突发中断 — 窗口内全部丢弃（会累积到 neighbour_timeout 触发降级）
            if self._in_comms_blackout(now):
                return
            # P2 注入: 丢包 — 直接丢弃本条预测（超时后走既有失联降级路径）
            if self.comms_dropout > 0.0 and random.random() < self.comms_dropout:
                return
            traj = data.reshape(-1, 3)
            if self.comms_delay_s > 0.0:
                # P2 注入: 延迟 — 入缓冲，到期才可见；stamp 记原始到达时刻，
                # 使既有 latency 时间对齐自然吸收注入延迟
                self._pred_delay_buf.setdefault(drone_idx, []).append((now, traj))
            else:
                self.peer_predictions[drone_idx] = traj
                self.peer_prediction_stamps[drone_idx] = now
        return cb

    def _drain_pred_delay_buf(self, now):
        """P2: 把注入延迟已到期的邻居预测放行到 peer_predictions。"""
        if self.comms_delay_s <= 0.0:
            return
        for j, buf in self._pred_delay_buf.items():
            while buf and (now - buf[0][0]) >= self.comms_delay_s:
                arrival_t, traj = buf.pop(0)
                self.peer_predictions[j] = traj
                self.peer_prediction_stamps[j] = arrival_t

    # =================================================================
    # control loop
    # =================================================================
    def _on_alt_datum(self, msg):
        self._datum_ref_alt = float(msg.data)

    def _on_xy_datum(self, msg):
        if len(msg.data) >= 2 and all(math.isfinite(v) for v in msg.data[:2]):
            self._datum_ref_latlon = (float(msg.data[0]), float(msg.data[1]))

    def _eff_target_alt(self):
        """有效目标高度（世界系 NED）= 基准 target_alt + 悬停期标定的 alt_trim。
        alt_trim=0 时等价原行为。"""
        return self.target_alt + self._alt_trim

    def _update_alt_trim(self):
        """悬停期：用本机 ref_alt 与 drone0 datum 之差偏调 target_alt，使各机收敛到
        同一绝对海拔（=同真高）。leader 起动即冻结，圆周中不再变。只调设定点、不动
        world_birth_z，故不扰碰撞/编队几何（避开 alt_resync 撞机根因）。"""
        if not self._alt_trim_enable or self._alt_trim_frozen:
            return
        # leader 起动 → 冻结当前 trim
        if float(np.linalg.norm(self.leader_vel)) > 0.05:
            self._alt_trim_frozen = True
            self.get_logger().info(
                f'[d{self.drone_id}] alt_trim 冻结 @ {self._alt_trim:+.2f}m（leader 起动）')
            return
        my_ref = self._ref_alt_now[self.drone_id]
        if (self._datum_ref_alt is None or my_ref is None
                or not self._pos_calibrated[self.drone_id]):
            return
        # 离地<1m（起飞期）不调，防地面暂态
        self_ds = self.drone_states[self.drone_id]
        if not self_ds.received or (-self_ds.pos[2]) < 1.0:
            return
        raw = float(np.clip(my_ref - self._datum_ref_alt,
                            -self._alt_trim_max, self._alt_trim_max))
        # 低通收敛，防 ref_alt 噪声抖动
        self._alt_trim = 0.9 * self._alt_trim + 0.1 * raw

    def _resync_world_birth_z(self, dt):
        """Tier2: 把(已标定)机的 world_birth_z 限速拉向 birth_z + (datum - 当前ref_alt)。
        ref_alt 连续温漂不触发 z_reset，:507 补不到；这里持续纠偏，且限速使 MPC 只看到
        ≤alt_resync_rate 的 z 速度，不会跳。基准 = drone0 当前 ref_alt（广播给全员）。"""
        # drone0 广播当前基准（己方标定后才发，避免广播 GPS-fix 前暂态）
        if (self.drone_id == 0 and self._pos_calibrated[0]
                and self._ref_alt_now[0] is not None):
            self._datum_ref_alt = self._ref_alt_now[0]
            if self.pub_alt_datum is not None:
                m = Float64()
                m.data = float(self._datum_ref_alt)
                self.pub_alt_datum.publish(m)
        # drone0 广播 XY 公共原点（己方标定后 _ref_lat[0]/_ref_lon[0] 已冻结，避免 fix 前暂态）
        if (self._xy_global_align_enable and self.drone_id == 0
                and self._pos_calibrated[0] and self.pub_xy_datum is not None
                and self._ref_lat[0] is not None and self._ref_lon[0] is not None):
            self._datum_ref_latlon = (self._ref_lat[0], self._ref_lon[0])
            mxy = Float64MultiArray()
            mxy.data = [float(self._ref_lat[0]), float(self._ref_lon[0])]
            self.pub_xy_datum.publish(mxy)
        # datum 或某机全局 fix 迟于其校准锁定时，该机 world_birth XY 曾退化为配置 birth
        # （未对齐）；齐备后一次性补算（_xy_aligned 防重复，不受 alt_resync 开关影响）。
        if self._xy_global_align_enable and self._datum_ref_latlon is not None:
            lat0, lon0 = self._datum_ref_latlon
            for idx in [self.drone_id] + list(self.neighbours):
                if (self._pos_calibrated[idx] and not self._xy_aligned[idx]
                        and self._ref_lat[idx] is not None
                        and self._ref_lon[idx] is not None):
                    dN, dE = _latlon_to_ned(
                        self._ref_lat[idx], self._ref_lon[idx], lat0, lon0)
                    self.world_birth[idx, 0] = float(self.birth_positions[0, 0] + dN)
                    self.world_birth[idx, 1] = float(self.birth_positions[0, 1] + dE)
                    self._xy_aligned[idx] = True
                    self.get_logger().info(
                        f'[veh {idx}] XY 全局对齐(datum 迟到补算): dN={dN:+.2f} '
                        f'dE={dE:+.2f} → world_birth_xy=('
                        f'{self.world_birth[idx,0]:+.2f},{self.world_birth[idx,1]:+.2f})')
        if not self._alt_resync_enable or self._datum_ref_alt is None:
            return
        # 自机离地 < 1m 时不做 resync：防起飞期 world_birth_z 误漂导致安全层误判高度
        self_ds_alt = self.drone_states[self.drone_id]
        if not self_ds_alt.received or (-self_ds_alt.pos[2]) < 1.0:
            return
        step = max(0.0, self._alt_resync_rate * dt)
        for idx in [self.drone_id] + list(self.neighbours):
            if not self._pos_calibrated[idx] or self._ref_alt_now[idx] is None:
                continue
            birth_z = float(self.birth_positions[idx, 2])
            target  = birth_z + (self._datum_ref_alt - self._ref_alt_now[idx])
            if abs(target - birth_z) > self._alt_resync_max:
                continue   # ref_alt 异常大偏移，安全起见不跟
            cur = float(self.world_birth[idx, 2])
            err = target - cur
            self.world_birth[idx, 2] = (
                target if abs(err) <= step else cur + math.copysign(step, err))

    def _dob_update(self, v_meas, u_nom, dt):
        """Q-filter 扰动观测器（标准 Ohishi DOB），逐轴给出加性集总扰动 d̂。

        名义模型 v̇ = u_nom + d ⇒ 瞬时残差 r = v̇_meas − u_nom 即扰动的含噪估计；
        d̂ = 一阶低通 LPF_L[r]，**无积分器 ⇒ 不 windup**。v̇_meas 用实测速度有限差分。
        圆周稳态下 r 均值≈0 → d̂≈0（不扰动编队）；持续风扰下 d̂ 跟残差 DC 分量。
        论文 §6.5 DOB baseline：把 d̂ 前馈进 legacy 预测模型 v̇=u+d̂（审稿意见 M2）。带宽有限
        ⇒ 跟不上输入通道滞后的快速动态，与 §3.3/§5.3 否定性预测一致。|d̂| clip 防异常。
        """
        v = np.asarray(v_meas, dtype=float).reshape(3)
        u = np.asarray(u_nom,  dtype=float).reshape(3)
        if self._dob_vprev is None or not np.all(np.isfinite(self._dob_vprev)):
            self._dob_vprev = v.copy()
            self._dob_dhat = np.zeros(3)
            return self._dob_dhat.copy()
        vdot = (v - self._dob_vprev) / max(dt, 1e-3)
        self._dob_vprev = v.copy()
        resid = np.clip(vdot - u, -self._dob_dmax, self._dob_dmax)   # 瞬时残差（含噪）
        a = (self._dob_L * dt) / (1.0 + self._dob_L * dt)            # 一阶 LPF 系数
        self._dob_dhat = (1.0 - a) * self._dob_dhat + a * resid
        if not np.all(np.isfinite(self._dob_dhat)):                 # 数值异常 → 复位
            self._dob_vprev = v.copy()
            self._dob_dhat = np.zeros(3)
        return self._dob_dhat.copy()

    def control_loop(self):
        # RELINQUISH（方案b 主动交还）：已命令 PX4 切 Hold，此后彻底停流——
        # 不发心跳/setpoint/ARM 指令，不自动恢复 OFFBOARD（停心跳兼作 COM_OF_LOSS_T 兜底）。
        # Hold(nav=AUTO_LOITER) 确认前每 1s 重发一次 DO_SET_MODE。
        if self._relinquished:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if (self._nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                    and now_s >= self._relinquish_next_cmd):
                self._relinquish_next_cmd = now_s + 1.0
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)  # AUTO/LOITER=Hold
                self.get_logger().warn(
                    f'[d{self.drone_id}] RELINQUISH: 重发 DO_SET_MODE(Hold), nav={self._nav_state}')
            self._publish_health()
            return
        # 任务降落终态：已命令 AUTO.LAND → 停 offboard 流，交给 PX4 自主降落+触地自动上锁。
        # 与 RELINQUISH 同构：停心跳兼作兜底，Land(nav=AUTO_LAND) 确认前每 1s 重发 DO_SET_MODE。
        if self._land_commanded:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if (self._nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                    and now_s >= self._land_next_cmd):
                self._land_next_cmd = now_s + 1.0
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)  # AUTO/LAND
                self.get_logger().warn(
                    f'[d{self.drone_id}] LAND: 重发 DO_SET_MODE(Land), nav={self._nav_state}')
            self._publish_health()
            return
        self.publish_offboard_mode()
        now_ros = self.get_clock().now()
        dt = (now_ros - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now_ros
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.control_hz

        self._resync_world_birth_z(dt)   # Tier2: 补偿 ref_alt 连续温漂
        self._update_alt_trim()          # 悬停期高度 trim（leader 起动前标定，之后冻结）

        # 使用当前姿态 yaw，避免起飞时强制转向正北
        yaw_hold = self.attitude_yaw if self.attitude_received else 0.0

        if self._startup_counter < self.startup_zero_vel_frames:
            self._startup_counter += 1
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        # After startup: retry ARM+OFFBOARD every 50 frames (1 s) until confirmed.
        # Also reset if OFFBOARD was lost after initial confirmation (nav dropped from 14).
        # 🔴 2026-08-20 去掉了这里原有的 `and self.auto_arm_enable` 门控。
        #    原写法的问题：auto_arm=false（真机默认、也是首飞用的）时飞手切走模式，
        #    本节点**毫不知情**，继续跑完整 MPC 回路、继续下发速度设定点。
        #    真机首飞 bag 实测：飞手 405.2s 切回 POSCTL 手动降落，节点的 `sp_vz` 却因为
        #    高度误差越拉越大而一路涨到满幅 −0.80 m/s（满爬升），全程持续下发。
        #    PX4 在非 OFFBOARD 下会忽略这些包，所以当时没出事 —— 但只要飞手再切回
        #    OFFBOARD，接管的第一帧就是满幅爬升指令。
        # ✅ 现在不论 auto_arm 与否都清 `_arm_offboard_confirmed`：
        #    · auto_arm=true  → 行为不变（重发 ARM+OFFBOARD）
        #    · auto_arm=false → 落回下面那段被动等待分支，只发
        #      `_hover_setpoint_world()`（当前 XY + target_alt 的位置设定点、零速度），
        #      并在飞手切回 OFFBOARD 时被 `_arm_and_engage_offboard()` 被动确认后恢复。
        #    `_arm_and_engage_offboard()` 内部本就有 auto_arm_enable 判断（行 1420），
        #    不会因此在真机上擅自发解锁指令。
        if self._arm_offboard_confirmed and self._nav_state != 14:
            self._arm_offboard_confirmed = False
            self._cmd_retry_counter = 0
            action = 'will re-arm' if self.auto_arm_enable else 'waiting for RC to re-engage'
            self.get_logger().warn(
                f'drone {self.drone_id}: OFFBOARD LOST (nav={self._nav_state}) — '
                f'resetting, {action}')
        if not self._arm_offboard_confirmed:
            if not self._ocp_ready:   # 等 acados 编译完再尝试 ARM，避免 OFFBOARD 超时
                self._hover_active = True
                self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
                return
            self._cmd_retry_counter += 1
            if self._cmd_retry_counter % 50 == 1:
                self._arm_and_engage_offboard()
                if not self._arm_offboard_confirmed:
                    action = ('retry ARM+OFFBOARD' if self.auto_arm_enable
                              else 'waiting RC ARM+OFFBOARD')
                    self.get_logger().info(
                        f'drone {self.drone_id}: {action} '
                        f'(nav={self._nav_state}, arm={self._arming_state}) '
                        f'frame={self._cmd_retry_counter}')
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        self_ds = self.drone_states[self.drone_id]
        if not self_ds.received:
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        # 通信失联自动降落（第 2 层）看门狗：OFFBOARD 飞行中，若超过 comms_loss_land_s 秒没收到
        # leader（WiFi 丢 / leader 崩），触发第 1 层降落。飞控层兜不住这种"companion 健康、仍在发
        # setpoint、只是收不到协同"的情形（飞控只见 offboard 正常，永不失联降落）。
        # 只在真正 OFFBOARD 飞行(nav=14)且已至少收到过一次 leader 后才武装 → 不会在起飞前的等待/
        # 悬停阶段误触发；触发即置 _landing_requested，交由下方现成的 settle→AUTO.LAND 状态机执行。
        if (self.comms_loss_land_s > 0.0 and not self._landing_requested
                and not self._land_commanded and self._nav_state == 14
                and self.leader_received and self._last_leader_rx is not None):
            _since_leader = (self.get_clock().now().nanoseconds * 1e-9
                             - self._last_leader_rx)
            if _since_leader > self.comms_loss_land_s and not self._comms_loss_landed:
                self._comms_loss_landed = True
                self._landing_requested = True
                self.get_logger().error(
                    f'[d{self.drone_id}] 通信失联：已 {_since_leader:.1f}s 未收到 leader '
                    f'(阈值 {self.comms_loss_land_s:.1f}s) — 触发自动降落，'
                    f'避免空中滞留至电池耗尽')

        # 任务完成降落（第 1 层）settle 段：收到 LAND 后先零速悬停 land_settle_s 秒把编队速度
        # 刹掉（保持当前 XY + 目标高度），再命令 AUTO.LAND 并转入停流终态。放在 leader 门控之前
        # → 即使 leader 掉线也能降落；只需自机位置有效（已过上面 self_ds.received）。
        if self._landing_requested:
            self._hover_active = True
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if self._land_settle_start is None:
                self._land_settle_start = now_s
                self.get_logger().warn(
                    f'[d{self.drone_id}] LAND settle: 零速悬停 {self.land_settle_s:.1f}s 后切 AUTO.LAND')
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            if now_s - self._land_settle_start >= self.land_settle_s:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)  # AUTO/LAND
                self._land_commanded = True
                self._land_next_cmd = now_s + 1.0
                self.get_logger().warn(
                    f'[d{self.drone_id}] LAND: settle 完成 → 命令 AUTO.LAND，停 offboard 流交给 PX4')
            self._publish_health()
            return

        # 失联降落"先冻结"中间阶段（第 2 层）：leader 失联超过 comms_loss_hold_s 但未到 land 阈值时，
        # **就地悬停**而非继续朝最后一次的陈旧 leader 参考飞（hover 无影响，circle/line 下否则会朝
        # leader 旧位置持续漂）。到 land 阈值由上方看门狗切降落；leader 在此之前恢复则退出、恢复编队。
        if (self.comms_loss_hold_s > 0.0 and self.comms_loss_land_s > 0.0
                and self.leader_received and self._last_leader_rx is not None):
            _stale = self.get_clock().now().nanoseconds * 1e-9 - self._last_leader_rx
            if _stale > self.comms_loss_hold_s:
                if not self._comms_hold_active:
                    self._comms_hold_active = True
                    self.get_logger().warn(
                        f'[d{self.drone_id}] leader 失联 {_stale:.1f}s(>{self.comms_loss_hold_s:.1f}s)'
                        f' — 就地悬停，不追陈旧参考（{self.comms_loss_land_s:.1f}s 仍未恢复则降落）')
                self._hover_active = True
                self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
                self._publish_health()
                return
            if self._comms_hold_active:
                self._comms_hold_active = False
                self.get_logger().info(
                    f'[d{self.drone_id}] leader 已恢复 — 退出就地悬停，恢复编队跟随')

        if not self.leader_received:
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        x_ref = self._build_reference_trajectory()
        nb_traj = self._collect_neighbour_predictions()
        x0 = np.concatenate([self_ds.pos, self_ds.vel])

        # 论文 §6.5 非-MPC 对照：分布式一致性编队律（旁路 MPC，自带安全+发布，直接返回）。
        if self.controller == 'consensus':
            self._consensus_step(self_ds, x_ref, dt, now_ros)
            return

        # 编队项对"mpc 已死/未参战但 PX4 仍在发位置"的队友失效化：该队友趴在原地时，
        # (d_i−d_star) 残差会把幸存机往它那边持续拽出恒定跟踪偏差（S2/S23/S24 FAIL 根因，
        # 2026-07-14 取证：S2 残差 1.6m、S23 0.8m、S24 圆周振荡同源）。将其 d_star 动态置为
        # 当前实际距离 → 残差归零无拉力；碰撞项走 d_safe 与 d_star 无关，硬地板不受影响。
        dd = self.desired_distances
        now_dd = self.get_clock().now().nanoseconds * 1e-9
        # ⚠️ 判据必须再加一道 health 门（2026-07-22）：只用"预测过期 + 位置新鲜"
        # **分不清"队友 mpc 崩了"与"预测包丢了"**。预测包 ~800-1000B，位置包
        # ~100-200B，丢包链路上大包更易丢 → 真机无线劣化时极易误判成趴窝、
        # 把编队项关掉（SITL 实测：只断预测的场景下误触发 81 次 vs 全断时 5 次）。
        # health 仅 ~36B 且 RELIABLE，用它的新鲜度独立判断对方 mpc 是否还在跑。
        # 从未收到过 health 的邻居（旧版本/尚未起来）退回原判据，保持向后兼容。
        for _idx, _j in enumerate(self.neighbours):
            _pred_stale = (now_dd - self.peer_prediction_stamps.get(_j, 0.0)) > self.neighbour_timeout
            _ds_j = self.drone_states[_j]
            _pos_fresh = _ds_j.received and (now_dd - _ds_j.last_stamp) <= self.neighbour_timeout
            _h_stamp = self.peer_health_stamps.get(_j)
            _health_alive = (_h_stamp is not None
                             and (now_dd - _h_stamp) <= self.neighbour_timeout)
            if _pred_stale and _pos_fresh:
                if _health_alive:
                    # mpc 还在心跳 → 预测过期是**通信丢包**，不是趴窝。保持编队项。
                    if self._dbg_counter % int(self.control_hz) == 0:
                        self.get_logger().warn(
                            f'[d{self.drone_id}] 预测过期但 d{_j} health 仍在心跳 '
                            f'→ 判为丢包(非趴窝)，编队项保持')
                    continue
                if dd is self.desired_distances:
                    dd = self.desired_distances.copy()
                dd[_idx] = float(np.linalg.norm(self_ds.pos[:2] - _ds_j.pos[:2]))
                if self._dbg_counter % int(self.control_hz) == 0:
                    self.get_logger().warn(
                        f'[d{self.drone_id}] formation-neutral: d{_j} mpc静默但PX4存活, '
                        f'd_star←{dd[_idx]:.2f}(当前距离), 仅保留碰撞地板')

        self._hover_active = False   # 假设本帧正常，后续如有降级会覆盖为 True

        # DOB baseline（§6.5）：解算前先更新扰动观测器，把 d̂ 注入预测模型 v̇=u+d̂。
        # 观测器名义输入 u_nom = 上一拍 MPC 命令加速度 u0；实测速度 = self_ds.vel。
        # 接入门：编队成型阶段 d̂=0（不干扰成型），leader 起动(绕圈)或 warmup 兜底后才接入。
        d_hat = None
        if self.mpc._dob:
            now_dob = self.get_clock().now().nanoseconds * 1e-9
            if not self._dob_engaged:
                if self._dob_first_ctrl_t is None:
                    self._dob_first_ctrl_t = now_dob
                leader_moving = float(np.linalg.norm(self.leader_vel[:2])) > 0.1
                warmed = (now_dob - self._dob_first_ctrl_t) > self._dob_warmup_s
                if leader_moving or warmed:
                    self._dob_engaged = True
                    self._dob_vprev = None          # 接入瞬间重置观测器，从干净初值起
                    self._dob_dhat = np.zeros(3)
                    self.get_logger().info(
                        f'[d{self.drone_id}] DOB 接入（leader_moving={leader_moving} '
                        f'warmed={warmed}）')
            if self._dob_engaged:
                d_hat = self._dob_update(self_ds.vel, self._last_u0,
                                         1.0 / max(self.control_hz, 1.0))
            else:
                d_hat = np.zeros(3)                 # 未接入 → 模型 d̂=0（等价 legacy）
            self._last_dhat = np.asarray(d_hat, dtype=float).reshape(3)  # 供 health/CSV 记录
            if self._dbg_counter % int(self.control_hz) == 0:
                self.get_logger().info(
                    f'[d{self.drone_id}] DOB d_hat=({d_hat[0]:+.3f},{d_hat[1]:+.3f},'
                    f'{d_hat[2]:+.3f}) |d_hat|={float(np.linalg.norm(d_hat)):.3f} m/s^2 '
                    f'engaged={self._dob_engaged}')

        try:
            u0, x_pred, info = self.mpc.solve(
                x0, x_ref, nb_traj,
                desired_distances=dd, dist=d_hat,
            )
        except Exception as e:
            self.get_logger().warn(f'MPC solve crashed: {e}; holding position')
            self._fallback_count += 1
            self._hover_active = True
            self._publish_health()
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), 0.0)
            return

        self._last_mpc_status = int(info['status'])
        self._last_solve_ms   = float(info['solve_time_s']) * 1000.0

        self._dbg_counter += 1
        if self._dbg_counter <= 5:
            self.get_logger().warn(
                f'DEBUG d{self.drone_id}: '
                f'x0=({x0[0]:.2f},{x0[1]:.2f},{x0[2]:.2f}) '
                f'ref0=({x_ref[0,0]:.2f},{x_ref[0,1]:.2f},{x_ref[0,2]:.2f}) '
                f'pred1=({x_pred[1,0]:.2f},{x_pred[1,1]:.2f},{x_pred[1,2]:.2f}) '
                f'leader=({self.leader_pos[0]:.2f},{self.leader_pos[1]:.2f},{self.leader_pos[2]:.2f})'
            )
        if self._dbg_counter % int(self.control_hz) == 0:
            wb = self.world_birth[self.drone_id]
            local_now = self_ds.pos - wb     # = PX4 原始 local（校准的逆运算）
            self.get_logger().info(
                f'[d{self.drone_id}] solve: status={info["status"]} '
                f'time={self._last_solve_ms:.2f}ms cost={info["cost"]:.1f} '
                f'pos=({self_ds.pos[0]:.2f},{self_ds.pos[1]:.2f},{self_ds.pos[2]:.2f}) '
                f'local=({local_now[0]:.2f},{local_now[1]:.2f},{local_now[2]:.2f}) '
                f'wbirth=({wb[0]:.2f},{wb[1]:.2f},{wb[2]:.2f}) '
                f'ref=({x_ref[0,0]:.2f},{x_ref[0,1]:.2f}) '
                f'leader=({self.leader_pos[0]:.2f},{self.leader_pos[1]:.2f}) '
                f'fallbacks={self._fallback_count}'
            )

        yaw_sp = self.leader_yaw if math.isfinite(self.leader_yaw) else 0.0

        # 0=成功, 2=达到最大迭代但解仍可用; 1=发散, 3=最小步长, 4=QP失败 → 位置保持
        if info['status'] not in (0, 2):
            self.get_logger().warn(
                f'[d{self.drone_id}] acados status={info["status"]} — holding position '
                f'(total fallbacks={self._fallback_count + 1})'
            )
            self._fallback_count += 1
            self._hover_active = True
            self._publish_health()
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_sp)
            return

        # ── Velocity control mode ──
        # Use predicted velocity at k=1 as velocity setpoint base.
        # z-axis: pure P-controller for altitude hold (decoupled from XY).
        pred_vel = x_pred[1, 3:6].copy()

        # DOB baseline：记录本拍命令加速度 u0 作为下一拍观测器的名义输入 u_nom。
        # legacy 下 u0 即命令加速度（模型 v̇=u）。仅有效解(status∈{0,2})时更新。
        if self.mpc._dob:
            self._last_u0 = np.asarray(u0, dtype=float).reshape(3)

        # Position error → velocity correction (P-controller)
        ref_pos = x_ref[0, 0:3]
        pos_err_vec = ref_pos - self_ds.pos
        Kp_pos = 1.0  # position gain (m/s per m of error)
        vel_correction = np.clip(pos_err_vec * Kp_pos, -self.max_speed, self.max_speed)

        # Blend: MPC velocity + position correction
        # 横向：MPC 优化速度前馈 + 候选①可选的饱和限幅位置 P。原先叠加 0.5·Kp·pos_err
        # 的外层「无限幅」比例环 → 僚机过冲、圆周震荡(1c4bc5a 删之)。这里改为有界 P：
        # vel_xy_kp=0 时退回纯前馈(等价 1c4bc5a)；>0 时纠偏被 vel_xy_cap 限幅，
        # 抓住密集 grid 运动中碰撞约束推离槽位的漂移、防发散，又不致无阻尼震荡。
        # 垂直：保留纯 P(高度保持，与横向解耦，不引起震荡)。
        vel_sp = np.zeros(3)
        if self.mpc.lag_model:
            # 接口一致：u0 就是速度设定点（模型的输入 = 实际下发量），直接下发。
            # 勿改回 x_pred[1]：在 τ 模型里那是"车将达到的速度"而非"该命令的速度"，
            # 拿它当设定点会系统性欠命令（§5.G 消融 C：峰峰只降到 10.4m，而 B 是 0.07m）。
            vel_sp[0] = u0[0]
            vel_sp[1] = u0[1]
        else:
            vel_sp[0] = pred_vel[0]
            vel_sp[1] = pred_vel[1]
        # 行动项③ 取证：在叠加 P 纠偏**之前**取范数——此时 vel_sp[:2] 就是 OCP 规划量
        # (lag: u0；legacy: x_pred[1])，其比值 >1 当且仅当逐轴箱允许了范数箱禁止的解。
        _plan_ratio = float(np.linalg.norm(vel_sp[:2])) / max(self.max_speed, 1e-9)
        if _plan_ratio > self._plan_ratio_max:
            self._plan_ratio_max = _plan_ratio

        if self.vel_xy_kp > 0.0:
            corr_xy = np.clip(pos_err_vec[:2] * self.vel_xy_kp,
                              -self.vel_xy_cap, self.vel_xy_cap)
            vel_sp[0] += corr_xy[0]
            vel_sp[1] += corr_xy[1]
        vel_sp[2] = vel_correction[2]  # pure P-controller for z (altitude hold)

        # Clip to limits
        vel_xy_norm = float(np.linalg.norm(vel_sp[:2]))
        if vel_xy_norm > self.max_speed:
            vel_sp[:2] *= self.max_speed / vel_xy_norm
            self._clip_count += 1
        vel_sp[2] = np.clip(vel_sp[2], -self.max_climb, self.max_climb)

        # ── 起飞期横向锁定（见 __init__ 里的长注释）────────────────────────
        # 放在限幅之后、安全滤波之前：此时 vel_sp 已是"本该下发的量"，置零最干净；
        # 也保证 _plan_ratio/_clip_count 这些取证量记的仍是 OCP 的真实规划，不被本层污染。
        # 闩锁本身由位置回调推进（见 _make_pos_callback 里的注释），这里只读结果。
        # 看门狗的**主驱动点在位置回调**（2026-09-01 挪过去的——控制回路依赖 leader，
        # 台架无 leader 时这里从未执行过）；此处保留一份属冗余兜底，幂等无害。
        self._check_takeoff_lock_watchdog()
        if self._takeoff_lock_enable and not self._airborne:
            vel_sp[0] = 0.0
            vel_sp[1] = 0.0
            if not self._takeoff_lock_logged:
                self._takeoff_lock_logged = True
                self.get_logger().info(
                    f'[d{self.drone_id}] 起飞期：横向速度指令锁零，'
                    f'升过 {self._takeoff_rise:.2f}m 后解锁（防地面积分饱和）')

        # Safety: NaN check
        if not np.all(np.isfinite(vel_sp)):
            vel_sp = np.zeros(3)

        # ── companion 安全滤波层：围栏/碰撞地板/估计门/失效状态机（下发前）──
        if self.safety is not None:
            now_s = now_ros.nanoseconds * 1e-9
            nbrs = [(self.drone_states[j].pos,
                     self.drone_states[j].received
                     and (now_s - self.drone_states[j].last_stamp) <= self.neighbour_timeout)
                    for j in self.neighbours]
            est_ok = (self_ds.xy_valid and self_ds.z_valid
                      and (now_s - self_ds.last_stamp) <= self._safety_self_timeout)
            sres = self.safety.step(self_ds.pos, self_ds.vel, vel_sp, ref_pos, dt,
                                    neighbours=nbrs, est_ok=est_ok)
            vel_sp = sres['vel_sp']
            if sres['state'] != 'NORMAL':
                self._hover_active = True
                if self._dbg_counter % int(self.control_hz) == 0:
                    self.get_logger().warn(
                        f"[d{self.drone_id}] SAFETY {sres['state']} {sres['reasons']}")
            if not sres['publish']:
                # RELINQUISH（方案b 主动交还）：直接命令 PX4 切 Hold，不依赖 COM_OF_LOSS_T 超时。
                # 永久闩锁（safety_filter 侧 RELINQUISH 本就 sticky），后续帧由 control_loop
                # 顶部短路：停心跳/setpoint、Hold 未确认前 1Hz 重发。
                self._relinquished = True
                self._relinquish_next_cmd = now_ros.nanoseconds * 1e-9 + 1.0
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)  # AUTO/LOITER=Hold
                self.get_logger().error(
                    f"[d{self.drone_id}] SAFETY RELINQUISH {sres['reasons']} — "
                    f"DO_SET_MODE(Hold) 主动交还 PX4，停流闩锁")
                self._publish_health()
                return

        # Position error for logging
        pos_err = float(np.linalg.norm(pos_err_vec[:2]))
        self._last_pos_err = pos_err
        self._last_z_err = abs(float(self_ds.pos[2] - self._eff_target_alt()))

        # ── 诊断：横向跟踪误差分解为 radial(径向)/tangential(切向)，1Hz ──
        # 切向 = leader 速度方向；径向 = 向心加速度反向(指向圆外)；圆周时二者正交。
        #   e_tan > 0 → 参考在前方，僚机滞后(切向滞后/相位问题)
        #   e_rad > 0 → 参考更靠外，僚机切内圈(径向/曲率/增益问题)
        # 看震荡主要落在哪个分量，即可定位机理(切向=速度前馈/相位；径向=曲率/增益)。
        if self._dbg_counter % int(self.control_hz) == 0:
            v_xy = self.leader_vel[:2]
            a_xy = self.leader_acc[:2]
            v_norm = float(np.linalg.norm(v_xy))
            a_norm = float(np.linalg.norm(a_xy))
            if v_norm > 1e-3 and a_norm > 1e-3:
                t_hat = v_xy / v_norm        # 切向(运动方向)
                r_hat = -a_xy / a_norm       # 径向向外(向心反向)
                e_tan = float(np.dot(pos_err_vec[:2], t_hat))
                e_rad = float(np.dot(pos_err_vec[:2], r_hat))
                self.get_logger().info(
                    f'[d{self.drone_id}] track-err total={pos_err:.3f}m '
                    f'radial={e_rad:+.3f} tangential={e_tan:+.3f} '
                    f'(|v|={v_norm:.2f} |a|={a_norm:.2f})'
                )

        self._publish_health()
        self.publish_velocity_setpoint(vel_sp, yaw_sp)
        self.publish_predicted_trajectory(x_pred[:, 0:3])

    def _consensus_step(self, self_ds, x_ref, dt, now_ros):
        """分布式 leader-follower 位移一致性编队律（论文 §6.5 非-MPC 对照基线）。

        v_cmd_i = v_leader + kL·((p_leader+off_i)−p_i) + kC·Σ_{j∈N_i}((p_j+off_i−off_j)−p_i)
        仅用本机+邻居信息(分布式)，输出速度设定点，过与 MPC 完全相同的限幅/安全滤波/发布
        路径；**无滞后补偿**(名义模型) ⇒ 检验速度环滞后致失效是否为一般分布式编队通病。
        """
        p_i = self_ds.pos
        ref_pos = x_ref[0, 0:3]
        yaw_sp = self.leader_yaw if math.isfinite(self.leader_yaw) else 0.0
        lp = self.leader_pos if np.all(np.isfinite(self.leader_pos)) else (p_i - self.my_offset)
        lv = self.leader_vel if np.all(np.isfinite(self.leader_vel)) else np.zeros(3)

        # leader 速度前馈 + leader/队形位置跟踪 + 邻居位移一致性
        vel_sp = lv.copy()
        vel_sp = vel_sp + self.consensus_kL * ((lp + self.my_offset) - p_i)
        now_s = now_ros.nanoseconds * 1e-9
        cons = np.zeros(3)
        for j in self.neighbours:
            dsj = self.drone_states[j]
            if dsj.received and (now_s - dsj.last_stamp) <= self.neighbour_timeout:
                rel = self.formation_offsets[self.drone_id] - self.formation_offsets[j]
                cons = cons + (dsj.pos + rel - p_i)
        vel_sp = vel_sp + self.consensus_kC * cons
        vel_sp[2] = np.clip((ref_pos[2] - p_i[2]) * 1.0, -self.max_climb, self.max_climb)

        pos_err_vec = ref_pos - p_i
        _plan_ratio = float(np.linalg.norm(vel_sp[:2])) / max(self.max_speed, 1e-9)
        if _plan_ratio > self._plan_ratio_max:
            self._plan_ratio_max = _plan_ratio

        # Clip to limits（与 MPC 路径同口径）
        vel_xy_norm = float(np.linalg.norm(vel_sp[:2]))
        if vel_xy_norm > self.max_speed:
            vel_sp[:2] *= self.max_speed / vel_xy_norm
            self._clip_count += 1
        vel_sp[2] = np.clip(vel_sp[2], -self.max_climb, self.max_climb)
        if not np.all(np.isfinite(vel_sp)):
            vel_sp = np.zeros(3)

        # 安全滤波层（与 MPC 路径相同）
        if self.safety is not None:
            nbrs = [(self.drone_states[j].pos,
                     self.drone_states[j].received
                     and (now_s - self.drone_states[j].last_stamp) <= self.neighbour_timeout)
                    for j in self.neighbours]
            est_ok = (self_ds.xy_valid and self_ds.z_valid
                      and (now_s - self_ds.last_stamp) <= self._safety_self_timeout)
            sres = self.safety.step(self_ds.pos, self_ds.vel, vel_sp, ref_pos, dt,
                                    neighbours=nbrs, est_ok=est_ok)
            vel_sp = sres['vel_sp']
            if sres['state'] != 'NORMAL':
                self._hover_active = True
                if self._dbg_counter % int(self.control_hz) == 0:
                    self.get_logger().warn(
                        f"[d{self.drone_id}] SAFETY {sres['state']} {sres['reasons']}")
            if not sres['publish']:
                self._relinquished = True
                self._relinquish_next_cmd = now_s + 1.0
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)  # AUTO/LOITER=Hold
                self._publish_health()
                return

        self._last_pos_err = float(np.linalg.norm(pos_err_vec[:2]))
        self._last_z_err = abs(float(self_ds.pos[2] - self._eff_target_alt()))
        self._dbg_counter += 1
        self._publish_health()
        self.publish_velocity_setpoint(vel_sp, yaw_sp)
        # 一致性用邻居**当前位置**(非预测)；仍发一条"当前位置"占位预测,免得其它节点误判预测过期。
        self.publish_predicted_trajectory(np.tile(self_ds.pos[0:3], (self.N + 1, 1)))

    def _build_reference_trajectory(self):
        N = self.N
        x_ref = np.zeros((N + 1, 6))
        leader_pos_safe = (self.leader_pos.copy()
                           if np.all(np.isfinite(self.leader_pos)) else np.zeros(3))
        leader_vel_safe = (self.leader_vel.copy()
                           if np.all(np.isfinite(self.leader_vel)) else np.zeros(3))
        leader_acc_safe = (self.leader_acc.copy()
                           if np.all(np.isfinite(self.leader_acc)) else np.zeros(3))
        for k in range(N + 1):
            t = k * self.mpc_dt
            # 二阶预测：pos + vel*t + 0.5*acc*t²（圆周运动时捕捉向心加速度）
            pos = leader_pos_safe + leader_vel_safe * t + 0.5 * leader_acc_safe * t * t + self.my_offset
            pos[2] = self._eff_target_alt()
            x_ref[k, 0:3] = pos
            # 速度前馈也随加速度变化：vel(t) = vel(0) + acc*t
            vel_t = leader_vel_safe[:2] + leader_acc_safe[:2] * t
            x_ref[k, 3:5] = vel_t
            x_ref[k, 5]   = 0.0
        return x_ref

    def _collect_neighbour_predictions(self):
        if not self.neighbours:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        self._drain_pred_delay_buf(now)
        # P2: blackout 进入/退出打点 —— 用于核对降级路径确实被触发过、且恢复了
        _bo = self._in_comms_blackout(now)
        if _bo != self._blackout_was_active:
            self._blackout_was_active = _bo
            self.get_logger().warn(
                f'[P2 BLACKOUT] drone {self.drone_id}: 邻居通信'
                f'{"中断开始" if _bo else "恢复"} @ t+{now - self._node_t0:.1f}s')
        N1 = self.N + 1
        out = np.zeros((len(self.neighbours), N1, 3))
        for idx, j in enumerate(self.neighbours):
            traj = self.peer_predictions.get(j)
            stamp = self.peer_prediction_stamps.get(j, 0.0)
            pred_fresh = (traj is not None and traj.shape[0] >= 1
                          and (now - stamp) <= self.neighbour_timeout)
            if pred_fresh:
                self._note_peer_degrade(j, 1)
                # 时间对齐：平移掉通信延迟对应的步数
                latency = now - stamp
                shift = min(int(round(latency / self.mpc_dt)), traj.shape[0] - 1)
                traj = traj[shift:]
                if traj.shape[0] >= N1:
                    out[idx] = traj[:N1]
                else:
                    pad = np.repeat(traj[-1:], N1 - traj.shape[0], axis=0)
                    out[idx] = np.vstack([traj, pad])
                continue
            ds = self.drone_states[j]
            if ds.received and (now - ds.last_stamp) <= self.neighbour_timeout:
                self._note_peer_degrade(j, 2)
                out[idx] = np.tile(ds.pos, (N1, 1))
            else:
                # 位置也过期。用**队形推断**还是**最后已知位置**，取决于对方 mpc 死没死：
                #   · health 还在心跳 → 它大概率仍在跟队 → 队形推断更准（正常飞行时
                #     最后已知位置会滞后：圆周 1m/s 断 3s 就差约 3m）
                #   · health 也停了 → 它可能已脱队(趴窝/失控) → **队形推断是系统性错误的**，
                #     而这个位置同时喂给 MPC 的碰撞项 ⇒ 规避基于错误位置工作。
                # 2026-07-22 S45 实测（killnode + blackout）：旧逻辑一律用队形推断，
                # min_spacing 掉到 1.245m、d_safe(1.5) 违规 158/239 帧；而同参数无断连的
                # S24 是 4.417m/0 违规。safety_filter 之所以能兜住(硬地板 1.2 零穿透)，
                # 正是因为它用的是 drone_states[j].pos —— 最后已知位置。
                _h = self.peer_health_stamps.get(j)
                _alive = _h is not None and (now - _h) <= self.neighbour_timeout
                if ds.received and not _alive:
                    self._note_peer_degrade(j, 4)
                    out[idx] = np.tile(ds.pos, (N1, 1))
                else:
                    self._note_peer_degrade(j, 3)
                    out[idx] = np.tile(self.drone_states[self.drone_id].pos + self.formation_offsets[j] - self.formation_offsets[self.drone_id], (N1, 1))
        # 周期汇总：**不能只靠退出时打印** —— run_one_trial.sh 用 pkill -9 收尾，
        # SIGKILL 不给 Python 执行 finally 的机会（2026-07-22 实测：退出汇总一条没有）。
        if now - self._degrade_last_report >= 60.0:
            self._degrade_last_report = now
            self._log_degrade_summary()
        return out

    def _log_degrade_summary(self):
        fr = self._peer_degrade_frames
        if not fr:
            return
        tot = sum(fr.values())
        self.get_logger().warn(
            f'[DEGRADE SUMMARY] drone {self.drone_id}: '
            + ' '.join(f'L{k}={fr.get(k, 0)}({100.0 * fr.get(k, 0) / tot:.1f}%)'
                       for k in (1, 2, 3, 4))
            + f'  累计 {tot} 帧·邻居')

    # 邻居数据降级级别：1=用其预测轨迹(正常) / 2=预测过期，用其当前位置常值外推
    # / 3=位置也过期，纯按队形推断其位置。
    # ⚠️ 3 级在**稳态编队**下几乎无害（各机确实就在队形位上，推断即真值），
    # 但编队机动中或某机已偏离时，推断会系统性偏离真实位置 —— 碰撞规避将基于
    # 错误的邻居位置工作。没有本打点，任何通信故障测试都无法证明降级到底走没走到。
    def _note_peer_degrade(self, j, level):
        self._peer_degrade_frames[level] = self._peer_degrade_frames.get(level, 0) + 1
        if self._peer_degrade_lvl.get(j) != level:
            prev = self._peer_degrade_lvl.get(j)
            self._peer_degrade_lvl[j] = level
            if prev is not None:          # 首帧不报，只报变化
                desc = {1: '恢复正常(用邻居预测)', 2: '预测过期→位置外推',
                        3: '位置也过期but health活→队形推断',
                        4: '位置过期且health停→用最后已知位置(防脱队误判)'}[level]
                self.get_logger().warn(
                    f'[DEGRADE] drone {self.drone_id} → peer {j}: L{prev}→L{level} {desc}')

    def _publish_health(self):
        """发布 MPC 健康诊断数据，供 diag_monitor.py 实时监控。
        格式: [drone_id, mpc_status, solve_ms, fallback_count, hover_active, pos_err_m,
               z_err_m, plan_ratio_max, clip_count, dhat_x, dhat_y, dhat_z, airborne]
        —— 纯增列，消费端按定长索引取值（index 9-11 = DOB baseline 的 d̂，dob 关时恒 0）

        index 12 = 起飞期横向锁定的闩锁状态（0=锁定中/未离地，1=已离地）。加这一列是为了
        让下次真机飞行**自动留证**：CSV 里能直接查锁了多久、何时释放、期间指令是否真为 0，
        不必靠现场人眼观察（首飞教训：diag_monitor 订阅缺 _v1 ⇒ arm/nav 全 0，
        复盘时最关键的列直接作废）。"""
        msg = Float32MultiArray()
        msg.data = [
            float(self.drone_id),
            float(self._last_mpc_status),
            float(self._last_solve_ms),
            float(self._fallback_count),
            float(1 if self._hover_active else 0),
            float(self._last_pos_err),
            float(self._last_z_err),   # index 6: 高度误差 |z - target_alt| (m)
            float(self._plan_ratio_max),  # index 7: §5.G③ max‖v_plan_xy‖/max_speed
            float(self._clip_count),      # index 8: §5.G③ 范数限幅触发帧数
            float(self._last_dhat[0]),    # index 9:  §6.5 DOB d̂_x (m/s²)
            float(self._last_dhat[1]),    # index 10: §6.5 DOB d̂_y
            float(self._last_dhat[2]),    # index 11: §6.5 DOB d̂_z
            float(1 if self._airborne else 0),  # index 12: 离地闩锁（0=起飞期锁 XY 中）
        ]
        self.pub_health.publish(msg)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True   # 速度控制模式：与 publish_velocity_setpoint 匹配
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(msg)

    def _hover_setpoint_world(self):
        """当前 XY 保持（有位置时）或出生点 XY，高度收敛到目标高度，均为世界系 NED。"""
        ds = self.drone_states[self.drone_id]
        if ds.received:
            return np.array([ds.pos[0], ds.pos[1], self._eff_target_alt()])
        birth = self.world_birth[self.drone_id]
        return np.array([birth[0], birth[1], self._eff_target_alt()])

    def publish_position_setpoint(self, pos_world_ned, vel_ff_world_ned, yaw):
        """位置闭环 + 速度前馈（PX4 官方推荐：position≠NaN时，velocity作前馈项）。
        pos_world_ned / vel_ff_world_ned 均为世界系 NED；内部自动转换到 PX4 本地系。"""
        my_birth = self.world_birth[self.drone_id]
        # 世界系 → PX4本地系（平移不影响速度方向，直接传速度即可）
        pos_local = pos_world_ned - my_birth
        nan3 = [float('nan')] * 3
        msg = TrajectorySetpoint()
        msg.position = [safe_finite(pos_local[0]), safe_finite(pos_local[1]), safe_finite(pos_local[2])]
        msg.velocity = [safe_finite(v, float('nan')) for v in vel_ff_world_ned]
        msg.acceleration = nan3
        msg.yaw = safe_finite(yaw, 0.0)
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_setpoint.publish(msg)

    def publish_velocity_setpoint(self, vel_world_ned, yaw):
        """保留兼容性；正常运行路径已不调用此方法。"""
        vx = safe_finite(vel_world_ned[0], 0.0)
        vy = safe_finite(vel_world_ned[1], 0.0)
        vz = safe_finite(vel_world_ned[2], 0.0)
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = safe_finite(yaw, 0.0)
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_setpoint.publish(msg)

    def publish_predicted_trajectory(self, pred_xyz):
        msg = Float64MultiArray()
        rows, cols = pred_xyz.shape
        msg.layout.dim = [
            MultiArrayDimension(label='rows', size=rows, stride=rows * cols),
            MultiArrayDimension(label='cols', size=cols, stride=cols),
        ]
        msg.layout.data_offset = 0
        msg.data = pred_xyz.astype(float).flatten().tolist()
        self.pub_predicted.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MpcControllerNode()
    # 多线程 executor：控制定时器(自己的回调组)与订阅(另一组)并行，
    # 保证 50Hz offboard 心跳/ setpoint 不被订阅回调拖延 → 不丢 offboard 信号。
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 退出汇总（仅正常退出时有效；SIGKILL 收尾时靠 _collect_neighbour_predictions
        # 里的 60s 周期汇总兜底）
        try:
            node._log_degrade_summary()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()