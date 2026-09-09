#!/usr/bin/env python3
"""
虚拟领队节点：发布 /leader/state (Float64MultiArray)
格式: [time, x, y, z, vx, vy, vz, yaw]

支持三种运动模式（通过 ROS2 参数配置）:
  hover  — 悬停在固定点（默认）
  circle — 匀速圆周运动
  line   — 沿 X 轴匀速直线飞行

yaw_mode 参数（运行时可切换）:
  fixed   — 固定起飞朝向（默认；取 drone0 实测 heading，非正北）
  center  — 朝向圆心（摄影/观测）
  tangent — 跟随飞行方向（仿生/展示）

运行时切换:
  ros2 param set /leader_node yaw_mode fixed
  ros2 param set /leader_node yaw_mode center
  ros2 param set /leader_node yaw_mode tangent
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String
from px4_msgs.msg import VehicleLocalPosition

# yaw 变化率上限 (rad/s)，防止机体抖动
MAX_YAW_RATE = math.radians(45.0)  # 45°/s


def _wrap_angle(a):
    """将角度归一化到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _limit_yaw_rate(current, target, max_rate, dt):
    """限幅 yaw 变化率，返回平滑过渡后的 yaw。"""
    diff = _wrap_angle(target - current)
    max_step = max_rate * dt
    if abs(diff) <= max_step:
        return target
    return current + max_step * (1.0 if diff > 0 else -1.0)


class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')

        self.declare_parameter('mode',       'hover')   # hover | circle | line
        self.declare_parameter('yaw_mode',   'fixed')   # fixed | center | tangent
        self.declare_parameter('start_x',     0.0)
        self.declare_parameter('start_y',     0.0)
        self.declare_parameter('altitude',   -5.0)      # NED：-5 = 离地5m
        self.declare_parameter('speed',       1.0)      # m/s
        self.declare_parameter('radius',     10.0)      # circle 半径 m
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('max_distance', 20.0)  # 直线模式最大飞行距离 (m)，到达后悬停
        # line 沿机头方向飞（2026-08-21）：默认 False = 旧行为（沿世界系 +X/正北），
        # 保持既有场景与论文回归不变；OUT_* 室外场景显式打开。
        self.declare_parameter('line_along_heading', False)
        self.declare_parameter('line_decel',   0.5)   # 直线终点前减速度 (m/s²)，平滑刹停防僚机过冲
        self.declare_parameter('circle_ramp_time', 5.0)  # 圆周缓启动 (s)：角速度 0→满速线性加速，消除从静止切入圆周的速度阶跃(防僚机震荡)；0=关闭
        self.declare_parameter('start_delay', 30.0)   # 起飞等待 (s)：leader 先原地不动，等僚机 ARM+爬升+组队（10s 太短，launch 默认同步为 30）
        # 闭环就绪门控：等各机进编队(pos_err<阈值)再开始运动，替代死等固定 start_delay
        self.declare_parameter('num_drones',        1)
        self.declare_parameter('ready_gate_enable', True)
        self.declare_parameter('ready_pos_err',     0.5)   # m，进编队判定阈值
        self.declare_parameter('ready_alt_err',     1.5)   # m，高度误差阈值（需接近目标高度）
        self.declare_parameter('ready_hold',        2.0)   # s，就绪需连续保持时长
        self.declare_parameter('ready_timeout',     90.0)  # s，超时兜底：仍未就绪也开动(告警)
        self.declare_parameter('health_timeout',    2.0)   # s，health 超此视为失联
        # 任务完成自动降落（第 1 层，2026-07-29）：运动开始后 mission_duration 秒广播 'LAND'
        # 到 /swarm/mission，各机收到后自主 settle→AUTO.LAND。0=禁用(永不自动降落，保持旧行为)。
        # 也可运行时 `ros2 param set /leader_node trigger_land true` 手动一键全体降落。
        self.declare_parameter('mission_duration', 0.0)    # s，从运动开始算；0=禁用
        self.declare_parameter('trigger_land',     False)  # 运行时置 true 立即广播 LAND
        # 固件消息版本化：'main'→vehicle_local_position_v1；v1.16/v1.14→无后缀版
        self.declare_parameter('px4_version', 'main')

        self._mode   = str(self.get_parameter('mode').value)
        self._x0     = float(self.get_parameter('start_x').value)
        self._y0     = float(self.get_parameter('start_y').value)
        self._alt    = float(self.get_parameter('altitude').value)
        self._speed  = float(self.get_parameter('speed').value)
        self._radius = float(self.get_parameter('radius').value)
        hz           = float(self.get_parameter('publish_hz').value)

        self._t  = 0.0
        self._dt = 1.0 / hz
        self._max_distance = float(self.get_parameter('max_distance').value)
        self._line_decel   = max(1e-3, float(self.get_parameter('line_decel').value))
        self._line_along_heading = bool(self.get_parameter('line_along_heading').value)
        self._circle_ramp_time = float(self.get_parameter('circle_ramp_time').value)
        self._start_delay = float(self.get_parameter('start_delay').value)
        self._initial_yaw = None   # 首次发布时记录
        self._current_yaw = 0.0    # 当前平滑后的 yaw（用于限幅）
        # 🔴 锁定航向时把 _current_yaw 一并对齐（2026-08-21，_lock_yaw 做这件事）：
        #    _current_yaw 初值 0.0 会被限速器当成"上一拍的朝向" ⇒ 启动瞬间先下发朝北的
        #    yaw 设定点，再按 MAX_YAW_RATE 花约 1s 摆到真实航向。飞机大概率跟不动，但
        #    方向恰好又是正北 —— 正是"yaw_mode=fixed 却朝北"要消除的那个行为。
        #    限速器是为**飞行中**防跳变而设，锁定瞬间并没有"上一拍"可平滑。

        # ── yaw_mode='fixed' 真正"保持起飞朝向"所需（2026-08-20 现场修）────────────
        # 🔴 旧实现的 bug：hover 模式下 leader 静止(vx=vy=0) ⇒ fixed 分支落到
        #    `self._initial_yaw = 0.0` = **正北**，而文档字符串写的是"固定起飞朝向"。
        #    实测后果：飞机起飞时机头朝 ~145°，一进 OFFBOARD 就左转 145° 去朝北——
        #    看着像失控自旋。hover 下无害(速度指令≈0)，但 line/circle 会把世界系速度
        #    指令转到错误方向。
        # ✅ 改为订阅 drone0 的 heading，用**飞机实际朝向**当 fixed 的目标。
        #    拿不到时不锁定(返回当前 yaw，等)；超时才回落 0.0 并告警——
        #    避免"没收到就静默锁 0"重蹈覆辙。
        self._observed_heading = None
        self._yaw_wait_t = 0.0
        self._YAW_WAIT_MAX = 5.0     # 秒；超时回落正北并告警
        hdg_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        _pxv = str(self.get_parameter('px4_version').value).strip()
        _lpt = ('vehicle_local_position_v1' if _pxv.startswith('main')
                else 'vehicle_local_position')
        self.create_subscription(
            VehicleLocalPosition, f'/fmu/out/{_lpt}',
            self._on_drone0_pos, hdg_qos)

        # line 模式：对准阶段
        self._line_align_done = False
        self._line_start_t = 0.0    # 对准完成、开始平移时的运动时钟
        self._hold_logged = False   # 起飞等待提示只打一次

        # 就绪门控状态
        self._num_drones        = int(self.get_parameter('num_drones').value)
        self._ready_gate_enable = bool(self.get_parameter('ready_gate_enable').value)
        self._ready_pos_err     = float(self.get_parameter('ready_pos_err').value)
        self._ready_alt_err     = float(self.get_parameter('ready_alt_err').value)
        self._ready_hold        = float(self.get_parameter('ready_hold').value)
        self._ready_timeout     = float(self.get_parameter('ready_timeout').value)
        self._health_timeout    = float(self.get_parameter('health_timeout').value)
        self._health_pos_err = [None] * max(1, self._num_drones)
        self._health_z_err   = [None] * max(1, self._num_drones)
        self._health_stamp   = [None] * max(1, self._num_drones)
        self._motion_started = False
        self._motion_start_t = 0.0
        self._ready_since    = None
        self._timeout_warned = False

        # 任务完成降落状态
        self._mission_duration = max(0.0, float(self.get_parameter('mission_duration').value))
        self._land_triggered   = False
        # ── 任务时钟：mission_duration 的计时基准（2026-08-21 修）──────────────
        # 🔴 首飞 bag 分析查出：原实现拿 _motion_start_t 当基准，而 hover 模式的
        #    _should_start_motion() 直接跳过就绪门控（return t >= start_delay）
        #    ⇒ 时钟在 leader 节点起来几秒后就走，**与飞机是否解锁/离地无关**。
        #    首飞时间线（leader t=0 起、飞手 t=271s 才切 OFFBOARD）下传
        #    mission_duration:=60 会在**起飞前 211 秒**广播 LAND；而 _land_triggered
        #    是 sticky、LAND 用 latched QoS 重播 ⇒ **一进 OFFBOARD 就被要求降落**。
        #    line/circle 虽走真门控，但 ready_timeout(默认 90s) 兜底分支同样会在
        #    没起飞时把时钟启动 —— 同一个坑，只是浅一点。
        # ✅ 拆成独立时钟，基准 = 全员真正到位（_all_formed_up(require_alt=True)：
        #    health 新鲜 + XY pos_err 小 + z_err ≤ ready_alt_err ⇒ 已爬到目标高度
        #    附近 = 已离地），连续保持 ready_hold 才起表。
        # 🔑 **刻意不给超时兜底**：没就绪就永不自动降落，这是安全的失效方向
        #    （上面还有第 2–4 层失控保护和飞手）；而"在地上开始倒计时"没有任何
        #    安全的解释。代价是 mission_duration 在 health 断流时不生效 —— 已在
        #    下方按 5s 节流打 warn，不会静默。
        self._mission_clock_started = False
        self._mission_clock_t0      = 0.0
        self._mission_ready_since   = None   # 独立于 _ready_since，避免与运动门控互相踩
        self._mission_warn_last     = -1e9
        self._last_x = self._x0   # 最近一次发布的 leader 位置（触发降落时冻结用）
        self._last_y = self._y0
        self._land_x = self._x0   # 触发降落时冻结的 leader 位置（发零速悬停参考）
        self._land_y = self._y0
        self._land_last_pub = -1.0  # 上次重播 LAND 的时刻（1Hz 兜底重播）

        self._pub = self.create_publisher(Float64MultiArray, '/leader/state', 10)
        # 降落指令：latched(TRANSIENT_LOCAL)+RELIABLE，晚加入/短暂掉线的机也能收到最后指令
        mission_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mission_pub = self.create_publisher(String, '/swarm/mission', mission_qos)
        # 订阅各机 MPC health（pos_err=data[5]）用于就绪门控
        for i in range(self._num_drones):
            topic = '/mpc/health' if i == 0 else f'/px4_{i}/mpc/health'
            self.create_subscription(
                Float32MultiArray, topic, self._make_health_cb(i), 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'Leader ready: mode={self._mode}, yaw_mode={self.get_parameter("yaw_mode").value}, '
            f'alt={self._alt}m (NED), speed={self._speed}m/s, radius={self._radius}m'
        )

    def _lock_yaw(self, psi):
        """锁定初始航向，并把限速器的"上一拍"一并对齐（见 __init__ 注释）。"""
        self._initial_yaw = float(psi)
        self._current_yaw = float(psi)
        return self._initial_yaw

    def _locked_heading(self):
        """返回锁定的起飞航向 psi（rad），还拿不到则 None。

        与 yaw_mode='fixed' 用同一个来源（drone0 实测 heading），保证"飞行方向"
        与"机头指向"是同一个角，不会各锁各的。
        """
        if self._initial_yaw is not None:
            return self._initial_yaw
        if self._observed_heading is not None:
            return self._lock_yaw(self._observed_heading)
        self._yaw_wait_t += self._dt
        if self._yaw_wait_t >= self._YAW_WAIT_MAX:
            self._lock_yaw(0.0)
            self.get_logger().warn(
                f'line 沿机头方向：{self._YAW_WAIT_MAX:.0f}s 内未收到 drone0 heading，'
                f'回落正北(0°)。请检查 /fmu/out/vehicle_local_position 是否在发布')
            return self._initial_yaw
        return None

    def _line_profile(self, tau):
        """直线段梯形速度曲线：巡航 → 终点前按 line_decel 平滑刹停。返回 (位移, 速度, 加速度)。

        旧版到点 vx 从 speed 直接跳 0，速度阶跃致僚机过冲/来回弹。
        抽成方法供"沿 X 轴"与"沿机头"两条分支共用，避免两份实现日后漂移。
        """
        spd   = abs(self._speed)
        s_max = self._max_distance
        a_dec = self._line_decel
        d_brake = min(s_max, spd * spd / (2.0 * a_dec))
        t_brake = max(0.0, (s_max - d_brake) / max(spd, 1e-6))
        t_decel = spd / a_dec
        if tau <= t_brake:
            d, v, a = spd * tau, spd, 0.0
        elif tau <= t_brake + t_decel:
            td = tau - t_brake
            d = (s_max - d_brake) + spd * td - 0.5 * a_dec * td * td
            v = spd - a_dec * td
            a = -a_dec
        else:
            d, v, a = s_max, 0.0, 0.0
        return max(0.0, min(d, s_max)), v, a

    def _compute_raw_yaw(self, x, y, vx, vy):
        """根据 yaw_mode 计算目标偏航角（无限幅）。"""
        yaw_mode = str(self.get_parameter('yaw_mode').value)

        if yaw_mode == 'center':
            cx = self._x0
            cy = self._y0
            return math.atan2(cy - y, cx - x)
        elif yaw_mode == 'tangent':
            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                return self._current_yaw
            return math.atan2(vy, vx)
        else:  # fixed —— 固定起飞朝向
            if self._initial_yaw is None:
                if abs(vx) > 1e-6 or abs(vy) > 1e-6:
                    # 起步就有速度（line/circle 从运动中接管）：用运动方向
                    self._lock_yaw(math.atan2(vy, vx))
                elif self._observed_heading is not None:
                    # ✅ 静止（hover）：用**飞机实际朝向**，这才是"固定起飞朝向"
                    self._lock_yaw(self._observed_heading)
                    self.get_logger().info(
                        f'yaw_mode=fixed: 锁定起飞朝向 '
                        f'{math.degrees(self._initial_yaw):.1f}°（取自 drone0 实测 heading）')
                else:
                    # 还没收到 heading —— **不锁定**，返回当前值继续等
                    self._yaw_wait_t += self._dt
                    if self._yaw_wait_t < self._YAW_WAIT_MAX:
                        return self._current_yaw
                    self._lock_yaw(0.0)
                    self.get_logger().warn(
                        f'yaw_mode=fixed: {self._YAW_WAIT_MAX:.0f}s 内未收到 drone0 heading，'
                        f'回落正北(0°)。飞机会转向正北——若非预期，检查 '
                        f'/fmu/out/vehicle_local_position 是否在发布')
            return self._initial_yaw

    def _on_drone0_pos(self, msg):
        """记录 drone0 的实测航向，供 yaw_mode='fixed' 锁定起飞朝向用。"""
        if math.isfinite(msg.heading):
            self._observed_heading = float(msg.heading)

    def _apply_yaw_limits(self, raw_yaw):
        """对目标 yaw 施加变化率限幅，返回平滑后的 yaw。"""
        smoothed = _limit_yaw_rate(self._current_yaw, raw_yaw, MAX_YAW_RATE, self._dt)
        self._current_yaw = smoothed
        return smoothed

    def _make_health_cb(self, idx):
        def cb(msg):
            if len(msg.data) >= 6:
                self._health_pos_err[idx] = float(msg.data[5])
                self._health_stamp[idx] = self.get_clock().now().nanoseconds * 1e-9
            if len(msg.data) >= 7:
                self._health_z_err[idx] = float(msg.data[6])
        return cb

    def _all_formed_up(self, require_alt=False):
        """所有机 health 新鲜、XY pos_err < 阈值、且高度已到位 → 编队已组好。

        require_alt=True 时高度检查变为**强制**：z_err 缺失(旧 mpc_node 不发 index6)
        直接判未就绪。任务时钟用这个严格口径——宁可不自动降落，也不能在地上起表。
        """
        if self._num_drones <= 0:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        for i in range(self._num_drones):
            stamp = self._health_stamp[i]
            err   = self._health_pos_err[i]
            if stamp is None or (now - stamp) > self._health_timeout:
                return False
            if err is None or err > self._ready_pos_err:
                return False
            # 高度检查：要求 drone 已飞到目标高度附近（旧 mpc_node 不发 index6 则跳过）
            z_err = self._health_z_err[i]
            if z_err is None:
                if require_alt:
                    return False          # 严格口径：拿不到高度就不认为已离地
            elif z_err > self._ready_alt_err:
                return False
        return True

    def _should_start_motion(self, t):
        # hover 不动、或门控关闭：退回旧的固定 start_delay 行为
        if self._mode == 'hover' or not self._ready_gate_enable:
            return t >= self._start_delay
        # 闭环就绪：全员组好并连续保持 ready_hold
        if self._all_formed_up():
            if self._ready_since is None:
                self._ready_since = t
                self.get_logger().info(
                    f'all drones formed up (pos_err < {self._ready_pos_err:.2f}m), '
                    f'confirming for {self._ready_hold:.1f}s...')
            elif t - self._ready_since >= self._ready_hold:
                self.get_logger().info(
                    f'formation ready — starting leader motion at t={t:.1f}s')
                return True
        else:
            self._ready_since = None
        # 超时兜底：太久没组好也开动，但大声告警（别静默卡住）
        if t >= self._ready_timeout:
            if not self._timeout_warned:
                self.get_logger().error(
                    f'readiness TIMEOUT at t={t:.1f}s — not all drones formed up; '
                    'starting motion anyway, CHECK STRAGGLERS')
                self._timeout_warned = True
            return True
        return False

    def _update_mission_clock(self, t):
        """推进任务时钟：全员真正到位并保持 ready_hold 后起表。无超时兜底（见 __init__ 注释）。"""
        if self._mission_clock_started or self._mission_duration <= 0.0:
            return
        if self._all_formed_up(require_alt=True):
            if self._mission_ready_since is None:
                self._mission_ready_since = t
            elif t - self._mission_ready_since >= self._ready_hold:
                self._mission_clock_started = True
                self._mission_clock_t0 = t
                self.get_logger().warn(
                    f'任务时钟起表 t={t:.1f}s（全员已到目标高度）—— '
                    f'{self._mission_duration:.0f}s 后广播 LAND 自动降落')
        else:
            self._mission_ready_since = None
            # 别静默：mission_duration 配了却迟迟不起表时，每 5s 说一次为什么
            if t - self._mission_warn_last >= 5.0:
                self._mission_warn_last = t
                self.get_logger().info(
                    f'任务时钟未起表（等全员到位；mission_duration='
                    f'{self._mission_duration:.0f}s 在起表后才开始计）')

    def _trigger_land(self, t, reason):
        """置位任务降落：冻结 leader 到当前位置，标记开始周期广播 LAND。"""
        if self._land_triggered:
            return
        self._land_triggered = True
        self._land_x, self._land_y = self._last_x, self._last_y
        self._land_last_pub = -1.0   # 强制本 tick 立即首播
        self.get_logger().warn(
            f'MISSION COMPLETE ({reason}) at t={t:.1f}s — 广播 LAND 给所有僚机，leader 冻结悬停')

    def _publish_land_frozen(self, t):
        """降落后 leader 的行为：1Hz 重播 LAND（latched 之外再兜底），并冻结发零速悬停参考。"""
        if t - self._land_last_pub >= 1.0:
            self._land_last_pub = t
            m = String()
            m.data = 'LAND'
            self._mission_pub.publish(m)
        # 冻结参考：当前位置 + 零速（僚机已各自进入降落，此参考被忽略，仅保活/防跳变）
        msg = Float64MultiArray()
        msg.data = [float(t), self._land_x, self._land_y, self._alt,
                    0.0, 0.0, 0.0, self._current_yaw, 0.0, 0.0]
        self._pub.publish(msg)

    def _tick(self):
        t = self._t
        self._t += self._dt
        # 读取运行时参数（支持 ros2 param set 动态切换）
        self._mode = str(self.get_parameter('mode').value)

        # 任务完成降落（第 1 层）：已触发 → 冻结+重播 LAND；否则检查手动/时间触发条件。
        if self._land_triggered:
            self._publish_land_frozen(t)
            return
        if bool(self.get_parameter('trigger_land').value):
            self._trigger_land(t, 'manual trigger_land param')
            self._publish_land_frozen(t)
            return
        self._update_mission_clock(t)
        if (self._mission_clock_started
                and (t - self._mission_clock_t0) >= self._mission_duration):
            self._trigger_land(t, f'mission_duration {self._mission_duration:.0f}s reached')
            self._publish_land_frozen(t)
            return

        # 运动起始门控：等编队组好(各机 pos_err 小)再开始运动。
        # 死等固定 start_delay 在 5/9 机会猜错(短了僚机边爬边追、长了干等)，
        # 改为订阅各机 health.pos_err 的闭环就绪门控，带超时兜底。
        if not self._motion_started:
            if self._should_start_motion(t):
                self._motion_started = True
                self._motion_start_t = t
            else:
                x, y = self._x0, self._y0
                vx, vy = 0.0, 0.0
                ax, ay = 0.0, 0.0
                yaw = self._apply_yaw_limits(self._compute_raw_yaw(x, y, vx, vy))
                self._last_x, self._last_y = x, y
                msg = Float64MultiArray()
                msg.data = [float(t), x, y, self._alt, vx, vy, 0.0, yaw, ax, ay]
                self._pub.publish(msg)
                return

        t_move = t - self._motion_start_t

        if self._mode == 'circle':
            omega_max = self._speed / max(self._radius, 0.1)
            # 圆心在出生点正西，使 t_move=0 时 leader 正好在出生点 (x0,y0)
            # 避免从 hold 切到 circle 时的 10m 位置阶跃
            cx = self._x0 - self._radius
            cy = self._y0
            # 缓启动：角速度 ω(τ) 前 T_ramp 秒从 0 线性增到 ω_max，相位 φ=∫ω dτ。
            # 消除速度阶跃 → 僚机平滑切入圆周，不被突然的满速前馈甩出。
            T = self._circle_ramp_time
            tau = t_move
            if T > 1e-3 and tau < T:
                omega     = omega_max * (tau / T)
                phi       = omega_max * tau * tau / (2.0 * T)
                omega_dot = omega_max / T
            else:
                omega     = omega_max
                phi       = omega_max * (tau - (0.5 * T if T > 1e-3 else 0.0))
                omega_dot = 0.0
            x   = cx + self._radius * math.cos(phi)
            y   = cy + self._radius * math.sin(phi)
            vx  = -self._radius * omega * math.sin(phi)
            vy  =  self._radius * omega * math.cos(phi)
            # 加速度 = 向心(-ω²r) + 切向(r·ω̇)；切向项在 ramp 期补偿角加速度
            ax  = -self._radius * (omega_dot * math.sin(phi) + omega * omega * math.cos(phi))
            ay  =  self._radius * (omega_dot * math.cos(phi) - omega * omega * math.sin(phi))
            raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
            yaw = self._apply_yaw_limits(raw_yaw)

        elif self._mode == 'line':
            vx = self._speed
            vy = 0.0
            ax, ay = 0.0, 0.0

            # ── 沿机头方向飞（2026-08-21）───────────────────────────────────
            # 🔴 旧行为：line 沿**世界系 +X（正北）**平移（x=x0+d、y 恒定），与机头无关；
            #    且对准阶段用 vx=speed 调 _compute_raw_yaw ⇒ yaw_mode='fixed' 会走
            #    "起步就有速度就用运动方向"分支、锁成 atan2(0,speed)=**正北**。
            #    ⇒ 飞机一进 OFFBOARD 先转到朝北再飞 —— 正是 hover 那次"像失控自旋"的
            #    同一个坑，8-20 那次修复只覆盖了 hover，没覆盖 line。
            # ✅ 打开本开关后：不做任何对准转向，直接沿**锁定的起飞航向** psi 平移，
            #    yaw 全程等于 psi（机头朝向不变）。
            if self._line_along_heading:
                psi = self._locked_heading()
                if psi is None:              # 还没拿到航向：原地不动地等（别猜正北）
                    x, y = self._x0, self._y0
                    vx = vy = ax = ay = 0.0
                    yaw = self._apply_yaw_limits(self._current_yaw)
                else:
                    if not self._line_align_done:
                        self._line_align_done = True
                        self._line_start_t = t_move
                        self.get_logger().info(
                            f'line 沿机头方向：航向锁定 {math.degrees(psi):.1f}°，'
                            f'不做对准转向，直接前飞 {self._max_distance:.1f}m')
                    d, v, a = self._line_profile(t_move - self._line_start_t)
                    c, sn = math.cos(psi), math.sin(psi)
                    x  = self._x0 + c * d
                    y  = self._y0 + sn * d
                    vx, vy = c * v, sn * v
                    ax, ay = c * a, sn * a
                    yaw = self._apply_yaw_limits(psi)
            # Phase 1: 对准阶段 — 先转 yaw，再开始移动
            elif not self._line_align_done:
                x, y = self._x0, self._y0
                target_yaw = self._compute_raw_yaw(x, y, vx, vy)
                yaw = self._apply_yaw_limits(target_yaw)
                if abs(_wrap_angle(yaw - target_yaw)) < math.radians(2.0):
                    self._line_align_done = True
                    self._line_start_t = t_move   # 记录开始平移时刻
                    self.get_logger().info(f'Yaw aligned to {math.degrees(yaw):.1f}°, starting line motion')
                # 对准阶段不移动
                vx, vy = 0.0, 0.0
            else:
                # 梯形速度曲线见 _line_profile（与"沿机头"分支共用同一实现）
                sgn   = 1.0 if self._speed >= 0 else -1.0
                d, v, a = self._line_profile(t_move - self._line_start_t)
                x  = self._x0 + sgn * d
                y  = self._y0
                vx, vy = sgn * v, 0.0
                ax, ay = sgn * a, 0.0
                raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
                yaw = self._apply_yaw_limits(raw_yaw)

        else:  # hover
            x, y   = self._x0, self._y0
            vx, vy = 0.0, 0.0
            ax, ay = 0.0, 0.0
            raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
            yaw = self._apply_yaw_limits(raw_yaw)

        self._last_x, self._last_y = x, y
        msg = Float64MultiArray()
        msg.data = [float(t), x, y, self._alt, vx, vy, 0.0, yaw, ax, ay]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
