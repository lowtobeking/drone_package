#!/usr/bin/env python3
"""启动 MPC 编队。队形几何 / 参数 / 测试工况统一来自 config/scenarios.yaml
（单一真值源；与 start_N 经 tools/gen_spawn.py 读同一份 birth，杜绝漂移）。

两种用法:
  # 1) 按队形（向后兼容旧命令）——几何取 scenarios.yaml 的 formations.<name>
  ros2 launch mpc_control swarm_launch.py formation:=cross5 \
       leader_mode:=line leader_speed:=1.0 max_distance:=20.0

  # 2) 按命名工况——formation/leader 默认取该工况，仍可用 key:=value 覆盖
  ros2 launch mpc_control swarm_launch.py scenario:=S4_cross5_line
  ros2 launch mpc_control swarm_launch.py scenario:=S11_cross5_perturbed   # 扰动出生

参数优先级（高→低）: 显式 key:=value 启动参数  >  scenario 设定  >  内置默认。

⚠️ 改了 config/scenarios.yaml 后需 `colcon build`（launch 读的是 install 下的安装副本；
   start_N 走 gen_spawn 读源码副本，build 后两者一致）。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# leader 在既无启动参数、又无 scenario 设定时的内置默认（与旧 DeclareLaunchArgument 一致）
_LEADER_FALLBACK = dict(mode='hover', speed=0.5, radius=10.0,
                        max_distance=20.0, yaw_mode='fixed', start_delay=30.0,
                        mission_duration=0.0)  # 0=禁用自动降落（保持旧行为）


def _load_cfg():
    path = os.path.join(get_package_share_directory('mpc_control'),
                        'config', 'scenarios.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _flatten(nested):
    """[[N,E,D],...] → 扁平 float 列表（mpc_node 的 *_flat 是 DOUBLE_ARRAY，必须 float）。"""
    return [float(v) for pt in nested for v in pt]


def _make_nodes(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    cfg = _load_cfg()
    formations = cfg['formations']
    scenarios = cfg.get('scenarios', {})

    # ── 选 scenario / formation ──────────────────────────────────────────────
    scen_name = arg('scenario')
    if scen_name:
        if scen_name not in scenarios:
            raise ValueError(f'未知 scenario "{scen_name}"，可选: {list(scenarios)}')
        scen = scenarios[scen_name]
        formation = scen['formation']
    else:
        scen = {}
        formation = arg('formation')
    if formation not in formations:
        raise ValueError(f'未知 formation "{formation}"，可选: {list(formations)}')
    fm = formations[formation]

    # ── 几何：birth / offsets / neighbours（scenario 可覆盖 birth/scale）──────
    births = scen.get('birth_override', fm['birth'])
    offsets = fm.get('offsets', fm['birth'])
    scale = scen.get('offsets_scale')
    if scale:
        offsets = [[v * float(scale) for v in pt] for pt in offsets]
    neighbours = fm['neighbours']
    num = len(births)
    birth_flat = _flatten(births)
    offsets_flat = _flatten(offsets)

    # ── MPC 公共参数：defaults + formation 默认 + scenario.limits 覆盖 ─────────
    common = dict(cfg.get('defaults', {}))

    # ── 3+ 机高度拉齐：开 Tier2 alt re-sync 并加快收敛（先设，limits 可覆盖）──
    # SITL 多 PX4 实例 baro/EKF 的 ref_alt 各自温漂，不拉齐则各机控到 local z=-5
    # 时真实高度散 ~1.5m。drone0 广播 ref_alt 基准、各机限速纠 world_birth_z。
    # rate 0.05→0.2(补 0.5m 偏差 10s→2.5s)、EMA alpha 0.05→0.1(滤波 0.4s→0.2s)。
    # 实测 5 机悬停真高散 1.5m→0.18m。grid16/grid25 机数更多、跨度更大 → 一并开启。
    # trio3 于 2026-07-14 加入：原"2/3 机漂移小、不开"被 Gazebo 真值旁路监视证伪——
    # S33(trio3 circle) 物理机间高差均值 1.04m/峰值 1.30m(d2 EKF 偏差 ~1.1m 遥测盲视)，
    # 同批开拉齐的 cross5 稳定 0.17-0.19m(zwatch_20260714.log)。pair2 无实测证据暂不动。
    if formation in ('trio3', 'cross5', 'star5', 'grid9', 'grid16', 'grid25'):
        common['alt_resync_enable'] = True
        common['alt_resync_rate'] = 0.2
        common['alt_ref_filter_alpha'] = 0.1

    # scenario.limits 最高优先级（后设，可覆盖 formation 默认值）
    # bool 项（如 alt_resync_enable）保留 bool，否则 ROS2 参数类型不匹配；其余转 float
    for k, v in scen.get('limits', {}).items():
        common[k] = v if isinstance(v, bool) else float(v)

    # ── 显式启动参数覆盖 MPC 模型开关（优先级最高，压过 scenario.limits）─────────
    # 用途：全回归的"候选配置"整批扫描（如 vel_lag_tau:=0.5 soc_norm_enable:=true），
    # 不必去改 defaults: —— 改那里会把所有场景的历史标定一起改掉。
    if arg('vel_lag_tau'):
        common['vel_lag_tau'] = float(arg('vel_lag_tau'))
    # 起飞期横向锁定（2026-08-21）：节点声明了参数但此前无任何 launch 透传 ⇒ 永远是默认值、
    # 也做不了 A/B。与 mission_duration 当初的缺口同类。空=不覆盖，用节点默认。
    # 半自主档（2026-08-30）：SITL 里验证「解锁=授权起飞」用。auto_arm 此前 SITL
    # 从不透传（默认 true 自动解锁），为测本档补上；空=不覆盖、行为不变。
    if arg('auto_arm'):
        common['auto_arm_enable'] = arg('auto_arm').lower() in ('1', 'true', 'yes', 'on')
    if arg('offboard_on_arm'):
        common['offboard_on_arm'] = arg('offboard_on_arm').lower() in ('1', 'true', 'yes', 'on')
    if arg('takeoff_xy_lock_enable'):
        common['takeoff_xy_lock_enable'] = arg('takeoff_xy_lock_enable').lower() in ('1', 'true', 'yes', 'on')
    if arg('takeoff_rise_m'):
        common['takeoff_rise_m'] = float(arg('takeoff_rise_m'))
    if arg('takeoff_climb_ms'):
        common['takeoff_climb_ms'] = float(arg('takeoff_climb_ms'))
    if arg('takeoff_climb_hold_s'):
        common['takeoff_climb_hold_s'] = float(arg('takeoff_climb_hold_s'))
    if arg('soc_norm_enable'):
        # 必须落成 bool：mpc_node 那侧 declare_parameter 的默认是 False，
        # 传字符串/浮点会 ROS2 参数类型不匹配。
        common['soc_norm_enable'] = arg('soc_norm_enable').lower() in ('1', 'true', 'yes', 'on')
    # 论文 §6.5 审稿意见 M4：队形权重 w_f 扫描（直接检验"队形代价放大"）。
    # 覆盖 defaults/scenario 的 w_formation，其余一切不变。改 w_f 会改代价表达式 →
    # acados 指纹自动重建（无需手动清缓存）。
    if arg('w_formation'):
        common['w_formation'] = float(arg('w_formation'))
    # 论文 §6.5 审稿意见 M2：DOB-MPC 对照 baseline（在 legacy 零滞后模型上叠加性 d̂ 前馈）。
    if arg('dob_enable'):
        common['dob_enable'] = arg('dob_enable').lower() in ('1', 'true', 'yes', 'on')
    if arg('dob_dmax'):     # d̂ 上限：0.5=保守档(≈legacy)、2.0=激进档(失稳)，两档都报
        common['dob_dmax'] = float(arg('dob_dmax'))
    # 论文 §6.5 分布式一致性编队算法对照：controller:=consensus 切换到 consensus 律。
    if arg('controller'):
        common['controller'] = arg('controller')
    if arg('consensus_kL'):
        common['consensus_kL'] = float(arg('consensus_kL'))
    if arg('consensus_kC'):
        common['consensus_kC'] = float(arg('consensus_kC'))

    # 动捕/外部定位：EKF 的 local 原点是**全场共享**的，不是各机自身出生点
    # ⇒ world_birth 校准的门控判据要换（详见 mpc_node 的 calib_shared_origin）。
    # 做成 launch 参数而非写进 IN_* 场景：同一个场景在普通 SITL(GPS，各机独立
    # 原点)下也要能跑，焊死进场景会让 GPS 模式用错判据。
    if arg('calib_shared_origin'):
        common['calib_shared_origin'] = arg('calib_shared_origin').lower() in ('1', 'true', 'yes', 'on')

    # XY 全局对齐（RTK 真机编队）：用各机 EKF 原点 GPS 经纬度相对 drone0 datum 投影，
    # 让 cm 精度自动对齐各机世界系，替代盲信配置 birth_xy。留空=节点默认 False（回归/动捕不变）。
    if arg('xy_global_align_enable'):
        common['xy_global_align_enable'] = arg('xy_global_align_enable').lower() in ('1', 'true', 'yes', 'on')

    # 第 2 层失联降落（companion 看门狗）：SITL 默认关（留空→节点默认 0），
    # 测试时传 comms_loss_land_s:=5.0 打开，用于验证"OFFBOARD 中 leader 停 N 秒→自动降落"。
    # 真机走 real_hardware_launch（那边默认 5.0）。
    if arg('comms_loss_land_s'):
        common['comms_loss_land_s'] = float(arg('comms_loss_land_s'))
    if arg('comms_loss_hold_s'):
        common['comms_loss_hold_s'] = float(arg('comms_loss_hold_s'))

    # ── P2 故障注入：scenario.faults（S14 杀节点 / S16 通信劣化）──────────────
    faults = scen.get('faults', {})
    if 'comms_delay_ms' in faults:
        common['comms_delay_ms'] = float(faults['comms_delay_ms'])
    if 'comms_dropout' in faults:
        common['comms_dropout'] = float(faults['comms_dropout'])
    # 突发中断（S42/S43）。⚠️ 这里是**显式白名单**：新增 fault 类型时漏加一项，
    # 场景会照跑但什么都没注入（静默无效），务必同步。
    for _k in ('comms_blackout_duration_s', 'comms_blackout_period_s',
               'comms_blackout_start_s'):
        if _k in faults:
            common[_k] = float(faults[_k])
    if 'comms_blackout_scope' in faults:      # 字符串，不能走上面的 float()
        common['comms_blackout_scope'] = str(faults['comms_blackout_scope'])

    # ── leader 参数：显式启动参数(非空) > scenario.leader > 内置默认 ──────────
    scen_leader = scen.get('leader', {})

    def resolve(arg_name, key, cast):
        a = arg(arg_name)
        if a != '':
            return cast(a)
        if key in scen_leader:
            return cast(scen_leader[key])
        return _LEADER_FALLBACK[key]

    leader_params = {
        'mode':         resolve('leader_mode', 'mode', str),
        'yaw_mode':     resolve('yaw_mode', 'yaw_mode', str),
        'start_x':      0.0,
        'start_y':      0.0,
        'altitude':     float(common['target_alt']),
        'speed':        resolve('leader_speed', 'speed', float),
        'radius':       resolve('leader_radius', 'radius', float),
        'max_distance': resolve('max_distance', 'max_distance', float),
        'start_delay':  resolve('leader_start_delay', 'start_delay', float),
        # line 沿机头方向飞（默认 False=旧的沿世界系 +X，保论文回归不变）
        'line_along_heading': bool(scen_leader.get('line_along_heading', False)),
        # 任务完成自动降落（第 1 层）：运动开始后 N 秒广播 LAND；0=禁用。
        'mission_duration': resolve('mission_duration', 'mission_duration', float),
        'publish_hz':   50.0,
        'num_drones':   num,
        # 就绪门控阈值：scenario.leader 可覆盖（默认同 leader_node）。
        # 加大 ready_hold 可让僚机在静止悬停下把队形畸变收敛到位再开动（先成型再走）。
        'ready_hold':    float(scen_leader.get('ready_hold', 2.0)),
        'ready_pos_err': float(scen_leader.get('ready_pos_err', 0.5)),
        # ready_timeout：从 leader 节点启动算起的兜底。默认 90s 对「起节点→手动起飞→切 OFFBOARD」
        # 太短，实飞已撞上（兜底在飞机还在地上时触发、把轨迹跑完）⇒ 允许场景覆盖。
        'ready_timeout': float(scen_leader.get('ready_timeout', 90.0)),
        # 固件消息版本化：跟 common（scenarios.defaults.px4_version）保持一致
        'px4_version':    str(common.get('px4_version', 'main')),
    }

    # 若 scenario/limits 未显式设置安全飞散阈值，从 leader max_distance 自动推导
    # (默认 5m 对追线场景太紧；leader 最远 30m 时阈值自动升到 36m)
    if 'safety_max_track_dist' not in scen.get('limits', {}):
        md = leader_params['max_distance']
        if md > 5.0:
            common.setdefault('safety_max_track_dist', round(md * 1.2, 1))

    nodes = [Node(
        package='mpc_control', executable='leader_node', name='leader_node',
        output='screen', parameters=[leader_params],
    )]

    for drone_id in range(num):
        p = dict(common)
        p['drone_id'] = drone_id
        p['num_drones'] = num
        p['birth_positions_flat'] = birth_flat
        p['formation_offsets_flat'] = offsets_flat
        p['neighbours'] = [int(x) for x in neighbours[drone_id]]
        nodes.append(Node(
            package='mpc_control', executable='mpc_node',
            name=f'mpc_node_{drone_id}', namespace=f'px4_{drone_id}',
            output='screen', parameters=[p],
        ))

    # ── S14 杀节点：kill_at_s 秒后 pkill 目标 mpc_node 进程（模拟整机失联）────
    if 'kill_drone' in faults:
        kill_id = int(faults['kill_drone'])
        kill_at = float(faults.get('kill_at_s', 30.0))
        if not 0 <= kill_id < num:
            raise ValueError(f'faults.kill_drone={kill_id} 越界 (num_drones={num})')
        nodes.append(TimerAction(period=kill_at, actions=[
            LogInfo(msg=f'[P2 FAULT INJECTION] t={kill_at:.0f}s — '
                        f'killing mpc_node_{kill_id} (drone {kill_id} 整机失联模拟)'),
            ExecuteProcess(
                cmd=['pkill', '-f', f'__node:=mpc_node_{kill_id}'],
                output='screen'),
        ]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='scenarios.yaml 里的工况名(如 S4_cross5_line)；'
                        '设了它则 formation/leader 默认从该工况取'),
        DeclareLaunchArgument(
            'formation', default_value='cross5',
            description='solo1|pair2|trio3|cross5|star5|grid9（scenario 未设时生效）'),
        DeclareLaunchArgument(
            'leader_mode', default_value='',
            description='hover|circle|line（留空=用 scenario/默认 hover）'),
        DeclareLaunchArgument(
            'leader_speed', default_value='',
            description='circle/line 速度 m/s（留空=用 scenario/默认 0.5）'),
        DeclareLaunchArgument(
            'leader_radius', default_value='',
            description='circle 半径 m（留空=用 scenario/默认 10）'),
        DeclareLaunchArgument(
            'yaw_mode', default_value='',
            description='fixed|center|tangent（留空=用 scenario/默认 fixed）'),
        DeclareLaunchArgument(
            'max_distance', default_value='',
            description='line 最大距离 m（留空=用 scenario/默认 20）'),
        DeclareLaunchArgument(
            'leader_start_delay', default_value='',
            description='起飞等待 s（留空=默认 30；就绪门控开启时此为兜底）'),
        DeclareLaunchArgument(
            'mission_duration', default_value='',
            description='任务完成自动降落(第1层)：leader 运动开始后 N 秒广播 LAND，各机自主'
                        ' settle→AUTO.LAND。留空/0=禁用（保持旧行为，靠 pkill/人工收尾）'),
        DeclareLaunchArgument(
            'vel_lag_tau', default_value='',
            description='速度环一阶滞后 τ(s)；>0 切"接口一致"模型并直接下发 u0。'
                        '留空=用 scenario.limits/节点默认 0.0（旧双积分模型）'),
        DeclareLaunchArgument(
            'calib_shared_origin', default_value='',
            description='true=定位源为全场共享原点(动捕/外部定位)，world_birth 校准'
                        '改按"与期望出生点的偏差"判定；留空/false=GPS/SITL 各机独立原点。'),
        DeclareLaunchArgument(
            'xy_global_align_enable', default_value='',
            description='true=用各机 EKF 原点 GPS 经纬度相对 drone0 datum 投影对齐 '
                        'world_birth XY(RTK 真机编队用，cm 精度自动对齐)；留空/false=沿用'
                        '配置 birth_xy(SITL/动捕不变)。'),
        DeclareLaunchArgument(
            'soc_norm_enable', default_value='',
            description='true=把逐轴 box 换成范数(内接16边形)约束，消除 √2 泄漏与 τ*=0.53s 上限。'
                        '留空=用 scenario.limits/节点默认 false'),
        DeclareLaunchArgument(
            'w_formation', default_value='',
            description='队形保持代价权重 w_f（论文 §6.5 M4 放大论断的 w_f 扫描）。'
                        '留空=用 scenario.limits/默认 0.5。改此值 acados 指纹自动重建。'),
        DeclareLaunchArgument(
            'dob_enable', default_value='',
            description='true=DOB-MPC 对照 baseline（论文 §6.5 M2）：legacy 零滞后模型叠加 '
                        'FxTDO 估计的加性 d̂ 前馈（v̇=u+d̂）。仅 legacy(vel_lag_tau<=0)生效。'),
        DeclareLaunchArgument(
            'controller', default_value='',
            description='mpc(默认)|consensus。consensus=分布式 leader-follower 位移一致性编队律'
                        '（论文 §6.5 非-MPC 对照基线）。留空=节点默认 mpc。'),
        DeclareLaunchArgument(
            'takeoff_xy_lock_enable', default_value='',
            description='起飞期锁定横向速度指令、防地面积分饱和；空=用节点默认(True)'),
        DeclareLaunchArgument(
            'takeoff_rise_m', default_value='',
            description='相对解锁时高度上升多少算已离地(m)；空=用节点默认(0.5)'),
        DeclareLaunchArgument('takeoff_climb_ms', default_value='',
                              description='离地判据②：持续爬升速度阈值 m/s（空=节点默认 0.2）'),
        DeclareLaunchArgument('takeoff_climb_hold_s', default_value='',
                              description='离地判据②：爬升须连续保持秒数（空=节点默认 1.0）'),
        DeclareLaunchArgument('auto_arm', default_value='',
                              description='空=默认(SITL 自动解锁)；false=等外部解锁'),
        DeclareLaunchArgument('offboard_on_arm', default_value='',
                              description='半自主档：解锁后自动切 OFFBOARD（配 auto_arm:=false）'),
        DeclareLaunchArgument('consensus_kL', default_value='',
                              description='consensus leader/队形位置增益（留空=默认 1.5）'),
        DeclareLaunchArgument('consensus_kC', default_value='',
                              description='consensus 邻居一致性增益（留空=默认 0.8）'),
        DeclareLaunchArgument('dob_dmax', default_value='',
                              description='DOB d̂ 上限 m/s²（留空=节点默认 0.5 保守档；2.0=激进档）'),
        DeclareLaunchArgument(
            'comms_loss_land_s', default_value='',
            description='第2层失联降落：OFFBOARD 中超过 N 秒收不到 leader→自动降落。'
                        '留空=节点默认 0=关（SITL 回归不受影响）；测试传 5.0 打开。'
                        '真机由 real_hardware_launch 默认开(5.0)'),
        DeclareLaunchArgument(
            'comms_loss_hold_s', default_value='',
            description='失联降落"先冻结"阶段：leader 停 N 秒(须<land)先就地悬停不追陈旧参考。'
                        '留空=节点默认 0=关；测试传 1.0 打开。真机由 real_hardware_launch 默认开(1.0)'),
        OpaqueFunction(function=_make_nodes),
    ])
