#!/usr/bin/env python3
"""null5_launch.py —— 论文 M1「五机独立单飞零模型」专用启动（2026-09-01）。

目的
----
§6.2c 报告"五机 per-agent 发散 ≈ 单机的 2.1×"，但论文里**没有任何实验能把
「场景效应」（5 架机同处一个世界/一台主机/同一风况）与「真多机效应」（显式或
隐式的机间耦合）分开**。审稿人一定会问。本 launch 就是那个零模型：

    与 S34_cross5 完全相同的**出生几何、参考轨迹、风扰协议、机数、主机负载**，
    但**一切机间耦合置零**：

      · 每架机一个**独立 leader**（各自的就绪门控、各自的运动时钟），
        参考圆心 = 各机自己的出生点 ⇒ 轨迹与 cross5 逐点相同
      · `neighbours=[]`  ⇒ 编队项 + 碰撞项在 OCP 里 active=0（代价恒为 0）
      · `alt_resync_enable=False` ⇒ 无 `/swarm/alt_datum` 共享高度基准
      · `/leader/state` `/swarm/mission` `/swarm/*_datum` 全部按机 remap 到独立命名空间

判据（见 report/审稿准备_2x放大依赖句清单.md）
    per-agent p2p ≈ 100 m ⇒ 2× 放大是**场景效应**，"多机放大"措辞全面降格
    per-agent p2p ≈  49 m ⇒ 放大确为**多机效应**，来源仍开放（现措辞保留 + 补引本实验）

用法
----
  ros2 launch mpc_control null5_launch.py scenario:=S_solo1_circle_nosafety_s1
  ros2 launch mpc_control null5_launch.py scenario:=S_solo1_circle_nosafety_s1 vel_lag_tau:=0.5

⚠️ `formation` 只用来取**出生几何**（必须与 start_5_px4.sh 的 spawn 一致，
   两者同读 scenarios.yaml 的 formations.<name>.birth）；队形 offsets 一律置零，
   因为每架机跟的是自己那个 leader，不再有"相对队形"这回事。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LEADER_FALLBACK = dict(mode='hover', speed=0.5, radius=10.0,
                        max_distance=20.0, yaw_mode='fixed', start_delay=30.0,
                        mission_duration=0.0)


def _load_cfg():
    path = os.path.join(get_package_share_directory('mpc_control'),
                        'config', 'scenarios.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _make_nodes(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    cfg = _load_cfg()
    formations = cfg['formations']
    scenarios = cfg.get('scenarios', {})

    scen_name = arg('scenario')
    scen = {}
    if scen_name:
        if scen_name not in scenarios:
            raise ValueError(f'未知 scenario "{scen_name}"，可选: {list(scenarios)}')
        scen = scenarios[scen_name]

    formation = arg('formation')
    if formation not in formations:
        raise ValueError(f'未知 formation "{formation}"，可选: {list(formations)}')
    births = formations[formation]['birth']
    num = len(births)
    birth_flat = [float(v) for pt in births for v in pt]
    # 🔑 offsets 全零：每架机跟自己的 leader（圆心已经是它的出生点），
    #    再叠 offset 会把圆心平移两次。
    offsets_flat = [0.0] * (3 * num)

    # ── MPC 公共参数：defaults + scenario.limits ─────────────────────────────
    common = dict(cfg.get('defaults', {}))
    for k, v in scen.get('limits', {}).items():
        common[k] = v if isinstance(v, bool) else float(v)
    # 🔴 零模型的核心：拆掉共享高度基准（这是一条真实的机间耦合，
    #    swarm_launch 对 cross5 是强制打开的）。
    common['alt_resync_enable'] = False
    alt_stagger = float(arg('alt_stagger') or 0.0)
    if arg('vel_lag_tau'):
        common['vel_lag_tau'] = float(arg('vel_lag_tau'))
    if arg('soc_norm_enable'):
        common['soc_norm_enable'] = arg('soc_norm_enable').lower() in ('1', 'true', 'yes', 'on')

    # ── leader 参数（每架一份，仅圆心不同）────────────────────────────────────
    scen_leader = scen.get('leader', {})

    def resolve(arg_name, key, cast):
        a = arg(arg_name)
        if a != '':
            return cast(a)
        if key in scen_leader:
            return cast(scen_leader[key])
        return _LEADER_FALLBACK[key]

    leader_common = {
        'mode':         resolve('leader_mode', 'mode', str),
        'yaw_mode':     resolve('yaw_mode', 'yaw_mode', str),
        'altitude':     float(common['target_alt']),
        'speed':        resolve('leader_speed', 'speed', float),
        'radius':       resolve('leader_radius', 'radius', float),
        'max_distance': resolve('max_distance', 'max_distance', float),
        'start_delay':  resolve('leader_start_delay', 'start_delay', float),
        'line_along_heading': bool(scen_leader.get('line_along_heading', False)),
        'mission_duration': resolve('mission_duration', 'mission_duration', float),
        'publish_hz':   50.0,
        # 🔑 每个 leader 只管自己那一架 ⇒ num_drones=1，它订阅的 '/mpc/health'
        #    由 remapping 指到本机的 health 话题。
        'num_drones':   1,
        'ready_hold':    float(scen_leader.get('ready_hold', 2.0)),
        'ready_pos_err': float(scen_leader.get('ready_pos_err', 0.5)),
        'ready_timeout': float(scen_leader.get('ready_timeout', 90.0)),
    }

    if 'safety_max_track_dist' not in scen.get('limits', {}):
        md = leader_common['max_distance']
        if md > 5.0:
            common.setdefault('safety_max_track_dist', round(md * 1.2, 1))

    nodes = []
    for i in range(num):
        px4_pre = '' if i == 0 else f'/px4_{i}'
        lp = dict(leader_common)
        lp['altitude'] = float(common['target_alt']) - i * alt_stagger
        lp['start_x'] = float(births[i][0])   # 圆心 = 本机出生点（NED N）
        lp['start_y'] = float(births[i][1])   # NED E
        nodes.append(Node(
            package='mpc_control', executable='leader_node',
            name=f'leader_node_{i}', namespace=f'null5_{i}',
            output='screen', parameters=[lp],
            remappings=[
                ('/leader/state',   f'/null5_{i}/leader/state'),
                ('/swarm/mission',  f'/null5_{i}/swarm/mission'),
                ('/mpc/health',     f'{px4_pre}/mpc/health'),
                ('/fmu/out/vehicle_local_position',
                 f'{px4_pre}/fmu/out/vehicle_local_position'),
            ],
        ))

        p = dict(common)
        p['drone_id'] = i
        p['num_drones'] = num
        # ── 竖向错层（2026-09-01 加）─────────────────────────────────────────
        # 零模型没有碰撞项，legacy 风扰下五架各自散开 ~100 m，水平间距实测掉到
        # 0.15 m（x500 旋翼跨度 ~0.5 m）⇒ Gazebo 里模型真的会穿插接触，发散量里
        # 混进物理碰撞扰动，零模型就不干净了。把各机错开固定高度即可物理隔离，
        # 而**不引入任何机间耦合**（各机仍只看自己的 leader），水平动力学与判据不变。
        # alt_stagger=0（默认）= 与 cross5 同高度，行为与本文件初版完全一致。
        if alt_stagger:
            p['target_alt'] = float(common['target_alt']) - i * alt_stagger
        p['birth_positions_flat'] = birth_flat
        p['formation_offsets_flat'] = offsets_flat
        # 🔑 零耦合：传自己的 id —— mpc_node 的过滤器会剔掉 j==drone_id，
        #    得到空邻居表（编队项/碰撞项 active=0、不订阅任何队友话题）。
        #    不直接传 [] 是因为空数组会让 ROS2 参数无法推断类型而启动失败；
        #    solo1 队形在 scenarios.yaml 里也正是用 [[0]] 表达"无邻居"。
        p['neighbours'] = [i]
        nodes.append(Node(
            package='mpc_control', executable='mpc_node',
            name=f'mpc_node_{i}', namespace=f'px4_{i}',
            output='screen', parameters=[p],
            remappings=[
                ('/leader/state',    f'/null5_{i}/leader/state'),
                ('/swarm/mission',   f'/null5_{i}/swarm/mission'),
                ('/swarm/alt_datum', f'/null5_{i}/swarm/alt_datum'),
                ('/swarm/xy_datum',  f'/null5_{i}/swarm/xy_datum'),
            ],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='S_solo1_circle_nosafety_s1',
                              description='取 leader 协议与 limits 的工况名'),
        DeclareLaunchArgument('formation', default_value='cross5',
                              description='只取出生几何（须与 start_5_px4.sh 一致）'),
        DeclareLaunchArgument('leader_mode', default_value=''),
        DeclareLaunchArgument('leader_speed', default_value=''),
        DeclareLaunchArgument('leader_radius', default_value=''),
        DeclareLaunchArgument('yaw_mode', default_value=''),
        DeclareLaunchArgument('max_distance', default_value=''),
        DeclareLaunchArgument('leader_start_delay', default_value=''),
        DeclareLaunchArgument('mission_duration', default_value=''),
        DeclareLaunchArgument(
            'alt_stagger', default_value='',
            description='竖向错层步长 m：第 i 架目标高度 = target_alt - i*步长。'
                        '零模型无碰撞项时用来物理隔离各机（不引入任何机间耦合）。'
                        '留空/0 = 全部同高度（与 cross5 一致）'),
        DeclareLaunchArgument('vel_lag_tau', default_value=''),
        DeclareLaunchArgument('soc_norm_enable', default_value=''),
        OpaqueFunction(function=_make_nodes),
    ])
