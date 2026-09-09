#!/usr/bin/env python3
"""动捕位姿注入链路 —— 多机编排。

    # SITL（同时起假 VRPN 源）
    ros2 launch mpc_control mocap_launch.py num_drones:=2

    # 真机（只起 bridge，位姿来自真 vrpn_mocap 客户端）
    ros2 launch mpc_control mocap_launch.py num_drones:=3 sitl:=false \
        rigid_names:=uav1,uav2,uav3

刚体名 ↔ drone_id 映射约定
--------------------------
默认 `drone{i}`（i = drone_id，从 0 起），即 `/vrpn_mocap/drone0/pose` 喂
drone_id=0。飞场刚体命名规则尚未确认，故提供 `rigid_names` 显式指定（逗号
分隔，**顺序即 drone_id 顺序**）。

⚠️ **接错刚体名是动捕多机的高发故障且后果严重**：drone0 拿到 drone1 的位姿后，
`world_birth` 会被烤成两机出生点之差，两机都以为自己在出生点、实际飞向同一
物理点 → 撞机。因此**务必同时启用 `calib_shared_origin`**：

    ros2 launch mpc_control swarm_launch.py scenario:=IN_pair2_hover \
        calib_shared_origin:=true

它会在校准阶段拦下这种错接（详见 mpc_node 的校准门控与 `docs/SITL动捕注入.md`）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(s, default=False):
    return str(s).lower() in ('1', 'true', 'yes', 'on') if str(s) else default


def _setup(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    num = int(arg('num_drones'))
    sitl = _bool(arg('sitl'), True)
    bridges = _bool(arg('bridges'), True)
    names = [s.strip() for s in arg('rigid_names').split(',') if s.strip()]
    if names and len(names) != num:
        raise ValueError(
            f'rigid_names 给了 {len(names)} 个但 num_drones={num} —— '
            f'顺序即 drone_id 顺序，数量必须一致')
    if not names:
        names = [f'drone{i}' for i in range(num)]

    rate = float(arg('rate_hz'))
    noise = float(arg('noise_pos_m'))
    latency = float(arg('latency_ms'))
    drop_start = float(arg('dropout_start_s'))
    drop_dur = float(arg('dropout_duration_s'))
    # 只对这些 drone_id 注入断流/噪声（逗号分隔，留空=全部）。
    # **差模扰动**（只让一架丢定位、其余正常）就靠它 —— 全员同时断是共模，
    # 相对队形不变，测不出编队被拉扯（见 docs/SITL动捕注入.md）。
    faulty = [int(s) for s in arg('fault_drones').split(',') if s.strip()]

    nodes = []
    for i, rigid in enumerate(names):
        if sitl:
            hit = (not faulty) or (i in faulty)
            nodes.append(Node(
                package='mpc_control', executable='sitl_mocap_source_node',
                name=f'sitl_mocap_source_{i}', output='screen',
                parameters=[{
                    'drone_id': i,
                    'rigid_body': rigid,
                    'rate_hz': rate,
                    'noise_pos_m': noise if hit else 0.0,
                    'latency_ms': latency if hit else 0.0,
                    'dropout_start_s': drop_start if hit else -1.0,
                    'dropout_duration_s': drop_dur if hit else 0.0,
                }],
            ))
        if bridges:
            nodes.append(Node(
                package='mpc_control', executable='mocap_bridge_node',
                name=f'mocap_bridge_{i}', output='screen',
                parameters=[{
                    'drone_id': i,
                    'rigid_body': rigid,
                    'timeout_s': float(arg('timeout_s')),
                }],
            ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='1'),
        DeclareLaunchArgument(
            'sitl', default_value='true',
            description='true=起假 VRPN 源(Gazebo 真值)；真机设 false（只起 bridge）'),
        DeclareLaunchArgument(
            'bridges', default_value='true',
            description='false=只起源不起 bridge。与 sitl:=false 互补，用于分别指定'
                        '源与 bridge 的刚体名 —— **测试刚体名错接必须这么做**：'
                        '同一次 launch 里两者用同一个 rigid_names，交换会互相抵消、'
                        '端到端映射仍然正确（2026-07-23 踩过，测了个寂寞）。'),
        DeclareLaunchArgument(
            'rigid_names', default_value='',
            description='逗号分隔的刚体名，顺序即 drone_id 顺序；留空=drone0,drone1,...'),
        DeclareLaunchArgument('rate_hz', default_value='100.0'),
        DeclareLaunchArgument('timeout_s', default_value='0.3'),
        # ── 故障注入（仅 SITL）──
        DeclareLaunchArgument('noise_pos_m', default_value='0.0'),
        DeclareLaunchArgument('latency_ms', default_value='0.0'),
        DeclareLaunchArgument('dropout_start_s', default_value='-1.0'),
        DeclareLaunchArgument('dropout_duration_s', default_value='0.0'),
        DeclareLaunchArgument(
            'fault_drones', default_value='',
            description='只对这些 drone_id 注入故障（逗号分隔）；留空=全部。'
                        '差模扰动用它指定单机。'),
        OpaqueFunction(function=_setup),
    ])
