#!/usr/bin/env python3
"""真机部署 launch — 每架机的 companion 只启【本机】节点（与 SITL swarm_launch 一机全启不同）。

拓扑（pair2 真机）:
  drone0 companion(Orin NX):  MicroXRCEAgent(串口连FC) + mpc_node(drone_id=0)
  drone1 companion(Orin NX):  MicroXRCEAgent(串口连FC) + mpc_node(drone_id=1)
  地面站:                      leader_node（编队参考 + 就绪门控）；diag_monitor.py --log 手动另开

注：Agent 已配 systemd 开机自启（microxrce-agent.service），台架/外场通常
    用 start_agent:=false 避免与之抢串口。

用法:
  # drone0 的 companion:
  ros2 launch mpc_control real_hardware_launch.py drone_id:=0 scenario:=S2_pair2_hover
  # drone1 的 companion:
  ros2 launch mpc_control real_hardware_launch.py drone_id:=1 scenario:=S2_pair2_hover
  # 地面站（最后启动——leader 一发参考+就绪门控通过，编队即开动）:
  ros2 launch mpc_control real_hardware_launch.py role:=leader scenario:=S2_pair2_hover

一次性前置（每架）:
  ⚠️ 当前台架/飞场硬件 = **ZeroOne X6 Air+（PX4 v1.16）+ 以太网**（不是早期 6C+串口）。
     以太网是默认（agent_transport:=udp4）；串口(TELEM2)仅作 6C 类有 TELEM2 板的 fallback。
  FC(QGC):  PX4 **v1.16**；MAV_SYS_ID = drone_id+1；
            以太网(默认，X6 Air+)：`UXRCE_DDS_CFG=Ethernet(1000)` / `UXRCE_DDS_AG_IP`=Orin NX
              IP(int32) / `UXRCE_DDS_PRT=8888`；SD 卡 `/fs/microsd/net.cfg` 设 IP+网关→`netman update`。
              安全/EKF2 参数用 `tools/fc_configure.py`（组 failsafe/limits/geofence/ekf2_mocap/xrce）。
            串口 fallback(6C 等有 TELEM2)：`UXRCE_DDS_CFG=TELEM2`；`SER_TEL2_BAUD=921600`；
              **MAV_1_CONFIG=0**（防 MAVLink 实例占 TELEM2 抢串口——v1.14.3 源码核实默认即
              Disabled、fmu-v6c 未覆盖，故全新 6C 为空操作；⚠️不能用"TELEM2 读不到"判接线好坏，
              出厂本就无人发送；临时释放 NSH `mavlink stop -d <TELEM2设备>`）。
            COM_OBL_RC_ACT=4(Land)、kill switch 必配（详见 report/真机安全参数配置单_QGC.md）。
            drone_id>=1 的 FC 还需 SD 卡 etc/extras.txt 设话题命名空间（对齐 SITL 约定，
            mpc_node 零改动），drone0 无命名空间（/fmu/...）:
                uxrce_dds_client stop
                uxrce_dds_client stop
                usleep 1500000
                uxrce_dds_client start -t udp -h <AgentIP> -p 8888 -n px4_<id>   # 串口则 -t serial -d <dev> -b 921600
            🔴 客户端传输名是 **udp** 不是 udp4（udp4 是 Agent 侧的名字）——2026-08-31 A08
               真机实测 "-t udp4" 报 unknown transport、客户端起不来且**无任何上层报警**；
            🔴 stop 是异步的，必须 usleep 隔开，否则 start 撞"仍在运行"静默失败。
               A08 已按此配置写入并过跨重启验证（extras 由 tools/fc_extras.py 经 MAVLink FTP 远程写）。
  companion(Orin NX):
    [DDS] 全员 **ROS_DOMAIN_ID=0** 一致（X6 Air+ 以太网时代；早期 6C 用过 42，已作废）+ chrony 时钟同步。
          RMW 用 **rmw_fastrtps_cpp，勿用 CycloneDDS**——MicroXRCEAgent 基于 Fast-DDS
          编译，与 CycloneDDS 不互通（2026-07-21 实测 fastrtps 22 话题 / cyclonedds 0
          话题）；仿真机亦从未装过，FastRTPS 才是本项目验证过的配置。
          **每架飞控还须设 UXRCE_DDS_DOM_ID=0**——Agent 的 DDS 域由该飞控参数决定
          （默认 0），只设 companion 侧无效，症状是"Agent 会话正常/话题全建/零报错，
          但 ROS2 里一个话题都看不到"的静默失效。
          验证：`ros2 topic hz /fmu/out/vehicle_status_v1`（应 ~1.97Hz；v1.16 话题带 _v1 后缀）。
          `ros2 topic list --no-daemon | grep fmu` 仅参考——**--no-daemon 有发现竞态**，判链路一律用 hz。
    [串口] **agent_dev 默认 /dev/ttyTHS1**：40 针 J12 的片上 UART，
          Pin8(TX)→TELEM2 RX、Pin10(RX)←TELEM2 TX、Pin6—GND，**VCC 不接**
          （两侧独立供电）；两端同为 3.3V 可直连。**TX↔RX 必须交叉**
          （2026-07-21 实机就是接反了导致 0 字节，对调后立刻通）。
          用户须加入 dialout 组：`sudo usermod -a -G dialout $USER` 后重登。
          ⚠️ **CH340 在 L4T 内核不可用**（无 ch341.ko；lsusb 能看到设备但不生成
          /dev/ttyUSB*，刷机无解，因 Jetson 只能用 NVIDIA 的 tegra 内核）。
          若必须用 USB 转串口，换 **CP2102 或 FTDI** 芯片（这两个驱动内核自带）。
    [历史] RPi4 时代用 CH340 → udev 固定名 /dev/ttyFC（idVendor 1a86），
          **仅在仍使用 RPi4 时适用**，Orin NX 上无效。
    首次 launch 会现编 acados OCP（数分钟），务必外场前在台架跑通一次。

与 SITL 的差异（本文件强制）:
  - scenario 带 faults（杀节点/通信注入）一律拒绝启动——真机不注入故障。
  - 默认 conservative:=true 保守限幅: max_speed≤1.5, max_climb≤1.0, max_accel≤2.0,
    d_safe≥2.5（与 scenario 取更保守一侧）。确有需要再显式 conservative:=false。
  - 默认 auto_arm:=false —— mpc_node 只发 setpoint 流并等待，由飞手 RC 解锁并切
    OFFBOARD（节点确认 nav_state/arming_state 后自动进入编队逻辑）。
  - alt_sync:=auto|true|false，默认 auto=沿用 yaml（SITL 关）。真机各机 home 海拔
    可能真不同，外场建议显式 alt_sync:=true。

起飞顺序（pair2）:
  1) 两架机摆到 scenarios.yaml births 标记点（NED，误差 < calib_max_origin_offset=2m）
  2) 各 companion 启动本 launch → 等日志 "waiting RC ARM+OFFBOARD"
  3) 地面站启 role:=leader
  4) 飞手逐机 RC 解锁 → 切 OFFBOARD → 各机爬升至 target_alt 悬停成队
  5) leader 就绪门控（全员 pos_err<ready_pos_err 保持 ready_hold）通过后开动
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ⚠️ 真机兜底：radius/max_distance 用**室内安全值**，不是 SITL 的室外 10/20。
# 这是"没指定 scenario 又被覆盖成 circle/line 时"的最后防线——飞场只准用 IN_* 场景，
# 别靠兜底飞轨迹。7×8.5m 场地下 radius 1.0(2×=位移 2m)/max_distance 1.5 都在包络内。(2026-07-30)
_LEADER_FALLBACK = dict(mode='hover', speed=0.4, radius=1.0,
                        max_distance=1.5, yaw_mode='fixed', start_delay=30.0)

# 真机保守限幅上限（conservative:=true 时与 scenario/defaults 取更保守一侧）
_CONSERVATIVE_CAPS = dict(max_speed=1.5, max_climb=1.0, max_accel=2.0)
_CONSERVATIVE_D_SAFE_FLOOR = 2.5
_CONSERVATIVE_D_EMERG_FLOOR = 1.8   # Phase 3：真机 d_safe=2.5 → 硬碰撞地板 1.8（< d_safe）


def _load_cfg():
    path = os.path.join(get_package_share_directory('mpc_control'),
                        'config', 'scenarios.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _flatten(nested):
    return [float(v) for pt in nested for v in pt]


def _make_nodes(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    cfg = _load_cfg()
    formations = cfg['formations']
    scenarios = cfg.get('scenarios', {})

    # ── 选 scenario / formation（与 swarm_launch 同逻辑）─────────────────────
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

    # ── 真机红线：故障注入场景一律拒绝 ───────────────────────────────────────
    if scen.get('faults'):
        raise ValueError(
            f'scenario "{scen_name}" 带 faults={scen["faults"]} —— '
            f'真机禁止故障注入，请改用无 faults 工况（如 S2_pair2_hover）')

    # ── 几何 ─────────────────────────────────────────────────────────────────
    births = scen.get('birth_override', fm['birth'])
    offsets = fm.get('offsets', fm['birth'])
    scale = scen.get('offsets_scale')
    if scale:
        offsets = [[v * float(scale) for v in pt] for pt in offsets]
    neighbours = fm['neighbours']
    num = len(births)
    birth_flat = _flatten(births)
    offsets_flat = _flatten(offsets)

    # ── 公共参数: defaults + scenario.limits + 真机保守限幅 ──────────────────
    common = dict(cfg.get('defaults', {}))
    for k, v in scen.get('limits', {}).items():
        common[k] = float(v)
    # 通信注入参数强制清零（双保险，faults 已拒绝）
    common['comms_delay_ms'] = 0.0
    common['comms_dropout'] = 0.0

    if arg('conservative').lower() != 'false':
        for k, cap in _CONSERVATIVE_CAPS.items():
            common[k] = min(float(common.get(k, cap)), cap)
        common['d_safe'] = max(float(common.get('d_safe', 1.5)),
                               _CONSERVATIVE_D_SAFE_FLOOR)
        common['safety_d_emergency'] = max(
            float(common.get('safety_d_emergency', 1.2)),
            _CONSERVATIVE_D_EMERG_FLOOR)

    alt_sync = arg('alt_sync').lower()
    if alt_sync in ('true', 'false'):
        v = (alt_sync == 'true')
        common['alt_sync_enable'] = v
        common['alt_resync_enable'] = v

    # 起飞期横向锁定（2026-08-21）：此前真机 launch **没有透传**这两个参数
    # ⇒ 现场想关也关不掉。空字符串=不覆盖，用节点默认。
    if arg('takeoff_xy_lock_enable'):
        common['takeoff_xy_lock_enable'] = arg('takeoff_xy_lock_enable').lower() in ('1', 'true', 'yes', 'on')
    if arg('takeoff_rise_m'):
        common['takeoff_rise_m'] = float(arg('takeoff_rise_m'))
    if arg('takeoff_climb_ms'):
        common['takeoff_climb_ms'] = float(arg('takeoff_climb_ms'))
    if arg('takeoff_climb_hold_s'):
        common['takeoff_climb_hold_s'] = float(arg('takeoff_climb_hold_s'))

    role = arg('role')
    nodes = []

    # ── role=leader: 地面站只跑 leader_node ──────────────────────────────────
    if role == 'leader':
        scen_leader = scen.get('leader', {})

        def resolve(arg_name, key, cast):
            a = arg(arg_name)
            if a != '':
                return cast(a)
            if key in scen_leader:
                return cast(scen_leader[key])
            return _LEADER_FALLBACK[key]

        nodes.append(Node(
            package='mpc_control', executable='leader_node', name='leader_node',
            output='screen', parameters=[{
                'mode':         resolve('leader_mode', 'mode', str),
                'yaw_mode':     resolve('yaw_mode', 'yaw_mode', str),
                'start_x':      0.0,
                'start_y':      0.0,
                'altitude':     float(common['target_alt']),
                'speed':        resolve('leader_speed', 'speed', float),
                'radius':       resolve('leader_radius', 'radius', float),
                'max_distance': resolve('max_distance', 'max_distance', float),
                'start_delay':  resolve('leader_start_delay', 'start_delay', float),
                'line_along_heading': bool(scen_leader.get('line_along_heading', False)),
                'publish_hz':   50.0,
                'num_drones':   num,
                'ready_hold':    float(scen_leader.get('ready_hold', 5.0)),
                'ready_pos_err': float(scen_leader.get('ready_pos_err', 0.5)),
                # ready_timeout：从 leader 节点启动算起的兜底。默认 90s 对「起节点→手动起飞→切 OFFBOARD」
                # 太短，实飞已撞上（兜底在飞机还在地上时触发、把轨迹跑完）⇒ 允许场景覆盖。
                'ready_timeout': float(scen_leader.get('ready_timeout', 90.0)),
                # 第 1 层「任务完成自动降落」：运动开始 mission_duration 秒后 leader
                # 广播 LAND，各机自主 settle→AUTO.LAND（2026-08-18 补透传：此前真机
                # launch 没传这一项，该功能在真机上够不到，只能靠第 2 层或飞手手动）。
                # 0 = 禁用。trigger_land 可运行时置 true 立即触发。
                # 2026-08-21：改用 resolve ⇒ 也能从 scenario.leader 取，让场景自洽
                # （启动参数仍最高优先级；两处都没有则 0=禁用）。
                'mission_duration': (float(arg('mission_duration'))
                                     if arg('mission_duration') != ''
                                     else float(scen_leader.get('mission_duration', 0.0))),
                'trigger_land':     (arg('trigger_land').lower() == 'true'),
            }],
        ))
        return nodes

    # ── role=drone: 本机 XRCE agent + 本机 mpc_node ──────────────────────────
    drone_id = int(arg('drone_id'))
    if not 0 <= drone_id < num:
        raise ValueError(f'drone_id={drone_id} 越界（{formation} 共 {num} 机）')

    if arg('start_agent').lower() != 'false':
        # 2026-07-23：默认传输由串口改以太网（X6 Air+ 无 TELEM2，TELEM1 留给 3DR 数传）。
        # ⚠️ 装了 deploy/microxrce-agent.service 的机器应传 start_agent:=false，
        #    否则两个 Agent 抢同一个 UDP 端口。
        if arg('agent_transport').lower() == 'serial':
            agent_cmd = ['MicroXRCEAgent', 'serial',
                         '--dev', arg('agent_dev'), '-b', arg('agent_baud')]
        else:
            agent_cmd = ['MicroXRCEAgent', 'udp4', '-p', arg('agent_port')]
        nodes.append(ExecuteProcess(cmd=agent_cmd, output='screen',
                                    name='xrce_agent'))

    # ── 动捕位姿注入 bridge ──────────────────────────────────────────────────
    # 上游是 vrpn_mocap 客户端发布的 /vrpn_mocap/<刚体>/pose(PoseStamped, ENU)，
    # 本节点转成 /fmu/in/vehicle_visual_odometry(VehicleOdometry, NED) 喂 EKF2。
    # **vrpn_mocap 客户端本身不在这里起**（是外部包，且可能跑在地面机器上）。
    # ⚠️ EKF2 配成 vision-only(EKF2_GPS_CTRL=0 / BARO_CTRL=disabled)后，
    #    没有这一路就没有位置估计 —— 解不了锁、也进不了 OFFBOARD。故默认开启。
    if arg('mocap').lower() != 'false':
        rigid = arg('rigid_body') or f'drone{drone_id}'
        mp = {'drone_id': drone_id, 'rigid_body': rigid,
              'timeout_s': float(arg('mocap_timeout_s')),
              # VRPN 默认 ENU→桥转 NED；少数动捕软件直出 NED 时传 input_ned:=true 走直通
              'input_ned': (arg('input_ned').lower() == 'true')}
        if arg('mocap_pose_topic'):
            mp['pose_topic'] = arg('mocap_pose_topic')
        nodes.append(Node(
            package='mpc_control', executable='mocap_bridge_node',
            name=f'mocap_bridge_{drone_id}', output='screen', parameters=[mp],
        ))

    p = dict(common)
    p['drone_id'] = drone_id
    p['num_drones'] = num
    p['birth_positions_flat'] = birth_flat
    p['formation_offsets_flat'] = offsets_flat
    p['neighbours'] = [int(x) for x in neighbours[drone_id]]
    p['auto_arm_enable'] = (arg('auto_arm').lower() == 'true')
    # 半自主档「解锁=授权起飞」：节点不发 ARM，检测到飞手已解锁后自动切 OFFBOARD。
    # 前提：takeoff_xy_lock 已过台架验证（地面接入 OFFBOARD 是 §6.6 的憋劲/过冲场景）。
    p['offboard_on_arm'] = (arg('offboard_on_arm').lower() == 'true')
    # 真机固件 v1.16 起消息版本化：vehicle_status → vehicle_status_v1。
    # 名字错了不报错、只是永远收不到 → 一直卡在 "retry ARM+OFFBOARD"。
    p['px4_version'] = arg('px4_version')
    # 动捕/外部定位：EKF local 原点全场共享，world_birth 校准判据要换
    p['calib_shared_origin'] = (arg('calib_shared_origin').lower() == 'true')
    # RTK 室外编队：各机 EKF 独立原点，用 GPS 经纬度相对 drone0 datum 对齐 world_birth XY。
    # ⚠️ 与动捕互斥：RTK 传 calib_shared_origin:=false xy_global_align_enable:=true。
    p['xy_global_align_enable'] = (arg('xy_global_align_enable').lower() == 'true')
    # 第 2 层：通信失联(收不到 leader)超时自动降落。补飞控层兜不住的"WiFi 丢但 companion
    # 仍发 setpoint"洞（飞控只见 offboard 正常、永不失联降落）。真机默认开(5s)，SITL 关。
    p['comms_loss_land_s'] = float(arg('comms_loss_land_s'))
    # 失联降落"先冻结"中间阶段：leader 停 comms_loss_hold_s 秒先就地悬停（不追陈旧参考），
    # 到 land 阈值再降落。真机默认 1s。
    p['comms_loss_hold_s'] = float(arg('comms_loss_hold_s'))
    nodes.append(Node(
        package='mpc_control', executable='mpc_node',
        name=f'mpc_node_{drone_id}', namespace=f'px4_{drone_id}',
        output='screen', parameters=[p],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'takeoff_xy_lock_enable', default_value='',
            description='起飞期锁定横向速度指令；空=用节点默认(True)。'
                        '⚠️ 2026-08-21 实测该功能有 bug（地面基准记成空中值、锁定永不解除），'
                        '修好并在 SITL 复验前，真机请显式传 false'),
        DeclareLaunchArgument(
            'takeoff_rise_m', default_value='',
            description='相对解锁时高度上升多少算已离地(m)；空=用节点默认(0.5)'),
        DeclareLaunchArgument('takeoff_climb_ms', default_value='',
                              description='离地判据②：持续爬升速度阈值 m/s（空=节点默认 0.2；'
                                          '2026-09-01 双条件判据，防怠速气压漂移假离地）'),
        DeclareLaunchArgument('takeoff_climb_hold_s', default_value='',
                              description='离地判据②：爬升须连续保持秒数（空=节点默认 1.0）'),
        DeclareLaunchArgument('role', default_value='drone',
                              description='drone=本机FC+mpc_node | leader=地面站参考节点'),
        DeclareLaunchArgument('drone_id', default_value='0',
                              description='本机编号（role:=drone 时必填，0..N-1）'),
        DeclareLaunchArgument('scenario', default_value='',
                              description='scenarios.yaml 工况名（faults 工况会被拒绝）'),
        DeclareLaunchArgument('formation', default_value='pair2',
                              description='scenario 未设时的队形'),
        DeclareLaunchArgument('auto_arm', default_value='false',
                              description='true=节点自动 ARM+OFFBOARD（仅台架）；'
                                          '默认 false=等飞手 RC 操作'),
        DeclareLaunchArgument('offboard_on_arm', default_value='false',
                              description='半自主档：飞手 RC 解锁后节点自动切 OFFBOARD 并'
                                          '起飞（解锁=授权起飞）。节点绝不发 ARM；每架次只'
                                          '自动切一次，飞手切走即接管、节点不抢回。'),
        DeclareLaunchArgument(
            'px4_version', default_value='1.16',
            description='真机固件版本：1.16 起 vehicle_status 话题改名 '
                        'vehicle_status_v1（X6 Air+ 实测 v1.16.0）。仿真机仍 1.14。'),
        DeclareLaunchArgument(
            'calib_shared_origin', default_value='true',
            description='动捕/外部定位=全场共享原点，world_birth 校准改按"与期望'
                        '出生点的偏差"判定，并能拦下刚体名↔drone_id 接错。'),
        DeclareLaunchArgument(
            'xy_global_align_enable', default_value='false',
            description='RTK 室外编队：用各机 EKF 原点 GPS 经纬度相对 drone0 datum 对齐 '
                        'world_birth XY。与 calib_shared_origin 互斥(RTK 传 false+本项 true)。'),
        DeclareLaunchArgument('conservative', default_value='true',
                              description='真机保守限幅 v≤1.5 climb≤1.0 a≤2.0 d_safe≥2.5'),
        DeclareLaunchArgument(
            'comms_loss_land_s', default_value='5.0',
            description='第2层失联降落：OFFBOARD 飞行中超过此秒数收不到 leader(WiFi丢/leader崩)'
                        '即自动降落。补飞控层兜不住的"companion健康仍发setpoint、只是收不到协同"'
                        '洞。0=关。默认 5s。SITL(swarm_launch) 不传此参→节点默认 0=关，回归不受影响'),
        DeclareLaunchArgument(
            'comms_loss_hold_s', default_value='1.0',
            description='失联降落"先冻结"中间阶段：leader 停此秒数(须 < comms_loss_land_s)先就地'
                        '悬停，不追陈旧参考（circle/line 下否则会朝 leader 旧位置漂）；到 land 阈值再降落。'
                        '0=关（整个窗口都追陈旧参考）。默认 1s。'),
        DeclareLaunchArgument('alt_sync', default_value='auto',
                              description='auto=沿用yaml | true | false（外场建议 true）'),
        DeclareLaunchArgument('start_agent', default_value='true',
                              description='随 launch 启动 MicroXRCEAgent。⚠️ 装了 '
                                          'deploy/microxrce-agent.service 的机器应设 '
                                          'false，否则两个 Agent 抢同一端口'),
        DeclareLaunchArgument('agent_transport', default_value='udp4',
                              description='udp4=以太网（默认，X6 Air+ 用）| serial=串口。'
                                          'X6 Air+ 没有 TELEM2，TELEM1 要留给 3DR 数传，'
                                          'companion 链路设计上就走以太网'),
        DeclareLaunchArgument('agent_port', default_value='8888',
                              description='udp4 端口，与 FC 参数 UXRCE_DDS_PRT 一致'),
        DeclareLaunchArgument('agent_dev', default_value='/dev/ttyTHS1',
                              description='仅 agent_transport:=serial 时用。Orin NX 片上 '
                                          'UART=/dev/ttyTHS1（40针 Pin8/Pin10）；RPi4+CH340 '
                                          '用 /dev/ttyFC。注意 CH340 在 Jetson/L4T 内核上'
                                          '不可用，详见文件头 [串口] 节'),
        DeclareLaunchArgument('agent_baud', default_value='921600',
                              description='仅 serial 时用，与 SER_TEL2_BAUD 一致'),
        # ── 动捕位姿注入 ──
        DeclareLaunchArgument('mocap', default_value='true',
                              description='启动 mocap_bridge_node（VRPN 位姿 → '
                                          'vehicle_visual_odometry）。EKF2 配成 vision-only '
                                          '后没有这一路就没有位置估计，解不了锁'),
        DeclareLaunchArgument('rigid_body', default_value='',
                              description='动捕刚体名，留空=drone<drone_id>。'
                                          '⚠️ 接错刚体名会让两机飞向同一物理点，'
                                          '靠 calib_shared_origin 的校准门控拦截'),
        DeclareLaunchArgument('mocap_pose_topic', default_value='',
                              description='显式指定位姿话题，覆盖 rigid_body 推导'),
        DeclareLaunchArgument('input_ned', default_value='false',
                              description='动捕位姿是否已是 NED。默认 false=ENU→NED 转换'
                                          '（VRPN 绝大多数是 ENU）；少数软件直出 NED 时设 '
                                          'true 走直通。对应 mocap_bridge_node.input_ned'),
        DeclareLaunchArgument('mocap_timeout_s', default_value='0.3',
                              description='动捕断流告警阈值（遮挡/刚体丢失/VRPN 断线）'),
        # leader 覆盖项（role:=leader 时生效，同 swarm_launch）
        DeclareLaunchArgument('leader_mode', default_value=''),
        DeclareLaunchArgument('leader_speed', default_value=''),
        DeclareLaunchArgument('leader_radius', default_value=''),
        DeclareLaunchArgument('yaw_mode', default_value=''),
        DeclareLaunchArgument('max_distance', default_value=''),
        DeclareLaunchArgument('leader_start_delay', default_value=''),
        DeclareLaunchArgument(
            'mission_duration', default_value='',
            description='第1层任务完成自动降落：运动开始后此秒数广播 LAND，各机自主 '
                        'settle→AUTO.LAND。0=禁用。仅 role:=leader 生效'),
        DeclareLaunchArgument(
            'trigger_land', default_value='false',
            description='立即广播 LAND（不等 mission_duration）。仅 role:=leader 生效'),
        OpaqueFunction(function=_make_nodes),
    ])
