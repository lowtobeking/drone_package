#!/usr/bin/env python3
"""
Arming node with readiness checks.

Sends VehicleCommand to switch each drone to OFFBOARD mode and arm it.

PX4 requires that OffboardControlMode messages are being published at >2Hz
BEFORE the offboard mode switch command is sent. This node verifies that
each drone is actually receiving setpoints (via VehicleStatus feedback)
before issuing commands.

Topic naming convention for PX4 SITL Gazebo Garden multi-instance:
  drone 0 -> /fmu/in/vehicle_command            (target_system=1)
  drone i (i>=1) -> /px4_<i>/fmu/in/vehicle_command  (target_system=i+1)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus


def make_px4_qos():
    """QoS that matches PX4 uXRCE-DDS (BEST_EFFORT, KEEP_LAST 5, TRANSIENT_LOCAL)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ArmingNode(Node):
    def __init__(self):
        super().__init__('arming_node')

        self.declare_parameter('num_drones', 9)
        self.declare_parameter('setup_seconds', 8.0)
        self.declare_parameter('arm_interval', 0.5)

        self.num_drones = int(self.get_parameter('num_drones').value)
        self.setup_seconds = float(self.get_parameter('setup_seconds').value)
        self.arm_interval = float(self.get_parameter('arm_interval').value)

        qos = make_px4_qos()
        self.cmd_publishers = {}
        for i in range(self.num_drones):
            if i == 0:
                topic = '/fmu/in/vehicle_command'
            else:
                topic = f'/px4_{i}/fmu/in/vehicle_command'
            self.cmd_publishers[i] = self.create_publisher(
                VehicleCommand, topic, qos
            )
            self.get_logger().info(f'Will arm drone {i} via {topic}')

        # Per-drone state tracking from VehicleStatus feedback
        self._drone_status = {}
        for i in range(self.num_drones):
            self._drone_status[i] = {
                'nav_state': 0,
                'arming_state': 0,     # 0=unknown, 1=disarmed, 2=armed
                'status_received': False,
            }
            # Subscribe to vehicle_status for readiness checks
            # 🔴 PX4 v1.16 起启用**消息版本化**：话题改名为 `vehicle_status_v1`
            #    （消息类型仍是 px4_msgs/msg/VehicleStatus，只有话题名带后缀）。
            #    v1.14 上不存在 _v1、v1.16 上不存在无后缀版 —— 而**订阅错了不会报错，
            #    只是永远收不到**，`status_received` 就恒为 False、`arming_state` 恒为 0。
            #    2026-08-20 复盘真机首飞 bag 时查出本处仍是旧写法：那次飞手手动解锁，
            #    所以没暴露；但 `auto_arm:=true` 的全自主阶段要靠这里确认解锁结果，
            #    届时本节点是**瞎的**。同一个坑当天在 diag_monitor.py 里也咬过一次。
            # ✅ 两个名字都订阅：哪个存在哪个生效，跨版本通用、无需加参数。
            prefix = '/fmu' if i == 0 else f'/px4_{i}/fmu'
            for _suffix in ('out/vehicle_status_v4', 'out/vehicle_status_v1', 'out/vehicle_status'):
                self.create_subscription(
                    VehicleStatus, f'{prefix}/{_suffix}',
                    self._make_status_cb(i), qos,
                )

        # State machine
        self.state = 'WAITING'  # WAITING -> SET_MODE -> ARM -> DONE
        self.tick_count = 0
        self.setup_ticks = int(self.setup_seconds * 10)  # 10 Hz state machine

        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f'Arming node started. Will wait {self.setup_seconds}s for '
            f'controllers to start streaming setpoints, then arm {self.num_drones} drones.'
        )

    def _make_status_cb(self, drone_id):
        def cb(msg):
            old = self._drone_status[drone_id]
            was_armed = old['arming_state'] == 2
            old['nav_state'] = msg.nav_state
            old['arming_state'] = msg.arming_state
            old['status_received'] = True

            # Log state transitions
            now_armed = msg.arming_state == 2
            if now_armed and not was_armed:
                self.get_logger().info(f'drone {drone_id}: ARMED (nav={msg.nav_state})')
            elif not now_armed and was_armed:
                self.get_logger().warn(f'drone {drone_id}: DISARMED (nav={msg.nav_state})')
        return cb

    def publish_command(self, drone_id, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = drone_id + 1  # MAV_SYS_ID, drone 0 -> 1, drone 1 -> 2, ...
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_publishers[drone_id].publish(msg)

    def tick(self):
        self.tick_count += 1

        if self.state == 'WAITING':
            if self.tick_count >= self.setup_ticks:
                # Check which drones have reported status (i.e. are alive)
                ready = [i for i in range(self.num_drones)
                         if self._drone_status[i]['status_received']]
                not_ready = [i for i in range(self.num_drones)
                             if not self._drone_status[i]['status_received']]
                if not_ready:
                    self.get_logger().warn(
                        f'Setup wait done but drones {not_ready} have not reported status. '
                        f'Arming {len(ready)} ready drones anyway.')
                self.get_logger().info(
                    f'Setup wait complete. Sending OFFBOARD mode commands to {len(ready)} drones.')
                self.state = 'SET_MODE'
                self.mode_drone_idx = 0
                self.mode_subtick = 0

        elif self.state == 'SET_MODE':
            # Send mode-switch command to one drone every arm_interval seconds
            if self.mode_subtick == 0:
                drone_id = self.mode_drone_idx
                # VEHICLE_CMD_DO_SET_MODE: param1 = base_mode, param2 = custom_main_mode
                # base_mode=1 (custom enabled), custom_main_mode=6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                self.publish_command(
                    drone_id,
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )
                self.get_logger().info(f'Sent OFFBOARD mode to drone {drone_id}')

            self.mode_subtick += 1
            if self.mode_subtick >= int(self.arm_interval * 10):
                self.mode_subtick = 0
                self.mode_drone_idx += 1
                if self.mode_drone_idx >= self.num_drones:
                    self.get_logger().info('All mode commands sent. Now arming.')
                    self.state = 'ARM'
                    self.arm_drone_idx = 0
                    self.arm_subtick = 0

        elif self.state == 'ARM':
            if self.arm_subtick == 0:
                drone_id = self.arm_drone_idx
                # VEHICLE_CMD_COMPONENT_ARM_DISARM: param1 = 1.0 to arm
                self.publish_command(
                    drone_id,
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )
                self.get_logger().info(f'Sent ARM to drone {drone_id}')

            self.arm_subtick += 1
            if self.arm_subtick >= int(self.arm_interval * 10):
                self.arm_subtick = 0
                self.arm_drone_idx += 1
                if self.arm_drone_idx >= self.num_drones:
                    self.get_logger().info('All drones armed. Arming node done.')
                    self.state = 'DONE'

        elif self.state == 'DONE':
            # Stay alive but stop sending. Could shutdown, but keep node up so launch doesn't kill peers.
            # Periodically log if any drone disarms unexpectedly
            if self.tick_count % 100 == 0:  # every 10s
                disarmed = [i for i in range(self.num_drones)
                            if self._drone_status[i]['status_received']
                            and self._drone_status[i]['arming_state'] != 2]
                if disarmed:
                    self.get_logger().warn(f'ALERT: drones {disarmed} are not armed!')


def main(args=None):
    rclpy.init(args=args)
    node = ArmingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
