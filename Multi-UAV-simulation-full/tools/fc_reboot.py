#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""重启飞控（MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1）。
安全：先检查是否上锁（armed 状态下拒绝执行），PX4 自身在解锁时也会拒绝该指令。"""
import sys
import time

from pymavlink import mavutil

CONN = sys.argv[1]
m = mavutil.mavlink_connection(CONN, source_system=250, source_component=190)
if not m.wait_heartbeat(timeout=20):
    sys.exit('!! 没心跳')
m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)

# 🔴 2026-08-20 修两个缺陷（旧写法：`hb = m.recv_match(type='HEARTBEAT', ...)`）：
#   ① **不过滤来源**：网络上不止飞控在发 HEARTBEAT —— QGC 经 `tools/mav_relay.py` 转发过来的
#      **地面站心跳**同样会被匹配到。实测因此把一台 DISARMED 的飞控误报成 ARMED；
#      反向同样可能：**真解锁却匹配到 GCS 心跳而误判成 DISARMED** ⇒ 保险直接失效。
#   ② **超时 fail-open**：旧写法 `armed = ... if hb else None`，收不到心跳时 armed=None，
#      而 None 是 falsy ⇒ 打印"✅ 已上锁，可以重启"并**继续执行重启**。安全检查必须 fail-closed。
# 正确判据：只认 srcComponent==1(AUTOPILOT1) 且 autopilot 字段非 INVALID 的 HEARTBEAT。
hb = None
_t0 = time.time()
while time.time() - _t0 < 8:
    _msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if _msg is None:
        continue
    if (_msg.get_srcComponent() == 1
            and _msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
        hb = _msg
        break
if hb is None:
    sys.exit('!! 8s 内没收到**飞控**(autopilot)心跳 —— 无法确认上锁状态，拒绝重启')

armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
print('心跳来源: sysid=%d comp=%d  base_mode=0x%02X'
      % (hb.get_srcSystem(), hb.get_srcComponent(), hb.base_mode))
print('当前状态: %s' % ('🔴 已解锁 —— 拒绝重启' if armed else '✅ 已上锁，可以重启'))
if armed:
    sys.exit(1)

print('发送重启指令...')
m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    0, 1, 0, 0, 0, 0, 0, 0)      # param1=1: reboot autopilot

t0 = time.time()
while time.time() - t0 < 5:
    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        print('  ACK result=%d (0=ACCEPTED)' % ack.result)
        break
else:
    print('  (没收到 ACK —— 飞控可能已直接重启，正常)')

print('等飞控回来（心跳消失再出现）...')
time.sleep(4)
m2 = mavutil.mavlink_connection(CONN, source_system=250, source_component=190)
for i in range(12):
    if m2.wait_heartbeat(timeout=5):
        print('✅ 飞控已重启并恢复心跳（约 %ds）' % (4 + i * 5))
        sys.exit(0)
print('⚠️ 60s 内没等到心跳，手动确认飞控状态')
