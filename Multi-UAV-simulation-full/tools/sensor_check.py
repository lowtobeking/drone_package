#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读：查飞控当前检测到哪些传感器（SYS_STATUS 位图）+ 解码 CAL_MAG0_ID。
重点回答：现在到底有没有磁罗盘在线（present / enabled / health）。"""
import sys
import time

from pymavlink import mavutil

CONN = sys.argv[1] if len(sys.argv) > 1 else 'udpin:0.0.0.0:14550'

BITS = [
    (0x01, '3D_GYRO'), (0x02, '3D_ACCEL'), (0x04, '★3D_MAG(磁罗盘)'),
    (0x08, 'ABSOLUTE_PRESSURE'), (0x20, 'GPS'), (0x1000, '3D_ANGULAR_RATE_CONTROL'),
    (0x2000, 'ATTITUDE_STABILIZATION'), (0x4000, 'YAW_POSITION'),
    (0x20000, 'XY_POSITION_CONTROL'), (0x40000, 'MOTOR_OUTPUTS'),
    (0x100000, 'RC_RECEIVER'), (0x400000, '2ND_3D_MAG'),
]

m = mavutil.mavlink_connection(CONN, source_system=250, source_component=190)
if not m.wait_heartbeat(timeout=20):
    sys.exit('!! 没心跳')
m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)

st = None
t0 = time.time()
while time.time() - t0 < 15 and st is None:
    st = m.recv_match(type='SYS_STATUS', blocking=True, timeout=2)

if st is None:
    print('!! 15s 内没收到 SYS_STATUS')
else:
    p, e, h = (st.onboard_control_sensors_present,
               st.onboard_control_sensors_enabled,
               st.onboard_control_sensors_health)
    print('SYS_STATUS  present=0x%X enabled=0x%X health=0x%X\n' % (p, e, h))
    print('  %-28s %-8s %-8s %s' % ('传感器', '存在', '启用', '健康'))
    for bit, name in BITS:
        print('  %-28s %-8s %-8s %s'
              % (name, '✅' if p & bit else '—',
                 '✅' if e & bit else '—', '✅' if h & bit else '—'))

# 解码 CAL_MAG0_ID
import struct
INT = (mavutil.mavlink.MAV_PARAM_TYPE_UINT32, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
got = {}
for name in ('CAL_MAG0_ID', 'CAL_MAG0_PRIO', 'SYS_HAS_MAG', 'EKF2_MAG_TYPE'):
    for _ in range(3):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        t = time.time()
        done = False
        while time.time() - t < 2:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if r and r.param_id.strip('\x00') == name:
                got[name] = struct.unpack('<i', struct.pack('<f', r.param_value))[0] \
                    if r.param_type in INT else r.param_value
                done = True
                break
        if done:
            break

did = got.get('CAL_MAG0_ID', 0)
print('\nCAL_MAG0_ID = %s' % did)
if did:
    devtype = did & 0xFF
    bus = (did >> 8) & 0x1F
    addr = (did >> 13) & 0xFF
    bus_type = (did >> 21) & 0x07
    BT = {0: 'UNKNOWN', 1: 'I2C', 2: 'SPI', 3: 'UAVCAN', 4: 'SIMULATION', 5: 'SERIAL'}
    print('  → devtype=0x%02X(%d)  bus=%d  addr=0x%02X  bus_type=%s'
          % (devtype, devtype, bus, addr, BT.get(bus_type, bus_type)))
    print('  （bus_type=I2C 且 bus 非 0 通常= GPS 模块上的外置磁罗盘）')
print('  CAL_MAG0_PRIO = %s   (0=禁用, 常见启用值 50/75/100)' % got.get('CAL_MAG0_PRIO'))
print('  SYS_HAS_MAG   = %s   (0=声明板载无磁罗盘)' % got.get('SYS_HAS_MAG'))
print('  EKF2_MAG_TYPE = %s   (0=Auto, 5=None)' % got.get('EKF2_MAG_TYPE'))
