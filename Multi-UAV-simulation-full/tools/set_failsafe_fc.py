#!/usr/bin/env python3
"""S31 前置：通过 MAVLink 给 PX4 SITL 实例0 设 failsafe 参数。
COM_OF_LOSS_T = 0.5 (s)     offboard 信号超时阈值
COM_OBL_RC_ACT = 5 (Hold)   offboard 失联动作
PX4 GCS mavlink 在 udp 18570(local)→14550(remote)，绑 14550 即收到心跳。
PX4 param 协议对 INT32 用字节 reinterpret-cast（非数值转换），须 struct pack/unpack。
"""
import struct
import sys
import time

from pymavlink import mavutil

m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print('waiting for PX4 heartbeat on :14550 ...')
hb = m.wait_heartbeat(timeout=30)
if hb is None:
    print('ERROR: no heartbeat'); sys.exit(1)
print(f'heartbeat: sys={m.target_system} comp={m.target_component}')


def set_param(name, value, ptype):
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        wire = struct.unpack('f', struct.pack('i', int(value)))[0]
    else:
        wire = float(value)
    for attempt in range(3):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), wire, ptype)
        t0 = time.time()
        while time.time() - t0 < 3:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if msg and msg.param_id == name:
                if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                    got = struct.unpack('i', struct.pack('f', msg.param_value))[0]
                else:
                    got = msg.param_value
                print(f'{name} = {got}')
                return got
    raise RuntimeError(f'no PARAM_VALUE ack for {name}')


v1 = set_param('COM_OF_LOSS_T', 0.5, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
v2 = set_param('COM_OBL_RC_ACT', 5, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

ok = abs(v1 - 0.5) < 1e-3 and int(v2) == 5
print('PARAM SET OK' if ok else f'PARAM MISMATCH: {v1}, {v2}')
sys.exit(0 if ok else 1)
