#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通用单参数写入（照抄仓库 fc_configure.py 的连接与编解码方式，含 GCS 心跳那步）。
用法: set_param.py <连接串> <参数名> <值> [--apply]   不加 --apply 只演练
"""
import struct
import sys
import time

from pymavlink import mavutil

CONN, NAME, VAL = sys.argv[1], sys.argv[2], sys.argv[3]
APPLY = '--apply' in sys.argv
INT_TYPES = (mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
             mavutil.mavlink.MAV_PARAM_TYPE_INT32)


def decode(r):
    if r.param_type in INT_TYPES:
        return struct.unpack('<i', struct.pack('<f', r.param_value))[0]
    return round(r.param_value, 6)


def read_param(m, name, timeout=3.0, retries=4):
    for _ in range(retries):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if r is not None and r.param_id.strip('\x00') == name:
                return decode(r), r.param_type
    return None, None


m = mavutil.mavlink_connection(CONN, source_system=250, source_component=190)
if not m.wait_heartbeat(timeout=20):
    sys.exit('!! 没心跳')
# 关键：发一拍 GCS 心跳，否则 PARAM_VALUE 不会单播回来（仓库实测教训）
m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)

cur, ptype = read_param(m, NAME)
if cur is None:
    sys.exit('!! 读不到 %s' % NAME)
is_int = ptype in INT_TYPES
target = int(VAL) if is_int else float(VAL)
print('%s: 当前 %s → 目标 %s   (%s)'
      % (NAME, cur, target, 'INT32' if is_int else 'FLOAT'))

if cur == target:
    print('  已是目标值，无需改动')
    sys.exit(0)
if not APPLY:
    print('  （演练，未写入。加 --apply 才写）')
    sys.exit(0)

if is_int:
    pv = struct.unpack('<f', struct.pack('<i', target))[0]
    t = ptype
else:
    pv, t = float(target), mavutil.mavlink.MAV_PARAM_TYPE_REAL32
m.mav.param_set_send(m.target_system, m.target_component, NAME.encode(), pv, t)
time.sleep(1.0)
back, _ = read_param(m, NAME)
print('  写入后回读 = %s   %s' % (back, '✅ 一致' if back == target else '❌ 不一致'))
