#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全参数导出 → QGC .params（只读）。
策略：先单读一个参数拿 param_count，再按 index 批量请求（PARAM_REQUEST_LIST 在本机实测无响应）。
用法: dump_params2.py <连接串> <输出文件> [备注]
"""
import struct
import sys
import time

from pymavlink import mavutil

CONN, OUT = sys.argv[1], sys.argv[2]
NOTE = sys.argv[3] if len(sys.argv) > 3 else ''
INT_TYPES = (mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
             mavutil.mavlink.MAV_PARAM_TYPE_INT32)

m = mavutil.mavlink_connection(CONN, source_system=250, source_component=190)
print('等心跳...')
if not m.wait_heartbeat(timeout=20):
    sys.exit('!! 没心跳')
m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)
print('已连接 sys=%d' % m.target_system)

params = {}
total = None


def absorb(timeout):
    global total
    t = time.time()
    n = 0
    while time.time() - t < timeout:
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if r is None:
            continue
        nm = r.param_id.strip('\x00') if isinstance(r.param_id, str) \
            else r.param_id.decode().strip('\x00')
        if nm:
            params[nm] = (r.param_value, r.param_type, r.param_index)
            total = r.param_count
            n += 1
    return n


# 1) 先单读一个已知参数，拿 param_count
for _ in range(5):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  b'SYS_AUTOSTART', -1)
    absorb(1.5)
    if total:
        break
if not total:
    sys.exit('!! 拿不到 param_count')
print('飞控共有 %d 个参数，按 index 批量抓取...' % total)

# 2) 按 index 分批请求（每批发一串请求再统一收，减少往返）
BATCH = 25
rounds = 0
while len(params) < total and rounds < 12:
    rounds += 1
    have = {v[2] for v in params.values()}
    todo = [i for i in range(total) if i not in have]
    if not todo:
        break
    print('  第 %d 轮：还差 %d 个' % (rounds, len(todo)))
    for k in range(0, len(todo), BATCH):
        for i in todo[k:k + BATCH]:
            m.mav.param_request_read_send(m.target_system, m.target_component, b'', i)
            time.sleep(0.004)
        absorb(1.2)
    time.sleep(0.2)

print('最终收到 %d / %d' % (len(params), total))

with open(OUT, 'w', encoding='utf-8', newline='\n') as f:
    f.write('# Onboard parameters for Vehicle 1\n#\n# Stack: PX4\n# Vehicle: Multirotor\n')
    if NOTE:
        f.write('# Note: %s\n' % NOTE)
    f.write('# Exported: %s\n# Params: %d / %d\n#\n'
            % (time.strftime('%Y-%m-%d %H:%M:%S'), len(params), total))
    f.write('# MAV ID\tCOMPONENT ID\tPARAM NAME\tVALUE\t(TYPE)\n')
    for name in sorted(params):
        v, t, _ = params[name]
        if t in INT_TYPES:
            f.write('1\t1\t%s\t%d\t%d\n'
                    % (name, struct.unpack('<i', struct.pack('<f', v))[0], t))
        else:
            f.write('1\t1\t%s\t%.10g\t%d\n' % (name, v, t))

print('已写出 %s' % OUT)
if len(params) < total:
    print('⚠️ 缺 %d 个未收到 —— 这份存档不完整，别当唯一副本' % (total - len(params)))
