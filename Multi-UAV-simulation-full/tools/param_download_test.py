#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在**地面站这一侧**测试全参数下载，用来把「链路丢包」与「QGC 自身问题」分开。

背景（2026-08-20 现场）：QGC 报「无法检索飞机1参数集」，但在 Jetson 上直连飞控测
PARAM_REQUEST_LIST 完全正常（首个响应 0.00s、15s 收齐 1140 个、param_count=1127）。
差别只在中间多了 `tools/mav_relay.py` + LQ-MESH 无线段 ⇒ 需要在地面站侧复现同一动作，
才能判断包到底丢在哪。

用法（在地面站 PC 上，先让桥把数据转到本脚本监听的端口）：
    # Jetson 侧：把桥指向地面站的 14551（避开 QGC 占用的 14550）
    MAV_RELAY_FC=10.41.10.2 python3 tools/mav_relay.py <地面站IP> 14551 &
    # 地面站侧：
    py tools/param_download_test.py 14551

🔴 **必须先关掉 QGC（或断开它那条链路）**：桥会把回程地址锁定到"最后一个发 MAVLink 来的"
   地面站，QGC 的心跳会把本脚本挤掉，测试就跑不成。

判读：
  · 收齐 param_count 个 → 链路没问题，是 QGC 侧的事（重载/换链路配置）
  · 收不齐且缺口分散    → 丢包，看丢在 mesh 还是桥（对比 Jetson 侧转发计数）
  · 一个都收不到        → 请求根本没到飞控，查桥的 gcs 学习是否指错
"""
import socket
import sys
import time
from collections import defaultdict

from pymavlink import mavutil

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14551
TIMEOUT = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

m = mavutil.mavlink_connection(f'udpin:0.0.0.0:{PORT}',
                               source_system=255, source_component=190)
print(f'监听 udpin:0.0.0.0:{PORT} —— 等飞控心跳...', flush=True)
hb = m.wait_heartbeat(timeout=30)
if hb is None:
    sys.exit('!! 30s 没等到心跳：桥没把数据转到本端口，或桥的 gcs 被别的地面站抢走了')
print(f'  心跳来自 sysid={m.target_system} comp={m.target_component}', flush=True)

# 先发一拍 GCS 心跳，让桥把回程锁定到本脚本
m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                     mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)
time.sleep(0.5)

print('  发送 PARAM_REQUEST_LIST ...', flush=True)
t_req = time.time()
m.mav.param_request_list_send(m.target_system, m.target_component)

got = {}                      # index -> name
total = None
last_rx = time.time()
while time.time() - t_req < TIMEOUT:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
    if msg is None:
        # 连续 8s 没有新参数就认为发完了
        if got and time.time() - last_rx > 8.0:
            break
        continue
    last_rx = time.time()
    total = msg.param_count
    got[msg.param_index] = msg.param_id.strip('\x00')
    n = len(got)
    if total and n % 200 == 0:
        print(f'    已收 {n}/{total}  ({time.time()-t_req:.1f}s)', flush=True)

el = time.time() - t_req
print()
print(f'  耗时 {el:.1f}s   收到 {len(got)} / {total if total else "?"} 个唯一参数')
if total and len(got) < total:
    missing = [i for i in range(total) if i not in got]
    print(f'  🔴 缺 {len(missing)} 个（{100*len(missing)/total:.1f}%）')
    # 看缺口是集中还是分散 —— 集中=某段时间整体断流，分散=随机丢包
    runs = 0
    prev = None
    for i in missing:
        if prev is None or i != prev + 1:
            runs += 1
        prev = i
    print(f'  缺口分成 {runs} 段 ⇒ ' +
          ('分散随机丢包（链路质量问题）' if runs > len(missing) / 3
           else '成段丢失（某时段整体断流）'))
    print(f'  前 20 个缺失 index: {missing[:20]}')
elif total:
    print('  ✅ 全部收齐 —— 链路没问题，问题在 QGC 侧')
