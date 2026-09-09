#!/usr/bin/env python3
"""从飞控下载一条 .ulg（只读）。

用法:  python3 ulog_get.py <log_id> <输出文件> [conn]

策略：按 90 字节一包的 LOG_DATA 拉。为了不被丢包卡死：
  · 顺序请求，收不到就重发当前偏移（有上限）；
  · 每 2s 补一拍 GCS 心跳；
  · 结束或超时后报告完成度，**绝不假装成功**。
"""
import os
import sys
import time

from pymavlink import mavutil

CHUNK = 90


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 64
    log_id = int(sys.argv[1])
    out_path = sys.argv[2]
    conn_str = sys.argv[3] if len(sys.argv) > 3 else "udpin:0.0.0.0:14550"

    m = mavutil.mavlink_connection(conn_str)
    if m.wait_heartbeat(timeout=20) is None:
        print("❌ 无心跳")
        return 2
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(0.3)

    # 先拿这条的大小
    m.mav.log_request_list_send(m.target_system, m.target_component, log_id, log_id)
    size = None
    t0 = time.time()
    while time.time() - t0 < 15:
        e = m.recv_match(type=["LOG_ENTRY"], blocking=True, timeout=2)
        if e is not None and e.id == log_id:
            size = e.size
            break
    if not size:
        print("❌ 拿不到 log %d 的大小" % log_id)
        return 1
    print("log %d 大小 %d 字节 (%.2f MB)" % (log_id, size, size / 1048576.0))

    buf = bytearray(size)
    have = bytearray(size)          # 每字节是否已填
    ofs = 0
    start = time.time()
    last_hb = time.time()
    last_report = time.time()
    stalls = 0

    # 🔑 一次请求**全部剩余**，让飞控连续流式发；只在真的空闲时才重发请求。
    #    早先每批只请 3600B 再等 1s 空闲 ⇒ 实测恰好卡在 3.4 KB/s，
    #    瓶颈是请求节奏而非链路。
    while ofs < size:
        m.mav.log_request_data_send(
            m.target_system, m.target_component, log_id, ofs, size - ofs)
        got_any = False
        idle_since = time.time()
        while time.time() - idle_since < 1.5:
            d = m.recv_match(type=["LOG_DATA"], blocking=True, timeout=0.5)
            if d is None:
                continue
            if d.id != log_id:
                continue
            got_any = True
            idle_since = time.time()
            n = min(d.count, size - d.ofs)
            if n <= 0:
                continue
            buf[d.ofs:d.ofs + n] = bytes(d.data[:n])
            have[d.ofs:d.ofs + n] = b"\x01" * n
            # 推进到"最靠前的未填字节"，避免乱序/丢包时原地打转
            while ofs < size and have[ofs]:
                ofs += 1
            if ofs >= size:
                break
            if time.time() - last_hb > 2:
                m.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = time.time()
            if time.time() - last_report > 5:
                el = time.time() - start
                print("  %6.1f%%  %8d/%d  %6.1f KB/s  已用 %.0fs"
                      % (100.0 * ofs / size, ofs, size,
                         ofs / 1024.0 / max(el, 1e-3), el))
                last_report = time.time()
        if time.time() - last_hb > 2:
            m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = time.time()
        if not got_any:
            stalls += 1
            if stalls > 25:
                print("⚠️ 连续 %d 次没收到数据，停在 %d/%d" % (stalls, ofs, size))
                break
        else:
            stalls = 0
        if time.time() - last_report > 5:
            el = time.time() - start
            print("  %6.1f%%  %8d/%d  %5.1f KB/s  已用 %.0fs"
                  % (100.0 * ofs / size, ofs, size, ofs / 1024.0 / max(el, 1e-3), el))
            last_report = time.time()

    m.mav.log_request_end_send(m.target_system, m.target_component)
    filled = sum(have)
    el = time.time() - start
    with open(out_path, "wb") as f:
        f.write(bytes(buf))
    print("写入 %s" % out_path)
    print("完成度 %d/%d 字节 = %.2f%%   用时 %.0fs   平均 %.1f KB/s"
          % (filled, size, 100.0 * filled / size, el, filled / 1024.0 / max(el, 1e-3)))
    if filled < size:
        print("🔴 **不完整** —— 缺 %d 字节，这个文件不能直接当完整日志用" % (size - filled))
        return 3
    print("✅ 完整")
    return 0


if __name__ == "__main__":
    sys.exit(main())
