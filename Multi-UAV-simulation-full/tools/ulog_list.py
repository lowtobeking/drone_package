#!/usr/bin/env python3
"""列举飞控 SD 卡上的 .ulg 日志（只读，不下载、不写任何参数）。

用法:  python3 ulog_list.py udpin:0.0.0.0:14550

🔑 照抄 tools/README_飞控工具.md 的两条硬约束：
   1) 连接串用 udpin（绑本地端口收飞控广播），不要 udpout；
   2) 连上后**先发一拍 GCS 心跳**，否则飞控不会把消息单播回来。
"""
import sys
import time
from datetime import datetime, timezone

from pymavlink import mavutil


def main():
    conn_str = sys.argv[1] if len(sys.argv) > 1 else "udpin:0.0.0.0:14550"
    print("连接 %s ..." % conn_str)
    m = mavutil.mavlink_connection(conn_str)
    hb = m.wait_heartbeat(timeout=20)
    if hb is None:
        print("❌ 20s 内没收到飞控心跳 —— 检查网线/IP/飞控是否上电")
        return 2
    print("✅ 心跳: sys=%d comp=%d type=%d autopilot=%d"
          % (m.target_system, m.target_component, hb.type, hb.autopilot))

    # 🔑 先发 GCS 心跳，否则飞控不单播回我们
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(0.5)

    print("请求日志列表 ...")
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)

    entries = {}
    total = None
    t0 = time.time()
    last_rx = time.time()
    while time.time() - t0 < 40:
        msg = m.recv_match(type=["LOG_ENTRY"], blocking=True, timeout=2)
        if msg is None:
            # 超过 6s 没新条目就认为收完了
            if entries and time.time() - last_rx > 6:
                break
            # 期间保持心跳，别让飞控把我们忘了
            m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            continue
        last_rx = time.time()
        total = msg.num_logs
        if msg.num_logs == 0:
            print("⚠️ 飞控报告日志数为 0（SD 卡没插 / 没日志 / 未挂载）")
            return 1
        entries[msg.id] = msg
        if len(entries) >= msg.num_logs:
            break

    if not entries:
        print("❌ 没收到任何 LOG_ENTRY")
        return 1

    print("\n共 %s 条日志，收到 %d 条条目：\n" % (total, len(entries)))
    print("%5s  %-21s  %12s  %s" % ("id", "UTC 时间", "字节", "MB"))
    print("-" * 60)
    rows = sorted(entries.values(), key=lambda e: e.id)
    for e in rows:
        if e.time_utc:
            ts = datetime.fromtimestamp(e.time_utc, tz=timezone.utc)
            tstr = ts.strftime("%Y-%m-%d %H:%M:%S")
        else:
            tstr = "(无 UTC 时间)"
        print("%5d  %-21s  %12d  %8.1f" % (e.id, tstr, e.size, e.size / 1048576.0))

    big = [e for e in rows if e.size > 1048576]
    print("\n>1MB 的有 %d 条（真正飞过的架次通常 >1MB）" % len(big))
    print("总计 %.1f MB" % (sum(e.size for e in rows) / 1048576.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
