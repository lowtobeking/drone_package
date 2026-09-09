#!/usr/bin/env python3
"""向 PX4 的 MAVLink shell (nsh) 发一条命令并收回输出。只读诊断用。

用法: python3 fc_shell.py "uxrce_dds_client status"
"""
import sys
import time

from pymavlink import mavutil

SERIAL_CONTROL_DEV_SHELL = 10


def main():
    cmd = (sys.argv[1] if len(sys.argv) > 1 else "uxrce_dds_client status") + "\n"
    m = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    if m.wait_heartbeat(timeout=15) is None:
        print("NO-HEARTBEAT")
        return 2
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(0.3)

    data = cmd.encode()
    chunk = data[:70] + b"\0" * (70 - len(data[:70]))
    m.mav.serial_control_send(
        SERIAL_CONTROL_DEV_SHELL,
        mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE,
        0, 0, len(data[:70]), chunk)

    out = bytearray()
    t0 = time.time()
    last_rx = time.time()
    while time.time() - t0 < 12:
        msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=1)
        if msg is None:
            if out and time.time() - last_rx > 2.5:
                break
            # 空拍时请求继续吐（部分固件要 poll）
            m.mav.serial_control_send(
                SERIAL_CONTROL_DEV_SHELL,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND, 0, 0, 0, b"\0" * 70)
            continue
        n = msg.count
        if n:
            out += bytes(msg.data[:n])
            last_rx = time.time()
    print(out.decode("utf-8", "replace"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
