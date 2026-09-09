#!/usr/bin/env python3
"""经 MAVLink FTP 读/写飞控 SD 卡的 etc/extras.txt（XRCE 命名空间用）。

用法:
  python3 fc_extras.py read                # 只读：下载并打印现有 extras.txt（若有）
  python3 fc_extras.py write px4_1         # 写入 stop+start -n px4_1（写前自动备份旧文件到本地）
  python3 fc_extras.py verify              # 回读并打印
  python3 fc_extras.py rollback            # 删除 extras.txt（回退到无命名空间）

⚠️ 写/删都是对真飞控 SD 卡的操作；写完须重启飞控才生效。
"""
import sys
import time

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP

REMOTE = "/fs/microsd/etc/extras.txt"
LOCAL_TMP = "/tmp/extras_download.txt"


def connect():
    m = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    if m.wait_heartbeat(timeout=15) is None:
        print("NO-HEARTBEAT")
        sys.exit(2)
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(0.3)
    ftp = MAVFTP(m, target_system=m.target_system, target_component=m.target_component)
    return m, ftp


def do_read(tag="read"):
    m, ftp = connect()
    ret = ftp.cmd_get([REMOTE, LOCAL_TMP])
    ftp.process_ftp_reply(ret.operation_name, timeout=10)
    try:
        data = open(LOCAL_TMP, "rb").read()
        print("=== %s: %s (%d bytes) ===" % (tag, REMOTE, len(data)))
        print(data.decode("utf-8", "replace"))
        return data
    except FileNotFoundError:
        print("=== %s: 远端无 %s（下载无产物）===" % (tag, REMOTE))
        return None


def do_write(ns):
    content = (
        # 🔑 stop 是异步的：紧跟的 start 会撞"仍在运行"而失败，客户端保持旧配置
        #    （2026-08-31 实测：无 usleep 时命名空间不生效）。usleep 单位微秒。
        # 🔑 echo 落一个 marker 文件：区分"extras 没执行"与"执行了但 start 失败"。
        "echo extras_begin > /fs/microsd/extras_marker.txt\n"
        "uxrce_dds_client stop\n"
        "usleep 1500000\n"
        # 🔴 传输名必须是 udp（udp4 是 Agent 侧的名字）：2026-08-31 真机实测
        #    "-t udp4" 报 unknown transport、客户端起不来。仓库文档此前写错。
        "uxrce_dds_client start -t udp -h 192.168.77.100 -p 8888 -n %s\n"
        "echo extras_end >> /fs/microsd/extras_marker.txt\n" % ns
    )
    # 先备份旧文件（若有）
    old = do_read("写前现状")
    if old is not None:
        bak = "/tmp/extras_backup_%d.txt" % int(time.time())
        open(bak, "wb").write(old)
        print("旧文件已备份: %s" % bak)

    src = "/tmp/extras_upload.txt"
    open(src, "w", newline="\n").write(content)

    m, ftp = connect()
    # 确保 etc 目录存在（已存在会报错，忽略）
    ret = ftp.cmd_mkdir(["/fs/microsd/etc"])
    ftp.process_ftp_reply(ret.operation_name, timeout=10)
    ret = ftp.cmd_put([src, REMOTE])
    ftp.process_ftp_reply(ret.operation_name, timeout=30)
    print("已上传，回读校验…")
    time.sleep(1)
    got = do_read("回读")
    if got is not None and got.decode("utf-8", "replace").strip() == content.strip():
        print("VERIFY-OK 内容一致")
    else:
        print("VERIFY-FAIL 回读与写入不一致！")
        sys.exit(3)


def do_rollback():
    m, ftp = connect()
    ret = ftp.cmd_rm([REMOTE])
    ftp.process_ftp_reply(ret.operation_name, timeout=10)
    print("已请求删除 %s；重启飞控后回到无命名空间状态" % REMOTE)


if __name__ == "__main__":
    op = sys.argv[1] if len(sys.argv) > 1 else "read"
    if op == "read" or op == "verify":
        do_read()
    elif op == "write":
        do_write(sys.argv[2] if len(sys.argv) > 2 else "px4_1")
    elif op == "rollback":
        do_rollback()
    else:
        print(__doc__)
