#!/usr/bin/env python3
"""Jetson 上的 MAVLink UDP 中转桥：让仿真机上的 QGC 走以太网连飞控。

为什么要它（2026-07-31 趟出来的）
--------------------------------
- 飞控 USB 串口丢包 → QGC "无法检索完整参数集"、UI 不完整；唯一治法是走以太网。
- 但飞控以太网只**广播**到 `192.168.77.255` + **单播**给 Jetson `.100`，**从不主动发仿真机**；
  而 QGC v5.0.8 的 UDP 链路又**只监听本地 14550、从不主动呼叫飞控**（Comm Links 页还一点就崩、
  没法从 UI 改）。两边互等 → QGC 以太网自己连不上。
- 本桥在 Jetson 上把两边接起来：飞控广播/单播 → 转给仿真机 QGC；QGC 回包 → 转给飞控。
  QGC 一收到飞控数据就建立连接、开始回心跳 → 经桥到飞控 → 完整双向、参数走以太网拉全。

前置（缺一不可）
----------------
1. Jetson 开转发 + DOCKER-USER 放行（Jetson 装了 Docker，FORWARD 默认 DROP）：
     sudo iptables -I DOCKER-USER -i wlP1p1s0 -o eno1 -j ACCEPT
     sudo iptables -I DOCKER-USER -i eno1 -o wlP1p1s0 -j ACCEPT
2. 仿真机加到飞控网段的路由（sim sudo 要密码，用户自己跑）：
     sudo ip route add 192.168.77.0/24 via 192.168.155.176   # via Jetson WiFi IP
3. **拔掉飞控上的 USB 线** —— 否则 QGC 会抢 USB 口(Link0)、不理以太网。
4. QGC ini 里要有以太网 UDP 链路（Link1, host0=飞控IP:14550, auto=true）——本台架已配。

跑法（Jetson 上）
-----------------
    setsid nohup python3 ~/…/tools/mav_relay.py [地面站IP] > /tmp/mav_relay.log 2>&1 &

走 LQ mesh 时（2026-08-17 起）
-----------------------------
地面站改挂在 LQ 地面端（192.168.1.123），Jetson 经 USB 网卡 192.168.1.101(A10)/
192.168.1.102(A08) 接天空端（🔴 2026-08-19 改编址：机载电脑用 .10x，.20x 是 LQ 电台自己的地址）：

    python3 tools/mav_relay.py 192.168.1.123

**此时上面的前置 1/2（iptables 转发 + 仿真机路由）都不需要** —— 本桥是纯应用层 UDP
转发，两侧地址都由 Jetson 自己直连（eno1 到飞控、USB 网卡到 mesh），不涉及 IP 转发。
仍然需要前置 3（拔掉飞控 USB，否则 QGC 抢 USB 口不理以太网）。
    # 停：kill -9 <pid>（pgrep 会自匹配命令行里的 "mav_relay"，用 ps aux|grep '[m]av_relay.py' 核实真进程）

要点
----
- **绑 0.0.0.0（不是 .100）**：这样能吃到飞控的 `.255` 广播 → 飞控重启忘记地址后仍能自愈
  （旧版绑 .100 只收单播，FC 一重启就死锁）。代价是同时收到单播+广播、给 QGC 双份包（QGC 容忍）。
- IP 是**本台架值**，8 台机换对应 IP 即可。
- QGC 走 USB 与本桥（以太网）可并存连同一飞控；但本桥占着 Jetson 的 14550，
  要用 pymavlink 走以太网读参数时得先停桥（否则端口被占）。
"""
import socket
import struct
import sys
import time

# 飞控以太网地址。⚠️ 两架机网段不同：**A08=192.168.77.2 / A10=10.41.10.2**
# （2026-08-19 现场踩到：写死 .77 在 A10 上连不到飞控，且**不报错、只是静默无数据**）。
# 用环境变量覆盖，默认值不变以兼容既有文档：
#     A10:  MAV_RELAY_FC=10.41.10.2   python3 tools/mav_relay.py 192.168.1.123
#     A08:  MAV_RELAY_FC=192.168.77.2 python3 tools/mav_relay.py 192.168.1.123
# 两台各跑一份、都指向同一个地面站 IP ⇒ QGC 在 14550 上按 MAV_SYS_ID 分辨出两架飞机。
# 🔴 前提：两架的 MAV_SYS_ID 必须不同（A10=1 / A08=2），否则 QGC 会当成同一架。
import os as _os
FC = (_os.environ.get('MAV_RELAY_FC', '192.168.77.2'), 14550)
# 地面站(QGC)地址：不再写死。默认指向当前地面站电脑，但**任何主动发来 MAVLink 的
# 地面站会被自动学习并锁定** —— 换地面站电脑 / DHCP 换 IP 都能自愈，无需再改本文件。
#   · 默认值只是"QGC 还没主动发过包"时的兜底方向（被动型 QGC 也能先收到飞控数据）。
#   · 一旦某地面站发来合法 MAVLink 包，gcs 立即切到它的真实地址。
#   · ⚠️ 但**自动学习无法自举**：QGC v5.0.8 的 UDP 链路只监听、从不主动呼叫，所以必须
#     先由本桥把飞控数据发到正确地址，QGC 才会回包。换了地面站就得把地址给对——
#     故支持命令行传入：`python3 mav_relay.py <地面站IP> [端口]`，不必再改源码。
#     例（LQ mesh 上的地面 PC）：python3 tools/mav_relay.py 192.168.1.123
_gcs_ip = sys.argv[1] if len(sys.argv) > 1 else '192.168.154.142'
_gcs_port = int(sys.argv[2]) if len(sys.argv) > 2 else 14550
GCS_DEFAULT = (_gcs_ip, _gcs_port)

def is_mavlink(buf):
    # MAVLink v1 起始字节 0xFE / v2 0xFD —— 过滤局域网杂包，避免污染 gcs 地址
    return len(buf) >= 1 and buf[0] in (0xFE, 0xFD)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
# 让 recvmsg 能拿到每个包的**目的地址**（区分广播那份与单播那份，见下方去重）。
# 不开这一项 → ancdata 为空 → _dest_ip() 恒返回 None → 去重不生效（静默失效）。
# ⚠️ 部分 Python 版本的 socket 模块没导出 IP_PKTINFO（Linux 上其值为 8），用数值兜底。
_IP_PKTINFO = getattr(socket, 'IP_PKTINFO', 8)
s.setsockopt(socket.IPPROTO_IP, _IP_PKTINFO, 1)
try:
    s.bind(('0.0.0.0', 14550))
except OSError as e:
    print('bind 14550 失败（被占？先停旧桥/占口进程）:', e, flush=True)
    sys.exit(1)

gcs = GCS_DEFAULT
# ── FC→GCS 去重（2026-08-20 加）────────────────────────────────────────────────
# PX4 的 MAV_2_BROADCAST=1 让飞控**把每个包发两遍**：一份到子网广播地址、一份单播给
# Jetson。实测 `tcpdump -i eno1 udp port 14550` 6 秒内：
#     269 个 10.41.10.2 → 10.41.10.255   (广播)
#     269 个 10.41.10.2 → 10.41.10.100   (单播)
# 本桥绑 0.0.0.0 是为了「吃广播、FC 重启自愈」，代价就是两份都收、两份都转 ⇒
# **QGC 收到的每个包都是双份、mesh 上跑 2 倍流量**。
# 平时无所谓（心跳/姿态丢一两个不影响），但**下载 1127 个参数时是 2254 个包的突发**，
# 走无线 mesh 很容易丢 ⇒ QGC 参数下载卡住、"看不到参数"（2026-08-20 现场实测症状）。
# 两架同时下载更是 4508 个包。
# ⚠️ **按整包哈希去重行不通**（2026-08-20 实测：4935 个包只抓到 64 个，1.3%）——
#    PX4 每次 sendto 都递增 MAVLink 序列号，广播那份和单播那份的 seq 与 CRC 都不同，
#    并非逐字节相同。
# ✅ 改为**按目的地址过滤**：用 IP_PKTINFO 拿到每个包的目的 IP，丢掉发往广播地址的那份。
#    保留自愈能力：只有在「最近确实收到过单播」时才丢广播；一旦单播断流（如 FC 重启后
#    还没学到我们），立刻放行广播，行为退回原来的样子。
_UNICAST_GRACE = 3.0     # 秒：超过这么久没收到单播，就认为单播路径不可用，放行广播
_last_unicast = 0.0

def _dest_ip(ancdata):
    """从 recvmsg 的辅助数据里取本包的目的 IP（in_pktinfo.ipi_addr）。"""
    for level, ctype, cdata in ancdata:
        if level == socket.IPPROTO_IP and ctype == _IP_PKTINFO:
            # struct in_pktinfo { int ipi_ifindex; struct in_addr ipi_spec_dst, ipi_addr; }
            _ifidx, _spec, dst = struct.unpack('I4s4s', cdata[:12])
            return socket.inet_ntoa(dst)
    return None

print(f'mav_relay UP: 0.0.0.0:14550  FC{FC[0]} <-> GCS(默认 {gcs[0]}，自动学习)  '
      f'（吃广播、FC 重启自愈、去重双份包）', flush=True)
n_fc = n_gcs = n_dup = 0
while True:
    data, ancdata, _flags, addr = s.recvmsg(65535, socket.CMSG_SPACE(64))
    ip = addr[0]
    if ip == FC[0]:
        dst = _dest_ip(ancdata)
        now = time.time()
        if dst is not None and dst.endswith('.255'):
            # 广播那份：单播路径正常时丢弃（省一半 mesh 带宽），否则放行以保自愈
            if now - _last_unicast < _UNICAST_GRACE:
                n_dup += 1
                continue
        else:
            _last_unicast = now
        s.sendto(data, gcs)          # 飞控 → 当前地面站
        n_fc += 1
    elif is_mavlink(data):
        if (ip, addr[1]) != gcs:     # 学到新地面站地址 → 锁定并打印
            gcs = (ip, addr[1])
            print(f'[gcs 更新] 地面站 = {gcs[0]}:{gcs[1]}', flush=True)
        s.sendto(data, FC)           # 地面站 → 飞控
        n_gcs += 1
    if (n_fc + n_gcs) and (n_fc + n_gcs) % 2000 == 0:
        print(f'fwd FC->gcs={n_fc} gcs->FC={n_gcs} dup丢弃={n_dup}  gcs={gcs[0]}', flush=True)
