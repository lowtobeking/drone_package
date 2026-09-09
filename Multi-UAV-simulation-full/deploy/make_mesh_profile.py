#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""为 LQ-MESH 无线组网生成本机的 Fast-DDS 单播发现 profile。

为什么要生成而不是手改
----------------------
`mesh_fastdds_profile.xml` 里有一行必须逐机不同（本机 mesh IP）。手改一行、三台机器、
每次重装都要再来一遍——这类"只差一行"的配置正是最容易改错又最难发现的。这里改成从
下面的 PEERS 表生成：加一台机器只改一处，生成的文件天然自洽。

用法（在每台 Jetson 上）
------------------------
    python3 deploy/make_mesh_profile.py                       # 自动探测本机 mesh IP
    python3 deploy/make_mesh_profile.py --self-ip 192.168.1.101
    python3 deploy/make_mesh_profile.py --check ~/mesh_dds.xml # 只校验已有文件

生成后按脚本末尾打印的三行导出环境变量再起节点；systemd 里用 Environment= 显式写，
非交互 shell 不读 .bashrc。

接口白名单（默认开启）
----------------------
Jetson 上同时存在 mesh 网卡(192.168.1.x)、飞控直连网卡(192.168.77.x) 和 WiFi。
不加白名单时 Fast-DDS 会在**所有**网卡上announce用户数据 locator，对端可能挑中一个
它根本路由不到的地址 —— 典型症状是"节点/话题能发现，但收不到数据"。白名单把
locator 钉在 mesh IP + 回环（回环是给本机 XRCE Agent ↔ mpc_node 用的，必须留）。

⚠️ 本文件与生成的 XML 都**尚未在真机上验证过**（截至 2026-08-30 双机从未联调）。
   若加了白名单反而更糟，用 --no-whitelist 生成对照版本再试，这是现场第一手排查动作。
"""
import argparse
import ipaddress
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

# ── 唯一真值源：mesh 网段上的全部 DDS 参与者 ────────────────────────────────
# 🔴 机载电脑一律 .10x。LQ 电台自身占用 .200/.202/.203，2026-08-19 现场撞过：
#    同一个 .202 从不同机器解析到不同 MAC，TCP 被电台 RST，机间 DDS 立即断链。
MESH_NET = "192.168.1.0/24"
PEERS = [
    ("192.168.1.100", "地面 PC / QGC（若在其上跑 ROS2）"),
    ("192.168.1.101", "drone1 = A10"),
    ("192.168.1.102", "drone2 = A08"),
    ("192.168.1.103", "drone3（预留）"),
]

TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<!--
  Fast-DDS 单播发现 profile —— LQ-MESH 无线组网
  本机 mesh IP: {self_ip}
  由 deploy/make_mesh_profile.py 生成，请勿手改；要改改生成器里的 PEERS 表。
  接口白名单: {wl_state}

  部署所需的三个环境变量已一并写入同名 .env 文件
  (systemd 可直接 EnvironmentFile= 它；非交互 shell 不读 .bashrc)。
  注：此处刻意不写绝对路径 —— XML 注释里不允许出现连续两个连字符，
  而路径里一旦含有它整份文件就解析不了。
-->
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
{transport}    <participant profile_name="mesh_unicast" is_default_profile="true">
      <rtps>
{user_transports}        <builtin>
          <discovery_config>
            <discoveryProtocol>SIMPLE</discoveryProtocol>
            <!-- 无线丢包下放宽租约，避免误判邻居掉线 -->
            <leaseDuration><sec>12</sec></leaseDuration>
            <leaseAnnouncement><sec>3</sec></leaseAnnouncement>
          </discovery_config>

          <!-- 本机监听地址：手动指定 metatraffic 单播 locator 会让 Fast-DDS 弃用默认
               多播 locator = 禁多播（无线多播极不稳，是组网时通时断的头号原因）。 -->
          <metatrafficUnicastLocatorList>
            <locator><udpv4><address>{self_ip}</address></udpv4></locator>
          </metatrafficUnicastLocatorList>

          <!-- 单播互探名单：全网一致，端口留空 = 自动探测标准 metatraffic 端口。 -->
          <initialPeersList>
            <locator><udpv4><address>127.0.0.1</address></udpv4></locator>  <!-- 本机 XRCE Agent -->
{peers}          </initialPeersList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
"""

TRANSPORT_BLOCK = """    <transport_descriptors>
      <transport_descriptor>
        <transport_id>mesh_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>{self_ip}</address>
          <address>127.0.0.1</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>

"""

USER_TRANSPORTS_BLOCK = """        <userTransports>
          <transport_id>mesh_udp</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>

"""


def detect_self_ip():
    """在 mesh 网段上找本机地址（Linux）。找不到返回 None。"""
    net = ipaddress.ip_network(MESH_NET)
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True,
                             text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for addr in re.findall(r"inet (\d+\.\d+\.\d+\.\d+)", out):
        if ipaddress.ip_address(addr) in net:
            return addr
    return None


def render(self_ip, whitelist=True, out_abs="<路径>"):
    peers = "".join(
        '            <locator><udpv4><address>%s</address></udpv4></locator>  <!-- %s -->\n'
        % (ip, note) for ip, note in PEERS)
    return TEMPLATE.format(
        self_ip=self_ip,
        wl_state="开启（只在 mesh 网卡 + 回环上通信）" if whitelist else "关闭（对照版本）",
        transport=TRANSPORT_BLOCK.format(self_ip=self_ip) if whitelist else "",
        user_transports=USER_TRANSPORTS_BLOCK if whitelist else "",
        peers=peers,
    )


def check(path):
    """校验一份已生成/已部署的 profile，返回问题列表（空 = 通过）。"""
    problems = []
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as e:
        return ["XML 无法解析：%s" % e]
    ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}

    def addrs(xpath):
        found = root.findall(xpath, ns) or root.findall(xpath.replace("d:", ""))
        return [a.text.strip() for a in found if a.text]

    meta = addrs(".//d:metatrafficUnicastLocatorList//d:address")
    if len(meta) != 1:
        problems.append("metatraffic 监听地址应恰好 1 条，实为 %d 条：%s" % (len(meta), meta))
    else:
        net = ipaddress.ip_network(MESH_NET)
        if ipaddress.ip_address(meta[0]) not in net:
            problems.append("监听地址 %s 不在 mesh 网段 %s 内" % (meta[0], MESH_NET))
        local = detect_self_ip()
        if local and meta[0] != local:
            problems.append("监听地址 %s 与本机 mesh 地址 %s 不一致（多半是拷错了别台的文件）"
                            % (meta[0], local))
        wl = addrs(".//d:interfaceWhiteList//d:address")
        if wl and meta[0] not in wl:
            problems.append("接口白名单 %s 不含本机监听地址 %s" % (wl, meta[0]))
        if wl and "127.0.0.1" not in wl:
            problems.append("接口白名单缺 127.0.0.1 —— 本机 XRCE Agent 与 mpc_node 将无法互相发现")

    peers = addrs(".//d:initialPeersList//d:address")
    if "127.0.0.1" not in peers:
        problems.append("initialPeersList 缺 127.0.0.1（本机 XRCE Agent）")
    for ip, _ in PEERS:
        if ip not in peers:
            problems.append("initialPeersList 缺 %s" % ip)
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-ip", help="本机 mesh IP（不给则自动探测）")
    ap.add_argument("--out", default=os.path.expanduser("~/mesh_dds.xml"))
    ap.add_argument("--no-whitelist", action="store_true",
                    help="不加接口白名单（现场排查用的对照版本）")
    ap.add_argument("--check", metavar="FILE", help="只校验已有 profile，不生成")
    ap.add_argument("--stdout", action="store_true", help="打到标准输出而不写文件")
    a = ap.parse_args()

    if a.check:
        problems = check(a.check)
        for p in problems:
            print("  [X] " + p)
        print("校验 %s：%s" % (a.check, "通过" if not problems else "%d 个问题" % len(problems)))
        return 1 if problems else 0

    ip = a.self_ip or detect_self_ip()
    if not ip:
        print("找不到本机 mesh IP。要么本机没接 mesh，要么网段变了；用 --self-ip 显式指定。",
              file=sys.stderr)
        return 2
    if ip not in [p[0] for p in PEERS]:
        print("警告：%s 不在 PEERS 表里，别的机器不会来探你。先把它加进生成器。" % ip,
              file=sys.stderr)

    out_abs = os.path.abspath(a.out)
    xml = render(ip, whitelist=not a.no_whitelist, out_abs=out_abs)
    if a.stdout:
        sys.stdout.write(xml)
        return 0
    with open(out_abs, "w", encoding="utf-8", newline="\n") as f:
        f.write(xml)
    env_abs = os.path.splitext(out_abs)[0] + ".env"
    with open(env_abs, "w", encoding="utf-8", newline="\n") as f:
        f.write("RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n")
        f.write("ROS_DOMAIN_ID=0\n")
        f.write("FASTRTPS_DEFAULT_PROFILES_FILE=%s\n" % out_abs)
    print("已生成 %s（本机 %s，白名单 %s）" % (out_abs, ip, "关" if a.no_whitelist else "开"))
    print("已生成 %s（systemd 可 EnvironmentFile= 它）" % env_abs)
    problems = check(out_abs)
    for p in problems:
        print("  [X] " + p)
    print("\n交互 shell 里则：  set -a; . %s; set +a" % env_abs)
    print("然后跑 tools/check_mesh_dds.py 做联通性预检。")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
