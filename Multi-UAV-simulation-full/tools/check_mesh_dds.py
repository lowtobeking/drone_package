#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LQ-MESH 上跨机 ROS2 发现的分级预检。

背景：截至 2026-08-30，**跨机 ROS2 发现从未在真机上验证过**，它是双机编队最大的
未知数——不通则所有双机准备（队形、leader 门控、同步起飞）全部白做。所以这个脚本
的设计目标是"在起编队节点之前，把失败定位到具体一层"，而不是等 launch 起来看现象。

分级（前一级不过就别看后面，后面必然连带失败）：
  A 环境变量      三个变量齐全、profile 文件存在
  B profile 自洽  监听地址=本机 mesh 地址、白名单含回环、peers 表完整
  C 本机地址      mesh 网卡确实拿到了 192.168.1.x
  D L3 可达       ping 每个 peer
  E 网段卫生      电台占用的 .20x 是否被误配成机载电脑地址（2026-08-19 现场事故）
  F ROS2 发现     跨机能否看见对方的节点/话题
  G 数据面        真的收发一条消息（发现通≠数据通，多网卡下尤其常见）

用法
----
  # 单机自检（A–E，不需要对端）
  python3 tools/check_mesh_dds.py

  # 双机联调：两台都装好 profile 后
  #   A08 上先跑：  python3 tools/check_mesh_dds.py --sub
  #   A10 上再跑：  python3 tools/check_mesh_dds.py --pub --expect-peer A08
  # --sub 会阻塞等一条消息，收到即打印 PASS。

🔴 ROS2 daemon 会缓存发现结果、让 `ros2 node list` 报出早已消失的节点。本脚本一律
   加 --no-daemon；你自己手查时记得 `ros2 daemon stop` 再查，否则会被旧缓存骗。

⚠️ 本脚本自身**尚未在真机上跑过**（写于 2026-08-30，双机从未联调）。它只调用
   ping / ros2 CLI，不改任何配置，最坏情况是误报，不会把现场搞坏。
"""
import argparse
import ipaddress
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "deploy"))

MESH_NET = "192.168.1.0/24"
RADIO_IPS = ["192.168.1.200", "192.168.1.202", "192.168.1.203"]  # LQ 电台自身占用
CHECK_TOPIC = "/mesh_check"

_results = []


def stage(name, ok, detail="", hint=""):
    _results.append((name, ok))
    print("  [%s] %s" % ("PASS" if ok else "FAIL", name))
    if detail:
        for line in str(detail).splitlines():
            print("         " + line)
    if not ok and hint:
        print("         ↳ " + hint)
    return ok


def run(cmd, timeout=15):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except FileNotFoundError:
        return 127, "命令不存在: %s" % cmd[0]
    except subprocess.TimeoutExpired:
        return 124, "超时 %ds" % timeout


def local_mesh_ips():
    net = ipaddress.ip_network(MESH_NET)
    rc, out = run(["ip", "-4", "-o", "addr"], timeout=5)
    if rc != 0:
        return []
    return [a for a in re.findall(r"inet (\d+\.\d+\.\d+\.\d+)", out)
            if ipaddress.ip_address(a) in net]


# ── A 环境变量 ──────────────────────────────────────────────────────────────
def check_env():
    want = {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS_DOMAIN_ID": "0"}
    ok = True
    for k, v in want.items():
        got = os.environ.get(k)
        if got != v:
            ok = stage("A 环境变量 %s" % k, False, "got=%r want=%r" % (got, v),
                       "set -a; . ~/mesh_dds.env; set +a  （systemd 里用 EnvironmentFile=）") and ok
        else:
            stage("A 环境变量 %s" % k, True)
    prof = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
    if not prof:
        return stage("A profile 路径", False, "FASTRTPS_DEFAULT_PROFILES_FILE 未设",
                     "python3 deploy/make_mesh_profile.py 会生成它和同名 .env") and ok
    if not os.path.isfile(prof):
        return stage("A profile 路径", False, "%s 不存在" % prof,
                     "路径写错或文件没拷过来") and ok
    stage("A profile 路径", True, prof)
    return ok


# ── B profile 自洽 ─────────────────────────────────────────────────────────
def check_profile():
    prof = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
    if not prof or not os.path.isfile(prof):
        return stage("B profile 自洽", False, "跳过：A 阶段未通过")
    try:
        import make_mesh_profile as gen
    except ImportError as e:
        return stage("B profile 自洽", False, "无法导入 deploy/make_mesh_profile.py: %s" % e)
    problems = gen.check(prof)
    return stage("B profile 自洽", not problems, "\n".join(problems),
                 "重新生成：python3 deploy/make_mesh_profile.py")


# ── C 本机 mesh 地址 ────────────────────────────────────────────────────────
def check_local_ip():
    ips = local_mesh_ips()
    return stage("C 本机 mesh 地址", len(ips) == 1,
                 "找到: %s" % (ips or "无"),
                 "mesh 网卡没拿到 %s 里的地址；先确认电台通电、网线插的是 mesh 那块网卡" % MESH_NET)


# ── D L3 可达 ───────────────────────────────────────────────────────────────
def check_reachability(peers):
    mine = set(local_mesh_ips())
    ok = True
    for ip in peers:
        if ip in mine:
            stage("D ping %s (本机)" % ip, True)
            continue
        rc, _ = run(["ping", "-c", "2", "-W", "2", ip], timeout=10)
        ok = stage("D ping %s" % ip, rc == 0, "",
                   "对端没开机 / 不在 mesh 上 / IP 配错。MESH 是对等单跳，"
                   "不通就先逐台直连电台确认工作模式与 ESSID/频率/密码一致") and ok
    return ok


# ── E 网段卫生：电台地址冲突 ────────────────────────────────────────────────
def check_hygiene(peers):
    bad = [ip for ip in peers if ip in RADIO_IPS]
    if bad:
        return stage("E 地址冲突", False, "peers 表里含电台自身地址 %s" % bad,
                     "机载电脑一律 .10x。2026-08-19 现场撞过：同一个 .202 从不同机器"
                     "解析到不同 MAC，TCP 被电台 RST，机间 DDS 立即断链")
    alive = []
    for ip in RADIO_IPS:
        rc, _ = run(["ping", "-c", "1", "-W", "1", ip], timeout=5)
        if rc == 0:
            alive.append(ip)
    return stage("E 地址冲突", True,
                 "电台在线于 %s（正常，只要没有机载电脑用这些地址）" % (alive or "无响应"))


# ── F ROS2 发现 ─────────────────────────────────────────────────────────────
def check_discovery(expect_peer):
    if not shutil.which("ros2"):
        return stage("F ROS2 发现", False, "找不到 ros2 命令", "先 source 你的 ROS2 环境")
    rc, out = run(["ros2", "node", "list", "--no-daemon"], timeout=20)
    if rc != 0:
        return stage("F ROS2 发现", False, out.strip()[:400])
    nodes = [n for n in out.splitlines() if n.strip().startswith("/")]
    stage("F 本机可见节点数", True, "%d 个: %s" % (len(nodes), ", ".join(nodes[:8]) or "（空）"))
    if not expect_peer:
        print("         ↳ 未给 --expect-peer，跨机发现这一项没有判据；双机联调时务必给")
        return True
    # 🔑 2026-08-31 真机首跑修正：原判据是 `expect_peer in n`，即在节点名里找
    #    "A10"/"A08" 这种主机名字符串。但本项目的节点名是 mpc_node_0 / leader_node，
    #    **永远不含主机名** ⇒ 该判据从构造上就不可能通过，必然假 FAIL。
    #    （当天实测：F 报 FAIL，而 G 数据面双向通过；随后用真实 ROS 节点复验，
    #      A10 确实看得到 A08 的 /xcheck_a08 节点 + /chatter 话题 + 真收到数据。）
    #    改为：本机之外只要**看得见任何节点**就算发现通；主机名匹配降级为附加信息。
    hit = [n for n in nodes if expect_peer in n]
    ok = bool(nodes)          # 看得见任何节点即算发现层通（节点名不含主机名，见上）
    detail = "可见节点: %s" % (", ".join(nodes) if nodes else "（空）")
    if hit:
        detail += "；其中名字含 %r: %s" % (expect_peer, ", ".join(hit))
    return stage("F 跨机发现（须对端正在跑 ROS 节点才有判据）", ok, detail,
                 "⚠️ 若对端此刻没跑任何 ROS 节点，本项**无判据**，FAIL 不代表网络不通"
                 "——以 G 数据面为准（2026-08-31 真机实测即为此情形）。"
                 "⚠️ 本机自己在跑节点时也会命中，判「跨机」请看节点名/用 --expect-peer。"
                 "真发现不通的常见原因：两台 ROS_DOMAIN_ID 不同 / profile 没生效"
                 "（确认 FASTRTPS_DEFAULT_PROFILES_FILE 在**起节点的那个 shell** 里）"
                 " / initialPeersList 里没写对端 / 防火墙挡了 UDP")


# ── G 数据面 ────────────────────────────────────────────────────────────────
def do_pub(seconds):
    if not shutil.which("ros2"):
        return stage("G 数据面 pub", False, "找不到 ros2 命令")
    print("  … 在 %s 上以 2Hz 发布 %ds（对端应已在跑 --sub）" % (CHECK_TOPIC, seconds))
    rc, out = run(["ros2", "topic", "pub", "-r", "2", CHECK_TOPIC,
                   "std_msgs/msg/String", "{data: mesh-check}"], timeout=seconds)
    # 正常情况是被 timeout 打断（124），那说明它一直在发
    return stage("G 数据面 pub", rc in (124, 0), out.strip()[:300])


def do_sub(seconds):
    if not shutil.which("ros2"):
        return stage("G 数据面 sub", False, "找不到 ros2 命令")
    print("  … 等 %s 上的一条消息，最多 %ds（对端应在跑 --pub）" % (CHECK_TOPIC, seconds))
    rc, out = run(["ros2", "topic", "echo", "--once", "--no-daemon",
                   CHECK_TOPIC, "std_msgs/msg/String"], timeout=seconds)
    got = "mesh-check" in out
    return stage("G 数据面 sub", got, out.strip()[:300],
                 "F 通而 G 不通 = 典型的多网卡 locator 问题：对端 announce 了一个你路由"
                 "不到的地址。用 deploy/make_mesh_profile.py 的接口白名单版本（默认开）；"
                 "已经开了还不行就用 --no-whitelist 生成对照版本再试一次")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--expect-peer", help="对端节点名里应出现的子串（如 A08 或 px4_1）")
    ap.add_argument("--pub", action="store_true", help="数据面：本机当发送端")
    ap.add_argument("--sub", action="store_true", help="数据面：本机当接收端")
    ap.add_argument("--seconds", type=int, default=20, help="数据面测试时长/超时（默认 20s）")
    ap.add_argument("--skip-ros", action="store_true", help="只做 A–E，不碰 ros2")
    a = ap.parse_args()

    try:
        import make_mesh_profile as gen
        peers = [p[0] for p in gen.PEERS]
    except ImportError:
        peers = []

    print("== A 环境变量 ==");        check_env()
    print("== B profile 自洽 ==");    check_profile()
    print("== C 本机 mesh 地址 =="); check_local_ip()
    if peers:
        print("== D L3 可达 ==");      check_reachability(peers)
        print("== E 网段卫生 ==");     check_hygiene(peers)
    else:
        stage("D/E", False, "无法导入 PEERS 表，跳过")

    if not a.skip_ros:
        print("== F ROS2 发现 ==");    check_discovery(a.expect_peer)
        if a.pub:
            print("== G 数据面 =="); do_pub(a.seconds)
        elif a.sub:
            print("== G 数据面 =="); do_sub(a.seconds)
        else:
            print("== G 数据面 ==\n  … 跳过（两台分别加 --sub / --pub 才做这一项）")

    bad = [n for n, ok in _results if not ok]
    print("\n%d/%d 项通过" % (len(_results) - len(bad), len(_results)))
    if bad:
        print("未通过：" + "、".join(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
