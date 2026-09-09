# deploy/ — 机载电脑部署件

## `microxrce-agent.service`

MicroXRCE-DDS Agent 的 systemd 开机自启单元。装了它，Jetson 上电即通，
**不需要再 ssh 进去手动起 tmux**。

### 安装（每台新 companion 各装一次）

```bash
scp deploy/microxrce-agent.service jetson:/tmp/
ssh jetson "sudo install -m 644 /tmp/microxrce-agent.service /etc/systemd/system/ \
            && sudo systemctl daemon-reload \
            && sudo systemctl enable --now microxrce-agent"
```

### 验证

```bash
systemctl is-enabled microxrce-agent      # enabled
systemctl is-active  microxrce-agent      # active
journalctl -u microxrce-agent -f          # 日志
# 环境变量是否真的生效（比看配置文件可靠）：
sudo tr '\0' '\n' < /proc/$(pgrep MicroXRCEAgent)/environ | grep -E 'ROS_DOMAIN_ID|RMW_'
# 链路（PX4 v1.16 起话题名带 _v1 后缀，见下）：
ros2 topic hz /fmu/out/vehicle_status_v1  # 应见 ~2.3Hz
```

2026-07-23 在 X6 Air+(v1.16) + Orin NX 以太网方式实测：`vehicle_status_v1` 2.3Hz /
`vehicle_local_position` 100Hz，字段值逐项合理（无 CDR 错位）。

## ⚠️ 上电顺序：**先 Jetson，后飞控**

**PX4 的 `uxrce_dds_client` 在 Agent 消失后不会自动重连。**
2026-07-23 实测：把 Agent 从手动进程换成 systemd 服务（命令完全相同的
`udp4 -p 8888`），飞控端就此静默 —— 以太网上连 MAVLink 广播都停了，
等约 10 分钟无自愈，**必须重启飞控**才重新建立会话。

排查时极易误判成硬件问题，按这个顺序排除：

| 现象 | 含义 |
|---|---|
| `cat /sys/class/net/eno1/carrier` = 1、`ethtool` Link detected: yes | 线材/转换模块正常，**不是物理问题** |
| ping 飞控通，但抓包只有 ARP、没有到 8888 的 UDP | 飞控网络栈活着，**XRCE 客户端没在发** → 重启飞控 |
| 飞控上电后 ~10s 内 ping 不通 | **正常**：net.cfg 先跑 DHCP，超时才回落静态 IP，别当故障 |

⇒ **飞场上电顺序：先开 Jetson（等 `systemctl is-active` 为 active），再给飞控上电。**
顺序反了、或中途重启过 Agent，重启飞控即可恢复。

### 换机器要改的

| 项 | 当前值 | 何时要改 |
|---|---|---|
| `User=` | `nvidia` | companion 用户名不同时 |
| 传输方式 | `udp4 -p 8888` | 2026-07-23 由串口改以太网。X6 Air+ 没有 TELEM2，TELEM1 要留给 3DR 数传 |
| `ROS_DOMAIN_ID` | `0` | 与 FC 参数 `UXRCE_DDS_DOM_ID` **两侧必须一致**（飞场固定要求 0） |
| FC 侧 | `UXRCE_DDS_CFG=1000` / `AG_IP=192.168.77.100` / `PRT=8888` | 换 companion IP 时改 `AG_IP`（见下方整数换算的坑） |

### 网络配置（以太网方式）

**点对点网段 `192.168.77.0/24`**：飞控 `.2`，Orin NX `.100`。

Orin NX 的网口是 **`eno1`**（不是 `eth0`），已用 nmcli 固化：

```bash
sudo nmcli con add type ethernet ifname eno1 con-name fc-link \
     ipv4.method manual ipv4.addresses 192.168.77.100/24 \
     ipv4.never-default yes ipv6.method ignore autoconnect yes
```

⚠️ **`ipv4.never-default yes` 不能省**：否则这条点对点链路可能抢走默认路由，
把走 WiFi 的 SSH 打断。

#### ⚠️ 为什么**不能**用飞场网段 10.41.10.x

飞控出厂 `netman` 配的正是 `10.41.10.2` + 网关 `10.41.10.254`，说明**飞场自己的
网络就是 10.41.10.0/24**（飞场文档把 Agent 写成 10.41.10.100 也是同一网段）。
而 Orin NX **还要连飞场 WiFi 取动捕(VRPN)数据** ⇒ 两个接口落在同一网段会造成
路由歧义：发往 VRPN server 的包可能被塞进只通向飞控的网线里黑洞掉，
**症状极像"动捕没数据"，极难排查**。换成私有段后飞场 WiFi 分到什么地址都不冲突。

#### 飞控侧网络（`netman`）

**`net.cfg` 出厂时并不存在**，地址存在 netman 的非易失存储里。改法（走 MAVLink NSH）：

```bash
# 看当前值
netman show
# 写新配置到 SD 卡再入库
echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=static >> /fs/microsd/net.cfg
echo NETMASK=255.255.255.0 >> /fs/microsd/net.cfg
echo IPADDR=192.168.77.2 >> /fs/microsd/net.cfg
echo ROUTER=192.168.77.100 >> /fs/microsd/net.cfg
echo DNS=192.168.77.100 >> /fs/microsd/net.cfg
netman update      # ⚠️ 执行时 USB 会断开重连（网络栈重配），正常现象
```

两处刻意的取值：

- **`BOOTPROTO=static`（原为 `fallback`）**：点对点链路上没有 DHCP 服务器，
  `fallback` 会先试 DHCP、超时才回落静态 ⇒ 每次上电白等约 10 秒。
  改 static 后**上电 3 秒即上线**。
- **`ROUTER`/`DNS` 指向 Orin NX 而非不存在的 `.1`**：否则飞控会持续 ARP 找网关
  （实测 10 秒 35 个广播帧）。点对点链路上路由项本就用不到。

#### ⚠️ `UXRCE_DDS_AG_IP` 的整数换算坑

该参数是 **INT32**，而 IP 的自然整数形式在**首字节 ≥128 时超出有符号 int32 上限**：

| IP | 自然整数 | 能否直接写 |
|---|---|---|
| `10.41.10.100` | 170461796 | ✅ |
| `192.168.77.100` | 3232255332 | ❌ > 2147483647，须折成 **−1062711964** |

`tools/fc_configure.py` 已内置 `as_int32()` 自动折算（读回值本就是有符号的，
不折算则比对永远不等、每次都判"需要修改"且验证必失败）。

### 与 launch 的关系

`real_hardware_launch.py` 自己也能起 Agent（`start_agent:=true`，默认）。
装了本 service 后，**台架/外场应传 `start_agent:=false`**，否则两个 Agent 抢同一个
串口。

---

## `mav-relay.service` —— QGC 链路开机自启（2026-08-20 新增）

`tools/mav_relay.py` 把飞控的以太网 MAVLink 转发到地面站，**QGC 靠它才能看到飞机**。
它原本只能手动起，而 **Jetson 每次断电重启后手动进程全没了**（XRCE Agent 有 systemd 会
自己回来，桥不会）——2026-08-20 一天之内手动重起了七八次。做成服务即可免除。

### 安装（逐台，**装之前必须改一行**）

```bash
# 1) 复制并改飞控 IP —— A10=10.41.10.2 / A08=192.168.77.2
sudo cp ~/Multi-UAV-simulation/deploy/mav-relay.service /etc/systemd/system/
sudo nano /etc/systemd/system/mav-relay.service     # 改 Environment=MAV_RELAY_FC=...
# 2) 启用
sudo systemctl daemon-reload
sudo systemctl enable --now mav-relay
# 3) 验证（应看到 fwd FC->gcs 计数增长、gcs->FC 非 0 表示 QGC 在回包）
journalctl -u mav-relay -n 20 --no-pager
```

🔴 **`MAV_RELAY_FC` 写错不报错、只是静默无数据**——两架机飞控网段不同，
2026-08-19 现场就因为脚本里写死 `192.168.77.2` 而在 A10 上完全没数据。

### 🔴 与参数读写的互斥（每次都会咬）

桥**独占 Jetson 的 UDP 14550**，而飞控的 MAVLink 正是往这个端口广播的，
所以 `tools/fc_configure.py` / `dump_params2.py` / MAVLink 控制台**都要用同一个端口**：

| 你要做什么 | 桥 |
|---|---|
| 用 QGC 看飞机 | **必须开着** |
| 批量读写飞控参数 / 进 NSH 控制台 | **必须先停** |

```bash
sudo systemctl stop mav-relay     # 改参数前
#   … python3 tools/fc_configure.py --udp <飞控IP>:14550 -g <组> --apply …
sudo systemctl start mav-relay    # 改完立刻恢复
```
💡 **遥测不受此限**：走 ROS2 话题读（`ros2 topic echo /fmu/out/...`）不占 14550，
可与桥并存。只有**参数读写**和 **NSH 控制台**必须停桥。

### 双机注意

两台各跑一份、都指向**同一个地面站 IP**，QGC 在 14550 上按 `MAV_SYS_ID` 分辨出两架。
🔴 前提：两架 `MAV_SYS_ID` 必须不同（A10=1 / A08=2）。相同的话 QGC 会当成同一架飞机。

---
**关联**：`report/真机安全配置清单_FC.md`、`report/真机安全参数配置单_QGC.md`

## `make_mesh_profile.py` —— mesh 上的 Fast-DDS 单播发现（2026-08-30 新增）

无线多播极不稳，是 mesh 组网"时通时断"的头号原因；跨机 ROS2 发现因此必须改成
单播互探。旧的 `mesh_fastdds_profile.xml` 需要**每台手改一行**（本机 mesh IP），
现已改为生成：

```bash
# 每台机器上各跑一次；自动探测本机 192.168.1.x 地址
python3 deploy/make_mesh_profile.py
# 生成 ~/mesh_dds.xml 和 ~/mesh_dds.env，后者可直接给 systemd 用：
#   [Service]
#   EnvironmentFile=/home/nvidia/mesh_dds.env
# 交互 shell 里：  set -a; . ~/mesh_dds.env; set +a
```

要加机器只改生成器里的 `PEERS` 表一处。`--check FILE` 可单独校验一份已部署的
profile（能抓出"拷了别台的文件"这种最难查的错）。

生成的 profile 默认带**接口白名单**（只在 mesh 网卡 + 回环上通信）。Jetson 上同时
有 mesh、飞控直连(192.168.77.x) 和 WiFi 三张网卡，不加白名单时 Fast-DDS 会在所有
网卡上 announce 用户数据 locator，对端可能挑中一个路由不到的地址——典型症状是
**节点和话题都能发现、就是收不到数据**。现场若怀疑白名单本身有问题，用
`--no-whitelist` 生成对照版本再试一次。

🔴 机载电脑一律 `.10x`。LQ 电台自身占用 `.200/.202/.203`，2026-08-19 现场撞过：
同一个 `.202` 从不同机器解析到不同 MAC，TCP 被电台 RST，机间 DDS 立即断链。

## `tools/check_mesh_dds.py` —— 跨机发现分级预检（2026-08-30 新增）

跨机 ROS2 发现**从未在真机上验证过**，是双机编队最大的未知数。别等 launch 起来
看现象，先分级定位：

```bash
python3 tools/check_mesh_dds.py                      # A–E 单机自检
# 双机联调：
#   A08:  python3 tools/check_mesh_dds.py --sub
#   A10:  python3 tools/check_mesh_dds.py --pub --expect-peer A08
```

A 环境变量 / B profile 自洽 / C 本机地址 / D ping / E 电台地址冲突 /
F ROS2 跨机发现 / G 真的收发一条消息。**F 通而 G 不通**基本就是上面那个多网卡
locator 问题。脚本只读不写，最坏是误报。

🔴 ROS2 daemon 会缓存发现结果、报出早已消失的节点。脚本一律加 `--no-daemon`；
你手查时记得先 `ros2 daemon stop`，否则会被旧缓存骗。

⚠️ 这两个脚本本身都**尚未在真机上跑过**（写于 2026-08-30，双机从未联调）。
