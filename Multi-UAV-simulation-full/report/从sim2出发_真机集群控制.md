# 从 sim2 出发：连接机载电脑 · 做真机集群控制

> 写给坐在 **sim2** 前面的人。目标：接进机群网络 → 连上机载电脑 → 把双机/三机飞起来。
> 建于 2026-08-31。
>
> 配套：`report/链路连接手册.md`（链路全貌与陷阱表）、
> `deploy/README.md`（飞控↔机载电脑那一段）、
> `report/室外RTK_offboard流程.md`（外场飞行流程，**要飞真机先读它**）、
> `report/双机实测前置清单.md`（29 条前置，**前一阶段不过别做后面**）。

---

## 0. 这台机器上已经有什么

| 路径 | 是什么 |
|------|--------|
| `~/ros2_ws/` | **SITL 工作区**，px4_msgs `release/1.14`（配 PX4 1.14）。跑仿真用。 |
| `~/ros2_ws_real/` | **真机工作区**，px4_msgs `release/1.16`（配飞控 PX4 v1.16）。**做地面站用这个。** |
| `~/Multi-UAV-simulation/` | 代码部署副本（`deploy_to_jetson.sh` 的落点），两个工作区都软链到它 |
| `~/Multi-UAV-simulation-full/` | **完整仓库**（401 commits，含 `report/`）。查文档、看历史用这个 |
| `~/handoff/交接/` | 交接包：硬件接线、飞控参数、飞行日志、知识库 |
| `~/mesh_dds.xml` `~/mesh_dds.env` | 本机的 mesh DDS 单播配置（sim2 = `192.168.1.100`） |
| `~/PX4-Autopilot-1.14/` | SITL 用的 PX4（**已打 `px4_overrides`**，见第 7 节） |

### 🔴 两个工作区绝不能混用

px4_msgs `1.14` 与 `1.16` 的消息定义不同（实测：`VehicleLocalPosition`
**49 个字段 → 57 个**，1.16 还多 `MESSAGE_VERSION` 常量），
**ROS 2 类型哈希不同 ⇒ DDS 根本不匹配**。
症状是"节点全活、零报错、一条数据都收不到"——**不会有任何错误提示**。

```bash
source ~/ros2_ws/install/setup.bash        # 只用于 SITL
source ~/ros2_ws_real/install/setup.bash   # 只用于真机
```

**开新终端第一件事就是想清楚这次要干哪个。**

### 0.1 先在 SITL 上把流程跑通（不碰真机，强烈建议）

⚠️ **`~/.bashrc` 里没有 `GZ_SIM_RESOURCE_PATH`**，手动起 Gazebo 会加载不出模型。
手动跑时必须先导出：

```bash
export PX4_DIR=~/PX4-Autopilot-1.14
export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds
export GZ_IP=127.0.0.1     # 见第 7 节：不加则网卡一抖第二架 PX4 起不来
```

（`tools/pair2_sitl_check.sh` 自己会导出这三项，用它就不用管。）

**双机验收套件**（2026-08-31 四项全过，可作回归基线）：

```bash
cd ~/Multi-UAV-simulation
bash tools/cleanup_sim.sh                       # 清进程 + 清 PX4 陈旧锁
WS=$HOME/ros2_ws GZ_IP=127.0.0.1 bash tools/pair2_sitl_check.sh reflat
#   子命令：reflat | hover | semiauto | land | all
```

🔴 **该脚本没有默认子命令**——开头就 pkill 清场并拉 Gazebo，误敲空参数会清掉别人的仿真。
🔑 **`WS=$HOME/ros2_ws` 必须传**（脚本默认找 `~/ros2_control_mpc_ws`，那是另一台机的路径）。

**手动分阶段跑**（队形/场景自选）见 `CLAUDE.md` 的「Ubuntu 端构建与启动顺序」，
或 `~/Multi-UAV-simulation-full/report/RUN_PLAN_仿真运行清单.md`。

### 0.2 本机文档在哪

| 文档 | 位置 |
|------|------|
| 本文 | `~/从sim2出发_真机集群控制.md`（`~/README_从这里开始.md` 是它的软链） |
| 链路连接手册 | `~/链路连接手册.md` |
| 外场飞行流程 | `~/Multi-UAV-simulation-full/report/室外RTK_offboard流程.md` |
| 双机前置清单 | `~/Multi-UAV-simulation-full/report/双机实测前置清单.md` |
| 飞控安全参数 | `~/Multi-UAV-simulation-full/report/真机安全参数配置单_QGC.md` |
| 硬件接线 | `~/handoff/交接/硬件安装与接口对应.md` |
| 交接总纲 | `~/handoff/交接/README_交接总纲.md` |
| 知识库 | `~/handoff/交接/知识库/` |

> ⚠️ 本文与「链路连接手册」建于 2026-08-31，**晚于 `~/Multi-UAV-simulation-full` 的
> 快照**，所以在那个仓库副本里找不到它们；以家目录的版本为准，或拉取更新的仓库。

---

## 1. 先决条件：把 sim2 接进机群网络

机载电脑在 **mesh 网段 `192.168.1.0/24`** 上，靠 **LQ-MESH 电台**（无线）互联。
电台对外是**以太网口**，所以 sim2 必须有一个网口接到电台。

### sim2 不需要新硬件：两个口各司其职

```
wlp0s20f3 (WiFi)     → 上网 / 远程登录
enp0s31f6 (以太网)   → 接 LQ 地面端电台 LAN 口 → mesh 192.168.1.100
```

交接文档 `硬件安装与接口对应.md` 里的拓扑本来就是这样，
**sim2 顶替其中"地面PC"的位置即可**：

```
Jetson USB3网卡 ─ 网线 ─ LQ天空端 LAN1 ══mesh══ LQ地面端 ─ 地面PC(ROS2/SSH)
```

⚠️ **WiFi 到不了 mesh。** 无论 sim2 连哪个 WiFi，都不在 `192.168.1.0/24` 上。
台架阶段若机载电脑也连了同一个 WiFi，可以临时用 WiFi 做部署/监控；
**但飞行必须走 mesh**——飞场 WiFi 弱信号 + 多播发现不稳，正是当初上 mesh 的原因。

⚠️ **地面端电台的 LAN 口只有一个（文档里只出现过 "LAN1"，未实物核实是否还有第二个）**
⇒ **sim2 和笔记本不能同时接 mesh**。要么 sim2 顶替笔记本当地面站（推荐，不需额外硬件），
要么加一个普通交换机让两台都挂上去。

### 配地址

```bash
# 建好放着，不激活（不影响当前正在用的连接）
sudo nmcli con add type ethernet ifname enp0s31f6 con-name lq-mesh \
     ipv4.method manual ipv4.addresses 192.168.1.100/24 \
     ipv4.never-default yes ipv6.method ignore autoconnect no

# 网线改插到电台后切过去
sudo nmcli con down "Wired connection 1"
sudo nmcli con up lq-mesh
```

🔑 **`ipv4.never-default yes` 不能漏。** mesh 网段没有外网，不加它可能抢走默认路由、
把 sim2 的上网断掉（交接文档对飞控口 `eno1` 记的是同一个坑）。
验证：`ip route | head -3` 的默认路由应**仍在 `wlp0s20f3`** 上。

🔑 **接上 mesh 后 sim2 就是双网卡机**（mesh + WiFi）⇒ DDS 的**接口白名单变成必需项**，
否则典型症状是"节点和话题都能发现、就是收不到数据"。生成的 profile 默认已开。

⚠️ **电台供电**：地面端是 Type-C **5V/3A**。电脑 USB 口对不发 PD 协商的哑负载
只给 0.5–0.9A，**喂不饱**——用独立 5V/3A 充电头。（供电不足的症状是抖动/丢包，
容易误判成组网问题。）

🔴 **严禁用 `.20x`**——三台电台自己占着 `.200/.202/.203`。冲突的表现极具迷惑性：
ping 两边都通，只有 TCP 露馅（被 RST），且 ARP 归属会来回翻。
**判据：同一 IP 在不同机器上 `ip neigh` 比 MAC 是否一致。**

| 地址 | 归属 |
|------|------|
| `.100` | **sim2（地面站）** |
| `.101` | A10 = drone1 的机载电脑 |
| `.102` | A08 = drone2 的机载电脑 |
| `.103` | drone3（预留） |
| `.123` | 笔记本 |
| `.200` `.202` `.203` | 🔴 电台自己 |

---

## 2. 连到机载电脑

### 2.1 ssh 免密（一次性）

```bash
ssh-keygen -t ed25519          # 若还没有
ssh-copy-id nvidia@192.168.1.101      # A10
ssh-copy-id nvidia@192.168.1.102      # A08
```

写进 `~/.ssh/config` 便于后续：

```
Host A10
    HostName 192.168.1.101
    User nvidia
Host A08
    HostName 192.168.1.102
    User nvidia
```

### 2.2 机载电脑上的另一条链路（别搞混）

每台机载电脑除了 mesh，还有一条**到自己飞控的点对点网线**，**两架的网段不一样**：

| 机 | 飞控 IP | 机载电脑 IP | 网口 |
|----|---------|-------------|------|
| **A10** | `10.41.10.2` | `10.41.10.100` | `eno1` |
| **A08** | `192.168.77.2` | `192.168.77.100` | `eno1` |

> 🔴 2026-08-19 现场就因为脚本里写死 `192.168.77.2`，在 A10 上完全没数据。
> **任何涉及飞控 IP 的操作都必须按机区分。**

### 2.3 ⚠️ 不要硬拔 Jetson 电源

会损坏 NVMe。正常 `sudo poweroff`。

### 2.4 现场先对时（每次开机）

**Jetson 没有 RTC 电池，开机时间是 1970-01-01**，且飞场没有 NTP。
sim2 自己有 RTC + `systemd-timesyncd`（已确认活跃、已同步），所以**拿 sim2 当时间源**：

```bash
# 🔴 必须走 UTC（date -u）：两端时区可能不同（2026-08-31 实测：地面机时区是
#    GMT、Jetson 是 CST，用墙上时间对时直接差 7 小时）。UTC 与时区无关。
ssh A10 "sudo date -u -s \"$(date -u '+%Y-%m-%d %H:%M:%S')\""
ssh A08 "sudo date -u -s \"$(date -u '+%Y-%m-%d %H:%M:%S')\""
```

> **不影响飞行安全**——邻居新鲜度判定用的是本机时钟在收包时刻取值再与本机时钟相减，
> 纯本地经过时间，跨机偏差进不来。
> **但会毁掉数据归档**：CSV/rosbag 文件名全变 1970，而飞控 `.ulg` 拿到 GPS 后是
> 真实时间，两边对不上、事后没法对齐。
>
> ⚠️ `real_hardware_launch.py` 的头注释要求 chrony 同步；**sim2 上没装 chrony**
> （靠 timesyncd，需要外网）。飞场无外网时 timesyncd 不工作，但 sim2 有 RTC、
> 时间不会跑飞，用上面的 `date -s` 把 Jetson 对齐到 sim2 即可满足需求。

---

## 2b. 必须找人拿到的东西（文档里没有，也不该有）

以下四项**无法从任何文档推导**，接手前必须从上一位负责人手里拿到：

1. **两台 Jetson 的登录凭据**（用户名是 `nvidia`，密码不在仓库里）。
   ⚠️ 仓库曾 public 泄露过明文 SSH 密码，**三台机的凭据是否已轮换未经确认**——
   接手时应重新轮换一遍，别沿用。
2. **GitHub 仓库权限**（`Aaron6099/Multi-UAV-simulation`，Private）。
   sim2 上的 `~/Multi-UAV-simulation-full` 是离线 bundle 解出来的、**没配 remote**，
   要跟进后续提交必须有仓库权限。
3. **遥控器与 kill 开关**：真机默认 `auto_arm:=false`，**必须有飞手用 RC 解锁并切
   OFFBOARD**，程序不会自己解锁。双机同飞建议两名操作手（一人双控不现实）。
4. **飞场准入与基站**：RTK 基站的架设位置/开机方式。

---

## 3. 部署代码到机载电脑

在 sim2 上，于仓库根目录执行：

```bash
cd ~/Multi-UAV-simulation-full
bash tools/deploy_to_jetson.sh A10 A08
```

它只打**飞行相关路径**（约 700KB，不含 `report/`），到机上再提交进本地 git 留追溯。
之所以不直接 `git push`：整仓 291MB 经无线 mesh 又慢又容易断。

**部署后必须在每台机载电脑上重新 build，否则装的还是旧的：**

```bash
ssh A10 'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --packages-select mpc_control'
ssh A08 'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --packages-select mpc_control'
```

核对版本：`ssh A10 'cat ~/Multi-UAV-simulation/DEPLOYED_VERSION'` 应与上游 hash 一致。

⚠️ **首次 launch 会现编 acados OCP（数分钟）**，务必外场前在台架跑通一次。

---

## 4. 真机集群控制：架构与启动

### 4.1 谁跑什么（与 SITL 不同！）

SITL 是一台机器起全部节点；**真机是每台 companion 只起本机节点**：

```
A10 的机载电脑:   MicroXRCEAgent(连飞控) + mpc_node(drone_id=0)
A08 的机载电脑:   MicroXRCEAgent(连飞控) + mpc_node(drone_id=1)
sim2（地面站）:    leader_node（编队参考 + 就绪门控）
                  另开 diag_monitor.py --log 记录
```

**`leader_node` 就是"控制指令"的来源**——它发 `/leader/state`，各机据此算自己的位置。

### 4.2 启动命令

```bash
# 每台 companion（在各自机器上）
ros2 launch mpc_control real_hardware_launch.py drone_id:=0 scenario:=OUT_pair2_hover start_agent:=false mocap:=false calib_shared_origin:=false xy_global_align_enable:=true   # A10
# 🔴 室外必须 mocap:=false（默认 true 会起 mocap_bridge 刷屏报错）；systemd 已装 Agent 的机器传 start_agent:=false
ros2 launch mpc_control real_hardware_launch.py drone_id:=1 scenario:=S2_pair2_hover   # A08

# sim2（地面站）—— 🔑 必须先 source ~/ros2_ws_real
source ~/ros2_ws_real/install/setup.bash
set -a; . ~/mesh_dds.env; set +a
ros2 launch mpc_control real_hardware_launch.py role:=leader scenario:=S2_pair2_hover
```

🔴 **leader 最后启动。** 它一发参考、就绪门控一过，编队立刻开动。

### 4.3 起飞顺序（pair2）

1. 两架机摆到 `config/scenarios.yaml` 里 births 标记点（误差 < 2m）
2. 各 companion 启动 launch → 等日志 `waiting RC ARM+OFFBOARD`
3. **地面站启 `role:=leader`**
4. 飞手逐机 RC 解锁 → 切 OFFBOARD → 各机爬升至 `target_alt` 悬停成队
5. leader 就绪门控通过（全员 `pos_err < ready_pos_err` 保持 `ready_hold`）后开动

### 4.4 真机默认值（launch 文件强制，与 SITL 不同）

- `conservative:=true` —— `max_speed≤1.5`、`max_climb≤1.0`、`max_accel≤2.0`、`d_safe≥2.5`
- `auto_arm:=false` —— 节点只发 setpoint 流并等待，**由飞手 RC 解锁并切 OFFBOARD**
- 带故障注入的 scenario **一律拒绝启动**（真机不注入故障）
- 外场建议显式 `alt_sync:=true`（各机 home 海拔可能真不同）

---

## 5. 扩到三机（及更多）要改什么

现有 `config/scenarios.yaml` 已有 `trio3` 队形。逐项：

| # | 项 | 改什么 |
|---|----|--------|
| 1 | 机载电脑 IP | drone3 用 **`192.168.1.103`**（PEERS 表已预留，无需改代码） |
| 2 | DDS profile | 每台重新生成：`python3 deploy/make_mesh_profile.py --self-ip <本机 mesh IP>` |
| 3 | 飞控 `MAV_SYS_ID` | **= drone_id + 1**（drone2 → `3`） |
| 4 | 飞控话题命名空间 | **`drone_id >= 1` 的飞控**需在 SD 卡 `etc/extras.txt` 里设：<br>`uxrce_dds_client stop`<br>`usleep 1500000`<br>`uxrce_dds_client start -t udp -h <AgentIP> -p 8888 -n px4_<id>`<br>🔴 传输名是 **udp** 不是 udp4（那是 Agent 的），且 stop 后必须 usleep（2026-08-31 真机踩过）<br>**drone0 无命名空间**（`/fmu/...`） |
| 5 | 飞控 `UXRCE_DDS_DOM_ID` | **必须 = 0**（见第 6 节，这是个静默失效点） |
| 6 | 场景 | 用 `S3_trio3_hover` 等 trio3 场景；先 hover，别一步到 line/circle |
| 7 | 启动 | 第三台 companion 加跑 `drone_id:=2`；地面站不变（仍只有一个 leader） |
| 8 | 电台 | 需要第三台 LQ-MESH，且**别占 `.20x`** |

> 队形几何（offsets / neighbours）在 `config/scenarios.yaml` 的 `formations.<name>`，
> **改队形去那里改，不要改 launch 或节点代码。**

---

## 6. 验证与判据

### 每台机载电脑（本机链路）

```bash
ros2 topic hz /fmu/out/vehicle_status_v1        # 应 ~1.97 Hz
```

🔑 **v1.16 的话题带 `_v1` 后缀**（SITL 的 1.14 不带）。
🔑 **判链路一律用 `hz`**，`ros2 topic list --no-daemon | grep fmu` 只作参考——
`--no-daemon` 有发现竞态。

### 跨机（mesh DDS）

```bash
python3 tools/check_mesh_dds.py --no-daemon
# A 环境变量 → B profile 自洽 → C 本机 mesh 地址 → D ping
# → E 电台地址冲突 → F 跨机 ROS2 发现 → G 真收发一条消息
# 双机联调：一台 --sub，另一台 --pub --expect-peer A08
```

🔑 **F 通而 G 不通 = 多网卡 locator 问题**（见下一节）。

### 看话题真有没有数据

```bash
ros2 topic echo --once --qos-reliability best_effort <topic>
```

🔑 **必须加 `--qos-reliability best_effort`**：PX4 的 out 话题是 `BEST_EFFORT`，
而 `echo` 默认按 `RELIABLE` 订阅 ⇒ 不兼容、一条都收不到，
**症状与"根本没数据"完全一样**。

---

## 7. 陷阱（都是实际踩过的）

**"发现得到但收不到数据"有四个不同成因，表现完全一样且都不报错**——按这个顺序排查：

| 成因 | 判据 | 处理 |
|------|------|------|
| `echo` 的 QoS | 只有 `echo` 读不到，节点自己正常 | 加 `--qos-reliability best_effort` |
| 多网卡无白名单 | `check_mesh_dds` F 通 G 不通 | profile 白名单（默认开，**须含 `127.0.0.1`**，否则本机 Agent 与 mpc_node 也发现不了） |
| 飞控 `UXRCE_DDS_DOM_ID` ≠ 0 | Agent 会话正常、话题全建、零报错，但 ROS2 里一个话题看不到 | 每架飞控设 `UXRCE_DDS_DOM_ID=0`。**只设 companion 侧无效** |
| px4_msgs 版本不符 | 同上，且换工作区就好 | 真机用 `~/ros2_ws_real`（1.16） |

其它：

- **RMW 必须 `rmw_fastrtps_cpp`，勿用 CycloneDDS**。MicroXRCEAgent 基于 Fast-DDS 编译，
  与 CycloneDDS 不互通（2026-07-21 实测：fastrtps 22 话题 / cyclonedds 0 话题）。
- **`ROS_LOCALHOST_ONLY=1` 不能用**。它是 rmw 层特性，而 Agent 是裸 DDS 程序、
  不读它 ⇒ 发现不匹配。（`ros2 topic info -v` 里 PX4 发布者显示为
  `_CREATED_BY_BARE_DDS_APP_`，就是这个原因。）
- **非交互 SSH 不读 `~/.bashrc`**。用 `~/mesh_dds.env` + 显式 `export`；
  systemd 用 `EnvironmentFile=`。否则 `mpc_node` 会因找不到 `libhpipm.so` **静默崩溃**
  （进程还在但从不 solve，`ros2 launch` 父进程不报错）。
- **`pkill -f` 经 ssh 内联会自杀**：整段脚本是远程 shell 的 cmdline，含 pattern 就会
  杀掉执行它的 shell。**杀进程与验证必须拆成两条独立 ssh 调用**，
  且验证不能看清理命令自己的输出。
- **Jetson 无 RTC 电池**，开机时间是 1970。不影响飞行安全（邻居新鲜度用的是本机
  经过时间），但**会毁掉数据归档**——CSV/rosbag 全变 1970，而飞控 `.ulg` 是真实时间。
  **每次现场开机先 `sudo date -s`。**
- **SITL 专用**：`GZ_IP=127.0.0.1`（gz-transport 组播绑真实网卡，网卡一抖第二架
  PX4 的 `gz_bridge` 就服务超时起不来）。这条与真机无关。
- **`px4_overrides` 是逐机应用的本地改动，不在 git 里**。sim2 的 PX4 已打
  （`SIM_BAT_MIN_PCT=100` / `COM_OBL_RC_ACT=5` / `COM_DISARM_PRFLT=0`）。
  **重装 PX4 会丢**，症状是 SITL 跑到 60 秒被低电失效保护踢出 OFFBOARD。
  重打：`cp px4_overrides/4001_gz_x500 $PX4/ROMFS/px4fmu_common/init.d-posix/airframes/`，
  **并同步到 `$PX4/build/px4_sitl_default/etc/init.d-posix/airframes/`（两处都要）**。

---

## 8. 🔴 尚未验证的部分（别当结论用）

截至 2026-08-31：

1. **整套 mesh 工具从未在真机上跑过。** `make_mesh_profile.py` 和
   `check_mesh_dds.py` 是 2026-08-30 写的，**跨机 DDS 发现（阶段 F/G）
   是双机的最大未知数**。第一次跑要留出调试时间。
2. **sim2 作为地面站从未跑过真机 leader。** 环境已就绪（`~/ros2_ws_real` 编译通过、
   DDS 配置已生成、接 mesh 不需新硬件），但**电台侧的线还没实际接过，一次都没验证过**。
   另：地面端电台是否只有一个 LAN 口，**未实物核实**（文档里只出现过 "LAN1"）。
3. **A08 收 RTCM 进 RTK-Fixed 从未验过。** 现在基站 RTCM 经 mav_relay → A10。
4. **真机从未双机同飞。** 已完成的是单机三场景（hover/line/circle，2026-08-21）。
5. **双机 SITL 验收已全过**（2026-08-31，四项：reflat/hover/semiauto/land），
   但 SITL 复刻不出"被地面顶住"的场景 ⇒ `takeoff_xy_lock` 挡不挡得住地面积分饱和
   **仍要拆桨台架 + 手举 0.5m 验证**。

⚠️ **多一台机器进同一个 DDS domain，它不只是能看、也能发。**
sim2 上误启一个 `mpc_node`/`leader_node` 就可能给真机发指令。
建议先只放开部署与监控，`leader` 角色等链路实测通过再交给它。

---

## 9. 一句话流程

```
接线进 mesh(.100) → 配 DDS(mesh_dds.env) → check_mesh_dds A–G 全绿
→ ssh 免密到 A10/A08 → deploy_to_jetson.sh + 各机 colcon build
→ 各 companion 起 drone_id:=N → 地面站最后起 role:=leader
→ 飞手逐机解锁+切 OFFBOARD → 就绪门过 → 编队开动
```

飞之前**必须**读 `report/室外RTK_offboard流程.md` 和
`report/双机实测前置清单.md`（29 条，前一阶段不过别做后面）。
