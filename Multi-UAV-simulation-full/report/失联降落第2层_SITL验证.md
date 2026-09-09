# 第 2 层失联降落 — SITL 验证命令序列

> 验证目标(commit `5a42956` 起):**OFFBOARD 飞行中 leader 停 `comms_loss_land_s` 秒 → 自动
> settle→AUTO.LAND**,且**正常飞行不误触发**。此功能真机安全相关,上真机前必须在 SITL 跑通本清单。
> 环境:仿真机 `ssh sim`(shirui@192.168.154.101),PX4 v1.14 SITL + Gazebo。
> 关联:`飞场当天流程.md` §6、记忆 `landing-autoland-layer1`。

---

## 🔴 0. 安全前置:域隔离(不做可能误发指令给真飞控)

SITL 的 drone0 用非命名空间 `/fmu`,与真机 X6 Air+ 在 domain0 撞车;SITL 默认 `auto_arm=true`
**会真的去解锁**。若真机/Jetson 此刻在同一局域网上电,**先隔离**:

```bash
# 二选一(或都做):
# ① 让真机离开 domain0(最稳,可逆):
ssh jetson "sudo systemctl stop microxrce-agent"     # 测完记得 start 回来
# ② 本次 SITL 所有终端显式用 domain 42(仿真机 .bashrc 默认就是 42,交互 shell 已是)
export ROS_DOMAIN_ID=42
```
判据:`ROS_DOMAIN_ID=0 ros2 topic list --no-daemon | grep fmu` 应为空(domain0 干净)。

---

## 1. 拉最新代码 + 构建(仿真机)

```bash
ssh sim
cd ~/ros2_control_mpc_ws/src/mpc_control && git pull origin main     # 应含 5a42956
cd ~/ros2_control_mpc_ws
rm -rf ~/.cache/mpc_control/acados_di_mpc_* /tmp/acados_di_mpc_*      # 结构未变,保险起见
colcon build --packages-select mpc_control && source install/setup.bash
```
> 环境变量(非交互才需;交互 shell 由 .bashrc 提供):
> `export GZ_SIM_RESOURCE_PATH=... ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH`

---

## 2. 起 SITL(solo1,四个终端;每个终端先 `source install/setup.bash` + `ROS_DOMAIN_ID=42`)

```bash
# 终端0 清残留(独立一条,别和后续命令串;pkill 自杀坑)
pkill -9 -f '[p]x4'; pkill -9 -f '[g]z sim'; pkill -9 -f 'Micro[X]RCEAgent'; pkill -9 -f '[m]pc_node'; pkill -9 -f '[l]eader_node'

# 终端1 Gazebo
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

# 终端2 PX4(solo1=1 机;等 Gazebo 就绪再起)
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_1_px4.sh

# 终端3 XRCE Agent
MicroXRCEAgent udp4 -p 8888
```

---

## 3. 测试 A:正确触发(leader 停 → 自动降落)  ✅ 主验证

```bash
# 终端4:带看门狗开关起飞(comms_loss_land_s:=5.0),日志留档便于判读
ros2 launch mpc_control swarm_launch.py scenario:=S0_solo1_hover \
     comms_loss_land_s:=5.0 2>&1 | tee /tmp/land2_A.log
```

**A-1 起飞武装确认**(等 ~40s,让它解锁→进 OFFBOARD→爬到 target_alt 悬停):
- 启动日志应有:`第2层失联降落已武装：OFFBOARD 中 leader 停 5.0s → 自动降落`
- 日志不再刷 `waiting/retry ARM+OFFBOARD`;Gazebo 里飞机已悬停在空中(不是趴地上)

**A-2 制造 leader 失联**(飞机确认在空中悬停后,**新开一条独立终端**执行,别和别的命令串):
```bash
# 用 PID 最稳(pgrep 不匹配自己);等价于真机"leader崩/WiFi丢收不到leader"
kill $(pgrep -f '__node:=leader_node')
# 备选(bracket 防 pkill 自杀): pkill -f '__node:=[l]eader_node'
```

**A-3 判据**(盯终端4 / `tail -f /tmp/land2_A.log`,应在 kill 后 ~5s 依次出现):
```
[d0] 通信失联：已 5.x s 未收到 leader (阈值 5.0s) — 触发自动降落...
[d0] LAND settle: 零速悬停 1.5s 后切 AUTO.LAND
[d0] LAND: settle 完成 → 命令 AUTO.LAND，停 offboard 流交给 PX4
```
随后 PX4(终端2)应打:`Landing detected` + `Disarmed by landing`,Gazebo 里飞机**受控垂直下降触地**。
- ✅ **PASS**:上述日志齐全 + 飞机 AUTO.LAND 触地上锁(不是悬停不动、不是坠落)。
- ❌ FAIL:kill 后飞机继续悬停不降(看门狗没触发)/ 未 settle 直接切模式 / 位置乱冲。

一键判读:
```bash
grep -E "已武装|通信失联|LAND settle|命令 AUTO.LAND" /tmp/land2_A.log
```

---

## 4. 测试 B:不误触发(正常飞行全程无降落)  ✅ 反向对照

```bash
# 终端0 清残留(同上一条独立命令)→ 终端1/2/3 重起 → 终端4:
ros2 launch mpc_control swarm_launch.py scenario:=S0_solo1_circle \
     comms_loss_land_s:=5.0 2>&1 | tee /tmp/land2_B.log
```
让它**正常飞满 ~90s**(leader 全程不动它),期间飞机应正常绕圈、绝不降落。
- ✅ **PASS**:全程无 `通信失联`/`LAND` 日志,飞机正常跟踪。
```bash
grep -cE "通信失联|LAND settle" /tmp/land2_B.log     # 判据: 0
```
> 用 circle(运动场景)比 hover 更能暴露"看门狗把正常抖动误判成失联"。若这里误触发,说明
> `_last_leader_rx` 没在每帧更新或阈值逻辑有误。

---

## 5. 测试 C:短暂抖动恢复(可选,低风险)

leader 掉 <5s 后恢复不应触发(每收一帧 `/leader/state` 就刷新计时)。SITL 里干净复现较麻烦
(要 5s 内重启 leader_node),逻辑本身简单(时间戳重置),**可选**。若要做:kill leader 后
在 ~3s 内于终端另起 `ros2 run mpc_control leader_node --ros-args -p num_drones:=1 ...`,
观察不触发降落。多数情况跳过,靠 A/B 两项已足够。

---

## 6. 收尾
```bash
pkill -9 -f '[p]x4'; pkill -9 -f '[g]z sim'; pkill -9 -f 'Micro[X]RCEAgent'; pkill -9 -f '[m]pc_node'; pkill -9 -f '[l]eader_node'
ssh jetson "sudo systemctl start microxrce-agent"     # 若第 0 步停过,恢复真机 Agent
```

---

## 验证记录
| 测试 | 结果 | 关键日志 / 备注 |
|---|---|---|
| A 正确触发(kill leader→5s→AUTO.LAND) | ✅ **PASS** (2026-07-31) | kill leader→**恰好 5.0s**后 `通信失联：已 5.0s 未收到 leader`→`LAND settle 1.5s`→`命令 AUTO.LAND`→PX4 `Landing detected`。solo1 hover。 |
| B 不误触发(circle 90s 无降落) | ✅ **PASS** (2026-07-31) | solo1 circle,leader 全程活着,飞满 93s:`通信失联`计数 **0**、`LAND`计数 **0**、`fallbacks` **0**、末帧 z=-5.00 稳定跟踪。 |
| **C pair2 多机降落** | ✅ **PASS** (2026-07-31) | S2_pair2_hover 两机成队(d0≈(0,0)/d1≈(-3,0))→杀 leader→**两机同时**(相差 10ms)恰好 5.0s 触发→各 settle→AUTO.LAND→**px4_0/px4_1 都触地**。触地水平间距 **3.02m**(≈队形 3.0m > d_safe 1.5)⇒ 垂直降落无收敛、构造上无碰撞。 |
| **D 先冻结再降落** | ✅ **PASS** (2026-07-31) | S0_solo1_circle(移动 leader,`comms_loss_hold_s=1.0`/`land=5.0`)→杀 leader→**恰好 1.0s** 后 `就地悬停,不追陈旧参考`→**冻结窗口内 solve 数=0**(vs 冻结前 29 次,证明真停了没追旧参考)→**恰好 5.0s** 后 `通信失联`→`LAND settle`→`命令 AUTO.LAND`→PX4 `Landing detected`;误恢复=0。 |
| E 抖动恢复(可选) | ⏭ 跳过 | 逻辑简单(每帧刷 `_last_leader_rx`+边沿标志),A~D 已足够。 |

**验证方式**:`ssh sim` 上 headless SITL(gz `-s`,域 0=PX4 SITL 默认,jetson 离线无串台风险),
编排脚本 scratchpad `land2_A.sh`/`land2_B.sh`(仿 `run_one_trial.sh` 的 bringup)。跑完自动 cleanup,
无残留进程。测试就绪判据用"MPC solving"(= armed+OFFBOARD+已收 leader+过校准),不依赖 ready gate。
注:测试 A(hover)不打 `formation ready` 是**设计使然**——`leader_node._should_start_motion` 里 hover 模式
绕过就绪门、用固定 `start_delay`(hover 不移动无需门控);测试 B(circle)就绪门正常开(`formation ready
— starting leader motion at t=10.9s`),证明门本身工作正常、drone0 health 管线通。**非缺陷。**

> 剩余:①同步到 Jetson(`git archive HEAD` + colcon build,见 `jetson-orin-nx-bench`)——**jetson 当前离线**,
> 上线后做;②真机飞场用第2层前,配合真动捕再验一次(SITL 已证逻辑正确)。
