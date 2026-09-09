# 任务完成自动降落（第 1 层）SITL 验证记录

> 2026-07-29。功能代码见 commit `aef6af7`。本文记录 solo1→grid9 全规模 SITL 验证结果。

## 1. 设计（第 1 层：单点触发、各机独立执行）

机数上去后不能再靠人工 Ctrl+C 逐台接管（且 SITL 里 Ctrl+C 会把整个仿真栈端掉炸机）。
第 1 层原则：**降落时去掉机间协同**——协同在飞行段用，降落段是负资产。各机原地垂直
下降，水平间距 `d_safe` 已在飞行段拉开 ⇒ 构造上无碰撞。

- **触发**：leader 在 `mission_duration` 到点（或运行时 `trigger_land` 手动一键）广播 `LAND`
  到 `/swarm/mission`（latched TRANSIENT_LOCAL + RELIABLE，短暂掉线/晚加入的机也能收到）。
- **执行**：每架 `mpc_node` 收到 `LAND` → 零速悬停 `land_settle_s`（默认 1.5s）刹掉编队速度
  → 命令**本机** `DO_SET_MODE(AUTO.LAND)` → 停 offboard 流交给 PX4 自主降落 + 触地自动上锁。
  终态每 1s 重发直到 nav 确认 `AUTO_LAND`（复用已验证的 RELINQUISH 停流模式）。
- 默认 `mission_duration=0` 禁用，不影响现有 41 场景回归/批跑。

## 2. 全规模验证结果（solo1→grid9，hover 场景 + mission_duration）

| 规模 | 场景 | 收到 LAND | 命令 AUTO.LAND | Landing detected | Disarmed by landing | 触地 min 间距 | 队形设计间距 |
|---|---|---|---|---|---|---|---|
| solo1 (1) | S0_solo1_hover | 1/1 | 1/1 | 1/1 | **1/1** | N/A | — |
| pair2 (2) | S2_pair2_hover | 2/2 | 2/2 | 2/2 | **2/2** | 3.03m | 3.0m |
| trio3 (3) | S3_trio3_hover | 3/3 | 3/3 | 3/3 | **3/3** | 5.17m | 5.196m |
| cross5 (5) | S6_cross5_hover | 5/5 | 4/5 ※ | 5/5 | **5/5** | 2.94m | 3.0m（中心-臂）|
| grid9 (9) | S7_grid9_hover | 9/9 | 9/9 | 9/9 | **9/9** | 2.92m | 3.0m（晶格）|

**结论：1→9 全部各机自主降落 + 触地自动上锁，无人工接管。触地最小间距 ≈ 队形设计间距，
垂直下降零收敛 ⇒ 构造上无碰撞（实测证实，不再只是论证）。**

### 关键指标（trio3 取证）
- **同步性**：三机收到 `LAND` 时间戳相差仅 **0.27ms**（latched 广播 → 全体近乎同一瞬间），
  命令 AUTO.LAND 相差 **6ms**。单点触发 → 全体同时，不是逐台。
- **降落是显式命令触发，非失联兜底**：SITL 的 `COM_OBL_RC_ACT=5` 是 Hold（失联只会悬停不降落），
  实际**降落了**⇒ 证明是主动发的 `DO_SET_MODE(AUTO.LAND)` 生效。SITL 测的是更严格的路径；
  真机 `COM_OBL_RC_ACT=4(Land)` 命令 + 兜底双保险更稳。

## 3. 已知事项 / 踩坑

- **※ cross5 "命令 AUTO.LAND=4/5"** 是其中 1 台的 `settle 完成` 日志行没被抓全（日志时序），
  PX4 console 的 Landing/Disarmed 都是 5/5，是硬证据——5 台确实全降落上锁。grid9 该项 9/9
  完整，进一步印证是日志计数问题而非行为缺陷。
- **SITL 多机 startup race**：cross5 首次只 4/5 PX4 就绪（一台 straggler），重试即过。属已知
  SITL 非确定性（报告 §5.H S2 同类），非降落代码问题；`run_one_trial.sh` 本就靠重试规避。
- **domain 0 隔离（真机在线时必做）**：SITL 的 drone0 用**非命名空间** `/fmu`（`topic_for_drone(0)`），
  与真机 X6 Air+（domain 0，同用 `/fmu`）冲突——SITL 的 mpc_node 会把 ARM/LAND 指令发到真飞控。
  验证时必须让真机离开 domain 0：**停 Jetson 的 `microxrce-agent`**（agent 是 FC↔DDS 唯一桥，
  停了真机即从 domain 0 消失，可逆）或直接给 FC 断电。每次测完记得恢复。
- **仿真机中途 spurious reboot 一次**：非 OOM（30GB RAM、26GB 空闲），journald 非持久化未捕获
  crash 原因，疑似电源/热/驱动瞬时事件（伴随 WiFi 抖动）。重启后首次 gz sim 未起来一次，
  机器 settle 后恢复正常。

## 4. 尚未做（真机相关，SITL 无关）

1. **Jetson 代码同步**：Jetson 非 git 仓库，`aef6af7` 需手动同步（`git archive`+scp+colcon build）
   才能在真机上用第 1 层降落。
2. **写进飞场收场流程**：`report/飞场当天流程.md` §6 应急处置目前是人工 Ctrl+C；可补一条
   "任务完成 → leader `trigger_land` 或 mission_duration 到点 → 全体自主降落"。
3. **第 2 层（失联兜底）**：`COM_OBL_RC_ACT=4(Land)` 已在真机配好（07-23），可作为独立验证——
   companion 全崩时各 FC 独立降落。真机降落依赖动捕位置估计，只能到飞场验。
