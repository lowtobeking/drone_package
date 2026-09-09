# Phase-2 续跑备忘（2026-08-13 下班时快照）

## 现在在跑什么
- **M2/M4 批**在仿真机 `setsid nohup` 跑（`~/flights/m2m4/`）。锁屏不影响，~19:10 完。
  - 进度看：`ssh sim "tail ~/flights/m2m4/summary.txt; grep -c 'OK ->' ~/flights/m2m4/summary.txt"`
  - 完成标志：summary.txt 出现 `batch DONE`（应 30 个 OK）。

## 已完成并验证（方案 A）
- **M2 DOB 保守档**（cap 0.5）cross5+trio3 各 n=5：≈legacy、|d̂|~0.7、够不到修复 0.6m。
  - cross5 峰峰 median ~125m（legacy 153）；trio3 median 86.5m（legacy 87.7）。

## 🔴 未决的关键科学问题（回来第一件事）
- **M4 w_f=0（无耦合）早期 n=2 已发散 ~120m ≈ legacy w_f=0.5（153m）** →
  "队形放大"(§5.5) 可能被高估：像是单机滞后本身就发散，耦合只加重 ~1.3×。
- **等 M4 w_f 全扫描（0/0.25/1.0/2.0 各 n=5）** 出 w_f→发散曲线判定：
  - 单调↑ → §5.5 软化成"耦合加重已严重的单机失效"（诚实、仍成立）。
  - 基本平 → §5.5 放大论点大改/撤，**需用户拍板**。
  - 交叉印证：§5.6 拓扑扫描（n=5 实证）已显示随耦合密度单调↑，方向没错。

## Phase-2 队列（代码全部本地 commit，**未 push sim**；M2/M4 完才能 push）
提交：`9b8b035`(solo1) `f4b5834`(激进DOB) `bec01a3`(consensus)。按此优先级跑：
1. **🔑 solo1 基线**（先跑，最关键，低风险 ~45min）：`report/solo1_baseline_batch.sh`
   （solo1 legacy tau=0 + solo1 fix tau=0.5 各 n=5，场景 `S_solo1_circle_nosafety_s1`=同 S34 协议单机版）。
   **判 w_f=0 那 120m 是单机滞后(solo1 legacy 也~120m)还是耦合放大(solo1 legacy 只几米)。**
2. **激进 DOB**（cap 2.0，现成免调 ~33min）：`report/dob_aggressive_batch.sh`
   → 论文报"DOB 保守/激进两档都够不到修复"。
3. **一致性对照**（最后，**需先 smoke 调 kL/kC**，最大变数）：`report/consensus_batch.sh`
   → 判失效是否 MPC 专属。若 consensus 不发散 → 需重构叙事，**找用户**。

## 回来的操作序
```bash
# 0. 确认 M2/M4 完
ssh sim "grep -c 'OK ->' ~/flights/m2m4/summary.txt"   # 应 30
# 1. push 全部本地 commit + build
git push sim main
ssh sim "cd ~/ros2_control_mpc_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select mpc_control"
# 2. 跑 phase-2（串行；solo1 先）
ssh sim "cd ~/ros2_control_mpc_ws/src/mpc_control && source ... && setsid nohup bash report/solo1_baseline_batch.sh > ~/flights/solo1/driver.log 2>&1 &"
#   solo1 完 → dob_aggressive_batch.sh → (smoke 调 consensus 增益) → consensus_batch.sh
# 3. 分析（⚠️ 用排除 _attempt 的 glob；高方差报 median+range）
#   repeat_stats.py --group X ~/flights/.../flight_..._run[1-5].csv --wind-window 110 258
```

## 分析卫生（别踩）
- glob 一律 `run[1-5].csv`（`run*` 会把 `_attempt` 也匹配进去，之前 n 翻倍）。
- 新 CSV 多了 d̂ 列（dN_dhatx/y/z），repeat_stats 按列名安全；出图前确认 make_figures 也按列名。
- 高方差数据（legacy/DOB 5 次里 4 发散 1 侥幸）报 median+range，别 mean±sd。

## 论文待更新（数据齐后）
Table III 补 DOB保守/DOB激进/consensus 行；§5.5 按 w_f 曲线+solo1 定夺；§6.5 方案A措辞；
§7.2 泛化性(consensus)；图文数字对齐(m3)；input-delay MPC 引用(m4)。数据 scp 回 `report/data/`，代码 push GitHub。
