# report/data — 仿真飞行 CSV / flight CSVs

放从 Ubuntu 回传的 `diag_monitor.py --log` 生成的飞行 CSV。
Put the flight CSVs (from `diag_monitor.py --log`) copied back from the Ubuntu host here.

- 命名 / naming：`flight_<formation>_<traj>[_<变体>].csv`
  例 / e.g.：`flight_cross5_line.csv`、`flight_grid9_circle.csv`、`flight_cross5_line_v2.5.csv`
- 列 / columns（每秒一行 / one row per second）：
  `t`,
  每机 per‑UAV `d{i}_x, d{i}_y, d{i}_z, d{i}_zerr, d{i}_velxy, d{i}_arm, d{i}_nav, d{i}_mpc, d{i}_solve_ms, d{i}_fallback, d{i}_hover, d{i}_poserr`,
  汇总 `min_spacing, formation_max_err, safety_violations, total_fallbacks, max_solve_ms, leader_x, leader_y, leader_vx, leader_vy`
  > `d{i}_x / d{i}_y`（世界系 NED 北/东）与 `leader_*` 为本阶段新增，用于俯视轨迹图。
- 出图 / plot：`py report/make_figures.py report/data/<csv> --out report/figures`
- 文本体检 / quick text report：`py analyze_flight.py report/data/<csv>`

> CSV 通常很小（每秒一行），可随仓库提交；如不想入库，把 `report/data/*.csv` 加进 `.gitignore`。

## 论文引用的实验数据 → 复算脚本对照（2026-08-17 补全）

论文里每个量化论断都应能从本目录复算。子目录与脚本的对应关系：

| 子目录 | 内容 | 复算脚本 / 论文位置 |
|---|---|---|
| `tau_n5/` | **A1/A2/B1/B2 各 n=5**（A=trio3 无风 τ0/τ0.5，B=cross5 2N 风 τ0/τ0.5）+ `batch_summary.txt` | `report/stats_significance.py` 的 `legacy_c5`；§6.2b 的 288.1±297.2 / 0.612±0.022 / 78.787±91.344 |
| `topo_sweep_v2/` | 拓扑扫描 trio3/star5/cross5/grid9 各 n=5（时长 300 s ⇒ 窗口 [110,298]） | `stats_significance.py`、`make_paper_figures.py::fig6`；§5.6 |
| `dob/` | DOB 保守 cap0.5（cross5+trio3）/ 激进 cap2.0，各 n=5 | `stats_significance.py`；§6.5 表 III 行 (c)/(c') |
| `consensus/` | 一致性对照：名义增益 + 高增益 kL6.3，calm/wind 各 n=5 | `stats_significance.py`；§6.5 行 (e)、Fig.7 |
| `wf_decomp/` | $w_f$ 扫描 0/0.25/1.0/2.0 + solo1 基线 | §6.2c、表 II-b |
| `exp2_raw/` | 130 Hz 速度环标定原始数据 | `report/tau_goodness.py`；§5.4 的 τ=0.48 / R²=0.999 |
| 根目录 `flight_exp{1,4,5}_*.csv` | 阶段性报告 §5.G 实验1/4/5（S34/S35/S36/S37） | **`report/headline_provenance.py`**；§6.2 的 33.14→0.13 与 0.368→0.035 |
| `review_2026_08_17/` | 顶刊审稿意见补充实验：**① 完全解耦对照** `flight_nocouple_run[1-5]`（场景 S38，$w_f$=0 且 $w_c$=0，机间只剩共享领队参考）；**② PX4 速度环增益扫描** `flight_velp{3.6,5.4}_run[1-3]`（legacy 风扰，增益 2×/3×，`gain_verify_*.txt` 为逐实例启动日志中的生效凭据） | `report/analyze_review_batch.py`；§6.2d（增益扫描）。批跑脚本 `report/review_batch.sh` + 补跑 `report/review_topup.sh`（后者应对已知启动竞态） |

> ⚠️ **`tau_n5/` 是 2026-08-17 才从仿真机补回来的**。此前 `stats_significance.py` 把 legacy cross5
> 的 5 个数**硬编码**在源码里（且注释谎称"由 data/ 现算"），构成循环论证——论文引它、它抄论文。
> 现已改为真读 CSV，并保留发表时的数值作回归基线。
> ⚠️ 取 n=5 时用 `run[1-5].csv`，**别用 `run*`**——会把 `_attempt` 重试文件算进去导致 n 翻倍。
