# topo_sweep_v3 — §5.6 拓扑扫描（第三轮，2026-08-18）

论文 §5.6 的受控拓扑对照实验数据。**本目录取代 `../topo_sweep_v2/`**，原因见下。

## 协议

四个拓扑完全统一，只有拓扑（与随之而来的机数）不同：

| 项 | 值 |
|---|---|
| 控制器 | legacy，`vel_lag_tau = 0`（即 MPC 内部模型断言 τ=0） |
| leader | circle，speed **0.5 m/s**，radius 10 m，yaw_mode center |
| 录制时长 | **300 s**（统计窗口 `t ∈ [110, 298]`） |
| 风扰 | 录制开始后 **+100 s** 起，对每架机施加 **2.0 N** 东向恒力（Gazebo） |
| 安全滤波器 | 关（`safety_filter_enable: false`） |
| 避碰权重 | `w_collision = 200`（四组一致，见下） |
| 重复 | 每拓扑 **n = 5** 次独立试验（整栈重启，非仅重启控制器） |
| 代码 | 全程同一 commit `90cad8f` |

场景：`S_trio3_circle_nosafety_v05` / `S_star5_circle_nosafety_v05` /
`S_cross5_circle_nosafety_v05_wc200` / `S_grid9_circle_nosafety`。

## 文件

- `flight_{trio3,star5,cross5,grid9}_run{1..5}.csv` — 四拓扑主批。
  `cross5` 一组用的是**带 `w_collision:500` 的原场景**，保留作为避碰权重对照。
- `flight_cross5wc200_run{1..5}.csv` — **配平组**：`w_collision = 200`，其余一切不变。
  **论文 §5.6 表格与 Fig 6 的 `cross5` 行取自这一组。**
- `summary.txt` — 批跑日志（含每次试验的 verdict 与列数校验结果）。

分析脚本：`report/topo_v3_stats.py`（两个口径 + 单侧 Mann–Whitney + Cliff's δ）；
`report/stats_significance.py` 亦已切到本目录。

## 🔴 为什么重跑：v2 的 grid9 只记录了 9 架机中的 5 架

v2（2026-07-16）的 grid9 组**确实起了 9 架机**——trial 日志里有
`all 9/9 PX4 instances ready` 与 `9/9 mpc_node processes confirmed alive`，
且 drone0…drone8 逐个回读过参数。但当时 `report/run_one_trial.sh` 里编队名是由**机数**推出的，
且只区分「3」与「非 3」，fn=9 落到了 `cross5` ⇒ `diag_monitor` 只开了 5 架机的列，
d5…d8 全程没有进 CSV。9 → `grid9` 的映射是 2026-08-13 commit `9b8b035` 才补上的。

⇒ **v2 的 grid9 统计量实为 9 机中的 5 机子样本**（恰好是 grid9 的中心十字，
因为 grid9 的前 5 个出生点与 cross5 逐点相同）。其余三组（trio3/star5/cross5）记录完整。

本轮的 `topology_sweep_v3.sh` 因此新增了一条校验：**CSV 里 `_poserr` 列数必须等于该组机数**，
不符即作废重试。`topo_v3_stats.py` 与 Fig 6 的生成函数里也各有一份同样的断言。

## 🔴 第二个混杂：cross5 的避碰权重与其余三组不同

`S_cross5_circle_nosafety_v05` 自 2026-07-16 引入（commit `a2cca15`）起就带
`w_collision: 500.0`，而 trio3 / star5 / grid9 用 `defaults` 的 **200.0**。
而避碰项在这些试验里确实在起作用——多数试验窗口内的最小机间距跌破 `d_safe = 1.5 m`
（star5 0.42–1.27 m、cross5 0.44–1.53 m、grid9 0.27–0.70 m），`safety_violations` 8–45 次。
⇒ 原四拓扑对比并非「只差拓扑」。故补跑了 `cross5wc200`。

实测该权重**无可检测效应**（按机峰峰中位数 94.2 m @200 vs 104.0 m @500，
双侧 Mann–Whitney p = 0.69，Cliff's δ = +0.20），但它明显影响离散度
（@500 那组按机范围 [101.1, 107.6] m，@200 为 [91.8, 182.2] m）。

## 结果摘要（按机口径，中位数 [范围]，m）

| 拓扑 | λmax | 机数 | 按机峰峰 | 最坏机峰峰 |
|---|---|---|---|---|
| trio3 | 3.000 | 3 | 77.3 [65.3, 92.7] | 90.7 |
| star5 | 3.618 | 5 | 130.5 [108.7, 187.6] | 181.8 |
| cross5 (wc200) | 5.000 | 5 | 94.2 [91.8, 182.2] | 118.1 |
| grid9 | 6.000 | 9 | 155.6 [111.6, 193.5] | 410.3 |

**预测排序 trio3 < star5 < cross5 < grid9 未被复现**：star5 与 cross5 对调，
而机数与权重都配平的那一对（star5 vs cross5@200）在 n=5 下两个方向都不显著
（单侧 p = 0.11 按机 / 0.15 最坏机）。所有**机数变大**的比较则全部 p = 0.004、δ = −1.0。
⇒ 严重度跟随机队规模，而非耦合密度；与 §6.2c（去掉全部显式机间耦合后五机仍放大）同向。

## v2 是否应当删除

不删。保留 `../topo_sweep_v2/` 作为历史记录，但**不得再用于任何统计**——
其 grid9 组的机数与目录名不符。若要复算，只用本目录。
