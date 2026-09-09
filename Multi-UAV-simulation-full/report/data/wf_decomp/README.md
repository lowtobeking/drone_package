# 发散分解数据（论文 §6.2c / §5.5 / N1）

分离"单机种子 vs 耦合放大"的两组受控实验，legacy τ_model=0、2N 风扰、cross5/solo1、n=5。
口径：`tools/repeat_stats.py --wind-window 110 258`，高方差报 median。

- `solo1/` — 单机基线：`flight_solo1_legacy_run[1-5]`（median 49m=发散种子）+ `flight_solo1_fix_run[1-5]`（τ=0.5，0.24m=修复对单机成立）。场景 `S_solo1_circle_nosafety_s1`（同 S34 协议单机版）。
- `m4_wf/` — 队形权重扫描（legacy）：`flight_wf{0p0,0p25,1p0,2p0}_run[1-5]`，median 峰峰 125/112/169/158m。w_f=0.5 那档=§6.2b 的 B1 数据（median 153m），不在此重复。

**分解结论**：单机 49m 种子 → 隐式耦合(w_f=0) 125m(≈2.5×) → 显式队形(w_f>0) 150–170m(再 ≈1.3×)。
w_f=0 不消除发散 ⇒ N1/§5.5 从"耦合制造失稳"软化为"单机已发散、耦合放大(隐式为主)"。
对照 consensus(无预测)不发散见 `../consensus/`。
