# DOB-MPC 闭环对照数据（论文 §6.5 Table III 行 (c)/(c')）

FxTDO 式加性扰动观测器（Q-filter 无积分器，L=1rad/s 准静态）喂进 legacy 零滞后 OCP（v̇=u+d̂），
成型后接入，cross5/trio3 同 §6.2b 风扰协议，n=5。口径 `tools/repeat_stats.py --wind-window 110 258`，median。

- `conservative_cap0p5/` — d̂ 上限 0.5 m/s²：cross5 峰峰 median **125m**、trio3 **86.5m** ≈ legacy(153/87.7)，够不到 fix 0.61m。
- `aggressive_cap2p0/`  — d̂ 上限 2.0 m/s²：cross5 峰峰 median **168m** > legacy 153m（**正反馈恶化**）。

**机理**：速度环使实现加速度 v̇≈0.1u ⇒ 观测残差 v̇−u≈−0.9u = 命令相关伪扰动(非外源) ⇒ 前馈成正反馈(环增益~1/0.1)。
保守 cap 压回 legacy、激进 cap 放开→更差。**经验证实 N2：加性 DOB 不适用于输入通道滞后，为权限调高反而失稳。**
对照 legacy(a)=report/data/(§6.2b)、fix(d)=0.61m、consensus(e)=../consensus/。
