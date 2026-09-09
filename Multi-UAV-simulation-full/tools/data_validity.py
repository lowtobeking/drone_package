#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""data_validity.py —— 逐机数据有效性判据（2026-09-01 立）。

为什么需要
----------
`diag_monitor` 在 MPC 的 health 通道停更时，会把**最后一个健康值一直重复写进 CSV**，
而 PX4 的位置仍在更新。结果是该机的 `d{i}_poserr` 变成一条常数线——峰峰值为 0、
均值是个假的常数——而从行数、`arm`/`nav` 标志、脚本退出码上**都看不出任何异常**。
2026-09-01 跑 M1 零模型时，一个五机架次全员如此，直接把中位数拉低了 4%。

⚠️ 本模块只判「这段数字是不是测量值」，**不对发散大小做任何判断**。
   绝不能因为某架散得多就剔除它——那会把结论洗成想要的样子。

三种失效机制（2026-09-01 逐条查原始行分出来的，签名各不相同）
--------------------------------------------------------------
  OVERFLOW   状态估计溢出：x 或 y 跑到 ±2.1e9（≈ ±2³¹），poserr/solve_ms 随之冻结
  NOHEALTH   health 根本没收到（`d{i}_mpc < 0`）：此时 poserr 被写成 0.0 而非 NaN
  STALE      health 通道死亡：poserr 冻结、`hover=0`、飞机还在动
  HOVER      悬停兜底：poserr 冻结但 `hover=1` —— 这是 `_hover_setpoint_world()`
             提前 return 的**已知后果**，状态本身合法，但那段 poserr 是陈旧值，
             同样不能当跟踪/发散的测量用

🔑 **不要用「solve_ms 恒定」单独判**：`controller:=consensus` 旁路 MPC 不解 OCP，
   `solve_ms` 恒为 0.000 是**正确**的。2026-09-01 初版判据因此差点误判 100 个机次。
   本模块只以 `poserr` 冻结为入口，`solve_ms` 仅用于旁证。
"""

OVERFLOW_ABS = 1e6      # |x| 或 |y| 超过即判溢出（真实场地量级是 1e2）
MIN_CONST_RUN = 20      # 冻结段的**绝对**下限（拍；1Hz ⇒ 20 s）
MIN_CONST_FRAC = 0.30   # 冻结段还须占分析窗口的这个比例——真实故障都在 37%-100%
HOVER_FRAC = 0.5        # 窗口内 hover=1 占比超过即判「兜底主导」
NOHEALTH_FRAC = 0.5     # 窗口内 mpc<0（health 未收到）占比超过即判无效

REASONS = ('OVERFLOW', 'NOHEALTH', 'STALE', 'HOVER')


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _longest_const_run(vals):
    best = cur = 1 if vals else 0
    for i in range(1, len(vals)):
        cur = cur + 1 if vals[i] == vals[i - 1] else 1
        if cur > best:
            best = cur
    return best


def agent_reason(rows, d, idx=None, min_const_run=MIN_CONST_RUN):
    """单机有效性。rows=CSV 行列表，d=机号，idx=参与判定的行下标（None=全部）。

    返回 None 表示有效；否则返回 (代码, 人读原因)。
    """
    sel = rows if idx is None else [rows[i] for i in idx]
    if not sel:
        return None

    raw = [r.get('d%d_poserr' % d) for r in sel]
    raw = [v for v in raw if v not in (None, '')]
    if len(raw) < min_const_run:
        return None                      # 样本太少，不做判断（短场景/缺列）

    xs = [_f(r.get('d%d_x' % d)) for r in sel]
    ys = [_f(r.get('d%d_y' % d)) for r in sel]
    xy = [v for v in xs + ys if v is not None]
    if xy and max(abs(v) for v in xy) > OVERFLOW_ABS:
        return ('OVERFLOW', '状态估计溢出（|x| 或 |y| > %g）' % OVERFLOW_ABS)

    # 🔑 最直接的一条：`d{i}_mpc < 0` = **health 根本没收到**，此时 diag_monitor 把
    #    poserr 写成 0.0 而不是 NaN。2026-08-30 复算 §6.6 真机 circle 时就栽在这里：
    #    124 个 OFFBOARD 行里 92 行 mpc=-1，均值被稀释成 1.3 cm，实为 3.9 cm。
    #    这是个显式标志位，不是推断，故排在冻结判据之前。
    mpc = [_f(r.get('d%d_mpc' % d)) for r in sel]
    mpc = [v for v in mpc if v is not None]
    if mpc:
        frac = sum(1 for v in mpc if v < 0) / float(len(mpc))
        if frac > NOHEALTH_FRAC:
            return ('NOHEALTH', 'health 未收到（mpc<0）占窗口 %.0f%% ⇒ poserr 被写成 0.0 非测量值'
                    % (100.0 * frac))

    # 🔑 只凭 poserr 冻结**不能**判定，而且两个通道的冻结段必须**重叠**。
    #    2026-09-01 初版判据犯了两个错，把 16 个历史文件误剔（其中
    #    `flight_grid9_circle_run4.csv` 是完全健康的 2868 行架次，poserr/solve_ms
    #    全程都在变、mpc=0、hover=0，却被全员剔除）：
    #      ① poserr 与 solve_ms 的最长恒定段各自独立算，两段可以毫不相干；
    #      ② 只卡绝对长度 20 拍，长架次里偶然凑够太容易。
    #    ⇒ 改为对 **(poserr, solve_ms) 二元组**求最长恒定连段（要求同时冻结），
    #      且该段须占窗口 MIN_CONST_FRAC 以上。真实故障都在 37%–100%
    #      （M1 null5_run2 100%、wf2p0_run4 58%、W1 61%、grid9_run3 d5 37%）。
    #    两种机制的签名不同，必须分别判，不能合并成一条：
    pe_run = _longest_const_run(raw)
    need = max(min_const_run, int(MIN_CONST_FRAC * len(raw)))

    # ── HOVER：悬停兜底 ──────────────────────────────────────────────────────
    # `_hover_setpoint_world()` 那条路径提前 return，**poserr 不再重算**，但 MPC
    # 仍在解 ⇒ solve_ms 照常变化。所以这一支**不能**要求 solve_ms 同时冻结
    # （2026-09-01 一度因此漏掉 wf2p0_run4 的 d1–d4）。
    # 判据 = poserr 长时间冻结 + hover 兜底主导。参考在动而机体被 parked 时，
    # poserr 本应随参考起伏，恒定只可能是陈旧值。
    hov = [_f(r.get('d%d_hover' % d)) for r in sel]
    hov = [v for v in hov if v is not None]
    hov_dom = bool(hov) and sum(1 for v in hov if v > 0.5) > HOVER_FRAC * len(hov)
    if pe_run >= need and hov_dom:
        return ('HOVER', 'poserr 冻结 %d 拍且 hover 兜底主导 —— 该段 poserr 是陈旧值' % pe_run)

    # ── STALE：health 包整体停更 ─────────────────────────────────────────────
    # 此时 poserr 与 solve_ms **同时**冻结（diag_monitor 重复写最后一条）。
    # 要求两者冻结段**重叠**（对二元组求连段），否则长架次里偶然各自凑够就会误剔。
    sv = [r.get('d%d_solve_ms' % d) for r in sel]
    if len(sv) != len(sel) or any(v in (None, '') for v in sv):
        return None
    # ⚠️ consensus 律旁路 MPC，solve_ms 恒为 0 是**正确**的 ⇒ 此时 solve_ms 不具
    #    佐证力，不能据此剔除（初版曾因此差点误判 100 个机次）。
    if all(_f(v) == 0.0 for v in sv):
        return None
    pe_raw = [r.get('d%d_poserr' % d) for r in sel]
    joint = [(pe_raw[i], sv[i]) for i in range(len(sel)) if pe_raw[i] not in (None, '')]
    run = _longest_const_run(joint)
    if run < max(min_const_run, int(MIN_CONST_FRAC * len(joint))):
        return None
    return ('STALE', 'poserr+solve_ms 连续 %d 拍不变（health 通道已停更）' % run)


def scan(rows, ndr, idx=None, min_const_run=MIN_CONST_RUN):
    """返回 {drone_id: (代码, 原因)}，只含**无效**的机。"""
    bad = {}
    for d in range(ndr):
        why = agent_reason(rows, d, idx, min_const_run)
        if why:
            bad[d] = why
    return bad


def format_exclusions(bad, prefix='  '):
    """把 scan() 的结果排成人读的剔除清单。"""
    if not bad:
        return []
    return ['%s⚠️ 剔除 d%d [%s]：%s' % (prefix, d, code, why)
            for d, (code, why) in sorted(bad.items())]
