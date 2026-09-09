#!/usr/bin/env python3
"""
verdict.py — 读 diag_monitor --log 的飞行 CSV，按 config/scenarios.yaml 的 thresholds
             自动判 PASS/FAIL。scenarios.yaml 头部一直宣称有这个工具，实际从未存在
             （2026-07-10 补齐）。

用法:
    python3 tools/verdict.py flight_S3.csv --scenario S3_trio3_circle_rh
    python3 tools/verdict.py --batch runs.tsv          # 每行 "<csv>\t<scenario>"
    python3 tools/verdict.py flight.csv --scenario S5_cross5_circle --json

判定量（全部取自 diag CSV 现成列，不重新推导几何）:
    min_spacing        全程最小值           > thr
    safety_violations  末值(累计)           <= thr
    total_fallbacks    末值(累计)           <= thr
    solve_ms           max_solve_ms 列的全程最大 < thr
                       （该列本身是「当前秒跨机最大」，全程最大由本脚本对整列取 max）
    pos_err            稳态窗内 max_i mean(d{i}_poserr)  < thr

阈值来源: scenarios.yaml 顶层 `thresholds:` + `scenarios.<id>.thresholds` 覆盖。

⚠️ 两类场景的判据**不同源**（2026-07-10 自检时发现）
  · 常规场景 (S0–S26, S32–S41)：CSV 阈值即判据，本工具可独立定案。
  · 故障注入场景 (S27–S31)：`report/data/safety_logs_S27-S33_20260703.txt` 头部写明
    "判定依据：诱发场景以节点日志 SAFETY 状态链为准"。这些场景**故意**触发
    collide_warn / flyaway / RELINQUISH，safety_violations>0 与 min_spacing 下探是
    预期行为而非失败。对它们只跑 CSV 阈值会误判 FAIL（实测 S29 min_spacing=0.957
    vs yaml 写的 1.3 —— yaml 那个数字从未被任何代码执行过，与归档数据不符）。
    → 这类场景在 yaml 里标 `criteria: log`，本工具报 LOG-REQUIRED，需 --log 补证。
    绝不通过下调阈值让历史数据变绿：那是拿判据拟合数据。

⚠️ 口径注记
  · 稳态窗 = t > t0 + steady_offset（默认 45s）。leader 带 ready_hold 的场景
    (S3_*_rh / S7 / S8 = 30s hold) 请用 --steady-offset 55 以上，否则会把成型段算进去。
    S15_cross5_conservative（保守 max_accel=2.0）收敛尾段显著更长，2026-07-17 实测
    t≈266s 才真正收敛（之前是真实、单调下降，不是发散），请用 --steady-offset 270
    以上 + duration_s>=350（见 scenarios.yaml 该场景注释），默认 45s 会把收敛中段
    错判成稳态而 FAIL。
  · diag CSV 的 t 列是 `f'{time.time()-t0:.1f}'`，即**真实经过秒**，由 1.0s 定时器驱动
    （不是行号：丢拍时会出现 dt=0.7/1.3 的抖动，行号不会）。本工具校验 1Hz 节拍。
  · 长文件（如 flight_grid9_circle_run5.csv 7915 行）是 diag_monitor 挂了 2.2 小时，
    飞行本体只占开头 ~200s，其余是空转 —— 不要直接喂给本工具。
  · **min_spacing 口径已于 2026-07-10 变更**：从"只枚举硬编码邻居边"改为 C(n,2)
    全对枚举。新跑的值会系统性 <= 历史基线（多看了非邻居对角对）。跨版本比较时注意。
"""
import argparse
import collections
import csv
import io
import itertools
import json
import math
import os
import re
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import data_validity  # noqa: E402  （逐机有效性判据，见该文件头注）

YAML_DEFAULT = os.path.join(HERE, '..', 'config', 'scenarios.yaml')


def _f(x):
    try:
        v = float(x)
        return v if v == v and abs(v) != float('inf') else None
    except (TypeError, ValueError):
        return None


def load_thresholds(yaml_path, scenario):
    """→ (thresholds, criteria)。criteria='csv'(默认) 或 'log'。"""
    import yaml
    with io.open(yaml_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    thr = dict(cfg.get('thresholds', {}))
    crit = 'csv'
    if scenario is not None:
        if scenario not in cfg.get('scenarios', {}):
            raise SystemExit(f'✗ scenarios.yaml 里没有场景 "{scenario}"')
        scen = cfg['scenarios'][scenario]
        thr.update(scen.get('thresholds', {}) or {})
        crit = scen.get('criteria', 'csv')
    return thr, crit


SAFE_RE = re.compile(r'\[d(\d+)\]\s+SAFETY\s+([A-Z]+)')


def scan_log(log_path):
    """数节点日志里的 SAFETY 状态链。故障注入场景的真正判据。"""
    counts = collections.Counter()
    reasons = collections.Counter()
    with io.open(log_path, encoding='utf-8', errors='replace') as f:
        for ln in f:
            m = SAFE_RE.search(ln)
            if not m:
                continue
            counts[m.group(2)] += 1
            for r in re.findall(r"'([a-z_]+)\(", ln) or re.findall(r"'([a-z_]+)'", ln):
                reasons[r] += 1
    return counts, reasons


def _recompute_spacing(rows, ndr, t_from=None):
    """从 d{i}_x/y/z 列按 C(n,2) 全对重算最小间距（3D，同 diag_monitor 的 norm(pos_i-pos_j)）。

    为什么必须重算：基线 CSV 的 min_spacing 列是**旧口径**（只枚举硬编码邻居边）写下的。
    cross5 旧口径只看中心-臂 4 对 / 10 对，star5 只看环上 5 对，grid9 只看 12 条晶格边 ——
    非邻居对角对从不入账。直接拿新跑的全对值去和旧列做差 = 假 DRIFT。
    solo1/pair2/trio3 恰好新旧同义（旧对已是全对），可用作本函数的自检点。
    """
    if ndr < 2:
        return None, None
    pairs = list(itertools.combinations(range(ndr), 2))
    best, best_t = float('inf'), None
    for r in rows:
        if t_from is not None:
            tt = _f(r['t'])
            if tt is None or tt <= t_from:
                continue
        pos = []
        for d in range(ndr):
            p = (_f(r.get(f'd{d}_x')), _f(r.get(f'd{d}_y')), _f(r.get(f'd{d}_z')))
            pos.append(None if any(v is None for v in p) else p)
        for i, j in pairs:
            if pos[i] is None or pos[j] is None:
                continue          # 未收到该机状态：同 diag_monitor，跳过该对
            dist = math.sqrt(sum((pos[i][k] - pos[j][k]) ** 2 for k in range(3)))
            if dist < best:
                best, best_t = dist, _f(r['t'])
    return (best if best < float('inf') else None), best_t


def measure(csv_path, steady_offset, trust_column=False):
    with io.open(csv_path, newline='', encoding='utf-8', errors='replace') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f'✗ 空日志: {csv_path}')

    t = [_f(r['t']) for r in rows]
    if any(v is None for v in t) or len(t) < 3:
        raise SystemExit(f'✗ t 列不可用: {csv_path}')
    dts = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    med_dt = st.median(dts)
    if not (0.8 <= med_dt <= 1.25):
        raise SystemExit(
            f'✗ {csv_path}: t 列中位步长 {med_dt:.4f}s，不是预期的 1Hz。'
            ' 拒绝静默换算——请确认这是 diag_monitor 的输出而非 flights- 副仓的高频 logger。')

    ndr = sum(1 for c in rows[0] if c.endswith('_poserr'))
    t0 = t[0]
    steady = [i for i, tt in enumerate(t) if tt > t0 + steady_offset]

    m = {'n_rows': len(rows), 'n_drones': ndr, 'duration_s': t[-1] - t[0],
         'steady_rows': len(steady)}

    sp = [(_f(r['min_spacing']), t[i]) for i, r in enumerate(rows)]
    sp = [(v, tt) for v, tt in sp if v is not None]
    col_sp, col_t = min(sp, key=lambda x: x[0]) if sp else (None, None)
    m['min_spacing_col'] = col_sp

    rc_sp, rc_t = _recompute_spacing(rows, ndr)
    m['min_spacing_recomp'] = rc_sp
    # 稳态窗最小值：剔除起飞成型暂态，方差小得多 → 用于对基线做差
    m['min_spacing_steady'], _ = _recompute_spacing(rows, ndr, t_from=t0 + steady_offset)
    if trust_column or rc_sp is None:
        m['min_spacing'], m['min_spacing_t'] = col_sp, col_t
    else:
        m['min_spacing'], m['min_spacing_t'] = rc_sp, rc_t
    # 列值应 >= 全对重算（全对是旧对的超集）。CSV 坐标只存 3 位小数 ⇒ 重算有 ~1e-3 舍入噪声，
    # 故容差取 5e-3；真正的口径不一致会差 0.1 m 量级。
    # 实测（2026-07-10）：star5/cross5/grid9 的归档基线上两者仅差 ~7e-4 —— 最近的一对
    # 始终是邻居对，非邻居对角(3√2=4.24m)从未成为最小值，与 Simulink 规模研究结论一致。
    if col_sp is not None and rc_sp is not None and col_sp < rc_sp - 5e-3:
        m['spacing_warn'] = f'列值 {col_sp:.3f} < 全对重算 {rc_sp:.3f}，口径存疑'

    for key in ('safety_violations', 'total_fallbacks'):
        vals = [_f(r[key]) for r in rows if _f(r[key]) is not None]
        m[key] = vals[-1] if vals else None       # 累计计数 → 取末值

    ms = [_f(r['max_solve_ms']) for r in rows if _f(r['max_solve_ms']) is not None]
    m['solve_ms'] = max(ms) if ms else None

    # pos_err: 稳态窗内，每机取均值，再对机取最大（= "各机跟踪误差"的最坏者）
    #
    # 🔑 2026-09-01：先剔除**非测量**的机（health 冻结 / 状态溢出 / 悬停兜底主导），
    #    见 tools/data_validity.py 的头注。此前这些机的"陈旧常数"会照常参与 max()，
    #    既可能虚高也可能虚低，而且从退出码与行数上完全看不出来。
    #    `pos_err_all_agents` 保留旧口径的值，便于与历史基线对照。
    bad = data_validity.scan(rows, ndr, idx=steady)
    per_drone, per_drone_all = [], []
    for d in range(ndr):
        vals = [_f(rows[i][f'd{d}_poserr']) for i in steady]
        vals = [v for v in vals if v is not None]
        if not vals:
            continue
        mean_d = st.mean(vals)
        per_drone_all.append(mean_d)
        if d not in bad:
            per_drone.append(mean_d)
    m['pos_err'] = max(per_drone) if per_drone else None
    m['pos_err_per_drone'] = per_drone
    m['pos_err_all_agents'] = max(per_drone_all) if per_drone_all else None
    m['invalid_agents'] = {d: bad[d][0] for d in bad}
    m['invalid_agents_detail'] = data_validity.format_exclusions(bad)
    if bad and not per_drone:
        # 全员无效 ⇒ 宁可让判定变成 n/a，也不要用一堆陈旧常数编一个数出来
        m['pos_err_note'] = '全部 %d 架机的数据均非测量值 ⇒ pos_err 置 None' % ndr
    return m


# (阈值名, 取哪个量, 比较方向)  '<' = 实测须小于阈值; '<=' = 须不超过
RULES = [
    ('min_spacing',       'min_spacing',       '>'),
    ('pos_err',           'pos_err',           '<'),
    ('solve_ms',          'solve_ms',          '<'),
    ('safety_violations', 'safety_violations', '<='),
    ('total_fallbacks',   'total_fallbacks',   '<='),
]


def judge(m, thr):
    results = []
    for name, key, op in RULES:
        if name not in thr:
            continue
        got, lim = m.get(key), thr[name]
        if got is None:
            results.append((name, None, lim, op, None))
            continue
        ok = (got > lim) if op == '>' else (got < lim) if op == '<' else (got <= lim)
        results.append((name, got, lim, op, ok))
    known = [r[4] for r in results if r[4] is not None]
    verdict = 'PASS' if known and all(known) else ('FAIL' if known else 'NODATA')
    return verdict, results


# 基线对比容差：Phase A（改动关闭时应行为中性）的判据。
#
# ⚠️ 这些数字**不是统计标定**。全部 52 个归档 CSV 里，真正"同配置重跑且两次都是良品"
#    的只有 S7_grid9_line v4/v5 这一对（n=2）。其余看着像重跑的（v1..v3、
#    grid9_circle_run5/6/7）都是调试收敛链，v1 有 517 次 fallback，不可用作散布估计。
#
# ⚠️ 为什么 min_spacing 用**稳态窗最小值**且是**单边**门（2026-07-10 Phase A 实证）：
#    · 全程最小值几乎总落在起飞成型的暂态里（S3 新跑 @t=24s），是个极值统计量，
#      对 EKF 初始化/线程调度的随机性极敏感 —— 实测同代码 Δ=0.605m，而稳态窗只差 0.18m。
#      拿全程最小值做 ±0.10 的双边门，必然假 DRIFT。
#    · 间距**变大**从不是安全问题，只有变小才是 ⇒ 中性门对安全裕度只能单边。
#      全程最小值仍由绝对阈值 thresholds.min_spacing 把关，那才是它该管的事。
#    · pos_err 取稳态窗均值（对时间平均过），方差小得多：S0 Δ=0.004、S3 Δ=0.023、
#      S7 v4/v5 Δ=0.064 ⇒ 双边 0.10 站得住。
BASELINE_TOL = {'pos_err': 0.10}                    # 双边
BASELINE_LOWER = {'min_spacing_steady': 0.30}       # 单边：新值 >= 基线 - tol（变大不算漂）
BASELINE_SLACK = {'safety_violations': 0, 'total_fallbacks': 2}   # 单边：新 <= 基线 + slack


def diff_baseline(m, mb):
    """→ [(量, 新, 基线, Δ, ok)]；ok=None 表示不参与定案。"""
    out = []
    for k, tol in BASELINE_TOL.items():
        a, b = m.get(k), mb.get(k)
        if a is None or b is None:
            out.append((k, a, b, None, None))
        else:
            out.append((k, a, b, a - b, abs(a - b) <= tol))
    for k, tol in BASELINE_LOWER.items():
        a, b = m.get(k), mb.get(k)
        ok = None if (a is None or b is None) else (a >= b - tol)
        out.append((k, a, b, (None if ok is None else a - b), ok))
    for k, slack in BASELINE_SLACK.items():
        a, b = m.get(k), mb.get(k)
        ok = None if (a is None or b is None) else (a <= b + slack)
        out.append((k, a, b, (None if ok is None else a - b), ok))
    # 只展示、不定案：solve_ms 受机器负载支配跨跑不可比（绝对预算由 thresholds 管）；
    # min_spacing 全程最小值是暂态极值（见上），同理只展示。
    for k in ('min_spacing', 'solve_ms'):
        a, b = m.get(k), mb.get(k)
        out.append((k, a, b, (a - b) if (a is not None and b is not None) else None, None))
    return out


def run_one(csv_path, scenario, yaml_path, steady_offset, log_path=None, quiet=False,
            baseline_csv=None, trust_column=False):
    thr, crit = load_thresholds(yaml_path, scenario)
    m = measure(csv_path, steady_offset, trust_column)
    verdict, results = judge(m, thr)

    if crit == 'log':
        # 故障注入场景：CSV 阈值降级为参考量，定案要看 SAFETY 状态链。
        if log_path:
            counts, reasons = scan_log(log_path)
            m['safety_states'] = dict(counts)
            m['safety_reasons'] = dict(reasons)
            verdict = 'LOG-SEEN'
        else:
            verdict = 'LOG-REQUIRED'

    if not quiet:
        print(f'\n== {scenario or "(无场景)"}  ←  {os.path.basename(csv_path)} ==')
        print(f'   {m["n_drones"]} 机, {m["duration_s"]:.0f}s, 稳态窗 {m["steady_rows"]} 行 '
              f'(t > t0+{steady_offset:g}s)' + ('   [判据=日志]' if crit == 'log' else ''))
        # 剔除清单永远显式打印——静默剔除比不剔除更危险
        for line in m.get('invalid_agents_detail', []):
            print('  ' + line.strip())
        if m.get('invalid_agents'):
            _old = m.get('pos_err_all_agents')
            _olds = 'n/a' if _old is None else ('%.3f' % _old)
            print('   （pos_err 已按上述剔除后计算；旧口径含全部机 = %s）' % _olds)
        if m.get('pos_err_note'):
            print('   ⚠️ ' + m['pos_err_note'])
        for name, got, lim, op, ok in results:
            if crit == 'log':
                tag = ' ref  '          # 参考量，不参与定案
            else:
                tag = '  ??  ' if ok is None else (' PASS ' if ok else '*FAIL*')
            gs = 'n/a' if got is None else f'{got:.3f}'
            print(f'   [{tag}] {name:18s} {gs:>10s}  {op} {lim}')
        if m.get('min_spacing_t') is not None:
            src = '列' if trust_column else '全对重算'
            extra = ''
            if m.get('min_spacing_col') is not None and m.get('min_spacing_recomp') is not None:
                extra = f'；CSV 列值 {m["min_spacing_col"]:.3f}（旧口径可能只数邻居边）'
            print(f'          (min_spacing[{src}] @ t={m["min_spacing_t"]:.0f}s{extra})')
        if m.get('spacing_warn'):
            print(f'   ⚠ {m["spacing_warn"]}')
        if 'safety_states' in m:
            print(f'   SAFETY 状态链: {m["safety_states"] or "（无）"}')
            if m['safety_reasons']:
                print(f'   触发原因: {m["safety_reasons"]}')

    if baseline_csv:
        mb = measure(baseline_csv, steady_offset, trust_column)
        dl = diff_baseline(m, mb)
        m['baseline'] = os.path.basename(baseline_csv)
        bad = [d for d in dl if d[4] is False]
        verdict = 'DRIFT' if bad else 'NEUTRAL'
        if not quiet:
            print(f'   ── 对基线 {os.path.basename(baseline_csv)} 做差 ──')
            for k, a, b, d, ok in dl:
                tag = '  --  ' if ok is None else (' same ' if ok else '*DRIFT*')
                fa = 'n/a' if a is None else f'{a:.3f}'
                fb = 'n/a' if b is None else f'{b:.3f}'
                fd = '' if d is None else f'  Δ={d:+.3f}'
                print(f'   [{tag}] {k:18s} {fa:>9s}  vs {fb:>9s}{fd}')
            print(f'   VERDICT: {verdict}')
    elif not quiet:
        print(f'   VERDICT: {verdict}')
    return verdict, m, results


def main():
    ap = argparse.ArgumentParser(description='按 scenarios.yaml thresholds 自动判 PASS/FAIL')
    ap.add_argument('csv', nargs='?', help='diag_monitor --log 的 CSV')
    ap.add_argument('--scenario', help='scenarios.yaml 里的场景 id（取其 thresholds 覆盖）')
    ap.add_argument('--yaml', default=YAML_DEFAULT)
    ap.add_argument('--steady-offset', type=float, default=45.0,
                    help='稳态窗起点(秒)，默认 45；ready_hold=30 的场景请给 55+')
    ap.add_argument('--log', help='节点日志；criteria=log 的场景据此数 SAFETY 状态链')
    ap.add_argument('--baseline', help='基线 CSV：改判为对基线做差（NEUTRAL / DRIFT）')
    ap.add_argument('--batch',
                    help='TSV: 每行 "<csv>\\t<scenario>[\\t<steady_offset>[\\t<log>[\\t<baseline>]]]"')
    ap.add_argument('--trust-column', action='store_true',
                    help='直接用 CSV 的 min_spacing 列，不从坐标重算（默认重算以跨版本可比）')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    def _n(v):
        return '     n/a' if v is None else f'{v:8.3f}'

    if a.batch:
        rows, nfail = [], 0
        with io.open(a.batch, encoding='utf-8') as f:
            for ln in f:
                ln = ln.strip()
                if not ln or ln.startswith('#'):
                    continue
                parts = ln.split('\t')
                cpath, scen = parts[0], (parts[1] if len(parts) > 1 else None)
                so = float(parts[2]) if len(parts) > 2 and parts[2] else a.steady_offset
                lg = parts[3] if len(parts) > 3 and parts[3] else a.log
                bl = parts[4] if len(parts) > 4 and parts[4] else a.baseline
                try:
                    v, m, _ = run_one(cpath, scen, a.yaml, so, lg, quiet=a.json, baseline_csv=bl,
                                      trust_column=a.trust_column)
                except SystemExit as e:
                    print(f'✗ {cpath}: {e}')
                    v, m = 'ERROR', {}
                rows.append((scen or cpath, v, m))
                nfail += v in ('FAIL', 'ERROR', 'DRIFT')
        print('\n' + '=' * 70)
        print(f'{"scenario":34s} {"verdict":13s} {"min_sp":>8s} {"pos_err":>9s}')
        for scen, v, m in rows:
            print(f'{scen:34s} {v:13s} {_n(m.get("min_spacing"))} {_n(m.get("pos_err"))}')
        good = sum(1 for _, v, _ in rows if v in ('PASS', 'NEUTRAL'))
        nlog = sum(1 for _, v, _ in rows if v.startswith('LOG'))
        print(f'\n通过 {good}  不通过 {nfail}  待日志定案 {nlog}   (共 {len(rows)})')
        return 1 if nfail else 0

    if not a.csv:
        ap.error('需要 <csv> 或 --batch')
    v, m, results = run_one(a.csv, a.scenario, a.yaml, a.steady_offset, a.log,
                            quiet=a.json, baseline_csv=a.baseline, trust_column=a.trust_column)
    if a.json:
        print(json.dumps({'verdict': v, 'metrics': {k: val for k, val in m.items()
                                                    if k != 'pos_err_per_drone'}},
                         ensure_ascii=False, indent=2))
    return 0 if v in ('PASS', 'NEUTRAL') else (2 if v.startswith('LOG') else 1)


if __name__ == '__main__':
    sys.exit(main())
