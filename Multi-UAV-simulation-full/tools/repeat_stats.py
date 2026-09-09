#!/usr/bin/env python3
"""repeat_stats.py — 聚合同一工况的 N 次重复跑，算稳态 pos_err 均值±标准差；
   --wind-window 额外算窗口内 pos_err 峰峰值(max-min)±标准差，用于风扰组 headline 数字。

配套 report/RUN_PLAN_tau_sensitivity_n5.md 的 Part A（n≥5 重复实验）数据处理。
复用 verdict.py 的 measure()（1Hz 校验 + 稳态窗解析），不重复实现解析逻辑。

用法:
    python3 tools/repeat_stats.py --group A1 flight_A1_run*.csv --scenario S3_trio3_circle_rh
    python3 tools/repeat_stats.py --group B1 flight_B1_run*.csv --wind-window 110 258
"""
import argparse
import csv
import io
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from verdict import measure, YAML_DEFAULT, _f  # noqa: E402
import data_validity  # noqa: E402  （逐机有效性判据，见该文件头注）


def window_peak_to_peak(csv_path, t_lo, t_hi, per_agent_out=None, excl_out=None):
    """窗口内各机 pos_err 峰峰值（max-min），取全队最坏机——对应报告 §5.F"峰峰"口径。

    🔑 2026-09-01 起先过 `tools/data_validity.py` 的逐机判据，剔除 health 冻结 /
       状态溢出 / 悬停兜底主导的机——那些机的 poserr 是**陈旧常数**，峰峰值恒为 0，
       会静默把统计拉低（M1 零模型那次把中位数拉低了 4%）。
    ⚠️ `per_agent_out` 传入 list 时会追加**逐机**峰峰值（§6.2c 用的是 per-agent
       中位数，与本函数返回的 worst-agent **不是同一个口径**，勿混用）。
    """
    with io.open(csv_path, newline='', encoding='utf-8', errors='replace') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    ndr = sum(1 for c in rows[0] if c.endswith('_poserr'))
    idx = [i for i, r in enumerate(rows)
           if _f(r.get('t')) is not None and t_lo <= _f(r['t']) <= t_hi]
    bad = data_validity.scan(rows, ndr, idx=idx)
    if excl_out is not None and bad:
        excl_out.append((os.path.basename(csv_path), bad))
    worst_pp = None
    for d in range(ndr):
        if d in bad:
            continue
        vals = [_f(rows[i].get(f'd{d}_poserr')) for i in idx]
        vals = [v for v in vals if v is not None]
        if len(vals) >= 2:
            pp = max(vals) - min(vals)
            if per_agent_out is not None:
                per_agent_out.append(pp)
            worst_pp = pp if worst_pp is None else max(worst_pp, pp)
    return worst_pp


def main():
    ap = argparse.ArgumentParser(description='聚合 N 次重复跑的稳态 pos_err（可选风窗口峰峰值）')
    ap.add_argument('csvs', nargs='+', help='同一工况的重复跑 CSV 列表')
    ap.add_argument('--group', required=True, help='组名，仅用于打印标题')
    ap.add_argument('--yaml', default=YAML_DEFAULT)
    ap.add_argument('--steady-offset', type=float, default=45.0)
    ap.add_argument('--wind-window', nargs=2, type=float, metavar=('T_LO', 'T_HI'),
                    help='额外计算该窗口内 pos_err 峰峰值±标准差（风扰组用，如 110 258）')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    pos_errs, pps, used = [], [], []
    per_agent, exclusions = [], []
    for c in a.csvs:
        try:
            m = measure(c, a.steady_offset)
        except SystemExit as e:
            print(f'✗ 跳过 {c}: {e}', file=sys.stderr)
            continue
        if m.get('pos_err') is None:
            why = m.get('pos_err_note') or 'pos_err 不可用'
            print(f'✗ 跳过 {c}: {why}', file=sys.stderr)
            for line in m.get('invalid_agents_detail', []):
                print('   ' + line.strip(), file=sys.stderr)
            continue
        pos_errs.append(m['pos_err'])
        used.append(c)
        if a.wind_window:
            pp = window_peak_to_peak(c, a.wind_window[0], a.wind_window[1],
                                     per_agent_out=per_agent, excl_out=exclusions)
            if pp is not None:
                pps.append(pp)

    n = len(pos_errs)
    out = {'group': a.group, 'n': n, 'csvs': used}
    if n == 0:
        print(f'\n== {a.group}  (n=0) ==\n  无有效数据', file=sys.stderr)
        return 1

    mean_pe = st.mean(pos_errs)
    sd_pe = st.stdev(pos_errs) if n > 1 else 0.0
    out['pos_err_mean'], out['pos_err_sd'], out['pos_err_values'] = mean_pe, sd_pe, pos_errs

    if a.wind_window and pps:
        mean_pp = st.mean(pps)
        sd_pp = st.stdev(pps) if len(pps) > 1 else 0.0
        out['peak_to_peak_mean'], out['peak_to_peak_sd'], out['peak_to_peak_values'] = mean_pp, sd_pp, pps

    if a.json:
        import json
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0

    print(f'\n== {a.group}  (n={n}, 若 <5 说明有跑被跳过，检查上面的 ✗ 提示) ==')
    print(f'  pos_err(稳态窗均值)   mean={mean_pe:.3f}  sd={sd_pe:.3f}  '
          f'values={[f"{v:.3f}" for v in pos_errs]}')
    if a.wind_window:
        if pps:
            print(f'  峰峰值[t∈{a.wind_window}]      mean={out["peak_to_peak_mean"]:.3f}  '
                  f'sd={out["peak_to_peak_sd"]:.3f}  values={[f"{v:.3f}" for v in pps]}')
            # per-agent 中位数：§6.2c 引用的就是这个量，与上面的 worst-agent 不同口径
            if per_agent:
                pa = sorted(per_agent)
                print(f'  峰峰值 per-agent          n={len(pa)} 机次  '
                      f'中位={st.median(pa):.3f}  范围=[{pa[0]:.3f}, {pa[-1]:.3f}]'
                      '   ← §6.2c 用的是这一行，勿与上一行的 worst-agent 混用')
                out['per_agent_p2p'] = per_agent
                out['per_agent_p2p_median'] = st.median(pa)
        else:
            print('  峰峰值：无有效窗口数据（检查 --wind-window 区间是否落在录制时长内）')
    # 剔除清单永远显式打印——静默剔除比不剔除更危险
    if exclusions:
        n_bad = sum(len(b) for _, b in exclusions)
        print(f'  ⚠️ 已剔除 {n_bad} 个「架次×机」（非测量数据，判据见 tools/data_validity.py）：')
        for fn, bad in exclusions:
            for d, (code, why) in sorted(bad.items()):
                print(f'       {fn}  d{d} [{code}] {why}')
        out['excluded'] = [{'csv': fn, 'drone': d, 'code': c2, 'why': w}
                           for fn, bad in exclusions for d, (c2, w) in sorted(bad.items())]
    return 0


if __name__ == '__main__':
    sys.exit(main())
