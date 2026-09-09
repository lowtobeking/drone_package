#!/usr/bin/env python3
"""动捕遮挡的共模/差模对照分析。

    python3 tools/analyze_mocap_dropout.py ~/flights/mocap_diffmode/flight_*.csv

**为什么要分共模/差模**：`report/阶段性报告.md` 与论文里已有的通信中断实验
（S42/S43）都是**共模**的 —— 全员同时受扰、三机同向偏移、相对队形不变，
于是"邻居在队形位上"这个推断依然≈真值，实测影响仅 +0.01m。该结论**不能
推广到差模**（单机失效/单机受扰），而差模那半边此前完全空白。

动捕遮挡天然是**逐刚体发生**的 ⇒ 差模才是现实工况，共模（整套动捕同时挂）
反而是人造对照。本脚本比较两者对编队的破坏程度。

窗口划分：注入在录制开始后约 30s（见 diffmode2.sh 时序），故
  before = 前 25s， during/after = 其余
"""
import csv
import sys
from pathlib import Path


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    t0 = float(rows[0]['t'])
    for r in rows:
        r['_t'] = float(r['t']) - t0
    return rows


def stat(rows, key, lo, hi):
    vals = []
    for r in rows:
        if lo <= r['_t'] < hi:
            try:
                v = float(r[key])
            except (KeyError, ValueError, TypeError):
                continue
            if v == v and abs(v) < 1e6:      # 排除 nan/inf
                vals.append(v)
    if not vals:
        return None
    return dict(n=len(vals), mean=sum(vals) / len(vals),
                mx=max(vals), mn=min(vals))


def fmt(s, key='mean'):
    return f'{s[key]:8.3f}' if s else '     n/a'


def main():
    paths = [Path(p) for p in sys.argv[1:]]
    if not paths:
        print('用法: analyze_mocap_dropout.py <csv...>'); return 1

    print(f'{"组":<12}{"指标":<20}{"注入前":>10}{"注入后":>10}{"劣化":>10}')
    print('-' * 64)
    summary = {}
    for p in sorted(paths):
        rows = load(p)
        if not rows:
            print(f'{p.name:<12}(空文件)'); continue
        tag = p.stem.replace('flight_', '')
        summary[tag] = {}
        for key, label in (('formation_max_err', '编队最大误差 m'),
                           ('min_spacing', '最小机间距 m')):
            if key not in rows[0]:
                continue
            b = stat(rows, key, 0, 25)
            a = stat(rows, key, 30, 1e9)
            if key == 'min_spacing':
                # 间距是"越小越危险"，劣化取最小值的下降
                delta = (b['mn'] - a['mn']) if (b and a) else None
                print(f'{tag:<12}{label:<20}{fmt(b, "mn")}{fmt(a, "mn")}'
                      f'{delta:>10.3f}' if delta is not None
                      else f'{tag:<12}{label:<20}      n/a')
                summary[tag][key] = (b['mn'] if b else None, a['mn'] if a else None)
            else:
                delta = (a['mx'] - b['mx']) if (b and a) else None
                print(f'{tag:<12}{label:<20}{fmt(b, "mx")}{fmt(a, "mx")}'
                      f'{delta:>10.3f}' if delta is not None
                      else f'{tag:<12}{label:<20}      n/a')
                summary[tag][key] = (b['mx'] if b else None, a['mx'] if a else None)
        print()

    print('=' * 64)
    print('判读：共模(B)与差模(C)的劣化幅度若量级相当，说明该扰动下"共模无害"')
    print('      这一结论可以推广；若差模明显更大，则此前基于共模的结论不能外推。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
