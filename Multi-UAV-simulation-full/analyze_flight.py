#!/usr/bin/env python3
"""
analyze_flight.py — 读 diag_monitor --log 生成的飞行 CSV，输出一次飞行的体检报告。

用法:
  python3 analyze_flight.py flight_trio3_20260603_120000.csv
  python3 analyze_flight.py log.csv --plot          # 另出曲线图(需 matplotlib)
  python3 analyze_flight.py log.csv --ready-err 0.5  # 自定义"进编队"阈值

报告内容:每机 pos_err / 高度误差(max/mean)、最小间距(+时刻)、安全违规、
         求解耗时、MPC fallback、编队成型时间。
"""
import argparse
import csv
import math
import statistics as st


def _f(x):
    """转 float，非数/inf/nan → None。"""
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description='飞行 CSV 体检报告')
    ap.add_argument('csv', help='diag_monitor --log 生成的 CSV')
    ap.add_argument('--ready-err', type=float, default=0.5,
                    help='判定"进编队"的 pos_err 阈值 (m)，用于算成型时间')
    ap.add_argument('--steady-offset', type=float, default=45.0,
                    help='飞行开始后多少秒起算稳态窗口 (s)，默认 45')
    ap.add_argument('--plot', action='store_true',
                    help='画 pos_err / 最小间距曲线(需 matplotlib)')
    args = ap.parse_args()

    with open(args.csv, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print('空日志')
        return

    cols = list(rows[0].keys())
    ndr = sum(1 for c in cols if c.endswith('_poserr'))
    t = [_f(r['t']) for r in rows]
    dur = (t[-1] - t[0]) if (t and t[0] is not None and t[-1] is not None) else 0.0

    print(f'== 飞行报告: {args.csv} ==')
    print(f'时长 {dur:.0f}s，{len(rows)} 采样点，{ndr} 机\n')

    # 稳态窗口掩码（t > t_start + steady_offset）
    t_start = t[0] if (t and t[0] is not None) else 0.0
    steady_t = t_start + args.steady_offset if t_start is not None else None

    # 每机 pos_err / 高度误差
    for i in range(ndr):
        pe_all  = [(_f(r[f'd{i}_poserr']), _f(r['t'])) for r in rows]
        pe      = [v for v, _ in pe_all if v is not None]
        pe_ss   = [v for v, tt in pe_all if v is not None and tt is not None
                   and steady_t is not None and tt > steady_t]
        ze      = [abs(v) for v in (_f(r[f'd{i}_zerr']) for r in rows) if v is not None]
        if pe and ze:
            ss_str = f' steady_mean(t>{args.steady_offset:.0f}s)={st.mean(pe_ss):.2f}m' if pe_ss else ''
            print(f'drone{i}: pos_err max={max(pe):.2f} mean={st.mean(pe):.2f}m{ss_str}   '
                  f'|z_err| max={max(ze):.2f} mean={st.mean(ze):.2f}m')

    # 最小间距 + 安全违规
    sp = [(v, tt) for v, tt in ((_f(r['min_spacing']), _f(r['t'])) for r in rows)
          if v is not None]
    if sp:
        vmin, tmin = min(sp, key=lambda x: x[0])
        print(f'\n最小间距 {vmin:.2f}m @ t={tmin:.0f}s')
    viol = [_f(r['safety_violations']) for r in rows if _f(r['safety_violations']) is not None]
    if viol:
        print(f'安全违规累计 {int(viol[-1])} 次')

    # 求解耗时 / fallback
    ms = [v for v in (_f(r['max_solve_ms']) for r in rows) if v is not None]
    if ms:
        print(f'求解耗时 max={max(ms):.2f} mean={st.mean(ms):.2f} ms')
    fb = [_f(r['total_fallbacks']) for r in rows if _f(r['total_fallbacks']) is not None]
    if fb:
        print(f'MPC fallback 累计 {int(fb[-1])} 次')

    # 编队成型时间：第一次"全员 pos_err < 阈值"的时刻
    settle = None
    for r in rows:
        errs = [_f(r[f'd{i}_poserr']) for i in range(ndr)]
        if errs and all(e is not None and e < args.ready_err for e in errs):
            settle = _f(r['t'])
            break
    print(f'\n编队成型(全员 pos_err<{args.ready_err}m) @ '
          + (f't={settle:.0f}s' if settle is not None else '未达成'))

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print('\n(未装 matplotlib，跳过画图)')
            return
        fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        for i in range(ndr):
            a1.plot(t, [_f(r[f'd{i}_poserr']) for r in rows], label=f'd{i}')
        a1.axhline(args.ready_err, ls='--', c='gray')
        a1.set_ylabel('pos_err [m]')
        a1.legend()
        a2.plot(t, [_f(r['min_spacing']) for r in rows], 'r')
        a2.set_ylabel('min spacing [m]')
        a2.set_xlabel('t [s]')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
