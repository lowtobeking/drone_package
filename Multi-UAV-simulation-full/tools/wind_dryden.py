#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dryden 湍流风注入器 —— 复用现有 ApplyLinkWrench persistent 通道（论文 M2 实验用）。

原理
----
MIL-HDBK-1797 低空 Dryden 谱：白噪声过一阶(u,v)/二阶(w 不用)成形滤波器得到阵风速度，
叠加平均风后按 --n-per-ms（牛顿 每 米/秒，沿用现有 WIND_N 的标定比）折成力，
以 --rate 周期性重发 `gz topic .../wrench/persistent`。

🔑 为什么低速率(默认 2 Hz)就够：Dryden 按空速 V 参数化，悬停场景取 V≈平均风速；
   V=5 m/s、低空 L_u≈O(100 m) ⇒ 阵风带宽 ~0.01–0.05 Hz，远低于 2 Hz。
🔑 persistent wrench 对同一 link 是**覆盖**语义（幂等，不叠加）——run_one_trial.sh
   已实测；因此周期重发即时变力，丢一拍=零阶保持，天然安全。
🔑 纯悬停 V→0 时 Dryden 退化 ⇒ 本脚本强制要求 --mean-wind > 0.5 m/s
   （实验设计=「同均值风、有/无湍流」对照，正好与现有恒定风实验可比）。

用法
----
  # 离线自检（不需要仿真机；PSD 对解析谱，PASS/FAIL）
  python3 tools/wind_dryden.py --selftest

  # 生成 CSV 预览（力序列，供画图/检查）
  python3 tools/wind_dryden.py --csv /tmp/wind.csv --duration 300 --seed 1

  # 实际注入（仿真机上；Ctrl-C 或 duration 到点后自动清零力）
  python3 tools/wind_dryden.py --publish --drones 5 --duration 300 \
      --mean-wind 5.0 --w20 7.7 --n-per-ms 0.6 --seed 1

参数化（MIL-HDBK-1797 低空，h<1000ft）
--------------------------------------
  sigma_w = 0.1*W20 ;  sigma_u = sigma_v = sigma_w / (0.177+0.000823*h_ft)^0.4
  L_w = h ;            L_u = L_v = h / (0.177+0.000823*h_ft)^1.2      （英尺制系数）
  H_u(s) = sigma_u*sqrt(2*L_u/(pi*V)) / (1 + (L_u/V)*s)              （一阶）
  水平两轴 u(顺风向)、v(侧向) 都用一阶形式（v 的二阶修正在此带宽差异可忽略，
  且实验只关心"时变 vs 恒定"，谱形二阶细节不是判据——如实写进论文措辞）。

⚠️ 与真气动的差距（论文措辞用）：力级注入，不含飞机自身运动引起的相对风（无气动
   阻尼项）——与现有恒定力方法同层级，对照实验内部自洽。
"""
import argparse
import math
import signal
import subprocess
import sys
import time

import numpy as np

FT_PER_M = 3.28084


def dryden_params(w20, h_m, v):
    """返回 (sigma_u, L_u_m)（u、v 同参）。w20=6m(20ft)高度平均风 m/s。"""
    h_ft = max(h_m, 1.0) * FT_PER_M
    sigma_w = 0.1 * w20
    den = (0.177 + 0.000823 * h_ft)
    sigma_u = sigma_w / den ** 0.4
    L_u_ft = h_ft / den ** 1.2
    return sigma_u, L_u_ft / FT_PER_M


def gen_series(n, dt, w20, h_m, v, seed):
    """生成 n 点、步长 dt 的 (gu, gv) 阵风速度序列（m/s，零均值）。

    一阶成形滤波离散化（精确 ZOH）：
      x[k+1] = a*x[k] + b*w[k],  a=exp(-dt/T), T=L/V
    使输出方差 = sigma^2：b = sigma*sqrt(1-a^2)（w 为单位白噪声）。
    """
    sigma, L = dryden_params(w20, h_m, v)
    T = L / max(v, 0.5)
    a = math.exp(-dt / T)
    b = sigma * math.sqrt(1.0 - a * a)
    rng = np.random.default_rng(seed)
    out = np.empty((n, 2))
    x = np.zeros(2)
    w = rng.standard_normal((n, 2))
    for k in range(n):
        x = a * x + b * w[k]
        out[k] = x
    return out, sigma, T


def analytic_psd(f, sigma, T):
    """一阶 Gauss-Markov **单边** PSD：S1(f)=4*sigma^2*T/(1+(2*pi*f*T)^2)。
    校验：∫0∞ S1 df = sigma^2（积分回方差）。首版少了因子 2（写成双边式却当单边用），
    selftest 以 104% 误差抓出——这正是自检存在的意义。"""
    return 4.0 * sigma * sigma * T / (1.0 + (2 * math.pi * f * T) ** 2)


def selftest():
    """PSD 对解析谱：中频带（0.3/T ~ 3/T rad/s 对应频段）平均相对误差 <20% 判 PASS。"""
    dt, dur = 0.5, 40000.0
    w20, h, v, seed = 7.7, 3.0, 5.0, 12345
    n = int(dur / dt)
    g, sigma, T = gen_series(n, dt, w20, h, v, seed)
    print("参数: sigma=%.3f m/s  T=L/V=%.1f s  (w20=%.1f h=%.1fm V=%.1f)"
          % (sigma, T, w20, h, v))
    # 方差核对
    var_ok = abs(g[:, 0].std() / sigma - 1) < 0.05
    print("样本 σ_u=%.3f (解析 %.3f)  %s"
          % (g[:, 0].std(), sigma, "OK" if var_ok else "FAIL"))
    # 分段平均周期图
    seg = 4096
    nseg = n // seg
    ps = np.zeros(seg // 2)
    for i in range(nseg):
        d = g[i * seg:(i + 1) * seg, 0]
        d = d - d.mean()
        Y = np.fft.rfft(d * np.hanning(seg))
        w_ = np.hanning(seg)
        # 周期图归一（双边→取正频，乘 2；窗能量补偿）
        p = (np.abs(Y) ** 2) * dt / (w_ ** 2).sum()
        ps += 2 * p[1:seg // 2 + 1][:seg // 2]
    ps /= nseg
    f = np.fft.rfftfreq(seg, dt)[1:seg // 2 + 1][:seg // 2]
    Sa = np.array([analytic_psd(x, sigma, T) for x in f])
    lo, hi = 0.3 / (2 * math.pi * T), 3.0 / (2 * math.pi * T)
    m = (f >= lo) & (f <= hi)
    rel = np.abs(ps[m] - Sa[m]) / Sa[m]
    err = float(rel.mean())
    print("PSD 中频带(%.4f–%.4f Hz, %d 点) 平均相对误差 %.1f%%"
          % (lo, hi, int(m.sum()), 100 * err))
    ok = var_ok and err < 0.20
    print("SELFTEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def publish_loop(args):
    """周期覆盖各机 persistent wrench；退出（含 Ctrl-C）时清零力。"""
    dt = 1.0 / args.rate
    n = int(args.duration / dt) + 1
    g, sigma, T = gen_series(n, dt, args.w20, args.alt, args.mean_wind, args.seed)
    print("[wind_dryden] sigma=%.2f m/s T=%.1fs | 均值风 %.1f m/s(+x) | %d 机 @%.1fHz %ds"
          % (sigma, T, args.mean_wind, args.drones, args.rate, args.duration),
          flush=True)

    def wrench(d, fx, fy):
        return ["gz", "topic", "-t", "/world/%s/wrench/persistent" % args.world,
                "-m", "gz.msgs.EntityWrench",
                "-p", 'entity: {name: "x500_%d::base_link", type: LINK}, '
                      'wrench: {force: {x: %.3f, y: %.3f}}' % (d, fx, fy)]

    def clear_all():
        for d in range(args.drones):
            for _ in range(3):
                subprocess.run(wrench(d, 0.0, 0.0), capture_output=True)
                time.sleep(0.1)
        print("[wind_dryden] 力已清零", flush=True)

    stop = {"f": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("f", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("f", True))

    # 预热重发（沿用 run_one_trial 的发现竞态规避）
    for d in range(args.drones):
        for _ in range(4):
            subprocess.run(wrench(d, args.n_per_ms * args.mean_wind, 0.0),
                           capture_output=True)
            time.sleep(0.15)
    t0 = time.time()
    k = 0
    try:
        while not stop["f"] and k < n:
            vx = args.mean_wind + g[k, 0]
            vy = g[k, 1]
            fx, fy = args.n_per_ms * vx, args.n_per_ms * vy
            for d in range(args.drones):
                subprocess.run(wrench(d, fx, fy), capture_output=True)
            k += 1
            t_next = t0 + k * dt
            time.sleep(max(0.0, t_next - time.time()))
            if k % max(1, int(10 * args.rate)) == 0:
                print("[wind_dryden] t=%5.0fs  v=(%.2f,%.2f) m/s  F=(%.2f,%.2f) N"
                      % (k * dt, vx, vy, fx, fy), flush=True)
    finally:
        clear_all()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--csv", help="仅生成力序列 CSV（t,vx,vy,fx,fy）")
    ap.add_argument("--publish", action="store_true", help="实际注入（仿真机上）")
    ap.add_argument("--drones", type=int, default=5)
    ap.add_argument("--world", default="default")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--rate", type=float, default=2.0, help="重发频率 Hz（带宽余量>40x）")
    ap.add_argument("--mean-wind", type=float, default=5.0,
                    help="平均风 m/s，+x 方向（须 >0.5，见头注 Dryden 悬停退化）")
    ap.add_argument("--w20", type=float, default=7.7,
                    help="20ft 高度风速 m/s（MIL 强度参数；7.7=15kt 中等湍流）")
    ap.add_argument("--alt", type=float, default=3.0, help="飞行高度 m（谱参数用）")
    ap.add_argument("--n-per-ms", type=float, default=0.6,
                    help="牛顿 每 (米/秒)：沿用恒定风标定比 WIND_N/等效风速")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest())
    if a.mean_wind <= 0.5:
        ap.error("--mean-wind 须 > 0.5 m/s（Dryden 悬停退化，见头注）")
    if a.csv:
        dt = 1.0 / a.rate
        n = int(a.duration / dt) + 1
        g, sigma, T = gen_series(n, dt, a.w20, a.alt, a.mean_wind, a.seed)
        with open(a.csv, "w") as f:
            f.write("t,vx,vy,fx,fy\n")
            for k in range(n):
                vx, vy = a.mean_wind + g[k, 0], g[k, 1]
                f.write("%.2f,%.4f,%.4f,%.4f,%.4f\n"
                        % (k * dt, vx, vy, a.n_per_ms * vx, a.n_per_ms * vy))
        print("CSV -> %s (sigma=%.2f T=%.1fs n=%d)" % (a.csv, sigma, T, n))
        return
    if a.publish:
        sys.exit(publish_loop(a))
    ap.print_help()


if __name__ == "__main__":
    main()
