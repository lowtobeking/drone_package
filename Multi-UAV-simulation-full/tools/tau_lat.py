#!/usr/bin/env python3
"""从 .ulg 辨识**水平**速度环一阶滞后 tau：  v_dot = (u - v)/tau

两条硬约束（沿用 2026-08-20 那轮的方法，别再踩）：
  1) **必须用自由仿真误差**做判据。一步预报的 R^2 恒 ~0.99，是陷阱。
  2) 用 EKF 速度当"实测"，EKF 自身有滞后 ⇒ 辨出的 tau 是**上界**。

另做 tau / 纯延迟 Td 的交换检验：若 tau+Td 近似恒定，说明量级由数据决定，
而不是由"把滞后归给谁"这个模型选择决定。
"""
import sys
import numpy as np
from pyulog import ULog


def resample_zoh(t_src, v_src, t_dst):
    """ZOH 重采样：指令是零阶保持的，不能线性插值。"""
    idx = np.searchsorted(t_src, t_dst, side="right") - 1
    idx = np.clip(idx, 0, len(v_src) - 1)
    return v_src[idx]


def free_sim(u, dt, tau, v0, td=0.0):
    """自由仿真：只用 u 和初值，逐步积分，不喂真实 v。"""
    n = len(u)
    v = np.empty(n)
    v[0] = v0
    shift = int(round(td / dt))
    for k in range(1, n):
        uk = u[k - 1 - shift] if k - 1 - shift >= 0 else u[0]
        v[k] = v[k - 1] + dt * (uk - v[k - 1]) / tau
    return v


def r2(y, yhat):
    ss_res = np.sum((y - yhat) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def fit_axis(u, v, dt, td=0.0, grid=None):
    if grid is None:
        grid = np.arange(0.05, 1.501, 0.005)
    best = (None, -1e9, None)
    for tau in grid:
        sim = free_sim(u, dt, tau, v[0], td)
        score = r2(v, sim)
        if score > best[1]:
            best = (tau, score, sim)
    return best


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "log_58.ulg"
    u_log = ULog(path, ["trajectory_setpoint", "vehicle_local_position", "vehicle_status"])
    D = {d.name: d.data for d in u_log.data_list}

    st = D["vehicle_status"]
    ns = np.array(st["nav_state"])
    ts = np.array(st["timestamp"]) / 1e6
    off = ts[ns == 14]
    t0, t1 = off.min(), off.max()
    print("%s  OFFBOARD %.1f -> %.1f s (%.1f s)" % (path, t0, t1, t1 - t0))

    sp = D["trajectory_setpoint"]
    tsp = np.array(sp["timestamp"]) / 1e6
    lp = D["vehicle_local_position"]
    tlp = np.array(lp["timestamp"]) / 1e6

    # 统一 50 Hz 网格（下发率实测约 44-50 Hz）
    dt = 0.02
    tg = np.arange(t0 + 3.0, t1 - 2.0, dt)   # 掐掉进/出 OFFBOARD 的瞬态

    for axis, ku, kv in (("x(北)", "velocity[0]", "vx"), ("y(东)", "velocity[1]", "vy")):
        u_raw = np.array(sp[ku])
        good = np.isfinite(u_raw)
        u = resample_zoh(tsp[good], u_raw[good], tg)
        v = np.interp(tg, tlp, np.array(lp[kv]))

        # 只保留有激励的窗口：指令绝对值超过阈值的段
        exc = np.abs(u) > 0.05
        frac = exc.mean()
        tau, score, sim = fit_axis(u, v, dt)
        rms = float(np.sqrt(np.mean((v - sim) ** 2)))
        print("\n=== 轴 %s ===" % axis)
        print("  指令 rms %.3f  峰值 %.3f  有激励占比 %.0f%%  样本 %d"
              % (float(np.sqrt(np.mean(u ** 2))), float(np.abs(u).max()), 100 * frac, len(u)))
        print("  自由仿真最优:  tau = %.3f s   R2 = %.4f   残差 rms = %.4f m/s"
              % (tau, score, rms))

        # 对照：一步预报（陷阱演示）
        vp = v[:-1] + dt * (u[:-1] - v[:-1]) / tau
        print("  [对照] 一步预报 R2 = %.5f  <- 恒接近 1，不能用作判据" % r2(v[1:], vp))

        # tau/Td 交换检验
        print("  交换检验 (纯延迟 Td 固定，重辨 tau):")
        for td in (0.0, 0.05, 0.10, 0.15, 0.20):
            tt, ss, _ = fit_axis(u, v, dt, td)
            print("     Td=%.2f -> tau=%.3f  (tau+Td=%.3f)  R2=%.4f" % (td, tt, tt + td, ss))


if __name__ == "__main__":
    main()
