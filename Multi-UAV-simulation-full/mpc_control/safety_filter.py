#!/usr/bin/env python3
"""companion 侧独立安全滤波层 —— 在 MPC 速度指令下发前统一过一道硬保护。

设计目标（按"挡炸机性价比"）：
  缺口1 飞散围栏    : 限 |pos-ref|（偏离参考点距离）+ 绝对高度，临界刹停、越界升级。
                     用 track-divergence 而非距出生点——line/circle 会远离出生点但
                     不会远离"它该在的位置"；只刹扩大偏差的速度、不挡正常追赶。
                     绝对区域围栏交给 FC GF_MAX_HOR_DIST。
  缺口2 硬碰撞地板  : 用实测邻居位置独立硬判，太近 → 横向刹停（绕过 MPC 软代价）
  缺口3 估计健康门  : self 估计中途变脏 → 失效保护（不拿坏状态飞）
  缺口4 失效状态机  : NORMAL→DEGRADED→HOLD→RELINQUISH（交还 PX4 失效保护）
  缺口5 leader 限跳 : 速度硬帽 + 转换率(jerk)限制，吸收参考巨跳

刻意做成**纯函数式、无 ROS 依赖**：
  - Windows `py -m py_compile` / `py safety_filter.py`(自测) 可直接跑；
  - mpc_node 每帧 step() 一次；
  - 同一逻辑可逐行移植到 Simulink 端，注入故障证明安全网会触发。

不改 MPC 的 OCP 结构 → 不用清 acados 缓存。坐标系：世界 NED（z 向下，
target_alt<0；离地高度 = birth_z - pos_z）。
"""
import math
import numpy as np

# 失效保护状态
NORMAL, DEGRADED, HOLD, RELINQUISH = 'NORMAL', 'DEGRADED', 'HOLD', 'RELINQUISH'


class SafetyFilter:
    def __init__(self,
                 max_track_dist=5.0,    # m，偏离参考点(leader+offset)最大距离=飞散阈值
                                        # （正常 track_err 0.5~1.2m，5m 已是明显异常）
                 max_alt=8.0,           # m，最大离地绝对高度
                 min_alt=0.3,           # m，最小离地绝对高度（防贴地）
                 fence_brake_frac=0.8,  # 超过 max_track*frac 开始刹扩大偏差的速度
                 d_emergency=1.2,       # m，硬碰撞地板（应 < d_safe；真机 d_safe=2.5 → 设 ~1.8）
                 d_warn=2.0,            # m，预警距离，开始抑制接近分量
                 max_speed=1.5,         # m/s，水平速度硬帽
                 max_climb=1.0,         # m/s，垂直速度硬帽
                 max_accel=2.0,         # m/s²，转换率(jerk)限制基准
                 accel_slack=1.5,       # 转换率限制裕度（× max_accel*dt）
                 degrade_speed_scale=0.5,   # DEGRADED 时速度缩放
                 escalate_frames=15,    # 连续违规多少帧升一级（50Hz→0.3s）
                 clear_frames=25,       # 连续正常多少帧降一级（0.5s）
                 relinquish_frames=50,  # HOLD 持续多少帧交还 PX4（1s）
                 grace_frames=150,      # arm 后冷启动保护窗口（50Hz→3s）：期间只激活碰撞+NaN，
                                        # 跳过高度/围栏检查（起飞穿过 min_alt 属正常，勿误触）
                 drone_id=0):
        self.max_track_dist = float(max_track_dist)
        self.max_alt = float(max_alt)
        self.min_alt = float(min_alt)
        self.fence_brake_frac = float(fence_brake_frac)
        self.d_emergency = float(d_emergency)
        self.d_warn = float(d_warn)
        self.max_speed = float(max_speed)
        self.max_climb = float(max_climb)
        self.max_accel = float(max_accel)
        self.accel_slack = float(accel_slack)
        self.degrade_speed_scale = float(degrade_speed_scale)
        self.escalate_frames = int(escalate_frames)
        self.clear_frames = int(clear_frames)
        self.relinquish_frames = int(relinquish_frames)
        self.grace_frames = int(grace_frames)
        self.drone_id = int(drone_id)

        self.state = NORMAL
        self._bad_streak = 0       # 连续 severity>=1 帧
        self._ok_streak = 0        # 连续 severity==0 帧
        self._hold_streak = 0      # 连续处于 HOLD/更高 帧
        self._prev_vel = np.zeros(3)
        self._frame = 0            # 总帧计数，用于 grace_frames 计时

    def reset(self):
        self.state = NORMAL
        self._bad_streak = self._ok_streak = self._hold_streak = 0
        self._prev_vel = np.zeros(3)
        self._frame = 0

    def step(self, pos, vel, vel_sp, ref, dt,
             neighbours=None, est_ok=True):
        """过滤一帧。
        pos/vel : self 世界 NED 位置/速度 (3,)
        vel_sp  : MPC 给的期望速度 (3,)（世界 NED）
        ref     : self 当前世界参考点 (3,) = leader_pos + my_offset（该在的位置）
        neighbours : [(nbr_pos(3,), fresh(bool)), ...] 实测邻居世界位置
        est_ok  : self 估计健康（xy_valid & z_valid & 新鲜）
        返回 dict: vel_sp(3,), state, reasons(list), publish(bool), severity
        """
        pos = np.asarray(pos, float); vel = np.asarray(vel, float)
        v = np.asarray(vel_sp, float).copy(); ref = np.asarray(ref, float)
        reasons = []
        severity = 0  # 0 ok, 1 degrade, 2 hold
        in_grace = self._frame < self.grace_frames
        self._frame += 1

        # 缺口5a：非有限 → 直接 HOLD（零速）
        if not np.all(np.isfinite(v)) or not np.all(np.isfinite(pos)):
            v = np.zeros(3); severity = max(severity, 2); reasons.append('nonfinite')

        # 缺口3：估计不健康 → HOLD（不拿坏状态飞），持续会升级到 RELINQUISH
        if not est_ok:
            v = np.zeros(3); severity = max(severity, 2); reasons.append('est_unhealthy')

        # 缺口1：飞散围栏 + 高度围栏（冷启动窗口内跳过，避免起飞穿越 min_alt 误触）
        if not in_grace and np.all(np.isfinite(pos)) and np.all(np.isfinite(ref)):
            dxy = pos[:2] - ref[:2]                    # 我相对"该在位置"的偏移
            r = float(np.linalg.norm(dxy))             # = 水平 track error
            if r > 1e-6:
                outward = dxy / r                      # 偏离参考点的方向
                v_out = float(np.dot(v[:2], outward))  # 扩大偏差的速度分量（>0 越飞越偏）
                if r > self.max_track_dist:            # 超出围栏
                    if v_out > 0:                      # 仍在外扩：刹止并 RELINQUISH
                        v[:2] -= v_out * outward
                        severity = max(severity, 2); reasons.append(f'flyaway_breach(track={r:.1f})')
                    else:                              # 已在收敛回参考点：告警但不 RELINQUISH
                        severity = max(severity, 1); reasons.append(f'flyaway_converging(track={r:.1f})')
                elif r > self.max_track_dist * self.fence_brake_frac:  # 临界
                    if v_out > 0:
                        v[:2] -= v_out * outward
                    severity = max(severity, 1); reasons.append(f'flyaway_near(track={r:.1f})')
            alt = float(-pos[2])                       # 离地绝对高度（NED: ground z=0）
            if alt > self.max_alt:
                if v[2] < 0:                           # z<0 = 上升 → 禁止继续升
                    v[2] = 0.0
                severity = max(severity, 2); reasons.append(f'fence_alt_high({alt:.1f})')
            elif alt < self.min_alt:
                if v[2] > 0:                           # z>0 = 下降 → 禁止继续降
                    v[2] = 0.0
                severity = max(severity, 1); reasons.append(f'fence_alt_low({alt:.1f})')

        # 缺口2：硬碰撞地板（实测邻居位置，独立于 MPC 软代价）
        if neighbours:
            for nbr_pos, fresh in neighbours:
                if not fresh:
                    continue
                npos = np.asarray(nbr_pos, float)
                if not np.all(np.isfinite(npos)):
                    continue
                dvec = pos[:2] - npos[:2]
                d = float(np.linalg.norm(dvec))
                if d < 1e-6:
                    v[:2] = 0.0; severity = max(severity, 2); reasons.append('collide_coincident'); continue
                toward = -dvec / d                     # 指向邻居方向
                v_close = float(np.dot(v[:2], toward))  # 接近速度（>0 在靠近）
                if d < self.d_emergency:               # 紧急：横向全停
                    v[:2] = 0.0
                    severity = max(severity, 2); reasons.append(f'collide_emerg(d={d:.2f})')
                elif d < self.d_warn:                   # 预警：抑制接近 + 排斥推开
                    if v_close > 0:                     # 抑制接近分量
                        v[:2] -= v_close * toward
                    # 排斥分量：越近越强，防横向交叉
                    away = dvec / d                     # 远离邻居方向
                    repel_mag = self.max_speed * (1.0/d - 1.0/self.d_warn) / (1.0/self.d_emergency - 1.0/self.d_warn)
                    repel_mag = max(0.0, min(repel_mag, self.max_speed))
                    v[:2] += repel_mag * away
                    severity = max(severity, 1); reasons.append(f'collide_warn(d={d:.2f})')

        # 缺口5b：速度硬帽
        vxy = float(np.linalg.norm(v[:2]))
        if vxy > self.max_speed:
            v[:2] *= self.max_speed / vxy
        v[2] = float(np.clip(v[2], -self.max_climb, self.max_climb))

        # 缺口5c：转换率(jerk)限制——吸收 leader 巨跳/setpoint 突变
        dv = v - self._prev_vel
        dv_max = self.max_accel * max(dt, 1e-3) * self.accel_slack
        ndv = float(np.linalg.norm(dv))
        if ndv > dv_max:
            v = self._prev_vel + dv * (dv_max / ndv)

        # ── 失效保护状态机（迟滞升降级）──────────────────────────────────
        if severity >= 1:
            self._bad_streak += 1; self._ok_streak = 0
        else:
            self._ok_streak += 1; self._bad_streak = 0

        if severity >= 2 and self._bad_streak >= self.escalate_frames:
            self.state = HOLD
        elif severity >= 1 and self._bad_streak >= self.escalate_frames:
            self.state = DEGRADED if self.state == NORMAL else self.state
        elif self._ok_streak >= self.clear_frames and self.state != RELINQUISH:
            self.state = NORMAL  # 清零回正常；RELINQUISH sticky，须外部 reset() 才可恢复

        if self.state in (HOLD, RELINQUISH):
            if not in_grace:
                self._hold_streak += 1
        else:
            self._hold_streak = 0

        # HOLD 持续不消 → 交还 PX4（停发 offboard，触发 FC 失效保护）
        # 冷启动窗口（grace_frames）内不升级到 RELINQUISH：此时段只降级到 HOLD(零速安全)，
        # 不触发不可逆的交还闩锁。2026-07-28 实测：解锁瞬间系统级调度延迟（Gazebo+2xPX4+
        # 2xmpc_node 同时起步争抢资源，观测到 ~1.6s 一次性卡顿）会让 self 估计短暂 stale
        # → est_unhealthy → hold_streak 在 grace 窗口内被推过 relinquish_frames，
        # 把本可正常起飞的架次误伤成永久交还（须重启 ROS2 进程才能恢复）。
        # grace 结束后如果坏状态仍未消退，hold_streak 从 0 重新计数，正常按原逻辑升级，
        # 不影响对真实持续故障的失效保护。
        if not in_grace and self._hold_streak >= self.relinquish_frames:
            self.state = RELINQUISH

        # 状态对指令的最终作用
        if self.state == DEGRADED:
            v[:2] *= self.degrade_speed_scale
            v[2] = float(np.clip(v[2], -self.max_climb * self.degrade_speed_scale,
                                 self.max_climb * self.degrade_speed_scale))
        elif self.state == HOLD:
            v = np.zeros(3)
        elif self.state == RELINQUISH:
            v = np.zeros(3)

        if not np.all(np.isfinite(v)):
            v = np.zeros(3)
        self._prev_vel = v.copy()

        return {
            'vel_sp': v,
            'state': self.state,
            'reasons': reasons,
            'severity': severity,
            'publish': self.state != RELINQUISH,  # False = 停发 setpoint，交还 PX4
        }


# ====================================================================
# 自测：构造对抗输入，断言安全网触发（Windows: py safety_filter.py）
# ====================================================================
def _selftest():
    dt = 0.02
    ref = np.array([0., 0., 0.])   # 参考点（该在的位置）；pos 偏离它 = track error
    ok = 0; total = 0

    def check(name, cond):
        nonlocal ok, total
        total += 1
        print(f'  [{"PASS" if cond else "FAIL"}] {name}')
        if cond:
            ok += 1

    # 1) 正常：稳态指令应透传（jerk 限制使冷启动需几帧爬到目标，故跑多帧看稳态）
    sf = SafetyFilter()
    for _ in range(20):
        r = sf.step(pos=[1, 0, -5], vel=[0.5, 0, 0], vel_sp=[0.5, 0, 0], ref=ref, dt=dt)
    check('正常态稳态指令透传 (state=NORMAL)', r['state'] == NORMAL and abs(r['vel_sp'][0] - 0.5) < 0.05)

    # 2) 围栏：在边界外且外向飞 → 外向分量被刹掉
    sf = SafetyFilter(max_track_dist=8.0, grace_frames=0)
    r = sf.step(pos=[9, 0, -5], vel=[1, 0, 0], vel_sp=[1.5, 0, 0], ref=ref, dt=dt)
    check('飞散外向速度被刹停 (vx<=0)', r['vel_sp'][0] <= 1e-6 and 'flyaway_breach' in str(r['reasons']))

    # 3) 围栏升级：持续越界 → 进 HOLD
    sf = SafetyFilter(max_track_dist=8.0, escalate_frames=15, grace_frames=0)
    for _ in range(20):
        r = sf.step(pos=[9, 0, -5], vel=[1, 0, 0], vel_sp=[1.5, 0, 0], ref=ref, dt=dt)
    check('持续越界升级到 HOLD', r['state'] in (HOLD, RELINQUISH))

    # 4) 碰撞地板：邻居 1.0m（< d_emergency=1.2）→ 横向全停
    sf = SafetyFilter(d_emergency=1.2, d_warn=2.0)
    r = sf.step(pos=[0, 0, -5], vel=[0, 0, 0], vel_sp=[1.0, 0, 0], ref=ref, dt=dt,
                neighbours=[(np.array([1.0, 0, -5]), True)])
    check('硬碰撞地板横向全停', float(np.linalg.norm(r['vel_sp'][:2])) < 1e-6 and 'collide_emerg' in str(r['reasons']))

    # 5) 碰撞预警：邻居 1.6m，朝它飞 → 接近分量被抑制，远离分量保留
    sf = SafetyFilter(d_emergency=1.2, d_warn=2.0)
    r = sf.step(pos=[0, 0, -5], vel=[0, 0, 0], vel_sp=[1.0, 0.0, 0], ref=ref, dt=dt,
                neighbours=[(np.array([1.6, 0, -5]), True)])
    check('碰撞预警抑制接近分量 (vx<=0)', r['vel_sp'][0] <= 1e-6)

    # 6) 估计变脏 → 零速 + 持续升级
    sf = SafetyFilter(escalate_frames=15)
    for _ in range(20):
        r = sf.step(pos=[1, 0, -5], vel=[0, 0, 0], vel_sp=[1.0, 0, 0], ref=ref, dt=dt, est_ok=False)
    check('估计不健康升级到 HOLD/RELINQUISH', r['state'] in (HOLD, RELINQUISH)
          and float(np.linalg.norm(r['vel_sp'])) < 1e-6)

    # 7) NaN 指令 → 零速
    sf = SafetyFilter()
    r = sf.step(pos=[1, 0, -5], vel=[0, 0, 0], vel_sp=[float('nan'), 0, 0], ref=ref, dt=dt)
    check('NaN 指令归零', float(np.linalg.norm(r['vel_sp'])) < 1e-6)

    # 8) leader 巨跳 → jerk 限制（首帧速度增量受限）
    sf = SafetyFilter(max_accel=2.0, accel_slack=1.5, max_speed=10.0)
    r = sf.step(pos=[0, 0, -5], vel=[0, 0, 0], vel_sp=[10, 0, 0], ref=ref, dt=dt)
    check('巨跳被 jerk 限制 (首帧 vx 远小于 10)', r['vel_sp'][0] < 1.0)

    # 9) RELINQUISH → publish=False（交还 PX4）；grace_frames=0 排除冷启动宽限干扰
    sf = SafetyFilter(escalate_frames=5, relinquish_frames=10, grace_frames=0)
    for _ in range(40):
        r = sf.step(pos=[1, 0, -5], vel=[0, 0, 0], vel_sp=[1, 0, 0], ref=ref, dt=dt, est_ok=False)
    check('持续失效 → RELINQUISH 且 publish=False', r['state'] == RELINQUISH and r['publish'] is False)

    # 10) 速度硬帽
    sf = SafetyFilter(max_speed=1.5, max_accel=100.0)
    r = sf.step(pos=[0, 0, -5], vel=[1.4, 0, 0], vel_sp=[5, 0, 0], ref=ref, dt=dt)
    check('水平速度硬帽 1.5', float(np.linalg.norm(r['vel_sp'][:2])) <= 1.5 + 1e-6)

    # 11) RELINQUISH sticky：故障消失后连续 clear_frames 帧正常，状态仍应保持 RELINQUISH
    # grace_frames=0 排除冷启动宽限干扰（宽限期行为见 13）
    sf = SafetyFilter(escalate_frames=5, relinquish_frames=10, clear_frames=5, grace_frames=0)
    for _ in range(40):   # 先触发 RELINQUISH
        sf.step(pos=[1, 0, -5], vel=[0, 0, 0], vel_sp=[1, 0, 0], ref=ref, dt=dt, est_ok=False)
    for _ in range(20):   # 故障消失，连续正常帧（远超 clear_frames=5）
        r = sf.step(pos=[1, 0, -5], vel=[0, 0, 0], vel_sp=[0.5, 0, 0], ref=ref, dt=dt, est_ok=True)
    check('RELINQUISH sticky（故障消失后不自恢复）', r['state'] == RELINQUISH and r['publish'] is False)

    # 11b) reset()：唯一解闩路径（mpc_node 在 disarmed→armed 新架次边沿调用）。
    #      复位后须回 NORMAL 且恢复发布；grace 窗口也须重开（_frame 归零），
    #      否则新架次起飞穿越 min_alt 会被误判。用独立实例（非零 grace_frames，
    #      与 11 的 grace_frames=0 隔离），驱动越过 grace+relinquish 到 RELINQUISH
    #      后 reset()，验证宽限期确实重新打开。
    sf_reset = SafetyFilter(escalate_frames=5, relinquish_frames=10, grace_frames=20)
    for _ in range(35):   # 越过 grace(20)+relinquish(10) 才能真正到 RELINQUISH
        sf_reset.step(pos=[1, 0, -5], vel=[0, 0, 0], vel_sp=[1, 0, 0], ref=ref, dt=dt, est_ok=False)
    sf_reset.reset()
    r = sf_reset.step(pos=[1, 0, 0.1], vel=[0, 0, 0], vel_sp=[0.5, 0, 0], ref=[10, 0, 0], dt=dt, est_ok=True)
    check('reset() 解闩：回 NORMAL 且 publish=True', r['state'] == NORMAL and r['publish'] is True)
    check('reset() 重开 grace 窗口（低空/远离参考不误触）', 'fence_alt_low' not in str(r['reasons'])
          and 'flyaway' not in str(r['reasons']))

    # 12) grace_frames：冷启动期间 alt_low + flyaway 不触发，jerk/碰撞/NaN 仍激活
    sf = SafetyFilter(grace_frames=50, d_emergency=1.2, max_track_dist=5.0)
    r = sf.step(pos=[0, 0, 0.1], vel=[0, 0, 0],   # alt=−0.1m < min_alt, track=10m
                vel_sp=[1, 0, 0], ref=[10, 0, 0], dt=dt)
    check('grace 期 alt_low/flyaway 不触发(state=NORMAL)', r['state'] == NORMAL and 'fence_alt_low' not in str(r['reasons']))
    # grace 期碰撞地板仍激活
    r = sf.step(pos=[0, 0, -5], vel=[0, 0, 0], vel_sp=[1, 0, 0], ref=[0, 0, -5], dt=dt,
                neighbours=[(np.array([1.0, 0, -5]), True)])
    check('grace 期碰撞地板仍触发', 'collide_emerg' in str(r['reasons']))

    # 13) grace 期不升级到 RELINQUISH（只降级到 HOLD，仍 publish）；grace 结束后
    #     若坏状态未消退，仍按原逻辑正常升级到 RELINQUISH（不削弱对真实持续故障的保护）
    sf = SafetyFilter(escalate_frames=5, relinquish_frames=10, grace_frames=30)
    for _ in range(30):   # 全程在 grace 内，持续 est_unhealthy
        r = sf.step(pos=[1, 0, -5], vel=[0, 0, 0], vel_sp=[1, 0, 0], ref=ref, dt=dt, est_ok=False)
    check('grace 期持续故障只降级到 HOLD（不交还）', r['state'] == HOLD and r['publish'] is True)
    for _ in range(15):   # grace 结束后故障仍在 → 重新计满 relinquish_frames 后应正常升级
        r = sf.step(pos=[1, 0, -5], vel=[0, 0, 0], vel_sp=[1, 0, 0], ref=ref, dt=dt, est_ok=False)
    check('grace 结束后持续故障仍正常升级到 RELINQUISH', r['state'] == RELINQUISH and r['publish'] is False)

    print(f'\n自测: {ok}/{total} 通过')
    return ok == total


if __name__ == '__main__':
    import sys
    sys.exit(0 if _selftest() else 1)
