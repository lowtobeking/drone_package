#!/usr/bin/env python3
"""场景活动包络静态检查 —— 场景配置 vs 飞控围栏 / 场地尺寸。

    python3 tools/check_scenario_envelope.py                    # 查全部
    python3 tools/check_scenario_envelope.py --prefix IN_        # 只查室内场景
    python3 tools/check_scenario_envelope.py --geofence-hor 2.0 --field-radius 2.5

**为什么需要这个工具**：2026-07-23 在 SITL 里实飞才发现 `IN_solo1_circle`
配 `radius: 1.5` 实际会跑到离起飞点 **3.005 m**（`leader_node.py:232` 是
`cx = x0 - radius`，圆心偏置在起飞点旁、起飞点落在圆周上 ⇒ 最远处是 **2×radius**），
超出 5×5 场地的可用半径、也超出飞控围栏 `GF_MAX_HOR_DIST=2.0`。
靠实飞撞出来的问题应该静态就能查出来 —— 尤其飞场只有一次机会。

**两套约束性质不同，必须分开算**：

- **飞控围栏** `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST`：PX4 是相对**各机自己的
  home（起飞点）**判定的，不是场地中心。超了触发 `GF_ACTION`（当前 Hold），
  会中断试验但不撞。
- **场地边界**：相对**场地中心**。超了是**撞墙**。
  多机时队形偏移把两者拉开：`pair2_in` 各机距中心 1.4 m，即使原地不动，
  离中心也已经 1.4 m。

轨迹语义（与 `mpc_control/leader_node.py` 对齐，改那边记得同步这里）：

| leader.mode | leader 轨迹 | 离起点最远 |
|---|---|---|
| `hover`  | 恒在 (0,0) | 0 |
| `circle` | 圆心 `(-radius, 0)`、半径 `radius`（起点在圆周上） | **2×radius** |
| `line`   | `(0,0)` → `(sgn·max_distance, 0)` | `max_distance` |

各机目标位置 = leader(t) + `offsets[i]`（`offsets` 缺省取 `birth`，
再乘 `offsets_scale`）。本工具算的是**几何参考轨迹**，不含跟踪暂态
——真实峰值还要叠加 `safety_max_track_dist` 量级的偏差，故留余量。
"""
import argparse
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(HERE, '..', 'config', 'scenarios.yaml')


def leader_samples(leader, n=720):
    """按 leader_node 的语义采样参考轨迹，返回 [(x, y), ...]（世界系 NED）。"""
    mode = leader.get('mode', 'hover')
    if mode == 'circle':
        r = float(leader.get('radius', 10.0))
        cx, cy = -r, 0.0        # leader_node.py:232  cx = x0 - radius（x0=0）
        return [(cx + r * math.cos(2 * math.pi * k / n),
                 cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    if mode == 'line':
        s_max = float(leader.get('max_distance', 20.0))
        sgn = 1.0 if float(leader.get('speed', 1.0)) >= 0 else -1.0
        return [(sgn * s_max * k / n, 0.0) for k in range(n + 1)]
    return [(0.0, 0.0)]         # hover


def analyse(cfg, name, scen, args):
    fm_name = scen.get('formation', 'cross5')
    fm = cfg['formations'][fm_name]
    # ⚠️ 场景级 birth_override 优先（swarm_launch.py:70 / gen_spawn.py:60 都这么取）。
    # 漏了它会凭空算出一条"从队形出生点收敛到缩放槽位"的腿：IN_trio3_* 用
    # offsets_scale 0.5 缩小队形，同时用 birth_override 把出生点也移到缩放后的
    # 位置，两者是配套的。只读 formations[].birth 会误报 1.5m 的收敛位移。
    births = scen.get('birth_override', fm['birth'])
    offsets = fm.get('offsets', fm['birth'])
    scale = scen.get('offsets_scale')
    if scale:
        offsets = [[v * float(scale) for v in pt] for pt in offsets]

    lead = leader_samples(scen.get('leader', {}))

    max_home = 0.0      # 离**自身起飞点**最远（对围栏）
    max_center = 0.0    # 离**场地中心**最远（对撞墙）
    worst_home_i = worst_center_i = 0
    for i, (b, off) in enumerate(zip(births, offsets)):
        for lx, ly in lead:
            tx, ty = lx + off[0], ly + off[1]
            d_home = math.hypot(tx - b[0], ty - b[1])
            d_ctr = math.hypot(tx, ty)
            if d_home > max_home:
                max_home, worst_home_i = d_home, i
            if d_ctr > max_center:
                max_center, worst_center_i = d_ctr, i

    limits = dict(cfg.get('defaults', {}))
    limits.update(scen.get('limits', {}))
    target_alt = float(limits.get('target_alt', -5.0))
    alt = abs(target_alt)      # NED 负向上

    issues = []
    if max_home > args.geofence_hor:
        issues.append(f'水平围栏: 离自身起飞点 {max_home:.2f}m > '
                      f'GF_MAX_HOR_DIST {args.geofence_hor}m (机 {worst_home_i})')
    if max_center > args.field_radius:
        issues.append(f'撞墙: 离场地中心 {max_center:.2f}m > 可用半径 '
                      f'{args.field_radius}m (机 {worst_center_i})')
    if alt > args.geofence_ver:
        issues.append(f'垂直围栏: 目标高度 {alt:.2f}m > '
                      f'GF_MAX_VER_DIST {args.geofence_ver}m')
    smax = limits.get('safety_max_alt')
    if smax is not None and alt > float(smax):
        issues.append(f'目标高度 {alt:.2f}m > safety_max_alt {float(smax):.2f}m')

    # 故障注入场景可能**故意**违反包络（如 S31 用 max_alt<target_alt 诱发
    # 限高压帽→HOLD→RELINQUISH 链）。这类场景在 scenarios.yaml 里显式标
    # `envelope_check: false`，否则工具会有已知误报、久而久之没人看。
    if scen.get('envelope_check', True) is False:
        return dict(name=name, formation=fm_name, n=len(births),
                    max_home=max_home, max_center=max_center, alt=alt,
                    issues=[], warns=[], skipped=True,
                    skip_note=f'故意违反包络（{len(issues)} 项已豁免）')

    warns = []
    m = args.margin
    if not issues:
        if max_home > args.geofence_hor - m:
            warns.append(f'离围栏仅剩 {args.geofence_hor - max_home:.2f}m 余量')
        if max_center > args.field_radius - m:
            warns.append(f'离墙仅剩 {args.field_radius - max_center:.2f}m 余量')

    return dict(name=name, formation=fm_name, n=len(births),
                max_home=max_home, max_center=max_center, alt=alt,
                issues=issues, warns=warns, skipped=False, skip_note='')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=DEFAULT_CFG)
    ap.add_argument('--prefix', default='', help='只查名字以此开头的场景')
    # 默认值 = 2026-07-23 实际写入 X6 Air+ 的围栏 + 5×5m 飞场
    ap.add_argument('--geofence-hor', type=float, default=2.0)
    ap.add_argument('--geofence-ver', type=float, default=3.0)
    ap.add_argument('--field-radius', type=float, default=2.5)
    ap.add_argument('--margin', type=float, default=0.4,
                    help='低于此余量只告警不判失败')
    a = ap.parse_args()

    with open(a.config, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    rows = [analyse(cfg, n, s, a) for n, s in cfg['scenarios'].items()
            if n.startswith(a.prefix)]
    if not rows:
        print(f'没有匹配 prefix={a.prefix!r} 的场景'); return 1

    print(f'围栏 水平{a.geofence_hor}m / 垂直{a.geofence_ver}m，'
          f'场地可用半径 {a.field_radius}m，告警余量 {a.margin}m\n')
    print(f'{"场景":<26}{"队形":<12}{"机":>3}{"离起飞点":>10}{"离中心":>9}{"高度":>7}  判定')
    print('-' * 88)
    bad = warn = skip = 0
    for r in sorted(rows, key=lambda x: -x['max_center']):
        if r['skipped']:
            verdict, skip = '➖', skip + 1
        elif r['issues']:
            verdict, bad = '❌', bad + 1
        elif r['warns']:
            verdict, warn = '⚠️', warn + 1
        else:
            verdict = '✅'
        print(f'{r["name"]:<26}{r["formation"]:<12}{r["n"]:>3}'
              f'{r["max_home"]:>10.2f}{r["max_center"]:>9.2f}{r["alt"]:>7.2f}  {verdict}')
        for t in r['issues'] + r['warns'] + ([r['skip_note']] if r['skipped'] else []):
            print(f'{"":>26}   └ {t}')

    print('-' * 88)
    print(f'共 {len(rows)} 个场景：{len(rows) - bad - warn - skip} 通过 / '
          f'{warn} 告警 / {bad} 超限 / {skip} 豁免')
    print('\n注：本工具只算**几何参考轨迹**，不含跟踪暂态。真实峰值还要叠加'
          '\n    safety_max_track_dist 量级的偏差，故不要贴着限值配。')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
