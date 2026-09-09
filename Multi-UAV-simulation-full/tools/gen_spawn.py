#!/usr/bin/env python3
"""gen_spawn.py — 从 config/scenarios.yaml 生成 PX4 SITL 出生 POSES(Gazebo ENU)。

根治 swarm_launch.py 的 BIRTH_* ↔ start_N_px4.sh 的 POSES 两处漂移：
两边都只认 scenarios.yaml 一份 birth，POSES 由本脚本生成。

坐标: scenarios.yaml 的 birth = 世界系 NED [North, East, Down]。
      PX4_GZ_MODEL_POSE 要 Gazebo ENU "x_east,y_north,z_up,roll,pitch,yaw"。
      换算只在这里做一次:  ENU=(East,North)=(NED[1],NED[0])。

用法:
  # 打印某队形/工况的 bash POSES 数组（贴进/生成 start_N_px4.sh）
  py tools/gen_spawn.py --formation cross5
  py tools/gen_spawn.py --scenario S11_cross5_perturbed
  py tools/gen_spawn.py --formation grid9 --format lines     # 仅 POSE 串，每行一个

  # yaml 自检：队形/工况一致性 + 出生间距体检
  py tools/gen_spawn.py --validate
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_YAML = os.path.join(REPO, 'config', 'scenarios.yaml')

try:
    import yaml
except ImportError:
    sys.exit('缺少 PyYAML：先 `py -m pip install pyyaml`（Ubuntu 上 ROS 自带，无需装）')


# ── 数字/坐标格式化 ──────────────────────────────────────────────────────────
def _num(v):
    """最简数字串：3.0→'3'、2.598→'2.598'、-1.5→'-1.5'、-0.0→'0'。匹配现有脚本写法。"""
    s = f'{float(v):g}'
    return '0' if s == '-0' else s


def ned_to_pose(ned):
    """NED [N,E,D(,...)] → Gazebo ENU pose 串 'east,north,up,roll,pitch,yaw'。"""
    n, e = float(ned[0]), float(ned[1])
    return f'{_num(e)},{_num(n)},0,0,0,0'


# ── 读 scenarios.yaml ────────────────────────────────────────────────────────
def load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def births_for(cfg, *, scenario=None, formation=None):
    """返回 (births[NED], formation_name)。scenario 的 birth_override 优先。"""
    if scenario:
        if scenario not in cfg['scenarios']:
            sys.exit(f'未知 scenario "{scenario}"，可选: {list(cfg["scenarios"])}')
        sc = cfg['scenarios'][scenario]
        formation = sc['formation']
        if 'birth_override' in sc:
            return sc['birth_override'], formation
    if formation not in cfg['formations']:
        sys.exit(f'未知 formation "{formation}"，可选: {list(cfg["formations"])}')
    return cfg['formations'][formation]['birth'], formation


def poses_for(births):
    return [ned_to_pose(b) for b in births]


# ── 出生间距体检（防"出生即相撞"）─────────────────────────────────────────────
def spacing_warnings(births, d_safe, tag):
    out = []
    for i in range(len(births)):
        for j in range(i + 1, len(births)):
            dn = float(births[i][0]) - float(births[j][0])
            de = float(births[i][1]) - float(births[j][1])
            d = (dn * dn + de * de) ** 0.5
            if d < d_safe:
                out.append(f'  ⚠ {tag}: drone{i}↔{j} 出生间距 {d:.3f} < d_safe {d_safe}')
    return out


# ── 输出 bash POSES 数组块 ───────────────────────────────────────────────────
def poses_block(poses):
    lines = ['# Gazebo ENU 出生位置  "x_east,y_north,z_up,roll,pitch,yaw"',
             '# 由 tools/gen_spawn.py 从 config/scenarios.yaml 生成；改 yaml 后重生成，勿手改',
             'declare -a POSES=(']
    width = max((len(p) for p in poses), default=0)
    for i, p in enumerate(poses):
        lines.append(f'    "{p}"'.ljust(width + 8) + f'# {i}')
    lines.append(')')
    return '\n'.join(lines)


# ── yaml 自检：队形/工况一致性 + 出生间距 ────────────────────────────────────
def cmd_validate(cfg):
    """检查 scenarios.yaml 自身一致性。脚本现由 gen_spawn 供 POSES、与 swarm_launch
       读同一份 birth，故无需再比对 start_N（'yaml 复现旧硬编码 5/5' 已在迁移前验证）。"""
    ok = True
    d_safe = cfg.get('defaults', {}).get('d_safe', 1.5)
    formations = cfg['formations']

    print('== 队形几何自检 ==')
    for name, fm in formations.items():
        births, nbr = fm['birth'], fm['neighbours']
        offs = fm.get('offsets', births)
        n = len(births)
        errs = []
        if len(nbr) != n:
            errs.append(f'neighbours 数 {len(nbr)} != 机数 {n}')
        if len(offs) != n:
            errs.append(f'offsets 数 {len(offs)} != 机数 {n}')
        for i, nb in enumerate(nbr):
            bad = [x for x in nb if not 0 <= x < n]
            if bad:
                errs.append(f'drone{i} 邻居越界 {bad}')
        ws = spacing_warnings(births, d_safe, name)
        if errs:
            ok = False
            print(f'  FAIL {name:6s} ' + '; '.join(errs))
        elif ws:
            print(f'  warn {name:6s} 出生间距偏近:')
            print('\n'.join(ws))
        else:
            print(f'  OK   {name:6s} 机数={n}，邻居/偏移/间距 一致')

    print('== 工况自检 ==')
    for sname, sc in cfg.get('scenarios', {}).items():
        fmn = sc.get('formation')
        errs = []
        if fmn not in formations:
            errs.append(f'formation "{fmn}" 不存在')
        elif 'birth_override' in sc and len(sc['birth_override']) != len(formations[fmn]['birth']):
            errs.append(f'birth_override 数 {len(sc["birth_override"])} != {fmn} 机数 {len(formations[fmn]["birth"])}')
        ws = spacing_warnings(births_for(cfg, scenario=sname)[0], d_safe, sname) if not errs else []
        if errs:
            ok = False
            print(f'  FAIL {sname}: ' + '; '.join(errs))
        elif ws:
            print(f'  warn {sname} 出生间距偏近:')
            print('\n'.join(ws))
        else:
            print(f'  OK   {sname}')
    return ok


# ── CLI ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='从 scenarios.yaml 生成 PX4 出生 POSES')
    ap.add_argument('--config', default=DEFAULT_YAML, help='scenarios.yaml 路径')
    ap.add_argument('--scenario', help='工况名，如 S11_cross5_perturbed')
    ap.add_argument('--formation', help='队形名，如 cross5')
    ap.add_argument('--format', choices=['poses', 'lines'], default='poses',
                    help='poses=bash 数组块(默认); lines=每行一个 POSE 串')
    ap.add_argument('--validate', action='store_true',
                    help='yaml 自检：队形/工况一致性 + 出生间距体检')
    args = ap.parse_args()

    cfg = load(args.config)

    if args.validate:
        sys.exit(0 if cmd_validate(cfg) else 1)

    if not (args.scenario or args.formation):
        ap.error('需 --scenario 或 --formation 之一（或 --validate）')

    births, formation = births_for(cfg, scenario=args.scenario, formation=args.formation)
    poses = poses_for(births)

    ws = spacing_warnings(births, cfg.get('defaults', {}).get('d_safe', 1.5),
                          args.scenario or formation)
    if ws:
        print('\n'.join(ws), file=sys.stderr)
        print('  ⚠ 出生点过近，实跑前请调整 scenarios.yaml', file=sys.stderr)

    print('\n'.join(poses) if args.format == 'lines' else poses_block(poses))


if __name__ == '__main__':
    main()
