"""
실험 0 오프라인 보정: P5 Gazebo 원자료 [측정] ↔ E1 P5 작업량 재현 [계산].

    python3 -m layopt.sim.calib gazebo --raw /home/CAPYI_AI/runs/b24/b4-p5/raw --out g.json
    python3 -m layopt.sim.calib compare --gazebo g.json --e1 <e1 run_dir> [--e1 …] --out <dir>

지표 정의 (결과를 보기 전에 고정 — 노트 항목 2026-10-10_0254_layopt-sim_e1-p5-calibration):
- makespan = 마지막 완료 − 게시 시각(t_base 60 s). 평균 작업 시간 = 완료 작업의 (완료 − 게시) 평균.
  Gazebo 는 metrics.json makespan · mean_task_s (valid: true 7 회만).
- 게이트 대기 구간:
  - Gazebo 분산 모드: 로봇의 agent_event gate 사건 중 blocking 이 비지 않은 것 → 그 로봇의 다음 gate 사건까지
    (blocking 'zone:?pose' 는 기동 자세 확인이라 뺀다). 구역 = blocking 첫 항목.
  - Gazebo 중앙 모드: 'hold' 사건의 true 구간 (true → 다음 false). 구역 = 구간 시작 때 gt 자세에서 가장 가까운
    구역(상자 거리 ≤ 3 m, 없으면 'none').
  - Gazebo 분산 모드의 'hold' true 구간(교차 양보 등)도 같은 방법으로 따로 센다 — 실행당 합계는 둘의 합.
  - E1: extras.hold_episodes (주행 단계에서 TrafficManager hold 를 받은 구간, 포켓 양보 주행 제외).
    구역 = 이유 'zone:a+b' 의 첫 구역, 교차 양보 · 사건 hold 는 구간 끝 자세에서 가장 가까운 구역.
  - 정지 1 회 = 구간 ≥ 1 s. 실행당 = 모든 구간 합. 정지 위치 = 구간 ≥ 5 s 의 구역 분포.
- 교착: Gazebo traffic 사건 traffic/DEADLOCK · UNRESOLVED 수, E1 TrafficManager.stats.
- 주행 거리: 로봇당 (Gazebo metrics distance_m, E1 extras.robot.distance_m).
"""

from __future__ import annotations

import argparse
import bisect
import collections
import csv
import json
import math
import os
import pathlib
import statistics
import sys
from typing import Dict, List, Optional, Tuple

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
ZONES_YAML = ROOT / 'src' / 'amr_fleet' / 'config' / 'traffic_zones.yaml'
STOP_MIN_S = 1.0
LOC_MIN_S = 5.0
T_BASE = 60.0


def _boxes(path=ZONES_YAML) -> Dict[str, Tuple[float, float, float, float]]:
    doc = yaml.safe_load(pathlib.Path(path).read_text(encoding='utf-8'))
    out = {}
    for z in doc['zones']:
        xs = [p[0] for p in z['polygon']]
        ys = [p[1] for p in z['polygon']]
        out[z['id']] = (min(xs), max(xs), min(ys), max(ys))
    return out


def nearest_zone(x: float, y: float, boxes, max_d: float = 3.0) -> str:
    best, bd = 'none', max_d
    for zid, (x0, x1, y0, y1) in boxes.items():
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        d = math.hypot(dx, dy)
        if d <= bd:
            best, bd = zid, d
    return best


# ---------------------------------------------------------------------- Gazebo
def gazebo_run(d: pathlib.Path, boxes) -> Optional[dict]:
    m = json.loads((d / 'metrics.json').read_text(encoding='utf-8'))
    if not m.get('valid'):
        return None
    text = (d / 'events.jsonl').read_text(encoding='utf-8')
    ev = [json.loads(line) for line in text.splitlines() if line.strip()]
    gt: Dict[str, List[Tuple[float, float, float]]] = collections.defaultdict(list)
    with open(d / 'gt.csv', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            gt[r['robot']].append((float(r['t']), float(r['x']), float(r['y'])))
    gtt = {r: [p[0] for p in v] for r, v in gt.items()}

    def pose_at(rid, t):
        i = max(0, bisect.bisect_right(gtt[rid], t) - 1)
        return gt[rid][i][1], gt[rid][i][2]

    stops = []                                   # (robot, t0, dur, zone, source)
    gates: Dict[str, list] = collections.defaultdict(list)
    holds: Dict[str, list] = collections.defaultdict(list)
    for e in ev:
        if e.get('k') == 'agent_event' and (e.get('ev') or {}).get('event') == 'gate':
            gates[e['r']].append((e['t'], [str(b) for b in e['ev'].get('blocking') or []]))
        elif e.get('k') == 'hold':
            holds[e['r']].append((e['t'], bool(e['hold'])))
    t_end = float(m.get('t_end') or 0.0)
    for rid, seq in gates.items():
        for i, (t, blk) in enumerate(seq):
            blk = [b for b in blk if b != 'zone:?pose']
            if not blk:
                continue
            t1 = seq[i + 1][0] if i + 1 < len(seq) else t_end
            stops.append((rid, t, t1 - t, blk[0].replace('zone:', ''), 'gate'))
    for rid, seq in holds.items():
        cur = None
        for t, h in seq:
            if h and cur is None:
                cur = t
            elif not h and cur is not None:
                x, y = pose_at(rid, cur)
                stops.append((rid, cur, t - cur, nearest_zone(x, y, boxes), 'hold'))
                cur = None
    tc = collections.Counter(e['name'] for e in ev if e.get('k') == 'traffic')
    return {
        'run': d.name, 'mode': m.get('mode'), 'completed': m.get('completed'),
        'n_tasks': m.get('n_tasks'), 'makespan': m.get('makespan'),
        'mean_task_s': m.get('mean_task_s'),
        'task_dt': [t['dt'] for t in m.get('tasks', []) if t.get('dt') is not None],
        'distance_m': m.get('distance_m'),
        'stops': [[r, round(t, 2), round(dur, 2), z, src] for r, t, dur, z, src in stops],
        'deadlocks': tc.get('traffic/DEADLOCK', 0), 'unresolved': tc.get('traffic/UNRESOLVED', 0),
        'traffic_counts': dict(tc),
    }


def cmd_gazebo(a) -> int:
    boxes = _boxes()
    runs = []
    for d in sorted(pathlib.Path(a.raw).glob('p5_*')):
        if not (d / 'metrics.json').is_file():
            continue
        r = gazebo_run(d, boxes)
        if r is not None:
            runs.append(r)
    out = {'source': str(a.raw), 'label': '[측정]', 'n_valid': len(runs), 'runs': runs}
    pathlib.Path(a.out).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'Gazebo 유효 {len(runs)} 회 → {a.out}')
    return 0


# ---------------------------------------------------------------------- E1
def e1_runs(run_dir: pathlib.Path, boxes) -> List[dict]:
    out = []
    for line in (run_dir / 'extras.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        x = json.loads(line)
        comp = list(x['completions'].values())
        stops = []
        for t0, dur, reason, hx, hy in x['hold_episodes']:
            if reason.startswith('zone:'):
                z = reason[5:].split('+')[0]
            else:
                z = nearest_zone(hx, hy, boxes)
            stops.append([None, t0, dur, z, reason.split(':')[0] or 'hold'])
        out.append({
            'spec_id': x['spec_id'], 'seed': x['seed'], 'completed': len(comp),
            'n_tasks': x['n_tasks'],
            'makespan': round(max(comp) - T_BASE, 2) if comp else None,
            'mean_task_s': round(sum(c - T_BASE for c in comp) / len(comp), 2) if comp else None,
            'distance_m': {r: v['distance_m'] for r, v in x['robot'].items()},
            'stops': stops,
            'deadlocks': x['tm_events'].get('traffic/DEADLOCK', 0),
            'unresolved': x['tm_events'].get('traffic/UNRESOLVED', 0),
        })
    return out


# ---------------------------------------------------------------------- 비교
def _q(xs: List[float], q: float) -> Optional[float]:
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def summarize(runs: List[dict]) -> dict:
    per_run = [sum(s[2] for s in r['stops']) for r in runs]
    per_stop = [s[2] for r in runs for s in r['stops'] if s[2] >= STOP_MIN_S]
    long_stops = [s for r in runs for s in r['stops'] if s[2] >= LOC_MIN_S]
    loc = collections.Counter(s[3] for s in long_stops)
    loc_s = collections.Counter()
    for s in long_stops:
        loc_s[s[3]] += s[2]
    tot = sum(loc_s.values()) or 1.0
    dist = [v for r in runs for v in (r['distance_m'] or {}).values() if v and v > 5.0]

    def stat(xs):
        xs = [x for x in xs if x is not None]
        if not xs:
            return None
        return {'n': len(xs), 'min': round(min(xs), 1), 'q05': round(_q(xs, 0.05), 1),
                'q25': round(_q(xs, 0.25), 1), 'median': round(_q(xs, 0.5), 1),
                'q75': round(_q(xs, 0.75), 1), 'q95': round(_q(xs, 0.95), 1),
                'max': round(max(xs), 1), 'mean': round(statistics.fmean(xs), 1)}
    return {
        'n_runs': len(runs),
        'completed': stat([r['completed'] for r in runs]),
        'makespan': stat([r['makespan'] for r in runs]),
        'mean_task_s': stat([r['mean_task_s'] for r in runs]),
        'gate_wait_per_run_s': stat(per_run),
        'gate_wait_per_stop_s': stat(per_stop),
        'stops_ge1s_per_run': stat([sum(1 for s in r['stops'] if s[2] >= STOP_MIN_S)
                                    for r in runs]),
        'stop_locations_ge5s': {z: {'n': loc[z], 'share_time': round(loc_s[z] / tot, 3)}
                                for z in sorted(loc, key=lambda z: -loc_s[z])},
        'deadlocks_per_run': stat([r['deadlocks'] for r in runs]),
        'unresolved_per_run': stat([r['unresolved'] for r in runs]),
        'runs_with_deadlock': sum(1 for r in runs if r['deadlocks']),
        'distance_per_robot_m': stat(dist),
    }


def within(e1: List[float], lo: float, hi: float) -> float:
    xs = [x for x in e1 if x is not None]
    return round(sum(1 for x in xs if lo <= x <= hi) / len(xs), 3) if xs else float('nan')


def cmd_compare(a) -> int:
    boxes = _boxes()
    g = json.loads(pathlib.Path(a.gazebo).read_text(encoding='utf-8'))
    out = {'gazebo': {'label': '[측정]', 'all': summarize(g['runs']),
                      'distributed': summarize([r for r in g['runs']
                                                if r['mode'] == 'distributed']),
                      'central': summarize([r for r in g['runs'] if r['mode'] == 'central'])},
           'e1': {}}
    for d in a.e1:
        runs = e1_runs(pathlib.Path(d), boxes)
        by = collections.defaultdict(list)
        for r in runs:
            by[r['spec_id']].append(r)
        for sid, rs in by.items():
            s = summarize(rs)
            gs = out['gazebo']['all']
            s['frac_in_gazebo_range'] = {
                k: within([r[k] for r in rs], gs[k]['min'], gs[k]['max'])
                for k in ('makespan', 'mean_task_s')}
            pr = [sum(x[2] for x in r['stops']) for r in rs]
            s['frac_in_gazebo_range']['gate_wait_per_run_s'] = within(
                pr, gs['gate_wait_per_run_s']['min'], gs['gate_wait_per_run_s']['max'])
            out['e1'][f'{sid} ({os.path.basename(d)})'] = dict(s, label='[계산]', run_dir=str(d))
    od = pathlib.Path(a.out)
    od.mkdir(parents=True, exist_ok=True)
    (od / 'compare.json').write_text(json.dumps(out, ensure_ascii=False, indent=1),
                                     encoding='utf-8')
    (od / 'compare.md').write_text(render(out), encoding='utf-8')
    print(render(out))
    return 0


def _fmt(s: Optional[dict]) -> str:
    if not s:
        return '—'
    return f"{s['median']} [{s['min']}–{s['max']}] (n={s['n']})"


def _fmt_e1(s: Optional[dict]) -> str:
    if not s:
        return '—'
    return f"{s['median']} (IQR {s['q25']}–{s['q75']}, 5–95% {s['q05']}–{s['q95']}, n={s['n']})"


def render(out: dict) -> str:
    rows = [('완료 작업', 'completed'), ('makespan [s]', 'makespan'),
            ('평균 작업 시간 [s]', 'mean_task_s'), ('게이트 대기/실행 [s]', 'gate_wait_per_run_s'),
            ('게이트 대기/정지(≥1 s) [s]', 'gate_wait_per_stop_s'),
            ('정지(≥1 s) 수/실행', 'stops_ge1s_per_run'), ('교착/실행', 'deadlocks_per_run'),
            ('UNRESOLVED/실행', 'unresolved_per_run'), ('로봇당 주행 [m]', 'distance_per_robot_m')]
    g = out['gazebo']
    cols = list(out['e1'])
    head = '| 지표 | Gazebo 전체 [측정] | 분산 [측정] | 중앙 [측정] | ' + \
        ' | '.join(f'E1 {c} [계산]' for c in cols) + ' |'
    lines = [head, '|' + '---|' * (4 + len(cols))]
    for name, k in rows:
        cells = [_fmt(g['all'][k]), _fmt(g['distributed'][k]), _fmt(g['central'][k])]
        cells += [_fmt_e1(out['e1'][c][k]) for c in cols]
        lines.append(f'| {name} | ' + ' | '.join(cells) + ' |')
    lines.append('')
    lines.append('정지 위치 (≥ 5 s, 시간 비율):')
    loc = json.dumps(g['all']['stop_locations_ge5s'], ensure_ascii=False)
    lines.append(f'- Gazebo [측정]: {loc}')
    for c in cols:
        lines.append(f"- E1 {c} [계산]: "
                     f"{json.dumps(out['e1'][c]['stop_locations_ge5s'], ensure_ascii=False)}")
        lines.append(f"  Gazebo 범위 안 비율: {out['e1'][c]['frac_in_gazebo_range']}")
    return '\n'.join(lines) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='E1 보정 (실험 0 오프라인)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('gazebo')
    g.add_argument('--raw', required=True)
    g.add_argument('--out', required=True)
    c = sub.add_parser('compare')
    c.add_argument('--gazebo', required=True)
    c.add_argument('--e1', action='append', required=True)
    c.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    return cmd_gazebo(a) if a.cmd == 'gazebo' else cmd_compare(a)


if __name__ == '__main__':
    sys.exit(main())
