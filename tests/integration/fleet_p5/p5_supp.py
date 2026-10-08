#!/usr/bin/env python3
"""
보너스 4 P5 보충 분석 (사전 등록 외 — 결과를 본 뒤 만든 탐색 지표, 판정에 쓰지 않는다).

구역 '스침': 정답 원판(r 0.36 m)이 구역 다각형과 겹치는데 **중심은 밖**인 구간. 구역 방문 판정(traffic_zones.zone_visits)이 경로의
중심점만 보므로, 경계를 따라 지나는 로봇은 그 구역을 요청하지 않고 몸체 일부가 구역 안에 들어간다 (AA 짝 1 a 에서 발견, 노트).
두 모드 모두 같은 함수를 쓴다 (중앙 교통 관리자 · 에이전트 경로 게이트).

  python3 p5_supp.py graze <원자료 루트> [--exp AA]
      → 갈래별 스침 구간 수 · 총 시간 · 최대 깊이, 지나침(중심이 앞뒤 2 s 안에 안 들어감), 다른 로봇이 그 구역 안인지
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import p5_analyze as A  # noqa: E402


def grazing(gt, zones, t0, t1):
    """[{robot, zone, t0, t1, depth_m, other_inside}] — 원판 겹침 · 중심 밖 구간."""
    out = []
    open_ = {}
    for t, cur in A.bins(gt, t0, t1):
        inside_c = {z: {r for r, (x, y) in cur.items() if A.inside(p, x, y)}
                    for z, p in zones.items()}
        seen = set()
        for r, (x, y) in cur.items():
            for z, p in zones.items():
                d = A.signed_distance(p, x, y)
                if 0.0 <= d < A.BODY_R:
                    key = (r, z)
                    seen.add(key)
                    e = open_.get(key)
                    other = sorted(inside_c[z] - {r})
                    if e is None:
                        e = open_[key] = {'robot': r, 'zone': z, 't0': round(t, 2),
                                          't1': round(t, 2), 'depth_m': 0.0,
                                          'other_inside': []}
                    e['t1'] = round(t + A.BIN, 2)
                    e['depth_m'] = round(max(e['depth_m'], A.BODY_R - d), 3)
                    if other:
                        e['other_inside'] = sorted(set(e['other_inside']) | set(other))
        for key in list(open_):
            if key not in seen:
                out.append(open_.pop(key))
    out.extend(open_.values())
    return sorted(out, key=lambda e: e['t0'])


def run(raw):
    meta = A.read_json(os.path.join(raw, 'meta.json'), {}) or {}
    summ = A.read_json(os.path.join(raw, 'summary.json'), {}) or {}
    if summ.get('why') != 'DONE' or not meta.get('zones_text'):
        return None
    zones = A.load_zones(meta['zones_text'])
    gt = A.read_gt(os.path.join(raw, 'gt.csv'))
    ev = grazing(gt, zones, summ['t_base'], summ['t_end'])
    ev = [e for e in ev if e['t1'] - e['t0'] >= 0.2]          # 한 칸 스침(잡음) 제외
    # 지나침 = 그 스침 앞뒤 2 s 안에 그 로봇 중심이 그 구역에 들어가지 않음 (진입 직전 · 직후의 앞뒤 몸체가 아니다).
    # 진입 앞뒤 스침은 결정점 거리(차체 앞 절반 0.30 m 포함)가 다루는 것이고, 지나침은 구역 방문 판정이 아예 요청하지 않는 것이다.
    for e in ev:
        v, poly = gt.get(e['robot'], []), zones[e['zone']]
        e['pass_by'] = not any(A.inside(poly, s_[1], s_[2]) for s_ in v
                               if e['t0'] - 2.0 <= s_[0] <= e['t1'] + 2.0)
    held = A.timeline(A.read_jsonl(os.path.join(raw, 'events.jsonl')), 'agent_state', 'held')
    for e in ev:
        h = A.value_at(held.get(e['robot'], []), e['t0'], None)
        e['held'] = None if h is None else (f"zone:{e['zone']}" in h)
    pb = [e for e in ev if e['pass_by']]
    return {'tag': os.path.basename(raw), 'mode': (meta.get('plan') or {}).get('mode'),
            'n': len(ev), 'total_s': round(sum(e['t1'] - e['t0'] for e in ev), 1),
            'max_depth_m': max((e['depth_m'] for e in ev), default=0.0),
            'pass_by_n': len(pb), 'pass_by_s': round(sum(e['t1'] - e['t0'] for e in pb), 1),
            'pass_by_max_depth_m': max((e['depth_m'] for e in pb), default=0.0),
            'pass_by_unheld': [e for e in pb if e['held'] is False],
            'with_other_inside': [e for e in ev if e['other_inside']], 'events': ev}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['graze'])
    ap.add_argument('root')
    ap.add_argument('--exp', default='')
    a = ap.parse_args(argv)
    rows = []
    for name in sorted(os.listdir(a.root)):
        if not name.startswith('p5_') or (a.exp and not name.startswith(f'p5_{a.exp}_')):
            continue
        r = run(os.path.join(a.root, name))
        if r:
            rows.append(r)
            print(f"{r['tag']} {r['mode']} 스침 {r['n']} 건 {r['total_s']} s · "
                  f"지나침 {r['pass_by_n']} 건 "
                  f"{r['pass_by_s']} s 최대 {r['pass_by_max_depth_m']} m (허가 없음 "
                  f"{len(r['pass_by_unheld'])}) · 다른 로봇 안 {len(r['with_other_inside'])}")
    with open(os.path.join(a.root, 'analysis', 'supp_graze.json'), 'w', encoding='utf-8') as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
