#!/usr/bin/env python3
"""
단계 C 사후 진단 — Nav2 성공 순간의 선속도와 목표 지나침 (사전 등록 밖, 기술).

    overshoot_c.py --c2 /home/CAPYI_AI/runs/bonus1/b1-gzc/c2 --out overshoot_c2.json

리뷰(2026-10-08) 가 RL 주 집합 실패 `m034` 를 위치추정 탓이 아니라 "달리는 중 Nav2 성공 → 미끄러져 나감" 으로
바로잡은 뒤, 배포 goal checker(`SimpleGoalChecker`, xy·yaw 만 본다)가 선속도를 보지 않는 결함의 크기를 원자료에서
센다. 행 선택은 `analyze_c.load_arm` 과 같다 (첫 시도 우선, `_rerun` 대체). 에피소드 원자료
`<시도>/episodes/<exec:04d>_<scenario_id>.json.gz` 의 지면 진실 `gt` [t, x, y, yaw, v, ω] 에서 Nav2 결과 시각
`t_end` 에 가장 가까운 표본을 'Nav2 성공 순간' 으로 본다 (metrics.py 의 `w_at_success` 와 같은 방식).
"""

import argparse
import gzip
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analyze_c as A  # noqa: E402

STRICT_XY = 0.10   # 계약 엄격 성공 반경 [m] (metrics.STRICT_XY)
STRICT_V = 0.05    # 계약 엄격 성공 정지 문턱 [m/s] (metrics.STRICT_V)


def episode_path(row):
    """행 → 에피소드 원자료 경로."""
    return os.path.join(row['_src'], 'episodes',
                        f"{int(row['exec_index']):04d}_{row['scenario_id']}.json.gz")


def at_nav2_end(path):
    """
    Nav2 결과 시각의 지면 진실 (|v|, |ω|, 목표 거리).

    원자료가 없거나 gt 가 비면 None.
    """
    if not os.path.exists(path):
        return None
    with gzip.open(path, 'rt', encoding='utf-8') as fh:
        ep = json.load(fh)['episode']
    gt = np.asarray(ep.get('gt') or [], dtype=float)
    if gt.ndim != 2 or gt.shape[0] == 0 or ep.get('t_end') is None:
        return None
    k = int(np.argmin(np.abs(gt[:, 0] - float(ep['t_end']))))
    gx, gy = ep['goal'][0], ep['goal'][1]
    return {'v': abs(float(gt[k, 4])), 'w': abs(float(gt[k, 5])),
            'd': math.hypot(gt[k, 1] - gx, gt[k, 2] - gy)}


def classify(row):
    """
    Nav2 성공인데 엄격 성공(지면 진실)이 아닌 에피소드의 분류 (아니면 None).

    'stall_first' — 계약 정체 사건(20 s 동안 0.3 m 못 움직임)이 엄격 성공보다 먼저 났다,
    'never_in' — (Nav2 가 먼저 끝났고) 지면 진실로 0.10 m 안에 들어간 적이 없다 (추정 자세만 안 — 위치추정이 경계를 가름),
    'slid' — (Nav2 가 먼저 끝났고) 0.10 m 안을 지났지만 정지 문턱 아래로 서지 못한 채 정착 뒤 0.10 m 밖 (지나쳐 미끄러져 나감),
    'other' — 그 밖.
    """
    if not (A.truthy(row['system_success']) and not A.truthy(row['strict_gt'])):
        return None
    if row.get('contract_note') != 'ended_without_success':
        return 'stall_first'
    mind, fin = A.num(row['min_dist_gt_m']), A.num(row['final_dist_gt_m'])
    if mind > STRICT_XY:
        return 'never_in'
    if fin > STRICT_XY:
        return 'slid'
    return 'other'


def summarize(rows):
    """팔·집합 하나의 집계."""
    out = {'n': len(rows), 'nav2_success': 0, 'missing_episode': 0,
           'v_at_success_q': None, 'w_at_success_q': None,
           'success_moving_v_gt_0.05': 0, 'success_moving_v_gt_0.20': 0,
           'nav2_success_not_strict': {'stall_first': [], 'never_in': [], 'slid': [], 'other': []},
           'passed_within_0.10_ended_outside_any_outcome': []}
    vs, ws = [], []
    for r in rows:
        sid = r['scenario_id'].rsplit('_', 1)[-1]
        mind, fin = A.num(r['min_dist_gt_m']), A.num(r['final_dist_gt_m'])
        if mind <= STRICT_XY < fin:
            out['passed_within_0.10_ended_outside_any_outcome'].append(
                {'id': sid, 'outcome': f"{r['outcome_contract']}|{r['outcome_system']}",
                 'min_dist_gt_m': round(mind, 4), 'final_dist_gt_m': round(fin, 4)})
        if not A.truthy(r['system_success']):
            continue
        out['nav2_success'] += 1
        s = at_nav2_end(episode_path(r))
        if s is None:
            out['missing_episode'] += 1
            continue
        vs.append(s['v'])
        ws.append(s['w'])
        out['success_moving_v_gt_0.05'] += s['v'] > STRICT_V
        out['success_moving_v_gt_0.20'] += s['v'] > 0.20
        c = classify(r)
        if c:
            out['nav2_success_not_strict'][c].append(
                {'id': sid, 'v_at_success': round(s['v'], 3), 'w_at_success': round(s['w'], 3),
                 'd_gt_at_success': round(s['d'], 4), 'min_dist_gt_m': round(mind, 4),
                 'final_dist_gt_m': round(fin, 4),
                 'final_dist_est_m': round(A.num(r['final_dist_est_m']), 4),
                 'strict_est': A.truthy(r['strict_est'])})
    if vs:
        out['v_at_success_q'] = A.q(vs, (5, 50, 95, 100))
        out['w_at_success_q'] = A.q(ws, (5, 50, 95, 100))
    return out


def main():
    """원자료 → JSON."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--c2', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    res = {'source': os.path.abspath(a.c2), 'strict_xy': STRICT_XY, 'strict_v': STRICT_V,
           'note': '사전 등록 밖 사후 진단 (기술). Nav2 성공 순간 = gt 표본 중 t_end 최근접.',
           'arms': {}}
    for arm in A.ARMS:
        rows = list(A.load_arm(a.c2, arm).values())
        res['arms'][arm] = {}
        for st in A.SETS:
            sel = [r for r in rows if r['scenario_set'].endswith('_' + st)]
            res['arms'][arm][st] = summarize(sel)
    with open(a.out, 'w', encoding='utf-8') as fh:
        json.dump(res, fh, ensure_ascii=False, indent=1)
    for arm, d in res['arms'].items():
        for st, s in d.items():
            ns = ' '.join(f'{k} {len(v)}' for k, v in s['nav2_success_not_strict'].items())
            vq = [round(x, 3) for x in s['v_at_success_q']] if s['v_at_success_q'] else None
            print(f"{arm:5s} {st:7s} n {s['n']} nav2_succ {s['nav2_success']} "
                  f"v>0.05 {s['success_moving_v_gt_0.05']} v>0.20 {s['success_moving_v_gt_0.20']} "
                  f"v_q(5/50/95/100) {vq} | not_strict {ns} "
                  f"| passed_out {len(s['passed_within_0.10_ended_outside_any_outcome'])} "
                  f"missing {s['missing_episode']}")


if __name__ == '__main__':
    main()
