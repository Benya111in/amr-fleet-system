#!/usr/bin/env python3
"""
B5 보충 (사전 등록 외 — 결과를 본 뒤에 만든 진단이다).

1. 스트레스 성공을 목표 직진 여유 c (Isaac 사각형 기하로 잰 끝 접선 자세의 풋프린트 앞 끝 직진 여유) 구간별로:
   2D DWA · 2D RL (10 시드 평균, 시드 7) · Isaac RL (같은 정책·같은 행) · Gazebo DWA · RL
   (시나리오 v1, 계약 결말 'success').
2. 2D DWA 의 목표 앞 정체 거리와 "STOP 거리 0.30 − c" 의 관계 (safety_lite 가 D ≤ 0.30 에서 멈춘다는 기제 확인).
3. 실패 꼴별 행 목록 (출발 교착·중간 정체·충돌) 과 거리 분포.
출력: --out 아래 supp.json, supp.md.
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RL = os.path.dirname(os.path.dirname(HERE))
REPO = os.path.dirname(RL)
SEEDS = range(10)
BINS = (0.0, 0.10, 0.20, 0.30, 0.40)


def load(p):
    with np.load(p, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def binned(c, ok):
    out = []
    for lo, hi in zip(BINS[:-1], BINS[1:]):
        m = (c >= lo) & (c < hi)
        out.append({'bin': f'[{lo:.2f}, {hi:.2f})', 'n': int(m.sum()),
                    'success': float(ok[m].mean()) if m.any() else None})
    return out


def _fmt(b):
    return '—' if b['success'] is None else f'{b["success"]:.3f}'


def gazebo(gz_dir, scen_json):
    """Gazebo C2 묶음 원자료 (episodes_v1.csv) — 팔은 묶음 디렉터리 이름으로 (dwa_b* · rl_b*, 인계 rlho_b* 는 뺀다)."""
    with open(scen_json) as f:
        sc = {s['id']: s for s in json.load(f)['scenarios']}
    rows = {}
    pat = os.path.join(gz_dir, 'c2', 'batches', '*', 'a*', 'episodes_v1.csv')
    for p in sorted(glob.glob(pat)):
        batch = os.path.basename(os.path.dirname(os.path.dirname(p)))
        arm = batch.split('_b')[0]
        if arm not in ('dwa', 'rl'):
            continue
        with open(p) as f:
            for r in csv.DictReader(f):
                rows[(arm, r['scenario_id'])] = r      # 같은 시나리오 재시도는 마지막 시도
    out = {}
    for arm in ('dwa', 'rl'):
        ids = [k[1] for k in rows if k[0] == arm and sc[k[1]]['set'] == 'stress']
        c = np.array([sc[i]['goal_straight_m'] if sc[i]['goal_straight_m'] is not None
                      else np.inf for i in ids], dtype=float)
        ok = np.array([rows[(arm, i)]['outcome'] == 'success' for i in ids])
        out[arm] = {'n': len(ids), 'success': int(ok.sum()), 'binned': binned(c, ok)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--isaac', required=True)
    ap.add_argument('--pool', required=True)
    ap.add_argument('--geom', required=True)
    ap.add_argument('--gazebo', default='/home/CAPYI_AI/runs/bonus1/b1-gzc')
    ap.add_argument('--tag', default='W1')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    sys.path.insert(0, os.path.join(RL, 'isaac'))
    from amr_isaac import eval_sets as ES
    os.makedirs(a.out, exist_ok=True)
    dwa = load(os.path.join(a.runs, f'B5_dwa_{a.tag}', 'episodes.npz'))
    rl = {s: load(os.path.join(a.runs, f'B5_rl_s{s}_{a.tag}', 'episodes.npz')) for s in SEEDS}
    isa = {s: load(os.path.join(a.isaac, f'B1e_ppo10_s{s}_final_sets', 'episodes.npz'))
           for s in SEEDS}
    pool = load(a.pool)
    obb = load(a.geom)['obb']
    poses = np.stack([ES.end_tangent_pose(pool['pts'][pool['offs'][p]:pool['offs'][p + 1]])
                      for p in dwa['pid']])
    clr = ES.straight_approach_batch(poses, obb)
    res = {'tag': a.tag, 'note': '보충 (사전 등록 외)', 'stress': {}, 'main': {}}
    md = [f'# B5 보충 ({a.tag}) — 사전 등록 외', '']
    for st in ('stress_3_15', 'stress_3_30'):
        m = dwa['set'] == st
        c = clr[m]
        blk = {'dwa': binned(c, dwa['outcome'][m] == 0),
               'rl_mean': None, 'rl_seed7': binned(c, rl[7]['outcome'][m] == 0),
               'isaac_seed7': binned(c, isa[7]['outcome'][m] == 0)}
        rlm = np.mean([rl[s]['outcome'][m] == 0 for s in SEEDS], axis=0)
        ism = np.mean([isa[s]['outcome'][m] == 0 for s in SEEDS], axis=0)
        blk['rl_mean'] = binned(c, rlm)
        blk['isaac_mean'] = binned(c, ism)
        # 목표 앞 정체 거리 대 0.30 − c
        ns = m & (dwa['outcome'] == 2) & (dwa['goal_min'] <= 0.25)
        cc = clr[ns]
        d_end = dwa['d_goal_end'][ns]
        fin = np.isfinite(cc)
        pred = 0.30 - cc[fin]
        blk['dwa_near_goal_stall'] = {
            'n': int(ns.sum()), 'd_end_q': [float(np.quantile(d_end, q)) for q in (0.1, 0.5, 0.9)],
            'corr_d_end_vs_0.30_minus_c': float(np.corrcoef(d_end[fin], pred)[0, 1])
            if fin.sum() > 2 else None,
            'median_abs_err_vs_0.30_minus_c': float(np.median(np.abs(d_end[fin] - pred)))
            if fin.any() else None,
            'stop_zone_time_s_median': float(np.median(dwa['stop_sub'][ns] * 0.02))}
        res['stress'][st] = blk
        md.append(f'## {st} 성공률 — 목표 직진 여유 c 구간별')
        md.append('| c 구간 | n | 2D DWA | 2D RL 10 시드 | 2D RL 시드 7 | Isaac RL 10 시드 | '
                  'Isaac RL 시드 7 |')
        md.append('|---|---|---|---|---|---|---|')
        for i in range(len(BINS) - 1):
            def f(x):
                return '—' if x[i]['success'] is None else f'{x[i]["success"]:.3f}'
            md.append(f'| {blk["dwa"][i]["bin"]} | {blk["dwa"][i]["n"]} | {f(blk["dwa"])} | '
                      f'{f(blk["rl_mean"])} | {f(blk["rl_seed7"])} | {f(blk["isaac_mean"])} | '
                      f'{f(blk["isaac_seed7"])} |')
        s = blk['dwa_near_goal_stall']
        md.append(f'\n2D DWA 목표 앞 정체 {s["n"]} 건: 끝 거리 10/50/90 % {s["d_end_q"]}, '
                  f'0.30 − c 와 상관 {s["corr_d_end_vs_0.30_minus_c"]}, 중앙 절대 오차 '
                  f'{s["median_abs_err_vs_0.30_minus_c"]}, '
                  f'STOP 존 시간 중앙 {s["stop_zone_time_s_median"]} s\n')
    # Gazebo
    scen = os.path.join(REPO, 'rl', 'eval', 'gazebo', 'scenarios_v1.json')
    if os.path.isdir(a.gazebo) and os.path.exists(scen):
        g = gazebo(a.gazebo, scen)
        res['gazebo_stress'] = g
        md.append('## Gazebo 스트레스 (시나리오 v1, episodes_v1.csv 의 계약 결말 success) — c 구간별')
        md.append('| c 구간 | n | Gazebo DWA | Gazebo RL 시드 7 |')
        md.append('|---|---|---|---|')
        for i in range(len(BINS) - 1):
            d_, r_ = g['dwa']['binned'][i], g['rl']['binned'][i]
            md.append(f'| {d_["bin"]} | {d_["n"]} | {_fmt(d_)} | {_fmt(r_)} |')
        gd, gr = g['dwa'], g['rl']
        md.append(f'\n합: DWA {gd["success"]}/{gd["n"]}, RL {gr["success"]}/{gr["n"]}\n')
    # 주 집합 실패 행
    for st in ('main_3_15', 'main_3_30'):
        m = dwa['set'] == st
        f_ = m & (dwa['outcome'] != 0)
        prog = dwa['d_start'] - dwa['goal_min']
        rows = []
        for i in np.nonzero(f_)[0]:
            rows.append({'pid': int(dwa['pid'][i]), 'outcome': int(dwa['outcome'][i]),
                         'progress_m': round(float(prog[i]), 3),
                         'goal_min': round(float(dwa['goal_min'][i]), 3),
                         'd_start': round(float(dwa['d_start'][i]), 3),
                         'clear_c': round(float(clr[i]), 3) if np.isfinite(clr[i]) else None,
                         'stop_zone_s': round(float(dwa['stop_sub'][i] * 0.02), 2),
                         'rl_success_10': int(sum(rl[s]['outcome'][i] == 0 for s in SEEDS))})
        rlf = []
        for s in SEEDS:
            fm = m & (rl[s]['outcome'] != 0)
            for i in np.nonzero(fm)[0]:
                rlf.append({'seed': s, 'pid': int(rl[s]['pid'][i]),
                            'outcome': int(rl[s]['outcome'][i]),
                            'goal_min': round(float(rl[s]['goal_min'][i]), 3),
                            'progress_m': round(float(rl[s]['d_start'][i] -
                                                      rl[s]['goal_min'][i]), 3),
                            'dwa_success': bool(dwa['outcome'][i] == 0)})
        gm = np.array([r['goal_min'] for r in rlf]) if rlf else np.zeros(0)
        res['main'][st] = {'dwa_fail_rows': rows, 'rl_fail_n': len(rlf),
                           'rl_fail_goal_min_q': ([float(np.quantile(gm, q))
                                                   for q in (0.1, 0.5, 0.9)]
                                                  if len(gm) else None),
                           'rl_fail_distinct_pids': int(len({r['pid'] for r in rlf})),
                           'rl_fail_rows': rlf}
    with open(os.path.join(a.out, 'supp.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    with open(os.path.join(a.out, 'supp.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
