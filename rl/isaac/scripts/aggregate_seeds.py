#!/usr/bin/env python3
"""
시드 집계 — 집합 평가(eval_unbiased.py --eval_set) 의 eval.json 여러 개 → 시드별 표·실패 시드 비율·IQM·부트스트랩.

    docker run --rm -v $WT:/ws -v $R:$R -w /ws amr-ai/rl:0.3 \
        python3 rl/isaac/scripts/aggregate_seeds.py \
        --arm 'ppo=/home/CAPYI_AI/runs/bonus1/b1-ppo10-eval/*/sets/eval.json' \
        --out $R/analysis/agg.json --md $R/analysis/agg.md

팔(arm)마다 eval.json 하나 = 시드 하나 (같은 집합 파일·같은 평가 seed 여야 한다 — 다르면 rc 1). 키 = eval.json 의 `by` 키
(집합, 또는 교란 평가면 '집합|조건'). 키마다:
  - 시드별 엄격 성공·Nav2 호환·충돌·개입/에피소드·n
  - 실패 시드 = 엄격 성공 < --fail_below (기본 0.90, 완성 계획의 통과 문턱) 의 수와 Wilson 95 % 구간 (amr_rl_eval.binomial)
  - 평균·IQM 과 시드 부트스트랩 95 % 구간 (amr_rl_eval.aggregate.stratified_bootstrap — 과제 하나 = 그 키)
교란 평가면 시드마다 'unseen_single_axis' = 한 축만 바꾼 unseen 조건들의 엄격 성공 평균 (같은 무게) 을 더한다.
팔이 둘이면 키마다 평균 차 (첫 팔 − 둘째 팔) 와 팔별 시드 복원 추출 부트스트랩 95 % 구간.
4–5 시드의 IQM 은 실패 시드를 잘라 버릴 수 있으므로 평균·실패 시드 비율과 함께만 쓴다 (후속 조사 보고서 「제약 RL」).
"""

import argparse
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'eval'))
sys.path.insert(0, os.path.dirname(HERE))

from amr_rl_eval import aggregate as AG  # noqa: E402
from amr_rl_eval import binomial as BN  # noqa: E402

from amr_isaac import perturb as PT  # noqa: E402

Z95 = 1.959963984540054


def seed_rows(doc):
    """eval.json → {키: 지표 dict} (+ 교란이면 unseen_single_axis)."""
    out = {}
    single = []
    for k, b in doc['by'].items():
        if not b.get('episodes'):
            continue
        lay = b['layered']
        out[k] = {'strict': lay['strict_success'], 'nav2': lay['nav2_compatible_success'],
                  'collisions': b['intervals']['collisions'] if 'intervals' in b else None,
                  'intervention_per_ep': lay['intervention_per_ep'], 'n': lay['n'],
                  'stall': b.get('stall'), 'time_s_mean_success': b.get('time_s_mean_success')}
        params = b.get('condition_params') or {}
        diff = [a for a in PT.KEYS if a in params and abs(params[a] - PT.NOMINAL[a]) > 1e-12]
        if b.get('condition_group') == 'unseen' and len(diff) == 1:
            single.append(lay['strict_success'])
    if single:
        out['unseen_single_axis'] = {'strict': float(np.mean(single)), 'n': len(single),
                                     'nav2': None, 'collisions': None,
                                     'intervention_per_ep': None, 'stall': None,
                                     'time_s_mean_success': None}
    return out


def summarize_arm(seeds, key, fail_below, reps, seed):
    """한 팔·한 키의 시드 집계."""
    v = np.array([s[key]['strict'] for s in seeds if key in s], dtype=np.float64)
    if v.size == 0:
        return None
    fails = int((v < fail_below).sum())
    lo, hi = BN.wilson_interval(fails, v.size, Z95)
    est, blo, bhi = AG.stratified_bootstrap(v[:, None], func=AG.mean, reps=reps, seed=seed)
    iq, ilo, ihi = AG.stratified_bootstrap(v[:, None], func=AG.iqm, reps=reps, seed=seed)
    return {'seeds': int(v.size), 'per_seed_strict': v.round(5).tolist(),
            'per_seed_nav2': [s[key]['nav2'] for s in seeds if key in s],
            'per_seed_collisions': [s[key]['collisions'] for s in seeds if key in s],
            'per_seed_intervention': [s[key]['intervention_per_ep'] for s in seeds if key in s],
            'mean': est, 'mean_boot95': [blo, bhi], 'iqm': iq, 'iqm_boot95': [ilo, ihi],
            'fail_below': fail_below, 'failed_seeds': fails, 'failed_seed_wilson95': [lo, hi]}


def diff_boot(a, b, reps, seed):
    """평균 차 a − b 와 팔별 시드 복원 추출 부트스트랩 95 % 구간."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    rng = np.random.default_rng(seed)
    d = a[rng.integers(0, a.size, (reps, a.size))].mean(1) - \
        b[rng.integers(0, b.size, (reps, b.size))].mean(1)
    lo, hi = np.percentile(d, [2.5, 97.5])
    return {'diff_mean': float(a.mean() - b.mean()), 'boot95': [float(lo), float(hi)]}


def main():
    """집계."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', action='append', required=True, help="'이름=glob' (eval.json)")
    ap.add_argument('--fail_below', type=float, default=0.90)
    ap.add_argument('--reps', type=int, default=4000)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--md', default='')
    a = ap.parse_args()
    arms, files, sig = {}, {}, set()
    for spec in a.arm:
        name, pat = spec.split('=', 1)
        fs = sorted(glob.glob(pat))
        if not fs:
            print(f'{name}: 파일 없음 {pat}', file=sys.stderr)
            return 1
        docs = [json.load(open(f, encoding='utf-8')) for f in fs]
        for d in docs:
            if d.get('mode') != 'eval_sets':
                print(f'집합 평가가 아니다: {d.get("policy")}', file=sys.stderr)
                return 1
            sig.add((d['pool_digest'], d['eval_seed'], tuple(d['sets']),
                     tuple(d['conditions']), d['safety_filter']))
        arms[name] = [seed_rows(d) for d in docs]
        files[name] = fs
    if len(sig) != 1:
        print(f'평가 조건이 섞였다: {sig}', file=sys.stderr)
        return 1
    keys = sorted({k for rows in arms.values() for r in rows for k in r})
    res = {'arms': {n: {'files': files[n]} for n in arms}, 'eval_signature': list(sig)[0],
           'by': {}}
    for k in keys:
        res['by'][k] = {n: summarize_arm(rows, k, a.fail_below, a.reps, a.seed)
                        for n, rows in arms.items()}
        if len(arms) == 2:
            n1, n2 = list(arms)
            s1, s2 = res['by'][k][n1], res['by'][k][n2]
            if s1 and s2:
                res['by'][k]['diff'] = {'arms': [n1, n2], **diff_boot(
                    s1['per_seed_strict'], s2['per_seed_strict'], a.reps, a.seed)}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    lines = ['| 키 | 팔 | 시드 | 시드별 엄격 성공 | 평균 [부트 95 %] | IQM [부트 95 %] '
             '| 실패 시드 (Wilson 95 %) |', '|---|---|---|---|---|---|---|']
    for k, by in res['by'].items():
        for n in arms:
            s = by.get(n)
            if not s:
                continue
            per = ' · '.join(f'{x:.4f}' for x in s['per_seed_strict'])
            lines.append(
                f"| {k} | {n} | {s['seeds']} | {per}"
                f" | {s['mean']:.4f} [{s['mean_boot95'][0]:.4f}, {s['mean_boot95'][1]:.4f}]"
                f" | {s['iqm']:.4f} [{s['iqm_boot95'][0]:.4f}, {s['iqm_boot95'][1]:.4f}]"
                f" | {s['failed_seeds']}/{s['seeds']} ({s['failed_seed_wilson95'][0]:.3f}–"
                f"{s['failed_seed_wilson95'][1]:.3f}) |")
        if 'diff' in by:
            d = by['diff']
            lines.append(f"| {k} | {d['arms'][0]} − {d['arms'][1]} | | | "
                         f"{d['diff_mean']:+.4f} [{d['boot95'][0]:+.4f}, "
                         f"{d['boot95'][1]:+.4f}] | | |")
    md = '\n'.join(lines) + '\n'
    if a.md:
        with open(a.md, 'w', encoding='utf-8') as f:
            f.write(md)
    print(md)
    return 0 if all(not math.isnan(v['mean']) for by in res['by'].values()
                    for n, v in by.items() if n in arms and v) else 1


if __name__ == '__main__':
    sys.exit(main())
