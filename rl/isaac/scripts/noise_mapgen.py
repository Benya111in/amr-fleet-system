"""
지도 변형 짝 열화의 잡음 바닥 — 보충 (사전 등록 외, 리뷰 반영, 결과를 본 뒤 작성).

리뷰 (b1-mapgen, 2026-10-08) 가 짚은 것: 같은 행의 명목 결과 (라운드 1, env j) 와 변형 결과 (라운드 2–4, 다른 env 색인)
는 시작·목표만 같고 평가 잡음 실현이 다르다 — 라운드마다 env.reset(seed) 로 다시 시드하므로 env 색인 i 의 리셋 draw
(시작 yaw U[−0.5, 0.5]) 는 라운드끼리 같지만, 행이 다른 색인에 놓이면 다른 draw 를 받는다. 매 스텝 LiDAR σ 0.03 잡음도
env 색인·배치 구성에 따라 다르다. 그래서 짝 열화의 잡음 바닥은 "먼 사본" 탐침 (색인·N 같음, float32 이동만) 보다 크다.

이 스크립트는 판정을 바꾸지 않는다 (P1map·S1 은 사전 등록 analyze_mapgen.py 그대로). 기술만 한다:
  1. 짝 불일치 수 — 팔·시드·조건 (1 차 모집단) 마다 n10 (명목 성공·변형 실패), n01 (명목 실패·변형 성공), 짝 열화
     (n10 − n01)/n, 변형 행이 명목 행과 같은 env 색인에 놓인 수. 팔마다 5 시드 합과 정확 부호 검정 p (양측) —
     지도 효과가 없다면 (H0) 두 결말은 같은 분포의 독립 draw 라 불일치 방향이 반반이다. 행 수준·이 10 정책 조건부라
     시드 일반화가 아니고, 결과를 본 뒤의 검정이라 확증으로 읽지 않는다.
  2. 리뷰 재실행 대조 — 같은 정책·seed 4242·같은 지도, 배치 구성만 다른 평가 (review/rerun/review-add-{on,off}0:
     명목 main_3_15 + add 만, 4 608 env 한 라운드) 와 본 평가 mg_{on,off}_s0 를 (조건, 집합, 순번) 으로 맞춘다.
     결말이 바뀐 행, 엄격 성공, 보상이 비트 같은 행, 같은 env 색인 비율, 첫 64 env 의 관측 스냅숏 (스텝 0·50·…) 일치.

    python3 rl/isaac/scripts/noise_mapgen.py --out <dir>/noise_mapgen.json \
        --md <dir>/noise_mapgen.md
"""

import argparse
import json
import math
import os

import numpy as np

RUNS = '/home/CAPYI_AI/runs/bonus1/b1-mapgen'
MAIN = 'main_3_15'
SUCCESS = 0


def load(d):
    """평가 디렉터리 → (eval.json, episodes dict, 행 위치 → env 색인)."""
    with open(os.path.join(d, 'eval.json'), encoding='utf-8') as f:
        ev = json.load(f)
    ep = dict(np.load(os.path.join(d, 'episodes.npz')))
    n_envs = int(ev['max_envs'])
    env_idx = np.arange(ep['outcome'].shape[0]) % n_envs
    return ev, ep, env_idx


def sign_test_p(a, b):
    """정확 양측 부호 검정 (불일치 a 대 b, H0 반반)."""
    n = a + b
    if n == 0:
        return 1.0
    k = min(a, b)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2.0 ** n
    return min(1.0, 2.0 * tail)


def nominal_index(ev, ep, env_idx):
    """명목 main 행: 순번 → (결말 성공, env 색인)."""
    si = ev['sets'].index(MAIN)
    m = np.flatnonzero((ep['cond_index'] == -1) & (ep['set_index'] == si) & ep['got'].astype(bool))
    return {int(ep['row_j'][k]): (bool(ep['outcome'][k] == SUCCESS), int(env_idx[k])) for k in m}


def paired_counts(ev, ep, env_idx):
    """조건마다 1 차 모집단 짝 불일치 수."""
    si = ev['sets'].index(MAIN)
    nom = nominal_index(ev, ep, env_idx)
    out = {}
    for ci, cn in enumerate(ev['conditions']):
        m = np.flatnonzero((ep['cond_index'] == ci) & (ep['set_index'] == si)
                           & ep['main_variant'].astype(bool) & ep['got'].astype(bool))
        n10 = n01 = same_env = 0
        for k in m:
            ok_n, e_n = nom[int(ep['row_j'][k])]
            ok_v = bool(ep['outcome'][k] == SUCCESS)
            n10 += int(ok_n and not ok_v)
            n01 += int(ok_v and not ok_n)
            same_env += int(e_n == int(env_idx[k]))
        n = int(m.shape[0])
        out[cn] = {'n': n, 'n10': n10, 'n01': n01, 'same_env': same_env,
                   'degradation_pp': 100.0 * (n10 - n01) / n if n else None}
    return out


def part1(eval_dir):
    """팔·시드·조건 짝 불일치 표."""
    res = {}
    for arm in ('on', 'off'):
        per_seed = {}
        for s in range(5):
            ev, ep, env_idx = load(os.path.join(eval_dir, f'mg_{arm}_s{s}'))
            per_seed[s] = paired_counts(ev, ep, env_idx)
        conds = list(per_seed[0])
        pooled = {}
        for cn in conds:
            n = sum(per_seed[s][cn]['n'] for s in per_seed)
            a = sum(per_seed[s][cn]['n10'] for s in per_seed)
            b = sum(per_seed[s][cn]['n01'] for s in per_seed)
            same = sum(per_seed[s][cn]['same_env'] for s in per_seed)
            degr = [per_seed[s][cn]['degradation_pp'] for s in per_seed]
            pooled[cn] = {'n': n, 'n10': a, 'n01': b, 'same_env': same,
                          'degradation_pp_seed_mean': float(np.mean(degr)),
                          'degradation_pp_seed_min': float(np.min(degr)),
                          'degradation_pp_seed_max': float(np.max(degr)),
                          'sign_test_p': sign_test_p(a, b)}
        res[arm] = {'per_seed': {str(s): v for s, v in per_seed.items()}, 'pooled': pooled}
    return res


def _key_rows(ev, ep, env_idx, set_name, cond_name):
    si = ev['sets'].index(set_name)
    ci = -1 if cond_name is None else ev['conditions'].index(cond_name)
    m = np.flatnonzero((ep['cond_index'] == ci) & (ep['set_index'] == si) & ep['got'].astype(bool))
    return {int(ep['row_j'][k]): int(k) for k in m}


def compare(orig, rerun, set_name, cond_name, primary_only=False):
    """같은 (조건, 집합, 순번) 행의 본 평가 대 재실행."""
    ev_a, ep_a, ix_a = orig
    ev_b, ep_b, ix_b = rerun
    ra = _key_rows(ev_a, ep_a, ix_a, set_name, cond_name)
    rb = _key_rows(ev_b, ep_b, ix_b, set_name, cond_name)
    js = sorted(set(ra) & set(rb))
    if primary_only:
        js = [j for j in js if bool(ep_a['main_variant'][ra[j]])]
    ka = np.array([ra[j] for j in js])
    kb = np.array([rb[j] for j in js])
    sa = ep_a['outcome'][ka] == SUCCESS
    sb = ep_b['outcome'][kb] == SUCCESS
    return {'rows': len(js), 'only_a': len(ra) - len(set(ra) & set(rb)),
            'outcome_changed': int((ep_a['outcome'][ka] != ep_b['outcome'][kb]).sum()),
            'success_to_fail': int((sa & ~sb).sum()), 'fail_to_success': int((~sa & sb).sum()),
            'strict_orig': float(sa.mean()), 'strict_rerun': float(sb.mean()),
            'strict_shift_pp': 100.0 * float(sb.mean() - sa.mean()),
            'return_bit_identical': int((ep_a['return'][ka] == ep_b['return'][kb]).sum()),
            'same_env_index': int((ix_a[ka] == ix_b[kb]).sum())}


def obs_snapshots(da, db):
    """첫 64 env 관측 스냅숏 (스텝 0, 50, …) 의 행별 비트 일치 수."""
    a = np.load(os.path.join(da, 'obs_batch.npz'))['obs']
    b = np.load(os.path.join(db, 'obs_batch.npz'))['obs']
    k = min(a.shape[0], b.shape[0]) // 64
    return [{'step': 50 * i,
             'rows_identical': int((a[64 * i:64 * i + 64] == b[64 * i:64 * i + 64]).all(1).sum()),
             'max_abs_diff': float(np.abs(a[64 * i:64 * i + 64] - b[64 * i:64 * i + 64]).max())}
            for i in range(k)]


def part2(eval_dir, rerun_dir):
    """리뷰 재실행 대조 (켬·끔 시드 0)."""
    res = {}
    for arm in ('on', 'off'):
        da = os.path.join(eval_dir, f'mg_{arm}_s0')
        db = os.path.join(rerun_dir, f'review-add-{arm}0')
        if not os.path.isdir(db):
            continue
        a, b = load(da), load(db)
        res[arm] = {'rerun_dir': db, 'rerun_max_envs': int(b[0]['max_envs']),
                    'rerun_rows': int(b[0]['rows']),
                    'nominal_main': compare(a, b, MAIN, None),
                    'add_main_primary': compare(a, b, MAIN, 'add', primary_only=True),
                    'add_main_all': compare(a, b, MAIN, 'add'),
                    'add_stress': compare(a, b, 'stress_3_15', 'add')}
        res['obs_' + arm + '_orig_vs_rerun'] = obs_snapshots(da, db)
    da, db = (os.path.join(eval_dir, f'mg_{x}_s0') for x in ('on', 'off'))
    res['obs_on_vs_off_orig'] = obs_snapshots(da, db)
    return res


def markdown(r):
    """사람이 읽는 요약."""
    L = ['# 짝 열화 잡음 바닥 — 보충 (사전 등록 외, 리뷰 반영)', '',
         '## 1. 짝 불일치 (1 차 모집단, 5 시드 합)', '',
         '| 조건 | 팔 | n | 명목만 성공 n10 | 변형만 성공 n01 | 같은 env 색인 | 짝 열화 %p 시드 평균 [최소, 최대]'
         ' | 부호 검정 p (보충) |', '|---|---|---|---|---|---|---|---|']
    for cn in r['paired']['on']['pooled']:
        for arm in ('on', 'off'):
            p = r['paired'][arm]['pooled'][cn]
            L.append(f"| {cn} | {arm} | {p['n']} | {p['n10']} | {p['n01']} | {p['same_env']} | "
                     f"{p['degradation_pp_seed_mean']:+.2f} [{p['degradation_pp_seed_min']:+.2f}, "
                     f"{p['degradation_pp_seed_max']:+.2f}] | {p['sign_test_p']:.3g} |")
    L += ['', '## 2. 리뷰 재실행 대조 (같은 정책·seed·지도, 배치 구성만 다름)', '',
          '| 팔 | 행 묶음 | 행 | 결말 바뀜 | 성공→실패 | 실패→성공 | 엄격 본 → 재 | 보상 비트 같음 | 같은 env 색인 |',
          '|---|---|---|---|---|---|---|---|---|']
    for arm in ('on', 'off'):
        if arm not in r['rerun']:
            continue
        for nm in ('nominal_main', 'add_main_primary', 'add_main_all', 'add_stress'):
            c = r['rerun'][arm][nm]
            L.append(f"| {arm} s0 | {nm} | {c['rows']} | {c['outcome_changed']} | "
                     f"{c['success_to_fail']} | {c['fail_to_success']} | "
                     f"{c['strict_orig']:.4f} → {c['strict_rerun']:.4f} "
                     f"({c['strict_shift_pp']:+.2f} %p) | {c['return_bit_identical']} | "
                     f"{c['same_env_index']} |")
    L += ['', '## 3. 첫 64 env 관측 스냅숏 비트 일치 (행 수 / 64)', '']
    for k in [k for k in r['rerun'] if k.startswith('obs_')]:
        snaps = ', '.join(f"스텝 {o['step']} {o['rows_identical']}" for o in r['rerun'][k])
        L.append(f'- {k}: {snaps}')
    return '\n'.join(L) + '\n'


def main():
    """집계."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--eval_dir', default=os.path.join(RUNS, 'eval'))
    ap.add_argument('--rerun_dir', default=os.path.join(RUNS, 'review', 'rerun'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--md', default='')
    a = ap.parse_args()
    r = {'note': '보충 (사전 등록 외, 리뷰 반영) — 판정을 바꾸지 않는다. 부호 검정은 행 수준·10 정책 조건부·결과 뒤.',
         'paired': part1(a.eval_dir), 'rerun': part2(a.eval_dir, a.rerun_dir)}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(r, f, ensure_ascii=False, indent=1)
    md = markdown(r)
    if a.md:
        with open(a.md, 'w', encoding='utf-8') as f:
            f.write(md)
    print(md)


if __name__ == '__main__':
    main()
