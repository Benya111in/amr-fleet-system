"""
지도 변형 사전 등록 분석 (mapgen_stats) 시험 — 가짜 평가 출력으로 판정 규칙·짝 열화·비트 대조·표를 확인한다.

결과를 보기 전에 분석 코드가 끝까지 돈다는 증거이기도 하다 (단계 B 교훈: 분석 스크립트 사전 커밋).
"""

import itertools
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'eval'))

from amr_isaac import mapgen_stats as MS  # noqa: E402

CONDS = ['jitter_0p15', 'jitter_0p30', 'jitter_0p60', 'scale', 'remove', 'add']
SETS = ['main_3_15', 'stress_3_15', 'main_3_30', 'stress_3_30']


def fake_run(p_nom, p_var, seed, n_main=200, n_stress=40, n_other=50, collide=0.0):
    """
    가짜 eval_mapgen 출력 — 명목 4 집합 + 조건 6 × (main 200, stress 40).

    p_var: 조건별 성공 확률 dict. 실패의 collide 비율은 충돌, 나머지는 정체.
    """
    rng = np.random.default_rng(seed)
    cols = {k: [] for k in ('cond_index', 'set_index', 'row_j', 'main_variant', 'bundle_row')}
    outcome = []

    def add(ci, si, n, p, mv=True, brow0=-1):
        for j in range(n):
            ok = rng.random() < p
            outcome.append(0 if ok else (1 if rng.random() < collide else 3))
            cols['cond_index'].append(ci)
            cols['set_index'].append(si)
            cols['row_j'].append(j)
            cols['main_variant'].append(mv if ci >= 0 else si in (0, 2))
            cols['bundle_row'].append(brow0 + j if brow0 >= 0 else -1)
    add(-1, 0, n_main, p_nom)
    add(-1, 1, n_stress, 0.9)
    add(-1, 2, n_other, p_nom)
    add(-1, 3, n_other, 0.9)
    for ci, c in enumerate(CONDS):
        add(ci, 0, n_main, p_var[c], brow0=ci * 1000)
        add(ci, 1, n_stress, 0.8, mv=False, brow0=ci * 1000 + 500)
    n = len(outcome)
    o = np.array(outcome)
    ep = {'outcome': o, 'time_s': rng.uniform(5, 20, n), 'd_end': np.where(o == 0, 0.05, 0.2),
          'nav2_reach': o != 1, 'cost': rng.uniform(0, 1, (n, 5)), 'got': np.ones(n, bool),
          'return': rng.normal(0, 1, n), 'steps': rng.integers(50, 200, n),
          'path_id': np.arange(n), 'tile': np.zeros(n, np.int64), 'pid_orig': np.arange(n)}
    for k, v in cols.items():
        ep[k] = np.array(v)
    ev = {'sets': SETS, 'conditions': CONDS, 'conditions_run': CONDS, 'bundle_sha256': 'x',
          'eval_seed': 4242, 'safety_filter': 'canonical', 'nominal_tile': '0',
          'rows_per_cond': 0, 'zero_policy': False, 'max_envs': 5120, 'unfinished': 0}
    return ev, ep


def ref_of(ep):
    """기존 평가처럼 명목 행만 남긴 episodes (행 메타 열 포함)."""
    m = ep['cond_index'] == -1
    out = {k: ep[k][m] for k in ('outcome', 'time_s', 'd_end', 'nav2_reach', 'cost', 'return',
                                 'steps', 'path_id')}
    out.update({'set_index': ep['set_index'][m], 'cond_index': np.zeros(int(m.sum()), int),
                'goal_group': np.zeros(int(m.sum()), int), 'got': np.ones(int(m.sum()), bool)})
    return out


def arms(p_on, p_off, collide_on=0.0, collide_off=0.0):
    on = [(s,) + fake_run(0.99, {c: p_on for c in CONDS}, 100 + s, collide=collide_on)
          for s in range(5)]
    off = [(s,) + fake_run(0.99, {c: p_off for c in CONDS}, 200 + s, collide=collide_off)
           for s in range(5)]
    return on, off


def test_boot_and_perm_match_reference():
    a = [0.98, 0.97, 0.99, 0.96, 0.985]
    b = [0.95, 0.97, 0.94, 0.96, 0.93]
    rng = np.random.default_rng(0)
    A, B = np.array(a), np.array(b)
    d = A[rng.integers(0, 5, (4000, 5))].mean(1) - B[rng.integers(0, 5, (4000, 5))].mean(1)
    diff, ci = MS.boot_diff(a, b)
    assert diff == pytest.approx(A.mean() - B.mean())
    assert ci == pytest.approx(list(np.percentile(d, [2.5, 97.5])))
    p, tot = MS.perm_p(a, b)
    assert tot == 252
    allv = np.array(a + b)
    obs = abs(A.mean() - B.mean())
    cnt = sum(abs(allv[list(i)].mean() - np.delete(allv, list(i)).mean()) >= obs - 1e-12
              for i in itertools.combinations(range(10), 5))
    assert p == cnt / 252
    assert MS.min_perm_p(5, 5) == pytest.approx(2 / 252)


def test_verdict_rule():
    assert MS.verdict(0.01, [0.001, 0.02], 0.04) == 'up'
    assert MS.verdict(0.01, [-0.001, 0.02], 0.01) == 'undetermined'     # 구간이 0 포함
    assert MS.verdict(0.01, [0.001, 0.02], 0.06) == 'undetermined'      # p > 0.05
    assert MS.verdict(-0.01, [-0.02, -0.001], 0.05) == 'down'
    assert MS.verdict(-0.01, [-0.02, 0.001], 0.01) == 'undetermined'


def test_strong_effect_is_detected_and_holm_possible():
    on, off = arms(1.0, 0.80)
    res = MS.analyze(on, off)
    assert res['P1map']['verdict'] == 'up'
    assert res['P1map']['perm_p'] == pytest.approx(2 / 252)
    assert res['holm_can_reject']                       # 6 조건: 0.0079 ≤ 0.05/6
    assert all(res['S2'][c]['holm_reject'] for c in CONDS)
    # 열화: 켬 ≈ 0 −, 끔 ≈ +0.19 → 효과(끔 − 켬) > 0
    assert res['S1_degradation']['diff_on_minus_off'] > 0.1
    assert res['S1_degradation']['verdict'] == 'up'


def test_no_effect_is_undetermined():
    on, off = arms(0.95, 0.95)
    res = MS.analyze(on, off)
    assert res['P1map']['verdict'] == 'undetermined'
    assert abs(res['P1map']['diff_on_minus_off']) < 0.03


def test_collision_endpoint_direction():
    on, off = arms(0.9, 0.9, collide_on=0.0, collide_off=1.0)
    res = MS.analyze(on, off)
    assert res['S3_collision']['diff_on_minus_off'] < 0
    assert res['S3_collision']['verdict'] == 'down'      # 켬 충돌률이 낮다


def test_primary_population_uses_main_variant_only():
    ev, ep = fake_run(0.99, {c: 1.0 for c in CONDS}, 1)
    # 조건 0 의 main 행 절반을 "변형 좁음" 으로 바꾸고 그 행을 모두 실패로
    m = np.flatnonzero((ep['cond_index'] == 0) & (ep['set_index'] == 0))[:100]
    ep['main_variant'][m] = False
    ep['outcome'][m] = 3
    sm = MS.seed_metrics(ev, ep)
    assert sm['cond']['jitter_0p15']['primary']['strict'] == 1.0
    assert sm['cond']['jitter_0p15']['primary']['n'] == 100
    assert sm['cond']['jitter_0p15']['valid_main']['strict'] == pytest.approx(0.5)


def test_identity_check():
    ev, ep = fake_run(0.99, {c: 0.9 for c in CONDS}, 3)
    ref = ref_of(ep)
    assert MS.identity_check(ep, ref)['pass']
    bad = {k: v.copy() for k, v in ref.items()}
    bad['time_s'][5] += 1e-9
    r = MS.identity_check(ep, bad)
    assert not r['pass'] and r['mismatch'] == ['time_s']
    on, off = arms(0.95, 0.94)
    res = MS.analyze(on, off, {s: ref_of(e) for s, _v, e in on},
                     {s: ref_of(e) for s, _v, e in off})
    assert res['identity_all_pass']


def test_near_added_subgroup_and_markdown():
    on, off = arms(0.95, 0.94)
    nb = 6000
    bundle = {'row_near_added': np.where(np.arange(nb) % 2 == 0, 0.2, 5.0)}
    res = MS.analyze(on, off, bundle=bundle)
    assert 'add|near_added' in res['descriptive']
    md = MS.markdown(res)
    assert 'P1map' in md and 'jitter_0p60' in md


def test_mixed_eval_signature_rejected():
    on, off = arms(0.95, 0.94)
    off[0][1]['bundle_sha256'] = 'y'
    with pytest.raises(ValueError):
        MS.analyze(on, off)
