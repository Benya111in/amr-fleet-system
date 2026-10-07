"""
시드 집계 스크립트 시험 (scripts/aggregate_seeds.py, 합성 eval.json — Isaac 불필요).

- 시드별 값·평균·실패 시드 수(문턱 0.90)·Wilson 구간·IQM, 두 팔의 평균 차
- 교란 평가면 한 축 unseen 조건 평균(unseen_single_axis) — combo·edge·nominal 은 빠진다
- 평가 조건이 섞이면(평가 seed 다름) 거부
"""

import importlib.util
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(os.path.dirname(HERE), 'scripts', 'aggregate_seeds.py')


def _doc(strict_by_key, seed=4242, conds=('nominal',), groups=None, params=None):
    by = {}
    for k, s in strict_by_key.items():
        cond = k.split('|')[1] if '|' in k else 'nominal'
        by[k] = {'episodes': 1024, 'condition_group': (groups or {}).get(cond, 'nominal'),
                 'condition_params': (params or {}).get(cond, {}),
                 'layered': {'strict_success': s, 'nav2_compatible_success': min(1.0, s + 0.01),
                             'intervention_per_ep': 0.5, 'n': 1024},
                 'intervals': {'collisions': 0}, 'stall': 1 - s}
    return {'mode': 'eval_sets', 'pool_digest': 'abc', 'eval_seed': seed,
            'sets': ['main_3_15'], 'conditions': list(conds), 'safety_filter': 'canonical',
            'by': by}


def _write(tmp, name, docs):
    d = tmp / name
    for i, doc in enumerate(docs):
        p = d / f's{i}'
        p.mkdir(parents=True)
        (p / 'eval.json').write_text(json.dumps(doc))
    return str(d / '*' / 'eval.json')


def _run(args):
    r = subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True)
    return r.returncode, r.stdout, r.stderr


@pytest.mark.skipif(importlib.util.find_spec('scipy') is None, reason='scipy 없음')
def test_two_arms_seed_table_and_diff(tmp_path):
    """팔 둘 × 시드 5: 실패 시드 1 개(0.80), 평균 차, IQM."""
    a = _write(tmp_path, 'a', [_doc({'main_3_15': s}) for s in (0.99, 0.98, 0.80, 0.97, 0.99)])
    b = _write(tmp_path, 'b', [_doc({'main_3_15': s}) for s in (0.95, 0.96, 0.94, 0.95, 0.97)])
    out = tmp_path / 'agg.json'
    rc, so, se = _run(['--arm', f'dr={a}', '--arm', f'nodr={b}', '--out', str(out),
                       '--reps', '500'])
    assert rc == 0, se
    res = json.loads(out.read_text())
    s = res['by']['main_3_15']['dr']
    assert s['seeds'] == 5 and s['failed_seeds'] == 1
    assert s['mean'] == pytest.approx((0.99 + 0.98 + 0.80 + 0.97 + 0.99) / 5)
    assert s['failed_seed_wilson95'][0] < 0.2 < s['failed_seed_wilson95'][1]
    assert s['iqm'] == pytest.approx(0.98, abs=1e-9)          # 25 % 절사: 0.80·0.99 하나씩 빠짐
    d = res['by']['main_3_15']['diff']
    assert d['arms'] == ['dr', 'nodr']
    assert d['diff_mean'] == pytest.approx(s['mean'] - 0.954)
    assert d['boot95'][0] <= d['diff_mean'] <= d['boot95'][1]
    assert '실패 시드' in so


def test_unseen_single_axis_mean(tmp_path):
    """한 축만 바꾼 unseen 조건만 평균에 들어간다."""
    groups = {'nominal': 'nominal', 'delay': 'unseen', 'gain': 'unseen', 'combo': 'unseen',
              'edge': 'edge'}
    params = {'delay': {'plant_delay_s': 0.12, 'drive_gain': 1.0},
              'gain': {'drive_gain': 0.8}, 'combo': {'plant_delay_s': 0.12, 'drive_gain': 0.9},
              'edge': {'lidar_sigma_m': 0.06}}
    conds = tuple(groups)
    d = _doc({'perturb_3_15|nominal': 0.99, 'perturb_3_15|delay': 0.90,
              'perturb_3_15|gain': 0.80, 'perturb_3_15|combo': 0.10,
              'perturb_3_15|edge': 0.98}, conds=conds, groups=groups, params=params)
    a = _write(tmp_path, 'a', [d, d])
    out = tmp_path / 'agg.json'
    rc, _, se = _run(['--arm', f'x={a}', '--out', str(out), '--reps', '200'])
    assert rc == 0, se
    res = json.loads(out.read_text())
    assert res['by']['unseen_single_axis']['x']['mean'] == pytest.approx(0.85)


def test_mixed_eval_signature_rejected(tmp_path):
    """평가 seed 가 다른 eval.json 을 섞으면 rc 1."""
    a = _write(tmp_path, 'a', [_doc({'main_3_15': 0.9}), _doc({'main_3_15': 0.9}, seed=1)])
    rc, _, se = _run(['--arm', f'x={a}', '--out', str(tmp_path / 'o.json')])
    assert rc == 1 and '섞였다' in se
