"""Golden 벡터: 개수, 재생성 일치, JSON 에서 다시 읽어 재계산한 값과의 일치 (r1·r2·r3 시험의 견본)."""

import glob
import json
import math
import os

import numpy as np
import pytest

import gen_golden
from amr_rl_contract import constants as C
from amr_rl_contract import kinematics as K
from amr_rl_contract import mask as M
from amr_rl_contract import observation as OB
from amr_rl_contract import profiler as P
from amr_rl_contract import reward as R
from amr_rl_contract import safety as S
from amr_rl_contract.state import State

GOLDEN = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'golden')
FILES = sorted(glob.glob(os.path.join(GOLDEN, '*.json')))


def _load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _inf(v):
    return math.inf if v is None else v


def test_golden_count_and_kinds():
    assert len(FILES) >= 20
    kinds = {_load(f)['kind'] for f in FILES}
    assert kinds == {'decision', 'step', 'integrate', 'profiler', 'safety'}
    for f in FILES:
        d = _load(f)
        assert d['contract_version'] == C.CONTRACT_VERSION
        assert os.path.basename(f) == d['name'] + '.json'


def test_golden_regenerates_identically(tmp_path):
    names = gen_golden.write_all(str(tmp_path))
    assert sorted(n + '.json' for n in names) == sorted(os.path.basename(f) for f in FILES)
    for n in names:
        with open(tmp_path / (n + '.json'), encoding='utf-8') as f:
            new = f.read()
        with open(os.path.join(GOLDEN, n + '.json'), encoding='utf-8') as f:
            old = f.read()
        assert new == old, n


def test_golden_json_is_standard():
    for f in FILES:
        with open(f, encoding='utf-8') as fh:
            txt = fh.read()
        assert 'Infinity' not in txt and 'NaN' not in txt, f


@pytest.mark.parametrize('path', [f for f in FILES if _load(f)['kind'] == 'decision'],
                         ids=os.path.basename)
def test_decision_replay(path):
    d = _load(path)
    st = State.from_dict(d['state'])
    e = d['expected']
    tol = d['tolerance']
    obs = OB.build_observation(st)
    assert np.allclose(obs, np.array(e['obs'], dtype=np.float64), atol=tol['obs'], rtol=0)
    res = M.compute_mask(st)
    assert [bool(v) for v in res['mask']] == e['mask']
    assert [bool(v) for v in res['fp_ok']] == e['fp_ok']
    assert [bool(v) for v in res['jerk_ok']] == e['jerk_ok']
    assert [bool(v) for v in res['speed_ok']] == e['speed_ok']
    assert res['saturated'] == e['saturated']
    for a, b in zip(res['vo_time'], e['vo_time']):
        assert (math.isinf(a) and b is None) or abs(a - b) <= tol['vo_time']
    assert np.allclose(res['fallback'], e['fallback'], atol=tol['fallback'])


@pytest.mark.parametrize('path', [f for f in FILES if _load(f)['kind'] == 'step'],
                         ids=os.path.basename)
def test_step_replay(path):
    d = _load(path)
    e = d['expected']
    tol = d['tolerance']
    pairs = [(tuple(a), tuple(b)) for a, b in d['safety_pairs']]
    r = R.step(State.from_dict(d['pre']), State.from_dict(d['post']), d['action_exec'],
               d['collision'], pairs)
    assert r.reward == pytest.approx(e['reward'], abs=tol['reward'])
    for k, v in e['terms'].items():
        assert r.terms[k] == pytest.approx(v, abs=tol['reward_terms'])
    assert np.allclose(r.cost, e['cost'], atol=tol['cost'])
    assert (r.terminated, r.success) == (e['terminated'], e['success'])
    assert r.dev.in_dev == e['dev']['in_dev']
    assert r.dev.t_since_peak == pytest.approx(e['dev']['t_since_peak'], abs=1e-9)


def test_integrate_profiler_safety_replay():
    for f in FILES:
        d = _load(f)
        e = d['expected']
        if d['kind'] == 'integrate':
            for row in e['b']:
                out = K.integrate_b(row['u'], row['v_cmd'], row['w_cmd'], row['v_cap'])
                assert np.allclose(out, row['out'], atol=1e-9)
            for row in e['c']:
                assert np.allclose(K.integrate_c(row['index'], row['v_cmd'], row['w_cmd']),
                                   row['out'], atol=1e-9)
            for row in e['nearest_cell']:
                assert K.nearest_cell(*row['a']) == row['cell']
        elif d['kind'] == 'profiler':
            prof = P.ProfilerReference()
            for stp in e['steps']:
                subs = P.run_decision_period(prof, *stp['cmd_from'], *stp['cmd_to'])
                assert np.allclose(subs, stp['ref'], atol=d['tolerance']['profiler'])
        elif d['kind'] == 'safety':
            for row in e['rows']:
                i = row['in']
                out = S.safety_filter(i['v'], i['w'], i['d_main'], i['min_ttc'],
                                      i['proximity_stop'])
                assert out['v'] == pytest.approx(row['out']['v'], abs=1e-9)
                assert out['w'] == pytest.approx(row['out']['w'], abs=1e-9)
                assert out['zone'] == row['out']['zone']
                assert _inf(row['out']['speed_limit']) == pytest.approx(out['speed_limit'])
