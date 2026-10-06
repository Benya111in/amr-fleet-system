"""amr_rl_bench.scenarios 시험 — 커밋한 시나리오 집합 v1 의 규칙과 선택."""

import copy
import os

from amr_rl_bench import scenarios as S
import pytest

from conftest import REPO

V1 = os.path.join(REPO, 'rl', 'eval', 'gazebo', 'scenarios_v1.json')


@pytest.fixture(scope='module')
def v1():
    """커밋한 집합."""
    if not os.path.exists(V1):
        pytest.skip('scenarios_v1.json 없음')
    return S.load(V1)


def test_v1_rules(v1):
    """주 200 · 스트레스 200, 규칙 검사 통과, 실행 순서가 전체를 한 번씩."""
    assert v1['counts'] == {'main': 200, 'stress': 200}
    assert S.check(v1) == []
    assert sorted(v1['exec_order']) == sorted(sc['id'] for sc in v1['scenarios'])
    seeds = [sc['scenario_seed'] for sc in v1['scenarios']]
    assert len(set(seeds)) == len(seeds)


def test_smoke_selection(v1):
    """스모크 = 실행 순서의 주 10 + 스트레스 10 (같은 비중)."""
    sm = S.select(v1, 'smoke')
    assert len(sm) == 20
    assert sum(1 for sc in sm if sc['set'] == 'main') == 10
    order = v1['exec_order']
    assert [sc['id'] for sc in sm] == sorted((sc['id'] for sc in sm), key=order.index)
    assert len(S.select(v1, 'all', start=5, count=7)) == 7
    assert all(sc['set'] == 'stress' for sc in S.select(v1, 'stress'))
    with pytest.raises(KeyError):
        S.select(v1, 'nope_1')


def test_check_catches_broken_sets(v1):
    """주 집합인데 직진 여유가 0.40 미만이거나, 시간 상한이 규칙과 다르면 잡는다."""
    d = copy.deepcopy(v1)
    m = next(sc for sc in d['scenarios'] if sc['set'] == 'main')
    m['goal_straight_m'] = 0.2
    s = next(sc for sc in d['scenarios'] if sc['set'] == 'stress')
    s['time_limit_s'] += 1.0
    probs = S.check(d)
    assert any('직진 여유' in p for p in probs)
    assert any('시간 상한' in p for p in probs)
