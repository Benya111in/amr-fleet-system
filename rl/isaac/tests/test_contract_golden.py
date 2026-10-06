"""
계약 일치 시험 — Isaac env 의 torch 구현(amr_isaac.contract_torch)을 golden 32 개(계약 v1.2.0)와 대조한다.

견본은 rl/contract/tests/test_golden.py 의 test_decision_replay·test_step_replay 다. 여기서는 기준 구현을
다시 부르지 않고 **golden JSON 의 기대값**과 직접 비교한다 (golden > 기준 구현 > 문서).
비교군 C 의 마스크는 이 env(비교군 B)에 구현하지 않았다 — 관측만 대조한다 (REPORT 참고).
GPU 가 있으면 float32 관측을 CUDA 에서도 한 번 더 확인한다 (env 가 실제로 도는 장치).
step 사례는 두 길로 본다: reward_step 이 pre 자세로 d(s) 를 계산하는 길, env 처럼 observe_with_cache 로
전이 전 거리를 캐시해 d_pre 로 넘기는 길 (env 는 step_outcome 시점에 자세가 이미 전이 뒤다).
"""

import glob
import json
import math
import os

import pytest
import torch

from amr_isaac import contract_torch as CT
from amr_rl_contract import constants as C
from amr_rl_contract.observation import obs_slices

GOLDEN = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'contract', 'golden')
FILES = sorted(glob.glob(os.path.join(GOLDEN, '*.json')))
DEVICES = ['cpu'] + (['cuda'] if torch.cuda.is_available() else [])


def _load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _kind(kind):
    return [f for f in FILES if _load(f)['kind'] == kind]


def _inf(v):
    return math.inf if v is None else float(v)


def test_golden_version_and_count():
    assert C.CONTRACT_VERSION == '1.2.0'
    assert len(FILES) == 32
    assert len(_kind('step')) == 12 and len(_kind('decision')) == 17
    for f in FILES:
        assert _load(f)['contract_version'] == '1.2.0'


def _step_inputs(d):
    pre = CT.states_from_dicts([d['pre']])
    post = CT.states_from_dicts([d['post']])
    pairs = None
    if d['safety_pairs']:
        pairs = torch.tensor([[a[0], a[1], b[0], b[1]] for a, b in d['safety_pairs']],
                             dtype=torch.float64).unsqueeze(0)
    return (pre, post.pose, post.v_meas, torch.tensor([d['action_exec']], dtype=torch.float64),
            torch.tensor([bool(d['collision'])]), pairs)


def _check_step(out, d):
    e = d['expected']
    tol = d['tolerance']
    assert float(out.reward[0]) == pytest.approx(e['reward'], abs=tol['reward'])
    # 보상 항 집합이 golden 과 같다 (v1.2.0: progress·dev 항이 빠지거나 남는 것을 잡는다)
    assert set(out.terms) == set(e['terms'])
    for k, v in e['terms'].items():
        assert float(out.terms[k][0]) == pytest.approx(v, abs=tol['reward_terms']), k
    assert torch.allclose(out.cost[0], torch.tensor(e['cost'], dtype=torch.float64),
                          atol=tol['cost'])
    assert (bool(out.terminated[0]), bool(out.success[0])) == (e['terminated'], e['success'])
    assert bool(out.in_dev[0]) == e['dev']['in_dev']
    assert float(out.dev_peak[0]) == pytest.approx(e['dev']['peak'], abs=1e-9)
    assert float(out.dev_t[0]) == pytest.approx(e['dev']['t_since_peak'], abs=1e-9)
    assert float(out.e[0]) == pytest.approx(e['e'], abs=tol['e'])
    assert float(out.phi_pre[0]) == pytest.approx(e['phi_pre'], abs=tol['reward'])
    assert float(out.phi_post[0]) == pytest.approx(e['phi_post'], abs=tol['reward'])


@pytest.mark.parametrize('device', DEVICES)
@pytest.mark.parametrize('path', _kind('decision'), ids=os.path.basename)
def test_decision_observation(path, device):
    d = _load(path)
    st = CT.states_from_dicts([d['state']], device=device)
    obs, parts = CT.build_observation(st, return_parts=True)
    exp = torch.tensor(d['expected']['obs'], dtype=torch.float64)
    got = obs[0].double().cpu()
    tol = d['tolerance']['obs']
    err = (got - exp).abs()
    if err.max() > tol:
        bad = {name: float(err[sl].max()) for name, sl in obs_slices().items()
               if float(err[sl].max()) > tol}
        pytest.fail(f'관측 불일치 블록 {bad}')


def test_decision_observation_batched():
    """17 사례를 한 배치로 — 패딩 경로·트랙 수가 섞여도 같은 값."""
    ds = [_load(p) for p in _kind('decision')]
    st = CT.states_from_dicts([d['state'] for d in ds])
    obs = CT.build_observation(st)
    for i, d in enumerate(ds):
        exp = torch.tensor(d['expected']['obs'], dtype=torch.float64)
        assert (obs[i].double() - exp).abs().max() <= d['tolerance']['obs'], d['name']


@pytest.mark.parametrize('path', _kind('step'), ids=os.path.basename)
def test_step_reward_cost(path):
    d = _load(path)
    _check_step(CT.reward_step(*_step_inputs(d)), d)


@pytest.mark.parametrize('path', _kind('step'), ids=os.path.basename)
def test_step_reward_cost_env_cache(path):
    """환경의 길: 관측 시점(pre 자세)에 observe_with_cache → 전이 뒤 reward_step(d_pre=캐시)."""
    d = _load(path)
    pre, post_pose, post_v, a, col, pairs = _step_inputs(d)
    _obs, d_pre = CT.observe_with_cache(pre)
    # env 처럼 버퍼의 자세를 전이 뒤로 바꾼 상태를 pre 로 넘긴다 — pre.pose 를 쓰면 틀린 값이 난다
    pre.pose = post_pose.clone()
    _check_step(CT.reward_step(pre, post_pose, post_v, a, col, pairs, d_pre=d_pre), d)


def test_integrate_b_rows():
    d = _load(_kind('integrate')[0])
    rows = d['expected']['b']
    u = torch.tensor([r['u'] for r in rows], dtype=torch.float64)
    v = torch.tensor([r['v_cmd'] for r in rows], dtype=torch.float64)
    w = torch.tensor([r['w_cmd'] for r in rows], dtype=torch.float64)
    cap = torch.tensor([r['v_cap'] for r in rows], dtype=torch.float64)
    out = torch.stack(CT.integrate_b(u, v, w, cap), dim=1)
    exp = torch.tensor([r['out'] for r in rows], dtype=torch.float64)
    assert (out - exp).abs().max() <= d['tolerance']['integrate']


def test_profiler_sequence():
    d = _load(_kind('profiler')[0])
    prof = CT.ProfilerBatch(1)
    n = int(round(C.DT / C.PROFILER_DT))
    for stp in d['expected']['steps']:
        v0, w0 = stp['cmd_from']
        v1, w1 = stp['cmd_to']
        subs = []
        for k in range(1, n + 1):
            f = k / n
            vr, wr = prof.step(torch.tensor([v0 + f * (v1 - v0)], dtype=torch.float64),
                               torch.tensor([w0 + f * (w1 - w0)], dtype=torch.float64))
            subs.append([float(vr[0]), float(wr[0])])
        exp = torch.tensor(stp['ref'], dtype=torch.float64)
        assert (torch.tensor(subs, dtype=torch.float64) - exp).abs().max() <= \
            d['tolerance']['profiler']


def test_profiler_batched_equals_single():
    """여러 env 가 서로 다른 곡률 모드에 있어도 배치 결과 = 개별 결과."""
    d = _load(_kind('profiler')[0])
    steps = d['expected']['steps']
    B = 3
    prof = CT.ProfilerBatch(B)
    n = int(round(C.DT / C.PROFILER_DT))
    for j, stp in enumerate(steps):
        v0, w0 = stp['cmd_from']
        v1, w1 = stp['cmd_to']
        for k in range(1, n + 1):
            f = k / n
            # env 0 = golden 순서, env 1 = 정지 명령, env 2 = golden 과 같은 명령
            vt = torch.tensor([v0 + f * (v1 - v0), 0.0, v0 + f * (v1 - v0)], dtype=torch.float64)
            wt = torch.tensor([w0 + f * (w1 - w0), 0.3, w0 + f * (w1 - w0)], dtype=torch.float64)
            vr, wr = prof.step(vt, wt)
            assert float(vr[0]) == float(vr[2]) and float(wr[0]) == float(wr[2])
        assert abs(float(vr[0]) - stp['ref'][-1][0]) <= d['tolerance']['profiler']


def test_safety_lite_rows():
    d = _load(_kind('safety')[0])
    rows = d['expected']['rows']

    def col(key):
        return torch.tensor([_inf(r['in'][key]) for r in rows], dtype=torch.float64)
    out = CT.safety_filter(col('v'), col('w'), col('d_main'), col('min_ttc'),
                           torch.tensor([bool(r['in']['proximity_stop']) for r in rows]))
    for i, r in enumerate(rows):
        assert float(out['v'][i]) == pytest.approx(r['out']['v'], abs=d['tolerance']['safety'])
        assert float(out['w'][i]) == pytest.approx(r['out']['w'], abs=d['tolerance']['safety'])
        assert int(out['zone'][i]) == r['out']['zone']
        assert float(out['speed_limit'][i]) == pytest.approx(_inf(r['out']['speed_limit']))
