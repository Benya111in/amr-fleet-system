"""
보상 설계 후보(RewardSpec)의 sim2d 구현 시험 — 기준 구현 일치, 정체 종단, 스크립트 정책 착취 순서.

후보 정의와 근거: rl/contract/amr_rl_contract/reward_variants.py,
docs/research/rl-navigation/reward_v12.md.
"""

import math

import numpy as np
import pytest
import torch

from amr_sim2d import exploit as X
from amr_sim2d import scripted as SP
from amr_sim2d.env import EnvConfig, Sim2DEnv
from amr_sim2d.state_io import states_from_bstate

from amr_rl_contract import reward as RR
from amr_rl_contract import reward_variants as RV

DEV = 'cuda' if torch.cuda.is_available() else 'cpu'
pytestmark = pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')


def make(spec, n=48, seed=5, mode='random', **kw):
    cfg = EnvConfig(num_envs=n, device=DEV, seed=seed, arm='B', obstacle_mode=mode,
                    path_len=kw.pop('path_len', (3.0, 12.0)), compile=False, n_paths=512,
                    reward=spec, **kw)
    return Sim2DEnv(cfg, log=lambda *a: None)


# v1.2.0 = 계약 기본이라 test_env 가 본다
@pytest.mark.parametrize('name', [n for n in RV.CANDIDATES if n != 'v1.2.0'])
def test_spec_reward_matches_reference_on_sim_states(name):
    """env(spec) 보상·종단 = 기준 구현 reward.step(spec) — 무작위 행동, 장애물, 모든 항."""
    spec = RV.get(name)
    env = make(name)
    env.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(3)
    n_cmp, seen_prog, seen_dev = 0, 0, 0
    for k in range(40):
        pre = states_from_bstate(env.bstate())
        u = torch.rand(env.N, 2, device=DEV, generator=g) * 2 - 1
        obs, rew, term, trunc, info = env.step(u)
        done = (term | trunc).cpu()
        post = states_from_bstate(env.bstate())
        for j in range(env.N):
            if done[j]:
                continue
            iv = float(info['interv'][j])
            r = RR.step(pre[j], post[j], post[j].prev_actions[0], bool(info['collision'][j]),
                        [((math.sqrt(iv), 0.0), (0.0, 0.0))], spec=spec,
                        stalled=bool(info['stalled'][j]))
            assert float(rew[j]) == pytest.approx(r.reward, abs=1e-4), (k, j)
            for key, val in r.terms.items():
                assert float(info['terms'][key][j]) == pytest.approx(val, abs=1e-4), (k, j, key)
            assert bool(term[j]) == r.terminated
            seen_prog += abs(r.terms.get('progress', 0.0)) > 1e-6
            seen_dev += r.terms.get('dev', 0.0) != 0.0
            n_cmp += 1
    assert n_cmp > 1000
    if spec.progress_kappa:
        assert seen_prog > 100


@pytest.mark.parametrize('name', ['prog50', 'prog50_st'])
def test_stall_is_terminal_only_for_stall_spec(name):
    """제자리 대기 → 20 s 뒤 정체. prog50_st 는 종단(−P_s, 부트스트랩 없음), prog50 은 truncation."""
    spec = RV.get(name)
    env = make(name, n=8, mode='none', dr=False)
    env.reset()
    zero = torch.zeros(env.N, 2, device=DEV)
    for k in range(260):
        _obs, rew, term, trunc, info = env.step(zero)
        if bool((term | trunc).any()):
            break
    assert 199 <= k <= 201
    assert bool(info['stalled'].all())
    d0 = info['episode']['d_start']                       # 대기라 d(s) = 시작 거리 그대로
    assert bool((info['episode']['goal_min'] > 0.5).all())  # 리셋 때 목표 직선거리가 채워졌다
    if spec.stall_terminal:
        assert bool(term.all()) and not bool(trunc.any())
        assert torch.allclose(info['terms']['stall'], torch.full_like(rew, -10.0).double())
        assert bool(info['episode']['stalled'].all())
        # 종단이므로 Φ(s') = 0: shaping = γ·0 − Φ(s) = κ·d(s)
        assert torch.allclose(info['terms']['shape'], spec.kappa * d0, atol=1e-6)
    else:
        assert bool(trunc.all()) and not bool(term.any())
        assert 'stall' not in info['terms']
        assert torch.allclose(info['terms']['shape'], (1.0 - 0.995) * spec.kappa * d0, atol=1e-6)


def test_success_or_collision_beats_stall_in_same_step():
    """정체와 성공·충돌이 같은 스텝이면 정체 종단이 아니다 (기준 구현 reward.step 과 같은 규칙)."""
    from amr_sim2d import contract_torch as T
    f = dict(dtype=torch.float64, device=DEV)
    one = torch.ones(3, **f)
    coll = torch.tensor([False, True, False], device=DEV)
    r = T.reward_step(0.2 * one, torch.tensor([0.0, 0.1, 0.1], **f), 0.0 * one,
                      torch.tensor([0.05, 0.1, 0.5], **f), 0.0 * one, torch.zeros(3, 2, **f),
                      torch.zeros(3, 2, **f), coll,
                      (torch.zeros(3, dtype=torch.bool, device=DEV), 0.0 * one, 0.0 * one),
                      0.0 * one, spec=RV.PROG50_ST, goal_dist_pre=0.2 * one,
                      stalled=torch.ones(3, dtype=torch.bool, device=DEV))
    assert r['success'].tolist() == [True, False, False]
    assert r['stall_term'].tolist() == [False, False, True]
    assert r['terminated'].tolist() == [True, True, True]
    assert r['terms']['stall'].tolist() == [0.0, 0.0, -10.0]


@pytest.mark.slow
def test_scripted_exploit_order_in_sim():
    """
    스크립트 정책의 무한 지평 가치 (연속 롤아웃) — 같은 시나리오 짝 비교.

    v1.1.0: 직전 정지·표류 ≤ 대기 (진행이 값을 올리지 못한다, G0 의 결함 고정).
    진행형 후보: 대기 < 직전 정지, 표류 < 직전 정지, 추종(성공) > 직전 정지 — 모든 시나리오에서.
    """
    env = Sim2DEnv(EnvConfig(num_envs=24, device=DEV, seed=21, arm='B', obstacle_mode='none',
                             path_len=(4.0, 9.0), dr=False, truncate=False, compile=False,
                             n_paths=256), log=lambda *a: None)
    recs = {p: X.record(env, SP.POLICIES[p], 1500, seed=21)
            for p in ('idle', 'stop_short', 'drift', 'follow')}
    res = {s: {p: X.spec_returns(r, RV.get(s)) for p, r in recs.items()}
           for s in ('v1.1.0', 'v1.2.0', 'prog25', 'prog50', 'prog50_st')}
    for p, r in res['v1.2.0'].items():          # env 기본 = 계약 v1.2.0
        assert r['consistency_vs_env'] < 1e-5, p
    assert res['v1.1.0']['idle']['consistency_vs_env'] is None
    v = res['v1.1.0']
    assert bool((v['stop_short']['g_boot'] <= v['idle']['g_boot'] + 1e-6).all())   # 진행해도 대기 이하
    assert bool((v['drift']['g_boot'] <= v['idle']['g_boot'] + 1e-6).all())
    for s in ('prog25', 'prog50', 'prog50_st'):
        r = res[s]
        ok = r['follow']['outcome'] == 0
        assert float(ok.double().mean()) > 0.75          # 추종기는 φ 앞에서 가끔 멈춘다 (G0: 0.98)
        # 스크립트 추종기가 성공하는 경로에서만 본다. 실패하는 경로(seed 21 의 5.05 m 경로 3 개)에서는 추종기가
        # ω 를 뱅뱅으로 떨어 평활 벌점(약 1.7)이 진행 효용(κ_b 0.25 × 4.2 m)보다 크다 — 보상의 결함이 아니라
        # 떨림에 대한 정당한 벌점이다 (reward_v12.md §4).
        moved = ok & (r['stop_short']['d_end'] < r['stop_short']['d0'] - 1.0)
        assert float(moved.double().mean()) > 0.75
        assert bool((r['stop_short']['g_boot'] > r['idle']['g_boot'])[moved].all()), s
        assert bool((r['drift']['g_boot'] < r['stop_short']['g_boot'])[moved].all()), s
        assert bool((r['follow']['g_boot'][ok] > r['stop_short']['g_boot'][ok]).all()), s
        assert np.isfinite(r['idle']['g_boot'].cpu().numpy()).all()
