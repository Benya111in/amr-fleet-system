"""
환경 대조 — 시뮬레이션이 만든 실제 상태에서 env 의 관측·마스크·보상이 numpy 기준 구현과 같은가.

golden 은 손으로 만든 17+11 사례뿐이다. 여기서는 env 를 무작위 행동·장애물·DR 로 굴려 생긴 상태를 계약 State 로
바꿔 기준 구현(amr_rl_contract)에 넣고 env 출력과 대조한다.
"""

import math

import numpy as np
import pytest
import torch

from amr_sim2d.env import EnvConfig, Sim2DEnv
from amr_sim2d.state_io import states_from_bstate

from amr_rl_contract import mask as RM
from amr_rl_contract import observation as RO
from amr_rl_contract import reward as RR

DEV = 'cuda' if torch.cuda.is_available() else 'cpu'
pytestmark = pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')


def make(arm='B', n=48, seed=5, compile_=False, mode='mixed', **kw):
    cfg = EnvConfig(num_envs=n, device=DEV, seed=seed, arm=arm, obstacle_mode=mode,
                    path_len=(3.0, 12.0), compile=compile_, n_paths=512, **kw)
    return Sim2DEnv(cfg, log=lambda *a: None)


def act_b(env, g):
    return torch.rand(env.N, 2, device=DEV, generator=g) * 2 - 1


def act_c(env, g):
    logits = torch.rand(env.N, 100, device=DEV, generator=g)
    logits = torch.where(env.mask, logits, torch.full_like(logits, -1.0))
    return logits.argmax(1)


@pytest.mark.parametrize('arm', ['B', 'C'])
def test_obs_and_mask_match_reference_on_sim_states(arm):
    env = make(arm)
    obs = env.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(0)
    checked = 0
    for k in range(36):
        a = act_b(env, g) if arm == 'B' else act_c(env, g)
        obs, *_ = env.step(a)
        if k % 6 != 5:
            continue
        sts = states_from_bstate(env.bstate(), range(0, env.N, 3))
        for j, st in zip(range(0, env.N, 3), sts):
            ref = RO.build_observation(st)
            err = np.abs(obs[j].double().cpu().numpy() - ref)
            assert err.max() <= 1e-5, (k, j, int(err.argmax()), float(err.max()))
            if arm == 'C':
                m = RM.compute_mask(st)['mask']
                assert env.mask[j].cpu().tolist() == [bool(x) for x in m], (k, j)
            checked += 1
    assert checked >= 90


def test_reward_cost_match_reference():
    env = make('B', mode='random')
    env.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(1)
    n_cmp = 0
    for k in range(40):
        pre = states_from_bstate(env.bstate())
        obs, rew, term, trunc, info = env.step(act_b(env, g))
        done = (term | trunc).cpu()
        post = states_from_bstate(env.bstate())
        for j in range(env.N):
            if done[j]:
                continue
            a_exec = post[j].prev_actions[0]
            iv = float(info['interv'][j])
            r = RR.step(pre[j], post[j], a_exec, bool(info['collision'][j]),
                        [((math.sqrt(iv), 0.0), (0.0, 0.0))])
            assert float(rew[j]) == pytest.approx(r.reward, abs=1e-4), (k, j)
            assert np.allclose(info['cost'][j].cpu().numpy(), r.cost, atol=1e-6)
            assert bool(term[j]) == r.terminated
            assert post[j].dev.in_dev == r.dev.in_dev
            assert post[j].dev.t_since_peak == pytest.approx(r.dev.t_since_peak, abs=1e-9)
            n_cmp += 1
    assert n_cmp > 1000


def test_terminal_events_and_autoreset():
    env = make('B', n=128, mode='random', n_obstacles=(4, 8))
    env.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(2)
    seen = {'collision': 0, 'trunc': 0}
    for _ in range(150):
        obs, rew, term, trunc, info = env.step(act_b(env, g))
        assert torch.isfinite(obs).all() and torch.isfinite(rew).all()
        coll = info['collision']
        assert bool((term[coll]).all())                         # 접촉 = 즉시 종단
        assert torch.allclose(info['terms']['collision'][coll].float(),
                              torch.full_like(rew[coll], -30.0))
        assert not bool((term & trunc).any())
        if 'done_idx' in info:
            di = info['done_idx']
            assert bool((env.ep_len[di] == 0).all())             # 리셋됨
            assert info['final_obs'].shape == (di.numel(), obs.shape[1])
            assert torch.allclose(obs[di, 213], (env.d_arc[di] / 10).clamp(max=1).float(),
                                  atol=1e-5)                    # 새 에피소드 관측
        seen['collision'] += int(coll.sum())
        seen['trunc'] += int(trunc.sum())
    assert seen['collision'] > 0


def test_seed_determinism():
    a, b = make('B', seed=9), make('B', seed=9)
    oa, ob = a.reset(), b.reset()
    g1 = torch.Generator(device=DEV)
    g1.manual_seed(3)
    for _ in range(15):
        u = act_b(a, g1)
        oa, ra, *_ = a.step(u)
        ob, rb, *_ = b.step(u)
        assert torch.equal(oa, ob) and torch.equal(ra, rb)


@pytest.mark.slow
def test_compiled_matches_eager():
    """torch.compile 경로 = eager 경로 (inductor argmax 동률 버그 회귀 방지, contract_torch.first_true)."""
    e0, e1 = make('B', compile_=False, seed=4), make('B', compile_=True, seed=4)
    o0, o1 = e0.reset(), e1.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(4)
    for _ in range(25):
        u = act_b(e0, g)
        o0, r0, *_ = e0.step(u)
        o1, r1, *_ = e1.step(u)
        assert float((o0 - o1).abs().max()) < 1e-4
        assert float((e0.pose - e1.pose).abs().max()) < 1e-9
