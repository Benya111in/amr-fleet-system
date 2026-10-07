"""
SAC 연결 시험 (amr_isaac.sac_skrl, Isaac 불필요 — skrl 2.1.0 CPU).

- 짓눌린 가우시안 log π 보정식 = 직접식, 표본 행동이 env 로 가는 raw 이고 log_prob 가 tanh(raw) 의 밀도
- ContractScaler 병렬 갱신 = 한꺼번에 계산한 평균·분산, forward = 계약 정규화 (분산 하한)
- skrl SAC 에이전트가 우리 모델로 갱신을 돈다 (α·Q 유한, 엔트로피 계수 움직임)
- 장난감 과제(상태별 목표 행동)에서 결정론 행동 tanh(μ) 가 목표로 다가간다 (학습 신호)
- export_state → PolicySAC (ONNX 입출력 모양) 이 학습 정책과 같은 μ 를 낸다
- 잘림 전이의 다음 관측 대체 규칙
"""

import importlib.util
import math

import gymnasium
import numpy as np
import pytest
import torch

from amr_isaac import policy_export as PE
from amr_isaac import sac_skrl as SK

# skrl 2.1.0 은 amr-ai/rl:0.3 과 Isaac 이미지 모두에 있다 — 없으면 이 파일만 건너뛴다 (모듈 수준 importorskip 금지,
# test_ppo_lag.py 머리말)
pytestmark = pytest.mark.skipif(importlib.util.find_spec('skrl') is None, reason='skrl 없음')


def spaces(n_obs=6, n_act=2):
    return (gymnasium.spaces.Box(-np.inf, np.inf, (n_obs,)),
            gymnasium.spaces.Box(-np.inf, np.inf, (n_act,)))


def test_squash_log_prob_matches_direct_formula():
    """짓눌린 밀도 log π(tanh(raw)) = Σ log N(raw) − Σ log(1 − tanh²(raw)) (|raw| ≤ 4 에서 직접식과 1e-9)."""
    torch.manual_seed(0)
    mu = torch.randn(64, 2, dtype=torch.float64)
    std = torch.rand(64, 2, dtype=torch.float64) + 0.1
    dist = torch.distributions.Normal(mu, std)
    raw = torch.empty(64, 2, dtype=torch.float64).uniform_(-4, 4)
    got = SK.squash_log_prob(dist, raw)
    ref = (dist.log_prob(raw) - torch.log1p(-torch.tanh(raw) ** 2)).sum(-1, keepdim=True)
    assert got.shape == (64, 1)
    assert torch.max(torch.abs(got - ref)) < 1e-9


def test_squash_density_integrates_to_one():
    """1 차원: a = tanh(raw) 공간에서 exp(log π) 를 (−1, 1) 위로 적분하면 1 (변수 변환 확인)."""
    dist = torch.distributions.Normal(torch.tensor([[0.3]], dtype=torch.float64),
                                      torch.tensor([[0.8]], dtype=torch.float64))
    a = torch.linspace(-0.999999, 0.999999, 400001, dtype=torch.float64).reshape(-1, 1)
    raw = torch.atanh(a)
    dens = SK.squash_log_prob(dist, raw).exp().reshape(-1)
    integral = torch.trapezoid(dens, a.reshape(-1))
    assert float(integral) == pytest.approx(1.0, abs=2e-3)


def test_contract_scaler_parallel_update_and_forward():
    """여러 배치로 갱신한 통계 = 전체를 한 번에 (epsilon 무게 제외), forward = contract_normalize."""
    torch.manual_seed(1)
    sc = SK.ContractScaler(5, epsilon=0.0 + 1e-12)
    xs = [torch.randn(n, 5) * torch.tensor([1.0, 0.01, 3.0, 0.5, 2.0]) + 4.0
          for n in (7, 31, 200, 3)]
    for x in xs:
        sc.update(x)
    allx = torch.cat(xs)
    assert torch.allclose(sc.mean, allx.mean(0, keepdim=True), atol=1e-5)
    assert torch.allclose(sc.var, allx.var(0, unbiased=False, keepdim=True), atol=1e-4)
    y = sc(allx, train=True)                      # train=True 는 갱신하지 않는다
    assert torch.allclose(sc.mean, allx.mean(0, keepdim=True), atol=1e-5)
    ref = PE.contract_normalize(allx, sc.mean, sc.var)
    assert torch.equal(y, ref)
    # 분산 하한: 두 번째 축 분산 1e-4 < 1e-2 → 하한으로 나눈다
    assert float(sc.var[0, 1]) < PE.VAR_FLOOR
    sc.frozen = True
    m0 = sc.mean.clone()
    sc.update(torch.randn(10, 5) + 100)
    assert torch.equal(sc.mean, m0)


def test_actor_samples_raw_and_reports_squashed_log_prob():
    """act() 는 raw (무한 범위) 를 내고 log_prob 는 taken_actions 를 주면 그 raw 의 짓눌린 밀도."""
    obs_s, act_s = spaces()
    models = SK.build_models(obs_s, act_s, 'cpu', hidden=(16, 16))
    pol = models['policy']
    x = torch.randn(32, 6)
    torch.manual_seed(3)
    raw, out = pol.act({'observations': x})
    assert raw.shape == (32, 2) and out['log_prob'].shape == (32, 1)
    mu, o2 = pol.compute({'observations': x})
    dist = torch.distributions.Normal(mu, o2['log_std'].exp())
    assert torch.allclose(out['log_prob'], SK.squash_log_prob(dist, raw), atol=1e-6)
    _, out3 = pol.act({'observations': x, 'taken_actions': raw})
    assert torch.allclose(out3['log_prob'], out['log_prob'], atol=1e-6)
    other = torch.zeros_like(raw) + 0.3                   # 표본과 다른 행동의 밀도
    _, out4 = pol.act({'observations': x, 'taken_actions': other})
    assert torch.allclose(out4['log_prob'], SK.squash_log_prob(dist, other), atol=1e-6)
    r, _ = pol.random_act({'observations': x})
    assert torch.all(torch.tanh(r).abs() < 1.0)
    # Q 는 tanh(raw) 를 받는다: raw 를 크게 바꿔도 tanh 가 포화되면 Q 가 같다
    q = models['critic_1']
    big = torch.full((32, 2), 30.0)
    v1, _ = q.act({'observations': x, 'taken_actions': big})
    v2, _ = q.act({'observations': x, 'taken_actions': big * 2})
    assert torch.allclose(v1, v2)


def _toy_target(o):
    return torch.tanh(o[:, :2])          # 상태별 목표 행동 ∈ (−1, 1)²


def _run_toy(steps=400, n_envs=32, hp=None, seed=0):
    torch.manual_seed(seed)
    obs_s, act_s = spaces()
    h = {'batch_size': 256, 'gradient_steps': 2, 'learning_starts': 5,
         'memory_transitions': 20000, 'initial_entropy_value': 0.05}
    h.update(hp or {})
    agent, memory, scaler, cfg = SK.make_agent(obs_s, act_s, n_envs, 'cpu', hp=h, hidden=(64, 64))
    agent.init()
    agent.enable_training_mode(True)
    obs = torch.randn(n_envs, 6)
    alphas = []
    for t in range(steps):
        scaler.update(obs)
        with torch.no_grad():
            raw, _ = agent.act(obs, None, timestep=t, timesteps=steps)
        rew = -((torch.tanh(raw) - _toy_target(obs)) ** 2).sum(-1, keepdim=True)
        nxt = torch.randn(n_envs, 6)
        term = torch.ones(n_envs, 1, dtype=torch.bool)       # 한 스텝 과제 (bandit)
        trunc = torch.zeros(n_envs, 1, dtype=torch.bool)
        agent.record_transition(observations=obs, states=None, actions=raw, rewards=rew,
                                next_observations=nxt, next_states=None, terminated=term,
                                truncated=trunc, infos={}, timestep=t, timesteps=steps)
        agent.post_interaction(timestep=t, timesteps=steps)
        alphas.append(float(agent._entropy_coefficient))
        obs = nxt
    return agent, scaler, cfg, alphas


def test_skrl_sac_learns_toy_bandit_and_exports():
    """장난감 bandit: 결정론 행동 tanh(μ) 의 목표 오차가 무작위(≈0.6)보다 크게 줄고, α 가 움직이고, PolicySAC = 정책."""
    agent, scaler, cfg, alphas = _run_toy()
    assert cfg['replay_ratio'] == pytest.approx(256 * 2 / 32)
    x = torch.randn(512, 6)
    with torch.no_grad():
        mu, _ = agent.policy.compute({'observations': scaler(x)})
    err = float(((torch.tanh(mu) - _toy_target(x)) ** 2).sum(-1).mean())
    with torch.no_grad():
        mu0, _ = SK.build_models(*spaces(), 'cpu', hidden=(64, 64))['policy'].compute(
            {'observations': x})
    err0 = float(((torch.tanh(mu0) - _toy_target(x)) ** 2).sum(-1).mean())
    assert err < 0.25 * err0, (err, err0)
    assert alphas[-1] != pytest.approx(alphas[0])
    assert all(math.isfinite(a) for a in alphas)
    ck = SK.export_state(agent)
    pol = PE.load_policy(ck)
    assert isinstance(pol, PE.PolicySAC)
    act, mu_e, ls = pol(x)
    assert act.shape == (512, 2) and ls.shape == (512, 2)
    assert torch.allclose(mu_e, mu, atol=1e-5)
    assert torch.allclose(act, torch.tanh(mu), atol=1e-5)


def test_policy_sac_onnx_roundtrip(tmp_path):
    """SAC 정책(PolicySAC)을 onnx_io.md 이름·모양으로 내보내고 ORT 출력 = torch (1e-5)."""
    if importlib.util.find_spec('onnxruntime') is None:
        pytest.skip('onnxruntime 없음')
    import onnxruntime as ort
    agent, scaler, _, _ = _run_toy(steps=30)
    pol = PE.load_policy(SK.export_state(agent)).eval()
    # 계약 관측 차원으로 다시 만들 필요는 없다 — 이름·모양·동적 배치만 본다
    path = str(tmp_path / 'sac.onnx')
    PE.export_onnx(pol, path, {'contract_version': '1.2.0', 'arm': 'B', 'algo': 'SAC test'})
    sess = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    assert [i.name for i in sess.get_inputs()] == ['obs']
    assert [o.name for o in sess.get_outputs()] == ['action', 'mu', 'log_std']
    x = np.random.default_rng(0).normal(size=(7, 6)).astype(np.float32)
    outs = sess.run(None, {'obs': x})
    with torch.no_grad():
        ref = pol(torch.from_numpy(x))
    for o, r in zip(outs, ref):
        assert np.max(np.abs(o - r.numpy())) < 1e-5


def test_next_obs_for_memory():
    """잘림이고 종단이 아닌 행만 현재 관측으로, 나머지는 다음 관측."""
    obs = torch.arange(8.0).reshape(4, 2)
    nxt = -obs - 1
    term = torch.tensor([False, True, False, True])
    trunc = torch.tensor([False, False, True, True])
    out = SK.next_obs_for_memory(obs, nxt, term, trunc)
    assert torch.equal(out[0], nxt[0]) and torch.equal(out[1], nxt[1])
    assert torch.equal(out[2], obs[2]) and torch.equal(out[3], nxt[3])
