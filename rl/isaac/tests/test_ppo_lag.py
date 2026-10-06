"""
제약 PPO (rsl_rl 상속) 시험 — rsl_rl 이 있는 Isaac 이미지 파이썬에서만 돈다 (amr-ai/rl:0.3 에서는 건너뛴다).

    rl/isaac/run_isaac.sh pytest_lag -m pytest -q rl/isaac/tests/test_ppo_lag.py   (README 「시험」)

- λ 를 0 으로 고정하면 vanilla rsl_rl PPO 와 actor·critic 매개변수가 비트 단위로 같다 (비용 critic 은 따로 둔 난수열)
- 비용이 큰 장난감 과제에서 예산이 작으면 λ 가 오르고, 예산이 크면 0 에 머문다
- 비용 critic 이 비용 반환을 배운다 (손실 감소)
"""

import importlib.util

import pytest
import torch

from amr_isaac.pid_lagrange import PIDLagrangian

# 모듈 수준 importorskip 은 amr-ai/rl:0.3 의 pytest 에서 (원인 미확인) 함께 모은 다른 시험 파일까지 건너뛰게
# 만들었다 — skipif 표식으로 이 파일만 건너뛴다.
HAVE_RSL = importlib.util.find_spec('rsl_rl') is not None
pytestmark = pytest.mark.skipif(not HAVE_RSL, reason='rsl_rl 없음 (Isaac 이미지 파이썬에서 돈다)')
if HAVE_RSL:
    from tensordict import TensorDict
    from rsl_rl.algorithms import PPO
    from rsl_rl.modules import ActorCritic

    from amr_isaac.ppo_lag import PPOPIDLag

N, D, T = 16, 4, 8
PPO_KW = dict(num_learning_epochs=2, num_mini_batches=2, clip_param=0.2, gamma=0.99, lam=0.95,
              value_loss_coef=1.0, entropy_coef=0.0, learning_rate=1e-3, max_grad_norm=1.0,
              use_clipped_value_loss=True, schedule='fixed', desired_kl=0.01, device='cpu')


class ToyEnv:
    """한 축 위치 x: 행동 a ∈ ℝ² 의 첫 성분만큼 이동, 보상 = −|x − 1|, 비용 = a₀² (움직일수록 비용)."""

    def __init__(self, seed=0, terminal=False):
        self.terminal = terminal          # True 면 10 스텝 끝이 종단 (시간 초과 bootstrap 없음)
        g = torch.Generator().manual_seed(seed)
        self.x = torch.rand(N, generator=g) * 2 - 1
        self.t = torch.zeros(N, dtype=torch.long)
        self.cost = torch.zeros(N)

    def obs(self):
        o = torch.zeros(N, D)
        o[:, 0] = self.x
        o[:, 1] = self.t.float() / 10
        return TensorDict({'policy': o}, batch_size=[N])

    def step(self, a):
        a0 = torch.tanh(a[:, 0])
        self.x = self.x + 0.3 * a0
        self.cost = a0 ** 2
        r = -(self.x - 1.0).abs()
        self.t += 1
        done = self.t >= 10
        to = torch.zeros_like(done) if self.terminal else done.clone()
        self.x = torch.where(done, torch.zeros_like(self.x), self.x)
        self.t = torch.where(done, torch.zeros_like(self.t), self.t)
        return self.obs(), r, done, {'time_outs': to}


def make_policy(seed):
    torch.manual_seed(seed)
    o = TensorDict({'policy': torch.zeros(N, D)}, batch_size=[N])
    return ActorCritic(o, {'policy': ['policy'], 'critic': ['policy']}, 2,
                       actor_obs_normalization=True, critic_obs_normalization=True,
                       actor_hidden_dims=[16, 16], critic_hidden_dims=[16, 16], activation='elu',
                       init_noise_std=0.6, noise_std_type='log')


def run(alg, env, iters):
    obs = env.obs()
    alg.policy.train()
    lams = []
    for _ in range(iters):
        with torch.inference_mode():
            for _ in range(T):
                a = alg.act(obs)
                obs, r, d, ex = env.step(a)
                alg.process_env_step(obs, r, d, ex)
            alg.compute_returns(obs)
        alg.update()
        lams.append(getattr(alg, 'multiplier', None))
    return lams


def build(kind, seed=0, budget=0.5, fixed=None, env=None, lr=None, cost_lr=3e-4):
    pol = make_policy(seed)
    kw = dict(PPO_KW)
    if lr is not None:
        kw['learning_rate'] = lr
    if kind == 'ppo':
        alg = PPO(pol, **kw)
    else:
        pid = PIDLagrangian(kp=0.5, ki=0.2, kd=0.0, budget=budget, lambda_max=5.0)
        alg = PPOPIDLag(pol, cost_fn=lambda: env.cost, pid=pid, cost_hidden=(16, 16),
                        fixed_lambda=fixed, cost_seed=seed, cost_lr=cost_lr, **kw)
    alg.init_storage('rl', N, T, TensorDict({'policy': torch.zeros(N, D)}, batch_size=[N]), [2])
    return alg


def test_lambda_zero_reproduces_vanilla_ppo_bitwise():
    """λ 고정 0: 같은 시드의 vanilla PPO 와 actor·critic·log_std 가 비트 단위로 같다."""
    e1, e2 = ToyEnv(1), ToyEnv(1)
    torch.manual_seed(5)
    a1 = build('ppo')
    torch.manual_seed(5)
    a2 = build('lag', fixed=0.0, env=e2)
    torch.manual_seed(11)
    run(a1, e1, 4)
    torch.manual_seed(11)
    run(a2, e2, 4)
    s1, s2 = a1.policy.state_dict(), a2.policy.state_dict()
    assert set(s1) == set(s2)
    for k in s1:
        assert torch.equal(s1[k], s2[k]), k


def test_lambda_responds_to_budget():
    """비용 a₀² 가 큰 과제: 예산 0.05 면 λ > 0 으로 오르고, 예산 100 이면 0 그대로."""
    e_small, e_big = ToyEnv(2), ToyEnv(2)
    small = build('lag', budget=0.05, env=e_small)
    big = build('lag', budget=100.0, env=e_big)
    torch.manual_seed(3)
    ls = run(small, e_small, 6)
    torch.manual_seed(3)
    lb = run(big, e_big, 6)
    assert ls[-1] > 0.5 and all(x == 0.0 for x in lb)
    # 마지막 반복에 끝난 에피소드가 없을 수 있다 (롤아웃 8 < 에피소드 10) — 그때 λ 는 그대로이고 J 는 None
    assert small.pid.last['J'] > 0.05 and small.pid.last['error'] > 0
    assert 'pid_error' in small.last_info


def test_cost_critic_learns():
    """
    정책 고정(학습률 0)·종단 에피소드(남은 비용 합이 t 의 함수)에서 비용 critic 손실이 준다 (앞 3 대 뒤 3 반복).

    시간 초과 bootstrap 이 있으면 목표가 c̄/(1−γ) 로 천천히 올라가 짧은 시험으로는 줄지 않는다 (처음 판의 실패).
    """
    env = ToyEnv(4, terminal=True)
    alg = build('lag', budget=0.5, env=env, fixed=0.0, lr=0.0, cost_lr=3e-3)
    obs = env.obs()
    losses = []
    torch.manual_seed(0)
    for _ in range(30):
        with torch.inference_mode():
            for _ in range(T):
                a = alg.act(obs)
                obs, r, d, ex = env.step(a)
                alg.process_env_step(obs, r, d, ex)
            alg.compute_returns(obs)
        losses.append(alg.update()['cost_value_function'])
    assert sum(losses[-3:]) < 0.5 * sum(losses[:3]), losses
