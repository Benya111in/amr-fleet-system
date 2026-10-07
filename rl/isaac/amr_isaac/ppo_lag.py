"""
제약 PPO (PID Lagrangian) — rsl_rl 3.1.2 PPO 를 고치지 않고 상속으로 감싼다 (Isaac 이미지의 rsl_rl 이 필요하다).

무엇이 vanilla 와 다른가:
  - **별도 비용 critic** V_c(s) (actor·보상 critic 과 매개변수를 나누지 않는다, 관측 정규화 통계만 보상 critic 과 공유)
  - 스텝 비용 c_t = cost_fn() (env 의 이번 전이 비용, 기본 계약 §7.1 개입 = 보정 크기) — 잘림에서는 γ·V_c(s_t) 로 bootstrap
  - 비용 GAE (보상과 같은 γ·λ) → 이익 결합 A = (std(A_r) − λ·center(A_c))/(1 + λ)
    (pid_lagrange.combine_advantages)
  - 반복마다 그 롤아웃에서 끝난 에피소드의 **비할인 비용 합 평균 J** 로 PID 승수 갱신 (pid_lagrange.PIDLagrangian)
  - 보상 critic 의 목표는 계약 보상 그대로 (env 의 보상 벌점 λ·c 는 쓰지 않는다 — env.lagrange 는 0 이어야 한다)
λ = 0 이면 이익이 vanilla 와 비트 단위로 같다 (같은 GAE 식·같은 표준화, 비용 critic 의 초기화·미니배치는 따로 둔
난수열을 쓴다) — tests/test_ppo_lag.py 가 vanilla PPO 와 매개변수가 같게 나오는지 본다.
"""

import torch
from torch import nn

from rsl_rl.algorithms import PPO

from .pid_lagrange import combine_advantages, EpisodeCostTracker, gae


def _mlp(n_in, hidden, activation):
    act = {'elu': nn.ELU, 'relu': nn.ReLU, 'tanh': nn.Tanh}[activation]
    layers, last = [], n_in
    for h in hidden:
        layers += [nn.Linear(last, h), act()]
        last = h
    layers.append(nn.Linear(last, 1))
    return nn.Sequential(*layers)


class PPOPIDLag(PPO):
    """PPO + 비용 critic + PID 승수 (승수 하나 = 비용 채널 하나)."""

    def __init__(self, policy, *, cost_fn, pid, cost_hidden=(512, 256, 128), activation='elu',
                 cost_lr=3e-4, cost_seed=0, fixed_lambda=None, **ppo_kwargs):
        """cost_fn() → [N] 이번 전이 비용, pid = PIDLagrangian, fixed_lambda 면 λ 를 갱신하지 않는다 (시험용)."""
        super().__init__(policy, **ppo_kwargs)
        self.cost_fn = cost_fn
        self.pid = pid
        self.fixed_lambda = fixed_lambda
        # 승수 이름을 lam 으로 쓰지 않는다 — rsl_rl PPO 의 self.lam 은 GAE λ 다 (처음 판이 덮어써 GAE 가 승수를 썼다,
        # 시험 test_lambda_zero_reproduces_vanilla_ppo_bitwise 가 잡았다)
        self.multiplier = float(fixed_lambda if fixed_lambda is not None else pid.lam)
        nz = policy.critic_obs_normalizer
        n_in = int(nz._mean.shape[-1]) if hasattr(nz, '_mean') else \
            int(policy.critic[0].in_features)
        dev = self.device
        devs = [torch.device(dev).index or 0] if torch.device(dev).type == 'cuda' else []
        with torch.random.fork_rng(devices=devs):
            torch.manual_seed(int(cost_seed) + 7919)
            self.cost_critic = _mlp(n_in, cost_hidden, activation).to(dev)
        self.cost_opt = torch.optim.Adam(self.cost_critic.parameters(), lr=cost_lr)
        self._gen = torch.Generator(device=dev)
        self._gen.manual_seed(int(cost_seed) + 104729)
        self._cv = None
        self.c_rewards = self.c_values = self.c_returns = None
        self.tracker = None
        self.last_info = {}
        self.history = []          # 반복마다 last_info (학습 스크립트가 lagrange_history.jsonl 로 옮긴다)

    def init_storage(self, training_type, num_envs, num_transitions_per_env, obs, actions_shape):
        """보상 저장소 + 비용 버퍼."""
        super().init_storage(training_type, num_envs, num_transitions_per_env, obs, actions_shape)
        shape = (num_transitions_per_env, num_envs, 1)
        self.c_rewards, self.c_values, self.c_returns = (
            torch.zeros(shape, device=self.device) for _ in range(3))
        self.tracker = EpisodeCostTracker(num_envs, self.device)

    def cost_value(self, obs):
        """V_c(s) — 보상 critic 과 같은 관측·같은 정규화 (통계 공유, 갱신은 policy 가 한다)."""
        x = self.policy.critic_obs_normalizer(self.policy.get_critic_obs(obs))
        return self.cost_critic(x)

    def act(self, obs):
        """행동 + V_c(s_t) 기록."""
        a = super().act(obs)
        self._cv = self.cost_value(obs).detach()
        return a

    def process_env_step(self, obs, rewards, dones, extras):
        """비용 기록 (잘림 bootstrap) 뒤 vanilla 처리."""
        k = self.storage.step
        c = self.cost_fn().reshape(-1, 1).to(torch.float32).to(self.device)
        self.tracker.step(c, dones)
        cb = c.clone()
        if 'time_outs' in extras:
            cb += self.gamma * self._cv * extras['time_outs'].reshape(-1, 1).to(self.device)
        self.c_rewards[k].copy_(cb)
        self.c_values[k].copy_(self._cv)
        super().process_env_step(obs, rewards, dones, extras)

    def compute_returns(self, obs):
        """보상·비용 GAE (GAE λ = self.lam) → 결합 이익 (승수 = self.multiplier, 지금 값)."""
        st = self.storage
        last_v = self.policy.evaluate(obs).detach()
        ret_r, adv_r = gae(st.rewards, st.values, st.dones, last_v, self.gamma, self.lam)
        last_c = self.cost_value(obs).detach()
        ret_c, adv_c = gae(self.c_rewards, self.c_values, st.dones, last_c, self.gamma, self.lam)
        st.returns = ret_r
        self.c_returns = ret_c
        st.advantages = combine_advantages(adv_r, adv_c, self.multiplier)
        self._adv_c_std = float(adv_c.std())

    def _update_cost_critic(self):
        """비용 critic 회귀 (PPO 와 같은 epoch·미니배치 수, 따로 둔 난수열)."""
        obs = self.storage.observations.flatten(0, 1)
        ret = self.c_returns.flatten(0, 1)
        B = ret.shape[0]
        mb = B // self.num_mini_batches
        tot = 0.0
        for _ in range(self.num_learning_epochs):
            perm = torch.randperm(B, device=self.device, generator=self._gen)
            for i in range(self.num_mini_batches):
                idx = perm[i * mb:(i + 1) * mb]
                v = self.cost_value(obs[idx])
                loss = (v - ret[idx]).pow(2).mean()
                self.cost_opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.cost_critic.parameters(), self.max_grad_norm)
                self.cost_opt.step()
                tot += float(loss)
        return tot / max(self.num_learning_epochs * self.num_mini_batches, 1)

    def update(self):
        """기본 PPO 갱신 → 비용 critic 갱신 → PID 승수 갱신."""
        loss = super().update()
        loss['cost_value_function'] = self._update_cost_critic()
        J, n_ep = self.tracker.pop()
        if J is not None and self.fixed_lambda is None:
            self.multiplier = self.pid.update(J)
        self.last_info = {'J': J, 'episodes': n_ep, 'lambda': self.multiplier,
                          'adv_c_std': getattr(self, '_adv_c_std', None), **{
                              f'pid_{k}': v for k, v in self.pid.last.items()}}
        self.history.append(dict(self.last_info))
        loss['lambda'] = self.multiplier
        return loss
