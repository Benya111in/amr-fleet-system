"""
SAC (skrl 2.1.0) 모델·정규화·에이전트 조립 — 같은 env·관측·행동·보상에 연결한다 (비교군 B, 연속 tanh 행동).

Isaac 비의존 (torch + skrl). amr-ai/rl:0.3 과 Isaac 이미지 모두 skrl 2.1.0 이 있다 — CPU 시험이 그대로 돈다.

행동 규약 (계약 §5, onnx_io.md): env 는 원출력 raw 를 받아 u = tanh(raw) 로 쓴다. 그래서 정책은 **tanh 전 표본 raw**
를 env·재생 버퍼에 넘기고, log π 는 tanh 로 짓눌린 행동 a = tanh(raw) 의 밀도로 계산한다 (Haarnoja et al. 2018
부록 C 의 변수 변환: log π(a) = Σ [log N(raw_i; μ_i, σ_i) − log(1 − tanh²(raw_i))]).
skrl 의 GaussianMixin 은 이 보정이 없어 (행동을 자르기만 한다) 쓰지 않고 SquashedGaussianActor 를 따로 둔다.
엔트로피 목표 −dim A = −2 는 짓눌린 행동 공간의 엔트로피에 건다 (계약 §9).
Q 는 짓눌린 행동 tanh(raw) 를 입력으로 받는다 (포화 영역에서 평평한 raw 대신).
결정론 평가·ONNX: action = tanh(μ) (onnx_io.md 의 action·mu·log_std, log_std 는 상태 의존 출력).

관측 정규화 (onnx_io.md, 학습 PPO 와 같은 식): x = clip((obs − μ)/√(max(σ², 1e-2) + 1e-8), −10, 10).
통계는 학습 루프가 롤아웃 관측으로만 갱신한다 (ContractScaler.update) — skrl 이 갱신 때 재생 표본으로
부르는 train=True 는 무시한다.
"""

import math

import torch
from torch import nn
from torch.distributions import Normal

from .policy_export import contract_normalize, NORM_CLIP, VAR_FLOOR

# 상태 의존 log σ 범위 (tanh 전) [설계 선택, SpinningUp SAC 는 −20·2]
LOG_STD_MIN, LOG_STD_MAX = -5.0, 2.0
ACTS = {'elu': nn.ELU, 'relu': nn.ReLU, 'tanh': nn.Tanh}


def mlp(sizes, activation='elu'):
    """Linear–활성 반복 (마지막 층 뒤 활성 없음)."""
    layers = []
    for i in range(len(sizes) - 1):
        layers.append(nn.Linear(sizes[i], sizes[i + 1]))
        if i < len(sizes) - 2:
            layers.append(ACTS[activation]())
    return nn.Sequential(*layers)


def squash_log_prob(dist, raw):
    """
    짓눌린 행동의 log π(tanh(raw)) [B, 1].

    수치 안정형 log(1 − tanh²(x)) = 2(log 2 − x − softplus(−2x)).
    """
    lp = dist.log_prob(raw)
    corr = 2.0 * (math.log(2.0) - raw - nn.functional.softplus(-2.0 * raw))
    return (lp - corr).sum(dim=-1, keepdim=True)


class ContractScaler(nn.Module):
    """계약 정규화 (running mean/var, 병렬 분산 갱신) — skrl 전처리기 자리에 넣는다."""

    def __init__(self, size, device=None, epsilon=1e-4):
        """관측 차원 size."""
        super().__init__()
        n = int(size if isinstance(size, int) else getattr(size, 'shape', [size])[0])
        dev = torch.device(device) if device is not None else None
        self.register_buffer('mean', torch.zeros(1, n, device=dev))
        self.register_buffer('var', torch.ones(1, n, device=dev))
        self.register_buffer('count', torch.tensor(epsilon, device=dev))
        self.frozen = False

    @torch.no_grad()
    def update(self, x):
        """배치 [B, n] 로 통계 갱신 (frozen 이면 그대로)."""
        if self.frozen:
            return
        x = x.reshape(-1, self.mean.shape[1]).to(self.mean.dtype)
        b = x.shape[0]
        bm = x.mean(0, keepdim=True)
        bv = x.var(0, unbiased=False, keepdim=True)
        tot = self.count + b
        delta = bm - self.mean
        self.mean += delta * b / tot
        m2 = self.var * self.count + bv * b + delta.pow(2) * self.count * b / tot
        self.var = m2 / tot
        self.count = tot

    def forward(self, x, train=False, inverse=False):
        """정규화 (train·inverse 는 skrl 호출 규약 — 갱신하지 않는다)."""
        if inverse:
            raise NotImplementedError('역정규화는 쓰지 않는다')
        if x is None:
            return None
        return contract_normalize(x, self.mean, self.var)


def _skrl_model():
    from skrl.models.torch import Model
    return Model


def build_models(obs_space, act_space, device, hidden=(512, 256, 128), activation='elu'):
    """policy·critic_1·critic_2·target_critic_1·target_critic_2 (skrl Model)."""
    Model = _skrl_model()
    from skrl.models.torch import DeterministicMixin

    class SquashedGaussianActor(Model):
        """raw ~ N(μ(s), σ(s)), 행동 a = tanh(raw) — env 에는 raw 를 넘긴다."""

        def __init__(self):
            Model.__init__(self, observation_space=obs_space, action_space=act_space,
                           device=device)
            n_obs, n_act = self.num_observations, self.num_actions
            self.trunk = mlp((n_obs,) + tuple(hidden), activation)
            self.trunk.append(ACTS[activation]())
            self.mu_head = nn.Linear(hidden[-1], n_act)
            self.log_std_head = nn.Linear(hidden[-1], n_act)
            self.hidden = tuple(hidden)
            self.activation = activation

        def compute(self, inputs, role=''):
            h = self.trunk(inputs['observations'])
            mu = self.mu_head(h)
            log_std = torch.clamp(self.log_std_head(h), LOG_STD_MIN, LOG_STD_MAX)
            return mu, {'log_std': log_std}

        def act(self, inputs, role=''):
            mu, out = self.compute(inputs, role)
            dist = Normal(mu, out['log_std'].exp())
            raw = dist.rsample()
            taken = inputs.get('taken_actions', raw)
            out['log_prob'] = squash_log_prob(dist, taken)
            out['mean_actions'] = mu
            return raw, out

        def random_act(self, inputs, role=''):
            b = inputs['observations'].shape[0]
            u = torch.rand(b, self.num_actions, device=self.device) * 1.998 - 0.999
            return torch.atanh(u), {}

    class SquashedQ(DeterministicMixin, Model):
        """Q(s, tanh(raw)) — 재생 버퍼의 raw 행동을 짓눌러 넣는다."""

        def __init__(self):
            Model.__init__(self, observation_space=obs_space, action_space=act_space,
                           device=device)
            DeterministicMixin.__init__(self, clip_actions=False)
            self.net = mlp((self.num_observations + self.num_actions,) + tuple(hidden) + (1,),
                           activation)

        def compute(self, inputs, role=''):
            a = torch.tanh(inputs['taken_actions'])
            return self.net(torch.cat([inputs['observations'], a], dim=-1)), {}

    models = {'policy': SquashedGaussianActor()}
    for k in ('critic_1', 'critic_2', 'target_critic_1', 'target_critic_2'):
        models[k] = SquashedQ()
    for m in models.values():
        m.to(device)
    return models


# 하이퍼파라미터 기본값 — 근거는 README 「SAC」 표 (계약 §9 고정값 + Haarnoja 2018 + 설계 선택)
SAC_DEFAULTS = {
    'discount_factor': 0.995,        # 계약 §9 (γ 0.995)
    'polyak': 0.005,                 # 계약 §9 (τ 0.005), Haarnoja 2018 표 1
    'learning_rate': 3e-4,           # Haarnoja 2018 표 1 (skrl 기본 1e-3)
    'batch_size': 8192,              # 병렬 env 1024 에 맞춘 큰 배치 [설계 선택]
    'gradient_steps': 4,             # env 스텝당 갱신 수 → 재생 비율 B·G/N = 32 [설계 선택]
    'initial_entropy_value': 0.05,   # 스텝 보상 크기(시간 비용 0.02)와 같은 자릿수 [설계 선택, skrl 기본 0.2]
    'target_entropy': -2.0,          # 계약 §9 (−dim A)
    'learning_starts': 50,           # env 스텝 (× N 전이) [설계 선택]
    'grad_norm_clip': 0.0,           # skrl 기본 (자르지 않음), Haarnoja 2018 도 자르지 않음
    'memory_transitions': 1_000_000,  # 재생 버퍼 크기 (Haarnoja 2018 표 1 의 10⁶)
}


def make_agent(obs_space, act_space, num_envs, device, hp=None, hidden=(512, 256, 128),
               activation='elu'):
    """SAC (skrl) 에이전트·재생 버퍼·정규화 → (agent, memory, scaler, cfg dict)."""
    from skrl.agents.torch.sac import SAC, SAC_CFG
    from skrl.memories.torch import RandomMemory
    h = dict(SAC_DEFAULTS)
    h.update(hp or {})
    mem_size = max(1, int(h['memory_transitions'] // num_envs))
    memory = RandomMemory(memory_size=mem_size, num_envs=num_envs, device=device,
                          replacement=True)
    models = build_models(obs_space, act_space, device, hidden, activation)
    n_obs = models['policy'].num_observations
    cfg = SAC_CFG(gradient_steps=int(h['gradient_steps']), batch_size=int(h['batch_size']),
                  discount_factor=float(h['discount_factor']), polyak=float(h['polyak']),
                  learning_rate=float(h['learning_rate']),
                  observation_preprocessor=ContractScaler,
                  observation_preprocessor_kwargs={'size': n_obs, 'device': device},
                  random_timesteps=0, learning_starts=int(h['learning_starts']),
                  grad_norm_clip=float(h['grad_norm_clip']), learn_entropy=True,
                  initial_entropy_value=float(h['initial_entropy_value']),
                  target_entropy=float(h['target_entropy']), mixed_precision=False)
    cfg.experiment.write_interval = 0
    cfg.experiment.checkpoint_interval = 0
    agent = SAC(models=models, memory=memory, observation_space=obs_space, state_space=None,
                action_space=act_space, device=device, cfg=cfg)
    scaler = agent._observation_preprocessor
    h['memory_size_per_env'] = mem_size
    h['hidden'] = list(hidden)
    h['activation'] = activation
    h['replay_ratio'] = h['batch_size'] * h['gradient_steps'] / num_envs
    return agent, memory, scaler, h


def export_state(agent):
    """학습된 SAC 정책 → 체크포인트 dict (policy_export.PolicySAC.from_checkpoint 가 읽는다)."""
    pol = agent.policy
    sc = agent._observation_preprocessor
    return {'format': 'amr_sac_v1',
            'mean': sc.mean.detach().cpu().reshape(-1), 'var': sc.var.detach().cpu().reshape(-1),
            'trunk': {k: v.detach().cpu() for k, v in pol.trunk.state_dict().items()},
            'mu_head': {k: v.detach().cpu() for k, v in pol.mu_head.state_dict().items()},
            'log_std_head': {k: v.detach().cpu()
                             for k, v in pol.log_std_head.state_dict().items()},
            'hidden': list(pol.hidden), 'activation': pol.activation,
            'log_std_min': LOG_STD_MIN, 'log_std_max': LOG_STD_MAX,
            'var_floor': VAR_FLOOR, 'norm_clip': NORM_CLIP}


def full_state(agent):
    """재개·분석용 전체 상태 (critic·target·최적화기·α 포함)."""
    st = {k: v.state_dict() for k, v in agent.models.items()}
    st['scaler'] = agent._observation_preprocessor.state_dict()
    st['log_entropy_coefficient'] = agent.log_entropy_coefficient.detach().cpu()
    st['policy_optimizer'] = agent.policy_optimizer.state_dict()
    st['critic_optimizer'] = agent.critic_optimizer.state_dict()
    st['entropy_optimizer'] = agent.entropy_optimizer.state_dict()
    return st


def next_obs_for_memory(obs, next_obs, terminated, truncated):
    """
    재생 버퍼에 넣을 다음 관측 — Isaac env 는 끝난 env 를 바로 리셋해 next_obs 가 새 에피소드의 첫 관측이다.

    종단(terminated)은 목표값에서 가려지므로 상관없다. 잘림(시간 상한·정체, terminated 아님)은 마지막 상태로
    bootstrap 해야 하는데 그 관측이 없으므로 **현재 관측 s_t 로 대신한다** (V(s_{t+1}) ≈ V(s_t);
    rsl_rl PPO 의 시간 초과 처리 r += γ·V(s_t) 와 같은 근사). 반환 [N, D].
    """
    cut = (truncated.reshape(-1) & ~terminated.reshape(-1)).unsqueeze(-1)
    return torch.where(cut, obs, next_obs)
