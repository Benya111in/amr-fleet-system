"""
비교군 B 정책의 계약형 모듈과 ONNX 내보내기 (rl/contract/onnx_io.md).

torch 만 쓴다 (rsl_rl·Isaac 불필요) — rsl_rl 체크포인트의 state dict 에서 같은 계산을 다시 짠다.
  x = clip((obs − μ)/√(σ² + 1e-8), −10, 10)    (σ² = max(학습 분산, VAR_FLOOR) — 학습과 같은 값)
  mu = actor_mlp(x), action = tanh(mu), log_std = 학습 매개변수 (상태 무관, 배치로 펼침)
"""

import json
import re

import torch
from torch import nn

VAR_FLOOR = 1e-2     # 학습 중 정규화와 같은 분산 하한 (train_ppo.ContractNormalization)
NORM_EPS = 1e-8
NORM_CLIP = 10.0
OPSET = 17


def contract_normalize(x, mean, var):
    """계약 정규화 식."""
    return torch.clamp((x - mean) / torch.sqrt(torch.clamp(var, min=VAR_FLOOR) + NORM_EPS),
                       -NORM_CLIP, NORM_CLIP)


def _mlp_from_state(sd, prefix, activation='elu'):
    keys = sorted({int(m.group(1)) for k in sd for m in [re.match(prefix + r'\.(\d+)\.weight$', k)]
                   if m})
    layers = []
    for n, i in enumerate(keys):
        w = sd[f'{prefix}.{i}.weight']
        lin = nn.Linear(w.shape[1], w.shape[0])
        lin.weight.data.copy_(w)
        lin.bias.data.copy_(sd[f'{prefix}.{i}.bias'])
        layers.append(lin)
        if n < len(keys) - 1:
            layers.append({'elu': nn.ELU(), 'relu': nn.ReLU(), 'tanh': nn.Tanh()}[activation])
    return nn.Sequential(*layers)


class PolicyB(nn.Module):
    """obs [B, 291] → (action [B, 2], mu [B, 2], log_std [B, 2])."""

    def __init__(self, mean, var, actor, log_std):
        """정규화 통계·actor·log_std."""
        super().__init__()
        self.register_buffer('mean', mean.clone().float().reshape(1, -1))
        self.register_buffer('var', torch.clamp(var.clone().float(), min=VAR_FLOOR).reshape(1, -1))
        self.actor = actor
        self.log_std = nn.Parameter(log_std.clone().float().reshape(1, -1), requires_grad=False)

    def forward(self, obs):
        """계약 출력 셋."""
        x = torch.clamp((obs - self.mean) / torch.sqrt(self.var + NORM_EPS), -NORM_CLIP, NORM_CLIP)
        mu = self.actor(x)
        return torch.tanh(mu), mu, self.log_std.expand(obs.shape[0], -1)

    @staticmethod
    def from_rsl_state_dict(sd, activation='elu'):
        """rsl_rl ActorCritic state dict (noise_std_type 'log', 관측 정규화 켬) → PolicyB."""
        if 'log_std' not in sd:
            raise ValueError("noise_std_type 'log' 체크포인트가 아니다 (log_std 없음)")
        return PolicyB(sd['actor_obs_normalizer._mean'].reshape(-1),
                       sd['actor_obs_normalizer._var'].reshape(-1),
                       _mlp_from_state(sd, 'actor', activation), sd['log_std'])


def export_onnx(policy, path, metadata):
    """onnx_io.md 의 이름·모양·dtype·opset 으로 내보내고 metadata_props 를 붙인다."""
    import onnx
    policy = policy.eval().cpu()
    dummy = torch.zeros(1, policy.mean.shape[1])
    torch.onnx.export(policy, (dummy,), path, input_names=['obs'],
                      output_names=['action', 'mu', 'log_std'],
                      dynamic_axes={'obs': {0: 'batch'}, 'action': {0: 'batch'},
                                    'mu': {0: 'batch'}, 'log_std': {0: 'batch'}},
                      opset_version=OPSET, dynamo=False)
    m = onnx.load(path)
    for k, v in metadata.items():
        e = m.metadata_props.add()
        e.key = str(k)
        e.value = v if isinstance(v, str) else json.dumps(v)
    onnx.checker.check_model(m)
    onnx.save(m, path)
    return path
