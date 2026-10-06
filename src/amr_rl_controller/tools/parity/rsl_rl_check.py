#!/usr/bin/env python3
"""
학습 때 쓴 정책 정의(rsl_rl 3.1.2 ActorCritic + 계약 정규화)로 같은 관측의 행동을 낸다 (보너스 1 A2).

amr-ai/isaaclab:2.3.2 의 Isaac Python 에서 돈다 (rsl_rl 은 그 이미지에만 있다, Isaac 앱은 띄우지 않는다):

    /isaac-sim/python.sh rsl_rl_check.py --pt v12_s1/policy_final.pt --obs a.npy b.npy \
        --out rsl_s1.npz [--cuda]

정책 구성은 rl/isaac/scripts/train_ppo.py 와 같게 한다: rl/isaac/amr_isaac/agents/rsl_rl_ppo_cfg.py 의
ActorCritic 인자(actor_obs_normalization 켬, noise_std_type 'log', 은닉 [512, 256, 128] — 체크포인트의
'hidden'·'activation' 을 따른다) → actor_obs_normalizer 를 계약 정규화로 바꾼 클래스로 교체
(train_ppo.ContractNormalization: rsl_rl 통계 갱신은 그대로, forward 만 policy_export.contract_normalize)
→ state dict 적재 → act_inference (= 정규화 → actor) → μ, tanh(μ).
출력 npz: 각 입력 파일 이름(확장자 뺀 것)마다 '<이름>_mu_cpu', '<이름>_act_cpu' (와 --cuda 면 _cuda).
"""

import argparse
import os
import sys

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.environ.get('RL_ISAAC_DIR', os.path.normpath(os.path.join(
    HERE, '..', '..', '..', '..', 'rl', 'isaac')))
sys.path.insert(0, ISAAC)

from amr_isaac import policy_export as PE  # noqa: E402
import numpy as np  # noqa: E402
from rsl_rl.modules import ActorCritic  # noqa: E402
from rsl_rl.networks import EmpiricalNormalization  # noqa: E402
from tensordict import TensorDict  # noqa: E402


class ContractNormalization(EmpiricalNormalization):
    """학습 스크립트와 같은 교체: 적용 식만 계약식 (분산 하한 포함)."""

    def forward(self, x):
        """clip((x − μ)/√(max(σ², floor) + 1e-8), −10, 10)."""
        return PE.contract_normalize(x, self._mean, self._var)


def build_policy(ck, obs_dim):
    """체크포인트 → 학습 때와 같은 ActorCritic (actor 정규화 교체)."""
    hidden = list(ck.get('hidden', [512, 256, 128]))
    dummy = TensorDict({'policy': torch.zeros(1, obs_dim)}, batch_size=[1])
    pol = ActorCritic(dummy, {'policy': ['policy'], 'critic': ['policy']}, 2,
                      actor_obs_normalization=True, critic_obs_normalization=True,
                      actor_hidden_dims=hidden, critic_hidden_dims=hidden,
                      activation=ck.get('activation', 'elu'), init_noise_std=0.6,
                      noise_std_type='log')
    for name in ('actor_obs_normalizer', 'critic_obs_normalizer'):
        old = getattr(pol, name)
        new = ContractNormalization(old._mean.shape[-1], eps=old.eps, until=old.until)
        new.load_state_dict(old.state_dict())
        setattr(pol, name, new)
    pol.load_state_dict(ck['state_dict'], strict=True)   # 키가 하나라도 어긋나면 예외
    return pol.eval()


def main():
    """입력 관측마다 μ·tanh(μ) 를 낸다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--pt', required=True)
    ap.add_argument('--obs', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--cuda', action='store_true')
    a = ap.parse_args()
    ck = torch.load(a.pt, map_location='cpu')
    devices = ['cpu'] + (['cuda'] if a.cuda and torch.cuda.is_available() else [])
    res = {}
    for dev in devices:
        torch.manual_seed(0)
        x0 = np.load(a.obs[0])
        pol = build_policy(ck, x0.shape[1]).to(dev)
        for path in a.obs:
            name = os.path.splitext(os.path.basename(path))[0]
            x = torch.from_numpy(np.load(path).astype(np.float32)).to(dev)
            with torch.inference_mode():
                mu = pol.act_inference(TensorDict({'policy': x}, batch_size=[x.shape[0]]))
            res[f'{name}_mu_{dev}'] = mu.float().cpu().numpy()
            res[f'{name}_act_{dev}'] = torch.tanh(mu).float().cpu().numpy()
    res['torch_version'] = np.array(torch.__version__)
    res['devices'] = np.array(devices)
    if 'cuda' in devices:
        res['tf32_matmul'] = np.array(torch.backends.cuda.matmul.allow_tf32)
        res['cuda_device'] = np.array(torch.cuda.get_device_name(0))
    np.savez(a.out, **res)
    print('rsl_rl_check', a.pt, devices, sorted(k for k in res if k.endswith('_cpu'))[:4])


if __name__ == '__main__':
    main()
