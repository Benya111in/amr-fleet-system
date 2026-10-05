#!/usr/bin/env python3
"""
ONNX 입출력 계약(onnx_io.md)을 지키는 **가짜 정책** 두 개와 기대 출력을 만든다 — r3 플러그인 시험용 고정물.

학습된 정책이 아니다 (가중치는 시드 고정 난수). 이름·모양·dtype·정규화 위치·마스크 처리만 검증한다.
사용: python3 make_fixtures.py [출력 디렉터리]   (torch, onnx, onnxruntime 필요 — amr-ai/rl:0.3)
"""

import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONTRACT = os.path.dirname(HERE)
sys.path.insert(0, CONTRACT)

from amr_rl_contract import constants as C  # noqa: E402
from amr_rl_contract.observation import OBS_DIM  # noqa: E402

OPSET = 17
MASK_FILL = -1e9


def _modules():
    import torch
    from torch import nn

    class Normalize(nn.Module):
        """running mean/std 정규화 (모델 안): clip((x − μ)/√(σ² + 1e-8), ±10)."""

        def __init__(self, mean, var):
            super().__init__()
            self.register_buffer('mean', torch.as_tensor(mean, dtype=torch.float32))
            var_t = torch.as_tensor(var, dtype=torch.float32)
            self.register_buffer('std', torch.sqrt(var_t + 1e-8))

        def forward(self, x):
            return torch.clamp((x - self.mean) / self.std, -10.0, 10.0)

    class PolicyB(nn.Module):
        """비교군 B: obs → (action = tanh(μ), μ, log_std)."""

        def __init__(self, norm):
            super().__init__()
            self.norm = norm
            self.net = nn.Sequential(nn.Linear(OBS_DIM, 64), nn.Tanh(), nn.Linear(64, 2))
            self.log_std = nn.Parameter(torch.tensor([-0.5, -0.7]))

        def forward(self, obs):
            mu = self.net(self.norm(obs))
            return torch.tanh(mu), mu, mu * 0.0 + self.log_std

    class PolicyC(nn.Module):
        """비교군 C: (obs, mask) → (마스크된 logits, argmax)."""

        def __init__(self, norm):
            super().__init__()
            self.norm = norm
            self.net = nn.Sequential(nn.Linear(OBS_DIM + C.N_ACTIONS_C, 64), nn.Tanh(),
                                     nn.Linear(64, C.N_ACTIONS_C))

        def forward(self, obs, mask):
            x = torch.cat([self.norm(obs), mask], dim=1)
            logits = self.net(x) + (mask - 1.0) * (-MASK_FILL)
            return logits, torch.argmax(logits, dim=1)

    return torch, Normalize, PolicyB, PolicyC


def build_models():
    """시드 고정 가짜 정책 두 개."""
    torch, Normalize, PolicyB, PolicyC = _modules()
    torch.manual_seed(20261006)
    rng = np.random.default_rng(20261006)
    mean = rng.normal(0.0, 0.1, OBS_DIM).astype(np.float32)
    var = rng.uniform(0.5, 2.0, OBS_DIM).astype(np.float32)
    return PolicyB(Normalize(mean, var)).eval(), PolicyC(Normalize(mean, var)).eval()


def _meta(model_path, arm, extra):
    import onnx
    m = onnx.load(model_path)
    props = {'contract_version': C.CONTRACT_VERSION, 'arm': arm, 'obs_dim': str(OBS_DIM),
             'dt': str(C.DT), 'normalization': 'inside', 'fixture': 'true'}
    props.update(extra)
    for k, v in props.items():
        e = m.metadata_props.add()
        e.key, e.value = k, v
    onnx.save(m, model_path)


def export(out_dir):
    """두 모델을 ONNX 로 내보낸다."""
    torch = _modules()[0]
    pb, pc = build_models()
    b_path = os.path.join(out_dir, 'fixture_b.onnx')
    c_path = os.path.join(out_dir, 'fixture_c.onnx')
    obs = torch.zeros(1, OBS_DIM)
    mask = torch.ones(1, C.N_ACTIONS_C)
    with torch.no_grad():
        torch.onnx.export(pb, (obs,), b_path, input_names=['obs'],
                          output_names=['action', 'mu', 'log_std'], opset_version=OPSET,
                          dynamo=False, dynamic_axes={'obs': {0: 'batch'}, 'action': {0: 'batch'},
                                                      'mu': {0: 'batch'}, 'log_std': {0: 'batch'}})
        torch.onnx.export(pc, (obs, mask), c_path, input_names=['obs', 'mask'],
                          output_names=['logits', 'action'], opset_version=OPSET, dynamo=False,
                          dynamic_axes={'obs': {0: 'batch'}, 'mask': {0: 'batch'},
                                        'logits': {0: 'batch'}, 'action': {0: 'batch'}})
    _meta(b_path, 'B', {'act_dim': '2', 'action_scale': f'{C.A_MAX},{C.ALPHA_MAX}'})
    _meta(c_path, 'C', {'n_actions': str(C.N_ACTIONS_C), 'fallback_index': str(C.FALLBACK_INDEX)})
    return b_path, c_path


def golden_inputs():
    """골든 decision 사례의 (이름, obs, mask)."""
    out = []
    for f in sorted(glob.glob(os.path.join(CONTRACT, 'golden', 'd*.json'))):
        with open(f, encoding='utf-8') as fh:
            d = json.load(fh)
        out.append((d['name'], np.array(d['expected']['obs'], dtype=np.float32),
                    np.array(d['expected']['mask'], dtype=np.float32)))
    return out


def run_ort(b_path, c_path, inputs):
    """기대 출력 계산 (onnxruntime CPU)."""
    import onnxruntime as ort
    sb = ort.InferenceSession(b_path, providers=['CPUExecutionProvider'])
    sc = ort.InferenceSession(c_path, providers=['CPUExecutionProvider'])
    rows = []
    for name, obs, mask in inputs:
        a, mu, ls = sb.run(None, {'obs': obs[None]})
        logits, act = sc.run(None, {'obs': obs[None], 'mask': mask[None]})
        rows.append({'name': name,
                     'b_action': [round(float(v), 7) for v in a[0]],
                     'b_mu': [round(float(v), 7) for v in mu[0]],
                     'b_log_std': [round(float(v), 7) for v in ls[0]],
                     'c_action': int(act[0]),
                     'c_logits_valid': [round(float(v), 6) if mask[i] > 0.5 else None
                                        for i, v in enumerate(logits[0])]})
    return rows


def main(out_dir):
    """모델·기대 출력 생성."""
    b_path, c_path = export(out_dir)
    rows = run_ort(b_path, c_path, golden_inputs())
    with open(os.path.join(out_dir, 'fixture_expected.json'), 'w', encoding='utf-8') as f:
        json.dump({'contract_version': C.CONTRACT_VERSION, 'tolerance': 1e-5, 'rows': rows}, f,
                  ensure_ascii=False, indent=1)
        f.write('\n')
    print(f'{b_path}, {c_path}, {len(rows)} 행')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'fixtures'))
