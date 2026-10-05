#!/usr/bin/env python3
"""
플러그인 시험용 ONNX 모델을 만든다 (학습된 정책이 아니다 — 가중치는 시드 고정 난수).

사용: make_test_models.py <출력 디렉터리>   (torch·onnx·onnxruntime 필요 — amr-ai/rl:0.3)

- fixture_b.onnx·fixture_c.onnx: r0 계약의 고정물 생성기(rl/contract/onnx/make_fixtures.py)를
  그대로 불러 만든다 → 기대 출력은 커밋된 rl/contract/onnx/fixtures/fixture_expected.json.
- fixture_c_gru.onnx: 같은 입출력 + h_in/h_out (GRU 1 층, hidden 16) — 순환 경로 시험.
- 결함 모델: 주 버전 2, obs_dim 290, 메타데이터 없음, NaN logits — configure 거부·폴백 시험.
저장소 .gitignore 가 *.onnx 를 무시하므로 모델은 커밋하지 않고 시험 때 만든다.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONTRACT = os.environ.get('RL_CONTRACT_DIR',
                          os.path.normpath(os.path.join(HERE, '..', '..', '..', 'rl', 'contract')))
sys.path.insert(0, CONTRACT)
sys.path.insert(0, os.path.join(CONTRACT, 'onnx'))

import make_fixtures as mf  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402
from amr_rl_contract.observation import OBS_DIM  # noqa: E402

OPSET = 17


def _meta(path, props):
    import onnx
    m = onnx.load(path)
    del m.metadata_props[:]
    for k, v in props.items():
        e = m.metadata_props.add()
        e.key, e.value = k, str(v)
    onnx.save(m, path)


def _base_meta(arm, **extra):
    props = {'contract_version': C.CONTRACT_VERSION, 'arm': arm, 'obs_dim': str(OBS_DIM),
             'dt': str(C.DT), 'normalization': 'inside', 'fixture': 'true'}
    props.update(extra)
    return props


def _gru_and_nan(out_dir):
    import torch
    from torch import nn

    class PolicyCGru(nn.Module):
        """비교군 C + GRU: (obs, mask, h_in) → (logits, action, h_out)."""

        def __init__(self, hidden=16):
            super().__init__()
            self.inp = nn.Linear(OBS_DIM + C.N_ACTIONS_C, 32)
            self.gru = nn.GRU(32, hidden, num_layers=1)
            self.head = nn.Linear(hidden, C.N_ACTIONS_C)

        def forward(self, obs, mask, h_in):
            x = torch.tanh(self.inp(torch.cat([obs, mask], dim=1))).unsqueeze(0)
            y, h_out = self.gru(x, h_in)
            logits = self.head(y[0]) + (mask - 1.0) * 1e9
            return logits, torch.argmax(logits, dim=1), h_out

    class PolicyCNan(nn.Module):
        """유효 셀 logit 이 NaN 인 결함 모델."""

        def forward(self, obs, mask):
            logits = obs[:, :1] * 0.0 + mask * float('nan')
            return logits, torch.argmax(mask, dim=1)

    torch.manual_seed(7)
    obs = torch.zeros(1, OBS_DIM)
    mask = torch.ones(1, C.N_ACTIONS_C)
    h = torch.zeros(1, 1, 16)
    gru_path = os.path.join(out_dir, 'fixture_c_gru.onnx')
    nan_path = os.path.join(out_dir, 'nan_c.onnx')
    with torch.no_grad():
        torch.onnx.export(PolicyCGru().eval(), (obs, mask, h), gru_path,
                          input_names=['obs', 'mask', 'h_in'],
                          output_names=['logits', 'action', 'h_out'], opset_version=OPSET,
                          dynamo=False,
                          dynamic_axes={'obs': {0: 'batch'}, 'mask': {0: 'batch'},
                                        'h_in': {1: 'batch'}, 'logits': {0: 'batch'},
                                        'action': {0: 'batch'}, 'h_out': {1: 'batch'}})
        torch.onnx.export(PolicyCNan().eval(), (obs, mask), nan_path,
                          input_names=['obs', 'mask'], output_names=['logits', 'action'],
                          opset_version=OPSET, dynamo=False,
                          dynamic_axes={'obs': {0: 'batch'}, 'mask': {0: 'batch'},
                                        'logits': {0: 'batch'}, 'action': {0: 'batch'}})
    _meta(gru_path, _base_meta('C', rnn_layers=1, rnn_hidden=16))
    _meta(nan_path, _base_meta('C'))


def _defects(out_dir, c_path):
    import onnx
    m = onnx.load(c_path)
    for name, props in (
            ('bad_major_c.onnx', _base_meta('C', contract_version='2.0.0')),
            ('bad_obsdim_c.onnx', _base_meta('C', obs_dim='290')),
            ('no_meta_c.onnx', {})):
        path = os.path.join(out_dir, name)
        onnx.save(m, path)
        _meta(path, props)


def main(out_dir):
    """모든 시험 모델을 만든다."""
    os.makedirs(out_dir, exist_ok=True)
    b_path, c_path = mf.export(out_dir)
    _gru_and_nan(out_dir)
    _defects(out_dir, c_path)
    print('models:', ' '.join(sorted(f for f in os.listdir(out_dir) if f.endswith('.onnx'))))


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('사용: make_test_models.py <출력 디렉터리>')
    main(sys.argv[1])
