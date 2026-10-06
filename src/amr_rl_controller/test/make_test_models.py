#!/usr/bin/env python3
"""
플러그인 시험용 ONNX 모델을 만든다 (학습된 정책이 아니다 — 가중치는 시드 고정 난수).

사용: make_test_models.py <출력 디렉터리>   (torch·onnx·onnxruntime 필요 — amr-ai/rl:0.3)

- fixture_b.onnx·fixture_c.onnx: r0 계약의 고정물 생성기(rl/contract/onnx/make_fixtures.py)를
  그대로 불러 만든다 → 기대 출력은 커밋된 rl/contract/onnx/fixtures/fixture_expected.json.
- fixture_c_gru.onnx: 같은 입출력 + h_in/h_out (GRU 1 층, hidden 16) — 순환 경로 시험.
- 결함 모델: 주 버전 2·0, obs_dim 290, 메타데이터 없음, arm 메타데이터 불일치, NaN logits —
  configure 거부·폴백 시험. 부 버전만 다른 모델(1.1.0·1.9.0)은 받아야 한다.
- 크기 대표 모델 (지연 측정용, --sizes 일 때만): size_c_mlp256 (MLP 391→256→256→100),
  size_c_cnn_gru128 (섹터 180 Conv1d 2 층 + MLP + GRU 128), size_b_mlp256 (291→256→256→2).
  학습 정책이 아직 없어서 실무 보고서·계약이 권하는 구조의 크기만 흉내 낸다.
- 손으로 만든 배관 시험 정책 (학습 아님): heuristic_c·heuristic_b — 관측의 subgoal 2
  (1 m 앞) 방위로 목표 (v, ω) 를 정하고 가속도 공간 P 제어로 고른다. 폐루프 스모크와 플러그인 시험
  (정책 → DWA 전환)에서 플러그인·마스크·적분 배관을 보려는 것이지 RL 성능과 무관하다.
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
            ('bad_major0_c.onnx', _base_meta('C', contract_version='0.9.0')),
            # 같은 주 버전의 다른 부 버전은 받는다 (onnx_io.md — 1.1.0 정책도 입출력이 같다)
            ('minor_110_c.onnx', _base_meta('C', contract_version='1.1.0')),
            ('minor_190_c.onnx', _base_meta('C', contract_version='1.9.0')),
            ('bad_obsdim_c.onnx', _base_meta('C', obs_dim='290')),
            ('no_meta_c.onnx', {}),
            # 입출력은 C 인데 메타데이터 arm 이 B — 메타데이터 검사만 가를 수 있다
            ('arm_b_meta_c_io.onnx', _base_meta('B'))):
        path = os.path.join(out_dir, name)
        onnx.save(m, path)
        _meta(path, props)


def _sizes(out_dir):
    import torch
    from torch import nn

    n_sec = C.N_SECTORS

    class MlpC(nn.Module):
        def __init__(self):
            super().__init__()
            self.net = nn.Sequential(nn.Linear(OBS_DIM + C.N_ACTIONS_C, 256), nn.Tanh(),
                                     nn.Linear(256, 256), nn.Tanh(),
                                     nn.Linear(256, C.N_ACTIONS_C))

        def forward(self, obs, mask):
            logits = self.net(torch.cat([obs, mask], dim=1)) + (mask - 1.0) * 1e9
            return logits, torch.argmax(logits, dim=1)

    class MlpB(nn.Module):
        def __init__(self):
            super().__init__()
            self.net = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(), nn.Linear(256, 256),
                                     nn.Tanh(), nn.Linear(256, 2))
            self.log_std = nn.Parameter(torch.tensor([-0.5, -0.5]))

        def forward(self, obs):
            mu = self.net(obs)
            return torch.tanh(mu), mu, mu * 0.0 + self.log_std

    class CnnGruC(nn.Module):
        def __init__(self, hidden=128):
            super().__init__()
            self.conv = nn.Sequential(nn.Conv1d(1, 16, 5, stride=2, padding=2), nn.ReLU(),
                                      nn.Conv1d(16, 32, 5, stride=2, padding=2), nn.ReLU())
            self.fc = nn.Linear(32 * 45 + OBS_DIM - n_sec + C.N_ACTIONS_C, 256)
            self.gru = nn.GRU(256, hidden)
            self.head = nn.Linear(hidden, C.N_ACTIONS_C)

        def forward(self, obs, mask, h_in):
            z = self.conv(obs[:, :n_sec].unsqueeze(1)).flatten(1)
            x = torch.relu(self.fc(torch.cat([z, obs[:, n_sec:], mask], dim=1))).unsqueeze(0)
            y, h_out = self.gru(x, h_in)
            logits = self.head(y[0]) + (mask - 1.0) * 1e9
            return logits, torch.argmax(logits, dim=1), h_out

    torch.manual_seed(11)
    obs = torch.zeros(1, OBS_DIM)
    mask = torch.ones(1, C.N_ACTIONS_C)
    dyn = {'obs': {0: 'batch'}, 'mask': {0: 'batch'}, 'logits': {0: 'batch'},
           'action': {0: 'batch'}}
    with torch.no_grad():
        p = os.path.join(out_dir, 'size_c_mlp256.onnx')
        torch.onnx.export(MlpC().eval(), (obs, mask), p, input_names=['obs', 'mask'],
                          output_names=['logits', 'action'], opset_version=OPSET, dynamo=False,
                          dynamic_axes=dyn)
        _meta(p, _base_meta('C'))
        p = os.path.join(out_dir, 'size_b_mlp256.onnx')
        torch.onnx.export(MlpB().eval(), (obs,), p, input_names=['obs'],
                          output_names=['action', 'mu', 'log_std'], opset_version=OPSET,
                          dynamo=False, dynamic_axes={'obs': {0: 'batch'},
                                                      'action': {0: 'batch'}})
        _meta(p, _base_meta('B'))
        p = os.path.join(out_dir, 'size_c_cnn_gru128.onnx')
        h = torch.zeros(1, 1, 128)
        torch.onnx.export(CnnGruC().eval(), (obs, mask, h), p,
                          input_names=['obs', 'mask', 'h_in'],
                          output_names=['logits', 'action', 'h_out'], opset_version=OPSET,
                          dynamo=False, dynamic_axes=dict(dyn, h_in={1: 'batch'},
                                                          h_out={1: 'batch'}))
        _meta(p, _base_meta('C', rnn_layers=1, rnn_hidden=128))


def _heuristic(out_dir):
    import torch
    from torch import nn

    av = [C.LATTICE_AV[i // len(C.LATTICE_AW)] for i in range(C.N_CELLS)] + [-1.0]
    aw = [C.LATTICE_AW[i % len(C.LATTICE_AW)] for i in range(C.N_CELLS)] + [0.0]
    ego = 180          # observation.OBS_LAYOUT: lidar 180 → ego
    path = 195         # → timing 3 → path

    def targets(obs):
        v_cmd = obs[:, ego + 4] * C.V_MAX_HW
        w_cmd = obs[:, ego + 5] * C.W_MAX
        sin_b = obs[:, path + 4]     # subgoal 2 (1.0 m 앞) sin 방위
        cos_b = obs[:, path + 5]
        goal_d = obs[:, 213] * 10.0  # 남은 호길이
        v_des = torch.clamp(0.6 * cos_b, min=0.0) * torch.clamp(goal_d / 1.0, max=1.0)
        w_des = torch.clamp(1.2 * torch.atan2(sin_b, cos_b), -1.0, 1.0)
        a_v = torch.clamp(3.0 * (v_des - v_cmd), -C.A_MAX, C.A_MAX)
        a_w = torch.clamp(4.0 * (w_des - w_cmd), -C.ALPHA_MAX, C.ALPHA_MAX)
        return a_v, a_w

    class HeuristicC(nn.Module):
        """가속도 공간 P 제어 목표에 가까운 셀일수록 큰 logit (마스크는 모델 안에서)."""

        def __init__(self):
            super().__init__()
            self.register_buffer('av', torch.tensor(av))
            self.register_buffer('aw', torch.tensor(aw))

        def forward(self, obs, mask):
            a_v, a_w = targets(obs)
            d = (self.av[None] - a_v[:, None]) ** 2 + ((self.aw[None] - a_w[:, None]) / 2.0) ** 2
            logits = -10.0 * d + (mask - 1.0) * 1e9
            return logits, torch.argmax(logits, dim=1)

    class HeuristicB(nn.Module):
        def forward(self, obs):
            a_v, a_w = targets(obs)
            mu = torch.stack([a_v / C.A_MAX, a_w / C.ALPHA_MAX], dim=1)
            return torch.clamp(mu, -1.0, 1.0), mu, mu * 0.0 - 1.0

    obs = torch.zeros(1, OBS_DIM)
    mask = torch.ones(1, C.N_ACTIONS_C)
    with torch.no_grad():
        p = os.path.join(out_dir, 'heuristic_c.onnx')
        torch.onnx.export(HeuristicC().eval(), (obs, mask), p, input_names=['obs', 'mask'],
                          output_names=['logits', 'action'], opset_version=OPSET, dynamo=False,
                          dynamic_axes={'obs': {0: 'batch'}, 'mask': {0: 'batch'},
                                        'logits': {0: 'batch'}, 'action': {0: 'batch'}})
        _meta(p, _base_meta('C', algo='heuristic-not-rl'))
        p = os.path.join(out_dir, 'heuristic_b.onnx')
        torch.onnx.export(HeuristicB().eval(), (obs,), p, input_names=['obs'],
                          output_names=['action', 'mu', 'log_std'], opset_version=OPSET,
                          dynamo=False, dynamic_axes={'obs': {0: 'batch'},
                                                      'action': {0: 'batch'}})
        _meta(p, _base_meta('B', algo='heuristic-not-rl'))


def main(out_dir, sizes=False):
    """모든 시험 모델을 만든다."""
    os.makedirs(out_dir, exist_ok=True)
    b_path, c_path = mf.export(out_dir)
    _gru_and_nan(out_dir)
    _defects(out_dir, c_path)
    _heuristic(out_dir)
    if sizes:
        _sizes(out_dir)
    print('models:', ' '.join(sorted(f for f in os.listdir(out_dir) if f.endswith('.onnx'))))


if __name__ == '__main__':
    argv = [a for a in sys.argv[1:] if a != '--sizes']
    if len(argv) != 1:
        sys.exit('사용: make_test_models.py <출력 디렉터리> [--sizes]')
    main(argv[0], sizes='--sizes' in sys.argv[1:])
