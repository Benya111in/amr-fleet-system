"""PPO 한 번 갱신 + ONNX 내보내기 + onnxruntime 왕복 (B·C), 정규화가 모델 안에 있는가."""

import os
import sys

import pytest
import torch

from amr_sim2d.env import EnvConfig, Sim2DEnv
from amr_sim2d.ppo import ActorCriticB, PPOConfig, Trainer, export_onnx

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'tools'))
from onnx_check import roundtrip  # noqa: E402

DEV = 'cuda' if torch.cuda.is_available() else 'cpu'
pytestmark = pytest.mark.skipif(DEV != 'cuda', reason='GPU 필요')


@pytest.mark.parametrize('arm', ['B', 'C'])
def test_one_update_and_onnx_roundtrip(arm, tmp_path):
    env = Sim2DEnv(EnvConfig(num_envs=64, device=DEV, arm=arm, compile=False, n_paths=512,
                             path_len=(3.0, 12.0)), log=lambda *a: None)
    tr = Trainer(env, PPOConfig(rollout=16, epochs=2, minibatches=2), seed=0)
    p0 = [p.detach().clone() for p in tr.model.parameters()]
    tr.collect()
    st = tr.update()
    assert all(map(lambda x: x == x, [st['pi_loss'], st['v_loss'], st['kl']]))   # NaN 아님
    assert any(not torch.equal(a, b) for a, b in zip(p0, tr.model.parameters()))
    assert float(tr.model.norm.count) > 64 * 16 * 0.99          # 관측 정규화가 갱신됐다
    path = str(tmp_path / f'policy_{arm}.onnx')
    props = export_onnx(tr.model, path, {'algo': 'ppo', 'seed': 0})
    assert props['arm'] == arm and props['normalization'] == 'inside'
    chk = roundtrip(tr.model, path, env)
    assert chk['pass'], chk
    exp_out = ['action', 'mu', 'log_std'] if arm == 'B' else ['logits', 'action']
    assert chk['outputs'] == exp_out
    assert chk['opset'] == 17


def test_normalization_inside_model():
    """같은 가중치라도 정규화 통계가 다르면 ONNX 출력이 달라야 한다 (= 통계가 모델 안에 있다)."""
    m = ActorCriticB()
    torch.nn.init.normal_(m.pi[-1].weight, std=0.5)
    x = torch.randn(32, 291)
    a0 = m(x)[1].detach().clone()
    m.norm.update(x * 3 + 1)
    a1 = m(x)[1].detach()
    assert not torch.allclose(a0, a1)
