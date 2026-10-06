"""
train_sac.py 마른 실행 — Isaac 없이 가짜 env 로 스크립트 전체(인자·에이전트 조립·학습 루프·기록·내보내기)를 돈다.

Isaac 스모크 1 회차(B0_smoke_sac_s0)가 시작하자마자 `C.OBS_DIM` (없는 이름) 으로 멈췄다 — 그런 오류를 GPU 슬롯을 쓰기 전에
잡으려고 둔다. isaaclab·amr_isaac.env·env_cfg 를 가짜 모듈로 바꿔 끼우고 os._exit 를 예외로 바꾼다.
"""

import importlib.util
import json
import os
import runpy
import sys
import types

import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(os.path.dirname(HERE), 'scripts', 'train_sac.py')
OUTCOMES = ('success', 'collision', 'time_limit', 'stall')
DIAG = ('progress', 'near_goal_0p5', 'spin_frac', 'offpath_1m_frac')


class FakeEnv:
    """계약 관측 291·행동 2 의 가짜 vec env (보상 = −|tanh(a) − 0.5|, 10 스텝마다 성공 종단)."""

    def __init__(self, cfg):
        self.num_envs = cfg.scene.num_envs
        self.device = 'cpu'
        self.pool_size = 7
        self.reward_spec = 'fake'
        self.t = torch.zeros(self.num_envs, dtype=torch.long)
        self.eps = 0

    def reset(self, seed=None):
        return {'policy': torch.randn(self.num_envs, 291)}, {}

    def step(self, a):
        assert a.shape == (self.num_envs, 2)
        self.t += 1
        term = self.t % 10 == 0
        self.eps += int(term.sum())
        r = -(torch.tanh(a) - 0.5).abs().sum(-1)
        return ({'policy': torch.randn(self.num_envs, 291)}, r, term,
                torch.zeros_like(term), {'time_outs': torch.zeros_like(term)})

    def pop_stats(self):
        out = {'episodes': self.eps, 'success': self.eps, 'collision': 0, 'time_limit': 0,
               'stall': 0, 'return_sum': 1.0, 'len_steps_sum': 10.0 * self.eps,
               'path_len_sum': 5.0 * self.eps}
        out.update({f'cost_{k}': 0.0 for k in ('collision', 'intervention', 'deviation_1m',
                                               'return_5s', 'cte_track')})
        out.update({f'{k}_sum': 0.0 for k in DIAG})
        self.eps = 0
        return out

    def dr_snapshot(self):
        return {'fake': {'min': 0.0, 'max': 0.0, 'mean': 0.0}}


def _stub(monkeypatch):
    app = types.ModuleType('isaaclab.app')

    class AppLauncher:
        def __init__(self, args):
            self.app = None

        @staticmethod
        def add_app_launcher_args(ap):
            return None
    app.AppLauncher = AppLauncher
    monkeypatch.setitem(sys.modules, 'isaaclab', types.ModuleType('isaaclab'))
    monkeypatch.setitem(sys.modules, 'isaaclab.app', app)
    env = types.ModuleType('amr_isaac.env')
    env.AmrNavEnv, env.DIAG_NAMES, env.OUTCOMES = FakeEnv, DIAG, OUTCOMES
    monkeypatch.setitem(sys.modules, 'amr_isaac.env', env)
    cfgm = types.ModuleType('amr_isaac.env_cfg')

    class _NS:
        pass

    class AmrNavEnvCfg:
        def __init__(self):
            self.scene, self.dr = _NS(), _NS()
            self.scene.num_envs, self.dr.enable = 1, True
    cfgm.AmrNavEnvCfg = AmrNavEnvCfg
    monkeypatch.setitem(sys.modules, 'amr_isaac.env_cfg', cfgm)

    def fake_exit(rc):
        raise SystemExit(rc)
    monkeypatch.setattr(os, '_exit', fake_exit)


@pytest.mark.skipif(importlib.util.find_spec('skrl') is None, reason='skrl 없음')
def test_train_sac_dry_run(tmp_path, monkeypatch):
    """스크립트 전체가 rc 0 으로 끝나고 curve·summary·policy_final(PolicySAC 로 읽힘)·dr_snapshot 을 남긴다."""
    _stub(monkeypatch)
    monkeypatch.setattr(sys, 'argv', [
        SCRIPT, '--num_envs', '8', '--env_steps', '640', '--chunk_steps', '20', '--run', 'dry',
        '--root', str(tmp_path), '--batch_size', '32', '--gradient_steps', '1',
        '--learning_starts', '5', '--memory', '4000', '--hidden', '16,16',
        '--save_at_env_steps', '300'])
    with pytest.raises(SystemExit) as ex:
        runpy.run_path(SCRIPT, run_name='__main__')
    assert ex.value.code == 0
    out = tmp_path / 'dry'
    rows = [json.loads(x) for x in (out / 'curve.jsonl').read_text().splitlines()]
    assert len(rows) == 4 and rows[-1]['env_steps'] == 640 and 'alpha' in rows[-1]
    summ = json.loads((out / 'summary.json').read_text())
    assert summ['algo'].startswith('SAC') and summ['policy_export_mu_max_abs_diff'] < 1e-5
    assert summ['checkpoints'] and (out / summ['checkpoints'][0]['file']).exists()
    from amr_isaac import policy_export as PE
    assert isinstance(PE.load_policy(str(out / 'policy_final.pt')), PE.PolicySAC)
    assert (out / 'dr_snapshot.json').exists() and (out / 'agent_final.pt').exists()
