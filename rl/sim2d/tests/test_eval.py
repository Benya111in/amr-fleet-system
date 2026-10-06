"""
평가 추정량 — env 마다 첫 에피소드 하나만 센다 (먼저 끝난 에피소드 k 개를 세면 일찍 끝나는 결말 쪽으로 치우친다).

2026-10-06 리뷰: 예전 evaluate 는 env 512 개를 굴려 끝난 에피소드가 500 개가 되면 멈췄다. 초반 충돌·짧은 경로가
과대 표집되어 B_bc3 성공이 0.408 로 나왔고, env 마다 첫 에피소드로 세면 0.532 였다.
"""

from types import SimpleNamespace

import pytest
import torch

from amr_sim2d.env import EnvConfig, Sim2DEnv
from amr_sim2d.heuristic import path_follower
from amr_sim2d.ppo import evaluate

DEV = 'cuda' if torch.cuda.is_available() else 'cpu'


class _TimedEnv:
    """env i 의 에피소드가 늘 lengths[i] 스텝에 끝나는 가짜 env (결말 = success[i])."""

    def __init__(self, lengths, success):
        self.N = len(lengths)
        self.L = torch.tensor(lengths)
        self.succ = torch.tensor(success)
        self.cfg = SimpleNamespace(arm='B')
        self.mask = None
        self.gen = torch.Generator()
        self.k = torch.zeros(self.N, dtype=torch.long)
        self.t_limit = self.L.double() * 0.1
        self.resets = 0

    def reset(self):
        self.k.zero_()
        self.resets += 1
        return torch.zeros(self.N, 291)

    def step(self, u):
        self.k += 1
        done = self.k >= self.L
        z = torch.zeros(self.N, dtype=torch.bool)
        info = {}
        di = torch.nonzero(done)[:, 0]
        if di.numel():
            n = di.numel()
            s = self.succ[di]
            info['done_idx'] = di
            info['episode'] = {
                'success': s, 'collision': ~s,
                'collision_moving': torch.zeros(n, dtype=torch.bool),
                'stalled': torch.zeros(n, dtype=torch.bool),
                'timeout': torch.zeros(n, dtype=torch.bool),
                'return': torch.zeros(n, dtype=torch.float64), 'length': self.k[di].clone(),
                'path_len': self.L[di].double(), 'time': self.k[di].double() * 0.1,
                'cost': torch.zeros(n, 5, dtype=torch.float64),
                'fallback': torch.zeros(n, dtype=torch.long)}
            self.k[di] = 0
        return torch.zeros(self.N, 291), torch.zeros(self.N), done, z, info


class _Zero:
    """행동 0 정책."""

    def act(self, obs, mask=None, deterministic=True):
        a = torch.zeros(obs.shape[0], 2, device=obs.device)
        return a, a, torch.zeros(obs.shape[0], device=obs.device)


class _Heuristic:
    """경로 추종 휴리스틱을 정책 인터페이스로."""

    def act(self, obs, mask=None, deterministic=True):
        u = path_follower(obs)
        return u, u, torch.zeros(obs.shape[0], device=obs.device)


def test_first_episode_per_env_is_not_biased_to_short_episodes():
    # 짧게(2 스텝) 끝나는 실패 4 개, 길게(30 스텝) 끝나는 성공 4 개 → 참 성공률 0.5.
    # 먼저 끝난 8 개를 세면 실패만 8 개(성공 0)가 된다.
    env = _TimedEnv([2] * 4 + [30] * 4, [False] * 4 + [True] * 4)
    s = evaluate(_Zero(), env, 8)
    assert s['episodes'] == 8 and s['unfinished'] == 0
    assert s['success'] == pytest.approx(0.5)
    assert s['length'] == pytest.approx(16.0)
    assert s['estimator'] == 'first_episode_per_env'
    assert s['success_se'] == pytest.approx((0.25 / 8) ** 0.5)


def test_rounds_when_more_episodes_than_envs():
    env = _TimedEnv([2] * 4 + [30] * 4, [False] * 4 + [True] * 4)
    s = evaluate(_Zero(), env, 20)
    # 라운드 3: 8 + 8 + 앞쪽 env 4 개 (가짜 env 는 인덱스로 결말이 정해져 있어 마지막 라운드는 실패 4 개)
    assert s['episodes'] == 20 and s['rounds'] == 3 and env.resets == 3
    assert s['success'] == pytest.approx(8 / 20)


def test_unfinished_envs_are_reported():
    env = _TimedEnv([2, 1000], [True, True])
    s = evaluate(_Zero(), env, 2, max_steps=10)
    assert s['episodes'] == 1 and s['unfinished'] == 1


@pytest.mark.skipif(DEV != 'cuda', reason='GPU 필요')
def test_seeded_eval_counts_the_episodes_drawn_at_reset():
    """같은 seed 면 같은 결과, 센 에피소드의 경로 길이 = 리셋 때 뽑힌 경로 길이 (선택 편향 없음)."""
    env = Sim2DEnv(EnvConfig(num_envs=48, device=DEV, compile=False, n_paths=512,
                             obstacle_mode='mixed', n_obstacles=(0, 4), path_len=(3.0, 8.0),
                             seed=31), log=lambda *a: None)
    env.gen.manual_seed(777)
    env.reset()
    drawn = float(env.path_len.mean())
    a = evaluate(_Heuristic(), env, 48, seed=777)
    b = evaluate(_Heuristic(), env, 48, seed=777)
    assert a['episodes'] == 48 and a['unfinished'] == 0
    assert a['path_len'] == pytest.approx(drawn, abs=1e-9)
    for k in ('success', 'collision', 'stalled', 'timeout', 'path_len', 'length'):
        assert a[k] == b[k], k
