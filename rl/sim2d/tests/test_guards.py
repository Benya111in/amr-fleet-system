"""
가드 시험 — 변이 검사에서 기존 시험이 놓친 가드를 직접 고정한다.

  - 경로 투영 동률을 torch.compile 아래에서도 첫 인덱스로 (inductor argmax 결함 회피, first_true)
  - 관측 지연: 스캔은 지연만큼 과거 자세에서 레이캐스트
  - 충돌한 env 는 서브스텝에서 더 움직이지 않는다 (접촉 = 즉시 정지·종료)
  - truncation 부트스트랩: 보상에 γ·V(final_obs)·σ_ret 를 더한다
"""

import math

import pytest
import torch

from amr_sim2d import contract_torch as T
from amr_sim2d import kernels as K
from amr_sim2d import raycast as RC
from amr_sim2d.env import EnvConfig, Sim2DEnv
from amr_sim2d.ppo import PPOConfig, Trainer

DEV = 'cuda' if torch.cuda.is_available() else 'cpu'
needs_gpu = pytest.mark.skipif(DEV != 'cuda', reason='GPU 필요')


@needs_gpu
@pytest.mark.slow
def test_projection_ties_compiled_equals_eager():
    env = Sim2DEnv(EnvConfig(num_envs=256, device=DEV, compile=False, n_paths=512,
                             obstacle_mode='none'), log=lambda *a: None)
    env.reset()
    pts, n, cum = env.path_pts, env.path_n, env.path_cum
    # 경로 꼭짓점 그 자체 (두 구간이 같은 거리 — 동률) 와 꼭짓점 옆 법선 방향 점
    g = torch.Generator(device=DEV)
    g.manual_seed(0)
    k = (torch.rand(256, device=DEV, generator=g) * (n - 2).double()).long() + 1
    p = pts[torch.arange(256, device=DEV), k]
    cp = torch.compile(T.project_onto_path, dynamic=False)
    off = torch.tensor([0.0, 0.2], device=DEV, dtype=p.dtype)
    for q in (p, p + off):
        a = T.project_onto_path(pts, n, cum, q)
        b = cp(pts, n, cum, q)
        assert float((a[0] - b[0]).abs().max()) < 1e-9
        assert float((a[3] - b[3]).abs().max()) < 1e-9


@needs_gpu
def test_obs_delay_casts_from_past_pose():
    cfg = EnvConfig(num_envs=32, device=DEV, compile=False, n_paths=512, obstacle_mode='none')
    cfg.dr_ranges = dict(cfg.dr_ranges, obs_delay=(0.1, 0.1), lidar_sigma=(0.0, 0.0),
                         beam_dropout=(0.0, 0.0))
    env = Sim2DEnv(cfg, log=lambda *a: None)
    env.reset()
    u = torch.tensor([[1.0, 0.5]], device=DEV).repeat(32, 1)
    for _ in range(15):
        env.step(u)
    assert bool((env.obs_delay == 5).all())
    assert torch.allclose(env.scan_age, torch.full_like(env.scan_age, 0.1))
    past = env.hist[:, 5]
    r_past = RC.cast_scan(env.grid, env.map_meta, env.map_idx, past)
    r_now = RC.cast_scan(env.grid, env.map_meta, env.map_idx, env.pose)
    same = torch.isfinite(r_past) == torch.isfinite(env.ranges)
    assert bool(same.all())
    fin = torch.isfinite(r_past)
    assert float((r_past - env.ranges)[fin].abs().max()) < 1e-4
    moved = (env.pose[:, :2] - past[:, :2]).norm(dim=-1) > 0.02
    assert bool(moved.any())
    assert not torch.allclose(torch.nan_to_num(r_now[moved], posinf=0.0),
                              torch.nan_to_num(env.ranges[moved], posinf=0.0))


def test_collided_env_does_not_move():
    n = 4
    z = torch.zeros(n, dtype=torch.float64)
    f64 = torch.float64
    S = {'collided': torch.tensor([True, False, True, False]),
         'dbuf': torch.zeros(n, 10, 2, dtype=f64),
         'v': torch.full((n,), 0.8, dtype=f64), 'w': torch.full((n,), 0.3, dtype=f64),
         'pose': torch.tensor([[1.0, 2.0, 0.3]] * n, dtype=f64)}
    X = {'td': z + 0.04, 'tm': z + 0.08, 'acc_scale': z + 1.0}
    out = K.plant_step(S, X, torch.full((n,), 0.8, dtype=torch.float64), z + 0.3)
    assert out['v'].tolist()[0] == 0.0 and out['w'].tolist()[2] == 0.0
    assert torch.equal(out['pose'][0], S['pose'][0]) and torch.equal(out['pose'][2], S['pose'][2])
    assert not torch.equal(out['pose'][1], S['pose'][1])


class _FakeEnv:
    """한 env 가 매 스텝 truncation 되는 가짜 환경 (부트스트랩 확인용)."""

    def __init__(self):
        self.N, self.dev = 2, torch.device(DEV)
        self.cfg = EnvConfig(num_envs=2, device=DEV)
        self.mask = None

    def reset(self):
        return torch.zeros(2, 291, device=self.dev)

    def step(self, u):
        obs = torch.zeros(2, 291, device=self.dev)
        z = torch.zeros(2, dtype=torch.bool, device=self.dev)
        trunc = torch.tensor([True, False], device=self.dev)
        di = torch.tensor([0], device=self.dev)
        info = {'cost': torch.zeros(2, 5, device=self.dev), 'done_idx': di,
                'final_obs': torch.ones(1, 291, device=self.dev),
                'episode': {'success': z[:1], 'cost': torch.zeros(1, 5, device=self.dev)}}
        return obs, torch.zeros(2, device=self.dev), z, trunc, info


def test_truncation_bootstraps_final_value():
    tr = Trainer(_FakeEnv(), PPOConfig(rollout=1, reward_norm=False), seed=0)
    with torch.no_grad():
        tr.model.v[-1].bias.fill_(2.0)
    tr.collect()
    with torch.no_grad():          # collect 안의 정규화 갱신 뒤 상태 = 부트스트랩 때 상태 (롤아웃 1)
        v_final = float(tr.model.value(torch.ones(1, 291, device=DEV))[0])
    assert float(tr.b_rew[0, 0]) == pytest.approx(0.995 * v_final, rel=1e-5)
    assert float(tr.b_rew[0, 1]) == 0.0
    assert float(tr.b_done[0, 0]) == 1.0 and math.isfinite(v_final)
