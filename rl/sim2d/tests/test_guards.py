"""
가드 시험 — 변이 검사에서 기존 시험이 놓친 가드를 직접 고정한다.

  - 경로 투영 동률을 torch.compile 아래에서도 첫 인덱스로 (inductor argmax 결함 회피, first_true)
  - 관측 지연: 스캔은 지연만큼 과거 자세에서 레이캐스트
  - 충돌한 env 는 서브스텝에서 더 움직이지 않는다 (접촉 = 즉시 정지·종료)
  - truncation 부트스트랩: 보상에 γ·V(final_obs)·σ_ret 를 더한다
  - safety STOP 확정: 새 스캔(첫 서브스텝) 2 번 연속 D ≤ 0.30 이어야 proximity STOP, D > 0.50 이면 해제
"""

import math

import pytest
import torch

from amr_sim2d import contract_torch as T
from amr_sim2d import geometry2d as G
from amr_sim2d import kernels as K
from amr_sim2d import raycast as RC
from amr_sim2d._contract import C
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


def test_running_norm_variance_floor():
    """늘 같던 특징이 나중에 바뀌어도 정규화 값이 ±10 에 붙지 않는다 (분산 하한 0.01)."""
    from amr_sim2d.ppo import RunningNorm
    nm = RunningNorm(3)
    for _ in range(50):
        nm.update(torch.zeros(256, 3))
    assert float(nm.var.max()) < 1e-8
    z = nm(torch.full((1, 3), 0.5))
    # 하한이 있으면 0.5/0.1 = 5, 없으면 ±10 에 붙는다
    assert float(z.abs().max()) == pytest.approx(0.5 / math.sqrt(0.01 + 1e-8), rel=1e-4)


def _tick_inputs(px):
    """전방 점 하나 (로봇 좌표 x = px, y = 0) 와 0.3 m/s 직진 명령·측정."""
    f64 = torch.float64
    S = {'v': torch.tensor([0.3], dtype=f64), 'w': torch.tensor([0.0], dtype=f64),
         'intent': torch.zeros(1, 2, dtype=f64), 'have_intent': torch.tensor([False]),
         'stop_count': torch.tensor([0]), 'prox': torch.tensor([False])}
    X = {'scan_px': torch.tensor([[px, 5.0, 0.0, 0.0]]),
         'scan_py': torch.tensor([[0.0, 3.0, 0.0, 0.0]]),
         'scan_ok': torch.tensor([[True, True, False, False]]),
         'min_ttc': torch.tensor([math.inf], dtype=f64)}
    return S, X, torch.tensor([0.3], dtype=f64), torch.tensor([0.0], dtype=f64)


def _tick(S, X, v, w, first):
    out, upd = K.safety_tick(S, X, v, w, torch.tensor(first))
    S = dict(S, **upd)
    return S, out


def test_safety_stop_needs_two_consecutive_scans():
    """D ≤ E_STOP 가 새 스캔 2 번 연속이어야 STOP. 같은 스캔의 서브스텝은 세지 않고, 끊기면 다시 센다."""
    S, X_near, v, w = _tick_inputs(0.40)          # 앞면(0.30)에서 0.10 m
    _, X_far, _, _ = _tick_inputs(0.70)           # 0.40 m: STOP 거리(0.30) 밖, 해제 거리(0.50) 안
    _, X_clear, _, _ = _tick_inputs(2.00)         # 1.70 m: 해제 거리 밖
    d = [float(G.swept_approach(X['scan_px'], X['scan_py'], X['scan_ok'], v.float(), w.float()))
         for X in (X_near, X_far, X_clear)]
    assert d[0] <= 0.30 < d[1] <= 0.50 < d[2], d
    # 기준: 확정 STOP 이 없을 때 X_far 에서는 φ 가 정지시키지 않는다 (아래 "유지" 확인의 대조)
    _, out_far = _tick(dict(S), X_far, v, w, True)
    assert float(out_far['v']) > 0.0
    S, out = _tick(S, X_near, v, w, True)          # 스캔 1: STOP 구역이지만 미확정 → CRITICAL
    assert int(S['stop_count']) == 1 and not bool(S['prox'])
    assert int(out['zone']) == C.ZONE_CRITICAL
    S, out = _tick(S, X_near, v, w, False)         # 같은 스캔의 다음 서브스텝 — 세지 않는다
    assert int(S['stop_count']) == 1 and not bool(S['prox'])
    S, out = _tick(S, X_far, v, w, True)           # 스캔 2: 멀어짐 → 카운터 0
    assert int(S['stop_count']) == 0 and not bool(S['prox'])
    S, out = _tick(S, X_near, v, w, True)          # 스캔 3: 다시 1
    assert int(S['stop_count']) == 1 and not bool(S['prox'])
    assert int(out['zone']) == C.ZONE_CRITICAL
    S, out = _tick(S, X_near, v, w, True)          # 스캔 4: 연속 2 → STOP
    assert int(S['stop_count']) == 2 and bool(S['prox'])
    assert float(out['v']) == 0.0 and float(out['w']) == 0.0 and int(out['zone']) == C.ZONE_STOP
    S, out = _tick(S, X_far, v, w, True)           # D ≤ 0.50 → 유지 (대조와 달리 정지)
    assert bool(S['prox']) and float(out['v']) == 0.0 and int(out['zone']) == C.ZONE_STOP
    S, out = _tick(S, X_clear, v, w, True)         # D > 0.50 → 해제
    assert not bool(S['prox']) and float(out['v']) > 0.0
