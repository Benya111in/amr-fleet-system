"""
2b 동적 장애물 (r6-dyn) 시험 — 시작점 거르기, 최소 이격, 배치 수준, learnability 표본, PID 승수, 후보 보상의 착취 순서.

근거: docs/research/rl-navigation/dyn_obstacles.md.
"""

import math

import pytest
import torch

from amr_sim2d import dyn_candidates as DC
from amr_sim2d import geometry2d as G
from amr_sim2d import obstacles as OB
from amr_sim2d import scripted as SP
from amr_sim2d.curriculum import LevelSampler, mixed_probs
from amr_sim2d.ppo import PPOConfig, Trainer

from amr_rl_contract import constants as CC
from amr_rl_contract import reward as RR
from amr_rl_contract.state import empty_scan, State

DEV = 'cuda' if torch.cuda.is_available() else 'cpu'
gpu = pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')


def make(n=64, seed=3, **kw):
    from amr_sim2d.env import EnvConfig, Sim2DEnv
    cfg = EnvConfig(num_envs=n, device=DEV, seed=seed, arm=kw.pop('arm', 'B'),
                    obstacle_mode=kw.pop('mode', 'mixed'), n_obstacles=kw.pop('n_obst', (0, 4)),
                    path_len=kw.pop('path_len', (3.0, 15.0)), compile=False, n_paths=512, **kw)
    return Sim2DEnv(cfg, log=lambda *a: None)


# ------------------------------------------------------------------ 기하
def test_body_distance_matches_sampling():
    """원·회전 사각형 몸체 거리 = 몸체 경계 조밀 표본까지의 최소 거리 (안쪽 0)."""
    g = torch.Generator().manual_seed(0)
    n, K = 200, 3
    st = {'xy': torch.rand(n, K, 2, generator=g, dtype=torch.float64) * 6 - 3,
          'yaw': torch.rand(n, K, generator=g, dtype=torch.float64) * 6.3,
          'kind': torch.randint(0, 2, (n, K), generator=g),
          'r': torch.full((n, K), 0.3, dtype=torch.float64),
          'box': torch.tensor(OB.FORKLIFT_BOX, dtype=torch.float64).expand(n, K, 4).clone()}
    p = torch.rand(n, 2, generator=g, dtype=torch.float64) * 6 - 3
    d = OB.body_distance(st, p)
    # 표본: 원 둘레 / 사각형 내부 격자
    u = torch.linspace(0, 1, 81, dtype=torch.float64)
    bx0, bx1, by0, by1 = OB.FORKLIFT_BOX
    gx, gy = torch.meshgrid(bx0 + (bx1 - bx0) * u, by0 + (by1 - by0) * u, indexing='ij')
    for i in range(n):
        for k in range(K):
            c, s = math.cos(st['yaw'][i, k]), math.sin(st['yaw'][i, k])
            if st['kind'][i, k] == OB.KIND_CIRCLE:
                ref = max(0.0, float(torch.linalg.norm(p[i] - st['xy'][i, k])) - 0.3)
            else:
                wx = st['xy'][i, k, 0] + c * gx - s * gy
                wy = st['xy'][i, k, 1] + s * gx + c * gy
                ref = float(torch.hypot(wx - p[i, 0], wy - p[i, 1]).min())
            assert float(d[i, k]) == pytest.approx(ref, abs=0.03), (i, k)


def test_footprint_obstacle_distance_zero_iff_contact_and_matches_sampling():
    """풋프린트–장애물 거리: 표본 기준과 일치하고, 떨어져 있으면 rect_hits_obstacles 가 거짓이다."""
    g = torch.Generator().manual_seed(1)
    n, K = 400, 2
    pose = torch.cat([torch.rand(n, 2, generator=g, dtype=torch.float64) * 4 - 2,
                      torch.rand(n, 1, generator=g, dtype=torch.float64) * 6.3], 1)
    kind = torch.randint(0, 2, (n, K), generator=g)
    box = torch.where((torch.rand(n, K, 1, generator=g) < 0.5),
                      torch.tensor(OB.FORKLIFT_BOX, dtype=torch.float64),
                      torch.tensor(OB.AMR_BOX, dtype=torch.float64))
    ob = {'xy': torch.rand(n, K, 2, generator=g, dtype=torch.float64) * 6 - 3,
          'yaw': torch.rand(n, K, generator=g, dtype=torch.float64) * 6.3, 'kind': kind,
          'r': torch.full((n, K), 0.3, dtype=torch.float64), 'box': box,
          'active': torch.ones(n, K, dtype=torch.bool)}
    d = G.footprint_obstacle_distance(pose, ob)
    hit = G.rect_hits_obstacles(pose, ob)
    assert bool((d[~hit] > 0).all())
    # 장애물 꼭짓점이 로봇 긴 변에 가장 가까운 배치 (로봇 꼭짓점만 보면 거리를 크게 잰다)
    p1 = torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float64)
    hd = math.hypot(0.3, 0.2)                  # AMR 상자 중심 → 꼭짓점 (0.3, 0.2)
    yaw = -math.pi / 2 - math.atan2(0.2, 0.3)  # 그 꼭짓점이 −y (로봇 쪽) 를 향하게
    ob1 = {'xy': torch.tensor([[[0.0, CC.HALF_W + 0.10 + hd]]], dtype=torch.float64),
           'yaw': torch.tensor([[yaw]], dtype=torch.float64),
           'kind': torch.tensor([[1]]), 'r': torch.zeros(1, 1, dtype=torch.float64),
           'box': torch.tensor([[OB.AMR_BOX]], dtype=torch.float64),
           'active': torch.ones(1, 1, dtype=torch.bool)}
    assert float(G.footprint_obstacle_distance(p1, ob1)[0]) == pytest.approx(0.10, abs=1e-6)
    # 표본 기준: 풋프린트 경계 점 ↔ 몸체 거리 (원은 중심−반경, 사각형은 body_distance)
    u = torch.linspace(-1, 1, 41, dtype=torch.float64)
    hl, hw = CC.HALF_L, CC.HALF_W
    edge = torch.cat([torch.stack([u * hl, torch.full_like(u, hw)], 1),
                      torch.stack([u * hl, torch.full_like(u, -hw)], 1),
                      torch.stack([torch.full_like(u, hl), u * hw], 1),
                      torch.stack([torch.full_like(u, -hl), u * hw], 1)])
    for i in range(0, n, 7):
        if bool(hit[i]):
            continue
        c, s = math.cos(pose[i, 2]), math.sin(pose[i, 2])
        wx = pose[i, 0] + c * edge[:, 0] - s * edge[:, 1]
        wy = pose[i, 1] + s * edge[:, 0] + c * edge[:, 1]
        pts = torch.stack([wx, wy], 1)
        best = math.inf
        for k in range(K):
            stk = {key: ob[key][i:i + 1, k:k + 1].expand(pts.shape[0], 1, *ob[key].shape[2:])
                   for key in ('xy', 'yaw', 'kind', 'r', 'box')}
            best = min(best, float(OB.body_distance(stk, pts).min()))
        assert float(d[i]) == pytest.approx(best, abs=0.03), i


# ------------------------------------------------------------------ 시작점 거르기
@gpu
def test_start_filter_removes_conflicts_and_keeps_obstacles():
    """거르기 뒤: 처음 창 동안 시작점 반경 안 장애물 0 (조밀 시각), 활성 장애물 수는 거르기 없는 쪽과 거의 같다."""
    e0 = make(n=256, seed=11)
    e1 = make(n=256, seed=11, start_clear_window=3.0, start_clear_radius=1.0)
    for e in (e0, e1):
        e.gen.manual_seed(11)
        e.reset()
    idx = torch.arange(256, device=e0.dev)
    c0 = e0.obst.start_conflict(idx, e0.t, e0.pose[:, :2], 3.0, 1.0, dt=0.05)
    c1 = e1.obst.start_conflict(idx, e1.t, e1.pose[:, :2], 3.0, 1.0, dt=0.05)
    assert int(c0.sum()) > 0, '거를 것이 없는 시드 — 시험이 공허하다'
    assert int(c1.sum()) == 0
    # start_conflict 와 다른 경로(Obstacles.state + body_distance)로 조밀 시각에서 다시 확인한다
    for k in range(61):
        st = e1.obst.state(e1.t + 0.05 * k)
        d = OB.body_distance(st, e1.pose[:, :2])
        assert not bool(((d < 1.0) & st['active']).any()), k
    assert e1.clear_stats['conflict0'] > 0
    # 위상만 바꾸고 끄는 것은 드물다
    assert e1.clear_stats['disabled'] <= 0.2 * e1.clear_stats['conflict0']
    n0, n1 = int(e0.obst.active.sum()), int(e1.obst.active.sum())
    assert n1 >= n0 - e1.clear_stats['disabled']
    # 시작·목표·DR 은 같다 (거르기는 장애물 위상만 바꾼다)
    assert torch.equal(e0.pose, e1.pose)
    assert torch.equal(e0.tm, e1.tm)


@gpu
def test_start_filter_off_keeps_random_stream():
    """창 0 이면 clear_start 가 난수를 쓰지 않는다 — 2a 까지의 배치와 같은 장애물."""
    e0 = make(n=64, seed=5)
    e1 = make(n=64, seed=5, start_clear_window=0.0, start_clear_radius=2.0)
    for e in (e0, e1):
        e.gen.manual_seed(5)
        e.reset()
    for k in ('active', 'a', 'b', 'phase', 'speed'):
        assert torch.equal(getattr(e0.obst, k), getattr(e1.obst, k)), k


# ------------------------------------------------------------------ 진단·수준
@gpu
def test_episode_min_separation_and_stationary_collision():
    """서 있는 로봇: 충돌한 에피소드의 최소 이격 = 0, 정지 중 접촉, 안 맞은 에피소드는 이격 > 0."""
    from amr_sim2d import dyn_diag as D
    env = make(n=128, seed=4242)
    rec = D.first_episodes(env, SP.idle, 4242)
    coll = rec['outcome'] == 1
    assert int(coll.sum()) > 0
    assert bool((rec['min_sep'][coll] == 0).all())
    assert not bool(rec['collision_moving'][coll].any())
    other = ~coll & torch.isfinite(rec['min_sep'])
    assert bool((rec['min_sep'][other] > 0).all())
    assert bool(rec['finished'].all())
    # 움직이는 정책: 주행 중 접촉과 '회전 중 정지 접촉' 은 겹치지 않는다
    rf = D.first_episodes(env, D.ESCAPE['follow_0.8'], 4242)
    cf = rf['outcome'] == 1
    assert int((cf & rf['collision_moving']).sum()) > 0
    assert not bool((cf & rf['collision_moving'] & rf['collision_turning']).any())


@gpu
def test_level_probs_sampling():
    """level_probs: 수준 표본이 확률을 따르고, 수준 = 무작위 개수 또는 Gazebo, 기록이 배치와 맞다."""
    env = make(n=2048, seed=2)
    p = torch.zeros(env.n_levels, dtype=torch.float64)
    p[0], p[3], p[-1] = 0.5, 0.3, 0.2
    env.set_level_probs(p)
    env.gen.manual_seed(2)
    env.reset()
    lv = env.ep_level.cpu()
    frac = torch.bincount(lv, minlength=env.n_levels).double() / lv.numel()
    assert float((frac - p).abs().max()) < 0.04
    gz = env.ep_level == env.n_levels - 1
    assert bool((env.obst.active[gz].sum(1) == len(OB.GAZEBO_AGENTS)).all())
    assert bool((env.obst.active[env.ep_level == 0].sum(1) == 0).all())
    assert bool((env.obst.active[env.ep_level == 3].sum(1) <= 3).all())
    with pytest.raises(ValueError):
        env.set_level_probs([1.0, 0.0])
    # 'random' 모드는 Gazebo 수준을 뽑지 않는다
    er = make(n=512, seed=2, mode='random')
    er.set_level_probs(p)
    er.gen.manual_seed(2)
    er.reset()
    assert not bool((er.ep_level == er.n_levels - 1).any())


def test_level_sampler_learnability():
    """p(1−p) 가 큰 수준이 더 자주, 허용 밖 수준은 0, 균일 몫 u 는 허용 수준 모두에 간다."""
    s = LevelSampler(10, uniform=0.2, ema=1.0, eps=0.0, min_eps=1)
    eps = [{'level': torch.tensor([0] * 10 + [1] * 10 + [2] * 10),
            'success': torch.tensor([True] * 10 + [True] * 5 + [False] * 5 + [False] * 10)}]
    s.update(eps)
    assert s.p[:3].tolist() == [1.0, 0.5, 0.0]
    al = s.allowed(2, full=False)
    pr = s.probs(al)
    assert float(pr.sum()) == pytest.approx(1.0)
    assert float(pr[1]) == pytest.approx(0.8 + 0.2 / 3)
    assert float(pr[0]) == pytest.approx(0.2 / 3)
    assert float(pr[3:].sum()) == 0.0
    assert bool(s.allowed(4, full=True)[9]) and not bool(s.allowed(4, full=False)[9])
    m = mixed_probs(10, (0, 4), 0.2)
    assert float(m.sum()) == pytest.approx(1.0) and float(m[9]) == pytest.approx(0.2)


# ------------------------------------------------------------------ PID 승수
class _FakeEnv:
    def __init__(self):
        self.dev = torch.device('cpu')
        self.N = 4
        self.cfg = type('c', (), {'arm': 'B'})()
        self.k = 0

    def reset(self):
        return torch.zeros(self.N, 291)

    def step(self, u):
        self.k += 1
        z = torch.zeros(self.N, dtype=torch.bool)
        return (torch.full((self.N, 291), float(self.k)), torch.zeros(self.N), z, z,
                {'cost': torch.zeros(self.N, 5)})


def _eps(c):
    return [{'cost': torch.tensor([[c, 0.0, 0.0, 0.0, 0.0]])}]


def test_pid_lagrangian_defaults_equal_pi_and_channels():
    """미분 계수 0·EMA 0·채널 전부 = 기존 PI 갱신, 채널 선택 밖은 λ = 0, 미분 항은 오차 증가에만."""
    cfg = PPOConfig(lagrangian=True, lag_kp=1.0, lag_ki=0.5, lag_max=100.0, rollout=2)
    tr = Trainer(_FakeEnv(), cfg)
    lam = tr.update_lagrange(_eps(0.52))
    assert lam[0] == pytest.approx(0.5 * 0.5 + 1.0 * 0.5)
    cfg2 = PPOConfig(lagrangian=True, lag_kp=0.0, lag_ki=0.0, lag_kd=2.0, lag_max=100.0,
                     rollout=2, lag_channels=(2,))
    tr2 = Trainer(_FakeEnv(), cfg2)
    assert tr2.update_lagrange(_eps(0.52))[0] == 0.0          # 채널 0 은 제약 밖
    cfg3 = PPOConfig(lagrangian=True, lag_kp=0.0, lag_ki=0.0, lag_kd=2.0, lag_max=100.0,
                     rollout=2, lag_channels=(0,))
    tr3 = Trainer(_FakeEnv(), cfg3)
    assert tr3.update_lagrange(_eps(0.22))[0] == pytest.approx(2.0 * 0.2)   # e 0 → 0.2
    assert tr3.update_lagrange(_eps(0.12))[0] == 0.0                         # 감소 → D 0
    cfg5 = PPOConfig(lagrangian=True, lag_kp=1.0, lag_ki=0.0, lag_kd=2.0, lag_max=100.0,
                     rollout=2, lag_channels=(0,))
    tr5 = Trainer(_FakeEnv(), cfg5)
    tr5.update_lagrange(_eps(0.22))
    assert tr5.update_lagrange(_eps(0.12))[0] == pytest.approx(0.1)   # 감소 오차는 D 에서 빼지 않는다
    cfg4 = PPOConfig(lagrangian=True, lag_ki=1.0, lag_ema=0.5, lag_max=100.0, rollout=2,
                     lag_channels=(0,))
    tr4 = Trainer(_FakeEnv(), cfg4)
    tr4.update_lagrange(_eps(0.42))
    lam = tr4.update_lagrange(_eps(0.02))                   # EMA 0.22 → e 0.2
    assert lam[0] == pytest.approx(0.4 + 0.2)


# ------------------------------------------------------------------ 후보 보상의 착취 순서
PATH = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0)]               # 15 m


def _arc(s):
    return RR.G.interpolate_at(PATH, s)


def _value(xys, spec, lam=0.0, collide_last=False, first_action=(0.0, 0.0), tail=3000):
    """
    상태열의 할인 가치 (끝나지 않으면 마지막 위치에 정지한 채 tail 스텝 — 연속 롤아웃 부트스트랩).

    라그랑주 승수 λ 는 충돌 비용 채널에 건다 (r − λ·cost₀ = 실효 충돌 벌점 C + λ).
    """
    pre = State(pose=(*xys[0], 0.0), path=PATH, scan_ranges=empty_scan())
    g, val = 1.0, 0.0
    n = len(xys) - 1
    for k in range(n + (0 if collide_last else tail)):
        xy = xys[min(k + 1, n)]
        post = State(pose=(*xy, 0.0), path=PATH, scan_ranges=empty_scan(),
                     v_meas=0.5 if k < n else 0.0)
        post.dev = pre.dev
        a = first_action if k == 0 else (0.0, 0.0)
        coll = collide_last and k == n - 1
        r = RR.step(pre, post, a, coll, spec=spec)
        val += g * (r.reward - lam * r.cost[0])
        g *= CC.GAMMA
        if r.terminated:
            break
        post.dev = r.dev
        post.prev_actions = [a] + list(pre.prev_actions[:2])
        pre = post
    return val


def _drive(s0, s1):
    k = max(1, int(round(abs(s1 - s0) / 0.1)))
    return [_arc(s0 + (s1 - s0) * i / k) for i in range(k + 1)]


CAND_SPECS = [('v1.2.0 (①②④)', RR.SPEC, 0.0), ('prog50_c10 λ=0 (③ 시작)', DC.get('prog50_c10'), 0.0),
              ('prog50_c10 λ=20 (③)', DC.get('prog50_c10'), 20.0),
              ('prog50_c6 λ=0', DC.get('prog50_c6'), 0.0)]


@pytest.mark.parametrize('name,spec,lam', CAND_SPECS)
def test_candidate_spin_and_wait_not_better_than_progress(name, spec, lam):
    """
    회전 ≤ 대기 < 진행 후 정지 < 성공 (할인, 연속 롤아웃). 회전 = 제자리에서 각가속 행동 (위치 그대로).

    보상은 ω 를 보지 않으므로 회전은 평활 비용만큼 대기보다 낮다 — 동적 장애물에서 본 회전 끌개는 보상이 회전을
    고르게 한 것이 아니라 대기와 값이 같아 ω 가 떠다닌 것이다 (dyn_obstacles.md §3).
    """
    idle = _value([_arc(0.0)], spec, lam)
    spin = _value([_arc(0.0)], spec, lam, first_action=(0.0, 2.0))
    partial = _value(_drive(0.0, 10.0), spec, lam)
    succ = _value(_drive(0.0, 15.0), spec, lam)
    assert spin < idle
    assert idle < partial < succ


@pytest.mark.parametrize('name,spec,lam', CAND_SPECS)
def test_candidate_stop_beats_crash_everywhere(name, spec, lam):
    """결정 수준 자살 방지: 같은 지점까지 와서 정지 ≻ 그 자리 충돌 — 실효 벌점 C + λ > γ·c̄."""
    for s in (1.0, 5.0, 10.0, 14.5):
        tr = _drive(0.0, s)
        assert _value(tr, spec, lam) > _value(tr, spec, lam, collide_last=True), s


def test_candidate_c_floor():
    """C ≤ γ·c̄ 후보는 만들지 않는다."""
    with pytest.raises(ValueError):
        DC.get('prog50_c3.9')
    with pytest.raises(KeyError):
        DC.get('nope')
    assert DC.get('prog50_c10').collision_penalty == 10.0


@gpu
def test_spin_not_better_than_idle_in_dynamic_env():
    """동적 장애물 env 에서도 회전은 대기보다 낫지 않다 (짝 비교, 고정 시나리오, 할인 연속 롤아웃)."""
    from amr_sim2d import exploit as X
    env = make(n=256, seed=21, truncate=False)
    ri = X.record(env, SP.idle, 600, 21)
    rs = X.record(env, SP.spin, 600, 21)
    for spec in (RR.SPEC, DC.get('prog50_c10')):
        gi = X.spec_returns(ri, spec)['g_boot']
        gs = X.spec_returns(rs, spec)['g_boot']
        assert float((gs - gi).mean()) <= 1e-6, spec.name


# ------------------------------------------------------------------ 특권 전문가 (후보 ④)
@gpu
def test_privileged_expert_collides_less_than_blind_follower():
    """특권 MPC 전문가는 같은 시나리오에서 장애물을 보지 않는 추종기보다 충돌이 적고 성공이 많다 (짝 비교)."""
    from amr_sim2d import dyn_diag as D
    from amr_sim2d.expert import PrivilegedExpert
    env = make(n=128, seed=4242, start_clear_window=3.0)
    ex = PrivilegedExpert(env)
    re_ = D.first_episodes(env, ex, 4242)
    rf = D.first_episodes(env, D.ESCAPE['follow_0.8'], 4242)
    ce, cf = (re_['outcome'] == 1).double().mean(), (rf['outcome'] == 1).double().mean()
    se, sf = (re_['outcome'] == 0).double().mean(), (rf['outcome'] == 0).double().mean()
    assert float(ce) < 0.5 * float(cf), (float(ce), float(cf))
    assert float(se) > float(sf)
    assert ex.stats['calls'] > 0 and ex.stats['follow_safe'] > 0


def test_kl_to_bc_reference_is_frozen_copy():
    """BC 기준은 깊은 사본이다: 학생 가중치를 바꿔도 기준 분포는 그대로, 같은 가중치면 KL = 0."""
    cfg = PPOConfig(rollout=2, kl_bc=0.1)
    tr = Trainer(_FakeEnv(), cfg)
    tr.set_bc_reference()
    obs = torch.randn(16, 291)
    kl0 = torch.distributions.kl_divergence(tr.model.dist(obs), tr.bc_ref.dist(obs)).sum(-1)
    assert float(kl0.abs().max()) < 1e-8
    with torch.no_grad():
        tr.model.pi[-1].bias.add_(0.5)
    kl1 = torch.distributions.kl_divergence(tr.model.dist(obs), tr.bc_ref.dist(obs)).sum(-1)
    assert float(kl1.min()) > 0.1
    assert not any(p.requires_grad for p in tr.bc_ref.parameters())


def test_freeze_norm_stops_running_stats():
    """freeze_norm 이면 롤아웃이 관측 정규화 통계를 바꾸지 않는다 (KL-to-BC 기준과 같은 정규화)."""
    for frozen in (False, True):
        tr = Trainer(_FakeEnv(), PPOConfig(rollout=3, freeze_norm=frozen))
        c0, m0 = float(tr.model.norm.count), tr.model.norm.mean.clone()
        tr.collect()
        changed = float(tr.model.norm.count) != c0 or not torch.equal(m0, tr.model.norm.mean)
        assert changed != frozen
