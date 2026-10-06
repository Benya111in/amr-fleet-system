"""
계약 v1.2.0 보상 반영 시험 — Isaac env 의 torch 보상(contract_torch.reward_step)과 env 배선의 순수 부분.

contract.md §6 (v1.2.0) 의 세 변경을 손 사례로 고정한다 (golden 32 개와 무작위 차분은 다른 파일):
  ① Φ·진행의 거리 d = max(d_arc, 목표 직선거리) — 끝점 너머·목표 옆에서 직선거리가 이긴다
  ② 진행 효용 κ_b·(d − d') — 종단(충돌·성공) 스텝도 실제 d' (Φ 처럼 0 으로 두지 않는다)
  ③ 이탈 힌지 −q_d·1[e > 1 m]
env 배선: observe_with_cache 가 관측 시점 자세로 d(s) 를 캐시하고, 다음 전이가 그것을 d_pre 로 쓴다
(env 의 step_outcome 시점에는 버퍼 자세가 이미 전이 뒤다). 경로 길이 필터·보상 판 선택·에피소드 기록도 본다.
"""

import math

import numpy as np
import pytest
import torch

from amr_isaac import contract_torch as CT
from amr_isaac import env_math as EM
from amr_rl_contract import constants as C
from amr_rl_contract import reward as R
from amr_rl_contract.state import DevState, State

F64 = torch.float64
STRAIGHT = [(0.0, 0.0), (10.0, 0.0)]
ELL = [(0.0, 0.0), (6.0, 0.0), (6.0, 6.0)]


def _st(pose, path=STRAIGHT, v=0.0, dev=None, prev=None):
    return State(pose=tuple(pose), path=path, scan_ranges=[None] * C.LIDAR_SAMPLES, v_meas=v,
                 dev=dev or DevState(), prev_actions=prev or [(0.0, 0.0)] * 3)


def _step(pre, post, col=False, spec=R.SPEC, d_pre=None):
    bp = CT.states_from_dicts([pre.to_dict()])
    bq = CT.states_from_dicts([post.to_dict()])
    return CT.reward_step(bp, bq.pose, bq.v_meas, torch.zeros(1, 2, dtype=F64),
                          torch.tensor([col]), None, d_pre=d_pre, spec=spec)


def _f(t):
    return float(t[0])


# ------------------------------------------------------------------ 판·기본값
def test_contract_version_and_default_spec():
    assert C.CONTRACT_VERSION == '1.2.0'
    assert (R.SPEC.phi_dist, R.SPEC.progress_kappa, R.SPEC.dev_hinge) == ('eff', 0.5, 0.05)
    assert (R.SPEC.kappa, R.SPEC.collision_penalty, R.SPEC.stall_terminal) == (1.0, 30.0, False)
    # reward_step 의 기본 설계 = 계약 기본 SPEC (값을 복사하지 않는다)
    import inspect
    assert inspect.signature(CT.reward_step).parameters['spec'].default is R.SPEC
    assert EM.reward_spec_for('1.2.0') is R.SPEC
    assert EM.reward_spec_for('1.1.0') is R.SPEC_V110
    with pytest.raises(ValueError):
        EM.reward_spec_for('1.3.0')


def test_stall_terminal_spec_is_rejected():
    from amr_rl_contract.reward_variants import CANDIDATES
    st = CANDIDATES['prog50_st']
    assert st.stall_terminal
    with pytest.raises(ValueError):
        _step(_st((1, 0, 0)), _st((1.1, 0, 0)), spec=st)


# ------------------------------------------------------------------ ① 거리
@pytest.mark.parametrize('pose,path,exp', [
    ((10.3, 0.0, 0.0), STRAIGHT, 0.3),                  # 끝점 너머: d_arc 0, 직선 0.3
    ((9.9, 0.5, 0.0), STRAIGHT, math.hypot(0.1, 0.5)),  # 목표 옆: d_arc 0.1 < 직선 0.51
    ((4.0, 0.2, 0.0), STRAIGHT, math.hypot(6.0, 0.2)),  # 경로 옆: d_arc 6.0 < 직선 6.003
    ((3.0, 0.0, 0.0), ELL, 9.0),                         # L 자: d_arc 9 > 직선 √(9+36)
])
def test_progress_distance_is_max_of_arc_and_goal(pose, path, exp):
    st = _st(pose, path)
    bs = CT.states_from_dicts([st.to_dict()])
    got = _f(CT.state_progress_distance(bs))
    ref = R.progress_distance(st)
    assert got == pytest.approx(ref, abs=1e-12)
    d_arc = _f(CT.remaining_arc(bs))
    d_goal = math.hypot(pose[0] - path[-1][0], pose[1] - path[-1][1])
    assert got == pytest.approx(max(d_arc, d_goal), abs=1e-12)
    assert got == pytest.approx(exp, abs=1e-12)
    # v1.1.0 은 d_arc 만
    assert _f(CT.state_progress_distance(bs, R.SPEC_V110)) == pytest.approx(d_arc, abs=1e-12)


def test_beyond_goal_shaping_and_progress_use_goal_distance():
    """끝점을 0.30 → 0.34 m 지나침 (golden s12 와 같은 꼴) — d_arc 는 둘 다 0 이다."""
    out = _step(_st((10.30, 0.0, 0.0)), _st((10.34, 0.0, 0.0)))
    assert _f(out.d_pre) == pytest.approx(0.30, abs=1e-12)
    assert _f(out.d_post) == pytest.approx(0.34, abs=1e-12)
    assert _f(out.terms['shape']) == pytest.approx(C.GAMMA * -0.34 + 0.30, abs=1e-12)
    assert _f(out.terms['progress']) == pytest.approx(0.5 * (0.30 - 0.34), abs=1e-12)
    v110 = _step(_st((10.30, 0.0, 0.0)), _st((10.34, 0.0, 0.0)), spec=R.SPEC_V110)
    assert _f(v110.terms['shape']) == 0.0 and 'progress' not in v110.terms


# ------------------------------------------------------------------ ② 진행 효용
def test_progress_on_path_is_half_delta_d():
    out = _step(_st((2.0, 0.0, 0.0)), _st((2.1, 0.0, 0.0)))
    assert _f(out.terms['progress']) == pytest.approx(0.5 * 0.1, abs=1e-12)


def test_progress_on_collision_uses_actual_post_distance():
    """멀리서의 충돌: Φ' = 0 이라 shape = +κ·d 이지만 진행 효용은 κ_b·(d − d') 이지 κ_b·d 가 아니다."""
    out = _step(_st((2.0, 0.0, 0.0)), _st((2.5, 0.0, 0.0)), col=True)
    assert bool(out.terminated[0]) and not bool(out.success[0])
    assert _f(out.phi_post) == 0.0
    assert _f(out.terms['shape']) == pytest.approx(8.0, abs=1e-12)
    assert _f(out.terms['progress']) == pytest.approx(0.5 * (8.0 - 7.5), abs=1e-12)
    assert _f(out.d_post) == pytest.approx(7.5, abs=1e-12)
    ref = R.step(_st((2.0, 0.0, 0.0)), _st((2.5, 0.0, 0.0)), (0.0, 0.0), True)
    assert _f(out.reward) == pytest.approx(ref.reward, abs=1e-12)


def test_progress_on_success_uses_actual_post_distance():
    out = _step(_st((9.8, 0.0, 0.0)), _st((9.95, 0.03, 0.0), v=0.02))
    assert bool(out.success[0])
    d_post = math.hypot(0.05, 0.03)              # 끝점 0.05 m 앞, 옆 0.03 → 직선이 이긴다
    assert _f(out.d_post) == pytest.approx(d_post, abs=1e-12)
    assert _f(out.terms['progress']) == pytest.approx(0.5 * (0.2 - d_post), abs=1e-12)
    assert _f(out.terms['goal']) == 10.0


def test_progress_sum_telescopes_through_env_cache():
    """
    환경 순서(관측 때 캐시 → 자세 이동 → reward_step(d_pre=캐시))로 여러 스텝 — 진행 효용 합 = κ_b(d₀ − d_T).

    캐시가 앞 스텝의 d' 와 같은 정의(max)여야 합이 텔레스코핑된다. 경로 끝을 지나 옆으로 비켜 가는 궤적.
    """
    poses = [(8.0, 0.0, 0.0), (9.0, 0.1, 0.0), (9.9, 0.2, 0.0), (10.2, 0.3, 0.0),
             (10.4, 0.6, 0.0), (10.1, 1.3, 0.0)]
    tot = 0.0
    hinge = 0
    for a, b in zip(poses[:-1], poses[1:]):
        pre = CT.states_from_dicts([_st(a).to_dict()])
        _obs, cache = CT.observe_with_cache(pre)
        post = CT.states_from_dicts([_st(b).to_dict()])
        pre.pose = post.pose.clone()                    # env: 버퍼 자세는 이미 전이 뒤
        out = CT.reward_step(pre, post.pose, post.v_meas, torch.zeros(1, 2, dtype=F64),
                             torch.tensor([False]), None, d_pre=cache)
        ref = R.step(_st(a), _st(b), (0.0, 0.0), False)
        assert _f(out.reward) == pytest.approx(ref.reward, abs=1e-12)
        tot += _f(out.terms['progress'])
        hinge += _f(out.terms['dev']) != 0.0
    d0 = R.progress_distance(_st(poses[0]))
    dT = R.progress_distance(_st(poses[-1]))
    assert tot == pytest.approx(0.5 * (d0 - dT), abs=1e-12)
    assert hinge == 1                                    # 마지막 자세만 e > 1 m


# ------------------------------------------------------------------ ③ 이탈 힌지
@pytest.mark.parametrize('e,hinge', [(0.99, 0.0), (1.0, 0.0), (1.0 + 1e-6, -0.05), (2.5, -0.05)])
@pytest.mark.parametrize('in_dev', [False, True])
def test_dev_hinge_threshold_and_independent_of_dev_segment(e, hinge, in_dev):
    dev = DevState(True, 1.5, 0.3) if in_dev else DevState()
    out = _step(_st((5.0, 0.0, 0.0), dev=dev), _st((5.0, e, 0.0), dev=dev))
    assert _f(out.e) == pytest.approx(e, abs=1e-12)
    assert _f(out.terms['dev']) == hinge
    ref = R.step(_st((5.0, 0.0, 0.0), dev=dev), _st((5.0, e, 0.0), dev=dev), (0.0, 0.0), False)
    assert ref.terms['dev'] == hinge
    # 힌지와 비용 deviation_1m 은 같은 문턱이다
    assert float(out.cost[0, 2]) == (1.0 if hinge else 0.0)


def test_hinge_exceeds_track_penalty_max():
    """q_d 0.05 > 추종 벌점 최댓값 q·3² = 0.036 — 1 m 밖이 경로 근처보다 싸지지 않는다 (§6.3)."""
    assert R.SPEC.dev_hinge > C.TRACK_WEIGHT * C.TRACK_CLIP ** 2


# ------------------------------------------------------------------ env 캐시
def test_observe_with_cache_matches_reference_distance():
    rng = np.random.default_rng(0)
    states = []
    for _ in range(64):
        path = ELL if rng.random() < 0.5 else STRAIGHT
        gx, gy = path[-1]
        pose = (gx + rng.uniform(-3, 3), gy + rng.uniform(-3, 3), rng.uniform(-3, 3))
        states.append(_st(pose, path))
    bs = CT.states_from_dicts([s.to_dict() for s in states])
    obs, cache = CT.observe_with_cache(bs)
    assert torch.equal(obs, CT.build_observation(bs))
    for i, s in enumerate(states):
        assert float(cache[i]) == pytest.approx(R.progress_distance(s), abs=1e-12)
    _o, c110 = CT.observe_with_cache(bs, R.SPEC_V110)
    assert torch.allclose(c110, CT.remaining_arc(bs), atol=0, rtol=0)
    assert bool((cache > c110 + 1e-6).any())


# ------------------------------------------------------------------ 경로 길이 필터
def _toy_pool():
    lens = np.array([2.999, 3.0, 7.5, 15.0, 15.001, 29.0])
    n = np.array([3, 4, 2, 5, 3, 6])
    offs = np.concatenate([[0], np.cumsum(n)])
    pts = np.stack([np.arange(offs[-1], dtype=float), -np.arange(offs[-1], dtype=float)], 1)
    return pts, offs, lens, n


def test_select_paths_by_length_inclusive_and_intact():
    pts, offs, lens, n = _toy_pool()
    p2, o2, l2, kept = EM.select_paths_by_length(pts, offs, lens, 3.0, 15.0)
    assert kept.tolist() == [1, 2, 3]
    assert l2.tolist() == [3.0, 7.5, 15.0]
    assert o2.tolist() == [0, 4, 6, 11]
    for j, k in enumerate(kept):
        assert np.array_equal(p2[o2[j]:o2[j + 1]], pts[offs[k]:offs[k + 1]])
    p3, o3, l3, k3 = EM.select_paths_by_length(pts, offs, lens)
    assert np.array_equal(p3, pts) and np.array_equal(o3, offs) and k3.tolist() == list(range(6))
    _p, _o, _l, k4 = EM.select_paths_by_length(pts, offs, lens, None, 15.0)
    assert k4.tolist() == [0, 1, 2, 3]
    with pytest.raises(ValueError):
        EM.select_paths_by_length(pts, offs, lens, 16.0, 20.0)


# ------------------------------------------------------------------ 에피소드 기록
def test_episode_log_first_episode_copy():
    log = EM.EpisodeLog(3, C.N_COSTS, 'cpu')
    one = torch.ones(3, dtype=F64)
    ep = {'return': torch.tensor([1.0, 2.0, 3.0], dtype=F64), 'path_len': 5 * one,
          'd_start': 5 * one, 'd_final': torch.tensor([0.0, 4.0, 2.0], dtype=F64),
          'steps': torch.tensor([7, 8, 9]), 'cost': torch.ones(3, C.N_COSTS, dtype=F64),
          'path_id': torch.tensor([10, 11, 12])}
    log.accumulate(torch.tensor([0.05, 0.5, 1.0], dtype=F64), torch.tensor([1.2, 1.2, 0.0],
                   dtype=F64), torch.tensor([0.2, 1.4, 0.0], dtype=F64), 3 * one)
    log.accumulate(torch.tensor([0.05, 0.5, 1.0], dtype=F64), torch.zeros(3, dtype=F64),
                   torch.tensor([0.1, 0.4, 0.0], dtype=F64), torch.tensor([2.0, 4, 1], dtype=F64))
    done = torch.tensor([True, False, True])
    res = torch.tensor([[False, True, False, False], [True, False, False, False],
                        [False, False, False, True]])
    log.finish(done, res, ep)
    assert log.fin_count.tolist() == [1, 0, 1]
    assert log.fin['outcome'].tolist() == [1, 0, 3]          # env 1 은 안 끝나 기본값 그대로
    assert log.fin['spin'].tolist() == [1, 0, 0]             # |ω| > 1.0 이고 |v| < 0.1 인 스텝
    assert log.fin['maxe'][0] == pytest.approx(0.2) and float(log.fin['maxe'][1]) == 0.0
    assert log.fin['gmin'].tolist() == [2.0, 0.0, 1.0]
    assert log.fin['travel'][2] == pytest.approx(2 * 1.0 * C.DT)
    assert log.fin['time_s'][2] == pytest.approx(9 * C.DT)
    assert log.fin['path_id'].tolist() == [10, 0, 12]
    log.reset(torch.tensor([0, 2]))
    assert float(log.travel[0]) == 0.0 and math.isinf(float(log.gmin[2]))
    assert float(log.travel[1]) == pytest.approx(2 * 0.5 * C.DT)   # 리셋 안 한 env 는 그대로
    # 두 번째로 끝나면 사본은 덮지만 횟수가 2 가 된다 — 평가는 fin_count 가 0→1 일 때만 읽는다
    log.finish(torch.tensor([True, False, False]),
               torch.tensor([[True, False, False, False]] * 3), ep)
    assert log.fin_count.tolist() == [2, 0, 1] and log.fin['outcome'].tolist() == [0, 0, 3]
