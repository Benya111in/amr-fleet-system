"""
보상·비용·종료 기준 구현 (contract.md §6–§8).

보상 r = r_time + r_shape + r_smooth + r_track + r_goal + r_collision
  r_shape = γ·Φ(s') − Φ(s),  Φ(s) = −κ·d_arc(s),  종단(성공·충돌)에서 Φ(s') = 0  (PBRS, Ng 1999)
비용 c = [collision, intervention, deviation_1m, return_5s, cte_track]  (비할인 에피소드 합으로 제약)
이탈 상태기계(reward machine) = tests/integration/amr_itest/metrics.py:240-263
deviation_episodes 와 같은 규칙.
"""

from dataclasses import dataclass
import math

from . import constants as C
from . import geometry as G
from .state import DevState


def cross_track_error(state):
    """경로 최근접점까지 거리 (부호 없음) — 이탈·CTE 판정 입력."""
    return G.project_onto_path(state.path, state.pose[:2])[1]


def potential(state, terminal):
    """Φ(s) = −κ·d_arc(s), 종단이면 0."""
    if terminal:
        return 0.0
    return -C.KAPPA * G.remaining_arc(state.path, state.pose[:2])


def dev_update(dev, e, dt=C.DT):
    """
    이탈 상태기계 한 스텝 (metrics.py:250-263 과 같은 순서).

    밖이면: e > DEV_OUT 에서 시작 (peak = e, 타이머 0).
    안이면: e > peak 면 peak 갱신·타이머 0, 아니면 타이머 += dt; 그 뒤 e ≤ DEV_BACK 이면 끝 (모두 0).
    """
    if not dev.in_dev:
        if e > C.DEV_OUT:
            return DevState(True, e, 0.0)
        return DevState(False, 0.0, 0.0)
    if e > dev.peak:
        nd = DevState(True, e, 0.0)
    else:
        nd = DevState(True, dev.peak, dev.t_since_peak + dt)
    if e <= C.DEV_BACK:
        return DevState(False, 0.0, 0.0)
    return nd


def is_success(state, collision=False):
    """목표 xy ≤ GOAL_XY_TOL 이고 |v_meas| ≤ GOAL_STOP_SPEED (충돌과 동시면 실패)."""
    if collision:
        return False
    gx, gy = state.path[-1]
    d = math.hypot(state.pose[0] - gx, state.pose[1] - gy)
    return d <= C.GOAL_XY_TOL and abs(state.v_meas) <= C.GOAL_STOP_SPEED


def intervention_cost(pairs):
    """
    안전 개입 크기 — (u, φ(u)) 쌍들의 평균 ((Δv/V_OP)² + (Δw/W_MAX)²).

    pairs: [((v_in, w_in), (v_out, w_out)), ...] — 결정 주기 안 safety 틱들. 빈 목록 = 0.
    """
    if not pairs:
        return 0.0
    tot = 0.0
    for (vi, wi), (vo, wo) in pairs:
        tot += ((vi - vo) / C.INTERVENTION_V_NORM) ** 2 + ((wi - wo) / C.INTERVENTION_W_NORM) ** 2
    return tot / len(pairs)


@dataclass
class StepResult:
    """한 전이의 보상·비용·종료."""

    reward: float
    terms: dict
    cost: tuple
    dev: DevState
    terminated: bool
    success: bool
    e: float
    phi_pre: float
    phi_post: float


def step(pre, post, action_exec, collision, safety_pairs=()):
    """
    전이 (pre, a, post) 의 보상·비용.

    action_exec: 이번 주기의 명령 가속도 (a_v, a_w) — 클램프·폴백 뒤 실제 값.
    직전 행동 = pre.prev_actions[0]. post.dev 는 무시하고 pre.dev 에서 갱신한다.
    collision: 환경의 지면 진실 접촉 (풋프린트 vs 장애물 기하).
    """
    success = is_success(post, collision)
    terminated = bool(collision or success)
    e = cross_track_error(post)
    dev = dev_update(pre.dev, e)
    phi_pre = potential(pre, False)
    phi_post = potential(post, terminated)
    a_prev = pre.prev_actions[0] if pre.prev_actions else (0.0, 0.0)
    dav = (action_exec[0] - a_prev[0]) / C.A_MAX
    daw = (action_exec[1] - a_prev[1]) / C.ALPHA_MAX
    terms = {
        'time': -C.TIME_COST,
        'shape': C.GAMMA * phi_post - phi_pre,
        'smooth': -C.SMOOTH_WEIGHT * (dav * dav + daw * daw),
        'track': 0.0 if dev.in_dev else -C.TRACK_WEIGHT * min(e / C.CTE_REF, C.TRACK_CLIP) ** 2,
        'goal': C.GOAL_REWARD if success else 0.0,
        'collision': -C.COLLISION_PENALTY if collision else 0.0,
    }
    cost = (
        1.0 if collision else 0.0,
        intervention_cost(list(safety_pairs)),
        1.0 if e > C.DEV_LIMIT else 0.0,
        1.0 if (dev.in_dev and dev.t_since_peak > C.RETURN_LIMIT) else 0.0,
        1.0 if (not dev.in_dev and e > C.CTE_TRACK_LIMIT) else 0.0,
    )
    reward = sum(terms.values())
    return StepResult(reward, terms, cost, dev, terminated, success, e, phi_pre, phi_post)


def time_limit(path):
    """에피소드 시간 상한 [s] = min(MAX, BASE + PER_M·경로 길이) — 넘으면 truncation."""
    return min(C.TIME_LIMIT_MAX, C.TIME_LIMIT_BASE + C.TIME_LIMIT_PER_M * G.path_length(path))


class ProgressChecker:
    """
    nav2 SimpleProgressChecker 와 같은 정체 판정 (N:86-87) — 정체면 truncation.

    기준점에서 STALL_RADIUS 이상 움직이면 기준점·시각을 갱신, 기준 시각 뒤 STALL_TIME 이 지나면 정체.
    """

    def __init__(self, xy, t):
        """기준점 (xy, t)."""
        self.base = (float(xy[0]), float(xy[1]))
        self.t0 = float(t)

    def update(self, xy, t):
        """정체면 True."""
        if math.hypot(xy[0] - self.base[0], xy[1] - self.base[1]) >= C.STALL_RADIUS:
            self.base = (float(xy[0]), float(xy[1]))
            self.t0 = float(t)
            return False
        return (t - self.t0) > C.STALL_TIME


def discounted_return(rewards, gamma=C.GAMMA):
    """Σ γ^t r_t."""
    g, tot = 1.0, 0.0
    for r in rewards:
        tot += g * r
        g *= gamma
    return tot
