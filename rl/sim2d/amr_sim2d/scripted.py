"""
착취 표(G0)용 결정론적 스크립트 정책 — 비교군 B 행동 공간 u ∈ [−1, 1]².

정책은 관측과 env 의 에피소드 시각 env.t 만 본다 (정책 상태 없음, 리셋하면 처음부터). 목표 속도 (v*, ω*) 를
가속도 행동으로 바꾸는 규칙은 heuristic.path_follower 와 같다: u = clip((v* − v_cmd)/0.1, ±1),
clip((ω* − ω_cmd)/0.2, ±1). 학습 정책이 아니다 — 보상 설계가 각 행동 양식에 매기는 값을 재는 기준 궤적이다.
"""

import torch

from . import heuristic as H

I_REM_ARC = 213                      # goal 블록: 남은 호길이/10


def _u(v_t, w_t, obs):
    v_cmd, w_cmd = obs[:, H.I_VCMD] * 2.0, obs[:, H.I_WCMD] * 1.5
    return torch.stack([torch.clamp((v_t - v_cmd) / 0.1, -1, 1),
                        torch.clamp((w_t - w_cmd) / 0.2, -1, 1)], 1)


def _follow_target(obs, v_des=H.V_DES, slow_down=True):
    """경로 추종기의 (v*, ω*) — heuristic.path_follower 와 같은 식."""
    b = torch.atan2(obs[:, H.I_SG2_SIN], obs[:, H.I_SG2_COS])
    gd = obs[:, H.I_GOAL_D] * 10.0
    v = v_des * torch.clamp(torch.cos(b), min=0.0)
    if slow_down:
        v = v * torch.clamp(gd, max=1.0)
        v = torch.where(gd < 0.08, torch.zeros_like(v), v)
    w = torch.clamp(2.0 * b, -1.2, 1.2)
    return v, w


def _const(obs, v, w):
    z = torch.zeros(obs.shape[0], device=obs.device, dtype=obs.dtype)
    return z + v, z + w


def idle(obs, t):
    """제자리 대기."""
    return _u(*_const(obs, 0.0, 0.0), obs)


def follow(obs, t):
    """경로 추종 휴리스틱 (BC 시연자와 같다) — 목표 1 m 안에서 선형 감속, 0.08 m 안에서 정지."""
    return _u(*_follow_target(obs), obs)


def follow_slow(obs, t):
    """같은 추종기를 0.3 m/s 로 (느린 성공)."""
    return _u(*_follow_target(obs, v_des=0.3), obs)


def overshoot(obs, t):
    """감속·정지 없이 0.8 m/s 로 추종 — 목표를 지나친 뒤 끝점 둘레를 돈다 (목표 근처 배회)."""
    return _u(*_follow_target(obs, slow_down=False), obs)


def stop_short(obs, t, margin=1.5):
    """추종하다 남은 호길이 margin 안에서 멈춘다 (목표 직전 정지 → 정체)."""
    v, w = _follow_target(obs)
    near = obs[:, I_REM_ARC] * 10.0 < margin
    return _u(torch.where(near, torch.zeros_like(v), v), torch.where(near, torch.zeros_like(w), w),
              obs)


def _phase(obs, t, t0, turn_s, w_turn, v_turn, v_after, w_after=0.0, resume=None):
    """시각에 따라 세 단계: t < t0 추종, t0 ≤ t < t0 + turn_s 선회, 그 뒤 (v_after, w_after) 또는 resume."""
    v_f, w_f = _follow_target(obs)
    v_t, w_t = _const(obs, v_turn, w_turn)
    if resume is None:
        v_a, w_a = _const(obs, v_after, w_after)
    else:
        v_a, w_a = resume
    tt = t.to(obs.dtype)
    before, turning = tt < t0, (tt >= t0) & (tt < t0 + turn_s)
    v = torch.where(before, v_f, torch.where(turning, v_t, v_a))
    w = torch.where(before, w_f, torch.where(turning, w_t, w_a))
    return v, w


def crash(obs, t):
    """4 s 추종 → 왼쪽으로 약 90° (ω 1.2, 1.3 s) → 1.0 m/s 직진 (φ 를 끄면 벽에 충돌)."""
    return _u(*_phase(obs, t, 4.0, 1.3, 1.2, 0.3, 1.0), obs)


def drift(obs, t):
    """3 s 추종 → 왼쪽으로 약 86° (ω 1.0, 1.5 s) → 0.5 m/s 직진 (경로를 떠나 표류, φ 가 벽 앞에서 세운다)."""
    return _u(*_phase(obs, t, 3.0, 1.5, 1.0, 0.5, 0.5), obs)


def swerve(obs, t):
    """3 s 추종 → 왼쪽 약 57° (1 s) → 2 s 직진 0.6 m/s (옆으로 약 1 m) → 다시 추종 (우회 후 복귀)."""
    v_f, w_f = _follow_target(obs)
    v, w = _phase(obs, t, 3.0, 1.0, 1.0, 0.6, 0.6, resume=(v_f, w_f))
    tt = t.to(obs.dtype)
    straight = (tt >= 4.0) & (tt < 6.0)
    v = torch.where(straight, torch.full_like(v, 0.6), v)
    w = torch.where(straight, torch.zeros_like(w), w)
    return _u(v, w, obs)


def spin(obs, t):
    """제자리 최대 회전."""
    return _u(*_const(obs, 0.0, 1.5), obs)


def creep(obs, t):
    """추종 방향으로 0.012 m/s (20 s 에 0.24 m < 0.3 m) — 진행은 있으나 정체 판정."""
    v, w = _follow_target(obs, v_des=0.012)
    return _u(v, w, obs)


def circle(obs, t):
    """원 주행 (v 0.4, ω 1.2, 반경 0.33 m) — 정체 판정을 피하며 배회."""
    return _u(*_const(obs, 0.4, 1.2), obs)


POLICIES = {
    'idle': idle, 'follow': follow, 'follow_slow': follow_slow, 'overshoot': overshoot,
    'stop_short': stop_short, 'crash': crash, 'drift': drift, 'swerve': swerve, 'spin': spin,
    'creep': creep, 'circle': circle,
}
