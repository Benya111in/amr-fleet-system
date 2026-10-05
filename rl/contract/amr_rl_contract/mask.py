"""
비교군 C 의 후보 마스크 기준 구현 (contract.md §5.2).

셀 (a_v, a_w) 는 직전 명령 (v_cmd, w_cmd) 기준 상대 가속도 격자다. 셀이 유효하려면 네 조건을
모두 만족해야 한다 (이론 보고서 「masking은 정당한 policy gradient지만 네 조건이 붙는다」):
  speed — v' = v_cmd + a_v·DT ∈ [V_MIN, v_cap], |w'| ≤ W_MAX         (V_s)
  jerk  — |a_v − a_prev_v| ≤ JERK_MAX·DT, |a_w − a_prev_w| ≤ ANG_JERK_MAX·DT  (mask 단계 저크)
  fp    — (v', w') 등속 원호를 T_sim(v') 동안 SIM_DT 간격으로 굴려 풋프린트에 스캔 점이 없다
          (dwa.cpp:682-724 의 충돌 검사, 목표 너머 d_goal + 0.1 m 제외)
  vo    — VO 진입 시각이 +inf (dwa.cpp:731-761, 같은 τ·반경·선별)
VO 가 fp 를 통과한 셀을 모두 덮으면(포화) DWA 선택 규칙(dwa.cpp:946-)처럼 VO 진입 시각이
최대값 − VO_TIME_TIE 이상인 셀을 유효로 한다. 폴백(인덱스 99)은 항상 유효하다.
"""

import math

import numpy as np

from . import constants as C
from . import geometry as G
from .kinematics import cell_accel, fallback_command
from .observation import select_tracks


def sim_time(v):
    """dwa.cpp:60-64 simTime."""
    return min(max(abs(v) / C.A_MAX + 0.5, C.SIM_TIME_MIN), C.SIM_TIME_MAX)


def d_goal_for_rollout(state):
    """dwa.cpp:449-459: max(남은 호길이, 목표까지 직선거리)."""
    x, y, _ = state.pose
    s0 = G.project_onto_path(state.path, (x, y))[0]
    rem = max(0.0, G.path_length(state.path) - s0)
    gx, gy = state.path[-1]
    return max(rem, math.hypot(gx - x, gy - y))


def footprint_free(pts_robot, v, w, d_goal):
    """
    (v, w) 원호 롤아웃이 스캔 점과 겹치지 않는가.

    poses k = 1..ceil(T/SIM_DT) (dwa.cpp:175-187 rollout 과 같은 n), 호길이 |v|·k·SIM_DT 가
    d_goal + 여유를 넘으면 그 앞에서 멈춘다 (dwa.cpp:711-723). 모든 점을 모든 자세에서 검사한다 —
    근접점 사전필터를 두지 않는다. v1.0.0 의 필터(반경 |v|·T + R)는 T 가 SIM_DT 의 배수가 아닐 때
    (|v| > 1.0) 마지막 자세 호길이 |v|·n·SIM_DT 보다 짧아 그 자세에만 닿는 점을 버렸다.
    """
    T = sim_time(v)
    n = max(1, int(math.ceil(T / C.SIM_DT - 1e-9)))
    for k in range(1, n + 1):
        s_k = abs(v) * k * C.SIM_DT
        if math.isfinite(d_goal) and s_k > d_goal + C.GOAL_OVERSHOOT_MARGIN:
            break
        pose = G.integrate_arc(v, w, k * C.SIM_DT)
        if G.points_in_footprint(pts_robot, pose):
            return False
    return True


def vo_obstacles(state):
    """VO 에 쓰는 동적 트랙: select_tracks ∩ 속력 ≥ 0.2 ∩ 거리 ≤ 6 m."""
    return [(p, v) for d, _i, p, v, _dyn, sp in select_tracks(state, C.VO_MAX_RANGE)
            if sp >= C.DYN_SPEED_THRESHOLD]


def cell_vo_time(v, w, obstacles):
    """셀 명령 (v, w) 의 VO 진입 시각 (장애물 최소), 없으면 +inf."""
    if not obstacles:
        return math.inf
    cx, cy = G.chord_velocity(0.0, v, w, C.VO_TIME_HORIZON)
    R = C.ROBOT_RADIUS + C.OBSTACLE_RADIUS + C.VO_MARGIN
    t = math.inf
    for p, vo in obstacles:
        t = min(t, G.vo_time((p[0], p[1]), (cx - vo[0], cy - vo[1]), R, C.VO_TIME_HORIZON))
    return t


def compute_mask(state):
    """
    State → 마스크와 셀별 판정.

    반환 dict: mask (bool, N_ACTIONS_C), speed_ok·jerk_ok·fp_ok (bool, N_CELLS),
    vo_time (float, N_CELLS; 계산 안 한 셀·VO 없음 = inf), saturated (bool),
    fallback (v, w, a_v, a_w), v_next·w_next (float, N_CELLS).
    """
    n = C.N_CELLS
    speed_ok = np.zeros(n, dtype=bool)
    jerk_ok = np.zeros(n, dtype=bool)
    fp_ok = np.zeros(n, dtype=bool)
    vo_t = np.full(n, math.inf)
    v_next = np.zeros(n)
    w_next = np.zeros(n)
    a_prev_v, a_prev_w = state.prev_actions[0] if state.prev_actions else (0.0, 0.0)
    v_hi = min(state.v_cap, C.V_MAX_HW)
    pts, valid = G.scan_points(state.scan_ranges)
    pts = pts[valid]
    d_goal = d_goal_for_rollout(state)
    obstacles = vo_obstacles(state)
    tol = C.LATTICE_TOL
    for i in range(n):
        a_v, a_w = cell_accel(i)
        v1 = state.v_cmd + a_v * C.DT
        w1 = state.w_cmd + a_w * C.DT
        v_next[i], w_next[i] = v1, w1
        speed_ok[i] = (C.V_MIN - tol <= v1 <= v_hi + tol) and abs(w1) <= C.W_MAX + tol
        jerk_ok[i] = (abs(a_v - a_prev_v) <= C.JERK_STEP_V + tol and
                      abs(a_w - a_prev_w) <= C.JERK_STEP_W + tol)
        if not (speed_ok[i] and jerk_ok[i]):
            continue
        fp_ok[i] = footprint_free(pts, v1, w1, d_goal)
        if fp_ok[i]:
            vo_t[i] = cell_vo_time(v1, w1, obstacles)
    sel = speed_ok & jerk_ok & fp_ok
    vo_ok = ~np.isfinite(vo_t)
    saturated = False
    cells = sel & vo_ok
    if not cells.any() and sel.any():
        saturated = True
        t_max = float(np.max(vo_t[sel]))
        cells = sel & (vo_t >= t_max - C.VO_TIME_TIE)
    mask = np.zeros(C.N_ACTIONS_C, dtype=bool)
    mask[:n] = cells
    mask[C.FALLBACK_INDEX] = True
    return {
        'mask': mask, 'speed_ok': speed_ok, 'jerk_ok': jerk_ok, 'fp_ok': fp_ok,
        'vo_time': vo_t, 'saturated': saturated,
        'fallback': fallback_command(state.v_cmd, state.w_cmd),
        'v_next': v_next, 'w_next': w_next,
    }
