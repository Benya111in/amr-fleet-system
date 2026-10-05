#!/usr/bin/env python3
"""
Golden 벡터 생성기 — 고정 시드, 언어 중립 JSON (golden/<이름>.json, 사례당 한 파일).

사용: python3 gen_golden.py [출력 디렉터리]   (기본: 이 파일의 디렉터리)
재생성 일치 시험: rl/contract/tests/test_golden.py 가 메모리에서 다시 만들어 파일과 비교한다.
경계 민감 사례(float32 에서 판정이 뒤집힐 수 있는 것)는 생성 단계에서 거부한다 (_assert_robust).
"""

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from amr_rl_contract import constants as C  # noqa: E402
from amr_rl_contract import geometry as G  # noqa: E402
from amr_rl_contract import kinematics as K  # noqa: E402
from amr_rl_contract import mask as M  # noqa: E402
from amr_rl_contract import observation as OB  # noqa: E402
from amr_rl_contract import profiler as P  # noqa: E402
from amr_rl_contract import reward as R  # noqa: E402
from amr_rl_contract import safety as S  # noqa: E402
from amr_rl_contract.state import DevState, SafetyState, State, Track  # noqa: E402

TOLERANCE = {
    'obs': 1e-5, 'vo_time': 1e-4, 'fallback': 1e-9, 'reward': 1e-4, 'reward_terms': 1e-4,
    'cost': 1e-6, 'integrate': 1e-9, 'profiler': 1e-6, 'safety': 1e-9, 'e': 1e-6,
}
ROBUST_EPS = 1e-4


def fin(x):
    """inf/NaN → None (JSON 표준)."""
    if x is None:
        return None
    x = float(x)
    return x if math.isfinite(x) else None


def rnd(x, nd=9):
    """JSON 에 쓸 반올림 (재생성 일치를 위해 결정적)."""
    return None if x is None else round(float(x), nd)


# ------------------------------------------------------------------ 합성 스캔 (레이캐스트)
def raycast(pose, segments=(), circles=(), rng=None, sigma=0.0):
    """
    지도 좌표 선분·원을 로봇 LiDAR 로 레이캐스트한 720 빔 (None = 무반사).

    빔 원점 = 로봇 좌표 (LIDAR_X, LIDAR_Y). sigma > 0 이면 rng 가우시안 잡음 (σ, 시드 고정).
    """
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    ox = x + c * C.LIDAR_X - s * C.LIDAR_Y
    oy = y + s * C.LIDAR_X + c * C.LIDAR_Y
    ang = G.beam_angles() + th
    out = []
    for a in ang:
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for (x1, y1, x2, y2) in segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            if t > 0.0 and 0.0 <= u <= 1.0:
                best = min(best, t)
        for (cx, cy, r) in circles:
            fx, fy = ox - cx, oy - cy
            b = fx * dx + fy * dy
            cc = fx * fx + fy * fy - r * r
            disc = b * b - cc
            if disc < 0.0:
                continue
            t = -b - math.sqrt(disc)
            if t > 0.0:
                best = min(best, t)
        if math.isfinite(best) and best <= C.LIDAR_RANGE_MAX:
            if rng is not None and sigma > 0.0:
                best += rng.normal(0.0, sigma)
            out.append(round(best, 6))
        else:
            out.append(None)
    return out


def box(cx, cy, hx, hy):
    """축 정렬 상자의 네 변 (지도 좌표)."""
    return [(cx - hx, cy - hy, cx + hx, cy - hy), (cx + hx, cy - hy, cx + hx, cy + hy),
            (cx + hx, cy + hy, cx - hx, cy + hy), (cx - hx, cy + hy, cx - hx, cy - hy)]


def track_from_robot(pose, tid, p_r, v_r, age, dyn):
    """
    로봇 좌표로 정한 트랙(지금 위치 p_r, 속도 v_r)을 계약 입력(지도 좌표, 스탬프 시각 위치)으로.

    지도 위치 = 자세 + R(θ)·p_r − v_map·age (select_tracks 가 나이만큼 외삽해 p_r 로 되돌린다),
    지도 속도 = R(θ)·v_r. 값은 소수 4 자리로 반올림한다 (JSON 입력이 정확히 이 값).
    """
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    vx, vy = c * v_r[0] - s * v_r[1], s * v_r[0] + c * v_r[1]
    px, py = x + c * p_r[0] - s * p_r[1], y + s * p_r[0] + c * p_r[1]
    return Track(tid, round(px - vx * age, 4), round(py - vy * age, 4), round(vx, 4),
                 round(vy, 4), dyn)


def walls_along(pose, lateral, x0, x1):
    """로봇 진행 방향과 평행한 벽 (로봇 좌표 y = ±lateral, x ∈ [x0, x1]) 을 지도 선분으로."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    out = []
    for ly in (lateral, -lateral):
        a = (x + c * x0 - s * ly, y + s * x0 + c * ly)
        b = (x + c * x1 - s * ly, y + s * x1 + c * ly)
        out.append((a[0], a[1], b[0], b[1]))
    return out


def straight_path(x0, y0, x1, y1, step=0.05):
    """두 점 사이 0.05 m 간격 경로 (N:42 output_spacing)."""
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    return [(round(x0 + (x1 - x0) * i / n, 6), round(y0 + (y1 - y0) * i / n, 6))
            for i in range(n + 1)]


# ------------------------------------------------------------------ 경계 민감 검사
def _pen(pts, pose):
    """점들의 풋프린트 침투 최대값 (양수 = 안쪽)."""
    if pts.shape[0] == 0:
        return -math.inf
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    return float(np.max(np.minimum(C.HALF_L + C.FP_PAD - np.abs(lx),
                                   C.HALF_W + C.FP_PAD - np.abs(ly))))


def _assert_robust(state, res, name):
    pts, valid = G.scan_points(state.scan_ranges)
    pts = pts[valid]
    d_goal = M.d_goal_for_rollout(state)
    for i in range(C.N_CELLS):
        if not (res['speed_ok'][i] and res['jerk_ok'][i]):
            continue
        v, w = res['v_next'][i], res['w_next'][i]
        T = M.sim_time(v)
        n = max(1, int(math.ceil(T / C.SIM_DT - 1e-9)))
        for k in range(1, n + 1):
            s_k = abs(v) * k * C.SIM_DT
            if abs(s_k - (d_goal + C.GOAL_OVERSHOOT_MARGIN)) < ROBUST_EPS:
                raise AssertionError(f'{name}: 셀 {i} 목표 너머 경계 민감')
            if s_k > d_goal + C.GOAL_OVERSHOOT_MARGIN:
                break
            pen = _pen(pts, G.integrate_arc(v, w, k * C.SIM_DT))
            if abs(pen) < ROBUST_EPS:
                raise AssertionError(f'{name}: 셀 {i} 풋프린트 경계 민감 (pen={pen})')
            if pen >= 0.0:
                break
    vt = res['vo_time']
    if res['saturated']:
        sel = res['speed_ok'] & res['jerk_ok'] & res['fp_ok']
        t_max = float(np.max(vt[sel]))
        near = np.abs(vt[sel] - (t_max - C.VO_TIME_TIE)) < ROBUST_EPS
        if near.any():
            raise AssertionError(f'{name}: VO 포화 동률 경계 민감')
    for t in vt[np.isfinite(vt)]:
        if abs(t - C.VO_TIME_HORIZON) < ROBUST_EPS:
            raise AssertionError(f'{name}: VO τ 경계 민감')
    # VO 의 모든 분기(안/밖·접근·접선·τ)가 양쪽(유한·inf) 모두에서 ROBUST_EPS 밖인가 (v1.1.0)
    R = C.ROBOT_RADIUS + C.OBSTACLE_RADIUS + C.VO_MARGIN
    obstacles = M.vo_obstacles(state)
    for i in np.flatnonzero(res['fp_ok']):
        cx, cy = G.chord_velocity(0.0, res['v_next'][i], res['w_next'][i], C.VO_TIME_HORIZON)
        for p, vo in obstacles:
            _vo_branch_margin(p, (cx - vo[0], cy - vo[1]), R, C.VO_TIME_HORIZON,
                              f'{name}: 셀 {i} VO')
    # 관측 트랙: 선별 문턱·거리 순서·TTC 분기 (TTC 는 min(t, 5)/5 라 τ 쪽은 연속이다)
    for t in state.tracks:
        sp = math.hypot(t.vx, t.vy)
        for thr in (C.DYN_FAST_SPEED, C.DYN_SPEED_THRESHOLD):
            if abs(sp - thr) < ROBUST_EPS:
                raise AssertionError(f'{name}: 트랙 {t.id} 속력 문턱 경계 민감')
    sel = OB.select_tracks(state, math.inf)
    ds = [r[0] for r in sel]
    for d in ds:
        if min(abs(d - C.TRACK_RANGE), abs(d - C.VO_MAX_RANGE)) < ROBUST_EPS:
            raise AssertionError(f'{name}: 트랙 거리 문턱 경계 민감')
    if any(b - a < ROBUST_EPS for a, b in zip(ds, ds[1:])):
        raise AssertionError(f'{name}: 트랙 거리 순서 경계 민감')
    for _d, _i, p, v, _dyn, _sp in OB.select_tracks(state)[:C.K_TRACKS]:
        _vo_branch_margin(p, (state.v_meas - v[0], -v[1]), C.ROBOT_RADIUS + C.OBSTACLE_RADIUS,
                          None, f'{name}: 트랙 TTC')


def _vo_branch_margin(p, w_rel, radius, tau, what):
    """geometry.vo_time 의 분기 조건이 ROBUST_EPS 안이면 거부한다 (tau=None 이면 τ 분기는 안 본다)."""
    px, py = p
    wx, wy = w_rel
    pn2 = px * px + py * py
    closing = px * wx + py * wy
    wn2 = wx * wx + wy * wy
    if abs(pn2 - radius * radius) < ROBUST_EPS:
        raise AssertionError(f'{what}: 반경 경계 민감')
    if pn2 < radius * radius:
        if abs(closing) < ROBUST_EPS:
            raise AssertionError(f'{what}: 안쪽 접근 부호 경계 민감')
        return
    if wn2 < 1e-12:
        return
    if abs(closing) < ROBUST_EPS:
        raise AssertionError(f'{what}: 접근 부호 경계 민감')
    if closing <= 0.0:
        return
    disc = closing * closing - wn2 * (pn2 - radius * radius)
    if abs(disc) < ROBUST_EPS * max(1.0, closing * closing):
        raise AssertionError(f'{what}: 접선 경계 민감 (disc={disc})')
    if disc <= 0.0 or tau is None:
        return
    t = (closing - math.sqrt(disc)) / wn2
    if abs(t - tau) < ROBUST_EPS:
        raise AssertionError(f'{what}: τ 경계 민감 (t={t})')


# ------------------------------------------------------------------ 사례 → JSON
def decision_case(name, desc, state):
    """관측 + 마스크 사례."""
    obs, parts = OB.build_observation(state, return_parts=True)
    res = M.compute_mask(state)
    _assert_robust(state, res, name)
    return {
        'kind': 'decision', 'name': name, 'description': desc,
        'state': state.to_dict(),
        'expected': {
            'obs': [rnd(v, 7) for v in obs.tolist()],
            'obs_blocks': [[n, k] for n, k in OB.OBS_LAYOUT],
            'mask': [bool(v) for v in res['mask']],
            'n_valid': int(res['mask'].sum()),
            'speed_ok': [bool(v) for v in res['speed_ok']],
            'jerk_ok': [bool(v) for v in res['jerk_ok']],
            'fp_ok': [bool(v) for v in res['fp_ok']],
            'vo_time': [rnd(fin(v)) for v in res['vo_time']],
            'saturated': bool(res['saturated']),
            'fallback': [rnd(v) for v in res['fallback']],
        },
    }


def step_case(name, desc, pre, post, action, collision, pairs=()):
    """전이 보상·비용 사례."""
    r = R.step(pre, post, action, collision, pairs)
    return {
        'kind': 'step', 'name': name, 'description': desc,
        'pre': pre.to_dict(), 'post': post.to_dict(),
        'action_exec': [float(action[0]), float(action[1])], 'collision': bool(collision),
        'safety_pairs': [[[float(a), float(b)], [float(c), float(d)]]
                         for (a, b), (c, d) in pairs],
        'expected': {
            'reward': rnd(r.reward), 'terms': {k: rnd(v) for k, v in r.terms.items()},
            'cost': [rnd(v) for v in r.cost], 'cost_names': list(C.COST_NAMES),
            'dev': {'in_dev': r.dev.in_dev, 'peak': rnd(r.dev.peak),
                    't_since_peak': rnd(r.dev.t_since_peak)},
            'terminated': r.terminated, 'success': r.success, 'e': rnd(r.e),
            'phi_pre': rnd(r.phi_pre), 'phi_post': rnd(r.phi_post),
        },
    }


def bc_label_rows():
    """
    BC 레이블 사례 (contract.md §5.2, v1.1.0): (마스크, v_cmd, ω_cmd, DWA 명령) → 레이블.

    마스크는 각 장면의 compute_mask 결과다 (장면 강건성도 decision 과 같이 검사). 레이블 거리 1·2 위의 차가
    1e-6 이하인 행은 거부한다 (float32 구현이 같은 레이블을 내도록).
    """
    path = straight_path(0.0, 0.0, 10.0, 0.0)
    wall = [(1.035, -1.0, 1.035, 1.0)]      # v′ 0.48 의 마지막 자세 앞 모서리 1.02, 0.50 은 1.05
    rest = [(0.0, 0.0)] * 3
    scenes = [
        ('정지 → DWA 최대 가속 (a 1.0): 저크 마스크 밖 → 저크만큼 가속 (0.2, 0)',
         base_state(path=path), 0.1, 0.0),
        ('순항 0.5 → DWA 가속 (a 1.0)', base_state(path=path, v_cmd=0.5, prev_actions=rest), 0.6, 0.0),
        ('순항 0.5 → DWA 선회 시작 (α 2.0): 저크만큼 선회 (0, 0.5)',
         base_state(path=path, v_cmd=0.5, prev_actions=rest), 0.5, 0.2),
        ('순항 0.5 → DWA 최대 감속 (a −1.0): 폴백이 정확히 같다',
         base_state(path=path, v_cmd=0.5, prev_actions=rest), 0.4, 0.0),
        ('전방 벽: 직진·가속 셀 충돌, DWA 감속 (a −0.4) → 남은 감속 셀 (−0.2, 0)',
         base_state(path=path, scan_ranges=raycast((0.0, 0.0, 0.0), wall), v_cmd=0.5,
                    prev_actions=rest), 0.46, 0.0),
        ('직전 (0.2, 1.0) 에서 DWA 가 반대로 꺾음 (−0.3, −2.0): 폴백(−1.0, −2.0)이 더 가깝다',
         base_state(path=path, v_cmd=0.5, w_cmd=0.3, prev_actions=[(0.2, 1.0)] * 3), 0.47, 0.1),
        ('VO 포화 (d07 장면)에서 DWA 선회 (0, 1.5)',
         base_state(path=path, v_cmd=0.6, prev_actions=rest,
                    tracks=[Track(3, 1.8, 0.05, -1.5, 0.0, True)]), 0.6, 0.15),
    ]
    rows = []
    for scene, st, v_dwa, w_dwa in scenes:
        res = M.compute_mask(st)
        _assert_robust(st, res, 'bc_label: ' + scene)
        m = res['mask']
        a = ((v_dwa - st.v_cmd) / C.DT, (w_dwa - st.w_cmd) / C.DT)
        fb = K.fallback_command(st.v_cmd, st.w_cmd)
        ds = sorted([K.accel_distance(K.cell_accel(i), a) for i in range(C.N_CELLS) if m[i]] +
                    [K.accel_distance((fb[2], fb[3]), a)])
        if len(ds) > 1 and ds[1] - ds[0] <= 1e-6:
            raise AssertionError(f'bc_label: {scene}: 레이블 동률 경계 민감')
        label = K.bc_label(v_dwa, w_dwa, st.v_cmd, st.w_cmd, m)
        rows.append({'scene': scene, 'v_cmd': st.v_cmd, 'w_cmd': st.w_cmd, 'dwa': [v_dwa, w_dwa],
                     'mask': [bool(x) for x in m],
                     'out': {'a': [rnd(a[0]), rnd(a[1])], 'label': label,
                             'nearest_unmasked': K.nearest_cell(*a)}})
    return rows


def base_state(**kw):
    """기본 상태 (원점, 전방 직선 경로 10 m, 무반사)."""
    d = dict(pose=(0.0, 0.0, 0.0), path=straight_path(0.0, 0.0, 10.0, 0.0),
             scan_ranges=[None] * C.LIDAR_SAMPLES)
    d.update(kw)
    return State(**d)


def build_cases():
    """모든 사례 (이름 순서 고정)."""
    cases = []
    rng = np.random.default_rng(20261006)
    corridor = [(-5.0, 2.5, 20.0, 2.5), (-5.0, -2.5, 20.0, -2.5)]

    # ---------------- decision
    st = base_state(path=straight_path(0, 0, 5, 0))
    cases.append(decision_case('d01_open_rest', '열린 공간 정지 상태: 가속 셀 a_v∈{0,0.2}×a_w∈{−0.5,0,0.5}',
                               st))

    pose = (0.0, 0.0, 0.0)
    st = base_state(scan_ranges=raycast(pose, corridor, rng=rng, sigma=C.LIDAR_NOISE_STD),
                    v_meas=0.78, w_meas=0.01, a_v_meas=0.35, v_cmd=0.8, w_cmd=0.0,
                    prev_actions=[(0.4, 0.0), (0.6, 0.0), (0.8, 0.5)])
    cases.append(decision_case('d02_corridor_cruise', '폭 5 m 통로 순항 (LiDAR σ 0.03 시드 잡음)', st))

    segs = corridor + box(1.273, 0.013, 0.2, 0.6)
    st = base_state(scan_ranges=raycast(pose, segs), v_meas=0.5, v_cmd=0.5,
                    prev_actions=[(0.0, 0.0)] * 3)
    cases.append(decision_case('d03_obstacle_front', '전면 모서리 0.773 m 앞 상자 (0.4×1.2): 전진 셀 충돌',
                               st))

    tight = box(0.013, 0.007, 0.367, 0.263)
    st = base_state(scan_ranges=raycast(pose, tight), v_meas=0.3, v_cmd=0.3, w_cmd=0.0,
                    prev_actions=[(0.0, 0.0)] * 3)
    cases.append(decision_case('d04_all_blocked', '풋프린트 둘레 0.06 m 상자: 모든 셀 충돌 → 폴백만', st))

    narrow = [(-3.0, 0.3, 6.0, 0.3), (-3.0, -0.3, 6.0, -0.3)]
    st = base_state(scan_ranges=raycast(pose, narrow), v_meas=0.5, v_cmd=0.5, w_cmd=0.0,
                    prev_actions=[(0.0, 0.0)] * 3)
    cases.append(decision_case('d05_narrow_0p60', '순폭 0.60 m 통로 (명세 4.4) 0.5 m/s', st))

    st = base_state(scan_ranges=raycast(pose, corridor), v_meas=0.8, v_cmd=0.8,
                    prev_actions=[(0.0, 0.0)] * 3,
                    tracks=[Track(7, 3.0, -2.0, 0.0, 1.0, True)], track_age=0.1)
    cases.append(decision_case(
        'd06_vo_crossing',
        '전방 3 m 지점을 1.0 m/s 로 횡단하는 작업자 — 상대 운동의 최근접 거리(셀별 1.05–1.26 m)가 합성 반경 '
        '0.961 m 보다 커서 VO 가 어떤 셀도 막지 않는 음성 사례 (VO 가 일부 셀을 막는 사례는 d16·d17)', st))

    st = base_state(scan_ranges=raycast(pose, corridor), v_meas=0.6, v_cmd=0.6,
                    prev_actions=[(0.0, 0.0)] * 3,
                    tracks=[Track(3, 1.8, 0.05, -1.5, 0.0, True)], track_age=0.0)
    cases.append(decision_case('d07_vo_saturated', '정면 1.8 m 에서 1.5 m/s 로 다가오는 지게차: VO 포화', st))

    st = base_state(scan_ranges=raycast(pose, corridor), v_meas=0.5, w_meas=0.4, v_cmd=0.5,
                    w_cmd=0.5, a_v_meas=0.9, prev_actions=[(1.0, 2.0), (0.8, 1.5), (0.6, 1.0)])
    cases.append(decision_case('d08_jerk_limited', '직전 가속 (1.0, 2.0): 저크 마스크가 위쪽 셀만 남긴다', st))

    st = base_state(scan_ranges=raycast(pose, corridor), v_meas=0.6, v_cmd=0.6, v_cap=0.5,
                    prev_actions=[(0.0, 0.0)] * 3)
    cases.append(decision_case('d09_speed_cap_drop', '외부 속도 상한 0.5 < v_cmd 0.6: 격자 셀 전부 무효', st))

    segs = box(0.95, 0.0, 0.05, 1.0)
    st = base_state(path=straight_path(0, 0, 0.4, 0), scan_ranges=raycast(pose, segs),
                    v_meas=0.2, v_cmd=0.2, prev_actions=[(-0.2, 0.0)] * 3)
    cases.append(decision_case('d10_goal_overshoot', '목표 0.4 m, 그 뒤 벽: 목표 너머 롤아웃은 검사 제외', st))

    tracks = [Track(1, 2.0, 1.0, 0.0, 0.0, False),        # 정적·느림 → 제외
              Track(2, 2.0, -1.0, 0.6, 0.0, False),       # 빠름(≥0.5) → 포함
              Track(3, 4.0, 0.0, -0.3, 0.0, True),
              Track(4, -3.0, 0.5, 0.2, 0.2, True),
              Track(5, 12.0, 0.0, -1.0, 0.0, True),       # 10 m 밖 → 제외
              Track(6, 1.0, 3.0, 0.0, -1.2, True),
              Track(7, 5.0, 5.0, -0.5, -0.5, True),
              Track(8, 0.5, -2.0, 1.0, 0.0, True),
              Track(9, -1.0, -1.0, 0.0, 0.3, True),
              Track(10, 6.0, -1.0, 0.0, 0.0, True),
              Track(11, 3.0, 3.0, 0.1, 0.0, True)]
    st = base_state(scan_ranges=raycast(pose, corridor), v_meas=0.4, v_cmd=0.4,
                    prev_actions=[(0.2, 0.0)] * 3, tracks=tracks, track_age=0.2)
    cases.append(decision_case('d11_tracks_topk', '트랙 11 개: 선별·외삽(나이 0.2 s)·거리순 상위 8', st))

    lpath = straight_path(0.0, -2.0, 4.0, -2.0)[:-1] + straight_path(4.0, -2.0, 4.0, 3.0)
    pose12 = (3.0, -1.6, 1.2)
    walls12 = [(2.0, -3.5, 6.0, -3.5), (5.6, -3.5, 5.6, 4.0)]
    st = State(pose=pose12, path=lpath, scan_ranges=raycast(pose12, walls12, rng=rng, sigma=0.03),
               v_meas=0.3, w_meas=0.2, a_v_meas=-0.2, a_w_meas=0.3, v_cmd=0.3, w_cmd=0.2,
               prev_actions=[(-0.2, 0.5), (0.0, 0.5), (0.2, 0.0)], v_cap=1.0, scan_age=0.07,
               track_age=0.35, safety=SafetyState(C.ZONE_WARNING, 0.5, False, 1.8),
               dev=DevState(True, 0.42, 1.3))
    cases.append(decision_case('d12_curved_offpath_warning',
                               'L 자 경로, 0.4 m 이탈 중, WARNING 존·상한 0.5·TTC 1.8, 스캔 나이 0.07', st))

    ranges = raycast(pose, corridor)
    ranges[360] = 0.05          # range_min 아래 → 무효
    ranges[361] = 30.0          # range_max 위 → 무효
    ranges[362] = None
    ranges[0] = 0.2             # 후방 0.2 m (풋프린트 뒤 0.45 안쪽 → 여유 0)
    st = base_state(scan_ranges=ranges, safety=SafetyState(C.ZONE_STOP, 0.0, True, None))
    cases.append(decision_case('d13_invalid_beams_stop',
                               '무효 빔 (range_min 미만·max 초과·null), STOP 래치',
                               st))

    st = base_state(scan_ranges=raycast(pose, corridor), v_meas=0.0, w_meas=1.0, v_cmd=0.0,
                    w_cmd=1.0, a_w_meas=0.5, prev_actions=[(0.0, 1.0), (0.0, 2.0), (0.0, 2.0)],
                    path=straight_path(0, 0, -5, 0))
    cases.append(decision_case('d14_spin_in_place', '경로가 뒤: 제자리 회전 1.0 rad/s 중', st))

    # v1.1.0 추가 사례 — 기존 사례의 rng 순서를 건드리지 않도록 아래는 공유 rng 를 쓰지 않는다.
    ranges = raycast(pose, corridor)
    ranges[360] = 5.0           # 전방 점 x = 5.15: v' 1.95 의 마지막(25 번째) 자세 앞부분에만 닿는다
    st = base_state(path=straight_path(0, 0, 20, 0), scan_ranges=ranges, v_meas=1.84,
                    a_v_meas=0.95, v_cmd=1.85, v_cap=2.0,
                    prev_actions=[(1.0, 0.0), (1.0, 0.0), (0.8, 0.0)])
    cases.append(decision_case(
        'd15_vcap2_last_pose',
        'v_cap 2.0, v′ 1.95 (T_sim 2.45, n 25): 전방 5.15 m 점이 마지막 자세에만 닿는다 → 셀 94 충돌',
        st))

    # 회전한 자세(θ ≠ 0)의 트랙: 지도 속도 → 로봇 축 회전이 관측(227–290)과 VO 마스크 둘 다에 걸린다.
    # 트랙은 로봇 좌표로 정하고 track_from_robot 으로 지도 입력을 만든다 (vx·vy 모두 0 아님).
    pose16 = (2.0, 1.0, 1.2)
    age16 = 0.15
    tr16 = [track_from_robot(pose16, 21, (2.85, 0.2), (-0.4, -0.15), age16, True),   # 비포화 VO
            track_from_robot(pose16, 22, (-2.5, 1.8), (0.3, 0.45), age16, True),     # 뒤에서 멀어짐
            track_from_robot(pose16, 23, (4.5, -2.0), (0.5, -0.45), age16, False),   # 정적 표시·빠름
            track_from_robot(pose16, 24, (1.5, -1.2), (0.1, -0.07), age16, True),    # 느림: 관측만
            track_from_robot(pose16, 25, (3.0, 1.5), (0.2, 0.1), age16, False)]      # 정적·느림: 제외
    end16 = (round(2.0 + 10.0 * math.cos(1.2), 6), round(1.0 + 10.0 * math.sin(1.2), 6))
    st = State(pose=pose16, path=straight_path(2.0, 1.0, *end16),
               scan_ranges=raycast(pose16, walls_along(pose16, 2.5, -5.0, 15.0)),
               v_meas=0.6, w_meas=0.2, v_cmd=0.6, w_cmd=0.2, prev_actions=[(0.0, 0.0)] * 3,
               tracks=tr16, track_age=age16)
    cases.append(decision_case(
        'd16_rotated_tracks_vo_partial',
        'θ 1.2 rad, 트랙 5 개(선별 4, VO 대상 3), 나이 0.15 s: 비스듬히 다가오는 작업자의 VO 진입 시각이 '
        '셀 사이에서 τ 2.0 s 를 가로질러 9 셀 중 일부만 거부, 비포화', st))

    pose17 = (-1.5, 3.0, -2.3)
    age17 = 0.25
    tr17 = [track_from_robot(pose17, 31, (1.1, -2.2), (-0.3, 0.6), age17, True),    # 오른쪽→왼쪽 횡단
            track_from_robot(pose17, 32, (7.0, 0.8), (1.2, 0.3), age17, False),     # 앞에서 멀어지는 지게차
            track_from_robot(pose17, 33, (-1.5, -3.0), (-0.3, -0.6), age17, True)]  # 뒤에서 멀어짐
    end17 = (round(-1.5 + 10.0 * math.cos(-2.3), 6), round(3.0 + 10.0 * math.sin(-2.3), 6))
    st = State(pose=pose17, path=straight_path(-1.5, 3.0, *end17),
               scan_ranges=raycast(pose17, walls_along(pose17, 3.5, -4.0, 14.0)),
               v_meas=0.8, w_meas=-0.35, v_cmd=0.8, w_cmd=-0.35,
               prev_actions=[(0.2, -0.5), (0.2, -0.5), (0.0, 0.0)], tracks=tr17, track_age=age17)
    cases.append(decision_case(
        'd17_rotated_crossing_vo_partial',
        'θ −2.3 rad (cos·sin 모두 음수), 오른쪽 옆에서 0.67 m/s 로 비스듬히 건너오는 작업자, 나이 0.25 s, '
        '우회전 중: VO 원뿔의 접선이 셀 사이를 지나 일부만 거부 (거부 셀 진입 < 1.9 s), 비포화', st))

    # ---------------- step
    pre = base_state(v_meas=0.8, v_cmd=0.8, prev_actions=[(0.2, 0.0)] * 3)
    post = base_state(pose=(0.08, 0.01, 0.0), v_meas=0.82, v_cmd=0.82,
                      prev_actions=[(0.2, 0.0)] * 3)
    cases.append(step_case('s01_progress', '경로 따라 0.08 m 진행, 이벤트 없음', pre, post,
                           (0.2, 0.0), False))

    p2 = straight_path(0, 0, 3, 0)
    pre = base_state(path=p2, pose=(2.9, 0.0, 0.0), v_meas=0.1, v_cmd=0.1)
    post = base_state(path=p2, pose=(2.95, 0.02, 0.0), v_meas=0.02, v_cmd=0.0)
    cases.append(step_case('s02_success', '목표 0.054 m·0.02 m/s: 성공, 종단 Φ=0', pre, post,
                           (-1.0, 0.0), False))

    pre = base_state(pose=(2.0, 0.0, 0.0), v_meas=0.9, v_cmd=0.9)
    post = base_state(pose=(2.09, 0.0, 0.0), v_meas=0.9, v_cmd=0.9)
    cases.append(step_case('s03_collision', '충돌: 종단, 비용[0]=1, 벌점 −10', pre, post,
                           (0.0, 0.0), True))

    pre = base_state(pose=(3.0, 0.25, 0.3), v_meas=0.5)
    post = base_state(pose=(3.05, 0.35, 0.3), v_meas=0.5)
    cases.append(step_case('s04_dev_start', '이탈 0.35 > 0.30: 이탈 구간 시작, 추종 벌점 꺼짐', pre, post,
                           (0.0, 0.0), False))

    pre = base_state(pose=(4.0, 0.55, 0.0), dev=DevState(True, 0.6, 4.95))
    post = base_state(pose=(4.05, 0.5, 0.0))
    cases.append(step_case('s05_return_overtime', '최대 이탈 뒤 5.05 s: return_5s 비용 1', pre, post,
                           (0.0, 0.0), False))

    pre = base_state(pose=(5.0, 0.2, 0.0), dev=DevState(True, 0.8, 2.0))
    post = base_state(pose=(5.05, 0.12, 0.0))
    cases.append(step_case('s06_dev_end_cte', '0.12 ≤ 0.15 복귀: 구간 끝, 곧바로 추종 모드 CTE 비용 1',
                           pre, post, (0.0, 0.0), False))

    pre = base_state(pose=(5.0, 1.15, 0.0), dev=DevState(True, 1.15, 0.0))
    post = base_state(pose=(5.05, 1.2, 0.0))
    cases.append(step_case('s07_deviation_1m', '이탈 1.2 m > 1.0: deviation_1m 비용 1', pre, post,
                           (0.0, 0.0), False))

    pre = base_state(pose=(1.0, 0.0, 0.0), v_meas=0.8, v_cmd=0.8)
    post = base_state(pose=(1.06, 0.0, 0.0), v_meas=0.6, v_cmd=0.8)
    pairs = [((0.8, 0.2), (0.4, 0.1)), ((0.8, 0.2), (0.5, 0.125)), ((0.8, 0.2), (0.8, 0.2))]
    cases.append(step_case('s08_intervention', 'safety 틱 3 개의 보정 크기 평균', pre, post,
                           (0.0, 0.0), False, pairs))

    pre = base_state(pose=(1.0, 0.0, 0.0), v_meas=0.5, prev_actions=[(-1.0, -2.0)] * 3)
    post = base_state(pose=(1.05, 0.0, 0.0), v_meas=0.5)
    cases.append(step_case('s09_smooth_bangbang', '가속 (−1,−2) → (1,2): 평활 벌점 최대 0.1', pre, post,
                           (1.0, 2.0), False))

    pre = base_state(path=p2, pose=(2.92, 0.0, 0.0), v_meas=0.05)
    post = base_state(path=p2, pose=(2.95, 0.0, 0.0), v_meas=0.01)
    cases.append(step_case('s10_success_and_collision', '도달 조건과 충돌이 같은 스텝: 충돌 우선', pre, post,
                           (0.0, 0.0), True))

    pre = base_state(path=lpath, pose=(3.9, -1.5, 1.5708), v_meas=0.7, v_cmd=0.7,
                     dev=DevState(False, 0.0, 0.0))
    post = base_state(path=lpath, pose=(3.92, -1.43, 1.5708), v_meas=0.7, v_cmd=0.7)
    cases.append(step_case('s11_curved_path_track', 'L 자 경로 모서리 근처 추종 (CTE 0.08)', pre, post,
                           (0.0, 0.0), False))

    # ---------------- integrate
    rows_b = []
    for u, v, w, cap in [((0.5, -0.5), 0.5, 0.0, 1.0), ((1.0, 1.0), 0.97, 1.45, 1.0),
                         ((-1.0, -1.0), 0.05, -1.45, 1.0), ((2.0, 0.3), 0.6, 0.2, 0.5),
                         ((0.0, 0.0), 0.0, 0.0, 1.0), ((-0.3, 0.25), 1.8, 0.0, 2.0)]:
        rows_b.append({'u': list(u), 'v_cmd': v, 'w_cmd': w, 'v_cap': cap,
                       'out': [rnd(x) for x in K.integrate_b(u, v, w, cap)]})
    rows_c = []
    for idx, v, w in [(0, 0.5, 0.2), (49, 0.3, 0.0), (98, 0.2, -0.3), (55, 0.0, 0.0),
                      (C.FALLBACK_INDEX, 0.05, 0.15), (C.FALLBACK_INDEX, 0.8, -0.6),
                      (C.FALLBACK_INDEX, 0.0, 0.0)]:
        rows_c.append({'index': idx, 'v_cmd': v, 'w_cmd': w,
                       'out': [rnd(x) for x in K.integrate_c(idx, v, w)]})
    rows_n = [{'a': [a, b], 'cell': K.nearest_cell(a, b)}
              for a, b in [(0.0, 0.0), (0.33, -0.7), (-0.95, 1.9), (0.1, 0.25), (1.0, -2.0)]]
    cases.append({'kind': 'integrate', 'name': 'i01_action_integration',
                  'description': '비교군 B 적분(클램프)·C 셀/폴백 적분·격자 최근접 셀·BC 레이블(마스크 안 최근접, '
                                 'v1.1.0)',
                  'expected': {'b': rows_b, 'c': rows_c, 'nearest_cell': rows_n,
                               'bc_label': bc_label_rows(),
                               'lattice_av': list(C.LATTICE_AV),
                               'lattice_aw': list(C.LATTICE_AW)}})

    # ---------------- profiler
    prof = P.ProfilerReference()
    seq = []
    v, w = 0.0, 0.0
    plan = [(0.2, 0.0)] * 5 + [(0.6, 0.0)] * 5 + [(0.0, 0.5)] * 6 + [(-1.0, -1.0)] * 8 + \
        [(0.0, 0.0)] * 4
    for a_v, a_w in plan:
        v1 = min(max(v + a_v * C.DT, 0.0), 1.0)
        w1 = min(max(w + a_w * C.DT, -C.W_MAX), C.W_MAX)
        subs = P.run_decision_period(prof, v, w, v1, w1)
        seq.append({'cmd_from': [rnd(v), rnd(w)], 'cmd_to': [rnd(v1), rnd(w1)],
                    'ref': [[rnd(a), rnd(b)] for a, b in subs], 'a_ref': rnd(prof.a_ref)})
        v, w = v1, w1
    cases.append({'kind': 'profiler', 'name': 'p01_profiler_sequence',
                  'description': '정책 명령 램프 28 주기 → velocity_profiler 기준 (50 Hz 서브스텝)',
                  'expected': {'dt': C.PROFILER_DT, 'steps': seq}})

    # ---------------- safety
    rows_s = []
    for v, w, d, ttc, stop in [(1.0, 0.0, None, None, False), (1.0, 0.0, 2.0, None, False),
                               (0.8, 0.4, 0.8, None, False), (0.5, 0.0, 0.45, None, False),
                               (0.9, 0.0, 3.0, 1.2, False), (0.3, 0.0, 0.28, None, False),
                               (0.3, 0.5, 0.25, None, True), (-0.2, 0.0, 5.0, 0.5, False),
                               (0.0, 1.0, 0.9, None, False)]:
        out = S.safety_filter(v, w, d, ttc, stop)
        rows_s.append({'in': {'v': v, 'w': w, 'd_main': d, 'min_ttc': ttc, 'proximity_stop': stop},
                       'out': {'v': rnd(out['v']), 'w': rnd(out['w']), 'zone': out['zone'],
                               'speed_limit': rnd(fin(out['speed_limit']))}})
    cases.append({'kind': 'safety', 'name': 'f01_safety_lite',
                  'description': 'φ_lite: 존 상한·여유거리 상한·TTC 상한·곡률 보존 스케일',
                  'expected': {'rows': rows_s}})
    return cases


def write_all(out_dir):
    """사례를 파일로 쓰고 이름 목록을 돌려준다."""
    cases = build_cases()
    names = []
    for case in cases:
        case['contract_version'] = C.CONTRACT_VERSION
        case['tolerance'] = TOLERANCE
        path = os.path.join(out_dir, case['name'] + '.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(case, f, ensure_ascii=False, indent=1, allow_nan=False)
            f.write('\n')
        names.append(case['name'])
    return names


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else HERE
    written = write_all(out)
    print(f'{len(written)} 사례 → {out}')
