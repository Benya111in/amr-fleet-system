"""기하·행동 적분·프로파일러·safety_lite 단위 시험."""

import math

import numpy as np
import pytest

from amr_rl_contract import constants as C
from amr_rl_contract import geometry as G
from amr_rl_contract import kinematics as K
from amr_rl_contract import profiler as P
from amr_rl_contract import safety as S


def test_wrap_angle_range():
    for a in np.linspace(-20, 20, 401):
        w = G.wrap_angle(a)
        assert -math.pi <= w < math.pi
        assert abs(math.sin(w) - math.sin(a)) < 1e-9 and abs(math.cos(w) - math.cos(a)) < 1e-9


def test_footprint_clearance():
    pts = [(0.0, 0.0), (0.5, 0.0), (0.0, -0.5), (0.4, 0.3), (-0.3, 0.2)]
    d = G.footprint_clearance(pts)
    assert d == pytest.approx([0.0, 0.2, 0.3, math.hypot(0.1, 0.1), 0.0])


def test_integrate_arc_exact():
    x, y, th = G.integrate_arc(1.0, 0.0, 2.0)
    assert (x, y, th) == pytest.approx((2.0, 0.0, 0.0))
    x, y, th = G.integrate_arc(1.0, 1.0, math.pi)       # 반지름 1 반원
    assert (x, y, th) == pytest.approx((0.0, 2.0, math.pi), abs=1e-12)


def test_vo_time_cases():
    R = 1.0
    assert G.vo_time((3.0, 0.0), (1.0, 0.0), R, 5.0) == pytest.approx(2.0)
    assert G.vo_time((3.0, 0.0), (1.0, 0.0), R, 1.9) == math.inf      # τ 밖
    assert G.vo_time((3.0, 0.0), (-1.0, 0.0), R, 5.0) == math.inf     # 멀어짐
    assert G.vo_time((0.5, 0.0), (1.0, 0.0), R, 5.0) == 0.0           # 이미 안, 다가감
    assert G.vo_time((0.5, 0.0), (-1.0, 0.0), R, 5.0) == math.inf     # 이미 안, 멀어짐
    assert G.vo_time((3.0, 1.0), (1.0, 0.0), R, 5.0) == math.inf      # 접선 = 밖


def test_chord_velocity():
    assert G.chord_velocity(0.0, 1.0, 0.0, 2.0) == pytest.approx((1.0, 0.0))
    cx, cy = G.chord_velocity(0.0, 1.0, 1.0, 2.0)
    x, y, _ = G.integrate_arc(1.0, 1.0, 2.0)
    assert (cx, cy) == pytest.approx((x / 2.0, y / 2.0))


def test_projection_signed_and_ties():
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    s, d, e, yaw = G.project_onto_path(path, (1.0, 0.3))
    assert (s, d, e, yaw) == pytest.approx((1.0, 0.3, 0.3, 0.0))
    s, d, e, yaw = G.project_onto_path(path, (1.0, -0.3))
    assert e == pytest.approx(-0.3)
    s, d, e, yaw = G.project_onto_path(path, (2.5, -0.5))    # 꼭짓점: 첫 구간이 이긴다
    assert s == pytest.approx(2.0) and yaw == pytest.approx(0.0)
    assert G.remaining_arc(path, (2.0, 1.0)) == pytest.approx(1.0)
    assert G.interpolate_at(path, 3.0) == pytest.approx((2.0, 1.0))
    assert G.interpolate_at(path, 99.0) == pytest.approx((2.0, 2.0))


def test_scan_points_validity():
    r = [None] * C.LIDAR_SAMPLES
    r[360] = 1.0
    r[0] = 0.05
    r[180] = 30.0
    r[540] = float('nan')
    pts, valid = G.scan_points(r)
    assert valid.sum() == 1 and valid[360]
    assert pts[360] == pytest.approx((C.LIDAR_X + 1.0, 0.0), abs=1e-6)
    with pytest.raises(ValueError):
        G.scan_points([None] * 10)


def test_integrate_b_clamps():
    v, w, av, aw = K.integrate_b((1.0, 1.0), 0.97, 1.45, 1.0)
    assert (v, w) == pytest.approx((1.0, 1.5)) and av == pytest.approx(0.3)
    v, w, av, aw = K.integrate_b((-1.0, 0.0), 0.05, 0.0, 1.0)
    assert v == 0.0 and av == pytest.approx(-0.5)
    v, w, av, aw = K.integrate_b((5.0, -5.0), 0.0, 0.0, 1.0)   # 범위 밖 u 는 자른다
    assert av == pytest.approx(1.0) and aw == pytest.approx(-2.0)


def test_cells_and_fallback():
    assert K.cell_accel(0) == (-1.0, -2.0)
    assert K.cell_accel(98) == (1.0, 2.0)
    assert K.cell_accel(49) == (0.0, 0.0)
    with pytest.raises(ValueError):
        K.cell_accel(99)
    v, w, av, aw = K.integrate_c(C.FALLBACK_INDEX, 0.8, -0.6)
    assert (v, w) == pytest.approx((0.7, -0.4)) and (av, aw) == pytest.approx((-1.0, 2.0))
    v, w, _, _ = K.integrate_c(C.FALLBACK_INDEX, 0.05, 0.15)
    assert (v, w) == pytest.approx((0.0, 0.0))
    for i in range(C.N_CELLS):
        assert K.nearest_cell(*K.cell_accel(i)) == i


def test_stop_time():
    t, d = K.stop_time_and_distance(2.0, 1.0, 1e9)
    assert t == pytest.approx(2.0) and d == pytest.approx(2.0)
    t, d = K.stop_time_and_distance(2.0)     # 저크 2 m/s³
    assert t == pytest.approx(2.5) and d == pytest.approx(2.5)


def test_profiler_respects_limits():
    rng = np.random.default_rng(1)
    prof = P.ProfilerReference()
    v_cmd, w_cmd = 0.0, 0.0
    last_a = 0.0
    for _ in range(300):
        v1 = float(np.clip(v_cmd + rng.uniform(-1, 1) * C.DT, 0.0, 1.0))
        w1 = float(np.clip(w_cmd + rng.uniform(-2, 2) * C.DT, -1.5, 1.5))
        for k in range(1, 6):
            f = k / 5.0
            vr, wr = prof.step(v_cmd + f * (v1 - v_cmd), w_cmd + f * (w1 - w_cmd))
            a = prof.a_ref
            assert abs(a) <= C.A_MAX + 1e-9
            assert abs(a - last_a) <= C.JERK_MAX * C.PROFILER_DT + 1e-9
            assert abs(wr) <= C.W_MAX + 1e-9
            last_a = a
        v_cmd, w_cmd = v1, w1


def test_profiler_converges_and_preserves_curvature():
    prof = P.ProfilerReference()
    for _ in range(200):
        vr, wr = prof.step(0.8, 0.4)
    assert vr == pytest.approx(0.8, abs=1e-9) and wr == pytest.approx(0.4, abs=1e-9)
    prof.reset(0.2, 0.0, 0.1)
    vr, wr = prof.step(0.6, 0.3)
    assert wr == pytest.approx(min(0.5 * vr, 0.1 + C.ALPHA_MAX * C.PROFILER_DT))


def test_safety_clearance_limits_from_config_comment():
    # R:137-138: D = 2.6 → 2.0, 1.0 → 1.04, 0.5 → 0.50
    assert S.clearance_speed_limit(2.6) == pytest.approx(2.0, abs=0.01)
    assert S.clearance_speed_limit(1.0) == pytest.approx(1.04, abs=0.01)
    assert S.clearance_speed_limit(0.5) == pytest.approx(0.50, abs=0.01)
    assert S.clearance_speed_limit(0.2) == 0.0


def test_safety_filter_rules():
    out = S.safety_filter(1.0, 0.0, None, None, False)
    assert (out['v'], out['w'], out['zone']) == (1.0, 0.0, C.ZONE_CLEAR)
    out = S.safety_filter(0.8, 0.4, 0.8, None, False)            # WARNING: 점 속도 0.5
    assert out['zone'] == C.ZONE_WARNING
    assert S.max_point_speed(out['v'], out['w']) == pytest.approx(0.5)
    assert out['w'] / out['v'] == pytest.approx(0.5)                # 곡률 보존
    out = S.safety_filter(0.9, 0.0, 3.0, 1.2, False)              # TTC: v ≤ a(TTC − 0.15)
    assert out['v'] == pytest.approx(1.05 if 0.9 > 1.05 else 0.9)
    out = S.safety_filter(0.9, 0.0, 3.0, 0.5, False)
    assert out['v'] == pytest.approx(0.35)
    out = S.safety_filter(0.3, 0.5, 0.25, None, True)
    assert (out['v'], out['w'], out['zone']) == (0.0, 0.0, C.ZONE_STOP)
    out = S.safety_filter(0.3, 0.0, 0.28, None, False)            # 확정 전 STOP → CRITICAL, 상한 0
    assert out['zone'] == C.ZONE_CRITICAL and out['v'] == 0.0


def test_safety_latched_stop_holds_beyond_stop_distance():
    # 래치는 해제 거리(0.50, P:223)까지 유지된다 — D 가 0.30 을 넘어도 정지
    out = S.safety_filter(0.4, 0.2, 0.45, None, True)
    assert (out['v'], out['w'], out['zone']) == (0.0, 0.0, C.ZONE_STOP)


def _brute_label(a, mask, fb):
    cands = [(K.accel_distance(K.cell_accel(i), a), i) for i in range(C.N_CELLS) if mask[i]]
    if mask[C.FALLBACK_INDEX]:
        cands.append((K.accel_distance(fb, a), C.FALLBACK_INDEX))
    d0 = min(d for d, _ in cands)
    return min(i for d, i in cands if d <= d0 + 1e-12)


def test_bc_label_is_masked_nearest_random():
    rng = np.random.default_rng(17)
    for _ in range(400):
        mask = rng.random(C.N_ACTIONS_C) < rng.uniform(0.02, 0.5)
        mask[C.FALLBACK_INDEX] = True
        v_cmd, w_cmd = float(rng.uniform(0, 1)), float(rng.uniform(-1.5, 1.5))
        v_dwa = v_cmd + float(rng.uniform(-0.1, 0.1))
        w_dwa = w_cmd + float(rng.uniform(-0.2, 0.2))
        lab = K.bc_label(v_dwa, w_dwa, v_cmd, w_cmd, mask)
        assert mask[lab]
        a = ((v_dwa - v_cmd) / C.DT, (w_dwa - w_cmd) / C.DT)
        fb = K.fallback_command(v_cmd, w_cmd)[2:]
        assert lab == _brute_label(a, mask, fb)
        un = K.nearest_cell(*a)
        if mask[un] and K.accel_distance(fb, a) > K.accel_distance(K.cell_accel(un), a) + 1e-12:
            assert lab == un                  # DWA 셀이 마스크 안이면 그대로


def test_bc_label_dwa_accel_start_is_not_fallback():
    # 리뷰 r0 #3: DWA 는 계획기에서 저크를 걸지 않아 정지·순항에서 a 1.0 / α 2.0 을 바로 낸다.
    # v1.0.0 규칙(격자 최근접이 마스크 밖이면 폴백)은 이것을 '제동' 으로 레이블했다.
    from amr_rl_contract import mask as M
    from amr_rl_contract.state import empty_scan, State
    path = [(0.0, 0.0), (10.0, 0.0)]
    for v_cmd, dwa, want in [(0.0, (0.1, 0.0), (0.2, 0.0)), (0.5, (0.6, 0.0), (0.2, 0.0)),
                             (0.5, (0.5, 0.2), (0.0, 0.5))]:
        st = State(pose=(0.0, 0.0, 0.0), path=path, scan_ranges=empty_scan(), v_cmd=v_cmd)
        m = M.compute_mask(st)['mask']
        lab = K.bc_label(*dwa, st.v_cmd, st.w_cmd, m)
        assert lab != C.FALLBACK_INDEX and K.cell_accel(lab) == pytest.approx(want)
        assert not m[K.nearest_cell((dwa[0] - v_cmd) / C.DT, dwa[1] / C.DT)]


def test_nearest_cell_mask_arguments():
    mask = [False] * C.N_ACTIONS_C
    with pytest.raises(ValueError):
        K.nearest_cell(0.0, 0.0, mask)                       # 후보 없음
    mask[C.FALLBACK_INDEX] = True
    with pytest.raises(ValueError):
        K.nearest_cell(0.0, 0.0, mask)                       # 폴백 가속도 없음
    assert K.nearest_cell(0.0, 0.0, mask, (-1.0, 0.0)) == C.FALLBACK_INDEX
    with pytest.raises(ValueError):
        K.nearest_cell(0.0, 0.0, mask[:-1], (-1.0, 0.0))     # 길이
    mask[49] = True                                          # (0, 0) 셀과 폴백 (0, 0) 동률 → 셀
    assert K.nearest_cell(0.0, 0.0, mask, (0.0, 0.0)) == 49
