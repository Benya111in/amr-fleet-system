import math

from amr_evaluation import segments
import numpy as np
import pytest


def _circle(r: float, n: int, start: float = 0.0, span: float = 2 * math.pi, ccw: bool = True):
    ang = start + np.linspace(0.0, span, n) * (1.0 if ccw else -1.0)
    return np.column_stack([r * np.cos(ang), r * np.sin(ang)])


@pytest.mark.parametrize('v, w, expected', [
    (0.0, 0.0, segments.STOP),
    (0.019, -0.019, segments.STOP),
    (0.5, 0.0, segments.STRAIGHT),
    (0.5, 0.049, segments.STRAIGHT),
    (0.02, 0.0, segments.STRAIGHT),      # 정지 임계 경계값은 정지가 아니다
    (0.5, 0.05, segments.TURN),
    (0.0, -0.06, segments.TURN),
    (2.0, 1.5, segments.TURN),
    # 느린 제자리 회전 (v ≈ 0, 0.02 ≤ |ω| < 0.05) 은 직선이 아니라 회전
    (0.0, 0.03, segments.TURN),
    (0.0, 0.045, segments.TURN),
    (0.01, -0.04, segments.TURN),
])
def test_classify_motion(v, w, expected):
    assert segments.classify_motion(v, w) == expected


def test_classify_motion_custom_thresholds():
    th = segments.MotionThresholds(stop_linear=0.1, stop_angular=0.1, straight_angular=0.3)
    assert segments.classify_motion(0.05, 0.05, th) == segments.STOP
    assert segments.classify_motion(1.0, 0.2, th) == segments.STRAIGHT
    assert segments.classify_motion(1.0, 0.4, th) == segments.TURN
    assert segments.classify_motion(0.05, 0.2, th) == segments.TURN


def test_cumulative_length():
    pts = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float)
    assert segments.cumulative_length(pts).tolist() == [0.0, 1.0, 2.0, 3.0]
    assert segments.cumulative_length(np.zeros((0, 2))).size == 0


def test_menger_curvature_sign_and_magnitude():
    pts = _circle(2.0, 3, span=1.0)
    assert segments.menger_curvature(*pts) == pytest.approx(0.5)
    pts_cw = _circle(2.0, 3, span=1.0, ccw=False)
    assert segments.menger_curvature(*pts_cw) == pytest.approx(-0.5)
    assert segments.menger_curvature([0, 0], [1, 0], [2, 0]) == 0.0
    assert segments.menger_curvature([0, 0], [0, 0], [2, 0]) == 0.0


def test_resample_path_drops_duplicates_and_is_uniform():
    pts = np.array([[0, 0], [0, 0], [0.5, 0], [0.5, 0], [0.5, 0.5]], dtype=float)
    out, s = segments.resample_path(pts, 0.05)
    assert len(out) == 21 and s[-1] == pytest.approx(1.0)
    assert np.allclose(np.diff(s), 0.05)
    assert out[10].tolist() == pytest.approx([0.5, 0.0])
    single, s1 = segments.resample_path(np.array([[1.0, 2.0]]))
    assert single.tolist() == [[1.0, 2.0]] and s1.tolist() == [0.0]


def test_smooth_path_keeps_lines_and_endpoints():
    line = np.column_stack([np.linspace(0, 3, 61), 0.5 * np.linspace(0, 3, 61)])
    assert np.allclose(segments.smooth_path(line, 4.0), line)      # 점대칭 패딩: 직선 그대로
    assert np.allclose(segments.smooth_path(line, 0.0), line)
    zig = np.column_stack([np.arange(40) * 0.05, np.tile([0.0, 0.02], 20)])
    sm = segments.smooth_path(zig, 4.0)
    assert np.ptp(sm[10:30, 1]) < 0.002                               # 지그재그가 펴진다


def test_path_curvature_circle_and_line():
    circle = _circle(2.0, 253, span=2 * math.pi)     # 정점 간격 ≈ 5 cm
    kappa = segments.path_curvature(circle)
    # 열린 경로의 끝 1 m 는 끝 방향 직선으로 늘여 계산하므로 안쪽만 정확하다
    assert np.allclose(kappa[25:-25], 0.5, rtol=0.02)
    assert np.all(kappa > 0.05)
    line = np.column_stack([np.linspace(0, 5, 101), np.zeros(101)])
    assert np.allclose(segments.path_curvature(line), 0.0)
    assert segments.path_curvature(line[:2]).tolist() == [0.0, 0.0]
    assert segments.path_curvature(np.zeros((0, 2))).size == 0
    # 창(1 m)보다 짧은 경로는 곡률을 정하지 않는다 (0)
    short = _circle(0.3, 20, span=math.pi / 2)
    assert np.all(segments.path_curvature(short) == 0.0)


def _staircase(angle_deg: float, length: float, res: float = 0.05) -> np.ndarray:
    """격자 플래너(8-연결 A*) 출력처럼 5 cm 격자 칸 중심을 잇는 계단 직선 (Bresenham)."""
    n1 = int(round(length * math.cos(math.radians(angle_deg)) / res))
    m1 = int(round(length * math.sin(math.radians(angle_deg)) / res))
    dx, dy = abs(n1), abs(m1)
    sx, sy = (1 if n1 >= 0 else -1), (1 if m1 >= 0 else -1)
    err, x, y, pts = dx - dy, 0, 0, [(0, 0)]
    while (x, y) != (n1, m1):
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
        pts.append((x, y))
    return np.array(pts, dtype=float) * res


def _snapped_line(angle_deg: float, length: float, res: float = 0.05, step: float = 0.05):
    s = np.arange(0.0, length, step)
    x, y = s * math.cos(math.radians(angle_deg)), s * math.sin(math.radians(angle_deg))
    return np.column_stack([np.round(x / res) * res, np.round(y / res) * res])


@pytest.mark.parametrize('angle', [0, 1, 2, 5, 10, 20, 26.57, 30, 45, 60, 88, 135, 200, 271])
@pytest.mark.parametrize('length', [1.5, 6.0, 20.0])
def test_grid_staircase_is_straight(angle, length):
    """5 cm 격자 계단 경로(축·45° 가 아닌 직선)는 전부 직선이어야 한다."""
    for pts in (_staircase(angle, length), _snapped_line(angle, length)):
        labels = segments.label_path_segments(pts)
        assert labels.count(segments.CURVE) == 0, (angle, length)


@pytest.mark.parametrize('sd', [0.002, 0.005, 0.01])
def test_mm_jittered_line_is_straight(sd):
    rng = np.random.default_rng(3)
    s = np.arange(0.0, 6.0, 0.05)
    for _ in range(10):
        pts = np.column_stack([s, rng.normal(0.0, sd, s.size)])
        assert segments.label_path_segments(pts).count(segments.CURVE) == 0


def _straight_arc_straight(radius: float = 1.0, deg: float = 90.0, step: float = 0.05):
    s1 = np.column_stack([np.arange(0.0, 5.0, step), np.zeros(int(round(5.0 / step)))])
    n_arc = max(int(round(radius * math.radians(deg) / step)), 2) + 1
    th = np.linspace(0, math.radians(deg), n_arc)
    arc = np.column_stack([5.0 + radius * np.sin(th), radius - radius * np.cos(th)])
    d = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])
    s2 = arc[-1] + d * step * np.arange(1, 101)[:, None]
    return np.vstack([s1, arc[1:], s2]), len(s1), n_arc - 1


def test_label_path_segments_straight_arc_straight():
    pts, n1, n_arc = _straight_arc_straight()
    labels = segments.label_path_segments(pts, curvature_threshold=0.1, window=1.0, margin=0.5)
    assert len(labels) == len(pts)
    assert labels[0] == segments.STRAIGHT
    assert labels[n1:n1 + n_arc] == [segments.CURVE] * n_arc
    assert labels[-1] == segments.STRAIGHT
    n_curve = labels.count(segments.CURVE)
    # 호 1.57 m + 앞뒤 천이대 0.5 m + 곡률 창 번짐 — 5 cm 간격이면 30 ~ 75 개
    assert 30 <= n_curve <= 75
    assert segments.label_path_segments(np.zeros((0, 2))) == []
    assert segments.label_path_segments(pts[:2]) == [segments.STRAIGHT] * 2


@pytest.mark.parametrize('radius', [0.3, 0.5, 2.0, 5.0, 8.0])
@pytest.mark.parametrize('deg', [45, 90, 180])
def test_arcs_are_curves(radius, deg):
    pts, n1, n_arc = _straight_arc_straight(radius, deg)
    labels = segments.label_path_segments(pts)
    assert labels[n1:n1 + n_arc].count(segments.CURVE) == n_arc
    assert labels[:n1 - 40].count(segments.CURVE) == 0


def test_quantised_arc_is_still_curve_and_start_arc_detected():
    pts, n1, n_arc = _straight_arc_straight(1.0, 90)
    q = np.round(pts / 0.05) * 0.05
    labels = segments.label_path_segments(q)
    assert labels[n1 + 3:n1 + n_arc - 3].count(segments.CURVE) == n_arc - 6
    # 경로가 곡선으로 시작해도 끝 창을 줄이지 않고 곡선으로 잡는다
    th = np.linspace(0, math.pi / 2, 16)
    arc = np.column_stack([0.5 * np.sin(th), 0.5 - 0.5 * np.cos(th)])
    tail = np.column_stack([0.5 * np.ones(60), 0.5 + np.arange(1, 61) * 0.05])
    labels = segments.label_path_segments(np.vstack([arc, tail]))
    assert labels[:16] == [segments.CURVE] * 16 and labels[-20:] == [segments.STRAIGHT] * 20


def test_min_turn_filters_small_bumps():
    # 5 m 직선 위 3 cm 짜리 완만한 요철 (회전각 수 도) → min_turn 10° 로 직선, 0 이면 곡선
    x = np.arange(0.0, 8.0, 0.05)
    y = 0.03 * np.exp(-((x - 4.0) / 0.3) ** 2)
    pts = np.column_stack([x, y])
    assert segments.label_path_segments(pts).count(segments.CURVE) == 0
    assert segments.label_path_segments(pts, min_turn=0.0).count(segments.CURVE) > 0


def test_label_without_margin_is_tighter():
    pts, _, _ = _straight_arc_straight()
    with_margin = segments.label_path_segments(pts, margin=0.5)
    without = segments.label_path_segments(pts, margin=0.0)
    assert without.count(segments.CURVE) < with_margin.count(segments.CURVE)


def test_curvature_profile_short_and_degenerate():
    s, k = segments.curvature_profile(np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]))
    assert np.all(k == 0.0)
    s, k = segments.curvature_profile(_circle(2.0, 253))
    assert len(s) == len(k) and s[-1] == pytest.approx(4 * math.pi, rel=1e-3)


def test_dilate_curve_labels():
    labels = [segments.STRAIGHT] * 10 + [segments.CURVE] + [segments.STRAIGHT] * 10
    cum = np.arange(21) * 0.1
    out = segments.dilate_curve_labels(labels, cum, margin=0.25)
    assert out.count(segments.CURVE) == 5
    assert out[8:13] == [segments.CURVE] * 5
    assert segments.dilate_curve_labels(labels, cum, margin=0.0) == labels
    assert segments.dilate_curve_labels([segments.STRAIGHT] * 3, cum[:3], 1.0) == \
        [segments.STRAIGHT] * 3
    assert segments.dilate_curve_labels([], cum[:0], 1.0) == []


def test_segment_label_for_index():
    labels = [segments.STRAIGHT, segments.CURVE, segments.STRAIGHT]
    assert segments.segment_label_for_index(labels, 0) == segments.CURVE
    assert segments.segment_label_for_index(labels, 1) == segments.CURVE
    assert segments.segment_label_for_index(labels, 2) == segments.STRAIGHT
    assert segments.segment_label_for_index(labels, 99) == segments.STRAIGHT
    assert segments.segment_label_for_index([], 0) == segments.STRAIGHT


def test_yaw_rate_from_headings_wraps():
    assert segments.yaw_rate_from_headings(3.1, -3.1, 0.1) == pytest.approx(
        (2 * math.pi - 6.2) / 0.1)
    assert segments.yaw_rate_from_headings(0.0, 1.0, 0.0) == 0.0
