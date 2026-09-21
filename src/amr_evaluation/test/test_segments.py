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
])
def test_classify_motion(v, w, expected):
    assert segments.classify_motion(v, w) == expected


def test_classify_motion_custom_thresholds():
    th = segments.MotionThresholds(stop_linear=0.1, stop_angular=0.1, straight_angular=0.3)
    assert segments.classify_motion(0.05, 0.05, th) == segments.STOP
    assert segments.classify_motion(1.0, 0.2, th) == segments.STRAIGHT
    assert segments.classify_motion(1.0, 0.4, th) == segments.TURN


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


def test_path_curvature_circle_and_line():
    circle = _circle(2.0, 253, span=2 * math.pi)     # 정점 간격 ≈ 5 cm
    kappa = segments.path_curvature(circle, window=0.5)
    assert np.allclose(kappa, 0.5, rtol=0.02)
    line = np.column_stack([np.linspace(0, 5, 101), np.zeros(101)])
    assert np.allclose(segments.path_curvature(line), 0.0)
    assert segments.path_curvature(line[:2]).tolist() == [0.0, 0.0]
    assert segments.path_curvature(np.zeros((0, 2))).size == 0


def _straight_arc_straight(step: float = 0.05):
    s1 = np.column_stack([np.arange(0.0, 5.0, step), np.zeros(int(5.0 / step))])
    arc = np.column_stack([5.0 + np.sin(np.linspace(0, math.pi / 2, 32)),
                           1.0 - np.cos(np.linspace(0, math.pi / 2, 32))])
    s2 = np.column_stack([6.0 * np.ones(100), 1.0 + np.arange(step, 5.0 + step, step)])
    return np.vstack([s1, arc, s2]), len(s1), len(arc)


def test_label_path_segments_straight_arc_straight():
    pts, n1, n_arc = _straight_arc_straight()
    labels = segments.label_path_segments(pts, curvature_threshold=0.1, window=0.5, margin=0.5)
    assert len(labels) == len(pts)
    assert labels[0] == segments.STRAIGHT
    assert labels[n1 + n_arc // 2] == segments.CURVE
    assert labels[-1] == segments.STRAIGHT
    n_curve = labels.count(segments.CURVE)
    # 호 1.57 m + 앞뒤 천이대 0.5 m + 곡률 창 번짐 — 5 cm 간격이면 30 ~ 75 개
    assert 30 <= n_curve <= 75
    assert segments.label_path_segments(np.zeros((0, 2))) == []


def test_label_without_margin_is_tighter():
    pts, _, _ = _straight_arc_straight()
    with_margin = segments.label_path_segments(pts, margin=0.5)
    without = segments.label_path_segments(pts, margin=0.0)
    assert without.count(segments.CURVE) < with_margin.count(segments.CURVE)


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
