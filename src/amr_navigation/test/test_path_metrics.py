"""경로·주행 지표(길이, 여유거리, CTE, 곡률, 주행 시간 예측, 저크) 단위 테스트."""
import math

from amr_navigation import path_metrics as pm
import numpy as np
import pytest


def straight(length, step=0.05):
    n = int(round(length / step))
    return np.column_stack([np.arange(n + 1) * step, np.zeros(n + 1)])


def arc(radius, angle, step=0.05):
    n = int(round(angle * radius / step))
    a = np.arange(n + 1) * step / radius
    return np.column_stack([radius * np.sin(a), radius * (1.0 - np.cos(a))])


def test_length_and_cumulative():
    xy = straight(4.0)
    assert pm.path_length(xy) == pytest.approx(4.0)
    s = pm.cumulative_length(xy)
    assert s[0] == 0.0 and s[-1] == pytest.approx(4.0)
    assert pm.path_length(np.zeros((1, 2))) == 0.0
    assert len(pm.cumulative_length(np.zeros((0, 2)))) == 0


def test_cte_sign_left_positive():
    xy = straight(4.0)
    assert pm.cross_track_error(xy, 1.0, 0.3) == pytest.approx(0.3)
    assert pm.cross_track_error(xy, 2.0, -0.2) == pytest.approx(-0.2)
    assert math.isnan(pm.cross_track_error(np.zeros((1, 2)), 0.0, 0.0))


def test_curvature_of_circle():
    k = pm.discrete_curvature(arc(2.0, 2.0), 0.25)
    assert np.allclose(k[10:-10], 0.5, atol=1e-3)
    assert k[0] == 0.0 and k[-1] == 0.0
    assert np.all(pm.discrete_curvature(np.zeros((2, 2))) == 0.0)


def test_min_clearance():
    dist = np.full((10, 20), 1.0)
    dist[5, 10] = 0.2
    xy = np.array([[0.05, 0.55], [1.95, 0.55]])   # 격자 0.1 m, 행 5 를 지난다
    assert pm.min_clearance(xy, dist, 0.1, (0.0, 0.0)) == pytest.approx(0.2)
    assert math.isnan(pm.min_clearance(np.zeros((0, 2)), dist, 0.1, (0.0, 0.0)))


def test_rotation_time():
    assert pm.rotation_time(math.pi / 2, 1.5, 2.0) == pytest.approx(math.pi / 3 + 0.75)
    assert pm.rotation_time(-0.5, 1.5, 2.0) == pytest.approx(1.0)
    assert pm.rotation_time(0.0, 1.5, 2.0) == 0.0


def test_travel_time_matches_trapezoid_and_cpp():
    # C++ SpeedProfile 단위 테스트와 같은 값: 4 m 직선, v 1, a = d = 1 → 5 s + a/j 0.5
    xy = straight(4.0)
    assert pm.predict_travel_time(xy) == pytest.approx(5.5, abs=1e-6)
    lim = pm.ProfileLimits(max_jerk=0.0)
    assert pm.predict_travel_time(xy, lim) == pytest.approx(5.0, abs=1e-6)
    assert pm.predict_travel_time(xy, lim, 0.0, math.pi / 2, 0.5) == pytest.approx(
        5.0 + math.pi / 3 + 0.75 + 1.0, abs=1e-6)
    assert pm.predict_travel_time(np.zeros((1, 2)), lim, 0.0, 0.5) == pytest.approx(1.0)


def test_travel_time_slower_on_curves():
    # R = 1 m 곡선: v ≤ √(0.8) → 같은 길이 직선보다 오래 걸린다
    a = arc(1.0, math.pi)
    s = straight(pm.path_length(a))
    assert pm.predict_travel_time(a) > pm.predict_travel_time(s) + 0.1


def test_jerk_from_velocity():
    t = np.arange(0.0, 2.0, 0.02)
    v = 0.5 * t ** 2        # a = t, j = 1
    j = pm.jerk_from_velocity(t, v)
    assert np.allclose(j, 1.0, atol=1e-6)
    assert len(pm.jerk_from_velocity(t[:2], v[:2])) == 0
    js = pm.jerk_from_velocity(t, v, smooth=5)
    assert np.allclose(js[5:-5], 1.0, atol=1e-6)
