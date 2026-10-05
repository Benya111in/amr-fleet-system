"""kinematics: 대역 시뮬레이터 운동학·레이캐스트·풋프린트 여유."""

import math

from amr_itest import kinematics as km
import numpy as np
import pytest


def test_limits_from_params():
    lim = km.Limits.from_params({'limits': {'max_linear_velocity': 1.2}})
    assert lim.v_max == 1.2 and lim.a_max == 1.0


def test_step_respects_acceleration_and_integrates_arc():
    s = km.State()
    s, ax, _ = km.step(s, 2.0, 0.0, 0.1)
    assert s.v == pytest.approx(0.1) and ax == pytest.approx(1.0)
    s = km.State(v=0.5, w=0.5)
    for _ in range(round(2 * math.pi / 0.5 / 0.01)):
        s, _, ay = km.step(s, 0.5, 0.5, 0.01)
    assert ay == pytest.approx(0.25)
    assert math.hypot(s.x, s.y) == pytest.approx(0.0, abs=5e-3)       # 반지름 1 m 원 한 바퀴
    same, ax0, _ = km.step(s, 1.0, 0.0, 0.0)
    assert same.x == s.x and ax0 == 0.0
    clipped, _, _ = km.step(km.State(v=1.9), 5.0, 9.0, 1.0)
    assert clipped.v == pytest.approx(2.0) and clipped.w == pytest.approx(1.5)


def test_wheel_kinematics_roundtrip():
    wl, wr = km.wheel_rates(0.5, 0.3, 0.0825, 0.36)
    ds, dth = km.body_increment(wl * 0.1, wr * 0.1, 0.0825, 0.36)
    assert ds == pytest.approx(0.05) and dth == pytest.approx(0.03)
    x, y, yaw = km.integrate_pose(0.0, 0.0, 0.0, 1.0, math.pi / 2)
    assert (x, y) == pytest.approx((math.cos(math.pi / 4), math.sin(math.pi / 4)))
    assert yaw == pytest.approx(math.pi / 2)
    tick = 2 * math.pi / 4096
    assert km.quantize(10.5 * tick, 4096) == pytest.approx(10 * tick)
    assert km.wrap(3 * math.pi) == pytest.approx(math.pi)
    assert km.approach(0.0, -1.0, 0.3) == pytest.approx(-0.3)


def test_raycast_circle_box_and_inside():
    ang = km.beam_angles(-math.pi, math.pi - 2 * math.pi / 8, 8)
    assert len(ang) == 8 and km.beam_angles(0.0, 1.0, 1).tolist() == [0.0]
    r = km.raycast(0.0, 0.0, 0.0, ang, circles=[(3.0, 0.0, 0.5)], boxes=[(-2.0, -1.0, -1.5, 1.0)])
    assert r[4] == pytest.approx(2.5)                 # 0 rad → 원
    assert r[0] == pytest.approx(1.5)                 # π → 박스
    assert math.isinf(r[2])                           # −π/2 → 없음
    inside = km.raycast(3.0, 0.0, 0.0, np.array([0.0]), circles=[(3.0, 0.0, 0.5)])
    assert inside[0] == pytest.approx(0.0)
    in_box = km.raycast(0.0, 0.0, 0.0, np.array([0.0, math.pi / 2]),
                        boxes=[(-1.0, -1.0, 1.0, 1.0)])
    assert in_box.tolist() == pytest.approx([0.0, 0.0])
    far = km.raycast(0.0, 0.0, 0.0, np.array([0.0]), circles=[(30.0, 0.0, 0.5)], range_max=25.0)
    assert math.isinf(far[0])


def test_footprint_clearance_and_transforms():
    pts = np.array([[0.6, 0.0], [0.0, 0.5], [0.4, 0.3]])
    assert km.footprint_clearance(pts, 0.6, 0.4) == pytest.approx(0.1414, abs=1e-3)
    assert math.isinf(km.footprint_clearance(np.zeros((0, 2)), 0.6, 0.4))
    scan_pts = km.scan_to_points([1.0, float('inf'), 2.0], np.array([0.0, 1.0, math.pi / 2]),
                                 (0.15, 0.0, 0.0))
    np.testing.assert_allclose(scan_pts, [[1.15, 0.0], [0.15, 2.0]], atol=1e-9)
    outline = km.box_outline((0.0, 0.0, 1.0, 0.5), step=0.25)
    assert outline[:, 0].min() == 0.0 and outline[:, 1].max() == 0.5
    w = km.base_to_world(np.array([[1.0, 0.0]]), 2.0, 3.0, math.pi / 2)
    assert w[0].tolist() == pytest.approx([2.0, 4.0])
    assert km.world_to_base(w, 2.0, 3.0, math.pi / 2)[0].tolist() == pytest.approx([1.0, 0.0])
    c = km.clearance_to_boxes(km.State(0.0, 0.0, 0.0), [(1.3, -0.5, 1.5, 0.5)], 0.6, 0.4)
    assert c == pytest.approx(1.0, abs=1e-6)
    assert math.isinf(km.clearance_to_boxes(km.State(), [], 0.6, 0.4))
    assert km.yaw_quaternion(math.pi)[2] == pytest.approx(1.0)
    rng = np.random.default_rng(0)
    assert km.gaussian(rng, 0.0) == 0.0 and km.gaussian(rng, 0.0, 3).tolist() == [0, 0, 0]
    assert km.gaussian(rng, 1.0, 4).shape == (4,)
