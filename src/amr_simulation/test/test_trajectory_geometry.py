"""actor 궤적 보간(Gazebo 스플라인 규칙)과 발자국 부호 거리 단위 테스트."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from amr_simulation.actor_trajectory import ActorTrajectory  # noqa: E402
from amr_simulation.dynamic_obstacles import ObstacleSet  # noqa: E402
from amr_simulation.footprint import Rect, point_rect, rect_circle, rect_rect  # noqa: E402


def _line(n=6, dt=0.5, v=1.0):
    """등시간 간격 직선 왕복 (x 0 → n·v·dt → 0), 닫힌 루프."""
    fwd = [(i * dt, i * v * dt, 0.0, 0.0, 0.0) for i in range(n + 1)]
    back = [(n * dt + i * dt, (n - i) * v * dt, 0.0, 0.0, math.pi) for i in range(1, n + 1)]
    return ActorTrajectory("line", fwd + back, tension=0.0)


def test_knots_are_reproduced():
    a = _line()
    for t, x, y, _z, _yaw in a.waypoints:
        px, py, *_ = a.state(t)
        assert (px, py) == pytest.approx((x, y), abs=1e-9)


def test_uniform_interior_segment_has_constant_speed():
    a = _line(n=8, dt=0.5, v=1.2)
    for t in (1.1, 1.3, 1.77, 2.2):               # 끝점에서 떨어진 구간: 접선 = 현 → 정확히 등속
        _, _, _, vx, vy = a.state(t)
        assert vx == pytest.approx(1.2, abs=1e-9) and vy == pytest.approx(0.0, abs=1e-12)


def test_loop_wraps_and_delay():
    a = _line()
    T = a.duration
    assert a.state(0.37)[:2] == pytest.approx(a.state(0.37 + 3 * T)[:2])
    b = ActorTrajectory("d", a.waypoints, delay_start=2.0)
    assert b.state(1.0)[:2] == pytest.approx(a.state(0.0)[:2])
    assert b.state(2.37)[:2] == pytest.approx(a.state(0.37)[:2])


def test_closed_curve_tangent_continuity():
    n, r = 24, 2.0
    th = [2 * math.pi * (i % n) / n for i in range(n + 1)]
    wps = [(i * 0.5, r * math.cos(a), r * math.sin(a), 0.0, 0.0) for i, a in enumerate(th)]
    a = ActorTrajectory("circle", wps)
    v0 = a.state(0.0 + 1e-9)[3:]
    v1 = a.state(a.duration - 1e-9)[3:]
    assert v0 == pytest.approx(v1, abs=1e-6)       # 닫힌 곡선: 끝 접선 = 첫 접선
    for t in (0.1, 3.3, 7.9):
        x, y, *_ = a.state(t)
        assert math.hypot(x, y) == pytest.approx(r, rel=0.01)


def test_tension_one_stops_at_knots():
    wps = [(0.0, 0, 0, 0, 0), (1.0, 1, 0, 0, 0), (2.0, 2, 0, 0, 0)]
    a = ActorTrajectory("t1", wps, tension=1.0, loop=False)
    assert a.state(1.0)[3] == pytest.approx(0.0, abs=1e-9)
    assert a.state(0.5)[3] == pytest.approx(1.5)     # smoothstep 중간 속도 = 평균의 1.5 배


def test_point_and_circle_distance():
    r = Rect(0, 0, 0, -0.3, 0.3, -0.2, 0.2)
    assert point_rect(1.0, 0.0, r) == pytest.approx(0.7)
    assert point_rect(0.0, 0.0, r) == pytest.approx(-0.2)
    assert rect_circle(r, 0.8, 0.0, 0.3) == pytest.approx(0.2)
    assert rect_circle(r, 0.5, 0.0, 0.3) == pytest.approx(-0.1)
    rr = Rect(1, 1, math.pi / 2, -0.3, 0.3, -0.2, 0.2)     # 90도 돌린 로봇: x 폭 0.4, y 길이 0.6
    assert point_rect(1.0, 2.0, rr) == pytest.approx(0.7)
    assert point_rect(1.5, 1.0, rr) == pytest.approx(0.3)


def test_rect_rect_distance():
    a = Rect(0, 0, 0, -0.3, 0.3, -0.2, 0.2)
    assert rect_rect(a, Rect(1.0, 0, 0, -0.3, 0.3, -0.2, 0.2)) == pytest.approx(0.4)
    assert rect_rect(a, Rect(0.5, 0, 0, -0.3, 0.3, -0.2, 0.2)) == pytest.approx(-0.1)
    d = rect_rect(a, Rect(1.0, 1.0, 0, -0.3, 0.3, -0.2, 0.2))      # 모서리끼리
    assert d == pytest.approx(math.hypot(0.4, 0.6))
    rot = Rect(0.3 + 0.2 * math.sqrt(2) + 0.1, 0, math.pi / 4, -0.2, 0.2, -0.2, 0.2)
    assert rect_rect(a, rot) == pytest.approx(0.1, abs=1e-9)          # 45도 돌린 정사각형 꼭짓점 → 변


def test_obstacle_set_extrapolates_models():
    s = ObstacleSet([], 0.3, 1.8, {"cart": (-0.3, 0.3, -0.2, 0.2)}, {"cart": 0.5})
    assert s.states(1.0) == []                       # 오도메트리 전에는 없음
    s.update_model("cart", 10.0, 1.0, 2.0, math.pi / 2, 1.0, 0.0, 0.0)   # 몸체 전진 1 m/s, 북쪽을 봄
    st = s.states(10.1)[0]
    assert (st.x, st.y) == pytest.approx((1.0, 2.1))
    assert (st.vx, st.vy) == pytest.approx((0.0, 1.0), abs=1e-12)
    assert st.heading == pytest.approx(math.pi / 2)
