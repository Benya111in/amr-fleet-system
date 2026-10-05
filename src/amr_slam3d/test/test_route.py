"""경로 추종(route.py) 시험 — 이상 유니사이클로 끝까지 돌려 웨이포인트 도달·횡오차·한계를 본다."""

import math
from pathlib import Path

from amr_slam3d.route import Limits, route_from_dict, RouteFollower, RouteSpec, simulate
from amr_slam3d.se2 import path_length
import pytest
import yaml

PILOT = Path(__file__).resolve().parents[1] / 'config' / 'pilot_route.yaml'


def _reached(log, wp, tol):
    return min(math.hypot(x - wp[0], y - wp[1]) for _, x, y, _, _ in log) <= tol


def _square():
    return RouteSpec(waypoints=[(4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)],
                     hold_start_s=2.0, hold_end_s=1.0, dwell_s=0.5)


def test_square_reaches_every_waypoint_in_order():
    spec = _square()
    log = simulate(spec, (0.0, 0.0, 0.0))
    assert log[-1][4].done
    for wp in spec.waypoints:
        assert _reached(log, wp, spec.limits.goal_tol + 0.02)
    idx = [c.index for *_, c in log]
    assert idx == sorted(idx)                      # 웨이포인트 순서를 거꾸로 가지 않는다
    x, y = log[-1][1], log[-1][2]
    assert math.hypot(x, y) < 0.1


def test_hold_start_keeps_robot_still():
    spec = _square()
    log = simulate(spec, (0.0, 0.0, 0.0))
    for t, _, _, _, c in log:
        if t < spec.hold_start_s - 1e-9:
            assert c.v == 0.0 and c.w == 0.0 and c.phase == 'hold_start'


def test_commands_respect_limits_and_drive_has_no_reverse():
    spec = _square()
    log = simulate(spec, (0.0, 0.0, 0.0))
    lim = spec.limits
    for *_, c in log:
        assert 0.0 <= c.v <= lim.v_max + 1e-12
        assert abs(c.w) <= lim.w_max + 1e-12
        if c.phase == 'rotate':
            assert c.v == 0.0                      # 제자리 회전 중에는 직진하지 않는다


def test_acceleration_limited():
    spec = _square()
    log = simulate(spec, (0.0, 0.0, 0.0), dt=0.02)
    vs = [c.v for *_, c in log]
    dv = max(b - a for a, b in zip(vs, vs[1:]))
    assert dv <= spec.limits.a_max * 0.02 + 1e-9


def test_cross_track_bounded_under_turn_bias():
    """실제 각속도에 0.02 rad/s 치우침 + 3 % 미끄러짐이 있어도 선분에서 5 cm 안 (정상 상태 ≈ bias/k_e)."""
    spec = RouteSpec(waypoints=[(10.0, 0.0)], hold_start_s=0.0, hold_end_s=0.0, dwell_s=0.0)
    log = simulate(spec, (0.0, 0.0, 0.0), slip=0.03, w_bias=0.02)
    late = [abs(y) for t, x, y, _, c in log if c.phase == 'drive' and x > 3.0]
    assert late and max(late) < 0.05
    assert _reached(log, (10.0, 0.0), spec.limits.goal_tol + 0.02)


def test_rotation_takes_shortest_direction_across_pi():
    """방위 3.1 에서 −3.1 방향으로: 각 정규화가 없으면 6.2 rad 를 돌아간다."""
    spec = RouteSpec(waypoints=[(-5.0, -0.2)], hold_start_s=0.0, hold_end_s=0.0, dwell_s=0.0)
    f = RouteFollower(spec)
    cmd = f.step(0.0, 0.0, 0.0, 3.1)
    target = math.atan2(-0.2, -5.0)                # ≈ −3.10
    assert cmd.phase == 'rotate'
    assert cmd.w > 0.0                             # +0.08 rad 만 돌면 된다
    log = simulate(spec, (0.0, 0.0, 3.1))
    t_rot = sum(1 for *_, c in log if c.phase == 'rotate') * 0.02
    assert t_rot < 3.0, t_rot
    assert abs(target) > 3.0


def test_heading_wraparound_while_driving_west():
    """서쪽(선분 방향 +π)으로 달리는데 방위가 −π 쪽에 있다: 직진 중 방향 오차도 정규화해야 한다."""
    spec = RouteSpec(waypoints=[(-6.0, 0.0), (-6.0, 1.0)], hold_start_s=0.0, hold_end_s=0.0,
                     dwell_s=0.0)
    log = simulate(spec, (0.0, 0.0, -math.pi + 0.001), t_max=200.0)
    assert log[-1][4].done
    drive_w = [abs(c.w) for *_, c in log if c.phase == 'drive']
    assert max(drive_w[:200]) < 0.2               # π 경계에서 방향 오차가 2π 로 튀지 않는다


def test_realign_when_pushed_off_heading():
    spec = RouteSpec(waypoints=[(5.0, 0.0)], hold_start_s=0.0, hold_end_s=0.0, dwell_s=0.0)
    f = RouteFollower(spec)
    assert f.step(0.0, 0.0, 0.0, 0.0).phase == 'drive'
    cmd = f.step(0.1, 1.0, 0.0, 1.2)               # 방향이 1.2 rad 틀어졌다 → 멈추고 다시 회전
    assert cmd.phase == 'rotate' and cmd.v == 0.0


def test_route_from_dict_rejects_unknown_keys():
    with pytest.raises(ValueError):
        route_from_dict({'waypoints': [[1, 2]], 'hold_star_s': 3})
    with pytest.raises(ValueError):
        route_from_dict({'waypoints': [[1, 2]], 'limits': {'vmax': 1.0}})
    with pytest.raises(ValueError):
        route_from_dict({'waypoints': []})


def test_pilot_route_file_runs_to_completion():
    data = yaml.safe_load(PILOT.read_text(encoding='utf-8'))['route']
    spec = route_from_dict(data)
    start = (data['start'][0], data['start'][1], math.pi / 2)
    log = simulate(spec, start)
    assert log[-1][4].done
    for wp in spec.waypoints:
        assert _reached(log, wp, spec.limits.goal_tol + 0.02)
    length = path_length([data['start']] + [list(w) for w in spec.waypoints])
    assert abs(length - 55.0) < 1e-6              # 머리말의 55 m 와 맞는지
    assert log[-1][0] < 200.0                     # 시뮬레이션 200 s 안 (bag 크기 예산)


def test_limits_default_is_conservative():
    lim = Limits()
    assert lim.v_max <= 0.5 and lim.w_max <= 0.5
