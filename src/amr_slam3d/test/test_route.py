"""경로 추종(route.py) 시험 — 이상 유니사이클로 끝까지 돌려 웨이포인트 도달·횡오차·한계를 본다."""

import math
from pathlib import Path

from amr_slam3d.route import Limits, route_from_dict, RouteFollower, RouteSpec, simulate
from amr_slam3d.se2 import path_length, wrap
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


# ---- b2-rec: 녹화 프로토콜 (wait_go·웨이포인트별 정지·후진·출발 시각) ----------------------------------------
ROUTES = Path(__file__).resolve().parents[1] / 'config' / 'routes'


def _load(name):
    data = yaml.safe_load((ROUTES / f'{name}.yaml').read_text(encoding='utf-8'))['route']
    return data, route_from_dict(data)


def test_wait_go_holds_until_armed_then_counts_hold_from_arm():
    spec = RouteSpec(waypoints=[(2.0, 0.0)], hold_start_s=3.0, hold_end_s=0.5, dwell_s=0.0)
    log = simulate(spec, (0.0, 0.0, 0.0), arm_at=7.0)
    first_move = min(t for t, _, _, _, c in log if c.v != 0.0 or c.w != 0.0)
    assert first_move == pytest.approx(10.0, abs=0.05)      # arm 7 s + hold 3 s (적분 시각 2 틱 안)
    assert all(c.phase == 'wait_go' for t, *_, c in log if t < 7.0 - 1e-9)
    assert log[-1][4].done


def test_unarmed_follower_never_moves():
    spec = RouteSpec(waypoints=[(2.0, 0.0)], hold_start_s=0.0, hold_end_s=0.0, dwell_s=0.0)
    f = RouteFollower(spec, armed=False)
    for k in range(500):
        c = f.step(0.02 * k, 0.0, 0.0, 0.0)
        assert c.v == 0.0 and c.w == 0.0 and c.phase == 'wait_go'


def test_depart_at_delays_departure_beyond_hold():
    spec = RouteSpec(waypoints=[(2.0, 0.0)], hold_start_s=1.0, hold_end_s=0.0, dwell_s=0.0,
                     depart_at_s=60.0)
    log = simulate(spec, (0.0, 0.0, 0.0), arm_at=5.0, t_max=200.0)
    first_move = min(t for t, _, _, _, c in log if c.v != 0.0 or c.w != 0.0)
    assert first_move == pytest.approx(60.0, abs=0.03)


def test_per_waypoint_dwell_and_reverse_parse():
    spec = route_from_dict({'waypoints': [[1, 0], {'at': [2, 0], 'dwell': 15},
                                          {'at': [0, 0], 'reverse': True}], 'dwell_s': 1.0})
    assert [spec.dwell_of(i) for i in range(3)] == [1.0, 15.0, 1.0]
    assert [spec.reverse_of(i) for i in range(3)] == [False, False, True]
    with pytest.raises(ValueError):
        route_from_dict({'waypoints': [{'at': [1, 0], 'dwel': 3}]})
    with pytest.raises(ValueError):
        route_from_dict({'waypoints': [{'xy': [1, 0]}]})
    with pytest.raises(ValueError):
        route_from_dict({'waypoints': [{'at': [1, 0], 'dwell': -1}]})


def test_reverse_leg_drives_backwards_without_turning():
    """후진 웨이포인트: 방향을 바꾸지 않고 v < 0 으로 간다 (제자리 회전 없음), 치우침이 있어도 선분 위."""
    spec = RouteSpec(waypoints=[(0.0, -3.0)], hold_start_s=0.0, hold_end_s=0.0, dwell_s=0.0,
                     reverse=[True])
    log = simulate(spec, (0.0, 0.0, math.pi / 2), slip=0.03, w_bias=0.02)
    assert log[-1][4].done
    assert _reached(log, (0.0, -3.0), spec.limits.goal_tol + 0.02)
    drive = [c for *_, c in log if c.phase == 'drive']
    assert drive and all(c.v <= 0.0 for c in drive) and min(c.v for c in drive) < -0.4
    assert max(abs(wrap(yw - math.pi / 2)) for _, _, _, yw, _ in log) < 0.05
    assert max(abs(x) for _, x, _, _, _ in log) < 0.05


def test_s6_inside_route_never_rotates_in_aisle_and_stays_centred():
    """
    S6 통로 안 경로: 회전 없이 중심선을 지킨다.

    순폭 0.60 m (반폭 0.30) 안에서 외접 반경 0.36 m 로봇이 돌면 랙에 닿는다 — 방위 변화 < 0.05 rad,
    중심선 횡오차 < 0.05 m (반폭 0.20 + 0.05 ≤ 0.30), 바퀴 치우침·미끄러짐이 있어도.
    """
    data, spec = _load('s6_inside')
    start = (data['start'][0], data['start'][1], data['start_yaw'])
    log = simulate(spec, start, slip=0.03, w_bias=0.02, arm_at=30.0, t_max=400.0)
    assert log[-1][4].done
    for wp in spec.waypoints:
        assert _reached(log, wp, spec.limits.goal_tol + 0.02)
    assert max(abs(wrap(yw - start[2])) for _, _, _, yw, _ in log) < 0.05
    assert max(abs(x) for _, x, _, _, _ in log) < 0.05
    ys = [y for _, _, y, _, _ in log]
    assert min(ys) > -11.4 - 0.06 and max(ys) < -8.5
    dep = min(t for t, _, _, _, c in log if c.v != 0.0 or c.w != 0.0)
    assert dep == pytest.approx(60.0, abs=0.03)              # depart_at_s (변형 짝)
    stops = [t for t, x, y, _, c in log if c.phase == 'dwell' and abs(y + 10.0) < 0.06]
    assert max(stops) - min(stops) >= 15.0 - 0.05            # 중앙 15 s 정지


@pytest.mark.parametrize('name', ['s1_seed1', 's1_seed2', 's1_seed3', 'map_long', 's3_aisle'])
def test_committed_routes_run_to_completion(name):
    data, spec = _load(name)
    start = (data['start'][0], data['start'][1], data['start_yaw'])
    log = simulate(spec, start, arm_at=12.0, t_max=4000.0)
    assert log[-1][4].done, name
    for wp in spec.waypoints:
        assert _reached(log, wp, spec.limits.goal_tol + 0.02), (name, wp)
    assert spec.hold_start_s >= 10.0 and spec.hold_end_s >= 0.5     # 정지 구간 규칙 (recgate)


def test_s3_route_stops_15s_in_aisle_centre_and_crosses_aisle_on_axis():
    data, spec = _load('s3_aisle')
    log = simulate(spec, (data['start'][0], data['start'][1], data['start_yaw']),
                   slip=0.03, w_bias=0.02, arm_at=12.0, t_max=1000.0)
    inside = [(x, y) for _, x, y, _, c in log if -12.0 <= y <= -8.0 and abs(x) < 2.0]
    assert inside and max(abs(x) for x, _ in inside) < 0.05
    stops = [t for t, x, y, _, c in log if c.phase == 'dwell' and abs(y + 10.0) < 0.06]
    assert max(stops) - min(stops) >= 15.0 - 0.05
    assert min(y for _, _, y, _, _ in log) < -14.4


def test_committed_route_files_match_generator(tmp_path):
    """config/routes·config/spawn 은 make_routes.py write 출력 그대로 (손으로 고치면 실패)."""
    import importlib.util
    script = Path(__file__).resolve().parents[1] / 'scripts' / 'make_routes.py'
    spec = importlib.util.spec_from_file_location('make_routes', script)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.main(['write', '--dir', str(tmp_path)]) == 0
    cfg = Path(__file__).resolve().parents[1] / 'config'
    for sub in ('routes', 'spawn'):
        made = sorted(p.name for p in (tmp_path / sub).iterdir())
        assert made == sorted(p.name for p in (cfg / sub).iterdir()), sub
        for n in made:
            assert (tmp_path / sub / n).read_text(encoding='utf-8') == \
                (cfg / sub / n).read_text(encoding='utf-8'), n
