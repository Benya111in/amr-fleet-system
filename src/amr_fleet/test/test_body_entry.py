"""
구역 진입 몸체 판정 (b4-p5b, p5.md §0-0 · §5-1) — 경로 게이트 · 중앙 교통 관리자 공통.

P5 Gazebo 의 실제 자세(원판 r 0.36 m) 기준 구역 동시 점유 15 건은 진입을 경로 **중심점**으로 잰 데서 나왔다:
요청 없는 경계 스침 13 건 (경로 중심이 구역에 들지 않아 zone_visits 가 그 구역을 고르지 않음)과, 요청했으나 막힌
자리(결정점)에서 몸체가 이미 구역 안이던 2 건 (막힘 거리 = 중심 진입까지 — 결정점 정지 −0.107 · −0.228 m).
여기서는 변을 따라가는 경로 · 모서리를 비스듬히 지나는 경로 · 결정점 정지 침범을 그 사건 기하로 재현하고, 몸체 판정이
셋을 모두 막는 것과 옛 판정(body_entry=False)이 그 결함을 그대로 가진 것을 함께 확인한다.
"""

import math
import pathlib

import numpy as np
import pytest
import yaml

from amr_fleet.agent.path_gate import load_path_gate, PathGate, PathGateConfig
from amr_fleet.traffic_geometry import discs_touch_polygon
from amr_fleet.traffic_manager import (
    RobotObservation, ST_LOADING, ST_MOVING, TrafficConfig, TrafficManager,
)
from amr_fleet.traffic_zones import (
    BODY_SCAN_STEP_M, decision_distance, load_layout, polygon_distance, request_chain, zone_visits,
    ZoneMap, zones_from_config, ZoneVisit,
)

CONFIG_DIR = pathlib.Path(__file__).resolve().parents[1] / 'config'
R = 0.36
# 배포 배치의 동쪽 도크 구역 (traffic_zones.yaml) — P5 사건 15 건 중 14 건이 이 둘
DOCK_B = [[26.8, 12.25], [29.8, 12.25], [29.8, 13.75], [26.8, 13.75]]
DOCK_A = [[26.8, 16.25], [29.8, 16.25], [29.8, 17.75], [26.8, 17.75]]
OVERSHOOT_M = 0.81      # P5 두 정지 사건: 막힌 뒤 1.52 s · 0.80–0.81 m 더 가서 섰다 [측정, p5.md §5-1]


def _zones(**polys):
    return zones_from_config([dict(id=k, kind='intersection', polygon=v)
                              for k, v in polys.items()])


def _gate(body, **polys):
    zs = _zones(**polys)
    return PathGate(zs, {z.zone_id: 2.0 for z in zs},
                    PathGateConfig(prerequest_m=1.0, body_entry=body))


def _along(path, step=0.05):
    """경로 위 점들 (step 간격) — 로봇이 그 경로를 따라갈 때의 자세 열."""
    path = np.asarray(path, dtype=float)
    out = []
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        for k in range(n):
            p = a + (b - a) * k / n
            out.append((float(p[0]), float(p[1]), yaw))
    return out


def _remaining(path, pose):
    """자세 이후의 남은 경로 (자세를 첫 점으로)."""
    path = np.asarray(path, dtype=float)
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    best, bk = None, 0
    for k in range(len(path) - 1):
        a, b = path[k], path[k + 1]
        t = np.clip(np.dot(np.array(pose[:2]) - a, b - a) / max(np.dot(b - a, b - a), 1e-12), 0, 1)
        d = np.linalg.norm(a + (b - a) * t - np.array(pose[:2]))
        if best is None or d < best:
            best, bk = d, k
    return np.vstack([[pose[0], pose[1]], path[bk + 1:]]), cum


def _first_block(gate, path, zone='zone:z'):
    """경로를 따라가며 그 구역이 처음 blocking 에 드는 자세 (없으면 None)."""
    for pose in _along(path):
        d = gate.decide(pose, _remaining(path, pose)[0], set())
        if zone in d.blocking:
            return pose
    return None


def _advance(path, pose, dist):
    """자세에서 경로를 따라 dist 더 간 자세."""
    pts = _along(path, 0.01)
    k = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - pose[0], pts[i][1] - pose[1]))
    return pts[min(len(pts) - 1, k + int(round(dist / 0.01)))]


def _disc_depth(poly, x, y, r=R):
    """원판이 다각형에 들어간 깊이 [m] (양수 = 침범, P5 분석의 −clear_m)."""
    p = np.asarray(poly, dtype=float)
    if discs_touch_polygon(np.array([[x, y]]), p, 0.0)[0]:
        return r
    d = min(_seg(x, y, p[i], p[(i + 1) % len(p)]) for i in range(len(p)))
    return r - d


def _seg(x, y, a, b):
    ab = b - a
    t = np.clip(np.dot(np.array([x, y]) - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(a + ab * t - np.array([x, y])))


# ---------------------------------------------------------------------- 기하
def test_discs_touch_polygon_edges_corners_and_exact_contact():
    sq = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]])
    pts = np.array([[1.0, 0.5],                     # 안
                    [1.0, -0.3],                    # 변에서 0.3 < 0.36
                    [-0.2, -0.2],                   # 모서리에서 0.283 < 0.36 (대각선)
                    [-0.3, -0.3],                   # 모서리에서 0.424 > 0.36
                    [1.0, -0.36],                   # 정확히 0.36 — 닿기만 함 (P5 분석과 같은 정의: 겹침 아님)
                    [5.0, 5.0]])
    assert discs_touch_polygon(pts, sq, R).tolist() == [True, True, True, False, False, False]
    assert discs_touch_polygon(pts, sq, 0.0).tolist() == [True] + [False] * 5


def test_zone_map_touching_reports_every_zone_the_disc_overlaps():
    zm = ZoneMap(_zones(a=[[0, 0], [1, 0], [1, 1], [0, 1]],
                        b=[[1.5, 0], [2.5, 0], [2.5, 1], [1.5, 1]]))
    assert zm.zones_touching(1.25, 0.5, R) == frozenset({'a', 'b'})     # 사이 0.5 m 틈의 가운데
    assert zm.zones_touching(1.25, 0.5, 0.2) == frozenset()
    assert zm.zone_at(1.25, 0.5) is None


# ---------------------------------------------------------------------- 방문 (zone_visits)
def test_edge_following_path_visits_the_zone_only_with_body_radius():
    """중심이 구역 변 밖 0.15 m 를 따라가는 경로: 중심 판정은 방문 0, 몸체 판정은 변 길이만큼 방문."""
    zm = ZoneMap(_zones(z=[[4.0, 0.0], [8.0, 0.0], [8.0, 2.0], [4.0, 2.0]]))
    path = np.array([[0.0, -0.15], [12.0, -0.15]])
    assert zone_visits(path, zm) == []
    v = zone_visits(path, zm, radius=R)
    assert [x.zone_id for x in v] == ['z']
    reach = math.sqrt(R * R - 0.15 * 0.15)                  # 원판이 모서리에 처음 닿는 곳 = 4 − reach
    assert 4.0 - reach - BODY_SCAN_STEP_M - 1e-9 <= v[0].s_in <= 4.0 - reach + 1e-9   # 보수적 (이르게)
    assert 8.0 + reach - 1e-9 <= v[0].s_out <= 8.0 + reach + BODY_SCAN_STEP_M + 1e-9  # 보수적 (늦게)


def test_diagonal_corner_graze_is_a_visit():
    """구역 모서리를 대각선으로 0.25 m 떨어져 지나는 경로 (중심은 끝까지 밖) — P5 dock_a 남서 모서리 사건의 기하."""
    zm = ZoneMap(_zones(z=DOCK_A))
    c, d = np.array([26.8, 16.25]), np.array([-1.0, 1.0]) / math.sqrt(2.0)
    p0 = c + 0.25 * np.array([-1.0, -1.0]) / math.sqrt(2.0)
    path = np.array([p0 - 3.0 * d, p0 + 3.0 * d])
    line = np.linspace(path[0], path[1], 2000)
    assert not discs_touch_polygon(line, np.array(DOCK_A), 0.0).any()
    assert zone_visits(path, zm) == []
    v = zone_visits(path, zm, radius=R)
    assert [x.zone_id for x in v] == ['z'] and v[0].s_in < 3.0 < v[0].s_out
    # 얕은 스침(깊이 0.005 m)도 놓치지 않는다 — 훑기 간격 0.05 m 의 놓침 깊이 상한 ≈ 0.001 m
    p1 = c + (R - 0.005) * np.array([-1.0, -1.0]) / math.sqrt(2.0)
    assert [x.zone_id for x in zone_visits(np.array([p1 - 3.0 * d, p1 + 3.0 * d]), zm,
                                           radius=R)] == ['z']


def test_body_visits_have_no_false_negatives_on_random_paths():
    """임의 경로 · 구역: 원판이 겹치는 경로 위 모든 점(0.01 m 간격)이 그 구역의 어느 방문 구간 안에 든다."""
    rng = np.random.default_rng(7)
    for _ in range(40):
        x0, y0 = rng.uniform(-3, 3, 2)
        poly = np.array([[x0, y0], [x0 + rng.uniform(0.5, 3), y0],
                         [x0 + rng.uniform(0.5, 3), y0 + rng.uniform(0.5, 3)],
                         [x0, y0 + rng.uniform(0.5, 3)]])
        zm = ZoneMap(zones_from_config([dict(id='z', kind='intersection',
                                             polygon=poly.tolist())]))
        path = rng.uniform(-6, 6, (4, 2))
        v = zone_visits(path, zm, radius=R)
        seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
        cum = np.concatenate(([0.0], np.cumsum(seg)))
        s = np.arange(0.0, cum[-1], 0.01)
        xy = np.stack([np.interp(s, cum, path[:, 0]), np.interp(s, cum, path[:, 1])], axis=1)
        hit = discs_touch_polygon(xy, poly, R)
        for si in s[hit]:
            assert any(x.s_in - 1e-9 <= si <= x.s_out + 1e-9 for x in v), (si, v)


def test_center_visits_unchanged_when_radius_is_zero():
    zm = ZoneMap(_zones(a=[[2, -1], [4, -1], [4, 1], [2, 1]],
                        b=[[5, -1], [7, -1], [7, 1], [5, 1]]))
    path = np.array([[0.0, 0.0], [10.0, 0.0]])
    v0 = zone_visits(path, zm)
    assert v0 == zone_visits(path, zm, radius=0.0)
    assert [x.zone_id for x in v0] == ['a', 'b'] and v0[0].s_in >= 2.0 and v0[1].s_in >= 5.0
    v1 = zone_visits(path, zm, radius=R)                    # 몸체: 원판이 닿는 곳부터 (0.36 m 이르게)
    assert [x.zone_id for x in v1] == ['a', 'b'] and v1[0].s_in <= 2.0 - R + 1e-9


# ---------------------------------------------------------------------- 사슬 (request_chain)
def test_chain_gap_is_measured_from_the_farthest_exit_of_overlapping_visits():
    v = [ZoneVisit('a', 1.0, 10.0, (0, 0), (0, 0)), ZoneVisit('b', 2.0, 3.0, (0, 0), (0, 0)),
         ZoneVisit('c', 11.0, 12.0, (0, 0), (0, 0))]
    got = request_chain(v, 2.0, 1.5, held=())
    assert [x.zone_id for x in got] == ['a', 'b', 'c']        # c 는 a 출구(10)에서 1 m — 묶음


def test_held_none_keeps_old_semantics_and_held_requests_unheld_zone_already_touched():
    v = [ZoneVisit('k', 0.0, 1.0, (0, 0), (0, 0)), ZoneVisit('m', 5.0, 6.0, (0, 0), (0, 0))]
    assert request_chain(v, 2.0, 1.5) == []                   # 옛 동작: 걸친 구역은 이미 안 — 요청 없음
    assert [x.zone_id for x in request_chain(v, 2.0, 1.5, held=())] == ['k']
    assert request_chain(v, 2.0, 1.5, held={'k'}) == []       # 쥐고 있으면 그대로


# ---------------------------------------------------------------------- 경로 게이트 — P5 사건 기하
def test_edge_following_graze_is_requested_and_blocked_before_the_body_touches():
    """
    요청 없음(스침) 13 건의 기하: 도크 접근 구역 서쪽 변(x 26.8) 0.2 m 밖을 따라 북쪽으로 지나는 경로.

    옛 판정은 그 구역을 한 번도 요청하지 않고 몸체가 0.16 m 걸친다. 몸체 판정은 결정점에서 막고, 막힌 뒤 P5 에서 잰
    0.81 m 를 더 가도 몸체가 구역 밖이다.
    """
    path = [[26.6, 6.0], [26.6, 20.0]]
    old, new = _gate(False, z=DOCK_B), _gate(True, z=DOCK_B)
    touched = [p for p in _along(path) if _disc_depth(DOCK_B, p[0], p[1]) > 0]
    assert touched                                                     # 몸체가 실제로 스친다
    for pose in _along(path):
        assert old.decide(pose, _remaining(path, pose)[0], set()).request == ()
    stop = _first_block(new, path)
    assert stop is not None
    after = _advance(path, stop, OVERSHOOT_M)
    assert _disc_depth(DOCK_B, after[0], after[1]) < -1.0              # 1 m 넘게 남는다
    # 허가가 있으면 막지 않고, 지나는 동안 구역은 '앞' 이라 해제 후보가 아니다
    held = new.decide(touched[0], _remaining(path, touched[0])[0], {'zone:z'})
    assert held.blocking == () and 'zone:z' in held.ahead and 'zone:z' in held.body


def test_decision_point_stop_intrusion_case_from_p5():
    """
    요청 · 막힘 · 구역 안 대기 (P5 `p5_C2_p7_distributedr2`, 60.6 s) — 결정점 정지 침범.

    경로가 dock_b 서쪽 변 바로 밖(x 26.79)을 따라 북쪽으로 가다가 y 13.2 에서 동쪽 도크로 꺾는다. 옛 판정은 중심
    진입(꺾는 곳)까지 2.0 m 에서 막아 0.81 m 더 간 자리에서 몸체가 남쪽 변을 넘는다 (P5: −0.107 m). 몸체 판정은
    원판이 처음 닿는 곳(y ≈ 11.89)까지 2.0 m 에서 막는다.
    """
    path = [[26.79, 6.0], [26.79, 13.2], [27.78, 13.2]]
    old, new = _gate(False, z=DOCK_B), _gate(True, z=DOCK_B)
    s_old = _first_block(old, path)
    s_new = _first_block(new, path)
    assert s_old is not None and s_new is not None
    assert s_old[1] == pytest.approx(11.2, abs=0.1)                     # P5 막힌 자리 (26.8, 11.19)
    stop_old = _advance(path, s_old, OVERSHOOT_M)
    assert _disc_depth(DOCK_B, stop_old[0], stop_old[1]) > 0.05         # 옛 판정: 침범 (결함 재현)
    entry_y = 12.25 - math.sqrt(R * R - 0.01 * 0.01)
    assert s_new[1] <= entry_y - 2.0 + 1e-6                             # 원판이 닿기 2.0 m 전에 막는다
    stop_new = _advance(path, s_new, OVERSHOOT_M)
    assert _disc_depth(DOCK_B, stop_new[0], stop_new[1]) < -1.0


def test_corner_stop_intrusion_case_from_p5():
    """
    요청 · 막힘 · 구역 안 대기 (P5 `p5_P_p1_dist_p_b`, 5.9 s): dock_a 남서 모서리 옆(x 26.67)을 지나 북쪽으로 가다 동쪽 도크로.

    옛 판정은 중심 진입(꺾는 곳 x 26.8)까지 2.0 m 에서 막아 0.81 m 더 간 (26.67, 16.24) 에서 몸체가 모서리를 0.23 m
    넘는다 (P5: (26.67, 16.23), −0.228 m).
    """
    path = [[26.67, 12.0], [26.67, 17.3], [27.78, 17.3]]
    old, new = _gate(False, z=DOCK_A), _gate(True, z=DOCK_A)
    stop_old = _advance(path, _first_block(old, path), OVERSHOOT_M)
    assert stop_old[1] == pytest.approx(16.24, abs=0.1)
    assert _disc_depth(DOCK_A, stop_old[0], stop_old[1]) == pytest.approx(0.228, abs=0.05)
    stop_new = _advance(path, _first_block(new, path), OVERSHOOT_M)
    assert _disc_depth(DOCK_A, stop_new[0], stop_new[1]) < -1.0


def test_unheld_zone_already_under_the_body_blocks_at_distance_zero():
    """허가 없이 원판이 이미 걸친 구역: 몸체 판정은 거리 0 으로 요청하고 막는다 (fail-closed). 옛 판정은 그냥 간다."""
    path = [[26.7, 11.95], [26.7, 20.0]]
    pose = (26.7, 11.95, math.pi / 2)                                   # 원판이 dock_b 에 0.06 m 걸침
    assert _disc_depth(DOCK_B, pose[0], pose[1]) > 0
    d = _gate(True, z=DOCK_B).decide(pose, np.array(path), set())
    assert d.blocking == ('zone:z',) and d.first_entry_m == pytest.approx(0.0)
    assert _gate(True, z=DOCK_B).decide(pose, np.array(path), {'zone:z'}).blocking == ()
    # 옛 판정: 중심은 끝까지 밖이라 요청 · 막힘이 없다
    assert _gate(False, z=DOCK_B).decide(pose, np.array(path), set()).blocking == ()


def test_body_zones_use_the_disc_not_four_points():
    """모서리 대각선 0.3 m: 네 점(앞뒤좌우 0.36 m)은 모두 밖이지만 원판은 겹친다."""
    c = np.array([26.8, 16.25])
    x, y = c + 0.3 * np.array([-1.0, -1.0]) / math.sqrt(2.0)
    assert _gate(False, z=DOCK_A).body_zones(x, y, 0.0) == frozenset()
    assert _gate(True, z=DOCK_A).body_zones(x, y, 0.0) == frozenset({'z'})


def test_shipped_config_turns_body_entry_on_for_both_modes():
    params = yaml.safe_load((CONFIG_DIR / 'traffic.yaml').read_text())
    z = params['/fleet/traffic_manager_node']['ros__parameters']['zones']
    assert z['body_entry'] is True
    assert TrafficConfig().zones.body_entry is True
    gate = load_path_gate(str(CONFIG_DIR / 'traffic_zones.yaml'), str(CONFIG_DIR / 'traffic.yaml'))
    assert gate.cfg.body_entry is True and gate.cfg.robot_radius == pytest.approx(R)


def test_shipped_layout_leaves_the_body_decision_distance_between_unchained_zones():
    """
    배포 배치 + 몸체 판정: 묶이지 않은 이웃 구역 사이 몸체 간격(g − 2r) ≥ 몸체 기준 결정점 거리.

    몸체 판정에서는 앞 구역을 몸체가 나온 뒤(occupied 규칙) 다음 구역 원판 진입까지 g − 2r 이 남는다. 결정점 거리 식
    (p0.md §5)은 중심 거리라 차체 앞 절반 0.30 을 더했는데, 원판(0.36 ≥ 0.30)이 닿는 곳까지 재면 그 항이 빠진다:
    1.85 − 0.30 = 1.55 m. spacing_conflicts(중심 기준)가 통과시킨 배포 배치는 이 조건도 만족한다 (도크 같은 쪽 쌍 1.78 m).
    """
    params = yaml.safe_load((CONFIG_DIR / 'traffic.yaml').read_text())
    p = params['/fleet/traffic_manager_node']['ros__parameters']
    zones, _ = load_layout(str(CONFIG_DIR / 'traffic_zones.yaml'), True)
    d_body = decision_distance(1.0, 0.5, 0.1, 0.2, 1.0, 0.05, 0.2, 0.30) - 0.30
    assert d_body == pytest.approx(1.55)
    gaps = []
    for i, a in enumerate(zones):
        for b in zones[i + 1:]:
            g = polygon_distance(a.polygon, b.polygon)
            if g - 2 * p['robot_radius'] > p['zones']['chain_gap_m']:      # 몸체 판정에서 묶이지 않는 쌍
                gaps.append((g - 2 * p['robot_radius'], a.zone_id, b.zone_id))
    assert min(gaps)[0] >= d_body
    dock = [x for x in gaps if {x[1], x[2]} == {'dock_a_approach', 'dock_b_approach'}]
    assert dock and dock[0][0] == pytest.approx(1.78)


# ---------------------------------------------------------------------- 중앙 교통 관리자 (같은 함수)
_DK = [[1.0, 6.0], [3.0, 6.0], [3.0, 7.5], [1.0, 7.5]]


def _tm(body):
    cfg = TrafficConfig()
    cfg.zones.body_entry = body
    cfg.prediction.crossing_hold = False
    return TrafficManager(cfg, zones_from_config([dict(id='dk', kind='intersection',
                                                       polygon=_DK)], True), [])


def _obs(rid, x, y, yaw, plan, status, t):
    return RobotObservation(rid, x, y, yaw, 0.0 if plan is None else 1.0,
                            None if plan is None else np.array(plan), status, 't', 100, None, t)


@pytest.mark.parametrize('body', [True, False])
def test_central_manager_holds_an_edge_follower_before_the_body_touches(body):
    """
    중앙 모드에도 같은 판정이 들어간다: H 가 dk 안에서 적재 중, G 가 dk 서쪽 변 0.15 m 밖을 북쪽으로 지난다.

    몸체 판정이면 G 는 원판 진입(y ≈ 5.67) 2.0 m 앞에서 zone:dk 로 선다. 옛 판정이면 요청이 없어 서지 않는다 (결함).
    """
    tm = _tm(body)
    holds = []
    for k, gy in enumerate(np.arange(2.0, 6.0, 0.1)):
        t = 0.5 * k
        res = tm.update(t, {'H': _obs('H', 2.0, 6.75, 0.0, None, ST_LOADING, t),
                            'G': _obs('G', 0.85, float(gy), math.pi / 2, [(0.85, gy), (0.85, 9.5)],
                                      ST_MOVING, t)})
        c = res.commands.get('G')
        if c is not None and c.hold and c.reason.startswith('zone:'):
            holds.append(round(float(gy), 2))
    assert tm.tokens.holders('dk') == ['H']
    if body:
        assert holds and holds[0] <= 6.0 - math.sqrt(R * R - 0.15 ** 2) - 2.0 + 0.05
    else:
        assert holds == []


def test_central_manager_holds_a_robot_already_touching_a_held_zone():
    """중앙 모드: 원판이 이미 dk 에 걸친 G (토큰 없음, H 가 보유) — 몸체 판정은 거리 0 요청으로 세운다."""
    tm = _tm(True)
    far = _obs('G', 0.8, 2.0, math.pi / 2, None, ST_LOADING, 0.0)       # 처음에는 멀리 (H 가 먼저 보유)
    tm.update(0.0, {'H': _obs('H', 2.0, 6.75, 0.0, None, ST_LOADING, 0.0), 'G': far})
    assert tm.tokens.holders('dk') == ['H']
    res = None
    for k in range(1, 4):
        t = 0.5 * k
        res = tm.update(t, {'H': _obs('H', 2.0, 6.75, 0.0, None, ST_LOADING, t),
                            'G': _obs('G', 0.8, 6.4, math.pi / 2, [(0.8, 6.4), (0.8, 9.5)],
                                      ST_MOVING, t)})
    assert res.commands['G'].hold and res.commands['G'].reason == 'zone:dk'
    assert tm.tokens.holders('dk') == ['H']


# ---------------------------------------------------------------------- 코어 + 투표 + 운동학 (loopback)
def _loopback(body):
    from amr_fleet.agent.core import CoreConfig, GATE_ZONE_MAP
    from amr_fleet.agent.loopback import LoopbackFleet
    from amr_fleet.agent.modes import ModeConfig
    from amr_fleet.distributed.vote_core import VoteConfig
    robots = ('amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05')
    vote = VoteConfig(robots, tentative_s=10.0, retry_s=0.5, backoff_min_s=0.3, backoff_max_s=1.5)
    cfgs = [CoreConfig(robot_id=r, members=robots, zone_vote=vote, task_vote=vote,
                       release_gate=GATE_ZONE_MAP,
                       modes=ModeConfig(resync_s=3.0, isolate_s=1.0, boot_grace_s=2.0))
            for r in robots]
    poses = {'amr_03': (20.0, 0.0), 'amr_04': (22.0, 0.0), 'amr_05': (24.0, 0.0)}
    return LoopbackFleet(cfgs, poses=poses, path_gate=_gate(body, z=DOCK_B))


@pytest.mark.parametrize('body', [True, False])
def test_vote_gate_keeps_an_edge_follower_out_of_a_held_dock_zone(body):
    """
    실제 투표 · 경로 게이트 (loopback 5 대) — 보유된 도크 구역 변을 따라가는 로봇.

    amr_02 가 dock_b 안 staging 에서 적재 중 (구역 보유), amr_01 이 서쪽 변 0.2 m 밖을 북쪽으로 지난다 (P5 '요청 없음'
    사건과 같은 기하). 감속 1 m/s² · 반응 0.3 s 로만 선다. 몸체 판정: 원판 동시 점유 0, amr_02 가 떠난 뒤 amr_01 도
    지나간다. 옛 판정: 요청 없이 스쳐 동시 점유가 난다 (결함 재현).
    """
    f = _loopback(body)
    f.run(2.0)
    holder = f.cores['amr_02']
    holder.set_pose(27.78, 10.0, math.pi / 2)                  # dock_b 로 들어가 보유를 얻는다
    holder.set_plan(np.array([[27.78, 10.0], [27.78, 13.0]]))
    holder.set_executor_phase('MOVING')
    f.run(2.0)
    holder.set_pose(27.78, 13.0, 0.0)
    holder.set_executor_phase('LOADING')                       # 움직이지 않는 단계 — 몸체가 안이라 계속 쥔다
    f.run(1.0)
    assert f.holders('zone:z') == ['amr_02']
    path = np.array([[26.6, 6.0], [26.6, 20.0]])
    seg = path[1] - path[0]
    length = float(np.linalg.norm(seg))
    s, v, dt, lag = 0.0, 0.0, 0.1, []
    overlap, min_clear = [], math.inf
    core = f.cores['amr_01']
    for k in range(int(60.0 / dt)):
        t = k * dt
        if t >= 25.0 and holder.executor_phase == 'LOADING':      # 적재 끝 — 동쪽으로 떠난다
            holder.set_executor_phase('IDLE')
            holder.set_plan(None)
            holder.set_pose(32.0, 13.0, 0.0)
        x, y = (path[0] + seg * s / length).tolist()
        core.set_pose(x, y, math.pi / 2)
        core.set_plan(None if s >= length else np.vstack([[x, y], path[1:]]))
        core.set_executor_phase('IDLE' if s >= length else 'MOVING')
        f.step(dt)
        st = core.state()
        lag.append(any(z.startswith('zone:') and z not in st.held for z in st.requested))
        blocked = lag[-4] if len(lag) >= 4 else False          # 반응 0.3 s
        v = max(0.0, v - 1.0 * dt) if blocked else min(1.0, v + 1.0 * dt)
        s = min(length, s + v * dt)
        depth = _disc_depth(DOCK_B, x, y)
        min_clear = min(min_clear, -depth)
        if depth > 0 and holder.executor_phase == 'LOADING':
            overlap.append(round(t, 1))
    if body:
        assert overlap == [], overlap
        assert s >= length                                     # 결국 지나간다
        assert any(e['event'] == 'zone_request' and e.get('robot') == 'amr_01' for e in f.events)
    else:
        assert overlap                                         # 옛 판정: 요청 없이 몸체가 스친다


@pytest.mark.parametrize('body', [True, False])
def test_central_body_occupancy_uses_the_disc(body):
    """중앙 몸체 점유: dk 남서 모서리 대각선 0.3 m 에 선 로봇 — 원판은 걸치고 네 점은 밖. 몸체 판정이면 점유로 등록한다."""
    tm = _tm(body)
    c = np.array(_DK[0], dtype=float)
    x, y = c + 0.3 * np.array([-1.0, -1.0]) / math.sqrt(2.0)
    tm.update(0.0, {'X': _obs('X', float(x), float(y), 0.0, None, ST_LOADING, 0.0)})
    assert tm.views['X'].zones == (frozenset({'dk'}) if body else frozenset())
    assert tm.tokens.holders('dk') == (['X'] if body else [])
