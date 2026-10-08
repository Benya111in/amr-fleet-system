"""녹화 경로 생성(routegen.py) 시험 — 차선 그래프, 씨드 걸음의 결정성·귀환, 일직선 합침, 장애물 여유 검사."""

import math

from amr_slam3d import routegen as rg
import numpy as np
import pytest


def test_lane_graph_edges_are_axis_aligned_lane_segments():
    adj = rg.lane_graph()
    assert rg.START_NODE in adj
    for a, nbrs in adj.items():
        assert nbrs, a
        for b in nbrs:
            assert a in adj[b]                                  # 무향
            assert a[0] == b[0] or a[1] == b[1]                 # 가로 또는 세로
            if a[1] == b[1]:
                assert a[1] in rg.H_LANES or a == rg.START_NODE or b == rg.START_NODE
    # ±9 세로 차선은 남쪽 홀(y < −6)로 내려가지 않는다 (기둥 (±10, −12) 에 0.75 m)
    assert (9.0, -15.0) not in adj and (-9.0, -15.0) not in adj
    assert (-9.0, -6.0) in adj and (-9.0, 13.3) in adj


def test_random_walk_is_deterministic_and_returns_to_start():
    for seed in (1, 2, 3, 17):
        a, b = rg.random_walk(seed, 60.0), rg.random_walk(seed, 60.0)
        assert a == b
        assert a[0] == a[-1] == rg.START_NODE
        assert rg.path_length(a) >= 60.0
    assert rg.random_walk(1, 60.0) != rg.random_walk(2, 60.0)


def test_random_walk_follows_graph_edges():
    adj = rg.lane_graph()
    w = rg.random_walk(5, 80.0, adj=adj)
    for p, q in zip(w, w[1:]):
        assert q in adj[p]


def test_merge_collinear_keeps_turns_and_u_turns():
    pts = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 1), (0, 1), (0, 1)]
    assert rg.merge_collinear(pts) == [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (2.0, 1.0), (0.0, 1.0)]


def test_shortest_path_prefers_shorter_route():
    adj = rg.lane_graph()
    p = rg.shortest_path(adj, (21.5, 13.3), rg.START_NODE)
    assert p[0] == (21.5, 13.3) and p[-1] == rg.START_NODE
    assert rg.path_length(p) == pytest.approx(13.3 + 15.0 + 3.5)


def _grid(walls, res=0.05, xmin=-2.0, ymin=-2.0, w=80, h=80):
    occ = np.zeros((h, w), dtype=bool)
    for (x0, x1, y0, y1) in walls:
        c0, c1 = int((x0 - xmin) / res), int((x1 - xmin) / res)
        r0, r1 = int((y0 - ymin) / res), int((y1 - ymin) / res)
        occ[r0:r1, c0:c1] = True
    return occ, res, (xmin, ymin)


def test_check_clearance_flags_wall_close_to_segment():
    occ, res, origin = _grid([(-2.0, 2.0, 0.5, 0.6)])          # 벽 y 0.5–0.6
    ok = rg.check_clearance((-1.5, -0.5), [(1.5, -0.5)], occ, res, origin)
    assert ok['ok'] and ok['segments'][0]['min_clearance'] >= 0.9
    bad = rg.check_clearance((-1.5, 0.1), [(1.5, 0.1)], occ, res, origin)
    assert not bad['ok'] and bad['segments'][0]['min_clearance'] < 0.66


def test_check_clearance_turn_circle():
    occ, res, origin = _grid([(0.4, 0.5, -2.0, 2.0)])          # 벽 x 0.4–0.5
    r = rg.check_clearance((-1.5, -1.0), [(0.0, -1.0), (0.0, -1.5)], occ, res, origin,
                           open_r=0.3)
    assert r['segments'][0]['ok'] and r['segments'][1]['ok']
    assert not r['turns'][0]['ok']                              # 회전 원 0.46 > 0.4


def test_check_clearance_uses_aisle_radius_only_inside_aisle_zone():
    """통로 상자(±0.3, y −12 ~ −8, y 로 0.6 넓힘) 안 표본은 aisle_r, 밖은 open_r."""
    occ, res, origin = _grid([(-1.3, -0.3, -12.0, -8.0), (0.3, 1.3, -12.0, -8.0)],
                             xmin=-3.0, ymin=-16.0, w=120, h=240)
    r = rg.check_clearance((0.0, -5.0), [(0.0, -15.0)], occ, res, origin)
    assert r['ok'], r['segments']
    assert r['segments'][0]['need_there'] in (0.25, 0.66)
    # 통로를 x = 0.2 로 지나면 (반폭 0.2 + 0.05 > 여유 0.1) 실패
    r2 = rg.check_clearance((0.2, -5.0), [(0.2, -15.0)], occ, res, origin)
    assert not r2['ok']
    assert bool(rg.in_aisle(0.0, -7.5, rg.AISLE_EXT)) and not bool(rg.in_aisle(0.0, -7.3,
                                                                               rg.AISLE_EXT))
    assert math.isclose(rg.AISLE_EXT, 0.6)
