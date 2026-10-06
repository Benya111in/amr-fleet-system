"""창고 변환기·경로 풀 시험 (numpy·scipy 만, Isaac 불필요)."""

import math

import numpy as np
import pytest

from amr_isaac import paths as PA
from amr_isaac import warehouse_geom as W


@pytest.fixture(scope='module')
def geo():
    boxes = W.load_boxes()
    obb = W.obstacles_2d(boxes)
    occ, origin, res = W.rasterize(obb)
    return boxes, obb, occ, origin, res


def test_box_inventory(geo):
    boxes, obb, _occ, _o, _r = geo
    names = [b.name for b in boxes]
    assert len(boxes) == 229
    assert len(obb) == 129
    assert sum(n.startswith('warehouse_walls') for n in names) == 4
    assert not any(n.startswith(('forklift', 'shuttle_amr', 'worker')) for n in names)
    assert all(b.is_planar() for b in boxes)


def test_walls_inner_faces(geo):
    boxes = geo[0]
    walls = [b for b in boxes if b.name.startswith('warehouse_walls')]
    xs = sorted(abs(b.center[0]) - 0.5 * b.size[0] for b in walls if b.size[0] < 1.0)
    ys = sorted(abs(b.center[1]) - 0.5 * b.size[1] for b in walls if b.size[1] < 1.0)
    assert xs == pytest.approx([30.0, 30.0])
    assert ys == pytest.approx([20.0, 20.0])


def test_narrow_passage_width(geo):
    obb = geo[1]
    # (0, −10) 의 좁은 통로: x = ±0.8 의 90° 랙 (깊이 1.0) → 안쪽 면 ±0.3 → 순폭 0.60 m
    near = [r for r in obb if abs(r[0]) < 1.5 and -12.5 < r[1] < -7.5]
    inner = sorted(abs(r[0]) - (r[2] * abs(math.cos(r[4])) + r[3] * abs(math.sin(r[4])))
                   for r in near)
    # SDF yaw 가 1.5708 (π/2 가 아니다) 이라 면이 3.7 µm 안쪽에 있다
    assert inner[0] == pytest.approx(0.30, abs=1e-5)


def test_footprint_collision(geo):
    obb = geo[1]
    hl, hw = 0.30, 0.20
    assert not W.footprint_hits_obb((0.0, 0.0, 0.0), obb, hl, hw)
    assert W.footprint_hits_obb((29.8, 0.0, 0.0), obb, hl, hw)          # 벽 침투
    assert not W.footprint_hits_obb((29.69, 0.0, 0.0), obb, hl, hw)      # 벽 앞 1 cm
    assert not W.footprint_hits_obb((0.0, -10.0, math.pi / 2), obb, hl, hw)   # 좁은 통로 정렬
    assert W.footprint_hits_obb((0.0, -10.0, 0.0), obb, hl, hw)          # 통로에 가로
    # 0.2 rad 기울면 x 폭 0.256 < 0.30 (통과), 0.5 rad 이면 0.320 > 0.30 (충돌)
    assert not W.footprint_hits_obb((0.0, -10.0, math.pi / 2 + 0.2), obb, hl, hw)
    assert W.footprint_hits_obb((0.0, -10.0, math.pi / 2 + 0.5), obb, hl, hw)


def test_triangle_mesh_closed(geo):
    v, t = W.triangle_mesh(geo[0])
    assert v.shape == (8 * 229, 3) and t.shape == (12 * 229, 3)
    # 각 상자 메시는 닫혀 있다: 모든 모서리가 정확히 두 삼각형에 속한다
    e = np.sort(np.concatenate([t[:, [0, 1]], t[:, [1, 2]], t[:, [2, 0]]]), axis=1)
    _u, cnt = np.unique(e, axis=0, return_counts=True)
    assert (cnt == 2).all()


def test_costmap_values():
    d = np.array([0.0, 0.1, 0.2, 0.21, 0.5, 1.2, 1.3])
    c = PA.inflation_cost(d)
    assert list(c[:3]) == [254, 253, 253]
    assert c[3] == int(math.floor(252 * math.exp(-2.0 * 0.01)))
    assert c[-1] == 0


def test_small_pool_properties(geo):
    _b, _obb, occ, origin, res = geo
    pool = PA.build_pool(occ, origin, n_goals=3, starts_per_goal=3, seed=7, log=None)
    cost, dist = PA.costmap(occ)
    assert len(pool['length']) >= 6
    for k in range(len(pool['length'])):
        pts = pool['pts'][pool['offs'][k]:pool['offs'][k + 1]]
        seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        assert seg.max() <= PA.SPACING + 1e-9
        assert PA.L_MIN <= seg.sum() <= PA.L_MAX
        ix = np.floor((pts[:, 0] - origin[0]) / res).astype(int)
        iy = np.floor((pts[:, 1] - origin[1]) / res).astype(int)
        assert cost[iy, ix].max() < 253                       # 모든 점이 통행 가능 셀
        assert dist[iy[0], ix[0]] >= PA.START_CLEARANCE - 1e-9  # 시작 여유
        assert dist[iy[-1], ix[-1]] >= PA.GOAL_CLEARANCE - 1e-9


def test_endpoint_cells_clearance(geo):
    """시작·목표 후보는 여유 문턱을 지킨다 — 창고 격자에는 문턱 아래 통행 가능 셀이 많아 문턱이 실제로 거른다."""
    _b, _obb, occ, _o, _r = geo
    cost, dist = PA.costmap(occ)
    start, goal = PA.endpoint_cells(cost, dist)
    d, c = dist.ravel(), cost.ravel()
    free = c < 253
    assert np.count_nonzero(free & (d < PA.START_CLEARANCE)) > 10000
    assert np.count_nonzero(free & (d < PA.GOAL_CLEARANCE)) > 1000
    assert d[start].min() >= PA.START_CLEARANCE
    assert np.array_equal(start, np.flatnonzero(d >= PA.START_CLEARANCE))
    assert d[goal].min() >= PA.GOAL_CLEARANCE and c[goal].max() < 253
    assert np.array_equal(goal, np.flatnonzero((d >= PA.GOAL_CLEARANCE) & free))
