"""fleetsim 기하·세계: 차체 겹침, 경로, 구역 방문 · 정지선의 성질."""

import math
import random

import numpy as np
import pytest

from amr_fleet.fleetsim.geometry import (
    Box, box_of_polygon, CIRCUMRADIUS, Path, rect_box_overlap, rect_corners, wrap,
)
from amr_fleet.fleetsim.world import DEFAULT_MAP, default_world, STOP_MARGIN_M

pytestmark = pytest.mark.skipif(not DEFAULT_MAP.is_file(), reason='maps/warehouse.yaml 없음')


def _brute_overlap(cx, cy, th, box, n=41):
    """차체 사각형 안의 격자 점 중 하나라도 상자 안(경계 제외)인가 — 느린 기준."""
    c, s = math.cos(th), math.sin(th)
    for a in np.linspace(-0.30, 0.30, n):
        for b in np.linspace(-0.20, 0.20, n):
            x, y = cx + a * c - b * s, cy + a * s + b * c
            if box.xmin < x < box.xmax and box.ymin < y < box.ymax:
                return True
    return False


def test_rect_box_overlap_matches_brute_force():
    rng = random.Random(7)
    box = Box(0.0, 0.0, 2.0, 1.0)
    disagree = 0
    for _ in range(600):
        cx, cy = rng.uniform(-0.6, 2.6), rng.uniform(-0.6, 1.6)
        th = rng.uniform(-math.pi, math.pi)
        fast = rect_box_overlap(cx, cy, th, box)
        slow = _brute_overlap(cx, cy, th, box)
        if fast != slow:
            # 격자 표본은 모서리 스침(< 0.6/40 m)을 놓칠 수 있다 — 그때만 허용
            corners = rect_corners(cx, cy, th)
            gap = min(box.distance(*p) for p in corners)
            assert fast and gap < 0.02, (cx, cy, th)
            disagree += 1
    assert disagree < 15


def test_box_of_polygon_rejects_non_rectangles():
    assert box_of_polygon([(0, 0), (2, 0), (2, 1), (0, 1)]) == Box(0, 0, 2, 1)
    with pytest.raises(ValueError):
        box_of_polygon([(0, 0), (2, 0), (1, 1)])
    with pytest.raises(ValueError):
        box_of_polygon([(0, 0), (2, 0), (2.5, 1), (0, 1)])


def test_path_point_heading_and_turns():
    p = Path([(0, 0), (3, 0), (3, 4)])
    assert p.length == pytest.approx(7.0)
    assert p.point(1.5) == pytest.approx((1.5, 0.0))
    assert p.point(5.0) == pytest.approx((3.0, 2.0))
    assert p.heading(0.0) == pytest.approx(0.0)
    assert p.heading(3.0) == pytest.approx(math.pi / 2)      # 꼭짓점에서는 다음 선분
    assert p.turn_angles() == pytest.approx([math.pi / 2])
    assert wrap(3 * math.pi) == pytest.approx(math.pi)
    single = Path([(1, 1)])
    assert single.length == 0.0 and single.point(3.0) == (1.0, 1.0)


@pytest.fixture(scope='module')
def world():
    return default_world()


def test_world_reads_repository_layout(world):
    assert world.robot_ids == ('amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05')
    assert set(world.docks) == {'dock_1', 'dock_2', 'dock_a', 'dock_b'}
    assert 'dock_1_approach' in world.boxes and 'narrow_aisle' in world.boxes
    assert world.approach['dock_1_approach'] == pytest.approx(1.5)   # yaml 항목 approach_m
    assert world.approach['x_ab_1'] == pytest.approx(2.0)
    assert world.approach['narrow_aisle'] == pytest.approx(2.5)


def _legs(world):
    names = list(world.places)
    return [world.leg(s, d) for s in names for d in world.docks if s != d]


def test_paths_keep_clearance_away_from_endpoints(world):
    """경로 위 점의 여유 ≥ los_clearance (끝점 1 m 안은 도크 박스 때문에 제외)."""
    for leg in _legs(world):
        p = leg.path
        ss = np.arange(1.0, max(1.0, p.length - 1.0), 0.05)
        if len(ss) == 0:
            continue
        xy = np.array([p.point(float(s)) for s in ss])
        assert world.clearance_at(xy).min() >= world.los_clearance_m - 1e-9, leg.start + leg.goal


def test_zone_passes_cover_every_touch_and_stop_lines_are_clear(world):
    """
    외접원이 구역에 닿는 모든 호 길이는 그 구역 방문의 [s_touch, s_clear] 안에 있고, 정지선 앞에서는 닿지 않는다.

    외접원은 어떤 방향의 차체도 덮으므로 정지선에 선 차체(회전 중 포함)는 구역에 겹치지 않는다.
    """
    r = CIRCUMRADIUS
    for leg in _legs(world):
        p = leg.path
        ss = np.arange(0.0, p.length + 1e-9, 0.01)
        for s in ss:
            x, y = p.point(float(s))
            for z, b in world.boxes.items():
                if b.distance(x, y) < r:
                    assert any(q.zone_id == z and q.s_touch - 1e-9 <= s <= q.s_clear + 1e-9
                               for q in leg.passes), (leg.start, leg.goal, z, s)
        for q in leg.passes:
            if q.s_touch <= 0.0:
                continue
            b = world.boxes[q.zone_id]
            for s in np.arange(0.0, q.stop_s + 1e-9, 0.01):
                x, y = p.point(float(s))
                assert b.distance(x, y) >= r, (leg.start, leg.goal, q.zone_id, s)
            assert q.stop_s == pytest.approx(max(0.0, q.s_touch - STOP_MARGIN_M))


def test_request_visits_shift_with_progress(world):
    leg = world.leg('spawn_amr_01', 'dock_1')
    first = leg.passes[0]
    vs = leg.visits_from(first.visit.s_in - 1.0)
    assert vs[0].zone_id == first.zone_id and vs[0].s_in == pytest.approx(1.0)
    assert all(v.s_out >= 0 for v in leg.visits_from(first.s_clear + 0.1))
