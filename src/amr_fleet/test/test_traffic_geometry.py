"""traffic_geometry: 경로 호 길이 · 투영 · 표본 · 다각형 · 격자."""

import math

import numpy as np
import pytest

from amr_fleet.traffic_geometry import (
    GridSpec, angle_diff, as_path, cumulative_length, disc_mask, distance_to_path, points_at,
    points_in_polygon, polygon_area, project, sub_path, wrap_angle,
)

L_PATH = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 3.0]])


def test_angles():
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(-0.5) == pytest.approx(-0.5)
    assert angle_diff(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(0.2)
    assert angle_diff(0.0, math.pi) == pytest.approx(math.pi)


def test_as_path_drops_duplicates_and_empty():
    assert as_path(None) is None
    assert as_path([]) is None
    p = as_path([(0, 0), (0, 0), (1, 0), (1, 0), (1, 1)])
    assert p.tolist() == [[0, 0], [1, 0], [1, 1]]
    assert as_path(np.array([[2.0, 3.0]])).shape == (1, 2)
    assert as_path(iter([(0, 0), (1, 0)])).shape == (2, 2)


def test_cumulative_project_points_subpath():
    cum = cumulative_length(L_PATH)
    assert cum.tolist() == [0.0, 4.0, 7.0]
    assert cumulative_length(L_PATH[:1]).tolist() == [0.0]
    s, d = project(L_PATH, cum, 2.0, 0.5)
    assert s == pytest.approx(2.0) and d == pytest.approx(0.5)
    s, d = project(L_PATH, cum, 5.0, 2.0)
    assert s == pytest.approx(6.0) and d == pytest.approx(1.0)
    assert project(L_PATH[:1], cum[:1], 3.0, 4.0) == (0.0, pytest.approx(5.0))
    xy, hd = points_at(L_PATH, cum, np.array([-1.0, 2.0, 5.5, 99.0]))
    assert xy.tolist() == [[0, 0], [2, 0], [4, 1.5], [4, 3]]
    assert hd[1] == pytest.approx(0.0) and hd[2] == pytest.approx(math.pi / 2)
    xy1, hd1 = points_at(L_PATH[:1], cum[:1], np.array([0.0, 1.0]))
    assert xy1.tolist() == [[0, 0], [0, 0]] and hd1.tolist() == [0.0, 0.0]
    sp = sub_path(L_PATH, cum, 3.0, 5.0, 0.5)
    assert sp[0].tolist() == [3.0, 0.0] and sp[-1].tolist() == [4.0, 1.0] and len(sp) == 5
    assert len(sub_path(L_PATH, cum, 9.0, 1.0, 0.5)) == 2       # 뒤집힌 구간은 한 점으로 모인다


def test_distance_to_path():
    d = distance_to_path(np.array([[2.0, 1.0], [5.0, 3.0], [-3.0, -4.0]]), L_PATH)
    assert d.tolist() == pytest.approx([1.0, 1.0, 5.0])
    assert distance_to_path(np.array([3.0, 4.0]), L_PATH[:1]).tolist() == [5.0]


def test_polygon_helpers():
    sq = np.array([[0, 0], [2, 0], [2, 2], [0, 2]], dtype=float)
    assert polygon_area(sq) == pytest.approx(4.0)
    inside = points_in_polygon(np.array([[1, 1], [3, 1], [-0.1, 1], [1, 1.9]]), sq)
    assert inside.tolist() == [True, False, False, True]


def test_grid_spec_and_disc_mask():
    spec = GridSpec(0.5, -1.0, -1.0, 8, 6)
    ix, iy = spec.world_to_cell(np.array([-1.0, 0.24, 99.0]), np.array([-1.0, 0.26, 0.0]))
    assert ix.tolist() == [0, 2, 200] and iy.tolist() == [0, 2, 2]
    assert spec.in_bounds(ix, iy).tolist() == [True, True, False]
    x, y = spec.cell_to_world(0, 0)
    assert (float(x), float(y)) == (-0.75, -0.75)
    gx, gy = spec.centers()
    assert gx.shape == (6, 8) and gx[0, 1] == pytest.approx(-0.25)
    assert gy[1, 0] == pytest.approx(-0.25)
    grid = np.arange(48).reshape(6, 8)
    assert spec.lookup(grid, np.array([[-0.9, -0.9], [50.0, 0.0]]), outside=-1).tolist() == [0, -1]
    m = disc_mask(spec, [(0.0, 0.0), (100.0, 100.0)], 0.5)
    assert m.sum() == 4 and m[1:3, 1:3].all()
