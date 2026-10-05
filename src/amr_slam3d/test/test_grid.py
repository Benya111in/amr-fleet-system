"""격자 월드 정렬·map_server 쓰기·읽기·점유 일치율 시험."""

import math

from amr_slam3d import grid
import numpy as np
import pytest


def _l_shape():
    """자기 프레임 격자: 5×4 셀, 0.05 m, 점유 L 자 + 자유 + 미지."""
    d = np.full((4, 5), -1, dtype=np.int16)
    d[0, :] = 100            # 아랫줄 점유 (y = 0 줄)
    d[:, 0] = 100            # 왼쪽 열 점유 (x = 0 열)
    d[1:3, 1:4] = 0          # 자유
    return grid.OccGrid(d, 0.05, (0.0, 0.0, 0.0))


def _occupied_world_centers(g):
    rows, cols = np.nonzero(g.data >= 65)
    return {(round(g.origin[0] + (c + 0.5) * g.resolution, 6),
             round(g.origin[1] + (r + 0.5) * g.resolution, 6)) for r, c in zip(rows, cols)}


def _expected_centers(g, anchor):
    out = set()
    rows, cols = np.nonzero(g.data >= 65)
    c, s = math.cos(anchor[2]), math.sin(anchor[2])
    for r, cc in zip(rows, cols):
        lx, ly = g.origin[0] + (cc + 0.5) * g.resolution, g.origin[1] + (r + 0.5) * g.resolution
        out.add((round(anchor[0] + c * lx - s * ly, 6), round(anchor[1] + s * lx + c * ly, 6)))
    return out


ANCHORS = [(18.0, -16.0, 1.5708), (18.0, -16.0, math.pi / 2), (-3.0, 2.0, 0.0),
           (1.0, 1.0, math.pi), (0.0, 0.0, -math.pi / 2)]


@pytest.mark.parametrize('anchor', ANCHORS)
def test_to_world_moves_cells_exactly_for_right_angles(anchor):
    g = _l_shape()
    w = grid.to_world(g, anchor)
    assert w.origin[2] == 0.0
    assert w.shape in ((4, 5), (5, 4))
    got = _occupied_world_centers(w)
    exp = _expected_centers(g, anchor)
    assert {(round(x, 3), round(y, 3)) for x, y in got} == \
        {(round(x, 3), round(y, 3)) for x, y in exp}
    assert np.count_nonzero(w.data == 0) == np.count_nonzero(g.data == 0)
    assert np.count_nonzero(w.data == -1) == np.count_nonzero(g.data == -1)


def test_to_world_rotation_direction():
    """+90° 고정: 자기 프레임 +x 줄 → 월드 +y 줄 (회전 방향을 반대로 하면 −y 로 간다)."""
    d = np.full((1, 4), 100, dtype=np.int16)      # 자기 프레임 x 0..0.2 줄
    g = grid.OccGrid(d, 0.05, (0.0, 0.0, 0.0))
    w = grid.to_world(g, (0.0, 0.0, math.pi / 2))
    ys = sorted(y for _, y in _occupied_world_centers(w))
    assert all(y > 0 for y in ys)
    assert w.shape == (4, 1)


def test_write_read_roundtrip(tmp_path):
    g = grid.to_world(_l_shape(), (18.0, -16.0, 1.5708))
    pgm, yml = grid.write_map(str(tmp_path / 'm'), g)
    back = grid.read_map(yml)
    assert back.shape == g.shape
    assert back.origin[0] == pytest.approx(g.origin[0]) and back.origin[1] == pytest.approx(
        g.origin[1])
    np.testing.assert_array_equal(back.data >= 65, g.data >= 65)
    np.testing.assert_array_equal(back.data == 0, g.data == 0)
    np.testing.assert_array_equal(back.data == -1, g.data == -1)


def test_write_refuses_rotated_origin(tmp_path):
    with pytest.raises(ValueError):
        grid.write_map(str(tmp_path / 'm'), grid.OccGrid(np.zeros((2, 2), np.int16), 0.05,
                                                         (0.0, 0.0, 0.3)))


def test_trinary_thresholds():
    px = grid.trinary_pixels(np.array([[-1, 0, 19, 20, 64, 65, 100, 101]], dtype=np.int16))
    assert px.tolist() == [[205, 254, 254, 205, 205, 0, 0, 205]]


def test_image_row_order_top_is_max_y(tmp_path):
    d = np.zeros((2, 1), dtype=np.int16)
    d[1, 0] = 100                                   # 위쪽 행(큰 y) 점유
    pgm, _ = grid.write_map(str(tmp_path / 'm'), grid.OccGrid(d, 0.05, (0.0, 0.0, 0.0)))
    img = grid.read_pgm(pgm)
    assert img[0, 0] == 0 and img[1, 0] == 254      # 영상 첫 행 = 최상단 y


def test_occupied_agreement_detects_shift():
    ref = np.full((40, 40), 0, dtype=np.int16)
    ref[20, :] = 100
    r = grid.OccGrid(ref, 0.05, (0.0, 0.0, 0.0))
    same = grid.occupied_agreement(r, r, 1)
    assert same['precision'] == 1.0 and same['recall_observed'] == 1.0
    shifted = grid.OccGrid(np.roll(ref, 4, axis=0), 0.05, (0.0, 0.0, 0.0))   # 0.20 m 어긋남
    bad = grid.occupied_agreement(shifted, r, 1)
    assert bad['precision'] == 0.0
    # 원점으로 같은 어긋남을 되돌리면 다시 일치한다 (원점 오프셋을 정수 셀로 맞추는지)
    fixed = grid.OccGrid(shifted.data, 0.05, (0.0, -0.20, 0.0))
    assert grid.occupied_agreement(fixed, r, 1)['precision'] == 1.0
