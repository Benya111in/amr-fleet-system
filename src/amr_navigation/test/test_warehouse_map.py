"""합성 창고 지도 생성·저장·읽기·거리장·무작위 쌍 단위 테스트."""
import math

from amr_navigation import warehouse_map as wm
import numpy as np
import pytest


@pytest.fixture(scope='module')
def grid():
    return wm.build_warehouse(0.05)


def test_size_and_origin(grid):
    assert (grid.width, grid.height) == (1200, 800)
    assert grid.origin == (-30.0, -20.0)
    assert grid.world_to_cell(-30.0, -20.0) == (0, 0)
    assert grid.cell_center(0, 0) == pytest.approx((-29.975, -19.975))
    assert grid.in_bounds(1199, 799) and not grid.in_bounds(1200, 0)


def test_walls_racks_and_free_aisle(grid):
    # 외벽은 점유, 창고 중심(스폰)은 자유
    assert grid.occ[0, 600] and grid.occ[400, 0] and grid.occ[799, 600]
    ix, iy = grid.world_to_cell(0.0, 0.0)
    assert not grid.occ[iy, ix]
    # 랙 A1 중심 (-18, 9)
    ix, iy = grid.world_to_cell(-18.0, 9.0)
    assert grid.occ[iy, ix]
    # 기둥 (10, 12)
    ix, iy = grid.world_to_cell(10.0, 12.0)
    assert grid.occ[iy, ix]


def test_narrow_passage_is_060m(grid):
    # 좁은 통로 (0, -10): y = -10 에서 x 방향으로 자유 셀 폭 = 0.60 m (12 셀).
    # 양쪽 랙(90° 회전, x 폭 1.0 m)이 |x| ∈ [0.3, 1.3] 을 덮으므로 |x| ≤ 1.2 창에서는 통로만 비어 있다.
    _, iy = grid.world_to_cell(0.0, -10.0)
    ix0, _ = grid.world_to_cell(-1.2, -10.0)
    ix1, _ = grid.world_to_cell(1.2, -10.0)
    row = grid.occ[iy, ix0:ix1]
    free = np.nonzero(~row)[0]
    assert len(free) == 12
    assert np.all(np.diff(free) == 1)


def test_write_and_load_roundtrip(tmp_path, grid):
    yml = wm.write_map(grid, str(tmp_path / 'wh'))
    back = wm.load_map(yml)
    assert back.resolution == pytest.approx(0.05)
    assert back.origin == pytest.approx(grid.origin)
    assert np.array_equal(back.occ, grid.occ)
    text = (tmp_path / 'wh.yaml').read_text(encoding='utf-8')
    assert 'image: wh.pgm' in text and 'mode: trinary' in text


def test_load_rejects_ascii_pgm(tmp_path):
    (tmp_path / 'a.pgm').write_bytes(b'P2\n# c\n2 1\n255\n0 254\n')
    (tmp_path / 'a.yaml').write_text('image: a.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n',
                                     encoding='utf-8')
    with pytest.raises(ValueError):
        wm.load_map(str(tmp_path / 'a.yaml'))


def test_distance_field_exact_small():
    occ = np.zeros((7, 9), dtype=bool)
    occ[3, 4] = True
    g = wm.GridMap(occ=occ, resolution=0.1, origin=(0.0, 0.0))
    d = wm.distance_field(g)
    assert d[3, 4] == 0.0
    assert d[3, 6] == pytest.approx(0.2)
    assert d[0, 0] == pytest.approx(0.1 * math.hypot(4, 3))
    # 대체 구현(Felzenszwalb)도 같은 값
    assert np.allclose(wm._edt_numpy(occ) * 0.1, d)


def test_sample_pairs_reproducible_and_valid(grid):
    a = wm.sample_pairs(grid, 10, seed=3)
    b = wm.sample_pairs(grid, 10, seed=3)
    assert a == b
    d = wm.distance_field(grid)
    for sx, sy, syaw, gx, gy, gyaw in a:
        assert math.hypot(gx - sx, gy - sy) >= 5.0
        for x, y in ((sx, sy), (gx, gy)):
            ix, iy = grid.world_to_cell(x, y)
            assert d[iy, ix] >= 0.45 - 1e-9
        assert -math.pi <= syaw <= math.pi and -math.pi <= gyaw <= math.pi


def test_extra_boxes(grid):
    g = wm.build_warehouse(0.1, extra_boxes=[(1.0, 1.0, 2.0, 2.0)])
    ix, iy = g.world_to_cell(1.5, 1.5)
    assert g.occ[iy, ix]
    assert len(wm.static_boxes()) == 21 + 4 + 8 + 3 + 8
