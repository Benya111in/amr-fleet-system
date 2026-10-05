"""negotiation.cost · grid: 중앙 할당과 같은 비용, 격자 A* 비용 선택지."""

import math
import pathlib
import random

import pytest

from amr_fleet import allocation as al
from amr_fleet.negotiation import scenario
from amr_fleet.negotiation.cost import EuclideanCost, GridAstarCost
from amr_fleet.negotiation.grid import OccupancyGrid, read_pgm

MAP_YAML = pathlib.Path(__file__).resolve().parents[3] / 'maps' / 'warehouse.yaml'


def test_euclidean_cost_equals_hungarian_cost_matrix():
    """같은 비용 정의: HungarianStrategy.cost_matrix 와 값이 비트 단위로 같다."""
    for speed in (1.0, 0.7):
        for k in range(20):
            case = scenario.make_case(5, random.Random(k), 'scatter')
            want = al.HungarianStrategy(nominal_speed=speed).cost_matrix(case.tasks, case.robots)
            assert EuclideanCost(speed).matrix(case.robots, case.tasks) == want


def test_cost_model_rejects_bad_speed():
    with pytest.raises(ValueError):
        EuclideanCost(0.0)


def test_scenario_uses_repo_docks_and_spawns_like_allocation_compare():
    import importlib.util
    path = pathlib.Path(__file__).resolve().parents[3] / 'scripts' / 'allocation_compare.py'
    if not path.is_file():
        pytest.skip('scripts/allocation_compare.py 가 없다 (설치 트리)')
    spec = importlib.util.spec_from_file_location('allocation_compare', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.DOCKS == scenario.DOCKS
    assert mod.SPAWNS == scenario.SPAWNS


def test_spawn_mode_places_robots_on_spawns():
    case = scenario.make_case(3, random.Random(0), 'spawn')
    assert [(r.robot_id, r.x, r.y) for r in case.robots] == scenario.SPAWNS
    with pytest.raises(ValueError):
        scenario.make_case(3, random.Random(0), 'nowhere')


def test_central_greedy_and_optimal():
    cost = [[1.0, 2.0], [1.5, 10.0]]
    assert scenario.central_greedy(cost) == {0: 0, 1: 1}           # 탐욕 11.0
    assert scenario.optimal_assignment(cost) == {0: 1, 1: 0}       # 최적 3.5
    assert scenario.total_cost(cost, {0: 1, 1: 0}) == 3.5
    assert scenario.central_greedy([[math.inf, 1.0], [math.inf, 2.0]]) == {1: 0}
    assert scenario.central_greedy([[2.0, 2.0], [2.0, 2.0]]) == {0: 0, 1: 1}   # 동점: 번호 순


# ---------- 격자 A* ----------

def test_astar_open_grid_is_octile():
    grid = OccupancyGrid.from_rows(['.....'] * 5, cell_m=1.0)
    assert grid.path_length((0.5, 0.5), (4.5, 0.5)) == pytest.approx(4.0)
    assert grid.path_length((0.5, 0.5), (4.5, 4.5)) == pytest.approx(4 * math.sqrt(2))
    assert grid.path_length((0.5, 0.5), (3.5, 1.5)) == pytest.approx(2 + math.sqrt(2))


def test_astar_detours_around_wall_and_reports_unreachable():
    rows = ['.....',
            '.###.',
            '.#...',
            '.#.#.',
            '.....']
    grid = OccupancyGrid.from_rows(rows, cell_m=1.0)
    d = grid.path_length((0.5, 4.5), (2.5, 2.5))              # 위 왼쪽 → 벽 안쪽
    assert d > math.hypot(2.0, 2.0) and math.isfinite(d)
    sealed = OccupancyGrid.from_rows(['.#.', '.#.', '.#.'], cell_m=1.0)
    assert sealed.path_length((0.5, 0.5), (2.5, 0.5)) == math.inf


def test_astar_no_corner_cutting():
    grid = OccupancyGrid.from_rows(['.#', '#.'], cell_m=1.0)
    assert grid.path_length((0.5, 1.5), (1.5, 0.5)) == math.inf


def test_snap_to_nearest_free_cell_adds_offset():
    grid = OccupancyGrid.from_rows(['..#'], cell_m=1.0)
    d = grid.path_length((0.5, 0.5), (2.5, 0.5))              # 목표가 막힌 칸 → (1.5, 0.5) 로
    assert d == pytest.approx(2.0)


def test_inflation_and_cache():
    grid = OccupancyGrid.from_rows(['.....', '..#..', '.....'], cell_m=1.0)
    inf = grid.inflated(1.0)
    assert not inf.is_free((1, 1)) and not inf.is_free((2, 2)) and inf.is_free((0, 0))
    assert grid.inflated(0.0) is grid
    grid.path_length((0.5, 0.5), (4.5, 2.5))
    n = grid.n_searches
    grid.path_length((0.5, 0.5), (4.5, 2.5))
    assert grid.n_searches == n                               # 같은 칸 쌍은 기억한다


def test_read_pgm_roundtrip(tmp_path):
    p = tmp_path / 'm.pgm'
    p.write_bytes(b'P5\n# c\n3 2\n255\n' + bytes([0, 254, 205, 254, 254, 0]))
    w, h, px = read_pgm(str(p))
    assert (w, h) == (3, 2) and px == bytes([0, 254, 205, 254, 254, 0])
    y = tmp_path / 'm.yaml'
    y.write_text('image: m.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0]\nnegate: 0\n'
                 'occupied_thresh: 0.65\nfree_thresh: 0.19\n')
    g = OccupancyGrid.from_map_yaml(str(y), cell_m=1.0, inflate_m=0.0)
    # PGM 첫 행이 위쪽(+y): (0,1) 점유, (1,1) 빈칸, (2,1) 미지 / (0,0)·(1,0) 빈칸, (2,0) 점유
    assert [g.is_free((x, 1)) for x in range(3)] == [False, True, False]
    assert [g.is_free((x, 0)) for x in range(3)] == [True, True, False]


def _write_map(tmp_path, width, height, pixels, resolution):
    (tmp_path / 'm.pgm').write_bytes(b'P5\n%d %d\n255\n' % (width, height) + bytes(pixels))
    y = tmp_path / 'm.yaml'
    y.write_text(f'image: m.pgm\nresolution: {resolution}\norigin: [0.0, 0.0, 0]\nnegate: 0\n'
                 'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return str(y)


def test_map_bottom_rows_are_read_when_height_is_not_a_multiple_of_cell(tmp_path):
    """맨 아래 PGM 행(칸 행 0)을 빠짐없이 읽는다 — 리뷰 B1 회귀 (높이 5 화소, 칸 2 화소)."""
    w, h = 4, 5                                    # 0.5 m 화소 → 1.0 m 칸, 높이 나머지 1 행
    px = bytearray([254] * (w * h))
    px[(h - 1) * w + 0] = px[(h - 1) * w + 1] = 0  # 마지막 PGM 행(y 0–0.5 m) 왼쪽 칸에 벽
    for row in (h - 2, h - 1):                     # 오른쪽 아래 칸 화소 4 개 모두 미지
        px[row * w + 2] = px[row * w + 3] = 205
    g = OccupancyGrid.from_map_yaml(_write_map(tmp_path, w, h, px, 0.5),
                                    cell_m=1.0, inflate_m=0.0)
    assert g.cell_of(0.2, 0.2) == (0, 0)
    assert not g.is_free((0, 0))                   # 벽 화소가 든 칸은 막힘
    assert not g.is_free((1, 0))                   # 미지 4/4 > 절반
    assert g.is_free((0, 1)) and g.is_free((1, 1))


def test_map_top_row_and_right_column_remainders_become_edge_cells(tmp_path):
    """위쪽 나머지 행·오른쪽 나머지 열도 칸이 된다 — 둘레 벽이 그 띠에 있어도 사라지지 않는다."""
    w, h = 5, 5                                    # 0.5 m → 1.0 m 칸: 3×3, 위 행·오른 열 칸은 반쪽
    px = bytearray([254] * (w * h))
    px[0] = px[1] = 0                              # PGM 첫 행(y 2.0–2.5 m) 왼쪽에 벽
    px[2 * w + 4] = 0                              # 오른쪽 끝 열(x 2.0–2.5 m), y 1.0–1.5 m 에 벽
    g = OccupancyGrid.from_map_yaml(_write_map(tmp_path, w, h, px, 0.5),
                                    cell_m=1.0, inflate_m=0.0)
    assert (g.width, g.height) == (3, 3)
    assert g.cell_of(2.2, 2.2) == (2, 2)
    assert not g.is_free((0, 2))                   # 위쪽 나머지 행의 벽
    assert not g.is_free((2, 1))                   # 오른쪽 나머지 열의 벽
    assert g.is_free((1, 2))                       # 빈 화소 2 + 지도 밖 2 (= 절반) → 빈칸
    assert g.is_free((2, 0))                       # 빈 화소 2 + 지도 밖 2
    assert not g.is_free((2, 2))                   # 빈 화소 1 + 지도 밖 3 (> 절반) → 막힘
    assert all(g.is_free((x, y)) for x in range(2) for y in range(2))
    inf = g.inflated(1.0)                          # 가장자리 벽도 안쪽으로 팽창한다
    assert not inf.is_free((0, 1)) and not inf.is_free((1, 1))
    assert inf.is_free((0, 0)) and inf.is_free((1, 0))


@pytest.mark.skipif(not MAP_YAML.is_file(), reason='maps/warehouse.yaml 없음')
def test_real_map_grid_cost_is_finite_and_not_shorter_than_euclid():
    grid = OccupancyGrid.from_map_yaml(str(MAP_YAML), cell_m=0.5, inflate_m=0.25)
    assert 0.3 < grid.free_fraction() < 0.95
    # 둘레 벽이 네 변 모두 칸에 들어가고 팽창된다: 아래쪽 벽(PGM 800–803 행)은 높이 806 = 10 × 80 + 6
    # 의 아래 나머지 행에, 위쪽 벽(0–3 행)과 오른쪽 벽(1201–1205 열, 너비 1207)은 위·오른쪽 나머지 띠에 있다
    w, h = grid.width, grid.height
    assert (w, h) == (121, 81)
    ring = ([(x, y) for x in range(w) for y in (0, 1, h - 2, h - 1)]
            + [(x, y) for y in range(h) for x in (0, 1, w - 2, w - 1)])
    assert not any(grid.is_free(c) for c in ring)
    model = GridAstarCost(grid, 1.0)
    case = scenario.make_case(4, random.Random(5), 'spawn', free_xy=grid.is_free_xy)
    euclid = EuclideanCost(1.0).matrix(case.robots, case.tasks)
    path = model.matrix(case.robots, case.tasks)
    for row_e, row_p in zip(euclid, path):
        for e, p in zip(row_e, row_p):
            assert math.isfinite(p)
            assert p >= e - 2 * 0.5 * math.sqrt(2)            # 칸 중심 이동 오차만큼 짧을 수 있다
