"""
커밋된 지도 maps/warehouse.{yaml,pgm} 와 시뮬레이션 월드의 일치 (통합 계약: map 프레임 = 월드 프레임).

월드·센서가 바뀌었는데 지도를 다시 만들지 않으면(벽 이동, 스캔 높이 변경, map 원점이 SLAM 시작 자세로 저장됨)
여기서 크게 실패한다. 지면 진실 고정값은 amr_simulation worlds/gen_warehouse_world.py 에서 옮겼고, 생성기를
불러올 수 있으면 값이 같은지와 래스터화한 월드 대비 ADNN 도 검사한다 (docs/algorithms/slam.md §3, §6.1).
"""

import importlib.util
import math
import os
from pathlib import Path

from amr_localization import map_quality as mq
from amr_localization import world_geometry as wg
import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
MAP_YAML = Path(os.environ.get('AMR_MAP_YAML', REPO / 'maps' / 'warehouse.yaml'))

# ---- gen_warehouse_world.py 에서 옮긴 지면 진실 (월드 = map 좌표, m) ----
HALF_X, HALF_Y = 30.0, 20.0                       # 외벽 안쪽 면
PILLARS = [(-10, 12), (10, 12), (-10, -12), (10, -12), (-25, 6), (25, 6), (-25, -6), (25, -6)]
PILLAR_BASE_HALF = 0.26                           # models/pillar 하단 보호대 visual 0.52 (스캔 평면 0.20 m)
ROWS = {'A': 9.0, 'B': 3.0, 'C': -3.0}
RACK_X = [-18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0]
RACK_L, RACK_D = 2.0, 1.0                         # 하단 적재 블록 2.0 x 1.0 x 0.50 (스캔 평면을 덮음)
CHARGERS_X = [-26.0, -22.0, -18.0]
CHARGER_Y, STATION_FRONT = -18.4, 0.25            # 스테이션 0.5 x 0.4 (yaw π/2) → 전면 y = -18.15
AISLES = [(x, y) for y in (0.0, 6.0, -6.0)
          for x in (-21.5, -15.0, -9.0, -3.0, 3.0, 9.0, 15.0, 21.5)]


def gen_module():
    """월드 생성기 모듈 (소스 트리 또는 설치 share). 없으면 None."""
    candidates = [REPO / 'src' / 'amr_simulation' / 'worlds' / 'gen_warehouse_world.py']
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('amr_simulation'))
        candidates.append(share / 'worlds' / 'gen_warehouse_world.py')
    except Exception:   # noqa: B902 — 패키지·ament 색인이 없으면 소스 트리만
        pass
    for path in candidates:
        if path.is_file():
            spec = importlib.util.spec_from_file_location('gen_warehouse_world', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    return None


@pytest.fixture(scope='module')
def est():
    if not MAP_YAML.is_file():
        pytest.skip(f'map not found: {MAP_YAML}')
    return mq.load_map(MAP_YAML)


def occupied_near(m: mq.MapImage, pts: np.ndarray, tol: float) -> np.ndarray:
    """각 점에서 tol 안(체비셰프)에 점유 셀이 있는지."""
    k = int(math.ceil(tol / m.resolution))
    out = np.zeros(len(pts), dtype=bool)
    for i, (x, y) in enumerate(pts):
        c = int(math.floor((x - m.origin_x) / m.resolution))
        r = int(math.floor((y - m.origin_y) / m.resolution))
        win = m.occupied[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1]
        out[i] = bool(win.any())
    return out


def face_samples(x0, y0, x1, y1, step=0.1, margin=0.1):
    """면 선분 위 표본점 (끝 margin 제외)."""
    length = math.hypot(x1 - x0, y1 - y0)
    s = np.arange(margin, length - margin + 1e-9, step) / length
    return np.stack([x0 + s * (x1 - x0), y0 + s * (y1 - y0)], 1)


def box_faces(cx, cy, hx, hy):
    """축 정렬 상자의 네 면 (x0, y0, x1, y1)."""
    return [(cx - hx, cy - hy, cx + hx, cy - hy), (cx - hx, cy + hy, cx + hx, cy + hy),
            (cx - hx, cy - hy, cx - hx, cy + hy), (cx + hx, cy - hy, cx + hx, cy + hy)]


def test_map_yaml_contract():
    """해상도 ≤ 0.05 m, 원점 yaw 0 (AMCL·costmap 이 무시), 미지(205)가 미지로 읽힘."""
    if not MAP_YAML.is_file():
        pytest.skip(f'map not found: {MAP_YAML}')
    meta = yaml.safe_load(MAP_YAML.read_text(encoding='utf-8'))
    assert float(meta['resolution']) <= 0.05
    assert meta.get('mode', 'trinary') == 'trinary'
    assert abs(float(meta['origin'][2])) < 1e-9
    assert not mq.unknown_loads_as_free(meta), \
        'free_thresh ≥ 0.196: map_server 가 미지(205)를 자유로 읽는다 (AMCL 전역 재초기화가 미관측 공간에 뿌림)'


def test_unobserved_cells_load_as_unknown(est):
    """벽 밖·기둥/랙 속은 관측되지 않았으므로 미지여야 한다 (자유로 읽히면 안 된다)."""
    unknown = ~est.occupied & ~est.free
    assert unknown.mean() > 0.005
    for x, y in [(-25.0, 6.0), (10.0, 12.0), (0.0, 9.0), (0.0, -3.0), (HALF_X + 0.1, 0.0)]:
        c = int(math.floor((x - est.origin_x) / est.resolution))
        r = int(math.floor((y - est.origin_y) / est.resolution))
        if 0 <= r < est.occupied.shape[0] and 0 <= c < est.occupied.shape[1]:
            assert not est.free[r, c], f'({x}, {y}) 는 관측 불가인데 자유로 읽힌다'


@pytest.mark.parametrize('name, faces', [
    ('pillars', [f for p in PILLARS
                 for f in box_faces(p[0], p[1], PILLAR_BASE_HALF, PILLAR_BASE_HALF)]),
    ('rack aisle faces', [(x - RACK_L / 2, y + s * RACK_D / 2, x + RACK_L / 2, y + s * RACK_D / 2)
                          for y in ROWS.values() for x in RACK_X for s in (-1, 1)]),
    ('rack row ends', [(sx * (RACK_X[-1] + RACK_L / 2), y - RACK_D / 2,
                        sx * (RACK_X[-1] + RACK_L / 2), y + RACK_D / 2)
                       for y in ROWS.values() for sx in (-1, 1)]),
    ('outer walls', [(-HALF_X, -HALF_Y + 1, -HALF_X, -1.0), (-HALF_X, 1.0, -HALF_X, HALF_Y - 1),
                     (HALF_X, -HALF_Y + 1, HALF_X, -1.0), (HALF_X, 1.0, HALF_X, HALF_Y - 1),
                     (-HALF_X + 1, -HALF_Y, -1.0, -HALF_Y), (1.0, -HALF_Y, HALF_X - 1, -HALF_Y),
                     (-HALF_X + 1, HALF_Y, -1.0, HALF_Y), (1.0, HALF_Y, HALF_X - 1, HALF_Y)]),
    ('charger fronts', [(x - 0.2, CHARGER_Y + STATION_FRONT, x + 0.2, CHARGER_Y + STATION_FRONT)
                        for x in CHARGERS_X]),
])
def test_landmarks_occupied_at_world_coordinates(est, name, faces):
    """
    알려진 구조물 면이 월드 좌표 그대로 점유다 (면 표본의 ≥ 90 % 가 1.5 셀 안에 점유 셀).

    map 원점이 SLAM 시작 자세로 저장되면(이전 결함: 16 m 어긋남) 0 %, 0.1° 회전·10 cm 어긋남도 떨어진다.
    """
    pts = np.concatenate([face_samples(*f) for f in faces])
    hit = occupied_near(est, pts, 0.075)
    assert hit.mean() >= 0.9, f'{name}: {hit.mean():.3f} of face samples occupied'


def test_aisles_are_free(est):
    """통로 중앙(로봇이 다니는 곳)은 자유 셀."""
    for x, y in AISLES:
        c = int(math.floor((x - est.origin_x) / est.resolution))
        r = int(math.floor((y - est.origin_y) / est.resolution))
        assert est.free[r, c], f'aisle ({x}, {y}) not free'


def test_fixture_matches_world_generator():
    """위 고정값이 월드 생성기와 같다 (월드가 바뀌면 지도를 다시 만들고 고정값도 옮길 것)."""
    gen = gen_module()
    if gen is None:
        pytest.skip('amr_simulation world generator not importable')
    assert (gen.HALF_X, gen.HALF_Y) == (HALF_X, HALF_Y)
    assert [tuple(map(float, p)) for p in gen.PILLARS] == [tuple(map(float, p)) for p in PILLARS]
    assert gen.ROWS == ROWS and gen.RACK_X == RACK_X
    assert (gen.RACK_L, gen.RACK_D) == (RACK_L, RACK_D)
    assert [x for x, _ in gen.CHARGERS.values()] == CHARGERS_X
    assert (gen.CHARGER_Y, gen.STATION_FRONT) == (CHARGER_Y, STATION_FRONT)
    # 랙 하단 블록이 스캔 평면을 덮어야 랙 외곽선이 지도에 온전히 나온다
    assert gen.DECK_H > wg.scan_plane_height()


def test_map_matches_rasterised_world():
    """
    월드 SDF visual 을 스캔 평면 높이에서 자른 GT 대비 ADNN·잔여 정렬.

    정렬 없이 ADNN ≤ 5 cm, 잔여 정렬 |t| ≤ 3 cm, |yaw| ≤ 맵 끝 반 셀 (0.039°).

    (벽 0.2 m 이동·스캔 높이 0.38→0.20 m 를 반영하지 않은 이전 지도: ADNN 13.3 cm; 0.1° 회전이 남은 지도:
    yaw −0.099° 로 실패)
    """
    gen = gen_module()
    world = REPO / 'src' / 'amr_simulation' / 'worlds' / 'warehouse.sdf'
    models = REPO / 'src' / 'amr_simulation' / 'models'
    if gen is None or not world.is_file() or not MAP_YAML.is_file():
        pytest.skip('world source or map not available')
    est = mq.load_map(MAP_YAML)
    shapes = wg.WorldLoader([models]).load_world(world)
    gt = wg.rasterize(shapes, wg.scan_plane_height(str(REPO / 'config')), est.resolution,
                      (-HALF_X - 1, HALF_X + 1, -HALF_Y - 1, HALF_Y + 1))
    field = mq.GtField(gt)
    occ = est.cell_centers(est.occupied)
    adnn0 = float(field.distance(occ).mean())
    assert adnn0 <= 0.05, f'ADNN at map frame = world {adnn0:.4f} m'
    (x, y, yaw), _ = mq.align(occ, field, (0.0, 0.0, 0.0))
    assert math.hypot(x, y) <= 0.03, f'residual alignment ({x:.4f}, {y:.4f}) m'
    # 남은 회전: 맵 끝에서 반 셀 미만 (그 이하는 격자로 표현할 수 없는 회전)
    limit = mq.min_resample_rotation(est.occupied.shape, est.resolution)
    assert abs(yaw) <= limit, f'residual rotation {math.degrees(yaw):.4f} deg'
