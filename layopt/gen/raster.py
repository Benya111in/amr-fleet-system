"""
월드 SDF → 점유 지도 (contract §1: 0.05 m, 스캔 높이 0.20 m 단면). 기준 배치도 같은 방식으로 만든다.

amr_localization/world_geometry.py (WorldLoader + rasterize, 시각체 단면) 를 그대로 쓰고 두 가지만 정한다:
- 움직이는 모델은 뺀다: world_geometry.DEFAULT_EXCLUDE(ground_plane, forklift, worker, amr_) + shuttle_amr
  (셔틀은 왕복 차량인데 DEFAULT_EXCLUDE 에 없어 출발 자세에 상자가 찍힌다 — b2-eval GT_EXCLUDE 와 같은 판단).
  도크 화물 상자(밀 수 있는 정적 시작 자세)는 SLAM 지도처럼 넣는다.
- 범위 [-30.5, 30.5] x [-20.5, 20.5] (world_to_map 기본). 외벽 바깥 면(|x| > 30.2, |y| > 20.2) 밖 셀은
  미지(205)로 쓴다 — SLAM 지도처럼 벽 밖을 자유로 두지 않는다 (전역 재위치추정 후보가 벽 밖 띠에 생기지 않게).
numpy·yaml 이 필요하다 (컨테이너).
"""

import math
import sys
from pathlib import Path

import numpy as np

from layopt.gen.common import MODELS_DIR, REPO

sys.path.insert(0, str(REPO / 'src' / 'amr_localization'))
from amr_localization import world_geometry as wg  # noqa: E402

EXCLUDE = tuple(wg.DEFAULT_EXCLUDE) + ('shuttle_amr',)
BOUNDS = (-30.5, 30.5, -20.5, 20.5)
RESOLUTION = 0.05
SCAN_HEIGHT = 0.20
OUTER = (30.2, 20.2)              # 외벽 바깥 면 (안쪽 면 ±30/±20 + 두께 0.2)


def rasterize_world(world_path, height=SCAN_HEIGHT, resolution=RESOLUTION, bounds=BOUNDS):
    """→ (Raster, 미지 마스크 bool, 건너뛴 시각체 목록)."""
    loader = wg.WorldLoader([MODELS_DIR], EXCLUDE)
    shapes = loader.load_world(Path(world_path))
    raster = wg.rasterize(shapes, height, resolution, bounds)
    rows, cols = raster.occupied.shape
    cx = raster.origin_x + (np.arange(cols) + 0.5) * resolution
    cy = raster.origin_y + (np.arange(rows) + 0.5) * resolution
    outside = (np.abs(cx)[None, :] > OUTER[0]) | (np.abs(cy)[:, None] > OUTER[1])
    unknown = outside & ~raster.occupied
    return raster, unknown, loader.skipped


def write_map(raster, unknown, stem):
    """map_server 형식 (P5 PGM + YAML): 점유 0, 자유 254, 미지 205, 위쪽 행 = +y 끝 (world_geometry 규약)."""
    stem = Path(stem)
    img = np.where(raster.occupied, 0, np.where(unknown, 205, 254)).astype(np.uint8)[::-1]
    pgm = stem.with_suffix('.pgm')
    with pgm.open('wb') as f:
        f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode())
        f.write(img.tobytes())
    yml = stem.with_suffix('.yaml')
    yml.write_text(
        f'image: {pgm.name}\nmode: trinary\nresolution: {raster.resolution}\n'
        f'origin: [{raster.origin_x}, {raster.origin_y}, 0.0]\nnegate: 0\n'
        f'occupied_thresh: 0.65\nfree_thresh: 0.19\n'
        f'# layopt/gen/raster.py: 월드 SDF 시각체의 스캔 높이 {SCAN_HEIGHT} m 단면 '
        f'(world_geometry), 제외 {",".join(EXCLUDE)}, 외벽 밖 = 미지\n', encoding='utf-8')
    return pgm, yml


def clearance(occupied_or_unknown, resolution=RESOLUTION):
    """셀 중심에서 가장 가까운 점유(또는 미지) 셀 가장자리까지 거리 [m]."""
    from scipy import ndimage
    return ndimage.distance_transform_edt(~occupied_or_unknown) * resolution - 0.5 * resolution


def cell_of(x, y, raster):
    c = int(math.floor((x - raster.origin_x) / raster.resolution))
    r = int(math.floor((y - raster.origin_y) / raster.resolution))
    return r, c
