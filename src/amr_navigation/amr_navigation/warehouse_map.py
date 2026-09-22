"""
합성 물류센터 점유 격자 (벤치마크·운동학 시뮬레이터용, rclpy 비의존).

60 x 40 m, 0.05 m → 1200 x 800 셀. 정적 배치는 src/amr_simulation/worlds/gen_warehouse_world.py 의
상수(외벽, 랙 A/B/C 열 7베이, 좁은 통로 0.60 m, 기둥 8개, 충전 스테이션, 도크 화물)를 그대로 옮겼다 —
Gazebo 월드와 같은 지도로 계획기를 비교하고, 같은 지도를 Gazebo 시험의 정적 지도로 쓸 수 있다.
좌표계: 원점 = 창고 중심, 지도 원점(좌하단) = (-30, -20).
"""
from dataclasses import dataclass
import math
import os
from typing import List, Optional, Sequence, Tuple

import numpy as np

HALF_X, HALF_Y = 30.0, 20.0
WALL_T = 0.2
RACK_L, RACK_D = 2.0, 1.0
ROWS = {'A': 9.0, 'B': 3.0, 'C': -3.0}
RACK_X = [-18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0]
NARROW_CLEAR = 0.60
NARROW_CX, NARROW_CY = 0.0, -10.0
PILLARS = [(-10, 12), (10, 12), (-10, -12), (10, -12), (-25, 6), (25, 6), (-25, -6), (25, -6)]
PILLAR_SIZE = 0.5
CHARGERS_X = [-26.0, -22.0, -18.0]
CHARGER_Y = -18.4
CHARGER_SIZE = (0.4, 0.5)          # yaw π/2 로 놓인 0.5 x 0.4 상자의 x, y 폭
DOCKS = [(-27.5, 17.0, 'inbound'), (-27.5, 13.0, 'inbound'),
         (27.5, 17.0, 'outbound'), (27.5, 13.0, 'outbound')]
BOX_LARGE = (0.60, 0.50)
BOX_MEDIUM = (0.50, 0.40)
BOX_YAW_MARGIN = 0.12              # [m] 월드의 무작위 yaw(±0.3 rad) 를 덮는 보수적 여유


@dataclass
class GridMap:
    """점유 격자. occ[iy, ix] (iy = 0 이 y 최소, 즉 ROS OccupancyGrid 행 순서)."""

    occ: np.ndarray
    resolution: float
    origin: Tuple[float, float]

    @property
    def width(self) -> int:
        return int(self.occ.shape[1])

    @property
    def height(self) -> int:
        return int(self.occ.shape[0])

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def cell_center(self, ix: int, iy: int) -> Tuple[float, float]:
        return (self.origin[0] + (ix + 0.5) * self.resolution,
                self.origin[1] + (iy + 0.5) * self.resolution)

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height


def static_boxes() -> List[Tuple[float, float, float, float]]:
    """월드 정적 장애물의 축정렬 사각형 목록 [(x0, y0, x1, y1)] (벽 제외)."""
    boxes = []

    def centered(cx, cy, sx, sy):
        boxes.append((cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2))

    for y in ROWS.values():
        for x in RACK_X:
            centered(x, y, RACK_L, RACK_D)
    off = NARROW_CLEAR / 2 + RACK_D / 2
    for sx in (-off, off):
        for dy in (-1.0, 1.0):
            centered(NARROW_CX + sx, NARROW_CY + dy, RACK_D, RACK_L)   # 90° 회전
    for x, y in PILLARS:
        centered(x, y, PILLAR_SIZE, PILLAR_SIZE)
    for x in CHARGERS_X:
        centered(x, CHARGER_Y, *CHARGER_SIZE)
    for x, y, kind in DOCKS:
        bx = x - (1.0 if kind == 'inbound' else -1.0) * 1.2
        centered(bx, y + 1.0, BOX_MEDIUM[0] + BOX_YAW_MARGIN, BOX_MEDIUM[1] + BOX_YAW_MARGIN)
        centered(bx, y - 1.0, BOX_LARGE[0] + BOX_YAW_MARGIN, BOX_LARGE[1] + BOX_YAW_MARGIN)
    return boxes


def build_warehouse(resolution: float = 0.05,
                    extra_boxes: Optional[Sequence[Tuple[float, float, float, float]]] = None
                    ) -> GridMap:
    """합성 창고 지도를 만든다. 셀 중심이 장애물 사각형 안(경계 포함)이면 점유."""
    w = int(round(2 * HALF_X / resolution))
    h = int(round(2 * HALF_Y / resolution))
    origin = (-HALF_X, -HALF_Y)
    xs = origin[0] + (np.arange(w) + 0.5) * resolution
    ys = origin[1] + (np.arange(h) + 0.5) * resolution
    occ = np.zeros((h, w), dtype=bool)
    # 외벽: 안쪽 면 |x| = HALF_X − WALL_T, |y| = HALF_Y − WALL_T
    occ[:, np.abs(xs) >= HALF_X - WALL_T - 1e-9] = True
    occ[np.abs(ys) >= HALF_Y - WALL_T - 1e-9, :] = True
    eps = 1e-9
    for x0, y0, x1, y1 in list(static_boxes()) + list(extra_boxes or []):
        cols = (xs >= x0 - eps) & (xs <= x1 + eps)
        rows = (ys >= y0 - eps) & (ys <= y1 + eps)
        occ[np.ix_(rows, cols)] = True
    return GridMap(occ=occ, resolution=resolution, origin=origin)


def write_map(grid: GridMap, path_prefix: str) -> str:
    """map_server 형식(PGM + YAML)으로 저장하고 YAML 경로를 돌려준다. 점유 0(검정), 자유 254(흰색)."""
    directory = os.path.dirname(os.path.abspath(path_prefix))
    os.makedirs(directory, exist_ok=True)
    pgm = path_prefix + '.pgm'
    img = np.where(grid.occ, 0, 254).astype(np.uint8)[::-1, :]   # PGM 첫 행 = y 최대
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{grid.width} {grid.height}\n255\n'.encode('ascii'))
        f.write(img.tobytes())
    yml = path_prefix + '.yaml'
    with open(yml, 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(pgm)}\n'
                f'mode: trinary\n'
                f'resolution: {grid.resolution}\n'
                f'origin: [{grid.origin[0]}, {grid.origin[1]}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return yml


def load_map(yaml_path: str) -> GridMap:
    """map_server 형식 지도(P5 PGM, negate 0)를 읽는다. 점유 = 픽셀 < (1 − occupied_thresh)·255."""
    import yaml
    with open(yaml_path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    with open(image, 'rb') as f:
        data = f.read()
    tokens = []
    idx = 0
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    if tokens[0] != b'P5':
        raise ValueError(f'{image}: only binary PGM (P5) is supported')
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    img = np.frombuffer(data[idx + 1: idx + 1 + w * h], dtype=np.uint8).reshape(h, w)
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    occ = img[::-1, :].astype(float) / maxval < (1.0 - occ_thresh)
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    return GridMap(occ=occ, resolution=float(meta['resolution']),
                   origin=(float(origin[0]), float(origin[1])))


def distance_field(grid: GridMap) -> np.ndarray:
    """각 셀 중심에서 가장 가까운 점유 셀 중심까지의 유클리드 거리 [m] (정확 EDT)."""
    try:
        from scipy import ndimage
        return ndimage.distance_transform_edt(~grid.occ) * grid.resolution
    except ImportError:  # pragma: no cover - scipy 가 없는 환경의 대체 경로
        return _edt_numpy(grid.occ) * grid.resolution


def _edt_numpy(occ: np.ndarray) -> np.ndarray:
    """Felzenszwalb–Huttenlocher 1D 하한 포락선을 두 축에 적용한 정확 EDT (셀 단위)."""
    inf = 1e20
    f = np.where(occ, 0.0, inf)

    def edt_1d(row: np.ndarray) -> np.ndarray:
        n = len(row)
        d = np.zeros(n)
        v = np.zeros(n, dtype=int)
        z = np.zeros(n + 1)
        k = 0
        z[0], z[1] = -inf, inf
        for q in range(1, n):
            s = ((row[q] + q * q) - (row[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
            while s <= z[k]:
                k -= 1
                s = ((row[q] + q * q) - (row[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
            k += 1
            v[k] = q
            z[k], z[k + 1] = s, inf
        k = 0
        for q in range(n):
            while z[k + 1] < q:
                k += 1
            d[q] = (q - v[k]) ** 2 + row[v[k]]
        return d

    tmp = np.apply_along_axis(edt_1d, 0, f)
    return np.sqrt(np.apply_along_axis(edt_1d, 1, tmp))


Pair = Tuple[float, float, float, float, float, float]


def sample_pairs(grid: GridMap, n: int, seed: int = 42, min_clearance: float = 0.45,
                 min_separation: float = 5.0) -> List[Pair]:
    """
    무작위 출발지-목적지 쌍 [(sx, sy, syaw, gx, gy, gyaw)].

    두 점 모두 여유거리(셀 중심 → 최근접 장애물) ≥ min_clearance (로봇 외접 반경 0.361 + 여유),
    쌍 사이 직선거리 ≥ min_separation. 같은 seed 면 같은 쌍 (재현성).
    """
    rng = np.random.default_rng(seed)
    dist = distance_field(grid)
    iy, ix = np.nonzero(dist >= min_clearance)
    pairs = []
    while len(pairs) < n:
        a, b = rng.integers(0, len(ix), size=2)
        sx, sy = grid.cell_center(int(ix[a]), int(iy[a]))
        gx, gy = grid.cell_center(int(ix[b]), int(iy[b]))
        if math.hypot(gx - sx, gy - sy) < min_separation:
            continue
        syaw, gyaw = rng.uniform(-math.pi, math.pi, size=2)
        pairs.append((round(sx, 3), round(sy, 3), round(float(syaw), 3),
                      round(gx, 3), round(gy, 3), round(float(gyaw), 3)))
    return pairs
