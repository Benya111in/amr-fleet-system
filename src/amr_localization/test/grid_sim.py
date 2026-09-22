"""테스트용 합성 점유격자 + 광선 투사 스캔 (pytest 헬퍼)."""

import math
from typing import List, Sequence, Tuple

import numpy as np

RES = 0.05


def make_room(width: float = 20.0, height: float = 14.0,
              boxes: Sequence[Tuple[float, float, float, float]] = (
                  (4.0, 3.0, 1.0, 2.0), (13.0, 9.0, 3.0, 0.6), (8.0, 10.5, 0.4, 0.4),
                  (16.0, 2.0, 0.6, 3.0))) -> Tuple[np.ndarray, float]:
    """
    외벽 + 비대칭 상자 방. 반환: (grid[int8, row 0 = y 0], resolution).

    OccupancyGrid 규약 값: 100 점유, 0 자유. 원점 (0, 0), 방 안쪽은 [0.1, width−0.1].
    """
    rows, cols = int(round(height / RES)), int(round(width / RES))
    g = np.zeros((rows, cols), dtype=np.int8)
    t = 2
    g[:t, :] = g[-t:, :] = 100
    g[:, :t] = g[:, -t:] = 100
    for x, y, w, h in boxes:
        c0, c1 = int(x / RES), int((x + w) / RES)
        r0, r1 = int(y / RES), int((y + h) / RES)
        g[r0:r1, c0:c1] = 100
    return g, RES


def raycast(grid: np.ndarray, res: float, pose: Tuple[float, float, float],
            n_beams: int = 720, range_max: float = 25.0, noise: float = 0.0,
            seed: int = 0) -> Tuple[List[float], float, float]:
    """격자에서 pose 의 360° 스캔 (angle_min −π, 증분 2π/n) → (ranges, angle_min, inc)."""
    rng = np.random.default_rng(seed)
    angle_min = -math.pi
    inc = 2.0 * math.pi / n_beams
    step = res / 4.0
    ranges = []
    for i in range(n_beams):
        a = pose[2] + angle_min + i * inc
        dx, dy = math.cos(a), math.sin(a)
        r = 0.0
        hit = math.inf
        while r < range_max:
            x, y = pose[0] + r * dx, pose[1] + r * dy
            c, rr = int(x / res), int(y / res)
            if not (0 <= rr < grid.shape[0] and 0 <= c < grid.shape[1]):
                break
            if grid[rr, c] >= 65:
                hit = r
                break
            r += step
        ranges.append(hit + (rng.normal(0.0, noise) if math.isfinite(hit) and noise else 0.0))
    return ranges, angle_min, inc
