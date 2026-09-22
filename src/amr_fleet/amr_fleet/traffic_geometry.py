"""
교통 관리 공용 기하 (traffic_manager_node, components.md §3.6).

평면 좌표는 map 프레임 [m], 경로(polyline)는 (N, 2) numpy 배열이다.
호 길이 s 는 경로 시작점 기준 누적 거리 [m]. ROS 에 의존하지 않는 순수 모듈.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]


def wrap_angle(a: float) -> float:
    """각도를 [-pi, pi) 로 정규화."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    """두 방향의 차 |a - b| 를 [0, pi] 로."""
    return abs(wrap_angle(a - b))


def as_path(points: Optional[Iterable[Sequence[float]]]) -> Optional[np.ndarray]:
    """점 목록 → (N, 2) float 배열. 연속 중복점은 뺀다. 비었으면 None."""
    if points is None:
        return None
    if not isinstance(points, np.ndarray):
        points = list(points)
    arr = np.asarray(points, dtype=float)
    if arr.size == 0:
        return None
    arr = arr.reshape(-1, 2)
    if len(arr) > 1:
        keep = np.ones(len(arr), dtype=bool)
        keep[1:] = np.any(np.abs(np.diff(arr, axis=0)) > 1e-9, axis=1)
        arr = arr[keep]
    return arr


def cumulative_length(path: np.ndarray) -> np.ndarray:
    """경로 각 점까지의 누적 호 길이 (N,). 첫 값은 0."""
    if len(path) < 2:
        return np.zeros(len(path))
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(seg)))


def project(path: np.ndarray, cum: np.ndarray, x: float, y: float) -> Tuple[float, float]:
    """점 (x, y) 에 가장 가까운 경로 위 점의 (호 길이 s, 거리)."""
    p = np.array([x, y], dtype=float)
    if len(path) == 1:
        return 0.0, float(np.linalg.norm(p - path[0]))
    a = path[:-1]
    ab = path[1:] - a
    ab2 = np.einsum('ij,ij->i', ab, ab)
    t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(ab2, 1e-12), 0.0, 1.0)
    closest = a + ab * t[:, None]
    d = np.linalg.norm(closest - p, axis=1)
    k = int(np.argmin(d))
    return float(cum[k] + t[k] * math.sqrt(ab2[k])), float(d[k])


def points_at(path: np.ndarray, cum: np.ndarray,
              s: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """호 길이 배열 s → (위치 (M, 2), 진행 방향 (M,)). s 는 [0, 길이] 로 자른다."""
    s = np.clip(np.asarray(s, dtype=float), 0.0, cum[-1] if len(cum) else 0.0)
    if len(path) < 2:
        xy = np.repeat(path[:1], len(s), axis=0)
        return xy, np.zeros(len(s))
    xy = np.stack([np.interp(s, cum, path[:, 0]), np.interp(s, cum, path[:, 1])], axis=1)
    seg = np.diff(path, axis=0)
    seg_heading = np.arctan2(seg[:, 1], seg[:, 0])
    idx = np.clip(np.searchsorted(cum, s, side='right') - 1, 0, len(seg) - 1)
    return xy, seg_heading[idx]


def sub_path(path: np.ndarray, cum: np.ndarray, s0: float, s1: float,
             step: float) -> np.ndarray:
    """[s0, s1] 구간을 step 간격으로 다시 뽑은 점열 (양 끝 포함, 최소 1점)."""
    total = float(cum[-1]) if len(cum) else 0.0
    s0 = min(max(s0, 0.0), total)
    s1 = min(max(s1, s0), total)
    n = max(1, int(math.ceil((s1 - s0) / max(step, 1e-6))))
    xy, _ = points_at(path, cum, np.linspace(s0, s1, n + 1))
    return xy


def distance_to_path(points: np.ndarray, path: np.ndarray) -> np.ndarray:
    """각 점에서 경로(선분열)까지의 최단 거리 (M,)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(path) == 1:
        return np.linalg.norm(pts - path[0], axis=1)
    a = path[:-1][None, :, :]
    ab = (path[1:] - path[:-1])[None, :, :]
    ap = pts[:, None, :] - a
    ab2 = np.maximum(np.sum(ab * ab, axis=2), 1e-12)
    t = np.clip(np.sum(ap * ab, axis=2) / ab2, 0.0, 1.0)
    d = np.linalg.norm(ap - ab * t[:, :, None], axis=2)
    return d.min(axis=1)


def points_in_polygon(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """다각형 내부 판정 (ray casting), (M,) bool. 경계 위 점은 어느 쪽이든 될 수 있다."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y = pts[:, 0], pts[:, 1]
    inside = np.zeros(len(pts), dtype=bool)
    xs, ys = polygon[:, 0], polygon[:, 1]
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi, xj, yj = xs[i], ys[i], xs[j], ys[j]
        crosses = (yi > y) != (yj > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= crosses & (x < x_cross)
        j = i
    return inside


def polygon_area(polygon: np.ndarray) -> float:
    """다각형 넓이 [m^2] (신발끈 공식, 부호 없음)."""
    x, y = polygon[:, 0], polygon[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


@dataclasses.dataclass(frozen=True)
class GridSpec:
    """점유 격자 기하: 셀 (ix, iy) 의 중심 = origin + (i + 0.5) * resolution (회전 없음)."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def world_to_cell(self, x, y):
        """월드 좌표 → 셀 인덱스 (ix, iy) (범위 밖일 수 있다)."""
        ix = np.floor((np.asarray(x, dtype=float) - self.origin_x) / self.resolution)
        iy = np.floor((np.asarray(y, dtype=float) - self.origin_y) / self.resolution)
        return ix.astype(int), iy.astype(int)

    def cell_to_world(self, ix, iy):
        """셀 인덱스 → 셀 중심 world 좌표."""
        x = self.origin_x + (np.asarray(ix, dtype=float) + 0.5) * self.resolution
        y = self.origin_y + (np.asarray(iy, dtype=float) + 0.5) * self.resolution
        return x, y

    def in_bounds(self, ix, iy):
        """셀 인덱스가 격자 안인지."""
        ix, iy = np.asarray(ix), np.asarray(iy)
        return (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)

    def centers(self) -> Tuple[np.ndarray, np.ndarray]:
        """모든 셀 중심 (X, Y), 각각 (height, width)."""
        xs, ys = self.cell_to_world(np.arange(self.width), np.arange(self.height))
        return np.meshgrid(xs, ys)

    def lookup(self, grid: np.ndarray, xy: np.ndarray, outside=0):
        """격자 값 조회. 범위 밖은 outside."""
        pts = np.asarray(xy, dtype=float).reshape(-1, 2)
        ix, iy = self.world_to_cell(pts[:, 0], pts[:, 1])
        ok = self.in_bounds(ix, iy)
        out = np.full(len(pts), outside, dtype=grid.dtype)
        out[ok] = grid[iy[ok], ix[ok]]
        return out


def disc_mask(spec: GridSpec, centers: Iterable[Sequence[float]], radius: float) -> np.ndarray:
    """중심 목록 주변 radius 안의 셀 (height, width) bool."""
    mask = np.zeros((spec.height, spec.width), dtype=bool)
    r_cells = int(math.ceil(radius / spec.resolution)) + 1
    for c in centers:
        cx, cy = float(c[0]), float(c[1])
        ix, iy = spec.world_to_cell(cx, cy)
        x0, x1 = max(0, int(ix) - r_cells), min(spec.width, int(ix) + r_cells + 1)
        y0, y1 = max(0, int(iy) - r_cells), min(spec.height, int(iy) + r_cells + 1)
        if x0 >= x1 or y0 >= y1:
            continue
        xs, ys = spec.cell_to_world(np.arange(x0, x1), np.arange(y0, y1))
        gx, gy = np.meshgrid(xs, ys)
        mask[y0:y1, x0:x1] |= (gx - cx) ** 2 + (gy - cy) ** 2 <= radius ** 2
    return mask
