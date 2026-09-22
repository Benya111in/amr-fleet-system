"""
스캔-맵 일치도 (납치 감지의 주 신호, rclpy 비의존).

점유격자에서 "가장 가까운 점유 셀까지 거리" 장(likelihood field, AMCL 관측 모델과 같은 구조)을
유클리드 거리 변환으로 한 번 만들고, 현재 map 자세로 옮긴 스캔 끝점마다 그 거리를 조회한다.
    ρ = |{ i : d(끝점_i) ≤ inlier_dist }| / |{ 유효 빔 }|
정상 추적이면 ρ 는 동적 장애물 비율만큼만 떨어지고(0.8 이상), 다른 곳으로 납치되면 끝점이
자유 공간/벽 안쪽에 떨어져 급락한다. 복잡도 O(빔 수) (거리장 조회), 거리장 생성은 맵당 1회.
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

import numpy as np
from scipy import ndimage


@dataclass
class GridSpec:
    """점유격자 메타데이터 (nav_msgs/OccupancyGrid.info 와 같은 의미)."""

    width: int
    height: int
    resolution: float          # [m/cell]
    origin_x: float = 0.0      # [m] 셀 (0, 0) 의 모서리
    origin_y: float = 0.0
    origin_yaw: float = 0.0    # [rad]


class DistanceField:
    """점유 셀까지의 거리장 [m]."""

    def __init__(self, data: Sequence[int], spec: GridSpec, occupied_thresh: int = 65,
                 max_distance: float = 2.0, free_thresh: int = 25) -> None:
        """
        점유격자 데이터(nav_msgs/OccupancyGrid, 행 우선, −1 미지 / 0~100 점유 확률)로 거리장을 만든다.

        max_distance 로 잘라 두면 맵 밖·먼 곳 조회값이 일정해진다. free = 알려진 자유 셀.
        """
        grid = np.asarray(data, dtype=np.int16).reshape(spec.height, spec.width)
        occupied = grid >= occupied_thresh
        self.free = (grid >= 0) & (grid <= free_thresh)
        self.spec = spec
        self.max_distance = max_distance
        if occupied.any():
            dist = ndimage.distance_transform_edt(~occupied) * spec.resolution
        else:
            dist = np.full(grid.shape, max_distance)
        self.field = np.minimum(dist, max_distance).astype(np.float32)
        # 조회용: 테두리 1 셀을 max_distance 로 두른 평탄 배열 → 맵 밖 좌표는 테두리로 클립되어
        # 불리언 마스킹 없이 한 번의 take 로 끝난다 (전역 가설 탐색이 수천만 번 조회)
        padded = np.full((spec.height + 2, spec.width + 2), max_distance, dtype=np.float32)
        padded[1:-1, 1:-1] = self.field
        self._flat = padded.ravel()
        self._stride = spec.width + 2
        self._c = math.cos(spec.origin_yaw) / spec.resolution
        self._s = math.sin(spec.origin_yaw) / spec.resolution

    def lookup(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """점들(map 좌표)의 최근접 점유 거리 [m] (맵 밖·비유한 좌표는 max_distance)."""
        dx = np.asarray(xs, dtype=np.float64) - self.spec.origin_x
        dy = np.asarray(ys, dtype=np.float64) - self.spec.origin_y
        with np.errstate(invalid='ignore'):    # inf × 0 → NaN 은 아래에서 맵 밖으로 보낸다
            gx = self._c * dx + self._s * dy   # [셀]
            gy = self._c * dy - self._s * dx
        # 비유한 좌표는 맵 밖(테두리)으로 보낸다
        np.nan_to_num(gx, copy=False, nan=-1.0, posinf=-1.0, neginf=-1.0)
        np.nan_to_num(gy, copy=False, nan=-1.0, posinf=-1.0, neginf=-1.0)
        col = np.clip(np.floor(gx), -1, self.spec.width).astype(np.int64)
        row = np.clip(np.floor(gy), -1, self.spec.height).astype(np.int64)
        return self._flat[(row + 1) * self._stride + (col + 1)].astype(np.float64)


def scan_endpoints(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_max: float, max_beams: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    """
    센서 프레임 끝점 (x, y). 무효(NaN/±Inf, range_max 이상) 빔은 뺀다.

    max_beams > 0 이면 균등 간격으로 솎는다 (AMCL max_beams 와 같은 방식).
    """
    r = np.asarray(ranges, dtype=np.float64)
    idx = np.arange(r.size)
    if 0 < max_beams < r.size:
        idx = np.unique(np.linspace(0, r.size - 1, max_beams).round().astype(np.int64))
    rr = r[idx]
    valid = np.isfinite(rr) & (rr < range_max) & (rr > 0.0)
    ang = angle_min + idx[valid] * angle_increment
    return rr[valid] * np.cos(ang), rr[valid] * np.sin(ang)


def transform_points(xs: np.ndarray, ys: np.ndarray,
                     pose: Tuple[float, float, float]) -> Tuple[np.ndarray, np.ndarray]:
    """2D 강체 변환 (pose 좌표계 → 부모 좌표계)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * xs - s * ys, pose[1] + s * xs + c * ys


def compose(a: Tuple[float, float, float],
            b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """T_a T_b."""
    x, y = transform_points(np.array([b[0]]), np.array([b[1]]), a)
    return float(x[0]), float(y[0]), math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2]))


def match_ratio(field: DistanceField, base_pose: Tuple[float, float, float],
                sensor_offset: Tuple[float, float, float], ranges: Sequence[float],
                angle_min: float, angle_increment: float, range_max: float,
                inlier_dist: float = 0.2, max_beams: int = 180,
                explain_unmapped: bool = True) -> Tuple[float, int]:
    """
    인라이어 비율 ρ 와 **판정에 쓴** 빔 수.

    base_pose: map 프레임 base_footprint 자세, sensor_offset: base_footprint → lidar 2D 변환.

    explain_unmapped 면 "지도에 없는 물체에 막혀 일찍 끝난 빔"(끝점이 점유에서 멀고, 센서→끝점 사이 지도에
    막는 것이 없다)은 분자·분모에서 모두 뺀다. 지도에 있는 벽을 **뚫고** 지나간 빔만 불일치로 센다.
    이렇게 하지 않으면 통로를 가로막은 큰 미지 장애물(6 × 0.5 m 벽) 앞에서 올바르게 위치 추정한 로봇의 ρ 가
    0.5 아래로 떨어져 거짓 LOST → 엉뚱한 전역 재초기화가 났다 (통합 시나리오 11 실측).
    """
    xs, ys = scan_endpoints(ranges, angle_min, angle_increment, range_max, max_beams)
    if xs.size == 0:
        return 0.0, 0
    sensor = compose(base_pose, sensor_offset)
    mx, my = transform_points(xs, ys, sensor)
    d = field.lookup(mx, my)
    inlier = d <= inlier_dist
    if not explain_unmapped:
        return float(np.count_nonzero(inlier)) / xs.size, int(xs.size)
    counted = inlier | _blocked_by_map(field, sensor, mx, my)
    n = int(np.count_nonzero(counted))
    if n == 0:
        return 0.0, 0
    return float(np.count_nonzero(inlier)) / n, n


def _blocked_by_map(field: DistanceField, sensor: Tuple[float, float, float],
                    mx: np.ndarray, my: np.ndarray, margin_cells: float = 1.5) -> np.ndarray:
    """빔마다 센서~끝점 사이에 지도의 점유 셀이 있으면 True (= 벽을 뚫고 간 빔)."""
    res = field.spec.resolution
    dx, dy = mx - sensor[0], my - sensor[1]
    length = np.hypot(dx, dy)
    stop = np.maximum(length - margin_cells * res, 0.0)          # 끝점 바로 앞까지만 본다
    steps = int(np.ceil(float(np.max(stop, initial=0.0)) / res)) + 1
    steps = max(1, min(steps, 400))                              # 20 m / 0.05 m 상한
    t = np.linspace(0.0, 1.0, steps)[None, :]                    # (1, steps)
    with np.errstate(invalid='ignore', divide='ignore'):
        frac = np.where(length > 0.0, stop / np.maximum(length, 1e-9), 0.0)[:, None]
    sx = sensor[0] + dx[:, None] * frac * t
    sy = sensor[1] + dy[:, None] * frac * t
    hit = field.lookup(sx.ravel(), sy.ravel()).reshape(sx.shape) <= 0.5 * res
    return np.any(hit, axis=1)
