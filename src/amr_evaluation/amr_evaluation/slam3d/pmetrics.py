"""
P 단계 재생 한 건의 보조 지표 (rclpy 비의존) — 통로 축·횡 분해, S6 블로커 가림, 스캔당 지연, 3D 점 지도 영역.

- `aisle_errors`: 정렬 없는 위치 오차를 통로 축(y)·횡(x) 성분으로 나눈다. 표본은 **GT 위치**가 통로 상자 안인 것만
  (S3·S6 — evaluation.md §6 의 좁은 통로: 중심 (0, −10), y −12…−8, 순폭 0.60 m).
- `mask_boxes`: 2D 지도 지표에서 S6 블로커 발자국(+ 여유 0.10 m, evaluation.md §6.1)을 추정·GT 양쪽에서 지운다
  (점유·알려짐 모두 거짓 → 정밀도·재현율 어느 쪽에도 들어가지 않는다).
- `blocker_rects`: localizability.s6_blockers 의 OBB → 월드 xy 축 정렬 사각형 (여유 포함).
- `icp_latency` / `rtabmap_latency`: RTAB-Map 로그의 스캔당 처리 시간 (icp_odometry "update time=", rtabmap
  "RTAB-Map=") 분위수 [ms] — 공유 부하 하 예비값.
- `region_raster` / `in_region`: GT 궤적 xy 에서 반경 r 안의 영역 (3D F-score 를 두 지도에 같은 영역으로 자른다).
"""

from __future__ import annotations

import math
import re
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

#: 좁은 통로 상자 (GT 위치 기준) — x 순폭 ±0.30, y −12…−8 (evaluation.md §6.1)
AISLE_BOX = (-0.30, 0.30, -12.0, -8.0)
#: S6 블로커 가림 여유 [m] (evaluation.md §6.1)
BLOCKER_MARGIN = 0.10

Rect = Tuple[float, float, float, float]          # xmin, xmax, ymin, ymax


def _stats(e: np.ndarray) -> Dict[str, float]:
    e = np.asarray(e, dtype=float).reshape(-1)
    if e.size == 0:
        return {'rmse': math.nan, 'max': math.nan, 'p95': math.nan, 'mean': math.nan, 'n': 0}
    a = np.abs(e)
    return {'rmse': float(np.sqrt(np.mean(e ** 2))), 'max': float(a.max()),
            'p95': float(np.percentile(a, 95)), 'mean': float(e.mean()), 'n': int(e.size)}


def aisle_errors(gt_xy: np.ndarray, est_xy: np.ndarray, box: Rect = AISLE_BOX) -> Dict[str, dict]:
    """
    짝지은 (같은 시각) GT·추정 위치 → 통로 상자 안 표본의 축(y)·횡(x) 오차 통계.

    'axis'·'lateral' 은 부호 있는 오차 e = 추정 − GT 의 통계 (rmse, |e| 의 max·p95, 부호 평균 mean, n).
    """
    g = np.asarray(gt_xy, dtype=float).reshape(-1, 2)
    e = np.asarray(est_xy, dtype=float).reshape(-1, 2)
    m = (g[:, 0] >= box[0]) & (g[:, 0] <= box[1]) & (g[:, 1] >= box[2]) & (g[:, 1] <= box[3])
    d = e[m] - g[m]
    return {'axis': _stats(d[:, 1]), 'lateral': _stats(d[:, 0])}


def blocker_rects(boxes, margin: float = BLOCKER_MARGIN) -> List[Rect]:
    """localizability.Boxes (OBB) → xy 축 정렬 외접 사각형 (+ 여유)."""
    if boxes is None:
        return []
    out = []
    for c, r, h in zip(boxes.c, boxes.R, boxes.h):
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                p = c + r @ np.array([sx * h[0], sy * h[1], 0.0])
                corners.append(p[:2])
        corners = np.array(corners)
        out.append((float(corners[:, 0].min() - margin), float(corners[:, 0].max() + margin),
                    float(corners[:, 1].min() - margin), float(corners[:, 1].max() + margin)))
    return out


def cell_mask(shape: Tuple[int, int], resolution: float, origin_x: float, origin_y: float,
              rects: Sequence[Rect]) -> np.ndarray:
    """격자 (행 0 = 최하단 y) 에서 셀 **중심**이 사각형 안인 셀 (bool)."""
    rows, cols = shape
    m = np.zeros(shape, dtype=bool)
    if not rects:
        return m
    xs = origin_x + (np.arange(cols) + 0.5) * resolution
    ys = origin_y + (np.arange(rows) + 0.5) * resolution
    for x0, x1, y0, y1 in rects:
        cx = (xs >= x0) & (xs <= x1)
        cy = (ys >= y0) & (ys <= y1)
        m |= cy[:, None] & cx[None, :]
    return m


def mask_boxes(grid, rects: Sequence[Rect]):
    """gridmap.Grid 사본에서 사각형 안 셀을 점유·알려짐 모두 거짓으로 (지표에서 뺀다)."""
    from dataclasses import replace
    m = cell_mask(grid.shape, grid.resolution, grid.origin_x, grid.origin_y, rects)
    return replace(grid, occupied=grid.occupied & ~m, known=grid.known & ~m)


_ICP = re.compile(r'Odom: ratio=[^\n]*?update time=([0-9.]+)s')
_RTAB = re.compile(r'RTAB-Map=([0-9.]+)s')


def _q(v: List[float]) -> Dict[str, float]:
    if not v:
        return {'p50_ms': math.nan, 'p95_ms': math.nan, 'max_ms': math.nan, 'n': 0}
    a = np.asarray(v) * 1000.0
    return {'p50_ms': float(np.percentile(a, 50)), 'p95_ms': float(np.percentile(a, 95)),
            'max_ms': float(a.max()), 'n': int(a.size)}


def icp_latency(log_text: str, skip_first: int = 1) -> Dict[str, float]:
    """icp_odometry 스캔당 처리 시간 (첫 줄 = 기준 없는 첫 장은 뺀다)."""
    return _q([float(m.group(1)) for m in _ICP.finditer(log_text)][skip_first:])


def rtabmap_latency(log_text: str) -> Dict[str, float]:
    """갱신(1 Hz) 당 rtabmap 처리 시간."""
    return _q([float(m.group(1)) for m in _RTAB.finditer(log_text)])


def region_raster(xy: np.ndarray, radius: float = 10.0, cell: float = 0.5,
                  pad: Optional[float] = None) -> Dict[str, object]:
    """GT 궤적 xy 에서 반경 radius 안 셀 (셀 중심 기준) 래스터."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    pad = radius + cell if pad is None else pad
    x0, y0 = float(xy[:, 0].min() - pad), float(xy[:, 1].min() - pad)
    nx = int(math.ceil((xy[:, 0].max() + pad - x0) / cell))
    ny = int(math.ceil((xy[:, 1].max() + pad - y0) / cell))
    cx = x0 + (np.arange(nx) + 0.5) * cell
    cy = y0 + (np.arange(ny) + 0.5) * cell
    gx, gy = np.meshgrid(cx, cy)
    centres = np.column_stack([gx.ravel(), gy.ravel()])
    # 궤적을 cell/2 간격으로 촘촘히(선분 보간) 한 뒤 솎고 최근접 거리
    dense = [xy[:1]]
    for a, b in zip(xy[:-1], xy[1:]):
        n = int(math.ceil(np.linalg.norm(b - a) / (cell / 2)))
        if n > 1:
            dense.append(a + (b - a) * (np.arange(1, n + 1) / n)[:, None])
        else:
            dense.append(b[None])
    xy = np.concatenate(dense)
    keep = [0]
    for i in range(1, len(xy)):
        if np.linalg.norm(xy[i] - xy[keep[-1]]) >= cell / 2:
            keep.append(i)
    pts = xy[keep]
    best = np.full(len(centres), np.inf)
    for k in range(0, len(pts), 256):
        d = np.linalg.norm(centres[:, None, :] - pts[None, k:k + 256, :], axis=2).min(axis=1)
        best = np.minimum(best, d)
    return {'mask': (best <= radius).reshape(ny, nx), 'x0': x0, 'y0': y0, 'cell': cell,
            'radius': radius}


def in_region(region: Dict[str, object], xyz: np.ndarray) -> np.ndarray:
    """점 (N, ≥2) 이 영역 래스터 안인가."""
    p = np.asarray(xyz, dtype=float)
    mask = region['mask']
    ix = np.floor((p[:, 0] - region['x0']) / region['cell']).astype(np.int64)
    iy = np.floor((p[:, 1] - region['y0']) / region['cell']).astype(np.int64)
    ok = (ix >= 0) & (ix < mask.shape[1]) & (iy >= 0) & (iy < mask.shape[0])
    out = np.zeros(len(p), dtype=bool)
    out[ok] = mask[iy[ok], ix[ok]]
    return out


def coverage(traj_t: np.ndarray, gt_t: np.ndarray) -> float:
    """추정 궤적이 GT 구간을 덮는 비율 [0, 1] (첫·끝 스탬프 기준)."""
    if len(traj_t) == 0 or len(gt_t) < 2:
        return 0.0
    span = float(gt_t[-1] - gt_t[0])
    if span <= 0:
        return 0.0
    lo, hi = max(float(traj_t[0]), float(gt_t[0])), min(float(traj_t[-1]), float(gt_t[-1]))
    return max(0.0, hi - lo) / span
