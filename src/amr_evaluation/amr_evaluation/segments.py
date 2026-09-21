"""
구간 분류 (rclpy 비의존 순수 모듈).

- 로봇 운동 상태: 정지 / 직선 / 회전 (명세 4.3 위치 추정 오차의 3 구간)
- 경로 기하: 직선 / 곡선 (명세 4.5 CTE 의 2 구간) — 국소 곡률 기준
"""

from dataclasses import dataclass
import math
from typing import List, Sequence

import numpy as np

# 위치 추정 구간 라벨 (명세 표기 그대로)
STOP = '정지'
STRAIGHT = '직선'
TURN = '회전'
MOTION_SEGMENTS = (STOP, STRAIGHT, TURN)

# 경로 추종 구간 라벨
CURVE = '곡선'
PATH_SEGMENTS = (STRAIGHT, CURVE)


@dataclass(frozen=True)
class MotionThresholds:
    """운동 상태 분류 임계값."""

    stop_linear: float = 0.02      # [m/s] 이 미만이면 정지 후보
    stop_angular: float = 0.02     # [rad/s] 이 미만이면 정지 후보
    straight_angular: float = 0.05  # [rad/s] 이 미만이면 직선, 이상이면 회전


def classify_motion(v: float, w: float, thresholds: MotionThresholds = MotionThresholds()) -> str:
    """
    GT 속도로 운동 상태를 분류한다.

    정지: |v| < stop_linear 이고 |w| < stop_angular
    직선: (정지가 아니고) |w| < straight_angular
    회전: 그 외
    """
    av, aw = abs(v), abs(w)
    if av < thresholds.stop_linear and aw < thresholds.stop_angular:
        return STOP
    if aw < thresholds.straight_angular:
        return STRAIGHT
    return TURN


def cumulative_length(points: np.ndarray) -> np.ndarray:
    """정점별 누적 호길이 [m]. points 는 (N, 2)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return np.zeros(0)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def menger_curvature(p_prev: Sequence[float], p: Sequence[float],
                     p_next: Sequence[float]) -> float:
    """
    세 점을 지나는 원의 부호 있는 곡률 [1/m] (좌회전 +).

    κ = 2·cross(b − a, c − a) / (|b − a| |c − b| |c − a|). 세 점 중 둘이 겹치면 0.
    """
    a = np.asarray(p_prev, dtype=float)
    b = np.asarray(p, dtype=float)
    c = np.asarray(p_next, dtype=float)
    ab, bc, ac = b - a, c - b, c - a
    denom = np.linalg.norm(ab) * np.linalg.norm(bc) * np.linalg.norm(ac)
    if denom < 1e-12:
        return 0.0
    cross = ab[0] * ac[1] - ab[1] * ac[0]
    return float(2.0 * cross / denom)


def path_curvature(points: np.ndarray, window: float = 0.5) -> np.ndarray:
    """
    각 정점의 국소 곡률 [1/m].

    5 cm 간격 경로에서 인접 3 점은 노이즈가 커서, 정점 앞뒤로 호길이 window/2 이상 떨어진
    정점을 골라 3 점 원적합(Menger)한다. 양 끝은 한쪽 이웃만 있으므로 가능한 만큼만 쓴다.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    n = len(pts)
    kappa = np.zeros(n)
    if n < 3:
        return kappa
    s = cumulative_length(pts)
    half = max(window / 2.0, 0.0)
    for i in range(1, n - 1):
        # 뒤로 half 이상 떨어진 마지막 정점, 앞으로 half 이상 떨어진 첫 정점 (없으면 바로 이웃)
        j = int(np.searchsorted(s, s[i] - half, side='right')) - 1
        k = int(np.searchsorted(s, s[i] + half, side='left'))
        j = min(max(j, 0), i - 1)
        k = max(min(k, n - 1), i + 1)
        kappa[i] = menger_curvature(pts[j], pts[i], pts[k])
    # 양 끝점은 이웃이 한쪽뿐이라 원적합이 안 된다 → 인접 내부 정점 값을 그대로 쓴다
    kappa[0] = kappa[1]
    kappa[n - 1] = kappa[n - 2]
    return kappa


def dilate_curve_labels(labels: List[str], cum_len: np.ndarray, margin: float) -> List[str]:
    """곡선 구간 앞뒤 호길이 margin 만큼을 곡선으로 넓힌다 (천이대를 보수적으로 곡선에 계상)."""
    if margin <= 0.0 or not labels:
        return list(labels)
    out = list(labels)
    curve_s = [cum_len[i] for i, lab in enumerate(labels) if lab == CURVE]
    if not curve_s:
        return out
    curve_s = np.asarray(curve_s)
    for i, s in enumerate(cum_len):
        if out[i] != CURVE and np.min(np.abs(curve_s - s)) <= margin:
            out[i] = CURVE
    return out


def label_path_segments(points: np.ndarray, curvature_threshold: float = 0.1,
                        window: float = 0.5, margin: float = 0.5) -> List[str]:
    """
    경로 정점별 직선/곡선 라벨.

    |κ| > curvature_threshold (기본 0.1 1/m, 곡률 반경 10 m 미만) 이면 곡선.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return []
    kappa = path_curvature(pts, window)
    labels = [CURVE if abs(k) > curvature_threshold else STRAIGHT for k in kappa]
    return dilate_curve_labels(labels, cumulative_length(pts), margin)


def segment_label_for_index(labels: Sequence[str], seg_index: int) -> str:
    """선분 seg_index (정점 i → i+1) 의 라벨: 양 끝 정점 중 하나라도 곡선이면 곡선."""
    if not labels:
        return STRAIGHT
    i = min(max(seg_index, 0), len(labels) - 1)
    j = min(i + 1, len(labels) - 1)
    return CURVE if CURVE in (labels[i], labels[j]) else STRAIGHT


def yaw_rate_from_headings(yaw_a: float, yaw_b: float, dt: float) -> float:
    """두 헤딩 사이의 각속도 [rad/s] (래핑 처리). dt ≤ 0 이면 0."""
    if dt <= 0.0:
        return 0.0
    d = math.atan2(math.sin(yaw_b - yaw_a), math.cos(yaw_b - yaw_a))
    return d / dt
