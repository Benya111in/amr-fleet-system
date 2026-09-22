"""
구간 분류 (rclpy 비의존 순수 모듈).

- 로봇 운동 상태: 정지 / 직선 / 회전 (명세 4.3 위치 추정 오차의 3 구간)
- 경로 기하: 직선 / 곡선 (명세 4.5 CTE 의 2 구간) — 평활화한 경로의 국소 곡률 기준
"""

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple

import numpy as np

# 위치 추정 구간 라벨 (명세 표기 그대로)
STOP = '정지'
STRAIGHT = '직선'
TURN = '회전'
MOTION_SEGMENTS = (STOP, STRAIGHT, TURN)

# 경로 추종 구간 라벨
CURVE = '곡선'
PATH_SEGMENTS = (STRAIGHT, CURVE)

# 곡률 추정 기본값 (cte_logger 파라미터 기본값과 같다)
DEFAULT_CURVATURE_WINDOW = 1.0      # [m] 앞뒤 현(chord) 길이의 합
DEFAULT_SMOOTHING = 0.2             # [m] 가우시안 평활 σ (호길이)
DEFAULT_MIN_TURN = math.radians(10.0)   # [rad] 곡선 구간 하나의 최소 누적 회전각
RESAMPLE_STEP = 0.05                # [m] 곡률 계산용 등간격 재표본 간격


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
    회전: |v| < stop_linear 인데 |w| ≥ stop_angular (제자리 회전), 또는 |w| ≥ straight_angular
    직선: |v| ≥ stop_linear 이고 |w| < straight_angular
    """
    av, aw = abs(v), abs(w)
    if av < thresholds.stop_linear:
        return STOP if aw < thresholds.stop_angular else TURN
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


def resample_path(points: np.ndarray, step: float = RESAMPLE_STEP
                  ) -> Tuple[np.ndarray, np.ndarray]:
    """폴리라인을 호길이 등간격 step 으로 다시 표본한다 → (점 (M, 2), 호길이 (M,)). 중복 정점 무시."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    s = cumulative_length(pts)
    if len(pts) < 2 or s[-1] <= 1e-9:
        return pts.copy(), s
    s_u, idx = np.unique(s, return_index=True)      # 길이 0 선분(중복 정점) 제거
    n = max(int(round(s_u[-1] / step)), 1)
    su = np.linspace(0.0, s_u[-1], n + 1)
    out = np.column_stack([np.interp(su, s_u, pts[idx, 0]), np.interp(su, s_u, pts[idx, 1])])
    return out, su


def smooth_path(points: np.ndarray, sigma_samples: float) -> np.ndarray:
    """
    등간격 점열을 가우시안(σ = sigma_samples 표본)으로 평활한다 (길이 유지, 'same').

    양 끝 3σ 는 끝점 기준 점대칭(2·p0 − p_k)으로 늘여 붙여 계산한다: 직선은 그대로 보존된다.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    n = len(pts)
    if sigma_samples <= 0.0 or n < 3:
        return pts.copy()
    r = int(math.ceil(3.0 * sigma_samples))
    kernel = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_samples) ** 2)
    kernel /= kernel.sum()
    k = np.arange(1, r + 1)
    head = 2.0 * pts[0] - pts[np.minimum(k, n - 1)][::-1]
    tail = 2.0 * pts[-1] - pts[np.maximum(n - 1 - k, 0)]
    padded = np.vstack([head, pts, tail])
    return np.column_stack([np.convolve(padded[:, 0], kernel, mode='valid'),
                            np.convolve(padded[:, 1], kernel, mode='valid')])


def _chord_curvature(pts: np.ndarray, h: int) -> np.ndarray:
    """앞뒤 h 표본 떨어진 현(chord) 의 방향 차 / 평균 현 길이 = 창 평균 곡률 [1/m] (좌회전 +)."""
    n = len(pts)
    i = np.arange(n)
    j = np.maximum(i - h, 0)
    k = np.minimum(i + h, n - 1)
    back = pts - pts[j]
    fwd = pts[k] - pts
    t1 = np.arctan2(back[:, 1], back[:, 0])
    t2 = np.arctan2(fwd[:, 1], fwd[:, 0])
    dth = np.arctan2(np.sin(t2 - t1), np.cos(t2 - t1))
    length = 0.5 * (np.linalg.norm(back, axis=1) + np.linalg.norm(fwd, axis=1))
    ok = (i - j >= 1) & (k - i >= 1) & (length > 1e-9)
    return np.where(ok, dth / np.where(ok, length, 1.0), 0.0)


def _end_line(pts: np.ndarray, s: np.ndarray, length: float, at_start: bool
              ) -> Tuple[np.ndarray, np.ndarray]:
    """경로 끝 length [m] 구간의 최소제곱 직선 → (끝점을 그 직선에 사영한 점, 진행 방향 단위벡터)."""
    sel = pts[s <= s[0] + length] if at_start else pts[s >= s[-1] - length]
    if len(sel) < 2:
        sel = pts[:2] if at_start else pts[-2:]
    c = sel.mean(axis=0)
    d = np.linalg.svd(sel - c)[2][0]
    if np.dot(d, sel[-1] - sel[0]) < 0.0:
        d = -d
    end = pts[0] if at_start else pts[-1]
    return c + np.dot(end - c, d) * d, d


def curvature_profile(points: np.ndarray, window: float = DEFAULT_CURVATURE_WINDOW,
                      smoothing: float = DEFAULT_SMOOTHING, step: float = RESAMPLE_STEP
                      ) -> Tuple[np.ndarray, np.ndarray]:
    """
    재표본 경로 위의 곡률 → (호길이 (M,), 곡률 (M,)) [1/m] (좌회전 +).

    1) 호길이 step 간격으로 재표본 (격자 경로의 중복·불균등 정점 정리)
    2) 양 끝을 끝 window/2 구간의 최소제곱 직선 방향으로 (window/2 + 3σ) 만큼 늘인다 — 끝점 하나의
       격자 오차가 꺾임이 되지 않고, 끝에서도 창을 줄이지 않고 같은 창으로 계산한다
    3) 가우시안 σ = smoothing [m] 평활 — 5 cm 격자 계단·mm 지터를 없앤다
    4) 앞뒤 window/2 [m] 현의 방향 차 / 현 길이 (3 점 원적합보다 잡음에 강한 창 평균 곡률)
    경로 길이가 window 보다 짧으면 곡률을 정할 수 없어 0 (직선) 으로 둔다.
    """
    pts, s = resample_path(points, step)
    m = len(pts)
    if m < 3 or s[-1] < window:
        return s, np.zeros(m)
    sigma_n = max(smoothing, 0.0) / step
    rad = int(math.ceil(3.0 * sigma_n)) if sigma_n > 0.0 else 0
    h = max(int(round(max(window, 0.0) / 2.0 / step)), 1)
    edge = h + rad
    a0, d0 = _end_line(pts, s, window / 2.0, at_start=True)
    a1, d1 = _end_line(pts, s, window / 2.0, at_start=False)
    head = a0 - d0 * step * np.arange(edge, 0, -1)[:, None]
    tail = a1 + d1 * step * np.arange(1, edge + 1)[:, None]
    extended = np.vstack([head, pts, tail])
    kappa = _chord_curvature(smooth_path(extended, sigma_n), h)
    return s, kappa[edge:edge + m]


def path_curvature(points: np.ndarray, window: float = DEFAULT_CURVATURE_WINDOW,
                   smoothing: float = DEFAULT_SMOOTHING) -> np.ndarray:
    """원래 정점별 곡률 [1/m] (curvature_profile 을 정점 호길이로 보간)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 3:
        return np.zeros(len(pts))
    s_r, k_r = curvature_profile(pts, window, smoothing)
    return np.interp(cumulative_length(pts), s_r, k_r)


def _curve_mask(s: np.ndarray, kappa: np.ndarray, threshold: float, min_turn: float) -> np.ndarray:
    """
    |κ| > threshold 인 같은 부호 연속 구간 중 누적 회전각 ∫|κ|ds ≥ min_turn 인 것만 곡선.

    히스테리시스: 평활 뒤에도 남는 격자 계단 한 칸의 요철(회전각 수 도)은 곡선으로 세지 않는다.
    """
    over = np.abs(kappa) > threshold
    mask = np.zeros(len(kappa), dtype=bool)
    turn = np.abs(kappa) * (np.gradient(s) if len(s) > 1 else np.zeros(len(s)))
    i = 0
    n = len(over)
    while i < n:
        if not over[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and over[j + 1] and np.sign(kappa[j + 1]) == np.sign(kappa[i]):
            j += 1
        if float(np.sum(turn[i:j + 1])) >= min_turn:
            mask[i:j + 1] = True
        i = j + 1
    return mask


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
                        window: float = DEFAULT_CURVATURE_WINDOW, margin: float = 0.5,
                        smoothing: float = DEFAULT_SMOOTHING,
                        min_turn: float = DEFAULT_MIN_TURN) -> List[str]:
    """
    경로 정점별 직선/곡선 라벨.

    평활 곡률 |κ| > curvature_threshold (기본 0.1 1/m, 곡률 반경 10 m 미만) 인 연속 구간 가운데
    누적 회전각이 min_turn (기본 10°) 이상인 것을 곡선으로 보고, 그 앞뒤 margin 을 곡선에 더한다.
    min_turn 은 격자 계단의 한 칸 요철(회전각 수 도)을 곡선으로 세지 않기 위한 히스테리시스다.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return []
    if len(pts) < 3:
        return [STRAIGHT] * len(pts)
    s_r, k_r = curvature_profile(pts, window, smoothing)
    mask_r = _curve_mask(s_r, k_r, curvature_threshold, min_turn)
    cum = cumulative_length(pts)
    # 원래 정점 → 가장 가까운 재표본 점의 판정
    idx = np.clip(np.searchsorted(s_r, cum), 0, len(s_r) - 1)
    prev = np.clip(idx - 1, 0, len(s_r) - 1)
    nearest = np.where(np.abs(s_r[prev] - cum) < np.abs(s_r[idx] - cum), prev, idx)
    labels = [CURVE if mask_r[i] else STRAIGHT for i in nearest]
    return dilate_curve_labels(labels, cum, margin)


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
