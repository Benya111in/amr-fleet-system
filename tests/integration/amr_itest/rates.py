"""
토픽 주기 통계 (rclpy 비의존 순수 모듈).

센서 주기는 메시지 header.stamp(시뮬레이션이면 sim time)로 잰다. 호스트 부하로 Gazebo RTF 가
1 보다 작으면 수신(wall) 주기는 규정보다 낮게 나오지만 센서가 발행하는 sim time 주기는 그대로이기
때문이다 (명세의 "10 Hz 이상" 은 시뮬레이션 시간 기준 센서 주기). wall 주기는 참고로 함께 남긴다.

판정 두 가지
  median_rate = 1 / median(연속 스탬프 간격)            — 센서 주기 (간헐 드롭에 강건). 명세 "N Hz 이상" 을
                                                      이것으로 본다: 지터만 허용(≥ 0.98×)
  mean_rate   = (n − 1) / (마지막 스탬프 − 첫 스탬프)   — 구독자 쪽 전달률 (전송 드롭 포함, ≥ 0.9×)
규정 주기는 명세 하한과 설정(sensors.yaml) 중 큰 값이다 (nominal_rate) — 설정을 낮춰 기준이 내려가지 않게.
"""

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence

import numpy as np


MEDIAN_TOL = 0.98     # 센서 주기: 스탬프 양자화·지터만 허용
MEAN_TOL = 0.9        # 전달률: 구독자 쪽 드롭 허용


def nominal_rate(config_hz: float, spec_min_hz: float = 0.0) -> float:
    """판정 기준 주기 [Hz] = max(설정, 명세 하한). 설정만 낮춰서 기준이 내려가지 않게 한다."""
    return max(float(config_hz), float(spec_min_hz))


@dataclass(frozen=True)
class RateStats:
    """스탬프 열의 주기 통계."""

    count: int
    span: float = 0.0            # [s] 첫~마지막 스탬프
    mean_rate: float = 0.0       # [Hz]
    median_rate: float = 0.0     # [Hz]
    max_gap: float = 0.0         # [s] 가장 긴 간격
    duplicates: int = 0          # 역행/중복 스탬프 수 (버림)

    def meets(self, nominal: float, mean_tol: float = MEAN_TOL,
              median_tol: float = MEDIAN_TOL) -> bool:
        """규정 주기 nominal [Hz] 에 대해 두 판정을 모두 통과하면 True."""
        if self.count < 2 or nominal <= 0.0:
            return False
        return (self.mean_rate >= mean_tol * nominal
                and self.median_rate >= median_tol * nominal)

    def as_dict(self) -> dict:
        return {'count': self.count, 'span_s': self.span, 'mean_rate_hz': self.mean_rate,
                'median_rate_hz': self.median_rate, 'max_gap_s': self.max_gap,
                'dropped_non_increasing': self.duplicates}


def rate_stats(stamps: Iterable[float]) -> RateStats:
    """
    스탬프 [s] 열 → RateStats.

    수신 순서를 유지하고, 직전보다 크지 않은 스탬프(역행·중복)는 버리고 개수만 센다.
    """
    kept: List[float] = []
    dup = 0
    for t in stamps:
        if not math.isfinite(t):
            dup += 1
            continue
        if kept and t <= kept[-1]:
            dup += 1
            continue
        kept.append(float(t))
    n = len(kept)
    if n < 2:
        return RateStats(count=n, duplicates=dup)
    arr = np.asarray(kept)
    periods = np.diff(arr)
    span = float(arr[-1] - arr[0])
    med = float(np.median(periods))
    return RateStats(
        count=n,
        span=span,
        mean_rate=(n - 1) / span if span > 0.0 else 0.0,
        median_rate=1.0 / med if med > 0.0 else 0.0,
        max_gap=float(np.max(periods)),
        duplicates=dup,
    )


def in_window(stamps: Sequence[float], t0: float, t1: float) -> List[float]:
    """[t0, t1] 안의 스탬프만."""
    return [t for t in stamps if t0 <= t <= t1]


def sample_std(values: Sequence[float]) -> float:
    """표본 표준편차 (n < 2 이면 NaN)."""
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size < 2:
        return float('nan')
    return float(np.std(arr, ddof=1))


def per_pixel_noise(frames: Sequence[np.ndarray], range_max: float) -> float:
    """
    정지 상태 깊이 영상 여러 장에서 화소별 표준편차의 중앙값 [m] (per_beam_noise 의 영상판).

    모든 장에서 유한하고 (0, range_max) 안인 화소만 쓴다. 쓸 화소가 없으면 NaN.
    """
    if len(frames) < 2:
        return float('nan')
    arr = np.stack([np.asarray(f, dtype=float).reshape(-1) for f in frames])
    valid = np.all(np.isfinite(arr) & (arr > 0.0) & (arr < range_max * 0.999), axis=0)
    if not np.any(valid):
        return float('nan')
    return float(np.median(np.std(arr[:, valid], axis=0, ddof=1)))


def finite_max(frames: Sequence[np.ndarray]) -> float:
    """깊이 영상들의 유한 양수 값 최댓값 [m] (없으면 NaN) — 최대 측정 거리 판정."""
    best = float('nan')
    for f in frames:
        a = np.asarray(f, dtype=float)
        a = a[np.isfinite(a) & (a > 0.0)]
        if a.size:
            best = float(a.max()) if not math.isfinite(best) else max(best, float(a.max()))
    return best


def per_beam_noise(scans: Sequence[Sequence[float]], range_max: float) -> float:
    """
    정지 상태 LaserScan 여러 장에서 빔별 표준편차의 중앙값 [m].

    모든 스캔에서 유한하고 range_max 미만인 빔만 쓴다 (미검출 빔 제외). 정지한 로봇이
    정적 환경을 볼 때 이 값이 곧 거리 노이즈 σ 의 추정치다. 쓸 빔이 없으면 NaN.
    """
    if len(scans) < 2:
        return float('nan')
    arr = np.asarray([list(s) for s in scans], dtype=float)
    valid = np.all(np.isfinite(arr) & (arr < range_max * 0.999) & (arr > 0.0), axis=0)
    if not np.any(valid):
        return float('nan')
    stds = np.std(arr[:, valid], axis=0, ddof=1)
    return float(np.median(stds))
