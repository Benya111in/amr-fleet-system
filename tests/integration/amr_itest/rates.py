"""
토픽 주기 통계 (rclpy 비의존 순수 모듈).

센서 주기는 메시지 header.stamp(시뮬레이션이면 sim time)로 잰다. 호스트 부하로 Gazebo RTF 가
1 보다 작으면 수신(wall) 주기는 규정보다 낮게 나오지만 센서가 발행하는 sim time 주기는 그대로이기
때문이다 (명세의 "10 Hz 이상" 은 시뮬레이션 시간 기준 센서 주기). wall 주기는 참고로 함께 남긴다.

판정 두 가지
  mean_rate   = (n − 1) / (마지막 스탬프 − 첫 스탬프)   — 실제 전달률 (드롭 포함)
  median_rate = 1 / median(연속 스탬프 간격)            — 설정된 센서 주기 (간헐 드롭에 강건)
"""

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence

import numpy as np


@dataclass(frozen=True)
class RateStats:
    """스탬프 열의 주기 통계."""

    count: int
    span: float = 0.0            # [s] 첫~마지막 스탬프
    mean_rate: float = 0.0       # [Hz]
    median_rate: float = 0.0     # [Hz]
    max_gap: float = 0.0         # [s] 가장 긴 간격
    duplicates: int = 0          # 역행/중복 스탬프 수 (버림)

    def meets(self, nominal: float, mean_tol: float = 0.9, median_tol: float = 0.95) -> bool:
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
