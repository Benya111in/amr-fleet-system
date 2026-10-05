"""
시계 영역 판별과 실시간 계수(RTF) 추정 (rclpy 비의존 순수 모듈).

ROS 스탬프는 노드의 use_sim_time 에 따라 시뮬 시간(/clock, 0 부터) 또는 벽시계 에포크(1970 기준)다.
시스템 일부만 use_sim_time 이면 두 영역의 스탬프가 섞여 짝짓기가 조용히 실패하므로, 로거와 analyze 는
스탬프 크기로 영역을 판별해 섞임을 경고한다. 시뮬 시간이 1e9 s(약 31.7 년)를 넘는 일은 없다.
"""

from collections import deque
import math
from typing import Deque, Iterable, List, Optional, Tuple

SIM = 'sim'
WALL = 'wall'
WALL_EPOCH_MIN = 1.0e9    # [s] 2001-09-09. 이 이상이면 벽시계 에포크 스탬프로 본다


def clock_domain(t: float) -> str:
    """스탬프 [s] → 'wall' (에포크) 또는 'sim' (시뮬 시간)."""
    return WALL if t >= WALL_EPOCH_MIN else SIM


def expected_domain(use_sim_time: bool) -> str:
    """노드의 use_sim_time → 그 노드가 찍는 스탬프의 영역."""
    return SIM if use_sim_time else WALL


def domains_of(values: Iterable[float]) -> List[str]:
    """유한한 양수 스탬프들의 영역 목록 (중복 제거, 정렬). 0/NaN 은 무시한다."""
    out = set()
    for v in values:
        try:
            f = float(v)
        except (TypeError, ValueError):
            continue
        if math.isfinite(f) and f > 0.0:
            out.add(clock_domain(f))
    return sorted(out)


class RtfEstimator:
    """
    실시간 계수 RTF = Δ(ROS 시각) / Δ(단조 벽시계) 를 최근 window [s] 로 추정한다.

    use_sim_time 이면 ROS 시각이 /clock 이라 Gazebo 가 실시간보다 느리면 RTF < 1 이고, 시뮬 시간으로
    잰 지연 d 의 벽시계 지연은 d / RTF 다 (multi_robot.md §7: 응답 시간 KPI 는 실시간 기준).
    use_sim_time 이 아니면 1 에 가깝다. 표본이 min_span [s] 보다 짧게 쌓였으면 NaN.
    ROS 시각이 거꾸로 가면(시뮬 리셋) 다시 시작한다.
    """

    def __init__(self, window: float = 5.0, min_span: float = 1.0):
        self.window = window
        self.min_span = min_span
        self._buf: Deque[Tuple[float, float]] = deque()

    def add(self, ros_t: float, wall_t: float) -> None:
        if self._buf and (ros_t < self._buf[-1][0] or wall_t < self._buf[-1][1]):
            self._buf.clear()
        self._buf.append((ros_t, wall_t))
        while len(self._buf) > 2 and wall_t - self._buf[1][1] >= self.window:
            self._buf.popleft()

    def value(self) -> float:
        if len(self._buf) < 2:
            return float('nan')
        (r0, w0), (r1, w1) = self._buf[0], self._buf[-1]
        if w1 - w0 < self.min_span:
            return float('nan')
        return (r1 - r0) / (w1 - w0)


def wall_latency_ms(latency_ms: float, rtf: Optional[float]) -> float:
    """시뮬 시간 지연 → 벽시계 환산 지연 [ms]. RTF 가 없거나 0 이하면 NaN."""
    if rtf is None or not math.isfinite(rtf) or rtf <= 0.0:
        return float('nan')
    return latency_ms / rtf
