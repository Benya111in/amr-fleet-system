"""
시스템 KPI 집계 (명세 4.9 모니터링: 작업 처리량, 평균 작업 시간, 로봇 가동률 + 교착 횟수).

amr_msgs/msg/FleetStatus 의 스칼라 필드를 모두 여기서 계산한다.
- throughput        : 최근 throughput_window_s 동안 완료 수 / 창 길이 [tasks/h]
                      (가동 시간이 창보다 짧으면 가동 시간으로 나눈다)
- avg_task_duration : 완료 작업의 (종료 - 시작) 평균 [s]
- robot_utilization : Σ busy 시간 / Σ 관측 시간 (로봇별 첫 관측부터), 0~1. busy 정의는 모드로 고른다
    assigned (기본) : 진행 중 작업이 있는 시간 = 작업 로그 duration 의 합과 같은 구간
                      (연구 브리프 §6.1 U^assigned — 교통 대기·E-stop 으로 멈춘 시간도 가동에 포함)
    moving          : RobotState.status 가 MOVING/DOCKING/LOADING 인 시간 (U^moving)
- deadlock_count    : /fleet/traffic_events 의 교착 탐지 이벤트 누적
"""

from __future__ import annotations

import collections
from typing import Deque, Dict, Iterable, Optional, Tuple

UTILIZATION_MODES = ('assigned', 'moving')

# amr_msgs/msg/RobotState.STATUS_* (ROS 없이 쓰도록 복제)
STATUS_IDLE = 0
STATUS_MOVING = 1
STATUS_DOCKING = 2
STATUS_LOADING = 3
STATUS_CHARGING = 4
STATUS_ERROR = 5
STATUS_ESTOP = 6
BUSY_STATUSES = frozenset({STATUS_MOVING, STATUS_DOCKING, STATUS_LOADING})


class KpiTracker:
    """완료/실패/로봇 상태 이벤트를 받아 FleetStatus 지표를 만든다."""

    def __init__(self, throughput_window_s: float = 1800.0,
                 busy_statuses: Iterable[int] = BUSY_STATUSES, min_window_s: float = 1.0,
                 utilization_mode: str = 'assigned'):
        if throughput_window_s <= 0.0:
            raise ValueError('throughput_window_s 는 양수여야 한다')
        if utilization_mode not in UTILIZATION_MODES:
            raise ValueError(f'utilization_mode ∉ {UTILIZATION_MODES}: {utilization_mode!r}')
        self.window_s = float(throughput_window_s)
        self.min_window_s = float(min_window_s)
        self.busy_statuses = frozenset(busy_statuses)
        self.utilization_mode = utilization_mode
        self.start_time: Optional[float] = None
        self._completions: Deque[float] = collections.deque()
        self.completed = 0
        self.failed = 0
        self._duration_sum = 0.0
        self.deadlock_count = 0
        # robot_id → (첫 관측 시각, 마지막 갱신 시각, 지금 busy 인지, 누적 busy 초)
        self._robots: Dict[str, Tuple[float, float, bool, float]] = {}

    # --- 이벤트 입력 ---
    def start(self, now: float) -> None:
        """집계 시작 시각 (처음 한 번만 유효)."""
        if self.start_time is None:
            self.start_time = now

    def record_completion(self, now: float, duration_s: float) -> None:
        """작업 완료 1건."""
        self.start(now)
        self.completed += 1
        self._duration_sum += max(0.0, duration_s)
        self._completions.append(now)
        self._trim(now)

    def record_failure(self, now: float) -> None:
        """작업 실패 1건."""
        self.start(now)
        self.failed += 1

    def record_deadlock(self, n: int = 1) -> None:
        """교착 탐지 이벤트."""
        self.deadlock_count += max(0, n)

    def is_busy(self, status: int, assigned: bool) -> bool:
        """모드에 따른 busy 판정 (assigned: 진행 중 작업 유무, moving: 상태 코드)."""
        if self.utilization_mode == 'assigned':
            return bool(assigned)
        return status in self.busy_statuses

    def update_robot(self, robot_id: str, status: int, now: float, assigned: bool = False) -> None:
        """
        로봇 상태 갱신. 직전 판정이 busy 였다면 그 구간을 busy 시간에 더한다.

        assigned = 그 로봇에 진행 중 작업이 있는지. robot_state 수신과 작업 시작/종료 때 부른다.
        """
        self.start(now)
        busy_now = self.is_busy(status, assigned)
        entry = self._robots.get(robot_id)
        if entry is None:
            self._robots[robot_id] = (now, now, busy_now, 0.0)
            return
        first, last, was_busy, busy = entry
        if now > last and was_busy:
            busy += now - last
        self._robots[robot_id] = (first, max(last, now), busy_now, busy)

    # --- 지표 ---
    def throughput(self, now: float) -> float:
        """[tasks/h] 슬라이딩 창 처리량."""
        self._trim(now)
        if self.start_time is None:
            return 0.0
        elapsed = max(now - self.start_time, self.min_window_s)
        window = min(self.window_s, elapsed)
        return len(self._completions) / window * 3600.0

    def avg_task_duration(self) -> float:
        """[s] 완료 작업 평균 소요 시간."""
        return self._duration_sum / self.completed if self.completed else 0.0

    def robot_utilization(self, now: float) -> float:
        """0~1. 관측된 로봇이 없으면 0."""
        busy_total = observed_total = 0.0
        for first, last, busy_now, busy in self._robots.values():
            if now > last and busy_now:
                busy += now - last
            busy_total += busy
            observed_total += max(0.0, now - first)
        return busy_total / observed_total if observed_total > 0.0 else 0.0

    def robot_busy_time(self, robot_id: str, now: float) -> float:
        """로봇 1대의 누적 busy 초 (현재 상태 포함)."""
        entry = self._robots.get(robot_id)
        if entry is None:
            return 0.0
        first, last, busy_now, busy = entry
        if now > last and busy_now:
            busy += now - last
        return busy

    def snapshot(self, now: float, counts: Optional[Dict[str, int]] = None) -> Dict[str, float]:
        """`FleetStatus` 스칼라 필드 dict. counts 는 상태 머신의 counts()."""
        counts = counts or {}
        return {
            'tasks_pending': int(counts.get('pending', 0)),
            'tasks_in_progress': int(counts.get('in_progress', 0)),
            'tasks_completed': int(counts.get('completed', self.completed)),
            'tasks_failed': int(counts.get('failed', self.failed)),
            'throughput': self.throughput(now),
            'avg_task_duration': self.avg_task_duration(),
            'robot_utilization': self.robot_utilization(now),
            'deadlock_count': int(self.deadlock_count),
        }

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_s
        while self._completions and self._completions[0] <= cutoff:
            self._completions.popleft()
