"""
rclpy 타이머 1개로 DelayQueue 를 해제 시각에 맞춰 비우는 도우미.

fleet ↔ 로봇 송신 지연 큐(multi_robot.md §6)와 어댑터의 모의 완료에 쓴다.
설계 문서의 "5 ms 폴링 타이머" 대신 다음 해제 시각에 맞춰 주기를 다시 거는 재무장 타이머를
쓴다: 해제 오차가 폴링 간격(최대 5 ms)이 아니라 타이머 지터 수준이고, 큐가 비면 깨어나지 않는다.
시각은 노드 시계(use_sim_time 이면 /clock) 기준이다.
"""

from __future__ import annotations

from typing import Any, Callable, Hashable, Optional

from amr_fleet.latency import DelayQueue
from rclpy.node import Node

MIN_PERIOD_S = 1.0e-4   # 이미 지난 해제 시각도 다음 스핀에서 곧바로 처리


class TimerQueue:
    """push(item, delay_s) 한 항목을 delay_s 뒤 on_release(item) 으로 넘긴다 (순서 보장)."""

    def __init__(self, node: Node, on_release: Callable[[Any], None]):
        self._node = node
        self._on_release = on_release
        self._queue = DelayQueue()
        self._timer = node.create_timer(1.0, self._on_timer)
        self._timer.cancel()
        self.released = 0

    def __len__(self) -> int:
        return len(self._queue)

    def now(self) -> float:
        """노드 시계 [s]."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def push(self, item: Any, delay_s: float, key: Optional[Hashable] = None) -> float:
        """항목을 넣고 해제 시각 [s] 을 돌려준다. key 가 같은 항목끼리는 FIFO (DelayQueue 참고)."""
        now = self.now()
        release = self._queue.push(item, now, delay_s, key)
        self._arm(now)
        return release

    def clear(self) -> None:
        """대기 항목을 모두 버린다."""
        self._queue.clear()
        self._timer.cancel()

    def _arm(self, now: float) -> None:
        nxt = self._queue.next_release()
        if nxt is None:
            self._timer.cancel()
            return
        self._timer.timer_period_ns = int(max(nxt - now, MIN_PERIOD_S) * 1e9)
        self._timer.reset()

    def _on_timer(self) -> None:
        for item in self._queue.pop_ready(self.now()):
            self.released += 1
            self._on_release(item)
        self._arm(self.now())
