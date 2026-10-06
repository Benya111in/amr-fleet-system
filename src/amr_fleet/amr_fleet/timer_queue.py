"""
rclpy 타이머 1개로 DelayQueue 를 해제 시각에 맞춰 비우는 도우미.

fleet ↔ 로봇 송신 지연 큐(multi_robot.md §6)와 어댑터의 모의 완료에 쓴다.
설계 문서의 "5 ms 폴링 타이머" 대신 다음 해제 시각에 맞춰 주기를 다시 거는 재무장 타이머를
쓴다: 해제 오차가 폴링 간격(최대 5 ms)이 아니라 타이머 지터 수준이고, 큐가 비면 깨어나지 않는다.
시각은 노드 시계(use_sim_time 이면 /clock) 기준이다.

EmulatedInbox 는 **수신 측** 통신 모의다. 방송 토픽(로봇↔로봇 비컨 · 투표처럼 한 번 내고 여럿이 받는 것)은
송신 측에서 늦추면 모든 수신자가 같은 지연을 받으므로, 수신자마다 링크 (송신자 → 나) 의 CommEmulator
결과로 늦추거나 버린다 (docs/research/fleet-distributed/p0.md §1).
"""

from __future__ import annotations

from typing import Any, Callable, Hashable, Optional

from amr_fleet.comm_emulator import CommEmulator
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


class EmulatedInbox:
    """
    수신 측 통신 모의: receive(src, item) → 링크 (src → receiver) 의 지연 뒤 on_deliver(item).

    유실이면 버리고 None 을, 아니면 해제 시각을 돌려준다. 같은 송신자의 항목끼리는 FIFO 다 (추월 없음,
    DDS reliable 과 같다). 자기 자신이 보낸 항목은 모의 없이 바로 넘긴다 (자기 루프는 네트워크를 타지 않는다).
    """

    def __init__(self, node: Node, emulator: CommEmulator, receiver: str,
                 on_deliver: Callable[[Any], None]):
        self._node = node
        self._emulator = emulator
        self.receiver = receiver
        self._on_deliver = on_deliver
        self._queue = TimerQueue(node, on_deliver)
        self.dropped = 0

    def __len__(self) -> int:
        return len(self._queue)

    def receive(self, src: str, item: Any) -> Optional[float]:
        """받은 항목을 모의에 넣는다. 유실이면 None."""
        now = self._queue.now()
        if src == self.receiver:
            self._on_deliver(item)
            return now
        delay = self._emulator.transmit(src, self.receiver, now)
        if delay is None:
            self.dropped += 1
            return None
        if not self._emulator.enabled(src, self.receiver):
            self._on_deliver(item)
            return now
        return self._queue.push(item, delay, key=src)

    def clear(self) -> None:
        """대기 항목을 모두 버린다."""
        self._queue.clear()
