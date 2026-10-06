"""
사건 큐와 링크 모의.

- EventQueue: (시각, 순번) 순서의 결정적 사건 큐. 같은 시각이면 넣은 순서.
- Network: 모든 송신이 CommEmulator.transmit(src, dst, now) 를 거친다 — 서버↔로봇, 로봇↔로봇 모두 같은 링크
  모델(보고서 189행 공정성). 링크마다 FIFO(앞 메시지보다 먼저 도착하지 않는다, 현행 EmulatedInbox 와 같은 의미).
  자기 자신에게 보내는 메시지는 모의 없이 같은 시각에 배달한다 (같은 프로세스 안).
  send_reliable 은 유실되면 retry_s 뒤 다시 보낸다 (RELIABLE/TRANSIENT_LOCAL 경로: task_board, 응답, task_status).
  송신자가 죽었으면 보내지 않고, 수신자가 배달 시각에 죽어 있으면 버린다.
"""

from __future__ import annotations

import collections
import heapq
from typing import Any, Callable, Dict, List, Tuple

from amr_fleet.comm_emulator import CommEmulator


class EventQueue:
    """결정적 사건 큐."""

    def __init__(self) -> None:
        self._heap: List[Tuple[float, int, Callable[..., None], tuple]] = []
        self._seq = 0
        self.now = 0.0                     # 지금 처리 중인 사건 시각 (호출자가 단계 시각으로도 맞춘다)

    def __len__(self) -> int:
        return len(self._heap)

    def schedule(self, t: float, fn: Callable[..., None], *args: Any) -> None:
        """시각 t 에 fn(t, *args) 를 부른다."""
        self._seq += 1
        heapq.heappush(self._heap, (float(t), self._seq, fn, args))

    def run_until(self, t_end: float) -> int:
        """시각 ≤ t_end 인 사건을 차례로 처리한다 (처리 중 새로 생긴 사건 포함). 처리한 수."""
        n = 0
        heap = self._heap
        while heap and heap[0][0] <= t_end:
            t, _, fn, args = heapq.heappop(heap)
            self.now = t
            fn(t, *args)
            n += 1
        return n

    def next_time(self) -> float:
        """다음 사건 시각 (없으면 inf)."""
        return self._heap[0][0] if self._heap else float('inf')


Handler = Callable[[float, str, str, Any], None]     # (now, src, kind, payload)


class Network:
    """링크 모의 위 메시지 전달."""

    def __init__(self, queue: EventQueue, comm: CommEmulator, alive: Callable[[str], bool],
                 retry_s: float = 0.2):
        self.q = queue
        self.comm = comm
        self.alive = alive
        self.retry_s = float(retry_s)
        self.handlers: Dict[str, Handler] = {}
        self._last: Dict[Tuple[str, str], float] = {}
        self.sent: Dict[str, int] = collections.Counter()
        self.lost: Dict[str, int] = collections.Counter()
        self.delivered: Dict[str, int] = collections.Counter()

    def register(self, name: str, handler: Handler) -> None:
        """끝점 이름의 수신 함수."""
        self.handlers[name] = handler

    def _schedule(self, src: str, dst: str, kind: str, payload: Any, at: float) -> None:
        key = (src, dst)
        at = max(at, self._last.get(key, at))          # 링크 FIFO
        self._last[key] = at
        self.q.schedule(at, self._deliver, src, dst, kind, payload)

    def send(self, now: float, src: str, dst: str, kind: str, payload: Any) -> bool:
        """최선 노력 송신 (유실되면 끝). 링크에 실었으면 True."""
        if not self.alive(src):
            return False
        self.sent[kind] += 1
        if src == dst:
            self.q.schedule(now, self._deliver, src, dst, kind, payload)
            return True
        delay = self.comm.transmit(src, dst, now)
        if delay is None:
            self.lost[kind] += 1
            return False
        self._schedule(src, dst, kind, payload, now + delay)
        return True

    def send_reliable(self, now: float, src: str, dst: str, kind: str, payload: Any) -> None:
        """유실되면 retry_s 마다 다시 보낸다 (송신자가 살아 있는 동안)."""
        if not self.alive(src):
            return
        if src == dst:
            self.sent[kind] += 1
            self.q.schedule(now, self._deliver, src, dst, kind, payload)
            return
        self.sent[kind] += 1
        delay = self.comm.transmit(src, dst, now)
        if delay is None:
            self.lost[kind] += 1
            self.q.schedule(now + self.retry_s, self._retry, src, dst, kind, payload)
            return
        self._schedule(src, dst, kind, payload, now + delay)

    def _retry(self, now: float, src: str, dst: str, kind: str, payload: Any) -> None:
        self.send_reliable(now, src, dst, kind, payload)

    def _deliver(self, now: float, src: str, dst: str, kind: str, payload: Any) -> None:
        if not self.alive(dst):
            return
        h = self.handlers.get(dst)
        if h is None:
            return
        self.delivered[kind] += 1
        h(now, src, kind, payload)


__all__ = ['EventQueue', 'Network']
