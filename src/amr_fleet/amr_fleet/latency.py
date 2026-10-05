"""
통신 지연 시뮬레이션용 지연 큐 (명세 4.9 "로봇 간 통신 지연(최대 100 ms) 반영").

설계: docs/architecture/multi_robot.md 6장 — fleet ↔ 로봇 경계 노드의 송신 지연 큐.
- fleet_manager 는 /amr_XX/assign_task 호출 전, fleet_adapter 는 robot_state 발행 전에 큐에 넣는다.
- 지연은 메시지마다 U(comm_latency_ms[0], comm_latency_ms[1]) ms, drop_rate 확률로 유실.
- simulate_latency=false 면 지연·유실 없이 바로 보낸다 (실기 배포).
"""

from __future__ import annotations

import collections
import heapq
import itertools
import random
from typing import Any, Deque, Dict, Hashable, List, Optional, Sequence, Tuple, Union

SPEC_MAX_LATENCY_MS = 100.0   # 명세 4.9 상한


def parse_latency_range(value: Union[float, int, Sequence[float]]) -> Tuple[float, float]:
    """
    comm_latency_ms 파라미터 → (하한, 상한) [ms].

    [lo, hi] 목록이 기본 형식이다. 숫자 하나는 [0, 값] 으로 본다(하위 호환).
    음수, lo > hi, 원소 수가 1·2 가 아니면 ValueError.
    """
    if isinstance(value, bool):
        raise ValueError('comm_latency_ms 는 숫자 또는 [lo, hi] 목록이어야 한다')
    if isinstance(value, (int, float)):
        lo, hi = 0.0, float(value)
    else:
        items = [float(v) for v in value]
        if len(items) == 1:
            lo, hi = 0.0, items[0]
        elif len(items) == 2:
            lo, hi = items
        else:
            raise ValueError(f'comm_latency_ms 는 [lo, hi] 두 값이어야 한다: {list(value)}')
    if lo < 0.0 or hi < 0.0:
        raise ValueError(f'comm_latency_ms 는 음수일 수 없다: [{lo}, {hi}]')
    if lo > hi:
        raise ValueError(f'comm_latency_ms 하한이 상한보다 크다: [{lo}, {hi}]')
    return lo, hi


def sample_latency_s(max_ms: float, rng: Optional[random.Random] = None,
                     min_ms: float = 0.0) -> float:
    """[min_ms, max_ms] 균등 분포에서 지연 1개를 초 단위로 뽑는다. max_ms <= 0 이면 0."""
    if max_ms <= 0.0:
        return 0.0
    r = rng if rng is not None else random
    return r.uniform(max(0.0, min(min_ms, max_ms)), max_ms) * 1e-3


class LatencyModel:
    """메시지 1건의 지연 [s] 또는 유실(None)을 뽑는다. 파라미터는 fleet.yaml 과 1:1."""

    def __init__(self, simulate: bool = True,
                 comm_latency_ms: Union[float, Sequence[float]] = (0.0, SPEC_MAX_LATENCY_MS),
                 drop_rate: float = 0.0, seed: int = 0, history: int = 1000):
        self.min_ms, self.max_ms = parse_latency_range(comm_latency_ms)
        if not 0.0 <= drop_rate < 1.0:
            raise ValueError(f'drop_rate 는 [0, 1) 이어야 한다: {drop_rate}')
        self.simulate = bool(simulate)
        self.drop_rate = float(drop_rate)
        self.seed = int(seed)
        self.rng = random.Random(self.seed or None)   # 0 = 무작위
        self.sent = 0
        self.dropped = 0
        # 최근에 주입한 지연 [s] (유실 제외). 시험·진단용: 주입값이 설정 범위 안인지 확인한다
        self.history: Deque[float] = collections.deque(maxlen=history)

    @property
    def enabled(self) -> bool:
        """지연 또는 유실이 실제로 걸리는지 (아니면 큐 없이 바로 보낸다)."""
        return self.simulate and (self.max_ms > 0.0 or self.drop_rate > 0.0)

    def exceeds_spec(self) -> bool:
        """상한이 명세 최대(100 ms)를 넘는지 (스트레스 시험용 설정 경고)."""
        return self.max_ms > SPEC_MAX_LATENCY_MS

    def sample(self) -> Optional[float]:
        """지연 [s]. 유실이면 None. simulate=false 면 항상 0."""
        if not self.simulate:
            self.sent += 1
            return 0.0
        if self.drop_rate > 0.0 and self.rng.random() < self.drop_rate:
            self.dropped += 1
            return None
        self.sent += 1
        delay = sample_latency_s(self.max_ms, self.rng, self.min_ms)
        self.history.append(delay)
        return delay

    def describe(self) -> str:
        """로그용 한 줄 요약."""
        if not self.simulate:
            return 'off'
        return (f'U[{self.min_ms:g}, {self.max_ms:g}] ms, drop={self.drop_rate:g}, '
                f'seed={self.seed or "random"}')


class DelayQueue:
    """
    (해제 시각, 항목) 최소 힙. pop_ready(now) 가 시각이 된 항목을 순서대로 돌려준다.

    key(링크 이름)를 주면 그 링크 안에서는 FIFO 를 지킨다: 해제 시각 = max(직전 해제, now + 지연).
    DDS reliable 한 발행자의 메시지는 순서가 바뀌지 않으므로, 지연을 무작위로 뽑아도 같은 링크의
    메시지가 추월하지 않게 한다 (연구 브리프 §5.1.6 d_k = max(d_{k-1}, t_k + τ_k)).
    """

    def __init__(self):
        self._heap: List[Tuple[float, int, Any]] = []
        self._seq = itertools.count()
        self._last: Dict[Hashable, float] = {}

    def __len__(self) -> int:
        return len(self._heap)

    def push(self, item: Any, now: float, delay_s: float, key: Optional[Hashable] = None) -> float:
        """항목을 now + delay_s (링크 FIFO 면 그 이후) 에 해제되도록 넣고 해제 시각을 돌려준다."""
        release = now + max(0.0, delay_s)
        if key is not None:
            release = max(release, self._last.get(key, release))
            self._last[key] = release
        heapq.heappush(self._heap, (release, next(self._seq), item))
        return release

    def pop_ready(self, now: float) -> List[Any]:
        """해제 시각 <= now 인 항목을 해제 시각(같으면 삽입) 순으로 모두 꺼낸다."""
        ready: List[Any] = []
        while self._heap and self._heap[0][0] <= now:
            ready.append(heapq.heappop(self._heap)[2])
        return ready

    def next_release(self) -> Optional[float]:
        """가장 이른 해제 시각 (비어 있으면 None)."""
        return self._heap[0][0] if self._heap else None

    def clear(self) -> None:
        """전체 비우기 (링크 FIFO 기록 포함)."""
        self._heap.clear()
        self._last.clear()
