"""
우선순위 + 마감 스케줄링 큐 (명세 4.8 "우선순위, 마감 시간을 고려한 스케줄링").

정렬 규칙
1. 유효 우선순위 = priority + 나이 가산(starvation guard). 가산은 age_boost_rate [1/s] 로
   선형 증가하고 age_boost_max 에서 멈춘다 → 낮은 우선순위 작업도 언젠가는 상위 밴드로 올라간다.
2. 유효 우선순위를 band_width 로 나눈 밴드(높을수록 먼저).
3. 같은 밴드 안에서 deadline_weight > 0 이면 마감 있는 작업이 마감 없는 작업보다 먼저다
   (명시적 플래그 — 마감이 아무리 멀어도 "마감 없음" 뒤로 밀리지 않는다).
4. 그다음 가중 점수가 작은 순:
       score = deadline_weight * slack + fifo_weight * (created - now)
   slack = deadline - now (마감 없는 작업은 slack 항 없이 fifo 항만). deadline_weight=1,
   fifo_weight=0 이면 순수 EDF, deadline_weight=0, fifo_weight=1 이면 FIFO 다 (마감 무시).
5. 동점은 생성 시각, task_id 순 (결정론).

deadline · created · now 는 모두 노드 시계 초다 (task_schema 가 접수 시 ISO 마감을 옮긴다).

큐 크기가 작고(수십) 키가 시간에 따라 변하므로 힙 대신 요청 시점에 정렬한다.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, Iterator, List, Optional, Tuple

from amr_fleet.task_schema import TaskSpec


@dataclasses.dataclass
class SchedulerConfig:
    """스케줄러 가중치. fleet.yaml 의 scheduler.* 와 1:1."""

    band_width: int = 32              # 우선순위 밴드 폭 (0~255 → 8 밴드)
    deadline_weight: float = 1.0      # 밴드 내 EDF 가중치
    fifo_weight: float = 0.0          # 밴드 내 도착순 가중치
    age_boost_rate: float = 0.5       # [priority/s] 대기 1초당 유효 우선순위 가산
    age_boost_max: float = 64.0       # 가산 상한 (= 2 밴드)

    def __post_init__(self):
        if self.band_width < 1:
            raise ValueError('band_width 는 1 이상이어야 한다')
        if self.age_boost_rate < 0 or self.age_boost_max < 0:
            raise ValueError('age_boost_* 는 음수일 수 없다')
        if self.deadline_weight < 0 or self.fifo_weight < 0:
            raise ValueError('deadline_weight · fifo_weight 는 음수일 수 없다')


class TaskScheduler:
    """대기(PENDING) 작업 큐. 같은 task_id 는 한 번만 들어간다."""

    def __init__(self, config: Optional[SchedulerConfig] = None):
        self.config = config or SchedulerConfig()
        self._tasks: Dict[str, TaskSpec] = {}

    # --- 컨테이너 프로토콜 ---
    def __len__(self) -> int:
        return len(self._tasks)

    def __contains__(self, task_id: str) -> bool:
        return task_id in self._tasks

    def __iter__(self) -> Iterator[TaskSpec]:
        return iter(list(self._tasks.values()))

    def get(self, task_id: str) -> Optional[TaskSpec]:
        """task_id 로 조회 (없으면 None)."""
        return self._tasks.get(task_id)

    # --- 삽입/제거 ---
    def push(self, task: TaskSpec) -> None:
        """대기 큐에 넣는다. 중복 task_id 면 ValueError."""
        if task.task_id in self._tasks:
            raise ValueError(f'중복 task_id: {task.task_id}')
        self._tasks[task.task_id] = task

    def remove(self, task_id: str) -> Optional[TaskSpec]:
        """큐에서 뺀다. 없으면 None."""
        return self._tasks.pop(task_id, None)

    def clear(self) -> None:
        """전체 비우기."""
        self._tasks.clear()

    # --- 정렬 ---
    def effective_priority(self, task: TaskSpec, now: float) -> float:
        """나이 가산이 반영된 우선순위 (starvation guard)."""
        age = max(0.0, now - task.created)
        boost = min(self.config.age_boost_max, self.config.age_boost_rate * age)
        return task.priority + boost

    def band(self, task: TaskSpec, now: float) -> int:
        """유효 우선순위 밴드 (클수록 먼저)."""
        return int(self.effective_priority(task, now) // self.config.band_width)

    def sort_key(self, task: TaskSpec, now: float) -> Tuple[int, int, float, float, str]:
        """오름차순 정렬 키: (-밴드, 마감 없음 플래그, 가중 점수, 생성 시각, task_id)."""
        cfg = self.config
        score = cfg.fifo_weight * (task.created - now)
        no_deadline = 0
        if cfg.deadline_weight > 0.0:
            if task.deadline is None:
                no_deadline = 1
            else:
                score += cfg.deadline_weight * (task.deadline - now)
        return (-self.band(task, now), no_deadline, score, task.created, task.task_id)

    def ordered(self, now: float) -> List[TaskSpec]:
        """현재 시각 기준 실행 순서."""
        return sorted(self._tasks.values(), key=lambda t: self.sort_key(t, now))

    def peek(self, now: float) -> Optional[TaskSpec]:
        """가장 먼저 실행할 작업 (제거하지 않음)."""
        order = self.ordered(now)
        return order[0] if order else None

    def pop(self, now: float) -> Optional[TaskSpec]:
        """가장 먼저 실행할 작업을 꺼낸다."""
        top = self.peek(now)
        if top is not None:
            del self._tasks[top.task_id]
        return top

    # --- 마감 ---
    def overdue(self, now: float) -> List[TaskSpec]:
        """마감이 지난 대기 작업 (마감 오름차순)."""
        late = [t for t in self._tasks.values() if is_deadline_missed(t, now)]
        return sorted(late, key=lambda t: (t.deadline, t.task_id))


def is_deadline_missed(task: TaskSpec, now: float) -> bool:
    """마감이 있고 now 가 마감을 지났으면 True."""
    return task.deadline is not None and now > task.deadline
