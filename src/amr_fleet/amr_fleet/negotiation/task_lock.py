"""
작업 ID 정족수 잠금 — **인터페이스만** (보고서 56행 "실행권").

협상(누가 할지)과 실행권(실제로 시작)을 나눈다. 협상이 국소 확정에 이르면 로봇은 작업 ID 에 대해
잠금을 청한다. 실제 구현은 구역 투표 모듈(f1 의 vote_core)을 "자원 ID → 배타 표"로 일반화해
작업 ID 에 같은 연산을 쓰는 것이고, 그 결합은 다음 물결에서 한다. 여기에는 계약과 시험용 가짜만 둔다.

계약 (구현이 지켜야 할 것)
- 배타성: 같은 task_id 에 동시에 GRANTED 를 받는 로봇은 하나다 (정족수 교차 + 배타·영속 표).
- GRANTED 는 비선점이다: 픽업을 시작한 작업은 재할당하지 않는다 (픽업 뒤 고장은 운영자 경보 FAILED).
- PENDING 은 "아직 과반을 못 모았다" 이고, DENIED 는 "다른 로봇이 쥐었다" 이다 (holder 를 알면 채운다).
- 응답이 제때 오지 않으면 GRANTED 가 아니다 (무응답 = 미허가, VDA 5050 3.0.0 의 RELEASE 의미론).
- release 는 멱등이다.

가짜 구현
- OptimisticTaskLock: 늘 GRANTED — "낙관 모드"(CBAA 국소 확정만) 비교군. 중복을 막지 않는다.
- ExclusiveTaskLock : 프로세스 안 공유 표로 배타성을 보장 — 과반 연결이 유지되는 이상적 정족수 잠금의 모형.
  grant_delay 로 확정 왕복(2 회)을 라운드 수로 흉내 낼 수 있다.
"""

from __future__ import annotations

import abc
import dataclasses
import enum
from typing import Dict, Optional, Tuple


class LockStatus(enum.Enum):
    """잠금 요청 결과."""

    GRANTED = 'granted'
    PENDING = 'pending'
    DENIED = 'denied'


@dataclasses.dataclass(frozen=True)
class LockResult:
    """잠금 요청 응답. DENIED 일 때 holder 는 지금 쥔 로봇 (모르면 None)."""

    status: LockStatus
    holder: Optional[str] = None


class TaskLock(abc.ABC):
    """작업 ID 배타 잠금 인터페이스."""

    @abc.abstractmethod
    def try_acquire(self, task_id: str, robot_id: str, now: float) -> LockResult:
        """task_id 의 실행권을 청한다. 같은 로봇이 다시 부르면 진행 상황을 돌려준다."""

    @abc.abstractmethod
    def release(self, task_id: str, robot_id: str, now: float) -> None:
        """잠금(또는 진행 중인 요청)을 놓는다. 쥐고 있지 않으면 아무것도 하지 않는다."""

    def holder(self, task_id: str) -> Optional[str]:
        """지금 쥔 로봇 (알 수 없으면 None). 진단·시험용."""
        return None


class OptimisticTaskLock(TaskLock):
    """늘 허가한다 (낙관 모드 비교군). 몇 번 허가했는지만 센다."""

    def __init__(self) -> None:
        self.grants: Dict[str, list] = {}

    def try_acquire(self, task_id: str, robot_id: str, now: float) -> LockResult:
        self.grants.setdefault(task_id, []).append(robot_id)
        return LockResult(LockStatus.GRANTED, robot_id)

    def release(self, task_id: str, robot_id: str, now: float) -> None:
        return None


class ExclusiveTaskLock(TaskLock):
    """
    프로세스 안 공유 표로 배타성을 보장하는 가짜 정족수 잠금.

    grant_delay > 0 이면 첫 요청부터 그만큼(같은 시간 단위) 지난 뒤의 요청에서 허가한다 —
    그 사이 다른 로봇이 먼저 허가를 받으면 DENIED. 기다리는 동안 청한 로봇이 release 하면 요청이 사라진다.
    """

    def __init__(self, grant_delay: float = 0.0) -> None:
        if grant_delay < 0.0:
            raise ValueError('grant_delay 는 0 이상이어야 한다')
        self.grant_delay = float(grant_delay)
        self._holder: Dict[str, str] = {}
        self._pending: Dict[Tuple[str, str], float] = {}
        self.denied = 0

    def try_acquire(self, task_id: str, robot_id: str, now: float) -> LockResult:
        held = self._holder.get(task_id)
        if held is not None:
            if held == robot_id:
                return LockResult(LockStatus.GRANTED, robot_id)
            self._pending.pop((task_id, robot_id), None)
            self.denied += 1
            return LockResult(LockStatus.DENIED, held)
        first = self._pending.setdefault((task_id, robot_id), now)
        if now - first >= self.grant_delay:
            self._pending.pop((task_id, robot_id), None)
            self._holder[task_id] = robot_id
            return LockResult(LockStatus.GRANTED, robot_id)
        return LockResult(LockStatus.PENDING)

    def release(self, task_id: str, robot_id: str, now: float) -> None:
        self._pending.pop((task_id, robot_id), None)
        if self._holder.get(task_id) == robot_id:
            del self._holder[task_id]

    def holder(self, task_id: str) -> Optional[str]:
        return self._holder.get(task_id)


__all__ = ['ExclusiveTaskLock', 'LockResult', 'LockStatus', 'OptimisticTaskLock', 'TaskLock']
