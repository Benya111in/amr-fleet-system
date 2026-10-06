"""
작업 ID 정족수 잠금 — negotiation.task_lock.TaskLock 계약을 투표 코어(task 채널)로 구현한다 (P3).

f3 의 TaskLock 은 인터페이스와 가짜뿐이었다. 여기서 실제 잠금을 붙인다 (f1 · f3 요청):
- try_acquire(task)  : 처음이면 `task:<id>` 를 hold_enter 로 요청 → PENDING. 과반 잠정 표(VOTED)를 쥐면
                       GRANTED — "이동 시작" 허가다 (아직 픽업 허가가 아니다). 확정 표가 막는다는 NACK(busy)
                       를 과반 불가 수(n − q + 1)만큼 받으면 DENIED.
- commit(task)       : 픽업 시작 직전에 ENTER 를 보낸다. 픽업은 과반 ENTER_ACK(HELD) 뒤에만 — is_held(task).
- release(task)      : 확정 전이면 요청을 포기한다 (잠정 표 회수). **확정(HELD) 뒤에는 풀지 않는다** —
                       완료 뒤 해제하면 늦게 도착한 옛 낙찰이 끝난 작업을 다시 시작한다
                       (TLC 반례 formal/fleet/traces/TL_N3_T1_R2_C0_releaseAfterDone.txt).

중복 픽업 0 의 근거: 픽업은 HELD 뒤에만 하고, HELD 는 확정 표 과반 → 정족수 교차 + 확정 표 배타
(protocol.md §2). 잠정 표가 기한으로 풀려 두 로봇이 같은 작업으로 **이동**할 수는 있지만 둘 다 확정할 수는 없다.

투표 코어의 요청자는 활성 시도가 하나라 로봇은 작업 잠금을 한 번에 하나만 추구한다 (CBAA L_t = 1 과 맞다).
보낼 메시지는 outbox 에 쌓고 호출자(core)가 비운다. 시각은 clock() (sim 초) — CBAA 가 넘기는 now 는 라운드
번호라 쓰지 않는다.
"""

from __future__ import annotations

from typing import Callable, List, Optional, Set

from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.vote_core import ENTERING, VOTED, ZoneAgent
from amr_fleet.negotiation.task_lock import LockResult, LockStatus, TaskLock

TASK_PREFIX = 'task:'


def task_resource(task_id: str) -> str:
    """작업 ID → 투표 자원 ID."""
    return f'{TASK_PREFIX}{task_id}'


class VoteTaskLock(TaskLock):
    """투표 코어 위의 작업 ID 잠금 (로봇 1 대 쪽)."""

    def __init__(self, agent: ZoneAgent, clock: Callable[[], float], hold_s: float = 60.0,
                 priority: int = 0):
        if not hold_s > 0:
            raise ValueError('hold_s 는 양수여야 한다')
        self.agent = agent
        self.clock = clock
        self.hold_s = float(hold_s)
        self.priority = int(priority)
        self.outbox: List[VoteMsg] = []
        self.denied: Set[str] = set()
        self.n_requests = 0

    # ------------------------------------------------------------------ 조회
    @property
    def deny_threshold(self) -> int:
        """확정 표가 막는 투표자가 이만큼이면 과반을 모을 수 없다 (n − q + 1)."""
        return self.agent.n - self.agent.q + 1

    def is_held(self, task_id: str) -> bool:
        """픽업 허가 (과반 ENTER_ACK 를 받은 확정 잠금)."""
        return self.agent.may_enter(task_resource(task_id))

    def is_denied(self, task_id: str) -> bool:
        """다른 로봇이 확정했다고 판단했나."""
        return task_resource(task_id) in self.denied

    def pursuing(self) -> Optional[str]:
        """지금 잡으려는 작업 ID (없으면 None)."""
        for res in self.agent.requested():
            if res.startswith(TASK_PREFIX):
                return res[len(TASK_PREFIX):]
        return None

    def phase(self, task_id: str) -> str:
        """'held' · 'denied' · 'voted'(잠정 과반) · 'pending' · 'idle'."""
        res = task_resource(task_id)
        if self.agent.may_enter(res):
            return 'held'
        if res in self.denied:
            return 'denied'
        if res in self.agent.requested():
            view = self.agent.attempt_view()
            if view is not None and view['phase'] in (VOTED, ENTERING):
                return 'voted'
            return 'pending'
        return 'idle'

    # ------------------------------------------------------------------ TaskLock 계약
    def try_acquire(self, task_id: str, robot_id: str, now: float) -> LockResult:
        if robot_id != self.agent.me:
            raise ValueError(f'다른 로봇의 잠금을 청할 수 없다: {robot_id} != {self.agent.me}')
        res = task_resource(task_id)
        t = self.clock()
        if self.agent.may_enter(res):
            return LockResult(LockStatus.GRANTED, robot_id)
        if res in self.denied:
            return LockResult(LockStatus.DENIED)
        if res in self.agent.requested():
            if self._check_denied(res):
                return LockResult(LockStatus.DENIED)
            view = self.agent.attempt_view()
            if view is not None and view['phase'] in (VOTED, ENTERING):
                return LockResult(LockStatus.GRANTED, robot_id)   # 잠정 과반: 이동 허가
            return LockResult(LockStatus.PENDING)
        self.outbox += self.agent.request([res], t, priority=self.priority, hold_enter=True,
                                          hold_s=self.hold_s)
        self.n_requests += 1
        return LockResult(LockStatus.PENDING)

    def release(self, task_id: str, robot_id: str, now: float) -> None:
        res = task_resource(task_id)
        if self.agent.may_enter(res):
            return                       # 확정 잠금은 풀지 않는다 (완료 뒤에도)
        if res in self.agent.requested():
            self.outbox += self.agent.cancel(self.clock())

    def holder(self, task_id: str) -> Optional[str]:
        return self.agent.me if self.is_held(task_id) else None

    # ------------------------------------------------------------------ 실행기 쪽
    def hold(self, task_id: str) -> None:
        """중앙 할당을 받은 작업의 잠정 잠금을 시작한다 (NORMAL: 픽업 직전 확정용)."""
        res = task_resource(task_id)
        if self.agent.may_enter(res) or res in self.agent.requested() or res in self.denied:
            return
        self.outbox += self.agent.request([res], self.clock(), priority=self.priority,
                                          hold_enter=True, hold_s=self.hold_s)
        self.n_requests += 1

    def commit(self, task_id: str) -> str:
        """
        픽업 시작 직전: 확정 단계로 보낸다. 돌려주는 값은 phase().

        요청이 없으면(예: 재시작 뒤) 곧바로 확정할 요청을 새로 낸다 (hold_enter + commit 표시).
        """
        res = task_resource(task_id)
        if self.agent.may_enter(res) or res in self.denied:
            return self.phase(task_id)
        if res not in self.agent.requested():
            self.hold(task_id)
        self.outbox += self.agent.commit(self.clock())
        return self.phase(task_id)

    def poll(self) -> None:
        """주기마다: 진행 중인 작업 요청이 확정 표에 막혔는지 본다."""
        task = self.pursuing()
        if task is not None:
            self._check_denied(task_resource(task))

    def drain(self) -> List[VoteMsg]:
        """쌓인 송신 메시지를 꺼낸다."""
        out, self.outbox = self.outbox, []
        return out

    def _own_unreleased(self, res: str) -> Set[str]:
        """내 옛 요청의 RELEASE/ABANDON 을 아직 ACK 하지 않은 투표자 — 그들의 busy 는 내 옛 확정 표 탓일 수 있다."""
        out: Set[str] = set()
        for rel in self.agent.releasing.values():
            if res in rel.resources:
                out |= set(self.agent.cfg.members) - set(rel.acked)
        return out

    def _check_denied(self, res: str) -> bool:
        view = self.agent.attempt_view()
        if view is None or res not in view['resources']:
            return False
        busy = set(view['busy']) - self._own_unreleased(res)
        if len(busy) >= self.deny_threshold:
            self.denied.add(res)
            self.outbox += self.agent.cancel(self.clock())
            return True
        return False


__all__ = ['TASK_PREFIX', 'VoteTaskLock', 'task_resource']
