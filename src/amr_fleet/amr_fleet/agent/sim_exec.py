"""
가짜 실행기 — 협상으로 잡은 작업의 이동 · 확정 · 픽업 · 하역을 sim 시각으로 흉내 낸다 (rclpy 비의존).

Gazebo · Nav2 · BT 실행기 없이 에이전트의 할당 · 잠금 경로를 끝까지 돌리려고 쓴다 (P3 launch 시험, P4 빠른 계층).
실제 배치(P5)에서는 이 자리에 task_executor 의 BT 가 온다 — 픽업 직전 CommitTaskLock 노드가 같은 commit 을 부른다.

단계: TRAVEL(travel_s) → COMMIT(과반 ENTER_ACK 대기, commit_timeout_s 넘으면 FAILED) → PICKUP(pickup_s)
      → DELIVER(deliver_s) → DONE. COMMIT 중 잠금이 DENIED 면 FAILED (다른 로봇이 확정했다 — 픽업하지 않는다).
**픽업 시작 사건(pickup_start)은 잠금이 held 일 때만 낸다** — 이 규칙이 "중복 픽업 0" 의 실행기 쪽 절반이다.
"""

from __future__ import annotations

import dataclasses
from typing import Callable, Dict, List, Optional

TRAVEL = 'TRAVEL'
COMMIT = 'COMMIT'
PICKUP = 'PICKUP'
DELIVER = 'DELIVER'
DONE = 'DONE'
FAILED = 'FAILED'


@dataclasses.dataclass(frozen=True)
class SimExecConfig:
    """가짜 실행기 시간 (sim 초)."""

    pickup_s: float = 2.0
    deliver_s: float = 3.0
    commit_timeout_s: float = 30.0
    min_travel_s: float = 0.5


@dataclasses.dataclass
class _Job:
    task_id: str
    phase: str
    since: float
    travel_s: float


class SimExecutor:
    """작업 하나씩. step() 이 사건 목록을 돌려준다: {'event', 'task_id', 't'} (+ 'reason')."""

    def __init__(self, cfg: SimExecConfig = SimExecConfig()):
        self.cfg = cfg
        self.job: Optional[_Job] = None
        self.picked_up = False
        self.history: List[Dict[str, object]] = []

    @property
    def busy(self) -> bool:
        """작업을 쥐고 있나 (DONE · FAILED 전)."""
        return self.job is not None

    @property
    def task_id(self) -> Optional[str]:
        return None if self.job is None else self.job.task_id

    @property
    def phase(self) -> str:
        return '' if self.job is None else self.job.phase

    def start(self, task_id: str, travel_s: float, now: float) -> Dict[str, object]:
        """잠정 실행권(GRANTED)을 받은 작업으로 이동을 시작한다."""
        if self.job is not None:
            raise RuntimeError(f'이미 작업 중: {self.job.task_id}')
        self.job = _Job(task_id, TRAVEL, float(now), max(self.cfg.min_travel_s, float(travel_s)))
        self.picked_up = False
        return self._event('travel_start', now)

    def step(self, now: float, lock_phase: Callable[[str], str],
             commit: Callable[[str], str]) -> List[Dict[str, object]]:
        """
        주기 처리.

        lock_phase(task) → 'held' · 'denied' · 'voted' · 'pending' · 'idle' (VoteTaskLock.phase).
        commit(task) → 확정 단계로 보낸다 (VoteTaskLock.commit).
        """
        j = self.job
        if j is None:
            return []
        out: List[Dict[str, object]] = []
        now = float(now)
        if j.phase == TRAVEL and now - j.since >= j.travel_s:
            j.phase, j.since = COMMIT, now
            commit(j.task_id)
            out.append(self._event('commit_request', now))
        if j.phase == COMMIT:
            ph = lock_phase(j.task_id)
            if ph == 'held':
                j.phase, j.since = PICKUP, now
                self.picked_up = True
                out.append(self._event('pickup_start', now))
            elif ph == 'denied':
                out.append(self._finish(FAILED, now, 'lock_denied'))
                return out
            elif now - j.since >= self.cfg.commit_timeout_s:
                out.append(self._finish(FAILED, now, 'commit_timeout'))
                return out
            else:
                commit(j.task_id)         # 재시도 · 백오프 뒤에도 확정을 원한다는 표시를 유지 (멱등)
        if j.phase == PICKUP and now - j.since >= self.cfg.pickup_s:
            j.phase, j.since = DELIVER, now
            out.append(self._event('pickup_done', now))
        if j.phase == DELIVER and now - j.since >= self.cfg.deliver_s:
            out.append(self._finish(DONE, now, ''))
        return out

    def _event(self, name: str, now: float, reason: str = '') -> Dict[str, object]:
        ev: Dict[str, object] = {'event': name, 'task_id': self.job.task_id, 't': float(now)}
        if reason:
            ev['reason'] = reason
        self.history.append(ev)
        return ev

    def _finish(self, phase: str, now: float, reason: str) -> Dict[str, object]:
        self.job.phase = phase
        ev = self._event('done' if phase == DONE else 'failed', now, reason)
        self.job = None
        return ev
