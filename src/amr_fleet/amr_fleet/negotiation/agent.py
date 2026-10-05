"""
로봇 1대의 CBAA 협상 상태기계 (rclpy 비의존).

한 라운드(주기 T_b)의 순서 — 연구 계산(mrta_small.py · verify_mrta2.py)과 같은 단계 순서다.
1. start_round(now)  : 지난 라운드 끝 상태를 기억하고, 쥔 작업이 없으면 입찰한다 (L_t = 1)
2. make_message(now) : 전체 상태를 방송한다 (바뀌지 않아도 다시 보낸다)
3. receive(msg, now) : 받은 상태를 병합 규칙(consensus)으로 합친다. 몇 번이든 부를 수 있다
4. finish_round(now) : 하트비트·건강 무효화 → 패배 판정 → 안정 계수 → 국소 확정 → 작업 잠금

국소 확정: "자기 상태(입찰 표 + 쥔 작업)가 K 라운드 연속 불변" 이고, require_all_heard 이면
"이번 라운드에 살아 있다고 보는 로봇 전원에게서 직접 받았다". 확정하면 TaskLock 에 실행권을 청하고,
GRANTED 를 받아야 실행을 시작한다. 실행 중인 작업은 announce_commits 이면 LOCKED 입찰로 방송해
다른 로봇이 그 작업을 놓게 한다 (비선점).

시간 단위는 호출자가 정한다 (시뮬레이터는 라운드 번호). heartbeat_timeout 도 같은 단위.
계약: 전원 직접 수신 판정은 last_direct == now **정확 일치**다. 한 라운드의 receive·finish_round 에는 같은
라운드 식별자(라운드 번호 또는 라운드 시작 스탬프)를 넘긴다 — 수신 시각(sim-time 초)을 넘기면 확정이 나지 않는다.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Dict, List, Mapping, Optional, Sequence

from amr_fleet.negotiation.bids import Bid, better, LOCKED_COST, NO_BID, StateMessage, table_key
from amr_fleet.negotiation.consensus import get_rule
from amr_fleet.negotiation.task_lock import LockStatus, OptimisticTaskLock, TaskLock

LIVENESS_MODES = ('direct', 'indirect')


@dataclasses.dataclass
class AgentConfig:
    """
    협상 파라미터.

    rule              : 병합 규칙 'cbba' (기본, 철회·무효화가 퍼진다) | 'max' (원 CBAA, 재현용)
    stable_rounds     : K — 국소 확정에 필요한 연속 불변 라운드 수 (≥ 1)
    require_all_heard : 이번 라운드에 생존 로봇 전원에게서 직접 받아야 확정
    heartbeat_timeout : H — 이보다 오래 소식이 없는 로봇의 낙찰을 무효화 (None = 끔)
    liveness          : 'indirect'(기본) = 출처별 최신 시각(간접 포함)으로 생존 판정 | 'direct' = 직접 수신만
                        (손실 40 %·H=3 에서 직접 판정은 실행당 오탐 무효화 8.6 건, 간접은 0.05 건)
    local_commit      : False 면 확정·잠금을 하지 않는다 (전역 종료 오라클 재현용)
    announce_commits  : 실행을 시작한 작업을 LOCKED 입찰로 방송한다
    """

    rule: str = 'cbba'
    stable_rounds: int = 3
    require_all_heard: bool = True
    heartbeat_timeout: Optional[float] = 5.0
    liveness: str = 'indirect'
    local_commit: bool = True
    announce_commits: bool = True

    @classmethod
    def strict_mode(cls) -> 'AgentConfig':
        """
        엄격 모드 제안값 (작업 ID 배타 잠금과 함께 쓴다) — 스윕 근거: negotiation.md 6 절.

        중복은 잠금이 막으므로 전원 직접 수신을 끄고 K=3 으로 확정 지연을 줄인다.
        """
        return cls(rule='cbba', stable_rounds=3, require_all_heard=False,
                   heartbeat_timeout=5.0, liveness='indirect')

    @classmethod
    def optimistic_mode(cls) -> 'AgentConfig':
        """낙관 모드(잠금 없이 국소 확정만, 비교군) 제안값: K=3 + 전원 직접 수신."""
        return cls(rule='cbba', stable_rounds=3, require_all_heard=True,
                   heartbeat_timeout=5.0, liveness='indirect')

    def validate(self) -> None:
        """잘못된 값이면 ValueError."""
        get_rule(self.rule)
        if self.stable_rounds < 1:
            raise ValueError('stable_rounds(K) 는 1 이상이어야 한다')
        if self.heartbeat_timeout is not None and self.heartbeat_timeout <= 0:
            raise ValueError('heartbeat_timeout(H) 는 양수이거나 None 이어야 한다')
        if self.liveness not in LIVENESS_MODES:
            raise ValueError(f'liveness 는 {LIVENESS_MODES} 중 하나여야 한다: {self.liveness!r}')


class CbaaAgent:
    """로봇 1대의 협상 상태. 같은 코드를 오프라인 시뮬레이터·시험·(다음 물결) ROS 노드가 쓴다."""

    def __init__(self, robot_id: str, peers: Sequence[str], tasks: Sequence[str],
                 costs: Optional[Mapping[str, float]] = None,
                 config: Optional[AgentConfig] = None,
                 task_lock: Optional[TaskLock] = None, now: float = 0.0):
        self.robot_id = robot_id
        self.peers: List[str] = [p for p in peers if p != robot_id]
        self.config = config or AgentConfig()
        self.config.validate()
        self._merge = get_rule(self.config.rule)
        self.task_lock = task_lock if task_lock is not None else OptimisticTaskLock()
        self.tasks: List[str] = list(tasks)
        self.table: Dict[str, Bid] = {t: NO_BID for t in self.tasks}
        self.costs: Dict[str, float] = {}
        self.set_costs(costs or {})
        # 출처별 최신 시각(CBBA s_i, 간접 포함)과 직접 수신 시각. 시작 시각에 한 번 들었다고 본다
        self.info_time: Dict[str, float] = {p: now for p in self.peers}
        self.info_time[robot_id] = now
        self.last_direct: Dict[str, float] = {p: now for p in self.peers}
        self.peer_healthy: Dict[str, bool] = {p: True for p in self.peers}
        self.healthy = True
        self.assigned: Optional[str] = None      # x_i: 지금 쥔(이긴다고 믿는) 작업
        self.executing: Optional[str] = None     # 잠금을 얻어 실행을 시작한 작업
        self.pending_lock: Optional[str] = None
        self.blocked: set = set()                # 잠금이 거절됐는데 보유자를 모르는 작업
        self.stable = 0
        self.commit_time: Optional[float] = None
        self._prev = (table_key(self.table), self.assigned)
        self.n_bids = 0
        self.n_received = 0
        self.n_invalidated = 0
        self.n_lock_denied = 0

    # ------------------------------------------------------------------ 입력
    def set_costs(self, costs: Mapping[str, float]) -> None:
        """작업별 내 비용 [s] (없거나 비유한 값이면 입찰하지 않는다)."""
        self.costs = {t: float(costs[t]) for t in self.tasks if t in costs}

    def set_tasks(self, tasks: Sequence[str]) -> None:
        """협상 후보 작업(순서 = 동점 시 우선)을 바꾼다. 빠진 작업의 입찰·보유는 버린다."""
        self.tasks = list(tasks)
        self.table = {t: self.table.get(t, NO_BID) for t in self.tasks}
        self.costs = {t: c for t, c in self.costs.items() if t in self.table}
        if self.assigned not in self.table:
            self._drop_assigned(None)
        if self.executing is not None and self.executing not in self.table:
            self.executing = None

    def set_localization_healthy(self, healthy: bool, now: float) -> None:
        """
        위치추정 건강 게이트.

        나빠지면 입찰하지 않고, 아직 실행을 시작하지 않은 낙찰은 철회한다 (자기 위치를 틀리게 알고
        입찰하면 비용이 거짓이 된다). 실행 중인 작업은 그대로 둔다 (픽업 뒤 처리는 실행기 몫).
        """
        healthy = bool(healthy)
        if healthy == self.healthy:
            return
        self.healthy = healthy
        if not healthy and self.executing is None and self.assigned is not None:
            if self.table.get(self.assigned, NO_BID).winner == self.robot_id:
                self.table[self.assigned] = NO_BID
            self._drop_assigned(now)
            self.stable = 0

    # ------------------------------------------------------------------ 라운드
    def start_round(self, now: float) -> Optional[str]:
        """라운드 시작: 지난 라운드 끝 상태를 기억하고 입찰한다. 새로 입찰한 작업을 돌려준다."""
        self._prev = (table_key(self.table), self.assigned)
        return self.bid(now)

    def bid(self, now: float) -> Optional[str]:
        """쥔 작업이 없으면 지금 표에서 이길 수 있는 가장 싼 작업에 입찰한다 (동점은 후보 순서)."""
        if self.executing is not None or self.assigned is not None or not self.healthy:
            return None
        best_task, best_cost = None, math.inf
        for task in self.tasks:
            cost = self.costs.get(task, math.inf)
            if not math.isfinite(cost) or cost >= best_cost or task in self.blocked:
                continue
            if better(Bid(cost, self.robot_id), self.table[task]):
                best_task, best_cost = task, cost
        if best_task is not None:
            self.assigned = best_task
            self.table[best_task] = Bid(best_cost, self.robot_id)
            self.n_bids += 1
        return best_task

    def make_message(self, now: float) -> StateMessage:
        """방송할 전체 상태."""
        info = dict(self.info_time)
        info[self.robot_id] = now
        return StateMessage(sender=self.robot_id, stamp=now, bids=dict(self.table),
                            info_time=info, healthy=self.healthy, assigned=self.assigned,
                            executing=self.executing is not None)

    def receive(self, msg: StateMessage, now: float) -> None:
        """다른 로봇의 상태를 병합한다 (모르는 작업의 입찰은 무시한다)."""
        sender = msg.sender
        if sender == self.robot_id:
            return
        self.n_received += 1
        for task, bk in msg.bids.items():
            bi = self.table.get(task)
            if bi is None:
                continue
            if bk.locked or bi.locked:
                # 잠금은 사실이다: 더 나은 잠금만 잠금을 바꾸고, 잠금은 늘 받아들인다 (두 규칙 공통)
                if bk.locked and better(bk, bi):
                    self.table[task] = bk
                continue
            self.table[task] = self._merge(self.robot_id, sender, bi, bk,
                                           self.info_time, msg.info_time)
        # 시각 갱신은 병합 뒤에 (병합은 받기 전 시각으로 비교한다)
        if sender not in self.last_direct:        # 처음 보는 로봇도 받아들인다
            self.peers.append(sender)
        self.last_direct[sender] = now
        self.peer_healthy[sender] = bool(msg.healthy)
        self.info_time[sender] = max(self.info_time.get(sender, -math.inf), now)
        for other, t in msg.info_time.items():
            if other in (self.robot_id, sender):
                continue
            if t > self.info_time.get(other, -math.inf):
                self.info_time[other] = t

    def finish_round(self, now: float) -> Optional[str]:
        """라운드 끝 처리. 이번 라운드에 실행을 시작한 작업 ID 를 돌려준다 (없으면 None)."""
        self._invalidate(now)
        if (self.executing is None and self.assigned is not None
                and self.table.get(self.assigned, NO_BID).winner != self.robot_id):
            self._drop_assigned(now)              # 더 나은 입찰에 졌다 → 다음 라운드에 재입찰
        same = (table_key(self.table), self.assigned) == self._prev
        self.stable = self.stable + 1 if same else 0
        if not self.config.local_commit or self.executing is not None or self.assigned is None:
            return None
        if self.stable < self.config.stable_rounds:
            return None
        if self.config.require_all_heard and not self.heard_all_alive(now):
            return None
        return self._acquire(now)

    # ------------------------------------------------------------------ 판정
    def liveness_time(self, peer: str) -> float:
        """생존 판정에 쓰는 마지막 소식 시각."""
        if self.config.liveness == 'indirect':
            return self.info_time.get(peer, -math.inf)
        return self.last_direct.get(peer, -math.inf)

    def is_alive(self, peer: str, now: float) -> bool:
        """하트비트 기준으로 살아 있다고 보는지 (H 가 None 이면 늘 True)."""
        h = self.config.heartbeat_timeout
        return h is None or now - self.liveness_time(peer) <= h

    def suspects(self, now: float) -> List[str]:
        """낙찰을 무효화할 로봇: 하트비트가 끊겼거나 위치추정 건강이 나쁘다고 알려 온 로봇."""
        return [p for p in self.peers
                if not self.is_alive(p, now) or not self.peer_healthy.get(p, True)]

    def heard_all_alive(self, now: float) -> bool:
        """이번 라운드(now)에 살아 있다고 보는 로봇 전원에게서 직접 받았는지."""
        return all(self.last_direct.get(p) == now for p in self.peers if self.is_alive(p, now))

    def snapshot_key(self):
        """안정 판정과 같은 기준의 현재 상태 정규형."""
        return (table_key(self.table), self.assigned)

    def table_changed(self) -> bool:
        """이번 라운드 시작 뒤 입찰 표가 바뀌었는지 (이벤트 구동 방송 판정용)."""
        return table_key(self.table) != self._prev[0]

    # ------------------------------------------------------------------ 내부
    def _invalidate(self, now: float) -> None:
        bad = set(self.suspects(now))
        if not bad:
            return
        for task, b in self.table.items():
            if b.winner in bad and not b.locked:
                self.table[task] = NO_BID
                self.n_invalidated += 1

    def _drop_assigned(self, now: Optional[float]) -> None:
        if self.pending_lock is not None:
            self.task_lock.release(self.pending_lock, self.robot_id,
                                   now if now is not None else 0.0)
            self.pending_lock = None
        self.assigned = None

    def _acquire(self, now: float) -> Optional[str]:
        task = self.assigned
        res = self.task_lock.try_acquire(task, self.robot_id, now)
        if res.status is LockStatus.GRANTED:
            self.executing = task
            self.pending_lock = None
            self.commit_time = now
            if self.config.announce_commits:
                self.table[task] = Bid(LOCKED_COST, self.robot_id)
            return task
        if res.status is LockStatus.PENDING:
            self.pending_lock = task
            return None
        # DENIED: 다른 로봇이 실행권을 쥐었다 → 그 작업을 잠긴 것으로 표시하고 놓는다
        self.n_lock_denied += 1
        self.pending_lock = None
        if res.holder:
            self.table[task] = Bid(LOCKED_COST, res.holder)
        else:
            self.table[task] = NO_BID
            self.blocked.add(task)
        self.assigned = None
        self.stable = 0
        return None


__all__ = ['AgentConfig', 'CbaaAgent', 'LIVENESS_MODES']
