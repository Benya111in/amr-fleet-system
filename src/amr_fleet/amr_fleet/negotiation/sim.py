"""
오프라인 동기 라운드 시뮬레이터 — 로봇 N 대의 CbaaAgent 를 완전연결 망에서 돌린다 (ROS 없음).

라운드 순서 (연구 계산 mrta_small.py / verify_mrta2.py 와 같다)
  0. 이탈·건강 사건 적용 (라운드 시작)
  1. 살아 있는 로봇이 start_round → 입찰
  2. 살아 있는 로봇이 송신자 id 순으로 전체 상태 방송. 수신자 id 순으로 링크 모델이 유실·지연을 정하고,
     전달되면 receive. 라운드 벽시계 = 유실이 하나라도 있으면 라운드 시한, 아니면 전달 지연의 최댓값
  3. 살아 있는 로봇이 finish_round (무효화 → 패배 판정 → 안정 → 국소 확정 → 작업 잠금)
  4. 종료 판정
     - 'local'  : 살아 있는 로봇 전원이 실행 중이거나 (K 라운드 안정 + 쥔 작업 없음) — 각 로봇의 국소 확정
     - 'global' : 살아 있는 로봇 전원이 K 라운드 안정 (전역 종료 오라클 — 실제 시스템에는 없다. 재현 전용)
     - 'fixed'  : max_rounds 까지. 고장 실험은 이것을 쓴다 — 'local' 은 살아 있는 로봇이 모두 "할 일 끝"
                  이라고 믿는 순간 멈추므로, 하트비트(H)가 이탈을 알아채기 전에 끝나 고착을 과대평가한다

지표 (연구 계산과 같은 정의)
- dup        : 같은 작업을 쥔 로봇 수 − 1 의 합 (local 모드는 실행을 시작한 로봇, 그 밖은 살아 있는 로봇의 x)
- unassigned : min(작업 수, 일할 수 있는 로봇 수) − 쥔 서로 다른 작업 수 (0 미만은 0).
               일할 수 있는 로봇 = 살아 있고 건강한 로봇 + 실행을 시작한 뒤 이탈·건강 저하된 로봇
- stuck      : 아무도 쥐지 않았는데 살아 있는 로봇의 표에서 죽은 로봇이 승자인 작업 수 (고착)
"""

from __future__ import annotations

import dataclasses
import math
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from amr_fleet.negotiation.agent import AgentConfig, CbaaAgent
from amr_fleet.negotiation.channel import Channel
from amr_fleet.negotiation.task_lock import TaskLock

TERMINATIONS = ('local', 'global', 'fixed')
HOLDER = 'holder'      # 이탈 대상 지정: 그 라운드 시작에 작업을 쥔 첫 로봇 (연구 계산과 같은 규칙)


@dataclasses.dataclass
class SimConfig:
    """시뮬레이터 설정."""

    max_rounds: int = 80
    termination: str = 'local'
    round_timeout_ms: float = 150.0
    event_driven: bool = False       # True = 상태가 바뀐 라운드에만 방송 (ACBBA 식 부정 대조군)

    def validate(self) -> None:
        """잘못된 값이면 ValueError."""
        if self.termination not in TERMINATIONS:
            raise ValueError(f'termination 은 {TERMINATIONS} 중 하나: {self.termination!r}')
        if self.max_rounds < 1:
            raise ValueError('max_rounds 는 1 이상')


@dataclasses.dataclass
class RunResult:
    """시뮬레이션 1회 결과."""

    rounds: int
    broadcasts: int
    deliveries: int
    wall_ms: float
    holders: Dict[str, List[str]]          # 작업 → 쥔 로봇들 (로봇 순서)
    assignment: Dict[str, str]             # 작업 → 첫 보유 로봇
    dup: int
    unassigned: int
    stuck: int
    alive: List[str]
    dead: List[str]
    commit_round: Dict[str, int]           # 로봇 → 실행 시작 라운드
    invalidations: int
    lock_denied: int
    dropped: Dict[str, Tuple[int, Optional[str]]] = dataclasses.field(default_factory=dict)
    unhealthy: List[str] = dataclasses.field(default_factory=list)

    def recovery_rounds(self, robot_id: str) -> Optional[int]:
        """
        이탈한 로봇이 쥐고 있던 작업을 다른 로봇이 실행 시작하기까지 걸린 라운드.

        이탈 때 쥔 작업이 없었으면 0, 끝까지 아무도 이어받지 못했으면 None.
        """
        rnd, task = self.dropped.get(robot_id, (None, None))
        if rnd is None or task is None:
            return 0 if rnd is not None else None
        others = [r for r in self.holders.get(task, []) if r != robot_id]
        starts = [self.commit_round[r] for r in others if r in self.commit_round]
        return min(starts) - rnd if starts else None

    @property
    def clean(self) -> bool:
        """중복·미할당 없이 끝났는지."""
        return self.dup == 0 and self.unassigned == 0


def run_case(cost: Sequence[Sequence[float]], robot_ids: Sequence[str], task_ids: Sequence[str],
             agent_config: AgentConfig, channel: Channel, sim: Optional[SimConfig] = None,
             task_lock: Optional[TaskLock] = None,
             dropouts: Optional[Mapping[str, int]] = None,
             health_events: Optional[Sequence[Tuple[int, str, bool]]] = None) -> RunResult:
    """
    한 사례를 돌린다.

    cost[i][j]  : 로봇 i 의 작업 j 비용 [s] (행 순서 = robot_ids, 열 순서 = task_ids = 후보 순서)
    dropouts    : {로봇 id 또는 'holder': 라운드} — 그 라운드 시작부터 송수신을 멈춘다 (실행 중 작업은 그대로)
    health_events: [(라운드, 로봇 id, 건강)] — 그 라운드 시작(입찰 전)에 위치추정 건강 플래그를 바꾼다
    """
    sim = sim or SimConfig()
    sim.validate()
    agent_config.validate()
    ids = list(robot_ids)
    agents: List[CbaaAgent] = []
    for i, rid in enumerate(ids):
        costs = {t: cost[i][j] for j, t in enumerate(task_ids)}
        agents.append(CbaaAgent(rid, ids, task_ids, costs, agent_config, task_lock, now=0))
    alive = [True] * len(ids)
    kills = dict(dropouts or {})
    events = sorted(health_events or [], key=lambda e: e[0])
    k_stable = agent_config.stable_rounds
    broadcasts = deliveries = 0
    wall = 0.0
    commit_round: Dict[str, int] = {}
    dropped: Dict[str, Tuple[int, Optional[str]]] = {}
    recv_changed = [False] * len(ids)
    rnd = 0
    for rnd in range(1, sim.max_rounds + 1):
        _apply_dropouts(kills, rnd, ids, agents, alive, dropped)
        for r, rid, ok in events:
            if r == rnd:
                agents[ids.index(rid)].set_localization_healthy(ok, rnd)
        for i, ag in enumerate(agents):
            if alive[i]:
                ag.start_round(rnd)
        msgs = {}
        snaps = {}
        for k, ag in enumerate(agents):
            if not alive[k]:
                continue
            snaps[k] = ag.snapshot_key()[0]
            if sim.event_driven and rnd > 1 and not ag.table_changed() and not recv_changed[k]:
                continue
            msgs[k] = ag.make_message(rnd)
        round_ms, lost_any = 0.0, False
        for k in sorted(msgs):
            broadcasts += 1
            for i, ag in enumerate(agents):
                if i == k or not alive[i]:
                    continue
                ok, delay = channel.deliver(ids[k], ids[i], rnd)
                if not ok or delay > sim.round_timeout_ms:
                    lost_any = True
                    continue
                round_ms = max(round_ms, delay)
                deliveries += 1
                ag.receive(msgs[k], rnd)
        wall += sim.round_timeout_ms if lost_any else round_ms
        recv_changed = [alive[i] and snaps.get(i) != agents[i].snapshot_key()[0]
                        for i in range(len(ids))]
        for i, ag in enumerate(agents):
            if alive[i] and ag.finish_round(rnd) is not None:
                commit_round[ids[i]] = rnd
        live = [ag for i, ag in enumerate(agents) if alive[i]]
        if sim.termination == 'global':
            if all(ag.stable >= k_stable for ag in live):
                break
        elif sim.termination == 'local':
            if all(ag.executing is not None or (ag.stable >= k_stable and ag.assigned is None)
                   for ag in live):
                break
    res = _summarize(agents, ids, task_ids, alive, rnd, broadcasts, deliveries, wall,
                     commit_round, agent_config.local_commit)
    res.dropped = dropped
    return res


def _apply_dropouts(kills: Dict[str, int], rnd: int, ids: List[str],
                    agents: List[CbaaAgent], alive: List[bool],
                    dropped: Dict[str, Tuple[int, Optional[str]]]) -> None:
    for who, r in list(kills.items()):
        if r != rnd:
            continue
        if who == HOLDER:
            holders = [i for i, ag in enumerate(agents) if alive[i] and ag.assigned is not None]
            victim = holders[0] if holders else 0
        else:
            victim = ids.index(who)
        alive[victim] = False
        ag = agents[victim]
        dropped[ids[victim]] = (rnd, ag.executing if ag.executing is not None else ag.assigned)
        del kills[who]


def _summarize(agents: List[CbaaAgent], ids: List[str], task_ids: Sequence[str],
               alive: List[bool], rnd: int, broadcasts: int, deliveries: int, wall: float,
               commit_round: Dict[str, int], local_commit: bool) -> RunResult:
    holders: Dict[str, List[str]] = {}
    for i, ag in enumerate(agents):
        task = ag.executing if local_commit else (ag.assigned if alive[i] else None)
        if task is not None:
            holders.setdefault(task, []).append(ids[i])
    dup = sum(len(v) - 1 for v in holders.values())
    n_alive = sum(alive)
    n_dead_exec = sum(1 for i, ag in enumerate(agents)
                      if not alive[i] and local_commit and ag.executing is not None)
    # 위치추정 건강이 나쁜 채 끝난 로봇은 입찰할 수 없으므로 일할 수 있는 로봇에서 뺀다
    n_sick_idle = sum(1 for i, ag in enumerate(agents)
                      if alive[i] and not ag.healthy and ag.executing is None)
    want = min(len(task_ids), n_alive - n_sick_idle + n_dead_exec)
    unassigned = max(0, want - len(holders))
    dead_ids = {ids[i] for i in range(len(ids)) if not alive[i]}
    stuck = 0
    for t in task_ids:
        if t in holders:
            continue
        if any(alive[i] and ag.table.get(t) is not None and ag.table[t].winner in dead_ids
               for i, ag in enumerate(agents)):
            stuck += 1
    return RunResult(
        rounds=rnd, broadcasts=broadcasts, deliveries=deliveries, wall_ms=wall,
        holders=holders, assignment={t: v[0] for t, v in holders.items()},
        dup=dup, unassigned=unassigned, stuck=stuck,
        alive=[ids[i] for i in range(len(ids)) if alive[i]], dead=sorted(dead_ids),
        commit_round=commit_round,
        invalidations=sum(ag.n_invalidated for ag in agents),
        lock_denied=sum(ag.n_lock_denied for ag in agents),
        unhealthy=[ids[i] for i, ag in enumerate(agents) if not ag.healthy])


def assignment_indices(result: RunResult, robot_ids: Sequence[str],
                       task_ids: Sequence[str]) -> Dict[int, int]:
    """결과 할당을 {작업 번호: 로봇 번호} 로 바꾼다 (scenario.central_greedy 와 비교용)."""
    r_idx = {r: i for i, r in enumerate(robot_ids)}
    t_idx = {t: j for j, t in enumerate(task_ids)}
    return {t_idx[t]: r_idx[r] for t, r in result.assignment.items()}


def finite_or(value: float, default: float) -> float:
    """비유한 값을 default 로 바꾼다 (지표 계산용)."""
    return value if math.isfinite(value) else default


__all__ = ['HOLDER', 'RunResult', 'SimConfig', 'TERMINATIONS', 'assignment_indices', 'finite_or',
           'run_case']
