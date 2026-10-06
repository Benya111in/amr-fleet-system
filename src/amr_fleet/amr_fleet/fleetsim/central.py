"""
중앙 서버 모형 — fleet_manager 의 할당 의미 + traffic_manager 의 구역 토큰.

노드 코드를 그대로 돌리지 않고(rclpy), 판단 규칙만 같은 순수 함수로 옮긴다. 할당 전략·배치 선택·스케줄 순서·
토큰 부여는 저장소 모듈(allocation · scheduler · traffic_zones)을 **그대로 부른다**.

할당 (fleet_manager_node.py · fleet.yaml)
- 0.5 s 마다: 후보 = robot_state 가 5 s 안에 왔고 lost 아님 · 호출 중 아님 · backoff 아님 · IDLE · 작업 없음.
  대기 작업을 스케줄 순서로 select_batch(k = 후보 수) → allocate(전략) → 작업마다 assign 송신.
- assign 송신이 유실되면 그 로봇을 reject_backoff_s(1 s) 빼고 곧바로 다시 할당한다.
- 응답(유실 없음, 지연만)이 assign_timeout_s(3 s) 안에 없으면 보류, 그 뒤 assign_reconcile_s(2 s) 더 지나 받은
  robot_state 에도 작업이 없으면 다시 할당. robot_state 에 작업이 보이면 시작으로 확정.
- robot_state 가 robot_state_timeout_s(5 s) 넘게 끊기면 lost: 진행 작업 FAILED(robot_lost), 재시도 0.
- task_status(COMPLETED)는 신뢰 경로.

교통 (traffic_manager*.py · traffic.yaml)
- 로봇 관측(자세 · 경로 · 진행량, 10 Hz, 모의 링크)으로 2 Hz 마다 request_chain → ZoneTokenManager.update.
  관측이 3 s 넘게 끊긴 로봇은 frozen(보유 유지), 5 s 넘으면 lost(마지막 몸체가 걸친 구역만 유지).
- 결과(로봇별 보유 구역)를 2 Hz 로 로봇에 보낸다 (상태 전송, 모의 링크). 로봇은 마지막 메시지의 보유 구역에만 들어간다.
- 교차 예측 hold · 교착 해소(포켓 · 우회)는 넣지 않는다 (p4.md 한계).
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Set, Tuple

from amr_fleet.allocation import (
    allocate, AllocationStrategy, approach_distance, create_strategy, RobotInfo, select_batch,
    travel_time,
)
from amr_fleet.comm_emulator import FLEET
from amr_fleet.negotiation.scenario import central_greedy
from amr_fleet.scheduler import TaskScheduler
from amr_fleet.task_schema import TaskSpec
from amr_fleet.traffic_zones import request_chain, TokenRequest, ZoneTokenManager

PENDING, INFLIGHT, IN_PROGRESS, COMPLETED, FAILED = (
    'PENDING', 'INFLIGHT', 'IN_PROGRESS', 'COMPLETED', 'FAILED')


class SsiStrategy(AllocationStrategy):
    """중앙 순차 단일 품목 경매(SSI) = negotiation.scenario.central_greedy (C3 비교군)."""

    name = 'ssi'

    def assign(self, tasks: Sequence[TaskSpec], robots: Sequence[RobotInfo]) -> Dict[str, str]:
        if not tasks or not robots:
            return {}
        cost = [[travel_time(approach_distance(r, t), self.nominal_speed) for t in tasks]
                for r in robots]
        return {tasks[j].task_id: robots[i].robot_id for j, i in central_greedy(cost).items()}


def make_strategy(name: str) -> AllocationStrategy:
    """현행 레지스트리 전략 또는 SSI."""
    if name == 'ssi':
        return SsiStrategy(1.0)
    return create_strategy(name, nominal_speed=1.0)


@dataclasses.dataclass
class _RobotView:
    state: Optional[dict] = None
    last_seen: float = -1e9
    lost: bool = False
    inflight: Optional[str] = None
    backoff_until: float = -1e9
    load: int = 0
    obs: Optional[dict] = None
    obs_rx: float = -1e9
    grant_seq: int = 0


@dataclasses.dataclass
class _Dispatch:
    robot: str
    t_cmd: float
    timeout_at: float
    sent: bool
    unconfirmed: bool = False


class CentralServer:
    """fleet_manager + traffic_manager 를 한 프로세스로 (F1 은 둘 다 죽는다)."""

    name = FLEET

    def __init__(self, sim, strategy: Optional[str], traffic: bool):
        self.sim = sim
        self.world = sim.world
        self.cfg = sim.cfg
        self.alive = True
        self.strategy = make_strategy(strategy) if strategy else None
        self.traffic = traffic
        self.robots: Dict[str, _RobotView] = {r: _RobotView() for r in self.world.robot_ids}
        self.tasks: Dict[str, Tuple[TaskSpec, str, Optional[str]]] = {}   # id → (spec, 상태, 로봇)
        self.sched = TaskScheduler()
        self.inflight: Dict[str, _Dispatch] = {}
        self.sent: Dict[str, Set[str]] = {}
        self.tm = ZoneTokenManager(self.world.same_direction, max_bypass=2)
        self.hb_seq = 0
        self.stats: Dict[str, int] = dict(dispatch=0, dispatch_lost=0, abort=0, robot_lost=0,
                                          failed_robot_lost=0, late_start=0, rejected=0)
        self._allocating = False
        self._realloc = False

    # ------------------------------------------------------------------ 타이머
    def start(self, q) -> None:
        """주기 타이머를 건다 (위상: 할당 0.0, 교통 0.25, 하트비트 0.1)."""
        if self.strategy is not None:
            q.schedule(0.5, self._alloc_timer)
        if self.traffic:
            q.schedule(0.25, self._traffic_timer)
        q.schedule(0.1, self._hb_timer)

    def crash(self) -> None:
        """F1: 서버가 죽는다 (상태는 사라진다고 본다 — 재기동 없음)."""
        self.alive = False

    def _hb_timer(self, now: float) -> None:
        if not self.alive:
            return
        self.hb_seq += 1
        for r in self.world.robot_ids:
            self.sim.net.send(now, FLEET, r, 'hb', (1, self.hb_seq))
        self.sim.q.schedule(now + 0.5, self._hb_timer)

    # ------------------------------------------------------------------ 수신
    def on_message(self, now: float, src: str, kind: str, payload) -> None:
        """서버 수신 함수."""
        if not self.alive:
            return
        if kind == 'board':
            spec: TaskSpec = payload
            if spec.task_id not in self.tasks:
                self.tasks[spec.task_id] = (spec, PENDING, None)
                self.sched.push(spec)
        elif kind == 'state':
            self._on_state(now, src, payload)
        elif kind == 'obs':
            r = self.robots[src]
            if r.obs is None or payload['seq'] > r.obs['seq']:
                r.obs = payload
                r.obs_rx = now
        elif kind == 'assign_resp':
            self._on_assign_resp(now, src, payload)
        elif kind == 'task_status':
            self._on_task_status(now, src, payload)

    def _on_state(self, now: float, rid: str, st: dict) -> None:
        r = self.robots[rid]
        if r.state is not None and st['seq'] <= r.state['seq']:
            return
        r.state = st
        r.last_seen = now
        if r.lost:
            r.lost = False
        cur = st.get('task')
        if cur:
            if r.inflight == cur:
                self._confirm(now, cur, rid)
            else:
                rec = self.tasks.get(cur)
                if (rec is not None and rec[1] == PENDING and cur not in self.inflight
                        and rid in self.sent.get(cur, ()) and self._active_task(rid) is None):
                    self.stats['late_start'] += 1
                    self._start_task(cur, rid)

    def _on_assign_resp(self, now: float, rid: str, payload) -> None:
        task_id, ok = payload
        d = self.inflight.get(task_id)
        if d is None or d.robot != rid:
            if ok:                                   # 늦은 수락: 대기 중이면 그 로봇에서 시작
                rec = self.tasks.get(task_id)
                if rec is not None and rec[1] == PENDING and self._active_task(rid) is None:
                    self.stats['late_start'] += 1
                    self._start_task(task_id, rid)
            return
        self._clear_inflight(task_id)
        if not ok:
            self.stats['rejected'] += 1
            self._abort(now, task_id, rid)
            return
        rec = self.tasks.get(task_id)
        if rec is not None and rec[1] == PENDING:
            self._start_task(task_id, rid)

    def _on_task_status(self, now: float, rid: str, payload) -> None:
        task_id, status = payload
        rec = self.tasks.get(task_id)
        if rec is None:
            return
        spec, st, robot = rec
        if status == COMPLETED and st in (PENDING, IN_PROGRESS):
            self.sched.remove(task_id)
            self._clear_inflight(task_id)
            self.tasks[task_id] = (spec, COMPLETED, rid)

    # ------------------------------------------------------------------ 할당
    def _active_task(self, rid: str) -> Optional[str]:
        for tid, (_, st, robot) in self.tasks.items():
            if st == IN_PROGRESS and robot == rid:
                return tid
        return None

    def _start_task(self, task_id: str, rid: str) -> None:
        spec, st, _ = self.tasks[task_id]
        if st != PENDING:
            return
        self.sched.remove(task_id)
        self.tasks[task_id] = (spec, IN_PROGRESS, rid)
        self.robots[rid].load += 1

    def _confirm(self, now: float, task_id: str, rid: str) -> None:
        d = self.inflight.get(task_id)
        if d is None or d.robot != rid:
            return
        self._clear_inflight(task_id)
        rec = self.tasks.get(task_id)
        if rec is not None and rec[1] == PENDING:
            self._start_task(task_id, rid)

    def _clear_inflight(self, task_id: str) -> None:
        d = self.inflight.pop(task_id, None)
        if d is not None and self.robots[d.robot].inflight == task_id:
            self.robots[d.robot].inflight = None

    def _abort(self, now: float, task_id: str, rid: str) -> None:
        self._clear_inflight(task_id)
        self.robots[rid].backoff_until = now + 1.0          # reject_backoff_s
        self.stats['abort'] += 1
        self._allocate(now)

    def _alloc_timer(self, now: float) -> None:
        if not self.alive:
            return
        self._check_lost(now)
        self._check_timeouts(now)
        self._allocate(now)
        self.sim.q.schedule(now + 0.5, self._alloc_timer)

    def _check_lost(self, now: float) -> None:
        for rid, r in self.robots.items():
            if r.lost or r.state is None or now - r.last_seen <= 5.0:
                continue
            r.lost = True
            self.stats['robot_lost'] += 1
            for tid, (spec, st, robot) in list(self.tasks.items()):
                if st == IN_PROGRESS and robot == rid:           # max_task_retries 0 → 최종 실패
                    self.tasks[tid] = (spec, FAILED, rid)
                    self.stats['failed_robot_lost'] += 1
            if r.inflight is not None:
                self._clear_inflight(r.inflight)

    def _check_timeouts(self, now: float) -> None:
        for task_id, d in list(self.inflight.items()):
            if now <= d.timeout_at:
                continue
            if not d.unconfirmed:
                d.unconfirmed = True
                continue
            r = self.robots[d.robot]
            cur = r.state.get('task') if r.state is not None else None
            if r.state is not None and r.last_seen >= d.timeout_at + 2.0 and cur != task_id:
                self._abort(now, task_id, d.robot)

    def _candidates(self, now: float) -> List[RobotInfo]:
        out = []
        for rid in self.world.robot_ids:
            r = self.robots[rid]
            st = r.state
            if (st is None or r.lost or now - r.last_seen > 5.0 or r.inflight is not None
                    or now < r.backoff_until or not st['idle'] or st.get('task')
                    or self._active_task(rid) is not None):
                continue
            out.append(RobotInfo(rid, st['x'], st['y'], load=r.load))
        return out

    def _allocate(self, now: float) -> None:
        if self._allocating:
            self._realloc = True
            return
        self._allocating = True
        try:
            self._realloc = True
            while self._realloc:
                self._realloc = False
                self._allocate_once(now)
        finally:
            self._allocating = False

    def _allocate_once(self, now: float) -> None:
        robots = self._candidates(now)
        if not robots:
            return
        pending = [t for t in self.sched.ordered(now) if t.task_id not in self.inflight]
        batch = select_batch(pending, robots, 0)
        if not batch:
            return
        result = allocate(self.strategy, batch, robots)
        for task_id, rid in result.assignments.items():
            self._dispatch(now, task_id, rid)

    def _dispatch(self, now: float, task_id: str, rid: str) -> None:
        spec = self.tasks[task_id][0]
        self.stats['dispatch'] += 1
        ok = self.sim.net.send(now, FLEET, rid, 'assign', spec)
        self.inflight[task_id] = _Dispatch(rid, now, now + 3.0, ok)
        self.robots[rid].inflight = task_id
        if not ok:
            self.stats['dispatch_lost'] += 1
            self._abort(now, task_id, rid)
        else:
            self.sent.setdefault(task_id, set()).add(rid)

    # ------------------------------------------------------------------ 교통
    def _traffic_timer(self, now: float) -> None:
        if not self.alive:
            return
        self._traffic_update(now)
        self.sim.q.schedule(now + 0.5, self._traffic_timer)

    def _traffic_update(self, now: float) -> None:
        w = self.world
        requests: Dict[str, TokenRequest] = {}
        occupancy: Dict[str, Set[str]] = {}
        upcoming: Dict[str, Set[str]] = {}
        directions: Dict[str, Dict[str, Optional[Tuple[float, float]]]] = {}
        frozen: Set[str] = set()
        for rid in w.robot_ids:
            r = self.robots[rid]
            ob = r.obs
            if ob is None:
                continue
            age = now - r.obs_rx
            body = set(w.circle_zones(ob['x'], ob['y']))
            occupancy[rid] = body
            leg = ob['leg']
            if age > 5.0:                          # lost: 마지막 몸체 구역만 남긴다
                upcoming[rid] = set(body)
                continue
            s = ob['s']
            ahead = set()
            if leg is not None:
                ahead = {p.zone_id for p in leg.passes if p.s_clear >= s - 1e-9}
                directions[rid] = dict(w.directions(leg))
            upcoming[rid] = ahead | body
            if age > 3.0:                          # stale: 보유 유지, 새 요청 없음
                frozen.add(rid)
                continue
            if leg is None:
                continue
            chain = request_chain(leg.visits_from(s), w.approach, w.chain_gap_m, occupied=body)
            if chain:
                zones = tuple(v.zone_id for v in chain)
                dirs = {z: directions[rid].get(z) for z in zones}
                requests[rid] = TokenRequest(rid, zones, dirs,
                                             priority=int(ob.get('priority', -1)))
        self.tm.update(now, requests, occupancy, upcoming, directions, frozen)
        for rid in w.robot_ids:
            r = self.robots[rid]
            r.grant_seq += 1
            held = frozenset(self.tm.held_by(rid))
            self.sim.net.send(now, FLEET, rid, 'grant', (r.grant_seq, held))


__all__ = ['CentralServer', 'SsiStrategy', 'make_strategy']
