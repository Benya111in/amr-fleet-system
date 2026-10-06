"""
로봇 1 대: 운동학, 작업 상태기계, 구역 게이트, 투표 · 협상 참여, 서버 장애 감지.

운동학 (nav2_params.yaml): 경로를 따라 호 길이 s 를 적분한다. v ≤ 구간 최고 속도(1.0 m/s × 잡음),
가속·감속 ≤ 1.0 m/s². 꺾임이 45° 넘는 꼭짓점(rotate_to_heading_min_angle 0.785)에서는 서서 1.5 rad/s 로 제자리 회전.
멈출 점(정지선 · 회전 꼭짓점 · 끝점)까지 남은 거리 d 에서 v ≤ −a·dt + √((a·dt)² + 2·a·d) — 다음 단계에서도 감속
a 로 설 수 있는 이산 상한 — 를 지키고, 위치는 멈출 점을 넘지 않게 자른다.

구역 게이트: 경로 위 구역 방문마다 정지선(차체 외접원이 닿기 전)이 있고, 허가가 없으면 그 앞에 선다.
- 중앙: 서버가 보낸 마지막 보유 구역 집합 (seq 증가분만)
- 투표: ZoneAgent.may_enter('zone:<id>') — 과반 ENTER_ACK 로 만든 HELD
- 없음(유령 기준선): 늘 허가
투표 모드에서 요청은 중앙과 같은 request_chain(요청 거리 · 사슬 간격 · 구역 안 대기 금지)으로 만들고, 차체가 구역을
완전히 빠져나오면(s > s_clear) release 한다.

작업: IDLE → to_pickup → (잠금 대기) → load → to_drop → unload → return → IDLE. "실행 시작" = load 시작
(중복 실행 판정 지점). 하역 뒤에는 대기 자세(스폰 자세)로 돌아가고(task_executor.xml ReturnToWaiting,
launch_utils waiting_pose = 스폰 자세), 복귀 중에는 새 작업을 받지 않는다 (behavior.yaml accept_while_returning:
false, 복귀 중 상태 MOVING → fleet_manager 후보 아님). 협상도 복귀 중에는 하지 않는다.
엄격 협상·폴백 설계는 load 전에 작업 ID 투표 잠금(task:<id>, 별도 ZoneAgent)을 쥐어야 한다. 잠금은 완료 뒤에도 풀지 않는다
(끝난 작업을 다른 로봇이 다시 잠가 실행하지 못하게).
"""

from __future__ import annotations

import math
import random
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple

from amr_fleet.allocation import approach_distance, RobotInfo, travel_time
from amr_fleet.comm_emulator import FLEET
from amr_fleet.distributed.failure_detector import (
    ALIVE, DEAD, DetectorConfig, HeartbeatMonitor, MajoritySuspicion,
)
from amr_fleet.distributed.store import MemoryStore
from amr_fleet.distributed.vote_core import VoteConfig, ZoneAgent
from amr_fleet.fleetsim.config import derive, dwell_s, leg_speed
from amr_fleet.fleetsim.geometry import wrap
from amr_fleet.negotiation.agent import AgentConfig, CbaaAgent
from amr_fleet.negotiation.task_lock import LockResult, LockStatus, OptimisticTaskLock, TaskLock
from amr_fleet.scheduler import TaskScheduler
from amr_fleet.task_schema import TaskSpec
from amr_fleet.traffic_zones import request_chain

V_MAX = 1.0                 # nav2 max_vel_x
ACC = 1.0                   # acc_lim_x = |decel_lim_x|
OMEGA = 1.5                 # rotate_to_heading_angular_vel
ROT_MIN = 0.785             # rotate_to_heading_min_angle
SNAP_M = 1e-3
IDLE, TO_PICKUP, WAIT_LOCK, LOAD, TO_DROP, UNLOAD, RETURN = (
    'idle', 'to_pickup', 'wait_lock', 'load', 'to_drop', 'unload', 'return')
DRIVING = (TO_PICKUP, TO_DROP, RETURN)
RETURN_HYSTERESIS_S = 30.0  # 관찰자 복귀 히스테리시스 (보고서)


class VoteTaskLock(TaskLock):
    """작업 ID 정족수 잠금 = 로봇의 작업용 ZoneAgent (자원 'task:<id>')."""

    def __init__(self, robot: 'Robot'):
        self.robot = robot
        self.denied = 0

    def try_acquire(self, task_id: str, robot_id: str, now: float) -> LockResult:
        rb = self.robot
        if rb.task_agent.may_enter('task:' + task_id):
            return LockResult(LockStatus.GRANTED, robot_id)
        holder = rb.executor_of(task_id)
        if holder is not None or task_id in rb.done:
            self.denied += 1
            rb.cancel_task_lock(task_id)
            return LockResult(LockStatus.DENIED, holder)
        rb.request_task_lock(task_id)
        return LockResult(LockStatus.PENDING)

    def release(self, task_id: str, robot_id: str, now: float) -> None:
        self.robot.cancel_task_lock(task_id)


class Robot:
    """로봇 1 대."""

    def __init__(self, sim, rid: str):
        self.sim = sim
        self.cfg = sim.cfg
        self.arm = sim.cfg.arm
        self.world = sim.world
        self.id = rid
        self.alive = True
        self.place = f'spawn_{rid}'
        self.x, self.y = self.world.places[self.place]
        self.th = math.pi / 2.0                       # fleet_spawn.yaml yaw 1.5708
        self.leg = None
        self.rot_s: List[float] = []
        self.s = 0.0
        self.v = 0.0
        self.vmax = V_MAX
        self.stage = IDLE
        self.task: Optional[TaskSpec] = None
        self.task_via = ''
        self.dwell_until = 0.0
        self.wait_zone: Optional[str] = None
        self.wait_since: Optional[float] = None
        self.max_decel = 0.0
        self.max_speed = 0.0
        # 중앙 교통 허가
        self.allowed: FrozenSet[str] = frozenset()
        self.allowed_seq = 0
        # 투표
        members = self.world.robot_ids
        self.zone_agent: Optional[ZoneAgent] = None
        self.task_agent: Optional[ZoneAgent] = None
        if self.arm.traffic == 'vote':
            self.zone_agent = ZoneAgent.boot(
                rid, VoteConfig(members), MemoryStore(), 0.0,
                rng=random.Random(derive(self.cfg.seed, self.cfg.rep, 'vz', rid)))[0]
        if self.arm.needs_task_lock:
            self.task_agent = ZoneAgent.boot(
                rid, VoteConfig(members), MemoryStore(), 0.0,
                rng=random.Random(derive(self.cfg.seed, self.cfg.rep, 'vt', rid)))[0]
        self.req_chain: FrozenSet[str] = frozenset()
        self.task_req: Optional[str] = None
        # 협상 · 장애 감지
        self.negotiating = self.arm.alloc in ('cbaa_strict', 'cbaa_opt', 'cbaa_event')
        self.cbaa: Optional[CbaaAgent] = None
        self.cbaa_round: Optional[int] = None
        self.cbaa_snap = None
        self.recv_changed = False
        if self.arm.alloc == 'cbaa_opt':
            self.lock: TaskLock = OptimisticTaskLock()
        else:
            self.lock = VoteTaskLock(self)
        self.known: Dict[str, TaskSpec] = {}
        self.done: Set[str] = set()
        self.peer: Dict[str, Tuple[float, dict]] = {}
        self.hb = HeartbeatMonitor(DetectorConfig())
        self.ms = MajoritySuspicion(members)
        self.alive_since: Optional[float] = None
        self.mode_switches: List[Tuple[float, str]] = []
        self.sched = TaskScheduler()
        self.seq = 0
        self.stats: Dict[str, float] = dict(wait_s=0.0, waits=0, lock_wait_s=0.0, rejected=0,
                                            cbaa_tasks=0, central_tasks=0, bcast=0)

    # ------------------------------------------------------------------ 송신 도우미
    def _send(self, now: float, dst: str, kind: str, payload: Any) -> bool:
        return self.sim.net.send(now, self.id, dst, kind, payload)

    def _send_votes(self, now: float, kind: str, msgs) -> None:
        for m in msgs:
            self.sim.net.send(now, self.id, m.dst, kind, m)

    # ------------------------------------------------------------------ 조회
    def idle(self) -> bool:
        return self.task is None and self.stage == IDLE

    def permitted(self, zone_id: str) -> bool:
        """구역 진입 허가 (게이트 기준)."""
        t = self.arm.traffic
        if t == 'none':
            return True
        if t == 'central':
            return zone_id in self.allowed
        return self.zone_agent.may_enter('zone:' + zone_id)

    def contact_zones(self) -> List[str]:
        """차체 외접원이 지금 닿은 구역 (경로 접촉 구간 기준, 경로가 없으면 위치로)."""
        if self.leg is None:
            return self.world.circle_zones(self.x, self.y)
        s = self.s
        return [p.zone_id for p in self.leg.passes if p.s_touch <= s + 1e-9 and s <= p.s_clear]

    def executor_of(self, task_id: str) -> Optional[str]:
        """다른 로봇이 이 작업을 실행 중이라고 아는가 (신선한 비컨 · 협상 표의 잠금)."""
        now = self.sim.now
        for r, (t, b) in sorted(self.peer.items()):
            if r != self.id and b.get('task') == task_id and now - t <= 2.0:
                return r
        if self.cbaa is not None:
            bid = self.cbaa.table.get(task_id)
            if bid is not None and bid.locked and bid.winner != self.id:
                return bid.winner
        return None

    # ------------------------------------------------------------------ 작업 잠금 (엄격)
    def request_task_lock(self, task_id: str) -> None:
        if self.task_agent is None:
            return
        res = 'task:' + task_id
        ag = self.task_agent
        if self.task_req == task_id and (ag.logical is not None or ag.attempt is not None):
            return
        prio = self.known[task_id].priority if task_id in self.known else 0
        self.task_req = task_id
        self._send_votes(self.sim.now, 'vt', ag.request([res], self.sim.now, priority=prio))

    def cancel_task_lock(self, task_id: str) -> None:
        if self.task_agent is None:
            return
        now = self.sim.now
        ag = self.task_agent
        if self.task_req == task_id:
            self._send_votes(now, 'vt', ag.cancel(now))
            self.task_req = None
        res = 'task:' + task_id
        executing = self.task is not None and self.task.task_id == task_id
        if ag.may_enter(res) and not executing:
            self._send_votes(now, 'vt', ag.release([res], now))

    # ------------------------------------------------------------------ 수신
    def on_message(self, now: float, src: str, kind: str, payload) -> None:
        if not self.alive:
            return
        if kind == 'vz':
            self._send_votes(now, 'vz', self.zone_agent.handle(payload, now))
        elif kind == 'vt':
            self._send_votes(now, 'vt', self.task_agent.handle(payload, now))
        elif kind == 'grant':
            seq, zones = payload
            if seq > self.allowed_seq:
                self.allowed_seq = seq
                self.allowed = zones
        elif kind == 'assign':
            self._on_assign(now, payload)
        elif kind == 'hb':
            self.hb.observe(now, *payload)
        elif kind == 'beacon':
            self.peer[src] = (now, payload)
            self.done |= payload['done']
            self.ms.observe(src, now, payload['server_dead'], 0, payload['seq'])
        elif kind == 'cbaa':
            k, msg = payload
            if self.cbaa is not None and k == self.cbaa_round:
                self.cbaa.receive(msg, k)
        elif kind == 'board':
            self.known.setdefault(payload.task_id, payload)

    def _on_assign(self, now: float, spec: TaskSpec) -> None:
        ok = False
        if self.task is not None and self.task.task_id == spec.task_id:
            ok = True
        elif self.idle() and not (self.arm.fallback and self.negotiating) \
                and spec.task_id not in self.done:
            ok = True
            self.known.setdefault(spec.task_id, spec)
            self.start_task(now, spec, 'central')
        else:
            self.stats['rejected'] += 1
        self.sim.net.send_reliable(now, self.id, FLEET, 'assign_resp', (spec.task_id, ok))

    # ------------------------------------------------------------------ 주기 송신
    def state_timer(self, now: float) -> None:
        if not self.alive:
            return
        self.seq += 1
        st = {'seq': self.seq, 'x': self.x, 'y': self.y, 'idle': self.idle(),
              'task': self.task.task_id if self.task is not None else None}
        self._send(now, FLEET, 'state', st)
        self.sim.q.schedule(now + 0.5, self.state_timer)

    def obs_timer(self, now: float) -> None:
        if not self.alive:
            return
        self.seq += 1
        ob = {'seq': self.seq, 'x': self.x, 'y': self.y, 'th': self.th, 'leg': self.leg,
              's': self.s, 'priority': self.task.priority if self.task is not None else -1}
        self._send(now, FLEET, 'obs', ob)
        self.sim.q.schedule(now + 0.1, self.obs_timer)

    def beacon_timer(self, now: float) -> None:
        if not self.alive:
            return
        self.seq += 1
        dead = self.hb.status(now) == DEAD if self.arm.fallback else False
        b = {'seq': self.seq, 'idle': self.idle(), 'server_dead': dead,
             'task': self.task.task_id if self.task is not None else None,
             'done': frozenset(self.done)}
        self.ms.observe(self.id, now, dead, 0, self.seq)
        for r in self.world.robot_ids:
            if r != self.id:
                self._send(now, r, 'beacon', b)
        if self.arm.fallback:
            self._fallback_check(now)
        self.sim.q.schedule(now + 0.5, self.beacon_timer)

    def _fallback_check(self, now: float) -> None:
        if not self.negotiating:
            if self.ms.degraded(now):
                self.negotiating = True
                self.alive_since = None
                self.mode_switches.append((now, 'negotiation'))
            return
        if self.hb.status(now) == ALIVE:
            if self.alive_since is None:
                self.alive_since = now
            elif (now - self.alive_since >= RETURN_HYSTERESIS_S
                  and (self.cbaa is None or self.cbaa.executing is None)):
                self.negotiating = False
                self.cbaa = None
                self.cbaa_round = None
                self.mode_switches.append((now, 'central'))
        else:
            self.alive_since = None

    # ------------------------------------------------------------------ 협상 라운드
    def cbaa_step(self, now: float, k: int) -> None:
        """라운드 k 시작 (sim 이 모든 로봇에 T_b 마다 부른다). 직전 라운드를 끝내고 새 라운드를 연다."""
        if not self.alive or not self.negotiating:
            return
        if self.task is None and self.stage != IDLE:
            self.cbaa_round = None          # 복귀 중: 협상하지 않는다 (돌아오면 새 라운드부터)
            return
        if self.arm.fallback and self.task is not None and (
                self.cbaa is None or self.cbaa.executing is None):
            self.cbaa_round = None          # 폴백 설계: 중앙이 준 작업 중에는 협상에 끼지 않는다 (L_t = 1)
            return
        ag = self.cbaa
        if ag is None:
            cfg = AgentConfig.strict_mode() if self.arm.alloc != 'cbaa_opt' or self.arm.fallback \
                else AgentConfig.optimistic_mode()
            ag = self.cbaa = CbaaAgent(self.id, self.world.robot_ids, [], {}, cfg, self.lock,
                                       now=k)
        elif self.cbaa_round is not None:
            self.recv_changed = self.cbaa_snap != ag.snapshot_key()[0]
            started = ag.finish_round(self.cbaa_round)
            if started is not None and self.task is None:
                self.stats['cbaa_tasks'] += 1
                self.start_task(now, self.known[started], 'cbaa')
        tasks = self._candidates(now)
        if tasks != ag.tasks:
            ag.set_tasks(tasks)
        me = RobotInfo(self.id, self.x, self.y)
        ag.set_costs({t: travel_time(approach_distance(me, self.known[t]), 1.0)
                      for t in tasks if t in self.known})
        ag.start_round(k)
        self.cbaa_round = k
        if (self.arm.alloc == 'cbaa_event' and not self.arm.fallback and k > 1
                and not ag.table_changed() and not self.recv_changed):
            self.cbaa_snap = ag.snapshot_key()[0]
            return
        msg = ag.make_message(k)
        self.cbaa_snap = ag.snapshot_key()[0]
        self.stats['bcast'] += 1
        for r in self.world.robot_ids:
            if r != self.id:
                self._send(now, r, 'cbaa', (k, msg))

    def _candidates(self, now: float) -> List[str]:
        ag = self.cbaa
        mine = ag.executing if ag is not None else None
        execd = set()
        for r, (t, b) in self.peer.items():
            if b.get('task') and now - t <= 2.0:
                execd.add(b['task'])
        keep = []
        if ag is not None:
            for t in ag.tasks:
                bid = ag.table.get(t)
                if bid is not None and bid.locked and t not in self.done and t != mine:
                    keep.append(t)
        pending = [s for tid, s in self.known.items()
                   if tid not in self.done and tid not in execd and tid != mine
                   and tid not in keep]
        pending.sort(key=lambda s: self.sched.sort_key(s, now))
        idle_n = 1 if self.idle() else 0
        for r, (t, b) in self.peer.items():
            if now - t <= 2.0 and b.get('idle'):
                idle_n += 1
        top = [s.task_id for s in pending[:max(1, idle_n)]]
        out = ([mine] if mine else []) + keep + top
        return list(dict.fromkeys(out))

    # ------------------------------------------------------------------ 작업
    def start_task(self, now: float, spec: TaskSpec, via: str) -> None:
        self.task = spec
        self.task_via = via
        if via == 'central':
            self.stats['central_tasks'] += 1
        pick, _ = self.sim.task_docks[spec.task_id]
        if self.arm.needs_task_lock and via == 'central':
            self.request_task_lock(spec.task_id)
        self._send_status(now, 'IN_PROGRESS')
        self.sim.oracle.assigned(spec.task_id, self.id, now, via)
        self._start_leg(now, pick, TO_PICKUP, spec.task_id)

    def _send_status(self, now: float, status: str) -> None:
        if self.arm.central_alloc and self.task is not None:
            self.sim.net.send_reliable(now, self.id, FLEET, 'task_status',
                                       (self.task.task_id, status))

    def _start_leg(self, now: float, goal: str, stage: str, tag: str) -> None:
        self.leg = self.world.leg(self.place, goal)
        self.s = 0.0
        self.v = 0.0
        self.stage = stage
        self.vmax = leg_speed(self.cfg, self.id, tag, stage, V_MAX)
        path = self.leg.path
        self.rot_s = [path.cum[i + 1] for i, a in enumerate(path.turn_angles()) if a > ROT_MIN]

    def _arrive(self, now: float) -> None:
        self.place = self.leg.goal
        if self.stage == TO_PICKUP:
            self.stage = WAIT_LOCK
            self._try_load(now)
        elif self.stage == TO_DROP:
            self.stage = UNLOAD
            self.dwell_until = now + dwell_s(self.cfg, self.task, self.id, 'unload')
        elif self.stage == RETURN:
            self.stage = IDLE

    def _try_load(self, now: float) -> None:
        tid = self.task.task_id
        if self.arm.needs_task_lock and not self.task_agent.may_enter('task:' + tid):
            if self.task_req != tid:
                self.request_task_lock(tid)
            return
        self.stage = LOAD
        self.dwell_until = now + dwell_s(self.cfg, self.task, self.id, 'load')
        self.sim.oracle.load_start(tid, self.id, now)

    def _complete(self, now: float) -> None:
        tid = self.task.task_id
        self.sim.oracle.complete(tid, self.id, now)
        self._send_status(now, 'COMPLETED')
        self.done.add(tid)
        self.task = None
        self.task_req = None if self.task_req == tid else self.task_req
        if self.cbaa is not None:
            self.cbaa.set_tasks([t for t in self.cbaa.tasks if t != tid])
        home = f'spawn_{self.id}'
        if self.place == home:
            self.stage = IDLE
        else:
            self._start_leg(now, home, RETURN, tid + ':return')

    # ------------------------------------------------------------------ 한 단계
    def step(self, now: float, dt: float) -> None:
        """시각 now → now + dt 적분 (now 의 사건은 이미 처리됨)."""
        if not self.alive:
            return
        t1 = now + dt
        if self.zone_agent is not None:
            self._send_votes(now, 'vz', self.zone_agent.tick(now))
        if self.task_agent is not None:
            self._send_votes(now, 'vt', self.task_agent.tick(now))
        st = self.stage
        if st == WAIT_LOCK:
            self.stats['lock_wait_s'] += dt
            self._try_load(now)
        elif st in (LOAD, UNLOAD) and t1 >= self.dwell_until:
            if st == LOAD:
                _, drop = self.sim.task_docks[self.task.task_id]
                self._start_leg(now, drop, TO_DROP, self.task.task_id)
            else:
                self._complete(t1)
        if self.arm.traffic == 'vote':
            self._vote_traffic(now)
        if self.stage in DRIVING:
            self._drive(now, dt)

    def _vote_traffic(self, now: float) -> None:
        ag = self.zone_agent
        contact = set(self.contact_zones())
        auth = {r[5:] for r in ag.authorized() if r.startswith('zone:')}
        leg = self.leg if self.stage in DRIVING else None
        ahead = set()
        if self.leg is not None:
            ahead = {p.zone_id for p in self.leg.passes if p.s_clear >= self.s - 1e-9}
        rel = sorted(z for z in auth if z not in contact and z not in ahead)
        if rel:
            self._send_votes(now, 'vz', ag.release(['zone:' + z for z in rel], now))
        if leg is None:
            return
        chain = request_chain(leg.visits_from(self.s), self.world.approach,
                              self.world.chain_gap_m, occupied=contact)
        want = frozenset('zone:' + v.zone_id for v in chain)
        need = want - ag.authorized()
        if not need:
            self.req_chain = frozenset()
            return
        if want != self.req_chain or (ag.logical is None and ag.attempt is None):
            prio = self.task.priority if self.task is not None else 0
            self.req_chain = want
            self._send_votes(now, 'vz', ag.request(want, now, priority=prio))

    def _barrier(self) -> float:
        s = self.s
        for p in self.leg.passes:
            if p.s_clear < s or p.s_touch <= s + 1e-9 or p.stop_s < s - 1e-6:
                continue
            if not self.permitted(p.zone_id):
                return p.stop_s, p.zone_id
        return math.inf, None

    def _drive(self, now: float, dt: float) -> None:
        path = self.leg.path
        s = self.s
        tgt = path.heading(s, self.th)
        d = wrap(tgt - self.th)
        if abs(d) > 1e-9:
            if abs(d) <= ROT_MIN or self.v > 1e-9:
                self.th = tgt
            else:
                step = OMEGA * dt
                self.th = tgt if abs(d) <= step else wrap(self.th + math.copysign(step, d))
                return
        stop = path.length
        for rs in self.rot_s:
            if rs > s + 1e-9:
                stop = min(stop, rs)
                break
        barrier, bz = self._barrier()
        stop = min(stop, barrier)
        dist = max(0.0, stop - s)
        adt = ACC * dt
        v_cap = -adt + math.sqrt(adt * adt + 2.0 * ACC * dist)
        v_new = max(0.0, min(self.v + adt, self.vmax, v_cap))
        s_new = s + v_new * dt
        if s_new >= stop - SNAP_M or dist <= SNAP_M:
            s_new = stop
            v_end = 0.0
        else:
            v_end = v_new
        dec = (self.v - v_new) / dt
        self.max_decel = max(self.max_decel, dec)
        self.max_speed = max(self.max_speed, (s_new - s) / dt)
        self.v = v_end
        self.s = s_new
        self.x, self.y = path.point(s_new)
        if s_new >= path.length - 1e-9 and self.v == 0.0:
            self.wait_zone = None
            self.wait_since = None
            self._arrive(now + dt)
            return
        if bz is not None and self.v == 0.0 and s_new >= barrier - 1e-9:
            if self.wait_zone != bz:
                self.wait_zone = bz
                self.wait_since = now + dt
                self.stats['waits'] += 1
            self.stats['wait_s'] += dt
        else:
            self.wait_zone = None
            self.wait_since = None

    def crash(self, now: float) -> None:
        """F6: 그 자리에 멈춘다. 송수신 중단, 영속 상태(투표)는 남는다."""
        self.alive = False
        self.v = 0.0


__all__ = ['ACC', 'OMEGA', 'ROT_MIN', 'Robot', 'V_MAX', 'VoteTaskLock']
