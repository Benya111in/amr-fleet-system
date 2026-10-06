"""
로봇 1 대의 분산 플릿 에이전트 코어 (보너스 4 P3) — rclpy 비의존 순수 Python.

묶는 것 (p3.md §1):
- 투표 코어 두 채널: zone(구역 · 포켓 사슬) · task(작업 ID 잠금). 채널마다 ZoneAgent 하나 · 영속 파일 하나.
  한 채널에 섞으면 요청자의 "활성 시도 하나" 때문에 구역 요청이 작업 잠금 시도를 포기시키고, 투표자의
  "같은 로봇의 더 새 요청 → 옛 잠정 표 회수" 가 다른 종류의 표를 회수한다.
- 장애 감지: 서버 하트비트 sim-time 감시(SUSPECT/DEAD) + Lifeguard 자기 점검 + 비컨 과반 의심.
- 모드 상태기 (modes.py), 에포크 펜스 (EpochFence — 하트비트 · 중앙 명령 모두 거른다, 따로 영속 파일).
- CBAA 협상 (DEGRADED 에서만, 라운드 = sim round_s), 작업 ID 잠금 VoteTaskLock (hold_enter + commit).
- 복제 WFG (비컨의 out-edge → 같은 Tarjan → 같은 희생자, 진단만 — 해소 동작은 P5).
- 가짜 실행기 (선택, 시험 · 빠른 계층용).

입력(메시지 · 시각)을 받으면 보낼 것을 내부 송신함에 쌓고, 호출자(ROS 노드)가 drain() 으로 꺼내 보낸다.
안전에 관한 규칙은 투표 코어에 있다 — 이 파일은 그 코어를 부르는 순서와 모드별 허용만 정한다.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

from amr_fleet.agent.modes import (
    DEGRADED, MODE_CODES, ModeConfig, ModeMachine, SERVER_CODES,
)
from amr_fleet.agent.sim_exec import SimExecConfig, SimExecutor
from amr_fleet.agent.task_lock_vote import TASK_PREFIX, task_resource, VoteTaskLock
from amr_fleet.deadlock import EDGE_TOKEN
from amr_fleet.distributed.failure_detector import (
    DetectorConfig, EpochFence, HeartbeatMonitor, LocalHealth, MajoritySuspicion,
)
from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.ordering import Stamp
from amr_fleet.distributed.store import Store
from amr_fleet.distributed.vote_core import VoteConfig, ZoneAgent
from amr_fleet.distributed.wfg import (
    OutEdge, ProgressConfirmer, ReplicatedWfg, select_victim, WaitInfo,
)
from amr_fleet.negotiation.agent import AgentConfig as NegConfig, CbaaAgent
from amr_fleet.negotiation.bids import StateMessage

CH_ZONE = 0          # amr_msgs/FleetVote.CHANNEL_ZONE
CH_TASK = 1          # amr_msgs/FleetVote.CHANNEL_TASK
CHANNELS = (CH_ZONE, CH_TASK)

ROLE_ACTIVE = 0      # amr_msgs/FleetHeartbeat.ROLE_*
ROLE_OBSERVER = 1

OP_ACQUIRE = 0       # amr_msgs/srv/ZoneCommand.OP_*
OP_RELEASE = 1
OP_CANCEL = 2
OP_COMMIT_TASK = 3

GATE_NONE = 'none'          # 해제 요청을 곧바로 RELEASE (몸체 판정 없음 — 시험 · 기하가 없을 때)
GATE_ZONE_MAP = 'zone_map'  # 위치추정 건강 + 몸체(원판)가 구역 다각형 밖일 때만 RELEASE (판정 불가면 보류)
GATES = (GATE_NONE, GATE_ZONE_MAP)


@dataclasses.dataclass(frozen=True)
class CoreConfig:
    """에이전트 파라미터 (시간은 sim 초)."""

    robot_id: str
    members: Tuple[str, ...]
    zone_vote: VoteConfig = None                  # None → VoteConfig(members) 기본값
    task_vote: VoteConfig = None
    task_hold_s: float = 60.0                     # 작업 잠정 표 기한 (이동 시간보다 길게 — 활성 파라미터)
    detector: DetectorConfig = DetectorConfig()
    modes: ModeConfig = ModeConfig()
    beacon_period_s: float = 0.5                  # 보고서 2 Hz
    beacon_ttl_s: float = 2.0
    round_s: float = 0.2                          # CBAA 라운드 T_b (f3 제안)
    nego: NegConfig = None                        # None → NegConfig.strict_mode() (배타 잠금 + K=3)
    nominal_speed: float = 1.0                    # 입찰 비용 = 거리 / 공칭 속도 (travel_time 과 같다)
    release_gate: str = GATE_NONE
    body_radius_m: float = 0.36                   # 풋프린트 외접 반경
    release_margin_m: float = 0.1
    accept_unfenced: bool = True                  # epoch 0 명령(옛 구성)을 펜스 없이 받는다 (p0.md §4)
    sim_exec: Optional[SimExecConfig] = None      # 가짜 실행기 (None = 실제 실행기가 따로 있다)

    def __post_init__(self):
        members = tuple(sorted(set(self.members)))
        if self.robot_id not in members:
            raise ValueError(f'{self.robot_id!r} 가 멤버 {members} 에 없다')
        object.__setattr__(self, 'members', members)
        if self.zone_vote is None:
            object.__setattr__(self, 'zone_vote', VoteConfig(members))
        if self.task_vote is None:
            object.__setattr__(self, 'task_vote', VoteConfig(members))
        for vc in (self.zone_vote, self.task_vote):
            if tuple(vc.members) != members:
                raise ValueError('투표 설정의 멤버가 에이전트 멤버와 다르다')
        if self.nego is None:
            object.__setattr__(self, 'nego', NegConfig.strict_mode())
        if self.release_gate not in GATES:
            raise ValueError(f'release_gate 는 {GATES} 중 하나: {self.release_gate!r}')
        for name in ('task_hold_s', 'beacon_period_s', 'beacon_ttl_s', 'round_s', 'nominal_speed'):
            if not getattr(self, name) > 0:
                raise ValueError(f'{name} 는 양수여야 한다')


@dataclasses.dataclass(frozen=True)
class BoardTask:
    """작업 게시판의 작업 1 건 (협상에 필요한 것만)."""

    task_id: str
    priority: int = 0
    pickup_x: float = 0.0
    pickup_y: float = 0.0
    robot_id: str = ''                            # 고정 로봇 ('' = 누구나)


@dataclasses.dataclass(frozen=True)
class Beacon:
    """비컨 (amr_msgs/AgentBeacon 과 같은 필드)."""

    robot_id: str
    incarnation: int
    seq: int
    stamp: float
    vote_epoch: int = 0
    fence_epoch: int = 0
    fence_leader: str = ''
    mode: str = 'NORMAL'
    server_status: str = 'UNKNOWN'
    server_dead: bool = False
    x: float = math.nan
    y: float = math.nan
    yaw: float = math.nan
    battery: float = 100.0
    localization_healthy: bool = True
    current_task_id: str = ''
    picked_up: bool = False
    held: Tuple[str, ...] = ()
    wait_edges: Tuple[OutEdge, ...] = ()
    priority: int = 0
    request_stamp: Optional[float] = None
    remaining_m: Optional[float] = None


@dataclasses.dataclass
class Outbox:
    """한 번의 처리에서 나온 송신 거리."""

    votes: List[Tuple[int, VoteMsg]] = dataclasses.field(default_factory=list)
    beacon: Optional[Beacon] = None
    neg: Optional[StateMessage] = None
    events: List[Dict[str, Any]] = dataclasses.field(default_factory=list)
    state_changed: bool = False


@dataclasses.dataclass
class StateView:
    """amr_msgs/AgentState 로 옮길 상태."""

    robot_id: str
    mode: str
    mode_reason: str
    server_status: str
    vote_epoch: int
    fence_epoch: int
    fence_leader: str
    held: Tuple[str, ...]
    requested: Tuple[str, ...]
    zone_phase: str
    denied: Tuple[str, ...]
    current_task_id: str
    task_phase: str
    deadlock_members: Tuple[str, ...]
    deadlock_victim: str
    low_epoch_rejected: int

    def key(self) -> tuple:
        """바뀌었는지 비교용 (발행 빈도 조절)."""
        return dataclasses.astuple(self)


def disc_clear_of_polygon(x: float, y: float, radius: float, polygon) -> bool:
    """반경 radius 원판이 다각형과 겹치지 않나 (안에 있지 않고 모든 변에서 radius 보다 멀다)."""
    pts = [(float(p[0]), float(p[1])) for p in polygon]
    n = len(pts)
    inside = False
    for i in range(n):
        (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if xc > x:
                inside = not inside
    if inside:
        return False
    for i in range(n):
        (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        if math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)) < radius:
            return False
    return True


class FleetAgentCore:
    """로봇 1 대의 에이전트. boot() 로 만든다."""

    def __init__(self, cfg: CoreConfig, zone: ZoneAgent, task: ZoneAgent, fence: EpochFence,
                 now: float, zone_polygons: Optional[Mapping[str, Any]] = None):
        self.cfg = cfg
        self.me = cfg.robot_id
        self.now = float(now)
        self.zone = zone
        self.task = task
        self.lock = VoteTaskLock(task, lambda: self.now, hold_s=cfg.task_hold_s)
        self.fence = fence
        self.health = LocalHealth(cfg.detector)
        self.monitor = HeartbeatMonitor(cfg.detector, self.health)
        self.suspicion = MajoritySuspicion(cfg.members, beacon_ttl_s=cfg.beacon_ttl_s)
        self.modes = ModeMachine(cfg.modes, now)
        self.wfg = ReplicatedWfg(ttl_s=cfg.beacon_ttl_s)
        self.confirmer = ProgressConfirmer()
        self.zone_polygons = dict(zone_polygons or {})   # 'zone:<id>' → 다각형 (zone_map 게이트)
        self.executor = SimExecutor(cfg.sim_exec) if cfg.sim_exec is not None else None
        # 관측
        self.pose: Optional[Tuple[float, float, float]] = None
        self.loc_healthy = True
        self.battery = 100.0
        self.server_active = True
        self.server_leader = ''
        self.peers: Dict[str, Beacon] = {}
        self.board: Dict[str, BoardTask] = {}
        self.taken: Set[str] = set()                   # 어딘가에서 시작 · 잠긴 작업 (단조 증가 — gossip)
        self.done: Set[str] = set()
        # 진행
        self.cbaa: Optional[CbaaAgent] = None
        self._round: Optional[int] = None
        self._beacon_seq = 0
        self._last_beacon: Optional[float] = None
        self.pending_release: Set[str] = set()
        self._boot_unknown_inside = False             # 부팅 때 몸체 위치를 몰라 보유를 모두 남겼다
        self._wait_since: Optional[float] = None
        self._wait_priority = 0
        self.central_task = ''                        # NORMAL 에서 프록시로 받은 작업
        self.low_epoch_rejected = 0
        self.deadlock: Tuple[Tuple[str, ...], str] = ((), '')
        self._out = Outbox()
        self.counters: Dict[str, int] = {}

    # ------------------------------------------------------------------ 기동
    @classmethod
    def boot(cls, cfg: CoreConfig, zone_store: Store, task_store: Store, fence_store: Store,
             now: float, inside_zones: Optional[Iterable[str]] = None,
             zone_polygons: Optional[Mapping[str, Any]] = None) -> 'FleetAgentCore':
        """
        영속 상태에서 기동한다.

        inside_zones = 몸체가 걸친 구역 자원. None(모름)이면 디스크의 구역 보유를 모두 남긴다 (fail-closed —
        자세를 받고 해제 게이트가 몸체 밖을 확인한 뒤에 푼다). 작업 잠금은 확정 뒤 풀지 않으므로 늘 모두 남긴다.
        """
        unknown = inside_zones is None
        if unknown:
            try:
                inside_zones = set().union(*ZoneAgent.held_in_state(zone_store.load()).values())
            except Exception:  # noqa: B902 — 손상은 boot() 가 기억 상실로 처리한다
                inside_zones = set()
        try:
            keep_tasks = set().union(*ZoneAgent.held_in_state(task_store.load()).values())
        except Exception:  # noqa: B902
            keep_tasks = set()
        zone, zout = ZoneAgent.boot(cfg.robot_id, cfg.zone_vote, zone_store, now,
                                    inside=set(inside_zones))
        task, tout = ZoneAgent.boot(cfg.robot_id, cfg.task_vote, task_store, now,
                                    inside=keep_tasks)
        fence = EpochFence(fence_store)
        core = cls(cfg, zone, task, fence, now, zone_polygons)
        core._boot_unknown_inside = unknown and bool(inside_zones)
        core._push(CH_ZONE, zout)
        core._push(CH_TASK, tout)
        for res in keep_tasks:
            if res.startswith(TASK_PREFIX):
                core.taken.add(res[len(TASK_PREFIX):])
        core._note('boot', incarnation=zone.incarnation, kept_zones=sorted(inside_zones or ()),
                   kept_tasks=sorted(keep_tasks))
        return core

    # ------------------------------------------------------------------ 송신함
    def _push(self, channel: int, msgs: Iterable[VoteMsg]) -> None:
        self._out.votes.extend((channel, m) for m in msgs)

    def _note(self, event: str, **kw: Any) -> None:
        ev = {'event': event, 't': self.now, 'robot': self.me}
        ev.update(kw)
        self._out.events.append(ev)
        self.counters[event] = self.counters.get(event, 0) + 1

    def drain(self) -> Outbox:
        """쌓인 송신 거리를 꺼낸다."""
        self._push(CH_TASK, self.lock.drain())
        out, self._out = self._out, Outbox()
        return out

    # ------------------------------------------------------------------ 입력: 투표
    def on_votes(self, channel: int, msgs: Sequence[VoteMsg], now: float) -> None:
        """받은 묶음 중 내 앞 메시지를 처리한다."""
        self.now = float(now)
        agent = self.zone if channel == CH_ZONE else self.task if channel == CH_TASK else None
        if agent is None:
            return
        for msg in msgs:
            if msg.dst != self.me:
                continue
            self._push(channel, agent.handle(msg, now))

    # ------------------------------------------------------------------ 입력: 서버 · 비컨 · 협상
    def on_heartbeat(self, epoch: int, leader_id: str, seq: int, role: int, now: float) -> bool:
        """서버 하트비트. 펜스보다 낮은 (epoch, leader_id) 는 생존 신호로 세지 않는다."""
        self.now = float(now)
        if not self._fence_ok(int(epoch), str(leader_id), what='heartbeat'):
            return False
        ok = self.monitor.observe(now, int(epoch), int(seq))
        if ok:
            self.server_active = int(role) == ROLE_ACTIVE
            self.server_leader = str(leader_id)
        return ok

    def _fence_ok(self, epoch: int, leader_id: str, what: str) -> bool:
        if epoch == 0:
            if self.cfg.accept_unfenced:
                return True
            self.low_epoch_rejected += 1
            self._note('fence_reject', what=what, epoch=epoch, leader=leader_id, why='unfenced')
            return False
        if self.fence.accept(epoch, leader_id):
            return True
        self.low_epoch_rejected += 1
        self._note('fence_reject', what=what, epoch=epoch, leader=leader_id,
                   max=list(self.fence.max) if self.fence.max else None)
        return False

    def on_beacon(self, b: Beacon, now: float) -> None:
        """다른 로봇의 비컨."""
        self.now = float(now)
        if b.robot_id == self.me or b.robot_id not in self.cfg.members:
            return
        if not self.suspicion.observe(b.robot_id, now, b.server_dead, b.incarnation, b.seq):
            return                                    # 옛 비컨
        self.peers[b.robot_id] = b
        self.wfg.update(WaitInfo(b.robot_id, b.incarnation, b.seq, tuple(b.wait_edges),
                                 b.priority, None if b.request_stamp is None
                                 else Stamp.coerce(b.request_stamp), b.remaining_m), now)
        for res in b.held:
            if res.startswith(TASK_PREFIX):
                self.taken.add(res[len(TASK_PREFIX):])
        if b.current_task_id:
            self.taken.add(b.current_task_id)

    def on_neg(self, msg: StateMessage, now: float) -> None:
        """다른 로봇의 협상 상태 (DEGRADED 에서만 병합)."""
        self.now = float(now)
        if self.cbaa is None or not self.modes.negotiates() or self._round is None:
            return
        if msg.sender == self.me or msg.sender not in self.cfg.members:
            return
        self.cbaa.receive(msg, float(self._round))

    def on_board(self, t: BoardTask) -> None:
        """작업 게시판 게시물 (같은 id 는 처음 것만 — TaskLedger 의 멱등 규칙과 같은 효과)."""
        self.board.setdefault(t.task_id, t)

    def set_pose(self, x: float, y: float, yaw: float, healthy: bool = True,
                 battery: Optional[float] = None) -> None:
        """자세 · 위치추정 건강 (odometry/filtered_map, localization/lost)."""
        self.pose = (float(x), float(y), float(yaw))
        self.loc_healthy = bool(healthy)
        if battery is not None:
            self.battery = float(battery)
        if self.cbaa is not None:
            self.cbaa.set_localization_healthy(self.loc_healthy, float(self._round or 0))

    # ------------------------------------------------------------------ 입력: BT 명령 · 중앙 명령
    def command(self, op: int, resources: Sequence[str], priority: int, now: float
                ) -> Tuple[bool, str]:
        """BT → 에이전트 (ZoneCommand). 허가 여부는 상태(held)로만 알린다."""
        self.now = float(now)
        res = [str(r) for r in resources]
        if op == OP_ACQUIRE:
            if not res:
                return False, 'empty'
            if any(r.startswith(TASK_PREFIX) for r in res):
                return False, '작업 잠금은 COMMIT_TASK 로'
            if not self.modes.allows_new_entry():
                return False, f'mode {self.modes.mode}'
            want = frozenset(res) - self.zone.authorized()
            if not want:
                return True, 'already held'
            if want == self.zone.requested():
                return True, 'in progress'           # 멱등 — 재요청이 나이(since)를 잃지 않게
            self.pending_release -= set(res)
            self._push(CH_ZONE, self.zone.request(sorted(want), now, priority=int(priority)))
            self._wait_since, self._wait_priority = self.now, int(priority)
            self._note('zone_request', resources=sorted(want), priority=int(priority))
            return True, 'requested'
        if op == OP_RELEASE:
            held = self.zone.authorized()
            target = (set(res) & held) if res else set(held)
            self.pending_release |= target
            self._note('zone_release_request', resources=sorted(target))
            self._process_releases()
            return True, f'release {len(target)}'
        if op == OP_CANCEL:
            self._push(CH_ZONE, self.zone.cancel(now))
            self._wait_since = None
            return True, 'cancelled'
        if op == OP_COMMIT_TASK:
            if len(res) != 1:
                return False, '작업 ID 하나'
            tid = res[0][len(TASK_PREFIX):] if res[0].startswith(TASK_PREFIX) else res[0]
            phase = self.lock.commit(tid)
            self._note('task_commit_request', task_id=tid, phase=phase)
            return True, phase
        return False, f'모르는 op {op}'

    def check_assignment(self, task_id: str, epoch: int, leader_id: str, now: float
                         ) -> Tuple[bool, str]:
        """
        중앙 assign_task 펜싱 (실행기 앞 프록시가 부른다). 거부면 실행기까지 가지 않는다.

        순서: 에포크 펜스(낮은 (epoch, leader_id) 거부) → 모드(NORMAL 만) → 작업 잠금이 이미 남에게 확정.
        받으면 그 작업의 잠정 잠금을 시작한다 (픽업 직전 COMMIT_TASK 로 확정).
        """
        self.now = float(now)
        if not self._fence_ok(int(epoch), str(leader_id), what='assign_task'):
            return False, f'stale_epoch ({epoch}, {leader_id}) < {self.fence.max}'
        if not self.modes.accepts_central_tasks():
            return False, f'mode {self.modes.mode}'
        if self.lock.is_denied(task_id) or (task_id in self.taken and task_id != self.central_task
                                            and not self.lock.is_held(task_id)):
            return False, 'task_taken'
        self.lock.hold(task_id)
        self.central_task = task_id
        self._note('central_accept', task_id=task_id, epoch=int(epoch), leader=leader_id)
        return True, 'accepted'

    # ------------------------------------------------------------------ 주기
    def tick(self, now: float) -> None:
        """주기 처리 (sim 0.1 s 권장)."""
        self.now = float(now)
        self.health.observe_tick(now)
        self._push(CH_ZONE, self.zone.tick(now))
        self._push(CH_TASK, self.task.tick(now))
        self.lock.poll()
        status = self.monitor.status(now)
        server_dead = self.modes.server_dead(status, self.now)
        # 비컨 (자기 판단도 자기 비컨으로 과반 의심에 넣는다)
        if self._last_beacon is None or self.now - self._last_beacon >= self.cfg.beacon_period_s:
            self._last_beacon = self.now
            self._out.beacon = self._make_beacon(status, server_dead)
        before = self.modes.mode
        mode = self.modes.update(self.now, status, self.suspicion.degraded(now),
                                 self.suspicion.connected_majority(now), self.server_active)
        if mode != before:
            self._note('mode', previous=before, mode=mode, reason=self.modes.reason,
                       server=status)
            if mode == DEGRADED:
                self._enter_degraded()
        self._negotiate()
        self._run_executor()
        self._process_releases()
        self._deadlock()

    # ------------------------------------------------------------------ 협상
    def _candidates(self) -> List[str]:
        busy = set(self.taken) | set(self.done)
        if self.executor is not None and self.executor.task_id:
            busy.discard(self.executor.task_id)
        ts = [t for t in self.board.values() if t.task_id not in busy
              and not self.lock.is_denied(t.task_id)]
        ts.sort(key=lambda t: (-t.priority, t.task_id))
        return [t.task_id for t in ts]

    def _costs(self) -> Dict[str, float]:
        if self.pose is None or not self.loc_healthy:
            return {}
        x, y, _ = self.pose
        out = {}
        for t in self.board.values():
            if t.robot_id and t.robot_id != self.me:
                continue
            out[t.task_id] = math.hypot(t.pickup_x - x, t.pickup_y - y) / self.cfg.nominal_speed
        return out

    def _enter_degraded(self) -> None:
        r = int(self.now // self.cfg.round_s)
        if self.cbaa is None:
            self.cbaa = CbaaAgent(self.me, self.cfg.members, self._candidates(), self._costs(),
                                  self.cfg.nego, task_lock=self.lock, now=float(r))
        self._round = None

    def _negotiate(self) -> None:
        if self.cbaa is None or not self.modes.negotiates():
            return
        r = int(self.now // self.cfg.round_s)
        if self._round is not None and r <= self._round:
            return
        if self._round is not None:
            started = self.cbaa.finish_round(float(self._round))
            if started is not None:
                self._start_task(started)
        cand = self._candidates()
        if cand != self.cbaa.tasks:
            self.cbaa.set_tasks(cand + ([self.cbaa.executing] if self.cbaa.executing
                                        and self.cbaa.executing not in cand else []))
        self.cbaa.set_costs(self._costs())
        self.cbaa.start_round(float(r))
        self._round = r
        self._out.neg = self.cbaa.make_message(float(r))

    def _start_task(self, task_id: str) -> None:
        self.taken.add(task_id)
        cost = self._costs().get(task_id, 0.0)
        self._note('task_granted', task_id=task_id, travel_s=cost)
        if self.executor is not None and not self.executor.busy:
            self._out.events.append(dict(self.executor.start(task_id, cost, self.now),
                                         robot=self.me))

    def _run_executor(self) -> None:
        ex = self.executor
        if ex is None or not ex.busy:
            return
        for ev in ex.step(self.now, self.lock.phase, self.lock.commit):
            ev = dict(ev, robot=self.me)
            self._out.events.append(ev)
            tid = str(ev['task_id'])
            if ev['event'] == 'pickup_start':
                self.taken.add(tid)
            elif ev['event'] == 'done':
                self.done.add(tid)
                if self.cbaa is not None:
                    self.cbaa.complete(tid)
            elif ev['event'] == 'failed':
                self.lock.release(tid, self.me, 0.0)
                if self.cbaa is not None:
                    self.cbaa.abort_execution(tid)

    # ------------------------------------------------------------------ 구역 해제 게이트
    def _body_outside(self, res: str) -> Optional[bool]:
        """몸체가 자원 밖인가 (판정 불가 = None)."""
        if self.pose is None or not self.loc_healthy:
            return None
        poly = self.zone_polygons.get(res)
        if poly is None:
            return None
        x, y, _ = self.pose
        return disc_clear_of_polygon(x, y, self.cfg.body_radius_m + self.cfg.release_margin_m,
                                     poly)

    def _process_releases(self) -> None:
        held = self.zone.authorized()
        self.pending_release &= held
        if self._boot_unknown_inside and self.cfg.release_gate == GATE_ZONE_MAP:
            # 부팅 때 몰라서 남긴 보유: 자세를 받은 뒤 몸체가 밖이면 푼다
            for res in held:
                if self._body_outside(res):
                    self.pending_release.add(res)
            if self.pose is not None and self.loc_healthy:
                self._boot_unknown_inside = False
        ready = set()
        for res in self.pending_release:
            if self.cfg.release_gate == GATE_NONE or self._body_outside(res):
                ready.add(res)
        if ready:
            self.pending_release -= ready
            self._push(CH_ZONE, self.zone.release(sorted(ready), self.now))
            self._note('zone_release', resources=sorted(ready))

    # ------------------------------------------------------------------ 교착 (진단)
    def _my_edges(self) -> Tuple[OutEdge, ...]:
        want = self.zone.requested()
        if not want:
            return ()
        edges = []
        for r, b in sorted(self.peers.items()):
            if want & set(b.held):
                edges.append(OutEdge(r, EDGE_TOKEN))
        return tuple(edges)

    def _deadlock(self) -> None:
        cands = self.wfg.candidates(self.now)
        rem = {r: None for c in cands for r in c}
        for sig in self.confirmer.update(self.now, cands, rem):
            infos = self.wfg.fresh(self.now)
            victim = select_victim(sorted(sig), infos)
            self.deadlock = (tuple(sorted(sig)), victim)
            self._note('deadlock', members=sorted(sig), victim=victim)
        if not cands:
            self.deadlock = ((), '')

    # ------------------------------------------------------------------ 비컨 · 상태
    def _make_beacon(self, status: str, server_dead: bool) -> Beacon:
        self._beacon_seq += 1
        x, y, yaw = self.pose if self.pose is not None else (math.nan, math.nan, math.nan)
        fe = self.fence.max or (0, '')
        ex = self.executor
        cur = ex.task_id if ex is not None and ex.task_id else self.central_task
        held = tuple(sorted(self.zone.authorized() | self.task.authorized()))
        edges = self._my_edges()
        b = Beacon(self.me, self.zone.incarnation, self._beacon_seq, self.now,
                   self.zone.max_epoch, int(fe[0]), str(fe[1]), self.modes.mode, status,
                   server_dead, x, y, yaw, self.battery, self.loc_healthy, cur or '',
                   bool(ex is not None and ex.picked_up), held, edges, self._wait_priority,
                   self._wait_since if edges else None, None)
        self.suspicion.observe(self.me, self.now, server_dead, b.incarnation, b.seq)
        req = None if b.request_stamp is None else Stamp.coerce(b.request_stamp)
        self.wfg.update(WaitInfo(self.me, b.incarnation, b.seq, edges, b.priority, req, None),
                        self.now)
        return b

    def state(self) -> StateView:
        """지금 상태 (amr_msgs/AgentState)."""
        fe = self.fence.max or (0, '')
        ex = self.executor
        status = self.monitor.status(self.now)
        cur = ex.task_id if ex is not None and ex.task_id else self.central_task
        return StateView(
            self.me, self.modes.mode, self.modes.reason, status, self.zone.max_epoch,
            int(fe[0]), str(fe[1]),
            tuple(sorted(self.zone.authorized() | self.task.authorized())),
            tuple(sorted(self.zone.requested())), self.zone.phase(),
            tuple(sorted(self.lock.denied)), cur or '',
            ex.phase if ex is not None else '', self.deadlock[0], self.deadlock[1],
            self.low_epoch_rejected)


def mode_code(mode: str) -> int:
    """모드 이름 → amr_msgs 상수."""
    return MODE_CODES[mode]


def server_code(status: str) -> int:
    """감시 상태 이름 → amr_msgs 상수."""
    return SERVER_CODES[status]


__all__ = [
    'Beacon', 'BoardTask', 'CH_TASK', 'CH_ZONE', 'CoreConfig', 'FleetAgentCore', 'GATE_NONE',
    'GATE_ZONE_MAP', 'OP_ACQUIRE', 'OP_CANCEL', 'OP_COMMIT_TASK', 'OP_RELEASE', 'Outbox',
    'ROLE_ACTIVE', 'ROLE_OBSERVER', 'StateView', 'disc_clear_of_polygon', 'mode_code',
    'server_code', 'task_resource',
]
