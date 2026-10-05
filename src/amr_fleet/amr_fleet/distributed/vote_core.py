"""
정적 과반 배타 투표 + 2단계 진입 (프로토콜 명세: docs/research/fleet-distributed/protocol.md).

로봇 1 대 = ZoneAgent 1 개 = 투표자(자원마다 표 1 장) + 요청자(활성 시도 1 개 + 보유 여러 개).
순수 함수 꼴 API — 입력(메시지 · 시각)을 받아 보낼 메시지 목록을 돌려준다. 네트워크 · 타이머 · ROS 는
호출자 몫이다. 호출자는 돌려받은 메시지를 **호출이 끝난 뒤에만** 보낸다. 모든 공개 메서드는 상태를
바꿨으면 돌아가기 전에 store.save() 를 끝낸다 (응답 전 영속화 — 투표를 잊으면 이중 투표).

안전 불변식 (protocol.md 2 절): 어느 자원에 대해서도 허가받은 로봇은 동시에 1 대 이하.
- 투표자: 자원마다 표는 하나의 요청에만 (배타). 사슬은 전부-아니면-전무. 잠정 표만 시간 · epoch 로 회수,
  확정 표는 그 요청의 RELEASE/ABANDON 으로만 회수 (fail-closed).
- 요청자: 정적 과반(기본 3/5) ENTER_ACK — 같은 요청 ID · 같은 epoch — 를 모은 뒤에만 HELD (진입 허가).
- epoch: 낮은 epoch 메시지는 어떤 상태도 바꾸지 않는다. 높은 epoch 를 보면 채택하고 잠정 표 · 진행 중 시도를
  버린다 (확정 · 보유는 유지).
"""

from __future__ import annotations

import collections
import dataclasses
import random
from typing import Any, Collection, Dict, FrozenSet, Iterable, List, Optional, Set, Tuple

from amr_fleet.distributed import messages as m
from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.ordering import PriorityKey, ReqId, Stamp
from amr_fleet.distributed.store import Store, StoreCorrupt

VOTING = 'VOTING'
ENTERING = 'ENTERING'
STATE_VERSION = 1


@dataclasses.dataclass(frozen=True)
class VoteConfig:
    """프로토콜 파라미터. 시간은 모두 sim 초."""

    members: Tuple[str, ...]
    quorum: Optional[int] = None       # None → 정적 과반 len(members)//2 + 1
    tentative_s: float = 15.0          # 잠정 표 기한 (활성 파라미터 — 안전과 무관)
    retry_s: float = 0.5               # 미응답 투표자에게 재전송 간격
    backoff_min_s: float = 0.5
    backoff_max_s: float = 3.0
    max_bypass: int = 2                # 시도 횟수가 이를 넘으면 기아 표시
    tombstone_cap: int = 256

    def __post_init__(self):
        members = tuple(sorted(set(self.members)))
        if len(members) != len(self.members) or not members:
            raise ValueError(f'멤버는 비어 있지 않고 중복 없어야 한다: {self.members!r}')
        object.__setattr__(self, 'members', members)
        q = len(members) // 2 + 1 if self.quorum is None else int(self.quorum)
        if not 2 * q > len(members) or q > len(members):
            # 두 정족수가 반드시 겹쳐야 한다 (q + q > n). 겹치지 않으면 두 요청이 동시에 허가된다
            raise ValueError(f'정족수 {q} 는 정적 과반이어야 한다 (멤버 {len(members)})')
        object.__setattr__(self, 'quorum', q)
        if self.tentative_s <= 0 or self.retry_s <= 0:
            raise ValueError('tentative_s · retry_s 는 양수')
        if not 0 <= self.backoff_min_s <= self.backoff_max_s:
            raise ValueError('0 ≤ backoff_min_s ≤ backoff_max_s')


@dataclasses.dataclass
class _Vote:
    req: ReqId
    committed: bool
    deadline: Stamp


@dataclasses.dataclass
class _Pending:
    resources: FrozenSet[str]
    key: PriorityKey
    deadline: Stamp


@dataclasses.dataclass
class _Logical:
    """논리 요청 (재시도를 넘어 유지): 자원 · 우선순위 · 나이."""

    resources: FrozenSet[str]
    priority: int
    task_deadline: Optional[Stamp]
    since: int
    attempts: int = 0
    backoff_until: Optional[Stamp] = None


@dataclasses.dataclass
class _Attempt:
    req: ReqId
    resources: FrozenSet[str]
    phase: str
    deadline: Stamp
    key: PriorityKey
    votes: Set[str] = dataclasses.field(default_factory=set)
    acks: Set[str] = dataclasses.field(default_factory=set)
    contended: Set[str] = dataclasses.field(default_factory=set)
    hard_nacks: Set[str] = dataclasses.field(default_factory=set)
    enter_nacks: Set[str] = dataclasses.field(default_factory=set)
    last_send: Dict[str, Stamp] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass
class _Releasing:
    resources: FrozenSet[str]
    acked: Set[str] = dataclasses.field(default_factory=set)
    last_send: Dict[str, Stamp] = dataclasses.field(default_factory=dict)


class ZoneAgent:
    """로봇 1 대의 투표 프로토콜 끝점 (투표자 + 요청자)."""

    def __init__(self, robot_id: str, config: VoteConfig, store: Store,
                 rng: Optional[random.Random] = None):
        """직접 쓰지 말고 boot() 로 만든다 (영속 상태 복원 · incarnation 증가)."""
        if robot_id not in config.members:
            raise ValueError(f'{robot_id!r} 는 멤버가 아니다: {config.members}')
        self.me = robot_id
        self.cfg = config
        self.store = store
        self.rng = rng if rng is not None else random.Random()
        self.n = len(config.members)
        self.q = int(config.quorum)
        # 영속
        self.incarnation = 0
        self.max_epoch = 0
        self.votes: Dict[str, _Vote] = {}
        self.tombstones: 'collections.OrderedDict[ReqId, None]' = collections.OrderedDict()
        self.held: Dict[ReqId, Set[str]] = {}
        self.releasing: Dict[ReqId, _Releasing] = {}
        self.attempt: Optional[_Attempt] = None        # ENTERING 이면 영속
        self.amnesic = False
        # 휘발
        self.view: Optional[FrozenSet[str]] = None     # 장애 감지기가 본 생존 멤버 (활성 힌트일 뿐)
        self.pending: Dict[ReqId, _Pending] = {}
        self.logical: Optional[_Logical] = None
        self.seq = 0
        self.lamport = 0
        self.last_now: Optional[Stamp] = None
        self.stats: Dict[str, int] = collections.Counter()
        self._dirty = False

    # ------------------------------------------------------------------ 기동 · 영속
    @classmethod
    def boot(cls, robot_id: str, config: VoteConfig, store: Store, now: Any,
             inside: Collection[str] = (), rng: Optional[random.Random] = None
             ) -> Tuple['ZoneAgent', List[VoteMsg]]:
        """
        영속 상태를 읽어 에이전트를 만든다. 돌려준 메시지(정리용 RELEASE/ABANDON)를 보낸다.

        inside = 지금 몸체가 걸친 자원 (위치추정 + 오도메트리 판단 — 호출자 몫). 디스크의 보유 중 inside 에
        없는 자원과 ENTERING 이던 시도는 내려놓는다. incarnation 은 첫 송신 전에 +1 해서 영속화한다.
        """
        agent = cls(robot_id, config, store, rng)
        now = Stamp.coerce(now)
        agent.last_now = now
        try:
            state = store.load()
            if state is not None:
                agent._restore(state)
        except (StoreCorrupt, KeyError, ValueError, TypeError):
            # 표를 잊었다 → 다시 투표하면 이중 투표. 영구 기권 (amnesic 도 영속화된다).
            agent = cls(robot_id, config, store, rng)
            agent.last_now = now
            agent.amnesic = True
            # 옛 incarnation 을 모른다 → 겹치지 않을 큰 무작위 값 (옛 요청의 늦은 ACK 와 섞이지 않게)
            agent.incarnation = random.SystemRandom().randrange(1 << 40, 1 << 62)
            agent.stats['amnesic_boot'] += 1
        agent.incarnation += 1
        out: List[VoteMsg] = []
        if agent.attempt is not None:                 # ENTERING 중 재시작 → 그 시도는 포기
            a, agent.attempt = agent.attempt, None
            out += agent._begin_release(a.req, a.resources, now, kind=m.ABANDON)
        inside = set(inside)
        for req in sorted(agent.held):
            outside = agent.held[req] - inside
            if outside:
                out += agent._drop_held(req, outside, now)
        for req, rel in sorted(agent.releasing.items()):
            if not any(o.req == req for o in out if o.kind in (m.RELEASE, m.ABANDON)):
                out += agent._send_release(req, rel, now, force=True)
        agent._dirty = True
        agent._flush()                                # incarnation 을 먼저 영속화
        return agent, out

    def _restore(self, s: Dict[str, Any]) -> None:
        if s.get('version') != STATE_VERSION:
            raise ValueError(f'영속 상태 버전 불일치: {s.get("version")!r}')
        self.incarnation = int(s['incarnation'])
        self.max_epoch = int(s['max_epoch'])
        self.amnesic = bool(s.get('amnesic', False))
        self.votes = {z: _Vote(ReqId.from_list(v['req']), bool(v['committed']),
                               Stamp.from_dict(v['deadline']))
                      for z, v in s['votes'].items()}
        self.tombstones = collections.OrderedDict(
            (ReqId.from_list(t), None) for t in s['tombstones'])
        self.held = {ReqId.from_list(h['req']): set(h['resources']) for h in s['held']}
        self.releasing = {ReqId.from_list(r['req']): _Releasing(frozenset(r['resources']))
                          for r in s['releasing']}
        e = s.get('entering')
        if e is not None:
            self.attempt = _Attempt(ReqId.from_list(e['req']), frozenset(e['resources']),
                                    ENTERING, Stamp(0), PriorityKey(0, None, 0))

    def to_state(self) -> Dict[str, Any]:
        """영속 상태 (JSON)."""
        a = self.attempt
        return {
            'version': STATE_VERSION,
            'robot': self.me,
            'incarnation': self.incarnation,
            'max_epoch': self.max_epoch,
            'amnesic': self.amnesic,
            'votes': {z: {'req': v.req.to_list(), 'committed': v.committed,
                          'deadline': v.deadline.to_dict()}
                      for z, v in sorted(self.votes.items())},
            'tombstones': [t.to_list() for t in self.tombstones],
            'held': [{'req': r.to_list(), 'resources': sorted(z)}
                     for r, z in sorted(self.held.items())],
            'releasing': [{'req': r.to_list(), 'resources': sorted(rel.resources)}
                          for r, rel in sorted(self.releasing.items())],
            'entering': (None if a is None or a.phase != ENTERING else
                         {'req': a.req.to_list(), 'resources': sorted(a.resources)}),
        }

    def _flush(self) -> None:
        """바뀐 상태를 영속화 (응답을 돌려주기 전에 반드시)."""
        if self._dirty:
            self.store.save(self.to_state())
            self._dirty = False

    # ------------------------------------------------------------------ 조회
    def authorized(self) -> Set[str]:
        """진입 허가된 자원 (HELD)."""
        out: Set[str] = set()
        for zones in self.held.values():
            out |= zones
        return out

    def may_enter(self, resource: str) -> bool:
        """진입 게이트 (자기 펜싱): 과반 ENTER_ACK(현재 요청 · 현재 epoch)로 만든 HELD 에 있을 때만."""
        return any(resource in zones for zones in self.held.values())

    def phase(self) -> str:
        """요청자 단계 (표시용)."""
        if self.attempt is not None:
            return self.attempt.phase
        if self.logical is not None:
            return 'BACKOFF'
        return 'HELD' if self.held else 'IDLE'

    def protocol_state(self) -> Dict[str, Any]:
        """시험용 상태 스냅숏 (Lamport · 재전송 시각 제외)."""
        a = self.attempt
        lg = self.logical
        return {
            'durable': self.to_state(),
            'pending': sorted((str(r), sorted(p.resources)) for r, p in self.pending.items()),
            'attempt': None if a is None else (str(a.req), a.phase, sorted(a.votes),
                                               sorted(a.acks), sorted(a.contended),
                                               sorted(a.hard_nacks), sorted(a.enter_nacks)),
            'logical': None if lg is None else (sorted(lg.resources), lg.attempts),
            'releasing_acked': sorted((str(r), sorted(rel.acked))
                                      for r, rel in self.releasing.items()),
        }

    @staticmethod
    def held_in_state(state: Optional[Dict[str, Any]]) -> Dict[ReqId, Set[str]]:
        """영속 상태의 HELD (크래시한 로봇의 허가를 셀 때)."""
        if not state:
            return {}
        return {ReqId.from_list(h['req']): set(h['resources']) for h in state['held']}

    @staticmethod
    def committed_in_state(state: Optional[Dict[str, Any]]) -> Dict[str, ReqId]:
        """영속 상태의 확정 표: 자원 → 요청."""
        if not state:
            return {}
        return {z: ReqId.from_list(v['req']) for z, v in state['votes'].items()
                if v['committed']}

    # ------------------------------------------------------------------ 송신 도우미
    def _msg(self, kind: str, dst: str, req: ReqId, resources: Iterable[str] = (),
             key: Optional[PriorityKey] = None, deadline: Optional[Stamp] = None,
             reason: str = '') -> VoteMsg:
        self.lamport += 1
        return VoteMsg(kind, self.me, dst, self.max_epoch, req, frozenset(resources), key,
                       deadline, self.lamport, reason)

    def _bcast(self, kind: str, req: ReqId, resources: Iterable[str],
               targets: Optional[Iterable[str]] = None, **kw) -> List[VoteMsg]:
        targets = self.cfg.members if targets is None else targets
        return [self._msg(kind, v, req, resources, **kw) for v in targets]

    def set_view(self, view: Iterable[str]) -> None:
        """
        장애 감지기가 본 생존 멤버. REQUEST 를 보낼 대상만 줄인다 (활성 힌트).

        정족수는 언제나 정적 멤버 전체의 과반이다 — 관측이 어긋나도 안전은 그대로이고, 관측이 틀리면
        과반을 못 모아 진입을 못 할 뿐이다. RELEASE/ABANDON 은 관측과 무관하게 전원에게 보낸다.
        """
        v = frozenset(view) | {self.me}
        unknown = v - set(self.cfg.members)
        if unknown:
            raise ValueError(f'멤버가 아닌 로봇: {sorted(unknown)}')
        self.view = v

    def _request_targets(self) -> List[str]:
        if self.view is None:
            return list(self.cfg.members)
        return [v for v in self.cfg.members if v in self.view]

    # ------------------------------------------------------------------ 시간 · epoch
    def _observe_time(self, now: Stamp) -> List[VoteMsg]:
        out: List[VoteMsg] = []
        if self.last_now is not None and now < self.last_now:
            out += self.on_time_jump(now)
        self.last_now = now
        return out

    def on_time_jump(self, now: Any) -> List[VoteMsg]:
        """시각 역행(시뮬레이션 리셋) → epoch + 1 채택. 잠정 표 · 시도 포기, 확정 · 보유 유지."""
        now = Stamp.coerce(now)
        self.last_now = now
        self.stats['time_jump'] += 1
        out = self._adopt_epoch(self.max_epoch + 1, now)
        if self.logical is not None:
            self.logical.backoff_until = now
        for rel in self.releasing.values():
            rel.last_send.clear()
        self._flush()
        return out

    def _adopt_epoch(self, epoch: int, now: Stamp) -> List[VoteMsg]:
        """더 높은 epoch 채택: TENTATIVE · 대기열 폐기 (COMMITTED 유지), 진행 중 시도 포기."""
        self.max_epoch = epoch
        self._dirty = True
        self.stats['epoch_adopt'] += 1
        for z in [z for z, v in self.votes.items() if not v.committed]:
            del self.votes[z]
        self.pending.clear()
        out: List[VoteMsg] = []
        if self.attempt is not None:
            out += self._abandon(now)
        return out

    # ------------------------------------------------------------------ 공개 입력
    def handle(self, msg: VoteMsg, now: Any) -> List[VoteMsg]:
        """메시지 1 개 처리 → 보낼 메시지."""
        if msg.dst != self.me:
            raise ValueError(f'{self.me} 앞이 아닌 메시지: {msg.short()}')
        now = Stamp.coerce(now)
        out = self._observe_time(now)
        if msg.epoch < self.max_epoch:
            # 낮은 epoch: 어떤 상태도 바꾸지 않는다 (RELEASE 포함 — fail-closed). 송신자가 따라오게 알린다
            self.stats['rejected_low_epoch'] += 1
            if msg.kind in m.TO_VOTER:
                out.append(self._msg(m.NACK, msg.src, msg.req, msg.resources,
                                     reason=m.R_STALE_EPOCH))
            self._flush()
            return out
        self.lamport = max(self.lamport, msg.lamport)
        if msg.epoch > self.max_epoch:
            out += self._adopt_epoch(msg.epoch, now)
        kind = msg.kind
        if kind == m.REQUEST:
            out += self._on_request(msg, now)
        elif kind == m.ENTER:
            out += self._on_enter(msg, now)
        elif kind in (m.ABANDON, m.RELEASE):
            out += self._on_release(msg, now)
        elif kind == m.VOTE:
            out += self._on_vote(msg, now)
        elif kind == m.NACK:
            out += self._on_nack(msg, now)
        elif kind == m.ENTER_ACK:
            out += self._on_enter_ack(msg, now)
        elif kind == m.RELEASE_ACK:
            self._on_release_ack(msg)
        self._flush()
        return out

    def tick(self, now: Any) -> List[VoteMsg]:
        """주기 처리: 잠정 표 만료, 시도 기한 · 재전송 · 백오프 재시도, RELEASE 재전송."""
        now = Stamp.coerce(now)
        out = self._observe_time(now)
        out += self._expire(now)
        a = self.attempt
        if a is not None:
            if now >= a.deadline:
                out += self._abandon(now)
            else:
                out += self._retransmit_attempt(a, now)
        lg = self.logical
        if self.attempt is None and lg is not None and (
                lg.backoff_until is None or now >= lg.backoff_until):
            out += self._start_attempt(now)
        for req, rel in sorted(self.releasing.items()):
            out += self._send_release(req, rel, now)
        self._flush()
        return out

    def request(self, resources: Iterable[str], now: Any, priority: int = 0,
                task_deadline: Optional[Any] = None) -> List[VoteMsg]:
        """
        사슬 S 진입 허가를 요청한다 (진행 중인 다른 요청은 포기).

        이미 보유한 자원은 빼고 요청한다. 남는 것이 없으면 아무것도 보내지 않는다.
        """
        now = Stamp.coerce(now)
        out = self._observe_time(now)
        if self.attempt is not None:
            out += self._abandon(now, backoff=False)
        self.logical = None
        want = frozenset(resources) - self.authorized()
        if want:
            td = None if task_deadline is None else Stamp.coerce(task_deadline)
            self.lamport += 1
            self.logical = _Logical(want, int(priority), td, self.lamport)
            out += self._start_attempt(now)
        self._flush()
        return out

    def cancel(self, now: Any) -> List[VoteMsg]:
        """진행 중인 요청을 포기한다 (보유는 그대로)."""
        now = Stamp.coerce(now)
        out = self._observe_time(now)
        if self.attempt is not None:
            out += self._abandon(now, backoff=False)
        self.logical = None
        self._flush()
        return out

    def release(self, resources: Iterable[str], now: Any) -> List[VoteMsg]:
        """
        보유 자원을 내려놓는다 (진출 확인 · 경로 변경). 허가를 먼저 지우고 영속화한 뒤 RELEASE 를 돌려준다.

        호출자 책임: 몸체가 그 자원 밖에 있다는 판단 (위치추정 건강 + 오도메트리 진행 거리 — P3).
        """
        now = Stamp.coerce(now)
        out = self._observe_time(now)
        want = set(resources)
        for req in sorted(self.held):
            zones = self.held[req] & want
            if zones:
                out += self._drop_held(req, zones, now)
        self._flush()
        return out

    # ------------------------------------------------------------------ 투표자
    def _expire(self, now: Stamp) -> List[VoteMsg]:
        gone = [z for z, v in self.votes.items() if not v.committed and v.deadline <= now]
        for z in gone:
            del self.votes[z]
        if gone:
            self._dirty = True
            self.stats['tentative_expired'] += 1
        return self._try_grant(now)

    def _try_grant(self, now: Stamp) -> List[VoteMsg]:
        """대기열을 순서 키대로 훑어 줄 수 있는 표를 준다. 막힌 기아 요청은 그 자원을 예약한다."""
        out: List[VoteMsg] = []
        if self.amnesic:
            return out
        reserved: Dict[str, ReqId] = {}
        for req, p in sorted(self.pending.items(), key=lambda kv: kv[1].key.sort_key(kv[0])):
            if p.deadline <= now:
                del self.pending[req]
                continue
            free = all(z not in self.votes for z in p.resources)
            blocked = any(reserved.get(z, req) != req for z in p.resources)
            if free and not blocked:
                for z in p.resources:
                    self.votes[z] = _Vote(req, False, p.deadline)
                del self.pending[req]
                self._dirty = True
                self.stats['votes_granted'] += 1
                out.append(self._msg(m.VOTE, req.robot, req, p.resources))
            elif p.key.starved:
                for z in p.resources:
                    reserved.setdefault(z, req)
        return out

    def _tombstone(self, req: ReqId) -> None:
        if req not in self.tombstones:
            self.tombstones[req] = None
            self._dirty = True
            while len(self.tombstones) > self.cfg.tombstone_cap:
                self.tombstones.popitem(last=False)

    def _on_request(self, msg: VoteMsg, now: Stamp) -> List[VoteMsg]:
        req = msg.req
        if self.amnesic:
            return [self._msg(m.NACK, msg.src, req, msg.resources, reason=m.R_AMNESIC)]
        out = self._expire(now)
        if req in self.tombstones:
            return out + [self._msg(m.NACK, msg.src, req, msg.resources, reason=m.R_FINISHED)]
        if not msg.resources:
            return out
        bound = [self.votes.get(z) for z in msg.resources]
        if all(v is not None and v.req == req for v in bound):     # 중복 REQUEST → 표 재전송
            return out + [self._msg(m.VOTE, msg.src, req, msg.resources)]
        if msg.deadline is None or msg.deadline <= now:
            return out + [self._msg(m.NACK, msg.src, req, msg.resources, reason=m.R_EXPIRED)]
        if any(r.robot == req.robot and r > req for r in self.pending):
            return out                                              # 더 새 시도가 이미 있음 — 무시
        # 같은 로봇의 더 옛 시도는 끝났다 (요청자는 활성 시도가 하나뿐): 대기열에서 빼고 잠정 표를 회수한다.
        # 확정 표(그 로봇이 보유 중)는 건드리지 않는다. 잠정 회수 = 이른 만료라 안전하다.
        for old in [r for r in self.pending if r.robot == req.robot and r < req]:
            del self.pending[old]
        stale = {v.req for v in self.votes.values()
                 if not v.committed and v.req.robot == req.robot and v.req < req}
        for z in [z for z, v in self.votes.items() if v.req in stale]:
            del self.votes[z]
        for r in sorted(stale):
            self._tombstone(r)
        if stale:
            self._dirty = True
        key = msg.key if msg.key is not None else PriorityKey(0, None, msg.lamport)
        self.pending[req] = _Pending(msg.resources, key, msg.deadline)
        granted = self._try_grant(now)
        out += granted
        if not any(g.req == req for g in granted):
            blockers = [self.votes[z] for z in msg.resources if z in self.votes]
            occupied = blockers and all(v.committed for v in blockers)
            out.append(self._msg(m.NACK, msg.src, req, msg.resources,
                                 reason=m.R_BUSY if occupied else m.R_CONTENDED))
        return out

    def _on_enter(self, msg: VoteMsg, now: Stamp) -> List[VoteMsg]:
        req = msg.req
        if self.amnesic:
            return [self._msg(m.NACK, msg.src, req, msg.resources, reason=m.R_AMNESIC)]
        out = self._expire(now)
        if req in self.tombstones:
            return out + [self._msg(m.NACK, msg.src, req, msg.resources, reason=m.R_FINISHED)]
        bound = [self.votes.get(z) for z in msg.resources]
        if msg.resources and all(v is not None and v.req == req for v in bound):
            for v in bound:
                if not v.committed:
                    v.committed = True
                    self._dirty = True
            self.stats['commits'] += 1
            return out + [self._msg(m.ENTER_ACK, msg.src, req, msg.resources)]
        return out + [self._msg(m.NACK, msg.src, req, msg.resources, reason=m.R_NOT_TENTATIVE)]

    def _on_release(self, msg: VoteMsg, now: Stamp) -> List[VoteMsg]:
        req = msg.req
        zones = msg.resources or frozenset(z for z, v in self.votes.items() if v.req == req)
        for z in zones:
            v = self.votes.get(z)
            if v is not None and v.req == req:
                del self.votes[z]
                self._dirty = True
        self.pending.pop(req, None)
        self._tombstone(req)
        out = [self._msg(m.RELEASE_ACK, msg.src, req, msg.resources)]
        return out + self._try_grant(now)

    # ------------------------------------------------------------------ 요청자
    def _start_attempt(self, now: Stamp) -> List[VoteMsg]:
        lg = self.logical
        if lg is None:
            return []
        lg.attempts += 1
        lg.backoff_until = None
        self.seq += 1
        req = ReqId(self.me, self.incarnation, self.seq)
        key = PriorityKey(lg.priority, lg.task_deadline, lg.since,
                          starved=lg.attempts > self.cfg.max_bypass)
        a = _Attempt(req, lg.resources, VOTING, now.add(self.cfg.tentative_s), key)
        self.attempt = a
        self.stats['attempts'] += 1
        targets = self._request_targets()
        out = self._bcast(m.REQUEST, req, a.resources, targets=targets, key=key,
                          deadline=a.deadline)
        for v in targets:
            a.last_send[v] = now
        return out

    def _retransmit_attempt(self, a: _Attempt, now: Stamp) -> List[VoteMsg]:
        out: List[VoteMsg] = []
        if a.phase == VOTING:
            targets = [v for v in self._request_targets()
                       if v not in a.votes and v not in a.hard_nacks]
            kind = m.REQUEST
        else:
            targets = [v for v in sorted(a.votes) if v not in a.acks and v not in a.enter_nacks]
            kind = m.ENTER
        for v in targets:
            last = a.last_send.get(v)
            if last is None or now >= last.add(self.cfg.retry_s):
                a.last_send[v] = now
                if kind == m.REQUEST:
                    out.append(self._msg(kind, v, a.req, a.resources, key=a.key,
                                         deadline=a.deadline))
                else:
                    out.append(self._msg(kind, v, a.req, a.resources))
        return out

    def _abandon(self, now: Stamp, backoff: bool = True) -> List[VoteMsg]:
        a, self.attempt = self.attempt, None
        if a is None:
            return []
        self.stats['abandons'] += 1
        out = self._begin_release(a.req, a.resources, now, kind=m.ABANDON)
        lg = self.logical
        if lg is not None and backoff:
            k = min(lg.attempts - 1, 3)
            wait = self.rng.uniform(self.cfg.backoff_min_s, self.cfg.backoff_max_s) * (2 ** k)
            lg.backoff_until = now.add(wait)
        return out

    def _begin_release(self, req: ReqId, resources: FrozenSet[str], now: Stamp,
                       kind: str = m.RELEASE) -> List[VoteMsg]:
        rel = self.releasing.get(req)
        if rel is None:
            rel = self.releasing[req] = _Releasing(frozenset(resources))
        elif not resources <= rel.resources:
            rel.resources = rel.resources | resources
            rel.acked.clear()
        self._dirty = True
        return self._send_release(req, rel, now, force=True, kind=kind)

    def _send_release(self, req: ReqId, rel: _Releasing, now: Stamp, force: bool = False,
                      kind: str = m.RELEASE) -> List[VoteMsg]:
        out: List[VoteMsg] = []
        for v in self.cfg.members:
            if v in rel.acked:
                continue
            last = rel.last_send.get(v)
            if force or last is None or now >= last.add(self.cfg.retry_s):
                rel.last_send[v] = now
                out.append(self._msg(kind, v, req, rel.resources))
        return out

    def _drop_held(self, req: ReqId, zones: Collection[str], now: Stamp) -> List[VoteMsg]:
        zones = frozenset(zones) & frozenset(self.held.get(req, ()))
        if not zones:
            return []
        self.held[req] -= zones
        if not self.held[req]:
            del self.held[req]
        self._dirty = True
        return self._begin_release(req, zones, now)

    def _stale_reply(self, msg: VoteMsg) -> List[VoteMsg]:
        """활성도 보유도 정리 중도 아닌 요청에 대한 VOTE/ENTER_ACK → 그 투표자에게 RELEASE (좀비 청소)."""
        if msg.req in self.held or msg.req in self.releasing:
            return []
        if msg.req.robot != self.me:
            return []
        self.stats['stale_reply_cleanup'] += 1
        return [self._msg(m.RELEASE, msg.src, msg.req, msg.resources)]

    def _on_vote(self, msg: VoteMsg, now: Stamp) -> List[VoteMsg]:
        a = self.attempt
        if a is None or msg.req != a.req:
            return self._stale_reply(msg)
        if msg.resources != a.resources:
            return []
        a.votes.add(msg.src)
        a.contended.discard(msg.src)
        a.hard_nacks.discard(msg.src)
        if a.phase == VOTING and len(a.votes) >= self.q:
            a.phase = ENTERING
            self._dirty = True                      # ENTERING 영속화 뒤에 ENTER
            out = []
            for v in sorted(a.votes):
                a.last_send[v] = now
                out.append(self._msg(m.ENTER, v, a.req, a.resources))
            return out
        if a.phase == ENTERING and msg.src not in a.acks:
            a.last_send[msg.src] = now
            return [self._msg(m.ENTER, msg.src, a.req, a.resources)]
        return []

    def _on_nack(self, msg: VoteMsg, now: Stamp) -> List[VoteMsg]:
        a = self.attempt
        if a is None or msg.req != a.req:
            return []
        if a.phase == VOTING:
            if msg.src in a.votes:
                return []
            if msg.reason == m.R_CONTENDED:
                a.contended.add(msg.src)
            elif msg.reason != m.R_BUSY:
                a.hard_nacks.add(msg.src)
            # 표 갈림 · 과반 불가 → 일찍 포기하고 백오프 (점유(busy)는 대기열에서 기다린다)
            if len(a.contended) > self.n - self.q or len(a.hard_nacks) > self.n - self.q:
                return self._abandon(now)
            return []
        if msg.reason in (m.R_BUSY, m.R_CONTENDED):
            return []
        a.enter_nacks.add(msg.src)
        a.votes.discard(msg.src)
        if self.n - len(a.enter_nacks) < self.q:
            return self._abandon(now)
        return []

    def _on_enter_ack(self, msg: VoteMsg, now: Stamp) -> List[VoteMsg]:
        a = self.attempt
        if a is None or msg.req != a.req or a.phase != ENTERING:
            return self._stale_reply(msg)
        if msg.epoch != self.max_epoch or msg.resources != a.resources:
            return []                               # 자기 펜싱: 현재 epoch · 같은 사슬만
        a.acks.add(msg.src)
        if len(a.acks) >= self.q:
            self.held.setdefault(a.req, set()).update(a.resources)
            self.attempt = None
            self.logical = None
            self._dirty = True
            self.stats['authorized'] += 1
        return []

    def _on_release_ack(self, msg: VoteMsg) -> None:
        rel = self.releasing.get(msg.req)
        if rel is None:
            return
        if msg.resources and not rel.resources <= msg.resources:
            return                                  # 더 작은 옛 부분집합에 대한 ACK
        rel.acked.add(msg.src)
        if len(rel.acked) >= self.n:
            del self.releasing[msg.req]
            self._dirty = True
