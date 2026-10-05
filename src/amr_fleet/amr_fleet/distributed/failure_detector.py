"""
장애 감지 (프로토콜 명세 8 절) — 안전에는 쓰지 않는다. 할당 모드 전환(효율)에만 쓴다.

- HeartbeatMonitor : 한 발신자(서버)의 하트비트를 **sim-time** 으로 잰다. 마지막 수신 뒤 suspect_s → SUSPECT,
                     dead_s → DEAD. 낮은 epoch · 되돌아간 seq 의 하트비트는 갱신하지 않는다.
- LocalHealth      : Lifeguard 식 자기 지연 점검. 자기 감시 주기가 늦게 돌면 점수를 올리고 타임아웃에
                     (1 + 점수) 를 곱한다. 방금 자기 주기가 늦었으면(자기 정지) 그 판정에서는 DEAD 를 내지 않는다
                     — 큐에 쌓인 하트비트를 아직 처리하지 못했을 수 있다.
- MajoritySuspicion: 각 로봇 비컨의 "서버 DEAD" 표시를 모아 신선한 비컨 기준 정적 과반(3/5)일 때만 DEGRADED.
                     한 로봇이 서버를 못 보는 것(그 로봇의 링크 문제)과 서버가 죽은 것을 가른다.
- EpochFence       : 리더 명령 펜싱. (epoch, leader_id) 가 본 최댓값보다 작으면 거부 (동률은 leader_id 로).

감지는 틀릴 수 있다 (FLP · Chandra–Toueg). 그래서 이 모듈의 출력은 구역 투표의 안전 논증에 들어가지 않는다.
시간은 전부 호출자가 넘기는 sim 시각(Stamp 또는 초)이다 — 벽시계를 읽지 않는다.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, Iterable, Optional, Set, Tuple, Union

from amr_fleet.distributed.ordering import Stamp
from amr_fleet.distributed.store import Store

ALIVE = 'ALIVE'
SUSPECT = 'SUSPECT'
DEAD = 'DEAD'
UNKNOWN = 'UNKNOWN'

Time = Union[Stamp, float, int]


def _sec(a: Stamp, b: Stamp) -> float:
    """두 시각의 차 b - a [s]."""
    return (b.total_ns() - a.total_ns()) / 1e9


@dataclasses.dataclass(frozen=True)
class DetectorConfig:
    """하트비트 감지 파라미터 (sim 초). 기본값은 보고서 135–158 행: 2 Hz, 3 회 누락 1.5 s, 5 s."""

    period_s: float = 0.5
    suspect_s: float = 1.5
    dead_s: float = 5.0
    late_factor: float = 2.0       # 자기 감시 간격 > late_factor × check_period 이면 '늦음'
    check_period_s: float = 0.1    # 자기 감시(tick) 기대 주기
    max_health: int = 8            # Lifeguard 지역 건강 점수 상한

    def __post_init__(self):
        if not 0 < self.period_s <= self.suspect_s < self.dead_s:
            raise ValueError('0 < period_s ≤ suspect_s < dead_s 이어야 한다')
        if self.late_factor <= 1.0 or self.check_period_s <= 0 or self.max_health < 0:
            raise ValueError('late_factor > 1, check_period_s > 0, max_health ≥ 0')


class LocalHealth:
    """자기 지연 점검 (Lifeguard Local Health Multiplier 의 단순판)."""

    def __init__(self, cfg: DetectorConfig):
        self.cfg = cfg
        self.score = 0
        self.last_tick: Optional[Stamp] = None
        self.last_gap_s = 0.0
        self.late = False

    def observe_tick(self, now: Time) -> int:
        """자기 감시 주기 1 회. 늦었으면 점수 +1, 제때면 -1. 돌려주는 값 = 점수."""
        now = Stamp.coerce(now)
        if self.last_tick is not None and now >= self.last_tick:
            gap = _sec(self.last_tick, now)
            self.last_gap_s = gap
            self.late = gap > self.cfg.late_factor * self.cfg.check_period_s
            if self.late:
                self.score = min(self.cfg.max_health, self.score + 1)
            else:
                self.score = max(0, self.score - 1)
        else:
            self.late = False
            self.last_gap_s = 0.0
        self.last_tick = now
        return self.score

    def multiplier(self) -> float:
        """타임아웃 배수 1 + 점수."""
        return 1.0 + self.score


class HeartbeatMonitor:
    """발신자 하나(서버)의 sim-time 하트비트 감시."""

    def __init__(self, cfg: DetectorConfig = DetectorConfig(),
                 health: Optional[LocalHealth] = None):
        self.cfg = cfg
        self.health = health
        self.last_rx: Optional[Stamp] = None
        self.last_key: Optional[Tuple[int, int]] = None     # (epoch, seq)
        self.ignored = 0
        self.time_jumps = 0

    def observe(self, now: Time, epoch: int, seq: int) -> bool:
        """하트비트 수신. 낮은 epoch · 같은 epoch 의 되돌아간/같은 seq 는 무시(False)."""
        now = Stamp.coerce(now)
        key = (int(epoch), int(seq))
        if self.last_key is not None and key <= self.last_key:
            self.ignored += 1
            return False
        self.last_key = key
        self.last_rx = now
        return True

    def status(self, now: Time) -> str:
        """지금 판정: UNKNOWN(아직 못 받음) · ALIVE · SUSPECT · DEAD."""
        now = Stamp.coerce(now)
        if self.last_rx is None:
            return UNKNOWN
        if now < self.last_rx:
            # 시각 역행(시뮬레이션 리셋): 기준을 지금으로 다시 잡는다 (epoch 변경은 호출자 몫)
            self.time_jumps += 1
            self.last_rx = now
            return ALIVE
        silence = _sec(self.last_rx, now)
        mult = self.health.multiplier() if self.health is not None else 1.0
        if silence >= self.cfg.dead_s * mult:
            if self.health is not None and self.health.late:
                return SUSPECT          # 방금 자기 주기가 늦었다 — 자기 탓일 수 있으니 DEAD 보류
            return DEAD
        if silence >= self.cfg.suspect_s * mult:
            return SUSPECT
        return ALIVE


@dataclasses.dataclass
class _Beacon:
    rx: Stamp
    key: Tuple[int, int]           # (incarnation, seq)
    server_dead: bool


class MajoritySuspicion:
    """비컨의 '서버 DEAD' 표시를 모아 정적 과반이 동의할 때만 DEGRADED."""

    def __init__(self, members: Iterable[str], beacon_ttl_s: float = 2.0,
                 quorum: Optional[int] = None):
        self.members = tuple(sorted(set(members)))
        n = len(self.members)
        self.quorum = n // 2 + 1 if quorum is None else int(quorum)
        if not 2 * self.quorum > n:
            raise ValueError(f'정족수 {self.quorum} 는 정적 과반이어야 한다 (멤버 {n})')
        self.ttl_s = float(beacon_ttl_s)
        self.beacons: Dict[str, _Beacon] = {}

    def observe(self, robot: str, now: Time, server_dead: bool, incarnation: int = 0,
                seq: int = 0) -> bool:
        """비컨 1 개 (자기 판단도 자기 비컨으로 넣는다). 옛 비컨((incarnation, seq) 역행)은 무시."""
        if robot not in self.members:
            return False
        now = Stamp.coerce(now)
        key = (int(incarnation), int(seq))
        old = self.beacons.get(robot)
        if old is not None and key <= old.key:
            return False
        self.beacons[robot] = _Beacon(now, key, bool(server_dead))
        return True

    def _fresh(self, now: Stamp) -> Dict[str, _Beacon]:
        return {r: b for r, b in self.beacons.items()
                if b.rx <= now and _sec(b.rx, now) <= self.ttl_s}

    def dead_votes(self, now: Time) -> Set[str]:
        """신선한 비컨 중 서버 DEAD 를 말하는 로봇."""
        now = Stamp.coerce(now)
        return {r for r, b in self._fresh(now).items() if b.server_dead}

    def degraded(self, now: Time) -> bool:
        """정적 과반이 서버 DEAD 에 동의."""
        return len(self.dead_votes(now)) >= self.quorum

    def connected_majority(self, now: Time) -> bool:
        """신선한 비컨(자기 포함)을 보낸 로봇이 정적 과반 이상 — 아니면 ISOLATED 후보."""
        return len(self._fresh(Stamp.coerce(now))) >= self.quorum


class EpochFence:
    """
    리더 명령 펜싱: (epoch, leader_id) 사전식 최댓값보다 작은 명령은 거부한다.

    store 를 주면 최댓값을 영속화한다 (재시작 뒤에도 옛 리더의 명령을 받지 않게).
    """

    def __init__(self, store: Optional[Store] = None):
        self.store = store
        self.max: Optional[Tuple[int, str]] = None
        self.rejected = 0
        if store is not None:
            st = store.load()
            if st and st.get('fence') is not None:
                self.max = (int(st['fence'][0]), str(st['fence'][1]))

    def accept(self, epoch: int, leader_id: str) -> bool:
        """명령을 받아도 되나. 더 크면 최댓값을 올리고(영속화 뒤) True."""
        key = (int(epoch), str(leader_id))
        if self.max is not None and key < self.max:
            self.rejected += 1
            return False
        if self.max is None or key > self.max:
            self.max = key
            if self.store is not None:
                self.store.save({'fence': [key[0], key[1]]})
        return True
