"""
결정적 모의 플릿 — 투표 코어를 네트워크 · 시계 · 크래시 · 물리 점유와 함께 돌리는 시험 하네스.

ROS 없이 프로토콜 코어(ZoneAgent 또는 같은 인터페이스의 변형)를 돌린다. 메시지는 하나의 목록(in-flight)에
있고, 호출자가 어느 것을 배달 · 버림 · 복제할지 고른다 (재정렬 · 손실 · 중복 · 임의 지연). 로봇마다 시계가
따로 간다 (시계 진행률 · 리셋 임의). 크래시는 휘발 상태를 버리고 저장소(MemoryStore)만 남긴다.
물리 점유(inside)는 크래시와 무관하게 남는다 (구역 안에서 죽은 로봇의 몸체).

check() 가 매 단계 확인하는 불변식 (protocol.md 2 절):
- I1 허가 배타: 자원마다 허가받은 로봇 ≤ 1 (크래시한 로봇은 디스크의 HELD 로 센다)
- I2 물리 배타: 자원마다 몸체가 안에 있는 로봇 ≤ 1
- I3 점유 ⊆ 허가: 안에 있는 로봇은 그 자원을 허가받고 있다
- I4 허가 ⇒ 과반 확정: 허가 (로봇, 요청, 자원) 마다 그 요청의 확정 표를 가진 투표자 ≥ 정족수 (strict 일 때)
- I5 낮은 epoch 수용 0: 받는 쪽 max_epoch 보다 낮은 epoch 메시지가 상태를 바꾼 횟수
"""

from __future__ import annotations

import random
from typing import Callable, Dict, Iterable, List, Optional, Set

from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.store import MemoryStore
from amr_fleet.distributed.vote_core import VoteConfig, ZoneAgent

AgentFactory = Callable[..., tuple]


class Cluster:
    """N 대 플릿 모의. 모든 연산은 trace 에 한 줄씩 남는다 (반례 출력용)."""

    def __init__(self, config: VoteConfig, seed: int = 0,
                 boot: Optional[AgentFactory] = None, strict: bool = True,
                 check_i4: bool = True):
        self.cfg = config
        self.members = list(config.members)
        self.seed = seed
        self._boot_fn = boot or ZoneAgent.boot
        self.strict = strict
        self.check_i4 = check_i4 and strict
        self.stores = {r: MemoryStore() for r in self.members}
        self.clock: Dict[str, float] = {r: 0.0 for r in self.members}
        self.agents: Dict[str, Optional[object]] = {}
        self.inside: Dict[str, Set[str]] = {r: set() for r in self.members}
        self.net: List[VoteMsg] = []
        self.trace: List[str] = []
        self.low_epoch_accepted = 0
        self.boots = {r: 0 for r in self.members}
        self.delivered = 0
        for r in self.members:
            self._boot(r)

    # ------------------------------------------------------------------ 내부
    def _rng(self, r: str) -> random.Random:
        return random.Random(f'{self.seed}:{r}:{self.boots[r]}')

    def _boot(self, r: str) -> None:
        agent, out = self._boot_fn(r, self.cfg, self.stores[r], self.clock[r],
                                   inside=set(self.inside[r]), rng=self._rng(r))
        self.boots[r] += 1
        self.agents[r] = agent
        self.send(out)

    def send(self, msgs: Iterable[VoteMsg]) -> None:
        """메시지를 in-flight 목록 끝에 넣는다."""
        self.net.extend(msgs)

    def alive(self, r: str) -> bool:
        """살아 있나."""
        return self.agents.get(r) is not None

    def _note(self, s: str) -> None:
        self.trace.append(s)

    # ------------------------------------------------------------------ 네트워크
    def deliver(self, i: int) -> None:
        """in-flight i 번째를 배달 (받는 로봇이 죽어 있으면 버려진다)."""
        msg = self.net.pop(i)
        a = self.agents.get(msg.dst)
        if a is None:
            self._note(f'deliver-to-dead {msg.short()}')
            return
        self._note(f'deliver {msg.short()}')
        low = msg.epoch < getattr(a, 'max_epoch', 0)
        snap = a.protocol_state() if low else None
        out = a.handle(msg, self.clock[msg.dst])
        self.delivered += 1
        if low and a.protocol_state() != snap:
            self.low_epoch_accepted += 1
        self.send(out)

    def drop(self, i: int) -> None:
        """in-flight i 번째를 버린다 (손실)."""
        msg = self.net.pop(i)
        self._note(f'drop {msg.short()}')

    def duplicate(self, i: int) -> None:
        """in-flight i 번째를 복제한다 (중복)."""
        self.net.append(self.net[i])
        self._note(f'dup {self.net[i].short()}')

    def chaos(self, rng: random.Random, loss: float = 0.0, dup: float = 0.0, hold: float = 0.0,
              max_events: int = 300) -> None:
        """
        무작위 순서로 배달한다 (재정렬 · 지연 · 손실 · 중복).

        매번 in-flight 에서 하나를 골라 hold 확률로 다음 라운드로 미루고(지연), loss 확률로 버리고(손실),
        dup 확률로 복제한 뒤 배달한다(중복). 새로 생긴 메시지도 같은 라운드에 섞인다.
        """
        later: List[VoteMsg] = []
        n = 0
        while self.net and n < max_events:
            i = rng.randrange(len(self.net))
            n += 1
            if rng.random() < hold:
                later.append(self.net.pop(i))
                continue
            if rng.random() < loss:
                self.drop(i)
                continue
            if rng.random() < dup:
                self.duplicate(i)
            self.deliver(i)
        if later:
            self._note(f'hold {len(later)} msgs')
        self.net.extend(later)

    def deliver_all(self, max_msgs: int = 10_000) -> None:
        """손실 없이 FIFO 로 모두 배달 (새로 생긴 메시지 포함, 상한 max_msgs)."""
        n = 0
        while self.net and n < max_msgs:
            self.deliver(0)
            n += 1

    # ------------------------------------------------------------------ 시계 · 타이머
    def advance(self, r: str, dt: float) -> None:
        """로봇 r 의 시계만 dt 진행."""
        self.clock[r] += dt
        self._note(f'advance {r} +{dt:g} -> {self.clock[r]:g}')

    def advance_all(self, dt: float) -> None:
        """모든 시계를 dt 진행."""
        for r in self.members:
            self.clock[r] += dt
        self._note(f'advance_all +{dt:g}')

    def tick(self, r: str) -> None:
        """로봇 r 의 주기 처리."""
        a = self.agents.get(r)
        if a is None:
            return
        self._note(f'tick {r} @{self.clock[r]:g}')
        self.send(a.tick(self.clock[r]))

    def tick_all(self) -> None:
        """살아 있는 모든 로봇 tick."""
        for r in self.members:
            self.tick(r)

    def reset_clock(self, r: str, to: float = 0.0) -> None:
        """로봇 r 의 sim 시각 역행 (시뮬레이션 리셋)."""
        self.clock[r] = to
        a = self.agents.get(r)
        self._note(f'reset_clock {r} -> {to:g}')
        if a is not None:
            self.send(a.on_time_jump(to))

    def settle(self, rounds: int = 50, dt: Optional[float] = None) -> None:
        """손실 없는 망에서 rounds 회: 모두 배달 → 모두 tick → 시계 dt 진행."""
        dt = self.cfg.retry_s if dt is None else dt
        for _ in range(rounds):
            self.deliver_all()
            self.tick_all()
            self.deliver_all()
            self.advance_all(dt)

    # ------------------------------------------------------------------ 로봇 동작
    def request(self, r: str, resources: Iterable[str], priority: int = 0) -> None:
        """로봇 r 이 사슬 진입 허가를 요청."""
        a = self.agents.get(r)
        if a is None:
            return
        res = sorted(resources)
        self._note(f'request {r} {res} p{priority}')
        self.send(a.request(res, self.clock[r], priority=priority))

    def crash(self, r: str) -> None:
        """휘발 상태를 버린다 (저장소 · 몸체 위치는 남는다)."""
        self._note(f'crash {r}')
        self.agents[r] = None

    def restart(self, r: str) -> None:
        """저장소에서 다시 기동 (incarnation + 1)."""
        if self.alive(r):
            return
        self._note(f'restart {r} inside={sorted(self.inside[r])}')
        self._boot(r)

    def enter(self, r: str) -> bool:
        """허가받은 자원으로 몸체가 들어간다 (허가 전체 — 사슬 통째)."""
        a = self.agents.get(r)
        if a is None:
            return False
        new = set(a.authorized()) - self.inside[r]
        if not new:
            return False
        self._note(f'enter {r} {sorted(new)}')
        self.inside[r] |= new
        return True

    def exit(self, r: str, resource: str) -> bool:
        """몸체가 자원 밖으로 나오고 RELEASE 한다."""
        a = self.agents.get(r)
        if a is None or resource not in self.inside[r]:
            return False
        self._note(f'exit {r} {resource}')
        self.inside[r].discard(resource)
        self.send(a.release([resource], self.clock[r]))
        return True

    def give_up(self, r: str, resource: str) -> bool:
        """허가는 받았지만 들어가지 않은 자원을 내려놓는다 (경로 변경)."""
        a = self.agents.get(r)
        if a is None or resource in self.inside[r] or resource not in a.authorized():
            return False
        self._note(f'give_up {r} {resource}')
        self.send(a.release([resource], self.clock[r]))
        return True

    def set_view(self, r: str, view: Iterable[str]) -> None:
        """로봇 r 의 생존 멤버 관측을 바꾼다 (멤버십 어긋남 주입)."""
        a = self.agents.get(r)
        if a is None:
            return
        v = sorted(set(view) | {r})
        self._note(f'set_view {r} {v}')
        a.set_view(v)

    # ------------------------------------------------------------------ 불변식
    def authorized(self) -> Dict[str, Dict[object, Set[str]]]:
        """로봇 → {요청: 허가 자원}. 크래시한 로봇은 디스크 상태로."""
        out: Dict[str, Dict[object, Set[str]]] = {}
        for r in self.members:
            a = self.agents.get(r)
            if a is not None:
                held = getattr(a, 'held', None)
                if held is None:
                    out[r] = {None: set(a.authorized())}
                else:
                    out[r] = {req: set(z) for req, z in held.items()}
            else:
                try:
                    st = self.stores[r].load()
                except Exception:    # noqa: B902 — 손상된 저장소는 허가 없음으로 센다
                    st = None
                out[r] = ZoneAgent.held_in_state(st) if self.strict else {}
        return out

    def committed(self) -> Dict[str, Dict[str, object]]:
        """투표자 → {자원: 확정 표의 요청}. 크래시한 투표자는 디스크 상태로."""
        out: Dict[str, Dict[str, object]] = {}
        for v in self.members:
            a = self.agents.get(v)
            if a is not None:
                out[v] = {z: vt.req for z, vt in a.votes.items() if vt.committed}
            else:
                try:
                    out[v] = ZoneAgent.committed_in_state(self.stores[v].load())
                except Exception:    # noqa: B902
                    out[v] = {}
        return out

    def check(self) -> List[str]:
        """불변식 위반 목록 (빈 목록 = 통과)."""
        bad: List[str] = []
        auth = self.authorized()
        by_res: Dict[str, Set[str]] = {}
        for r, reqs in auth.items():
            for zones in reqs.values():
                for z in zones:
                    by_res.setdefault(z, set()).add(r)
        for z, rs in sorted(by_res.items()):
            if len(rs) > 1:
                bad.append(f'I1 허가 배타 위반: {z} ← {sorted(rs)}')
        phys: Dict[str, Set[str]] = {}
        for r, zs in self.inside.items():
            for z in zs:
                phys.setdefault(z, set()).add(r)
        for z, rs in sorted(phys.items()):
            if len(rs) > 1:
                bad.append(f'I2 물리 배타 위반: {z} ← {sorted(rs)}')
        for r, zs in self.inside.items():
            have = set().union(*auth.get(r, {}).values()) if auth.get(r) else set()
            if zs - have:
                bad.append(f'I3 허가 없이 안에 있음: {r} {sorted(zs - have)}')
        if self.check_i4:
            com = self.committed()
            for r, reqs in auth.items():
                for req, zones in reqs.items():
                    for z in zones:
                        n = sum(1 for v in self.members if com[v].get(z) == req)
                        if n < self.cfg.quorum:
                            bad.append(f'I4 허가 {r} {req} {z} 의 확정 표 {n} < {self.cfg.quorum}')
        if self.low_epoch_accepted:
            bad.append(f'I5 낮은 epoch 수용 {self.low_epoch_accepted} 회')
        return bad
