"""
에이전트 코어 N 개를 ROS 없이 한 프로세스에서 돌리는 결정적 모의 플릿 (시험 · 로그 재생용, rclpy 비의존).

ROS 노드와 같은 배선을 흉내 낸다: 모든 방송(투표 묶음 · 비컨 · 협상 · 하트비트)은 받는 쪽에서 CommEmulator 의
링크 (송신자 → 나) 로 늦추거나 버린다 (노드의 EmulatedInbox 와 같은 규칙, 같은 송신자끼리는 추월 없음).
자기 자신에게 보낸 것은 다음 단계에 바로 받는다. 서버는 하트비트만 내는 가짜다 (켜기 · 끄기 · epoch).

ROS launch 시험(test_agent_launch.py)이 실제 프로세스 · DDS 로 같은 성질을 다시 본다 — 이 모의는 그보다 빠르고
결정적이라 손실 · 시드를 많이 돌리는 데 쓴다.
"""

from __future__ import annotations

import collections
import dataclasses
import heapq
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from amr_fleet.agent.core import (
    Beacon, BoardTask, CH_TASK, CH_ZONE, CoreConfig, FleetAgentCore, ROLE_ACTIVE,
)
from amr_fleet.agent.path_gate import PathGate
from amr_fleet.comm_emulator import CommEmulator, FLEET
from amr_fleet.distributed.store import MemoryStore


@dataclasses.dataclass
class FakeServer:
    """하트비트만 내는 가짜 서버."""

    epoch: int = 1
    leader_id: str = '/fleet/fleet_manager_node'
    alive: bool = True
    role: int = ROLE_ACTIVE
    period_s: float = 0.5
    seq: int = 0
    last: Optional[float] = None


class LoopbackFleet:
    """에이전트 코어 N 개 + 수신 측 통신 모의 + 가짜 서버."""

    def __init__(self, configs: Sequence[CoreConfig], comm: Optional[CommEmulator] = None,
                 t0: float = 1.0, poses: Optional[Dict[str, Tuple[float, float]]] = None,
                 path_gate: Optional[PathGate] = None):
        self.t = float(t0)
        self.comm = comm if comm is not None else CommEmulator(seed=1, simulate=False)
        self.cfgs = {c.robot_id: c for c in configs}
        self.members = sorted(self.cfgs)
        self.stores = {r: (MemoryStore(), MemoryStore(), MemoryStore()) for r in self.members}
        self.cores: Dict[str, Optional[FleetAgentCore]] = {}
        self.servers: List[FakeServer] = [FakeServer()]
        self._q: List[Tuple[float, int, str, str, Any]] = []
        self._seq = 0
        self._fifo: Dict[Tuple[str, str], float] = collections.defaultdict(float)
        self.events: List[Dict[str, Any]] = []
        self.poses = dict(poses or {})
        self.path_gate = path_gate          # 모든 로봇이 같은 배치 (경로 게이트, b4-wire)
        self.dispatched: List[Tuple[str, str, float]] = []   # (로봇, 작업, t) — 실제 실행기로 넘길 협상 낙찰
        self.sent = collections.Counter()
        for r in self.members:
            self.boot(r)

    # ------------------------------------------------------------------ 로봇
    def boot(self, r: str, inside_zones=None) -> FleetAgentCore:
        """(재)기동 — 저장소는 남는다."""
        zs, ts, fs = self.stores[r]
        core = FleetAgentCore.boot(self.cfgs[r], zs, ts, fs, self.t, inside_zones=inside_zones,
                                   path_gate=self.path_gate)
        if r in self.poses:
            core.set_pose(self.poses[r][0], self.poses[r][1], 0.0)
        self.cores[r] = core
        self._route(r, core)
        return core

    def crash(self, r: str) -> None:
        """휘발 상태를 버린다."""
        self.cores[r] = None

    def alive(self) -> List[str]:
        return [r for r in self.members if self.cores.get(r) is not None]

    # ------------------------------------------------------------------ 망
    def _send(self, src: str, dst: str, kind: str, payload: Any) -> None:
        self.sent[kind] += 1
        if src == dst:
            delay = 0.0
        else:
            delay = self.comm.transmit(src, dst, self.t)
            if delay is None:
                return
        due = max(self.t + delay, self._fifo[(src, dst)])   # 같은 송신자끼리 추월 없음
        self._fifo[(src, dst)] = due
        self._seq += 1
        heapq.heappush(self._q, (due, self._seq, dst, kind, payload))

    def _route(self, src: str, core: FleetAgentCore) -> None:
        out = core.drain()
        for ev in out.events:
            self.events.append(ev)
        for task_id in out.dispatch:
            self.dispatched.append((src, task_id, self.t))
        for dst in self.members:
            if out.votes:
                mine = [(ch, v) for ch, v in out.votes if v.dst == dst]
                if mine:
                    # 한 묶음이 한 패킷 — 수신자마다 링크 하나 (ROS 노드와 같다)
                    self._send(src, dst, 'vote', mine)
            if out.beacon is not None:
                self._send(src, dst, 'beacon', out.beacon)
            if out.neg is not None:
                self._send(src, dst, 'neg', out.neg)

    def _deliver(self, dst: str, kind: str, payload: Any) -> None:
        core = self.cores.get(dst)
        if core is None:
            return
        if kind == 'vote':
            for ch in (CH_ZONE, CH_TASK):
                msgs = [v for c, v in payload if c == ch]
                if msgs:
                    core.on_votes(ch, msgs, self.t)
        elif kind == 'beacon':
            core.on_beacon(payload, self.t)
        elif kind == 'neg':
            core.on_neg(payload, self.t)
        elif kind == 'hb':
            core.on_heartbeat(*payload, self.t)

    # ------------------------------------------------------------------ 서버 · 게시판
    def server(self, i: int = 0) -> FakeServer:
        return self.servers[i]

    def add_server(self, **kw) -> FakeServer:
        """서버를 하나 더 (예: 낮은 epoch 의 옛 권위자)."""
        s = FakeServer(**kw)
        self.servers.append(s)
        return s

    def post(self, tasks: Sequence[BoardTask]) -> None:
        """작업 게시판 (TRANSIENT_LOCAL — 늦게 뜬 로봇도 받는다: 재기동 때 다시 넣는다)."""
        for core in self.cores.values():
            if core is not None:
                for t in tasks:
                    core.on_board(t)

    # ------------------------------------------------------------------ 진행
    def step(self, dt: float = 0.1, hook: Optional[Callable[['LoopbackFleet'], None]] = None
             ) -> None:
        """한 단계(dt): 도착 메시지 배달 → 하트비트 → 모든 로봇 tick → 송신."""
        self.t += dt
        while self._q and self._q[0][0] <= self.t:
            _, _, dst, kind, payload = heapq.heappop(self._q)
            self._deliver(dst, kind, payload)
        for s in self.servers:
            if s.alive and (s.last is None or self.t - s.last >= s.period_s):
                s.last = self.t
                s.seq += 1
                for r in self.members:
                    self._send(FLEET, r, 'hb', (s.epoch, s.leader_id, s.seq, s.role))
        for r in self.members:
            core = self.cores.get(r)
            if core is None:
                continue
            core.tick(self.t)
            self._route(r, core)
        if hook is not None:
            hook(self)

    def run(self, seconds: float, dt: float = 0.1,
            hook: Optional[Callable[['LoopbackFleet'], None]] = None) -> None:
        for _ in range(int(round(seconds / dt))):
            self.step(dt, hook)

    def command(self, r: str, op: int, resources: Sequence[str], priority: int = 0):
        core = self.cores[r]
        res = core.command(op, resources, priority, self.t)
        self._route(r, core)
        return res

    # ------------------------------------------------------------------ 관측
    def modes(self) -> Dict[str, str]:
        return {r: c.modes.mode for r, c in self.cores.items() if c is not None}

    def holders(self, resource: str) -> List[str]:
        """그 자원을 지금 허가받은 로봇 (살아 있는 로봇은 메모리, 죽은 로봇은 디스크)."""
        out = []
        for r in self.members:
            c = self.cores.get(r)
            if c is not None:
                agent = c.task if resource.startswith('task:') else c.zone
                if agent.may_enter(resource):
                    out.append(r)
            else:
                from amr_fleet.distributed.vote_core import ZoneAgent
                zs, ts, _ = self.stores[r]
                st = (ts if resource.startswith('task:') else zs).load()
                if any(resource in z for z in ZoneAgent.held_in_state(st).values()):
                    out.append(r)
        return out

    def beacons_of(self, r: str) -> Dict[str, Beacon]:
        return dict(self.cores[r].peers)
