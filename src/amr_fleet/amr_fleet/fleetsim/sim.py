"""
실행 1 건: 외생 입력을 만들고 서버·로봇·링크·오라클을 엮어 고정 dt 로 돌린 뒤 결과 dict 를 낸다.

단계 k (시각 t = k·dt): 시각 ≤ t 의 사건(메시지 배달 · 타이머 · 작업 도착 · 고장)을 처리 → 로봇마다
step(t, dt) (투표 tick · 작업 상태 · 운동학) → 오라클이 t + dt 의 실제 자세로 동시 점유를 센다.

결과의 digest 는 완료·load·할당·위반 사건열의 해시다 — 같은 설정이면 같아야 한다 (결정성 시험).
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from typing import Dict, List, Optional

from amr_fleet.comm_emulator import CommEmulator, FLEET, LinkModel, Partition
from amr_fleet.fleetsim.central import CentralServer
from amr_fleet.fleetsim.config import comm_seed, fault_victim, RunConfig, task_stream
from amr_fleet.fleetsim.net import EventQueue, Network
from amr_fleet.fleetsim.oracle import Oracle
from amr_fleet.fleetsim.robot import Robot
from amr_fleet.fleetsim.world import default_world, World

BOARD = 'board'
F5_MINORITY = frozenset({'amr_01', 'amr_02'})
Q_BIN_S = 60.0


class FleetSim:
    """실행 1 건."""

    def __init__(self, cfg: RunConfig, world: Optional[World] = None):
        self.cfg = cfg
        self.arm = cfg.arm
        self.world = world or default_world()
        self.q = EventQueue()
        parts = []
        if cfg.fault == 'F5':
            parts.append(Partition(cfg.t_fault, cfg.t_fault + cfg.partition_s, (F5_MINORITY,)))
        lo, hi = cfg.latency_ms
        self.comm = CommEmulator(seed=comm_seed(cfg), default=LinkModel(lo, hi, cfg.loss),
                                 partitions=parts, history=0)
        self.net = Network(self.q, self.comm, self._alive)
        self.oracle = Oracle(self)
        self.robots: List[Robot] = [Robot(self, r) for r in self.world.robot_ids]
        self.by_id: Dict[str, Robot] = {r.id: r for r in self.robots}
        need_server = self.arm.central_alloc or self.arm.traffic == 'central'
        self.server: Optional[CentralServer] = None
        if need_server:
            self.server = CentralServer(self, self.arm.alloc if self.arm.central_alloc else None,
                                        traffic=self.arm.traffic == 'central')
            self.net.register(FLEET, self.server.on_message)
        for r in self.robots:
            self.net.register(r.id, r.on_message)
        self.arrivals = task_stream(cfg, self.world.docks, self.world.places)
        self.task_docks = {a.spec.task_id: (a.pickup, a.dropoff) for a in self.arrivals}
        self.created = {a.spec.task_id: a.t for a in self.arrivals}
        self.victim = fault_victim(cfg, self.world.robot_ids)
        self._schedule()

    @property
    def now(self) -> float:
        return self.q.now

    def _alive(self, name: str) -> bool:
        if name == FLEET:
            return self.server is not None and self.server.alive
        if name == BOARD:
            return True
        r = self.by_id.get(name)
        return r is not None and r.alive

    # ------------------------------------------------------------------ 일정
    def _schedule(self) -> None:
        q, cfg, arm = self.q, self.cfg, self.arm
        board_to_server = arm.central_alloc
        board_to_robots = not arm.central_alloc or arm.fallback
        for a in self.arrivals:
            q.schedule(a.t, self._arrival, a.spec, board_to_server, board_to_robots)
        if self.server is not None:
            self.server.start(q)
        n = len(self.robots)
        for i, r in enumerate(self.robots):
            ph = 0.5 * i / n                                   # 로봇마다 위상을 엇갈린다
            if arm.central_alloc:
                q.schedule(ph + 0.01, r.state_timer)
            if arm.traffic == 'central':
                q.schedule(ph / 5.0 + 0.02, r.obs_timer)
            if not arm.central_alloc or arm.fallback:
                q.schedule(ph + 0.03, r.beacon_timer)
        if not arm.central_alloc or arm.fallback:
            q.schedule(0.0, self._cbaa_timer, 0)
        q.schedule(0.5, self._wfg_timer)
        if cfg.fault == 'F1':
            q.schedule(cfg.t_fault, self._crash_server)
        elif cfg.fault == 'F6':
            q.schedule(cfg.t_fault, self._crash_robot)

    def _arrival(self, now: float, spec, to_server: bool, to_robots: bool) -> None:
        if to_server:
            self.net.send_reliable(now, BOARD, FLEET, 'board', spec)
        if to_robots:
            for r in self.robots:
                self.net.send_reliable(now, BOARD, r.id, 'board', spec)

    def _cbaa_timer(self, now: float, k: int) -> None:
        for r in self.robots:
            r.cbaa_step(now, k)
        self.q.schedule((k + 1) * self.cfg.cbaa_period_s, self._cbaa_timer, k + 1)

    def _wfg_timer(self, now: float) -> None:
        self.oracle.wfg(now)
        self.q.schedule(now + 0.5, self._wfg_timer)

    def _crash_server(self, now: float) -> None:
        if self.server is not None:
            self.server.crash()

    def _crash_robot(self, now: float) -> None:
        self.by_id[self.victim].crash(now)

    # ------------------------------------------------------------------ 실행
    def run(self) -> dict:
        """끝까지 돌리고 결과 dict."""
        cfg = self.cfg
        wall0 = time.perf_counter()
        steps = int(round(cfg.horizon_s / cfg.dt))
        dt = cfg.dt
        for k in range(steps):
            t = k * dt
            self.q.run_until(t)
            self.q.now = t
            for r in self.robots:
                r.step(t, dt)
            self.oracle.sample(t + dt)
        return self._result(time.perf_counter() - wall0)

    def _result(self, wall: float) -> dict:
        cfg = self.cfg
        H = cfg.horizon_s
        orc = self.oracle.summary(H)
        comp = {t: min(v) for t, v in self.oracle.completions.items()}
        lat = sorted(tc - self.created[t] for t, (tc, _) in comp.items())
        nb = int(math.ceil(H / Q_BIN_S))
        bins = [0] * nb
        for tc, _ in comp.values():
            bins[min(nb - 1, int(tc // Q_BIN_S))] += 1
        after = sum(1 for tc, _ in comp.values() if tc >= cfg.t_fault)
        vz = _agent_stats([r.zone_agent for r in self.robots])
        vt = _agent_stats([r.task_agent for r in self.robots])
        srv = self.server.stats if self.server is not None else {}
        switches = [t for r in self.robots for t, m in r.mode_switches if m == 'negotiation']
        res = {
            'arm': self.arm.name, 'alloc': self.arm.alloc, 'traffic': self.arm.traffic,
            'fallback': self.arm.fallback, 'loss': cfg.loss, 'fault': cfg.fault,
            'seed': cfg.seed, 'rep': cfg.rep, 'horizon_s': H, 'arrival_rate': cfg.arrival_rate,
            'victim': self.victim,
            'n_tasks': len(self.arrivals),
            'completed': len(comp),
            'completed_after_fault': after,
            'unfinished': len(self.arrivals) - len(comp),
            'latency_mean': _r(sum(lat) / len(lat)) if lat else None,
            'latency_p50': _r(_pct(lat, 0.5)), 'latency_p90': _r(_pct(lat, 0.9)),
            'q_bins': bins,
            'server_failed_tasks': srv.get('failed_robot_lost', 0),
            'server': dict(srv),
            'wait_s': _r(sum(r.stats['wait_s'] for r in self.robots)),
            'waits': int(sum(r.stats['waits'] for r in self.robots)),
            'lock_wait_s': _r(sum(r.stats['lock_wait_s'] for r in self.robots)),
            'assign_rejected': int(sum(r.stats['rejected'] for r in self.robots)),
            'cbaa_tasks': int(sum(r.stats['cbaa_tasks'] for r in self.robots)),
            'central_tasks': int(sum(r.stats['central_tasks'] for r in self.robots)),
            'cbaa_bcast': int(sum(r.stats['bcast'] for r in self.robots)),
            'vote_zone': vz, 'vote_task': vt,
            'msgs_sent': int(sum(self.net.sent.values())),
            'msgs_lost': int(sum(self.net.lost.values())),
            'msgs_by_kind': dict(sorted(self.net.sent.items())),
            'first_fallback_s': _r(min(switches)) if switches else None,
            'max_speed': _r(max(r.max_speed for r in self.robots), 6),
            'max_decel': _r(max(r.max_decel for r in self.robots), 6),
            'wall_s': _r(wall),
        }
        res.update(orc)
        res['digest'] = self._digest()
        return res

    def _digest(self) -> str:
        o = self.oracle
        blob = json.dumps({
            'c': sorted((t, [(round(a, 6), b) for a, b in v]) for t, v in o.completions.items()),
            'l': sorted((t, [(round(a, 6), b) for a, b in v]) for t, v in o.loads.items()),
            'a': sorted((t, [(round(a, 6), b, c) for a, b, c in v]) for t, v in o.assigns.items()),
            'v': [(x['zone'], round(x['t0'], 6), x['robots']) for x in o.viol],
            'p': [(r.id, round(r.x, 9), round(r.y, 9)) for r in self.robots],
        }, sort_keys=True)
        return hashlib.sha256(blob.encode()).hexdigest()[:16]


def _agent_stats(agents) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for ag in agents:
        if ag is None:
            continue
        for k in ('attempts', 'abandons', 'authorized', 'votes_granted', 'tentative_expired'):
            out[k] = out.get(k, 0) + int(ag.stats.get(k, 0))
    return out


def _pct(xs: List[float], q: float) -> Optional[float]:
    if not xs:
        return None
    k = min(len(xs) - 1, max(0, int(math.ceil(q * len(xs))) - 1))
    return xs[k]


def _r(x: Optional[float], nd: int = 3) -> Optional[float]:
    return None if x is None else round(float(x), nd)


def run(cfg: RunConfig, world: Optional[World] = None) -> dict:
    """실행 1 건 → 결과 dict."""
    return FleetSim(cfg, world).run()


__all__ = ['BOARD', 'F5_MINORITY', 'FleetSim', 'run']
