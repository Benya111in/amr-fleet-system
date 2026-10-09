"""
실행 1 건: 배치 · 시드 · 부하 → contract.md §4 행 + 부가 진단(extras).

단계 k (t = k·dt, dt 0.1 s):
 1. 작업 도착(t 이하) → 할당 대기열. 0.5 s 마다 할당 라운드 (fleet.Dispatcher).
 2. 0.5 s 마다(위상 0.25 s) TrafficManager.update(관측) → 로봇마다 명령 적용 (hold · 포켓 · keepout).
 3. 몸체 표(로봇 + actor, 단계 시작 위치)를 만들고 로봇마다 step.
 4. 체류 기록 · 측위 사건 · 판정(물리 교착 · 구역 동시 점유 · 최소 거리).

결과 지표 정의 (sim.md §출력):
- completed / throughput_per_h: 지평 안에 하역을 끝낸 작업 수 / (지평/3600).
- task_p50_s / task_p95_s: 끝낸 작업의 (하역 끝 − 게시) 분위수 (끝나지 않은 작업은 extras.latency_rmean 이 지평에서
  잘라 반영한다).
- deadlocks / unresolved: TrafficManager.stats (확정 교착 사건 · UNRESOLVED 알림 수).
- gate_wait_s: 주행 단계에서 교통 hold(구역 토큰 · 교차 양보 · 사건 hold, 포켓으로 가는 양보 주행 제외) 를 받은 시간 합.
- distance_m: 로봇 주행 거리 합 (도킹 전진 · 후진 1.55 m × 2 는 넣지 않는다).
"""

from __future__ import annotations

import copy
import dataclasses
import hashlib
import json
import math
import time
from typing import Dict, List, Optional

import numpy as np

from layopt.sim import ensure_paths
from layopt.sim.actors import ActorField
from layopt.sim.dwell import DwellTracker
from layopt.sim.fleet import ChargerManager, Dispatcher
from layopt.sim.layout import DOCK_IDS, Layout
from layopt.sim.loc import LocEvents, LocModel
from layopt.sim.navmap import NavMap
from layopt.sim.robot import CIRCUM, HALF_L, HALF_W, Robot
from layopt.sim.traffic import E1TrafficManager, load_config, UPDATE_RATE_HZ
from layopt.sim.workload import initial_soc, p5_tasks, poisson_tasks, Task

ensure_paths()
from amr_fleet.deadlock import strongly_connected_components  # noqa: E402
from amr_fleet.fleetsim.geometry import box_of_polygon, rect_box_overlap  # noqa: E402
from amr_fleet.traffic_manager import RobotObservation  # noqa: E402

# 기준 도착률 [1/s] (부하 1.0). 기준 묶음 · 시드 100–119 에서 비유휴 이용률(1 − 대기 슬롯 체류/(5×지평))이
# 0.75 에 가장 가까운 값: λ=1/70 → 0.757 (1/80 → 0.676, 1/60 → 0.834). e1/20261009T180906Z_baserate ·
# 20261009T181246Z_baserate2. 처음 규칙(작업 보유 이용률)은 0.72 에서 포화해 버렸다 (sim.md §부하)
BASE_RATE = 1.0 / 70.0
TM_PHASE_S = 0.25
TM_LOG_EVENTS = ('traffic/DEADLOCK', 'traffic/RESOLVED', 'traffic/UNRESOLVED', 'traffic/ESCALATED')
TM_LOG_MAX = 300


@dataclasses.dataclass
class SimConfig:
    """실행 설정 (배치 · 시드 밖)."""

    horizon_s: float = 3600.0
    dt: float = 0.1
    load: float = 1.0
    base_rate: float = BASE_RATE
    workload: str = 'poisson'           # poisson | p5
    actors: bool = True
    charging: bool = True
    soc: Optional[float] = None         # 고정 시작 잔량 (None = U(25, 60))
    traffic: bool = True                # False = 교통 관리 없음 (진단)
    loc: LocModel = dataclasses.field(default_factory=LocModel)
    queue_radius: float = 4.0           # 토큰 대기 로봇 → 그 구역 대기열 지점 귀속 반경 (dwell.py)
    # 구간 주행 시간 보정 (sim.md §보정). 1 단계 e1/20261009T180136Z_legfit (잡음 없이 0.5 · 2.0 s,
    # RMS 5.55 → 2.28 s). 2 단계: 경합 없는 구간의 상대 산포(Gazebo IQR 0.976–1.051)에 순항 잡음 하한을
    # 맞추고(0.80 → E1 IQR 0.963–1.034) 그 잡음으로 다시 적합 e1/20261009T180643Z_legfit_lo0.80_wide
    # (RMS 3.26 s, 치우침 +0.85 s). 보정 전 값은 1.5 · 0.0 · 1.0
    omega_curve: float = 1.0            # 곡선 주행 각속도 상한 [rad/s] (navmap.speed_profile)
    nav_settle_s: float = 0.0           # 주행 목표 도착 뒤 정착 시간 [s]
    leg_speed_lo: float = 0.80          # 구간마다 순항 상한 × U(leg_speed_lo, 1) (1.0 = 잡음 없음)
    end_when_done: bool = False         # 모든 작업 완료 + 복귀 뒤 끝 (P5)
    record: bool = False


class World:
    """배치 하나의 공유물 (프로세스 안에서 시드 · 부하끼리 재사용): 지도 · 경로 기억 · TM 원형."""

    def __init__(self, layout: Layout):
        self.layout = layout
        self.navmap = NavMap(layout)
        self.tm_cfg = load_config(layout.zones_yaml)
        tm = E1TrafficManager(self.tm_cfg)
        tm.set_map(self.navmap.occupied, self.navmap.spec)
        self._tm = tm
        zones = tm.zone_map.zones()
        self.zone_ids = [z.zone_id for z in zones]
        self.boxes = [box_of_polygon([tuple(p) for p in z.polygon]) for z in zones]
        self.bx = np.array([[b.xmin, b.ymin, b.xmax, b.ymax] for b in self.boxes]) \
            if self.boxes else np.zeros((0, 4))
        self.shared = {z.zone_id for z in zones if z.same_direction}

    def new_tm(self) -> E1TrafficManager:
        return copy.deepcopy(self._tm)


class E1Sim:
    """실행 1 건."""

    def __init__(self, world: World, cfg: SimConfig, seed: int):
        self.world = world
        self.layout = world.layout
        self.navmap = world.navmap
        self.cfg = cfg
        self.seed = int(seed)
        self.charging_enabled = cfg.charging
        lay = self.layout
        rids = lay.robot_ids
        self.robots: List[Robot] = []
        for r in rids:
            soc = cfg.soc if cfg.soc is not None else initial_soc(self.seed, r)
            self.robots.append(Robot(self, r, lay.slots[r], soc))
        self.by_id = {r.id: r for r in self.robots}
        self.n = len(self.robots)
        if cfg.workload == 'p5':
            self.tasks: List[Task] = p5_tasks()
        else:
            self.tasks = poisson_tasks(self.seed, cfg.base_rate, cfg.load, cfg.horizon_s,
                                       DOCK_IDS)
        self.created = {t.task_id: t.t for t in self.tasks}
        self.dispatch = Dispatcher(lay)
        self.chargers = ChargerManager(lay)
        self.tm = world.new_tm() if cfg.traffic else None
        self.actor_field = ActorField(lay.actors, self.seed, cfg.horizon_s, cfg.dt, cfg.actors)
        na = self.actor_field.n
        self.body_ids = [r.id for r in self.robots] + self.actor_field.names
        self.body_idx = {b: i for i, b in enumerate(self.body_ids)}
        self.hl = np.concatenate([np.full(self.n, HALF_L), self.actor_field.half_len])
        self.hw = np.concatenate([np.full(self.n, HALF_W), self.actor_field.half_wid])
        self.kinds = ['robot'] * self.n + self.actor_field.kinds
        self._held = [False] * (self.n + na)
        self.wait_zones: List[set] = [set() for _ in range(self.n + na)]   # hold 로 기다리는 구역
        self.token_zones: Dict[str, set] = {}                                # 로봇 → 쥔 구역 토큰
        self.bodies = None
        self.robot_th = [0.0] * self.n
        self.dwell = DwellTracker(lay.dwell_points, self.n, queue_radius=cfg.queue_radius)
        self.loc = LocEvents(cfg.loc, self.seed, rids)
        self.counts: Dict[str, int] = {}
        self.completions: Dict[str, float] = {}
        self.assigned: Dict[str, tuple] = {}
        self.loads: Dict[str, float] = {}
        self.tm_events: Dict[str, int] = {}
        self.tm_log: List[tuple] = []
        self.hold_by: Dict[str, float] = {}
        self.hold_eps: List[list] = []
        self._hold_open: Dict[str, list] = {}
        self.cycles: List[dict] = []
        self._cyc_open: Dict[frozenset, dict] = {}
        self.coocc: List[dict] = []
        self._co_open: Dict[str, dict] = {}
        self.min_dist = math.inf
        self.min_at = None
        self.trace: List[tuple] = []
        self.now = 0.0

    # ------------------------------------------------------------------ 로봇이 부르는 것
    def count(self, key: str, n: int = 1) -> None:
        self.counts[key] = self.counts.get(key, 0) + n

    def on_load(self, robot: Robot, now: float) -> None:
        self.loads[robot.task.task_id] = now

    def on_complete(self, robot: Robot, now: float) -> None:
        self.completions[robot.task.task_id] = now
        if self.cfg.record:
            self.trace.append((round(now, 2), 'done', robot.id, robot.task.task_id))

    def body_index(self, bid: str) -> Optional[int]:
        return self.body_idx.get(bid)

    def mask_cells(self, km) -> np.ndarray:
        """교통 관리자의 keepout 마스크 → 지도 격자 bool (같은 격자면 그대로, 아니면 치명 셀 중심을 옮긴다)."""
        nm = self.navmap
        sp = km.spec
        if (sp.width == nm.w and sp.height == nm.h and abs(sp.resolution - nm.res) < 1e-9
                and abs(sp.origin_x - nm.ox) < 1e-6 and abs(sp.origin_y - nm.oy) < 1e-6):
            return km.data >= 100
        out = np.zeros((nm.h, nm.w), dtype=bool)
        pts = km.lethal_points()
        if len(pts):
            cx = np.floor((pts[:, 0] - nm.ox) / nm.res).astype(int)
            cy = np.floor((pts[:, 1] - nm.oy) / nm.res).astype(int)
            ok = (cx >= 0) & (cx < nm.w) & (cy >= 0) & (cy < nm.h)
            out[cy[ok], cx[ok]] = True
        return out

    def hold_tick(self, robot: Robot, dt: float) -> None:
        key = robot.hold_reason.split(':', 1)[0] if robot.hold_reason else 'hold'
        if key == 'zone':
            key = 'zone:' + robot.hold_reason.split(':', 1)[1]
        self.hold_by[key] = self.hold_by.get(key, 0.0) + dt

    # ------------------------------------------------------------------ 실행
    def run(self) -> dict:
        cfg = self.cfg
        wall0 = time.perf_counter()
        dt = cfg.dt
        steps = int(round(cfg.horizon_s / dt))
        tm_every = int(round(1.0 / (UPDATE_RATE_HZ * dt)))
        tm_phase = int(round(TM_PHASE_S / dt))
        arrivals = sorted(self.tasks, key=lambda t: (t.t, t.task_id))
        ai = 0
        end_k = steps
        for k in range(steps):
            t = k * dt
            self.now = t
            while ai < len(arrivals) and arrivals[ai].t <= t + 1e-9:
                self.dispatch.post(arrivals[ai])
                ai += 1
            for r, task in self.dispatch.round(t, self.robots):
                r.start_task(t, task)
                self.assigned[task.task_id] = (t, r.id)
            if self.tm is not None and k % tm_every == tm_phase % tm_every:
                self._traffic(t)
            self._make_bodies(k)
            for r in self.robots:
                r.step(t, dt, k)
            self._after_step(t + dt, k, dt)
            if cfg.end_when_done and ai == len(arrivals) and \
                    len(self.completions) == len(self.tasks) and \
                    all(r.stage == 'idle' for r in self.robots):
                end_k = k + 1
                break
        self.end_t = end_k * dt
        return self._result(time.perf_counter() - wall0)

    def _traffic(self, now: float) -> None:
        obs = {}
        for r in self.robots:
            plan = r.plan_xy()
            obs[r.id] = RobotObservation(r.id, r.x, r.y, r.th, r.v, plan, r.status(),
                                         r.task.task_id if r.task is not None else '',
                                         r.priority(), None, None)
        res = self.tm.update(now, obs)
        for ev in res.events:
            self.tm_events[ev.name] = self.tm_events.get(ev.name, 0) + 1
            if ev.name in TM_LOG_EVENTS and len(self.tm_log) < TM_LOG_MAX:
                self.tm_log.append((round(now, 1), ev.name.split('/')[-1], ev.message[:120]))
            if self.cfg.record:
                self.trace.append((round(now, 2), 'tm', ev.name, ev.message))
        for rid, cmd in res.commands.items():
            self.by_id[rid].apply_command(now, cmd)
        self.token_zones = {r: set(z) for r, z in self.tm.tokens.holdings().items()}

    def _make_bodies(self, k: int) -> None:
        n = self.n
        rx = np.array([[r.x, r.y] for r in self.robots])
        rv = np.array([[r.v * math.cos(r.th), r.v * math.sin(r.th)] for r in self.robots])
        for i, r in enumerate(self.robots):
            self.robot_th[i] = r.th
            self._held[i] = r.hold and not r.yielding
            zoned = self._held[i] and r.hold_reason.startswith('zone:')
            self.wait_zones[i] = set(r.hold_reason[5:].split('+')) if zoned else set()
        if self.actor_field.n:
            axy, av = self.actor_field.at(k)
            bx = np.concatenate([rx, axy])
            bv = np.concatenate([rv, av])
        else:
            bx, bv = rx, rv
        self.bodies = (bx, bv, self.body_ids, self.hl, self.hw, self.kinds, self._held)
        del n

    def _after_step(self, now: float, k: int, dt: float) -> None:
        xy = np.array([[r.x, r.y] for r in self.robots])
        v = np.array([r.v for r in self.robots])
        forced = [r.dwell_place() for r in self.robots]
        hz = [r.hold_reason[5:].split('+')
              if r.hold and not r.yielding and r.hold_reason.startswith('zone:') else None
              for r in self.robots]
        bb = [self.body_idx.get(r.blocked_by, -1) if r.blocked_by is not None else -1
              for r in self.robots]
        bb = [b if b < self.n else -1 for b in bb]
        at = self.dwell.step(xy, v, dt, forced, hz, bb)
        if self.loc.model.enabled and at:
            for i, pid, kk in at:
                self.loc.step(now, dt, self.robots[i], pid, kk)
        for r in self.robots:
            if r.stage in ('to_pickup', 'to_drop', 'return', 'to_charger', 'to_queue') and \
                    r.hold and not r.yielding:
                self.hold_tick(r, dt)
                op = self._hold_open.get(r.id)
                if op is None:
                    self._hold_open[r.id] = [round(now - dt, 2), 0.0, r.hold_reason,
                                             round(r.x, 2), round(r.y, 2)]
                else:
                    op[1] += dt
                    op[3], op[4] = round(r.x, 2), round(r.y, 2)
            else:
                op = self._hold_open.pop(r.id, None)
                if op is not None:
                    op[1] = round(op[1] + 0.0, 2)
                    self.hold_eps.append(op)
        if k % 5 == 0:
            self._oracle(now, xy)

    def _oracle(self, now: float, xy: np.ndarray) -> None:
        if self.n > 1:
            d = np.hypot(xy[:, None, 0] - xy[None, :, 0], xy[:, None, 1] - xy[None, :, 1])
            d[np.diag_indices(self.n)] = np.inf
            m = float(d.min())
            if m < self.min_dist:
                i, j = np.unravel_index(int(d.argmin()), d.shape)
                self.min_dist = m
                self.min_at = (round(now, 1), self.robots[int(i)].id, self.robots[int(j)].id,
                               round(float(xy[i, 0]), 2), round(float(xy[i, 1]), 2),
                               self.robots[int(i)].stage, self.robots[int(j)].stage)
        if self.cfg.record:
            self.trace.append((round(now, 1), 'pos', [(r.id, round(r.x, 2), round(r.y, 2),
                                                       round(r.v, 2), r.stage, r.hold,
                                                       r.blocked_by) for r in self.robots]))
        # 물리 대기 그래프 (몸체에 막혀 선 로봇 → 막는 로봇)
        adj = {r.id: [] for r in self.robots}
        for r in self.robots:
            if r.blocked_by in adj:
                adj[r.id].append(r.blocked_by)
        comps = [frozenset(c) for c in strongly_connected_components(adj) if len(c) >= 2]
        live = set(comps)
        for c in list(self._cyc_open):
            if c not in live:
                self._cyc_open.pop(c)['t1'] = now
        for c in comps:
            if c not in self._cyc_open:
                rec = {'members': sorted(c), 't0': now, 't1': None}
                self._cyc_open[c] = rec
                self.cycles.append(rec)
        # 구역 동시 점유 (실제 차체, 용량 1 구역)
        bx = self.world.bx
        if not len(bx):
            return
        ddx = np.maximum(np.maximum(bx[None, :, 0] - xy[:, None, 0], 0.0),
                         xy[:, None, 0] - bx[None, :, 2])
        ddy = np.maximum(np.maximum(bx[None, :, 1] - xy[:, None, 1], 0.0),
                         xy[:, None, 1] - bx[None, :, 3])
        near = np.hypot(ddx, ddy) < CIRCUM
        occ: Dict[str, List[str]] = {}
        for i, j in zip(*np.nonzero(near)):
            r = self.robots[int(i)]
            if rect_box_overlap(r.x, r.y, r.th, self.world.boxes[int(j)]):
                occ.setdefault(self.world.zone_ids[int(j)], []).append(r.id)
        for z in list(self._co_open):
            if len(occ.get(z, ())) < 2:
                self._co_open.pop(z)['t1'] = now
        for z, rs in occ.items():
            if len(rs) >= 2 and z not in self._co_open:
                rec = {'zone': z, 't0': now, 't1': None, 'robots': sorted(rs),
                       'shared': z in self.world.shared}
                self._co_open[z] = rec
                self.coocc.append(rec)

    # ------------------------------------------------------------------ 결과
    def _result(self, wall: float) -> dict:
        cfg = self.cfg
        H = cfg.horizon_s
        lat = sorted(tc - self.created[t] for t, tc in self.completions.items())
        for op in self._hold_open.values():
            self.hold_eps.append(op)
        for c in self.cycles:
            if c['t1'] is None:
                c['t1'] = self.end_t
        for c in self.coocc:
            if c['t1'] is None:
                c['t1'] = self.end_t
        phys = [c for c in self.cycles if c['t1'] - c['t0'] >= 1.0]
        tm = self.tm
        tms = tm.stats if tm is not None else {'deadlocks': 0, 'unresolved': 0}
        row = {
            'spec_id': self.layout.spec_id, 'seed': self.seed, 'load': cfg.load,
            'horizon_s': H,
            'completed': len(self.completions),
            'throughput_per_h': round(len(self.completions) * 3600.0 / H, 3),
            'task_p50_s': _r(_pct(lat, 0.5)), 'task_p95_s': _r(_pct(lat, 0.95)),
            'deadlocks': int(tms['deadlocks']), 'unresolved': int(tms['unresolved']),
            'gate_wait_s': round(sum(r.st['hold_s'] for r in self.robots), 2),
            'distance_m': round(sum(r.st['distance_m'] for r in self.robots), 2),
            'dwell': self.dwell.rows(),
            'loc_events': {'lost': self.loc.lost, 'misconverge': self.loc.misconverge,
                           'recovery_s': round(self.loc.recovery_s, 2)},
        }
        rm = [min(self.completions.get(t, H), H) - c for t, c in self.created.items()]
        busy = sum(r.st['busy_s'] for r in self.robots)
        extras = {
            'spec_id': self.layout.spec_id, 'seed': self.seed, 'load': cfg.load,
            'workload': cfg.workload, 'n_tasks': len(self.tasks), 'end_t': round(self.end_t, 2),
            'unfinished': len(self.tasks) - len(self.completions),
            'pending_end': self.dispatch.pending(),
            'latency_mean': _r(sum(lat) / len(lat)) if lat else None,
            'latency_rmean': _r(sum(rm) / len(rm)) if rm else None,
            'utilization': round(busy / (self.n * self.end_t), 4) if self.end_t > 0 else None,
            'robot': {r.id: {k: (round(v, 2) if isinstance(v, float) else v)
                             for k, v in r.st.items()} | {'soc_end': round(r.soc, 2)}
                      for r in self.robots},
            'hold_by': {k: round(v, 2) for k, v in sorted(self.hold_by.items())},
            'hold_episodes': self.hold_eps,
            'tm_events': dict(sorted(self.tm_events.items())),
            'tm_log': self.tm_log,
            'tm_strategies': dict(tms.get('strategies', {})),
            'tm_resolved': int(tms.get('resolved', 0)),
            'phys_deadlocks': len(phys),
            'phys_deadlock_s': [round(c['t1'] - c['t0'], 1) for c in phys],
            'phys_unresolved': sum(1 for c in phys if c['t1'] >= self.end_t - 1e-6),
            'cooccupancy': [{'zone': c['zone'], 't0': round(c['t0'], 1),
                             'dur': round(c['t1'] - c['t0'], 1), 'robots': c['robots'],
                             'shared': c['shared']} for c in self.coocc],
            'min_robot_dist_m': (None if not math.isfinite(self.min_dist)
                                 else round(self.min_dist, 3)),
            'min_robot_dist_at': self.min_at,
            'counts': dict(sorted(self.counts.items())),
            'charger_claims': self.chargers.claims, 'charger_waits': self.chargers.waits,
            'plans': self.navmap.plans,
            'completions': {t: round(v, 2) for t, v in sorted(self.completions.items())},
            'assigned': {t: [round(a, 2), r] for t, (a, r) in sorted(self.assigned.items())},
            'loc_log': self.loc.log,
            'unattributed_hold_s': round(self.dwell.unattributed_hold_s, 2),
            'wall_s': round(wall, 3),
        }
        extras['digest'] = hashlib.sha256(json.dumps(
            [row['completed'], extras['completions'], row['gate_wait_s'], row['distance_m'],
             [(r.id, round(r.x, 6), round(r.y, 6)) for r in self.robots]],
            sort_keys=True).encode()).hexdigest()[:16]
        if cfg.record:
            extras['trace'] = self.trace
        return {'row': row, 'extras': extras}


def _pct(xs: List[float], q: float) -> Optional[float]:
    if not xs:
        return None
    k = min(len(xs) - 1, max(0, int(math.ceil(q * len(xs))) - 1))
    return xs[k]


def _r(x: Optional[float], nd: int = 2) -> Optional[float]:
    return None if x is None else round(float(x), nd)


def run(world: World, cfg: SimConfig, seed: int) -> dict:
    """실행 1 건 → {'row': contract §4 행, 'extras': 진단}."""
    return E1Sim(world, cfg, seed).run()


__all__ = ['BASE_RATE', 'E1Sim', 'SimConfig', 'World', 'run']
