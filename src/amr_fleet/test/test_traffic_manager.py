"""
traffic_manager: 설정 · 관측 → 명령/이벤트 한 주기, 작은 운동학 시뮬레이터로 끝까지 돌려 보는 시험.

지도는 test_traffic_resolution 과 같은 20 x 10 m (서 · 동 방, 1차선 통로 y 4.5~5.5, 남쪽 우회 도로).
MiniSim: 로봇은 경유점을 1 m/s 로 따라가고, hold 면 서고, yield_pose 가 오면 포켓으로 곧장 가서 기다리며,
풀리면 원래 목표로 돌아가고, keepout 이 오면 준비된 우회 경로로 바꾼다. 다른 로봇과 0.7 m 안으로
가까워지는 걸음은 딛지 않는다 (safety_node 대용). 주기 0.5 s (2 Hz), 걸음 0.1 s.
"""

import math
import pathlib

import numpy as np
import pytest
import yaml

from amr_fleet.alerts import (
    ALERT_DEADLOCK, TRAFFIC_DEADLOCK, TRAFFIC_ESCALATED, TRAFFIC_RESOLVED, TRAFFIC_STALL,
    TRAFFIC_UNRESOLVED,
)
from amr_fleet.traffic_geometry import GridSpec
from amr_fleet.traffic_manager import (
    LEVEL_ERROR, LEVEL_OK, ST_DOCKING, ST_IDLE, ST_MOVING, RobotObservation, TrafficConfig,
    TrafficManager, occupancy_from_values,
)
from amr_fleet.traffic_resolution import Pocket
from amr_fleet.traffic_zones import zones_from_config

CONFIG_DIR = pathlib.Path(__file__).resolve().parents[1] / 'config'
POCKETS = [Pocket('pw_n', 2.0, 8.0), Pocket('pw_s', 2.0, 2.0), Pocket('pe_n', 18.0, 8.0),
           Pocket('pe_s', 18.0, 2.0)]
CORRIDOR = [{'id': 'corr', 'kind': 'corridor',
             'polygon': [[4, 4.5], [16, 4.5], [16, 5.5], [4, 5.5]]}]


def world_map():
    occ = np.ones((100, 200), dtype=bool)
    occ[5:95, 5:40] = False
    occ[5:95, 160:195] = False
    occ[45:55, 40:160] = False
    occ[10:30, 40:160] = False
    return occ, GridSpec(0.1, 0.0, 0.0, 200, 100)


def make_tm(tokens=True, crossing=True, pockets=POCKETS, zones=CORRIDOR, **over):
    cfg = TrafficConfig()
    cfg.zones.tokens = tokens
    cfg.prediction.crossing_hold = crossing
    cfg.resolution.pocket_search_radius_m = 20.0
    for k, v in over.items():
        grp, _, name = k.rpartition('.')
        setattr(getattr(cfg, grp) if grp else cfg, name, v)
    tm = TrafficManager(cfg, zones_from_config(zones, True), list(pockets))
    occ, spec = world_map()
    tm.set_map(occ, spec)
    return tm


class Bot:
    def __init__(self, rid, route, prio, detour=None, dwell=0.0):
        self.rid = rid
        self.x, self.y = map(float, route[0])
        self.route = [tuple(map(float, p)) for p in route[1:]]
        self.goal = self.route[-1]
        self.prio = prio
        self.detour = detour
        self.hold = False
        self.pocket = None
        self.keepout = None
        self.done = False
        self.speed = 0.0
        self.yaw = math.atan2(self.route[0][1] - self.y, self.route[0][0] - self.x)
        self.done_time = None

    def waypoints(self):
        return [self.pocket[:2]] if self.pocket is not None else self.route

    def obs(self, t):
        pts = self.waypoints()
        plan = np.array([(self.x, self.y)] + list(pts)) if pts and not self.done else None
        return RobotObservation(self.rid, self.x, self.y, self.yaw, self.speed, plan,
                                ST_IDLE if self.done else ST_MOVING, '' if self.done else 't',
                                -1 if self.done else self.prio, None, t)


class MiniSim:
    def __init__(self, tm, bots):
        self.tm = tm
        self.bots = {b.rid: b for b in bots}
        self.t = 0.0
        self.events = []
        self.holds = {b.rid: [] for b in bots}
        self.min_gap = math.inf

    def apply(self, rid, cmd):
        b = self.bots[rid]
        if cmd.hold != b.hold or (cmd.yield_pose is not None) != (b.pocket is not None):
            self.holds[rid].append((round(self.t, 1), cmd.hold, cmd.reason))
        was_away = b.pocket is not None
        b.hold = cmd.hold
        b.pocket = cmd.yield_pose if cmd.hold else None
        if was_away and b.pocket is None and not b.done:
            b.route = [b.goal]                                # 포켓에서 원래 목표로
        if cmd.keepout is not None and b.keepout is None and b.detour is not None:
            b.route = list(b.detour)                          # 대체 경로 (Nav2 KeepoutFilter 대용)
        b.keepout = cmd.keepout

    def step_bot(self, b, dt=0.1):
        b.speed = 0.0
        if b.done or (b.hold and b.pocket is None):
            return
        pts = self.waypoints_for(b)
        if not pts:
            if b.pocket is None:
                b.done, b.done_time = True, self.t
            return
        tx, ty = pts[0]
        d = math.hypot(tx - b.x, ty - b.y)
        step = min(d, dt)
        nx = b.x + (tx - b.x) / d * step if d > 1e-9 else tx
        ny = b.y + (ty - b.y) / d * step if d > 1e-9 else ty
        for o in self.bots.values():
            if o is b:
                continue
            dn = math.hypot(nx - o.x, ny - o.y)
            if dn < 0.7 and dn < math.hypot(b.x - o.x, b.y - o.y):
                return                                          # 안전 정지
        if step > 1e-9:
            b.yaw = math.atan2(ny - b.y, nx - b.x)
        b.x, b.y, b.speed = nx, ny, step / dt
        if math.hypot(tx - b.x, ty - b.y) < 1e-6 and b.pocket is None:
            b.route.pop(0)                                      # 포켓에 닿으면 그 자리에서 기다린다

    def waypoints_for(self, b):
        if b.pocket is not None:
            return [] if math.hypot(b.pocket[0] - b.x, b.pocket[1] - b.y) < 1e-6 \
                else [b.pocket[:2]]
        return b.route

    def run(self, t_max=120.0):
        while self.t < t_max and not all(b.done for b in self.bots.values()):
            if abs(self.t / 0.5 - round(self.t / 0.5)) < 1e-6:
                res = self.tm.update(self.t, {r: b.obs(self.t) for r, b in self.bots.items()})
                self.events += [(round(self.t, 1), e) for e in res.events]
                for rid, cmd in res.commands.items():
                    self.apply(rid, cmd)
            for b in self.bots.values():
                self.step_bot(b)
            bs = list(self.bots.values())
            for i in range(len(bs)):
                for j in range(i + 1, len(bs)):
                    self.min_gap = min(self.min_gap, math.hypot(bs[i].x - bs[j].x,
                                                                bs[i].y - bs[j].y))
            self.t = round(self.t + 0.1, 6)
        return all(b.done for b in self.bots.values())

    def names(self):
        return [e.name for _, e in self.events]


# ---------------------------------------------------------------------- 설정
def test_config_flat_roundtrip_and_errors():
    flat = TrafficConfig().to_flat()
    assert 'resolution.robot_radius' not in flat and 'deadlock.clear_s' in flat
    cfg = TrafficConfig.from_flat(dict(flat, **{'robot_radius': 0.5, 'zones.max_bypass': '3',
                                                'prediction.crossing_hold': 'False',
                                                'mode': 'observe'}))
    assert cfg.resolution.robot_radius == 0.5 and cfg.safety_distance == pytest.approx(1.3)
    assert cfg.zones.max_bypass == 3 and cfg.prediction.crossing_hold is False
    assert cfg.resolution.safety_distance == pytest.approx(1.3)
    for bad in ({'nope': 1}, {'zones.nope': 1}, {'x.y': 1}, {'resolution.robot_radius': 1.0},
                {'zones': 1}, {'zones.tokens': 'maybe'}, {'mode': 'fast'},
                {'zones.source': 'db'}, {'robot_radius': 0.0}, {'zones.max_bypass': -1},
                {'resolution.max_attempts_per_robot': 0}, {'prediction.dt_s': math.nan}):
        with pytest.raises(ValueError):
            TrafficConfig.from_flat(bad)


def test_traffic_yaml_matches_config():
    data = yaml.safe_load((CONFIG_DIR / 'traffic.yaml').read_text())
    params = data['/fleet/traffic_manager_node']['ros__parameters']

    def flatten(d, prefix=''):
        for k, v in d.items():
            if isinstance(v, dict):
                yield from flatten(v, f'{prefix}{k}.')
            else:
                yield f'{prefix}{k}', v
    flat = dict(flatten(params))
    node_only = {'update_rate_hz', 'map_topic', 'frame_id'}
    defaults = TrafficConfig().to_flat()
    assert set(flat) - node_only == set(defaults)
    for k, v in defaults.items():
        assert type(flat[k]) is type(v), k                  # ROS 파라미터 타입 = 기본값 타입
        assert flat[k] == v, k                              # yaml 은 코드 기본값과 같다
    TrafficConfig.from_flat({k: flat[k] for k in defaults})


def test_manager_loads_zones_file_and_describes():
    cfg = TrafficConfig(zones_file=str(CONFIG_DIR / 'traffic_zones.yaml'))
    tm = TrafficManager(cfg)
    assert len(tm.zone_map) == 13 and len(tm.pockets) >= 30
    assert 'narrow_aisle(corridor,yaml' in tm.describe_zones()
    none = TrafficManager(TrafficConfig())
    assert none.describe_zones() == '(없음)' and none.pockets == []
    cfg2 = TrafficConfig(zones_file=str(CONFIG_DIR / 'traffic_zones.yaml'))
    cfg2.zones.source = 'none'
    assert len(TrafficManager(cfg2).zone_map) == 0


def test_set_map_derives_zones_when_source_map():
    cfg = TrafficConfig()
    cfg.zones.source = 'both'
    tm = TrafficManager(cfg, zones_from_config(CORRIDOR, True), [])
    occ, spec = world_map()
    tm.set_map(occ, spec)
    kinds = {z.kind for z in tm.zone_map.zones()}
    assert 'corr' in tm.zone_map and 'corridor' in kinds
    assert tm.tgrid is not None and tm.mask_spec.width == 400        # 0.05 m 마스크
    vals = np.array([[-1, 0, 49, 50, 100]])
    assert occupancy_from_values(vals).tolist() == [[True, False, False, True, True]]


# ---------------------------------------------------------------------- 시뮬레이터 시험
def head_on_bots():
    """A(200) 서→동, B(100) 동→서 가 1차선 통로에서 만난다. B 의 우회로 = 남쪽 도로."""
    a = Bot('A', [(5.0, 5.0), (18.0, 5.0)], 200)
    b = Bot('B', [(15.0, 5.0), (2.0, 5.0)], 100,
            detour=[(16.8, 5.0), (16.8, 2.0), (3.0, 2.0), (2.0, 5.0)])
    return a, b


def test_head_on_in_corridor_resolved_by_yield():
    sim = MiniSim(make_tm(tokens=False), list(head_on_bots()))
    assert sim.run(120.0), sim.names()
    names = sim.names()
    assert names.count(TRAFFIC_DEADLOCK) == 1 and names.count(TRAFFIC_RESOLVED) == 1
    dl = next(e for _, e in sim.events if e.name == TRAFFIC_DEADLOCK)
    assert dl.values['type'] == 'HEAD_ON' and dl.values['victim'] == 'B'
    assert set(dl.values['edges'].split(',')) == {'A→B:block', 'B→A:block'}
    assert dl.values['strategy'] == 'YIELD' and dl.values['pocket'] in ('pe_n', 'pe_s')
    assert dl.alert_name == ALERT_DEADLOCK and dl.level == LEVEL_ERROR
    res = next(e for _, e in sim.events if e.name == TRAFFIC_RESOLVED)
    assert res.level == LEVEL_OK and res.values['strategy'] == 'YIELD'
    assert float(res.values['resolve_time_s']) < 30.0
    assert any(h and 'yield' in why for _, h, why in sim.holds['B'])
    assert not any(h for _, h, _ in sim.holds['A'])
    assert sim.min_gap >= 0.6
    st = sim.tm.resolve_time_stats()
    assert st['count'] == 1 and st['max_s'] == pytest.approx(float(res.values['resolve_time_s']))
    assert sim.tm.stats['strategies']['YIELD'] == 1


def test_head_on_without_pocket_escalates_to_keepout_detour():
    sim = MiniSim(make_tm(tokens=False, pockets=[], **{'resolution.auto_pockets': False}),
                  list(head_on_bots()))
    assert sim.run(150.0), sim.names()
    names = sim.names()
    assert names[:2] == [TRAFFIC_DEADLOCK, TRAFFIC_ESCALATED] and TRAFFIC_RESOLVED in names
    res = next(e for _, e in sim.events if e.name == TRAFFIC_RESOLVED)
    assert res.values['strategy'] == 'ALT_PATH'
    assert sim.bots['B'].keepout is None                   # 해소 뒤 마스크 해제
    assert sim.tm.stats['escalations'] == 1


def test_corridor_token_prevents_head_on():
    a = Bot('A', [(1.5, 5.0), (4.0, 5.0), (16.0, 5.0), (18.5, 8.0)], 200)
    b = Bot('B', [(18.5, 3.0), (16.0, 5.0), (4.0, 5.0), (1.5, 2.0)], 100)
    sim = MiniSim(make_tm(tokens=True), [a, b])
    assert sim.run(120.0), sim.names()
    assert TRAFFIC_DEADLOCK not in sim.names()
    assert any(h and why == 'zone:corr' for _, h, why in sim.holds['B'])
    assert sim.tm.tokens.grant_log[0][1] == 'A'
    assert a.done_time < b.done_time


def test_crossing_lower_priority_holds():
    a = Bot('A', [(11.0, 2.0), (19.0, 2.0)], 200)           # 남쪽 도로 → 동쪽 방, 서→동
    b = Bot('B', [(17.0, 8.0), (17.0, 1.2)], 50)            # 동쪽 방 북→남, 6 s 뒤 (17, 2) 에서 만난다
    sim = MiniSim(make_tm(tokens=False), [a, b])
    assert sim.run(80.0), sim.names()
    assert any(h and why == 'crossing:A' for _, h, why in sim.holds['B'])
    assert TRAFFIC_DEADLOCK not in sim.names() and sim.min_gap >= 0.7 - 1e-6


# ---------------------------------------------------------------------- 스크립트 관측
def ob(rid, x, y, yaw, goal, prio=100, speed=0.0, t=None, status=ST_MOVING, task='t'):
    plan = np.array([(x, y), goal]) if goal is not None else None
    return RobotObservation(rid, x, y, yaw, speed, plan, status, task, prio, None, t)


def test_observe_mode_reports_once_despite_flicker_and_clears_after_progress():
    tm = make_tm(mode='observe')
    ev = []
    t = 0.0
    ax = 9.0
    for k in range(60):                                     # 30 s 대치, 5 s 마다 A 가 0.2 m 흔들림
        jiggle = 0.2 if k % 10 == 5 else 0.0
        ev += tm.update(t, {'A': ob('A', ax + jiggle, 5.0, 0.0, (18.0, 5.0), 200,
                                    speed=0.3 if jiggle else 0.0),
                            'B': ob('B', 9.8, 5.0, math.pi, (2.0, 5.0))}).events
        t += 0.5
    names = [e.name for e in ev]
    assert names.count(TRAFFIC_DEADLOCK) == 1, names
    assert TRAFFIC_RESOLVED not in names and names.count(TRAFFIC_STALL) <= 2
    assert all(e.values['strategy'] == 'none' for e in ev if e.name == TRAFFIC_DEADLOCK)
    # observe 는 명령을 내리지 않는다
    res = tm.update(t, {'A': ob('A', 9.0, 5.0, 0.0, (18.0, 5.0), 200),
                        'B': ob('B', 9.8, 5.0, math.pi, (2.0, 5.0))})
    assert not any(c.hold or c.keepout is not None for c in res.commands.values())
    # B 가 비켜(진전) A 가 지나가면 clear_s 뒤 한 번 RESOLVED
    for k in range(20):
        t += 0.5
        ev += tm.update(t, {'A': ob('A', 9.0 + 0.5 * k, 5.0, 0.0, (18.0, 5.0), 200, speed=1.0),
                            'B': ob('B', 9.8 + 0.5 * k, 8.0, 0.0, (19.0, 8.0), speed=1.0)}).events
    names = [e.name for e in ev]
    assert names.count(TRAFFIC_RESOLVED) == 1 and names.count(TRAFFIC_DEADLOCK) == 1
    assert tm.stats['deadlocks'] == 1 and tm.stats['resolved'] == 1


def test_idle_blocker_active_and_observe():
    for mode in ('active', 'observe'):
        tm = make_tm(tokens=False, mode=mode)
        ev = []
        t = 0.0
        for _ in range(24):                                 # 12 s (전략 1 → 2 전환 전)
            ev += tm.update(t, {
                'A': ob('A', 7.0, 5.0, 0.0, (18.0, 5.0), 200),
                'P': ob('P', 7.8, 5.0, 0.0, None, -1, status=ST_IDLE, task=''),
            }).events
            t += 0.5
        dls = [e for e in ev if e.name == TRAFFIC_DEADLOCK]
        assert len(dls) == 1 and dls[0].values['type'] == 'BLOCKED', (mode, [e.name for e in ev])
        if mode == 'observe':                                # A 가 돌아서 진전하면 한 번 풀림
            for k in range(8):
                ev += tm.update(t, {
                    'A': ob('A', 7.0, 4.0 - 0.4 * k, 0.0, (18.0, 2.0), 200, speed=0.8),
                    'P': ob('P', 7.8, 5.0, 0.0, None, -1, status=ST_IDLE, task=''),
                }).events
                t += 0.5
            assert [e.name for e in ev].count(TRAFFIC_RESOLVED) == 1
            assert tm.stats['resolved'] == 1
        if mode == 'active':
            assert dls[0].values['victim'] == 'P'             # 작업 없는 로봇이 비킨다
            cmds = tm.update(t, {'A': ob('A', 7.0, 5.0, 0.0, (18.0, 5.0), 200),
                                 'P': ob('P', 7.8, 5.0, 0.0, None, -1, status=ST_IDLE,
                                         task='')}).commands
            assert cmds['P'].hold and cmds['P'].yield_pose is not None


def test_livelock_and_single_stall():
    tm = make_tm(tokens=False, crossing=False, **{'deadlock.stall_time_s': 5.0})
    ev = []
    t = 0.0
    for k in range(30):                                     # 둘 다 제자리에서 흔들린다 (진전 없음)
        d = 0.1 if k % 2 else -0.1
        ev += tm.update(t, {
            'A': ob('A', 5.0 + d, 2.0, 0.0, (15.0, 2.0), 200, speed=0.4),
            'B': ob('B', 6.0 - d, 2.8, math.pi, (1.0, 8.0), 100, speed=0.4),
            'C': ob('C', 17.0 + d, 8.0, 0.0, (17.0, 1.0), 50, speed=0.4),
        }).events
        t += 0.5
    names = [e.name for e in ev]
    ll = [e for e in ev if e.name == TRAFFIC_DEADLOCK and e.values['type'] == 'LIVELOCK']
    assert len(ll) == 1 and set(ll[0].values['robots'].split(',')) == {'A', 'B'}
    stalls = [e for e in ev if e.name == TRAFFIC_STALL]
    assert len(stalls) == 1 and stalls[0].robots == ('C',)
    assert TRAFFIC_UNRESOLVED not in names


def test_stale_and_missing_observations_and_busy_status():
    tm = make_tm()
    res = tm.update(10.0, {'A': ob('A', 2.0, 5.0, 0.0, (18.0, 5.0), 200, t=0.0),
                           'B': ob('B', 18.0, 3.0, 0.0, (2.0, 2.0), 100, t=10.0)})
    assert set(res.commands) == {'B'} and set(tm.views) == {'B'}
    tm.update(10.5, {'B': ob('B', 18.0, 3.0, 0.0, (2.0, 2.0), 100, status=ST_DOCKING)})
    assert not tm.views['B'].active                         # 도킹 중 = 정지 작업
    tm.update(11.0, {})
    assert tm.views == {} and tm.still_for('B', 11.0) == 0.0
