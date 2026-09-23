"""
traffic_manager: 설정 · 관측 → 명령/이벤트 한 주기, 작은 운동학 시뮬레이터로 끝까지 돌려 보는 시험.

지도는 test_traffic_resolution 과 같은 20 x 10 m (서 · 동 방, 1차선 통로 y 4.5~5.5, 남쪽 우회 도로)와
실제 창고 배치(test_traffic_zones.warehouse_grid — gen_warehouse_world.py 상수) + 배포
config/traffic_zones.yaml · traffic.yaml 기본값 (리뷰 재현 시나리오: 통로 AB 반대 방향, 좁은 통로의
관측 끊긴 보유자, 교차로 안 E-stop · 적재 보유자, 좁은 통로 줄 서기와 교차 양보).
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
    ALERT_DEADLOCK, ALERT_TRAFFIC_STALE, TRAFFIC_DEADLOCK, TRAFFIC_ESCALATED, TRAFFIC_RESOLVED,
    TRAFFIC_STALE, TRAFFIC_STALL, TRAFFIC_UNRESOLVED,
)
from amr_fleet.traffic_geometry import GridSpec
from amr_fleet.traffic_manager import (
    LEVEL_ERROR, LEVEL_OK, LEVEL_WARN, ST_DOCKING, ST_ERROR, ST_ESTOP, ST_IDLE, ST_LOADING,
    ST_MOVING, RobotObservation, TrafficConfig, TrafficManager, _leads_to, occupancy_from_values,
)
from amr_fleet.traffic_resolution import ALT_PATH, Pocket
from amr_fleet.traffic_zones import zones_from_config
from test_traffic_zones import warehouse_grid

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
    def __init__(self, rid, route, prio, detour=None, dwell=0.0, status=None):
        self.rid = rid
        self.status = status          # 고정 상태 (ESTOP · LOADING 등) — 그동안 움직이지 않는다
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
        status = self.status if self.status is not None else ST_IDLE if self.done else ST_MOVING
        return RobotObservation(self.rid, self.x, self.y, self.yaw, self.speed, plan,
                                status, '' if self.done else 't',
                                -1 if self.done else self.prio, None, t)


class MiniSim:
    def __init__(self, tm, bots):
        self.tm = tm
        self.bots = {b.rid: b for b in bots}
        self.t = 0.0
        self.events = []
        self.holds = {b.rid: [] for b in bots}
        self.min_gap = math.inf
        self.on_tick = None                                     # (t, TickResult) 기록용

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
        if b.done or (b.hold and b.pocket is None) or b.status is not None:
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

    def run(self, t_max=120.0, until=None):
        while self.t < t_max and not all(b.done or b.status is not None
                                         for b in self.bots.values()):
            if until is not None and until(self):
                return False
            if abs(self.t / 0.5 - round(self.t / 0.5)) < 1e-6:
                res = self.tm.update(self.t, {r: b.obs(self.t) for r, b in self.bots.items()})
                if self.on_tick is not None:
                    self.on_tick(self.t, res)
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
        return all(b.done or b.status is not None for b in self.bots.values())

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
    assert len(tm.zone_map) == 17 and len(tm.pockets) >= 30   # 구역 13 + 도크 접근 4
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
    # 오래된 관측은 빼지 않는다 (fail closed): 마지막 자세에 멈춘 장애물, 주행 · 유휴 아님
    assert set(tm.views) == {'A', 'B'} and tm.views['A'].stale and not tm.views['A'].mobile
    assert not tm.views['A'].active and not tm.views['A'].idle and tm.views['A'].path is None
    lost = [e for e in res.events if e.name == TRAFFIC_STALE]
    assert len(lost) == 1 and lost[0].values['state'] == 'lost' and lost[0].level == LEVEL_ERROR
    assert lost[0].alert_name == ''                          # 토큰을 쥐지 않았다 → 알림 없음
    tm.update(10.5, {'B': ob('B', 18.0, 3.0, 0.0, (2.0, 2.0), 100, status=ST_DOCKING)})
    assert not tm.views['B'].active and tm.views['B'].mobile  # 도킹 중 = 정지 작업 (움직일 수는 있다)
    tm.update(11.0, {'B': ob('B', 18.0, 3.0, 0.0, None, 100, status=ST_ESTOP)})
    assert not tm.views['B'].mobile
    tm.update(11.5, {})
    assert tm.views == {} and tm.still_for('B', 11.5) == 0.0   # 관측 목록에서 빠지면 잊는다


def test_config_validation_and_layout_spacing(tmp_path):
    for bad in ({'zones.spacing_check': 'maybe'}, {'lost_timeout_s': 1.0},
                {'deadlock.wait_max_s': 5.0}, {'obs_timeout_s': 0.0}):
        with pytest.raises(ValueError):
            TrafficConfig.from_flat(bad)
    wide = tmp_path / 'wide.yaml'           # 리뷰 finding 1 의 배치: 통로를 따라 2 m 간격 교차로
    wide.write_text('zones:\n'
                    '  - {id: a, kind: intersection, polygon: [[0, 0], [4, 0], [4, 5], [0, 5]]}\n'
                    '  - {id: b, kind: intersection,\n'
                    '     polygon: [[6, 0], [10, 0], [10, 5], [6, 5]]}\n')
    cfg = TrafficConfig(zones_file=str(wide))
    cfg.zones.approach_distance_m = 2.5
    with pytest.raises(ValueError, match='a-b 2.00 m'):
        TrafficManager(cfg)
    cfg.zones.spacing_check = 'warn'
    assert 'a-b' in TrafficManager(cfg).layout_warnings[0]
    cfg.zones.spacing_check, cfg.zones.tokens = 'reject', False     # 토큰을 안 쓰면 검사하지 않는다
    assert TrafficManager(cfg).layout_warnings == []
    cfg.zones.tokens, cfg.mode = True, 'observe'
    assert TrafficManager(cfg).layout_warnings == []
    # 지도 유도 구역은 경고만 (고칠 yaml 이 없다): 통로 두 개가 2 m 간격
    auto = TrafficConfig()
    auto.zones.source, auto.zones.approach_distance_m = 'map', 2.5
    tm = TrafficManager(auto)
    occ = np.ones((40, 200), dtype=bool)
    occ[17:23, 5:195] = False               # y 1.7~2.3 폭 0.6 m 통로
    occ[10:30, 90:130] = False              # 가운데 4 m 방이 통로를 둘로 나눈다 (간격 ≈ 2.5 m)
    tm.set_map(occ, GridSpec(0.1, 0.0, 0.0, 200, 40))
    corridors = [z for z in tm.zone_map.zones() if z.kind == 'corridor']
    assert len(corridors) == 2 and len(tm.layout_warnings) == 1


def test_lost_timeout_matches_fleet_robot_state_timeout():
    fleet = yaml.safe_load((CONFIG_DIR / 'fleet.yaml').read_text())
    params = next(v['ros__parameters'] for v in fleet.values() if 'ros__parameters' in v)
    assert TrafficConfig().lost_timeout_s == params['robot_state_timeout_s']


def test_leads_to():
    chain = {'a': 'b', 'b': 'c', 'x': 'y', 'y': 'x'}
    assert _leads_to(chain, 'a', 'c') and not _leads_to(chain, 'c', 'a')
    assert not _leads_to(chain, 'x', 'q') and _leads_to(chain, 'q', 'q')


# ---------------------------------------------------------------------- 실제 창고 배치 (리뷰 재현)
_WH = {}


def make_wh_tm(**over):
    """배포 traffic_zones.yaml + traffic.yaml 기본값 + gen_warehouse_world.py 상수로 그린 지도."""
    cfg = TrafficConfig(zones_file=str(CONFIG_DIR / 'traffic_zones.yaml'))
    for k, v in over.items():
        grp, _, name = k.rpartition('.')
        setattr(getattr(cfg, grp) if grp else cfg, name, v)
    tm = TrafficManager(cfg)
    if 'map' not in _WH:
        _WH['map'] = warehouse_grid(0.1)
    tm.set_map(*_WH['map'])
    return tm


class Watch:
    """틱마다: 구역 대기(hold zone:*) 중인 로봇의 몸체가 구역 밖인지, token ↔ token 2-사이클이 없는지."""

    def __init__(self, sim):
        self.sim = sim
        self.inside_waits = []
        self.token_cycles = []
        self.zone_holds = []
        sim.on_tick = self

    def __call__(self, t, res):
        views = self.sim.tm.views
        for rid, c in res.commands.items():
            if c.hold and c.reason.startswith('zone:'):
                self.zone_holds.append((t, rid, c.reason))
                if views[rid].zones:
                    self.inside_waits.append((t, rid, sorted(views[rid].zones)))
        tok = {(a, b) for a, b, why in res.wait_for if why == 'token'}
        self.token_cycles += [(t, a, b) for a, b in tok if (b, a) in tok]


def test_warehouse_opposite_directions_and_crossing_on_aisle_ab():
    """
    리뷰 finding 1 (t1_aisle): 통로 AB 반대 방향 + 열 틈 가로지르기.

    A 서→동 y=5, B 동→서 y=7 이 통로 AB 교차로 여섯 개를 지나고, C 가 열 틈 4 (x=3)를 북→남으로
    가로지른다. 기다림은 모두 구역 밖에서, token ↔ token 사이클 없이, 교착 없이 끝난다.
    """
    done = {}
    for tokens in (True, False):
        a = Bot('A', [(-20.0, 5.0), (20.0, 5.0)], 200)
        b = Bot('B', [(10.0, 7.0), (-20.0, 7.0)], 100)
        c = Bot('C', [(3.0, 16.0), (3.0, -6.0)], 150)
        sim = MiniSim(make_wh_tm(**{'zones.tokens': tokens}), [a, b, c])
        w = Watch(sim)
        assert sim.run(200.0), sim.names()
        assert TRAFFIC_DEADLOCK not in sim.names() and sim.min_gap >= 0.7 - 1e-6
        assert w.inside_waits == [] and w.token_cycles == []
        done[tokens] = max(bb.done_time for bb in (a, b, c))
        if tokens:
            assert w.zone_holds                                  # 교차로 토큰이 실제로 일했다
            assert {r for _, r, _ in w.zone_holds} <= {'A', 'B', 'C'}
            grants = [g for g in sim.tm.tokens.grant_log if g[1] == 'C']
            assert ('x_ab_4', 'x_bc_4') in {g[2] for g in grants}   # 열 틈 두 교차로는 묶음 부여
    assert done[True] <= done[False] + 8.0                       # 대기는 교차로 하나 통과 시간 수준


def _stale_run(t_end, recover_at=None, n_speed=True):
    """S 가 좁은 통로 안 (0, -11) 에 서 있다가 t=5 부터 관측이 끊긴다. N 이 북쪽에서 통로로 온다."""
    tm = make_wh_tm()
    sy, ny, t = -11.0, -4.0, 0.0
    s_last, log, events = None, [], []
    while t <= t_end:
        fresh = t < 5.0 or (recover_at is not None and t >= recover_at)
        if fresh:
            s_last = ob('S', 0.0, sy, math.pi / 2, (0.0, -4.5), 100, t=t)
        n_obs = ob('N', 0.0, ny, -math.pi / 2, (0.0, -16.0), 50, t=t)
        res = tm.update(t, {'S': s_last, 'N': n_obs})
        events += [(t, e) for e in res.events]
        held = res.commands['N'].hold
        step = 0.0 if held else 0.5
        step = max(0.0, min(step, (ny - sy) - 0.7))              # 물리 안전 정지
        ny -= step
        n_obs.speed = step / 0.5
        log.append((t, ny, held, res.commands['N'].reason, tm.tokens.holders('narrow_aisle'),
                    sorted(tm.views['N'].zones)))
        t += 0.5
    return tm, log, events


def test_warehouse_stale_holder_in_narrow_aisle_fails_closed():
    """리뷰 finding 2 (t3_stale): 관측이 끊긴 보유자의 토큰 · 몸체는 남고, 알림을 내고, 사건이 그 로봇을 적는다."""
    tm, log, events = _stale_run(60.0)
    assert all(h == ['S'] for *_, h, _ in log)                  # 토큰은 끝까지 S (lost 뒤에도 몸체 구역은 유지)
    assert all('narrow_aisle' not in z for *_, z in log)        # N 은 통로에 들어가지 않는다
    assert all(held for t, _, held, *_ in log if t >= 3.0)
    stale = [e for _, e in events if e.name == TRAFFIC_STALE]
    assert [e.values['state'] for e in stale] == ['stale', 'lost']
    assert [e.level for e in stale] == [LEVEL_WARN, LEVEL_ERROR]
    assert all(e.alert_name == ALERT_TRAFFIC_STALE and e.values['zones'] == 'narrow_aisle'
               for e in stale)
    assert stale[1].values['released'] == ''
    assert tm.views['S'].stale and tm.views['S'].zones == {'narrow_aisle'}
    dls = [e for _, e in events if e.name == TRAFFIC_DEADLOCK]
    assert len(dls) == 1 and dls[0].values['type'] == 'BLOCKED'  # 되풀이하지 않는다
    assert dls[0].values['immobile'] == 'S' and dls[0].values['victim'] == 'N'
    assert dls[0].values['strategy'] == ALT_PATH                 # N 이 통로를 돌아가게 keepout
    # N 이 새 경로를 받지 못하는 스크립트 → 역전 후보가 없어 UNRESOLVED (S 를 적는다), 한 번만
    unres = [e for _, e in events if e.name == TRAFFIC_UNRESOLVED]
    assert len(unres) == 1 and unres[0].values['immobile'] == 'S'
    assert [e for _, e in events if e.name == TRAFFIC_STALL] == []   # 사건이 이미 알렸다
    # 관측이 돌아오면 recovered (알림 OK)
    tm, log, events = _stale_run(20.0, recover_at=15.0)
    rec = [e for _, e in events if e.name == TRAFFIC_STALE][-1]
    assert rec.values['state'] == 'recovered' and rec.level == LEVEL_OK
    assert rec.alert_name == ALERT_TRAFFIC_STALE and not tm.views['S'].stale


def test_stale_holder_keeps_all_tokens_until_lost():
    """관측이 끊긴(stale) 동안은 들어가지 않은 구역 토큰까지 유지, lost 면 마지막 몸체 구역만 남긴다."""
    tm = make_wh_tm()
    # S: x_ab_4 → 열 틈 → x_bc_4 로 내려가는 중, 두 교차로를 묶음으로 받았다
    s = ob('S', 3.0, 3.8, -math.pi / 2, (3.0, -6.0), 100, speed=1.0, t=0.0)
    tm.update(0.0, {'S': s})
    assert tm.tokens.held_by('S') == ['x_ab_4', 'x_bc_4']
    for t in (1.0, 3.5, 4.5):                                   # t=3.5: stale, 4.5: 아직 stale
        tm.update(t, {'S': s})
        assert tm.tokens.held_by('S') == ['x_ab_4', 'x_bc_4']
    res = tm.update(5.5, {'S': s})                              # lost: 들어가지 않은 x_bc_4 반납
    assert tm.tokens.held_by('S') == ['x_ab_4']
    ev = [e for e in res.events if e.name == TRAFFIC_STALE][0]
    assert ev.values['state'] == 'lost' and ev.values['released'] == 'x_bc_4'
    # 다른 로봇은 x_ab_4 를 계속 기다리고 막는 로봇을 lost 로 적는다
    tm2 = make_wh_tm(**{'deadlock.token_wait_stall_s': 5.5, 'deadlock.idle_block_s': 1e9})
    tm2.update(0.0, {'S': s})
    evs = []
    for k in range(14):
        t = 0.5 * k
        w = ob('W', 0.8, 6.0, 0.0, (10.0, 6.0), 200, t=t)
        res = tm2.update(t, {'S': s, 'W': w})
        evs += res.events
        assert res.commands['W'].hold and res.commands['W'].reason == 'zone:x_ab_4'
    stall = [e for e in evs if e.name == TRAFFIC_STALL]
    assert len(stall) == 1 and stall[0].values['blockers'] == 'S'
    assert stall[0].values['states'] == 'lost' and stall[0].values['type'] == 'TOKEN_WAIT'


def test_warehouse_estop_holder_in_intersection_blocked_then_detour():
    """리뷰 finding 3 (t4_estop): 교차로 안 E-stop 로봇 → BLOCKED (희생 로봇 = 대기 로봇) → 구역을 돌아간다."""
    for status in (ST_ESTOP, ST_ERROR):
        e = Bot('E', [(3.0, 6.0), (20.0, 6.0)], 50, status=status)
        w = Bot('W', [(-6.0, 5.0), (20.0, 5.0)], 200,
                detour=[(1.5, 11.0), (21.0, 11.0), (21.0, 5.0), (20.0, 5.0)])
        sim = MiniSim(make_wh_tm(), [e, w])
        watch = Watch(sim)
        assert sim.run(120.0), sim.names()
        assert sim.names() == [TRAFFIC_DEADLOCK, TRAFFIC_RESOLVED]
        dl = sim.events[0][1]
        assert dl.values['type'] == 'BLOCKED' and dl.values['robots'] == 'E,W'
        assert dl.values['immobile'] == 'E' and dl.values['victim'] == 'W'
        assert dl.values['strategy'] == ALT_PATH and 'x_ab_4' in dl.values['zones']
        assert watch.inside_waits == [] and watch.zone_holds[0][2] == 'zone:x_ab_4'
        assert sim.holds['E'] == []                             # 움직일 수 없는 로봇에는 명령하지 않는다
        assert sim.events[1][1].values['strategy'] == ALT_PATH and w.done


def test_warehouse_loading_holder_token_wait_watchdog():
    """리뷰 finding 3: 교차로 안 적재(LOADING) 보유자 → 대기 감시 STALL → UNRESOLVED (막는 로봇 · 상태)."""
    tm = make_wh_tm(**{'deadlock.token_wait_stall_s': 5.0, 'deadlock.wait_max_s': 10.0})
    evs = []
    for k in range(40):
        t = 0.5 * k
        e = ob('E', 3.0, 6.0, 0.0, (20.0, 6.0), 50, t=t, status=ST_LOADING)
        w = ob('W', 0.8, 5.0, 0.0, (20.0, 5.0), 200, t=t)
        res = tm.update(t, {'E': e, 'W': w})
        evs += [(t, x) for x in res.events]
        assert res.commands['W'].hold and not res.commands['E'].hold
    names = [x.name for _, x in evs]
    assert names == [TRAFFIC_STALL, TRAFFIC_UNRESOLVED]         # BLOCKED 아님 (적재는 끝난다)
    (t1, stall), (t2, unres) = evs
    assert t1 == pytest.approx(5.0, abs=0.6) and t2 == pytest.approx(10.0, abs=0.6)
    assert stall.values['blockers'] == 'E' and stall.values['states'] == 'LOADING'
    assert unres.values['reason'] == 'token_wait_max' and unres.alert_name == ALERT_DEADLOCK
    assert unres.values['robots'] == 'W,E' and tm.stats['unresolved'] == 1
    # 적재가 끝나 E 가 떠나면 W 가 받는다
    res = tm.update(20.5, {'E': ob('E', 12.0, 6.0, 0.0, (20.0, 6.0), 50, t=20.5),
                           'W': ob('W', 0.8, 5.0, 0.0, (20.0, 5.0), 200, t=20.5)})
    assert not res.commands['W'].hold and 'W' in tm.tokens.holders('x_ab_4')


def test_warehouse_narrow_aisle_queue_no_crossing_hold_inside_zone():
    """
    리뷰 finding 4 (t9_queue): 좁은 통로 줄 서기와 교차 양보.

    H 가 좁은 통로를 0.2 m/s 로 북진한 뒤 동쪽으로 꺾고, A · B 는 북쪽 입구 앞에서 통로 토큰을
    기다리며 C 가 그 뒤에 줄 서 있다. 통로 안 H 에 교차 양보를 걸지 않고(줄 선 C 는 제자리로
    예측), 줄 선 로봇을 정체로 세지 않으며, 교착 없이 H 가 빠져나간 뒤 A 가 토큰을 받는다.
    """
    tm = make_wh_tm()
    south = -math.pi / 2
    hx, hy, t, evs, a_granted = 0.0, -11.8, 0.0, [], None
    ys = {'A': -6.8, 'B': -5.9, 'C': -5.0}                      # 줄: A 가 맨 앞
    while t <= 40.0:
        plan = (np.array([(hx, hy), (0.0, -7.6), (5.0, -7.6)]) if hy < -7.6
                else np.array([(hx, hy), (5.0, -7.6)]))
        obs = {'H': RobotObservation('H', hx, hy, math.pi / 2 if hy < -7.6 else 0.0,
                                     0.2 if hy < -7.6 else 1.0, plan, ST_MOVING, 't', 50,
                                     None, t)}
        for rid, prio in (('A', 150), ('B', 120), ('C', 100)):
            obs[rid] = ob(rid, 0.0, ys[rid], south, (0.0, -16.0), prio, t=t)
        res = tm.update(t, obs)
        evs += res.events
        h = res.commands['H']
        assert not h.hold, (t, h.reason)                        # 통로 안 · 입구에서 H 를 세우지 않는다
        if t >= 3.0 and res.commands['A'].hold:                # 요청 거리 밖에서 줄 선 C 는 제자리 예측
            assert not any(c.kind == 'CROSSING' and 'C' in (c.robot_a, c.robot_b)
                           for c in res.conflicts)
        if a_granted is None and not res.commands['A'].hold:
            a_granted = t
        if hy < -7.6:
            hy = min(hy + 0.1, -7.6)
        elif hx < 5.0:
            hx = min(hx + 0.5, 5.0)
        front = None
        for rid in ('A', 'B', 'C'):                             # 풀린 로봇은 앞 로봇과 0.9 m 두고 따라간다
            if not res.commands[rid].hold:
                ys[rid] = max(ys[rid] - 0.5, -15.0 if front is None else front + 0.9)
            front = ys[rid]
        t += 0.5
    assert evs == []                                            # 교착 · 정체 없음
    assert a_granted is not None and 20.0 <= a_granted <= 26.0  # H 몸체가 통로를 나온 뒤
    assert ys['C'] < -8.0                                       # 같은 방향 추종으로 셋 다 통로에 들어갔다


def test_yield_victim_reserves_free_zones_on_its_pocket_route():
    """리뷰 finding 5 (t8): 포켓으로 가는 희생 로봇은 경로 위 빈 구역 토큰을 미리 받고, 남이 쥔 구역은 피한다."""
    east = [Pocket('pe_n', 18.0, 8.0)]

    def run(extra):
        tm = make_tm(pockets=east, **{'resolution.auto_pockets': False,
                                      'resolution.pocket_search_radius_m': 30.0})
        res = None
        for k in range(14):
            t = 0.5 * k
            obs = {'M': ob('M', 3.0, 8.0, -math.pi / 2, (3.0, 1.0), 200, t=t),
                   'P': ob('P', 3.0, 7.0, 0.0, None, -1, t=t, status=ST_IDLE, task='')}
            obs.update(extra(t))
            res = tm.update(t, obs)
        return tm, res
    tm, res = run(lambda t: {})
    inc = tm.incidents.active()[0]
    assert inc.victim == 'P' and inc.pocket.pocket_id == 'pe_n' and res.commands['P'].hold
    assert tm.tokens.holders('corr') == ['P']                  # 통로 토큰을 미리 받았다
    q = ob('Q', 17.0, 5.0, math.pi, (1.0, 5.0), 250, t=7.0)     # 동쪽에서 오는 고우선 로봇은 기다린다
    res = tm.update(7.0, {'M': ob('M', 3.0, 8.0, -math.pi / 2, (3.0, 1.0), 200, t=7.0),
                          'P': ob('P', 3.0, 7.0, 0.0, None, -1, t=7.0, status=ST_IDLE, task=''),
                          'Q': q})
    assert res.commands['Q'].hold and res.token_waits['Q'].blockers == ('P',)
    # 통로 안에 다른 로봇(K)이 있으면 포켓 경로는 통로를 피해 남쪽 도로로 간다
    tm, res = run(lambda t: {'K': ob('K', 10.0, 5.0, 0.0, (15.0, 5.0), 250, t=t)})
    inc = tm.incidents.active()[0]
    route = np.array(inc.pocket.route)
    assert tm.zone_map.zones_at(route).count('corr') == 0 and route[:, 1].min() < 3.0
    assert tm.tokens.holders('corr') == ['K']


def test_alt_path_victim_is_not_held_by_the_token_it_is_masked_out_of():
    """
    ALT_PATH 로 마스크를 받은 로봇은 그 구역 토큰 대기로 세우지 않는다.

    실행기는 교통 hold 동안 주행 goal 을 취소한다 (move_to.xml) — 세워 두면 재계획을 못 해
    "마스크를 피하는 경로"가 영영 나오지 않는다. 통합 시나리오 12 의 강제 교착이 그랬다:
    마스크(5080 셀)를 주고도 replan_grace_s 안에 새 경로가 없어 no_alt_path 로 끝났다.
    구역에 못 들어가게 막는 일은 마스크가 한다.
    """
    from amr_fleet.traffic_resolution import IncidentCommand
    from amr_fleet.traffic_zones import TokenDecision
    tm = make_tm()
    views = {'V': ob('V', 1.1, 5.0, 0.0, (17.0, 5.0), 100, t=0.0)}
    waits = {'V': TokenDecision(False, ('x_ab_4',), ('B',), 3.0)}
    mask = object()

    tm.incidents.commands = lambda: {  # noqa: E731 - 합성 단계만 보는 시험
        'V': IncidentCommand(1, False, None, mask, frozenset({'x_ab_4'}))}
    cmd = tm._compose(views, waits, {})['V']
    assert not cmd.hold and cmd.keepout is mask and cmd.incident_id == 1

    # 마스크가 막지 않는 다른 구역을 기다리는 중이면 토큰 hold 는 그대로다
    waits2 = {'V': TokenDecision(False, ('x_ab_5',), ('B',), 3.0)}
    cmd2 = tm._compose(views, waits2, {})['V']
    assert cmd2.hold and cmd2.reason == 'zone:x_ab_5'
