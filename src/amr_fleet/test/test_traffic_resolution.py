"""
traffic_resolution: 포켓 · keepout 마스크 · 길 존재 확인 · 사건 상태 기계(전략 1 → 2 → 역전 → 실패).

지도 20 x 10 m (0.1 m): 서쪽 방 x 0.5~4, 동쪽 방 x 16~19.5, 가운데 1차선 통로 y 4.5~5.5,
(bypass=True 면) 남쪽 우회 도로 y 1~3. 포켓은 두 방의 네 귀퉁이.
"""

import math

import numpy as np
import pytest

from amr_fleet.traffic_geometry import GridSpec
from amr_fleet.traffic_resolution import (
    ALT_PATH, EV_DEADLOCK, EV_ESCALATED, EV_RESOLVED, EV_UNRESOLVED, LETHAL, YIELD,
    IncidentManager, KeepoutMask, Pocket, ResolutionConfig, RobotView, TraversabilityGrid,
    WorldView, build_keepout_mask, empty_mask, find_yield_pocket, mask_spec_for,
    near_points_mask, pockets_from_config, route_exists,
)
from amr_fleet.traffic_zones import ZoneMap, zones_from_config

RES = 0.1
POCKETS = [Pocket('pw_n', 2.0, 8.0), Pocket('pw_s', 2.0, 2.0), Pocket('pe_n', 18.0, 8.0),
           Pocket('pe_s', 18.0, 2.0)]


def world_map(bypass=True):
    occ = np.ones((100, 200), dtype=bool)
    occ[5:95, 5:40] = False           # 서쪽 방
    occ[5:95, 160:195] = False        # 동쪽 방
    occ[45:55, 40:160] = False        # 1차선 통로
    if bypass:
        occ[10:30, 40:160] = False    # 남쪽 우회 도로
    return occ, GridSpec(RES, 0.0, 0.0, 200, 100)


def cfg(**kw):
    c = ResolutionConfig(pocket_search_radius_m=20.0)
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def line(*pts):
    return np.array(pts, dtype=float)


def view(rid, x, y, goal, prio=100, active=True, idle=False, yaw=0.0, zones=frozenset()):
    path = line((x, y), goal) if active else None
    return RobotView(rid, x, y, yaw, 0.0, path, active, idle, prio, None, frozenset(zones), path)


def make_world(robots, bypass=True, pockets=POCKETS, zone_map=None, c=None):
    occ, spec = world_map(bypass)
    c = c or cfg()
    tg = TraversabilityGrid.from_occupancy(occ, spec, c.robot_radius, 0.25, zone_map,
                                           c.passage_radius)
    return WorldView({r.robot_id: r for r in robots}, zone_map, tg, list(pockets),
                     mask_spec_for(spec, 0.1))


def head_on(a_xy=(8.0, 5.0), b_xy=(9.1, 5.0)):
    """A(우선 200) 서→동, B(100) 동→서, 통로 안에서 마주 섬."""
    return [view('A', *a_xy, (18.0, 5.0), 200), view('B', *b_xy, (2.0, 5.0), 100, yaw=math.pi)]


# ---------------------------------------------------------------------- 설정 · 격자
def test_pockets_from_config():
    ps = pockets_from_config([{'id': 'p1', 'x': 1, 'y': 2}, {'id': 'p2', 'x': 3.5, 'y': 4,
                                                             'yaw': 1.57}])
    assert ps == [Pocket('p1', 1.0, 2.0, 0.0), Pocket('p2', 3.5, 4.0, 1.57)]
    assert pockets_from_config(None) == []
    for bad, msg in [('x', '목록'), ([1], '사전'), ([{'id': 'p', 'x': 1, 'y': 1, 'q': 0}], '모르는'),
                     ([{'id': '', 'x': 1, 'y': 1}], 'id'),
                     ([{'id': 'p', 'x': 1, 'y': 1}, {'id': 'p', 'x': 2, 'y': 2}], '중복'),
                     ([{'id': 'p', 'x': 'a', 'y': 1}], '수여야'), ([{'id': 'p', 'y': 1}], '수여야'),
                     ([{'id': 'p', 'x': math.inf, 'y': 1}], '유한')]:
        with pytest.raises(ValueError, match=msg):
            pockets_from_config(bad)


def test_traversability_grid_keeps_narrow_corridor_passable():
    occ, spec = world_map(bypass=False)
    zm = ZoneMap(zones_from_config([{'id': 'c', 'kind': 'corridor',
                                     'polygon': [[4, 4.5], [16, 4.5], [16, 5.5], [4, 5.5]]}]))
    tg = TraversabilityGrid.from_occupancy(occ, spec, 0.6, 0.25, zm, 0.22)
    ix, iy = tg.spec.world_to_cell(10.0, 5.0)
    assert tg.passable[iy, ix] and not tg.trav[iy, ix]      # 통로: 지나갈 수는 있고 포켓은 못 둔다
    ix2, iy2 = tg.spec.world_to_cell(2.0, 5.0)
    assert tg.trav[iy2, ix2] and not tg.zone_mask[iy2, ix2] and tg.zone_mask[iy, ix]
    plain = TraversabilityGrid(tg.spec, tg.trav)
    assert (plain.passable == plain.trav).all() and not plain.zone_mask.any()


def test_near_points_mask():
    spec = GridSpec(0.5, 0.0, 0.0, 10, 10)
    assert not near_points_mask(spec, [], 1.0).any()
    assert not near_points_mask(spec, [(99.0, 99.0)], 1.0).any()
    m = near_points_mask(spec, [(2.25, 2.25)], 0.5)
    assert m.sum() == 5 and m[4, 4]


# ---------------------------------------------------------------------- 포켓
def test_find_yield_pocket_prefers_reachable_nearest():
    robots = head_on()
    w = make_world(robots)
    c = cfg()
    avoid = [robots[0].intent_path()]
    p = find_yield_pocket(w.tgrid, robots[1].xy, [robots[0].xy], avoid, c, POCKETS)
    assert p is not None and p.pocket_id in ('pe_n', 'pe_s')     # A 몸이 서쪽을 막는다 → 동쪽 방
    assert p.cost_m > 7.0
    # 동쪽 두 포켓을 다른 사건이 예약 → 남쪽 우회로를 돌아 서쪽 방까지는 20 m 안에 못 간다
    p2 = find_yield_pocket(w.tgrid, robots[1].xy, [robots[0].xy], avoid, c, POCKETS,
                           reserved=[(18.0, 8.0), (18.0, 2.0)])
    assert p2 is None or p2.pocket_id == 'auto'
    assert find_yield_pocket(w.tgrid, (99.0, 99.0), [], [], c, POCKETS) is None


def test_find_yield_pocket_auto_when_named_pockets_unusable():
    robots = head_on()
    w = make_world(robots)
    far = [Pocket('far', 0.1, 0.1)]                                 # 벽 안 → 못 쓴다
    p = find_yield_pocket(w.tgrid, robots[1].xy, [robots[0].xy], [robots[0].intent_path()],
                          cfg(), far)
    assert p is not None and p.pocket_id == 'auto'
    assert find_yield_pocket(w.tgrid, robots[1].xy, [robots[0].xy], [],
                             cfg(auto_pockets=False), far) is None
    assert find_yield_pocket(w.tgrid, robots[1].xy, [robots[0].xy], [],
                             cfg(auto_pockets=False), []) is None


# ---------------------------------------------------------------------- keepout 마스크 · 길
def test_build_keepout_mask_band_blockers_and_guards():
    a, b = head_on()
    c = cfg()
    m = build_keepout_mask(b, [a.xy], c)                 # 작은 격자
    assert m is not None and m.lethal_cells > 0
    assert m.blocks(np.array([[8.3, 5.0], [8.0, 5.0]])).all()      # B 앞 분쟁 구간 · A 몸
    assert not m.blocks(np.array([[9.1, 5.0], [2.0, 5.0]])).any()  # B 자기 자리 · 목표는 비운다
    pts = m.lethal_points()
    assert pts.shape[1] == 2 and len(pts) == m.lethal_cells
    full = build_keepout_mask(b, [a.xy], c, spec=mask_spec_for(GridSpec(RES, 0, 0, 200, 100), 0.1))
    assert full.spec.width == 200 and full.blocks(np.array([[8.3, 5.0]])).all()
    # 가드: 상대가 목표 자리에 있음 / 목표가 분쟁 구간 안 / 경로 없음
    assert build_keepout_mask(b, [(2.2, 5.0)], c) is None
    near_goal = view('B', 4.0, 5.0, (3.0, 5.0), yaw=math.pi)
    assert build_keepout_mask(near_goal, [(3.9, 5.0)], c) is None
    assert build_keepout_mask(view('B', 1, 1, None, active=False), [a.xy], c) is None
    # 분쟁 구역 출구까지 칠한다
    zm = ZoneMap(zones_from_config([{'id': 'c', 'kind': 'corridor',
                                     'polygon': [[4, 4.5], [16, 4.5], [16, 5.5], [4, 5.5]]}]))
    mz = build_keepout_mask(b, [a.xy], c, zm, {'c'})
    assert mz.blocks(np.array([[4.5, 5.0], [3.8, 5.0]])).all()
    assert not m.blocks(np.array([[4.5, 5.0]])).any()
    assert not mz.blocks(np.array([[2.0, 5.0], [2.4, 5.0]])).any()  # 출구 너머 연장은 목표 앞에서 자른다
    inside = ZoneMap(zones_from_config([{'id': 'w', 'kind': 'corridor',
                                         'polygon': [[1, 4], [16, 4], [16, 6], [1, 6]]}]))
    assert build_keepout_mask(b, [a.xy], c, inside, {'w'}) is None  # 목표가 분쟁 구역 안
    # 상대가 경로에서 멀면 몸 앞 extend 만
    lone = build_keepout_mask(b, [(9.0, 9.0)], c)
    assert lone.blocks(np.array([[8.0, 5.0]])).all()
    assert not lone.blocks(np.array([[6.0, 5.0]])).any()
    e = empty_mask(lone.spec)
    assert e.lethal_cells == 0 and len(e.lethal_points()) == 0
    assert (m.data.max(), m.data.min()) == (LETHAL, 0)


def test_route_exists_with_and_without_bypass():
    a, b = head_on()
    for bypass, expect in ((True, True), (False, False)):
        w = make_world([a, b], bypass=bypass)
        mask = build_keepout_mask(b, [a.xy], cfg(), spec=w.mask_spec)
        assert route_exists(w.tgrid, b.xy, (2.0, 5.0))
        assert route_exists(w.tgrid, b.xy, (2.0, 5.0), mask) is expect
    assert not route_exists(w.tgrid, (-5.0, 5.0), (2.0, 5.0))
    assert not route_exists(w.tgrid, b.xy, (2.0, 50.0))


# ---------------------------------------------------------------------- 사건 상태 기계
def _names(evs):
    return [e.kind for e in evs]


def test_yield_then_resolved_when_winner_passes():
    im = IncidentManager(cfg())
    a, b = head_on()
    w = make_world([a, b])
    evs = im.open(0.0, ['A', 'B'], 'HEAD_ON', w, {'c'})
    assert _names(evs) == [EV_DEADLOCK]
    ev = evs[0]
    assert ev.values['victim'] == 'B' and ev.values['strategy'] == YIELD
    assert ev.values['pocket'] in ('pe_n', 'pe_s') and ev.values['robots'] == 'A,B'
    assert 'B' in ev.message and 'A' in ev.message
    cmds = im.commands()
    assert set(cmds) == {'B'} and cmds['B'].hold and cmds['B'].yield_pose[0] == 18.0
    assert im.members() == {'A', 'B'} and im.attempts_of('A') == 1
    # A 가 아직 분쟁 영역 → 진행 중
    assert im.step(1.0, make_world(head_on())) == []
    # B 는 포켓, A 는 동쪽 방으로 지나감 → 해소
    b2 = view('B', 18.0, 8.0, (2.0, 5.0), 100)
    a2 = view('A', 17.5, 5.0, (18.0, 5.0), 200)
    evs = im.step(8.0, make_world([a2, b2]))
    assert _names(evs) == [EV_RESOLVED]
    assert evs[0].values['strategy'] == YIELD and evs[0].values['resolve_time_s'] == '8.00'
    assert evs[0].values['pocket'] in ('pe_n', 'pe_s')
    assert im.active() == [] and im.commands() == {}
    assert im.history[-1]['outcome'] == EV_RESOLVED


def test_release_waits_until_no_head_on_prediction():
    """A 는 분쟁 영역을 벗어났지만 B 를 지금 풀면 통로에서 다시 마주친다 → 아직 풀지 않는다."""
    im = IncidentManager(cfg(contested_radius_m=0.5, pass_check_m=1.0))
    a, b = head_on()
    im.open(0.0, ['A', 'B'], 'HEAD_ON', make_world([a, b]))
    a2 = view('A', 12.0, 5.0, (18.0, 5.0), 200)
    b2 = view('B', 15.0, 5.0, (2.0, 5.0), 100, yaw=math.pi)       # 아직 통로 동쪽 끝
    assert im.step(2.0, make_world([a2, b2])) == []
    im.cfg.release_check = False
    assert _names(im.step(2.5, make_world([a2, b2]))) == [EV_RESOLVED]


def test_no_pocket_goes_straight_to_alt_path_and_resolves():
    im = IncidentManager(cfg(auto_pockets=False))
    a, b = head_on()
    evs = im.open(0.0, ['A', 'B'], 'HEAD_ON', make_world([a, b], pockets=[]))
    assert _names(evs) == [EV_DEADLOCK, EV_ESCALATED]
    assert evs[0].values['strategy'] == ALT_PATH and 'pocket' not in evs[0].values
    assert evs[1].values['reason'] == 'no_pocket'
    cmd = im.commands()['B']
    assert not cmd.hold and cmd.keepout is not None and cmd.yield_pose is None
    # B 가 아직 옛 경로(마스크 통과) → 대기, replan_grace_s 전
    assert im.step(2.0, make_world([a, b], pockets=[])) == []
    # B 가 우회 경로로 재계획 (동쪽 → 남쪽 도로 → 서쪽 방), A 는 지나감
    detour = line((9.1, 5.0), (16.5, 5.0), (16.5, 2.0), (3.0, 2.0), (2.0, 5.0))
    b2 = RobotView('B', 9.1, 5.0, 0.0, 0.0, detour, True, False, 100, None, frozenset(),
                   detour)
    a2 = view('A', 17.8, 5.0, (18.0, 5.0), 200)
    evs = im.step(4.0, make_world([a2, b2], pockets=[]))
    assert _names(evs) == [EV_RESOLVED] and evs[0].values['strategy'] == ALT_PATH


def test_persisting_cycle_escalates_then_victim_swap_then_unresolved():
    c = cfg(escalate_after_s=5.0, replan_grace_s=3.0)
    im = IncidentManager(c)
    a, b = head_on()
    w = make_world([a, b])
    im.open(0.0, ['A', 'B'], 'HEAD_ON', w)
    assert im.step(4.0, w) == []
    evs = im.step(5.0, w)                                  # 아무도 못 움직임 → 전략 2
    assert _names(evs) == [EV_ESCALATED] and evs[0].values['reason'] == 'persist'
    assert im.commands()['B'].keepout is not None
    evs = im.step(8.5, w)                                  # 마스크를 피하는 plan 없음 → 희생 로봇 교체
    assert _names(evs) == [EV_ESCALATED]
    assert evs[0].values['victim'] == 'A' and evs[0].values['reason'] == 'no_alt_path'
    assert evs[0].values['strategy'] == YIELD and evs[0].values['pocket'] in ('pw_n', 'pw_s')
    assert im.commands()['A'].hold
    evs = im.step(14.0, w)                                 # A 도 못 비킴 → A 전략 2
    assert _names(evs) == [EV_ESCALATED]
    evs = im.step(17.5, w)                                 # A 도 plan 없음 → 후보 소진
    assert _names(evs) == [EV_UNRESOLVED] and 'exhausted' in evs[0].values['reason']
    assert im.in_cooldown(['B', 'A'], 20.0) and not im.in_cooldown(['A', 'B'], 50.0)
    assert im.attempts_of('A') == 0                        # 실패 뒤 다시 시도할 수 있게 초기화


def test_no_route_on_map_swaps_victim_immediately():
    im = IncidentManager(cfg(auto_pockets=False))
    a, b = head_on()
    evs = im.open(0.0, ['A', 'B'], 'HEAD_ON', make_world([a, b], bypass=False, pockets=[]))
    # 우회로가 없는 지도: B 마스크 → 길 없음 → A 로 역전 → A 도 길 없음 → 실패
    assert _names(evs) == [EV_DEADLOCK, EV_UNRESOLVED]
    hist = im.history[-1]
    assert hist['outcome'] == EV_UNRESOLVED and any('no_route' in x for x in hist['attempts'])


def test_timeouts_and_deadlock_max():
    c = cfg(yield_timeout_s=10.0, alt_path_timeout_s=10.0, deadlock_max_s=25.0,
            escalate_after_s=100.0, replan_grace_s=100.0)
    im = IncidentManager(c)
    a, b = head_on()
    im.open(0.0, ['A', 'B'], 'HEAD_ON', make_world([a, b]))
    a_moved = view('A', 8.8, 5.0, (18.0, 5.0), 200)        # 조금 움직였지만 못 지나감
    w = make_world([a_moved, b])
    assert _names(im.step(10.0, w)) == [EV_ESCALATED]     # yield_timeout
    assert im.active()[0].strategy == ALT_PATH
    evs = im.step(20.0, w)                                 # alt_timeout → A 로 역전
    assert _names(evs)[0] == EV_ESCALATED and evs[0].values['reason'] == 'alt_timeout'
    evs = im.step(26.0, w)
    assert _names(evs) == [EV_UNRESOLVED] and evs[0].values['reason'] == 'deadlock_max'


def test_attempt_limit_and_progress_reset():
    im = IncidentManager(cfg(max_attempts_per_robot=2, auto_pockets=False))
    a, b = head_on()
    w = make_world([a, b], bypass=False, pockets=[])
    im.open(0.0, ['A', 'B'], 'HEAD_ON', w)                # 1: 실패(길 없음)
    im._cooldown.clear()
    im._robot_attempts['A'] = 2
    im._anchor['A'] = 10.0
    evs = im.open(1.0, ['A', 'B'], 'HEAD_ON', w)           # A 가 3번째 → 바로 실패
    assert _names(evs) == [EV_DEADLOCK, EV_UNRESOLVED] and 'attempts(A)' in evs[1].values['reason']
    im._robot_attempts['A'] = 2
    im._anchor['A'] = 10.0
    im.observe_progress(make_world([view('A', 12.0, 5.0, (18.0, 5.0), 200)]))
    assert im.attempts_of('A') == 0                        # 목표 경로가 2 m 넘게 줄었다


def test_parked_blocker_yields_first_then_mover_detours():
    im = IncidentManager(cfg(escalate_after_s=5.0))
    mover = view('B', 9.1, 5.0, (2.0, 5.0), 100, yaw=math.pi)
    parked = view('P', 8.0, 5.0, None, -1, active=False, idle=True)
    w = make_world([mover, parked])
    evs = im.open(0.0, ['B', 'P'], 'BLOCKED', w)
    # 희생 1순위 = 작업 없는 P → 서쪽 방 포켓으로 비킨다
    assert _names(evs) == [EV_DEADLOCK] and evs[0].values['victim'] == 'P'
    assert evs[0].values['pocket'] in ('pw_n', 'pw_s') and im.commands()['P'].hold
    # P 가 움직이지 않는다(유휴 로봇이 양보 명령을 처리하지 못함) → P 는 주행하지 않아 마스크 불가 →
    # B 로 역전 → 상대(P) 가 주차라 비킬 로봇이 없다 → B 가 돌아간다 (전략 2)
    evs = im.step(5.0, w)
    assert _names(evs) == [EV_ESCALATED]
    assert evs[0].values['victim'] == 'B' and evs[0].values['strategy'] == ALT_PATH
    inc = im.active()[0]
    assert inc.victim == 'B' and inc.strategy == ALT_PATH and 'P' not in im.commands()
    detour = line((9.1, 5.0), (16.5, 5.0), (16.5, 2.0), (3.0, 2.0), (2.0, 5.0))
    b2 = RobotView('B', 12.0, 2.0, 0.0, 0.0, detour[2:], True, False, 100, None, frozenset(),
                   detour[2:])
    evs = im.step(20.0, make_world([b2, parked]))
    assert _names(evs) == [EV_RESOLVED]


def test_keepout_mask_type():
    spec = GridSpec(1.0, 0.0, 0.0, 3, 2)
    m = KeepoutMask(spec, np.array([[0, 100, 0], [0, 0, 0]], dtype=np.int8))
    assert m.lethal_cells == 1 and m.lethal_points().tolist() == [[1.5, 0.5]]
    hits = m.blocks(np.array([[1.2, 0.2], [0.2, 0.2], [9.0, 9.0]]))
    assert hits.tolist() == [True, False, False]
    s = mask_spec_for(GridSpec(0.05, -1.0, -2.0, 101, 41), 0.1)
    assert (s.width, s.height, s.origin_x, s.origin_y) == (51, 21, -1.0, -2.0)
    f32 = mask_spec_for(GridSpec(float(np.float32(0.1)), 0.0, 0.0, 200, 100), 0.05)
    assert (f32.width, f32.height) == (400, 200)          # OccupancyGrid 해상도는 float32


# ---------------------------------------------------------------------- 보유 구역 우회 · 움직일 수 없는 로봇
CORR = ZoneMap(zones_from_config([{'id': 'c', 'kind': 'corridor',
                                   'polygon': [[4, 4.5], [16, 4.5], [16, 5.5], [4, 5.5]]}]))


def test_pocket_route_avoids_blocked_zone_cells():
    w = make_world([], zone_map=CORR, c=cfg(auto_pockets=False))
    tg = w.tgrid
    assert tg.zone_ids == ['c'] and tg.zones_mask(['c']).sum() > 0
    assert not tg.zones_mask(['x']).any()
    c = cfg(auto_pockets=False, pocket_search_radius_m=30.0)
    east = [Pocket('pe_n', 18.0, 8.0)]
    p = find_yield_pocket(tg, (3.0, 5.0), [], [], c, east)
    assert p is not None and p.route[0] == (3.0, 5.0) and p.route[-1] == (18.0, 8.0)
    assert 'c' in CORR.zones_at(np.array(p.route))                      # 가장 짧은 길 = 통로
    q = find_yield_pocket(tg, (3.0, 5.0), [], [], c, east, blocked=tg.zones_mask(['c']))
    assert q is not None and 'c' not in CORR.zones_at(np.array(q.route))  # 보유 구역을 피해 남쪽 도로로
    assert q.cost_m > p.cost_m
    closed = make_world([], bypass=False, zone_map=CORR).tgrid
    assert find_yield_pocket(closed, (3.0, 5.0), [], [], c, east,
                             blocked=closed.zones_mask(['c'])) is None
    auto = find_yield_pocket(tg, (3.0, 5.0), [], [], cfg(), [])
    assert auto is not None and auto.pocket_id == 'auto' and len(auto.route) >= 2


def test_keepout_mask_paints_block_zones():
    v = view('W', 2.0, 5.0, (18.0, 5.0), 100)
    spec = mask_spec_for(world_map()[1], 0.1)
    plain = build_keepout_mask(v, [(10.0, 5.0)], cfg(), CORR, (), spec)
    assert not plain.blocks(np.array([[15.5, 5.0]]))[0]                 # 띠는 상대 투영점 + 1.5 m 까지
    m = build_keepout_mask(v, [(10.0, 5.0)], cfg(), CORR, (), spec, block_zones={'c'})
    assert m is not None and m.blocks(np.array([[15.5, 5.0]]))[0]      # 막힌 구역은 끝까지 전부
    inside_goal = view('W', 2.0, 5.0, (12.0, 5.0), 100)
    assert build_keepout_mask(inside_goal, [], cfg(), CORR, (), spec, block_zones={'c'}) is None


def test_immobile_members_are_never_victims_and_failures_are_not_repeated():
    im = IncidentManager(cfg())
    mover = view('W', 2.5, 5.0, (18.0, 5.0), 200)                       # 통로 서쪽 입구 앞에서 대기
    estop = RobotView('E', 10.0, 5.0, 0.0, 0.0, None, False, False, 10, None, frozenset({'c'}),
                      None, mobile=False)
    w = make_world([mover, estop], zone_map=CORR)
    evs = im.open(0.0, ['W', 'E'], 'BLOCKED', w, {'c'})
    # E 의 우선순위가 더 낮아도 E 는 비킬 수 없다 → W 가 희생 로봇, 상대가 모두 못 움직이니 곧바로 전략 2
    assert evs[0].values['victim'] == 'W' and evs[0].values['immobile'] == 'E'
    inc = im.active()[0]
    assert inc.strategy == ALT_PATH and inc.mask.blocks(np.array([[15.5, 5.0]]))[0]
    # 우회 경로가 마스크를 피하면 해소
    detour = line((2.5, 5.0), (3.0, 2.0), (17.0, 2.0), (18.0, 5.0))
    w2 = make_world([RobotView('W', 3.0, 2.0, 0.0, 1.0, detour[1:], True, False, 200, None,
                               frozenset(), detour[1:]), estop], zone_map=CORR)
    assert _names(im.step(1.0, w2)) == [EV_RESOLVED]
    # 모두 못 움직이면 열자마자 UNRESOLVED, E 가 다시 움직일 수 있을 때까지 같은 집합을 다시 열지 않는다
    other = RobotView('F', 8.0, 5.0, 0.0, 0.0, None, False, False, 5, None, frozenset({'c'}),
                      None, mobile=False)
    w3 = make_world([estop, other], zone_map=CORR)
    evs = im.open(2.0, ['E', 'F'], 'BLOCKED', w3)
    assert _names(evs) == [EV_DEADLOCK, EV_UNRESOLVED]
    assert evs[1].values['reason'] == 'no_mobile_victim' and evs[1].values['immobile'] == 'E,F'
    assert im.in_cooldown(['E', 'F'], 1e6) and im.suppressed() == {'E', 'F'}
    im.step(3.0, w3)
    assert im.in_cooldown(['E', 'F'], 1e6)
    estop.mobile = True
    im.step(4.0, w3)                                                    # E 복구 → 다시 시도 가능
    assert not im.in_cooldown(['E', 'F'], 4.0) and im.suppressed() == set()
