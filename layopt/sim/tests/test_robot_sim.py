"""로봇 운동학 · 몸체 막힘 · 충전 · 체류 · 측위 훅 · 실행 결과 형식 · 결정성."""

import math

import numpy as np
import pytest

from conftest import needs_core
from layopt.sim.dwell import DwellTracker
from layopt.sim.layout import DwellPoint
from layopt.sim.loc import LocModel
from layopt.sim.navmap import NavPath
from layopt.sim.robot import (
    BATTERY_RESUME, CHARGING, GAP_LON, HALF_L, IDLE, QUEUED, RETURN, TO_CHARGER, V_PASS,
)
from layopt.sim.sim import E1Sim, SimConfig

CONTRACT_KEYS = {'spec_id', 'seed', 'load', 'horizon_s', 'completed', 'throughput_per_h',
                 'task_p50_s', 'task_p95_s', 'deadlocks', 'unresolved', 'gate_wait_s',
                 'distance_m', 'dwell', 'loc_events'}


def _sim(world, **kw):
    cfg = SimConfig(workload='p5', horizon_s=kw.pop('horizon_s', 600), soc=kw.pop('soc', 100.0),
                    actors=kw.pop('actors', False), traffic=kw.pop('traffic', False), **kw)
    sim = E1Sim(world, cfg, 0)
    sim.tasks = []
    sim._k = 0
    return sim


def _advance(sim, seconds, watch=None):
    dt = sim.cfg.dt
    out = []
    for _ in range(int(round(seconds / dt))):
        k = sim._k
        t = k * dt
        sim.now = t
        sim._make_bodies(k)
        for r in sim.robots:
            r.step(t, dt, k)
        sim._after_step(t + dt, k, dt)
        sim._k += 1
        if watch is not None:
            out.append(watch())
    return out


def _park_all(sim, keep=()):
    """남길 로봇(keep) 밖은 지도 구석(서로 멀리)으로 치운다."""
    for i, r in enumerate(sim.robots):
        if r.id not in keep:
            r.x, r.y, r.th = -28.0 + 1.5 * i, -19.0, 0.0


@needs_core
def test_leg_time_close_to_length(world):
    sim = _sim(world)
    r = sim.by_id['amr_01']
    goal = sim.layout.docks['dock_1']
    r._nav(0.0, goal, RETURN)
    L = r.path.length
    stages = _advance(sim, 120, lambda: r.stage)
    t_arr = stages.index(IDLE) * sim.cfg.dt
    assert L / 1.0 < t_arr < L / 1.0 + 15.0
    assert abs(r.x - goal[0]) < 1e-6 and abs(r.y - goal[1]) < 1e-6
    assert abs(math.remainder(r.th - goal[2], 2 * math.pi)) <= 0.05


@needs_core
def test_hold_stops_within_braking_distance(world):
    sim = _sim(world)
    r = sim.by_id['amr_01']
    r._nav(0.0, sim.layout.docks['dock_1'], RETURN)
    _advance(sim, 15)
    assert r.v > 0.9
    s0 = r.s
    r.hold = True
    vs = _advance(sim, 5, lambda: r.v)
    assert vs[-1] == 0.0 and r.s - s0 <= 0.5 + 0.11
    s1 = r.s
    _advance(sim, 5)
    assert r.s == s1
    r.hold = False
    _advance(sim, 3)
    assert r.s > s1 + 0.5


def _driver(sim, rid, start, goal):
    """직선 경로로 주행 (계획기와 무관하게 몸체 규칙만 본다)."""
    r = sim.by_id[rid]
    r.x, r.y, r.th = start[0], start[1], math.atan2(goal[1] - start[1], goal[0] - start[0])
    r.stage, r.goal, r.align_goal = RETURN, (goal[0], goal[1], r.th), False
    r.path, r.s, r.v, r.nav_phase = NavPath([start, goal], r.th), 0.0, 0.0, 'follow'
    return r


def _static(sim, rid, x, y, th, stage=IDLE):
    b = sim.by_id[rid]
    b.x, b.y, b.th, b.stage, b.v = x, y, th, stage, 0.0
    b.path = None
    return b


@needs_core
def test_static_robot_in_narrow_aisle_blocks_then_detours(world):
    sim = _sim(world)
    _park_all(sim, ('amr_01', 'amr_02'))
    b = _static(sim, 'amr_02', 0.0, -10.0, -math.pi / 2)
    a = _driver(sim, 'amr_01', (0.0, -6.0), (0.0, -14.0))
    _advance(sim, 4.5)
    assert a.v == 0.0 and a.blocked_by == 'amr_02'
    assert math.hypot(a.x - b.x, a.y - b.y) >= HALF_L + HALF_L + GAP_LON - 0.05
    _advance(sim, 3.0)                                  # 3 s 넘게 막히면 그 몸체를 칠하고 다시 계획
    assert a.st['detours'] == 1
    _advance(sim, 60)
    assert a.stage == IDLE and math.hypot(a.x, a.y + 14.0) < 1e-6


@pytest.fixture
def no_detour(monkeypatch):
    import layopt.sim.robot as rm
    monkeypatch.setattr(rm, 'DETOUR_AFTER_S', 1e9)


@needs_core
def test_static_robot_in_wide_aisle_is_passed_slowly(world):
    sim = _sim(world)
    _park_all(sim, ('amr_01', 'amr_02'))
    _static(sim, 'amr_02', 0.0, 6.0, 0.0)
    a = _driver(sim, 'amr_01', (-6.0, 6.0), (6.0, 6.0))
    vs = _advance(sim, 30, lambda: (a.x, a.v))
    assert a.stage == IDLE                                         # 도착
    near = [v for x, v in vs if -0.5 < x < 0.5]
    assert near and max(near) <= V_PASS + 1e-9


@needs_core
def test_queue_behind_held_robot_same_direction(world, no_detour):
    sim = _sim(world)
    _park_all(sim, ('amr_01', 'amr_02'))
    b = _driver(sim, 'amr_02', (2.0, 6.0), (10.0, 6.0))
    b.hold = True
    a = _driver(sim, 'amr_01', (-6.0, 6.0), (10.0, 6.0))
    _advance(sim, 20)
    assert a.v == 0.0 and a.x < b.x - (2 * HALF_L + GAP_LON) + 0.05


@needs_core
def test_pass_held_robot_facing_opposite(world):
    sim = _sim(world)
    _park_all(sim, ('amr_01', 'amr_02'))
    b = _driver(sim, 'amr_02', (2.0, 6.0), (-10.0, 6.0))
    b.hold = True
    a = _driver(sim, 'amr_01', (-6.0, 6.0), (10.0, 6.0))
    _advance(sim, 25)
    assert a.x > 5.0


@needs_core
def test_no_overtaking_a_queue(world, no_detour):
    """정지했지만 hold 가 아닌 로봇(줄 선 로봇) 너머에 내 정지 목표가 있으면 그 뒤에 선다 (두 번 보기)."""
    sim = _sim(world)
    _park_all(sim, ('amr_01', 'amr_02', 'amr_03'))
    c = _driver(sim, 'amr_03', (4.0, 6.0), (10.0, 6.0))
    c.hold = True
    _static(sim, 'amr_02', 4.0 - (2 * HALF_L + GAP_LON), 6.0, 0.0)
    a = _driver(sim, 'amr_01', (-6.0, 6.0), (10.0, 6.0))
    _advance(sim, 25)
    b = sim.by_id['amr_02']
    assert a.v == 0.0 and a.x < b.x - 0.5


@needs_core
def test_charge_cycle_and_return(world):
    sim = _sim(world, horizon_s=900)
    r = sim.by_id['amr_03']
    r.battery.percent = 19.0
    r.battery.params.charge_power_w = 48000.0          # 시험용: 충전만 빠르게
    seen = _advance(sim, 600, lambda: r.stage)
    assert TO_CHARGER in seen and CHARGING in seen and RETURN in seen
    assert r.stage == IDLE and r.charger is None
    assert r.soc >= BATTERY_RESUME - 1.0
    assert (r.x, r.y) == sim.layout.slots['amr_03'][:2]


@needs_core
def test_charger_nearest_free_then_queue(world):
    sim = _sim(world)
    cm = sim.chargers
    rs = sim.robots
    got = [cm.claim(r) for r in rs[:3]]
    assert sorted(got) == ['c1', 'c2', 'c3']
    assert cm.claim(rs[3]) is None
    rs[3].start_charge(0.0)
    assert rs[3].stage == 'to_queue'
    _advance(sim, 120)
    assert rs[3].stage == QUEUED
    cm.release(rs[0])
    _advance(sim, 1)
    assert rs[3].stage == TO_CHARGER and rs[3].charger == got[0]


def test_dwell_k_hist():
    pts = [DwellPoint('wait:a', 'waiting', 0.0, 0.0, 0.0, ((2.0, 0.0, 0.0), (4.0, 0.0, 0.0))),
           DwellPoint('wait:b', 'waiting', 2.0, 0.0, 0.0, ((0.0, 0.0, 0.0), (4.0, 0.0, 0.0)))]
    dt = DwellTracker(pts, 3)
    xy = np.array([[0.1, 0.0], [2.2, 0.1], [9.0, 9.0]])
    v = np.array([0.0, 0.0, 0.0])
    at = dt.step(xy, v, 0.5)
    assert sorted(at) == [(0, 'wait:a', 1), (1, 'wait:b', 1)]
    xy[1] = [2.0, 0.6]                                  # 이웃 자세에서 0.6 m — 이웃 아님, 체류도 아님
    dt.step(xy, v, 0.5)
    xy[1] = [4.3, 0.0]                                  # a 의 둘째 이웃 자세
    dt.step(xy, np.array([0.0, 0.2, 0.0]), 0.5)         # 1 번은 움직이는 중이어도 이웃으로 센다
    rows = {r['id']: r['k_hist_s'] for r in dt.rows()}
    assert rows['wait:a'] == {'0': 0.5, '1': 1.0, '2': 0.0}
    assert rows['wait:b'] == {'0': 0.0, '1': 0.5, '2': 0.0}
    v2 = np.array([0.1, 0.0, 0.0])                       # 움직이면 체류 아님
    assert dt.step(np.array([[0.0, 0.0], [9, 9], [9, 9]]), v2, 0.5) == []


@needs_core
def test_loc_hook_default_zero_and_table(world):
    sim = _sim(world, horizon_s=300)
    _advance(sim, 60)
    assert sim.loc.lost == 0
    model = LocModel.from_table({('kind:wait', 0): (60.0, 1.0), ('kind:wait', 1): (60.0, 1.0),
                                 ('kind:wait', 2): (60.0, 1.0), ('kind:wait', 3): (60.0, 1.0),
                                 ('kind:wait', 4): (60.0, 1.0)}, reinit_s=10.0,
                                failed_stop_s=20.0)
    sim2 = _sim(world, horizon_s=300, loc=model)
    _advance(sim2, 5)
    assert sim2.loc.lost >= 1 and sim2.loc.misconverge == sim2.loc.lost
    r = next(r for r in sim2.robots if r.frozen_until > sim2.now)
    assert not r.available(sim2.now) and r.status() == 5     # ST_ERROR (FAILED)


@needs_core
def test_p5_run_contract_row_and_determinism(world):
    cfg = SimConfig(workload='p5', horizon_s=900, soc=100.0, end_when_done=True)
    a = E1Sim(world, cfg, 3).run()
    b = E1Sim(world, cfg, 3).run()
    assert a['extras']['digest'] == b['extras']['digest']
    row = a['row']
    assert set(row) == CONTRACT_KEYS
    assert row['completed'] == 5
    assert [d['id'] for d in row['dwell']] == [p.id for p in world.layout.dwell_points]
    assert all(set(d['k_hist_s']) == {'0', '1', '2', '3', '4'} for d in row['dwell'])
    assert set(row['loc_events']) == {'lost', 'misconverge', 'recovery_s'}
    c = E1Sim(world, cfg, 4).run()
    assert c['extras']['digest'] != a['extras']['digest']   # 다른 시드 = 다른 actor 위상 · 도킹 시간
