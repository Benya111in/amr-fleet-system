"""배치 읽기 · world.sdf 동적 장애물 · 작업 열 공통 난수."""

import json
import math

import numpy as np

from layopt.sim.layout import load_bundle, shuttle_profile, synth_dwell_points
from layopt.sim.workload import initial_soc, LOAD_MAX, p5_tasks, poisson_tasks, SOC_RANGE

DOCKS = ('dock_1', 'dock_2', 'dock_a', 'dock_b')


def test_current_layout_matches_config(layout):
    assert set(layout.docks) == set(DOCKS)
    assert layout.docks['dock_1'] == (-27.78, 17.0, 3.14159265)
    assert sorted(layout.chargers) == ['c1', 'c2', 'c3']
    assert layout.slots['amr_03'] == (22.0, -16.0, 1.5708)
    kinds = sorted(d.kind for d in layout.dwell_points)
    assert kinds.count('waiting') == 5 and kinds.count('dock') == 4 and kinds.count('charger') == 3
    w1 = next(d for d in layout.dwell_points if d.id == 'wait:amr_01')
    assert [n[0] for n in w1.neighbors] == [20.0, 22.0, 24.0, 26.0]     # 가까운 순


def test_world_actors_from_sdf(layout):
    names = {a.name: a for a in layout.actors}
    assert len([a for a in layout.actors if a.kind == 'human']) == 6
    fk = names['forklift_main']
    assert np.allclose(fk.y, 15.0)
    assert fk.x.min() == np.float64(fk.x.min()) and abs(fk.x.min() + 20.0) < 0.2
    assert abs(fk.x.max() - 20.0) < 0.2
    sh = names['shuttle_amr']
    assert np.allclose(sh.x, 9.0) and abs(sh.y.min() + 6.0) < 0.2 and abs(sh.y.max() - 11.0) < 0.2
    ws = names['worker_straight_slow']
    assert abs(ws.x.min() + 16.0) < 0.05 and abs(ws.x.max() - 16.0) < 0.05
    assert np.allclose(ws.y, 6.0)


def test_shuttle_profile_speed_and_accel_limits():
    t, s = shuttle_profile(-20.0, 20.0, -18.9, 1.5, 1.0)
    v = np.diff(s) / np.diff(t)
    assert np.all(np.abs(v) <= 1.5 + 1e-6)
    a = np.diff(v) / np.diff(t)[1:]
    assert np.all(np.abs(a) <= 1.0 + 1e-3)
    assert abs(s.min() + 20.0) < 1e-6 and abs(s.max() - 20.0) < 1e-6


def test_synth_dwell_queue_pose_is_outward():
    d = synth_dwell_points({'dock_1': (-27.78, 17.0, math.pi)}, {}, {}, queue_m=2.5)
    q = d[0].neighbors[0]
    assert abs(q[0] - (-25.28)) < 1e-6 and abs(q[1] - 17.0) < 1e-6


def test_load_bundle_minimal(tmp_path, layout):
    lj = {'spec_id': 'v1x',
          'docks': [{'id': k, 'staging': list(v)} for k, v in layout.docks.items()],
          'chargers': [{'id': k, 'staging': list(v)} for k, v in layout.chargers.items()],
          'waiting_slots': [{'robot': r, 'x': p[0], 'y': p[1], 'yaw': p[2]}
                            for r, p in layout.slots.items()],
          'dwell_points': [{'id': 'wait:amr_01', 'kind': 'waiting', 'x': 18.0, 'y': -16.0,
                            'yaw': 1.57, 'neighbors': [[20.0, -16.0, 1.57]]}]}
    (tmp_path / 'layout.json').write_text(json.dumps(lj))
    (tmp_path / 'valid.json').write_text(json.dumps({'valid': False, 'reasons': ['x']}))
    lay = load_bundle(tmp_path)
    assert lay.spec_id == 'v1x' and not lay.valid and lay.reasons == ('x',)
    assert len(lay.dwell_points) == 1
    assert lay.dwell_points[0].neighbors == ((20.0, -16.0, 1.57),)
    assert lay.actors == []


def test_poisson_stream_common_random_numbers():
    a = poisson_tasks(3, 1 / 60, 1.0, 3600, DOCKS)
    b = poisson_tasks(3, 1 / 60, 1.0, 3600, DOCKS)
    assert a == b
    lo = {t.task_id for t in poisson_tasks(3, 1 / 60, 0.5, 3600, DOCKS)}
    mid = {t.task_id for t in a}
    hi = {t.task_id for t in poisson_tasks(3, 1 / 60, LOAD_MAX, 3600, DOCKS)}
    assert lo <= mid <= hi                                     # 솎아내기: 부하끼리 겹친다
    assert all(t.pickup != t.dropoff for t in a)
    assert a != poisson_tasks(4, 1 / 60, 1.0, 3600, DOCKS)


def test_poisson_rate_is_right():
    n = sum(len(poisson_tasks(s, 1 / 60, 1.0, 3600, DOCKS)) for s in range(200))
    assert abs(n / 200 - 60.0) < 2.5                           # 평균 60 건/h (표준오차 0.55)


def test_initial_soc_range_and_crn():
    vals = [initial_soc(s, 'amr_01') for s in range(500)]
    assert min(vals) >= SOC_RANGE[0] and max(vals) <= SOC_RANGE[1]
    assert initial_soc(7, 'amr_02') == initial_soc(7, 'amr_02')
    assert initial_soc(7, 'amr_02') != initial_soc(7, 'amr_03')


def test_p5_tasks():
    ts = p5_tasks()
    assert [(t.pickup, t.dropoff) for t in ts] == [
        ('dock_1', 'dock_a'), ('dock_a', 'dock_1'), ('dock_2', 'dock_b'), ('dock_b', 'dock_2'),
        ('dock_1', 'dock_b')]
    assert all(t.t == 60.0 and t.mass == 2.0 and t.load_time == 5.0 for t in ts)
