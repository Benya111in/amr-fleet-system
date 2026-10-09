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


def test_sweep_helpers(tmp_path):
    from layopt.sim.sweep import done_keys, find_bundles, parse_seeds
    assert parse_seeds('0:3') == [0, 1, 2, 3]
    assert parse_seeds('1,5:6') == [1, 5, 6]
    b = tmp_path / 'v1'
    b.mkdir()
    (b / 'layout.json').write_text('{}')
    (tmp_path / 'junk').mkdir()
    assert find_bundles(f'{tmp_path}/*,current') == [str(b), 'current']
    (tmp_path / 'results.jsonl').write_text(
        json.dumps({'spec_id': 'a', 'seed': 1, 'load': 1.0}) + '\n{"spec_id": "a", "se')
    assert done_keys(tmp_path) == {('a', 1, 1.0)}           # 끊긴 마지막 줄은 무시


def test_stage2_selection_rule():
    from layopt.sim import select_stage2 as s2
    rng = __import__('random').Random(3)
    obj = {f'v1{i}-x{j}': (rng.uniform(40, 50), rng.uniform(0, 3000)) for i in range(5)
           for j in range(30)}
    obj[s2.BASELINE] = (30.0, 5000.0)                    # 지배당하는 기준
    chosen, reasons, info = s2.select(obj, ['v11-x3', 'nope'])
    front = info['front']
    assert all(not any(s2.dominates(obj[o], obj[f]) for o in obj) for f in front)
    par = [s for s, r in reasons.items() if r == 'pareto']
    assert len(par) == s2.N_PARETO and set(front) <= set(par)
    assert reasons[s2.BASELINE] == 'baseline'
    assert reasons['v11-x3'] in ('pareto', 'must') and info['must_missing'] == ['nope']
    assert sum(1 for r in reasons.values() if r == 'random') == s2.N_RANDOM
    assert len(chosen) == len(set(chosen))
    again, _, _ = s2.select(obj, ['v11-x3', 'nope'])
    assert again == chosen                                # 결정적


def test_summarize_paired_and_main_effects(tmp_path):
    from layopt.sim.summarize import summarize

    def row(sid, seed, thr, w):
        return {'spec_id': sid, 'seed': seed, 'load': 1.0, 'throughput_per_h': thr,
                'task_p50_s': 100.0, 'task_p95_s': 200.0, 'gate_wait_s': 5.0, 'deadlocks': 0,
                'unresolved': 0, 'distance_m': 1.0,
                'dwell': [{'id': 'wait:amr_01', 'k_hist_s': {'0': 10.0, '1': w, '2': 0.0}},
                          {'id': 'queue:z:w', 'k_hist_s': {'0': 3.0, '1': 0.0, '2': 0.0}}]}
    rows = [row('v10-v20-v30-v40-v50', s, 40.0 + s, 100.0) for s in range(3)]
    rows += [row('v11-v20-v30-v40-v50', s, 42.0 + s, 50.0 + s) for s in range(3)]
    p = tmp_path / 'results.jsonl'
    p.write_text('\n'.join(json.dumps(r) for r in rows) + '\n')
    doc = summarize(p)
    c = doc['cells']['v11-v20-v30-v40-v50|1']
    assert c['n_seeds'] == 3 and c['W'] == 51.0 and c['queue_s'] == 3.0
    assert c['paired_vs_baseline']['throughput_per_h'] == 2.0
    assert c['paired_vs_baseline']['W'] == -49.0
    assert doc['main_effects']['load=1|V1']['v11:W'] == 51.0
