"""
보너스 4 P5 분석 · 계획 (tests/integration/fleet_p5) — 합성 자료로 판정 규칙을 고정한다 (rclpy 불필요).

분석기가 세는 것: 실제 자세 기준 구역 동시 점유(원판 · 중심 · 도크 고정 지표), 믿은 허가 겹침, 허가 없는 진입,
픽업 중복, 완료 · makespan, 장애 반응 시각, 낮은 epoch 수용, 도킹 오차, 부호 검정 · A/A 발산.
"""

import json
import math
import os
from pathlib import Path
import sys

import pytest

P5 = Path(__file__).resolve().parents[1] / 'fleet_p5'
if str(P5) not in sys.path:
    sys.path.insert(0, str(P5))
import p5_analyze as A  # noqa: E402
import p5_plan as P  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
ZONES = """
frame_id: map
zones:
  - id: x_ab_4
    kind: intersection
    polygon: [[2.0, 3.5], [4.0, 3.5], [4.0, 8.5], [2.0, 8.5]]
  - id: dock_a_approach
    kind: intersection
    polygon: [[26.8, 16.25], [29.8, 16.25], [29.8, 17.75], [26.8, 17.75]]
"""
BEHAVIOR = """
/**:
  ros__parameters:
    docks:
      ids: [dock_a]
      dock_a: {staging: [27.78, 17.0, 0.0], standoff: 0.65, marker_id: 2,
               marker_pose: [29.99, 17.0, 3.14159265]}
"""


def write_run(tmp, gt_rows, events, *, mode='distributed', tasks=None, faults=(), t_base=100.0,
              t_end=200.0, why='DONE', exp='C1', pair=1, label='distributed'):
    d = tmp / f'{exp}_{pair}_{label}'
    d.mkdir(parents=True, exist_ok=True)
    plan = {'exp': exp, 'pair': pair, 'label': label, 'mode': mode, 'layout': 'new',
            'robots': ['amr_01', 'amr_02'], 'tasks': tasks or [], 'faults': []}
    (d / 'meta.json').write_text(json.dumps({'plan': plan, 'zones_text': ZONES,
                                             'behavior_text': BEHAVIOR}))
    (d / 'summary.json').write_text(json.dumps({'why': why, 't_base': t_base, 't_end': t_end,
                                                't_ready': t_base - 10,
                                                'hold_first': ['amr_01', 'amr_02']}))
    with open(d / 'events.jsonl', 'w') as f:
        for e in events:
            f.write(json.dumps(e) + '\n')
    with open(d / 'faults.jsonl', 'w') as f:
        for e in faults:
            f.write(json.dumps(e) + '\n')
    with open(d / 'gt.csv', 'w') as f:
        f.write('t,robot,x,y,yaw,v\n')
        for row in gt_rows:
            f.write(','.join(str(v) for v in row) + '\n')
    return d


def track(robot, t0, t1, x0, y0, x1, y1, v=0.5):
    out, n = [], int(round((t1 - t0) / 0.1))
    for i in range(n + 1):
        u = i / n
        out.append((round(t0 + i * 0.1, 3), robot, x0 + u * (x1 - x0), y0 + u * (y1 - y0), 0.0, v))
    return out


# ---------------------------------------------------------------------- 기하
def test_disc_and_center_geometry():
    poly = [[2.0, 3.5], [4.0, 3.5], [4.0, 8.5], [2.0, 8.5]]
    assert A.inside(poly, 3.0, 6.0) and not A.inside(poly, 1.9, 6.0)
    assert A.signed_distance(poly, 1.0, 6.0) == pytest.approx(1.0)
    assert A.signed_distance(poly, 3.0, 6.0) == pytest.approx(-1.0)
    assert A.disc_overlaps(poly, 1.7, 6.0)            # 0.30 m 밖 — 원판(0.36)은 겹친다
    assert not A.disc_overlaps(poly, 1.6, 6.0)        # 0.40 m 밖


# ---------------------------------------------------------------------- 동시 점유
def test_two_robots_in_one_zone_at_once_is_one_event(tmp_path):
    gt = (track('amr_01', 100, 120, 3.0, 4.0, 3.0, 8.0)
          + track('amr_02', 100, 120, 3.0, 8.0, 3.0, 4.0))
    d = write_run(tmp_path, gt, [])
    m = A.run_metrics(str(d))
    ev = [e for e in m['occupancy_disc'] if e['zone'] == 'x_ab_4']
    assert len(ev) == 1 and ev[0]['robots'] == ['amr_01', 'amr_02'] and ev[0]['dur'] >= 19.0
    assert ev[0]['depth_m'] > 0.36
    assert len([e for e in m['occupancy_center'] if e['zone'] == 'x_ab_4']) == 1


def test_one_after_another_is_not_concurrent(tmp_path):
    gt = (track('amr_01', 100, 110, 3.0, 2.0, 3.0, 10.0)
          + track('amr_01', 110.1, 130, 10.0, 10.0, 10, 10)
          + track('amr_02', 100, 112, 3.0, 12.0, 3.0, 12.0)
          + track('amr_02', 112.1, 125, 3.0, 12.0, 3.0, 1.0))
    m = A.run_metrics(str(write_run(tmp_path, gt, [])))
    assert [e for e in m['occupancy_disc'] if e['zone'] == 'x_ab_4'] == []
    assert m['zone_entries'] >= 2


def test_disc_touch_without_center_is_counted_only_by_disc(tmp_path):
    # 두 로봇이 구역 서쪽 경계 밖 0.2 m (중심은 밖, 원판은 겹침)
    gt = (track('amr_01', 100, 110, 1.8, 5.0, 1.8, 5.0, 0)
          + track('amr_02', 100, 110, 1.8, 7.0, 1.8, 7.0, 0))
    m = A.run_metrics(str(write_run(tmp_path, gt, [])))
    assert len(m['occupancy_disc']) == 1 and m['occupancy_center'] == []


def test_fixed_dock_metric_uses_the_old_big_box(tmp_path):
    # y 16.1 — 새 상자(16.25..17.75)에서는 중심 밖, 옛 큰 상자(16..18)에서는 중심 안
    gt = (track('amr_01', 100, 110, 27.0, 16.1, 27.0, 16.1, 0)
          + track('amr_02', 100, 110, 29.0, 16.1, 29.0, 16.1, 0))
    m = A.run_metrics(str(write_run(tmp_path, gt, [])))
    assert [e for e in m['occupancy_center'] if e['zone'] == 'dock_a_approach'] == []
    assert len([e for e in m['occupancy_dock_fixed_center']
                if e['zone'] == 'dock_a_approach']) == 1


# ---------------------------------------------------------------------- 믿은 허가 · 허가 없는 진입
def test_held_overlap_and_unauthorized_entry(tmp_path):
    gt = track('amr_01', 100, 110, 3.0, 2.0, 3.0, 6.0) + track('amr_02', 100, 110, 20, 0, 20, 0, 0)
    ev = [{'t': 100.0, 'k': 'agent_state', 'r': 'amr_01', 'held': [], 'mode': 0},
          {'t': 100.0, 'k': 'agent_state', 'r': 'amr_02', 'held': ['zone:x_bc_1'], 'mode': 0},
          {'t': 105.0, 'k': 'agent_state', 'r': 'amr_01', 'held': ['zone:x_bc_1'], 'mode': 0}]
    m = A.run_metrics(str(write_run(tmp_path, gt, ev)))
    assert 'zone:x_bc_1' in m['held_overlaps']
    assert m['unauth_s'].get('amr_01:x_ab_4', 0) > 1.0
    assert m['unauth_events'][0]['robot'] == 'amr_01'


def test_central_mode_has_no_belief_metrics(tmp_path):
    gt = track('amr_01', 100, 110, 3.0, 2.0, 3.0, 6.0)
    m = A.run_metrics(str(write_run(tmp_path, gt, [], mode='central')))
    assert 'held_overlaps' not in m and m['unauth_s'] == {}


# ---------------------------------------------------------------------- 작업 · 픽업
TASKS = [{'task_id': 'p5c1.00', 'offset_s': 0.0}, {'task_id': 'p5c1.01', 'offset_s': 0.0}]


def test_completion_makespan_and_duplicate_pickup(tmp_path):
    ev = [{'t': 101, 'k': 'task_status', 'r': 'amr_01', 'task_id': 'p5c1.00', 'status': 1},
          {'t': 102, 'k': 'task_status', 'r': 'amr_02', 'task_id': 'p5c1.00', 'status': 1},
          {'t': 110, 'k': 'phase', 'r': 'amr_01', 'phase': 'LOADING'},
          {'t': 111, 'k': 'phase', 'r': 'amr_02', 'phase': 'LOADING'},
          {'t': 150, 'k': 'task_status', 'r': 'amr_01', 'task_id': 'p5c1.00', 'status': 2},
          {'t': 160, 'k': 'task_status', 'r': 'amr_02', 'task_id': 'p5c1.00', 'status': 2}]
    m = A.run_metrics(str(write_run(tmp_path, [], ev, tasks=TASKS)))
    assert m['completed'] == 1 and m['n_tasks'] == 2 and m['makespan'] == 50.0
    assert set(m['duplicate_pickups']) == {'p5c1.00'}
    assert m['completions_dup']['p5c1.00'] and m['pickups'] == 2


def test_single_pickup_is_not_a_duplicate(tmp_path):
    ev = [{'t': 101, 'k': 'task_status', 'r': 'amr_01', 'task_id': 'p5c1.00', 'status': 1},
          {'t': 110, 'k': 'phase', 'r': 'amr_01', 'phase': 'LOADING'},
          {'t': 111, 'k': 'task_status', 'r': 'amr_02', 'task_id': 'p5c1.01', 'status': 1},
          {'t': 120, 'k': 'phase', 'r': 'amr_02', 'phase': 'LOADING'}]
    m = A.run_metrics(str(write_run(tmp_path, [], ev, tasks=TASKS)))
    assert m['duplicate_pickups'] == {} and m['completed'] == 0 and m['makespan'] is None


# ---------------------------------------------------------------------- 장애 · 펜스
def test_fault_response_and_low_epoch_acceptance(tmp_path):
    ev = [{'t': 100, 'k': 'agent_state', 'r': 'amr_01', 'mode': 0, 'fence_epoch': 2, 'held': []},
          {'t': 121.5, 'k': 'agent_state', 'r': 'amr_01', 'mode': 1, 'fence_epoch': 2, 'held': []},
          {'t': 125.2, 'k': 'agent_state', 'r': 'amr_01', 'mode': 2, 'fence_epoch': 2, 'held': []},
          {'t': 130, 'k': 'agent_event', 'r': 'amr_01',
           'ev': {'event': 'assign_proxy', 'accepted': True, 'epoch': 1, 'task_id': 'x'}},
          {'t': 131, 'k': 'agent_event', 'r': 'amr_01',
           'ev': {'event': 'assign_proxy', 'accepted': True, 'epoch': 2, 'task_id': 'y'}},
          {'t': 132, 'k': 'agent_event', 'r': 'amr_01', 'ev': {'event': 'fence_reject'}}]
    kill = [{'action': 'kill', 'scheduled': 20.0, 'sim': 120.0}]
    m = A.run_metrics(str(write_run(tmp_path, [], ev, faults=kill)))
    r = m['fault_response'][0]['amr_01']
    assert r == {'hold_new_s': 1.5, 'degraded_s': 5.2}
    assert [d['task_id'] for d in m['low_epoch_accepted']] == ['x']
    assert m['fence_rejects'] == 1 and m['assign_proxy'] == 2


def test_docking_error_against_marker_and_standoff(tmp_path):
    # 기대: 마커 (29.99, 17) 법선 −x → 판 면 29.98, standoff 0.65 → (29.33, 17), 방위 0
    gt = track('amr_01', 100, 120, 29.33 - 0.01, 17.0, 29.33 - 0.01, 17.0, 0)
    ev = [{'t': 105, 'k': 'phase', 'r': 'amr_01', 'phase': 'DOCKING'},
          {'t': 115, 'k': 'phase', 'r': 'amr_01', 'phase': 'LOADING'}]
    m = A.run_metrics(str(write_run(tmp_path, gt, ev)))
    dk = m['docking']['completed'][0]
    assert dk['dock'] == 'dock_a' and dk['pos_err_m'] == pytest.approx(0.01, abs=1e-3)
    assert m['dock_within_2cm_1deg'] == 1 and m['docking']['docking_entries'] == 1


def test_invalid_run_is_marked(tmp_path):
    m = A.run_metrics(str(write_run(tmp_path, [], [], why='READY_TIMEOUT')))
    assert m['valid'] is False


# ---------------------------------------------------------------------- 통계
def test_sign_test_exact_values():
    assert A.sign_test([1] * 8)[0] == pytest.approx(2 / 256)
    assert A.sign_test([1] * 7 + [-1])[0] == pytest.approx(18 / 256)
    assert A.sign_test([0, 0, 0])[0] == 1.0
    p, pos, neg, zero = A.sign_test([1, -1, 0, 2])
    assert (pos, neg, zero) == (2, 1, 1) and p == 1.0


def test_decide_never_says_same():
    c = A.compare([({'pair': 1, 'm': 5}, {'pair': 1, 'm': 5})] * 8, 'm', 'a', 'b')
    txt = A.decide(c, margin=1.0)
    assert '가르지 못함' in txt and '같' not in txt
    c2 = A.compare([({'pair': k, 'm': 3}, {'pair': k, 'm': 5}) for k in range(8)], 'm', 'a', 'b')
    assert A.decide(c2).startswith('b > a')


def test_outage_completions_count_between_kill_and_start():
    ma = {'pair': 1, 'faults': [{'action': 'kill', 'scheduled': 60, 'sim': 160},
                                {'action': 'start', 'scheduled': 300, 'sim': 400}],
          'tasks': [{'completed_t': 150}, {'completed_t': 200}, {'completed_t': 450}]}
    mb = {'pair': 1, 'faults': [{'action': 'kill', 'scheduled': 60, 'sim': 161},
                                {'action': 'start', 'scheduled': 300, 'sim': 401}],
          'tasks': [{'completed_t': 170}, {'completed_t': 300}, {'completed_t': 390}]}
    assert A.outage_completions((ma, mb)) == {'a': 1, 'b': 3, 'pair': 1}


def test_divergence_finds_first_split(tmp_path):
    ga = track('amr_01', 100, 130, 0, 0, 30, 0, 1.0)
    gb = track('amr_01', 200, 230, 0, 0, 15, 0, 0.5)
    da = write_run(tmp_path, ga, [], t_base=100, t_end=130, label='a')
    db = write_run(tmp_path, gb, [], t_base=200, t_end=230, label='b')
    ma, mb = A.run_metrics(str(da)), A.run_metrics(str(db))
    dv = A.divergence(ma, mb)
    assert dv['gt_diverge_s'] == pytest.approx(2.1, abs=0.15)


# ---------------------------------------------------------------------- 계획
def test_schedule_alternates_first_arm_and_domains():
    s = P.schedule(['AA', 'C1'])
    assert len(s) == 6 + 8
    assert [x[2] for x in s[:4]] == [0, 1, 0, 1]
    assert s[0][3] != s[1][3] and all(21 <= d <= 29 for x in s for d in x[3])


def test_plan_tasks_match_the_scenario_12_order_and_docks():
    plan = P.run_plan('C2', 3, 1, t_start=300.0)
    docks = [t['pickup_dock'] for t in plan['tasks']][:5]
    assert docks == ['dock_1', 'dock_a', 'dock_2', 'dock_b', 'dock_1']
    assert [t['offset_s'] for t in plan['tasks']] == [0.0] * 5 + [90.0] * 3
    assert plan['faults'] == [[60.0, 'kill', 1], [300.0, 'start', 2]]
    assert plan['mode'] == 'distributed' and plan['t_start'] == 300.0
    assert len({t['task_id'] for t in plan['tasks']}) == 8
    assert not P.check_docks(str(REPO / 'src/amr_behavior/config/behavior.yaml'))


def test_comm_models_are_absolute_and_symmetric():
    m = json.loads(P.comm_yaml('partition_contention'))
    assert m['partitions'][0]['start_s'] == P.T_START + 40.0
    m60 = json.loads(P.comm_yaml('partition_contention', 60.0))
    assert m60['partitions'][0]['start_s'] == 100.0 and m60['outages'][0]['end_s'] == 460.0
    assert m['partitions'][0]['groups'] == [['amr_01', 'amr_02']]
    pairs = {(o['src'], o['dst']) for o in m['outages']}
    assert pairs == {('amr_01', 'amr_02'), ('amr_02', 'amr_01')}
    x = json.loads(P.comm_yaml('stress'))['links'][0]
    assert x['comm_latency_ms'] == [0.0, 1000.0] and x['drop_rate'] == 0.1 and 'burst' in x


def test_every_experiment_has_two_arms_and_known_actions():
    for name, e in P.EXPERIMENTS.items():
        assert len(e['arms']) == 2, name
        for a in e['arms']:
            assert a['mode'] in ('central', 'distributed')
            assert a['layout'] in ('old', 'new')
            for off, action, k in a['faults']:
                assert math.isfinite(off)
                if action in ('agent_kill', 'agent_restart'):
                    assert k in P.ROBOTS
                else:
                    assert action in ('kill', 'pause', 'unpause', 'start') and k >= 1
            assert a['comm'] in (None, 'partition_contention', 'stress')
    assert set(P.ORDER) <= set(P.EXPERIMENTS)


def test_old_dock_boxes_match_fd17e49():
    """고정 지표의 옛 상자 = fd17e49 traffic_zones.yaml (도크 ±1 m)."""
    for zid, poly in A.OLD_DOCK_BOXES.items():
        ys = sorted({p[1] for p in poly})
        assert ys[1] - ys[0] == pytest.approx(2.0), zid
    new = A.load_zones((REPO / 'src/amr_fleet/config/traffic_zones.yaml').read_text())
    for zid in A.OLD_DOCK_BOXES:
        ys = sorted({p[1] for p in new[zid]})
        assert ys[1] - ys[0] == pytest.approx(1.5), zid


def test_registry_has_required_fields(tmp_path):
    d = write_run(tmp_path, [], [])
    m = A.run_metrics(str(d))
    info = {k: 'x' for k in ('run_id', 'stage', 'agent', 'kind', 'sim', 'commit', 'dirty',
                             'config', 'seed', 'started', 'ended', 'raw_dir', 'label', 'load')}
    rec = A.registry(m, info)
    assert isinstance(rec['result'], dict) and rec['result']['n'] == 1
    assert os.path.basename(str(d))


def test_release_inside_commit_failure_and_false_transitions(tmp_path):
    gt = track('amr_01', 100, 140, 3.0, 6.0, 3.0, 6.0, 0)        # x_ab_4 가운데에 서 있다
    ev = [{'t': 100, 'k': 'agent_state', 'r': 'amr_01', 'mode': 0, 'held': []},
          {'t': 105, 'k': 'agent_event', 'r': 'amr_01',
           'ev': {'event': 'zone_release', 'resources': ['zone:x_ab_4']}},
          {'t': 110, 'k': 'agent_event', 'r': 'amr_01',
           'ev': {'event': 'task_commit_request', 'task_id': 'T1'}},
          {'t': 112, 'k': 'agent_event', 'r': 'amr_01',
           'ev': {'event': 'held', 'added': ['task:T1'], 'removed': []}},
          {'t': 115, 'k': 'agent_event', 'r': 'amr_01',
           'ev': {'event': 'task_commit_request', 'task_id': 'T2'}},
          {'t': 120, 'k': 'agent_state', 'r': 'amr_01', 'mode': 1, 'held': []}]
    m = A.run_metrics(str(write_run(tmp_path, gt, ev)))
    assert m['zone_releases'] == 1 and len(m['release_inside']) == 1
    assert [c['ok'] for c in m['commits']] == [True, False] and m['commit_failures'] == 1
    assert m['false_transitions'] == 1
    kill = [{'action': 'kill', 'scheduled': 18.0, 'sim': 118.0}]
    m2 = A.run_metrics(str(write_run(tmp_path / 'f', gt, ev, faults=kill)))
    assert m2['false_transitions'] is None


def test_contention_vacuity_and_holm():
    ev = [{'t': 110, 'k': 'agent_event', 'r': 'amr_01',
           'ev': {'event': 'task_commit_request', 'task_id': 'T1'}},
          {'t': 111, 'k': 'agent_event', 'r': 'amr_02',
           'ev': {'event': 'task_commit_request', 'task_id': 'T1'}},
          {'t': 112, 'k': 'agent_event', 'r': 'amr_02',
           'ev': {'event': 'task_commit_request', 'task_id': 'T2'}}]
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        m = A.run_metrics(str(write_run(Path(tmp), [], ev)))
    assert m['contention_tasks'] == ['T1']
    h = A.holm({'a': 0.01, 'b': 0.04, 'c': 0.03})
    assert h == {'a': pytest.approx(0.03), 'c': pytest.approx(0.06), 'b': pytest.approx(0.06)}


def _pilot_metrics(raw, k, label, mode, rtf, t_ready, valid=True):
    d = raw / f'p5_pilot_p{k}_{label}'
    d.mkdir(parents=True)
    (d / 'metrics.json').write_text(json.dumps({
        'valid': valid, 'rtf': rtf, 't_ready': t_ready, 'completed': 3, 'mode': mode,
        'distance_m': {'amr_01': 10.0}, 'mode_transitions': [{'mode': 'NORMAL'}],
        'stack_has_server': False}))


def test_pilot_rule_concurrent_and_t_start(tmp_path):
    import p5_campaign as C
    _pilot_metrics(tmp_path, 1, 'central', 'central', 0.30, 150.0)
    _pilot_metrics(tmp_path, 1, 'distributed', 'distributed', 0.28, 170.0)
    _pilot_metrics(tmp_path, 2, 'central', 'central', 0.20, 160.0)
    _pilot_metrics(tmp_path, 2, 'distributed', 'distributed', 0.17, 200.0)
    conc, t_start, problems, basis = C.decide(str(tmp_path))
    assert conc is True and problems == []
    assert t_start == 270.0                         # 1.25 × 200 = 250 → 30 의 배수로 올림 270
    (tmp_path / 'p5_pilot_p2_distributed' / 'metrics.json').write_text(json.dumps({
        'valid': True, 'rtf': 0.16, 't_ready': 100.0, 'completed': 3, 'mode': 'distributed',
        'distance_m': {'amr_01': 1.0}, 'mode_transitions': [{}], 'stack_has_server': False}))
    conc2, t2, _, _ = C.decide(str(tmp_path))
    assert conc2 is False and t2 == 240.0          # 0.16 < 0.6 × 0.28, 1.25 × 170 → 240


def test_pilot_t_start_floor_when_ready_is_early(tmp_path):
    import p5_campaign as C
    for k in (1, 2):
        _pilot_metrics(tmp_path, k, 'central', 'central', 0.3, 9.4)
        _pilot_metrics(tmp_path, k, 'distributed', 'distributed', 0.3, 12.0)
    assert C.decide(str(tmp_path))[1] == C.T_FLOOR == 60.0


def test_pilot_sanity_problems_halt(tmp_path):
    import p5_campaign as C
    _pilot_metrics(tmp_path, 1, 'central', 'central', 0.30, 150.0, valid=False)
    conc, t_start, problems, basis = C.decide(str(tmp_path))
    assert any('무효' in p for p in problems) and any('metrics 없음' in p for p in problems)


def test_b2_recordings_doc_condition(tmp_path):
    import p5_campaign as C
    f = tmp_path / 'recordings.md'
    f.write_text('# x\n\n## 0. 먼저 알아야 할 불리한 사실\n\n(녹화 뒤 채운다)\n\n## 1. y\n(녹화 뒤 채운다)\n')
    assert C.b2_doc_done(str(f)) is False
    f.write_text('# x\n\n## 0. 먼저 알아야 할 불리한 사실\n\n- 녹화 8 개 중 …\n\n## 1. y\n(녹화 뒤 채운다)\n')
    assert C.b2_doc_done(str(f)) is True
    assert C.b2_doc_done(str(tmp_path / 'none.md')) is False


def test_stack_process_table_flags_a_second_manager(tmp_path):
    d = write_run(tmp_path, [], [])
    (d / 'ps_stack.txt').write_text('t\n  PID ARGS\n 1 bash\n 2 /opt/x/fleet_adapter_node\n')
    assert A.run_metrics(str(d))['stack_has_server'] is False
    (d / 'ps_stack.txt').write_text('t\n  PID ARGS\n 1 bash\n 2 /opt/x/fleet_manager_node\n')
    m = A.run_metrics(str(d))
    assert m['stack_has_server'] is True and m['stack_processes'] == 2


def test_agent_restart_boot_and_orphan_release(tmp_path):
    ev = [{'t': 100, 'k': 'agent_event', 'r': 'amr_03',
           'ev': {'event': 'boot', 'incarnation': 1, 'kept_zones': [], 'kept_tasks': []}},
          {'t': 178.2, 'k': 'agent_event', 'r': 'amr_03',
           'ev': {'event': 'boot', 'incarnation': 2, 'kept_zones': ['zone:x_bc_4'],
                  'kept_tasks': ['task:p5r.01']}},
          {'t': 181.0, 'k': 'agent_event', 'r': 'amr_03',
           'ev': {'event': 'zone_release', 'resources': ['zone:x_bc_4']}}]
    faults = [{'action': 'agent_kill', 'robot': 'amr_03', 'scheduled': 60.0, 'sim': 160.0},
              {'action': 'agent_restart', 'robot': 'amr_03', 'scheduled': 75.0, 'sim': 175.0}]
    m = A.run_metrics(str(write_run(tmp_path, [], ev, faults=faults)))
    b = [x for x in m['boots'] if x['incarnation'] == 2][0]
    assert b['kept_zones'] == ['zone:x_bc_4'] and b['first_release_after_s'] == 2.8
    r = [x for x in m['fault_response'] if x['action'] == 'agent_restart'][0]
    assert r['boot_after_s'] == 3.2


def test_written_comm_model_follows_the_run_t_start(tmp_path):
    plan, comm = tmp_path / 'plan.json', tmp_path / 'comm.yaml'
    assert P.main(['write', '--exp', 'P', '--pair', '1', '--arm', '0', '--out', str(plan),
                   '--comm-out', str(comm), '--t-start', '60']) == 0
    assert json.loads(plan.read_text())['t_start'] == 60.0
    assert json.loads(comm.read_text())['partitions'][0]['start_s'] == 100.0


def test_recorder_readiness_helpers():
    """기록기 준비 판정 도우미 (rclpy 가 있는 환경에서만 — 모듈이 rclpy 를 import 한다)."""
    pytest.importorskip('rclpy')
    pytest.importorskip('amr_msgs')
    import p5_recorder as R
    assert R.watchdog_ok(-1, -1) is False
    assert R.watchdog_ok(500, 100) is True and R.watchdog_ok(100, 500) is False
    assert R.seed_due(10.0, None, None, 0) is False                  # 나쁜 상태 아님
    assert R.seed_due(10.0, 0.0, None, 0) is False                   # 15 s 안 됨
    assert R.seed_due(16.0, 0.0, None, 0) is True
    assert R.seed_due(30.0, 0.0, 16.0, 1) is False                   # 간격 20 s 안 됨
    assert R.seed_due(36.5, 0.0, 16.0, 1) is True
    assert R.seed_due(500.0, 0.0, 16.0, R.SEED_MAX) is False         # 상한


def test_initial_server_start_is_not_an_injected_fault(tmp_path):
    gt = track('amr_01', 100, 140, 20, 0, 20, 0, 0)
    ev = [{'t': 100, 'k': 'agent_state', 'r': 'amr_01', 'mode': 0, 'held': []},
          {'t': 120, 'k': 'agent_state', 'r': 'amr_01', 'mode': 1, 'held': []}]
    faults = [{'action': 'start', 'server': 1, 'scheduled': None, 'sim': None}]
    m = A.run_metrics(str(write_run(tmp_path, gt, ev, faults=faults)))
    assert m['false_transitions'] == 1
    more = [{'action': 'kill', 'scheduled': 60, 'sim': 160},
            {'action': 'start', 'scheduled': 300, 'sim': 400}]
    ma = {'pair': 1, 'faults': faults + more, 'tasks': [{'completed_t': 200}]}
    assert A.outage_completions((ma, ma))['a'] == 1


def test_gate_stop_before_grant_and_pass_through(tmp_path):
    # amr_01 은 x_ab_4 서쪽 (1.0, 6) 으로 다가가 sim 105 에 선다 → 108 에 허가 → 들어감
    gt = (track('amr_01', 100, 105, -2.0, 6.0, 1.0, 6.0, 0.6)
          + track('amr_01', 105.1, 108, 1.0, 6.0, 1.0, 6.0, 0.0)
          + track('amr_01', 108.1, 112, 1.0, 6.0, 3.0, 6.0, 0.5))
    ev = [{'t': 100, 'k': 'agent_state', 'r': 'amr_01', 'mode': 0, 'held': []},
          {'t': 103, 'k': 'agent_event', 'r': 'amr_01',
           'ev': {'event': 'gate', 'blocking': ['zone:x_ab_4']}},
          {'t': 108, 'k': 'agent_state', 'r': 'amr_01', 'mode': 0, 'held': ['zone:x_ab_4']}]
    m = A.run_metrics(str(write_run(tmp_path, gt, ev)))
    g = m['gate_stops_before_grant']
    assert len(g) == 1 and g[0]['held_after_s'] == 5.0 and g[0]['clear_m'] == pytest.approx(0.64)
    ev2 = ev[:2] + [{'t': 103.5, 'k': 'agent_state', 'r': 'amr_01', 'mode': 0,
                     'held': ['zone:x_ab_4']}]
    m2 = A.run_metrics(str(write_run(tmp_path / 'b', gt, ev2)))
    assert m2['gate_pass_without_stop'] == 1 and m2['gate_stops_before_grant'] == []


def test_invalid_concurrent_pilot_means_sequential_not_halt(tmp_path):
    import p5_campaign as C
    _pilot_metrics(tmp_path, 1, 'central', 'central', 0.24, 22.0)
    _pilot_metrics(tmp_path, 1, 'distributed', 'distributed', 0.17, 25.0)
    _pilot_metrics(tmp_path, 2, 'central', 'central', 0.15, 30.0, valid=False)
    conc, t_start, problems, _ = C.decide(str(tmp_path))      # 짝 2 분산은 metrics 없음
    assert conc is False and problems == [] and t_start == 60.0


# ---------------------------------------------------------------------- 이어받기 줄이는 규칙 (2026-10-13)
RESUME_ORDER = ['C2', 'C3', 'P:1', 'S:1', 'X:1', 'P:2', 'S:2', 'X:2', 'R', 'C1', 'AA:3-6', 'F5L']


def test_order_tokens_keep_pair_bound_arm_and_domain():
    import p5_driver as D
    full = {(x[0], x[1]): x for x in P.schedule(P.ORDER)}
    s = D.schedule_tokens(RESUME_ORDER)
    assert [x[:2] for x in s[:3]] == [('C2', 1), ('C2', 2), ('C2', 3)]
    tech = [('P', 1), ('S', 1), ('X', 1), ('P', 2), ('S', 2), ('X', 2)]
    assert [x[:2] for x in s[16:22]] == tech
    assert all(full[x[:2]] == x for x in s)      # 짝 번호의 먼저 띄울 갈래 · 도메인은 그대로
    keys = {x[:2] for x in s}
    assert ('AA', 1) not in keys and ('AA', 2) not in keys and ('AA', 3) in keys
    assert len(s) == 8 + 8 + 2 * 3 + 2 + 8 + 4 + 8
    for bad in (['P:1', 'P'], ['C2:0-3'], ['C2:3-9'], ['C9']):
        with pytest.raises(ValueError):
            D.schedule_tokens(bad)


def test_new_pair_deadline():
    import datetime as dt
    import p5_driver as D
    t = dt.datetime(2026, 10, 15, 22, 0, tzinfo=dt.timezone(dt.timedelta(hours=9)))
    assert D.past_deadline('', t) is False
    assert D.past_deadline('2026-10-15T22:00:00+09:00', t - dt.timedelta(seconds=1)) is False
    assert D.past_deadline('2026-10-15T22:00:00+09:00', t) is True
    assert D.past_deadline('2026-10-15T13:00:00+00:00', t) is True       # 시간대 환산
    with pytest.raises(ValueError):
        D.past_deadline('2026-10-15T22:00:00', t)                         # 시간대 없음은 거부


def test_driver_starts_no_pair_after_the_deadline(tmp_path):
    import argparse
    import p5_driver as D
    a = argparse.Namespace(raw=str(tmp_path), repo=str(REPO), order='C2,AA:3-6',
                           concurrent=False, t_start=60.0, pair_range='1-99',
                           no_new_pair_after='2000-01-01T00:00:00+09:00')
    (tmp_path / 'pairs').mkdir()
    (tmp_path / 'pairs' / 'C2_p1.done').write_text('{}')
    drv = D.Driver(a)
    assert drv.run_pair('C2', 1, 0, (21, 23)) == 'done'          # 끝난 짝은 마감과 무관하게 건너뜀
    assert drv.run_pair('C2', 2, 1, (25, 27)) == 'deadline'
    assert drv.run() == 0
    assert sorted(p.name for p in tmp_path.iterdir()) == ['driver.log', 'pairs']   # 갈래 없음
    log = (tmp_path / 'driver.log').read_text()
    assert '지남 — C2_p2 부터 시작하지 않고 끝' in log and '무효 3 번' not in log


def _arm_metrics(root, name, exp, pair, label, valid, completed):
    d = root / name
    d.mkdir()
    (d / 'metrics.json').write_text(json.dumps({
        'exp': exp, 'pair': pair, 'label': label, 'valid': valid, 'completed': completed,
        'mode': 'central' if label == 'central' else 'distributed', 'raw': f'/raw/{name}'}))


def test_exp_summary_picks_the_latest_attempt_numerically(tmp_path):
    # 결과 전 결함 수정 (2026-10-13): 이름순이면 …r10 이 …r9 앞에 온다
    for lab in ('central', 'distributed'):
        _arm_metrics(tmp_path, f'p5_C1_p1_{lab}', 'C1', 1, lab, False, None)
        for n in range(2, 11):
            _arm_metrics(tmp_path, f'p5_C1_p1_{lab}r{n}', 'C1', 1, lab, n == 10,
                         5 if lab == 'central' else 4)
    res = A.exp_summary('C1', str(tmp_path))
    assert res['n_pairs_valid'] == 1
    assert res['compare']['completed']['diffs'] == [-1]


def test_exp_summary_prefers_the_done_tags(tmp_path):
    for lab, c in (('central', 5), ('distributed', 3)):
        _arm_metrics(tmp_path, f'p5_C1_p1_{lab}r3', 'C1', 1, lab, True, c)
        _arm_metrics(tmp_path, f'p5_C1_p1_{lab}r4', 'C1', 1, lab, False, None)
    (tmp_path / 'pairs').mkdir()
    (tmp_path / 'pairs' / 'C1_p1.done').write_text(json.dumps(
        {'tags': {'0': 'p5_C1_p1_centralr3', '1': 'p5_C1_p1_distributedr3'}}))
    res = A.exp_summary('C1', str(tmp_path))
    assert res['n_pairs_valid'] == 1 and res['compare']['completed']['diffs'] == [-2]
    assert res['tags'] == {'1': ['p5_C1_p1_centralr3', 'p5_C1_p1_distributedr3']}


# ------------------------------------------------------- 갈래별 기동 재시도 (2026-10-13 23:50)
def _fake_driver(tmp_path, script):
    """script: 태그 → (posted, valid, load_abort). 없으면 (5, True, False)."""
    import argparse
    import p5_driver as D
    a = argparse.Namespace(raw=str(tmp_path), repo=str(REPO), order='C2', concurrent=False,
                           t_start=60.0, pair_range='1-99', no_new_pair_after='',
                           arm_retry_max=6)
    drv = D.Driver(a)
    launched = []

    def run_arm(exp, k, i, tag, dom, res):
        launched.append(tag)
        posted, _, load = script.get(tag, (5, True, False))
        d = tmp_path / tag
        d.mkdir()
        (d / 'clock.json').write_text(json.dumps({'posted': posted, 'state': 'END'}))
        (d / 'faults.jsonl').write_text(json.dumps({'scheduled': None, 'action': 'start'}) + '\n')
        if load:
            res['load_abort'] = True
        res['rc'] = 0

    def analyze(exp, k, tag, res, label, mode):
        return script.get(tag, (5, True, False))[1] if (tmp_path / tag / 'clock.json').exists() \
            else False

    drv.run_arm, drv.analyze, drv.wait_quiet = run_arm, analyze, lambda key: True
    return drv, launched


def test_startup_failure_retries_only_that_arm(tmp_path):
    script = {'p5_C2_p1_central': (0, False, False), 'p5_C2_p1_centralr2': (0, False, False)}
    drv, launched = _fake_driver(tmp_path, script)
    assert drv.run_pair('C2', 1, 0, (21, 23)) == 'ok'
    assert launched == ['p5_C2_p1_central', 'p5_C2_p1_centralr2', 'p5_C2_p1_centralr3',
                        'p5_C2_p1_distributed']
    done = json.loads((tmp_path / 'pairs' / 'C2_p1.done').read_text())
    assert set(done['tags'].values()) == {'p5_C2_p1_centralr3', 'p5_C2_p1_distributed'}


def test_failure_after_tasks_invalidates_pair_and_skips_second_arm(tmp_path):
    drv, launched = _fake_driver(tmp_path, {'p5_C2_p2_distributed': (3, False, False)})
    assert drv.run_pair('C2', 2, 1, (25, 27)) == 'invalid'      # 짝 2 는 분산(1 번 갈래)이 먼저
    assert launched == ['p5_C2_p2_distributed']
    assert (tmp_path / 'p5_C2_p2_central' / 'ABORTED.txt').exists()
    assert not (tmp_path / 'pairs' / 'C2_p2.done').exists()


def test_scheduled_fault_means_not_a_startup_failure(tmp_path):
    drv, _ = _fake_driver(tmp_path, {})
    d = tmp_path / 'p5_C3_p1_dist_dead'
    d.mkdir()
    (d / 'clock.json').write_text(json.dumps({'posted': 0}))
    (d / 'faults.jsonl').write_text(json.dumps({'scheduled': -30.0, 'action': 'kill'}) + '\n')
    assert drv.pre_task('p5_C3_p1_dist_dead') is False
    (d / 'faults.jsonl').write_text('')
    assert drv.pre_task('p5_C3_p1_dist_dead') is True
    (d / 'events.jsonl').write_text(json.dumps({'k': 'post', 't': 60.0}) + '\n')
    assert drv.pre_task('p5_C3_p1_dist_dead') is False


def test_repeated_startup_failure_halts_and_load_abort_retries_the_arm(tmp_path):
    script = {f'p5_C2_p1_distributed{s}': (0, False, False)
              for s in ['', 'r2', 'r3', 'r4', 'r5', 'r6']}
    drv, launched = _fake_driver(tmp_path, script)
    assert drv.run_pair('C2', 1, 0, (21, 23)) == 'halt'
    assert launched.count('p5_C2_p1_central') == 1 and len(launched) == 7
    (tmp_path / 'x').mkdir()
    drv2, launched2 = _fake_driver(tmp_path / 'x', {'p5_C2_p1_distributed': (5, False, True)})
    waits = []
    drv2.wait_quiet = lambda key: waits.append(key) or True
    # 부하 중단은 그 갈래만 시작 조건을 기다린 뒤 다시 (2026-10-14 04:10) — 첫 갈래(중앙) 결과는 둔다
    assert drv2.run_pair('C2', 1, 0, (21, 23)) == 'ok'
    assert launched2 == ['p5_C2_p1_central', 'p5_C2_p1_distributed', 'p5_C2_p1_distributedr2']
    assert len(waits) == 2                                  # 짝 시작 + 부하 중단 뒤
    done = json.loads((tmp_path / 'x' / 'pairs' / 'C2_p1.done').read_text())
    assert set(done['tags'].values()) == {'p5_C2_p1_central', 'p5_C2_p1_distributedr2'}


def test_reuse_arm_takes_only_a_valid_arm_of_the_same_pair(tmp_path):
    drv, launched = _fake_driver(tmp_path, {})
    for tag, pair, valid in (('p5_C2_p3_centralr2', 3, True), ('p5_C2_p3_centralr3', 3, False),
                             ('p5_C2_p1_central', 1, True)):
        (tmp_path / tag).mkdir()
        (tmp_path / tag / 'metrics.json').write_text(json.dumps(
            {'exp': 'C2', 'pair': pair, 'label': 'central', 'valid': valid}))
    drv.a.reuse_arm = ['p5_C2_p1_central', 'p5_C2_p3_centralr3', 'p5_C2_p3_centralr2']
    assert drv.reusable('C2', 3, 'central') == 'p5_C2_p3_centralr2'
    assert drv.reusable('C2', 3, 'distributed') == ''
    assert drv.run_pair('C2', 3, 0, (21, 23)) == 'ok'
    assert launched == ['p5_C2_p3_distributed']
    done = json.loads((tmp_path / 'pairs' / 'C2_p3.done').read_text())
    assert set(done['tags'].values()) == {'p5_C2_p3_centralr2', 'p5_C2_p3_distributed'}
