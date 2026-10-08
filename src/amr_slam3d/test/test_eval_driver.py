"""
P 단계 재생 구동기·분석 스크립트 계약 시험.

행·이미지·bag 종류가 rows.py 와 같고, 재생 배속이 녹화 RTF 에 맞고, 사전 등록 작업 목록의 크기·우선순위·이름이
규칙대로이며, 분석의 창·실패 규칙이 문서와 같다.
"""

import importlib.util
import json
import math
from pathlib import Path

from amr_slam3d import rows
import pytest

PKG = Path(__file__).resolve().parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, PKG / 'scripts' / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


drv = _load('eval_driver')


def test_row_images_and_bag_kinds_match_rows_py():
    for name, (img, kind) in drv.ROW_IMAGE.items():
        r = rows.row(name)
        assert r.image == img, name
        assert kind == {'orig': 'bag', 'madg': 'bag_madg'}[r.bag], name


def test_rate_matches_recorded_rtf():
    # 유효 sim 속도 = 녹화 RTF × 배속 = 0.25 (recordings.md §4)
    for bag, (rtf, _, _) in drv.BAGS.items():
        assert drv.rate_for(bag) * rtf == pytest.approx(0.25, abs=0.0006), bag
    assert drv.rate_for('s3_aisle_a1') == pytest.approx(0.507)
    assert drv.rate_for('map_long_a1') == pytest.approx(1.008)


def test_bags_table_matches_recordings_md():
    assert set(drv.BAGS) == {'s3_aisle_a1', 's1_seed1_a1', 's1_seed3_a2', 's1_seed2x_a1',
                             's6_open_a3', 's6_queue_a1', 's6_wall_a1', 'map_long_a1'}
    assert [b for b, v in drv.BAGS.items() if not v[2]] == ['s1_seed2x_a1']


def test_plan_e2_size_priority_and_names():
    jobs = drv.plan_e2()
    assert len(jobs) == 6 * 8 * 3
    assert len({j['id'] for j in jobs}) == len(jobs)
    first = sorted(jobs, key=lambda j: (j['priority'], j['id']))[:6]
    want = {('map_long_a1', 'slam_toolbox'), ('map_long_a1', 'rtabmap3d_noimu')}
    assert {(j['bag'], j['row']) for j in first} == want
    for j in jobs:
        assert j['domain'] == 45
        assert drv.cname(j).startswith('amr-ai-b2-eval-e2-')
        assert all(c.isalnum() or c in '_.-' for c in drv.cname(j))
        assert j['image'] and j['kind'] in ('bag', 'bag_madg')
    oracle = [j for j in jobs if rows.row(j['row']).oracle]
    assert oracle and min(j['priority'] for j in oracle) > max(
        j['priority'] for j in jobs if j['row'] in drv.E2_ROWS_MAIN)


def test_claim_is_exclusive_and_done_skips(tmp_path):
    j = drv.job('e2', 's3_aisle_a1', 'slam_toolbox', 0, 1, 45)
    assert drv.try_claim(str(tmp_path), j, set())
    assert not drv.try_claim(str(tmp_path), j, set())       # 120 s 안 claim 은 낡은 것이 아니다
    out = Path(drv.out_dir(str(tmp_path), j))
    out.mkdir(parents=True, exist_ok=True)
    (out / 'DONE').write_text('{}')
    assert drv.is_done(str(tmp_path), j)
    assert not drv.try_claim(str(tmp_path), j, set())


def test_finish_job_retries_only_startup_failures(tmp_path):
    root = str(tmp_path)
    j = drv.job('e2', 's3_aisle_a1', 'slam_toolbox', 0, 1, 45)
    for rc, moved in ((4, True), (0, False)):
        o = Path(drv.out_dir(root, j))
        o.mkdir(parents=True, exist_ok=True)
        Path(str(o) + '.claim').mkdir(exist_ok=True)
        drv.finish_job(root, j, rc)
        assert drv.is_done(root, j) is (not moved)
        assert not Path(str(o) + '.claim').exists()
    assert drv.attempts(root, j) == 1


peval = _load('peval')


def test_peval_window_uses_covering_runs_only():
    gt_t = [0.0, 100.0]
    w = peval.window_for(gt_t, [[1.0, 99.0], [2.0, 98.0], [50.0, 60.0], None])
    assert w == (2.0, 98.0)
    assert peval.window_for(gt_t, [None]) == (0.0, 100.0)


def test_peval_fail_rules():
    ok = {'row': 'rtabmap3d_noimu', 'family': 'rtabmap', 'rc': 0, 'missing': 0, 'fatal': 0,
          'coverage': 0.99}
    assert peval.fail_of(ok) == []
    assert peval.fail_of(dict(ok, missing=4)) == ['missing=4']
    assert peval.fail_of(dict(ok, missing=None)) == ['missing=미측정']
    assert peval.fail_of(dict(ok, rc=4)) == ['rc=4']
    assert peval.fail_of(dict(ok, coverage=0.5)) == ['coverage=0.50']
    st = {'row': 'slam_toolbox', 'family': 'slam_toolbox', 'rc': 0, 'missing': None,
          'fatal': 0, 'msgfilter_drop': 1, 'coverage': 1.0}
    assert peval.fail_of(st) == ['msgfilter=1']


def test_peval_families_and_feasibility(tmp_path):
    recs = []
    for i, bag in enumerate(peval.BAGS_ALL):
        for k in range(3):
            for row, v in (('rtabmap3d_noimu', 1.0), ('rtabmap2d_noimu', 2.0),
                           ('slam_toolbox', 1.5), ('liosam_planar', 1.4)):
                recs.append({'bag': bag, 'row': row, 'replay': k, 'fail': [],
                             'ate_rmse_cm': v + 0.01 * i, 'map_adnn_surf_cm': v})
    t8 = peval.family_tests(recs, peval.BAGS_ALL, 'ALL8')
    c1 = t8['families']['T'][0]
    assert c1['n_bags'] == 8 and c1['n_neg'] == 8
    assert c1['wilcoxon']['p'] == pytest.approx(2 / 256)
    assert c1['p_holm'] == pytest.approx(3 * 2 / 256)
    assert c1['rejectable'] and c1['verdict'] == 'A<B'
    t2 = peval.family_tests(recs, peval.S1_PRE, 'S1_2')
    assert not t2['families']['T'][0]['rejectable']
    assert t2['families']['T'][0]['verdict'] == 'undecided'
    # 사전 등록 bag 7 개는 가족 T(3)·M(2) 둘 다 기각 가능한 설계 (2/128 × 3 < 0.05)
    t7 = peval.family_tests(recs, peval.BAGS_PRE, 'PRE7')
    assert all(r['rejectable'] for f in t7['families'].values() for r in f)
    path = tmp_path / 'r.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in recs))
    out = tmp_path / 'out'
    assert peval.cmd_compare(type('N', (), {'replays': [str(path)], 'out': str(out)})) == 0
    res = json.loads((out / 'compare.json').read_text())
    assert res['rows']['rtabmap3d_noimu']['fail'] == 0
    assert math.isfinite(res['rows']['rtabmap2d_noimu']['metrics']['ate_rmse_cm']['between_sd'])


def test_snapshot_head_is_used_when_present(tmp_path):
    (tmp_path / 'SNAPSHOT_HEAD').write_text('abc123\n')
    head, st = drv.wt_head(str(tmp_path))
    assert head == 'abc123' and st.startswith('snapshot')


def test_job_wt_override_reaches_docker_mount(tmp_path, monkeypatch):
    calls = {}

    class P:
        pid = 1

    def fake_popen(cmd, stdout=None, stderr=None):
        calls['cmd'] = cmd
        return P()
    monkeypatch.setattr(drv.subprocess, 'Popen', fake_popen)
    snap = tmp_path / 'snap'
    snap.mkdir()
    (snap / 'SNAPSHOT_HEAD').write_text('deadbeef\n')
    j = dict(drv.job('e3', 's3_aisle_a1', 'slam_toolbox', 0, 1, 46), wt=str(snap))
    p, fh = drv.start_job(str(tmp_path / 'root'), '/other', j)
    fh.close()
    assert f'{snap}:/ros2_ws:ro' in calls['cmd']
    assert (tmp_path / 'root' / j['id'] / 'HEAD.txt').read_text().strip() == 'deadbeef'


def test_plan_e3_e4_shapes_and_dependencies():
    e3 = drv.plan_e3()
    assert len(e3) == 1 + 4 * 7 * 3
    assert len({j['id'] for j in e3}) == len(e3)
    gtdb = [j for j in e3 if j['row'] == 'rtabmap3d_gtdb']
    assert len(gtdb) == 1 and gtdb[0]['bag'] == 'map_long_a1' and not gtdb[0]['requires']
    for j in e3:
        assert j['wt'] == drv.WS_E3 and j['domain'] == 46 and j['image'] == 'amr-ai/slam:0.1'
        assert rows.row(j['row']).image == j['image']
        if j['row'] == 'rtabmap3d_gtdb':
            continue
        assert j['bag'] != 'map_long_a1'
        assert j['requires'] and j['mounts'] == [f'{drv.MAPS}:/data/map:ro']
        r = rows.row(j['row'])
        if r.loc_map == 'db':
            assert j['env']['MAP_SPAWN'] == drv.MAP_LONG_SPAWN and 'LOC_DB' in j['env']
        else:
            assert 'LOC_MAP' in j['env']
    e4 = drv.plan_e4()
    assert len(e4) == 9 * 4 * 2
    assert {j['row'] for j in e4} == set(drv.ABL_ROWS)
    assert all(rows.row(r) for r in drv.ABL_ROWS)
    assert min(j['priority'] for j in e4) > max(j['priority'] for j in e3)
    e2 = drv.plan_e2()
    assert min(j['priority'] for j in e3 if j['row'] != 'rtabmap3d_gtdb') > max(
        j['priority'] for j in e2 if j['row'] in drv.E2_ROWS_MAIN)


def test_select_rule_preregistered():
    assert peval.select_rule([(0, 1.0, True), (1, 3.0, True), (2, 2.0, True)]) == (2, 'median_ate')
    assert peval.select_rule([(1, 9.0, True), (2, 1.0, True)]) == (1, 'lowest_index_clean')
    assert peval.select_rule([(2, 5.0, True)]) == (2, 'only_clean')
    assert peval.select_rule([(1, 1.0, False), (0, 2.0, False)]) == (1, 'failed_replay_map')
    assert peval.select_rule([]) == (None, 'none')


def test_loc_fail_rules_and_e2_rules_unchanged():
    base = {'row': 'rtabmap3d_loc_gt', 'rc': 0, 'coverage': 1.0, 'missing': 0, 'null_odom': 1,
            'fatal': 0, 'ate_max_cm': 3.0, 'yaw_max_deg': 0.5}
    assert peval.loc_fail_of(base) == []
    assert peval.loc_fail_of(dict(base, null_odom=3)) == ['lost=2']
    assert peval.loc_fail_of(dict(base, ate_max_cm=60.0)) == ['diverged']
    assert peval.loc_fail_of(dict(base, yaw_max_deg=11.0)) == ['diverged']
    am = {'row': 'amcl_gt', 'rc': 0, 'coverage': 1.0, 'missing': None, 'msgfilter_drop': 2,
          'fatal': 0, 'ate_max_cm': 3.0, 'yaw_max_deg': 0.5}
    assert peval.loc_fail_of(am) == ['msgfilter=2']
    assert peval.is_loc_row('amcl_slam') and peval.is_loc_row('rtabmap3d_loc_slam')
    assert not peval.is_loc_row('rtabmap3d_noimu') and not peval.is_loc_row('slam_toolbox')
    # E2 정의는 그대로 (발산은 E2 실패 사유가 아니다)
    e2 = {'row': 'rtabmap3d_noimu', 'family': 'rtabmap', 'rc': 0, 'missing': 0, 'fatal': 0,
          'coverage': 0.99, 'ate_max_cm': 80.0}
    assert peval.fail_of(e2) == []


def test_compare_loc_and_abl_run_on_synthetic(tmp_path):
    recs = []
    for i, bag in enumerate(peval.LOC_BAGS_ALL):
        for k in range(3):
            for row, v in (('amcl_gt', 2.0), ('rtabmap3d_loc_gt', 1.0), ('amcl_slam', 2.5),
                           ('rtabmap3d_loc_slam', 1.5)):
                recs.append({'bag': bag, 'row': row, 'replay': k, 'fail': [],
                             'ate_rmse_cm': v + 0.01 * i, 'seg_still_rmse_cm': v,
                             'seg_still_max_cm': v * 2, 'seg_still_p95_cm': v,
                             'aisle_axis_rmse_cm': v})
    path = tmp_path / 'loc.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in recs))
    ns = type('N', (), {'replays': [str(path)], 'out': str(tmp_path / 'o')})
    assert peval.cmd_compare_loc(ns) == 0
    res = json.loads((tmp_path / 'o' / 'compare_loc.json').read_text())
    l1 = res['tests']['ALL7']['families']['L1'][0]
    assert l1['n_neg'] == 7 and l1['verdict'] == 'A<B' and l1['rejectable']
    l1p = res['tests']['PRE6']['families']['L1'][0]
    assert l1p['n_bags'] == 6 and l1p['rejectable']           # 2/64 < 0.05 (가족마다 하나)
    assert res['segments']['amcl_gt']['still']['rmse_pass'] == 21
    assert res['segments']['amcl_gt']['still']['max_pass'] == 0   # 4 cm > 3 cm
    abl = []
    for bag in peval.ABL_BAGS:
        for k in range(3):
            abl.append({'bag': bag, 'row': 'rtabmap3d_noimu', 'replay': k, 'fail': [],
                        'ate_rmse_cm': 1.0, 'lat_icp': {'p50_ms': 40.0}})
        for k in range(2):
            abl.append({'bag': bag, 'row': 'rtabmap3d_abl_vox02', 'replay': k, 'fail': [],
                        'ate_rmse_cm': 1.5, 'lat_icp': {'p50_ms': 20.0}})
    p2 = tmp_path / 'abl.jsonl'
    p2.write_text('\n'.join(json.dumps(r) for r in abl))
    ns = type('N', (), {'replays': [str(p2)], 'out': str(tmp_path / 'a')})
    assert peval.cmd_compare_abl(ns) == 0
    res = json.loads((tmp_path / 'a' / 'compare_abl.json').read_text())
    m = res['rows']['rtabmap3d_abl_vox02']['metrics']
    assert m['ate_rmse_cm']['n_differs'] == 4
    assert m['ate_rmse_cm']['mean_diff'] == pytest.approx(0.5)
    assert m['lat_icp_p50_ms']['mean_diff'] == pytest.approx(-20.0)
