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
