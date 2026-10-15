"""
재녹화(b2-rerec) 도구 시험.

재생 작업 목록(rerec_plan.py)이 원래 재생과 같은 조건인지, 분석 보기 링크, 구동기의 에이전트·bag 디렉터리 바꾸기,
녹화 기록(rec_report.py)의 새 필드, 위치추정 인자(loc_stack_args.py)가 bringup 과 같은지.
"""

import importlib.util
import json
import os
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, PKG / 'scripts' / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


plan = _load('rerec_plan')
TAGS = {'rr_s6_wall': 'rr_s6_wall_a1', 'rr_s1_seed3': 'rr_s1_seed3_a2',
        'rr_s1_seed2': 'rr_s1_seed2_a1'}
RTFS = {'rr_s6_wall': 0.248, 'rr_s1_seed3': 0.231, 'rr_s1_seed2': 0.203}


def test_plan_sizes_and_unique_ids():
    p = plan.build_plans('/R', TAGS, RTFS)
    assert len(p['e2']) == 3 * 6 * 3
    assert len(p['e3']) == 3 * 4 * 3
    assert len(p['e4']) == 2 * 9 * 2
    ids = [j['id'] for st in p.values() for j in st]
    assert len(ids) == len(set(ids))


def test_plan_same_conditions_as_original_replays():
    drv = plan._drv()
    p = plan.build_plans('/R', TAGS, RTFS)
    for j in p['e2']:
        assert j['wt'] == plan.WS_E2 and j['domain'] == 45
        assert j['bagdir'] == f"/R/rec/{j['bag']}/{j['kind']}"
        assert j['kind'] == drv.ROW_IMAGE[j['row']][1]
        main = j['row'] in drv.E2_ROWS_MAIN
        ref = j['row'] in drv.E2_ROWS_REF
        assert j['priority'] == (20 if main else 30 if ref else 55) + j['replay']
    for j in p['e3']:
        assert j['wt'] == plan.WS_E3 and j['domain'] == 46 and j['priority'] == 45 + j['replay']
        assert j['bagdir'].endswith('/bag') and '/R/go/e3' in j['requires']
        assert j['mounts'] == [f'{plan.MAPS}:/data/map:ro']
    for j in p['e4']:
        assert j['wt'] == plan.WS_E3 and j['domain'] == 48 and j['priority'] == 60 + j['replay']
        assert j['bag'] in (TAGS['rr_s1_seed3'], TAGS['rr_s1_seed2'])
        assert j['row'] in drv.ABL_ROWS and '/R/go/e4' in j['requires']


def test_plan_e3_env_matches_original_plan():
    """E3(b) 행의 지도·초기 자세 환경은 b2-eval plan_e3 와 같다 (bag 만 다르다)."""
    drv = plan._drv()
    orig = {j['row']: (j['env'], [r for r in j['requires'] if not r.endswith('/go/e3')])
            for j in drv.plan_e3() if j['row'] != 'rtabmap3d_gtdb'}
    for j in plan.build_plans('/R', TAGS, RTFS)['e3']:
        assert j['env'] == orig[j['row']][0]
        assert [r for r in j['requires'] if not r.endswith('/go/e3')] == orig[j['row']][1]


def test_rate_rule_matches_original():
    drv = plan._drv()
    for bag, (rtf, _, _) in drv.BAGS.items():
        assert plan.rate_of(rtf) == drv.rate_for(bag), bag
    p = plan.build_plans('/R', TAGS, RTFS)
    for j in p['e2'] + p['e3'] + p['e4']:
        i = next(k for k, v in TAGS.items() if v == j['bag'])
        assert j['rate'] * RTFS[i] == pytest.approx(0.25, abs=0.0006)


def test_view_links_map_slots_to_new_bags_and_keep_old_ones():
    links = plan.view_links('/R', TAGS, old_eval='/E', old_rec='/B')
    assert links['e2/s1_seed3_a2'] == '/R/e2/rr_s1_seed3_a2'
    assert links['e2/s1_seed2x_a1'] == '/R/e2/rr_s1_seed2_a1'
    assert links['e2/s6_wall_a1'] == '/R/e2/rr_s6_wall_a1'
    assert links['rec/s6_wall_a1'] == '/R/rec/rr_s6_wall_a1'
    assert links['cache/s1_seed2x_a1'] == '/R/cache/s1_seed2x_a1'
    assert 'e4/s6_wall_a1' not in links                  # E4 는 S3·S1 bag 만
    assert links['e4/s1_seed3_a2'] == '/R/e4/rr_s1_seed3_a2'
    for b in plan.KEEP:
        assert links[f'rec/{b}'] == f'/B/{b}' and links[f'cache/{b}'] == f'/E/cache/{b}'
    slots = set(plan.SLOTS.values())
    assert slots | set(plan.KEEP) == {'s3_aisle_a1', 's1_seed1_a1', 's1_seed3_a2',
                                      's1_seed2x_a1', 's6_open_a3', 's6_queue_a1',
                                      's6_wall_a1', 'map_long_a1'}


def test_driver_agent_env_and_default(monkeypatch):
    monkeypatch.delenv('B2_AGENT', raising=False)
    assert _load('eval_driver').AGENT == 'b2-eval'
    monkeypatch.setenv('B2_AGENT', 'b2-rerec')
    d = _load('eval_driver')
    assert d.AGENT == 'b2-rerec' and d.PREFIX == 'amr-ai-b2-rerec-'


def test_rec_report_summary_carries_new_checks(tmp_path):
    rep = _load('rec_report')
    gate = {'pass': False, 'reasons': ['기동 바이어스: x'], 'topics': {}, 'protocol': {},
            'bias': {'ok': False, 'kind': 'fallback', 'samples': None, 'gyro_bias': None},
            'odom_drift': {'ok': False, 'rate_rad_s': 0.01, 'yaw_err_max_deg': 48.0}}
    (tmp_path / 'gate.json').write_text(json.dumps(gate), encoding='utf-8')
    (tmp_path / 'settle.json').write_text(json.dumps({'ok': True, 'wall_s': 14.0,
                                                      'last': {'sim_now': 3.1}}),
                                          encoding='utf-8')
    (tmp_path / 'env.txt').write_text('loc_after_settle: 1\nbias_pre_record: ok\n',
                                      encoding='utf-8')
    s = rep.summary(str(tmp_path))
    assert s['bias_ok'] is False and s['bias_kind'] == 'fallback'
    assert s['drift_ok'] is False and s['ekf_yaw_drift_rad_s'] == 0.01
    assert s['settle_ok'] is True and s['settle_sim_s'] == 3.1
    assert s['loc_after_settle'] == '1' and s['bias_pre_record'] == 'ok'


def test_rec_report_old_attempt_without_new_fields(tmp_path):
    rep = _load('rec_report')
    gate = {'pass': True, 'reasons': [], 'topics': {}, 'protocol': {}}
    (tmp_path / 'gate.json').write_text(json.dumps(gate), encoding='utf-8')
    s = rep.summary(str(tmp_path))
    assert 'bias_ok' not in s and 'drift_ok' not in s and 'settle_ok' not in s


def test_rerec_plan_file_matches_original_scenarios():
    """재녹화 계획의 경로·월드 변형·스폰·씨드는 원래 사전 등록 계획(rec_plan.txt)의 같은 시나리오와 같다."""
    def rows(name):
        out = {}
        for line in (PKG / 'config' / name).read_text(encoding='utf-8').splitlines():
            if line.strip() and not line.startswith('#'):
                p = line.split()
                out[p[0]] = tuple(p[1:5])
        return out
    orig, rr = rows('rec_plan.txt'), rows('rerec_plan.txt')
    assert set(rr) == set(plan.SLOTS)
    for i in rr:
        assert rr[i] == orig[i[len('rr_'):]], i


def test_loc_stack_args_match_bringup():
    pytest.importorskip('amr_bringup.launch_utils')
    las = _load('loc_stack_args')
    spawn = PKG / 'config' / 'spawn' / 's6_inside.yaml'
    a = las.loc_arguments(str(spawn), 'odom')
    assert a['mode'] == 'odom' and a['robot_name'] == 'amr_01' and a['namespace'] == 'amr_01'
    assert a['frame_prefix'] == 'amr_01/' and a['use_sim_time'] == 'true'
    assert float(a['initial_x']) == pytest.approx(0.0) and float(a['initial_y']) == \
        pytest.approx(-8.6)
    assert 'map' not in a
    assert os.path.isfile(spawn)
