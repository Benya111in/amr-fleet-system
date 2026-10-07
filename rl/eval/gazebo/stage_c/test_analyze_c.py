"""단계 C 집계·실행기 상태 도우미 시험 (pytest, numpy 만)."""

import csv
import gzip
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analyze_c as A  # noqa: E402
import c_state as S  # noqa: E402
import overshoot_c as OV  # noqa: E402


def test_mcnemar_exact_known_values():
    """이항 꼬리 직접 계산과 같다."""
    assert A.mcnemar_exact(0, 0) == 1.0
    assert math.isclose(A.mcnemar_exact(0, 6), 2 * 0.5 ** 6)
    assert math.isclose(A.mcnemar_exact(1, 9), 2 * (1 + 10) / 2 ** 10)
    assert A.mcnemar_exact(5, 5) == 1.0
    assert A.mcnemar_exact(9, 1) == A.mcnemar_exact(1, 9)


def test_boot_ci_constant_and_coverage():
    """모두 같은 차이면 구간이 한 점, 섞이면 점추정을 감싼다."""
    e = A.boot_ci([1, 1, 1, 1])
    assert e['est'] == 1.0 and e['lo'] == 1.0 and e['hi'] == 1.0
    d = [1] * 10 + [0] * 90
    e = A.boot_ci(d)
    assert e['lo'] < 0.1 < e['hi'] and e['n'] == 100
    c = A.boot_ci(d, clusters=list(range(100)))
    assert c['lo'] < 0.1 < c['hi']


def test_goal_clusters_single_linkage():
    """1.0 m 사슬로 이어진 목표는 한 군집."""
    scen = {'a': {'goal': [0, 0]}, 'b': {'goal': [0.9, 0]}, 'c': {'goal': [1.8, 0]},
            'd': {'goal': [5, 5]}}
    cl = A.goal_clusters(['a', 'b', 'c', 'd'], scen)
    assert cl[0] == cl[1] == cl[2] and cl[3] != cl[0]


def test_excl_order():
    """E1 > E2 > E3 > E4."""
    base = {'setup_ok': 'True', 'outcome_contract': 'success', 'loc_err_start_m': '0.02',
            'lost_at_start': 'False', 'cancel_reason': '', 'rtf': '0.8',
            'loc_collapse': 'False', 'lost_events': '0'}
    assert A.excl(base) is None
    assert A.excl({**base, 'setup_ok': 'False'}) == 'E1'
    assert A.excl({**base, 'loc_err_start_m': '0.2', 'rtf': '0.1'}) == 'E2'
    assert A.excl({**base, 'rtf': '0.4', 'loc_collapse': 'True'}) == 'E3'
    assert A.excl({**base, 'lost_events': '1'}) == 'E4'


def test_success_modes():
    """전체에서는 E1–E3 이 실패, E4 는 지면 진실 그대로."""
    r = {'setup_ok': 'True', 'outcome_contract': 'success', 'loc_err_start_m': '0.02',
         'lost_at_start': 'False', 'cancel_reason': 'watchdog', 'rtf': '0.8',
         'loc_collapse': 'False', 'lost_events': '0', 'strict_gt': 'True'}
    assert A.success(r, 'strict_gt', 'valid') == 1
    assert A.success(r, 'strict_gt', 'all') == 0
    r4 = {**r, 'cancel_reason': '', 'loc_collapse': 'True'}
    assert A.success(r4, 'strict_gt', 'all') == 1


def _write(path, idxs):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, ['exec_index'])
        w.writeheader()
        for i in idxs:
            w.writerow({'exec_index': i})


def test_status_resume_and_done(tmp_path):
    """끊긴 묶음은 첫 빠진 index 부터, 다 차면 DONE 표지."""
    b = str(tmp_path / 'dwa_b1')
    _write(os.path.join(b, 'a1', 'gzbench.csv'), range(50, 63))
    assert S.status(b, 50, 50) == 'TODO 63 37 0'
    _write(os.path.join(b, 'a2', 'gzbench.csv'), range(63, 100))
    assert S.status(b, 50, 50) == 'DONE 50'
    assert os.path.exists(os.path.join(b, 'DONE'))


def test_status_no_progress_failed(tmp_path):
    """진전 없는 시도 3 번이면 FAILED."""
    b = str(tmp_path / 'rl_b0')
    for k in (1, 2, 3):
        os.makedirs(os.path.join(b, f'a{k}'))
        assert S.attempt_end(os.path.join(b, f'a{k}')) == '0'
    assert S.status(b, 0, 50).startswith('FAILED')


def test_lists_one_lane_interleaves(tmp_path):
    """레인 하나면 35·36 목록을 pos 순으로 번갈아."""
    here = os.path.dirname(os.path.abspath(__file__))
    S.lists(os.path.join(here, 'schedule_c.tsv'), '35', str(tmp_path))
    lines = open(tmp_path / 'lane35.list').read().split('\n')
    assert [x.split()[0] for x in lines[:4]] == ['dwa_b0', 'dwa_b1', 'rl_b0', 'rl_b1']
    assert len([x for x in lines if x]) == 24


def test_schedule_balance():
    """각 팔이 각 레인에서 4 묶음, 묶음 k 의 DWA·RL 은 같은 레인."""
    here = os.path.dirname(os.path.abspath(__file__))
    rows = [ln.split() for ln in open(os.path.join(here, 'schedule_c.tsv'))
            if ln.strip() and not ln.startswith('#')]
    for lane in ('35', '36'):
        for arm in ('dwa', 'rl', 'rlho'):
            assert sum(1 for r in rows if r[0] == lane and r[2].split('_b')[0] == arm) == 4
    lane_of = {r[2]: r[0] for r in rows}
    for k in range(8):
        assert lane_of[f'dwa_b{k}'] == lane_of[f'rl_b{k}']
    starts = sorted(int(r[4]) for r in rows if r[2].startswith('dwa_'))
    assert starts == list(range(0, 400, 50))
    assert np.all([int(r[5]) == 50 for r in rows])


def test_overshoot_classify_and_nav2_end(tmp_path):
    """사후 지나침 진단: 분류 순서(정체 먼저 → 들어간 적 없음 → 미끄러져 나감)와 Nav2 끝 표본."""
    def row(sys_ok, strict, note, mind, fin):
        return {'system_success': sys_ok, 'strict_gt': strict, 'contract_note': note,
                'min_dist_gt_m': str(mind), 'final_dist_gt_m': str(fin)}
    ews = 'ended_without_success'
    assert OV.classify(row('True', 'True', None, 0.01, 0.02)) is None
    assert OV.classify(row('False', 'False', ews, 0.01, 0.2)) is None
    assert OV.classify(row('True', 'False', '', 0.05, 0.05)) == 'stall_first'
    assert OV.classify(row('True', 'False', ews, 0.1001, 0.1001)) == 'never_in'
    assert OV.classify(row('True', 'False', ews, 0.0048, 0.1354)) == 'slid'
    assert OV.classify(row('True', 'False', ews, 0.05, 0.09)) == 'other'
    gt = [[10.0, 1.0, 0.0, 0.0, 0.9, 0.1], [10.5, 0.03, 0.0, 0.0, 0.54, 0.63],
          [11.0, -0.1, 0.0, 0.0, 0.0, 0.0]]
    p = tmp_path / 'e.json.gz'
    with gzip.open(p, 'wt', encoding='utf-8') as fh:
        json.dump({'episode': {'gt': gt, 't_end': 10.52, 'goal': [0.0, 0.0, 0.0]}}, fh)
    s = OV.at_nav2_end(str(p))
    assert math.isclose(s['v'], 0.54) and math.isclose(s['w'], 0.63)
    assert math.isclose(s['d'], 0.03)
    assert OV.at_nav2_end(str(tmp_path / 'missing.json.gz')) is None
