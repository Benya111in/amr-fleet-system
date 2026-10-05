"""DWA 기준선 추출 — 합성 로그 트리로 출처·집계 규칙을, 실제 로그로 performance.md 수치를 확인."""

import csv
from datetime import datetime, timedelta, timezone
import json
import os

from amr_rl_eval import baseline as BL
from amr_rl_eval import schema
import pytest

COLS = ['trial', 'reached', 'time_s', 'contacts', 'min_distance_m', 'nearest', 'min_ttc_s',
        'max_deviation_m', 'episodes', 'return_s', 'n_plans', 'max_dev_first_m', 'return_first_s']


def _mk_run(root, tag, trials, head='abc1234', head_lag_s=2.0, failed=()):
    d = os.path.join(root, tag, BL.SCEN_08)
    os.makedirs(d)
    start = datetime(2026, 10, 1, 2, 0, 0, tzinfo=timezone.utc)
    with open(os.path.join(d, 'result.json'), 'w', encoding='utf-8') as f:
        json.dump({'started_at': start.isoformat(), 'settings': {'seed': 7},
                   'host': {'start': {'loadavg': [3.0]}, 'end': {'loadavg': [9.0]}},
                   'checks': [{'name': n, 'passed': n not in failed}
                              for n in ('contacts', 'return')],
                   'measurements': {'summary': {}}}, f)
    with open(os.path.join(d, 'avoidance.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        w.writeheader()
        for t in trials:
            w.writerow(t)
    if head is not None:
        hp = os.path.join(root, tag, 'HEAD.txt')
        with open(hp, 'w') as f:
            f.write(head + '\n')
        ts = (start - timedelta(seconds=head_lag_s)).timestamp()
        os.utime(hp, (ts, ts))
        with open(os.path.join(root, tag, 'uncommitted.txt'), 'w') as f:
            f.write('')


def _trial(i, contacts=0, reached=1, t=15.0, dev_first=0.2, ret_first=0.0, n_plans=1):
    return {'trial': i, 'reached': reached, 'time_s': t, 'contacts': contacts,
            'min_distance_m': 0.5 + 0.01 * i, 'nearest': 'worker', 'min_ttc_s': 2.0,
            'max_deviation_m': 0.3, 'episodes': 0, 'return_s': 0.5, 'n_plans': n_plans,
            'max_dev_first_m': dev_first, 'return_first_s': ret_first}


def test_synthetic_pooling_and_provenance(tmp_path):
    root = str(tmp_path)
    _mk_run(root, 'R1', [_trial(i) for i in range(4)])
    _mk_run(root, 'R2', [_trial(0, contacts=1, reached=0), _trial(1, dev_first=1.5, n_plans=3),
                         _trial(2, ret_first=6.0), _trial(3, t=40.0)],
            head=None, failed=('return',))
    _mk_run(root, 'R3', [_trial(i) for i in range(4)], head='abc1234', head_lag_s=-5.0)
    s = BL.summarize_08(root, ('R1', 'R2', 'R3'))
    p = s['pooled']
    assert p['n'] == 12 and p['contact_trials'] == 1 and p['reached'] == 11
    assert p['dev_first_over_1m'] == 1 and p['return_first_over_5s'] == 1 and p['replanned'] == 1
    assert p['time_max'] == 40.0
    assert p['harness_failed_runs'] == ['R2']
    runs = {r['tag']: r for r in s['runs']}
    assert runs['R1']['head_status'] == '있음' and runs['R1']['head'] == 'abc1234'
    assert runs['R2']['head_status'].startswith('없음') and runs['R2']['head'] is None
    # HEAD.txt 가 실행 시작보다 **뒤에** 쓰였으면 같은 실행 래퍼가 쓴 것이 아니다 → 재사용 의심
    assert runs['R3']['head_status'] == '재사용 의심'
    # 결과 CSV 계약: 접촉 시행은 collision, 미도달은 timeout, 커밋 모름은 unknown
    eps = s['episodes']
    assert len(eps) == 12
    by = {(e['run_tag'], e['scenario_seed']): e for e in eps}
    assert by[('R2', 0)]['outcome'] == 'collision'
    assert by[('R2', 0)]['commit'] == 'unknown'
    assert by[('R1', 0)]['outcome'] == 'success'
    # 같은 정책(커밋) 의 두 번째 실행은 episode 1 → 짝 키가 겹치지 않는다
    assert by[('R3', 0)]['episode'] == 1
    path = tmp_path / 'eps.csv'
    schema.write_csv(str(path), eps)
    assert len(schema.read_csv(str(path))) == 12


def test_reached_but_timeout_outcome(tmp_path):
    root = str(tmp_path)
    _mk_run(root, 'R1', [_trial(0, reached=0)])
    eps = BL.summarize_08(root, ('R1',))['episodes']
    assert eps[0]['outcome'] == 'timeout'


REAL = BL.DEFAULT_LOG_ROOT
needs_real = pytest.mark.skipif(not os.path.isdir(os.path.join(REAL, 'regG')),
                                reason='측정 로그 없음')


@needs_real
def test_real_logs_reproduce_performance_md():
    """performance.md §2 의 08 수치 — 0/240, 이탈 0.565~0.883, 첫 계획 45·15, 재계획 82."""
    s = BL.summarize_08()
    p = s['pooled']
    assert (p['n'], p['contact_trials'], p['reached']) == (240, 0, 240)
    devs = [r['max_dev'] for r in s['runs']]
    rets = [r['return_max'] for r in s['runs']]
    firsts = [r['return_first_max'] for r in s['runs']]
    assert (round(min(devs), 3), round(max(devs), 3)) == (0.565, 0.883)
    assert (round(min(rets), 2), round(max(rets), 2)) == (0.94, 3.82)
    assert (round(min(firsts), 2), round(max(firsts), 2)) == (3.38, 8.70)
    assert p['dev_first_over_1m'] == 45 and p['return_first_over_5s'] == 15
    assert sum(r['return_first_over_5s'] > 0 for r in s['runs']) == 6
    assert p['replanned'] == 82
    assert round(100 * p['contact_rate']['cp_upper'], 2) == 1.24
    assert {r['head'] for r in s['runs']} == {'cebd351', 'bcc2cca', '80874d3'}
    assert all(r['head_status'] == '있음' for r in s['runs'])
    assert p['harness_failed_runs'] == ['S0a', 'S0b', 'S0c', 'S0d']


@needs_real
def test_real_cte_matches_result_json():
    rows = {r['tag']: r for r in BL.summarize_07()}
    assert all(r['cte_matches_result_json'] for r in rows.values())
    g = rows['regG']['cte']
    assert round(100 * g['직선']['mean'], 2) == 1.86 and round(100 * g['곡선']['mean'], 2) == 3.44
    assert round(100 * g['직선']['max'], 1) == 60.2
    assert rows['E07']['eta']['max_err_pct'] == 5.55


@needs_real
def test_real_excluded_runs_have_contacts():
    exc = {r['tag']: r for r in BL.summarize_excluded_08()}
    assert exc['S0f']['contacts'] == 1 and exc['S0f']['head'] == 'bcc2cca'
    assert exc['itest_fx2']['contacts'] == 1 and exc['itest_fx2']['head'] is None


def test_committed_episode_csv_is_valid():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(here, 'baseline', 'dwa_gz_s08_episodes.csv')
    rows = schema.read_csv(path)
    assert len(rows) == 240
    assert sum(schema.collided(r) for r in rows) == 0
    assert {r['policy_id'] for r in rows} == {'dwa@cebd351', 'dwa@bcc2cca', 'dwa@80874d3'}
