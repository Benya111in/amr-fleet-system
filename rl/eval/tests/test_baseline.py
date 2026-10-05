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
    # episode = 비교군 안의 실행 순번 — R1 과 R3 은 같은 커밋이고 R2 는 커밋을 모르지만 셋이
    # 이어서 0, 1, 2 를 받는다 (커밋마다 0 부터 다시 매기면 짝 키가 커밋 사이에서 겹친다)
    assert {e['run_tag']: e['episode'] for e in eps} == {'R1': 0, 'R2': 1, 'R3': 2}
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
    assert p['chronological']
    assert [e['exec_index'] for e in s['episodes']] == list(range(240))


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


BASELINE_CSV = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'baseline', 'dwa_gz_s08_episodes.csv')


def test_committed_episode_csv_is_valid():
    rows = schema.read_csv(BASELINE_CSV)
    assert len(rows) == 240
    assert sum(schema.collided(r) for r in rows) == 0
    assert {r['policy_id'] for r in rows} == {'dwa@cebd351', 'dwa@bcc2cca', 'dwa@80874d3'}
    # 짝 키: episode = PRIMARY_08 순서의 실행 순번, 240 행이 모두 다른 키
    assert {r['run_tag']: r['episode'] for r in rows} == {
        t: i for i, t in enumerate(BL.PRIMARY_08)}
    assert len({schema.pair_key(r) for r in rows}) == 240


def _c_runs_8x30(tmp_path):
    """시험 비교군 C 의 08 실행 8 개 × 30 시행 (합성 로그 → s08_runs_to_episodes, 실제 변환 경로)."""
    root = str(tmp_path / 'c_logs')
    tags = [f'C{k}' for k in range(8)]
    for k, tag in enumerate(tags):
        _mk_run(root, tag, [_trial(i, t=14.0 + 0.01 * k) for i in range(30)], head='feed123')
    rows, _ = BL.s08_runs_to_episodes(root, tags, 'C_hybrid', 'C_hybrid/s3/best', 3)
    return rows


def test_fallback_set_pairs_all_240_with_committed_baseline(tmp_path):
    """확증 대체안 gz_s08_actor42: 기준선 240 과 C 8×30 이 240 쌍, C 의 k 번째 실행 ↔ A 의 k 번째 실행."""
    from collections import Counter

    from amr_rl_eval import report as R
    a_rows = schema.read_csv(BASELINE_CSV)
    c_rows = _c_runs_8x30(tmp_path)
    pairs = R.pair_rows(c_rows, a_rows)
    assert len(pairs) == 240
    assert Counter(a['run_tag'] for _, a in pairs) == {t: 30 for t in BL.PRIMARY_08}
    assert all(a['run_tag'] == BL.PRIMARY_08[c['episode']] for c, a in pairs)
    assert all(a['scenario_seed'] == c['scenario_seed'] for c, a in pairs)
    pre = dict(R.PREREG, confirm_set=R.PREREG['confirm_set_fallback'])
    res = R.confirmatory(a_rows + c_rows, prereg=pre)
    assert res['pairs'] == 240 and res['alarm']['n'] == 240 and res['n'] == 240
    md = R.render_markdown(R.evaluate(a_rows + c_rows, reps=50, fallback=True))
    assert '`gz_s08_actor42`' in md and '확증 대체안' in md


def test_old_per_commit_numbering_is_rejected(tmp_path):
    """커밋마다 episode 를 0 부터 매긴 기준선(이 수정 전) — 짝짓기와 CSV 읽기가 둘 다 거부한다."""
    from amr_rl_eval import report as R
    a_rows = schema.read_csv(BASELINE_CSV)
    per_commit, seen = [], {}
    for r in a_rows:
        k = seen.setdefault(r['policy_id'], {}).setdefault(r['run_tag'], len(seen[r['policy_id']]))
        per_commit.append(dict(r, episode=k))
    assert len({schema.pair_key(r) for r in per_commit}) == 90
    with pytest.raises(ValueError, match='기준 짝 키 중복'):
        R.pair_rows(_c_runs_8x30(tmp_path), per_commit)
    path = tmp_path / 'old.csv'
    schema.write_csv(str(path), per_commit)
    with pytest.raises(ValueError, match='기준 A_dwa 짝 키 중복'):
        schema.read_csv(str(path))


def test_s08_runs_to_episodes_for_any_arm(tmp_path):
    root = str(tmp_path)
    _mk_run(root, 'C1', [_trial(i) for i in range(3)], head='feed123')
    rows, provs = BL.s08_runs_to_episodes(root, ['C1'], 'C_hybrid', 'C_hybrid/s3/best', 3)
    assert {r['arm'] for r in rows} == {'C_hybrid'} and rows[0]['train_seed'] == 3
    assert rows[0]['policy_id'] == 'C_hybrid/s3/best' and rows[0]['commit'] == 'feed123'
    assert [r['exec_index'] for r in rows] == [0, 1, 2]


def test_s08_cli(tmp_path):
    import subprocess
    import sys
    root = tmp_path / 'logs'
    _mk_run(str(root), 'C1', [_trial(i) for i in range(3)])
    out = tmp_path / 'c.csv'
    script = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'scripts', 's08_to_csv.py')
    r = subprocess.run([sys.executable, script, '--log-root', str(root), '--arm', 'C_hybrid',
                        '--policy-id', 'C/s0', '--out', str(out), 'C1'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert len(schema.read_csv(str(out))) == 3
