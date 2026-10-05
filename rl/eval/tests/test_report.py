"""
판정 파이프라인 — 참값을 아는 합성 데이터로 판정 규칙이 맞게 동작하는지 본다.

합성 데이터는 측정값이 아니다 (amr_rl_eval.synth). 실제 실행 경로(고정 순서·SPRT·짝·게이트)를
모두 밟도록 시나리오를 고른다 (handover1006 §7.3: 실제 시스템이 밟는 경로를 시험하라).
"""

import json
import os

from amr_rl_eval import report as R
from amr_rl_eval import schema, synth
import pytest

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts')


@pytest.fixture(scope='module')
def good():
    rows = synth.make_dataset(seed=1)
    return rows, R.evaluate(rows, reps=300)


def _verdicts(res):
    return [h['verdict'] for h in res['confirmatory']['hypotheses']]


def test_good_hybrid_confirms_h1_to_h4(good):
    _, res = good
    v = _verdicts(res)
    assert v[:4] == [R.MET] * 4
    # 0/300 으로는 δ = 1 %p 를 못 보인다 (381 회 필요) → 미달
    assert v[4] == R.NOT_MET
    assert res['confirmatory']['sprt'].n == 243


def test_collision_heavy_hybrid_stops_sequence():
    rows = synth.make_dataset(seed=2, gz_n=600, gz_collision={'C_hybrid': 0.04})
    res = R.evaluate(rows, reps=200)
    v = _verdicts(res)
    assert v[0] == R.NOT_MET                       # 상한 600 도달, NI 미입증
    assert v[1:] == [R.SKIPPED] * 4                # 고정 순서 — 뒤는 검정하지 않는다
    assert '(참고)' in res['confirmatory']['hypotheses'][1]['detail']


def test_in_progress_is_undecidable():
    rows = synth.make_dataset(seed=3, gz_n=150)
    v = _verdicts(R.evaluate(rows, reps=200))
    assert v[0] == R.UNDECIDABLE and v[1:] == [R.SKIPPED] * 4


def test_missing_exec_index_is_undecidable():
    rows = synth.make_dataset(seed=4, exec_index=False)
    hyp = R.evaluate(rows, reps=200)['confirmatory']['hypotheses']
    assert hyp[0]['verdict'] == R.UNDECIDABLE and 'exec_index' in hyp[0]['detail']


def test_error_rate_over_limit_blocks_all():
    rows = synth.make_dataset(seed=5, gz_error_rate=0.05)
    v = _verdicts(R.evaluate(rows, reps=200))
    assert v == [R.UNDECIDABLE] * 5


def test_errors_count_as_failures_in_success_rate():
    rows = synth.make_dataset(seed=6, gz_error_rate=0.01)
    s = [x for x in R.arm_summary(rows) if x['arm'] == 'C_hybrid' and x['simulator'] == 'gazebo']
    assert s[0]['errors'] > 0
    assert s[0]['success'] + s[0]['errors'] + s[0]['timeouts'] + s[0]['collisions'] == s[0]['n']
    assert s[0]['n_valid'] == s[0]['n'] - s[0]['errors']


def test_worse_success_fails_h2():
    rows = synth.make_dataset(seed=7, gz_success={'C_hybrid': 0.80})
    v = _verdicts(R.evaluate(rows, reps=200))
    assert v[0] == R.MET and v[1] == R.NOT_MET and v[2:] == [R.SKIPPED] * 3


def test_two_primary_checkpoints_need_preregistration(good):
    rows, _ = good
    extra = [dict(r, policy_id='C_hybrid/s1/best') for r in rows
             if r['simulator'] == 'gazebo' and r['arm'] == 'C_hybrid']
    for r in extra:
        r['episode'] = 1
    res = R.confirmatory(rows + extra)
    assert all(h['verdict'] == R.UNDECIDABLE for h in res['hypotheses'])
    res2 = R.confirmatory(rows + extra, primary_policy='C_hybrid/s0/best')
    assert res2['hypotheses'][0]['verdict'] == R.MET


def test_gate_2d(good):
    _, res = good
    g = {x['policy_id']: x for x in res['gate_2d']}
    assert all(g[f'C_hybrid/s{s}/best']['verdict'] == R.MET for s in range(5))
    assert all(g[f'B_ppo/s{s}/best']['verdict'] == R.NOT_MET for s in range(5))


def test_seed_aggregate_and_poi(good):
    _, res = good
    assert 'gazebo' not in res['seed_aggregate']
    sim2d = {r['arm']: r for r in res['seed_aggregate']['sim2d']}
    assert sim2d['C_hybrid']['lo'] <= sim2d['C_hybrid']['iqm'] <= sim2d['C_hybrid']['hi']
    assert sim2d['C_hybrid']['iqm'] > sim2d['B_ppo']['iqm']
    assert sim2d['B_ppo']['poi_vs_ref'] < 0.5 < sim2d['C_hybrid']['poi_vs_ref']


def test_few_seeds_flagged():
    rows = synth.make_dataset(seed=8, n_seeds=3, levels=8, eps_per_level=10, isaac=False)
    sa = {r['arm']: r for r in R.evaluate(rows, reps=100)['seed_aggregate']['sim2d']}
    assert R.UNDECIDABLE in sa['C_hybrid'].get('note', '')


def test_cvar_and_srcc(good):
    _, res = good
    cv = [c for c in res['cvar'] if c['alpha'] == 0.10]
    assert {c['arm'] for c in cv} == {'A_dwa', 'B_ppo', 'C_hybrid'}
    assert all(c['enough'] for c in cv)
    s = res['srcc'][0]
    assert s['n_policies'] == 11 and s['verdict'] == R.MET and s['rho'] > 0.7


def test_srcc_few_policies_undecidable():
    rows = synth.make_dataset(seed=9, n_seeds=2, levels=8, eps_per_level=10)
    s = R.srcc_table(rows, reps=100)
    assert s and s[0]['n_policies'] == 5 and s[0]['verdict'] == R.UNDECIDABLE


def test_spec_checks(good):
    _, res = good
    sp = {s['arm']: s for s in res['spec']}
    assert sp['A_dwa']['blocks'] == 10 and sp['A_dwa']['contact_verdict'] == R.MET
    assert sp['A_dwa']['oc_pass_at_2pct'] == pytest.approx(0.545, abs=1e-3)


def test_secondary_is_exploratory(good):
    _, res = good
    sec = {s['arm']: s for s in res['secondary']}
    assert 'B_ppo' in sec and 'holm_p' in sec['B_ppo']


def test_markdown_renders_all_sections(good):
    _, res = good
    md = R.render_markdown(res)
    for h in ('## 1. 확증', '## 2. 비교군 요약', '## 3. 시드 집계', '## 4. 2D → Gazebo',
              '## 5. 보조', '## 6. 최악 난도', '## 7. 시뮬레이터 간', '## 8. 명세 4.7',
              '## 9. Gazebo 짝 차이'):
        assert h in md
    assert '미측정' in md          # jerk 열이 비어 있다


def test_baseline_csv_feeds_report():
    """커밋한 DWA 기준선 CSV 가 판정 파이프라인에 그대로 들어간다 (확증 묶음이 아니므로 판정 불가)."""
    path = os.path.join(os.path.dirname(SCRIPTS), 'baseline', 'dwa_gz_s08_episodes.csv')
    rows = schema.read_csv(path)
    res = R.evaluate(rows, reps=100)
    assert _verdicts(res) == [R.UNDECIDABLE] * 5
    s = [x for x in res['summary'] if x['arm'] == 'A_dwa'][0]
    assert (s['n'], s['collisions'], s['success']) == (240, 0, 240)
    assert s['interventions_n'] == 0       # 미측정


def test_cli_synthetic(tmp_path):
    import subprocess
    import sys
    out = tmp_path / 'o'
    r = subprocess.run([sys.executable, '-W', 'ignore', os.path.join(SCRIPTS, 'evaluate.py'),
                        '--synthetic', '--out-dir', str(out), '--reps', '200'],
                       capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr
    for f in ('report.md', 'report.json', 'seed_iqm.png', 'sprt.png', 'time_ecdf.png',
              'srcc.png', 'synthetic_episodes.csv'):
        assert (out / f).exists(), f
    assert '합성 데이터' in (out / 'report.md').read_text(encoding='utf-8')
    js = json.loads((out / 'report.json').read_text(encoding='utf-8'))
    assert js['confirmatory']['hypotheses'][0]['verdict'] == R.MET
    assert len(schema.read_csv(str(out / 'synthetic_episodes.csv'))) == js['n_rows']


def test_safety_alarm_fires_on_excess_contacts():
    rows = synth.make_dataset(seed=10, gz_n=300, gz_collision={'C_hybrid': 0.03})
    res = R.confirmatory(rows)
    al = res['alarm']
    assert al['fired'] and al['k_t'] - al['k_r'] >= 3 and al['p'] < 0.05
    assert '울림' in R.render_markdown(R.evaluate(rows, reps=100))


def test_safety_alarm_silent_when_clean(good):
    _, res = good
    al = res['confirmatory']['alarm']
    assert not al['fired'] and al['k_t'] == 0 and al['test'] == 'barnard'


def test_safety_alarm_threshold_three_vs_zero():
    """240 쌍, A 0 접촉, C 접촉 3 회 → Barnard p = 0.0481 로 경보 (Boschloo 였다면 5 회)."""
    def mk(arm, coll):
        return {'arm': arm, 'outcome': 'collision' if coll else 'success', 'contacts': int(coll)}
    pairs = []
    for i in range(240):
        t = mk('C_hybrid', i in (10, 50, 239))
        t['exec_index'] = i
        pairs.append((t, mk('A_dwa', False)))
    al = R.safety_alarm(pairs)
    assert al['fired'] and al['n'] == 240 and al['k_t'] == 3
    assert al['p'] == pytest.approx(0.0481, abs=3e-4)
    pairs2 = [(dict(t, outcome='success', contacts=0) if t['exec_index'] == 239 else t, r)
              for t, r in pairs]
    assert not R.safety_alarm(pairs2)['fired']
