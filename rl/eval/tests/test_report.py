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
    assert s['n_policies'] == 11 and s['rho'] > 0.7
    # 충족 기준 (§6.10): 두 하한이 모두 0.5 초과 — 이 합성 자료는 넘는다 (ρ ≈ 0.9)
    assert s['bw_lo'] > 0.5 and s['boot_lo'] > 0.5 and s['verdict'] == R.MET


def test_srcc_few_policies_undecidable():
    rows = synth.make_dataset(seed=9, n_seeds=2, levels=8, eps_per_level=10)
    s = R.srcc_table(rows, reps=100)
    assert s and s[0]['n_policies'] == 5 and s[0]['verdict'] == R.UNDECIDABLE


def test_spec_checks(good):
    _, res = good
    sp = {(s['arm'], s['policy_id'], s['scenario_set']): s for s in res['spec']}
    a = sp[('A_dwa', 'dwa@synthetic', 'gz_new600')]
    assert a['blocks'] == 10 and a['contact_verdict'] == R.MET
    assert a['oc_pass_at_2pct'] == pytest.approx(0.545, abs=1e-3)


def test_spec_checks_do_not_mix_sets_or_policies():
    """
    명세 점검은 (비교군, 정책, 묶음) 마다 따로, 08 형 동적 장애물 묶음만 본다.

    리뷰 반례: 기준선 CSV(gz_s08_actor42, exec 0–239) 와 A 의 gz_new600 을 함께 넣으면 exec_index 로
    섞여 정렬돼 첫 30 묶음이 두 묶음 15 개씩으로 채워졌다.
    """
    base = os.path.join(os.path.dirname(SCRIPTS), 'baseline', 'dwa_gz_s08_episodes.csv')
    rows = schema.read_csv(base)
    new = [_ep('A_dwa', s, 'collision' if s == 45 else 'success') for s in range(60)]
    for r in new:
        r['max_dev_m'] = 0.5
    s07 = [dict(_ep('A_dwa', g, 'success', sset='gz_s07'), scenario_id='07_path_tracking_cte',
                max_dev_m=3.0, return_max_s=9.0) for g in range(5)]
    sp = {(s['arm'], s['policy_id'], s['scenario_set']): s
          for s in R.spec_checks(rows + new + s07)}
    assert set(sp) == {('A_dwa', 'dwa@cebd351', 'gz_s08_actor42'),
                       ('A_dwa', 'dwa@bcc2cca', 'gz_s08_actor42'),
                       ('A_dwa', 'dwa@80874d3', 'gz_s08_actor42'),
                       ('A_dwa', 'dwa@x', 'gz_new600')}
    s08 = [sp[k] for k in sp if k[2] == 'gz_s08_actor42']
    assert sorted(s['blocks'] for s in s08) == [2, 3, 3]
    assert all(s['contact_verdict'] == R.MET for s in s08)
    assert max(s['dev_max'] for s in s08) < 1.0 and max(s['return_max'] for s in s08) < 5.0
    g = sp[('A_dwa', 'dwa@x', 'gz_new600')]
    # 접촉은 두 번째 30 묶음에만 — 한 묶음이라도 접촉이 있으면 미달
    assert (g['blocks'], g['blocks_zero_contact'], g['contact_verdict']) == (2, 1, R.NOT_MET)


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


def test_gate_needs_both_conditions():
    """접촉은 낮지만 성공률이 DWA 보다 크게 낮은 체크포인트 → 미달 (두 조건 AND)."""
    rows = synth.make_dataset(seed=11, n_seeds=2, levels=20, eps_per_level=40, isaac=False,
                              sim2d_skill={'C_hybrid': 0.80},
                              sim2d_collision={'C_hybrid': 0.0, 'B_ppo': 0.05},
                              gz_n=30, gz_secondary=False)
    g = {x['policy_id']: x for x in R.gate_2d(rows)}
    c = g['C_hybrid/s0/best']
    assert c['cp_upper'] <= R.PREREG['gate_collision_cp_upper']     # 접촉 조건은 통과
    assert c['tango_z'] > -1.96                                     # 성공률 NI 는 실패
    assert c['verdict'] == R.NOT_MET
    b = g['B_ppo/s0/best']
    assert b['cp_upper'] > R.PREREG['gate_collision_cp_upper'] and b['verdict'] == R.NOT_MET


def test_safety_alarm_counts_reference_contacts():
    """DWA 도 같은 만큼 부딪치면 경보가 울리지 않는다 (누적 k_r 을 센다)."""
    def mk(arm, coll):
        return {'arm': arm, 'outcome': 'collision' if coll else 'success', 'contacts': int(coll)}
    pairs = []
    for i in range(240):
        t = mk('C_hybrid', i in (10, 50, 239))
        t['exec_index'] = i
        pairs.append((t, mk('A_dwa', i in (5, 60, 200))))
    al = R.safety_alarm(pairs)
    assert not al['fired'] and (al['k_t'], al['k_r']) == (3, 3)


def _ep(arm, seed, outcome, sim='gazebo', sset='gz_new600'):
    """짝 시험용 에피소드 한 행 (접촉은 outcome 에서)."""
    r = dict.fromkeys(schema.NAMES)
    r.update(schema_version=schema.SCHEMA_VERSION, arm=arm,
             policy_id='dwa@x' if arm == 'A_dwa' else f'{arm}/s0/best',
             train_seed=None if arm == 'A_dwa' else 0, simulator=sim, scenario_set=sset,
             scenario_id='G', scenario_seed=seed, difficulty='d', episode=0, run_tag='t',
             exec_index=seed, commit='x', safety_node='on', outcome=outcome,
             contacts=int(outcome == 'collision'), time_s=15.0,
             return_max_s=1.0 if arm == 'C_hybrid' else 2.0)
    return r


def test_c_errors_count_as_failures_in_h2():
    """
    리뷰 반례: C 오류 6/306 (1.96 %, 오류율 문턱 안), C만 실패 3, A만 실패 0.

    오류를 짝에서 빼면 z = −2.031 로 '충족' 이 나왔다. §6.6 대로 오류를 실패로 세면 z = −0.060 → 미달.
    """
    rows = []
    for s in range(306):
        rows.append(_ep('A_dwa', s, 'success'))
        rows.append(_ep('C_hybrid', s, 'error' if s >= 300 else
                        ('timeout' if s < 3 else 'success')))
    res = R.confirmatory(rows)
    h = res['hypotheses']
    assert h[0]['verdict'] == R.MET                       # 접촉 SPRT 는 오류를 뺀 300 으로
    assert res['errors'] == 6 and res['n'] == 300
    assert res['pairs'] == 306 and res['pairs_valid'] == 300
    assert h[1]['verdict'] == R.NOT_MET and 'z=-0.060' in h[1]['detail']
    assert 'C만 실패 9' in h[1]['detail'] and 'C 오류 6 는 실패로 셈' in h[1]['detail']
    assert h[2]['verdict'] == R.SKIPPED
    # 같은 자료에서 오류 6 행만 없애면 충족 — 판정을 뒤집은 것이 오류 처리라는 확인
    res2 = R.confirmatory([r for r in rows if r['outcome'] != 'error'])
    assert res2['hypotheses'][1]['verdict'] == R.MET


def test_reference_errors_drop_the_pair_not_count_as_a_failures():
    """A 오류를 A 실패로 세면 C 에 유리해진다 (z −1.692 → −2.808) — 그 짝은 빼야 한다."""
    rows = []
    for s in range(306):
        rows.append(_ep('A_dwa', s, 'error' if s >= 300 else 'success'))
        rows.append(_ep('C_hybrid', s, 'timeout' if s < 4 else 'success'))
    res = R.confirmatory(rows)
    assert res['pairs'] == 300 and res['pairs_ref_error_dropped'] == 6
    h2 = res['hypotheses'][1]
    assert h2['verdict'] == R.NOT_MET and 'z=-1.692' in h2['detail'] and 'A만 실패 0' in h2['detail']


def test_gate_counts_test_errors_as_failures():
    """2D 게이트: 1010 짝, C 오류 10 (1 %), C만 실패 5 → 오류 = 실패면 z −1.169 로 미달 (빼면 −3.388 충족)."""
    rows = []
    for s in range(1010):
        rows.append(_ep('A_dwa', s, 'success', sim='sim2d', sset='sim2d_heldout'))
        rows.append(_ep('C_hybrid', s, 'error' if s >= 1000 else
                        ('timeout' if s < 5 else 'success'), sim='sim2d', sset='sim2d_heldout'))
    g = R.gate_2d(rows)[0]
    assert g['errors'] == 10 and g['n'] == 1000 and g['pairs'] == 1010
    assert g['cp_upper'] <= R.PREREG['gate_collision_cp_upper']
    assert g['tango_z'] == pytest.approx(-1.169, abs=1e-3) and g['verdict'] == R.NOT_MET
    g2 = R.gate_2d([r for r in rows if r['outcome'] != 'error'])[0]
    assert g2['tango_z'] == pytest.approx(-3.388, abs=1e-3) and g2['verdict'] == R.MET


def _reverse_isaac_ranking(rows, s):
    """Isaac 쪽 정책 이름을 2D 순위의 정반대 정책으로 바꾼다 (리뷰 반례)."""
    import numpy as np
    pols = s['policies']
    order = np.argsort(s['scores_a'])
    rev = {pols[order[i]]: pols[order[-1 - i]] for i in range(len(pols))}
    return [dict(r, policy_id=rev[r['policy_id']])
            if r['simulator'] == 'isaac' and r['policy_id'] in rev else r for r in rows]


def test_srcc_reversed_or_weak_ranking_is_not_met():
    """
    리뷰 반례: 정책 N = 11 이면 ρ 와 무관하게 '충족' 이 나왔다.

    순위를 뒤집은 자료 (ρ < 0) 와 약한 자료 (ρ ≈ 0.7, BW 하한 ≈ 0.1) 모두 미달이어야 한다.
    """
    rows = synth.make_dataset(seed=1, n_seeds=5, levels=12, eps_per_level=20, isaac_eps=20,
                              gz_n=30)
    weak = R.srcc_table(rows, reps=200)[0]
    assert weak['n_policies'] == 11 and weak['bw_lo'] < 0.5
    assert weak['verdict'] == R.NOT_MET
    rev = R.srcc_table(_reverse_isaac_ranking(rows, weak), reps=200)[0]
    assert rev['n_policies'] == 11 and rev['rho'] < 0 and rev['verdict'] == R.NOT_MET


@pytest.mark.parametrize('n,bw,boot,want', [
    (11, 0.60, 0.60, R.MET),
    (11, 0.90, 0.30, R.NOT_MET),      # BW 만 넘는다
    (11, 0.30, 0.90, R.NOT_MET),      # 부트스트랩만 넘는다
    (11, 0.50, 0.90, R.NOT_MET),      # 경계는 '초과' 가 아니다
    (11, -0.93, -0.08, R.NOT_MET),
    (9, 0.95, 0.95, R.UNDECIDABLE),
    (11, 0.90, float('nan'), R.UNDECIDABLE),
])
def test_srcc_verdict_needs_both_lower_bounds(n, bw, boot, want):
    assert R.srcc_verdict(n, bw, boot)[0] == want
