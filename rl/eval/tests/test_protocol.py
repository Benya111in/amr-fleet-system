"""PROTOCOL.md 의 수치가 코드(report.PREREG·통계 함수·기준선 CSV)와 같은지 — 문서와 판정 코드가 갈라지지 않게."""

import math
import os
import re

from amr_rl_eval import baseline as BL
from amr_rl_eval import noninferiority as N
from amr_rl_eval import report as R
from amr_rl_eval import schema, sequential
import numpy as np
import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with open(os.path.join(HERE, 'PROTOCOL.md'), encoding='utf-8') as _f:
    DOC = _f.read()


def _row(label):
    m = re.search(r'^\| ' + re.escape(label) + r' \| (.+?) \|', DOC, re.M)
    assert m, label
    return m.group(1)


def test_prereg_table_matches_code():
    p = R.PREREG
    assert _row('δ_c') == str(p['delta_collision'])
    assert _row('δ_c,aux') == str(p['delta_collision_aux'])
    assert _row('δ_s') == str(p['delta_success'])
    sprt = _row('SPRT')
    assert f'p0 = {p["sprt_p0"]}' in sprt and f'p1 = {p["sprt_p1"]}' in sprt
    assert f'상한 {p["sprt_n_max"]}' in sprt
    assert f'{math.log(1 / p["alpha"]):.3f}' in sprt
    assert f'{100 * p["gate_collision_cp_upper"]:.1f} %' in _row('2D 게이트')
    assert f'{100 * p["gate_success_delta"]:.0f} %p' in _row('2D 게이트')
    assert _row('오류율 상한') == f'{100 * p["error_rate_max"]:.0f} %'
    assert _row('최소 시드') == str(p['min_seeds'])
    assert _row('SRCC 최소 정책 수') == str(p['srcc_min_policies'])
    assert f'모두 > {p["srcc_ci_lower_min"]}' in _row('SRCC 충족')
    assert _row('CVaR 최소 레벨당 에피소드') == str(p['cvar_min_episodes_per_level'])


def test_sprt_numbers_in_text():
    d = R.sprt_design()
    assert f'{d.step_ok:+.5f}' in DOC and f'{d.step_fail:.3f}'.replace('-', '−') in DOC
    assert f'{d.n_zero} 번째에서' in DOC
    assert f'약 {round(d.cost_per_failure)} 회' in DOC


def test_alarm_threshold_text():
    assert R.PREREG['alarm_test'] == 'barnard' and 'Barnard 단측 p < 0.05' in _row('안전 경보')
    k = N.superiority_alarm_k(240, 240, 0, alpha=R.PREREG['alarm_alpha'], k_max=8)
    assert f'**0 대 {k}** (p = {N.barnard_greater(k, 240, 0, 240):.4f})' in DOC
    assert f'Boschloo 는 0 대 5 (p = {N.boschloo_greater(5, 240, 0, 240):.4f}' in DOC
    assert f'0 대 4 는 p = {N.boschloo_greater(4, 240, 0, 240):.4f}' in DOC


def test_h5_reachability_text():
    assert f'C 가 {N.ni_n_zero_events(0.01)} 회 이상' in DOC or '381 회 이상' in DOC
    assert N.ni_n_zero_events(0.01) == 381


def test_scenario_sets_are_known_to_report():
    assert f'`{R.PREREG["confirm_set"]}`' in DOC
    assert '`gz_s08_actor42`' in DOC
    eps = schema.read_csv(os.path.join(HERE, 'baseline', 'dwa_gz_s08_episodes.csv'))
    assert {e['scenario_set'] for e in eps} == {'gz_s08_actor42'}


def test_baseline_table_matches_committed_csv():
    eps = schema.read_csv(os.path.join(HERE, 'baseline', 'dwa_gz_s08_episodes.csv'))
    t = np.array([e['time_s'] for e in eps])
    c = np.array([e['min_clearance_m'] for e in eps])
    dev = np.array([e['max_dev_m'] for e in eps])
    ret = np.array([e['return_max_s'] for e in eps])
    assert (f'{np.percentile(t, 50):.2f} / {np.percentile(t, 90):.2f} / {t.max():.2f} s'
            in _row('주행 시간 중앙 / p90 / 최대'))
    assert f'{c.min():.3f} / {np.percentile(c, 5):.3f} m' in _row('최소 이격 최소 / p5')
    assert f'{dev.max():.3f} m' in _row('최대 이탈 (활성) 최대 · 첫 계획 > 1 m')
    assert f'{ret.max():.2f} s' in _row('복귀 (활성) 최대 · 첫 계획 > 5 s')


@pytest.mark.skipif(not os.path.isdir(os.path.join(BL.DEFAULT_LOG_ROOT, 'regG')),
                    reason='측정 로그 없음')
def test_baseline_first_plan_counts_match_logs():
    p = BL.summarize_08()['pooled']
    assert f'{p["dev_first_over_1m"]}/240' in _row('최대 이탈 (활성) 최대 · 첫 계획 > 1 m')
    assert f'{p["return_first_over_5s"]}/240' in _row('복귀 (활성) 최대 · 첫 계획 > 5 s')
    assert _row('재계획 시행') == f'{p["replanned"]}/240'


def test_ville_oc_quoted_matches_simulation():
    d = sequential.SPRTDesign()
    for p_true, quoted in ((0.0025, 0.966), (0.005, 0.779), (0.02, 0.020)):
        assert f'{quoted}' in DOC
        r = sequential.simulate_oc(d, p_true, reps=20000, seed=21)
        assert r['p_ni'] == pytest.approx(quoted, abs=0.012)


def test_error_rule_example_numbers():
    """§6.6 의 예 (C 오류 6/306 → H2 z) 가 Tango 계산과 같은지."""
    d = R.PREREG['delta_success']
    z_fail = N.tango_z(3 + 6, 0, 306, d)
    z_drop = N.tango_z(3, 0, 300, d)
    assert f'z = {z_fail:.3f} (미달)'.replace('-', '−') in DOC
    assert f'z = {z_drop:.3f} 로 충족'.replace('-', '−') in DOC
    assert z_fail > -N.z_quantile(1 - R.PREREG['alpha']) > z_drop


def test_fallback_run_count_reaches_sprt_end():
    """대체안: 0 접촉 SPRT 종료(243) 에 필요한 30 시행 실행 수가 문서와 같다 (8 실행 = 240 으로는 안 끝난다)."""
    d = R.sprt_design()
    runs = math.ceil(d.n_zero / 30)
    assert 8 * 30 < d.n_zero and runs == 9
    assert f'**최소 {runs} 실행**' in DOC and f'상한 {d.n_max} = {d.n_max // 30} 실행' in DOC
