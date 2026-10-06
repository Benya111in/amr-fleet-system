"""fleetsim 분석·스윕 도우미: 쌍차 통계, MDE, rule of three, 발산 시각, Holm, 범위 파싱."""

import math

import pytest

from amr_fleet.fleetsim.analysis import (
    compare, divergence, holm, index, mde, metric, paired, pearson, zero_upper,
)
from amr_fleet.fleetsim.sweep import parse_range


def test_paired_basic_and_ci():
    a = [10, 12, 11, 13, 9]
    b = [12, 13, 13, 14, 12]
    s = paired(a, b)
    assert s['n'] == 5
    assert s['mean_diff'] == pytest.approx(1.8)
    lo, hi = s['ci95']
    assert lo < 1.8 < hi and lo > 0                 # 모든 쌍이 같은 방향
    assert s['k_pos'] == 5 and s['k_neg'] == 0
    assert s['wilcoxon_p'] == pytest.approx(0.0625)  # n=5 정확 검정 최소 양측 p = 2/2^5
    assert s['dz'] == pytest.approx(1.8 / s['sd_diff'])


def test_paired_all_zero_differences():
    s = paired([1, 2, 3], [1, 2, 3])
    assert s['mean_diff'] == 0 and s['dz'] == 0 and s['wilcoxon_p'] == 1.0
    with pytest.raises(ValueError):
        paired([1], [1, 2])


def test_mde_matches_report_power_table():
    """
    보고서 180행: 검정력 0.8 표준화 쌍차 효과 n=5 → 1.69, n=30 → 0.53 (sd=1 이면 MDE 와 같다).

    여기 식은 t 분위수 근사라 n=5 에서 1.66 (비중심 t 정확값 1.69 보다 조금 작다), n=30 에서는 같다.
    """
    assert mde(1.0, 5) == pytest.approx(1.662, abs=0.005)
    assert mde(1.0, 30) == pytest.approx(0.53, abs=0.01)
    assert math.isinf(mde(1.0, 1))


def test_zero_upper_rule_of_three():
    assert zero_upper(300) == pytest.approx(0.00994, abs=1e-4)   # ≈ 3/300
    assert zero_upper(10) == pytest.approx(0.259, abs=1e-3)      # 보고서 180행 0.26
    assert zero_upper(0) == 1.0


def test_divergence_and_pearson():
    t0 = [(1.0, 't1', 'a'), (2.0, 't2', 'b'), (5.0, 't3', 'c')]
    t1 = [(1.1, 't1', 'a'), (2.5, 't2', 'c'), (5.0, 't3', 'c')]
    assert divergence(t0, t1) == pytest.approx(2.0)
    assert divergence(t0, t0) is None
    assert divergence(t0, t0[:2]) == pytest.approx(5.0)
    assert pearson([1, 2, 3, 4], [2, 4, 6, 8]) == pytest.approx(1.0)
    assert pearson([1, 1, 1], [1, 2, 3]) is None


def test_holm_and_parse_range():
    assert holm([0.01, 0.04, 0.03]) == pytest.approx([0.03, 0.06, 0.06])
    assert parse_range('1-3,7') == [1, 2, 3, 7]


def test_compare_pairs_by_seed():
    rows = []
    for s in (1, 2, 3):
        rows.append({'arm': 'A', 'loss': 0.0, 'fault': 'F0', 'seed': s, 'rep': 0,
                     'completed': 10 + s, 'vote_zone': {'abandons': s}})
        rows.append({'arm': 'B', 'loss': 0.0, 'fault': 'F0', 'seed': s, 'rep': 0,
                     'completed': 12 + s, 'vote_zone': {'abandons': 0}})
    rows.append({'arm': 'B', 'loss': 0.0, 'fault': 'F0', 'seed': 4, 'rep': 0, 'completed': 0})
    idx = index(rows)
    c = compare(idx, 'A', 'B', 0.0, 'F0', 'completed')
    assert c['n'] == 3 and c['mean_diff'] == pytest.approx(2.0)
    assert metric(rows[0], 'vote_zone.abandons') == 1.0
    assert metric(rows[0], 'nope') is None


def _rows(arm, fault, loss, vals, metric_name, rep=0):
    return [{'arm': arm, 'loss': loss, 'fault': fault, 'seed': s, 'rep': rep,
             metric_name: v} for s, v in enumerate(vals, 1)]


def test_primary_decision_rules(monkeypatch):
    """사전 등록 판정 규칙: 우월은 Holm p·CI·k+ 모두, 동등은 CI 가 ±margin 안."""
    from amr_fleet.fleetsim import plan, report
    monkeypatch.setattr(plan, 'PRIMARY', [
        {'id': 'S', 'a': 'A', 'b': 'B', 'loss': 0.2, 'fault': 'F1', 'metric': 'completed',
         'kind': 'superiority', 'pred': ''},
        {'id': 'E', 'a': 'A', 'b': 'C', 'loss': 0.0, 'fault': 'F0', 'metric': 'latency_rmean',
         'kind': 'equivalence', 'pred': ''},
        {'id': 'U', 'a': 'A', 'b': 'D', 'loss': 0.0, 'fault': 'F0', 'metric': 'latency_rmean',
         'kind': 'equivalence', 'pred': ''},
    ])
    n = 30
    rows = (_rows('A', 'F1', 0.2, [5] * n, 'completed')
            + _rows('B', 'F1', 0.2, [20 + (i % 3) for i in range(n)], 'completed')
            + _rows('A', 'F0', 0.0, [150 + (i % 5) for i in range(n)], 'latency_rmean')
            + _rows('C', 'F0', 0.0, [150 + (i % 5) + (1 if i % 2 else -1) for i in range(n)],
                    'latency_rmean')
            + _rows('D', 'F0', 0.0, [150 + (i % 5) + (40 if i % 2 else -40) for i in range(n)],
                    'latency_rmean'))
    res = {c['id']: c for c in report.primary(index(rows))}
    assert res['S']['decision'] == '우월'
    assert res['E']['decision'].startswith('동등')
    assert res['U']['decision'] == '판정 불가'


def _hold_row(arm, seed, fault='F1', loss=0.2, completed=4, zones=None, pre=0):
    zones = zones or {}
    return {'arm': arm, 'loss': loss, 'fault': fault, 'seed': seed, 'rep': 0,
            'completed': completed, 'completed_after_fault': 0, 'latency_rmean': 100.0,
            'cooccupancy_events': sum(zones.values()), 'cooccupancy_zones': zones,
            'cooccupancy_pre_fault': pre, 'cooccupancy_s': float(sum(zones.values())),
            'stuck_end': 0, 'dup_tasks': 0, 'q_bins': [0] * 20}


def test_hold_report_classifies_dock_and_prefault_events_and_p1h():
    """리뷰 B1 보고(report hold): 도크(결정점 미달) 사건 · 고장 전 사건 분리, P1h 판정 규칙."""
    from amr_fleet.fleetsim.report import hold_markdown, hold_safety
    rows = []
    for s in range(1, 31):
        z = {'dock_a_approach': 1, 'x_bc_1': 1} if s <= 3 else {}
        rows.append(_hold_row('A1h_nearest_hold', s, completed=5, zones=z, pre=1 if z else 0))
        rows.append(_hold_row('A1_nearest_central', s, completed=4))
        rows.append(_hold_row('A9_hybrid_vote', s, completed=20 + s % 3))
    saf = {(x['arm'], x['fault'], x['loss']): x for x in hold_safety(rows)}
    h = saf[('A1h_nearest_hold', 'F1', 0.2)]
    assert (h['cooc_runs'], h['cooc_events'], h['cooc_dock'], h['cooc_pre']) == (3, 6, 3, 3)
    assert h['cooc_upper'] is None
    a = saf[('A1_nearest_central', 'F1', 0.2)]
    assert a['cooc_runs'] == 0 and a['cooc_upper'] == pytest.approx(1 - 0.05 ** (1 / 30))
    md, data = hold_markdown(rows)
    assert data['primary']['decision'] == '우월' and data['primary']['k_pos'] == 30
    assert 'P1h' in md
