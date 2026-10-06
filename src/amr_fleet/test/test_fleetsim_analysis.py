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
