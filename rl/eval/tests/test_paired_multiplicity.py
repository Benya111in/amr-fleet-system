"""짝지은 비교 (McNemar·Wilcoxon·HL) 와 다중 비교 (Holm·고정 순서)."""

import math

from amr_rl_eval import multiplicity as M
from amr_rl_eval import paired as P
from conftest import load_calc
import numpy as np
import pytest
from scipy import stats


@pytest.mark.parametrize('pa,pb,psi,connor,unpaired', [
    # evaluation_statistics_theory.md §2 (5) 표 (양측 0.05, 80 %)
    (0.90, 0.95, 0.07, 218, 435), (0.90, 0.95, 0.14, 438, 435), (0.90, 0.93, 0.05, 434, 1356),
    (0.90, 0.93, 0.12, 1045, 1356), (0.95, 0.97, 0.04, 783, 1506),
])
def test_connor_table(pa, pb, psi, connor, unpaired):
    d = pb - pa
    assert math.ceil(P.connor_n((psi + d) / 2, (psi - d) / 2)) == connor
    pbar = (pa + pb) / 2
    za, zb = stats.norm.ppf(0.975), stats.norm.ppf(0.8)
    nu = (za * math.sqrt(2 * pbar * (1 - pbar))
          + zb * math.sqrt(pa * (1 - pa) + pb * (1 - pb))) ** 2 / d ** 2
    assert math.ceil(nu) == unpaired


@pytest.mark.parametrize('pa,pb,psi,n', [(0.90, 0.95, 0.07, 200), (0.90, 0.93, 0.05, 410)])
def test_mcnemar_exact_n(pa, pb, psi, n):
    d = pb - pa
    assert P.mcnemar_exact_n((psi + d) / 2, (psi - d) / 2) == n


def test_mcnemar_power_matches_calc4():
    ns = load_calc('calc4.py', ['mcnemar_exact_power'])
    for n, p10, p01 in ((200, 0.06, 0.01), (150, 0.05, 0.02)):
        assert P.mcnemar_exact_power(n, p10, p01) == pytest.approx(
            ns['mcnemar_exact_power'](n, p10, p01), rel=1e-10)


def test_mcnemar_midp_values():
    assert P.mcnemar_midp(0, 0) == 1.0
    # 10 대 2: 양측 mid-p = 2·(P(X>10) + ½P(X=10)), X ~ Bin(12, ½)
    b = stats.binom(12, 0.5)
    assert P.mcnemar_midp(10, 2) == pytest.approx(2 * (b.sf(10) + 0.5 * b.pmf(10)))
    assert P.mcnemar_midp(10, 2, 'greater') < 0.05 < P.mcnemar_midp(10, 2, 'less')
    assert P.mcnemar_midp(5, 5) == pytest.approx(1.0)


def test_mcnemar_from_outcomes():
    t = [1, 1, 1, 0, 1, 1]
    r = [1, 0, 0, 0, 1, 0]
    out = P.mcnemar_from_outcomes(t, r)
    assert out['n10'] == 3 and out['n01'] == 0 and out['diff'] == pytest.approx(0.5)


def test_hodges_lehmann_shift():
    rng = np.random.default_rng(0)
    d = rng.normal(-1.5, 1.0, 60)
    est, lo, hi = P.hodges_lehmann(d)
    assert lo < est < hi
    assert est == pytest.approx(-1.5, abs=0.4)
    walsh = [(d[i] + d[j]) / 2 for i in range(60) for j in range(i, 60)]
    assert est == pytest.approx(np.median(walsh))


def test_wilcoxon_drops_nan_pairs():
    t = np.array([10.0, 11.0, np.nan, 9.0, 8.5, 10.2, 9.9, 9.1])
    r = np.array([11.0, 12.5, 10.0, np.nan, 9.9, 11.1, 10.8, 10.0])
    out = P.wilcoxon_paired(t, r)
    assert out['n_pairs'] == 6 and out['hl'] < 0
    ref = stats.wilcoxon(t[[0, 1, 4, 5, 6, 7]] - r[[0, 1, 4, 5, 6, 7]])
    assert out['p_value'] == pytest.approx(ref.pvalue)


def test_wilcoxon_all_zero():
    out = P.wilcoxon_paired([1, 2, 3], [1, 2, 3])
    assert out['p_value'] == 1.0


def test_holm_known_example():
    rej, adj = M.holm([0.01, 0.04, 0.03, 0.005], alpha=0.05)
    assert adj.tolist() == pytest.approx([0.03, 0.06, 0.06, 0.02])
    assert rej.tolist() == [True, False, False, True]


def test_holm_alpha_levels_match_theory():
    # 이론편: 비교군 4 개 Holm → 첫 단계 α/4 = 0.00625 (0 충돌 NI 306 회의 근거)
    assert M.holm_alpha(1, 4, 0.025) == pytest.approx(0.00625)
    assert M.holm_alpha(4, 4, 0.025) == pytest.approx(0.025)


def test_fixed_sequence_stops_at_first_failure():
    assert M.fixed_sequence([0.001, 0.02, 0.2, 0.001], 0.025) == [True, True, False, False]
    assert M.fixed_sequence([None, 0.001], 0.025) == [False, False]


def test_fixed_sequence_fwer_simulation():
    """모든 귀무가 참이면 고정 순서의 가족 오류율 ≤ α (균등 p 모의)."""
    rng = np.random.default_rng(1)
    p = rng.random((20000, 4))
    any_rej = [any(M.fixed_sequence(row, 0.025)) for row in p]
    assert np.mean(any_rej) <= 0.025 + 0.004
