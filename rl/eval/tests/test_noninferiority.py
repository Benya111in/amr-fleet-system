"""두 비율 NI·우월성 — 이론편 「DWA가 0회이면 …」 절과 evaluation_statistics_theory.md §2 재현."""

import math

from amr_rl_eval import binomial as B
from amr_rl_eval import noninferiority as N
from conftest import load_calc
import numpy as np
import pytest
from scipy import stats


@pytest.mark.parametrize('alpha,n_d1,n_d2', [
    # §2 (2) 표: 0 대 0 사건 NI 의 닫힌 해 n_t > z²(1−δ)/δ
    (0.05, 268, 133), (0.025, 381, 189), (0.0125, 498, 247), (0.00625, 618, 306),
])
def test_ni_zero_event_closed_form(alpha, n_d1, n_d2):
    assert N.ni_n_zero_events(0.01, alpha) == n_d1
    assert N.ni_n_zero_events(0.02, alpha) == n_d2


@pytest.mark.parametrize('delta', [0.01, 0.02])
@pytest.mark.parametrize('n_r', [30, 240, 600, 5000])
def test_dwa_sample_size_does_not_matter(delta, n_r):
    # 이론편: 양측 0 사건이면 DWA 표본 n_r 이 식에 나타나지 않는다
    n = N.ni_n_zero_events(delta)
    zc = N.z_quantile(0.975)
    assert N.fm_z(0, n, 0, n_r, delta) < -zc
    assert N.fm_z(0, n - 1, 0, n_r, delta) >= -zc
    assert N.newcombe_upper(0, n, 0, n_r, 0.975) < delta
    assert N.newcombe_upper(0, n - 1, 0, n_r, 0.975) >= delta
    pt, pr = N.fm_restricted(0, n, 0, n_r, delta)
    assert pt == pytest.approx(delta, abs=1e-9) and pr == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('k,n_d1,n_d2', [(1, 563, 280), (2, 726, 361), (3, 878, 437)])
def test_ni_n_given_events(k, n_d1, n_d2):
    # §2 (2): RL 에서 k 회 접촉 (DWA 0) 일 때 필요한 n_t (Wilson 97.5 % 상한 < δ)
    assert N.ni_n_given_events(k, 0.01) == n_d1
    assert N.ni_n_given_events(k, 0.02) == n_d2


def test_240_cannot_show_delta_1pp():
    # 이론편: RL 240 회로는 0 충돌이어도 δ = 1 %p NI 를 보일 수 없다 (Wilson 97.5 % 상한 1.575 %)
    u = N.newcombe_upper(0, 240, 0, 240, 0.975)
    assert round(100 * u, 3) == 1.575
    assert not N.ni_test(0, 240, 0, 240, 0.01)['ni']
    assert N.ni_test(0, 240, 0, 240, 0.02)['ni']


@pytest.mark.parametrize('x_t,n_t,x_r,n_r,d', [
    (0, 200, 0, 240, 0.02), (3, 300, 1, 250, 0.02), (10, 400, 2, 380, 0.01),
    (0, 50, 5, 50, 0.05), (40, 100, 35, 120, 0.1), (5, 100, 0, 240, 0.02),
    (1, 1000, 0, 240, 0.01), (7, 30, 2, 40, 0.15),
])
def test_fm_closed_form_matches_numeric_and_calc2(x_t, n_t, x_r, n_r, d):
    ns = load_calc('calc2.py', ['fm_restricted', 'fm_z'])
    pt, pr = N.fm_restricted(x_t, n_t, x_r, n_r, d)
    qt, qr = N._fm_restricted_numeric(x_t, n_t, x_r, n_r, d)
    assert pt - pr == pytest.approx(d, abs=1e-12)
    assert pt == pytest.approx(qt, abs=2e-6)
    ct, cr = ns['fm_restricted'](x_t, n_t, x_r, n_r, d)
    assert pt == pytest.approx(ct, abs=2e-6)
    assert N.fm_z(x_t, n_t, x_r, n_r, d) == pytest.approx(ns['fm_z'](x_t, n_t, x_r, n_r, d),
                                                          rel=1e-4, abs=1e-6)


def test_newcombe_matches_calc2():
    ns = load_calc('calc2.py', ['wilson', 'newcombe_U'])
    z = N.z_quantile(0.975)
    for args in [(0, 240, 0, 240), (2, 300, 1, 300), (5, 400, 0, 240), (20, 100, 10, 90)]:
        assert N.newcombe_upper(*args, 0.975) == pytest.approx(ns['newcombe_U'](*args, z),
                                                               rel=1e-12)


def test_newcombe_reference_zero_reduces_to_wilson():
    for n in (100, 240, 400):
        assert N.newcombe_upper(2, n, 0, 240, 0.975) == pytest.approx(
            B.wilson_upper(2, n, 0.975), rel=1e-12)


@pytest.mark.parametrize('d,pt,pr,n80,n90', [
    # §2 (3) 표 괄호 안의 FM 공식값 (군당, α = 0.025)
    (0.01, 0.001, 0.001, 589, 688), (0.01, 0.0025, 0.0025, 784, 968),
    (0.01, 0.005, 0.005, 1101, 1411), (0.01, 0.005, 0.0, 2589, 3259),
    (0.02, 0.001, 0.001, 255, 287), (0.02, 0.0025, 0.0025, 309, 366),
    (0.02, 0.005, 0.005, 389, 480), (0.02, 0.005, 0.0, 496, 592),
])
def test_fm_formula_table(d, pt, pr, n80, n90):
    assert math.ceil(N.fm_n_formula(pt, pr, d, 0.025, 0.8)) == n80
    assert math.ceil(N.fm_n_formula(pt, pr, d, 0.025, 0.9)) == n90


def test_exact_power_grid_point():
    # §2 (3): δ = 2 %, 0.5 % / 0.5 %, FM 정확 검정력 80 % → 370 (격자 10)
    assert N.ni_exact_power(370, 370, 0.005, 0.005, 0.02) >= 0.8
    assert N.ni_exact_power(360, 360, 0.005, 0.005, 0.02) < 0.8


def test_type_one_error_at_boundary_conservative():
    # §2 (3): 경계 p_t = p_r + δ, n = 600, δ = 1 % 에서 실제 1 종 오류는 명목 2.5 % 이하
    for pr in (0.001, 0.005):
        size = N.ni_exact_power(600, 600, pr + 0.01, pr, 0.01)
        assert size <= 0.025


def test_tango_both_zero_equals_unpaired():
    # §2 (4): 짝지은 설계도 양측 0 이면 같은 문턱
    for d in (0.01, 0.02):
        n = N.ni_n_zero_events(d)
        zc = N.z_quantile(0.975)
        assert N.tango_z(0, 0, n, d) < -zc <= N.tango_z(0, 0, n - 1, d)


@pytest.mark.parametrize('b,c,n,d', [(0, 0, 200, 0.02), (3, 1, 300, 0.02), (5, 5, 400, 0.01),
                                     (10, 2, 150, 0.05)])
def test_tango_closed_form_matches_calc3(b, c, n, d):
    ns = load_calc('calc3.py', ['tango_restricted', 'tango_z'])
    q10, q01 = N.tango_restricted(b, c, n, d)
    r10, r01 = ns['tango_restricted'](b, c, n, d)
    assert q01 == pytest.approx(r01, abs=2e-6)
    assert N.tango_z(b, c, n, d) == pytest.approx(ns['tango_z'](b, c, n, d), rel=1e-4, abs=1e-6)


def test_tango_paired_overlap_gain():
    # §2 (4): 둘 다 0.5 %, 충돌의 50 % 가 같은 시드 → δ = 2 % 80 % 검정력 280 쌍 (격자 10)
    assert N.tango_exact_power(280, 0.0025, 0.0025, 0.02) >= 0.8
    assert N.tango_exact_power(270, 0.0025, 0.0025, 0.02) < 0.8


def test_paired_table():
    t = N.paired_table([1, 0, 1, 0, 0], [1, 1, 0, 0, 0])
    assert t == (1, 1, 1, 2)


# ------------------------------------------------------------ 우월성 (안전 경보)

FISHER = [0.500, 0.250, 0.124, 0.062, 0.031, 0.015]
MIDP = [0.250, 0.125, 0.062, 0.031, 0.015, 0.008]
BOSCHLOO_ARMS = [0.4818, 0.2264, 0.1089, 0.0550, 0.0254, 0.0126]


@pytest.mark.parametrize('k', range(1, 7))
def test_fisher_and_midp_match_theory(k):
    # §2 (1) 표 — Fisher·mid-p 는 표 방향과 무관 (조건부)
    assert N.fisher_greater(k, 240, 0, 240) == pytest.approx(FISHER[k - 1], abs=6e-4)
    assert N.fisher_midp_greater(k, 240, 0, 240) == pytest.approx(MIDP[k - 1], abs=6e-4)


@pytest.mark.parametrize('k', range(1, 7))
def test_boschloo_arms_as_columns(k):
    assert N.boschloo_greater(k, 240, 0, 240) == pytest.approx(BOSCHLOO_ARMS[k - 1], abs=1e-4)


def test_boschloo_bruteforce_independent():
    """독립 전수 계산 (scipy 비의존) — 두 군 n = 240 이항, 통계량 = Fisher 단측 p, 귀무 p 위 sup."""
    n1 = n2 = 240
    x1 = np.arange(n1 + 1)[:, None]
    x2 = np.arange(n2 + 1)[None, :]
    t = stats.hypergeom.sf(x1 - 1, n1 + n2, x1 + x2, n1)
    for k, ref in ((4, 0.0550), (5, 0.0254)):
        tobs = stats.hypergeom.sf(k - 1, n1 + n2, k, n1)
        rej = t <= tobs * (1 + 1e-7)
        best = 0.0
        # 상한(sup)은 귀무 p 전 범위에서 잡는다 — 이 표에서는 큰 p 쪽에 있다
        for p in np.concatenate([np.linspace(1e-4, 0.05, 500), np.linspace(0.05, 0.5, 500)]):
            b1 = stats.binom.pmf(np.arange(n1 + 1), n1, p)
            b2 = stats.binom.pmf(np.arange(n2 + 1), n2, p)
            best = max(best, float((b1[:, None] * b2[None, :] * rej).sum()))
        assert best == pytest.approx(ref, abs=5e-4)


def test_calc1_boschloo_used_transposed_table():
    """
    이론편 'Boschloo 0 대 4 → p = 0.043' 의 출처를 재현한다 — 행을 군으로 넣은 표.

    scipy 는 열을 군으로 보므로 그 표는 '총 사건 수 k 를 표본 크기로 고정' 한 다른 설계다.
    """
    k = 4
    transposed = stats.boschloo_exact([[k, 240 - k], [0, 240]], alternative='greater').pvalue
    assert transposed == pytest.approx(0.0430, abs=1e-4)
    assert N.boschloo_greater(k, 240, 0, 240) > 0.05


def test_superiority_alarm_threshold_is_5_not_4():
    assert N.superiority_alarm_k(240, 240, 0, alpha=0.05, k_max=8) == 5
