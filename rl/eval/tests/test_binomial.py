"""단일 비율 — 이론편 「0/240은 seed 42 actor 구성 하나에 대한 진술이다」 수치 재현."""

import math

from amr_rl_eval import binomial as B
from conftest import load_calc
import pytest


def test_cp_zero_240_is_1p24_percent():
    # 이론편: 0/240 의 단측 95 % CP 상한 1 − 0.05^(1/240) = 1.24 %
    u = B.cp_upper(0, 240, 0.95)
    assert u == pytest.approx(1 - 0.05 ** (1 / 240), abs=1e-12)
    assert round(100 * u, 2) == 1.24


def test_cp_zero_30_is_9p5_percent():
    # 이론편: 실효 표본이 30 이면 상한 9.5 %
    assert round(100 * B.cp_upper(0, 30), 2) == 9.50


@pytest.mark.parametrize('n,cp95,cp975,w95,w975,j95,u95', [
    # evaluation_statistics_theory.md §1 (1) 표 (단위 %)
    (30, 9.50, 11.57, 8.27, 11.35, 6.15, 9.21),
    (240, 1.240, 1.525, 1.115, 1.575, 0.796, 1.235),
    (300, 0.994, 1.222, 0.894, 1.264, 0.638, 0.990),
    (600, 0.498, 0.613, 0.449, 0.636, 0.319, 0.497),
    (1000, 0.299, 0.368, 0.270, 0.383, 0.192, 0.299),
])
def test_zero_event_bound_table(n, cp95, cp975, w95, w975, j95, u95):
    tol = 0.006 if n == 30 else 0.0006
    assert 100 * B.cp_upper(0, n, 0.95) == pytest.approx(cp95, abs=tol)
    assert 100 * B.cp_upper(0, n, 0.975) == pytest.approx(cp975, abs=tol)
    assert 100 * B.wilson_upper(0, n, 0.95) == pytest.approx(w95, abs=tol)
    assert 100 * B.wilson_upper(0, n, 0.975) == pytest.approx(w975, abs=tol)
    assert 100 * B.jeffreys_upper(0, n, 0.95) == pytest.approx(j95, abs=tol)
    assert 100 * B.beta_posterior(0, n, 1, 1)['upper'] == pytest.approx(u95, abs=tol)


def test_wilson_zero_closed_form():
    # x = 0 이면 Wilson 상한 = z²/(n + z²)
    z = 1.959963984540054
    assert B.wilson_interval(0, 240, z)[1] == pytest.approx(z * z / (240 + z * z), rel=1e-12)


def test_rule_of_three():
    assert B.rule_of_three(240) == pytest.approx(0.0125)
    # rule of three 는 −ln 0.05 = 2.996 의 근사 — CP 와 1 % 안
    assert B.rule_of_three(240) == pytest.approx(B.cp_upper(0, 240), rel=0.01)


@pytest.mark.parametrize('a,b,p_below', [
    # evaluation_statistics_theory.md §1 (2): 0/240 의 P(p < 1 %)
    (0.5, 0.5, 0.972), (1, 1, 0.911), (2, 98, 0.853), (1, 49, 0.945),
])
def test_prior_sensitivity(a, b, p_below):
    assert B.beta_posterior(0, 240, a, b, 0.01)['p_below'] == pytest.approx(p_below, abs=6e-4)


@pytest.mark.parametrize('p0,conf,c,n', [
    # §1 (3) 무사고 / c 회 허용 실증 표본수
    (0.02, 0.90, 0, 114), (0.02, 0.90, 1, 194), (0.02, 0.95, 0, 149), (0.02, 0.95, 1, 236),
    (0.02, 0.95, 2, 313), (0.01, 0.95, 0, 299), (0.01, 0.95, 1, 473), (0.005, 0.95, 0, 598),
    (0.003, 0.95, 0, 998), (0.001, 0.90, 0, 2302),
])
def test_demonstration_n(p0, conf, c, n):
    assert B.c_failure_n(p0, conf, c) == n
    if c == 0:
        assert B.zero_failure_n(p0, conf) == n


@pytest.mark.parametrize('p,pass30', [(0.005, 0.860), (0.01, 0.740), (0.02, 0.545),
                                      (0.05, 0.215)])
def test_spec_30_trial_oc(p, pass30):
    # 명세 '30 회 0 충돌' 판정은 충돌률 2 % 정책도 54.5 % 확률로 통과한다
    assert B.pass_probability(p, 30) == pytest.approx(pass30, abs=6e-4)


@pytest.mark.parametrize('icc,ub', [(0.0, 1.24), (0.1, 2.10), (0.3, 3.80), (0.5, 5.46),
                                    (1.0, 9.50)])
def test_cluster_design_effect(icc, ub):
    # §1 (5): 30 시나리오 × 8 회, ρ 에 따른 근사 상한
    assert 100 * B.cp_upper_effective(240, 8, icc) == pytest.approx(ub, abs=0.006)


@pytest.mark.parametrize('p,wald,wilson', [(0.5, 2401, 2398), (0.8, 1537, 1535),
                                           (0.9, 865, 868), (0.95, 457, 469),
                                           (0.98, 189, 223)])
def test_precision_n(p, wald, wilson):
    # §7 (b) 2D 성공률 ±2 %p
    assert B.precision_n(p, 0.02, method='wald') == wald
    assert B.precision_n(p, 0.02, method='wilson') == wilson


def test_cp_two_sided_matches_soak_report():
    # performance.md: 소크 55/56 의 CP 95 % 구간 [90.45 %, 99.95 %]
    lo, hi = B.cp_interval(55, 56)
    assert round(100 * lo, 2) == 90.45
    assert round(100 * hi, 2) == 99.95


def test_cross_check_calc1_wilson():
    ns = load_calc('calc1.py', ['wilson_ub'])
    for n in (30, 240, 300, 600, 1000):
        for x in (0, 1, 3):
            for conf in (0.95, 0.975):
                assert B.wilson_upper(x, n, conf) == pytest.approx(ns['wilson_ub'](x, n, conf),
                                                                   rel=1e-12)


def test_invalid_inputs():
    with pytest.raises(ValueError):
        B.cp_upper(5, 3)
    with pytest.raises(ValueError):
        B.zero_failure_n(0.0)
    assert math.isnan(B.rate_summary(1, 10)['rule3'])
