"""SRCC 소표본 성질과 CVaR 편향 — evaluation_statistics_theory.md §6 재현."""

import math

from amr_rl_eval import cvar as V
from amr_rl_eval import planning as PL
from amr_rl_eval import srcc as R
import numpy as np
import pytest


@pytest.mark.parametrize('n,crit', [(4, 1.000), (5, 0.900), (6, 0.829), (8, 0.643)])
def test_srcc_critical_exact(n, crit):
    assert R.critical_rho(n) == pytest.approx(crit, abs=6e-4)


def test_srcc_n4_perfect_agreement_min_p():
    # N = 4 면 순위가 완벽히 일치해도 p = 1/24 = 0.042 가 한계
    assert R.permutation_p([1, 2, 3, 4], [1, 2, 3, 4]) == pytest.approx(1 / 24)


def test_srcc_critical_n10_mc():
    # N = 10: 단측 5 % 임계 0.564 (모의)
    assert R.critical_rho(10, reps=40000, seed=1) == pytest.approx(0.564, abs=0.02)


@pytest.mark.parametrize('n,r,lo,hi', [
    (4, 0.8, -0.82, 1.00), (5, 0.8, -0.46, 0.99), (5, 0.9, -0.17, 1.00), (6, 0.9, 0.13, 0.99),
    (8, 0.8, 0.09, 0.97), (8, 0.9, 0.41, 0.99), (10, 0.8, 0.24, 0.96), (10, 0.9, 0.53, 0.98),
    (15, 0.9, 0.66, 0.97),
])
def test_bonett_wright_table(n, r, lo, hi):
    a, b = R.bonett_wright_ci(r, n)
    assert a == pytest.approx(lo, abs=0.006) and b == pytest.approx(hi, abs=0.006)


def test_n10_needed_for_lower_bound_above_half():
    # 이론편: 관측 ρ = 0.9 에서 하한 > 0.5 는 N ≥ 10
    lows = {n: R.bonett_wright_ci(0.9, n)[0] for n in range(4, 16)}
    assert min(n for n, v in lows.items() if v > 0.5) == 10


def test_scenario_bootstrap_recovers_rank_agreement():
    rng = np.random.default_rng(0)
    skill = np.linspace(0.6, 0.95, 10)
    a = np.clip(skill[:, None] + rng.normal(0, 0.05, (10, 30)), 0, 1)
    b = np.clip(skill[:, None] - 0.05 + rng.normal(0, 0.05, (10, 30)), 0, 1)
    out = R.scenario_bootstrap(a, b, reps=500)
    assert out['rho'] > 0.8 and out['lo'] <= out['rho'] <= out['hi']
    rep = R.srcc_report(a.mean(1), b.mean(1))
    assert rep['enough_policies'] and rep['perm_p'] < 0.01


def test_scenario_bootstrap_shape_mismatch():
    with pytest.raises(ValueError):
        R.scenario_bootstrap(np.zeros((3, 4)), np.zeros((3, 5)))


def test_cvar_plugin_simple():
    rates = [0.1, 0.5, 0.9, 1.0, 0.95, 0.8, 0.7, 0.6, 0.99, 0.97]
    assert V.cvar_plugin(rates, 0.2) == pytest.approx(0.3)     # 최악 2 개 평균
    assert V.cvar_plugin(rates, 0.01) == pytest.approx(0.1)    # 최소 1 개


def test_cvar_crossfit_brackets_truth():
    rng = np.random.default_rng(3)
    p = rng.beta(9, 1, 300)
    outcomes = [rng.random(50) < pi for pi in p]
    both = V.cvar_both(outcomes, 0.1, seed=0)
    truth = np.sort(p)[:30].mean()
    assert both['plugin'] < truth < both['crossfit'] + 0.02
    assert both['episodes_per_level_min'] == 50


@pytest.mark.parametrize('m,plug,cross', [(10, -0.085, 0.082), (50, -0.019, 0.030)])
def test_cvar_bias_simulation(m, plug, cross):
    # §6 (1): 레벨 500, Beta(9,1), α 10 % — plug-in 비관 / cross-fit 낙관
    bp, bc, tv = V.simulate_bias(episodes=m, reps=300, seed=4)
    assert bp == pytest.approx(plug, abs=0.006)
    assert bc == pytest.approx(cross, abs=0.008)
    assert tv == pytest.approx(0.698, abs=0.01)


def test_cvar_loss_upper_tail():
    x = np.arange(1, 101, dtype=float)
    assert V.cvar_loss(x, 0.1) == pytest.approx(95.5)          # 상위 10 개 평균
    assert math.isnan(V.cvar_loss([np.nan], 0.1))


def test_plan_table_matches_theory():
    rows = {r['purpose']: r for r in PL.plan_table()}
    assert rows['충돌 NI δ=2%p 고정 n']['n'] == 189
    assert round(rows['충돌 NI δ=2%p 고정 n']['hours'], 1) == 10.5
    assert rows['충돌 NI δ=2%p SPRT(Ville)']['n'] == 243
    assert round(rows['충돌 NI δ=2%p SPRT(Ville)']['hours'], 1) == 13.5
    assert rows['충돌 NI δ=1%p 고정 n']['n'] == 381
    assert rows['충돌 NI δ=1%p SPRT(Ville)']['n'] == 489
    assert rows['충돌 NI δ=2%p 비교군 4 개 Holm(α/4)']['n'] == 306
    assert rows['성공률 0.90→0.95 McNemar mid-p (ψ=0.07)']['n'] == 200
    assert round(rows['성공률 0.90→0.95 McNemar mid-p (ψ=0.07)']['hours']) == 22
    assert rows['2D 성공률 ±2%p (p=0.9, Wald)']['n'] == 865
    srow = rows['SRCC ρ=0.9 BW 하한 > 0.5']
    assert srow['bw_lo_n10'] > 0.5 > srow['bw_lo_n8']
