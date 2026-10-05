"""시드 집계 — IQM·층화 부트스트랩·개선 확률 (rliable 과 일치) 과 시드 수 계획 재현."""

import importlib.util

from amr_rl_eval import aggregate as A
import numpy as np
import pytest
from scipy import stats

HAS_RLIABLE = importlib.util.find_spec('rliable') is not None
needs_rliable = pytest.mark.skipif(not HAS_RLIABLE, reason='rliable 없음')


def _scores(seed=0, runs=10, tasks=4, mu=0.85):
    rng = np.random.default_rng(seed)
    base = rng.uniform(mu - 0.1, mu + 0.05, tasks)
    return np.clip(base[None, :] + rng.normal(0, 0.03, (runs, tasks)), 0, 1)


def test_iqm_is_25pct_trimmed_mean():
    s = _scores()
    assert A.iqm(s) == pytest.approx(stats.trim_mean(s.ravel(), 0.25))


@needs_rliable
def test_iqm_and_poi_match_rliable():
    from rliable import metrics
    x, y = _scores(1), _scores(2, mu=0.80)
    assert A.iqm(x) == pytest.approx(metrics.aggregate_iqm(x))
    assert A.median(x) == pytest.approx(metrics.aggregate_median(x))
    assert A.mean(x) == pytest.approx(metrics.aggregate_mean(x))
    assert A.optimality_gap(x) == pytest.approx(metrics.aggregate_optimality_gap(x))
    assert A.probability_of_improvement(x, y) == pytest.approx(
        metrics.probability_of_improvement(x, y))


def test_poi_ties_are_half():
    x = np.array([[0.5, 0.7], [0.5, 0.7]])
    assert A.probability_of_improvement(x, x) == pytest.approx(0.5)


@needs_rliable
def test_numpy_bootstrap_matches_rliable():
    sd = {'C': _scores(3), 'B': _scores(4, mu=0.8)}
    ours = A.interval_estimates(sd, A.iqm, reps=4000, seed=0, backend='numpy')
    ref = A.interval_estimates(sd, A.iqm, reps=4000, seed=0, backend='rliable')
    for k in sd:
        assert ours[k][0] == pytest.approx(ref[k][0])
        assert ours[k][1] == pytest.approx(ref[k][1], abs=0.006)
        assert ours[k][2] == pytest.approx(ref[k][2], abs=0.006)


def test_bootstrap_ci_contains_estimate_and_shrinks_with_runs():
    small = A.stratified_bootstrap(_scores(5, runs=5), reps=1500)
    large = A.stratified_bootstrap(_scores(5, runs=40), reps=1500)
    for est, lo, hi in (small, large):
        assert lo <= est <= hi
    assert (large[2] - large[1]) < (small[2] - small[1])


def test_stratified_resampling_keeps_tasks():
    # task 2 개, run 사이 변동 0 → 층화 부트스트랩 구간 폭 0
    s = np.array([[0.2, 0.9]] * 6)
    est, lo, hi = A.stratified_bootstrap(s, A.mean, reps=300)
    assert lo == pytest.approx(est) and hi == pytest.approx(est)


def test_bootstrap_coverage_reasonable():
    """층화 부트스트랩 평균 CI 의 포함률 — run 10 개에서 대략 명목 근처 (rliable 논문 ~95 %)."""
    rng = np.random.default_rng(7)
    true_mean = 0.8
    hit = 0
    trials = 150
    for i in range(trials):
        s = np.clip(true_mean + rng.normal(0, 0.05, (10, 3)), 0, 1)
        _, lo, hi = A.stratified_bootstrap(s, A.mean, reps=600, seed=i)
        hit += lo <= true_mean <= hi
    assert 0.85 <= hit / trials <= 1.0


def test_poi_ci():
    est, lo, hi = A.probability_of_improvement_ci(_scores(8), _scores(9, mu=0.7), reps=500)
    assert lo <= est <= hi and est > 0.8


def test_performance_profile_monotone():
    prof = A.performance_profile(_scores(), np.linspace(0, 1, 11))
    assert np.all(np.diff(prof) <= 0) and prof[0] == 1.0


@pytest.mark.parametrize('sb,k5m200,k5m1000,k10,k20,kmin', [
    # evaluation_statistics_theory.md §3 (3) 표 (p = 0.9, 단위 %p)
    (0.0, 2.63, 1.18, 0.68, 0.44, 4), (0.01, 2.91, 1.71, 0.99, 0.65, 5),
    (0.02, 3.62, 2.75, 1.58, 1.04, 8), (0.03, 4.56, 3.91, 2.25, 1.47, 12),
    (0.05, 6.74, 6.32, 3.64, 2.38, 28),
])
def test_seed_halfwidth_table(sb, k5m200, k5m1000, k10, k20, kmin):
    assert 100 * A.seed_halfwidth(sb, 200, 5) == pytest.approx(k5m200, abs=0.006)
    assert 100 * A.seed_halfwidth(sb, 1000, 5) == pytest.approx(k5m1000, abs=0.006)
    assert 100 * A.seed_halfwidth(sb, 1000, 10) == pytest.approx(k10, abs=0.006)
    assert 100 * A.seed_halfwidth(sb, 1000, 20) == pytest.approx(k20, abs=0.006)
    assert A.seeds_needed(sb, 0.02, 1000) == kmin
