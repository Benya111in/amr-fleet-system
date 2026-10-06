"""
학습 시드 집계 — IQM·층화 부트스트랩 CI·개선 확률 (rliable, Agarwal et al. NeurIPS 2021).

근거: 실무편 「0/240 무접촉이 말해 주는 것은 상한 1.24%뿐이다」 절의 시드·집계 문단,
evaluation_statistics_theory.md §3 (b)(c).

점수 행렬 모양은 rliable 과 같다: (run 수 × task 수).
  run  = 학습 시드 (DWA 는 시드 변동이 없으므로 run 1 개)
  task = 시나리오 범주 또는 난도 수준 (PROTOCOL.md §3)
층화 부트스트랩: task 마다 run 을 독립적으로 복원 추출한다 (task_bootstrap=False 기본).

rliable 이 있으면 `backend='rliable'` 로 그 구현을 그대로 쓰고, 없으면 같은 정의의 numpy 구현을 쓴다.
두 구현의 일치는 tests/test_aggregate.py 가 확인한다.
"""

import importlib.util

import numpy as np
from scipy import stats


def _as2d(scores):
    a = np.asarray(scores, dtype=float)
    if a.ndim == 1:
        a = a[:, None]
    if a.ndim != 2:
        raise ValueError('점수는 (run × task) 2 차원이어야 한다')
    return a


def iqm(scores):
    """사분위 평균 — 전체 (run × task) 원소의 25 % 절사 평균 (rliable.metrics.aggregate_iqm)."""
    return float(stats.trim_mean(_as2d(scores), proportiontocut=0.25, axis=None))


def mean(scores):
    """전체 평균."""
    return float(np.mean(_as2d(scores)))


def median(scores):
    """과제(task) 평균 점수의 중앙값 (rliable.metrics.aggregate_median)."""
    return float(np.median(np.mean(_as2d(scores), axis=0)))


def optimality_gap(scores, gamma=1.0):
    """γ 에 못 미치는 양의 평균 (rliable.metrics.aggregate_optimality_gap)."""
    return float(gamma - np.mean(np.minimum(_as2d(scores), gamma)))


AGGREGATES = {'iqm': iqm, 'mean': mean, 'median': median, 'optimality_gap': optimality_gap}


def stratified_bootstrap(scores, func=iqm, reps=2000, conf=0.95, seed=0, task_bootstrap=False):
    """
    층화 부트스트랩 백분위 구간 (numpy 구현).

    반환: (점추정, 하한, 상한). task_bootstrap=True 면 task 도 복원 추출한다.
    """
    a = _as2d(scores)
    n_run, n_task = a.shape
    rng = np.random.default_rng(seed)
    est = func(a)
    boots = np.empty(reps)
    cols = np.arange(n_task)
    for i in range(reps):
        tasks = rng.integers(0, n_task, n_task) if task_bootstrap else cols
        idx = rng.integers(0, n_run, (n_run, n_task))
        boots[i] = func(a[idx, tasks[None, :]])
    lo, hi = np.percentile(boots, [100 * (1 - conf) / 2, 100 * (1 + conf) / 2])
    return float(est), float(lo), float(hi)


def interval_estimates(score_dict, func=iqm, reps=2000, conf=0.95, seed=0, backend='auto'):
    """
    비교군별 (점추정, 하한, 상한).

    backend: 'rliable' | 'numpy' | 'auto'(rliable 이 import 되면 그것). 반환 dict 의 값은 3-튜플.
    """
    if backend == 'auto':
        backend = 'rliable' if importlib.util.find_spec('rliable') else 'numpy'
    if backend == 'numpy':
        return {k: stratified_bootstrap(v, func, reps, conf, seed) for k, v in score_dict.items()}
    from rliable import library as rly
    sd = {k: _as2d(v) for k, v in score_dict.items()}
    pt, ci = rly.get_interval_estimates(
        sd, lambda x: np.array([func(x)]), reps=reps, confidence_interval_size=conf,
        random_state=np.random.RandomState(seed))
    return {k: (float(pt[k][0]), float(ci[k][0, 0]), float(ci[k][1, 0])) for k in sd}


def probability_of_improvement(scores_x, scores_y):
    """
    P(X > Y) — task 마다 Mann–Whitney U/(n_x n_y) (동점 ½) 의 평균 (rliable 정의).

    개선 크기는 반영하지 않으므로 IQM 차이와 함께 보고한다.
    """
    x, y = _as2d(scores_x), _as2d(scores_y)
    if x.shape[1] != y.shape[1]:
        raise ValueError('task 수가 다르다')
    probs = []
    for m in range(x.shape[1]):
        xm, ym = x[:, m], y[:, m]
        gt = (xm[:, None] > ym[None, :]).sum()
        eq = (xm[:, None] == ym[None, :]).sum()
        probs.append((gt + 0.5 * eq) / (len(xm) * len(ym)))
    return float(np.mean(probs))


def probability_of_improvement_ci(scores_x, scores_y, reps=2000, conf=0.95, seed=0):
    """개선 확률과 두 군을 각각 층화 재표집한 백분위 구간."""
    x, y = _as2d(scores_x), _as2d(scores_y)
    rng = np.random.default_rng(seed)
    est = probability_of_improvement(x, y)
    boots = np.empty(reps)
    for i in range(reps):
        xi = x[rng.integers(0, x.shape[0], x.shape), np.arange(x.shape[1])[None, :]]
        yi = y[rng.integers(0, y.shape[0], y.shape), np.arange(y.shape[1])[None, :]]
        boots[i] = probability_of_improvement(xi, yi)
    lo, hi = np.percentile(boots, [100 * (1 - conf) / 2, 100 * (1 + conf) / 2])
    return est, float(lo), float(hi)


def performance_profile(scores, taus):
    """성능 프로파일 — 각 τ 에서 점수 > τ 인 (run, task) 비율 (rliable run-score 분포)."""
    a = _as2d(scores).ravel()
    return np.array([(a > t).mean() for t in np.asarray(taus, dtype=float)])


def seed_halfwidth(sigma_between, episodes_per_seed, n_seeds, p=0.9, conf=0.95):
    """
    시드 계층 반폭 t_{K−1}·√(σ_b²/K + p(1−p)/(K m)) — 이론편 「학습 비교군의 정밀도」.

    σ_b = 시드 간 성공률 표준편차, m = 시드당 에피소드, K = 시드 수.
    """
    k, m = n_seeds, episodes_per_seed
    t = stats.t.ppf(1 - (1 - conf) / 2, k - 1)
    return float(t * np.sqrt(sigma_between ** 2 / k + p * (1 - p) / (k * m)))


def seeds_needed(sigma_between, half_width, episodes_per_seed=1000, p=0.9, conf=0.95, k_max=500):
    """반폭 ≤ half_width 를 얻는 최소 시드 수 (K ≥ 3)."""
    for k in range(3, k_max + 1):
        if seed_halfwidth(sigma_between, episodes_per_seed, k, p, conf) <= half_width:
            return k
    return None
