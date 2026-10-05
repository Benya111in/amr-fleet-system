"""
최악 난도 강건성 — 레벨별 성공률의 CVaR (최악 α % 레벨 평균) 과 연속 손실의 CVaR.

근거: 실무편 (No Regrets, arXiv 2408.15099 — 낮은 α 에서만 방법 차가 드러난다),
이론편 「희귀 사건 가속, CVaR, SRCC의 유한표본 성질」, evaluation_statistics_theory.md §6 (1).

  plug-in    : 레벨 성공률 추정치로 최악 k 개를 골라 그 추정치를 평균 → **비관 편향**
  cross-fit  : 에피소드를 반으로 나눠 앞 반으로 고르고 뒤 반으로 평가 → **낙관 편향**
  둘은 참값을 위아래로 감싼다 — 둘 다 보고하고 레벨당 에피소드 수를 같이 적는다.
  레벨당 50 회에서 편향 −1.9 / +3.0 %p (이론편 모의).
연속 손실(최소 이격 부족분 등)은 Rockafellar–Uryasev 표본식 min_c {c + E[(L − c)+]/α}.
"""

import math

import numpy as np


def worst_k(n_levels, alpha):
    """최악 α 비율에 해당하는 레벨 수 (최소 1) — calc4 와 같이 int(α·L)."""
    return max(1, int(alpha * n_levels))


def cvar_plugin(level_success, alpha=0.1):
    """레벨 성공률 배열 → 최악 ⌊αL⌋ 개 평균 (plug-in)."""
    s = np.sort(np.asarray(level_success, dtype=float))
    return float(s[:worst_k(s.size, alpha)].mean())


def cvar_crossfit(level_outcomes, alpha=0.1, seed=0, symmetric=True):
    """
    교차적합 CVaR.

    level_outcomes: 레벨별 0/1 성공 배열의 리스트 (길이가 달라도 된다).
    각 레벨의 에피소드를 무작위로 반씩 나눠 A 로 최악 레벨을 고르고 B 로 평가한다.
    symmetric=True 면 A·B 역할을 바꾼 값과 평균한다 (calc4 는 한 방향).
    """
    rng = np.random.default_rng(seed)
    a_rates, b_rates = [], []
    for o in level_outcomes:
        o = np.asarray(o, dtype=float)
        if o.size < 2:
            raise ValueError('cross-fit 은 레벨당 에피소드가 2 이상이어야 한다')
        idx = rng.permutation(o.size)
        h = o.size // 2
        a_rates.append(o[idx[:h]].mean())
        b_rates.append(o[idx[h:]].mean())
    a_rates, b_rates = np.array(a_rates), np.array(b_rates)
    k = worst_k(a_rates.size, alpha)
    one = float(b_rates[np.argsort(a_rates, kind='stable')[:k]].mean())
    if not symmetric:
        return one
    two = float(a_rates[np.argsort(b_rates, kind='stable')[:k]].mean())
    return 0.5 * (one + two)


def cvar_both(level_outcomes, alpha=0.1, seed=0):
    """plug-in·cross-fit 과 레벨당 에피소드 수 요약을 한 번에."""
    rates = [float(np.mean(o)) for o in level_outcomes]
    sizes = [len(o) for o in level_outcomes]
    return {'alpha': alpha, 'n_levels': len(rates), 'k': worst_k(len(rates), alpha),
            'plugin': cvar_plugin(rates, alpha),
            'crossfit': cvar_crossfit(level_outcomes, alpha, seed),
            'episodes_per_level_min': int(min(sizes)),
            'episodes_per_level_median': float(np.median(sizes))}


def cvar_loss(losses, alpha=0.1):
    """
    연속 손실의 상위 α 꼬리 평균 (Rockafellar–Uryasev 표본식).

    min_c { c + mean((L − c)+)/α } — c 는 표본 (1−α) 분위에서 최소가 된다.
    """
    x = np.sort(np.asarray(losses, dtype=float))
    x = x[~np.isnan(x)]
    if x.size == 0:
        return math.nan
    best = math.inf
    for c in np.unique(x):
        best = min(best, c + np.mean(np.maximum(x - c, 0.0)) / alpha)
    return float(best)


def simulate_bias(n_levels=500, episodes=50, alpha=0.1, reps=400, seed=1, a=9.0, b=1.0):
    """
    plug-in·cross-fit 편향 모의 — 레벨 참 성공률 ~ Beta(a, b) (이론편 설정: Beta(9,1), α 10 %).

    반환: (plug-in 평균 편향, cross-fit 평균 편향, 참 CVaR 평균). calc5 와 같은 한 방향 cross-fit.
    """
    rng = np.random.default_rng(seed)
    k = worst_k(n_levels, alpha)
    e_p, e_c, tv_all = [], [], []
    h = episodes // 2
    for _ in range(reps):
        p = rng.beta(a, b, n_levels)
        tv = np.sort(p)[:k].mean()
        y = rng.binomial(episodes, p) / episodes
        e_p.append(np.sort(y)[:k].mean() - tv)
        y1 = rng.binomial(h, p) / h
        y2 = rng.binomial(episodes - h, p) / (episodes - h)
        e_c.append(y2[np.argsort(y1)[:k]].mean() - tv)
        tv_all.append(tv)
    return float(np.mean(e_p)), float(np.mean(e_c)), float(np.mean(tv_all))
