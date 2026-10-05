"""
시뮬레이터 간 순위 상관(SRCC) — 2D·Isaac·Gazebo 에서 같은 정책 집합의 순위가 같은가.

근거: 실무편 「시뮬레이터 간 타당성은 SRCC로 직접 잰다」 (Kadian et al. arXiv 1912.06321),
이론편 「희귀 사건 가속, CVaR, SRCC의 유한표본 성질」, evaluation_statistics_theory.md §6 (2).

  N = 정책 수가 지배한다: N = 5 면 ρ ≥ 0.9 여야 단측 5 % 유의, N = 4 면 완전 일치도 p = 1/24.
  관측 ρ = 0.9 에서 Bonett–Wright 95 % 구간 하한 > 0.5 가 되려면 N ≥ 10.
  같은 비교군의 시드들은 독립 정책이 아니다 → 구간이 낙관적 (보고서에 명시).

부트스트랩은 **시나리오 수준**이다: 공통 시나리오를 복원 추출해 두 시뮬레이터의 정책별 평균을 다시
계산하고 ρ 를 다시 잰다 (정책 재표집은 동점이 생겨 쓰지 않는다).
"""

import itertools
import math

import numpy as np
from scipy import stats


def spearman(a, b):
    """Spearman ρ (동점은 평균 순위)."""
    return float(stats.spearmanr(a, b)[0])


def kendall(a, b):
    """Kendall τ-b."""
    return float(stats.kendalltau(a, b)[0])


def bonett_wright_ci(rho, n, conf=0.95):
    """Fisher z 척도 Spearman 구간, SE = √((1 + ρ²/2)/(N − 3)) (Bonett & Wright 2000)."""
    if n <= 3:
        return -1.0, 1.0
    rho = min(max(rho, -0.999999), 0.999999)
    z = math.atanh(rho)
    se = math.sqrt((1 + rho * rho / 2) / (n - 3))
    q = stats.norm.ppf(1 - (1 - conf) / 2)
    return math.tanh(z - q * se), math.tanh(z + q * se)


def permutation_p(a, b, reps=200000, seed=0, exact_max_n=8):
    """
    단측(양의 상관) 순열 p — N ≤ exact_max_n 이면 전수, 아니면 몬테카를로.

    P(ρ_perm ≥ ρ_obs). 정수 순위에서 부동소수 비교 오차를 막으려 1e-9 여유를 둔다.
    """
    a = stats.rankdata(a)
    b = stats.rankdata(b)
    n = a.size
    obs = spearman(a, b)
    if n <= exact_max_n:
        rhos = np.array([spearman(a, np.array(p)) for p in itertools.permutations(b)])
        return float((rhos >= obs - 1e-9).mean())
    rng = np.random.default_rng(seed)
    cnt = 0
    for _ in range(reps):
        if spearman(a, rng.permutation(b)) >= obs - 1e-9:
            cnt += 1
    return (cnt + 1) / (reps + 1)


def critical_rho(n, alpha=0.05, reps=200000, seed=0):
    """단측 α 의 최소 유의 ρ (N ≤ 8 전수, 그 밖은 순열 모의) — calc4 의 표와 같은 정의."""
    if n <= 8:
        perms = np.array(list(itertools.permutations(range(n))))
        d2 = ((perms - np.arange(n)) ** 2).sum(1)
        rho = 1 - 6 * d2 / (n * (n * n - 1))
    else:
        rng = np.random.default_rng(seed)
        base = np.arange(n)
        rho = np.array([stats.spearmanr(base, rng.permutation(n))[0] for _ in range(reps)])
        rho = np.round(rho, 6)
    crit = None
    for v in np.unique(rho)[::-1]:
        if (rho >= v - 1e-9).mean() <= alpha:
            crit = float(v)
        else:
            break
    return crit


def scenario_bootstrap(sim_a, sim_b, reps=2000, conf=0.95, seed=0, method='spearman'):
    """
    시나리오 수준 부트스트랩 SRCC.

    sim_a, sim_b: (정책 수 × 공통 시나리오 수) 점수 행렬 — 같은 행·열 순서.
    반환 dict: rho(전체 평균으로 잰 ρ), lo, hi, n_policies, n_scenarios.
    """
    a = np.asarray(sim_a, dtype=float)
    b = np.asarray(sim_b, dtype=float)
    if a.shape != b.shape:
        raise ValueError('두 시뮬레이터 행렬의 모양이 다르다')
    f = spearman if method == 'spearman' else kendall
    est = f(np.nanmean(a, 1), np.nanmean(b, 1))
    rng = np.random.default_rng(seed)
    n_s = a.shape[1]
    boots = []
    for _ in range(reps):
        idx = rng.integers(0, n_s, n_s)
        ra, rb = np.nanmean(a[:, idx], 1), np.nanmean(b[:, idx], 1)
        if np.ptp(ra) == 0 or np.ptp(rb) == 0:
            continue
        boots.append(f(ra, rb))
    lo, hi = (np.percentile(boots, [100 * (1 - conf) / 2, 100 * (1 + conf) / 2])
              if boots else (math.nan, math.nan))
    return {'rho': est, 'lo': float(lo), 'hi': float(hi), 'n_policies': a.shape[0],
            'n_scenarios': n_s, 'n_boot_valid': len(boots)}


def srcc_report(policy_scores_a, policy_scores_b, conf=0.95):
    """정책별 점수 두 벡터 → ρ, τ, BW 구간, 순열 p, N 과 '판단 가능 여부'(N ≥ 10)."""
    a = np.asarray(policy_scores_a, dtype=float)
    b = np.asarray(policy_scores_b, dtype=float)
    n = a.size
    rho = spearman(a, b)
    lo, hi = bonett_wright_ci(rho, n, conf)
    return {'n_policies': n, 'rho': rho, 'tau': kendall(a, b), 'bw_lo': lo, 'bw_hi': hi,
            'perm_p': permutation_p(a, b, reps=20000), 'enough_policies': bool(n >= 10)}
