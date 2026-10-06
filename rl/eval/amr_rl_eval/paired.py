"""
짝지은 비교 — 같은 시나리오 시드에서 두 정책을 돌린 쌍.

근거: 실무편 「0/240 …」 절 (성공 = McNemar, 연속 지표 = Wilcoxon 부호순위),
evaluation_statistics_theory.md §2 (5) (calc4.py 의 McNemar 표본수).

  McNemar mid-p (Fagerland, Lydersen, Laake 2013) — 성공 여부
  Wilcoxon 부호순위 + Hodges–Lehmann 추정·구간 — 주행 시간·CTE·jerk 같은 연속 지표
  Connor(1987) 표본수, 정확 mid-p 검정력
"""

import math

import numpy as np
from scipy import stats


def mcnemar_midp(n10, n01, alternative='two-sided'):
    """
    짝지은 McNemar mid-p — 불일치 쌍 n10(시험만 성공)·n01(기준만 성공) 의 조건부 이항 검정.

    alternative: 'two-sided' | 'greater'(시험이 더 자주 성공) | 'less'.
    """
    m = n10 + n01
    if m == 0:
        return 1.0
    b = stats.binom(m, 0.5)
    upper = b.sf(n10) + 0.5 * b.pmf(n10)       # P(X > n10) + ½P(X = n10)
    lower = b.cdf(n10) - 0.5 * b.pmf(n10)      # P(X < n10) + ½P(X = n10)
    if alternative == 'greater':
        return float(upper)
    if alternative == 'less':
        return float(lower)
    return float(min(1.0, 2.0 * min(upper, lower)))


def mcnemar_from_outcomes(success_t, success_r, alternative='two-sided'):
    """같은 시드 쌍의 성공 0/1 배열 → dict(n10, n01, p_value, diff)."""
    a = np.asarray(success_t).astype(bool)
    b = np.asarray(success_r).astype(bool)
    n10, n01 = int((a & ~b).sum()), int((~a & b).sum())
    return {'n10': n10, 'n01': n01, 'n_pairs': int(a.size),
            'diff': (n10 - n01) / a.size if a.size else float('nan'),
            'p_value': mcnemar_midp(n10, n01, alternative)}


def connor_n(p10, p01, alpha=0.05, power=0.8):
    """Connor(1987) 짝지은 표본수 (양측 α). p10·p01 = 불일치 확률."""
    psi, d = p10 + p01, p10 - p01
    za, zb = stats.norm.ppf(1 - alpha / 2), stats.norm.ppf(power)
    return (za * math.sqrt(psi) + zb * math.sqrt(psi - d * d)) ** 2 / (d * d)


def mcnemar_exact_power(n, p10, p01, alpha=0.05, mid=True):
    """쌍 n 개에서 양측 McNemar (mid-p 또는 정확 조건부) 의 정확 검정력 (calc4 와 같은 열거)."""
    psi = p10 + p01
    q = p10 / psi
    pw = 0.0
    for m in range(n + 1):
        wm = stats.binom.pmf(m, n, psi)
        if wm < 1e-13:
            continue
        b = np.arange(m + 1)
        lo = stats.binom.cdf(b, m, 0.5)
        hi = stats.binom.sf(b - 1, m, 0.5)
        pm = stats.binom.pmf(b, m, 0.5)
        if mid:
            p = 2 * np.minimum(lo - 0.5 * pm, hi - 0.5 * pm)
        else:
            p = np.minimum(1, 2 * np.minimum(lo, hi))
        pw += wm * np.sum(stats.binom.pmf(b, m, q)[p <= alpha])
    return float(pw)


def mcnemar_exact_n(p10, p01, alpha=0.05, power=0.8, start=20, step=10, n_max=5000):
    """정확 mid-p 검정력이 power 이상인 최소 쌍 수 (격자 step)."""
    for n in range(start, n_max + 1, step):
        if mcnemar_exact_power(n, p10, p01, alpha) >= power:
            return n
    return None


def hodges_lehmann(diff, conf=0.95):
    """
    짝 차이의 Hodges–Lehmann 추정 (Walsh 평균 중앙값) 과 부호순위 기반 구간.

    구간: Walsh 평균의 순서통계량 (K, M − K + 1), K 는 부호순위 정규 근사 임계 (n ≥ 6 권장).
    """
    d = np.asarray(diff, dtype=float)
    d = d[~np.isnan(d)]
    n = d.size
    if n == 0:
        return float('nan'), float('nan'), float('nan')
    i, j = np.triu_indices(n)
    walsh = np.sort((d[i] + d[j]) / 2.0)
    est = float(np.median(walsh))
    m = walsh.size
    z = stats.norm.ppf(1 - (1 - conf) / 2)
    k = int(math.floor(n * (n + 1) / 4.0 - z * math.sqrt(n * (n + 1) * (2 * n + 1) / 24.0)))
    if k < 1:
        return est, float(walsh[0]), float(walsh[-1])
    return est, float(walsh[k - 1]), float(walsh[m - k])


def wilcoxon_paired(x_t, x_r, alternative='two-sided', conf=0.95):
    """
    Wilcoxon 부호순위 (차이 = 시험 − 기준) + Hodges–Lehmann.

    둘 중 하나라도 NaN 인 쌍은 뺀다 (실패 에피소드의 주행 시간처럼 정의되지 않는 값).
    """
    a = np.asarray(x_t, dtype=float)
    b = np.asarray(x_r, dtype=float)
    ok = ~(np.isnan(a) | np.isnan(b))
    d = a[ok] - b[ok]
    out = {'n_pairs': int(ok.sum()), 'median_diff': float(np.median(d)) if d.size else math.nan}
    if d.size == 0 or np.all(d == 0):
        out.update(p_value=1.0, hl=0.0 if d.size else math.nan, hl_lo=math.nan, hl_hi=math.nan)
        return out
    res = stats.wilcoxon(d, alternative=alternative, zero_method='wilcox')
    hl, lo, hi = hodges_lehmann(d, conf)
    out.update(statistic=float(res.statistic), p_value=float(res.pvalue), hl=hl, hl_lo=lo,
               hl_hi=hi)
    return out
