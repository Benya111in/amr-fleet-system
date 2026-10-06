"""
단일 이항 비율 추론 — 희귀 사건(접촉 0 회)의 상한과 무사고 실증 표본수.

근거: 이론편 「0/240은 seed 42 actor 구성 하나에 대한 진술이다」 절,
research_notes/.../evaluation_statistics_theory.md §1 (calc1.py 와 같은 공식).

  Clopper–Pearson 단측 상한   x = 0 이면 1 − α^(1/n)  (Clopper & Pearson 1934)
  Wilson score               x = 0 이면 z² / (n + z²)  (Wilson 1927)
  Jeffreys                   Beta(x + ½, n − x + ½) 분위수 (Brown, Cai, DasGupta 2001)
  rule of three              3 / n  (Hanley & Lippman-Hand 1983)

안전 주장은 보수적인 Clopper–Pearson 을 주 지표로 쓰고 나머지는 병기한다 (PROTOCOL.md §5).
"""

import math

import numpy as np
from scipy import stats


def _check(x, n):
    if n <= 0:
        raise ValueError(f'n 은 양수여야 한다: {n}')
    if not 0 <= x <= n:
        raise ValueError(f'0 ≤ x ≤ n 이어야 한다: x={x}, n={n}')


def cp_upper(x, n, conf=0.95):
    """Clopper–Pearson 단측 상한 (신뢰수준 conf)."""
    _check(x, n)
    if x == n:
        return 1.0
    return float(stats.beta.ppf(conf, x + 1, n - x))


def cp_lower(x, n, conf=0.95):
    """Clopper–Pearson 단측 하한 (신뢰수준 conf)."""
    _check(x, n)
    if x == 0:
        return 0.0
    return float(stats.beta.ppf(1.0 - conf, x, n - x + 1))


def cp_interval(x, n, conf=0.95):
    """Clopper–Pearson 양측 등꼬리 구간 (신뢰수준 conf)."""
    a = (1.0 - conf) / 2.0
    return cp_lower(x, n, 1.0 - a), cp_upper(x, n, 1.0 - a)


def wilson_interval(x, n, z):
    """Wilson score 구간 (z 를 직접 받는다 — 단측이면 z_{1−α}, 양측이면 z_{1−α/2})."""
    _check(x, n)
    ph = x / n
    centre = ph + z * z / (2 * n)
    half = z * math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n))
    den = 1 + z * z / n
    return (centre - half) / den, (centre + half) / den


def wilson_upper(x, n, conf=0.95):
    """Wilson score 단측 상한."""
    return wilson_interval(x, n, float(stats.norm.ppf(conf)))[1]


def jeffreys_upper(x, n, conf=0.95):
    """Jeffreys 단측 상한 — Beta(x+½, n−x+½) 의 conf 분위수."""
    _check(x, n)
    if x == n:
        return 1.0
    return float(stats.beta.ppf(conf, x + 0.5, n - x + 0.5))


def rule_of_three(n):
    """사건 0/n 의 95 % 상한 근사 3/n."""
    if n <= 0:
        raise ValueError('n 은 양수여야 한다')
    return 3.0 / n


def zero_failure_n(p0, conf=0.95):
    """
    무사고 실증 표본수 — 0 회로 '접촉률 < p0' 를 신뢰수준 conf 로 보이는 최소 n.

    n = ⌈ln(1 − conf) / ln(1 − p0)⌉ (success-run 공식).
    """
    if not 0 < p0 < 1:
        raise ValueError('0 < p0 < 1')
    return int(math.ceil(math.log(1.0 - conf) / math.log(1.0 - p0) - 1e-12))


def c_failure_n(p0, conf=0.95, c=0):
    """사건 c 회까지 허용하고 '접촉률 < p0' 를 보이는 최소 n (P[X ≤ c | n, p0] ≤ 1 − conf)."""
    n = max(1, c + 1)
    while stats.binom.cdf(c, n, p0) > 1.0 - conf:
        n += 1
    return n


def pass_probability(p, n):
    """참 접촉률 p 인 정책이 'n 회 0 충돌' 판정을 통과할 확률 (1 − p)^n — 명세 30 회 판정의 OC."""
    return (1.0 - p) ** n


def effective_n(n, cluster_size, icc):
    """군집 표본 실효 크기 n / (1 + (m − 1)ρ) (design effect, Killip 2004)."""
    return n / (1.0 + (cluster_size - 1) * icc)


def cp_upper_effective(n, cluster_size, icc, conf=0.95):
    """
    0 회 사건, 군집 상관 ρ 일 때 실효 n 을 넣은 근사 CP 상한 (plug-in 휴리스틱).

    정확한 상한이 아니다 — '반복이 독립이 아니면 상한이 얼마나 커질 수 있는가' 를 보이는 용도.
    """
    ne = effective_n(n, cluster_size, icc)
    return 1.0 - (1.0 - conf) ** (1.0 / ne)


def beta_posterior(x, n, a=1.0, b=1.0, threshold=0.01, conf=0.95):
    """
    Beta(a, b) 사전 → 사후 Beta(a + x, b + n − x) 요약.

    반환: dict(mean, upper, p_below) — p_below = P(p < threshold).
    """
    _check(x, n)
    pa, pb = a + x, b + n - x
    return {
        'mean': pa / (pa + pb),
        'upper': float(stats.beta.ppf(conf, pa, pb)),
        'p_below': float(stats.beta.cdf(threshold, pa, pb)),
    }


def rate_summary(x, n, conf=0.95):
    """
    접촉률 보고 한 줄 — 주 지표 CP 상한과 병기 지표들.

    반환 dict 의 키: x, n, rate, cp_upper, wilson_upper, jeffreys_upper, rule3 (x=0 일 때만).
    """
    out = {
        'x': int(x), 'n': int(n), 'rate': x / n,
        'cp_upper': cp_upper(x, n, conf),
        'wilson_upper': wilson_upper(x, n, conf),
        'jeffreys_upper': jeffreys_upper(x, n, conf),
    }
    out['rule3'] = rule_of_three(n) if x == 0 else float('nan')
    return out


def precision_n(p, half_width, conf=0.95, method='wald'):
    """
    성공률 p 를 ±half_width 로 추정하는 데 필요한 n (단일 정책·독립 에피소드).

    method='wald': z²p(1−p)/E², 'wilson': Wilson 반폭이 E 이하가 되는 최소 n.
    """
    z = float(stats.norm.ppf(1 - (1 - conf) / 2))
    if method == 'wald':
        return int(math.ceil(z * z * p * (1 - p) / half_width ** 2 - 1e-9))
    if method != 'wilson':
        raise ValueError(method)
    n = 2
    while True:
        hw = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
        if hw <= half_width:
            return n
        n += 1


def binom_counts(outcomes):
    """0/1 배열 → (사건 수, 시행 수)."""
    a = np.asarray(outcomes).astype(bool)
    return int(a.sum()), int(a.size)
