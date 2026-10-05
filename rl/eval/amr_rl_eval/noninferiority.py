"""
두 비율 비교 — 접촉률 비열등성(NI)·우월성 검정과 표본수.

표기: t = 시험 정책(학습 비교군 B·C), r = 기준(DWA, 비교군 A). 접촉률은 높을수록 나쁘다.
  NI 귀무가설  H0: p_t − p_r ≥ δ   (시험 정책이 δ 이상 더 자주 부딪친다)
  NI 대립가설  H1: p_t − p_r < δ

근거: 이론편 「DWA가 0회이면 비열등성 표본수에 DWA 표본은 나오지 않는다」 절,
evaluation_statistics_theory.md §2 (calc2.py·calc3.py 와 같은 공식).

  Farrington–Manning(1990) score 검정 — 제약 MLE 는 Miettinen–Nurminen 3 차식의 닫힌 해
  Newcombe(1998) hybrid score 상한 — 각 군 Wilson 한계로 만든다
  Tango(1998) 짝지은 score 검정 — 같은 시나리오 시드에서 두 정책을 돌린 쌍
  우월성(시험 정책이 더 자주 부딪친다): Fisher·mid-p·Boschloo (scipy)

**주의 — Boschloo 표 방향.** scipy 의 boschloo_exact·barnard_exact 는 **열**을 이항 표본(군)으로
본다. 이론편 calc1.py 는 행을 군으로 넣어 (`[[k, 240−k], [0, 240]]`) 다른 설계(총 사건 수 고정)의
값을 냈다. 240 대 240 에서 0 대 4 의 올바른 Boschloo 단측 p 는 0.0550 이고 (calc1: 0.043),
유의(α=0.05)는 0 대 5 (p=0.0254) 부터다 — tests/test_noninferiority.py 가 전수 계산으로 확인한다.
"""

import math

import numpy as np
from scipy import optimize, stats

from . import binomial

EPS = 1e-12


def z_quantile(p):
    """표준정규 분위수."""
    return float(stats.norm.ppf(p))


# ---------------------------------------------------------------- 비짝지은 (독립 두 군)

def fm_restricted(x_t, n_t, x_r, n_r, delta):
    """
    p_t − p_r = δ 제약 아래 최대우도 (p̃_t, p̃_r).

    Farrington–Manning(1990) 의 3 차식 닫힌 해 (Miettinen–Nurminen 1985 와 같은 식).
    수치가 불안정하면 (판별식 경계) 유계 1 차원 최적화로 대신한다.
    """
    p1, p2 = x_t / n_t, x_r / n_r
    theta = n_r / n_t
    d = delta
    a = 1.0 + theta
    b = -(1.0 + theta + p1 + theta * p2 + d * (theta + 2.0))
    c = d * d + d * (2.0 * p1 + theta + 1.0) + p1 + theta * p2
    e = -p1 * d * (1.0 + d)
    v = b ** 3 / (3.0 * a) ** 3 - b * c / (6.0 * a * a) + e / (2.0 * a)
    u2 = b * b / (3.0 * a) ** 2 - c / (3.0 * a)
    pt = float('nan')
    if u2 > 0:
        u = math.copysign(math.sqrt(u2), v) if v != 0 else math.sqrt(u2)
        ratio = v / u ** 3
        if -1.0 <= ratio <= 1.0:
            w = (math.pi + math.acos(ratio)) / 3.0
            pt = 2.0 * u * math.cos(w) - b / (3.0 * a)
    lo, hi = max(0.0, -d), min(1.0, 1.0 - d)
    if not (lo - 1e-9 <= pt - d <= hi + 1e-9) or math.isnan(pt):
        return _fm_restricted_numeric(x_t, n_t, x_r, n_r, d)
    pr = min(max(pt - d, 0.0), 1.0)
    return min(max(pr + d, 0.0), 1.0), pr


def _fm_restricted_numeric(x_t, n_t, x_r, n_r, d):
    lo, hi = max(EPS, -d + EPS), min(1.0 - EPS, 1.0 - d - EPS)

    def nll(pr):
        pt = pr + d
        return -(x_t * math.log(pt) + (n_t - x_t) * math.log(1 - pt)
                 + x_r * math.log(pr) + (n_r - x_r) * math.log(1 - pr))

    r = optimize.minimize_scalar(nll, bounds=(lo, hi), method='bounded',
                                 options={'xatol': 1e-13})
    return r.x + d, r.x


def fm_z(x_t, n_t, x_r, n_r, delta):
    """Farrington–Manning score 통계량. NI 는 z < −z_{1−α} 일 때 선언."""
    pt, pr = fm_restricted(x_t, n_t, x_r, n_r, delta)
    se = math.sqrt(pt * (1 - pt) / n_t + pr * (1 - pr) / n_r)
    if se == 0:
        return -math.inf if x_t / n_t - x_r / n_r < delta else math.inf
    return (x_t / n_t - x_r / n_r - delta) / se


def newcombe_upper(x_t, n_t, x_r, n_r, conf=0.975):
    """
    Newcombe hybrid score — p_t − p_r 의 상한 (단측 신뢰수준 conf).

    U = (p̂_t − p̂_r) + √((u_t − p̂_t)² + (p̂_r − l_r)²), (l, u) = Wilson 한계 (z = z_conf).
    NI 는 U < δ 일 때 선언. x_r = 0 이면 U = u_t (시험 정책 Wilson 상한) — DWA 표본수가 빠진다.
    """
    z = z_quantile(conf)
    pt, pr = x_t / n_t, x_r / n_r
    _, ut = binomial.wilson_interval(x_t, n_t, z)
    lr, _ = binomial.wilson_interval(x_r, n_r, z)
    return (pt - pr) + math.sqrt((ut - pt) ** 2 + (pr - lr) ** 2)


def ni_test(x_t, n_t, x_r, n_r, delta, alpha=0.025):
    """
    접촉률 NI 판정 — 주 분석 FM 검정과 Newcombe 상한을 함께 낸다 (PROTOCOL.md §6).

    반환 dict: z, p_value(FM 단측), newcombe_upper, ni_fm, ni_newcombe, ni(둘 다 통과).
    """
    z = fm_z(x_t, n_t, x_r, n_r, delta)
    u = newcombe_upper(x_t, n_t, x_r, n_r, 1.0 - alpha)
    zc = z_quantile(1.0 - alpha)
    return {
        'z': z, 'p_value': float(stats.norm.cdf(z)), 'newcombe_upper': u,
        'ni_fm': bool(z < -zc), 'ni_newcombe': bool(u < delta),
        'ni': bool(z < -zc and u < delta),
    }


def ni_n_zero_events(delta, alpha=0.025):
    """
    양측 0 사건일 때 NI 를 선언하는 최소 시험 정책 표본수 — n_t > z²(1 − δ)/δ.

    FM 제약 MLE 가 (p̃_t, p̃_r) = (δ, 0) 이 되어 n_r 이 식에서 사라진다 (이론편, FM 1990).
    """
    z = z_quantile(1.0 - alpha)
    return int(math.floor(z * z * (1.0 - delta) / delta)) + 1


def ni_n_given_events(k, delta, alpha=0.025, n_r=None, method='newcombe', n_max=100000):
    """
    시험 정책에서 k 회 접촉, 기준 0 회일 때 NI 를 선언하는 최소 n_t.

    method='newcombe' 는 x_r = 0 이면 n_r 과 무관 (Wilson 상한 < δ). 'fm' 은 n_r 이 필요하다.
    """
    n = max(k + 1, 2)
    while n <= n_max:
        if method == 'newcombe':
            ok = binomial.wilson_upper(k, n, 1.0 - alpha) < delta
        else:
            ok = fm_z(k, n, 0, n_r, delta) < -z_quantile(1.0 - alpha)
        if ok:
            return n
        n += 1
    raise ValueError('n_max 안에서 찾지 못함')


def fm_n_formula(p_t, p_r, delta, alpha=0.025, power=0.8):
    """
    Farrington–Manning 근사 표본수 (군당, 같은 n).

    n = [z_{1−α}√(p̃_t q̃_t + p̃_r q̃_r) + z_{power}√(p_t q_t + p_r q_r)]² / (p_t − p_r − δ)²,
    p̃ 는 모집단 값에서의 제약 MLE. 이산성 때문에 정확 계산보다 3~34 % 크게 나올 수 있다.
    """
    def nll(q2):
        q1 = q2 + delta
        val = p_t * math.log(q1) + (1 - p_t) * math.log(1 - q1) + (1 - p_r) * math.log(1 - q2)
        if p_r > 0:
            val += p_r * math.log(q2)
        return -val

    r = optimize.minimize_scalar(nll, bounds=(EPS, 1 - delta - EPS), method='bounded',
                                 options={'xatol': 1e-12})
    q2 = r.x
    q1 = q2 + delta
    num = (z_quantile(1 - alpha) * math.sqrt(q1 * (1 - q1) + q2 * (1 - q2))
           + z_quantile(power) * math.sqrt(p_t * (1 - p_t) + p_r * (1 - p_r))) ** 2
    return num / (p_t - p_r - delta) ** 2


def ni_exact_power(n_t, n_r, p_t, p_r, delta, alpha=0.025, method='fm'):
    """
    비짝지은 NI 의 정확 검정력 — 두 이항분포를 전수 열거 (calc2.exact_power 와 같은 방법).

    각 x_r 에서 기각하는 x_t 는 0 부터 단조이므로 최대 기각 x_t 까지 누적한다.
    """
    zc = z_quantile(1 - alpha)
    x_t_max = int(stats.binom.isf(1e-9, n_t, max(p_t, 1e-9))) + 2
    x_r_max = int(stats.binom.isf(1e-9, n_r, max(p_r, 1e-9))) + 2
    pw = 0.0
    for x_r in range(0, min(x_r_max, n_r) + 1):
        w = stats.binom.pmf(x_r, n_r, p_r)
        if w < 1e-14:
            continue
        cmax = -1
        for x_t in range(0, min(x_t_max + 40, n_t) + 1):
            if method == 'fm':
                rej = fm_z(x_t, n_t, x_r, n_r, delta) < -zc
            else:
                rej = newcombe_upper(x_t, n_t, x_r, n_r, 1 - alpha) < delta
            if rej:
                cmax = x_t
            else:
                break
        if cmax >= 0:
            pw += w * stats.binom.cdf(cmax, n_t, p_t)
    return pw


# ---------------------------------------------------------------- 짝지은 (같은 시나리오 시드)

def tango_restricted(n10, n01, n, delta):
    """
    p10 − p01 = δ 제약 MLE (Tango 1998 닫힌 해).

    n10 = 시험 정책만 접촉한 쌍, n01 = 기준만 접촉한 쌍, n = 전체 쌍.
    2n q² − [n10 + n01 − (2n − n10 + n01)δ] q − n01 δ (1 − δ) = 0 의 양의 근 q = p̃01.
    """
    a = 2.0 * n
    b = -(n10 + n01) + (2.0 * n - n10 + n01) * delta
    c = -n01 * delta * (1.0 - delta)
    q = (-b + math.sqrt(max(b * b - 4 * a * c, 0.0))) / (2 * a)
    return q + delta, q


def tango_z(n10, n01, n, delta):
    """Tango 짝지은 score 통계량. NI 는 z < −z_{1−α} 일 때."""
    q10, q01 = tango_restricted(n10, n01, n, delta)
    var = (q10 + q01 - delta * delta) / n
    if var <= 0:
        return -math.inf if (n10 - n01) / n < delta else math.inf
    return ((n10 - n01) / n - delta) / math.sqrt(var)


def tango_exact_power(n, p10, p01, delta, alpha=0.025):
    """
    짝지은 NI(Tango) 의 정확 검정력 — n01 을 이항으로, n10 | n01 을 조건부 이항으로 열거 (calc3).

    p10 = P(시험만 접촉), p01 = P(기준만 접촉).
    """
    zc = z_quantile(1 - alpha)
    pw = 0.0
    c_max = int(stats.binom.isf(1e-10, n, max(p01, 1e-9))) + 2
    q = p10 / (1 - p01) if p01 < 1 else 0.0
    for c in range(0, min(c_max, n) + 1):
        wc = stats.binom.pmf(c, n, p01)
        if wc < 1e-14:
            continue
        bmax = -1
        for b in range(0, n - c + 1):
            if tango_z(b, c, n, delta) < -zc:
                bmax = b
            else:
                break
        if bmax >= 0:
            pw += wc * stats.binom.cdf(bmax, n - c, q)
    return pw


def paired_table(fail_t, fail_r):
    """같은 시드 쌍의 0/1 배열 → (n11, n10, n01, n00). 1 = 사건(접촉)."""
    a = np.asarray(fail_t).astype(bool)
    b = np.asarray(fail_r).astype(bool)
    if a.shape != b.shape:
        raise ValueError('쌍의 길이가 다르다')
    return (int((a & b).sum()), int((a & ~b).sum()), int((~a & b).sum()), int((~a & ~b).sum()))


# ---------------------------------------------------------------- 우월성 (시험 정책이 더 위험한가)

def _arms_table(k_t, n_t, k_r, n_r):
    # scipy 2×2 정확 검정은 **열이 군**이다: [[군1 사건, 군2 사건], [군1 무사건, 군2 무사건]]
    return np.array([[k_t, k_r], [n_t - k_t, n_r - k_r]])


def fisher_greater(k_t, n_t, k_r, n_r):
    """Fisher 단측 p — 시험 정책의 사건률이 더 크다."""
    return float(stats.fisher_exact(_arms_table(k_t, n_t, k_r, n_r),
                                    alternative='greater').pvalue)


def fisher_midp_greater(k_t, n_t, k_r, n_r):
    """Fisher mid-p 단측 — P(X > k_t) + ½P(X = k_t), X ~ 초기하(총 사건 고정)."""
    hg = stats.hypergeom(n_t + n_r, k_t + k_r, n_t)
    return float(hg.sf(k_t) + 0.5 * hg.pmf(k_t))


def boschloo_greater(k_t, n_t, k_r, n_r, n=32):
    """Boschloo 정확 비조건부 단측 p — 시험 정책의 사건률이 더 크다 (군 = 열)."""
    return float(stats.boschloo_exact(_arms_table(k_t, n_t, k_r, n_r),
                                      alternative='greater', n=n).pvalue)


def superiority_alarm_k(n_t, n_r, k_r=0, alpha=0.05, k_max=None):
    """기준 k_r 회일 때 Boschloo 단측 p < α 가 되는 시험 정책 최소 사건 수 (안전 경보 문턱)."""
    k_max = n_t if k_max is None else k_max
    for k in range(k_r, k_max + 1):
        if boschloo_greater(k, n_t, k_r, n_r) < alpha:
            return k
    return None
