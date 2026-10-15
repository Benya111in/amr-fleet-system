"""
P 단계 짝 비교 통계 (rclpy·scipy 비의존, numpy 만) — 보너스 2 E2–E5 사전 등록 분석이 쓴다.

정의 (docs/research/slam3d/notebook 의 b2-eval 사전 등록 항목과 같다)
- 짝: 같은 bag·같은 재생 번호의 두 행 값 (A − B). bag 별 차 d_b = 그 bag 의 짝 차 평균.
- `wilcoxon_exact`: d_b 의 부호순위 정확 검정 (양측, 모든 부호 뒤집기 2^n 열거, 0 은 버린다 — Wilcoxon 규칙,
  같은 절댓값은 평균 순위). n ≤ 20.
- `min_p_wilcoxon(n)`: 그 n 에서 낼 수 있는 가장 작은 양측 p (= 2 / 2^n) — 기각 가능한 설계인지 먼저 본다.
- `cluster_bootstrap`: bag 을 복원 추출하고, 뽑힌 bag 안에서 짝을 다시 복원 추출하는 두 단계 부트스트랩의
  평균 차 백분위 95 % 구간 (재생 간·bag 간 변동을 둘 다 담는다).
- `holm`: Holm–Bonferroni 보정 p.
- `wilson`: 실패율 Wilson 95 % 구간.
- `variance_components`: bag 안 재생 간 sd (합동) 과 bag 평균들의 sd.
- `verdict`: 사전 등록 판정 — 보정 p < α 이고 구간이 0 을 포함하지 않고 |평균 차| ≥ δ 이면 'A<B' / 'A>B',
  구간이 (−δ, δ) 안이면 'equiv' (차이가 δ 이하로 검출되지 않음), 그 밖은 'undecided' (가르지 못함).
"""

from __future__ import annotations

import itertools
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Pairs = Dict[str, List[Tuple[float, float]]]   # bag → [(A, B), ...] (재생 번호 순)


def _ranks(a: np.ndarray) -> np.ndarray:
    """평균 순위 (1 부터, 같은 값은 평균)."""
    order = np.argsort(a, kind='mergesort')
    r = np.empty(len(a), dtype=float)
    sa = a[order]
    i = 0
    while i < len(a):
        j = i
        while j + 1 < len(a) and sa[j + 1] == sa[i]:
            j += 1
        r[order[i:j + 1]] = 0.5 * (i + j) + 1.0
        i = j + 1
    return r


def wilcoxon_exact(d: Sequence[float]) -> Dict[str, float]:
    """
    부호순위 정확 검정 (양측). 0 차는 버린다. 반환: n(0 뺀 수), w_plus, p.

    p = P(|W+ − E| ≥ |w+ − E|) — 2^n 부호 뒤집기 열거 (같은 절댓값은 평균 순위라 이산 분포가 그대로 쓰인다).
    n = 0 이면 p = 1.
    """
    x = np.asarray([v for v in d if v is not None and math.isfinite(v)], dtype=float)
    x = x[x != 0.0]
    n = len(x)
    if n == 0:
        return {'n': 0, 'w_plus': 0.0, 'p': 1.0}
    if n > 20:
        raise ValueError('정확 검정은 n ≤ 20 만')
    r = _ranks(np.abs(x))
    w = float(r[x > 0].sum())
    e = float(r.sum()) / 2.0
    obs = abs(w - e)
    cnt = 0
    for signs in itertools.product((0, 1), repeat=n):
        s = float(np.dot(r, signs))
        if abs(s - e) >= obs - 1e-9:
            cnt += 1
    return {'n': n, 'w_plus': w, 'p': cnt / 2.0 ** n}


def min_p_wilcoxon(n: int) -> float:
    """0 아닌 차 n 개로 낼 수 있는 가장 작은 양측 p — 모두 같은 부호일 때 2 / 2^n."""
    return 1.0 if n <= 0 else min(1.0, 2.0 / 2.0 ** n)


def holm(ps: Sequence[float]) -> List[float]:
    """Holm–Bonferroni 보정 p (입력 순서 그대로 반환, 단조 보정, 1 로 자름)."""
    m = len(ps)
    order = sorted(range(m), key=lambda i: ps[i])
    adj = [0.0] * m
    run = 0.0
    for rank, i in enumerate(order):
        run = max(run, (m - rank) * ps[i])
        adj[i] = min(1.0, run)
    return adj


def wilson(k: int, n: int, z: float = 1.959963984540054) -> Tuple[float, float]:
    """Wilson 점수 구간 (k/n). n = 0 이면 (0, 1)."""
    if n <= 0:
        return 0.0, 1.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def bag_diffs(pairs: Pairs) -> Dict[str, float]:
    """짝 차의 bag 별 평균 {bag: 평균} (유한한 짝만; 없으면 NaN)."""
    out = {}
    for b, ps in pairs.items():
        d = [a - c for a, c in ps if math.isfinite(a) and math.isfinite(c)]
        out[b] = float(np.mean(d)) if d else math.nan
    return out


def cluster_bootstrap(pairs: Pairs, n_boot: int = 10000, seed: int = 20261008,
                      level: float = 0.95) -> Dict[str, float]:
    """두 단계(bag → 짝) 부트스트랩: 평균 차(= bag 별 평균의 평균)의 백분위 구간."""
    bags = [b for b, ps in pairs.items()
            if any(math.isfinite(a) and math.isfinite(c) for a, c in ps)]
    if not bags:
        return {'mean': math.nan, 'lo': math.nan, 'hi': math.nan, 'n_bags': 0}
    diffs = [np.array([a - c for a, c in pairs[b] if math.isfinite(a) and math.isfinite(c)])
             for b in bags]
    rng = np.random.default_rng(seed)
    est = float(np.mean([d.mean() for d in diffs]))
    nb = len(bags)
    stats = np.empty(n_boot)
    for i in range(n_boot):
        pick = rng.integers(0, nb, nb)
        vals = []
        for j in pick:
            d = diffs[j]
            vals.append(d[rng.integers(0, len(d), len(d))].mean())
        stats[i] = np.mean(vals)
    a = (1 - level) / 2
    return {'mean': est, 'lo': float(np.quantile(stats, a)),
            'hi': float(np.quantile(stats, 1 - a)), 'n_bags': nb}


def variance_components(values: Dict[str, Sequence[float]]) -> Dict[str, float]:
    """
    재생 간·bag 간 분산 성분.

    입력 {bag: 재생 값들}. 반환: within_sd (bag 안 재생 간 합동 표본 sd), between_sd (bag 평균들의 표본 sd),
    n_bags, n_values. 유한값만 쓴다.
    """
    ss, dof, means, nv = 0.0, 0, [], 0
    for v in values.values():
        x = np.asarray([u for u in v if u is not None and math.isfinite(u)], dtype=float)
        if len(x) == 0:
            continue
        nv += len(x)
        means.append(float(x.mean()))
        if len(x) > 1:
            ss += float(((x - x.mean()) ** 2).sum())
            dof += len(x) - 1
    within = math.sqrt(ss / dof) if dof else math.nan
    between = float(np.std(means, ddof=1)) if len(means) > 1 else math.nan
    return {'within_sd': within, 'between_sd': between, 'n_bags': len(means), 'n_values': nv}


def verdict(p_adj: Optional[float], lo: float, hi: float, mean: float, delta: float,
            alpha: float = 0.05) -> str:
    """사전 등록 판정 ('A<B' | 'A>B' | 'equiv' | 'undecided'). 차 = A − B."""
    if not all(math.isfinite(v) for v in (lo, hi, mean)):
        return 'undecided'
    if (p_adj is not None and p_adj < alpha and (hi < 0 or lo > 0)
            and abs(mean) >= delta):
        return 'A<B' if mean < 0 else 'A>B'
    if -delta < lo and hi < delta:
        return 'equiv'
    return 'undecided'


def replay_rule(a: Sequence[float], b: Sequence[float], delta: float) -> str:
    """
    한 bag 안 재생 비교 (E4·E5 기술 판정).

    평균 차가 2 × 합동 재생 sd 와 δ 를 둘 다 넘으면 'differs', 아니면 'undecided'.
    재생 2 개 미만이면 sd 를 0 으로 본다 (δ 만 쓴다).
    """
    x = np.asarray([v for v in a if math.isfinite(v)], dtype=float)
    y = np.asarray([v for v in b if math.isfinite(v)], dtype=float)
    if len(x) == 0 or len(y) == 0:
        return 'undecided'
    sx = float(np.std(x, ddof=1)) if len(x) > 1 else 0.0
    sy = float(np.std(y, ddof=1)) if len(y) > 1 else 0.0
    sp = math.sqrt((sx * sx + sy * sy) / 2.0)
    diff = float(x.mean() - y.mean())
    return 'differs' if abs(diff) > max(2.0 * sp, delta) else 'undecided'
