"""
쌍비교 통계 — 셀별 효과크기 · 95 % CI · 같은 방향 쌍 수 k/n, A/A 산포, rule of three.

- 분석 단위는 실행이다 (실행 안의 작업을 독립 표본으로 세지 않는다 — Hurlbert 1984).
- 쌍 = 같은 seed (같은 외생 입력). 쌍차 d_i = B_i − A_i.
- 보고: 평균 쌍차, t 기반 95 % CI, d_z = 평균/표준편차 (표준화 쌍차 효과), k+/k−/k0, Wilcoxon 부호순위 p(보조).
- A/A: 같은 비교군·셀에서 rep 0 대 rep 1 (작업 열·고장 같고 통신 난수·실행 잡음만 다름). 쌍차 SD 로
  최소 검출 효과 MDE = (t_{0.975,n−1} + t_{0.8,n−1})·SD/√n 를 내고, 두 반복의 상관 ρ 와 사건열 발산 시각을 잰다.
- 0 건 지표: n 실행 모두 0 이면 "실행당 1 건 이상 생길 확률" 의 95 % 상한 = 1 − 0.05^{1/n} (≈ 3/n, rule of three).
"""

from __future__ import annotations

import json
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from scipy import stats


def load_runs(paths: Iterable[str]) -> List[dict]:
    """runs.jsonl 여러 개를 읽는다."""
    out: List[dict] = []
    for p in paths:
        with open(p, encoding='utf-8') as f:
            out.extend(json.loads(line) for line in f if line.strip())
    return out


def metric(r: dict, name: str) -> Optional[float]:
    """실행 결과에서 지표 하나 (점 표기로 하위 키: 'vote_zone.abandons')."""
    v = r
    for part in name.split('.'):
        if not isinstance(v, dict) or part not in v:
            return None
        v = v[part]
    return None if v is None else float(v)


def paired(a: Sequence[float], b: Sequence[float]) -> dict:
    """쌍차 B − A 요약."""
    if len(a) != len(b):
        raise ValueError('쌍의 길이가 다르다')
    d = [y - x for x, y in zip(a, b)]
    n = len(d)
    out = {'n': n, 'mean_a': _mean(a), 'mean_b': _mean(b)}
    if n == 0:
        return out
    m = _mean(d)
    sd = math.sqrt(sum((x - m) ** 2 for x in d) / (n - 1)) if n > 1 else 0.0
    half = stats.t.ppf(0.975, n - 1) * sd / math.sqrt(n) if n > 1 else math.inf
    if sd > 0:
        dz = m / sd
    else:
        dz = 0.0 if m == 0 else math.copysign(math.inf, m)
    nz = [x for x in d if x != 0]
    p = 1.0
    if len(nz) >= 1:
        try:
            p = float(stats.wilcoxon(nz, zero_method='wilcox', alternative='two-sided').pvalue)
        except ValueError:
            p = 1.0
    out.update({
        'mean_diff': m, 'sd_diff': sd, 'ci95': (m - half, m + half), 'dz': dz,
        'k_pos': sum(1 for x in d if x > 0), 'k_neg': sum(1 for x in d if x < 0),
        'k_zero': sum(1 for x in d if x == 0), 'wilcoxon_p': p,
    })
    return out


def mde(sd: float, n: int, alpha: float = 0.05, power: float = 0.8) -> float:
    """쌍 t 검정 최소 검출 효과 (원 단위)."""
    if n < 2:
        return math.inf
    return (stats.t.ppf(1 - alpha / 2, n - 1) + stats.t.ppf(power, n - 1)) * sd / math.sqrt(n)


def zero_upper(n: int, conf: float = 0.95) -> float:
    """모두 0 건인 n 회에서 '실행당 1 건 이상' 확률의 단측 95 % 상한 (Clopper–Pearson, ≈ 3/n)."""
    if n <= 0:
        return 1.0
    return 1.0 - (1.0 - conf) ** (1.0 / n)


def pearson(a: Sequence[float], b: Sequence[float]) -> Optional[float]:
    """상관 (분산 0 이면 None)."""
    if len(a) < 3:
        return None
    ma, mb = _mean(a), _mean(b)
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((y - mb) ** 2 for y in b))
    if sa == 0 or sb == 0:
        return None
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def divergence(t0: Sequence[Sequence], t1: Sequence[Sequence]) -> Optional[float]:
    """두 사건열((시각, 작업, 로봇) 시간순)이 처음 갈리는 시각 (끝까지 같으면 None)."""
    for x, y in zip(t0, t1):
        if tuple(x[1:]) != tuple(y[1:]):
            return min(float(x[0]), float(y[0]))
    if len(t0) != len(t1):
        k = min(len(t0), len(t1))
        rest = (t0[k:] or t1[k:])
        return float(rest[0][0])
    return None


def index(rows: Iterable[dict]) -> Dict[Tuple, dict]:
    """(arm, loss, fault, seed, rep) → 실행."""
    return {(r['arm'], float(r['loss']), r['fault'], int(r['seed']), int(r['rep'])): r
            for r in rows}


def compare(idx: Dict[Tuple, dict], arm_a: str, arm_b: str, loss: float, fault: str,
            name: str, fault_b: Optional[str] = None, rep: int = 0) -> dict:
    """같은 seed 쌍으로 B − A (fault_b 를 주면 같은 비교군의 고장 대조 등)."""
    fb = fault if fault_b is None else fault_b
    seeds = sorted(s for (a, lo, f, s, r) in idx if a == arm_a and lo == loss and f == fault
                   and r == rep and (arm_b, loss, fb, s, rep) in idx)
    xa, xb = [], []
    for s in seeds:
        va = metric(idx[(arm_a, loss, fault, s, rep)], name)
        vb = metric(idx[(arm_b, loss, fb, s, rep)], name)
        if va is None or vb is None:
            continue
        xa.append(va)
        xb.append(vb)
    out = paired(xa, xb)
    out.update({'a': arm_a, 'b': arm_b, 'loss': loss, 'fault': fault, 'fault_b': fb,
                'metric': name})
    return out


def holm(ps: Sequence[float]) -> List[float]:
    """Holm 보정 p 값 (입력 순서대로)."""
    order = sorted(range(len(ps)), key=lambda i: ps[i])
    adj = [0.0] * len(ps)
    run = 0.0
    m = len(ps)
    for k, i in enumerate(order):
        run = max(run, min(1.0, (m - k) * ps[i]))
        adj[i] = run
    return adj


def fmt(x: Optional[float], nd: int = 2) -> str:
    """표 칸 문자열."""
    if x is None:
        return '-'
    if isinstance(x, float) and math.isinf(x):
        return '∞' if x > 0 else '-∞'
    return f'{x:.{nd}f}'


def _mean(xs: Sequence[float]) -> Optional[float]:
    return sum(xs) / len(xs) if xs else None


__all__ = ['compare', 'divergence', 'fmt', 'holm', 'index', 'load_runs', 'mde', 'metric',
           'paired', 'pearson', 'zero_upper']
