"""
다중 비교 — Holm step-down 과 고정 순서(fixed-sequence) 검정.

근거: 이론편 「권장 평가 계획」 (주 비교군 C 는 고정 순서, B-PPO·B-SAC 는 Holm 또는 탐색),
evaluation_statistics_theory.md §3 (d) — Holm(1979), Dmitrienko & Tamhane(2007).
"""

import numpy as np


def holm(p_values, alpha=0.05):
    """
    Holm step-down.

    반환: (기각 여부 배열, 보정 p 배열) — 입력 순서 그대로.
    보정 p_(i) = max_{j ≤ i} min(1, (m − j + 1) p_(j)).
    """
    p = np.asarray(p_values, dtype=float)
    m = p.size
    order = np.argsort(p, kind='stable')
    adj_sorted = np.minimum(1.0, np.maximum.accumulate((m - np.arange(m)) * p[order]))
    adj = np.empty(m)
    adj[order] = adj_sorted
    return adj <= alpha, adj


def fixed_sequence(p_values, alpha=0.025):
    """
    고정 순서 검정 — 미리 정한 순서대로 각 가설을 α 그대로 검정하고 첫 비기각에서 멈춘다.

    반환: 기각 여부 리스트 (멈춘 뒤의 가설은 '검정하지 않음' = False). FWER ≤ α.
    """
    out = []
    go = True
    for p in p_values:
        r = go and (p is not None) and (p <= alpha)
        out.append(bool(r))
        go = go and r
    return out


def holm_alpha(rank, m, alpha=0.025):
    """Holm 에서 rank 번째(1부터) 로 작은 p 에 쓰이는 수준 α/(m − rank + 1)."""
    return alpha / (m - rank + 1)
