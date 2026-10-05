"""
평가 예산 계획 — 이론편 「희귀 사건 가속, CVaR, SRCC의 유한표본 성질」 끝의 표를 다시 계산한다.

  | 목적 | 조건 | 필요 표본 |
  | 충돌 NI δ = 2 %p, 고정 n | 양측 0 충돌, α = 0.025 | 189 |
  | 충돌 NI δ = 2 %p, SPRT   | 0 충돌 시 종료, 상한 600 | 243 (+92/충돌) |
  | 충돌 NI δ = 1 %p         | 0 충돌 | 381 (고정) · 489 (SPRT) |
  | 비교군 4 개 Holm         | 0 충돌 | 비교군당 306 |
  | 성공률 0.90→0.95 McNemar | 불일치 0.07 | 200 쌍 |
  | 2D 성공률 ±2 %p          | p = 0.9 | 865 |
  | SRCC 구간 하한 > 0.5     | ρ = 0.9 | 정책 10 개 이상 |

Gazebo 시간은 에피소드 60 s · RTF 0.3 → 에피소드당 200 s 가정 (보고서 전제). 실측 RTF 는 PROTOCOL.md §8.
"""

from . import binomial, noninferiority as ni, paired, sequential, srcc


def gazebo_hours(n_episodes, episode_s=60.0, rtf=0.3):
    """단일 인스턴스 Gazebo 벽시계 시간 [h]."""
    return n_episodes * episode_s / rtf / 3600.0


def plan_table(alpha=0.025):
    """이론편 계획 표의 각 행을 계산해 dict 리스트로 돌려준다 (rl/eval/PROTOCOL.md §7)."""
    rows = []
    n2 = ni.ni_n_zero_events(0.02, alpha)
    rows.append({'purpose': '충돌 NI δ=2%p 고정 n', 'n': n2, 'hours': gazebo_hours(n2)})
    s2 = sequential.SPRTDesign(p0=0.02, p1=0.005, alpha=alpha)
    rows.append({'purpose': '충돌 NI δ=2%p SPRT(Ville)', 'n': s2.n_zero,
                 'per_failure': s2.cost_per_failure, 'hours': gazebo_hours(s2.n_zero)})
    n1 = ni.ni_n_zero_events(0.01, alpha)
    s1 = sequential.SPRTDesign(p0=0.01, p1=0.0025, alpha=alpha)
    rows.append({'purpose': '충돌 NI δ=1%p 고정 n', 'n': n1, 'hours': gazebo_hours(n1)})
    rows.append({'purpose': '충돌 NI δ=1%p SPRT(Ville)', 'n': s1.n_zero,
                 'per_failure': s1.cost_per_failure, 'hours': gazebo_hours(s1.n_zero)})
    nh = ni.ni_n_zero_events(0.02, alpha / 4)
    rows.append({'purpose': '충돌 NI δ=2%p 비교군 4 개 Holm(α/4)', 'n': nh,
                 'hours': gazebo_hours(nh)})
    d = 0.05
    psi = 0.07
    nm = paired.mcnemar_exact_n((psi + d) / 2, (psi - d) / 2, alpha=0.05, power=0.8)
    rows.append({'purpose': '성공률 0.90→0.95 McNemar mid-p (ψ=0.07)', 'n': nm,
                 'hours': gazebo_hours(2 * nm)})
    rows.append({'purpose': '2D 성공률 ±2%p (p=0.9, Wald)', 'n': binomial.precision_n(0.9, 0.02),
                 'hours': None})
    lo10 = srcc.bonett_wright_ci(0.9, 10)[0]
    lo8 = srcc.bonett_wright_ci(0.9, 8)[0]
    rows.append({'purpose': 'SRCC ρ=0.9 BW 하한 > 0.5', 'n': 10, 'bw_lo_n10': lo10,
                 'bw_lo_n8': lo8, 'hours': None})
    return rows
