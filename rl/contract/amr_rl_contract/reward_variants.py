"""
보상 설계 후보 — 2차 물결 2a (r5-reward). 근거와 착취 표는 docs/research/rl-navigation/reward_v12.md.

계약 v1.1.0 의 문제 [계산, reward_v12.md §1]: 진행이 PBRS 로만 들어가 있어서, 끝나지 않는(정체·시간 상한
truncation 으로 부트스트랩되는) 궤적의 가치가 진행량과 무관하게 κ·d₀ − c/(1−γ) 로 같다. 정확한 critic 아래에서
PBRS 는 advantage 를 바꾸지 않으므로(「이론」 185행) 성공 사건 말고는 학습 신호가 없다.

후보 (모두 성공 조건·γ·c·G·C·평활·추종 항은 v1.1.0 그대로):

- ``pbrs_eff``  : Φ 의 거리만 max(d_arc, 목표 직선거리) 로 (PBRS 불변성 유지 — 진단용 대조).
- ``prog40``    : pbrs_eff + 기저 거리 비용 κ_b = 0.4 + 1 m 초과 이탈 힌지 0.05.
- ``prog20``    : 같은 구조, κ_b = 0.2 (민감도).
- ``prog40_st`` : prog40 + 정체를 실패 종단으로 (P_s = 20).

κ_b 상한 0.4 의 근거 [계산]: 계약 §6.1 의 사슬 MDP(돌진 0.2 % 충돌)에서 κ_b ≤ 0.4 이면 30 m 안 어느 상태도
돌진을 고르지 않는다 (0.45 는 22.6 m 너머에서 고른다). 자살 하한 C > c̄_max = (c + κ_b(1−γ)d_max)/(1−γ)
= 4 + 0.4·30 = 16 < 30.
"""

from .reward import RewardSpec, SPEC_V110

PBRS_EFF = RewardSpec(name='pbrs_eff', phi_dist='eff')
PROG40 = RewardSpec(name='prog40', phi_dist='eff', progress_kappa=0.4, dev_hinge=0.05)
PROG20 = RewardSpec(name='prog20', phi_dist='eff', progress_kappa=0.2, dev_hinge=0.05)
PROG40_ST = RewardSpec(name='prog40_st', phi_dist='eff', progress_kappa=0.4, dev_hinge=0.05,
                       stall_terminal=True, stall_penalty=20.0)

CANDIDATES = {s.name: s for s in (SPEC_V110, PBRS_EFF, PROG40, PROG20, PROG40_ST)}


def get(name):
    """이름 → RewardSpec (없으면 KeyError)."""
    return CANDIDATES[name]
