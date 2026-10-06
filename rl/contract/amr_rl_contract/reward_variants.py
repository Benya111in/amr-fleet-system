"""
보상 설계 후보 — 2차 물결 2a (r5-reward). 근거와 착취 표는 docs/research/rl-navigation/reward_v12.md.

G2 를 통과한 prog50 이 계약 v1.2.0 의 기본 보상(reward.SPEC)이 되었다 — 'v1.2.0' 과 'prog50' 은 이름만 다르고
값이 같다 (G0–G2 기록의 재현을 위해 후보 이름을 남긴다).

계약 v1.1.0 의 문제 [측정·계산, reward_v12.md §2]: 진행이 PBRS 로만 들어가 있어서, 끝나지 않는(정체·시간 상한
truncation 으로 부트스트랩되는) 궤적의 가치가 진행량과 무관하게 κ·d₀ − c/(1−γ) 이하이고, 같음은 제자리 대기
뿐이다 (움직이면 평활·추종·개입 벌점만 더해진다). 정확한 critic 아래에서 PBRS 는 advantage 를 바꾸지 않으므로
(「이론」 185행) 성공 사건 말고는 진행 쪽 학습 신호가 없다.

후보 (모두 성공 조건·γ·c·G·C·평활·추종 항·PBRS κ = 1 은 v1.1.0 그대로):

- ``pbrs_eff``  : Φ 의 거리만 max(d_arc, 목표 직선거리) 로 (PBRS 불변성 유지 — 진단용 대조).
- ``prog50``    : pbrs_eff + 기저 진행 효용 κ_b·(d − d'), κ_b = 0.5 + 1 m 초과 이탈 힌지 0.05/스텝.
- ``prog25``    : 같은 구조, κ_b = 0.25 (민감도).
- ``prog50_st`` : prog50 + 정체를 실패 종단으로 (P_s = 10).

κ_b 의 창 [계산, tests/test_reward_variants.py]: 계약 §6.1 의 사슬 MDP(돌진 0.2 % 충돌)에서 κ_b ≤ 0.8 이면 30 m
안 어느 상태도 돌진을 고르지 않는다 (1.0 = Δd 는 16.8 m 너머). 충돌과 대기의 비교는 세 가지로 갈린다:

- 결정 수준 (같은 지점까지 와서 정지 ≻ 충돌): 차이 γ^{T−1}(C − γ·c̄), κ_b 와 무관 — C > γ·c̄ 로 충분하다.
- 무할인 계약 에피소드 (진행 κ_b·d 를 모은 뒤 충돌 ≺ 20 s 정체로 끝나는 대기): C > c·200 + κ_b·d_max =
  4 + 30κ_b (κ_b 0.5 → 19 < 30).
- 할인 궤적 전체 (Knox τ_crash ≺ τ_idle): κ_b·Σ_{t<T} γ^t Δd_t < γ^{T−1}(C − γ·c̄) — 늦은 충돌의 벌점이 γ^T 로
  줄어 κ_b > 0 이면 충분히 늦은 충돌에서 깨진다. prog50(= v1.2.0)은 목표 0.5 m 앞 충돌 기준 0.3/0.5/1.0 m/s
  에서 14.5/19/26.5 m 이상이면 깨진다 (알려진 대가, 계약 §6.3).

정체 종단 벌점은 c̄ = 4 ≤ P_s < C − c·200 = 26.
"""

from .reward import RewardSpec, SPEC, SPEC_V110

# 모든 후보는 기본값(= 계약 v1.2.0)에 기대지 않고 v1.1.0 과 다른 항을 전부 적는다 — 기본값이 바뀌어도 후보의
# 뜻이 그대로이게 (v1.2.0 으로 올릴 때 pbrs_eff 가 기본 κ_b 를 물려받는 결함을 시험이 잡았다)
PBRS_EFF = RewardSpec(name='pbrs_eff', phi_dist='eff', progress_kappa=0.0, dev_hinge=0.0)
PROG50 = RewardSpec(name='prog50', phi_dist='eff', progress_kappa=0.5, dev_hinge=0.05)
PROG25 = RewardSpec(name='prog25', phi_dist='eff', progress_kappa=0.25, dev_hinge=0.05)
PROG50_ST = RewardSpec(name='prog50_st', phi_dist='eff', progress_kappa=0.5, dev_hinge=0.05,
                       stall_terminal=True, stall_penalty=10.0)

CANDIDATES = {s.name: s for s in (SPEC_V110, SPEC, PBRS_EFF, PROG50, PROG25, PROG50_ST)}


def get(name):
    """이름 → RewardSpec (없으면 KeyError)."""
    return CANDIDATES[name]
