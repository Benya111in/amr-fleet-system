"""
2b 동적 장애물 후보의 보상 설계 (sim2d 쪽 실험 등록부 — 계약 기본값은 바꾸지 않는다).

- ``prog50_c10``: v1.2.0(= prog50) 과 같고 충돌 벌점만 C = 10. 후보 ③ (충돌을 §7 라그랑주 제약으로 옮기고 C 를
  줄인다, 「이론」 201행). 결정 수준 자살 방지 C > γ·c̄ ≈ 3.98 (계약 §6.2) 은 지킨다. 무할인 에피소드 순서
  C > 4 + κ_b·d 는 d > 12 m 진행 뒤 충돌에서 깨지고, 할인 Knox 위반 범위도 넓어진다 — 그 몫은 제약 승수 λ 가
  메운다고 보는 설계다 (실효 벌점 C + λ, dyn_obstacles.md §3).
"""

from amr_rl_contract.reward import RewardSpec

PROG50_C10 = RewardSpec(name='prog50_c10', phi_dist='eff', progress_kappa=0.5, dev_hinge=0.05,
                        collision_penalty=10.0)

CANDIDATES = {s.name: s for s in (PROG50_C10,)}
C_MIN = 3.98          # 결정 수준 자살 방지 하한 γ·c̄ (계약 §6.2)


def get(name):
    """
    이름 → RewardSpec. 등록부에 없고 'prog50_c<수>' 꼴이면 C 만 그 값으로 바꾼 prog50 을 만든다.

    C ≤ γ·c̄ 이면 고의 충돌이 그 자리 정지보다 나아지므로 거부한다 (ValueError). 없는 이름은 KeyError.
    """
    if name in CANDIDATES:
        return CANDIDATES[name]
    if name.startswith('prog50_c'):
        c = float(name[len('prog50_c'):])
        if c <= C_MIN:
            raise ValueError(f'C = {c} ≤ γ·c̄ = {C_MIN}: 결정 수준 자살 방지가 깨진다')
        return RewardSpec(name=name, phi_dist='eff', progress_kappa=0.5, dev_hinge=0.05,
                          collision_penalty=c)
    raise KeyError(name)
