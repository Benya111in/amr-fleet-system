# 강화학습 내비게이션 (명세 보너스 1) — 조사 사본과 계약

보너스 1 결과는 [bonus1_results.md](bonus1_results.md) 부터 읽는다 (불리한 것 먼저, D1–D6 판정표, 기록 지도).

| 문서 | 내용 |
|---|---|
| [bonus1_results.md](bonus1_results.md) | **보너스 1 결과 (단계 D)** — 결론(불리한 것 먼저), 명세 4 줄 ↔ D1–D6 판정표, 단계별 요약, 파이프라인 검증 수치와 남은 전이 간극, 2D·Isaac·Gazebo 방향과 같은 이름·다른 정의, 한계, 다음 일, 원자료·노트·브랜치 지도 |
| [bonus1_completion_plan.md](bonus1_completion_plan.md) | 보너스 1 완성 계획 — 완료 정의 D1–D6, 단계 A–D, 판정 원칙, 변경 이력 (목표 여유 규칙, 플러그인 결정 주기, 단계 D 상태, 부분 메우기 뒤 D3·D5·D6 상태) |
| [b_ppo10.md](b_ppo10.md) | B1 PPO 10 시드 (정본 φ) 의 주·스트레스 집합 평가 — D1 최종 표, it160 대 최종, C 단계 대표 정책 (시드 7) 선택 |
| [b_sac.md](b_sac.md) | B3 SAC 5 시드 대 같은 env-step 의 PPO it160 (D2) — 주 집합은 SAC 가 낮음, 스트레스는 가르지 못함, ONNX 1e-5 미달 |
| [b_sweep.md](b_sweep.md) | B2 개입 예산 훑기 — 제약 PPO (PID 라그랑주) d ∈ {0.2, 0.5, 1.0} × 5 시드, 같은 env-step 무제약 PPO 와 비교 |
| [b_dr.md](b_dr.md) | B4 Domain Randomization 켬 대 끔 (D3) — 학습 때 보지 않은 교란 묶음 v1·v2, 1 차 지표 가르지 못함 |
| [b_dr_map.md](b_dr_map.md) | B4 지도 축 — DR 켬 대 끔을 지도 변형 6 조건 (지터·크기·제거·새 상자, 재계획 경로) 에서 (D3 의 빠진 축). 1 차 지표 가르지 못함, 학습 DR 에 지도 축 없음, 새 상자 조건에서 두 팔 모두 충돌 증가 |
| [c_gazebo.md](c_gazebo.md) | 단계 C — Gazebo 에서 학습 정책 대 팀 DWA (D4·D5), 근접 인계, C4 동적 장애물 스트레스 |
| [b_b5_2d.md](b_b5_2d.md) | B5 — 같은 정책들의 2D 교차 평가와 2D-DWA (D5 의 2D 방향). 스트레스는 Gazebo 와 같은 방향, 주 집합은 판정 불가, 2D 주 집합의 RL 우세는 경로 출처에 민감 |
| [dyn_obstacles.md](dyn_obstacles.md) | 동적 장애물 학습 가능성 (2차 물결 2b) — 피할 수 없는 충돌, 회피 후보 4 개 모두 실패 |
| [notebook/](notebook/README.md) | 보너스 1 실험 노트 — 노트 항목·실행 목록·색인 ([index.md](notebook/index.md)) |
| [rl-practical-survey.md](rl-practical-survey.md) | 「물류 AMR 강화학습 AI 최신 연구」 사본 — 세 갈래 비교군(DWA·순수 RL·후보 선택 하이브리드), 관측·행동·보상, PPO/SAC, 2D·Isaac·Gazebo 역할, DR 우선순위, 평가 통계 |
| [rl-theory-deep-dive.md](rl-theory-deep-dive.md) | 「물류 AMR 강화학습 이론 심층 연구」 사본 — safety node 의 보장 범위, 마스크의 네 조건, PBRS 와 Δd 분해, 정지·자살 창, Slater·비할인 비용, γ·λ 의 초 단위 지평, DR 이론, SRCC |
| [reward_v12.md](reward_v12.md) | 보상 v1.2 검토 (2차 물결 r5-reward) — v1.1.0 의 대기·표류 끌개 착취 표(G0), 후보 설계와 착취 단위 시험(G1), 순수 PPO 학습 가능성(G2), 계약 v1.2.0 반영(G3) |
| [../../../rl/contract/contract.md](../../../rl/contract/contract.md) | **규범 계약** — 위 두 보고서와 저장소 코드에서 확정한 관측·행동·보상·비용·에피소드 (출처 파일:행) |
| [../../../rl/contract/onnx_io.md](../../../rl/contract/onnx_io.md) | 정책 ONNX 입출력 (비교군 B·C) |
| [../../../rl/README.md](../../../rl/README.md) | `rl/` 디렉터리 구조와 소유자 (contract = r0, isaac = r1, sim2d = r2, eval = r4, 플러그인 `src/amr_rl_controller` = r3) |

두 사본은 원본을 고치지 않았다 (머리말 8 행만 추가, 원본 sha256 은 머리말에). 보고서의 수치는 문헌 인용과
오프라인 산술이며 **이 저장소에서 측정한 RL 성능이 아니다**. 계약이 보고서와 다르게 정한 항목(Δd 대신 호장 PBRS,
γ 0.995, 트랙 클래스·크기 제외, 고정 가속도 격자 등)과 그 이유는 `contract.md` §13 에 있다.
