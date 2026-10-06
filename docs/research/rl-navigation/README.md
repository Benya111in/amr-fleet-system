# 강화학습 내비게이션 (명세 보너스 1) — 조사 사본과 계약

| 문서 | 내용 |
|---|---|
| [rl-practical-survey.md](rl-practical-survey.md) | 「물류 AMR 강화학습 AI 최신 연구」 사본 — 세 갈래 비교군(DWA·순수 RL·후보 선택 하이브리드), 관측·행동·보상, PPO/SAC, 2D·Isaac·Gazebo 역할, DR 우선순위, 평가 통계 |
| [rl-theory-deep-dive.md](rl-theory-deep-dive.md) | 「물류 AMR 강화학습 이론 심층 연구」 사본 — safety node 의 보장 범위, 마스크의 네 조건, PBRS 와 Δd 분해, 정지·자살 창, Slater·비할인 비용, γ·λ 의 초 단위 지평, DR 이론, SRCC |
| [reward_v12.md](reward_v12.md) | 보상 v1.2 검토 (2차 물결 r5-reward) — v1.1.0 의 대기·표류 끌개 착취 표(G0), 후보 설계와 착취 단위 시험(G1), 순수 PPO 학습 가능성(G2), 계약 v1.2.0 반영(G3) |
| [../../../rl/contract/contract.md](../../../rl/contract/contract.md) | **규범 계약** — 위 두 보고서와 저장소 코드에서 확정한 관측·행동·보상·비용·에피소드 (출처 파일:행) |
| [../../../rl/contract/onnx_io.md](../../../rl/contract/onnx_io.md) | 정책 ONNX 입출력 (비교군 B·C) |
| [../../../rl/README.md](../../../rl/README.md) | `rl/` 디렉터리 구조와 소유자 (contract = r0, isaac = r1, sim2d = r2, eval = r4, 플러그인 `src/amr_rl_controller` = r3) |

두 사본은 원본을 고치지 않았다 (머리말 8 행만 추가, 원본 sha256 은 머리말에). 보고서의 수치는 문헌 인용과
오프라인 산술이며 **이 저장소에서 측정한 RL 성능이 아니다**. 계약이 보고서와 다르게 정한 항목(Δd 대신 호장 PBRS,
γ 0.995, 트랙 클래스·크기 제외, 고정 가속도 격자 등)과 그 이유는 `contract.md` §13 에 있다.
