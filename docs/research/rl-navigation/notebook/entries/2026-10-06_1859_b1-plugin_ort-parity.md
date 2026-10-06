# A2-2 학습 정책 ONNX 의 Python ORT·torch·C++ 플러그인 출력 일치

- 시각: 2026-10-06 18:59 KST · 에이전트: b1-plugin · 단계: A2
- 코드: `a7da8a2` (브랜치 `feature/b1-plugin`) — 도구는 이 항목과 함께 커밋한다
- 관련 실행 id: (끝나면 채운다 — runs/b1-plugin.jsonl)

## 질문

Isaac 에서 학습한 정책(r7-isaac12 `v12_s{0,1,2}/policy_final.onnx`, 계약 v1.2.0, 비교군 B)을 Gazebo 플러그인의
C++ `OnnxPolicy` 로 돌렸을 때, 같은 관측에서 학습 쪽 정의(torch)·Python onnxruntime 과 **같은 행동**을 내는가?
그리고 관측 정규화가 두 번 적용되지 않고, 행동 후처리(tanh → integrate_b → 한계)가 Isaac env 와 같은가?
근거: `/home/CAPYI_AI/research/reports/실측 기반 후속 기술 조사.md` 「전이 점검과 순위 상관」 — "학습 프레임워크 출력과
C++ ORT 출력의 최대 차이(FP32 에서 10⁻⁵ 미만), ORT 스레드를 1 개로 고정했을 때의 비트 동일성, 관측 정규화가 두 번
적용되지 않는지" (자체 제안, 추론). 완성 계획 D4 의 "Python/C++ ORT 출력 일치".

## 가설과 예측 (결과를 보기 전에 적는다)

- 같은 ORT 1.23.2 (C++ `/opt/onnxruntime`, Python 휠) · 같은 그래프 · 같은 CPU 커널이므로 C++ 과 Python ORT 는
  **비트 단위로 같을 것**이라고 예측한다 (최소한 1e-6 안). torch(float32, CPU)와는 연산 순서가 달라 1e-6 수준 차이를
  예측한다 (r7 의 내보내기 검사 [측정]: ORT 대 torch PolicyB 최대 3.3e-6 — 표준정규 무작위 관측, `onnx_check_policy_final.json`).
- 정규화는 모델 안에만 있다 (`policy_export.PolicyB`: clip((obs − μ)/√(max(σ², 0.01) + 1e-8), −10, 10)). 플러그인은
  계약 관측만 만든다 (golden d01–d17 일치). 예측: C++ 출력은 "한 번 정규화" 경로와 1e-5 안, "두 번 정규화" 경로와는
  크게(> 1e-3) 다르다.
- 행동 후처리: Isaac env 는 `tanh(clamp(raw, −20, 20))` 를 float64 로, ONNX 는 float32 `tanh` 를 그래프 안에서 한다.
  예측: 그 차이는 float32 반올림 수준(≤ 1e-6)이고 integrate_b 의 세 구현(C++·numpy 기준·Isaac torch)은 같은 double
  입력에서 1e-12 안.

## 설정 (판정 규칙 — 결과 전에 고정)

정책: `/home/CAPYI_AI/runs/r7-isaac12/train/v12_s{0,1,2}/policy_final.onnx` (+ 같은 디렉터리 `policy_final.pt`).
관측 묶음 (정책마다):

| 묶음 | 내용 | 비교 |
|---|---|---|
| G | golden decision d01–d17 의 상태 → C++ 관측 / numpy 기준 관측 | 상태→관측→행동 끝-끝 |
| R | 무작위 계약 상태 4096 개 (고정 시드, 경로·스캔(무효 빔 포함)·속도·트랙·safety·이탈 무작위) | 끝-끝 + 관측 대조 |
| N | 표준정규 무작위 관측 4096 개 (클립 밖 값 포함) | 망 자체 |
| I | Isaac 기록 관측 `eval_*_heldout_*/obs_batch.npz` (seed 마다 512 × 2–3) — 같은 seed 의 final 정책 기록은 Isaac CUDA 출력 `out` 도 | 망 자체 + Isaac 실측 출력 |

구현: (a) Python ORT (CPU EP, intra_op 1, 순차), (b1) torch `PolicyB` 재구성 (`rl/isaac/amr_isaac/policy_export.py`,
float32 CPU, `amr-ai/rl:0.3`), (b2) **학습 때 쓴 rsl_rl 3.1.2 `ActorCritic` + `ContractNormalization`**
(`amr-ai/isaaclab:2.3.2` 의 Isaac Python, CPU — 가능하면 CUDA 도), (c) C++ `amr_rl_controller::OnnxPolicy`
(플러그인과 같은 클래스, intra_op 1, 배치 1).

판정 (하나라도 어기면 "불일치" — Gazebo 로 가기 전에 원인을 찾는다):
- P1 C++ 대 Python ORT: 모든 묶음·세 정책에서 행동 `action` 최대 |Δ| < 1e-5. 비트 동일 비율도 보고.
- P2 Python ORT 대 torch (b1·b2, CPU): 최대 |Δ| < 1e-5. Isaac CUDA 기록 `out` 과의 차이는 **보고만** (TF32 등).
- P3 C++ ORT intra_op 1 반복: 같은 프로세스에서 순서를 바꿔 3 회(정순·역순·섞음) + 별개 프로세스 2 회 → 모든 출력
  **비트 동일** (다른 비트 0 개). intra_op 4 도 참고로 잰다.
- P4 C++ 관측 대 numpy 기준 관측 (G·R): 최대 |Δ| ≤ 1e-5 (계약 §12 허용오차).
- P5 정규화 한 번: ONNX 그래프의 입력 직후 정규화(Sub·Div·Clip) 패턴이 하나이고, C++ 출력은 "한 번" 경로와 1e-5 안,
  "두 번" 경로와는 최대 |Δ| > 1e-3 (= 이 검사가 이중 정규화를 가를 수 있다).
- P6 후처리: C++ `integrateB` · numpy `kinematics.integrate_b` · Isaac `contract_torch.integrate_b` 를 무작위 10⁵ 입력
  (u 범위 밖 포함, v_cap ∈ {0, 0.3, 1.0, 2.5} 와 무작위)에서 최대 |Δ| ≤ 1e-12. tanh float32 대 float64 차이는 보고.

산출: `/home/CAPYI_AI/runs/bonus1/b1-plugin/A2_parity/` (표 JSON, 원출력 .npy/.f32). 도구:
`src/amr_rl_controller/tools/parity_dump.cpp` (C++), `src/amr_rl_controller/tools/parity/*.py` (Python).

## 결과

(실행 뒤 채운다)

## 해석

(실행 뒤 채운다)

## 다음

(실행 뒤 채운다)
