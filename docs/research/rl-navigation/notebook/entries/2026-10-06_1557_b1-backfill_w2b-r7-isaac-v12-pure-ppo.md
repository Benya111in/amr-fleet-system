# 2b r7 — Isaac 에 계약 v1.2.0: BC 없는 순수 PPO 3 시드 0.979–0.983, 개입 비용 10–18 배 (제약 없음)

> **소급 기록** (작성 2026-10-06 19:18 KST 첫 커밋, b1-backfill, 단계 A4). 원 실험: r7-isaac12, 2026-10-06 15:55–16:45 KST (참조 재평가 15:55–15:57,
> 학습 15:57–16:44, 평가 16:13–16:45), 리뷰 독립 시드·대조 16:55–17:04. 사전 등록 `protocol.json` (mtime 15:57:18 < 학습 시작 15:57:52 — 리뷰 확인).
> 수치는 `summary.json`·`curve.jsonl`·`eval_*/eval.json`·`start.txt`·`end.txt`·`gpu_mem.csv` 에서 다시 읽었다. `episodes.npz` 는 읽지 않았다.

- 시각: 2026-10-06 15:57 KST (원 실험) · 에이전트: r7-isaac12 (기록 b1-backfill) · 단계: W2b
- 코드: 학습·평가 5e48670 (git archive 사본), ONNX·golden 174fda8, 참조 평가 bbdbe2f + 미커밋 env.py·train_ppo.py diff, 리뷰 ab8b16f
- 관련 실행 id: `W2b_r7_v12_s{0,1,2}`, `W2b_r7_v12_s{0,1,2}_eval_final_3_15`, `…_eval_final_3_30`, `W2b_r7_v12_s{0,1}_eval_it160_3_15`,
  `W2b_r7_ref_{bcppo_eval_3_15,bcppo_eval_3_15_rerun,bc_eval_3_15,bcppo_eval_3_30}`, `W2b_r7_review_rv_v12_s3`, `…_eval_3_15`,
  `W2b_r7_review_rv_v11_s0`, `…_eval_3_15`, `W2b_r7_golden_in_sim_174fda8`, `W2b_r7_smoke_174fda8`

## 질문
2D 에서 순수 PPO 를 학습시킨 계약 v1.2.0 보상을 Isaac 에 넣으면, 1차 물결에 실패한 Isaac 순수 PPO(BC 없음)가 학습되는가.

## 당시 가설과 예측 (protocol.json, 학습 전)
- 판정: 시드별 마지막 반복 정책, `eval_unbiased.py` (held-out 풀 seed 1 의 3–15 m 2785 경로, 명목 동역학, 평가 seed 4242, env 2048 의 첫 에피소드,
  결정론 tanh(mu), unfinished 0), 통과 기준 성공 ≥ 0.90 (2D 통과 시드 0.948–0.990 / 실패 0.000 을 가르는 값).
- 멈춤 규칙: 반복 160(20.97M) 사본에서 시드 0·1 모두 < 0.05 면 시드 2 를 돌리지 않고 진단으로.
- 대조 v1.1.0 은 다시 돌리지 않고 1차 물결 6 설정(0–0.1 %)을 인용.

## 설정
- rsl_rl 3.1.2, `rsl_rl_ppo_cfg.py` 그대로 (1차 물결 대조와 같음: 엔트로피 0.003, lr 3e-4 adaptive, init_noise_std 0.6, MLP 512-256-128, γ 0.995, λ 0.97),
  보상 v1.2.0 (RewardSpec κ_b 0.5, Φ eff, 힌지 0.05, C 30), BC·라그랑주·커리큘럼 없음 (summary 의 critic_warmup 20), DR 켬, 학습 풀 seed 0 의 3–15 m 2819 경로,
  2048 env × 300 반복 = 39.32M, safety_lite + 근사 D.
- 공유 부하: 2D(r6)와 GPU 공유, loadavg1 시작 13.4·16.0·9.1, gpu_mem.csv 최대 18.8.

## 결과
**판정 평가 (3–15 m)** [측정, eval.json, n 2048, unfinished 0]

| | 시드 0 | 시드 1 | 시드 2 | BC+PPO (1차, v1.1.0) | BC 만 |
|---|---|---|---|---|---|
| 성공 ± SE | **0.979 ± 0.003** (2005) | **0.982 ± 0.003** (2011) | **0.983 ± 0.003** (2014) | 0.979 | 0.981 |
| 정체 / 충돌 / 시간 상한 (건) | 41 / 1 / 1 | 37 / 0 / 0 | 32 / 2 / 0 | 43 / 0 / 0 | 39 / 0 / 0 |
| 평균 주행 s (전체 / 성공) | 12.06 / 11.60 | 11.99 / 11.62 | 11.98 / 11.65 | 13.12 / 12.67 | 17.64 / 17.28 |
| 개입 비용 /에피소드 | **2.836** | **1.656** | **2.995** | 0.168 | 0.055 |
| 반복 160 사본 성공 | 0.983 | 0.979 | — | — | — |
| 3–30 m 성공 (보조) | **0.980** (2008) | 0.978 | 0.978 | 0.982 | — |
| 3–30 m 충돌 (건) | 5 | 2 | 1 | 0 | — |
| 학습 롤아웃 성공 ≥ 0.1 / ≥ 0.9 | 2.62M / 3.93M | 2.62M / 3.93M | 2.62M / 5.24M | — | — |

- **실패 시드 0/3** (0/3 의 95 % 상한 70.8 % — 2D 실패율 1/11 을 반박하지 못한다 [계산]). 멈춤 규칙: 반복 160 에서 0.983·0.979 → 시드 2 실행.
- 학습 벽시계 (start–end): 1 955 · 1 963 · 1 711 s, 합 5 629 s = 1.56 h. 컨테이너 GPU 최대 약 3.4 GB/run.
- 참조 재실행이 비트 단위로 같았다 (성공·개입 동일) — 평가 결정성 [측정].
- 리뷰 [측정]: 독립 시드 3 (v1.2.0, 60 반복 7.86M) 롤아웃 2.62M 0.354 → 3.93M 0.901 → 7.86M 0.969, 판정 평가 **0.9375** (정체 0.0625, 충돌 0, 개입 1.81);
  대조 v1.1.0 시드 0 (같은 3–15 m·7.86M) 롤아웃 0 전 구간, 판정 평가 **0.000** (정체 0.814, 시간 상한 0.180, 충돌 0.0054, 개입 64.3).
- golden 주입 @174fda8: decision 17·step 12 fails [], 최대 오차 5.0e-8 / 4.8e-10. smoke 256 env fail [], LiDAR 최대 2.2e-5 m.
- 실패 분석 [문서주장 — episodes.npz 기반, 이 기록에서 다시 세지 않음]: 다섯 정책 모두 실패한 13 건은 9 경로·목표 4 개, 그 목표는 held-out 목표 511 개 중
  장애물 여유가 가장 좁은 쪽 (0.375–0.425 m); 정체 d_final 중앙값 0.155·0.180·0.148 m, 정체 110 건 중 11 건은 1.6–7.4 m 앞 (리뷰).
  개입의 86–97 % 가 정체 에피소드(약 2 %)에서 나온다.

## 해석
- **Isaac 순수 PPO 실패(1차 물결)는 보상 탓이었다**: 같은 코드 계열·같은 하이퍼파라미터에서 v1.1.0 은 0.000, v1.2.0 은 3/3 (+ 리뷰 1 시드) 통과.
  경로를 3–15 m 로 좁힌 탓이라는 교란은 리뷰 대조(같은 3–15 m 에서 v1.1.0 0.000)로 닫혔다 — 단 대조는 1 시드·7.86M.
- 2D v1.2.0 (11 시드 중 10 통과, 0.948–0.990) 과 **같은 방향**이다 (주행 시간 12 s 대로 비슷). 그러나 SRCC 가 아니다 — 경로 생성기·φ·DR·PPO 구현·예산이 다르다.
- **안전 비용이 크다**: 제약 없는 PPO 라 개입 비용이 BC+PPO 의 10–18 배이고 계약 임시 범위 {0.02–0.2} 밖, 충돌 0–5 건/2048 (3–15 m 1·0·2, 3–30 m 5·2·1). 이 정책들은
  "제약 충족" 근거가 될 수 없다. 개입은 목표 0.15–0.18 m 앞에서 멈춘 채 φ 를 미는 정체 에피소드에 몰린다 — 근사 φ(도킹 예외 없음)와
  좁은 목표(성공 0.10 m vs E-STOP 0.30 m)의 겹침으로 보이나 인과는 미검증 (완성 계획 A3: 정본 φ 로 재판정).
- 0.15–0.18 m 정지는 Nav2 기본(0.25 m)·Habitat(0.2 m) 기준으로는 성공이다 — 우리 성공 기준이 가장 엄격하다 (리서치 「목표 직전 정지와 전이」).

## 무엇을 바꾸게 했나
- 완성 계획 D1 (시드 ≥ 10, 정본 φ, 3–15 m 와 3–30 m 평가, Wilson·IQM), B2 (개입 예산 훑기 {0.2, 0.5, 1.0} × 5 시드, PID), A3 (정본 φ 이식 + 직전 정지 재판정),
  D5 의 "엄격 0.10 m / Nav2 호환 0.25 m / 최종 거리 분포" 다중 성공 정의와 "RL 단독 / RL + 근접 인계" 분리.
- ONNX 3 개(`train/v12_s{0,1,2}/policy_final.onnx`, algo "unconstrained", contract 1.2.0)가 플러그인(A2)·Gazebo(C) 의 첫 후보.

## 원자료
- `/home/CAPYI_AI/runs/r7-isaac12/` — `protocol.json`, `train/v12_s{0,1,2}/` (summary.json, curve.jsonl, start/end.txt, gpu_mem.csv, eval_*/eval.json·episodes.npz, policy_*.{pt,onnx}),
  `ref/ppo_b_bc_fz/`, `review/train/`, `golden/`, `smoke/`, `g1_summary.json`, `g1_curves.png`
- 증거 사본 `/home/CAPYI_AI/logs/agents/r7-isaac12/{g0,g1,g2}/`, 문서 `rl/isaac/README.md` 「계약 v1.2.0 과 BC 없는 순수 PPO」

## WAVE 보고와 원자료의 차이
- 3–30 m 시드 0 성공: WAVE·README 0.981, 원자료 0.98047 (2008/2048) → **0.980** (리뷰가 지적).
- loadavg: WAVE "5–17", README "5–16" — 원자료 gpu_mem.csv loadavg1 최대 18.8 (리뷰가 지적).
- WAVE "충돌이 1–5 건/2048" — 원자료 판정 평가는 3–15 m 1·0·2, 3–30 m 5·2·1 건이라 범위는 0–5 다 (시드 1 의 3–15 m 가 0).
  (리뷰 반영으로 더함 — 처음엔 「해석」 에 WAVE 값을 그대로 옮겼다. 표는 처음부터 맞았다.)
- "정체는 목표 0.15–0.18 m 앞" 은 중앙값만 맞고 11/110 건은 1.6–7.4 m 앞이다 (리뷰가 지적, episodes.npz 는 이 기록에서 다시 읽지 않음).
- 리서치 「목표 직전 정지와 전이」 절의 "정지 거리 중앙값 0.15–0.18 m" 도 같은 반올림 (리뷰 재계산 0.148–0.180 m).
- 그 밖(시드별 성공·건수·주행 시간·개입·학습 시점·벽시계)은 원자료와 같다.

## 실행 목록에 등록한 run (`runs/b1-backfill.jsonl`, entry = 이 파일)

| run_id | 종류 | 시뮬 | 커밋 | 시드 | 표기 |
|---|---|---|---|---|---|
| `W2b_r7_v12_s0` | train | isaac | 5e48670 | 0 | [측정] |
| `W2b_r7_v12_s0_eval_final_3_15` | eval | isaac | 5e48670 | 평가 seed 4242 | [측정] |
| `W2b_r7_v12_s0_eval_final_3_30` | eval | isaac | 5e48670 | 평가 seed 4242 | [측정] |
| `W2b_r7_v12_s0_eval_it160_3_15` | eval | isaac | 5e48670 | 평가 seed 4242 | [측정] |
| `W2b_r7_v12_s1` | train | isaac | 5e48670 | 1 | [측정] |
| `W2b_r7_v12_s1_eval_final_3_15` | eval | isaac | 5e48670 | 평가 seed 4242 | [측정] |
| `W2b_r7_v12_s1_eval_final_3_30` | eval | isaac | 5e48670 | 평가 seed 4242 | [측정] |
| `W2b_r7_v12_s1_eval_it160_3_15` | eval | isaac | 5e48670 | 평가 seed 4242 | [측정] |
| `W2b_r7_v12_s2` | train | isaac | 5e48670 | 2 | [측정] |
| `W2b_r7_v12_s2_eval_final_3_15` | eval | isaac | 5e48670 | 평가 seed 4242 | [측정] |
| `W2b_r7_v12_s2_eval_final_3_30` | eval | isaac | 5e48670 | 평가 seed 4242 | [측정] |
| `W2b_r7_ref_bcppo_eval_3_15` | eval | isaac | bbdbe2f | 평가 seed 4242 | [측정] |
| `W2b_r7_ref_bcppo_eval_3_15_rerun` | eval | isaac | bbdbe2f | 평가 seed 4242 | [측정] |
| `W2b_r7_ref_bc_eval_3_15` | eval | isaac | bbdbe2f | 평가 seed 4242 | [측정] |
| `W2b_r7_ref_bcppo_eval_3_30` | eval | isaac | bbdbe2f | 평가 seed 4242 | [측정] |
| `W2b_r7_review_rv_v12_s3` | train | isaac | ab8b16f | 3 | [측정] |
| `W2b_r7_review_rv_v12_s3_eval_3_15` | eval | isaac | ab8b16f | 평가 seed 4242 | [측정] |
| `W2b_r7_review_rv_v11_s0` | train | isaac | ab8b16f | 0 | [측정] |
| `W2b_r7_review_rv_v11_s0_eval_3_15` | eval | isaac | ab8b16f | 평가 seed 4242 | [측정] |
| `W2b_r7_golden_in_sim_174fda8` | test | isaac | 174fda8 | n/a | [측정] |
| `W2b_r7_smoke_174fda8` | probe | isaac | 174fda8 | n/a | [측정] |
