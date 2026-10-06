# rl/isaac — Isaac Lab 2.3.2 학습 환경 (보너스 1, 비교군 B)

명세 5장 보너스 1 (`31_*.md` 323행): "Isaac Sim의 강화학습 환경을 활용하여 PPO/SAC 기반 내비게이션 정책을
학습시킨다." 이 디렉터리는 그 Isaac 트랙이다. 관측·행동·보상·비용·종료는 `rl/contract/` **계약 v1.2.0** 을 따른다
(2차 물결 r7 이 보상을 v1.1.0 → v1.2.0 으로 올렸다 — 아래 「계약 v1.2.0 과 BC 없는 순수 PPO」).
결과 수치·실패 기록의 원자료는 커밋하지 않는다: 1차 물결(v1.1.0, BC 예열) `/home/CAPYI_AI/runs/r1-isaac/`,
2차 물결(v1.2.0, 순수 PPO) `/home/CAPYI_AI/runs/r7-isaac12/` (`train/<run>/` 의 `curve.jsonl`·`summary.json`·
`eval_*/eval.json`·`episodes.npz`, 학습 전에 정한 판정 규칙 `protocol.json`). 이 문서는 구조와 사용법이다.

## 구성

| 파일 | 내용 | Isaac 필요 |
|---|---|---|
| `amr_isaac/contract_torch.py` | 계약 기준 구현(numpy)의 배치 torch 판 — 관측 291, 보상 8 항(v1.2.0, `reward.RewardSpec` 을 그대로 받는다), 전이 전 거리 캐시 `observe_with_cache`, 비용 5, 이탈 상태기계, `integrate_b`, 프로파일러, safety_lite | 아니오 |
| `amr_isaac/warehouse_geom.py` | `warehouse.sdf`(읽기 전용) + include 모델 → 충돌 상자 229 개 → 삼각형 메시 / 2D 사각형 129 개 / 0.05 m 점유 격자 | 아니오 |
| `amr_isaac/paths.py`, `assets.py` | 전역 코스트맵 팽창 + A* 간선 비용으로 비용 최단 경로 풀 (캐시 `$AMR_ISAAC_RUNS/assets`, 커밋 안 함 — 해시 `58eaa34ae5`) | 아니오 |
| `amr_isaac/env_math.py` | env 의 순수 함수 — 풋프린트 분리축 충돌 `footprint_hits`, safety_lite 접근 거리 `approach_distance`, 원호 적분 `integrate_pose`, 서브스텝 이동·접촉 `move_and_collide`, 경로 길이 필터 `select_paths_by_length`, 보상 판 `reward_spec_for`, 에피소드 기록 `EpisodeLog` | 아니오 |
| `amr_isaac/env_cfg.py`, `env.py`, `tasks.py` | `DirectRLEnv` — `AMR-Isaac-Nav-B-v0` (학습), `AMR-Isaac-Nav-B-Eval-v0` (held-out 경로·명목 동역학) | 예 |
| `amr_isaac/agents/rsl_rl_ppo_cfg.py` | rsl_rl PPO 설정 (γ 0.995, λ 0.97, rollout 64) | 예 |
| `amr_isaac/expert.py` | BC 예열용 특권 정보 전문가 (경로 순수 추종) | 아니오 |
| `amr_isaac/policy_export.py` | rsl_rl state dict → 계약형 `PolicyB` → ONNX (`onnx_io.md`) | 아니오 |
| `scripts/` | `build_assets` · `smoke_env` · `check_golden_in_sim` · `train_ppo` · `eval_unbiased`(판정용) · `eval_policy`(1차 물결, 편향 있음) · `export_onnx` · `plot_curves` · `gpu_monitor.sh` | 일부 |
| `tests/` | golden 32 대조(직접·env 캐시 두 길), 기준 구현과 무작위 차분(v1.2.0·v1.1.0), v1.2.0 보상 손 사례(`test_reward_v12.py`), 창고 변환기·경로 풀, env 순수 함수(`test_env_math.py`), ONNX 메타데이터 | 아니오 |
| `run_isaac.sh` | Isaac 컨테이너 실행기 (`env/verify/verify_isaac.sh` 의 마운트·환경변수). `AMR_AI_ID`(기본 `r7-isaac12`)로 이름 `amr-ai-<id>-<태그>`·산출 루트 `/home/CAPYI_AI/runs/<id>` 를 정하고 컨테이너에 `AMR_ISAAC_RUNS` 로 넘긴다 | — |

## 실행

```bash
WT=/home/CAPYI_AI/wt/r7-isaac12; R=/home/CAPYI_AI/runs/r7-isaac12
# 1) 순수 시험 (CPU, Isaac 불필요)
docker run --rm -v $WT:/ws -w /ws/rl/isaac amr-ai/rl:0.3 python3 -m pytest -q tests
# 2) 창고 기하·경로 풀 캐시 (없으면 env 가 스스로 만든다 — 한 프로세스로 수 분)
docker run --rm -v $WT:/ws -v $R:$R -w /ws amr-ai/rl:0.3 python3 rl/isaac/scripts/build_assets.py --goals 512
# 3) env 점검·Isaac 안 golden 주입 시험
rl/isaac/run_isaac.sh smoke  rl/isaac/scripts/smoke_env.py --num_envs 256
rl/isaac/run_isaac.sh golden rl/isaac/scripts/check_golden_in_sim.py
# 4) 학습 → 평가 → ONNX
# ppo_b_bc_fz 의 조합: 전문가 시연 BC 예열 → 정규화 통계 고정 → critic 예열 20 반복 → PPO(고정 lr 1e-4).
# --lagrange 의 마지막 값이 개입 예산 d₁ 이다. 아래 5.0 은 계약 §7.2 의 훑기 값 {0.02, 0.05, 0.1, 0.2} 밖이라
# λ 가 반복 60 부터 전부 0 이 됐다 — 이 조합의 PPO 는 사실상 제약이 없다 (아래 「계약과 다르게」 10).
rl/isaac/run_isaac.sh train rl/isaac/scripts/train_ppo.py --num_envs 2048 --iterations 300 --run <run> \
    --bc_steps 200 --critic_warmup 20 --init_std 0.25 --lr 1e-4 --schedule fixed --entropy 0 \
    --lagrange 0,0.05,0.05,0.05,0.005/0.5,0.01,0.01,0.01,0.001/5.0
# 2차 물결 (계약 v1.2.0): BC·라그랑주 없는 순수 PPO = 제약 없는 PPO, 경로 3–15 m, 20M 사본
rl/isaac/run_isaac.sh v12s0 rl/isaac/scripts/train_ppo.py --num_envs 2048 --iterations 300 --run v12_s0 \
    --seed 0 --reward 1.2.0 --path_len 3:15 --save_at_env_steps 20e6
# 판정 평가: env 마다 첫 에피소드만, 평가 seed 고정, unfinished 0 (eval_policy.py 는 짧은 결말 쪽으로 치우친다)
rl/isaac/run_isaac.sh ev rl/isaac/scripts/eval_unbiased.py --policy $R/train/<run>/policy_final.pt \
    --num_envs 2048 --seed 4242 --path_len 3:15 --tag final_heldout_3_15
# algo·train_steps·라그랑주 사실은 summary.json·curve.jsonl·bc.json 에서 메타데이터로 옮긴다 (--algo 를 주지 않는다)
docker run --rm -v $WT:/ws -v $R:$R -w /ws amr-ai/rl:0.3 python3 rl/isaac/scripts/export_onnx.py --run $R/train/<run> \
    --obs_batch $R/train/<run>/eval_final_heldout_3_15/obs_batch.npz
```

앱 종료(`app.close()`)가 CPU 100 % 로 멈춘 적이 있어 모든 Isaac 스크립트는 끝에서 `os._exit` 한다.

## 계약 v1.2.0 과 BC 없는 순수 PPO (2차 물결 r7, 2026-10-06)

**불리한 것 먼저.**
- **제약 없는 PPO 다.** 라그랑주 승수는 0 그대로(`--lagrange` 안 줌), BC 없음. 계약 §7 의 비용 제약을 쓰지 않았고
  1차 물결의 d₁ 문제(§7.2 훑기)는 이번 범위 밖이다. 그 대가 [측정, 아래 표]: 안전 개입 비용이 에피소드당
  1.66–3.00 으로 1차 물결 BC+PPO(0.168)의 10–18 배, §7.2 임시 범위 {0.02–0.2} 밖이다. 86–97 % 가 정체 에피소드
  (약 2 %)에서 나온다 — 목표 0.15–0.18 m 앞에서 멈춘 채 φ 를 향해 계속 명령한다. 성공 에피소드만 봐도 0.05–0.26 으로
  BC+PPO(0.011)보다 크다.
- **충돌이 0 이 아니다.** 3–15 m 1·0·2 건 / 2048, 3–30 m 5·2·1 건 / 2048 (BC+PPO 는 둘 다 0). 3–30 m 의 8 건은
  두 목표로 가는 경로뿐이다 — 시드 0 의 5 건은 목표 (−29.58, 18.43), 시드 1·2 의 3 건은 목표 (1.68, −8.57) (최대 이탈
  0.57–0.78 m, 목표 1.1–2.7 m 앞). 두 목표는 아래의 좁은 목표 축이다 (장애물 여유 0.425·0.375 m). 3–15 m 시드 2 의
  1 건은 다른 목표다.
- **시드 3 개뿐이다.** 실패 0/3 은 2D 의 실패율 1/11 을 반박하지 못한다 (0/3 의 95 % 상한 70.8 %; p = 1/11 이어도
  0/3 이 나올 확률 75 %).
- 학습 경로를 3–15 m 로 좁혔다 (2D 2a 와 같게, 계약 §10 은 3–30 m). 대조 v1.1.0(1차 물결 `ppo_b_s0`)은 3–30 m·11.8M
  이라 경로 범위·예산이 다르다 — 같은 3–15 m 의 v1.1.0 Isaac run 은 **미측정** (경로 커리큘럼 run `ppo_b_s0_cur` 는 짧은
  경로에서 시작해도 7.9M 동안 0 이었다).
- 남은 실패 약 2 % (대부분 정체)는 정책들이 상당 부분 같은 경로에서 낸다. 다섯 정책(v1.2.0 3 시드·BC+PPO·BC)이
  모두 실패한 13 건은 9 경로·목표 4 개이고, 그 목표는 held-out 3–15 m 목표 511 개 중 장애물 여유가 가장 좁은 쪽
  (0.375–0.425 m, 1–5 번째)이다 — 몸체 반길이 0.30 m 라 목표에서 앞 끝이 상자에 0.08–0.13 m 로 E-STOP 거리 0.30 m 안이다.
  정책보다 경로 풀(`paths.GOAL_CLEARANCE` 0.40)과 φ 의 문제로 본다 [계산, 인과는 미검증].

**설정** (학습 전에 정함, `/home/CAPYI_AI/runs/r7-isaac12/protocol.json`): `rsl_rl_ppo_cfg.py` 그대로(1차 물결 대조와
같은 하이퍼파라미터 — 엔트로피 0.003, lr 3e-4 adaptive, MLP 512-256-128), 보상 v1.2.0, DR 켬, 커리큘럼 없음,
2048 env × 300 반복 = 39.3M env-steps, 시드 0·1·2. 멈춤 규칙: 20M(반복 160) 사본에서 시드 0·1 모두 5 % 미만이면 시드 2
를 돌리지 않는다 → 0.983·0.979 라 돌렸다. 판정: 마지막 반복 정책, `eval_unbiased.py` (held-out 풀 seed 1 의 3–15 m,
명목 동역학, 평가 seed 4242, env 2048 개의 첫 에피소드, unfinished 0), 통과 기준 성공 ≥ 0.90.

**결과** [측정, 공유 부하 하 예비값 — loadavg 5–16, 2D 학습과 GPU 공유]:

| | 시드 0 | 시드 1 | 시드 2 | BC+PPO (1차, v1.1.0) | BC 만 |
|---|---|---|---|---|---|
| 성공 3–15 m (판정) | **0.979 ± 0.003** | **0.982 ± 0.003** | **0.983 ± 0.003** | 0.979 | 0.981 |
| 정체 / 충돌 / 시간 상한 | 0.020 / 0.0005 / 0.0005 | 0.018 / 0 / 0 | 0.016 / 0.0010 / 0 | 0.021 / 0 / 0 | 0.019 / 0 / 0 |
| 평균 주행 시간 (전체 / 성공) s | 12.06 / 11.60 | 11.99 / 11.62 | 11.98 / 11.65 | 13.12 / 12.67 | 17.64 / 17.28 |
| 개입 비용 /에피소드 | 2.84 | 1.66 | 3.00 | 0.168 | 0.055 |
| 20M 사본 성공 | 0.983 | 0.979 | (평가 안 함) | — | — |
| 성공 3–30 m (보조) | 0.981 | 0.978 | 0.978 | 0.982 | — |
| 학습 롤아웃 성공 ≥ 0.1 / ≥ 0.9 | 2.6M / 3.9M | 2.6M / 3.9M | 2.6M / 5.2M | — | — |

- 1차 물결 대조 v1.1.0 순수 PPO 6 설정은 5.9–11.8M 동안 0–0.1 % (`/home/CAPYI_AI/runs/r1-isaac/train/ppo_b_s0*` 등).
- 학습 벽시계 (프로세스, 시작~끝): 1 955 · 1 963 · 1 711 s, 합 1.56 h (상한 4 h). 컨테이너 GPU 최대 3.4 GB/run.
- ONNX: 세 정책 모두 `export_onnx.py` fails 0, ORT 대 torch ≤ 3.3e-6, Isaac CUDA 대 ORT ≤ 7.2e-7, 메타데이터
  `algo` = "PPO (rsl_rl 3.1.2), unconstrained (no Lagrangian / cost constraint)", `constraint` = none,
  `contract_version`·`reward_version` = 1.2.0.
- 평가 재현: 같은 정책·seed 로 두 번 돌리면 에피소드별 결과가 비트 단위로 같고, 다른 정책도 첫 에피소드 경로가 같다
  (`path_ids_sha` 39105bab9dde84e6).

**2D(2a)와의 예비 비교 — "같은 방향인가" 수준이다 (SRCC 가 아니다).** 같은 계약 v1.2.0·같은 3–15 m 범위·같은 평가
추정량(첫 에피소드, seed 4242, n 2048)이지만 경로 생성기·φ(2D 는 C++ SafetyGate 바인딩, Isaac 은 근사 D)·DR·PPO
구현(2D: 엔트로피 0, 고정 lr + KL 조기 중단, 보상 표준편차 정규화, MLP 256-256)·예산(2D 14.9M)이 다르다.

| | Isaac v1.2.0 (3 시드) | 2D v1.2.0 (11 시드) |
|---|---|---|
| 시드별 성공 | 0.979 · 0.982 · 0.983 | 10 시드 0.948–0.990, 시드 2 0.0005 |
| 실패 시드 (< 0.90) | 0/3 | 1/11 (직전 정지 끌개) |
| 평균 주행 시간 (전체 에피소드) | 11.98–12.06 s | 통과 시드 11.87–12.63 s |
| 평가 경로 평균 길이 | 9.98 m | 9.97 m |
| 정체 / 충돌 | 1.6–2.0 % / 0–0.10 % | 통과 시드 1.0–5.2 % / 0 |
| 개입 비용 /에피소드 | 1.66–3.00 | 통과 시드 0.02–0.14 (φ 가 다르다) |
| 학습 롤아웃 성공 10 % 도달 | 2.6M (chunk 해상도 1.3M) | 통과 시드 2.6–3.7M |

방향은 같다: v1.1.0 에서 두 시뮬레이터 모두 순수 PPO 가 실패했고, v1.2.0 에서 두 시뮬레이터 모두 BC 없이 약 98 % 에
닿으며 주행 시간도 비슷하다. 다른 점은 Isaac 쪽의 충돌·개입이 크다는 것이다 (제약 없는 PPO + 근사 φ).

## 전이 한 번 (Δt = 0.1 s = 물리 0.02 s × 5)

raw 행동 → `u = tanh(raw)` → `integrate_b` (ray 클램프) → 결정 주기 안 선형 명령 램프 → [50 Hz × 5]
프로파일러 기준 경로 → safety_lite φ → 플랜트(순수 지연 + 1 차 지연 + 가속 한계) → 정확 원호 적분 →
풋프린트 대 정적 사각형 분리축 충돌 판정 (접촉 = 그 자리 정지·종단) → Warp `RayCaster` 720 빔 → 계약 관측.
버퍼에는 rsl_rl 이 낸 원출력(raw)과 그 log-prob 가 들어간다 (contract §5). 결정론적 배포 행동 = `tanh(mu)` = ONNX `action`.

## 계약과 다르게, 또는 계약이 정하지 않아 여기서 정한 것

1. **safety_lite 의 접근 거리 D** — 계약은 D 를 입력으로 받는다(§7.4). 여기서는 정본 `SweptFootprint` 대신
   근사: 직진(|v| ≥ 1e-3)은 진행 방향 앞·차폭 띠(|y| ≤ 0.20) 안 점까지의 세로 거리, 제자리 회전은 외접원 밖
   반지름 거리, 정지는 +inf, 풋프린트 안 점은 0. 원호·꼬리 직선·가설 여러 개·강건 통계·지연 보정 일부가 빠졌다.
   STOP 래치는 lite 규칙대로 없다 (확정 전 STOP = CRITICAL). r2(2D) 와 같은 φ 가 아니다 — SRCC 보고에 적을 것.
2. **하류 사슬 순서** — contract §5.4 는 "프로파일러 → 플랜트 → φ" 로 적었다. 여기서는 실제 배선(프로파일러가
   낸 cmd_vel 을 safety_node 가 거르고 DiffDrive 가 응답)대로 **프로파일러 → φ → 플랜트** 다. 개입 비용의
   (u, φ(u)) 는 계약 §7.1 대로 (프로파일러 출력, φ 출력) 이다.
3. **실행 가속도 관측** — `a_v_meas` = 프로파일러 선속도 필터 상태 `a`, `a_w_meas` = 프로파일러 `w_ref` 의
   서브스텝 차분 (계약 State 주석 "프로파일러 상태").
4. **동적 장애물 없음** — 트랙 블록은 늘 0, `min_ttc` 없음. 학습 분포에서 트랙 특징이 상수라 배포에서 트랙이
   생기면 정규화 뒤 값이 커진다 (분산 하한 1e-2 로 ±10 이하이지만 학습되지 않은 입력이다). 다음 물결 과제.
5. **맵 다양성(DR 축 1) 없음** — 창고 한 장. held-out 은 같은 맵의 다른 시작·목표(경로 풀 seed 1)다.
6. **DR 구현분** — 플랜트 T_m·T_d·가속 한계 배율, LiDAR σ·빔 드롭아웃. 관측·행동 지연(축 4)과 트랙 누락(축 5)
   은 없다. 순수 지연은 0.02 s 단위로 양자화 (1–4 서브스텝).
7. **A\* 경로** — `amr_navigation` C++ 플래너를 부르지 않고 같은 비용 규칙의 Dijkstra(같은 최적 비용, 동률 경로
   선택은 다를 수 있다). C++ 평활기(숏컷 + Gauss–Seidel) 대신 숏컷만 근사한 뒤 0.05 m 재표본.
8. **관측 정규화** — rsl_rl `EmpiricalNormalization` 의 통계 갱신을 쓰고 적용 식만 계약식으로 바꿨다
   (`train_ppo.ContractNormalization`). 분산 하한 1e-2 를 두었고 ONNX 에도 같은 σ² 를 넣는다.
9. **학습 전용 커리큘럼** (1차 물결, v1.1.0) — 경로 길이 상한(구조 파라미터, 20 % 는 전체 풀 균일, contract §10)과 **v_cap**
   (관측된다; 평가·비교는 v_cap = 1.0). 둘 다 계약에 없는 [설계 선택]이고, 처음부터의 PPO 에서는 성공을 만들지
   못했다 (`train/ppo_b_s0_cur`·`ppo_b_s0_cv`·`ppo_b_s0_cv_ent`·`ppo_b_lag_a`·`ppo_b_lag_b` 의 `curve.jsonl`:
   60–100 반복에서 성공률 최대 0.1 %). 지금 조합은 쓰지 않는다.
10. **PPO-Lagrangian (보상 벌점형) — `ppo_b_bc_fz` 에서는 승수가 0 으로 꺼져 사실상 제약 없는 PPO 였다.**
   계약 보상만으로는 추종 벌점이 이탈 구간(e > 0.30 m) 밖에서만 걸려 경로에서 멀리 벗어나는 쪽이 유리하다. 그래서
   계약 §7 의 비용 5 채널을 PPO 보상에서 λ·c 로 빼고, 10 반복마다 λ ← max(0, λ + η·clip((J − d)/d, −1, 5)) 로
   갱신하게 만들었다 (단일 critic, J = 끝난 에피소드의 비할인 비용 합 평균). 기록하는 반환은 계약 보상이다.
   - **계약과 다르게 한 것**: 개입 예산 d₁ 을 5.0/에피소드로 두었다. 계약 §7.2 는 d₁ 이 미측정인 동안
     {0.02, 0.05, 0.1, 0.2} 를 훑어 (개입, 주행시간, 성공률) Pareto 곡선과 마지막 λ 를 보고하라고 정해 두었다.
     5.0 은 그 범위 밖이고 훑기도 하지 않았다. 처음 이 문서는 "계약이 미정이라" 5.0 으로 두었다고 적었는데,
     계약에서 미정인 것은 최종 값(DWA 실측 × 1.2)뿐이고 그때까지의 훑기 범위는 정해져 있었다.
   - **무슨 일이 있었나** [측정, `train/ppo_b_bc_fz/curve.jsonl`·`summary.json`]: 개입 비용(0.02–0.31/에피소드)이
     늘 d₁ 보다 훨씬 작아 λ₁ 은 줄기만 했다. 다른 채널도 비용이 예산(충돌 0.02·이탈 5·복귀 10·CTE 50)보다 작아
     (충돌·이탈·복귀 0, CTE 1.06–1.75) 같은 방향이었다. λ 는 반복 50 기록에 [0, 0.00033, 0, 0, 0.00012],
     **반복 60–300 기록에 전부 0** 이고 `lagrange_final` 도 전부 0 이다. 곧 반복 61–300 은 λ = 0 으로 학습했다.
     **최종 정책 = BC 예열 + 사실상 제약 없는 PPO.**
   - **그 대가** [측정]: 학습 중 개입 비용이 반복 10 의 0.023/에피소드에서 반복 230–300 의 0.18–0.31/에피소드로
     올랐다 (기록 8 개 중 7 개가 계약 임시 범위 상한 0.2 초과). held-out 평가(경로 풀 seed 1, 명목 동역학,
     512 env, 끝난 에피소드 2 059·2 095 개)에서 BC → PPO 는 평균 에피소드 길이 28.1 s → 23.4 s (−17 %) 를 얻은 대신
     개입 비용이 0.038 → 0.112/에피소드 (약 3 배) 가 됐다 (리뷰어 재실행 0.040 → 0.130). 성공률 98.8 % → 98.5 %,
     충돌 0 → 0. 시간 이득은 이 개입 증가와 함께 읽어야 한다.
   - **ONNX 메타데이터**: 처음 내보낸 `policy_final.onnx` 의 `algo` 는 "Lagrangian penalty" 라고 적었다 — 틀렸다.
     이제 `export_onnx.py` 가 algo 를 학습 기록에서 만들고(`policy_export.describe_algo`), `lagrange_final`·
     `lagrange_all_zero_from_iteration`·`intervention_budget_d1`·`intervention_budget_in_contract_sweep` 을
     메타데이터에 넣는다. 끝 λ 가 전부 0 인데 algo 가 라그랑주를 주장하면 (`--algo` 로 주어도) 모델 파일을 쓰지 않고
     rc 1 로 끝난다 (`algo_claim_consistent`). `ppo_b_bc_fz` 의 두 ONNX 는 이렇게 다시 내보냈다 (가중치는 같다).
     BC 정책은 `train_steps` = 0 (RL 단계 없음) 과 `bc_samples` 를 적는다 (처음에는 PPO 단계 수를 적었다).
   - **다음 물결**: d₁ ∈ {0.02, 0.05, 0.1, 0.2} 훑기와 마지막 λ 보고 (계약 §7.2), 승수 갱신은 PID Lagrangian 권장.
11. **BC 예열** — 특권 정보 전문가(`amr_isaac/expert.py`, 1.0 m 앞 경로점 순수 추종 + 목표 거리 비례 감속)
   시연으로 actor 를 tanh 공간 MSE 로 맞춘 뒤 PPO 로 미세조정한다. **BC 뒤 정규화 통계를 고정한다** — 고정하지
   않으면 actor 를 얼린 채로도 롤아웃 통계 이동만으로 성공률이 94.6 % → 5.3 % 로 무너졌다.

## Isaac 함정 (이 호스트, Isaac Sim 5.1 / Isaac Lab 2.3.2 / RTX 5090 / Rocky 8.10)

- `kinematic_enabled=True` 강체에 GPU PhysX `set_transforms` → `CUDA error 700 (illegal memory access)`
  `GpuRigidBodyView.cpp:512`. 충돌체가 하나도 없는 장면도 첫 step 에서 CUDA 700 (narrowphase). → 동적 강체
  (중력·충돌 끔) + 바닥 평면. 재현 스크립트 `/home/CAPYI_AI/runs/r1-isaac/probe/probe_kin.py` (변형 A–J).
- `RayCasterCfg.ray_alignment='yaw'` 는 시작점만 돌리고 방향은 세계 고정(높이 스캐너용) → 2D LiDAR 는 `'base'`.
- `RayCaster.data.pos_w` 는 부착 강체 원점이다. `cfg.offset` 은 광선 시작점에만 더해진다.
- 쓴 자세는 같은 step 안에서 RayCaster 가 바로 읽는다 (리셋 직후 원점 오차 7.7e-7 m, smoke 점검).
