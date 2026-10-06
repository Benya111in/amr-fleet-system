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
| `amr_isaac/env_math.py` | env 의 순수 함수 — 풋프린트 분리축 충돌 `footprint_hits`, safety_lite 접근 거리 `approach_distance`, 원호 적분 `integrate_pose`, 서브스텝 이동·접촉 `move_and_collide`, 경로 길이 필터 `select_paths_by_length`, 보상 판 `reward_spec_for`, 에피소드 기록 `EpisodeLog`, 정본 φ 결선 `CanonicalPhi` | 아니오 |
| `amr_isaac/safety_gate_torch.py` | **정본 safety φ** — C++ `amr_perception::SafetyGate` 상태기계 전체의 배치 torch 이식 (A3, 아래 「정본 safety φ」) | 아니오 |
| `amr_isaac/eval_metrics.py` | 층위 성공 지표·목표 직전 정지 진단·dwell·목표 여유 기하 (평가·분석이 쓴다) | 아니오 |
| `amr_isaac/eval_sets.py` | B 단계 평가 집합: 끝 접선 자세의 직진 여유(배치), 주(≥ 0.40 m)/스트레스 분할·표본, 목표 군집 부트스트랩 구간 | 아니오 |
| `amr_isaac/perturb.py` | DR 축 목록(명목값·학습 범위)과 교란 묶음 파일 검증·조건별 값 | 아니오 |
| `amr_isaac/sac_skrl.py` | SAC (skrl 2.1.0): tanh 짓눌린 가우시안 정책·Q(s, tanh(raw))·계약 정규화·에이전트 조립·내보내기 상태 | 아니오 (skrl) |
| `amr_isaac/pid_lagrange.py`, `ppo_lag.py` | 제약 PPO: PID 승수·비용 GAE·이익 결합 (순수) / rsl_rl PPO 상속 러너 `PPOPIDLag` (비용 critic) | ppo_lag 은 rsl_rl |
| `eval_sets/` | **결과 전에 고정한** 평가 집합 `b1_eval_sets_v1.json` 과 교란 묶음 `b1_perturb_suite_v1.json` (바꾸면 v2 로 새 파일) | — |
| `tools/phi_crosscheck/` | C++ 원본 SafetyGate 와 torch 이식의 교차 시험 (`crosscheck.sh`, `gate_runner.cpp`, `crosscheck.py`) 과 변이 검사 (`mutate.py`), env 안 교차 시험 재생 (`replay_env.{sh,py}` — `record_phi_wiring.py` 기록을 불변식 검사·C++ 재생) 과 결선 변이 (`mutate_wiring.py`) | 아니오 |
| `amr_isaac/env_cfg.py`, `env.py`, `tasks.py` | `DirectRLEnv` — `AMR-Isaac-Nav-B-v0` (학습), `AMR-Isaac-Nav-B-Eval-v0` (held-out 경로·명목 동역학) | 예 |
| `amr_isaac/agents/rsl_rl_ppo_cfg.py` | rsl_rl PPO 설정 (γ 0.995, λ 0.97, rollout 64) | 예 |
| `amr_isaac/expert.py` | BC 예열용 특권 정보 전문가 (경로 순수 추종) | 아니오 |
| `amr_isaac/policy_export.py` | rsl_rl state dict → 계약형 `PolicyB` → ONNX (`onnx_io.md`) | 아니오 |
| `scripts/` | `build_assets` · `smoke_env` · `check_golden_in_sim` · `train_ppo` (`--phi`·`--dr`·`--constraint` PID) · `train_sac` · `eval_unbiased`(판정용, `--phi`·`--dwell_k`·층위 지표·진단·`--eval_set`·`--perturb`) · `aggregate_seeds`(시드 집계) · `build_eval_sets` · `eval_policy`(1차 물결, 편향 있음) · `export_onnx`(PPO·SAC·PID) · `plot_curves` · `gpu_monitor.sh` · `analyze_rejudge`(A3b 분석) · `record_phi_wiring`(정본 φ 결선 기록, A3d) | 일부 |
| `tests/` | golden 32 대조(직접·env 캐시 두 길), 기준 구현과 무작위 차분(v1.2.0·v1.1.0), v1.2.0 보상 손 사례(`test_reward_v12.py`), 창고 변환기·경로 풀, env 순수 함수(`test_env_math.py`), ONNX 메타데이터, 정본 φ (`test_safety_gate_torch.py` — 설정 원문 대조·원본 손 사례(해제 거리·지연 보정 포함)·창 통계 순차 기준판·작은 C++ 교차 시험(`AMR_ISAAC_REQUIRE_CPP=1` 이면 필수)·env 결선(명세 게이트 순차 대조·리셋 = 새 게이트)·평가 지표(`nav2_update`)·결선 변이 대상) | 아니오 |
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

## 정본 safety φ (A3, 2026-10-06, b1-isaac-phi)

근거·원자료: 노트 `docs/research/rl-navigation/notebook/entries/2026-10-06_19{00,01,02}_b1-isaac-phi_*.md`, 실행 목록
`notebook/runs/b1-isaac-phi.jsonl`, 원자료 `/home/CAPYI_AI/runs/bonus1/b1-isaac-phi/`.

**불리한 것 먼저.**
- **원본 C++ 의 창 통계가 문서와 다른 값을 낸다.** `safety_gate.cpp:255` 의 `values[i] >= best` 가지치기는 결과를 바꾼다
  (창의 k 번째 값은 중심 값보다 작을 수 있다) — 배포 safety_node 의 D·가드 거리는 헤더 정의("창 통계의 최소")보다 크게
  나올 수 있고 빔 순서에 따라 다르다 [측정: 탐침 300 개 중 20 개, 교차 시험의 **합성 무작위 폐루프 시나리오** 틱의 5.5 % —
실제 주행 빈도가 아니다; 리뷰의 독립 C++ 탐침은 합성 스캔 꼴 배열 2 000 개 중 500 개에서 원본 값이 더 크고 차이 중앙 3.5 mm ·
p90 11 mm · 최대 34.5 mm; **Isaac 실제 주행** (A3d 그림자 게이트, 960 000 틱): LiDAR D 가 유한한 틱의 약 20 % 에서 원본 D 가
가지치기 없는 값보다 크고 (중앙 5 mm · p90 17 mm · 최대 0.44 m), 가지치기 없는 정의라면 STOP 래치였을 틱이 318 개 (반대 0)]. 이식은
**원본 동작 그대로**다
  (safety_node 가 실제로 적용하는 φ 가 목표라서). 결함 판단·수정은 amr_perception 소유자 몫이다.
- **Isaac 의 입력 모델은 safety_node 의 입력과 같지 않다 — 낙관 쪽이다.** 깊이 점군은 "신선하고 비어 있음" 으로 둔다 (Isaac 에
  깊이 카메라가 없다 — 창고 2D 장애물 상자 129 개가 모두 LiDAR 평면(지면 +0.20 m)을 지나 LiDAR 가 못 보는 낮은 물체는 없다
  [시험 `test_warehouse_paths.py::test_all_2d_obstacles_cross_lidar_plane`]. 그러나 배포 점군 채널은 같은 면에서 3 번째로 가까운 점을
  쓰므로 LiDAR 창 통계(60 % 순위)보다 일찍 멈출 수 있고, 이것은 목표 직전 정지에 직접 영향을 준다: **미측정** — Gazebo 하네스가
  safety_node `/diagnostics` 의 `lidar_distance_m`·`depth_distance_m` 을 기록해 재도록 요청했다). 트랙(TTC)·도킹 예외는 정적 지도라 넣지 않는다. E-stop 래치·
  센서 생존 감시는 옮기지 않았다.
- **정본 φ 로 바꿔도 충돌은 줄지 않았다** (r7 정책, 3–15 m 합 3 → 5 건 / 6144), 그리고 근접 정지도 남는다 (아래).
- 계산이 무겁다: 평가 결정 스텝당 약 99 ms 대 근사 23 ms (2048 env, 공유 부하 하 예비값).

**무엇을 옮겼나** (`safety_gate_torch.py` 머리말에 원본 행): 스윕 풋프린트 접근 거리(직진·원호·끝 자세 접선 직선, 옆 여유,
영역 길이), 운동 가설(명령 · 신선한 측정 · 정지 중 의도 유지), 측정 속도 저역 통과·지연 보정, 자기 차체 필터, 빔 창 순위 통계
(4 cm·5 빔·60 %, 가드 6 cm·9 빔·중앙값, 원형/비원형, 원본 가지치기 포함), 점군 채널, STOP 확정(2 프레임 또는 0.25 m)·원인별
해제(0.50 m)·존 히스테리시스(0.05 m), 존·여유 거리·도킹 예외·낡은 점군·TTC 상한, 곡률 보존 스케일, 접촉 가드 탈출, TTC 후퇴·
몸 원통 양보 탈출(배포 설정 꺼짐), 명령 낡음. 매개변수 기본값 = safety_node 배포 설정 (시험이 config/robot_params.yaml·
perception.yaml·safety_gate.hpp·safety_node.cpp 원문과 대조 — hpp 기본과 다른 것은 yield_escape 끔·점군 필수 둘).

**env 결선** (`env_math.CanonicalPhi`): 결정 스텝마다 새 스캔(촬영 시각 = 결정 시작, base_link 빔 = 원점 + r(cos, sin),
무효 빔 NaN), 50 Hz 서브스텝마다 측정(플랜트 속도) → 명령(프로파일러 출력) → 판정. STOP 래치는 관측의 근접 정지 비트로 나간다.
리셋 = 새 게이트. lite 길은 그대로 남아 r7 평가를 비트 단위로 재현한다 [측정: 6 평가 × 14 배열].

**env 안 교차 시험과 결선 변이 (A3d)** [측정, 노트 `2026-10-06_2129_b1-isaac-phi_env-wiring-check.md`]: Isaac 폐루프(DR 켬, A3c 정책 확률
행동 + 무작위 행동)에서 게이트 입력의 독립 출처(env.scan, 플랜트 속도, `_pairs`, 리셋, 관측 비트)를 기록하고, 에피소드마다 새 C++
SafetyGate 에 명세대로 재구성한 입력을 재생 — 960 000 틱 · 875 에피소드 · 1 920 만 비교에서 **불일치 0**, 결선 불변식(스캔 시각 = 0.1·k,
측정 = 플랜트 속도, 리셋 = 새 게이트, 관측 래치 비트 = 게이트 래치) **위반 0**. 결선 변이 E1–E4 는 이 검사가, E2–E5·G3·G4 는 CPU 시험
(g++ 없이)이 잡는다. pytest 는 `AMR_ISAAC_REQUIRE_CPP=1` 로 돌리면 g++·Eigen 이 없을 때 C++ 교차 시험이 skip 대신 실패한다.
변이 계수 때 메타 시험 `test_wiring_mutants_target_current_source` (어떤 변이든 실패) 는 뺀다.

```bash
# 결선 검사 (B 단계 학습 스냅숏마다 한 번, 약 2 분): Isaac 기록 → amr-ai/rl:0.3 에서 C++ 재생 대조 (rc 0 = 통과)
rl/isaac/run_isaac.sh wire rl/isaac/scripts/record_phi_wiring.py --policy <policy.pt> --num_envs 64 --steps 600 --out <dir>
docker run --rm -v $WT:/ws:ro -v <dir>:<dir> -w /ws amr-ai/rl:0.3 bash -lc 'rl/isaac/tools/phi_crosscheck/replay_env.sh <dir> --out <dir>/replay.json'
```

**교차 시험** (`tools/phi_crosscheck/crosscheck.sh`, C++ 원본을 ROS 없이 직접 빌드) [측정]: 독립 seed 두 묶음 4 938 400 · 5 938 400
비교 — 점별 접근 거리 20 만, 창 통계 2 만 + k 번째 1 만, 경계 시나리오 1 794 × 설정 3 종(직진 = 사칙연산이라 문턱 ±ulp 를 비트 단위로),
무작위 폐루프 3 000 · 4 000 × 50 틱(통로·정면 벽·잡동사니·모서리·열린 곳, 점군·트랙·예외 다각형·명령 끊김), CUDA 대 CPU —
**이산 불일치 0, 연속 최대 차 2.2e-12**. 변이 28 개 중 25 개 잡힘, 남은 3 개는 동치(옆 여유 규칙 = swept_margin 0, 창 순위 −1e-9 =
0.6·len 반올림, 새 래치의 원인 지우기 = 불변식).

```bash
# 교차 시험 (amr-ai/rl:0.3)
docker run --rm -v $WT:/ws -w /ws amr-ai/rl:0.3 bash -lc 'rl/isaac/tools/phi_crosscheck/crosscheck.sh --cuda_check --out /tmp/x.json'
# 정본 φ 로 평가·학습
rl/isaac/run_isaac.sh ev rl/isaac/scripts/eval_unbiased.py --policy <pt> --phi canonical --path_len 3:15 --out <dir>
rl/isaac/run_isaac.sh tr rl/isaac/scripts/train_ppo.py --phi canonical ... (나머지는 r7 과 같음)
```

**r7 정책 재판정** (학습은 근사 φ, 평가만 바꿈; held-out 3–15 m, seed 4242, n 2048 첫 에피소드) [측정]:

| 시드 | 엄격 성공 근사 → 정본 | Nav2 호환 0.25 m (근사 / 정본) | 개입 /에피소드 근사 → 정본 | 충돌 근사 / 정본 |
|---|---|---|---|---|
| 0 | 0.9790 → 0.9834 | 0.9985 / 0.9985 | 2.84 → 0.65 | 1 / 2 |
| 1 | 0.9819 → 0.9888 | 0.9966 / 0.9976 | 1.66 → 0.74 | 0 / 0 |
| 2 | 0.9834 → 0.9922 | 0.9946 / 0.9971 | 3.00 → 0.73 | 2 / 3 |

- 목표 직전 정지의 원인: 근사 φ 에서 근접 실패의 94 % 가 φ 차단(정책은 계속 밀고 φ 가 깎는다) 이고 그중 40 % 는 근사의 외접원
  회전 규칙에 막힌 제자리 회전이다. 실패는 **좁은 목표**에 몰린다: held-out 첫 에피소드의 2.3 % (47/2048) 는 경로 끝 접선 자세에서
  앞 끝 직진 여유가 0.30 m 미만이라 **직진으로** 들어오면 두 φ 모두 여유 상한 0 이다 (근사 φ 실패의 67–73 %).
- **정정 (리뷰 반영, 2026-10-06 21:40)**: 처음 판은 "φ 종류는 주원인 아님 / 실패의 주원인은 목표의 기하" 로 적었는데 틀렸다.
  사전 규칙(±1 %p·2 %p)은 근사 실패율 1.66–2.10 %p 때문에 천장에 걸린 규칙이었다. 짝 비교로 정본 φ 는 근사 실패를 43→34 · 37→23 ·
  34→16 (합 41/114 = **36 %**) 줄이고, 그 이득은 거의 전부 좁은 목표에서 난다 (좁은 +3 · +12 · +18, 넓은 +6 · +2 · 0). 좁은 목표의
  "근사 실패 → 정본 성공" 37 건은 근사 쪽에서 회전 차단 16 · 원호 차단 17 이었다. **실패는 좁은 목표에 몰리고, 그 실패의 상당 부분은
  근사 φ 의 회전·원호 차단 때문이다. 정본 φ 아래에서도 좁은 목표는 도달할 수 있고 정책에 따라 크게 다르다** (r7 21 · 32 · 41/47,
  A3c 37/47; 직진 여유 < 0.10 m 인 7 개도 r7 s2 6/7). 성공한 좁은 목표는 접선에서 yaw 가 중앙 40–54° 벗어난 자세로 끝난다 —
  직진 진입만 막힌다. 근거: 노트 `2026-10-06_1901_b1-isaac-phi_rejudge-r7.md` 「리뷰 반영」.

**정본 φ 로 다시 학습한 1 시드** (`A3c_v12_canon_s0`: r7 G1 과 같은 설정 — 2048 env × 300 반복 = 39.3M, DR 켬, 3–15 m, 시드 0,
`--phi canonical`) [측정]: 판정(마지막 반복, 정본 φ, 3–15 m) **0.9937 (2035/2048), 충돌 0** — 통과(≥ 0.90). 3–30 m 0.9883.
좁은 목표 37/47·넓은 목표 1998/2001. **대가**: 개입 1.33/에피소드 (성공 에피소드 0.19) — 제약 없는 PPO 라 φ 를 브레이크로 쓴다.
같은 정책을 근사 φ 로 평가하면 0.9800 (막힌 정지의 대부분이 근사가 막는 제자리 회전). B 단계를 정본 φ 로 학습하는 근거는
**배포 φ(safety_node) = 정본** 이라는 것이다 — "학습과 평가의 φ 가 같아야 한다" 는 한쪽 방향만 뒷받침된다 (r7 정책은 근사로
학습했는데 정본 평가에서 오히려 좋아졌다). 1 시드라 "정본 φ 아래 학습된다" 의 근거는 아니다 (B1 에서 시드 ≥ 10).
학습 벽시계 (같은 기준, 공유 부하 하 예비값): 프로세스 시작~끝 A3c 2 709 s 대 r7 1 711–1 963 s, `train_wall_s` 2 694 대 1 695–1 945 s.

## B 단계 실행자용 (b1-binfra, 2026-10-07) — 코드 수정 없이 돌리는 명령

근거·사전 등록: 노트 `docs/research/rl-navigation/notebook/entries/2026-10-06_2353_b1-binfra_eval-sets.md` (평가 집합),
`…_2354_b1-binfra_perturb-suite.md` (DR 축·교란 묶음), `…_2358_b1-binfra_infra-smokes.md` (스모크). 판정 규칙은 완성 계획 §2·§4·§6 과
A3c 항목(마지막 반복, 정본 3–15, 평가 seed 4242, 통과 ≥ 0.90)을 따르고, **각자 자기 노트 항목에 사전 등록한 뒤** 돌린다.

**불리한 것 먼저.**
- 스트레스 집합은 3–15 m 경로 83 개·목표 28 개, 3–30 m 경로 244 개·목표 32 개를 되풀이한 n 512 다 — 이항 구간은 너무 좁다.
  **목표 군집 부트스트랩 구간**으로 주장한다 (A3c 스모크: Wilson [0.789, 0.855] 대 군집 [0.679, 0.958]).
- 맵 다양성(계약 §11 의 1 순위)은 학습에도 교란 묶음에도 없다 — D3 는 동역학·센서·지연 축의 일반화만 말한다.
- SAC 하이퍼파라미터의 대부분은 [설계 선택]이다 (아래 표). PID 이득도 [설계 선택]이다 — 스모크는 λ 가 예산에 반응하는지만 봤다.
- 공유 GPU 에서 처리량은 부하에 크게 달렸다 (아래 시간은 공유 부하 하 예비값).

### 0. 준비 (실행자마다 한 번)

```bash
ID=b1-<id>; WT=/home/CAPYI_AI/wt/$ID; R=/home/CAPYI_AI/runs/bonus1/$ID
C=$(git -C $WT rev-parse --short HEAD); mkdir -p $R/src/$C $R/assets
git -C $WT archive $C | tar -x -C $R/src/$C          # 실행은 커밋 사본에서 (worktree 수정과 섞이지 않게)
for f in /home/CAPYI_AI/runs/bonus1/b1-binfra/assets/*; do \cp -f $f $R/assets/; done
(cd $R/assets && sha256sum -c SHA256SUMS.txt)      # 경로 풀 해시 = b1-ppo10·b1-isaac-phi 와 같다
# Isaac 실행 공통 (슬롯 4 개 공유, GPU 여유 ≥ 16 GB 확인은 run_isaac.sh 가 한다)
isaac() { AMR_AI_ID=$ID AMR_RUNS_ROOT=$R EXTRA_DOCKER_ARGS="${EXTRA:-}" \
          /home/CAPYI_AI/env/isaac_slot.sh $ID $R/src/$C/rl/isaac/run_isaac.sh "$@"; }
ES=/workspace/amr/rl/isaac/eval_sets                 # 컨테이너 안 경로 (worktree 사본이 /workspace/amr)
```

다른 실행자의 정책을 평가할 때는 그 runs 디렉터리를 읽기 전용으로 더한다:
`EXTRA="-v /home/CAPYI_AI/runs/bonus1/b1-ppo10:/home/CAPYI_AI/runs/bonus1/b1-ppo10:ro"`.
실행마다 `HEAD.txt`(커밋)·시작/끝 시각·loadavg 를 산출 디렉터리에 남기고 `python3 rl/tools/notebook.py register` 로 등록한다.

### 1. 평가 (모든 정책 공통) — 집합 4 개, 한 프로세스

```bash
isaac ev_s0 rl/isaac/scripts/eval_unbiased.py --policy <run>/policy_final.pt --phi canonical \
    --eval_set $ES/b1_eval_sets_v1.json --out <run>/eval_sets_final
```
- 행 = main_3_15·stress_3_15·main_3_30·stress_3_30 (2 048 + 512 + 2 048 + 512 = 5 120 env, 한 라운드). env 마다 첫 에피소드,
  경로 고정, DR 끔(명목값), seed 4242, 결정론 tanh(μ). rc 0 = unfinished 0.
- 결과 `eval.json` 의 `by.<집합>`: 층위 지표(`layered`: 엄격·Nav2 호환·최종 거리 분포·개입), `intervals` (Wilson 95 %·**목표 군집
  부트스트랩 95 %**·충돌 수·서로 다른 목표/경로), 정지 진단. `episodes.npz` 에 `set_index`·`goal_group` 이 있다.
- 시간 [측정, 공유 부하 하 예비값 loadavg 27]: A3c 정책 5 120 env, 507 결정 스텝, 330 s + 시작 약 60 s. torch 최대 할당 2.2 GB.
- PPO·PPO-PID·SAC 정책 모두 같은 명령이다 (`policy_export.load_policy` 가 체크포인트 형식을 본다).

### 2. 교란 묶음 (B4 의 D3 평가)

```bash
isaac pert_s0 rl/isaac/scripts/eval_unbiased.py --policy <run>/policy_final.pt --phi canonical \
    --eval_set $ES/b1_eval_sets_v1.json --sets perturb_3_15 --perturb $ES/b1_perturb_suite_v1.json \
    --out <run>/eval_perturb_final
```
- 시간 [측정, A3c, 공유 부하 하 예비값 loadavg 9–15]: 두 라운드 359 s + 469 s, 전체 851 s + 시작 약 60 s, torch 최대 할당 3.6 GB.
- 15 조건 × 1 024 = 15 360 행을 `--max_envs 8192` 씩 두 라운드로 돈다. 키 = `perturb_3_15|<조건>`. 라운드마다 `dr_snapshot`(실제로 넣은 값의
  최소·최대)이 기록된다.
- 묶음 정의와 "학습 때 보지 않은" 규칙: `eval_sets/b1_perturb_suite_v1.json`, `amr_isaac/perturb.py`. 관측 지연은 정책 관측의 스캔만 늦춘다.

### 3. 학습 명령 (모두 정본 φ, 계약·보상 v1.2.0, 3–15 m 학습 경로, 경로 풀 전체 = 좁은 목표 포함)

예산은 COORDINATION §8.3 (2026-10-07 01:15) 이 정한다: **B2·B3 는 160 반복 = 20 971 520 env-step** 이고 비교 기준은 b1-ppo10 각 시드의
`policy_it160.pt` (같은 스텝), **B4 의 DR 끔은 300 반복 = 39.3M** (DR 켬 팔 = b1-ppo10 의 최종 정책). GPU 가 정본 φ run 하나로 거의 포화라 동시
실행은 총 처리량을 늘리지 않는다 [측정, §8.3: 혼자 약 14 600, 3 개 동시 각 약 4 050 env-step/s].

```bash
# B1 PPO (지금 코드 그대로 — b1-ppo10 의 명령과 같다)
isaac ppo_s$S rl/isaac/scripts/train_ppo.py --num_envs 2048 --iterations 300 --run B1_ppo_s$S --seed $S \
    --reward 1.2.0 --path_len 3:15 --phi canonical --save_at_env_steps 20e6 --root $R/train
# B2 제약 PPO (PID 라그랑주, 개입 예산 d ∈ {0.2, 0.5, 1.0}) — 이득은 기본값 그대로 (바꾸면 사전 등록)
isaac pid_d${D}_s$S rl/isaac/scripts/train_ppo.py --num_envs 2048 --iterations 160 --run B2_pid_d${D}_s$S \
    --seed $S --reward 1.2.0 --path_len 3:15 --phi canonical --constraint intervention --cost_budget $D \
    --pid 0.5,0.05,0.1 --lambda_max 10 --pid_d_delay 5 --pid_ema 0.0,0.5 --root $R/train
#    선택지 (기본 꺼짐, 쓰면 사전 등록에 적는다): --pid_err_clip=-1,1 (음수로 시작하는 값은 반드시 '=' 꼴 — 띄어 쓰면
#    argparse 가 옵션으로 읽어 실패한다), --constraint_start N (처음 N 반복 λ = 0). 근거는 아래 「PID 스모크」.
# B3 SAC (env-step = PPO it160 과 같은 저울 20 971 520, §8.3)
isaac sac_s$S rl/isaac/scripts/train_sac.py --num_envs 1024 --env_steps 20971520 --run B3_sac_s$S --seed $S \
    --phi canonical --path_len 3:15 --dr on --save_at_env_steps 10e6 --root $R/train
# B4 DR 없음 (DR 있음 팔 = b1-ppo10 의 같은 시드 — 나머지 인자는 B1 과 같다)
isaac droff_s$S rl/isaac/scripts/train_ppo.py --num_envs 2048 --iterations 300 --run B4_droff_s$S --seed $S \
    --reward 1.2.0 --path_len 3:15 --phi canonical --dr off --save_at_env_steps 20e6 --root $R/train
```
산출 `<root>/<run>/`: `curve.jsonl` (chunk 마다 롤아웃 결과율·비용·처리량), `summary.json`, `policy_final.pt`, `dr_snapshot.json`
(첫 리셋 뒤 env 별 DR 값 — `--dr off` 면 모두 명목값), PID 는 `lagrange_history.jsonl` (반복마다 J·λ·P·I·D)·`cost_critic.pt`,
SAC 는 `agent_final.pt` (critic·target·최적화기·α).

### 4. ONNX (amr-ai/rl:0.3, CPU)

```bash
docker run --rm -v $R/src/$C:/ws:ro -v $R:$R -w /ws amr-ai/rl:0.3 python3 rl/isaac/scripts/export_onnx.py \
    --run <run> --obs_batch <run>/eval_sets_final/obs_batch.npz
```
입출력 = onnx_io.md (obs [B,291] → action·mu·log_std [B,2]), opset 17. 메타데이터 `algo` 는 학습 기록에서 만든다: SAC 는
"SAC (skrl 2.1.0, …), unconstrained", PID 는 "PPO-PIDLagrangian (…, cost intervention <= d …, final lambda …)" 와 `constraint`.
끝 λ 가 0 이면 "constraint not binding at end, effectively unconstrained at end" 를 붙인다 (`algo_claim_consistent`).

### 5. 시드 집계 (amr-ai/rl:0.3)

```bash
docker run --rm -v $R/src/$C:/ws:ro -v /home/CAPYI_AI/runs/bonus1:/home/CAPYI_AI/runs/bonus1 -w /ws amr-ai/rl:0.3 \
    python3 rl/isaac/scripts/aggregate_seeds.py --arm "on=<DR 켬 정책들의 eval_perturb_final/eval.json glob>" \
    --arm "off=$R/train/B4_droff_s*/eval_perturb_final/eval.json" --out $R/analysis/agg.json --md $R/analysis/agg.md
```
키(집합 또는 집합|조건)마다 시드별 엄격 성공, 평균·IQM 과 시드 부트스트랩 95 % 구간 (`amr_rl_eval.aggregate`), 실패 시드(< 0.90) 수와
Wilson 95 % (`amr_rl_eval.binomial`), 팔이 둘이면 평균 차 구간. 교란이면 `unseen_single_axis` (한 축 unseen 12 조건 평균).
평가 조건(풀 해시·평가 seed·집합·조건·φ)이 섞이면 rc 1.

### 6. SAC 하이퍼파라미터 (`amr_isaac/sac_skrl.py` SAC_DEFAULTS)

| 값 | 근거 |
|---|---|
| γ 0.995, τ 0.005, 목표 엔트로피 −2 (−dim A) | 계약 §9 (고정) |
| lr 3e-4 (정책·Q·α 같음), 버퍼 10⁶ 전이, 쌍 Q + 목표망, 자동 엔트로피 | Haarnoja et al. 2018 (SAC 응용편) 표 1 [문서주장 — 원문 이 세션에서 재확인 안 함]; skrl 기본 lr 은 1e-3 |
| tanh 짓눌린 가우시안 + log π 보정, Q(s, tanh(raw)), log σ ∈ [−5, 2] | Haarnoja 2018 부록 C (보정식); 범위는 [설계 선택] |
| MLP 512-256-128 ELU (정책·Q 모두) | PPO 와 같은 용량 [설계 선택] |
| 1024 env, 배치 8 192, env 스텝당 갱신 4 (재생 비율 32), learning_starts 50 env 스텝 | 대규모 병렬 off-policy 의 큰 배치 관행 [설계 선택 — 수치 근거 없음] |
| α₀ 0.05 | 스텝 보상(시간 비용 0.02)과 같은 자릿수 [설계 선택]; skrl 기본 0.2 |
| 관측 정규화 = 계약식, 롤아웃 관측으로만 갱신 | onnx_io.md (PPO 와 같은 식) |
| 잘림 전이 = 현재 관측으로 bootstrap | Isaac 자동 리셋이 마지막 관측을 주지 않아서 (rsl_rl 의 시간 초과 처리와 같은 근사) |

### 7. 시험

```bash
docker run --rm -v $WT:/ws -v /home/CAPYI_AI/runs/bonus1/b1-binfra:/home/CAPYI_AI/runs/bonus1/b1-binfra:ro \
    -w /ws/rl/isaac amr-ai/rl:0.3 python3 -m pytest -q tests          # rsl_rl 이 필요한 test_ppo_lag 는 건너뜀
docker run --rm -v $WT:/workspace/amr --entrypoint bash amr-ai/isaaclab:2.3.2 -lc \
    'cd /workspace/isaaclab && ./isaaclab.sh -p -m pytest -q /workspace/amr/rl/isaac/tests/test_ppo_lag.py'
```
- 시험 파일 머리에서 `pytest.importorskip` 를 쓰지 않는다 — amr-ai/rl:0.3 의 pytest 에서 (원인 미확인) 함께 모은 **다른 파일의 시험까지
  전부 건너뛰었다** (`1 skipped`). 모듈 수준 건너뛰기는 `pytestmark = pytest.mark.skipif(...)` 로.
- 변이 검사: `tools/mutate_binfra.py` (대조 사본 통과가 먼저, cpu 27 · isaac 5 변이).

## 전이 한 번 (Δt = 0.1 s = 물리 0.02 s × 5)

raw 행동 → `u = tanh(raw)` → `integrate_b` (ray 클램프) → 결정 주기 안 선형 명령 램프 → [50 Hz × 5]
프로파일러 기준 경로 → safety φ (`--phi lite` 근사 또는 `canonical` 정본) → 플랜트(순수 지연 + 1 차 지연 + 가속 한계) → 정확 원호 적분 →
풋프린트 대 정적 사각형 분리축 충돌 판정 (접촉 = 그 자리 정지·종단) → Warp `RayCaster` 720 빔 → 계약 관측.
버퍼에는 rsl_rl 이 낸 원출력(raw)과 그 log-prob 가 들어간다 (contract §5). 결정론적 배포 행동 = `tanh(mu)` = ONNX `action`.

## 계약과 다르게, 또는 계약이 정하지 않아 여기서 정한 것

1. **safety_lite 의 접근 거리 D (`--phi lite`, 기본 — r7 정책이 이것으로 학습됐다)** — 정본 φ 는 `--phi canonical`
   (아래 「정본 safety φ」). 계약은 D 를 입력으로 받는다(§7.4). lite 는 정본 `SweptFootprint` 대신
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
