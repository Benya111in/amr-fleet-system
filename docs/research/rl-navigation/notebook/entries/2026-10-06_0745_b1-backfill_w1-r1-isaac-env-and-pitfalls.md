# 1차 물결 r1 — Isaac Lab 창고 env 구축, 계약 일치, 처리량, 이 호스트의 Isaac 함정

> **소급 기록** (작성 2026-10-06 19:10 KST 첫 커밋, b1-backfill, 단계 A4). 원 작업: r1-isaac, 2026-10-06 07:42–08:09 KST
> (env·경로 풀·스모크·golden 주입·처리량), 리뷰 반영 09:34–09:50. 학습 결과는 다른 두 항목
> (`…_0805_…pure-ppo-failures.md`, `…_0832_…bc-ppo-lagrangian.md`)에 있다.

- 시각: 2026-10-06 07:45 KST (원 작업) · 에이전트: r1-isaac (기록 b1-backfill) · 단계: W1
- 코드: `feature/rl-isaac-env` f3a1b41 (07:42) · 6b8f774 (07:47) · 42b9528 (08:00) · ef0b609 (08:05) · 972bff6 (08:11); 리뷰 반영 c55ea9d·29eba4d
- 이미지: `amr-ai/isaaclab:2.3.2` (Isaac Sim 5.1, Isaac Lab 2.3.2), RTX 5090
- 관련 실행 id: `W1_r1_smoke_256`, `W1_r1_golden_in_sim_first`, `W1_r1_test0`, `W1_r1_scale_1024`, `W1_r1_scale_2048`,
  `W1_r1_golden_in_sim_f262d91`, `W1_r1_smoke_2048`, `W1_r1_smoke_256_review_fix`, `W1_r1_golden_in_sim_review_fix`

## 질문
명세의 "Isaac Sim 강화학습 환경"으로 계약(v1.1.0)과 비트 수준까지 같은 관측·보상을 내는 창고 내비게이션 env 를 만들 수 있는가,
그리고 그 env 가 PPO 학습에 충분한 처리량을 내는가.

## 당시 가설과 예측
- warehouse.sdf 의 충돌 상자를 변환하면 Gazebo 와 같은 정적 지도를 Isaac 에 그대로 옮길 수 있다.
- Isaac 의 PhysX 강체를 "자세를 써 넣는 그릇"으로 쓰고 운동학은 계약 식으로 직접 적분하면 golden 과 일치시킬 수 있다.
- 2048 env 에서 수만 env-steps/s 가 나온다.

## 설정
- 변환기: warehouse.sdf + include 모델 → 충돌 상자 229 → Warp 메시 / 2D 사각형 129 / 0.05 m 점유 격자. 같은 비용 A\* 경로 풀
  8192 개 (seed 0 학습, seed 1 held-out). 캐시 `/home/CAPYI_AI/runs/r1-isaac/assets` (해시 a9f4d87e62 → 리뷰 반영 뒤 58eaa34ae5,
  배열 동일). 경로 길이 [측정, `paths_58eaa34ae5_s0.json`]: 3.01–30.0 m, 평균 18.2 m.
- env: `AMR-Isaac-Nav-B-v0`·`-Eval-v0` (DirectRLEnv), 행동 tanh → integrate_b → 50 Hz 프로파일러 → safety_lite φ(근사 D) →
  플랜트 지연 → 원호 적분, 풋프린트 SAT 충돌, RayCaster 720 빔, 계약 관측 291. DR: 플랜트 T_m·T_d·가속 배율, LiDAR σ·드롭아웃.

## 결과
| 실행 | 결과 | 표기 |
|---|---|---|
| golden 주입 (첫, 08:08) / f262d91 / 리뷰 반영 29eba4d | decision 17·step 11 fails [] , 최대 오차 관측 5.0e-8, 보상 1.5e-10 (세 번 같음) | [측정] `golden/*.json` |
| smoke 256 env (08:00) | 휴리스틱 945/949 성공, LiDAR 대 해석 레이캐스트 최대 2.3e-5 m, 무작위 6.5k env-steps/s | [측정] `smoke/smoke.json` |
| smoke 2048 env (09:06) | 무작위 55.0k / 휴리스틱 55.4k env-steps/s (env 만), 휴리스틱 1907/1911, LiDAR 최대 3.1e-5 m, p99 7.2e-6 m (n 1646), 리셋 원점 오차 7.8e-7 m | [측정] `smoke/smoke_2048.json`, 공유 부하 하 예비값 (WAVE 보고의 loadavg 36 은 json 에 없음) |
| smoke 256 env 리뷰 반영 (09:50) | 휴리스틱 1204/1214, 무작위 7.8k / 휴리스틱 6.8k env-steps/s, LiDAR 최대 1.6e-5 m | [측정] |
| PPO 처리량 (갱신 포함) | 1024 env 22.8k, 2048 env 42.3k env-steps/s (chunk 마지막) | [측정] `train/scale_*/curve.jsonl` |
| 배관 스모크 test0 | 256 env × 4 반복, 성공 0 (배관 확인용) | [측정] |

Isaac 함정 (이 호스트, Isaac Sim 5.1 / Lab 2.3.2 / RTX 5090 / Rocky 8.10):
- `kinematic_enabled=True` 강체에 GPU PhysX `set_transforms` → CUDA error 700 (`GpuRigidBodyView.cpp:512`); 충돌체 0 개 장면도 첫
  step 에서 CUDA 700 → 동적 강체(중력·충돌 끔) + 바닥 평면으로 우회. **[문서주장]** — 재현 스크립트
  `/home/CAPYI_AI/runs/r1-isaac/probe/probe_kin.py`(변형 A–J)만 있고 출력 로그가 없다 (리뷰 지적, 이 기록도 확인).
- `RayCasterCfg.ray_alignment='yaw'` 는 광선 방향을 돌리지 않는다 → `'base'`. `RayCaster.data.pos_w` 에 `cfg.offset` 이 안 들어간다.
- 한 프로세스에서 env 두 개를 만들면 RecursionError (Isaac Lab 제약, 리뷰 확인) → 스크립트는 env 를 직접 만든다.
- 평가 스크립트가 env seed 를 주지 않아 ("Seed not set") 재실행 때 ±0.3 %p 흔들렸다 → 2b 에서 `eval_unbiased.py` 로 고침.

리뷰(09:14–09:25) 차단 1: env.py 의 핵심 순수 로직(SAT 충돌·접근 거리 D·원호 적분)을 CPU 시험이 덮지 못했고 변이 R4·R5 가
살아남았다 → `env_math.py` 로 떼고 시험 25 건 (변이 17/17).

## 해석
- **계약 일치와 센서 충실도는 강하다** (golden 주입 오차 1e-8 급, LiDAR 해석 대조 1e-5 m 급). Isaac 쪽 RL 결과가 2D 와 다르다면
  관측·보상 구현 차이보다 φ 근사(safety_lite + 근사 D), 경로 생성기, PPO 구현 차이를 먼저 의심할 근거다.
- **φ 가 정본 SweptFootprint 가 아니다** — 이후 Isaac 의 개입 비용(2b 1.66–3.00)이 2D(0.02–0.14)와 비교할 수 없게 된 원인이고,
  완성 계획 A3 (정본 φ 이식)의 이유다.
- 맵은 한 장, 동적 장애물 없음, 지연 DR·트랙 누락 DR 없음, SAC 미연결 — 1차 물결 Isaac 은 정적 단일 맵 환경이다.

## 무엇을 바꾸게 했나
- 1차 물결 PPO 학습(다음 항목)의 기반. 처리량 ~4 만 env-steps/s 로 39M step 이 약 17 분.
- 정본 φ 이식(A3)과 편향 없는 평가(eval_unbiased, 2b) 요구.

## 원자료
- `/home/CAPYI_AI/runs/r1-isaac/{assets,smoke,golden,probe,train/test0,train/scale_1024,train/scale_2048}`
- 리뷰: `/home/CAPYI_AI/logs/agents/r1-isaac/{review_evidence,review_fix,recheck}/`
- 보고: `/home/CAPYI_AI/logs/agents/r1-isaac/WAVE1_RESULT.md`, `rl/isaac/README.md`

## WAVE 보고와 원자료의 차이
- smoke 2048 의 "loadavg 36" 은 원자료 json 에 없다 (출처 미확인 — 공유 부하 수치로만 인용).
- CUDA 700 함정은 WAVE impl 이 [측정] 으로 적었지만 출력 로그가 없어 [문서주장] 이다 (리뷰가 이미 지적).
- (리뷰 반영 중 변이 run 을 등록하며 찾음) WAVE "변이 14 종 가운데 golden 만으로는 10 종" 과 "golden 만으로는 M2·M4·M8 이 통과한다
  (r1 mutation_result.json)" — 그 파일에는 golden + 무작위 결합 실행(13/14) 하나만 있다. golden 만의 결과는 원자료에 없다 → [문서주장].
  또 그 결합 실행(08:14:26 이전)은 `test_contract_random.py` 가 커밋(b062a1a 08:16:51)되기 전이다 — 미커밋 시험으로 돌았다.
- 그 밖의 수치(golden 오차, smoke 처리량, 휴리스틱 성공, PPO 처리량 22.8k/42.3k)는 원자료와 같다.

## 실행 목록에 등록한 run (`runs/b1-backfill.jsonl`, entry = 이 파일)

| run_id | 종류 | 시뮬 | 커밋 | 시드 | 표기 |
|---|---|---|---|---|---|
| `W1_r1_test0` | probe | isaac | unknown-noHEAD | 0 | [측정] |
| `W1_r1_scale_1024` | probe | isaac | unknown-noHEAD | 0 | [측정] |
| `W1_r1_scale_2048` | probe | isaac | unknown-noHEAD | 0 | [측정] |
| `W1_r1_smoke_2048` | probe | isaac | unknown-noHEAD | n/a | [측정] |
| `W1_r1_smoke_256` | probe | isaac | unknown-noHEAD | n/a | [측정] |
| `W1_r1_smoke_256_review_fix` | probe | isaac | 29eba4d | n/a | [측정] |
| `W1_r1_golden_in_sim_first` | test | isaac | unknown-noHEAD | n/a | [측정] |
| `W1_r1_golden_in_sim_f262d91` | test | isaac | f262d91 | n/a | [측정] |
| `W1_r1_golden_in_sim_review_fix` | test | isaac | 29eba4d | n/a | [측정] |

## 리뷰 반영으로 더 등록한 run (변이 검사·학습 스모크)
A4 리뷰 비차단 6("등록하지 않은 원자료")을 반영해 소급 등록했다 (`runs/b1-backfill.jsonl`, `revision` 필드).
커밋이 `unknown-noHEAD` 인 것은 config 에 추정 근거를 적었다. 시각이 파일 mtime 인 것은 상·하한이다.

| run_id | 종류 | 시뮬 | 커밋 | dirty | 결과 [측정] |
|---|---|---|---|---|---|
| `W1_r1_mutation_contract_torch` | test | offline | unknown-noHEAD | true | 13/14 잡힘 — 생존: M10 곡률 모드 이탈 재설정 없음 (등가 변이 — 리뷰 동의) |
| `W1_r1_review_mutation_f8feb56` | test | offline | f8feb56 | unknown | 4/8 잡힘 — 생존: R4 footprint_hits 장애물 축; R5 approach_distance 안쪽 점; R6 충돌 시 정지 (등가); R8 시작 여유 필터 |
| `W1_r1_fix_mutation_c55ea9d` | test | offline | c55ea9d | true | 17/17 잡힘 |
| `W1_r1_fix_mutation2_f9aff27` | test | offline | f9aff27 | false | 5/5 잡힘 |
| `W1_r1_fix_mutation3_29eba4d` | test | offline | 29eba4d | false | 3/4 잡힘 — 생존: R8c 목표 통행 가능 조건 제거 (등가 주장: GOAL_CLEARANCE 0.40 > INSCRIBED 0.20 — [문서주장]) |
| `W1_r1_recheck_mutation_1afe9dc` | test | offline | 1afe9dc | unknown | 12/13 잡힘 — 생존: N11 env.py 접촉 래치 반영 누락 (copy_ 제거) — Isaac 접착 코드, CPU 시험 밖 |
