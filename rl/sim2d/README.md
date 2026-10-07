# rl/sim2d — GPU 배치 2D 레이캐스트 시뮬레이터 (보너스 1 주 학습장)

계약 [`rl/contract/contract.md`](../contract/contract.md) v1.2.0 의 관측 291·행동(B 연속 / C 격자 100)·보상·비용 5·
종료를 팀 지도 위에서 수천 env 병렬로 돌리는 torch 시뮬레이터. 근거: 「실무」 54–57행(2D 주 학습장), 80–100행(DR),
「이론」 261–317행(DR·SRCC·접촉 = 즉시 종료). 소유: r2-sim2d (`/home/CAPYI_AI/COORDINATION.md` §2).

수치 표기: **[측정]** 명령·로그 있음 / **[계산]** 오프라인 / **미측정**. 이 서버는 공유 서버다 — 처리량은 전부
**공유 부하 하 예비값**이다 (아래 표의 loadavg 는 22–44, 32 코어. 그 전 `35ce14e` 측정 때는 90–136).

## 구조

| 파일 | 내용 |
|---|---|
| `amr_sim2d/contract_torch.py` | 계약 함수의 env 축 배치판: 관측·마스크(C)·보상·비용·이탈 상태기계·safety_lite·프로파일러 기준 모델·행동 적분·BC 레이블 |
| `amr_sim2d/kernels.py` | 뜨거운 경로의 순수 함수 (서브스텝·플랜트·safety 틱·전이 끝) — `torch.compile` 로 묶는다 |
| `amr_sim2d/env.py` | `Sim2DEnv`: 한 결정 스텝 = 50 Hz 서브스텝 5 회, 자동 리셋, DR, 커리큘럼 손잡이 |
| `amr_sim2d/raycast.py` | 720 빔 LiDAR: 격자 DDA(Triton 커널 + torch 기준) + 원·회전 사각형 장애물 해석해 |
| `amr_sim2d/geometry2d.py` | 풋프린트 SAT 충돌 (격자 셀·원·사각형), safety_gate `SweptFootprint` 접근 거리 D 이식 |
| `amr_sim2d/gridmap.py` | `maps/warehouse.yaml` 읽기 (점유 ∪ 미지 = 막힘, 지도 밖 = 막힘), EDT, A* 비용 격자 |
| `amr_sim2d/paths.py` | A* 경로 풀 — 팀 A* 코어 그대로 (바인딩), 캐시 `/home/CAPYI_AI/runs/r2-sim2d/cache` |
| `amr_sim2d/obstacles.py` | 비반응 장애물: 작업자(원 0.30)·지게차(사각 −1.2…2.0×±0.64)·셔틀(0.6×0.4), 무작위/Gazebo 배치 |
| `amr_sim2d/ppo.py` | GPU 상주 PPO (B tanh 가우시안, C 마스크 범주형), PI 라그랑주, BC 초기화, ONNX 내보내기 |
| `amr_sim2d/heuristic.py` | 관측만 쓰는 경로 추종기 (처리량 측정 행동, BC 시연자). **DWA 가 아니다** |
| `amr_sim2d/scripted.py` | 착취 표용 결정론적 스크립트 정책 12 종 (대기·추종·지나침·직전 정지·우회·표류·회전·기어가기·원 배회·충돌) |
| `amr_sim2d/exploit.py` | 궤적을 truncation 없이 기록하고 RewardSpec 별 보상·부트스트랩 가치(연속 롤아웃)를 다시 매긴다 |
| `dwa_binding/` | pybind11 모듈 `amr_core_py`: `src/amr_navigation/src/core/{astar,grid,path_smoother,geometry}.cpp` + `src/amr_perception/src/{safety_gate,geometry2d}.cpp` 를 **읽기 전용으로 컴파일** |
| (갱신 2026-10-08) `amr_sim2d/{dwa2d,crosseval}.py`, `dwa_binding/{dwa2d_py.cpp,build_dwa2d.sh,dwa_golden_main.cpp}`, `tools/b5_*` | 2D DWA 와 B5 교차 평가 (b1-b5) — 아래 「2D DWA (b1-b5)」. 기존 파일은 바꾸지 않았다 |
| `tools/` | `bench_throughput.py`, `train_ppo.py` (`--reward` 후보 이름, `--eval-seed`), `onnx_check.py`, `exploit_table.py` (G0·G1 착취 표), `g2_summary.py` (학습 가능성 표·곡선) |
| `tests/` | 아래 표 |

### 한 결정 스텝 (0.1 s)

1. 행동 → 명령 적분 (B: `integrate_b` ray 클램프 / C: `integrate_c`, 마스크 밖이면 폴백 99)
2. 서브스텝 5 회 (50 Hz): 명령 선형 램프 → 프로파일러 기준 모델(저크 필터 + 곡률 보존) → safety φ
   (safety_lite, D = 명령·측정 운동 가설의 스윕 접근 거리) → 플랜트(순수 지연 T_d + 행동 지연 버퍼 보간 →
   1 차 지연 T_m → 가속 한계 × 배율) → 정확 원호 적분 → 지면 진실 충돌(격자 SAT + 장애물). 접촉하면 그 자리에서 정지·종단.
3. 새 스캔: 관측 지연만큼 과거 자세(서브스텝 이력)와 과거 장애물 위치에서 레이캐스트 → 잡음 σ → [0.1, 25] 밖·드롭아웃 = 무반사.
   트랙 = 지면 진실 장애물 + 위치·속도 잡음 + Bernoulli 누락, 나이 = 관측 지연.
4. 보상·비용·이탈 갱신 (`reward_step`), 성공·충돌 = 종단, 시간 상한·정체 = truncation (`final_obs` 로 부트스트랩).
5. 관측(+C 마스크). 끝난 env 는 즉시 리셋 (경로 풀에서 새 시작·목표, DR 재추첨, 장애물 재배치).

## 시험

```bash
docker run --rm --gpus all --name amr-ai-r2-sim2d-test -v <worktree>:/ws -w /ws/rl/sim2d amr-ai/rl:0.3 \
  bash -lc 'bash dwa_binding/build.sh && python3 -m pytest -q tests'          # -m "not slow" 로 컴파일 시험 제외
```

| 파일 | 무엇을 대조하나 |
|---|---|
| `test_golden_torch.py` | golden 31 사례 전부 (decision 17 을 한 배치로, step 11, integrate·profiler·safety) — CPU float64 와 GPU 혼합 정밀도 |
| `test_env.py` | env 를 무작위 행동·장애물·DR 로 굴린 **실제 상태**를 계약 `State` 로 바꿔 numpy 기준 구현의 관측(1e-5)·마스크(정확)·보상(1e-4)·비용과 대조, 자동 리셋, 시드 결정성, 컴파일 = eager. **종료 조건**: 서 있는 로봇에 다가온 작업자·φ 없이 달려가 닿은 정지 장애물 = 충돌 종단(−30, 장애물 없는 쌍둥이 env 대조), 행동 0 → 200 스텝째 정체 truncation, 상한 1 s → timeout truncation + `final_obs` = 쌍둥이 env 의 같은 스텝 관측, 관측 지연 0.1 s → 트랙 장애물 위치 = 0.1 s 전 |
| `test_eval.py` | 평가 추정량: 결말 시간이 다른 가짜 env 에서 env 마다 첫 에피소드만 세는가(참 0.5, 예전 추정량 0.0), 라운드·미완료 집계, 실제 env 에서 센 에피소드의 경로 길이 = 리셋 때 뽑힌 경로 길이, 같은 시드 = 같은 결과 |
| `test_raycast.py` | 축정렬 벽·사선·지도 밖·시작 셀 막힘·사거리 밖 해석해, 원·회전 사각형(0.5 mm 행진 기준), 방 한가운데 720 빔 해석해, Triton = torch |
| `test_collision.py` | 격자 SAT 를 일반 볼록 다각형 SAT 와 3 000 자세 대조, 경계 접촉 포함, 지도 밖, 원·사각 장애물 4 000 건, 스윕 접근 거리를 **C++ `SweptFootprint` 원본**과 67 운동 × 405 점 대조 |
| `test_guards.py` | 변이 검사가 처음에 놓친 가드: 컴파일 아래 투영 동률, 관측 지연, 충돌 뒤 정지, truncation 부트스트랩, 정규화 분산 하한, safety STOP 확정(새 스캔 2 번 연속, D > 0.50 에서 해제) |
| `test_ppo_onnx.py` | PPO 한 번 갱신(B·C), ONNX 내보내기·onnxruntime 왕복, 정규화가 모델 안에 있는가 |
| `test_reward_spec.py` | 보상 설계 후보(RewardSpec): env(spec) 보상·항·종단 = 기준 구현 `reward.step(spec)`, 정체 종단(prog50_st)은 종단·Φ' = 0, 같은 스텝의 성공·충돌 우선, 스크립트 정책 무한 지평 가치 순서(`slow`) |

변이 검사 [측정, `a6660f3` 사본, 변이마다 전체 63 시험, `/home/CAPYI_AI/logs/agents/r2-sim2d/fix/mut_fix_summary.log`·
`mut_fix_result.json`]: 정규화 분산 하한 제거, env 의 장애물 접촉 무시, 정체 truncation 제거, 스캔 장애물의 관측 지연
미적용, STOP 을 스캔 1 번에 확정, 시간 상한 제거, 정체 기준점 항상 갱신, 평가가 먼저 끝난 k 개에서 멈춤, 평가가 env 의
반복 에피소드를 셈, 평가 시드 무시 — **10/10 잡힘**. 예전 `evaluate` 를 통째로 되돌리면 `test_eval.py` 4 개가 실패한다
(가짜 env 에서 0.0 ≠ 0.5, `fix/mut_e0.log`). 리뷰 때(`658aa73`)는 앞의 여섯 중 다섯이 잡히지 않았다.

## 처리량 [측정]

`python3 rl/sim2d/tools/bench_throughput.py --envs 256 1024 4096 --steps 200` (+ `--arm C --modes random --steps 100`),
커밋 `9f3d0cd`, RTX 5090, 장애물 random 0–8, DR 켬, safety 켬, 컴파일 켬. 로그
`/home/CAPYI_AI/logs/agents/r2-sim2d/bench/throughput_{B,C}_9f3d0cd.json`. 설정마다 새 프로세스.

| 비교군 | env | 행동 | env-steps/s | 리셋/스텝 | GPU 메모리 (reserved) | loadavg(1 분) |
|---|---:|---|---:|---:|---:|---:|
| B | 256 | 경로 추종 | 14 068 | 1.6 | 0.30 GB | 22.0 |
| B | 1 024 | 경로 추종 | 52 312 | 6.3 | 0.63 GB | 31.8 |
| B | 4 096 | 경로 추종 | 138 742 | 25.1 | 1.96 GB | 43.4 |
| B | 256 | 무작위 | 30 784 | 0.3 | 0.20 GB | 37.8 |
| B | 1 024 | 무작위 | 65 137 | 1.0 | 0.63 GB | 43.9 |
| B | 4 096 | 무작위 | 146 855 | 3.6 | 1.95 GB | 37.7 |
| C (마스크 포함) | 256 | 무작위(마스크 안) | 19 380 | 0.2 | 0.30 GB | 40.1 |
| C (마스크 포함) | 1 024 | 무작위(마스크 안) | 36 909 | 1.0 | 0.63 GB | 36.9 |
| C (마스크 포함) | 4 096 | 무작위(마스크 안) | 90 165 | 3.9 | 1.95 GB | 33.6 |

- 같은 측정을 커밋 `35ce14e`(리셋 관측·C 마스크 컴파일 전)에서 loadavg 97–136 일 때 하면 B 4 096 env 97 501 /
  100 151 env-steps/s 였다 (`bench/throughput_B_35ce14e.json`). 두 표의 차이에는 코드 변경과 부하 차이가 섞여 있다.
- 병목은 GPU 연산이 아니라 **CPU 의 커널 발행**이다. 컴파일 전 4 096 env 한 스텝 = CUDA 커널 8 343 개, 컴파일 뒤
  1 513 개 [측정했으나 **로그 미보존** — `torch.profiler`, 재현 스크립트 `/home/CAPYI_AI/runs/r2-sim2d/scratch/prof_env.py`
  는 개수를 표준 출력에만 냈다]. 남은 큰 몫은 컴파일하지 않은 부분집합 리셋(`_reset_idx`: 장애물 배치·스캔)이다.
- 레이캐스트 자체: Triton DDA 4 096 × 720 빔 = 3.3 ms [측정 1 회, **로그 미보존**].
- 알려진 낭비: `geometry2d.py:55` 의 `float(map_meta[0, 0])` 이 컴파일된 `substep` 안에서 그래프 끊김과 호스트 동기화를
  서브스텝마다 일으킨다 (dynamo 경고). 해상도를 파이썬 상수로 넘기면 없어진다 — 아직 고치지 않았다.

## PPO 학습 결과 [측정] — 성공률 상승은 **미달**

**평가 추정량 (2026-10-06 리뷰 뒤 고침, `a8afe77`)**: env 마다 리셋 직후 **첫 에피소드 하나만** 센다 (`ppo.evaluate`).
그 전 `evaluate` 는 env 512 개를 굴려 먼저 끝난 에피소드 ≈500 개를 셌다. 일찍 끝나는 결말(초반 충돌·짧은 경로)이
과대 표집되고 정체처럼 늦게 끝나는 결말이 빠지는 **편향 추정량**이었다 — B_bc3 최종 체크포인트에서 예전 0.408,
편향 없이 0.532 다. 장애물이 없으면 방향이 반대다 (예전 1.000, 편향 없이 0.957 — 리뷰 측정
`/home/CAPYI_AI/logs/agents/r2-sim2d/review/diag_summary.log`). 아래 표는 저장된 체크포인트를 새 추정량으로
**다시 잰 값**이다 (재학습 없음): `tools/reeval_runs.py`, 코드 `5b42ca8`, env 2 048 개 = 에피소드 2 048 개, 평가 env
시드 1000 (학습 때 평가와 같은 경로 풀), 체크포인트끼리 같은 첫 에피소드 집합(공통 난수), 장애물 mixed(무작위 0–4 +
20 % Gazebo 7 개)·DR·safety 켬, 결정론적 정책(B: tanh(μ), C: argmax). 성공률 표준오차 ≤ 0.011, 끝나지 않은 env 0.
분산 하한(`6be890c`) 전 run 의 체크포인트는 하한 없이(var_floor 0) 정규화해 학습 때와 같게 쟀다. 결과
`/home/CAPYI_AI/runs/r2-sim2d/<run>/reeval.jsonl`, 실행 로그 `/home/CAPYI_AI/logs/agents/r2-sim2d/fix/reeval_all.log`,
요약 `/home/CAPYI_AI/logs/agents/r2-sim2d/runs_summary.{json,txt}`, 그림 `.../learning_curves.png`. 학습 때 `eval.jsonl` 은
편향 추정량 기록으로 그대로 둔다.

| run | 학습 코드 | 설정 | 평가 성공 (스텝) | 마지막 체크포인트의 결말 |
|---|---|---|---|---|
| B_main1 | `35ce14e`+커밋 안 된 코드 (HEAD 기록 없음) | 순수 PPO, 제약 없음 | 0 → 0 (7.9M) | 정체 0.794 · 시간 상한 0.161 · 충돌 0.045 |
| B_lag1 | `f0d293c` | 순수 PPO + PI 라그랑주 + 커리큘럼 | 0 → 0 (7.9M) | 정체 0.870 · 충돌 0.099 · 시간 상한 0.031 |
| B_noobs1 | `9f3d0cd` | 순수 PPO + 라그랑주, 장애물 없음 | 0 → 0 (13.8M, 학습은 15.3M 에서 멈춤) | 정체 1.000 (같은 설정의 휴리스틱은 성공 0.993) |
| B_bc1 | `f0d293c`+커밋 안 된 BC diff | BC + PPO + 라그랑주 (정규화 결함) | BC 0.538 → 0.009 (6.2M) | 충돌 0.493 · 시간 상한 0.385 · 정체 0.113 |
| B_bc2 | `24543a2`+target-kl diff | BC + PPO (정규화 결함) | BC 0.538 → 0.012 (6.2M) | 충돌 0.485 · 시간 상한 0.289 · 정체 0.214 |
| **B_bc3** | `6be890c` | BC(휴리스틱 시연 30.7 만 스텝) + PPO(σ 0.08, lr 1e-4, KL 0.01) | 0 → **0.538 (BC)** → 0.5347–0.5425 (갱신 15–105) → **0.532** (15.9M) | 충돌 0.429 · 정체 0.040 · 시간 상한 0 |
| C_scratch1 | `24543a2` | 마스크 PPO(C) 처음부터 + 커리큘럼 | 0 → 0.0005 (2.6M) | 시간 상한 0.726 · 정체 0.169 · 충돌 0.105 |
| C_scratch2 | `6be890c` | 같음, 정규화 수정 뒤 | 0 → 0.0005 (2.6M) | 정체 0.465 · 시간 상한 0.442 · 충돌 0.093 |

- B_bc3 는 평가 시드 2000(다른 경로 풀)에서도 BC 0.533 → 갱신 15–105 0.533–0.542 → 최종 0.533 이다. 같은 평가에서
  **시연자 휴리스틱 자체가 0.535 (시드 2000: 0.532)** 다 — BC 정책 ≈ 휴리스틱이고 PPO 는 그 위로 올리지 못했다.
  체크포인트 사이 차이(최대 0.011)는 표준오차 0.011 수준이다.
- 실패의 내용: B_bc3 최종 체크포인트의 충돌은 전부 장애물 접촉이고 격자 접촉은 0 건이다 (n = 512 에서 211/211, 리뷰 측정
  `review/diag.json`). 휴리스틱 시연자는 장애물을 보지 않으므로 BC 정책도 피하지 못하고, PPO 15.6M 스텝은 회피를 배우지
  못했다. 그리고 `obstacles.place_gazebo` 에는 시작점 근처를 거르는 단계가 없어 (무작위 배치는 시작점 2 m 안을 거른다)
  mixed 의 20 % 에피소드는 시작 직후 비반응 장애물에 닿을 수 있다 — 피할 수 없는 에피소드가 섞여 있을 수 있다
  (그 비율은 재지 않았다).
- 학습 롤아웃 곡선(`train.jsonl` 의 success)은 롤아웃 동안 **끝난** 에피소드의 비율이라 짧게 끝나는 에피소드 쪽으로
  치우친다 — 평가 지표가 아니라 학습 진단으로만 본다.
- **성공률이 오른 것은 BC 덕이지 PPO 덕이 아니다.** B_bc3 의 PPO 15.6M 스텝은 성공률을 BC 수준에서 바꾸지 못했다
  (편향을 없앤 재평가에서도 같다).
- 순수 PPO 가 못 배우는 이유 [계산]: 계약 보상에서 끝나지 않는(truncation 으로 V(final_obs) 부트스트랩되는) 궤적의
  무한 지평 가치는 같다 — PBRS 합이 −Φ(s₀) = κ·d₀ 로 텔레스코핑되고 시간 비용은 −c/(1−γ) = −4 이므로 "제자리 대기"와
  "목표 옆까지 가서 정지 조건(0.10 m, |v| ≤ 0.05 m/s)을 못 맞춤"의 가치가 모두 κ·d₀ − 4 다. 충돌로 끝나는 궤적은
  −30γ^T 와 잘린 시간 비용 때문에 값이 다르다. 대기와의 차이는 성공 보상 +G 뿐이고, 움직이면 평활·
  라그랑주 벌점과 충돌 위험만 더해진다. 스텝 단위로는 Φ = −κ·d_arc 가 대기에 +(1−γ)κd(d > 4 m 면 시간 비용보다 큼)를
  준다. 학습 롤아웃 성공률이 갱신당 최대 0.26 %(B_main1)·0.36 %(B_lag1)·0.9 %(B_noobs1)에 그친 run 이 전부 대기로
  수렴했다 — 성공 신호가 거의 없으면 대기가 (약하게) 최적이라는 위 계산과 맞는다. 이것은 설명이지 증명이 아니다.
- **정규화 결함 (고침, `6be890c`)**: BC 시연 동안 늘 0 이던 이탈 블록(224–226)·빈 트랙 칸(4–7)의 running 분산이
  3e-10 이 되어, 학습 중 그 특징이 바뀌자 입력이 ±10 에 붙었다 (B_bc2 학습 롤아웃 성공: 첫 갱신 0.007, 42 갱신 평균
  0.0145, 범위 0.004–0.028). 분산 하한 0.01 로 고친 뒤 같은 BC 정책의 학습 롤아웃 성공이 갱신당 0.51–0.58 (평균 0.54,
  114 갱신)이 됐다. B_main1·B_lag1·B_bc1·B_bc2·C_scratch1·smoke_B1 은 이 수정 **전**의 run 이다.
- ONNX: 끝까지 돈 두 run(smoke_B1, B_bc3)에서 onnxruntime 왕복 통과 — B_bc3 `policy.onnx` 최대 오차 2.1e-6 (golden 17
  사례, env 관측 배치 1·N), 메타데이터 `contract_version 1.1.0, arm B, obs_dim 291, dt 0.1, normalization inside` [측정,
  `/home/CAPYI_AI/runs/r2-sim2d/B_bc3/onnx_check.json`]. 모델 파일은 커밋하지 않는다 (루트 `.gitignore:42-43`).

## 보상 v1.2 (2차 물결 r5-reward) — 순수 PPO 학습 가능성 [측정]

`--reward` 로 보상 설계를 고른다 (기본 = 계약 v1.2.0). 같은 예산(순수 PPO, BC 없음, 14.94M env-steps, 2 048 env,
정적 지도, 평가 시드 4242·n = 2 048, 마지막 반복 정책)에서 평가 성공: **v1.2.0(= prog50) 11 시드 중 10 개
0.948–0.990, 시드 2 는 0.000 (목표 약 2 m 앞 "직전 정지" 끌개)**, **동적 장애물(mixed 0–4, prog50) 0.000 — 제자리
회전으로 굳는다** (1 run), v1.1.0 0.000, PBRS 만 고친 pbrs_eff 0.000·0.000, prog25 0.198·0.000, prog50_st
0.992·0.977, 비교군 C(prog50) 0.937 (1 run). 표·곡선·착취 표와 이유는
[`docs/research/rl-navigation/reward_v12.md`](../../docs/research/rl-navigation/reward_v12.md), 원자료
`/home/CAPYI_AI/runs/r5-reward/g2/`. 위 1차 물결 표는 v1.1.0 보상으로 학습한 것이다.

## 동적 장애물 (2차 물결 2b r6-dyn) [측정]

v1.2.0 보상, mixed 0–4 + 20 % Gazebo, 30M env-steps, 평가 n = 2 048 (시작점 거르기 3 s·1.0 m), 마지막 반복:
① 커리큘럼 + learnability 0.000·0.000, ② 비교군 C 0.001·0.057, ③ 라그랑주 + C 10 0.279 / 0.521 / 0.000 / 0.041 /
0.018 (5 시드, 50 % 넘은 시드 1/5), ④ 특권 전문가 BC + DAgger + KL-to-BC 0.562–0.575 (5/5) — 단 ④ 는 장애물을 보지
않는 0.8 m/s 추종기(0.545)와 결말이 92.5 % 같다. 회피를 배운 후보는 없다. 근거·원인 분석·다음 시도는
[`docs/research/rl-navigation/dyn_obstacles.md`](../../docs/research/rl-navigation/dyn_obstacles.md).
새 인자 (모두 기본 꺼짐): `EnvConfig.start_clear_window/radius` (`obstacles.clear_start`), `EnvConfig.level_probs`,
`train_ppo.py --learnability --start-clear-window --final-eval-orig --lag-kd/-max/-ema/-channels --lag-budget-collision
--bc-expert privileged --dagger --kl-bc`, 보상 후보 `prog50_c<C>` (`dyn_candidates.py`), 진단 `tools/dyn_g0.py`·`dyn_eval.py`,
특권 전문가 `amr_sim2d/expert.py`, 시험 `tests/test_dyn_obstacles.py`.

## 계약·보고서와 다르게 한 것

| 항목 | 여기서 한 것 | 이유 |
|---|---|---|
| 하류 사슬 순서 | 프로파일러 → **φ → 플랜트** | 계약 §5.4 문장은 "플랜트 → φ" 지만 §1·§7.1(φ 입력 = 프로파일러 출력)과 실제 사슬(velocity_profiler → safety_node → DiffDrive)은 φ 가 플랜트 앞이다. r0 에 확인 요청 |
| safety D | 스윕 접근 거리(C++ 원본과 점별 일치)의 **유효 빔 최솟값** | `robustBeams`(5 빔 창 3 번째 값)·지연 보정·접촉 가드·도킹 예외 없음 — 계약 §7.4 safety_lite 와 같은 범위. STOP 확정 = 연속 2 스캔(서브스텝 1 의 D), 해제 = D > 0.50 |
| 미지 셀 | 막힘으로 | SLAM 이 못 본 랙 내부·기둥·벽 뒤 (주 자유 영역에 맞닿은 미지 셀 374 개 [계산]) |
| 장애물 운동 | 등속 왕복·원 순환, 끝에서 즉시 반전 | Gazebo 의 사다리꼴 가감속·지게차 1.0 m/s² 램프 미구현 (과제 "1 차는 등속·왕복") |
| 작업자 LiDAR 모양 | 원 r 0.30 | 충돌 판정 반경과 같게. Gazebo actor 메시의 0.2 m 높이 단면은 더 작다 |
| 트랙 | 지면 진실 중심 + 잡음·누락 | 실제 추적기는 관측 점의 중심을 낸다 (지게차처럼 큰 물체에서 차이) |
| 행동 지연 DR | 플랜트 순수 지연 T_d 에 **더한다** | 계약 표가 둘을 따로 두되 합성 규칙이 없다 |
| 지도 다양성 (DR 1 순위) | **팀 지도 1 장만** | 절차적 생성기 5 종 미구현 (다음 물결). `stack_maps` 는 여러 지도를 받도록 해 두었다 |
| 커리큘럼 | 경로 길이·장애물 수 선형 단계만 | learnability p(1−p) 샘플링 미구현 |
| 관측 정규화 | (obs − μ)/√(max(σ², 0.01) + 1e-8) | onnx_io.md 식은 √(σ² + 1e-8). 하한이 없으면 늦게 바뀌는 특징이 폭주한다 (위). 하한은 모델 그래프 안이라 플러그인 쪽은 그대로 |
| B 의 BC 초기화 | 관측만 쓰는 경로 추종 휴리스틱 시연 | 계약 §1 은 B 를 순수 RL 로 정한다 — BC 를 쓴 run 은 "순수 B" 가 아니다. DWA 시연(C 의 BC)은 바인딩이 없어 못 했다 (갱신 2026-10-08: 바인딩은 b1-b5 가 만들었다 — 「2D DWA (b1-b5)」. DWA 시연 BC 는 여전히 안 했다) |
| 개입 비용 λ | 예산 미정(§7.2)이라 λ = 0 | DWA(A) 의 2D 개입 분포가 필요하다 — 바인딩 미구현 (갱신 2026-10-08: 바인딩·2D-DWA 개입 기록은 B5 에 있다 — b_b5_2d.md §4.4. 예산은 아직 정하지 않았다) |

## 2D DWA (b1-b5, 2026-10-08) — 팀 DWA 코어 바인딩과 B5 교차 평가

근거·사전 등록: 노트 `docs/research/rl-navigation/notebook/entries/2026-10-08_0410_b1-b5_2d-crosseval-dwa.md` (사전 등록 커밋 `ade60d3`,
평가 코드 `669a384`), 결과·재현: [`docs/research/rl-navigation/b_b5_2d.md`](../../docs/research/rl-navigation/b_b5_2d.md)
(§1 무엇을 쟀나, §2 붙인 방법, §7 2D 의 한계, §8 시험·변이·재현). 아래 「DWA 코어 바인딩 (과제 5)」 의 설계를 이것으로 구현했다.

**기존 기본 동작은 바뀌지 않았다 — 새 파일만 더했다** [git 기록: `git diff --name-status 74d3a13 4b323d7 -- rl/sim2d` 이 모두 A]. `Sim2DEnv`·
`kernels`·기존 `amr_core_py` (`build.sh`) 는 그대로이고 새 모듈 `amr_dwa2d_py` 는 따로 빌드한다. `crosseval.FixedEpisodeEnv` 는 `Sim2DEnv` 를
상속해 감싸며, `step_rl` 이 `Sim2DEnv.step` 과 비트 같음을 시험이 확인한다 (120 스텝, 생성기 상태까지).

**불리한 것 먼저** (b_b5_2d.md §0·§7):
- φ 는 **safety_lite** 다 (정본 아님). 2D-DWA 는 직진 여유 c ∈ [0.20, 0.30) m 목표에서 Gazebo DWA 보다 낮다 (0.47–0.53 대 0.87) —
  safety_lite 의 D (유효 빔 최솟값) 가 일찍 STOP 한다는 해석은 [추정]. 2D 의 스트레스 RL − DWA 크기를 Gazebo 와 견주지 않는 까닭이다.
- 정적 세계 이식이다: 입력은 지면 진실 자세·플랜트 속도 (위치추정·EKF 잡음 없음), 지역 코스트맵에 깊이 층 없음. 동적 장애물·트랙·VO/TTC 입력·
  횡단 게이트·결정 dwell·적재 질량·BT 복구 행동 (회전·대기·후진) 은 이식하지 않았다 — 계약 정체 20 s 에서 끝난다.
- 2D-DWA 는 경로 위 교착 (출발 교착·중간 정체) 을 보였다 (B5 주 집합 2 048 중 8 + 9). Gazebo 에서도 나는지는 **미측정**.
- 판정은 계약 엄격 성공 (0.10 m ∧ |v| ≤ 0.05) 에서 끝난다 — DWA 의 yaw 정렬·goal checker (Nav2 성공) 는 재지 않는다.

### 파일

| 파일 | 내용 |
|---|---|
| `dwa_binding/dwa2d_py.cpp`, `dwa_binding/build_dwa2d.sh` | pybind11 모듈 `amr_dwa2d_py` — `src/amr_navigation/src/core/{dwa,velocity_obstacle,crossing_gate,crossing_yield,speed_profile,geometry,footprint,grid,scan_denoise}.cpp` 를 **읽기 전용으로 컴파일** (`-O2 -std=c++17 -Wall -Wextra -Werror`). 래퍼 상태 (직전 명령 기억, `no_valid_patience` 예외 틱 → 0 명령), 경로 창 (`PlanWindow::window` 이식, path_horizon 4.0 m), 스캔 기반 rolling 지역 코스트맵 (8 × 8 m · 0.025 m, 게이트 중앙값 → 레이트레이스 지우기 6.0 m · 마킹 5.5 m → 풋프린트 지우기 → 팽창 0.8 m · s 3.0) |
| `dwa_binding/dwa_golden_main.cpp` | 바인딩을 거치지 않는 독립 실행 파일 (설정을 C++ 에 손으로 옮김) — 골든 대조의 기준 |
| `amr_sim2d/dwa2d.py` | `load_config()` — 손으로 고르지 않는 배포 설정 읽기: 표 `PARAMS` (래퍼 `dwa_controller.cpp` `configure()` 의 param() 을 옮긴 것) 를 `nav2_params.yaml` `controller_server.DWA` → `config/robot_params.yaml` `limits.*` → 래퍼 기본값 순으로 채우고 필드마다 출처 `파일:행` 을 `sources` 에 남긴다. `module()` (없으면 `build_dwa2d.sh` 로 빌드 뒤 import), `DwaBatch` (env 마다 `Dwa2D` 하나 — `reset`·`update_costmap`·`step`, C++ 스레드 풀) |
| `amr_sim2d/crosseval.py` | `export_world` (Isaac 기하 npz → 2D 래스터 세계 PGM + YAML, `Sim2DEnv` 가 `cfg.map_yaml` 로 읽음), `start_yaw_isaac` (첫 점 → 0.5 m 앞 점 접선 + 잡음), `FixedEpisodeEnv` (env 마다 고정 에피소드 하나, 끝난 env 는 리셋하지 않음; `step_rl`, `step_dwa` — 결정 주기 0.1 s 안에서 DWA 를 t 와 t + 0.06 s 에 두 번 부르고 각 명령을 다음 서브스텝들의 프로파일러 목표로 0 차 유지 = 20 Hz) |
| `tools/b5_crosseval.py` | 실행기 — Isaac 평가 집합을 RL (ONNX, `action` = tanh(μ)) 또는 2D-DWA 로, env 마다 첫 에피소드, DR 끔·장애물 없음·safety_lite. 출력 `episodes.npz`·`summary.json`·`registry.json` (`rl/tools/notebook.py register` 초안)·`run.log`. unfinished 는 0 이어야 한다 |
| `tools/b5_plan_paths.py` | 평가 행의 (시작, 목표, 끝 접선 yaw) 를 팀 A* 코어 + 평활기 (`amr_core_py.plan`, nav2_params AStar 설정) 로 다시 계획 — 계획 지도 = Isaac 0.05 m 점유 격자 + 전역 팽창 |
| `tools/b5_mapcheck.py` | 사전 점검 — Isaac 평가 집합을 2D 지도에서 그대로 쓸 수 있는가 (좌표계·여유·주/스트레스 분할 일치, 성공률을 보지 않는다) |
| `tools/b5_run_all.sh` | 본 실행 드라이버 (lane `plan`·`dwa`·`rl`, `cpu_slot.sh` 안, 끝난 run 은 건너뜀) — 경로·이름이 b1-b5 원자료에 묶여 있다 |
| `tools/b5_analysis.py` | 사전 등록 집계 (Δ = 10 시드 평균 − DWA, 2 원 부트스트랩 시드 × 목표 군집, 방향 규칙; sha256 `e09eadf0…` 결과 뒤 바뀌지 않음) |
| `tools/b5_supp.py` | 보충 (사전 등록 외) — 직진 여유 구간별 성공, Gazebo 구성 재가중 |
| `tools/b5_make_dwa_golden.py` | `tests/golden_dwa2d.json` 생성 (코어·배포 설정이 바뀌면 다시 만든다) |

### 명령 (모두 `amr-ai/rl:0.3` 컨테이너 안)

```bash
bash rl/sim2d/dwa_binding/build_dwa2d.sh       # → dwa_binding/build/amr_dwa2d_py*.so, dwa_binding/build/dwa_golden_main
# 2D 세계 (Isaac 기하 0.025 m · pad 0 래스터) 는 crosseval.export_world, 지도 점검은 b5_mapcheck.py
python3 rl/sim2d/tools/b5_mapcheck.py --eval-set rl/isaac/eval_sets/b1_eval_sets_v1.json \
    --pool <paths_58eaa34ae5_s1.npz> --geom <warehouse_58eaa34ae5.npz> --out <dir>
# 평가 행 경로 계획 (주 실행 W1 용; 주지 않으면 Isaac 풀 경로 = 보조 W1i)
python3 rl/sim2d/tools/b5_plan_paths.py --pool <풀 npz> --geom <기하 npz> --world-dir <dir> --threads 16 --out <plans.npz>
# 교차 평가 한 실행 (DWA 팔 / RL 팔)
python3 rl/sim2d/tools/b5_crosseval.py --arm dwa --world <W1 yaml> --pool <풀 npz> [--paths-npz <plans.npz>] \
    --seed 4242 --threads 16 --out <dir> --run-id <id>
python3 rl/sim2d/tools/b5_crosseval.py --arm rl --policy <B1_ppo10_s<N>/policy_final.onnx> --world <W1 yaml> --pool <풀 npz> \
    [--paths-npz <plans.npz>] --seed 4242 --threads 8 --out <dir> --run-id <id>
# 사전 등록 집계 (--world-tag W1 주 / W1i 보조)
python3 rl/sim2d/tools/b5_analysis.py --runs <eval 디렉터리> --isaac /home/CAPYI_AI/runs/bonus1/b1-ppo10-eval/eval \
    --pool <풀 npz> --world-tag W1 --out <dir>
```

- 시험용 선택지: `--rows k` (앞 k 행만), `--pool-rows lo:hi` (집합 대신 풀 행 — 스모크), `--sets` (집합 고르기), `--no-compile`.
- B5 본 실행은 `tools/b5_run_all.sh plan|dwa|rl` 로 돌렸다 (b_b5_2d.md §8 재현). DWA 330 s/실행 (C++ 16 스레드), RL 62–94 s/실행 —
  공유 부하 하 예비값 (loadavg 5.4–21.0). 성공·충돌은 sim 지표라 부하와 무관하다 (스모크 64 행이 N 64·1 024 에서 행마다 같은 결말).

```python
from amr_sim2d import dwa2d
conf = dwa2d.load_config()          # dict(cfg, wrapper, costmap, footprint, sources) — sources[필드] = '파일:행' 또는 래퍼 기본값
batch = dwa2d.DwaBatch(n_env, conf, threads=16)
batch.reset(i, path_xy_or_poses)    # 새 에피소드: 경로 [K,2] 또는 [K,3]
batch.update_costmap(idx, sensor_pose, robot_pose, ranges)   # 스캔마다 (10 Hz)
out = batch.step(idx, pose, v, w)   # 제어 틱 → dict(v, w, found, exception, n_valid, …)
```

### 시험 (B5 에서 더한 것)

- `tests/test_dwa2d.py` 18 개: 설정 표 ↔ 래퍼 원문 정규식 대조 (param() 94 개), YAML 값·왕복, **골든 5 사례** — 바인딩 `compute_raw` 한 걸음 출력 =
  독립 실행 파일 `dwa_golden_main` 출력 (`%.17g` 비트 일치) + 저장 golden `tests/golden_dwa2d.json`, 래퍼 첫·둘째 걸음, `no_valid_patience` 예외,
  코스트맵 마킹·지우기·rolling·팽창·마킹 범위·게이트 중앙값, `step_rl` = `Sim2DEnv.step` 비트 대조, `step_dwa` 두 틱 시각, 직선·회전·막힘 주행, 첫 에피소드 규칙.
- `tests/test_b5_analysis.py` 3 개: 가짜 원자료로 집계·판정 규칙.
- rl/sim2d 전체 [측정, 평가 커밋 `669a384`]: **116 통과 · 실패 0** (기존 95 + 새 21). 변이 13 개 중 10 잡힘, 남은 3 은 등가 (b_b5_2d.md §8).

## DWA 코어 바인딩 (과제 5) — 이번에 못 했다. 설계

(갱신 2026-10-08: b1-b5 가 위 「2D DWA (b1-b5)」 로 구현했다. 아래는 1차 물결 당시의 설계 기록이다. 구현과 다른 점 — 새 모듈 `amr_dwa2d_py` 로
따로 빌드했고 (`amr_core_py` 에 넣지 않음), 컴파일 목록에 `grid`·`scan_denoise` 가 더해졌고, 설정 사상은 param() 94 개, 지역 코스트맵은 배포 설정
(`nav2_params.yaml` 526 행 `plugins: [obstacle_layer, depth_layer, inflation_layer]` — 정적 층 없음) 에서 깊이 층만 뺀 스캔 기반 rolling 이식이다.
양보 결정·게이트는 배포 설정에서 꺼져 있다 (yield_decision·yield_gate false).
DWA 시연 BC·개입 예산 λ 는 여전히 하지 않았다.)

지금 `dwa_binding/` 은 A*·팽창·평활기·safety 스윕 접근 거리만 묶는다. 비교군 A(튜닝된 DWA)를 2D 에서 돌리려면:

1. **컴파일 목록**: `core/{dwa,velocity_obstacle,footprint,crossing_gate,crossing_yield,geometry,speed_profile}.cpp`
   (r0 의 `rl/contract/tools/crosscheck_cpp.sh` 가 같은 목록을 이미 ROS 없이 빌드한다).
2. **설정 사상**: `src/amr_navigation/src/ros/dwa_controller.cpp` 의 매개변수 → `DwaConfig` 대입(약 64 곳)을 그대로 옮긴
   `config_from_yaml(nav2_params.yaml)` — 손으로 고르면 "튜닝된 DWA" 가 아니게 된다.
3. **지역 코스트맵**: DWA 는 `FootprintCostFn`·`PointCostFn` 으로 코스트맵을 본다. 2D 에서는 정적 지도 ∪ 이번 스캔 끝점을
   지역 팽창(cost_scaling 3.0, 반경 0.8, `nav2_params.yaml:577-580`)으로 만든 롤링 격자 + `FootprintChecker`.
   계약 §14.3 대로 마스크(C)는 스캔 끝점만 쓰므로 A 와 C 의 장애물 입력이 다르다는 점을 그대로 보고한다.
4. **플러그인 상태**: 양보 결정·게이트 상태·직전 명령(`yield_limit_last`, `gate_state`)을 env 별로 이월.
5. **배치**: DWA 는 env 당 CPU 1 회 (C++ 수 ms) → 스레드 풀. 학습용이 아니라 비교군 A 평가·BC 시연(`contract_torch.bc_label`) 용.
6. **시험**: 같은 입력에서 바인딩 결과 = ROS 플러그인 결과 (플러그인 단위 시험의 입력을 재사용), 그리고 2D 에서 A 의
   성공·접촉·이탈 분포가 Gazebo 시나리오 08 과 같은 방향인가 (실무 표 단계 1 의 넘어가는 기준).
