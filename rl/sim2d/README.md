# rl/sim2d — GPU 배치 2D 레이캐스트 시뮬레이터 (보너스 1 주 학습장)

계약 [`rl/contract/contract.md`](../contract/contract.md) v1.1.0 의 관측 291·행동(B 연속 / C 격자 100)·보상·비용 5·
종료를 팀 지도 위에서 수천 env 병렬로 돌리는 torch 시뮬레이터. 근거: 「실무」 54–57행(2D 주 학습장), 80–100행(DR),
「이론」 261–317행(DR·SRCC·접촉 = 즉시 종료). 소유: r2-sim2d (`/home/CAPYI_AI/COORDINATION.md` §2).

수치 표기: **[측정]** 명령·로그 있음 / **[계산]** 오프라인 / **미측정**. 이 서버는 공유 서버이고 측정 때 loadavg 가
90–136(32 코어)이었다 — 처리량은 전부 **공유 부하 하 예비값**이다.

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
| `dwa_binding/` | pybind11 모듈 `amr_core_py`: `src/amr_navigation/src/core/{astar,grid,path_smoother,geometry}.cpp` + `src/amr_perception/src/{safety_gate,geometry2d}.cpp` 를 **읽기 전용으로 컴파일** |
| `tools/` | `bench_throughput.py`, `train_ppo.py`, `onnx_check.py` |
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
| `test_env.py` | env 를 무작위 행동·장애물·DR 로 굴린 **실제 상태**를 계약 `State` 로 바꿔 numpy 기준 구현의 관측(1e-5)·마스크(정확)·보상(1e-4)·비용과 대조, 접촉 = 즉시 종단, 자동 리셋, 시드 결정성, 컴파일 = eager |
| `test_raycast.py` | 축정렬 벽·사선·지도 밖·시작 셀 막힘·사거리 밖 해석해, 원·회전 사각형(0.5 mm 행진 기준), 방 한가운데 720 빔 해석해, Triton = torch |
| `test_collision.py` | 격자 SAT 를 일반 볼록 다각형 SAT 와 3 000 자세 대조, 경계 접촉 포함, 지도 밖, 원·사각 장애물 4 000 건, 스윕 접근 거리를 **C++ `SweptFootprint` 원본**과 67 운동 × 405 점 대조 |
| `test_guards.py` | 변이 검사가 처음에 놓친 가드: 컴파일 아래 투영 동률, 관측 지연, 충돌 뒤 정지, truncation 부트스트랩 |
| `test_ppo_onnx.py` | PPO 한 번 갱신(B·C), ONNX 내보내기·onnxruntime 왕복, 정규화가 모델 안에 있는가 |

## 처리량 [측정]

`python3 rl/sim2d/tools/bench_throughput.py --envs 256 1024 4096 --steps 200` (커밋 `35ce14e`, RTX 5090, 비교군 B,
장애물 random 0–8, DR 켬, safety 켬, 컴파일 켬). 로그 `/home/CAPYI_AI/logs/agents/r2-sim2d/bench/throughput_B_35ce14e.json`.

| env | 행동 | env-steps/s | 리셋/스텝 | GPU 메모리 (reserved) | loadavg(1 분) |
|---:|---|---:|---:|---:|---:|
| 256 | 경로 추종 | 9 561 | 1.6 | 0.30 GB | 117.7 |
| 1 024 | 경로 추종 | 30 913 | 6.3 | 0.63 GB | 136.0 |
| 4 096 | 경로 추종 | 97 501 | 25.1 | 1.96 GB | 116.2 |
| 256 | 무작위 | 18 745 | 0.3 | 0.20 GB | 122.7 |
| 1 024 | 무작위 | 43 749 | 1.0 | 0.63 GB | 108.2 |
| 4 096 | 무작위 | 100 151 | 3.6 | 1.95 GB | 97.1 |

- 병목은 GPU 연산이 아니라 **CPU 의 커널 발행**이다 (공유 부하가 큰 이유). 컴파일 전 4 096 env 한 스텝 = CUDA 커널
  8 343 개였고, 컴파일 뒤 1 513 개 [측정, `torch.profiler`]. 남은 큰 몫은 **컴파일하지 않은 부분집합 리셋 경로**다
  (4 096 env 에서 스텝당 약 8 ms [측정, 구간 계측 1 회]) — 다음 개선 대상.
- 레이캐스트 자체: Triton DDA 4 096 × 720 빔 = 3.3 ms [측정, 1 회, 같은 부하].

## PPO

(아래 「학습 결과」 절은 실행 뒤 채운다.)

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

## DWA 코어 바인딩 (과제 5) — 이번에 못 했다. 설계

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
