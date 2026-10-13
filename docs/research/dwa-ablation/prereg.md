# DWA 절제·고전 제어기 비교 사전 등록

작성 2026-10-14 03시대 KST. 이 문서는 held-out 결과를 보기 **전에** 커밋한다. 결과는 `results.md`, 실행 기록은
`runs.jsonl` (실행 묶음마다 한 줄). 이 문서 커밋 뒤에 바꾼 것은 `results.md` 의 「사전 등록과 다르게 한 것」에 적는다.

## 0. 무엇을 재는가

1. **고전 비교 (표 1)**: 같은 코드 버전, 같은 하류 사슬(velocity_profiler 저크 제한 2 m/s³ → 서보 지연 0.04 s +
   1 차 0.08 s + 가속 제한), 같은 목표 판정(xy 0.10 m, yaw 0.05 rad, stateful false), 같은 진행 검사(0.3 m / 20 s)
   에서 우리 DWA · 교과서 DWA(우리 코어의 수정 전부 끔) · Nav2 DWB 기본·튜닝 · MPPI 기본·튜닝 · RPP 기본·튜닝 ·
   우리 PP 를 held-out 경로에서 비교한다.
2. **절제 (표 2)**: 우리 DWA 전체에서 수정 하나씩 끈 팔과 전체의 경로별 짝 차.
3. **M3 기전 (표 3)**: 창 중심(직전 명령 대 측정값)의 효과가 하류 사슬이 있을 때만 나타나는지, 이상 플랜트
   (사슬 없음)와 제어 주기 10/20/40 Hz 로 본다.
4. **두 시험대 차이 (표 4)**: 우리 팔을 ROS 시험대와 ROS 없는 시험대에서 같은 경로로 돌린 차.

주장할 수 있는 범위는 **시뮬레이션 두 가지(모형화한 서보)** 다. 실로봇·Gazebo 는 이 연구에 없다.

## 1. 코드 버전과 환경

- 브랜치 `feature/dwa-ablation` (origin/main `2345e27` 에서 분기). 절제 스위치 `03f9ff3`, 시험대 도구 `b02bf54`,
  실험은 이 문서를 담은 커밋에서 돈다 (실행마다 `runs.jsonl` 에 commit·dirty 를 적는다).
- 기본값 비트 동일: `test_dwa_ablation` 의 golden 해시 8 개가 스위치 추가 전 코어와 같다. 기본값 5 개를 하나씩
  뒤집으면 golden 이 깨진다 (불일치 8/2/6/6/2, `runs/dwa-abl/logs/mutation_defaults.txt`).
- 이미지 `amr-ai/rl:0.3` (`sha256:eefb526d3d02…`, ROS 2 Humble, Nav2 1.1.20: nav2_dwb_controller,
  nav2_mppi_controller, nav2_regulated_pure_pursuit_controller 1.1.20). 읽기 전용으로 쓴다.
- 컨테이너 `amr-ai-dwa-abl-dev` (`--cpus 12`, `cpu_slot.sh dwa-abl` 슬롯 하나 안), ROS 도메인 61–63,
  `ROS_LOCALHOST_ONLY=1`. Release `-Werror` 빌드.
- **공유 부하**: 이 서버는 다른 사용자의 Gazebo(보너스 4 P5)·평가 작업이 함께 돈다 (등록 시점 loadavg 50–90 / 32 코어).
  ROS 시험대는 벽시계 실시간이라 부하가 결과에 들어간다 — 그 수치는 **공유 부하 하 예비값**으로 적고 실행마다
  loadavg 와 실제 명령 주기(cmd_rate_hz)를 남긴다. chain_sim 은 결정적이라 부하와 무관하다 (계산 시간 지표만 예외).

## 2. 시험대

| | A. chain_sim (`tools/chain_sim.cpp`) | B. ROS 운동학 시험대 (`kinematic_sim.launch.py` + `scripts/ros_bench.py`) |
| --- | --- | --- |
| 제어기 | 우리 코어 (DWA, PP) 를 직접 호출, 플러그인과 같은 파라미터 이름·기본값·경로 창(PlanWindow)·예외 규칙 | 실제 Nav2 controller_server 에 팔마다 플러그인 (우리 DWA·PP, DWB, MPPI, RPP) |
| 하류 사슬 | 실제 `VelocityProfiler` 50 Hz → 서보 100 Hz (지연 0.04 + 1 차 0.08 s + 가속 제한) | 실제 `velocity_profiler_node` 50 Hz → `kinematic_sim.py` 같은 서보 모형 |
| 코스트맵 | 잡음 없음: 상자 기하를 0.025 m 로 찍고 Nav2 식 팽창 (0.2 / 0.8 / 3.0 = 지역 코스트맵 값). 잡음 조건: 10 Hz σ 0.03 LiDAR 로 매 스캔 새로 만든 8 × 8 m 창 | 합성 지도(0.05 m)에 광선 투사한 σ 0.03 LiDAR → scan filter → 지역 코스트맵 0.025 m (항상 잡음 있음) |
| 시간 | 결정적 (100 Hz 고정 스텝) | 벽시계 실시간, 공유 부하 |
| 참값 충돌 | 로봇 사각형과 상자의 겹침 | `sim/footprint_clearance` ≤ 0 (지도 격자 기준, 반 셀 보정) |
| 이상 플랜트 | `plant=ideal`: 명령을 그대로 적분 | 없음 |

시험대 A 를 배포 cfg 로 튜닝 4 경로에 돌린 시간 21.30 / 13.60 / 14.80 / 9.80 s 는 기존 `tracking_sim`·ROS 측정
(dwa.md §8.1: 21.3 / 13.6 / 14.8 / 9.8 s)과 같다 (하네스 확인, §9).

## 3. 경로 집합 (`scripts/make_paths.py --seed 20261014`, 원자료 `runs/dwa-abl/paths/`)

- **튜닝 집합** (우리 DWA 를 튜닝한 4 경로, 최종 비교에 쓰지 않는다): 직선 20 m · U 턴 R 2 · S 자 R 3 · 좁은 통로 0.60 m
  (closed_loop_eval.py synthetic 배치 그대로).
- **held-out 60 경로**
  - 매끈한 무작위 32 개 (`heldout_smooth.json`): 절반 구간별 상수 곡률, 절반 구간별 선형 곡률. 매듭 간격 1.5–3.0 m,
    매듭 곡률은 30 % 확률로 0, 아니면 ±U(0.1, 0.7) 1/m. 길이 U(8, 24) m. 호길이 3 m 넘게 떨어진 두 점이 2.0 m 안이면
    버림 (34 번 뽑아 32 개). 빈 동쪽 마당 (70, 0) 에서 무작위 방향으로 출발.
  - 좁은 통로 8 개 (`heldout_narrow.json`): 폭 0.60/0.70 m, 길이 4/6 m, 진입 직선·90° 회전(R 1.5)·S 자 옆걸음 0.5 m,
    전역 경로 편향 0–4 cm. 랙 깊이 1.0 m.
  - 창고 A* 20 개 (`heldout_astar.json`): 합성 창고에서 여유 ≥ 0.45 m 인 무작위 시작·목표 (직선거리 ≥ 5 m) 24 쌍을
    뽑아 배포 planner_server(AStar)로 계획, 순서대로 앞 20 개 (실패 0). 길이 9.9–56.6 m. 시작·끝 자세는 경로 접선.
  - 시작 자세는 모두 경로 첫 점·접선 (출발 오프셋 없음).
- **기전 탐침** (표 3 전용, held-out 아님): 빈 마당 직선 30 m.

| 파일 | sha256 |
| --- | --- |
| tuning.json | 1ffdf0e581f4c05b607fdd1eaf4255b47062e7eb5eed8b409d7ed7b4a6e3e179 |
| heldout_smooth.json | 3b75380b8ad0e895ae7542e0e5ceeacb682403d67e2362e488262b37ed2bd359 |
| heldout_narrow.json | e4bb33fb98513a99e08b974606f8809b5c6b21da5b9f042b164812f781fed6e5 |
| heldout_astar.json | 819d8c952a57cef156a8d32aeb6ee7e4dc80a9d0c85d843ba024c5f5b3d0fb07 |
| astar_pairs.json | aef0b4d40add9116fdb4d34746bc2a1ed3fa2569e7cd7c790b8cdc38dcc9e8fb |
| probe.json | fdbe05c6d7f764e2b7975a41c2ad259f1abdd5dac23fea6b98669aaa2c6f86de |
| composite.pgm / .yaml | 4b5c315b… / c65d6116… |
| scenes_heldout_smooth / narrow / astar.txt | bef5ca2e… / 3fb95ffb… / 0e5f339e… |

## 4. 팔 (`scripts/params.py`)

### 4.1 우리 DWA 절제 팔 — 배포 `nav2_params.yaml` DWA 블록 위에 덮는 값

| 팔 | 끄는 수정 (끈 쪽 = 교과서 쪽) | 덮는 값 |
| --- | --- | --- |
| dwa_full | 없음 (배포) | – |
| dwa_no_m1 | M1 비용 재설계 → Fox 1997 3 항 (방향: 제동점 = 후보 원호를 따라 \|v\|Δt_c + v²/2a 간 자세, 목표점 = 로봇 투영점에서 경로 1.2 m 앞; 여유: 롤아웃 최대 **절대** 비용/252; 속도: \|v_des − v\|/v_span, v_des = 상한을 M2 접근 법칙으로만 깎은 값). 경로·이탈·진동 항 0 | textbook_cost true |
| dwa_no_m1c | M1-c 상대 여유 → 절대 여유 | relative_clearance false |
| dwa_no_m1e | M1-e 추종 지평 0.8 s → 전체 롤아웃 | path_eval_time 0 |
| dwa_no_m2 | M2 저크·지연 정지거리 역함수 목표 접근 → 없음 | goal_approach none |
| dwa_m2_trap (보조) | M2 → 사다리꼴 √(2ad) | goal_approach trapezoid |
| dwa_no_m3 | M3 창 중심 직전 명령 → 측정값 | window_center_measured true |
| dwa_no_m4 | M4 제동 후보·속도별 지평 → 격자만(유효 샘플 없으면 0 명령)·고정 지평 1.5 s | brake_candidate false, sim_time_min = max = 1.5 |
| dwa_no_m4a | M4-a 속도별 지평만 → 고정 1.5 s | sim_time_min = max = 1.5 |
| dwa_no_m5 | M5 좁은 곳 재중심 → 끔 | recenter_narrow false |
| dwa_textbook | 전부 끔 (위 전부 + M6·M7 끔: VO 끔, 동적 가중 0, 양보 끔) | |

- **M4-a 는 배포 조건에서 상수다** (코드: T_sim = clip(\|v\|/a + 0.5, 1.5, 2.5), a = 1.0, \|v\| ≤ 1.0 → 늘 1.5 s).
  그래서 dwa_no_m4a 는 dwa_full 과 같은 명령열을 내야 한다 (단위 시험 `SpeedDependentHorizonIsConstantAtDeployedLimits`).
  지평이 실제로 변하는 조건(적재 가속 축소, 1.0 m/s 초과)은 이 연구에 없다. 지평 몫은 "배포 조건에서 0" 으로 보고한다.
- **M6·M7 (동적 장애물 VO/TTC·횡단 양보)**: 이 경로 집합에 움직이는 장애물이 없어 **미측정 — Gazebo 필요**.
  kinematic_sim 에 왕복 등속 원판이 있지만 완전 트랙·비반응·고정 주기라 시나리오 08 의 기전(반환·추적 잡음)을 재현하지
  못해 범위에서 뺀다. 교과서 팔에서 M6·M7 을 끈 것은 움직이는 장애물이 없으므로 결과에 영향이 없어야 한다.
- M8 (velocity_profiler) 은 절제하지 않는다 — 모든 팔이 지나는 조건 자체다. 사슬 유무는 이상 플랜트(표 3)로 본다.
- 우리 PP (`pp`) 는 배포 PurePursuit 블록 그대로.

### 4.2 Nav2 비교 제어기 — 기본과 공정 튜닝

"기본" = Nav2 Humble 문서·nav2_bringup 의 예시 블록 그대로 + 로봇 한계만 바꿈 (최고 1.0 m/s, **후진 없음**,
ω 1.5 rad/s, 가속 1.0 m/s², 각가속 2.0 rad/s², DWB xy_goal_tolerance 0.10). 출처: 이미지의
`nav2_bringup/params/nav2_params.yaml` (DWB), navigation2 humble 브랜치 README (MPPI sha256 eef92363…,
RPP 06cee7db…, `runs/dwa-abl/ref/`). MPPI 의 기본 vx_min −0.35 는 0 으로 바꿨다 (모든 팔 후진 없음).

튜닝 예산: 제어기마다 **설정 12 개 (기본 포함)** 의 작은 격자, 근거 있는 손잡이만:

| | 손잡이 (근거) | 설정 |
| --- | --- | --- |
| DWB | sim_time (예측 지평), PathDist (추종), RotateToGoal (목표 접근·정렬), trans_stopped_velocity, vθ 샘플, 지난 튜닝 DWB_E (dwa.md §8.1) | D00 기본, D01 DWB_E, D02 sim 1.0, D03 sim 1.5, D04 PathDist 48, D05 PathDist 64, D06 RotateToGoal 완화(1.0, 0.3 s), D07 trans_stopped 0.1, D08 vθ 40, D09 DWB_E + PathAlign 16, D10 DWB_E + sim 1.0, D11 DWB_E + PathDist 64 |
| MPPI | PathFollow·PathAlign·PathAngle 가중 (추종), temperature, 샘플 표준편차 (1.0 m/s 탐색), Goal 임계 (목표 접근), time_steps, batch | P00 기본, P01 PathFollow 10, P02 PathAlign 20, P03 둘 다, P04 temperature 0.15, P05 vx_std 0.3·wz_std 0.6, P06 GoalAngle 임계 0.25·Goal 1.0, P07 time_steps 40, P08 batch 1000, P09 PathAngle 4, P10 P03 + std, P11 P03 + temperature |
| RPP | lookahead (속도 비례 여부·길이), approach_velocity_scaling_dist·min_approach (목표 접근), 곡률 감속 반경, 배포 RPP 블록 | R00 기본, R01 배포 블록, R02 속도 비례 lookahead, R03 속도 비례 0.4–1.2 m·0.8 s, R04 접근 1.5 m, R05 접근 2.0 m·최저 0.02, R06 곡률 반경 0.5, R07 R03 + 접근 1.5, R08 R03 + 접근 2.0·0.02, R09 곡률 감속 끔, R10 R01 + 접근 1.5, R11 R01 + 접근 2.0·0.02 |

**고르는 기준 (튜닝 집합 4 경로, ROS 시험대, 설정마다 1 회)**: ① 성공 경로 수 최대, ② 그중
J = 경로 평균 [CTE 최대/10 cm + \|정지 오차\|/10 cm + T/T_lb] 최소 (실패 경로의 항은 10, T_lb = L/v + v/a + a/j),
③ 이름 순. 코드 `analyze.py select`. 고른 설정을 held-out 에 **한 번** 적용한다. 우리 DWA·PP 는 배포 값 그대로
(이미 같은 튜닝 집합에서 튜닝됐다 — 그래서 튜닝 집합 결과는 비교에 쓰지 않는다).

## 5. 지표 (실행마다, 두 시험대 같은 이름)

- **성공**: 목표 판정 통과 + 참값 충돌 없음. 실패 원인: 진행 검사(no_progress/aborted), 시간 상한(max(60, 3·T_pred + 30) s),
  제어기 중단, 충돌.
- **CTE** [cm]: 참값 위치 ↔ 기준 경로, 직전 투영 근처 창에서 찾음. 직선/곡선 = 경로 곡률 > 0.1 1/m 정점 ± 0.5 m.
  경로마다 평균·최대. 성공한 실행만 비교에 쓴다.
- **주행 시간** T (목표 수락 → 판정 통과), 비교는 log(T_a/T_b) 짝 차. 표에는 T/T_lb.
- **목표 정지 오차**: 정지 뒤 위치의 목표 접선 방향 성분(+ 지나침, − 못 미침), 가로 성분, 거리. **정착 시간**: 판정 통과 →
  참값 \|v\|, \|ω\| < 1e-3.
- **명령 저크 p99**: 제어기 출력(cmd_vel_nav, 20 Hz 2 차 차분)과 프로파일러 출력(50 Hz). 프로파일러 출력은 모든 팔에서
  저크 한계 2 m/s³ 로 잘리므로 제어기 출력 쪽이 팔을 가른다.
- **좁은 통로**: 관통(= 성공) 수, 참값 최소 여유.
- **계산 시간**: 우리 팔은 플러그인 cycle_ms (chain_sim 은 코어 호출 시간). Nav2 팔은 controller_server 프로세스 CPU 시간
  증가분 − 유휴 10 s 비율 × 실행 시간, 명령 수로 나눈 근사 [ms/주기] (코스트맵 갱신 일부가 남을 수 있다 — 근사).
- **도착 시간 예측 오차** \|T − T_pred\|/T: **보조로만**. 예측 모형(SpeedProfile)이 우리 DWA 에 맞춰져 있어 우리 쪽에
  유리하게 편향된다.
- 기전 지표 (표 3): 탐침 직선에서 0 → 0.9 m/s 시간, a_eff = 0.9/그 시간.

## 6. 조건과 반복

- **잡음 없음** (주 조건): chain_sim 결정적 1 회, ROS 1 회.
- **잡음 조건** (seed 1–5):
  - chain_sim: 오도메트리 백색 σ_v 0.02 m/s · σ_ω 0.02 rad/s (제어기·프로파일러 되먹임 각각), 위치추정 가우스-마르코프
    σ 0.01 m · 0.005 rad · τ 1 s (제어기·목표 판정 입력), 상자가 있는 장면은 σ 0.03 LiDAR 코스트맵. 절제 팔 전부, held-out 60.
  - ROS: `kinematic_sim` 오도메트리 σ_v 0.02 · σ_ω 0.02, noise_seed 1–5 (LiDAR σ 0.03 은 원래 늘 있다, 위치 잡음은 도구에
    없다). 부분집합 12 경로 (S01–S06, N1·N4·N6, 길이 20 m 미만인 앞 A* 셋 A05·A06·A12) × 표 1 팔.
    ROS 의 seed 는 실시간 실행이라 잡음 열이 재현되지 않는다 — 반복 번호로 쓴다.
- **이상 플랜트**: chain_sim, dwa_full·no_m3·no_m2·textbook·pp, held-out 60.
- **제어 주기 10 / 40 Hz**: chain_sim 사슬, dwa_full·no_m3, held-out 60. 탐침은 사슬·이상 × 10/20/40 Hz.
- ROS 시험대 팔 (held-out): dwa_full, dwa_textbook, pp, DWB_D00, MPPI_P00, RPP_R00, 튜닝으로 고른 셋, 그리고 절제 확인
  dwa_no_m1·no_m2·no_m3. 경로를 셋으로 나눠 도메인 61–63 에서 동시에, 경로마다 팔 순서를 경로 번호만큼 돌린다.

## 7. 통계

- 표본 단위 = **경로**. 결정적 시뮬레이터라 반복 대신 경로 수로 표본을 늘린다. 잡음 seed 는 경로 안에서 먼저 평균한다.
- 제어기·팔 비교 = 같은 경로의 짝 차. 구간 = 경로 묶음 재표집 부트스트랩 (B = 10 000, seed 12345, 백분위 95 %).
- 절제 = "전체 − 하나 끔" 짝 차. 결과가 경로 전부에서 비트 단위로 같으면 "효과 0 (동일)" 으로 따로 센다.
- 다중 비교는 **기술**(구간)로만 한다. 유의성 판정은 아래 H1 하나뿐이다.

### 7.1 확증 주장 (하나)

**H1.** chain_sim 사슬 20 Hz, held-out 60 경로, 잡음 없음에서 **M3 를 끄면(창 중심 = 측정값) 주행 시간이 길어진다.**
통계량 = 두 팔이 모두 성공한 경로의 log(T_no_m3 / T_full) 평균. **판정: 부트스트랩 95 % 구간의 하한 > 0 이면 확증**,
아니면 "확증 안 됨". 두 팔의 성공 수도 함께 보고한다 (M3 끔이 실패를 늘리면 시간 비교에서 빠지는 경로가 생긴다).

### 7.2 미리 적어 두는 예상 (기술용, 판정 아님)

- 탐침 직선 사슬 20 Hz 에서 M3 끔의 a_eff 는 한계의 0.2–0.45 배, 이상 플랜트에서는 켬과 같다 (명령열 동일).
- held-out 에서 M3 의 시간 효과는 이상 플랜트에서 \|비\| < 1 %.
- a_eff(M3 끔) 는 제어 주기 Δt_c 에 거의 비례한다 (10/20/40 Hz) — 설명 모형 a_eff ≈ a·Δt_c/τ_eq. 맞지 않으면 모형을 주장하지 않는다.
- M4 (지평 부분)는 모든 경로에서 동일, M4 제동 후보는 동일에 가깝다.
- M2 를 끄면 목표에서 지나치거나 되돌아오느라 시간이 늘고, 사다리꼴은 그 중간.

## 8. 실행 계획 (`scripts/orchestrate.py`)

1. ROS 튜닝 (`ros-tuning`): 도메인 61 DWB 12 + dwa_full, 62 MPPI 12 + pp, 63 RPP 12 + dwa_textbook, 튜닝 4 경로.
   → `analyze.py select` → `raw/tuning_selection.json`.
2. chain_sim 전부 (`chain`): 튜닝 집합(교정), held-out 잡음 없음·잡음·이상·10/40 Hz, 탐침.
3. ROS held-out (`ros-heldout`), 4. ROS 잡음 (`ros-noise`).
5. `analyze.py tables` → `results.md`.

- 부하 때문에 실제 명령 주기가 18 Hz 아래로 떨어진 ROS 실행은 지우지 않고 표시하며, 표 1 을 그 실행을 뺀 민감도 표로도 낸다.
- 중단되면 `--skip-done` 으로 같은 (경로, 팔, seed) 를 건너뛰고 이어서 돈다.

## 9. 등록 전에 이미 본 것 (정직성)

하네스 확인용 파일럿을 **튜닝 집합에서만** 돌렸고 그 결과를 보고 이 문서를 썼다:
- chain_sim, 튜닝 4 경로, 절제 팔 전부와 이상 플랜트 dwa_full·no_m3 (`runs/dwa-abl/tmp/pilot2.jsonl`). 예: M3 끔 직선 20 m
  21.3 → 36.2 s, 0 → 0.9 m/s 1.57 → 3.99 s, 이상 플랜트에서는 켬·끔 결과가 같았다; M1 끔은 U 턴 곡선 CTE 평균 0.5 → 8.1 cm,
  좁은 통로를 우회해 33.8 s; M1-c 끔과 교과서 팔은 좁은 통로 입구에서 진행 검사 실패; M2 끔은 목표를 지나친 뒤 되돌아와
  직선 40.3 s.
- ROS, 튜닝 4 경로, dwa_full·pp·DWB_D00·MPPI_P00·RPP_R00 각 1 회 (`raw/pilot_ros.jsonl`, loadavg 51–94). 이 파일럿은
  튜닝 선택에 쓰지 않는다 (선택은 §8.1 의 정식 실행으로만).

H1 을 M3 로 고른 것은 이 파일럿과 선행 연구 보고서(측정 중심 창의 실효 가속 약 1/3 이 문헌에 정량이 없는 몫)를 보고 정했다.
held-out 결과는 아직 없다.

## 10. 분석·도구 파일 (커밋 `b02bf54`)

| 파일 | sha256 |
| --- | --- |
| scripts/analyze.py | 4463400f2e6288d3b7a2f6f6573c72fd6dca68d0c4ba52de82c599d694b6736c |
| scripts/orchestrate.py | 0c7fcae8207230b9d1a1b56e753747571da6fe4b9471e5f05d4eddadaff6dbdb |
| scripts/params.py | 844e8bec4e53e292e5cff68b20e3565d748fcd0f150a6d3fad07dd599dae33f9 |
| scripts/ros_bench.py | ef6fa66a09d80f472ee561d2c61f3307497365e9df96a3704a6a70565ca6645e |
| scripts/make_paths.py | d78985759902f68161d917e163a69c8b6231bb2fc9a226b8e198578dfed9e305 |
| scripts/plan_astar.py | 45d2eafdbedbdb3f090e8374e5e09a6fa6e17053b10b53871a36f8957626b95c |
| scripts/ros_worker.sh | 0b1613f7284fbcb51b377bfec2f9629c902db0df799038531d412add6f5439c0 |
| src/amr_navigation/tools/chain_sim.cpp | 5e104cf336e15f4effc2f7e8d058ec1e6343dc408f024386ff868a81b2cc7961 |
