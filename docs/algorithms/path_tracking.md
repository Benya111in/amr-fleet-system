# 경로 추종 (Pure Pursuit)과 속도 제어 (S-curve 프로파일 + PID) — 명세 4.5

> 구현 (ROS 비의존 코어): `core/pure_pursuit.hpp`, `core/speed_profile.hpp`, `core/jerk_limiter.hpp`,
> `core/pid.hpp`, `core/velocity_profiler.hpp`.
> ROS: `amr_navigation::PurePursuitController` (`nav2_core::Controller`, `controller_id: PurePursuit`),
> `velocity_profiler_node` (`cmd_vel_nav` → `cmd_vel_smoothed`, 50 Hz).
> 설정: `config/nav2_params.yaml` `controller_server.PurePursuit.*`, `config/velocity_profiler.yaml`.
> 연구 브리프: `research/path-tracking-control` §2 (기준 PP·PI·S-curve)를 구현했고, §4.1 CC-PP 를
> `use_chord_correction` 옵션으로 넣었다 (실제 지연 체인에서 이득이 없어 기본은 끔, §1.5).
> 튜닝 도구: `ros2 run amr_navigation tracking_sim pp key=value …` (ROS 없는 폐루프, [dwa.md](dwa.md) §4.1).

속도 명령 체인 (components.md §4.1):
`controller_server (DWA | PurePursuit) --cmd_vel_nav--> velocity_profiler_node --cmd_vel_smoothed--> safety_node --cmd_vel--> DiffDrive`

## 1. Pure Pursuit

### 1.1 기하 유도

로봇 좌표계에서 look-ahead 점 G = (x_g, y_g), ‖G‖ = L. 로봇 원점에서 x 축에 접하며 G 를 지나는 원의 반지름 R:
원의 중심 (0, R) → x_g² + (y_g − R)² = R² → x_g² + y_g² = 2 y_g R → **κ = 1/R = 2 y_g / L² = 2 sin α / L**
(α = G 방향각). 각속도 **ω = v·κ**. 로봇이 반지름 R_p 원 경로 위에 있으면 G 도 같은 원 위이므로 κ = 1/R_p 가
정확히 나온다 (원 경로 정확성, 단위 테스트 `PurePursuit.CurvatureFormula`: 여러 R, α 에서 1e-12).
정상상태 해석: 로봇이 반지름 r 동심원을 돌 때 G 가 반지름 R 원 위에 있으면 κ = (r² − R² + L²)/(r L²) = 1/r ⇔ r = R
→ **원 경로에서 정상상태 CTE = 0**.

### 1.2 직선 선형화와 속도 적응 look-ahead

직선 경로에서 횡 오차 e, 방향 오차 ψ 가 작을 때 y_g ≈ −e − Lψ 이므로 ω = −(2v/L²)(e + Lψ), ė = vψ, ψ̇ = ω →
**ë + (2v/L) ė + (2v²/L²) e = 0**: ω_n = √2·v/L, ζ = 1/√2 (L 과 무관하게 일정 감쇠, 오버슈트 4.3 %).

**L = clamp(k·|v|, L_min, L_max)**, k = `lookahead_time` 0.8 s, L_min 0.4 m, L_max 1.8 m.
- L ∝ v 이면 ω_n = √2/k = 1.77 rad/s 로 **속도와 무관한 게인 스케줄** (명세 "Look-ahead 거리, 게인 파라미터를 속도에
  따라 적응적으로 조정").
- 하한 L_min 0.4 m: 위치추정 잡음 증폭 억제 (∂ω/∂e = 2v/L², 브리프 §2.4 의 σ_e 0.04 m, σ_ω,max 0.15 rad/s 에서
  v = 0.2 m/s 의 L_min = 0.33 m → 0.4 로 올림). 저속에서 L_min 이 활성.
- 상한 L_max 1.8 m: 로컬 코스트맵 반폭(6 m)보다 작고, R = 1 m 곡선에서 Lκ ≤ 0.9 를 넘지 않게.
- 단위 테스트 `LookaheadClamping`, `LookaheadPointOnCircle` (선분-원 교점, 원 반지름 = L 1e-9).

### 1.3 look-ahead 점 탐색

투영 선분(직전 인덱스 부근 창)부터 앞으로 가며 **로봇 중심 거리 L 인 첫 교점**(선분-원 2차식의 큰 근)을 찾는다.
끝점이 원 안이면 경로 끝점. 투영은 경로 frame(map)에서 한다 — 로봇 자세를 TF 로 경로 frame 에 옮기므로 경로를
매 주기 변환하지 않는다.

### 1.4 속도 조절 (곡률·목표·방향)

v = min(v_des, v_limit, v_allow(s), v_κ(κ)) × max(0, cos α), ω = v·κ (|ω| > ω_max 이면 ω 포화 + v = ω_max/|κ| 로 곡률 보존)
- **v_allow(s)**: 경로 속도 상한 프로파일의 앞보기 감속 원뿔 `SpeedProfile::allowedSpeed`
  v_allow(s) = min_{s_k ≥ s} √(v_cap,k² + 2·a_dec·(s_k − s)),
  v_cap,k = min(v_des, ω_max/|κ_k|, √(a_lat/|κ_k|)). **목표 정점만** 사다리꼴 대신 저크·지연을 넣은 정지거리의
  역함수 d_stop⁻¹(s_goal − s) (`approach_latency` t_c 0.3 s) — 하류 저크 필터 때문에 사다리꼴 접근은 목표를
  40 cm 지나친다 (튜닝 표: [dwa.md](dwa.md) §1.6, 단위 테스트 `GoalApproachThroughJerkFilterStopsInsideTolerance`).
  연구 브리프 §4.2 (JRG) 가 같은 현상("원뿔은 궤적이 아니다", 인과 저크 필터가 a²/2j 뒤처짐)을 지적하고, 프로파일러
  필터의 **미러 상태** (v_f, a_f) 로 결속점마다 제동 시작을 온라인 판정하는 거버너를 제안한다. 여기서는 기준선으로
  목표점에만 지연 t_c 를 넣은 정지거리 역함수를 적용했다 — 곡선 진입(비영 상한 결속점)은 사다리꼴 원뿔 그대로라
  곡선 앞 감속이 약간 늦을 수 있다 (폐루프 곡선 CTE 는 목표 안, §5). JRG 는 `SpeedProfile` 에 결속점 집합과
  미러 필터(`JerkLimitedFilter` 재사용)를 더하는 확장 지점이다.
  곡률 κ_k 는 앞뒤 0.25 m 창 이산 곡률 (격자 양자화 잡음 억제). a_lat = 0.8 m/s² (적재물 안정).
- **v_κ(κ)**: 현재 look-ahead 곡률의 즉시 상한.
- **cos α**: look-ahead 방향 오차가 크면 감속.
- 모드: 방향 오차 > 45° 이고 거의 정지(≤ 0.1 m/s) → 제자리 회전 ω = sign·min(ω_max, √(2α_max|θ|)) (0 에 정지하는
  감속 곡선), 목표 0.08 m 안 → 목표 방향 정렬, 0.02 rad 안 → 완료.
- 단위 테스트 `SteersTowardPathAndModes`, `SpeedRegulation`, `RotationCommand`.

### 1.5 CC-PP (chord-corrected, 브리프 §4.1, 옵션 `use_chord_correction`, 기본 끔)

기본 PP 는 곡률이 바뀌는 곳(직선→곡선 진입)에서 look-ahead 점이 이미 곡선 위라 미리 안쪽으로 파고든다.
CC-PP 는 **완벽 추종 시의 기준 현** G⁰ (투영점에서 경로를 따라 L 앞 점, Frenet 좌표 y_g⁰)을 빼고 경로 곡률을
피드포워드한다: **κ = κ_path(s) + K·2(y_g − y_g⁰)/L²**. 직선·원에서는 y_g = y_g⁰ 가 정상상태이므로 기본 PP 와 같고,
곡률 변화 구간에서만 다르다.

폐루프 비교 — 이상 차동구동(명령 즉시 반영) 단위 테스트 `ClosedLoopCurveWithinSpec` (직선 3 m → R 2 m 좌 90° →
직선 3 m → R 1.5 m 우 90° → 직선 3 m, 1.0 m/s)와, 실제 명령 체인(`VelocityProfiler` 저크 필터 + 서보 지연 0.04 s +
1차 0.08 s)을 넣은 `tracking_sim` (U 턴 R 2 m, S 자 R 3 m 물결):

| 제어기 | 이상 플랜트: 직선 / 곡선 CTE 평균 (최대) | 지연 체인 U 턴 곡선 평균 / 최대 | 지연 체인 S 자 곡선 평균 / 최대 |
| --- | --- | --- | --- |
| **PP (기본, 채택)** | 0.92 (4.21) / 1.74 (4.24) cm | **0.4 / 2.0 cm** | **0.9 / 2.7 cm** |
| CC-PP | **0.15 (0.85) / 0.24 (0.75) cm** | 0.6 / 3.1 cm | 1.3 / 4.2 cm |

이상 플랜트에서는 CC-PP 의 곡률 피드포워드가 곡선 진입 오차를 1/7 로 줄이지만, 실제 체인에서는 피드포워드한
곡률이 지연(≈ 0.1 s + 저크 램프) 뒤에 실현되어 이득이 사라진다(오히려 0.2–0.4 cm 나쁨). 두 방식 모두 명세
(곡선 10 cm)의 1/20 이하이므로 설명이 단순한 **기본 PP 를 채택**하고 CC-PP 는 `use_chord_correction` 옵션으로
남겼다 (지연 보상 — 예: 피드포워드 곡률을 t_c 앞 호길이에서 읽기 — 이 확장 지점).

### 1.6 Nav2 Regulated Pure Pursuit (RPP) 와의 비교

| | 본 PurePursuitController | nav2 RPP (Humble 1.1.20) |
| --- | --- | --- |
| look-ahead | clamp(k·v) — 같은 식 (`use_velocity_scaled_lookahead_dist`) | 같음 |
| 곡률 감속 | 경로 곡률 프로파일의 **앞보기 감속 원뿔** (a_dec 로 곡선 전에 미리 감속) + √(a_lat/κ) | 현재 look-ahead 곡률 반경 < `regulated_linear_scaling_min_radius` 이면 비례 감속 (앞보기 없음) |
| 목표 접근 | 저크·지연 포함 정지거리의 역함수 (하류 저크 필터를 고려) | `approach_velocity_scaling_dist` 안에서 거리 비례 (하류 필터 지연 고려 없음 → §5 에서 28–32 cm 지나침) |
| 곡률 변화 | CC-PP 피드포워드 옵션 (기본 끔) | 없음 |
| 충돌 | 명령 원호를 max(1 s, v/a) 동안 풋프린트 검사 → 예외 | carrot 까지 원호 검사 |
| 가감속 | 하류 velocity_profiler (저크 제한) | 자체 없음 (velocity_smoother) |

두 제어기 모두 같은 controller_server 에 올라가 있어 (`controller_id: RPP`) 같은 경로로 CTE 를 비교했다 (§5).

## 2. 경로 속도 프로파일과 주행 시간 예측 (명세 4.4 "예측 시간 오차 15 %")

`SpeedProfile::predictTravelTime` (Python 판 `path_metrics.predict_travel_time`, 같은 식):
1. 정점별 곡률 상한 v_cap,i (위 식), 끝점 0.
2. 전진 패스 v_i = min(v_cap,i, √(v_{i−1}² + 2aΔs)), 후진 패스 v_i = min(v_i, √(v_{i+1}² + 2dΔs)) — 사다리꼴 프로파일.
3. T = Σ 2Δs/(v_i + v_{i+1}) (등가속 구간에서 정확) + **a/j** (S-curve 보정: 정지→출발 한 쌍에서 가속·감속 각 a/(2j))
   + τ_rot(출발 방향 오차) + τ_rot(도착 방향 오차), τ_rot(θ) = θ/ω + ω/α (사다리꼴 회전) 또는 2√(θ/α).
4. 단위 테스트 `TravelTimePrediction`: 4 m 직선 L/v + v/a + a/j = 5.5 s (1e-6), Python 교차 검증
   `test_travel_time_matches_trapezoid_and_cpp`.
5. 폐루프 실측 오차 평균 2.2–5.0 %, 최대 8.1 % (§5.2). 브리프 §4.2 는 "같은 거버너 + 필터 코드를 경로 위에서
   오프라인 재생" 한 ETA 를 권한다 — 목표점 저크 접근·제자리 회전과 전진의 겹침까지 반영되므로 남은 계통 오차(PP 가
   예측보다 빠른 쪽)를 줄일 확장 지점이다 (`tracking_sim` 의 체인 시뮬레이션이 그 재생기의 원형).

## 3. 저크 제한 S-curve 필터 (velocity_profiler_node)

상태 (v, a), 목표 v_ref, 한계 |a| ≤ a_max (1.0), |ȧ| ≤ j_max (2.0) — robot_params.yaml `limits`.
스위칭면 σ(v, a) = (v_ref − v) − a|a|/(2j) ("지금부터 최대 저크로 a → 0 하면 v_ref 에 닿는가")에 다음 스텝에서
정확히 착지하는 가속도를 폐형식으로 구한다:

- c = v_ref − v − a·dt/2
- a₁* = sign(c)·j·(−dt/2 + √(dt²/4 + 2|c|/j))   (a₁|a₁|/(2j) + a₁·dt/2 = c 의 해)
- j_k = clamp((a₁* − a)/dt, −j, j), a₁ = clamp(a + j_k·dt, −a_max, a_max), v ← v + (a + a₁)·dt/2 (사다리꼴 적분)
- 종단: |v_ref − v| ≤ j·dt² 이고 |a| ≤ j·dt 이면 a = 0, v = v_ref.

| 검증 (`test_profiler.cpp`) | 결과 |
| --- | --- |
| 무작위 목표 20 만 스텝 (50·100 Hz, 한계 밖 목표 포함) | max \|a\| = 1.0000, max \|j\| = 2.0000 (한계 초과 0 회), \|Δv\| ≤ a·dt |
| 0 → 2 m/s 도달 시간 (이론 v/a + a/j = 2.5 s) | 2.5 s ± 0.02, 오버슈트 0 |
| 0 → 0.3 m/s (a_max 미도달, 이론 2√(v/j) = 0.775 s) | ± 0.03 s |
| j_max = 0 (사다리꼴 모드) | 0 → 1 m/s 1.0 s |

**곡률 보존**: 선속도를 저크 제한하는 동안 ω_ref = (ω*/v*)·v_ref 로 두어 경로 곡률을 그대로 유지한다 (가속 중에도
DWA/PP 가 의도한 원호를 따름, `CurvaturePreservedDuringAcceleration`: |ω/v − κ| < 1e-9). |v*| ≤ 0.05 m/s 면 제자리
회전으로 보고 각속도를 별도 저크 제한(α 2.0, 각저크 6.0).
**적재 질량**: `payload/mass` → 가속·저크 한계 × m/(m + m_payload) (25 kg → 0.656, `PayloadScaleSlowsAcceleration`).
**재동기화**: 필터 기준과 측정 차 > 0.4 m/s (safety_node 급감속·E-stop 뒤) → 필터를 측정값으로 맞추고 적분기 리셋.
**정지 착지**: 목표·기준이 0 이 되는 순간(정지 데드밴드) 남은 PI 보정(슬립 플랜트에서 수 cm/s)을 바로 끊으면 명령에
계단이 생긴다(폐루프 실측 저크 5–12 m/s³ 스파이크). 직전 출력 상태 (v, a) 에서 같은 저크 필터로 0 에 착지시킨 뒤
정확히 0 을 낸다 (`StopLandsResidualCorrectionWithinJerkLimit`: 이득 0.8 플랜트, 잔여 보정 0.12 m/s → 착지 구간 저크
최대 2.000 m/s³).
**PI 보정과 저크**: 최종 명령 = 기준(저크 ≤ j 보장) + PI 보정이므로, 보정이 기준의 최대 저크 램프에 더해지는 순간에는
명령 저크가 한도를 약간 넘을 수 있다 (폐루프 p99 2.0–2.2 m/s³, §5). 최종 출력에 2차 차분 가드(오차 채널 시간 최적
착지)를 두는 방식을 시험했으나, 가드의 기울기 관성이 PI 루프 안에서 한계 주기 진동(불일치 플랜트에서 ±0.2 m/s)을
만들어 채택하지 않았다 — 적재물 안정의 기준은 저크가 보장된 **프로파일 기준**과 실측 플랜트 저크(§5)로 본다.

## 4. PID 속도 제어기와 게인 튜닝 (명세 "PID 기반 속도 제어기, 게인 튜닝 과정 문서화")

### 4.1 구조: 피드포워드 + 모델 추종 2-자유도 PI (+ 역계산 안티와인드업)

DiffDrive 는 바퀴 속도 서보이므로 몸체 속도 플랜트는 1차 지연 + 순수 지연으로 근사한다
**P(s) = K·e^{−T_d s}/(T_p s + 1)** (K ≈ 1, T_p ≈ 0.08 s 서보·접촉, T_d ≈ 0.04 s = 프로파일러 1 주기 + EKF·브리지).

**u = r + PI(y_m − y)**,  y_m = M(r), M(s) = e^{−T_d s}/(T_m s + 1) (기준 모델 = 명목 플랜트),  y = `odometry/filtered`
- 피드포워드 r 이 명목 추종을 맡는다. PI 는 **기준 모델 응답과 측정의 차**만 본다 → 플랜트가 모델대로면 오차 0
  (스텝 오버슈트 0), 적재·슬립·서보 포화로 생긴 차이만 보정한다.
- 한 주기 정렬: 주기 k 의 측정은 k−1 까지의 명령이 만든 응답이므로 기준 모델도 이번 기준을 넣기 전 출력과 비교한다
  (정렬하지 않으면 명목 플랜트에서도 0.5–1 % 오버슈트가 생겼다 — 튜닝 1차 시도에서 발견).
- 역계산 안티와인드업: İ = K_i·e + (u − u_raw)/T_t, T_t = √(K_p/K_i). 보정 폭은 피드포워드 ± 0.2 m/s
  (± 0.3 rad/s)로 제한. 미분항 없음(K_d = 0: 측정 잡음 증폭).
- 대안(고전 2-DOF, u = r + K_p(b·r − y) + K_i∫(r − y)): 명목 플랜트에서도 피드포워드와 PI 가 같은 오차에 반응해
  0.15 m/s 스텝 오버슈트 20 % (K_p 0.3, K_i 3) — 채택하지 않음 (`use_reference_model: false` 로 재현 가능).

### 4.2 튜닝 절차 (재현: `ros2 run amr_navigation pid_tuning out.csv`)

1. **플랜트 모델**: T_p 0.08 s, T_d 0.04 s, K 1 (브리프 §2.6.1 추정 범위 0.05–0.10 / 0.04–0.06 의 중앙).
   실제 값은 Gazebo 에서 0.15 m/s 스텝 응답 최소자승 식별로 갱신한다 (`model_time_constant`, `model_delay`).
2. **격자 탐색**: K_p ∈ {0, 0.1, …, 1.0} × K_i ∈ {0.5, …, 8} (56 쌍), 50 Hz 이산 시뮬레이션:
   - os_nom: 명목 플랜트 0.15 m/s 스텝 오버슈트
   - os_worst: 불일치 플랜트 27 종 (K 0.8/0.9/1.0 × T_p 0.05/0.08/0.12 × T_d 0.02/0.04/0.06) 최악 오버슈트
   - t_dist: 1 m/s 주행 중 입력 외란 −0.1 m/s (경사·적재 저항) 회복 시간 (|e| < 0.01)
   - t_gain: 이득 0.8 플랜트(슬립)에서 0.5 m/s 정상오차 < 0.01 까지 시간
   - noise: 측정 잡음 σ 0.014 m/s (명세 4.1 슬립 잡음, v = 2 m/s 기준)가 만드는 명령 표준편차 (폐루프)
3. **선택 규칙**: os_nom ≤ 1 %, os_worst ≤ 5 %, noise ≤ 0.0075 m/s (브리프의 지령 잡음 예산 0.007 수준) 중
   t_dist + t_gain 최소.

결과 (발췌, 전체 56 행은 도구 출력):

| K_p | K_i | os_nom [%] | os_worst [%] | t_dist [s] | t_gain [s] | noise [m/s] | 조건 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.0 | 1.0 | 0.00 | 4.32 | 2.22 | 2.80 | 0.0014 | 만족 (느림) |
| 0.3 | 1.0 | 0.00 | 2.86 | 2.62 | 3.20 | 0.0043 | 만족 (느림) |
| 0.3 | 2.0 | 0.00 | 5.20 | 1.28 | 1.56 | 0.0044 | os_worst 초과 |
| **0.4** | **2.0** | **0.00** | **4.60** | **1.34** | **1.64** | **0.0057** | **선택** |
| 0.5 | 2.0 | 0.00 | 4.06 | 1.40 | 1.68 | 0.0071 | 만족 (잡음 ↑) |
| 0.4 | 3.0 | 0.00 | 6.55 | 0.88 | 1.06 | 0.0058 | os_worst 초과 |
| 0.7 | 3.0 | 0.00 | 4.57 | 0.98 | 1.18 | 0.0100 | 잡음 초과 |

관찰: K_i 를 키우면 외란 회복이 빨라지지만 느린 서보(T_p 0.12)·긴 지연(0.06)에서 오버슈트가 커진다. K_p 는 잡음을
선형으로 키운다(≈ 0.014·K_p). 선택값 **K_p 0.4, K_i 2.0 s⁻¹, T_t 0.447 s** 은 브리프 초기값 범위(K_p 0.3–0.5)
안이다. 각속도 축도 같은 게인 (몸체 ω 서보가 같은 DiffDrive 라 같은 시정수).

단위 테스트 (`Pid.TunedGainsStepAndRampAcceptance`): 명목 스텝 오버슈트 ≤ 1 %, 불일치 8 종 ≤ 5 % 및 3 s 뒤 정상오차
< 0.01, 저크 제한 램프 0 → 1 m/s 에서 명목 플랜트면 PI 개입 0 (1e-9). `BackCalculationAntiWindup`: 2 s 포화 뒤
적분 0.50 vs 안티와인드업 없음 4.27, 회복 0.64 s vs 5.38 s.

## 5. 폐루프 검증 결과

**시험대** (`launch/kinematic_sim.launch.py` + `scripts/closed_loop_eval.py`): 실제 Nav2 서버(planner·controller·
behavior·bt_navigator, 본 플러그인) → `velocity_profiler_node` (50 Hz, PI) → `kinematic_sim.py` (정확 원호 적분 100 Hz,
DiffDrive 속도 서보 근사 = 순수 지연 0.04 s + 1차 지연 0.08 s + 가속 제한, 합성 창고 지도에서 LiDAR 광선 투사 10 Hz)
의 폐루프. CTE 는 참값(`ground_truth/odom`) 위치에서 기준 경로 폴리라인까지의 수직 거리, 직선/곡선 구분은 경로 곡률
> 0.1 1/m 정점 ± 0.5 m (천이대는 곡선). 위치추정 오차는 넣지 않았다 (map → odom 항등) — 제어기 자체 성능.
측정 시각 11:17–11:28 (KST), load average 2–18 (`uptime`; 외부 부하 종료 후).

### 5.1 경로 추종 (FollowPath 직접, 1.0 m/s)

| 제어기 | 경로 | CTE 직선 평균 / 최대 [cm] | CTE 곡선 평균 / 최대 [cm] | 주행 시간 실제 / 예측 [s] (오차) | 목표 통과량 [cm] | 저크 기준 최대 / 명령 p99 / 참값 p99 [m/s³] |
| --- | --- | --- | --- | --- | --- | --- |
| **PurePursuit** | 직선 20 m | 0.0 / 0.0 | – | 21.1 / 21.5 (1.9 %) | +0.4 | 2.00 / 2.08 / 2.15 |
| | U 턴 R 2 m | 0.2 / 1.7 | 0.4 / 1.9 | 13.5 / 13.8 (2.2 %) | +0.2 | 2.00 / 2.10 / 2.34 |
| | S 자 R 3 m | 0.7 / 1.2 | 0.8 / 2.6 | 14.7 / 15.0 (2.4 %) | +0.2 | 2.00 / 2.11 / 2.10 |
| | 좁은 통로 0.60 m | 0.0 / 0.0 | – | 9.6 / 10.0 (4.2 %) | +0.4 | 2.00 / 2.08 / 2.02 |
| RPP (Nav2, 비교) | 직선 | 0.0 / 0.0 | – | 20.8 / 21.5 (3.4 %) | **+28.3** | 2.00 / 2.09 / 2.02 |
| | U 턴 | 0.2 / 1.8 | 0.4 / 2.0 | 13.1 / 13.8 (5.3 %) | **+27.8** | 2.00 / 2.22 / 2.14 |
| | S 자 | 0.9 / 1.3 | 0.8 / 2.7 | 14.3 / 15.0 (4.9 %) | **+32.4** | 2.00 / 2.11 / 2.04 |

- **명세 판정 (4.5)**: CTE 직선 평균 ≤ 0.7 cm (목표 5 cm), 곡선 평균 ≤ 0.8 cm · 최대 2.6 cm (목표 10 cm) — 통과.
  좁은 통로에서 풋프린트 최소 여유 0.10 m (기하 최대치) — 충돌 없음.
- **RPP 와 비교**: 추종 정확도는 같은 수준(같은 look-ahead 식)이지만 RPP 는 목표 접근을 하류 저크 필터 지연 없이
  거리 비례로만 줄여 목표를 28–32 cm 지나쳤다 (목표 판정 xy 0.10 m 뒤 제동 꼬리). 본 제어기는 저크·지연을 넣은 접근
  속도(§1.4)로 +0.2–0.4 cm.
- **저크**: 프로파일 기준은 모든 시험에서 최대 2.00 m/s³ (= `limits.max_linear_jerk`, 구조적 보장). 최종 명령(기준 +
  PI 보정)의 p99 는 2.08–2.11 (PI 보정이 기준 램프에 더해진 몫, §3), 참값 속도(발행 시각 기준 0.2 s 평활 3차 미분)
  p99 1.87–2.34. 정지 착지 처리(§3) 뒤 정지 순간의 5–12 m/s³ 스파이크는 사라졌다.

### 5.2 계획 경로 주행 시간 예측 (명세 4.4 "예측 시간 오차 15 %")

NavigateToPose 로 창고 목표 12 개를 순회 (AStar 전역 경로 → 제어기, 1 Hz 재계획, 목표 방향 정렬 포함). 예측은
첫 계획 경로 + 출발·도착 방향 오차로 `path_metrics.predict_travel_time` (C++ `SpeedProfile` 과 같은 식):

| 제어기 | 성공 | 예측 오차 평균 / 최대 | 단순 L / v_max 오차 평균 / 최대 |
| --- | --- | --- | --- |
| DWA | 12 / 12 | **2.2 % / 4.3 %** | 20.6 % / 42.7 % |
| PurePursuit | 12 / 12 | **5.0 % / 8.1 %** | 17.7 % / 38.5 % |

목표별 표 (DWA, 길이 10.0–30.0 m): (−10, 0) 1.6 %, (10, 0) 3.3 %, (22, 6) 4.3 %, (22, −8) 2.7 %, (5, −6) 0.1 %,
(−15, −6) 3.3 %, (−24, 0) 0.2 %, (−15, 6) 4.1 %, (15, 6) 2.1 %, (0, −6.5) 0.6 %, (0, −14) 2.9 %, (0, 0) 1.0 %.
단순 L/v 가 크게 틀리는 이유는 출발·도착 제자리 회전(최대 180°)과 가감속·곡선 감속을 무시하기 때문이다.
PP 는 12 개 중 11 개에서 예측보다 빨랐다(DWA 7/12). PP 는 방향 오차가 45° 아래로 내려가면 제자리 회전을 멈추고
원호로 출발해 회전과 전진이 겹치는데, 예측은 "제자리 회전 후 출발" 로 둘을 더하기 때문으로 본다 (추정).

### 5.3 Gazebo (실제 로봇 모델·창고 월드, headless)

`amr_description` 로봇(DiffDrive·LiDAR·카메라, `feature/robot-description`)과 창고 월드(`amr_simulation`,
60 × 40 m, 동적 actor 5 + 지게차)를 Ignition Fortress 로 띄우고, 같은 Nav2 + velocity_profiler 체인으로 같은 형태의
경로를 추종했다. 위치추정·scan_filter·safety_node 는 이 산출물 밖이라 scratch 중계 노드가 참값 오도메트리 →
`odometry/filtered`(+ TF), `scan` → `scan_filtered`, `cmd_vel_smoothed` → `cmd_vel` 로 대신했다. 경로는 월드의 동적
장애물 경로를 피한 남측 공터에 두었다 (`closed_loop_eval.py --layout gazebo`; 직선 14 m, U 턴·S 자 같은 모양, 좁은
통로는 월드의 실제 0.60 m 통로). RTF 0.99–1.00, load average 5.6–12.7 (11:38–11:43 KST).

| 제어기 | 경로 | CTE 직선 평균 / 최대 [cm] | CTE 곡선 평균 / 최대 [cm] | 주행 시간 실제 / 예측 [s] (오차) | 목표 통과량 [cm] | 저크 기준 최대 / 명령 p99 / 참값 p99 [m/s³] |
| --- | --- | --- | --- | --- | --- | --- |
| PurePursuit | 직선 14 m | 0.0 / 0.0 | – | 15.1 / 15.5 (2.6 %) | −0.6 | 2.00 / 2.02 / 1.82 |
| | U 턴 R 2 m | 0.3 / 2.3 | 0.6 / 2.5 | 13.6 / 13.8 (1.5 %) | −0.5 | 2.00 / 2.34 / 1.84 |
| | S 자 R 3 m | 0.9 / 1.8 | 1.2 / 3.4 | 14.9 / 15.0 (0.4 %) | +0.8 | 2.00 / 2.36 / 1.84 |
| | 좁은 통로 | 0.0 / 0.0 | – | 7.3 / 7.6 (4.7 %) | −0.5 | 2.00 / 2.41 / 2.34 |
| DWA | 직선 14 m | 0.0 / 0.0 | – | 15.3 / 15.5 (1.4 %) | −2.7 | 2.00 / 2.21 / 1.81 |
| | U 턴 | 0.3 / 2.3 | 0.5 / 2.5 | 13.6 / 13.8 (1.1 %) | −2.7 | 2.00 / 2.64 / 1.74 |
| | S 자 | 0.9 / 2.2 | 1.1 / 3.5 | 15.4 / 15.0 (2.8 %) | −1.8 | 2.00 / 2.72 / 1.79 |
| | 좁은 통로 | 0.0 / 0.0 | – | 7.4 / 7.6 (3.0 %) | −2.6 | 2.00 / 2.24 / 2.33 |

8/8 성공. 운동학 시뮬레이터 결과(§5.1)와 CTE·주행 시간이 거의 같다 — 서보 근사(지연 0.04 s + 1차 0.08 s)가 Gazebo
DiffDrive 와 맞는다는 뜻이기도 하다. 실제 플랜트가 기준 모델과 조금 달라 PI 보정이 늘어난 만큼 명령 저크 p99 가
2.2–2.7 m/s³ 로 커졌지만, 로봇 몸체(참값)의 저크 p99 는 1.74–2.34 m/s³ 로 한도 근처다. 좁은 통로의 여유 0.10 m 는
중계 노드가 합성 지도(월드와 같은 치수)로 계산한 값이다.

### 5.4 팀 공용 평가 도구 교차 확인 (`amr_evaluation`, `feature/evaluation-tools`)

같은 시험대에 `cte_logger` 를 붙이고 `analyze` 게이트로 판정했다 (NavigateToPose 12 목표, 경로 = 매 1 Hz 재계획된
`plan`, NavigateToPose 실행 중 표본만): **직선 CTE 평균 0.30 cm (p95 1.77, 최대 6.41) PASS, 곡선 평균 1.77 cm (p95 4.75,
최대 7.73) PASS** (DWA). 재계획 경로 기준이라 §5.1 의 고정 경로 값보다 크다 (목표 방향 정렬·재계획 순간 포함).
고정 경로 4 종(FollowPath 실행 중 표본)에서는 PurePursuit **직선 0.05 cm (최대 1.58) PASS, 곡선 0.63 cm (최대 2.69)
PASS**. (주의: 순간 이동 직후 표본이 이전 기준 경로와 비교되지 않도록 평가기가 기준 경로를 이동보다 먼저 발행한다 —
처음에는 이 순서가 반대라 로거가 이동 직후 2.6 s 동안 16 m 오차를 기록했다.) 참값 저크 p99 는 시뮬레이터 타이머
지터에 민감해 같은 설정의 반복 실행에서 2.1 → 4.1 m/s³ 까지 흔들렸다 (load 12, 병렬 실행) — 저크 보장은 명령 쪽
(기준 최대 2.00) 으로 판정한다.

### 5.5 PID (게인 튜닝 결과의 폐루프 확인)

운동학 시뮬레이터의 서보가 기준 모델과 같으므로(명목 플랜트) PI 보정은 거의 0 이고, 모든 시험에서 재동기화
(`resync`) 0 회. 불일치 플랜트에서의 성능은 §4.2 격자 탐색과 `Pid.TunedGainsStepAndRampAcceptance` 가 확인한다.

## 6. 파라미터

`controller_server.PurePursuit.*`

| 파라미터 | 기본 | 단위 | 의미 / 근거 |
| --- | --- | --- | --- |
| `desired_linear_vel` | 1.0 | m/s | 순항 속도 (DWA 와 같은 운용 속도) |
| `lookahead_time` | 0.8 | s | k: ω_n = √2/k = 1.77 rad/s (브리프 t_L ∈ [0.6, 1.0]) |
| `min_lookahead_dist` / `max_lookahead_dist` | 0.4 / 1.8 | m | §1.2 |
| `max_lateral_accel` | 0.8 | m/s² | 적재물 안정 (R 2 m → 1.26 m/s, R 1 m → 0.89 m/s) |
| 가속·각속도·저크 한계 | limits.* | | robot_params.yaml 단일 출처 |
| `min_approach_linear_velocity` | 0.05 | m/s | 목표 직전 0 수렴 방지 |
| `approach_latency` | 0.3 | s | 목표 정지 접근의 명령 지연 t_c ([dwa.md](dwa.md) §1.6 표) |
| `curvature_window` | 0.25 | m | 곡률 추정 창 (5 셀) |
| `use_rotate_to_heading` / `rotate_to_heading_min_angle` / `rotate_to_heading_max_v` | true / 0.785 / 0.1 | – / rad / m/s | |
| `goal_align_distance` / `goal_yaw_tolerance` | 0.08 / 0.02 | m / rad | goal checker 0.10 / 0.05 보다 안쪽 |
| `use_chord_correction` / `chord_gain` | false / 1.0 | | CC-PP 옵션 (§1.5) |
| `use_collision_detection` / `collision_check_time` | true / 1.0 | – / s | 명령 원호 충돌 → 예외 |
| `velocity_reset_threshold` | 0.3 | m/s | look-ahead 기준 속도 = 직전 명령 (측정과 이만큼 벌어지면 측정) |

`velocity_profiler_node` (`config/velocity_profiler.yaml`): `rate` 50 Hz, `cmd_timeout` 0.5 s, `odom_timeout` 0.2 s,
`jerk_limit_enabled`, `max_angular_jerk` 6.0, `preserve_curvature`, `curvature_min_speed` 0.05, `use_pid`,
`use_reference_model`, `model_time_constant` 0.08, `model_delay` 0.04, `pid.{linear,angular}.{kp,ki,kd,setpoint_weight,
tracking_time}` = 0.4 / 2.0 / 0 / 1.0 / 0.447, `max_linear_correction` 0.2, `max_angular_correction` 0.3,
`resync_threshold_v/w` 0.4 / 0.6, `stop_deadband` 0.001. 파일 안에 항목별 단위·근거 주석.
