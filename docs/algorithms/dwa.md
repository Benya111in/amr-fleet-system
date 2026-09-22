# DWA 지역 계획기 (명세 4.4 Local Planner, 4.7 동적 장애물 회피)

> 구현: `src/amr_navigation/include/amr_navigation/core/{dwa,footprint,velocity_obstacle,geometry}.hpp`
> (ROS 비의존 코어), `src/amr_navigation/src/ros/dwa_controller.cpp` (`amr_navigation::DWAController`,
> `nav2_core::Controller` 플러그인, `controller_id: DWA`). 설정: `config/nav2_params.yaml` `controller_server.DWA.*`.
> 연구 브리프: `research/local-planning` §3 (기준 DWA)을 구현했고, §5 (PVT-DWA v2)의 VO/TTC 는 결정론적
> 기본형으로 구현, 공분산 기회제약·ORCA 는 확장 지점으로 남겼다 (§7).

## 1. 한 제어 주기의 절차 (Δt_c = 1/20 s)

`DwaPlanner::compute()` 는 Fox, Burgard & Thrun (1997) 의 네 단계를 그대로 따른다.

```
(0) 좁은 곳 재중심     ← 양쪽이 막힌 골에서 기준 경로를 지역 코스트맵 골 바닥으로 (§1.7)
(1) 동적 창 V_d        ← 창 중심 (v_c, ω_c), 가속 한계, 외부 속도 제한
(2) 속도 샘플링        ← N_v × N_ω 격자 (+ ω = 0 열, + 제동 후보)
(3) 궤적 시뮬레이션    ← 샘플마다 (v, ω) 일정 원호의 정확 적분, T_sim(v)
(4) 충돌 검사 + VO     ← 풋프린트(외곽선) 충돌, 동적 장애물 VO 원뿔
(5) 비용 계산          ← 6 항 정규화 가중합
(6) 최적 속도 선택     ← 최소 비용 → VO 포화면 VO 진입 최지연 (§2.1) → 유효 샘플 없으면 제동 후보
```

### 1.1 동적 창

**V_d = [v_c − a_dec·Δt_c, v_c + a_acc·Δt_c] × [ω_c − α·Δt_c, ω_c + α·Δt_c] ∩ V_s**,
V_s = [v_min, min(v_max, v_limit)] × [−ω_max, ω_max].

- a = 1.0 m/s², α = 2.0 rad/s² (robot_params.yaml `limits.*`) → 창 폭 ±0.05 m/s, ±0.10 rad/s.
- **창 중심 = 직전 명령** (v_last, ω_last), 측정값(`odometry/filtered`)과 0.3 m/s / 0.5 rad/s 넘게 벌어지면 측정값.
  측정값을 중심으로 두면 하류 저크 필터(velocity_profiler) 지연 때문에 매 주기 "측정 + 0.05" 만 요청하게 되어
  실효 가속이 한계의 약 1/3 로 떨어진다 (연구 브리프 §3.2, 프로토타입 c06). 재설정 규칙이 안전 노드 개입(급감속)
  뒤의 괴리를 막는다.
- 외부 속도 제한(`setSpeedLimit`, 안전 구역 등)이 창 아래로 내려가면 창 = {최대 감속} 한 점.
- 단위 테스트 `Dwa.DynamicWindowFromLimits`, `Dwa.WindowCenterRule`.

### 1.2 속도 샘플링

`vx_samples × vth_samples` = 11 × 31 균등 격자 (해상도 0.01 m/s, 0.0067 rad/s) + ω 창이 0 을 포함하면 ω = 0 열 강제
포함(직진 가능성 보장) + **제동 후보** (최대 감속, ω 를 0 쪽으로 — 항상 창 안, 유효 샘플이 없을 때의 출력).
단위 테스트 `Dwa.SamplingGridZeroColumnAndBrake`.

### 1.3 궤적 시뮬레이션 (정확 원호 적분)

(v, ω) 일정 → 차동구동 운동학의 해석해 (`core::integrateArc`):

x(t) = x₀ + (v/ω)[sin(θ₀ + ωt) − sin θ₀],  y(t) = y₀ − (v/ω)[cos(θ₀ + ωt) − cos θ₀],  θ(t) = θ₀ + ωt
(|ω| < 1e-9 이면 직선). 매 스텝을 시작 자세에서 직접 계산해 오차가 누적되지 않는다.

- 시야 **T_sim(v) = clip(|v|/a + 0.5, 1.5, 2.5) s**, 간격 `sim_dt` 0.1 s → 최대 25 자세.
  T_sim·v ≥ 저크 포함 정지거리 s_stop(v) (§1.4 식) 이므로 "검사 구간 무충돌" 이면 Fox 의 허용 속도 V_a
  (충돌 전 정지 가능)도 성립한다.
- 단위 테스트 `Dwa.RolloutIsExactArc` (모든 점이 반지름 v/ω 원 위, 1e-9), `Geometry.IntegrateArcIsExact`
  (20만 스텝 오일러 적분과 1e-6 m 일치).

### 1.4 충돌 검사

풋프린트 0.60 × 0.40 사각형, 자세마다 3 단계 (`core::FootprintChecker`, Nav2 `FootprintCollisionChecker` 와 같은 원리):
1. 중심 셀 비용 c₀ ≥ 253 → 충돌 (내접원 안에 장애물),
2. c₀ < c_circ (외접 반경 0.361 m 의 inflation 비용; 지역 s = 3 에서 155) → 자유 (외접원 안에 치명 셀 없음 — O(1)),
3. 그 사이 → 외곽선 4 변을 Bresenham 으로 래스터화해 LETHAL(254) 셀이 있으면 충돌.

목표 너머(호길이 > d_goal + 0.1 m)는 검사하지 않는다 (목표 뒤 벽 때문에 접근 샘플이 모두 막히는 것 방지,
`Dwa.GoalApproachAndAlignment`).

저크 제한 정지거리 (제동 꼬리 해석, `DwaPlanner::stoppingDistance`, 브리프 §3.2 표와 1 mm 이내 일치):
s_stop(v) = v·T_c + v·t_R − j·t_R³/6 + (v − j·t_R²/2)²/(2a),  t_R = a/j (v > a²/2j),  T_c = commit_time 0.2 s
→ v = 0.5 / 1.0 / 2.0 m/s 에서 0.315 / 0.889 / 2.789 m (t_c 0.15 기준, `Dwa.JerkLimitedStoppingDistance`).

### 1.5 비용 함수 (최소화, 항별 [0, 1] 정규화)

**J = w_h·J_head + w_c·J_clear + w_v·J_vel + w_p·J_path + w_o·J_osc + w_d·J_dyn**

| 항 | 식 | 가중치 | 의미 |
| --- | --- | --- | --- |
| J_head | \|wrap(atan2(target − p_E) − θ_E)\| / π, p_E = 평가 끝자세(롤아웃 앞 `path_eval_time` 0.8 s 지점), target = p_E 의 경로 투영점에서 ℓ(v) = clip(0.4v + 0.3, 0.4, 1.2) m 앞 경로 점 | 0.6 | 가까운 미래에 경로 진행 방향을 보는가 (목표 0.08 m 안에서는 목표 방향 정렬) |
| J_clear | max_k max(0, c(p_k) − c(g_k)) / 252, g_k = 같은 호길이의 기준 경로 점 | 1.0 | 장애물 근접. **경로 자체가 지나는 좁은 곳의 비용은 벌점이 아님** (좁은 통로 입구 앞 정지 국소최소 방지, 브리프 §3.5) |
| J_vel | \|v_des − v\| / (v_max − v_min), v_des = min(v_cap, d_stop⁻¹(d_goal)) × max(0, cos α₀) | 0.4 | 빨리 가려는 힘 + 목표 접근 감속(§1.6), 경로가 옆/뒤면 감속(제자리 회전 유도) |
| J_path | mean_{k ≤ k_E} min(\|e⊥(p_k)\| / 0.8, 1), k_E = ⌈path_eval_time / sim_dt⌉ | 2.0 | 경로 이탈 (CTE 를 직접 줄이는 항) |
| J_osc | 전후진 반전 또는 제자리 회전 방향 반전이면 1 | 0.5 | 진동 억제 |
| J_dyn | min(1, max(0, \|v\| − v_safe(TTC₀))/v_span + 0.3·max(0, 1 − TTC₀/T_pred)), v_safe = a·max(0, TTC₀ − 0.45 s), T_pred = 5 s | 1.5 | 예측 접촉 전에 설 수 없는 속도의 초과분 + TTC 보조항 (§2.2) |

**추종 지평과 안전 지평의 분리 (`path_eval_time`)**: 충돌·여유 비용은 정지거리를 덮는 전체 롤아웃 T_sim(v)
(1 m/s 에서 1.5 s)으로 보지만, 헤딩·경로 항은 앞 0.8 s 만 본다. (v, ω) 일정 원호는 곡률이 하나뿐이라, 곡률 부호가
바뀌는 경로(S 자: R 3 m 30°·60° 물결, 호 1.6–3.1 m)를 1.5 m 전체로 평가하면 두 구간의 **평균 곡률**을 가진 원호가
뽑혀 안쪽을 가로지른다. 앞부분만 평가하면 지금 구간의 곡률을 따르고, 다음 구간은 20 Hz 재계획이 맡는다
(추종은 짧은 제어 지평, 안전은 긴 정지 지평 — MPC 의 두 지평 분리와 같은 생각). 단위 테스트
`Dwa.ShortTrackingHorizonFollowsSCurve`: 이상 폐루프 S 자 평균 CTE 7.8 cm(전체 평가) → 1.8 cm(앞 0.8 s).

유효 샘플(충돌 없음, VO 밖) 중 J 최소를 고른다. 충돌 없는 샘플이 모두 VO 안이면 VO 진입이 가장 늦은 샘플(§2.1),
충돌 없는 샘플이 하나도 없으면 제동 후보를 내고, 정지 상태에서
`no_valid_patience`(10) 주기 계속되면 `PlannerException` → controller_server `failure_tolerance` 1.0 s 뒤
FollowPath 실패 → BT 복구(코스트맵 초기화·회전·대기·후진).
단위 테스트: `FreeSpaceGoesStraightAndAccelerates`, `SteersBackTowardPath`, `AvoidsObstacleOnPath`,
`AllCollidingReturnsBrake`, `GoalApproachAndAlignment`, `OscillationPenalty`, 플러그인 `ThrowsWhenStoppedAndBlocked`.

### 1.6 목표 접근 속도 (저크·지연 포함 정지거리의 역함수)

사다리꼴 v_des = √(2·a·d_goal) 는 "지금 감속을 시작하면 목표에 선다" 는 속도지만, 명령 뒤에 저크 제한 필터
(velocity_profiler, 감속도 0 → −a 에 a/j = 0.5 s)와 서보 지연이 있어 실제 로봇은 늦게 감속한다. 그래서 목표 접근
속도는 §1.4 의 저크 포함 정지거리에 명령 지연 t_c 를 넣은 **d_stop(v) = v·t_c + v·t_R − j t_R³/6 + (v − a²/2j)²/(2a)**
의 역함수(단조 증가 → 40 회 이분법, `SpeedProfile::maxSpeedForStop`)로 둔다. t_c = `approach_latency` 0.3 s 는
체인 시뮬레이션(`PurePursuit.GoalApproachThroughJerkFilterStopsInsideTolerance`: 20 Hz 제어기 → 50 Hz 저크 필터 →
지연 0.04 s + 1차 0.08 s 서보, 5 m 직선)에서 골랐다:

| 목표 접근 법칙 | 목표 통과량 | 출발→정지 시간 |
| --- | --- | --- |
| 사다리꼴 √(2ad) | **+40.0 cm** (목표 판정 0.10 m 뒤 제동 꼬리) | 7.32 s |
| 저크 포함, t_c 0.0 | +16.3 cm | 7.09 s |
| 저크 포함, t_c 0.1 | +9.1 cm | 7.03 s |
| 저크 포함, t_c 0.2 | +3.8 cm | 7.02 s |
| **저크 포함, t_c 0.3 (채택)** | **+0.2 cm** | **7.07 s** |
| 저크 포함, t_c 0.4 | −2.5 cm (앞에 섬) | 7.16 s |

t_c 0.3 ≈ 저크 램프의 시간 지연 a/(2j) 0.25 s + 서보 지연 0.04 s. 지나침이 없으므로 오히려 사다리꼴보다 빨리
선다 (지나친 거리를 되돌아오지 않음). DWA 와 Pure Pursuit 가 같은 함수를 쓴다.

### 1.7 좁은 곳 경로 재중심 (0 단계, `recenter_*`)

전역 경로는 map 프레임이라 위치추정·지도 오차만큼 실제 통로 중심에서 비낀다 (0.60 m 통로의 측면 여유는 0.10 m).
지역 코스트맵은 로봇과 같은 odom 프레임에서 LiDAR 로 만든 것이라 **실제 통로 중심**을 안다. 그래서 매 주기, 창으로
잘라 온 기준 경로의 점마다:

1. 점의 비용이 `recenter_min_cost` 100 이상(지역 s = 3 에서 장애물까지 ≈ 0.5 m 이내)이면, 경로 법선 방향
   ±`recenter_max_shift` 0.10 m 를 코스트맵 해상도 간격으로 비용을 읽는다.
2. 양 끝이 최소보다 높은 **골(valley)** 일 때만 (한쪽만 막힌 곳 — 벽을 끼고 가는 경로 — 은 끝이 최소라 옮기지 않는다)
   최소 비용 구간의 가운데로 옮긴다.
3. 이동량을 ±4 점(±0.2 m) 이동평균해 입구에서 완만하게 들어가고, 목표 0.5 m 안은 옮기지 않는다.

헤딩·경로·여유 항은 모두 옮긴 경로를 쓴다. 넓은 곳(비용 < 100)은 그대로라 추종 결과(§8)에는 영향이 없다.
단위 테스트 `Dwa.NarrowAisleWithLidarNoise` (σ 0.03 LiDAR 로 매 스캔 새로 만든 0.025 m 코스트맵, 게이트 중앙값, 프로파일러 +
서보 지연 체인): 전역 경로가 통로 중심에서 3.5 cm 비껴 있고 입구 1.2 m 앞에서 9 cm 옆·8° 틀어진 채 0.3 m/s 로
들어오는 경우(이전 기준 Gazebo 에서 본 입구 끼임을 옮긴 장면), 재중심 켬: 3 seed × 2 헤딩 모두 관통, 참값 최소 여유 0.063–0.087 m,
유효 샘플 없음 0 주기 / 끔: 관통은 하지만 여유 0.039–0.064 m, 유효 샘플 없음 최대 3 주기.
Gazebo 통합 체인(이 기준의 재매핑 지도, 추종 중 위치추정 오차 평균 약 1 cm)에서는 켬 20 회 / 끔 10 회 모두 관통했고 통로 안
최대 횡 편차 0.5–4.2 cm / 1.2–2.9 cm, 참값 최소 여유 0.055 / 0.070 m 로 차이가 잡음 안이었다 ([costmap.md](costmap.md)
§6.2) — 전역 경로 편향이 작으면 옮길 것이 없다. 편향이 큰 지도·위치추정에 대한 보호 장치로 켜 둔다.

## 2. 동적 장애물: Velocity Obstacle 샘플 제외 + TTC 비용 (명세 4.7)

플러그인이 `perception/tracked_obstacles` (amr_msgs/TrackedObstacleArray, map 프레임)를 구독해, 트랙마다
메시지 나이만큼 등속 전진시킨 뒤 코스트맵 프레임(odom)으로 옮긴다. 속도 < 0.2 m/s 트랙은 정적으로 보고(코스트맵이 처리)
제외한다. TrackedObstacle 에 반경이 없어 덮개 반경 0.25 m (사람) 를 쓴다 (`obstacle_radius`).

### 2.1 VO 원뿔 판정 (시간 절단, Fiorini & Shiller 1998)

상대 위치 **p** = o − r, 합성 반경 R = r_robot(0.361) + r_obs(0.25) + margin(0.30 = e-stop 거리),
상대 속도 **w** = v_robot − u_obs.

VO^τ = { w : ∃ t ∈ [0, τ], ‖p − t·w‖ < R } — 꼭짓점이 원점, 축이 p, 반각 asin(R/‖p‖) 인 원뿔을 τ 에서
원판 D(p/τ, R/τ) 로 자른 집합. 판정은 최근접 시각 t* = clamp(p·w/‖w‖², 0, τ) 에서 ‖p − t*·w‖ < R 과 동치:
(⇐) t* 에서 거리가 R 미만이면 정의상 VO. (⇒) 거리 ‖p − t·w‖² 는 t 의 볼록 2차식이고 [0, τ] 위 최솟값은 t* 에서
나므로 어떤 t 에서 R 미만이면 t* 에서도 R 미만. τ → ∞ 이고 p·w > 0 이면 t* = p·w/‖w‖² 에서
‖p − t*w‖ = ‖p‖ sin∠(p, w) < R ⇔ ∠(p, w) < asin(R/‖p‖) — 원뿔 각도식과 같다.
이미 R 안(‖p‖ < R)이면 접근(p·w > 0)하는 속도만 VO 로 본다 (이탈은 허용).

차동구동 샘플 (v, ω) 의 등가 직선 속도는 τ 구간 현(chord) 속도
**v_eff = v·sinc(ωτ/2)·(cos(θ + ωτ/2), sin(θ + ωτ/2))** (τ 뒤 원호 끝점 = v_eff·τ, `VelocityObstacle.ChordVelocity`).

VO 안의 샘플은 제외한다 (`use_velocity_obstacles`, 파라미터로 끔). 샘플마다 VO 진입 시각
t_VO = min{t ∈ [0, τ] : ‖p − t·w‖ < R} (‖p − t·w‖² = R² 의 작은 근, `core::velocityObstacleTime`; VO 안 ⇔ t_VO 유한)
도 기록한다.

**VO 포화 (리뷰 결함과 수정)**: 한 주기 동적 창은 ±a·Δt_c = ±0.05 m/s, ±0.1 rad/s 로 좁아서, 1 m/s 장애물이 2–3 m
앞에 오면 R = 0.911 m 절단 원뿔이 창의 342 샘플을 **전부** 덮는다. 리뷰 전 코드는 이때 그 주기만 VO 를 끄고 비용
최소 샘플을 골랐는데(`vo_fallback`), TTC 항(아래 §2.2 의 이전 식)이 속도 항보다 약해 창의 최고 속도가 뽑혔다 — 리뷰
하네스 `vo_probe` (nav2_params 의 DWA 값, 이상 플랜트 20 Hz) 를 기준 코어(a7d43f7)에 다시 링크해 이번에 돌린 결과: 정면
3 m 에서 로봇이 0.8 → 1.0 m/s 로 **가속**해 중심 거리 0.02 m, 횡단 (2, 1.5) → −y 1 m/s 에서 0.35 m (접촉 < 0.61 m).
현재는 VO 를 끄지 않는다: 충돌 없는 샘플이 모두 VO 안이면(`vo_saturated`) **VO 진입이 가장 늦은 샘플**을 고른다
(진입 시각이 `vo_time_tie` 0.02 s 안으로 같으면 비용 최소). 정면 접근이면 창의 최저 속도·회피 쪽, 횡단이면 감속 쪽이
뽑혀 매 주기 한 창씩 VO 밖으로 움직인다. 제동이 VO 를 벗어나는 속도 집합으로 이어지므로 Fox 의 "허용 속도" 를 동적
장애물로 넓힌 것과 같은 효과이고, 창이 VO 밖 속도를 다시 포함하면 정상 선택으로 돌아온다.

| 같은 하네스 (`vo_probe`, 4.5 s) | 기준 코어: 최소 중심 거리 / 접근 속도 | 현재: 최소 중심 거리 / 접근 속도 |
| --- | --- | --- |
| 정면 3 m, 장애물 −1 m/s, 로봇 0.8 m/s | 0.02 m / 1.0 m/s (가속) | 0.01 m / **0.0 m/s** (1.5 s 안에 정지) |
| 정면 2 m, 로봇 0.8 m/s | 0.01 m / 1.0 m/s | 0.00 m / 0.0 m/s |
| 횡단 (2, 1.5) → −y 1.0 m/s, 로봇 1.0 m/s | **0.35 m** (접촉) | **1.11 m** |
| 횡단 (4, 3.2) → −y 1.0 m/s | 1.11 m | 1.11 m |
| 같은 경우 + 장애물 현재 원판을 코스트맵에도 표시 | 0.53 / 1.24 m | 1.12 / 1.19 m |

정면 접근은 후진 없이(`min_vel_x` 0) 피할 수 없다 — 장애물이 비키지 않으면 멈춰 선 로봇에 닿는다. 요구는 "가속하지
않고, 닿기 전에 멈춰 선다" 로 두었다 (`ClosedLoopHeadOnYields`). 옆으로 0.7 m 비껴 오는 정면 장애물은 풋프린트 여유
0.285 m 로 지나간다.

단위 테스트: `ConeCases` (정면 접근, 시간 절단, 이탈, 원뿔 경계 ±0.02 rad, 이미 겹침), `VoTimeMatchesCone`
(t_VO 유한 ⇔ VO 안), `VelocityObstacleRejectsFastCrossing` (횡단: v > 0.5 m/s 샘플만 제외),
`VelocityObstacleSaturationKeepsVoAndBrakes` (포화: 창 최저 속도 선택, 가속하지 않음), `TtcCostSlowsBeforeVoSaturates`,
폐루프 `ClosedLoopCrossingKeepsVoRadius` (횡단 4 경우, 이상 플랜트: 최소 중심 거리 1.111 m, 사각형 여유 ≥ 0.50 m,
기준 R 0.911 − 0.03 을 단언), `ClosedLoopHeadOnYields`.

### 2.2 예측 충돌 시각 TTC₀ 와 감속 비용

현 근사는 원호가 크게 휘면 충돌을 놓칠 수 있으므로(브리프 c03), 원호 롤아웃을 T_pred = 5 s 로 연장해 장애물 예측
위치 o(t) = o + u·t 와의 거리가 R_d = r_robot + r_obs + 0.5 (critical zone) 미만이 되는 첫 시각을 구간별 2차식 근으로
정확히 계산한다 (`core::firstContactTime`, `FirstContactTimeHeadOn`: 정면 2 m/s 접근 → 2.000 s).

리뷰 전 J_dyn = max(0, 1 − TTC₀/T_pred) 는 이웃 샘플(Δv 0.01 m/s) 사이 변화가 속도 항 w_v·Δv/v_span 보다 작아
TTC 가 짧아져도 로봇을 늦추지 못했다. 현재 식은 **예측 접촉 전에 설 수 있는 속도**의 초과분이 주항이다:

  v_safe(TTC₀) = a_dec·max(0, TTC₀ − t_lag),  t_lag = commit_time + a_dec/(2j) = 0.2 + 0.25 s
  J_dyn = min(1, max(0, |v| − v_safe)/v_span + λ·max(0, 1 − TTC₀/T_pred)),  λ = `dynamic_steer_gain` 0.3

초과 1 m/s 당 비용 기울기 w_d/v_span = 1.5 가 속도 항 기울기 w_v/v_span = 0.4 보다 커서, 과속 영역에서는 감속이
이긴다 (`TtcCostSlowsBeforeVoSaturates`: VO 밖이지만 TTC₀ 안인 횡단 장애물 앞에서 1.0 m/s 대신 감속을 고르고, 동적
항을 끄면 1.0 m/s 유지). λ 항은 같은 속도에서 TTC 가 긴 방향(회피 쪽)을 고르는 보조항이다.

## 3. Nav2 통합

- 전역 경로(map) → 코스트맵 프레임(odom) 변환은 TF 한 번 조회 후 2D 강체 합성, 로봇 최근접 정점부터
  `path_horizon` 4 m (지역 코스트맵 8 × 8 m 의 반폭) 까지만 (새 경로는 앞 20 m, 이후 직전 인덱스부터 10 m 창에서 최근접 탐색 → 되돌아오는
  경로에서 뒤 구간으로 튀지 않음). 창 끝이 실제 목표가 아니면 목표 감속·목표 너머 검사 제외를 끈다.
- `payload/mass` (latched) → 가속 한계 × m/(m + m_payload) (공차 47.6 kg).
- 발행: `local_plan` (선택 궤적), `dwa/stats` [cycle_ms, n_samples, n_valid, n_collision, n_vo_rejected,
  vo_saturated, best_ttc, v, w, d_goal, n_recentered] — 주기 시간·회피 동작·재중심 로그.
- 한계값 기본값은 robot_params.yaml `limits.*` (controller_server 에 함께 로드), 운용 최고속도 `max_vel_x` 1.0 m/s.

## 4. 파라미터 (`controller_server.DWA.*`)

| 파라미터 | 기본 | 단위 | 의미 / 근거 |
| --- | --- | --- | --- |
| `max_vel_x` / `min_vel_x` | 1.0 / 0.0 | m/s | 운용 속도 (하드웨어 2.0 은 전방 2.6 m 이상 빈 경우만 safety 가 허용) / 후진은 BackUp 복구만 |
| `max_vel_theta`, `acc_lim_x`, `decel_lim_x`, `acc_lim_theta`, `jerk_lim_x` | limits.* | | robot_params.yaml 단일 출처 |
| `vx_samples` / `vth_samples` | 11 / 31 | | §1.2 (브리프 §3.3) |
| `sim_dt` / `sim_time_min` / `sim_time_max` | 0.1 / 1.5 / 2.5 | s | §1.3 |
| `commit_time` | 0.2 | s | 정지거리 계산의 명령 유지 시간 |
| `approach_latency` | 0.3 | s | 목표 접근 v_des = d_stop⁻¹(d_goal) 의 명령 지연 t_c (§1.6 표) |
| `window_reset_v` / `window_reset_w` | 0.3 / 0.5 | m/s, rad/s | 창 중심 재설정 |
| `heading_weight` … `dynamic_weight` | 0.6 / 1.0 / 0.4 / 2.0 / 0.5 / 1.5 | | §1.5, §4.1 튜닝 (브리프 §3.5 프로토타입 0.8 / 1.0 / 0.4 / 1.2 에서 출발) |
| `path_eval_time` | 0.8 | s | 헤딩·경로 항 평가 지평 (§1.5; 충돌·여유는 전체 T_sim) |
| `heading_lookahead_{gain,offset,min,max}` | 0.4 / 0.3 / 0.4 / 1.2 | s, m | ℓ(v) = clip(0.4v + 0.3, 0.4, 1.2) |
| `path_band` | 0.8 | m | J_path 정규화 |
| `goal_align_distance` | 0.08 | m | ≤ goal checker xy 0.10 (그래야 정렬 중 멈춰도 목표 판정) |
| `path_horizon` | 4.0 | m | 로컬 코스트맵 반폭 (8 × 8 m) |
| `use_dynamic_obstacles` / `use_velocity_obstacles` | true / true | | |
| `prediction_time` | 5.0 | s | TTC₀ 지평 |
| `dynamic_margin` / `vo_margin` | 0.5 / 0.3 | m | safety critical / e-stop 거리 |
| `vo_time_horizon` / `vo_max_range` | 2.0 / 6.0 | s / m | |
| `vo_time_tie` | 0.02 | s | VO 포화 시 진입 시각 동률 폭 (§2.1) |
| `dynamic_speed_threshold` / `dynamic_fast_speed` | 0.2 / 0.5 | m/s | 동적 트랙 = 이 속도 이상이면서 추적기 `is_dynamic` 이거나 0.5 m/s 이상 (정지 물체 트랙의 속도 잡음 0.2–0.3 m/s 제외) |
| `dynamic_steer_gain` | 0.3 | | J_dyn 의 TTC 보조항 λ (§2.2) |
| `recenter_narrow` / `recenter_min_cost` / `recenter_max_shift` / `recenter_smooth` / `recenter_goal_keep` | true / 100 / 0.10 / 4 / 0.5 | – / cost / m / 점 / m | 좁은 곳 재중심 (§1.7) |
| `obstacle_radius` / `track_timeout` | 0.25 / 0.5 | m / s | |
| `no_valid_patience` | 10 | 주기 | |

### 4.1 튜닝 절차 (브리프 §6.5, 순서대로)

1. 장애물 없는 직선: `path_weight` 와 `vth_samples` 로 직선 CTE (목표 < 5 cm) — w_p 를 올리면 CTE ↓, 너무 크면
   회피 시 경로 복귀가 급해진다.
2. 곡선: `heading_lookahead_*`, `heading_weight`, `path_eval_time` — ℓ 이 길면 곡선 안쪽을 자르고, 평가 지평이 길면
   곡률 평균화로 S 자에서 안쪽을 가로지른다.
3. 목표 접근: `approach_latency` (§1.6).
4. 좁은 통로(0.60 m): 지역 inflation(0.8 / 3.0)과 `clearance_weight` — 상대 여유 비용이라 통로 진입을 막지 않는지 확인.
5. 동적 장애물: `vo_margin`, `dynamic_weight`, `prediction_time`.

도구: `ros2 run amr_navigation tracking_sim dwa key=value …` (ROS 없는 폐루프: 제어기 20 Hz → 실제 `VelocityProfiler`
50 Hz → 서보 지연 0.04 s + 1차 0.08 s + 가속 제한 100 Hz, 빈 코스트맵, closed_loop_eval.py 와 같은 기준 경로 3 종,
목표 판정 xy 0.10 m / yaw 0.05 rad). 한 설정에 1 초 미만이라 격자 탐색을 반복할 수 있고, ROS 폐루프(§8)와 수치가
맞는다 (예: 기본값 U 턴 곡선 CTE 평균 0.5 cm vs ROS 0.4 cm, 주행 시간 13.6 vs 13.6 s — §8.1).

| 설정 (heading / path, ℓ, 평가 지평) | 직선 T [s] (예측 21.5) | U 턴 곡선 CTE 평균 / 최대 [cm] | U 턴 T 오차 | S 자 곡선 CTE 평균 / 최대 [cm] | S 자 T 오차 | 목표 통과량 [cm] |
| --- | --- | --- | --- | --- | --- | --- |
| 출발점: 0.8 / 1.2, ℓ 0.8v+0.5 [0.6, 2.0], 전체 롤아웃 | 23.00 | 3.7 / 9.7 | 17.9 % | **16.1 / 35.9** | 20.0 % | −7.9 (느린 꼬리) |
| 평가 지평 0.8 s 만 | 21.30 | **29.4 / 36.3** | 5.3 % | 11.7 / 23.6 | 2.6 % | −2.3 |
| + ℓ 0.4v+0.3 [0.4, 1.2] | 21.30 | 10.3 / 13.5 | 3.4 % | 6.3 / 10.4 | 2.0 % | −2.3 |
| + path 1.2 → 3.0 | 21.30 | 0.4 / 2.0 | 1.5 % | 0.9 / 2.8 | 1.4 % | −2.3 |
| + heading 0.8 → 0.5, path 1.5 | 21.30 | 0.5 / 2.1 | 1.5 % | 1.0 / 3.1 | 1.6 % | −2.3 |
| **채택: heading 0.6, path 2.0** | **21.30** | **0.5 / 2.1** | **1.5 %** | **0.9 / 3.0** | **1.4 %** | **−2.3** |
| 채택값, 0.5 m 옆에서 출발 | 21.45 | 0.5 / 2.0 | 0.0 % | (출발 구간 포함) 7.3 / 50 | 0.3 % | −2.5 |

(이 기준의 코어로 `tracking_sim` 을 다시 돌린 값 — 결정적 시뮬레이션이라 부하와 무관.)

관찰: (1) 평가 지평만 줄이면 긴 헤딩 목표점(ℓ ≤ 2.0 m)이 원 경로에서 안쪽 정상상태 오차(29 cm)를 만든다 — 헤딩
목표점도 평가 끝점 기준으로 짧게 해야 한다. (2) path 가중이 1.2 → 2.0 을 넘으면 원 경로의 정상상태 오차가
10 cm → 0.5 cm 로 떨어진다(헤딩 항이 만드는 안쪽 편향을 경로 항이 이김). 3.0 도 비슷하지만 회피 뒤 복귀가 급해지는
쪽이라 2.0 을 택했다. (3) 0.5 m 옆에서 출발해도 진동 없이 수렴한다.

## 5. 계산 예산 (20 Hz, 1 코어)

최악 셀 접근 ≈ 342 샘플 × 25 자세 × (외곽선 ≈ 40 셀) — 대부분의 자세는 2 단계 조기 판정(O(1))으로 끝난다.
한 주기 계산 시간 (`dwa/stats` 의 cycle_ms, 경로 창 변환·트랙 변환 포함, 단일 스레드):

| 측정 | 평균 | p95 | 최대 | 예산 50 ms 대비 (최대) |
| --- | --- | --- | --- | --- |
| 단위 테스트 `Dwa.CycleTimeBudget` (11 × 31, 12 × 12 m, 랙 장애물) | 1.16 ms | | | |
| 이상화 폐루프 (운동학 시험대), 직선·U 턴·S 자 | 0.91–0.99 ms | 1.12–1.26 ms | 1.32 ms | 2.6 % |
| 이상화 폐루프, 0.60 m 좁은 통로 (σ 0.03 스캔, 재중심) | 1.52 ms | 2.74 ms | 3.09 ms | 6.2 % |
| Gazebo 통합 체인, 좁은 통로 20 회 관통 (5 416 주기, 재중심 4 407 주기) | 1.67 ms | 2.77 ms | 3.90 ms | 7.8 % |
| Gazebo 통합 체인, 1.0 m/s 횡단 actor 12 회 (3 309 주기, VO 포화 26 주기) | 1.35 ms | 1.88 ms | 2.58 ms | 5.2 % |

(load average 20–31, 32 스레드 호스트, 이 작업의 Gazebo 2–3 개와 함께. `dwa/stats` cycle_ms: 경로 창 변환·트랙 변환
포함, 단일 스레드.) Pure Pursuit 는 0.02 ms (최대 0.04 ms).

## 6. TEB · DWB 와의 비교 (명세 4.4)

### 6.1 TEB 빌드 (서드파티 소스는 저장소에 넣지 않는다)

`amr-fleet-system:wf-final` 컨테이너에서 `apt-get update` 뒤 `apt-cache policy` 로 확인한 결과 packages.ros.org jammy 에
`ros-humble-teb-local-planner`·`ros-humble-costmap-converter` 는 **없고** `ros-humble-libg2o` (2020.5.29-4jammy) 만 있다
(2026-09-22). 그래서 별도 오버레이 워크스페이스에 소스로 빌드했다:

| 저장소 | 브랜치 | 커밋 |
| --- | --- | --- |
| rst-tu-dortmund/teb_local_planner | `humble-devel` | `630a22e88dc9fd45be726a762edbb5b776bef231` (2022-09-12, "Humble maintenance (#374)") |
| rst-tu-dortmund/costmap_converter | `humble` | `9565858432664e9cfe0b3556e1809336c4ab06d4` (2020-06-22, "0.1.2") |

```bash
# 일회용 컨테이너 안 (이미지에 libg2o 가 없으므로 deb 를 받아 설치), 본 저장소 빌드와 분리된 워크스페이스
apt-get update && apt-get download ros-humble-libg2o && dpkg -i ros-humble-libg2o_*.deb
mkdir -p ~/teb_ws/src && cd ~/teb_ws/src
git clone -b humble-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
git clone -b humble https://github.com/rst-tu-dortmund/costmap_converter.git
source /opt/ros/humble/setup.bash && cd ~/teb_ws
rosdep install --from-paths src -i -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to teb_local_planner   # 4 패키지, 약 54 s
# 실행: 본 워크스페이스 install 을 소싱한 뒤 ~/teb_ws/install/setup.bash 를 위에 소싱 (컨테이너마다 libg2o deb 설치)
```

### 6.2 비교 조건 (공정성)

- 같은 통합 체인·같은 월드: Gazebo, EKF/AMCL + scan-to-map, 인지 스택, safety_node 가 명령 사슬에 있음. 같은
  `controller_server`·지역 코스트맵(8 × 8 m, 0.025 m, §1 의 레이어)·풋프린트 다각형·한계(1.0 m/s, a 1.0 m/s², ω 1.5,
  α 2.0). 파라미터 파일은 배포 `nav2_params.yaml` 에 DWB 튜닝값과 TEB 블록만 더한 것 (`mk_cmp.py`, scratch).
- **DWB**: §8.1 의 튜닝 격자에서 고른 DWB_E (20 × 40 샘플, sim_time 1.5, PathDist 48, RotateToGoal 완화).
  **TEB**: `teb_autosize` dt_ref 0.3, 다각형 풋프린트, `min_obstacle_dist` 0.05 (0.60 m 통로 측면 여유 0.10), inflation 0.3,
  costmap_converter `CostmapToPolygonsDBSMCCH` 5 Hz, 호모토피 탐색 3 클래스, 후진 0.1 m/s 허용, 동적 장애물은 DWA 의
  VO/TTC 와 **같은 트랙**(`perception/tracked_obstacles` → `ObstacleArrayMsg` 중계, 반경 0.25 m, 0.2 m/s 이상).
- **BT 는 셋 모두 1 Hz 무조건 재계획판**(TTC 가지 포함, PipelineSequence 형제 구조)을 쓰고, FollowPath 직접 시험의 경로
  헤더 시각은 0 (= 최신 TF). 이유: TEB 는 전역 경로를 **경로 헤더 시각의 TF** 로 변환하는데, 배포 BT(경로가 유효하면 재계획
  안 함)에서는 경로가 TF 캐시(10 s)보다 오래돼 변환이 실패한다 — 배포 BT 로 먼저 돌린 TEB 실행(통로 6 구간 + 횡단 5 회
  + 쌍 1 구간)에서 "Could not transform the global plan" 657 회, 제어기 patience 초과 79 회, 통로 6 구간 중 4 구간 중단
  (3 구간은 시작 11 s 에), 횡단 5 회 중 3 회 중단. TEB 를 쓰려면 주기적 재계획(또는 경로 시각 갱신)이 필요하다.
- safety_node 는 TTC 제한만 끈 설정 (costmap.md §6.2 와 같은 이유 — 배포 설정은 좁은 통로 안에서 멈춘다). 그래서 이 표의
  동적 장애물 결과는 제어기 + 근접 정지만의 결과다 (배포 설정의 횡단 결과는 costmap.md §6.3).
- 시나리오 (`cmp_probe.py`): ① 0.60 m 통로 FollowPath 왕복 3 회(6 구간, 회전점에서 제자리 180°), ② 1.0 m/s 횡단 actor
  차선 주행 5 회 (같은 seed 의 위상), ③ 계획기 벤치마크의 무작위 쌍 파일(`slam_pairs_42.csv`, 두 점 모두 여유 ≥ 0.45 m,
  직선거리 ≥ 5 m)의 점을 차례로 이은 NavigateToPose 8 구간 (계획 경로 합 269–271 m).
- 부하: 다른 사용자의 통합 시험 컨테이너가 겹쳐 DWB·TEB 는 동시에 RTF 0.63–0.64 (load 22–90), DWA 는 그 뒤 RTF 0.43
  (load 67–95) 로 돌았다. 시간·CTE 는 시뮬레이션 시각 기준이라 직접 비교하지만, CPU 는 DWA 만 시뮬레이션 시간으로
  정규화해 쟀고 DWB·TEB 는 벽시계 % 를 실행 전체 RTF 로 나눈 근삿값이다.

### 6.3 결과

| | **DWA (본 구현)** | DWB_E (튜닝) | TEB |
| --- | --- | --- | --- |
| ① 통로 6 구간: 관통 | 2 / 6 | 1 / 6 | **6 / 6** |
| ① 통로 안 시간 / 참값 최소 여유 (관통 구간) | 3.96–3.98 s / 0.080 m | 4.0 s / 0.080 m | 3.9–5.1 s / 0.072 m |
| ① 실패 원인 | 3 번째 구간 입구 앞에서 횡단 actor 를 피해 랙 모서리 옆으로 꺾인 뒤 후진 없이 끼임 (여유 0.026 m) → 이후 구간은 그 자리에서 시작해 모두 진행 검사 실패 | 북행 3 구간 모두 시작의 제자리 180° 회전을 못 해 진행 검사 실패 | – |
| ② 횡단 5 회: 도착 / 충돌 / 최소 부호 거리 | 5 / **0** / 0.29 m | 5 / 1 / −0.34 m | 5 / 1 / −0.50 m |
| ② 차선 이탈 최대 / 복귀(0.10 m 띠) 최대 | 0.45 m / 7.5 s | 0.70 m / 8.0 s | 0.40 m / 8.4 s |
| ③ 무작위 쌍 8 구간: 도착 | 8 / 8 | 8 / 8 | 8 / 8 |
| ③ 주행 시간 합 | **328.7 s** | 336.4 s | 361.3 s |
| ③ 주행 시간 예측 오차 평균 / 최대 | **5.7 % / 20.5 %** | 8.2 % / 20.1 % | 14.0 % / 34.4 % |
| ③ 첫 전역 경로 대비 편차 (구간 평균의 평균 / 최대) | 0.27 m / 4.44 m | 0.12 m / 0.78 m | 0.30 m / 3.12 m |
| ③ 정적 구조물 최소 여유 (풋프린트) | 0.72 m | 0.72 m | 0.56 m |
| ③ 동적 장애물 접촉 (8 구간, actor 5 + 지게차) | 2 | 1 | 2 |
| controller_server CPU, 시뮬레이션 1 s 당 (코어 %) | 43–59 % (측정) | ≈ 78–80 % (근사) | ≈ 48–65 % (근사) |
| 벽시계 CPU (%, 실행 RTF) | 20–24 % (0.43) | 48–51 % (0.63) | 24–47 % (0.64) |

- ③ 의 "첫 경로 대비 편차" 는 1 Hz 재계획으로 경로가 바뀐 구간을 포함한다 — DWA 의 4.44 m, TEB 의 3.12 m 는 재계획이
  다른 통로를 고른 구간이다 (추종 오차가 아니다). 추종 정확도 자체는 §8.1 (이상화)와 path_tracking.md §5.3 (통합).
- controller_server CPU 에는 지역 코스트맵 갱신(0.025 m 격자, 10 Hz, 팽창)이 포함되고 대부분을 차지한다. DWA 한 주기
  계산은 1.35–1.67 ms (20 Hz 에서 코어 2.7–3.3 %, §5). DWB 는 샘플 800 개(20 × 40)라 가장 무겁다.
- 동적 장애물 접촉은 모두 대본 이동 actor(회피하지 않고 로봇을 통과)가 로봇에 걸어 들어온 경우를 포함한 collision_monitor
  판정이다. 이 비교는 safety_node 의 TTC 감속이 꺼진 설정이다 — 배포 설정의 DWA 는 횡단 26 회 충돌 0 (costmap.md §6.3).

### 6.4 장단점 (측정 근거)

| | 본 DWA | Nav2 DWB (튜닝) | TEB |
| --- | --- | --- | --- |
| 최적화 | 동적 창 샘플 + 정규화 비용합 (결정적) | 샘플 + critic 합 | 시간 탄성 밴드, g2o 희소 비선형 최소제곱 |
| 궤적 모델 | (v, ω) 일정 원호 1 개 | 같음 | 다중 자세 + 시간 간격 (가감속이 시간에 따라 변함) |
| 동적 장애물 | 트랙 등속 예측: VO 샘플 제외 + TTC 비용 | 코스트맵 현재 점유만 | 등속 예측 (같은 트랙을 중계해야 함) |
| 장점 (측정) | 무작위 쌍 가장 빠름(328.7 s)·예측 오차 가장 작음(5.7 %), 횡단 충돌 0 (이 비교 5 회 + 배포 26 회), 주기 1.4–1.7 ms, 항별 로그로 설명 가능 | 설정만으로 쓸 수 있고 정적 경로 편차가 작음 (0.12 m) | 좁은 통로·제자리 선회에 가장 강함 (6 / 6, 이 시험에서 끼인 적 없음 — 후진 0.1 m/s 허용), 시간 탄성 밴드라 가감속이 궤적 안에서 계획됨 |
| 단점 (측정) | 후진 샘플이 없어 회피 기동 뒤 모서리에 끼면 BT 복구(BackUp) 없이는 못 나옴 (① 3 번째 구간), 회피 후 같은 방향으로 한 바퀴 도는 국소최소 (costmap.md §6.3 run 0) | 경로 시작의 180° 선회 불가 (① 북행 3 / 3 실패), CPU 최대, 횡단 이탈 0.70 m·접촉 1, 목표 접근 감속이 critic 균형뿐 (§8.1: 목표 앞 8 cm) | 경로 헤더 시각 TF 조회 → 주기 재계획 필수(배포 BT 에서 변환 실패 657 회), 주행 시간 +10 %·예측 오차 14 % (최대 34 %), "trajectory is not feasible" 재설정, 횡단 접촉 1 (−0.50 m), Humble 바이너리 없음 |

## 7. 확장 지점 (연구 브리프 §5 PVT-DWA v2)

- **공분산 기회제약**: `DynamicObstacle` 에 위치·속도 공분산 필드를 더하고 `firstContactTime` 의 R 을 σ 로 키우면
  된다 (TrackedObstacle 메시지 확장 필요 — 브리프 §7.3, 인지 영역과 합의).
- **ORCA (동료 AMR)**: `inVelocityObstacle` 을 반평면 제약으로 바꾸는 판정 함수 한 개 추가 (`/amr_XX/robot_state` 구독).
- **추월 목표·복귀 밴드**: `DwaInput` 에 측방 목표 y_ref 를 추가해 J_head/J_path 가 쓰게 한다.

## 8. 검증 결과

§8.1–8.2 는 이상화 시험대([path_tracking.md](path_tracking.md) §5.1 과 같은 조건: 실제 Nav2 서버 + velocity_profiler +
운동학 시뮬레이터(lidar_link σ 0.03 스캔), 참값 CTE, 위치추정 오차 없음, safety_node 없음, 1.0 m/s, 경로마다 1 회,
2026-09-22 23:17–23:44 KST, load average 20–87 — 23:40 이후 다른 사용자 작업이 겹침). 통합 체인(Gazebo) 결과는
[path_tracking.md](path_tracking.md) §5.3, 제어기 비교는 §6.

### 8.1 경로 추종 — 본 DWA vs Nav2 DWB (같은 controller_server, 같은 한계, 이상화 시험대)

DWB 는 기본 critic(DWB_A)과, 같은 시험대에서 6 변형을 돌려(`mk_dwbgrid.py`, 표 아래) 4 경로 모두 성공한 두 변형 중
직선·통로 시간이 짧은 DWB_E 를 "공정하게 튜닝한 DWB" 로 둔다: RotateToGoal slowing_factor 1.0 · lookahead 0.3 s,
sim_time 1.5 s, vx × vθ 샘플 20 × 40, PathDist 48, trans_stopped_velocity 0.1 (DWA 는 11 × 31).

| 제어기 | 경로 | 성공 | CTE 직선 평균 / 최대 [cm] | CTE 곡선 평균 / 최대 [cm] | 주행 시간 실제 / 예측 [s] (오차) | 목표 통과량 [cm] | 주기 평균 / 최대 [ms] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **DWA** | 직선 20 m | ✓ | 0.0 / 0.1 | – | 21.3 / 21.5 (1.2 %) | −2.3 | 0.92 / 1.22 |
| | U 턴 R 2 m | ✓ | 0.1 / 1.7 | **0.4 / 1.9** | 13.6 / 13.8 (1.5 %) | −2.4 | 0.91 / 1.28 |
| | S 자 R 3 m | ✓ | 1.1 / 1.6 | **0.9 / 2.9** | 14.8 / 15.0 (1.7 %) | −1.3 | 0.99 / 1.32 |
| | 좁은 통로 0.60 m | ✓ | 0.6 / 1.4 | – | 9.8 / 10.0 (2.6 %) | −1.9 | 1.52 / 3.09 |
| DWB_A (기본 critic) | 직선 | ✓ | 0.0 / 0.0 | – | 23.2 / 21.5 (7.2 %) | −8.1 | – |
| | U 턴 | ✗ | 1.6 / 11.7 | 7.2 / 15.5 | 33.4 (진행 없음 → 실패) | −9.7 | – |
| | S 자 | ✗ | 0.4 / 3.1 | 9.3 / 17.6 | 35.6 (진행 없음 → 실패) | −10.8 | – |
| | 좁은 통로 | ✓ | 0.0 / 0.0 | – | 12.0 / 10.0 (16.8 %) | −8.6 | – |
| **DWB_E (튜닝)** | 직선 | ✓ | 0.4 / 1.5 | – | 29.7 / 21.5 (27.6 %) | −8.2 | – |
| | U 턴 | ✓ | 1.8 / 7.7 | 4.8 / 11.2 | 15.5 / 13.8 (11.1 %) | −8.2 | – |
| | S 자 | ✓ | 0.8 / 1.1 | 6.6 / 12.5 | 16.0 / 15.0 (6.3 %) | −8.2 | – |
| | 좁은 통로 | ✓ | 0.1 / 1.1 | – | 11.1 / 10.0 (9.6 %) | −8.3 | – |

DWB 튜닝 격자 (같은 실행, 4 경로 성공 수): A 기본 2/4 · B RotateToGoal 완화 3/4 · C 정렬 critic 완화 1/4 ·
D sim_time 1.5 + 샘플 20 × 40 + PathDist 48 4/4 · **E = B + D + trans_stopped 0.1 4/4** · F GoalAlign 제거 2/4.
실패는 모두 목표 직전 정렬 단계의 진행 검사(20 s 동안 0.3 m) 초과다.

- DWA 곡선 CTE 평균 0.4–0.9 cm · 최대 2.9 cm (명세 곡선 10 cm, 직선 5 cm), 좁은 통로 풋프린트 최소 여유 0.086 m.
- 튜닝한 DWB_E 도 4 경로를 모두 끝내지만 곡선 CTE 평균이 7–12 배 크고(4.8–6.6 cm, **최대 11.2–12.5 cm 로 명세 10 cm
  초과**), 목표 앞 8 cm 에 서며, 직선에서 저속 구간이 길어 주행 시간 예측 오차가 27.6 % 다. DWB 는 목표 접근 감속을
  RotateToGoal·GoalDist critic 의 비용 균형으로만 만들고, 저크·지연을 넣은 정지거리 역함수(§1.6)가 없다.
- 통합 체인(Gazebo) 결과는 [path_tracking.md](path_tracking.md) §5.3 (DWA, PurePursuit), 제어기 비교는 §6.

### 8.2 NavigateToPose (전역 AStar + DWA, 12 목표)

이상화 시험대: 12 / 12 성공, 계획 경로 주행 시간 예측 오차 평균 **2.2 %**, 최대 **4.4 %** (명세 15 %), 풋프린트 최소
여유 0.93 m (표: [path_tracking.md](path_tracking.md) §5.2). 12 목표 중 (0, −6.5) → (0, −14) 는 좁은 통로 남쪽이지만
계획기가 10.6 m 우회 경로를 택했다(좁은 통로 배율, [costmap.md](costmap.md) §4.1) — 통로 주행은 §8.1 의 `narrow`
경로와 [costmap.md](costmap.md) §6.2 에서 따로 확인. 통합 체인의 무작위 목표 쌍 결과는 §6.

### 8.3 동적 장애물 (VO / TTC)

- 단위 시험: `ConeCases` (정면·절단·이탈·경계 ±0.02 rad·겹침), `VoTimeMatchesCone`, `VelocityObstacleRejectsFastCrossing`
  (1 m/s 횡단 장애물 앞에서 v > 0.5 m/s 샘플만 제외), `VelocityObstacleSaturationKeepsVoAndBrakes`,
  `TtcCostSlowsBeforeVoSaturates`, `FirstContactTimeHeadOn` (정면 2 m/s → TTC 2.000 s), 폐루프
  `ClosedLoopCrossingKeepsVoRadius` (횡단 4 경우: 최소 중심 거리 1.111 m, 사각형 여유 0.50–0.55 m) ·
  `ClosedLoopHeadOnYields` (정면 3 m / 2 m, 출발 0.8 m/s: 가속하지 않고(최고 0.75) 최근접 순간 속도 0 — 장애물이
  비키지 않아 중심 거리는 0.01 / 0.00 m; 0.7 m 비낀 정면은 여유 0.285 m 로 지나감).
- 리뷰 하네스 `vo_probe` (위 §2.1 표): 이 기준에서 다시 링크해 같은 값.
- **Gazebo 통합 체인** (배포 설정, 1.0 m/s 횡단 actor, 26 회 — [costmap.md](costmap.md) §6.3): 충돌 0, 풋프린트 ↔ actor
  최소 부호 거리 0.33 m, 원래 경로 이탈 최대 0.83 m (중앙값 0.12 m), 복귀 5 s 이내 13 / 14 (최대 8.8 s — 미달 1 회는
  VO 회피 뒤 같은 방향으로 한 바퀴 돈 경우). 이 26 회의 DWA 주기 3 309 개(r1) 중 VO 로 샘플이 빠진 주기 95, 포화 26.
