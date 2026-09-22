# DWA 지역 계획기 (명세 4.4 Local Planner, 4.7 동적 장애물 회피)

> 구현: `src/amr_navigation/include/amr_navigation/core/{dwa,footprint,velocity_obstacle,geometry}.hpp`
> (ROS 비의존 코어), `src/amr_navigation/src/ros/dwa_controller.cpp` (`amr_navigation::DWAController`,
> `nav2_core::Controller` 플러그인, `controller_id: DWA`). 설정: `config/nav2_params.yaml` `controller_server.DWA.*`.
> 연구 브리프: `research/local-planning` §3 (기준 DWA)을 구현했고, §5 (PVT-DWA v2)의 VO/TTC 는 결정론적
> 기본형으로 구현, 공분산 기회제약·ORCA 는 확장 지점으로 남겼다 (§7).

## 1. 한 제어 주기의 절차 (Δt_c = 1/20 s)

`DwaPlanner::compute()` 는 Fox, Burgard & Thrun (1997) 의 네 단계를 그대로 따른다.

```
(1) 동적 창 V_d        ← 창 중심 (v_c, ω_c), 가속 한계, 외부 속도 제한
(2) 속도 샘플링        ← N_v × N_ω 격자 (+ ω = 0 열, + 제동 후보)
(3) 궤적 시뮬레이션    ← 샘플마다 (v, ω) 일정 원호의 정확 적분, T_sim(v)
(4) 충돌 검사 + VO     ← 풋프린트(외곽선) 충돌, 동적 장애물 VO 원뿔
(5) 비용 계산          ← 6 항 정규화 가중합
(6) 최적 속도 선택     ← 최소 비용 (유효 샘플 없으면 제동 후보)
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
| J_dyn | max(0, 1 − TTC₀ / T_pred), T_pred = 5 s | 1.5 | 추적 장애물과의 예측 충돌 시각 (§2) |

**추종 지평과 안전 지평의 분리 (`path_eval_time`)**: 충돌·여유 비용은 정지거리를 덮는 전체 롤아웃 T_sim(v)
(1 m/s 에서 1.5 s)으로 보지만, 헤딩·경로 항은 앞 0.8 s 만 본다. (v, ω) 일정 원호는 곡률이 하나뿐이라, 곡률 부호가
바뀌는 경로(S 자: R 3 m 30°·60° 물결, 호 1.6–3.1 m)를 1.5 m 전체로 평가하면 두 구간의 **평균 곡률**을 가진 원호가
뽑혀 안쪽을 가로지른다. 앞부분만 평가하면 지금 구간의 곡률을 따르고, 다음 구간은 20 Hz 재계획이 맡는다
(추종은 짧은 제어 지평, 안전은 긴 정지 지평 — MPC 의 두 지평 분리와 같은 생각). 단위 테스트
`Dwa.ShortTrackingHorizonFollowsSCurve`: 이상 폐루프 S 자 평균 CTE 7.8 cm(전체 평가) → 1.8 cm(앞 0.8 s).

유효 샘플(충돌 없음, VO 밖) 중 J 최소를 고른다. 유효 샘플이 없으면 제동 후보를 내고, 정지 상태에서
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

VO 안의 샘플은 제외한다 (`use_velocity_obstacles`, 파라미터로 끔). VO 가 유효 샘플을 모두 제외하면 그 주기만
VO 를 끄고(충돌 검사는 유지) TTC 비용으로 고른다 (`vo_fallback`, 마주 오는 장애물을 좁은 창으로 피할 수 없을 때
멈추기보다 감속·회피를 택함). 단위 테스트 `ConeCases` (정면 접근, 시간 절단, 이탈, 원뿔 경계 ±0.02 rad,
이미 겹침), `VelocityObstacleRejectsFastCrossing` (횡단 장애물: v > 0.5 m/s 샘플만 제외), `…FallbackWhenAllRejected`.

### 2.2 예측 충돌 시각 TTC₀ (현 근사의 보완)

현 근사는 원호가 크게 휘면 충돌을 놓칠 수 있으므로(브리프 c03), 원호 롤아웃을 T_pred = 5 s 로 연장해 장애물 예측
위치 o(t) = o + u·t 와의 거리가 R_d = r_robot + r_obs + 0.5 (critical zone) 미만이 되는 첫 시각을 구간별 2차식 근으로
정확히 계산한다 (`core::firstContactTime`, `FirstContactTimeHeadOn`: 정면 2 m/s 접근 → 2.000 s).
J_dyn = max(0, 1 − TTC₀/T_pred).

## 3. Nav2 통합

- 전역 경로(map) → 코스트맵 프레임(odom) 변환은 TF 한 번 조회 후 2D 강체 합성, 로봇 최근접 정점부터
  `path_horizon` 6 m 까지만 (새 경로는 앞 20 m, 이후 직전 인덱스부터 10 m 창에서 최근접 탐색 → 되돌아오는
  경로에서 뒤 구간으로 튀지 않음). 창 끝이 실제 목표가 아니면 목표 감속·목표 너머 검사 제외를 끈다.
- `payload/mass` (latched) → 가속 한계 × m/(m + m_payload) (공차 47.6 kg).
- 발행: `local_plan` (선택 궤적), `dwa/stats` [cycle_ms, n_samples, n_valid, n_collision, n_vo_rejected,
  vo_fallback, best_ttc, v, w, d_goal] — 주기 시간·회피 동작 로그.
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
| `path_horizon` | 6.0 | m | 로컬 코스트맵 반폭 |
| `use_dynamic_obstacles` / `use_velocity_obstacles` | true / true | | |
| `prediction_time` | 5.0 | s | TTC₀ 지평 |
| `dynamic_margin` / `vo_margin` | 0.5 / 0.3 | m | safety critical / e-stop 거리 |
| `vo_time_horizon` / `vo_max_range` | 2.0 / 6.0 | s / m | |
| `dynamic_speed_threshold` | 0.2 | m/s | 추적기 is_dynamic 기준과 같게 |
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
맞는다 (예: 기본값 U 턴 곡선 CTE 평균 0.5 cm vs ROS 0.4 cm, 주행 시간 13.6 vs 13.7 s).

| 설정 (heading / path, ℓ, 평가 지평) | 직선 T [s] (예측 21.5) | U 턴 곡선 CTE 평균 / 최대 [cm] | U 턴 T 오차 | S 자 곡선 CTE 평균 / 최대 [cm] | S 자 T 오차 | 목표 통과량 [cm] |
| --- | --- | --- | --- | --- | --- | --- |
| 출발점: 0.8 / 1.2, ℓ 0.8v+0.5 [0.6, 2.0], 전체 롤아웃 | 23.00 | 3.7 / 9.7 | 17.6 % | **16.1 / 35.9** | 20.0 % | −7.9 (느린 꼬리) |
| 평가 지평 0.8 s 만 | 21.30 | **29.4 / 36.4** | 5.3 % | 11.7 / 23.6 | 1.6 % | −2.3 |
| + ℓ 0.4v+0.3 [0.4, 1.2] | 21.30 | 10.3 / 13.5 | 3.4 % | 6.3 / 10.4 | 2.0 % | −2.3 |
| + path 1.2 → 3.0 | 21.30 | 0.4 / 2.0 | 1.5 % | 0.9 / 2.8 | 1.4 % | −2.8 |
| + heading 0.8 → 0.5, path 1.5 | 21.30 | 0.5 / 2.1 | 1.5 % | 1.0 / 3.1 | 1.6 % | −2.4 |
| **채택: heading 0.6, path 2.0** | **21.30** | **0.5 / 2.1** | **1.5 %** | **0.9 / 3.0** | **0.3 %** | **−2.3** |
| 채택값, 0.5 m 옆에서 출발 | 21.45 | 0.5 / 2.0 | 0.0 % | (출발 구간 포함) 7.3 / 50 | 0.3 % | −2.5 |

관찰: (1) 평가 지평만 줄이면 긴 헤딩 목표점(ℓ ≤ 2.0 m)이 원 경로에서 안쪽 정상상태 오차(29 cm)를 만든다 — 헤딩
목표점도 평가 끝점 기준으로 짧게 해야 한다. (2) path 가중이 1.2 → 2.0 을 넘으면 원 경로의 정상상태 오차가
10 cm → 0.5 cm 로 떨어진다(헤딩 항이 만드는 안쪽 편향을 경로 항이 이김). 3.0 도 비슷하지만 회피 뒤 복귀가 급해지는
쪽이라 2.0 을 택했다. (3) 0.5 m 옆에서 출발해도 진동 없이 수렴한다.

## 5. 계산 예산 (20 Hz, 1 코어)

최악 셀 접근 ≈ 342 샘플 × 25 자세 × (외곽선 ≈ 40 셀) — 대부분의 자세는 2 단계 조기 판정(O(1))으로 끝난다.
한 주기 계산 시간 (`dwa/stats` 의 cycle_ms, 경로 창 변환·트랙 변환 포함, 단일 스레드):

| 측정 | 평균 | p95 | 최대 | 예산 50 ms 대비 |
| --- | --- | --- | --- | --- |
| 단위 테스트 `Dwa.CycleTimeBudget` (11 × 31, 12 × 12 m, 랙 장애물) | 1.1–1.6 ms | | | 3 % |
| ROS 폐루프, 직선·U 턴·S 자 (빈 통로) | 0.82–0.84 ms | 1.04–1.12 ms | 1.28 ms | 2.6 % |
| ROS 폐루프, 0.60 m 좁은 통로 (외곽선 검사 다수) | 1.00 ms | 1.60 ms | 1.83 ms | 3.7 % |

(load average 2–18, 32 스레드 호스트.) Pure Pursuit 는 0.02 ms (최대 0.05 ms).

## 6. TEB 와의 비교 (명세 4.4) — 한계와 대체

`teb_local_planner` 는 **ROS 2 Humble 바이너리 릴리스가 없다** (image 의 dpkg·index.ros.org 확인, 브리프 §2).
이번 산출물에서는 같은 controller_server 에 **Nav2 DWB** (`controller_id: DWB`, 같은 샘플 수 11 × 31, 같은 한계)를
비교군으로 올렸다 (§8). TEB 비교를 재현하려면 소스 빌드가 필요하다:

```bash
# 컨테이너 안, 별도 워크스페이스 (본 저장소 빌드와 분리)
mkdir -p ~/teb_ws/src && cd ~/teb_ws/src
git clone -b ros2-master https://github.com/rst-tu-dortmund/teb_local_planner.git
git clone -b ros2 https://github.com/rst-tu-dortmund/costmap_converter.git
sudo apt install -y libsuitesparse-dev ros-humble-libg2o
cd ~/teb_ws && rosdep install --from-paths src -i -y && colcon build --symlink-install
# nav2_params.yaml controller_plugins 에 "TEB" 추가:
#   TEB: {plugin: "teb_local_planner::TebLocalPlannerROS", max_vel_x: 1.0, acc_lim_x: 1.0, ...}
```

원리 비교 (브리프 §3.7):

| | 본 DWA | Nav2 DWB | TEB |
| --- | --- | --- | --- |
| 최적화 | 동적 창 샘플 + 정규화 비용합 (결정적) | 샘플 + critic 합 | 시간 탄성 밴드, g2o 희소 비선형 최소제곱 |
| 궤적 모델 | (v, ω) 일정 원호 1 개 | 같음 | 다중 자세 + 시간 간격 (가감속이 시간에 따라 변함) |
| 동적 장애물 | 트랙 등속 예측: VO 샘플 제외 + TTC 비용 | 코스트맵 현재 점유만 | `include_dynamic_obstacles` (등속 예측, costmap_converter 필요) |
| 장점 | 단순·설명 가능(항별 로그)·주기 시간 예측 가능, 동적 창으로 가속 한계 자연 반영 | 플러그인 critic 구조 | 좁은 공간·후진 기동, 시간 최적 궤적, 호모토피 탐색 |
| 단점 | 한 주기 앞의 원호만 봐서 U 자 회피·후진 기동이 약함(국소최소), 샘플 해상도 한계 | 같음 + 동적 예측 없음 | 수십 ms 계산, 국소최소·파라미터 민감, Humble 미지원 |

## 7. 확장 지점 (연구 브리프 §5 PVT-DWA v2)

- **공분산 기회제약**: `DynamicObstacle` 에 위치·속도 공분산 필드를 더하고 `firstContactTime` 의 R 을 σ 로 키우면
  된다 (TrackedObstacle 메시지 확장 필요 — 브리프 §7.3, 인지 영역과 합의).
- **ORCA (동료 AMR)**: `inVelocityObstacle` 을 반평면 제약으로 바꾸는 판정 함수 한 개 추가 (`/amr_XX/robot_state` 구독).
- **추월 목표·복귀 밴드**: `DwaInput` 에 측방 목표 y_ref 를 추가해 J_head/J_path 가 쓰게 한다.

## 8. 검증 결과

시험대와 측정 조건은 [path_tracking.md](path_tracking.md) §5 와 같다 (실제 Nav2 서버 + velocity_profiler + 운동학
시뮬레이터, 참값 CTE, 1.0 m/s, 11:17–11:28 KST, load average 2–18).

### 8.1 경로 추종 — 본 DWA vs Nav2 DWB (같은 controller_server, 같은 샘플 수 11 × 31, 같은 한계)

| 제어기 | 경로 | 성공 | CTE 직선 평균 / 최대 [cm] | CTE 곡선 평균 / 최대 [cm] | 주행 시간 실제 / 예측 [s] (오차) | 목표 통과량 [cm] | 주기 평균 / 최대 [ms] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **DWA** | 직선 20 m | ✓ | 0.0 / 0.1 | – | 21.3 / 21.5 (1.2 %) | −1.8 | 0.82 / 1.18 |
| | U 턴 R 2 m | ✓ | 0.2 / 1.7 | **0.4 / 2.0** | 13.6 / 13.8 (1.5 %) | −2.0 | 0.84 / 1.28 |
| | S 자 R 3 m | ✓ | 1.0 / 1.6 | **0.9 / 2.9** | 14.8 / 15.0 (1.3 %) | −2.2 | 0.83 / 1.24 |
| | 좁은 통로 0.60 m | ✓ | 0.0 / 0.1 | – | 9.8 / 10.0 (2.5 %) | −1.8 | 1.00 / 1.83 |
| DWB | 직선 | ✓ | 0.0 / 0.0 | – | 23.2 / 21.5 (7.4 %) | −8.3 | – |
| | U 턴 | ✗ | 1.9 / 10.9 | 7.8 / 16.7 | 33.7 (진행 없음 → 실패) | −9.3 | – |
| | S 자 | ✗ | 0.5 / 3.2 | 9.3 / 17.8 | 36.4 (진행 없음 → 실패) | −10.9 | – |

- DWA 는 곡선 CTE 평균 0.4–0.9 cm (명세 곡선 10 cm, 직선 5 cm), 좁은 통로 풋프린트 최소 여유 0.10 m (충돌 0).
- 튜닝 전 설정(§4.1 표 첫 줄)으로 같은 시험대를 돌린 1 차 결과는 S 자 곡선 CTE 14.0 / 36.9 cm, 주행 시간 오차
  19.6 %, U 턴 15.9 % 였다 — 평가 지평 분리와 가중 조정으로 위 값이 됐다.
- DWB (Nav2 기본 critic: PathAlign/PathDist 32, GoalAlign/GoalDist 24, RotateToGoal 32 등)는 곡선 경로에서 목표 직전
  정렬 단계에 진행 검사(20 s 동안 0.3 m)에 걸려 실패했고, 곡선 CTE 도 본 DWA 의 10–20 배다. DWB critic 튜닝은 이
  산출물의 범위 밖이며, 같은 샘플 수·한계에서의 기본 설정 비교다.

**Gazebo (실제 로봇 모델·창고 월드, headless, RTF 0.99–1.00)** — 같은 형태의 4 경로를 DWA 로 모두 성공: 곡선 CTE
평균 0.5 cm (U 턴) / 1.1 cm (S 자), 최대 3.5 cm, 주행 시간 오차 1.1–3.0 %, 목표 통과량 −1.8 ~ −2.7 cm (앞에 섬),
주기 평균 0.80–0.93 ms · 최대 1.39 ms (표: [path_tracking.md](path_tracking.md) §5.3).

### 8.2 NavigateToPose (전역 AStar + DWA, 12 목표)

12 / 12 성공, 계획 경로 주행 시간 예측 오차 평균 **2.2 %**, 최대 **4.3 %** (명세 15 %), 풋프린트 최소 여유 0.93 m
(표: [path_tracking.md](path_tracking.md) §5.2). 팀 공용 `amr_evaluation` 의 `analyze` 게이트(재계획 `plan` 기준)도
직선 CTE 평균 0.30 cm · 곡선 1.77 cm 로 PASS ([path_tracking.md](path_tracking.md) §5.4). 12 목표 중 (0, −6.5) → (0, −14) 는 좁은 통로 남쪽이지만 계획기가 10.6 m
우회 경로를 택했다(좁은 통로 배율, [costmap.md](costmap.md) §4.1) — 통로 주행은 §8.1 의 `narrow` 경로로 따로 확인.

### 8.3 동적 장애물 (VO / TTC)

폐루프 시험은 통합 시험(`tests/`, itests) 담당이고, 여기서는 단위 시험으로 확인했다: `ConeCases` (정면·절단·이탈·
경계 ±0.02 rad·겹침), `VelocityObstacleRejectsFastCrossing` (1 m/s 횡단 장애물 앞에서 v > 0.5 m/s 샘플만 제외),
`VelocityObstacleFallbackWhenAllRejected`, `FirstContactTimeHeadOn` (정면 2 m/s → TTC 2.000 s),
플러그인 `AcceleratesAlongPlanAndRespectsLimits` (트랙 메시지 map → 코스트맵 프레임 변환·나이 보정 경로).
운동학 시뮬레이터는 `moving_obstacles:=x,y,vx,vy,r,…` 로 왕복 원판 장애물과 그 참값 트랙을 발행하므로 같은 시험대로
재현할 수 있다.
