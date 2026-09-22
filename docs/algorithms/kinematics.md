# 차동 구동 기구학 · 휠 오도메트리 · 센서 전처리 (명세 4.2)

> 구현: `src/amr_localization` — `diff_drive_kinematics`, `encoder_model`, `odometry_covariance`,
> `wheel_odometry` (C++ 라이브러리, ROS 비의존) + `wheel_odometry_node` / `imu_filter_node` /
> `scan_filter_node` (얇은 래퍼). 파라미터: `src/amr_localization/config/*.yaml`.
> 설계 기준: research brief `kinematics-odometry` §2~3 (베이스라인). 측정 조건은 §7 머리
> (2026-09-22 통합 트리 a7d43f7 + glue0 기준 재측정; 조건·부하·RTF 는 표마다).

## 1. 순기구학 (Forward Kinematics)

기호: 좌/우 바퀴 반지름 `r_L, r_R` [m], 바퀴 간격 `b` [m], 바퀴 각속도 `ω_L, ω_R` [rad/s].

```
v = (r_R ω_R + r_L ω_L) / 2          (전진 속도, base_footprint x)
ω = (r_R ω_R − r_L ω_L) / b          (요 각속도)
v_y ≡ 0                               (비홀로노믹 구속)
```

한 주기 증분: `Δs_i = r_i Δφ_i`, `Δs = (Δs_R + Δs_L)/2`, `Δθ = (Δs_R − Δs_L)/b`.
역기구학 `ω_R = (v + ωb/2)/r_R`, `ω_L = (v − ωb/2)/r_L` 도 함께 제공한다 (테스트·시뮬레이터용).
좌우 반지름을 따로 두어 UMBmark 캘리브레이션 결과(`left/right_wheel_radius`)를 바로 넣을 수 있다.

| 코드 | 위치 |
| --- | --- |
| `forwardKinematics`, `inverseKinematics`, `wheelAnglesToIncrement` | `src/diff_drive_kinematics.cpp` |

### 1.1 적분: 정확한 원호 해 (오일러 아님)

주기 동안 `(v, ω)` 가 일정하면 로봇은 반지름 `R = Δs/Δθ` 의 원호를 그린다. 원호의 현(chord) 길이는
`2R sin(Δθ/2) = Δs · sinc(Δθ/2)` 이고 방향은 `θ + Δθ/2` 이므로

```
x' = x + Δs · sinc(Δθ/2) · cos(θ + Δθ/2)
y' = y + Δs · sinc(Δθ/2) · sin(θ + Δθ/2)          sinc(u) = sin(u)/u  (u→0 은 테일러 전개)
θ' = θ + Δθ
```

`Δθ ≠ 0` 에서 `x' = x + (Δs/Δθ)[sin(θ+Δθ) − sin θ]` 와 같고 `Δθ → 0` 에서도 특이점이 없다.
비교용으로 오일러(`x += Δs cos θ`, 곡선에서 O(Δt) 편향)와 중점(상대 오차 ≤ Δθ²/24) 을 같은 인터페이스
(`integration: exact_arc | midpoint | euler`) 로 둔다.

검증 (`test_diff_drive_kinematics`, `test_wheel_odometry`):
- 직진·제자리 회전·반지름 1 m 90° 원호에서 원호 해 = 해석해 (오차 < 1e-9 m), 한 스텝으로 적분해도 정확
- 곡선 10 s (v = ω = 1): 오일러는 Δt 를 반으로 줄이면 오차가 반으로 (1차), 원호는 0
- 중점 적분 단계 오차 ≤ `Δs·Δθ²/24` (격자 전수)

## 2. 휠 엔코더 모델 (4096 틱 + 거리당 슬립 잡음, 다회전 카운터)

Gazebo Fortress 의 `JointStatePublisher` 는 잡음 없는 누적 조인트 각을 물리 스텝마다(최대 1 kHz) 준다.
`WheelEncoderModel` 이 실제 엔코더처럼 만든다 (`config/sensors.yaml wheel_encoder`, `wheel_odometry.yaml`):

1. **샘플링**: `update_rate` 50 Hz 로 솎는다. 사이 입력은 건너뛰지만 누적 각 기반이라 이동량은 잃지 않는다.
2. **다회전 카운터**: Gazebo 조인트 각은 감기지 않는 누적각이므로 샘플 차를 그대로 더한다. 이전 구현은 차를
   (−π, π] 로 접어(`remainder`) 샘플 간 공백이 π/ω_바퀴 를 넘으면 한 바퀴(2πr = 0.518 m)를 잃었다 — 1 m/s 에서
   0.26 s, 2 m/s 에서 0.13 s (구독은 best-effort depth 5, 입력 1 kHz). (−π, π] 로 감겨 오는 입력
   (`joint_position_wrapped: true`)은 `joint_states.velocity × 공백` 에 가장 가까운 2π 분기를 고르고, 속도가 없는데
   공백이 π/ω_max 를 넘으면 분기 모호로 표시한다. 연속 입력에서도 |Δφ| > ω_max·Δt + π 인 불가능한 점프(조인트 리셋)
   는 표시한다. 표시된 샘플은 그 구간 바퀴 변위 분산에 (2πr)² 를 더해 EKF 가 믿지 않게 한다 (`max_wheel_speed` 30 rad/s
   ≥ (v_max 2.0 + ω_max 1.5 × b/2)/r = 27.5).
3. **양자화**: 누적 각을 틱으로 `n = round(φ N / 2π)` (int64, 4 h 연속 운용에도 오버플로 없음).
   1 틱 = `2π/4096` rad = `2πr/4096 = 0.127 mm` (r = 0.0825).
4. **거리당 슬립 잡음**: 양자화된 주기 회전 Δφ_q 에 `η ~ N(0, σ_s² φ_ref |Δφ_q|)` 를 더한다 (φ_ref = ℓ_ref / r,
   ℓ_ref = `slip_reference_distance` 0.01 m, σ_s = 0.01, 바퀴마다 독립). 변위로 쓰면 `Var = σ_s² ℓ_ref |Δs|` —
   분산이 굴린 거리에 비례하므로 누적 잡음이 **속도·샘플 주기와 무관**하다. 이전 모델(주기마다 `(1 + N(0, σ_s))` 곱)은
   거리 1 m 당 분산이 σ_s²·v/f 라 빠를수록·주기가 낮을수록 잡음이 커졌다 (리뷰 몬테카를로: 10 m 직진 along-track σ
   0.5/1/2 m/s 에서 2.17/3.02/4.41 mm). ℓ_ref = 0.5 m/s × 50 Hz 한 주기 이동이라 그 조건에서는 두 모델이 같다
   (이전 Gazebo 드리프트 실험이 NEES ≈ 3 으로 검증한 조건).

검증 (`test_encoder_model`, `test_wheel_odometry`): 양자화 계단 = 정확히 1 틱, 누적 각 양자화 오차 |q| ≤ δ/2,
주기 차 오차는 MA(1) (`Var = δ²/6`, lag-1 공분산 `−δ²/12`), 정규화 슬립 잡음 표본 σ 가 10 000 표본에서 1 ± 10 %,
**같은 총 회전 50 rad 를 0.05 rad / 0.5 rad 주기로 굴려도 누적 분산 = σ_s² φ_ref Θ (±10 %)**, 같은 seed 재현성,
**다회전 카운터**: 연속 입력 0.3 s 공백 @ 1·2 m/s (바퀴 3.6 / 7.3 rad) 무손실, 감긴 입력 + 속도 힌트 무손실, 감긴 입력 +
속도 없음 → 모호 표시·트위스트 분산 ≥ (2πr)²/(4T²), 0.02 s 에 100 rad 점프 → 표시.

실측 (라이브러리 탐침 `odom_probe`, 잡음 끔, 1 kHz 입력에 1 s 마다 0.3 s 공백): 1 m/s 4 s 주행 오차 −0.03 mm,
2 m/s 8 m −0.05 mm (이전 구현: 1 m/s 에서 x = 3.482 vs 4.000 m, 리뷰 재현). 슬립 잡음 10 m 직진 along-track σ
(300 seed): 50 Hz 0.5/1/2 m/s → 2.20/2.13/2.10 mm, 100 Hz → 2.14/2.20/2.14 mm (폐형 √(kL/2) = 2.24 mm, §4.1).

## 3. 공분산 (EKF 입력 계약, 명세 4.3 "공분산 근거")

`config/ekf.yaml` 은 `wheel_odom` 의 **트위스트**(vx, vy, vyaw)만 융합하고 공분산은 메시지에서 읽는다.
따라서 퍼블리셔가 잡음 모델에서 공분산을 계산해 채운다.

### 3.1 바퀴 변위 분산

```
주기 (자세 전파용): Var(Δs_i) = (σ_s² ℓ_ref + k) |Δs_i|          (k: 물리 슬립 계수, 사전값 0)
발행 구간 T (트위스트용): σ_i²(T) = σ_s² ℓ_ref Σ|Δs_i| + k |Σ Δs_i| + δ²/6
분기 모호·불가능 점프 샘플: 위 두 식에 (2πr)² 가산
```

양자화는 **누적 카운트에서 유계**(|q| ≤ δ/2)이므로 자세 전파에 매 주기 백색으로 더하면 가짜 랜덤워크
(1 h 에 σ ≈ 2 cm)가 된다. 그래서 자세에는 발행 시 한 번(`q_end − q_start` 의 분산 δ²/6)만 더하고,
실제로 차분을 취하는 트위스트에만 구간 양 끝 오차 δ²/6 을 넣는다.

### 3.2 트위스트 공분산 (`twist.covariance`, 6×6 행 우선)

`v = (Δs_R + Δs_L)/(2T)`, `ω = (Δs_R − Δs_L)/(bT)` 에 오차 전파:

```
Var(v)    = (σ_R² + σ_L²) / (4T²)
Var(ω)    = (σ_R² + σ_L²) / (b² T²)
Cov(v, ω) = (σ_R² − σ_L²) / (2b T²)
Var(v_y)  = σ_vy0² + (κ_y v ω)²          (비홀로노믹 구속의 신뢰도, σ_vy0 = 0.02 m/s)
```

+ 파라미터 불확실성 항 `hᵀ Σ_ψψ h` (`h_v = (b/2)[φ̇_R, φ̇_L]`, `h_ω = [φ̇_R, −φ̇_L]`, `ψ = r/b`).
인덱스: `[0] = Var(v)`, `[7] = Var(v_y)`, `[35] = Var(ω)`, `[5] = [30] = Cov(v, ω)`.
평면 밖 성분은 `unfused_variance` (1e-3, two_d_mode 에서 무시).

수치 (식 대입, 직진 1 m/s, T = 0.02 s, σ_s = 0.01, ℓ_ref = 0.01 m, b = 0.362 m): 바퀴당
σ_i²(T) = 1e-6 × 0.02 + δ²/6 = 2.27e-8 m² → `σ_v = 5.3e-3 m/s`, `σ_ω = 0.029 rad/s`. 정지 시 양자화만
`σ_v = 1.8e-3 m/s`. 거리당 슬립이라 트위스트 분산은 속도에 비례한다 (σ ∝ √v).

### 3.3 자세 공분산 — 야코비안 전파 (증강 상태)

상태 `s = [p; ψ]`, `p = (x, y, θ)`, `ψ = (r_R/b, r_L/b)`. 원호 적분 `p' = f(p, Δs_R, Δs_L)` 의 야코비안
(`exactArcJacobians`, `u = Δθ/2`, `φ = θ + u`, `S = sinc(u)`, `S' = dS/du`):

```
F_p = ∂p'/∂p = [1 0 −Δs S sin φ;  0 1 Δs S cos φ;  0 0 1]
G   = ∂p'/∂(Δs, Δθ) = [S cos φ,  Δs(½S' cos φ − ½S sin φ);
                       S sin φ,  Δs(½S' sin φ + ½S cos φ);
                       0,        1]
M   = ∂(Δs, Δθ)/∂(Δs_R, Δs_L) = [½ ½; 1/b −1/b]
F_u = G M,   F_ψ = F_u diag(b Δφ_R, b Δφ_L)
```

전파 (`OdometryCovariance::propagate`):

```
Σ_pp ← F_p Σ_pp F_pᵀ + F_p Σ_pψ F_ψᵀ + F_ψ Σ_ψp F_pᵀ + F_ψ Σ_ψψ F_ψᵀ + F_u Σ_u F_uᵀ
Σ_pψ ← F_p Σ_pψ + F_ψ Σ_ψψ,        Σ_u = diag(Var Δs_R, Var Δs_L)
발행: Σ_pub = Σ_pp + F_u⁰ diag(δ²/6, δ²/6) F_u⁰ᵀ   (양자화 유계항 1 회)
```

파라미터 오차는 주기 간 **완전 상관**이라 매 주기 백색으로 더하면 `Var θ ∝ L` 로 과소평가된다.
증강 상태로 교차공분산을 들고 가면 체계 오차 몫이 `Var θ ∝ L²` 로 정확히 자란다
(`wheel_param_rel_stddev > 0` 일 때; 기본 0). 온라인 캘리브레이션(research brief GRC) 확장점:
`WheelOdometry::setWheelRadii`, `setParameterCovariance`, `OdometryCovariance::applyParameterUpdate`.

검증 (`test_odometry_covariance`, `test_wheel_odometry`):
- 야코비안 = 중앙 차분 (1e-6)
- 직진·원호·제자리 회전에서 전파 공분산 = Monte-Carlo (슬립 잡음 인코더, 시행 수천 회) 5 % 이내
- 직진 헤딩 분산 = 폐형 `2kL/b²`, 파라미터 오차에서 `Var θ` 가 L² 로 성장
- 트위스트 공분산 = 바퀴 잡음 Monte-Carlo
- 300 seed 원호 주행의 NEES 평균 ∈ [2.5, 3.5] (χ²₃ 기대값 3 → 공분산이 과신/과소 아님)

## 4. 누적 오차 (드리프트) 분석

### 4.1 폐형 (Kelly 2004; Chong & Kleeman 1997, 거리당 슬립 잡음 `k = σ_s² ℓ_ref`)

```
직진 L:      Var θ = 2kL/b²,   Var x ≈ kL/2,   Var y ≈ 2kL³/(3b²)
제자리 회전 Θ: Var θ = k|Θ|/b   (바퀴마다 |Θ|b/2 굴림)
```

횡오차가 L^{3/2} 로 자라는 것은 헤딩 오차(랜덤워크)가 이동 거리만큼 증폭되기 때문이다.
k = 1e-4 × 0.01 = 1e-6 m, L = 10 m, b = 0.362: `σ_θ = 0.0124 rad (0.71°)`, `σ_y = 0.071 m`, `σ_x = 2.2 mm`
(속도·주기와 무관). 제자리 2π: `σ_θ = 0.0042 rad (0.24°)`.
`drift_analysis.predicted_*` 가 같은 식을 쓰고 `test_drift.py` 가 Monte-Carlo 4000 회와 5 % 이내 일치를 확인한다.

체계 오차는 따로 자란다: 반지름 비 오차 `ε = (r_R − r_L)/r̄` → 직진 L 후 헤딩 `Lε/b`, 횡오차 `L²ε/(2b)`
(ε = 0.1 % 라도 20 m 에서 0.56 m) → 오도메트리 단독으로 5 cm 는 불가능하고 AMCL 보정이 전제다.

### 4.2 드리프트 측정 실험 (명세 4.2 "리포트 포함")

도구: `ros2 run amr_localization odom_drift_experiment` (GT 되먹임 주행 + `wheel_odom` 을 `ground_truth/odom`
과 시각 보간 비교, 시나리오마다 `wheel_odom/reset`) → `logs/eval/odom_drift/<run>/*.csv`,
`drift_report` 가 `summary.csv` / `summary.md` 작성. 시나리오: 직진 10 m (0.5 m/s), 제자리 2π (0.5 rad/s),
5 × 4 m 직사각형 반시계·시계 (UMBmark), 각 3 회.

시각 정렬: GT(50 Hz)는 브리지를 거쳐 `wheel_odom` 보다 늦게 도착할 수 있다. 외삽은 하지 않으므로
`wheel_odom` 샘플을 대기열에 두고 GT 가 그 시각을 지나면 보간해 기록한다 (즉시 처리하면 GT 가 아직 없는
샘플이 버려져, 고부하 호스트의 이전 실행에서 5 × 4 m 한 바퀴가 9~29 샘플만 남았다). 첫 런 전에는
`wheel_odom/reset` 서비스 발견을 최대 `service_wait`(15 s) 기다린다 — 리셋 없이 시작하면 공분산이 0 에서
시작하지 않아 NEES 비교가 무의미해진다.

결과: §7 참조.

## 5. IMU 전처리 (`imu_filter_node`)

1. **바이어스 보정**: 기동 후 `bias_estimation_time`(60 s) 정지 평균. Welford 누적으로 평균·분산을 구하고
   `b_g = mean(ω)`, `b_a = mean(a) − (0, 0, g)` (Gazebo 는 정지 시 +g 를 보고). 표본 σ 가 상한을 넘으면
   (움직임) 그 직전까지의 평균을 쓴다. `imu/calibrate` (std_srvs/Trigger) 는 정지 N 샘플을 평균해 바이어스를
   갱신하고 `logs/calibration/imu_bias_<robot>_<시각>.yaml` (다음 기동의 `--params-file`) 과 `imu_bias.csv` 를 쓴다.
2. **1차 IIR 저역 통과**: `y_k = α x_k + (1−α) y_{k−1}`, `H(z) = α/(1 − βz⁻¹)`, `β = 1−α`.
   α 는 후진 오일러 근사 `Δt/(τ+Δt)` 가 아니라 `|H(e^{jΩc})|² = 1/2` 를 정확히 풀어 구한다:
   `β = (2 − c) − sqrt((2 − c)² − 1)`, `c = cos(2π f_c Δt)`. 근사식은 f_s 100 Hz, f_c 20 Hz 에서 실제 −3 dB 점이
   13.7 Hz 로 어긋난다 (`BackwardEulerApproximationMissesCutoff`). 20 Hz: α = 0.673, 잡음 σ × 0.71, 군지연 ≈ 5 ms.
3. **출력 `imu/data`**: 각속도·가속도(중력 포함 — EKF 가 `imu0_remove_gravitational_acceleration` 로 뺀다),
   `orientation_covariance[0] = −1` (Gazebo orientation 은 노이즈 없는 GT 라 EKF 가 쓰면 안 된다),
   공분산 대각 = `σ² + SE²` (sensors.yaml 잡음 + 바이어스 추정 표준오차). LPF 후 분산을 넣지 않는 이유:
   LPF 출력은 자기상관이 있어 EKF 가 샘플을 독립으로 취급하면 분산만 줄인 값은 정보량을 과장한다.

검증 (`test_low_pass_filter`, `test_imu_filter`): −3 dB 가 정확히 f_c, 2 f_c 에서 이론 감쇠와 일치,
정현파 시뮬레이션 −3 dB 점이 f_c ± 2 %, DC 이득 1, 기동 추정으로 바이어스 제거, 이동 중 기동 시 폴백,
calibrate 요청, 공백 후 LPF 재시작.

## 6. LiDAR 전처리 (`scan_filter_node`)

빔 수·`angle_min/increment` 는 바꾸지 않고 값만 바꾼다 (AMCL·slam_toolbox·costmap 이 빔 인덱스로 각을 계산).
REP-117: `< range_min → −Inf`, `> range_max`·미검출 → `+Inf`, 제거 → `NaN`.

1. 거리 필터 `[max(센서, range_min), min(센서, range_max)]` = [0.10, 25.0] m
2. 각도 필터: `angle_window_min/max` 밖과 `angle_mask` 반시계 호(±π 넘는 호 지원) 제거
3. 아웃라이어(스페클): 빔 i 끝점 `P_i` 에 대해 ±`outlier_window` 빔 안에서
   `|P_i − P_j| ≤ outlier_thresh + outlier_range_gain · r_i · |i−j| · Δα` 인 이웃이 `outlier_min_neighbors` 개
   미만이면 고립점 → NaN. 임계의 `r·Δα` 항은 먼 거리에서 인접 끝점 간격이 커지는 것을 보정한다
   (20 m 에서 1 빔 간격 0.17 m). 360° 스캔은 배열 끝에서 이웃 탐색이 감긴다.
4. 섀도우(베일): 인접 끝점 선분과 광선의 각 `β = atan2(r_j sin Δα, r_i − r_j cos Δα)` 가 10° 미만/170° 초과이고
   **거리 차가 잡음으로 설명되지 않을 때** (`|r_i − r_j| > k·√2·σ_r`, k = 3, σ_r = sensors.yaml `noise_stddev` 0.03 →
   0.127 m) 먼 쪽 점 제거 (모서리 혼합 픽셀). 같은 면 두 빔의 잡음 차 N(0, 2σ_r²) 가 문턱을 넘을 확률은 0.27 %.
   잡음 조건이 없던 이전 판정(laser_filters 기본)은 저잡음 LiDAR 전제라, r·Δα ≪ σ_r 인 근거리에서 같은 면의 잡음 차만으로
   β 가 작아져 먼 쪽(+ 잡음) 점만 지웠다 → 남은 점이 짧게 치우침 (AMCL·SLAM·costmap·추적기·safety_node 공통 입력).

   실측 (라이브러리 탐침 `shadow_probe`, 수직 벽 ±20° 빔, σ 0.03 m, 200 스캔, 기본 설정 전 단계):

   | 벽 거리 | 이전 유지율 / 평균 거리 오차 | 잡음 조건 후 |
   | --- | --- | --- |
   | 0.3 m | 48.4 % / −18.3 mm | 99.7 % / −0.04 mm |
   | 0.5 m | 59.0 % / −14.7 mm | 99.7 % / −0.04 mm |
   | 1.0 m | 80.7 % / −7.4 mm | 99.7 % / −0.04 mm |
   | 1.5 m | 93.4 % / −2.8 mm | 99.7 % / −0.03 mm |
   | 2.0 m | 98.4 % / −0.7 mm | 99.7 % / −0.03 mm |

   (유지율 99.7 % 의 나머지 0.3 % 는 아웃라이어 단계.) Gazebo 정지 로봇(월드 원점, 랙 B-C 통로)에서 스캔당 섀도우 제거는
   14.6~15.3 개 (720 빔; 랙 끝·기둥 모서리의 실제 깊이 불연속, 재매핑 세션 로그).

검증 (`test_scan_filter`): REP-117 값, 각도 창·마스크·감긴 섹터, 고립 아웃라이어 제거 + 벽 유지,
작은 물체(이웃 지지) 보존, 360° 감김, 섀도우 제거, **잡음 벽(σ 0.03, 0.5~2 m) 유지율 ≥ 95 %·|편향| < 2 mm (잡음 조건을
끄면 같은 벽에서 < 75 %·< −1 cm 로 결함 재현)**, **잡음 속 베일 점 제거**, 빈 스캔, **메타데이터(angle_min/increment/빔 수)
보존**.

## 7. 측정 결과

이 절의 수치는 모두 통합 트리 (a7d43f7 + glue0 + 이 변경) 에서 2026-09-22 에 잰 것이다. 이전 기반(LiDAR 0.38 m, 바퀴 토크
한계 변경 전)의 합성 체인·보정 전 드리프트 표는 다시 재지 않았으므로 뺐다.

### 7.1 라이브러리 탐침 (Gazebo 없이, `amr_localization_core`)

| 항목 | 조건 | 결과 |
| --- | --- | --- |
| joint_states 공백 (다회전 카운터, §2) | 잡음 끔, 1 kHz 입력에 1 s 마다 0.3 s 공백, 4 s | 1 m/s: 최종 x 오차 −0.03 mm, 2 m/s: −0.05 mm (이전 구현 1 m/s: −518 mm, 리뷰) |
| 슬립 잡음 거리당 (§2, §4.1) | 10 m 직진 along-track σ, 300 seed | 50 Hz: 0.5/1/2 m/s → 2.20/2.13/2.10 mm; 100 Hz: 2.14/2.20/2.14 mm (폐형 2.24 mm) |
| 섀도우 단계 (§6) | 수직 벽 ±20° 빔, σ 0.03 m, 200 스캔 | 0.3~2 m 유지율 99.7 %, 평균 거리 오차 −0.04 mm (이전 48~98 %, −18.3 ~ −0.7 mm) |

### 7.2 Gazebo 드리프트 실험 (명세 4.2 "드리프트 측정 실험", `wheel_separation_multiplier` 1.00565)

`system.launch.py prefix:=amr_01/` (월드 + 로봇, 스택 없음) + `localization.launch.py mode:=odom` + `odom_drift_experiment`
(repetitions 3, 0.5 m/s · 0.5 rad/s, 가속 0.5 m/s² · 1 rad/s²), 스폰 (−16, −2, 0) (랙 B-C 통로), 기동 바이어스 추정 60 s 뒤 시작.
시나리오당 3 회, 12 런 750~2 776 샘플 (50 Hz). 2026-09-22 14:33~15:09 KST, load average 67~247 (다른 에이전트 시뮬레이션과
공유), RTF 중앙 0.23 (0.16~0.52). 원자료: 세션 `drift1` `summary.csv/md`.

| 시나리오 | 주행 | 최종 위치 오차 | 최종 헤딩 오차 | 발행 σ_pos | NEES (런별) | EKF(odom) 최종 |
| --- | --- | --- | --- | --- | --- | --- |
| 직진 10 m | 10.00 m | 6.2 ± 5.6 cm (0.4 / 6.4 / 11.7) | 0.51 ± 0.29° | 7.1 cm | 2.3 / 2.7 / 15.5 | 0.9 ± 0.7 cm |
| 제자리 2π | 6.29 rad | 0.06 ± 0.08 cm | 0.11 ± 0.08° | 0.08 cm | 0.4 / 8.9 / 0.1 | 0.06 cm |
| 5 × 4 m 반시계 | 18.00 m | 8.7 ± 2.2 cm | 0.32 ± 0.26° | 7.1 cm | 11.7 / 3.2 / 4.1 | 0.4 ± 0.2 cm |
| 5 × 4 m 시계 | 18.00 m | 7.0 ± 0.5 cm | 0.63 ± 0.94° | 7.1 cm | 5.9 / 4.2 / 5.6 | 0.5 ± 0.1 cm |

- 폐형 예측 (§4.1, 거리당 슬립만): 직진 10 m σ_y 7.2 cm, σ_θ 0.71°; 제자리 2π σ_θ 0.24°. 직진·회전 오차는 예측 σ 안이다.
- **UMBmark**: c_cw = (−0.3, −6.3) cm, c_ccw = (+2.1, −4.6) cm, E_max,syst = **6.3 cm**. 두 방향 모두 −y 로 끝나 헤딩(바퀴 간격)
  이 아닌 성분이다 (간격 오차면 cw/ccw 부호가 반대). 사각 NEES 3~12 (χ²₃ 기대 3 보다 큼) — 슬립 잡음 모델 밖의 체계 성분이
  사각(가감속 8 회·회전 4 회)에 수 cm 남는다.
- **유효 파라미터**: 거리 배율 L_odom/L_gt = **1.00040**, 회전 배율 Θ_odom/Θ_gt = **0.99969** (odom 이 쓰는 간격 b = 0.36 ×
  1.00565 = 0.36203 기준) → 이 기반에서 다시 식별한 배율 = 1.00565 × 0.99969 = **1.0053** (회전 1 바퀴에 0.11° 차이; 3 런 제자리
  회전 헤딩 오차 0.11 ± 0.08°). 이전 기반 식별값 1.00565 와의 차이 0.03 % 는 odom EKF 가 헤딩을 자이로로 정하므로 결과에
  영향이 없어(EKF(odom) 최종 0.06~0.9 cm) 설정은 1.00565 로 둔다. 실기는 UMBmark 로 다시 식별한다.
- 결론: 휠 오도메트리의 누적 오차는 (1) 거리에 따라 L^{3/2} 로 자라는 무작위 슬립 성분(공분산 모델과 일치: 직진·회전 NEES)
  과 (2) 회전량에 비례하는 체계 성분(유효 바퀴 간격, 보정 후 잔여 0.03 %)과 (3) 사각 주행에 남는 수 cm 체계 성분으로 나뉜다.
  남는 오차는 자이로·AMCL·정합 융합(EKF)이 맡는다 — 같은 런에서 `ekf_filter_node_odom` 최종 오차 0.06~0.9 cm.
