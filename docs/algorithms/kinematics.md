# 차동 구동 기구학 · 휠 오도메트리 · 센서 전처리 (명세 4.2)

> 구현: `src/amr_localization` — `diff_drive_kinematics`, `encoder_model`, `odometry_covariance`,
> `wheel_odometry` (C++ 라이브러리, ROS 비의존) + `wheel_odometry_node` / `imu_filter_node` /
> `scan_filter_node` (얇은 래퍼). 파라미터: `src/amr_localization/config/*.yaml`.
> 설계 기준: research brief `kinematics-odometry` §2~3 (베이스라인). 측정 조건은 §7 머리
> (load average 6~25, 외부 작업 종료 후 — 잠정치 아님).

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

## 2. 휠 엔코더 모델 (4096 틱 + 슬립 잡음)

Gazebo Fortress 의 `JointStatePublisher` 는 잡음 없는 누적 조인트 각을 물리 스텝마다(최대 1 kHz) 준다.
`WheelEncoderModel` 이 실제 엔코더처럼 만든다 (`config/sensors.yaml wheel_encoder`):

1. **샘플링**: `update_rate` 50 Hz 로 솎는다. 사이 입력은 건너뛰지만 누적 각 기반이라 이동량은 잃지 않는다.
   슬립 잡음이 "주기당" 정의이므로 입력 주기와 무관하게 잡음 통계가 같아진다.
2. **양자화**: 누적 각을 틱으로 `n = round(φ N / 2π)` (int64, 4 h 연속 운용에도 오버플로 없음).
   1 틱 = `2π/4096` rad = `2πr/4096 = 0.127 mm` (r = 0.0825).
3. **슬립 잡음**: 주기 변위에 `(1 + ε)`, `ε ~ N(0, σ_s²)`, `σ_s = 0.01` 을 곱한다 (바퀴마다 독립).
4. ±π 로 감겨 오는 조인트 각은 주기 차가 π 를 넘으면 2π 감김으로 보고 푼다.

검증 (`test_encoder_model`): 양자화 계단 = 정확히 1 틱, 누적 각 양자화 오차 |q| ≤ δ/2,
주기 차 오차는 MA(1) (`Var = δ²/6`, lag-1 공분산 `−δ²/12`), 슬립 잡음 표본 σ 가 10 000 표본에서
설정값의 ±10 % 이내 (seed 고정), 같은 seed 재현성, 감김 해제.

## 3. 공분산 (EKF 입력 계약, 명세 4.3 "공분산 근거")

`config/ekf.yaml` 은 `wheel_odom` 의 **트위스트**(vx, vy, vyaw)만 융합하고 공분산은 메시지에서 읽는다.
따라서 퍼블리셔가 잡음 모델에서 공분산을 계산해 채운다.

### 3.1 바퀴 변위 분산

```
주기 (자세 전파용): Var(Δs_i) = σ_s² Δs_i² + k |Δs_i|            (k: 물리 슬립 계수, 사전값 0)
발행 구간 T (트위스트용): σ_i²(T) = Σ σ_s² Δs_i² + k |Σ Δs_i| + δ²/6
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

수치 (직진 1 m/s, T = 0.02 s, σ_s = 0.01): `σ_v = 7.1e-3 m/s`, `σ_ω = 0.039 rad/s` — `config/ekf.yaml`
주석의 상수 자리표시(σ² = 1e-4)는 v ≈ 1.4 m/s 에 해당하고, 실제 값은 속도에 비례해 달라진다.

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

### 4.1 폐형 (Kelly 2004; Chong & Kleeman 1997, 곱셈 슬립 잡음 `k = σ_s² |Δs_step|`)

```
직진 L:      Var θ = 2kL/b²,   Var x ≈ kL/2,   Var y ≈ 2kL³/(3b²)
제자리 회전 Θ: Var θ = k|Θ|/b   (k = σ_s² · ωbΔt/2)
```

횡오차가 L^{3/2} 로 자라는 것은 헤딩 오차(랜덤워크)가 이동 거리만큼 증폭되기 때문이다.
0.5 m/s, 50 Hz(`Δs = 0.01 m`, `k = 1e-6 m`), L = 10 m: `σ_θ = 0.012 rad (0.70°)`, `σ_y = 0.072 m`,
`σ_x = 2.2 mm`. 제자리 2π @0.5 rad/s: `σ_θ = 0.0024 rad (0.14°)`.
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
4. 섀도우(베일): 인접 끝점 선분과 광선의 각 `β = atan2(r_j sin Δα, r_i − r_j cos Δα)` 가 10° 미만/170° 초과면
   먼 쪽 점 제거 (모서리 혼합 픽셀).

검증 (`test_scan_filter`): REP-117 값, 각도 창·마스크·감긴 섹터, 고립 아웃라이어 제거 + 벽 유지,
작은 물체(이웃 지지) 보존, 360° 감김, 섀도우 제거, 빈 스캔, **메타데이터(angle_min/increment/빔 수) 보존**.

## 7. 측정 결과

측정 조건: 2026-09-22 10:50~12:30 KST, 호스트 32 스레드, `uptime` load average 6~25 (외부 `gsim` 작업
종료 후 — 이전 실행의 40~110 과 달리 **잠정치 아님**), Gazebo Fortress headless + GPU 렌더링, 창 RTF
0.97~1.00. 카메라 갱신율만 1 Hz 로 낮춘 설정 사본(`cfg_fast`, 위치 추정은 카메라 미사용)을 썼다.

### 7.1 합성 입력 — 해석해 대비 (Gazebo 없이, `wheel_odometry_node` 실행 파일)

합성 로봇이 해석 궤적의 누적 바퀴 각을 200 Hz `joint_states` 로 발행 (노드가 50 Hz 로 솎음), 마지막
`wheel_odom` 을 그 스탬프의 해석해와 비교. 양자화 한계: 위치 δ/2 = 0.063 mm, 헤딩 2δ/b = 7.0e-4 rad
(0.040°). load average 11.7 → 7.8.

| 경로 | 설정 | 위치 오차 | 헤딩 오차 | 판정 |
| --- | --- | --- | --- | --- |
| 직진 5 m (0.5 m/s) | 양자화만, 원호 적분 | 1.1 µm | 0 | 양자화 한계 안 |
| 90° 원호 (r = 1 m) | 양자화만, 원호 적분 | 0.014 mm | 0.0073° | 양자화 한계 안 |
| 90° 원호 | 양자화만, **오일러** | **7.07 mm** | 0.0073° | 오일러의 O(Δt) 곡선 편향 (원호 대비 500 배) |
| 직진 5 m | 슬립 σ_s 0.01 (seed 3) | 4.9 cm (횡) | 1.24° | 발행 σ_y 2.5 cm, σ_θ 0.50° — 1 회 표본 (2σ) |
| 90° 원호 | 슬립 σ_s 0.01 (seed 3) | 2.3 mm | 0.36° | 발행 σ 3.4/2.5 mm, 0.29° |

발행 주기 50.0 Hz, 정지 시 트위스트 분산 `Var v = 3.3e-6` (양자화 σ 1.8 mm/s), `Var v_y = 4e-4`,
`Var ω = 1.0e-4`.

### 7.2 체인 — wheel_odom + imu_filter + ekf_filter_node_odom (`config/ekf.yaml`, 합성 입력)

`localization.launch.py mode:=odom` (robot_name amr_00, 접두어 `amr_00/`), 합성 IMU: 바이어스 gyro
(0.004, −0.006, 0.01) rad/s, accel (0.08, −0.06, 0.1) m/s² + sensors.yaml 백색 잡음. 경로: 정지 3 s →
직진 4 s → 원호 π s → 직진 4 s → 제자리 π/2 → 정지 2 s (총 ≈ 7.6 m, 19.3 s).

| 항목 | 값 |
| --- | --- |
| 기동 바이어스 추정 (2.5 s, 251 샘플) | gyro (0.004006, −0.006025, 0.009998) rad/s, SE 1.2e-5 — 참값 대비 ≤ 6e-6; accel (0.077, −0.059, 0.099) m/s², SE 1.1e-3 |
| `wheel_odom` | 964 샘플, 50.0 Hz |
| `odometry/filtered` | 978 샘플 모두 유한 (NaN 없음) |
| EKF 위치 오차 vs 해석해 | RMSE **1.07 cm**, 최대 3.4 cm, 최종 3.4 cm |
| EKF 헤딩 오차 | RMSE 0.15°, 최종 0.23° |
| wheel_odom 단독 최종 오차 | 2.4 cm, 헤딩 0.59° (발행 σ 2.2/1.2 cm, 0.54°) |

### 7.3 Gazebo 드리프트 실험 (명세 4.2 "드리프트 측정 실험", 보정 전 `wheel_separation_multiplier` 1.0)

`localization.launch.py mode:=odom` + `odom_drift_experiment` (repetitions 3, 0.5 m/s · 0.5 rad/s,
가속 0.5 m/s² · 1 rad/s²), 스폰 (−16, −2, 0). 시나리오당 3 회, 12 런 모두 1 139~2 772 샘플 (50 Hz).
load average 10.0 → 18.1, 창 RTF 0.99. 원자료: `logs/eval/odom_drift/*.csv`, `summary.md`.

| 시나리오 | 주행 | 최종 위치 오차 | 최종 헤딩 오차 | 오차 증가율 | 발행 σ_pos | NEES | EKF(odom) 최종 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 직진 10 m | 10.00 m | 6.7 ± 2.1 cm | 0.62 ± 0.60° | 0.71 %/m | 7.1 cm | **2.8 ± 1.7** | 0.45 cm |
| 제자리 2π | 2π rad | 0.03 cm | **2.03 ± 0.10°** (체계) | 2.03 °/rev | 0.03 cm | 406 | 0.04 cm |
| 5 × 4 m 반시계 | 18.0 m | 13.9 ± 1.1 cm | 2.19 ± 0.03° | 0.94 %/m | 6.7 cm | 6.8 | 0.97 cm |
| 5 × 4 m 시계 | 18.0 m | 14.7 ± 5.6 cm | 2.33 ± 1.48° | 0.93 %/m | 6.8 cm | 11.9 | 1.45 cm |

- **UMBmark**: c_cw = (6.2, 10.5) cm, c_ccw = (6.6, −12.2) cm, E_max,syst = **13.9 cm**.
- **유효 파라미터** (`drift_analysis.effective_parameters`): 거리 배율 L_odom/L_gt = **1.00004** (반지름 오차
  없음), 회전 배율 Θ_odom/Θ_gt = **1.00565** (3 회 1.00550 / 1.00597 / 1.00548) → b_eff = 0.36202 m.
- 해석: 직진은 NEES ≈ 3 (χ²₃ 기대값) 으로 **슬립 잡음 공분산 모델이 일관적**이다 — 오차가 발행 σ 와 같은
  크기로 L^{3/2} 형태(횡)로 자란다. 제자리 회전은 매번 같은 부호로 0.57 % 더 돌았다고 본다(1 회전 2.0°):
  무작위 모델이 설명하지 못하는 **체계 오차**(NEES 406)다. Gazebo 접지 패치·캐스터 마찰로 실제 회전 중심
  간격이 기하값 0.36 m 보다 크게 작동한다. 사각 주행은 90° 회전 4 번에 이 체계 오차(≈ 2°)가 실려 복귀 오차
  14 cm 를 만든다. 반면 자이로를 융합한 `ekf_filter_node_odom` 은 같은 런에서 0.4~1.5 cm 로, 헤딩을 자이로가
  잡기 때문에 체계 회전 오차의 영향을 거의 받지 않는다.
- 조치: 회전 배율로 `wheel_separation_multiplier = 1.00565` (`config/wheel_odometry.yaml`) 를 넣고 같은
  실험을 다시 돌렸다 (§7.4).

### 7.4 보정 후 드리프트 (`wheel_separation_multiplier` 1.00565 → b = 0.36203 m)

같은 실험·같은 시작 자세, load average 5.2 → 17.3, 창 RTF 0.99. 리포트의 기준 b 도 보정값(0.362034)으로 준다.

| 시나리오 | 최종 위치 오차 | 최종 헤딩 오차 | NEES | EKF(odom) 최종 | 보정 전 (위치 / 헤딩 / NEES) |
| --- | --- | --- | --- | --- | --- |
| 직진 10 m | 2.7 ± 2.0 cm | 0.41 ± 0.18° | 1.1 ± 0.4 | 0.72 cm | 6.7 cm / 0.62° / 2.8 |
| 제자리 2π | 0.02 cm | **0.16 ± 0.11°** | **4.1 ± 3.6** | 0.02 cm | 0.03 cm / 2.03° / 406 |
| 5 × 4 m 반시계 | 4.7 ± 4.3 cm | 0.49 ± 0.29° | 3.6 ± 2.7 | 0.24 cm | 13.9 cm / 2.19° / 6.8 |
| 5 × 4 m 시계 | 6.4 ± 5.8 cm | 0.68 ± 0.06° | 3.7 ± 4.8 | 0.28 cm | 14.7 cm / 2.33° / 11.9 |

- UMBmark E_max,syst **13.9 cm → 4.0 cm** (c_cw = (−4.0, −0.3) cm, c_ccw = (1.3, −2.7) cm), 회전 배율 잔차
  Θ_odom/Θ_gt = 0.99968 (−0.03 %), 거리 배율 1.00005.
- 제자리 회전 NEES 가 406 → 4.1 로 χ²₃ 기대값 3 근처가 되었다: 남은 오차는 공분산 모델이 설명하는 무작위 슬립
  수준이다. 사각 주행 NEES 3.6~3.7 도 일관적이다. 직진은 보정 대상이 아니므로 차이(6.7 → 2.7 cm)는 3 회 표본
  변동이다 (둘 다 발행 σ 7 cm 안).
- 결론: 휠 오도메트리의 누적 오차는 (1) 거리에 따라 L^{3/2} 로 자라는 무작위 슬립 성분(공분산 모델과 일치)과
  (2) 회전량에 비례하는 체계 성분(유효 바퀴 간격)으로 나뉘고, (2) 는 드리프트 실험 → 유효 파라미터 식별 →
  `wheel_separation_multiplier` 로 제거된다. 남는 (1) 은 자이로·AMCL 융합(EKF)이 맡는다 — 같은 런에서
  `ekf_filter_node_odom` 최종 오차 0.2~0.7 cm.
