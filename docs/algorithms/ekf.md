# EKF 센서 퓨전 — 이중 필터, 예측/업데이트, 공분산 근거 (명세 4.3)

> 필터: robot_localization `ekf_node` 두 개 — **설정** `config/ekf.yaml`. 명세는 "robot_localization 또는 직접 구현" 을
> 허용하고 이 프로젝트는 전자를 택했다. **프로젝트 코드**는 필터 입력 쪽이다: 각 입력 메시지의 공분산을 잡음 모델에서
> 계산해 채우는 퍼블리셔 (`wheel_odometry_node`, `imu_filter_node`, `scan_matcher_node`) 와 AMCL·납치 감시 설정.
> 런치: `ros2 launch amr_localization localization.launch.py`. 측정은 §6 (조건은 표마다).

## 1. 구조 — 무엇이 설정이고 무엇이 우리 코드인가

| 노드 | world_frame | 입력 | 출력 | TF |
| --- | --- | --- | --- | --- |
| `ekf_filter_node_odom` | `<r>/odom` | `wheel_odom` (vx, vy, vyaw), `imu/data` (vyaw, ax, ay) | `odometry/filtered` 50 Hz | `<r>/odom → <r>/base_footprint` |
| `ekf_filter_node_map` | `map` | 위 둘 + `amcl_pose` (x, y, yaw) + `scan_match_pose` (x, y, yaw) | `odometry/filtered_map` 50 Hz | `map → <r>/odom` |

odom 필터는 연속 입력만 써서 점프하지 않는 odom 프레임(로컬 플래너·장애물 누적용)을, map 필터는 AMCL·스캔 정합의
절대 자세로 드리프트를 보정한 map 자세(전역 계획·플릿용)를 낸다 (REP-105). AMCL 은 `tf_broadcast: false`.

| 부분 | 종류 | 위치 | 시험 |
| --- | --- | --- | --- |
| 예측·업데이트·Joseph 공분산 갱신·지연 측정 재적용 (§2) | robot_localization 3.5.x (**설정만**) | `config/ekf.yaml` | Gazebo 정확도 (slam.md §6.3) |
| 프로세스 노이즈 Q, 초기 P0 (§4) | 설정 | `config/ekf.yaml` | 같음 |
| 휠 트위스트 공분산 R_wheel (§3) | **우리 코드** | `wheel_odometry` / `odometry_covariance` (C++) | `test_odometry_covariance`, `test_wheel_odometry` (몬테카를로·NEES) |
| IMU 공분산 R_imu, 바이어스 보정 (§3) | **우리 코드** | `imu_filter` / `imu_bias_estimator` (C++) | `test_imu_filter` |
| 스캔-지도 정합 자세 + 헤시안 공분산 R_scan (§3) | **우리 코드** | `scan_registration` (Python) | `test_scan_registration` (수렴·헤시안 공분산 몬테카를로·통로 퇴화) |
| AMCL 자세 공분산 R_amcl (§3) | nav2_amcl (파티클 표본 공분산) + 설정 | `config/amcl.yaml` | slam.md §6.3 |
| 납치 복구 시 필터 재설정 (`set_pose`) | **우리 코드** | `kidnap_monitor_node` | `test_nodes`, slam.md §6.4 |

**research brief 와의 차이 (명시)**: brief `state-estimation` 베이스라인은 팀 자체 6-상태 C++ EKF `[x, y, θ, v, ω, b_g]`
를 주 결과물로, robot_localization 을 비교 대상으로 두었다. 이 저장소는 그 필터를 만들지 않았고 robot_localization 을
필터로 쓴다 — 명세 4.3 이 허용하는 선택이고, 명세 9장 "핵심 알고리즘의 독립 모듈·시험" 요구는 필터 입력 쪽(오도메트리
공분산 전파, IMU 필터, 스캔 정합)을 ROS 비의존 모듈로 구현·시험하는 것으로 채운다. 자체 EKF 는 §5 확장점이다.

런치는 파일 하나에 프레임 접두어(`odom_frame`, `base_link_frame`)만 덮어쓰고, 두 노드의 서비스 (`set_pose`, `toggle`,
`enable`) 이름 충돌을 피하려고 `<노드이름>/<서비스>` 로 리맵한다.

## 2. 예측 / 업데이트 단계 (robot_localization 이 푸는 식)

상태 (15): `x = [x y z | φ θ ψ | vx vy vz | ωx ωy ωz | ax ay az]` (위치·자세는 world_frame, 속도·가속도는
base_footprint). `two_d_mode: true` 가 z, roll, pitch 와 그 미분을 0 으로 고정하므로 실제로는 평면 8 상태
`[x, y, ψ, vx, vy, ωz, ax, ay]` 가 움직인다.

**예측** (등가속 운동 모델, 주기 Δt = 1/50 s):

```
ψ_{k+1} = ψ + ωz Δt
x_{k+1} = x + (vx cos ψ − vy sin ψ) Δt + ½(ax cos ψ − ay sin ψ) Δt²
y_{k+1} = y + (vx sin ψ + vy cos ψ) Δt + ½(ax sin ψ + ay cos ψ) Δt²
v_{k+1} = v + a Δt,   ω, a: 상수 (랜덤워크)
P⁻ = F P Fᵀ + Q·Δt              (F = ∂f/∂x, Q = process_noise_covariance)
```

**업데이트** (센서 메시지마다, 그 센서가 `*_config` 에서 true 인 변수만 H 로 선택):

```
y = z − H x⁻                   (각도 성분은 정규화)
S = H P⁻ Hᵀ + R                (R = 메시지의 covariance 필드 — 아래 §3)
거부: yᵀ S⁻¹ y ≥ n²  이면 버림  (n = *_rejection_threshold; scan_match_pose 만 5, 나머지는 끔)
K = P⁻ Hᵀ S⁻¹
x⁺ = x⁻ + K y
P⁺ = (I − K H) P⁻ (I − K H)ᵀ + K R Kᵀ      (Joseph 형: 수치적으로 대칭·양정치 유지)
```

map 필터의 `smooth_lagged_data: true`, `history_length: 1.0` 은 스캔 처리 지연으로 늦게 도착하는 AMCL·스캔 정합 측정을
측정 시각으로 되돌아가 적용한 뒤 이후 측정을 재적용한다 (0.5 m/s × 100 ms = 5 cm 의 지연 오차 제거).

## 3. 입력별 측정 공분산 R 의 근거

`R` 은 `ekf.yaml` 이 아니라 각 메시지의 공분산 필드에서 온다. 0 으로 두면 robot_localization 이 작은 값으로
대체해 그 센서를 과신하므로, 퍼블리셔가 잡음 모델에서 계산해 채운다.

| 입력 | 융합 변수 | R 출처 (식) | 값 (예) |
| --- | --- | --- | --- |
| `wheel_odom.twist` | vx, vyaw | 발행 구간 T 바퀴 변위 분산 `σ_i²(T) = σ_s² ℓ_ref Σ|Δs_i| + k|ΣΔs_i| + δ²/6` (거리당 슬립 + 양자화, kinematics.md §3) 을 `Var v = (σ_R²+σ_L²)/(4T²)`, `Var ω = (σ_R²+σ_L²)/(b²T²)`, `Cov = (σ_R²−σ_L²)/(2bT²)` 로 전파 | 1 m/s: σ_v 5.3 mm/s, σ_ω 0.029 rad/s; 정지: 양자화만 σ_v 1.8 mm/s, σ_ω 0.010 rad/s |
| `wheel_odom.twist` | vy | 비홀로노믹 구속 v_y ≡ 0 의 신뢰도 `σ_vy0 = 0.02 m/s` (+ 선회 횡미끄럼 항, 기본 끔) | 4e-4 m²/s² |
| `wheel_odom.twist` (분기 모호·불가능 점프 구간) | vx, vyaw | 그 구간 변위 분산에 한 바퀴 둘레² (2πr)² 가산 → EKF 가 사실상 무시 | Var v ≥ 0.27/(4T²) |
| `imu/data` | vyaw | `σ_g² + SE_g²` (sensors.yaml 0.0002 rad/s + 기동 바이어스 추정 표준오차 2.6e-6) | 4.0e-8 rad²/s² |
| `imu/data` | ax, ay | `σ_a² + SE_a²` (0.017 m/s² + 2.2e-4) | 2.9e-4 m²/s⁴ |
| `imu/data` | yaw | **융합 안 함**: `orientation_covariance[0] = −1`. Gazebo IMU orientation 은 노이즈 없는 GT 라 융합하면 정확도가 실제보다 좋게 나온다 (명세 8장 예시와 다른 결정 — 헤딩은 vyaw 적분 + AMCL/정합 yaw) | — |
| `amcl_pose` | x, y, yaw | AMCL 파티클 분포의 표본 공분산 (매 갱신). 수렴 시 작고, 납치·발산 시 m² 단위로 커져 EKF 가 자동으로 덜 믿는다 | §6.2 실측 |
| `scan_match_pose` | x, y, yaw | 정합 헤시안 역 × 상관 팽창: `Σ = κ s² (JᵀWJ)⁻¹`, `s² = Σ w r²/(Σw − 3)`, κ = 10, 하한 σ_xy 5 mm, σ_ψ 0.002 rad (slam.md §4.3). 통로처럼 한 방향만 구속되면 그 방향 분산만 커진다 | §6.2 실측 |

설계 포인트
- **바퀴 위치는 융합하지 않는다**: 슬립이 누적된 적분값이라 정보가 속도와 중복되고 공분산이 무한히 커진다.
- **자이로 vyaw 가 헤딩의 주 정보원**: σ_g = 2e-4 ≪ 휠 σ_ω ≈ 0.01~0.03 → K 가 자이로 쪽으로 기울어 바퀴 비대칭 슬립에 강하다.
  바이어스(0.01 rad/s = 34°/min)는 `imu_filter_node` 가 기동 60 s 정지 평균으로 제거한다.
- **공분산 과신 방지**: IMU 는 LPF 후 분산이 아닌 원시 분산을 넣는다 (kinematics.md §5). 스캔 정합은 빔 잔차가 지도 격자를
  공유해 독립이 아니므로 헤시안 역에 κ 를 곱한다 (합성 독립 잡음에서는 κ = 1 이 맞음을 `test_scan_registration` 이 확인).
- **두 절대 측정(AMCL, 정합)의 상관**: 둘 다 같은 스캔·같은 지도에서 나오므로 오차가 상관돼 있고 EKF 는 독립으로 본다 →
  map 필터 공분산은 낙관적이다. 정확도 판정은 공분산이 아니라 GT 대비 오차로 한다 (slam.md §6.3).

## 4. 프로세스 노이즈 Q, 초기 공분산 P0 (`config/ekf.yaml`)

15×15 전체 행렬 (robot_localization 3.5.x 는 길이를 검사하지 않아 15 개만 주면 쓰레기값을 읽는다). 값은
robot_localization 기본값이고, 아래가 이 입력 구성에서 그 값이 맞는 이유다 (Q 는 연속시간 밀도, 한 주기 증가 = Q·Δt, Δt 0.02 s).

| 상태 | Q | 근거 |
| --- | --- | --- |
| x, y (odom 필터) | 0.05 m²/s | 위치를 직접 재는 입력이 없으므로 추정치는 속도 적분으로만 정해지고, Q 는 보고 공분산의 증가율만 정한다 (1 s 에 σ 0.22 m — 실제 odom 드리프트 σ ≈ 7 cm/10 m (kinematics.md §4.1) 보다 크게 잡아 odom 공분산을 소비자가 과신하지 않게) |
| x, y (map 필터) | 0.05 m²/s | 한 주기 1e-3 m² ≫ 정합 R (5~10 mm)² 와 AMCL R (2~5 cm)²: 절대 측정이 오면 칼만 이득 ≈ 0.3~0.97 로 곧바로 따라간다 (지도 기준 정확도가 목표이므로 odom 예측보다 절대 측정을 우선). 측정 사이 50 Hz 예측은 속도 적분이 맡는다 |
| ψ | 0.06 rad²/s | x, y 와 같은 논리 (odom 필터는 자이로 적분, map 필터는 절대 yaw 가 보정) |
| vx, vy | 0.025 m²/s³ | 한 주기 5e-4 ≫ 휠 R_v (1.8~5.3 mm/s)² → K_v ≈ 0.95: 속도는 지연 없이 휠 측정을 따른다 (가속 1 m/s² 의 한 주기 변화 2 cm/s 도 수용) |
| vyaw | 0.02 rad²/s³ | 한 주기 4e-4 ≫ 자이로 R 4e-8 → 각속도는 자이로가 정한다 |
| ax, ay | 0.01 m²/s⁵ | 가속도는 IMU 가 정하고 (R 2.9e-4), 저크 2 m/s³ (robot_params) 를 수용 |
| z, roll, pitch 등 | 기본값 | two_d_mode 에서 0 고정 (의미 없음) |
| P0 | 1e-9 (전 상태) | odom 프레임은 기동 자세를 원점으로 정의하므로 초기 불확실성 0. map 필터도 1e-9 에서 시작하지만 Q 가 주기마다 1e-3 m² 씩 더해져 첫 AMCL·정합 측정(초기 자세 = 스폰 자세)을 곧바로 받아들인다 |

납치 복구처럼 AMCL 이 수 m 뛰는 경우는 `kidnap_monitor_node` 가 `ekf_filter_node_map/set_pose` 로 즉시 옮긴다
(AMCL 입력에는 거부 문턱이 없어 큰 점프도 결국 받아들이지만 수 초 걸린다; 정합은 위치 상실 동안 멈춘다).

## 5. 확장점 (research brief `state-estimation` §2.2, §4.A)

- 팀 자체 6-상태 EKF `[x, y, θ, v, ω, b_g]` (자이로 바이어스 상태 포함) 를 같은 입력 계약(`wheel_odom`, `imu/data`,
  `amcl_pose`, `scan_match_pose` 공분산)으로 붙일 수 있다 — 입력 공분산이 메시지에 담겨 있으므로 필터 구현만 교체하면 된다.
- IAG-EKF(슬립 χ² 게이팅 + 적응 R): `wheel_odom` 트위스트 공분산 팽창으로 구현 가능한 지점이
  `WheelOdometry` 의 `TwistCovarianceInput` (확장 인자 `lateral_skid_coeff`, `slip_distance_coeff`).

## 6. 측정 결과

위치 추정 정확도의 측정 조건·표는 [slam.md §6.3](slam.md) (map EKF `odometry/filtered_map` 대 GT, 경로 3 개, 정지/직선/회전
분리). 요약: 정합 켬(기본) 정지 1.52 / 4.23, 직선 1.74 / 3.95, 회전 0.90 / 2.39 cm (RMSE / max) — RMSE 는 명세 안, 정지 최대는
서측 도크 앞 지도 국소 어긋남(≈ 3 cm) 때문에 넘는다. 정합 끔(AMCL 만): 1.83 / 3.24, 1.92 / 3.94, 1.17 / 1.96 cm.

- **입력 기여**: 정합(pose1)을 끄면 지도가 정확한 구역의 정지 오차가 0.76–1.07 → 0.91–1.84 cm 로 커진다 — AMCL 파티클 평균의
  cm 단위 흔들림이 정지 중 굳는 것이 AMCL 만의 바닥이다 (slam.md §4.3). 헤딩 RMSE 는 0.09–0.11° → 0.04–0.06°.
