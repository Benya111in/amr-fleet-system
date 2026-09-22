# EKF 센서 퓨전 — 이중 필터, 예측/업데이트, 공분산 근거 (명세 4.3)

> 구현: robot_localization `ekf_node` 두 개 (`config/ekf.yaml`) + 입력 퍼블리셔 `wheel_odometry_node`,
> `imu_filter_node` (amr_localization, 공분산을 잡음 모델에서 계산) + AMCL. 런치:
> `ros2 launch amr_localization localization.launch.py`. 명세는 "robot_localization 또는 직접 구현" 을 허용하며,
> 이 문서는 필터가 푸는 식과 각 입력 공분산의 출처를 코드·설정과 1:1 로 대응시킨다.
> 측정 조건: load average 6~25 (외부 작업 종료 후 — 잠정치 아님), Gazebo 창 RTF 0.97~0.99 (§6).

## 1. 구조 — 이중 EKF (REP-105)

| 노드 | world_frame | 입력 | 출력 | TF |
| --- | --- | --- | --- | --- |
| `ekf_filter_node_odom` | `<r>/odom` | `wheel_odom` (vx, vy, vyaw), `imu/data` (vyaw, ax, ay) | `odometry/filtered` 50 Hz | `<r>/odom → <r>/base_footprint` |
| `ekf_filter_node_map` | `map` | 위 둘 + `amcl_pose` (x, y, yaw) | `odometry/filtered_map` 50 Hz | `map → <r>/odom` |

odom 필터는 연속 입력만 써서 점프하지 않는 odom 프레임(로컬 플래너·장애물 누적용)을, map 필터는 AMCL 절대
위치로 드리프트를 보정한 map 자세(전역 계획·플릿용)를 낸다. AMCL 은 `tf_broadcast: false`.
런치는 파일 하나에 프레임 접두어(`odom_frame`, `base_link_frame`)만 덮어쓰고, 두 노드의 서비스
(`set_pose`, `toggle`, `enable`) 이름 충돌을 피하려고 `<노드이름>/<서비스>` 로 리맵한다.

## 2. 예측 / 업데이트 단계

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
K = P⁻ Hᵀ S⁻¹
x⁺ = x⁻ + K y
P⁺ = (I − K H) P⁻ (I − K H)ᵀ + K R Kᵀ      (Joseph 형: 수치적으로 대칭·양정치 유지)
```

map 필터의 `smooth_lagged_data: true`, `history_length: 1.0` 은 스캔 처리 지연으로 늦게 도착하는 AMCL 측정을
측정 시각으로 되돌아가 적용한 뒤 이후 측정을 재적용한다 (0.5 m/s × 100 ms = 5 cm 의 지연 오차 제거).

## 3. 입력별 측정 공분산 R 의 근거

`R` 은 `ekf.yaml` 이 아니라 각 메시지의 공분산 필드에서 온다. 0 으로 두면 robot_localization 이 작은 값으로
대체해 그 센서를 과신하므로, 퍼블리셔가 잡음 모델에서 계산해 채운다.

| 입력 | 융합 변수 | R 출처 | 값 (예) |
| --- | --- | --- | --- |
| `wheel_odom.twist` | vx, vyaw | 엔코더 슬립 σ_s + 양자화 전파 (kinematics.md §3.2): `Var v = (σ_R²+σ_L²)/(4T²)`, `Var ω = (σ_R²+σ_L²)/(b²T²)` | 1 m/s: σ_v 7.1 mm/s, σ_ω 0.039 rad/s; 정지: 양자화만 σ_v 1.8 mm/s |
| `wheel_odom.twist` | vy | 비홀로노믹 구속 v_y ≡ 0 의 신뢰도 `σ_vy0 = 0.02 m/s` | 4e-4 m²/s² |
| `imu/data` | vyaw | `σ_g² + SE_g²` (sensors.yaml 0.0002 rad/s + 바이어스 추정 표준오차) | 4e-8 rad²/s² |
| `imu/data` | ax, ay | `σ_a² + SE_a²` (0.017 m/s²) | 2.9e-4 m²/s⁴ |
| `imu/data` | yaw | **융합 안 함**: `orientation_covariance[0] = −1`. Gazebo IMU orientation 은 노이즈 없는 GT 라 융합하면 정확도가 실제보다 좋게 나온다 (명세 8장 예시와 다른 결정 — 헤딩은 vyaw 적분 + AMCL yaw) | — |
| `amcl_pose` | x, y, yaw | AMCL 파티클 분포의 표본 공분산 (매 갱신). 수렴 시 수 cm², 납치·발산 시 m² 단위로 커져 EKF 가 자동으로 덜 믿는다 | 추적 중 σ_xy 2~5 cm |

설계 포인트
- **바퀴 위치는 융합하지 않는다**: 슬립이 누적된 적분값이라 정보가 속도와 중복되고 공분산이 무한히 커진다.
- **자이로 vyaw 가 헤딩의 주 정보원**: σ_g = 2e-4 ≪ 휠 σ_ω ≈ 0.04 → K 가 자이로 쪽으로 기울어 바퀴 비대칭 슬립에 강하다.
  바이어스(0.01 rad/s = 34°/min)는 `imu_filter_node` 가 기동 정지 평균으로 제거한다 (보정 전 그대로 넣으면
  10 m 직진에 헤딩 1°, 횡오차 9 cm 편향).
- **공분산 과신 방지**: IMU 는 LPF 후 분산이 아닌 원시 분산을 넣는다 (kinematics.md §5).

## 4. 프로세스 노이즈 Q, 초기 공분산 P0

`config/ekf.yaml` (15×15 전체 행렬 — robot_localization 3.5.x 는 길이를 검사하지 않아 15 개만 주면 쓰레기값을
읽는다). robot_localization 기본값에서 출발: 위치 x, y 0.05, yaw 0.06, vx/vy 0.025, vyaw 0.02, ax/ay 0.01.
map 필터의 x, y Q 는 AMCL 을 따라가는 속도를 정한다: 50 Hz 에서 P 가 AMCL 갱신(10 Hz) 사이에 Q·Δt 만큼 자라
AMCL R(σ 2~5 cm) 과 비슷해지므로 K ≈ 0.2~0.5 — 정지 시 map EKF 와 amcl_pose 의 차이는 0.1 cm 이하로 수렴한다
(Gazebo 진단 기록, §6.2). 납치 복구처럼 AMCL 이 수 m 뛰는 경우는 `kidnap_monitor_node` 가
`ekf_filter_node_map/set_pose` 로 즉시 옮긴다.
P0 는 1e-9 (odom 프레임 원점 정의상 초기 불확실성 0).

## 5. 확장점 (research brief `state-estimation` §2.2, §4.A)

- 팀 자체 6-상태 EKF `[x, y, θ, v, ω, b_g]` (자이로 바이어스 상태 포함) 를 같은 입력 계약(`wheel_odom`,
  `imu/data` 공분산)으로 붙일 수 있다 — 입력 공분산이 메시지에 담겨 있으므로 필터 구현만 교체하면 된다.
- IAG-EKF(슬립 χ² 게이팅 + 적응 R): `wheel_odom` 트위스트 공분산 팽창으로 구현 가능한 지점이
  `WheelOdometry` 의 `TwistCovarianceInput` (확장 인자 `lateral_skid_coeff`, `slip_distance_coeff`).

## 6. 측정 결과

조건: 2026-09-22 KST 11~12 시, load average 6~25 (외부 작업 종료 후 — 잠정치 아님), Gazebo 창 RTF 0.97~0.99.
자세한 경로·설정은 kinematics.md §7, slam.md §6.

### 6.1 입력 퍼블리셔와 주기

| 토픽 | 실측 주기 (벽시계, RTF ≈ 1) | 공분산 |
| --- | --- | --- |
| `wheel_odom` | 50.01 Hz (σ 0.04 ms) | 트위스트: 정지 `Var v` 3.3e-6 (양자화), `Var v_y` 4e-4, `Var ω` 1.0e-4; 자세: 야코비안 전파 |
| `imu/data` | 100.0 Hz (σ 0.03 ms) | `σ_g² + SE²`, `σ_a² + SE²`, `orientation_covariance[0] = −1` |
| `odometry/filtered_map` | 49.7~49.9 Hz | robot_localization ("Failed to meet update rate" 경고: 반복 정확도 세션 0 회, 납치 세션 3 회) |

기동 바이어스 추정 (Gazebo IMU: `sensors.yaml` |b| ~ N(0.01 rad/s, σ), N(0.1 m/s², σ), 축별 부호 무작위; 5 s 정지
평균 501 샘플, `bias_estimation_time:=5.0`): 두 실행에서 gyro (0.00999, 0.00998, −0.00999) /
(0.01000, −0.01000, 0.01001) rad/s (SE 9e-6), accel (0.100, 0.099, −0.101) / (0.102, −0.101, −0.100) m/s²
(SE 7.5e-4) — 부호·크기 모두 설정과 일치한다. 추정 후 자이로 잔여 바이어스 ≤ 1e-5 rad/s 는 60 s 에 0.03° 드리프트.

### 6.2 odom 필터 — 휠 + 자이로 융합의 효과 (드리프트 실험, `odometry/filtered` 최종 오차)

| 시나리오 | wheel_odom 단독 (보정 전 → 후) | `ekf_filter_node_odom` (보정 전 → 후) |
| --- | --- | --- |
| 직진 10 m | 6.7 → 2.7 cm | 0.45 → 0.72 cm |
| 제자리 2π | 헤딩 2.03° → 0.16° | 0.04 → 0.02 cm |
| 5 × 4 m 반시계 (18 m) | 13.9 → 4.7 cm | 0.97 → 0.24 cm |
| 5 × 4 m 시계 (18 m) | 14.7 → 6.4 cm | 1.45 → 0.28 cm |

자이로(σ_g = 2e-4 rad/s)가 헤딩을 잡으므로 odom 필터는 바퀴 간격 보정 전에도 체계 회전 오차에 거의 영향받지 않는다
(§3 "자이로 vyaw 가 헤딩의 주 정보원"). 합성 체인(kinematics.md §7.2): RMSE 1.07 cm, 최대 3.4 cm, 헤딩 0.15°.

### 6.3 map 필터 — AMCL 융합 정확도 (GT 대비, `amr_evaluation` 판정)

최종 설정 4 바퀴 합산 (slam.md §6.3): **정지 2.99 cm, 직선 2.30 cm, 회전 1.52 cm (RMSE)**, 헤딩 RMSE ≤ 0.10°,
최대 오차 5.3 cm. 명세 3 / 5 / 8 cm 에 대해 직선·회전은 충분한 여유, 정지는 경계 (바퀴별 2.38~3.79 cm; 0.8 m/s
통로 주행 직후 정지가 지배, 그 외 정지는 0.9~1.0 cm). 진단 기록에서 정지 시 map EKF 와 `amcl_pose` 의 차이는
0.1 cm 이하로, 정지 오차는 EKF 가 아니라 AMCL 추정 자체에서 온다 (slam.md §6.3 분석).
