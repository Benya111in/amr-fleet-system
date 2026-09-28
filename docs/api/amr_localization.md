# API — `amr_localization`

> 명세 4장 2·3절 (오도메트리 / SLAM · 위치 추정). [← 색인](README.md)
> 알고리즘·튜닝 근거: [../algorithms/kinematics.md](../algorithms/kinematics.md),
> [../algorithms/ekf.md](../algorithms/ekf.md), [../algorithms/slam.md](../algorithms/slam.md).
> 런치: `src/amr_localization/launch/localization.launch.py` (`mode:=slam|localization|odom`).
> 모든 노드는 로봇 네임스페이스(`/amr_01`) 아래에서 뜬다 — 아래 토픽 이름은 전부 **상대 이름**이다
> (`localization.launch.py:127-130`). 예외는 `/map` 뿐이다.

## 0. 이 패키지의 노드

| 노드 | 언어 | 형태 | 소스 | 기동 조건 (`localization.launch.py`) |
| --- | --- | --- | --- | --- |
| `wheel_odometry_node` | C++ | own | `src/amr_localization/src/wheel_odometry_node.cpp` | 항상 (`:140-141`) |
| `imu_filter_node` | C++ | own | `src/amr_localization/src/imu_filter_node.cpp` | 항상 (`:142-143`) |
| `scan_filter_node` | C++ | own | `src/amr_localization/src/scan_filter_node.cpp` | 항상 (`:144-146`) |
| `amcl_map_adapter` | Python | own | `src/amr_localization/amr_localization/amcl_map_adapter.py` | `amcl_half_cell_fix:=true` (기본, `:195-198`) |
| `kidnap_monitor_node` | Python | own | `src/amr_localization/amr_localization/kidnap_monitor_node.py` | `use_kidnap_monitor:=true` (기본, `:207-215`) |
| `scan_matcher_node` | Python | own | `src/amr_localization/amr_localization/scan_matcher_node.py` | `use_scan_matcher:=true` (기본, `:216-223`) |
| `ekf_filter_node_odom` | C++ | ext (`robot_localization`) | — | 항상 (`:156`) |
| `ekf_filter_node_map` | C++ | ext | — | `mode:=localization` (`:178-179`) |
| `amcl` | C++ | ext (`nav2_amcl`) | — | `mode:=localization` (`:199-202`) |
| `slam_toolbox` | C++ | ext | — | `mode:=slam` (`:159-162`) |
| `map_server` / `map_saver_server` | C++ | ext (`nav2_map_server`) | — | `:184-186` / `:171-174` |
| `lifecycle_manager_*` | C++ | ext | — | `:175-177`, `:187-190`, `:203-205` |

---

## 1. `wheel_odometry_node`

- **패키지**: `amr_localization` · **언어**: C++ · **소스**: `src/amr_localization/src/wheel_odometry_node.cpp`
- **역할**: `joint_states` 의 바퀴 각도 → 인코더 양자화·슬립 잡음 모델 → 순기구학 적분 → `wheel_odom` (공분산 포함).
- **설정 파일**: `src/amr_localization/config/wheel_odometry.yaml` (런치가 `config/robot_params.yaml`,
  `config/sensors.yaml` 값으로 일부를 덮어쓴다 — `wheel_odometry.yaml:4-5`).

### 발행 토픽

| 토픽 | 타입 | QoS | 주기 | 근거 | 비고 |
| --- | --- | --- | --- | --- | --- |
| `wheel_odom` | `nav_msgs/Odometry` | 기본 (depth 10) | `publish_rate` = 50 Hz | `:106` | odom EKF 의 `odom0` 입력 |
| `wheel_odom/ticks` | `sensor_msgs/JointState` | 기본 (depth 10) | `publish_rate` | `:108-110` | `publish_ticks:=true` (기본) 일 때만 |
| `/tf` (`odom → base_footprint`) | `tf2_msgs/TFMessage` | — | `publish_rate` | `:111-113` | `publish_tf:=true` 일 때만. **기본은 false** — 운용에서는 EKF 가 이 변환을 낸다 |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `joint_states` | `sensor_msgs/JointState` | `sensor` (BEST_EFFORT) | `:115-118` |

### 서비스 (서버)

| 서비스 | 타입 | 근거 | 설명 |
| --- | --- | --- | --- |
| `wheel_odom/reset` | `std_srvs/srv/Trigger` | `:119-120` | 적분 자세를 원점으로 되돌린다 |

### 액션

없다.

### 파라미터

기본값은 모두 `declare_parameter` 선언부에서 뽑았다 (파일 `wheel_odometry_node.cpp`).

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `wheel_radius` | double | `0.0825` | 62 | [m] 공칭 바퀴 반지름 |
| `left_wheel_radius` | double | `0.0` | 63 | [m] > 0 이면 좌 바퀴만 덮어씀 (0 = `wheel_radius`) |
| `right_wheel_radius` | double | `0.0` | 64 | [m] 위와 같음 (우) |
| `wheel_separation` | double | `0.36` | 69 | [m] 바퀴 중심 간 거리 |
| `wheel_separation_multiplier` | double | `1.0` | 70 | 유효 간격 배율 (config yaml 은 `1.00565`) |
| `ticks_per_revolution` | int | `4096` | 72 | [tick/rev] 엔코더 분해능 |
| `slip_noise_stddev` | double | `0.01` | 73 | 기준 굴림 거리에서의 상대 슬립 σ |
| `enable_quantization` | bool | `true` | 74 | 인코더 양자화 모델 사용 |
| `enable_slip_noise` | bool | `true` | 75 | 슬립 잡음 모델 사용 |
| `slip_reference_distance` | double | `0.01` | 79 | [m] `slip_noise_stddev` 를 정의한 굴림 거리 |
| `joint_position_wrapped` | bool | `false` | 80 | 입력 조인트 각이 (−π, π] 로 감기는지 |
| `max_wheel_speed` | double | `30.0` | 81 | [rad/s] 분기 판정·점프 검출 상한 |
| `slip_distance_coeff` | double | `0.0` | 82 | 거리 비례 슬립 계수 |
| `wheel_param_rel_stddev` | double | `0.0` | 83 | 바퀴 기하 파라미터 상대 불확실도 |
| `lateral_velocity_stddev` | double | `0.02` | 84 | [m/s] 횡방향 속도 잡음 σ |
| `lateral_skid_coeff` | double | `0.0` | 85 | 횡슬립 계수 |
| `publish_rate` | double | `50.0` | 86 | [Hz] 인코더 샘플 = 발행 주기 |
| `integration` | string | `"exact_arc"` | 89 | 적분 방식 (원호 정적분) |
| `noise_seed` | int | `0` | 90 | 0 = 매 실행 무작위 |
| `unfused_variance` | double | `1e-3` | 92 | 융합하지 않는 상태의 분산 |
| `left_wheel_joint` | string | `"left_wheel_joint"` | 94 | `joint_states` 의 좌 바퀴 조인트 이름 |
| `right_wheel_joint` | string | `"right_wheel_joint"` | 95 | 우 바퀴 조인트 이름 |
| `frame_prefix` | string | `""` | 96 | 프레임 접두어. 런치가 `amr_01/` 를 넣는다 |
| `odom_frame` | string | `"odom"` | 100 | 접두어가 붙기 전 이름 |
| `base_frame` | string | `"base_footprint"` | 101 | 접두어가 붙기 전 이름 |
| `publish_tf` | bool | `false` | 102 | TF 발행 여부 (운용에서는 EKF 가 낸다) |
| `publish_ticks` | bool | `true` | 103 | `wheel_odom/ticks` 발행 여부 |

---

## 2. `imu_filter_node`

- **소스**: `src/amr_localization/src/imu_filter_node.cpp` · C++
- **역할**: 원시 IMU → 저역통과 + 바이어스 추정·보정 → `imu/data`.
- **설정 파일**: `src/amr_localization/config/imu_filter.yaml`

### 발행 / 구독 / 서비스

| 종류 | 이름 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| 발행 | `imu/data` | `sensor_msgs/Imu` | 기본 (depth 10) | `:102` |
| 구독 | `imu/data_raw` | `sensor_msgs/Imu` | `sensor` | `:103-104` |
| 서비스 (서버) | `imu/calibrate` | `std_srvs/srv/Trigger` | — | `:106-107` |

내부 타이머 500 ms 주기로 보정 시간 초과를 검사한다 (`:112-113`). 발행 주기는 입력 IMU 주기를 따른다 (콜백 구동).

### 액션

없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `lpf_cutoff_hz` | double | `20.0` | 75 | [Hz] 저역통과 차단 주파수 |
| `gyro_bias` | double[3] | `[0,0,0]` | 77 | [rad/s] 자이로 바이어스 초기값 |
| `accel_bias` | double[3] | `[0,0,0]` | 79 | [m/s²] 가속도계 바이어스 초기값 |
| `gravity` | double | `9.8` | 80 | [m/s²] 중력 크기 |
| `bias_estimation_time` | double | `60.0` | 81 | [s] 기동 시 정지 바이어스 추정 시간 |
| `min_startup_samples` | int | `20` | 83 | 바이어스 추정 최소 표본 수 |
| `stationary_gyro_std_max` | double | `0.01` | 84 | 정지 판정: 자이로 표준편차 상한 |
| `stationary_accel_std_max` | double | `0.2` | 85 | 정지 판정: 가속도 표준편차 상한 |
| `accel_norm_tolerance` | double | `0.5` | 86 | [m/s²] 가속도 크기가 중력에서 벗어나는 허용치 |
| `max_gap` | double | `0.5` | 87 | [s] 이보다 벌어진 샘플 간격은 필터 상태를 초기화 |
| `gyro_noise_stddev` | double | `0.0002` | 88 | 출력 공분산용 자이로 잡음 σ |
| `accel_noise_stddev` | double | `0.017` | 89 | 출력 공분산용 가속도 잡음 σ |
| `calibration_samples` | int | `1000` | 91 | `imu/calibrate` 서비스가 모을 표본 수 |
| `calibration_timeout` | double | `0.0` | 93 | [s] 보정 시간 초과 (0 = 없음) |
| `calibration_dir` | string | `""` | 97 | 보정 결과 저장 디렉터리 (`""` = 저장 안 함) |
| `frame_id` | string | `""` | 98 | 출력 `header.frame_id` (`""` = 입력 그대로) |

---

## 3. `scan_filter_node`

- **소스**: `src/amr_localization/src/scan_filter_node.cpp` · C++
- **역할**: 원시 `scan` → 거리·각도 창, 이상점(outlier)·그림자(shadow) 제거 → `scan_filtered`.
- **설정 파일**: `src/amr_localization/config/scan_filter.yaml`

### 발행 / 구독

| 종류 | 이름 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| 발행 | `scan_filtered` | `sensor_msgs/LaserScan` | 기본 (depth 10) | `:54` |
| 구독 | `scan` | `sensor_msgs/LaserScan` | `sensor` | `:55-57` |

주기 = 입력 스캔 주기 (LiDAR 10 Hz, 콜백 구동). 서비스·액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `range_min` | double | `0.10` | 31 | [m] 최소 유효 거리 |
| `range_max` | double | `25.0` | 32 | [m] 최대 유효 거리 |
| `angle_window_min` | double | `-π` | 33 | [rad] 유효 각도 창 하한 |
| `angle_window_max` | double | `π` | 34 | [rad] 유효 각도 창 상한 |
| `angle_mask` | double[] | `[]` | 35 | [rad] 마스킹할 각도 구간 쌍 목록 |
| `outlier_window` | int | `2` | 42 | 이상점 판정 이웃 창 (한쪽) |
| `outlier_thresh` | double | `0.15` | 43 | [m] 이웃 대비 거리차 문턱 |
| `outlier_range_gain` | double | `3.0` | 44 | 거리에 비례해 문턱을 키우는 계수 |
| `outlier_min_neighbors` | int | `1` | 46 | 살아남는 데 필요한 최소 이웃 수 |
| `shadow_filter_enabled` | bool | `true` | 47 | 그림자(비스듬한 면) 빔 제거 |
| `shadow_min_angle_deg` | double | `10.0` | 48 | [deg] 이보다 얕은 입사각 빔은 그림자로 본다 |
| `shadow_range_noise_stddev` | double | `0.03` | 49 | [m] LiDAR 거리 잡음 σ |
| `shadow_noise_factor` | double | `3.0` | 50 | 그림자 판정 잡음 배수 |
| `stats_log_period` | double | `30.0` | 51 | [s] 통계 로그 주기 (0 = 끔) |

---

## 4. `amcl_map_adapter`

- **소스**: `src/amr_localization/amr_localization/amcl_map_adapter.py` · Python
  (진입점 `src/amr_localization/scripts/amcl_map_adapter`)
- **역할**: `nav2_amcl` (Humble 1.1.x) 의 반 셀 좌표 편향을 상쇄하려고, 원점만 `(+s/2, +s/2)` 옮긴 지도 사본을
  AMCL 전용 토픽으로 재발행한다. 다른 구독자(costmap, 추적기, `kidnap_monitor_node`)가 보는 `/map` 은 그대로 둔다.
  근거와 유도는 소스 파일 상단 docstring (`:1-18`).

### 발행 / 구독

| 종류 | 이름 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- | --- |
| 발행 | `map_amcl` | `output_topic` | `nav_msgs/OccupancyGrid` | **latched** (RELIABLE + TRANSIENT_LOCAL, depth 1) | `:64`, `:66-67` |
| 구독 | `/map` | `input_topic` | `nav_msgs/OccupancyGrid` | **latched** | `:63`, `:68-69` |

주기: 입력 지도가 올 때마다 (콜백 구동, 보통 1회). 서비스·액션 없다.

런치는 이 둘을 명시적으로 넘기고, 동시에 `amcl` 의 `map_topic` 을 `map_amcl` 로 바꾼다
(`localization.launch.py:195-198`).

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `input_topic` | string | `/map` | 63 | 원본 지도 토픽 (전역) |
| `output_topic` | string | `map_amcl` | 64 | AMCL 전용 보정 지도 토픽 (상대 이름) |

---

## 5. `kidnap_monitor_node`

- **소스**: `src/amr_localization/amr_localization/kidnap_monitor_node.py` · Python
- **역할**: 납치(kidnapped robot) 감지 — AMCL 공분산·자세 점프·스캔-지도 일치도·정합기 거부율·ArUco 마커 역산 —
  과 복구 (전역 가설 시드 → `initialpose` → AMCL 전역 재초기화 → 제자리 회전). 정지 중 AMCL 무이동 갱신도 건다.
- **설정 파일**: `src/amr_localization/config/kidnap_monitor.yaml` (감지·복구 임계값의 근거가 주석으로 들어 있다)

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `localization/lost` | `lost_topic` | `std_msgs/Bool` | **latched** | `:84`, `:135-136` |
| `localization/kidnap_status` | `status_topic` | `std_msgs/String` | 기본 (depth 10) | `:87`, `:137-138` |
| `initialpose` | `initial_pose_topic` | `geometry_msgs/PoseWithCovarianceStamped` | 기본 (depth 10) | `:73`, `:147-148` |

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `amcl_pose` | `amcl_pose_topic` | `geometry_msgs/PoseWithCovarianceStamped` | 기본 (10) | `:71`, `:139-140` |
| `odometry/filtered` | `odom_topic` | `nav_msgs/Odometry` | 기본 (20) | `:72`, `:141-142` |
| `scan_filtered` | `scan_topic` | `sensor_msgs/LaserScan` | `sensor` | `:82`, `:143-144` |
| `/map` | `map_topic` | `nav_msgs/OccupancyGrid` | **latched** | `:83`, `:145-146` |
| `initialpose` | `initial_pose_topic` | `geometry_msgs/PoseWithCovarianceStamped` | 기본 (10) | `:151-153` (자기가 낸 것은 무시) |
| `scan_match/accepted` | `scan_match_accepted_topic` | `std_msgs/Bool` | 기본 (10) | `:74`, `:156-158` |
| `localization/marker_fix` | `marker_fix_topic` | `geometry_msgs/PoseWithCovarianceStamped` | 기본 (10) | `:86`, `:179-181` |

### 서비스 (클라이언트)

| 서비스 (기본값) | 파라미터 | 타입 | 근거 |
| --- | --- | --- | --- |
| `reinitialize_global_localization` | `reinit_service` | `std_srvs/srv/Empty` | `:88`, `:159-160` |
| `request_nomotion_update` | `nomotion_service` | `std_srvs/srv/Empty` | `:103`, `:164-165` |
| `ekf_filter_node_map/set_pose` | `ekf_set_pose_service` | `robot_localization/srv/SetPose` | `:91` (기본 `''` = 사용 안 함), 런치가 값을 넣는다 `localization.launch.py:208` |

### 액션 (클라이언트)

| 액션 (기본값) | 파라미터 | 타입 | 근거 |
| --- | --- | --- | --- |
| `spin` | `spin_action` | `nav2_msgs/action/Spin` | `:89`, `:167` |

### 파라미터

인터페이스 이름 파라미터는 위 표에 있다. 나머지 — 감지·복구 임계값 — 은
**`src/amr_localization/config/kidnap_monitor.yaml` 이 단일 출처**이고, 각 값의 근거(통합 시나리오 실측)가
주석으로 붙어 있다. 코드 `declare_parameter` 로 확인한 값만 아래에 적는다.

| 이름 | 타입 | 코드 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `seed_attempts` | int | `3` | 76 | 전역 가설 상위 N 개를 차례로 `initialpose` 로 시도 (0 = AMCL 전역만) |
| `alias_reject_margin` | double | `0.15` | 79 | 별칭 가설 배제 여유 |
| `seed_cov_xy` | double | `0.0625` | 80 | [m²] 시드 `initialpose` 위치 분산 (σ 0.25 m) |
| `seed_var_yaw` | double | `0.01` | 81 | [rad²] 시드 헤딩 분산 (σ ≈ 5.7°) |
| `map_frame` | string | `map` | 75 | 지도 프레임 |
| `spin_time_allowance` | double | `30.0` | 90 | [s] Spin 액션 제한 시간 |
| `fallback_cmd_vel_topic` | string | `''` | 92 | `''` = 직접 회전 안 함 (spin 서버가 없는 단독 시험 전용) |
| `fallback_angular_speed` | double | `0.5` | 93 | [rad/s] 위 대체 회전 속도 |
| `lidar_offset` | double[3] | `[0.15, 0.0, 0.0]` | 94 | [m, m, rad] `base_footprint → lidar_link` 2D (런치가 `sensors.yaml` 로 덮어씀) |
| `inlier_dist` | double | `0.2` | 95 | [m] 스캔-지도 인라이어 거리 |
| `max_beams` | int | `180` | 96 | 일치도 계산 빔 수 |
| `occupied_thresh` | int | `65` | 97 | `OccupancyGrid` 점유 판정 값 |
| `match_rate` | double | `2.0` | 98 | [Hz] 스캔-지도 일치도 계산 주기 |
| `tick_rate` | double | `5.0` | 99 | [Hz] 상태 기계 tick 주기 |
| `event_log` | string | `''` | 100 | 이벤트 CSV 경로 (`''` = 기록 안 함) |
| `nomotion_updates_on_stop` | int | `3` | 101 | 정지당 AMCL 무이동 갱신 호출 수 (0 = 끔) |
| `nomotion_period` | double | `1.0` | 102 | [s] 위 호출 간격 |

`kidnap_monitor.yaml` 에만 있고 코드 `declare_parameter` 행을 직접 확인하지 못한 값(공통 선언 루프 `:69-70` 으로
한꺼번에 선언된다): `cov_thresh`, `yaw_cov_thresh`, `jump_thresh`, `jump_yaw_thresh`, `match_thresh`,
`match_window`, `min_match_beams`, `reg_reject_ratio`, `reg_window`, `reg_min_samples`,
`reg_marker_veto_ratio`, `suspect_time`, `reinit_retry`, `spin_retry`, `spin_angle`, `converge_cov`,
`converge_match`, `converge_margin`, `converge_count`, `recovery_timeout`, `cooldown`,
`marker_fix_min_error`, `marker_fix_persist`. 값과 근거는 config 파일을 본다.

---

## 6. `scan_matcher_node`

- **소스**: `src/amr_localization/amr_localization/scan_matcher_node.py` · Python
- **역할**: 스캔 → 정적 지도 띠 중심선 점-대-면 Gauss–Newton 정합 → map EKF 의 `pose1` 측정.
  위치 상실(`localization/lost`) 중에는 측정을 내지 않는다.
- **설정 파일**: `src/amr_localization/config/scan_matcher.yaml`

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `scan_match_pose` | `output_topic` | `geometry_msgs/PoseWithCovarianceStamped` | 기본 (10) | `:50`, `:91-92` |
| `scan_match/accepted` | `accepted_topic` | `std_msgs/Bool` | 기본 (10) | `:51`, `:89-90` |

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `scan_filtered` | `scan_topic` | `sensor_msgs/LaserScan` | `sensor` | `:48`, `:97-98` |
| `/map` | `map_topic` | `nav_msgs/OccupancyGrid` | **latched** | `:49`, `:93-94` |
| `localization/lost` | `lost_topic` | `std_msgs/Bool` | **latched** | `:52`, `:95-96` |

서비스·액션 없다. 처리 주기는 `max_rate` = 10 Hz 상한 (`:57`).

### 파라미터 (코드 선언부)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `scan_topic` | string | `scan_filtered` | 48 | 입력 스캔 |
| `map_topic` | string | `/map` | 49 | 정적 지도 (전역) |
| `output_topic` | string | `scan_match_pose` | 50 | 정합 결과 자세 |
| `accepted_topic` | string | `scan_match/accepted` | 51 | 스캔마다 수락/거부 (납치 감지가 쓴다) |
| `lost_topic` | string | `localization/lost` | 52 | 위치 상실 상태 입력 |
| `map_frame` | string | `map` | 53 | 지도 프레임 |
| `base_frame` | string | `base_footprint` | 54 | 로봇 프레임. 런치가 `<prefix>base_footprint` 로 덮어씀 (`localization.launch.py:217`) |
| `lidar_offset` | double[3] | `[0.15, 0.0, 0.0]` | 55 | [m, m, rad] `base→lidar` 2D |
| `max_beams` | int | `360` | 56 | 정합에 쓰는 빔 수 |
| `max_rate` | double | `10.0` | 57 | [Hz] 스캔 처리 상한 |
| `tf_timeout` | double | `0.05` | 58 | [s] 스캔 시각 `map→base` TF 대기 |
| `max_tf_age` | double | `0.5` | 59 | [s] 최근 TF 로 대체 허용 |
| `occupied_thresh` | int | `65` | 60 | `OccupancyGrid` 점유 판정 값 |
| `stats_log_period` | double | `30.0` | 61 | [s] 통계 로그 주기 (0 = 끔) |

정합 알고리즘 파라미터(`centroid_radius`, `normal_radius`, `min_planarity`, `surface_offset`,
`max_correspondence`, `range_noise`, `huber_k`, `max_iterations`, `converge_translation`,
`converge_rotation`, `min_inlier_ratio`, `min_points`, `max_correction`, `max_rotation_correction`,
`covariance_scale`, `min_sigma_xy`, `min_sigma_yaw`)는 공통 선언 루프 `:46-47` 로 한꺼번에 선언되고
기본값·근거는 `src/amr_localization/config/scan_matcher.yaml` 이 단일 출처다.

---

## 7. 외부 노드 — 우리가 정한 이름·프레임만

`localization.launch.py` 가 설정하는 값이다. 인터페이스 자체는 각 외부 패키지 문서를 따른다.

| 노드 | 패키지 | 우리가 정한 것 | 근거 |
| --- | --- | --- | --- |
| `ekf_filter_node_odom` | `robot_localization` | `world_frame: <prefix>odom`, `odom_frame`, `base_link_frame`, `map_frame: map`. 서비스 `set_pose`/`toggle`/`enable` 을 `ekf_filter_node_odom/<이름>` 으로 리맵. 출력 `odometry/filtered`, TF `odom → base_footprint` | `:151-156` |
| `ekf_filter_node_map` | `robot_localization` | `world_frame: map`. 출력 토픽을 `odometry/filtered` → **`odometry/filtered_map`** 으로 리맵. TF `map → odom` | `:178-179` |
| `amcl` | `nav2_amcl` | `global_frame_id: map`, `odom_frame_id: <prefix>odom`, `base_frame_id: <prefix>base_footprint`, `initial_pose.{x,y,yaw}`, `map_topic: map_amcl` (반 셀 보정 사용 시). `tf_broadcast: false` 는 `config/amcl.yaml` | `:195-202` |
| `slam_toolbox` | `slam_toolbox` | `odom_frame`, `base_frame`, `map_frame: map`. `mode:=slam` 일 때만 | `:159-162` |
| `map_saver_server` | `nav2_map_server` | 토픽 `map` → `/map` 리맵, `save_map_timeout: 15.0`, `free_thresh_default: 0.19`, `occupied_thresh_default: 0.65` | `:171-174` |
| `map_server` | `nav2_map_server` | **루트 네임스페이스**(`namespace=''`), `topic_name: map`, `frame_id: map`, `yaml_filename` = 런치 인자 `map` | `:184-186` |
| `lifecycle_manager_localization` / `_map` / `_map_saver` | `nav2_lifecycle_manager` | `autostart: true`, `node_names`, `bond_timeout` | `:175-177`, `:187-190`, `:203-205` |

### 런치 인자

| 인자 | 기본값 | 설명 | 행 |
| --- | --- | --- | --- |
| `mode` | `localization` | `slam` \| `localization` \| `odom` | 229 |
| `robot_name` | `amr_01` | 네임스페이스 (`''` = 없음) | 231 |
| `frame_prefix` | `auto` | `auto` = `robot_name + '/'` | 233 |
| `use_sim_time` | `true` | | 235 |
| `config_dir` | `$ROS_WS/config` | | 236 |
| `map` | `$ROS_WS/maps/warehouse.yaml` | | 237 |
| `start_map_server` | `true` | | 238 |
| `initial_x` / `initial_y` / `initial_yaw` | `0.0` | AMCL 초기 자세 | 239-241 |
| `use_kidnap_monitor` | `true` | | 242 |
| `kidnap_fallback_cmd_vel` | `''` | | 243 |
| `use_scan_matcher` | `true` | 스캔-지도 정합 → map EKF `pose1` | 244 |
| `amcl_half_cell_fix` | `true` | AMCL 에 `map_amcl` 을 준다 | 246 |
| `bias_estimation_time` | `''` | IMU 바이어스 추정 시간 덮어쓰기 | 248 |
| `noise_seed` | `0` | | 249 |
| `log_level` | `info` | | 250 |

---

## 8. 확인 못 함

- `kidnap_monitor_node` 의 `fallback_cmd_vel_topic` 이 비어 있지 않을 때만 만드는 `Twist` 발행자
  (`kidnap_monitor_node.py:178`)는 **단독 시험 전용**이라 발행 토픽 표에 넣지 않았다. 운용에서는 `''` 이다
  (`kidnap_monitor.yaml`: "운용 시 cmd_vel 발행자는 safety_node 하나").
- `kidnap_monitor_node` · `scan_matcher_node` 의 공통 선언 루프(`:69-70`, `:46-47`)로 선언되는 파라미터들은
  **기본값이 코드가 아니라 dataclass/config 쪽에 있다.** 위 표에서 "config 가 단일 출처" 로 표시한 항목들이며,
  각 값을 코드 한 행으로 지목하지는 못했다.
- 각 노드의 **실제 발행 주기**는 파라미터 값으로만 적었다. 런타임 실측은 하지 않았다
  (이 작업은 정적 분석만 허용됨).
- 외부 노드(`amcl`, `ekf_*`, `slam_toolbox`, `map_server`)의 **전체 토픽·서비스 목록**은 적지 않았다.
