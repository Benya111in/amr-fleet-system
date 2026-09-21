# 센서 캘리브레이션 절차

> 명세 4장 1절: "센서 캘리브레이션 절차를 문서화하고, 외부 파라미터(extrinsic) 설정 파일을 제공한다."
> 외부 파라미터 파일은 `config/sensors.yaml` 이다. 아래 절차는 시뮬레이션에서 그대로 수행하고,
> Ground Truth(GT) 출처만 바꾸면 실제 로봇에도 적용된다. 결과는 3장 표 템플릿으로 `docs/reports/` 에 남긴다.

## 1. 외부 파라미터(extrinsic) 관리 원칙

### 1.1 값이 사는 곳

- `config/sensors.yaml` 의 `<센서>.extrinsic: {x, y, z, roll, pitch, yaw}` 가 **유일한 원본**이다.
  값은 `base_link` 기준 센서 **본체 링크**(body convention, REP-103: x 전방 / y 좌 / z 상)의 자세다.

| 링크 | extrinsic (m, rad) | 메시지 `frame_id` |
| --- | --- | --- |
| `lidar_link` | (0.15, 0, 0.20, 0, 0, 0) | `lidar_link` |
| `camera_link` | (0.18, 0, 0.25, 0, 0, 0) | RGB `camera_optical_frame`, Depth `camera_depth_optical_frame` |
| `imu_link` | (0, 0, 0.10, 0, 0, 0) | `imu_link` |

- 카메라 광학 프레임(`camera_optical_frame`, `camera_depth_optical_frame`)은 `camera_link` 의 **고정 자식**이며
  rpy = (−π/2, 0, −π/2) 로만 회전한다 (REP-103 optical: z 전방 / x 우 / y 하). 쿼터니언 (x, y, z, w) = (−0.5, 0.5, −0.5, 0.5).
  extrinsic 값을 광학 프레임에 직접 적지 않는다. 광학 회전은 URDF 상수다.
- URDF(xacro, `src/amr_description/urdf/`)와 Gazebo 센서 `<pose>` 는 이 값을 그대로 쓴다. **URDF == sensors.yaml == TF** 가 원칙이며,
  값을 바꿀 때는 `sensors.yaml` 을 먼저 고치고 xacro 를 맞춘다. 다중 로봇 프레임 접두어는 `docs/architecture/multi_robot.md` 참조.

### 1.2 일치 검증 (extrinsic 을 바꿀 때마다)

```bash
ros2 run tf2_tools view_frames -o logs/frames            # logs/frames.pdf, logs/frames.gv
ros2 run tf2_ros tf2_echo base_link lidar_link            # Translation [0.150, 0.000, 0.200], RPY [0, 0, 0]
ros2 run tf2_ros tf2_echo base_link camera_link           # [0.180, 0.000, 0.250]
ros2 run tf2_ros tf2_echo camera_link camera_optical_frame  # [0, 0, 0], Quaternion [-0.500, 0.500, -0.500, 0.500]
ros2 run tf2_ros tf2_echo base_link imu_link              # [0.000, 0.000, 0.100]
ros2 topic echo /scan --once | grep frame_id              # lidar_link — 메시지 frame_id 도 TF 와 같아야 한다
ros2 topic echo /camera/camera_info --once | grep frame_id  # camera_optical_frame
```

세 값(URDF, `sensors.yaml`, `tf2_echo` 출력) 중 하나라도 다르면 캘리브레이션 결과는 무효다. 아래 절차는 이 검증을 통과한 뒤 시작한다.

## 2. 센서별 절차

공통 준비
- 로봇 정지: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`.
- 시뮬레이션 GT: Gazebo `OdometryPublisher` 시스템이 발행하는 `ground_truth` 토픽(`nav_msgs/Odometry`, `frame_id: world`, 월드 원점 기준 — 이미지의 Gazebo 6.18 로 확인). 벽/선반 위치는 월드 SDF(`src/amr_simulation/worlds/`)의 값을 쓴다.
- 실제 로봇 GT: 줄자·레이저 거리계(벽 거리), 바닥 마킹(직진/회전), 가능하면 모션캡처. 절차와 로그 포맷은 동일하다.
- 기록: `ros2 bag record -o logs/calibration/<항목> <토픽...>`, 분석 스크립트는 `scripts/` 에 둔다 (명세 10장 로그 규칙과 동일).

### 2.1 LiDAR ↔ base 외부 파라미터 검사 (벽 / 코너)

1. 평평한 벽 앞 약 2 m 에 로봇을 세운다. 벽 평면 위치는 월드 SDF 에서, 로봇 자세는 `ground_truth` 에서 읽는다.
2. 60 스캔 기록: `ros2 bag record -o logs/calibration/lidar_wall /scan /ground_truth`.
3. 스캔마다 |각도| ≤ 30° 이고 range 가 유한한 빔을 `lidar_link` 평면 점 (x = r cos θ, y = r sin θ) 으로 바꾼다.
4. 총최소제곱 직선 적합(PCA: 공분산 최소 고유벡터 = 법선 n, 중심 c). 측정 거리 d_meas = |c · n|, 벽 법선 방향 ψ_meas = atan2(n_y, n_x).
5. 기대값: GT 로봇 자세 ⊕ extrinsic(0.15, 0, 0.20) 로 LiDAR 원점을 월드로 옮겨 벽 평면까지 거리 d_exp, GT yaw 로 ψ_exp 를 계산.
6. Δd = d_meas − d_exp 는 벽 법선 방향의 장착 오프셋 오차, Δψ 는 장착 yaw 오차다. 정면(벽 법선 = +x)에서 Δd 는 x 오프셋,
   로봇을 90° 돌려 벽을 옆에 두면 Δd 가 y 오프셋이 된다. 직교하는 두 벽이 만나는 **코너**를 쓰면 한 자세로 x, y, yaw 를 동시에 얻는다.

- 판정: |Δd| ≤ 0.01 m, |Δψ| ≤ 0.5° (LiDAR 각해상도 1 빔). 60 스캔 평균의 표준오차는 0.03/√(60·120) ≈ 0.0004 m 이므로 0.01 m 기준은 충분히 느슨하다.
- 사전 확인(독립 SDF, Gazebo 6.18, 벽 2.0 m, 60 스캔): d_meas = 1.8505 m, 기대 1.85 m (2.0 − 0.15). 파이프라인 자체는 검증됐다.
- 로그: `[timestamp, gt_x, gt_y, gt_yaw, d_meas, d_exp, delta_d, delta_yaw]` → `logs/calibration/lidar_extrinsic.csv`

### 2.2 카메라 내부 파라미터 (intrinsics)

시뮬레이션: Gazebo 가 `/camera/camera_info` 로 K 를 발행한다. `config/sensors.yaml` 의 640×480, hfov 1.518436 rad(87°) 에서 기대값은

```
fx = fy = (W/2) / tan(hfov/2) = 320 / tan(0.7592) = 337.2 px,  cx = 320, cy = 240, D = 0
```

이미지의 Gazebo 6.18 실측: fx = fy = 337.2098, cx = 320, cy = 240, 왜곡 계수 0 (독립 SDF). `amr_perception` 의 Pinhole 2D→3D 변환은
K 를 하드코딩하지 말고 **반드시 `camera_info` 를 구독**해서 읽는다. 그래야 실제 카메라로 바꿔도 코드 수정이 없다.

실제 로봇: 체커보드(9×6 내부 코너, 25 mm) 를 화면 네 모서리를 포함해 20장 이상 촬영 → `cv2.findChessboardCorners` + `cv2.calibrateCamera`.
판정: RMS 재투영 오차 ≤ 0.3 px. 결과는 카메라 드라이버의 `camera_info_url` YAML(ROS 표준 형식)로 저장해 같은 `camera_info` 토픽으로 나가게 한다.
(`camera_calibration` 패키지는 현재 이미지에 없으므로 OpenCV 스크립트를 `scripts/` 에 둔다.)

- 로그: `[timestamp, fx, fy, cx, cy, rms_px]` → `logs/calibration/camera_intrinsics.csv`

### 2.3 Depth ↔ RGB 정렬 검사

시뮬레이션에서는 두 카메라의 extrinsic(0.18, 0, 0.25) 과 해상도·FOV 가 같으므로 픽셀 (u, v) 가 1:1 대응한다.

1. `ros2 run tf2_ros tf2_echo camera_optical_frame camera_depth_optical_frame` → 항등 변환이어야 한다.
2. 카메라 정면, GT 거리 d 인 위치에 박스(명세 8장 물품)를 둔다. RGB 에서 박스 중심 픽셀 (u, v)(YOLO bbox 중심 또는 수동)를 잡고
   `/camera/depth/image_raw` 의 같은 (u, v) 깊이를 읽는다.
3. 기대 깊이 = d − 0.18(카메라 x 오프셋) − 박스 반두께. 허용 오차 = 깊이 노이즈 σ(d) = 0.005 + 0.002·d² (`sensors.yaml`, d = 2 m 이면 0.013 m).
4. 박스 좌우 에지 열(column) 이 RGB 와 Depth 에서 2 px 이내로 일치해야 한다.

실제 로봇: RGB-D 센서의 정렬 스트림(예: align_depth)을 쓰거나, 체커보드로 `cv2.stereoCalibrate`(IR/Depth ↔ RGB) 해서 R, t 를 얻어
`camera_depth_optical_frame` 의 URDF 조인트에 반영한 뒤 위 2~4 를 다시 수행한다.

- 로그: `[timestamp, u, v, depth_meas, depth_exp, delta]` → `logs/calibration/depth_rgb_align.csv`

### 2.4 IMU 바이어스 추정

`config/sensors.yaml` 모델: 가속도 σ_a = 0.017 m/s², 바이어스 σ = 0.001; 자이로 σ_g = 0.0002 rad/s, 바이어스 σ = 7.5e-6; 100 Hz.
Gazebo 는 SDF `<noise>` 의 `bias_mean`/`bias_stddev` 로 **실행마다 상수 바이어스를 새로 뽑는다**(부호도 무작위). 따라서 바이어스는
파일에 고정하지 말고 **기동 시마다** 추정한다. (`dynamic_bias_stddev` 를 켜면 랜덤워크가 추가되므로 주기적 재추정이 필요하다.)

절차: 수평 바닥에서 정지 상태로 T 초 기록(`/imu/data_raw`), 평균을 낸다.

```
b_a = mean(a) − g_ref,   g_ref = (0, 0, +9.81)   # Gazebo IMU 는 정지 시 z ≈ +9.8 을 보고한다 (실측 9.825)
b_g = mean(ω)
표준오차  SE = σ / √N,  N = f · T
```

| 센서 | σ | T = 60 s (N = 6000) | T = 300 s (N = 30000) | 바이어스 σ (모델) |
| --- | --- | --- | --- | --- |
| 가속도 | 0.017 m/s² | 0.017/√6000 ≈ 2.2e-4 | 9.8e-5 | 1.0e-3 → 60 s 면 ≈ 4.5σ 로 분해 |
| 자이로 | 0.0002 rad/s | 0.0002/√6000 ≈ 2.6e-6 | 1.2e-6 | 7.5e-6 → 60 s 는 ≈ 3σ, **300 s 권장** |

적용: `src/amr_localization` 의 IMU 필터 노드(명세 4장 2절 "저역 통과 필터 및 바이어스 보정")가 `imu/data_raw` 를 받아
`a' = LPF(a − b_a)`, `ω' = LPF(ω − b_g)` 를 `imu/data` 로 발행하고, EKF(`config/ekf.yaml` 의 `imu0: imu/data`)는 보정된 값만 본다.
바이어스는 노드 파라미터(`accel_bias`, `gyro_bias`, double[3])로 두고, 기동 시 `bias_estimation_time`(기본 60 s) 동안 정지 상태로 자동 추정하되
실제 로봇에서는 파일 값으로 덮어쓸 수 있게 한다. 2D 주행에서 실질적으로 중요한 것은 자이로 z(yaw 드리프트: 7.5e-6 rad/s ≈ 1.5°/h) 와
가속도 x, y(EKF 가 ax, ay 를 융합하므로 0.001 m/s² 바이어스도 누적된다)다.

- 로그: `[timestamp, ax, ay, az, gx, gy, gz]` 원본 + 요약 1행 `[T, N, bax, bay, baz, bgx, bgy, bgz, se_a, se_g]` → `logs/calibration/imu_bias.csv`

### 2.5 휠 오도메트리 캘리브레이션 (UMBmark 계열)

공칭값: `config/robot_params.yaml` 의 `wheel_radius` r = 0.0825 m, `wheel_separation` b = 0.36 m. 인코더 4096 tick/rev, 슬립 노이즈 σ 0.01
(`sensors.yaml` `wheel_encoder`). 오도메트리 노드(`src/amr_localization`, 순기구학 직접 구현)가 `joint_states` 로부터 `wheel_odom` 을 발행한다.
차동 구동 순기구학에서 이동 거리는 r 에, 회전각은 r/b 에 비례하므로 두 실험으로 분리해서 보정한다.

(a) 직진 5 m — 반지름 스케일
1. 0.3 m/s 로 전진, `wheel_odom` 누적 거리가 정확히 5.000 m 가 되면 정지. GT 거리 L_gt = 시작·종료 `ground_truth` 위치의 유클리드 거리.
2. k_r = L_gt / L_odo, r ← k_r · r. 전진 5회 + 후진 5회 평균.
3. 종료 시 GT 횡방향 편차 y 가 있으면 좌우 바퀴 지름 비 E_d 가 원인이다 (Borenstein & Feng, 1996):
   `R = L² / (2y)`, `E_d = D_R / D_L = (R + b/2) / (R − b/2)`, `r_L = 2r / (E_d + 1)`, `r_R = 2·E_d·r / (E_d + 1)`.
   오도메트리 노드가 좌우 반지름을 따로 받을 때만 적용한다.

(b) 제자리 5회전 — 축간 거리
1. ω = 0.5 rad/s 로 회전, `wheel_odom` yaw 누적(unwrap)이 10π 가 되면 정지. GT Δθ_gt 는 `ground_truth` yaw 누적(unwrap).
2. `Δθ = r (Δφ_R − Δφ_L) / b` 이므로 (a) 에서 r 을 먼저 고친 뒤 `k_b = θ_odo / θ_gt`, b ← k_b · b. CW/CCW 각 3회 평균.

- 판정(보정 후 재실험): 직진 5 m 오차 ≤ 0.05 m(1 %), 5회전 오차 ≤ 2°(0.035 rad).
- 시뮬레이션에서는 URDF 값과 공칭값이 같으므로 k_r, k_b ≈ 1.000 ± 슬립 노이즈가 나와야 한다. 크게 벗어나면 오도메트리 노드의 단위/부호 버그다.
- 실제 로봇: L_gt 는 바닥 테이프 선을 따라 줄자로, Δθ 는 바닥 마킹 또는 바이어스 보정된 자이로 적분으로 교차 확인한다.
- 로그: 명세 표준 포맷 `[timestamp, gt_x, gt_y, est_x, est_y, error]` (est = `wheel_odom`) → `logs/calibration/odom_straight_<n>.csv`,
  회전은 `[timestamp, gt_yaw, odom_yaw, error_yaw]` → `logs/calibration/odom_rotate_<n>.csv`

### 2.6 노이즈 검증 실험 (명세 7장 "센서 노이즈 모델을 반드시 적용")

- LiDAR: 2.1 과 같은 벽 설정, 60 스캔 이상. 스캔마다 직선 적합 잔차의 표준편차 = σ_meas. 설정값 `lidar.noise_stddev: 0.03` 대비 ±10 %(0.027~0.033) 이면 합격.
  감사 측정 0.0299 m, 독립 SDF 사전 확인 0.0285 m(60 스캔 × 약 120 빔). 잔차 > 4σ 인 빔 수가 0 에 가까운지도 본다(아웃라이어 모델 없음).
- IMU: 2.4 의 정지 기록에서 std(a) ≈ 0.017, std(ω) ≈ 0.0002 (±10 %).
- Depth: 고정 픽셀의 100 프레임 표준편차가 σ(d) = 0.005 + 0.002·d² 와 ±20 % 이내.
- SDF 주의: LiDAR 노이즈는 `<noise><type>gaussian</type>...</noise>`(자식 요소), IMU/카메라는 `<noise type="gaussian">`(속성)이다.
  섞어 쓰면 Gazebo 가 "XML Attribute[type] in element[noise] not defined" 경고를 내며 노이즈가 적용되지 않을 수 있다 (사전 확인에서 관찰).

## 3. 결과 표 템플릿

`docs/reports/sensor_calibration_<YYYYMMDD>.md` 에 아래 표를 채운다. 로그는 `logs/calibration/` 아래 위 절차의 파일명을 쓴다.

| 항목 | 설정값 / 공칭 | 측정값 | 허용 오차 | 판정 | 로그 | 일자 / 담당 |
| --- | --- | --- | --- | --- | --- | --- |
| TF == URDF == sensors.yaml | `tf2_echo` 4종 (1.2) | | 0 | | `logs/frames.pdf` | |
| LiDAR extrinsic Δd / Δψ | (0.15, 0, 0.20) | 사전 확인 Δd = +0.0005 m | 0.01 m / 0.5° | | `lidar_extrinsic.csv` | |
| LiDAR 노이즈 σ | 0.030 m | 0.0299 (감사) / 0.0285 (사전) | ±10 % | | `lidar_wall/` | |
| 카메라 fx, fy, cx, cy | 337.2, 337.2, 320, 240 | 337.21, 337.21, 320, 240 (사전) | 1 px / RMS 0.3 px(실기) | | `camera_intrinsics.csv` | |
| Depth ↔ RGB 정렬 | 항등 변환, Δ ≤ σ(d) | | 2 px, σ(d) | | `depth_rgb_align.csv` | |
| IMU 가속도 바이어스 (x, y, z) | 모델 σ 0.001 | | SE 2.2e-4 (60 s) | | `imu_bias.csv` | |
| IMU 자이로 바이어스 (x, y, z) | 모델 σ 7.5e-6 | | SE 1.2e-6 (300 s) | | `imu_bias.csv` | |
| IMU 노이즈 σ_a / σ_g | 0.017 / 0.0002 | | ±10 % | | `imu_bias.csv` | |
| 휠 반지름 스케일 k_r | 1.000 (r = 0.0825) | | 직진 5 m ≤ 0.05 m | | `odom_straight_*.csv` | |
| 축간 거리 스케일 k_b | 1.000 (b = 0.36) | | 5회전 ≤ 2° | | `odom_rotate_*.csv` | |
| Depth 노이즈 σ(d) | 0.005 + 0.002·d² | | ±20 % | | `depth_noise.csv` | |

로그 포맷 요약 (명세 10장 표준 + 본 문서 정의)

```
위치/오도메트리 : [timestamp, gt_x, gt_y, est_x, est_y, error]
회전            : [timestamp, gt_yaw, odom_yaw, error_yaw]
LiDAR extrinsic : [timestamp, gt_x, gt_y, gt_yaw, d_meas, d_exp, delta_d, delta_yaw]
IMU             : [timestamp, ax, ay, az, gx, gy, gz]
Depth 정렬      : [timestamp, u, v, depth_meas, depth_exp, delta]
```

## 4. 실제 로봇 전이 체크리스트

- [ ] 센서 드라이버가 같은 토픽·`frame_id`(`scan`/`lidar_link`, `imu/data_raw`/`imu_link`, `camera/*`/`camera_optical_frame`)로 발행한다.
- [ ] `sensors.yaml` extrinsic 을 실측(도면 + 2.1 벽 검사)으로 갱신하고 1.2 검증을 다시 통과했다.
- [ ] 카메라 `camera_info` 가 체커보드 결과로 채워졌다 (2.2). Depth 정렬 확인(2.3).
- [ ] IMU 바이어스를 파일 값으로 고정할지, 기동 시 자동 추정할지 결정했다 (2.4).
- [ ] 휠 r, b 를 UMBmark 결과로 갱신했다 (2.5). 노이즈 파라미터는 실측 σ 로 갱신해 EKF 공분산(`docs/algorithms/ekf.md`)에 반영했다.

## 5. 미확정 항목

- 시뮬레이션 사전 확인은 저장소 URDF 가 아직 없어 독립 SDF 로 수행했다. 저장소 xacro 가 생기면 같은 절차로 재측정해 3장 표를 채운다.
- `imu/data_raw` → `imu/data` 토픽 분리, 바이어스 파라미터 이름, `ground_truth` 토픽 이름은 이 문서의 제안이며 구현 시 확정한다.
