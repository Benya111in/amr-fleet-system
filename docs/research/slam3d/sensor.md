# 3D LiDAR 센서 계약 — 토픽·프레임·QoS·필드 (보너스 2, s1-sensor)

이 문서는 3D LiDAR SLAM(RTAB-Map·LIO-SAM, s3)이 **그대로 쓰는** 센서 쪽 계약이다. 바꿀 필요가 생기면 이 문서와
`src/amr_description/config/lidar3d.yaml` 을 함께 고친다. 근거 보고서: 「3D LiDAR SLAM 도입 방안」 31–80행(센서),
191–200행(오라클), 255–272행(1–2단계). 표기: **[측정]** 명령·로그 있음, **[코드]** 이 저장소 코드로 정해진 값,
**[문서주장]** 보고서·상류 소스 주장(우리 실행에서 미확인).

## 1. 켜는 법

| 방법 | 효과 |
|---|---|
| `multi_robot.launch.py robots_file:=…/fleet_spawn_lidar3d.yaml` | amr_01 만 3D (나머지 4대는 기본과 같다) |
| 스폰 목록 로봇 항목에 `lidar3d: true` | 그 로봇만 3D (`amr_bringup/fleet_spawn.py` 가 형식 검증) |
| `description.launch.py lidar3d:=true` + `spawn.launch.py lidar3d:=true` | 단독 기동 (두 런치에 같은 값을 줘야 한다) |

- 두 런치의 `lidar3d` 기본값은 `auto` 다: 런치 범위에 `robots_file` 이 있으면 그 파일의 이 로봇 `lidar3d` 키, 없으면
  false (`amr_description/lidar3d.py resolve`). `multi_robot`·`system.launch.py` 는 `robots_file` 을 최상위 인자로
  선언하므로 그 아래 description·spawn 런치가 같은 값을 본다 [코드].
- 기본(키 없음·false)의 URDF 는 3D 이전(d8ae80c)과 **바이트 단위로 같고**, 브리지 항목도 같다 —
  `src/amr_description/test/test_urdf_golden.py` [코드·시험].

## 2. 토픽 (로봇 네임스페이스 `/<robot>/` 아래 상대 이름)

| ROS 토픽 | 형 | 발행자 | QoS | 내용 |
|---|---|---|---|---|
| `lidar3d/points_raw` | sensor_msgs/PointCloud2 | ros_gz_bridge (gz `/<robot>/lidar3d/points`) | reliable, keep_last 10 | Gazebo 원본. **lazy**: ROS 구독자가 없으면 브리지가 gz 구독을 열지 않고 Gazebo 도 렌더하지 않는다 |
| `lidar3d/points` | sensor_msgs/PointCloud2 | `lidar3d_preprocessor` (amr_lidar3d) | reliable, keep_last 5, volatile | **소비자는 이것을 쓴다.** 비유한 점 제거, `height 1`, `is_dense true`, 필드 배치 그대로 |
| `imu/data_raw` | sensor_msgs/Imu | ros_gz_bridge | reliable, keep_last 10 | 3D 로봇만 **200 Hz** (다른 로봇 100 Hz). 방위는 정답 — 6절 |
| `ground_truth/odom` | nav_msgs/Odometry | ros_gz_bridge | reliable | 평가 전용 정답 (frame `world`, child `<prefix>base_footprint`, 50 Hz) |

- 다층 LaserScan(gz `/<robot>/lidar3d`)은 **브리지하지 않는다**: ros_gz_bridge(humble)는 다층 스캔을 "가운데 행"만
  넘긴다 [문서주장 — 보고서, ros_gz convert/sensor_msgs.cpp].
- `lidar3d_preprocessor` 도 lazy 다 (기본): `lidar3d/points` 구독자가 생겨야 `points_raw` 를 구독한다 (0.5 s 마다
  구독자 수 확인 — Humble 에 matched 이벤트가 없다). 따라서 **아무도 3D 점군을 구독하지 않으면 렌더 비용이 0** 이고,
  SLAM 노드가 붙는 순간 체인이 켜진다. 첫 메시지까지 최대 ≈ 0.5 s(전처리) + 1 s(브리지 heartbeat) 지연 [코드].
- QoS 를 reliable 로 둔 이유: RTAB-Map 기본 구독이 reliable 이고, reliable 발행자에는 best-effort 구독자도 붙는다.
  반대(best-effort 발행자 + reliable 구독자)는 연결되지 않는다.
- 컴포넌트: `amr_lidar3d::CloudPreprocessor` — SLAM 노드와 같은 컨테이너에 올리면 intra-process (`launch/preprocessor.launch.py container:=…`).
  spawn.launch.py 가 띄우는 기본 인스턴스와 **중복으로 띄우지 않는다** (`lidar3d_preprocessor:=false` 로 끈 뒤 컨테이너에 싣는다).

## 3. 프레임

```
<prefix>base_footprint ─ <prefix>base_link (z 0.18) ─ <prefix>lidar3d_link   xyz (0.27, 0, 0.72), rpy 0
```

- `lidar3d_link`: 본체 규약 (x 전방, y 좌, z 상), **광학 중심 지면 +0.90 m**, 데크 앞끝(차체 전면 3 cm 안쪽) 마스트 위 [코드].
  base_footprint 기준 (0.27, 0, 0.90). robot_state_publisher 가 `/tf_static` 으로 낸다.
- 점군 `header.frame_id` = `<prefix>lidar3d_link` (Gazebo `ignition_frame_id`; 전처리는 기본으로 바꾸지 않는다).
  `multi_robot` 의 접두어는 `amr_01/`, `system.launch.py` 단일 로봇은 접두어 없음.
- `header.stamp` = 스캔 한 장의 시뮬레이션 시각 (`/clock`). LiDAR·IMU·정답이 같은 시계라 시간 오프셋 0.

## 4. 점군 형식

| 항목 | 값 | 근거 |
|---|---|---|
| 필드 | `x` FLOAT32 @0, `y` FLOAT32 @4, `z` FLOAT32 @8, `intensity` FLOAT32 @16, `ring` UINT16 @24 | 실측 대기 |
| point_step | 32 B (정렬 패딩) | 실측 대기 |
| **점별 time** | **없다.** 스캔 전체가 한 시각·한 자세의 순간 렌더 (모션 왜곡 없음) | [문서주장] GpuLidarSensor.cc |
| ring | 0 = 가장 아래 빔(−15°) … 15 = 위(+15°), 2° 간격 | [코드] SDF + 실측 대기 |
| 방위 순서 | 링마다 −π 부터 0.2° 씩 증가 (1800 열, 첫·끝 열 겹침 없음) | [코드] SDF |
| raw 조직 | 실측 대기 | |
| raw 비유한 점 | 범위 밖 빔 ±inf (성분이 0 인 축은 inf·0 = NaN 이 될 수 있다), 그때 `is_dense false` | 실측 대기 |
| 전처리 출력 | `height 1`, `width` = 유한 점 수, `row_step = width·32`, `is_dense true`, 순서 유지 | [코드·시험] |
| intensity | 시각체의 `laser_retro` 값 — 반사율 물리 모델이 아니다 | [문서주장] |
| 거리 | 0.10–30 m, 빔별 가우시안 σ 0.03 m (Fortress 는 이것만 지원) | [코드] |
| 크기 | 스캔당 28 800 빔 × 32 B = 0.92 MB (raw), 10 Hz | [계산] |

### 소비자가 할 것 (s3)

- **RTAB-Map**: `deskewing:=false` (time 이 없다), 최소거리는 SLAM 쪽에서 0.2 m (`Icp/RangeMin` 등 — 통로 랙 면이 횡 0.30 m).
- **LIO-SAM**: `ring` 이 있어 "ring channel not available" 로 죽지 않는다. `time` 이 없어
  "Point cloud timestamp not available, deskew function disabled" 경고가 나온다 — 정상이다.
  **time 을 합성하지 않는다** (`col/width·0.1 s` 를 넣으면 LIO-SAM 이 없는 왜곡을 IMU 회전 ω·t 만큼 "보정"해
  1.5 rad/s 에서 10 m 점이 약 1.5 m 어긋난다 — 보고서 계산). `N_SCAN 16`, `Horizon_SCAN 1800`, `lidarMinRange 0.2`.
- PCL 로 받을 때 `time` 필드가 없다는 경고(pcl::fromROSMsg "Failed to find match for field 'time'")는 무시해도 된다.

## 5. 센서 정의 (Gazebo Fortress `gpu_lidar`, `urdf/amr_lidar3d.xacro`)

16 채널, 수직 −15°…+15° (samples 16, resolution 1), 수평 1800 열 −π…π−2π/1800 (resolution 1 — Fortress 는
resolution≠1 보간을 하지 않는다), 10 Hz, range 0.10–30 m, 가우시안 σ 0.03, `visibility_mask` 로 자기 차체·마스트·화물
제외(다른 로봇은 보인다), 시각체만 있는 마스트(충돌체·관성 없음 — 동역학·적재 동작 무변화).

## 6. IMU 방위 오라클 — 쓰지 마라

Fortress IMU 는 가속도·자이로에만 잡음을 붙이고 **방위(orientation)는 잡음 없는 정답**이다 (기준 = 스폰 시점 자세).
`enable_orientation` 을 끄면 w=0 쿼터니언이 나가 소비 노드가 깨질 수 있어 켜 둔다 (기존 결정). 그러므로
3D SLAM 의 IMU 입력은 방위를 버리고 다시 만든다: `imu/data_raw` → 프로젝트 imu_filter_node(바이어스 보정) →
imu_filter_madgwick(`use_mag:=false`) (보고서 65–69행). 정답 방위를 쓴 실행은 "오라클 행"으로만 따로 보고한다.
3D 로봇 IMU: 200 Hz, 이산 σ = 기본 σ × √2 (자이로 2.83e-4 rad/s, 가속도 0.024 m/s²), 바이어스 그대로 [코드].
우리 실행에서의 확인: 실측 대기.

## 7. 측정 (보고서 1–2단계)

실측 대기.
