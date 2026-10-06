# 3D SLAM 매핑 행 — RTAB-Map 3D·2D, slam_toolbox, LIO-SAM (보너스 2 4–5단계, s3)

패키지 `src/amr_slam3d` 의 설계·사용법·파일럿 결과. 근거 보고서: 「3D LiDAR SLAM 도입 방안」 81–164행(SLAM 선택·
LIO-SAM 방위), 191–200행(오라클), 255–272행(4–5단계). 센서 계약은 [`sensor.md`](sensor.md)(s1)를 그대로 쓴다.
표기: **[측정]** 명령·로그 있음, **[코드]** 이 저장소·이미지 안 코드로 정해진 값, **[문서주장]** 보고서·상류 주장(우리 실행에서
미확인), **미측정**.

## 1. 행과 구성

| 행 | 이미지 | 입력 | 휠-EKF 사전값 | IMU | 지도 프레임 → 월드 |
|---|---|---|---|---|---|
| `rtabmap3d` | amr-ai/slam:0.1 (rtabmap 0.23.7) | `/amr_01/lidar3d/points` (전처리 점군) | `guess_frame_id = amr_01/odom` | `imu/data_madgwick` | `map → slam_map` = 스폰 자세 |
| `rtabmap2d` | 같음 | `/amr_01/scan_filtered` | 같음 | 같음 | 같음 |
| `slam_toolbox` | 같음 | `/amr_01/scan_filtered` | odom TF (Karto) | 없음 | 같음 |
| `liosam_loop` / `liosam_noloop` | amr-ai/liosam:0.1 (08af3f3, GTSAM 4.1.1) | `/amr_01/lidar3d/points` | 없음 | `imu/data_madgwick` | `map → slam_map` = 스폰, `slam_map → amr_01/liosam_odom` = LiDAR 장착 자리 |

- **RTAB-Map 구성 B** (보고서): `icp_odometry` 가 휠-EKF odom 을 초기값으로 받아 `icp_odom → odom` 보정 TF 만 내고
  (`odom → base_footprint` 는 EKF 가 그대로), `rtabmap` 이 `slam_map → icp_odom` 을 낸다. 3D·2D 는 **같은 설정 파일**
  `config/rtabmap.yaml` 을 쓰고 입력 구독(`subscribe_scan_cloud` ↔ `subscribe_scan`)과 리맵만 다르다 —
  `test_config.py::test_rtabmap_launch_2d_3d_share_everything_but_input` 이 이를 고정한다.
- **slam_toolbox** 는 `amr_localization/config/slam_toolbox.yaml` 을 읽기만 한다. 덮어쓰는 것: 노드 `sync_slam_toolbox_node`
  (보고서 '동기'), `scan_queue_size 10`, 프레임 접두어, `map_frame slam_map`, `map_name /amr_01/slam_toolbox/map`.
- **LIO-SAM** 은 우리 작업공간이 없는 이미지라 런치를 경로로 띄운다 (`launch/liosam.launch.py`, 설정 `config/liosam.yaml`).
  상류 run.launch.py 의 rviz·robot_state_publisher·`map→odom` 항등 TF 는 띄우지 않는다.

### 1.1 오라클 차단 (보고서 191–200행)

| 오라클 | 막는 곳 |
|---|---|
| Gazebo IMU 방위 = 정답 ([측정] sensor.md §6) — **`imu/data_raw` 에만 있다** | 모든 SLAM 노드의 IMU 입력은 `imu/data_madgwick` (`use_mag false`, `publish_tf false`). `rtabmap.launch.py check_imu_topic` 이 `imu/data_raw`·`imu/data` 를 거절한다. 재생: 기본 다섯 행은 `rows.ORACLE_TOPICS` 를 틀지 않는다. 변형 행 `rtabmap3d_imulive` 는 `/amr_01/imu/data` 를 madgwick 입력으로 튼다 — 이 토픽의 방위는 **정답이 아니라 항등**이다 (아래 고침) |
| 정답 자세 `ground_truth/odom` | 주행 제어(route_driver)와 평가에만. 어떤 행도 재생하지 않는다 |
| 월드 고정 | 정답이 아니라 **스폰 자세**(18, −16, 1.5708 — 현행 2D 스택의 AMCL 초기 자세와 같은 정보)로 `map → slam_map` 을 준다 |
| 시뮬 휠 (1–2 자릿수 정확) | 이 물결에서는 손대지 않았다 — 휠 사전값을 쓰는 행(RTAB-Map·slam_toolbox)이 유리할 수 있다 (열화 휠은 다음 물결) |

**고침 (b2-offline, 2026-10-06) — `imu/data` 의 방위는 정답이 아니다.** 이 문서의 앞 판은 "이 토픽의 방위 필드는 Gazebo
정답" 이라고 썼다 — **틀렸다.** `imu/data` 는 프로젝트 `imu_filter_node` 출력이고 방위를 항등 쿼터니언으로 채운다
(`src/amr_localization/src/imu_filter_node.cpp` 147–149행 `orientation.w = 1`, `orientation_covariance[0] = −1` = "방위 없음")
[코드]. 파일럿 bag 의 `imu/data` 35 766 개 모두 항등과의 각이 0.0° 이고 yaw 가 55 m·네 번 회전 내내 0 이다 [측정:
`/home/CAPYI_AI/runs/b24/b2-offline/derive/analysis.json` `imu_data_orientation_vs_identity_max_deg`, `tmp/yawcheck.py`].
정답 방위는 Gazebo 원시 `imu/data_raw` 에만 있다 (sensor.md §6 의 측정 대상도 그것이다). 그래서 `rtabmap3d_imulive` 가
`imu/data` 를 틀어도 정답 방위는 새지 않는다 — 아래 두 근거는 그대로 맞지만 필요하지 않았다. 다만 이 월드는 GT roll·pitch 가
정확히 0 이라 항등 방위의 roll·pitch 는 정답과 같다 — `imu/data` 를 SLAM 에 직접 넣으면 평면 사전이 되므로 계속 막는다.
정답 방위가 필요한 상한 행은 `imu/data` 에 같은 스탬프 `imu/data_raw` 의 방위만 옮긴 파생 토픽 `imu/data_gtorient` 를 쓴다
([`replay.md`](replay.md) §1).

앞 판의 설명 (전제가 틀렸다 — 기록으로 남긴다): 이 행은 재생 중 madgwick 을 다시 돌리려고 `/amr_01/imu/data` 를 튼다
(`play_topics`, 실제 기록 `rows/rtabmap3d_imulive/env.txt` "play_topics: … /amr_01/imu/data"). 정답 방위가 SLAM 으로 새지
않는다고 보는 근거는 둘이다.
- SLAM 노드는 madgwick 출력만 구독한다 — `check_imu_topic` 이 강제하고, 실행 로그도 그렇다 [측정: `rows/rtabmap3d_imulive/row.log`
  "Subscribing to IMU topic /amr_01/imu/data_madgwick"]. imu/data 를 받는 노드는 pre_launch 의 madgwick 하나다.
- madgwick 은 입력 방위 필드를 읽지 않는다 [측정, 리뷰 반영 때 추가]: 각속도·가속도가 같고 방위 필드만 다른(항등 ↔ 무작위
  단위 사원수) 입력 600 개를 새 노드에 두 번 넣었더니 출력 방위가 비트 단위로 같았다 (최대 차 0). 대조로 자이로 z 에
  0.3 rad/s 를 더하면 0.44 차이 난다. imu_filter_madgwick 2.1.5 (amr-ai/slam:0.1), 우리 `madgwick.yaml`.
  `logs/agents/s3-rtabmap/review_fix/madg_orient_probe.py`, 셋 다 600/600 받은 두 실행 `madg_orient_probe_{3,4}.log`.
  입력을 잃은 실행은 비교에서 뺐다: 발행 간격 2 ms 판에서 a 가 599 개만 받자 차가 2.2e-4 였다 (`madg_orient_probe_2.log`).
  잃은 입력만큼 필터 상태가 달라지기 때문이다. a·b 가 600/600 이던 실행(`madg_orient_probe.log`, c 599)도 차 0 이었다.

시험: `test_config.py::test_rows_play_recorded_clock_and_never_oracle_topics` 는 `pre_launch` 에 madgwick 이 있는 행에만
`imu/data` 재생을 **허용**한다 (예외를 시험이 명시). `test_oracle_list_covers_known_ground_truth_topics` 가 정답 토픽 목록 자체와
"`ground_truth/odom`·`imu/data_raw` 는 어떤 행도 틀지 않는다" 를 고정 목록으로 단언한다. `test_rtabmap_launch_rejects_oracle_imu`
가 SLAM 노드 쪽 거절을 시험한다.

### 1.2 TF 트리 (재생 때)

```
map ─(anchor_tf: 스폰)→ slam_map ─(rtabmap)→ amr_01/icp_odom ─(icp_odometry)→ amr_01/odom ─(bag: EKF)→ amr_01/base_footprint → …
map ─(anchor_tf: 스폰)→ slam_map ─(slam_toolbox)→ amr_01/odom ─(bag: EKF)→ amr_01/base_footprint → …
map ─(스폰)→ slam_map ─(anchor_tf: base_footprint→lidar3d_link, /tf_static 에서 읽음)→ amr_01/liosam_odom ─(LIO-SAM transformFusion)→ amr_01/base_footprint → …
```

LIO-SAM 행은 bag 의 `/tf`(EKF `odom → base_footprint`)를 **틀지 않는다** — 같이 틀면 `base_footprint` 의 부모가 둘이 된다
(`tftree.check` 가 잡는다, `test_tf_tree_two_parents_invalid`). LIO-SAM mapOptimization 은 `liosam_odom → "lidar_link"`
(상류 하드코딩 이름) 도 내는데, 우리 `amr_01/lidar3d_link` 와 이름이 달라 충돌하지 않는다 [코드: mapOptmization.cpp publishOdometry].

## 2. 설정 근거와 확인한 사실

- RTAB-Map 코어 파라미터(`Grp/Name`)는 **문자열** (s0 VERSIONS.md 1.1 [측정]). `test_rtabmap_core_parameters_are_strings`.
- `icp_odometry` 는 `scan` 과 `scan_cloud` 를 **둘 다** 구독한다 — 기동 로그 "make sure only one of this topic is published,
  otherwise remap one to a dummy topic name" [측정, `logs/agents/s3-rtabmap/probe/icp_odom_start.log`]. 우리 네임스페이스에는
  `/amr_01/scan` 이 실제로 있으므로 안 쓰는 쪽을 `scan_not_used`/`scan_cloud_not_used` 로 돌린다.
- `icp_odometry` ROS 파라미터 이름 (0.23.7, `ros2 param list` [측정]): `topic_queue_size`(기본 1 → 10), `publish_null_when_lost`,
  `deskewing`, `scan_voxel_size`, `scan_normal_k`, `scan_range_min`, `wait_imu_to_init`, `guess_frame_id` 등.
  `Icp/VoxelSize`·`Icp/PointToPlaneK` 는 ROS 파라미터로 옮겨지고 0 이 된다 (기동 경고) → 두 값을 ROS 파라미터로도 준다.
- `imu_filter_madgwick` 기본값은 `use_mag true`, `publish_tf true` [측정, `ros2 param dump`] — 둘 다 끈다.
- LIO-SAM `extrinsicTrans` 는 **LiDAR 프레임에서 본 IMU 위치**다: `imuPreintegration.cpp` 221–222행
  `lidar2Imu = Pose3(I, extTrans)`, 324행 `prevPose_ = lidarPose.compose(lidar2Imu)` [코드]. URDF 로 IMU − LiDAR =
  (0, 0, 0.28) − (0.27, 0, 0.90) = **(−0.27, 0, −0.62)**. 보고서 표의 "lidar3d_link–imu_link 차이" 는 부호가 모호했다.
- LIO-SAM 의 오도메트리 원점 = 출발 때 **LiDAR** 자세 (`transformTobeMapped` 가 LiDAR 자세, 첫 yaw 0 —
  `useImuHeadingInitialization false`) [코드]. 그래서 월드 고정 TF 를 base_footprint 가 아니라 LiDAR 장착 자리만큼 옮긴다.

## 3. 파일럿 기록 (4단계) — `s3_pilot_a` [측정]

- 실행: `record_pilot.sh` (HEAD `a4e15e3`), 이미지 amr-ai/slam:0.1, DOMAIN 208 / 파티션 amr_ai_s3, `gz_slot.sh` 경유.
  산출 `/home/CAPYI_AI/runs/s3-rtabmap/s3_pilot_a/` (bag 3.1 GB mcap — 저장소 밖). HEAD.txt·diff_stat.txt·env.txt 함께.
- 조건: 동적 행위자(작업자 6·지게차·셔틀)를 뺀 월드 사본, amr_01(3D) 1대, localization mode `odom`(휠·imu_filter·
  scan_filter·odom EKF), Nav2·perception 없음. **공유 부하 하**: loadavg 32→48 (32 코어), 기록 구간 RTF **0.587**
  (header 178.70 s / 기록 벽시계 304.6 s).
- 경로 (`config/pilot_route.yaml`, GT 되먹임): (18, −16) → 북 9 m → 서 13 m → 남 7 m → 동 13 m → 북 7 m → 서 6 m
  = 55 m, 정지 20 s 로 시작. 도착 판정 0.05 m, 끝 자세 (12.021, −7.000). RTAB-Map 이 센 오도메트리 길이 54.88 m.
- GT 출발 자세 = (18.000000, −16.000000, yaw 1.570800) — 스폰 자세와 1e-6 m 안에서 같다 → `map → slam_map` 고정이 정확하다.

| 토픽 | 형 | 메시지 | 주기 (header, sim) | 간격 최대 s | 공백(> 1.5 주기) | MB |
|---|---|---|---|---|---|---|
| `/amr_01/lidar3d/points_raw` | PointCloud2 | 1 788 | 10.000 | 0.100 | 0 | 1 648.1 |
| `/amr_01/lidar3d/points` | PointCloud2 | 1 788 | 10.000 | 0.100 | 0 | 1 442.8 |
| `/amr_01/scan` · `scan_filtered` | LaserScan | 1 788 · 1 788 | 10.000 | 0.100 | 0 | 10.4 |
| `/amr_01/imu/data_raw` · `imu/data` | Imu | 35 766 · 35 766 | 200.000 | 0.005 | 0 | 11.9 |
| `/amr_01/imu/data_madgwick` | Imu | **35 738** | 199.843 | **0.140** | **2** (sim 9.48 s 0.14 s, 167.64 s 0.01 s) | 11.9 |
| `/amr_01/wheel_odom` | Odometry | 8 934 | 49.960 | 0.168 | 2 | 6.6 |
| `/amr_01/odometry/filtered` (EKF) | Odometry | 8 934 | 49.957 | 0.100 | 53 | 6.6 |
| `/amr_01/ground_truth/odom` | Odometry | 8 942 | 50.000 | 0.020 | 0 | 6.5 |
| `/amr_01/joint_states` | JointState | 178 584 | 998.6 | 0.019 | 88 | 27.9 |
| `/tf` · `/tf_static` · `/clock` · `/amr_01/cmd_vel` | — | 12 621 · 1 · 178 618 · 3 429 | (header 없음) | | | |

불리한 점: madgwick 출력에 sim 9.48 s 에 0.14 s 공백(28 개)이 있다 — 입력 `imu/data` 는 공백이 없으므로 기록 중
madgwick 쪽에서 잃었다 (route_driver 가 뜬 시각과 겹친다; 원인 미확인). 같은 시각 EKF 출력도 0.065·0.1 s 공백이 있다.
이 공백 때문에 IMU 를 쓰는 행이 그 스캔 하나를 버린다 (§4).
**갱신 (b2-offline)**: 원인을 좁혔다 — 공백 직후 기록 출력과 잠금 단계 재계산 출력의 차가 7 × 10⁻⁵° 에서 1.47° 로 뛴다 → 기록 때
madgwick 이 **입력을 놓쳤다** (기록기만 놓친 것이 아니다). 같은 sim 9.48–9.64 s 에 `/clock` 기록과 EKF TF 도 멈췄다
([`replay.md`](replay.md) §1.2·§7). 파생 bag 의 재계산 madgwick 에는 공백이 없다.

## 4. 매핑 행 재생 결과 (5단계) — **n = 1 (bag 1개 × 재생 1회), 공유 부하 하 예비값** [측정]

> **갱신 (b2-offline, 2026-10-06)**: 같은 bag 을 행마다 3–4 회 다시 재생하고, 모든 행을 같은 SE(2) 지표·공통 시간창으로 잰 표와
> LIO-SAM 평면·절제·오라클 행, IMU 없는 RTAB-Map 행, 처리 스캔 누락 표는 [`replay.md`](replay.md) 에 있다. 아래 n = 1 표의 xy 값은
> 새 도구로 표기 자릿수까지 재현됐고(replay.md §2), 원 bag 재생 3 회의 평균도 0.77 / 2.19 / 5.62 / 5.29 cm 로 이 표와 0.4 cm 안이다.

재생: `replay_row.sh <행> bag rows/<행> 0.5` (배속 0.5, 기록된 /clock), 컨테이너마다 `ROS_LOCALHOST_ONLY=1`,
`--shm-size=2g` + Fast DDS SHM 16 MB 프로파일. 재생 중 loadavg 21–110 (다른 에이전트 작업). 산출 `rows/<행>/`.
실행 HEAD 는 행마다 다르다 (`rows/HEAD_<행>.txt`·`diff_<행>.txt`): rtabmap3d·slam_toolbox·liosam_loop = `6170e85`,
rtabmap2d·liosam_noloop = `df91c97`, rtabmap3d_imulive = `df91c97` + 미커밋 diff 3 파일 (`diff --stat` 줄 수가 `cb4aac5` 의 같은 3 파일과 같다 — 내용 대조는 못 했다. `cb4aac5` 는 여기에 `bag_tools.py` 독스트링 3 줄을 더 담는다).
행 런치는 깨끗하게 끝나지 않았다: `rows/host_*.log` 6 개 모두 "Killed … ros2 launch" (SIGINT 처리 줄 없음 → 20 s 뒤 SIGKILL 로
보인다). 그래서 RTAB-Map DB 에 최적화 자세가 저장되지 않았다 (`db_info.txt` "Optimized graph: 0 poses").

### 4.1 기동·통과 기준

| 행 | 기동 | "Failed to deskew" | 처리 스캔 (1 788 중) | TF 트리 | 지도 원점 = 월드 | 2D 격자 | 3D 점 지도 |
|---|---|---|---|---|---|---|---|
| rtabmap3d | 성공 | 0 | 1 787 (누락 1: sim 9.5 s — IMU 공백, 로그 "didn't receive IMU newer … dropped") | 유효 (부모 하나, 뿌리 map) | 예 (ATE 정렬 없음 0.77 cm) | `map_world.yaml` 0.05 m | `cloud_world.ply` 101 만 점 |
| rtabmap2d | 성공 | 0 | 1 787 (누락 1: 같은 9.5 s) | 유효 | 예 (2.16 cm) | `map_world.yaml` | 해당 없음 (2D 입력) |
| slam_toolbox | 성공 | 해당 없음 | 처리 수 출력 없음 (`Message Filter dropping` 0) | 유효 | 예 (1.74 cm) | `map_world.yaml` | 해당 없음 |
| liosam_loop | 성공 | 해당 없음 ("Point cloud timestamp not available" 1 — 계약대로) | imageProjection 1 785 (누락 3: 9.6 s madgwick 공백 1 장 + 182.4·182.5 s = bag 의 마지막 두 장 — 2 장 캐시 구조, 아래). 끝 2 장을 뺀 판정: 1 785 / 1 786, 누락 1. mapOptimization 893 (0.15 s 간격 설계) | 유효 | 예 (5.27 cm) | **없음** (LIO-SAM 은 격자를 내지 않는다 — 미완) | `cloud_world.ply` (특징점 GlobalMap 63 만 점) |
| liosam_noloop | 성공 | 같음 | 같음 (1 785 / 893) | 유효 | 예 (5.32 cm) | 없음 | `cloud_world.ply` 63 만 점 |
| rtabmap3d_imulive (변형) | 성공 | 0 | 1 786 (누락 2: 9.6 s, 154.5 s — IMU 버림 0) | 유효 | 예 (0.77 cm [계산, 리뷰 반영 때 numpy 재계산 — `eval/results.csv` 에는 없음]) | 있음 | 있음 |

판정: **"처리 스캔 누락 0" 은 미달**이다 — RTAB-Map 1/1 788, LIO-SAM 3/1 788 (아래 계수 규칙으로 1/1 786).
**slam_toolbox 는 처리 수를 재지 못해(미측정) 이 판정에서 빠졌다** — 동기 모드의 최소 이동 필터 때문에 처리 수가 1 788 과
같을 수도 없으므로 "누락 0" 을 같은 방식으로 판정할 수 없다.

누락 원인 (행마다 근거를 따로 적는다):
- **sim 9.5–9.6 s 1 장 (RTAB-Map·LIO-SAM)** — 기록 bag 의 madgwick 출력 공백(9.48 s 부터 0.14 s, §3). RTAB-Map 로그
  "didn't receive IMU newer … (9.500000)" 1 줄, LIO-SAM 로그 "Waiting for IMU data" 1 줄 (`log_scan.json`) [측정].
  bag 의 madgwick 스탬프는 9.48 s 다음이 9.62 s 다 [측정: `logs/agents/s3-rtabmap/review_fix/stamps_near.log`, rosbag2_py].
  LIO-SAM 은 9.5 s 장을 처리하며 IMU 큐에서 9.49 s 미만을 잘라 내므로(349–350행) 다음 9.6 s 장 때 큐 맨 앞이 9.62 s 가 되어
  `deskewInfo` 의 `imuQueue.front() > timeScanCur` 조건(328–330행)에 걸린다 [코드: imageProjection.cpp].
- **sim 182.4·182.5 s 2 장 (LIO-SAM 만)** — bag 의 **마지막 두 장**이다. 원인은 IMU 가 아니라 LIO-SAM imageProjection 의
  2 장 캐시다 [코드: LIO-SAM 08af3f3 `src/imageProjection.cpp` 228–230행 `cachePointCloud`]:
  ```cpp
  cloudQueue.push_back(*laserCloudMsg);
  if (cloudQueue.size() <= 2)
      return false;
  ```
  새 점군이 올 때마다 큐 맨 앞 것을 처리하므로 **어느 bag 이든 마지막 두 장은 뒤에 오는 점군이 없어 처리되지 않는다.**
  IMU 는 원인이 아니다: bag 의 `imu/data_madgwick` 마지막 스탬프 182.6 s 가 점군 마지막 182.5 s 보다 늦고
  (`bag_meta.json` header last, `stamps_near.log` "cloud last 4: [182.2, 182.3, 182.4, 182.5]"), 두 장에 대한
  "Waiting for IMU data" 줄도 없다 (전체 1 줄 = 9.6 s 장). 그래서 IMU 를 더 길게 기록해도 이 두 장은 그대로 빠진다.
  **고침**: 이 문서의 앞 판은 이 두 장을 "IMU 가 스캔 뒤로 없음" 이라 적고 원인을 "특정했다" 고 썼다 — 틀렸다 (리뷰 B1).
- **rtabmap3d_imulive 의 9.6 s·154.5 s** — madgwick 을 재생 중 다시 돌린 변형은 IMU 버림이 0 이 됐지만 두 장을 놓쳤다.
  9.6 s 는 같은 시각 EKF TF 공백(§3)과 겹치고, 154.5 s 는 **원인 미확인**이다.
  → 9.6 s 는 휠-EKF TF guess 조회 실패로 확인했다: 첫 15 s 재생에서 guess 를 끄면 처리되고, `wait_for_transform` 1.0 s 로는 안 풀린다
  ([`replay.md`](replay.md) §7). IMU 를 빼거나 공백 없는 IMU 를 준 RTAB-Map 행 10 재생 모두 이 장 하나를 놓쳤다.

**LIO-SAM 계수 규칙.** 지금 계수(입력 1 788 장 전부를 분모로)로는 LIO-SAM 이 "누락 0" 을 **구조적으로 달성할 수 없다.**
그래서 LIO-SAM 행은 **bag 의 마지막 2 장을 분모에서 뺀다** (1 788 → 1 786). 이 규칙으로 LIO-SAM 누락은 1/1 786 (9.6 s) 이고
여전히 **미달**이다. 다른 선택지(재생 끝에 점군 2 장을 더 흘려 캐시를 비우기)는 §7 에 넘긴다. 이 규칙은 아직 문서 규칙일 뿐
코드(`bag_tools match`)에는 들어가지 않았다 — `match_*.json` 의 `missing 3` 은 옛 계수 그대로다.

나머지 셋(deskew 실패 없음, TF 트리 유효, 원점 = 월드)은 모든 행에서 통과.

처리 수를 세는 법 — 측정 경로마다 수가 달랐다 (부하 하): rtabmap3d v2 재생에서 icp_odometry 로그 1 785 줄,
Python 측정 노드 1 607 개, C++ 기록기 1 586 개 — RTAB-Map odom 발행자가 reliable keep_last 1 이라 구독자가 늦으면 덮인다.
SHM 프로파일을 준 v3 에서는 로그 1 787 = 기록기 1 787 로 맞았다. 그래서 1 차 근거는 노드 로그 줄 수, 2 차는 C++ 기록기다.

### 4.2 궤적 (amr_evaluation.slam3d `traj`, 기준 = bag 의 ground_truth/odom) [측정·계산]

| 행 | 정렬 없음 RMSE / max | SE(2) 정렬 RMSE | 정지 / 직선 / 회전 RMSE (max) cm | RPE 1 m / 10 m |
|---|---|---|---|---|
| rtabmap3d (온라인 TF) | **0.77 / 1.72 cm** | 0.57 cm | 0.68 (1.39) / 0.79 (1.72) / 0.83 (1.56) | 0.40 % / 0.085 % |
| rtabmap3d (export 재최적화 그래프 138 자세) | 0.99 / 1.54 cm | 0.51 cm | | 0.31 % / 0.078 % |
| rtabmap2d (온라인) | 2.16 / 5.70 cm | 2.04 cm | 1.53 (3.70) / 2.31 (4.97) / 2.41 (5.70) | 1.53 % / 0.32 % |
| rtabmap2d (export 재최적화 그래프) | 2.55 / 5.32 cm | 1.93 cm | | 1.26 % / 0.29 % |
| slam_toolbox (온라인) | 1.74 / 3.64 cm | 0.97 cm | 1.28 (2.74) / 1.86 (3.64) / 1.88 (2.76) | 1.17 % / 0.24 % |
| liosam_loop (온라인) | 5.27 / 9.23 cm | 2.76 cm | 3.73 (8.01) / 5.66 (9.23) / 5.76 (8.97) | 1.46 % / 0.31 % |
| liosam_noloop (온라인) | 5.32 / 9.21 cm | 2.83 cm | 4.05 (8.22) / 5.64 (9.21) / 5.81 (9.07) | 1.43 % / 0.32 % |

읽는 법과 한계:
- **LIO-SAM 은 수직으로 떠돈다 (불리한 사실, 리뷰 반영 때 추가).** 위 표의 ATE 는 **xy 만**이다. 평평한 바닥(GT z = 0)에서
  `traj_online` 의 base_footprint z 가 최대 0.260 m (loop) / 0.264 m (noloop), 둘 다 t = 81.5 s 까지 오른다. xyz ATE 는
  RMSE 12.79 / 12.66 cm, max 26.93 / 27.48 cm 다. 점 지도도 z 최소 −0.374 / −0.383 m, 바닥 아래(z < −0.05 m) 점이
  3.9 % / 4.2 % 다 (RTAB-Map 3D 는 z 최소 −0.038 m, 0 %). §4.3 의 LIO-SAM 3D F-score 가 낮은 원인 후보다 [계산: numpy,
  gt.tum 선형 보간·`cloud_world.ply` 직접 읽기]. RTAB-Map 행은 `Reg/Force3DoF` 라 z 가 0 으로 묶인다.
- **"온라인 TF" 는 엄밀한 온라인이 아니다.** `slam_probe.py` 는 입력 스탬프의 `map → base_footprint` 를 그 스탬프보다
  `--lookup-delay` 1 s(sim) 늦게 조회한다. 그 1 s 사이에 나온 map 보정 TF 가 보간에 섞인다.
- **"export 재최적화 그래프" 는 rtabmap 이 남긴 그래프가 아니다.** DB 에 최적화 자세가 없어(위 SIGKILL) `rtabmap-export` 가
  전체 전역 최적화를 다시 돌린 결과다 (`export_poses.log` "no saved optimized poses … full global optimization", poses=138,
  links=307; DB 의 전역 그래프는 139 자세·310 링크).
- **차원 효과로 읽지 마라.** bag 1개·재생 1회라 재생 간 산포를 모른다 (보고서는 구성당 표본 ≥ 10). RTAB-Map 3D·2D 와
  slam_toolbox 는 **시뮬 휠-EKF 사전값**(오라클 수준, 보고서 5장)을 받는다 — LIO-SAM 은 휠을 쓰지 않는다. 그래서 LIO-SAM 이
  나쁜 것이 3D LIO 의 한계인지, 휠 사전값 차이인지 이 표로는 가를 수 없다.
- LIO-SAM 루프 on/off 가 거의 같다 — 루프 폐쇄가 실제로 일어났는지는 **확인하지 않았다** (loop_closure_constraints 를 기록하지 않음).
- RTAB-Map 은 LiDAR 만이라 전역 루프(BoW)가 없고 근접 루프만 있다 (DB: rtabmap3d LocalSpaceClosure 33·GlobalClosure 0,
  rtabmap2d 45·0 — 2D 는 최대 6.99 m 짜리 근접 링크가 있다, 오루프인지 미확인).

### 4.3 지도 (같은 도구 `grid`·`cloud`, GT = gen_warehouse_world.py --gt-dir 1 cm) [계산]

2D 격자 — GT 0.20 m 단면 (0.05–0.40 m 대역과 이 월드에서 같은 래스터, s2 evaluation.md §0-4):

| 행 | ADNN₀ surface / filled | 정밀도 / 재현율 (surface) | F1 | IoU (공통 0.05) | 점유 셀 |
|---|---|---|---|---|---|
| rtabmap3d (0.05–0.40 대역) | 4.30 / 1.16 cm | 0.624 / 0.650 | 0.637 | 0.471 | 4 864 |
| rtabmap2d | 4.55 / 1.14 cm | 0.546 / 0.602 | 0.572 | 0.579 | 4 918 |
| slam_toolbox | 4.62 / 2.78 cm | 0.634 / 0.905 | 0.746 | 0.548 | 6 640 |

3D 점 지도 — GT 표면 점 전체(all_gt)와 파일럿 지역 ROI x 0–30, y −20–0 (`roi_pilot_area` — GT 를 깎은 조작):

| 행 | F@5 cm (P / R) | F@10 cm | AC@5 cm |
|---|---|---|---|
| rtabmap3d, ROI | 0.689 (0.948 / 0.541) | 0.750 | 1.1 cm |
| liosam_loop, ROI | 0.263 (0.488 / 0.180) | 0.473 | 2.3 cm |
| liosam_noloop, ROI | 0.267 (0.490 / 0.183) | 0.478 | 2.2 cm |

LIO-SAM 지도는 특징점(모서리·면)만 모은 GlobalMap 이라 전체 스캔을 붙인 RTAB-Map 지도와 밀도가 다르다 — 재현율 비교는 공정하지 않다.
두 지도 모두 벽 밖 점이 0.07 % 있다 (벽 개구부 너머 — 정렬 문제 아님).

## 5. 재현

산출물 경로 규약 (행마다 `rows/<행>/`): `probe.json`(입력·처리 수·TF 트리·고정 TF), `traj_online.tum`(입력 스탬프마다
map → base_footprint), `traj_final.tum`(RTAB-Map DB 를 `rtabmap-export` 가 재최적화한 그래프, 월드), `map_world.{pgm,yaml}`(월드 정렬 격자),
`cloud_world.ply`(월드 점 지도), `match_*.json`(처리 대응), `log_scan.json`(로그 판정), `out_bag/`(처리 출력·TF), `row.log`.

호스트에서 (공유 서버 규칙: Gazebo 는 gz_slot, 컨테이너마다 `ROS_LOCALHOST_ONLY=1` 로 DDS 격리):

```bash
WT=/home/CAPYI_AI/wt/s3-rtabmap
docker run --rm --name amr-ai-s3-rtabmap-build -v $WT:/ros2_ws -w /ros2_ws amr-ai/slam:0.1 \
  bash -lc 'MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 4 --packages-up-to amr_slam3d'
# 4단계: 파일럿 bag (출력 디렉터리에 bag/, bag_meta.json, gt.tum, route.jsonl, launch.log …)
/home/CAPYI_AI/env/gz_slot.sh s3-rtabmap docker run --rm --name amr-ai-s3-rtabmap-gz --gpus all --shm-size=8g \
  -e ROS_DOMAIN_ID=208 -e IGN_PARTITION=amr_ai_s3 -e ROS_LOCALHOST_ONLY=1 \
  -v $WT:/ros2_ws -v <OUT>:/out -w /ros2_ws amr-ai/slam:0.1 \
  bash -lc '/ros2_ws/install/amr_slam3d/lib/amr_slam3d/record_pilot.sh /out'
# 5단계: 행마다 (rtabmap3d | rtabmap2d | slam_toolbox | rtabmap3d_imulive → amr-ai/slam:0.1,
#        liosam_loop | liosam_noloop → amr-ai/liosam:0.1). 컨테이너마다 DDS 가 격리되므로 동시에 여러 행을 돌려도 된다
docker run --rm --name amr-ai-s3-rtabmap-<행> --shm-size=2g -e ROS_DOMAIN_ID=208 -e ROS_LOCALHOST_ONLY=1 \
  -v $WT:/ros2_ws -v <OUT>:/data -w /ros2_ws <이미지> \
  bash -lc '/ros2_ws/src/amr_slam3d/scripts/replay_row.sh <행> /data/bag /data/rows/<행> 0.5'
# 지표 (s2 의 amr_evaluation.slam3d, GT 는 gen_warehouse_world.py --gt-dir)
python3 -m amr_evaluation.slam3d.cli traj --gt <OUT>/gt.tum --est rows/<행>/traj_online.tum --by-motion …
```

## 6. 보고서와 다르게 한 것

| 항목 | 보고서 | 여기 | 이유·근거 |
|---|---|---|---|
| 재생 시간원 | 기록된 /clock 은 빼고 `--clock 200` | **기록된 /clock 을 튼다**, `--clock` 안 씀 | Humble 기록기는 수신 벽시계를 메시지 시각으로 남겨 `--clock` 이 1.79e9 s 를 낸다 [측정: 첫 재생 row.log "earliest data is at time 1791240775"] |
| 2D 입력 | scan | `scan_filtered` | slam_toolbox 행(현행 2D 스택)과 같은 입력 — 2D 행끼리 입력을 같게 |
| IMU | RTAB-Map `imu_topic` 비움(기본), 쓸 때 madgwick | 3D·2D 모두 madgwick 사용 (`wait_imu_to_init`) | 과제 지시 "icp_odometry + madgwick". `Reg/Force3DoF` 라 영향은 중력 정렬뿐 (DB Gravity 링크 139 — 앞 판의 137 은 오기) |
| LIO-SAM `extrinsicTrans` | "lidar3d_link–imu_link 차이" (부호 모호) | IMU − LiDAR = (−0.27, 0, −0.62) | imuPreintegration.cpp 221–222·324행 [코드] |
| LIO-SAM 복셀 | 표에 없음 | 상류 주석의 실내 값 (0.2 / 0.1 / 0.2) | 상류 params.yaml 주석 "indoor" |
| 3D 점 지도 (RTAB-Map) | `rtabmap-export` | `rtabmap-export` 가 재최적화한 그래프 자세(§4.2)에 bag 점군을 붙임 (`bag_tools assemble`, 복셀 0.05) | `rtabmap-export --cloud --scan` 이 0 점 [측정 v1 export_cloud.log] — 원인 미확인 |
| 운용 대역 격자 (0.05–2.0 m) | 비교·운용 두 대역 | **비교 대역만** | 이 월드에서 두 대역의 차이는 표지판 2 356 셀(0.25 %)뿐 (s2 evaluation.md §0-4) — 미완으로 둔다 |
| slam_toolbox | 동기 | 동기 + `scan_queue_size 10`, YAML 은 읽기만 | |

## 7. 다음 물결에 넘길 것

- 표본: 씨드·경로가 다른 bag 3–5 개 × 재생 3–5 회 (보고서 ≥ 10/구성). 지금은 n = 1 — 행 간 차이를 주장할 수 없다. **b2-offline**: 같은 bag 재생 3–4 회는 했다 (replay.md §5) — bag 은 여전히 1 개.
- 기록: madgwick 을 기록 전에 충분히 띄우고 공백 검사를 기록 직후 자동 판정 (`bag_meta.json` gaps > 0 이면 재기록).
- **(b2-offline 에서 (a) 로 코드화 — `rows.LIOSAM_TAIL`, `bagstat.matched(tail_exempt=)`)** **LIO-SAM 처리 수 계수 규칙을 코드로** (리뷰 B1): imageProjection 의 2 장 캐시(`imageProjection.cpp` 228–230행) 때문에 bag 의
  마지막 2 장은 어떤 기록에서도 처리되지 않는다. 둘 중 하나를 정해 `bag_tools match`·판정에 넣는다.
  (a) LIO-SAM 행은 분모에서 끝 2 장을 뺀다 (이 문서의 현재 규칙, 문서에만 있음).
  (b) 재생 끝에 점군 2 장(마지막 장 복제, 스탬프 +0.1·+0.2 s)을 더 흘려 캐시를 비운다 — 이때 IMU 는 마지막으로 처리할 장의
  `timeScanEnd` 뒤까지만 있으면 된다 (지금 bag 은 182.6 s 까지 있어 충분). **IMU 를 더 길게 기록하는 것은 효과가 없다.**
  9.6 s 장(madgwick 공백)은 어느 규칙으로도 누락으로 남는다.
- **(b2-offline 에서 고침, `2126897` — 진단은 replay.md §7)** 재생 정리: `replay_row.sh` 가 행 런치를 SIGINT 로 못 끈다 (비대화형 bash 의 백그라운드 작업은 SIGINT 를 무시한 채 시작한다는
  리뷰 진단, 미검증). SIGTERM 을 쓰거나 SIGINT 처분을 복구해 RTAB-Map 이 최적화 자세를 DB 에 저장하게 한다.
- LIO-SAM: 2D 격자(octomap_server projected_map 또는 대역 투영), 루프 폐쇄 발생 여부(loop_closure_constraints 기록),
  ATE 가 RTAB-Map 보다 큰 원인(휠 사전값 없음 / madgwick 초기 기울기 / extrinsicTrans) 분리.
- 요인 B(guess 없음), 열화 휠(요인 D), 오라클 행(정답 IMU 방위) — `rtabmap.launch.py use_guess:=false` 는 준비돼 있다.
- RTAB-Map 2D 의 6.99 m 근접 링크가 오루프인지 GT 로 판정.

