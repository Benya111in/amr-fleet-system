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
| Gazebo IMU 방위 = 정답 ([측정] sensor.md §6) | 모든 SLAM 행 IMU 는 `imu/data_madgwick` (`use_mag false`, `publish_tf false`). `rtabmap.launch.py check_imu_topic` 이 `imu/data_raw`·`imu/data` 를 거절, `rows.ORACLE_TOPICS` 는 어떤 행도 재생하지 않는다 (`test_rows_never_play_oracle_topics_or_clock`) |
| 정답 자세 `ground_truth/odom` | 주행 제어(route_driver)와 평가에만. 재생하지 않는다 |
| 월드 고정 | 정답이 아니라 **스폰 자세**(18, −16, 1.5708 — 현행 2D 스택의 AMCL 초기 자세와 같은 정보)로 `map → slam_map` 을 준다 |
| 시뮬 휠 (1–2 자릿수 정확) | 이 물결에서는 손대지 않았다 — 휠 사전값을 쓰는 행(RTAB-Map·slam_toolbox)이 유리할 수 있다 (열화 휠은 다음 물결) |

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

## 4. 매핑 행 재생 결과 (5단계) — **n = 1 (bag 1개 × 재생 1회), 공유 부하 하 예비값** [측정]

재생: `replay_row.sh <행> bag rows/<행> 0.5` (배속 0.5, 기록된 /clock), 컨테이너마다 `ROS_LOCALHOST_ONLY=1`,
`--shm-size=2g` + Fast DDS SHM 16 MB 프로파일. 재생 중 loadavg 21–110 (다른 에이전트 작업). 산출 `rows/<행>/`.

### 4.1 기동·통과 기준

| 행 | 기동 | "Failed to deskew" | 처리 스캔 (1 788 중) | TF 트리 | 지도 원점 = 월드 | 2D 격자 | 3D 점 지도 |
|---|---|---|---|---|---|---|---|
| rtabmap3d | 성공 | 0 | 1 787 (누락 1: sim 9.5 s — IMU 공백, 로그 "didn't receive IMU newer … dropped") | 유효 (부모 하나, 뿌리 map) | 예 (ATE 정렬 없음 0.77 cm) | `map_world.yaml` 0.05 m | `cloud_world.ply` 101 만 점 |
| rtabmap2d | 성공 | 0 | 1 787 (누락 1: 같은 9.5 s) | 유효 | 예 (2.16 cm) | `map_world.yaml` | 해당 없음 (2D 입력) |
| slam_toolbox | 성공 | 해당 없음 | 처리 수 출력 없음 (`Message Filter dropping` 0) | 유효 | 예 (1.74 cm) | `map_world.yaml` | 해당 없음 |
| liosam_loop | 성공 | 해당 없음 ("Point cloud timestamp not available" 1 — 계약대로) | imageProjection 1 785 (누락 3: 9.6 s IMU 공백, 182.4·182.5 s bag 끝 — IMU 가 스캔 뒤로 없음). mapOptimization 893 (0.15 s 간격 설계) | 유효 | 예 (5.27 cm) | **없음** (LIO-SAM 은 격자를 내지 않는다 — 미완) | `cloud_world.ply` (특징점 GlobalMap 63 만 점) |
| liosam_noloop | 성공 | 같음 | 같음 (1 785 / 893) | 유효 | 예 (5.32 cm) | 없음 | `cloud_world.ply` 63 만 점 |
| rtabmap3d_imulive (변형) | 성공 | 0 | 1 786 (누락 2: 9.6 s, 154.5 s — IMU 버림 0) | 유효 | 예 | 있음 | 있음 |

판정: **"처리 스캔 누락 0" 은 미달**이다 — RTAB-Map 1/1 788, LIO-SAM 3/1 788. 원인은 기록 bag 의 madgwick 공백
(9.5–9.6 s)과 bag 끝 처리(LIO-SAM)로 특정했다. madgwick 을 재생 중 다시 돌린 변형(rtabmap3d_imulive)은 IMU 버림이
0 이 됐지만 9.6 s(같은 시각 EKF TF 공백과 겹침)·154.5 s(원인 미확인) 두 장을 놓쳤다.
나머지 셋(deskew 실패 없음, TF 트리 유효, 원점 = 월드)은 모든 행에서 통과.

처리 수를 세는 법 — 측정 경로마다 수가 달랐다 (부하 하): rtabmap3d v2 재생에서 icp_odometry 로그 1 785 줄,
Python 측정 노드 1 607 개, C++ 기록기 1 586 개 — RTAB-Map odom 발행자가 reliable keep_last 1 이라 구독자가 늦으면 덮인다.
SHM 프로파일을 준 v3 에서는 로그 1 787 = 기록기 1 787 로 맞았다. 그래서 1 차 근거는 노드 로그 줄 수, 2 차는 C++ 기록기다.

### 4.2 궤적 (amr_evaluation.slam3d `traj`, 기준 = bag 의 ground_truth/odom) [측정·계산]

| 행 | 정렬 없음 RMSE / max | SE(2) 정렬 RMSE | 정지 / 직선 / 회전 RMSE (max) cm | RPE 1 m / 10 m |
|---|---|---|---|---|
| rtabmap3d (온라인 TF) | **0.77 / 1.72 cm** | 0.57 cm | 0.68 (1.39) / 0.79 (1.72) / 0.83 (1.56) | 0.40 % / 0.085 % |
| rtabmap3d (최종 그래프 138 노드) | 0.99 / 1.54 cm | 0.51 cm | | 0.31 % / 0.078 % |
| rtabmap2d (온라인) | 2.16 / 5.70 cm | 2.04 cm | 1.53 (3.70) / 2.31 (4.97) / 2.41 (5.70) | 1.53 % / 0.32 % |
| rtabmap2d (최종 그래프) | 2.55 / 5.32 cm | 1.93 cm | | 1.26 % / 0.29 % |
| slam_toolbox (온라인) | 1.74 / 3.64 cm | 0.97 cm | 1.28 (2.74) / 1.86 (3.64) / 1.88 (2.76) | 1.17 % / 0.24 % |
| liosam_loop (온라인) | 5.27 / 9.23 cm | 2.76 cm | 3.73 (8.01) / 5.66 (9.23) / 5.76 (8.97) | 1.46 % / 0.31 % |
| liosam_noloop (온라인) | 5.32 / 9.21 cm | 2.83 cm | 4.05 (8.22) / 5.64 (9.21) / 5.81 (9.07) | 1.43 % / 0.32 % |

읽는 법과 한계:
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
map → base_footprint), `traj_final.tum`(RTAB-Map 최종 그래프, 월드), `map_world.{pgm,yaml}`(월드 정렬 격자),
`cloud_world.ply`(월드 점 지도), `match_*.json`(처리 대응), `log_scan.json`(로그 판정), `out_bag/`(처리 출력·TF), `row.log`.

호스트에서 (공유 서버 규칙: Gazebo 는 gz_slot, 컨테이너마다 `ROS_LOCALHOST_ONLY=1` 로 DDS 격리):

```bash
WT=/home/CAPYI_AI/wt/s3-rtabmap
docker run --rm --name amr-ai-s3-rtabmap-build -v $WT:/ros2_ws -w /ros2_ws amr-ai/slam:0.1 \
  bash -lc 'MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 4'
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
| IMU | RTAB-Map `imu_topic` 비움(기본), 쓸 때 madgwick | 3D·2D 모두 madgwick 사용 (`wait_imu_to_init`) | 과제 지시 "icp_odometry + madgwick". `Reg/Force3DoF` 라 영향은 중력 정렬뿐 (DB Gravity 링크 137) |
| LIO-SAM `extrinsicTrans` | "lidar3d_link–imu_link 차이" (부호 모호) | IMU − LiDAR = (−0.27, 0, −0.62) | imuPreintegration.cpp 221–222·324행 [코드] |
| LIO-SAM 복셀 | 표에 없음 | 상류 주석의 실내 값 (0.2 / 0.1 / 0.2) | 상류 params.yaml 주석 "indoor" |
| 3D 점 지도 (RTAB-Map) | `rtabmap-export` | 최종 그래프 자세에 bag 점군을 붙임 (`bag_tools assemble`, 복셀 0.05) | `rtabmap-export --cloud --scan` 이 0 점 [측정 v1 export_cloud.log] — 원인 미확인 |
| 운용 대역 격자 (0.05–2.0 m) | 비교·운용 두 대역 | **비교 대역만** | 이 월드에서 두 대역의 차이는 표지판 2 356 셀(0.25 %)뿐 (s2 evaluation.md §0-4) — 미완으로 둔다 |
| slam_toolbox | 동기 | 동기 + `scan_queue_size 10`, YAML 은 읽기만 | |

## 7. 다음 물결에 넘길 것

- 표본: 씨드·경로가 다른 bag 3–5 개 × 재생 3–5 회 (보고서 ≥ 10/구성). 지금은 n = 1 — 행 간 차이를 주장할 수 없다.
- 기록: madgwick 을 기록 전에 충분히 띄우고 공백 검사를 기록 직후 자동 판정 (`bag_meta.json` gaps > 0 이면 재기록).
- LIO-SAM: 2D 격자(octomap_server projected_map 또는 대역 투영), 루프 폐쇄 발생 여부(loop_closure_constraints 기록),
  ATE 가 RTAB-Map 보다 큰 원인(휠 사전값 없음 / madgwick 초기 기울기 / extrinsicTrans) 분리.
- 요인 B(guess 없음), 열화 휠(요인 D), 오라클 행(정답 IMU 방위) — `rtabmap.launch.py use_guess:=false` 는 준비돼 있다.
- RTAB-Map 2D 의 6.99 m 근접 링크가 오루프인지 GT 로 판정.

