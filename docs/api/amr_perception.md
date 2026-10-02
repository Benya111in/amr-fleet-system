# API — `amr_perception`

> 명세 4장 5·6·7절 (딥러닝 인식 / 2D→3D / 동적 장애물 추적·안전). [← 색인](README.md)
> 알고리즘·튜닝 근거: [../algorithms/perception.md](../algorithms/perception.md),
> [../algorithms/tracking.md](../algorithms/tracking.md).
> 런치: `src/amr_perception/launch/perception.launch.py` — 7개 노드를 모두
> 로봇 네임스페이스(`/amr_01`) 아래 `namespace=robot` 로 띄우고, 노드 이름 = 실행 파일 이름이다 (`:51-55`).
> 아래 토픽은 전부 **상대 이름**이다. 절대 이름은 `/map`, `/fleet/estop` 둘뿐이다.
> **설정 파일**: `src/amr_perception/config/perception.yaml` (노드별 `/**/​<노드 이름>:` 블록).
> 런치가 그 앞에 `config/robot_params.yaml`, `config/sensors.yaml` 을 함께 넘긴다 (`:42-44`).

## 0. 이 패키지의 노드

| 노드 | 언어 | 소스 | 런치 플래그 | perception.yaml 블록 |
| --- | --- | --- | --- | --- |
| `yolo_node` | Python | `amr_perception/yolo_node.py` | `use_yolo` | `:12` |
| `object_localizer_node` | Python | `amr_perception/object_localizer_node.py` | `use_localizer` | `:38` |
| `detection_marker_node` | Python | `amr_perception/detection_marker_node.py` | `use_markers` | `:69` |
| `aruco_detector_node` | Python | `amr_perception/aruco_detector_node.py` | `use_aruco` | `:75` |
| `pointcloud_filter_node` | C++ | `src/pointcloud_filter_node.cpp` | `use_pointcloud` | `:106` |
| `obstacle_tracker_node` | C++ | `src/obstacle_tracker_node.cpp` | `use_tracker` | `:115` |
| `safety_node` | C++ | `src/safety_node.cpp` | `use_safety` | `:206` |

(소스 경로는 `src/amr_perception/` 기준. 런치 행은 `perception.launch.py:59-65`.)

### 데이터 흐름

```
camera/image_raw ─► yolo_node ─► perception/detections_2d ─┐
camera/depth/image_raw ───────────────────────────────────┴─► object_localizer_node
                                                                 │
                                        perception/detected_objects ─► detection_marker_node
                                                                    └► task_executor_node

camera/depth/image_raw ─► pointcloud_filter_node ─► camera/depth/points_filtered ─► safety_node
scan_filtered ─► obstacle_tracker_node ─► perception/tracked_obstacles ─► safety_node, DWA, BT
camera/image_raw ─► aruco_detector_node ─► perception/dock_marker_pose ─► docking_server_node

cmd_vel_smoothed ─► safety_node ─► cmd_vel          (cmd_vel 의 유일한 발행자)
```

---

## 1. `yolo_node`

- **소스**: `src/amr_perception/amr_perception/yolo_node.py` · Python
  (진입점 `src/amr_perception/scripts/yolo_node`)
- **역할**: `camera/image_raw` → YOLOv8 추론 → `perception/detections_2d` (`vision_msgs/Detection2DArray`).
  2단 파이프라인(수신 스레드 / 추론 스레드, 최신 1장만 유지)은 `async_inference` 로 켠다 (`:154-160`).

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 주기 | 근거 |
| --- | --- | --- | --- | --- | --- |
| `perception/detections_2d` | `topics.detections` | `vision_msgs/Detection2DArray` | 기본 (10) | 입력 이미지마다 (콜백 구동) | `:108`, `:145` |
| `perception/detections_image` | `topics.annotated` | `sensor_msgs/Image` | depth 1 | 위와 같음 | `:109`, `:146-147` — `publish_annotated:=true` 일 때만 |

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `camera/image_raw` | `topics.image` | `sensor_msgs/Image` | BEST_EFFORT, depth 1, KEEP_LAST (`:143-144`) | `:107`, `:148-149` |

서비스·액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `weights` | string | `yolov8n_warehouse.pt` | 90 | 가중치 파일 (창고 월드 학습본) |
| `fallback_weights` | string | `yolov8n.pt` | 91 | 위 파일이 없을 때 대체 (COCO — 상자·표지판을 못 찾는다) |
| `device` | string | `auto` | 92 | `auto` \| `cuda` \| `cpu` |
| `imgsz` | int | `640` | 93 | GPU 추론 입력 변 길이 |
| `cpu_imgsz` | int | `320` | 94 | CPU 추론 입력 변 길이 |
| `conf_thresh` | double | `0.35` | 95 | 검출 신뢰도 문턱 |
| `iou` | double | `0.5` | 96 | NMS IoU 문턱 |
| `half` | bool | `True` | 97 | FP16 추론 |
| `max_det` | int | `50` | 98 | 프레임당 최대 검출 수 |
| `torch_threads` | int | `2` | 99 | torch 스레드 수 |
| `cpu_threads` | int | `2` | 100 | CPU 백엔드 스레드 수 |
| `fast_path` | bool | `True` | 101 | 전처리 빠른 경로 |
| `async_inference` | bool | `False` | 102 | 수신/추론 분리 스레드 |
| `classes_file` | string | `''` | 103 | 클래스 매핑 yaml (`''` = 패키지 `config/classes.yaml`) |
| `publish_annotated` | bool | `False` | 104 | 주석 이미지 발행 |
| `log_detections` | bool | `True` | 105 | 검출 로그 |
| `stats_period` | double | `5.0` | 106 | [s] FPS·지연 통계 로그 주기 |
| `topics.image` | string | `camera/image_raw` | 107 | 입력 이미지 |
| `topics.detections` | string | `perception/detections_2d` | 108 | 2D 검출 출력 |
| `topics.annotated` | string | `perception/detections_image` | 109 | 주석 이미지 출력 |

런치가 `device`(인자 `device`) 와 `weights`(인자 `weights`, 비어 있지 않을 때) 를 덮어쓴다
(`perception.launch.py:48-52`). 환경변수 `YOLO_CONFIG_DIR=/tmp/Ultralytics` 를 주입한다 (`:59`).

---

## 2. `object_localizer_node`

- **소스**: `src/amr_perception/amr_perception/object_localizer_node.py` · Python
- **역할**: 2D 검출 + 깊이 영상 → 핀홀 역투영 → `map` 프레임 3D 위치. 검출과 깊이를
  `ApproximateTimeSynchronizer` 로 맞춘다 (`:161-165`).

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `perception/detected_objects` | `topics.output` | `amr_msgs/DetectedObjectArray` | 기본 (10) | `:114`, `:152` |
| `perception/detections_3d` | `topics.output_3d` | `vision_msgs/Detection3DArray` | 기본 (10) | `:115`, `:153` |

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `perception/detections_2d` | `topics.detections` | `vision_msgs/Detection2DArray` | 기본 (동기화기 입력) | `:109`, `:159` |
| `camera/depth/image_raw` | `topics.depth` | `sensor_msgs/Image` | `sensor` (동기화기 입력) | `:110`, `:160-161` |
| `camera/camera_info` | `topics.camera_info` | `sensor_msgs/CameraInfo` | `sensor` | `:111`, `:154-155` |
| `camera/depth/camera_info` | `topics.depth_camera_info` | `sensor_msgs/CameraInfo` | `sensor` | `:112`, `:156-157` |
| `odometry/filtered_map` | `topics.odometry` | `nav_msgs/Odometry` | 기본 (10) | `:113`, `:158` |

TF 는 전용 스레드 리스너로 받는다 (`:150`). 서비스·액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `frame_prefix` | string | `''` | 85 | 프레임 접두어 (런치가 `amr_01/`) |
| `target_frame` | string | `map` | 86 | 출력 좌표 프레임 |
| `fallback_frames` | string[] | `['odom','base_link']` | 87 | `target_frame` TF 가 없을 때 대체 |
| `base_frame` | string | `base_link` | 88 | 거리 계산 기준 프레임 |
| `sync_slop` | double | `0.017` | 89 | [s] 검출↔깊이 시각 허용차 |
| `sync_queue` | int | `10` | 90 | 동기화 큐 길이 |
| `tf_timeout` | double | `0.05` | 91 | [s] TF 대기 |
| `roi_frac` | double | `0.5` | 92 | bbox 중심 ROI 비율 (깊이 표본 영역) |
| `min_valid_fraction` | double | `0.3` | 93 | ROI 안 유효 깊이 픽셀 최소 비율 |
| `depth_camera.range_min` | double | `0.20` | 94 | [m] 유효 깊이 하한 |
| `depth_camera.range_max` | double | `10.0` | 95 | [m] 유효 깊이 상한 |
| `depth_camera.noise_base` | double | `0.005` | 96 | [m] 깊이 잡음 상수항 |
| `depth_camera.noise_quadratic_coeff` | double | `0.002` | 97 | 깊이 잡음 2차항 계수 (σ ∝ d²) |
| `add_quadratic_noise` | bool | `True` | 98 | 2차 잡음 항 사용 |
| `iid_pixels` | bool | `True` | 99 | ROI 픽셀 독립 가정 |
| `kappa` | double | `0.05` | 100 | 공분산 보정 계수 |
| `sigma_skew_px` | double | `0.0` | 101 | [px] 픽셀 비대칭 σ |
| `mad_k` | double | `3.0` | 102 | MAD 이상점 배제 배수 |
| `seed` | int | `0` | 103 | 0 = 무작위 |
| `use_pose_covariance` | bool | `True` | 104 | 로봇 자세 공분산을 결과에 전파 |
| `log_transforms` | bool | `True` | 105 | TF 변환 로그 |
| `camera_offset.<클래스>` | double[2] | 클래스별 `(μ, σ)` | 106-107 | [m] 클래스별 표면→중심 오프셋 모델. 이름 목록은 `DEFAULT_OFFSETS` |
| `camera_offset.default` | double[2] | `[0.0, 0.05]` | 108 | 위 표에 없는 클래스의 기본 오프셋 |
| `topics.detections` | string | `perception/detections_2d` | 109 | |
| `topics.depth` | string | `camera/depth/image_raw` | 110 | |
| `topics.camera_info` | string | `camera/camera_info` | 111 | |
| `topics.depth_camera_info` | string | `camera/depth/camera_info` | 112 | |
| `topics.odometry` | string | `odometry/filtered_map` | 113 | |
| `topics.output` | string | `perception/detected_objects` | 114 | |
| `topics.output_3d` | string | `perception/detections_3d` | 115 | |

---

## 3. `detection_marker_node`

- **소스**: `src/amr_perception/amr_perception/detection_marker_node.py` · Python
- **역할**: `DetectedObjectArray` → RViz `MarkerArray` (박스 + 클래스 라벨).

### 발행 / 구독

| 종류 | 이름 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- | --- |
| 발행 | `perception/markers` | `topics.markers` | `visualization_msgs/MarkerArray` | 기본 (10) | `:77`, `:82` |
| 구독 | `perception/detected_objects` | `topics.input` | `amr_msgs/DetectedObjectArray` | 기본 (10) | `:76`, `:83-84` |

주기 = 입력마다 (콜백 구동). 서비스·액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `lifetime` | double | `1.0` | 73 | [s] 마커 수명 |
| `text_height` | double | `0.25` | 74 | [m] 라벨 글자 높이 |
| `capitalize_class` | bool | `True` | 75 | 클래스 이름 첫 글자 대문자 |
| `topics.input` | string | `perception/detected_objects` | 76 | |
| `topics.markers` | string | `perception/markers` | 77 | |

---

## 4. `aruco_detector_node`

- **소스**: `src/amr_perception/amr_perception/aruco_detector_node.py` · Python
- **역할**: `camera/image_raw` → ArUco 검출 → 마커 자세를 `base_link` 기준으로 발행 (정밀 도킹·위치 보정용).
  도킹 서버가 `topics.preferred_id` 로 원하는 마커를 지정한다 (`:141-144` 주석: 랙 마커가 도크 마커보다
  가까워 도킹이 실패한 통합 10 사례).

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `perception/dock_marker_pose` | `topics.pose` | `geometry_msgs/PoseStamped` | 기본 (10) | `:102`, `:129` |
| `perception/dock_marker_id` | `topics.id` | `std_msgs/Int32` | 기본 (10) | `:103`, `:130` |
| `perception/dock_marker_pose_cov` | `topics.pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | 기본 (10) | `:104`, `:131-132` |
| `perception/aruco_image` | `topics.debug_image` | `sensor_msgs/Image` | depth 1 | `:106`, `:133-134` — `publish_debug_image:=true` 일 때만 |

**발행 순서 주의**: 코드 주석(`:141-143`)에 따르면 단일 스레드 실행기에서 콜백은 **등록 순서**로 돌기 때문에
`id` 발행자를 공분산보다 먼저 등록해야 한다. 도킹 서버 쪽 구독 등록 순서도 같은 이유로 고정되어 있다
(`docking_server_node.cpp:191-203` 주석).

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `camera/image_raw` | `topics.image` | `sensor_msgs/Image` | BEST_EFFORT depth 1 (`:135-136`) | `:100`, `:139` |
| `camera/camera_info` | `topics.camera_info` | `sensor_msgs/CameraInfo` | `sensor` | `:101`, `:137-138` |
| `perception/aruco/preferred_id` | `topics.preferred_id` | `std_msgs/Int32` | 기본 (10) | `:105`, `:144` |

### 서비스 (서버)

| 서비스 (기본값) | 파라미터 | 타입 | 근거 | 설명 |
| --- | --- | --- | --- | --- |
| `perception/aruco/enable` | `topics.enable_service` | `std_srvs/srv/SetBool` | `:107`, `:140` | 검출 on/off (도킹 서버의 `detector_enable_service` 상대) |

액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `True` | 81 | 기동 시 검출 활성 여부 |
| `dictionary` | string | `DICT_4X4_50` | 82 | ArUco 사전 |
| `marker_size` | double | `0.18` | 83 | [m] 기본 마커 한 변 |
| `marker_ids` | int[] | `[-1]` | 84 | 허용 id 목록 (`-1` = 전부) |
| `preferred_id` | int | `-1` | 85 | 선호 id 시작값 (`topics.preferred_id` 로 변경) |
| `min_side_px` | double | `12.0` | 86 | [px] 이보다 작은 마커는 버림 |
| `marker_size_by_id` | dynamic | `[]` | 93 | `[id, 크기, id, 크기, …]` — 도크/충전(0~9)은 근거리 정밀용 |
| `ambiguity_px` | double | `1.0` | 94 | [px] 자세 모호성 판정 재투영 오차 |
| `use_upright_prior` | bool | `True` | 95 | 마커가 수직이라는 사전지식으로 모호성 해소 |
| `frame_prefix` | string | `''` | 96 | |
| `base_frame` | string | `base_link` | 97 | 출력 자세의 기준 프레임 |
| `tf_timeout` | double | `0.1` | 98 | [s] |
| `publish_debug_image` | bool | `False` | 99 | |
| `topics.image` | string | `camera/image_raw` | 100 | |
| `topics.camera_info` | string | `camera/camera_info` | 101 | |
| `topics.pose` | string | `perception/dock_marker_pose` | 102 | |
| `topics.id` | string | `perception/dock_marker_id` | 103 | |
| `topics.pose_cov` | string | `perception/dock_marker_pose_cov` | 104 | |
| `topics.preferred_id` | string | `perception/aruco/preferred_id` | 105 | |
| `topics.debug_image` | string | `perception/aruco_image` | 106 | |
| `topics.enable_service` | string | `perception/aruco/enable` | 107 | |

> 위 표의 "행" 은 `p(...)` 호출이 있는 행이다. `p = self.declare_parameter` (`:80`).

---

## 5. `pointcloud_filter_node`

- **소스**: `src/amr_perception/src/pointcloud_filter_node.cpp` · C++
- **역할**: 깊이 영상 → 점군 생성 + 복셀 다운샘플 → `camera/depth/points_filtered`.
  LiDAR 평면 아래 물체(소형 박스, 지게차 포크)를 코스트맵·안전 게이트에 넘기는 경로다.

### 발행 / 구독

| 종류 | 이름 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- | --- |
| 발행 | `camera/depth/points_filtered` | `topics.output` | `sensor_msgs/PointCloud2` | `sensor` | `:51`, `:50-52` |
| 구독 | `camera/depth/image_raw` | `topics.depth` | `sensor_msgs/Image` | `sensor` | `:63`, `:62-65` |
| 구독 | `camera/depth/camera_info` | `topics.camera_info` | `sensor_msgs/CameraInfo` | `sensor` | `:54`, `:53-56` |

주기 = 입력 깊이 영상마다 (콜백 구동). 서비스·액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `depth_camera.range_min` | double | `0.20` | 39 | [m] 유효 깊이 하한 |
| `depth_camera.noise_quadratic_coeff` | double | `0.002` | 41 | 깊이 잡음 2차항 계수 |
| `max_range` | double | `5.0` | 42 | [m] 이보다 먼 점은 버림 |
| `leaf_size` | double | `0.05` | 43 | [m] 복셀 격자 크기 |
| `add_noise` | bool | `true` | 44 | 깊이 잡음 모델 적용 |
| `pixel_step` | int | `1` | 45 | 픽셀 샘플 간격 (다운샘플) |
| `min_points_per_voxel` | int | `1` | 46 | 복셀 유지 최소 점 수 |
| `seed` | int | `0` | 47 | 잡음 seed |
| `topics.output` | string | `camera/depth/points_filtered` | 51 | |
| `topics.camera_info` | string | `camera/depth/camera_info` | 54 | |
| `topics.depth` | string | `camera/depth/image_raw` | 63 | |

---

## 6. `obstacle_tracker_node`

- **소스**: `src/amr_perception/src/obstacle_tracker_node.cpp` · C++
- **역할**: `scan_filtered` → 배경(정적 지도) 제거 → 클러스터링 → 칼만 필터 + Hungarian 연관 →
  동적/정적 분류 → TTC 계산 → `perception/tracked_obstacles`.

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 주기 | 근거 |
| --- | --- | --- | --- | --- | --- |
| `perception/tracked_obstacles` | `topics.output` | `amr_msgs/TrackedObstacleArray` | `QoS(10).reliable()` | 스캔마다 (LiDAR 10 Hz, 콜백 구동 `:343`) | `:146`, `:145-147` |
| `perception/tracked_markers` | `topics.markers` | `visualization_msgs/MarkerArray` | 기본 (10) | 위와 같음 (`:420`) | `:149`, `:148-149` — `publish_markers:=true` (기본) 일 때만 |

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `scan_filtered` | `topics.scan` | `sensor_msgs/LaserScan` | `sensor` | `:151`, `:150-152` |
| `/map` | `topics.map` | `nav_msgs/OccupancyGrid` | **latched** | `:154`, `:153-155` |
| `odometry/filtered_map` | `topics.odometry` | `nav_msgs/Odometry` | 기본 (10) | `:157`, `:156-160` |
| `plan` | `topics.plan` | `nav_msgs/Path` | 기본 (10) | `:163`, `:162-166` |

서비스·액션 없다.

### 파라미터

프레임·타이밍·출력

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `frame_prefix` | string | `""` | 50 | 프레임 접두어 |
| `tracking_frame` | string | `odom` | 52 | 추적 상태를 유지하는 프레임 |
| `base_frame` | string | `base_link` | 53 | 로봇 기준 프레임 |
| `output_frame` | string | `map` | 54 | 출력 좌표 프레임 (접두어 안 붙음) |
| `tf_timeout` | double | `0.05` | 55 | [s] TF 대기 |
| `plan_timeout` | double | `5.0` | 56 | [s] 경로가 이보다 오래되면 TTC 계산에 안 씀 |
| `odom_timeout` | double | `0.5` | 57 | [s] 오도메트리 노후 판정 |
| `map_occupied_threshold` | int | `65` | 58 | `OccupancyGrid` 점유 판정 값 |
| `publish_markers` | bool | `true` | 59 | RViz 마커 발행 |
| `output.heading_min_speed` | double | `0.1` | 123 | [m/s] 이 이하면 heading 을 내지 않음 |
| `output.confidence_sigma_ref` | double | `0.3` | 129 | 신뢰도 기준 σ |
| `output.publish_tentative` | bool | `false` | 130 | 미확정(tentative) 트랙도 발행 |

세그먼테이션 (`segmentation.*`, 61-75행)

| 이름 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- |
| `segmentation.lambda_deg` | `10.0` | 61 | [deg] 적응 거리 문턱의 각도 파라미터 |
| `segmentation.sigma_r` | `0.03` | 62 | [m] 거리 잡음 σ (`lidar.noise_stddev` 가 > 0 이면 그 값, `:64-67`) |
| `segmentation.merge_min_gap` | `0.10` | 68 | [m] 이보다 가까운 클러스터는 병합 |
| `segmentation.min_points_near` | `3` | 69 | 근거리 클러스터 최소 점 수 |
| `segmentation.min_points_far` | `2` | 70 | 원거리 최소 점 수 |
| `segmentation.far_range` | `6.0` | 71 | [m] 근/원거리 경계 |
| `segmentation.max_extent` | `1.5` | 72 | [m] 클러스터 최대 크기 |
| `segmentation.max_range` | `12.0` | 73 | [m] 처리 최대 거리 |
| `segmentation.background_radius` | `0.10` | 74 | [m] 정적 배경 판정 반경 |
| `segmentation.overlap_radius` | `0.25` | 75 | [m] 클러스터 중첩 판정 반경 |

배경 정합 (`background.*`, 76-90행) — 스캔을 정적 지도에 맞춰 배경을 정확히 지우는 단계

| 이름 | 기본값 | 행 | 이름 | 기본값 | 행 |
| --- | --- | --- | --- | --- | --- |
| `background.align_to_map` | `true` | 76 | `background.max_correspondence` | `0.30` | 78 |
| `background.huber` | `0.05` | 79 | `background.iterations` | `8` | 80 |
| `background.min_points` | `40` | 81 | `background.max_translation` | `0.30` | 82 |
| `background.max_rotation` | `0.06` | 83 | `background.max_residual` | `0.05` | 84 |
| `background.k_sigma` | `3.0` | 85 | `background.max_radius` | `0.35` | 86 |
| `background.prior_sigma_xy_min` | `0.03` | 87 | `background.prior_sigma_xy_max` | `0.20` | 88 |
| `background.prior_sigma_yaw_min` | `0.005` | 89 | `background.prior_sigma_yaw_max` | `0.05` | 90 |

클러스터 관측 모델 (`cluster_model.*`, 92-100행)

| 이름 | 기본값 | 행 | 이름 | 기본값 | 행 |
| --- | --- | --- | --- | --- | --- |
| `cluster_model.bias_mu` | `0.15` | 92 | `cluster_model.sigma_delta` | `0.10` | 93 |
| `cluster_model.sigma_lat` | `0.03` | 94 | `cluster_model.sigma_floor` | `0.02` | 96 |
| `cluster_model.sigma_seg` | `0.03` | 97 | `cluster_model.min_radius` | `0.10` | 98 |
| `cluster_model.sigma_occluded` | `0.20` | 99 | `cluster_model.occlusion_margin` | `0.30` | 100 |

칼만 필터 · 연관 · 생명주기 · 동적 분류 (103-122행)

| 이름 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- |
| `kf.q` | `0.25` | 103 | 프로세스 잡음 세기 |
| `kf.init_velocity_std` | `1.5` | 104 | [m/s] 초기 속도 σ |
| `association.gate_chi2` | `9.21` | 105 | 연관 게이트 χ² (2 자유도 99 %) |
| `association.pd_visible` | `0.9` | 106 | 가시 트랙 검출 확률 |
| `association.pd_occluded` | `0.5` | 107 | 가림 트랙 검출 확률 |
| `association.far_detection_range` | `8.0` | 108 | [m] |
| `association.lambda_birth` | `0.01` | 109 | 신규 트랙 발생률 |
| `association.v_phys` | `3.0` | 110 | [m/s] 물리적 최대 속도 |
| `lifecycle.confirm_hits` | `3` | 111 | 확정에 필요한 적중 수 |
| `lifecycle.confirm_window` | `5` | 112 | 확정 판정 창 |
| `lifecycle.min_confirm_age` | `0.2` | 113 | [s] 최소 확정 나이 |
| `lifecycle.max_misses` | `5` | 114 | 연속 미검출 삭제 문턱 |
| `lifecycle.max_misses_occluded` | `15` | 115 | 가림 상태의 삭제 문턱 |
| `dynamic.window` | `10` | 116 | 속도 판정 창 |
| `dynamic.min_samples` | `4` | 117 | 속도 판정 최소 표본 |
| `dynamic.chi2` | `13.82` | 118 | 동적 판정 χ² |
| `dynamic.v_min` | `0.15` | 119 | [m/s] 동적 최소 속도 |
| `dynamic.consecutive` | `2` | 121 | 동적 확정 연속 횟수 |
| `dynamic.release` | `5` | 122 | 동적 해제 연속 횟수 |

TTC (`ttc.*`, 133-139행)

| 이름 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- |
| `ttc.horizon` | `5.0` | 133 | [s] 예측 지평 |
| `ttc.step` | `0.1` | 134 | [s] 예측 시간 간격 |
| `ttc.k_sigma` | `1.0` | 135 | 불확실도 반영 배수 |
| `ttc.sigma_cap` | `0.5` | 136 | [m] σ 상한 |
| `ttc.robot_radius` | `0.361` | 137 | [m] 외접 반경 (0.60×0.40 풋프린트) |
| `ttc.min_robot_speed` | `0.2` | 138 | [m/s] 이 미만이면 TTC 계산 안 함 |
| `ttc.max_path_deviation` | `1.0` | 139 | [m] 경로에서 이보다 벗어난 장애물은 제외 |

`output.beta` (124-128행, `std::vector<double>`) 는 선언 행만 확인했고 기본값을 한 행으로 지목하지 못했다.

---

## 7. `safety_node`

- **소스**: `src/amr_perception/src/safety_node.cpp` · C++
- **역할**: **명령 사슬의 마지막 게이트.** `cmd_vel_smoothed` 를 받아 안전 영역(경고/위험/비상정지) ·
  TTC · E-stop · 센서 감시 · 도킹 제외 영역을 적용한 뒤 `cmd_vel` 로 낸다.
  `cmd_vel` 의 **유일한 발행자**다.
- **주기**: `rate` = 50 Hz 타이머 (`:175`, `:281-284`) + 입력 변화 시 즉시 (`kOnChange`).

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `cmd_vel` | `topics.cmd_out` | `geometry_msgs/Twist` | `QoS(10).reliable()` | `:210`, `:209-210` |
| `safety/zone` | `topics.zone` | `std_msgs/UInt8` | **latched** | `:212`, `:211-212` |
| `safety/zone_name` | `topics.zone_name` | `std_msgs/String` | **latched** | `:214`, `:213-214` |
| `safety/estop_active` | `topics.estop_active` | `std_msgs/Bool` | **latched** | `:216`, `:215-216` |
| `diagnostics` | `topics.diagnostics` | `diagnostic_msgs/DiagnosticArray` | 기본 (10) | `:218`, `:217-218` |

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `cmd_vel_smoothed` | `topics.cmd_in` | `geometry_msgs/Twist` | `QoS(10).reliable()` | `:221`, `:220-223` |
| `scan_filtered` | `sensor_topics.lidar` | `sensor_msgs/LaserScan` | `sensor` | `:77`, `:195`, `:227-229` |
| `wheel_odom` | `sensor_topics.wheel_encoder` | `nav_msgs/Odometry` | `sensor` | `:79`, `:230-236` |
| `imu/data` | `sensor_topics.imu` | `sensor_msgs/Imu` | `sensor` | `:80`, `:237-239` |
| `camera/camera_info` | `sensor_topics.rgb_camera` | `sensor_msgs/CameraInfo` | `sensor` | `:81`, `:240-242` |
| `camera/depth/camera_info` | `sensor_topics.depth_camera` | `sensor_msgs/CameraInfo` | `sensor` | `:82`, `:243-245` |
| `camera/depth/points_filtered` | `depth_cloud.topic` | `sensor_msgs/PointCloud2` | `sensor` | `:248`, `:246-250` — `depth_cloud.enabled` 일 때만 |
| `perception/tracked_obstacles` | `topics.tracked_obstacles` | `amr_msgs/TrackedObstacleArray` | `QoS(10).reliable()` | `:253`, `:252-255` |
| `estop` | `topics.estop` | `std_msgs/Bool` | **latched + volatile 이중 구독** | `:260`, `:259-265` |
| `/fleet/estop` | `topics.fleet_estop` | `std_msgs/Bool` | **latched + volatile 이중 구독** | `:261`, `:259-265` |
| `safety/dock_exclusion` | `topics.dock_exclusion` | `geometry_msgs/PolygonStamped` | 기본 (10) | `:267`, `:266-268` |

> **E-stop 이중 구독** (`:256-265`): latched(TRANSIENT_LOCAL) 발행자와 volatile 발행자를 모두 받으려고
> 같은 토픽을 두 QoS 로 구독한다. 같은 표본이 두 번 오므로 발행자별 원천 시각으로 거른다.

### 서비스 (서버)

| 서비스 (기본값) | 파라미터 | 타입 | 근거 | 설명 |
| --- | --- | --- | --- | --- |
| `safety/reset_estop` | `topics.reset_estop` | `std_srvs/srv/Trigger` | `:270`, `:269-279` | E-stop 래치 해제. 대시보드가 부른다 (`dashboard_node.py:102`) |

액션 없다.

### 파라미터

로봇 기하·한계

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robot.footprint_length` | double | `0.60` | 101 | [m] 풋프린트 길이 |
| `robot.footprint_width` | double | `0.40` | 102 | [m] 풋프린트 폭 |
| `robot.base_link_height` | double | `0.18` | 103 | [m] `base_link` 지면 높이 |
| `limits.max_linear_velocity` | double | `2.0` | 104 | [m/s] |
| `limits.min_linear_velocity` | double | `-0.5` | 105 | [m/s] (후진) |
| `limits.max_angular_velocity` | double | `1.5` | 106 | [rad/s] |
| `limits.max_linear_acceleration` | double | `1.0` | 107 | [m/s²] 정지 거리 계산용 감속도 |

안전 영역

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `safety.distance_reference` | string | `footprint_edge` | 109 | 거리 기준 (풋프린트 모서리) |
| `safety.emergency_stop_distance` | double | `0.30` | 115 | [m] 비상정지 영역 |
| `safety.critical_zone_distance` | double | `0.50` | 116 | [m] 위험 영역 |
| `safety.warning_zone_distance` | double | `1.00` | 117 | [m] 경고 영역 |
| `safety.warning_zone_max_speed` | double | `0.5` | 118 | [m/s] 경고 영역 속도 상한 |
| `safety.critical_zone_max_speed` | double | `0.2` | 119 | [m/s] 위험 영역 속도 상한 |
| `safety.reaction_latency` | double | `0.15` | 120 | [s] 반응 지연 (정지 거리에 더함) |
| `safety.clearance_speed_limit_enabled` | bool | `true` | 122 | 여유 거리 기반 속도 제한 |
| `safety.degraded_mode_max_speed` | double | `0.2` | 123 | [m/s] 센서 성능 저하 시 상한 |
| `stop_release_distance` | double | `0.50` | 125 | [m] 정지 해제 거리 (히스테리시스) |
| `zone_hysteresis` | double | `0.05` | 126 | [m] 영역 전환 히스테리시스 |

TTC · 명령 · E-stop

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `ttc.enabled` | bool | `true` | 127 | TTC 기반 속도 제한 |
| `ttc.critical` | double | `2.15` | 128 | [s] 이보다 작으면 정지 |
| `ttc.max_age` | double | `0.5` | 129 | [s] 트랙 노후 문턱 |
| `ttc.only_dynamic` | bool | `true` | 130 | **동적 트랙만** TTC 제한 (정적 벽에 걸려 0.60 m 통로를 못 지나던 문제) |
| `command_timeout` | double | `0.5` | 131 | [s] 입력 명령이 끊기면 정지 |
| `estop_release_requires_reset` | bool | `true` | 132 | E-stop 해제에 `reset_estop` 서비스 필요 |

접근 영역(swept region) · 로버스트성

| 이름 | 기본값 | 행 | 이름 | 기본값 | 행 |
| --- | --- | --- | --- | --- | --- |
| `approach.swept_margin` | `0.0` | 133 | `approach.region_margin` | `0.10` | 134 |
| `approach.hold_motion_intent` | `true` | 135 | `approach.measured_timeout` | `0.2` | 136 |
| `approach.measured_time_constant` | `0.1` | 138 | `self_filter_margin` | `0.02` | 139 |
| `robust.beam_window_width` | `0.04` | 140 | `robust.beam_window` | `5` | 141 |
| `robust.beam_support` | `0.6` | 142 | `robust.guard_window_width` | `0.06` | 143 |
| `robust.guard_window` | `9` | 144 | `robust.contact_guard_distance` | `0.02` | 145 |
| `robust.stop_confirm_frames` | `2` | 146 | `robust.immediate_stop_margin` | `0.05` | 147 |

깊이 점군 · 지연 보상 · 탈출

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `depth_cloud.enabled` | bool | `true` | 170 | 깊이 점군 사용 |
| `depth_cloud.topic` | string | `camera/depth/points_filtered` | 248 | |
| `depth_cloud.min_points` | int | `3` | 148 | 유효 판정 최소 점 수 |
| `depth_cloud.timeout` | double | `0.3` | 149 | [s] |
| `depth_cloud.max_age` | double | `0.4` | 150 | [s] |
| `depth_cloud.min_height` | double | `0.03` | 172 | [m] 지면 기준 하한 |
| `depth_cloud.max_height` | double | `0.40` | 173 | [m] 상한 |
| `depth_cloud.max_range` | double | `3.5` | 174 | [m] |
| `latency_compensation` | bool | `true` | 151 | 스캔 시각 보상 |
| `max_compensation_age` | double | `0.5` | 152 | [s] |
| `allow_escape` | bool | `true` | 153 | 막힌 상태에서 벗어나는 명령 허용 |
| `escape_horizon` | double | `0.3` | 154 | [s] |
| `escape_max_speed` | double | `critical_zone_max_speed` | 155 | [m/s] |
| `retreat_max_speed` | double | (내부 기본값) | 156 | [m/s] 후퇴 속도 상한 |
| `yield_escape.enabled` | bool | (내부 기본값) | 158 | 양보 탈출 |
| `yield_escape.speed` | double | (내부 기본값) | 159 | [m/s] |
| `yield_escape.obstacle_radius` | double | (내부 `dynamic_obstacle_radius`) | 161 | [m] |
| `yield_escape.overlap_margin` | double | (내부 기본값) | 163 | [m] |
| `yield_escape.min_gain` | double | (내부 기본값) | 165 | |
| `yield_escape.min_obstacle_speed` | double | (내부 기본값) | 166-167 | [m/s] |
| `exclusion_stop_distance` | double | `0.10` | 168 | [m] 도킹 제외 영역 안 정지 거리 |
| `exclusion_timeout` | double | `0.3` | 169 | [s] 제외 영역 폴리곤 노후 문턱 |
| `rate` | double | `50.0` | 175 | [Hz] 게이트 주기 |
| `frame_prefix` | string | `""` | 176 | |
| `base_frame` | string | `base_link` | 177 | |
| `odom_frame` | string | `odom` | 178 | |

센서 감시 — `kSensorDefaults` (`:76-83`) 로 5개 센서를 루프 선언한다 (`:184-196`).
각 센서 `<name>` 마다 `safety.sensor_timeouts.<name>`, `<name>.update_rate`,
`sensor_fault_periods.<name>`, `sensor_actions.<name>`, `sensor_topics.<name>` 이 생긴다.

| `<name>` | 기본 토픽 | 기본 `update_rate` [Hz] | 기본 지연 [s] | 기본 `fault_periods` | 기본 동작 | 행 |
| --- | --- | --- | --- | --- | --- | --- |
| `lidar` | `scan_filtered` | 10.0 | 0.3 | 3 | `stop` | 77 |
| `wheel_encoder` | `wheel_odom` | 50.0 | 0.06 | 10 | `stop` | 79 |
| `imu` | `imu/data` | 100.0 | 0.05 | 20 | `degraded` | 80 |
| `rgb_camera` | `camera/camera_info` | 30.0 | 0.1 | 9 | `degraded` | 81 |
| `depth_camera` | `camera/depth/camera_info` | 15.0 | 0.2 | 6 | `degraded` | 82 |

고장 판정 시간 = `max(지연 문턱, fault_periods / update_rate)` (`:191`).
`지연 문턱 ≤ 0` 이면 그 센서는 감시하지 않는다 (`:196`).

---

## 8. 런치 인자 (`perception.launch.py`)

| 인자 | 기본값(확인) | 설명 |
| --- | --- | --- |
| `use_yolo` / `use_localizer` / `use_markers` / `use_aruco` / `use_pointcloud` / `use_tracker` / `use_safety` | — | 노드별 기동 플래그 (`IfCondition`, `:53-57`) |
| `device` | — | `yolo_node` 의 `device` 덮어쓰기 (`:48`) |
| `weights` | — | 비어 있지 않으면 `yolo_node` 의 `weights` 덮어쓰기 (`:49-51`) |
| `params_file` | 패키지 `config/perception.yaml` | `:38-40` |
| `config_dir` | — | `robot_params.yaml`, `sensors.yaml` 위치 (`:41-44`) |

각 인자의 `DeclareLaunchArgument` 기본값 행은 확인하지 않았다 (아래 참조).

---

## 9. 확인 못 함

- `perception.launch.py` 의 `DeclareLaunchArgument` **기본값**을 행 단위로 확인하지 않았다.
  노드 기동 플래그가 있다는 것과 `parameters` 조립 순서(`shared + [params_file, overrides]`, `:54`)만 확인했다.
- `obstacle_tracker_node` 의 `output.beta` (`:124-128`) 기본값 벡터를 한 행으로 지목하지 못했다.
- `safety_node` 의 `retreat_max_speed`, `yield_escape.*` 기본값은 `declare_parameter` 의 두 번째 인자가
  구조체 필드(`p.<필드>`)라 **코드 상수가 아니라 구조체 초기값**이다. 그 구조체 정의 파일까지 추적하지 않았다.
  실제 운용값은 `src/amr_perception/config/perception.yaml` 의 `safety_node` 블록(`:206` 이하)을 본다.
- `perception.yaml` 각 블록이 위 기본값 중 무엇을 덮어쓰는지 **항목별로 대조하지 않았다.**
  블록 시작 행(§0 표)만 확인했다.
- `sensor_msgs/Image` 인코딩, `Detection2DArray` 의 `class_id` 문자열 규약 등 **메시지 내용 규약**은
  이 문서 범위 밖이다 (`src/amr_perception/config/classes.yaml` 참조).
