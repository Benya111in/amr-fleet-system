# API — `amr_navigation`

> 명세 4장 4절 (경로 계획 / 지역 계획 / 경로 추종 / Costmap). [← 색인](README.md)
> 알고리즘·튜닝 근거: [../algorithms/astar.md](../algorithms/astar.md),
> [../algorithms/dwa.md](../algorithms/dwa.md), [../algorithms/path_tracking.md](../algorithms/path_tracking.md),
> [../algorithms/costmap.md](../algorithms/costmap.md).
> 런치: `src/amr_navigation/launch/navigation.launch.py`. 모든 노드가 로봇 네임스페이스 아래에 뜬다
> (`Node(namespace=robot_name)`, `:130`).
> **설정 파일**: `src/amr_navigation/config/nav2_params.yaml` (템플릿 — 런치가 `<prefix>` 와 `<robot_ns>` 를 치환하고
> `RewrittenYaml(root_key=robot_name)` 으로 네임스페이스 키를 씌운다, `:117-125`).

## 0. 이 패키지의 노드·플러그인

| 이름 | 언어 | 형태 | 소스 | 기동 |
| --- | --- | --- | --- | --- |
| `velocity_profiler_node` | C++ | own | `src/nodes/velocity_profiler_node.cpp` | `navigation.launch.py:153-154` |
| `costmap_scan_filter_node` | C++ | own | `src/nodes/costmap_scan_filter_node.cpp` | `navigation.launch.py:155-156` |
| `amr_navigation::AStarPlanner` | C++ | plugin (`nav2_core::GlobalPlanner`) | `plugins/nav2_plugins.xml:2` | `planner_server` 가 로드 |
| `amr_navigation::DWAController` | C++ | plugin (`nav2_core::Controller`) | `src/ros/dwa_controller.cpp` | `controller_server` 가 로드 |
| `amr_navigation::PurePursuitController` | C++ | plugin (`nav2_core::Controller`) | `src/ros/pure_pursuit_controller.cpp` | `controller_server` 가 로드 |
| `planner_server` | C++ | ext (`nav2_planner`) | — | `:137-138` |
| `controller_server` | C++ | ext (`nav2_controller`) | — | `:139-140` |
| `behavior_server` | C++ | ext (`nav2_behaviors`) | — | `:141-142` |
| `bt_navigator` | C++ | ext (`nav2_bt_navigator`) | — | `:143-147` |
| `lifecycle_manager_navigation` | C++ | ext | — | `:148-152` |

오프라인 도구(ROS 노드지만 운용 스택이 아님): `scripts/bench_planners.py`, `scripts/closed_loop_eval.py`,
`scripts/kinematic_sim.py`, `scripts/make_warehouse_map.py`, `tools/pid_tuning.cpp`,
`tools/astar_benchmark.cpp`, `tools/tracking_sim.cpp`.

---

## 1. `velocity_profiler_node`

- **소스**: `src/amr_navigation/src/nodes/velocity_profiler_node.cpp` · C++
- **역할**: Nav2 컨트롤러가 낸 `cmd_vel_nav` 를 받아 속도·가속도·저크 한계와 PID(기준 모델 추종)를 적용해
  `cmd_vel_smoothed` 로 낸다. 적재 질량(`payload/mass`)에 따라 한계를 조정한다.
  Nav2 의 `velocity_smoother` 를 대체한다 ([components.md §1](../architecture/components.md)).
- **주기**: `rate` = 50 Hz 타이머, 노드 시계 기준 (`use_sim_time` 이면 `/clock`) (`:28`, `:82`).

### 발행 토픽

| 토픽 | 타입 | QoS | 주기 | 근거 |
| --- | --- | --- | --- | --- |
| `cmd_vel_smoothed` | `geometry_msgs/Twist` | `QoS(1)` | 50 Hz | `:77` |
| `velocity_profiler/state` | `std_msgs/Float64MultiArray` | 기본 (10) | 50 Hz | `:77-78` |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `cmd_vel_nav` | `geometry_msgs/Twist` | `cmd_qos` (`:68-70`) | `:68-70` |
| `odometry/filtered` | `nav_msgs/Odometry` | `QoS(5)` | `:71-73` |
| `payload/mass` | `std_msgs/Float32` | `QoS(1).transient_local()` (**latched**) | `:74-76` |

서비스·액션 없다. 토픽 이름은 **모두 하드코딩**이다 (파라미터로 못 바꾼다).

### 파라미터

한계 (`limits.*`) — 런치가 `config/robot_params.yaml` 을 함께 넘겨 덮어쓴다 (`navigation.launch.py:126-128`, `:153-154`)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `limits.max_linear_velocity` | double | `2.0` | 16 | [m/s] |
| `limits.min_linear_velocity` | double | `-0.5` | 17 | [m/s] (후진) |
| `limits.max_linear_acceleration` | double | `1.0` | 18 | [m/s²] |
| `limits.max_angular_velocity` | double | `1.5` | 19 | [rad/s] |
| `limits.max_angular_acceleration` | double | `2.0` | 20 | [rad/s²] |
| `limits.max_linear_jerk` | double | `2.0` | 21 | [m/s³] |

질량 (관성 보정용)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robot.base_mass` | double | `45.0` | 22 | [kg] 차체 질량 |
| `robot.wheel_mass` | double | `1.0` | 23 | [kg] 바퀴 1개 |
| `robot.caster_mass` | double | `0.3` | 24 | [kg] 캐스터 1개 |

프로파일러 동작

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `rate` | double | `50.0` | 28 | [Hz] 출력 주기 |
| `cmd_timeout` | double | `0.5` | 30 | [s] 명령이 끊기면 감속 정지 |
| `odom_timeout` | double | `0.2` | 31 | [s] 오도메트리 노후 문턱 |
| `idle_publish_time` | double | `1.0` | 32 | [s] 정지 후에도 0 명령을 계속 낼 시간 |
| `jerk_limit_enabled` | bool | `true` | 35 | false 면 저크 제한 끔 |
| `max_angular_jerk` | double | `6.0` | 36 | [rad/s³] |
| `preserve_curvature` | bool | `true` | 37 | 감속 시 v/ω 비(곡률) 유지 |
| `curvature_min_speed` | double | `0.05` | 38 | [m/s] 곡률 보존 하한 |
| `use_pid` | bool | `true` | 39 | PID 보정 사용 |
| `use_reference_model` | bool | `true` | 40 | 1차 지연 + 데드타임 기준 모델 사용 |
| `model_time_constant` | double | `0.08` | 41 | [s] 기준 모델 τ |
| `model_delay` | double | `0.04` | 42 | [s] 기준 모델 데드타임 |
| `stop_deadband` | double | `1e-3` | 64 | 이보다 작은 명령은 0 으로 |

PID — 축 `<ax>` 는 선형·각속도 두 축에 대해 루프로 선언된다 (`:44-53`)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `pid.<ax>.kp` | double | (축별 내부 기본값 `kp`) | 46 | 비례 이득 |
| `pid.<ax>.ki` | double | (축별 내부 기본값 `ki`) | 47 | 적분 이득 |
| `pid.<ax>.kd` | double | `0.0` | 48 | 미분 이득 |
| `pid.<ax>.setpoint_weight` | double | `1.0` | 49 | 목표값 가중 (b) |
| `pid.<ax>.tracking_time` | double | (축별 `tt_default`) | 51 | [s] anti-windup 추종 시간 |
| `pid.<ax>.derivative_tau` | double | `0.02` | 52 | [s] 미분 필터 시정수 |

보정량 제한 · 재동기화

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `max_linear_correction` | double | `0.2` | 57 | [m/s] PID 보정 상한 |
| `max_angular_correction` | double | `0.3` | 58 | [rad/s] |
| `resync_threshold_v` | double | `0.4` | 59 | [m/s] 이보다 크게 벌어지면 기준 모델 재동기화 |
| `resync_threshold_w` | double | `0.6` | 60 | [rad/s] |
| `hold_error_v` | double | `0.15` | 61 | [m/s] 오차 유지 판정 |
| `hold_time` | double | `0.3` | 62 | [s] |
| `measurement_filter_time` | double | `0.04` | 63 | [s] 측정 필터 시정수 |

설정 파일: `navigation.launch.py` 가 별도 프로파일러 파일(`profiler_file`)과 `robot_params.yaml` 을 넘긴다
(`:153-154`).

---

## 2. `costmap_scan_filter_node`

- **소스**: `src/amr_navigation/src/nodes/costmap_scan_filter_node.cpp` · C++
- **역할**: 코스트맵에 넣기 전 전처리 — 스캔에 게이트 중앙값 필터를 걸고, **동적 트랙 위치의 점을 뺀
  별도 출력**을 만든다. 전역 코스트맵은 동적 제거본(`*_static`)을, 지역 코스트맵은 원본을 쓴다
  ([costmap.md §1](../algorithms/costmap.md)).
- **파라미터**: `nav2_params.yaml` 의 `costmap_scan_filter_node` 블록 (`navigation.launch.py:155-156` 이
  `configured` 로 넘긴다).

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `scan_costmap` | `output_topic` | `sensor_msgs/LaserScan` | `sensor` | `:32`, `:52` |
| `scan_costmap_static` | `static_output_topic` | `sensor_msgs/LaserScan` | `sensor` | `:34`, `:53` |
| `camera/depth/points_static` | `cloud_output_topic` | `sensor_msgs/PointCloud2` | `sensor` | `:38`, `:54` |

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `scan_filtered` | `input_topic` | `sensor_msgs/LaserScan` | `sensor` | `:31`, `:55-57` |
| `camera/depth/points_filtered` | `cloud_input_topic` | `sensor_msgs/PointCloud2` | `sensor` | `:36`, `:58-60` |
| `perception/tracked_obstacles` | `tracks_topic` | `amr_msgs/TrackedObstacleArray` | `QoS(5)` | `:40`, `:61-63` |

주기 = 입력마다 (콜백 구동). 서비스·액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `input_topic` | string | `scan_filtered` | 31 | 입력 스캔 |
| `output_topic` | string | `scan_costmap` | 32 | 지역 코스트맵용 출력 |
| `static_output_topic` | string | `scan_costmap_static` | 34 | 전역 코스트맵용 (동적 트랙 제외) |
| `cloud_input_topic` | string | `camera/depth/points_filtered` | 36 | 입력 점군 |
| `cloud_output_topic` | string | `camera/depth/points_static` | 38 | 전역용 점군 (동적 트랙 제외) |
| `tracks_topic` | string | `perception/tracked_obstacles` | 40 | 동적 트랙 입력 |
| `half_window` | int | `5` | 41 | 게이트 중앙값 필터 창 반폭 (빔 수) |
| `range_gate` | double | `0.15` | 42 | [m] 이웃과 같은 물체로 볼 거리차 |
| `min_support` | int | `3` | 43 | 창 안에 이만큼 있어야 값을 유지 |
| `exclude_dynamic` | bool | `true` | 44 | 동적 트랙 위치의 점을 `*_static` 에서 뺌 |
| `dynamic_min_speed` | double | `0.2` | 45 | [m/s] 동적 판정 최소 속도 |
| `dynamic_fast_speed` | double | `0.5` | 46 | [m/s] 빠른 동적 판정 속도 |
| `dynamic_radius` | double | `0.55` | 47 | [m] 트랙 주변 제외 반경 |
| `track_timeout` | double | `0.5` | 48 | [s] 트랙 노후 문턱 |

---

## 3. Nav2 플러그인

세 플러그인은 독립 노드가 아니라 `planner_server` / `controller_server` **프로세스 안에서** 동작한다.
따라서 토픽은 그 서버 노드의 이름으로 뜨고, 파라미터 이름 앞에는 플러그인 인스턴스 이름(`AStar`, `DWA`, …)이 붙는다.
플러그인 등록은 `src/amr_navigation/plugins/nav2_plugins.xml`, 서버 설정은 `config/nav2_params.yaml` 이다.

| 클래스 | base class | 설명 (`nav2_plugins.xml`) |
| --- | --- | --- |
| `amr_navigation::AStarPlanner` | `nav2_core::GlobalPlanner` | 8-연결 비용 인지 A* (옥타일 휴리스틱, 이진 힙) + 여유거리 보존 숏컷·경사 하강 평활화 (`:2-4`) |
| `amr_navigation::DWAController` | `nav2_core::Controller` | Dynamic Window Approach: 동적 창 샘플링, 원호 롤아웃, 풋프린트 충돌, Velocity Obstacle 샘플 제외, 동적 장애물 TTC 비용 (`:5-7`) |
| `amr_navigation::PurePursuitController` | `nav2_core::Controller` | 속도 적응 look-ahead Pure Pursuit (곡률·목표 감속, 제자리 회전, 명령 원호 충돌 검사) (`:8-10`) |

### `DWAController` 의 토픽 (`src/amr_navigation/src/ros/dwa_controller.cpp`)

| 종류 | 이름 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- | --- |
| 발행 | `local_plan` | — (하드코딩) | `nav_msgs/Path` | depth 1 | `:208` |
| 발행 | `dwa/stats` | — (하드코딩) | `std_msgs/Float64MultiArray` | 기본 (10) | `:209` |
| 구독 | `perception/tracked_obstacles` | `<plugin>.tracked_obstacles_topic` | `amr_msgs/TrackedObstacleArray` | `QoS(5)` | `:195-196`, `:202-204` |
| 구독 | `payload/mass` | `<plugin>.payload_topic` | `std_msgs/Float32` | **latched** (`QoS(1).transient_local()`) | `:197`, `:205-207` |

확인한 플러그인 파라미터 (전체는 `config/nav2_params.yaml` 의 `DWA` 블록):

| 이름 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- |
| `<plugin>.yield_lookahead` | `4.0` | 191 | [m] 정지선 탐색 거리 (경로 창으로 상한) |
| `<plugin>.no_valid_patience` | `10` | 192 | 유효 샘플이 없는 연속 주기 허용 수 |
| `<plugin>.transform_tolerance` | `0.2` | 193 | [s] TF 허용 지연 |
| `<plugin>.allow_unknown` | `true` | 194 | 미지 셀 통행 허용 |
| `<plugin>.tracked_obstacles_topic` | `perception/tracked_obstacles` | 195-196 | |
| `<plugin>.payload_topic` | `payload/mass` | 197 | |

### `PurePursuitController` 의 토픽 (`src/amr_navigation/src/ros/pure_pursuit_controller.cpp`)

| 종류 | 이름 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- | --- |
| 발행 | `local_plan` | — (하드코딩) | `nav_msgs/Path` | depth 1 | `:71` |
| 발행 | `lookahead_point` | — (하드코딩) | `geometry_msgs/PointStamped` | depth 1 | `:72` |
| 발행 | `pure_pursuit/stats` | — (하드코딩) | `std_msgs/Float64MultiArray` | 기본 (10) | `:73` |
| 구독 | `payload/mass` | `<plugin>.payload_topic` | `std_msgs/Float32` | **latched** | `:64`, `:68-70` |

확인한 플러그인 파라미터:

| 이름 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- |
| `<plugin>.goal_align_distance` | `0.08` | 55 | [m] 목표 정렬 시작 거리 |
| `<plugin>.goal_yaw_tolerance` | `0.02` | 56 | [rad] 목표 헤딩 허용 오차 |
| `<plugin>.use_chord_correction` | `false` | 57 | 현(chord) 보정 사용 |
| `<plugin>.chord_gain` | `1.0` | 58 | 현 보정 이득 |
| `<plugin>.velocity_reset_threshold` | `0.3` | 59 | [m/s] 창 초기화 속도 문턱 |
| `<plugin>.use_collision_detection` | `true` | 60 | 명령 원호 충돌 검사 |
| `<plugin>.collision_check_time` | `1.0` | 61 | [s] 충돌 검사 지평 |
| `<plugin>.transform_tolerance` | `0.2` | 62 | [s] |
| `<plugin>.robot_mass` | `47.6` | 63 | [kg] 공차 총 질량 |
| `<plugin>.payload_topic` | `payload/mass` | 64 | |

`AStarPlanner` 는 토픽을 만들지 않는다 — 계획 결과는 `planner_server` 가 `plan` / `ComputePathToPose` 로 낸다.
A* 파라미터(`cost_weight`, `allow_unknown`, `unknown_cost`, `tolerance`, `max_iterations`,
`max_planning_time`, `allow_corner_cutting`, `allow_start_in_inscribed`,
`use_final_approach_orientation`, `smoother.*`)의 값과 근거는 `config/nav2_params.yaml:24-40` 에 있다.

---

## 4. 외부 Nav2 노드 — 우리가 정한 리맵·설정

| 노드 | 우리가 정한 것 | 근거 |
| --- | --- | --- |
| `planner_server` | `planner_plugins: ["AStar", "NavFn", "Smac"]`, `expected_planner_frequency: 1.0` | `nav2_params.yaml:21-26`, 런치 `:137-138` |
| `controller_server` | **`cmd_vel` → `cmd_vel_nav` 리맵.** `robot_params.yaml` 을 추가로 넘겨 플러그인이 `limits.*` 를 쓰게 함 | `navigation.launch.py:139-140`, `nav2_params.yaml:11-12` |
| `behavior_server` | **`cmd_vel` → `cmd_vel_nav` 리맵** | `navigation.launch.py:141-142` |
| `bt_navigator` | `plugin_lib_names` (TTC BT 플러그인 포함 여부는 `nav_ttc_bt` 인자), `default_nav_to_pose_bt_xml`, `default_nav_through_poses_bt_xml` | `navigation.launch.py:143-147`, `:110-114` |
| `lifecycle_manager_navigation` | `autostart`, `node_names: LIFECYCLE_NODES`, `bond_timeout` | `navigation.launch.py:148-152` |

Nav2 서버가 여는 **액션**(이름은 Nav2 기본값, 로봇 네임스페이스 아래):

| 액션 | 타입 | 서버 | 근거 |
| --- | --- | --- | --- |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` | 클라이언트: `src/amr_behavior/include/amr_behavior/bt_nodes/navigate_to_pose.hpp:23` |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | `planner_server` | 클라이언트: `scripts/bench_planners.py:80` |
| `follow_path` | `nav2_msgs/action/FollowPath` | `controller_server` | 클라이언트: `scripts/closed_loop_eval.py:145` |
| `spin` | `nav2_msgs/action/Spin` | `behavior_server` | 클라이언트: `bt_nodes/spin.hpp:22`, `kidnap_monitor_node.py:167` |
| `backup` | `nav2_msgs/action/BackUp` | `behavior_server` | 클라이언트: `bt_nodes/back_up.hpp:21` |
| `wait` | `nav2_msgs/action/Wait` | `behavior_server` | 클라이언트: `bt_nodes/wait.hpp:19` |

Nav2 서버가 여는 **서비스** 중 우리가 쓰는 것:

| 서비스 | 타입 | 클라이언트 | 근거 |
| --- | --- | --- | --- |
| `global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | BT `ClearCostmapService` | `bt_nodes/clear_costmap.hpp:21-22` |

### 런치 인자 (`navigation.launch.py`)

확인한 것: `robot_name`, `frame_prefix`(치환용 `<prefix>`), `params_file`, `config_dir`,
`use_sim_time`, `autostart`, `respawn`, `log_level`, `nav_ttc_bt`(TTC BT 플러그인 사용 여부 —
`auto` 면 플러그인 존재 여부로 결정, `:110-112`).
각 인자의 `DeclareLaunchArgument` 기본값 행은 확인하지 않았다.

---

## 5. 확인 못 함

- `velocity_profiler_node` 의 `pid.<ax>.kp` / `.ki` / `.tracking_time` **기본값**은 `declare_parameter` 의
  두 번째 인자가 축별 변수(`kp`, `ki`, `tt_default`)라 코드 상수로 지목하지 못했다 (`:44-53`).
  실제 값은 런치가 넘기는 프로파일러 설정 파일을 본다.
- **`nav2_params.yaml` 전체 파라미터**(코스트맵 레이어, A*/DWA/PurePursuit 세부, BT 설정)는
  이 문서에 옮기지 않았다. 그 파일이 단일 출처이고 값마다 근거 주석이 달려 있다.
  이 문서에는 **토픽·서비스·액션 인터페이스와 코드에서 뽑은 기본값**만 적었다.
- `navigation.launch.py` 의 `DeclareLaunchArgument` 기본값을 행 단위로 확인하지 않았다.
- `LIFECYCLE_NODES`, `NAV2_BT_LIBS`, `TTC_BT_LIB` 상수의 내용을 확인하지 않았다 (`navigation.launch.py` 상단).
- 오프라인 도구(`bench_planners.py`, `closed_loop_eval.py`, `kinematic_sim.py`)는 ROS 토픽을 쓰지만
  운용 스택이 아니라 인터페이스 표를 만들지 않았다. §4 의 액션 표에는 근거로만 인용했다.
