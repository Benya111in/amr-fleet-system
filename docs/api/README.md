# API 문서 — 노드별 토픽 / 서비스 / 액션 인터페이스

> 명세 4장 10절 산출물: "API 문서를 작성하여 각 노드의 토픽, 서비스, 액션 인터페이스를 명세한다."
> 기준 트리: `feature/system-integration` 6635065 (2026-09-29).
> **이 문서는 소스 코드에서 기계적으로 추출한 것이다.** 모든 항목에 `파일:행` 근거를 달았다.
> 설계 의도·다이어그램은 [../architecture/components.md](../architecture/components.md),
> 네임스페이스 규칙은 [../architecture/multi_robot.md](../architecture/multi_robot.md),
> 알고리즘과 튜닝 근거는 [../algorithms/](../algorithms/README.md) 를 본다.

## 0. 이 문서를 읽는 법

- **추출 방법**: 정적 분석만 했다 (`grep` / 파일 읽기). 빌드·실행·`ros2 node info` 는 쓰지 않았다.
  근거 표기는 항상 `경로:행` 이고, 행 번호는 위 커밋 기준이다.
- **토픽 이름이 파라미터인 경우**: 코드가 `declare_parameter("topics.x", "기본이름")` 으로 이름을 받는 노드가 많다.
  표에는 **선언부의 기본값**을 적고 "파라미터" 열에 그 파라미터 이름을 적었다. 런치나 config yaml 이 덮어쓰면
  실제 이름이 달라진다 — 덮어쓰는 곳을 확인한 경우 비고에 적었다.
- **확인 못 한 것은 적지 않았다.** 빠진 항목은 각 문서 끝의 "확인 못 함" 절에 모았다.
- **QoS 열**: 기본(`rclcpp::QoS(10)` / rclpy `10`, RELIABLE·VOLATILE)과 다를 때만 적었다.
  - `sensor` = `rclcpp::SensorDataQoS()` / `qos_profile_sensor_data` (BEST_EFFORT, depth 1~5)
  - `latched` = RELIABLE + TRANSIENT_LOCAL (depth 1). 늦게 뜬 구독자도 마지막 값을 받는다.
- **주기 열**: 타이머 주기(파라미터)나 "콜백 구동"(입력 메시지마다 발행)을 적었다. 알 수 없으면 비워 두었다.

## 1. 패키지별 문서

| 문서 | 패키지 | 문서화한 노드 |
| --- | --- | --- |
| [amr_localization.md](amr_localization.md) | `amr_localization` | 6 (직접 구현) + 외부 6 |
| [amr_perception.md](amr_perception.md) | `amr_perception` | 7 |
| [amr_navigation.md](amr_navigation.md) | `amr_navigation` | 2 + Nav2 플러그인 3 + 외부 5 |
| [amr_behavior.md](amr_behavior.md) | `amr_behavior` | 3 (+ BT 노드의 액션·서비스 클라이언트) |
| [amr_fleet.md](amr_fleet.md) | `amr_fleet` | 3 |
| [amr_dashboard.md](amr_dashboard.md) | `amr_dashboard` | 1 |
| [amr_simulation.md](amr_simulation.md) | `amr_simulation` | 3 |
| [amr_bringup.md](amr_bringup.md) | `amr_bringup`, `amr_description`, `amr_evaluation` | 5 + CLI 도구 |
| [amr_msgs.md](amr_msgs.md) | `amr_msgs` | **부록** — 커스텀 msg/srv/action 필드 정의 |

## 2. 네임스페이스 규칙

설계 근거는 [multi_robot.md §1–2](../architecture/multi_robot.md), 구현 근거는
`src/amr_navigation/config/nav2_params.yaml:1-10` 머리말과 각 패키지 런치다.

| 네임스페이스 | 무엇이 뜨나 | 근거 |
| --- | --- | --- |
| `/amr_01` … `/amr_05` | 로봇 1대의 온보드 스택 전부 (브리지, 오도메트리, EKF, AMCL, Nav2, 인지, BT, 도킹, 배터리, `fleet_adapter_node`) | `src/amr_localization/launch/localization.launch.py:127-130`, `src/amr_perception/launch/perception.launch.py:55`, `src/amr_behavior/launch/behavior.launch.py:87-99`, `src/amr_fleet/launch/fleet_manager.launch.py:92-93` |
| `/fleet` | `fleet_manager_node`, `traffic_manager_node` (각 1개) | `src/amr_fleet/launch/fleet_manager.launch.py:70-83` |
| `/` (루트) | Gazebo, `/clock` 브리지, `map_server`, `lifecycle_manager_map`, `dashboard_node` | `src/amr_localization/launch/localization.launch.py:184-190` (`namespace=''`) |

**규칙**

1. 로봇 노드의 토픽은 코드에 **상대 이름**으로만 적는다. 절대 이름(`/scan`)을 코드에 쓰지 않는다.
   런치가 `Node(namespace=robot_name)` 으로 네임스페이스를 주입한다.
   따라서 아래 패키지 문서의 상대 이름 `scan_filtered` 는 실제로 `/amr_01/scan_filtered` 가 된다.
2. **공유(전역) 토픽·서비스** — 코드에 절대 이름(`/` 로 시작)으로 적혀 있거나 런치가 `namespace=''` 로 띄운다:

   | 이름 | 타입 | 누가 내나 | 근거 |
   | --- | --- | --- | --- |
   | `/map` | `nav_msgs/OccupancyGrid` (latched) | `map_server` 1개 (루트 네임스페이스) | `localization.launch.py:184-186` |
   | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 로봇별 EKF 2개 + `robot_state_publisher`. **리맵하지 않는다** | `nav2_params.yaml:6`, [multi_robot.md §2](../architecture/multi_robot.md) |
   | `/clock` | `rosgraph_msgs/Clock` | `ros_gz_bridge` | [components.md §3.1](../architecture/components.md) |
   | `/fleet/status` | `amr_msgs/FleetStatus` | `fleet_manager_node` (`/fleet` 네임스페이스의 상대 이름 `status`) | `fleet_manager_node.py:206` |
   | `/fleet/alerts` | `diagnostic_msgs/DiagnosticArray` | `fleet_manager_node`, `traffic_manager_node` | `fleet_manager_node.py:207`, `traffic_manager_node.py:180` |
   | `/fleet/task_events` | `amr_msgs/Task` | `fleet_manager_node` | `fleet_manager_node.py:205` |
   | `/fleet/traffic_events` | `diagnostic_msgs/DiagnosticArray` | `traffic_manager_node` (절대 이름 하드코딩) | `traffic_manager_node.py:178` |
   | `/fleet/task_request` | `std_msgs/String` (JSON) | `dashboard_node` → `fleet_manager_node` | `dashboard_node.py:99`, `fleet_manager_node.py:208` |
   | `/fleet/estop` | `std_msgs/Bool` (latched) | `dashboard_node` → 각 로봇 `safety_node` | `dashboard_node.py:100`, `safety_node.cpp:261` |
   | `/sim/*` | 시뮬레이션 지면 진실 | `amr_simulation` 노드 | [amr_simulation.md](amr_simulation.md) |

3. **TF 프레임 접두어**: `map` 을 뺀 모든 프레임에 `amr_01/` 접두어가 붙는다. 노드는 `frame_prefix`
   파라미터로 받는다 (`localization.launch.py:108-111` 이 `robot_name + '/'` 를 만든다).
   프레임 파라미터 기본값이 `odom`, `base_footprint`, `base_link` 인 것은 **접두어가 붙기 전 값**이다.
4. **Nav2 파라미터의 코스트맵 레이어 토픽**만 예외로 `<robot_ns>/…` 자리표시자를 쓴다 — Humble 의 `Costmap2DROS`
   가 `/<ns>/<costmap 이름>` 네임스페이스의 자식 노드라 상대 이름이 `/amr_01/local_costmap/...` 로 풀리기 때문이다
   (`nav2_params.yaml:7-10`).

## 3. 노드 색인 (전체)

"형태" 는 [components.md §3](../architecture/components.md) 의 표기를 따른다:
**own** = 직접 구현, **ext** = 외부 패키지 실행 파일(설정만 작성), **plugin** = 다른 프로세스에 로드,
**tool** = ROS 노드가 아닌 CLI 도구.

| 노드 (실행 파일) | 패키지 | 언어 | 형태 | 소스 | 문서 |
| --- | --- | --- | --- | --- | --- |
| `wheel_odometry_node` | amr_localization | C++ | own | `src/amr_localization/src/wheel_odometry_node.cpp` | [→](amr_localization.md#1-wheel_odometry_node) |
| `imu_filter_node` | amr_localization | C++ | own | `src/amr_localization/src/imu_filter_node.cpp` | [→](amr_localization.md#2-imu_filter_node) |
| `scan_filter_node` | amr_localization | C++ | own | `src/amr_localization/src/scan_filter_node.cpp` | [→](amr_localization.md#3-scan_filter_node) |
| `amcl_map_adapter` | amr_localization | Python | own | `src/amr_localization/amr_localization/amcl_map_adapter.py` | [→](amr_localization.md#4-amcl_map_adapter) |
| `kidnap_monitor_node` | amr_localization | Python | own | `src/amr_localization/amr_localization/kidnap_monitor_node.py` | [→](amr_localization.md#5-kidnap_monitor_node) |
| `scan_matcher_node` | amr_localization | Python | own | `src/amr_localization/amr_localization/scan_matcher_node.py` | [→](amr_localization.md#6-scan_matcher_node) |
| `yolo_node` | amr_perception | Python | own | `src/amr_perception/amr_perception/yolo_node.py` | [→](amr_perception.md#1-yolo_node) |
| `object_localizer_node` | amr_perception | Python | own | `src/amr_perception/amr_perception/object_localizer_node.py` | [→](amr_perception.md#2-object_localizer_node) |
| `detection_marker_node` | amr_perception | Python | own | `src/amr_perception/amr_perception/detection_marker_node.py` | [→](amr_perception.md#3-detection_marker_node) |
| `aruco_detector_node` | amr_perception | Python | own | `src/amr_perception/amr_perception/aruco_detector_node.py` | [→](amr_perception.md#4-aruco_detector_node) |
| `pointcloud_filter_node` | amr_perception | C++ | own | `src/amr_perception/src/pointcloud_filter_node.cpp` | [→](amr_perception.md#5-pointcloud_filter_node) |
| `obstacle_tracker_node` | amr_perception | C++ | own | `src/amr_perception/src/obstacle_tracker_node.cpp` | [→](amr_perception.md#6-obstacle_tracker_node) |
| `safety_node` | amr_perception | C++ | own | `src/amr_perception/src/safety_node.cpp` | [→](amr_perception.md#7-safety_node) |
| `velocity_profiler_node` | amr_navigation | C++ | own | `src/amr_navigation/src/nodes/velocity_profiler_node.cpp` | [→](amr_navigation.md#1-velocity_profiler_node) |
| `costmap_scan_filter_node` | amr_navigation | C++ | own | `src/amr_navigation/src/nodes/costmap_scan_filter_node.cpp` | [→](amr_navigation.md#2-costmap_scan_filter_node) |
| `AStarPlanner` | amr_navigation | C++ | plugin | `src/amr_navigation/src/ros/` | [→](amr_navigation.md#3-nav2-플러그인) |
| `DWAController` | amr_navigation | C++ | plugin | `src/amr_navigation/src/ros/dwa_controller.cpp` | [→](amr_navigation.md#3-nav2-플러그인) |
| `PurePursuitController` | amr_navigation | C++ | plugin | `src/amr_navigation/src/ros/pure_pursuit_controller.cpp` | [→](amr_navigation.md#3-nav2-플러그인) |
| `task_executor_node` | amr_behavior | C++ | own | `src/amr_behavior/src/task_executor_node.cpp` | [→](amr_behavior.md#1-task_executor_node) |
| `docking_server_node` | amr_behavior | C++ | own | `src/amr_behavior/src/docking/docking_server_node.cpp` | [→](amr_behavior.md#2-docking_server_node) |
| `battery_model_node` | amr_behavior | Python | own | `src/amr_behavior/amr_behavior/battery_model_node.py` | [→](amr_behavior.md#3-battery_model_node) |
| `fleet_manager_node` | amr_fleet | Python | own | `src/amr_fleet/amr_fleet/fleet_manager_node.py` | [→](amr_fleet.md#1-fleet_manager_node) |
| `traffic_manager_node` | amr_fleet | Python | own | `src/amr_fleet/amr_fleet/traffic_manager_node.py` | [→](amr_fleet.md#2-traffic_manager_node) |
| `fleet_adapter_node` | amr_fleet | Python | own | `src/amr_fleet/amr_fleet/fleet_adapter_node.py` | [→](amr_fleet.md#3-fleet_adapter_node) |
| `dashboard_node` | amr_dashboard | Python | own | `src/amr_dashboard/amr_dashboard/dashboard_node.py` | [→](amr_dashboard.md) |
| `obstacle_truth_node` | amr_simulation | Python | own | `src/amr_simulation/amr_simulation/obstacle_truth_node.py` | [→](amr_simulation.md#1-obstacle_truth_node) |
| `collision_monitor_node` | amr_simulation | Python | own | `src/amr_simulation/amr_simulation/collision_monitor_node.py` | [→](amr_simulation.md#2-collision_monitor_node) |
| `payload_manager_node` | amr_simulation | Python | own | `src/amr_simulation/amr_simulation/payload_manager_node.py` | [→](amr_simulation.md#3-payload_manager_node) |
| `lifecycle_watchdog` | amr_bringup | Python | own | `src/amr_bringup/amr_bringup/lifecycle_watchdog.py` | [→](amr_bringup.md#1-lifecycle_watchdog) |
| `pose_error_logger` | amr_evaluation | Python | own | `src/amr_evaluation/amr_evaluation/pose_error_logger.py` | [→](amr_bringup.md#3-pose_error_logger) |
| `cte_logger` | amr_evaluation | Python | own | `src/amr_evaluation/amr_evaluation/cte_logger.py` | [→](amr_bringup.md#4-cte_logger) |
| `response_time_logger` | amr_evaluation | Python | own | `src/amr_evaluation/amr_evaluation/response_time_logger.py` | [→](amr_bringup.md#5-response_time_logger) |
| `cpu_sampler` | amr_evaluation | Python | own | `src/amr_evaluation/amr_evaluation/cpu_sampler.py` | [→](amr_bringup.md#6-cpu_sampler) |
| `world_gate` | amr_bringup | Python | tool | `src/amr_bringup/amr_bringup/world_gate.py` | [→](amr_bringup.md#2-world_gate-ros-노드-아님) |
| `gz_world.py` | amr_description | Python | tool | `src/amr_description/scripts/gz_world.py` | [→](amr_bringup.md#7-gz_worldpy-ros-노드-아님) |

외부(`ext`) 노드 — 우리가 설정만 작성한다: `map_server`, `map_saver_server`, `amcl`, `ekf_filter_node_odom`,
`ekf_filter_node_map`, `slam_toolbox`, `planner_server`, `controller_server`, `behavior_server`,
`bt_navigator`, `lifecycle_manager*`, `robot_state_publisher`, `ros_gz_bridge`.
이들의 인터페이스는 상위 패키지가 아니라 해당 외부 패키지의 문서를 따른다 —
우리가 정한 **이름·리맵·프레임**만 각 패키지 문서에 적었다.

## 4. 명령 사슬 (cmd_vel 계보)

토픽 이름이 비슷해서 헷갈리기 쉬운 부분이라 따로 적는다. 근거는 각 노드 문서.

```
controller_server ─(remap cmd_vel→cmd_vel_nav)─┐
behavior_server   ─(remap cmd_vel→cmd_vel_nav)─┤
docking_server_node ───────────────────────────┴─► cmd_vel_nav
                                                     │
                                       velocity_profiler_node (50 Hz, 속도·가속·저크 + PID)
                                                     │
                                                     ▼
                                              cmd_vel_smoothed
                                                     │
                                          safety_node (50 Hz, 영역·TTC·E-stop 게이트)
                                                     │
                                                     ▼
                                                 cmd_vel ─► ros_gz_bridge ─► Gazebo DiffDrive
```

`cmd_vel` 의 발행자는 **`safety_node` 하나뿐**이다 (`safety_node.cpp:209-210`, `topics.cmd_out` 기본값 `cmd_vel`).
`navigation.launch.py:139,141` 가 Nav2 두 서버의 `cmd_vel` 을 `cmd_vel_nav` 로 리맵한다.

## 5. 확인 못 함 (전체)

패키지별 세부는 각 문서 끝을 본다. 문서 전체에 걸친 미확인 항목:

- **`ros2 node info` 로 대조하지 않았다.** 이 작업은 정적 분석만 하도록 제한되었다 (같은 워크트리에서
  Gazebo 측정이 돌고 있어 빌드·실행 금지). 런타임에서 실제로 뜨는 이름·QoS 는 재확인이 필요하다.
- **외부 노드(Nav2 / AMCL / robot_localization / slam_toolbox / ros_gz_bridge)의 전체 인터페이스**는
  적지 않았다. 우리가 설정한 이름·리맵·프레임만 적었다.
- **`ros_gz_bridge` 의 브리지 토픽 목록**은 `src/amr_simulation` 의 브리지 설정에서 생성되는데,
  이 문서에서는 개별 항목까지 추출하지 않았다 ([amr_simulation.md](amr_simulation.md) "확인 못 함" 참조).
- **`amr_navigation` 의 오프라인 도구**(`bench_planners.py`, `closed_loop_eval.py`, `kinematic_sim.py`,
  `make_warehouse_map.py`)와 `amr_localization` 의 분석 CLI(`drift_report`, `map_quality`, `world_to_map`,
  `odom_drift_experiment`)는 운용 스택 노드가 아니라 실험 도구라 인터페이스 표를 만들지 않았다.
  이 중 `kinematic_sim.py`, `closed_loop_eval.py`, `odom_drift_experiment` 는 ROS 노드로 동작하며
  토픽을 쓴다 — 필요하면 각 소스의 `create_publisher`/`create_subscription` 을 직접 본다.
