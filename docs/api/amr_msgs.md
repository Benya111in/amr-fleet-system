# 부록 — `amr_msgs` 커스텀 인터페이스 정의

> [← 색인](README.md)
> 소스: `src/amr_msgs/msg/*.msg`, `src/amr_msgs/srv/*.srv`, `src/amr_msgs/action/*.action`.
> 빌드: `src/amr_msgs/CMakeLists.txt` 가 `file(GLOB … CONFIGURE_DEPENDS)` 로 세 디렉터리를 모두 모아
> `rosidl_generate_interfaces` 에 넘긴다 (`:16-24`) — 파일을 추가하면 재구성 없이 다음 빌드에 반영된다.
> 의존: `std_msgs`, `geometry_msgs`, `builtin_interfaces` (`:23`).

## 0. 목록

| 종류 | 이름 | 파일 | 쓰는 곳 |
| --- | --- | --- | --- |
| msg | `DetectedObject` | `msg/DetectedObject.msg` | `object_localizer_node` |
| msg | `DetectedObjectArray` | `msg/DetectedObjectArray.msg` | `object_localizer_node` → `detection_marker_node`, `task_executor_node` |
| msg | `TrackedObstacle` | `msg/TrackedObstacle.msg` | `obstacle_tracker_node` |
| msg | `TrackedObstacleArray` | `msg/TrackedObstacleArray.msg` | `obstacle_tracker_node` → `safety_node`, DWA, BT, `costmap_scan_filter_node`, `obstacle_truth_node` |
| msg | `RobotState` | `msg/RobotState.msg` | `fleet_adapter_node` → `fleet_manager_node`, `traffic_manager_node` |
| msg | `Task` | `msg/Task.msg` | `fleet_manager_node`, `task_executor_node`, `dashboard_node` |
| msg | `FleetStatus` | `msg/FleetStatus.msg` | `fleet_manager_node` → `dashboard_node` |
| srv | `AssignTask` | `srv/AssignTask.srv` | `fleet_manager_node` ↔ `task_executor_node` (또는 `fleet_adapter_node`) |
| action | `Dock` | `action/Dock.action` | `task_executor_node` (BT `DockAction`) ↔ `docking_server_node` |

---

## 1. 메시지

### `amr_msgs/msg/DetectedObject`

YOLOv8 인식 결과 + Pinhole 모델로 변환한 3D 위치 (명세 6장).

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `header` | `std_msgs/Header` | 2 | |
| `class_name` | `string` | 4 | `"box"`, `"person"`, `"sign"` 등 3종 이상 |
| `class_id` | `uint32` | 5 | |
| `confidence` | `float32` | 6 | |
| `bbox_x` | `uint32` | 9 | 2D 바운딩 박스 좌상단 x [px] |
| `bbox_y` | `uint32` | 10 | 좌상단 y [px] |
| `bbox_width` | `uint32` | 11 | [px] |
| `bbox_height` | `uint32` | 12 | [px] |
| `pose_3d` | `geometry_msgs/PoseStamped` | 14 | `map` 프레임 3D 좌표 |
| `distance` | `float32` | 15 | 로봇 기준 거리 [m] |

### `amr_msgs/msg/DetectedObjectArray`

| 필드 | 타입 | 행 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 1 |
| `objects` | `DetectedObject[]` | 2 |

### `amr_msgs/msg/TrackedObstacle`

칼만 필터 기반 동적 장애물 추적 결과 (명세 7장).

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `header` | `std_msgs/Header` | 2 | |
| `track_id` | `uint32` | 4 | |
| `position` | `geometry_msgs/Point` | 5 | |
| `velocity` | `geometry_msgs/Vector3` | 6 | 추정 속도 [m/s] |
| `heading` | `float32` | 7 | 이동 방향 [rad] |
| `confidence` | `float32` | 8 | 추적 신뢰도 0~1 |
| `is_dynamic` | `bool` | 9 | 동적/정적 분류 결과 |
| `time_to_collision` | `float32` | 10 | TTC [s], 충돌 없으면 `inf` |

> `is_dynamic` 은 `safety_node` 의 `ttc.only_dynamic`(기본 `true`) 이 쓴다 —
> 정적 벽 때문에 0.60 m 통로에서 정지하던 문제의 해법이다
> ([amr_perception.md §7](amr_perception.md#7-safety_node)).

### `amr_msgs/msg/TrackedObstacleArray`

| 필드 | 타입 | 행 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 1 |
| `obstacles` | `TrackedObstacle[]` | 2 |

### `amr_msgs/msg/RobotState`

Fleet 대시보드용 로봇 상태 (명세 9장).

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `header` | `std_msgs/Header` | 2 | |
| `robot_id` | `string` | 4 | 네임스페이스와 같은 문자열 (`"amr_01"`) |
| `pose` | `geometry_msgs/PoseStamped` | 5 | |
| `battery_level` | `float32` | 6 | 0~100 [%] |
| `current_task_id` | `string` | 7 | |
| `status` | `uint8` | 8 | 아래 상수 |

상수 (`:10-16`)

| 상수 | 값 | 행 |
| --- | --- | --- |
| `STATUS_IDLE` | 0 | 10 |
| `STATUS_MOVING` | 1 | 11 |
| `STATUS_DOCKING` | 2 | 12 |
| `STATUS_LOADING` | 3 | 13 |
| `STATUS_CHARGING` | 4 | 14 |
| `STATUS_ERROR` | 5 | 15 |
| `STATUS_ESTOP` | 6 | 16 |

### `amr_msgs/msg/Task`

JSON Task Description 에 대응하는 작업 정의 (명세 8장).

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `header` | `std_msgs/Header` | 2 | |
| `task_id` | `string` | 4 | |
| `robot_id` | `string` | 5 | 미할당 시 빈 문자열 |
| `priority` | `uint8` | 6 | 0(낮음) ~ 255(높음) |
| `deadline` | `builtin_interfaces/Time` | 7 | |
| `pickup_pose` | `geometry_msgs/PoseStamped` | 9 | |
| `dropoff_pose` | `geometry_msgs/PoseStamped` | 10 | |
| `item_type` | `string` | 12 | `"small"`, `"medium"`, `"large"` |
| `item_mass` | `float32` | 13 | [kg] — 적재 시 동역학 반영 |
| `status` | `uint8` | 15 | 아래 상수 |

상수 (`:17-20`)

| 상수 | 값 | 행 |
| --- | --- | --- |
| `STATUS_PENDING` | 0 | 17 |
| `STATUS_IN_PROGRESS` | 1 | 18 |
| `STATUS_COMPLETED` | 2 | 19 |
| `STATUS_FAILED` | 3 | 20 |

### `amr_msgs/msg/FleetStatus`

시스템 전체 KPI (명세 9장 모니터링).

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `header` | `std_msgs/Header` | 2 | |
| `robots` | `RobotState[]` | 4 | |
| `tasks_pending` | `uint32` | 5 | |
| `tasks_in_progress` | `uint32` | 6 | |
| `tasks_completed` | `uint32` | 7 | |
| `tasks_failed` | `uint32` | 8 | |
| `throughput` | `float32` | 10 | 작업 처리량 [tasks/hour] |
| `avg_task_duration` | `float32` | 11 | 평균 작업 시간 [s] |
| `robot_utilization` | `float32` | 12 | 로봇 가동률 0~1 |
| `deadlock_count` | `uint32` | 13 | |

---

## 2. 서비스

### `amr_msgs/srv/AssignTask`

중앙 집중식 작업 할당 (명세 9장).

**요청**

| 필드 | 타입 | 행 |
| --- | --- | --- |
| `task` | `Task` | 2 |

**응답**

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `success` | `bool` | 4 | 수락 여부 |
| `robot_id` | `string` | 5 | 수락한 로봇 (`"amr_01"`) |
| `message` | `string` | 6 | 거절 사유 등 |

서버: `task_executor_node` (`assign_task`, `task_executor_node.cpp:344-345`),
`fleet_manager_node` (`/fleet/assign_task`, `fleet_manager_node.py:210`),
`fleet_adapter_node` (모의 서버, `serve_assign_task:=true` 일 때만, `fleet_adapter_node.py:110`).
클라이언트: `fleet_manager_node` → `/<robot_id>/assign_task` (`fleet_manager_node.py:355`).

---

## 3. 액션

### `amr_msgs/action/Dock`

ArUco 마커 기반 정밀 도킹 (명세 8장: 위치 2 cm, 각도 1° 이내).

**Goal**

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `dock_id` | `string` | 2 | 도크 식별자 (`docking_server_node` 의 `docks.<id>.*` 와 대응) |
| `approach_pose` | `geometry_msgs/PoseStamped` | 3 | 접근 자세 |
| `max_retries` | `uint8` | 4 | 재시도 상한 (0 이면 노드 파라미터 `max_attempts` 사용) |

**Result**

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `success` | `bool` | 6 | |
| `final_position_error` | `float32` | 7 | [m] |
| `final_angle_error` | `float32` | 8 | [rad] |
| `attempts_used` | `uint8` | 9 | |

**Feedback**

| 필드 | 타입 | 행 | 설명 |
| --- | --- | --- | --- |
| `current_phase` | `string` | 11 | 진행 단계 |
| `distance_remaining` | `float32` | 12 | [m] |
| `attempt` | `uint8` | 13 | 현재 시도 번호 |

서버: `docking_server_node` (`dock`, `docking_server_node.cpp:229-233`).
클라이언트: `task_executor_node` 의 BT `DockAction` (`bt_nodes/dock.hpp:23,27`).

---

## 4. 확인 못 함

- **`Task` 메시지 ↔ JSON Task Description 의 필드 대응 규칙**(스키마)은 이 문서 범위 밖이다.
  `src/amr_fleet/amr_fleet/task_schema.py` 와 `task_msg.py` 가 변환을 맡고,
  `fleet_manager_node` 의 `task_schema_path` 파라미터가 스키마 파일을 가리킨다.
- `DetectedObject.class_name` / `class_id` 의 **허용 값 목록**은
  `src/amr_perception/config/classes.yaml` 에 있다 (이 문서에서 대조하지 않았다).
- `Dock.action` 의 `current_phase` 문자열이 가질 수 있는 **값 목록**을 확인하지 않았다.
- `RobotState.status` 와 `task_executor_node` 의 `executor/phase` 문자열의 **대응 관계**를
  코드로 확인하지 않았다 (`fleet_adapter_node` 가 변환한다).
