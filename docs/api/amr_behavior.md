# API — `amr_behavior`

> 명세 4장 8절 (작업 수행 · BT · 정밀 도킹). [← 색인](README.md)
> 알고리즘 근거: [../algorithms/behavior_tree.md](../algorithms/behavior_tree.md),
> [../algorithms/docking.md](../algorithms/docking.md).
> 런치: `src/amr_behavior/launch/behavior.launch.py` — 3개 노드를 `namespace` 인자 아래 띄운다 (`:85-100`).
> **설정 파일**: `src/amr_behavior/config/behavior.yaml` (+ `config/robot_params.yaml` 이 먼저 들어가고
> `behavior.yaml` 이 덮어쓴다, `:80`).
> 행동 트리 XML: `src/amr_behavior/behavior_trees/task_executor.xml` 과 `subtrees/*.xml`.

## 0. 이 패키지의 노드

| 노드 | 언어 | 형태 | 소스 | 런치 플래그 |
| --- | --- | --- | --- | --- |
| `task_executor_node` | C++ | own | `src/task_executor_node.cpp` (+ `src/task_executor_main.cpp`) | `start_executor` (`:91-95`) |
| `docking_server_node` | C++ | own | `src/docking/docking_server_node.cpp` (+ `docking_server_main.cpp`) | `start_docking` (`:86-90`) |
| `battery_model_node` | Python | own | `amr_behavior/battery_model_node.py` | `start_battery_model` (`:96-100`) |
| `IsTTCBelowThreshold` | C++ | plugin (BT 조건) | `src/plugins/is_ttc_below_threshold_condition.cpp` | `bt_navigator` 가 로드 (`amr_navigation` 런치 인자 `nav_ttc_bt`) |
| `bt_tool` | C++ | tool | `src/bt_tool.cpp` | — (오프라인 BT 검사 도구) |

---

## 1. `task_executor_node`

- **소스**: `src/amr_behavior/src/task_executor_node.cpp` · C++ (BehaviorTree.CPP)
- **역할**: 작업 수락(`assign_task`) → 행동 트리 tick → 이동·인지·도킹·적재·충전·복구 조율 →
  `task_status` 보고. 충전소 점유는 `/fleet/charger_claims` 로 로봇끼리 조정한다.
- **tick 주기**: `tick_rate_hz` = 50 Hz (`:125`).

### 발행 토픽

| 토픽 | 타입 | QoS | 근거 | 설명 |
| --- | --- | --- | --- | --- |
| `executor/phase` | `std_msgs/String` | **latched** | `:262` | 현재 단계 (`moving`, `docking`, `perceiving`, …) |
| `task_status` | `amr_msgs/Task` | 기본 (10) | `:263` | 작업 상태 보고 → `fleet_manager_node` |
| `payload/attach` | `std_msgs/String` | **latched** | `:264` | 적재/하역 요청 → `payload_manager_node` (계약 C5) |
| `payload/mass` | `std_msgs/Float32` | **latched** | `:266` | 적재 질량 → `velocity_profiler_node`, DWA, 배터리 모델. `publish_payload_mass:=true` (기본) 일 때만 |
| `charging/enable` | `std_msgs/Bool` | **latched** | `:268` | 충전 시작/중지 → `battery_model_node` |
| `/fleet/charger_claims` | `std_msgs/String` | 기본 (10) | `:244-245`, `:375` | 충전소 점유 선언 (기본값 = 절대 이름, 파라미터 `charger_claims_topic`) |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `battery_state` | `sensor_msgs/BatteryState` | 기본 (10) | `:308-309` |
| `safety/estop_active` | `std_msgs/Bool` | **latched** | `:316-317` |
| `localization/lost` | `std_msgs/Bool` | **latched** | `:323-324` |
| `traffic/hold` | `std_msgs/Bool` | 기본 (10) | `:328-329` |
| `traffic/yield_pose` | `geometry_msgs/PoseStamped` | 기본 (10) | `:332-333` |
| `perception/detected_objects` | `amr_msgs/DetectedObjectArray` | `sensor` | `:335-336` |
| `perception/dock_marker_pose` | `geometry_msgs/PoseStamped` | `sensor` | `:340-341` |
| `/fleet/charger_claims` | `std_msgs/String` | 기본 (10) | `:376` (자기 발행분 포함, 파라미터 `charger_claims_topic`) |

### 서비스 (서버)

| 서비스 | 타입 | 근거 | 설명 |
| --- | --- | --- | --- |
| `assign_task` | `amr_msgs/srv/AssignTask` | `:344-345` | 중앙 할당기(`fleet_manager_node`)의 작업 배정을 수락/거절 |
| `clear_payload` | `std_srvs/srv/Trigger` | `:351-352` | 적재 상태 강제 초기화 |

### 액션 · 서비스 (클라이언트 — 행동 트리 노드가 연다)

BT 노드는 `task_executor_node` 프로세스 안에서 돌기 때문에 이 클라이언트들도 같은 네임스페이스(`/amr_01`)에 붙는다.

| 액션 | 타입 | BT 노드 | 근거 |
| --- | --- | --- | --- |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `NavigateToPoseAction` | `include/amr_behavior/bt_nodes/navigate_to_pose.hpp:18,23` |
| `spin` | `nav2_msgs/action/Spin` | `SpinAction` | `bt_nodes/spin.hpp:18,22` |
| `backup` | `nav2_msgs/action/BackUp` | `BackUpAction` | `bt_nodes/back_up.hpp:15,21` |
| `wait` | `nav2_msgs/action/Wait` | `WaitAction` | `bt_nodes/wait.hpp:15,19` |
| `dock` | `amr_msgs/action/Dock` | `DockAction` | `bt_nodes/dock.hpp:23,27` |

| 서비스 | 타입 | BT 노드 | 근거 |
| --- | --- | --- | --- |
| `global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | `ClearCostmapService` | `bt_nodes/clear_costmap.hpp:15,21-22` (포트 `service_name` 으로 지역 코스트맵 선택 가능) |

### 파라미터

기본·BT

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robot_id` | string | `""` | 114 | `""` = 네임스페이스에서 유도 |
| `bt_xml` | string | `""` | 120 | 행동 트리 XML 경로 (`""` = 패키지 기본) |
| `tick_rate_hz` | double | `50.0` | 125 | [Hz] BT tick 주기 |
| `bt_log_file` | string | `""` | 130 | BT 실행 로그 파일 (`""` = 안 씀) |
| `groot.enabled` | bool | `true` | 126 | Groot 모니터링 |
| `groot.publisher_port` | int | `1666` | 127 | |
| `groot.server_port` | int | `1667` | 128 | |
| `groot.max_msg_per_second` | int | `25` | 129 | |
| `publish_payload_mass` | bool | `true` | 131 | `payload/mass` 발행 여부 |
| `server_wait_timeout_ms` | int | `5000` | 132 | [ms] 액션 서버 대기 |
| `goal_response_timeout_ms` | int | `2000` | 133 | [ms] 목표 수락 응답 대기 |

작업 수락 정책

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `battery_low_percent` | double | (내부 기본값) | 137-138, 210 | [%] 이보다 낮으면 작업 거절·충전 |
| `battery_resume_percent` | double | (내부 기본값) | 211-212 | [%] 충전 후 작업 재개 문턱 |
| `accept_while_returning` | bool | (내부 기본값) | 139-140 | 복귀 중 새 작업 수락 |
| `allow_undocked_tasks` | bool | (내부 기본값) | 141-142 | 도크 밖에서 시작하는 작업 허용 |

물품 (적재 모델) — `payload_types` 목록을 돌며 타입별로 선언한다 (`:148-157`)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `payload_types` | string[] | (내부 기본 목록) | 148-149 | 물품 타입 이름 목록 |
| `payload.<type>.mass` | double | 타입별 기본값 | 155 | [kg] |
| `payload.<type>.load_time` | double | 타입별 기본값 | 156 | [s] 적재 소요 시간 |

도크 — `dock_ids` 목록을 돌며 도크별로 선언한다 (`:162-188`)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `map_frame` | string | `map` | 162 | |
| `dock_ids` | string[] | (내부 기본 목록) | 163-164 | 도크 id 목록 |
| `<dock>.staging` (배열) | double[] | — | 167 | 도크 앞 대기 자세 |
| `<dock>` 인지 거리 | double | — | 179 | 도크 앞 인지 수행 거리 |
| `dock_match_radius` | double | `1.8` | 187 | [m] 현재 자세 ↔ 도크 매칭 반경 |
| `dock_match_yaw_tolerance` | double | `π` | 188 | [rad] 매칭 헤딩 허용 |

재시도·복구

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `nav_attempts` | int | (내부 기본값) | 192 | 이동 재시도 횟수 |
| `perception_attempts` | int | (내부 기본값) | 193 | 인지 재시도 횟수 |
| `dock_attempts` | int | (내부 기본값) | 194 | 도킹 재시도 횟수 |
| `dock_max_retries` | int | (내부 기본값) | 196 | Dock 액션 goal 의 `max_retries` |
| `recovery_spin_angle` | double | (내부 기본값) | 213 | [rad] 복구 회전각 |
| `recovery_wait_s` | double | (내부 기본값) | 214 | [s] 복구 대기 |
| `recovery_backup_dist` | double | (내부 기본값) | 215-216 | [m] 복구 후진 거리 |
| `perception_class` | string | (내부 기본값) | 207 | 인지 단계에서 찾을 클래스 |
| `perception_max_distance` | double | (내부 기본값) | 208-209 | [m] 인지 유효 거리 |
| `perception_spin_angle` | double | (내부 기본값) | 217-218 | [rad] 인지용 회전각 |
| `dock_backup_dist` | double | (내부 기본값) | 219 | [m] |
| `marker_max_age` | double | (내부 기본값) | 220 | [s] 마커 관측 유효 시간 |
| `undock_dist` | double | (내부 기본값) | 221 | [m] 언도킹 후진 거리 |
| 대기 자세 (배열) | double[] | — | 222 | 작업이 없을 때 갈 자세 |

충전소

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `charger_dock_ids` | string[] | (내부 기본 목록) | 232 | 충전소로 쓸 도크 id. **비어 있으면 자동 충전을 끈다** (`:246-251`) |
| `charger_claim_timeout` | double | `3.0` | 243 | [s] 점유 선언 유효 시간 |
| `charger_claims_topic` | string | `/fleet/charger_claims` | 244-245 | 충전소 점유 조정 토픽 (전역) |

`task_timeout_ms` 등 일부 값은 `:199` 의 헬퍼 루프로 선언된다 — 이름을 한 행으로 지목하지 못했다 (아래 참조).

---

## 2. `docking_server_node`

- **소스**: `src/amr_behavior/src/docking/docking_server_node.cpp` · C++
- **역할**: `amr_msgs/action/Dock` 액션 서버. ArUco 마커 자세를 보고 접근→정렬→진입 제어법을 돌려
  명세 8장의 **위치 2 cm / 각도 1° 이내** 도킹을 수행한다. 도킹 중에는 안전 게이트에
  제외 영역(`safety/dock_exclusion`)을 주고, 마커로 역산한 자세를 `localization/marker_fix` 로 낸다.
- **제어 주기**: `control_rate_hz` = 20 Hz (`:118`).

### 액션 (서버)

| 액션 | 타입 | 근거 |
| --- | --- | --- |
| `dock` | `amr_msgs/action/Dock` | `:229-233` |

메시지 필드는 [amr_msgs.md §3](amr_msgs.md#3-액션) 참조 (goal: `dock_id`, `approach_pose`, `max_retries` /
result: `success`, `final_position_error`, `final_angle_error`, `attempts_used` /
feedback: `current_phase`, `distance_remaining`, `attempt`).

### 발행 토픽

| 토픽 | 타입 | QoS | 근거 | 설명 |
| --- | --- | --- | --- | --- |
| `cmd_vel_nav` | `geometry_msgs/Twist` | `QoS(1)` | `:186` | 도킹 제어 명령 (`velocity_profiler_node` 로 들어간다) |
| `safety/dock_exclusion` | `geometry_msgs/PolygonStamped` | 기본 (10) | `:187-188` | 안전 게이트 제외 영역 (도크 구조물에 정지하지 않게) |
| `localization/marker_fix` | `geometry_msgs/PoseWithCovarianceStamped` | 기본 (10) | `:189-190` | 마커 역산 자세 → `kidnap_monitor_node` |
| `perception/aruco/preferred_id` | `std_msgs/Int32` | **latched** (`QoS(1).reliable().transient_local()`) | `:191-192` | 검출기에 "이 마커를 내라" |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `perception/dock_marker_id` | `std_msgs/Int32` | 기본 (10) | `:203-204` |
| `perception/dock_marker_pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | `sensor` | `:213-214` |
| `perception/dock_marker_pose` | `geometry_msgs/PoseStamped` | `sensor` | `:225-226` |

> **구독 등록 순서가 기능적으로 중요하다** (`:193-202` 주석): 검출기는 자세 → id → 공분산 순으로 내지만
> 단일 스레드 실행기는 한 주기 안에서 **등록 순서**로 콜백을 돌린다. 그래서 id 를 공분산보다 **먼저** 등록한다.

### 서비스 (클라이언트)

| 서비스 | 파라미터 | 타입 | 근거 |
| --- | --- | --- | --- |
| (`detector_enable_service` 값) | `detector_enable_service` (기본 `""` = 사용 안 함) | `std_srvs/srv/SetBool` | `:124`, `:227-229` — `aruco_detector_node` 의 `perception/aruco/enable` 상대 |

토픽 이름은 **`detector_enable_service` 를 뺀 전부가 하드코딩**이다.

### 파라미터

제어 법칙·허용오차

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `control_law` | string | `graceful` | 79 | 제어 법칙 선택 |
| `standoff` | double | (내부 기본값) | 85 | [m] 마커 앞 접근점 거리 |
| `position_tolerance` | double | (내부 기본값) | 86 | [m] 성공 판정 위치 오차 (명세 2 cm) |
| `angle_tolerance` | double | (내부 기본값) | 87 | [rad] 성공 판정 각도 오차 (명세 1°) |
| `settle_frames` | int | (내부 기본값) | 88 | 허용오차 안에 머물러야 하는 프레임 수 |
| `final_distance` | double | (내부 기본값) | 89 | [m] 최종 진입 거리 |
| `stop_distance` | double | (내부 기본값) | 90 | [m] 정지 거리 |
| `heading_stop_tolerance` | double | (내부 기본값) | 91-92 | [rad] |
| `align_threshold` | double | (내부 기본값) | 93 | [rad] 정렬 단계 진입 문턱 |

속도

| 이름 | 행 | 이름 | 행 |
| --- | --- | --- | --- |
| `max_linear_speed` | 94 | `final_linear_speed` | 95 |
| `min_linear_speed` | 96 | `max_angular_speed` | 97 |
| `search_angular_speed` | 98-99 | `search_sweep` | 100 |

제어 이득 (graceful control law)

| 이름 | 행 | 이름 | 행 |
| --- | --- | --- | --- |
| `k_distance` | 101 | `k_heading` | 102 |
| `lookahead` | 103 | `k_phi` | 104 |
| `k_delta` | 105 | `beta` | 106 |
| `lambda` | 107 | `slowdown_radius` | 108 |
| `linear_deadband` | 109 | `angular_deadband` | 110 |

시간 제한·재시도

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `marker_timeout` | double | (내부 기본값) | 111 | [s] 마커가 안 보이는 허용 시간 |
| `search_timeout` | double | (내부 기본값) | 112 | [s] 마커 탐색 제한 |
| `attempt_timeout` | double | (내부 기본값) | 113 | [s] 시도 1회 제한 |
| `backup_distance` | double | (내부 기본값) | 114 | [m] 재시도 전 후진 거리 |
| `backup_speed` | double | (내부 기본값) | 115 | [m/s] |
| `max_overshoot` | double | (내부 기본값) | 116 | [m] |
| `filter_coef` | double | (내부 기본값) | 117 | 마커 자세 필터 계수 |
| `control_rate_hz` | double | `20.0` | 118 | [Hz] |
| `max_attempts` | int | `3` | 120 | goal 이 지정하지 않을 때의 기본 시도 수 |
| `marker_id_max_age` | double | (내부 기본값) | 182 | [s] 마커 id 유효 시간 |

프레임·마커

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `base_frame` | string | `base_link` | 121 | |
| `marker_normal_axis` | string | `x` | 123 | 마커 법선 축 (계약: `+x` 가 정면) |
| `detector_enable_service` | string | `""` | 124 | `""` = 검출기 on/off 안 함 |

도킹 제외 영역 (`exclusion.*`)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `exclusion.enabled` | bool | `true` | 125 | |
| `exclusion.half_width` | double | (내부 기본값) | 126-127 | [m] |
| `exclusion.depth` | double | (내부 기본값) | 128 | [m] |
| `exclusion.front_margin` | double | (내부 기본값) | 129-130 | [m] |
| `exclusion_release_distance` | double | (내부 기본값) | 131-132 | [m] |
| `exclusion_marker_timeout` | double | (내부 기본값) | 133-134 | [s] |

도크별 설정 — `docks` 목록을 돌며 선언한다 (`:136-145`)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `docks` | string[] | (내부 기본 목록) | 136-137 | 도크 id 목록 |
| `docks.<id>.standoff` | double | 전역 `standoff` | 139 | [m] |
| `docks.<id>.marker_id` | int | `-1` | 140 | 이 도크의 ArUco id |
| `docks.<id>` 자세 (배열) | double[] | — | 143 | |

재위치추정 (`relocalization.*`) — 마커로 위치를 되잡는 기능

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `relocalization.enabled` | bool | (내부 기본값) | 153 | |
| `relocalization.max_range` | double | (내부 기본값) | 154 | [m] |
| `relocalization.tolerance` | double | (내부 기본값) | 155 | [m] |
| `relocalization.min_observations` | int | (내부 기본값) | 156-157 | 연속 관측 수 |
| `relocalization.position_sigma` | double | (내부 기본값) | 158-159 | [m] |
| `relocalization.yaw_sigma` | double | (내부 기본값) | 160 | [rad] |
| `relocalization.max_yaw_var` | double | (내부 기본값) | 161-162 | [rad²] |
| 지도 마커 id 목록 | int64[] | — | 166-167 | 위치 표지 마커 id |
| 지도 마커 자세 목록 | double[] | — | 168 | 위 마커들의 지도 상 자세 |

---

## 3. `battery_model_node`

- **소스**: `src/amr_behavior/amr_behavior/battery_model_node.py` · Python
  (진입점 `src/amr_behavior/scripts/battery_model_node`)
- **역할**: 소비 전력을 적분해 `battery_state` 를 낸다. Gazebo 는 배터리를 모사하지 않으므로
  (`components.md §3.1`) 이 노드가 대신한다.

### 발행 / 구독

| 종류 | 토픽 | 타입 | QoS | 주기 | 근거 |
| --- | --- | --- | --- | --- | --- |
| 발행 | `battery_state` | `sensor_msgs/BatteryState` | 기본 (10) | `publish_rate_hz` = 1 Hz | `:44` |
| 구독 | `odometry/filtered` | `nav_msgs/Odometry` | 기본 (10) | — | `:45` |
| 구독 | `payload/mass` | `std_msgs/Float32` | **latched** | — | `:46` |
| 구독 | `charging/enable` | `std_msgs/Bool` | **latched** | — | `:47` |

서비스·액션 없다. 토픽 이름은 모두 하드코딩이다.

### 파라미터

`BatteryParams` dataclass 의 필드가 그대로 파라미터가 된다 (`:35-39` 루프).
기본값 출처: `src/amr_behavior/amr_behavior/battery_model.py:23-35`.

| 이름 | 타입 | 기본값 | 근거 | 설명 |
| --- | --- | --- | --- | --- |
| `capacity_wh` | double | `480.0` | `battery_model.py:26` | [Wh] 팩 용량 |
| `nominal_voltage` | double | `24.0` | `:27` | [V] |
| `idle_power_w` | double | `30.0` | `:28` | [W] 정지 중 소비 (컴퓨터·센서) |
| `drive_power_w_per_mps` | double | `60.0` | `:29` | [W/(m/s)] 공차 주행 |
| `payload_power_w_per_kg_mps` | double | `1.0` | `:30` | [W/(kg·m/s)] 적재 추가 주행 전력 |
| `turn_power_w_per_radps` | double | `10.0` | `:31` | [W/(rad/s)] 제자리 회전 |
| `charge_power_w` | double | `480.0` | `:32` | [W] 충전 (1 C) |
| `drain_scale` | double | `1.0` | `:33` | 방전 배율 (시험 가속용, 충전에는 미적용) |
| `stationary_speed` | double | `0.02` | `:34` | [m/s] 이 이하면 정지로 보고 충전 허용 |
| `initial_percent` | double | `100.0` | `:35` | [%] 시작 잔량 |
| `odom_timeout` | double | `1.0` | `battery_model_node.py:40` | [s] 오도메트리 노후 문턱 |
| `publish_rate_hz` | double | `1.0` | `battery_model_node.py:41` | [Hz] |

---

## 4. `IsTTCBelowThreshold` (BT 조건 플러그인)

- **소스**: `src/amr_behavior/src/plugins/is_ttc_below_threshold_condition.cpp` · C++
- **로드되는 곳**: `bt_navigator` 프로세스 (Nav2 BT 플러그인). `amr_navigation` 런치 인자
  `nav_ttc_bt` 로 켜고 끈다 (`navigation.launch.py:110-114`).
- **역할**: 추적된 동적 장애물의 TTC 가 문턱보다 작으면 조건이 참이 되어 회피 서브트리가 돈다.

### 구독 토픽

| 토픽 (기본값) | BT 입력 포트 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `perception/tracked_obstacles` | `tracked_obstacles_topic` | `amr_msgs/TrackedObstacleArray` | `QoS(10)` (reliable) | `:22-30` |

전용 콜백 그룹(`MutuallyExclusive`, 비자동 추가)에서 돌린다 (`:18-20`).

---

## 5. 런치 인자 (`behavior.launch.py`)

| 인자 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- |
| `namespace` | `''` | 107 | 로봇 네임스페이스 |
| `robot_name` | `''` | 108-109 | `namespace` 의 별칭 (docker-compose 규약) |
| `use_sim_time` | `true` | 110 | |
| `params_file` | (패키지 `config/behavior.yaml`) | 111- | `robot_params.yaml` 뒤에 붙어 덮어쓴다 (`:80`) |
| `start_docking` / `start_executor` / `start_battery_model` | — | — | 노드별 기동 플래그 (`:86`, `:91`, `:96`) |
| `log_level` | — | — | `--ros-args --log-level` 로 전달 (`:82`) |

---

## 6. 확인 못 함

- `task_executor_node` · `docking_server_node` 의 **"(내부 기본값)" 로 표시한 파라미터들**은
  `declare_parameter` 의 두 번째 인자가 구조체 필드(`c.<필드>`, `p.<필드>`)라 코드 상수로 지목하지 못했다.
  구조체 정의 파일(`include/amr_behavior/...`)까지 추적하지 않았다.
  **실제 운용값은 `src/amr_behavior/config/behavior.yaml` 이 단일 출처**다.
- `task_executor_node:199` 의 헬퍼 루프(`declare_parameter<int>(name, fallback)`)로 선언되는
  파라미터 **이름 목록**을 확인하지 못했다. `TaskDeadline(task_timeout_ms)` 가 그중 하나라는 것만
  `docs/reports/` 와 커밋 이력으로 알고 있으나, 이 문서에는 코드로 확인한 것만 적는 원칙이라 넣지 않았다.
- `behavior.launch.py` 의 `start_docking` / `start_executor` / `start_battery_model` /
  `params_file` / `log_level` **기본값 행**을 특정하지 않았다 (`_overrides`, `arg()` 헬퍼 경유).
- 행동 트리 XML(`behavior_trees/task_executor.xml`, `subtrees/*.xml`)의 **노드 구성과 블랙보드 포트**는
  이 문서 범위 밖이다 — [../algorithms/behavior_tree.md](../algorithms/behavior_tree.md) 를 본다.
- BT 조건 노드(`is_battery_ok`, `is_estop_clear`, `is_localized`, `is_object_detected`,
  `is_task_assigned`, `is_traffic_hold`, `is_dock_marker_visible`)는 **블랙보드 값만 읽고
  자체 토픽을 만들지 않는다** — `task_executor_node` 의 구독이 그 값을 채운다. 개별 확인은 하지 않았다.
