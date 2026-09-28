# API — `amr_fleet`

> 명세 4장 9절 (다중 로봇 · 중앙 집중식 작업 할당 · 교통 관리 · 교착 해소). [← 색인](README.md)
> 알고리즘 근거: [../algorithms/deadlock.md](../algorithms/deadlock.md),
> [../research/fleet-traffic-deadlock/](../research/fleet-traffic-deadlock/fleet-traffic-deadlock.md).
> 런치: `src/amr_fleet/launch/fleet_manager.launch.py`.

## 0. 이 패키지의 노드와 네임스페이스

| 노드 | 언어 | 네임스페이스 | 개수 | 런치 행 |
| --- | --- | --- | --- | --- |
| `fleet_manager_node` | Python | **`/fleet`** | 1 | `:70-73` |
| `traffic_manager_node` | Python | **`/fleet`** | 1 | `:81-85` (인자 `traffic_manager`) |
| `fleet_adapter_node` | Python | **`/<robot_id>`** (로봇마다 1개) | N | `:92-95` (인자 `launch_adapters`) |

`fleet_manager_node` 와 `traffic_manager_node` 는 `/fleet` 네임스페이스에 뜨므로,
코드의 상대 이름 `status` 는 실제로 `/fleet/status` 가 된다.
`traffic_manager_node` 는 일부 토픽을 **절대 이름으로 하드코딩**한다 (`/fleet/traffic_events`,
`/fleet/alerts`, `/fleet/task_events`) — 같은 결과지만 코드 표기가 다르다.

### QoS 상수

| 이름 | 정의 | 정의 위치 |
| --- | --- | --- |
| `qos_events` / `EVENTS_QOS` | depth 100, RELIABLE | `fleet_manager_node.py:204`, `traffic_manager_node.py:65` |
| `LATCHED_QOS` | depth 1, RELIABLE, TRANSIENT_LOCAL | `traffic_manager_node.py:63-64`, `fleet_adapter_node.py:50-51` |
| `ODOM_QOS` | depth 5, BEST_EFFORT | `traffic_manager_node.py:67` |

---

## 1. `fleet_manager_node`

- **소스**: `src/amr_fleet/amr_fleet/fleet_manager_node.py` · Python
  (진입점 `src/amr_fleet/scripts/fleet_manager_node`)
- **역할**: 중앙 작업 할당기. JSON 작업 요청을 받아 스케줄링·할당하고, 로봇별 `assign_task` 서비스를
  호출한다. 통신 지연·유실(명세 최대 100 ms)을 모델링하고 KPI(처리량·가동률)를 집계한다.

### 발행 토픽

| 토픽 (상대) | 전역 이름 | 타입 | QoS | 주기 | 근거 |
| --- | --- | --- | --- | --- | --- |
| `task_events` | `/fleet/task_events` | `amr_msgs/Task` | `qos_events` (depth 100, RELIABLE) | 상태 변화 시 | `:205` |
| `status` | `/fleet/status` | `amr_msgs/FleetStatus` | 기본 (10) | `status_period_s` = 1 Hz | `:206`, `:223` |
| `alerts` | `/fleet/alerts` | `diagnostic_msgs/DiagnosticArray` | `qos_events` | 이벤트 시 | `:207` |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `task_request` (`/fleet/task_request`) | `std_msgs/String` (JSON) | depth 50 | `:208` |
| `traffic_events` (`/fleet/traffic_events`) | `diagnostic_msgs/DiagnosticArray` | depth 20 | `:209` |
| `/<robot_id>/robot_state` | `amr_msgs/RobotState` | 기본 (10) | `:349-351` (로봇 등록 시 동적 생성) |
| `/<robot_id>/task_status` | `amr_msgs/Task` | depth 20 | `:352-354` (동적) |

### 서비스

| 종류 | 서비스 | 타입 | 근거 |
| --- | --- | --- | --- |
| 서버 | `assign_task` (`/fleet/assign_task`) | `amr_msgs/srv/AssignTask` | `:210` |
| 클라이언트 | `/<robot_id>/assign_task` | `amr_msgs/srv/AssignTask` | `:355` (로봇마다 동적 생성) |

액션 없다.

### 타이머

| 목적 | 파라미터 | 기본값 | 근거 |
| --- | --- | --- | --- |
| 할당 주기 | `allocation_period_s` | `0.5` s | `:222` |
| 상태 발행 | `status_period_s` | `1.0` s | `:223` |
| 로봇 탐색 | `discovery_period_s` | `2.0` s | `:225` (`discover_robots` 일 때만) |

### 파라미터

전부 `_declare_params` (`:240-276`) 의 `defaults` 사전에서 왔다.
`comm_latency_ms` 와 `robot_ids` 는 `dynamic_typing=True` 다 (`:274-275`).

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robot_ids` | string \| string[] | `''` | 242 | `"amr_01,amr_02"` 또는 배열. 비우면 탐색만 |
| `discover_robots` | bool | `True` | 243 | `/<id>/robot_state` 로 자동 등록 |
| `allocation_strategy` | string | `nearest` | 244 | 할당 전략 |
| `allocation_batch_size` | int | `0` | 245 | 0 = 가용 로봇 수만큼 |
| `nominal_speed` | double | `1.0` | 246 | [m/s] 이동 시간 추정용 |
| `simulate_latency` | bool | `True` | 247 | false = 지연·유실 없이 호출 (실기) |
| `comm_latency_ms` | double[2] | `[0.0, 100.0]` | 248 | 호출마다 U(lo, hi) ms |
| `drop_rate` | double | `0.0` | 249 | 호출 유실 확률 |
| `seed` | int | `0` | 250 | 0 = 무작위 |
| `log_dir` | string | `''` | 251 | `''` = `$ROS_WS/logs` |
| `log_time_format` | string | `iso` | 252 | `iso` \| `epoch` |
| `allocation_period_s` | double | `0.5` | 253 | [s] |
| `allocation_batch_window_s` | double | `0.01` | 254 | [s] JSON 도착 후 모으는 시간 (서비스는 즉시) |
| `status_period_s` | double | `1.0` | 255 | [s] |
| `discovery_period_s` | double | `2.0` | 256 | [s] |
| `robot_state_timeout_s` | double | `5.0` | 257 | [s] 넘으면 후보 제외 + 오프라인 |
| `assign_timeout_s` | double | `3.0` | 258 | [s] 서비스 응답 대기 상한 |
| `assign_reconcile_s` | double | `2.0` | 259 | [s] 보류 후 "수락 안 함" 확인 시간 |
| `reject_backoff_s` | double | `1.0` | 260 | [s] 거절한 로봇 재시도 대기 |
| `max_task_retries` | int | `0` | 261 | 실패 후 재시도 횟수 (0 = 재할당 없음) |
| `task_timeout_s` | double | `600.0` | 262 | [s] 진행 중 작업 상한 (0 = 없음) |
| `throughput_window_s` | double | `1800.0` | 263 | [s] 처리량 슬라이딩 창 (30분) |
| `utilization_mode` | string | `assigned` | 264 | `assigned` \| `moving` |
| `task_schema_path` | string | `''` | 265 | 작업 JSON 스키마 |
| `robot_params_path` | string | `''` | 266 | 물품 질량표 출처 |
| `scheduler.band_width` | int | `32` | 267 | 우선순위 밴드 폭 |
| `scheduler.deadline_weight` | double | `1.0` | 268 | 마감 가중 |
| `scheduler.fifo_weight` | double | `0.0` | 269 | FIFO 가중 |
| `scheduler.age_boost_rate` | double | `0.5` | 270 | 대기 시간 가산율 |
| `scheduler.age_boost_max` | double | `64.0` | 271 | 대기 가산 상한 |

`use_sim_time` 은 표준 파라미터로 읽는다 (`:227`) — `/clock` 없이 `use_sim_time` 이면 타이머가
돌지 않으므로 벽시계 감시를 붙인다 (`:227-232`).
`STARTUP_ONLY_PARAMS` 와 `scheduler.*` 는 **실행 중 변경이 거절된다** (`:281-285`).

---

## 2. `traffic_manager_node`

- **소스**: `src/amr_fleet/amr_fleet/traffic_manager_node.py` · Python
- **역할**: 로봇들의 남은 경로를 예측해 충돌·교착을 찾고, 로봇별 정지(`traffic/hold`) ·
  양보 자세(`traffic/yield_pose`) · 진입 금지 마스크(`keepout_mask`)를 낸다.
- **주기**: `update_rate_hz` = 2 Hz (`:188-189`).

### 발행 토픽

전역 (절대 이름 하드코딩)

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `/fleet/traffic_events` | `diagnostic_msgs/DiagnosticArray` | `EVENTS_QOS` | `:178-179` |
| `/fleet/alerts` | `diagnostic_msgs/DiagnosticArray` | `EVENTS_QOS` | `:180` |

로봇별 (`base` = `/<robot_id>`, `_add_robot` 에서 동적 생성)

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `/<id>/traffic/hold` | `std_msgs/Bool` | **latched** | `:252` |
| `/<id>/traffic/yield_pose` | `geometry_msgs/PoseStamped` | **latched** | `:253-254` |
| `/<id>/keepout_mask` | `nav_msgs/OccupancyGrid` | **latched** | `:255` |
| `/<id>/costmap_filter_info` | `nav2_msgs/CostmapFilterInfo` | **latched** | `:256-257` |

### 구독 토픽

| 토픽 | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `/map` | `map_topic` | `nav_msgs/OccupancyGrid` | **latched** | `:181` |
| `/fleet/task_events` | — (하드코딩) | `amr_msgs/Task` | `EVENTS_QOS` | `:182` |
| `/<id>/plan` | — | `nav_msgs/Path` | depth 5 | `:246` |
| `/<id>/odometry/filtered_map` | — | `nav_msgs/Odometry` | `ODOM_QOS` (BEST_EFFORT, depth 5) | `:248-249` |
| `/<id>/robot_state` | — | `amr_msgs/RobotState` | 기본 (10) | `:250` |

서비스·액션 없다.

### 파라미터

노드 고유 (`_declare_params`, `:198-208`)

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robot_ids` | string \| string[] | `''` | 200 | `"amr_01,amr_02"` 또는 배열 |
| `update_rate_hz` | double | `2.0` | 201 | [Hz] 평가 주기 |
| `map_topic` | string | `/map` | 202 | |
| `frame_id` | string | `map` | 203 | 경로·자세·마스크 프레임 |

`TrafficConfig` 에서 온 것 (`:205` 가 `TrafficConfig().to_flat()` 를 합친다).
정의: `src/amr_fleet/amr_fleet/traffic_manager.py:145-161`.

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `mode` | string | `active` | `traffic_manager.py:149` | 교통 관리 모드 |
| `robot_radius` | double | `0.36` | `:150` | [m] 풋프린트 외접 반경 |
| `passage_radius` | double | `0.22` | `:151` | [m] 로봇 중심이 지날 수 있는 벽까지 거리 |
| `nominal_speed` | double | `1.0` | `:152` | [m/s] (`fleet.yaml` 과 같은 값) |
| `goal_tolerance_m` | double | `0.3` | `:153` | [m] |
| `obs_timeout_s` | double | `3.0` | `:154` | [s] 넘으면 stale (마지막 자세·토큰 유지) |
| `lost_timeout_s` | double | `5.0` | `:155` | [s] 넘으면 lost |
| `max_path_m` | double | `60.0` | `:156` | [m] 남은 경로를 이만큼만 본다 |
| `zones_file` | string | `''` | `:157` | 구역·포켓 yaml 경로 (`''` = 없음) |
| `prediction.*` | — | `PredictionConfig` | `:158` | 경로 예측 (충돌 예측 창, 정면·추종 각도 등) |
| `zones.*` | — | `ZonesConfig` | `:159` | 구역 설정 |
| `deadlock.*` | — | `DeadlockConfig` | `:160` | 교착 탐지 |
| `resolution.*` | — | `ResolutionConfig` | `:161` | 교착 해소 전략 |

> 그룹 파라미터(`prediction.*`, `zones.*`, `deadlock.*`, `resolution.*`)의 **개별 이름과 기본값**은
> 각 dataclass 정의에 있다. 이 문서에서는 그룹 단위까지만 확인했다 (아래 "확인 못 함").
> 실제 운용값은 런치 인자 `traffic_params_file` 이 가리키는 yaml 이 단일 출처다 (`fleet_manager.launch.py:84`).
> `resolution` 과 `prediction` 의 일부 값은 `_sync()` (`:166-174`) 가 상위 값에서 파생시킨다 —
> yaml 에서 따로 주지 않는다.

---

## 3. `fleet_adapter_node`

- **소스**: `src/amr_fleet/amr_fleet/fleet_adapter_node.py` · Python
- **역할**: 로봇 1대의 온보드 상태(자세·배터리·작업·단계·E-stop·안전 영역)를 모아
  `robot_state` 를 2 Hz 로 낸다. 통신 지연·유실 모델을 적용한다.
- **네임스페이스**: `/<robot_id>` (로봇마다 1개, `fleet_manager.launch.py:92-95`).

### 발행 토픽

| 토픽 | 타입 | QoS | 주기 | 근거 |
| --- | --- | --- | --- | --- |
| `robot_state` | `amr_msgs/RobotState` | 기본 (10) | `publish_rate_hz` = 2 Hz | `:97`, `:115` |
| `task_status` | `amr_msgs/Task` | 기본 (10) | 모의 완료 시 | `:99-100` — **`auto_complete_after_s > 0` 일 때만** (테스트용). 운용에서는 `task_executor_node` 가 낸다 |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `odometry/filtered_map` | `nav_msgs/Odometry` | 기본 (10) | `:101` |
| `battery_state` | `sensor_msgs/BatteryState` | 기본 (10) | `:102` |
| `task_status` | `amr_msgs/Task` | 기본 (10) | `:103` |
| `executor/phase` | `std_msgs/String` | 기본 (10) | `:104` |
| `safety/estop_active` | `std_msgs/Bool` | **latched** | `:105` |
| `safety/zone` | `std_msgs/UInt8` | 기본 (10) | `:107` — volatile 구독은 latched·volatile 발행자 모두와 맞는다 (`:106` 주석) |

### 서비스 (서버)

| 서비스 | 타입 | 근거 | 조건 |
| --- | --- | --- | --- |
| `assign_task` | `amr_msgs/srv/AssignTask` | `:110` | **`serve_assign_task:=true` 일 때만** (모의 서버). 운용에서는 `task_executor_node` 가 이 서비스를 연다 |

액션 없다. 토픽 이름은 모두 하드코딩이다.

### 파라미터

`defaults` 사전 (`:58-69`). `comm_latency_ms` 만 `dynamic_typing=True` (`:71`).

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robot_id` | string | `''` | 59 | `''` = 네임스페이스에서 유도 (`/amr_01` → `amr_01`) |
| `publish_rate_hz` | double | `2.0` | 60 | [Hz] |
| `simulate_latency` | bool | `True` | 61 | false = 지연 없이 발행 (실기) |
| `comm_latency_ms` | double[2] | `[0.0, 100.0]` | 62 | 메시지마다 U(lo, hi) ms |
| `drop_rate` | double | `0.0` | 63 | `robot_state` 유실 확률 |
| `seed` | int | `0` | 64 | 0 = 무작위 |
| `auto_complete_after_s` | double | `0.0` | 65 | > 0 이면 수락 N s 뒤 COMPLETED 를 흉내 (테스트) |
| `serve_assign_task` | bool | `False` | 66 | true = 모의 `assign_task` 서버 |
| `initial_battery` | double | `100.0` | 67 | [%] `battery_state` 가 없을 때 보고할 잔량 |
| `log_dir` | string | `''` | 68 | `''` = `$ROS_WS/logs` |

> 런치가 `auto_complete_after_s > 0` 인데 `serve_assign_task` 를 안 주면 자동으로 `True` 로 맞춘다
> (`fleet_manager.launch.py:88-90`). 노드도 반대 경우를 경고한다 (`:111-113`).

---

## 4. 런치 인자 (`fleet_manager.launch.py`)

확인한 것: `robot_ids`(쉼표 목록), `params_file`, `use_sim_time`, `traffic_manager`(bool),
`traffic_mode`, `traffic_params_file`, `launch_adapters`(bool),
그리고 `_MANAGER_ARGS` / `_ADAPTER_ARGS` 로 노드 파라미터를 덮어쓰는 인자들.
각 인자의 `DeclareLaunchArgument` 기본값 행은 확인하지 않았다.

---

## 5. 확인 못 함

- `TrafficConfig` 의 **그룹 파라미터 개별 이름·기본값** (`prediction.*`, `zones.*`, `deadlock.*`,
  `resolution.*`). 그룹이 있다는 것과 `to_flat()` 이 `'<group>.<name>'` 키를 만든다는 것
  (`traffic_manager.py:181-192`) 만 확인했다. 각 dataclass 정의
  (`PredictionConfig`, `ZonesConfig`, `DeadlockConfig`, `ResolutionConfig` — `traffic_resolution.py`,
  `traffic_prediction.py`, `traffic_zones.py`) 까지 추적하지 않았다.
- `fleet_manager_node` 의 `STARTUP_ONLY_PARAMS` **목록 내용**을 확인하지 않았다 (`:281` 에서 쓰인다).
- `fleet_manager.launch.py` 의 `_MANAGER_ARGS` / `_ADAPTER_ARGS` **목록 내용**과
  `DeclareLaunchArgument` 기본값을 확인하지 않았다.
- `allocation_strategy` 로 고를 수 있는 **전략 이름 목록** (`available_strategies()`, `:196-199`) 을
  확인하지 않았다. 기본값 `nearest` 만 확인했다.
- `traffic_manager_node` 가 내는 `keepout_mask` / `costmap_filter_info` 를 **Nav2 코스트맵 필터가
  실제로 구독하도록 설정되어 있는지** `nav2_params.yaml` 에서 대조하지 않았다
  (`costmap.md §1` 은 `keepout_filter` 를 전역 코스트맵 필터로 적고 있다).
