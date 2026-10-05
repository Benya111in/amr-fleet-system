# API — `amr_simulation`

> 명세 4장 1·7절 (시뮬레이션 환경 / 동적 장애물). [← 색인](README.md)
> 설계 근거: [../architecture/components.md §3.1](../architecture/components.md).
> 이 패키지의 노드는 **평가용 지면 진실(ground truth)** 을 만들거나 Gazebo 에 조작을 거는 것이라,
> 실기 배포에는 올라가지 않는다. 그래서 토픽이 대부분 `/sim/…` 전역 이름이다.

## 0. 이 패키지의 노드

| 노드 | 언어 | 소스 | 네임스페이스 |
| --- | --- | --- | --- |
| `obstacle_truth_node.py` | Python | `amr_simulation/obstacle_truth_node.py` | 루트 (`/sim/…` 절대 이름) |
| `collision_monitor_node.py` | Python | `amr_simulation/collision_monitor_node.py` | 루트 (`/sim/…`, `/<robot>/…` 절대 이름) |
| `payload_manager_node.py` | Python | `amr_simulation/payload_manager_node.py` | **로봇 네임스페이스** (상대 이름). `amr_description spawn.launch.py` 가 로봇마다 띄운다 |

공통 장애물 파라미터는 `amr_simulation/dynamic_obstacles.py:105-113` 의 `declare_obstacle_params()` 가
선언한다 — `obstacle_truth_node` 와 `collision_monitor_node` 가 같이 쓴다.

### 공통 장애물 파라미터 (`declare_obstacle_params`)

| 이름 | 타입 | 기본값 | 행 (`dynamic_obstacles.py`) | 설명 |
| --- | --- | --- | --- | --- |
| `world_file` | string | `""` | 106 | 월드 SDF 경로 (actor 궤적을 읽는다) |
| `actor_radius` | double | `0.30` | 107 | [m] 사람 actor 원기둥 반경 |
| `actor_height` | double | `1.80` | 108 | [m] 사람 actor 높이 |
| `models` | string[] | `["forklift_main", "shuttle_amr"]` | 109 | 물리 모델 동적 장애물 이름 |
| `model_footprints` | double[8] | `[-1.2, 2.0, -0.64, 0.64, -0.30, 0.30, -0.20, 0.20]` | 110-111 | 모델별 사각형 발자국 `(x_min, x_max, y_min, y_max)` × 2 |
| `model_heights` | double[2] | `[2.2, 0.45]` | 112 | [m] 모델별 높이 |
| `model_odom_topic` | string | `/sim/{name}/odom` | 113 | 모델 오도메트리 토픽 형식 (`{name}` 치환) |

---

## 1. `obstacle_truth_node`

- **소스**: `src/amr_simulation/amr_simulation/obstacle_truth_node.py` · Python
  (진입점 `src/amr_simulation/scripts/obstacle_truth_node.py`)
- **역할**: 동적 장애물의 지면 진실. 사람 actor 궤적을 Gazebo 와 같은 스플라인으로 sim time 보간하고
  지게차·셔틀 오도메트리를 합쳐 발행한다. 명세 4.7 추적기 평가의 기준값이다.
- **주기**: `rate` = 50 Hz 타이머 (`:53`, `:67`).

### 발행 토픽

| 토픽 | 타입 | QoS | 주기 | 근거 |
| --- | --- | --- | --- | --- |
| `/sim/dynamic_obstacles` | `visualization_msgs/MarkerArray` | 기본 (10) | 50 Hz | `:59` |
| `/sim/dynamic_obstacles/tracks` | `amr_msgs/TrackedObstacleArray` | 기본 (10) | 50 Hz | `:60-61` |
| `/sim/dynamic_obstacles/info` | `std_msgs/String` (JSON) | **latched** (RELIABLE + TRANSIENT_LOCAL, depth 1) | 1회 (기동 시) | `:62-66` |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `/sim/<model>/odom` (`model_odom_topic` 형식) | `nav_msgs/Odometry` | 기본 (10) | `:57-58` — `models` 의 모델마다 1개 |

서비스·액션 없다.

### 파라미터

공통 장애물 파라미터(위 표) + 아래 2개.

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `rate` | double | `50.0` | 53 | [Hz] 발행 주기 |
| `frame_id` | string | `world` | 54 | 출력 `header.frame_id` |

---

## 2. `collision_monitor_node`

- **소스**: `src/amr_simulation/amr_simulation/collision_monitor_node.py` · Python
- **역할**: 로봇 발자국 ↔ 동적 장애물(사람 원, 차량 사각형, 다른 로봇)의 부호 거리를 계산하고
  접촉(거리 < 0 m) 사건을 센다. 명세 4.7 "30회 시행 충돌 0건" 판정의 근거다.
  로봇 발자국 크기는 `config/robot_params.yaml` 의 `robot.footprint_*` 에서 읽는다 (`:60-63`).
- **주기**: 요약 타이머 1 Hz (`:81`). 거리 계산은 오도메트리 콜백 구동.

### 발행 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `/<robot>/collision_monitor/min_distance` | `std_msgs/Float64` | 기본 (10) | `:76` — `robots` 의 로봇마다 1개 |
| `/<robot>/collision_monitor/contacts` | `std_msgs/UInt32` | 기본 (10) | `:77` — 로봇마다 1개 |
| `/sim/collision_monitor/events` | `std_msgs/String` | depth 50 | `:79` |
| `/sim/collision_monitor/summary` | `std_msgs/String` | 기본 (10) | `:80`, 1 Hz |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `/sim/<model>/odom` | `nav_msgs/Odometry` | 기본 (10) | `:69-71` — 동적 장애물 모델마다 |
| `/<robot>/ground_truth/odom` | `nav_msgs/Odometry` | depth 50 | `:78` — 로봇마다 |

### 서비스 (서버)

| 서비스 | 타입 | 근거 | 설명 |
| --- | --- | --- | --- |
| `/sim/collision_monitor/reset` | `std_srvs/srv/Trigger` | `:81` | 접촉 카운터·최소 거리 통계 초기화 |

액션 없다.

### 파라미터

공통 장애물 파라미터(위 표) + 아래 4개.

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robots` | string[] | `["amr_01"]` | 56 | 감시할 로봇 목록 |
| `config_dir` | string | (`_default_config_dir()`) | 57 | `robot_params.yaml` 이 있는 디렉터리 |
| `footprint_padding` | double | `0.0` | 58 | [m] 발자국에 더할 여유 |
| `contact_release` | double | `0.05` | 59 | [m] 접촉 해제 거리 (히스테리시스) |

---

## 3. `payload_manager_node`

- **소스**: `src/amr_simulation/amr_simulation/payload_manager_node.py` · Python
- **역할**: 주행 중 적재/하역 (명세 4.1 · 8장). `task_executor_node` 가 내는 `payload/attach` ·
  `payload/mass` (계약 C5) 를 받아, 물품 표의 크기·질량으로 화물 모델 `<robot>_cargo` 를 데크 위에 생성하고
  (Gazebo `UserCommands create`, `ign service` CLI) 로봇의 `DetachableJoint` 로 붙인다.
  질량·관성이 동역학에 합산된다. 하역은 detach + remove.
- **네임스페이스**: 로봇 네임스페이스 (`/amr_01`) — 아래 토픽은 **상대 이름**이다.

### 발행 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `cargo/attach` | `std_msgs/Empty` | 기본 (10) | `:139` — Gazebo `DetachableJoint` 부착 트리거 |
| `cargo/detach` | `std_msgs/Empty` | 기본 (10) | `:140` — 분리 트리거 |
| `payload/sim_state` | `std_msgs/String` (JSON) | **latched** | `:141`, `:150` (기동 시 `none` 상태 1회) |

### 구독 토픽

| 토픽 | 타입 | QoS | 근거 |
| --- | --- | --- | --- |
| `payload/attach` | `std_msgs/String` | **latched** | `:136` — `task_executor_node` 가 낸다 |
| `payload/mass` | `std_msgs/Float32` | **latched** | `:137` |
| `ground_truth/odom` | `nav_msgs/Odometry` | 기본 (10) | `:138` — 정지 판정용 |
| `cargo/state` | `std_msgs/String` | 기본 (10) | `:139` — `DetachableJoint` 상태 |

서비스·액션 없다 (Gazebo 조작은 `ign service` CLI 로 한다 — ROS 서비스가 아니다).

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robot_name` | string | 네임스페이스 또는 `amr_01` | 115 | Gazebo 모델 이름 |
| `world` | string | `warehouse` | 116 | Gazebo 월드 이름 |
| `config_dir` | string | (`_default_config_dir()`) | 117 | `robot_params.yaml` 위치 (물품 표) |
| `self_visibility_bit` | int | `-1` | 118 | 자기 차체 LiDAR 가시성 비트 (`-1` = 이름에서 유도) |
| `settle` | double | `0.1` | 119 | [s] 요청 안정화 시간 |
| `stationary_speed` | double | `0.05` | 120 | [m/s] 이 이하면 정지로 보고 적재 허용 |
| `deck_gap` | double | `0.002` | 148 | [m] 데크와 화물 사이 간격 |
| `joint_timeout` | double | `5.0` | 148 | [s] `DetachableJoint` 응답 대기 |
| `stationary_timeout` | double | `5.0` | 149 | [s] 정지 대기 상한 |

---

## 4. 확인 못 함

- **`ros_gz_bridge` 가 브리지하는 토픽 목록**을 항목별로 추출하지 않았다.
  [components.md §3.1](../architecture/components.md) 에 개요가 있다 (센서, `cmd_vel`, `joint_states`,
  `odom_gz`, `ground_truth/odom`, `/clock`, 지게차·셔틀 `/sim/<이름>/odom`).
  실제 목록은 `src/amr_simulation/` 의 브리지 설정 파일과 런치를 본다.
- `src/amr_simulation/launch/` 의 **런치 인자와 각 노드 기동 조건**을 확인하지 않았다.
- `_default_config_dir()` 이 돌려주는 실제 경로를 확인하지 않았다 (`config_dir` 기본값).
- `payload_manager_node` 가 Gazebo 에 거는 **`ign service` 호출 목록**(create / remove / DetachableJoint)은
  ROS 인터페이스가 아니라 이 문서 범위 밖이다.
- 월드 SDF(`src/amr_simulation/worlds/`)가 내는 Gazebo 토픽은 ROS 토픽이 아니므로 적지 않았다.
