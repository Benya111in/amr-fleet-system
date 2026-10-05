# API — `amr_dashboard`

> 명세 9장 (웹 모니터링 대시보드). [← 색인](README.md)
> 런치: `src/amr_dashboard/launch/` — 노드는 **루트 네임스페이스**에 뜬다
> ([multi_robot.md §1](../architecture/multi_robot.md): "로봇 수와 무관한 공용 인프라").
> 그래서 이 문서의 토픽 이름은 대부분 **절대 이름**이다.

## `dashboard_node`

- **소스**: `src/amr_dashboard/amr_dashboard/dashboard_node.py` · Python
  (진입점 `src/amr_dashboard/scripts/dashboard_node`)
- **역할**: ROS 상태를 모아 HTTP/웹 UI 로 보여 주고, 웹에서 들어온 작업 요청과 E-stop 조작을 ROS 로 내보낸다.
  웹 자원은 `src/amr_dashboard/web/`.
- **개수**: 시스템 전체에 1개.

### 발행 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `/fleet/task_request` | `topics.task_request` | `std_msgs/String` (JSON) | RELIABLE, depth 10 | `:99`, `:127` |
| `/fleet/estop` | `topics.fleet_estop` | `std_msgs/Bool` | **latched** (depth 1, TRANSIENT_LOCAL) | `:100`, `:128-129` |
| `/<robot_id>/estop` | `topics.robot_estop` | `std_msgs/Bool` | **latched** | `:101`, `:271-274` — `robot_ids` 의 로봇마다 1개 (동적 생성) |

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | QoS | 근거 |
| --- | --- | --- | --- | --- |
| `/fleet/status` | `topics.fleet_status` | `amr_msgs/FleetStatus` | RELIABLE, depth 10 | `:95`, `:116-117` |
| `/fleet/alerts` | `topics.fleet_alerts` | `diagnostic_msgs/DiagnosticArray` | RELIABLE, depth 10 | `:96`, `:118-119` |
| `/fleet/task_events` | `topics.task_events` | `amr_msgs/Task` | RELIABLE, depth 50 (+ `task_events_transient_local` 이면 TRANSIENT_LOCAL) | `:97`, `:120-122` |
| `/map` | `topics.map` | `nav_msgs/OccupancyGrid` | RELIABLE depth 1 + TRANSIENT_LOCAL (**latched**) | `:98`, `:123-124` |

### 서비스 (클라이언트)

| 서비스 (기본값) | 파라미터 | 타입 | 근거 |
| --- | --- | --- | --- |
| `/<robot_id>/safety/reset_estop` | `topics.reset_estop_service` | `std_srvs/srv/Trigger` | `:102`, `:278-287` — 로봇마다 1개 (동적 생성). `''` 이면 만들지 않는다 (`:280-281`) |

액션 없다.

### 로봇별 토픽 이름 조립

`robot_topic(robot_id, name)` 이 `/<robot_id>/<name>` 을 만든다 (`:272-273`, `:285`).
`robot_ids` 중 토픽 이름으로 쓸 수 없는 값은 경고 후 제외한다 (`:105-108`).

### 파라미터

HTTP 서버

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `host` | string | `127.0.0.1` | 79 | 바인드 주소 |
| `port` | int | `8080` | 80 | HTTP 포트 |
| `web_dir` | string | `''` | 81 | 정적 웹 자원 디렉터리 (`''` = 패키지 `web/`) |
| `log_dir` | string | `''` | 82 | 로그 디렉터리 |
| `heartbeat_period` | double | `5.0` | 83 | [s] 웹 하트비트 주기 |
| `executor_threads` | int | `1` | 84 | ROS 실행기 스레드 수 |
| `allowed_hosts` | string[] | `['']` | 87 | Host 헤더 허용 추가분 (`'*'` = 검사 끔) |
| `api_token` | string | `''` | 88 | `''` → 환경변수 `$AMR_DASHBOARD_TOKEN` → 없음 |

E-stop 조작

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `require_reset_ack` | bool | `True` | 89 | 리셋 서버가 없으면 해제를 실패로 처리 |
| `reset_timeout` | double | `2.0` | 90 | [s] `reset_estop` 응답 대기 |
| `reset_ack_timeout` | double | `0.2` | 91 | [s] `false` 발행 ack 대기 (리셋 전) |

상태 저장소

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `robot_ids` | string[] (dynamic) | `['amr_01','amr_02','amr_03','amr_04','amr_05']` | 85-86, `:51` | 표시할 로봇 목록 |
| `task_events_transient_local` | bool | `False` | 92 | `task_events` 구독을 latched 로 |
| `max_alerts` | int | `100` | 93 | 보관할 경보 수 |
| `max_task_events` | int | `50` | 94 | 보관할 작업 이벤트 수 |

토픽·서비스 이름

| 이름 | 타입 | 기본값 | 행 |
| --- | --- | --- | --- |
| `topics.fleet_status` | string | `/fleet/status` | 95 |
| `topics.fleet_alerts` | string | `/fleet/alerts` | 96 |
| `topics.task_events` | string | `/fleet/task_events` | 97 |
| `topics.map` | string | `/map` | 98 |
| `topics.task_request` | string | `/fleet/task_request` | 99 |
| `topics.fleet_estop` | string | `/fleet/estop` | 100 |
| `topics.robot_estop` | string | `estop` | 101 |
| `topics.reset_estop_service` | string | `safety/reset_estop` | 102 |

> `topics.robot_estop` 과 `topics.reset_estop_service` 만 **상대 이름**이다 —
> `robot_topic()` 이 앞에 `/<robot_id>` 를 붙인다.

월드 (지도 배경 그리기용) — `DEFAULT_WORLD` 의 키마다 `world.<key>` 를 선언한다 (`:102-104`).
기본값 출처 `src/amr_dashboard/amr_dashboard/state_store.py:53`.

| 이름 | 타입 | 기본값 | 설명 |
| --- | --- | --- | --- |
| `world.origin_x` | double | `0.0` | [m] |
| `world.origin_y` | double | `0.0` | [m] |
| `world.width` | double | `60.0` | [m] 창고 월드 가로 |
| `world.height` | double | `40.0` | [m] 창고 월드 세로 |

---

## 확인 못 함

- **HTTP API 엔드포인트**(경로·메서드·요청/응답 형식)는 이 문서 범위 밖이다. 이 문서는 명세 4.10 이 요구한
  "노드의 토픽·서비스·액션 인터페이스" 만 다룬다. 웹 API 는 `src/amr_dashboard/amr_dashboard/`
  의 HTTP 핸들러와 `src/amr_dashboard/web/` 을 본다.
- 런치 파일(`src/amr_dashboard/launch/`)의 **인자 이름·기본값**을 확인하지 않았다.
  런치가 `executable='dashboard_node'` 로 띄운다는 것만 확인했다 (`:48`).
- `qos_reliable(depth, transient_local)` 헬퍼의 정의 위치를 확인하지 않았다 — 인자 의미로만 QoS 를 적었다.
