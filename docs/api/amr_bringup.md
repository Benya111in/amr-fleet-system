# API — `amr_bringup` · `amr_description` · `amr_evaluation`

> 통합 런치 · URDF · 평가 계측. [← 색인](README.md)
> 이 세 패키지는 노드 수가 적어 한 문서에 묶었다.
> `amr_bringup` 런치: `src/amr_bringup/launch/system.launch.py` (로봇 1대),
> `src/amr_fleet/launch/multi_robot.launch.py` 계열 (다중 로봇).

## 0. 노드 목록

| 노드 | 패키지 | 언어 | 형태 | 소스 |
| --- | --- | --- | --- | --- |
| `lifecycle_watchdog` | amr_bringup | Python | own | `amr_bringup/lifecycle_watchdog.py` |
| `world_gate` | amr_bringup | Python | **tool** (ROS 노드 아님) | `amr_bringup/world_gate.py` |
| `pose_error_logger` | amr_evaluation | Python | own | `amr_evaluation/pose_error_logger.py` |
| `cte_logger` | amr_evaluation | Python | own | `amr_evaluation/cte_logger.py` |
| `response_time_logger` | amr_evaluation | Python | own | `amr_evaluation/response_time_logger.py` |
| `cpu_sampler` | amr_evaluation | Python | own | `amr_evaluation/cpu_sampler.py` |
| `analyze` | amr_evaluation | Python | **tool** | `amr_evaluation/analyze.py` |
| `gz_world.py` | amr_description | Python | **tool** (ROS 노드 아님) | `src/amr_description/scripts/gz_world.py` |

`amr_description` 은 그 밖에 외부 노드 `robot_state_publisher` / `joint_state_publisher` 를 설정만 해서 띄운다
([components.md §3.1](../architecture/components.md)).

---

## 1. `lifecycle_watchdog`

- **소스**: `src/amr_bringup/amr_bringup/lifecycle_watchdog.py` · Python
  (진입점 `src/amr_bringup/scripts/lifecycle_watchdog`)
- **역할**: `nodes` 파라미터에 적힌 **관리(lifecycle) 노드들을 `active` 로 올린다.**
  Nav2 `lifecycle_manager` 가 놓친 노드를 주기적으로 다시 밀어 준다
  (5대 동시 기동 시 스택 1~3개가 매번 죽던 문제의 대책).
- **주기**: `period` = 5 s 마다 `check_once()` (스핀은 `run()` 이 직접 돌린다, `:60-64` 주석).

### 토픽

발행·구독 토픽이 **없다.** 서비스 클라이언트만 쓴다.

### 서비스 (클라이언트 — 호출할 때마다 만들고 지운다, `:64-74`)

| 서비스 | 타입 | 근거 |
| --- | --- | --- |
| `<node>/get_state` | `lifecycle_msgs/srv/GetState` | `:77` |
| `<node>/change_state` | `lifecycle_msgs/srv/ChangeState` | `:66` 의 일반 호출 헬퍼를 통해 (전이 요청) |

`<node>` 는 `nodes` 파라미터의 각 이름이다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `nodes` | string[] | `[]` | 55 | active 로 올릴 관리 노드 이름 목록 (`_NODES_DESCRIPTOR` 사용) |
| `grace` | double | `60.0` | 57 | [s] 기동 유예 — 이 시간 안에는 강제하지 않는다 |
| `period` | double | `5.0` | 58 | [s] 점검 주기 |
| `service_timeout` | double | `3.0` | 59 | [s] 서비스 호출 대기 |

액션 없다.

---

## 2. `world_gate` (ROS 노드 아님)

- **소스**: `src/amr_bringup/amr_bringup/world_gate.py` · Python CLI
- **역할**: Gazebo 월드 로드 대기 → 일시정지 → 스폰 확인 → 재개. `system.launch.py` /
  `multi_robot.launch.py` 가 `ExecuteProcess` 로 두 번 부른다 (`launch_utils.spawn_sequence`, `:5`).
- **ROS 토픽·서비스·액션을 만들지 않는다.** `ign service` / `ign model` / `ign topic` CLI 만 호출한다 (`:47-55`).

```
python3 -m amr_bringup.world_gate hold    --world warehouse --pause
python3 -m amr_bringup.world_gate release --world warehouse --models amr_01,amr_02 --unpause
```

| 종료 코드 | 의미 | 근거 |
| --- | --- | --- |
| 0 | 정상 | `:22`, `:38` |
| 2 | 일부 모델 누락 (재개는 함) | `:22`, `:38` |
| 3 | 월드 없음 (시간 초과) | `:22`, `:38` |
| 4 | 일시정지/재개 요청 실패 | `:22`, `:38` |

`IGN_PARTITION` 을 Gazebo 서버와 같게 물려받아야 한다 ([multi_robot.md §5](../architecture/multi_robot.md)).
왜 멈춘 채로 스폰하는지(센서 따라잡기 문제)는 소스 docstring `:24-38` 에 실측과 함께 적혀 있다.

---

## 3. `pose_error_logger`

- **소스**: `src/amr_evaluation/amr_evaluation/pose_error_logger.py` · Python
- **역할**: 지면 진실과 추정 자세를 시각으로 맞춰 위치 오차를 CSV 로 기록한다
  (명세 4.3 정지 3 cm / 직선 5 cm / 회전 8 cm 판정).
- **네임스페이스**: `evaluation.launch.py` 의 `namespace` 인자 (기본 `amr_01`, `:60`).

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | 근거 |
| --- | --- | --- | --- |
| `ground_truth/odom` | `gt_topic` | `nav_msgs/Odometry` | `:25`, `:48` |
| `odometry/filtered_map` | `est_topic` | `nav_msgs/Odometry` | `:26`, `:49` |

발행 토픽·서비스·액션 **없다** (결과는 파일로 쓴다). 타이머 `flush_rate` = 20 Hz (`:49`).

### 파라미터

`EvalLoggerNode` 공통 (`logger_base.py:26-29`): `output_dir`(`''`), `run_name`(`''`),
`best_effort`(`False`), `report_period`(`5.0`).

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `gt_topic` | string | `ground_truth/odom` | 25 | 지면 진실 |
| `est_topic` | string | `odometry/filtered_map` | 26 | 추정 자세 |
| `stop_linear_threshold` | double | `0.02` | 27 | [m/s] 정지 구간 판정 |
| `stop_angular_threshold` | double | `0.02` | 28 | [rad/s] |
| `straight_angular_threshold` | double | `0.05` | 29 | [rad/s] 직선 구간 판정 |
| `gt_max_gap` | double | `0.2` | 30 | [s] 이보다 벌어진 GT 사이는 보간 안 함 |
| `gt_buffer_sec` | double | `5.0` | 31 | [s] GT 보관 길이 |
| `max_wait` | double | `1.0` | 32 | [s] GT 를 기다리는 상한 |
| `flush_rate` | double | `20.0` | 33 | [Hz] 대기 추정 처리 주기 |
| `target_samples` | int | `100` | 34 | 명세: 최소 100 시점 |

---

## 4. `cte_logger`

- **소스**: `src/amr_evaluation/amr_evaluation/cte_logger.py` · Python
- **역할**: 계획 경로 대비 횡방향 오차(CTE)를 기록한다. 직선·곡선 구간을 곡률로 갈라 따로 집계한다.

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | 근거 |
| --- | --- | --- | --- |
| `plan` | `path_topic` | `nav_msgs/Path` | `:39`, `:68` |
| `ground_truth/odom` | `gt_topic` | `nav_msgs/Odometry` | `:40`, `:69` |
| `navigate_to_pose/_action/status` | `nav_status_topic` | `action_msgs/GoalStatusArray` | `:41`, `:78` — `''` 이면 끔 |
| `executor/phase` | `phase_topic` | `std_msgs/String` | `:42`, `:82` — `''` 이면 끔 |

발행 토픽·서비스·액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `path_topic` | string | `plan` | 39 | 계획 경로 |
| `gt_topic` | string | `ground_truth/odom` | 40 | 지면 진실 |
| `nav_status_topic` | string | `navigate_to_pose/_action/status` | 41 | 활성 구간 판정 (`''` = 끔) |
| `phase_topic` | string | `executor/phase` | 42 | 활성 구간 판정 (`''` = 끔) |
| `active_phases` | string[] | `['moving']` | 43 | 이 단계에서만 기록 |
| `min_speed` | double | `0.02` | 44 | [m/s] 정지 판정 |
| `min_angular` | double | `0.02` | 45 | [rad/s] 정지 판정 |
| `plan_timeout` | double | `3.0` | 46 | [s] 경로 폐기 시간 (0 = 끔) |
| `skip_endpoint` | bool | `True` | 47 | 끝점 너머로 잘린 사영 제외 |
| `curvature_threshold` | double | `0.1` | 48 | [1/m] 초과 시 곡선 (R < 10 m) |
| `curvature_window` | double | (`segments.DEFAULT_CURVATURE_WINDOW`) | 49 | [m] 곡률 계산 창 |
| `curvature_smoothing` | double | (`segments.DEFAULT_SMOOTHING`) | 50 | [m] σ |
| `min_curve_turn_deg` | double | (`degrees(segments.DEFAULT_MIN_TURN)`) | 51 | [deg] 곡선 최소 회전각 |
| `curve_margin` | double | `0.5` | 52 | [m] 곡선 앞뒤 천이대도 곡선으로 |
| `log_rate` | double | `20.0` | 53 | [Hz] GT 데시메이션 |
| `min_path_points` | int | `2` | 54 | 유효 경로 최소 점 수 |

(+ `EvalLoggerNode` 공통 파라미터 4개)

---

## 5. `response_time_logger`

- **소스**: `src/amr_evaluation/amr_evaluation/response_time_logger.py` · Python
- **역할**: 명령(작업 이벤트 또는 목표 자세) → 실제 움직임 시작까지의 지연을 잰다
  (명세 9장 응답 시간 50 회 이상 표본).

### 구독 토픽

| 토픽 (기본값) | 파라미터 | 타입 | 근거 | 조건 |
| --- | --- | --- | --- | --- |
| `/fleet/task_events` | `cmd_topic` | `amr_msgs/Task` | `:81`, `:124` | `cmd_type:=task` (기본) |
| (같은 `cmd_topic`) | `cmd_topic` | `geometry_msgs/PoseStamped` | `:127` | `cmd_type:=pose` |
| `ground_truth/odom` | `motion_topic` | `nav_msgs/Odometry` | `:83`, `:130` | `motion_type:=odom` (기본) |
| (같은 `motion_topic`) | `motion_topic` | `geometry_msgs/Twist` | `:133` | `motion_type:=twist` |

발행 토픽·서비스·액션 없다.

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `cmd_topic` | string | `/fleet/task_events` | 81 | 명령 토픽 (전역) |
| `cmd_type` | string | `task` | 81 | `task` \| `pose` |
| `cmd_stamp` | string | `auto` | 82 | `auto` \| `pickup` \| `dropoff` \| `header` |
| `motion_topic` | string | `ground_truth/odom` | 83 | 움직임 판정 입력 |
| `motion_type` | string | `odom` | 84 | `odom` \| `twist` |
| `motion_threshold` | double | `0.05` | 85 | [m/s] 움직임 시작 판정 |
| `angular_threshold` | double | `0.1` | 86 | [rad/s] |
| `robot_id` | string | `''` | 87 | `task` 모드: 이 로봇의 작업만 (런치 기본 = `namespace`, `:62-63`) |
| `require_rest` | bool | `True` | 88 | 이미 움직이는 중의 명령은 제외 |
| `timeout` | double | `10.0` | 89 | [s] 무응답 판정 |
| `max_clock_skew` | double | `3600.0` | 90 | [s] 이보다 떨어지면 시계 불일치 |
| `rtf_window` | double | `5.0` | 91 | [s] RTF 추정 창 (벽시계) |
| `target_samples` | int | `50` | 92 | 명세: 50 회 이상 |

(+ `EvalLoggerNode` 공통 파라미터 4개)

---

## 6. `cpu_sampler`

- **소스**: `src/amr_evaluation/amr_evaluation/cpu_sampler.py` · Python
- **역할**: `/proc/stat` 과 프로세스 그룹·cgroup 을 샘플링해 CPU 사용률을 기록한다
  (명세 4.10 "5대 운용 시 CPU 80 % 이하").
- **주기**: `sample_rate` = 1 Hz, **STEADY_TIME 시계** 사용 (`:72-73`) — sim time 과 무관하게 벽시계로 샘플링한다.

### 토픽·서비스·액션

**없다.** ROS 토픽을 전혀 쓰지 않는다 (`/proc` 을 직접 읽고 파일로 쓴다).
`EvalLoggerNode` 를 상속해 같은 실행 디렉터리 규약을 쓰는 ROS 노드일 뿐이다 (`:47`).

### 파라미터

| 이름 | 타입 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- | --- |
| `sample_rate` | double | `1.0` | 52 | [Hz] |
| `proc_stat_path` | string | `/proc/stat` | 52 | 전체 CPU 통계 |
| `proc_root` | string | `/proc` | 53 | 프로세스 그룹 계산 (`''` = 끔) |
| `process_groups` | string[] | (`acct.DEFAULT_GROUPS`) | 54 | 집계할 프로세스 그룹 |
| `cgroup_root` | string | `/sys/fs/cgroup` | 55 | `''` 이면 cgroup 열 없음 |
| `num_cpus` | int | `0` | 56 | 0 → 호스트 온라인 CPU 수 |

(+ `EvalLoggerNode` 공통 파라미터 4개)

### 런치 (`evaluation.launch.py`)

| 인자 | 기본값 | 행 | 설명 |
| --- | --- | --- | --- |
| `run_name` | `_DEFAULT_RUN_NAME` | 55-56 | `<output_dir>/<run_name>/` |
| `output_dir` | `_DEFAULT_OUTPUT_DIR` (= `$ROS_WS/logs/eval`) | 57-58 | 로그 루트 |
| `namespace` | `amr_01` | 60-61 | 로거들이 뜨는 네임스페이스 |
| `robot_id` | `namespace` | 62-64 | 응답 시간: 이 로봇의 작업만 |
| `use_sim_time` | — | 46 | |
| `params_file` | 패키지 `config/amr_evaluation.yaml` | 37-38 | |

---

## 7. `gz_world.py` (ROS 노드 아님)

- **소스**: `src/amr_description/scripts/gz_world.py` · Python CLI
- **역할**: Gazebo 모델 스폰(`create` 서비스 대기 → 중복 검사 → 생성 → `scene`/`info` 로 생성 확인,
  실패 시 런치 종료)과 일시정지 해제(전 로봇 확인 후).
- **ROS 인터페이스**: 확인 절차 중 **임시 노드로 토픽 하나를 구독한다** (`:129`) —
  토픽 이름과 타입(`std_msgs/String`)이 인자로 들어오므로 고정된 이름이 없다.
  그 밖에는 `ign service` CLI 로 동작한다.

---

## 8. `analyze` (ROS 노드 아님)

- **소스**: `src/amr_evaluation/amr_evaluation/analyze.py` (진입점 `scripts/analyze`) · Python CLI
- **역할**: `logs/eval/<run>/` 의 CSV 를 읽어 명세 판정을 내린다. **종료 코드 = 판정 결과**
  (`scripts/analyze:2`).

```
ros2 run amr_evaluation analyze --input logs/eval/<run>/
```

---

## 9. 확인 못 함

- **`system.launch.py` / `multi_robot.launch.py` 의 전체 구성**(어떤 하위 런치를 어떤 인자로 include 하는지,
  스택 stagger 6 s, `lifecycle_watchdog` 에 넘기는 `nodes` 목록)을 이 문서에 정리하지 않았다.
  이 문서는 노드 인터페이스만 다룬다.
- `amr_description` 의 `robot_state_publisher` / `joint_state_publisher` 설정
  (`robot_description` 생성, xacro 인자 `prefix`)은 [multi_robot.md §2](../architecture/multi_robot.md) 가 다룬다.
- `EvalLoggerNode` (`logger_base.py`) 의 `report_period` 타이머가 무엇을 내는지 확인하지 않았다 —
  토픽 발행은 없고 로그/파일 출력인 것으로 보이나 코드로 확정하지 않았다.
- `cte_logger` 의 `curvature_window`, `curvature_smoothing`, `min_curve_turn_deg` **기본값 수치**는
  `segments.DEFAULT_*` 상수라 값을 확인하지 않았다 (`amr_evaluation/segments.py`).
- `cpu_sampler` 의 `process_groups` 기본값(`acct.DEFAULT_GROUPS`) 내용을 확인하지 않았다.
- `evaluation.launch.py` 의 `_DEFAULT_RUN_NAME` 실제 값을 확인하지 않았다.
- `gz_world.py` 가 구독하는 토픽의 **호출자별 실제 이름**을 확인하지 않았다 (인자로 주입된다).
