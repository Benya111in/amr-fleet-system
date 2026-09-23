# 성능 측정 및 분석 리포트

> 명세 10장 요구 산출물. 측정 절차는 명세의 "성능 지표 측정 절차 표준화" 표를 따르며,
> 측정 도구는 `src/amr_evaluation` 패키지로 표준화한다.
> 측정 결과는 [performance.md](performance.md) — 아래 표의 '상태' 열은 그 요약이다.

## 측정 대상 지표

| 지표 | 목표 | 산출 방법 | 상태 |
| --- | --- | --- | --- |
| 위치 추정 RMSE | 정지 3cm / 직선 5cm / 회전 8cm | GT 대비 제곱평균제곱근, 100개 시점 이상 | **1.47 / 1.32 / 1.44 cm** (04) |
| 경로 추종 CTE | 직선 5cm / 곡선 10cm | 계획 경로와 실제 궤적의 수직 거리 | **2.87 / 3.12 cm** (07) |
| 응답 시간 | 200ms 이내 | 명령 발행 ~ 첫 움직임, 50회 이상 | **144.3 ms** (n=50, 13) |
| 경로 계획 성공률 | 98% 이상 (50쌍) | 성공/전체 | **100 %** (06) |
| 경로 재계획 시간 | 500ms 이내 | 재계획 트리거 ~ 완료 | **최대 13.0 ms** (06) |
| 동적 회피 충돌 | 30회 중 0건 | 충돌 이벤트 카운트 | **0건 / 30회** (08) |
| 도킹 정밀도 | 위치 2cm / 각도 1도 | GT 대비 최종 오차 | **10/10** (10) |
| 작업 성공률 | 97% 이상 | 완료/전체 | 미측정 |
| CPU 사용률 | 5대 운용 시 80% 이하 | 지속 샘플링 | **56.1 %** (n=398, 12) |
| 테스트 커버리지 | 70% 이상 | `colcon test` / `pytest --cov` | **95 %** (Python 문장) |
| 연속 운용 | 4시간 안정성 | 장시간 테스트 | 미측정 |

## 측정 도구: `amr_evaluation`

표의 앞 세 지표와 CPU 사용률은 `amr_evaluation` 의 로거 노드가 CSV 로 남기고 `analyze` 가 판정한다.
나머지 지표(계획 성공률, 재계획 시간, 충돌, 도킹, 작업 성공률)는 각 패키지의 통합 테스트/작업 로그에서 집계한다.

| 지표 | 노드 (`ros2 run amr_evaluation …`) | 입력 토픽 (로봇 네임스페이스 기준) | 출력 CSV |
| --- | --- | --- | --- |
| 위치 추정 RMSE | `pose_error_logger` | `ground_truth/odom` (GT, `nav_msgs/Odometry`), `odometry/filtered_map` (EKF) | `pose_error.csv` |
| 경로 추종 CTE | `cte_logger` | `plan` (`nav_msgs/Path`), `ground_truth/odom` | `cte.csv` |
| 경로 추종 CTE (주행 상태) | `cte_logger` | `navigate_to_pose/_action/status` (`action_msgs/GoalStatusArray`), `executor/phase` (`std_msgs/String`) — 있으면 주행 중 표본만 | (같은 `cte.csv`) |
| 응답 시간 | `response_time_logger` | `/fleet/task_events` (`amr_msgs/Task`, IN_PROGRESS 로 전이한 작업의 명령 시각), `ground_truth/odom` (첫 움직임) | `response_time.csv` |
| CPU 사용률 | `cpu_sampler` | `/proc/stat` (호스트 전체), `/proc/<pid>/stat` (우리 ROS 프로세스 그룹), cgroup — 1 Hz (이미지에 mpstat 없음) | `cpu.csv` |

파라미터 기본값은 `src/amr_evaluation/config/amr_evaluation.yaml` (토픽, 구간 임계값, 움직임 판정 속도 등).

### 절차

1. 시스템을 기동한다 (`docker compose --profile run up -d` 또는 개별 launch). GT 토픽 `ground_truth/odom` 은
   시뮬레이션 브리지가 로봇 네임스페이스 아래에 발행한다.
2. 로거를 한 런 이름으로 묶어 기동한다 (컨테이너 셸에서):

   ```bash
   ros2 launch amr_evaluation evaluation.launch.py run_name:=ekf_s2_straight namespace:=amr_01
   ```

   네 로거가 모두 `logs/eval/<run_name>/` (기본 `$ROS_WS/logs/eval`, `output_dir:=` 로 변경) 에 쓴다.
   파일 이름에 네임스페이스가 붙는다 (`pose_error_amr_01.csv`) — 여러 로봇을 같은 `run_name` 으로 재려면
   로봇마다 `namespace:=amr_0X` 로 띄우고, CPU 는 호스트 값이라 두 번째부터 `with_cpu:=false` 를 준다.
   같은 이름의 파일이 이미 있으면 덮어쓰지 않고 `_1`, `_2` 를 붙인다. `analyze` 는 이 파일들을 합쳐 판정한다.
   `run_name:=20260922` 처럼 숫자만 줘도 된다 (문자열 파라미터로 넘긴다).
   `use_sim_time` 기본값은 `true` — `fleet_manager.launch.py`·`dashboard.launch.py` 와 같다. 타임스탬프는
   시뮬레이션 시각(메시지 헤더)이다. 한쪽만 `use_sim_time:=false` 이면 시뮬 시간(0 부터)과 벽시계 에포크
   (~1.8e9 s)가 섞이는데, 로거는 스탬프 크기로 시계 영역을 판별해 스트림마다 처음 한 번 경고하고
   (`시계 불일치` 카운트) 응답 시간은 짝짓지 않는다. 응답 시간은 `robot_id` (기본 = `namespace`,
   앞뒤 `/` 는 뗀다) 로봇의 작업 이벤트만 센다.
3. 시나리오를 수행한다 (정지 / 직선 / 회전 구간이 각각 100 샘플 이상 들어가도록: 50 Hz 에서 2 s 이상씩,
   CTE 는 직선 / 곡선 구간이 각각 100 샘플 이상 — 20 Hz 에서 5 s 이상씩, CPU 는 60 s 이상).
   응답 시간은 명령 시각에 정지해 있던 작업만 센다 (`require_rest`) — 50 회 이상 작업을 투입한다.
4. `Ctrl-C` 로 로거를 끝낸다. 각 행은 즉시 flush 되므로 도중에 죽어도 그때까지의 로그는 남는다.
5. 분석·판정:

   ```bash
   ros2 run amr_evaluation analyze --input logs/eval/ekf_s2_straight
   # 또는: python3 -m amr_evaluation.analyze --input logs/eval/ekf_s2_straight
   ```

   런 디렉토리에 `report.md` (Markdown 표) 와 `pose_error.png` `trajectory.png` `cte.png` `response_time.png`
   `cpu.png` 를 쓰고 표를 stdout 에 찍는다. `--input` 을 생략하면 가장 최근 런을 고른다.
6. 리포트에는 `report.md` 의 표를 그대로 옮기고 PNG 를 첨부한다.

### 판정과 종료 코드 (CI 게이트)

| 지표 | 판정 통계 | 기준 | 옵션 |
| --- | --- | --- | --- |
| 위치 추정 오차 | 구간별 RMSE (`--gate-metric max\|p95\|mean` 로 변경) | 정지 3 / 직선 5 / 회전 8 cm | `--pose-thresholds 0.03,0.05,0.08` |
| CTE | 구간별 평균 \|CTE\| | 직선 5 / 곡선 10 cm | `--cte-thresholds 0.05,0.10` |
| 응답 시간 | 평균 (벽시계 환산) | 200 ms | `--response-ms 200`, `--response-clock wall\|sim` |
| CPU | 평균 (호스트 전체) | 80 % | `--cpu-percent 80`, `--cpu-column cpu_total_percent` |

- 종료 코드: `0` 통과, `1` 기준 위반 (`--no-fail` 이면 `0`), `2` 런 디렉토리 없음·인자 오류,
  `3` 데이터 부족, `4` 분석 중 예상하지 못한 오류 (트레이스백 출력 — 기준 위반과 구분된다).
- 데이터 부족(`3`): 필수 지표(`--require`, 기본 `pose,cte,response,cpu` 전부)의 CSV 가 없거나 헤더뿐이거나
  필수 열이 없을 때, 또는 판정 구간의 표본이 최소 개수 미만일 때 — 위치 오차 구간(정지/직선/회전)마다
  100, CTE 구간(직선/곡선)마다 100, 응답 50, CPU 60 (`--min-pose-samples` 등으로 변경). 로거가 아무것도
  못 남긴 런(토픽 이름·QoS·시계·robot_id 불일치)이 통과로 보이지 않게 한다. 한 지표만 재는 실험은
  `--require response` 처럼 필요한 지표만 요구하고, 부족을 알고도 표를 보려면 `--allow-missing`
  (부족은 표 아래에 적히고 판정에서 빠진다). 기준 위반이 있으면 `1` 이 `3` 보다 앞선다.
- 잘린 마지막 줄(기록 중 강제 종료)·숫자가 아닌 칸은 건너뛰고 경고한다. `--strict` 면 경고도 실패(`1`).
- 응답 시간은 기본으로 벽시계 환산 지연(`latency_wall_ms` = `latency_ms` / RTF)의 평균으로 판정한다 —
  설계([multi_robot.md](../architecture/multi_robot.md) §7)가 응답 KPI 를 실시간 기준으로 두기 때문이다.
  스탬프(시뮬 시간) 기준 평균은 참고 행이고 `--response-clock sim` 이면 그것으로 판정한다. RTF 평균이
  0.95 미만이면 경고한다. 옛 로그처럼 RTF 가 없으면 스탬프 기준으로 판정하고 그 사실을 경고한다.
- CPU 는 기본으로 호스트 전체(`cpu_total_percent`)로 판정한다 (보수적: 시스템 몫 ≤ 호스트 전체).
  공유 서버에서 다른 사용자 부하로 실패하면 우리 ROS 프로세스 귀속 값(`cpu_ros_percent`)을 경고에 함께
  적는다. 시스템 전체 프로세스가 보이는 곳(같은 컨테이너 또는 `pid: host`)에서 쟀다면
  `--cpu-column cpu_ros_percent` (또는 `cpu_cgroup_percent`) 로 판정한다. 나머지 사용률 열·`load1` 은 참고 행.
- 파일 여러 개(`<지표>_<ns>.csv`)는 합쳐서 판정한다. 파일마다 시각 열의 시계 영역이 다르면 경고한다.
- max / p95 / mean 은 참고 열에 함께 적힌다. `--align` 을 주면 SE(2) 최소제곱 정렬 후 오차도 참고 행으로
  더한다 (map↔world 프레임의 상수 오프셋이 의심될 때; 판정은 항상 원시 오차로 한다).
- 통합 테스트 예: `ros2 run amr_evaluation analyze --input logs/eval/<run> || exit 1`

## 로그 포맷

명세 표의 열을 앞에 그대로 두고, 분석에 필요한 열을 뒤에 덧붙인다 (단위: m, rad, s, ms, %).

```
pose_error.csv    : [timestamp, gt_x, gt_y, est_x, est_y, error] + yaw_error, segment(정지|직선|회전)
cte.csv           : [timestamp, planned_x, planned_y, actual_x, actual_y, cte] + segment(직선|곡선)
response_time.csv : [cmd_time, response_time, latency_ms] + cmd_id, cmd_source, rtf, latency_wall_ms
cpu.csv           : [timestamp, cpu_total_percent] + cpu_<그룹>_percent…, procs_<그룹>…, cpu_cgroup_percent,
                    rtf, load1, cpu0, cpu1, ...        (그룹 기본: ros, gazebo, nav)
```

- `pose_error`: GT 를 추정 스탬프 시각에 선형 보간한다 (최근접 GT 샘플로 외삽하지 않는다 — 2 m/s 에서
  10 ms 어긋나면 2 cm 의 가짜 오차가 생긴다). GT 간격이 `gt_max_gap`(0.2 s) 을 넘는 구간은 버린다.
  구간은 GT 속도로 나눈다: 정지 |v| < 0.02 m/s 이고 |ω| < 0.02 rad/s, 회전 |ω| ≥ 0.05 rad/s 이거나
  |v| < 0.02 m/s 인 제자리 회전(|ω| ≥ 0.02), 직선 그 외 (|v| ≥ 0.02 m/s 이고 |ω| < 0.05 rad/s).
- `cte`: GT 위치에서 최신 `plan` 폴리라인의 최근접 선분까지 수직 거리, 경로 진행 방향 좌측이 +.
  `planned_x/y` 는 경로 위 최근접점. GT(world) 와 경로(map) 프레임이 일치한다고 가정한다.
  **로봇이 그 경로를 따라가는 중인 표본만 쓴다** (`cte_gate.py`). 아래를 순서대로 검사해 처음 걸린 사유로
  버리고 사유별 개수를 진행·종료 로그에 남긴다:

  | 사유 | 조건 |
  | --- | --- |
  | `no_plan` | 경로를 아직 받지 못함 (정점 < `min_path_points`) |
  | `nav_inactive` | `navigate_to_pose/_action/status` 를 받았고 최신 목표(수락 시각이 가장 늦은 것)가 ACCEPTED/EXECUTING 이 아님 — 도착 후 도킹(마커 서보, 경로 아님)·적재 대기·다음 할당 대기 |
  | `phase` | `executor/phase` 를 받았고 값이 `active_phases`(기본 `moving`) 밖 — docking / loading / charging / idle / error |
  | `stale_plan` | 경로가 최신 목표보다 오래됨: plan 스탬프 < 최신 목표 수락 스탬프, 또는 plan 수신 + 1 s < phase 가 moving 으로 바뀐 시각, 또는 plan 을 받은 지 `plan_timeout`(기본 3 s, `0` 끔) 이 지남 — `bt_navigator` 는 주행 중 1 Hz 로 재계획하므로 목표가 끝나면 경로가 더 오지 않는다 |
  | `stopped` | GT \|v\| < 0.02 m/s 이고 \|ω\| < 0.02 rad/s (정지·주차) |
  | `endpoint` | 최근접점이 경로 시작점 앞/끝점 너머로 잘림 — 그 거리는 종방향이라 CTE 가 아니다 |

  주행 상태 원천(항법 상태·phase)을 한 번도 받지 못하면 그 두 규칙은 건너뛰고 `stale_plan`(`plan_timeout`)·
  `stopped`·`endpoint` 만 적용한다. GT 시각이 1 s 넘게 거꾸로 가면(시뮬 리셋) 경로·목표 기록을 비우고
  다시 시작한다 (데시메이션도 새 시간축에서 이어진다).
  직선/곡선: 경로를 5 cm 등간격으로 다시 표본하고, 양 끝을 끝 0.5 m 의 최소제곱 직선 방향으로 늘인 뒤
  가우시안(σ 0.2 m)으로 평활하고, 앞뒤 0.5 m 현(chord)의 방향 차 / 현 길이로 곡률을 구한다.
  |κ| > 0.1 1/m (반경 10 m 미만) 인 같은 부호 연속 구간 가운데 누적 회전각이 10° 이상인 것과 그 앞뒤
  0.5 m 를 곡선으로 본다. 5 cm 격자 계단 경로(축·45° 가 아닌 직선)와 mm 지터는 직선으로 남고, 반경
  0.3~8 m 의 호는 곡선이 된다 (`test_segments.py`). 창(1 m)보다 짧은 경로는 곡률을 정할 수 없어 직선으로
  본다 (직선 기준 5 cm 가 더 엄격하므로 보수적).
- `response_time`: 명세 표의 "작업 명령 발행 → 로봇 첫 움직임". `response_time` 은 \|v\| ≥ 0.05 m/s 또는
  \|ω\| ≥ 0.1 rad/s 인 첫 GT 샘플의 스탬프, `cmd_time` 은 IN_PROGRESS 로 전이한 작업 이벤트에서
  `cmd_stamp` 규칙으로 고른다 (`cmd_source` 열에 원천):

  | `cmd_stamp` | `cmd_time` | `cmd_source` | 구간 |
  | --- | --- | --- | --- |
  | `auto` (기본) | `pickup_pose.header.stamp` 가 0 이 아니고 `header.stamp` 보다 이르면 그것, 아니면 `header.stamp` | `dispatch` / `event` | 명령 발행(fleet 가 `assign_task` 를 송신 지연 큐에 넣은 시각) → 첫 움직임 |
  | `pickup` | `pickup_pose.header.stamp` (0 이면 `header.stamp`) | `dispatch` | 위와 같음 |
  | `dropoff` | `dropoff_pose.header.stamp` = 접수 시각 | `request` | 작업 접수 → 첫 움직임 (배치 창·큐 대기 포함) |
  | `header` | `header.stamp` = IN_PROGRESS 전이 시각 | `event` | 로봇 수락 → 첫 움직임 (예전 정의) |

  `amr_fleet` 는 IN_PROGRESS 이벤트의 `pickup_pose.header.stamp` 에 명령 시각, `dropoff_pose.header.stamp` 에
  접수 시각을 싣는다. 명령 시각이 없는(0 이거나 전이 시각과 같은) 옛 fleet 이벤트에서는 전이 시각으로 대체하고
  한 번 경고한다 — 전이 시각은 로봇 수락 뒤라 통신 지연(0~100 ms)과 서비스 왕복이 빠져 지연을 적게 잰다.
  이벤트가 로봇 출발보다 늦게 도착해도 최근 운동 이력(`timeout` 동안)으로 명령 시각의 정지 여부와 그 뒤 첫
  움직임을 소급해 짝짓는다. 10 s 안에 움직임이 없으면 무응답으로 센다. 명령과 움직임 스탬프의 시계 영역이
  다르거나 1 시간 넘게 떨어지면 `시계 불일치` 로 세고 짝짓지 않는다.
  `rtf` 는 로거의 ROS 시각 대 단조 벽시계 비(최근 5 s), `latency_wall_ms` = `latency_ms` / `rtf`.
  명령·움직임 원천은 노드 파라미터로 바꾼다 (`params_file:=` 로 YAML 을 넘기거나 로거를 따로
  `ros2 run amr_evaluation response_time_logger --ros-args -r __ns:=/amr_01 -p …` 로 띄운다).
  목표 자세를 직접 발행하는 실험은 `cmd_type:=pose cmd_topic:=goal_pose`. 설계 문서
  ([sequences.md](../architecture/sequences.md) §1) 의 "`assign_task` 요청 → 첫 `cmd_vel` ≠ 0" 으로 재려면
  `motion_type:=twist motion_topic:=cmd_vel motion_threshold:=0.001` (응답 시각 = 수신 시각). 기본값
  (GT 첫 움직임) 은 가속 지연까지 포함하므로 더 엄격하다.
- `cpu`: `cpu_total_percent` 는 `/proc/stat` 두 스냅샷의 jiffies 차분이다. `/proc/stat` 은 네임스페이스가 없어
  컨테이너 안에서도 호스트 전체라서, 서버를 여러 사람이 쓰면 다른 사용자 작업이 섞인다 — KPI 의 참고값이고
  `load1` 을 옆에 남긴다. 시스템 귀속 값: `cpu_<그룹>_percent` 는 이 PID 네임스페이스에서 보이는 프로세스 중
  cmdline 이 그룹 정규식에 맞는 것의 utime+stime 합 (`ros` = `--ros-args`·`/opt/ros`·`/ros2_ws`·`ros2`·Gazebo,
  `gazebo`, `nav` = Nav2 서버·EKF·SLAM → 설계의 `cpu_gazebo_pct`·`cpu_nav_pct`), `procs_<그룹>` 은 그 개수,
  `cpu_cgroup_percent` 는 이 컨테이너 cgroup 전체 (v2 `cpu.stat`, v1 `cpuacct.usage`). 모두 호스트 CPU 수 ×
  경과 시간 대비 [%] 라 호스트 전체와 같은 척도다. 다른 컨테이너의 프로세스는 보이지 않으므로 시스템 전체를
  귀속하려면 시스템과 같은 컨테이너(또는 `pid: host`)에서 띄운다. 샘플러 타이머는 단조 시계라 시뮬 시간이 멈춰도
  돈다. `rtf` = Δ(ROS 시각) / Δ(단조 시계).

로그는 `logs/eval/<run_name>/` 에, 리포트 표는 같은 디렉토리의 `report.md` 에 남는다. 커버리지는
`./scripts/test.sh` 가 `logs/coverage/` 에 낸다.
