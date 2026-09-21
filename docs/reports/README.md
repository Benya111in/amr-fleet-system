# 성능 측정 및 분석 리포트

> 명세 10장 요구 산출물. 측정 절차는 명세의 "성능 지표 측정 절차 표준화" 표를 따르며,
> 측정 도구는 `src/amr_evaluation` 패키지로 표준화한다.

## 측정 대상 지표

| 지표 | 목표 | 산출 방법 | 상태 |
| --- | --- | --- | --- |
| 위치 추정 RMSE | 정지 3cm / 직선 5cm / 회전 8cm | GT 대비 제곱평균제곱근, 100개 시점 이상 | 미측정 |
| 경로 추종 CTE | 직선 5cm / 곡선 10cm | 계획 경로와 실제 궤적의 수직 거리 | 미측정 |
| 응답 시간 | 200ms 이내 | 명령 발행 ~ 첫 움직임, 50회 이상 | 미측정 |
| 경로 계획 성공률 | 98% 이상 (50쌍) | 성공/전체 | 미측정 |
| 경로 재계획 시간 | 500ms 이내 | 재계획 트리거 ~ 완료 | 미측정 |
| 동적 회피 충돌 | 30회 중 0건 | 충돌 이벤트 카운트 | 미측정 |
| 도킹 정밀도 | 위치 2cm / 각도 1도 | GT 대비 최종 오차 | 미측정 |
| 작업 성공률 | 97% 이상 | 완료/전체 | 미측정 |
| CPU 사용률 | 5대 운용 시 80% 이하 | 지속 샘플링 | 미측정 |
| 테스트 커버리지 | 70% 이상 | `colcon test` / `pytest --cov` | 미측정 |
| 연속 운용 | 4시간 안정성 | 장시간 테스트 | 미측정 |

## 측정 도구: `amr_evaluation`

표의 앞 세 지표와 CPU 사용률은 `amr_evaluation` 의 로거 노드가 CSV 로 남기고 `analyze` 가 판정한다.
나머지 지표(계획 성공률, 재계획 시간, 충돌, 도킹, 작업 성공률)는 각 패키지의 통합 테스트/작업 로그에서 집계한다.

| 지표 | 노드 (`ros2 run amr_evaluation …`) | 입력 토픽 (로봇 네임스페이스 기준) | 출력 CSV |
| --- | --- | --- | --- |
| 위치 추정 RMSE | `pose_error_logger` | `ground_truth/odom` (GT, `nav_msgs/Odometry`), `odometry/filtered_map` (EKF) | `pose_error.csv` |
| 경로 추종 CTE | `cte_logger` | `plan` (`nav_msgs/Path`), `ground_truth/odom` | `cte.csv` |
| 응답 시간 | `response_time_logger` | `/fleet/task_events` (`amr_msgs/Task`, IN_PROGRESS 전이 = 명령), `ground_truth/odom` (첫 움직임) | `response_time.csv` |
| CPU 사용률 | `cpu_sampler` | `/proc/stat` 1 Hz (이미지에 mpstat 없음) | `cpu.csv` |

파라미터 기본값은 `src/amr_evaluation/config/amr_evaluation.yaml` (토픽, 구간 임계값, 움직임 판정 속도 등).

### 절차

1. 시스템을 기동한다 (`docker compose --profile run up -d` 또는 개별 launch). GT 토픽 `ground_truth/odom` 은
   시뮬레이션 브리지가 로봇 네임스페이스 아래에 발행한다.
2. 로거를 한 런 이름으로 묶어 기동한다 (컨테이너 셸에서):

   ```bash
   ros2 launch amr_evaluation evaluation.launch.py run_name:=ekf_s2_straight namespace:=amr_01
   ```

   네 로거가 모두 `logs/eval/<run_name>/` (기본 `$ROS_WS/logs/eval`, `output_dir:=` 로 변경) 에 쓴다.
   `use_sim_time` 기본값은 `true` — 타임스탬프는 시뮬레이션 시각(메시지 헤더)이다. 응답 시간은
   `robot_id` (기본 = `namespace`) 로봇의 작업 이벤트만 센다.
3. 시나리오를 수행한다 (정지 / 직선 / 회전 구간이 각각 100 샘플 이상 들어가도록: 50 Hz 에서 2 s 이상씩).
   응답 시간은 정지 상태에서 받은 작업 명령만 센다 (`require_rest`) — 50 회 이상 작업을 투입한다.
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
| 위치 추정 오차 | 구간별 RMSE (`--gate-metric max|p95|mean` 로 변경) | 정지 3 / 직선 5 / 회전 8 cm | `--pose-thresholds 0.03,0.05,0.08` |
| CTE | 구간별 평균 \|CTE\| | 직선 5 / 곡선 10 cm | `--cte-thresholds 0.05,0.10` |
| 응답 시간 | 평균 | 200 ms | `--response-ms 200` |
| CPU | 평균 | 80 % | `--cpu-percent 80` |

- 종료 코드 `0` 통과, `1` 기준 위반, `2` 런 디렉토리 없음. `--no-fail` 이면 위반이 있어도 `0`.
- 샘플 부족(위치 100 시점, 응답 50 회 미만)·구간 샘플 없음·CSV 없음은 경고로 표에 적힌다. `--strict` 면 경고도 실패.
- max / p95 / mean 은 참고 열에 함께 적힌다. `--align` 을 주면 SE(2) 최소제곱 정렬 후 오차도 참고 행으로
  더한다 (map↔world 프레임의 상수 오프셋이 의심될 때; 판정은 항상 원시 오차로 한다).
- 통합 테스트 예: `ros2 run amr_evaluation analyze --input logs/eval/<run> || exit 1`

## 로그 포맷

명세 표의 열을 앞에 그대로 두고, 분석에 필요한 열을 뒤에 덧붙인다 (단위: m, rad, s, ms, %).

```
pose_error.csv    : [timestamp, gt_x, gt_y, est_x, est_y, error] + yaw_error, segment(정지|직선|회전)
cte.csv           : [timestamp, planned_x, planned_y, actual_x, actual_y, cte] + segment(직선|곡선)
response_time.csv : [cmd_time, response_time, latency_ms] + cmd_id
cpu.csv           : [timestamp, cpu_total_percent, cpu0, cpu1, ...]
```

- `pose_error`: GT 를 추정 스탬프 시각에 선형 보간한다 (최근접 GT 샘플로 외삽하지 않는다 — 2 m/s 에서
  10 ms 어긋나면 2 cm 의 가짜 오차가 생긴다). GT 간격이 `gt_max_gap`(0.2 s) 을 넘는 구간은 버린다.
  구간은 GT 속도로 나눈다: 정지 |v| < 0.02 m/s 이고 |ω| < 0.02 rad/s, 직선 |ω| < 0.05 rad/s, 회전 그 외.
- `cte`: GT 위치에서 최신 `plan` 폴리라인의 최근접 선분까지 수직 거리, 경로 진행 방향 좌측이 +. 경로 양 끝
  너머는 끝점까지의 거리. `planned_x/y` 는 경로 위 최근접점. 곡률 |κ| > 0.1 1/m (반경 10 m 미만) 인 정점과
  그 앞뒤 0.5 m 를 곡선으로 본다. GT(world) 와 경로(map) 프레임이 일치한다고 가정한다.
- `response_time`: `cmd_time` 은 `Task.header.stamp` (0 이면 수신 시각), `response_time` 은 |v| ≥ 0.05 m/s
  또는 |ω| ≥ 0.1 rad/s 인 첫 GT 샘플의 스탬프. 10 s 안에 움직임이 없으면 무응답으로 센다.
  명령·움직임 원천은 노드 파라미터로 바꾼다 (`params_file:=` 로 YAML 을 넘기거나 로거를 따로
  `ros2 run amr_evaluation response_time_logger --ros-args -r __ns:=/amr_01 -p …` 로 띄운다).
  목표 자세를 직접 발행하는 실험은 `cmd_type:=pose cmd_topic:=goal_pose`. 설계 문서
  ([sequences.md](../architecture/sequences.md) §1) 의 "첫 `cmd_vel` ≠ 0" 정의로 재려면
  `motion_type:=twist motion_topic:=cmd_vel motion_threshold:=0.001` (응답 시각 = 수신 시각). 기본값
  (GT 첫 움직임) 은 가속 지연까지 포함하므로 더 엄격하다.
- `cpu`: `/proc/stat` 두 스냅샷의 jiffies 차분. 컨테이너 안에서도 호스트 전체 CPU 다.

로그는 `logs/eval/<run_name>/` 에, 리포트 표는 같은 디렉토리의 `report.md` 에 남는다. 커버리지는
`./scripts/test.sh` 가 `logs/coverage/` 에 낸다.
