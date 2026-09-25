# 통합 테스트 시나리오

> 명세 4.10: 통합 테스트 시나리오를 최소 10개 작성하고 자동화한다.

시나리오 14개를 `launch_testing` 파일 하나씩(`test_NN_<slug>.py`)으로 작성했다. 러너는
`launch_testing_ros.LaunchTestRunner`, 기동 게이트는 `launch_testing_ros.WaitForTopics` 다. 실행 방법·결과 형식·하네스
구조는 [../README.md](../README.md), 최근 실행 결과와 조건은 [docs/reports/integration_tests.md](../../docs/reports/integration_tests.md).

```bash
docker compose exec dev ./scripts/run_integration.sh           # 기본 세트 (14 제외, GPU·--shm-size 8g 컨테이너)
docker compose exec dev ./scripts/run_integration.sh 4 9 13    # 선택
./scripts/run_integration.sh --sim kinematic --profile component 2 4 9 13   # GPU 없는 CI (운동학 구성)
./scripts/run_integration.sh --soak-hours 4 14                 # 4 h 연속 운용
```

## 판정 원칙

- **skip 은 통과가 아니다.** 시나리오마다 쓰는 워크스페이스 패키지(catalog `needs`)가 모두 설치돼 있는데 전부 또는
  일부 skip 되면 집계(`amr_itest/report.py`)가 실패로 센다 — GPU 없음, 백엔드·구성 불가, `--standins never` 도 포함.
  패키지가 없는 부분 워크스페이스에서만 skip 으로 남는다. 종료 요약에 실행·skip 수(시나리오·테스트 케이스)를 찍는다.
  `--allow-skip` 은 하네스 개발용, `--fail-on-skip` 은 모든 skip 을 실패로.
- **명세 판정 구성이 기본이다.** 04(위치 추정)·13(응답 시간)은 `system` 구성(실제 AMCL, 실제 실행기 → Nav2 →
  velocity_profiler → safety 체인)이 첫 번째다. `component` 구성(AMCL·실행기 대역)은 체인 회귀용이며 판정 이름에
  `EKF plumbing (AMCL stand-in)` · `[component: executor stand-in]` 이 붙는다. `--standins never` 는 대역
  (운동학 시뮬레이터, AMCL, 실행기 포함)으로 합격하지 못하게 건너뛴다.
- **map 프레임 = 월드 프레임을 재서 확인한다.** `maps/warehouse.yaml` 은 월드에 정합된 지도이고 시나리오는 map 좌표로
  목표를 주고 월드(GT·SDF)로 판정한다. system 구성 시나리오는 판정 전에 `/map` 점유 셀을 월드 SDF visual 의 LiDAR 평면
  단면에 SE(2) 로 맞춰 항등인지 본다 (`check_map_registration`: 잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°, 거리 중앙값 ≤ 5 cm).
  03 은 SLAM 지도(월드 원점 스폰)에 같은 판정을 한다. 스폰 자세는 모든 system 시나리오가 명시한다.
- **launch 인자는 계약을 대조한다.** `Stack.system()`/`multi_robot()` 은 bringup 의 `with_<스택>` 인자만 넘기고
  (`SYSTEM_ARGS`), 단위 시험(`unit/test_launch_contract.py`)이 설치된 `system.launch.py`·`multi_robot.launch.py` 의
  `DeclareLaunchArgument` 와 대조한다. 시나리오 파일의 요구사항(런치·실행 파일·설정)이 머지된 트리에 있는지도 본다.
- **지연은 항상 최댓값으로 판정한다.** 부하 중(loadavg > CPU/2) 초과면 그 묶음을 한 번 다시 재고 두 시도를 모두 남긴다.
  음수 지연(프로브가 원인보다 결과를 먼저 받음)은 자르지 않는다.
- **측정 중 크래시만 실패다.** pre-shutdown 마지막 테스트가 그때까지 죽은 프로세스를 기록하고, 그 뒤 launch 종료
  단계에서만 난 비정상 코드(SIGINT 중 -11 등)는 `shutdown_exit_warnings` 로 남긴다. Nav2 는 lifecycle 활성을 확인한 뒤
  요청한다.
- **러너 상한 안에서 측정값을 남긴다.** 시행 대기는 판정 기준 + 여유로 잡고, 반복 시나리오는 러너가 준 상한
  (`ITEST_SCENARIO_TIMEOUT`)까지 남은 시간으로 새 시행을 정한다. CSV·시행별 판정은 시행마다 갱신한다.

## 시나리오

지표·기준·로그 포맷의 단일 출처는 `amr_itest/catalog.py` 다 (`python3 -m amr_itest.report --readme-table`).

| # | 시나리오 | 지표 | 합격 기준 | 로그 | 구성 / 백엔드 |
| --- | --- | --- | --- | --- | --- |
| 1 | 시뮬레이션 기동 및 센서 토픽 발행 | 센서 토픽 header.stamp(sim time) 주기: 중앙값 1/median(Δt) = 센서 주기, 평균 (n−1)/span = 전달률; 메시지 규격(빔 수·해상도·FOV·거리), 정지 상태 노이즈 σ (LiDAR 빔별, IMU, depth 화소별) | 규정 주기 = max(명세, sensors.yaml): 중앙값 ≥ 0.98×(지터만 허용), 전달률 평균 ≥ 0.9×; 규격 일치; LiDAR σ ∈ [0.5, 2]×0.03 m; depth 유효값 ≤ 10 m, 정지 화소 σ > 0 | `topic_rates.csv [topic, type, nominal_hz, count, mean_rate_hz, median_rate_hz, max_gap_s, wall_rate_hz, frame_id, ok]` | component / gazebo |
| 2 | TF 트리 무결성 | /tf·/tf_static 간선 그래프 vs 계약 간선(map→odom→base_footprint→base_link→센서): 존재·부모 유일성·순환·정적 변환 값(config extrinsic)·동적 간선 주기 | 누락 간선 0, 이중 부모 0, 순환 0, 루트 = map 하나, 정적 변환 오차 ≤ 1 mm / 0.1°, EKF 간선 ≥ 0.9×50 Hz | `tf_edges.csv [parent, child, publisher, kind, present, translation_error_m, rotation_error_deg, rate_hz, ok, problems] + frames.dot` | component / gazebo, kinematic |
| 3 | SLAM 맵 생성 | slam_toolbox /map (월드 원점 스폰 → map = 월드) vs 월드 SDF visual 의 LiDAR 평면 단면: 해상도, 보이는 구조물 가장자리 재현율, 자유 공간 오점유율, SE(2) 잔여 정합 | resolution ≤ 0.05 m, 재현율 ≥ 0.9, 오점유 ≤ 5 %, map↔월드 항등 정합 (잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°, 거리 중앙값 ≤ 5 cm), save_map 성공 | `map_info.json {resolution, width, height, origin} + map.pgm/yaml (save_map) + map_received.pgm/yaml (판정한 /map)` | system / gazebo |
| 4 | EKF 위치 추정 정확도 | odometry/filtered_map vs ground_truth/odom (GT 를 추정 스탬프에 보간), GT 속도로 정지/직선/회전 분류, 구간별 RMSE·최대. system: 실제 AMCL + maps/warehouse.yaml (map↔월드 정합 확인 후) / component: AMCL 대역(GT+σ 1.5 cm) — EKF 배관 확인일 뿐 명세 판정 아님 | RMSE 정지 ≤ 0.03 m, 직선 ≤ 0.05 m, 회전 ≤ 0.08 m; 구간별 ≥ 100 샘플; system: map↔월드 항등 정합 (잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°, 거리 중앙값 ≤ 5 cm) | `pose_error.csv [timestamp, gt_x, gt_y, est_x, est_y, error, yaw_error, segment] (amr_evaluation pose_error_logger) + harness_pose_error.csv` | system, component / gazebo, kinematic |
| 5 | Kidnapped Robot 복구 | Gazebo set_pose 로 로봇 순간 이동 → localization/lost 전이 → 복구 후 odometry/filtered_map vs GT 오차, 복구 시간 | lost 감지 ≤ 5 s, 복구(오차 ≤ 0.10 m 로 3 s 유지) ≤ 60 s, 5회 중 5회; map↔월드 항등 정합 (잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°, 거리 중앙값 ≤ 5 cm) | `kidnap.csv [trial, from_x, from_y, to_x, to_y, detect_s, recover_s, final_error_m, ok] (시행마다 갱신)` | system / gazebo |
| 6 | 경로 계획 성공률 | 월드 자유 지점 50 쌍(시드 고정)에 compute_path_to_pose(planner_id=AStar, 직접 구현 A*), planner_server 활성·지도 수신 후; 성공·계획 시간(result.planning_time)·경로 길이; 참고 NavFn·Smac 같은 쌍 | 성공률 ≥ 98 % (≥ 49/50), 경로가 구조물(visual∪collision)을 지나지 않음, 계획 시간 최대 ≤ 500 ms; map↔월드 항등 정합 (잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°, 거리 중앙값 ≤ 5 cm) | `plans.csv [pair, planner, start_x, start_y, goal_x, goal_y, success, plan_ms, length_m, straight_m, collision_free]` | system / gazebo |
| 7 | 경로 추종 CTE | navigate_to_pose 주행 중 plan(map) vs ground_truth(월드) 수직 거리 — map = 월드 정합을 먼저 확인, 곡률로 직선/곡선 분리 평균 (amr_evaluation cte_logger → analyze) | 평균 \|CTE\| 직선 ≤ 0.05 m, 곡선 ≤ 0.10 m, 목표 전부 도달; map↔월드 항등 정합 (잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°, 거리 중앙값 ≤ 5 cm) | `cte.csv [timestamp, planned_x, planned_y, actual_x, actual_y, cte, segment] (amr_evaluation cte_logger)` | system / gazebo |
| 8 | 동적 장애물 회피 | 작업자 횡단선을 지나는 왕복 30회: amr_simulation collision_monitor(사람·지게차·셔틀 모두) 접촉·최소 거리, 시행별 최근접 장애물·TTC, 첫 계획 경로 대비 GT 이탈, 이탈 > 0.3 m 구간 뒤 복귀 시간 | 접촉 0 / 30회, 최대 이탈 ≤ 1.0 m, 복귀 ≤ 5 s, 목표 도달 ≥ 29회, 실제 조우(최근접 동적 장애물 ≤ 2.0 m) ≥ 시행/6 (30회면 5), 시행 ≥ 30 | `avoidance.csv [trial, reached, time_s, contacts, min_distance_m, nearest, min_ttc_s, max_deviation_m, episodes, return_s, contacts_robot_moving, episodes_open, open_peak_m, open_peak_t, dev_at_end_m, goal_dist_m, plan_end_dist_m, track_end_gap_s, track_n, loc_err_m, loc_goal_dist_m, min_goal_dist_m] (시행마다 갱신), contacts.csv [trial, time, obstacle, distance_m, robot_speed_mps, robot_moving, obstacle_speed_mps, obstacle_heading_deg, lane_lateral_m, lane_along_m, yield_state, stop_distance_m] (뒤 6개는 원인 귀속 근거 — 판정에 쓰지 않는다), episodes.csv [trial, t_start, t_peak, peak_m, t_end, return_s, stopped_frac, mean_v_mps, max_v_mps, yield_frac, cmd_v_mean, gate_out_v_mean, vo_rejected_mean] (복귀가 늦은 구간에서 로봇이 멈춰 있었는지·무엇이 세웠는지), 접촉이 난 시행만 stats_trial<N>.csv [t, cmd_v, cmd_w, yield_state, stop_distance_m, vo_rejected, collisions, ttc_s, zone_entry_m, yield_obstacle] 와 tracks_trial<N>.csv [t, track_id, x, y, vx, vy, heading_deg, is_dynamic] (제어 이력·추적 입력 — 전부 근거일 뿐 판정에 쓰지 않는다)` | system / gazebo |
| 9 | 긴급 정지 | (a) 장애물 주입 → 첫 cmd_vel=0: 침범 스캔 수신 기준(safety 처리) · 주입 기준(스캔 주기 포함) (b) E-stop 버튼(estop, /fleet/estop) → cmd_vel=0, 래치·reset (c) 벽 접근 폐루프: 정지 명령 시점 GT 여유 | 최댓값으로 판정 (부하 중 초과면 한 번 재측정): 스캔→0 ≤ 20 ms, 주입→0 ≤ 스캔 주기 + 20 ms, 버튼→0 ≤ 20 ms; 정지 시점 여유 ≥ 0.25 m, 충돌 0; 버튼 래치는 reset_estop 성공 전까지 (해제 false 뒤에도) 유지 | `estop_latency.csv [trial, kind, t_trigger, t_zero, latency_ms, clearance_m] (음수 = 프로브 해상도, 그대로) + approach.csv` | component / kinematic, gazebo |
| 10 | 정밀 도킹 | dock 액션 10회: GT 최종 자세 vs 기대 도킹 자세(월드 마커 판 면 + behavior.yaml standoff, 마커 법선 반대 방위) — 서버 자기 보고는 참고; 마커 가림 1회: 재시도 소진 | GT 위치 ≤ 0.02 m, 각도 ≤ 1°, 10/10; 마커 가림 시 success=false · attempts_used = 3; map↔월드 항등 정합 (잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°, 거리 중앙값 ≤ 5 cm) | `docking.csv [trial, dock_id, success, attempts, gt_pos_err_m, gt_ang_err_deg, srv_pos_err_m, srv_ang_err_deg, time_s] (시행마다 갱신)` | system / gazebo |
| 11 | BT 에러 복구 | 작업 중 에러 주입 3종 — 경로 차단(작업 끝까지 유지), 위치 상실(순간 이동), 인식 실패(적재 도크 상자 치움, 인식 복구 관측 후 복원) → 복구 증거(우회 경로·RECOVERING·재위치추정·재인식) + task_status | 3종 모두 주입 뒤 ≤ 60 s 에 복구 증거, 작업 COMPLETED (작업 ≤ 600 s) | `recovery.csv [case, injected_at, recovered_at, recovery_s, final_status, recovery_seen, phases, reset_before]` | system / gazebo |
| 12 | 5대 동시 운용 교착 | (a) 도크 간 작업 → /fleet/task_events 완료, /fleet/traffic_events 탐지·해소, CPU = 시뮬레이터 포함 시스템 프로세스 CPU 합 / 호스트 코어 (b) 강제 교착: 교차로 x_ab_4 안 E-stop 로봇 + 실행기 작업으로 그 교차로를 지나야 하는 로봇 → BLOCKED 탐지·대체 경로 해소 | (a) 로봇 5대, 작업 모두 COMPLETED (작업당 ≤ 600 s), 탐지된 교착은 모두 해소(UNRESOLVED 0), 평균 CPU ≤ 80 % (b) traffic/DEADLOCK ≥ 1 이고 모두 traffic/RESOLVED | `traffic.csv [recv_time, level, event, robot, message] + tasks.csv + cpu.csv [time, system_cpu_percent, host_cpu_percent, cores_used] + forced.csv` | system / gazebo |
| 13 | 응답 시간 | 작업 명령(assign_task, header.stamp = 발행 시각) → 로봇 첫 움직임 = ground_truth/odom 의 \|v\| ≥ 0.01 m/s 또는 \|ω\| ≥ 0.02 rad/s 첫 표본 (참고: \|v\| ≥ 0.05 m/s 변형). system: 실제 task_executor → Nav2 → velocity_profiler → safety_node / component: 실행기 대역 (체인 회귀용) | 평균 ≤ 200 ms (벽시계, 최대 기록), 샘플 ≥ 50, 무응답 0 | `response_time.csv [cmd_time, response_time, latency_ms, cmd_id] (amr_evaluation response_time_logger, odom 모드) + harness_response_time.csv (+ wall_ms, latency_05_ms, cmd_vel_ms, rtf)` | system, component / gazebo, kinematic |
| 14 | 4시간 연속 운용 | system 스택에 작업을 반복 투입(작업당 상한 시간)하며 프로세스별 RSS 샘플링(60 s), 선형 회귀 기울기, 프로세스 종료 감시, 완료 작업 수 | 크래시 0, 프로세스별 RSS 증가율 ≤ 5 MB/h (첫 10 분 워밍업 제외, 적합 증가량 ≤ 2 MB 는 할당기 요동으로 봄, 표본 < 5 는 판정 불가 = 실패), 작업 실패율 ≤ 2 % (작업당 900 s 초과 = 실패), 완료 작업 ≥ 4 /h × 기간 (멈춘 실행기 검출) | `memory.csv [time, process, pid, rss_mb] + tasks.csv + soak.json` | system / gazebo, 장시간 (명시 선택 시만) |

시행 수 노브 (명세 캠페인 기본값 — 줄여서 스모크하면 시행 수 판정이 실패로 드러난다): `ITEST_TRIALS` (08, 30),
`ITEST_RT_SAMPLES` (13, 50), `ITEST_MR_TASKS` (12, 5), `ITEST_MR_ROBOTS` (12, 5 — 줄이면 '로봇 수' 판정 실패),
`ITEST_SOAK_HOURS` / `--soak-hours` (14, 4.0).

## 명세 7장과의 차이 (기록)

명세 7장은 "모든 소스 코드는 src 폴더 내 패키지" 를 요구한다. 이 하네스(`amr_itest`, 대역 노드 포함)는 제품 코드가
아니라 빌드된 워크스페이스를 밖에서 시험하는 도구라 `tests/integration` 에 두었다: colcon 패키지로 만들면
`colcon test` 가 시나리오(Gazebo·GPU 필요)를 패키지 시험으로 끌어들이고, 제품 패키지가 시험 대역 노드에 의존할 수 있게
된다. 대신 하네스 자체의 단위 시험(`unit/`, 커버리지)은 `run_integration.sh --unit-only --coverage` 와 CI 가 돌린다.
