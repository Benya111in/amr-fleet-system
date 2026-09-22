# 통합 테스트 시나리오

> 명세 4.10: 통합 테스트 시나리오를 최소 10개 작성하고 자동화한다.

시나리오 14개를 `launch_testing` 파일 하나씩(`test_NN_<slug>.py`)으로 작성했다. 러너는
`launch_testing_ros.LaunchTestRunner`, 기동 게이트는 `launch_testing_ros.WaitForTopics` 다. 각 파일은
필요한 노드·런치·GPU 를 먼저 확인하고, 없으면 **무엇이 없는지 정확한 사유로 skip** 한다 — 패키지가
머지되면 같은 파일이 그대로 켜진다. 실행 방법·결과 형식·하네스 구조는 [../README.md](../README.md).

```bash
docker compose exec dev ./scripts/run_integration.sh           # 기본 세트 (14 제외)
docker compose exec dev ./scripts/run_integration.sh 4 9 13    # 선택
```

## 시나리오

지표·기준·로그 포맷의 단일 출처는 `amr_itest/catalog.py` 다
(`python3 -m amr_itest.report --readme-table` 로 이 표의 원본을 뽑는다).

| # | 시나리오 | 측정 지표 | 합격 기준 | 로그 (logs/itest/NN_slug/) | 백엔드 | 상태 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 시뮬레이션 기동 및 센서 토픽 발행 | 센서 토픽 header.stamp(sim time) 주기 (평균 (n−1)/span, 중앙값 1/median Δt), 메시지 규격, 정지 노이즈 σ | 평균 ≥ 0.9×, 중앙값 ≥ 0.95× 규정 주기; LiDAR 720빔·0.5°·360°·25 m; 카메라 640×480·HFOV 87±1°; LiDAR σ ∈ [0.5, 2]×0.03 m; IMU σ > 0 | `topic_rates.csv` | gazebo | 구현 |
| 2 | TF 트리 무결성 | /tf·/tf_static 간선 그래프 vs 계약 간선, tf2 map→센서 조회 | 누락·이중 부모·순환 0, 루트 = map, 정적 오차 ≤ 1 mm / 0.1°, EKF 간선 ≥ 45 Hz | `tf_edges.csv`, `frames.dot` | gazebo → kinematic | 구현 |
| 3 | SLAM 맵 생성 | /map 해상도, 월드 SDF 정적 구조물 가장자리 재현율, 오점유율 | ≤ 0.05 m, 재현율 ≥ 0.9, 오점유 ≤ 5 %, save_map 성공 | `map_info.json`, `map.pgm/yaml` | gazebo | 스켈레톤 |
| 4 | EKF 위치 추정 정확도 | odometry/filtered_map vs ground_truth (GT 보간), 정지/직선/회전 구간 RMSE | RMSE ≤ 3 / 5 / 8 cm, 구간별 ≥ 100 샘플 | `pose_error.csv` [timestamp, gt_x, gt_y, est_x, est_y, error, yaw_error, segment] + `harness_pose_error.csv` | kinematic → gazebo | 구현 |
| 5 | Kidnapped Robot 복구 | set_pose 순간 이동 → localization/lost → 오차 ≤ 0.1 m 3 s 유지 | 감지 ≤ 5 s, 복구 ≤ 60 s, 5/5 | `kidnap.csv` | gazebo | 스켈레톤 |
| 6 | 경로 계획 성공률 | compute_path_to_pose 50쌍 (자유 지점, 시드 고정) | ≥ 98 % (49/50), 경로가 구조물 내부를 지나지 않음 | `plans.csv` | gazebo | 스켈레톤 |
| 7 | 경로 추종 CTE | navigate_to_pose 주행 중 plan vs GT 수직 거리 (cte_logger) | 평균 \|CTE\| 직선 ≤ 5 cm, 곡선 ≤ 10 cm | `cte.csv` [timestamp, planned_x/y, actual_x/y, cte, segment] | gazebo | 스켈레톤 |
| 8 | 동적 장애물 회피 | 작업자 횡단 경로 30회: GT 풋프린트 ~ actor 최소 여유, 계획 경로 대비 이탈 | 충돌 0/30, 이탈 ≤ 1.0 m, 도달 ≥ 29 | `avoidance.csv` | gazebo | 스켈레톤 |
| 9 | 긴급 정지 | 장애물 주입·E-stop 버튼 → 첫 cmd_vel = 0 지연, 벽 접근 폐루프 GT 여유 | 지연 ≤ 100 ms (외부 부하 시 p95), 정지 시점 여유 ≥ 0.25 m, 충돌 0, 래치·reset | `estop_latency.csv`, `approach.csv` | kinematic → gazebo | 구현 |
| 10 | 정밀 도킹 | dock 액션 10회 결과 오차 (+ GT 참고) | 위치 ≤ 2 cm, 각도 ≤ 1°, 10/10 | `docking.csv` | gazebo | 스켈레톤 |
| 11 | BT 에러 복구 | E-stop·경로 차단·위치 상실 주입 → task_status | 3종 모두 COMPLETED, ≤ 180 s | `recovery.csv` | gazebo | 스켈레톤 |
| 12 | 5대 동시 운용 교착 | /fleet/traffic_events 교착 탐지·해소, 작업 완료, 시스템 CPU | 탐지 ≥ 1·전부 해소, 5/5 완료, CPU 평균 ≤ 80 % | `traffic.csv`, `cpu.csv` | gazebo | 스켈레톤 |
| 13 | 응답 시간 | assign_task header.stamp → 첫 cmd_vel ≠ 0 (sequences.md §1), 50회; 참고: GT 첫 \|v\| ≥ 0.05 m/s | 평균 ≤ 200 ms, 무응답 0 | `response_time.csv` [cmd_time, response_time, latency_ms, cmd_id] + `harness_response_time.csv`, `harness_motion_onset.csv` | kinematic → gazebo | 구현 |
| 14 | 4시간 연속 운용 | 작업 반복 투입 + 프로세스 RSS(60 s), 크래시 감시 | 크래시 0, RSS ≤ 5 MB/h, 작업 실패율 ≤ 2 % | `memory.csv`, `soak.json` | gazebo | 스켈레톤, 장시간 (명시 선택 시만) |

상태
- **구현**: 지금 워크스페이스에서 끝까지 돈다. 아직 머지되지 않은 노드(wheel_odometry, imu_filter,
  scan_filter, velocity_profiler, safety, task_executor, amcl)는 계약 인터페이스 그대로의 대역
  (`amr_itest/standins/`)이 채우고, 실제 노드가 설치되면 자동으로 실제 노드를 쓴다
  (`result.json` 의 `components` 에 real/standin 기록, `--standins never` 면 대역 금지).
- **스켈레톤**: 판정 코드까지 계약 토픽·액션 기준으로 완성돼 있고, 필요한 패키지
  (amr_localization·amr_navigation·amr_perception·amr_behavior·amr_fleet 의 런치/노드)가 없으면
  skip 한다.

측정 경로: 명세 지표(04 위치 오차, 13 응답 시간, 07 CTE, 12 CPU)는 amr_evaluation 로거가
명세 표준 로그(`pose_error.csv` 등)를 쓰고 `analyze` 로 판정하며, 하네스가 같은 지표를 자기
기록으로 독립 계산해 함께 판정한다 (`harness_*.csv`).

## 최근 검증

2026-09-22, 일회용 컨테이너 `amr-fleet-system:wf-final` + `--gpus all`(RTX 5090), 워크스페이스
= feature/dev-process-docs + amr_description(feature/robot-description) + amr_simulation 월드 스냅샷 +
amr_evaluation(feature/evaluation-tools). 호스트 32 스레드, 실행 중 loadavg 4~20 (CPU 수의 절반 16
미만 → 지연 판정은 최댓값, `provisional_under_load=false`). 하네스 단위 테스트 57 통과, 커버리지 97 %.

| 실행 | 결과 | JUnit | 소요 |
| --- | --- | --- | --- |
| 기본 세트 + 14 (`run_integration.sh --coverage --include-long`) | 통과 5 (01·02 gazebo, 04·09·13 kinematic), skip 9 (사유 = 없는 런치/노드 목록) | 115 tests, 0 failures, 0 errors, 29 skipped | 240 s |
| Gazebo 백엔드 (`--sim gazebo 1 2 4 9 13`) | 통과 5 | 29 tests, 0 failures, 2 skipped (09 의 장애물·벽 주입은 운동학 전용) | 207 s |
| 실제 노드 (형제 워크트리 amr_localization·amr_navigation·amr_perception 사본, 대역 자동 교체) | kinematic 2·4·9·13 통과 4, gazebo 1·2·4·9·13 통과 5 | 21 + 29 tests, 0 failures | 239 + 260 s |

측정값 (대역 / 실제 노드, 백엔드 표기)

| # | 지표 | 기준 | 측정 |
| --- | --- | --- | --- |
| 1 | 센서 주기 (sim time, RTF 0.97~0.99) | ≥ 0.9× 규정 | scan 10.0 · imu/data_raw 100.0 · RGB 30.3 · depth 15.15 · camera_info 30.3/15.15 · ground_truth 50.0 Hz, joint_states 1000 Hz (Fortress JointStatePublisher 는 물리 스텝마다 — amr_gazebo.xacro 설계, 엔코더 노드가 50 Hz 로 서브샘플링). LiDAR 정지 σ 0.029~0.030 m, 720 빔·0.5°·360°·25 m, 카메라 640×480·HFOV 87.0° |
| 2 | TF 간선 | 누락·이중 부모·순환 0 | 계약 간선 10개 전부 (map→odom 50 Hz, odom→base_footprint 50 Hz, 정적 6개 오차 0, 바퀴 2개 20 Hz), 루트 map, tf2 map→센서 조회 전부 성공 — 대역·실제 노드 모두 |
| 4 | EKF RMSE 정지/직선/회전 | 3 / 5 / 8 cm | 2.14 / 1.79 / 1.93 cm (kinematic), 2.13 / 1.78 / 1.93 cm (gazebo), 실제 wheel_odometry·imu_filter 로 2.13 / 1.77~1.79 / 1.93 cm; amr_evaluation analyze 2.01 / 1.78 / 1.93 cm. 최대 3.65 cm. **AMCL 은 대역(GT + σ 1.5 cm)** 이라 이 값은 이중 EKF 설정·측정 경로 검증이고 AMCL 정확도는 아니다 (≈ √2·σ) |
| 9 | 정지 지연 (최댓값) | ≤ 100 ms | 침범 스캔 → cmd_vel=0: 0.42 ms (대역) / 0.35 ms (실제 safety_node). 버튼 estop·/fleet/estop: 0.40·0.30 ms (kinematic 대역), 0.26·0.31 ms (gazebo 대역), 0.49·0.51 ms (실제, kinematic), 1.67·0.78 ms (실제, gazebo). 벽 접근 정지 시점 여유 0.38 m (대역) / 0.40 m (실제), 충돌 0. 래치·reset 거부·reset 후 재개 모두 통과 |
| 13 | 명령 → 첫 cmd_vel ≠ 0 (평균, 50회) | ≤ 200 ms | 1.4 ms (대역 체인), 37.7 ms (실제 velocity_profiler·safety, kinematic), 41.8 ms (실제, gazebo); amr_evaluation 1.05 / 37.8 / 41.4 ms. 참고 지표 GT \|v\| ≥ 0.05 m/s 도달: 58 / 67 ms (대역), **246 / 257 ms (실제 S-커브 프로파일러)** — 저크 2 m/s³ 에서 0.05 m/s 까지 224 ms 가 물리 하한이라 판정은 sequences.md §1 정의를 따른다 |

실제 노드 실행에서 하네스가 찾은 계약 해석 차이 두 가지는 시나리오에 반영했다: (1) 실제 safety_node 는
버튼이 눌린 동안의 reset 을 거부하되 1 s(`reset_grace`) 보류했다가 false 가 오면 해제한다(대시보드의
false/reset 경합 대응) — 09 는 "눌린 동안 거부·정지 유지"만 판정하고 해제 순서는 false → reset 으로
검사한다. (2) 13 의 판정 정의를 sequences.md §1("첫 cmd_vel ≠ 0")에 맞추고 GT 움직임은 참고로 남겼다.
