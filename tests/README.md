# 테스트

명세 4.10 "테스트 및 검증" 의 두 층을 나눠 둔다.

| 층 | 위치 | 실행 | 산출 |
| --- | --- | --- | --- |
| 단위 테스트 (패키지별, 커버리지 ≥ 70 %) | `src/<pkg>/test/` | `./scripts/test.sh [--packages-select <pkg>]` | `colcon test-result`, `logs/coverage/` |
| 통합 테스트 시나리오 (≥ 10, 자동화) | `tests/integration/` | `./scripts/run_integration.sh [시나리오...]` | `logs/itest/` (JSON + CSV + JUnit) |

`tests/` 는 `src/` 밖이라 colcon 패키지가 아니다. 통합 테스트는 **빌드된 워크스페이스를 대상으로**
스크립트가 단독 실행한다 (먼저 `./scripts/build.sh`).

시나리오 파일 하나 = `launch_testing` 런 하나. 러너는 `launch_testing_ros.LaunchTestRunner` 다
(`python3 -m amr_itest.launch_test_ros <file> --junit-xml …` — ros2test 의 `ros2 test` 와 같은 경로.
Humble 이미지에는 ros2test 가 없어 진입점을 하네스에 두었다). pytest 로 직접 돌려도
launch_testing_ros pytest 플러그인이 같은 러너를 쓴다:

```bash
source install/setup.bash && export PYTHONPATH=$PWD/tests/integration
python3 -m amr_itest.launch_test_ros tests/integration/test_09_emergency_stop.py   # 시나리오 하나
python3 -m pytest tests/integration/test_09_emergency_stop.py -rs                   # pytest 경유
```

## 통합 테스트 실행

```bash
docker compose exec dev ./scripts/run_integration.sh              # 기본 세트 (장시간 14 제외)
docker compose exec dev ./scripts/run_integration.sh 4 9 13       # 번호 / id / slug 로 선택
docker compose exec dev ./scripts/run_integration.sh --list       # 시나리오 목록
docker compose exec dev ./scripts/run_integration.sh --unit-only --coverage   # 하네스 자체 단위 테스트
```

주요 옵션 (전체는 `--help`)

| 옵션 | 환경 변수 | 의미 |
| --- | --- | --- |
| `--sim auto\|kinematic\|gazebo` | `ITEST_SIM` | 시뮬레이터 백엔드. auto 는 시나리오 기본값 (01 gazebo, 02 gazebo→kinematic, 04·09·13 kinematic→gazebo) |
| `--profile auto\|component\|system` | `ITEST_PROFILE` | component: 하네스가 노드를 하나씩 조립 / system: `amr_bringup/system.launch.py` 전체 |
| `--standins auto\|always\|never` | `ITEST_STANDINS` | 대역 노드 정책. auto: 실제 노드가 설치돼 있으면 실제, 없으면 대역. never: 최종 통합 CI (대역 금지) |
| `--timeout-scale X` | `ITEST_TIMEOUT_SCALE` | 모든 대기 상한 배율. 공유 호스트 부하가 크면 2~4 |
| `--domain-id N\|env` | `ROS_DOMAIN_ID` | 기본은 실행마다 전용 도메인(100~199)이라 dev 스택과 섞이지 않는다 |
| `--fail-on-skip` | — | skip 도 실패로 셈 (모든 패키지가 머지된 뒤 CI) |

GPU: Gazebo 헤드리스 렌더링 센서(카메라·깊이)는 GPU 가 있어야 돈다. dev 컨테이너는
docker-compose.yml 의 nvidia 장치 예약으로, 단독 컨테이너는 `docker run --gpus all` 로 띄운다.
GPU·Gazebo 패키지가 없으면 Gazebo 시나리오는 **사유와 함께 skip** 되고 04·09·13 은 운동학
대역(kinematic_sim)으로 돈다.

종료 코드: 0 전부 통과 또는 skip / 1 실패·에러·시간 초과(또는 하네스 단위 테스트 실패) / 2 사용법·환경
오류 / 130 중단.

## 결과 형식 (CI 가 읽는 것)

```
logs/itest/
├── junit.xml            모든 시나리오 + 하네스 단위 테스트 JUnit 합본 (CI 리포터 입력)
├── summary.json         시나리오별 상태·skip 사유·판정 항목·측정값·소요 시간·호스트 부하
├── summary.md           사람이 읽는 표
├── unit/junit.xml       하네스 단위 테스트
└── <NN_slug>/
    ├── result.json      메타데이터(명세 지표·기준·로그 포맷) + 구성(real/standin) + 측정 + 판정
    ├── junit.xml        러너(launch_testing_ros) 결과 (시간 초과·크래시면 스크립트가 error 로 대신 씀)
    ├── launch.log       러너 전체 출력 (노드 로그 포함)
    └── *.csv            명세 4.10 로그 포맷 원시 측정 (시나리오별 — tests/integration/README.md)
```

`result.json` 의 `host.start/end.loadavg` 와 `provisional_under_load` 는 시간 지표(지연·주기)를
해석할 때 쓴다 — 외부 부하가 CPU 수의 절반을 넘으면 잠정치로 본다.

## 하네스 구조 (`tests/integration/amr_itest/`)

| 모듈 | 역할 | rclpy |
| --- | --- | --- |
| `catalog` | 시나리오 14개의 명세 조항·지표·합격 기준·로그 포맷 (README 표·`--list` 의 단일 출처) | — |
| `scenario` | `Context`: 결과 기록, 백엔드/구성 선택, 요구사항 확인 → `unittest.SkipTest` | — |
| `stack` | 시나리오별 launch 조립: 실제 노드가 있으면 실제, 없으면 대역 (`components` 에 기록) | launch |
| `probe` | 테스트 스레드용 rclpy 노드: 토픽 기록(raw CDR 헤더 포함), 주기, TF 그래프, 서비스 | ✔ |
| `actions` | 주행 명령·정지 대기·장애물 주입·웨이포인트 추종·액션 호출 | ✔ |
| `cases` | `ProbeCase`(판정 → result.json + assert, `ready_gate` = launch_testing_ros `WaitForTopics` 기동 게이트), `AfterShutdown`(종료 코드·마무리) | ✔ |
| `launch_test_ros` | 시나리오 러너 진입점 (`launch_testing_ros.LaunchTestRunner`, launch_test 와 같은 인자) | launch |
| `rates` `tf_tree` `kinematics` `metrics` `worldmap` `procmon` `cdr` `junit` `results` `report` | 순수 로직 (단위 테스트 대상) | — |
| `evaluation` | amr_evaluation 로거 CSV → `analyze` 판정 | — |
| `gz` | `ign service` 로 모델 자세 설정·생성·제거 (납치·도킹·경로 차단) | — |
| `standins/` | 아직 머지되지 않은 노드의 계약 대역 (kinematic_sim, wheel_odometry, imu_filter, amcl, safety_gate, cmd_relay, task_executor, static_tf, topic_relay) | ✔ |

측정 원칙

- **명세 지표는 두 경로로 잰다**: amr_evaluation 로거(pose_error / response_time / cte /
  cpu)가 명세 표준 절차로 CSV 를 쓰고 `analyze` 가 판정하며, 하네스가 같은 지표를 자기 기록으로
  독립 계산(`metrics.py`)한다. 둘 다 합격이어야 통과이고, amr_evaluation 이 없으면 하네스 계산만으로
  판정한다.
- **주기는 header.stamp 로** 잰다 (sim time). RTF < 1 인 고부하 호스트에서도 센서가 sim time 규정
  주기로 발행하는지가 기준이다. wall 주기와 RTF 는 참고로 남긴다.
- **지연은 wall clock 으로** 잰다 (같은 호스트의 모든 노드가 공유하는 시계).

## 시나리오 추가

1. `amr_itest/catalog.py` 에 `Scenario(number, slug, title, spec, metric, threshold, log_format,
   backends, profiles, implemented)` 한 항목.
2. `tests/integration/test_NN_<slug>.py` 한 파일: `CTX = Context(catalog.get(NN))`,
   `generate_test_description()` 에서 `CTX.require(...)` → `Stack` 조립, `cases.ProbeCase` 상속
   테스트, `cases.AfterShutdown` 상속 post-shutdown 클래스.
3. 필요한 순수 계산은 `amr_itest/` 모듈 + `unit/` 테스트로.

필요한 노드가 아직 없으면 `CTX.require()` 가 무엇이 없는지 정확한 사유로 skip 하고, 패키지가
머지되면 같은 파일이 그대로 켜진다.

## 확장 지점: 연구 브리프의 통합 테스트 제안

영역별 연구 브리프(state-estimation §7, local-planning §7.5, task-execution-docking §5.2,
fleet-traffic-deadlock)가 제안한 IT 항목은 아래처럼 기존 시나리오에 붙이거나, 위 "시나리오 추가"
절차로 15번 이후 시나리오로 더한다. 하네스 쪽 준비물(대역·지표·로그)은 이미 있다.

| 브리프 제안 | 붙일 곳 | 필요한 것 |
| --- | --- | --- |
| TF 트리 / 정지·직선·회전 오차 (state-estimation IT-01~05) | 02, 04 (`PROGRAM` 에 1·2 m/s 구간 추가) | 실제 AMCL: `--profile system` |
| 납치 복구·오수렴 0 (state-estimation IT-06) | 05 | amr_localization kidnap 감시 |
| 센서 중단 → 감속·정지 (local-planning IT-05·06) | 09 에 `test_40_sensor_timeout` 추가 (kinematic_sim 에 스캔 발행 중단 입력을 더해야 한다 — safety_gate 대역은 이미 `sensor_timeouts.lidar` 로 정지) | safety_node `sensor_timeouts` |
| 대시보드 E-stop ≤ 20 ms (local-planning IT-07) | 09 `test_20_button_estop` (기준값만 강화) | — |
| 존 1→2→3 안전 사다리 (local-planning IT-08) | 09 `test_30_wall_approach` 의 `safety/zone` 기록 | safety_node |
| 도킹 중 E-stop·마커 가림·재할당 (task-execution-docking IT-3·4·7) | 10, 11 | amr_behavior docking_server_node |
| 교착 탐지·해소, 5대 CPU (fleet-traffic-deadlock, local-planning IT-10·11) | 12 | amr_fleet traffic manager |
| 4 h 소크 RSS·goal handle (task-execution-docking) | 14 (`procmon` 이 RSS 기록) | 전체 system 스택 |
