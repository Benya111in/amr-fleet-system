# 통합 테스트 리포트

> 명세 4.10 "통합 테스트 시나리오 최소 10개 작성·자동화", "성능 지표 측정 절차 표준화". 시나리오 정의·판정 원칙의
> 단일 출처는 [tests/integration/README.md](../../tests/integration/README.md) 와 `tests/integration/amr_itest/catalog.py`,
> 하네스 구조는 [tests/README.md](../../tests/README.md) — 실행 옵션·종료 코드·skip 정책은 이 문서 1장이 최신이다
> (tests/README.md 의 같은 표 갱신은 소유 경로 밖이라 별도 요청으로 넘겼다).

## 1. 실행 방법

빌드된 워크스페이스(`./scripts/build.sh`)에서 GPU 컨테이너로 돌린다. Gazebo 헤드리스 센서 렌더링에 GPU,
bringup 의 Fast DDS SHM 프로파일에 `/dev/shm` ≥ 2 GB 가 필요하다.

```bash
# dev 컨테이너 (docker-compose.yml: nvidia 장치 예약, ipc: host)
docker compose exec dev ./scripts/run_integration.sh              # 기본 세트 01~13 (+ 단위 테스트)
docker compose exec dev ./scripts/run_integration.sh 4 9 13       # 선택
docker compose exec dev ./scripts/run_integration.sh --soak-hours 4 14   # 4 h 연속 운용 (러너 상한 4.5 h)

# 일회용 컨테이너
docker run --rm --gpus all --shm-size=8g -v "$PWD:/ros2_ws" amr-fleet-system:wf-final \
  bash -lc 'cd /ros2_ws && ./scripts/run_integration.sh --domain-id 42 1 4 9'

# GPU 없는 CI: 운동학 구성만 (04·13 은 AMCL·실행기 대역 — 체인 회귀이지 명세 판정이 아니다)
./scripts/run_integration.sh --coverage --sim kinematic --profile component 2 4 9 13
```

| 옵션 | 환경 변수 | 의미 |
| --- | --- | --- |
| `--sim auto\|kinematic\|gazebo` | `ITEST_SIM` | 백엔드. auto 는 구성이 허용하는 시나리오 백엔드 중 앞에서부터 (system 구성은 gazebo 만) |
| `--profile auto\|component\|system` | `ITEST_PROFILE` | auto 는 시나리오의 첫 구성 (04·13 은 명세 판정인 system) |
| `--standins auto\|always\|never` | `ITEST_STANDINS` | never: 대역 금지 (운동학 시뮬레이터·AMCL·실행기 대역 포함 — 해당 구성은 skip = 실패) |
| `--domain-id N\|auto\|env` | `ROS_DOMAIN_ID` | 기본 auto = 실행마다 1~101 난수 (호출 셸 값과 다르게). 병렬 CI 는 N 명시 |
| `--ign-partition NAME\|auto\|env` | `IGN_PARTITION` | 기본 auto = `ITEST_IGN_PARTITION` 또는 `itest_<난수>` (바꾸면 출력) |
| `--timeout-scale X` | `ITEST_TIMEOUT_SCALE` | 대기 상한 배율 (공유 호스트 2~4) |
| `--scenario-timeout S` | — | 시나리오 하나의 wall 상한 (기본 1800 s, 14 는 운용 시간 + 30 분). 시나리오에 `ITEST_SCENARIO_TIMEOUT` 으로 전달돼 남은 시간(`time_left`)이 모자라면 시행을 줄이지 않고 실패로 끝낸다 |
| `--soak-hours H` | `ITEST_SOAK_HOURS` | 14 운용 시간 (기본 4.0), 러너 상한 H + 30 분 |
| `--fail-on-skip` / `--allow-skip` | — | 기본: needs 패키지가 설치된 시나리오의 skip·일부 skip 은 실패. fail-on-skip = 모든 skip 실패, allow-skip = 하네스 개발용 |

종료 코드: 0 전부 통과 / 1 실패·에러·시간 초과·실패로 세는 skip·하네스 단위 테스트 실패 / 2 사용법·환경 오류 / 130 중단.

명세 캠페인 시행 수는 기본값이다. 줄여서 스모크할 때는 `ITEST_TRIALS`(08, 30) · `ITEST_RT_SAMPLES`(13, 50) ·
`ITEST_MR_TASKS`(12, 5) · `ITEST_MR_ROBOTS`(12, 5) · `--soak-hours`(14, 4.0) 를 준다 — 줄이면 해당 판정(시행 수 ·
로봇 수 · 기간)이 실패로 남는다 (통과로 보이지 않게).
공유 호스트가 붐비면 `--timeout-scale 2`(대기 상한 배율)를 준다.

결과: `logs/itest/<NN_slug>/{result.json, junit.xml, launch.log, *.csv}`, 합본 `logs/itest/{junit.xml, summary.json,
summary.md}`. 종료 코드 0 = 전부 통과. **skip 은 통과가 아니다**: 시나리오가 쓰는 패키지(catalog `needs`)가 모두
설치돼 있는데 skip(전부·일부)이면 실패로 센다 (GPU 없음·구성 불가·`--standins never` 포함). 종료 요약에 실행·skip 수를 찍는다.

## 2. 시나리오가 판정하는 것

| # | 시나리오 | 구성 | 판정 (요약) | 표준 로그 |
| --- | --- | --- | --- | --- |
| 1 | 센서 토픽 | Gazebo | 센서 주기 = 스탬프 간격 중앙값 ≥ 0.98 × max(명세, 설정) (LiDAR 10 · IMU 100 · RGB 30 · depth 15 Hz), 규격, LiDAR σ, depth ≤ 10 m · 화소 노이즈 σ | `topic_rates.csv` |
| 2 | TF 트리 | 운동학/Gazebo | 계약 간선 전부, 이중 부모·순환 0, 정적 변환 ≤ 1 mm / 0.1°, tf2 조회 | `tf_edges.csv`, `frames.dot` |
| 3 | SLAM | system (slam) | 해상도 ≤ 0.05 m, 월드 visual 단면 대비 재현율 ≥ 0.9 · 오점유 ≤ 5 %, map↔월드 잔여 정합, save_map | `map_info.json`, `map.pgm/yaml` |
| 4 | 위치 추정 RMSE | system (실제 AMCL) | 정지/직선/회전 RMSE ≤ 3/5/8 cm, 구간별 ≥ 100 샘플, map↔월드 정합 | `pose_error.csv` [timestamp, gt_x, gt_y, est_x, est_y, error, …] |
| 5 | Kidnapped Robot | system | 감지 ≤ 5 s, 복구(오차 ≤ 0.1 m 3 s 유지) ≤ 60 s, 5/5 | `kidnap.csv` |
| 6 | 경로 계획 | system | 직접 구현 A*(planner_id=AStar, 플러그인 확인) 50쌍 ≥ 98 %, 계획 시간 최대 ≤ 500 ms (NavFn·Smac 비교 열) | `plans.csv` |
| 7 | 경로 추종 CTE | system | 직선 ≤ 5 cm, 곡선 ≤ 10 cm (amr_evaluation cte_logger, 두 구간 행 모두 필요), 목표 전부 도달 | `cte.csv` [timestamp, planned_x/y, actual_x/y, cte, segment] |
| 8 | 동적 장애물 | system | 모든 동적 장애물(사람·지게차·셔틀) 접촉 0 / 30회 (collision_monitor), 이탈 ≤ 1 m, 복귀 ≤ 5 s, 도달 ≥ 29, 조우 ≥ 5. 접촉마다 그 순간 GT 로봇 속도를 남긴다 (움직이며 부딪침 / 멈춘 로봇에 닿음 — 귀속 근거, 판정 불변) | `avoidance.csv`, `contacts.csv` |
| 9 | 긴급 정지 | 운동학/Gazebo | 최댓값: 스캔→정지 ≤ 20 ms, 주입→정지 ≤ 스캔 주기 + 20 ms, 버튼→정지 ≤ 20 ms; 벽 앞 여유 ≥ 0.25 m; 버튼 래치는 reset 성공 전까지 유지 | `estop_latency.csv`, `approach.csv` |
| 10 | 정밀 도킹 | system | GT 최종 자세 vs 월드 마커 + standoff 기대 자세: 2 cm / 1° × 10/10; 마커 가림 → 재시도 3회 소진 | `docking.csv` |
| 11 | BT 에러 복구 | system | 경로 차단·위치 상실·인식 실패 각각 60 s 안에 복구 증거 + 작업 COMPLETED. 앞 사례가 lost 로 끝나면 사례 사이에 대기 자세 + initialpose 로 초기화 (판정 창 밖, `reset_before` 열) | `recovery.csv` |
| 12 | 5대 교착 | multi_robot | 작업 전부 완료, 탐지된 교착 전부 해소, CPU(ROS 노드 + Gazebo)/호스트 코어 ≤ 80 %; 강제 교착(교차로 안 E-stop 로봇 + 실행기 작업 로봇의 토큰 대기) → BLOCKED 탐지·해소 | `traffic.csv`, `cpu.csv`, `tasks.csv`, `forced.csv` |
| 13 | 응답 시간 | system (실제 실행기 체인) | 작업 명령 → GT 첫 움직임(\|v\| ≥ 0.01 m/s ∨ \|ω\| ≥ 0.02 rad/s) 평균 ≤ 200 ms, 50회, 최대 기록 (0.05 m/s 변형 병기) | `response_time.csv` [cmd_time, response_time, latency_ms] |
| 14 | 연속 운용 | system | 크래시 0, 누수 없음 (워밍업 10 분 뒤 RSS 기울기 > 5 MB/h 이고 적합 증가량 > 2 MB 면 누수, 표본 < 5 는 판정 불가 = 실패), 실패율(시간 초과 포함) ≤ 2 %, 완료 ≥ 4 /h × 기간 | `memory.csv`, `tasks.csv`, `soak.json` |

공통: system 구성은 스폰 자세·지도(`maps/warehouse.yaml`)를 명시하고, 판정 전에 lifecycle 활성과 map↔월드 항등 정합
(잔여 이동 ≤ 5 cm, 회전 ≤ 0.1°)을 확인한다. 측정 중 크래시는 실패, launch 종료 단계에서만 난 비정상 종료는 경고로 남긴다.
커버리지(명세 4.10 표): `run_integration.sh --unit-only --coverage` → 모듈별 term-missing + HTML (`logs/itest/unit/htmlcov`).

## 3. 스모크 결과

조건 (2026-09-22 23:32 ~ 09-23 02:34 KST, 호스트 uptime 52 일 9:44 ~ 12:47):

- 이미지 `amr-fleet-system:wf-final`, 워크트리 HEAD `42a2631` + 이 하네스 변경 (제품 패키지 `src/` 는 손대지 않았다 —
  `src/amr_navigation` 은 다른 작업이 고치는 중이라 이 스모크의 내비게이션 원인 실패는 그대로 기록했다).
- 일회용 컨테이너 `docker run --rm --gpus all --shm-size=8g` (이름 `itest_<n>`, `ROS_DOMAIN_ID` 81~88,
  `IGN_PARTITION=itest_<n>`, `ROS_LOCALHOST_ONLY=1`), 러너 `--no-unit --domain-id 8<n>`, 긴 시나리오는
  `--timeout-scale 2`.
- 호스트 32 스레드를 다른 작업(다른 Gazebo 컨테이너)과 공유: 실행 중 loadavg 5 ~ 163 (12 는 5대 스택 자체가
  loadavg 100 을 넘긴다). 시간 지표는 전부
  `provisional_under_load` (loadavg > 16) 조건에서 쟀다. 각 시나리오 `result.json` 의 `host` 에 시작·끝 부하가 있다.
- 시행 수를 줄인 것: 13 `ITEST_RT_SAMPLES=6` (명세 50), 14 `--soak-hours 0.25` (명세 4), 12 의 축소 구성
  `ITEST_MR_ROBOTS=2 ITEST_MR_TASKS=1` — 줄인 만큼 해당 판정(샘플 수 · 로봇 수)이 실패로 남는다.
  08(30회)·05(5회)·06(50쌍)·10(10회)·12 명세 구성(5대·5건)은 명세 시행 수.
- 한 번 돈 뒤 하네스를 고친 시나리오(09 운동학, 11, 12, 14, 08)는 고친 판으로 다시 돌렸다 — 표는 마지막 실행,
  앞 실행은 비고에 적었다. 마지막 실행 뒤에 바뀐 하네스 코드: 11 의 사례 사이 초기화가 `actions.seed_pose`
  (initialpose + map EKF set_pose + AMCL 무이동 갱신)를 쓰게 한 것 — 마지막 11 실행에서는 초기화가 필요 없어
  (위치 상실 사례가 스스로 복구) 이 경로가 돌지 않았다.

| # | 구성 / 백엔드 | 결과 | 주요 수치 | 시작 KST · loadavg 시작→끝 | 실패 귀속 |
| --- | --- | --- | --- | --- | --- |
| 01 | component / gazebo | 통과 | 스탬프 주기 LiDAR 10.0 · IMU 100.0 · RGB 30.3 · depth 15.15 · GT 50 Hz, LiDAR σ 0.0297 m, depth 최대 9.93 m · σ 4.9 mm, RTF 0.72 | 23:32 · 39.5→38.4 | — |
| 02 | component / kinematic · gazebo | 통과 · 통과 | 계약 간선 전부, 이중 부모·순환 0, 뿌리 map, tf2 조회 전부 | 23:32 · 40.4 / 00:09 · 47.1→62.6 | — |
| 03 | system(slam) / gazebo | 실패 | 해상도 0.05, 재현율 0.929 (통과), 오점유 5.92 % (> 5 %), map↔월드 잔여 8.4 cm / −0.154° (> 5 cm / 0.1°), 경로 10/10, save_map 실패 | 00:09 · 46.6→119.5 | SLAM(amr_localization): 오점유가 서쪽 벽 띠 x −30~−29, y −20~−10 에 몰림 = 지도 8 cm 표류; save_map 서비스 이름공간 |
| 04 | system / gazebo | 통과 | RMSE 정지 1.54 (n 821) · 직선 1.37 (n 417) · 회전 1.54 cm (n 617), amr_evaluation 1.48 · 1.37 · 1.54 cm, 정합 1.7 cm / −0.031° | 23:32 · 39.8→45.4 | — |
| 04 | component / kinematic (AMCL 대역) | 통과 | EKF 배관 RMSE 2.13 · 1.78 · 1.93 cm (명세 판정 아님) | 23:32 · 39.1→46.6 | — |
| 05 | system / gazebo | 통과 | 5/5, 감지 3.20~3.48 s, 복구 4.26~4.68 s, 초기 수렴 1.46 cm | 23:41 · 69.5→79.9 | — |
| 06 | system / gazebo | 통과 | AStar(amr_navigation::AStarPlanner) 50/50, 계획 평균 3.0 · 최대 13 ms (참고 NavFn 50/50 최대 20 ms, Smac 45/50 최대 38 ms) | 23:35 · 45.5→54.6 | — |
| 07 | system / gazebo | 통과 | CTE 평균 직선 0.97 cm (n 1244), 곡선 2.45 cm (n 243), 목표 5/5 | 23:40 · 56.8→76.8 | — |
| 08 | system / gazebo | 실패 | 30회, 조우 30, 도달 28 (< 29), 접촉 27건 (13 시행; worker_crossing 25 · worker_random 2) — **접촉 순간 GT 로봇 속도가 전부 ≤ 0.02 m/s** (움직이며 부딪친 접촉 0, contacts.csv), 최대 이탈 6.45 m, 최대 복귀 30.0 s | 00:58 · 5.4→5.4 (중간에 11·12 와 겹쳐 최고 ~160) | 로봇은 사람 앞에서 멈추고 스크립트 actor 가 멈춘 로봇으로 걸어 들어온다 — 횡단선 위에서 멈추는 회피 전략(amr_navigation) 또는 로봇을 피하지 않는 actor(월드) 중 어느 쪽을 고칠지는 열린 질문. 이탈 · 복귀는 다른 통로로 도는 재계획 (amr_navigation). 앞 실행(23:45, loadavg 78): 접촉 22 · 이탈 5.95 m · 복귀 33.4 s · 도달 30 |
| 09 | component / kinematic | 통과 | 최댓값: 스캔→정지 2.16 ms, 주입→정지 103.3 ms (≤ 120), 버튼 1.79 · /fleet/estop 2.51 ms, 래치 의미 전부, 정지 여유 0.315 m | 23:36 · 49.0→66.4 | 첫 실행(23:32)은 하네스 결함(깊이 없는 스택에 depth_cloud 켬 → 0.2 m/s 상한)으로 실패 — 고침 |
| 09 | component / gazebo | 실패 | 근접 정지: trial 2 "장애물이 남았는데 주행 명령 재개"; 버튼 1.57 · 2.02 ms, 래치 전부, 정지 여유 0.310 m 는 통과 | 23:36 · 68.1→72.5 | safety_node (amr_perception): 깊이 오래된 채 STOP 해제 |
| 10 | system / gazebo | 실패 | 9/10 이 2 cm / 1° 안 (GT 최대 5.8 mm / 0.60°), trial 0 dock_1 3회 실패 (GT 11.0 cm / 30.5°); 마커 가림 → success false · 3회 소진 통과 | 23:45 · 70.5→81.0 | docking_server (amr_behavior): 마커 id 나이 게이트 |
| 11 | system / gazebo | 실패 (2/3) | 위치 상실 복구 34.5 s + COMPLETED, 인식 실패 복구 12.9 s + COMPLETED, 경로 차단: 복구 증거 없음 · 작업 미종료 | 00:37 · 74.8→163.0 | 6 m 차단벽에 kidnap_monitor 가 LOST 오판 → 잘못된 전역 시드(−28.4, −19.4) → 실행기가 MOVING 에 멈춤 (amr_localization · amr_behavior). 앞 실행(00:09)은 lost 가 옆 통로로 오수렴한 뒤 나머지 2종이 실행되지 못함 → 사례 사이 초기화 추가 |
| 12 | multi_robot / gazebo (5대, 명세 구성) | 실패 | 5대 중 1 ~ 3 대의 `lifecycle_manager_navigation`(한 번은 `_localization` 도) 이 `get_state service client: async_send_request failed` → `Aborting bringup` 으로 기동 실패 → 작업·CPU·강제 교착 판정 전에 중단 (세 번 모두, 조용한 호스트에서도). cpu_sampler: ROS 프로세스 79.0 % · Gazebo 3.7 % · 호스트 90.4 % (32 스레드), nav 30.3 %, RTF 0.18 | 02:03 · 5.0→108.5 | Nav2 lifecycle 기동 (amr_bringup · amr_navigation). 5대 스택만으로 호스트가 포화 (명세 4.10 CPU ≤ 80 % 도 위태) |
| 12 | multi_robot / gazebo (2대, `ITEST_MR_ROBOTS=2` · `ITEST_MR_TASKS=1` 축소 — '로봇 수' 판정은 실패로 남는다) | 실패 (축소 구성) | lifecycle 전부 활성, 작업 1/1 COMPLETED, CPU 41.7 % (코어 13.4 / 32), 강제 교착: **탐지 1건** (`traffic/DEADLOCK BLOCKED: amr_01 ↔ amr_02`, E-stop 보유자 배치 5.0 s 뒤) → 해소 실패 `traffic/UNRESOLVED exhausted(no_alt_path)` | 02:28 · 84.9→18.6 | 탐지는 기대대로. 해소가 실제 창고 배치에서 대체 경로를 찾지 못한다 (amr_fleet traffic_resolution — 같은 배치의 패키지 회귀 시험은 합성 시뮬레이터에서 RESOLVED) |
| 13 | system / gazebo | 실패 | 6 샘플(축소): 벽시계 평균 271.1 · 최대 349.0 ms (> 200), 스탬프 평균 124.2 · 최대 131 ms, 0.05 변형 247.5 ms, cmd_vel≠0 90.0 ms, RTF 0.33~0.63, 작업 6/6 완료 | 23:50 · 74.9→60.6 | 샘플 수(축소), 부하로 RTF 0.49 — 스탬프 기준은 124 ms |
| 13 | component / kinematic (실행기 대역) | 통과 | 50 샘플 벽시계 평균 128.7 ms (amr_evaluation 127.7) — 체인 회귀, 명세 판정 아님 | 23:33 · 46.7→50.8 | — |
| 14 | system / gazebo (0.25 h) | 실패 | 크래시 0, 실패율 0, 완료 2 (≥ 1), 누수: planner_server 151 MB/h (워밍업 뒤 4 분 적합 증가 10.1 MB, 전 구간 87→125 MB 단조 증가; 앞 실행 120 MB/h 재현) | 00:37 · 74.8→24.6 | planner_server RSS 증가 (amr_navigation). 앞 실행의 gz_image_bridge 18 MB/h 는 1.5 MB 요동 외삽 → 요동 바닥 추가 |

하네스 단위 테스트: `run_integration.sh --unit-only --coverage` → 83 통과, 하네스 모듈 커버리지 97 %
(amr_itest 3255 문장, 모듈별 71 ~ 100 %; ament_flake8 · ament_pep257 문제 0).

실패 귀속 요약 (하네스가 아닌 제품 쪽 — 이 작업의 소유 경로 밖이라 고치지 않고 넘긴다):

- 내비게이션 (amr_navigation): 08 회피 — 횡단선 위 정지·큰 우회(이탈 6.45 m, 복귀 30 s), 14 planner_server RSS
  증가(120 ~ 151 MB/h, 두 번 재현).
- 기동 (amr_bringup / Nav2 lifecycle): 12 에서 5대 동시 기동 때 1 ~ 3 대의 lifecycle_manager_navigation 이
  `get_state service client: async_send_request failed` → `Aborting bringup` (세 번 모두, 조용한 호스트에서도).
  2대 구성에서는 같은 스택이 전부 활성 — 동시 기동 수·자원 문제로 보인다.
- 교통 관리 (amr_fleet): 강제 교착은 5.0 s 에 BLOCKED 로 탐지되지만 해소가 `exhausted(no_alt_path)` 로 끝난다.
- 위치 추정 (amr_localization): 11 큰 미지 장애물에 LOST 오판 + 잘못된 전역 시드, 앞 실행의 옆 통로 오수렴;
  03 SLAM 표류(8 cm)·오점유 5.9 %·save_map 서비스 이름.
- 인식 (amr_perception): 09 Gazebo 근접 정지 해제.
- 행동 (amr_behavior): 10 docking_server 마커 id 나이 게이트 (dock_1 3회 실패), 11 LOST 뒤 실행기가 MOVING 에서
  끝나지 않음 (작업 시간 상한 없음).
- 부하: 13 system 은 RTF 0.33 ~ 0.63 에서 잰 값 — 스탬프 기준 124 ms 는 200 ms 안, 판정 기준(벽시계)은 271 ms.

열린 질문 (판정 기준을 정해야 하는 것들):

- 13 을 벽시계로 판정하면 RTF < 1 인 호스트에서는 시뮬레이터 느림이 지연에 섞인다. 명세 캠페인은 RTF ≥ 0.9 를
  전제로 돌릴지, 스탬프 기준을 함께 문턱으로 둘지. 50 샘플은 작업 하나당 ~2 분(sim)이라 RTF 0.5 에서 100 분 넘는다.
- 08 의 접촉이 전부 '멈춘 로봇에 스크립트 actor 가 걸어 들어온 것'이다. 판정에서 로봇 정지 접촉을 빼야 하는지,
  월드 actor 가 로봇을 피해야 하는지, 회피가 횡단선 위에서 멈추지 말아야 하는지.
- 11 의 경로 차단 주입(통로를 가로지르는 6 m 벽)은 위치 추정 LOST 오판까지 일으킨다 — 그대로 둘지(스트레스),
  통로를 막되 스캔을 덜 가리는 크기로 줄일지.
- 12 의 명세 구성(5대)은 이 호스트에서 기동 자체가 안 된다. 기동 수정 전까지는 축소 구성(로봇 수 판정 실패)만
  나머지를 확인할 수 있다.

## 4. 명세 7장과의 차이

하네스(`tests/integration/amr_itest`, 대역 노드 포함)는 `src/` 밖에 있다 — 제품 패키지가 아니라 빌드된 워크스페이스를
밖에서 시험하는 도구이기 때문이다 (colcon 패키지로 만들면 `colcon test` 가 GPU·Gazebo 시나리오를 끌어들이고, 제품
패키지가 시험 대역에 의존할 수 있게 된다). 하네스 자체의 단위 시험·커버리지는 `run_integration.sh --unit-only --coverage`
와 CI 가 돌린다.
