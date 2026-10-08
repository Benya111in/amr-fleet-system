# 보너스 2 시나리오 녹화 — bag 목록·검사 결과·재생 방법 (b2-rec, R·G 단계)

작성 2026-10-08, 브랜치 `feature/b2-rec` (분기 `feature/bonus2-slam3d` 3409599). 완료 정의 `bonus2_completion_plan.md` §4 의
**R 녹화 준비**와 **G 녹화**. 사전 등록·결과·해석의 원본은 실험 노트
[`notebook/entries/2026-10-08_1030_b2-rec_record-gate-scenarios.md`](notebook/entries/2026-10-08_1030_b2-rec_record-gate-scenarios.md),
실행 목록 `notebook/runs/b2-rec.jsonl`, 원자료 `/home/CAPYI_AI/runs/b24/b2-rec/` (저장소 밖).
표기: **[측정]** 명령·로그 있음, **[계산]** 오프라인 스크립트, **[코드]** 소스로 정해진 값, **미측정**.

> 이 문서는 P 단계(재생·평가)가 바로 쓰게 하려는 것이다. 재생·평가 결과는 여기 없다 (스모크 한 행만).

## 0. 먼저 알아야 할 불리한 사실

(녹화 뒤 채운다)

## 1. 녹화 전에 고친 것 (R 단계, `replay.md` §9 의 1–5 + 도구 결함)

| replay.md §9 | 고친 것 | 코드 | 시험 |
|---|---|---|---|
| 1 원시 IMU 만 | `rows.RECORD_TOPICS` 에서 `imu/data_madgwick` 를 뺐다. madgwick 은 녹화 뒤 `madg_lockstep.py` 로 파생 bag(`bag_madg`)에 넣는다 — 기록된 madgwick 이 없는 bag 에서는 파생 토픽 QoS 를 `imu/data` 의 것으로 | `rows.py`, `madg_lockstep.py`, `record_bag.sh` | `test_config.py::test_record_topics_cover_report_list` (옛 행 넷만 `PILOT_ONLY_TOPICS` 를 튼다) |
| 2 기록 직후 공백 검사 | `recgate.py` + `bag_tools.py gate`: `/clock`(1 ms)·EKF `/tf` odom→base_footprint(20 ms)·점군 둘(100 ms)·IMU 둘(5 ms)의 sim 스탬프 이웃 간격 > 1.5 주기 + 1 µs (정수 ns) 면 실패, 반복·역행·덮음 부족도 실패. 실패 bag 은 쓰지 않고 다시 기록 (상한 3 회) | `recgate.py`, `bag_tools.py`, `rec_driver.sh` | `test_recgate.py` 15 개 |
| 3 출발 전 ≥ 10 s 정지 후 기록 | `route_driver` 를 기록 **전에** 띄워 sim 10 s 동안 0 명령 → ready 파일 → 기록기 → 14 토픽 구독 확인 → go 파일 → `hold_start_s` 10 s 정지 → 출발. 기록 중에는 새 프로세스를 띄우지 않는다 (파일 검사만) | `route.py`(wait_go·arm), `route_driver.py`(ready/go/done/quit), `record_bag.sh` | `test_route.py` (arm 전 0 명령, arm 뒤 hold) |
| 4 재생 시작을 구독 확인 뒤로 | `replay_row.sh`: `ros2 bag play -p` → `rows.ready_requirements(행)` 의 (토픽, 구독 노드)가 `ros2 topic info -v` 에 모두 보이고 재생기가 입력 토픽 발행자로 보이면 → 2 s → `/rosbag2_player/resume`. 상한 300 s 넘으면 재생 안 함(종료 코드 4) | `rows.py`, `replay_row.sh` | `test_config.py` 구독 확인 3 개 (Humble 실제 출력 형식) |
| 5 끝 ≥ 0.5 s 정지 | `hold_end_s` 3 s, 기록기를 먼저 멈추고 주행기를 나중에 끝낸다 (quit 파일). 공백 검사가 정답 자세로 처음 ≥ 10 s·끝 ≥ 0.5 s 정지를 확인 | `route.py`, `record_bag.sh`, `recgate.py` | `test_recgate.py::test_protocol_still_rules` |
| (evaluation.md §10) N6 | `cli traj --align se2` 연관 표본 < 2 → 멈추지 않고 `failed=1`·NaN 행 | `slam3d/cli.py`, `se2.py` | `test_slam3d_review_fixes.py` N6 3 개 |
| (evaluation.md §10) N11 | `observed_mask` — 1 차원 색인 바깥 곱·행 묶음 조회, 거리변환 대신 행 묶음 원판 팽창. 셀당 최대 추가 메모리 **84.1 B → 4.1 B** [측정, 2400 x 3600 격자, tracemalloc], 결과는 예전 구현과 같다 | `slam3d/gridmap.py` | N11 24 개 (예전 구현 대조 16, 원판 팽창 = 거리변환 문턱 6, 빈 마스크, 메모리) |
| (evaluation.md §10) N4 | queue 정지점 감도 — 아래 §2.3 | `slam3d/localizability.py` (`queue@Y`) | N4 1 개 |

변이 검사 [측정, `B2rec-test-mut`, `/home/CAPYI_AI/runs/b24/b2-rec/mut/`]: 가드 25 개를 하나씩 되돌림 (recgate 12, route 5, rows 4,
gridmap 3, cli 1) → **25/25 잡힘**. 1 차(8f5f340)는 24/25 — 역행 시험 사례가 간격 초과로도 실패해 역행 조건을 따로 시험하지 못했다
(`gate_back_ignored` 생존) → 시험을 고쳐(a1426ee) 다시 돌렸다.
시험 [측정]: `colcon test` amr_slam3d·amr_evaluation 521 개 (ament 린트 포함), 실패 0, 건너뜀 8 (evo 교차 확인 — 이미지에 evo 없음).

구독 확인 시험 [측정, `B2rec-ready-probe-pilot`]: 파일럿 bag(`rtabmap3d_noimu`·`slam_toolbox`)과 b2-offline 파생 bag(`liosam_planar`) 에서
구독 확인이 1–2 s 에 끝났고, `ready_*.txt` 에 실제 구독 노드(icp_odometry·rtabmap·slam_probe·rosbag2_recorder /
lio_sam_imageProjection·lio_sam_imu_preintegration)가 보였다. 재생은 8 s 만 했다 (처리 수 미측정 — 녹화 bag 스모크가 따로 있다).

## 2. 시나리오 (사전 등록 — `config/rec_plan.txt`, 경로 `config/routes/*.yaml`)

### 2.1 표

| id | 월드 (`gen_warehouse_world.py --variant`) | 스폰 | 경로 | 길이 [계산] | 목적 |
|---|---|---|---|---|---|
| `s3_aisle` | s1 (동적 없음) | (18, −16, π/2) | (18, −5.5) → (0, −5.5) → 중앙 (0, −10) **15 s 정지** → (0, −14.5) 2 s → (0, −5.5) | 46.5 m | S3 좁은 통로 왕복 — 통로 축·횡 분해 |
| `s1_seed1` | s1 | 같음 | 경로 씨드 1 | 61.0 m | S1 정상 |
| `s1_seed2` | s1 | 같음 | 경로 씨드 2 | 97.0 m | S1 |
| `s1_seed3` | s1 | 같음 | 경로 씨드 3 | 80.0 m | S1 |
| `s6_open` | s6-open (블로커 없음, actor 4·차량 2) | **(0, −8.6, −π/2) 통로 안** | 중앙 (0, −10) **15 s** → (0, −11.4) 2 s → **후진** (0, −8.6) 2 s, **sim 60 s 출발** | 5.6 m | S6 기준 (요인 E 열림) |
| `s6_queue` | s6-queue (북쪽 대기 AMR (0, −5.5)) | 같음 | 같음 | 5.6 m | S6 현실 대기 |
| `s6_wall` | s6-wall (양끝 입구 화물 (0, −7.75)·(0, −12.25)) | 같음 | 같음 | 5.6 m | S6 최악 |
| `map_long` | s1 | (18, −16, π/2) | 차선 지그재그 y −15 → −6 → 0 → 6 → 13.3 (동·서 끝 ±21.5), 서쪽 끝 차선·남쪽 홀 되짚어 귀환 | 308.6 m | 긴 매핑 |

공통: v_max 0.5 m/s, w_max 0.5 rad/s, 회전 지점 정지 1 s, 정답 되먹임 20 Hz(sim). 출발 전 bag 안 정지 ≥ 10 s, 끝 3 s.

- **S1 의 "씨드"** = 경로 생성 난수 씨드 (`routegen.random_walk`): 차선 그래프(가로 y 13.3·6·0·−6·−15, 세로 x ±21.5·랙 틈
  ±15·±9·±3) 위를 직전 마디로 되돌아가지 않고 걷다가 60 m 를 넘으면 출발 마디 (18, −15)로 최단 경로로 돌아온다 (루프 폐쇄
  기회). 정적 월드라 evaluation.md §6 의 "출발 지연 씨드"는 장면을 바꾸지 못해 쓰지 않았다.
- **S6 경로가 통로 안인 이유**: s6-wall 은 양끝을 막아 밖에서 들어갈 수 없다. 세 변형의 짝 비교를 위해 같은 경로를 쓰므로 모두
  통로 안에서 시작한다. 순폭 0.60 m < 외접 지름 0.72 m 라 통로 안에서 돌 수 없어 돌아올 때는 후진한다. 출발 sim 시각을 60 s 로
  고정해 세 변형에서 actor·차량과의 상대 시각이 같다 (공백 검사가 [60, 60.5] s 를 확인).
- **긴 매핑 경로**는 slam.md §6.1 의 322 m 경로와 같은 차선이지만 원 웨이포인트 파일이 저장소에 없어 다시 만든 것이다 (308.6 m).
- 장애물 여유 [계산, `make_routes.py check`, GT 대역 0.05–2.0 m 래스터 0.05 m, `explore/clearance_v1.json`]: 통로 밖 선분 중심선
  ≥ 0.66 m, 회전 지점 ≥ 0.46 m, 통로 안 ≥ 0.25 m 를 6 경로 모두 만족.

### 2.2 S6 월드의 동적 요소

s6-* 는 actor 4 명(`worker_random`·`worker_slow_south` 를 뺀 것)과 차량 2 대(지게차 북쪽 차선 y 13.8–16.2, 셔틀 x 8.25–9.75)를 둔다.
`worker_crossing` 이 통로 상자(x ±0.5, y −12.4…−7.6)에서 0.60 m 까지 온다 [계산, `build_sdf('s6-open')`]. 블로커는 정답 GT 에
들어 있지 않다 (사전 지도에 없는 물체 — evaluation.md §6.1).

### 2.3 N4 — queue 정지점 감도 [계산, `B2rec-n4`]

교통 관리자의 통로 토큰 요청 지점은 입구(y = −8) 2.5 m 앞 (0, −5.5)이고, 요청 뒤 멈출 때까지 최대 1.1 m 더 들어온다
(`traffic.yaml` 34–35행 — 1 m/s, 주기 0.5 s + 지연 0.1 s + 제동 0.5 m) → **현실 정지 구간 y ∈ [−6.6, −5.5]**.
통로 중앙 (0, −10) 단일 스캔 축 구속 점 (블로커 = 이상점):

| 정지점 y | −4.4 | −5.0 | **−5.5** | −6.0 | −6.6 | (open) | (wall) |
|---|---|---|---|---|---|---|---|
| 2D 720 @0.20 | 61 | 61 | **59** | 57 | 55 | 70 | 0 |
| 16ch @0.90 1800 열 | 7 906 | 7 902 | **7 902** | 7 927 | 7 927 | 7 927 | 7 927 |

현실 구간 변화 59 → 55 = 열림의 5.7 % (사전 규칙 ≤ 10 %) → **기존 `s6-queue` (0, −5.5) 를 그대로 녹화**한다. 리뷰 N4 의
(0, −5.0)·(0, −4.4) 는 요청 지점에서 입구 반대쪽이라 후보에서 뺐다. 원자료 `/home/CAPYI_AI/runs/b24/b2-rec/n4/`.

## 3. 녹화 결과

(녹화 뒤 채운다 — `rec_report.py table` 출력)

## 4. P 단계에서 쓰는 법

(녹화 뒤 채운다)

## 5. 재생 스모크

(녹화 뒤 채운다)
