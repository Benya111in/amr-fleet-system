# 보너스 2 R·G — 녹화 프로토콜·자동 공백 검사와 시나리오 녹화 S3·S1×3·S6×3·MAP (사전 등록)

- 시각: 2026-10-08 10:30 KST · 에이전트: b2-rec · 단계: R(녹화 준비)·G(녹화) — `bonus2_completion_plan.md` §4
- 코드: 사전 등록 커밋 = 이 항목을 넣은 커밋 (브랜치 `feature/b2-rec`, 분기 `3409599`). 녹화·검사 코드는 `cb93ae8`,
  평가 도구 수정(N6·N11·N4) `d988950`. 실행 때 커밋은 실행 목록 `runs/b2-rec.jsonl` 의 `commit` 과 시도 디렉터리 `HEAD.txt`.
- 관련 실행 id: `B2rec-<시나리오>_a<k>` (녹화 시도), `B2rec-dry-*` (시운전), `B2rec-smoke-*` (재생 스모크),
  `B2rec-n4` (정지점 감도 계산), `B2rec-test-*` (시험·변이)
- 근거: `replay.md` §1.2·§5·§6·§7·§9 (파일럿 bag 결함), `evaluation.md` §6·§6.1·§6.2·§8·§10, `rtabmap.md` §3·§7,
  `sensor.md` §2·§6, 탐색 계산 `/home/CAPYI_AI/runs/b24/b2-rec/explore/pilot_gaps{,2}.json`·`pilot_gate.json`

## 질문

1. 파일럿 bag 의 기록 결함(sim 9.48–9.64 s 정체: `/clock` 틱 누락·EKF `/tf` 0.065·0.1 s 공백, 기록 madgwick 의 입력 유실)을
   녹화 프로토콜과 기록 직후 자동 공백 검사로 막아, 시나리오마다 검사를 통과한 bag 을 얻을 수 있는가 — 몇 번 시도에?
2. P 단계(재생·평가)가 그대로 쓸 bag 목록: S1 정상 3 경로 + 긴 매핑 1, S3 좁은 통로 왕복 1, S6 통로 끝 열림·대기열·벽 3 —
   각 bag 의 토픽별 메시지 수·sim 주기·공백, 씨드·경로·RTF·loadavg·HEAD.
3. 재생 시작을 "고정 15 s" 대신 "멈춘 재생 + 구독 확인 뒤 resume" 으로 바꾸면, 검사 통과 bag 에서 RTAB-Map 3D(IMU 없음)의
   처리 스캔 누락이 0 인가 (파일럿 bag 은 모든 RTAB-Map 행에서 1/1 788).

## 가설과 예측 (결과 보기 전에 적는다)

정직성 메모: 공백 검사 규칙은 **새 녹화를 보기 전에** 파일럿 bag 의 간격 분포를 보고 정했다 (탐색 `pilot_gaps2.json`:
부하 32–48 에서 179 s 동안 `/clock` 1 ms 틱 누락 63 회(2–23 ms), EKF 간격 > 30 ms 11 회(35·39·65·100 ms), 점군·IMU 0 회).
규칙 자체는 과제 지시("간격 > 1.5 주기면 실패")를 그대로 따랐고, 파일럿을 보고 바꾼 것은 **경계 처리**(정수 ns + 1 µs —
실수로 세면 정확히 1.5 주기인 EKF 0.030 s 가 공백 42 개로 더 잡힌다)뿐이다.

- **H-R0 (도구 정합)**: 새 검사 도구는 파일럿 bag 을 실패로 판정한다 (`/clock`·EKF). → 이미 확인 [측정 `explore/pilot_gate.json`:
  `/clock` 63, EKF 11, 점군·IMU 0, 정지 구간 26.0 / 8.8 s 통과]. 이것은 가설이 아니라 도구 검증이다.
- **H-G1 (녹화 통과율)**: 녹화 중 새 프로세스를 띄우지 않는 프로토콜이면 bag 마다 3 회 안에 검사를 통과한다. 예측: 8 bag 중
  ≥ 6 이 1 회차 통과, 8 bag 모두 3 회 안 통과. 실패한다면 가장 흔한 원인은 `/clock` 1 kHz 틱 누락(2–5 ms)일 것이다
  (파일럿에서 가장 흔했다). **확신 낮음**: 공유 부하가 높으면(다른 과제의 시나리오 12 짝이 겹치지 않아도 학습·빌드 등)
  1 kHz `/clock` 이 179 s 동안 한 번도 안 빠질지 모른다.
- **H-G2 (스모크)**: 검사 통과 bag 하나(`s1_seed1`)를 `rtabmap3d_noimu` 로 재생(배속 0.5, 구독 확인 뒤 resume)하면 처리 스캔
  판정 누락 0 (분모 = bag 장 수, `bag_tools match`·로그 줄 중 큰 값). 예측: 0.
- **H-G3 (구독 확인)**: 구독 확인 단계가 행 세 갈래(RTAB-Map·slam_toolbox·LIO-SAM)에서 모두 300 s 안에 끝나고(`ready_ok: 1`),
  LIO-SAM 이 첫 장을 "Waiting for IMU data" 로 버리지 않는다. 예측: 셋 다 통과.

## 설정

### 녹화 프로토콜 (`src/amr_slam3d/scripts/record_bag.sh`, 컨테이너 `amr-ai/slam:0.1`)

1. 월드: `gen_warehouse_world.py --variant <V> <출력>/world/warehouse.sdf` (기본 월드 출력은 바이트 동일 — 생성기 시험).
2. `multi_robot.launch.py` amr_01(3D LiDAR) 1 대, `localization_mode:=odom` (휠·imu_filter·scan_filter·odom EKF), Nav2·
   perception·behavior·fleet 없음. **madgwick 을 띄우지 않는다** — 기록은 원시 IMU(`imu/data_raw`·`imu/data`)만
   (`rows.RECORD_TOPICS`, `/amr_01/imu/data_madgwick` 없음). madgwick 은 녹화 뒤 `madg_lockstep.py` 로 파생 bag(`bag_madg`, gain
   0.1/0.01 + 오라클 방위 토픽)을 만든다 (검사 통과 bag 만).
3. `route_driver.py` 를 **기록 전에** 띄워 0 명령을 낸다. sim 10 s 가 지나면 ready 파일 → 그때 `ros2 bag record -s mcap`
   (SIGINT 처분 복구 래퍼). 기록기 로그에서 14 토픽 모두 "Subscribed to topic" 을 확인하고 3 s(벽) 뒤 go 파일. 기록 중에는 새
   프로세스를 띄우지 않는다 (파일 검사만 — 파일럿 정체는 route_driver 기동과 겹쳤다, replay.md §7).
4. 주행기는 go 부터 `hold_start_s` 10 s(S6 은 sim 60 s 까지) 정지 → 경로 → `hold_end_s` 3 s 정지 → done 파일 → 기록기 SIGINT
   (metadata.yaml 확인) → quit 파일 → 주행기 종료 → 정리.
5. 사후: `ros2 bag info`, `bag_tools stats`(토픽별 수·sim 주기), `bag_tools gate`(공백 검사), `gt.tum`, 스폰 파일 사본
   `<bag>/b2rec_spawn.yaml` (재생 anchor 용).
6. 실행 환경: 호스트 구동기 `rec_driver.sh`(nohup setsid, 재개 가능, `driver.lock`)가 `gz_slot.sh b2-rec` 안에서 `rec_attempt.sh` →
   docker run (`amr-ai-b2-rec-gz`, `--rm`, `--gpus all`, `--shm-size=8g`), **ROS_DOMAIN_ID 47**, `IGN_PARTITION=b2rec_<태그>`,
   `ROS_LOCALHOST_ONLY=1`. 슬롯을 얻은 뒤 시도 디렉터리에 `HEAD.txt`·`diff_stat.txt`·`status.txt`·`started.txt`·`loadavg_start.txt`,
   끝에 `rc.txt`·`loadavg_end.txt`. 원자료 `/home/CAPYI_AI/runs/b24/b2-rec/rec/<id>_a<k>/`.

### 공백 검사 규칙 (`amr_slam3d/recgate.py`, `bag_tools.py gate`) — 판정

통과 = 아래 모두:
- **간격**: 검사 토픽마다 기록된 sim 스탬프(정수 ns)의 이웃 간격 ≤ 1.5 × 명목 주기 + 1 µs, 같은 스탬프 반복 0, 역행 0.
  검사 토픽·명목 주기: `/clock` 1 ms, `/tf` 안 EKF `amr_01/odom → amr_01/base_footprint` 20 ms, `lidar3d/points_raw`·
  `lidar3d/points` 100 ms, `imu/data_raw`·`imu/data` 5 ms.
- **덮음**: 각 검사 토픽의 첫·끝 스탬프가 `/clock` 의 첫·끝과 max(1.5 주기, 0.5 s) 안.
- **정지 구간**: 정답 자세로 bag 처음 정지 ≥ 10 s, 끝 정지 ≥ 0.5 s (움직임 = 첫/끝 자세에서 xy 5 mm 또는 yaw 0.2° 초과).
- **S6 출발 시각**: 처음 움직인 sim 시각 ∈ [60, 60.5] s (세 변형의 actor 상대 시각을 맞추는 짝 조건).
- 진단(판정 밖): 기록 순서상 직전 `/clock` 대비 센서 스탬프 앞섬(clock lead), 점군 원본·전처리 수 차.
- 실패하면 그 bag 은 쓰지 않고 다시 기록한다. **재기록 상한 3 회** (시나리오마다). 3 회 모두 실패하면 `state/<id>.failed` 로
  남기고 실패 시도별 원인 표를 이 항목에 쓴다 — 규칙을 결과 뒤에 완화하지 않는다 (완화가 필요하면 "결과 뒤 변경" 으로
  따로 적고 그 bag 은 "검사 미통과" 표기).
- 끊긴 시도(디렉터리만 있고 `rc.txt` 없음)도 시도 1 회로 센다. 슬롯 대기 초과(rc 75, 시작 전)는 세지 않는다.

### 시나리오 (`src/amr_slam3d/config/rec_plan.txt`, 경로 `config/routes/*.yaml` = `make_routes.py write`, 시험이 대조)

| id | 월드 | 스폰 | 경로 | 길이 | 비고 |
|---|---|---|---|---|---|
| `s3_aisle` | s1 | (18, −16, π/2) | (18, −5.5) → (0, −5.5) → 중앙 (0, −10) **15 s 정지** → (0, −14.5) 2 s → (0, −5.5) | 46.5 m | S3 좁은 통로 왕복 (evaluation.md §6) |
| `s1_seed1` | s1 | 같음 | routegen 씨드 1 걸음 (60 m 뒤 출발 마디 (18, −15) 로 최단 귀환), 일직선 합침 | 61.0 m | S1 정상 |
| `s1_seed2` | s1 | 같음 | 씨드 2 | 97.0 m | S1 |
| `s1_seed3` | s1 | 같음 | 씨드 3 | 80.0 m | S1 |
| `s6_open` | s6-open | (0, −8.6, −π/2) 통로 안 | 중앙 (0, −10) **15 s** → (0, −11.4) 2 s → **후진** (0, −8.6) 2 s, sim 60 s 출발 | 5.6 m | 요인 E 기준 (actor 4) |
| `s6_queue` | s6-queue (N4 결과로 정함 — 아래) | 같음 | 같음 | 5.6 m | 현실 대기 |
| `s6_wall` | s6-wall | 같음 | 같음 | 5.6 m | 최악 (양끝 입구 화물) |
| `map_long` | s1 | (18, −16, π/2) | 차선 지그재그 y −15 → −6 → 0 → 6 → 13.3, 서쪽 끝 차선·남쪽 홀 되짚어 귀환 | 308.6 m | 긴 매핑 (MAP) |

- 공통: v_max 0.5 m/s, w_max 0.5 rad/s, 회전 지점 정지 1 s, GT 되먹임 (route.py, 20 Hz, sim 시각).
- **"씨드"** = S1 경로 생성 난수 씨드 (`routegen.random_walk`, 차선 그래프 위 걸음 — 직전 마디로 되돌아가지 않음). 정적 월드라
  출발 지연 씨드(evaluation.md §6 의 동적 시나리오용 정의)는 장면을 바꾸지 못하므로 쓰지 않는다. 씨드 1·2·3 은 경로를 보기
  전에 정했다 (계획 문서·과제 지시의 "씨드가 다른 경로 3 개").
- **S6 경로를 통로 안에 둔 이유**: s6-wall 은 양끝 입구(y −7.75·−12.25)를 막아 밖에서 들어갈 수 없다. 세 변형의 짝 비교를 위해
  같은 경로를 쓰므로 모두 통로 안에서 출발한다. 순폭 0.60 m < 외접 지름 0.72 m 라 통로 안에서 돌 수 없어 돌아올 때는
  후진한다 (route.py reverse — 이상 모델 시험: 방위 변화 < 0.05 rad, 횡오차 < 0.05 m, 바퀴 치우침 0.02 rad/s·미끄러짐 3 %).
  s6-* 월드에는 actor 4 명·차량 2 대가 있고 `worker_crossing` 이 통로 상자(±0.5, y −12.4…−7.6)에서 0.6 m 까지 온다 [계산
  `build_sdf('s6-open')` specs]. 출발 sim 시각을 60 s 로 고정해 세 변형에서 actor 와의 상대 시각이 같게 한다.
- **긴 매핑 경로**: slam.md §6.1 의 322 m 경로는 웨이포인트 파일이 저장소에 없다 (`git log -S'322.4'`·작업 디렉터리 검색).
  같은 차선(y 0·±6, 북 13.3, 남 −15, 동·서 끝 ±21.5)으로 다시 만들었다 (308.6 m, ±27.5 차선은 기둥 (±25, ±6) 때문에 뺐다).
- 장애물 여유 [계산 `make_routes.py check`, GT 대역 0.05–2.0 m 래스터 0.05 m (`runs/b24/b2-rec/gt05`, sdf sha256
  2efa665e…, 커밋 SDF 와 같음) → `explore/clearance_v1.json`]: 통로 밖 선분 중심선 ≥ 0.66 m(외접 0.36 + 0.30), 회전 지점 ≥ 0.46 m,
  통로 상자(y 로 0.6 m 넓힘) 안 ≥ 0.25 m — 6 경로 모두 통과. 최악: map_long (10.24, 13.3) 기둥 (10, 12) 앞 여유 1.05 m,
  s3·s6 통로 안 0.30 m (래스터 셀 중심 기준).

### N4 — S6 queue 정지점 감도 (녹화 전에 계산, 판정 규칙을 먼저 적는다)

- 계산: `python3 -m amr_evaluation.slam3d.localizability --gt-dir <gt05> --poses aisle_centre aisle_near_end
  --variants open queue@-4.4 queue@-5.0 queue@-5.5 queue@-6.0 queue@-6.6 wall --sensors 0 7` (2D 720 @0.20, 16ch @0.90 1800 열),
  블로커 = 지도에 없는 이상점(기본). 
- 근거: `traffic.yaml` 34–35행 — 통로 요청 거리 2.5 m (입구 y = −8 에서 → 요청 지점 y = −5.5), 요청 뒤 멈출 때까지
  "1 m/s 에서 주기 0.5 s + 지연 0.1 s + 제동 0.5 m = 1.1 m" 더 간다 → **현실 정지 구간 y ∈ [−6.6, −5.5]** (입구 쪽).
  리뷰 N4 가 제안한 (0, −5.0)·(0, −4.4) 는 요청 지점에서 입구 **반대쪽**이다 — 계산은 하되 후보에서 뺀다.
- 판정 규칙: 통로 중앙(aisle_centre)의 2D 축 구속 점 수가 현실 구간 세 점(−5.5, −6.0, −6.6)에서 열림 값의 10 % 이내로만
  변하면 기존 `s6-queue` (−5.5) 를 그대로 녹화한다. 10 % 를 넘게 변하면 현실 구간에서 2D 축 구속 점이 가장 적은 정지점
  (현실 최악)을 새 변형 `s6-queue-stop` 으로 월드 생성기에 더하고(기본 출력 바이트 동일, `check_blockers` 통과) 그것을 녹화한다.

### 재생 스모크와 구독 확인 (H-G2·H-G3)

- `replay_row.sh` (cb93ae8): `ros2 bag play -p` → `rows.ready_requirements(행)` 의 (토픽, 구독 노드)가 `ros2 topic info -v` 에
  모두 보이고 재생기가 입력 토픽 발행자로 보일 때 → 2 s → `/rosbag2_player/resume`. 상한 300 s (넘으면 종료 코드 4).
- H-G2: `s1_seed1` 의 통과 bag, `rtabmap3d_noimu`, 배속 0.5, DOMAIN 47, `cpu_slot.sh`. 판정 = 처리 스캔 판정 누락 0.
- H-G3 (보충 — 녹화 bag 이 생긴 뒤): `s3_aisle` bag 으로 `slam_toolbox`(원 bag)·`liosam_planar`(파생 bag) 각 1 회 —
  `ready_ok: 1`, LIO-SAM 기동 경합 버림 0 (로그 "Waiting for IMU data" 0, 판정 누락 0).
- 이 스모크는 P 단계 평가가 아니다 — 궤적·지도 지표는 재지 않는다 (P 단계에서 행 × bag × 재생 3).

### 시운전 (P 집합에 넣지 않음)

녹화 전 시운전 1 회: `s6_inside` 경로 + `s6-wall` 월드 (통로 안 스폰·후진·출발 시각·블로커가 모두 걸린 가장 까다로운 조합),
원자료 `/home/CAPYI_AI/runs/b24/b2-rec/dryrun/`. 프로토콜·도구 결함을 찾으면 고치고 다시 시운전한다. 시운전 bag 은 검사를
통과해도 P 집합에 넣지 않는다 (코드가 바뀔 수 있으므로).

### 공유 부하

녹화는 `gz_slot.sh` 로 돈다 — b4-s12 의 `exclusive` (시나리오 12 짝, Gazebo 둘)가 끝난 뒤 자동으로 시작한다. RTF·시간은
"공유 부하 하 예비값" 이고 loadavg 를 시도마다 남긴다. 공백 검사는 부하에 민감하다 — 부하가 실패 원인이면 그렇게 적는다.

## 결과

### N4 정지점 감도 (녹화 전, 실행 `B2rec-n4`) [계산]

원자료 `/home/CAPYI_AI/runs/b24/b2-rec/n4/n4.csv`·`n4.log` (HEAD `d7c6b2c`, 10:22–10:23 KST, loadavg 72–74 — b4-s12 시나리오 12 짝
실행 중; 결정적 계산이라 부하는 결과에 영향 없음). 축 구속 점 수 (|n_x| > 0.7, 블로커 = 이상점):

| 자세 | 센서 | open | queue@−4.4 | queue@−5.0 | **queue@−5.5** | queue@−6.0 | queue@−6.6 | wall |
|---|---|---|---|---|---|---|---|---|
| 통로 중앙 (0, −10) | 2D 720 @0.20 | 70 | 61 | 61 | **59** | 57 | 55 | 0 |
| 통로 중앙 (0, −10) | 16ch @0.90 1800 열 | 7 927 | 7 906 | 7 902 | **7 902** | 7 927 | 7 927 | 7 927 |
| 남쪽 끝 근처 (0, −11.5) | 2D | 142 | 135 | 135 | 133 | 133 | 133 | 0 |
| 남쪽 끝 근처 (0, −11.5) | 16ch | 9 738 | 9 721 | 9 721 | 9 738 | 9 738 | 9 738 | 9 738 |

- 현실 정지 구간 [−6.6, −5.5] 에서 통로 중앙 2D 축 구속 점은 59 → 55 (열림 70 의 **5.7 %**) — 사전 규칙(≤ 10 %)에 따라
  **기존 `s6-queue` (0, −5.5) 를 그대로 녹화한다** (계획 파일 그대로). 정지점이 입구 쪽으로 1.1 m 들어와도 2D 는 퇴화하지 않는다
  (55 개 남음, λmin/λmax 0.085) — evaluation.md §0-5 "현실적인 대기는 2D 를 퇴화시키지 못한다" 가 정지점 감도에도 유지된다.
- 리뷰 N4 의 (0, −5.0)·(0, −4.4) 는 요청 지점에서 입구 반대쪽이라 후보가 아니었다 — 값은 61 (−5.5 보다 구속이 더 남는다).
- 3D 16 채널은 모든 queue 정지점에서 7 902–7 927 (열림 대비 −0.3 % 이내), wall 에서도 7 927 — 단일 스캔 완전 대응 하한이다.

### 시운전 1 회차 (`B2rec-dry_s6wall_a1`, 실제 녹화 전 — P 집합 밖) [측정]

원자료 `/home/CAPYI_AI/runs/b24/b2-rec/dryrun/rec/dry_s6wall_a1/` (HEAD `898d067`, 16:23–16:25 KST — b4-s12 의 exclusive 가
16:22:54 에 끝나 바로 시작, loadavg 1 분값 14.4 → 11.0, 다른 사용자 프로세스(openroad·dgcom 등) 동시 실행).

- 프로토콜 기계 부분은 모두 됐다: 스택 기동 17 s, 주행기 ready 13 s 뒤, 기록기 14/14 구독 즉시, go → **출발 sim 60.14 s**
  ([60, 60.5] 안), 처음 정지 46.4 s·끝 정지 5.0 s, 기록기 SIGINT 깨끗한 종료(metadata.yaml), 통로 안 스폰·후진 완주.
- **공백 검사 실패**: `/clock` 1 ms 틱 누락 9 (2 ms ×5, 4·6·10·13 ms), EKF 간격 초과 5 (33·34·35·38·54 ms), `imu/data` 1 (15 ms,
  sim 34.15 s). 점군·`imu/data_raw` 0. 실제 RTF 0.85 (sim 81.3 s / 벽 95.5 s). 큰 것들은 sim 34.1–34.4 s 한 곳에 몰렸다
  (순간 정체), 2 ms 짜리는 흩어져 있다.
- 원인 진단 [측정 + 추론]: (1) 기록 bag 의 `joint_states`(1 kHz) 가 `/clock` 보다 30 개 적다 — 기록기 구독 깊이 기본 10 은 1 kHz
  토픽을 10 ms 지연에도 잃는다 → 2 ms 틱 누락은 기록기 쪽일 가능성이 크다. (2) `imu/data` 의 15 ms 공백은 `imu_filter_node` 의
  입력 구독이 SensorDataQoS(best-effort 깊이 5 = 25 ms) 라 처리 노드 쪽에서 잃은 것이다 (`imu/data_raw` 는 온전). (3) EKF 33–54 ms
  는 20 ms 격자가 아닌 간격이라 EKF 타이머가 늦게 돈 것이다 (sim 시각 타이머 — 처리 노드가 CPU 를 기다림).

**시운전 뒤 프로토콜 변경 (실제 녹화 전, 공백 검사 규칙은 그대로)** — 사전 등록의 "시운전으로 결함을 찾으면 고치고 다시
시운전" 에 따른 것:
1. 생성 월드 사본의 물리 `real_time_factor` 를 1.0 → **0.5** 로 (`RTF_TARGET`, `record_bag.sh`). 물리 스텝·센서 주기·주행은
   sim 시각이라 데이터는 같고, 처리 노드(EKF·imu_filter·scan_filter·전처리)가 sim 주기마다 쓸 벽시계 여유가 두 배가 된다.
   녹화 벽시계 시간은 늘어난다 (긴 매핑 약 25 분).
2. 기록기 구독 QoS 깊이 (`rows.record_qos_yaml`, `--qos-profile-overrides-path`): `/clock`·`/tf`·IMU 둘·`joint_states` 1000,
   odom 셋 500, `cmd_vel` 200, 스캔 둘 100, 점군 둘 50. reliable·volatile (발행자와 같음 — 시운전 bag metadata.yaml).
3. 시운전 2 회차를 같은 조합(`s6_inside` + `s6-wall`)으로 다시 돈다. 통과하면 본 녹화를 시작하고, 실패하면 원인을 더 본 뒤
   이 항목에 적는다 (본 녹화의 시도 3 회 상한·규칙은 그대로).

### 시운전 2 회차 (`B2rec-dry2_s6wall_a1`, 88de1fa — RTF 0.5 + 기록기 깊이) [측정]

원자료 `dryrun/rec/dry2_s6wall_a1/` (16:29–16:34 KST, loadavg 1 분값 13.0 → 4.3). **공백 검사 통과**: 검사 토픽 6 개 모두 간격 초과 0
(`/clock` 82 776 개 간격 최대 1 ms, EKF 4 139 개 최대 25 ms, 점군 828·828 최대 0.100 s, IMU 16 556·16 556 최대 5 ms), 출발
sim 60.14 s, 처음 정지 47.9 s·끝 5.0 s. 실제 RTF 0.49 (목표 0.5). `joint_states` 82 777 = `/clock` 82 776 + 1 (1 회차의 30 개 부족이
사라졌다 → 기록기 깊이가 원인이었다는 진단과 맞다). clock lead 최대 점군 −0.001 s·IMU 0.008 s. 파생 bag 16 556 표본, 잠금 단계
1.2 s, 예상 밖 출력 0. 한 번의 통과라 "RTF 0.5 면 늘 통과" 는 아니다 — 본 녹화의 시도별 결과로 본다.
본 녹화는 16:34 에 시작했다 (`rec_plan.txt` 그대로, 시도 상한 3).

### 결과 뒤 변경 — 프로토콜 (s1_seed2 시도 1·2 실패 뒤, 2026-10-08 17:23) [측정 근거]

시도별 실제 RTF 와 판정 (rec/*/bag_meta.json, gate.json): 통과 — dry2 0.490, s3_aisle_a1 0.493, s1_seed1_a1 0.493 (모두 목표 0.5 에
붙음); 실패 — s1_seed2_a1 **0.413** (EKF 35–44 ms 19, imu/data 25 ms 1, loadavg 1 분값 3.4 → 5.2, 5 분값 5.3 → 12.0),
s1_seed2_a2 **0.339** (EKF 29, imu/data 2, loadavg 5.2 → 37.7 — 다른 사용자 프로세스 openroad·dgcom·ruby 등이 몰림).
**실제 RTF 가 목표 아래로 떨어진 녹화만 실패했다** — 시뮬이 목표를 못 따라가면 처리 노드의 벽시계 여유가 없어진다.
공백 검사 규칙·시도 상한은 그대로 두고 프로토콜만 바꾼다 (시도 s1_seed2_a3 은 바꾸기 전 판으로 이미 시작):
1. `RTF_TARGET` 0.5 → **0.25** (`record_bag.sh`). 녹화 벽시계 시간은 두 배 (긴 매핑 약 50 분).
2. 시작 조건: loadavg 1 분값 < 8 일 때만 시도를 시작 (`rec_attempt.sh`, b4-s12 짝 실행과 같은 문턱, 최대 6 h 대기 —
   기다린 시간은 시도 디렉터리 `load_wait.txt`). 시도 중간의 부하 급증은 막지 못한다.
3. 적용: 바꿔치기(17:23:33) 뒤에 시작한 시도부터. **s1_seed3_a1 은 14 s 먼저(17:23:19) 시작해 바꾸기 전 판(RTF 0.5, 시작 조건
   없음, HEAD 32caa7d)으로 돌았다** (env.txt `rtf_target: 0.5`, `load_wait.txt` 없음) — 실패했고 s1_seed3 의 시도 1 회로 센다.
   실행 중인 스크립트를 깨지 않게 새 파일로 바꿔치기(rename)했다 — 진행 중이던 시도는 바꾸기 전 판으로 끝까지 돈다.
4. s1_seed2 가 사전 등록 3 회를 모두 실패하면 그 결과(`state/s1_seed2.failed`)는 그대로 두고, 바뀐 프로토콜로 **사전 등록 밖
   추가 시도** `s1_seed2x` (같은 경로, 최대 3 회)를 따로 돌린다 — 그 bag 은 "사전 등록 밖 · 결과 뒤 변경" 으로 표기한다.

### 본 녹화 결과 (16:34–19:46, 추가 21:33–21:56) [측정, 공유 부하 하 예비값]

상세 표: [`../../recordings.md`](../../recordings.md) §3 (bag·토픽별 수·간격, 모든 시도), 원본 `runs/b24/b2-rec/table_final.md`.

| 시나리오 | 판정 | 통과 시도 | 실패 시도 (원인) |
|---|---|---|---|
| s3_aisle | 통과 | a1 (RTF 0.5 / 0.493) | — |
| s1_seed1 | 통과 | a1 (0.5 / 0.493) | — |
| **s1_seed2** | **실패 (사전 등록 3 회 모두)** | — | a1 EKF 19·imu/data 1 (0.413), a2 EKF 29·imu/data 2 (0.339), a3 EKF 12 (0.467) |
| s1_seed3 | 통과 | a2 (0.25 / 0.231) | a1 EKF 16·imu/data 1 (0.5 / 0.327, 바꾸기 전 판) |
| s6_open | 통과 | a3 (0.25 / 0.249) | a1 EKF 1 (31 ms, 0.232), a2 EKF 1 (35 ms, 0.238) |
| s6_queue | 통과 | a1 (0.25 / 0.249) | — |
| s6_wall | 통과 | a1 (0.25 / 0.248) | — |
| map_long | 통과 | a1 (0.25 / 0.248, 695.6 s sim, 12.5 GB) | — |
| s1_seed2x (**사전 등록 밖**) | 통과 | a1 (0.25 / 0.203) | — |

- **H-G1 틀림**: 1 회차 통과 5/8 (예측 ≥ 6), 3 회 안 통과 7/8 (예측 8/8). 실패 원인은 예측한 `/clock` 틱 누락이 아니라 **EKF `/tf`
  타이머 지연**이었다 (`/clock`·점군·`imu/data_raw` 공백 0 — 기록기 깊이 1000 이 `/clock` 손실을 없앴다).
- **H-G2 미달**: `s1_seed1_a1` 의 `rtabmap3d_noimu` 판정 누락 **4 / 1 674** (예측 0). bag 기록 순서로는 설명되지 않는다 (recordings.md §5) —
  재생 쪽, 원인 미확인.
- **H-G3**: 구독 확인 세 갈래 모두 1–5 s, LIO-SAM 기동 경합 버림 0·판정 누락 0 (통과). slam_toolbox 는 메시지 필터 버림 1 (미진단).
- 파생 bag: 8 bag 모두 (s3_aisle 은 다시 파생), 출력 수 = imu/data 수, 예상 밖 출력 0, 오라클 토픽 짝 없음은 s3_aisle 의 첫 1 개뿐.

## 해석

- 녹화 결함(파일럿의 `/clock` 정체·madgwick 입력 유실)은 이번 bag 들에 없다: 공백 검사 통과 bag 은 `/clock` 1 ms·점군 100 ms·IMU 5 ms·
  EKF ≤ 30 ms 간격이고 clock lead(센서가 기록 순서상 `/clock` 보다 앞섬)는 ≤ 13 ms (파일럿 정체 120–160 ms). madgwick 은 기록하지
  않고 결정적으로 파생했다.
- 공백 검사 실패는 **처리 노드(EKF·imu_filter)의 타이머·입력 지연**이었고, 시뮬이 RTF 목표를 못 따라갈 때(다른 사용자 부하) 생겼다.
  이것은 "기록"이 아니라 "녹화 중 스택이 만든 데이터" 의 흔들림이다 — 같은 지연은 실제 운용에서도 생긴다. 규칙은 그런 흔들림 없는
  bag 만 고르게 했고, 그 대가로 시도가 늘었다 (14 시도에 8 bag). 31·35 ms 한 번으로 버린 s6_open a1·a2 는 규칙이 매우 엄격하다는
  예다 — 규칙은 결과 뒤에 바꾸지 않았다.
- H-G2 의 4 장 누락은 "검사 통과 bag 이면 RTAB-Map 누락 0" 이 아니라는 뜻이다. P 단계는 누락을 재생마다 세어 지표와 함께 내야 하고,
  부하 조건을 맞춰야 한다 (recordings.md §5.2 보충 재실행 참고).
- RTF 목표가 bag 마다 0.5 / 0.25 로 다르다 (결과 뒤 변경). sim 데이터는 같지만 재생 배속을 RTF 에 맞춰야 처리 조건이 같아진다.

## 다음

- P 단계 (다음 물결): `recordings.md` §4 대로 행 × bag × 재생 3. 재생 배속 = 목표 유효 속도 / bag RTF 를 사전 등록. S1 은 사전 등록
  bag 2 개(s1_seed1·s1_seed3) + 사전 등록 밖 1 개(s1_seed2x) — 둘 다의 결과를 낸다. H-G2 누락을 행·재생마다 센다.
- 공백 검사 규칙: EKF 간격 1.5 주기는 부하 하에서 엄격하다. 다음 녹화를 한다면 규칙을 **녹화 전에** 다시 정한다 (예: EKF 는
  2 주기, `/clock`·센서는 1.5 주기) — 이번 결과를 보고 정한 것이므로 이번 bag 에는 소급하지 않는다.
- 미확인으로 남긴 것: H-G2 4 장 누락의 기제, slam_toolbox 메시지 필터 버림 1, s6_open 의 EKF 지연이 둘 다 sim 41–43 s(출발 전
  정지)에 난 것이 우연인지.
