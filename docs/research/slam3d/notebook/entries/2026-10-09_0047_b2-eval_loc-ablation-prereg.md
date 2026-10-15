# 보너스 2 P 단계 E3(b) 위치추정 모드·E5(위치추정)·E4 절제 실험 (사전 등록)

- 시각: 2026-10-09 00:47 KST (작성 시작) · 에이전트: b2-eval · 단계: P-E3E4 (`bonus2_completion_plan.md` §3 E3(b)·E4·E5)
- 코드: 사전 등록 커밋 = 이 항목을 넣은 커밋. 재생 사본 `ws_e3b` = `a0850e1` (git archive, `SNAPSHOT_HEAD`).
  정답 자세 DB(`rtabmap3d_gtdb`, map_long) 만 그 전 사본 `ws_e3` = `ebf0cea` 에서 돌았다 (그 행의 코드는 두 커밋에서 같다 —
  `git diff ebf0cea a0850e1 -- src/amr_slam3d/scripts/gt_odom_tf.py src/amr_slam3d/launch/rtabmap.launch.py` 빈 출력).
  분석 스크립트 sha256 (이 커밋의 파일):
  - `src/amr_slam3d/scripts/peval.py` `dfe200753dcf8d43ce17ada9387ed322f37fdf1f113115b6feafa015f548b91c`
  - `src/amr_evaluation/amr_evaluation/slam3d/pstats.py` `58c37a2e1846735eaac447b774443a4df6ee34deb56064bb3df310c52249d048` (E2 항목과 같음)
  - `src/amr_evaluation/amr_evaluation/slam3d/pmetrics.py` `916666f0a4223e1f121418adb24c111629cc5a30fbe59bcba05831a90ac0d6f0` (같음)
  - `src/amr_slam3d/scripts/eval_driver.py` `cd98ff7e4d9d967612e5d18e51b658db4cb82c3564097007b8a6f7b520cb3e84` (실행 중 구동기는 같은 판의 사본 `runs/b24/b2-eval/bin/eval_driver_2648ad7.py` — 이후 바뀐 것은 작업 목록(`plan_e3` 의 2D 정답 지도 경로)뿐)
  peval.py 는 E2 항목(686d902 판 `6de70f13…`) 뒤에 **더하기만** 했다 (select-maps·perfect-map·compare-loc·compare-abl·`loc_fail_of`·
  `--grid-band`·`--warmup`·`family_tests(families=)` 기본값 그대로). E2 의 실패 규칙·가족 T·M·창 규칙은 바뀌지 않았다 — 시험
  `test_loc_fail_rules_and_e2_rules_unchanged`·`test_peval_families_and_feasibility` 가 고정한다.
- 관련 실행 id: `B2eval-e3-<bag>-<행>-r<k>`, `B2eval-e3-map_long-gtdb`, `B2eval-e4-<bag>-<행>-r<k>`, `B2eval-pilot-*` (기동 확인 — 데이터 밖)
- 근거: E2 항목 `2026-10-09_0000_b2-eval_mapping-compare-prereg.md` (지도 선택 규칙, 공통 규칙), 보고서 「위치추정은 AMCL 자리만 바꾸는
  구성으로 비교한다」·「포인트 클라우드 처리와 최적화는 여섯 개의 절제 실험으로」 절, `docs/algorithms/slam.md` §4·§6.3 (배포 2D 위치추정
  사슬과 그 정확도), `evaluation.md` §6.1·§6.2, `rtabmap.md` §1–§2.

## 질문

1. (E3(b)) 같은 녹화 7 개(map_long 제외 — 지도를 만든 bag)에서, **정답 지도 위**와 **각자의 SLAM 지도 위**의 위치추정 정확도가
   2D(배포 AMCL 사슬) 와 3D(RTAB-Map 위치추정 모드) 사이에 다른가? 명세 구간(정지·직선·회전) RMSE·max·p95 가 팀 기준
   3 / 5 / 8 cm 안인가? 위치추정 실패(발산·LOST)는 몇 번인가?
2. (지도의 몫) SLAM 지도 − 정답 지도의 위치추정 오차 차이 (보고서 "지도 품질이 위치추정에 주는 몫").
3. (E5 위치추정 쪽) 사전 지도(블로커 없음) 위에서 통로 끝을 막으면(s6_wall·s6_queue) 2D 의 통로 축 오차가 열림 대비 커지고 3D 는
   덜 커지는가 — evaluation.md §6.2 의 단일 스캔 기하 계산(2D 축 구속 점 open 70 · queue 59 · wall 0, 16ch 7 927 · 7 902 · 7 927)이
   예측하는 방향이 실제 위치추정 오차로 나타나는가?
4. (E4) RTAB-Map 3D 매핑에서 점군 처리·최적화 손잡이 하나씩(복셀 크기, 점-대-점/평면, 높이 대역, 루프 거부 문턱, 비유한 점 제거)이
   궤적·지도·스캔당 지연을 어떻게 바꾸는가?

## 가설과 예측 (결과 보기 전에 적는다)

E2 의 결과는 아직 보지 않았다 (재생 진행 상태 — 누락·rc 만 확인). 아래 예측의 근거는 파일럿 bag(`s3_pilot_a`)의 기동 확인 재생
(아래 "기동 확인")과 slam.md §6.3 의 배포 측정이다.

- **P-L1** (정답 지도, RTAB-Map 위치추정 − 배포 2D, xy ATE RMSE): `A<B` (3D 가 낫다). 근거: 파일럿에서 RTAB-Map 위치추정 0.67 cm,
  배포 사슬 1.72 cm (완전한 지도 위; slam.md §6.3 의 배포 측정 1–2 cm). 확신 중간.
- **P-L2** (SLAM 지도): `A<B`. 확신 낮음 — slam_toolbox 지도(E2)의 국소 어긋남이 2D 에 더해진다.
- 명세 구간: RTAB-Map 위치추정은 7 bag × 3 재생 모두 RMSE·max 둘 다 3 / 5 / 8 cm 통과, 배포 2D 는 RMSE 통과·max 는 일부 실패
  (slam.md §6.3 의 정지 max 4.23 cm 처럼).
- 실패: 두 행 모두 발산 0. 배포 2D 의 kidnap_monitor LOST 0.
- **P-E5L**: 정답 지도 위 배포 2D 의 wall − open 통로 축 RMSE 차는 `undecided` (δ 0.2 cm 미만) — 5.6 m 짧은 경로에서 휠-EKF
  odom(사실상 오라클)이 축 방향을 붙든다. 3D 도 `undecided`. 차의 차도 δ 미만 → **가설은 위치추정 쪽에서도 지지되지 않을 것으로
  예측**한다 (기하 계산의 퇴화가 오차로 드러나려면 휠 열화(요인 D)가 필요하다 — 이번 범위 밖).
- **P-E4** (기준 = E2 `rtabmap3d_noimu`, 같은 bag 4 개):
  - 복셀 0.05: icp 지연 p50 ↑ (2 배 이상), ATE `undecided`. 0.2·0.3: 지연 ↓, ATE 는 0.3 에서 `differs`(+) 가 bag 절반 이상.
  - 점-대-점: ATE ↑ (`differs` ≥ 3/4 bag) — 정형 환경에서 평면 잔차가 낫다는 보고서 근거.
  - 높이 대역 1.0·2.0: 궤적 같음(`undecided`), 단면 GT 대비 지도 정밀도 ↓·대역 GT(0.05–2.0) 대비 재현율 변화 ≤ 0.01 — 이 월드에
    돌출물이 없다 (evaluation.md §3).
  - 루프 거부 1.0·3.0: 모두 `undecided` (LiDAR 단독 RTAB-Map 은 근접 루프만, 거부가 드물 것).
  - 비유한 점 제거 끔(`points_raw`, 조직 16 × 1800, +inf 13.7 %): 확신 낮음 — RTAB-Map 이 비유한 점을 스스로 거르면 `undecided`,
    아니면 오류·실패. 결과를 그대로 적는다.

## 설정

### E3(b) 행 (`rows.py`, 사본 `ws_e3b`)

| 행 | 사슬 | 지도 (읽기 전용 `/data/map`) | 초기 자세 |
|---|---|---|---|
| `amcl_gt` | **배포 2D 위치추정 사슬** (`amcl_loc.launch.py chain:=deploy`): map_server + amcl_map_adapter + nav2_amcl(amcl.yaml, `tf_broadcast: false`) + ekf_filter_node_map(config/ekf.yaml: 휠 odom·IMU 각속도·가속도·amcl_pose·scan_match_pose) + scan_matcher_node + kidnap_monitor_node (회전 명령 없음). 전처리 출력(scan_filtered·wheel_odom·imu/data·odometry/filtered·EKF odom TF)은 bag | **완전한 지도** (`maps/perfect_r0.050.yaml`, `peval.py perfect-map`): GT 0.01 m 단면(z 0.20, sdf sha 2efa665e… = 커밋 SDF)의 **면이 지나는** 0.05 m 셀만 점유, 고체 안 미지, 나머지 자유 (evaluation.md §2 의 "완전한 지도" — 셀 중심이 면에서 ±2.5 cm 에 고르게, 평균 편향 0). sha256 pgm `b222c52a…`, yaml `7efee45e…` | 이 bag 의 스폰 (AMCL `initial_pose` — 배포와 같은 정보) |
| `amcl_slam` | 같음 | E2 의 slam_toolbox map_long 지도 (선택 규칙 — E2 항목) | 같음 |
| `rtabmap3d_loc_gt` | RTAB-Map 3D 위치추정: 구성 B 오도메트리(icp_odometry + 휠-EKF guess, E2 와 같음) + rtabmap `Mem/IncrementalMemory false`, `Mem/InitWMWithAllNodes true`, `RGBD/StartAtOrigin true` | **정답 자세 DB**: map_long 의 점군 장들을 정답 자세(ground_truth/odom → `gt_odom` TF)에 놓고 근접·루프 검출을 끈 RTAB-Map DB (`rtabmap3d_gtdb`) | `map → slam_map` = map_long 스폰, initialpose = 이 bag 스폰을 slam_map 으로 옮긴 것 (구독 확인 뒤, 재생 전) |
| `rtabmap3d_loc_slam` | 같음 | E2 의 rtabmap3d_noimu map_long DB (선택 규칙) | 같음 |

- **정답 지도의 비대칭 (주장 범위에 적는다)**: 2D 정답 지도는 기하 래스터(잡음 없음), 3D 정답 지도는 **센서 장들**(σ 3 cm)을 정답 자세에
  놓은 것이다. RTAB-Map 위치추정은 장 대 장 정합이라 기하 래스터를 지도로 받을 수 없다. 그래서 "정답 지도 위" 는 두 행에서 "자세 오차
  없는 지도" 라는 같은 뜻이지만 지도 표현은 다르다.
- 같은 bag 의 장으로 지도를 만들지 않는다 (지도 = map_long, 평가 = 나머지 7 bag) — 같은 장끼리 정합하는 순환을 피한다. S6 세 bag 은
  다른 월드(actor 4·차량 2, 블로커)지만 지도는 정적 s1 월드의 map_long 이다 — 블로커는 사전 지도에 없는 물체다 (evaluation.md §6.1).
- 재생: 7 bag × 행 4 × 재생 3 = 84 + 정답 자세 DB 1. 배속 = 0.25 / 녹화 RTF (E2 와 같다). DOMAIN 46. 컨테이너
  `amr-ai-b2-eval-e3-…`. 사전 등록 신호 파일 `go/e3` 를 이 항목 커밋 뒤에 만든다 (그 전에는 구동기가 집지 않는다).
- 2D 보조 (기술만, 검정 밖): 같은 재생의 out_bag 에 `amcl_pose`(AMCL 단독 추정)·`scan_match_pose`·`odometry/filtered_map`·
  `localization/lost`·`kidnap_status` 를 기록한다.

### E3(b) 지표·판정 (`peval.py replays --stage e3 --window-stage e3` → `compare-loc`)

- 궤적 = probe 의 `traj_online.tum` (입력 스탬프마다 map → base_footprint — 2D 는 map EKF 의 map→odom, 3D 는 rtabmap 의
  slam_map→icp_odom 경유). 창 = 그 bag 의 E3(b) 재생 중 GT 의 90 % 이상을 덮는 것들의 공통 구간에서 **처음 2.0 s 를 뺀 것**
  (`--warmup 2.0` — 배포 map EKF 는 첫 amcl_pose 전 원점에 있다; 두 사슬 같은 규칙, 모든 bag 은 처음 ≥ 10 s 정지). 지표는 E2 와 같다 (정렬 없는 xy ATE
  RMSE·max·p95 가 주, 명세 구간 RMSE·max·p95, 통로 축·횡, SE(2) 정렬은 보조).
- **위치추정 실패** (`peval.loc_fail_of`): rc ≠ 0, 덮음 < 90 %, RTAB-Map 처리 스캔 판정 누락 > 0 (E2 와 같은 대응), 2D 는
  "Message Filter dropping" > 0 (대용 — AMCL 은 처리 수 근거가 없다), 치명 패턴 > 0, **발산** = 창 안 xy 오차 최대 > 50 cm 또는 yaw
  최대 > 10°, **LOST** = RTAB-Map null odom(첫 장 제외) > 0. 2D 의 kidnap_monitor LOST 는 out_bag 의 `localization/lost` 로 따로 센다.
  실패율 Wilson 95 %. 실패 재생은 평균에서 빼지 않는다.
- 가족 (각자 비교 하나 — Holm 없음, α 0.05): **L1** `rtabmap3d_loc_gt − amcl_gt`, **L2** `rtabmap3d_loc_slam − amcl_slam`,
  지표 `ate_rmse_cm`, δ 0.2 cm. 짝·검정·구간·판정은 E2 와 같다 (bag 차 Wilcoxon 정확, 두 단계 부트스트랩, `verdict`).
  bag 7 (가능 최소 p 0.0156) 과 사전 등록 6 bag (s1_seed2x 제외, 0.031) 둘 다 — **기각 가능한 설계** (모든 bag 이 같은 부호일 때만).
  둘의 판정이 다르면 6 bag 이 주 결과 (S1 규칙). S1 만 (3·2 bag) 은 기각 불가능 — 기술만.
- 명세 구간 표: 행 × 구간(정지 3 / 직선 5 / 회전 8 cm)마다 RMSE·max·p95 의 bag 평균의 평균, 가장 나쁜 재생, 기준 통과 재생 수
  (RMSE·max·p95 따로). 판정은 "통과 재생 수 / 전체" 를 그대로 적는다 (평균만으로 통과라 쓰지 않는다).
- 지도의 몫 (기술): (SLAM 지도 − 정답 지도) xy ATE RMSE, 같은 bag·재생 짝, 두 단계 부트스트랩 구간.

### E5 (위치추정 쪽) — S6 세 bag

- 주 = 정답 지도 위 두 행 (`amcl_gt`, `rtabmap3d_loc_gt` — 사전 지도에 블로커가 없는 조건, 기하 계산과 같은 가정). 보조 = SLAM 지도 위 두 행.
- 지표 `aisle_axis_rmse_cm` (보조: lateral, axis max, 전체 ATE). 행마다 wall − open, queue − open 을 `pstats.replay_rule`
  (평균 차 > max(2 × 합동 재생 sd, δ 0.2 cm) → `differs`). 차의 차 (2D − 3D) 를 같은 δ 로 기술.
- 가설 지지 = 2D 의 wall − open 이 `differs`(+) 이고 3D 는 `undecided` 이며 차의 차 > δ. 그 밖은 "지지 안 됨" (불리해도 그대로).
  bag 이 변형마다 하나라 일반화하지 않는다.

### E4 절제 (`rtabmap3d_abl_*`, 사본 `ws_e3b`)

| 손잡이 (보고서 표) | 행 | 바꾼 것 (기준 `config/rtabmap.yaml`) |
|---|---|---|
| 다운샘플링 | `vox005` · `vox02` · `vox03` | `Icp/VoxelSize` (+ icp_odometry `scan_voxel_size`) 0.1 → 0.05 · 0.2 · 0.3 |
| 정합 오차 척도 | `p2p` | `Icp/PointToPlane` true → false |
| 지면·높이 대역 | `obs10` · `obs20` | `Grid/MaxObstacleHeight` 0.40 → 1.0 · 2.0 |
| 그래프 최적화·루프 검증 | `opt10` · `opt30` | `RGBD/OptimizeMaxError` 0.3 → 1.0 · 3.0 |
| 입력 위생 | `raw` | 입력을 전처리 `lidar3d/points` 대신 원본 `lidar3d/points_raw` (조직 16 × 1800, +inf 포함) |

- 파일 `config/ablation/<손잡이>.yaml` 은 한 손잡이만 담는다 (시험 `test_ablation_rows_change_exactly_one_knob`).
- bag 4 개 = S3 (`s3_aisle_a1`) + S1 셋 (`s1_seed1_a1`·`s1_seed3_a2`·`s1_seed2x_a1` — S1 규칙대로 3 bag·2 bag 둘 다 표시), 재생 2 회
  (r0·r1), 행 9 → 72 재생. DOMAIN 48. 기준 = E2 의 `rtabmap3d_noimu` 같은 bag 재생 3 회 (같은 창 — E2 창).
- 지표: E2 와 같은 궤적·지도·3D 점 지표 + `--grid-band` (대역 0.05–2.0 m GT 대비 지도 지표, `_band`) + 스캔당 지연 (icp_odometry
  "update time" p50·p95, rtabmap "RTAB-Map=" p50 — **공유 부하 하 예비값**, 기준과 다른 시각에 쟀다).
- 판정 (기술 — 4 bag 이라 검정하지 않는다, 가능 최소 p 0.125): bag 마다 `replay_rule` (δ: ATE·ADNN 0.2 cm, RPE 0.05 %p, 비율 0.01,
  지연 p50·p95 5 ms, rtabmap 10 ms). 손잡이가 지표를 "바꾼다" = 4 bag 중 ≥ 3 이 같은 방향 `differs`. 그 밖은 "가르지 못함".
- 사전 등록 신호 파일 `go/e4`.
- E4 처리 사슬 문서화 (결과 문서): 비유한 점 제거·frame_id (amr_lidar3d 전처리), 평면 고정 (`Reg/Force3DoF`, LIO-SAM SE(2) clamp),
  madgwick 방위 (잠금 단계 파생 bag), 복셀·법선 k·대응 거리 — 코드·설정의 실제 값으로.

### 동시 수

E2 항목의 규칙이 01:25 에 발동해 k = 2 / 슬롯 (동시 4) 이다 (map_long rtabmap3d 세 재생 누락 7–12 장). E3(b)·E4 도 같은 k 와 부하
규칙(다른 부하 ≤ 48)으로 돈다 — 단계마다 조건을 같게. 더 줄이는 규칙(처음 끝난 E3(b)·E4 재생 12 개 중 누락 > 0 이 3 개 이상이면 k 1)만 둔다.

### 기동 확인 (파일럿 bag `s3_pilot_a` — P 데이터 밖, `pilot/`) [측정, 공유 부하 하]

모든 기동 확인은 E3(b)·E4 의 bag 이 아닌 파일럿 bag(b2-offline·s3 이 쓰던 `s3_pilot_a`, 9.5 s 기록 결함 있음)이나, 메커니즘만
본 S6 한 번이다. 결과 수치는 기동·설계 판단에만 썼고 P 데이터에 넣지 않는다. 원자료 `runs/b24/b2-eval/pilot/`, 지표
`validate/out/pilotloc{,2,3}.jsonl` (peval.py replays).

| 확인 | 결과 | 결정 |
|---|---|---|
| 정답 자세 DB (`rtabmap3d_gtdb`, s3_pilot_a, 배속 0.85) | rc 0, 139 노드 / 276 링크, DB 궤적 = GT (xy 오차 0.0 cm, 1 788 표본) | 정답 지도 생성 방식 확정 |
| RTAB-Map 위치추정 (그 DB, 배속 0.85 = 유효 0.5) | rc 0, xy RMSE 0.67 cm, **처리 누락 27 장** (부하 75–111) | 본 실행은 E2 와 같은 유효 0.25 |
| AMCL 단독 (`tf_broadcast true`, **채운** GT 단면 래스터) | xy RMSE 5.70 / max 9.52 cm — 출발 정지 중 0, 움직이며 4–9 cm 로 커짐 | 배포 출력이 아니다 (배포는 map EKF + scan_matcher) → 2D 행을 **배포 사슬**로 바꿈 |
| 배포 사슬, 채운 GT 단면 래스터 | 첫 표본이 map EKF 원점(24 m, 첫 amcl_pose 전), 그 뒤 **정지 중에도 7 cm** 일정 오차 | 원인: scan_matcher 의 면 점(자유와 닿은 점유 셀 주변 0.12 m 점유 셀 평균 = "띠 중심선", slam.md §4.3)이 채운 고체에서는 고체 안쪽으로 밀린다 → 2D 정답 지도를 **완전한 지도**로 바꿈, 평가 창에서 처음 2 s 를 뺀다 (`--warmup 2.0`, 두 사슬 같게) |
| 배포 사슬, 완전한 지도 (배속 0.426 = 유효 0.25, 부하 25–178) | xy RMSE **1.72** / max 4.15 cm, 정지 1.82 / 2.86, 직선 1.63 / 3.38, 회전 1.99 / 4.15 cm, 실패 0 | slam.md §6.3 의 배포 측정(1–2 cm)과 같은 수준 — 채택 |
| RTAB-Map 위치추정 initialpose (s6_open_a3, 정답 자세 DB, 메커니즘만) | initialpose (7.40, 18.00, π) in slam_map = 스폰 (0, −8.6, −π/2), 첫 궤적 자세 (0.000, −8.599), 처리 누락 0 / 840 (부하 166–178) | S6 의 통로 안 출발이 지도 프레임에 맞게 시작한다. 이 재생의 정확도 수치는 보지 않았고 데이터에 넣지 않는다 |

**정직성 메모**: 2D 행의 사슬(AMCL 단독 → 배포 사슬)과 2D 정답 지도(채운 래스터 → 완전한 지도)는 **파일럿 결과를 본 뒤** 바꿨다.
둘 다 파일럿 bag 에서 2D 가 배포 문서의 성능에 못 미친 원인을 찾아 고친 것이고, E3(b) bag 의 결과는 보기 전이다. 바꾸지
않았다면 2D 는 채운 래스터의 설계 편향(약 5–7 cm)을 안고 비교됐을 것이다 — 그 편향은 "2D 대 3D" 가 아니라 지도 표현의 문제다.


### 시간 어림

E3(b) 85 재생 ≈ 직렬 14 h → 동시 4 면 약 3.5 h (+ 부하 대기). E4 72 재생 ≈ 직렬 17.5 h → 약 4.4 h. E2 뒤 E3(b) → E4 순서
(우선순위 45·60). 분석 약 1 h.

### 이어받기 뒤 실행 중 기록 (2026-10-13, E3(b)·E4 재생 0 건 상태)

- 구동기가 2026-10-09 07:07 에 끊겨 E3(b) 는 정답 자세 DB 1 건만, E4 는 0 건이었다 (E2 항목 「이어받기」 절). 2026-10-13 22:04 에 다시 띄웠다.
- **도구 추가 (E3(b) 결과 전)**: 위 「위치추정 실패」 의 "2D 의 kidnap_monitor LOST 는 out_bag 의 `localization/lost` 로 따로 센다" 를
  셀 도구가 없었다 (peval.py 에 없음). `src/amr_slam3d/scripts/loc_lost.py` (sha256 `c4859fa8…`) — 재생마다 `localization/lost` 메시지 수·
  True 수·False→True 전이(LOST 사건) 수, `kidnap_status` 마지막 상태, `amcl_pose` 수. 토픽이 없으면 None(미측정). 정의는 이 항목의 문구
  그대로이고 판정(L1·L2)에는 들어가지 않는 별도 셈이다. 시험 `test_loc_lost.py` 2 개. 파일럿 bag(P 데이터 밖) 배포 사슬 두 재생에서
  lost 1 메시지(False)·사건 0·마지막 상태 TRACKING·amcl_pose 513 으로 돈다 (`validate/` 와 같은 범주의 도구 확인).
- **결과 표 도구 (E3(b)·E4 결과 전)**: `src/amr_slam3d/scripts/presults.py` — peval 출력(json·jsonl)을 마크다운 표로 옮기기만 한다
  (판정·구간·p 는 peval 값 그대로). 더하는 해석 하나: **E4 의 사전 등록 3 bag 표시**(s1_seed2x 제외 — S1 규칙)에서 "바꾼다" =
  **3 bag 모두 같은 방향 `differs`** (4 bag 규칙 "≥ 3/4" 의 비율을 3 bag 에 옮긴 것). 4 bag 판정이 주이고, 둘이 다르면 S1 규칙대로
  3 bag 쪽을 주로 쓴다. 합성 자료(파일럿 기록의 bag 이름만 바꾼 것 — 결과 아님)로 표 모양만 확인했다.
- **E3(b)·E4 결과 전에 더하는 보충 분석 규칙 (2026-10-14 11:40, 사전 등록 외 — E2 중간 결과를 본 뒤 정함)**: E2 중간 분석에서
  휠-EKF guess 를 쓰는 행의 오차가 세 bag 에서 1–2 m 로 커진 것을 보고 원인을 찾았다 (E2 항목 「EKF 결함 bag」). 녹화 때
  `imu_filter_node` 가 기동 바이어스 추정에서 **"motion detected during startup bias estimation; falling back to parameter bias"** 를
  낸 bag (`s1_seed3_a2`·`s1_seed2x_a1`·`s6_wall_a1`, 녹화 `launch.log`) 은 자이로 z 바이어스(시뮬 ±0.01 rad/s)가 빠지지 않아 bag 의
  EKF `odometry/filtered`·odom TF 가 0.01 rad/s 로 yaw 표류한다 [측정]. E3(b) 의 두 사슬(배포 2D 는 EKF odom TF·`odometry/filtered`,
  RTAB-Map 위치추정은 EKF guess)과 E4 의 모든 행이 그 odom 을 쓴다. 그래서 **E3(b)·E4 결과를 보기 전에** 다음을 정한다:
  - 주 결과는 사전 등록 그대로 (E3(b) 7·6 bag, E4 4·3 bag). 바꾸지 않는다.
  - **보충 표**: 같은 지표·판정 규칙을 "EKF 결함 bag 제외" 집합으로도 낸다 — E3(b) 4 bag (`s3_aisle_a1`·`s1_seed1_a1`·`s6_open_a3`·
    `s6_queue_a1`, 가능 최소 p 0.125 → **기각 불가능한 설계, 기술만**), E4 2 bag (`s3_aisle_a1`·`s1_seed1_a1`, 기술만), E5(위치추정) 는
    wall 이 결함 bag 이라 **wall − open 비교가 EKF 결함과 섞인다**고 표시한다 (queue − open 은 영향 없음).
  - 결함 판정 기준은 위 로그 한 줄 (SLAM 결과와 무관한 녹화 기록) 이다. s3_aisle 은 바이어스를 추정했지만 0.01048 rad/s (다른 통과 bag
    0.00999–0.01001) 로 잔차 4.8e-4 rad/s → EKF yaw 오차 최대 3.0° (EKF xy 33 cm) 라 "부분" 으로 표시만 하고 제외하지 않는다.

## 결과

### E3(b)·E5(위치추정) — 2026-10-14 23:21 분석 (84 재생 + 정답 자세 DB 1, 모두 rc 0) [측정, 공유 부하 하]

원자료 `runs/b24/b2-eval/e3/`, 지표 `analysis/e3/replays.jsonl`·`compare_loc.json`, 표 `analysis/e3/tables/e3.md` (presults.py),
LOST 셈 `analysis/e3/lost.jsonl`, 보충 `analysis/supp/e3_clean.md`. 분석 HEAD `05d7828` (peval.py 는 사전 등록 판 `dfe20075…` 그대로).
재생 기간 2026-10-14 10:53–22:55, 시작 1 분 loadavg 11.5–50.4 (`job.json` `load_start`) 〔리뷰 반영 정정 2026-10-15 — 처음 문장은 아래 「리뷰 반영」 절〕.

**불리한 것 먼저**
- **배포 2D 사슬이 9/42 재생에서 발산했다** (창 안 xy 최대 > 50 cm): `s1_seed3_a2` 정답 지도 3/3 (ATE RMSE 14.7–18.5 m),
  `s6_wall_a1` 정답·SLAM 지도 6/6 (31–34 m). 두 bag 모두 **EKF 결함 bag** 이다. RTAB-Map 위치추정은 발산 0/42.
- **RTAB-Map 위치추정의 처리 스캔 누락 실패가 19/42** (정답 9, SLAM 10). 그중 `s1_seed3_a2` r1 은 두 행 모두 **727 장 누락**
  (정답 지도 행은 궤적 덮음 0.68) — **처리가 멈춘 사건이 아니다** 〔리뷰 반영 정정 2026-10-15 — 처음 문장은 아래 「리뷰 반영」 절〕: 처리 로그 기준 누락은 92 장(두 컨테이너가 같은
  벽시계 19:08:37–19:09:15 에 38 s 함께 멈춤 — 같은 시각 호스트 메모리 고갈, 커널 OOM 이 다른 사용자 openroad 를 죽임)이고, 정지 뒤
  icp_odometry 는 재생 끝까지 돌았지만 **기록기(두 행)의 icp_odom, 정답 지도 행 탐침의 점군, 정답 지도 행 RTAB-Map 위치추정 노드의 입력**
  구독이 복구되지 않았다 — 정답 지도 행의 위치추정 노드는 sim 163–235 s 동안 갱신 0 (갱신 줄 224/152/224), SLAM 지도 행은 8 회를 잃고 다시
  돌았다 (216/224) [측정, `supp/stall_1909/`, `recheck/b1/`; 리뷰 반영 2 회차]. 기제는 [추정]. 실패 판정은 규칙(큰 값)대로 둔다. 나머지는 시작 첫
  장만(s3_aisle 11.5 s, s6_wall 10.7–10.9 s — E2 와 같은 bag 결정적 사건) 6 재생과 주행 중 1–6 장 11 재생.
- 2D 의 "Message Filter dropping" 1 이 s6_wall 6 재생에서 났다 (그 재생들은 발산으로도 실패).

**사전 등록 가족 판정** (차 = 3D − 2D, xy ATE RMSE, δ 0.2 cm):

| 가족 | 7 bag (ALL7) | 사전 등록 6 bag (PRE6) | 주 결과 (S1 규칙: 다르면 6 bag) |
|---|---|---|---|
| L1 정답 지도 | −673.7 cm [−1580.7, −0.42], 음 6 / 양 1, p 0.031 → `A<B` | −785.8 [−1824.3, −0.25], 5/1, p 0.0625 → `undecided` | **가르지 못함** |
| L2 SLAM 지도 | −487.4 [−1459.4, −1.14], 7/0, p 0.0156 → `A<B` | −568.6 [−1702.4, −1.41], 6/0, p 0.031 → `A<B` | **`A<B` (3D 가 낫다)** |

- 평균 차의 크기는 거의 전부 EKF 결함 bag 의 2D 발산(−1600 cm·−3114 cm·−3403 cm)이 만든다. 결함 없는 bag 의 bag 차는
  L1: s3_aisle +0.11, s1_seed1 −0.32, s6_open −0.56, s6_queue −0.48 cm, L2: −2.45, −0.93, −2.09, −1.90 cm.
  **보충(결과 전 규칙)** 결함 bag 제외 4 bag: L1 −0.31 [−0.53, −0.03] cm (3/1), L2 −1.84 [−2.32, −1.22] cm (4/0) — 기각 불가능한
  설계(가능 최소 p 0.125)라 기술만.
- 행별 xy ATE RMSE (결함 제외 4 bag, bag 평균의 평균): 2D 정답 1.15 · 3D 정답 0.84 · 2D SLAM 2.78 · 3D SLAM 0.94 cm.

**명세 구간 (팀 기준 정지 3 / 직선 5 / 회전 8 cm, 통과 재생 수 — RMSE · max · p95)** — 7 bag 21 재생씩. 회전 구간은 S6 bag 에 없어
(통로 안 직진·후진만) peval 의 셈에서 NaN 이 "통과 아님" 으로 들어간다 → 회전은 회전이 있는 12 재생을 분모로 다시 적는다:
- `rtabmap3d_loc_gt`: 정지 21/21 · 15/21 · 15/21, 직선 21/21 · 21/21 · 21/21, 회전 12/12 · 12/12 · 12/12. 최악 max 정지 3.89 cm.
- `rtabmap3d_loc_slam`: 정지 21/21 · 12/21 · 12/21, 직선 21/21 · 18/21 · 21/21, 회전 12/12 · 12/12 · 12/12.
- `amcl_gt`: 정지 12/21 · 12/21 · 13/21, 직선 15/21 · 11/21 · 12/21, 회전 9/12 · 6/12 · 7/12 (발산 재생 포함).
- `amcl_slam`: 정지 6/21 · 0/21 · 0/21, 직선 18/21 · 3/21 · 12/21, 회전 12/12 · 12/12 · 12/12.
- 결함 제외 4 bag (보충): `amcl_gt` 정지·직선 RMSE 12/12, max 정지 12/12 · 직선 11/12 (s3_aisle r0 12.89 cm), `amcl_slam` 정지 max
  0/12 (최악 5.48 cm — SLAM 지도 위 2D 는 정지 중 3 cm 를 넘는다), `rtabmap3d_loc_gt` 정지 max 9/12 (최악 3.24 cm).

**위치추정 실패·LOST** — 실패 재생 (Wilson 95 %): `amcl_gt` 6/21 [0.14, 0.50] (발산 6), `rtabmap3d_loc_gt` 9/21 [0.24, 0.63] (누락 9,
덮음 1), `amcl_slam` 3/21 [0.05, 0.35] (발산 3), `rtabmap3d_loc_slam` 10/21 [0.28, 0.68] (누락 10). 배포 kidnap_monitor LOST 사건
(`loc_lost.py`): `amcl_gt` 63 사건 / 10 재생 (s1_seed2x 8–13·s1_seed3 7–9·s6_wall 1–2·s3_aisle r0 1), `amcl_slam` 18 / 9 (s6_wall 4,
s1_seed3·s1_seed2x 1). 결함 없는 S1·S6 bag 은 0. RTAB-Map null odom(LOST) 0.

**지도의 몫 (SLAM 지도 − 정답 지도, 기술)**: 3D +0.38 [0.06, 0.90] cm. 2D 는 발산 재생 때문에 평균이 뜻이 없다 (−186 [−684, 124]);
결함 없는 4 bag 은 +2.58·+1.01·+1.53·+1.42 cm (평균 +1.64 cm) — 2D 는 지도 품질의 몫이 3D 보다 크다 (기술).

**E5 (위치추정, 주 = 정답 지도)** — 통로 축 RMSE [cm] 재생 3:
| 행 | open | queue | wall | queue − open | wall − open |
|---|---|---|---|---|---|
| `amcl_gt` | 0.71 / 0.75 / 0.72 | 0.65 / 0.65 / 0.65 | 1059.6 / 1074.5 / 1059.6 | −0.08 `undecided` | **+1063.8 `differs`** |
| `rtabmap3d_loc_gt` | 0.18 ×3 | 0.18 ×3 | 0.17 ×3 | −0.00 `undecided` | −0.01 `undecided` |
- 사전 등록 규칙(2D wall − open `differs`(+), 3D `undecided`, 차의 차 1063.8 > δ)으로는 **"지지"** 다. 그러나 **wall bag 은 EKF 결함 bag**
  이다 (E3(b)·E4 결과 전에 "wall − open 이 EKF 결함과 섞인다" 고 적었다). 시간 분해 [측정, `e3/s6_wall_a1/amcl_*/r0/traj_online.tum`]:
  2D 는 **출발 전 정지 중, 첫 출력(10.8 s) 뒤 약 2 s 만에** 0.1 cm 에서 한 번에 21 m 로 뛰었고 (t 12.9 s — r0·r2, 13.0 s — r1) 〔리뷰 반영 정정 2026-10-15 — 처음 문장은 아래 「리뷰 반영」 절〕,
  kidnap_monitor 가 시작부터 LOST → RECOVERING 에 머물렀다. 그때 EKF yaw 표류는 약 1.2° (첫 자세 정렬) 뿐이라, 방아쇠는 사전 지도에 없는 양끝 화물과 스캔의 불일치일 가능성이 크다 **[추정]**. 3D 는 같은 bag 에서
  EKF 표류(최대 48°)와 화물을 모두 견디고 0.17 cm 였다. 현실적 막힘(queue)에서는 2D·3D 모두 열림과 가르지 못했다 (2D 0.65 vs 0.72 cm).
  bag 이 변형마다 하나라 일반화하지 않는다.
- 예측 P-E5L ("가설은 위치추정 쪽에서도 지지되지 않을 것") 은 **틀렸다** — 단 위 혼입 때문에 "막힘이 2D 를 깨뜨린다" 를 이 자료만으로
  주장할 수 없다.

**예측 대조**: P-L1 (`A<B`) — 7 bag 에서는 맞고 주 결과(6 bag)에서는 가르지 못함. P-L2 (`A<B`, 확신 낮음) — 맞음. 명세 구간 예측
("RTAB-Map 은 RMSE·max 모두 통과, 2D 는 max 일부 실패") — RTAB-Map 정지 max 가 6/21 재생에서 3 cm 를 넘어 **틀렸다** (최악 3.89 cm).
"두 행 모두 발산 0" — 2D 9 발산으로 **틀렸다**. "kidnap LOST 0" — **틀렸다** (결함 bag 과 s3_aisle r0).

### E4 절제 — 2026-10-15 08:54 분석 (72 재생 모두 rc 0, 재생 2026-10-15 01:47–08:27, 시작 loadavg 평균 42) [측정, 공유 부하 하]

원자료 `runs/b24/b2-eval/e4/`, 지표 `analysis/e4/replays.jsonl`·`compare_abl.json` (기준 = E2 `rtabmap3d_noimu` 같은 bag 재생 3,
같은 E2 창), 표 `analysis/tables/e4.md`. 분석 HEAD `f12a515` (peval.py 사전 등록 판 그대로).

**불리한 것 먼저**
- **bag 4 개 중 2 개(`s1_seed3_a2`·`s1_seed2x_a1`)가 EKF 결함 bag** 이다 — 그 bag 에서 모든 행이 ATE 100–220 cm 라 손잡이 효과가 결함에
  묻힌다. 결함 없는 bag 은 `s3_aisle_a1`·`s1_seed1_a1` 둘뿐 (E4 결과 전 정한 보충 규칙대로 기술만).
- **지연 비교는 측정 시각 혼입으로 사전 등록 판정을 믿을 수 없다**: icp_odometry 에 닿지 않는 손잡이(`Grid/MaxObstacleHeight`·
  `RGBD/OptimizeMaxError` — rtabmap 노드에만 들어간다)도 기준 대비 icp p50 −14…−17 ms, p95 −41…−48 ms 였고, obs10·obs20 의 icp p95 는
  규칙상 "바꾼다(−) 3/4" 가 나왔다. 기준(E2)은 다른 시각·다른 동시 재생 조건에서 쟀다 (사전 등록에 "기준과 다른 시각" 이라 적은 위험).
- 모든 절제 행이 실패 2/8 — 결함 bag 이 아닌 `s3_aisle_a1` 의 시작 첫 장 누락(11.5 s, 결정적)이다.

**사전 등록 판정** — "바꾼다" = 4 bag 중 ≥ 3 같은 방향 `differs`, 사전 등록 3 bag(s1_seed2x 제외)은 3/3 (결과 전에 정한 해석);
둘이 다르면 3 bag 이 주 결과. 차 = 절제 − 기준.

| 손잡이 (행) | 궤적 | 2D 격자 지도 | 3D F@5 | 지연 (기준 대비 — 혼입) |
|---|---|---|---|---|
| 점-대-점 (`p2p`) | **ATE RMSE·max·RPE 1 m 나빠짐** (4/4·3/3): s1_seed1 1.82 → 506 cm, s3_aisle 1.33 → 10.1 cm | 4 bag 은 F1·재현율 −, ADNN₀ + (3/4) / 3 bag 가르지 못함 → 가르지 못함 | 4 bag − (3/4) / 3 bag 가르지 못함 | 가르지 못함 |
| 복셀 0.05 (`vox005`) | **ATE max 나빠짐** (4/4·3/3; s3 2.46 → 6.00, s1_seed1 3.83 → 9.08 cm). ATE RMSE 4 bag + 3/4 / 3 bag 가르지 못함 | 가르지 못함 | 가르지 못함 | 가르지 못함 |
| 복셀 0.2 (`vox02`) | **RPE 1 m 좋아짐** (3/4·3/3). ATE·max 4 bag − 3/4 / 3 bag 가르지 못함 (s3 1.33 → 1.02, s1_seed1 1.82 → 1.23 cm, s1_seed3 +0.53) | ADNN₀ 4 bag − 3/4 / 3 bag 가르지 못함 | 가르지 못함 | icp p95 − (4/4·3/3) |
| 복셀 0.3 (`vox03`) | **ATE max 나빠짐** (4/4·3/3; +0.8–2.0 cm) | 가르지 못함 | 가르지 못함 | icp p50·p95 − (4/4·3/3) |
| 높이 대역 1.0 (`obs10`) | 가르지 못함 (차 0.000) | **ADNN₀ +** (4/4·3/3, 3.8 → 13.0 cm), **IoU +** (3/4·3/3, 0.41 → 0.64), 대역 GT 도 같음 | 가르지 못함 | icp p95 − 3/4 / 3 bag 가르지 못함 |
| 높이 대역 2.0 (`obs20`) | 가르지 못함 | **ADNN₀ +, IoU +, 재현율 +** (4/4·3/3; 3.8 → 14.7 cm, 0.41 → 0.74, 0.72 → 0.83) — 대역 GT 도 같음 | 가르지 못함 | icp p95 − 3/4 / 3 bag 가르지 못함 |
| 루프 거부 1.0 (`opt10`) | 가르지 못함 (s3_aisle ATE −0.22, 한 bag) | 가르지 못함 | 가르지 못함 | 가르지 못함 |
| 루프 거부 3.0 (`opt30`) | 가르지 못함 (차 0.000) | 가르지 못함 | 가르지 못함 | 가르지 못함 |
| 비유한 점 제거 끔 (`raw`) | ATE 4 bag + 3/4 / 3 bag 가르지 못함 → **가르지 못함** (결함 없는 두 bag 은 1.33 → 3.78, 1.82 → 5.50 cm; s1_seed3 −1.7) | 가르지 못함 | 가르지 못함 | 가르지 못함 |

- `raw` 행은 실패하지 않았다 (rc 0, 누락은 시작 장만) — RTAB-Map 은 +inf 가 섞인 조직 점군을 받아 돌았다.
- 높이 대역: 정밀도 0.72 → 0.46·0.38 (s3_aisle) 로 내려가고 IoU 는 올라간다 — 지표 방향이 엇갈려 "지도가 나아졌다/나빠졌다" 고 쓰지 않는다.
  대역 GT(0.05–2.0 m) 대비 ADNN₀ 도 같은 만큼 커진 이유는 확인하지 않았다.

**보충(사전 등록 외, 결과 뒤) — 같은 시각의 "무관 손잡이" 를 기준으로 한 지연**: icp_odometry 에 닿지 않는 obs10·obs20·opt10·opt30
네 행(같은 E4 기간·같은 bag)의 평균을 기준으로 다시 쟀다. icp p50 / p95 차 (4 bag 평균, bag 4 개 모두 같은 부호): 복셀 0.05 **+20.7 / +24.4 ms**,
0.2 **−22.0 / −24.0**, 0.3 **−30.2 / −34.0**, 점-대-점 −7.1 / −7.3, 비유한 점 제거 끔 +5.6 / +6.6 ms (기준 icp p50 62.8–71.6 ms). 복셀 크기에 단조로운
속도 곡선이다. 공유 부하 하 예비값.

**예측 대조** (P-E4): 점-대-점 ATE ↑ (`differs` ≥ 3/4) — **맞음** (4/4, 한 bag 은 5 m 발산). 복셀 0.05 지연 2 배 — **틀림** (+30 %), ATE
`undecided` — RMSE 는 주 결과 가르지 못함이지만 max 는 나빠짐. 0.2·0.3 지연 ↓ — 맞음; 0.3 ATE `differs`(+) bag 절반 이상 — RMSE 3/4 `differs`(+)
로 맞음 (주 판정은 max). 높이 대역 궤적 같음 — 맞음; 단면 GT 정밀도 ↓ — 맞음; 대역 GT 재현율 변화 ≤ 0.01 — obs20 **틀림** (+0.06).
루프 거부 `undecided` — 맞음. 비유한 점 — 오류 없이 돌았다 (예측 둘 중 "스스로 거른다" 쪽), 단 결함 없는 bag 에서 ATE 가 2–4 cm 나빠졌다.

## 해석

### E3(b) (2026-10-14)

- **주 결과**: SLAM 지도 위(L2)에서 RTAB-Map 3D 위치추정이 배포 2D 사슬보다 xy ATE 가 작다 (`A<B`, 6·7 bag 모두 모든 bag 이 같은 부호).
  정답 지도 위(L1)는 주 결과(6 bag)에서 가르지 못했다 — 결함 없는 bag 중 s3_aisle 에서 2D 가 0.11 cm 작았고, 6 bag 의 가능 최소 p 가
  0.031 이라 한 bag 의 반대 부호로 기각이 안 된다.
- **크기는 사건이 정한다**: 평균 차의 크기는 EKF 결함 bag 에서의 2D 발산이 거의 전부다. 결함 없는 4 bag 의 차(3D − 2D)는 정답 지도
  −0.56…+0.11 cm (3 bag 에서 3D 가 작고 s3_aisle 은 2D 가 0.11 cm 작다), SLAM 지도 −2.45…−0.93 cm 다 〔리뷰 반영 정정 2026-10-15 — 처음 문장은 아래 「리뷰 반영」 절〕. 즉 정상 입력에서
  3D 의 이득은 SLAM 지도 위 약 1–2.5 cm 이고 정답 지도 위에서는 bag 에 따라 방향이 갈린다 (크기 ≤ 0.6 cm), 큰 차이는 **휠-EKF odom 이 틀렸을 때와 사전 지도에 없는
  물체가 시야를 막을 때의 강건성** 에서 나왔다 (발산 2D 9/42 대 3D 0/42). 둘 다 2D 사슬의 kidnap 감시·AMCL 이 odom 과 스캔-지도
  일치에 기대기 때문으로 본다 **[추정]**.
- **3D 쪽 약점**: 처리 스캔 누락 실패가 19/42 로 2D(메시지 필터 6/42)보다 많다. 그중 6 은 bag 결정적 시작 장, 2 (`s1_seed3_a2` r1,
  727 장)는 호스트 OOM 정지 뒤 구독이 복구되지 않은 재생이라 CPU 여유의 근거에서는 빼지만, 정답 지도 행은 **3D 위치추정이 정지에서 복구하지
  못한 재생**으로 불리한 사실에 남긴다 (리뷰 반영 2 회차) 〔리뷰 반영 정정 2026-10-15 — 처음 문장은 아래 「리뷰 반영」 절〕. 주행 중 1–6 장 누락 11 재생과 지연으로 보면
  공유 부하에서 3D 위치추정의 처리 여유가 작다 (스캔당 지연 — E2 중간 분석, 공유 부하 하 예비값: 3D icp p50 62 ms · p95 118 ms, 2D 7 · 47 ms). 실시간 운용 주장은 할 수 없다.
- 명세 구간: 결함 없는 bag 에서 두 사슬 모두 RMSE 는 기준 안이고, max 는 3D 정지 3/12 재생·2D 직선 1/12 재생이 넘었다. SLAM 지도 위 2D 는
  정지 max 가 12/12 재생에서 3 cm 를 넘었다 (최악 5.48 cm) — "지도의 몫" 이 2D 에서 크다.

### E4 (2026-10-15)

- 가장 큰 손잡이는 **정합 오차 척도**다: 점-대-점은 결함 없는 bag 에서도 5 m 발산을 냈다. 정형 창고에서 점-대-평면이 필수라는 보고서
  근거와 같은 방향이다.
- **복셀 크기는 정확도-속도 곡선**을 그린다: 0.05 m 는 느리고(+21 ms) max 오차가 커졌고, 0.2 m 는 빠르고(−22 ms) 국소 오차(RPE)가 줄었다
  (정확도는 주 결과; 지연 수치는 결과 뒤 보충 기준이라 **관찰** 〔리뷰 반영 2026-10-15〕). 0.3 m 는 더 빠르지만 max 오차가 커졌다. 이 월드·속도에서는 0.2 m 가 현재 0.1 m 보다 낫다는 신호가 있다 — 결함 없는 bag 둘이라
  주장하지 않는다 (열린 문제).
- 높이 대역은 궤적을 바꾸지 않고 지도 지표만 바꿨다 (예측대로). 루프 거부 문턱은 아무것도 바꾸지 않았다 — LiDAR 단독 RTAB-Map 은 근접
  루프만 받아 거부할 일이 거의 없다 (예측대로).
- 비유한 점 제거를 끄면 RTAB-Map 이 멈추지는 않지만 궤적이 2–4 cm 나빠졌다 (결함 없는 bag 둘, 주 결과는 가르지 못함) — 전처리를 유지할
  근거로 쓰되 "필수" 라고는 쓰지 않는다.
- 지연 비교는 기준을 같은 기간에 다시 재지 않은 설계 결함이 있다 — 다음 절제는 기준 행도 같은 묶음에 넣는다.

## 다음

(2026-10-15 리뷰 반영 때 채움 — 자세한 목록은 `bonus2_results.md` §12)
- 결함 bag 3 개는 다시 녹화하거나 `wheel_odom` guess 행을 더해 다시 본다 (재생만으로 가능, 사전 등록 필요).
- 절제는 기준 행을 같은 묶음·같은 시각에 넣고, 복셀 0.2 m 를 결함 없는 bag 여럿에서 사전 등록해 다시 본다.
- 재생 측정 사슬(기록기·탐침)이 호스트 정지 뒤 스스로 복구하는지, 끊김을 표시하는지 확인한다 (아래 B1).

## 리뷰 반영 (2026-10-15, 리뷰 대상 `c0a0751`)

리뷰 판정 fix_needed — 문서만. 사전 등록 주 판정(L1·L2·E5·가족 T·M)은 리뷰의 독립 재계산과 같았다. 위 본문은 고친 곳에
〔리뷰 반영 정정〕 표지를 달고 바꿨다. **처음 문장**(바꾸기 전, 기록용)과 확인:

- **B1 — 727 장** (결과 「불리한 것 먼저」 둘째 줄, 해석 「3D 쪽 약점」). 처음: "`s1_seed3_a2` r1 은 두 행 모두 727 장 누락 … 재생 중 처리가
  멈춘 사건이다", "한 재생은 727 장을 잃었다 — 공유 부하에서 3D 위치추정의 처리 여유가 작다". 확인 [측정, `runs/b24/b2-eval/supp/stall_1909/`]:
  - 두 행 `row.log` 의 'Odom: ratio' 는 2150 줄, 마지막 줄 19:13:25 (재생 끝). 처리 로그 기준 누락 92, 기록기 기준 727 (`replays.jsonl`
    `missing_log` 92 · `match_missing` 727 · `missing_disagree` True). 두 셈을 가진 다른 재생 160 개(E2 48·E3(b) 40·E4 72)는 모두 같다.
  - **처리 로그 공백**: 두 행 모두 벽시계 19:08:37.4 → 19:09:15.4/15.6 (38.0·38.2 s). 38 s × 유효 sim 0.25 × 10 Hz ≈ 95 장 [계산] — 92 와 맞는다.
    SLAM 지도 행의 재생기는 19:09:15.4 에 "Message queue starved" 를 냈다.
  - **같은 시각 호스트**: `journalctl -k` — 19:09:14 커널 OOM (Free swap 0 kB / 17 GB), 다른 사용자 `openroad` (anon RSS 36 GB) 를 죽임,
    19:09:59 에 한 번 더 (`openroad` 48 GB). `driver.log` 19:09:14 의 1 분 loadavg 120.0 (19:08:23 표본 29.7). 함께 돌던 s3_aisle `amcl_*` r1 은
    기록 끝 19:08:31 에서 구동기 END 까지 43 s (다른 E3 r1 2D 재생은 5–14 s). 2026-10-13 21시 – 10-15 09시 커널 기록에 OOM 은 이 두 번뿐이다.
    재부팅(10-10) 전 기록은 없다.
  - **기록기·탐침**: 두 행 out_bag `icp_odom` 마지막 스탬프 162.9·163.0, 정답 지도 행 `traj_online.tum` 마지막 163.1 (덮음 0.68, 창 안
    표본 1498/2219), SLAM 지도 행 탐침은 235.2 까지 (덮음 0.998). 공백 시작의 sim 시각은 정지 전 처리 장 수(1515)로 어림해 약 163 s [계산].
  - 해석: 처리 정지 92 장도 3D 고유의 여유 문제라기보다 호스트 전체 정지다 **[추정]**. 기록기가 다시 붙지 않은 기제는 미확인.
  - 결과 문서 §0-5·§6.2·§10·§11·§12-5 를 고쳤다. 실패 판정(727, 큰 값 규칙)은 그대로. 정답 지도 행 r1 ATE 1.01 cm 는 앞 68 % 구간 값.
- **B2 — 결함 없는 bag 의 L1 부호** (해석 「크기는 사건이 정한다」). 처음: "결함 없는 bag 의 차는 정답 지도 0.1–0.6 cm, SLAM 지도
  0.9–2.5 cm 다. 즉 정상 입력에서 3D 의 이득은 수 mm–2 cm". s3_aisle L1 은 +0.11 (2D 가 작다) — 3D 쪽으로 후했다. 결과 문서 §0-4 도 고침.
- **B3 — loadavg** (결과 문서 §3.2 "재생 시작 loadavg 130–190", 머리말 "시작 5–80", 이 항목 결과 머리 "시작 loadavg 26–79"). 실제 1 분
  시작 값은 s1_seed1 r2 39.28 · s1_seed2x r2 38.41, 전체 308 job 5.3–50.4 (E3 11.5–50.4). 재생 중 최대 204·186 (두 재생), 전체 254.
  "26–79" 가 어디서 왔는지는 다시 찾지 못했다 — 1 분·5 분·15 분 어느 범위와도 맞지 않는다.
- **B4 — 지연 판정** (결과 문서 §10 "복셀을 키우면 지연이 준다 = 가능"). 근거가 결과 뒤 보충 기준이라 "관찰" 로 낮췄다. 해석 E4 의 지연 수치에도 표지.
- **비차단**: (1) ADNN₀ 도구 편향 — `gridmap.sample_field` `mode='nearest'` [코드]; s3_aisle r0 3.80 → 4.07, s1_seed1 r0 4.16 → 4.52 cm,
  래스터 밖 셀 4·8 개, 2D 두 행은 0 개 [계산, `supp/adnn_oob/out.jsonl`, 스크립트 `adnn_oob.py`]. 리뷰의 5 bag·8 bag 수치(4.37, C4 부호 3/5)는
  재현하지 않았다. 주 결과는 사전 등록 도구 값 그대로, 결과 문서 §0-7·§4.2·§11 에 적음. (2) 요청 경로 → `src/amr_localization/src/imu_filter.cpp`
  `ImuFilter::finishStartup`, 루트 `config/sensors.yaml` 129행 (E2 항목에도 표지). (3) 회전 분모 12 는 결과 커밋 `db8cea2` 에서 처음 적음 —
  결과 문서 §6.3 에 "결과 뒤 표기 변경". (4) 슬롯 밖 재생 5 개를 결과 문서 §11 에 더함 (그중 map_long `rtabmap2d_noimu` r0 누락 1 — 실패로 셈).
  (5) s6_wall 2D 이탈 t 12.9–13.0 s (처음 "t 15 s, 처음 4 s 안"), EKF yaw 표류 1.2° (처음 "약 2°") [계산, `traj_online.tum` 대 `gt.tum`,
  `supp/odom_v2/s6_wall_a1_filtered.tum`]. (6) E4 점-대-점 3/3 중 `s1_seed3` 차 +0.22 cm (δ 0.2) — 결과 문서 §8.2·§10 에 단서.
  (7) 린트 실패 커밋 `40b881e` 은 `fff2f91` 에서 고쳐졌다 — 변경 없음.
- 리뷰와 다르게 본 것: 없음. 재생은 다시 돌리지 않았다 (리뷰도 불필요로 판정, 리뷰의 재실행 2 건은 원 재생과 비트 단위로 같았다).

## 리뷰 반영 (2 회차, 2026-10-15)

재확인에서 B1 이 일부만 해결됐다 — 1 회차 정정의 "처리는 끝까지 돌았다 / 측정 사슬 사건" 은 icp_odometry 에만 맞았고, 정답 지도 행의
RTAB-Map 위치추정 노드는 정지 뒤 갱신하지 않았다 (갱신 줄 224/152/224, `runs/b24/b2-eval/recheck/b1/rtabmap_updates.out`). 위 두 곳과 결과 문서
§0-5·§6.2·§10·§12-5·§15 를 고쳤다. 판정 영향 없음: 실패 재생은 평균에 남고 r1 을 r0·r2 값으로 바꿔도 bag 평균 0.03 cm 차, 그 bag 의 L1 차는 2D 발산(−1600 cm)이 정한다.

