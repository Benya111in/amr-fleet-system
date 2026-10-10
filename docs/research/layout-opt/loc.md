# layopt 측위 쪽 — E0 지도 지표, E2 단일 로봇 시행, 실험 1

작성 layopt-loc, 2026-10-10. 설계 `design.md` §4·§6(실험 1), 판정 `prereg.md` §2–§4, 형식 `contract.md` §3을 따른다.
코드는 `layopt/loc/`에 있다. 원자료는 저장소에 넣지 않는다 (`/home/CAPYI_AI/runs/layopt/{e0,e2,analysis}/`).

## 1. E0 — 지도만으로 구하는 측위 위험 (`layopt/loc/e0.py`)

자세 하나와 이웃 k대(k ∈ {0, 2, 4})마다 다음을 계산한다.

1. **레이캐스트 세계**: 기준 지도에 이웃 로봇 상자(0.60 × 0.40 m, 이웃 자세)를 덧씌운다. 점수에 쓰는 기준 지도에는 넣지 않는다.
   운용 로봇이 가진 지도에는 다른 로봇이 없기 때문이다.
2. **가상 스캔**: 720빔, 25 m, σ 0.03, 스캔 잡음 시드 0. `raycast.py`는 `docs/research/state-estimation/checks/alias_margin.py`의
   raycast와 비트 단위로 같다(`test_e0.py`가 확인한다). 이 결과를 벡터화로 얻는다.
3. **운용 코드 그대로**: `global_seed.search`(coarse 0.5 m × 5°, 90빔, top-50, NMS 1 m/20°, fine 0.1 m × 1°, 180빔)로 가설을 얻는다.
   - `margin` = `global_seed.alias_margin(참 자세, 1 m/20° 밖 가설, 0.2 m)`. kidnap_monitor 수렴 게이트와 같은 정의다.
   - `inlier` = `scan_map_match.match_ratio(참 자세, 180빔, 0.2 m)`. 운용 LOST 판정(0.50)과 같은 정의다.
4. **보조 필드** (contract 밖): `inlier_raw`(지도에 없는 물체 빔을 빼지 않은 비율), `occluded_frac`(이웃 상자에 막힌 빔 비율),
   `rho_true`, `second_rho`, `truth_rank`, 가설 목록.

출력은 `e0/<run_id>/<spec_id>.json`(contract §3의 상위 집합)이다. 경로 표본(`route_samples`)은 자유 공간 자세 40개다.
조건은 점유까지 ≥ 0.8 m, 헤딩 0/90/180/270°, 시드 0이며, k=0 마진의 q05·q50을 낸다.
`batch`는 묶음마다 파일을 쓰고, 있는 결과는 건너뛴다(재개). 새 실행 디렉터리가 이미 있으면 거부한다.

### 1.1 검증 (SLAM 지도, k=0) — [계산]

| 슬롯 | 설문 `spawn_alias.py` | E0 | 둘째 가설 (거울) |
|---|---|---|---|
| (18, −16) | 0.048 | 0.0476 | (−18.1, 16.0) 48.3 m |
| (20, −16) | 0.034 | 0.0338 | 51.3 m |
| (22, −16) | 0.027 | 0.0268 | 54.5 m |
| (24, −16) | 0.034 | 0.0336 | 57.8 m |
| (26, −16) | 0.007 | 0.0068 | 61.1 m |

원자료: `e0/20261009T172430Z_validate-slam2/`. 앞 실행 `…_validate-slam`은 `occluded_frac` 결함(잡음 수열 밀림)이 있어 NOTE로 남겼다.

### 1.2 발견 — 운용 inlier 정의는 이웃 가림에 무감하다

운용 `match_ratio`는 `explain_unmapped`로 "지도에 없는 물체에 막혀 일찍 끝난 빔"을 분자와 분모에서 모두 뺀다
(통합 11의 거짓 LOST 대책). 그래서 참 자세에서는 이웃 0/2/4대 모두 `inlier = 1.0`이다. 반면 `inlier_raw`는 0.88–0.94로 떨어진다.
E2 자연 단계 실측도 같았다(정지한 로봇, `kidnap_status.match_ratio` 최소 1.0).

P5 대기 슬롯 실패의 정확한 기술은 **"정지 중 원인 미확인 LOST → 낮은 마진에서 거울 수렴"**이다.
- 근거는 조정자의 재분석(prereg §7, 2026-10-10 05:00)이다.
  - LOST 시점에 로봇과 이웃은 30 s 넘게 정지해 있었고, 추정 오차는 0.01–0.03 m였다.
  - 6 m 안에 동적 장애물이 없었다.
  - LOST 직후 탐색에서 참 자세의 inlier는 0.923이었다.
- LOST 트리거는 이웃 가림이 아니며, 원인은 아직 모른다.
- → R_loc의 λ_LOST는 배치와 무관한 상수다(prereg §7). 그래서 실험 1의 핵심 산출물은 강제 복구 단계의 P_mis 대 E0 마진 관계와 AUROC다.
  자연 단계 결과는 등록대로 보고만 한다. "정지 가림만으로는 LOST가 나지 않는다"를 확인하는 몫이다.
- 트리거 후보 (미검증, layopt-loc 의견):
  - kidnap_monitor는 정지 중 `map_pose_at` = (마지막 AMCL 갱신) ∘ (그 뒤 `odometry/filtered` 상대 이동)으로 inlier를 잰다.
  - AMCL은 정지 중 갱신하지 않는다. 그래서 정지가 길어지면 odom EKF의 헤딩 흐름(자이로 바이어스 적분)이 합성 자세에 그대로 쌓인다.
  - map EKF는 정합기·AMCL로 보정되므로 흐르지 않는 것처럼 보일 수 있다.
  - 검증 방법: 정지 5분 이상 동안 `odometry/filtered` 헤딩과 `kidnap_status.match_ratio`를 함께 기록한다.

### 1.3 이웃 상자 마진 효과

이웃은 참 가설과 거울 가설에 똑같이 아웃라이어로 작용한다. 그래서 마진은 거의 변하지 않는다
(대기 슬롯 0.0338 → 0.0323, k=2). [계산]

## 2. E2 — 단일 로봇 Gazebo 측위 시행 (`e2_runner.py`, `e2_container.sh`, `e2_campaign.py`)

### 2.1 스택

`amr_bringup system.launch.py`를 쓴다. `multi_robot.launch.py`와 같은 `launch_utils.bringup_actions`이고, 패키지 YAML도 같다.
- 로봇 이름과 프레임: `robot_name amr_01`, `prefix amr_01/` (다중 로봇과 같은 프레임 이름).
- 켬: localization(AMCL + 이중 EKF + kidnap_monitor + scan_matcher), navigation(behavior_server spin),
  perception(safety_node — cmd_vel의 유일한 발행자. 빼면 복구 회전이 끊긴다).
- 끔: behavior, fleet, dashboard, evaluation (§4 참고).
- DOMAIN 57, `IGN_PARTITION=amr_ai_layopt_e2`, `gz_slot.sh layopt-loc`, 컨테이너 `amr-ai-layopt-e2-*`.

### 2.2 시행 절차 (시행 하나)

1. 더미 주차 → 로봇 순간 이동(`/world/<w>/set_pose`) → 곧바로 참 자세를 시드한다.
   시드는 initialpose ×3 + map EKF set_pose + AMCL 무이동 갱신 ×2로, `amr_itest.actions.seed_pose`와 같은 묶음이다.
   그다음 이웃 더미 k대를 놓는다. 더미는 정적 상자 0.60 × 0.40 × 0.30이고 중심 z는 0.18이다.
   세션 시작에 4개를 만들고 set_pose로 옮긴다. 실스캔에서 옆 이웃까지 1.80/1.77 m로 보이는 것을 확인했다.
   - 시드를 이동 직후에 주는 이유: 정합기 거부 창(20스캔)이 차면 이동 2 s 안에 LOST가 선언된다.
2. 안정 대기: TRACKING, lost false, 추정 오차 < 0.15 m, 헤딩 오차 < 5°가 2 s 연속이어야 한다(상한 40 s sim).
3. **자연 단계 20 s (sim)**: LOST 여부를 보고, `kidnap_status.match_ratio`(1 Hz)를 기록한다.
4. **강제 단계**(자연 LOST가 없을 때): `scan_match/accepted`에 false를 100 Hz로 넣는다(정합기 자체 발행은 10 Hz).
   이것으로 운용 감지기의 `scan registration rejected > 0.8` 경보를 일으킨다. 그러면 SUSPECT 1 s → `_declare_lost`로 넘어간다.
   LOST를 보면 즉시 멈춘다(실측 1.1 s).
   - 이 경로를 고른 이유:
     - `_declare_lost` 뒤 과정은 실제 실패와 같다: 현재 스캔으로 가설 탐색 → 시드 #1 → 회전 → 여백 게이트
       → 재초기화 #2 → … → AMCL 전역 → FAILED.
     - P5에서도 이 경보로 LOST가 났다(ar9 amr_01 첫 LOST: `scan registration rejected 20/20`).
     - 운용 코드를 바꾸지 않는다.
     - 외부 initialpose와 달리 여백 유예 창을 열지 않는다. `_declare_lost`가 유예를 닫는다.
   - 실제 실패와 다른 점은 두 가지다. 첫째, LOST 사유 문자열이 다르다. 둘째, LOST 직전 추정이 참 자세다(P5에서도 오차 1 cm 미만이었다).
5. **판정** (prereg §3):
   - 판정 시각: TRACKING 선언 시각이다. 선언은 `localization/lost` false 수신 또는 `[TRACKING] recovered` 로그 중 먼저 온 것으로 본다.
     선언이 없으면 LOST + 120 s다.
   - 오차: 그 시각 `odometry/filtered_map`과 `ground_truth/odom`의 위치 오차.
   - 분류: < 0.5 m 정수렴, > 1.0 m 오수렴, 그 사이 모호.
   - 민감도용으로 +1 s 시점의 오차와 AMCL 오차도 함께 남긴다.
6. **기록**:
   - 시행마다 `trials/<id>_a<n>/`에 남긴다:
     - `track.csv`: gt·est 10 Hz, AMCL 전부.
     - `status.jsonl`: kidnap_status 전부.
     - `rosout.jsonl`: kidnap_monitor·amcl·behavior_server·scan_matcher.
     - `scans.json`: 자연 시작 시각과 LOST 시각의 실스캔.
   - 결과 한 줄을 `trials.jsonl`과 `attempts.jsonl`에 fsync로 바로 쓴다.
   - 결과에 들어가는 것: 회복 시간, 재초기화 수, 시드(참/거짓), 첫 가설 탐색, 여백 게이트 값, 부하(loadavg), RTF.

### 2.3 세션·재개·이상

- 세션 하나 = Gazebo + 스택 + 러너, 시행 20개다. 새 세션은 Gazebo를 새로 띄운다.
- 이상이 나면 그 시도를 `invalid`로 남기고 세션을 새로 띄운다. 이상: 시계 정지, 기동 실패, 배치 실패, 안정 실패, 강제 LOST 미선언.
  시도는 시행당 2번까지다.
- done 시행은 건너뛴다. 중간에 끊겨도 잃는 것은 진행 중 시행 하나다.

### 2.4 함정 (실측)

- `/rosout`의 `stamp`는 rcutils 로깅 시각, 즉 **벽시계**다(use_sim_time과 무관). 사건 시각은 수신 sim 시각으로 쓴다.
  dbg2는 이 결함 때문에 판정이 불가했다.
- kidnap_monitor의 상태기 사건 로그(`[RECOVERING] LOST:` 등)는 동작 실행(가설 탐색 0.2–0.3 s) 뒤에 몰아서 찍힌다.
  그래서 LOST 시각은 `localization/lost` 수신 시각으로 잡는다.
- AMCL은 정지 중에 발행하지 않는다. 판정 시각의 AMCL 값은 "그 시각 이전 마지막 발행"으로 잡는다.

## 3. 실험 1 (`make_trials.py exp1`, `analyze_exp1.py`)

- 기준 묶음 `v10-v20-v30-v40-v50`(래스터 지도)의 world.sdf와 map.yaml을 쓴다. E0와 E2가 같은 지도를 쓴다.
- 자세:
  - 체류 지점: waiting, dock, charger 전부. queue는 시드로 12개를 고른다.
  - 자유 공간: 후보 400개를 E0 k=0 마진 5분위로 나눈다. 후보 조건은 점유까지 ≥ 0.8 m, 이웃 자리 4개가 모두 빔, 체류 지점에서 1.5 m 밖이다.
  - 자세 수는 (240/3 − 체류 지점 수)를 5분위로 같은 수씩 나눈다.
- 시행: 자세 × k ∈ {0, 2, 4}마다 한 번, 순서는 시드 20261010으로 섞는다. 시행마다 같은 자세·k·이웃의 E0 값을 적는다.
- 분석: `analyze_exp1.py` 머리말의 정의를 사전 등록한다.
  - y_mis = 오수렴 여부. 모호는 주 분석에서 0이다.
  - 위험 점수 = −margin.
  - AUROC의 95% CI는 자세 단위 군집 부트스트랩 2,000회로 구한다.
  - 로지스틱: P(오수렴 | margin), P(LOST | inlier).
  - 게이트 A: AUROC ≥ 0.8이면 지지, 0.7–0.8이면 약한 지지, < 0.7이면 기각. 판정이 불가하면 미통과다.
- 시행 목록 커밋 해시와 실행 id는 노트 항목에 적는다.

## 4. 설계와 다르게 한 것

| 항목 | 설계·지시 | 이번 | 이유 |
|---|---|---|---|
| E2 스택의 behavior(작업 실행기·도킹 서버) | "다중 로봇과 같은 스택" | 끔 | 단일 로봇에는 작업이 없다. 도킹 서버의 marker_fix는 lost 중 무시된다(`on_marker_fix`). 실행기가 대기 자세로 되돌리는 동작이 시행을 오염시킬 수 있다 |
| 자연 단계 | 20 s | 20 s, **정지 상태, 시드 직후** | 지시대로다. P5의 LOST는 도착 뒤 52–184 s 정지 중에 났다(원인 미확인). 20 s 자연 단계는 그 시간대를 덮지 못한다. λ_LOST는 prereg §7에 따라 P5 상수를 쓴다 |
| LOST 트리거 | "실제 실패와 같은 방식" | 정합기 거부 주입 (§2.2-4) | 운용 코드를 바꾸지 않는 가장 가까운 경로다 |
| E0 inlier | 운용 정의 | 운용 정의 + 보조 2개 | §1.2 |

## 5. 결과 요약 (2026-10-10, 상세는 노트 항목 `entries/2026-10-10_0239_layopt-loc_e0-predicts-misconvergence.md`)

| 항목 | 값 | 표기 |
|---|---|---|
| E0 전수 | 유효 묶음 500/500, 오류 0 (`e0/20261009T180626Z_all540`) | [계산] |
| E0 처리량 | 워커 8: 묶음당 벽시계 17.4 s. 워커당 묶음 130 s(중앙). 묶음당 평가 약 214회, 0.5 s/평가 | 공유 부하 21–61 아래 [측정] |
| 실험 1 | 237 시행 / 79 자세, 오수렴 51 (0.215), 자연 LOST 4 (0.017) | [측정] |
| AUROC (−E0 margin) | **0.956** [0.902, 0.992], 자세 군집 부트스트랩 2,000 | [계산] |
| 게이트 A | **지지 → 통과** | prereg §2 |
| P_mis(margin) | σ(3.515 − 78.08 · margin), P=0.5 에서 margin 0.045 | [계산] |
| E2 시행 시간 | 벽시계 중앙 68 s (마진 큰 자세 47–130 s, 120 s 창을 다 쓰는 자세 190–730 s), RTF 0.16–0.93 | 공유 부하 아래 예비값 |

분석 산출물: `/home/CAPYI_AI/runs/layopt/analysis/exp1_20261009T180550Z_exp1/`.
