# A5-2 Gazebo 다회 에피소드 하네스·시나리오 집합 v1·에피소드 지표 정의

- 시각: 2026-10-06 22:49 KST (아래 "설정"까지는 하네스로 에피소드를 하나도 돌리기 전에 쓰고 커밋한다) · 에이전트: b1-gzbench · 단계: A5
- 코드: 이 항목과 함께 커밋 (`src/amr_rl_bench/**`, `rl/eval/gazebo/**`) — 커밋 해시는 결과 절에 적는다
- 관련 실행 id: `A5_b1gzbench_scenarios_v1` (생성기), 하네스 시험·스모크는 A5-4 항목

## 질문

단계 C(RL 대 DWA, 팔당 ≥ 200 에피소드)를 **한 Gazebo 세션에서 에피소드를 연달아** 돌릴 수 있게, (1) 결과 전에 고정한
같은 시작·목표 집합, (2) 에피소드마다 같은 방식의 준비(순간 이동·위치추정·코스트맵), (3) 정답 자세와 추정 자세를 둘 다 쓰는
층위 지표를 만드는가? 이 항목은 **도구의 정의**를 사전 등록한다. 도구가 실제로 끝까지 돌고 지표가 채워지는지는 A5-4 (스모크).

## 가설과 예측 (결과를 보기 전에 적는다)

- 하네스 순수 모듈 시험(`src/amr_rl_bench/test`, rclpy 비의존 pytest 26 개)이 통과하고, 정적 기하가 Isaac 학습 환경의
  사각형과 같다 (`test_warehouse_rectangles_match_isaac`, 허용 1e-12).
- 직진 접근 거리·풋프린트 여유가 A3 재판정 구현(`feature/b1-isaac-phi` 의 `rl/isaac/amr_isaac/eval_metrics.py`)과
  무작위 자세 3000 개에서 정확히 같다 [계산 — 이 항목을 쓰기 전에 이미 돌렸다: 최대 차 0, 직진 거리 유한 912 개.
  `/home/CAPYI_AI/runs/bonus1/b1-gzbench/A5_scenarios/a3_eval_metrics.py` 로 대조].
- 시나리오 집합 v1 의 좁은 목표 비율은 A3 의 Isaac held-out (3–15 m) 2.3 % 와 같은 수준일 것이다 [계산 — 생성은 이미
  했다: 2.35 % (201/8559). 같은 규칙·같은 계획기라 예측이라기보다 확인이다].

## 설정

### 시나리오 집합 v1 (`rl/eval/gazebo/scenarios_v1.json`, 생성 `make_scenarios.py`, 규칙 `rl/eval/gazebo/README.md` §1)

- 주 200 + 좁은 목표 스트레스 200. 시드 20261006 (순서 시드 20261007). 생성 통계 [계산, 실행 `A5_b1gzbench_scenarios_v1`]:
  후보 9024, 길이 범위 밖 465, 계획 실패·자기 근접·여유 미달 0, 조건 통과 주 8358 · 스트레스 201.
- 길이 L 사분위 주 3.08 / 12.95 / 19.80 / 25.43 / 29.99 m, 스트레스 3.33 / 13.22 / 18.26 / 24.77 / 29.82 m.
  스트레스의 직진 여유 0.101–0.398 m (중앙 0.276), 주는 0.418 m 이상 (200 중 150 개는 5 m 넘게 비어 inf).
- 실행 순서 = 두 집합 번갈아, 스모크 = 실행 순서의 처음 주 10 + 스트레스 10.
- 결정 근거: 완성 계획 §6 2026-10-06 21:05 결정과 21:55 정정 (평가는 주 집합과 좁은 목표 스트레스 집합을 같은 비중으로).
  "각 시나리오는 실행 순서대로 한 번씩, 실패를 골라 다시 돌리지 않는다" (PROTOCOL.md §2) — 하네스 고장(error)만
  새 실행 순번으로 덧붙인다.

### 에피소드 절차 (`src/amr_rl_bench/scripts/gzbench_runner.py`, 머리말이 규범)

1. 이전 에피소드가 멈출 때까지 (≤ 5 s sim).
2. 순간 이동 `ign service /world/warehouse/set_pose` → 0.3 s 뒤 바로 위치추정 초기화 = initialpose(정답 자세, σ 0.05 m·3°) ×3
   + map EKF set_pose + AMCL 무이동 갱신 ×2 (`tests/integration/amr_itest/actions.seed_pose` 와 같은 묶음 — 운영자가
   초기 자세를 주는 실배포 절차). 바로 주는 이유: 납치 감시가 정합 거부 1 s + 의심 1 s 뒤 LOST 를 선언하면 회전 복구가
   시작 자세를 흐트러뜨린다. 지면 진실로 도착 확인 (0.05 m·0.05 rad, 최대 3 회).
3. `localization/lost` 가 True 가 아니고, 정지해 있고, 추정(TF map→base_footprint)이 정답과 0.10 m·0.05 rad 안으로 2 s 유지
   (최대 30 s, 한 번 더 알려 주고 다시 30 s). 못 맞추면 그 에피소드는 error (사유 기록) — b1-plugin 다음 ⑦ (LOST 상태로
   목표를 보내지 않는다).
4. 전역·지역 코스트맵 clear_entirely → 1 s.
5. navigate_to_pose (수락 확인 최대 3 회 — 한 노드가 계속 살아 있어 A2 의 "목표마다 새 프로세스" 를 쓰지 않는다).
6. 감시 (50 Hz): Nav2 종료 / 지면 진실 풋프린트–정적 상자 부호 거리 < 0 이면 **즉시 취소** (collision, 계약 §8 접촉 = 종료) /
   sim 경과 > 시간 상한이면 취소 (timeout) / sim 시계가 30 s(벽시계) 멈추면 취소 (watchdog → error).
7. 정착 (지면 진실 |v| < 0.01, |ω| < 0.02 또는 3 s) 뒤 최종 자세(지면 진실·추정) → 지표 → `gzbench.csv`(gzbench/v1),
   `episodes_v1.csv`(rl/eval episode/v1), `episodes/<순번>_<id>.json.gz` (원자료 전부).

### 지표 정의 (`src/amr_rl_bench/amr_rl_bench/metrics.py` 머리말이 규범, 요약은 `rl/eval/gazebo/README.md` §2)

- 층위 성공: strict(xy ≤ 0.10 ∧ |v| ≤ 0.05 인 순간) · nav2compat(xy ≤ 0.25 에 접촉 전) · system_success(Nav2 SUCCEEDED) ·
  final(정착 뒤 거리 분포) — 각각 **정답 자세(_gt)와 추정 자세(_est)** 둘 다.
- 계약 종단 사건 `outcome_contract` = 시간순 첫 사건 (collision → success → stall → timeout), episode/v1 의 outcome.
  stall 은 Nav2 SimpleProgressChecker 규칙 (기준점에서 0.3 m 넘게 움직이면 기준 갱신, 기준 뒤 20 s).
  배포 시스템은 정체 뒤에도 BT 복구를 하므로 실행기는 에피소드를 끊지 않는다 — 시스템 결과(`outcome_system`)는 따로 남는다.
- 위치추정 붕괴 = 에피소드 중 추정–정답 > 0.5 m (`loc_collapse`), LOST 전이 수 — 제어기 비교에서 따로 센다 (b1-plugin 다음 ④⑥).
- 목표 직전 정지 시간 `near_goal_stop_s` (b1-plugin 다음 ②⑧), goal checker 가 회전 중 성공 `success_while_rotating`
  (Nav2 성공 시각 |ω_gt| > 0.1, 다음 ⑨).

### 판정 (이 항목)

- 합격 = 순수 모듈 시험 전부 통과 + 린트(flake8 99, pep257) 통과 + 시나리오 집합이 결과 전에 커밋됨.
- 하네스가 Gazebo 에서 끝까지 도는지, 지표가 채워지는지는 A5-4 의 판정이다.

## 결과

(아래는 결과 뒤에 쓴다)

## 해석

## 다음
