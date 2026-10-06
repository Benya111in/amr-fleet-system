# A5-4 하네스 스모크 — DWA 20 · RL(v12_s1) 20, 에피소드당 벽시계·RTF 와 단계 C 소요 추정

- 시각: 2026-10-06 23:35 KST (아래 "설정"까지는 스모크를 돌리기 전에 쓰고 커밋한다) · 에이전트: b1-gzbench · 단계: A5
- 코드: 하네스 `src/amr_rl_bench` (A5-2), 플러그인 `85c0ca8` 이후 (A5-1b), 시나리오 `rl/eval/gazebo/scenarios_v1.json` (`c7be0db`)
- 관련 실행 id: 결과 절에 적는다

## 질문

A5-2 의 하네스가 실제 Gazebo 세션에서 **끝까지 돌고 지표가 채워지는가**? 에피소드당 벽시계 시간·RTF 는 얼마이고, 그로 보아
단계 C 의 팔당 200 에피소드(주 100 + 스트레스 100 또는 주 200 + 스트레스 200)는 얼마나 걸리는가? 이것은 **도구 시험**이다 —
n = 20 이라 RL 대 DWA 의 차이를 주장하지 않는다.

## 가설과 예측 (결과를 보기 전에 적는다)

판정 (합격 조건):

- H1 완주: 세션마다 사전 등록한 스모크 20 개(`smoke`, 실행 순서대로, 주 10 + 스트레스 10)를 사람 개입 없이 끝낸다.
  준비 실패(error)는 팔당 ≤ 2 이고 하나하나 사유가 기록된다. 하네스가 멈추면(세션 미완) 불합격.
- H2 지표: error 가 아닌 에피소드는 핵심 열이 모두 채워진다 — outcome_contract, strict_gt, strict_est, nav2compat_gt,
  system_success, final_dist_gt_m, final_dist_est_m, nav_time_s, path_length_m, cte_active_mean_m, interventions,
  min_clear_static_m, loc_err_max_m, loc_err_end_m, rtf, wall_episode_s, ctrl_dt_sim_mean, scan_to_cmd_p50_s
  (RL 은 decisions·decision_period_mean_s·scan_age_p50_s 도). `episodes_v1.csv` 가 `rl/eval` `schema.read_csv(strict=True)` 를 통과한다.
- H3 일관성: system_success(Nav2 성공 = 추정 자세 xy 0.10)인 에피소드의 ≥ 90 % 에서 정착 뒤 final_dist_est_m ≤ 0.12
  (기록된 추정 자세가 Nav2 가 본 자세와 같은 것인지 확인).
- H4 준비 품질: error 가 아닌 에피소드는 모두 loc_err_start_m ≤ 0.10, lost_at_start = False (하네스가 그렇게 만들었는지).

기술 (주장 없음, 방향만 적어 둔다 — 낮은 확신):

- DWA: 주 집합 strict_gt ≥ 8/10, 스트레스는 그보다 낮다. RL(v12_s1): 주 집합 strict_gt ≥ 7/10 — A2 에서 반복된 목표 직전 정지
  (지면 진실 0.10–0.14 m) 가 엄격 기준에서 떨어뜨릴 것. 두 팔 모두 접촉 0.
- RTF 0.7–0.95 (부하에 따라), 에피소드당 벽시계 60–120 s (준비 약 10–15 s 포함) → 단계 C 팔당 200 에피소드 ≈ 3.5–7 h.

## 설정

- 세션: (1) `dwa` → (2) `rl` (v12_s1, 모드 B, replace_dwa). 둘 다 같은 스택(`run_bench.sh`: system.launch navigation 끔 +
  rl_navigation.launch — DWA 세션은 RL 을 'RL' 로 싣되 쓰지 않음), 같은 정적 월드(RTF 목표 1.0), 같은 스모크 20 개·같은 순서.
  세션마다 앞에서 탐침 scan·step (A5-3). 원자료 `/home/CAPYI_AI/runs/bonus1/b1-gzbench/A5_smoke/<tag>/`.
- **흔들기(shakedown)**: 본 스모크 전에 하네스 통합 결함을 찾으려고 RL 세션 2–3 에피소드를 따로 돈다 (태그 `shake*`, 결과는 이
  항목에 적되 스모크 표에 넣지 않는다). 흔들기에서 하네스를 고치면 커밋하고 그 커밋으로 본 스모크를 처음부터 돈다.
- 추가 기능 확인 (합격 조건 아님): `rl_handover` (반경 0.4 m) 세션을 스모크의 처음 6 개(실행 순서)로 한 번 — 세 제어기 선택이
  모두 돈다는 것만 본다. 근접 인계의 효과는 단계 C3 몫.
- 자원: gz_slot b1-gzbench, DOMAIN 33, IGN_PARTITION amr_ai_b1g, LOCALHOST_ONLY, 컨테이너 amr-ai-b1-gzbench-gz, 이미지 amr-ai/rl:0.3.
  공유 부하 하 예비값 — 세션 시작·끝 loadavg, 에피소드마다 loadavg.
- 소요 추정: 팔당 N 에피소드 시간 ≈ 기동(시스템·Nav2 수명주기, 세션 로그 시각) + N × 에피소드 벽시계 평균, 구간은 에피소드 벽시계의
  5–95 % 로. 주 집합과 스트레스 집합을 따로 본다 (스트레스는 시간 상한까지 갈 수 있다).

## 흔들기 (본 스모크 전, 23:51 KST 에 적는다 — 스모크 표에 넣지 않는다)

- 흔들기 1 `shake1_rl` (`eaf2ac8`) [측정, 실행 `A5_b1gzbench_shake1_rl`]: **실패** — `ros2 run amr_rl_bench gzbench_runner.py` 가
  "No executable found" (Write 로 만든 스크립트에 실행 권한이 없었다, symlink 설치). 스택은 정상, 에피소드 0. → `f015ee9`.
- 흔들기 2 `shake2_rl` (`f015ee9`) [측정, `A5_b1gzbench_shake2_rl`]: 탐침 2 개와 에피소드 3 개가 끝까지 돌았다 — 3/3 계약 성공,
  지면 진실 최종 0.042·0.031·0.028 m, 에피소드 벽시계 43.3·35.6·27.9 s, RTF 0.86–0.90. 찾은 하네스 결함 (고침 `f75f8f6`):
  1. 수명주기 확인(`ros2 lifecycle get`, 부를 때마다 새 DDS 참여자)이 get_state 응답을 잃어 300 s 동안 0/4 — 실제로는 기동 2 s 만에
     active 였다 (nav.log). → 계속 사는 lifecycle_watchdog 의 "모두 active" 로그를 기다리고, 파라미터·/clock 기록은 실행기 뒤로.
  2. **개입 짝짓기 결함**: safety 출력은 직전 입력을 ~20 ms 늦게 그대로 내는데, 기록 노드가 출력과 다음 입력을 같은 시각에 받으면
     "직전 입력" 짝이 틀려 램프 중 거짓 개입이 31·19·4 건 났다. → 출력 시각 창 [−60, +5] ms 의 입력 중 하나라도 같으면 개입 아님
     (PROTOCOL §5 의 정의는 그대로, 짝짓기만). 다시 계산하면 1·0·0 건 — 남은 1 건(0.30 s)은 같은 에피소드의 CRITICAL 0.52 s 와 같은 때다.
  3. 제어기가 받는 측정 속도(odometry/filtered)를 기록하지 않았다 → 기록 (전이 점검 5 의 관측 사슬).
  4. 스캔 탐침 (0,0,0) 은 좌우 대칭 통로 — A5-3 참조. 비대칭 자세 (−6, −7, 2.0) 를 더했다.
  5. 플랜트 식별 격자 하한(5 ms)에 걸림 → 1 ms 로 세분 — A5-3 참조.
- 사전 등록한 판정 H1–H4·기술 항목은 바꾸지 않는다. 본 스모크는 `f75f8f6` 로 처음부터 돈다.

## 결과

(스모크 뒤에 쓴다)

## 해석

## 다음
