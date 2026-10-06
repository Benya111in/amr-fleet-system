# A2-5 목표 yaw 정렬이 지나치는 결함 — 저크·지연을 넣은 정지 속도로 고치기

- 시각: 2026-10-06 20:22 KST · 에이전트: b1-plugin · 단계: A2
- 코드: `9f18023` (브랜치 `feature/b1-plugin`) — 고치기 전
- 관련 실행 id: (끝나면 채운다 — runs/b1-plugin.jsonl)

## 질문

Gazebo 스모크(A2-4)에서 목표 위치에는 0.03–0.08 m 로 닿는데 끝 yaw 가 요청 yaw 와 0.16–0.26 rad 어긋났다 (지면 진실로도 같음,
위치추정 오차 ≤ 0.03 rad — 3 회차 자세 시계열). 배포 goal checker 의 `yaw_goal_tolerance` 는 0.05 rad (N:99, 실린 값 확인).
플러그인의 제자리 정렬이 원인인가, 그리고 고칠 수 있는가?

## 진단 (결과를 본 뒤의 사후 분석 — 사실만)

[측정, `/home/CAPYI_AI/runs/bonus1/b1-plugin/A2_gz_smoke/smoke/gz_v12s1_static_c/goal_1/series.json`]
목표 1 에서 로봇은 yaw 2.06 rad 로 목표 위치에 닿았다 (목표 yaw 0). 정렬 결정(source 3) 이 ω 를 α·Δt 씩 −1.5 rad/s 까지
올렸다가 23.30 s 부터 줄였는데, yaw 가 0 을 지날 때(≈ 23.95 s) 명령 ω 는 −0.28 이고 실제 회전(자세 차분)은 약 −0.9 rad/s 였다.
goal checker 가 |오차| < 0.05 에서 성공을 내고 명령이 0 이 된 뒤에도 하류 velocity_profiler (각저크 6 rad/s³, α 2 rad/s²)와
서보 지연 때문에 계속 돌아 −0.163 rad 에서 섰다.
원인: `DecisionEngine` 의 정렬 속도 상한이 사다리꼴 √(2·α·|e|) 이다 — 하류 저크 필터의 램프 시간(α/j = 0.33 s)과 명령→실속도
지연을 무시한다. amr_navigation DWA 는 선속도 목표 접근에서 같은 이유로 사다리꼴을 버리고
`SpeedProfile::maxSpeedForStop(d, a, j, approach_latency 0.3 s)` 를 쓴다 (`dwa.cpp:650-653`, nav2_params.yaml:140-142 주석:
"사다리꼴 √(2ad) 는 하류 저크 필터 때문에 지나친다").

## 가설과 예측 (고치기 전에 적는다)

- H: 정렬 속도 상한을 `maxSpeedForStop(|e|, α 2.0, j_ang 6.0, t_c)` 로 바꾸면 (t_c = 정렬 지연 파라미터, 기본 0.3 s —
  α/2j 0.17 + 서보 0.04–0.12 + 결정 ZOH 반 주기 0.05), goal checker 가 성공을 내는 순간의 잔여 회전이 작아 끝 오차가 0.05 안에 든다.
- 예측 1 (오프라인 폐루프 시험, 결과 전 고정): 엔진(20 Hz) → amr_navigation `VelocityProfiler` 코어(50 Hz, PI 끔) → 1 차 지연
  0.08 s + 순수 지연 0.04 s 플랜트, goal checker 흉내(|오차| < 0.05 이면 명령 0)로 초기 오차 ±0.3·±1.0·±2.06·±3.0 rad 에서
  **정지 뒤 |오차| ≤ 0.05** (8 사례 전부). 옛 규칙(√(2αe))은 이 시험에서 실패해야 한다 (시험이 결함을 가르는지 확인).
- 예측 2 (Gazebo 재실행): 목표 5 개 중 ≥ 4 개의 끝 yaw 오차(지면 진실) ≤ 0.05 rad. 정렬 시간은 늘어난다 (보고).
- 기각: 시험에서 한 사례라도 0.05 를 넘으면 지연 값을 맞추기 전에 원인을 다시 본다 (값을 결과에 맞춰 고르는 것 금지 —
  t_c 는 위 근거로 정한 0.3 s 한 값만 시험한다).

## 설정

- 고칠 곳: `src/amr_rl_controller/src/core/decision_engine.cpp` 정렬 분기, 파라미터 `rl.align_latency` (기본 0.3 s).
- 시험: `test_decision_engine` 에 폐루프 정렬 시험 추가 (amr_navigation_core 의 VelocityProfiler·SpeedProfile 사용).
- Gazebo 재실행: A2-4 와 같은 설정(정적 월드, 목표 A–E, v12_s1), 고친 커밋으로.

## 결과

(실행 뒤 채운다)

## 해석

(실행 뒤 채운다)

## 다음

(실행 뒤 채운다)
