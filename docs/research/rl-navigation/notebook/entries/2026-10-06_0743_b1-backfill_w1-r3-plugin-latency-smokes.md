# 1차 물결 r3 — ONNX Nav2 제어기 플러그인: 지연, 운동학 스모크, 첫 Gazebo 스모크 실패와 재실행

> **소급 기록** (작성 2026-10-06 19:41 KST, b1-backfill, 단계 A4). 원 작업: r3-controller, 2026-10-06 07:43–08:28 KST
> (빌드 07:43, ae0ef2c 08:03 … dc56d03 08:28). **학습된 정책 없이** 무작위 가중치 고정물과 손으로 만든 배관 정책(heuristic_b/c)으로만
> 돌렸다 — RL 성능 근거가 아니다. 수치는 `summary.json`·`latency_bench_2.log`·`load.txt`·`HEAD.txt` 에서 다시 읽었다.

- 시각: 2026-10-06 07:43 KST (원 작업) · 에이전트: r3-controller (기록 b1-backfill) · 단계: W1
- 코드: `feature/rl-controller` ae0ef2c·e085271·d5417a0·dc56d03 (계약 v1.1.0). 스모크는 d5417a0 + 미커밋 rl_smoke.py·rl_controller.cpp diff (뒤에 dc56d03 로 커밋)
- 관련 실행 id: `W1_r3_latency_bench_2`, `W1_r3_kin_follow_c_heur`, `W1_r3_kin_follow_c_heur2`, `W1_r3_kin_follow_b_heur`,
  `W1_r3_kin_follow_c_fixture`, `W1_r3_kin_follow_c_heur_obst`, `W1_r3_kin_nav_c_heur`, `W1_r3_gz_c_heur_a`, `W1_r3_gz_c_heur_b`

## 질문
계약 관측·마스크를 C++ 로 옮기고 ONNX Runtime 으로 정책을 돌리는 `nav2_core::Controller` 플러그인이 20 Hz 제어 예산 안에서 돌고,
Nav2·Gazebo 폐루프에 실제로 끼워지는가.

## 당시 가설과 예측
- 한 결정(관측 + 마스크 + 추론) p99 가 50 ms 예산의 1 % 미만이다.
- 정책을 쓸 수 없는 결정 주기(입력 무효·추론 실패·시간 초과·비유한 출력)는 내부 DWA 폴백이 받는다.

## 설정
- 플러그인: 코어(ROS·ORT 비의존, 관측 291·마스크 100·적분·BC 레이블·이탈 상태기계; T_sim·롤아웃·VO 는 amr_navigation_core 원본 재사용),
  결정 엔진(20 Hz 틱 → 10 Hz 결정, ZOH 가속도), OnnxPolicy (ORT C++ 1.23.2, 메타데이터·모양·dtype 검증), RLController(내부 DWAController 폴백).
- 지연: `latency_bench` intra_op 1, 5000 회 + 워밍업 200, 모델 6 종 × (golden d01–d17, 합성 최악 장면 720 빔·트랙 8·경로 600 점).
- 운동학 폐루프: ROS kinematic_sim + Nav2 + rl_navigation.launch.py (**Gazebo 아님**).
- Gazebo: gz_slot.sh, DOMAIN 204, IGN_PARTITION amr_ai_r3, ROS_LOCALHOST_ONLY=1, system.launch.py with_navigation:=false + 플러그인 런치(replace_dwa).

## 결과
**지연** [측정, `latency_bench_2.log`, HEAD e085271, loadavg 106 → 99 / 32 코어, 공유 부하 하 예비값]

| 모델 | golden 17 total p50 / p99 / max ms | 최악 장면 total p50 / p99 / max ms |
|---|---|---|
| fixture_b | 0.028 / 0.032 / 1.07 | 0.033 / 0.038 / 0.29 |
| fixture_c | 0.086 / 0.137 / 2.27 | 0.137 / 0.156 / 0.32 |
| size_b_mlp256 | 0.029 / 0.036 / 5.16 | 0.037 / 0.045 / **7.43** |
| size_c_mlp256 | 0.093 / 0.150 / 0.61 | 0.151 / 0.219 / 3.29 |
| size_c_cnn_gru128 | 0.150 / 0.196 / 0.34 | 0.171 / 0.214 / 1.21 |

p99 는 모든 조합에서 ≤ 0.22 ms (50 ms 예산의 0.44 %). 최댓값 7.43 ms 는 부하 선점으로 보인다 (원인 미검증).

**폐루프** [측정, `smoke/*/summary.json`]

| 실행 | 정책 | 결과 | 결정 (정책/폴백셀/DWA/정렬) | cycle p99 ms |
|---|---|---|---|---|
| kin follow C (×2) | heuristic_c | 성공 11.4·11.5 s, 5.90 m | 114·116 / 0 / 0 / 0 | 0.40·0.35 |
| kin follow B | heuristic_b | 성공 11.4 s | 115 / 0 / 0 / 0 | 0.26 |
| kin follow C fixture | 무작위 고정물 | **실패** status 6, 0 m 이동 | 0 / **200** / 0 / 0 | 0.32 |
| kin follow C 횡단 장애물 0.6 m/s | heuristic_c | 성공 12.3 s, 최소 동적 여유 0.431 m | 122 / 1 / 0 / 0 | 0.39 |
| kin navigate (3,−6) BT | heuristic_c | 성공 16.4 s | 147 / 0 / 0 / 17 | 0.36 |
| **Gazebo a** | heuristic_c | **실패**: follow 목표 거절, navigate 액션 서버 없음 | — | — |
| Gazebo b (watchdog) | heuristic_c | follow 5 m 성공 33.1 s; navigate (16,−8,π/2) 성공 42.8 s | follow 331/0/0/0, nav 389/0/0/39 | 0.43 |

- Gazebo a 원인 [측정, `gz_c_heur_a/nav.log:116`]: `failed to send response to /amr_01/controller_server/change_state (timeout)` —
  configure 는 끝났으나 lifecycle 응답이 유실돼 lifecycle_manager 가 멈췄다. loadavg 98 → 99. b 는 amr_bringup lifecycle_watchdog 를 붙여
  4/4 active (10 s) 뒤 성공. loadavg 84 → 132. RTF 미측정.

## 해석
- 플러그인 지연은 문제가 아니다 (p99 0.2 ms 대). Gazebo 지연 예산의 걱정은 scan→cmd 사슬·safety 관측 지연(diagnostics 1 Hz)·
  BT 1 Hz 재계획 쪽이다 (r3 이 위험으로 적음; 완성 계획 D4 전이 점검 6 종의 근거).
- 무작위 고정물 C 는 200 결정 전부 폴백 셀(제동)을 골라 움직이지 않았다 — 마스크는 안전을 지키지만 목표 도달은 물려받지 않는다.
- 공유 부하에서 lifecycle 응답 유실은 정책과 무관한 하네스 실패다 — 이후 Gazebo 평가는 watchdog·부하 기록이 필요하다.
- 이 모든 실행은 학습되지 않은 정책이다. **학습 정책의 Gazebo 폐루프는 1·2차 물결 모두 미측정**이다.

## 무엇을 바꾸게 했나
- 리뷰가 DWA 폴백 전환의 속도 점프를 찾았다 → 다음 항목 (`…_0911_…dwa-fallback-jump-fix.md`).
- 통합 하네스(run_integration.sh)에 nav params_file 전달이 없어 시나리오 판정으로 돌리지 못했다 → 완성 계획 A5 (Gazebo 다회 평가 하네스).

## 원자료
- `/home/CAPYI_AI/runs/r3-controller/smoke/{kin_*,gz_c_heur_a,gz_c_heur_b}/`, 모델 `/home/CAPYI_AI/runs/r3-controller/models/`
- 로그 `/home/CAPYI_AI/logs/agents/r3-controller/` (latency_bench_{1,2}.log, kin_smokes.log, test*.log)

## WAVE 보고와 원자료의 차이
- 없음 (지연 p99·최댓값, 스모크 결과·결정 수·시간 모두 원자료와 같다). `kin_follow_c_heur/HEAD.txt` 는 빈 파일이라 그 run 의 커밋은 미상이다 (리뷰 지적).
