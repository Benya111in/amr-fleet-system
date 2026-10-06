# P4-5 리뷰 반영: 현행 latched hold 비교군 스윕과 작업 잠금 정족수 변이 검사 (사전 등록)

- 시각: 2026-10-07 00:10 KST · 에이전트: b4-p4 · 단계: P4
- 코드: `1300540` (브랜치 `feature/b4-p4`) — 이 항목의 가설·설정·판정 규칙은 스윕·변이 검사 **전에** 커밋한다
- 관련 실행 id: P4-test-03 (시험 48), P4-hold-01 (hold 비교군 스윕), P4-hold-01r (기존 비교군 digest 재현),
  P4-mut-03 (변이 검사 3 차)
- 근거: 리뷰 `/home/CAPYI_AI/runs/b24/b4-p4/review/` (차단 B1 · B2), `docs/research/fleet-traffic-deadlock/hold_failsafe.md`
  D2 · D4 · D5 · D6 · §4 위험 1, `docs/research/fleet-distributed/p0.md` §5 (도크 구역 결정점 거리 미달)

## 질문

1. P4 1 차의 중앙 교통(A1–A3 · A5)은 "마지막 허가 구역 집합에만 진입"(허가 목록, 서버가 죽으면 fail-closed)이었다.
   현행 traffic_manager 의 **latched `traffic/hold`(거부 목록)** 의미로 바꾸면, 서버 충돌(F1) · 분할(F5)에서 현행의 처리량과
   안전(동시 점유)이 어떻게 달라지고, P1 판정(A9 − A1)과 2×2 해석은 유지되는가?
2. 리뷰 변이 X4(작업 잠금 자기 정족수)와 X3(과반 대신 자기 의심으로 폴백)을, 새 시험이 잡는가?

## 리뷰가 지적한 것 (B1 · B2)과 이 항목 전에 이미 본 것 (사전 노출 공개)

- B1: `robot.py` 의 중앙 게이트가 fail-closed 라서 A1 F1 의 Q(t) 360 s 뒤 0, 끝 고착 116/120 이 "현행의 성질" 처럼 보고됐다.
  현행은 관리자가 죽어도 마지막 hold 를 유지하고 false 였던 로봇은 관리 없이 움직인다(hold_failsafe.md:93-94, §4 위험 1).
  리뷰의 근사 민감도(충돌 순간 정지선 대기 로봇만 선다, A1 F1 손실 0.2 쌍 30): 완료 4.23(모형 3.70), 동시 점유 2/30 실행,
  A9 − A1h +17.3 [16.0, 18.6] 30/30.
- B2: X4 를 시험 22 개가 모두 통과했다. 같은 변이로 F5 셀을 다시 돌리면 A6 중복 26/120 실행.
- 이 항목을 쓰기 전에 **시험 시드(101 · 102 · 11, 지평선 600 s)** 로 hold 모드 스모크를 돌려 본 것: A1h F1 seed 101 완료 7
  (A1 4) · 동시 점유 2 사건, A5h F0 손실 0 seed 102 에 도크 구역 동시 점유 1 사건(원인 추적: 도크 a 를 나와 도크 b 쪽으로
  가는 로봇의 요청이 "구역 안에서는 다음 구역을 묶이지 않으면 요청하지 않는다" 규칙과 approach 1.5 m 때문에 늦어, 거절 hold 를
  받았을 때 이미 멈출 수 없는 거리였다 — p0.md §5 가 적은 "도크 구역 결정점 거리 미달" 과 같은 기제), F0 손실 0.4 seed 101
  A1h · A5h 각 2 사건(도크). **스윕 시드 1–30 의 hold 결과는 아직 보지 않았다.**

## hold 모드 설계 (코드 `fleetsim/central.py` · `robot.py`, 현행 코드와 맞춘 것과 다른 것)

| 항목 | 모형 | 현행 근거 |
|---|---|---|
| hold 값 | 토큰 요청이 이번 주기에 허가되지 않은 로봇만 true, 나머지 관측된 로봇(stale · lost 포함, 요청이 없다)은 false | `traffic_manager._compose` 토큰 hold, `_tokens`(stale 은 경로 없음 → 요청 없음) |
| 송신 | 값이 바뀔 때만, 신뢰 경로(유실 시 0.2 s 재전송), seq 로 옛 값 무시 | `traffic_manager_node._apply` `hold != r.hold`, `LATCHED_QOS` RELIABLE/TRANSIENT_LOCAL |
| 초기값 | 내지 않는다, 처음 평가한 값을 낸다 · 관측 못 한 로봇은 3 s 뒤 false | D5 · D6 |
| 로봇 | 첫 값 전 대기, 마지막 값 유지(서버 사망에도), true 면 그 자리에서 최대 감속 1.0 m/s² 로 정지 | D2 · D4, 실행기 hold → 주행 goal 취소 |
| 넣지 않은 것 | 교차 예측 hold, 사건(양보 · 포켓) hold — 허가 목록 모형과 같다 | p4.md §1 한계 |

허가 목록 모형과의 차이는 두 가지가 섞여 있다: (a) 게이트 의미(허가 목록 대 거부 목록), (b) 송신(매 주기 상태 대 바뀔 때만
신뢰 경로). 둘 다 "현행에 더 가까운 쪽" 이라 함께 바꿨다 — 효과를 둘로 가르지 않는다.

## 가설과 예측 (결과를 보기 전에)

| id | 셀 | 예측 | 버리는 조건 |
|---|---|---|---|
| H-1 | F1, 손실 4 수준, A1h − A1 완료 | 모든 손실에서 > 0 (쌍차 CI 하한 > 0). 손실 0.2 의 A1h 평균은 4.2–6.2 (리뷰 범위) | 어느 손실에서든 CI 상한 ≤ 0 이면 "fail-open 이 진행 중 작업을 끝낸다" 기각 |
| H-2 | F1, A1h 고장 뒤 동시 점유 | 손실 수준마다 30 실행 중 1 실행 이상 (A1 은 0/120) | 120 실행 모두 0 이면 이 모형에서는 "충돌 뒤 안전 구멍" 을 보이지 못한 것으로 적는다 |
| H-3 | F5, A1h 동시 점유 | 4 손실 수준 합 120 실행 중 1 실행 이상 (분할된 소수쪽이 마지막 hold=false 로 움직인다) | 0 이면 상한만 적는다 |
| H-4 | F0, 손실 0, A1h · A2h · A5h | 동시 점유는 도크 구역(결정점 거리 미달)에서만, 그 밖 구역 0. 도크 사건 실행 비율 1–20 % | 도크 밖 구역 사건이 손실 0 에서 나오면 모형 결함을 먼저 의심하고 사건을 추적한다 |
| H-5 | F0, 손실 0.4 | 늦은 hold(관측 유실) 때문에 동시 점유 실행이 손실 0 보다 많다 | — (기술만) |
| H-6 | P1h: A9 − A1h, 손실 0.2 F1, 완료 | 우월 유지 (P1 규칙: Wilcoxon p < 0.05 · CI 하한 > 0 · k+ ≥ 25/30), 크기 +12–17 | 판정 불가면 P1 해석을 "모형의 fail-closed 기준에서만" 으로 낮춘다 |
| H-7 | 2×2 hold (CC A2h · CV A4 · DC A5h · DV A6), F1 완료 | DC(A5h) 가 F0 수준(≥ 15)에 가깝고 동시 점유 실행 ≥ 50 % → 허가 목록 2×2 의 "한쪽만 분산하면 못 이어 간다" 는 hold 의미에서 성립하지 않는다(처리량은 잇지만 안전하지 않다) | DC 완료 < 10 이면 이 예측 기각 |
| H-8 | F6, A1h − A1 완료 | 모든 손실에서 |평균 쌍차| < 1.0 (서버는 산다) | — |

판정 규칙: H-6 만 판정(단일 비교 — 1 차 Holm 가족 밖이라 Wilcoxon p 그대로). 나머지는 효과크기 · 95 % CI · k/n 과 실행 수만
적고 p 를 주장하지 않는다. 0 건 칸은 rule of three 상한(n = 30 이면 9.5 %, 120 이면 2.5 %).

변이 검사 P4-mut-03 (예측: 모두 잡힘). 판정 = 원본 사본에 변이 하나를 넣고 `test_fleetsim_sim.py` **전체**(−x 없음)를 돌려 종료
코드 ≠ 0 이면 잡힘. 원본 사본도 같은 방식으로 한 번 돌려 통과를 확인한다.

| 변이 | 무엇 | 잡을 시험 (예측) |
|---|---|---|
| X4 | 작업 잠금 `VoteConfig((rid,))` — 자기 표만으로 잠김 (리뷰) | `test_task_lock_quorum_prevents_partition_duplicates` |
| X3 | 폴백 전환을 과반 대신 자기 하트비트 DEAD 로 (리뷰, 비차단 생존) | `test_no_fallback_without_server_loss[F5]` |
| H1 | hold 모드 정지 게이트 끔 (hold=true 여도 감) | `test_hold_true_stops_robot_with_bounded_decel_and_false_releases` |
| H2 | 서버가 hold=true 를 내지 않음 | `test_hold_mode_protects_zones_meeting_decision_distance` (hold_true_pub) |
| H3 | hold 게이트를 서버 생존 시에만 통과 (fail-closed 로 되돌림) | `test_latched_hold_fails_open_after_server_crash` |
| H4 | hold 수신이 seq 를 무시 | `test_hold_follows_latest_seq_and_waits_before_first_value` |

## 설정

- P4-hold-01: `--arms A1h_nearest_hold,A2h_hungarian_hold,A5h_cbaa_hold --losses 0,0.05,0.2,0.4 --faults F0,F1,F5,F6
  --seeds 1-30 --rate 0.022222222222222223` (지평선 1200 s, 고장 300 s, F5 분할 180 s, rep 0) = 1,440 실행. 코드 `1300540` 을
  `git archive` 로 뽑아 고정, `cpu_slot.sh b4-p4`, 컨테이너 amr-ai/rl:0.3, PYTHONHASHSEED=0, 작업자 6.
- P4-hold-01r: 기존 비교군 재현 `--arms A1_nearest_central,A2_hungarian_central,A5_cbaa_central --losses 0.2 --faults F1,F5`
  (180 실행) — digest 가 P4-main-01(`e34fa90`) 과 같아야 짝(A1 · A2 · A5)을 P4-main-01 에서 그대로 쓴다. 다르면 짝도 다시 돈다.
- 분석: `python3 -m amr_fleet.fleetsim.report hold --runs P4-main-01/out/runs.jsonl P4-hold-01/out/runs.jsonl --out …/report`.
- 변이: `P4-mut-03/mutate.py` (cpu_slot, 컨테이너, 코드 `1300540` git archive).

## 결과

(스윕 뒤 채운다)

## 해석

(스윕 뒤 채운다)

## 다음

(스윕 뒤 채운다)
