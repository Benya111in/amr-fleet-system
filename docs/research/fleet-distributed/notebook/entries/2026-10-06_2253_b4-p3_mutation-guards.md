# P3 변이 검사 — 에이전트 가드를 되돌리면 시험이 잡는가 (사전 등록)

- 시각: 2026-10-06 22:53 KST · 에이전트: b4-p3 · 단계: P3
- 코드: `c23c511` (브랜치 `feature/b4-p3`)
- 관련 실행 id: P3_mutation_v1 · P3_mutation_v2 · P3_mutation_ros

## 질문
P3 에서 넣은 가드(모드 · 에포크 펜스 · 작업 잠금 어댑터 · 가짜 실행기 · 지연 확정 · 해제 게이트)를 하나씩 되돌리면 순수 시험(test_agent_*.py)이 실패하는가 — 시험이 공허하지 않은가.

## 가설과 예측
- 예측: 변이 23 종 모두 KILLED. 단 ML1(잠금 없이 GRANTED)은 손실만 건 협상 시험으로는 잡히지 않고
  (CBAA 가 독립 손실에서 스스로 중복을 거의 내지 않는다 — f3 [계산]) 분할 시험만 잡을 것이다 (개발 중 확인, 이 실행으로 다시 잰다).
- 활성 변이(MC4 ACQUIRE 멱등 제거)는 살아남을 수 있다 — 안전이 아니라 나이(기아 순서)를 잃는 것이라 지금 시험이 보지 않는다.
- 가설을 버리는 결과: 안전 가드(MF1·MF2·MF4·ML1·ML2·ME1·ME2·MV4·MC2·MC5) 중 하나라도 SURVIVED → 시험 보강 전에는 그 가드를 "시험으로 확인" 이라고 쓰지 않는다.

## 설정
- 도구: /home/CAPYI_AI/runs/b24/b4-p3/mut/mutate_p3.py (저장소 밖 — 소유 경로 밖이라). 변이마다 src/amr_fleet 사본에
  문자열 치환 하나, 지정 시험 파일을 pytest -x 로. 이미지 amr-ai/rl:0.3, cpu_slot 경유.
- 판정: 시험이 실패(rc ≠ 0) = KILLED, 통과 = SURVIVED, 치환 대상이 1 곳이 아니면 BAD_MUTANT (도구 결함으로 따로 센다).
- 변이 목록 · 설명은 도구 파일의 MUTANTS 표 (23 종).

### 추가 사전 기록 (2026-10-06 23:20, 1 · 2 회차 결과를 본 뒤 · ROS 수준 변이 실행 전)
- 순수 변이는 실제로 24 종이었다 (위 "23 종" 은 셈 실수). 1 회차 전에 활성 수정 두 건의 변이 MC6 · MC7 을 더해 26 종.
- ROS 수준 변이 5 종을 더한다: C1 ZoneGuard 상태 신선도 무시, C2 holdsAll 이 "하나라도" 로, C3 CommitTaskLock 이 denied 를
  무시, C4 AcquireZoneChain halt 가 CANCEL 을 안 보냄 (gtest `test_zone_bt_nodes`), N1 노드 프록시가 펜스 · 모드 검사를
  건너뜀 (launch 시험 1 회). 예측: 모두 KILLED. 도구 `/home/CAPYI_AI/runs/b24/b4-p3/mut_ros/run.sh`.

## 결과

[측정] 원자료 `/home/CAPYI_AI/runs/b24/b4-p3/mut/` (mutation_v1.json · run_v1.log · mutation_v2.json · run_v2.log),
`/home/CAPYI_AI/runs/b24/b4-p3/mut_ros/` (results.txt · *.gtest.log · N1.pytest.log · N1_raw/). 결정적 시험이라 시드 무관
(N1 의 launch 시험만 시드 1).

| 변이 | 무엇을 되돌렸나 | 1 회차 (7db1beb 전후 작업 트리) | 2 회차 (5e6ac59) |
|---|---|---|---|
| MA1 | 혼자 DEAD 로 DEGRADED (과반 의심 가드 제거) | SURVIVED | KILLED |
| MA2 | RESYNC 히스테리시스 제거 | KILLED | KILLED |
| MA3 | 관찰자 서버로도 NORMAL 복귀 | KILLED | KILLED |
| MA4 | ISOLATED 덮개 제거 | KILLED | KILLED |
| MA5 | 중앙 할당을 모든 모드에서 받음 | KILLED | KILLED |
| MF1 | 중앙 명령 에포크 펜스 제거 | KILLED | KILLED |
| MF2 | 하트비트 에포크 펜스 제거 (옛 권위자가 생존 신호) | KILLED | KILLED |
| MF3 | 관찰자 하트비트가 펜스를 올림 (되먹임) | KILLED | KILLED |
| MF4 | 펜스 비교 무력화 | KILLED | KILLED |
| ML1 | 잠금 없이 GRANTED (낙관) | KILLED | KILLED |
| ML2 | 완료 뒤 확정 잠금 해제 | KILLED | KILLED |
| ML3 | 내 옛 확정 표 busy 를 거부로 셈 | KILLED | KILLED |
| ML4 | 거부 문턱 1 (과반 불가 전에 포기) | SURVIVED | KILLED |
| ME1 | 잠정 과반(확정 전)에서 픽업 시작 | SURVIVED | KILLED |
| ME2 | 거부돼도 픽업 시작 | SURVIVED | KILLED |
| MV1 | hold_enter 무시 (곧바로 ENTER) | KILLED | KILLED |
| MV2 | VOTED 에서 commit 이 ENTER 를 안 보냄 | KILLED | KILLED |
| MV3 | NACK(busy) 를 세지 않음 | KILLED | KILLED |
| MV4 | 과반 ENTER_ACK 대신 1 개 (f1 M 계열과 같은 가드, hold 경로로 재확인) | KILLED | KILLED |
| MC1 | ISOLATED 에서도 새 진입 요청 | KILLED | KILLED |
| MC2 | 해제 게이트 제거 (몸체 판정 없이 RELEASE) | KILLED | KILLED |
| MC3 | 재기동 고아 보유 추적 제거 (영구 점유) | KILLED | KILLED |
| MC4 | ACQUIRE 멱등 제거 (재요청이 나이를 잃음 — 활성) | SURVIVED | KILLED |
| MC5 | 재기동 때 자세를 모르면 보유를 버림 (fail-open) | KILLED | KILLED |
| MC6 | DEGRADED 를 벗어날 때 남은 잠금 요청을 놓지 않음 (활성) | KILLED | KILLED |
| MC7 | 중앙 작업이 끝나도 확정 전 잠금 요청을 놓지 않음 (활성) | KILLED | KILLED |

- 1 회차 21/26. 살아남은 다섯: MA1(혼자 DEAD 로 DEGRADED — NORMAL 에서 곧바로 DEAD 인 경우를 시험이 안 봤다),
  ML4(거부 문턱 1), ME1 · ME2(가짜 실행기 단위 시험 없음 — 손실 · 분할 협상 시험으로는 잠정 과반 · 거부 상태 픽업이
  드러나지 않았다), MC4(ACQUIRE 멱등 — 예측대로 활성 변이). 다섯을 겨냥한 시험을 더했다 (5e6ac59).
- 2 회차 **26/26**.
- 1 회차는 변이마다 그 순간의 작업 트리를 복사했고 실행 도중 활성 수정(7db1beb)이 들어갔다 — 판정은 2 회차를 쓴다.
- ROS 수준 (P3_mutation_ros, 6471507): C1 · C2 · C3 · C4 · N1 **5/5 KILLED**. N1 은 ③ 에서 잡혔다 — 낮은 epoch 명령 3 건이
  가짜 실행기에 닿았다 (원래 코드는 0).

## 해석

- 사전 등록한 안전 가드는 1 회차에서도 모두 잡혔다 (MF1 · MF2 · MF4 · ML1 · ML2 · MV4 · MC2 · MC5) — 단 ME1 · ME2 는
  가짜 실행기의 안전 가드인데 1 회차에 살아남았다. 그 둘은 "시험으로 확인" 이라고 쓸 수 있는 것이 2 회차부터다.
- 변이 생존이 "안전 무관" 을 증명하지 않는다 (f1 리뷰 교훈) — MC4 는 활성 변이라고 판단했지만 시험을 더해 잡게 했다.
- 말하지 않는 것: 변이 목록 밖의 결함. 투표 코어 자체의 변이는 f1 의 31 종(별도)이 맡는다.

## 다음

- 최종 회귀 (colcon -Werror + 세 패키지 시험) 뒤 p3.md §6.4 에 반영.

## 리뷰 반영 (2026-10-07, 항목 `2026-10-06_2348_b4-p3_review-fix-proxy-leak-contention.md`)

- 리뷰 변이: RM1(가짜 실행기가 held 를 기다리지 않음)은 순수 시험에서 KILLED 였지만 **launch 시험(④)에서는 SURVIVED** — 이 항목의
  "ROS 수준 5/5" 는 프록시 펜스(N1)와 존 BT 헤더(C1–C4)만 본 것이고 작업 잠금의 ROS 경로는 변이로 확인한 적이 없었다.
- 고친 코드에서: 앞 회차 26 종 + 새 가드 MR1–MR11 = **37/37 KILLED** (P3R_mutation_v3). ROS 수준: RM1 은 새 ④c 에서 KILLED,
  N2(실행기 거부를 수락으로)는 ④b 에서 KILLED, RM2(ENTER 가 다른 요청의 잠정 표로도 확정)는 ④b · ④c 둘 다 **SURVIVED** (새 항목 결과).
