# 구역 투표 코어 검증 (보너스 4 P1)

- 대상: `src/amr_fleet/amr_fleet/distributed/` (protocol.md 의 구현)
- 시험: `src/amr_fleet/test/test_dist_*.py`
- 변이 도구: `docs/research/fleet-distributed/tools/mutate.py`
- 주장 수준: **"시험됨 (n 회, 위반 0)"** — 모델 검사(TLC, f2)나 증명이 아니다. 수작업 안전 논증은 protocol.md 2 절.

## 1. 층

| 층 | 무엇 | 파일 |
|---|---|---|
| 단위 | 투표자 · 요청자 전이 하나하나, epoch, 재시작, 영속 fsync 순서, (sec, nanosec) 순서, 감지기, WFG | `test_dist_vote_core.py` `test_dist_store.py` `test_dist_ordering.py` `test_dist_failure_detector.py` `test_dist_wfg.py` |
| 상태기계 | Hypothesis `RuleBasedStateMachine` 위 5 대 모의 플릿(`simnet.Cluster`) | `test_dist_vote_stateful.py::test_vote_core_state_machine` |
| 대조 | 보고서가 기각한 "생존 멤버 전원 REPLY" 변형(`RaLiveAgent`)을 같은 하네스에 올려 반례가 나오는지 | `test_dist_vote_stateful.py::test_rejected_*` |
| 변이 | 가드를 하나씩 되돌린 사본에서 위 시험이 실패하는지 | `tools/mutate.py` |

## 2. 상태기계 시험

모의 플릿: 로봇 5 대(투표자 = 요청자), 자원 3 개(`zone:a` `zone:b` `pocket:p`), 사슬 4 종
(`{a}` `{b}` `{a,b}` `{p,b}`), 잠정 기한 4 s, 재전송 0.5 s, 묘비 상한 8.

규칙 (Hypothesis 가 임의로 엮는다):

| 분류 | 규칙 |
|---|---|
| 로봇 | `request` · `contend`(2–5 대가 같은 사슬 동시 요청) · `enter` · `enter_all` · `exit` · `exit_one` · `give_up` |
| 네트워크 | `deliver(i)`(임의 순서 = 재정렬 · 지연) · `drop(i)` · `duplicate(i)` · `deliver_all` · `chaos_round`(무작위 순서 + 손실 0/0.1/0.3 + 중복 0/0.1 + 다음 라운드로 미룸 0/0.2/0.5) · `protocol_round`(손실 없는 라운드) |
| 시계 | `advance(r)`(로봇별, 어긋남 상한 12 s) · `advance_all` · `tick` · `tick_all` · `reset_clock`(시각 역행 → epoch, 실행당 ≤ 2) |
| 장애 | `crash`(휘발 상태 버림, 실행당 ≤ 3) · `restart`(저장소에서 재기동, incarnation + 1) |
| 멤버십 | `set_view`(각자 다른 생존 멤버 관측, 자기 + 2–4 대) · `clear_view` |

매 단계 불변식 (`simnet.Cluster.check`):

| id | 불변식 | 비고 |
|---|---|---|
| I1 | 자원마다 허가받은 로봇 ≤ 1 | **과제의 "한 구역 동시 허가 ≤ 1"**. 크래시한 로봇은 디스크의 HELD 로 센다 |
| I2 | 자원마다 몸체가 안에 있는 로봇 ≤ 1 | 모의 물리 |
| I3 | 안에 있는 로봇은 그 자원을 허가받고 있다 | |
| I4 | 허가 (로봇, 요청, 자원) 마다 그 요청의 확정 표를 가진 투표자 ≥ 3 | 안전 논증의 귀납 보조 불변식. 위반이 I1 보다 짧은 trace 로 드러난다 |
| I5 | 낮은 epoch 메시지가 받는 쪽 상태를 바꾼 횟수 = 0 | **과제의 "낮은 epoch 수용 0"**. 배달 전후 `protocol_state()` 비교 |

결과 (2026-10-06, `amr-ai/rl:0.3`, hypothesis 6.168.4, 공유 서버 loadavg 30–60 / 32 코어 — 시간은 예비값).
로그는 저장소 밖 `/home/CAPYI_AI/logs/agents/f1-vote/` 에 있다.

| 실행 | 설정 | 결과 | 단계 (허가 상태 / 경합) |
|---|---|---|---|
| CI 기본 (b3269d6) | 400 예제 × ≤80 단계, derandomize | 위반 0 | 26 787 (4 978 / 3 438) |
| 깊은 탐색 (cae54be) | 무작위 시드 11·22·33·44·55 × 3000 × ≤100 | 5/5 위반 0 | 1 289 337 (294 826 / 196 927) |
| 깊은 탐색 (b3269d6, 손상 규칙 포함) | 같은 설정 | 5/5 위반 0 | 1 262 978 (241 186 / 157 543) |
| 기각 변형 (b3269d6) | 무작위 시드 0–9 × 2000 상한 × 60 | **10/10 반례 발견**, 첫 위반 예제 7–403 (중앙값 122.5) | — |

"허가 상태" = 살아 있는 로봇 중 하나라도 허가를 쥔 단계, "경합" = 남이 허가받은 자원을 다른 로봇이 요청 중인 단계.
전체의 약 19 % · 12 % 다 — 위반 0 의 무게는 이 비율만큼이다.

## 3. 시험 설계에서 배운 것

- **1 차 상태기계(개별 배달 규칙만)는 정족수 2/5 변이를 놓쳤다.** 표가 갈리는 끼어들기 순서(REQUEST_A 가 a·b 에,
  REQUEST_E 가 c·d 에 먼저 닿음)를 `deliver(i)` 로 하나씩 고르는 확률이 너무 낮았다. 무작위 순서 라운드(`chaos_round`)와
  동시 경합(`contend`)을 넣은 뒤 잡혔다. 시험이 통과하는 것은 시험이 공허하지 않다는 증거가 아니다.
- 허가 상태에 닿은 단계 비율을 함께 센다 (`AMR_DIST_HYP_STATS`). 이 비율이 낮으면 통과는 의미가 약하다.
- 잠정 표 배타만 되돌린 변이(M2)는 안전 위반을 만들지 못한다 — 2 단계 확정(ENTER 때 "그 요청에 묶인 잠정 표" 를
  확인)이 배타를 다시 세우기 때문이다. 확정 표까지 덮어쓰는 변이(M2b)는 잡힌다. 즉 **안전을 지는 것은 확정 표의
  배타**이고, 잠정 표의 배타는 활성(헛된 ENTER 줄이기)용이다.
- 기각 변형의 축소 반례는 보고서의 대칭 반례({A,B,C} 대 {C,D,E})보다 약한 조건이다: **한 로봇의 관측만 어긋나도**
  (r3 이 r1 을 못 본다고 믿음) 두 대가 들어간다. `test_rejected_variant_shrunk_one_sided_counterexample` 에 고정.

## 4. 변이 검사 요약 (CI 예산, b3269d6)

31 개 변이(투표 코어 20 · 감지기 6 · WFG 5) 모두 단위 ∪ 상태기계 시험 중 하나 이상이 잡았다. 그러나 **상태기계
단독의 힘은 경계 수준이다**:

- 놓침 (안전 무관 — 등가 변이로 판단): M2 잠정 표 배타 제거, M13 사슬 부분 부여(둘 다 2 단계 확정이 가린다),
  M14 묘비 끄기, M15 incarnation 고정(요청 ID + 자원 집합 확인이 같은 일을 한다).
- 놓침 (안전 영향 있음): **M8 요청자가 ENTER_ACK 의 요청 ID 를 확인하지 않음** — CI 예산 · 깊은 예산(3000 × 100) 모두
  놓쳤다. 손으로 만든 결정적 시험 `test_stale_enter_ack_of_old_request_is_not_counted` 로 메웠다.
- 예산 · 규칙 구성에 따라 잡히기도 놓치기도 함: M9(ENTER_ACK 정족수 −1, I4 없이), M11(ENTER 가 남의 표 확정).
- 상태기계 밖: M17–M19 FileStore fsync 순서는 단위 시험(호출 순서 기록)만 잡는다 — 상태기계는 MemoryStore.

## 5. 범위 밖 (주장하지 않는 것)

- 활성(교착 · 라이브락 없음, 기아 없음): 손실 없는 망에서 5 대가 차례로 모두 들어가는 단위 시험 1 건뿐이다.
- 물리 계층: 제동 · 위치추정 · 점대칭 별칭. "허가 없이 진입선을 넘지 않는다" 는 로봇 쪽 게이트(P3)와 Gazebo 시험의 몫.
- ROS 전송(QoS · DDS 손실 특성)과 실제 sim-time(`/clock`) 지연: P3.
- 비잔틴 고장, 저장 매체의 거짓 fsync.
