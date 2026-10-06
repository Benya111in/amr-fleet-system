# P3 리뷰 반영 — 중앙 프록시 작업 상태 누수와 launch ④ 경합 단계

- 시각: 2026-10-06 23:48 KST · 에이전트: b4-p3 · 단계: P3 (리뷰 반영)
- 코드: `81444cd` (리뷰 대상, 브랜치 `feature/b4-p3`) — 사전 등록 시점. 결과 절에 실제 측정 커밋을 적는다
- 관련 실행 id: P3R_probe_review_after, P3R_dev_main_s1, P3R_dev_part_s1, P3R_launch_s1–s3, P3R_part_s1–s3,
  P3R_mutation_v3, P3R_mut_ros_try1(실패), P3R_mut_RM1_part, P3R_mut_RM2_part, P3R_mut_N2_main, P3R_mut_RM2_main,
  P3R_sweep_tasklock_v2, P3R_probe_enter_partition, P3R_final_regression
- 근거: 리뷰 "b4-p3 리뷰: 보너스 4 P3 분산 플릿 ROS 통합" (판정 fix_needed, 차단 2건). 리뷰 원자료
  `/home/CAPYI_AI/runs/b24/b4-p3/review/` (탐침 `probe/p1_proxy_leak.py`·`p1b_failed_task_reassign.py`, 변이 RM1–RM3 `out/status.txt`)

## 질문

1. (차단 1) 중앙 작업 프록시 경로에서 실행기가 거부하거나 픽업 전에 FAILED 로 끝난 작업을, 관리자가 다른 로봇에
   다시 줄 때 그 로봇이 받아서 확정(HELD)까지 갈 수 있는가. 리뷰 탐침은 "못 간다"(150 s 동안 HELD 안 됨, 4/5 로봇의
   `taken` 에 남음)를 보였다.
2. (차단 2) launch 시험 ④ 가 작업 ID 잠금을 ROS 수준에서 실제로 시험하게 하려면 — 경합이 생기는 단계를 넣었을 때
   잠금이 확정을 하나로 막는가, 그리고 잠금을 우회하는 ROS 수준 변이(RM1)가 그 시험에서 잡히는가.

## 리뷰 확인 (코드 원문, 결과 보기 전)

- `core.py:488-492` `check_assignment` 가 `lock.hold(T)`·`central_task=T` 를 실행기 응답 전에 한다. `node.py:425-443` 은
  'executor unavailable'·'executor timeout'·`success=false` 어느 때도 코어에 알리지 않는다 — 리뷰와 같다.
- `core.py:390-391` 은 비컨 `current_task_id`(잠정 — 이동 중 · 중앙 수락 직후)를 단조 집합 `taken` 에 넣고 빼는 곳이 없다.
  `core.py:575` `_start_task` 도 내 잠정 낙찰을 `taken` 에 넣는다 (같은 종류 — 내 실패 작업을 나중에 중앙이 다시 줘도 거부).
- 추가로 찾은 것: "확정 거부 때 `lock.release` + `central_task=''`" 만으로는 부족하다. 실행기는 작업 중이면 `busy:<id>` 로
  거부한다(`executor_context.cpp:350-352`). 지금 순서(먼저 hold)에서는 바쁜 로봇에 온 새 작업 B 의 `lock.hold(B)` 가 진행 중인
  작업 A 의 잠정 시도를 포기시키고(`vote_core.request` 는 진행 중 시도를 포기한다) `central_task` 를 B 로 덮는다 — 거부 뒤
  되돌릴 수 없다. 그래서 **실행기 수락을 확인한 뒤에 hold** 하는 순서로 바꾼다 (리뷰 고칠 방향과 결과는 같고 덮어쓰기가 없다).

## 고칠 것 (설계, 결과 보기 전)

- F1 순서: `check_assignment` 는 검사만(펜스 · 모드 · 확정된 남의 잠금). 노드가 실행기에 넘기고 결과를
  `on_central_result(T, outcome, token)` 으로 알린다. accepted → hold + central. rejected/unavailable → 상태 변경 없음 + 사건.
  timeout → 결과를 모르므로 이전과 같이 hold + central (단 다른 중앙 작업이 진행 중이면 덮지 않음).
  token = 프록시가 넘길 때의 그 작업 종료 상태 수 — 응답보다 먼저 FAILED/COMPLETED 가 오면 hold 하지 않는다.
- F2 `taken` 은 확정만 단조로: 비컨 held 의 `task:*`, `picked_up` 인 `current_task_id`, 내 픽업 시작, 재기동 때 남긴 확정.
  비컨의 잠정 `current_task_id` 는 신선한 비컨(수신 뒤 ≤ beacon_ttl_s)에서 매번 다시 계산하는 일시 집합 —
  CBAA 후보에서만 뺀다. 중앙 할당 검사(`check_assignment`)는 일시 집합으로 거부하지 않는다: 관리자는 거부를 받으면 곧바로
  다음 로봇에 준다(`fleet_manager_node.py:901-908`) — 옛 비컨이 남은 2 s 동안 다른 로봇들이 연달아 거부해 모두 backoff 에
  빠지게 된다. 둘이 같은 작업을 쫓아도 확정은 잠금이 하나로 막는다.
- F3 (리뷰 비차단) `accept_unfenced` 는 펜스가 생기기 전에만 epoch 0 을 받는다 — 펜스(≥ 1)가 생긴 뒤의 epoch 0 은 순서를
  매길 수 없으므로 거부.

## 가설과 예측 (결과를 보기 전에 적는다)

- H1 (순수, loopback 5 대): 리뷰 탐침 P1 · P1b 를 회귀 시험으로 옮긴다. 고친 코드에서 P1(실행기 거부 뒤 재배정) ·
  P1b(픽업 전 FAILED 뒤 재배정) 모두 두 번째 로봇이 수락하고 commit 뒤 sim 10 s 안에 HELD. 고치기 전 코드(`81444cd`)에서는
  두 시험이 실패한다 (리뷰 탐침과 같은 이유).
- H2 (launch ④b, 새 단계, NORMAL): 같은 task_id 를 두 로봇 프록시에 주고(둘 다 수락) 둘 다 COMMIT_TASK →
  task 잠금 허가 구간이 서로 다른 로봇에 겹치지 않고, 정확히 한 로봇이 HELD, 다른 로봇은 확정 거부(denied 또는 끝까지 HELD 아님) ≥ 1.
  같은 단계에서 실행기가 거부한 작업을 다른 로봇에 다시 주면 수락 → HELD (차단 1 의 ROS 수준 회귀).
- H3 (launch ④c, 새 파일, DEGRADED + 분할): 서버 없이 시작해 DEGRADED, 협상 뒤 {amr_01, amr_02} | {amr_03–05} 로
  sim [118, 160) s 분할. 소수 쪽은 정족수가 없으므로 분할 중 픽업 시작 0, 같은 task_id 픽업 시작 2 회 이상 0.
  공허 검사: 소수 쪽 commit 요청이 분할 중 ≥ 1 (확정 거부 ≥ 1), 다수 쪽 분할 중 픽업 ≥ 1 (활성 진단).
- H4 (변이, 사전 예측): RM1(가짜 실행기가 held 를 기다리지 않음)은 ④c 에서 **KILLED** (소수 쪽 분할 중 픽업). RM2(ENTER 가
  다른 요청의 잠정 표로도 확정)는 ④b 또는 ④c 에서 KILLED 를 예상하지만 확률적이다 — 살아남으면 그대로 적는다.
  순수 변이(새 가드): 거부 때 hold, token 무시, 시간 초과 때 hold 안 함, 일시 집합 신선도 무시, current_task_id 를 다시 단조로,
  picked_up 무시, CBAA 후보에서 일시 집합 제외 제거, 펜스 뒤 epoch 0 수용 — 모두 KILLED 예상.
- 가설을 버리는 결과: H1 에서 sim 10 s 안에 HELD 못 함, H2 · H3 에서 겹친 확정 · 중복 픽업 · 소수 쪽 분할 중 픽업 1 건 이상,
  공허 검사 미충족(경합이 안 생김 — 이때는 "판정 불가"로 적고 원인 진단), RM1 SURVIVED.

### 추가 예측 (구현 뒤 · 판정 실행 전, 2026-10-07 00:00 경 — 커밋 `36f190c`; 처음 적은 "00:2x" 는 시각 오기)

- 개발 실행 2 회(dev_main_s1 · dev_part_s1, `1a3fcbd` 와 같은 코드)는 둘 다 통과했다 — 판정 실행이 아니다 (실행 목록에는 개발로 등록).
  dev_part_s1 에서 본 것: **다수 쪽** amr_04 도 분할 중 commit_timeout 으로 실패했다. 분할 전에 소수 쪽 투표자의 잠정 표로
  VOTED 였던 시도는 ENTER 를 그 투표자에게만 보내므로 분할 중 확정을 못 하고, 시도 기한(task_hold_s 60 s)이 실행기
  commit_timeout(25 s)보다 길어 다시 모으지 못한다. 안전 문제는 아니고 활성 비용 — 판정 실행에서 횟수를 센다.
- 변이 추가 (예측 모두 KILLED): MR5 시간 초과가 진행 중 중앙 작업을 덮음, MR10 실행기 응답 전에 잠정 잠금(고치기 전 순서),
  MR11 내 잠정 낙찰을 단조 taken 에. 앞 회차 26 종도 고친 코드에 다시 (모두 KILLED 예측, 문자열이 바뀌면 BAD_MUTANT 로 적는다).
- ROS 수준 변이 추가: N2 노드가 실행기 거부도 accepted 로 코어에 알림 → ④b 에서 KILLED 예측 (amr_04 central_accept).

## 설정

- 이미지 `amr-ai/rl:0.3`, `ROS_DOMAIN_ID=41`, `ROS_LOCALHOST_ONLY=1`, 기본 브리지 네트워크, Gazebo 없음. 무거운 실행은 `cpu_slot.sh`.
- launch: 기존 `test_agent_launch.py`(①–④a + 새 ④b, ③ 의 중복 사례 'first_epoch'(=lower_epoch 와 같은 입력)을 'unfenced_epoch0' 으로
  바꿈) 시드 1 · 2 · 3, 새 `test_agent_launch_partition.py`(④c) 시드 1 · 2 · 3. 로봇↔로봇 손실 0.2 · 지연 U(0, 100) ms (수신 측).
- 판정 규칙 (사전): 위 "버리는 결과" 0 건이면 통과, 공허 검사 미충족이면 판정 불가. 0 건 지표는 rule of three 상한.
  ④a(기존 협상)는 경합 수(실패 · denied · commit 뒤 미허가)를 함께 적고 잠금 효과의 증거로 세지 않는다.
- 회귀: 순수 시험 전부(amr_fleet test_agent_* · test_dist_* · test_nego_*), loopback 스윕(`sweep_p3.py`, 같은 셀 · 시드 1–30)을
  고친 코드에서 다시 — 후보 집합 규칙이 바뀌었으므로 앞 수치를 그대로 쓰지 않는다. 최종 `colcon build -Werror` + `colcon test` amr_fleet.

## 결과

판정 코드 `36f190c` (= 코드 `1a3fcbd`, 뒤 커밋은 노트만), 작업 트리 깨끗 (스윕 · 최종 회귀 · 진단 탐침 때는 p3.md 문서 수정만).
원자료 `/home/CAPYI_AI/runs/b24/b4-p3/rfix/<run_id>/`. 공유 서버 loadavg(1 분) 19–43 / 32 코어 — 시간 값은 **공유 부하 하 예비값**.

**H1 (순수 회귀) [측정]** — `test_agent_core.py` 새 시험 6 개 통과 (P3R_final_regression 안, 그리고 개발 중 275 passed · 1 skipped).
리뷰 탐침을 경로만 바꿔 고친 코드에 돌림 (P3R_probe_review_after): P1 amr_03 재배정 `(True, 'ok')`, commit 뒤 **0.4 s** 에 HELD,
amr_02 의 작업 잠금 요청 0 회 (리뷰 전: `task_taken`, 150 s 동안 HELD 안 됨, amr_02 잠정 표 3 회). P1b 모든 로봇의 taken 에 T 없음,
amr_03 재배정 수락 (리뷰 전 4/5 로봇 taken 에 남음, `task_taken`). 단 P1b 탐침은 바뀐 API 때문에 수락 경로(on_central_result)를
거치지 않는다 — 수락 경로의 P1b 는 회귀 시험 `test_failed_before_pickup_lets_another_robot_take_the_task` 가 본다.
"고치기 전 코드에서 새 시험이 실패한다" 는 리뷰 탐침 출력(`review/probe/p1.out` · `p1b.out`)과 변이 MR1 · MR2 · MR10 으로 갈음했다
(고치기 전 코드에는 on_central_result 가 없어 새 시험을 그대로 돌릴 수 없다).

**launch ①–④b (test_agent_launch.py) [측정]** — 손실 0.2 · U(0, 100) ms, 시드별:

| 판정 | s1 | s2 | s3 |
|---|---|---|---|
| ① kill 3 s 뒤 /fleet_p2p/{vote,beacon,neg} 에이전트 발행·구독 | 5/5 셋 다 | 같음 | 같음 |
| ① 허가 구간 / 서로 다른 로봇 겹침 / 최소 간격(sim) / kill 뒤 허가 | 27 / 0 / 0.228 s / 16 | 30 / 0 / 0.180 s / 19 | 28 / 0 / 0.170 s / 17 |
| ② 네 모드 순서 (5 대) · kill→HOLD_NEW · kill→DEGRADED (벽시계 s) · RESYNC→NORMAL(sim s) · 관찰자 epoch | 예 · 1.57–1.58 · 5.28 · 3.0–3.1 · 4 | 예 · 1.24–1.34 · 5.04 · 3.0 · 4 | 예 · 1.54 · 5.24 · 3.0–3.1 · 4 |
| ③ 낮은 명령 수용 (lower_epoch · same_epoch_lower_leader · unfenced_epoch0) / 실행기 도달 | 0/3 · 0 | 0/3 · 0 | 0/3 · 0 |
| ③ 옛 권위자 하트비트 거부 / 로봇, 옛 권위자 살아 있어도 DEGRADED | 10 · 예 | 10–11 · 예 | 12 · 예 |
| ④b T1: amr_04 실행기 거부 → amr_02 재배정 수락 → HELD (벽시계) | 예 0.77 s | 예 0.79 s | 예 0.30 s |
| ④b T2: 두 로봇 수락 · 둘 다 COMMIT_TASK → HELD / 겹침 / 확정 거부(denied) / 정착 | amr_03 / 0 / amr_05(denied) / 5.79 s | amr_05 / 0 / amr_03(denied) / 8.59 s | amr_03 / 0 / amr_05(denied) / 4.93 s |
| ④a 협상 픽업 / 중복 / 완료 / 실패, 경합(commit 뒤 미허가) | 10 / 0 / 10 / 0, 0 | 10 / 0 / 10 / 0, 0 | 10 / 0 / 10 / 0, 0 |

- ④a 는 이번에도 **경합 0** (task_granted 10 개가 모두 다른 작업) — 잠금 효과의 증거로 세지 않는다.
- ④b 경합 3/3 시드에서 HELD 한 대 · 다른 한 대 denied · 허가 구간 겹침 0 → 확정 거부 3 건 (rule of three 로 말할 수준은 아니다 — n 3).

**launch ④c (test_agent_launch_partition.py, 분할 sim [118, 160) s) [측정]**:

| 판정 | s1 | s2 | s3 |
|---|---|---|---|
| 소수 쪽 분할 중 픽업 시작 | **0** | **0** | **0** |
| 같은 task_id 픽업 2 회 이상 | 0 | 0 | 0 |
| 공허 검사: 소수 쪽 분할 중 commit (= 확정 거부) | 2 | 2 | 2 |
| 공허 검사: 다수 쪽 분할 중 픽업 | 1 | 1 | 1 |
| commit_timeout 실패 (소수 · **다수**) | amr_02 · amr_03, amr_04 | 같음 | 같음 |
| 픽업 / 완료 (작업 6, sim 175 까지) | 4 / 4 | 4 / 4 | 4 / 4 |

- 세 시드가 거의 같다 — 같은 기하 · 같은 분할이고 시드는 링크 손실 · 지연만 바꾼다 (사실상 n 은 3 보다 작다).
- **다수 쪽 amr_03 · amr_04 가 3/3 시드에서 분할 중 확정을 못 했다** (예측: 개발 실행에서 본 것). 진단 (P3R_probe_enter_partition,
  loopback 같은 기하 · 분할, 시드 1–5): 다수 쪽 로봇의 commit_timeout 은 분할 직전 VOTED 표 집합에 **다수 쪽 투표자가 3 미만**일
  때만 났다 (시드 2 amr_04 {01,04,05} · amr_03 {02,03,04}, 시드 3 amr_03 {01,03,05} …; 다수 쪽 commit_timeout 수 시드 1–5 = 0, 2, 1, 1, 2).
  ENTER 는 표를 준 투표자에게만 가고, VOTED 뒤에는 표를 더 모으지 않으며, 시도 기한(task_hold_s 60 s)이 실행기
  commit_timeout(25 s)보다 길어 시도를 버리고 다시 모으지 못한다. ROS 실행의 표 집합은 상태에 실리지 않아 직접 보지 못했다 —
  loopback 과 같은 메커니즘이라고 **추정**한다.

**변이 [측정]**: 순수 37/37 KILLED (앞 회차 26 + 새 MR1–MR11 — 어느 시험이 잡았는지 p3.md §6.4). ROS 수준 (시드 1, 사본을
`src/amr_fleet` 자리에 겹쳐 마운트):

| 변이 | 시험 | 예측 | 결과 |
|---|---|---|---|
| RM1 가짜 실행기가 held 를 기다리지 않음 | ④c | KILLED | **KILLED** — 소수 쪽 분할 중 픽업 2 (amr_01 136.4 s, amr_02 130.5 s) |
| N2 노드가 실행기 거부를 accepted 로 코어에 알림 | ①–④b | KILLED | **KILLED** — amr_04 central_accept 1, 그리고 amr_02 가 15 s 안에 T1 HELD 못 함 (리뷰 P1 과 같은 꼴이 ROS 에서 재현) |
| RM2 ENTER 가 다른 요청의 잠정 표로도 확정 | ④c | 확률적 | **SURVIVED** |
| RM2 | ①–④b | 확률적 | **SURVIVED** |

- 첫 시도(P3R_mut_ros_try1)는 4 회 모두 **판정 불가**: 사본을 `/mut` 에 마운트해 `test_nego_cost.py` 의 `parents[3]` 가 IndexError
  → pytest 수집 오류 rc 4. 마운트 경로를 고쳐 다시 돌렸다.

**스윕 (P3R_sweep_tasklock_v2) [측정]**: 잠금 팔 5 셀 × 30 시드 = 픽업 1,200 / 중복 0 / 완료 1,200 (앞 회차와 같음, 마지막 완료
중앙값 차이 ≤ 0.3 s). **낙관 대조군**: 손실 0 · 0.2 중복 0/30, **손실 0.4 2/30 (시드 14 · 16), 버스트 10/30** (앞 회차 0/30 · 0/30),
분할 29/30 (같음).

**최종 회귀 (P3R_final_regression) [측정]**: 새 build/install 에서 `colcon build -Werror --packages-up-to amr_msgs amr_fleet` rc 0,
경고 0, stderr 0. `colcon test` amr_fleet xunit 788 / 실패 0 / 오류 0 / 건너뜀 1 (기존 f3 연구 스크립트), launch 두 시험
94.7 s · 75.3 s 통과 (각 네 번째 실행), amr_msgs 2 / 0. `colcon test-result` 는 읽기 전용 마운트로 실패 → xunit 직접 집계.
amr_behavior 는 리뷰 반영에서 바뀌지 않아 이번 회귀에서 뺐다 (`cb65301` 회귀 370 / 0).

## 해석

- H1 유지: 실행기 거부 · 픽업 전 실패 뒤 다른 로봇 재배정이 수락 → HELD 된다 (순수 회귀 + ROS ④b T1 3/3).
  N2 변이가 ROS 수준에서 리뷰 P1 과 같은 막힘(15 s 안에 HELD 못 함)을 다시 만들었다 — ④b 가 이 결함 종류를 잡는다.
- H2 유지 (n 3): 중앙 경로 경합에서 확정은 한 대, 진 쪽은 denied. 이 시험은 RM2 를 잡지 못했다 — 두 요청이 동시에 VOTED 가
  되지 않는 한(잠정 표 배타가 먼저 막는다) 확정 표 배타 변이는 드러나지 않는다. RM2 는 순수 시험(test_agent_hold_enter 등)이 잡는다.
- H3 유지: 분할 중 소수 쪽 픽업 0 (3/3), RM1 KILLED — **ROS 수준에서 작업 잠금이 픽업을 막는 증거는 이제 ④c 다.**
  단 표본이 작고(시드 셋이 거의 같은 실행) 경합의 모양이 하나(2 | 3 분할)뿐이다.
- 새 사실 둘: (1) 분할 전 VOTED 표가 건너편에 쏠린 다수 쪽 시도는 분할 중 확정을 못 한다 — 활성 비용, 안전 무관. (2) 앞 회차
  스윕의 "손실 · 버스트에서 대조군 중복 0" 은 단조 gossip 배제 덕이 컸다 — 그 배제가 리뷰 결함의 원인이었고, 없애자 대조군이
  고손실 · 버스트에서 중복을 낸다. 잠금 팔은 그대로 0.
- 말하지 않는 것: 실제 실행기 · BT(CommitTaskLock) 경로, 시간 초과 경로의 남은 누수(설계상 둠), Gazebo 의 실제 이동 · 동시 점유.

## 다음

- p3.md §8(P5 인계) 11–15: 시간 초과 경로 잔여 위험, VOTED 뒤 표 집합 고정(ENTERING 하위 기한 또는 VOTED 중 표 계속 모으기 —
  투표 코어 프로토콜 변경이라 f2 모델과 함께), ZoneAgentLink 미래 stamp, BT 벽시계 시간 초과 대 sim, ISOLATED launch 미시험.
- P4(b4-p4)에 넘길 것: 낙관 대조군이 버스트 · 고손실에서도 중복을 낸다 — 잠금 효과를 보이는 셀이 분할만이 아니다.
