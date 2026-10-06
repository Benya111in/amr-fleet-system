# P3 리뷰 반영 — 중앙 프록시 작업 상태 누수와 launch ④ 경합 단계

- 시각: 2026-10-06 23:48 KST · 에이전트: b4-p3 · 단계: P3 (리뷰 반영)
- 코드: `81444cd` (리뷰 대상, 브랜치 `feature/b4-p3`) — 사전 등록 시점. 결과 절에 실제 측정 커밋을 적는다
- 관련 실행 id: (결과 절에 채운다)
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

### 추가 예측 (구현 뒤 · 판정 실행 전, 2026-10-07 00:2x — 커밋 `1a3fcbd` 다음 커밋)

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

(실행 뒤 채운다)

## 해석

(실행 뒤 채운다)

## 다음

(실행 뒤 채운다)
