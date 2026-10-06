# traffic/hold fail-open 수정 — 설계 결정과 근거 (보너스 4 P0 첫 항목)

작성 2026-10-06, 브랜치 `feature/traffic-hold-failsafe` (기준 `origin/develop` `d8ae80c`).
근거 표기: **[코드]** 파일:행을 이 브랜치의 기준 커밋에서 직접 읽음 · **[시험]** 저장소 시험으로 재현 ·
**[측정]** Gazebo 로그 · **[보고서]** 「분산 플릿 관리 협상 할당과 폴백」(검증하지 않은 주장은 그렇게 적는다).

## 0. 결정 (요약)

| # | 무엇을 | 어떻게 | 왜 |
|---|---|---|---|
| D1 | 실행기 `traffic/hold`·`traffic/yield_pose` 구독 QoS | volatile `QoS(10)` → **reliable + transient_local, depth 1** (발행자와 같다) | 늦게 뜬(재기동한) 실행기가 latched 현재 값을 받게 |
| D2 | 실행기의 "첫 값을 받기 전" 의미 | 교통 관리 중이면 **허가 없음 = 대기**, 아니면 기존대로 통과. `traffic_hold.policy` = `auto`(기본) / `required`, `behavior.launch.py traffic_hold_policy:=` 로도 준다 | 보고서의 "허가 없음 = 진입 금지" 반전을, traffic_manager 가 없는 구성을 영영 세우지 않는 범위에서만 |
| D3 | `auto` 의 "교통 관리 중" 판정 | `traffic/hold` 발행자가 그래프에 보이거나, 노드 시작 뒤 **발견 유예 2.0 s(벽시계)** 안 | 재기동 직후 DDS 발견 전에는 발행자가 안 보여 "통과" 로 판정되는 창을 닫는다 |
| D4 | 첫 값을 받은 뒤 | 받은 값만 따른다. 발행자가 사라져도 마지막 값 유지 (**기존과 같다**) | 관리자 사망 처리는 분산 폴백(보너스 4 P1~P3) 범위 |
| D5 | traffic_manager 기동 시 초기 `hold=false` | **내지 않는다.** 처음 관측한 주기에 평가한 값을 처음 낸다 | 평가 전 false 는 "허가" 다 — 재기동한 관리자가 이전 인스턴스가 세워 둔 로봇을 첫 주기까지 풀어 줬다 |
| D6 | 한 번도 관측하지 못한 로봇 | 그 로봇의 hold 구독자(실행기)가 붙은 주기부터 `obs_timeout_s`(3 s) 뒤 `hold=false` + 경고 | odometry/filtered_map 이 없는 구성(slam·odom 측위)에서 D2·D5 가 로봇을 영영 세우지 않게 |
| D7 | "포켓 먼저" 규칙이 hold=true 를 한 주기 미룰 때 | 이전 값을 유지 (아직 낸 적이 없으면 아무것도 내지 않는다) | 미루는 동안 false 를 내던 경로를 막는다 (지금은 도달 불가 — §3.3) |
| D8 | 재연결 시 hold 재발행 | ~~하지 않는다~~ → **§7 D9 로 대체** (평가된 값 1 Hz 재발행) | (당시) 바뀔 때만 내고 depth 1 이므로 writer 이력 = 현재 값. D1 만으로 재연결한 구독자가 현재 값을 받는다 — 이 전제가 §6 실측에서 깨졌다 |

## 1. 결함 — 코드로 다시 확인한 것

| 쪽 | 위치 (`d8ae80c`) | 내용 |
|---|---|---|
| 발행 | `src/amr_fleet/amr_fleet/traffic_manager_node.py:63-64` | `LATCHED_QOS` = depth 1, RELIABLE, TRANSIENT_LOCAL **[코드]** |
| 발행 | 같은 파일 `:252` | `/<id>/traffic/hold` 를 `LATCHED_QOS` 로 만든다 **[코드]** |
| 발행 | 같은 파일 `:24`, `:365-366` | 주기마다 평가하지만 **바뀐 명령만** 낸다 (`if hold != r.hold`) **[코드]** |
| 발행 | 같은 파일 `:266` | 로봇 등록 시 `hold=false` 를 한 번 낸다. 주석 "늦게 뜬 실행기도 확실한 초기값을 받는다" **[코드]** |
| 구독 | `src/amr_behavior/src/task_executor_node.cpp:327-331` | `rclcpp::QoS(10)` (volatile). 주석 "QoS 미정 → volatile 구독 (latched/volatile 발행자 모두와 호환)" **[코드]** |
| 구독 | 같은 파일 `:332-334` | `traffic/yield_pose` 도 `QoS(10)` — 발행은 `LATCHED_QOS`(`traffic_manager_node.py:253-254`) **[코드]** |
| 기본값 | `src/amr_behavior/include/amr_behavior/executor_context.hpp:231` | `bool traffic_hold_{false};` — 타임아웃·워치독 없음 **[코드]** |
| 판정 | `src/amr_behavior/include/amr_behavior/bt_nodes/is_traffic_hold.hpp:30` | `!trafficHold()` 면 FAILURE → `TrafficGate`(`behavior_trees/subtrees/yield.xml:15-20`)가 통과 **[코드]** |

**DDS 동작.** volatile reader 는 transient_local writer 와 호환(짝이 맺어짐)되지만 지난 표본을 받지 않는다.
보고서 노트(`gap_distributed_zone_mutex_safety_verification.md`)는 이것을 "DDS durability 규칙에서 끌어낸 추론,
공식 문장으로 확인하지 않음" 이라고 적었다. 이번에 **[시험]** 으로 확인했다:
`test_task_executor_node.cpp` `LateExecutorReceivesLatchedHoldAndPocket` 에서 latched 발행자가 `hold=true` 를 낸 **뒤**
만든 volatile 구독은 짝이 맺어진(`get_publisher_count()==1`) 뒤에도 0 건을 받고, 같은 때 만든 실행기(수정 후 QoS)는
`hold=true` 와 포켓을 받는다 (amr-ai/rl:0.3 이미지의 Humble 기본 RMW, 한 프로세스 안).

**결과.** `hold=true` 인 동안 실행기가 재기동되거나 늦게 매칭되면, 관리자가 값을 바꿔 다시 낼 때까지 실행기는
기본값 false 로 `TrafficGate` 를 통과한다 (fail-open). 관리자 `:266` 의 초기값 의도도 volatile 구독 때문에 무효였다.
**측정 로그에서 관측된 적은 없다** — 정상 실행은 노드가 함께 뜨고 관리자가 먼저 뜨므로(아래 §2) 이 경로를 밟지 않는다
(handover1006 §8 "코드 판독, 측정 아님" 과 같은 판단). 이 수정은 잠재 결함 제거이고, 통과율을 올리는 수정이 아니다.

## 2. 어떤 구성이 traffic_manager 를 띄우나 — 코드 확인

| 구성 | traffic_manager | 실행기 | 근거 |
|---|---|---|---|
| `amr_fleet fleet_manager.launch.py` | `traffic_manager:=true` 가 **기본** | 없음 | `fleet_manager.launch.py:76-85, 123-124` |
| `amr_bringup system.launch.py` (단일 로봇) | `with_fleet` 기본 true → 위 런치 포함 | `with_behavior` 기본 true | `launch_utils.py:50, 122-123, 312-320` |
| `amr_bringup multi_robot.launch.py` | 같음 | 같음 | 같은 `launch_utils.global_actions` |
| docker-compose `fleet` 서비스 | 있음 (별도 컨테이너) | 다른 서비스 | `docker-compose.yml` `fleet: … fleet_manager.launch.py` |
| 통합 하네스 `system` 프로필 (시나리오 10·11·13·14, `use_behavior=True`) | **없음** — `with_fleet/dashboard/evaluation` 을 false 로 덮는다 | 있음 | `tests/integration/amr_itest/stack.py:421` |
| 통합 하네스 `multi_robot` 프로필 (시나리오 12) | 있음 (`with_fleet: 'true'`) | 5대 | `stack.py:453` |
| `amr_behavior behavior.launch.py` 단독 | 없음 | 있음 | 런치 내용 |

추가로, 관리자가 떠 있어도 **로봇을 관측하는 조건**은 `/<id>/odometry/filtered_map` 이다
(`traffic_manager_node.py:248`, `observations()` 는 `r.odom is None` 인 로봇을 뺀다 `:327-329`). 이 토픽은 측위 스택의
`localization` 모드에서만 나온다 (`amr_localization/launch/localization.launch.py:176-178` 의 `ekf_filter_node_map` 리맵).
`slam`·`odom` 모드에서는 관리자가 떠 있어도 로봇을 영영 관측하지 못하고, 그런 로봇에는 `update()` 명령도 없다.

→ 보고서 원안처럼 **"첫 값 전 = 항상 대기"** 로 뒤집으면 시나리오 10·11·13·14 와 단독 실행기는 영영 움직이지 않는다.
관리자 쪽 초기값만 없애면(D5) slam·odom 측위의 다중 로봇 구성이 영영 서 있다. 그래서 D2·D3·D6 을 함께 둔다.

## 3. 설계

### 3.1 실행기 QoS (D1)

`traffic/hold`·`traffic/yield_pose` 를 `latchedQos()`(같은 파일의 `estop`·`localization/lost` 구독과 같은 함수, depth 1
reliable transient_local)로 구독한다. 발행자 `LATCHED_QOS` 와 정확히 같다. 부작용: **volatile 발행자와는 짝이 맺어지지
않는다** (요청 transient_local > 제공 volatile). 저장소에 그런 발행자는 없다 (`grep traffic/hold` — 발행은
`traffic_manager_node.py` 뿐). 누가 volatile 로 내면 그래프에는 발행자가 보이고 값은 오지 않으므로 `auto` 가 대기를
유지하고 경고한다 (§3.2, fail-closed). rclcpp 기본 incompatible-QoS 경고도 함께 찍힌다.

### 3.2 실행기 상태 (D2·D3·D4)

`ExecutorContext` (ROS 비의존)에 상태를 둔다. 노드는 tick 마다(그리고 생성자에서) 첫 값을 받기 전이면
`trafficManaged(policy, 시작 뒤 경과, 유예, 그래프의 traffic/hold 발행자 수)` 로 `setTrafficManaged()` 를 갱신한다.

| 상태 | 조건 | `trafficHold()` | 게이트 |
|---|---|---|---|
| `unmanaged` | 받은 값 없음 ∧ 교통 관리 아님 | false | 통과 (기존 동작) |
| `awaiting` | 받은 값 없음 ∧ 교통 관리 중 | **true** | 대기 (포켓 없음 → 제자리) |
| `hold` | 받은 값 true | true | 대기 / 포켓 |
| `clear` | 받은 값 false | false | 통과 |

- `auto`: 관리 중 ⇔ 발행자 수 > 0 ∨ 시작 뒤 경과 < `discovery_grace_s`. `required`: 항상. 모르는 문자열은 경고 후 `auto`.
- 유예는 **벽시계**(steady)다. `use_sim_time` 노드의 `now()` 는 `/clock` 을 받기 전 0 이라 유예 기준으로 못 쓴다.
- 유예 비용: traffic_manager 가 없는 구성에서 실행기 시작 뒤 2 s 동안 움직이지 않는다. 하네스 시나리오 13 은
  Nav2 lifecycle 3 개가 active 이고 `executor/phase` IDLE, GT 1 s 정지를 본 뒤에 명령한다
  (`test_13_response_time.py` 머리말) → 그 시점은 실행기 시작 뒤 2 s 보다 늦을 것으로 **본다(미측정)**.
  노드 시험 `AssignRunsFullTaskAndRejectsWhileBusy` 는 시작 직후 할당하므로 유예만큼 기다린 뒤 끝난다 (수정 후 2.8 s).
- 첫 값을 받은 뒤에는 `managed` 를 보지 않는다 (D4). 따라서 관리자가 죽어도 마지막 값이 유지된다. hold=true 였으면
  계속 서 있고(fail-closed), false 였으면 관리 없이 움직인다 — **수정 전과 같은 성질**이고 §4 위험 1 로 남긴다.
- 관측성: 첫 값 전 상태 전이(`awaiting`·`unmanaged`·첫 수신)를 INFO 로 남기고, `awaiting` 이 `유예 + 5 s` 를 넘으면
  10 s 간격으로 "발행자 N (QoS 맞는 것 M)" 과 함께 경고한다.

### 3.3 traffic_manager (D5·D6·D7·D8)

- **D5 초기값 제거.** 수정 전 `_add_robot` 이 `hold=false` 를 latched 로 냈다. D1 이후에는 이 값이 실제로 전달되므로,
  관리자가 재기동하면 이전 인스턴스가 세워 둔 로봇이 새 인스턴스의 첫 평가(2 Hz → 최대 0.5 s + 관측 지연)까지 풀린다.
  제거하면 그동안 실행기는 이전 인스턴스의 마지막 값을 유지한다 (D4). 새로 뜬 실행기는 `awaiting` 으로 기다린다.
  포켓 없음(빈 `yield_pose`)은 기동 시 계속 낸다 — 해가 없고 "포켓 없음" 의 latched 기준선이다.
- **D6 관측 못 한 로봇 해제.** hold 를 한 번도 내지 않았고 이번 명령에도 없는 로봇은, 그 로봇의 `traffic/hold` 에
  구독자(= 기다리는 실행기)가 처음 보인 주기부터 `obs_timeout_s` 가 지나면 `hold=false`(사유 `unobserved`)를 내고
  경고한다. 그런 로봇은 예측·토큰에 들어오지 않아 이 관리자가 관리할 수 없다 — 세워 두는 것은 안전이 아니라 고장이다.
  **한 번이라도 관측한 로봇은 여기에 오지 않는다** (관측이 끊기면 기존 stale·lost 규칙, fail-closed). 시간 기반
  해제라는 점은 §4 위험 3. 새 파라미터를 만들지 않고 `obs_timeout_s` 를 쓴다 ("이만큼 관측이 없으면 stale" 과 같은 척도).
  시계 기준을 관리자 첫 주기로 했던 첫 구현(2620160)은 고쳤다: 관리자는 런치 시작 즉시 뜨고 로봇 스택은 스폰 뒤
  로봇마다 `stack_stagger_s`(6 s) 간격으로 뜨므로(`launch_utils.py` `stack_actions`), 늦게 뜬 로봇이 관측되기도 전에
  "관측 없음" 으로 풀려 D5 가 무력해지고 오해를 부르는 경고가 남는다. 구독자가 없으면 기다리는 실행기도 없으므로
  아무것도 내지 않는다.
- **D7 포켓 먼저.** `_apply` 는 새 포켓과 hold=true 가 같은 주기에 오면 포켓만 내고 hold 는 다음 주기로 미룬다
  (`traffic_manager_node.py:363-364`). 수정 전에는 미루는 동안 `hold = False` 를 넣었고, `r.hold` 가 False 면 발행이
  없었다. hold 를 낸 적이 없는(`None`) 로봇이면 false 를 **냈다**. 지금은 이전 값(`r.hold`)을 유지하고 `None` 이면 내지
  않는다. 이 경로는 현재 도달하지 않는다 — 사건(incident)은 `confirm_s` 동안 관측된 로봇에만 열리고, 그 전 주기에 이미
  첫 값이 나간다. 그래서 동작 변화는 없고 시험으로만 고정한다.
- **D8 재발행 불필요.** (→ §7 D9 로 대체: 아래 전제가 §6 의 B·A/B 실측에서 깨졌다.) `_publish_hold` 만 hold 를 내고 바뀔 때만 부르며(`_apply`, D6), 매번 `r.hold` 를 갱신한다
  → writer 의 depth 1 이력은 항상 `r.hold` 와 같다. 재연결한 transient_local 구독자는 그 값을 받는다
  (기존 시험 `test_deadlock_detected_counted_and_resolved` 의 `_late_value(... hold).data is True` 가 이미 확인).
  주기 재발행(하트비트)은 관리자 사망 감지용으로는 의미가 있지만 그것은 P1 의 하트비트 설계로 넘긴다.

### 3.4 보고서와 다르게 한 것

- 보고서 P0 은 "실행기 hold 구독을 TRANSIENT_LOCAL 로 바꾸고 '허가 없음 = 진입 금지' 의미로 반전" 이라고 쓴다
  (보고서 218–276 행 표). **반전은 첫 값을 받기 전, 교통 관리 중일 때만** 적용했다. 무조건 반전하면 §2 의 단일 로봇
  구성이 영영 선다. 보고서도 traffic_manager 가 없는 구성을 다루지 않았다.
- 보고서 99–104 행의 구조(구역 사슬·결정점·`AcquireZoneChain`·`ZoneGuard`·로봇 쪽 구역 계산)는 **하지 않았다.**
  그것은 hold(거부 목록)를 grant(허가 목록)로 바꾸는 P1~P3 이고, 이 수정은 현재의 거부 목록 의미 안에서 유실만 막는다.
  즉 "받은 값 false = 통과" 는 그대로다.

## 4. 남은 위험 (불리한 것부터)

0. **발견·전송이 늦은 로봇이 선다 — 이 수정이 바꾼 것. 시나리오 12 회귀 판정은 미확정.** §6 의 시나리오 12 수정 후
   실행에서 늦게 뜬 실행기(amr_04·amr_05)가 관리자 발행자를 본 뒤에도(발행자 1, QoS 맞는 것 1) 값을 받지 못한 채
   `awaiting` 으로 수 분~19 분 머물렀다. 관리자가 그 실행기의 구독을 발견하지 못했거나, 발견하고 값을 냈는데도 도착하지
   않았다(B·A/B 의 amr_04) — 같은 컨테이너에서 새로 띄운 탐침은 latched 값을 즉시 받았다(`logs/f0_probe/probe_b1.txt`).
   관리자의 오도메트리 구독(이 수정이 건드리지 않은 경로)도 같은 로봇을 못 봤으므로 발견·전송 실패 자체는 이 수정과
   무관하다. 그러나 결과는 달라진다: 수정 전에는 그 로봇이 아무것도 받지 못한 채 **조용히 통과**했고, 수정 뒤에는 **경고를
   내며 선다.** 관리자가 보지 못하는 로봇은 관리자가 지켜 줄 수 없으므로 안전 쪽으로는 맞는 방향이지만, 발견이 느린
   환경에서는 작업이 시작되지 않아 시나리오 판정을 떨어뜨릴 수 있다. 수정 후 시나리오 12 다섯 번은 모두 판정 전에 멈췄고
   (준비 단계 정체 2, 하네스 정지 3), 수정 전 대조는 두 번 중 한 번 통과·한 번 같은 하네스 정지였다 — 이 수정이 통과율에
   주는 영향은 **미확정**이다. 도메인 210 의 포트가 임시 포트 범위와 겹치는 점은 도메인 61 에서도 같은 증상이 나와
   필요조건이 아니다.
1. **관리자 사망 뒤 마지막 값.** 받은 값이 false 였던 로봇은 관리자가 죽은 뒤 관리 없이 움직인다. 보고서가 말하는
   "인계 공백" 과 같은 종류이고 이 수정으로 바뀌지 않았다. 보고서의 비대칭 분할(상향만 끊김 → lost 반납 → 다른 로봇에
   부여, 이 로봇은 hold 갱신을 못 받음) 틈도 그대로다 — 둘 다 분산 폴백(P1~P3) 범위.
2. **유예보다 늦은 발견.** `auto` 에서 실행기 시작 뒤 2 s 안에 관리자 발행자가 그래프에 안 보이면 그 뒤 발행자가
   보일 때까지 `unmanaged`(통과)다. 보이면 `awaiting` 으로 바뀌고 BT 게이트가 reactive 라 그 순간 멈춘다. 재기동한
   실행기는 작업이 없어 그 창에서 움직일 이유가 대개 없지만(작업은 fleet_manager → assign_task 경로), 배터리 부족
   자동 충전처럼 작업 없이 움직이는 경로는 있다. 교통 관리자가 있다고 아는 런치는 `required` 를 줘야 이 창이 닫힌다 —
   `behavior.launch.py` 에 `traffic_hold_policy` 인자를 두었고, `amr_bringup` 이 `with_fleet` 일 때 넘기는 배선은 그 소유자
   몫이다. 측정(§6)에서 늦게 뜬 실행기의 발행자 발견은 2.3 s · 2.7 s · 3.5 s · 14 s · 77 s · 83 s 였다 — 어떤 유예 값도
   이 창을 닫지 못하고, `required` 는 창을 닫는 대신 그 로봇을 발견될 때까지 세운다(위험 0). 그래서 기본값으로 켜지
   않았다.
3. **관측 못 한 로봇의 시간 기반 해제(D6).** 시간이 지나면 풀어 주는 fail-open-after-timeout 형태다. 대상을
   "한 번도 관측하지 못한" 로봇으로 좁혔고, 그런 로봇은 관리자의 예측·토큰 밖이므로 해제해도 관리자가 줄 수 있었던
   보호를 잃는 것은 없다. 실행기가 뜬 뒤 측위(EKF)가 `obs_timeout_s`(sim 3 s)보다 늦게 오도메트리를 내면 그 로봇은
   잠깐 false 를 받았다가 관측되는 주기에 평가 값으로 바뀐다 — 수정 전(기동 즉시 false)보다 나빠지지는 않는다.
4. **volatile 발행자.** 누군가 volatile 로 `traffic/hold` 를 내면 `auto` 는 그 로봇을 영영 세운다(경고와 함께).
   의도한 fail-closed 이지만 운영자가 알아야 한다.
5. **유예 2 s 가 단일 로봇 측정 판정에 닿는지.** 수정 전 단일 로봇 실행 34 개(시나리오 11 21 · 13 9 · 14 4, 측정용 worktree 로그)에서
   실행기 시작 → 첫 `assign_task` 는 최소 10.0 s · 중앙 11.0 s 였다 **[계산]** — 유예 2 s 는 첫 작업 전에 끝난다.
   Gazebo 로는 시나리오 10(통과, 실행기에 작업을 주지 않는다)과 11(작업 시작 +19.8 s, 유예 2 s 는 그 전에 끝남 — 판정은
   알려진 `lost` 실패, §6)을 돌렸다. 13(응답 시간)은 돌리지 않았다.

## 5. 검증

- 단위(`test_executor_context.cpp`): 상태 4 개 전이, 받은 뒤 `managed` 무시, latched 포켓과 hold 동시 도착,
  정책 파싱·`trafficManaged` 규칙(유예 경계 1.99/2.0 s, 발행자, required).
- BT(`test_task_tree.cpp` `AwaitingFirstTrafficValueBlocksMotion`): `awaiting` 동안 작업 목표로 주행 goal 을 보내지
  않고, 첫 값 false 뒤 작업 완료.
- 노드(`test_task_executor_node.cpp`): 늦게 뜬 실행기가 latched hold=true·포켓을 받음(+ volatile 대조 0 건),
  발행자만 있고 값이 없으면 `awaiting`, 발행자가 없으면 유예 뒤 `unmanaged`(모르는 정책 → auto), `required` 는
  발행자 없이도 `awaiting`.
- 관리자(`test_traffic_node.py`): 관측 전에는 hold 를 아무도 받지 않음(포켓 없음은 latched), 관측한 로봇은 첫 평가
  값, 관측 못 한 로봇은 구독자가 붙은 주기 + `obs_timeout_s` 뒤 false, 구독자가 없는 로봇에는 내지 않음, 포켓 먼저
  규칙이 낸 적 없는 로봇에 false 를 내지 않음.
- 전체 단위 시험: `colcon test --packages-select amr_behavior amr_fleet` 774 tests, 0 failures, 60 skipped(cppcheck)
  — 수정 전 764. `colcon build` 전 패키지 Release `-Wall -Wextra -Wpedantic -Werror` 경고 0.
- 변이 검사 15 종, 전부 시험이 잡았다 (가드를 하나씩 되돌리고 위 시험 실행):

  | # | 되돌린 가드 | 잡은 시험 |
  |---|---|---|
  | M1 | 실행기 hold 구독 QoS → volatile | `LateExecutorReceivesLatchedHoldAndPocket` |
  | M2 | yield_pose 구독 QoS → volatile | 같음 |
  | M3 | `trafficHold()` 가 교통 관리 여부 무시 | 단위 · BT · 노드 4 건 |
  | M4 | 수신해도 received 안 세움 | 단위 · BT · 노드 다수 |
  | M5 | 발견 유예 무시 | `ParseAndManagedRule`, `WithoutTrafficManagerProceedsAfterDiscoveryGrace` |
  | M6 | 발행자 수 무시 | `ParseAndManagedRule`, `AwaitsFirstValueWhileTrafficPublisherExists` |
  | M7 | required 무시 | `ParseAndManagedRule`, `RequiredPolicyWaitsWithoutPublisher` |
  | M8 | tick 마다 갱신 안 함 | `AssignRunsFullTask…`, `WithoutTrafficManager…` |
  | M9 | 받은 뒤에도 교통 관리 여부를 봄 | 단위 · BT · 노드 |
  | M10 | 관리자 기동 시 hold=false 복원 | `test_hold_published_only_after_evaluation` 외 1 |
  | M11 | 관측 못 한 로봇 해제 제거 | `test_hold_published_only_after_evaluation` |
  | M12 | 해제를 obs_timeout_s 없이 즉시 | 같음 |
  | M13 | 포켓 먼저 미룸에서 false 를 냄 | `test_first_hold_behind_new_pocket_is_deferred_not_false` |
  | M14 | 해제를 이미 낸 로봇에도 반복 | `test_hold_published_only_after_evaluation` (처음엔 놓쳤다 → "다시 내지 않음" 단언 추가 뒤 잡음) |
  | M15 | 구독자 없이도 해제 시계 시작 | `test_hold_published_only_after_evaluation` |

  b4ca041(발행자 수 조회를 200 ms 마다로)에서 실행기 쪽 M1·M5·M6·M8 을 다시 돌려 모두 잡혔다.

## 6. Gazebo 회귀 — 측정 기록 (공유 부하 하 예비값)

모든 실행: `amr-ai/rl:0.3`, `run_integration.sh --no-unit --timeout-scale 3`, IGN 파티션 `amr_ai_f0*`, 로그는 worktree 의
`logs/<태그>/` (HEAD.txt·uncommitted.txt·start.txt·end.txt·ABORTED.txt 동봉, gitignore). loadavg 는 32 코어 호스트의 1 분 값.
수정 전 대조(`d8ae80c`)는 `git archive` 사본을 같은 이미지·같은 Release `-Werror` 로 빌드해 돌렸다.

**판정: 시나리오 12 회귀는 미확정이다.** 수정 후 다섯 번은 모두 판정 전에 멈췄고, 수정 전은 두 번 중 한 번 통과했다.
그러나 멈춘 다섯 번 중 셋은 하네스(launch_test) 프로세스 자체가 정지한 것이고, **같은 시각에 나란히 돌린 수정 전 대조도
같은 방식으로 정지했다**(`f0_ab_base`). 나머지 둘(A·B)은 준비 단계(lifecycle·서비스 발견)에서 멈췄고 그 단계는 이 수정이
건드리는 경로(traffic/hold)를 쓰지 않는다. 이 수정이 그 정체를 부르는 기제는 찾지 못했지만, 표본이 작아 무관하다고 단정하지
않는다.

| 태그 | 시나리오 | 커밋 | DDS | loadavg 시작→끝 | 결과 |
|---|---|---|---|---|---|
| `f0_s01_a` | 01 센서 토픽 | 2620160 | 210 | 17.6 → 16.9 | **통과** 10/10 (15.8 s) — 이 수정과 무관한 스모크 |
| `f0_s12_a` | 12 | 2620160 | 210 | 17.6 → 65.9 | **판정 없음** — 준비 단계 정체(amr_04·05 ROBOT_LOST, E-stop sensor_late), 16 분에 수동 중단 |
| `f0_s12_b` | 12 | 19a1b82 | 210 | 27.8 → 48.7 (중간 90) | **판정 없음** — 17 분 동안 작업 할당 0, lifecycle 관리자 7/10 활성, 수동 중단 |
| `f0_s10_a` | 10 정밀 도킹 (단일, 관리자 없음) | 19a1b82 | 210 | 48.7 → 34.9 | **통과** 5/5 (968.8 s). 실행기 `awaiting` 2.0 s → `unmanaged` |
| `f0_s12_c61` | 12 | 19a1b82 | 61 | 34.9 → 59.6 (중간 108) | **판정 없음** — 시작 +2.5 분에 launch.log 정지(하네스 정지로 봄), amr_04 navigation bringup 중단 |
| `f0_s11_a` | 11 BT 복구 (단일, 관리자 없음) | 19a1b82 | 210 | 50.1 → 44.9 | **실패** 1/4 — `lost` 사례 작업이 task_timeout(900 s)으로 FAILED. 수정 전 측정용 worktree 의 최근 10 실행 중 6 회가 같은 `lost: … final 3` 서명. 실행기는 작업 전에 `unmanaged` 확정 → hold 게이트가 한 번도 서지 않았다. **이 수정 탓 아님** |
| `f0_base_s12` | 12 | **d8ae80c (수정 전)** | 210 | 38.2 → 109.9 | **통과** 5/5 (2622.7 s) |
| `f0_s12_d` | 12 | b4ca041 | 210 | 15.2 → 33.0 | **판정 없음** — 시작 +1.3 분(08:47:11)에 하네스 launch_test 주 스레드 futex 대기·CPU 0, launch.log 18 분 정지 |
| `f0_ab_fix` | 12 | b4ca041 | 210 + `ROS_LOCALHOST_ONLY=1` | 23.1 → 41.7 | **판정 없음** — 하네스 정지 (09:15:25) |
| `f0_ab_base` | 12 | **d8ae80c (수정 전)** | 210 + `ROS_LOCALHOST_ONLY=1` | 23.1 → 41.7 | **판정 없음** — 하네스 정지 (09:10:26). 위와 **같은 시각 대조** |

시나리오 12 의 실행기별 첫 값 (실행기 시작 기준, 관리자 로그 `hold=… (첫 값)` 과 실행기 로그 `traffic/hold:` 대조):

| 실행 | amr_01 | amr_02 | amr_03 | amr_04 | amr_05 |
|---|---|---|---|---|---|
| A (210) | 유예 안 발견, 관리자 발행 +0.37 s 수신 | 유예 안, +0.10 s | 중단까지 15 분 미발견 | 미발견 | 미발견 |
| B (210) | 유예 안, +0.002 s | 유예 안, +0.015 s | 발견 2.3 s, +0.010 s | 발견 83 s, 중단까지 14 분 값 없음 | 발견 2.7 s, 중단까지 15 분 값 없음 |
| C (61) | 유예 안, +0.32 s | 유예 안, +0.84 s | 발견 3.5 s, +0.14 s | 발견 77 s, 중단까지 18 분 값 없음 | 발견 14 s, 중단까지 19 분 값 없음 |
| D (210) | 유예 안, +0.006 s | 유예 안, +0.021 s | 발견 2.3 s, +0.026 s | 발견 2.4 s, +4.2 s | 발견 7.9 s, 하네스 정지까지 값 없음 |
| A/B 수정 (210 lo) | 유예 안, +0.011 s | 유예 안, +0.013 s | 유예 안, +2.6 s | 발견 72 s, 관리자 unobserved 해제(+138 s) 뒤에도 로그 정지까지 값 없음 | 로그 정지(09:15:25)까지 미발견 |

B 와 A/B 수정 갈래의 amr_04 는 **양쪽이 다 매칭됐는데도**(실행기: 발행자 1·QoS 맞는 것 1, 관리자: 구독자가 있어 unobserved
해제를 냈다) 값이 도착하지 않았다. 같은 컨테이너의 새 참가자는 같은 latched 값을 즉시 받았으므로(`logs/f0_probe/probe_b1.txt`)
관리자 쓰기 쪽 결함은 아니다. 전송 계층(같은 실행에서 `Failed init_port fastrtps_port…: open_and_lock_file failed` SHM 포트 경합
로그)이 원인일 수 있으나 **미확인**이다. 이 전송 문제는 구독 QoS 와 무관하므로 수정 전에도 같은 로봇은 hold 를 받지 못했을
것이다 — 차이는 수정 전에는 조용히 통과하고 수정 뒤에는 경고를 내며 선다는 것이다(§4 위험 0).

기준선(수정 전) 시나리오 12 는 handover1006 §4 의 측위 수정 뒤 7 실행 4 통과(`Z12`·`R12c`·`S12a`·`S12c` 통과, `R12a`·`R12b`
`test_20` 378 행, `S12b` `test_10` 337 행)이고, 그 실행들의 시작 loadavg 는 3.3~48 이었다. 이번 날의 수정 전 대조는 2 회 중
1 회 통과, 1 회 하네스 정지다.

## 7. DDS 늦은 참여자 대응 (브랜치 `feature/traffic-hold-dds`, 2026-10-06 22시~)

작성: hold-dds 에이전트, 기준 `b33817f`. 원자료 `/home/CAPYI_AI/runs/b24/hold-dds/`, 실행 목록
`/home/CAPYI_AI/runs/b24/hold-dds/notebook/runs/hold-dds.jsonl` (보너스 4 의 `scripts/notebook.py --nb <그 절대 경로>` 로
필수 필드를 검사한다 — 이 브랜치에는 notebook 폴더가 없다). **7.1–7.4 는 실험·시험을 돌리기 전에 커밋했다(사전 등록).**
7.5 이후가 결과다. Gazebo 는 이 과제에서 돌리지 않는다(COORDINATION §9) — 효과 판정은 7.4 의 짝 실행 몫이다.

### 7.1 문제와 근거 — 이 브랜치가 고칠 수 있는 것과 없는 것

- **실측(§6)**: 늦게 뜬 실행기(amr_04·05)가 발행자와 매칭된 뒤(실행기 쪽 "발행자 1, QoS 맞는 것 1")에도 latched 값을
  수 분~19 분 받지 못했다. SHM `open_and_lock_file failed` 는 `f0_s12_a` 18 건 · `f0_s12_b` 10 건 · `f0_ab_fix` 19 건
  (localhost 프로파일 + `ROS_LOCALHOST_ONLY=1`), `f0_s12_c61`·`f0_s12_d`·`f0_base_s12`·`f0_ab_base` 0 건
  **[측정]** (`grep -rh open_and_lock_file /home/CAPYI_AI/wt/f0-holdfix/logs/<태그>`, 2026-10-06 22:10 다시 셈).
- **리서치**: 「실측 기반 후속 기술 조사」 "Fast DDS 늦은 참여자" 절과 노트 `fastdds_late_joiner.md`.
  원인 후보 1 = 참가자 쌍 단위 데이터 경로 고장(SHM 포트 재생성 등), 2 = writer 쪽 비대칭 매칭(SEDP 유실),
  3 = 관리자가 아직 안 냄(관측 전 — D5 로 의도한 대기), 기여 = 도메인 210 의 임시 포트 겹침. 권고 = 1 Hz 재발행 ·
  30 s 재구독 · 120 s 재시작 · 양쪽 매칭 수 로그 · UDP 전용 A/B · 도메인 102–214 금지.
- **불리한 것 먼저 — 이 브랜치의 한계** (DDS 동작에서 끌어낸 추론, 측정 아님):
  - 재발행은 writer 가 매칭하지 않은 reader(후보 2)에는 닿지 않는다 — DataWriter 는 매칭된 reader 에만 보낸다.
    경로가 고장 난 쌍(후보 1)에도 닿지 않는다. 재발행이 직접 고치는 것은 "매칭됐고 경로도 살아 있는데 transient_local
    재생이 한 번 실패한" 경우뿐이고, 그런 경우가 실제로 있었는지는 모른다. 확실한 효과는 **감지**다: 재발행이 있으면
    "매칭됐는데 30 s 동안 값이 없다" 는 정상 지연이 아니라 경로 고장이다.
  - 구독 재생성은 새 DataReader(새 GUID)를 SEDP 로 다시 알리므로 후보 2 에 기회를 준다. 같은 참가자(같은 수신 포트)를
    쓰므로 후보 1 은 못 고친다 — 후보 1 은 새 참가자, 즉 프로세스 재시작이 필요하다(D12 는 경보만 한다).
  - 시나리오 12 통과율에 주는 영향은 **미측정**이다.

### 7.2 결정 (D9–D14)

| # | 무엇을 | 어떻게 | 왜 |
|---|---|---|---|
| D9 | 관리자 hold 재발행 (D8 대체) | 평가된 값(`r.hold` 가 None 아님)만 `hold_republish_s`(1.0 s, 0 = 끔) 마다 다시 낸다(노드 시계, 평가 주기 안에서). 평가 전에는 내지 않는다(**D5 유지**). hold=true 이고 포켓이 있으면 포켓을 먼저 다시 낸다. 포켓-먼저 미룸 중(포켓 있음 ∧ hold≠true)에는 내지 않는다. 바뀐 값 발행도 재발행 시계를 0 으로 돌린다 | D8 의 전제("depth 1 이력 = 현재 값이면 재연결한 구독자가 받는다")가 §6 B·A/B 의 amr_04 에서 깨졌다. 비용은 로봇당 Bool 1 Hz. **미룸 예외**: 실행기는 false 를 받을 때마다 포켓을 지운다(`executor_context.cpp` `updateTrafficHold`) → 포켓과 hold=true 사이에 false 를 다시 내면 새 포켓이 지워진다. **포켓 재발행**: 실행기는 hold 시작보다 2 s 넘게 먼저 받은 포켓을 버린다(`kYieldPoseGrace`) → 재발행으로 hold=true 를 늦게 받은 실행기가 예전 포켓을 버리고 제자리에 선다 |
| D10 | 관리자 매칭 수 로그 | 주기마다 로봇별 `pub_hold.get_subscription_count()`(writer 쪽 DDS 매칭 수)를 보고 바뀌면 INFO (마지막 hold 값과 함께). 바뀐 기록을 진단용으로 남긴다 | 후보 1·2 를 가르는 측정(리서치 Gaps 첫 줄: writer matched 수가 측정되지 않았다) |
| D11 | 실행기 매칭 로그 · 구독 재생성 | 200 ms 마다 `get_publisher_count()`(reader 쪽 DDS 매칭 수)를 보고 바뀌면 INFO (그래프 발행자 수 · 첫 값 여부 · 재생성 횟수). awaiting ∧ 매칭 > 0 이 `traffic_hold.resubscribe_after_s`(30 s, 0 = 끔)를 넘으면 hold · yield_pose 구독을 새로 만들고, 새 구독이 매칭되면 시계를 다시 잰다 | 후보 2 에 기회. 매칭 0 이면 재생성하지 않는다: 그래프에 보이는데 매칭 0 이면 QoS 불일치(§3.1 의 volatile 발행자)이고 재생성으로 안 고쳐진다 |
| D12 | 실행기 경보 | awaiting 이 `traffic_hold.alarm_after_s`(120 s, 0 = 끔)를 넘으면 ERROR 한 번(awaiting 을 벗어났다 다시 들어오면 다시). 기존 10 s 경고는 그대로. **프로세스는 스스로 끝내지 않는다** | 재시작은 런치 정책이다: `behavior.launch.py` 의 실행기에 respawn 이 없어 스스로 끝내면 그 로봇 실행기가 영영 없어진다. 작업 · 적재 · 충전소 점유 상태도 잃는다. 재시작은 170 참가자 발견 폭주를 더한다(후보 2 의 원인 후보). 관리자 쪽 원인(관측 못 함 → D6 해제 전)이면 재시작해도 소용없다. respawn · lifecycle 감시는 bringup 소유자에게 요청한다 |
| D13 | `run_integration.sh` 도메인 | 명시값 102–214 · 233 이상을 **거부**(종료 코드 2). `auto` 의 1–101 무작위 선택은 유지. 앞자리 0 은 10 진수로 정규화. 선택한 도메인의 "임시 포트 범위 · 65535 에 닿기 전 참가자 수" 를 계산해 200 미만이면 안내 | ROS 2 문서 안전 범위(0–101 · 215–232). 경고는 사람이 놓친다(§6 의 B 실행이 210 을 명시했고 경고 대상이었다) |
| D14 | UDP 전용 프로파일 | 새 파일 `fastdds_multi_robot_udponly*.xml` — 구성은 E1 결과로 확정 | 후보 1(SHM) A/B. `_noshm` 은 이름과 달리 SHM 을 쓴다 → 머리말에서 바로잡는다 |

**과제 문구와 다르게 한 것**: 경보 조건을 "첫 값 미수신 + 발행자 매칭됨이 120 s" 대신 "awaiting 120 s(매칭 여부 무관)" 로 했다.
그래프에는 보이는데 매칭이 0 인 채(QoS 불일치 · reader 쪽 매칭 실패) 영영 선 로봇도 경보 대상이어야 해서다. 매칭된 경우는 둘이
거의 같다(awaiting 시작 ≤ 매칭 시작).

### 7.3 사전 등록 — 실험 · 시험과 판정 규칙

| id | 질문 | 가설 · 예측 | 설정 | 판정 규칙 |
|---|---|---|---|---|
| E1 | Humble rmw_fastrtps 6.2.10 에서 각 프로파일이 오류 없이 읽히고 실제로 무슨 전송을 쓰나. `ROS_LOCALHOST_ONLY=1` 은 XML 사용자 전송에 무엇을 하나 | 모두 읽힌다. multi_robot · localhost · noshm 은 SHM 을 쓰고 udponly 는 안 쓴다. **[코드 기억, 미확인]** rmw 는 localhost_only 일 때 XML 사용자 전송 뒤에 UDPv4(127.0.0.1)+SHM 을 **덧붙인다** → udponly + `ROS_LOCALHOST_ONLY=1` 에서도 SHM 이 생긴다 (그렇다면 `test_launch.py` 주석 "사용자 전송이 rmw 의 localhost 설정을 대체" 와 어긋난다) | `amr-ai/rl:0.3` 새 컨테이너(사설 IPC · 기본 브리지), DOMAIN 43, 프로파일 5(기존 3 + 새 것) × `ROS_LOCALHOST_ONLY` {0, 1}. 프로세스 2 개(발행 · 구독, rclpy, reliable + transient_local), 값 하나 | SHM 사용 ⇔ 새 IPC 이름공간의 /dev/shm 에 `fastrtps_*` 파일 ≥ 1. "읽힘" ⇔ stderr 에 `XMLPARSER`·`Error` 없음 ∧ 값 수신. 새 파일을 "UDP 전용" 이라 부를 조건 = 그 조합에서 SHM 파일 0 ∧ 수신 |
| E2 | 새 · 기존 프로파일이 Fast DDS 2.6.12 XSD 에 맞나 | 맞는다 | `xmllint --schema fastRTPS_profiles.xsd`(Fast-DDS v2.6.12 태그의 파일, 저장소에 넣지 않음) | 종료 코드 0 |
| E3 | 실행기 "QoS 맞는 것"(`get_publisher_count`)과 관리자 `get_subscription_count` 가 DDS 매칭 수인가 (리서치 노트 22 행 [미검증]) | 그렇다: QoS 비호환이면 그래프 수 1 · 매칭 0 | 노드 시험: volatile 발행자 + 실행기(transient_local 구독), deadline 을 요구하는 구독자 + 관리자 latched 발행자 | 그래프 1 ∧ 매칭 0 이면 확인 |
| E4 | 새 동작이 시험되고 회귀가 없나 | 새 단위 · 노드 시험 통과, 기존 774 시험 그대로 통과 | `colcon build` Release `-Wall -Wextra -Wpedantic -Werror`, `colcon test --packages-select amr_behavior amr_fleet amr_bringup`, DOMAIN 43, `ROS_LOCALHOST_ONLY=1` | 경고 0, 실패 0, 시험 수 = 기준선 + 새 시험 (기준선: amr_behavior · amr_fleet 774 · 0 실패 · 60 skip, `runs/b24/hold-dds/logs/base_*`) |
| E5 | 시험이 공허하지 않은가 (변이 검사, `cpu_slot.sh`) | 아래 21 개 변이를 모두 시험이 잡는다 | 변이 하나씩 적용 → 관련 시험만 → 되돌림 | 하나라도 살아남으면 시험을 더하거나 "동치 변이" 근거를 적는다 |
| E6 | 도메인 규칙이 맞게 거부 · 허용하나 | 0–101 · 215–232 허용(정규화 값 출력), 102–214 · 233+ · 정수 아님 거부, auto 는 1–101 이고 호출 셸 값을 피한다 | `run_integration.sh --self-test` (ROS 없이) | 표 전부 일치 |

E5 변이 목록 (f0 의 M1–M15 에 이어 번호):

| # | 되돌릴 가드 | 잡을 것으로 예상한 시험 |
|---|---|---|
| M16 | 재생성 조건에서 매칭 무시 (매칭 0 이어도 재생성) | 감시 단위 시험, 노드 시험(volatile 발행자 = 재생성 없음) |
| M17 | 재생성 경계 `>=` → `>` | 감시 단위 시험(경계값) |
| M18 | 재생성 뒤 매칭 시계를 다시 재지 않음 | 감시 단위 시험 |
| M19 | 매칭이 0 이 돼도 재생성 시계 유지 | 감시 단위 시험 |
| M20 | 경보가 한 번이 아니라 매 단계 | 감시 단위 시험 |
| M21 | 첫 값 수신 · unmanaged 에서 시계를 비우지 않음 | 감시 단위 시험 |
| M22 | 노드가 재생성 신호를 받고도 구독을 다시 만들지 않음 | 노드 시험(구독 GID 가 바뀜) |
| M23 | 노드가 매칭 대신 그래프 발행자 수를 감시에 넘김 | 노드 시험(volatile 발행자) |
| M24 | 매칭 수 변화 표시 안 함 | 감시 단위 시험 |
| M25 | 평가 전(None)에도 재발행 (None → false) | `test_hold_published_only_after_evaluation` |
| M26 | 재발행 끔 (주기 판정이 늘 거짓) | 새 재발행 시험 |
| M27 | 재발행 주기 무시 (매 주기) | 새 재발행 시험 |
| M28 | 포켓-먼저 미룸 중에도 false 재발행 | 새 미룸 시험 |
| M29 | hold=true 재발행 때 포켓을 다시 내지 않음 | 새 재발행 시험 |
| M30 | 바뀐 값 발행이 재발행 시계를 0 으로 돌리지 않음 | 새 재발행 시험 |
| M31 | writer 매칭 수 변화 기록 안 함 | 새 매칭 기록 시험 |
| M32 | 시계가 뒤로 가면(sim 재시작) 재발행을 영영 안 함 | 재발행 판정 단위 시험 |
| M33 | 102–214 거부 → 경고만 (기존 동작) | E6 |
| M34 | 거부 범위 102–213 (214 허용) | E6 |
| M35 | 233 이상 허용 | E6 |
| M36 | 앞자리 0 정규화 제거 | E6 |

### 7.4 Gazebo 시나리오 12 짝 실행 계획 (이번에는 돌리지 않음 — 조용한 시간대, `exclusive.sh`/`gz_slot.sh`)

- **짝**: 수정 전(`b33817f`) · 수정 후(이 브랜치 HEAD) 를 같은 이미지(`amr-ai/rl:0.3`), 같은 Release `-Werror` 빌드로
  **같은 시각에 나란히**(gz 슬롯 2 개) 돌린다 — 부하 조건을 짝 안에서 맞춘다. **≥ 6 쌍**, 짝마다 어느 쪽이 슬롯 0 인지 번갈아.
- **도메인**: 짝마다 안전 범위의 서로 다른 값(예: 61 · 63, 71 · 73 …, 간격 2 이상 — 170 참가자는 d+1 범위까지 쓴다),
  IGN 파티션도 따로. `--domain-id` 를 명시하고 `ROS_LOCALHOST_ONLY` 는 두 갈래 같게.
- **로그 태그**: `hdds_s12_<쌍>_{base,fix}`, 각 로그 디렉터리에 HEAD.txt · `git diff --stat` · loadavg 시작/끝 (§3 규칙).
- **1차 지표(사전 등록)**: 로봇별 time-to-first-hold(실행기 시작 → 첫 값, 실행기 로그), awaiting 이 120 s 를 넘은 로봇 수
  (수정 후는 ERROR 경보 수와 같아야 한다), 관리자 writer 매칭 시각(D10 로그)과 실행기 reader 매칭 시각(D11 로그)의 쌍.
  **2차**: 시나리오 판정(통과 · 실패 · 판정 없음), 하네스 정지, `open_and_lock_file` 건수, 재생성 횟수, 재발행 뒤 첫 값까지 시간.
- **판정 규칙**: (a) 수정 후 갈래의 "120 s 넘게 awaiting" 로봇 수 합 ≤ 수정 전 갈래의 "120 s 넘게 값 없음" 로봇 수 합이면
  "나빠지지 않음". 6 쌍으로는 통계적 차이를 주장하지 않고 쌍별 값을 모두 적는다. (b) 수정 후에 writer 매칭 ≥ 1 ∧ reader
  매칭 ≥ 1 ∧ 재생성 뒤에도 값 없음인 로봇이 하나라도 나오면 후보 1(경로 고장) 쪽 증거 → 다음 단계 UDP 전용 A/B
  (수정 후 코드로 SHM 현행 vs UDP 전용, ≥ 6 쌍). (c) writer 매칭 0 ∧ reader 매칭 ≥ 1 이 오래 가면 후보 2 쪽 증거.
  UDP 전용 갈래는 카메라 영상이 UDP 조각 전송으로 떨어져 인식 처리율이 낮아질 수 있으므로(프로파일 머리말의 실측 0.32–0.54)
  **판정은 1차 지표로만** 하고 시나리오 통과율은 참고로 둔다.

### 7.5 결과 (사전 등록 뒤, 2026-10-06 22:10–23:20, 공유 서버 · 결과는 Gazebo 없이 낸 것)

**불리한 것 먼저.**
- 이 절의 어떤 결과도 **시나리오 12 의 늦은 참여자 문제가 줄었는지 보여 주지 않는다** — 그것은 7.4 의 짝 실행 몫이고
  **미측정**이다. 아래는 "코드가 설계대로 동작하고 시험이 공허하지 않다" 와 "전송 설정의 실제 동작" 까지다.
- 기존 `_localhost` 프로파일 + `ROS_LOCALHOST_ONLY=1`(f0 의 A/B 실행 `f0_ab_*` 구성)에서 참가자마다 **SHM 전송이 2 개**다
  (E1). 소규모 탐침에서는 오류 없이 통신했지만 170 참가자에서 무슨 영향이 있는지는 모른다. f0 의 A/B 수정 갈래
  `open_and_lock_file` 19 건 · 수정 전 갈래 0 건이 이 구성에서 나왔으므로 원인 후보에서 뺄 수 없다.
- Fast DDS 2.6.12 의 공식 XSD 는 그대로는 쓸 수 없다 (E2). 스키마 검사는 XSD 를 두 군데 고친 사본으로 했고, 그 사본은
  전송 `type` 값을 제한하지 않는다(음성 대조 `UDPv5` 통과). **프로파일이 실제로 무엇을 하는지는 E1 의 실행 확인이 근거다.**
- 재생성(D11)은 재발행(D9)이 매 초 값을 보내는 한 드물어야 하는데, 그 빈도는 Gazebo 로만 잴 수 있다 (미측정).
- 남은 위험: 재발행한 hold=false 가 다음 주기의 새 포켓보다 늦게 도착하면(다른 토픽 사이 순서 보장 없음) 실행기가 새 포켓을
  지운다 → hold=true 뒤 제자리 대기(안전 쪽, 교착 해소 지연). 같은 참가자 쌍 · 같은 수신 큐라 드물 것으로 **본다(미측정)**.
  또 BT `Yield` 는 시작할 때 포켓을 한 번 읽으므로 hold=true 를 포켓보다 먼저 받은 늦은 참여자는 뒤에 온 포켓으로 가지
  않는다 — 수정 전부터 있던 성질이고 이 브랜치가 바꾸지 않았다(BT 는 소유 밖 — 요청).

**E1 — 프로파일별 실제 전송 [측정]** (`runs/b24/hold-dds/E1/e1_results.txt`, v2; v1 은 오류 검사가 stderr 만 봐서 다시 돌렸다 —
Fast DDS 로그는 stdout 으로 나온다). 새 컨테이너(사설 IPC)마다 발행 · 구독 프로세스 2 개, 값 1 개, DOMAIN 43. 10 조합 모두 수신,
XMLPARSER · Error 0 줄.

| 프로파일 | `ROS_LOCALHOST_ONLY=0`: /dev/shm `fastrtps_*` | `=1` | `=1` 의 UDP 바인드 |
|---|---|---|---|
| `fastdds_multi_robot.xml` | 8 | **12** | 0.0.0.0 + 127.0.0.1 + 컨테이너 주소 |
| `fastdds_multi_robot_localhost.xml` | 8 | **12** | 127.0.0.1 만 |
| `fastdds_multi_robot_noshm.xml` | **8** | 8 | 127.0.0.1 만 |
| `fastdds_multi_robot_udponly.xml` (새) | **0** | **8** | 0.0.0.0 + 127.0.0.1 + 컨테이너 주소 |
| (후보 `udponly_localhost`, 지움) | 0 | 8 | 127.0.0.1 만 |

8 = 참가자 2 × (세그먼트 + 잠금 + 포트 + 포트 잠금), 12 = 참가자마다 세그먼트가 2 개(SHM 전송 2 개).
- (a) 모두 오류 없이 읽힌다. (b) `_noshm` 은 SHM 을 쓴다(이름이 틀렸다 → 머리말 정정). (c) **예측대로** rmw 는
  `ROS_LOCALHOST_ONLY=1` 이면 XML 사용자 전송을 대체하지 않고 UDPv4(127.0.0.1)+SHM 을 덧붙인다: udponly 에 SHM 파일 8 개와
  127.0.0.1 바인드가 생긴다. → **UDP 전용 갈래는 `ROS_LOCALHOST_ONLY=0` 으로만 성립한다.** `test_launch.py` 의 주석
  "사용자 전송이 rmw 의 localhost 설정을 대체" 는 틀렸다(소유 밖 — 요청). `_localhost` 변형이 필요한 것은 맞다 — XML 의
  묶이지 않은 UDPv4 는 0.0.0.0 에 묶이므로 — 그러나 이유가 다르다(머리말 정정).
- 결정 D14 확정: 새 파일은 `fastdds_multi_robot_udponly.xml` **하나**(모든 인터페이스, SHM 없음). A/B 의 두 갈래가
  `fastdds_multi_robot.xml` 과 SHM 유무만 다르게 하려고 127.0.0.1 변형은 지웠다.
- 단일 표본 지연(v1 의 2.67 s 수신)은 v2 에서 재현되지 않았다(0.01–0.14 s) — 해석하지 않는다.

**E2 — XSD [측정]** (`runs/b24/hold-dds/E2/`). Fast-DDS v2.6.12 `resources/xsd/fastRTPS_profiles.xsd`(sha256 `92e54009…`):
- 원본은 컴파일되지 않는다 (정의 없는 타입 `int32` · `uint64` · `uint32` 3 곳). 전역 루트도 `<dds>` 뿐이라 `<profiles>`
  문서는 `<dds>` 로 감싸 검사했다.
- 타입을 고친 사본은 `userTransports` 의 자식으로 `<id>` 를 요구한다. 그런데 **2.6.12 파서는 `<id>` 를 거부한다**:
  `<id>` 로 쓴 변형을 실제로 띄우면 `XMLPARSER Error: Node 'userTransports' without content` → 프로파일 전체를 버리고
  **기본 전송(SHM + UDP)으로 조용히 돌아간다**(SHM 파일 8 개, 통신은 된다). 파서가 읽는 것은 `<transport_id>` 다.
- `transport_id` 로 고친 사본(`xsd_patch2.diff`, 4 줄)에서 4 프로파일 모두 유효, 오타 태그(`mutation_triez`) 음성 대조는 거부,
  전송 `type` 오값(`UDPv5`)은 통과(스키마가 값을 제한하지 않는다).
- 덤: 2.6.12 XSD 에 SHM 전송의 `healthy_check_timeout_ms` 가 있다(리서치 Gaps "2.6 XML 지원 미확인" 의 스키마 쪽 답. 파서 지원은 미확인).

**E3 — 매칭 수의 뜻 [시험]**: 실행기 `get_publisher_count` 와 관리자 `get_subscription_count` 는 그래프(발견)가 아니라 **DDS 매칭**
수다. QoS 비호환 끝점은 그래프에 1 로 보이고 매칭은 0 이다 — 실행기 쪽 `IncompatiblePublisherIsSeenButNotMatchedSoNoResubscribe`
(volatile 발행자: `count_publishers` 1, 매칭 0), 관리자 쪽 `test_hold_matching_logged_and_incompatible_not_counted`(deadline 을
요구하는 구독자: `count_subscribers` 1, 매칭 0). → §6 B 실행의 실행기 경고 "QoS 맞는 것 1" 은 **reader 쪽 DDS 매칭**이었다
(리서치 노트 22 행의 [미검증] 해소). writer 쪽 매칭은 그때 측정되지 않았다 — D10 이 이제 그것을 남긴다.

**E4 — 시험 · 빌드 [측정]** (`runs/b24/hold-dds/logs/`, `amr-ai/rl:0.3`, DOMAIN 43, `ROS_LOCALHOST_ONLY=1`):
- 전 패키지 11 개 Release `-Wall -Wextra -Wpedantic -Werror` 빌드 rc 0, 경고 0 (`fix1_build.log`).
- `colcon test amr_behavior amr_fleet amr_bringup`: **892 시험, 0 실패, 60 skip(cppcheck)** (`fix2_*`). amr_behavior · amr_fleet 만
  786 = 기준선 774(`b33817f`, `base_*`) + 새 12 (`test_task_executor_node` 11 → 20: 노드 2 + 감시 단위 7, amr_fleet pytest
  371 → 374). amr_bringup 106 통과(xmllint 가 fastdds 프로파일 4 개 + package.xml 을 본다). 첫 실행(`fix1`)은 린트만 28 건 실패(cpplint 한글 폭 23 · uncrustify 1 ·
  pep257 D403 1)라 고친 뒤 다시 돌렸다.
- 안정성: 새 gtest 10 개(감시 7 + 노드 3) `--gtest_repeat=10` 10/10, 새 · 바꾼 pytest 4 개 × 10 회 10/10 (loadavg 표는 실행 목록).
- 시험 소요 [측정, 공유 부하 하 예비값]: `test_task_executor_node` 11.5 s (새 노드 시험 2 개가 약 1.8 s · 3.3 s).

**E5 — 변이 검사 [측정]** (`runs/b24/hold-dds/E5/results.jsonl`, `cpu_slot.sh`, loadavg 19.5 → 26.6): **21 개 모두 잡혔다**, 같은 실행기의
변이 없는 대조는 gtest 20 · pytest 11 · self-test 통과, 끝난 뒤 worktree 변경 0.

| # | 되돌린 가드 | 잡은 시험 |
|---|---|---|
| M16 | 매칭 0 이어도 재생성 | `IncompatiblePublisher…`(노드), `MatchLoss…ZeroMatchNeverResubscribes` |
| M17 | 재생성 경계 `>=` → `>` | 감시 단위 4 개 (경계값) |
| M18 | 재생성 뒤 시계 유지 | `ResubscribesAtBoundaryAndRestartsClockAfterward`, `ContextSteps…` |
| M19 | 매칭이 끊겨도 시계 유지 | `MatchLossRestartsResubscribeClock…` |
| M20 | 경보 매 단계 | `ResubscribesWhileMatched…AlarmsOnce`(노드), `AlarmsOncePerAwaiting…` |
| M21 | awaiting 을 벗어나도 시계 · 경보 유지 | `AlarmsOncePerAwaiting…RearmsAfterLeaving`, `FirstValueClearsClocks` |
| M22 | 재생성 신호에도 구독을 안 만듦 | `ResubscribesWhileMatched…`(구독 GID 가 바뀌지 않음) |
| M23 | 감시에 그래프 수를 넘김 | `IncompatiblePublisher…`(노드) |
| M24 | 매칭 변화 표시 안 함 | `MatchedChangeIsFlaggedInEveryState` |
| M25 | 평가 전(None)에도 재발행 | `test_hold_published_only_after_evaluation`, 판정 단위, 재발행 노드 |
| M26 | 재발행 끔 | 판정 단위, `test_hold_republished_at_period_pocket_first` |
| M27 | 매 주기 재발행 | 같은 두 시험 |
| M28 | 포켓-먼저 미룸 중 false 재발행 | 같은 두 시험 |
| M29 | hold=true 재발행에 포켓 안 붙임 | `test_hold_republished_at_period_pocket_first` |
| M30 | 바뀐 값 발행이 재발행 시계를 안 돌림 | 같음 |
| M31 | writer 매칭 변화 기록 안 함 | `test_hold_matching_logged_and_incompatible_not_counted` |
| M32 | 시계가 뒤로 가면 재발행 안 함 | 판정 단위 |
| M33 | 102–214 경고만 | `--self-test` (102–214 전부) |
| M34 | 214 허용 | `--self-test` (214) |
| M35 | 233 이상 허용 | `--self-test` (233–240) |
| M36 | 앞자리 0 정규화 제거 | `--self-test` (`0101` · `007` · `000` · 긴 0) |

**E6 — 도메인 규칙 [측정]** (`runs/b24/hold-dds/E6/e6_results.txt`): `--self-test` 통과(0–240 전부 · 정수 아님 · 앞자리 0 · 긴 값 ·
auto 2000 회 · 참가자 상한). 실제 경로(ROS 소싱 뒤 도메인 블록)에서 210 · 102 · 214 · 233 · `env`(150) 는 종료 코드 2 와 이유,
`0101` → 101(+ "참가자 54 개" 안내), 215 · 61 허용, auto(호출 셸 43)는 65 — 호출 셸 값을 피하는 규칙은 self-test 가 2000 회로 확인. 계산 확인: 도메인 232 의 상한 63 이 ROS 2 문서의
"232 는 63 프로세스" 와 같다. **도메인 100 · 101 은 참가자 179 · 54 개를 넘으면 임시 포트 범위에 든다** — auto(1–101)가 1/101 씩의
확률로 고를 수 있다. 과제 지시대로 auto 범위는 바꾸지 않았고 안내만 낸다 (요청: 1–99 로 좁히는 것을 검토).

**E7 — 실제 노드가 그 프로파일로 뜨는지 [측정, 사전 등록 밖 추가]** (`runs/b24/hold-dds/E7/e7_results.txt`): `ros2 run` 으로
`traffic_manager_node`(amr_01, 구역 없음, `obs_timeout_s` 1 s) + `task_executor_node`(/amr_01)를 20 s, `ROS_LOCALHOST_ONLY=0`.
`fastdds_multi_robot.xml`: SHM 파일 10, 파서 오류 0. `fastdds_multi_robot_udponly.xml`: **SHM 파일 0**, 파서 오류 0. 두 경우 모두
관리자 `amr_01 traffic/hold 매칭 구독자 0 → 1`(D10) → 관측 없음 해제 `hold=False (첫 값) unobserved`(D6) → 실행기
`traffic/hold 매칭 발행자 0 → 1 (그래프 1, 상태 awaiting, 구독 재생성 0)`(D11) → `첫 값 수신 clear`. 첫 시도는 탐침 스크립트가
빈 파라미터(`charger_claims_topic:=`)를 넘겨 rcl 이 실행기를 띄우지 못했다 — 탐침 결함이라 그 인자를 빼고 다시 돌렸다(v1 로그 보존).

### 7.6 해석 · 다음

- 이 브랜치가 바꾼 동작 (수정 후 정상 경로에서 보이는 것): 관리자는 로봇마다 hold 를 1 Hz 로 다시 내고(포켓 양보 중이면
  포켓도), 매칭 구독자 수 변화를 `… traffic/hold 매칭 구독자 A → B` 로 남긴다. 실행기는 `traffic/hold 매칭 발행자 A → B` 를
  남긴다. 값이 오는 한 재생성 · 경보는 일어나지 않는다(E4 의 노드 시험 `LateExecutor…` 에서 재생성 0 · 경보 0).
- 다음 (조용한 시간대): 7.4 의 짝 실행 ≥ 6 쌍 → 판정 (a)(b)(c) → 필요하면 UDP 전용 A/B. 둘 다 `ROS_LOCALHOST_ONLY=0`(E1).
- 소유 밖 요청: (1) `src/amr_behavior/launch/behavior.launch.py` 실행기 respawn 정책 결정(D12 — 재시작이 후보 1 의 유일한 회복책),
  (2) `task_executor_node.hpp` 머리말에 새 파라미터 두 개 문서화, (3) `tests/integration/unit` 에서 `run_integration.sh --self-test`
  를 부르는 시험 한 줄(CI 에 걸리게), (4) `src/amr_bringup/test/test_launch.py:310` 주석 정정(E1), (5) auto 도메인 범위 1–99
  검토(E6), (6) BT `Yield` 가 뒤에 온 포켓을 쓰게 할지(7.5 남은 위험).

### 7.7 짝 실행 절차 (사전 등록 뒤 추가 — 판정 규칙은 7.4 그대로)

```bash
# 0) 조용한 시간대: gz 슬롯 2 개가 비고 loadavg(1 분) < 8. 보너스 1 측정 시간대(exclusive.sh)와 겹치지 않게.
W=/home/CAPYI_AI/runs/b24/hold-dds/s12
mkdir -p $W/base_ws && git -C /home/CAPYI_AI/wt/hold-dds archive b33817f | tar -x -C $W/base_ws
# 1) 두 갈래 모두 같은 이미지 · 같은 빌드 (Release, -Wall -Wextra -Wpedantic -Werror, --packages-up-to amr_bringup)
# 2) 쌍 k: 두 갈래를 동시에 (도메인 간격 2 이상, 파티션 따로, ROS_LOCALHOST_ONLY 는 비움 = fastdds_multi_robot.xml)
gz() {  # $1 = 태그, $2 = 워크스페이스, $3 = 도메인
  /home/CAPYI_AI/env/gz_slot.sh hold-dds docker run --rm --name amr-ai-hold-dds-gz-$1 --gpus all --shm-size=8g \
    -v $2:/ros2_ws -w /ros2_ws amr-ai/rl:0.3 bash -lc "./scripts/run_integration.sh --no-unit --timeout-scale 3 \
    --domain-id $3 --ign-partition amr_ai_hdds_$1 --log-dir /ros2_ws/logs/hdds_s12_$1 12"
}
gz p1_base $W/base_ws 61 & gz p1_fix /home/CAPYI_AI/wt/hold-dds 63 & wait    # 다음 쌍은 슬롯 순서를 바꾼다
# UDP 전용 A/B (수정 후 코드 두 갈래): docker run 에 -e FASTRTPS_DEFAULT_PROFILES_FILE=\
#   /ros2_ws/install/amr_bringup/share/amr_bringup/config/fastdds_multi_robot_udponly.xml 를 한쪽에만
```

각 로그 디렉터리에 HEAD.txt · `git diff --stat` · 시작/끝 loadavg 를 직접 쓴다(§3). 지표 추출(`launch.log`):
- 실행기 시작 `BT 로드`, 첫 값 `traffic/hold: 첫 값 수신`, 대기 경고 `traffic/hold 첫 값을 N s 째` (두 갈래 공통).
- 수정 후만: reader 매칭 `traffic/hold 매칭 발행자 A → B`, writer 매칭 `<로봇> traffic/hold 매칭 구독자 A → B`,
  재생성 `구독을 다시 만든다`, 경보 `첫 값을 N s 째 못 받았다 (매칭`.
- 공통: `grep -c open_and_lock_file`, `XMLPARSER`(0 이어야 한다), UDP 갈래는 실행 중 `ls /dev/shm | grep -c fastrtps`(0 이어야 한다).

### 7.8 실행 목록 (요약 — 원본 `/home/CAPYI_AI/runs/b24/hold-dds/notebook/runs/hold-dds.jsonl`, `check --paths` 문제 0)

| run_id | 무엇 | 커밋 | 결과 | 표기 |
|---|---|---|---|---|
| HDDS_base_test_b33817f | 기준선 빌드 · 시험 | b33817f | 774 · 0 실패 · 경고 0 | [측정] |
| HDDS_E1_v1 | 프로파일 × LOCALHOST_ONLY (오류 검사 결함) | afa22f0+ | v2 로 대체 | [측정] |
| HDDS_E1_v2 | 같음, stdout 포함 | afa22f0+ | 위 E1 표 | [측정] |
| HDDS_E2_xsd | XSD 검사 · `<id>` 변형 실행 | afa22f0+ | 위 E2 | [측정] |
| HDDS_fix1_build_test | 전 패키지 -Werror · 시험 | afa22f0+ | 경고 0, 린트 28 실패 | [측정] |
| HDDS_fix2_build_test | 린트 고친 뒤 | afa22f0+ (= c8d64e8 내용) | 892 · 0 실패 | [측정] |
| HDDS_E6_domain | 도메인 규칙 | c8d64e8 | 위 E6 | [측정] |
| HDDS_E4b_stability | 새 시험 반복 | a91bbfe | 10/10 · 10/10 | [측정] |
| HDDS_E5_mutation | 변이 21 | a91bbfe | 21/21 잡힘 | [측정] |
| HDDS_E7_realnodes | 실제 관리자 · 실행기, 두 프로파일 (사전 등록 밖) | f0ec40e | udponly SHM 0 · 첫 값 수신 | [측정] |
