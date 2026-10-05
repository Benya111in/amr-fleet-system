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
| D8 | 재연결 시 hold 재발행 | **하지 않는다** | 바뀔 때만 내고 depth 1 이므로 writer 이력 = 현재 값. D1 만으로 재연결한 구독자가 현재 값을 받는다 |

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
- **D8 재발행 불필요.** `_publish_hold` 만 hold 를 내고 바뀔 때만 부르며(`_apply`, D6), 매번 `r.hold` 를 갱신한다
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

1. **관리자 사망 뒤 마지막 값.** 받은 값이 false 였던 로봇은 관리자가 죽은 뒤 관리 없이 움직인다. 보고서가 말하는
   "인계 공백" 과 같은 종류이고 이 수정으로 바뀌지 않았다. 보고서의 비대칭 분할(상향만 끊김 → lost 반납 → 다른 로봇에
   부여, 이 로봇은 hold 갱신을 못 받음) 틈도 그대로다 — 둘 다 분산 폴백(P1~P3) 범위.
2. **유예보다 늦은 발견.** `auto` 에서 실행기 시작 뒤 2 s 안에 관리자 발행자가 그래프에 안 보이면 그 뒤 매칭될
   때까지 `unmanaged`(통과)다. 매칭되면 latched 값이 곧 도착하고 BT 게이트가 reactive 라 그 순간 멈춘다. 재기동한
   실행기는 작업이 없어 그 창에서 움직일 이유가 대개 없지만(작업은 fleet_manager → assign_task 경로), 배터리 부족
   자동 충전처럼 작업 없이 움직이는 경로는 있다. 교통 관리자가 있다고 아는 런치는 `required` 를 줘야 이 창이 닫힌다 —
   `behavior.launch.py` 에 `traffic_hold_policy` 인자를 두었고, `amr_bringup` 이 `with_fleet` 일 때 넘기는 배선은 그 소유자
   몫이다.
   **[측정] 이 창은 이론이 아니다.** Gazebo 시나리오 12 첫 실행(`logs/f0_s12_a`, 2620160, 호스트 loadavg 17→78 /
   32 코어, 공유 부하 하 예비값, 기동 단계에서 수동 중단)에서 실행기 5 대 중 `amr_01`·`amr_02` 는 유예 안에 관리자
   발행자를 보고 첫 값을 받았지만(`awaiting` → `clear`, 관리자 발행 뒤 0.37 s·0.10 s), 스택 지연 간격(6~9 s)으로 늦게 뜬
   `amr_03`·`amr_04`·`amr_05` 는 2 s 안에 발행자를 못 보고 `unmanaged` 가 됐다. 그 뒤 중단까지 약 15 분 동안 `amr_03` 은 관리자가
   낸 첫 값(관리자 로그 `amr_03 hold=False (첫 값)`)을 받았다는 기록이 없다. 관리자는 `amr_04`·`amr_05` 에 첫 값도,
   당시 구현의 "관측 못 한 로봇 해제"(첫 주기 + sim 3 s)도 내지 않았다 — 둘 중 하나는 주기마다 반드시 나와야 하므로
   관리자 주기 자체가 멈췄을(예: `/clock` 수신 중단) 가능성이 크지만 원인은 확인하지 못했다. 같은 때 컨테이너 안에서
   45 s 동안 돈 그래프 탐침은 노드 77 개만 보았다
   (`logs/f0_probe/probe1.txt`, 프로세스는 약 150). 즉 이 부하에서는 DDS 발견이 분 단위로 늦거나 실패했고, 그 경우
   어떤 유예 값도 창을 닫지 못한다. `required` 는 창을 닫는 대신 그 로봇을 발견될 때까지 세운다 (fail-closed) —
   통과율과 안전 사이의 선택이라 측정 없이 기본값으로 켜지 않았다. 할당된 `ROS_DOMAIN_ID` 210 의 Fast DDS 포트
   (59900~60258)가 리눅스 임시 포트 범위(32768~60999)와 겹친다는 점(`run_integration.sh` 경고)이 원인에 끼는지는
   **미측정**이다 — 같은 실행에서 59900 멀티캐스트 포트에 171 개 소켓이 정상으로 묶여 있었다.
3. **관측 못 한 로봇의 시간 기반 해제(D6).** 시간이 지나면 풀어 주는 fail-open-after-timeout 형태다. 대상을
   "한 번도 관측하지 못한" 로봇으로 좁혔고, 그런 로봇은 관리자의 예측·토큰 밖이므로 해제해도 관리자가 줄 수 있었던
   보호를 잃는 것은 없다. 실행기가 뜬 뒤 측위(EKF)가 `obs_timeout_s`(sim 3 s)보다 늦게 오도메트리를 내면 그 로봇은
   잠깐 false 를 받았다가 관측되는 주기에 평가 값으로 바뀐다 — 수정 전(기동 즉시 false)보다 나빠지지는 않는다.
4. **volatile 발행자.** 누군가 volatile 로 `traffic/hold` 를 내면 `auto` 는 그 로봇을 영영 세운다(경고와 함께).
   의도한 fail-closed 이지만 운영자가 알아야 한다.
5. **유예 2 s 가 측정 판정에 닿는지 미측정.** 단일 로봇 시나리오(10·11·13·14)는 이번에 Gazebo 로 돌리지 않았다 (§5).

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
- 변이 검사·전체 단위 시험·Gazebo 회귀 결과는 에이전트 보고(`/home/CAPYI_AI/logs/agents/f0-holdfix/`)와 PR 본문에 적는다.
