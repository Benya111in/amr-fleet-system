# 설계 브리프: BT 작업 수행 · ArUco 정밀 도킹 · 마감 기반 스케줄링 (명세 4.8) — 리뷰 반영 개정판

작성일 2026-09-21, 개정 2026-09-22 · 대상 스택: ROS2 Humble / Nav2 1.1.20 / Gazebo Fortress 6.18 / C++17·Python 3.10
검증 범위: 스택 관련 주장은 현행 이미지 `amr-fleet-system:wf-final` 을 일회용 컨테이너(`docker run --rm`)로 띄워 확인했고(컨테이너 `amr_dev` 는 건드리지 않음), "Humble 에 배포됨" 여부는 rosdistro `humble/distribution.yaml` 사본(`checks/humble_distribution.yaml`, 2026-09-22 취득)으로 확인했다. 문헌은 arXiv API·Crossref·Europe PMC·저자 페이지·Nav2 소스에서 초록/코드/메타데이터를 직접 가져온 것만 VERIFIED 로 표기한다. 수치는 `rev_calc.py` 와 독립 재유도 `checks/audit_checks.py`(출력 `checks/audit_checks.out`), 3차 감사 추가분 `checks/audit2_{nees_bias,control_20hz,gate_diag}.py`(출력 `*.out`)로, BT 의미론은 `checks/bt/mission_test.cpp`(BT.CPP 4.10 에서 컴파일·실행한 결함 주입 시험, §2.3)로 재현된다. 인터페이스 이름은 `docs/architecture/components.md` §5 와 `config/{sensors,robot_params,ekf}.yaml` 에 맞췄고, 다른 부분은 §8 에 components.md 갱신 요청으로 명시했다.

---

## 0. 요약 (TL;DR)

| 항목 | 결정 | 근거 |
| --- | --- | --- |
| BT 라이브러리 | **BehaviorTree.CPP 4.10 (ros-humble-behaviortree-cpp)** 로 자체 실행기(`task_executor_node`). Nav2 는 자기 프로세스에서 v3 사용 → 액션 인터페이스로만 연결 | 컨테이너에 v3 3.8.7 / v4 4.10.0 공존. `nav2_behavior_tree` 는 v3 링크 → v4 실행기 타깃은 `behaviortree_cpp` 만 링크. components.md 의 v3 플러그인 `IsTTCBelowThreshold`(bt_navigator 에 로드)는 **별도 공유 라이브러리 타깃**으로 분리(한 프로세스에 v3/v4 공존 금지) |
| 시각화 | Groot2 **편집기**로 XML 시각화(문서 스크린샷) + 실시간 모니터는 무료 티어 한도(20 노드) 안에서 서브트리 단위 + XML→Mermaid 자동 렌더 | behaviortree.dev/groot: Free "Monitor & Log Visualizer: 20 nodes", PRO Unlimited(€590/yr, 미사용) |
| 도킹 | 2단계: Nav2 `NavigateToPose`(staging 1.5 m) → 자체 정밀 서보(3-DoF EKF + Park–Kuipers 제어 + 결합 확률 게이트) | 단일 프레임 ArUco 는 카메라 프레임에서는 mm 급이지만 **도크 프레임 횡오차는 요각 지렛대 $(Z+l_c)\sigma_\theta$ 에 지배**(pre-dock 16 mm, 1 m 62 mm) → 다중 프레임 + LiDAR 융합 필수 (§3.2) |
| 독자 알고리즘 | **CGD: Covariance-Gated Docking** — 변환 사슬을 통과시킨 프레임별 PnP 공분산 + LiDAR 직선 적합 융합 + 종단 진입 전 결합(bivariate) 확률 게이트 + 사유 코드 재시도 | `opennav_docking`(staging 재시도 + 1차 저역필터 `filter_coef`) 와 Adámek 2023(적응 분산)·Dai & Lee 2025(LiDAR 헤딩 융합)의 **변형(variant)**. 우리 증분 = 폐형식 공분산 전파 + 결합 확률 게이트 + 사유별 재시도. 우주 랑데부의 chance-constrained MPC 계열과는 무관함을 명시(§3.6) |
| 스케줄링 | **위험조정 최소여유(LST) 디스패칭 + 기대 지연 비용 Hungarian + 구역별 주행시간 EWMA** | 고전 LST/MOD 디스패칭 규칙의 `engineering_adaptation`. σ→0, 우선순위 동일이면 **LST 와 정확히 동치**(EDF 가 아님) |

---

## 1. 명세 요구사항 → 정량 목표 매핑

| 명세 항목 | 목표 | 절 |
| --- | --- | --- |
| BT 노드 ≥15, 서브트리, 복구 ≥3, Groot | 자체 노드 24종 + 내장 14종(**XML 에 실제 등장하는 것만** 집계), 서브트리 8개(전부 XML 에 정의), 복구 5종 | §2 |
| '대기-이동-인식-작업-복귀' | WaitForTask → NavigateWithRecovery → DockWithRecovery(SearchMarker) → Load/Unload → **ReturnHome** | §2.3 |
| 도킹 위치 2 cm / 각도 1° | 종단 3σ 예상 1.0 cm / 0.63° + 추종오차 0.5 cm / 0.3° | §3.6 |
| 재시도 ≤3, 실패 시 보고+대체 작업 | **단일 카운터**(`RetryUntilSuccessful(3)`), 서버 내부 재시도 0 | §2.3, §3.6 |
| JSON Task, 상태 이벤트, 우선순위·마감 | 스키마·Task.msg 매핑·시각 도메인 명시 + LST 스케줄러 | §2.4, §4 |
| 단일 작업 성공률 ≥97 % | 300건 중 ≥291(점추정 기준) + Clopper–Pearson 하한 보고, 설계 목표 99 % | §5 E4 |
| 예측 주행시간 오차 ≤15 % (4.4) | 구간(leg) 단위 MAPE, n ≥ 200 | §5 E7 |
| 응답시간 ≤200 ms, 50회, 평균/최대 | 이벤트 구동 디스패치 + `[cmd_time, response_time, latency_ms]` 로그(주입 통신지연 포함 예산 평균 ≈177 ms) | §5 E8 |
| 통합 시나리오 ≥10 자동화, 4 h 연속 | IT-1…IT-10 (`launch_testing`), 야간 소크 | §5.2 |
| 5대 CPU ≤80 % | 본 영역 평상시 < 0.5 코어, 최악(5대 동시 도킹) ≈ 1.4 코어 | §6.5 |

---

## 2. Behavior Tree 설계

### 2.1 BehaviorTree.CPP v3 vs v4 결정 (Humble 문맥)

컨테이너 확인: `ros-humble-behaviortree-cpp-v3 3.8.7` 과 `ros-humble-behaviortree-cpp 4.10.0` 동시 설치. `nav2_behavior_tree 1.1.20` 은 `behaviortree_cpp_v3` 에 의존. v4 로거: `groot2_publisher.h`, `bt_sqlite_logger.h`, `bt_file_logger_v2.h`. v4 내장 controls 12종, decorators 14종(`ForceSuccess/ForceFailure/KeepRunningUntilFailure/RetryUntilSuccessful/Timeout/Delay/…`), 스크립트 전제조건 `_skipIf/_failureIf/_successIf/_while` (`tree_node.h` 확인).

| 기준 | v3 (Nav2 방식) | v4 (선택) |
| --- | --- | --- |
| Nav2 BT 플러그인 재사용 | 가능 | 불가(ABI) — `NavigateToPose`/`Spin`/`BackUp` 액션 클라이언트 3개만 직접 작성 |
| Groot | Groot1(미유지보수) | Groot2 편집기(무제한) + 모니터/로그(무료 20 노드, PRO €590/yr 미사용) |
| 서브트리 포트 리매핑, `_skipIf`, Script | 제한적 | 지원 → 단계 재개(§2.3)와 노드 수 절감 |

결정: **v4 단독 실행기**. 실행기 타깃(`task_executor_node`, components.md §3.5 의 이름 유지)은 `behaviortree_cpp` 만 링크하고 `nav2_behavior_tree`/`nav2_bt_navigator` 헤더를 include 하지 않는다(v3·v4 모두 `namespace BT` → 같은 프로세스에 로드되면 심볼 인터포지션 크래시). 단, components.md §3.3 의 `amr_behavior::IsTTCBelowThreshold` 는 **bt_navigator(v3) 에 로드되는 Nav2 플러그인**이므로 삭제 대상이 아니다 → `amr_behavior_nav2_bt_plugins` 라는 별도 공유 라이브러리 타깃으로 분리하고 그 타깃만 `nav2_behavior_tree`/`behaviortree_cpp_v3` 를 링크한다(`package.xml` 의 `nav2_behavior_tree` 의존은 이 타깃 때문에 **유지**; 금지 규칙은 "한 바이너리에 v3·v4 동시 링크 금지" 로 타깃 단위로 강제). Nav2 의 `RecoveryNode`, `RoundRobin` 은 v4 API 로 재구현(각 ≤40줄; `checks/bt/mission_test.cpp` 에 시험 구현 포함). components.md 의 "`nav2_behavior_tree::BtActionNode` 재사용" 문구는 이 결정과 충돌하므로 갱신 요청(§8).

**Groot 계획(명세 "시각화(Groot)")**: (i) 문서화 = Groot2 편집기로 전체 트리 렌더(편집기는 노드 제한 없음) + `scripts/bt_xml_to_mermaid.py` 자동 생성; (ii) 실시간 모니터 = 무료 티어 20 노드 한도 안에서 **서브트리 단독 실행 모드**(`main_tree_to_execute` 를 `DockWithRecovery`(9 노드) 등으로 바꿔 시연) 또는 `bt_file_logger_v2` 의 `.btlog` 를 20 노드 이하 서브트리로 재생; (iii) 전체 트리 런타임 추적은 우리 `bt_sqlite_logger` + 타임라인 스크립트. PRO 라이선스는 사용하지 않는다. `Groot2Publisher(tree, port)` 는 `port` 와 `port+1` 두 개를 바인드하므로(BT.CPP 4.10 `groot2_publisher.cpp` 확인) 한 호스트의 5대는 `groot_port = 1667 + 2·i` 로 분리한다(components.md 의 ":1666/1667" 은 Groot1 규약 → 갱신 요청). 이 해석이 "시각화(Groot)" 를 충족하는지 평가자 확인 항목(§8).

### 2.2 BT vs FSM (평가 질문 대비)

FSM 은 상태 N개·전이 O(N²) 로 복구 상태 추가 시 모든 상태에 전이가 필요하지만, BT 는 `Fallback` 한 층으로 복구가 붙고 tick 재평가로 반응성(E-Stop·배터리)이 상단 조건으로 표현된다. 단점인 장기 기억(어디까지 했는지)은 블랙보드 `task_phase` 와 `_skipIf` 로 명시 처리한다(§2.3).

### 2.3 트리 구조 (자체 노드 24종 + 내장 14종, 서브트리 8개) — BT.CPP 4.10 에서 로드·실행 검증

아래 XML 은 `checks/bt/mission.xml` 과 **동일**하며, 모든 자체 노드를 모의 구현으로 등록한 결함 주입 시험 `checks/bt/mission_test.cpp` 를 `amr-fleet-system:wf-final` 일회용 컨테이너에서 컴파일·실행해 6개 시나리오(A 정상, B 도킹 3회 실패, C 하역 주행 중 E-Stop, D 주행 영구 실패, E 주행 중 배터리 저하 선점, F 교통 hold)가 모두 통과함을 확인했다(`checks/bt/mission_test.out`).

```xml
<root BTCPP_format="4" main_tree_to_execute="MissionLoop">
 <BehaviorTree ID="MissionLoop">
  <KeepRunningUntilFailure>                  <!-- 자식 SUCCESS → reset 후 RUNNING (루프) -->
   <ForceSuccess>                            <!-- 한 사이클의 FAILURE 흡수 → 루트는 FAILURE 를 받지 않음 -->
    <ReactiveSequence>                       <!-- 자식 RUNNING 시 나머지 자식 halt -->
     <SafetyGate/>                           <!-- R5: E-Stop 활성 = RUNNING → 뒤 형제 전부 halt -->
     <ReactiveFallback>                      <!-- R4: 매 tick 배터리 재평가 → 작업 선점 -->
      <Sequence> <IsBatteryLow enter_pct="20" exit_pct="80"/> <SubTree ID="GoCharge" _autoremap="true"/> </Sequence>
      <SubTree ID="FetchAndDeliver" _autoremap="true"/>
     </ReactiveFallback>
    </ReactiveSequence>
   </ForceSuccess>
  </KeepRunningUntilFailure>
 </BehaviorTree>

 <BehaviorTree ID="FetchAndDeliver">         <!-- 대기-이동-인식-작업-이동-인식-작업-복귀 -->
  <Fallback>
   <Sequence>
    <WaitForTask task_id="{task_id}" _skipIf="task_phase > 0"/>
    <Sequence _skipIf="task_phase > 0">
     <AcceptTask task_id="{task_id}" pickup_staging="{pickup_staging}" pickup_dock="{pickup_dock}"
                 dropoff_staging="{dropoff_staging}" dropoff_dock="{dropoff_dock}" item_type="{item_type}"
                 item_mass="{item_mass}" load_time_ms="{load_time_ms}" budget_ms="{budget_ms}"/>
     <Script code="task_phase:=1"/>
    </Sequence>
    <Timeout msec="{budget_ms}" _skipIf="task_phase >= 8">
     <Sequence>
      <Sequence _skipIf="task_phase >= 2"> <SubTree ID="NavigateWithRecovery" goal="{pickup_staging}" fail_reason="{fail_reason}"/> <Script code="task_phase:=2"/> </Sequence>
      <Sequence _skipIf="task_phase >= 3"> <SubTree ID="DockWithRecovery" dock_id="{pickup_dock}" fail_reason="{fail_reason}"/> <Script code="task_phase:=3"/> </Sequence>
      <Sequence _skipIf="task_phase >= 4"> <SubTree ID="LoadPayload" item_type="{item_type}" load_time_ms="{load_time_ms}"/> <Script code="task_phase:=4"/> </Sequence>
      <Sequence _skipIf="task_phase >= 5"> <SubTree ID="NavigateWithRecovery" goal="{dropoff_staging}" fail_reason="{fail_reason}"/> <Script code="task_phase:=5"/> </Sequence>
      <Sequence _skipIf="task_phase >= 6"> <SubTree ID="DockWithRecovery" dock_id="{dropoff_dock}" fail_reason="{fail_reason}"/> <Script code="task_phase:=6"/> </Sequence>
      <Sequence _skipIf="task_phase >= 7"> <SubTree ID="UnloadPayload" load_time_ms="{load_time_ms}"/> <Script code="task_phase:=7"/> </Sequence>
      <PublishTaskEvent status="COMPLETED" reason="OK"/> <LogTaskRecord/> <Script code="task_phase:=8"/>
     </Sequence>
    </Timeout>
    <Fallback> <HasPendingTask/> <ForceSuccess><SubTree ID="ReturnHome" _autoremap="true"/></ForceSuccess> </Fallback>   <!-- 복귀 -->
    <Script code="task_phase:=0"/>
   </Sequence>
   <Sequence>                                <!-- 단일 실패 처리기: R1 소진 · R3 3회 소진 · Timeout · 거부 -->
    <PublishTaskEvent status="FAILED" reason="{fail_reason}"/> <LogTaskRecord/> <RequestReassignment/>
    <Script code="task_phase:=0"/>
    <ForceSuccess><SubTree ID="ReturnHome" _autoremap="true"/></ForceSuccess>          <!-- 대체 작업: 대기구역 복귀 -->
   </Sequence>
  </Fallback>
 </BehaviorTree>

 <BehaviorTree ID="NavigateWithRecovery">   <!-- R1 -->
  <RecoveryNode number_of_retries="4">
   <Sequence>
    <ForceSuccess><Sequence> <IsDeadlineAtRisk speed_limit="{lim}"/> <SetSpeedLimit limit="{lim}"/> </Sequence></ForceSuccess>
    <ReactiveSequence> <TrafficGate/> <NavigateTo goal="{goal}" eta="{eta}" result="{fail_reason}"/> </ReactiveSequence>
   </Sequence>
   <RoundRobin> <ClearCostmaps/> <BackUp dist="0.3"/> <Spin angle="1.57"/> <Delay delay_msec="3000"><AlwaysSuccess/></Delay> </RoundRobin>
  </RecoveryNode>
 </BehaviorTree>

 <BehaviorTree ID="DockWithRecovery">       <!-- R2, R3 -->
  <RetryUntilSuccessful num_attempts="3">   <!-- 유일한 도킹 재시도 카운터 (명세 ≤3) -->
   <Sequence>
    <Fallback> <IsMarkerVisible dock_id="{dock_id}"/> <SearchMarker dock_id="{dock_id}" budget_s="8" result="{fail_reason}"/> </Fallback>   <!-- R2 -->
    <Fallback>
     <DockToStation dock_id="{dock_id}" max_retries="0" result="{fail_reason}"/>   <!-- 접근 1회, 서버 내부 재시도 없음 -->
     <ForceFailure><BackToStaging dock_id="{dock_id}"/></ForceFailure>             <!-- R3: staging 복귀, 이 시도는 실패로 계수 -->
    </Fallback>
   </Sequence>
  </RetryUntilSuccessful>
 </BehaviorTree>

 <BehaviorTree ID="LoadPayload">   <Sequence> <VirtualLoad item_type="{item_type}"/> <Delay delay_msec="{load_time_ms}"><AlwaysSuccess/></Delay> </Sequence> </BehaviorTree>
 <BehaviorTree ID="UnloadPayload"> <Sequence> <VirtualUnload/> <Delay delay_msec="{load_time_ms}"><AlwaysSuccess/></Delay> </Sequence> </BehaviorTree>
 <BehaviorTree ID="ReturnHome">    <SubTree ID="NavigateWithRecovery" goal="{home_pose}" fail_reason="{fail_reason}"/> </BehaviorTree>
 <BehaviorTree ID="GoCharge">
  <SequenceWithMemory>
   <SubTree ID="NavigateWithRecovery" goal="{charger_staging}" fail_reason="{fail_reason}"/>
   <SubTree ID="DockWithRecovery" dock_id="{charger_dock}" fail_reason="{fail_reason}"/>
   <ChargeUntil level_pct="80"/>
  </SequenceWithMemory>
 </BehaviorTree>
</root>
```

동작 의미(헤더 + 컨테이너 실행으로 확인한 v4.10 규칙에 근거):
- **정상 루프**: 사이클이 SUCCESS/FAILURE 로 끝나면 `ForceSuccess` → `KeepRunningUntilFailure` 가 자식을 reset 하고 RUNNING → 다음 tick 에 `WaitForTask` 부터 재시작. 루트가 FAILURE 를 받는 경로는 없다(시나리오 A–F 모두 루트 RUNNING 유지).
- **R5 E-Stop**: `SafetyGate`(StatefulActionNode, `safety/estop_active` 구독) 는 E-Stop 활성 동안 RUNNING → `ReactiveSequence` 가 뒤 형제를 `halt()` 하고 `NavigateTo/DockToStation` 의 `onHalted()` 가 `cancel_goal` 을 보낸다(물리 정지는 `safety_node` 의 E-stop 래치가 보장, components.md §5.4). 4.10 의 `ReactiveSequence` 는 "다른 자식이 RUNNING 을 반환" 할 때 예외를 던지지 않음(기본값, 컨테이너 실행 확인). 해제 시 본체가 처음부터 tick 되지만 `task_phase` 전제조건이 완료 단계를 SKIPPED 로 건너뛰어 **중단된 단계만 재수행**한다(시나리오 C: 픽업 도킹·적재 1회만). E-Stop 으로 중단된 도킹은 `RetryUntilSuccessful` 카운터가 리셋된다 — E-Stop 은 도킹 실패가 아니므로 의도된 동작. `Timeout` 도 재개 시 다시 시작하므로 예산은 "중단 없는 구간" 기준임을 문서화.
- **단계 재개 규칙**: 각 단계는 `<Sequence _skipIf="task_phase >= k"> 서브트리 + Script(task_phase:=k) </Sequence>` 로 **묶는다**. 구 개정판처럼 `Script` 를 서브트리 옆에 평탄하게 두면 건너뛴 단계의 `Script` 가 여전히 실행되어 `task_phase` 를 **되돌린다**(컨테이너 시험: 5 에서 재개 → 3 으로 후퇴 → 픽업 도킹 재실행). 또한 `AcceptTask` 직후 `task_phase:=1` 이 없으면 픽업 주행 중 E-Stop 후 `WaitForTask` 가 다시 실행된다 — 둘 다 수정했다. 실행기는 기동 시 루트 블랙보드에 `task_phase=0`, `fail_reason="NONE"`, `home_pose`, `charger_staging`, `charger_dock`(behavior.yaml/docks.yaml) 을 넣는다.
- **단일 실패 처리기**: `FetchAndDeliver` 의 `Fallback` 두 번째 가지가 R1 소진(주행 불가)·R3 3회 소진·`Timeout`·`AcceptTask` 거부를 **모두** 받아 FAILED 이벤트 1회 + 재할당 요청 + 대기구역 복귀를 수행한다(시나리오 B, D). 구 개정판은 도킹 실패에만 FAILED 경로가 있어 주행 불가·Timeout 시 이벤트 없이 같은 작업을 무한 재개했다.
- **R4 배터리 선점**: `ReactiveFallback` 이 매 tick `IsBatteryLow`(20 % 진입, 80 % 해제 히스테리시스 래치)를 재평가 → 참이면 `GoCharge` 가 RUNNING 이 되며 `FetchAndDeliver` 를 halt, 충전 후 `task_phase` 로 재개(시나리오 E). 구 개정판의 일반 `Fallback` 은 실행 중인 작업을 선점하지 못했다. 시나리오 E 로그의 순서(`NAV:charger_staging` → `NAV_CANCEL:dropoff_staging`)처럼 `ReactiveFallback` 은 앞 자식이 새 goal 을 **보낸 뒤** 뒤 자식을 halt 하므로, `NavigateTo::onHalted()` 는 반드시 **자기 goal handle 만** 취소한다(`async_cancel_goal(handle)`; `cancel_all_goals` 금지 — 방금 보낸 충전 goal 이 취소됨). 같은 `navigate_to_pose` 서버에서 새 goal 은 기존 goal 을 선점(abort)하므로 뒤이은 개별 취소는 무해하다.
- **교통·위치 추정 일시정지**: components.md §5.5 의 `traffic/hold`(교통 관리자)와 `localization/lost`(납치 감시) 가 참인 동안 `TrafficGate` 가 RUNNING → `NavigateTo` halt(goal 취소) 후 대기, 해제 시 재전송(시나리오 F). `traffic/yield_pose` 이동은 교통 관리 브리프 소관.
- **`IsDeadlineAtRisk` 위치**: 조건 노드가 "위험 아님 = FAILURE" 를 반환해도 주행이 막히지 않도록 `ForceSuccess` 로 감쌌다(구 개정판은 이 FAILURE 가 `RecoveryNode` 복구를 촉발).
- **단일 재시도 카운터**: 접근 1회 = 서버 내부 `SEARCH→APPROACH→GATE→TERMINAL→VERIFY`(§3.6). 게이트의 정지 관측(≤2 s 누적)·제자리 회전(≤1회)은 **한 접근 안의 제어 동작**으로 접근 시간 예산 `docking.yaml approach_budget_s = 45` 에 묶이며, 예산 뒤에도 게이트 실패면 그 접근이 실패(카운터 +1)한다. 따라서 물리적 재접근 ≤3(시나리오 B: `DockToStation` 정확히 3회), 총 도킹 시간 ≤ 3×45 s.
- 포트는 모두 **평탄 블랙보드 키**(`{budget_ms}` 등). BT.CPP 4 에는 구조체 필드 접근(`{task.budget_ms}`)이 없으므로 `AcceptTask` 가 Task 를 키로 전개한다.

노드 목록 — **XML 에 등장하는 것만** 센다(자체 = C++ 클래스 1개/파일 1개, `BT_REGISTER_NODES`; 새 작업 추가 시 수정 파일 ≤3: 노드 .cpp, XML, 등록 목록):
- Action 18: WaitForTask, AcceptTask, SafetyGate, TrafficGate, NavigateTo(`nav2_msgs/NavigateToPose`), SearchMarker, DockToStation(`amr_msgs/Dock`), BackToStaging, VirtualLoad, VirtualUnload, PublishTaskEvent, LogTaskRecord, ClearCostmaps(srv), BackUp(`nav2_msgs/BackUp`), Spin(`nav2_msgs/Spin`), SetSpeedLimit(`nav2_msgs/SpeedLimit` 토픽 `speed_limit`), RequestReassignment, ChargeUntil.
- Condition 4: IsBatteryLow, HasPendingTask, IsMarkerVisible, IsDeadlineAtRisk.
- Control 2(자체): RecoveryNode, RoundRobin. → **자체 24종**. (구 목록의 IsNavStalled, IsDockResultOk, RateController 는 어느 트리에도 쓰이지 않아 삭제; ChargeUntil 은 누락돼 있어 추가.)
- 내장 14종: Sequence, SequenceWithMemory, ReactiveSequence, Fallback, ReactiveFallback, KeepRunningUntilFailure, ForceSuccess, ForceFailure, RetryUntilSuccessful, Timeout, Delay, Script, SubTree, AlwaysSuccess.
- 서브트리 8: MissionLoop(main), FetchAndDeliver, NavigateWithRecovery, DockWithRecovery, LoadPayload, UnloadPayload, ReturnHome, GoCharge.

복구(≥3): **R1** 주행 불가 → 클리어→후진→회전→대기 순환 4회 → 실패 시 FAILED+재할당. **R2** 마커 미검출 → `SearchMarker`(±20° 스캔·0.2 m 후진, 8 s 예산). **R3** 도킹 정밀도/관측 실패 → staging 복귀 후 재접근, 총 3회 → FAILED 이벤트 + 대체 작업(대기구역 복귀 + 재할당 요청). **R4** 배터리 → 선점 후 충전·재개. **R5** E-Stop → halt 후 단계 재개.

### 2.4 JSON Task Description ↔ `amr_msgs/Task.msg` 매핑과 시각 도메인

```json
{ "task_id": "T-20260921-0042", "priority": 180,
  "deadline": {"rel_s": 420.0},                         // 또는 "iso": "2026-09-21T10:15:00Z" (벽시계)
  "pickup":  {"dock_id": "RACK-A3"}, "dropoff": {"dock_id": "OUT-B"},
  "item": {"type": "medium", "mass_kg": 10.0},
  "constraints": {"max_retries": 3, "budget_s": 600} }
```
수신·검증 위치(components.md §3.6/§5.6 준수): JSON 은 **`/fleet/task_request`**(`std_msgs/String`)로 들어오고 중앙 **`fleet_manager_node`**(amr_fleet)가 JSON Schema 로 검증·파싱해 `amr_msgs/Task` 로 만든 뒤 할당하여 로봇의 `assign_task`(`amr_msgs/srv/AssignTask`)를 호출한다. 파싱·시각 변환·상태기계 로직은 `amr_fleet/amr_fleet/task_schema.py`, `task_state.py` 모듈로 제공한다(구 개정판의 로봇 측 `task_manager.py` + `/fleet/task_json` 은 components.md 와 중복되어 폐기).
- `pickup.dock_id` → `docks.yaml` 조회 → `pickup_pose` = 해당 도크의 **staging 자세**(도크 전면 중앙에서 바깥쪽으로 1.5 m, 즉 D 프레임 $x_r=-1.50$; map 프레임 `PoseStamped`). 마커 id·도크 자세는 `dock_id` 로 도킹 서버가 다시 조회하므로 JSON 에 `marker_id` 를 두지 않는다(SSOT = docks.yaml).
- **dock id 전달**: 현 `Task.msg` 에는 도크 id 필드가 없고 `Dock.action` goal 은 `dock_id` 를 요구한다. 따라서 `Task.msg` 확장 요청에 `string pickup_dock_id`, `string dropoff_dock_id` 를 포함한다(§8). 확장 전에는 `AcceptTask` 가 `pickup_pose`/`dropoff_pose` 를 docks.yaml 의 staging 자세와 대조(위치 ≤0.05 m, 방위 ≤5°)해 역조회하고, 일치가 없으면 작업을 거부(FAILED, reason `UNKNOWN_DOCK`)한다.
- `deadline`: **모든 내부 시각은 ROS 시뮬레이션 시각(`/clock`, `use_sim_time`)**. `rel_s` 는 수신 시 `now_sim + rel_s`; `iso` 는 수신 시 `now_sim + (t_iso − now_wall)` 로 변환해 `Task.deadline`(`builtin_interfaces/Time`, sim 시각)에 넣는다. 로그에는 두 도메인 모두 기록.
- `item.type` → `item_type`, `item.mass_kg` → `item_mass`; 적재 시간은 `robot_params.yaml payload.<type>.load_time`(5/10/15 s, 명세 표)에서 유도하므로 메시지 필드 불필요. `constraints` 는 `behavior.yaml` 기본값(3, 600)과 병합하되 `max_retries` 는 명세 상한 3 으로 클램프(현 XML 은 상한 3 고정).
- Task.msg 에 없는 `budget_s`·`max_retries` 는 로봇 측에 전달되지 않으므로 **`Task.msg` 확장 요청**(`string pickup_dock_id`, `string dropoff_dock_id`, `float32 budget_s`, `uint8 max_retries`; 빈 문자열/0 = 기본값, 하위호환) — amr_msgs 소유자 조율(§8). 확장 전에는 기본값 사용.

상태기계: `PENDING --AcceptTask--> IN_PROGRESS --COMPLETED`, `IN_PROGRESS --RecoveryExhausted|Timeout--> FAILED`, `PENDING --deadline<now--> FAILED(EXPIRED)`. 이벤트 경로(components.md §5.5–5.6): 로봇의 `PublishTaskEvent` 가 **`task_status`**(`amr_msgs/Task`, 전이마다)와 `executor/phase`(`idle/moving/docking/loading/charging/error`, `fleet_adapter_node` 가 `RobotState.status` 로 매핑)를 발행하고, `fleet_manager_node` 가 이를 모아 **`/fleet/task_events`**(`amr_msgs/Task`; QoS 는 transient_local depth 100 을 제안) 로 재발행한다. 실패 사유(`fail_reason`)는 `Task.msg` 에 필드가 없으므로 `/fleet/alerts`(`diagnostic_msgs/DiagnosticArray`, components.md) 의 message 와 JSONL 로 남긴다. JSONL `[stamp_sim, stamp_wall, task_id, robot_id, old, new, reason, odom_distance_m, elapsed_s]`.

---

## 3. ArUco 정밀 도킹

### 3.1 기하 (모든 높이는 바닥 기준)

센서(config/sensors.yaml): LiDAR `z=0.20`(base_link 기준), 주석 "스캔 평면 = 지면 +0.38 m" → **base_link 높이 0.18 m, 카메라 광축 0.43 m**(z=0.25). RGB 640×480, HFOV 87°:

$$f=\frac{320}{\tan 43.5^\circ}=337.2\ \text{px},\qquad \mathrm{VFOV}=2\arctan\frac{240}{337.2}=70.9^\circ$$

도크 계약(docks.yaml + SDF, 월드 담당과 합의 항목 §8): 전면판 폭 $L=0.8$ m, **바닥 0.20–0.65 m** 구간(LiDAR 평면 0.38 m 은 상단에서 0.27 m, 하단에서 0.18 m 여유); 마커(한 변 $s=0.15$ m, `DICT_4X4_50`) **중심 0.43 m = 카메라 광축 높이**, 판에 평면 텍스처로 인쇄. 도크 프레임 $D$: 원점 = 전면판 중앙(바닥 투영), **$x_D$ = 안쪽 법선(도킹 진행 방향)**, 도킹 완료 자세 $(x_r,y_r,\psi)=(-0.50,\,0,\,0)$ (범퍼–전면판 0.20 m). 카메라–마커 거리 $Z=-x_r-l_c$ ($l_c=0.18$): docked 0.32 m, pre-dock($x_r=-0.80$) 0.62 m, staging($x_r=-1.50$) 1.32 m. 화면상 변 길이 $\ell=fs/Z$: 158 / 81.6 / 38.3 px (≥24 px 검출 한계 OK). 시야 검사(Z=0.32): 가시 반폭 $0.32\tan43.5^\circ=0.304$ m, 반높이 $0.32\tan35.4^\circ=0.228$ m ≥ 마커 반변 0.075 m → 횡오차 0.22 m 까지 전체 가시.

### 3.2 프레임별 공분산: 카메라 프레임 → 도크 프레임 전파

코너 검출 노이즈 $\sigma_c$[px] (서브픽셀 정련 + Gazebo 가우시안 영상 노이즈 σ=0.007 → ≈0.2 px, 시뮬레이터 GT 로 보정). 마커 자세 $\mathbf p=(X,Y,Z,\theta,\cdot,\cdot)$ 에 대한 코너 투영 야코비안 $J_\pi\in\mathbb R^{8\times6}$ 로

$$\Sigma_{PnP}=\sigma_c^2\,(J_\pi^\top J_\pi)^{-1}\quad(\text{CRLB, LM 정련 PnP 의 점근 공분산})$$

정면 근처 폐형식(감도 점검용): $\sigma_\theta=\dfrac{2Z\sigma_c}{s\,\ell}$ (CRLB 와 일치), $\sigma_Z^{cf}=\dfrac{Z\sigma_c}{\ell}$ ($\sigma_\ell=\sigma_c$ 가정; 4변을 쓰는 CRLB 는 이의 0.71배), $\sigma_X\approx\dfrac{Z\sigma_c}{2f}$ ($\theta$ 를 안다고 둔 값; $X$–$\theta$ 결합 때문에 CRLB 주변분포는 이의 1.41배, 아래 표). 차원: $Z$[m]·px/(m·px) = 무차원 ✓.

**핵심 정정.** EKF 상태는 도크 프레임의 base_link 이며 $T_D^B=T_D^M\,(T_C^M)^{-1}(T_B^C)^{-1}$ 로 얻는다. 마커 자세를 뒤집을 때 지렛대 $(Z+l_c)$ 가 요각 오차에 곱해지므로, 1차 근사(정면)에서

$$y_r\approx X-(Z+l_c)\,\theta,\qquad \psi=\theta\quad\Rightarrow\quad \sigma_{y_r}^2\approx\sigma_X^2+(Z+l_c)^2\sigma_\theta^2-2(Z+l_c)\,\mathrm{cov}(X,\theta)$$

(부호는 §3.4 의 축 규약 기준. $\sigma_{y_r}$ 는 $\mathrm{var}(X-L\theta)=\mathrm{var}(-X+L\theta)$ 이므로 규약과 무관하고, 상관계수의 부호만 규약에 따른다 — §3.4 규약에서는 $\rho(y_r,\psi)\approx-1$.) 즉 도크 프레임 횡오차는 카메라 프레임 횡오차가 아니라 **요각 지렛대 항에 지배**되고 $y_r$–$\psi$ 는 거의 완전 상관($|\rho|\approx1$)이다. 수치($\sigma_c=0.2$ px, $s=0.15$ m; `rev_calc.py` 400회 MC 16.7 mm, 독립 재유도 `checks/audit_checks.py` 는 §3.4 의 $h_c$ 를 정확히 역변환해 선형 16.40 mm / LM-PnP 600회 MC 16.6 mm, $\rho=-1.000$):

| $Z$ | $\ell$ | $\sigma_Z$ (CRLB) | $\sigma_X$ | $\sigma_\theta$ | $\lvert \mathrm{corr}(X,\theta)\rvert $ | **$\sigma_{y_r}$ 도크 프레임** | $\lvert \mathrm{corr}(y_r,\psi)\rvert $ |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1.32 m (staging) | 38 px | 4.9 mm | 0.55 mm | 5.26° | 0.71 | **138 mm** | 1.00 |
| 1.00 | 51 | 2.8 | 0.42 | 3.02° | 0.71 | **62 mm** | 1.00 |
| 0.62 (pre-dock) | 82 | 1.1 | 0.26 | 1.16° | 0.71 | **16.4 mm** | 1.00 |
| 0.32 (docked) | 158 | 0.29 | 0.13 | 0.31° | 0.71 | **2.8 mm** | 1.00 |

결론: 단일 프레임으로 위치가 mm 급인 것은 **카메라 프레임에서만** 참이다. 게이트가 검사하는 도크 프레임 횡오차는 pre-dock 에서 16 mm(1σ)로 목표 15 mm 를 단일 프레임으로는 만족하지 못하며, 요각 분산이 $Z^4$ 로 줄어드는 것을 이용해 접근하며 다중 프레임을 누적하고 **요각을 독립 관측하는 LiDAR** 를 융합해야 한다. LiDAR 요각 갱신은 상관 항을 통해 $y_r$ 도 함께 조인다. 이 스케일링은 Adámek 2023 의 경험식(회전분산 ∝ $S_n^{-p}$)과 일치한다. Oh & Kim 2025 의 SolvePnP 6.6° vs 회귀 3.1° 는 참고로만 인용한다(같은 논문의 SolvePnP 거리오차 58.5 cm 는 보정 불량을 시사하므로 "단일 카메라 요각이 병목" 의 강한 근거로 쓰지 않음).

평면 자세 모호성(IPPE 두 해): $|\theta|\lesssim\sigma_\theta$ 이면 두 해의 재투영오차가 비슷해져 요각 부호가 뒤집힐 수 있다($2\theta$ 점프). §3.4 로 처리.

대안 A(선택, **원거리 보조로만**): 마커 2개(변 0.12 m) 보드는 기준선 $b$ 로 요각 $\sigma_\theta\approx\sqrt2\,\sigma_Z/b$. 구 설계의 $b=0.5$ m 는 docked 거리에서 바깥 가장자리 0.31 m > 가시 반폭 0.304 m 라 종단에서 잘린다. $b\le0.40$ m 로 줄이면(가장자리 0.26 m, 횡 여유 0.044 m) 종단에서도 보이지만 여유가 작으므로, 두 마커 요각은 $Z>0.45$ m 에서만 쓰고 종단은 중앙 단일 마커로 한다. $b=0.40$ m 에서의 이득(`checks/audit_checks.py` §11): Z=1 m 단일 프레임 요각 σ 0.96°(CRLB $\sigma_Z$)–1.00°(폐형식) vs 단일 0.15 m 마커 3.02° → 약 3배(구 설계 $b=0.5$ 의 3.7배보다 작음), Z=0.45 m 에서 0.19° vs 0.61°. 기본안은 단일 0.15 m 마커.

### 3.3 LiDAR 직선 적합에 의한 독립 요각 관측

LiDAR(base_link 앞 $l_x=0.15$ m, 0.5°, $\sigma_r=0.03$ m)가 전면판(폭 $L=0.8$ m)을 본다. 판까지 수직거리 $R=-(x_r+l_x\cos\psi)$, 빔 수 $N=2\arctan(L/2R)/0.5^\circ$, 최소제곱 직선의 기울기·거리 분산:

$$\sigma_\alpha\approx\frac{\sigma_r}{L}\sqrt{\frac{12}{N}},\qquad\sigma_d\approx\frac{\sigma_r}{\sqrt N}$$

| 위치 | $R$ | $N$ | $\sigma_\alpha$/스캔 | $\sigma_d$/스캔 |
| --- | --- | --- | --- | --- |
| staging | 1.35 m | 66 | 0.92° | 3.7 mm |
| pre-dock | 0.65 m | 126 | 0.66° | 2.7 mm |
| docked | 0.35 m | 195 | 0.53° | 2.1 mm |

위 식은 점이 판 위에 균등하다고 둔 근사다. 실제 빔은 각도 균등(중앙이 촘촘)하고 잡음은 빔 방향이므로, 정확한 기하로 MC 한 값은 $\sigma_\alpha$ = 0.88°/0.63°/0.48°, $\sigma_d$ = 3.7/2.5/1.9 mm 로 표가 5–10 % **보수적**이다(`checks/audit_checks.py`). 입력은 components.md 규약대로 `scan_filtered`(`scan_filter_node` 출력)를 쓰되, 판 가장자리 빔이 아웃라이어 필터에 깎이지 않는지 IT-2 에서 확인한다.

검출: 예상 도크 자세 주변 ±0.4 m 창에서 RANSAC(50회) → 인라이어 직선 법선 = $-x_D$. 카메라(요각 약함·횡 상관)와 LiDAR(요각 안정, 횡은 판 가장자리 의존 → 미사용)가 상보적이다.

### 3.4 상대자세 EKF (도크 프레임 $D$, 안쪽 법선 규약)

상태 $\mathbf x=[x_r,y_r,\psi]^\top$ (base_link in $D$, 도킹 완료 $=(-0.50,0,0)$, $e_\psi=\psi$). 예측(오도메트리 $v,\omega$):

$$\mathbf x_{k+1}=\mathbf x_k+\Delta t[v\cos\psi,\ v\sin\psi,\ \omega]^\top,\quad F_k=\begin{bmatrix}1&0&-v\Delta t\sin\psi\\0&1&v\Delta t\cos\psi\\0&0&1\end{bmatrix},\quad Q_k=B_k\mathrm{diag}(\sigma_v^2,\sigma_\omega^2)B_k^\top,\ B_k=\Delta t\begin{bmatrix}\cos\psi&0\\\sin\psi&0\\0&1\end{bmatrix}$$

오도메트리 입력은 components.md §5.2 의 **`odometry/filtered`**(`ekf_filter_node_odom`, 50 Hz; `wheel_odom` 속도 + IMU `vyaw` 융합, `config/ekf.yaml`)이다. $\sigma_v,\sigma_\omega$ 는 그 메시지의 twist 공분산을 쓰되 하한을 `sensors.yaml` 에서 둔다: $\sigma_v\ge0.01|v|$(`wheel_encoder.slip_noise_stddev`), $\sigma_\omega\ge2\times10^{-4}$ rad/s(`imu.gyro_noise_stddev`; 바이어스 0.01 rad/s 는 `imu_filter_node` 정지 보정 후 잔차 $\approx6\times10^{-6}$ rad/s). 종단 0.30 m(0.1 m/s, 3 s) 동안 자이로 각 랜덤워크는 $2\times10^{-4}\sqrt{0.01\cdot3}=3.5\times10^{-5}$ rad 로 무시 가능하다.

**관측 1 (카메라, 30 Hz)**: 입력은 `amr_perception/aruco_detector_node`(components.md §3.4, Python) 가 추가 발행하는 `perception/dock_marker_observation`(제안 메시지 `amr_msgs/MarkerObservation`: 두 IPPE 해의 $(X,Z,\theta)$·재투영오차·코너 4점·$\ell$·3×3 공분산, 카메라 optical 프레임; §8) — 기존 `perception/dock_marker_pose`(PoseStamped, 공분산 없음)는 호환용으로 유지. 관측 벡터는 카메라 프레임의 마커 자세 $\mathbf z_c=[X,Z,\theta]^\top$ (광축 $z$ = 로봇 진행 방향, $x$ = 우측; $\mathrm{Rot}(\psi)$ 는 이 카메라 축을 $D$ 로 돌리는 회전). 비선형 관측 모델은 $T_C^M(\mathbf x)=(T_B^C)^{-1}(T_D^B(\mathbf x))^{-1}T_D^M$ 의 평면 성분이며(마커 장착 오프셋 $T_D^M$ 과 외부 파라미터 $T_B^C$ 는 우리가 SDF/URDF 를 작성하므로 정확히 알려짐), 마커 원점이 $D$ 의 $(0,0)$, 카메라가 $(x_r+l_c\cos\psi,\ y_r+l_c\sin\psi)$ 에 있으므로

$$h_c(\mathbf x)=\begin{bmatrix}X\\Z\\\theta\end{bmatrix}=\begin{bmatrix}-(x_r+l_c\cos\psi)\sin\psi+(y_r+l_c\sin\psi)\cos\psi\\-(x_r+l_c\cos\psi)\cos\psi-(y_r+l_c\sin\psi)\sin\psi\\ \psi\end{bmatrix},\qquad H_c=\frac{\partial h_c}{\partial\mathbf x}\ (\text{해석적 또는 수치 미분},\ \varepsilon=10^{-6})$$

(카메라 우측축 $x_{cam}=(\sin\psi,-\cos\psi)$, 광축 $z_{cam}=(\cos\psi,\sin\psi)$ 를 $D$ 에서 표현하고 마커 상대 위치 $-\mathbf c$ 를 투영한 것.) 검산: 도킹 완료 $(-0.50,0,0)$ → $X=0$, $Z=0.50-0.18=0.32$ m ✓; 좌회전($\psi>0$, $y_r=0$) 시 마커가 광축 우측($X>0$)에 보임 ✓; 정면 근처 1차 근사 $X\approx y_r+(Z+l_c)\psi$ 로 §3.2 의 지렛대 식 $y_r\approx X-(Z+l_c)\theta$ 와 일치 ✓. $R_c=\Sigma_{PnP}$ 의 $(X,Z,\theta)$ 3×3 부분행렬(비대각 $\mathrm{cov}(X,\theta)$ **유지**), 측정된 $\ell$ 로 매 프레임 $J_\pi$ 를 재계산(적응 관측잡음). 부풀림 $\kappa\in[1,2]$ 는 GT 대비 NEES 로 보정하되, 구 설계처럼 30–350배 과소평가를 흡수하는 용도가 아니라 미모델 편향(모션 블러·렌즈)만 보정한다.

**공통 편향 상태 $\beta$ (E2 일관성, 3차 감사 추가)**: 융합 사후분포는 카메라가 정밀하게 묶는 $y_r+(Z+l_c)\psi$ 방향으로 매우 얇다(pre-dock 1 s 정지 후 $\sigma(y_r+0.80\,\psi)\approx0.04$ mm, $\sigma_y$ 2 mm 의 1/50). 프레임 간 독립이 아닌 코너 공통 편향(렌더링·서브픽셀 정련의 계통오차) $\beta$ 가 0.05 px 만 있어도 이 방향 오차가 $Z\beta/f\approx0.09$ mm 로 고정되어 3-DoF NEES 가 +7.5 커지고 E2 대역 [2.52, 3.53] 을 벗어난다. 스칼라 κ 는 프레임 수 $N$ 으로 줄지 않는 편향을 흡수하지 못한다. 따라서 상태를 $[x_r,y_r,\psi,\beta]$ 로 **증강**한다: $h_c$ 의 $X$ 성분에 $+Z\beta$ ($\partial X/\partial\beta=Z$), $\beta$[rad] 는 접근 내 상수($Q_\beta=0$), $P_{0,\beta}=(\sigma_\beta/f)^2$, $\sigma_\beta=0.05$ px(W2 에서 GT 정지 프레임의 코너 잔차 평균으로 보정). 이 방향의 σ 는 0.04 → 0.10 mm 로 현실화되어 같은 편향의 NEES 증분은 0.88 로 줄고, $\sigma_y$·$\sigma_\psi$ 는 사실상 불변이다(부풀림 전 2.05 mm·0.145° 그대로; `checks/audit2_nees_bias.py`). 게이트와 E2 는 앞 3성분의 주변분포를 쓴다.

**관측 2 (LiDAR, 10 Hz)**: $h_L(\mathbf x)=[-(x_r+l_x\cos\psi),\ \psi]^\top$, $H_L=\begin{bmatrix}-1&0&l_x\sin\psi\\0&0&1\end{bmatrix}$, $R_L=\mathrm{diag}(\sigma_d^2,\sigma_\alpha^2)$.

**초기화(staging 도착 시)**: 사전분포가 없으므로 첫 유효 프레임에서 두 IPPE 해 중 **LiDAR 요각과 가까운 해**로 $\hat{\mathbf x}_0$ 를 두고 $P_0=4\,J\Sigma_{PnP}J^\top$(변환 사슬 야코비안 $J$) + LiDAR 항으로 초기화한다(LiDAR 가 부호 모호성을 깨므로 마할라노비스 검정 없이도 첫 프레임이 안전). 이후 프레임: `cv::solvePnPGeneric(SOLVEPNP_IPPE_SQUARE)` 의 두 해에 대해 재투영오차 비 <1.5 이면 $d_i^2=\nu_i^\top S^{-1}\nu_i$ 가 작은 해 채택, 둘 다 $\chi^2_{3,0.99}=11.3$ 초과면 기각.

### 3.5 제어 법칙 (Park–Kuipers; Nav2 `smooth_control_law.cpp` humble 브랜치 확인)

목표 자세를 로봇 중심 극좌표 $(r,\phi,\delta)$ 로 두고

$$\kappa=-\frac1r\Big[k_\delta\big(\delta-\arctan(-k_\phi\phi)\big)+\Big(1+\frac{k_\phi}{1+(k_\phi\phi)^2}\Big)\sin\delta\Big]$$
$$v=\mathrm{clamp}\Big(\min\Big(\frac{v_{max}}{1+\beta|\kappa|^\lambda},\ v_{max}\frac{r}{r_{slow}},\ \underbrace{\sqrt{2\,r\,a_{dock}}}_{\text{우리 추가}}\Big),\ v_{min},\ v_{max}\Big),\qquad\omega=\mathrm{clamp}(\kappa v,\pm\omega_{max}),\qquad v\leftarrow\omega/\kappa\ (\kappa\ne0)$$

Nav2 원본(humble `calculateRegularVelocity`, 소스 재확인)은 처음 두 항의 min 뒤 $[v_{min},v_{max}]$ 클램프, 이어서 $\omega$ 포화 시 곡률을 지키도록 $v=\omega_{bound}/\kappa$ 로 **재계산**한다(이때 $v<v_{min}$ 가능). 감속 상한 $\sqrt{2ra_{dock}}$ 은 우리가 추가한 항이다. $\kappa$[1/m], $\beta$[m$^\lambda$], $v_{min}=0.05$ m/s(정지 방지, Nav2 기본 0.1). 파라미터: $k_\phi=2,\ k_\delta=1,\ \beta=0.4,\ \lambda=2,\ v_{max}=0.25$ m/s, $r_{slow}=0.25$ m, $\omega_{max}=0.75$ rad/s, **도킹 감속 상한 $a_{dock}=0.5\,\mathrm{m/s^2}\cdot m_0/(m_0+m_{load})$**, $m_0=47.6$ kg(§3.7; 25 kg 적재 시 0.33 m/s²). `TERMINAL` 은 $v_{max}=0.10$ m/s 로 낮춘다(pre-dock 에서 풋프린트 전면–판 여유가 0.50 m = `safety_node` Critical 경계이므로 종단 전 구간이 Critical 존(≤0.2 m/s, `robot_params.yaml safety.critical_zone_max_speed`) 안이다; 제동거리 $0.1\cdot0.15+0.1^2/2=0.02$ m). 출력은 components.md §4.1 체인 `cmd_vel_nav → velocity_profiler_node → cmd_vel_smoothed → safety_node → cmd_vel` 을 그대로 탄다(프로파일러의 가가속 제한 지연은 EKF 자세 피드백 루프 안에 있으므로 정밀도가 아니라 수렴 시간에만 영향). 스테이징 오프셋(±0.15 m, ±10°) 9조합을 이 법칙(ω 재계산 포함)으로 50 Hz 적분하면 모두 $r<5$ mm, $|\psi|\le0.36^\circ$ 로 수렴하고, components.md §5.5 의 `cmd_vel_nav` **20 Hz** 출력(50 ms 영차 유지)으로 이산화해도 $r<5$ mm, $|\psi|\le0.41^\circ$ 이며 두 경우 모두 ω 포화는 없었다(`checks/audit_checks.py` §10, `checks/audit2_control_20hz.py`). 이 잔여 헤딩은 게이트의 ψ 주변확률을 0.82 까지 낮출 수 있으므로 §3.6 의 "방위 편차" 행(제자리 회전 1회)이 흡수한다. 종단 0.30 m 는 폐루프라 pre-dock 에서 $\psi_0=0.6^\circ$, $y_0=5$ mm 여도 docked 에서 $|\psi|\approx0.06^\circ$ 로 수렴한다(같은 스크립트) — 게이트의 직진 전파는 보수적이다. `nav2_graceful_controller` 는 Humble 에 배포되어 있으나(rosdistro navigation2 1.1.20-1 패키지 목록) 이미지에 미설치이며, 명세의 "직접 구현·설명" 취지로 우리는 위 식을 직접 구현하고 B0 기준선은 apt 패키지를 쓴다. §3.1 에서 보인 대로 docked 거리(Z=0.32 m)에서도 마커는 전부 시야 안이므로 종단 끝까지 카메라 관측이 유지된다. 검출 누락 프레임은 EKF 예측으로 메운다(슬립이 완전 상관이라는 보수 가정에서 0.15 m 당 1.5 mm; 실제 주기별 독립 슬립 모델에서는 $0.01\cdot2\,\mathrm{mm}\cdot\sqrt{75}\approx0.2$ mm).

### 3.6 독자 알고리즘: CGD (Covariance-Gated Docking)

**접근 1회의 구조**(서버 `docking_server_node` 내부 상태): `SEARCH`(마커 미검출 시 ≤8 s) → `APPROACH`(staging→pre-dock, EKF+제어) → `GATE` → `TERMINAL`(pre-dock→docked 0.30 m, $v\le0.10$ m/s) → `VERIFY`(components.md §5.5 판정: 위치 ≤0.02 m, 각도 ≤1°, 0.5 s 유지). 액션 피드백 `current_phase` 는 components.md 의 문자열을 쓴다: `search`=SEARCH, `approach`=APPROACH, `align`=GATE(정지 관측·제자리 회전), `final`=TERMINAL+VERIFY.

**게이트**: pre-dock 에서 사후분포 $\mathcal N(\boldsymbol\mu,P)$ 를 종단 0.30 m 직진으로 전파한 $\boldsymbol\mu_t=\Phi\boldsymbol\mu,\ P_t=\Phi P\Phi^\top+Q_t$ ($\Phi=\prod_kF_k\approx\begin{bmatrix}1&0&0\\0&1&0.30\\0&0&1\end{bmatrix}$, 즉 EKF 예측 식 그대로; $Q_t$ = 종단 오도메트리 드리프트) 의 $(y_r,\psi)$ 2×2 주변분포로 **결합 확률**을 계산한다(구 개정판의 $P_t=P+Q_t$ 는 $\partial y/\partial\psi=0.30$ 항을 빠뜨렸다):

$$\Pr\big(|e_y|\le\tau_y\ \wedge\ |e_\psi|\le\tau_\psi\big)=\int_{[-\tau_y,\tau_y]\times[-\tau_\psi,\tau_\psi]}\mathcal N(\mathbf e;\boldsymbol\mu_{y\psi},P_{t,y\psi})\,d\mathbf e\ \ge 1-\delta$$

($\tau_y=0.015$ m, $\tau_\psi=0.6^\circ$, $\delta=0.05$). 상관이 있으므로 주변확률의 곱은 결합확률이 아니다(예: $\sigma_y$=8 mm, $\sigma_\psi$=0.3°, ρ=0.7 에서 결합 0.911 vs 곱 0.897, Bonferroni 0.894; 이변량 CDF 로 재확인). 계산은 $10^4$ 표본 몬테카를로(µs 급) 또는 이변량 정규 CDF; 보수적 대안 Bonferroni $1-\Pr(|e_y|>\tau_y)-\Pr(|e_\psi|>\tau_\psi)$ 를 단위 테스트의 하한으로 쓴다.

게이트 실패 시 **사유 코드**(맹목 재시도 대신; 모두 한 접근의 시간 예산 안):

| 진단 | 조건 (게이트 실패 시 위에서부터 첫 일치; 주변확률은 $\Phi$ 전파한 $P_t$ 기준) | 행동 | 접근 실패? |
| --- | --- | --- | --- |
| 정보 부족 | $\sigma_\psi>\tau_\psi/2$ | pre-dock 정지, 누적 관측 총 ≤2 s | 예산 뒤에도 실패면 예 |
| 방위 편차 | $\Pr(\lvert e_\psi\rvert \le\tau_\psi)<1-\delta$ (σ 가 위 조건을 통과한 상태에서는 $\lvert \mu_\psi\rvert \gtrsim\tau_\psi-1.64\sigma_\psi\approx0.26^\circ$) | 제자리 회전 $\hat\psi\to0$ 1회 후 재관측 | 재관측 후 실패면 예 |
| 횡 편차 | $\Pr(\lvert e_y\rvert \le\tau_y)<1-\delta$ | 접근 종료 → `BackToStaging` → 재접근 | 예 |
| 상관 잔여 | 두 주변확률은 통과하나 결합확률 미달(드묾) | "정보 부족" 과 동일 | 예산 뒤에도 실패면 예 |
| 관측 상실 | 카메라·LiDAR χ² 기각 3연속(어느 단계든) | 접근 종료 → staging → `SearchMarker` | 예 |

(3차 감사 정정: 구 표의 조건 "$|\mu_\psi|>\tau_\psi$", "$|\mu_y|>\tau_y$" 로는 $0.26^\circ<|\mu_\psi|\le0.6^\circ$ 에서 게이트는 실패하는데 어느 행에도 해당하지 않았다 — 결합확률 μψ=0.3°: 0.93, 0.41°: 0.82, `checks/audit2_gate_diag.py`. 아래 게이트 예의 "0.3° → 방위 편차" 와도 모순이었고, §3.5 의 20 Hz APPROACH 잔여 헤딩 ≤0.41° 가 바로 이 구간이다.)

접근 실패는 `Dock.action` result `success=false`, `final_*_error`, `attempts_used=1` 로 반환되고 BT 의 `RetryUntilSuccessful(3)` 만이 횟수를 센다(§2.3).

**예상 성능(보수적)**: pre-dock 에서 1 s 정지 관측(카메라 30 프레임 + LiDAR 10 스캔, 시간 상관을 위해 분산 2배 부풀림)만으로 $\sigma_\psi\approx0.20^\circ$, $\sigma_y\approx(0.80\ \mathrm m)\cdot\sigma_\psi\approx2.9$ mm($\rho\approx-1$; 독립 재유도 0.204°/2.88 mm). 접근 중 누적분은 덤이다. 종단 0.30 m 전파: 상관을 무시하고 $0.30\,\sigma_\psi=1.1$ mm 를 제곱합하면 1σ ≈ 3.1 mm / 0.21°(상한), $\Phi P\Phi^\top$ 로 상관을 살리면 $|{-0.80}+0.30|\sigma_\psi\approx1.8$ mm — 카메라가 $y_r+(Z+l_c)\psi$ 를 정밀하게 묶기 때문. 보수적 상한으로 **3σ ≈ 1.0 cm / 0.63°**, 추종오차(0.1 m/s, 헤딩 보정) <5 mm/0.3° 를 선형 합산해도 1.5 cm/0.93° 로 2 cm/1° 이내. 게이트 예($\Phi$ 전파, 이변량 CDF): $\mu=0$ 이면 $\Pr\approx\Pr(|e_\psi|\le0.6^\circ)=0.997$ 통과; $\mu_\psi=0.3^\circ$ 면 0.93 → "방위 편차" 로 회전. 두 경우 모두 $\psi$ 제약이 지배한다($0.5\,\mathrm m\cdot\tau_\psi=5.2$ mm $<\tau_y$).

**문헌 대비 위치(정직한 표기)**: `opennav_docking`(humble README 확인) 은 이미 "N retries … driving back to the dock's staging pose" 와 검출 자세 1차 저역필터(`filter_coef` 0.1)를 갖고, Adámek 2023/Hinderer 2025 는 적응 관측분산 KF 를, Dai & Lee 2025 는 LiDAR 헤딩+태그 융합을 한다. 따라서 CGD 는 **`opennav_docking` + 적응분산 EKF 의 변형(variant)** 이며 우리 증분은 (i) 변환 사슬을 통과한 프레임별 PnP 공분산(상관 항 포함, 고정 필터 계수 대체), (ii) 종단 진입 전 결합 확률 go/no-go, (iii) 사유 코드 재시도이다. "Chance-constrained docking" 이라는 구 명칭은 우주 랑데부의 chance-constrained MPC/컨벡스화 계열(Berning 2019, Sanchez 2025, D'Onofrio & Zanetti 2026)과 충돌하므로 폐기했다 — 그 계열은 궤적 최적화 안의 확률 제약이고 우리는 EKF 사후분포에 대한 단발 검사다.

### 3.7 가상 적재/하역과 질량 변화

인터페이스는 components.md §3.1/§5.1 의 `payload_manager_node` 를 그대로 쓴다: `VirtualLoad` 는 `payload/attach`(`std_msgs/String`, `"small"/"medium"/"large"`), `VirtualUnload` 는 같은 토픽에 `""` 를 발행하고, `payload_manager_node` 가 박스 스폰 + Gazebo Fortress 6.18 `DetachableJoint` 부착(물리 질량이 실제로 바뀜)과 latched `payload/mass`(`std_msgs/Float32`) 발행을 맡는다(구 개정판의 BT 직접 `/world/<w>/set_pose` + gz `attach` 호출은 이 노드와 중복되어 폐기). 적재 시간 5/10/15 s(`robot_params.yaml payload.<type>.load_time`)는 `Delay`. 질량: `robot_params.yaml` 의 `robot.base_mass` = 45 kg(차체) + 바퀴 2×1.0 + 캐스터 2×0.3 → 공차 $m_0=47.6$ kg(같은 파일 `base_mass` 주석의 "총 공차" 값; `payload` 블록 주석의 "총 질량 = base_mass + payload" 는 바퀴·캐스터 2.6 kg 을 뺀 근사로, 가감속 스케일 차이는 25 kg 에서 0.656 vs 0.643 m/s², 2 % 미만 — `checks/audit_checks.py` §7. 구현은 한 곳(`robot_params.yaml`)에서 $m_0$ 를 계산해 프로파일러·DWA·도킹 서버가 공유), 적재 $m_{load}\in\{2,10,25\}$ kg(`payload.*.mass`). 가감속 한계 $a_{max}(m)=a_{nom}\,m_0/(m_0+m_{load})$ 는 이미 `payload/mass` 를 구독하는 `velocity_profiler_node`(피드포워드)·`controller_server`(DWA 가속 한계)가 적용하고(components.md §5.3), 도킹 서버도 같은 토픽을 구독해 **도킹 감속 상한에 동일 스케일**을 적용한다($a_{nom}$ = 주행 1.0(`limits.max_linear_acceleration`), 도킹 0.5 m/s²): 2/10/25 kg → 주행 0.96/0.83/0.66, 도킹 0.48/0.41/0.33 m/s².

### 3.8 도킹 모드 안전 설계 (명세 4.7 과의 충돌 해소)

`robot_params.yaml safety` 의 거리는 **풋프린트 모서리 기준**이다. 도킹 경로의 판까지 여유는 staging 1.20 m(존 밖) → pre-dock 0.50 m(Critical 경계, ≤0.2 m/s) → docked **0.20 m < 긴급정지 0.30 m** 이므로, 예외 없이는 종단 마지막 0.10 m 에서 `safety_node` 가 정지·래치한다. 안전 노드 소유자는 components.md §3.4 의 **`safety_node`(amr_perception, 자체 C++, `cmd_vel` 유일 발행자)** 이며 `nav2_collision_monitor` 는 쓰지 않는다(components.md 가 자체 노드로 대체). 제안(합의 항목 §8):
- `docking_server_node` 가 APPROACH/ALIGN/TERMINAL 동안 **`safety/dock_exclusion`**(`geometry_msgs/PolygonStamped`, frame `<r>/base_link`, 10 Hz)을 발행한다: EKF 가 주는 판 직선 ±0.10 m 두께, 판 폭 0.8 m 의 사각형. 0.3 s 무수신이면 `safety_node` 는 즉시 NORMAL 로 복귀(모드 문자열 토픽 대신 형상을 보내는 이유: 안전 노드가 "어느 빔을 판으로 볼지" 알아야 하므로).
- `safety_node` 는 폴리곤 **안의** 스캔 점에 대해서만 정지거리를 0.10 m 로 줄이고(docked 0.20 m 에서 여유 0.10 m; 판과 범퍼 사이에 끼어든 물체는 여전히 정지), 폴리곤 **밖**은 Warning 1.0 / Critical 0.5 / 정지 0.3 m 를 그대로 적용한다. 속도 상한은 Critical 존 상한 0.2 m/s 를 유지하며 TERMINAL 은 0.10 m/s 로 설계(제동거리 0.02 m).
- `RobotState.status=STATUS_DOCKING` 은 도킹 서버가 아니라 `fleet_adapter_node` 가 `executor/phase="docking"` 에서 매핑한다(components.md §5.6). E-Stop(`estop`, `/fleet/estop`) 은 모드와 무관하게 `safety_node` 의 래치로 즉시 0 속도.

---

## 4. 우선순위·마감 기반 스케줄링

### 4.1 문제 정의

작업 $j$: 출시 $r_j$, 마감 $d_j$(sim 시각), 우선순위 $p_j\in[0,255]$, 픽업 $P_j$, 하역 $Q_j$, 서비스 $s_j^L,s_j^U$(5/10/15 s) + 도킹 $t_{dock}\approx20$ s. 로봇 $i$: 위치 $x_i$, 현 작업 잔여 $\rho_i$:

$$\hat C_{ij}=t+\rho_i+\hat\tau(x_i\!\to\!P_j)+s_j^L+2t_{dock}+\hat\tau(P_j\!\to\!Q_j)+s_j^U,\qquad\lambda_{ij}=d_j-\hat C_{ij}\ [\mathrm s]$$

### 4.2 주행시간 예측기와 온라인 보정 (명세 4.4: 오차 ≤15 %)

구역 $e$(통로/교차로/도크 접근) 분할: $\hat\tau=\sum_e\ell_e/\hat v_e+n_{turn}t_{turn}$. 스테이션 간 경로길이 표는 오프라인(~30 스테이션 → 900 경로), 로봇 현위치→다음 스테이션만 온라인 `ComputePathToPose`. 구역 속도·분산 EWMA($\eta=0.2$, 전 로봇 관측 공유):

$$\hat v_e\leftarrow(1-\eta)\hat v_e+\eta\frac{\ell_e}{\tau_e^{obs}},\qquad\hat\sigma_e^2\leftarrow(1-\eta)\hat\sigma_e^2+\eta(\tau_e^{obs}-\ell_e/\hat v_e)^2,\qquad\sigma_{ij}^2=\sum_e\hat\sigma_e^2$$

진행 중 구간은 `NavigateToPose` 피드백 `distance_remaining`, `estimated_time_remaining` 로 $\rho_i$ 갱신(2 s). 혼잡은 구역 속도에 자연히 반영 — CAMETA 의 GNN 충돌 인지 ETA 를 설명 가능한 EWMA 로 대체한 단순화.

### 4.3 정책: 위험조정 최소여유(LST) 디스패칭 + 기대 지연 비용

지연 확률 $\Pr(C_{ij}>d_j)=1-\Phi(\lambda_{ij}/\sigma_{ij})$, 기대 지연(정규 손실함수, $z=-\lambda/\sigma$): $\mathbb E[(C_{ij}-d_j)^+]=\sigma_{ij}\big(z\Phi(z)+\varphi(z)\big)$ (예 $\lambda=-12$ s, $\sigma=20$ s: 폐형식 15.37 s vs MC 15.38 s).

**후보 선정(연속 긴급도, 포화 없음)**: 위험조정 여유 $\tilde\lambda_{ij}=\lambda_{ij}-z_\alpha\sigma_{ij}$ ($z_{0.9}=1.28$; σ→0 이면 $\lambda_{ij}$), 작업 긴급도
$$u_j=-\max_i\tilde\lambda_{ij}+\kappa_p\,\frac{p_j}{255},\qquad\kappa_p=60\ \mathrm s$$
(우선순위는 "마감을 최대 60 s 앞당김" 으로 가산 — 곱셈 가중은 여유 부호에 따라 방향이 뒤집혀 폐기). 유휴 로봇 $m$ + look-ahead $L=2$ 개를 $u_j$ 상위에서 뽑아 비용행렬에 넣는다. 곧 끝나는 로봇($\rho_i<30$ s)은 예측 가용. **디스패치는 이벤트 구동**(작업 도착·로봇 유휴 즉시 실행, 2 s 주기는 재평가용) — 응답시간 200 ms 요건 때문(§5 E8).

**할당 비용(Hungarian, `amr_fleet` 할당기 입력)**: $c_{ij}=\hat\tau(x_i\!\to\!P_j)+w_{late}(1+p_j/255)\,\mathbb E[(C_{ij}-d_j)^+]$, $w_{late}=3$, 모두 [s]. 재조정: 2 s 마다 큐 안 작업의 $\Pr(\text{late})>0.5$ 이고 교환 시 $<0.2$ 면 스왑(히스테리시스).

**성질(단위 테스트, 정정)**: (i) σ→0, $p_j$ 동일 → $u_j=-\max_i\lambda_{ij}$ 이므로 후보 순서는 **최소 여유 우선(LST/LLF)** 과 정확히 동치. (ii) 추가로 모든 $\min_i\hat C_{ij}$ 가 동일(주행+서비스 시간 동일)하면 EDF 와 동치 — 이는 충분조건이고, 일반적으로는 다르다(예: A: $d$=300, $\hat C$=200 → 여유 100; B: $d$=250, $\hat C$=60 → 여유 190; EDF 는 B 먼저, LST 는 A 먼저). (iii) $w_{late}\to\infty$ 의 Hungarian 은 총 가중 기대지연 최소화이며 EDF 가 아니다. 계보: 이 정책은 고전 최소여유(LLF, Mok 1983)·MOD 디스패칭 규칙(Baker & Kanet 1983)과 Hungarian(Kuhn 1955) 에 뉴스벤더형 기대지연을 얹은 **`engineering_adaptation`** 이며, 최근 AGV 슬랙 인지 스케줄링(Li et al., Computers & OR 187, 2026) 및 불확실성 인지 MRTA(Rossano 2026, Mendoza 2026)와 같은 정신이다. 명명된 "독자 알고리즘" 으로 주장하지 않는다.

인터페이스 소유: 본 영역은 `amr_fleet/amr_fleet/scheduler/deadline_dispatch.py` 의 순수 함수 `cost_matrix(robots, tasks, eta_model, t) -> (C[m×n], candidates)` 와 `eta_model.py` 를 제공하고(이름에서 "WLSF" 약어를 뺀 것은 명명된 신규 알고리즘이 아니기 때문), Hungarian(`scipy.optimize.linear_sum_assignment`)과 로봇 `assign_task` 호출은 components.md §3.6 의 `fleet_manager_node` 가 소유한다. `fleet.yaml allocation_strategy` 에 기존 `min_distance / load_balance / hungarian` 옆에 `deadline_lst` 값을 추가한다(§8). 복잡도: 행렬 ≤5×7, Hungarian $O(n^3)$ → µs.

---

## 5. 평가 계획 (명세 지표 매핑)

### 5.1 실험

| # | 시험 | 절차 | 합격 기준 | 비교 대상 |
| --- | --- | --- | --- | --- |
| E1 | 도킹 정밀도 | 도크 3종 × staging 오프셋(±0.15 m, ±10°) × 10회 = 90회. GT = components.md §5.1 의 평가 전용 `ground_truth/odom`(OdometryPublisher, 50 Hz)을 docks.yaml 의 도크 월드 자세로 $D$ 에 변환(필요 시 `/world/<w>/pose/info` 로 교차 확인) | 위치 ≤2 cm, 각 ≤1° 100 %, 3σ 보고 | B0: `ros-humble-opennav-docking` 0.0.2 + 동일 검출기; B1: CGD 에서 게이트 제거·고정 R·맹목 재시도; B2: CGD 전체 |
| E2 | 공분산 일관성 | **접근당 1개**: 도킹 완료 시 $(x_r,y_r,\psi)$ NEES $\nu^\top P^{-1}\nu$ (GT 대비, $\beta$ 증강 EKF 의 앞 3성분 주변), $N=90$ | 3-DoF 평균 NEES ∈ $[\chi^2_{270}(0.025)/90,\ \chi^2_{270}(0.975)/90]=[2.52,3.53]$; 스텝별 NEES(1 Hz 로 솎음)는 진단용 | κ 보정 전/후, $\beta$ 상태 유/무(§3.4), B1 |
| E3 | 모호성 강건성 | 정면 ±2° 접근 50회, yaw flip 발생률 | 채택 해 flip 0건 | 재투영오차 최소해만 사용 |
| E4 | 단일 작업 성공률 | 무작위 작업 **300건**(적재 3종) | 합격 = 점추정 ≥291/300(97 %, 명세 문구 기준) + Clopper–Pearson 95 % 하한 보고. 설계 목표 99 %: 참 97 % 공정은 100건에서 64.7 %, 300건에서 58.7 % 확률로만 통과하지만 참 99 % 면 300건에서 99.9 % 통과. 참고로 "$p\ge0.97$" 을 단측 95 % 신뢰로 **입증**하려면 ≥297/300 이 필요(291/300 의 양측 CP 하한은 0.944) — 보고서에 둘 다 기재 | — |
| E5 | BT 복구 | 결함 주입 (a) 통로 장애물 (b) 마커 visual off (c) staging 오프셋 0.3 m (d) 배터리 15 % (e) E-Stop(도킹 중 포함) | 각 10회 자동 복구 ≥9/10, 복구 시간, E-Stop 후 단계 재개 확인 | — |
| E6 | 스케줄링 | 포아송 도착(λ=1/40 s) 300작업, 마감 $r_j+U(120,400)$ s, 우선순위 3단계 | 마감 미준수율, 평균 지연, 처리량, 총 이동거리 | FIFO / EDF / LST(σ=0) / 최소거리 Hungarian(명세) / 제안 |
| E7 | ETA 오차 | E6 의 **주행 구간(leg: staging→staging)** 단위, $n\ge200$; 도킹·서비스는 별도 상수 | leg MAPE ≤15 %(보정 후) + 작업 총시간 MAPE 병기; 20 s 미만 leg 는 별도 표기 | 명목 속도 모델 |
| E8 | 응답시간·자원 | 명세 표 포맷 `[cmd_time, response_time, latency_ms]`: cmd_time = 시험 클라이언트가 `/fleet/task_request` 를 **발행한** 시각(명세 "작업 명령 발행 timestamp"; JSON 에 `issued_at` 으로 동봉, 도킹 단독 시험은 Dock goal 전송 시각), response_time = 해당 로봇 `cmd_vel`(`safety_node` 출력) 의 첫 $\lvert v\rvert >0.01$ m/s 또는 $\lvert \omega\rvert >0.01$ rad/s 수신 시각(보조 열: `ground_truth/odom` 속도로 본 실제 첫 움직임). 두 시각 모두 벽시계(steady)와 sim 시각을 기록하고 **벽시계 지연을 주 지표**로 보고(연산 지연은 벽시계에 묶임; RTF<1 이면 sim 지연이 과소평가됨). **≥50회**, 평균/최대. 5대 동시 도킹 CPU | 평균 ≤200 ms, BT tick 지터 <10 ms, 본 영역 최악 ≈1.4 코어 | — |

예산(E8, ms, 평균/최악): JSON 검증 5/5 + 이벤트 디스패치 1/1 + `assign_task` RPC 5/5 + **주입 통신지연 U(0,100) 50/100**(components.md §5.6 `fleet_manager_node → /amr_XX/assign_task`, 구 개정판 누락) + BT wake 1/50 + `NavigateToPose` 수락 10/10 + 계획 60/100 + `controller_server` 20 Hz 첫 출력 25/50 + `velocity_profiler_node`·`safety_node` 50 Hz 단계 각 10/20 ≈ **평균 177 / 최악 361 ms**(`checks/audit_checks.py` §9) → 평균 ≤200 ms 는 여유 23 ms 로 빠듯하게 달성 가능, 최대는 초과 가능. 여유 확보책: 대기 중 다음 목적지 경로 선계산(`ComputePathToPose`), 할당 직후 제자리 회전 선행(첫 움직임을 계획 완료 전에 발생).

### 5.2 자동화 통합 시나리오(`launch_testing`, 헤드리스 Gazebo)와 소크

IT-1 정상 fetch&deliver, IT-2 staging 오프셋 도킹, IT-3 마커 가림→복구(R2), IT-4 정밀도 실패 주입→FAILED+재할당(R3, 정확히 3회 접근), IT-5 통로 차단(R1), IT-6 배터리 저하 선점(R4), IT-7 도킹 중 E-Stop→halt→재개(R5), IT-8 마감 순서 디스패치(LST 순서 검증), IT-9 잘못된 JSON 거부(이벤트 없음), IT-10 5대 동시 도킹 CPU/응답시간. IT-4·5·6·7 의 BT 흐름은 Gazebo 없이도 `checks/bt/mission_test.cpp` 시나리오 B·D·E·C 로 단위 수준에서 먼저 검증한다. **4 h 소크**(야간, CI 밖; E4 300건 실행을 겸할 수 있음 — 5대 병렬 작업당 ≈150 s 면 ≈2.5 h): 5대 연속 작업, 60 s 마다 `task_executor_node/docking_server_node/fleet_manager_node` RSS·스레드·활성 goal handle 수 기록, 합격 = RSS 증가 <10 %/h, 누수 goal 0, JSONL 50 MB 회전.

로그 포맷: 도킹 `[stamp, dock_id, attempt, gt_x, gt_y, gt_yaw, est_x, est_y, est_yaw, pos_err, ang_err, gate_p, reason]`; 작업 `[start, end, task_id, robot_id, distance_m, duration_s, result, deadline_slack_s]`.

---

## 6. 구현 계획

### 6.1 패키지·언어·파일

노드 이름·패키지 배치는 components.md §3 을 따른다(`amr_behavior`: `task_executor_node`, `docking_server_node`; `amr_perception`: `aruco_detector_node`; `amr_fleet`: `fleet_manager_node`). `amr_behavior` 는 현재 Python 전용 CMake → C++ 타깃 추가(ament_cmake + ament_cmake_python). `package.xml`: `behaviortree_cpp`, `rclcpp_action`, `tf2_ros`, `nav2_msgs`, `amr_msgs`, `std_msgs`, `geometry_msgs` 유지/추가; `nav2_behavior_tree` 는 v3 플러그인 라이브러리(`IsTTCBelowThreshold`) 전용으로 **유지**하되 실행기 타깃에는 링크하지 않는다(§2.1; 현 `package.xml` 은 `behaviortree_cpp`·`nav2_behavior_tree` 둘 다 선언 — 확인함. v3 는 `nav2_behavior_tree` 를 통한 전이 의존뿐이므로 `behaviortree_cpp_v3` 를 명시 `<depend>` 로 추가하고 CMake 에서 두 타깃의 `ament_target_dependencies` 를 분리). Dockerfile apt 추가는 B0 기준선용 `ros-humble-opennav-docking`(0.0.2-4)과 그 의존 `ros-humble-nav2-graceful-controller`(navigation2 1.1.20-1) 뿐이다(둘 다 Humble 배포, 이미지 미설치 — rosdistro 사본 확인). `twist_mux`(4.3.0-1, Humble 배포)는 **도입하지 않는다**: components.md §4.1 이 `safety_node` 를 `cmd_vel` 유일 발행자로 두고 `cmd_vel_nav` 발행자(controller·behavior·docking) 간 배타를 BT 가 보장하므로 mux 가 필요 없고, mux 를 넣으면 프로파일러·안전 노드를 우회하는 두 번째 `cmd_vel` 발행자가 생긴다.

```
src/amr_behavior/
  include/amr_behavior/bt/{recovery_node,round_robin}.hpp                       # v4 재구현 (checks/bt/mission_test.cpp 에 시험 구현)
  include/amr_behavior/docking/{pnp_covariance,dock_ekf,smooth_control_law,line_fit,gate}.hpp  # ROS 무관 순수 로직
  src/bt/task_executor_node.cpp       # XML 로드, 노드 등록, Groot2Publisher(1667+2i), 20 Hz tick + 이벤트 wake, assign_task 서버
  src/bt/nodes/*.cpp                  # 노드 1개/파일, BT_REGISTER_NODES (자체 24종)
  src/docking/docking_server_node.cpp # amr_msgs/Dock 서버: 1회 접근(SEARCH…VERIFY), LiDAR 직선 적합 내장, safety/dock_exclusion 발행
  src/nav2_plugins/is_ttc_below_threshold.cpp   # 별도 라이브러리 amr_behavior_nav2_bt_plugins (v3, bt_navigator 전용)
  behavior_trees/mission.xml, config/{behavior,docking,docks}.yaml
  test/{test_pnp_covariance,test_dock_ekf,test_control_law,test_gate,test_bt_nodes}.cpp
src/amr_perception/amr_perception/aruco_detector_node.py   # 기존 노드 확장: 두 IPPE 해 + 코너 + ℓ + Σ → perception/dock_marker_observation
src/amr_fleet/amr_fleet/{task_schema.py,task_state.py}, scheduler/{eta_model.py,deadline_dispatch.py}, test/{test_task_schema.py,test_task_state.py,test_scheduler.py}
scripts/bt_xml_to_mermaid.py, scripts/response_time_log.py
```

### 6.2 ROS 인터페이스 (components.md §5 이름 기준, 로봇 네임스페이스 상대)

| 방향 | 인터페이스 | 타입 | 비고 |
| --- | --- | --- | --- |
| 플릿→로봇 | `assign_task` (SrvS, `task_executor_node`) | `amr_msgs/srv/AssignTask`(기존) | `fleet_manager_node` 가 `/fleet/task_request`(JSON String) 를 파싱해 호출 |
| 로봇→플릿 | `task_status`, `executor/phase` | `amr_msgs/Task`, `std_msgs/String` | 플릿이 `/fleet/task_events` 로 재발행, adapter 가 `RobotState.status` 매핑 |
| BT 입력 | `safety/estop_active`, `traffic/hold`, `localization/lost`, `battery_state` | `std_msgs/Bool` ×3, `sensor_msgs/BatteryState` | SafetyGate / TrafficGate / IsBatteryLow |
| BT→Nav2 | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 피드백 → 블랙보드 → 스케줄러 |
| BT→Nav2 | `spin`, `backup`, `global_costmap/clear_entirely_global_costmap`, `local_costmap/clear_entirely_local_costmap`, `speed_limit` | `Spin`, `BackUp`, `ClearEntireCostmap`, `nav2_msgs/SpeedLimit` | 복구 / 마감 위험 시 속도 상한(자체 DWA 플러그인이 `setSpeedLimit()` 구현 필요) |
| BT→도킹 | `dock` | `amr_msgs/action/Dock`(기존, `max_retries=0`) | 피드백 `current_phase` ∈ `search/approach/align/final` |
| 도킹 입력 | `perception/dock_marker_observation`(제안), `perception/dock_marker_pose`(기존), `scan_filtered`, `odometry/filtered`, `payload/mass` | `amr_msgs/MarkerObservation`, `PoseStamped`, `LaserScan`, `Odometry`, `Float32` | |
| 도킹 출력 | `cmd_vel_nav` → `velocity_profiler_node` → `safety_node` → `cmd_vel`; `safety/dock_exclusion`(제안) | `geometry_msgs/Twist`; `geometry_msgs/PolygonStamped` | components.md §4.1 체인 그대로 |
| 적재 | `payload/attach`, `charging/enable` | `std_msgs/String`, `std_msgs/Bool` | `payload_manager_node` 가 DetachableJoint·질량 처리 |

### 6.3 ArUco 구현 세부

검출은 components.md 대로 `amr_perception/aruco_detector_node`(Python)에서 한다: `cv2.aruco.ArucoDetector`(`DICT_4X4_50`, `CORNER_REFINE_SUBPIX`) + `cv2.solvePnPGeneric(..., flags=cv2.SOLVEPNP_IPPE_SQUARE)` 로 두 해와 재투영오차를 얻고, 8×6 코너 투영 야코비안(중심차분, µs 급)으로 $\Sigma_{PnP}$ 를 계산해 `perception/dock_marker_observation` 으로 발행한다(`enabled` 파라미터로 도킹 중만 동작). 현행 이미지 `wf-final` 의 pip: `opencv-contrib-python-headless 4.11.0.86`(Dockerfile 고정, components.md) + `opencv-python 4.11.0.86` 동시 설치, `import cv2` → 4.11.0(구 개정판이 본 `latest` 이미지의 contrib 5.0.0.93 은 이미 교체됨) → 같은 버전이라 당장 충돌은 없지만 cv2 휠 두 개가 같은 경로를 덮으므로 `opencv-python` 제거 권장(§8). 시스템 OpenCV 4.5.4 는 C++ 전용이며 쓰지 않는다. Gazebo 마커: 평면 visual PBR `albedo_map` PNG(여백 포함 0.15 m), 카메라 `<noise type="gaussian" stddev="0.007">`.

### 6.4 단위 테스트 (커버리지 70 % 목표)

- `test_pnp_covariance`: 합성 코너 + σ_c 노이즈, LM-PnP 1000회 → 표본 공분산 vs $\sigma_c^2(J^\top J)^{-1}$ (Z∈{0.32,0.62,1.0}, 허용 ±15 %); 변환 사슬 통과 후 $\sigma_{y_r}$ 가 16 mm(0.62 m) 인지.
- `test_dock_ekf`: 시뮬레이션 궤적 NEES 일관성(N=100 → [2.54,3.50]); 코너 공통 편향 0.05 px 주입 시 $\beta$ 상태 on 이면 대역 유지·off 이면 이탈(`checks/audit2_nees_bias.py` 재현); 부호 반전 해 주입 시 올바른 선택; 초기화가 LiDAR 요각으로 모호성 해소.
- `test_gate`: 결합 확률 MC vs 이변량 CDF(±0.005), Bonferroni 하한 성립, ρ=±1 극한; 사유 코드 표가 게이트 실패의 모든 경우를 한 행에 배정(μψ ∈ {0.3°, 0.41°, 0.7°} → 방위 편차, `checks/audit2_gate_diag.py`).
- `test_control_law`: 유니사이클 적분기, 임의 초기 자세 100건 → 수렴(r<1 cm, |ψ|<0.5°; 스테이징 오프셋 9조합은 `checks/audit_checks.py`(50 Hz) 에서 r<5 mm, |ψ|≤0.36°, `checks/audit2_control_20hz.py`(20 Hz 출력) 에서 r<5 mm, |ψ|≤0.41° 확인), $|\omega|\le\omega_{max}$, $\omega$ 비포화 시 $v\ge v_{min}$·포화 시 $v=\omega_{max}/|\kappa|$(Nav2 원본과 동일 동작).
- `test_bt_nodes`: `checks/bt/mission_test.cpp` 를 gtest 로 이식 — 실제 `mission.xml` 을 로드해 시나리오 A–F(정상 / 도킹 3회 실패 → FAILED 1회·재할당·복귀 / E-Stop halt 후 단계 재개·픽업 재도킹 없음 / 주행 영구 실패 → 복구 4회 후 FAILED / 배터리 선점 후 재개 / 교통 hold) 와 루트 RUNNING 유지 검증; RecoveryNode/RoundRobin tick 시퀀스.
- `test_task_schema.py`·`test_task_state.py`(amr_fleet; 폐기된 `task_manager.py` 대신 모듈 이름에 맞춤): JSON 스키마, 시각 변환(rel/iso → sim `Time`), dock_id 역조회, 상태 전이 표, 이벤트 QoS.
- `test_scheduler.py`: **σ→0·동일 우선순위 → LST 순서와 동치**(독립 구현 비교); 동일 $\hat C$ 추가 시 EDF 동치; 기대 지연 폐형식 vs 수치적분; EWMA 수렴; 스왑 히스테리시스.

### 6.5 연산 예산 (32 스레드 중)

| 구성요소 | 주기 | 비용 | 5대 합 |
| --- | --- | --- | --- |
| ArUco 검출 640×480 + PnP + 8×6 야코비안(Python, cv2 내부 C++) | 30 Hz(Z<0.8 m), 15 Hz(그 외), 도킹 중만 | 3–9 ms | 5대 동시 도킹 최악 30×9 ms×5 ≈ 1.4 코어(32 스레드의 4 %) |
| LiDAR 직선 적합(RANSAC 50) | 10 Hz | <1 ms | 무시 |
| EKF 3+1상태($\beta$) + 게이트 MC 10⁴ | 30 Hz / 접근당 1회 | µs / 0.2 ms | 무시 |
| BT tick | 20 Hz + 이벤트 | <0.1 ms | 무시 |
| 스케줄러 | 이벤트 + 0.5 Hz | <1 ms | 무시 |

### 6.6 일정(주 단위)

W1 v4 실행기·노드·XML(`mission_test` 이식)·Groot2·응답시간 로거. W2 ArUco 노드·PnP 공분산·변환 사슬·단위테스트·Gazebo 마커/도크(docks.yaml 계약). W3 EKF·LiDAR 적합·제어·Dock 서버·E1/E2. W4 게이트·사유 코드·안전 모드·결함주입 E3/E5·IT-1~7. W5 amr_fleet task_schema/state·스케줄러·E6/E7/E8·IT-8~10·소크·문서.

---

## 7. 문헌 조사 (2023-09 → 2026-09, 그리고 고전)

VERIFIED = 초록/코드/메타데이터를 직접 가져옴. RECALLED = 기억(고전만 허용).

**도킹·마커 자세 추정**
1. [VERIFIED] Adámek et al., "Analytical Models for Pose Estimate Variance of Planar Fiducial Markers for Mobile Robot Localisation", Sensors 23(12):5746, 2023-06. 회전분산 $\sigma^2=p_1S_n^{-p_2}e^{-\phi/p_3}+p_4/(90-|\phi|)+p_5$, 적응분산 EKF RMSE 1.28/1.61 cm. https://pmc.ncbi.nlm.nih.gov/articles/PMC10300747/
2. [VERIFIED] Hinderer, Scheffler, Yang, "Investigation of ArUco Marker Placement for Planar Indoor Localization", arXiv 2509.17345, 2025-09. 적응 관측잡음 KF. https://arxiv.org/abs/2509.17345
3. [VERIFIED] Oh & Kim, "Regression-Based Docking System for AMRs Using a Monocular Camera and ArUco Markers", Sensors 25(12):3742, 2025. SolvePnP 6.64° vs 회귀 3.11°, 실도킹 2 cm/3.07°; 단 SolvePnP 거리오차 58.54 cm vs 1.18 cm 는 기준선 보정 불량을 시사 → 근거로서 약함(§3.2). 초록은 Europe PMC 로 재취득·수치 대조(2026-09-22). https://doi.org/10.3390/s25123742
4. [VERIFIED] Dai & Lee, "Multi-Sensor Fusion for AMR Docking: LiDAR, YOLO-Based AprilTag Detection, and Depth-Aided Localization", Electronics 14(14):2769, 2025. LiDAR 군대칭으로 헤딩 편차 추정 + 하이브리드 PID. https://doi.org/10.3390/electronics14142769
5. [VERIFIED] Laurent & Sandoz, "FMAC: a Fair Fiducial Marker Accuracy Comparison Software", arXiv 2601.07723, 2026-01. https://arxiv.org/abs/2601.07723
6. [VERIFIED] Saputra et al., "Autonomous Docking Method via Non-linear MPC", arXiv 2312.16629, 2023. https://arxiv.org/abs/2312.16629
7. [VERIFIED] Min et al., "DVDP: An End-to-End Policy for Mobile Robot Visual Docking with RGB-D", arXiv 2509.13024, 2025-09. 학습 기반(설명가능성 요건상 미채택). https://arxiv.org/abs/2509.13024
8. [VERIFIED] Günzel et al., "Docking and Persistent Operations for a Resident Underwater Vehicle", arXiv 2602.16360, 2026-02. ArUco+EKF 도킹 90 %. https://arxiv.org/abs/2602.16360
9. [VERIFIED] Thakur, Vrba, Saska, "AstraTag: Spacecraft Fiducial Marker for Autonomous RPO and Docking", arXiv 2606.27566, 2026-06. 다중 스케일 마커. https://arxiv.org/abs/2606.27566
10. [VERIFIED] Phoompho et al., "Task-Aware Environment Augmentation for Reliable Navigation via Shielded Conditional Diffusion", arXiv 2606.15154, 2026-06. 마커 배치. https://arxiv.org/abs/2606.15154
11. [VERIFIED(Crossref)] Lamine, Makki, Romdhane, "Experimental Evaluation of Position-Based Visual Servoing Using AprilTag for Mobile Robot Docking", *Emerging Fields in Mechanism and Machine Science* (Mechanisms and Machine Science), Springer Nature, 2026. Nav2→PBVS→오도메트리 마무리 = 우리 2단계 구조. https://link.springer.com/chapter/10.1007/978-3-032-22270-1_30
12. [VERIFIED] Open Navigation, `opennav_docking` humble 브랜치 README: "N retries may be made by driving back to the dock's staging pose", `max_retries` 3, `filter_coef` 0.1, `k_phi` 3.0/`k_delta` 2.0/`beta` 0.4/`lambda` 2.0, `v_linear_min` 0.1/`v_linear_max` 0.25/`slowdown_radius` 0.25, `detected_dock_pose` 구독. https://github.com/open-navigation/opennav_docking/blob/humble/README.md ; 제어식: https://github.com/ros-navigation/navigation2/blob/humble/nav2_graceful_controller/src/smooth_control_law.cpp (`v=v_max/(1+β|κ|^λ)`, `min(v_max·r/r_slow, v)`, `clamp(v_min,v_max)`, ω 포화 후 `v = w_bound/κ` 재계산 확인; 감속 제곱근 항은 humble 소스에 없음; opennav_docking humble `package.xml` 이 `nav2_graceful_controller` 에 의존함도 확인)
13. [VERIFIED(존재)/RECALLED(내용)] Park & Kuipers, "A smooth control law for graceful motion of differential wheeled mobile robots in 2D environment", ICRA 2011, pp. 4896–4902, DOI 10.1109/ICRA.2011.5980167.
14. [RECALLED] Garrido-Jurado et al., ArUco, Pattern Recognition 2014. [RECALLED] Collins & Bartoli, IPPE, IJCV 2014 (OpenCV 헤더 `@cite Collins14` 확인).
15. [VERIFIED, 명칭 구분용] Berning et al., "Spacecraft Relative Motion Planning Using Chained Chance-Constrained Admissible Sets", arXiv 1910.08048; Sanchez, Gavilan, Vazquez, "Chance-constrained MPC for NRHO spacecraft rendezvous", arXiv 2501.10437; D'Onofrio & Zanetti, "Intrinsic Stochastic Successive Convexification on SE(3) for Chance Constrained 6-DOF Rendezvous", arXiv 2608.04114. — 궤적 최적화 안의 확률 제약(우리와 다른 문제); "chance-constrained docking" 명칭 회피 근거.

**Behavior Tree**
16. [VERIFIED] Heppner et al., "Behavior Tree Capabilities for Dynamic Multi-Robot Task Allocation with Heterogeneous Robot Teams", ICRA 2024, arXiv 2402.02833. https://arxiv.org/abs/2402.02833
17. [VERIFIED] Ahmad et al., "Adaptable Recovery Behaviors in Robotics: BT and Motion Generators (BTMG)", arXiv 2404.06129. https://arxiv.org/abs/2404.06129
18. [VERIFIED] Ingrand (단독 저자), "A formal implementation of Behavior Trees to act in robotics", arXiv 2502.11904, 2025-02. https://arxiv.org/abs/2502.11904
19. [VERIFIED] Cai et al., "MRBTP: Efficient Multi-Robot Behavior Tree Planning and Collaboration", arXiv 2502.18072. https://arxiv.org/abs/2502.18072
20. [VERIFIED] Izzo, Bardaro, Matteucci, "BTGenBot-2", arXiv 2602.01870. https://arxiv.org/abs/2602.01870
21. [VERIFIED] Wang et al., "Bridging Probabilistic Inference and Behavior Trees", arXiv 2512.04404. https://arxiv.org/abs/2512.04404
22. [VERIFIED] Gargani, Frosi, Matteucci, "Obstacle-Aware Autonomous Coverage and Navigation for Outdoor Robots", arXiv 2609.01384, 2026-09. https://arxiv.org/abs/2609.01384
23. [VERIFIED] Iovino et al., "A Survey of Behavior Trees in Robotics and AI", RAS 2022, arXiv 2005.05842; Macenski et al., "The Marathon 2", IROS 2020, arXiv 2003.00368. [RECALLED] Colledanchise & Ögren, *Behavior Trees in Robotics and AI*, 2018.
24. [VERIFIED] BehaviorTree.CPP 마이그레이션 가이드 https://www.behaviortree.dev/docs/migration/ ; Groot2 가격/한도(Free: Monitor & Log Visualizer 20 nodes; PRO Unlimited) https://www.behaviortree.dev/groot ; v4 4.10 헤더(`keep_running_until_failure_node.h`, `reactive_sequence.h`, `force_success_node.h`, `tree_node.h` 전제조건) 컨테이너 확인.

**스케줄링·작업 할당**
25. [VERIFIED] Tuck et al., "SMT-Based Dynamic Multi-Robot Task Allocation", NFM 2024, arXiv 2403.11737. https://arxiv.org/abs/2403.11737
26. [VERIFIED] Pal, Chauhan, Baranwal, "Together We Rise: Real-Time MRTA using Coordinated Heterogeneous Plays", AAMAS 2025, arXiv 2502.16079. https://arxiv.org/abs/2502.16079
27. [VERIFIED] Rossano, Lim, How, "Uncertainty-Aware MRTA With Strongly Coupled Inter-Robot Rewards", arXiv 2509.22469 (v3 comment "accepted to IROS 2026"). https://arxiv.org/abs/2509.22469
28. [VERIFIED] Mendoza et al., "Dynamic MRTA under Uncertainty and Communication Constraints: A Game-Theoretic Approach", arXiv 2604.11954, 2026-04. EDD/Hungarian/SCoBA 기준선. https://arxiv.org/abs/2604.11954
29. [VERIFIED] Li et al., "Auction-Based Task Allocation with Energy-Conscientious Trajectory Optimization for AMR Fleets", arXiv 2603.21545. https://arxiv.org/abs/2603.21545
30. [VERIFIED] Sejersen & Kayacan, "CAMETA: Conflict-Aware Multi-Agent ETA Prediction for Mobile Robots", IROS 2023 (arXiv 2503.00074, 2025-02 게시). 초록(arXiv API 재취득): "mean average percentage error improvement of 29.5% and 44% when compared to a traditional path planning method (A *) which does not consider conflicts" — 즉 비교 대상은 충돌 무시 A* 기반 ETA. https://arxiv.org/abs/2503.00074
31. [VERIFIED] Meseguer & Blanes, "Task Allocation in Mobile Robot Fleets: A review", arXiv 2501.08726. https://arxiv.org/abs/2501.08726
32. [VERIFIED(메타데이터: Crossref·Semantic Scholar; 초록은 ScienceDirect 403 으로 미취득)] Li, Wu, Wang, Li, "Slack-aware scheduling of AGVs in just-in-time matrix manufacturing systems via adaptive large neighborhood search", Computers & Operations Research 187, 2026(온라인 2025), DOI 10.1016/j.cor.2025.107340. 제목 수준의 계보 근거로만 인용(세부 기법은 미확인). https://doi.org/10.1016/j.cor.2025.107340
33. [RECALLED] Mok, "Fundamental design problems of distributed systems for the hard-real-time environment", MIT 1983 (LLF/LST). [RECALLED] Baker & Kanet, "Job shop scheduling with modified due dates", J. Oper. Manag. 1983 (MOD). [RECALLED] Liu & Layland, JACM 1973 (EDF). [RECALLED] Kuhn, NRLQ 1955 (Hungarian).

---

## 8. 미결 질문 / 타 영역과의 조율

1. **도크·마커 계약**(월드·description 담당): `docks.yaml`(도크 id, 전면 중앙 자세, 안쪽 법선, staging 자세, 충전 도크 포함) + SDF(전면판 0.8 m × 바닥 0.20–0.65 m, 마커 0.15 m 중심 0.43 m). 현재 월드에 도크 모델·마커 자산 없음(grep 확인).
2. **안전 예외**(§3.8, 소유 = `safety_node`/amr_perception): `safety/dock_exclusion`(`PolygonStamped`, 0.3 s 타임아웃) 신설과 폴리곤 내부 정지거리 0.10 m 규칙 합의.
3. **amr_msgs 변경**: `Task.msg` 확장(`pickup_dock_id`, `dropoff_dock_id`, `budget_s`, `max_retries`; 빈 값/0 = 기본값) 및 `MarkerObservation.msg` 신설(두 IPPE 해 + 코너 + ℓ + 3×3 공분산) — amr_msgs 소유자. 별도 `TaskEvent.msg` 는 만들지 않는다(`task_status` 로 충분).
4. **components.md 갱신 요청**(아키텍처 담당): (a) §3.5/§5.5 `task_executor_node` 의 "`nav2_behavior_tree::BtActionNode` 재사용" → BT.CPP v4 자체 액션 클라이언트, `IsTTCBelowThreshold` 는 별도 v3 라이브러리; (b) §5.5 `docking_server_node` "최대 3회 재시도"·goal `max_retries(=3)` → BT 단일 카운터, 서버 `max_retries=0`(1회 접근); (c) §3.5 BT 노드·서브트리 목록을 §2.3 의 24+14 종·8 서브트리로 교체; (d) Groot 포트 ":1666/1667" → Groot2 `1667+2i`/`1668+2i`; (e) §3.4 `aruco_detector_node` 출력에 `perception/dock_marker_observation` 추가, `docking_server_node` 입력에 `scan_filtered`·`odometry/filtered`·`payload/mass` 추가; (f) §3.6 `allocation_strategy` 에 `deadline_lst` 추가; (g) §5.5 `task_executor_node` 에 Pub `speed_limit`(`nav2_msgs/SpeedLimit`, `SetSpeedLimit`) 추가(DWA 플러그인 `setSpeedLimit()` 구현 필요, §6.2), Sub `perception/detected_objects`(`IsObjectDetected`)·ActC `wait` 는 트리에서 쓰지 않으므로 삭제(각각 `IsMarkerVisible`·`Delay` 로 대체); (h) §5.5 `docking_server_node` Pub `safety/dock_exclusion` 과 `safety_node` 의 해당 Sub 추가(§3.8).
5. **Groot 해석**: 편집기 시각화 + 20 노드 이하 서브트리 모니터가 "시각화(Groot)" 를 충족하는지 평가자 확인.
6. **cv2 이중 설치**: `wf-final` 에 `opencv-contrib-python-headless 4.11.0.86`(고정) 과 `opencv-python 4.11.0.86` 이 함께 있음 → 후자 제거.
7. **cmd_vel 체인**: 도킹 명령은 `cmd_vel_nav` → `velocity_profiler_node` 를 탄다(twist_mux 미도입). 프로파일러의 가가속 제한이 종단 0.10 m/s 추종에 주는 지연을 W3 에서 측정하고, 필요하면 도킹 중 프로파일 파라미터 완화를 내비게이션 담당과 합의.
8. **스케줄러↔할당기 경계**: `cost_matrix()` 제공, Hungarian·`assign_task` 호출은 `fleet_manager_node`; 이벤트 구동 디스패치 훅 필요. 교통 관리자의 `traffic/yield_pose` 이동은 교통 브리프 소관(본 BT 는 hold 동안 대기만).
9. (해결됨) $m_0$: `robot_params.yaml robot.base_mass` 45 kg + 바퀴·캐스터 = 47.6 kg 사용. URDF 관성도 같은 파일에서 생성되므로 별도 가정값 없음.

---

## 9. 리뷰 반영 이력 (2026-09-22)

리뷰 점수 5/10 의 지적을 항목별로 처리했다. 수치는 `rev_calc.py` 로 재현. 아래 표의 "처리" 열은 1차 개정 내용이며, 2차 감사(§9.1)에서 정정된 행은 비고에 **[감사 정정]** 으로 표시했고, 3차 감사(§9.2)의 추가 수정은 §9.2 에 모았다.

| 지적 | 처리 | 비고 |
| --- | --- | --- |
| §3.2/3.4 횡오차 공분산이 카메라 프레임 값(0.2 mm)이고 도크 프레임 지렛대 항(16 mm) 누락, R_c 블록대각 | **수용·재유도**. $\Sigma_{PnP}=\sigma_c^2(J^\top J)^{-1}$ 을 변환 사슬로 전파, 상관 항 유지, 표 재작성(138/62/16.4/2.8 mm, MC 16.7 mm 확인), 결론 재작성 | 리뷰어의 corr ≈ −0.71 은 $(X_{cam},\theta)$ 의 값이고, 게이트가 보는 도크 프레임 $(y_r,\psi)$ 의 상관은 지렛대 항 지배로 $\lvert \rho\rvert \approx1$ 이다(부호는 축 규약 의존). "상관 항을 반드시 모델링" 이라는 결론은 동일하며, 그 결과 게이트는 사실상 $\psi$ 의 주변분포로 결정된다 |
| 프레임/부호 불일치(바깥 법선 vs ψ→0) | **수용**. $x_D$ = 안쪽 법선, 도킹 완료 $(-0.50,0,0)$, $h_L=-(x_r+l_x\cos\psi)$, $H_L=[-1,0,l_x\sin\psi;0,0,1]$, LiDAR 표 거리 1.35/0.65/0.35 m 로 정정, 초기화 절차 추가 | |
| 게이트가 독립 가정의 곱 | **수용**. 결합 rectangle 확률(MC 10⁴ 또는 이변량 CDF) + Bonferroni 하한 | |
| EDF 동치 주장 거짓, 긴급도가 σ→0 에서 포화 | **수용**. LST 동치로 정정(반례 명시), 위험조정 여유 + 가산 우선순위로 교체, 단위 테스트 재정의, LLF/MOD/C&OR 2025 계보 명시, "독자 알고리즘" 주장 철회 | |
| 높이 기준 혼재, VFOV | **수용**. 바닥 기준으로 통일(base 0.18, LiDAR 0.38, 카메라 0.43), 판 0.20–0.65 m, 마커 중심 0.43 m; Z=0.32 에서 반높이 0.228 m ≥ 0.075 m 확인 | 리뷰어의 "0.18 m 아래 마커 → 42°" 는 구 문서의 모호한 표기(0.25 m 를 base 기준으로 읽으면 광축과 일치)에서 비롯; 개정판은 명시적으로 광축 높이에 둠 |
| 두 마커 보드 종단 가시성 | **수용**. $b\le0.40$ m, $Z>0.45$ m 전용 보조로 격하 | |
| 노드/서브트리 수 불일치 | **수용**. 자체 25 + 내장 13, 서브트리 8 로 재집계 | **[감사 정정]** 1차 목록은 XML 과 불일치(ChargeUntil 누락, IsNavStalled·IsDockResultOk·RateController·Inverter 미사용, AlwaysSuccess 누락) → XML 등장 기준 자체 24 + 내장 14, 8 서브트리 전부 XML 정의 |
| NEES 구간의 N 미명시 | **수용**. 접근당 1개, N=90 → [2.52, 3.53]; 스텝별은 진단용 | |
| σ_Z 폐형식 vs CRLB(0.71배), ±30 % 경계 | **수용**. 테스트는 CRLB 기반 모델과 ±15 % 로 비교, 폐형식은 감도 점검용으로 강등 | |
| a_max 0.5 vs 질량식, m_base 출처 없음 | **수용**. 도킹 상한에도 질량 스케일 적용, 45 kg 은 가정값으로 명시 | **[감사 정정]** `config/robot_params.yaml` 에 `robot.base_mass: 45.0` 이 있다(리뷰 시점 이후 추가) → "어디에도 없음" 삭제, 공차 $m_0=47.6$ kg(바퀴·캐스터 포함) 사용 |
| CCD 판정 flawed: 명칭 충돌, opennav_docking·Dai&Lee 와의 증분 불명확, 재시도 회계 | **수용**. CGD 로 개명, "variant of opennav_docking + 적응분산 EKF" 로 위치 재설정, 증분 3가지 명시, 우주 CC 문헌 3편 인용해 구분; 단일 카운터(BT `RetryUntilSuccessful(3)`, 서버 `max_retries=0`), 정지·회전은 접근 내 시간 예산 | 리뷰어가 "Nav2 법칙에 sqrt(2 r a_dec) 항이 있다" 고 기술한 부분은 우리 확인과 다름 — humble 브랜치 소스에는 $v_{max}/(1+\beta\lvert \kappa\rvert ^\lambda)$, $\min(v_{max}r/r_{slow},\cdot)$, clamp 만 있고 감속 항은 우리 추가. v_min 클램프 누락 지적은 수용 |
| BT: KRUF 가 FAILURE 전파(E-Stop·도킹 실패로 실행기 종료), 구조체 포트 문법, Groot2 20 노드, 3×3 재시도 | **수용**. `ForceSuccess` 흡수, `SafetyGate`(RUNNING) + `ReactiveSequence` halt 규칙(헤더 인용), 평탄 키, `task_phase`+`_skipIf` 재개, 복귀 단계 추가, Groot 계획 재작성 | **[감사 정정]** 1차 XML 의 재개 로직은 건너뛴 단계의 `Script` 가 `task_phase` 를 되돌리는 결함(컨테이너 시험 5→3)과 `task_phase:=1` 누락, 주행 불가·Timeout 의 FAILED 경로 부재, 비선점 배터리 분기, `IsDeadlineAtRisk` 가 복구를 촉발하는 결함이 있었다 → §2.3 재작성, BT.CPP 4.10 에서 시나리오 A–F 통과 |
| 스택 사실: graceful/twist_mux/opennav_docking 이 "Humble 에 없음" | **수용·정정**. 셋 다 rosdistro 에 배포됨(navigation2 1.1.20-1 목록, twist_mux 4.3.0-1, opennav_docking 0.0.2-4)이고 이미지에 미설치일 뿐 → apt 로 B0·mux 도입; 직접 구현 근거는 명세 취지로만 유지 | **[감사 정정]** 배포 사실은 rosdistro 사본(`checks/humble_distribution.yaml`)으로 재확인. 단 twist_mux 는 components.md §4.1(`safety_node` 가 유일 `cmd_vel` 발행자) 과 충돌하므로 **도입하지 않음**(리뷰가 허용한 "자체 경로 정당화" 선택지); B0 용 opennav_docking + graceful 만 apt |
| 명세 공백: 복귀 단계, 재시도 회계, 응답시간 로그, 통합 시나리오·4 h, JSON↔Task.msg·시각 도메인, Groot, 안전 모드, 도크 계약, E7 단위, E4 검정력 | **모두 반영**: §2.3 ReturnHome, §2.3 단일 카운터, §5 E8 로그 포맷·≥50회, §5.2 IT-1~10·소크, §2.4 매핑·sim 시각, §2.1 Groot, §3.8 안전 설계, §3.1/§8 도크 계약, §5 E7 leg 단위 n≥200, §5 E4 300건·CP 하한 | 응답시간 요건 때문에 스케줄러를 이벤트 구동으로 바꾼 것은 리뷰가 지적하지 않은 추가 수정 |
| 인용: #11 저자/시리즈, #3 한정, #17 단독 저자, #26 연도 일관, #29 수치 미검증, #12 과장 | **수용**. Crossref 로 #11 확정, #3 에 보정 불량 주석, Ingrand 단독, Rossano "IROS 2026" 통일, CAMETA 초록 원문 인용(29.5 %/44 %), #12 README 원문 인용 | **[감사 재확인]** #11(Crossref), #3(Europe PMC), #18 Ingrand·#27 Rossano·#30 CAMETA·#15 우주 3편(arXiv API), #12(README·소스·package.xml), Groot2 한도(behaviortree.dev) 모두 재취득 일치. CAMETA 수치는 초록에 실제로 있음 → 리뷰의 "초록에 없음" 은 오류. #32 는 초록 미취득이라 메타데이터만 VERIFIED 로 강등·저자/권/DOI 보완 |

### 9.1 2차 감사 (2026-09-22) — 리뷰 항목별 확인과 추가 수정

1차 개정 직후(자체 점검 전) 문서를 리뷰(`critique.json`)의 **42개 항목**(수학 오류 10, 제안 판정 4, 명세 공백 10, 인용 7, 필수 수정 11) 전부와 대조했다. 수학 항목은 `checks/audit_checks.py` 로 독립 재유도, 인용은 재취득, BT 는 컨테이너에서 컴파일·실행했다. 결과: 1차 개정으로 이미 올바르게 해결된 항목 31개, 부분·오해결 11개(아래) → 모두 수정해 **42/42 해결**. 외부 합의가 필요한 잔여 사항은 §8.

| 항목 | 1차 개정의 문제 | 감사 수정 |
| --- | --- | --- |
| 수학: 노드 수(#7) | 목록 ≠ XML | XML 등장 기준 24+14, 8 서브트리 전부 정의 (§2.3) |
| 수학: a_max/m_base(#10) | "저장소 어디에도 없음" 은 현재 거짓 | `robot_params.yaml` 45 kg → $m_0$ 47.6 kg, 적재별 a 값 표기 (§3.5, §3.7) |
| 판정: BT 실행기 / 필수 수정 §2.3 | 재개 시 `task_phase` 후퇴, `task_phase:=1` 누락, 주행 불가·Timeout FAILED 경로 없음, 배터리 비선점, `IsDeadlineAtRisk` 가 복구 촉발 | 단계별 `Sequence _skipIf` 래핑, 단일 실패 처리기, `ReactiveFallback`+히스테리시스, `ForceSuccess` 래핑, `TrafficGate`(components.md `traffic/hold`·`localization/lost`); `checks/bt/mission_test.cpp` 6 시나리오 통과 |
| 판정: CGD / 필수 수정 §8 안전 | 안전 노드 소유자·수단이 components.md 와 불일치(`amr_navigation`, `nav2_collision_monitor`, 형상 없는 String 모드, 상한 0.25 m/s > Critical 0.2 m/s) | 소유자 `safety_node`, `safety/dock_exclusion` 폴리곤, 폴리곤 내부 정지거리 0.10 m, TERMINAL 0.10 m/s (§3.8) |
| 명세 공백: 응답시간 | cmd_time 을 "수신" 으로 정의(명세는 "발행"), 주입 통신지연 0–100 ms 누락, 존재하지 않는 토픽 | 발행 시각·벽시계 주 지표·예산 평균 177/최악 361 ms (§5 E8) |
| 명세 공백: JSON↔Task.msg / 필수 수정 §4.3 | Task.msg 에 dock id 없음, 수신 토픽·소유 노드가 components.md 와 불일치 | `/fleet/task_request`·`fleet_manager_node`, dock id 확장 요청 + 역조회 폴백 (§2.4) |
| 명세 공백: E4 검정력 | 300건 통과확률 61 % 는 계산 오류 | 58.7 %, 입증에는 ≥297/300 필요함을 병기 (§5 E4) |
| 필수 수정: 스택 사실 | twist_mux 도입이 components.md 속도 명령 체인과 충돌 | mux 미도입 근거 명시, B0 용 apt 만 (§6.1) |
| 수학: 게이트(#3) 보강 | $P_t=P+Q_t$ 는 EKF 예측의 $\partial y/\partial\psi=0.30$ 항 누락 | $P_t=\Phi P\Phi^\top+Q_t$; 상관 반영 시 종단 $\sigma_y$ 1.8 mm(3.1 mm 는 상한으로 유지), 게이트 예 0.997/0.93 (§3.6) |
| 판정: 제어 법칙 / 필수 수정 v_min | Nav2 의 ω 포화 후 $v=\omega/\kappa$ 재계산 누락, "마커를 잃는 마지막 0.15 m" 가 §3.1 과 모순 | 식·단위 테스트 성질 수정, 종단 관측 유지로 정정 (§3.5, §6.4) |
| 인용 #32 | "VERIFIED(초록)" 이나 초록 취득 불가, 저자 없음 | 메타데이터 VERIFIED 로 강등, 저자·권·DOI 보완 (§7) |

리뷰 쪽 오류로 판단해 **반영하지 않은** 부분: (i) $(y_r,\psi)$ 상관은 −0.7 이 아니라 ≈−1(−0.71 은 카메라 프레임 $(X,\theta)$ 값; 두 스크립트 모두 확인), (ii) Nav2 humble 제어식에 $\sqrt{2ra}$ 항은 없음(소스 재확인), (iii) CAMETA 29.5 %/44 % 는 초록에 있음.

그 밖의 정합성 수정(리뷰 항목 외): 노드 이름을 components.md 로 통일(`task_executor_node`, `docking_server_node`; ArUco 검출은 `amr_perception/aruco_detector_node` 확장), 오도메트리 입력 `odometry/filtered`, 스캔 `scan_filtered`, 적재는 `payload_manager_node` 의 `payload/attach`, 이벤트는 `task_status` → `/fleet/task_events`, v3 플러그인 `IsTTCBelowThreshold` 보존을 위한 라이브러리 분리, Groot2 포트 `1667+2i`, cv2 버전 사실(`wf-final` 4.11.0.86), 폐형식 $\sigma_X$ 가 CRLB 의 0.71배라는 주석, LiDAR 표가 정확한 빔 기하 MC 대비 5–10 % 보수적이라는 주석, CPU 최악 ≈1.4 코어.

### 9.2 3차 감사 (2026-09-22, 중단 후 재개) — 42개 항목 재대조와 추가 수정

리뷰(`critique.json`)의 42개 항목(수학 10, 제안 판정 4, 명세 공백 10, 인용 7, 필수 수정 11)을 다시 하나씩 대조했다. **재현**: `checks/audit_checks.py` 재실행 출력이 `checks/audit_checks.out` 과 동일, `rev_calc.py` 재실행 일치; `checks/bt/mission_test.cpp` 를 `amr-fleet-system:wf-final` 일회용 컨테이너에서 재컴파일·재실행해 시나리오 A–F 통과(BT.CPP 4.10.0 과 v3 3.8.7 공존 재확인); XML 파싱으로 자체 24·내장 14·서브트리 8(미정의 참조 0) 재계수; 컨테이너에서 graceful/opennav_docking/twist_mux 미설치, `opencv-contrib-python-headless`·`opencv-python` 4.11.0.86 공존, 현 `amr_behavior/package.xml` 의 `behaviortree_cpp`·`nav2_behavior_tree` 동시 선언 확인. **인용 재취득**(2026-09-22): Crossref — #11 Lamine·Makki·Romdhane, *Mechanisms and Machine Science*(Emerging Fields in Mechanism and Machine Science) 2026, #32 Li·Wu·Wang·Li, C&OR 187:107340(2026-03, 온라인 2025-11); arXiv API — #18 Ingrand 단독 저자, #27 Rossano "accepted to IROS 2026", #30 CAMETA 초록의 "29.5% and 44%" 와 "heterogeneous graph neural network"(§4.2 의 GNN 표기 근거), #15 우주 랑데부 3편; Europe PMC — #3 Oh & Kim(1.18 cm/3.11°, SolvePnP 58.54 cm/6.64°, 실도킹 2 cm/3.07°); behaviortree.dev/groot — 무료 "20 nodes", PRO "Unlimited", €590/yr; rosdistro master `humble/distribution.yaml` 실시간 사본 — navigation2 1.1.20-1 패키지 목록에 `nav2_graceful_controller`, opennav_docking 0.0.2-4, twist_mux 4.3.0-1; opennav_docking humble README 의 재시도 문구와 파라미터 기본값.

결과: 40개는 2차 감사 상태에서 올바르게 해결돼 있었고, 2개는 부분 해결이어서 아래처럼 수정했다 → **42/42 해결**.

| 항목 | 남아 있던 문제 | 3차 감사 수정 |
| --- | --- | --- |
| 수학 #1 (R_c 과소평가; 귀결 "κ 로 흡수 불가 → E2 필연 실패") | 지렛대 항·상관 항은 옳게 고쳤으나, 융합 사후분포가 $y_r+0.80\psi$ 방향으로 σ≈0.04 mm 까지 얇아져 프레임 공통 코너 편향 0.05 px 만으로 3-DoF NEES 가 +7.5 → E2 대역 [2.52, 3.53] 이탈. 원 지적의 귀결("스칼라 κ 로 흡수 불가")이 이 방향에서 그대로 재현됨 | EKF 에 공통 편향 상태 $\beta$ 증강($X$ 에 $+Z\beta$, 상수, $\sigma_\beta=0.05$ px, W2 보정) → 같은 편향의 NEES 증분 0.88, $\sigma_y$·$\sigma_\psi$ 불변(§3.4); E2 비교 대상에 β 유/무, `test_dock_ekf` 편향 주입 시험(§5, §6.4); `checks/audit2_nees_bias.py` |
| 제안 판정: CCD→CGD (게이트 + 사유 코드 재시도) | 사유 코드 조건 "$\lvert \mu_\psi\rvert >\tau_\psi$ / $\lvert \mu_y\rvert >\tau_y$" 로는 $0.26^\circ<\lvert \mu_\psi\rvert \le0.6^\circ$ 의 게이트 실패(결합확률 μψ=0.3°/0.41°/0.6° 에서 0.93/0.82/0.50)가 어느 행에도 배정되지 않았고, 본문 예시 "0.3° → 방위 편차" 와 모순. 제어 주기를 components.md 의 20 Hz 로 이산화하면 APPROACH 잔여 헤딩이 0.41° 로 바로 이 구간 | 주변확률 기반 "첫 일치" 규칙 + "상관 잔여" 행(§3.6), 20 Hz 수렴 수치(r<5 mm, ψ 잔차 ≤0.41°)와 종단 폐루프 수렴(0.06°) 추가(§3.5), `test_gate` 에 배정 완전성 시험(§6.4); `checks/audit2_gate_diag.py`, `checks/audit2_control_20hz.py` |

그 밖의 정합성 수정(리뷰 항목 외): `NavigateTo::onHalted()` 는 자기 goal handle 만 취소(§2.3 R4 — 시나리오 E 로그에서 새 goal 이 옛 goal 취소보다 먼저 나감); `robot_params.yaml` 의 두 질량 주석(총 공차 47.6 kg vs "base_mass + payload")의 차이가 가감속 스케일 2 % 미만임을 명시하고 $m_0$ 를 한 곳에서 계산(§3.7); `behaviortree_cpp_v3` 명시 의존과 CMake 타깃 분리(§6.1); amr_fleet 시험 파일명을 `task_schema`/`task_state` 모듈에 맞춤(§6.1, §6.4); components.md 갱신 요청에 (g) `task_executor_node` 의 `speed_limit` 발행 추가·미사용 `perception/detected_objects`·`wait` 삭제, (h) `safety/dock_exclusion` 발행/구독 추가(§8.4); 머리말 검증 범위에 3차 감사 스크립트 추가.
