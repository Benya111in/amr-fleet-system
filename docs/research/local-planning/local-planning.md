# 로컬 플래너 설계 브리프 — 예측형 동적 장애물 회피 DWA (PVT-DWA v2, 리뷰 반영판)

- 작성일: 2026-09-21 / **개정: 2026-09-22 (적대적 리뷰 반영, 이력은 §10)** / 담당 영역: 명세 4.4(Local Planner·Costmap), 4.7(동적 환경 대응), 4.10(성능·테스트), 연계 4.5(CTE·저크)
- 대상 스택: ROS2 Humble, Nav2 1.1.20 (`ros-humble-nav2-core 1.1.20-1jammy.20260908`), Gazebo Sim 6.18 (Fortress), C++17
- 정합 기준: `config/robot_params.yaml`, `config/sensors.yaml`, `config/ekf.yaml`, `docs/architecture/components.md` §3·§4.1·§5, `docs/architecture/sequences.md` §2. **이 문서의 노드·토픽·클래스 이름은 components.md 를 따른다**(플러그인 `amr_navigation::DWAController`, `controller_id: DWA`, 구독 `perception/tracked_obstacles`, 속도 체인 `cmd_vel_nav → velocity_profiler_node → cmd_vel_smoothed → safety_node → cmd_vel`). v1 의 `PvtDwaController`, Nav2 `velocity_smoother`/`collision_monitor` 체인, `/<ns>/fleet/robot_states` 는 폐기했다.
- 문헌 표기: **[VERIFIED]** = 이번 조사에서 원문/초록/서지 레코드를 직접 가져옴(가져온 경로를 괄호로 명시), **[RECALLED]** = 고전 문헌, 기억 기반. 확인 못 한 주장은 문장 안에 "(미확인)" 으로 적었다.
- 수치 검증: 모든 수치는 `research/local-planning/checks/` 의 스크립트로 재유도했다(§5.10, 부록 A). 스택 사실은 일회용 컨테이너 `docker run --rm amr-fleet-system:wf-final`(v1 은 `:latest`; 두 이미지의 Nav2 패키지 버전은 1.1.20 으로 동일)와 GitHub `humble` 브랜치 소스로 확인했다.

## 0. 요약

1. **기준 DWA** 를 차동구동 운동학에서 유도한다(§3). v1 의 오류를 고쳤다: 롤아웃 포즈 수 $N_s\le25$(v1 은 30 과 25 를 혼용), $J_{vel}\in[0,1]$ 정규화, 동적창 $V_d$ 안에서만 만든 제동 후보(ω 도 $\alpha_\omega\Delta t_c$ 제약), 제자리 회전 후보는 $|v_a|\le0.05$ m/s 일 때만, 동적창 중심은 직전 명령(측정값 중심은 실효 가속을 1/3 로 떨어뜨림), $J_{clear}$ 는 참조 경로 대비 초과 비용(좁은 통로 입구 정지 방지). 허용 속도 $V_a$ 는 **저크 제한을 포함한 정지거리**로 정의한다(2 m/s 에서 2.79 m, 저크를 무시한 값은 2.30 m).
2. **문헌 재정위**: v1 이 "없다" 고 한 것은 이미 있다. 확률적 TTC(상계 $\Phi$ 가 ε 를 처음 넘는 시각)를 일정 $(v,\omega)$ 유니사이클 롤아웃에 쓴 **SPAN**(Zhi et al., ICRA 2021), VO 를 DWA 롤아웃 포즈마다 적용한 **DWB critic**(Coissac, KTH 2023, 코드 공개), DWA+ORCA 와 비홀로노믹→홀로노믹 변환(반경 팽창)을 청구한 **Locus Robotics 특허 US 10,429,847 B2**, 예측형 DWA 의 선행(Seder & Petrović ICRA 2007, Molinos et al. RAS 2019), 비홀로노믹 VO(Wilkie et al. IROS 2009 GVO), 확률 DWA(Yasuda et al. RA-L 2023). §4.
3. **PVT-DWA v2** (§5) 는 새 이론이 아니라 **Nav2 Humble 에 맞춘 공학적 통합**이다. (A) 평균 예측에 대한 결정론적 **GVO-TTC**(5 s, 롤아웃 포즈별; Wilkie/Coissac 형) + 동료 AMR 에 한정한 **부호를 바로잡은 ORCA**, (B) 저크 제한 **제동 꼬리(brake tail)** 에 대한 **수동 안전(passive-safety) 기회제약**: $\Pr(\text{가장자리 거리}<0.30\text{ m})<5\,\%$, 상계는 반평면·주축 상자 상계의 최솟값($\bar P=\min(P_{HP},P_{BX})$, MC 대비 항상 상계 성립·차이 중앙값 0.027), (C) 정규화 비용 + 단기(≤1.5 s) 위험지수 + 1 m 이탈 밴드 + 추월 측방 목표 + 복귀 스케줄.
4. **v1 설계가 멈추는(freezing) 이유를 수치로 확인하고 고쳤다**: 긴 시야(3 s) 확률 비용은 σ(3 s)=1.47 m(사람) 때문에 사람 뒤를 0.3 m/s 로 따라가기만 한다(프로토타입 추적, §5.10). v2 는 확률 판정을 **짧은 제동 꼬리**에만 쓰고, 긴 시야는 **평균 예측(결정론)** 으로만 본다.
5. **평가**: 계획기 내부 신호가 아닌 **GT 기하**로 이탈·복귀·충돌을 정의해 DWB/MPPI/TEB 와 같은 정의로 잰다(§6.1). 30회 세트(S1–S6 × 5시드) + 전체 창고 S7(동적 장애물 ≥5 + 로봇 5대) + 안전 사다리 S8 + **자동화 통합 테스트 12개**(§7.5). 통계는 30쌍 풀링 양측 정확 Wilcoxon(n=5 시나리오별 검정은 최소 p=0.0625 라 폐기).
6. **증거 수준**: Python 운동학 프로토타입(추적기 = GT + 잡음, 비반응 장애물, S1–S6 × 8 시드)에서 v2 기본값은 48 회 모두 충돌 0, e-stop·Critical 진입 0, 최대 이탈 0.70 m, 복귀 최대 3.89 s 였다. 예측 없는 대리 변형은 22/48 회 충돌했다. 상계 조임(BX)의 이점은 이 수준에서 드러나지 않았다(§5.10). Gazebo 평가를 대신하지 않는다.

## 1. 명세 요구사항 ↔ 설계 답 ↔ 측정 방법

| 명세 | 정량 목표 | 설계 답(절) | 측정 방법(절) |
|---|---|---|---|
| 4.4 Local Planner | DWA 핵심 직접 구현(샘플링·궤적 시뮬레이션·비용·선택) | `amr_dwa_core`(ROS 비의존) + `amr_navigation::DWAController`(§3, §5, §7.1). DWB 코드 미사용 | gtest(§7.4), 코드 리뷰 체크리스트 |
| 4.4 | TEB 와 성능 비교·장단점 | TEB 는 Humble 바이너리 없음 → W2 소스빌드 go/no-go(§6.4). 실패 시 MPPI 로 대체하고 빠지는 증거를 명시 | §6.4 비교 항목표(시간 분포·국소최소·호모토피 전환·튜닝 공수·동적 모드) |
| 4.4 Costmap | static/obstacle/inflation, inflation 튜닝 | 로컬: obstacle(scan) + voxel(depth) + inflation(0.8 m / 3.0), 전역: global-planning 브리프와 공통(§3.4) | 레이어 설정 덤프 + IT-02 |
| 4.4 | 로봇 폭+20 cm(0.6 m) 통로 무충돌 | 통로 중앙 비용 187 < 253 이라 통과 가능, 헤딩 오차 허용 22.6°(오프셋 0) / 10.2°(오프셋 5 cm)(§3.4). **전제: safety_node 존 거리에서 정적 지도 구조 제외**(§8-2) | S6, IT-02: 충돌 0, 최소 벽 여유, e-stop 0 |
| 4.4 | 동적 장애물용 센서 갱신 주기·장애물 유지 시간 | 로컬 costmap 10 Hz, `observation_persistence: 0.0`, `expected_update_rate` 0.3 s(scan)/0.2 s(depth) = `safety.sensor_timeouts` 와 동일(§3.4) | "유령 셀 수명": 사람이 지나간 셀이 비워지는 시간(GT 로 셀 목록 생성) p95 ≤ 0.3 s |
| 4.4 | 재계획 ≤ 500 ms | 전역 플래너 영역. 로컬은 새 경로 수신 후 첫 명령 ≤ 1주기(50 ms) 보장, 예외로 abort 하지 않음(§5.6) | $T_{plan}$(`planning_time`), $T_{e2e}$(TTC<τ_warn 트랙 메시지 → 새 `plan` 헤더), $T_{ctrl}$(새 `plan` → 첫 `cmd_vel_nav`) 모두 기록(§6.1) |
| 4.5 (연계) | CTE 직선 5 cm / 곡선 10 cm | 컨트롤러 배정: 공용 구역은 `DWA`, CTE 코스·도킹 전 접근은 `PurePursuit`(BT 가 `controller_id` 선택, §5.6.4). 9장 질문에 맞춰 **두 컨트롤러 모두** 같은 코스에서 측정 | 명세 표 포맷 `[timestamp, planned_x/y, actual_x/y, cte]`, GT 기준 + 추정치 기준 병기 |
| 4.5 (연계) | 최대 저크 제한 | 저크 제한은 `velocity_profiler_node`(components.md §3.3, 2.0 m/s³) 가 집행. 플래너는 같은 한계로 정지거리·제동 꼬리를 계산(§3.2) | GT 속도 2회 미분(5 Hz 저역) $\vert j\vert $ p99 ≤ 2.0 m/s³, 안전 클램프 구간 제외·횟수 별도 보고 |
| 4.7 | 미래 위치 예측·충돌 가능성 | CV 예측 + 공분산 $\Sigma(t)$(§5.2), 충돌확률 상계 $\bar P$(§5.4) | gtest: 상계 ≥ MC(등방·이방 60례), 표 형태 차이 보고 |
| 4.7 | TTC 계산·임계값 이하 회피 | 두 TTC: 추적기 TTC(경로 기준, τ_warn 3.0 s → BT 재계획, τ_crit 2.15 s → safety_node 감속; sequences.md §2) + 플래너 GVO-TTC$_0(u)$(샘플별). `avoidance_active` ⟺ TTC$_0(u_{nom})<T_{act}=3.0$ s $<T_{pred}=5$ s(공허하지 않음) | TTC 로그와 회피 시작(횡이탈>0.1 m 또는 감속>0.3 m/s²) 사이 지연 ≤ 2주기 |
| 4.7 | 1.0 m/s 장애물 인식 후 재계획 회피 | BT `IsTTCBelowThreshold` 즉시 재계획 + 1 Hz 재계획, 전역 obstacle layer `observation_persistence 0.0`/5 Hz. 로컬은 20 Hz 연속 회피(§5.6.2) | S1(1.0 m/s 횡단), IT-04 |
| 4.7 | VO 또는 ORCA 반응형 회피 | 사람·지게차 = GVO(롤아웃 포즈별 VO, §5.3.1), 동료 AMR = ORCA(부호 수정·반경 팽창, §5.3.2) | gtest(대칭 정면 대면, 오가지치기율), IT-10 |
| 4.7 | 0.3 m 긴급정지, Warning/Critical 존, E-Stop, 센서 고장 | 집행 주체 `safety_node`(components.md §3.4). 플래너는 `safety/zone` 을 읽어 속도창을 같은 상한(0.5/0.2 m/s)으로 줄이고, 하드 판정 여유 = 0.30 m, 소프트 여유 = 0.50 m 로 **안전 노드가 개입할 일을 미리 피한다**(§5.4, §5.6) | S8·IT-06·IT-07·IT-08: 존 전이 로그, E-stop 지연, 센서 타임아웃 반응 |
| 4.7 성능 | 30회 충돌 0 | §5 전체 + 시나리오 회피가능성 규칙(§6.2) | GT 기하 판정(액터는 물리 충돌 없음) |
| 4.7 성능 | 이탈 ≤ 1 m | 하드 밴드 0.8 m(이미 넘었으면 비증가), 불가 시 1.0 m 완화 | GT 포즈의 **회피 시작 시점 경로** 기준 최대 수직거리(§6.1) |
| 4.7 성능 | 복귀 ≤ 5 s | 복귀 스케줄(e<0.1 m 또는 5 s 까지), 운동학 하한 2.0–3.5 s(§5.5) | GT 기하로 정의한 $t_{clear}$→$t_{return}$(§6.1) |
| 4.1 | 동적 장애물 ≥5, 0.3–1.5 m/s, 직선/곡선/무작위 | S7 전체 창고(사람 3 + 지게차 2 + 동료 AMR 4) | S7 10회 × 10분 |
| 4.10 | 응답 ≤ 200 ms | 로컬 몫: FollowPath 수신 → 첫 명령 ≤ 50 ms | 50회, `[cmd_time, response_time, latency_ms]`, 구간별 분해 |
| 4.10 | 5대 CPU ≤ 80 % | 플러그인 전형 3–6 ms/주기, 최악 ≈30 ms/주기(§5.9) | S7 에서 **호스트 전체** CPU(모든 노드 + Gazebo), 프로세스별 분해 |
| 4.10 | 커버리지 ≥ 70 %, 통합 테스트 ≥ 10 | gtest 12종 + pytest(§7.4), 통합 테스트 IT-01…IT-12(§7.5) | `colcon test --coverage`, CI 로그 |

## 2. 설치 스택·프로젝트 설정에서 확인한 사실

- **컨트롤러 API**(`/opt/ros/humble/include/nav2_core/controller.hpp`): `configure/cleanup/activate/deactivate`, `setPlan(nav_msgs::msg::Path)`, `computeVelocityCommands(PoseStamped, Twist, GoalChecker*) → TwistStamped`, `setSpeedLimit(double, bool)`. 예외는 `nav2_core::PlannerException` 하나(`exceptions.hpp`). DWB 는 이를 상속한 `dwb_core::IllegalTrajectoryException`, `dwb_core::PlannerTFException` 을 던진다(`dwb_core/exceptions.hpp`).
- **controller_server 예외 처리(humble 소스 `nav2_controller/src/controller_server.cpp`)**: 파라미터 `failure_tolerance` 코드 기본값 **0.0**(bringup 예제 yaml 은 0.3). 플러그인이 `PlannerException` 을 던지면 `failure_tolerance>0`(또는 −1)일 때 0 속도를 내고 계속하다가 마지막 유효 명령 후 tolerance 를 넘기면 `"Controller patience exceeded"` 로 다시 던진다. 0.0 이면 즉시 → `publishZeroVelocity()` + `terminate_current()` = **FollowPath abort**. 진행 검사기 실패(`"Failed to make progress"`, `SimpleProgressChecker` 기본 0.5 m / 10 s)도 같은 경로로 abort.
- **기본 BT**(`navigate_to_pose_w_replanning_and_recovery.xml`): `RateController hz=1.0` 재계획, FollowPath 는 `RecoveryNode(retries=1)` 안에서 실패 시 `ClearEntireCostmap(local)`, 전체 `RecoveryNode(6)` 의 복구는 `RoundRobin{ClearCostmaps(local+global), Spin 1.57, Wait 5 s, BackUp 0.30 m @0.05 m/s}`. 즉 **"예외 → 즉시 재계획" 이 아니라 "abort → 복구"** 다(v1 서술 오류, §5.6).
- **Costmap**: obstacle/voxel layer 관측원 파라미터 이름은 `observation_persistence`, `expected_update_rate`, `marking`, `clearing`, `obstacle_max_range`, `raytrace_max_range`, `inf_is_valid`, `data_type`(`libnav2_costmap_2d_core.so`/`liblayers.so` 문자열로 확인). v1 의 `observation_keep_time` 은 ROS1 이름이라 Nav2 에서 무시된다. 비용 상수 `LETHAL 254 / INSCRIBED 253 / MAX_NON_OBSTACLE 252 / NO_INFORMATION 255`. STVL(시공간 감쇠 voxel) 미설치 → 궤적 흔적은 raytrace 로만 지워진다.
- **속도 체인**: Nav2 `velocity_smoother` 파라미터는 `max_velocity/min_velocity/max_accel/max_decel/deadband_velocity/velocity_timeout/feedback/odom_*/scale_velocities/smoothing_frequency` 뿐이며 **저크 제한이 없다**. 프로젝트는 이를 `velocity_profiler_node`(S-curve + 저크 2.0 m/s³ + PID, 50 Hz)로 대체하고, `collision_monitor` 는 `safety_node`(존·TTC 감속·E-stop·센서 타임아웃, 50 Hz, 유일한 `cmd_vel` 발행자)로 대체한다(components.md §4.1). v1 의 "velocity_smoother 가 저크 제한" 은 사실이 아니었다.
- **미설치**: `teb_local_planner`, `costmap_converter`, `spatio_temporal_voxel_layer`(dpkg 확인; index.ros.org 에도 TEB Humble 릴리스 없음).
- **MPPI**: 기본 56 스텝 × 0.05 s, 배치 1000; `ObstaclesCritic`/`CostCritic` 은 costmap 기반(장애물 속도 미사용). README: 4세대 i5 에서 50 Hz 이상.
- **Gazebo Sim 6.18**: actor 에 물리 충돌체가 없다(gz-sim issue #1364 open) → 충돌은 GT 기하로 판정(§6.1). sequences.md §2 의 "Gazebo 접촉 이벤트 카운트" 는 사람 actor 에 대해 성립하지 않는다(§8-3).
- **`config/robot_params.yaml`**: $v\in[-0.5,2.0]$ m/s, $a=1.0$ m/s², $\omega_{max}=1.5$ rad/s, $\alpha_\omega=2.0$ rad/s², 저크 2.0 m/s³, footprint 0.60×0.40 m, 바퀴 반경 0.0825 m, 윤거 0.36 m. `safety`: 거리 기준 **footprint_edge**, e-stop 0.30 / critical 0.50 / warning 1.00 m, **구역 속도 상한 warning 0.5 m/s, critical 0.2 m/s**, `reaction_latency` 0.15 s, 여유거리 기반 연속 상한 $v_{max}(D)=-at_r+\sqrt{(at_r)^2+2a(D-0.30)}$, 센서 타임아웃(LiDAR 0.3 / depth 0.2 / RGB 0.1 / IMU 0.05 / 엔코더 0.06 s), LiDAR·엔코더 고장 → 정지, IMU·카메라 고장 → `degraded_mode_max_speed` 0.2 m/s. v1 의 "Warning 50 %", "slowdown ×0.3", "센서 고장 30 %" 는 설정과 달라 모두 이 값으로 바꿨다.
- **`config/sensors.yaml`**: LiDAR 10 Hz, σ 0.03 m, 720 빔, 25 m, 원점 base_link 전방 0.15 m. Depth 15 Hz.
- **메시지**: `amr_msgs/TrackedObstacle{header, track_id, position, velocity, heading, confidence, is_dynamic, time_to_collision}` — 공분산·반경·클래스 없음(§7.3). `amr_msgs/RobotState{header, robot_id, pose, battery_level, current_task_id, status}` — 속도 없음, 2 Hz, 통신 지연 0–100 ms(multi_robot.md §6). 추적기 출력은 `perception/tracked_obstacles`(map 프레임, 10 Hz, reliable), `is_dynamic` 기준 |v|>0.2 m/s(sequences.md §2).

## 3. 기준 알고리즘: DWA 수학적 유도

### 3.1 상태와 운동학
상태 $\mathbf x=(x,y,\theta,v,\omega)$, 제어 $\mathbf u=(v,\omega)$. 차동구동 순기구학($r=0.0825$ m, $L=0.36$ m):
$$v=\tfrac r2(\omega_R+\omega_L),\quad \omega=\tfrac rL(\omega_R-\omega_L),\quad \dot x=v\cos\theta,\ \dot y=v\sin\theta,\ \dot\theta=\omega .$$
제어주기 $\Delta t_c=1/20$ s. DWA 가정: 한 롤아웃 동안 $(v,\omega)$ 일정(원호).

### 3.2 탐색공간과 허용 속도
$$V_s=\{(v,\omega):v\in[v_{min},\bar v],\ |\omega|\le\omega_{max}\},\qquad \bar v=\min\big(v_{max},\ v_{zone},\ v_{lim}\big)$$
$v_{zone}$ = `safety/zone` 에 따른 0.5(Warning)/0.2(Critical)/0(Stop) m/s, $v_{lim}$ = `setSpeedLimit`·센서 열화 상한(0.2 m/s).
$$V_d=\{|v-v_a|\le a\Delta t_c,\ |\omega-\omega_a|\le\alpha_\omega\Delta t_c\}=[v_a\pm0.05]\times[\omega_a\pm0.10]\quad(\text{m/s, rad/s})$$
**창 중심은 직전 명령** $(v_{c},\omega_c)$ 이고, 측정값(`odometry/filtered`)과 0.3 m/s(0.3 rad/s) 넘게 벌어지면 측정값으로 재설정한다. 측정값을 중심으로 두면 저크 제한 프로파일러의 지연 때문에 매 주기 "측정값 + 0.05" 만 요청하게 되어 **실효 가속도가 한계의 약 1/3** 로 떨어진다(c08 가속 점검, 장애물 없는 30 m: 측정 중심이면 1.95 m/s 도달 7.0 s·주행 18.5 s·평균 가속 0.28 m/s², 명령 중심이면 2.2 s·16.3 s·0.88 m/s²; 저크 제한 이상값은 2.5 s). $\bar v<v_c-0.05$ 이면 창 상한을 $v_c-0.05$ 로 둔다 — 감속은 안전 노드 클램프가 먼저 한다.

**허용 속도 $V_a$** (Fox et al. 의 "최초 충돌 전 정지 가능"): 샘플 원호를 따라 정적 장애물과의 최초 충돌까지의 호 길이 $d(v,\omega)$ 가 제동 꼬리 길이 $s_{stop}(v)$ 보다 길어야 한다.
$$(v,\omega)\in V_a\iff d(v,\omega)>s_{stop}(v)+0.05,\qquad s_{stop}(v)=v\,T_c+s_J(v),\ T_c=\Delta t_c+t_r=0.20\text{ s}$$
$s_J$ 는 가속도 0 에서 저크 $j=2$ m/s³ 로 $-a$ 까지 램프한 뒤 등감속하는 거리다(`velocity_profiler_node` 의 실제 감속). 차원: $vT_c$ [m/s·s], $s_J=v t_R-\tfrac{j t_R^3}{6}+\tfrac{(v-\frac{jt_R^2}{2})^2}{2a}$, $t_R=a/j$ [s] ✓. (c01)

| $v$ [m/s] | 저크 무시 $vt_r+v^2/2a$ (설정 주석식) | 저크 포함, $a_0=0$ | 저크 포함, $a_0=+1$(가속 중) | 정지까지 시간($a_0=0$) |
|---|---|---|---|---|
| 0.5 | 0.200 | 0.315 | 0.867 | 0.90 s |
| 1.0 | 0.650 | 0.889 | 1.816 | 1.40 s |
| 1.5 | 1.350 | 1.714 | 3.016 | 1.90 s |
| 2.0 | 2.300 | 2.789 | 4.466 | 2.40 s |

표는 설정 주석식과 비교하려고 반응 지연 $t_r=0.15$ s 를 썼다. 제동 꼬리는 $T_c=0.20$ s 이므로 거리에 $0.05v$, 시간에 0.05 s 를 더한다. $s_J$ 식은 $v>jt_R^2/2=0.25$ m/s 에서 유효하고, 그 아래는 램프 도중 정지하므로 수치 적분값을 쓴다(c01). `robot_params.yaml` 의 연속 상한식은 저크를 무시한다 — **안전 노드 클램프는 프로파일러를 우회**하므로(설정 주석 "프로파일러 우회") 그 경로에서는 맞다. 플래너가 내는 제동은 프로파일러를 거치므로 저크 포함 값을 쓴다(같은 여유 $D$ 에서 허용 속도: $D$=1.0 m → 0.857 m/s(설정식 1.04), $D$=2.6 m → 1.787 m/s(설정식 2.0), 2.0 m/s 에는 $D\ge3.1$ m 필요). 가속 중 제동($a_0>0$)은 램프가 더 길어 표 4열처럼 커지므로 실제 구현은 현재 $a_0$(프로파일러 상태 추정)를 넣는다.

v1 주의: $V_d$ 폭 ±0.05 m/s 는 물리적으로 옳고 해상도 $\Delta v=0.01$ m/s, $\Delta\omega=0.00667$ rad/s 를 준다. 저크 한계 때문에 실제 가속도 변화는 주기당 0.1 m/s² 로 더 좁다. 명령 중심 창은 프로파일러가 따라가는 목표를 연속으로 주고, 0.3 m/s 재설정 규칙이 실제 속도와의 괴리(예: 안전 클램프 후)를 막는다.

### 3.3 속도 샘플링과 궤적 전개
- 격자 $N_v\times N_\omega=11\times31=341$ 개 + **제동 후보** $u_b=\big(\max(v_a-0.05,0),\ \omega_a-\mathrm{sgn}(\omega_a)\min(|\omega_a|,0.10)\big)$ (항상 $V_d$ 안) + **제자리 회전 후보**는 $|v_a|\le0.05$ m/s 일 때만(v1 은 $V_d$ 밖 후보를 넣었다). 여기서 $(v_a,\omega_a)$ 는 §3.2 의 창 중심값이다.
- **두 시야를 분리**한다. 비용·정적 검사용 $T_{sim}(v)=\mathrm{clip}(|v|/a+0.5,\,1.5,\,2.5)$ s → $N_s=T_{sim}/\Delta t_s\le25$ ($\Delta t_s=0.1$ s; v1 의 상한 3.0 s 는 $v\le2$ 에서 도달 불가). 동적 예측용 $T_{pred}=5.0$ s → $N_p=50$ (평균 예측 거리만 계산, §5.3).
- 정확한 원호 적분($|\omega|\ge10^{-3}$):
$$x_{k+1}=x_k+\tfrac v\omega[\sin(\theta_k+\omega\Delta t_s)-\sin\theta_k],\ \ y_{k+1}=y_k-\tfrac v\omega[\cos(\theta_k+\omega\Delta t_s)-\cos\theta_k],\ \ \theta_{k+1}=\theta_k+\omega\Delta t_s$$
$|\omega|<10^{-3}$ 이면 직선. ($v/\omega$ [m] ✓)
- **제동 꼬리**: $T_c=0.2$ s 동안 $(v,\omega)$ 유지 후 $s_J$ 프로파일로 정지, 곡률 $\kappa=\omega/v$ 유지($|v|\le0.02$ 이면 ω 를 $\alpha_\omega$ 로 감속). 0.1 s 간격 포즈, 최대 25 개. 꼬리는 원호의 앞부분이므로 정적 검사는 원호 결과를 재사용한다.

### 3.4 정적 충돌검사와 costmap 설정
포즈 $k$ 에서 (1) 중심 셀 비용 $c_0\ge253$ → 충돌, (2) $c_0<c_{circ}$ → 자유, (3) 그 사이 → `FootprintCollisionChecker::footprintCostAtPose` 로 4변 래스터(둘레 2.0 m/0.05 m ≈ 40 셀) 후 `LETHAL` 존재 시 충돌. $r_{ins}=0.20$, $r_{circ}=0.361$ m, $c(d)=252e^{-k_s(d-r_{ins})}$, $k_s=3.0$ → $c_{circ}=156$, 0.6 m 통로 중앙($d=0.30$) $c=187<253$ → 통과 가능(c05). 통로에서 횡 오프셋 0 / 2 / 5 cm 일 때 허용 헤딩 오차 22.6° / 17.3° / 10.2°(풋프린트 반폭 $0.3\sin\psi+0.2\cos\psi\le0.3-\text{오프셋}$).

로컬 costmap(`controller_server`): rolling 12 × 12 m(2 m/s × 2.5 s = 5 m 전방 + 여유), 0.05 m, `update_frequency 10.0`, `publish_frequency 2.0`, 레이어 `obstacle_layer`(scan_filtered) + `voxel_layer`(`camera/depth/points_filtered`) + `inflation_layer`(`inflation_radius 0.8`, `cost_scaling_factor 3.0`), 관측원: scan `observation_persistence 0.0`, `expected_update_rate 0.3`(= `sensor_timeouts.lidar`), `obstacle_max_range 10.0`, `raytrace_max_range 12.0`; depth `observation_persistence 0.0`, `expected_update_rate 0.2`, `obstacle_max_range 5.0`. 동적 장애물의 현재 셀은 costmap 에 남겨 둔다(추적 누락 시 폴백). 전역 costmap 은 global-planning 브리프와 같다(`update_frequency 5.0`, scan `observation_persistence 0.0`). path-tracking 브리프의 "로컬 6 × 6 m" 는 5 m 롤아웃을 자르므로 12 × 12 m 로 맞춰야 한다(§8-6).

### 3.5 비용 함수(최소화, 항별 [0,1])
$$J=w_hJ_{head}+w_cJ_{clear}+w_vJ_{vel}+w_pJ_{path}+w_oJ_{osc}+w_tJ_{ttc}+w_rJ_{risk}$$
- $J_{head}=|\mathrm{wrap}(\mathrm{atan2}(y_{ref}-y_N,\ \ell)-\theta_N)|/\pi$ (경로 프레임, 전방 거리 $\ell=\mathrm{clip}(0.8v+0.5,0.6,2.0)$ m, $y_{ref}$ = 추월 측방 목표, 평소 0; §5.5).
- $J_{clear}=\max_{k<N_s}\max\big(0,\ c(\mathbf p_k)-c(\mathbf g_k)\big)/252$, $\mathbf g_k$ = $\mathbf p_k$ 와 같은 호길이 위치의 참조 경로점. **경로 자체가 지나가는 좁은 곳의 비용은 벌점이 아니다.** v1 식($\max_k c(\mathbf p_k)$)은 빠른 샘플일수록 롤아웃이 0.6 m 통로에 먼저 닿아 비용이 커지므로, 통로 입구 앞에서 스스로 감속해 멈추는 국소최소를 만든다(c08b: run4 기본값에서 $J_{clear}$ 만 절대 비용으로 되돌리면 S6 8/8 이 통로 앞에서 멈춰 70 s 안에 도달하지 못한다. 멈춤 최대 59 s 이고, 사람을 무시한 정적 경우도 같다. 상대 비용에서는 8/8 도달, §5.10).
- $J_{vel}=|v_{des}-v|/(v_{max}-v_{min})\in[0,1]$ (v1 의 $(v_{max}-v)/v_{max}$ 는 후진에서 1.25 까지 커졌다, c01). $v_{des}=\min(\bar v,\sqrt{2a\,d_{goal}},\ v_{warn})$, $v_{warn}$ = 예측 Warning 진입점까지 0.5 m/s 로 줄일 수 있는 속도(§5.5).
- $J_{path}=\frac1{N_s}\sum_k\min(|e_\perp(\mathbf p_k)-y_{ref}|/d_{band},1)$, $d_{band}=0.8$ m.
- $J_{osc}$: 전진↔후진, 좌↔우 부호 반전 플래그. $J_{ttc}$, $J_{risk}$ 는 §5.3–5.5.
- 초기 가중치 $w_h=0.8,\ w_c=1.0,\ w_v=0.4,\ w_p=1.2,\ w_o=0.5,\ w_t=1.5,\ w_r=1.0$ (프로토타입 c06 최종값, $w_o$ 는 프로토타입에 없음; Gazebo 튜닝 절차 §6.5).

### 3.6 복잡도(정적 부분)
$M=342$(+회전 후보), $N_s\le25$, $P\approx40$ → 최악 셀 접근 $342\times25\times40=3.42\times10^5$ (v1 의 30 포즈 기준 4.1×10⁵ 는 틀림). 경로 투영은 슬라이딩 인덱스 $\mathcal O(N_s)$.

### 3.7 DWB·MPPI·TEB 와의 원리 비교
| | 본 설계(DWAController) | Nav2 DWB | Nav2 MPPI | TEB |
|---|---|---|---|---|
| 최적화 | 샘플 격자 + 정규화 비용합 | 샘플 + critic 합 | 1000×56 확률 샘플 + softmax 가중 | g2o 희소 비선형 최소제곱 |
| 동적 장애물 | 추적 트랙 평균 예측(GVO, 5 s) + 제동 꼬리 기회제약 | costmap 현재 점유만 | costmap 현재 점유만(Humble) | `include_dynamic_obstacles`(CV 예측, costmap_converter 필요) [RECALLED] |
| 결정성/설명성 | 결정적, 항별 로그 | 결정적 | 확률적(시드) | 국소최소·호모토피 의존 |
| 계산 | 전형 3–6 ms, 최악 ≈30 ms(§5.9) | 수 ms | 수십 ms | 수십 ms |

## 4. 문헌 조사(2023-09 → 2026-09, 재정위)

### 4.1 가장 가까운 선행(v1 누락분, 이번에 확인)
| # | 문헌 | 상태 | 관계 |
|---|---|---|---|
| P1 | Zhi, Lai, Ott, Ramos, *Anticipatory Navigation in Crowds by Probabilistic Prediction of Pedestrian Future Movements* (SPAN), arXiv 2011.06235 (2020), ICRA 2021 | VERIFIED(arXiv PDF 본문; 학회는 검색 인덱스) | §V-A "time-to-collision as the first time a collision occurs under constant controls", §V-B 식 (13)–(14): $p\le\frac12[1+\mathrm{erf}(\frac{r_{\hat x}+r_o-a^\top d}{\sqrt{2a^\top\Sigma a}})]$, $a=d/\Vert d\Vert $, "take the first time probability upper-bound exceeds ε as the time-to-collision". 비홀로노믹 유니사이클, $-1\le v\le1$, $T=4$ s, ε=0.25. **v1 Stage B 의 핵심은 SPAN 과 같다.** |
| P2 | F. Coissac, *Velocity Obstacle method adapted for Dynamic Window Approach*, KTH MSc thesis, 2023-11 (DiVA diva2:1836829), 코드 github.com/FloCoicoi/fc_thesis | VERIFIED(논문 전문) | DWB 의 `AdaptedVO` critic: 롤아웃 포즈마다 $f(t)=\Vert p_{obst}(t)\Vert ^2-R^2$ 의 근·최솟값으로 점수(§4.3.2, 식 4.6–4.8), ROS/Gazebo. **v1 의 "DWA 에 VO 적용 사례 없음" 은 거짓.** |
| P3 | Locus Robotics, US 10,429,847 B2 *Dynamic Window Approach using Optimal Reciprocal Collision Avoidance Cost-Critic* (출원 2017-09-22, 등록 2019-10-01) | VERIFIED(Google Patents) | DWA 선호 속도 → ORCA VO → 결합 목적함수. "Converting the preferred velocity to a holonomic velocity may include increasing the radius of the other robots by a maximum distance between a preferred trajectory and a straight-line trajectory." **v1 Stage A(현 속도 VO/ORCA)의 선행이자 IP 유의 사항**(§8-8). |
| P4 | Seder & Petrović, *Dynamic window based approach to mobile robot motion control in the presence of moving obstacles*, ICRA 2007, pp. 1986–1991 | VERIFIED(학회 PDF) | 이동 셀을 예측해 로봇 예측 궤적과의 충돌점을 가상 장애물로(TVDW 계열) |
| P5 | Molinos, Llamazares, Ocaña, *Dynamic window based approaches for avoiding obstacles in moving*, RAS 118:112–130, 2019 | VERIFIED(서지·초록, ScienceDirect/ACM 검색) | DW4DO / DW4DOT |
| P6 | Wilkie, van den Berg, Manocha, *Generalized Velocity Obstacles*, IROS 2009, pp. 5573–5578 | VERIFIED(저자 PDF) | 제어 입력 공간의 VO: 제약된 로봇의 실제 궤적이 미래에 충돌하는 제어 집합. **v2 Stage A 의 정의**(§5.3.1) |
| P7 | Yasuda, Kumagai, Yoshida, *Safe and Efficient DWA for Differential Mobile Robots With Stochastic Dynamics Using Deterministic Sampling*, RA-L 8(5):2614–2621, 2023, DOI 10.1109/LRA.2023.3257681 | VERIFIED(Crossref 서지; 본문 미확인) | 확률적 동역학 DWA(내용은 미확인이라 비교 주장 없음) |

### 4.2 2023–2026 문헌(v1 유지분, 주장 보정)
| # | 논문 | 상태 | 관계 |
|---|---|---|---|
| L1 | Jian et al., *Long-Term DWA*, RA-L 2023, arXiv 2310.02648 | VERIFIED | 장기 창 + 그래프 최적화, 보완 관계 |
| L2 | Zhang et al., *GF-DWA*, IROS 2025, arXiv 2504.03260 | VERIFIED | 거리 그래디언트 비용, $J_{clear}$ 대안 |
| L3 | Martini et al., *Adaptive Social Force Window Planner with RL*, arXiv 2404.13678 | VERIFIED | 가중치 학습 적응 |
| L4 | Liu, Deng, Wymeersch, *Goal-Oriented Semantic Communication for ISAC-Enabled Robotic Obstacle Avoidance* (MD-DWA), IEEE TWC(accepted), arXiv 2603.02291 | VERIFIED(초록) | UAV, KF 위치 예측 + 마할라노비스 DWA. **장애물이 정적인지는 초록으로 확인 불가**(v1 "정적 회피" 삭제) |
| L5 | Missura & Bennewitz, *Predictive Collision Avoidance for the DWA*, ICRA 2019, pp. 8620–8626, DOI 10.1109/ICRA.2019.8794386 | VERIFIED(Crossref 서지; 본문 미확인) | 결정론적 동적 충돌 모델 DWA. v1 URL(hrl.uni-bonn.de …pdf)은 현재 디렉터리 페이지를 반환 → DOI 로 교체. 장애물 모델(원/다각형)을 읽기 전에는 "Σ≡0 절제 = L5 재현" 이라 말하지 않는다 |
| L6 | Missura et al., *Fast-Replanning Motion Control … Aborting A\**, IROS 2022, arXiv 2109.07775 | VERIFIED | 예측 DWA 를 이겼다고 보고 — DWA 짧은 시야 한계 |
| L7 | Asselmeier et al., *Safe Gap-based Planning in Dynamic Settings*, arXiv 2509.07239 | VERIFIED | 대안 로컬 플래너 |
| L8 | Farrell et al., *Safe Human Robot Navigation in Warehouse Scenario*, arXiv 2503.21141 | VERIFIED | 창고·보행자 시나리오 참고 |
| L9 | Park, Kim, Panagou, *Dynamic Parabolic CBF*, ICRA 2026, arXiv 2510.01402 | VERIFIED | 충돌콘 과보수성 지적 |
| L10 | Stern & Shiller, *From NLVO to NAO*, arXiv 2506.06255 | VERIFIED | 가속도 장애물 |
| L11 | Huang et al., *VO-based CBF*, IEEE TCST 2025, arXiv 2503.00606 | VERIFIED | VO 를 CBF 로 |
| L12 | Huang et al., *GeoPro-VO*, arXiv 2403.10043 | VERIFIED | VO 투영 |
| L13 | Martinez-Baselga et al., *AVOCADO*, T-RO 2025, arXiv 2407.00507 | VERIFIED | 협력도 적응 VO(우리는 상태 기반 책임 ½/1) |
| L14 | Bonanni et al., *MCTS with Velocity Obstacles*, arXiv 2501.09649 | VERIFIED | VO 가지치기 절제 |
| L15 | Trevisan et al., *DRA-MPPI*, IROS 2025, arXiv 2506.21205 | VERIFIED | MC 결합 충돌확률(MPPI) |
| L16 | Dergachev & Yakovlev, *Uncertainty-Aware MA-CA with MPPI*, IROS 2025, arXiv 2507.20293 | VERIFIED | 확률적 ORCA + MPPI, 차동구동·Gazebo |
| L17 | de Groot et al., *Topology-Driven Parallel Trajectory Optimization*, T-RO 2024, arXiv 2401.06021 | VERIFIED | 다중 호모토피 |
| L18 | de Groot et al., *Scenario-based motion planning with bounded probability of collision*, **IJRR 44(9), 2025**, DOI 10.1177/02783649251315203, arXiv 2307.01070 | VERIFIED(Crossref) | 시나리오 기법으로 계획 전체의 결합 충돌확률을 제한. **v2 의 위험지수는 이것의 이산화가 아니다**(v1 귀속 철회) |
| L19 | Wang et al., *Enhanced Probabilistic Collision Detection*, arXiv 2502.15525 | VERIFIED | PCD 일반화 |
| L20 | Liang et al., *Time-aware Motion Planning with Conformal Prediction*, arXiv 2511.18170 | VERIFIED | δ 자동 보정 후보 |
| L21 | Saviolo et al., *Reactive Collision Avoidance for Safe Agile Navigation*, arXiv 2409.11962 | VERIFIED | 최소 TTC 제약 |
| L22 | Jafari et al., *Pedestrian Safety … Predictive Social Force*, AIM 2026, arXiv 2607.09192 | VERIFIED | TTC 통합의 안전지표 개선 |
| L23 | Arul et al., *DS-MPEPC*, arXiv 2303.10133 | VERIFIED | 충돌확률 + TTC 종단비용 |
| L24 | Shen et al., *Motion Planning in Dynamic Environments: A Survey*, arXiv 2606.02677 | VERIFIED(초록) | 138편, VO·포텐셜·동적창을 "additional classical local planning approaches" 로 분류. **v1 의 "하이브리드의 로컬/폴백 모듈" 은 초록에 없는 우리 해석** |
| L25 | Kolomeytsev & Golembiovsky, *Hybrid Motion Planning with DRL*, arXiv 2512.24651 | VERIFIED | 클래스별 안전마진 |
| L26 | Lee & Kim, *Adaptive Trajectory Refinement … Narrow Passages*, arXiv 2510.26142 | VERIFIED | 좁은 통로 |
| L27 | Macenski et al., *From the Desks of ROS Maintainers*, RAS 2023, arXiv 2307.15236 | VERIFIED | Nav2 컨트롤러 개관 |
| C1 | Fox, Burgard, Thrun, *The Dynamic Window Approach to Collision Avoidance*, IEEE RAM 4(1), 1997 | RECALLED | DWA 원전 |
| C2 | Fiorini & Shiller, *Motion Planning in Dynamic Environments using Velocity Obstacles*, IJRR 17(7), 1998 | RECALLED | VO 원전 |
| C3 | van den Berg, Guy, Lin, Manocha, *Reciprocal n-Body Collision Avoidance*, ISRR 2011 | RECALLED | ORCA 반평면(부호 규약 §5.3.2) |
| C4 | Alonso-Mora et al., *Optimal Reciprocal Collision Avoidance for Multiple Non-Holonomic Robots*, DARS 2010 | VERIFIED(제목·출처) | 비홀로노믹 ORCA(추종오차 ε 반경 팽창) |
| C5 | Kluge & Prassler, *Reflective Navigation … Probabilistic Velocity Obstacles*, 2004 | RECALLED | 확률 VO |
| C6 | Zhu & Alonso-Mora, *Chance-Constrained Collision Avoidance for MAVs in Dynamic Environments*, RA-L 4(2), 2019 | RECALLED(식은 리뷰어가 대조 확인) | 선형화 기회제약 — P1 식 (13) 의 출처 [23] 과 같은 형식 |
| C7 | Rösmann et al., TEB, RAS 88, 2017 | RECALLED | TEB |
| C8 | Williams et al., MPPI, ICRA 2017 | RECALLED | MPPI |
| C9 | Fraichard & Asama, *Inevitable collision states — a step towards safer robots?*, Advanced Robotics 18(10), 2004 | RECALLED | 수동 안전·ICS 개념(제동 꼬리 기회제약의 개념적 출처) |

### 4.3 공백 진술(수정)
v1 의 두 공백 진술은 철회한다. (i) 트랙 공분산을 시간 매개 롤아웃에 닫힌형 상계로 넣는 것은 SPAN(P1) 이 이미 했다. Nav2 Humble **플러그인 형태의 공개 구현**은 찾지 못했을 뿐이다(좁은 의미). (ii) VO 를 DWA 샘플 평가에 쓰는 것은 Coissac(P2, DWB critic)·Locus 특허(P3)가 이미 했다. 남는 것은 **공학적 통합의 차이**뿐이다: 저크 제한 제동 꼬리에 대한 수동 안전 기회제약, 안전 노드 존 거리와 맞춘 여유(0.30/0.50 m), 상계의 주축 상자 개선, 1 m 이탈 밴드, 명세 지표에 맞춘 GT 기반 평가·절제.

## 5. PVT-DWA v2

### 5.1 동기와 설계 원칙
DWB/MPPI(Humble)는 costmap 의 **현재** 점유만 본다. 1.5 m/s 지게차와 2 m/s 로봇의 접근 속도는 3.5 m/s 다. 현재 점유만 보는 2.5 s 롤아웃(2 m/s 에서 5 m)은 지게차가 5 m 안에 들어와야 반응하고, 그때 충돌까지 남은 시간 5/3.5 = 1.43 s 는 2 m/s 에서의 제동 시간 2.45 s(§5.4)보다 짧다. 예측을 넣어도 2.5 s 시야로는 8.75 m 밖의 충돌을 보지 못하므로 동적 예측 시야는 5 s(17.5 m)로 따로 둔다. 한편 v1 처럼 3 s 확률 비용을 쓰면 사람의 σ(3 s)=1.47 m 때문에 멀리서부터 속도를 줄여 **사람 뒤를 따라가기만** 한다(§5.10 추적). 그래서 v2 는 원칙을 셋으로 나눈다.
1. **먼 시야(≤5 s)는 평균 예측으로만**: 결정론적 GVO-TTC → 비용($J_{ttc}$)과 상태 전이. 불확실성 폭증이 없다.
2. **확률은 짧은 제동 꼬리에만**: "이 샘플을 $T_c$ 동안 실행한 뒤 저크 제한으로 멈추면, 그동안 가장자리 거리 0.30 m(e-stop) 침범 확률 < 5 %" 를 하드 제약으로 둔다(수동 안전; C9 개념). 꼬리의 시간은 속도에 거의 비례하고($v+0.45$ s) 거리는 속도 제곱에 가깝게 늘어나므로 사람 근처에서는 자동으로 저속이 된다.
3. **안전 노드와 같은 좌표**: 하드 여유 0.30 m = `emergency_stop_distance`, 소프트 여유 0.50 m = `critical_zone_distance`, 속도창 상한 = 구역 상한. 목표는 **안전 노드가 개입할 일을 계획 단계에서 없애는 것**이다.

### 5.2 동적 장애물 모델과 공분산 전파
트랙 $j$(map 프레임) → odom(costmap 프레임) 변환 $T=(R_{mo},\mathbf t)$: $\mathbf p'=R\mathbf p+\mathbf t$, $\mathbf u'=R\mathbf u$, **$\Sigma'_{pp}=R\Sigma_{pp}R^\top,\ \Sigma'_{vv}=R\Sigma_{vv}R^\top,\ \Sigma'_{pv}=R\Sigma_{pv}R^\top$**(v1 은 병진만 다뤘다). CV 예측 + 백색 가속 잡음 $q$ [m²/s³]:
$$\hat{\mathbf p}_j(t)=\mathbf p_j+\mathbf u_j\tau,\qquad \Sigma_j(t)=\Sigma_{pp}+\tau(\Sigma_{pv}+\Sigma_{vp})+\tau^2\Sigma_{vv}+\tfrac{q\tau^3}{3}I_2,\qquad \tau=t+(t_{now}-t_{stamp})$$
(차원: $\tau^2\Sigma_{vv}$ [m²], $q\tau^3/3$ [m²] ✓.) 트래커가 공분산을 주지 않으면 클래스 폴백 $\sigma_p=0.05$ m, $\sigma_v=0.2$ m/s. **σ(t)**(폴백, c02):

| 클래스 | $q$ | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 3.0 s |
|---|---|---|---|---|---|---|
| 사람 | 0.2 | 0.144 | 0.330 | 0.563 | **0.834** | 1.471 |
| 지게차 | 0.1 | 0.129 | 0.275 | 0.453 | 0.655 | 1.124 |
| 동료 AMR | 0.05 | 0.121 | 0.243 | 0.386 | 0.544 | 0.901 |

(v1 은 사람에 대해 σ(2 s)≈0.6–0.66 m 라 했으나 그것은 $q=0.1$ 값이었다. 사람 $q=0.2$ 이면 0.83 m.)

**덮개(cover) 원** — 모두 정확한 외접 덮개이고 여유는 따로 더한다(c05):
- 로봇: 중심 $\pm0.15$ m(축 방향), 반경 $r_R=0.25$ m (모서리 $\sqrt{0.15^2+0.2^2}=0.25$, 사각형 전체 포함 확인).
- 사람: $r=0.25$ m, 원 1개. 지게차(차체 2.0 × 1.0 m 가정, 시뮬레이션 모델과 맞출 것): 원 3개(중심 0, ±0.667 m, 속도 방향 축), 정확 덮개 반경 $\sqrt{0.333^2+0.5^2}=0.601$ m(c05). 본문 표·프로토타입은 0.60 m 로 계산했다(모서리 0.9 mm 미포함, 여유 0.30 m 대비 무시 가능). 구현 파라미터는 덮개 조건을 지키도록 0.61 m 로 올린다(§7.2). |u|≤0.2 m/s 로 방향을 모르면 원 1개 1.12 m. 동료 AMR: 로봇과 같은 2원(방향 = 속도 방향, 정지 시 원 1개 0.361 m). 미분류: 사람 파라미터 + 클러스터 반경.
- 충돌 반경: 하드 $R^h_j=r_R+r_j+0.30$, 소프트 $R^s_j=r_R+r_j+0.50$(사람: 0.80/1.00 m, 지게차 원: 1.15/1.35 m). 원쌍 거리 ≥ $R$ 이면 차체 가장자리 거리 ≥ 여유가 보장된다(덮개가 차체를 포함하므로).

### 5.3 Stage A — 결정론적 VO: 사람·지게차는 GVO, 동료 AMR 은 ORCA

#### 5.3.1 GVO-TTC (P6 의 정의를 롤아웃에 적용, P2 와 같은 계열)
$$\mathrm{GVO}^{T}_j=\{\mathbf u:\exists t\in[0,T_{pred}],\ \min_{\pm,i}\|\mathbf c_\pm(t;\mathbf u)-\hat{\mathbf o}_{j,i}(t)\|<R^s_j\},\qquad \mathrm{TTC}_0(\mathbf u)=\min_j\inf\{t\}$$
($\mathbf c_\pm$ = 로봇 덮개 원 중심, $\hat{\mathbf o}_{j,i}$ = 장애물 덮개 원 중심의 평균 예측.) 0.1 s 포즈 사이는 선형 보간하고 구간마다 $\|\mathbf r_0+\mathbf w s\|^2=R^2$ 의 이차식 근으로 정확한 진입 시각을 구한다(Coissac 식 4.6 형식; $a=\|\mathbf w\|^2,\ b=2\mathbf r_0\cdot\mathbf w,\ c=\|\mathbf r_0\|^2-R^2$, 단위 [m²/s², m²/s, m²] → $s$ [s] ✓). 선형 보간의 호-현 오차는 0.1 s 구간의 새지타 $(v/\omega)(1-\cos(\omega\Delta t_s/2))=1.33(1-\cos0.075)=0.0037$ m 이하($v=2,\omega=1.5$). **원래 VO(현 속도 사상)를 폐기한 근거**(c03): 무작위 장면 39,034 개(40,000 개 생성, 시작 시 겹침 제외)에서 v1 의 현 속도 VO($\tau=2$ s)는 실제 호 충돌의 **24.6 % 를 놓치고**, 자유 샘플의 0.9 % 를 잘못 가지치기했다(|ω|τ/2≤0.3 rad 로 제한해도 0.2 %). 현과 호의 최대 편차는 새지타 $(v/\omega)(1-\cos(\omega\tau/2))$ 와 같다(v=1, τ=2: ω=0.5/1.0/1.5 → 0.245/0.460/0.620 m). v1 의 "필요조건 근사" 서술은 틀렸다.

**롤아웃 연장 모델**: $t\le T_{sim}(v)$ 는 샘플 원호, 그 뒤 $T_{pred}$ 까지는 **도달한 횡오프셋을 유지하며 경로와 평행하게** 속도 $v$ 로 진행한다(Frenet $d$ 일정). 일정 곡률 원호를 5 s 까지 늘리면 횡탈출을 과대평가한다(2 m/s, ω=0.1 rad/s → 5 s 에 2.5 m) — 다음 주기의 경로·헤딩 비용이 로봇을 곧게 펴므로 그런 탈출은 실행되지 않는다. 프로토타입에서는 이 연장의 **효과가 측정되지 않았다**. 최종 설정에서 연장만 원호로 되돌려도 S2 e-stop 0/8, S3 도달 8/8 로 같다(c08, §5.10). S2 의 e-stop 을 없앤 것은 §5.5 의 접근시간 트리거다(초판 6 m 트리거로 되돌리면 e-stop 5/8, 충돌 2). 연장은 다음 주기의 실제 거동에 맞는 모델이라서 채택하며, 성능 이득은 주장하지 않는다.
용도: $J_{ttc}=\max(0,1-\mathrm{TTC}_0/T_{pred})$(비용), `avoidance_active` 판정($\mathrm{TTC}_0(\mathbf u_{nom})<T_{act}=3.0$ s = sequences.md 의 τ_warn; $\mathbf u_{nom}$ = 동적 항을 끈 최적 샘플). 5 s 평균 예측은 곡선 지게차(S5)에서 크게 틀릴 수 있으므로 **하드 판정에는 쓰지 않는다**.

#### 5.3.2 동료 AMR 의 ORCA (부호 규약 수정)
규약(C3): 자기 $A$=로봇, 상대 $B$=동료. $\mathbf p=\mathbf p_B-\mathbf p_A$, 상대속도 $\mathbf v_{rel}=\mathbf v_A-\mathbf v_B$,
$$VO^\tau_{A|B}=\{\mathbf v_{rel}:\exists t\in[0,\tau],\ t\mathbf v_{rel}\in D(\mathbf p,R_o)\},\quad \mathbf u=\Big(\arg\min_{\mathbf x\in\partial VO^\tau_{A|B}}\|\mathbf x-\mathbf v^{opt}_{rel}\|\Big)-\mathbf v^{opt}_{rel},\quad \mathbf n=\text{경계 바깥 법선}$$
$$\mathrm{ORCA}^\tau_{A|B}=\{\mathbf v:(\mathbf v-(\mathbf v^{opt}_A+\lambda\mathbf u))\cdot\mathbf n\ge0\},\quad \lambda=\tfrac12\ (\text{동료 MOVING}),\ 1\ (\text{그 외 상태})$$
v1 은 $\mathbf w=\mathbf u_j-\mathbf v_{eff}$(= $-\mathbf v_{rel}$) 공간에서 $\mathbf u$ 를 구해 로봇 속도에 더했다. 대칭 정면 대면($\mathbf p_B=(4,0.05)$, $\mathbf v_A=(1,0)$, $\mathbf v_B=(-1,0)$, $R_o=0.72$, $\tau=2$)에서 v2 규약은 $\mathbf u_A=-\mathbf u_B=(-0.056,-0.331)$, $\mathbf n_A\cdot\mathbf n_B=-1$, 둘이 ½ 씩 적용하면 τ 동안 최소거리 = 0.720 = $R_o$(경계 접촉)이다. v1 규약은 $\mathbf v_A'=(2.82,0.01)$ 로 **VO 안으로 밀어 넣는다**(c03). 적용 방법(NH-ORCA C4 와 Locus P3 의 방식): 샘플의 홀로노믹 대리 속도 = 현 속도 $\mathbf v_{eff}=v\,\mathrm{sinc}(\omega\tau_o/2)(\cos(\theta+\omega\tau_o/2),\sin(\theta+\omega\tau_o/2))$, $\tau_o=2$ s. **$|\omega|\tau_o/2\le0.3$ rad 인 창에서만** 쓰고 $R_o=0.361+0.361+e_{max}+0.05$, $e_{max}$ = 창 안 새지타 최댓값(v=1: ≤0.149 m, v=2: ≤0.30 m). ORCA 위반 샘플을 제거하되 남는 샘플이 없으면 ORCA 를 이번 주기에 버린다(Stage B 가 최종 판정). 동료 상태는 `/amr_XX/robot_state`(2 Hz) 의 `status`, 트랙 연관은 클래스 `amr` + 최근 `pose` 와 1.5 m 이내. ORCA 는 **책임 분담(상호 춤 방지)** 을 위한 것이고 안전은 Stage B 가 맡는다.

### 5.4 Stage B — 제동 꼬리 수동 안전 기회제약
꼬리 포즈 $k$($t_k\le t_{stop}(v)$), 로봇 원 $\pm$, 트랙 $j$ 의 장애물 원 $i$: $\boldsymbol\xi=\hat{\mathbf o}_{j,i}(t_k)-\mathbf c_\pm(t_k)+\boldsymbol\epsilon$, $\boldsymbol\epsilon\sim\mathcal N(0,\Sigma)$, $\Sigma=\Sigma_j(t_k)+\Sigma_R$($\Sigma_R$ = EKF 위치 공분산, 선택). 사건 $\{\|\boldsymbol\xi\|<R^h_j\}$ 의 확률 상계 두 가지:
1. **반평면(HP; C6·P1)**: 원판 ⊂ $\{\mathbf a^\top\boldsymbol\xi\le R\}$, $\mathbf a=\boldsymbol\mu/\|\boldsymbol\mu\|$ → $P_{HP}=\Phi\big((R-\|\boldsymbol\mu\|)/\sqrt{\mathbf a^\top\Sigma\mathbf a}\big)$.
2. **주축 상자(BX)**: $\Sigma=E\Lambda E^\top$ 의 고유축 $\mathbf e_{1,2}$ 에서 $\mathbf e_i^\top\boldsymbol\xi$ 는 **서로 독립**인 정규변수이고 원판 ⊂ 그 축 정렬 정사각형 $\{|\mathbf e_i^\top\boldsymbol\xi|\le R\}$ 이므로
$$P_{BX}=\prod_{i=1}^2\Big[\Phi\Big(\tfrac{R-m_i}{\sqrt{\lambda_i}}\Big)-\Phi\Big(\tfrac{-R-m_i}{\sqrt{\lambda_i}}\Big)\Big],\quad m_i=\mathbf e_i^\top\boldsymbol\mu .$$
$\bar P=\min(P_{HP},P_{BX})$ 는 여전히 **엄밀한 상계**다(두 포함관계 모두 성립). 등방이면 $P_{BX}\le P_{HP}$ 가 항상 성립한다(각 인자 ≤ 1). 새로운 기법은 아니며 선택 이유는 σ≈R 영역에서의 조임이다(c02, 정확값 = 비중심 카이 적분):

| $\Vert \mu\Vert $, $R$, σ | 정확 | HP(v1) | $\min$(HP,BX) |
|---|---|---|---|
| 1.0, 0.6, 0.66 | 0.133 | 0.272 | 0.168 |
| 1.0, 0.6, 0.83 | 0.122 | 0.315 | 0.153 |
| 1.2, 0.6, 0.33 | 0.022 | 0.035 | 0.032 |
| 1.0, 0.66, 0.15 | 0.009 | 0.012 | 0.012 |

무작위 이방 60례(MC 4×10⁵): 상계 위반 0, 차이 중앙값 0.027·최대 0.121, 비율 중앙값 1.43. v1 단위테스트의 "MC 대비 차이 < 0.03" 은 성립하지 않는다(§7.4 에서 교체).

**하드 판정**: $\max_{k,\pm,i,j}\bar P<\delta_{hard}=0.05$ → "$T_c$ 실행 후 제동하는 동안 e-stop 거리 침범 확률 < 5 %". 모든 샘플(제동 포함)이 위반하면 **최소 위험 폴백**: $V_a$·완화 밴드(1.0 m)를 지키는 샘플 중 $\max\bar P\le\min_{\mathbf u\in V_a}\max\bar P+0.02$ 인 것에서 최소 비용, 그것도 없으면 제동 후보(프로토타입 초기 판의 "제동 후보보다 위험이 낮으면 허용" 규칙이 2 m/s 정면 지게차 장면에서 e-stop 을 낸 것을 보고 바꿨다. 그 판의 소스·기록은 보관하지 않아 이 관찰은 재현 근거가 없다. 폴백 규칙은 위험을 먼저 줄이고 비용은 그다음에 보는 사전식 원칙에 따라 고른 것이다). 필요한 중심거리 $d^*$(δ=0.05, c02b):

| 클래스($R^h$) | $t$=0.2 s | 0.5 | 1.0 | 1.5 | 2.5 |
|---|---|---|---|---|---|
| 사람(0.80) | 0.91 | 1.04 | 1.34 | 1.68 | 2.26 |
| 지게차 원(1.15) | 1.26 | 1.36 | 1.60 | 1.89 | 2.50 |
| 동료 AMR 원(0.80) | 0.91 | 1.00 | 1.20 | 1.43 | 1.87 |

꼬리 길이(정지까지, $T_c$ 포함): 0.5 m/s ≈0.95 s, 1.0 m/s ≈1.45 s, 2.0 m/s ≈2.45 s → 고속일수록 넓은 여유가 필요하므로 사람 근처에서 자동 감속한다. v1 의 "δ_hard=0.10 을 3 s 롤아웃 전체에 적용" 은 지게차에 2.34 m 측방 여유를 요구해 3 m 통로에서 **모든 전진 샘플을 무효화**했다(리뷰 지적, 재확인).

### 5.5 Stage C — 비용 결합, 이탈 밴드, 추월 목표, 복귀
- **위험지수** $J_{risk}=1-\prod_j\big(1-\max_{t_k\le T_{risk}}\bar P^h_{kj}\big)$, $T_{risk}=1.5$ s, **하드 반경 $R^h$**, $w_r=1.0$. (최종 설정에서 위험지수만 소프트 반경·$w_r=2$ 로 되돌리면 S3 추월이 0/8 로 정체된다. 해석: 동행자 옆을 지나는 샘플의 위험지수가 속도 이득보다 커진다. GVO 연장 모델을 되돌리는 것은 S3 에 영향이 없다. c08, §5.10.) 트랙 간 독립 가정의 **지수**이지 확률 상계가 아니다(같은 트랙의 최초 통과 확률은 $\max_k$ 보다 클 수 있다). v1 의 $1-\prod_{k,j}$ 는 250 항 × 0.02 에서 0.994 로 포화했다(c02).
- **이탈 하드 밴드**: $\max_{k<N_s}|e_\perp|\le\max(0.8,\ |e_\perp(0)|+0.02)$ m(이미 넘었으면 더 늘리지 않음). 유효 샘플이 없으면 1.0 m 로 완화.
- **측방 통과 목표 $y_{ref}$**(Frenet 식 측방 오프셋 휴리스틱, 신규성 주장 없음): 경로 회랑 안($|y_j|<R^h_j+0.3$)에서 **경로를 따라 움직이는**($|u_{\perp}|<0.3$ m/s) 트랙이 경로 방향 접근시간 $\text{ahead}/(v_{des}-u_\parallel)<T_{pred}$ 이고 $u_\parallel<\tfrac12v_{des}$(느린 동행자 또는 마주 오는 차)이면 $y_{ref}=y_j\pm\big(R^h_j+1.64\,\sigma_j(t_{ev})\big)$ 중 $|y_{ref}|\le0.75$ 이고 작은 쪽. $t_{ev}$ = 1.2 s(동행: $|u_\parallel-0.5|<0.5$, 제동 꼬리 내내 옆에 있음) / 0.5 s(마주 오는 차: 빨리 지나감). 방향은 장애물을 지날 때까지 유지(히스테리시스). 횡단자($|u_\perp|$ 큼)에는 적용하지 않는다(속도로 양보). 불가능하면 0(따라가기) — BT 의 진행 저하 조건이 재계획을 요청한다(§5.6.2). 초판의 "전방 6 m" 조건은 3.5 m/s 접근에서 충돌 1.7 s 전에야 켜져 늦었다(되돌리면 S2 e-stop 5/8, 충돌 2; c08, §5.10).
- **예견 구역 속도** $v_{warn}$: GVO 평균 예측으로 가장자리 거리 1.0 m 진입까지 남은 경로 길이 $d_w$ 에서 $v_{warn}=\sqrt{0.5^2+2a(d_w-vt_r)}$ (저크 무시 근사; 구현은 §3.2 의 $s_J$ 를 수치로 역산) → 안전 노드의 Warning 클램프(프로파일러 우회, 저크 스파이크)를 피한다. 프로토타입에는 없다(§5.10).
- **복귀**: `avoidance_active` 해제(모든 트랙 TTC$_0=\infty$ 3주기) **또는 측방 통과 목표 해제** 후 $|e_\perp|<0.1$ m 가 될 때까지(최대 5 s) $w_p\times1.5$, $w_h\times1.2$. (프로토타입은 전자에만 스케줄을 걸었다. TTC 활성화 없이 측방 통과만 한 런의 복귀 시간은 기본 가중치로 잰 값이다, §5.10.) 운동학 하한(방향각 뱅뱅, $\alpha_\omega=2$, c01): $e_0=0.8$ m 에서 $v$=0.5/1.0/1.5 m/s → 3.04/2.33/2.02 s, $e_0=1.0$ m → 3.48/2.54/2.20 s. v1 의 "2–3 s" 는 저속에서 낙관적이었고, 리뷰의 "≈4.6 s" 는 1 s 유지 확인창(복귀 **시작** 시각을 정의하는 확인 조건일 뿐 복귀 시간에 더하지 않음)과 선회 중 횡이동을 빼고 계산한 과대평가다. DWA 는 시간 최적이 아니므로 실제 복귀는 하한보다 길다. 하한의 1.3–1.6 배라고 가정하면(가정값, 미측정) 0.5 m/s 에서 4.0–5.6 s 로 경계에 걸린다 → 밴드 경계(0.8 m)·저속(0.5 m/s) 복귀 시험을 따로 둔다(§7.4-9). 프로토타입의 측정값은 §5.10.

### 5.6 안전 계층·Nav2 통합(수정)
#### 5.6.1 속도 체인과 중재
```
controller_server(DWAController, 20 Hz) ─cmd_vel_nav→ velocity_profiler_node(S-curve, 저크 2.0, PID, 50 Hz)
  ─cmd_vel_smoothed→ safety_node(존 0.5/0.2 m/s 상한, TTC 감속 v≤a(TTC−0.15), 0.30 m 즉시정지·래치,
                                   E-stop, 센서 타임아웃; 50 Hz) ─cmd_vel→ ros_gz_bridge → DiffDrive
```
- 우선순위: safety_node(최종 게이트, 두 상한 중 낮은 쪽) > velocity_profiler(E-stop·클램프 시 우회) > 플래너. 플래너는 `safety/zone`(UInt8), `safety/estop_active`(Bool) 을 구독해 $V_s$ 상한을 같은 값으로 줄이고, `estop_active` 동안 0 을 내며 창을 측정 속도로 재설정한다. 플래너는 안전층을 덮어쓰지 않는다.
- E-stop: `estop`/`/fleet/estop` → safety_node 가 0 을 즉시 발행(≤20 ms), DiffDrive 가 관절 속도를 0 으로 → "물리적 즉시 정지". 프로파일러를 거치지 않는다.
- **센서 고장**: 감지·판단 주체는 safety_node(설정 타임아웃: LiDAR/엔코더 → 정지, IMU/카메라 → 0.2 m/s). 플래너 쪽 규칙: `perception/tracked_obstacles` 수신 간격 > 0.3 s(추적 3주기)면 마지막 트랙을 공분산을 키워 0.5 s 까지 쓰고, 그 뒤 동적 항 비활성 + $v_{lim}=0.2$ m/s(= `degraded_mode_max_speed`) + 경고. costmap 기반 정적 검사는 계속한다.
#### 5.6.2 재계획과 예외(Humble 실제 동작 기준)
- 플러그인은 "유효 궤적 없음" 에 **예외를 던지지 않는다**. 제동/최소 위험 명령을 내고 `invalid_streak` 를 센다. 예외는 TF 실패·빈 경로에만 던진다. `failure_tolerance: 1.0`(일시적 TF 결손 흡수), 진행 검사기 `required_movement_radius 0.3`, `movement_time_allowance 20.0`(지게차·사람에게 양보하는 대기가 10 s 를 넘을 수 있다).
- 재계획 경로: (1) BT `IsTTCBelowThreshold`(추적기 TTC < τ_warn 3.0 s) → 즉시 `ComputePathToPose`, (2) `RateController 1 Hz`, (3) **진행 저하**(평균 속도 < 0.3 $v_{des}$ 가 10 s 또는 `invalid_streak` ≥ 40 = 2 s) → 재계획 요청. 전역 costmap 은 scan `observation_persistence 0.0`, `update_frequency 5.0`(global-planning 브리프). 새 경로가 오면 `setPlan` 에서 $y_{ref}$·히스테리시스·복귀 스케줄을 초기화한다.
- 자체 BT 요구사항(amr_behavior 와 합의): 통로에서 `Spin` 금지, 추적 동적 장애물이 3 m 안에 있으면 `ClearEntireCostmap(local)` 금지(움직이는 장애물 앞에서 costmap 을 지우지 않음), `Wait 2 s` → 재계획 → 후방 여유 확인 시에만 `BackUp 0.3 m @0.1 m/s`. 기본 BT 의 abort 경로(ClearLocalCostmap → Spin 1.57 → Wait 5 → BackUp)는 쓰지 않는다.
#### 5.6.3 명세 4.7 "재계획으로 회피" 의 충족
1.0 m/s 횡단자(S1)에 대해 ① 로컬 20 Hz 회피(GVO 비용 + 제동 꼬리), ② TTC 기반 즉시 전역 재계획, ③ safety_node 감속이 겹친다. 측정은 §6.1 의 $T_{e2e}$, $T_{ctrl}$.
#### 5.6.4 컨트롤러 배정(명세 4.5 와의 공존)
components.md 대로 `controller_plugins: ["DWA", "PurePursuit"]`, BT `ControllerSelector` 가 구간별로 `controller_id` 를 지정한다. 배정: 공용 구역(통로·교차로·개방 구역) = `DWA`; 명세 4.5 CTE 코스와 도킹 전 접근(`docking_server_node` 인계 전) = `PurePursuit`. CTE 는 **두 컨트롤러를 같은 코스에서 모두** 측정하고(명세 4.5 목표는 PP, 9장 질문은 DWA), GT 기준과 EKF 추정 기준을 병기한다: CTE$_{GT}$ ≲ CTE$_{est}$ + 위치오차, 직선 위치오차 목표가 5 cm(명세 4.3)이므로 DWA 자체 추종오차는 2–3 cm 이하여야 한다. path-tracking 브리프의 합성 `TrackingController` 안과의 통합 여부는 §8-6.

### 5.7 의사코드
```text
computeVelocityCommands(pose, vel, goal_checker):
  if estop_active: return 0
  tracks ← perception/tracked_obstacles (age ≤ 0.8 s), SE(2) map→odom 변환(평균·속도·공분산 회전), 지연 τ 보정
  vbar ← min(v_max, zone_cap(safety/zone), speed_limit, degraded?0.2)
  c ← last_cmd if |last_cmd − vel| ≤ 0.3 else vel                                   # 창 중심 = 직전 명령
  V ← grid(V_s(vbar) ∩ V_d(c)) ∪ {brake_feasible(c)} ∪ ({spin} if |c_v|≤0.05)
  V ← V \ ORCA-violators(peers MOVING, |ω|τ_o/2≤0.3)   (전부 제거되면 ORCA 무시)       # Stage A-2
  for u in V:
     arc ← rollout(u, 0..T_pred)                       # N_p=50, 비용·정적검사는 앞 N_s(≤25)
     d_free ← 최초 정적 충돌까지 호 길이;  adm ← d_free > s_stop(v)+0.05              # V_a
     band ← max|e⊥| ≤ max(0.8, |e⊥(0)|+0.02)
     TTC0 ← GVO 구간별 이차식(arc→T_sim 후 경로평행 연장, 평균 예측, R^s)             # Stage A-1
     tail ← brake_tail(u);  Pmax ← max min(HP,BX)(tail, R^h)                          # Stage B
     risk ← 1 − Π_j(1 − max_{t≤1.5} Pbar(arc, R^h))
     J ← Σ w_i J_i (y_ref 기준 경로·헤딩, J_clear = 참조 경로 대비 초과 비용) + w_t(1−TTC0/T_pred)^+ + w_r risk
  valid ← adm ∧ band ∧ (Pmax < 0.05);  if none: valid ← adm ∧ band(1.0) ∧ (Pmax<0.05)
  if none: valid ← adm ∧ band(1.0) ∧ {u : Pmax(u) ≤ min_{adm} Pmax + 0.02}             # 최소 위험 폴백
  if none: valid ← {brake_feasible(c)}
  best ← argmin_{valid} J;  invalid_streak 갱신;  avoidance/복귀 스케줄 갱신
  publish ~/dwa/status, ~/dwa/best_trajectory, ~/dwa/candidates;  return best
```

### 5.8 신규성의 정직한 위치
- **주장하지 않는 것**: 새 충돌확률 이론, 새 VO, 최적성·안전성 정리, 학습. 확률적 TTC(P1), 롤아웃별 VO critic(P2), DWA+ORCA 와 반경 팽창(P3), 예측 DWA(P4, P5, L5), GVO(P6), 수동 안전(C9) 이 모두 선행이다.
- **남는 것(공학적 통합)**: (a) 확률 판정을 저크 제한 제동 꼬리에 한정하고 긴 시야는 평균 예측만 쓰는 **시간 분할**(프로토타입에서 v1 식 3 s 확률 비용 대비 S3 추월 8/8 대 0/8, 대신 최소 거리 중앙 0.59 m 감소, §5.10 — 이것이 시험 가능한 주된 설계 차이), (b) 여유를 안전 노드 존 거리(0.30/0.50 m)와 같은 좌표로 둔 것, (c) $\min(P_{HP},P_{BX})$ 로 σ≈R 영역 보수성 완화(정적 계산에서는 확인, 폐루프 이점은 프로토타입에서 드러나지 않음, §5.10), (d) 명세 지표용 GT 기반 평가와 절제.
- **특허 유의**: 동료 AMR ORCA(§5.3.2)는 P3 청구항과 겹칠 수 있다. 연구·시뮬레이션 과제에서는 문제 삼지 않지만 제품화 시 법무 검토 대상이며, 끄더라도(절제 A1) 안전 판정은 Stage B 가 유지한다.

### 5.9 계산 예산(1코어, 20 Hz)
측정(c07, 컨테이너 g++ -O2, 3회): 반평면 상계 18–27 ns/회, $\min(P_{HP},P_{BX})$ 44–52 ns/회, `hypot` 10–26 ns/회. 호스트는 다른 사용자 작업으로 load average 63–111(32 코어)이라 편차가 크다 — 조용한 머신에서 다시 잰다.

| 단계 | 연산량 | 시간(위 측정값) |
|---|---|---|
| 정적: 롤아웃 + 풋프린트 | 342 × 25 포즈 × ≤40 셀 = 3.4×10⁵ 셀 읽기 | ≈1–2 ms(추정, 미측정) |
| GVO 거리(평균 예측) | 거리순 상위 5 트랙, 포즈 35개(0–2 s 0.1 s 간격 20 + 2–5 s 0.2 s 간격 15) × 로봇 원 2 × 장애물 원(사람 1, AMR 2, 지게차 3): 사람만 1.2×10⁵ / 전부 지게차 3.6×10⁵ | 1.2–3.1 ms / 3.6–9.3 ms |
| 제동 꼬리 상계(≤25) + 위험지수(≤15 포즈) | 342 × 40 × 5 트랙 = 6.8×10⁴(등방 Σ: 원쌍 최소거리에서 1회, 상계가 거리에 단조) / 이방 Σ 로 원쌍 전부 평가 시 최대 4.1×10⁵ | 3.0–3.6 ms / 최대 21 ms |
| 비용·정렬·디버그 | — | < 1 ms |

- **전형**(근접 트랙 1–3, 사람): ≈3–6 ms. **최악**(인접 지게차 5대, 이방 Σ): ≈30 ms — 50 ms 주기 안이지만 목표 초과. 대책: $d-R>6\sqrt{\lambda_{max}}$ 이면 $\bar P<10^{-9}$ 이므로 평가 생략(6σ 선별), `max_tracks` 5 → 3, 꼬리 포즈 0.2 s 간격.
- 설계 목표: 조용한 머신에서 S1–S7 p99 ≤ 10 ms(예산의 20 %). gtest 타이밍(§7.4-12)은 CI 잡음을 감안한 회귀 방지선 p99 < 25 ms. v1 의 "5–8 ms"(erfc 30 ns 가정)와 "p99 < 15 ms" 는 근거 없는 값이었다.
- 5대: 전형 0.3–0.6 코어, 최악(모두 동시에 최악 장면) 3 코어. CPU ≤ 80 % 판정은 S7 호스트 전체 측정(§6.1)으로 한다.

### 5.10 프로토타입 검증(운동학 시뮬레이션, c06·c08)
**목적과 한계.** 본문 기본값이 명세 합격선(충돌 0, 이탈 ≤ 1 m, 복귀 ≤ 5 s)과 "멈추지 않음" 을 만족하는지, 그리고 절제 A2·A5·A6 가 무엇을 바꾸는지 **운동학 수준에서** 미리 확인한다. LiDAR·costmap 지연·자기위치 오차·반응형 보행자가 없으므로 Gazebo 평가(§6)를 대신하지 않는다.

**설정**(`checks/c06_proto_sim.py`). 한계는 `robot_params.yaml`($v\in[-0.5,2]$, $a$ 1.0, 저크 2.0, $\omega$ 1.5, $\alpha_\omega$ 2.0). 플래너 20 Hz, 적분 100 Hz. 프로파일러는 저크 제한 1차 추종이고, 안전 게이트는 GT 가장자리 거리로 Warning 0.5 m/s·Critical 0.2 m/s 상한을 걸며 0.30 m 에 들어가면 1.0 m/s² 로 즉시 감속·래치(0.50 m 밖에서 해제)하고 프로파일러를 우회한다. 추적기는 GT + 잡음(위치 σ 0.03 m, 속도 σ 0.08 m/s, 10 Hz, 15 m 이내, 지연 0.05 s 가산)이다. 플래너 공분산은 클래스 폴백($\sigma_p$ 0.05, $\sigma_v$ 0.2)이다. 시나리오는 §6.2 의 S1–S6 기하(3 m 통로, 차선을 지키는 2.0 × 1.0 m 지게차, 0.6 m 통로 등)에 시드 8개(출발 시각·속도·위치 교란)를 쓰고, 충돌 시각은 장애물 없는 명목 주행 시간표에 맞췄다. 목표는 30 m(S6 24 m)이고 70 s 안에 도달하지 못하면 실패로 센다. 지표는 §6.1 의 GT 정의(가장자리 거리, 로봇과 함께 움직이는 회랑으로 정한 $t_{clear}$, 1 s 유지 $t_{return}$)를 따른다. 변형은 다음과 같다: v2(본문 기본값), A2 Σ≡0, A5 HP 상계만, A6 v1 식 3 s 확률 TTC·위험 비용(소프트 반경), **현재 위치만**(예측 없음: costmap 기반 DWB/MPPI 의 운동학 대리일 뿐 그 구현은 아니다), **무시**(동적 장애물을 무시해 시나리오가 실제로 충돌을 만드는지 확인).

**본문 설계와 다른 점**(프로토타입이 검증하지 않는 것): 동료 AMR·ORCA 없음. 제자리 회전 후보·$v_{warn}$·$J_{osc}$ 없음. 정적 검사는 점유 격자 + 풋프린트 20점이고 $J_{clear}$ 는 거리변환 기반 지수 비용이다(costmap 아님). GVO-TTC 는 0.1 s 포즈 표본 거리로 구한다(구간 이차식 아님). `avoidance_active` 는 선택 샘플의 TTC$_0$ 로 판정하고, TTC$_0\ge T_{act}$ 가 3주기 이어지면 해제한다(본문은 $\mathbf u_{nom}$ 과 TTC$_0=\infty$). 복귀 스케줄은 회피 해제에만 걸었다. $t_{start}$ 에는 회랑 진입 선행 조건이 없다(로봇이 경로 위에서 출발하므로 첫 0.1 m 이탈이 곧 회피다). 공분산은 등방 폴백이고 트랙 범위는 15 m 다. 최소 위험 폴백의 최솟값은 $V_a$ 가 아니라 전체 샘플에서 구한 뒤 $V_a$·밴드와 교집합을 잡는다(비면 제동 후보).

**결과**(run4: 6 변형 × 6 시나리오 × 8 시드 = 288 회, `checks/c06_results.json`, 표는 `c06_table.py` 출력 `c06_table_out.md`). "e-stop/Critical 진입 런" 은 해당 존에 한 번 이상 들어간 런 수다. 복귀 열의 [a/b] 는 복귀 시간을 잰 런 수다. "–" 는 잰 런이 없다는 뜻으로, 이탈이 0.10 m 를 넘지 않았거나 장애물이 회랑을 끝까지 떠나지 않아 $t_{clear}$ 가 없는 경우다(A6 S3 은 후자: 사람 뒤를 따라감). ∞ 는 $t_{clear}$ 뒤 기록 끝까지 복귀하지 않은 런이다. 최소 가장자리의 −0.25 는 사람 중심이 로봇 사각형 안에 들어간 겹침, 0.00 은 차체 겹침이다.

| 변형 | 시나리오 | 충돌 런 | e-stop 진입 런 | Critical 진입 런 | 최소 가장자리 [m] | 최대 이탈 [m] | 복귀 최대 [s] [측정 런] | 멈춤 최대 [s] | 도달 | 평균 완료 [s] |
|---|---|---|---|---|---|---|---|---|---|---|
| v2 | S1 | 0/8 | 0 | 0 | 0.87 | 0.03 | – | 0.0 | 8/8 | 18.7 |
| v2 | S2 | 0/8 | 0 | 0 | 0.59 | 0.70 | 2.39 [8/8] | 0.0 | 8/8 | 20.0 |
| v2 | S3 | 0/8 | 0 | 0 | 1.08 | 0.70 | 3.89 [8/8] | 0.0 | 8/8 | 17.9 |
| v2 | S4 | 0/8 | 0 | 0 | 0.59 | 0.54 | 1.81 [3/8] | 0.6 | 8/8 | 21.8 |
| v2 | S5 | 0/8 | 0 | 0 | 0.91 | 0.15 | 0.01 [4/8] | 0.0 | 8/8 | 23.1 |
| v2 | S6 | 0/8 | 0 | 0 | 0.86 | 0.07 | – | 0.0 | 8/8 | 15.6 |
| A2 Σ≡0 | S1 | 0/8 | 0 | 1 | 0.48 | 0.06 | – | 0.0 | 8/8 | 19.1 |
| A2 Σ≡0 | S2 | 0/8 | 0 | 0 | 0.54 | 0.62 | 2.72 [8/8] | 0.0 | 8/8 | 18.2 |
| A2 Σ≡0 | S3 | 0/8 | 0 | 0 | 1.08 | 0.69 | 3.88 [8/8] | 0.0 | 8/8 | 16.3 |
| A2 Σ≡0 | S4 | 4/8 | 7 | 7 | -0.25 | 0.25 | 2.38 [4/8] | 2.1 | 8/8 | 20.3 |
| A2 Σ≡0 | S5 | 2/8 | 2 | 2 | 0.00 | 0.30 | 0.65 [4/8] | 1.8 | 8/8 | 22.7 |
| A2 Σ≡0 | S6 | 0/8 | 0 | 1 | 0.47 | 0.06 | – | 0.0 | 8/8 | 16.1 |
| A5 HP만 | S1 | 0/8 | 0 | 0 | 0.92 | 0.05 | – | 0.0 | 8/8 | 18.4 |
| A5 HP만 | S2 | 0/8 | 0 | 0 | 0.59 | 0.70 | 2.42 [8/8] | 0.0 | 8/8 | 20.1 |
| A5 HP만 | S3 | 0/8 | 0 | 0 | 1.06 | 0.68 | 3.80 [8/8] | 0.0 | 8/8 | 18.6 |
| A5 HP만 | S4 | 0/8 | 0 | 0 | 0.63 | 0.34 | 2.15 [5/8] | 0.4 | 8/8 | 21.7 |
| A5 HP만 | S5 | 0/8 | 0 | 0 | 0.90 | 0.17 | 0.01 [3/8] | 0.0 | 8/8 | 23.1 |
| A5 HP만 | S6 | 0/8 | 0 | 0 | 0.94 | 0.04 | – | 0.0 | 8/8 | 15.4 |
| A6 v1식 3 s 확률비용 | S1 | 0/8 | 0 | 0 | 1.41 | 0.03 | – | 0.0 | 8/8 | 18.8 |
| A6 v1식 3 s 확률비용 | S2 | 0/8 | 0 | 0 | 0.63 | 0.71 | 2.30 [8/8] | 2.3 | 8/8 | 21.7 |
| A6 v1식 3 s 확률비용 | S3 | 0/8 | 0 | 0 | 2.26 | 0.64 | – | 0.0 | 0/8 | 70.0 |
| A6 v1식 3 s 확률비용 | S4 | 0/8 | 0 | 0 | 0.66 | 0.56 | 2.33 [3/8] | 2.3 | 8/8 | 25.5 |
| A6 v1식 3 s 확률비용 | S5 | 0/8 | 0 | 0 | 1.16 | 0.16 | 0.01 [3/8] | 0.0 | 8/8 | 22.3 |
| A6 v1식 3 s 확률비용 | S6 | 0/8 | 0 | 0 | 1.42 | 0.07 | – | 0.0 | 8/8 | 15.8 |
| 현재위치만(costmap형) | S1 | 6/8 | 8 | 8 | -0.25 | 0.07 | – | 0.4 | 8/8 | 18.4 |
| 현재위치만(costmap형) | S2 | 0/8 | 0 | 1 | 0.50 | 0.62 | 2.61 [8/8] | 0.0 | 8/8 | 18.1 |
| 현재위치만(costmap형) | S3 | 0/8 | 0 | 0 | 1.08 | 0.69 | 3.90 [8/8] | 0.0 | 8/8 | 16.4 |
| 현재위치만(costmap형) | S4 | 4/8 | 8 | 8 | -0.25 | 0.56 | ∞(1) [3/8] | 1.5 | 8/8 | 21.8 |
| 현재위치만(costmap형) | S5 | 4/8 | 5 | 5 | 0.00 | 0.31 | 1.11 [8/8] | 2.5 | 8/8 | 23.5 |
| 현재위치만(costmap형) | S6 | 8/8 | 8 | 8 | -0.25 | 0.17 | 1.27 [1/8] | 0.4 | 8/8 | 15.3 |
| 무시(충돌 확인용) | S1 | 8/8 | 8 | 8 | -0.25 | 0.00 | – | 0.0 | 8/8 | 16.3 |
| 무시(충돌 확인용) | S2 | 8/8 | 8 | 8 | 0.00 | 0.00 | – | 0.0 | 8/8 | 16.3 |
| 무시(충돌 확인용) | S3 | 0/8 | 0 | 8 | 0.41 | 0.00 | – | 0.0 | 8/8 | 16.3 |
| 무시(충돌 확인용) | S4 | 8/8 | 8 | 8 | -0.25 | 0.00 | – | 0.0 | 8/8 | 16.3 |
| 무시(충돌 확인용) | S5 | 8/8 | 8 | 8 | 0.00 | 0.00 | – | 0.0 | 8/8 | 16.3 |
| 무시(충돌 확인용) | S6 | 8/8 | 8 | 8 | -0.25 | 0.00 | – | 0.0 | 8/8 | 13.3 |

**쌍비교**(시나리오×시드 48 쌍, 양측 Wilcoxon, 차이 = v2 − 비교 변형; 1차 지표 최소 가장자리 거리, 2차 완료 시간):

| 비교 | 최소 가장자리 차이 중앙값 | p | 완료 시간 차이 중앙값 | p |
|---|---|---|---|---|
| v2 vs 현재 위치만 | +0.722 m | 2.0×10⁻¹¹ | +0.64 s | 5.0×10⁻⁵ |
| v2 vs A2 Σ≡0 | +0.290 m | 5.2×10⁻⁹ | +0.62 s | 2.1×10⁻⁴ |
| v2 vs A5 HP 만 | −0.013 m | 0.0095 | −0.02 s | 0.91 |
| v2 vs A6 3 s 확률 비용 | −0.591 m | 7.1×10⁻¹⁵ | −0.58 s | 1.0×10⁻³ |

(미도달 런은 완료 시간을 70 s 로 넣었다. 운동학 대리 실험의 p 값이라 §6.3 의 Gazebo 검정을 대신하지 않는다.)

**읽는 법.**
1. **v2 기본값**: 48 회 모두 충돌 0, e-stop·Critical 진입 0, 전부 도달. 최소 가장자리 0.59 m, 최대 이탈 0.70 m(밴드 0.8 m 안), 복귀 시간은 잰 23 회에서 중앙값 2.33 s·최대 3.89 s(S3 추월; S2 는 최대 2.39 s), 멈춤 최대 0.6 s. 이 조건에서는 명세 4.7 합격선을 만족한다. S3 의 최대 3.89 s 는 운동학 하한(§5.5: $e_0$=0.8 m 에서 속도 1.5–0.5 m/s 일 때 2.0–3.0 s)보다 길고 5 s 까지 1.1 s 여유뿐이다 → §7.4-9 저속 경계 시험을 유지한다.
2. **예측의 효과**(v2 vs 현재 위치만): "현재 위치만" 은 S1·S4·S5·S6 에서 48 회 중 22 회 충돌했다. 사람이 좁은 통로 입구를 가로지르는 S6 은 8/8 이다. 최소 가장자리 중앙값 차이는 +0.72 m 다. "무시" 가 S3 을 뺀 모든 시나리오에서 8/8 충돌하므로 시나리오 자체는 실제 충돌 상황이다(S3 은 사람이 랙 쪽을 걸어 Critical 만 8/8).
3. **공분산의 효과**(A2 Σ≡0): 평균 예측만 쓰면 무작위 보행(S4) 4/8, 곡선 지게차(S5) 2/8 에서 충돌했다. CV 평균 예측이 틀리는 장면에서 제동 꼬리 기회제약의 여유가 안전을 만든다.
4. **상계 조임**(A5, HP 만): 차이가 작다(최소 가장자리 −0.013 m, 완료 시간 차이 없음). **이 프로토타입에서는 $\min(P_{HP},P_{BX})$ 의 이점이 드러나지 않는다**. 제동 꼬리(≤2.45 s)에서는 상계 차이가 큰 σ≈R 영역이 드물기 때문이라고 추정한다(미검증). 계산 비용(44–52 대 18–27 ns/회, §5.9)을 고려해 Gazebo 절제 A5 로 HP 만으로 충분한지 다시 판정한다.
5. **시간 분할의 효과**(A6, §5.8 (a)): A6 은 S3 에서 8/8 모두 0.3 m/s 사람 뒤를 2.26 m 이상 떨어져 따라가다 70 s 안에 도달하지 못했다. 48 쌍 중앙값으로 보면 A6 이 v2 보다 0.59 m 더 넓게, 0.58 s 더 느리게 지나갔다. 두 변형 모두 충돌·e-stop 은 0 이다. 즉 v2 는 이 조건에서 **안전 판정 손실 없이 추월 정체를 없앴지만 더 가깝게 지나간다**. 가까워진 만큼이 허용되는지는 1차 지표로 Gazebo 에서 판정한다.

**S2·S3 수정의 귀속**(`checks/c08_attrib.py`, `c08_results.json`; run4 기본값 F 에서 한 요인씩 되돌림, S2·S3 × 8 시드. F 는 run4 v2 의 S2·S3 결과를 그대로 재현함을 확인):

| 설정 | S2 e-stop 진입 런 | S2 충돌 런 | S2 최소 가장자리 | S3 도달 | S3 멈춤 최대 |
|---|---|---|---|---|---|
| F (최종: 경로평행 연장, 접근시간 트리거, 위험지수 $R^h$·$w_r$=1) | 0/8 | 0 | 0.59 m | 8/8 | 0.0 s |
| A: 일정 곡률 원호 연장 | 0/8 | 0 | 0.57 m | 8/8 | 0.0 s |
| B: 초판 트리거(전방 6 m) | 5/8 | 2 | 0.00 m | 8/8 | 0.0 s |
| C: 위험지수 소프트 반경·$w_r$=2 | 0/8 | 0 | 0.58 m | **0/8** | 1.1 s |
| ABC: 셋 다 되돌림 | 0/8(Critical 1) | 0 | 0.41 m | 0/8 | 2.0 s |

- F 에서 한 요인만 되돌렸을 때, S2 의 e-stop·충돌은 **통과 트리거**를 되돌릴 때만 생기고(B), S3 의 추월 정체는 **위험지수의 반경·가중치**를 되돌릴 때만 생긴다(C). **경로평행 연장은 두 시나리오에서 측정 가능한 효과가 없다**(A ≈ F). 연장은 모델링 정합성(§5.3.1) 때문에 유지하지만 효과는 주장하지 않는다.
- 상호작용이 있다. 셋을 모두 되돌린 ABC 에서는 S2 e-stop 이 0/8 이다. 해석(미검증): 소프트 반경·$w_r$=2 위험지수가 일찍 감속하게 만들어 늦은 트리거를 보상한다. 따라서 B 의 e-stop 은 "최종 위험지수 설정에서 트리거만 초판으로 둘 때" 의 결과로 한정해 읽어야 한다.
- ABC 는 run2 의 S2 e-stop 4/8 을 재현하지 못한다. run2 소스는 보관하지 않았고 이 세 요인 밖의 차이도 있었을 것이다. 그래서 run2 수치는 **귀속 근거로 쓰지 않는다**. 이전 판의 "연장만 고친 시험 6회 중 4회 → 트리거까지 고친 뒤 0/6", "두 변경 뒤 6/6 도달" 은 저장되지 않은 중간 실행에 근거한 문장이라 이 표로 대체했다.

**실행 이력**(모두 `checks/` 에 보관): run1 = 측정 속도 중심 창(`c06_results_run1_measuredwindow.json`) → run2 = 원호 연장·6 m 트리거·소프트 반경 위험지수(`c06_results_run2.json`, 소스 미보관) → run3 = 현 설계, 단 $T_{act}$ 2.5 s(`c06_results_run3.json`, 소스 `c06_proto_sim_run3.py`) → **run4** = $T_{act}$ 3.0 s(본문 값) + 엄격한 복귀 유지창(`c06_results.json`, 현 소스). run3→run4 에서 288 회의 충돌·e-stop·도달 판정은 하나도 바뀌지 않았다. 개별 런에서는 A6 S4 한 런의 최소 가장자리가 1.85→1.02 m, A2 S2 세 런의 복귀 시간이 ±0.3 s 바뀌었다. 끝 절단 판정은 0 회였다.

**계산 시간**: Python/numpy 구현은 부하 걸린 공유 호스트에서 주기당 p50 98.9 ms 다. C++ 예산(§5.9)과 무관하므로 실시간성 근거로 쓰지 않는다.

## 6. 평가 계획

### 6.1 측정 정의(모든 플래너에 같은 GT 기하 정의)
| 지표(명세) | 정의 | 로그 |
|---|---|---|
| 충돌(4.7) | `/world/*/pose/info`(GT, 100 Hz)로 로봇 사각형과 장애물 형상(사람 원 0.25 m, 지게차·AMR 박스)의 최소거리 ≤ 0. actor 는 물리 충돌체가 없어 접촉 센서 불가 | `[t, run_id, obs_id, edge_dist, collided]` |
| 근접(사전 등록) | 동적 장애물까지 GT 가장자리 거리 최솟값, e-stop 진입(<0.30 m) 횟수, Critical 진입(<0.50 m) 횟수 | 같은 로그 |
| 회피 창 | $t_{start}$ = 동적 장애물 GT 형상이 회랑 $C(t)$(경로 호길이 $[s_R-1,\ s_R+5]$ m, 폭 ±1.0 m + 장애물 반경)에 처음 들어간 시각 이후 $\vert e_\perp\vert >0.10$ m 가 처음 된 시각. $t_{clear}$ = $t_{start}$ 이후 모든 $s\in[t,t+3]$ s 에서 어떤 장애물 GT 형상도 $C(s)$ 와 겹치지 않는 첫 시각. **$C(s)$ 는 시각 $s$ 의 로봇 GT 위치로 다시 잡는다**(고정하면 로봇이 다가가기 전에 '해소' 로 판정된다 — 프로토타입 지표 구현에서 실제로 난 오류). 오프라인 GT 미래를 쓰고 플래너 내부 TTC 는 쓰지 않는다 | `[run_id, t_start, t_clear, t_return]` |
| 이탈 ≤ 1 m | $[t_{start},t_{return}]$ 동안 **$t_{start}$ 시점의 경로**(재계획 전 원래 경로) 기준 GT 최대 수직거리. 재계획이 다른 통로로 가면 "우회" 로 분류해 따로 보고 | 명세 CTE 포맷 |
| 복귀 ≤ 5 s | $t_{return}$ = $t_{clear}$ 이후 $\vert e_\perp\vert <0.10$ m 이고 헤딩 오차 < 10° 가 1 s 유지되는 구간의 **시작** 시각(기록이 1 s 안에 끝나면 남은 구간 전체가 조건을 만족해야 하며 "끝 절단" 으로 표시); 복귀 시간 = $t_{return}-t_{clear}$ (이탈이 0.10 m 를 넘지 않았으면 0, 정지 양보만 한 경우 "양보" 로 분류) | 같은 로그 |
| 멈춤 | 목표 미도달 상태에서 $\vert v\vert <0.05$ m/s 누적 시간, 60 s 초과 미도달 = 실패 | |
| CTE(4.5) | 장애물 없는 직선/곡선 분리, GT·추정 병기, DWA·PP 각각 | 명세 포맷 |
| 저크 | `ground_truth/odom` 50 Hz 속도를 4차 Butterworth 5 Hz 저역 후 미분, $\vert j\vert $ p99 ≤ 2.0 m/s³, RMS 보고. 안전 클램프·E-stop 구간 제외, 클램프 횟수 별도 | |
| 재계획 지연 | $T_{plan}$ = `planning_time`; $T_{e2e}$ = TTC<τ_warn 인 첫 트랙 메시지 헤더 → 새 `plan` 헤더; $T_{ctrl}$ = 새 `plan` → 그 경로로 계산한 첫 `cmd_vel_nav`; $T_{inv}$ = `invalid_streak` 시작 → 첫 비제동 유효 명령 | |
| 응답 시간(4.10) | `navigate_to_pose` 목표 스탬프 → GT 속도 > 0.01 m/s 첫 시각, 50회, 구간 분해(계획/첫 cmd_vel_nav/첫 cmd_vel) | 명세 포맷 |
| 계산 시간 | `computeVelocityCommands` 벽시계 p50/p99, 유효·가지치기·폴백 샘플 수 | `[t, dt_ms, n_valid, n_pruned, n_fallback]` |
| CPU(4.10) | S7 에서 호스트 전체(모든 노드 + Gazebo, 5대) `pidstat -u 1` 프로세스별 + `mpstat` 전체, 코어 수 명기. 다른 사용자 작업이 있는 공유 호스트에서는 기준선을 빼지 말고 **전용 머신**에서 잰다 | |
| 유령 셀(4.4) | 사람이 지나간 셀(GT 로 산출)이 로컬 costmap 에서 비워지기까지 시간 p95 | |

### 6.2 시나리오
세트 A(명세 30회): S1–S6 × 5 시드. 시드는 출발 시각(±0.5 s), 속도(±20 %), 시작점을 바꾼다. **회피가능성 규칙**: 스크립트 장애물은 로봇을 향해 돌진하지 않는다(무작위 보행자는 새 방향의 2 s 직선 예측이 로봇 현재 풋프린트+0.5 m 와 교차하면 다시 뽑는다, 지게차는 차선을 지킨다). 규칙 위반으로 정지한 로봇에 부딪힌 경우는 원시 수치와 함께 따로 보고한다.
- **S1** 직교 횡단: 사람 1.0 m/s, 개방 구역, 로봇 도착 시각에 맞춰 교차.
- **S2** 정면 지게차(재설계): 3 m 통로, 지게차 2.0×1.0 m(원 3개, 반경 0.60 m), 1.5 m/s, **차선 준수**(축이 벽에서 0.8 m). 통과 가능: 필요한 축간 거리 $d^*$ = 1.26 m(제동 꼬리 0.2 s, 저속)–1.60 m(꼬리 1 s) → 로봇 측방 0.56–0.90 m(0.8 m 초과분은 1.0 m 완화 밴드), 지게차와 가장자리 0.56–0.90 m, 벽 여유 0.74–0.40 m(c05). 즉 **통과 속도가 낮을수록 좁게 지나간다**. 측방 통과 목표는 $y_{ref}=-0.7+1.15+1.64\,\sigma_{fl}(0.5\,\text{s})=0.66$ m(§5.5). v1 의 반경 1.2 m 원 1개 모델은 통로에 0.6 m 만 남겨 어떤 δ 에서도 통과 불가였다. **S2-Y**(양보 변형, 통합 테스트 IT-09): 지게차가 통로 중앙(축 1.5 m)이면 가능한 축간 거리 ≤ 1.20 m < 최소 필요 1.26 m → 로봇은 정지·양보해야 하며, 합격 기준은 충돌 0, 정지 시 가장자리 ≥ 0.5 m, 지게차 운전자 모델(로봇 3 m 앞 정지)과 재계획으로 과업 완료. 이탈·복귀 지표는 적용하지 않는다.
- **S3** 추월: 사람 0.3 m/s 가 통로 랙 쪽(축 y = −0.9 m)을 걷는다. 필요 측방 목표 0.59 m. 사람이 y = −0.7 m 이면 목표 0.79 m > 0.75 m 라 **따라가기**가 정답(설계상 한계, §5.5·§8-10).
- **S4** 무작위 보행 2명(0.5–1.2 m/s, 1 s 마다 ±60° 방향 변경), 개방 구역.
- **S5** 곡선 지게차: 반경 4 m, 1.2 m/s, 경로를 두 번 가로지른다.
- **S6** 0.6 m 좁은 통로 입구 직전 교차 통로에서 사람 1.0 m/s 횡단.

세트 B: **S7** 전체 창고 60×40 m, 사람 3(직선·무작위) + 지게차 2(직선·곡선) + 동료 AMR 4, 속도 U(0.3, 1.5) m/s, 10회 × 10분, 충돌 0·CPU 측정. **S8** 안전 사다리: 사람이 정지·저속 로봇에 1.0 m/s 로 접근해 Warning→Critical→Stop 전이와 해제(히스테리시스 0.5 m)를 순서대로 발생시킨다.

### 6.3 기준선·절제·통계
- 기준선: (B1) Nav2 DWB(동일 한계, costmap 만), (B2) Nav2 MPPI(기본 critic), (B3) TEB(§6.4, 빌드 성공 시 `include_dynamic_obstacles` on/off), (B4) 우리 정적 DWA(§3 만), (B5) **DWB + 제동 꼬리 확률 critic**(Stage B 를 `dwb_core::TrajectoryCritic` 으로 재구현; SPAN·Coissac 형으로 기발표된 구성이라 신규성 없음 — 예측 효과와 우리 DWA 코어 효과를 분리하기 위한 비교군).
- 절제: (A1) ORCA 끔, (A2) Σ≡0(결정론 예측 — L5 재현이라 부르지 않는다), (A3) 하드 판정 끔(비용만), (A4) 이탈 밴드 끔, (A5) HP 상계만(v1), (A6) v1 식 3 s 확률 TTC·위험 비용.
- 통계: **30 쌍(시나리오×시드)을 풀링**한 양측 정확 Wilcoxon 부호순위(n=30 최소 p = 1.9×10⁻⁹; 시나리오별 n=5 는 최소 p = 0.0625 라 불가, c04), 기준선 간 Holm 보정, 효과크기(matched-pairs rank-biserial), 중앙 차이의 부트스트랩 95 % CI(10⁴). 시나리오별로는 기술통계만. **사전 등록**: 1차 지표 = 동적 장애물 GT 최소 가장자리 거리; 2차 = e-stop·Critical 진입 수, 최대 이탈, 복귀 시간, 완료 시간, 멈춤 시간.
- 합격: 세트 A·B 충돌 0, 이탈 ≤ 1 m(우회 제외), 복귀 ≤ 5 s, e-stop 진입 0(목표). 우위가 없으면 그대로 보고한다.

### 6.4 TEB 비교(명세 4.4) 와 go/no-go
- **W2 말 go/no-go**: Dockerfile 별도 스테이지에서 `teb_local_planner`(ros2-master) + `costmap_converter` 소스 빌드 후 Nav2 1.1.20 에 로드. 실패하면 W2 에 MPPI 를 "최적화 기반 비교 대상" 으로 확정하고 보고서에 "TEB 비교 미충족, 사유" 를 적는다(명세 4.4 증거 결손 명시).
- 비교 항목: 세트 A 지표 전부, 계산 시간 분포(p50/p99/최대), 국소최소·정체 횟수, 호모토피 전환 횟수(측방 통과 방향 뒤집힘), 튜닝 공수(바꾼 파라미터 수·시간), 동적 모드 유무에 따른 차이, 좁은 통로(S6) 통과율.

### 6.5 튜닝 절차(문서화 대상)
1) 장애물 없는 직선 2 m/s → $w_p$, $N_\omega$ 로 CTE; 2) 곡선 → $\ell$, $w_h$; 3) S1·S2 로 $\delta_{hard}$, $w_t$, $T_{pred}$; 4) S4 로 사람 $q$(추적기 NIS 로 보정한 값 우선); 5) S6 로 inflation·밴드; 6) S7 → CPU·상위 트랙 수. 각 단계의 전후 값을 표로 남긴다.

## 7. 구현 계획

### 7.1 패키지·파일(`src/amr_navigation`, ament_cmake, C++17)
```
amr_navigation/
  include/amr_navigation/dwa_core/          # ROS 비의존 순수 라이브러리 amr_dwa_core (단위테스트 대상)
    types.hpp  dynamic_window.hpp  rollout.hpp  brake_tail.hpp  footprint_checker.hpp
    obstacle_model.hpp (CV 예측, Σ(t), SE(2) 공분산 회전, 덮개 원)
    gvo.hpp (구간별 이차식 TTC0)  orca.hpp (C3 규약)  collision_bound.hpp (HP, BX, min, 6σ 선별)
    critics.hpp  pass_offset.hpp
  include/amr_navigation/dwa_controller.hpp   # amr_navigation::DWAController : nav2_core::Controller
  src/dwa_core/*.cpp  src/dwa_controller.cpp
  plugins/controller_plugins.xml              # DWAController (+ PurePursuitController, path-tracking 영역)
  config/nav2_params.yaml                      # components.md §6: 모든 가중치·δ·q·반경·여유 외부화
  test/ (gtest 12종, §7.4)  test/test_eval_metrics.py
  scripts/eval_local_planner.py              # 시나리오 실행·GT 로그·표·통계
```
CMake: `add_library(amr_dwa_core)`(`-Wall -Wextra -Wpedantic` 경고 0), `add_library(dwa_controller SHARED)`, `pluginlib_export_plugin_description_file(nav2_core plugins/controller_plugins.xml)`, `ament_add_gtest`. package.xml 추가: `pluginlib, nav2_util, angles, tf2, tf2_geometry_msgs, visualization_msgs, dwb_msgs`(디버그 `Trajectory2D`), test `ament_cmake_gtest`(components.md "설계상 필요하지만 없는 의존" 목록에 `pluginlib` 이미 있음).

### 7.2 ROS 인터페이스(components.md §5.3 과 일치)
- 등록: `PLUGINLIB_EXPORT_CLASS(amr_navigation::DWAController, nav2_core::Controller)`; `controller_server: controller_plugins: ["DWA", "PurePursuit"]`, `DWA.plugin: "amr_navigation::DWAController"`, `failure_tolerance: 1.0`, 진행 검사기 0.3 m / 20 s.
- 구독(플러그인 내부, 부모 노드로 생성): `perception/tracked_obstacles`(`amr_msgs/msg/TrackedObstacleArray`, map), `safety/zone`(`std_msgs/msg/UInt8`), `safety/estop_active`(`std_msgs/msg/Bool`), `payload/mass`(`std_msgs/msg/Float32`, 가속 한계 보정 — components.md 기존 항목), 동료 상태 `/amr_XX/robot_state`(`amr_msgs/msg/RobotState`, 자기 제외 4대). **components.md 갱신 제안**: `safety/zone`·`safety/estop_active`·`/amr_XX/robot_state` 의 새 소비자로 `controller_server` 추가, §3 의 DWAController 설명("예측 위치 → clearance 비용, VO → 샘플 제외")을 "사람·지게차 GVO-TTC → 비용, 샘플 제외 = 동료 ORCA + 제동 꼬리 기회제약" 으로 수정. TF: map→odom.
- 발행: `cmd_vel_nav`(controller_server 가 발행), `~/dwa/status`(avoidance_active, TTC0_min, n_valid, n_pruned, fallback; 대시보드·로그용 — 메시지 타입은 `diagnostic_msgs/DiagnosticArray` 로 시작), `~/dwa/best_trajectory`(`dwb_msgs/Trajectory2D`), `~/dwa/candidates`(`visualization_msgs/MarkerArray`), 기존 `local_plan`.
- 파라미터(`src/amr_navigation/config/nav2_params.yaml`, `DWA.*`): `vx_samples 11, vtheta_samples 31, sim_dt 0.1, sim_time_min/max 1.5/2.5, pred_time 5.0, commit_time 0.2, delta_hard 0.05, risk_time 1.5, margin_hard 0.30, margin_soft 0.50, t_act 3.0, band 0.8, band_relax 1.0, orca_tau 2.0, orca_max_half_turn 0.3, track_max_range 18.0, max_tracks 5, track_timeout 0.3, track_hold 0.5, class_table {person:{r:0.25,q:0.2,circles:[0]}, forklift:{r:0.61,q:0.1,circles:[-0.667,0,0.667]}, amr:{r:0.25,q:0.05,circles:[-0.15,0.15]}}, weights {...}`. `track_max_range` 는 $T_{pred}\times3.5$ m/s(최대 접근 속도) = 17.5 m 이상이어야 5 s 시야가 의미가 있다(이전 판의 6 m 는 이를 무력화했다; 프로토타입은 15 m). `max_tracks` 는 거리순 상위. 한계값(속도·가속·저크)과 안전 거리·구역 상한은 `config/robot_params.yaml` 에서 읽고 복제하지 않는다.

### 7.3 메시지 확장(perception 영역과 합의 필요)
`amr_msgs/TrackedObstacle` 에 추가: `float32[4] position_covariance`(2×2 row-major, m²), `float32[4] velocity_covariance`(m²/s²), `float32[4] position_velocity_covariance`(m²/s), `float32 radius`(m), `uint8 object_class`(UNKNOWN/PERSON/FORKLIFT/AMR/BOX). perception-tracking 브리프는 `TrackedObject{pose: PoseWithCovariance, twist: TwistWithCovariance, class_probs…}` 를 `perception/tracks` 로 제안하므로 두 안 중 하나로 통일해야 한다(§8-4). 공분산이 없으면 클래스 폴백($\sigma_p$=0.05, $\sigma_v$=0.2)과 1회 경고.

### 7.4 단위테스트(gtest, 커버리지 ≥ 70 %)
1) 동적창: 한계·가속 제약, **제동 후보 ∈ $V_d$**(|ω_a|=1.0 에서 ω 변화 ≤ 0.10), 회전 후보는 $|v_a|\le0.05$ 에서만; 2) 롤아웃: $\omega\to0$ 연속성, 원호 종점 해석해 1e-9; 3) 제동 꼬리: 저크 포함 정지거리 표(§3.2) ±1 mm, $a_0>0$ 단조성; 4) 풋프린트 3단계(합성 costmap); 5) **상계**: 등방·이방 Σ 무작위 60례에서 $\min(P_{HP},P_{BX})\ge$ MC − 4 SE, 차이 표 출력(차이 < 0.03 같은 단정 없음), $P_{BX}\le P_{HP}$(등방), 6σ 선별 오차 < 1e-9; 6) GVO: 정면 접근 $t_1=(\|r_0\|-R)/\|w\|$, 구간 이차식 = 조밀 샘플링(1e-3 s) 결과 ±1 ms; 7) **ORCA**: 대칭 정면 대면에서 $\mathbf u_A=-\mathbf u_B$, $\mathbf n_A\cdot\mathbf n_B=-1$, ½ 적용 후 τ 동안 최소거리 ≥ $R_o-10^{-6}$, v1 규약 회귀 방지; 8) **오가지치기율**: 무작위 1만 장면에서 ORCA 대리 속도 판정 vs 롤아웃 포즈별 진실, |ω|τ/2≤0.3 에서 오가지치기 < 1 % 보고; 9) **복귀 시간**: 밴드 경계(0.8 m)·v=0.5 m/s 에서 폐루프 운동학 시뮬 복귀 ≤ 5 s; 10) 비용: 정규화 범위($J_{vel}$ 후진 포함 [0,1]), 가중치 0 무영향, $J_{risk}$ 비포화(트랙 1개 max 0.02 → 0.02); 11) 공분산 SE(2) 회전: 90° 회전에서 대각 교환; 12) 타이밍: 342 샘플 × 트랙 5 p99 < 25 ms(CI 회귀선). pytest: GT 기반 $t_{start}/t_{clear}/t_{return}$ 합성 로그, Wilcoxon 풀링 스크립트.

### 7.5 통합 테스트(launch_testing + pytest, 헤드리스 Gazebo, CI 자동화, ≥10)
| ID | 내용 | 합격 기준 |
|---|---|---|
| IT-01 | 수명주기·플러그인 로드, 빈 직선 경로 FollowPath | 목표 도달, `controller_id` DWA/PurePursuit 둘 다 |
| IT-02 | 0.6 m 통로(정적) | 충돌 0, e-stop 0(safety_node 정적 제외 전제) |
| IT-03 | S1 1.0 m/s 횡단 | 충돌 0, e-stop 0 |
| IT-04 | 통로 봉쇄 → 재계획 | $T_{plan}\le500$ ms, 새 경로 추종, abort 0 |
| IT-05 | `perception/tracked_obstacles` 중단 | 0.8 s 안에 $v\le0.2$ m/s, 재개 시 복귀 |
| IT-06 | LiDAR 토픽 중단 | 0.3 s 타임아웃 후 정지, 플래너 예외 0 |
| IT-07 | 대시보드 E-stop | `cmd_vel`=0 ≤ 20 ms, 래치·`safety/reset_estop` 해제 |
| IT-08 | S8 안전 사다리 | 존 1→2→3 전이 순서, 구역 상한 준수, 해제 |
| IT-09 | S2-Y 양보·교착 | 충돌 0, 정지 가장자리 ≥ 0.5 m, 재계획으로 과업 완료, 기본 복구(Spin/ClearLocal) 미사용 |
| IT-10 | 동료 AMR 2대 정면(ORCA) | 충돌 0, 교착 0, 양쪽 도달 |
| IT-11 | S7 전체 창고 10분 | 충돌 0, 호스트 CPU ≤ 80 % |
| IT-12 | 컨트롤러 전환(DWA↔PurePursuit) | 전환 시 저크 p99 ≤ 2.0 m/s³, 정지 없음 |

### 7.6 일정(주, 1–2인)
W1 `amr_dwa_core`(§3, 제동 꼬리·상계·GVO·ORCA) + gtest 1–11 → **W2 플러그인·파라미터·DWB 정적 비교 + TEB 빌드 go/no-go** → W3 트랙 구독·Stage A/B/C·프로토타입 결과와 대조 → W4 안전층 연동·자체 BT 요구 반영·IT-01…IT-12 자동화 → W5 세트 A/B 평가·튜닝 → W6 절제·통계·리포트(동료평가 대응 문서 `docs/algorithms/dwa.md`).

## 8. 리스크와 열린 질문
1. **TEB 빌드**(§6.4): W2 go/no-go. 실패 시 명세 4.4 증거 결손을 명시.
2. **안전 존과 0.6 m 통로의 충돌**: components.md 의 safety_node 는 `scan_filtered` 전체로 존을 판정한다. 0.6 m 통로에서 벽 가장자리 거리는 0.10 m 라 e-stop(0.30 m)이 걸려 명세 4.4(좁은 통로)와 4.7(0.3 m 정지)이 양립하지 않는다. 존 판정에서 정적 지도 구조를 빼거나(맵 마스킹) 전방 보호영역으로 바꾸는 결정이 안전 영역에 필요하다(global-planning 브리프도 같은 지적). 결정 전까지 S6/IT-02 는 이 전제로만 합격 가능하다.
3. **actor 충돌 물리 부재**: 충돌은 GT 기하로만 센다. sequences.md §2 "Gazebo 접촉 이벤트 카운트" 를 GT 기하로 고쳐야 한다(문서 담당에 전달).
4. **트래커 공분산·클래스 필드**(§7.3): 없으면 폴백 σ 가 보수적(σ_v 0.2 대 실제 추정 ≈0.1)이라 추월 여유가 커진다(사람 1.2 s: σ 0.42 → 0.36 m).
5. **동료 AMR 상태 구독**: `/amr_XX/robot_state` 는 2 Hz + 0–100 ms 지연이라 연관 게이트가 1.5 m 로 크다. 클래스 `amr` 인식이 우선.
6. **브리프 간 정합**: path-tracking 브리프의 합성 `TrackingController`(PP 명목 + DWA)와 로컬 costmap 6×6 m 제안 ↔ 본 브리프의 `DWA`/`PurePursuit` 분리와 12×12 m. 컨트롤러 소유권과 costmap 크기를 한 번에 정해야 한다.
7. $\alpha_\omega=2.0$ rad/s² 는 명세에 없는 팀 설정 — DiffDrive 플러그인 한계와 일치 확인.
8. **특허**(P3): 동료 ORCA 비용/가지치기는 Locus 특허 청구와 겹칠 수 있음 — 연구용 한정, 제품화 시 검토.
9. **긴 시야 평균 예측의 오판**: 곡선 지게차는 5 s CV 예측이 크게 틀린다. 하드 판정은 짧은 꼬리만 쓰므로 안전에는 영향이 작지만 불필요한 회피(비용)는 생길 수 있다 → S5 에서 측정.
10. **추월 가능 폭**: 3 m 통로 중앙 근처의 사람은 추월하지 않고 따라간다(설계상). 과업 지연은 진행 저하 재계획으로 처리하며, 허용 여부는 운영 정책 결정 사항이다.
11. 저크 무시 여유거리식(`robot_params.yaml` 주석)과 플래너의 저크 포함 정지거리 차이(2 m/s 에서 0.49 m)는 경로가 달라서(안전 노드 = 프로파일러 우회) 모순은 아니지만, 설정 주석에 두 경로를 구분해 적어 두기를 제안한다.

## 9. 참고문헌(URL/DOI)
- P1 https://arxiv.org/abs/2011.06235 · P2 https://www.diva-portal.org/ (diva2:1836829), https://github.com/FloCoicoi/fc_thesis · P3 https://patents.google.com/patent/US10429847B2 · P4 ICRA 2007 pp.1986–1991 (http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/IEEE_ICRA_2007/data/papers/1765.pdf) · P5 https://doi.org/10.1016/j.robot.2019.05.003 · P6 IROS 2009, DOI 10.1109/IROS.2009.5354175 (http://gamma-web.iacs.umd.edu/NHRVO/WilkieIROS09.pdf) · P7 https://doi.org/10.1109/LRA.2023.3257681
- L1 https://arxiv.org/abs/2310.02648 · L2 …/2504.03260 · L3 …/2404.13678 · L4 …/2603.02291 · L5 https://doi.org/10.1109/ICRA.2019.8794386 · L6 …/2109.07775 · L7 …/2509.07239 · L8 …/2503.21141 · L9 …/2510.01402 · L10 …/2506.06255 · L11 …/2503.00606 · L12 …/2403.10043 · L13 …/2407.00507 · L14 …/2501.09649 · L15 …/2506.21205 · L16 …/2507.20293 · L17 …/2401.06021 · L18 https://doi.org/10.1177/02783649251315203 (arXiv 2307.01070) · L19 …/2502.15525 · L20 …/2511.18170 · L21 …/2409.11962 · L22 …/2607.09192 · L23 …/2303.10133 · L24 …/2606.02677 · L25 …/2512.24651 · L26 …/2510.26142 · L27 …/2307.15236 (… = https://arxiv.org/abs)
- C1 IEEE RAM 4(1) 1997 · C2 IJRR 17(7) 1998 · C3 ISRR 2011 · C4 https://la.disneyresearch.com/wp-content/uploads/alonsomora10dars_paper1.pdf · C5 2004 · C6 RA-L 4(2) 2019 · C7 RAS 88 2017 · C8 ICRA 2017 · C9 Advanced Robotics 18(10) 2004
- Nav2/Gazebo: controller_server.cpp (humble) https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp · DWB/MPPI/Collision Monitor README (humble 브랜치) · TEB 배포현황 https://index.ros.org/p/teb_local_planner/ · gz-sim actor 충돌 https://github.com/gazebosim/gz-sim/issues/1364

## 10. 리뷰 반영 이력
리뷰(`critique.json`, 점수 5) 항목 55개: 수학 오류 12(M), 제안 판정 3(V), 인용 문제 14(CP), 명세 공백 11(G), 필수 수정 15(R). 상태: **반영** = 지적대로 고침, **반영(대안)** = 문제는 인정하되 다른 방법으로 해결(근거 병기), **부분 이견** = 일부 수치·해석에 동의하지 않음(근거 병기). 55개 모두 문서 안에 설계 답이 있고, 외부 결정에 걸린 것은 §8 로 넘겼다.

| ID | 리뷰 지적(요지) | 조치·근거 | 위치 | 상태 |
|---|---|---|---|---|
| M1 | ORCA 반평면이 $w$ 공간/로봇 속도를 섞어 VO 안으로 민다 | C3 규약($\mathbf v_{rel}=\mathbf v_A-\mathbf v_B$)으로 재정의. c03: 대칭 정면 대면에서 $\mathbf u_A=-\mathbf u_B$, $\mathbf n_A\cdot\mathbf n_B=-1$, ½ 적용 후 최소거리 = $R_o$; v1 규약은 $\mathbf v_A'=(2.82,0.01)$ 로 VO 안 — 지적 확인. gtest 7 추가 | §5.3.2, §7.4 | 반영 |
| M2 | 현 속도 사상은 필요·충분 조건이 아님, σ_j 스칼라 미정의 | 사람·지게차는 롤아웃 포즈별 GVO(구간 이차식)로 교체. 현 속도는 동료 ORCA 대리 속도로만, $\vert \omega\vert \tau/2\le0.3$ + 새지타 팽창. c03: v1 현 VO 는 호 충돌 24.6 % 누락, 자유 샘플 0.9 % 오가지치기(새지타 = 최대 편차 0.245/0.460/0.620 m 확인). 스칼라 σ 사용처 제거(Stage B 는 2×2 Σ 전체 사용) | §5.3 | 반영 |
| M3 | 상계가 느슨하고(0.272 vs MC 0.132) δ_hard=0.10 이 S2 를 얼린다, 테스트 5 불성립 | 정확값 0.133 으로 확인. $\bar P=\min(P_{HP},P_{BX})$ 로 0.168. 하드 판정을 제동 꼬리(≤2.4 s)로 한정, 여유 0.30 m, δ=0.05. 여유거리 표 재계산. 테스트 5 를 "상계 ≥ MC − 4SE + 차이 표" 로 교체(이방 60례 위반 0, 최대 차이 0.121). S2 재설계(R7) | §5.4, §6.2, §7.4 | 반영 |
| M4 | $J_{risk}$ 곱이 포화, L18 귀속 부정확 | $1-\prod_j(1-\max_{t\le1.5}\bar P)$ 로 바꾸고 "지수" 로 명명(확률 상계 아님을 명시). 0.994 → 단일 트랙 0.02. L18 귀속 철회 | §5.5, §4.2 | 반영 |
| M5 | $T_{ttc,min}=2.5\ge T_{sim}$ 이라 공허, 저속에서 예측 시야 1.5 s 로 붕괴 | 시야 분리: 평균 예측 GVO $T_{pred}=5$ s(샘플 속도 무관), $T_{act}=3.0$ s $<T_{pred}$. 리뷰 제안(3 s 제동 꼬리 연장)과 달리 **확률은 제동 꼬리에만, 긴 시야는 결정론** — c06 에서 3 s 확률 비용(A6)이 추종 정체를 만들었기 때문(S3 도달 0/8, v2 8/8) | §3.3, §5.3.1, §5.10 | 반영(대안) |
| M6 | 제동·회전 후보가 $V_d$ 밖 | $u_b=(\max(v_a-0.05,0),\ \omega_a-\mathrm{sgn}\,\omega_a\min(\vert \omega_a\vert ,0.10))$, 회전은 $\vert v_a\vert \le0.05$ 에서만, 매 주기 α_ω 적용. gtest 1 | §3.3 | 반영 |
| M7 | $N_t\le30$ 과 25 혼용 | $T_{sim}$ 상한 2.5 s, $N_s\le25$, 셀 접근 3.42×10⁵(c01) | §3.3, §3.6 | 반영 |
| M8 | 사람 σ(2 s)=0.83 m(0.66 은 q=0.1) | 표로 정정(0.834/0.655/0.544), 여유거리 논의 반영 | §5.2 | 반영 |
| M9 | 복귀 추정 2–3 s 가 낙관, 스케줄 2 s 짧음 | c01 운동학 하한(방향각 뱅뱅, 최적 각도 탐색): e0=0.8 m 에서 v=0.5/1.0/1.5 → 3.04/2.33/2.02 s. 저속 낙관은 인정. 리뷰의 ≈4.6 s 는 선회 중 횡이동을 빼고, 1 s 유지창(복귀 **시작** 시각을 확정하는 조건)을 더한 값이라 과대. 스케줄을 "e<0.1 m 또는 5 s" 로 연장, 밴드 경계 저속 복귀 테스트 추가. 프로토타입 복귀 시간은 중앙값 2.33 s·최대 3.89 s(S3)로 5 s 안이지만 여유가 1.1 s 뿐임을 명시 | §5.5, §6.1, §7.4-9, §5.10 | 부분 이견 |
| M10 | n=5 Wilcoxon 은 최소 p=0.0625 | c04 확인. 30쌍 풀링 양측 정확 검정(최소 1.9e-9) + Holm + 효과크기 + 부트스트랩 CI, 시나리오별은 기술통계 | §6.3 | 반영 |
| M11 | $J_{vel}$ 이 후진에서 1.25 | $\vert v_{des}-v\vert /(v_{max}-v_{min})\in[0,1]$(c01) | §3.5 | 반영 |
| M12 | map→odom 에서 공분산 회전 누락 | 평균·속도·세 공분산 블록 모두 $R\Sigma R^\top$, gtest 11 | §5.2 | 반영 |
| V1 | 전체 플러그인 "flawed"(얼림 기본값, 포화, 공허 임계, ORCA 부호, 후보, 예외 경로, 선행 누락) | 각 결함을 M1–M6·R8 로 수정, SPAN/Coissac/Locus 인용으로 재정위, 프로토타입으로 기본값 검증(§5.10: 48 회 충돌·e-stop 0, 멈춤 최대 0.6 s), TEB W2 | §5 전체 | 반영 |
| V2 | Stage B 단독은 SPAN 으로 기발표 | 인정. 신규성 주장 삭제, "제동 꼬리에 한정한 시간 분할" 만 공학적 차이로 남기고 시험 가능하게(A6 대비) 둠. DWB critic 이식은 재구현 비교군으로만 | §4.1, §5.8 | 반영 |
| V3 | Stage A 단독은 Locus 특허·Coissac·GVO 로 기발표, 20–60 % 근거 없음 | 인정. 현 속도 VO 가지치기는 동료 ORCA 로 축소, 사람·지게차는 GVO. "20–60 %" 삭제(가지치기 수는 로그로만 측정). 특허 유의 추가 | §5.3, §5.8, §8-8 | 반영 |
| CP1 | SPAN 누락 | P1 추가, arXiv PDF 본문에서 TTC 정의·식 (13)(14)·ε 사용·유니사이클 한계 확인 | §4.1 | 반영 |
| CP2 | Locus 특허 누락 + IP | P3 추가(Google Patents 에서 제목·양수인·일자·청구 문구 확인), IP 유의 | §4.1, §8-8 | 반영 |
| CP3 | Coissac 논문 누락, 공백 (ii) 거짓 | P2 추가(전문 확인: AdaptedVO critic, 식 4.6), 공백 (ii) 삭제 | §4.1, §4.3 | 반영 |
| CP4 | Seder & Petrović, Molinos et al. 누락 | P4(학회 PDF 확인), P5(서지·초록 확인) 추가 | §4.1 | 반영 |
| CP5 | Wilkie GVO, Yasuda 누락 | P6(저자 PDF 확인, Stage A 정의로 채택), P7(Crossref 서지만 — 내용 비교 주장 안 함) | §4.1 | 반영 |
| CP6 | 공백 (i) 은 좁은 의미에서만 참 | "Nav2 Humble 플러그인 형태의 공개 구현을 못 찾음" 으로 축소 | §4.3 | 반영 |
| CP7 | L5 URL 이 디렉터리 페이지 | WebFetch 로 확인, DOI 10.1109/ICRA.2019.8794386(Crossref: pp. 8620–8626)로 교체, "Σ≡0 = L5 재현" 삭제 | §4.2, §6.3 | 반영 |
| CP8 | L24 주장이 초록에 없음 | 초록 표현("additional classical local planning approaches") 인용, 기존 문장은 우리 해석으로 표시 | §4.2 | 반영 |
| CP9 | L4 "정적 회피" 미확인 | 초록 재확인 — 장애물 성격 불명, 표현 삭제 | §4.2 | 반영 |
| CP10 | L18 IJRR DOI, $J_{risk}$ 귀속 | Crossref: IJRR 44(9) 2025, DOI 추가, 귀속 철회 | §4.2 | 반영 |
| CP11 | velocity_smoother 에 저크 없음 | 컨테이너 `.so` 파라미터 문자열로 확인. 프로젝트 체인은 `velocity_profiler_node`(저크 2.0)로 components.md 에 이미 정의 → 문서 체인 교체 | §2, §5.6.1 | 반영 |
| CP12 | "예외 → 즉시 재계획" 은 Humble 에서 abort → 복구 | humble `controller_server.cpp`(failure_tolerance 기본 0.0, 처리 분기)와 BT XML 을 직접 확인해 서술 교체 | §2, §5.6.2 | 반영 |
| CP13 | DWB 예외 클래스명 | `IllegalTrajectoryException`, `PlannerTFException`(헤더 확인) | §2 | 반영 |
| CP14 | 확인된 인용 목록(L1, L2, L6, L9, L14–L16, L20, L22, L25, C4, C6, 스택 사실) | 유지. 조치 불필요 | §4.2, §2 | 반영(변경 없음) |
| G1 | 통합 테스트 ≥10 매핑 없음 | IT-01…IT-12(센서 고장·E-stop·재계획·양보·ORCA·전체 창고 포함), §1 에 매핑 | §7.5, §1 | 반영 |
| G2 | ≥5 동시 동적 장애물 전체 창고 시나리오 없음 | S7(사람 3 + 지게차 2 + AMR 4, 0.3–1.5 m/s, 10회×10분), IT-11 | §6.2 | 반영 |
| G3 | PP/Stanley 와 DWA 의 공존·CTE 측정 대상 불명 | `controller_id` DWA/PurePursuit 배정 규칙, 두 컨트롤러 모두 CTE 측정, GT/추정 병기와 위치오차 예산. path-tracking 브리프와의 최종 소유권은 §8-6 | §5.6.4 | 반영 |
| G4 | 저크 2.0 m/s³ 가 어디서도 집행 안 됨 | `velocity_profiler_node` 가 집행(components.md), 플래너는 같은 저크로 정지거리·꼬리 계산, 합격선 p99 ≤ 2.0(5 Hz 필터), 클램프 제외·계수 | §3.2, §6.1 | 반영 |
| G5 | 센서 고장 감지 로직·주체·안전정지 분기 미정 | 주체 safety_node, 설정 타임아웃·분기(LiDAR/엔코더 정지, IMU/카메라 0.2 m/s), 플래너의 트랙 노후 규칙(0.3/0.5 s → 0.2 m/s), IT-05/06 | §5.6.1 | 반영 |
| G6 | Warning→Critical→E-stop 중재·시험 없음 | 우선순위·존 인지 속도창·여유 정렬(0.30/0.50)·예견 구역 속도·진행 검사기 설정, S8/IT-08 | §5.6.1, §5.5 | 반영 |
| G7 | TEB 비교 항목·빌드 시점 | 항목표와 W2 go/no-go, 실패 시 증거 결손 명시 | §6.4, §7.6 | 반영 |
| G8 | 이탈·복귀 정의가 플래너 내부 TTC 의존 | GT 기하 회랑·오프라인 GT 미래로 $t_{start}/t_{clear}/t_{return}$ 정의, 원래 경로 고정, 우회 분류 | §6.1 | 반영 |
| G9 | 예외 경로는 복구, 전역 obstacle layer 설정 없음 | 예외 미사용·failure_tolerance·자체 BT 요구, 전역 `observation_persistence 0.0`/5 Hz, TTC 즉시 재계획 경로 | §5.6.2 | 반영 |
| G10 | 예외/제동 → 새 유효 명령 지연 미측정 | $T_{plan}, T_{e2e}, T_{ctrl}, T_{inv}$ 정의 | §6.1 | 반영 |
| G11 | 호스트 전체 CPU 계획 없음, p99 15 ms 가 5–8 ms 와 불일치 | S7 호스트 전체 측정(전용 머신), 계산 예산 재산정(최악 수치 공개, 트랙 상위 5·6σ 선별), 설계 목표 p99 ≤ 10 ms vs CI 회귀선 25 ms 근거 | §5.9, §6.1 | 반영 |
| R1 | 신규성 재정위 | §0-2·3, §4.3, §5.8 | — | 반영 |
| R2 | ORCA 수정 + σ_j 정의 | M1, M2 참조 | §5.3.2 | 반영 |
| R3 | Stage A 주장 교체, 오가지치기율 | GVO 채택, 동료 ORCA 제한, 오가지치기 테스트(gtest 8), 20–60 % 삭제 | §5.3, §7.4 | 반영 |
| R4 | Stage B 기본값 수리(상계 조이기, σ 상한, 시간 의존 δ, $J_{risk}$, 여유표) | 상계 min, 하드는 제동 꼬리(시간 의존이 구조적으로 들어감), $J_{risk}$ 교체, 표 공개. **σ 상한은 채택 안 함**: 공분산을 자르면 "상계" 라는 진술이 거짓이 되므로, 대신 확률 판정의 시야를 줄였다. 상계 조임(min)의 폐루프 이점은 프로토타입에서 드러나지 않았다(A5 와 최소 가장자리 차이 −0.013 m) — Gazebo A5 로 재판정 | §5.4, §5.5 | 반영(대안) |
| R5 | 시야 분리, 공허 임계 제거 | M5 참조 | §3.3, §5.3.1 | 반영(대안) |
| R6 | 후보·$N_t$·$J_{vel}$·σ(2 s)·공분산 회전 | M6, M7, M11, M8, M12 참조 | §3, §5.2 | 반영 |
| R7 | S2 재설계 | 지게차 3원 덮개(2.0×1.0), 차선 준수로 통과 가능(c05 기하), 중앙 주행은 S2-Y 양보 시험(고유 합격 기준, 이탈·복귀 비적용) | §6.2 | 반영 |
| R8 | PlannerException 재계획 교체 | 예외 미사용·failure_tolerance 1.0·진행 검사기 20 s·자체 BT(Spin/ClearLocal 금지)·지연 측정 | §5.6.2 | 반영 |
| R9 | velocity_smoother 주장 정정·저크 집행 | CP11, G4 참조 | §2, §6.1 | 반영 |
| R10 | GT 기반 §6.1 | G8 참조 | §6.1 | 반영 |
| R11 | 통계 수정 | M10 참조, 1·2차 지표 사전 등록 | §6.3 | 반영 |
| R12 | 테스트 5·오가지치기 테스트 | gtest 5, 8 | §7.4 | 반영 |
| R13 | 명세 공백 일괄 | G1–G11 | §1, §6, §7 | 반영 |
| R14 | TEB go/no-go 를 W2 로 | §6.4, §7.6 | — | 반영 |
| R15 | L5/L24/L4/L18 | CP7–CP10 | §4.2 | 반영 |

**리뷰 밖에서 이번에 추가로 찾아 고친 것**: (X1) obstacle layer 파라미터 `observation_keep_time` → Nav2 이름 `observation_persistence`(v1 이름은 무시됨); (X2) 존 대응값을 설정과 일치(Warning 0.5 m/s·Critical 0.2 m/s 절대 상한, 센서 열화 0.2 m/s; v1 은 50 %/×0.3/30 %); (X3) 플러그인·토픽·체인 이름을 components.md 와 일치(`DWAController`, `perception/tracked_obstacles`, `velocity_profiler_node`/`safety_node`, `/amr_XX/robot_state`); (X4) 저크 포함 정지거리와 설정 주석식의 경로 차이 문서화(§3.2, §8-11); (X5) 계산 예산의 erfc 30 ns 가정이 근거 없었음을 밝히고 재산정(§5.9); (X6) 0.6 m 통로와 0.3 m e-stop 의 양립 불가 조건을 위험으로 명시(§8-2); (X7) 모든 샘플이 하드 판정을 어길 때의 탈출 규칙을 최소 위험 폴백(사전식)으로 정함(§5.4; 초기 판 규칙의 e-stop 관찰은 기록 미보관이라 근거로 쓰지 않음); (X8) 측정 속도 중심 동적창 + 저크 제한 프로파일러 = 실효 가속 ≈1/3 → 명령 중심 창(§3.2, c08 가속 점검); (X9) $\max_k c(\mathbf p_k)$ 형 $J_{clear}$ 가 좁은 통로 입구에서 정지 국소최소를 만듦 → 참조 경로 대비 초과 비용(§3.5; c08b 에서 S6 0/8 도달로 재현); (X10) 5 s 일정 곡률 원호가 횡탈출을 과대평가 → $T_{sim}$ 뒤 경로평행 연장(§5.3.1; 모델링 정합성 때문이며 c08 에서 S2·S3 효과는 측정되지 않음); (X11) 위험지수 반경·가중치가 추월을 막음 → $R^h$, $w_r=1.0$(§5.5; c08 에서 되돌리면 S3 0/8 도달); (X12) GT 회랑을 고정 위치로 두면 $t_{clear}$ 가 너무 이르게 나옴 → 로봇과 함께 이동(§6.1). (X13) 프로토타입의 활성화 임계 $T_{act}$ 가 2.5 s 로 본문(3.0 s = τ_warn)과 달랐음 → 3.0 s 로 전체 재실행(run4, §5.10); (X14) 프로토타입 복귀 지표가 기록 끝 1 s 안에서는 첫 표본만 보고 복귀로 판정하던 결함 → 남은 구간 전체 조건으로 엄격화하고 "끝 절단" 표시(§6.1, c06); (X15) `c02b` 의 꼬리 시간 출력이 계산이 아닌 하드코딩 문자열이었고 값도 $t_r=0.15$ s 기준(0.90/1.40/2.40 s)인데 "T_c 포함" 으로 표기돼 있었음 → 수치 적분으로 교체, 본문의 0.95/1.45/2.45 s 확인; (X16) 지게차 3원 덮개의 정확 반경은 0.601 m(0.60 m 는 0.9 mm 부족) → 구현 파라미터 0.61 m(§5.2, §7.2); (X17) 파라미터 `track_max_range 6.0` 이 5 s 예측 시야(최대 접근 17.5 m)와 모순 → 18 m(§7.2); (X18) 가속 점검 수치를 저장된 스크립트로 재현(c08: 측정 중심 1.95 m/s 도달 7.0 s, 명령 중심 2.2 s; 이전 판의 "약 8 s" 정정, §3.2); (X19) S2·S3 수정의 귀속 문장("연장만 고친 시험 6회 중 4회 → 0/6", "두 변경 뒤 6/6 도달")이 저장되지 않은 중간 실행에 근거해 있었음 → c08 한 요인씩 절제로 대체: S2 e-stop 은 통과 트리거, S3 정체는 위험지수 설정이 결정, 연장 모델은 효과 없음(§5.3.1, §5.5, §5.10); (X20) 문서의 제동 꼬리 시간을 속도 비례로 서술했던 것 정정(시간 ≈ $v+0.45$ s, 거리는 속도 제곱에 가까움, §5.1).

## 부록 A. 검증 스크립트(`research/local-planning/checks/`)
| 파일 | 내용 | 주요 출력 |
|---|---|---|
| `c01_window_horizons.py` | $V_d$, $T_{sim}$/$N_s$, 셀 접근 수, $J_{vel}$ 범위, 저크 포함 정지거리, 허용 속도, 복귀 운동학 하한 | §3.2 표, $N_s\le25$, 3.43×10⁵, 복귀 2.0–3.5 s |
| `c02_covariance_bounds.py` | σ(t), HP/BX/정확/MC, 이방 60례, 여유거리, $J_{risk}$ 포화 | §5.2·§5.4 표 |
| `c02b_clearance_v2.py` | v2 반경·여유의 $d^*$, 추월 목표, 제동 꼬리 시간(수치 적분) | §5.4 표, 0.59/0.79 m, 0.95/1.45/1.95/2.45 s |
| `c03_vo_orca.py` | 새지타, 현 VO 누락·오가지치기율, ORCA 대칭·부호 | 24.6 % / 0.9 %, $\mathbf u_A=-\mathbf u_B$ |
| `c04_stats.py` | Wilcoxon 최소 p | n=5 0.0625, n=8 0.0078, n=30 1.9e-9 |
| `c05_geometry.py` | 덮개 원(로봇 0.25 m, 지게차 3원 0.601 m), S2/S3 기하, 통로 헤딩 허용, inflation 비용 | §3.4, §5.2, §6.2 |
| `c06_proto_sim.py` | 운동학 프로토타입(변형 6종 × S1–S6 × 8 시드), 현 소스 = run4($T_{act}$ 3.0 s, 엄격한 복귀 유지창, `PASS_TRIGGER` 스위치) | §5.10 |
| `c06_results.json`, `c06_summary.txt`, `c06_table.py` → `c06_table_out.md` | run4 결과(288 회), 요약, 표·48 쌍 Wilcoxon | §5.10 표 |
| `c06_results_run3.json`, `c06_summary_run3.txt`, `c06_proto_sim_run3.py` | run3($T_{act}$ 2.5 s) 결과와 그 소스 | §5.10 실행 이력 |
| `c06_results_run1_measuredwindow.json`, `c06_summary_run1_measuredwindow.txt`, `c06_results_run2.json`, `c06_summary_run2.txt` | run1(측정 중심 창)·run2(원호 연장·6 m 트리거·소프트 위험지수) 결과. **소스 미보관** — 참고용, 귀속 근거로 쓰지 않음 | §5.10 실행 이력 |
| `dbg1.json`…`dbg5.json`, `dbg_s3.py`, `quick.py` | 초기 판의 단일 시드 디버그 기록(소스 미보관), 계획기 상태 추적 도구, pvt 빠른 실행 도구 | 참고용 |
| `c08_attrib.py`, `c08_results.json`, `c08_out.txt` | S2·S3 수정의 한 요인씩 귀속(F/A/B/C/ABC × 8 시드), 창 중심별 가속 점검 | §3.2, §5.3.1, §5.5, §5.10 |
| `c08b_clear.py`, `c08b_results.json`, `c08b_out.txt` | 절대 $J_{clear}$ 로 되돌린 S6(8 시드, 동적 장애물 유무) | §3.5 |
| `c07_bench.cpp`, `c07_out.txt` | 상계·거리 계산 비용(컨테이너 g++ -O2, 3회) | §5.9 |
| `check/controller_server_humble.cpp`, `check/span.txt`, `check/coissac.txt`, `check/seder.txt`, `check/wilkie.txt` | 소스·원문 사본 | §2, §4 |
