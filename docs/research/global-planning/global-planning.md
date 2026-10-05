# 글로벌 경로 계획 설계 브리프 — A\*, 평활화, NavFn/Smac 비교 (slug: `global-planning`)

작성일 2026-09-21, 개정 2026-09-22(리뷰 반영 + 감사 보정 §9.1 + 2차 감사 §9.2) · 대상: 명세 4장-4 "경로 계획 시스템 / Global Planner", 4장-5·7·9·10 연계 항목, 7장 제약 · 스택: ROS2 Humble, Nav2 1.1.20(컨테이너 `amr-fleet-system:wf-final`의 dpkg·헤더·XML·YAML 확인 + GitHub `humble` 브랜치 소스 직접 조회) · 프로젝트 정합 기준: `config/robot_params.yaml`(limits·safety·payload), `config/sensors.yaml`, `config/ekf.yaml`, `docs/architecture/components.md` §3.3·§5.3·§6, `sequences.md` §1–3

## 0. 요약

- **기준(baseline)**: 1200×800 코스트맵 위 8-연결 cost-aware A\*. 옥타일 휴리스틱의 허용성·일관성 증명. 메모리 ≈ 12 MB/인스턴스, 최악 O(N log N), N = 960 000.
- **문헌(2023-09 → 2026-09)**: VERIFIED 논문 24편 + 고전 검증 4건(Field D\*, FM², Mitsch IJRR 2017, OMPL PathSimplifier). 흐름: (i) any-angle의 정확·고속화, (ii) JPS 변형, (iii) "재계획은 처음부터 vs 증분"의 재평가, (iv) A\* 후처리. 단일 로봇 Nav2 코스트맵 위에서 **주행시간 예측을 계획 비용과 같은 모델로 내는** 공개 구현은 찾지 못했으나, 아이디어 자체는 FM²(속도맵 시간 비용)·ISO 3691-4(정지거리 규칙)·Nav2 MPPI(inflation 역함수)에 이미 있다.
- **제안: TAR-A\*** (Time-Aware Repairing A\*), 세 요소. (C1) 정지거리 속도맵 시간 비용 탐색 = **Nav2 코스트맵 위 FM²의 공학적 변형**(variant). (C2) 비용 재평가 기반 튜브 한정 국소 복구 = Local-Repair/윈도우 탐색의 **공학적 적응**, 단일 마감시간으로 최악 지연 유계. (C3) 비용적분 LOS 숏컷 + 여유거리 평활화 + 속도 프로파일 → 예측 주행시간 = OMPL 비용 인지 단순화·Dolgov 평활화·경로-속도 분해의 **변형**. 패키지 전체는 **new_combination**이며 개별 요소의 신규성은 주장하지 않는다.
- 구현: C++17 `nav2_core::GlobalPlanner` 플러그인 `amr_navigation::AStarPlanner`(components.md §3.3의 이름, `planner_id: AStar`) + ROS 비의존 코어(gtest ≥ 70 %). 예산: 새 목표 FULL 평균 ≤ 50 ms(sequences.md §1 예산)·p95 ≤ 100 ms, LOCAL p50 ≤ 5 ms, **최악(FULL 포함) ≤ 400 ms 단일 마감**.
- 실행 속도는 `safety_node` 게이트(`robot_params.yaml safety.*`)를 거치므로 C1 속도맵은 그 게이트보다 **느슨한 상계**이고(수치 확인, §4.1; 게이트의 연속 제한이 등방일 때 — 방향은 안전팀 확인 대상, §7-5), 예측 시간(C3)은 게이트를 포함해 계산한다.

## 1. 범위·제약과 검증된 스택 사실

명세 요구(4장-4): A\* 직접 구현, NavFn 또는 Smac과 **경로 길이·계획 시간·장애물 여유 거리** 비교, 추종 가능한 평활 경로, 50쌍 성공률 ≥ 98 %, 재계획 ≤ 500 ms, 예측 주행시간 오차 ≤ 15 %, 로봇 폭 + 20 cm(= 0.6 m) 통로 통과, 동적 장애물용 센서 갱신 주기·장애물 유지 시간 설정. 연계: 4장-5(Pure Pursuit/Stanley, CTE 5/10 cm, 사다리꼴/S-curve), 4장-7(회피 이탈 ≤ 1 m, 복귀 ≤ 5 s, 안전거리 0.3 m), 4장-10(5대 CPU ≤ 80 %, 커버리지 ≥ 70 %), 4장-1(적재 질량 반영). 7장: Nav2 기반이되 핵심 로직 직접 구현, 상용 금지.

컨테이너(`/opt/ros/humble`)와 `humble` 브랜치 소스에서 확인한 사실:

| 항목 | 확인 결과 |
| --- | --- |
| 패키지 | nav2_core/navfn/smac/theta_star/smoother/costmap_2d/planner/**mppi_controller** 모두 1.1.20 |
| 플러그인 인터페이스 | `nav2_core::GlobalPlanner`: `configure(parent, name, tf, Costmap2DROS)`, `activate/deactivate/cleanup`, `nav_msgs::msg::Path createPlan(start, goal)` |
| 액션 | `ComputePathToPose` result `planning_time` = 액션 실행 시작 → 결과 반환(`this->now()` 차, planner_server.cpp `computePlan()` L462/L499; L370/L437은 `computePlanThroughPoses()`). 이 구간에 **`waitForCostmap()`**(코스트맵 `isCurrent()`가 참이 될 때까지 100 Hz 폴링)이 포함 → 관측 버퍼가 `expected_update_rate`[s]보다 오래되면 계획이 **블록**된다 |
| 코스트맵 발행 | `costmap_updates`(OccupancyGridUpdate)의 `header.stamp = rclcpp::Time()`(**0**, costmap_2d_publisher.cpp L207), 전체 `costmap`은 발행 시각; 둘 다 `publish_frequency` 주기 → e2e 지연의 시작 시각으로 쓸 수 없다(§5.3) |
| 코스트맵 필터 | `KeepoutFilter`는 플러그인 층(static·obstacle·inflation) **이후에** 적용(layered_costmap.cpp)되어 inflation되지 않으며, 마스크 OccupancyGrid 0…100을 `round(v·254/100)`로 변환해 기존 값보다 크면 덮어쓴다 → 마스크가 0/100 이진이면 0/254만 추가 |
| IsPathValid | srv에 `invalid_pose_indices[]` 필드가 있으나 **Humble planner_server.cpp는 채우지 않는다**(`is_valid`만 설정; 반경 모드 LETHAL∨INSCRIBED, 풋프린트 모드 LETHAL만) → C2는 무효 인덱스를 자체 계산 |
| 기본 BT | `navigate_to_pose_w_replanning_and_recovery.xml` RateController 1 Hz; `nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml` 2 Hz + `PathExpiringTimer 10 s` |
| 코스트 값 | FREE 0, MAX_NON_OBSTACLE 252, INSCRIBED 253, LETHAL 254, NO_INFORMATION 255 |
| Inflation | `computeCost(d)`: d=0→254; d·res ≤ r_ins→253; else `static_cast<unsigned char>(252·exp(−s(d·res − r_ins)))` (**절삭 = floor**) |
| **코드 기본값** | planner_server `expected_planner_frequency 1.0`; costmap `update_frequency 5.0`, `publish_frequency 1.0`, `robot_radius 0.1`, `footprint "[]"`, **`footprint_padding 0.01`**(폴리곤을 각 축 0.01 m 팽창 → 내접반경 계산에 반영), `robot_base_frame base_link`, `trinary_costmap true`, `lethal_cost_threshold 100`; inflation `cost_scaling_factor 10.0`, `inflation_radius 0.55`; obstacle `observation_persistence 0.0`, `expected_update_rate 0.0`, `obstacle_max_range 2.5`, `raytrace_max_range 3.0`, `marking true`, `clearing false`; Smac2D `cost_travel_multiplier 1.0`, `tolerance 0.125`, `allow_unknown true`, `max_planning_time 2.0`; NavFn `tolerance 0.5`, `use_astar false`, `allow_unknown true` |
| **bringup 예시값**(`nav2_params.yaml`, 기본값 아님) | planner 20.0 Hz 기대; global_costmap 1.0 Hz, `robot_radius 0.22`, `track_unknown_space true`, obstacle scan `clearing/marking True`, raytrace 3.0/obstacle 2.5; inflation s=3.0, R=0.55 |
| MPPI | `ObstaclesCritic::distanceToObstacle`: `(s·r_ins − log(cost) + log(253))/s` — inflation 역함수가 **이미 Nav2에 존재**(분모 253 사용, $s$는 크리틱 자체 파라미터 `inflation_scale_factor`) |
| 스무더 | Smac 내부 `smoother.cpp`: **제자리(Gauss–Seidel) 스윕**, `cost>252 ∧ ≠255`면 마지막 유효 경로로 복원 후 실패; `nav2_smoother` SimpleSmoother/SavitzkyGolay 플러그인 |
| 컴파일러 | g++ 11.4, cmake 3.22, 32 스레드 |

500 ms 주장은 **우리 YAML**(§5.1, §6)에서만 성립하며 위 기본값·예시값과 무관하다.

### 1.1 설정 전제(모든 비교 플래너 공통, 리뷰 반영)

1. global_costmap은 `footprint: "[[0.3,0.2],[0.3,-0.2],[-0.3,-0.2],[-0.3,0.2]]"`(폴리곤, `robot_params.yaml robot.footprint_length/width` 0.60×0.40) + **`footprint_padding: 0.0`** → $r_{ins}=0.20$ m, $r_{circ}=0.361$ m. 코드 기본 패딩 0.01을 두면 $r_{ins}=0.21$, $r_{circ}=0.374$가 되어 0.6 m 통로 중앙이 c = 210($d_{free}$ 0.089 m, $v_{lim}$ 0.42 m/s)으로 바뀐다 — 통과는 되지만 이 문서의 수치는 패딩 0 기준이다(`checks/out_speedmap.txt`). `robot_radius 0.36`을 쓰면 0.6 m 통로 중앙이 INSCRIBED(253)가 되어 모든 플래너가 실패한다.
2. static_layer `trinary_costmap: true`(코드 기본). 그래야 1…252 값이 **inflation 층에서만** 나와 §2.2 역함수가 성립한다. components.md §3.3의 `keepout_filter`(교통 관리자의 `keepout_mask`, sequences.md §3 전략 2)는 **이진 마스크(0/100 → 0/254)에 한해** 허용한다: 필터는 inflation 뒤에 적용되어 1…252 값을 만들지 않기 때문이다(위 표). 중간값 마스크(1…99 → 3…251)나 lane 선호 비용은 코스트맵에 그리지 않는다(fleet 선호는 별도 `extra_cost[N]` 훅, §7-7). keepout 셀은 inflation되지 않으므로 교통 관리자는 분쟁 구간 마스크를 $r_{circ}$만큼 팽창해 발행하도록 요청한다(§7-8).
3. `allow_unknown: true`, `tolerance: 0.25`를 모든 플래너에 동일 적용(NavFn 기본 0.5, Smac 0.125 → 명시).
4. Smac Hybrid-A\*(Reeds-Shepp, r_min 0.4 m)는 차동구동에 like-for-like가 아니므로 **별도 표**로 보고한다.
5. inflation은 `inflation_radius 1.2`, `cost_scaling_factor 2.0`(구성 A) 또는 `2.2 / 2.0`(구성 B, §4.1)을 모든 플래너에 동일 적용.
6. 프레임·토픽(ekf.yaml·components.md와 정합): `global_frame: map`, `robot_base_frame: <r>/base_footprint`(ekf.yaml `base_link_frame: base_footprint`, 접두어는 런치가 주입), static layer `map_topic: /map`(절대 이름, `map_subscribe_transient_local: true`), obstacle layer 관측원 `scan_filtered`(상대 이름, 10 Hz).

## 2. 기준 알고리즘의 수학적 유도

### 2.1 상태공간과 그래프

격자 $W\times H=1200\times800$, 해상도 $r=0.05$ m, $N=9.6\times10^5$. 셀 $v=(x,y)$, 인덱스 $n=x+yW$, 코스트 $c(v)\in\{0,\dots,255\}$. $V=\{v:\ c(v)\le252\ \lor\ (c(v)=255\wedge\texttt{allow\_unknown})\}$, 8-연결 엣지 길이

$$\ell(u,v)=\begin{cases} r & \text{직교}\\ \sqrt2\,r & \text{대각}\end{cases}\quad[\mathrm{m}]$$

**코너 컷 금지**: 대각 이동은 두 직교 이웃이 모두 ≤ 252일 때만 허용(Smac 2D는 자식만 검사).

### 2.2 코스트맵 → 여유거리 → 엣지 가중치

`computeCost`의 floor 때문에 역함수는 **구간**이다(1 ≤ c ≤ 252, 셀이 inflation 층에서만 값을 받는다는 §1.1-2 전제):

$$c=\Big\lfloor 252\,e^{-s(d-r_{ins})}\Big\rfloor\ \Longleftrightarrow\ d\in\Big(\,d_{lo}(c),\ d_{hi}(c)\,\Big],\quad d_{lo}(c)=r_{ins}-\tfrac1s\ln\tfrac{c+1}{252},\ \ d_{hi}(c)=r_{ins}-\tfrac1s\ln\tfrac{c}{252}\ [\mathrm{m}]$$

이하 모든 안전 관련 계산은 **보수적 끝점 $d_{lo}$**를 쓰고, $d_{free}(c)=d_{lo}(c)-r_{ins}$로 둔다. 양자화 폭 $\ln\frac{c+1}{c}/s$는 $s=2$에서 c=1: 0.35 m, c=34: 0.014 m, c=186: 0.003 m, c=228: 0.002 m — 장애물 근처에서는 무시할 만하고 먼 곳에서만 거칠다. $c=0$이면 $d>R_{infl}$만 안다. 이 역함수는 Nav2 MPPI `ObstaclesCritic::distanceToObstacle`가 이미 사용하는 것이며(우리 기여 아님), 정확한 $d$가 필요하면 Felzenszwalb O(N) EDT를 코스트맵 갱신 시 백그라운드로 돌린다(≈ 10–20 ms).

기준 A\*의 엣지 가중치(Smac 2D `node_2d.cpp` 형식):

$$w_\kappa(u,v)=\ell(u,v)\Big(1+\kappa\,\frac{c(v)}{252}\Big)\quad[\mathrm{m}]$$

기존 플래너의 가중치(모두 `humble` 소스 확인):

| 플래너 | 셀 통과 비용 | 등가 $\kappa$ | 휴리스틱 | 탐색 구조 |
| --- | --- | --- | --- | --- |
| NavFn | $50+0.8c$ (253→254 처리, ≥254 클립) | $0.8\cdot252/50\approx4.03$ (**셀 비용 등가일 뿐 경로 최적성 등가 아님**) | `hypot·50` (use_astar) | 4-이웃 보간 퍼텐셜(Eikonal 근사); "A\* 모드"는 힙 A\*가 아니라 **임계값 버킷 파면**(`priInc = 2·COST_NEUTRAL = 100`, curT 증가)이며 그래디언트 하강으로 경로 추출(0.5셀) |
| Smac 2D | $(1,\sqrt2)\cdot(1+\kappa c/252)$ | $\kappa$ = `cost_travel_multiplier` = 1.0 (Humble 기본; rolling 문서 2.0) | Euclid | 8-이웃 `std::priority_queue`, 충돌 ≥ 253 |
| Nav2 Theta\* | $1.0\cdot\|pv\|_{\text{셀}}+\sum_{\text{LOS 셀}}2.0\Big(\tfrac{26+0.9c}{252}\Big)^2$ (`w_euc_cost 1.0`, `w_traversal_cost 2.0`, `getCost = 26+0.9c`; 유클리드 항은 **셀 단위** `hypot`) — 비선형, c=0에서도 셀당 0.021 | 비선형 | $1.0\cdot$Euclid[셀] (`w_heuristic = min(w_euc,1)`; 유클리드 항에 대해 일관) | 8-이웃 + Bresenham LOS(비용 누적). 노드 안전 판정 c ≤ 252, LOS 셀은 $26+0.9c\le252$ ⇒ c ≤ 251; unknown은 `OCCUPIED_COST−1`(=253)로 누적 |

### 2.3 옥타일 휴리스틱의 허용성·일관성

$$f(v)=g(v)+h(v),\quad h_{oct}(v)=r\big[\max(|\Delta x|,|\Delta y|)+(\sqrt2-1)\min(|\Delta x|,|\Delta y|)\big]\ [\mathrm{m}],\ (\Delta x,\Delta y)=v-v_{goal}$$

- 허용성: $h_{oct}$는 장애물 없는 8-연결 격자의 정확한 최단거리이고 $w_\kappa\ge\ell$이므로 $h_{oct}\le$ 잔여 비용.
- 일관성: $h_{oct}(u)\le\ell(u,v)+h_{oct}(v)\le w_\kappa(u,v)+h_{oct}(v)$ → 노드당 최대 1회 확장.
- 8-연결의 기하 최적 대비 최대 초과율 $\max_\theta[\cos\theta+(\sqrt2-1)\sin\theta]=1.0824$ (θ = 22.5°) → ≤ 8.2 %. 후처리 숏컷이 이를 얼마나 회수하는지는 가정하지 않고 측정한다(δ = 0.05 시간 조건이 inflation 띠를 가로지르는 현을 막을 수 있음, §4.3(a)·§5.4-5).

타이브레이크: 키 $(f,h)$ 사전순(같은 f면 h 작은 것) → 결정론적, 평원 확장 감소.

### 2.4 자료구조·메모리·복잡도

| 배열 | 형 | 크기 |
| --- | --- | --- |
| `g[N]` | float32 | 3.84 MB |
| `parent[N]` | int32 | 3.84 MB |
| `stamp[N]` (세대 g: 2g = open/방문, 2g+1 = closed; memset·closed 비트셋 불필요) | uint32 | 3.84 MB |
| open list | 이진 힙 (f, h, idx) 12 B, lazy deletion | 전형 < 3 MB |

합계 ≈ 12 MB/인스턴스, 5대 ≈ 60 MB. $O((N+E)\log N)$, $E\le8N$. 확장당 ≈ 150–300 ns → 전 격자 최악 ≈ 150–300 ms, 전형(20–40 m) ≈ 20–60 ms. 도달 불가는 **연결성 라벨 사전검사**(코스트맵이 바뀐 뒤 첫 FULL 직전에 O(N) BFS로 갱신, §4.2-5)로 즉시 실패. 참고 스케일: [V6]는 33 600 m² 창고에서 Cost-Aware 2D-A\* 평균 1 358 ms(10회, i7-8565U)를 보고한다. 논문은 이 맵의 **해상도를 밝히지 않는다** — 0.05 m라고 가정하면 1 344만 셀(우리의 14배)이라 선형 환산 시 우리 96만 셀에서 ~100 ms 급이지만, 이는 가정에 기댄 대략값이며 우리 예산은 §5.4-1 마이크로벤치로만 확정한다.

### 2.5 NavFn/Smac/Theta\* 내부 동작 요약

- NavFn: 목표→시작 퍼텐셜 전파(4-이웃 두 최소 이웃 보간), 경로는 퍼텐셜 그래디언트를 따라 추출되어 any-angle에 가깝고 부드럽다. 국소 평원 진동, 253을 장애물로 취급.
- Smac 2D: 우리 기준과 동형 + 내부 Gauss–Seidel 스무더(§2.6). `max_on_approach_iterations`로 허용오차 내 근사 목표.
- Smac Hybrid-A\*: 차량형 프리미티브 + 해석적 확장. 차동구동에는 like-for-like 아님(§1.1-4).
- Theta\*: 부모의 부모와 LOS면 직접 연결, LOS 도중 비용 누적(위 표). Discourse 벤치[V24]는 Theta\* 86 %/Smac2D 90 %/A\* 100 %를 보고하지만 **미검토 단일 게시물**이고 맵 크기 미기재, 댓글(chfritz)이 "A\*가 최적이 아니라는 결과는 허용적 휴리스틱과 모순"이라 반박했다 → 일화적 참고로만 취급(§9).

### 2.6 경로 평활화 기준식(Smac `smoother.cpp`, Humble 소스 확인)

$$y_i\leftarrow y_i+w_d(x_i-y_i)+w_s(y_{i-1}+y_{i+1}-2y_i)$$

$x$ 원 경로, $y$ 평활 경로, $\sum|\Delta y|<\text{tol}$이면 종료, 어느 점이든 $c>252$(255 제외)면 직전 스윕 결과로 복원 후 실패. 소스는 $y_{i-1}$을 **이미 갱신된 `new_path`**에서 읽고 $y_i$를 즉시 써 넣으므로 **Gauss–Seidel** 스윕이다. 고정점은 $(w_dI-w_sL)y=w_dx$($L$ = tridiag(1,−2,1))이고, 갱신을

$$y_i\leftarrow(1-\omega)y_i+\omega\,\frac{w_dx_i+w_s(y_{i-1}+y_{i+1})}{w_d+2w_s},\qquad\omega=w_d+2w_s$$

로 쓰면 SPD 행렬 $w_dI-w_sL$에 대한 SOR이므로(Ostrowski–Reich) **수렴 조건은 $w_d>0,\ 0<w_d+2w_s<2$**. Nav2 기본 $(0.2,0.3)$ → $\omega=0.8$. 초판의 Jacobi 조건 $w_d+4w_s<2$는 충분조건일 뿐이다(예: (0.2, 0.5)는 Jacobi 조건 위반이나 실제 코드는 수렴). 수치 확인(양끝 고정, n = 60, 반복행렬 $(I-L_{low})^{-1}(D+U)$의 정확한 스펙트럼 반경): GS (0.2, 0.3) 0.70, (0.2, 0.5) 0.52, (0.2, 0.85) 0.90, (0.2, 0.89)[ω = 1.98] 0.98, (0.2, 0.91)[ω = 2.02] 1.02 발산, (0.2, 0.95)[ω = 2.1] 1.10 발산 — 경계가 정확히 ω = 2; Jacobi (0.2, 0.5) 1.20 발산(`checks/out_audit2.txt`; 시간 전개로 잰 유한 반복 축소율은 `out_misc.txt`). 우리 C3(b)도 같은 Gauss–Seidel 스윕을 쓰되 비선형 여유거리 항이 들어가므로 수렴 보장은 주장하지 않고 롤백 가드·최대 반복으로 안전을 확보한다(§4.3). 이 식만으로는 여유거리를 늘리지 못하고 곡률 한계가 없다.

## 3. 문헌 조사 (2023-09 → 2026-09)

검색: arXiv 검색 UI/abs 페이지(export API 429), GitHub `humble` raw 소스(curl·WebFetch), Nav2 docs, Discourse, IJCAI/RA-L/RSS/ICRA 페이지, CMU RI 출판 페이지, OMPL 문서. 초록/본문을 직접 가져오면 VERIFIED, 검색 스니펫만이면 **S(스니펫)**, 고전은 RECALLED.

### 3.1 격자 최적·any-angle 탐색

| # | 논문 | 핵심 | 우리와의 관계 |
| --- | --- | --- | --- |
| V11 | Ibrahim et al., RA-L 2024 | 2D 격자 O(n) 정확 any-angle one-to-all | 균일 비용 전제. NavFn류 근사 Eikonal의 정확 대안 |
| V1/V2 | Zou & Borst, Zeta\*/Zeta\*-SIPP, IJCAI 2024·arXiv 2026 | 타원 전방 확장 + FoV, TO-AA-SIPP 대비 20× | any-angle SOTA 기준점; 동적 장애물은 SIPP |
| V13 | Reijgwart et al., RSS 2025 | 다해상도 계층 any-angle | 대형 맵용; 우리는 단일 해상도로 충분 |
| V8 | Lai, R2/R2+ 학위논문 2024 | 벡터/버그 any-angle | 비용 가중 미지원 |
| V9 | Cobano et al., RAS 2025 | EDF 항이 삼각부등식 만족 → Lazy Theta\* 부모 선택 | 여유거리를 비용에 넣으며 일관성 유지하는 사례 |
| V14 | Yakovlev et al., IROS 2024 | any-angle MAPF | fleet 팀 |
| **V28** | Ferguson & Stentz, **Field D\***, ISRR 2005 / JFR 2006 | 셀 경계 **선형 보간**으로 비균일 비용 격자에서 연속 헤딩 경로·재계획 | any-angle이 균일 비용에 묶여 있다는 초판 주장을 **반증**. 미채택 사유는 §4.7 |

### 3.2 JPS 계열

| # | 논문 | 핵심 | 관계 |
| --- | --- | --- | --- |
| V16 | Zhao, Harabor, Stuckey, CJPS, 2023-06 | JPS 중복 스캔 제거, 동적 격자 7–14× | JPS는 균일 비용 전제 → inflation 코스트맵에 부적합 |
| V12 | Baum, JPS4, 2025 | 4-연결 JPS; 밀집에서만 우세 | 열린 통로 많은 창고에 이득 불확실 |
| S21 | Electronics 14(8):1669, 2025 | 적응 가중 휴리스틱 + 동적 제약원 양방향 JPS | **스니펫만 확인**(MDPI 403). 가중 휴리스틱은 최적성 상실, 미채택 |

### 3.3 증분·재계획

| # | 논문 | 핵심 | 관계 |
| --- | --- | --- | --- |
| V10 | Sabbadini et al., ICRA 2026 | 빠른 ASAO 플래너로 **처음부터 반복**이 반응형 수리보다 중앙값 경로 길이에서 우세 | C2의 대항 가설 → 같은 시나리오에서 실측 |
| V19 | Espahbodi Nia, FMT^X, 2025/26 | lazy wavefront 동적 재계획 | 샘플링 기반, 격자 비적용 |
| V3 | Talia et al., IGHA\*, RA-L 2025 | Hybrid A\* 격자 지배 프루닝을 동적 조직으로 대체, **저자들의 자체 최적화 HA\*M 대비** 6× 적은 확장 | Nav2 Smac Hybrid와 직접 비교 아님 → 정성 참고만 |
| V7 | Shen et al., 서베이 2026 | 138편 분류 | 배경 |
| V27 | Ha et al., MMP-A\*, 2026 | VLM 웨이포인트 휴리스틱 | 설명가능성 상충, 미채택 |

### 3.4 유계 준최적·휴리스틱

| # | 논문 | 핵심 | 관계 |
| --- | --- | --- | --- |
| V17 | Duc et al., Probabilistic Focal Search, 2026-09 | FOCAL/최소-f 확장을 확률로 교대, f_min 정체 시 ≈ 90 % 확장 감소 — **N-Puzzle·TSP 도메인**(실험은 Pancake·GCTSP 포함, 격자 없음) | 격자 결과 아님; 우리 맵의 f 평원은 짧을 것으로 예상(측정 항목) |
| V18 | Shperberg et al., 2025 | BAE\* 유계 준최적 변형 | 동일 |

### 3.5 후처리·평활화·속도맵·안전거리

| # | 논문 | 핵심 | 관계 |
| --- | --- | --- | --- |
| V15 | Li & Cheng, APP, RA-L 2023 | 코스트맵 위 양방향 숏컷 + 섭동 | C3의 가장 가까운 최근 선행 |
| **V31** | OMPL `PathSimplifier` 문서 | 생성자에서 `OptimizationObjective`를 받아 `reduceVertices`를 제외한 단순화 메서드(부분 숏컷·섭동·B-spline 등)에 쓰는 **비용 인지** 단순화(문서가 채택 규칙의 세부는 밝히지 않음) | C3(a) "비용을 악화시키지 않는 숏컷"의 선행 → variant |
| V4 | Pastorelli et al., RA-L 2025 | G¹·유계 곡률 폴리라인 평활화 | 곡률 한계 처리 참고 |
| V23 | Chen & Li, FDSPC 2024 | 연속 곡률 적분 | 격자 비대상 |
| V5 | Gonzalez-Garcia et al., ICRA 2026 | 직사각 코리도 그래프 | 2단계 후보 |
| V20 | Tang et al., Sensors 2025 리뷰 | A\* 개선·spline 후처리 추세 | 배경 |
| **V29** | Valero-Gómez, Gómez, Garrido, Moreno, **FM²**(saturated, FM²\*), IEEE RAM 2013 (PDF 확인) | 장애물 거리 → 속도맵 → 시간 최적 파면 | **C1의 원형**. 우리는 파면 대신 A\*, 속도맵을 정지거리 규칙으로 정의 |
| **V30** | Mitsch, Ghorbal, Vogelbacher, Platzer, IJRR 2017 | 수동 안전(passive safety)의 형식 검증(초록 확인); 불변식 형태 $\|p-o\|>\tfrac{v^2}{2b}+\big(\tfrac Ab+1\big)\big(\tfrac A2\varepsilon^2+\varepsilon v\big)$는 본문 기억(RECALLED) | C1 $v\le\sqrt{2ad}$ 규칙과, 반응 지연 항을 가진 프로젝트 안전 규칙(`robot_params.yaml` $t_r$=0.15 s)의 형식적 배경 |
| R15 | ISO 3691-4:2020 | 무인 산업차량 안전(인원 감지·속도 의존 요구) | 정지거리 보호영역 관행의 출처(본문 미열람) |

### 3.6 스택·벤치마크

- V6 Macenski et al. (2024, **본문 확인**): 임의 10 000 m² 맵 3종(장애물 밀도 10/15/20 %, 각 1 000쌍)에서 **Hybrid-A\*/State Lattice**(38.8–43.3 ms)가 Cost-Aware 2D-A\*(66–89 ms) 대비 ≈ 50 %, NavFn(61–71 ms) 대비 38 % 빠르고, 길이는 최적 대비 +2.5 % 이내. 33 600 m² 창고(10회 평균, i7-8565U): Hybrid-A\* 290 ms, Lattice 473 ms, **2D-A\* 1 358 ms**. 두 실험 모두 맵 해상도는 논문에 없다. 2D-A\* 비용식 $C=d(1+\alpha c/c_{max})$ 확인. 초판이 이 수치를 "feasible 플래너"로 뭉뚱그린 것을 정정.
- V24 Discourse(2026-05): Dijkstra/A\* ≈ 22 ms·100 %, Theta\* ≈ 18.6 ms·86 %, Smac2D ≈ 19 ms·90 %(무작위 과제 3 000건·독립 실행 3회 — 게시물 문구로는 3 000이 총수인지 회당 수인지 불명, 맵 크기 미기재, chfritz 반박) → **일화적**.
- V22 GRACE(ICRA 2026) 다중 표현 벤치 — fleet 팀. V26 ROS 메인테이너 서베이(2023-07).

### 3.7 시사점

1. 96만 셀에서 cost-aware A\*는 수십 ms → 차별화는 경로 품질(여유·부드러움), 재계획 안정성·지연 유계, 시간 예측.
2. any-angle과 비균일 비용은 양립한다(Field D\*, Theta\*의 LOS 비용 누적). 그럼에도 탐색은 8-연결 A\*, any-angle 효과는 후처리로 얻는 이유: (i) 완전성·최적성 증명이 단순해 동료 평가에서 설명 가능, (ii) 8-연결 초과율 상한이 8.2 %로 작고 비용적분 LOS 숏컷이 그 일부를 회수할 것으로 기대(회수율은 §5.4-5에서 측정), (iii) Nav2에 Field D\* 플러그인이 없어 like-for-like 비교군을 만들 수 없음. V24는 근거로 쓰지 않는다.
3. "증분 vs 처음부터"(V10)는 실측으로 판정 — 우리 C2는 실패 비용을 ≤ 40 ms로 제한한 절충안이다.

## 4. 독자 알고리즘 제안: TAR-A\* (Time-Aware Repairing A\*)

### 4.1 C1 — 정지거리 속도맵과 시간 비용 탐색 (FM²의 Nav2 코스트맵 변형)

**속도맵**(등방 정지거리 대용물):

$$v_{lim}(c)=\begin{cases} v_{max} & c=0\\ \mathrm{clamp}\big(\sqrt{2a_{max}\,d_{free}(c)},\ v_{floor},\ v_{max}\big) & 1\le c\le252\\ v_{floor} & c=255\ (\texttt{allow\_unknown})\end{cases}\quad[\mathrm{m/s}],\qquad d_{free}(c)=d_{lo}(c)-r_{ins}$$

$a_{max}=1.0$ m/s², $v_{max}=2.0$(`robot_params.yaml limits.max_linear_acceleration/max_linear_velocity`), $v_{floor}=0.2$(= `safety.critical_zone_max_speed`). 이 규칙은 ISO 3691-4류 보호영역 설계와 [V30]류 안전거리 불변식 $v^2\le2a\,d$의 등방 근사다: $d_{free}$는 전방향 최소 여유이므로 벽과 나란히 0.1 m 간격으로 달릴 때도 "0.1 m 안에 정지"를 요구한다 — **물리 제약의 유도가 아니라 휴리스틱**이다. 측방 장애물에 대해서는 정지거리 규칙보다 보수적이지만, 반응 지연·정지 여유를 둔 **프로젝트 안전 게이트보다는 느슨하다**(아래 "안전 게이트와의 관계"). 방향성 속도맵은 상태를 3D로 만들기 때문에 보류(§4.7). 적재 등급(`payload.small/medium/large` = 2/10/25 kg)별 $a_{max}$는 모션 팀 값을 받되, 현재 `robot_params.yaml`은 모든 등급에 1.0을 쓴다(§5.3).

**엣지·휴리스틱**: $w_t(u,v)=\ell(u,v)/v_{lim}(c(v))$ [s], $h_t(v)=h_{oct}(v)/v_{max}$ [s]. **명제**: $h_t$는 $w_t$에 대해 허용적·일관적. 증명: $v_{lim}\le v_{max}$ ⇒ $w_t\ge\ell/v_{max}$; §2.3 삼각부등식을 $v_{max}$로 나누면 $h_t(u)\le w_t(u,v)+h_t(v)$. ∎ (일관 휴리스틱의 상수 배 — 기여가 아니라 정합성 확인.) A\*가 최소화하는 $J=\sum w_t$는 **실행 속도가 경로 위 모든 점에서 $v\le v_{lim}(c)$일 때만** 주행시간 하한이다(가감속·회전 한계는 시간을 늘리기만 하므로 하한을 깨지 않고 느슨하게 할 뿐). 그 조건이 없으면(예: 여유와 무관한 DWA 속도 정책) 상·하한 어느 쪽도 아니다. 이 프로젝트에서는 아래 안전 게이트가 그 조건을 컨트롤러와 무관하게 강제한다 — 단, 하한은 **느슨**하므로 예측 시간은 $J$가 아니라 C3(c)(d)로 낸다(§7-3).

**권고 구성에서의 수치**($s=2.0$, $r_{ins}=0.20$; 초판의 $s=3$ 표를 정정):

| c | $d_{free}$ 구간 (lo, hi] [m] | $v_{lim}$ (lo 기준) [m/s] | 등가 승수 $m=v_{max}/v_{lim}$ | Smac κ=1 승수 |
| --- | --- | --- | --- | --- |
| 0 (d > R_infl) | — | 2.00 | 1.00 | 1.00 |
| 34 (R_infl=1.2 경계 안쪽) | (0.987, 1.002] | 1.41 | 1.42 | 1.13 |
| 50 | (0.799, 0.809] | 1.26 | 1.58 | 1.20 |
| 100 | (0.457, 0.462] | 0.96 | 2.09 | 1.40 |
| 186 | (0.149, 0.152] | 0.55 | 3.66 | 1.74 |
| **206 (0.6 m 통로 중앙)** | (0.098, 0.101] | 0.44 | 4.51 | 1.82 |
| 228 (d = 0.25 m = 5셀, 축 방향) | (0.048, 0.050] | 0.31 | 6.46 | 1.90 |
| 240 / 245 / 248 (d = √20·r, √18·r, √17·r = 0.224/0.212/0.206 m) | (0.022, 0.024] / (0.012, 0.014] / (0.006, 0.008] | 0.21 / **0.20** / **0.20** | 9.47 / 10.0 / 10.0 | 1.95–1.98 |

Nav2 inflation의 거리는 셀 중심 간 `hypot(i,j)·r`이라 0.05 m 배수가 아니다. 따라서 INSCRIBED 바로 바깥 셀은 c = 248(d = 0.206 m)이고, 1 < d/r < 5 구간에서 **c ∈ {240, 245, 248}이 실제로 나타나며 245·248은 $v_{floor}$로 클램프**된다(초판·개정판의 "c ∈ [229, 252]는 나타나지 않는다, $v_{floor}$는 255에만"은 틀림, `checks/out_speedmap.txt`). 0.6 m 통로 중앙(벽 셀 중심까지 0.30 m)은 $c=\lfloor252e^{-0.2}\rfloor=206<253$이라 **코스트맵상** 통과 가능하고 0.44 m/s로 감속된다(실제 주행 가능 여부는 안전 게이트에 달림, 아래).

**inflation 경계 불연속(구성 A: $R_{infl}=1.2$ m)**: 경계 안쪽 c=34에서 미터당 0.712 s, 바깥 0.5 s → 경계를 나가며 **29.7 % 급락**. 결과: 열린 구역에서는 경로가 랙에서 ≥ 1.2 m 떨어지려 하고, 띠를 가로지르는 데 드는 추가 시간은 $\Delta T=2\int_{d_1}^{1.2}\big(\tfrac1{v_{lim}}-\tfrac1{v_{max}}\big)\,dd$; 예컨대 $d_1=r_{circ}$(0.36 m)까지 접근했다 돌아오는 비용 ≈ 0.86 s이므로 플래너는 최대 $0.86\cdot v_{max}\approx1.7$ m 길이를 더 써서라도 그 접근을 피한다. 3 m 통로(랙-랙)에서는 중앙 0.6 m 폭만 c=0이라 자연스럽게 중앙 주행. **구성 B**: $R_{infl}=r_{ins}+v_{max}^2/2a_{max}=2.2$ m이면 경계에서 $v_{lim}(4)=1.98\approx v_{max}$로 **연속**(점프 1 %)이나 맵 대부분이 inflation 안에 들어가 갱신 비용이 커진다. 초판의 "길이 −2…+3 %" 가설은 근거가 없어 철회하고 P50×{A, B, Nav2 기본 0.55/3.0}에서 길이·여유를 **측정**한다(§5.4-6).

**프로젝트 안전 게이트와의 관계(감사 보정)**: 실행 속도는 `velocity_profiler_node` → `safety_node`를 거쳐서만 로봇에 닿고(components.md §3.3·§3.4), `robot_params.yaml safety`는 이미 `distance_reference: footprint_edge`, `clearance_speed_limit_enabled: true`로 정해져 있다: 풋프린트 가장자리–장애물 최단거리 $D$에 대해 $D\le0.3$ 정지, $D\le0.5$ → 0.2 m/s, $D\le1.0$ → 0.5 m/s, 그 밖 $v_{gate}(D)=-at_r+\sqrt{(at_r)^2+2a(D-0.3)}$ ($t_r=0.15$ s, $a=1.0$). **방향 가정(2차 감사)**: 설정은 구역 거리를 "풋프린트 모서리–장애물 최단거리"(등방)로 정의하지만 연속 제한 주석은 "최고속 2.0 m/s는 **전방** 여유 2.6 m 이상일 때만"이라고 적어 연속 제한의 방향이 모호하다. 이 절의 하한 논증과 아래 통로 속도는 연속 제한도 **등방**이라고 가정한다. 연속 제한이 전방 여유에만 걸리면 측방 장애물의 게이트는 구역 상한(0 / 0.2 / 0.5 m/s, D ≥ 1.0 m에서 2.0 m/s)뿐이어서 (a) $J$의 하한 성질이 구성 A에서는 경계 c = 34(D = 1.00 m) 한 값에서만, 구성 B에서는 c = 4…34(D 1.0–2.0 m) 전체에서 깨지고, (b) 2.4·3.0 m 통로 직진 게이트가 2.0 m/s가 되어 아래 "−36 %" 편향이 사라진다(`checks/out_audit2.txt`). 따라서 C3(c)의 $v_{gate}$는 `safety_node` 구현과 같은 방향 규칙을 써야 하고, 방향 정의를 §7-5로 안전팀에 확인한다. 기하적으로 $D\in[d_c-r_{circ},\,d_c-r_{ins}]$이고 $v_{gate}$는 단조 비감소이므로 실행 속도 $\le v_{gate}(d_{hi}(c)-r_{ins})$이며, 모든 도달 가능한 c(구성 A·B)에서 $v_{gate}(d_{hi}-r_{ins})\le v_{lim}(c)$(최소 여유 0.20 m/s, `checks/out_safety_gate.txt`) → **안전 게이트가 켜져 있고, 로봇이 경로 위에 있으며, 코스트맵 장애물이 LiDAR 스캔 평면(지상 0.38 m, sensors.yaml)에 보이는 한 $J$는 컨트롤러와 무관한 하한**이다(스캔에 안 보이는 정적 맵 장애물 옆에서는 게이트가 더 빠른 속도를 허용할 수 있다). 대신 게이트가 훨씬 느리다:

| 가장자리 여유 $D$ [m] | 0.1 | 0.3 | 0.5 | 0.8 | 1.0 | 1.3 | 2.0 | 2.6 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| $\sqrt{2aD}$ (C1) | 0.45 | 0.77 | 1.00 | 1.26 | 1.41 | 1.61 | 2.00 | 2.00 |
| $v_{gate}$ (safety_node) | **0 (정지)** | 0 | 0.20 | 0.50 | 1.04 | 1.27 | 1.70 | 2.00 |

정적 구조(랙·벽)도 스캔에 잡히므로 중앙 주행 시 통로 폭 0.6/1.2/2.4/3.0 m → $v_{gate}$ = 0(E-stop)/0.20/1.04/1.27 m/s. 결과(연속 제한이 등방일 때): (i) 3 m 통로 20 m 직진에서 C1은 10 s, 게이트는 15.7 s → 게이트를 빼면 예측이 36 % 짧다 → **C3(c) 예측 프로파일에 $v_{gate}$를 반드시 포함**; (ii) 현재 설정으로는 명세 4.4 "폭 + 20 cm 통로"가 E-stop으로 **불가능** → §5.3·§7-5로 안전팀에 올린다. 탐색 비용에도 게이트를 쓰는 선택지(`speed_rule: gate` = $\min(\sqrt{2aD},v_{gate}(D))$)는 코스트맵이 $D\le R_{infl}-r_{ins}$만 분해하므로 $R_{infl}\ge r_{ins}+2.6=2.8$ m(구성 C)를 요구한다 → §5.4-6 구성 실험에 추가하고, 기본값은 연속·분해 가능한 `speed_rule: brake`로 둔다.

### 4.2 C2 — 비용 재평가 기반 튜브 한정 국소 복구

저장 상태(계획 시점): 목표 자세 $g$, 격자 경로 $Q=(q_0,\dots,q_M)$, 각 셀의 당시 코스트 $c^{old}_i$, 시간 비용 접두합 $G_i=\sum_{j<i}w_t(q_j,q_{j+1})$(**이전 코스트맵 기준, 저장값**), 발행 경로 $P_s$(C3 후처리 결과, off-grid)와 대응표 $\pi$: $P_s$의 각 정점 → $Q$ 인덱스(숏컷 끝점은 $Q$의 점, 평활 점은 원래 인덱스). $J_{old}(i_a\!:\!i_b):=G_{i_b}-G_{i_a}$ — 새 코스트맵 위에서 재정의하지 않는다.

1. **분류(매 호출, O(M) ≈ 0.1 ms, 필수)**: 목표가 저장된 $g$와 다르거나(새 작업) 로봇이 $P_s$에서 $\epsilon_{off}=0.5$ m 밖이면 FULL. 아니면 (a) **충돌 검사는 실제로 추종 중인 $P_s$**의 supercover 셀에서 한다(숏컷 현은 $Q$에 없는 셀을 지나므로 $Q$만 보면 놓친다); c ≥ 253(255 ∧ `allow_unknown` 제외)인 $P_s$ 구간을 $\pi$로 $Q$ 인덱스 집합 $I_{col}$로 옮긴다. (b) **비용 드리프트**는 $Q$ 위에서 $w^{new}_i$를 다시 계산해 $I_{drift}=\{i:\ w^{new}_i>(1+\eta_c)w^{old}_i\}$, $\eta_c=0.5$. $I=I_{col}\cup I_{drift}$. $I=\emptyset\wedge J^{new}\le(1+\eta_v)J^{stored}$($\eta_v=0.1$)이면 **VALID**: 탐색 없이 현재 위치에서 잘라 반환. $I=\emptyset$인데 $J^{new}>(1+\eta_v)J^{stored}$(작은 드리프트가 넓게 퍼짐)이면 국소 창을 정의할 수 없으므로 **FULL**. 초판처럼 c ≥ 253만 보면 동적 장애물 inflation으로 c≈252가 된 경로를 "유효"로 두어 $v_{floor}$ 포복이 생기므로 **비용 드리프트 검사를 의무화**한다(리뷰 반영).
2. 구간 $[i_a,i_b]=[\min I-k,\ \max I+k]$, $k=\lceil1.0\,\mathrm{m}/\overline{\Delta s}\rceil$. 구간 길이 $L>L_{max}=15$ m면 FULL.
3. 튜브 $T_\rho=\{v:\min_{j\in[i_a,i_b]}\|v-q_j\|_2\le\rho\}$ — **유클리드** 원판 스탬핑(감사 보정: 개정판의 $\|\cdot\|_\infty$ 튜브는 대각 경로에서 셀이 경로로부터 $\sqrt2\rho$ = 1.41 m까지 벗어날 수 있어 아래 "이탈 ≤ 1 m" 주장과 모순이었다). 셀 수 $|T_\rho|\approx(2L\rho+\pi\rho^2)/r^2$(방향 무관; 래스터 경계 셀 때문에 이 값을 최대 +2.3 % 넘으므로 **상계가 아니다**) $\le(2L\rho+4\rho^2)/r^2$(리뷰가 제시한 축 정렬 $\infty$-튜브의 값; L ∈ {0.5, 2, 6, 15} m × ρ ∈ {1, 3} m × 0°/22.5°/45°의 모든 유클리드 래스터가 이 아래; 45° 경로의 $\infty$-튜브는 이를 넘는다 — ρ=1 m에서 8 566 > 6 400셀). L=6 m에서 ρ=1 m → 6.1–6.2×10³(상계 6.4×10³), ρ=3 m → 2.6×10⁴(상계 2.9×10⁴), ρ=6 m → 8.6×10⁴ 이하, ρ=12 m → 2.9×10⁵ 이하(맵의 30 %) (`checks/out_tube_euclid.txt`, `out_audit2.txt`). **$\rho_0=1.0$ m**의 복구 **격자** 경로는 이전 격자 경로 $Q$로부터 구성상 ≤ 1 m로 **명세 4.7 "이탈 ≤ 1 m"와 정합**한다 — 단 (i) 4단계의 창 재후처리 숏컷 현은 비볼록 튜브를 벗어날 수 있으므로 창 안에서는 현의 supercover 셀이 모두 $T_\rho$에 속할 때만 채택하고(2차 감사), 평활화의 점 이동은 작지만 보장되지 않으므로 발행 경로 기준 이탈은 §5.3에서 측정한다; (ii) ρ₁ 복구와 FULL은 이 보장이 없어 §5.3에서 모드별로 보고한다.
4. $T_\rho$ 안에서 TAR-A\*로 $q_{i_a}\to q_{i_b}$. 채택 조건 $J_{new}\le(1+\eta)J_{old}(i_a\!:\!i_b)+J_{slack}$, $\eta=0.5$, $J_{slack}=2$ s. 채택 시 $Q$에 스플라이스, $c^{old}, G$를 창 안에서 갱신, **C3를 창 $[i_a-2k,\ i_b+2k]$에 양끝 고정(Dirichlet)으로 재실행**해 이음새 곡률 불연속을 없앤다(이음새 κ를 §5.3에서 보고).
5. **유계 에스컬레이션(단일 마감 $D=t_0+400$ ms 공유)**: 튜브 총예산 40 ms. ρ₀에서 경로 **없음**(튜브가 너무 좁음) → ρ₁=3 m 1회. 경로는 **찾았으나 비용 조건 실패** → 더 큰 튜브로 가지 않고 곧장 FULL(새 코스트맵 최적성은 FULL만 보장). FULL의 마감은 $D_1=t_0+340$ ms(튜브 40 ms + 전 격자 최악 288 ms = 328 ms를 덮음), 그 뒤 $D_w=t_0+370$ ms까지 **30 ms**에 $\varepsilon=1.2$ 가중 A\*(anytime 1단계), 그래도 없으면 빈 경로 → BT 복구(감사 보정: 개정판 의사코드는 FULL과 가중 A\*가 같은 마감 D를 써서 FULL이 마감으로 끝나면 가중 A\*가 즉시 종료되는 죽은 코드였다). **후처리 예약(2차 감사)**: 탐색이 400 ms를 다 쓰면 C3 후처리(숏컷·평활·프로파일·게이트 거리 탐색)가 마감 뒤에 돌아 "≤ 400 ms"가 깨진다. 후처리 작업량 상계는 40 m 경로 2.5×10⁶ 조회(5–13 ms), 100 m 6.4×10⁶(13–32 ms)이므로(`checks/out_audit2.txt`; 대부분 §4.3(c)의 나선 탐색) 후처리는 $D=t_0+400$ ms를 **자기 마감**으로 받아 단계별로 검사하고, 넘으면 남은 단계를 건너뛴다(평활 생략 → 숏컷 생략 → 게이트 항 대신 $v_{lim}$만 쓴 프로파일, `mode`에 `post_truncated` 기록). 어느 단계에서 끊겨도 격자 경로 자체는 충돌 없는 유효 경로다. 최악 셀 방문 $6.2\times10^3+2.6\times10^4+9.6\times10^5\approx1.0\times10^6$ → 150–300 ns/셀에서 **150–300 ms**, 후처리 포함 **≤ 400 ms**. 연결성 라벨(8-연결 BFS, O(N), 코스트맵이 바뀐 뒤 첫 FULL 직전에만 갱신)로 도달 불가 시 FULL도 즉시 종료.
6. 드리프트 리셋: 연속 LOCAL의 $\sum(J_{new}-J_{old})>\theta=5$ s 또는 연속 K=5회면 FULL. 또한 마지막 FULL 이후 $T_{full}=10$ s가 지나면 FULL 1회(Nav2 `PathExpiringTimer 10 s`와 같은 취지): 분류는 비용 **증가**만 보므로, 봉쇄가 풀려 더 짧은 경로가 열려도 VALID가 계속 옛 우회를 반환하는 것을 막는다(감사 보정).

보장: FULL 폴백으로 **완전성**; VALID/LOCAL 결과 비용은 *이전 계획 대비* $(1+\eta)$ 유계이고 새 코스트맵 전역 최적 대비 유계는 아님(6단계의 리셋·주기 FULL로 최대 $T_{full}$ 동안만 지속, 리포트 명시); **최악 지연 유계**(5단계). D\* Lite 미채택 사유: 목표가 작업마다 바뀌어 전 격자 초기화가 반복되고 5대×96만 셀 상태 유지가 무거우며 코스트맵 변화가 국소적. 통로 봉쇄처럼 우회가 $(1+\eta)J_{old}+J_{slack}$을 넘는 흔한 경우엔 튜브 시도 ≤ 40 ms가 순손실이라는 [V10]의 대항 가설을 §5.4-3에서 같은 시나리오로 판정한다.

### 4.3 C3 — 비용적분 LOS 숏컷 · 여유거리 평활화 · 속도 프로파일 · 시간 예측

(a) **숏컷**(탐욕 전진 스캔; 초판의 "이진 탐색으로 가장 먼 j"는 술어가 j에 대해 단조가 아니라 정당화되지 않아 철회): $i$에서 $j=i+2,i+3,\dots$로 늘리며 현 $\overline{p_ip_j}$의 supercover 셀이 모두 ≤ 252이고

$$\int_{\overline{p_ip_j}}\frac{ds}{v_{lim}(c(s))}\ \le\ (1+\delta)\,(G_j-G_i),\quad\delta=0.05\ (\text{표본 간격 } r/2)$$

인 동안 진행, 첫 실패에서 $j-1$을 채택하고 $i\leftarrow j-1$. 얻는 숏컷은 **허용적이지만 최대는 아니다**. 현 길이 상한 10 m. 복잡도 $O(\sum\text{현 길이}/r)\le O(M\cdot L_{max}/r)$, 2패스(전진·후진). 비용 인지 단순화는 OMPL PathSimplifier[V31]의 관행이며 우리는 시간 적분을 목적함수로 쓴다. δ=0.05는 1.2 m inflation 띠를 가로지르는 현을 많이 막을 것이므로 회수율은 가정하지 않고 측정한다.

(b) **평활화**: §2.6 Gauss–Seidel 스윕에 여유거리 항 추가

$$y_i\leftarrow y_i+w_d(x_i-y_i)+w_s(y_{i-1}+y_{i+1}-2y_i)+w_c\max\big(0,\ d_{safe}-d_{lo}(y_i)\big)\,\hat n(y_i)$$

$\hat n$은 $d_{lo}(c)$의 중심차분 그래디언트의 단위벡터(c ≥ 1인 셀에서만 정의, 아니면 0), $d_{safe}=r_{circ}+0.10=0.46$ m, $w_c\in[0,1]$, 각 항 단위 m. 비선형이라 §2.6의 선형 수렴 조건은 적용되지 않으며 **보장은 롤백 가드(스윕마다 충돌·곡률 검사, 실패 시 직전 스윕으로)와 최대 반복 30회**뿐이다.

(c) **곡률·속도 프로파일**: 곡률은 **이산 회전각 추정** $\kappa_i=|\Delta\theta_i|\big/\tfrac{\Delta s_i+\Delta s_{i+1}}2$ [1/m](초판의 "Menger" 표기 정정; Menger는 $2\sin\Delta\theta_i/|p_{i+1}-p_{i-1}|$이며 등간격 h에서 $2\sin(\Delta\theta/2)/h\le\Delta\theta/h$이므로 우리 추정이 더 보수적; 부등간격에서도 성립함을 무작위 20만 사례로 확인, `checks/out_misc.txt`). 추종 가능성 기준(명세 4.5 연계): Pure Pursuit 최소 look-ahead $L_{d,min}$에서 명령 곡률 상한 $2/L_{d,min}$이므로 평활 후 **$\kappa_{max}\le\min(2/L_{d,min},\ 2.0)$ [1/m]** (R ≥ 0.5 m), 점 간격 $\Delta s=r\le L_{d,min}/5$를 목표로 둔다. 속도 한계 $v^{lim}_i=\min\big(v_{max},\ \omega_{max}/\kappa_i,\ \sqrt{a_{lat}/\kappa_i},\ v_{lim}(c_i),\ v_{gate}(D_i)\big)$ ($a_{lat}$ 적재 안정 횡가속, 모션 팀 값; 미정 시 항 생략. $v_{gate}$는 §4.1의 `safety_node` 규칙이며 $D_i\approx d_c-r_{ins}$이며 $d_c$는 0.25 m 간격 경로 표본마다 코스트맵 LETHAL 셀을 반경 $r_{ins}+2.6=2.8$ m까지 나선 탐색해 얻는다(40 m 경로 ≈ 160표본 × ≤ 10⁴셀 ≈ 1.6×10⁶ 조회, 수 ms; 전 맵 EDT 10–20 ms를 매 계획마다 돌리지 않음) — 코스트맵 역함수는 $D>R_{infl}-r_{ins}$를 분해하지 못하기 때문이다. c ≥ 1인 표본은 역함수 $d_{hi}(c)$로 충분하므로 탐색하지 않고, c = 0 표본은 $d_c$의 1-립시츠성($d_c(s+\Delta s)\ge d_c(s)-\Delta s$)으로 직전 값 $-\Delta s$ 반경부터 바깥으로만 훑는다(직전이 "2.8 m 안에 없음"이면 2.55–2.8 m 고리 ≈ 1.7×10³셀, 전체 원판의 1/6). 이 단계는 §4.2-5의 후처리 마감 $D$를 따른다. VALID는 저장된 프로파일을 재사용한다. 이 항이 없으면 3 m 통로에서 예측이 36 % 짧다(연속 제한이 등방일 때; 전방 전용이면 측방 여유에는 구역 상한만 적용, §4.1 방향 가정). 경로 위 어느 점에서든 $v_{gate}=0$(가장자리 여유 ≤ 0.3 m, 예: 0.6 m 통로)이면 T_pred를 계산하지 않고 `mode`에 "gate_stop"을 기록해 보고한다). 전진 $v_i=\min(v^{lim}_i,\sqrt{v_{i-1}^2+2a\Delta s_i})$, 후진 $v_i=\min(v_i,\sqrt{v_{i+1}^2+2a\Delta s_{i+1}})$, $v_0=v_{now}$, $v_M=0$. 이 프로파일은 **예측용**이며, 실행 프로파일은 `velocity_profiler_node`(사다리꼴/S-curve + 저크 제한 + PID, components.md §3.3)가 만든다. 두 쪽이 **같은 파라미터 원본** `config/robot_params.yaml limits.*`(v 2.0, a 1.0, ω 1.5, α 2.0 rad/s², jerk 2.0 m/s³)와 `safety.*`를 읽어야 T_pred가 의미를 가진다(플래너 YAML에 값을 복제하지 않는다).

(d) **예측 시간**($\Delta s_i=0$인 구간은 건너뛰고 분모에 $\epsilon=10^{-3}$ m/s 가드):

$$T_{pred}=\alpha_{load}\Big[\tau_{rot}(|\Delta\theta_0|)+\sum_{i:\Delta s_i>0}\frac{2\Delta s_i}{\max(v_{i-1}+v_i,\epsilon)}+n_{stop}\frac{a_{max}}{j_{max}}+\tau_{rot}(|\Delta\theta_M|)\Big],\quad \tau_{rot}(\theta)=\begin{cases}\theta/\omega_{max}+\omega_{max}/\alpha_{max} & \theta\ge\omega_{max}^2/\alpha_{max}\\ 2\sqrt{\theta/\alpha_{max}} & \text{else}\end{cases}$$

$\tau_{rot}$은 `max_angular_acceleration` 2.0 rad/s²를 쓴 사다리꼴 제자리 회전 시간(개정판의 $|\Delta\theta|/\omega_{max}$는 90°에서 1.05 s 대 1.80 s로 0.75 s 과소), $n_{stop}\,a_{max}/j_{max}$는 S-curve 저크 제한(2.0 m/s³)이 정지-출발 쌍마다 더하는 0.5 s(20 m 직진에서 +4 %, 5 m에서 +11 %; `checks/out_misc.txt`), $n_{stop}$은 프로파일의 정지-출발 쌍 수(보통 1). $\alpha_{load}$는 적재 등급별 보정계수(초기 1.0, **학습 분할에서만** 중앙값 $T_{act}/T_{pred}$로 추정, §5.3).

### 4.4 의사코드(코어, ROS 비의존)

```cpp
// grid_astar.hpp — cost: const uint8_t*, W,H,res; speed: LUT[256] -> v_lim
struct Key { float f, h; uint32_t idx; };                       // (f,h) 사전순
std::optional<GridPath> GridAStar::plan(Cell s, Cell g, const Mask* tube, TimePoint deadline) {
  ++gen_; open_.clear(); g_[s]=0; stamp_[s]=2*gen_; parent_[s]=s; push({h_t(s),h_t(s),s});
  while(!open_.empty()){ auto k=pop(); if(stamp_[k.idx]==2*gen_+1) continue;   // closed
    if(k.f > f_of(k.idx)) continue;                                            // lazy deletion
    stamp_[k.idx]=2*gen_+1; if(k.idx==g) return backtrack(g);
    for(auto [nb,len] : neighbors8(k.idx)){                                     // 코너컷 검사 포함
      if(!traversable(cost_[nb]) || (tube && !tube->in(nb))) continue;
      float ng = g_[k.idx] + len / speed_[cost_[nb]];                           // [s]
      if(stamp_[nb] < 2*gen_ || ng < g_[nb]){ g_[nb]=ng; parent_[nb]=k.idx; stamp_[nb]=2*gen_;
        push({ng+h_t(nb), h_t(nb), nb}); } }
    if((++exp_ & 1023)==0 && clock()>deadline) return std::nullopt; }
  return std::nullopt; }
```

```cpp
// astar_planner.cpp — amr_navigation::AStarPlanner::createPlan 흐름 (Smac와 같이 코스트맵 뮤텍스 잠금)
Path createPlan(start, goal){
  std::lock_guard lk(*costmap_->getMutex()); auto t0=steady_clock::now();
  auto D1=t0+340ms, Dw=t0+370ms, D=t0+400ms;                         // FULL / 가중 A* / 후처리 포함 전체 마감
  auto [mode, I] = repair_.classify(state_, costmap, start, goal);  // VALID | LOCAL | FULL, O(M); 목표 변경·T_full 경과 → FULL
  if(mode==VALID) return publish(trim(state_.smoothed, start), mode);
  std::optional<GridPath> q;
  if(mode==LOCAL) q = repair_.tube(state_, I, costmap, /*rho*/{1.0,3.0}, /*budget*/40ms);  // 유클리드 튜브
  if(!q){ mode=FULL; q = astar_.plan(cell(start), cell(goal), nullptr, D1); }
  if(!q && !astar_.unreachable()){ q = astar_.planWeighted(cell(start), cell(goal), 1.2, Dw); } // 30 ms
  if(!q) return {};                                                          // BT recovery
  post_.run(*q, window(mode, I), D);            // shortcut→smooth→profile, D 넘으면 남은 단계 생략(post_truncated)
  state_.store(*q, costmap);                                                 // Q, c_old, G, smoothed
  publish_info({post_.T_pred, length, min_clear, mode, exp_, ms(t0)});
  return toPath(post_.path, costmap_ros_->getGlobalFrameID()); }
```

### 4.5 문헌 대비 위치(정직한 표기)

| 요소 | 가장 가까운 선행 | 주장 |
| --- | --- | --- |
| C1 시간 비용 탐색 | FM²/saturated FM²[V29]; 정지거리 규칙(ISO 3691-4[R15], [V30]); inflation 역함수(Nav2 MPPI ObstaclesCritic); Smac Cost-Aware A\*[V6]; EDF-Lazy Theta\*[V9] | **variant_of_prior** — "Nav2 코스트맵 위 FM²의 공학적 변형". 초판이 새 점으로 적은 "역함수로 EDT 없이"와 "시간 휴리스틱 증명"은 철회(전자는 MPPI에 존재, 후자는 상수 배). 우리 몫: nav2_core 플러그인으로의 구성, 예측 시간과 같은 모델 사용, 경계 불연속·통로 통과의 정량 분석 |
| C2 튜브 복구 | Local Repair A\*(Silver 2005[R8]), 윈도우 탐색, D\* Lite[R4], [V10] | **engineering_adaptation** — 비용 드리프트 의무 재평가, 추종 경로 $P_s$ 기준 충돌 검사, 유클리드 튜브 ρ₀=1 m(명세 4.7 정합), 단일 마감 유계, 드리프트 리셋·주기 FULL을 갖춘 실측 비교 |
| C3 후처리 | OMPL PathSimplifier[V31], APP[V15], Dolgov CG 평활화[R9], 경로-속도 분해(고전), Nav2 smoother | **variant_of_prior** — 시간 적분 채택 기준, inflation 역함수 여유거리 항, 추종 가능성 기준과 프로파일→예측 시간 |
| 전체 | — | **new_combination**(세 요소를 창고 AMR·Nav2 문맥에서 함께 다루고 명세 지표로 검증한 공개 사례를 찾지 못함) |

### 4.6 예상 효과(가설)와 리스크

| 지표 | 기준(Smac2D κ=1 / NavFn) | TAR-A\* 가설 |
| --- | --- | --- |
| 전역 계획 시간 | p95 30–80 / 30–60 ms | 새 목표 FULL(후처리 포함) 평균 ≤ 50 ms(sequences.md §1 첫 `cmd_vel` 200 ms 예산의 A\* 몫), p95 ≤ 100 ms(LUT 256로 나눗셈 상쇄). 평균 50 ms를 못 맞추면 sequences.md 예산 재배분을 요청 |
| 재계획 시간 D30 | 전역 재탐색과 동일 | VALID ≤ 0.5 ms; LOCAL p50 ≤ 5 ms(≥ 60 % 시나리오); **최악(FULL 포함) ≤ 400 ms** |
| 경로 길이 | Smac2D 기준 | 구성 A: +0 … +6 %(여유 우선), 숏컷 후 재측정 — 가설, 근거 없음 |
| 여유거리 | κ=1 기준 | 평균 $d_c$ +30 %, 열린 구역 min $d_c$ ≥ 0.46 m, 통로는 중앙 ±2 cm |
| 예측 시간 오차(검증 분할) | $L/v_{max}$: 30–50 % | ≤ 15 % |
| 연속 계획 간 거리 | 큼 | LOCAL(ρ₀): 격자 경로 간 Hausdorff ≤ 1 m(유클리드 튜브, 구성상); 이산 Fréchet은 단조 대응이 보장되지 않아 측정·목표 ≤ 1 m |
| 5대 CPU | 플래너 5×60 ms/s ≈ 0.3 코어 | 플래너 ≈ 0.05 코어(VALID 5 Hz ≈ 0.013 + 주기 FULL 0.1 Hz×60 ms×5 ≈ 0.03 + 새 목표·LOCAL) + **코스트맵 5 Hz ≈ 0.08–0.25 코어(추정, §6)** |

리스크: (1) 구성 A의 경계 불연속으로 열린 구역 우회 → 구성 B 또는 $s$ 조정; (2) 튜브 복구의 누적 우회·비용 감소 미반영 → 6단계 리셋·주기 FULL; (3) 시간 예측은 실행 프로파일 공유에 의존 → §7-3을 요구사항으로 격상; (4) inflation 역함수 전제(§1.1-2)가 깨지면 C1이 무의미 → 시작 시 코스트맵 플러그인·필터 목록, `footprint_padding`, keepout 마스크 이진성 검증·경고; (5) 안전 게이트(§4.1)가 바뀌면 T_pred 모델도 바뀜 → 시작 시 `robot_params.yaml safety.*`를 읽어 로그에 기록.

### 4.7 검토 후 보류한 후보

- **JPS/JPS+**: 균일 비용 전제. "c=0 영역만 점프"는 이득 수 ms 대비 복잡도 큼.
- **Theta\*/Lazy Theta\*/Field D\* 탐색**: 비균일 비용과 양립하나(§3.7-2) 완전성·최적성 설명이 복잡하고 Field D\*는 Nav2 비교군이 없다. 후처리 숏컷이 길이 이득을 회수할 것으로 기대하나 가정하지 않고 §5.4-5 어블레이션으로 측정한다.
- **D\* Lite/LPA\***: §4.2 사유. 벤치 대조군으로 간이 구현(선택).
- **가중 A\*/ARA\*/Focal**: 최적성 상실이 여유·시간 예측을 흔듦. 마감 임박 시 ε=1.2 폴백으로만.
- **방향성 속도맵**(헤딩 방향 여유거리): 상태가 (x,y,θ)로 커짐. 2단계.
- **코리도 그래프(V5)·HPA\***: fleet lane 예약과 결합 시 재검토.

## 5. 평가 계획(명세 지표 매핑)

### 5.1 비교 대상과 설정

운영 설정 `src/amr_navigation/config/nav2_params.yaml`(components.md §6)의 planner_server는 components.md §5.3 그대로 `planner_plugins: [AStar, NavFn, Smac]`(`AStar` = `amr_navigation/AStarPlanner`, `NavFn` use_astar=false, `Smac` = SmacPlanner2D κ=1.0). 벤치 전용 `benchmark/nav2_params_bench.yaml`은 여기에 `NavFnAStar`(use_astar=true), `SmacK2`(κ=2.0), `ThetaStar`를 더하고, 별도 표용 `SmacHybrid`(REEDS_SHEPP, min_turning_radius 0.4)를 둔다(운영 `planner_id`는 바꾸지 않는다). §1.1 전제 공통. 호출 `compute_path_to_pose{planner_id}`; `planning_time`과 플러그인 내부 `steady_clock`(sim 배율 무관)을 함께 기록. 코스트맵: `update_frequency 5.0`, `publish_frequency 1.0`, `always_send_full_costmap false`; obstacle 관측원 `scan_filtered`(sensors.yaml LiDAR 10 Hz, `range_max` 25 m) `observation_persistence 0.0`(최신 스캔만 → 유령 최소), **`expected_update_rate 0.3`**(단위 초; = `robot_params.yaml safety.sensor_timeouts.lidar`. 이 값을 넘으면 코스트맵이 not-current가 되어 planner_server `waitForCostmap()`이 계획을 **블록**하므로 개정판의 0.2 s(2주기)는 DDS 지터 한 번에 계획을 멈춘다 — 경보가 아님), `obstacle_max_range 10.0`, `raytrace_max_range 12.0`(통로를 따라 유령 셀을 빨리 지움), `marking/clearing true`, `obstacle_min_range 0.05`; 국소 코스트맵의 유지 시간(0.5 s)은 DWA 팀 담당. 재계획 BT: 프로젝트 기준(components.md §3.3, sequences.md §2)은 `RateController 1 Hz` + `IsTTCBelowThreshold` 즉시 재계획이다. 이 문서는 VALID가 O(M)이라 싸므로 **`RateController hz=5`로 올리는 변경을 요청**하며(§7-8), 두 설정 모두에서 e2e 지연을 보고한다(§5.3).

### 5.2 데이터셋

- **P50-U**: 명세용 50쌍, 시드 고정, $c\le252$ 셀 위 **균등 무제한** 샘플링(거리 필터 없음). **P50-R**: $c\le128$·유클리드 ≥ 10 m 기각 샘플링(끝점이 inflation 깊숙이 있거나 인접한 쌍은 플래너가 아니라 작업 정의의 문제라는 판단; 두 결과를 모두 보고하고 실패 원인을 분류). **P500** 확장, **H10** 수작업(0.6 m 통로, 막다른 통로, 도킹 접근, 대각 횡단, 열린 구역 기둥).
- **D30**: 동적 장애물 30 시나리오(0.3–1.5 m/s 작업자/지게차 횡단·봉쇄) — 회피 팀과 공유, 적재 등급 3종 균등.

### 5.3 지표·정의·목표

| 지표 | 산출 | 목표/근거 |
| --- | --- | --- |
| 성공률 | 비어 있지 않은 경로 & 양끝 tolerance 내 | ≥ 98 %(P50-U, P50-R 각각) |
| 계획 시간 | p50/p95/**max**, VALID/LOCAL/FULL 분리 | 플러그인 ≤ 400 ms(p100) |
| **재계획 500 ms 시계** | (i) $T_{plugin}$ = `planning_time`(액션 수신→결과; `waitForCostmap()` 대기 포함, sequences.md §2의 측정법과 동일); (ii) $T_{e2e}$ = 기존 경로의 풋프린트 회랑 안에 처음 반환점이 들어온 `scan_filtered`의 헤더 시각(rosbag에서 오프라인 산출, Gazebo 참값으로 교차 확인) → 새 `plan` 헤더 시각(플러그인이 `now()`로 설정). 토픽은 로봇 네임스페이스 상대 이름(`/<r>/scan_filtered`, `/<r>/plan`, 예: `/amr_01/plan`; components.md §4.2의 `<r>`). 개정판의 시작점 `costmap_updates` 헤더는 Humble에서 stamp가 0이고 1 Hz로만 발행되어 쓸 수 없다(§1 표) | (i) ≤ 500 ms(명세 해석). (ii) 보고: BT 5 Hz(요청안)에서 최악 200(코스트맵 5 Hz) + 200(BT) + 400(마감) = 800 ms, FULL p95 기준 500 ms → 목표 p95 ≤ 700 ms·p100 ≤ 800 ms; 프로젝트 기준 BT 1 Hz에서는 최악 1.6 s(주기 재계획), TTC 트리거 재계획은 별도 집계. 조건을 리포트에 명기 |
| 경로 길이 | $\sum\|p_{i+1}-p_i\|$ | Smac2D 대비 % |
| **여유거리** | 정적 맵 EDT로 각 점의 중심-장애물 거리 $d_c$(명세 비교 지표, 플래너 간 비교 가능); 풋프린트 가장자리 여유 $d_e\in[d_c-r_{circ},\ d_c-r_{ins}]$(통로 직진 시 $d_e=d_c-r_{ins}$) | 열린 구역 min $d_c$ ≥ 0.46 m(= 어느 자세든 가장자리 ≥ 0.10 m); 폭 w < 1.2 m 통로는 $|d_c-w/2|\le0.02$ m. **안전팀 전달(현재 설정과의 충돌)**: `robot_params.yaml safety`는 이미 `distance_reference: footprint_edge`, E-stop 0.30 m를 스캔 점 전체에 등방 적용한다(설정 주석에 정적 구조 제외 규정이 없음; 추적기만 `/map` 배경을 뺀다) → 0.6 m 통로 중앙($d_e$ = 0.10 m)은 **E-stop**, 1.2 m 통로는 Critical(0.2 m/s), 2.4 m 통로는 1.04 m/s(§4.1 표; 연속 제한이 등방일 때, 전방 전용이면 2.0 m/s). 명세 4.4 "폭 + 20 cm 통로 통과"와 양립 불가 → 정적 구조 제외 전방 보호영역(폭 = 풋프린트 폭 + 2×0.05 m, 길이 = $v\,t_r+v^2/2a$ + 0.3 m) 또는 동적 장애물 전용 규칙이 필요하다. 결정 전까지 H10의 0.6 m 통로 시험은 "플래너 경로 생성"과 "주행 통과"를 분리 보고 |
| 부드러움·추종성 | 헤딩 변화 > 10° 횟수, $\max\kappa$, 이음새 κ, $\sum|\Delta\theta|$ | $\max\kappa\le2.0$ 1/m, 이음새 κ ≤ 2.0 |
| **예측 시간 오차** | $|T_{act}-T_{pred}|/T_{act}$; $T_{act}$ = 경로 발행 후 첫 이동 → 목표 tolerance 도달, 도킹 제외, 회피 정지 포함(제외 값도 병기); 적재 등급별 | ≤ 15 % — **$\alpha_{load}$는 학습 분할(등급별 8회)에서만 추정, 검증 분할(등급별 ≈ 9회)에서만 오차 보고**; 보정 전 값 병기 |
| 경로 안정성 | 연속 계획 간 Hausdorff·이산 Fréchet 거리(D30), 모드별 | LOCAL(ρ₀) Hausdorff ≤ 1 m(구성상), Fréchet ≤ 1 m(목표) |
| **회피 이탈·복귀(명세 4.7)** | 복구 경로와 원 경로의 창 내 최대 이격; 실제 궤적이 원 경로에서 0.2 m 넘게 벗어난 시점 → 0.2 m 이내 복귀 시점 | ≤ 1 m, ≤ 5 s(D30, 회피 팀과 공동) |
| 자원 | 플러그인 RSS, 5대 동시 CPU(플래너·코스트맵 분리) | ≤ 15 MB/인스턴스; 합계 예산 §6 |

로그(`logs/planning/*.csv`): `[timestamp, planner_id, pair_id, success, plan_time_ms, wall_ms, length_m, min_dc_m, mean_dc_m, n_turns, max_curv, seam_curv, T_pred_s, mode, expansions, tube_cells, load_class]`; 주행 로그 `[task_id, load_class, split, T_pred, T_actual, T_actual_noavoid, err]`; 재계획 로그 `[t_scan_hit, t_action_goal, t_plan_pub, planning_time_ms, bt_rate_hz, trigger(rate|ttc), mode, detour_m, return_s]`(`t_scan_hit` = §5.3 (ii)의 시작 시각).

### 5.4 절차와 재현

1. 오프라인 마이크로벤치(gtest/benchmark): PGM → Costmap2D → 코어 직접 호출, 3회 중앙값, 결정론(경로 해시).
2. 온라인 벤치 `ros2 launch amr_navigation planner_benchmark.launch.py planners:=all pairs:=benchmark/pairs_50.yaml` → `benchmark/analyze.py` — **동료 평가자가 한 명령으로 재현**(README에 명시).
3. D30: TAR-A\*(LOCAL 허용) vs TAR-A\*(FULL 강제) vs Smac2D: 재계획 시간 분포·안정성·이탈·복귀. (선택) 간이 D\* Lite.
4. 주행 50회를 적재 등급별 학습/검증으로 분할해 $\alpha_{load}$ 추정·오차 산출(누수 없음).
5. 어블레이션: C1 끔(κ=1), C2 끔, C3 각 항 끔(숏컷 없음 → 8-연결 초과율 실측).
6. 구성 실험: inflation {A: 1.2/2.0, B: 2.2/2.0, C: 2.8/2.0 + `speed_rule: gate`(§4.1), Nav2 예시 0.55/3.0} × P50 — 길이·여유·코스트맵 CPU, 그리고 D30 주행에서 예측 시간 오차.

## 6. 구현 계획

- **언어/패키지**: C++17, 기존 `src/amr_navigation`(ament_cmake + python) 확장. Python은 벤치·분석만.
- **의존**(package.xml): 기존 `rclcpp`, `nav2_core`, `nav2_costmap_2d`, `nav2_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `amr_msgs`에 `pluginlib`, `rclcpp_lifecycle`, `nav2_util`, `std_msgs`, `builtin_interfaces` 추가; 테스트 `ament_cmake_gtest`.
- **파일**
  - `include/amr_navigation/global/{grid_types,speed_map,grid_astar,tube_repair,path_postprocess,astar_planner}.hpp`, `src/global/*.cpp`
  - `plugins/global_planner_plugin.xml` → `<class name="amr_navigation/AStarPlanner" type="amr_navigation::AStarPlanner" base_class_type="nav2_core::GlobalPlanner">`(components.md §3.3 이름); CMake `pluginlib_export_plugin_description_file(nav2_core plugins/global_planner_plugin.xml)`
  - 파라미터는 components.md §6대로 `src/amr_navigation/config/nav2_params.yaml`의 `planner_server.AStar.*`: `speed_rule brake|gate, v_floor 0.2, a_lat, a_max_by_load{small,medium,large}(미정 시 limits 값), tube_radius[1.0,3.0], tube_budget_ms 40, full_deadline_ms 340, weighted_deadline_ms 370, deadline_ms 400(후처리 포함), t_full_s 10, eta_c 0.5, eta_v 0.1, eta 0.5, j_slack 2.0, k_margin_m 1.0, l_max_m 15, delta_shortcut 0.05, max_chord_m 10, w_data, w_smooth, w_clear, d_safe 0.46, kappa_max 2.0, max_its 30, allow_unknown`. **운동 한계·안전 규칙은 복제하지 않고** `config/robot_params.yaml`(`limits.*`, `safety.*`, `robot.footprint_*`)을 시작 시 읽는다; `cost_scaling_factor·inflation_radius`는 코스트맵 파라미터와 비교해 불일치 시 경고
  - 인터페이스 추가(components.md §5.3 갱신 요청, §7-8): `amr_msgs/msg/PlanInfo.msg`(신규; 현재 amr_msgs에 없음) `float64 predicted_time, length, min_clearance; string mode; uint32 expansions, tube_cells; float64 plan_time_ms` → planner_server Pub `plan_info`; 플러그인 Sub `payload/mass`(`std_msgs/msg/Float32`, latched)로 적재 등급 선택
  - 테스트: `test_speed_map.cpp`(구간 역함수·단조성·경계·LUT), `test_grid_astar.cpp`(무작위 격자에서 Dijkstra와 비용 동일성, 휴리스틱 일관성 수치검사, 코너컷, 결정론, 마감), `test_tube_repair.cpp`(비용 드리프트가 LOCAL을 유발, $P_s$ 숏컷 현만 막힌 경우 검출, 목표 변경·$T_{full}$ → FULL, $(1+\eta)$ 유계, 대각 경로에서도 격자 경로 이탈 ≤ ρ₀, 창 재후처리 숏컷 현의 튜브 제한, 에스컬레이션 순서, FULL 마감 후 가중 A\* 실행, 후처리 포함 400 ms 마감 준수·`post_truncated` 경로 유효성, 이음새 연속성), `test_postprocess.cpp`(숏컷 무충돌·시간 비악화·비단조 술어 사례, 평활화 롤백, 프로파일 가감속·곡률·$v_{gate}$ 제약, $\tau_{rot}$·저크 항, $\Delta s=0$ 가드, 합성 경로 T_pred), `test_planner_plugin.cpp`(LifecycleNode + 합성 Costmap2DROS)
  - `benchmark/{planner_benchmark.launch.py, nav2_params_bench.yaml, bench_planners.py, pairs_50.yaml, analyze.py}`, `docs/algorithms/astar.md`
- **코스트맵 설정**(같은 `nav2_params.yaml`의 `global_costmap` 블록, 리뷰 필수 수정 9): §1.1의 footprint 폴리곤·`footprint_padding 0.0`·inflation 구성 A(1.2/2.0), §5.1의 obstacle 관측원 `scan_filtered` `observation_persistence 0.0`·`expected_update_rate 0.3`·`obstacle_max_range 10.0`·`raytrace_max_range 12.0`·`marking/clearing true`, `update_frequency 5.0`·`publish_frequency 1.0`. 플러그인은 시작 시 이 값을 읽어 §4.6 리스크 (4)의 전제 검사에 쓴다.
- **Nav2 연동**: 프로젝트 BT(`RateController` 1 Hz + `IsTTCBelowThreshold`)에서 동작하며, 5 Hz로의 변경은 BT 담당에 요청(§5.1, §7-8). 코스트맵 뮤텍스는 `createPlan` 범위. IsPathValid 미사용(Humble에서 인덱스 미제공).
- **일정(주)**: 1 코어 A\*+속도맵+테스트 → 2 플러그인·벤치 파이프라인·구성 실험 → 3 C3·시간 예측 → 4 C2·D30·어블레이션 → 5 리포트·튜닝.
- **연산 예산**: 플래너 전역 p95 ≤ 100 ms, LOCAL p50 ≤ 5 ms, 최악 400 ms, 메모리 ≤ 15 MB/인스턴스, 5대 플래너 ≤ 0.1 코어(주기 FULL과 연결성 라벨 BFS 포함; BFS는 코스트맵이 바뀐 뒤 첫 FULL 직전에만 O(N) ≈ 수 ms). **코스트맵**(리뷰 반영): 5 Hz 전체 맵 갱신은 obstacle 층 레이트레이싱(720빔 × 240셀 ≈ 1.7×10⁵) + 갱신 창 inflation(≈ (24+2.4 m)²/r² ≈ 2.8×10⁵ 셀) ≈ 3–10 ms/갱신 → 5대×5 Hz ≈ **0.08–0.25 코어(추정, §5.4-6에서 실측)**; 초과 시 rolling_window 24 m 전역 코스트맵 + 정적 맵 계획으로 후퇴.

## 7. 미결 질문

1. 최종 맵의 통로 폭 분포 → $R_{infl}, s, v_{floor}$ 튜닝(구성 A/B 선택).
2. 코스트맵 5 Hz 전체 맵의 실측 CPU(§6 추정 검증).
3. **[요구사항으로 격상]** 실행 체인(DWA/PurePursuit 플러그인 → `velocity_profiler_node` → `safety_node`)이 §4.3(c) 예측 프로파일과 같은 원본(`robot_params.yaml limits.*`, `safety.*`)을 쓰는가, 적재 등급별 $a_{max}$를 모션 팀이 정의하는가(현재 전 등급 1.0) — 15 % 목표의 전제.
4. $T_{act}$ 정의는 §5.3에 고정; 리포트 팀 승인 필요.
5. 안전팀: `robot_params.yaml`이 이미 가장자리 기준 등방 적용(정적 구조 제외 규정 없음)으로 정해져 0.6 m 통로가 E-stop이 된다(§4.1, §5.3). 정적 구조 제외 또는 전방 보호영역으로 바꿀지 — 명세 4.4 통로 요구와 4.7 안전거리 요구의 충돌 해소 주체 결정 필요. 아울러 `clearance_speed_limit_enabled`의 연속 제한 $v_{max}(D)$가 등방 최단거리 $D$에 걸리는지, 주석의 "전방 여유"대로 진행 방향에만 걸리는지 확정 필요 — C1 하한 논증(§4.1)과 T_pred 게이트 항(§4.3c)이 이 답에 따라 바뀐다.
6. amr_perception `perception/tracked_obstacles`의 예측 궤적을 전역 코스트맵 층으로 넣을지(LOCAL 비율·유령 셀 영향). sequences.md §2는 TTC 트리거 재계획이 "예측 위치 반영"이라고 가정하므로, 넣지 않으면 그 문구를 고쳐야 한다(층을 넣을 경우 이진 254로만 표시하고 inflation 층 앞에 두어 §1.1-2를 지킨다 — 중간값은 inflation 앞이든 뒤든 역함수를 깬다).
7. fleet lane 선호를 `extra_cost[N]`($w_t$ 가산) 훅으로 받을지 — 중간값으로 코스트맵에 그리면 §1.1-2가 깨진다(이진 keepout은 허용).
8. **components.md·sequences.md 변경 요청**(이 문서가 전제하는 것): (a) 재계획 BT `RateController` 1 → 5 Hz; (b) planner_server Pub `plan_info`(`amr_msgs/msg/PlanInfo`, 신규), 플러그인 Sub `payload/mass`; (c) 교통 관리자 `keepout_mask`는 0/100 이진 + $r_{circ}$ 팽창; (d) sequences.md §1의 "A\* ≤ 50 ms"는 새 목표 FULL 평균으로 해석(§4.6).

## 8. 참고문헌

### VERIFIED (초록/본문/소스 페이지를 직접 가져옴)

- [V1] Y. Zou, C. Borst, "Optimal any-angle path planning in static and dynamic environments," arXiv:2607.00065, 2026. https://arxiv.org/abs/2607.00065
- [V2] Y. Zou, C. Borst, "Zeta\*-SIPP," IJCAI 2024. https://www.ijcai.org/proceedings/2024/754
- [V3] S. Talia, O. Salzman, S. Srinivasa, "Incremental Generalized Hybrid A\*," RA-L 2025, arXiv:2508.13392 (6×는 자체 HA\*M 대비). https://arxiv.org/abs/2508.13392
- [V4] P. Pastorelli et al., "Fast Shortest Path Polyline Smoothing With G¹ Continuity and Bounded Curvature," RA-L 10(4) 2025. https://arxiv.org/abs/2409.09816
- [V5] A. Gonzalez-Garcia et al., "…Rectangular Corridor Representation…," ICRA 2026. https://arxiv.org/abs/2602.09714
- [V6] S. Macenski, M. Booker, J. Wallace, T. Fischer, "Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics," arXiv:2401.13078 (본문 HTML v2 확인: 38 %/50 %는 Hybrid-A\*·Lattice vs NavFn/2D-A\*, 임의 맵 10–20 % 밀도; 2.5 %는 최적 길이 대비; 창고 33 600 m² 2D-A\* 1 358 ms, 10회 평균; 맵 해상도 미기재 — 감사 시 재확인). https://arxiv.org/abs/2401.13078
- [V7] Z. Shen et al., "Motion Planning in Dynamic Environments: A Survey…," arXiv:2606.02677, 2026. https://arxiv.org/abs/2606.02677
- [V8] Y. K. Lai, "Rapid Vector-based Any-angle Path Planning…," PhD thesis, arXiv:2408.05806. https://arxiv.org/abs/2408.05806
- [V9] J. A. Cobano, L. Merino, F. Caballero, "Exploiting Euclidean Distance Field Properties… Lazy Theta\*," RAS 2025. https://arxiv.org/abs/2505.24024
- [V10] M. E. C. Sabbadini et al., "Revisiting Replanning from Scratch…," ICRA 2026. https://arxiv.org/abs/2510.21074
- [V11] I. Ibrahim et al., "Exact Wavefront Propagation…," RA-L 9(11) 2024. https://arxiv.org/abs/2409.11545
- [V12] J. Baum, "Jump Point Search Pathfinding in 4-connected Grids," arXiv:2501.14816. https://arxiv.org/abs/2501.14816
- [V13] V. Reijgwart et al., "Efficient Hierarchical Any-Angle Path Planning on Multi-Resolution 3D Grids," RSS 2025. https://arxiv.org/abs/2602.21174
- [V14] K. Yakovlev, A. Andreychuk, R. Stern, "Optimal and Bounded Suboptimal Any-Angle MAPF," IROS 2024. https://arxiv.org/abs/2404.16379
- [V15] Y. Li, H. Cheng, "APP: A\* Post-Processing Algorithm…," RA-L 8(11) 2023. https://arxiv.org/abs/2511.13042
- [V16] S. Zhao, D. Harabor, P. J. Stuckey, "Reducing Redundant Work in Jump Point Search," arXiv:2306.15928. https://arxiv.org/abs/2306.15928
- [V17] M. V. Duc et al., "Probabilistic Focal Search…," arXiv:2609.10584, 2026 (N-Puzzle·Pancake·TSP·GCTSP 실험, 격자 맵 없음; ≈ 90 %는 N-Puzzle·TSP). https://arxiv.org/abs/2609.10584
- [V18] S. S. Shperberg et al., "Bidirectional Bounded-Suboptimal Heuristic Search with Consistent Heuristics," arXiv:2511.10272. https://arxiv.org/abs/2511.10272
- [V19] S. Espahbodi Nia, "FMT^X: Lazy Wavefront Search for Dynamic Replanning," arXiv:2509.08521. https://arxiv.org/abs/2509.08521
- [V20] Y. Tang, M. A. Zakaria, M. Younas, "Path Planning Trends for AMR Navigation: A Review," Sensors 25(4):1206, 2025. https://pmc.ncbi.nlm.nih.gov/articles/PMC11861809/
- [V22] C. Zang et al., "GRACE…," ICRA 2026. https://arxiv.org/abs/2603.10858
- [V23] Z. Chen, Y. Li, "FDSPC…," arXiv:2405.03281. https://arxiv.org/abs/2405.03281
- [V24] haris-mujeeb, "Comparing Global Planners for navigation," Open Robotics Discourse, 2026-05 (**일화적**; 맵 크기 미기재, chfritz 반박 댓글 포함). https://discourse.openrobotics.org/t/comparing-global-planners-for-navigation/54457
- [V25] Nav2 `humble` 소스(raw 확인): `nav2_theta_star_planner/src/theta_star_planner.cpp`, `include/.../theta_star.hpp`, `nav2_smac_planner/src/{node_2d.cpp, smac_planner_2d.cpp, smoother.cpp}`, `nav2_navfn_planner/src/navfn.cpp`, `nav2_planner/src/planner_server.cpp`, `nav2_costmap_2d/src/costmap_2d_ros.cpp`, `nav2_costmap_2d/plugins/{inflation_layer, obstacle_layer, static_layer}.cpp`, `nav2_mppi_controller/src/critics/obstacles_critic.cpp`. https://github.com/ros-navigation/navigation2/tree/humble ; Smac 2D 문서 https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/
- [V26] S. Macenski et al., "From the Desks of ROS Maintainers…," RAS 2023. https://arxiv.org/abs/2307.15236
- [V27] M. H. Ha et al., "MMP-A\*…," arXiv:2601.01910, 2026. https://arxiv.org/abs/2601.01910
- [V28] D. Ferguson, A. Stentz, "Field D\*: An Interpolation-based Path Planner and Replanner," ISRR 2005 (CMU RI 페이지 초록 확인). https://publications.ri.cmu.edu/field-d-an-interpolation-based-path-planner-and-replanner/ ; 확장판 "Using Interpolation to Improve Path Planning: The Field D\* Algorithm," JFR 23(2):79–101, 2006 (검색 스니펫 초록). https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.20109
- [V29] A. Valero-Gómez, J. V. Gómez, S. Garrido, L. Moreno, "The Path to Efficiency: Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories," IEEE RAM 20(4):111–120, 2013, doi:10.1109/MRA.2013.2248309 (서지·DOI는 검색 결과로 확인; 같은 저자의 프리프린트 "Fast Marching Methods in Path Planning" PDF 본문 확인: saturated FM², FM²\*, "trajectory + control speed, optimal in completion time"). https://jvgomez.github.io/files/pubs/fm2star.pdf
- [V30] S. Mitsch, K. Ghorbal, D. Vogelbacher, A. Platzer, "Formal Verification of Obstacle Avoidance and Navigation of Ground Robots," IJRR 36(12):1312–1340, 2017 (초록 확인: passive safety·passive friendly safety; 정지거리형 불변식의 구체 형태는 본문 기억 = RECALLED). https://arxiv.org/abs/1605.00604
- [V31] OMPL, `ompl::geometric::PathSimplifier` 문서(생성자 인자 `OptimizationObjectivePtr`; 목적함수는 `reduceVertices` 외 모든 메서드에 사용 — 2차 감사 재조회). https://ompl.kavrakilab.org/classompl_1_1geometric_1_1PathSimplifier.html

### S (검색 스니펫만 확인)

- [S21] "Research on Mobile Robot Path Planning Based on an Improved Bidirectional Jump Point Search Algorithm," Electronics 14(8):1669, 2025-04 (MDPI 페이지 403). https://doi.org/10.3390/electronics14081669

### RECALLED (고전; 페이지 미열람)

- [R1] Hart, Nilsson, Raphael, IEEE TSSC 1968 (A\*). [R2] Pohl, AIJ 1970 (weighted A\*). [R3] Likhachev, Gordon, Thrun, NeurIPS 2003 (ARA\*).
- [R4] Koenig, Likhachev, "D\* Lite," AAAI 2002; Koenig, Likhachev, Furcy, "LPA\*," AIJ 2004.
- [R5] Nash, Daniel, Koenig, Felner, "Theta\*," AAAI 2007 / JAIR 2010; Nash, Koenig, Tovey, "Lazy Theta\*," AAAI 2010.
- [R6] Harabor, Grastien, JPS AAAI 2011; JPS+ ICAPS 2014. [R7] Botea, Müller, Schaeffer, HPA\* 2004.
- [R8] D. Silver, "Cooperative Pathfinding," AIIDE 2005 (Local Repair A\*, WHCA\*).
- [R9] Dolgov, Thrun, Montemerlo, Diebel, IJRR 2010 (Hybrid A\* + CG 평활화).
- [R10] S. Garrido, L. Moreno, D. Blanco, "Voronoi diagram and fast marching applied to path planning," ICRA 2006, pp. 3049–3054, doi:10.1109/ROBOT.2006.1642165; S. Garrido, L. Moreno, M. Abderrahim, D. Blanco, "FM²: A real-time sensor-based feedback controller for mobile robots," IJRA 24(1), 2009 (초판의 2011 ICRA DOI는 확인 불가라 삭제. 두 서지는 [V29] PDF 참고문헌 [6]·[7]과 검색 결과로 확인, 원문 미열람).
- [R11] Sethian, PNAS 1996; Felzenszwalb, Huttenlocher, ToC 2012. [R12] Uras, Koenig, SoCS 2015. [R13] Macenski et al., Marathon 2, IROS 2020. [R14] Phillips, Likhachev, SIPP, ICRA 2011.
- [R15] ISO 3691-4:2020, "Industrial trucks — Safety requirements and verification — Part 4: Driverless industrial trucks and their systems" (제목·연도는 iso.org 검색으로 확인, 본문 미열람). https://www.iso.org/standard/70660.html

### 도구 가용성 메모

WebSearch/WebFetch 사용 가능. arXiv export API는 429로 실패해 UI/abs/HTML 본문으로 대체. GitHub `humble` raw 소스는 curl·WebFetch 모두 성공. iso.org·MDPI·IEEE Xplore 403, Semantic Scholar API 429 → 해당 항목은 S/RECALLED로 강등하거나 2차 페이지(저자 사이트 PDF)로 확인.

## 9. 리뷰 반영 이력 (2026-09-22)

**수학 오류 정정**

1. §2.2 Theta\* 행: 가중치 뒤바뀜과 아핀 변환 누락을 정정(1.0·유클리드 + Σ 2.0·((26+0.9c)/252)², 휴리스틱 1.0·유클리드), "느슨" 표현 삭제. NavFn "A\* 모드 = 임계값 버킷 파면" 주석 추가.
2. §2.6: Smac 스무더가 제자리 Gauss–Seidel임을 소스로 확인, 수렴 조건을 SOR(Ostrowski–Reich) $0<w_d+2w_s<2$로 교체. Jacobi 조건은 충분조건으로만 언급. C3(b) 비선형 항에는 보장 없음을 명시.
3. §4.2-3: 튜브 셀 수 $(2L\rho+4\rho^2)/r^2$로 정정(예 2.9×10⁴). 에스컬레이션을 단일 마감·튜브 예산 40 ms·ρ₀=1 m로 재설계해 최악 ≈ 1.0×10⁶ 셀(150–300 ms)로 유계.
4. §2.2/§4.1: floor 구간 역함수 도입, 보수적 끝점 $d_{lo}$ 사용, 양자화 폭 표기.
5. §4.3(c): "Menger" → 이산 회전각 곡률로 개명, Menger와의 관계(우리 추정이 보수적) 명시.
6. §4.3(a): 이진 탐색 → 탐욕 전진 스캔, "허용적이나 최대 아님" 명시, 복잡도 재기술.
7. §4.1: 예시 표를 권고 구성 $s=2$, $R_{infl}=1.2$로 재계산(통로 중앙 c=206, 승수 4.51), 경계 29.7 %(감사 시 29.8 → 29.7 정정) 불연속과 우회 상한(≈ 1.7 m) 분석, 구성 B(2.2 m, 연속) 추가.
8. §4.1: "주행시간 하한"을 실행 정책 가정부로 한정, $v_{lim}$을 "등방 정지거리 대용물"로 재기술(감사 시 "보수적" 삭제 — 프로젝트 안전 게이트보다 느슨함), ISO 3691-4·Mitsch 인용.

**제안 재배치**

- C1: "새 점"(inflation 역함수, 휴리스틱 증명) 철회 → MPPI ObstaclesCritic·FM²·ISO 3691-4 인용, 라벨 "Nav2 코스트맵 위 FM²의 공학적 변형". 길이 −2…+3 % 가설 철회, 측정 항목화. 역함수 전제(trinary·keepout 금지) 명시.
- C2: 비용 드리프트 재평가를 의무화(VALID 맹점 제거), $J_{old}$를 저장 접두합으로 정의, off-grid 발행 경로와 격자 경로를 분리 저장·창 재후처리로 이음새 처리, 최악 지연을 §4.6에 보고 항목으로 추가. 초판의 ρ₀=3 m를 1 m로 낮춰 명세 4.7 이탈 ≤ 1 m와 정합.
- C3: 그대로 "variant_of_prior"이되 OMPL PathSimplifier를 선행으로 추가, $\Delta s=0$·분모 가드, $a_{lat}$ 결합 옵션, 추종 가능성 기준($\kappa_{max}$, 점 간격) 추가, α는 학습/검증 분할로.
- 전체: §3.7(2) "any-angle은 균일 비용 전제" 주장 철회, Field D\* 추가, A\*+후처리 근거를 완전성·설명 가능성·비교군 부재로 교체. V24는 일화적으로 강등.

**명세 공백 보완**: 장애물 층 갱신·유지 파라미터(§5.1), 추종 가능성 기준(§4.3c), 500 ms 시계 정의와 BT/코스트맵 조건(§5.3), α 검증 분할·적재 등급(§4.3d, §5.3), 풋프린트·trinary·tolerance 전제(§1.1), 여유거리 정의와 안전팀 전달(§5.3), 코스트맵 CPU 예산(§6), 4.7 이탈·복귀 지표(§5.3), P50 무제한/제한 병행(§5.2), 재현 명령(§5.4-2).

**인용 정정**: V6 본문 수치와 맥락, V3·V17 적용 범위 한정, V21→S21 강등, R10 DOI 삭제·원저 교체, V24 caveat, 코드 기본값 vs bringup 예시값 분리, IsPathValid 인덱스 미제공 사실 정정.

**리뷰어와 다르게 유지한 점**

1. 리뷰는 "경로는 찾았으나 비용 조건 실패 시 ρ 확대를 건너뛰라"고 했는데, 우리는 그 경우 곧장 FULL로 간다 — 같은 취지이나 더 큰 튜브가 더 싼 우회를 찾을 가능성을 부정하지는 않으며, FULL이 유계(≤ 300 ms)이고 새 코스트맵 최적성을 유일하게 보장하므로 단순한 쪽을 택했다.
2. P50의 기각 샘플링은 유지하되(끝점이 253 근처거나 1 m 떨어진 쌍은 작업 정의 문제) 무제한 샘플링 결과를 같은 비중으로 보고한다.
3. 8-연결 A\* + 후처리 결정은 유지한다. 리뷰가 지적한 대로 근거를 바꿨고(§3.7-2), 어블레이션(§5.4-5)으로 숏컷 회수율을 실측해 결정을 재검토한다.
4. 500 ms는 `planning_time`(액션 수신→결과)로 해석해 주장하고, 코스트맵 갱신→발행의 end-to-end는 별도 목표(≤ 700 ms)로 보고한다 — 명세 문구가 시계의 시작점을 정의하지 않기 때문이며, 조건을 리포트에 명시한다.

### 9.1 감사 보정 (2026-09-22, 개정판 자체 점검 전 중단분에 대한 항목별 검증)

critique.json 42개 항목(수학 8, 제안 판정 4, 명세 공백 10, 인용 9, 필수 수정 11)을 문서와 대조했다. 수식은 `checks/*.py`로 재유도했고(`out_speedmap.txt`, `out_safety_gate.txt`, `out_misc.txt`, `out_tube_euclid.txt`), Nav2 사실은 `humble` 원본 소스와 컨테이너 `amr-fleet-system:wf-final`(nav2 1.1.20)로, 인용은 재조회로 확인했다. 31개는 개정판이 올바르게 해소했고, **11개는 부분·오해소**여서 아래처럼 고쳤다. 결과적으로 42/42 해소(단, 2차 감사에서 이 중 7개 항목에 남은 결함을 추가로 찾아 §9.2에서 보정).

**부분·오해소였던 항목과 보정**

1. 수학 7 / 필수 4(§4.1 표): "격자 거리가 0.05 m 배수라 c ∈ [229, 252]는 없다, $v_{floor}$는 255에만"은 틀림 — 거리는 `hypot(i,j)·r`이라 c = 240/245/248이 나타나고 245·248은 $v_{floor}$로 클램프. 표에 행 추가, c=34의 $v_{lim}$ 1.40 → 1.41, 경계 급락 29.8 → 29.7 %.
2. 제안 판정 2(C2) / 필수 3: (a) 충돌 검사를 격자 경로 $Q$가 아니라 **실제 추종하는 $P_s$**(숏컷 현 포함)에서 하고 $\pi$로 $Q$ 인덱스에 대응; (b) $I=\emptyset$인데 총비용이 $(1+\eta_v)$를 넘는 경우 → FULL로 명시; (c) 목표 변경 → FULL; (d) 비용 **감소**를 놓쳐 옛 우회가 남는 문제 → $T_{full}=10$ s 주기 FULL; (e) 의사코드의 가중 A\* 폴백이 FULL과 같은 마감을 써서 죽은 코드 → FULL 마감 340 ms, 전체 400 ms로 분리; (f) $\infty$-노름 튜브는 대각에서 1.41 m까지 벗어나 "ρ₀=1 m ⇒ 이탈 ≤ 1 m"가 거짓 → 유클리드 튜브로 바꾸고 셀 수를 $(2L\rho+\pi\rho^2)/r^2$로 재기술(리뷰의 $(2L\rho+4\rho^2)/r^2$는 축 정렬 상계로 유지). 연속 계획 거리는 Hausdorff ≤ 1 m(구성상)·Fréchet(측정)로 구분.
3. 명세 공백 1: `expected_update_rate`는 초 단위이고 넘으면 `waitForCostmap()`이 계획을 **블록**한다(경보 아님) → 0.2 → 0.3 s(= 안전 LiDAR 타임아웃), 관측원 `scan_filtered` 명시.
4. 명세 공백 2: 예측 프로파일을 실행 체인(`velocity_profiler_node`, `safety_node`)과 `robot_params.yaml`에 묶음 — 회전 시간에 α_max 2.0 rad/s²(90°: 1.05 → 1.80 s), S-curve 저크 항 $a/j$ = 0.5 s, 안전 게이트 $v_{gate}$ 항 추가.
5. 명세 공백 3 / 필수 9: e2e 시작점으로 쓴 `costmap_updates` 헤더는 Humble에서 stamp 0·1 Hz 발행이라 측정 불가 → `scan_filtered` 헤더 시각(오프라인)으로 교체, 프로젝트 기준 BT 1 Hz에서의 값(최악 1.6 s)도 병기.
6. 명세 공백 5 / 필수 8: `footprint_padding` 코드 기본 0.01이 $r_{ins}$를 0.21로 바꿈 → `footprint_padding: 0.0` 명시(기본값일 때 수치 병기). "keepout 금지"가 components.md §3.3의 `keepout_filter`와 충돌 → 필터가 inflation 뒤에 적용됨을 소스로 확인하고 **이진 마스크만 허용**으로 정정.
7. 제안 판정 3(C3): 8.2 % 초과율의 "대부분 회수" 단정이 §2.3·§3.7-2·§4.7에 남아 있었음 → 측정 항목으로 표현 통일.

**프로젝트 정합 보정(리뷰 범위 밖)**

- 이름: 플러그인 `amr_navigation::AStarPlanner`, 운영 `planner_id` `AStar`/`NavFn`/`Smac`(components.md §3.3·§5.3), 벤치 전용 id는 별도 YAML. 파라미터 위치 `src/amr_navigation/config/nav2_params.yaml`(components.md §6), 운동·안전 한계는 `robot_params.yaml`에서 읽고 복제하지 않음. 프레임 `map`/`<r>/base_footprint`(ekf.yaml), `/map` 절대·`scan_filtered`·`plan` 상대 이름.
- **안전 게이트**: `robot_params.yaml safety`(가장자리 기준, $t_r$ 0.15 s, 0.3/0.5/1.0 m 구역, 연속 제한)와 C1을 대조 — C1은 게이트보다 느슨한 상계라 "보수적" 표현을 삭제하고, 대신 게이트가 모든 도달 가능 c에서 $v\le v_{lim}$을 강제함을 확인(하한 주장 강화). 현재 설정으로는 0.6 m 통로가 E-stop, 3 m 통로가 1.27 m/s → 예측에 게이트 필수(없으면 −36 %), 구성 C(2.8 m, `speed_rule: gate`)를 실험에 추가, 통로 충돌을 안전팀에 구체 수치로 전달.
- BT: 프로젝트 기준 1 Hz + TTC 트리거를 전제로 두고 5 Hz는 변경 요청으로 표기. sequences.md §1의 "A\* ≤ 50 ms"를 새 목표 FULL 평균 목표로 반영. `plan_info`·`payload/mass` 인터페이스 추가를 components.md 변경 요청으로 명시(§7-8).
- 인용: V6 맵 해상도 미기재 → §2.4의 "14배" 환산을 가정부로 한정; V29 DOI, R10 서지(V29 참고문헌·검색으로 확인), V30은 초록만 확인·불변식 형태 RECALLED, V17 실험 도메인 보완, Theta\* 유클리드 항이 셀 단위이고 LOS 판정은 c ≤ 251.
- 의사코드: 시작 노드의 stamp/parent 초기화.

### 9.2 2차 감사 (2026-09-22, §9.1 이후 재검증)

critique.json 42개 항목(수학 8, 제안 판정 4, 명세 공백 10, 인용 9, 필수 수정 11)을 다시 하나씩 대조했다. 수식은 독립 스크립트 `checks/check_audit2.py`(`out_audit2.txt`)로 재유도했고 기존 `check_*.py`도 재실행했다. Nav2 사실은 `checks/src/`의 `humble` 원본으로, 인용은 재조회로 확인했다(V6 본문 HTML v2, V24 Discourse, V17·V3 초록, V29 PDF 참고문헌·IEEE 서지와 DOI 검색, V28 CMU RI 페이지, V31 OMPL 문서). **35개는 §9.1 상태로 올바르게 해소됐고, 7개(수학 3, 제안 2, 명세 공백 7, 인용 5, 필수 3·9·10)에 결함이 남아 있어 아래처럼 고쳤다 → 42/42 해소.**

**남은 결함과 보정**

1. 수학 3 / 필수 3(c) — 튜브 셀 수: §9.1이 쓴 "$|T_\rho|\le(2L\rho+\pi\rho^2)/r^2$"는 래스터 경계 셀 때문에 최대 +2.3 % 넘어 **상계가 아니다**(L=6, ρ=1: 6 177 > 6 057). "≈"로 바꾸고, 실제 상계는 리뷰의 $(2L\rho+4\rho^2)/r^2$(시험한 모든 L·ρ·방향에서 성립)로 명시, ρ=1 m 값을 6.1–6.2×10³로 정정.
2. 제안 2(C2) / 필수 3(c) — 단일 마감: 가중 A\*가 400 ms까지 돌 수 있어 C3 후처리(40 m 경로 5–13 ms, 100 m 13–32 ms 상계)가 마감 뒤에 실행 → "후처리 포함 ≤ 400 ms"가 깨졌다. 마감을 FULL 340 / 가중 A\* 370 / 전체 400 ms로 나누고 후처리가 전체 마감을 받아 단계별로 생략(`post_truncated`)하도록 §4.2-5·의사코드·§6 파라미터(`weighted_deadline_ms 370`)·테스트를 고침. 게이트 거리 나선 탐색에 c ≥ 1 생략과 1-립시츠 고리 탐색(열린 구역 비용 1/6) 추가.
3. 제안 2(C2) — 이탈 보장: "ρ₀ = 1 m ⇒ 이탈 ≤ 1 m"는 격자 경로에 대해서만 참이고, 창 재후처리의 숏컷 현은 비볼록 튜브를 벗어날 수 있었다 → 창 안 숏컷은 supercover 셀이 모두 $T_\rho$ 안일 때만 채택, 발행 경로 이탈은 측정 항목으로 명시.
4. 명세 공백 7 / 필수 10 — 코스트맵 CPU: §4.6(≈ 0.15 코어)과 §6(0.1–0.25 코어)이 어긋났고, 3–10 ms × 25 Hz의 하한은 0.075 코어 → 두 곳 모두 "0.08–0.25 코어(추정)"로 통일.
5. 인용 5 — V6: Hybrid-A\*/Lattice 시간 "39–42 ms"는 본문 표 I의 38.77–43.25 ms와 다름 → 38.8–43.3 ms로 정정(2D-A\* 66–89 ms, NavFn 61–71 ms, 창고 290/473/1 358 ms, 해상도 미기재는 재확인).
6. 필수 9 — 장애물 층 파라미터를 "§1과 §6에" 넣으라는 요구 중 §6이 비어 있었다 → §6에 `global_costmap` 블록 설정 항목 추가(§1.1·§5.1 값 참조).

**리뷰 범위 밖 정합 보정**

- **안전 게이트 방향**: `robot_params.yaml`은 구역 거리를 등방 최단거리로 정의하지만 연속 제한 주석은 "전방 여유 2.6 m"라 적는다. §4.1의 하한 논증·통로 속도표·"−36 %"는 등방 가정에서만 성립 — 전방 전용이면 구성 A는 c = 34 한 값, 구성 B는 c = 4…34 전체에서 하한이 깨지고 2.4/3.0 m 통로 게이트는 2.0 m/s가 된다(`out_audit2.txt`). §0·§4.1·§4.3(c)에 조건을 달고 §7-5에 안전팀 확인 항목 추가.
- §1 표: `planning_time` 근거 줄 번호를 `computePlan()` L462/L499로 정정(개정판의 L370/436은 `computePlanThroughPoses()`).
- §2.6: 수렴 수치를 반복행렬의 정확한 스펙트럼 반경으로 교체(GS 0.70/0.52/0.90, ω = 1.98 → 0.98, ω = 2.02 → 1.02; Jacobi (0.2, 0.5) 1.20) — 경계가 정확히 ω = 2임을 확인. 결론은 불변.
- V24 "3 000 과제×3회"는 게시물 문구상 총수/회당 수가 불명 → 그대로 옮겨 적음. V31 OMPL은 목적함수가 `reduceVertices` 외 메서드에 쓰인다는 문서 문구로 한정.
- 토픽 예시 `/amr_0k/...` → components.md §4.2의 `<r>` 표기(`/amr_01/plan`).

**재확인만 한 항목(변경 없음)**: Theta\* 가중치·`26+0.9c`·LOS c ≤ 251·unknown 253(theta_star.hpp), Smac 2D 대각 $\sqrt2(1+\kappa c/252)$·κ 기본 1.0·tolerance 0.125(node_2d.cpp, smac_planner_2d.cpp), NavFn $50+0.8c$·`priInc = 100`(navfn.cpp), Smac 스무더 제자리 GS(smoother.cpp L144–151), inflation floor·`hypot`(inflation_layer), MPPI 역함수 분모 253, KeepoutFilter `round(v·254/100)`·필터 후적용, `expected_update_rate` 초 단위·`waitForCostmap()` 블록, bringup 예시값(planner 20 Hz, global 1 Hz, r 0.22, s 3.0/R 0.55), §4.1 속도표·경계 29.7 %·구성 B 1 %, Menger ≤ 회전각 추정(등간격·부등간격 모두; $c\ge(a+b)\cos\tfrac{\Delta\theta}2$로 해석적 증명), $\tau_{rot}$·저크 항, e2e 800/1 600 ms, `robot_params.yaml`·`sensors.yaml`(스캔 평면 0.38 m, 720빔)·`ekf.yaml`(`base_footprint`)·components.md §3.3/§5.3/§6 이름.
