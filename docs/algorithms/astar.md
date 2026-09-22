# A* 전역 계획기와 경로 평활화 (명세 4.4 Global Planner)

> 구현: `src/amr_navigation/include/amr_navigation/core/{astar,path_smoother,grid}.hpp` (ROS 비의존 코어),
> `src/amr_navigation/src/ros/astar_planner.cpp` (`amr_navigation::AStarPlanner`, `nav2_core::GlobalPlanner` 플러그인).
> 설정: `config/nav2_params.yaml` `planner_server.AStar.*`. 비교군: `NavFn` (`nav2_navfn_planner/NavfnPlanner`),
> `Smac` (`nav2_smac_planner/SmacPlanner2D`) — BT `PlannerSelector` / `planner_id` 로 선택.
> 연구 브리프: `research/global-planning` §2 (기준 알고리즘)를 구현했고 §4 (TAR-A*)는 확장 지점만 남겼다 (§6).

## 1. 문제 정의

코스트맵 격자 G (셀 값 c ∈ {0…252 통행 가능, 253 INSCRIBED, 254 LETHAL, 255 미지}) 위에서 시작 셀 s 와 목표 셀 g 를
잇는 최소 비용 경로를 찾는다.

- 정점 = 셀, 간선 = 8-이웃. 대각 이동은 두 직교 이웃이 모두 통행 가능할 때만 (`allow_corner_cutting: false`,
  모서리 스침 방지).
- 간선 가중치 (Smac 2D `cost_travel_multiplier` 와 같은 꼴):

  **w(u, v) = ℓ(u, v) · m(c(v)),  ℓ = 1 (직교) / √2 (대각) [셀],  m(c) = 1 + κ · c / 252**

  κ = `cost_weight` (기본 2.0 = Smac 비교 설정과 같게). m(c) 는 `setTraversalTable()` 로 통째 교체할 수 있는 256 칸 표.
- 통행 불가: c ≥ `obstacle_threshold` (253). 미지 255 는 `allow_unknown` 이면 `unknown_cost` (252) 로 취급.
- 시작 셀이 INSCRIBED(253) 구역 안이면(벽에 붙어 정지한 로봇) 그 구역을 빠져나오는 동안만 253 셀 통행 허용
  (`allow_start_in_inscribed`), 목표가 막혔으면 `tolerance` 반경 안에서 가장 가까운 통행 가능 셀로 대체.

## 2. 휴리스틱: 옥타일 거리 (허용성·일관성 증명)

**h(n) = m_min · (max(|dx|, |dy|) + (√2 − 1) · min(|dx|, |dy|))**,  m_min = min_c m(c) (= 1, 빈 셀)

- **허용성**: 장애물이 없고 모든 셀 비용이 최소인 격자에서 8-연결 최단거리가 정확히 옥타일 거리다
  (대각으로 min(|dx|,|dy|) 칸, 나머지 직교). 실제 격자는 장애물(경로 연장)과 비용(m ≥ m_min)만 더하므로
  h(n) ≤ h*(n).
- **일관성** (h(u) ≤ w(u,v) + h(v)): 옥타일 거리는 8-이웃 이동에 대한 거리 함수(노름)라 삼각부등식
  oct(u, g) ≤ oct(u, v) + oct(v, g) 가 성립하고 oct(u, v) = ℓ(u, v), w(u, v) = ℓ·m ≥ ℓ·m_min 이므로
  h(u) ≤ m_min·ℓ + h(v) ≤ w(u, v) + h(v).
- 일관성 ⇒ 각 셀은 한 번만 닫히고(재확장 없음) 닫힐 때의 g 가 최적 → **닫힌 집합 + lazy deletion 힙**으로 충분.
- 단위 테스트 `test_astar.cpp`: `HeuristicAdmissibleAndConsistent` (무작위 격자의 모든 셀에서 h ≤ 역방향
  Dijkstra 거리, 모든 간선에서 h(u) ≤ w + h(v)), `OptimalVersusDijkstraOnRandomGrids` (무작위 소격자에서
  A* 비용 = Dijkstra 비용, 반환 경로의 비용 재계산 = 보고 비용), `UnreachableGoalReportsNoPath`,
  `BlockedGoalUsesTolerance`, `EscapesWhenStartInsideInscribedZone`, `NoCornerCutting`, `DeadlineAndIterationLimit`.

## 3. 구현 (자료구조·메모리·동점 처리)

| 요소 | 구현 | 비고 |
| --- | --- | --- |
| open | `std::vector<HeapNode>` + `std::push_heap/pop_heap` 이진 힙, lazy deletion | decrease-key 대신 중복 삽입, 닫힌 셀은 pop 시 버림 |
| g, parent | `float g_[N]`, `int32 parent_[N]` (행 우선, Costmap2D 와 같은 배치) | 1200 × 800 → 3.8 + 3.8 MB |
| 방문/닫힘 | `uint32 stamp_[N]`: 2·gen = 열림, 2·gen+1 = 닫힘 | 계획마다 O(N) 초기화 없음 (세대 번호만 증가) |
| 동점 처리 | 힙 키 (f_q, h), f_q = round(f · 1024) | 같은 f 면 목표에 가까운 셀 먼저. float 누적 오차가 동점을 깨지 않게 1/1024 셀 양자화 (준최적 한계 ≤ 1/1024 셀 = 0.05 mm) |
| 마감 | 첫 확장과 이후 1024 확장마다 `steady_clock` 확인 → `kTimeout` | `max_planning_time 0.4 s` (명세 재계획 500 ms 에서 평활화·통신 여유 0.1 s) |

메모리 12 B/셀 (1200 × 800 → 11.5 MB, 연구 브리프 예산 15 MB 이내), 계획 간 버퍼 재사용.
동점 처리 효과 (단위 테스트 `LargeOpenGridCornerToCorner`): 1200 × 800 빈 격자 대각선 끝→끝에서 확장 **1 200 셀**
(= 경로 길이; 양자화 전에는 float 오차 때문에 25 359 셀).

## 4. 경로 평활화 (여유거리 보존)

A* 격자 경로는 8방향 지그재그(최대 8.2 % 초과 길이)이고 방향이 45° 단위로 꺾여 제어기가 추종하기 어렵다.
세 단계로 후처리한다 (`core::PathSmoother::process`).

1. **비용 인지 숏컷**: 기준점 i 에서 j 를 늘려 가며 현 p_i→p_j 가
   (a) supercover 셀이 모두 통행 가능(≤ 252),
   (b) 현이 지나는 셀의 최대 비용 ≤ 원래 격자 경로 구간 [i, j] 의 최대 셀 비용 + `clearance_cost_margin`(10),
   (c) 현을 따라 적분한 A* 비용 ∫m(c)ds ≤ (1 + δ)·(G_j − G_i), δ = `shortcut_cost_ratio` 0.05
   인 동안 전진, 처음 실패하면 직전 j 채택. (b) 는 모서리를 가로질러 장애물에 붙는 현을, (c) 는 inflation 띠를 길게
   가로지르는 현을 막는다. (b) 의 10 비용 단위 ≈ c≈100, s = 2 에서 여유거리 1 셀(0.05 m).
2. **등간격 재표본화** (`output_spacing` 0.05 m).
3. **Gauss–Seidel 경사 하강 평활화** (양 끝 고정), 각 내부 점 y_i:

   y_i ← y_i + w_d (x_i − y_i) + w_s (y_{i−1} + y_{i+1} − 2 y_i) + w_c · r · s_i · n̂_i

   x = 숏컷 경로(데이터), n̂ = −∇c/|∇c| (코스트맵 중심차분, 비용이 줄어드는 방향), r = 해상도,
   s_i = min(1, (c − c_thr)/(252 − c_thr)) (c > c_thr = 100 인 점만 밀어냄).
   - 선형 부분(w_c = 0)은 (1 + w_d) y_i = w_d x_i + (1 − 2w_s)… 꼴의 대각 우세 3중대각 시스템에 대한 SOR 이고
     w_d > 0, 0 < w_d + 2 w_s < 2 에서 수렴한다 (기본 w_d 0.2, w_s 0.3 → 0.8).
   - **점별 사영(projection)**: 갱신 후 점의 셀 비용이나 이웃 두 선분의 최대 셀 비용이 입력 경로의 같은 자리 천장
     ceil_i (점 i 에 닿는 입력 선분 두 개의 최대 비용)를 넘으면 그 점은 이번 스윕에 움직이지 않는다 →
     평활화가 여유거리를 줄이지 않는다. 비선형 여유 항 때문에 전체 수렴을 주장하지 않으므로, 매 스윕 뒤 전체 선분의
     통행 가능성을 다시 검사해 실패하면 직전 스윕으로 되돌린다(롤백 가드). 이동량 합 < 1e-4 m 이면 종료.
4. **방향**: 내부 점은 중심차분 접선, 마지막 점은 목표 방향.

**보장 (단위 테스트)**: 평활 경로의 최대 셀 비용 ≤ 원래 격자 경로의 최대 셀 비용 + 10
(`SmoothingKeepsClearanceInRackAisle`: 1.2 m 랙 통로 시나리오에서 raw 113 → 평활 113, 길이 10.26 → 9.89 m),
장애물 쪽으로 수축하는 V 자 입력에서도 모든 선분이 통행 가능 (`ProjectionStopsAtObstacle`), 여유 항이
벽 반대로 민다 (`ClearanceTermPushesAwayFromObstacle`).

> 설계 이력: 처음 구현(숏컷 조건 (a)(c)만)은 랙 통로 입구 모서리를 가로지르는 현을 채택해 최대 비용이
> 113 → 149 (중심 여유 0.60 → 0.50 m)로 나빠졌다. 조건 (b)와 점별 사영을 넣어 해결했다.

## 5. 파라미터 (`planner_server.AStar.*`)

| 파라미터 | 기본 | 단위 | 의미 / 튜닝 근거 |
| --- | --- | --- | --- |
| `cost_weight` | 2.0 | – | κ. 0 이면 순수 최단거리(벽에 붙음), 클수록 장애물에서 멀어지지만 길어진다. Smac `cost_travel_multiplier` 와 같게 두어 비교를 공정하게 |
| `allow_unknown` | true | – | 미지 셀 통행. SLAM 지도 가장자리의 미지 영역을 관통하는 목표 허용 |
| `unknown_cost` | 252 | cost | 미지 셀의 비용 — 가장 비싼 통행 가능 셀과 같게 해서 미지 영역을 우회 선호 |
| `tolerance` | 0.25 | m | 목표가 막혔을 때 대체 탐색 반경 (NavFn/Smac 과 같은 값, 연구 브리프 §1.1-3) |
| `max_iterations` | 0 | 회 | 확장 상한 (0 = 셀 수) |
| `max_planning_time` | 0.4 | s | 탐색 마감 (명세 재계획 500 ms) |
| `allow_corner_cutting` | false | – | 대각 이동 시 두 직교 이웃이 통행 가능해야 함 |
| `allow_start_in_inscribed` | true | – | INSCRIBED 안에서 출발 허용 |
| `use_final_approach_orientation` | false | – | false: 마지막 자세 = 목표 방향 |
| `smoother.enable_shortcut` | true | – | |
| `smoother.shortcut_max_length` | 10.0 | m | 긴 현이 적분 비용을 과소평가하지 않도록 |
| `smoother.shortcut_cost_ratio` | 0.05 | – | δ |
| `smoother.clearance_cost_margin` | 10 | cost | 여유거리 천장 여유 |
| `smoother.output_spacing` | 0.05 | m | = 해상도 (제어기 투영 정밀도) |
| `smoother.w_data / w_smooth / w_clearance` | 0.2 / 0.3 / 0.3 | – | 수렴 조건 w_d + 2w_s < 2, 여유 항은 셀 단위 스텝 |
| `smoother.clearance_cost_threshold` | 100 | cost | 이 비용(여유 ≈ 0.66 m) 이상인 점만 밀어낸다 |
| `smoother.max_iterations / tolerance` | 100 / 1e-4 | 회 / m | |

모든 값은 YAML 로 외부화되어 있고, 위 표의 키는 **모두** 실행 중 `ros2 param set /amr_01/planner_server
AStar.cost_weight 3.0` 으로 바꿀 수 있다 (재빌드·재시작 불필요, 명세 9장). 동적 재설정 콜백(`onParameters`)은 선언된 키를
모두 다음 계획부터 반영하고, **모르는 키·잘못된 타입·범위 밖 값(예: `unknown_cost` > 252, 평활 수렴 조건
w_data + 2 w_smooth ≥ 2)은 묶음 전체를 거부**하며 이유를 돌려준다 (`plugin` 은 재시작이 필요해 거부). 리뷰 전에는
`unknown_cost`, `allow_corner_cutting`, `max_iterations`, `smoother.shortcut_*` 등을 성공으로 답하고 무시했다.
단위 테스트 `AStarPlannerPlugin.PlansThroughAisleAndHandlesErrors` (`max_iterations 5` 반영 → 계획 실패, 접근 방향·출력
간격 반영, 모르는 키·타입·범위·수렴 조건 거부, 거부된 묶음은 아무것도 반영하지 않음).

## 6. 확장 지점 (연구 브리프 global-planning §4 TAR-A*)

- **C1 정지거리 속도맵 시간 비용**: `AStar::setTraversalTable()` 로 m(c) 를 "여유거리 → 허용 속도 → 셀 통과 시간"
  표로 바꾸면 탐색 코드 수정 없이 시간 최적 A* 가 된다 (휴리스틱 배율은 표의 최솟값으로 자동 갱신 → 허용성 유지).
- **C2 튜브 한정 국소 복구**: `AStar::plan` 이 세대 번호 기반이라 부분 격자(튜브 마스크)를 같은 버퍼로 재탐색 가능.
- **C3 속도 프로파일·시간 예측**: `core::SpeedProfile::predictTravelTime` (곡률 상한 + 가감속 원뿔 + S-curve 보정),
  Python 판 `amr_navigation/path_metrics.predict_travel_time` — 명세 "예측 시간 오차 15 %" 측정에 쓴다
  ([path_tracking.md](path_tracking.md) §5.2: 이상화 12 목표 DWA 평균 2.2 % · 최대 4.4 %, PP 5.1 % · 14.8 %;
  통합 체인 무작위 쌍 8 구간 DWA 평균 5.7 % · 최대 20.5 % — [dwa.md](dwa.md) §6.3).

## 7. 벤치마크 방법

- **지도**: 이 기준의 SLAM 지도 `maps/warehouse.yaml` (1207 × 806 @ 0.05 m, map 프레임 = 월드, 재매핑본 — 자유
  928 470 셀, 미지 23 688 셀). 합성 지도(`amr_navigation/warehouse_map.py`, 1200 × 800)는 운동학 시험대용이다.
- **쌍**: 지도의 자유 셀 중 점유·미지에서 0.45 m(외접 반경 + 여유) 이상 떨어진 셀에서 무작위로 두 점, 직선거리 ≥ 5 m,
  50 쌍 × seed 42 / 7 (`slam_pairs.py`, scratch — 계획기·제어기 비교가 같은 쌍 파일을 쓴다: [dwa.md](dwa.md) §6).
- **코어 측정** (`astar_benchmark`, ROS 없음): Nav2 식 팽창(0.2 / 1.2 / 2.0) 후 A* + 평활화, 쌍마다 3 회 반복 중
  최솟값(외부 부하 잡음 제거).
- **Nav2 비교** (`planner_benchmark.launch.py` + `bench_planners.py`): 같은 지도·쌍(seed 42)으로 `planner_server` 에서
  AStar / NavFn / Smac 2D 를 `compute_path_to_pose` (use_start) 로 호출, 서버 측 `planning_time` 과 왕복 시간,
  경로 길이, 최소 여유거리(지도 EDT, 로봇 중심)를 기록.

## 8. 결과

측정 환경: `amr-fleet-system:wf-final` 컨테이너, 호스트 32 스레드, 2026-09-23 00:10 KST, **load average 62–65** (`uptime`
— 다른 사용자의 통합 시험 컨테이너와 이 작업의 Gazebo 가 겹친 부하 아래; 계획 시간은 무부하보다 길게 나온다).

### 8.1 코어 A* (ROS 없음, SLAM 지도 1207 × 806, 50 쌍, 쌍마다 3 회 중 최솟값)

| 지표 | seed 42 평균 | p95 | 최대 | seed 7 평균 / 최대 |
| --- | --- | --- | --- | --- |
| 성공 | 50/50 (100 %) | | | 50/50 (100 %) |
| 탐색 시간 | 4.36 ms | 16.93 ms | 22.08 ms | 5.71 / 36.50 ms |
| 평활화 시간 | 1.78 ms | 3.60 ms | 3.90 ms | 2.19 / 9.24 ms |
| 합계 (A* + 평활화) | 6.15 ms | 19.28 ms | **25.30 ms** | 7.90 / **41.52 ms** (명세 500 ms 의 8.3 %) |
| 확장 셀 수 | 23 320 | 81 630 | 107 474 (전체 97 만 셀의 11 %) | 29 818 / 167 952 |
| 경로 길이 (격자 → 평활) | 27.01 → 26.28 m (−2.7 %) | | | 28.75 → 27.86 m (−3.1 %) |
| 최소 여유 (로봇 중심, 지도 EDT) | 1.129 m | | 최소 0.450 m (끝점) | 1.237 / 최소 0.461 m |

### 8.2 Nav2 planner_server 비교 (같은 전역 코스트맵·같은 50 쌍, `compute_path_to_pose`)

| planner | 성공 | 계획 시간 평균 / p95 / 최대 [ms] | 왕복 평균 [ms] | 길이 평균 [m] | 길이 비 (AStar = 1) | 여유거리 평균 / 최소 [m] (끝점 포함) | 경로 중간 여유 평균 / 최소 [m] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **AStar (본 구현)** | **50/50 (100 %)** | **6.6 / 20.1 / 26.5** | **19.3** | 26.30 | 1.000 | 1.127 / 0.450 | 1.269 / 0.814 |
| NavFn (Dijkstra, 보간) | 50/50 (100 %) | 12.9 / 22.7 / 26.0 | 37.8 | 26.06 | 0.993 | 1.098 / 0.400 | 1.286 / 1.097 |
| Smac 2D | 50/50 (100 %) | 24.9 / 73.7 / 137.2 | 38.2 | 26.47 | 1.008 | 1.089 / 0.450 | 1.241 / 0.863 |

- **명세 판정**: 성공률 100 % (목표 ≥ 98 %), 재계획(서버 계획 시간) 최대 26.5 ms · 왕복(요청 → 경로 수신) 평균
  19.3 ms (목표 < 500 ms) — load 62–65 아래에서. Gazebo 통합 체인의 목표 수락 → 첫 경로 발행은 52 목표 평균 1.6 ms ·
  최대 3 ms (시뮬레이션 시각, [path_tracking.md](path_tracking.md) §5.5).
- **길이**: 세 계획기 차이 ±0.8 % — 같은 비용 가중(κ = 2)이라 같은 통로를 고른다. NavFn 이 가장 짧은 것은 격자
  경로가 아니라 퍼텐셜 기울기를 따라 보간하기 때문이다.
- **여유거리**: 끝점(시작·목표를 여유 ≥ 0.45 m 로 뽑음)을 뺀 경로 중간 최소 여유는 AStar 0.81 m 로 NavFn(1.10)보다
  작다. 추정 원인: 비용 인지 숏컷이 "원래 격자 경로의 최대 비용 + 10" 까지 허용하므로 랙 모서리를 돌 때 중심선이 벽 쪽으로
  당겨진다 (길이 −2.7 % 와 교환, 미검증 — 쌍별 격자 경로 여유와의 비교는 하지 않았다). 0.81 m 는 외접 반경 0.361 m 의 2 배
  이상이므로 충돌 여유는 충분하지만, 더 넓은 여유가 필요하면 `smoother.clearance_cost_margin` 을 10 → 0, `w_clearance` 를
  0.3 → 0.6 으로 올린다 (§5).
- **계획 시간**: AStar 가 평균 2.0 배(NavFn), 3.8 배(Smac) 빠르다 — 옥타일 휴리스틱(허용·일관), 세대 번호 방문 표시,
  1/1024 셀 동점 양자화(§3). Smac 2D 는 자체 평활기·방향 계산을 포함해 p95 가 길다. 최대값은 AStar 26.5 ms 와 NavFn
  26.0 ms 가 비슷하고 (Smac 137 ms) — 부하 아래 긴 쌍에서 AStar 의 확장 수가 10 만을 넘는 경우다.

### 8.3 좁은 통로 (costmap.md §3)

무작위 50 쌍 중 0.60 m 통로를 지나는 쌍은 없었다 (우회 ≤ 6.5 m 면 넓은 통로 선호, costmap.md §4.1). 이 기준의
SLAM 지도(`maps/warehouse.pgm`)에 코어 A* 를 직접 돌려 통로가 열려 있는지 확인했다 (전역과 같은 팽창 0.2 / 1.2 / 2.0,
κ 2.0, 도구 `astar_aisle`): 통로 78 개 행 모두 최소 비용 ≤ 228 (< 253), 통로 안 목표 (0, −6.5) → (0, −10) 과
(0, −13.5) → (0, −10) 은 통로 중심선에서 0.00–0.05 m 안으로 계획, 관통 목표 (0, −6.5) → (0, −14) 는 11.1 m 우회를
고른다. 지도의 통로 벽은 흩어져 있어(점유 안쪽 경계 −0.228…+0.222 m, 가장 좁은 행 자유 폭 0.45 m) 전역 경로는 통로
중심에서 수 cm 비낄 수 있다 — 통로 안의 실제 중심 추종은 DWA 의 지역 재중심이 맡는다 ([dwa.md](dwa.md) §1.7).
Gazebo 관통 결과는 [costmap.md](costmap.md) §6.2. 전역 계획기가 통로 안 목표를 계획하는 경우는 단위 테스트
`AStarPlannerPlugin.PlansThroughAisleAndHandlesErrors` (1.2 m 통로 중앙 ±0.15 m)와 `test_warehouse_map.py` 로도 본다.
