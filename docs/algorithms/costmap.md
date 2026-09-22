# Costmap 구성과 튜닝 (명세 4.4 "Costmap 구성")

> 대상: `src/amr_navigation/config/nav2_params.yaml` 의 `global_costmap`, `local_costmap`.
> 관련: [astar.md](astar.md) (전역 계획기가 비용을 쓰는 방식), [dwa.md](dwa.md) (지역 계획기의 충돌·여유 비용).

## 1. 레이어 구성

| 코스트맵 | 호스트 노드 | 프레임 | 크기 / 해상도 | 레이어 (순서) | 갱신 / 발행 |
| --- | --- | --- | --- | --- | --- |
| 전역 | `planner_server` | `map` | 정적 지도 크기 (60 × 40 m → 1200 × 800) / 0.05 m | `static_layer` → `obstacle_layer`(scan_filtered) → `inflation_layer` + 필터 `keepout_filter` | 5 Hz / 1 Hz |
| 지역 | `controller_server` | `<prefix>odom` (rolling) | 12 × 12 m / 0.05 m | `obstacle_layer`(scan_filtered) → `voxel_layer`(camera/depth/points_filtered) → `inflation_layer` | 10 Hz / 2 Hz |

- **Static Layer**: `map_topic: /map` (공유 지도, `multi_robot.md`), `trinary_costmap: true` — 지도 값은 0/254/255
  뿐이고 1…252 는 inflation 에서만 나오게 해서 비용 ↔ 거리 역함수(§3)가 성립하게 한다.
- **토픽 이름 (다중 로봇)**: Humble 의 `Costmap2DROS` 는 `/<ns>/<costmap 이름>` 네임스페이스의 자식 노드라서
  레이어 토픽을 상대 이름 `scan_filtered` 로 적으면 `/amr_01/local_costmap/scan_filtered` 로 풀린다 (실측:
  `ros2 topic info` — 발행자 0, 코스트맵 비최신 → controller_server 가 `isCurrent()` 대기에서 멈춤). 그래서
  템플릿에 `<robot_ns>/scan_filtered` 로 적고 `navigation.launch.py` 가 `<robot_ns>` → `/amr_01` 로 치환한다
  (`/tf` 는 리맵하지 않는다 — multi_robot.md). 서버 노드 자신의 토픽(`odom_topic` 등)은 상대 이름 그대로다.
- **Obstacle Layer**: 2D LiDAR (`scan_filter_node` 출력 `<robot_ns>/scan_filtered`). 마킹 10 m(전역 8 m),
  레이트레이싱 12 m(전역 10 m) — 레이트레이싱을 마킹보다 길게 둬야 먼 곳의 지난 장애물 셀도 지워진다.
- **Voxel Layer** (지역만): 깊이 카메라 점군 → LiDAR 평면(지면 +0.38 m) 밖의 장애물(선반 선반판, 지게차 포크,
  낮은 상자). 0.125 m × 16 층 = 2.0 m 높이까지, `min_obstacle_height 0.05` 로 바닥 잡음 제거,
  깊이 잡음 σ ∝ d² (sensors.yaml) 때문에 4 m 이내만 마킹.
- **Inflation Layer**: Nav2 식 `c(d) = ⌊252·exp(−s·(d − r_ins))⌋` (d ≤ r_ins 이면 253, 장애물 셀 254).
- **Keepout Filter** (전역): 교통 관리자(`traffic_manager_node`)가 발행하는 `keepout_mask` + `costmap_filter_info`
  (`<robot_ns>/costmap_filter_info`, 로봇별). **이진 마스크(0/100 → 0/254)만** 허용한다 — 필터는 inflation 뒤에
  적용되어 1…252 값을 만들지 않으므로
  §3 의 역함수를 깨지 않는다. keepout 셀은 팽창되지 않으므로 교통 관리자가 분쟁 구간 마스크를 r_circ(0.361 m)만큼
  미리 팽창해서 발행해야 한다 (연구 브리프 global-planning §1.1-2).

## 2. 풋프린트

`robot_params.yaml robot.footprint_length/width` = 0.60 × 0.40 m 사각형 (base_link 중심 = 회전 중심).

```yaml
footprint: "[[0.30, 0.20], [0.30, -0.20], [-0.30, -0.20], [-0.30, 0.20]]"
footprint_padding: 0.0
```

| 값 | 식 | 결과 |
| --- | --- | --- |
| 내접 반경 r_ins | 원점에서 가장 가까운 변까지 | **0.20 m** |
| 외접 반경 r_circ | √(0.30² + 0.20²) | **0.361 m** |

`footprint_padding` 은 0 으로 둔다 (Nav2 기본 0.01). 패딩 0.01 이면 r_ins = 0.21 이 되어 0.60 m 통로 중앙 비용이
206 → 210 으로 올라가고 통로 안에서 허용되는 횡 편차가 줄어든다. `robot_radius: 0.36` (원형 근사) 를 쓰면
0.60 m 통로 중앙(d = 0.30)이 r_ins(0.36) 안이 되어 **253 (INSCRIBED) → 모든 계획기가 통로를 막힌 것으로 본다**.
사각형 풋프린트가 필수인 이유다.

## 3. 좁은 통로 (로봇 폭 + 20 cm = 0.60 m) 산술

명세 4.4 "로봇 폭 +20 cm 이내의 좁은 통로를 충돌 없이 주행". 월드의 좁은 통로(중심 (0, −10), 순폭 0.60 m,
길이 4 m)와 합성 지도가 같은 치수다 (`amr_navigation/warehouse_map.py`, 단위 테스트 `test_narrow_passage_is_060m`).

로봇이 통로 중앙에 있으면 양쪽 벽까지 d = 0.30 m:

| 조건 | 통로 중앙 비용 c(0.30) | 판정 |
| --- | --- | --- |
| r_ins 0.20, s = 1.0 | 228 | 통과 (A* 배율 1 + 2·228/252 = 2.81) |
| **r_ins 0.20, s = 2.0 (전역)** | **206** | **통과 (배율 2.63)** |
| **r_ins 0.20, s = 3.0 (지역)** | **186** | **통과** |
| r_ins 0.20, s = 5.0 | 152 | 통과 |
| r_ins 0.21 (padding 0.01), s = 2.0 | 210 | 통과 (여유 감소) |
| r_ins 0.36 (robot_radius 0.36) | 253 | **막힘** |

통로 안에서 로봇 중심이 통과 가능한(c < 253) 띠: d > r_ins ⇔ |횡 편차| < 0.30 − 0.20 = **0.10 m**
(격자 정렬 때문에 벽 셀 한 칸이 안으로 들어오는 최악의 경우 0.075 m). 편차별 중앙 비용 (s = 2.0):
0 → 206, 2.5 cm → 216, 5 cm → 228, 7.5 cm → 239, 10 cm → 253.

풋프린트 외곽선이 벽에 닿지 않는 최대 헤딩 오차 ψ (반폭 0.3 sin ψ + 0.2 cos ψ ≤ 0.30 − 편차):
편차 0 → **22.7°**, 2 cm → 17.3°, 5 cm → 10.3°. 따라서 통로 안에서는 제어기의 CTE 가 2 cm 이내, 헤딩 오차가
15° 이내여야 한다. DWA 의 경로 이탈 비용과 Pure Pursuit 의 CTE 수렴이 이 조건을 만든다 ([dwa.md](dwa.md),
[path_tracking.md](path_tracking.md)).

## 4. Inflation 파라미터 튜닝

전역과 지역을 다르게 둔다. 둘 다 **"0.60 m 통로 중앙 < 253"** 이 하드 제약이고, 나머지는 경로 품질 트레이드오프다.

### 4.1 전역: `inflation_radius 1.2`, `cost_scaling_factor 2.0`

- A* 간선 가중 `ℓ·(1 + κ·c/252)` (κ = 2.0) 에서 통로 중앙의 배율이 2.63 → 4 m 좁은 통로를 지나는 비용이
  빈 공간 10.5 m 와 같다. 즉 **돌아가는 길이 6.5 m 이내면 넓은 통로를 택하고**, 그보다 멀면 좁은 통로를 쓴다.
  창고 레이아웃(랙 열 사이 5 m 통로)에서 좁은 통로는 남쪽 구역으로 가는 지름길일 때만 선택된다.
- 반경 1.2 m 에서 c = 34 → 5 m 통로 중앙(벽까지 2.5 m)은 0 이라 넓은 통로는 비용 0 구간이 있고,
  계획기는 벽에서 ≥ 1.2 m 떨어진 중앙 띠를 선호한다 (평균 여유거리 결과: [astar.md](astar.md) 벤치마크 표).
- s 를 키우면(5.0) 통로 중앙 비용이 152 로 내려가 좁은 통로를 더 자주 쓰고 벽에 더 붙는다. s 를 줄이면(1.0)
  넓은 통로 안에서도 비용이 남아 중앙 선호가 강해지지만 좁은 통로 배율이 2.81 로 커진다.
  2.0 은 연구 브리프(global-planning §1.1-5) 구성 A 와 같다.

### 4.2 지역: `inflation_radius 0.8`, `cost_scaling_factor 3.0`

- DWA 의 여유 비용 `J_clear = max_k max(0, c(p_k) − c(g_k))/252` 는 **기준 경로 대비 초과 비용**만 벌점을 준다
  ([dwa.md](dwa.md) §1.5). 가파른 감쇠(s = 3.0)로 벽 0.5 m 이내에서만 비용이 크게 변해 궤적 선택이 날카롭다.
- 외접 반경 비용 c(0.361) = 155 → 풋프린트 충돌 검사의 2단계 조기 판정(중심 비용 < 155 이면 외곽선 검사 생략)이
  대부분의 자세를 O(1) 에 처리한다.
- 반경 0.8 m 는 2 m/s 정지거리(2.8 m, 저크 포함)보다 짧다 — 고속에서의 안전은 여유 비용이 아니라
  충돌 검사(롤아웃 T_sim ≥ 정지 시간)와 `safety_node` 의 여유거리 속도 제한이 맡는다.

## 5. 동적 장애물 반영 (센서 주기·장애물 유지 시간)

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| 지역 `update_frequency` | 10 Hz | LiDAR 10 Hz 와 같게 — 매 스캔 반영 |
| 전역 `update_frequency` | 5 Hz | 1 Hz 재계획보다 충분히 빠르게, 전역 지도 전체 갱신 비용 절감 |
| `observation_persistence` | 0.0 s | 가장 최근 스캔만 마킹 → 지나간 사람·지게차의 셀은 다음 스캔의 레이트레이싱으로 즉시 지워진다 (잔상 없음) |
| scan `expected_update_rate` | 0.3 s | `safety.sensor_timeouts.lidar` 와 같다. 넘으면 코스트맵이 비최신 → controller_server 가 명령을 멈춘다 (LiDAR 고장 = 안전 정지, robot_params.yaml 정책) |
| depth `expected_update_rate` | 0.0 (검사 안 함) | 카메라 고장은 정지가 아니라 저속 운행(`degraded_mode_max_speed 0.2`) 대상 → 코스트맵 최신성 판정에서 뺀다 |
| 추적 트랙 | DWA 플러그인이 `perception/tracked_obstacles` 를 직접 구독 | 코스트맵은 "현재 점유"만 안다 → 예측(등속, 5 s)·VO 는 제어기가 처리 ([dwa.md](dwa.md) §2) |

`observation_persistence 0` 의 대가: 한 스캔에서 빔이 빠지면(반사율, 가림) 그 주기 동안 장애물이 사라진다.
추적기(`obstacle_tracker_node`)가 트랙을 유지하므로 동적 장애물은 DWA 의 TTC·VO 항이 계속 본다.

## 6. 검증

- 단위 테스트: `test_grid.cpp` (`InflationCostMatchesNav2Formula`: 코어 팽창식 = Nav2 `computeCost`),
  `test_collision.cpp` (3단계 풋프린트 검사, 외접 비용 경계), `test_warehouse_map.py`
  (`test_narrow_passage_is_060m`).
- 폐루프 좁은 통로 관통 (`closed_loop_eval.py` 의 `narrow` 경로, 통로 중심선 8.5 m, 1.0 m/s, 실제 지역 코스트맵
  s = 3.0 / 0.8 m 와 전역 코스트맵 활성):

  | 시험대 | 제어기 | 성공 | CTE 평균 / 최대 | 풋프린트 최소 여유 | 주행 시간 실제 / 예측 |
  | --- | --- | --- | --- | --- | --- |
  | 운동학 시뮬레이터 | DWA | ✓ | 0.0 / 0.1 cm | **0.10 m** | 9.8 / 10.0 s |
  | 운동학 시뮬레이터 | PurePursuit | ✓ | 0.0 / 0.0 cm | **0.10 m** | 9.6 / 10.0 s |
  | Gazebo (월드의 실제 통로, 6.1 m) | DWA | ✓ | 0.0 / 0.0 cm | 0.10 m | 7.4 / 7.6 s |
  | Gazebo | PurePursuit | ✓ | 0.0 / 0.0 cm | 0.10 m | 7.3 / 7.6 s |

  0.10 m 는 통로 중앙에서의 기하 최대 여유(0.30 − 0.20)다 — 지역 코스트맵 비용 186(§3)이 DWA 의 상대 여유 비용
  (`J_clear`, 기준 경로 대비 초과분만 벌점)에서 벌점이 되지 않아 감속 없이 통과했다.
- 전역 계획: 무작위 50 쌍 중 통로를 지나는 쌍은 0 (우회 ≤ 6.5 m 면 넓은 통로 선호, §4.1), NavigateToPose 의
  (0, −6.5) → (0, −14) 목표도 10.6 m 우회 경로를 택했다 ([astar.md](astar.md) §8.3).
