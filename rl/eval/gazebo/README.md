# rl/eval/gazebo — Gazebo 정적 시나리오 v1 과 다회 에피소드 하네스 (보너스 1 A5)

완료 정의 D4(전이 점검)·D5(RL 대 DWA 정량 비교)의 도구다. 단계 C 의 "팔당 ≥ 200 에피소드" 를 같은 시작·목표 집합에서
돌리도록 시나리오를 **결과 전에** 고정해 두고, 한 Gazebo 세션에서 에피소드를 연달아 도는 실행기
(`src/amr_rl_bench`)가 에피소드마다 같은 지표를 남긴다. 기록: 노트 항목 `*_b1-gzbench_harness-scenarios.md`.

| 파일 | 내용 |
| --- | --- |
| `scenarios_v1.json` | 시나리오 집합 v1 — 주 200 + 좁은 목표 스트레스 200, 실행 순서, 스모크 부분집합 (아래 §1) |
| `make_scenarios.py` | 위 파일을 만든 생성기 (시드 고정 — 다시 돌리면 같은 파일) |
| `build_planner.sh` | 생성기가 쓰는 팀 A* 파이썬 모듈 빌드 (산출물은 저장소 밖에) |
| `recompute.py` | 세션 원자료(`episodes/*.json.gz`)에서 지표를 지금의 정의로 다시 계산 → `gzbench_recomputed.csv`·`episodes_v1_recomputed.csv` |
| `analyze_smoke.py` | 세션 요약: 판정 H1–H4 (A5-4), 집합별 층위 성공·시간·CTE·개입, 전이 점검 3·4·6, 소요 |
| `analyze_scan.py` · `analyze_step.py` · `transfer_limits.py` | 전이 점검 2 (LaserScan 규약·광선 잔차) · 5 (플랜트 T_m·T_d, 관측 사슬) · 1 (한계 대조표) |
| `compare_sessions.py` | 부하 조건이 다른 세션(공유 대 독점)의 RTF·지연·주기·플랜트·벽시계를 나란히, 단계 C 소요 추정 (A5-5) |

## 1. 시나리오 집합 v1 (`gz_static_v1`)

- 월드: `warehouse.sdf` 에서 움직이는 장애물(actor 6, 지게차·셔틀)을 뺀 정적 사본 (`src/amr_rl_controller/scripts/
  make_static_world.py`). 지도(AMCL·전역 코스트맵)는 `maps/warehouse` 그대로 — map 프레임 = 월드.
- 경로: 팀 A* 코어(`astar.cpp` + `path_smoother.cpp`, κ 2.0, 대각 모서리 금지, 허용 0.25 m)를 전역 코스트맵 정적 팽창
  (내접 0.20, 반경 1.2, 감쇠 2.0) 위에서 — `rl/sim2d` 경로 풀과 같은 계획기·같은 비용. 길이 L ∈ [3, 30] m, 자기 근접 제외.
- 시작: 막힘 경계와 ≥ 0.861 m (= 0.5 + 외접 반경, 계약 §10), yaw = 경로 시작 접선 + U[−0.5, 0.5].
- 목표: 막힘 경계와 ≥ 0.40 m (2D·Isaac 경로 풀과 같은 goal_clear), yaw = 경로 끝 접선 (마지막 점과 0.25 m 앞 점).
- **주 집합 / 스트레스 집합** (완성 계획 §6): 끝 접선 자세에서 풋프린트를 직진시킬 때 처음 닿기까지의 거리(A3 의
  straight_approach — 정본 SafetyGate 직진 접근 거리의 기하값)가 ≥ 0.40 m 이면 주, 아니면 좁은 목표 스트레스.
  두 집합을 **같은 비중**으로 돌려 나란히 보고한다. 생성 중 조건을 통과한 후보의 2.35 % (201/8559) 가 스트레스였다
  [계산] — A3 의 Isaac held-out 2.3 % 와 같은 수준.
- 실행 순서 `exec_order`: 두 집합을 번갈아 (쌍 안의 순서는 무작위) 섞은 고정 순서. 단계 C 는 이 순서로 돌고 순번을
  `exec_index` 로 남긴다 (PROTOCOL.md §2-4). 스모크 `smoke` = 실행 순서의 처음 주 10 + 스트레스 10.
- 시간 상한: min(120, 20 + 3·L) s (sim, 계약 §8). L 은 생성기의 A* 경로 길이 (Nav2 가 실제로 낸 경로가 아니다).

## 2. 하네스 (`src/amr_rl_bench`)

```bash
# 호스트에서 (gz_slot 아래 포그라운드). 출력은 /runs/<tag>/ 에 gzbench.csv · episodes_v1.csv · episodes/*.json.gz
/home/CAPYI_AI/env/gz_slot.sh <id> docker run --rm --name amr-ai-<id>-gz --gpus all --shm-size=8g \
  -e ROS_DOMAIN_ID=<D> -e IGN_PARTITION=<P> -e ROS_LOCALHOST_ONLY=1 -e WORLD=/w/warehouse.sdf \
  -e MODEL=/models/v12_s1.onnx -e COMMIT=$(git rev-parse --short HEAD) \
  -v <worktree>:/ros2_ws -v <runs>:/runs -v <world dir>:/w:ro -v <models>:/models:ro -w /ros2_ws amr-ai/rl:0.3 \
  bash src/amr_rl_bench/scripts/run_bench.sh /runs/<tag> rl --scenarios /ros2_ws/rl/eval/gazebo/scenarios_v1.json \
    --select smoke --policy-id v12_s1 --train-seed 1 --run-tag <tag> \
    --world-sdf /ros2_ws/src/amr_simulation/worlds/warehouse.sdf --models-dir /ros2_ws/src/amr_simulation/models
```

제어기 인자: `dwa` (BT 기본 'DWA' = 팀 DWA 그대로), `rl` (`replace_dwa` — 'DWA' 자리에 RL 플러그인, 모드 B, ONNX),
`rl_handover` (RL + 목표 `HANDOVER_RADIUS` m 안은 내부 DWA, 기본 0.4). 세 경우 모두 같은 스택·같은 nav2 파라미터다.

에피소드 절차는 `scripts/gzbench_runner.py` 머리말, 지표 정의는 `amr_rl_bench/metrics.py` 머리말이 규범이다. 요약:

| 지표 | 정의 |
| --- | --- |
| `outcome_contract` | 계약 §8 종단 사건 중 시간순 첫째: collision(지면 진실 풋프린트–정적 상자 부호 거리 < 0, 또는 collision_monitor 접촉) → success(strict_gt) → stall(SimpleProgressChecker 0.3 m·20 s) → timeout. 접촉·성공은 Nav2 종료 뒤 정착 구간까지 본다 (goal checker 는 속도를 안 봐 DWA 가 움직이며 성공을 낸다). 사건 없이 시스템이 먼저 끝나면 stall + `contract_note = ended_without_success`. 준비 실패·감시견은 error |
| `strict_gt` / `strict_est` | 목표 xy ≤ 0.10 m 이고 \|v\| ≤ 0.05 m/s 인 순간 (지면 진실 / 위치추정 자세)이 계약 종단 사건 중 첫째 — 두 기준 모두 같은 순서 규칙(접촉 → 성공 → 정체 → 상한, 접촉·정체·상한은 지면 진실). 추정 자세는 실제 표본 사이 틈 ≤ 0.25 s 에서만 보간하고 최종 추정은 잰 시각의 표본으로 덧붙인다 (A5-5 리뷰 반영, `0fd7157` 부터 — 그 전 지표는 정착 구간의 낡은 추정을 끌어 썼다) |
| `outcome_contract_est` · `est_end_gap_s` | 추정 기준 계약 결과 · 기록 끝 − 마지막 실제 추정 표본 (실행기가 정착 구간까지 표집하면 ≤ 0.1 s; A5-4 원자료는 0.13–0.80 s 라 그 재계산은 근사). 2026-10-07 에 덧붙인 열 |
| `nav2compat_gt` / `_est` | 목표 xy ≤ 0.25 m 에 접촉 전 닿았다 (Nav2 기본 goal checker, 속도·yaw 무관) |
| `system_success` | Nav2 navigate_to_pose SUCCEEDED (배포 goal checker: 추정 xy 0.10, yaw 0.05) |
| `final_*` | 끝나고 정착 뒤 최종 거리·yaw 오차·속도 (지면 진실·추정) |
| `nav_time_s` · `path_length_m` | 목표 수락 → Nav2 종료(또는 취소) sim 시간, 지면 진실 주행 거리 |
| `cte_first_*` · `cte_active_*` · `max_dev_m` · `return_max_s` | 움직이는 표본(\|v\| > 0.05)의 첫 경로·활성 경로까지 거리, 이탈 구간 0.30/0.15 m |
| `interventions` · `intervention_time_s` | cmd_vel_smoothed(입력) 대 cmd_vel(출력) 차 > 0.01 의 연속 구간 수·시간 (PROTOCOL §5). 출력은 시각 창 −60…+5 ms 의 입력 중 하나라도 같으면 바뀌지 않은 것으로 본다 (safety 는 직전 입력을 약 20 ms 늦게 내고, 기록 노드의 수신 순서가 뒤바뀐다) |
| `zone_*_s` | safety/zone WARNING·CRITICAL·STOP 지속 시간 |
| `contacts_static` · `contacts_dynamic` · `min_clear_*` | 정적 기하 접촉 사건, collision_monitor 접촉 증가, 최소 부호 거리 |
| `near_goal_stop_s` | 목표 0.10–0.50 m 에서 정지(\|v\| ≤ 0.02, \|ω\| ≤ 0.05)한 시간 (A2 직전 정지) |
| `loc_err_*` · `loc_collapse` · `lost_events` | 추정 대 지면 진실 오차, > 0.5 m 붕괴, localization/lost 전이 |
| `decisions` · `src_*` · `dwa_*` | RL 플러그인 결정 출처·DWA 폴백 사유 (5 = 근접 인계) |
| `decision_period_*` · `tick_dt_sim_mean` · `ctrl_dt_*` | 결정 주기·틱 간격 (sim), cmd_vel_nav 간격 (sim·벽시계) |
| `scan_to_cmd_*` · `chain_lag_*` · `scan_age_*` | 명령 시각의 최신 스캔 나이, cmd_vel_nav → cmd_vel 지연, 플러그인 관측 스캔 나이 |
| `rtf` · `wall_*` | 주행 구간 sim/벽시계, 준비·주행·전체 벽시계 |

`episodes_v1.csv` 는 같은 행을 `rl/eval` 의 episode/v1 열로 옮긴 것이다 (outcome = outcome_contract, contacts =
계약 종단 사건까지의 접촉, scenario_set = `gz_static_v1_main` / `gz_static_v1_stress`). `rl/eval/scripts/evaluate.py` 의
입력이 된다.

## 3. 단계 C 에 넘기는 것 (A5-4 스모크에서)

- 에피소드 벽시계 — **독점 시간대** (`exclusive.sh`, A5-5): DWA 주 31.9 s · 스트레스 69.7 s, RL 32.7 · 34.2 s, 준비 5.3 s, 세션 고정비 약 3 min
  (공유 부하 하 예비값: DWA 34.7 · 72.1, RL 38.0 · 38.3). 팔당 200 (주 100 + 스트레스 100), 50 에피소드 묶음(`--start/--count`) 4 세션 ≈
  **DWA 3.0 h (± 0.7), RL 2.1 h (± 0.2)** — ± 는 스모크 n 10 의 평균 추정 불확실성. RTF 는 독점에서도 중앙 0.88 (위 한계 약 0.9).
- 부하에 민감한 측정(RTF·벽시계·지연·구동 응답)은 `exclusive.sh` 로 돈다 (COORDINATION §8.1). Gazebo 스택 자체가 loadavg 를 약 20 까지
  올리므로 loadavg 만으로는 외부 오염을 판단할 수 없다. Isaac 슬롯은 독점 대상이 아니다 — 동시 Isaac 작업을 기록한다.
- 지표를 바꾸면 Gazebo 를 다시 돌리지 말고 `recompute.py` 로 다시 계산한다 (원자료가 다 있다). 원래 CSV 는 지우지 않고 바꿀 때마다
  새 접미사를 쓴다 (`_recomputed` = 5222f95 지표, `_recomputed_estfix` = 0fd7157 지표). **단, 원자료에 없는 것은 재계산으로 못 만든다** —
  A5-4 원자료에는 정착 구간 추정 표본이 없어 strict_est 재계산이 근사다 (A5 리뷰 차단 1). 지표가 새 시계열을 요구하면 실행기도 고친다.
- 세 제어기 모두로 짧은 흔들기를 먼저 한다 — A5 에서는 RL 로만 흔들어 DWA 의 "움직이며 성공" 꼴의 지표 결함을 스모크에서야 찾았다.
- 노트: A5-2 (하네스·지표 정의), A5-3 (전이 점검 표), A5-4 (스모크·소요).
