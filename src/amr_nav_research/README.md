# amr_nav_research — DWA 연구 사본 (DwaPlannerX · DwaControllerX · chain_sim_x)

DWA 논문 보강 실험(`docs/research/paper-research/prereg.md`, 작업 계약 `contract.md`)에서 쓰는 사본 패키지다.
원본 `src/amr_navigation` 은 **한 줄도 고치지 않는다.** 코어 라이브러리 `amr_navigation_core`
(SpeedProfile, footprint, velocity_obstacle, crossing_yield/gate, profiler …)는 링크해서 그대로 쓴다.
사본은 아래 세 가지다.

| 사본 | 원본 | 바뀐 것 |
| --- | --- | --- |
| `amr_nav_research::DwaPlannerX` (`include/amr_nav_research/dwa_x.hpp`, `src/dwa_x.cpp`) | `core::DwaPlanner` | 연구 스위치 R1·R2·R3, 연구 상태 입출력, 재설정 정보 |
| `amr_nav_research::DwaControllerX` (`src/dwa_controller_x.cpp`, `plugins.xml`) | `amr_navigation::DWAController` | 코어가 DwaPlannerX, `<plugin>.research.*` 파라미터, 재설정 기록, dwa/stats 4 칸 추가 |
| `chain_sim_x` (`tools/chain_sim_x.cpp`) | `tools/chain_sim.cpp` | 코어가 DwaPlannerX, 연구 키·`stateful`·`reset_log`, JSONL 끝에 필드 추가 |

`ros_utils_x` 는 원본 `ros_utils` 의 사본이다. 원본은 플러그인 라이브러리 안에 있어서, 링크하면
pluginlib 등록이 엉킬 수 있다. 시험대는 원본 팔과 연구 팔을 같은 controller_server 에 올리므로 사본을 두었다.

**동치:** 연구 스위치가 모두 기본값이면 세 사본의 출력이 원본과 비트 단위로 같다.
- `test_dwa_x_equivalence`
  - 원본 golden 해시 8 개
  - 절제 스위치 8 설정의 폐루프
  - 무작위 한 주기 1 600 회 (결과 구조 전체)
  - 무작위 폐루프 24 × 400 주기
- `test_dwa_controller_x`: 두 플러그인을 pluginlib 으로 한 프로세스에 올려 220 주기 명령을 비교한다.
- 2 라운드: `test_dwa_x_round2` (F1·F2·F3). 새 스위치를 모두 끈 chain_sim_x 는 1 라운드 빌드(85e66cb)와 비트 동일하다.
- 3 라운드: `test_dwa_x_round3` (G1). G1 을 끈 chain_sim_x 는 2 라운드 빌드(47feeaa)와 비트 동일하다.
  - 시험: `run_equiv_x.py`, held-out A 60 × dwa_full·X03·Y11 × 잡음 없음·seed 1
  - 시험: `scripts/run_equiv_x.py`, held-out A 60 × dwa_full·X03 × 잡음 없음·seed 1
  - 증거: `/home/CAPYI_AI/paper_research/runs/equiv/<UTC>_r2chain/`
- chain_sim 동치 시험 (prereg §3.3): `scripts/run_equiv_chain.py`, 증거는 `/home/CAPYI_AI/paper_research/runs/equiv/`.

## 연구 스위치

| 플러그인 파라미터 | chain_sim_x 키 | 형 | 기본 | 뜻 |
| --- | --- | --- | --- | --- |
| `<plugin>.research.reset_persist_cycles` | `reset_persist_cycles` | int | 0 | R1 K. 0 = 원본 규칙 |
| `<plugin>.research.reset_hard_v` | `reset_hard_v` | double | 0.6 | R1: 선속도 차가 이보다 크면 즉시 재설정 (K > 0 일 때만) |
| `<plugin>.research.jerk_consistent` | `jerk_consistent` | bool | false | R2 |
| `<plugin>.research.jerk_lim` | `jerk_lim` | double | 2.0 | R2 선저크 [m/s³] |
| `<plugin>.research.pp_candidate` | `pp_candidate` | bool | false | R3 |
| `<plugin>.research.log_resets` | `log_resets` (= 인자 `reset_log=`) | string | "" | 재설정 기록 CSV |
| `<plugin>.research.reset_model` | `reset_model` | bool | false | F1 사슬 모형 재설정 (2 라운드) |
| `<plugin>.research.reset_model_theta` | `reset_model_theta` | double | 0.15 | F1 θ [m/s] |
| `<plugin>.research.reset_model_theta_w` | `reset_model_theta_w` | double | −1 | F1 θ_w [rad/s]. ≤ 0 이면 θ·window_reset_w/window_reset_v (배포 0.15 → 0.25) |
| `<plugin>.research.model_delay` | `model_delay` | double | 0.04 | F1 서보 순수 지연 [s] |
| `<plugin>.research.model_tau` | `model_tau` | double | 0.08 | F1 서보 1차 시간 상수 [s] |
| `<plugin>.research.goal_extension_dist` | `goal_extension_dist` | double | 0 (끔) | F2 D [m] |
| `<plugin>.research.blocked_detour_shift` | `blocked_detour_shift` | double | 0 (끔) | F3 S [m] |
| `<plugin>.research.blocked_lookahead` | `blocked_lookahead` | double | 3.0 | F3 막힘 탐색 거리 [m] |
| `<plugin>.research.accel_hold` | `accel_hold` | bool | false | G1 가속 국면 재설정 유예 (3 라운드) |

R4(접근 지연)는 기존 파라미터 `approach_latency` 를 그대로 쓴다.
`reset_model_theta_w` 는 조정자 지시 목록에 없던 이름이라 더했다 (기본 −1 이면 θ 에서 유도하므로 격자 Y01–Y11 은 θ 만 바꾸면 된다).

### R1 — 창 재설정 지속 (`DwaPlannerX::windowCenterX`)
- 원본 규칙에서는 `|v_last − v_meas| ≤ window_reset_v` 이고 `|w_last − w_meas| ≤ window_reset_w` 이면 창 중심이 직전 명령, 아니면 측정값이다.
- K > 0 이면 차가 임계를 **K 주기 연속** 넘은 주기(K 번째 주기)부터 측정값이다. 그 전에는 직전 명령을 유지한다 (`suppressed`).
- 선속도 차가 `reset_hard_v` 를 넘으면 그 주기에 바로 측정값이다 (`hard`).
- 차가 계속 남으면 그다음 주기들도 측정값이다. 차가 임계 안으로 한 번 들어오면 연속 수는 0 부터 다시 센다.
- 직전 명령이 없는 주기(`has_last` false)와 M3 절제(`window_center_measured`)는 원본처럼 측정값이다. 둘 다 재설정 사건으로 세지 않는다.

### R2 — 저크 일관 방출 (`DwaPlannerX::jerkWindow`)
- 격자 표본 창(= 방출 명령 창)을 `[v_c + (a_last − jΔt)Δt, v_c + (a_last + jΔt)Δt] ∩ 가속 창` 으로 좁힌다. j = `jerk_lim`, Δt = 제어 주기.
  - `a_last` 는 직전 방출 가속 `(v_emit − v_c)/Δt` 다.
  - 재설정 주기가 아니면 v_c = v_last 라서 prereg 의 `[v_last + …]` 식과 같다.
  - 재설정 주기에는 측정값으로 다시 잡은 중심에서 잰다.
- 교집합이 비면 가속 창 안에서 띠에 가장 가까운 끝 한 점을 쓴다. 속도 상한·0 바닥·외부 제한이 저크보다 앞선다.
- **제동 후보(M4-d)는 예외**다. 좁히기 전 가속 창으로만 자른다. 모든 후보가 막히면 최대 감속이 그대로 나간다.
- **각속도:** 하류 velocity_profiler 는 ω 에 저크를 거는 경우가 제자리 회전(|v*| ≤ `curvature_min_speed` 0.05 m/s)뿐이다. 주행 중에는 곡률 보존(ω = κ·v_ref)과 α 제한만 건다.
  - 그래서 R2 도 표본 창 전체가 그 영역일 때(v 창 ⊂ [−0.05, 0.05])만 ω 에 같은 띠를 건다. 띠는 `jerk_lim_theta`(= `limits.max_angular_jerk` 6.0 rad/s³)로 만든다.
  - 주행 중에는 ω 를 건드리지 않는다.
- 통로 탈출(escaping, 동적 장애물 양보)은 안전 기동이라 좁히지 않는다.
- sustained_sampling 이면 방출 클램프 창이 같은 규칙을 따른다.

### R3 — 순수 추종 후보
- 후보 (v*, v*·κ_pp) 하나를 격자 뒤, 제동 후보 앞에 넣는다. 제동 후보는 늘 마지막이다.
- v* 는 표본 창으로 자른 `v_des` 다. v_des 에는 목표 접근 상한(M2), 헤딩 감속, 정렬 구간 0 이 들어 있다.
- lookahead `L = clip(heading_lookahead_gain·|v*| + offset, min, max)` 는 헤딩 항과 같은 식이다 (배포 0.4·v + 0.3, [0.4, 1.2]).
- 목표점은 로봇 투영 호길이 + L 의 경로 점이다. `κ_pp = core::PurePursuit::curvatureToPoint(x_L, y_L) = 2·y_L/(x_L² + y_L²)` 이다.
  - 원본 PP 와 같은 식이다. 목표점까지의 현 길이로 나누므로 원호가 그 점을 정확히 지난다. 직선 위·이탈 0 이면 L² 와 같다.
- ω 를 표본 창으로 자른 뒤 다른 후보와 같은 충돌 검사·비용으로 평가한다. 선택되면 `pp_selected` 가 켜진다.

### F1 — 사슬 모형 재설정 (2 라운드, `chain_model.hpp`, `DwaPlannerX::modelStep`)
- 직전 명령을 배포 하류 사슬의 명목 모형에 통과시켜 "지금 측정돼야 할 속도" v̂·ŵ 를 만든다.
  - 프로파일러 기준(50 Hz): 선속도는 `core::JerkLimitedFilter`(a = `acc_lim_x`, j = `jerk_lim_x` — 배포 profiler 와 같은 `limits.*` 출처, 1.0 · 2.0)이다.
  - ω 는 |v*| > 0.05 이면 곡률 보존 κ·v_ref(α·dt 변화율 제한), 아니면 각저크 필터(`jerk_lim_theta` 6.0)다. `core::VelocityProfiler::update` 의 기준 경로 그대로다.
  - 서보(100 Hz): 순수 지연 `model_delay` → 1차 `model_tau` → 가속 제한(`acc_lim_x` 1.0, `acc_lim_theta` 2.0). chain_sim `Servo`, kinematic_sim `ServoAxis` 와 같은 식이다.
- 이산화: 한 제어 주기에 서보 round(Δt_c/0.01) 스텝(20 Hz → 5)을 돈다. 프로파일러는 전역 서보 스텝 번호가 짝수일 때 갱신한다 (chain_sim 의 k % 2 시각표와 같다).
  - 직전 명령이 없는 주기(시작·예외 뒤)에 측정값으로 초기화한다.
  - ROS 시험대의 비동기 위상은 이 고정 위상으로 근사한다.
- 모형은 **명령만으로** 돈다. 프로파일러의 PI 보정·측정 재동기화·붙잡힘 판정은 넣지 않았다 — 측정과의 괴리를 재는 검출기에 측정을 넣으면 괴리가 흡수된다.
  - chain_sim 사슬과의 차는 최대 0.006 m/s, 0.0004 rad/s (`test_dwa_x_round2`).
- ω 도 같은 방식으로 판정한다 (각속도 사슬 모형이 마땅해서 기존 ω 규칙으로 되돌리지 않았다).
- 판정
  - 재설정 후보는 `|v_meas − v̂| > θ` 또는 `|w_meas − ŵ| > θ_w` 다.
  - R1 의 K 지속 규칙은 그대로다 (K = 0 이면 후보 즉시).
  - hard 는 원래대로 `|v_last − v_meas| > reset_hard_v` 로 판정한다 (K > 0 일 때). 모형 후보 여부와 무관하게 즉시 재설정한다 — 막힘·와인드업 방지.
- 기록 CSV 의 `v_hat,w_hat` 열과 dwa/stats [49] 에 v̂ 가 남는다. phase 는 원래 임계로 정규화한 모형 괴리가 큰 축으로 정한다.

### F2 — 목표 연장 (2 라운드)
- `d_goal < D` 이고 로봇이 아직 목표 앞이면(목표 yaw 축 위의 위치 < 0) 다음 세 가지가 쓰는 경로를 경로 끝에서 목표 yaw 방향으로 늘린 경로로 바꾼다.
  - 방향 항 목표점: 롤아웃 끝 투영 + ℓ
  - 경로 추종 항: J_path·J_off 의 수직 거리 투영
  - R3 후보 목표점
- 목표점은 원래 lookahead ℓ 를 그대로 유지한 채 경로 끝을 넘으면 연장선 위로 간다. 연장 길이는 ℓ_max + sim_time_max·v_max + 0.5 m 이고 간격은 0.05 m 다.
- 목표를 지나친 뒤에는 쓰지 않는다. 지나친 뒤에는 연장선 목표점이 로봇 앞(목표 반대쪽)에 생긴다. 그러면 헤딩 감속으로 v_des = 0 인 채 목표 yaw 로 정렬만 하고 서는 교착이 된다.
  - 기능 확인(held-out A, A04)에서 F1 0.15 + F2 0.6 이 목표 0.135 m 너머에 정렬한 채 멈춰 진행 검사로 실패해 고쳤다.
- 바꾸지 않는 것: d_goal, 목표 접근 상한(M2), 헤딩 감속(cos), 목표 판정, 여유 항의 기준 경로 비용, 충돌 검사 구간(d_goal + 여유), 정렬 구간(goal_align_distance)의 목표 yaw 비교.

### F3 — 막힌 경로 우회 (2 라운드, `DwaPlannerX::detourPath`)
- **막힘:** 로봇 투영 −0.3 m ~ +`blocked_lookahead` 안 경로 점에 발자국을 놓아 충돌이면 막혔다고 본다. 자세는 이웃 점 방향이고 비용 함수는 DWA 충돌 검사와 같은 것이다.
  - 경로 자세 θ 대신 이웃 점 방향을 쓰는 것은 M5 `recenterPath` 와 같은 방식이다.
- **우회:** 막힌 구간 [첫 막힌 점 − 0.3, 마지막 막힌 점 + 0.3] 을 옆으로 d 만큼 민다. 앞뒤는 길이 r = max(1, 2|d|) m 의 smoothstep 램프로 원래 경로에 합류한다.
  - 목표 쪽 램프는 목표 전에 끝나게 줄인다. 목표가 막혔으면 우회하지 않는다.
- **d 고르기:** 쪽마다 d = 0.05, 0.10, …, S 를 시험한다.
  - 로봇 앞의 옮긴 점(자세 = 옮긴 경로의 이웃 방향) 발자국이 모두 비충돌이어야 가능하다.
  - 가능한 것 중 옮긴 점 중심 비용 최대가 가장 낮은 d 를 고른다 (같으면 작은 d).
  - 쪽은 비용이 낮은 쪽이다. 직전 주기 쪽이 가능하면 그 쪽을 유지한다 (주기마다 뒤집히지 않게).
- **양쪽 모두 불가:** 원래 경로를 쓴다. 정지·제동은 DWA 가 한다.
- **기억:** 경로 창(path_horizon)이 로봇 바로 뒤부터라 막힌 점이 로봇 옆·뒤로 가면 창에서 사라진다.
  - 그래서 마지막 막힌 점(세계 좌표)과 d 를 ResearchState 에 기억한다.
  - 되돌아오는 램프가 로봇 뒤 0.3 m 에서 끝날 때까지 같은 d 로 잇는다.
- 상수(여유 0.3 m, 탐색 간격 0.05 m, 램프 max(1, 2|d|) m)는 파라미터로 내지 않았다.
- 계측
  - chain_sim_x JSONL: `n_detour`(우회 경로를 쓴 주기 수), `n_detour_blocked`(막힘을 본 주기 수)
  - dwa/stats: [46] 우회 옆 이동, [47] 막힘

### G1 — 가속 국면 재설정 유예 (3 라운드, `prereg_r3.md` §1, `windowCenterX`)
- 다음 셋이 모두 성립하면 **선속도 쪽 재설정 후보를 세지 않는다.**
  - (a) 직전 두 주기 방출 명령이 증가 중 (`v_last > v_last_prev`)
  - (b) `v_last > v_meas`
  - (c) `|v_last − v_meas| ≤ reset_hard_v`
- 세지 않는 후보는 두 가지다.
  - 임계 규칙의 `|v_last − v_meas| > window_reset_v`
  - F1 모형 규칙의 `|v_meas − v̂| > θ`
- 그 주기는 후보가 아닌 주기라서 K 연속 계수가 늘지 않고 0 으로 돌아간다.
- `v_last_prev`·`has_v_last_prev` 는 ResearchState 에 있다. 직전 두 주기 명령을 모르면(시작·예외 뒤) 유예하지 않는다.
- **hard:** G1 이 켜져 있으면 `|v_last − v_meas| > reset_hard_v` 는 K 와 무관하게 즉시 재설정이다 (이유 `hard`).
  - K > 0 이면 원래와 같다. K = 0 이면 원래도 즉시 재설정이었으니 이름만 threshold → hard 로 바뀐다. F1 + K = 0 에서는 모형 후보가 없어도 hard 가 걸린다.
- **ω:** 기존 규칙 그대로다. ω 차(임계 또는 F1 θ_w)가 넘으면 유예 조건이어도 재설정 후보다.
  - ω 용 별도 스위치는 두지 않았다. 1 라운드 chain 기록에서 재설정 299 건 가운데 ω 쪽(turn)은 15 건뿐이고, 진단(prereg_r3 §0)의 대상은 선속도 가속 국면이다.
  - "증가 중" 의 ω 대응(|ω| 증가인가, 부호 방향 증가인가)도 정의가 갈린다.
- 계측
  - `WindowCenterX::accel_held`: 유예가 없었다면 재설정 후보였던 주기
  - chain_sim_x JSONL: `accel_hold`, `n_g1_held`
  - 재설정 CSV 끝 열 `g1_held`: 실행 안 누적
  - dwa/stats: [50] 이번 주기 유예, [51] setPlan 이후 누적
- 시험 (`test_dwa_x_round3`, chain 사슬, F1 끔)
  - 정지 출발 직선: K6 재설정 1 → 0, 0 → 0.9 m/s 1.56 → 1.31 s. K0 는 명령 증가 중 재설정이 2 → 0 이고, 상한 도달 뒤 1 회는 남는다.
  - 막힘(측정 0): 명령이 0.65 m/s 가 된 주기에 hard 로 재설정한다 (K 0·6).
  - 감속(명령 감소) 상태: 무작위 4 000 판정에서 G1 끔과 같다. 잡음 폐루프에서 감속 주기 유예는 0 이다.

### 주기 사이 상태가 사는 곳
- `compute()` 는 const 다. 상태는 `DwaInputX::research` 로 들어가 `DwaResultX::next` 로 나온다.
  - 1 라운드: `persist_count`, `a_last`, `alpha_last`
  - 2 라운드: F1 사슬 모형 `model`, F3 쪽·기억 `detour_side`, `detour_mem`, `detour_shift`, `detour_end_x/y`
  - 3 라운드: G1 `v_last_prev`, `has_v_last_prev`
- 플러그인은 `research_state_` 멤버로, chain_sim_x 는 장면 루프의 지역 변수로 되먹인다.
- `has_last` 가 false 면(시작·activate·유효 후보 없음 예외 뒤) 코어가 상태를 0 으로 본다.

## 재설정 기록 CSV
- 창 중심이 직전 명령에서 측정값으로 돌아간 주기만 적는다 (reason ≠ none). 열은 다음과 같다.

  ```
  t,v_last,v_meas,w_last,w_meas,gap_v,gap_w,persist_count,reason,d_goal,phase,<식별 열>,v_hat,w_hat
  ```

- 2 라운드부터 맨 끝에 `v_hat,w_hat`(F1 예상 측정, 모형이 꺼져 있으면 `nan`)이 붙는다. 3 라운드부터 그 뒤에 `g1_held` 가 붙는다.
  - `g1_held` 는 이 실행에서 이 줄까지 G1 이 유예한 주기 수다 (chain_sim_x 는 장면, 플러그인은 setPlan 이후).
  - 앞 열은 1 라운드와 같다.

- `gap_*` 는 부호 있는 차(직전 명령 − 측정)다. `persist_count` 는 이 주기까지 차가 임계를 넘은 연속 주기 수다. `reason` 은 `threshold`(K = 0) / `hard` / `persist` 중 하나다. `d_goal` 은 남은 경로 거리이고, 목표가 창 밖이면 -1 이다.
- `phase`:
  - 선속도 차가 임계를 넘었으면 부호로 정한다. 명령 > 측정이면 `accel`, 그 밖이면 `decel` 이다.
  - 각속도 차만 넘었으면, 제자리 회전(|v_last| < 0.05) 또는 |w_last| ≥ 0.3 rad/s 일 때 `turn`, 그 밖이면 `cruise` 다.
- 식별 열
  - chain_sim_x: `scene,arm,seed`. `t` 는 장면 안 시뮬레이션 시각이다.
  - 플러그인: `plan_id,controller`. `t` 는 노드 시계(sim time)이고, `plan_id` 는 이 인스턴스의 setPlan 순번이다.
- 덧붙이기로 열고, 빈 파일일 때만 머리줄을 쓴다. 기록은 거동을 바꾸지 않는다 (동치 시험 `xlog` 묶음).

## chain_sim_x

```bash
chain_sim_x /abl/paths/scenes_heldout_smooth.txt out.jsonl cfg=/abl/cfg/dwa_full.cfg arm=dwa_x_X01 \
    reset_persist_cycles=2 stateful=1 reset_log=/runs/<stage>/<run>/resets.csv [seed=1 odom_noise_v=0.02 …]
```

- 연구 키는 cfg 파일 줄(`reset_persist_cycles 2`)로 줘도 된다. 원본 키·잡음 인자·출력 형식은 원본과 같다.
- 모르는 키는 원본처럼 종료 코드 2 로 거부한다.
- `stateful=1` 은 Nav2 1.1.20 `SimpleGoalChecker` stateful 을 흉내 낸다 (`GoalCheckEmu`). xy 0.10 m 안에 한 번 들면 xy 검사를 잠그고, 그 뒤로는 yaw 0.05 만 본다.
  - 비교 연산은 원본 chain_sim 그대로 둔다: dist ≤ 0.10, |dyaw| ≤ 0.05. Nav2 1.1.20 의 yaw 는 `<` 라서 정확히 같은 값일 때만 다르다.
- JSONL 끝에 더한 필드:
  - 설정: `stateful`, `reset_persist_cycles`, `reset_hard_v`, `jerk_consistent`, `jerk_lim`, `pp_candidate`
  - 재설정 계측: `n_resets`, `n_resets_hard`, `n_resets_persist`, `n_r1_suppressed`
  - 그 밖: `n_pp_best`, `n_jerk_applied`, `t_xy_latch` (stateful 잠금 시각, 없으면 -1)
  - 2 라운드
    - 설정: `reset_model`, `reset_model_theta`, `goal_extension_dist`, `blocked_detour_shift`
    - 계측: `n_detour`, `n_detour_blocked`, `n_goal_ext`
    - 첫 진입: `t_entry`, `yaw_err_entry` — 참값 자세가 목표 0.10 m 안에 처음 든 시각과 그때 |yaw 오차|, 100 Hz 플랜트 스텝 기준, 없으면 −1
  - 3 라운드: `accel_hold`, `n_g1_held`
  - 원본 필드의 이름·순서·형식은 그대로다.

## Nav2 에서 플러그인 올리기 (ROS 시험대)

작업공간을 빌드하고 `install/setup.bash` 를 소싱하면 pluginlib 이 `amr_nav_research::DwaControllerX` 를 찾는다.
확인은 `ros2 pkg prefix amr_nav_research` 로 한다. 원본 DWA 블록을 그대로 복사하고 `plugin` 과 `research` 만 바꾼다.

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath", "dwa_x_X01"]      # 원본 팔과 같은 서버에 함께 올려도 된다
    dwa_x_X01:
      plugin: "amr_nav_research::DwaControllerX"
      # ... 배포 FollowPath(amr_navigation::DWAController) 블록의 키를 모두 그대로 ...
      research:
        reset_persist_cycles: 2
        reset_hard_v: 0.6
        jerk_consistent: false
        jerk_lim: 2.0
        pp_candidate: false
        log_resets: "/runs/<stage>/<run_id>/resets_dwa_x_X01.csv"
        # 2 라운드 (Y 격자는 X03 = reset_persist_cycles 6 위에 덮는다)
        reset_model: false
        reset_model_theta: 0.15
        goal_extension_dist: 0.0
        blocked_detour_shift: 0.0
        blocked_lookahead: 3.0
        # 3 라운드 (Z 격자는 Y11 위에 덮는다)
        accel_hold: false
```

- 기록 파일은 플러그인 인스턴스마다 따로 둔다. 같은 파일을 두 인스턴스가 쓰면 줄이 섞인다.
- `log_resets` 를 쓰는 디렉터리는 controller_server 가 쓸 수 있어야 한다. 열지 못하면 경고만 남기고 기록을 끈다.
- dwa/stats 배열은 원본 [0]–[41] 뒤에 네 칸을 더한다.
  - [42] 재설정 이유 (0 없음 · 1 threshold · 2 hard · 3 persist)
  - [43] 차 초과 연속 주기 수
  - [44] R1 이 미룸
  - [45] R3 후보 선택
  - [46] F3 우회 옆 이동 [m] (0 = 우회 안 함), [47] F3 앞 경로 막힘, [48] F2 목표 연장 사용, [49] F1 예상 측정 v̂ (모형 꺼짐이면 −1e9)
  - [50] G1 이 이번 주기 선속도 후보를 유예함, [51] setPlan 이후 G1 유예 주기 수
- stateful 판정은 goal checker 플러그인 파라미터(`general_goal_checker.stateful`)다. 이 패키지가 아니라 하네스가 바꾼다.
  - Nav2 1.1.20 은 `ControllerServer::setPlannerPath` 에서만 `reset()` 한다. 그래서 FollowPath 목표 하나(ros_bench)에서는 잠금이 끝까지 간다.
  - BT 재계획으로 경로가 바뀌면 그때마다 풀린다.

## 빌드·시험

```bash
docker run --rm --name amr-ai-pr-build -v /home/CAPYI_AI/paper_research/ws:/ros2_ws -w /ros2_ws amr-ai/rl:0.3 bash -lc \
  'source /opt/ros/humble/setup.bash && MAKEFLAGS=-j16 colcon build --symlink-install --parallel-workers 8 \
   --packages-up-to amr_nav_research --cmake-args -DCMAKE_BUILD_TYPE=Release \
   "-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic -Werror" && colcon test --packages-select amr_nav_research && \
   colcon test-result --verbose'
```

빌드 플래그는 원본 절제 연구(prereg §1 "Release -Werror", `scripts/build.sh WERROR=1`, CI)와 같다.
린터는 원본 패키지와 같은 정책을 쓴다 (저작권 머리 검사 제외, cpplint `-legal/copyright`).

chain_sim 동치 시험 (컨테이너 안, `/abl` = `/home/CAPYI_AI/runs/dwa-abl` 읽기 전용, `/runs` = `paper_research/runs`):

```bash
python3 src/amr_nav_research/scripts/run_equiv_chain.py --out /runs/equiv/<UTC>_chain \
    --commit <sha> --dirty 0 --paths /abl/paths --cfg /abl/cfg --workers 12
```
