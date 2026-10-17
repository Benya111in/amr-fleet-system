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

R4(접근 지연)는 기존 파라미터 `approach_latency` 를 그대로 쓴다.

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

### 주기 사이 상태가 사는 곳
- `compute()` 는 const 다. 상태 `ResearchState{persist_count, a_last, alpha_last}` 는 `DwaInputX::research` 로 들어가 `DwaResultX::next` 로 나온다.
- 플러그인은 `research_state_` 멤버로, chain_sim_x 는 장면 루프의 지역 변수로 되먹인다.
- `has_last` 가 false 면(시작·activate·유효 후보 없음 예외 뒤) 코어가 상태를 0 으로 본다.

## 재설정 기록 CSV
- 창 중심이 직전 명령에서 측정값으로 돌아간 주기만 적는다 (reason ≠ none). 열은 다음과 같다.

  ```
  t,v_last,v_meas,w_last,w_meas,gap_v,gap_w,persist_count,reason,d_goal,phase,<식별 열>
  ```

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
```

- 기록 파일은 플러그인 인스턴스마다 따로 둔다. 같은 파일을 두 인스턴스가 쓰면 줄이 섞인다.
- `log_resets` 를 쓰는 디렉터리는 controller_server 가 쓸 수 있어야 한다. 열지 못하면 경고만 남기고 기록을 끈다.
- dwa/stats 배열은 원본 [0]–[41] 뒤에 네 칸을 더한다.
  - [42] 재설정 이유 (0 없음 · 1 threshold · 2 hard · 3 persist)
  - [43] 차 초과 연속 주기 수
  - [44] R1 이 미룸
  - [45] R3 후보 선택
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
