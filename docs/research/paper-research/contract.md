# paper_research 작업 계약 (C++ 사본 코드 ↔ 하네스)

## 0. 절대 규칙
- 기존 파일은 **한 줄도 고치지 않는다.**
  - 대상: `src/**`(새 패키지 `src/amr_nav_research/` 제외), `tests/**`, `scripts/**`, `docs/research/dwa-ablation/**`, `config/**` 등.
- 새 파일은 두 곳에만 둔다.
  - `src/amr_nav_research/**`
  - `docs/research/paper-research/**`
- 커밋 전 확인: `git diff --name-status 474264e..HEAD | grep -v -E '^A\s+(src/amr_nav_research/|docs/research/paper-research/)'` 출력이 비어야 한다.
- 원자료: `/home/CAPYI_AI/paper_research/runs/<단계>/<run_id>/`.
  - 덮어쓰지 않고, 끝난 디렉터리는 `/home/CAPYI_AI/paper_research/bin/seal` 로 봉인한다.
  - 일지: `/home/CAPYI_AI/paper_research/bin/jlog <agent> "<msg>"`.
  - 부하: `/home/CAPYI_AI/paper_research/bin/load`.
- 셸 안전: 장치 경로(/dev/null, /proc/…)를 프로그램 인자로 넘기지 않는다. `> /dev/null` 리다이렉트만 쓴다. `cp`·`mv`·`rm` 은 `command cp -f` 또는 Python `shutil`.
- 컨테이너: 이미지 `amr-ai/rl:0.3`.
  - 이름 `amr-ai-pr-<용도>`, 항상 `--rm`.
  - ROS_DOMAIN_ID 70–89, `ROS_LOCALHOST_ONLY=1`.
  - 작업공간: `/home/CAPYI_AI/paper_research/ws` → 컨테이너 `/ros2_ws`, 원자료 `/home/CAPYI_AI/paper_research/runs` → `/runs`.

## 1. 새 패키지 `amr_nav_research` (C++)
- 원본의 사본이다.
  - 코어: `src/amr_navigation/src/core/dwa.cpp` 와 헤더
  - 플러그인: `src/ros/dwa_controller.cpp`, `include/amr_navigation/dwa_controller.hpp`
  - 도구: `tools/chain_sim.cpp`
- 네임스페이스는 `amr_nav_research`, 클래스 이름은 `DwaPlannerX`, `DwaControllerX`.
- 원본 패키지 `amr_navigation` 의 다른 코어(속도 프로파일러, SpeedProfile, footprint, velocity_obstacle 등)는 **링크해서 그대로** 쓴다.
- 연구 파라미터. 플러그인 접두사는 `<plugin>.research.*`, chain_sim_x cfg 키는 같은 이름에서 `research.` 를 뺀다.

  | 이름 | 형 | 기본 | 뜻 |
  |---|---|---|---|
  | reset_persist_cycles | int | 0 | R1. 0 = 원본 규칙. K>0 이면 차가 임계(window_reset_v/w)를 K 주기 연속 넘을 때만 측정값 중심 |
  | reset_hard_v | double | 0.6 | R1. 차가 이 값(m/s)을 넘으면 K 와 무관하게 즉시 측정값 중심 (K>0 일 때만 쓰임) |
  | jerk_consistent | bool | false | R2. 방출 명령 가속 변화를 \|a_k − a_{k−1}\| ≤ jerk_lim·Δt 로 묶음 (제동 후보 예외) |
  | jerk_lim | double | 2.0 | R2 저크 한계 [m/s³] (하류 velocity_profiler 와 같은 값) |
  | pp_candidate | bool | false | R3. 순수 추종 곡률 후보 (v*, v*·κ_pp) 를 샘플에 추가 |
  | log_resets | string | "" | 창 재설정(측정값 중심으로 돌아간 주기) 기록 CSV 경로. 빈 문자열이면 끔 |

- **동치:** 연구 파라미터가 모두 기본값이면 `DwaPlannerX`/`DwaControllerX`/`chain_sim_x` 의 출력이 원본과 비트 단위로 같다. 이를 시험으로 보인다.
- **재설정 기록 CSV 열:** `t, v_last, v_meas, w_last, w_meas, gap_v, gap_w, persist_count, reason(threshold|hard|persist), d_goal, phase(accel|cruise|decel|turn)`.
- **plugin XML:** `amr_nav_research/plugins.xml`, 등록 이름 `amr_nav_research::DwaControllerX`.
- **chain_sim_x 추가 인자:**
  - `stateful=0|1` (기본 0 = 원본 판정)
  - `reset_log=<path>`
  - cfg 키로 연구 파라미터.
  - 나머지 인자와 출력 JSONL 형식은 원본과 같고, 필드만 추가할 수 있다.

## 2. 하네스 (Python, `docs/research/paper-research/scripts/`)
- 사본: `make_paths.py`, `plan_astar.py`, `params.py`, `orchestrate.py`, `ros_bench.py`, `analyze.py`. 파일 이름에 `_x` 를 붙인다.
- 추가할 것:
  - `scenes_obstacles.py`: O 장면 생성
  - ROS stateful 옵션: 모든 팔의 goal checker `stateful`
  - O 장면용 `moving_obstacles` 속도 0 주입과, 추적 장애물 토픽을 모든 팔에서 쓰지 못하게 리매핑 (`perception/tracked_obstacles` → `/pr_unused_tracks`)
  - 장애물 최소 이격 지표
  - J' 선택 규칙
  - H1r/H2/H3 판정
- **팔 이름:** 기존 이름을 그대로 쓰고, 연구 변형은 `dwa_x_X00`…`dwa_x_X11`, 선택된 것은 `dwa_R` 이다.

## 3. 일지 신호
- **C++ 쪽:** 동치 시험을 모두 통과하면 `jlog pr-core "CORE READY <commit>"` 를 남긴다. 하네스 쪽은 이 줄이 나온 뒤에 튜닝을 시작한다.
- **하네스 쪽:** 단계마다 시작과 끝을 남긴다. 예: `jlog pr-harness "STAGE <이름> START/END run_dir=… n=…"`.
