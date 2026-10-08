# P5 Gazebo 5 대 확증 (F1·F2·F3·F5) — 사전 등록

- 시각: 2026-10-08 15:40 KST (작성) · 에이전트: b4-p5 · 단계: P5 (`bonus4_completion_plan.md` §3 F1–F3·F5, §4 P5)
- 코드: 이 항목을 커밋한 `feature/b4-p5` (분기 `f97ece6` = bonus4-fleet, W 연결 포함). 실험은 이 커밋의 `git archive` 사본을
  빌드해 돈다 (아래 설정). 사본 커밋은 실행마다 `HEAD.txt` 에 남는다.
- 근거: 보고서 사본 `report-2026-10-06.md` 185행(장애 F1–F10 정의, 우선 F1·F3·F5·F7) · 193행(짝 = 같은 외생 입력 블록, A/A 6 쌍으로
  ρ · 발산 시각) · 220행(Gazebo 는 사전 등록 비교 2–3 개 × 쌍 8) · 250행(P5 행), `p3.md` §8 · §10.6 · §10.8, `p4.md` §0 · §4,
  `p0.md` §1 · §4 · §5, b4-s12 사전 등록 `…_1010_b4-s12_s12-pairs-prereg.md` (하네스 정지 관찰).
- 관련 실행 id: `P5_p5_<실험>_p<짝>_<갈래>[r<n>]` (`runs/b4-p5.jsonl`), 원자료 `/home/CAPYI_AI/runs/b24/b4-p5/raw/<태그>/`.
- 도구 (모두 `tests/integration/fleet_p5/`, 결과 전에 커밋 · sha256 은 아래 표): `p5_plan.py`(실험 · 짝 · 갈래 · 장애 일정의
  기계 판), `p5_recorder.py`(컨테이너 안 기록기 · 작업 소스 · 준비 판정), `p5_container.sh`(스택), `p5_server.sh`(서버 컨테이너),
  `p5_driver.py`(호스트 드라이버 · 장애 주입 · 분석 · 실행 목록), `p5_analyze.py`(지표 · 판정). 시험 `tests/integration/unit/test_fleet_p5.py`.

## 질문

1. **F1 회귀**: 분산 모드(`fleet_mode:=distributed`)로 Gazebo 5 대가 같은 작업 묶음을 돌 때 중앙 모드보다 완료 작업 수 · 완료 시간이
   나빠지는가 (A/A 잡음 바닥 대비)?
2. **F3 서버 장애 폴백**: 서버 컨테이너(관리자 + 교통 관리자)를 실행 중 kill 하면 분산 모드는 DEGRADED 로 넘어가 진행을 잇는가
   (중앙 모드 = 폴백 끔과 짝 비교), 그동안 **실제 자세 기준** 구역 동시 점유 0 · 낮은 epoch 명령 수용 0 인가, 서버 재기동 뒤
   관찰자 → ACTIVE (epoch 증가) · RESYNC → NORMAL 로 돌아오는가? 보고서 장애 F3(정지 뒤 재개) · F5(부분 분할) · F7(무장애 스트레스)
   에서도 같은 안전 불변식이 지켜지는가?
3. **F2 협상 할당**: 서버 장애 중 CBAA 협상이 할당하고 실제 실행기가 수행할 때, 같은 묶음을 중앙(nearest) 할당한 짝과 비교해
   완료 시간 · 이동 거리가 어떤가 (P4 모의: 협상이 같거나 조금 느림 — A6 − A4 지연 +1.5 [0.6, 2.5] s, 손실 0.2)? 일부러 만든
   작업 ID 잠금 경합에서 중복 실행이 0 인가?
4. **F5 도크 배치**: 도크 구역 ±1 m(옛 `fd17e49`) → ±0.75 m(새, `3f33338`) 가 중앙 모드 시나리오에서 도크 구역 실제 자세 동시 점유 ·
   도킹 성공 · 정밀도를 나쁘게 하지 않는가 (`p3.md` §10.6-3)?

## 가설과 예측 (결과를 보기 전에)

- **H1 (C1)**: 분산 모드는 완료 작업 수에서 회귀하지 않는다 — 5 작업 · 같은 중앙 할당(서버가 살아 있으면 분산 모드도 중앙 할당)이라
  차이는 교통(투표 경로 게이트 대 중앙 토큰 · hold)과 PickupLocked 확정 비용뿐이다. **makespan 은 분산이 조금 길 것**으로 본다 (결정점
  정지 · 투표 왕복, 확정 왕복). 버리는 조건: 완료 수 부호 검정 p < 0.05 로 분산 < 중앙.
- **H2 (C2)**: 장애 구간(kill ~ 재기동) 완료 작업 수는 분산 > 중앙 8/8 (중앙은 kill 뒤 새 할당이 없다 — 장애 중 게시된 묶음 B 3 건은
  재기동 전까지 시작 못 함; 진행 중이던 묶음 A 는 양쪽 다 끝낼 수 있다). 분산 갈래 kill → DEGRADED 는 sim 5–7 s (하트비트 dead 5 s +
  과반 의심). 위험 예측 (불리한 것): **중앙 재기동 뒤 관리자가 TRANSIENT_LOCAL 게시판 이력으로 끝난 작업을 다시 접수해 중복 실행**할
  것이다 (`p0.md` §0 "완료 대조 없음") — 분산 갈래는 작업 ID 잠금(확정 표는 풀지 않음)이 막아 0 일 것이다.
- **H3 (C3)**: 협상(DEGRADED) 대 중앙 할당의 makespan · 평균 작업 시간 · 이동 거리는 **가르지 못할** 것이다 (P4: nearest · Hungarian ·
  SSI 가 480 쌍 중 476 쌍 비트 같음, 5 대가 모두 대기 구역에 있어 할당 후보가 거의 같다). 방향은 P4 와 같게 협상 쪽이 같거나 조금 느릴
  것 (예측 확신 낮음). **혼입**: DEGRADED 에서는 중앙 교통 hold 를 따르지 않으므로(IsFleetServerAlive) C3 은 "할당만" 이 아니라
  "분산 모드 두 상태(NORMAL 대 DEGRADED) 전체" 의 비교다.
- **H4 (안전 불변식, 분산 모드 모든 실행)**: 실제 자세(원판) 기준 구역 동시 점유 0, 믿은 허가 겹침 0, 낮은 epoch 수용 0, 픽업 중복 0.
  위험 예측 (불리한 것): 원판 기준 동시 점유는 **0 이 아닐 수 있다** — 경로 게이트는 plan 이 한 번도 없으면 막지 않고(fail-open,
  `p3.md` §10.8), PERCEIVING · DOCKING 단계의 양보 이동은 구역 게이트를 거치지 않으며, 해제 게이트는 추정 자세(AMCL)에 기댄다.
  나오면 그 사건의 시각 · 자세 · 모드 · 단계를 표로 적는다 (해석은 결과 뒤, 판정은 미달).
- **H5 (F5L)**: 새 배치(±0.75 m)는 옛 배치보다 도크 구역 동시 점유(고정 지표 = 옛 큰 상자)가 **적거나 같다** (운동학 모의 6/30 → 0/30
  의 방향). 도킹 정밀도는 배치와 무관할 것 (도크 구역은 도킹 제어에 관여하지 않는다). Gazebo 차체 막힘 · Nav2 · AMCL 때문에 모의보다
  사건이 많을 수 있다.
- **H6 (기술 장애 S · P · X)**: S(정지 20 s 뒤 재개)에서 로봇이 DEGRADED → RESYNC → NORMAL 로 돌아오고, split-brain(정지 중 새 서버
  epoch + 1, 옛 서버 재개)에서 옛 서버의 하트비트 · 명령이 모두 거부된다 (fence_reject > 0, 낮은 epoch 수용 0). P(부분 분할)에서 소수 쪽
  {amr_01, amr_02} 는 ISOLATED · 새 진입 없음, 다수 쪽은 계속; 경합(서버 장애 중 01 ↔ 02 단절)에서 같은 작업 확정 요청이 둘 이상 나와도
  중복 픽업 0. X(지연 U(0, 1000) ms · 손실 0.1 · 버스트)에서 오탐 전환이 **나올 것**이다 (하트비트 2 Hz · dead 5 s, 버스트 평균 2.5 s —
  P4 는 F7 을 모의하지 않았다); 안전 불변식은 그대로.

## 설정

| 항목 | 값 |
|---|---|
| 시스템 | Gazebo Fortress `warehouse.sdf`, 5 대(`fleet_spawn.yaml` 대기 구역 (18…26, −16)), `multi_robot.launch.py` 전체 스택(위치추정 · Nav2 · 인식 · 실행기), 지도 `maps/warehouse.yaml` |
| 서버 | **별도 컨테이너** `amr-ai-b4-p5-srv-<태그>-<k>`: `fleet_manager.launch.py launch_adapters:=false` (관리자 + 교통 관리자, `fleet.yaml` 기본 — 할당 nearest). 스택 컨테이너와 네트워크 · IPC 이름공간 공유. 스택은 `launch_manager:=false traffic_manager:=false` (어댑터는 스택). **두 모드 모두 같은 배치** (짝 공정성). epoch 파일은 갈래마다 하나 (`/raw/<태그>/epoch.txt`, 재기동이 이어 씀) |
| 분산 모드 | `fleet_mode:=distributed`, 에이전트 저장소 `fleet_store_dir:=/raw/<태그>/store` (실행마다 새 경로 — 옛 펜스가 새 관리자를 거부하지 않게, `p3.md` §10.6-2) |
| 작업 | `/fleet/task_board` (RELIABLE · TRANSIENT_LOCAL · 256) 로 게시 — 관리자와 에이전트가 같은 게시판. 묶음 A = 시나리오 12 PLAN 앞 5 건 (dock_1→dock_a, dock_a→dock_1, dock_2→dock_b, dock_b→dock_2, dock_1→dock_b; 앞 넷은 같은 통로 반대 방향 쌍), 묶음 B = 그 뒤 3 건. **시나리오 12 를 고른 근거**: 명세 4.9/4.10 의 5 대 작업 시나리오이고 반대 방향 통로 쌍 · 도크 넷의 경합이 교통 · 구역 · 도크를 모두 지나며, 작업 시간 기록(sim 270–280 s)이 있다. 강제 교착 단계(b)는 쓰지 않는다 — E-stop 로봇을 순간 이동시켜 구역 투표를 건너뛰므로 분산 모드에서 의미가 없다 |
| 시각 | t_base = max(T_START, 준비 시각 + lead_s) (sim). 작업 · 장애 offset 은 t_base 기준. 통신 모의의 분할 · 단절은 절대 sim (T_START + offset) — 준비가 T_START 뒤면 `LATE_START` 로 표시 |
| 준비 판정 | 5 대 실행기 IDLE · lost 아님 · 추정 대 정답 ≤ 0.30 m 3 s 유지 · lifecycle_manager 11 개 is_active (900 s 뒤에는 응답 없는 관리자를 활성으로 보고 `lifecycle_assumed` 표시) · 실행기 `traffic/hold: 첫 값 수신` 5 대 · 분산이면 에이전트 5 대 NORMAL + 서버 ALIVE, 중앙이면 /fleet/status 수신 |
| 끝 | t_base + horizon (C1 · AA · C3 · F5L · X 660 s = 시나리오 12 상한, C2 · S 720 s, P 780 s), 또는 모든 작업 실행기 COMPLETED + 5 대 IDLE 5 s + sim ≥ t_base + min_end_s (C2 · S 360, P 460) |
| 기록기 | 하네스(launch_testing)를 쓰지 않는다 — b4-s12 의 시나리오 12 짝 12 갈래 중 7 갈래가 시험 스레드의 rclpy 서비스 호출(client.py call_async)에서 멈췄다. 단일 스레드 실행기 · UDP 전용 DDS 프로파일. 정답 자세 10 Hz, 추정 5 Hz, 사건 전부 (`p5_recorder.py` 머리말) |
| 짝 | 같은 계획 · 같은 장애 일정 (외생 입력 고정 — 보고서 193행). 짝마다 먼저 띄우는 갈래와 도메인((21, 23) / (25, 27))을 번갈아. 동시 실행 여부는 아래 파일럿 규칙 |
| 빌드 | 이 커밋의 `git archive` → `/home/CAPYI_AI/runs/b24/b4-p5/ws_new` (새 배치) · `ws_old` (같은 사본 + `fd17e49` 의 `traffic_zones.yaml` — 차이는 그 파일 하나, `traffic.yaml` 차이는 주석뿐). `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic -Werror" --packages-up-to amr_bringup`, cpu_slot |
| 이미지 | `amr-ai/rl:0.3` (두 컨테이너 · 분석 모두) |
| 자원 | 드라이버 전체를 `CPU_SLOTS=0 /home/CAPYI_AI/env/exclusive.sh b4-p5 …` 안에서 (gz 슬롯 둘). **b2-rec 녹화가 끝난 뒤에만** (COORDINATION §10.1). 짝 시작 조건 loadavg(1 분) < 8 두 번 연속 (최대 30 분 기다리고 그 뒤에는 그대로 시작하며 기록). 시작 · 끝 loadavg · GPU 를 갈래마다 |
| 장애 주입 | 호스트 드라이버가 clock.json(sim) 을 보고: kill = `docker kill` 서버 컨테이너 (보고서 F1 충돌, 관리자 + 교통 관리자 함께), pause/unpause = `docker pause/unpause` (F3 정지), start = 새 서버 컨테이너 (재기동, 분산이면 관찰자 3 s → max+1). 통신 장애(F5 · F7 · 경합)는 앱 수준 CommEmulator — 에이전트(환경 변수 `AMR_FLEET_COMM_MODEL_FILE`, 이 브랜치에서 더함) · 어댑터 · 관리자(launch 인자 `comm_model_file`)가 같은 파일 |

### 실험 (`p5_plan.EXPERIMENTS` — 표와 코드가 다르면 코드가 실행된 것이다)

| id | 짝 | 갈래 a · b | 작업 | 장애 (t_base 기준 sim s) | 목적 |
|---|---|---|---|---|---|
| AA | 6 | 분산 · 분산 | A @0 | — | A/A: ρ · 발산 시각 · 짝차 SD |
| C1 | 8 | 중앙 · 분산 | A @0 | — | F1 회귀 (사전 등록 비교 1) |
| C2 | 8 | 중앙 · 분산 | A @0, B @90 | 서버 kill @60, 새 서버 @300 | F3 폴백 (비교 2) |
| C3 | 8 | 분산(서버 kill @−30, 재기동 없음) · 분산(서버 생존) | A @0, lead_s 40 | — | F2 협상 대 중앙 할당 (비교 3) |
| F5L | 8 | 중앙 새 배치 · 중앙 옛 배치 | A @0 | — | F5 배치 짝 (비교 4) |
| S | 2 | 분산 · 분산 (복제) | A @0, B @200 | pause @60, unpause @80 (F3 정지 20 s), pause @240, 새 서버 @250, unpause @270 (옛 서버 재개 = split-brain), 옛 서버 kill @330 | 보고서 F3 + F10 (기술) |
| P | 2 | 분산 · 분산 (복제) | A @0, B @210 | 서버 kill @180, 새 서버 @420; 분할 {01, 02} \| 나머지 + 서버 [40, 100); 01 ↔ 02 단절 [200, 400) | 보고서 F5 + 경합 K (기술) |
| X | 2 | 분산 · 분산 (복제) | A @0 | 모든 링크 U(0, 1000) ms · 손실 0.1 · 버스트(좋음 50 s / 나쁨 2.5 s, 나쁨 손실 1.0) | 보고서 F7 (기술) |

- **경합을 일부러 만드는 방법 (F2)**: P 의 [200, 400) — 서버가 죽은 DEGRADED 에서 amr_01 과 amr_02 는 서로의 투표 · 비컨 · 협상을 못 받는다
  (나머지 셋과는 통한다). 둘 다 대기 구역(18, −16 · 20, −16)에서 IDLE 이면 같은 작업을 각자 낙찰했다고 믿고 확정을 요청한다 → 정족수가
  하나만 HELD. 분할 [40, 100) 중에는 관리자가 소수 쪽 로봇을 lost 로 보고 그 작업을 다수 쪽에 다시 줄 수 있다 → 같은 작업 확정 경합.
  공허 검사: 같은 작업에 확정 요청(task_commit_request)을 낸 로봇이 둘 이상인 사건 수를 센다 (0 이면 "경합이 일어나지 않았다" 로 적고
  중복 0 을 증거로 세지 않는다).
- 짝 수: 비교 4 개 × 쌍 8 (보고서 "2–3 개 × 쌍 8" 보다 하나 많다 — F5L 은 `p3.md` §10.6-3 요청), A/A 6, 기술 장애 3 × 2 짝(4 실행).

## 지표 (조작적 정의 — `p5_analyze.py` 머리말 · 함수가 원본)

| 지표 | 정의 |
|---|---|
| 완료 (1차) | 실행기 `task_status` COMPLETED 를 낸 작업 수 (게시한 작업 중) |
| makespan · 평균 작업 시간 | 마지막 완료 − t_base / (완료 − 게시) 평균 (완료한 작업만) |
| 이동 거리 | 정답 자세 경로 길이 합 (0.1 s 칸, < 0.02 m 걸음은 버림). C3 은 t_base ~ makespan 구간 (`distance_to_makespan_m`) |
| 실제 자세 동시 점유 (안전 1차) | 원판 r = 0.36 m 가 같은 구역 다각형과 겹치는 로봇 ≥ 2 인 0.1 s 칸이 이어진 구간 = 사건. 중심 기준 · 도크 고정 지표(옛 큰 상자)도 따로. 사건마다 최대 깊이 · 시각 · 자세. 노출 = 로봇-구역 진입 수 |
| 믿은 허가 겹침 | 분산: 같은 `zone:*` 를 두 로봇의 `AgentState.held` 가 동시에 |
| 허가 없는 진입 | 분산: 원판이 구역 Z 와 겹치는데 held 에 `zone:Z` 없음 — 초 합계 · 사건(시각 · 자세 · 모드 · 실행기 단계). plan 신선도 · 양보 이동 우회 · plan 없음 fail-open 의 결과 지표 |
| 픽업 · 중복 | LOADING 진입 때 그 로봇의 마지막 IN_PROGRESS 작업. 같은 작업 픽업 ≥ 2 = 중복 (다른 로봇이든 같은 로봇이든 — 재기동 뒤 재접수 포함) |
| 낮은 epoch 수용 | `assign_proxy` accepted 인데 epoch < 그 로봇의 그때 fence_epoch. fence_reject 수도 |
| 장애 반응 | kill · pause 뒤 로봇별 HOLD_NEW · DEGRADED 까지, start · unpause 뒤 RESYNC · NORMAL 까지 (sim s). 하트비트 epoch · role |
| 오탐 전환 | 장애 주입이 없는 분산 실행에서 t_base 뒤 NORMAL 밖으로 간 전환 수 |
| 결정점 정지 거리 | `gate`(blocking) 사건 뒤 첫 정지(정답 속력 < 0.05) 때 원판 경계 ↔ 막는 구역 거리 (음수 = 침범) |
| plan 신선도 | MOVING · RETURNING 시작 뒤 첫 plan 까지 시간, 2 s 넘게 plan 없이 움직인 수 |
| 해제 게이트 | `zone_release` 때 정답 원판이 아직 그 구역과 겹치는 수 (AMCL 별칭 · 자세 튐 — `p3.md` §8-3) |
| CommitTaskLock 실패 | `task_commit_request` 뒤 35 s(sim) 안에 그 로봇 held 에 `task:<id>` 가 없음. 실행 RTF 와 함께 (`p3.md` §8-14) |
| 같은 픽업 자세 모임 | 로봇 쌍 최소 중심 거리, 0.8 m 미만 구간 (`p3.md` §10.8) |
| 교착 | `/fleet/traffic_events` DEADLOCK · RESOLVED · UNRESOLVED 수 |
| 메시지 · 대역 | `/fleet_p2p/{vote,beacon,neg}` · `/fleet/heartbeat` 의 sim 초당 메시지 수 · 직렬화 바이트 |
| 영속 파일 | 실행 끝 `store/<로봇>/{zone,task,fence}.json` 크기 (`p3.md` §8-9 · 10) |
| 도킹 | DOCKING → LOADING · UNLOADING 전이 때 정답 자세 대 기대 도킹 자세(behavior.yaml marker_pose + 판 두께 0.02/2 + standoff, 방위 = 법선 반대) 위치 · 방위 오차, ≤ 0.02 m · ≤ 1° 수, DOCKING 진입 수 (재시도 포함) |
| RTF | (sim 경과)/(벽시계 경과), t_base 뒤 사건 기준 |

## 판정 규칙

- **짝 차**: b − a (갈래 이름 사전순: C1 분산 − 중앙, C2 분산 − 중앙, C3 dist_dead − dist_alive, F5L central_old − central_new). 양측 정확
  부호 검정(0 은 뺀다), t 기반 95 % CI, 같은 방향 짝 수 k/n. **n = 8 이면 부호 검정 최소 양측 p = 2/256 = 0.0078** (8/8 같은 방향) —
  7/8 이면 p = 0.070 이라 기각하지 못한다. 이 설계로 기각할 수 있는 것은 "8 쌍 모두 같은 방향" 인 효과뿐이다. A/A 는 n = 6 (ρ 의 추정은
  거칠다 — 수치만 적는다).
- **C1 (1차 완료 수)**: 부호 검정 p < 0.05 이고 분산 < 중앙이면 "회귀". p ≥ 0.05 면 "가르지 못함" 이고, 95 % CI 하한 > −1 작업이면
  "비열등(마진 1 작업) 충족" 을 덧붙인다 (모든 짝 차가 같아 CI 가 정의되지 않으면 그렇게 적는다). makespan 은 같은 검정 (보조).
- **C2 (1차 장애 구간 완료)**: kill ~ 새 서버 시작 사이 완료 수의 짝 차. p < 0.05 이고 분산 > 중앙이면 "폴백이 진행을 잇는다". 보조:
  전체 완료, 재기동 뒤 중복 픽업 수(두 모드), 분산 갈래 장애 반응 시각 · 재기동 뒤 epoch 증가 · 5 대 NORMAL 복귀 여부.
- **C3**: makespan · 평균 작업 시간 · 이동 거리(t_base ~ makespan) 각각 부호 검정 — 다중 비교 셋이므로 Holm 보정 값도 적는다. P4 방향
  (협상 ≥ 중앙)과 같은지 k/n 으로. 중복 픽업 0, 미완료 작업 수.
- **F5L**: 도크 구역 동시 점유 사건(고정 지표 원판 · 중심, 그리고 각 배치 자기 상자) 합 옛 대 새, 완료 · makespan 부호 검정, 도킹
  ≤ 2 cm · 1° 비율과 오차 중앙값 · 최대. "새 배치가 나쁘지 않다" 는 새 ≤ 옛 이고 완료 회귀가 없을 때만 쓴다.
- **안전 불변식 (모든 분산 실행 합)**: 원판 기준 동시 점유 · 믿은 허가 겹침 · 낮은 epoch 수용 · 중복 픽업 — 하나라도 > 0 이면 **미달**이고
  사건 표(시각 · 구역 · 로봇 · 자세 · 모드 · 단계 · 깊이)를 결과 문서 앞에 둔다. 0 이면 노출(진입 수 · 확정 수)로 rule of three 상한을 적는다.
  중앙 모드의 동시 점유는 비교 · 기록만 한다 (중앙 교통은 정족수 안전을 주장하지 않는다).
- **유효 실행**: 기록기가 DONE 으로 끝나고 감시 중단이 없을 것. 무효(READY_TIMEOUT · CLOCK_STALL · 감시 중단)는 짝째로 새 태그로 다시 돈다.
  같은 짝 연속 무효 3 번이면 그 실험을 멈추고 진단한다 (재시도 안 함). `lifecycle_assumed` · `LATE_START` · hold 첫 값 미수신 로봇은 유효로
  세되 표시한다 (P 실험의 LATE_START 는 통신 일정이 어긋나므로 기술 표에서 따로).
- 문구: 차이를 가르지 못하면 "가르지 못함" — "효과 없음" · "같다" 를 쓰지 않는다. 공유 부하 하 값은 "공유 부하 하 예비값" (loadavg 함께).

## 파일럿 (셈 밖, 본 실행 전) — 동시 실행 · T_START 규칙

- 짝 1 (차례로 = 단독 RTF): 중앙 → 분산, 짝 2 (동시): 중앙 + 분산. 작업은 C1 과 같은 묶음.
- **동시 실행 규칙**: 짝 2 의 두 갈래 RTF 가 모두 ≥ 0.6 × 짝 1 같은 모드 RTF 이고, 짝 2 두 갈래가 DONE 이면 본 실행은 짝 두 갈래를 동시에
  (처리량 ≥ 1.2 배 · 짝 안 부하가 같다). 아니면 차례로 (먼저 띄우는 갈래를 짝마다 번갈아).
- **T_START 규칙**: T_START = max(240, 30 의 배수로 올림(1.25 × 파일럿 네 갈래 t_ready 최댓값)). C3 은 lead_s 40 이 더해진다.
- 파일럿에서 도구 결함이 나오면 고치고 파일럿을 다시 돈다 — 이것은 도구 결함 수정이지 결과(지표 값)에 따른 변경이 아니다. 고친 내용 ·
  새 sha256 을 이 항목에 "결과 뒤 변경 (도구)" 로 적는다. 파일럿 지표 값으로 가설 · 판정 규칙을 바꾸지 않는다.

## `p3.md` §8 · §10.6 · §10.8 확인 항목 — 이번에 하는 것 · 안 하는 것

| 항목 | 이번 | 지표 / 이유 |
|---|---|---|
| §10.6-1 분산 모드 기동 확인 | 함 | 준비 판정(에이전트 5 대 NORMAL), launch.log 의 assign_task 서버 (서비스 그래프는 기록하지 않음 — 실행기 · 에이전트 사건으로 대신) |
| §10.6-2 실행마다 새 저장소 · epoch | 함 | 설정 표 |
| §10.6-3 F5 배치 Gazebo 짝 | 함 | F5L |
| §10.6-4 결정점 정지 실제 거리 | 함 | 결정점 정지 거리 (gate 사건) |
| §10.6-5 plan 신선도 · plan 없음 fail-open | 함 | plan 신선도 · 허가 없는 진입 |
| §10.6-6 중앙 교통 결합 · 교착 빈도 | 함 | C1 교착 수 |
| §10.6-7 협상 + 실제 Nav2, 낡은 LOCKED 정리, CommitTaskLock 시간 초과 | 함 | C3 · C2 · P: stale_lock_cleared 사건 수, CommitTaskLock 실패 · RTF |
| §10.6-8 서버 컨테이너 kill | 함 | C2 · S · P |
| §8-3 해제 게이트 + AMCL 별칭 | 함 | 해제 게이트 (원판이 구역 안인데 해제) |
| §8-4 RTF 하트비트 오탐 (F7) | 함 | 오탐 전환 (AA · C1 · C3 생존 · X) |
| §8-6 메시지 수 · 대역 | 함 | 메시지 · 대역 |
| §8-7 에이전트 재기동 (고아 구역 해제 · 작업 잠금 유지) | **함 (17:55 보완 — 실험 R)** | 처음에는 "안 함"(런치가 respawn 하지 않고 같은 파라미터로 다시 띄우는 도구가 없음)이었다. 같은 명령줄 · 환경을 그대로 다시 쓰는 도구 `p5_agent_ctl.py` 로 풀었다 — 아래 보완 절 |
| §8-8 좁은 통로 교착 스트레스 | **안 함** | 시나리오 12 작업은 narrow_aisle 을 지나지 않는다. 좁은 통로 전용 작업 · 배치를 새로 만들어야 한다 — 시간 우선순위로 뒤로 (P4 도 Gazebo 값을 우선하라고 했다 — 결과 문서의 "주장할 수 없는 것" 에 적는다) |
| §8-9 · 10 영속 파일 · 재전송 | 일부 | 영속 파일 크기만. 투표자 영구 사망(F6) 실험은 안 함 (보고서 우선순위 밖) |
| §8-11 프록시 시간 초과 잠정 잠금 | 기록만 | 에이전트 사건(central_timeout_ignored 등) 수 |
| §8-12 분할 전 VOTED 다수 쪽 확정 실패 | 함 | P 의 CommitTaskLock 실패 (분할 구간) |
| §8-15 ISOLATED (launch 수준) | 함 | P 의 소수 쪽 모드 전환 |
| §10.8 양보 이동 구역 게이트 우회 | 함 | 허가 없는 진입 사건의 실행기 단계(PERCEIVING · DOCKING) |
| §10.8 같은 픽업 자세 모임 | 함 | 최소 거리 · 0.8 m 미만 구간 |
| §10.8 F5 고정 지표 | 함 | 도크 고정 지표 |
| §10.6-10 formal VOTED · §10.6-11 ④c 경주 · §10.6-12 같은 robot_id 둘 | 안 함 | 이 과제 밖 (f2 · P3 소유자 결정 · 프로토콜 밖) |

## 시간 어림 (보고서: 실행 1.5–2 일)

[추정] 공유 부하 하 5 대 Gazebo RTF 0.15–0.32 (b4-s12 동시 짝 rtf_mean 0.15–0.22, hold-dds 단독 0.21–0.32). 준비 5–8 분(벽시계), 묶음 A
sim ~300 s. 동시 짝이면 AA · C1 · C3 · F5L 짝당 ≈ 40 분 × 30 = 20 h, C2 짝당 ≈ 70 분 × 8 = 9.3 h, S · P · X ≈ 70 분 × 6 = 7 h, 파일럿 ≈ 2 h —
**합 ≈ 38 h (1.6 일)**. 차례로면 1.4–1.8 배. 무효 재실행은 넣지 않은 값이다.

## 분석 스크립트 (결과 전에 고정) — 커밋 `8c5e9ff`

| 파일 | sha256 |
|---|---|
| `tests/integration/fleet_p5/p5_analyze.py` | `37d33ae7bba599f07955e33acf2fbb684c0a533dc17bc9c89aaabf6e99131bc3` |
| `tests/integration/fleet_p5/p5_driver.py` | `8c53eded623b9ad8f4d1d4c37d69375b233dccca8c3de99dcdfdf28269eb8250` |
| `tests/integration/fleet_p5/p5_plan.py` | `95480ac2be7a76210f23e68f77f1e39163b0fba5a5780e64569665369ab9c195` |
| `tests/integration/fleet_p5/p5_recorder.py` | `df2b6fb1f6f414a108abbb2a675a88ea7a40d40d8c92d61f77b17fb5621992bf` |
| `tests/integration/fleet_p5/p5_container.sh` | `73665598adf88c67a3b252722a729dc80ee8c8d1f9b134aa904a9ed9dc03e6f4` |
| `tests/integration/fleet_p5/p5_server.sh` | `7d36d6e55d7f4bbddf0141e111fc2599da16ac42cd7b9afd7b276796f80d5cae` |
| `tests/integration/unit/test_fleet_p5.py` | `b6ef062dff98711ec5231e3774136ef635bf729495b741baee621fcbbecdb2bc` |

- 시험 [측정]: `unit/test_fleet_p5.py` 24 통과 (호스트 Python 3.13 · 컨테이너 3.10), 하네스 unit 전체 125 통과 (`2b2167d` 기준),
  amr_fleet colcon test 913 / 실패 0 / 건너뜀 1 (`2b2167d`, 공유 부하 loadavg 65–92 하, 8 분 9 초).
- 분석기 가드 변이 [측정, `/home/CAPYI_AI/runs/b24/b4-p5/tmp/mut/mut_result_v2.txt`]: 12/12 KILLED — 동시 점유 문턱(≥ 2 → ≥ 3), 원판 →
  중심, 픽업 중복 문턱, 낮은 epoch 비교 방향, 믿은 허가 겹침 문턱, 허가 없는 진입 조건, 확정 창 35 → 1 s, 해제 게이트 조건, 부호 검정
  양측, 도크 고정 지표 → 자기 상자, 오탐 전환 조건, 완료 상태(IN_PROGRESS 를 완료로).
- 파이프라인 시험 [측정, Gazebo 없음, `/home/CAPYI_AI/runs/b24/b4-p5/dry/`, 셈 밖]: 실험 `dry` (가짜 /clock RTF 1, 서버 컨테이너 pause @250 ·
  unpause @260 · kill @270 · 새 서버 @280.5 sim — 예정 대비 지연 ≤ 0.5 s sim), 분산 갈래 하트비트 epoch 1(관찰자 → ACTIVE) → 2 (재기동 관찰자 →
  ACTIVE), 재기동 관리자가 게시판 이력의 작업 2 건을 다시 접수 (H2 의 위험 기제가 실제로 있다는 확인 — 지표 값이 아니다).

### 사전 등록 보완 (결과 전, 2026-10-08 16:10 — Gazebo 실행 0 회 시점)

- **캠페인 스크립트 `p5_campaign.py` 추가** (감독 에이전트가 끊겨도 측정이 이어지게): `wait` 는 b2-rec 녹화 끝 조건(녹화 드라이버 ·
  시도 프로세스 없음 + b2-rec `recordings.md` §0 이 자리표시가 아님, 10 분 연속; 수동 `GO` 파일)을 기다린 뒤 exclusive 로 넘어가고,
  `run` 은 파일럿 짝 1(차례로) → 짝 2(동시) → 위 「파일럿」 규칙을 코드로(`decide`) → 점검(파일럿 네 갈래 유효 · 정답 자세 이동 · 완료 ≥ 1 ·
  분산 에이전트 상태 기록) 통과면 본 실행. 점검 실패면 본 실행을 시작하지 않는다 (`CAMPAIGN_HALT.txt`). 결정과 근거는
  `raw/pilot_decision.json`. 규칙 자체는 위 「파일럿」 절 그대로다.
- **짝 시작 대기 상한 1800 s → 300 s**: loadavg(1 분) < 8 두 번 연속을 기다리는 상한. 다른 사용자 부하가 계속 8 을 넘으면 짝마다 30 분을
  버린다 (짝 44 개면 22 h). 짝 안 두 갈래는 동시에 돌거나(같은 부하) 번갈아 돌고, 시작 · 끝 loadavg 를 기록하므로 판정에는 영향이 없다.
- 해시: `p5_campaign.py` `c6822c6a381c59c0dab26aab2e334889e0e84f3557fef067e18b96f87dc766e5`, `unit/test_fleet_p5.py` `a20951ffd5a40bf4161231fbfcfdf610d045b22943ba1cc14b2b7f74651ed412`
  (27 통과 — 파일럿 규칙 · b2-rec 조건 시험 3 개 추가). 나머지 도구 해시는 위 표와 같다 (바꾸지 않았다).
- 실험 사본: `ws_new` · `ws_old` 를 이 보완 커밋의 `git archive` 로 갱신했다 (`src/` 는 `41e90ca` 와 바이트 같다 — 빌드 재사용, 확인 명령
  `git diff --stat 41e90ca <이 커밋> -- src` 빈 출력).
- **(16:20 보완, 결과 전) 스택 컨테이너 프로세스 표**: 드라이버가 갈래 시작 420 s(벽시계) 뒤 `docker exec <스택> ps` 를 `ps_stack.txt` 로
  남기고, 분석이 `stack_has_server`(스택에 fleet_manager_node · traffic_manager_node 가 있나)를 센다. 스택 런치에 준
  `launch_manager:=false traffic_manager:=false` 가 하위 런치로 상속되는지는 Gazebo 없이 확인하지 못했다 — 상속이 안 되면 관리자가 둘(스택 +
  서버 컨테이너)이 되어 실험이 무의미하므로 파일럿 점검 항목에 넣었다 (어긋나면 본 실행을 시작하지 않는다). 해시 갱신: `p5_driver.py`
  `d52dd71c79d3cb9de65ccc85f029be7ed6b5ab7f3e6a811a48a79f6ad20bf216`, `p5_analyze.py` `f7597db93c51cf44d76ad2a7cfcd4bbd65c8e342c61d12dad3bba00329565fb2`
  (지표 정의 변경 없음 — `stack_processes` · `stack_has_server` 두 필드 추가), `p5_campaign.py`
  `64dcda2971094ede61cec15189db3e72f502acc9f10d36e086593f52e97aa6dd`, `unit/test_fleet_p5.py`
  `6fc87dbe147b09f3db1ed596f8caa20c0557fdd6c2a01b5a62e49b914f4361c7` (28 통과).
- **(17:55 보완, 결과 전 — Gazebo 실행 0 회) 실험 R 추가 · §8-7 을 "안 함" → "함"**: b2-rec 녹화를 기다리는 동안 에이전트 재기동 도구를
  만들었다. `p5_agent_ctl.py` — 스택 컨테이너 안에서 `fleet_agent_node`(argv[0] 또는 python 의 argv[1] 이 실행 파일이고 정확히
  `__ns:=/<로봇>` 인 프로세스만)를 SIGKILL 하고 그 명령줄(런치가 만든 `/tmp/launch_params_*` 포함)을 남긴 뒤, `p5_container.sh` 가 남긴
  `export -p` 환경으로 **같은 명령줄 · 같은 store_dir** 로 다시 띄운다. 도구 시험 [측정, `/home/CAPYI_AI/runs/b24/b4-p5/tmp/agentctl/`]:
  가짜 `fleet_agent_node` 셋(amr_03 · amr_033 · 그 낱말이 든 셸) 중 amr_03 만 죽고, 같은 argv · 환경 변수로 다시 떴다. 첫 판은 명령줄
  문자열에 그 낱말이 든 셸(PID 1)을 골랐다 — 시험이 잡았고 실행 파일 위치로 고쳤다.
  - **R**: 짝 2 (분산 · 분산 복제), 묶음 A @0, `agent_kill amr_03` @60 → `agent_restart amr_03` @75 (sim s). 지표: 재기동 boot 사건
    (incarnation 증가, 디스크에서 남긴 구역 kept_zones · 작업 잠금 kept_tasks), 남긴 구역의 첫 해제까지 시간, 그 사이 다른 로봇이 그 구역을
    받았는지(믿은 허가 겹침 · 원판 동시 점유 — 안전 불변식과 같은 판정), amr_03 작업 완료 여부. **가설**: 재기동한 에이전트는 몸체가 구역
    안이면 그 구역을 계속 쥐고(fail-closed), 밖이면 해제한다; 작업 잠금(HELD)은 남는다; 죽은 15 s 동안 그 로봇의 실행기는 상태가 3 s 넘게
    오래돼 `ZonePathClear` 가 막아 멈춘다 (안전 쪽). 순서 `ORDER` 끝 (X 다음).
  - 해시 갱신: `p5_agent_ctl.py` `586e19a4bba00f1540cb689a2beafc740dd87a6f9f75a63fe33822dcaf6f01dd`, `p5_analyze.py`
    `f3c31b17e0783a524c21f7eecfb8d97182c9682781e902034bbf792638c574b9` (boots · 재기동 반응 추가, 다른 지표 그대로), `p5_driver.py`
    `8b02b128ac6dda94d33ac37f4e310c1765bc8d0672710742e9c4e20389e0e55e`, `p5_plan.py` `0cf2471c0c8335e8edb5b3fc48e52448c629ff806747d9873beb8adc6f38a60c`,
    `p5_container.sh` `dfd2b54f945b327de41320741d07f4fb232165aa6f35f045cef1562931059134` (환경 덤프 한 줄), `unit/test_fleet_p5.py`
    `f49e9e3c143307f88e1537f2ff763bb3e15de371656bf8c72f2507e5d72ce71f` (29 통과), 형식 도구 `p5_report.py`
    `132d3b3eb299d21a924c7c07722e0732a8d2952faa1bc2846e5a359dc1e3e53f`. 시간 어림 + 1.5 h.
- **(22:20 결과 뒤 변경 — 운영 파라미터, 파일럿 첫 갈래 진행 중) T_START 하한 240 → 60 s**: 파일럿 첫 갈래(`p5_pilot_p1_central`)가
  sim 9.4 s(벽시계 약 1 분)에 준비를 마쳤다 [측정, `raw/p5_pilot_p1_central/events.jsonl` ready]. 하한 240 이면 갈래마다 sim ~230 s(RTF 0.25 면
  벽시계 ~15 분)를 그냥 기다린다 — 본 실행 92 갈래(동시 짝 46)면 약 9–11 h. 하한을 imu_filter `bias_estimation_time` 60 s(정지 평균 창)로
  낮춘다: T_START = max(60, ceil30(1.25 × 파일럿 t_ready 최댓값)). **이 값은 작업 게시 시각만 정하고 지표 · 판정 규칙과 무관하다** (짝의 두 갈래는
  같은 T_START). 파일럿 지표(완료 · 안전 등)는 아직 하나도 나오지 않았다 — 준비 시각만 보고 바꿨다.
- **(같은 때, 결과 전 결함 수정) 통신 모의 절대 시각**: `p5_plan.comm_yaml` 이 실행의 t_start 가 아니라 모듈 기본 T_START(240)로 분할 · 단절
  시각을 썼다 — `--t-start` 를 바꾸면 P 의 분할이 작업 일정과 어긋났을 것이다. 실행의 t_start 를 받게 고치고 시험으로 고정했다 (P 실행 0 회).
- 해시 갱신: `p5_plan.py` `11ea9e75f3cf6b77a2e4b28b332de624dc28a2e7a7926ac5c23925b3776fb60d`, `p5_campaign.py`
  `26455d05e7d61f068c2b8d33ef605f28f5e44aac2d2224e66ef5f76f6dfc515d`, `unit/test_fleet_p5.py` `7150b6a575da0bab5dd4b3822f5a3e0310b9e6daf8593c9fcb5dc38acb704740`
  (31 통과). 이미 돈 캠페인 프로세스는 옛 규칙을 쥐고 있으므로, 파일럿 짝 2 시작 뒤 STOP 파일로 본 실행 드라이버를 첫 짝 전에 멈추고 새 코드로
  캠페인을 다시 띄운다 (파일럿 짝은 `.done` 이라 건너뛴다).
- **(23:05 결과 뒤 변경 — 도구: 준비 판정 · 준비 단계 조치, 파일럿 짝 1 에서 발견)** 파일럿 짝 1 [측정]: 중앙 갈래
  `p5_pilot_p1_central` 은 sim 9.4 s 에 준비, 5/5 완료(makespan 283.5 s), 도킹 10/10 이 ≤ 2 cm · 1°, 스택 컨테이너 프로세스 172 개 ·
  관리자 없음, RTF 0.266 (공유 부하 하 예비값, 시작 load1 8.4). **분산 갈래 `p5_pilot_p1_distributed` 는 준비하지 못했다**: 기동 때
  lifecycle 전이 응답이 유실돼(`failed to send response to /amr_0X/amcl/change_state (timeout)`) amr_02 · 03 · 05 의 AMCL 과 amr_04 의
  Nav2 가 늦게(lifecycle_watchdog 직접 전이, 벽시계 +100 s) 활성됐고, AMCL 이 초기 자세를 놓쳐 amr_02 · 03 · 05 가 lost 를 되풀이했다
  (추정 오차 46–54 m, kidnap_monitor `recovery exhausted` · `alias margin blocks convergence`). 관리자 is_active 는 끝까지 false 였다
  (응답 유실 — 하네스가 아는 결함, cases.wait_lifecycle_active 의 wedge 폴백). 이것은 플릿 모드 효과의 증거가 아니다 (기동 시점 lifecycle
  응답 유실은 b4-s12 · 시나리오 12 기록에도 있다). sim 194 s 에 수동으로 멈췄다 (`MANUAL_STOP.txt`), 파일럿 짝 1 은 무효 → 새 태그로 다시.
  - 고친 것 (지표 · 판정 무관, 두 모드 같은 규칙): (1) lifecycle 판정 — 관리자 is_active 가 모두 true **또는** launch.log 에
    `lifecycle_watchdog: 감시 대상이 모두 active` 가 마지막 `멈춰 있다` 경고 뒤에 있으면 통과 (`summary.lifecycle_watchdog` 표시).
    (2) **준비 단계 자세 알림** — 작업 게시 전, 로봇이 lost 이거나 추정 오차 > 0.30 m 가 sim 15 s 이어지면 그 로봇의 정답 자세를
    initialpose(3 회) + map EKF set_pose + AMCL 무이동 갱신으로 준다 (하네스 `actions.seed_pose` 와 같은 묶음, 로봇당 20 s 간격 ·
    5 회 상한, 사건 `seed` · `summary.seeds`). 로봇은 작업 전이라 대기 구역에 서 있다 — 운영자가 자세를 알려 주는 준비 단계이고,
    작업 게시 뒤(t_base 뒤)에는 하지 않는다. 분석은 seed 가 있었던 실행을 표시한다. (3) 파일럿도 T_FLOOR(60 s)에 게시.
  - 해시: `p5_recorder.py` `71890edc4778ce640d96dc105d6c1dcfcdf5ee9ad23759825747a91551b7cac4`, `p5_analyze.py` `14fc14a8606210b7638c8c58e01119754765fa0df25873eadbfd2c3eff928429`
    (seeds · lifecycle_watchdog 필드만 추가), `p5_campaign.py` `800023fa5c20da4dd8107dbbe76dc11b923cd6ae01dd0682eaa833be210e926c`,
    `unit/test_fleet_p5.py` `cc4badcdafce2ec7962b9923219685fa4bc38b6d78ebdc38f2aa6d847a980591` (컨테이너 32 통과).
- **(00:40 결과 뒤 변경 — 분석, 파일럿 짝 1 재시도에서 발견, 본 실행 0 회)** 파일럿 짝 1 재시도 [측정, 셈 밖]: 중앙 `…_centralr2`
  5/5 · makespan 552.1 s (교착 1 · 해소 실패 2 — dock_1_approach 토큰 120 s 대기), 분산 `…_distributedr2` 5/5 · 295.0 s, 두 갈래 모두 원판 동시
  점유 0 · 도킹 10/10 ≤ 2 cm · 1° · 자세 알림 1 대씩 · lifecycle_watchdog 폴백. RTF 0.242 · 0.168 (load1 58–121, 다른 사용자 openroad 작업 —
  공유 부하 하 예비값). 메시지 [측정, 분산]: 비컨 9.85 /s (1.85 kB/s), 투표 3.17 /s (0.79 kB/s), 하트비트 2.0 /s.
  분석 결함 둘 (정의 변경 아님 — 원자료로 다시 계산): (1) 드라이버가 갈래 시작 때 서버 1 기동을 `start`(scheduled 없음)로 적어, 분석이 그것을
  주입 장애로 보고 오탐 전환을 세지 않았고(None) **C2 장애 구간 완료 수의 '첫 start' 가 서버 1 기동(sim 없음)이 되어 계산이 비었을 것이다** —
  scheduled 가 있는 것만 주입 장애로 센다. (2) 사전 등록의 결정점 정지 정의("gate 뒤 첫 정지")는 허가를 받은 **뒤**의 정지(도킹 · 적재)까지
  골랐다 (파일럿 dock_b_approach −1.06 m 등). 원래 정의 값(`gate_stops`)은 그대로 두고 **보충 지표**를 더한다: 그 로봇 held 에 zone:Z 가 들어오기
  전(최대 30 s) 첫 정지만 결정점 정지(`gate_stops_before_grant`), 정지 없이 허가를 받으면 지나감(`gate_pass_without_stop` — 사전 요청 효과).
  해시: `p5_analyze.py` `8ad6852b435da4d7362ed83c10785d789f8f908d371639ba181759d4b62400db`, `unit/test_fleet_p5.py`
  `9e9eb017872934834b868a99b8edfc6e3718d60c45c909fa59d4de5030770465` (34 통과, 가드 변이 12/12 KILLED 다시 확인 `tmp/mut/mut_result_v3.txt`).
- **(00:50 파일럿 짝 2 결과와 결정 — 규칙 그대로)** 파일럿 짝 2(동시) [측정, 셈 밖]: 다른 사용자 부하(load1 91–121, openroad) 아래 5 대 스택
  둘을 함께 띄우자 lifecycle bond 시간 초과(`SERVER amcl IS DOWN after not receiving a heartbeat for 30000 ms`) · AMCL change_state 실패가 잇따랐고,
  분산 갈래는 /clock 이 sim 1.7 s 에서 사실상 멈춰 기록기가 CLOCK_STALL 로 끝났으며 중앙 갈래는 벽시계 10 분에 sim 101 s · 준비 못 함 (수동 중단,
  `MANUAL_STOP.txt` — 재시도 3 회 × 30 분을 피하려고). **규칙: 짝 2 가 DONE 이 아니므로 본 실행은 차례로** (짝 안 먼저 띄우는 갈래를 번갈아).
  T_START = max(60, ceil30(1.25 × 25.0)) = **60 s** (파일럿 짝 1 재시도 t_ready 22.0 · 25.0 s). 짝 1 단독 RTF [측정]: 중앙 0.242 · 분산 0.168.
- **(같은 때, 캠페인 결함 — 사전 등록 규칙과 어긋난 코드)** `p5_campaign.decide` 가 짝 2 의 무효를 도구 결함 점검 문제로 보고 본 실행을 멈췄다
  (`CAMPAIGN_HALT_2_0046.txt`). 사전 등록 규칙은 "짝 2 가 DONE 이 아니면 차례로" 다 — 점검은 짝 1 만 보도록 고치고 시험으로 고정했다.
  해시 `p5_campaign.py` `8e64e76b33df54c938523a0c18d2d1501335e7fac66ba687e957879a29dc20b0`, `unit/test_fleet_p5.py`
  `1b40238ebd52af5a1633269a0e72f4b30927293da1cb75bdcafba917a9fba9b9` (35 통과). 캠페인은 `--skip-pilot` 으로 다시 띄운다.
- **부하에 대한 주의 (불리한 것)**: 본 실행도 같은 공유 서버 부하 아래 돈다. 짝은 차례로 돌므로 짝 안 두 갈래의 부하가 같지 않을 수 있다 —
  갈래마다 시작 · 끝 loadavg 를 남기고, 짝 차와 부하 차의 관계를 결과에서 따로 본다 (판정 규칙은 그대로).
- **(01:10 결과 뒤 변경 — 운영 · 진단, 본 실행 AA 짝 1 갈래 a 에서)** `p5_AA_p1_dist_a` 가 sim 40.58 s 에서 ROS /clock 이 멈춰 CLOCK_STALL
  로 끝났다 [측정]. Gazebo 내부 시각은 계속 갔다 (`ign topic /stats` sim 211.6 s, RTF 0.08). 새 참가자(UDP · SHM 둘 다)로 그래프를 보니 노드
  170 개 남짓 중 128 개만 보였고 **clock_bridge · amr_01 · amr_02 gz_bridge 참가자가 발견되지 않았다** — 브리지 프로세스는 살아 있었다.
  그 전에 lifecycle bond 시간 초과(`SERVER map_server IS DOWN after not receiving a heartbeat for 30000 ms`)가 났다. b4-s12 의 진단(SHM 송신
  경로에서 멈춘 쓰기 스레드, hold_failsafe.md §7.9.1-4)과 같은 계열로 **추정**한다 (이 갈래에서는 gdb 를 받지 못했다). 플릿 모드 효과의 증거가 아니다.
  - 바꾼 것 (지표 · 판정 무관): 기록기 /clock 정지 상한 600 → **180 s**(벽시계), 준비 상한 1800 → **900 s**, lifecycle 가정 900 → **600 s** —
    죽은 갈래에 쓰는 시간을 줄인다 (무효 짝은 규칙대로 새 태그로 다시 돈다). 기록기가 정상 끝이 아니면 그때 보이는 그래프
    (`graph_at_end.json`: /clock 발행자 수 · 노드 이름)를 남긴다. 해시 `p5_recorder.py` `d0ff03fa9492afe7b5b102ddb8b378104b805c58d7ad6e648b10d3b83153eec6`.
  - 전송을 UDP 전용으로 바꾸는 것은 **하지 않았다**: 카메라 영상 조각 전송으로 인식 처리율이 0.32–0.54 로 떨어진다는 기록(프로파일 머리말)이 있고
    파일럿 짝 1 재시도는 SHM 으로 두 갈래 모두 정상이었다. 무효율이 높으면 다시 판단한다 (판단과 근거를 이 항목에 적는다).
- **(01:25 결과 뒤 변경 — 환경: DDS 를 UDP 전용으로, 본 실행 유효 0 회 시점)** AA 짝 1 의 두 갈래가 **모두** ROS /clock 정지(CLOCK_STALL)로
  끝났다 [측정]: 갈래 b 는 sim 5.5 s 에서 기록기의 /clock 이 멎었는데 같은 기록기가 받은 정답 odom(gz_bridge) 머리 시각은 179 s 까지 갔고
  (`gt.csv`), 서버 하트비트(서버 sim 시계 타이머)도 sim 5.5 s 무렵 끊겼다 — **clock_bridge 의 /clock 전달만** 멎었다. Gazebo 내부 RTF 0.33.
  지금까지 SHM 프로파일 갈래 7 개 중 /clock 정지 3 (파일럿 짝 2 분산 · AA 짝 1 a · b), 기동 lifecycle 실패 1, 정상 3. clock_bridge 는 /clock 을
  sim 갱신마다 노드 ~170 개에 보낸다 — SHM 전송은 읽는 쪽 포트가 막히면 쓰는 쪽 송신이 멈출 수 있다 (b4-s12 진단 `SharedMemTransport::send`).
  그래서 **스택 · 서버 컨테이너의 DDS 를 `fastdds_multi_robot_udponly.xml` 로** 바꾼다 (기록기는 처음부터 UDP 전용). 두 갈래 · 모든 실험에
  같다 — 비교의 공정성은 그대로다. 비용 [문서주장, 프로파일 머리말]: 카메라 영상이 UDP 조각 전송이라 인식 처리율 0.32–0.54. 도킹 · 인식 실패가
  늘면 결과에 함께 적는다. `P5_DDS=shm` 으로 예전 프로파일을 다시 쓸 수 있다 (진단용). 드라이버가 스택 `df -k` 를 남겨 shm 사용량(UDP 전용이면
  fastrtps 세그먼트 없음)을 확인한다. 파일럿 결정(차례로 · T_START 60)은 그대로 쓰되, UDP 갈래의 t_ready 가 60 s 를 넘으면(LATE_START)
  P 실험 전에 T_START 를 같은 규칙으로 다시 정한다. 해시 `p5_container.sh` `e8b53c7398e0fd8157a23881e35ad8696d9aea6a3dc47838df24bf6272924faf`,
  `p5_server.sh` `c741c59bb2d4e53fc71bc1eabf4d8b952db4d22623989250ec59ca2eb22ac883`, `p5_driver.py` `7c703ba79b0985977751d82731950e18ae2d4d86cdf84ea70b913d82613ff60b`.

## 결과

(실행 뒤 채운다)

## 해석

(실행 뒤 채운다)

## 다음

(실행 뒤 채운다)
