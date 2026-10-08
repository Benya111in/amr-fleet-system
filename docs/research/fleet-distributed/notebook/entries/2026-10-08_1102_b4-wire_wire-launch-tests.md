# 연결 확인 — 실제 실행기로 ①–④c, 연결 변이 검사, 회귀

- 시각: 2026-10-08 11:02 KST · 에이전트: b4-wire · 단계: W
- 코드: `cb7be26` (브랜치 `feature/b4-wire`)
- 관련 실행 id: W-W1-r{1,2,3}-s{1,2,3}, W-W2-r{1,2,3}-s{1,2,3} (1 회차는 r1-s1–s3 · W2 r1-s1–s2), W-mut · W-mut2 · W-mut3 · W-mut4, W-regress-r1 · W-regress2 · W-regress3, W-dev-* (runs/b4-wire.jsonl). 사전 등록 때 적은 W-W*-s* · W-P3-relaunch 는 이 이름으로 바꿨다 (P3 launch 재실행은 회귀 안의 CTest 와 W-diag 로 대신했다)

## 질문
1. P3 launch 판정 ①–④c 가 **실제 task_executor**(분산 트리 · PickupLocked · TrafficGate 구역 게이트) · **실제 관리자 하트비트**
   (fleet_manager_node + heartbeat) · 에이전트 경로 게이트 · 협상 낙찰 → 실행기 결선을 붙인 뒤에도 성립하는가 (Nav2 · Gazebo 없이,
   가짜 세계 = 직선 운동학 대역)?
2. 새 연결(하트비트 · 관찰자 epoch · 존 노드 등록 · 리매핑 · plan 결선 · PickupLocked · 서버 생존 게이트)과 b4-wire 가드
   (미래 stamp · sim 시계 시간 초과 · 낡은 LOCKED 정리 · 결정점 blocking)를 되돌리면 시험이 잡는가?
3. 중앙 모드 기존 시험은 그대로인가 (회귀)?

## 가설과 예측
판정 규칙 = 시험 단언 그대로 (`src/amr_behavior/test_fleet_world/test_wire_launch.py`, `test_wire_partition.py`, 이 항목과 같은
커밋). 시드 1 · 2 · 3 각각 통과해야 그 판정 "충족" — 한 시드라도 실패하면 그 판정 "미달" 로 적고 원인을 추적한다 (재시도로
덮지 않는다; 같은 실패 세 번이면 진단). 세 시드는 같은 기하 · 같은 작업, 다른 것은 로봇↔로봇 손실 0.2 · 지연 난수뿐이다 —
**표본 3 으로 읽지 않는다** (안전 성질의 예시 실행이지 비율 추정이 아니다).
- W1 ① 실제 자세 원판(0.36 m) 기준 구역 동시 점유 0 · 믿은 허가 구간 겹침 0 (모든 구역) · kill 3 s 뒤 /fleet_p2p 끝점 5/5 ·
  고정 중앙 작업 4/4 COMPLETED. 공허 검사: 결정점 blocking ≥ 1 (경합 있었음), kill 뒤 구역 허가 ≥ 1.
  예측: 충족. 기각: 원판 점유 > 0 이면 구역 안전 결선 결함 (중심 기준과 함께 사건 · 자세를 추적).
- W1 ② 다섯 모드 순서 5/5, 관찰자 epoch > 3 (실제 관리자 하트비트). ③ 낮은 명령 세 사례 거부 · 실행기 로그에 그 작업 0,
  현재 epoch 작업 수락 · 완료, 옛 권위자 하트비트가 살아 있어도 DEGRADED · 거부 수 증가.
- W1 ④b 실행기 busy 거부 T1 → 다른 로봇 완료(적재 그 로봇만, 거부 로봇 central_accept 0), 같은 T2 를 두 로봇에 → 적재 정확히
  한 대 · 다른 한 대 task_lock_failed · task:T2 허가 겹침 0.
- W2 (④a · ④c) 소수 쪽 분할 중 작업 확정 0 · 같은 작업 적재 2 대 이상 0 · 모든 적재는 그 로봇의 그 작업 확정 뒤. 공허 검사:
  소수 쪽 분할 중 확정 요청 ≥ 1, 다수 쪽 분할 중 확정 ≥ 1. 활성(완료 수 · task_lock_failed 수 · 낡은 LOCKED 정리)은 보고만.
- 변이 (`tools/mutate_wire.py` 의 MUTATIONS — 기대: 전부 KILLED): M-HB(하트비트 발행 삭제 → test_manager_heartbeat),
  M-OBS(max+1 삭제 → test_heartbeat), M-REG(존 노드 등록 삭제 → test_task_executor_node), M-REMAP(리매핑 삭제 →
  test_config_launch), M-PL(PickupLocked 삭제 → test_task_tree_distributed), M-SRV(서버 생존 게이트 삭제 → 같은 시험),
  M-FRESH · M-SIMCLK(→ test_zone_bt_nodes), M-GC(→ test_agent_dispatch), M-GATE(blocking 비움 → test_agent_path_gate),
  **M-PLAN**(노드가 plan 을 게이트에 안 넣음 → launch W1 ① 이 잡아야 — ROS 결선의 종단 검사). SURVIVED 는 숨기지 않고 적는다.
- 회귀: `tools/regress.sh` (최종 커밋 git archive, Release -Werror, amr_msgs · amr_fleet · amr_behavior · amr_bringup) 를
  W_baseline(fd17e49)과 `tools/xunit_summary.py compare` — 기준에서 pass 인 사례가 pass 가 아니면 회귀. 예측: 회귀 0,
  빌드 경고 0. (기준 skip 40 = fleetsim 지도 경로 · 67 = cppcheck 는 그대로일 것)

## 설정
- 이미지 amr-ai/rl:0.3, 컨테이너 기본 브리지 · ROS_LOCALHOST_ONLY=1, 도메인 W1 56 · W2 55 · 변이 59 (b4-wire 배정 51–59,
  컨테이너마다 망이 갈라져 겹치지 않는다). cpu_slot.sh. 가짜 /clock rtf 1 (sim = 벽시계 경과 — 공유 부하 하 예비값, loadavg 기록).
- 구성 (harness.py): 실제 task_executor_node 5 (fleet.distributed, 분산 트리, 도크 없는 작업 허용, `-r assign_task:=executor/
  assign_task`), 실제 fleet_agent 5 (proxy · path_gate · zone_map 해제 · traffic_zones.yaml · traffic.yaml), W1 은 실제 어댑터 5 +
  실제 관리자(manager_shim = FleetManagerNode + heartbeat), 가짜 세계 1 (V 1.0 m/s · 취소 뒤 감속 1.0 m/s² · 직선 plan ·
  odometry 20 Hz · 상자 인식), fake_clock 1. 로봇↔로봇 손실 0.2 · U(0, 100) ms (수신 측, 묶음 단위), 서버→로봇 지연만.
- W1 기하: 로봇 (-23, 0.5) · (-15, -9) · (-7, -0.5) · (-15.5, 11.5) · (-23, -7). 고정 작업이 x_bc_1 을 가로 둘 · 세로 둘 (세로는
  x_ab_1 과 사슬)로 두 번씩 지난다. W2: 소수 {amr_01, amr_02} (0, 12 · 14), 다수 (6 · 8 · 10, -15), 작업 6 (y 13 · -15, x 18–24),
  분할 sim [112, 175). (사전 등록 커밋 `0748e72` 의 이 줄은 [125, 190) 로 남은 오기였다 — 같은 커밋의 시험 코드와 아래 개발 실행
  문단은 [112, 175) 이고 판정 실행도 그 코드로 돌았다. 결과 뒤 고친 것은 이 문서 줄뿐이다.)
- 개발 실행 (판정 아님, 원자료 `/home/CAPYI_AI/runs/b24/b4-wire/dev_launch/`): w1a W1 통과 (실행기 로그 기반 "실행기 도달"
  확인이 공허했다 — 프록시 시험 요청에 stamp 가 없어 실행기가 수락 로그를 안 남김 → stamp 를 싣고 task_status 도 함께 보게
  고침), w2a W2 공허 검사 실패 (분할 sim [125, 190) 이면 다수 쪽 확정이 모두 분할 전 — 안전 판정은 통과) → 분할을 [112, 175) 로
  옮김, w2b W2 통과. **이 두 변경은 판정 실행 전이다** (판정 기준 자체는 바꾸지 않았다).
- 판정 실행: `tools/wire_launch_runs.sh` (최종 커밋 git archive · Release -Werror 새 빌드 · W1/W2 × 시드 1–3, 재개 가능).
- **판정 실행 회차 (결과 뒤 변경 — 실행 전에 이 줄을 커밋)**: 1 회차 W-launch(`0748e72`)는 시험 격리 결함으로 중단 (결과 절),
  2 회차 W-launch2(`f42eff0`, harness · 러너 고침 뒤), 코어의 낡은 LOCKED 정리를 고친 뒤(`f618e59`, 노트 `…_1200_…`) 최종 HEAD
  로 **3 회차 W-launch3** · 회귀 W-regress3 · 변이 W-mut4(M-GC · M-STAMP)를 같은 판정 규칙으로 다시 돈다. 최종 판정은 3 회차,
  1 · 2 회차는 그대로 적는다.
- 명령: `colcon test` 대상 amr_behavior_fleet_world (CMake, TIMEOUT 2400, ROS_DOMAIN_ID 56) 또는 시드별
  `AMR_WIRE_SEED=s AMR_WIRE_RAW=<원자료> python3 -m pytest test_wire_launch.py` (원자료 `/home/CAPYI_AI/runs/b24/b4-wire/W-W*/`).

## 결과
[측정] — Gazebo 아님 (운동학 대역 세계 + 실제 ROS 노드). 시간 값은 공유 서버 부하 하 예비값 (load1 열 — 3 회차는 다른 에이전트의
Gazebo 실행으로 load1 96–176 / 32 코어). 원자료 `/home/CAPYI_AI/runs/b24/b4-wire/W-launch{-aborted,2,3}/<시험>_s<시드>/raw/`
(summary.json · events_*.jsonl · exec_*.log · poses.jsonl), meta 에 시작/끝 loadavg · 남은 프로세스 수.

### 결과 뒤 변경 두 건 (불리한 것 먼저)

1. **판정 실행 1 회차 미달 — 원인은 시험 격리** (W-launch, 원자료 `/home/CAPYI_AI/runs/b24/b4-wire/W-launch-aborted/`, 스냅숏
   `0748e72`, 실행 목록 W-W1-r1-s1–3 · W-W2-r1-s1–2). 시드 1 은 W1 · W2 통과, **시드 2 는 W1 · W2 실패**, 시드 3 은 도중에
   멈췄다. 사전 등록 규칙(한 시드라도 실패하면 미달, 원인 추적)대로 1 회차는 미달로 남긴다. 원인 [측정]: harness 가 에이전트를
   `bash -c '<명령> > log'` (exec 없음)로 띄워 launch 종료 신호가 bash 에만 갔고, 에이전트 python 은 고아로 남았다. 러너는 한
   컨테이너에서 시드를 이어 돌리므로 시드 2 W1(도메인 56)에는 시드 1 W1 의 고아 에이전트 5 개가, 시드 2 W2(도메인 55)에는 시드 1 W2 의
   고아 5 개가 같은 이름으로 섞였다. 증거: W1 시드 2 의 kill 3 s 뒤 `/fleet_p2p` 끝점 **10/10** (단언 5/5 에서 실패), W2 시드 2 는 고아의
   DEGRADED 상태를 먼저 보고 시작 6 s 만에 사건 0 으로 실패, 컨테이너 프로세스 목록에 앞 시드 store_dir 을 쓰는 에이전트 30 개
   (`W-launch-aborted/orphans_ps_112257.txt`). 시험 대상 코드(에이전트 · 실행기)는 원인이 아니다 — 운영 런치(behavior.launch.py)는
   `Node` 로 스크립트를 직접 실행한다.
   - 고침 `f42eff0` (결과 뒤 변경): harness 에 exec, **같은 이름 노드가 둘 이상이면 격리 실패로 단언** (W1 · W2 시작 직후),
     `tools/wire_launch_runs.sh` 가 시험 앞뒤 남은 프로세스를 세어 meta 에 적고(strays_before · strays_after) PID 로 죽인다.
   - 고친 것 확인 (개발 실행 W-dev-orphan, 시드 101, 한 컨테이너 · 같은 도메인 연속): W1 · W2 통과, 남은 프로세스 앞 0 · 뒤 0,
     launch 의 SIGTERM 승격 0.
   - 그 뒤 고친 HEAD 로 **세 시드 전부** 다시 돌렸다 (2 회차 W-launch2, 실행 목록 W-W*-r2-s*). 아래 판정은 2 회차다.
   - 1 회차 시드 1 은 오염이 없다고 본다 (W1 은 새 컨테이너의 첫 시험, W2 앞의 고아는 다른 도메인 56 뿐) — 참고 행으로 싣는다.
     1 회차 시드 2 W1 은 끝점 단언 앞에 ① 원자료를 남겼다 (원판 동시 점유 0 · 믿은 겹침 0) — 오염 실행이라 판정에 쓰지 않는다.
2. **변이 재실행 W-mut2 판정 불가 — 원인은 내가 넣은 시험 파일** (원자료 `W-mut2/`, `ed14426`). M-HB · M-OBS 둘 다 INVALID
   (수집 0 — 도구의 INVALID 검사가 걸렀다). 원인 [측정]: `src/amr_fleet/test/test_launch_distributed_bringup.py` 의 모듈 수준
   `pytest.importorskip('amr_bringup.launch_utils')`. 이미지(pytest 8.3.5)의 launch_testing pytest 플러그인 아래에서 이 skip 이
   amr_fleet/test **수집 전체를 멈춘다**. 변이 기준 빌드에 amr_bringup 이 없어 드러났다. 같은 원인으로 **amr_bringup 이 없는
   작업공간에서는 amr_fleet 시험 디렉터리 전체가 '1 skipped' (rc 0) 로 끝난다** — W-dev-nobringup: 고치기 전 1 skipped → 고친 뒤
   772 passed · 5 skipped. 저장소에 같은 교훈 주석이 이미 있었다 (`test_agent_codec.py` · `test_task_msg.py`, "f1 교훈") — 지키지 않았다.
   W_baseline 과 앞선 개발 실행은 amr_bringup 을 함께 빌드해 드러나지 않았다. 고침 `f42eff0` (가드된 import + skipif), W-mut3 에서 다시.
   - W-mut 1 회차의 다른 amr_fleet 변이(M-OBS · M-GC · M-GATE)는 그때 도구가 PYTHONPATH 를 덮어써 ROS pytest 플러그인이 안 실렸고,
     로그에 실제 단언 실패가 있다 (각 1 failed). M-REMAP 은 amr_behavior 시험이라 해당 없다 (1 failed, 4 passed).
3. **코어 변경 뒤 3 회차** (`f618e59` — 회귀 진단 중 찾은 낡은 LOCKED 정리 거짓 양성 고침, 노트 `…_1200_…`): 최종 코드로
   W-launch3 · W-regress3 · W-mut4 를 다시 돌렸다 (판정 규칙 그대로, 실행 전 설정 절에 회차를 적고 `6cb4821` 로 커밋).

### W1 (①②③④b) — 3 회차 W-launch3 (스냅숏 `6cb4821`, 최종 판정)

| 시드 | rc | 실제 점유 원판 · 중심 | 믿은 겹침 | kill 3 s 뒤 끝점 vote/beacon/neg | 중앙 완료 | 결정점 blocking | kill 뒤 구역 허가 | 관찰자 epoch | 낮은 명령 수용 · 실행기 도달 | 옛 권위자 살아도 DEGRADED | T1 적재 | T2 적재 · task_lock_failed | load1 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0 | 0 · 0 | 0 | 5,5 / 5,5 / 5,5 | 4/4 | 9 | 10 | 4 | 0/3 · 0 | 예 | amr_02 | amr_01 · amr_03 | 96 → 118 |
| 2 | 0 | 0 · 0 | 0 | 5,5 / 5,5 / 5,5 | 4/4 | 7 | 10 | 4 | 0/3 · 0 | 예 | amr_02 | amr_01 · amr_03 | 115 → 96 |
| 3 | 0 | 0 · 0 | 0 | 5,5 / 5,5 / 5,5 | 4/4 | 10 | 10 | 4 | 0/3 · 0 | 예 | amr_02 | amr_01 · amr_03 | 153 → 176 |

- ② 다섯 로봇 모두 HOLD_NEW → DEGRADED → RESYNC → NORMAL (세 시드), 관찰자 복귀 관리자(실제 FleetManagerNode + heartbeat)의
  epoch 3 → 4. ③ 옛 권위자(epoch 2) 하트비트 거부 로봇당 14–19 회, 그래도 DEGRADED. ③ 낮은 명령 세 사례(lower_epoch ·
  same_epoch_lower_leader · unfenced_epoch0) 모두 거부 · 실행기 로그에 0, 현재 epoch 작업 수락 · 완료.
- ④b busy 거부(실행기 사유 `busy:w4.busy`) 뒤 T1 은 amr_02 가 적재 · 완료, 거부 로봇 central_accept 0. T2 를 두 프록시에 → 적재 한 대,
  다른 한 대 task_lock_failed, task:T2 허가 겹침 0 (세 시드).
- 구역 허가 (시드 1): x_bc_1 8 · x_bc_2 3 · x_ab_1 4, kill 뒤 10. 같은 이름 노드 0 (격리 확인).
- 2 회차 W-launch2 (`f42eff0`, load1 10–17): 세 시드 모두 통과, 위 열과 같은 값 (blocking 8 · 7 · 7, T2 적재 amr_03 · 실패 amr_01).
- 1 회차 시드 1 (`0748e72`, load1 9): 통과, blocking 9 · T2 적재 amr_03.

### W2 (④a · ④c) — 3 회차 W-launch3

| 시드 | rc | 소수 쪽 분할 중 확정 | 같은 작업 적재 2 대 이상 | 확정 없는 적재 | 소수 쪽 분할 중 확정 요청 | 다수 쪽 분할 중 확정 | 완료 | task_lock_failed | 낡은 LOCKED 정리 | load1 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0 | 0 | 0 | 0 | 60 | 3 | 6/6 | 4 | 0 | 118 → 105 |
| 2 | 0 | 0 | 0 | 0 | 60 | 2 | 6/6 | 4 | 0 | 99 → 147 |
| 3 | 0 | 0 | 0 | 0 | 60 | 3 | 6/6 | 3 | 0 | 166 → 120 |

- task_lock_failed 에 **다수 쪽 로봇**이 있다 (amr_05 w2.s0 세 시드, amr_03 w2.s2 시드 1 · 2) — p3 §8-12 (분할 전 VOTED 의 다수 쪽 확정
  실패)와 같은 활성 비용이 실제 실행기에서도 보인다. 안전 판정과 무관.
- 2 회차: 세 시드 통과 (다수 쪽 분할 중 확정 3 · 3 · 2, task_lock_failed 3 · 4 · 4). 1 회차 시드 1: 통과 (3, 3).

### 변이 — 12 개 모두 KILLED (최종 코드 기준)

| id | 변이 | 판정 · 회차 | 잡은 시험 |
|---|---|---|---|
| M-HB | 관리자 하트비트 발행 삭제 | KILLED (W-mut3; W-mut 은 도구 결함으로 공허 SURVIVED, W-mut2 는 INVALID) | test_manager_heartbeat (관찰자 → ACTIVE 비컨 대기) |
| M-OBS | 관찰자 max+1 삭제 | KILLED (W-mut · W-mut3) | test_heartbeat |
| M-REG | 분산 실행기가 존 노드 등록 안 함 | KILLED (W-mut) | test_task_executor_node |
| M-REMAP | 런치가 assign_task 리매핑 안 함 | KILLED (W-mut) | test_config_launch |
| M-PL | 분산 트리에서 PickupLocked 삭제 | KILLED (W-mut) | test_task_tree_distributed |
| M-SRV | 서버 생존 게이트 삭제 | KILLED (W-mut) | test_task_tree_distributed |
| M-FRESH | 미래 stamp 허용치 삭제 | KILLED (W-mut) | test_zone_bt_nodes |
| M-SIMCLK | 존 노드 시간 초과를 벽시계로 | KILLED (W-mut) | test_zone_bt_nodes |
| M-GC | 낡은 LOCKED 정리 끔 | KILLED (W-mut · W-mut4) | test_agent_dispatch |
| M-STAMP | 정리가 "LOCKED 를 본 뒤 만든 비컨" 을 안 봄 (f618e59 에서 등록) | KILLED (W-mut4) | test_agent_dispatch::test_stale_lock_needs_a_beacon_made_after_the_lock_was_seen |
| M-GATE | 결정점 blocking 비움 | KILLED (W-mut) | test_agent_path_gate |
| **M-PLAN** | 에이전트 노드가 plan 을 경로 게이트에 안 넣음 (ROS 결선) | KILLED (W-mut) | **launch W1 ①** — 실제 자세 원판 동시 점유 6 사건(x_bc_1 에 최대 4 대), 중심 기준 4, 구역 허가 0 |

W-mut 의 C++/트리 변이는 `0748e72` 코드, M-GC · M-STAMP 는 최종 코드(W-mut4). 그 사이 바뀐 C++ · 트리 · 노드 코드는 없다
(`git diff --stat 0748e72 6cb4821 -- src/amr_behavior src/amr_fleet/amr_fleet/agent/node.py` 에 시험 · harness 만).

### 회귀 — W-regress3 대 W_baseline (`tools/xunit_summary.py compare`, 스냅숏 `6cb4821`)
- 빌드 Release `-Werror` 경고 **0** (네 패키지). 새 사례 91 개 (amr_behavior 42 · amr_fleet 49) 모두 통과, 기준 pass 가 pass 아닌 것 3 건 —
  모두 옮김 · 이름 바꿈 (`zone_bt_nodes_test.cpp` → `test/test_zone_bt_nodes.cpp` 의 cpplint · uncrustify, F5 의
  `…_except_docks` → `test_shipped_approach_meets_decision_distance`). 합계: amr_fleet 863/903 (skip 40, 기준과 같은 이유),
  amr_behavior 348/418 (skip 70 = 모두 cppcheck), amr_bringup 101/101, amr_msgs 2/2. CTest 안의 W1 174 s · W2 122 s 통과.
- **그러나 2 회차 W-regress2 (`f42eff0`)에서는 P3 launch 시험 ④c (`test_agent_launch_partition`) 가 pass → fail 이었다** — 공허 검사
  "다수 쪽 분할 중 픽업 ≥ 1" 이 0 (안전 단언은 통과). 진단은 노트 `…_1200_b4-wire_p3-partition-regression-diagnosis.md` (W-diag ·
  W-diag2 · W-diag3) — 요약은 해석 절.

## 해석
- 사전 등록 판정 (최종 코드, 3 회차): W1 ①②③④b · W2 ④a ④c **세 시드 모두 충족**, 변이 12/12 KILLED, 회귀 0 · 경고 0. 실제 실행기
  (분산 트리 · PickupLocked · TrafficGate 구역 게이트) · 실제 관리자 하트비트 · 경로 게이트 · 협상 결선을 붙인 뒤에도 P3 판정이 선다.
- 말하지 않는 것: (1) **Gazebo · Nav2 · AMCL 에서는 아무것도 보지 않았다** — 가짜 세계는 직선 plan · 이상 감속 · 완벽한 자세다. 결정점
  정지가 구역 밖인지, plan 신선도, 실제 도킹은 P5 몫 (p3.md §10.6). (2) 세 시드는 같은 기하 · 같은 작업이고 다른 것은 손실 난수뿐 —
  안전 성질의 예시 실행이지 비율 추정이 아니다. (3) 3 회차는 공유 부하가 매우 높은 때였다 (load1 96–176) — 통과했지만 시간 값은 쓰지
  않는다.
- 1 회차 미달은 시험 격리 결함이었고(같은 이름 고아 에이전트 — 이 결함은 단언 하나가 잡아냈다: 끝점 10/10), 이제 같은 이름 노드
  단언과 러너의 남은 프로세스 계수가 막는다. 같은 종류(같은 이름 에이전트가 둘)가 실제 배치에서 생기면 투표가 섞인다 — 운영
  런치는 노드를 직접 실행해 고아가 남지 않지만, **같은 robot_id 의 에이전트 두 개를 막는 장치는 프로토콜에 없다** (P5 · 운영 몫).
- P3 ④c 공허 검사 실패 (W-regress2): 사전 등록 진단 두 번 모두 "가르지 못함" (W-diag 기준 0/4 · HEAD 1/4, W-diag2 0/8 · 1/8), 정리를
  끈 팔에서도 실패(2/4) — 정리는 원인이 아니다. 실패는 "amr_05 가 분할 시작 전에 첫 작업을 끝내는가" 의 경계 경주로 보인다.
  사후로 모으면 시드 1 에서 기준 0/16 · HEAD 계열 5/17 이고 amr_05 첫 낙찰 지연도 기준 0/12 · HEAD 5/16 이라 b4-wire 가 이 경주의
  확률을 바꿨을 가능성을 배제하지 못했다 — 시드 2–9 로 다시 본 W-diag3 에서 **기준도 실패했다** (시드 8; 기준 1/8 · HEAD 0/8, 가르지
  못함). 기준 코드에도 있는 시험의 경계 경주로 본다 (사전 등록 비교 셋 모아 기준 1/20 · HEAD 2/20). 안전 단언은 진단 44 실행 모두
  지켜졌다. 최종 회귀 W-regress3 에서는 이 시험도 통과했다.

## 다음
- P5 (Gazebo): p3.md §10.6 목록 — 분산 모드 기동 확인(로봇마다 executor/assign_task 와 fleet_agent), F5 배치 짝 비교(사전 등록 요청),
  결정점 정지 실제 거리 · plan 신선도, 협상 할당 + 실제 Nav2, F3 서버 kill.
- P3 ④c 시험의 경계 경주 (분할 시작 시각이 amr_05 의 첫 작업 끝과 겹침) — 시험 설계를 바꾸는 것은 P3 판정 조건 변경이라 소유자 결정.
- 자기 LOCKED 정리의 사건 반복 (W-diag2 D3) — 기억을 두는 것이 후보.
- 같은 robot_id 에이전트 중복 방지 (incarnation 충돌 탐지 등) — 프로토콜 밖, 후보로만 적는다.
