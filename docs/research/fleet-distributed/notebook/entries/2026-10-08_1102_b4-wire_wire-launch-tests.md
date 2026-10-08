# 연결 확인 — 실제 실행기로 ①–④c, 연결 변이 검사, 회귀

- 시각: 2026-10-08 11:02 KST · 에이전트: b4-wire · 단계: W
- 코드: `cb7be26` (브랜치 `feature/b4-wire`)
- 관련 실행 id: W-W1-s{1,2,3}, W-W2-s{1,2,3}, W-mut, W-regress, W-P3-relaunch (runs/b4-wire.jsonl)

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
  분할 sim [125, 190).
- 개발 실행 (판정 아님, 원자료 `/home/CAPYI_AI/runs/b24/b4-wire/dev_launch/`): w1a W1 통과 (실행기 로그 기반 "실행기 도달"
  확인이 공허했다 — 프록시 시험 요청에 stamp 가 없어 실행기가 수락 로그를 안 남김 → stamp 를 싣고 task_status 도 함께 보게
  고침), w2a W2 공허 검사 실패 (분할 sim [125, 190) 이면 다수 쪽 확정이 모두 분할 전 — 안전 판정은 통과) → 분할을 [112, 175) 로
  옮김, w2b W2 통과. **이 두 변경은 판정 실행 전이다** (판정 기준 자체는 바꾸지 않았다).
- 판정 실행: `tools/wire_launch_runs.sh` (최종 커밋 git archive · Release -Werror 새 빌드 · W1/W2 × 시드 1–3, 재개 가능).
- 명령: `colcon test` 대상 amr_behavior_fleet_world (CMake, TIMEOUT 2400, ROS_DOMAIN_ID 56) 또는 시드별
  `AMR_WIRE_SEED=s AMR_WIRE_RAW=<원자료> python3 -m pytest test_wire_launch.py` (원자료 `/home/CAPYI_AI/runs/b24/b4-wire/W-W*/`).

## 결과
(표: 시드별 값, 실패 시드 수. [측정]/[계산]/미측정 표기. 원자료 경로)

## 해석
(결과가 말하는 것과 말하지 않는 것. 공유 부하 하 예비값 여부)

## 다음
(이 결과로 바뀌는 결정, 남은 질문)
