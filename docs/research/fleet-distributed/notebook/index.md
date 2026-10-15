# 실험 노트 색인 (docs/research/fleet-distributed/notebook)

> `python3 scripts/notebook.py --nb <노트 경로> index` 가 만든다 — 손으로 고치지 않는다.

## 노트 항목 (시간순)

| 시각 | 에이전트 | 제목 | 파일 |
|---|---|---|---|
| 2026-10-06 22:07 | b4-p3 | P3 분산 플릿 ROS 통합 — 에이전트 5대 launch 시험 ①–④ 사전 등록 | [entries/2026-10-06_2207_b4-p3_ros-integration-launch-tests.md](entries/2026-10-06_2207_b4-p3_ros-integration-launch-tests.md) |
| 2026-10-06 22:12 | b4-p4 | P4-1 T1 운동학 플릿-인-더-루프 시뮬레이터 설계와 결정성·불변식 시험 | [entries/2026-10-06_2212_b4-p4_fleetsim-build.md](entries/2026-10-06_2212_b4-p4_fleetsim-build.md) |
| 2026-10-06 22:31 | b4-p4 | P4-2 작업 도착률 보정 (보정 시드 9001–9010, 평가 시드와 분리) | [entries/2026-10-06_2231_b4-p4_load-calibration.md](entries/2026-10-06_2231_b4-p4_load-calibration.md) |
| 2026-10-06 22:46 | b4-p4 | P4-3 A/A 산포: 같은 외생 입력, 다른 미시 난수 — 잡음 바닥 | [entries/2026-10-06_2246_b4-p4_aa-noise-floor.md](entries/2026-10-06_2246_b4-p4_aa-noise-floor.md) |
| 2026-10-06 22:46 | b4-p4 | P4-4 본 스윕 사전 등록: 9 비교군 × 손실 4 × 고장 4 × 쌍 30 | [entries/2026-10-06_2246_b4-p4_main-sweep-prereg.md](entries/2026-10-06_2246_b4-p4_main-sweep-prereg.md) |
| 2026-10-06 22:53 | b4-p3 | P3 변이 검사 — 에이전트 가드를 되돌리면 시험이 잡는가 (사전 등록) | [entries/2026-10-06_2253_b4-p3_mutation-guards.md](entries/2026-10-06_2253_b4-p3_mutation-guards.md) |
| 2026-10-06 23:07 | b4-p3 | P3 loopback 스윕 — 손실·버스트·분할에서 픽업 중복 0 과 잠금 없는 대조군 (사전 등록) | [entries/2026-10-06_2307_b4-p3_loopback-sweep-task-lock.md](entries/2026-10-06_2307_b4-p3_loopback-sweep-task-lock.md) |
| 2026-10-06 23:48 | b4-p3 | P3 리뷰 반영 — 중앙 프록시 작업 상태 누수와 launch ④ 경합 단계 | [entries/2026-10-06_2348_b4-p3_review-fix-proxy-leak-contention.md](entries/2026-10-06_2348_b4-p3_review-fix-proxy-leak-contention.md) |
| 2026-10-07 00:10 | b4-p4 | P4-5 리뷰 반영: 현행 latched hold 비교군 스윕과 작업 잠금 정족수 변이 검사 (사전 등록) | [entries/2026-10-07_0010_b4-p4_review-hold-quorum.md](entries/2026-10-07_0010_b4-p4_review-hold-quorum.md) |
| 2026-10-08 10:10 | b4-s12 | F4 — P0 hold 수정(DDS 늦은 참여자 대응)의 Gazebo 시나리오 12 짝 실행: 사전 등록 | [entries/2026-10-08_1010_b4-s12_s12-pairs-prereg.md](entries/2026-10-08_1010_b4-s12_s12-pairs-prereg.md) |
| 2026-10-08 10:21 | b4-wire | F5 도크 배치 변경 — 운동학 모의로 결정점 미달 침범이 사라지는지 | [entries/2026-10-08_1021_b4-wire_f5-dock-layout-fleetsim.md](entries/2026-10-08_1021_b4-wire_f5-dock-layout-fleetsim.md) |
| 2026-10-08 11:02 | b4-wire | 연결 확인 — 실제 실행기로 ①–④c, 연결 변이 검사, 회귀 | [entries/2026-10-08_1102_b4-wire_wire-launch-tests.md](entries/2026-10-08_1102_b4-wire_wire-launch-tests.md) |
| 2026-10-08 12:00 | b4-wire | 회귀 진단 — P3 ④c 공허 검사 실패 (W-regress2) | [entries/2026-10-08_1200_b4-wire_p3-partition-regression-diagnosis.md](entries/2026-10-08_1200_b4-wire_p3-partition-regression-diagnosis.md) |
| 2026-10-08 15:40 | b4-p5 | P5 Gazebo 5 대 확증 (F1·F2·F3·F5) — 사전 등록 | [entries/2026-10-08_1540_b4-p5_gazebo-confirm-prereg.md](entries/2026-10-08_1540_b4-p5_gazebo-confirm-prereg.md) |

## 실행 목록 요약 (211 건, 원본 `runs/<에이전트>.jsonl`)

| run_id | 단계 | 종류 | 시뮬 | 커밋 | 시드 | 핵심 결과 | 표기 |
|---|---|---|---|---|---|---|---|
| P3_bt1_gtest_first | P3 | test | offline | c23c511+미커밋 zone_* (6436bed 직전) | 0 | n=1, gtests=8, failed=1, failed_test=ZoneGatesSubtree... RELEASE 비동기 확인 경합 (시험 결함) | 실패 |
| P3_bt1_gtest_rerun | P3 | test | offline | 6436bed 와 같은 코드 (커밋 직전) | 0 | n=3, gtests_per_run=8, passed_runs=3, build_warnings=0 | [측정] |
| P3_colcon1_fleet | P3 | test | offline | 9467c0e | 1 | n=1, amr_fleet_tests=769, failures=0, errors=0 | [측정] |
| P3_launch_dev1 | P3 | test | offline | c2a9f9e+미커밋 | 1 | n=1, passed=False, ①_overlaps=0, ①_intervals=27 | 실패 |
| P3_launch_dev2 | P3 | test | offline | c2a9f9e+미커밋 (9467c0e 와 같은 코드) | 1 | n=1, passed=True, ④_pickups=10, ④_dups=0 | [측정] |
| P3_launch_s1 | P3 | test | offline | 6436bed | 1 | pytest_rc=0, n=1, ①_overlaps=0, ①_intervals=28 | [측정] |
| P3_launch_s2 | P3 | test | offline | 6436bed | 2 | pytest_rc=0, n=1, ①_overlaps=0, ①_intervals=28 | [측정] |
| P3_smoke1 | P3 | probe | offline | c2a9f9e+미커밋 node/shim | 1 | n=1, modes_ok_all5=True, observer_epoch=5, expected_observer_epoch=2 | [측정] |
| P3_launch_s3 | P3 | test | offline | a014f23 | 3 | pytest_rc=0, n=1, ①_overlaps=0, ①_intervals=31 | [측정] |
| P3_mutation_v1 | P3 | test | offline | 6436bedfba95451290ac59e9bd57a1996e4cf95e (변이마다 실행 시점 작업 트리를 복사 — 실행 중 7db1beb 의 core 변경이 들어감) | 0 | n=26, killed=21, survived=['MA1', 'ML4', 'ME1', 'ME2', 'MC4'], file=mutation_v1.json | [측정] |
| P3_sweep_tasklock | P3 | eval | offline | 5e6ac59 | [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30] | n_runs=300, n_per_cell_arm=30, loss0.0/lock={'pickups': 240, 'dup_tasks': 0, 'runs_with_dup': 0, 'done': 240, 'last_done_median_s': 24.6}, loss0.0/optimistic={'pickups': 240, 'dup_tasks': 0, 'runs_with_dup': 0, 'done': 240, 'last_done_median_s': 23.8} | [측정] |
| P3_mutation_v2 | P3 | test | offline | 5e6ac59 | 0 | n=26, killed=26, survived=[], file=mutation_v2.json | [측정] |
| P3_mutation_ros | P3 | test | offline | 6471507 | 1 | n=5, killed=5, by_id={'C1': 'KILLED', 'C2': 'KILLED', 'C3': 'KILLED', 'C4': 'KILLED', 'N1': 'KILLED'}, N1_killed_at=③ 낮은 epoch 3 건이 가짜 실행기에 도달 | [측정] |
| P3_final_regression | P3 | test | offline | cb65301 | 1 | n=1, build_rc=0, build_warnings=0, amr_fleet_xunit_total={'tests': 780, 'failures': 0, 'errors': 0, 'skipped': 1} | [측정] |
| P3R_probe_review_after | P3 | probe | offline | 6b3d51f + 미커밋 core·node·test (6d084b4 로 커밋한 것과 같은 코드) | 0 | n=2, p1_amr_03_reassign=ok, p1_amr_03_held_wait_s=0.4, p1_amr_02_requests=0 | [측정] |
| P3R_dev_main_s1 | P3 | test | offline | 6d084b4 (작업 트리: 1a3fcbd 와 같은 코드 · 커밋 전) | 1 | pytest_rc=0, n=1, test=test/test_agent_launch.py, ①_overlaps=0 | [측정] 개발 실행 (판정 아님 — 판정 실행 전 동작 확인) |
| P3R_dev_part_s1 | P3 | test | offline | 6d084b4 (작업 트리: 1a3fcbd 와 같은 코드 · 커밋 전) | 1 | pytest_rc=0, n=1, test=test/test_agent_launch_partition.py, ④c_minority_pickups_in_partition=0 | [측정] 개발 실행 (판정 아님 — 판정 실행 전 동작 확인) |
| P3R_launch_s1 | P3 | test | offline | 36f190c | 1 | pytest_rc=0, n=1, test=test/test_agent_launch.py, ①_overlaps=0 | [측정] |
| P3R_launch_s2 | P3 | test | offline | 36f190c | 2 | pytest_rc=0, n=1, test=test/test_agent_launch.py, ①_overlaps=0 | [측정] |
| P3R_launch_s3 | P3 | test | offline | 36f190c | 3 | pytest_rc=0, n=1, test=test/test_agent_launch.py, ①_overlaps=0 | [측정] |
| P3R_part_s1 | P3 | test | offline | 36f190c | 1 | pytest_rc=0, n=1, test=test/test_agent_launch_partition.py, ④c_minority_pickups_in_partition=0 | [측정] |
| P3R_part_s2 | P3 | test | offline | 36f190c | 2 | pytest_rc=0, n=1, test=test/test_agent_launch_partition.py, ④c_minority_pickups_in_partition=0 | [측정] |
| P3R_part_s3 | P3 | test | offline | 36f190c | 3 | pytest_rc=0, n=1, test=test/test_agent_launch_partition.py, ④c_minority_pickups_in_partition=0 | [측정] |
| P3R_mutation_v3 | P3 | test | offline | 36f190c | 0 | n=37, killed=37, survived=[], bad_mutant=[] | [측정] |
| P3R_sweep_tasklock_v2 | P3 | eval | offline | 36f190c | [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30] | n_runs=300, n_per_cell_arm=30, new={'loss0.0/lock': {'pickups': 240, 'dup': 0, 'runs_dup': 0, 'done': 240, 'failed': 0, 'med': 24.6, 'msgs': 16585, 'dup_seeds': []}, 'loss0.0/optimistic': {'pickups': 240, 'dup': 0, 'runs_dup': 0, 'done': 240, 'failed': 0, 'med': 23.8, 'msgs': 16584, 'dup_seeds': []}, 'loss0.2/lock': {'pickups': 240, 'dup': 0, 'runs_dup': 0, 'done': 240, 'failed': 0, 'med': 25.9, 'msgs': 16577, 'dup_seeds': []}, 'loss0.2/optimistic': {'pickups': 240, 'dup': 0, 'runs_dup': 0, 'done': 240, 'failed': 0, 'med': 24.5, 'msgs': 16572, 'dup_seeds': []}, 'loss0.4/lock': {'pickups': 240, 'dup': 0, 'runs_dup': 0, 'done': 240, 'failed': 0, 'med': 29.5, 'msgs': 16619, 'dup_seeds': []}, 'loss0.4/optimistic': {'pickups': 242, 'dup': 2, 'runs_dup': 2, 'done': 240, 'failed': 0, 'med': 25.6, 'msgs': 16600, 'dup_seeds': [14, 16]}, 'burst/lock': {'pickups': 240, 'dup': 0, 'runs_dup': 0, 'done': 240, 'failed': 0, 'med': 26.15, 'msgs': 16578, 'dup_seeds': []}, 'burst/optimistic': {'pickups': 250, 'dup': 10, 'runs_dup': 10, 'done': 240, 'failed': 0, 'med': 28.25, 'msgs': 16571, 'dup_seeds': [1, 7, 8, 10, 11, 13, 19, 24, 25, 26]}, 'partition/lock': {'pickups': 240, 'dup': 0, 'runs_dup': 0, 'done': 240, 'failed': 0, 'med': 32.85, 'msgs': 16016, 'dup_seeds': []}, 'partition/optimistic': {'pickups': 296, 'dup': 56, 'runs_dup': 29, 'done': 240, 'failed': 0, 'med': 31.65, 'msgs': 16009, 'dup_seeds': [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29]}}, old_same_cells=compare_old_new.json | [측정] |
| P3R_mut_ros_try1 | P3 | test | offline | 36f190c | 1 | n=4, pytest_rc=[4, 4, 4, 4], cause=IndexError parents[3] at /mut/test/test_nego_cost.py (collection), fix=사본을 /ros2_ws/src/amr_fleet 자리에 겹쳐 마운트 (launch_run.sh) | 실패 (판정 불가) — 변이 사본을 /mut 에 마운트해 test_nego_cost 의 parents[3] 가 IndexError, pytest 수집 오류 rc 4 (4 회 모두) |
| P3R_probe_enter_partition | P3 | probe | offline | 36f190c | [1, 2, 3, 4, 5] | n=5, finding=다수 쪽 로봇의 commit_timeout 은 VOTED 표 집합에 다수 쪽 투표자가 3 미만일 때만 (ENTER 는 표를 준 투표자에게만, 시도 기한 60 s > commit_timeout 25 s), majority_commit_timeouts_by_seed={'1': 0, '2': 2, '3': 1, '4': 1, '5': 2} | [측정] |
| P3R_mut_RM1_part | P3 | test | offline | 36f190c | 1 | pytest_rc=1, n=1, test=test/test_agent_launch_partition.py, ④c_minority_pickups_in_partition=2 | [측정] 변이 검사 |
| P3R_mut_RM2_part | P3 | test | offline | 36f190c | 1 | pytest_rc=0, n=1, test=test/test_agent_launch_partition.py, ④c_minority_pickups_in_partition=0 | [측정] 변이 검사 |
| P3R_mut_N2_main | P3 | test | offline | 36f190c | 1 | pytest_rc=1, n=1, test=test/test_agent_launch.py, ①_overlaps=0 | [측정] 변이 검사 |
| P3R_mut_RM2_main | P3 | test | offline | 36f190c | 1 | pytest_rc=0, n=1, test=test/test_agent_launch.py, ①_overlaps=0 | [측정] 변이 검사 |
| P3R_final_regression | P3 | test | offline | 36f190c | 1 | n=1, build_rc=0, build_warnings=0, build_stderr_lines=0 | [측정] |
| P4-calib-01-r30 | P4 | eval | offline | 8875cf2 | 9001-9010 (n=10), reps=[0] | n_runs=20, n_err=0, note=무효: 모형이 하역 뒤 로봇을 도크 구역 안에 세워 둠 (854cbbb 에서 수정). 도착률 1/30 보정, cooccupancy_events_total=0 | 실패 |
| P4-calib-01-r40 | P4 | eval | offline | 8875cf2 | 9001-9010 (n=10), reps=[0] | n_runs=20, n_err=0, note=무효: 모형이 하역 뒤 로봇을 도크 구역 안에 세워 둠 (854cbbb 에서 수정). 도착률 1/40 보정, cooccupancy_events_total=0 | 실패 |
| P4-calib-01-r50 | P4 | eval | offline | 8875cf2 | 9001-9010 (n=10), reps=[0] | n_runs=20, n_err=0, note=무효: 모형이 하역 뒤 로봇을 도크 구역 안에 세워 둠 (854cbbb 에서 수정). 도착률 1/50 보정, cooccupancy_events_total=0 | 실패 |
| P4-calib-01-r60 | P4 | eval | offline | 8875cf2 | 9001-9010 (n=10), reps=[0] | n_runs=20, n_err=0, note=무효: 모형이 하역 뒤 로봇을 도크 구역 안에 세워 둠 (854cbbb 에서 수정). 도착률 1/60 보정, cooccupancy_events_total=0 | 실패 |
| P4-calib-02-r45 | P4 | eval | offline | dbb9cc5 | 9001-9010 (n=10), reps=[0] | n_runs=20, n_err=0, note=부하 보정 λ=1/45, A1·A6, F0, 손실 0.05, 보정 시드. 판정 λ=1/45 (노트 P4-2), cooccupancy_events_total=0 | [측정] |
| P4-calib-02-r55 | P4 | eval | offline | dbb9cc5 | 9001-9010 (n=10), reps=[0] | n_runs=20, n_err=0, note=부하 보정 λ=1/55, A1·A6, F0, 손실 0.05, 보정 시드. 판정 λ=1/45 (노트 P4-2), cooccupancy_events_total=0 | [측정] |
| P4-calib-02-r65 | P4 | eval | offline | dbb9cc5 | 9001-9010 (n=10), reps=[0] | n_runs=20, n_err=0, note=부하 보정 λ=1/65, A1·A6, F0, 손실 0.05, 보정 시드. 판정 λ=1/45 (노트 P4-2), cooccupancy_events_total=0 | [측정] |
| P4-calib-02-r80 | P4 | eval | offline | dbb9cc5 | 9001-9010 (n=10), reps=[0] | n_runs=20, n_err=0, note=부하 보정 λ=1/80, A1·A6, F0, 손실 0.05, 보정 시드. 판정 λ=1/45 (노트 P4-2), cooccupancy_events_total=0 | [측정] |
| P4-test-01 | P4 | test | offline | 1e2d5fc | 시험 안 고정 시드 (101, 5–7, 11, 102–103 등) | passed=33, failed=0, n=33, note=dirty = 미커밋 p4.md 초안만 (코드 변경 없음) | [측정] |
| P4-aa-01 | P4 | eval | offline | e34fa90 | 1-30 (n=30), reps=[0, 1] | n_runs=720, n_err=0, note=A/A 산포: A1·A6·A9 × (0.05,F0)(0.2,F0)(0.2,F1)(+0.05,F1) × 시드 1-30 × rep 0,1. 표 report/aa.md, cooccupancy_events_total=0 | [측정] |
| P4-main-01 | P4 | eval | offline | 1e2d5fc | 1-30 (n=30), reps=[0] | n_runs=4320, n_err=0, note=본 스윕 9 비교군 × 손실 4 × 고장 4 × 시드 1-30 (rep 0). 표 report/main.md, cooccupancy_events_total=0 | [측정] |
| P4-main-01x | P4 | eval | offline | 5167430 | 1-30 (n=30), reps=[0] | n_runs=120, n_err=0, note=진단 X0 교통 무시(유령) F0 × 손실 4 × 시드 1-30 — 오라클 양성 대조, 혼잡 비용 상한, cooccupancy_events_total=1286 | [측정] |
| P4-mut-01 | P4 | test | offline | 1e2d5fc | 시험 고정 시드 | caught=5, survived=['M4_central_lost_releases_all', 'M5_load_without_lock', 'M6_lock_adapter_always_grants'], n=8 | [측정] |
| P4-mut-02 | P4 | test | offline | 5167430 | 시험 고정 시드 | caught=6, survived=['M5_load_without_lock', 'M6_lock_adapter_always_grants'], double_M5_M6=caught (aggressive 시험 실패), n=9 | [측정] |
| P4-probe-f6 | P4 | probe | offline | 5167430 | 1-30 | victim_in_zone=A2 6/30, A4 8/30, held_outside_body=0/30 둘 다, diff_when_status_differs=-2.83 (6 쌍), diff_when_same=-0.38 (24 쌍) | [측정] |
| P4-test-02 | P4 | test | offline | 5167430 | 시험 안 고정 시드 | passed=37, failed=0, n=37, flake8=0 errors | [측정] |
| P4-test-03 | P4 | test | offline | 1300540 | 시험 안 고정 시드 | passed=48, failed=0, n=48, flake8=0 | [측정] |
| P4-hold-01 | P4 | eval | offline | 1300540 | 1-30 | n=1440, n_err=0, P1h_A9_minus_A1h_loss0.2_F1_completed=+15.4 [14.0, 16.7], d_z 4.25, 30/0/0, 우월, A1h_minus_A1_F1_completed=+2.4~+2.5 (CI 하한 ≥ 2.0, 29/0/1, 4 손실) | [측정] |
| P4-hold-01r | P4 | eval | offline | 1300540 | 1-30 | n=180, same_digest_and_metrics=180, diff=0 | [측정] |
| P4-mut-03 | P4 | test | offline | 1300540 | 시험 고정 시드 | n=7, NONE=32 통과 (대조), caught=['X4_task_lock_self_quorum (정족수 시험 3)', 'X3_fallback_on_own_suspicion (F5 무전환 1)', 'H1_hold_gate_off (정지 기제 1)', 'H2_server_never_holds (결정점 충족 구역 2)', 'H3_hold_fail_closed_on_server_death (fail-open 2)', 'H4_hold_ignores_seq (seq 1)'], survived=[] | [측정] |
| P4-probe-hold | P4 | probe | offline | 1300540 | 1-30 | n=240, digest_same_as_P4-hold-01=240/240, F0_loss0=6 사건 · 모두 도크 · 침범 로봇 hold=true(늦은 hold) · 깊이 중앙값 0.11 최대 0.18 m, F1_loss0=고장 뒤 18 사건(12 실행) · 침범 로봇 hold=false(관리 없음) · 깊이 중앙값 0.40(완전 진입) | [측정] |
| P5_p5_pilot_p1_central | P5-pilot | gazebo | gazebo | c06f6b7 | 1 | completed=5, n_tasks=5, makespan=283.48, mean_task_s=244.15 | pilot/central |
| P5_p5_pilot_p1_distributed | P5-pilot | gazebo | gazebo | 0e08323 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | pilot/distributed |
| P5_p5_pilot_p1_centralr2 | P5-pilot | gazebo | gazebo | ba01b3b | 1 | completed=5, n_tasks=5, makespan=552.12, mean_task_s=305.87 | pilot/central |
| P5_p5_pilot_p1_distributedr2 | P5-pilot | gazebo | gazebo | ba01b3b | 1 | completed=5, n_tasks=5, makespan=294.95, mean_task_s=236.14 | pilot/distributed |
| P5_p5_pilot_p2_central | P5-pilot | gazebo | gazebo | e70ef2c | 2 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | pilot/central |
| P5_p5_pilot_p2_distributed | P5-pilot | gazebo | gazebo | e70ef2c | 2 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | pilot/distributed |
| P5_p5_AA_p1_dist_a | P5-AA | gazebo | gazebo | 9592fa4 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p1_dist_b | P5-AA | gazebo | gazebo | 9592fa4 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_AA_p1_dist_ar2 | P5-AA | gazebo | gazebo | 6053789 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p1_dist_br2 | P5-AA | gazebo | gazebo | 6053789 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_AA_p1_dist_ar3 | P5-AA | gazebo | gazebo | ba62d9c | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p1_dist_br3 | P5-AA | gazebo | gazebo | ? | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_AA_p1_dist_ar4 | P5-AA | gazebo | gazebo | ba62d9c | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p1_dist_br4 | P5-AA | gazebo | gazebo | ? | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_AA_p1_dist_ar5 | P5-AA | gazebo | gazebo | 63852ea | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p1_dist_br5 | P5-AA | gazebo | gazebo | ? | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_AA_p1_dist_ar6 | P5-AA | gazebo | gazebo | b28c3be | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p1_dist_br6 | P5-AA | gazebo | gazebo | ? | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_AA_p1_dist_ar7 | P5-AA | gazebo | gazebo | b28c3be | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p1_dist_br7 | P5-AA | gazebo | gazebo | 7fbb739 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_AA_p1_dist_ar8 | P5-AA | gazebo | gazebo | 7fbb739 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p1_dist_br8 | P5-AA | gazebo | gazebo | ? | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_AA_p1_dist_ar9 | P5-AA | gazebo | gazebo | 7fbb739 | 1 | completed=5, n_tasks=5, makespan=482.93, mean_task_s=308.59 | AA/dist_a |
| P5_p5_AA_p1_dist_br9 | P5-AA | gazebo | gazebo | 7fbb739 | 1 | completed=5, n_tasks=5, makespan=337.29, mean_task_s=262.6 | AA/dist_b |
| P5_p5_AA_p2_dist_a | P5-AA | gazebo | gazebo | 7fbb739 | 2 | completed=5, n_tasks=5, makespan=293.5, mean_task_s=244.4 | AA/dist_a |
| P5_p5_AA_p2_dist_b | P5-AA | gazebo | gazebo | 7fbb739 | 2 | completed=4, n_tasks=5, makespan=280.73, mean_task_s=236.83 | AA/dist_b |
| P5_p5_AA_p3_dist_a | P5-AA | gazebo | gazebo | 7fbb739 | 3 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_a |
| P5_p5_AA_p3_dist_b | P5-AA | gazebo | gazebo | ? | 3 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | AA/dist_b |
| P5_p5_C2_p1_central | P5-C2 | gazebo | gazebo | ea434cb | 1 | completed=5, n_tasks=8, makespan=544.25, mean_task_s=323.78 | C2/central |
| P5_p5_C2_p1_distributed | P5-C2 | gazebo | gazebo | ea434cb | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p1_centralr2 | P5-C2 | gazebo | gazebo | ea434cb | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/central |
| P5_p5_C2_p1_distributedr2 | P5-C2 | gazebo | gazebo | ? | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p1_centralr3 | P5-C2 | gazebo | gazebo | ac1c47c | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/central |
| P5_p5_C2_p1_centralr4 | P5-C2 | gazebo | gazebo | ac1c47c | 1 | completed=5, n_tasks=8, makespan=539.57, mean_task_s=294.88 | C2/central |
| P5_p5_C2_p1_distributedr3 | P5-C2 | gazebo | gazebo | ac1c47c | 1 | completed=7, n_tasks=8, makespan=592.23, mean_task_s=364.74 | C2/distributed |
| P5_p5_C2_p2_distributed | P5-C2 | gazebo | gazebo | ac1c47c | 2 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p2_central | P5-C2 | gazebo | gazebo | ? | 2 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/central |
| P5_p5_C2_p1_centralr5 | P5-C2 | gazebo | gazebo | 3e0fc81 | 1 | completed=5, n_tasks=8, makespan=535.66, mean_task_s=360.73 | C2/central |
| P5_p5_C2_p1_distributedr4 | P5-C2 | gazebo | gazebo | 3e0fc81 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p1_distributedr5 | P5-C2 | gazebo | gazebo | b696804 | 1 | completed=7, n_tasks=8, makespan=522.9, mean_task_s=337.99 | C2/distributed |
| P5_p5_C2_p2_distributedr2 | P5-C2 | gazebo | gazebo | b696804 | 2 | completed=7, n_tasks=8, makespan=465.5, mean_task_s=295.83 | C2/distributed |
| P5_p5_C2_p2_centralr2 | P5-C2 | gazebo | gazebo | b696804 | 2 | completed=5, n_tasks=8, makespan=297.92, mean_task_s=237.77 | C2/central |
| P5_p5_C2_p3_central | P5-C2 | gazebo | gazebo | b696804 | 3 | completed=5, n_tasks=8, makespan=494.78, mean_task_s=323.68 | C2/central |
| P5_p5_C2_p3_distributed | P5-C2 | gazebo | gazebo | b696804 | 3 | completed=7, n_tasks=8, makespan=486.2, mean_task_s=297.76 | C2/distributed |
| P5_p5_C2_p4_distributed | P5-C2 | gazebo | gazebo | b696804 | 4 | completed=7, n_tasks=8, makespan=529.24, mean_task_s=290.97 | C2/distributed |
| P5_p5_C2_p4_central | P5-C2 | gazebo | gazebo | b696804 | 4 | completed=5, n_tasks=8, makespan=311.6, mean_task_s=245.43 | C2/central |
| P5_p5_C2_p5_central | P5-C2 | gazebo | gazebo | b696804 | 5 | completed=4, n_tasks=8, makespan=264.14, mean_task_s=233.9 | C2/central |
| P5_p5_C2_p5_distributed | P5-C2 | gazebo | gazebo | b696804 | 5 | completed=8, n_tasks=8, makespan=539.9, mean_task_s=336.46 | C2/distributed |
| P5_p5_C2_p6_distributed | P5-C2 | gazebo | gazebo | b696804 | 6 | completed=6, n_tasks=8, makespan=480.88, mean_task_s=292.56 | C2/distributed |
| P5_p5_C2_p6_central | P5-C2 | gazebo | gazebo | b696804 | 6 | completed=5, n_tasks=8, makespan=352.47, mean_task_s=269.73 | C2/central |
| P5_p5_C2_p7_central | P5-C2 | gazebo | gazebo | b696804 | 7 | completed=5, n_tasks=8, makespan=303.2, mean_task_s=243.42 | C2/central |
| P5_p5_C2_p7_distributed | P5-C2 | gazebo | gazebo | b696804 | 7 | completed=6, n_tasks=8, makespan=465.0, mean_task_s=269.49 | C2/distributed |
| P5_p5_C2_p8_distributed | P5-C2 | gazebo | gazebo | b696804 | 8 | completed=8, n_tasks=8, makespan=593.79, mean_task_s=364.36 | C2/distributed |
| P5_p5_C2_p8_central | P5-C2 | gazebo | gazebo | b696804 | 8 | completed=4, n_tasks=8, makespan=541.64, mean_task_s=300.38 | C2/central |
| P5_p5_P_p1_dist_p_a | P5-P | gazebo | gazebo | b696804 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | P/dist_p_a |
| P5_p5_C2_p1_distributedr6 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p1_distributedr7 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 1 | completed=7, n_tasks=8, makespan=523.71, mean_task_s=293.85 | C2/distributed |
| P5_p5_C2_p2_distributedr3 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 2 | completed=8, n_tasks=8, makespan=558.89, mean_task_s=323.95 | C2/distributed |
| P5_p5_C2_p3_distributedr2 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 3 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p3_distributedr3 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 3 | completed=7, n_tasks=8, makespan=525.5, mean_task_s=308.33 | C2/distributed |
| P5_p5_C2_p4_distributedr2 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 4 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p4_distributedr3 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 4 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p4_distributedr4 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 4 | completed=8, n_tasks=8, makespan=703.94, mean_task_s=351.05 | C2/distributed |
| P5_p5_C2_p5_distributedr2 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 5 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p5_distributedr3 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 5 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C2/distributed |
| P5_p5_C2_p5_distributedr4 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 5 | completed=7, n_tasks=8, makespan=510.79, mean_task_s=296.27 | C2/distributed |
| P5_p5_C2_p6_distributedr2 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 6 | completed=7, n_tasks=8, makespan=621.07, mean_task_s=361.93 | C2/distributed |
| P5_p5_C2_p7_distributedr2 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 7 | completed=7, n_tasks=8, makespan=508.64, mean_task_s=286.04 | C2/distributed |
| P5_p5_C2_p8_distributedr2 | P5-C2 | gazebo | gazebo | 9c5f2d6 | 8 | completed=6, n_tasks=8, makespan=514.84, mean_task_s=301.37 | C2/distributed |
| P5_p5_P_p1_dist_p_ar2 | P5-P | gazebo | gazebo | 9c5f2d6 | 1 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | P/dist_p_a |
| P5_p5_P_p1_dist_p_ar3 | P5-P | gazebo | gazebo | 9c5f2d6 | 1 | completed=8, n_tasks=8, makespan=553.98, mean_task_s=284.6 | P/dist_p_a |
| P5_p5_P_p1_dist_p_b | P5-P | gazebo | gazebo | 9c5f2d6 | 1 | completed=8, n_tasks=8, makespan=592.5, mean_task_s=323.21 | P/dist_p_b |
| P5_p5_S_p1_dist_s_a | P5-S | gazebo | gazebo | 9c5f2d6 | 1 | completed=8, n_tasks=8, makespan=632.11, mean_task_s=300.55 | S/dist_s_a |
| P5_p5_S_p1_dist_s_b | P5-S | gazebo | gazebo | 9c5f2d6 | 1 | completed=8, n_tasks=8, makespan=602.52, mean_task_s=311.52 | S/dist_s_b |
| P5_p5_X_p1_dist_x_a | P5-X | gazebo | gazebo | 9c5f2d6 | 1 | completed=5, n_tasks=5, makespan=323.74, mean_task_s=277.52 | X/dist_x_a |
| P5_p5_X_p1_dist_x_b | P5-X | gazebo | gazebo | 9c5f2d6 | 1 | completed=5, n_tasks=5, makespan=312.0, mean_task_s=260.47 | X/dist_x_b |
| P5_p5_C3_p1_dist_dead | P5-C3 | gazebo | gazebo | 9c5f2d6 | 1 | completed=5, n_tasks=5, makespan=298.09, mean_task_s=248.49 | C3/dist_dead |
| P5_p5_C3_p1_dist_alive | P5-C3 | gazebo | gazebo | 9c5f2d6 | 1 | completed=5, n_tasks=5, makespan=287.14, mean_task_s=245.79 | C3/dist_alive |
| P5_p5_C3_p2_dist_alive | P5-C3 | gazebo | gazebo | 9c5f2d6 | 2 | completed=5, n_tasks=5, makespan=582.96, mean_task_s=314.24 | C3/dist_alive |
| P5_p5_C3_p2_dist_dead | P5-C3 | gazebo | gazebo | 9c5f2d6 | 2 | completed=5, n_tasks=5, makespan=387.65, mean_task_s=294.94 | C3/dist_dead |
| P5_p5_C3_p3_dist_dead | P5-C3 | gazebo | gazebo | 9c5f2d6 | 3 | completed=5, n_tasks=5, makespan=310.06, mean_task_s=251.27 | C3/dist_dead |
| P5_p5_C3_p3_dist_alive | P5-C3 | gazebo | gazebo | 9c5f2d6 | 3 | completed=None, n_tasks=None, makespan=None, mean_task_s=None | C3/dist_alive |
| P5_p5_C3_p3_dist_aliver2 | P5-C3 | gazebo | gazebo | 9c5f2d6 | 3 | completed=5, n_tasks=5, makespan=333.09, mean_task_s=263.5 | C3/dist_alive |
| P5_p5_C3_p4_dist_alive | P5-C3 | gazebo | gazebo | 9c5f2d6 | 4 | completed=5, n_tasks=5, makespan=346.03, mean_task_s=262.21 | C3/dist_alive |
| P5_p5_C3_p4_dist_dead | P5-C3 | gazebo | gazebo | 9c5f2d6 | 4 | completed=5, n_tasks=5, makespan=320.4, mean_task_s=254.91 | C3/dist_dead |
| P5_p5_C3_p5_dist_dead | P5-C3 | gazebo | gazebo | 9c5f2d6 | 5 | completed=5, n_tasks=5, makespan=282.68, mean_task_s=235.02 | C3/dist_dead |
| P5_p5_C3_p5_dist_alive | P5-C3 | gazebo | gazebo | 9c5f2d6 | 5 | completed=5, n_tasks=5, makespan=298.41, mean_task_s=247.15 | C3/dist_alive |
| P5_p5_C3_p6_dist_alive | P5-C3 | gazebo | gazebo | 9c5f2d6 | 6 | completed=5, n_tasks=5, makespan=313.36, mean_task_s=259.2 | C3/dist_alive |
| P5_p5_C3_p6_dist_dead | P5-C3 | gazebo | gazebo | 9c5f2d6 | 6 | completed=5, n_tasks=5, makespan=312.59, mean_task_s=246.88 | C3/dist_dead |
| P5_p5_C3_p7_dist_dead | P5-C3 | gazebo | gazebo | 9c5f2d6 | 7 | completed=5, n_tasks=5, makespan=397.01, mean_task_s=261.35 | C3/dist_dead |
| P5_p5_C3_p7_dist_alive | P5-C3 | gazebo | gazebo | 9c5f2d6 | 7 | completed=5, n_tasks=5, makespan=323.4, mean_task_s=256.55 | C3/dist_alive |
| P5_p5_C3_p8_dist_alive | P5-C3 | gazebo | gazebo | 9c5f2d6 | 8 | completed=5, n_tasks=5, makespan=324.56, mean_task_s=273.83 | C3/dist_alive |
| P5_p5_C3_p8_dist_dead | P5-C3 | gazebo | gazebo | 9c5f2d6 | 8 | completed=5, n_tasks=5, makespan=289.69, mean_task_s=240.56 | C3/dist_dead |
| P5_p5_P_p2_dist_p_b | P5-P | gazebo | gazebo | 9c5f2d6 | 2 | completed=8, n_tasks=8, makespan=650.86, mean_task_s=366.13 | P/dist_p_b |
| P5_p5_P_p2_dist_p_a | P5-P | gazebo | gazebo | 9c5f2d6 | 2 | completed=7, n_tasks=8, makespan=566.88, mean_task_s=308.48 | P/dist_p_a |
| P5_p5_S_p2_dist_s_b | P5-S | gazebo | gazebo | 9c5f2d6 | 2 | completed=8, n_tasks=8, makespan=620.14, mean_task_s=312.04 | S/dist_s_b |
| P5_p5_S_p2_dist_s_a | P5-S | gazebo | gazebo | 9c5f2d6 | 2 | completed=6, n_tasks=8, makespan=529.29, mean_task_s=250.66 | S/dist_s_a |
| P5_p5_X_p2_dist_x_b | P5-X | gazebo | gazebo | 9c5f2d6 | 2 | completed=5, n_tasks=5, makespan=409.25, mean_task_s=269.87 | X/dist_x_b |
| P5_p5_X_p2_dist_x_a | P5-X | gazebo | gazebo | 9c5f2d6 | 2 | completed=5, n_tasks=5, makespan=305.17, mean_task_s=256.74 | X/dist_x_a |
| S12_hdds_s12_p1_base | F4-S12 | gazebo | gazebo | b33817f | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=판정 없음, verdict_detail=중단(감시): launch.log 가 900 s 동안 늘지 않음 (하네스 정지로 봄, 마지막 크기 271436 B) — 2026-10-08T10:30:23+09:00, stall=True, runner_wall_s=None | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p1_fix | F4-S12 | gazebo | gazebo | fa7291c | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=실패, verdict_detail=failure: test_10_concurrent_tasks, stall=False, runner_wall_s=3124 | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p2_fix | F4-S12 | gazebo | gazebo | fa7291c | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=실패, verdict_detail=failure: test_10_concurrent_tasks, test_20_forced_deadlock, stall=False, runner_wall_s=2208 | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p2_base | F4-S12 | gazebo | gazebo | b33817f | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=판정 없음, verdict_detail=중단(감시): launch.log 가 900 s 동안 늘지 않음 (하네스 정지로 봄, 마지막 크기 164045 B) — 2026-10-08T11:40:55+09:00, stall=True, runner_wall_s=None | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p3_base | F4-S12 | gazebo | gazebo | b33817f | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=판정 없음, verdict_detail=중단(감시): launch.log 가 900 s 동안 늘지 않음 (하네스 정지로 봄, 마지막 크기 239745 B) — 2026-10-08T12:19:39+09:00, stall=True, runner_wall_s=None | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p3_fix | F4-S12 | gazebo | gazebo | fa7291c | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=판정 없음, verdict_detail=중단(감시): launch.log 가 900 s 동안 늘지 않음 (하네스 정지로 봄, 마지막 크기 178514 B) — 2026-10-08T12:18:40+09:00, stall=True, runner_wall_s=None | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p4_fix | F4-S12 | gazebo | gazebo | fa7291c | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=실패, verdict_detail=failure: test_10_concurrent_tasks, stall=False, runner_wall_s=2332 | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p4_base | F4-S12 | gazebo | gazebo | b33817f | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=판정 없음, verdict_detail=중단(감시): launch.log 가 900 s 동안 늘지 않음 (하네스 정지로 봄, 마지막 크기 3431002 B) — 2026-10-08T13:09:27+09:00, stall=True, runner_wall_s=None | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p5_base | F4-S12 | gazebo | gazebo | b33817f | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=판정 없음, verdict_detail=중단(감시): launch.log 가 900 s 동안 늘지 않음 (하네스 정지로 봄, 마지막 크기 278627 B) — 2026-10-08T13:51:40+09:00, stall=True, runner_wall_s=None | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p5_fix | F4-S12 | gazebo | gazebo | fa7291c | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=통과, verdict_detail=5 케이스, stall=False, runner_wall_s=2352 | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p6_fix | F4-S12 | gazebo | gazebo | fa7291c | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=판정 없음, verdict_detail=중단(감시): launch.log 가 900 s 동안 늘지 않음 (하네스 정지로 봄, 마지막 크기 167482 B) — 2026-10-08T14:41:25+09:00, stall=True, runner_wall_s=None | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p6_base | F4-S12 | gazebo | gazebo | b33817f | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=실패, verdict_detail=failure: test_10_concurrent_tasks, test_20_forced_deadlock, stall=False, runner_wall_s=2205 | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p7_base | F4-S12 | gazebo | gazebo | b33817f | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=실패, verdict_detail=failure: test_10_concurrent_tasks, stall=False, runner_wall_s=2049 | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p7_fix | F4-S12 | gazebo | gazebo | fa7291c | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=판정 없음, verdict_detail=중단(감시): launch.log 가 900 s 동안 늘지 않음 (하네스 정지로 봄, 마지막 크기 10065160 B) — 2026-10-08T15:39:29+09:00, stall=True, runner_wall_s=None | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p8_fix | F4-S12 | gazebo | gazebo | fa7291c | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=실패, verdict_detail=failure: test_10_concurrent_tasks, stall=False, runner_wall_s=2043 | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_hdds_s12_p8_base | F4-S12 | gazebo | gazebo | b33817f | Gazebo 비결정 (시드 없음), 시나리오 settings.seed 7 | verdict=실패, verdict_detail=failure: test_10_concurrent_tasks, test_20_forced_deadlock, stall=False, runner_wall_s=2193 | [측정] 짝 동시 실행 (gz 슬롯 둘 독점, cpu 슬롯은 공유) — 공유 부하 하 예비값 |
| S12_build_base_fix | F4-S12 | test | offline | b33817f+fa7291c | 해당 없음 (빌드) | base_build_rc=0, base_warnings=0, fix_build_rc=0, fix_warnings=0 | [측정] 짝 실행용 빌드 (시험은 돌리지 않음 — 두 커밋의 시험은 hold-dds 가 이미 기록) |
| S12_review_mgr_tally | F4-S12 | eval | offline | 2182b1f | 해당 없음 (오프라인 집계) | manager_procs_diagnosed=6, with_lock_wait=3, with_lock_wait_list=['p5 base traffic_manager', 'p5 base fleet_manager', 'p7 fix traffic_manager'], without_lock_wait_list=['p6 fix traffic_manager', 'p6 fix fleet_manager', 'p7 fix fleet_manager'] | [계산] 사후 · 리뷰 반영 (보충, 사전 등록 외) |
| W_baseline | W | test | offline | fd17e49 | 0 | n=1, build_rc=0, build_warnings=0, amr_fleet={'tests': 855, 'pass': 815, 'skip': 40, 'fail': 0} | [측정] |
| W-f5-old | W | eval | 2d | fd17e49 | 1-30 | n=240, n_err=0, per_cell_n=30, A1h_nearest_hold|F0|dock_runs=6 | [측정] 운동학 모의 (Gazebo 아님) — 결과는 결정적(digest), 공유 부하는 시간 값에만 영향 |
| W-f5-new | W | eval | 2d | 3f33338 | 1-30 | n=240, n_err=0, per_cell_n=30, A1h_nearest_hold|F0|dock_runs=0 | [측정] 운동학 모의 (Gazebo 아님) — 결과는 결정적(digest), 공유 부하는 시간 값에만 영향 |
| W-dev-hb1 | W | test | offline | 8eaab5c | 1 | n=1, pytest=1 passed (93 s), note=실제 관리자 하트비트로 P3 ①–④b 통과 | 개발 실행 (판정 아님) |
| W-dev-pg1 | W | test | offline | 23fd48b | 1 | n=1, pytest=2 passed (172 s), note=에이전트 노드 변경 뒤 P3 launch 시험 회귀 없음 (가짜 실행기 경로) | 개발 실행 (판정 아님) |
| W-dev-w1a | W | test | offline | cb7be26 | 1 | n=1, pytest=1 passed (177 s), note=③ 실행기 도달 확인이 공허했다 (시험 요청에 stamp 없음 → 실행기 수락 로그 없음) → 판정 전 고침 | 개발 실행 (판정 아님) |
| W-dev-w2a | W | test | offline | cb7be26 | 1 | n=1, pytest=1 failed — 공허 검사(다수 쪽 분할 중 확정 ≥ 1) 실패: 다수 쪽 확정이 모두 분할 전. 안전 판정(소수 쪽 확정 0 · 중복 0 · 확정 없는 적재 0)은 통과, action=분할을 [112, 175) 로 옮김 (판정 전) | 개발 실행 (판정 아님) |
| W-dev-w2b | W | test | offline | cb7be26 | 1 | n=1, pytest=1 passed (127 s) | 개발 실행 (판정 아님) |
| W-mut | W | test | offline | 0748e72 | 1 | n_mutations=11, base_build_rc=0, M-HB={'verdict': 'SURVIVED', 'tests': 'test/test_manager_heartbeat.py', 'note': '공허 — 도구가 PYTHONPATH 를 덮어써 rclpy 노드 시험 3 개가 모두 건너뜀 (판정 불가, W-mut2 에서 다시)'}, M-OBS={'verdict': 'KILLED', 'tests': 'test/test_heartbeat.py test/test_manager_heartbeat.py', 'note': '순수 시험 test_heartbeat 가 잡음 (같은 도구 결함으로 test_manager_heartbeat 는 건너뜀 — W-mut2 에서 다시)'} | [측정] |
| W-W1-r1-s1 | W | test | offline | 0748e72 | 1 | n=1, pytest_rc=0, note=판정 실행 1 회차 (중단): 같은 컨테이너에서 앞 시드의 고아 에이전트(harness bash -c 에 exec 없음)가 같은 도메인에 남아 시드 2·3 을 오염 — orphans_ps_112257.txt, f42eff0 에서 고침, duplicate_nodes=None | [측정] 유효로 본다 — W1 은 새 컨테이너의 첫 시험(고아 없음), W2 시드 1 앞의 고아는 다른 도메인(56) 뿐. Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W2-r1-s1 | W | test | offline | 0748e72 | 1 | n=1, pytest_rc=0, note=판정 실행 1 회차 (중단): 같은 컨테이너에서 앞 시드의 고아 에이전트(harness bash -c 에 exec 없음)가 같은 도메인에 남아 시드 2·3 을 오염 — orphans_ps_112257.txt, f42eff0 에서 고침, minority_held_in_partition=[] | [측정] 유효로 본다 — W1 은 새 컨테이너의 첫 시험(고아 없음), W2 시드 1 앞의 고아는 다른 도메인(56) 뿐. Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W1-r1-s2 | W | test | offline | 0748e72 | 2 | n=1, pytest_rc=1, note=판정 실행 1 회차 (중단): 같은 컨테이너에서 앞 시드의 고아 에이전트(harness bash -c 에 exec 없음)가 같은 도메인에 남아 시드 2·3 을 오염 — orphans_ps_112257.txt, f42eff0 에서 고침, duplicate_nodes=None | [측정] 오염 — 판정 불가 (앞 시드 고아 에이전트 5 개씩이 같은 도메인에 남음) |
| W-W2-r1-s2 | W | test | offline | 0748e72 | 2 | n=1, pytest_rc=1, note=판정 실행 1 회차 (중단): 같은 컨테이너에서 앞 시드의 고아 에이전트(harness bash -c 에 exec 없음)가 같은 도메인에 남아 시드 2·3 을 오염 — orphans_ps_112257.txt, f42eff0 에서 고침, minority_held_in_partition=None | [측정] 오염 — 판정 불가 (앞 시드 고아 에이전트 5 개씩이 같은 도메인에 남음) |
| W-W1-r1-s3 | W | test | offline | 0748e72 | 3 | n=1, pytest_rc=None, note=판정 실행 1 회차 (중단): 같은 컨테이너에서 앞 시드의 고아 에이전트(harness bash -c 에 exec 없음)가 같은 도메인에 남아 시드 2·3 을 오염 — orphans_ps_112257.txt, f42eff0 에서 고침, stopped=시험 도중 docker stop (11:22:57 KST, 오염 확인 뒤) | [측정] 오염 — 판정 불가 (앞 시드 고아 에이전트 5 개씩이 같은 도메인에 남음) |
| W-mut2 | W | test | offline | ed14426 | 1 | n_mutations=2, base_build_rc=0, M-HB={'verdict': 'INVALID', 'test_rc': 4, 'pytest': 'ERROR: found no collectors · 1 skipped'}, M-OBS={'verdict': 'INVALID', 'test_rc': 4, 'pytest': 'ERROR: found no collectors ×2 · 2 skipped'} | [측정] 판정 불가 (INVALID — 도구가 걸러냄) |
| W-regress-r1 | W | test | offline | 0748e72 | 0 | n=1, build_rc=0, build_warnings=0, tests=중단 — amr_fleet 시험 중에 멈춤 (결과 없음) | [측정] 빌드 경고 0 만 유효, 시험 결과 없음 |
| W-dev-orphan | W | test | offline | ed14426 | 101 | n=2, W1={'rc': 0, 'strays_before': 0, 'strays_after': 0}, W2={'rc': 0, 'strays_before': 0, 'strays_after': 0}, launch_escalations=0 | 개발 실행 (판정 아님) |
| W-dev-nobringup | W | test | offline | ed14426 | 0 | n=1, before_fix=1 skipped (rc 0) — 같은 설치 · 고치기 전 소스(W-mut2 스냅숏), after_fix=772 passed, 5 skipped (369 s) | 개발 실행 (판정 아님) |
| W-W1-r2-s1 | W | test | offline | f42eff0 | 1 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W2-r2-s1 | W | test | offline | f42eff0 | 1 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W1-r2-s2 | W | test | offline | f42eff0 | 2 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W2-r2-s2 | W | test | offline | f42eff0 | 2 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W1-r2-s3 | W | test | offline | f42eff0 | 3 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W2-r2-s3 | W | test | offline | f42eff0 | 3 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-regress2 | W | test | offline | f42eff0 | 0 | n=1, build_rc=0, build_warnings=0, base_totals={'amr_behavior': {'pass': 312, 'tests': 379, 'skip': 67}, 'amr_bringup': {'pass': 101, 'tests': 101}, 'amr_fleet': {'pass': 815, 'tests': 855, 'skip': 40}, 'amr_msgs': {'pass': 2, 'tests': 2}} | [측정] 회귀 1 건 (진단 중) · 빌드 경고 0 |
| W-mut3 | W | test | offline | f42eff0 | 1 | n_mutations=2, base_build_rc=0, M-HB={'verdict': 'KILLED', 'test_rc': 1, 'by': 'test_manager_heartbeat::test_observer_then_active_with_max_seen_plus_one (비컨 대기 단언, 1 failed · 1 passed, -x)'}, M-OBS={'verdict': 'KILLED', 'test_rc': 1, 'by': 'test_heartbeat (1 failed · 2 passed, -x)'} | [측정] |
| W-diag-A | W | test | offline | fd17e49 | 1 ×4 | n=4, vacuity_fail=0, pytest_fail=0, unsafe=0 | [측정] 진단 (판정 규칙: 노트 …_1200_…) |
| W-diag-B | W | test | offline | f42eff0 | 1 ×4 | n=4, vacuity_fail=1, pytest_fail=1, unsafe=0 | [측정] 진단 (판정 규칙: 노트 …_1200_…) |
| W-diag-C | W | test | offline | f42eff0 | 1 ×4 | n=4, vacuity_fail=2, pytest_fail=2, unsafe=0 | [측정] 진단 (판정 규칙: 노트 …_1200_…) |
| W-diag2-A | W | test | offline | fd17e49 | 1 ×8 | n=8, vacuity_fail=0, unsafe=0, runs=[{'run': 'run_1', 'rc': 0, 'majority_pickups_in_partition': 3, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_2', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_3', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_4', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_5', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_6', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_7', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_8', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}] | [측정] 진단 (판정: 가르지 못함, 노트 …_1200_…) |
| W-diag2-D | W | test | offline | 6cb4821 | 1 ×8 | n=8, vacuity_fail=1, unsafe=0, runs=[{'run': 'run_1', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_2', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_3', 'rc': 1, 'majority_pickups_in_partition': 0, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 13}, {'run': 'run_4', 'rc': 0, 'majority_pickups_in_partition': 3, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_5', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_6', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_7', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}, {'run': 'run_8', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0}] | [측정] 진단 (판정: 가르지 못함, 노트 …_1200_…) |
| W-W1-r3-s1 | W | test | offline | 6cb4821 | 1 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W2-r3-s1 | W | test | offline | 6cb4821 | 1 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W1-r3-s2 | W | test | offline | 6cb4821 | 2 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W2-r3-s2 | W | test | offline | 6cb4821 | 2 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W1-r3-s3 | W | test | offline | 6cb4821 | 3 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-W2-r3-s3 | W | test | offline | 6cb4821 | 3 | n=1, pytest_rc=0, strays_before=0, strays_after=0 | [측정] Gazebo 아님 (운동학 대역) — 시간은 공유 부하 하 예비값 |
| W-mut4 | W | test | offline | 6cb4821 | 1 | n_mutations=2, base_build_rc=0, M-GC={'verdict': 'KILLED', 'test_rc': 1, 'by': 'test_agent_dispatch::test_busy_rejection_releases_without_blocking_and_invalid_blocks (-x 첫 실패)'}, M-STAMP={'verdict': 'KILLED', 'test_rc': 1, 'by': 'test_agent_dispatch::test_stale_lock_needs_a_beacon_made_after_the_lock_was_seen (1 failed, 5 passed)'} | [측정] |
| W-regress3 | W | test | offline | 6cb4821 | 0 | n=1, build_rc=0, build_warnings=0, base_totals={'amr_behavior': {'pass': 312, 'tests': 379, 'skip': 67}, 'amr_bringup': {'pass': 101, 'tests': 101}, 'amr_fleet': {'pass': 815, 'tests': 855, 'skip': 40}, 'amr_msgs': {'pass': 2, 'tests': 2}} | [측정] 회귀 0 (이름 바뀜 · 옮김 3 건 설명) · 빌드 경고 0 — 공유 부하 매우 높음 |
| W-diag3-A | W | test | offline | fd17e49 | 2-9 | n=8, vacuity_fail=1, unsafe=0, runs=[{'run': 'run_1', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 2}, {'run': 'run_2', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 3}, {'run': 'run_3', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 4}, {'run': 'run_4', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 5}, {'run': 'run_5', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 6}, {'run': 'run_6', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 7}, {'run': 'run_7', 'rc': 1, 'majority_pickups_in_partition': 0, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 8}, {'run': 'run_8', 'rc': 0, 'majority_pickups_in_partition': 3, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 9}] | [측정] 진단 (판정: 기준에도 실패 · 가르지 못함, 노트 …_1200_…) — 공유 부하 높음 |
| W-diag3-D | W | test | offline | 921a0d1 | 2-9 | n=8, vacuity_fail=0, unsafe=0, runs=[{'run': 'run_1', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 2}, {'run': 'run_2', 'rc': 0, 'majority_pickups_in_partition': 1, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 3}, {'run': 'run_3', 'rc': 0, 'majority_pickups_in_partition': 3, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 4}, {'run': 'run_4', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 5}, {'run': 'run_5', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 6}, {'run': 'run_6', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 7}, {'run': 'run_7', 'rc': 0, 'majority_pickups_in_partition': 2, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 8}, {'run': 'run_8', 'rc': 0, 'majority_pickups_in_partition': 3, 'minority_pickups_in_partition': 0, 'duplicates': 0, 'stale_lock_cleared': 0, 'seed': 9}] | [측정] 진단 (판정: 기준에도 실패 · 가르지 못함, 노트 …_1200_…) — 공유 부하 높음 |
