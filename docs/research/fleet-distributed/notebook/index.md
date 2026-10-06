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

## 실행 목록 요약 (53 건, 원본 `runs/<에이전트>.jsonl`)

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
