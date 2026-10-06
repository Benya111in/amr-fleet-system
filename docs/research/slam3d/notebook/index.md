# 실험 노트 색인 (docs/research/slam3d/notebook)

> `python3 scripts/notebook.py --nb <노트 경로> index` 가 만든다 — 손으로 고치지 않는다.

## 노트 항목 (시간순)

| 시각 | 에이전트 | 제목 | 파일 |
|---|---|---|---|
| 2026-10-06 22:05 | b2-offline | 파일럿 bag 재생 — 지표 대칭화, LIO-SAM 평면·절제 행, RTAB-Map IMU 제거, 재생 3 회 산포 (사전 등록) | [entries/2026-10-06_2205_b2-offline_symmetric-replay-ablation.md](entries/2026-10-06_2205_b2-offline_symmetric-replay-ablation.md) |

## 실행 목록 요약 (56 건, 원본 `runs/<에이전트>.jsonl`)

| run_id | 단계 | 종류 | 시뮬 | 커밋 | 시드 | 핵심 결과 | 표기 |
|---|---|---|---|---|---|---|---|
| B2off-madg-derive-v0-fail | B2-5r | probe | offline | 4ce445a | n/a (결정적 입력) | rc=1, error=AttributeError: Subscription.get_publisher_count (Humble 에 없음) | 실패 |
| B2off-madg-derive-v1 | B2-5r | probe | offline | 4ce445a | n/a (결정적 입력) | n_out=35766, superseded_by=B2off-madg-derive, note=결정성 비교 기준 | [측정] |
| B2off-madg-derive | B2-5r | probe | offline | 4ce445a | n/a (결정적 입력) | n_in=35766, n_out_per_gain=35766, lockstep_wall_s=2.56, unexpected_outputs=0 | [측정] |
| B2off-sym-old | B2-5r | eval | offline | 3106929 | n/a (결정적 입력) | H0=pass, max_xy_change_common_window_cm=0.0015, rtabmap3d_xy_cm=0.769, liosam_loop_xy_cm=5.271 | [계산] |
| B2off-test-mut | B2-5r | test | offline | a31cc28 | n/a (결정적 입력) | mutants=14, caught=14, original_suite_in_copy=1 failed, 77 passed in 0.34s, note=사본의 실패 1 은 amr_description 미복사 (변이 대상 시험 아님) | [측정] |
| B2off-probe-sigint | B2-5r | probe | offline | 2126897 | n/a (결정적 입력) | plain_bg_sigint_ignored=True, function_wrapper=alive after 30s (SIGINT 가 서브셸로), array_wrapper=exited after 1s, nodes finished cleanly | [측정] |
| B2off-probe-tfguess | B2-5r | probe | offline | 2126897 | n/a (결정적 입력) | wait0.2_has_9.6=False, wait1.0_has_9.6=False, noguess_has_9.6=True, ekf_tf_gap=9.475→9.540 recv 0.337 s wall, 9.540→9.640 recv 0.002 s | [측정] |
| B2off-test-lint | B2-5r | test | offline | 0c51b24 | n/a (결정적 입력) | flake8=0, pep257=0, pytest_slam3d=70 passed, pytest_eval=47 passed | [측정] |
| B2off-rtabmap3d_noimu-r0 | B2-5r | eval | offline | 3106929 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7707, ate_xy_max_cm=1.689, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d_noimu-r1 | B2-5r | eval | offline | 2126897 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7711, ate_xy_max_cm=1.723, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d_noimu-r2 | B2-5r | eval | offline | 2126897 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7711, ate_xy_max_cm=1.723, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d_noimu-r3 | B2-5r | eval | offline | 2126897 | 재생 r3 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7708, ate_xy_max_cm=1.689, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d_imu-r0 | B2-5r | eval | offline | 2126897 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7696, ate_xy_max_cm=1.687, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d_imu-r1 | B2-5r | eval | offline | 2126897 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7699, ate_xy_max_cm=1.722, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d_imu-r2 | B2-5r | eval | offline | 2126897 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7695, ate_xy_max_cm=1.687, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap2d_noimu-r0 | B2-5r | eval | offline | 2126897 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=2.1298, ate_xy_max_cm=5.657, ate_yaw_rmse_deg=0.0721 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap2d_noimu-r1 | B2-5r | eval | offline | 2126897 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=2.1764, ate_xy_max_cm=5.868, ate_yaw_rmse_deg=0.0725 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap2d_noimu-r2 | B2-5r | eval | offline | 2126897 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=2.1519, ate_xy_max_cm=5.693, ate_yaw_rmse_deg=0.0723 | [측정] 공유 부하 하 예비값 |
| B2off-slam_toolbox-r0 | B2-5r | eval | offline | a31cc28 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=1.7405, ate_xy_max_cm=3.645, ate_yaw_rmse_deg=0.1632 | [측정] 공유 부하 하 예비값 |
| B2off-slam_toolbox-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=1.7407, ate_xy_max_cm=3.645, ate_yaw_rmse_deg=0.1632 | [측정] 공유 부하 하 예비값 |
| B2off-slam_toolbox-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=2.1309, ate_xy_max_cm=4.368, ate_yaw_rmse_deg=0.1428 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_planar-r0 | B2-5r | eval | offline | 3106929 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=1.7672, ate_xy_max_cm=4.588, ate_yaw_rmse_deg=0.0909 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_planar-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=1.587, ate_xy_max_cm=4.662, ate_yaw_rmse_deg=0.0776 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_planar-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=1.7666, ate_xy_max_cm=4.588, ate_yaw_rmse_deg=0.0909 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_loop_re-r0 | B2-5r | eval | offline | a31cc28 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.8432, ate_xy_max_cm=9.874, ate_yaw_rmse_deg=0.0764 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_loop_re-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.4255, ate_xy_max_cm=9.449, ate_yaw_rmse_deg=0.0777 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_loop_re-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.8079, ate_xy_max_cm=10.383, ate_yaw_rmse_deg=0.0761 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_noloop_re-r0 | B2-5r | eval | offline | a31cc28 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.4916, ate_xy_max_cm=9.579, ate_yaw_rmse_deg=0.0747 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_noloop_re-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.5761, ate_xy_max_cm=9.637, ate_yaw_rmse_deg=0.0756 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_noloop_re-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.4196, ate_xy_max_cm=9.171, ate_yaw_rmse_deg=0.0735 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_rpy0-r0 | B2-5r | eval | offline | a31cc28 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=6.1449, ate_xy_max_cm=10.845, ate_yaw_rmse_deg=0.0719 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_rpy0-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.8148, ate_xy_max_cm=11.049, ate_yaw_rmse_deg=0.0759 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_rpy0-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=6.1652, ate_xy_max_cm=11.055, ate_yaw_rmse_deg=0.0692 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_g001-r0 | B2-5r | eval | offline | 3106929 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.3733, ate_xy_max_cm=9.484, ate_yaw_rmse_deg=0.0782 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_g001-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.8083, ate_xy_max_cm=9.886, ate_yaw_rmse_deg=0.0765 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_g001-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.556, ate_xy_max_cm=9.512, ate_yaw_rmse_deg=0.0778 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_oracle-r0 | B2-5r | eval | offline | 3106929 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.5607, ate_xy_max_cm=10.056, ate_yaw_rmse_deg=0.0792 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_oracle-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.793, ate_xy_max_cm=9.926, ate_yaw_rmse_deg=0.0689 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_oracle-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.3984, ate_xy_max_cm=9.506, ate_yaw_rmse_deg=0.0773 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d-orig-r0 | B2-5r | eval | offline | a31cc28 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7699, ate_xy_max_cm=1.688, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d-orig-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7699, ate_xy_max_cm=1.688, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap3d-orig-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=0.7698, ate_xy_max_cm=1.688, ate_yaw_rmse_deg=0.0113 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap2d-orig-r0 | B2-5r | eval | offline | a31cc28 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=2.2126, ate_xy_max_cm=5.907, ate_yaw_rmse_deg=0.0738 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap2d-orig-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=2.2011, ate_xy_max_cm=5.904, ate_yaw_rmse_deg=0.073 | [측정] 공유 부하 하 예비값 |
| B2off-rtabmap2d-orig-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=2.1479, ate_xy_max_cm=5.676, ate_yaw_rmse_deg=0.0725 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_loop-orig-r0 | B2-5r | eval | offline | a31cc28 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=6.1764, ate_xy_max_cm=10.617, ate_yaw_rmse_deg=0.075 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_loop-orig-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.3178, ate_xy_max_cm=9.729, ate_yaw_rmse_deg=0.069 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_loop-orig-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.3711, ate_xy_max_cm=9.549, ate_yaw_rmse_deg=0.0703 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_noloop-orig-r0 | B2-5r | eval | offline | a31cc28 | 재생 r0 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.3026, ate_xy_max_cm=9.213, ate_yaw_rmse_deg=0.0693 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_noloop-orig-r1 | B2-5r | eval | offline | a31cc28 | 재생 r1 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.3422, ate_xy_max_cm=9.468, ate_yaw_rmse_deg=0.0683 | [측정] 공유 부하 하 예비값 |
| B2off-liosam_noloop-orig-r2 | B2-5r | eval | offline | a31cc28 | 재생 r2 (bag 1 개, 씨드 없음) | n_eval=1780, ate_xy_rmse_cm=5.2328, ate_xy_max_cm=9.135, ate_yaw_rmse_deg=0.0688 | [측정] 공유 부하 하 예비값 |
| B2off-test-colcon | B2-5r | test | offline | 338dcb6 | n/a | tests=445, failures=0, errors=0, skipped=8 | [측정] |
| B2off-calc-rv-drops | B2-5r | eval | offline | 6d70d3e | n/a (결정적 입력) | runs=43, missing_same_as_before=43, rtabmap_runs=16, rtabmap_missing_disagree=0 | [계산] |
| B2off-test-rv-lint | B2-5r | test | offline | 37ccd3d | n/a (결정적 입력) | flake8=0, pep257=0, pytest_slam3d=74 passed, pytest_eval_slam3d=89 passed, 8 skipped (evo 없음) | [측정] |
| B2off-test-rv-mut | B2-5r | test | offline | 6d70d3e | n/a (결정적 입력) | mutants=27, caught=27, new=8/8, old=13/13 | [측정] |
| B2off-test-rv-aflake8 | B2-5r | test | offline | 8463984 | n/a (결정적 입력) | ament_flake8_rc=0, files=4, log=rv/ament_flake8.log, note=colcon 전체 시험은 다시 돌리지 않음 | [측정] |
