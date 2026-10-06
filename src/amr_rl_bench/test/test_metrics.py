"""amr_rl_bench.metrics 시험 — 합성 에피소드로 층위 성공·계약 종단 사건·정체·접촉·개입·이탈."""

import math

from amr_rl_bench import metrics as M
import numpy as np
import pytest


def drive(stop_at, goal=(10.0, 0.0, 0.0), v=1.0, dt=0.02, t0=100.0, hold=5.0, y=0.0):
    """지면 진실 [[t, x, y, yaw, v, w]]: x 축으로 v 로 달려 stop_at 에서 멈춘 뒤 hold 초 서 있다."""
    rows, t, x = [], t0, 0.0
    while x < stop_at:
        rows.append([t, x, y, 0.0, v, 0.0])
        t += dt
        x = min(stop_at, x + v * dt)
    for _ in range(int(hold / dt)):
        rows.append([t, x, y, 0.0, 0.0, 0.0])
        t += dt
    return rows


def episode(gt, goal=(10.0, 0.0, 0.0), t_limit=50.0, status=4, reason='', **kw):
    """지표 입력 dict."""
    t0 = gt[0][0]
    ep = {'goal': goal, 't0': t0, 't_end': gt[-1][0], 't_limit': t_limit, 'nav2_status': status,
          'cancel_reason': reason, 'gt': gt, 'est': [[r[0], r[1], r[2], r[3]] for r in gt],
          'final': {'gt': gt[-1][1:6], 'est': gt[-1][1:4]},
          'plans': [[t0, [[x, 0.0] for x in np.arange(0.0, 10.05, 0.05)]]],
          'wall': {'t0': 0.0, 't_end': (gt[-1][0] - t0) / 0.8}}
    ep.update(kw)
    return ep


def runner_shape(ep, final_est=None, final_t=None):
    """
    실행기가 남기는 모양으로 바꾼다: 추정 표본은 t_end(Nav2 종료)까지만, 최종 추정은 기록 끝에 따로.

    A5 리뷰 차단 1: episode() 는 추정을 정착 구간까지 지면 진실과 같게 넣어, 정착 구간 추정이 없는 실제 원자료(A5-4)의 결함을
    시험이 잡지 못했다. final_t 가 None 이면 'final' 에 't' 를 넣지 않는다 (옛 원자료 — 지표가 기록 끝 시각을 쓴다).
    """
    ep['est'] = [r for r in ep['est'] if r[0] <= ep['t_end']]
    if final_est is not None:
        ep['final']['est'] = list(final_est)
    if final_t is not None:
        ep['final']['t'] = final_t
    return ep


def test_success_layers_and_rtf():
    """목표 0.03 m 앞에서 정지 → 엄격·Nav2 호환 성공, 계약 성공, RTF 0.8."""
    m = M.episode_metrics(episode(drive(9.97)))
    assert m['outcome_contract'] == 'success'
    assert m['strict_gt'] and m['strict_est'] and m['nav2compat_gt'] and m['system_success']
    assert m['final_ok_gt']
    assert m['final_dist_gt_m'] == pytest.approx(0.03, abs=1e-6)
    assert m['t_strict_gt_s'] == pytest.approx(9.97, abs=0.03)
    assert m['t_contract_s'] == pytest.approx(m['t_strict_gt_s'])
    assert m['rtf'] == pytest.approx(0.8)
    assert m['path_length_m'] == pytest.approx(9.97, abs=1e-6)
    assert m['cte_first_max_m'] == pytest.approx(0.0, abs=1e-9)
    assert m['contacts_static'] == 0 and m['contacts_contract'] == 0
    assert m['near_goal_stop_s'] == 0.0


def test_near_goal_stop_then_stall():
    """0.12 m 앞에서 멈춤 (A2 직전 정지 모양): 엄격 실패, Nav2 호환 성공, 20 s 뒤 정체가 계약 종단 사건."""
    gt = drive(9.88, hold=25.0)
    m = M.episode_metrics(episode(gt, t_limit=60.0, status=6))
    assert not m['strict_gt'] and m['nav2compat_gt']
    assert m['outcome_contract'] == 'stall'
    # 마지막 기준점은 멈춘 자리에서 0.3 m 안 (t ∈ [9.58, 9.88] s) → 그 뒤 20 s
    assert 9.58 + 20.0 <= m['t_stall_s'] <= 9.88 + 20.0 + 0.05
    assert m['near_goal_stop_s'] == pytest.approx(25.0, abs=0.1)
    assert m['near_goal_stop_max_s'] == pytest.approx(25.0, abs=0.1)
    assert m['outcome_system'] == 'aborted'
    assert m['final_dist_gt_m'] == pytest.approx(0.12, abs=1e-6)


def test_timeout_before_stall():
    """시간 상한이 정체보다 먼저면 timeout."""
    gt = drive(9.5, hold=25.0)
    m = M.episode_metrics(episode(gt, t_limit=15.0, status=5, reason='timeout'))
    assert m['outcome_contract'] == 'timeout'
    assert m['t_contract_s'] == pytest.approx(15.0)
    assert m['outcome_system'] == 'timeout'
    assert not m['nav2compat_gt']


def test_collision_first_wins_and_contacts_counted():
    """정적 접촉(부호 거리 < 0) 이 먼저면 collision — 그 뒤 목표에 닿아도 성공이 아니다."""
    gt = drive(9.97)
    t0 = gt[0][0]
    clear = [[r[0], -0.01 if 3.0 <= r[0] - t0 <= 3.5 else 0.2] for r in gt]
    m = M.episode_metrics(episode(gt, clear_static=clear, reason='collision', status=5))
    assert m['outcome_contract'] == 'collision'
    assert m['contacts_static'] == 1 and m['contacts_contract'] == 1
    assert not m['strict_gt'] and not m['nav2compat_gt']
    assert m['min_clear_static_m'] == pytest.approx(-0.01)
    # 접촉이 성공 뒤라면 계약 결과는 success, 계약까지의 접촉 0
    clear2 = [[r[0], -0.01 if r[0] - t0 > 11.0 else 0.2] for r in gt]
    m2 = M.episode_metrics(episode(gt, clear_static=clear2))
    assert m2['outcome_contract'] == 'success'
    assert m2['contacts_static'] == 1 and m2['contacts_contract'] == 0


def test_dynamic_contacts_from_collision_monitor_delta():
    """collision_monitor 누적 수의 증가만 센다 (시작 전 값은 빼고)."""
    gt = drive(9.97)
    t0 = gt[0][0]
    cm = [[t0 - 5.0, 3], [t0 + 2.0, 3], [t0 + 4.0, 4]]
    m = M.episode_metrics(episode(gt, cm_contacts=cm))
    assert m['contacts_dynamic'] == 1
    assert m['outcome_contract'] == 'collision'


def test_estimate_offset_changes_est_layers_only():
    """추정이 0.08 m 어긋나면 정답 기준 성공, 추정 기준 실패 (A2: map 0.093 대 지면 진실 0.111 의 반대 꼴)."""
    gt = drive(9.97)
    est = [[r[0], r[1] - 0.08, r[2], r[3]] for r in gt]
    ep = episode(gt, est=est)
    ep['final']['est'] = [gt[-1][1] - 0.08, 0.0, 0.0]
    m = M.episode_metrics(ep)
    assert m['strict_gt'] and not m['strict_est']
    assert m['loc_err_max_m'] == pytest.approx(0.08)
    assert m['loc_err_end_m'] == pytest.approx(0.08)
    assert not m['loc_collapse']
    est2 = [[r[0], r[1] + (1.0 if r[0] - gt[0][0] > 5 else 0.0), r[2], r[3]] for r in gt]
    assert M.episode_metrics(episode(gt, est=est2))['loc_collapse']


def test_cte_against_first_and_active_plan_and_deviation():
    """평행 주행 (y 0.4): 첫 경로(y 0) 기준 CTE 0.4, 이탈 구간 1 개(돌아오지 않음)."""
    gt = drive(9.97, y=0.4)
    m = M.episode_metrics(episode(gt))
    assert m['cte_first_mean_m'] == pytest.approx(0.4)
    assert m['cte_active_p95_m'] == pytest.approx(0.4)
    assert m['dev_episodes'] == 1 and m['max_dev_m'] == pytest.approx(0.4)
    # 출발 전에 y 0.4 의 새 경로가 오면 활성 경로 기준은 0 (첫 경로 기준은 그대로 0.4)
    t0 = gt[0][0]
    plans = [[t0 - 1.0, [[x, 0.0] for x in np.arange(0, 10.05, 0.05)]],
             [t0, [[x, 0.4] for x in np.arange(0, 10.05, 0.05)]]]
    m2 = M.episode_metrics(episode(gt, plans=plans))
    assert m2['cte_first_mean_m'] == pytest.approx(0.4)
    assert m2['cte_active_p95_m'] == pytest.approx(0.0, abs=1e-9)
    assert m2['n_plans'] == 2


def test_interventions_pairing():
    """출력이 입력과 0.01 넘게 다른 연속 구간 하나 = 1 회."""
    t_in = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10]
    v_in = [0.5] * 6
    t_out = [0.01, 0.03, 0.05, 0.07, 0.09, 0.11]
    v_out = [0.5, 0.3, 0.3, 0.5, 0.2, 0.5]
    n, dur, segs = M.interventions(t_in, v_in, [0.0] * 6, t_out, v_out, [0.0] * 6)
    assert n == 2
    assert segs[0] == (0.03, 0.07)
    assert dur == pytest.approx(0.04 + 0.02)
    assert M.interventions([], [], [], t_out, v_out, [0.0] * 6)[0] == 0


def test_interventions_tolerates_echo_lag_and_receive_order():
    """
    램프 (0.0156 m/s / 20 ms) 를 safety 가 20 ms 늦게 그대로 내고, 출력과 다음 입력을 같은 시각에 받아도 개입 0.

    흔들기 2 의 원자료 모양 — 직전 입력과만 짝지으면 램프 동안 거짓 개입이 생긴다.
    """
    t_in = [0.02 * k for k in range(50)]
    v_in = [0.0156 * k for k in range(50)]
    t_out = [t + 0.02 for t in t_in]          # 다음 입력과 같은 시각에 받는다
    v_out = list(v_in)                         # 20 ms 전 입력 그대로
    assert M.interventions(t_in, v_in, [0.0] * 50, t_out, v_out, [0.0] * 50)[0] == 0
    # 진짜 제한: 출력이 0.2 에서 묶이면 개입
    v_cap = [min(v, 0.2) for v in v_out]
    n, dur, _ = M.interventions(t_in, v_in, [0.0] * 50, t_out, v_cap, [0.0] * 50)
    assert n == 1 and dur > 0.5


def test_first_stall_progress_checker_semantics():
    """기준점에서 0.3 m 넘게 움직이면 기준 갱신 — 천천히(0.01 m/s) 움직여도 30 s 마다 0.3 m 라 20 s 안에 갱신 안 됨."""
    t = np.arange(0.0, 60.0, 0.1)
    xy = np.stack([0.01 * t, 0 * t], 1)
    assert M.first_stall(t, xy, 0.0) == pytest.approx(20.1, abs=0.11)
    xy_fast = np.stack([0.02 * t, 0 * t], 1)    # 15 s 마다 0.3 m → 갱신
    assert M.first_stall(t, xy_fast, 0.0) is None


def test_zone_times_latched_value_before_window():
    """래치 존: 구간 시작 전 마지막 값부터 센다."""
    z = M.zone_times([0.0, 12.0, 13.0], [1, 3, 0], 10.0, 20.0)
    assert z[1] == pytest.approx(2.0) and z[3] == pytest.approx(1.0) and z[2] == 0.0


def test_jerk_rms_cubic():
    """세제곱 x = t³ → 저크 6."""
    t = np.arange(0.0, 2.0, 0.02)
    assert M.jerk_rms(t, t ** 3) == pytest.approx(6.0, rel=1e-6)


def test_lost_events_and_start_flag():
    """위치 상실은 False→True 전이만 센다. 시작 전 True 면 lost_at_start."""
    gt = drive(9.97)
    t0 = gt[0][0]
    m = M.episode_metrics(episode(gt, lost=[[t0 - 1, True], [t0 + 1, False], [t0 + 2, True],
                                            [t0 + 3, True], [t0 + 4, False]]))
    assert m['lost_events'] == 1 and m['lost_at_start']


def test_rl_stats_fields():
    """플러그인 stats (31 필드): 결정 출처·DWA 사유·결정 주기."""
    gt = drive(9.97)
    t0 = gt[0][0]
    stats = []
    for k in range(40):
        row = [0.0] * 31
        row[12] = 1.0 if k % 2 == 0 else 0.0
        row[4] = 2.0 if k in (10, 11) else 0.0
        row[5] = 5.0 if k in (10, 11) else 0.0
        row[3] = 0.2
        row[18] = 0.05
        row[26] = 0.05
        row[27] = t0 + 0.05 * k
        stats.append([t0 + 0.05 * k, 0.0, row])
    m = M.episode_metrics(episode(gt, stats=stats))
    assert m['decisions'] == 20 and m['src_dwa'] == 1 and m['dwa_handover'] == 1
    assert m['decision_period_mean_s'] == pytest.approx(0.1)
    assert m['tick_dt_sim_mean'] == pytest.approx(0.05)
    assert m['infer_ms_p95'] == pytest.approx(0.2)
    assert math.isclose(m['scan_age_p50_s'], 0.05)


def test_nav2_success_while_moving_counts_when_robot_stops_in_settle():
    """DWA 꼴: goal checker 가 0.05 m/s 넘게 움직일 때 성공을 내고 정착 중 선다 → 정착 구간의 엄격 성공 (A5-4)."""
    gt = drive(9.97, hold=2.0)
    t_move_end = next(r[0] for r in gt if r[4] == 0.0)
    ep = episode(gt, t_limit=50.0)
    ep['t_end'] = t_move_end - 0.06        # 아직 1 m/s 로 움직일 때 Nav2 성공
    m = M.episode_metrics(ep)
    assert m['outcome_contract'] == 'success' and m['strict_gt']
    assert m['t_strict_gt_s'] > m['nav_time_s']
    assert m['contract_note'] is None
    assert m['strict_est'] and m['outcome_contract_est'] == 'success'


def test_runner_shape_moving_success_uses_final_estimate():
    """
    m035 꼴: Nav2 가 목표 0.13 m 에서 움직이며 성공 → 추정 표본은 거기서 끝나고 정착 뒤 최종 추정 0.03 m.

    낡은 추정(0.13 m)을 끝값으로 끌어 쓰면 strict_est 가 거짓 실패가 된다 (리뷰 차단 1 (a)). 최종 추정을 기록 끝 표본으로
    덧붙이면 정답 기준과 같이 성공.
    """
    gt = drive(9.97, hold=2.0)
    t_move_end = next(r[0] for r in gt if r[4] == 0.0)
    ep = episode(gt)
    ep['t_end'] = t_move_end - 0.10            # x ≈ 9.87, 1 m/s
    runner_shape(ep, final_est=[9.97, 0.0, 0.0])
    m = M.episode_metrics(ep)
    assert m['strict_gt'] and m['outcome_contract'] == 'success'
    assert m['strict_est'] and m['outcome_contract_est'] == 'success'
    assert m['t_strict_est_s'] == pytest.approx(gt[-1][0] - gt[0][0])   # 기록 끝 한 점에서
    assert m['est_end_gap_s'] == pytest.approx(gt[-1][0] - ep['est'][-1][0])
    assert m['est_end_gap_s'] > 1.0


def test_runner_shape_stale_estimate_inside_goal_is_not_success():
    """
    반대 꼴: Nav2 종료 순간 추정이 0.05 m (움직이는 중) → 로봇은 지나쳐 0.13 m 에 선다 (최종 추정도 0.13).

    낡은 추정을 끝값으로 쓰면 '멈춘 지면 진실 속도 + 0.05 m 추정' 으로 거짓 성공이 난다.
    """
    gt = drive(10.13, hold=2.0)
    ep = episode(gt, status=4)
    ep['t_end'] = next(r[0] for r in gt if r[1] >= 9.95)
    runner_shape(ep, final_est=[10.13, 0.0, 0.0])
    m = M.episode_metrics(ep)
    assert not m['strict_gt']
    assert not m['strict_est'] and m['outcome_contract_est'] == m['outcome_contract'] == 'stall'
    assert m['final_dist_est_m'] == pytest.approx(0.13, abs=1e-6)


def test_est_success_after_stall_is_not_strict():
    """
    s114 꼴: 0.12 m 에 서서 20 s 뒤 정체(계약 종단) — 그 뒤 추정이 0.09 m 로 바뀌어도 추정 기준 성공이 아니다 (리뷰 차단 1 (b)).

    정체 전에 추정이 이미 0.09 m 였다면 (s089 꼴, 진짜 위치추정 갈림) 추정 기준은 성공이다.
    """
    gt = drive(9.88, hold=40.0)
    t0 = gt[0][0]
    late = [[r[0], r[1] + (0.03 if r[0] - t0 > 40.0 else 0.0), r[2], r[3]] for r in gt]
    ep = episode(gt, t_limit=120.0, est=late)
    ep['final']['est'] = [9.91, 0.0, 0.0]
    m = M.episode_metrics(ep)
    assert m['outcome_contract'] == 'stall' and not m['strict_gt']
    assert m['t_strict_est_s'] == pytest.approx(40.0, abs=0.05)
    assert m['t_stall_s'] < m['t_strict_est_s']
    assert m['outcome_contract_est'] == 'stall' and not m['strict_est']
    early = [[r[0], r[1] + 0.03, r[2], r[3]] for r in gt]
    ep2 = episode(gt, t_limit=120.0, est=early)
    ep2['final']['est'] = [9.91, 0.0, 0.0]
    m2 = M.episode_metrics(ep2)
    assert m2['outcome_contract'] == 'stall' and not m2['strict_gt']
    assert m2['outcome_contract_est'] == 'success' and m2['strict_est']


def test_timeout_final_estimate_measured_after_limit_is_outside_window():
    """
    시간 상한(12 s)에 0.15 m 에 서 있다가 취소 → 정착 뒤(상한 뒤) 잰 최종 추정이 0.08 m.

    최종 추정은 잰 시각(기록 끝)의 표본이라 상한 밖 — 상한 전 성공으로 끼지 않는다. 추정 표본은 상한 0.5 s 전에 끊긴다
    (TF 조회 실패) — 최종 추정을 '상한 안 마지막 지면 진실 시각' 에 붙이면 거짓 성공이 된다.
    """
    gt = drive(9.85, hold=4.0)
    t0 = gt[0][0]
    ep = episode(gt, t_limit=12.0, status=5, reason='timeout')
    ep['t_end'] = t0 + 12.0
    runner_shape(ep, final_est=[9.92, 0.0, 0.0])
    ep['est'] = [r for r in ep['est'] if r[0] - t0 <= 11.5]
    m = M.episode_metrics(ep)
    assert m['outcome_contract'] == 'timeout' and not m['strict_gt']
    assert m['outcome_contract_est'] == 'timeout' and not m['strict_est']
    assert m['t_strict_est_s'] is None
    # 실행기가 final.t 를 남기면 그 시각을 쓴다 (같은 결론)
    ep2 = runner_shape(episode(gt, t_limit=12.0, status=5, reason='timeout'),
                       final_est=[9.92, 0.0, 0.0], final_t=gt[-1][0])
    ep2['t_end'] = t0 + 12.0
    ep2['est'] = [r for r in ep2['est'] if r[0] <= ep2['t_end']]
    assert not M.episode_metrics(ep2)['strict_est']


def test_interp_pose_only_inside_samples_and_short_gaps():
    """추정 보간은 표본 구간 안, 틈 ≤ EST_MAX_GAP 에서만 (끝값 고정·긴 틈 보간 금지)."""
    tt = np.array([0.0, 0.1, 0.2, 1.0, 1.1])
    tr = np.stack([tt, 0 * tt, 0 * tt], 1)
    q = np.array([-0.1, 0.05, 0.2, 0.5, 1.05, 1.1, 1.2])
    x, _, _, ok = M._interp_pose(tt, tr, q)
    assert list(ok) == [False, True, True, False, True, True, False]
    assert x[1] == pytest.approx(0.05) and math.isnan(x[3]) and math.isnan(x[0])
    _, _, _, ok1 = M._interp_pose([1.0], [[0.0, 0.0, 0.0]], [1.0, 1.01])
    assert list(ok1) == [True, False]
    _, _, _, ok0 = M._interp_pose([], np.zeros((0, 3)), [1.0])
    assert list(ok0) == [False]


def test_late_estimate_is_not_extrapolated_back_to_start():
    """추정 표본이 5 s 늦게 시작해도 그 앞을 첫 표본 값으로 채우지 않는다 (채우면 5 m 오차 = 거짓 붕괴)."""
    gt = drive(9.97)
    t0 = gt[0][0]
    est = [[r[0], r[1], r[2], r[3]] for r in gt if r[0] - t0 >= 5.0]
    m = M.episode_metrics(episode(gt, est=est))
    assert m['loc_err_max_m'] == pytest.approx(0.0, abs=1e-9)
    assert not m['loc_collapse']
    assert m['strict_est']


def test_contract_first_order_and_ties():
    """시간순 첫째, 같은 시각이면 접촉 → 성공 → 정체 → 상한."""
    assert M.contract_first(None, None, None, None) is None
    assert M.contract_first(5.0, 5.0, None, None) == (5.0, 'collision')
    assert M.contract_first(None, 7.0, 7.0, 7.0) == (7.0, 'success')
    assert M.contract_first(None, 9.0, 8.0, None) == (8.0, 'stall')
    assert M.contract_first(None, None, 9.0, 8.0) == (8.0, 'timeout')


def test_system_ended_short_of_goal_is_stall_not_timeout():
    """Nav2 가 끝났는데(성공·중단) 0.12 m 에 섰다 → 시간 상한 전이므로 timeout 이 아니라 stall (끝난 채)."""
    gt = drive(9.88, hold=3.0)
    ep = episode(gt, t_limit=60.0, status=4)
    m = M.episode_metrics(ep)
    assert m['outcome_contract'] == 'stall'
    assert m['contract_note'] == 'ended_without_success'
    assert m['t_contract_s'] == pytest.approx(ep['t_end'] - ep['t0'])
    # 감시견 취소는 error
    ep2 = episode(gt, t_limit=60.0, status=5, reason='watchdog')
    assert M.episode_metrics(ep2)['outcome_contract'] == 'error'
