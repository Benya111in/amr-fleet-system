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
