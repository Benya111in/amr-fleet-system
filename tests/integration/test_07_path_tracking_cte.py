"""
시나리오 07: 경로 추종 CTE (명세 4.5 "CTE 직선 5 cm / 곡선 10 cm", 4.10 CTE 산출 절차).

스택(system 프로필): Gazebo + localization + navigation(직접 구현 DWA 컨트롤러 + A* 플래너) + perception
(safety_node = cmd_vel 발행자). 스폰 = 월드 원점 (명시).
준비: lifecycle 활성, map ↔ 월드 항등 정합 확인 — cte_logger 는 계획 경로(map 프레임)와 ground_truth(월드)를
정렬 없이 비교하므로 그 가정을 여기서 잰다. IMU 바이어스 정지 표본.
주행: navigate_to_pose 로 통로 직선(y = 0 → x = 15), 랙 끝 U 턴(곡선), 반대 통로 직선, 다시 U 턴.
목표마다 대기 상한 = 직선 거리 / 0.5 m/s × 2.5 + 60 s (전체가 러너 상한 안에 들게).
측정: amr_evaluation cte_logger 가 plan 과 ground_truth/odom 의 수직 거리를 곡률로 직선/곡선 구간으로 나눠
cte.csv 에 기록 → post-shutdown 에서 analyze 로 구간별 평균 |CTE| 판정 (직선·곡선 행이 모두 있어야 한다).
"""

import math

from amr_itest import actions, cases, catalog, evaluation, metrics
from amr_itest import requirements as req
from nav_msgs.msg import Odometry, Path
import numpy as np
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
import pytest

CTX = Context(catalog.get(7))

SPAWN = (0.0, 0.0, 0.0)
# (x, y, yaw) 목표 — 통로 직선 + 랙 끝(x ≈ ±20) U 턴 곡선
GOALS = [(15.0, 0.0, 0.0), (10.0, 6.0, math.pi), (-15.0, 6.0, math.pi), (-10.0, -6.0, 0.0),
         (0.0, 0.0, 0.0)]
NOMINAL_V = 0.5
# 명세 4장 4절 151행 "계획된 경로의 실제 주행 시간 대비 예측 시간 오차가 15% 이내여야 한다".
# **평균이 아니라 매 구간**이다 — 명세는 평균을 뜻할 때 '평균' 이라고 쓴다 (241행 "시스템 평균
# 응답 시간", 252행 "수직 거리 평균", 253행 "평균/최대"). 151행에는 그 말이 없다.
ETA_ERR_MAX_PCT = 15.0
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')


def goal_timeouts(goals, start=SPAWN, v: float = NOMINAL_V):
    """목표마다 대기 상한 [s] = 직선 거리 / v × 2.5 + 60 (우회·U 턴·복구 여유)."""
    out, prev = [], start
    for g in goals:
        out.append(math.hypot(g[0] - prev[0], g[1] - prev[1]) / v * 2.5 + 60.0)
        prev = g
    return out


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements()
                + [req.executable('amr_evaluation', 'cte_logger', 'CTE 측정 (명세 4.10 절차)'),
                   req.package('nav2_msgs')], 'path tracking')
    stack = Stack(CTX, *CTX.select())
    stack.system(use_localization=True, use_navigation=True, use_perception=True, pose=SPAWN)
    stack.eval_logger('cte_logger', {'path_topic': 'plan', 'gt_topic': 'ground_truth/odom'})
    return stack.launch_description(), {'stack': stack}


class TestPathTracking(cases.ProbeCase):
    """목표 순회 (측정은 cte_logger)."""

    CTX = CTX

    def _eta(self, plan, gt, w0: float, goal_yaw: float) -> dict:
        """목표 하나의 계획 경로 → 예측 주행 시간 [s] (명세 151행).

        예측은 저장소 자신의 모델 `amr_navigation.path_metrics.predict_travel_time` 을 쓴다
        (C++ SpeedProfile 과 같은 식: 곡률 상한 · 횡가속 상한 · 가감속 스윕 · 저크 보정).
        기본 ProfileLimits 가 nav2_params 의 실제 한계와 같다 (1.0 m/s · 1.0 m/s² ·
        1.5 rad/s · 2.0 rad/s²) — 확인하고 쓴다.

        **회전 시간을 넣는다.** 모델에 start/goal_heading_error 인자가 있는데 종전 측정
        (scratchpad/fixwave, 09-23)은 0 을 넘겨 제자리 회전을 통째로 빼먹었고, 그래서 예측이
        체계적으로 낮았다 (8 구간 합 −5.45 %). 0 을 넘기는 것은 더 엄격한 게 아니라 **모델을
        덜 쓰는 것**이다. 비교용으로 회전 없는 값도 t_pred_no_rot 으로 함께 남긴다.

        **판정은 t_pred(회전 포함)로 한다. 결과를 보고 갈아타지 않는다.** 09-23 자료로 미리
        확인하니 회전 보정이 합계 편향을 −5.45 % → +2.94 % 로 고치지만 가장 짧은 구간
        (7.59 m)에서 2.46 % → 16.73 % 로 **과보정**한다. (그 계산은 시작→목표 방위로 회전량을
        근사한 것이라 여기 배선보다 거칠다 — 여기는 계획 첫 구간 방향과 GT 방위를 쓴다.)
        실측에서도 짧은 구간이 과보정되면 그것은 **모델이 '서서 돌고 간다' 를 가정하는데 DWA 는
        돌면서 간다**는 뜻이다. 그때 할 일은 t_pred_no_rot 으로 판정을 바꾸는 것이 아니라
        선회-주행 동시 수행을 모델에 넣는 것이다.
        """
        from amr_navigation import path_metrics as pm
        msgs = [m for _, m in plan.messages(w0) if m.poses]
        if not msgs:
            return {'path_len': math.nan, 't_pred': None, 't_pred_no_rot': None}
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msgs[0].poses], dtype=float)
        if len(xy) < 2:
            return {'path_len': 0.0, 't_pred': None, 't_pred_no_rot': None}
        lim = pm.ProfileLimits()
        # 출발 시점의 **지면 진실** 방위. probe 에 pose() 가 없으므로 ground_truth/odom 의
        # 이 구간 첫 표본을 쓴다 (test_08 과 같은 방식: metrics.sample_from_odom).
        samples = [metrics.sample_from_odom(m) for _, m in gt.messages(w0)]
        path_dir = math.atan2(xy[1][1] - xy[0][1], xy[1][0] - xy[0][0])
        yaw0 = samples[0].yaw if samples else path_dir
        d = path_dir - yaw0
        start_err = abs(math.atan2(math.sin(d), math.cos(d)))
        end_dir = math.atan2(xy[-1][1] - xy[-2][1], xy[-1][0] - xy[-2][0])
        goal_err = abs(math.atan2(math.sin(goal_yaw - end_dir), math.cos(goal_yaw - end_dir)))
        return {
            'path_len': float(np.sum(np.hypot(*np.diff(xy, axis=0).T))),
            't_pred': pm.predict_travel_time(xy, lim, start_heading_error=start_err,
                                             goal_heading_error=goal_err),
            't_pred_no_rot': pm.predict_travel_time(xy, lim),
        }

    def test_10_drive(self) -> None:
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        plan = self.probe.subscribe('plan', Path, keep_messages=50)
        gt = self.probe.subscribe('ground_truth/odom', Odometry, keep_messages=4000)
        nav = actions.ActionCaller(self.probe, NavigateToPose, 'navigate_to_pose')
        self.assertTrue(nav.wait_server(self.timeout(300.0)), 'navigate_to_pose 서버 없음')
        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        self.wait_startup_still()
        results = []
        for (x, y, yaw), limit in zip(GOALS, goal_timeouts(GOALS)):
            goal = NavigateToPose.Goal()
            goal.pose = actions.pose_stamped(x, y, yaw)
            w0, t0 = self.probe.wall(), self.probe.now()
            status, _ = nav.call(goal, self.timeout(limit))
            actual = self.probe.now() - t0
            eta = self._eta(plan, gt, w0, yaw)
            err = (100.0 * abs(actual - eta['t_pred']) / actual
                   if eta['t_pred'] is not None and actual > 0 else math.nan)
            results.append({'goal': [x, y], 'status': status,
                            'time_s': cases.fmt(actual, 1),
                            'limit_s': round(limit, 1),
                            'path_len_m': cases.fmt(eta['path_len'], 2),
                            't_pred_s': cases.fmt(eta['t_pred'], 2),
                            't_pred_no_rot_s': cases.fmt(eta['t_pred_no_rot'], 2),
                            'eta_err_pct': cases.fmt(err, 2)})
            self.measure('goals', results)
        ok = [r['status'] == GoalStatus.STATUS_SUCCEEDED for r in results]
        self.check('goals reached', sum(ok), len(GOALS), all(ok))
        # 명세 151행 — 구간마다 15 % 이내여야 한다 (최댓값으로 판정).
        errs = [float(r['eta_err_pct']) for r in results
                if r['eta_err_pct'] not in ('', None) and math.isfinite(float(r['eta_err_pct']))]
        worst = max(errs, default=math.nan)
        self.measure('eta', {'per_goal_err_pct': [r['eta_err_pct'] for r in results],
                             'max_err_pct': cases.fmt(worst, 2),
                             'mean_err_pct': cases.fmt(sum(errs) / len(errs), 2) if errs else None,
                             'judged_goals': len(errs)})
        self.check(f'travel time prediction error (max of {len(errs)} goals)',
                   cases.fmt(worst, 2), ETA_ERR_MAX_PCT,
                   bool(errs) and worst <= ETA_ERR_MAX_PCT, '%')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX

    def test_cte(self) -> None:
        """cte.csv → 구간별 평균 |CTE| (직선 ≤ 0.05 m, 곡선 ≤ 0.10 m)."""
        result = evaluation.analyze(CTX.log_dir)
        rows = evaluation.gated_rows(result, '경로 추종 CTE')
        CTX.record.measure('evaluation_cte_rows', rows)
        CTX.record.measure('evaluation_warnings', result['warnings'])
        for r in rows:
            CTX.record.check(f"CTE mean {r['segment']} (n={r['count']})", cases.fmt(r['value']),
                             r['threshold'], bool(r['passed']), 'm')
        self.assertEqual(len(rows), 2, f"직선·곡선 판정 행이 모두 있어야 한다: {result['warnings']}")
        self.assertTrue(all(r['passed'] for r in rows), f'CTE 기준 초과: {rows}')
