"""
시나리오 07: 경로 추종 CTE (명세 4.5 "CTE 직선 5 cm / 곡선 10 cm", 4.10 CTE 산출 절차).

스택(system 프로필): Gazebo + localization + navigation(직접 구현 DWA 컨트롤러 + A* 플래너).
주행: navigate_to_pose 로 통로 직선(y = 0 → x = 15), 랙 끝 U 턴(곡선), 반대 통로 직선, 다시 U 턴.
측정: amr_evaluation cte_logger 가 plan(계획 경로) 과 ground_truth/odom 의 수직 거리를 곡률로
직선/곡선 구간으로 나눠 cte.csv 에 기록 → post-shutdown 에서 analyze 로 구간별 평균 |CTE| 판정.
"""

import math

from amr_itest import actions, cases, catalog, evaluation
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
import pytest

CTX = Context(catalog.get(7))

# (x, y, yaw) 목표 — 통로 직선 + 랙 끝(x ≈ ±20) U 턴 곡선
GOALS = [(15.0, 0.0, 0.0), (10.0, 6.0, math.pi), (-15.0, 6.0, math.pi), (-10.0, -6.0, 0.0),
         (0.0, 0.0, 0.0)]
GOAL_TIMEOUT_S = 240.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements()
                + [req.executable('amr_evaluation', 'cte_logger', 'CTE 측정 (명세 4.10 절차)'),
                   req.package('nav2_msgs')], 'path tracking')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.system(use_localization=True, use_navigation=True, use_perception=True)
    stack.eval_logger('cte_logger', {'path_topic': 'plan', 'gt_topic': 'ground_truth/odom'})
    return stack.launch_description(), {'stack': stack}


class TestPathTracking(cases.ProbeCase):
    """목표 순회 (측정은 cte_logger)."""

    CTX = CTX

    def test_10_drive(self) -> None:
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        nav = actions.ActionCaller(self.probe, NavigateToPose, 'navigate_to_pose')
        self.assertTrue(nav.wait_server(self.timeout(300.0)), 'navigate_to_pose 서버 없음')
        results = []
        for x, y, yaw in GOALS:
            goal = NavigateToPose.Goal()
            goal.pose = actions.pose_stamped(x, y, yaw)
            status, _ = nav.call(goal, self.timeout(GOAL_TIMEOUT_S))
            results.append(status == GoalStatus.STATUS_SUCCEEDED)
        self.measure('goals_succeeded', results)
        self.check('goals reached', sum(results), len(GOALS), all(results))


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX

    def test_cte(self) -> None:
        """cte.csv → 구간별 평균 |CTE| (직선 ≤ 0.05 m, 곡선 ≤ 0.10 m)."""
        result = evaluation.analyze(CTX.log_dir)
        rows = evaluation.gated_rows(result, '경로 추종 CTE')
        CTX.record.measure('evaluation_cte_rows', rows)
        for r in rows:
            CTX.record.check(f"CTE mean {r['segment']} (n={r['count']})", cases.fmt(r['value']),
                             r['threshold'], bool(r['passed']), 'm')
        self.assertEqual(len(rows), 2, f"직선·곡선 판정 행이 모두 있어야 한다: {result['warnings']}")
        self.assertTrue(all(r['passed'] for r in rows), f'CTE 기준 초과: {rows}')
