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

from amr_itest import actions, cases, catalog, evaluation
from amr_itest import requirements as req
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

    def test_10_drive(self) -> None:
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        nav = actions.ActionCaller(self.probe, NavigateToPose, 'navigate_to_pose')
        self.assertTrue(nav.wait_server(self.timeout(300.0)), 'navigate_to_pose 서버 없음')
        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        self.wait_startup_still()
        results = []
        for (x, y, yaw), limit in zip(GOALS, goal_timeouts(GOALS)):
            goal = NavigateToPose.Goal()
            goal.pose = actions.pose_stamped(x, y, yaw)
            t0 = self.probe.now()
            status, _ = nav.call(goal, self.timeout(limit))
            results.append({'goal': [x, y], 'status': status,
                            'time_s': cases.fmt(self.probe.now() - t0, 1),
                            'limit_s': round(limit, 1)})
            self.measure('goals', results)
        ok = [r['status'] == GoalStatus.STATUS_SUCCEEDED for r in results]
        self.check('goals reached', sum(ok), len(GOALS), all(ok))


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
