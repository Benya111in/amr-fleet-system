"""
시나리오 08: 동적 장애물 회피 (명세 4.7 "30회 충돌 0건, 경로 이탈 1 m 이내").

스택(system 프로필): Gazebo(동적 장애물 6개 — worker_crossing 이 y = −7 을 1.0 m/s 로 횡단) +
localization + navigation(DWA) + perception(obstacle_tracker_node 칼만 추적·TTC, safety_node).
시행 30회: A(−4, −5) ↔ B(4, −9) 왕복 navigate_to_pose — 경로가 작업자 횡단선과 교차한다.
  충돌   GT 풋프린트 가장자리 ~ actor 위치(SDF <trajectory> 를 sim time 으로 보간, 반지름 0.25 m)
         최소 여유 ≤ 0 이면 충돌 (actor 는 충돌체가 없어 물리 충돌이 나지 않으므로 기하로 판정)
  이탈   목표 직후 첫 plan(전역 경로) 대비 GT 최대 수직 거리
판정: 충돌 0/30, 최대 이탈 ≤ 1.0 m, 도달 ≥ 29/30. 로그 avoidance.csv.
"""

import math

from amr_itest import actions, cases, catalog, config, metrics, worldmap
from amr_itest import kinematics as km
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry, Path
import numpy as np
import pytest

CTX = Context(catalog.get(8))

TRIALS = 30
POINTS = [(-4.0, -5.0, -0.46), (4.0, -9.0, 2.68)]
ACTOR_RADIUS = 0.25        # [m] 작업자 외접원
DEVIATION_MAX = 1.0
TRIAL_TIMEOUT_S = 180.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements()
                + [req.executable('amr_perception', 'obstacle_tracker_node', '동적 장애물 추적'),
                   req.package('nav2_msgs')], 'dynamic obstacle avoidance')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.system(use_localization=True, use_navigation=True, use_perception=True)
    return stack.launch_description(), {'stack': stack}


class TestDynamicObstacles(cases.ProbeCase):
    """30회 왕복 — 충돌·이탈·도달."""

    CTX = CTX

    def test_10_trials(self) -> None:
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        rp = config.robot_params()
        length = config.get(rp, 'robot.footprint_length', 0.60)
        width = config.get(rp, 'robot.footprint_width', 0.40)
        share = req.share_dir('amr_simulation')
        actors = worldmap.actor_tracks(share / 'worlds' / self.settings.world)
        gt = self.probe.subscribe('ground_truth/odom', Odometry, keep_messages=20000)
        plan = self.probe.subscribe('plan', Path, keep_messages=50)
        nav = actions.ActionCaller(self.probe, NavigateToPose, 'navigate_to_pose')
        self.assertTrue(nav.wait_server(self.timeout(300.0)), 'navigate_to_pose 서버 없음')
        rows, collisions, reached = [], 0, 0
        worst_dev = 0.0
        for trial in range(TRIALS):
            x, y, yaw = POINTS[(trial + 1) % 2]
            goal = NavigateToPose.Goal()
            goal.pose = actions.pose_stamped(x, y, yaw)
            w0, t0 = self.probe.wall(), self.probe.now()
            status, _ = nav.call(goal, self.timeout(TRIAL_TIMEOUT_S))
            ok = status == GoalStatus.STATUS_SUCCEEDED
            reached += int(ok)
            first_plan = next((m for _, m in plan.messages(w0) if m.poses), None)
            path = (np.array([[p.pose.position.x, p.pose.position.y] for p in first_plan.poses])
                    if first_plan is not None else np.zeros((0, 2)))
            min_clear, max_dev = math.inf, 0.0
            for _, m in gt.messages(w0):
                s = metrics.sample_from_odom(m)
                for a in actors:
                    ax, ay = a.position(s.t)
                    pts = km.world_to_base(np.array([[ax, ay]]), s.x, s.y, s.yaw)
                    min_clear = min(min_clear,
                                    km.footprint_clearance(pts, length, width) - ACTOR_RADIUS)
                if len(path):
                    max_dev = max(max_dev, worldmap.polyline_distance(path, s.x, s.y))
            collisions += int(min_clear <= 0.0)
            worst_dev = max(worst_dev, max_dev)
            rows.append([trial, 'crossing', 1.0, min_clear, max_dev, int(ok),
                         self.probe.now() - t0])
        self.ctx.record.write_csv('avoidance.csv', ['trial', 'pattern', 'speed_mps',
                                                    'min_clearance_m', 'max_deviation_m',
                                                    'reached', 'time_s'], rows)
        self.check('collisions', collisions, 0, collisions == 0)
        self.check('max path deviation', worst_dev, DEVIATION_MAX, worst_dev <= DEVIATION_MAX,
                   'm')
        self.check('goals reached', reached, f'>= {TRIALS - 1}', reached >= TRIALS - 1)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
