"""
시나리오 06: 경로 계획 성공률 (명세 4.4 "임의 출발지-목적지 50쌍 성공률 98 % 이상").

스택(system 프로필): Gazebo + localization(지도 + AMCL + EKF) + navigation(planner_server 에
직접 구현한 A* 전역 플래너 플러그인).
쌍 생성: 월드 SDF 정적 구조물(worldmap.footprints)에서 0.6 m 이상 떨어진 자유 지점을 고정 시드로
100 개 뽑아 50 쌍 (ITEST_SEED). compute_path_to_pose(use_start=true) 액션으로 계획만 한다(주행 없음).
판정: 성공(SUCCEEDED + 경로 점 ≥ 2 + 목표 도달 0.1 m 이내 + 경로 점이 구조물 내부에 없음) ≥ 49/50.
로그 plans.csv (계획 시간·경로 길이·직선 거리 비).
"""

import math

from amr_itest import actions, cases, catalog, config, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
import numpy as np
import pytest

CTX = Context(catalog.get(6))

PAIRS = 50
SUCCESS_MIN = 0.98
MARGIN = 0.6               # [m] 구조물에서 떨어진 거리 (로봇 외접원 0.36 + 여유)
BOUNDS = (-29.0, -19.0, 29.0, 19.0)   # 60 × 40 m 벽 안쪽


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_perception=False) + [req.package('nav2_msgs')],
                'path planning')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.system(use_localization=True, use_navigation=True, use_perception=False)
    return stack.launch_description(), {'stack': stack}


class TestPathPlanning(cases.ProbeCase):
    """50 쌍 계획 성공률."""

    CTX = CTX

    def test_10_success_rate(self) -> None:
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import ComputePathToPose
        caller = actions.ActionCaller(self.probe, ComputePathToPose, 'compute_path_to_pose')
        self.assertTrue(caller.wait_server(self.timeout(300.0)), 'compute_path_to_pose 서버 없음')
        share = req.share_dir('amr_simulation')
        plane_z = (config.get(config.robot_params(), 'robot.base_link_height', 0.18)
                   + config.get(config.sensors(), 'lidar.extrinsic.z', 0.20))
        shapes = worldmap.footprints(share / 'worlds' / self.settings.world, share / 'models',
                                     plane_z)
        rng = np.random.default_rng(self.settings.seed)
        pts = worldmap.sample_free(shapes, BOUNDS, MARGIN, 2 * PAIRS, rng)
        self.assertEqual(len(pts), 2 * PAIRS, '자유 지점 표본 부족')
        rows, ok_count = [], 0
        for i in range(PAIRS):
            (sx, sy), (gx, gy) = pts[2 * i], pts[2 * i + 1]
            goal = ComputePathToPose.Goal()
            goal.start = actions.pose_stamped(sx, sy)
            goal.goal = actions.pose_stamped(gx, gy)
            goal.use_start = True
            t0 = self.probe.wall()
            status, result = caller.call(goal, self.timeout(30.0))
            plan_ms = (self.probe.wall() - t0) * 1e3
            path = (np.array([[p.pose.position.x, p.pose.position.y]
                              for p in result.path.poses]) if result is not None
                    else np.zeros((0, 2)))
            length = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))) \
                if len(path) > 1 else 0.0
            free = bool(len(path)) and not any(
                bool(np.any(s.contains(path[:, 0], path[:, 1]))) for s in shapes)
            reached = bool(len(path)) and math.hypot(path[-1, 0] - gx, path[-1, 1] - gy) <= 0.1
            success = status == GoalStatus.STATUS_SUCCEEDED and len(path) >= 2 and reached \
                and free
            ok_count += int(success)
            rows.append([i, sx, sy, gx, gy, int(success), plan_ms, length,
                         math.hypot(gx - sx, gy - sy), int(free)])
        self.ctx.record.write_csv('plans.csv', ['pair', 'start_x', 'start_y', 'goal_x', 'goal_y',
                                                'success', 'plan_ms', 'length_m', 'straight_m',
                                                'collision_free'], rows)
        rate = ok_count / PAIRS
        times = [r[6] for r in rows if r[5]]
        self.measure('plan_ms', {'mean': cases.fmt(float(np.mean(times)), 1) if times else None,
                                 'max': cases.fmt(float(np.max(times)), 1) if times else None})
        self.check('planning success rate', rate, SUCCESS_MIN, rate >= SUCCESS_MIN, '',
                   f'{ok_count}/{PAIRS}')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
