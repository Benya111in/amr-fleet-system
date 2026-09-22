"""
시나리오 10: 정밀 도킹 (명세 4.8 "위치 오차 2 cm, 각도 오차 1° 이내").

스택(system 프로필): Gazebo + localization + navigation + perception(ArUco 도킹 마커 검출) +
behavior(docking_server_node — amr_msgs/action/Dock).
시행 10회 (dock_1 · dock_a 번갈아): 로봇을 접근 자세(도크 패드에서 마커 반대쪽으로 1.5 m,
±0.2 m · ±10° 난수 오프셋)로 순간 이동 → Dock 목표(dock_id, approach_pose, max_retries 3).
판정: 결과 success 이고 final_position_error ≤ 0.02 m, final_angle_error ≤ 1° — 10/10.
GT(ground_truth/odom) 최종 자세와 도크 패드 중심의 차이도 함께 기록한다 (도킹 기준점이 패드 중심과
다를 수 있어 판정에는 쓰지 않는 참고값).
"""

import math

from amr_itest import actions, cases, catalog, gz, metrics, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import numpy as np
import pytest

CTX = Context(catalog.get(10))

TRIALS = 10
DOCKS = ('dock_1', 'dock_a')
POS_TOL = 0.02
ANG_TOL = math.radians(1.0)
APPROACH_DIST = 1.5
DOCK_TIMEOUT_S = 180.0


def approach_pose(world_sdf, dock: str):
    """패드·마커 자세 → (접근 x, y, yaw(마커를 향함), 패드 x, y)."""
    pad = worldmap.include_pose(world_sdf, f'{dock}_pad')
    marker = worldmap.include_pose(world_sdf, f'{dock}_marker')
    if pad is None or marker is None:
        return None
    dx, dy = pad[0] - marker[0], pad[1] - marker[1]
    n = math.hypot(dx, dy) or 1.0
    ax, ay = pad[0] + APPROACH_DIST * dx / n, pad[1] + APPROACH_DIST * dy / n
    return ax, ay, math.atan2(-dy, -dx), pad[0], pad[1]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_behavior=True)
                + [req.executable('amr_behavior', 'docking_server_node', 'Dock 액션 서버')],
                'docking')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.system(use_localization=True, use_navigation=True, use_perception=True,
                 use_behavior=True)
    return stack.launch_description(), {'stack': stack}


class TestDocking(cases.ProbeCase):
    """도킹 10회 정밀도."""

    CTX = CTX

    def test_10_docking(self) -> None:
        from amr_msgs.action import Dock
        share = req.share_dir('amr_simulation')
        world_sdf = share / 'worlds' / self.settings.world
        world = self.settings.world.rsplit('.', 1)[0]
        gt = self.probe.subscribe('ground_truth/odom', Odometry)
        dock = actions.ActionCaller(self.probe, Dock, 'dock')
        self.assertTrue(dock.wait_server(self.timeout(300.0)), 'dock 액션 서버 없음')
        rng = np.random.default_rng(self.settings.seed)
        rows, good = [], 0
        for trial in range(TRIALS):
            dock_id = DOCKS[trial % len(DOCKS)]
            ap = approach_pose(world_sdf, dock_id)
            self.assertIsNotNone(ap, f'{dock_id}: 월드에 패드/마커 모델 없음')
            ax, ay, ayaw, px, py = ap
            ox, oy = rng.uniform(-0.2, 0.2, 2)
            oyaw = math.radians(rng.uniform(-10.0, 10.0))
            ok, out = gz.set_pose(world, self.settings.robot, ax + ox, ay + oy, ayaw + oyaw)
            self.assertTrue(ok, f'set_pose 실패: {out}')
            self.assertTrue(self.probe.sleep_ros(3.0, self.timeout(120.0)))
            goal = Dock.Goal()
            goal.dock_id = dock_id
            goal.approach_pose = actions.pose_stamped(ax, ay, ayaw)
            goal.max_retries = 3
            t0 = self.probe.now()
            _, result = dock.call(goal, self.timeout(DOCK_TIMEOUT_S))
            g = metrics.sample_from_odom(gt.last())
            success = (result is not None and result.success
                       and result.final_position_error <= POS_TOL
                       and abs(result.final_angle_error) <= ANG_TOL)
            good += int(success)
            rows.append([trial, dock_id, int(bool(result and result.success)),
                         result.attempts_used if result else -1,
                         result.final_position_error if result else math.nan,
                         math.degrees(result.final_angle_error) if result else math.nan,
                         math.hypot(g.x - px, g.y - py),
                         math.degrees(abs(metrics.wrap(g.yaw - ayaw))),
                         self.probe.now() - t0])
        self.ctx.record.write_csv('docking.csv', [
            'trial', 'dock_id', 'success', 'attempts', 'pos_err_m', 'ang_err_deg',
            'gt_pos_err_m', 'gt_ang_err_deg', 'time_s'], rows)
        self.check('docking within 2 cm / 1 deg', good, TRIALS, good == TRIALS)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
