"""
시나리오 05: Kidnapped Robot 복구 (명세 4.3 "납치 후 제자리 회전 등으로 스스로 위치 복구").

스택(system 프로필): Gazebo + localization(amcl + 이중 EKF + kidnap_monitor_node) + navigation.
시행 5회: Gazebo set_pose 로 로봇을 다른 통로로 순간 이동(ign service /world/<w>/set_pose) →
  감지  localization/lost (Bool, latched) 가 true 로 바뀔 때까지 ≤ 5 s
  복구  odometry/filtered_map 과 ground_truth/odom 의 위치 오차가 0.10 m 이하로 3 s 유지 ≤ 60 s
        (kidnap_monitor_node 가 reinitialize_global_localization + spin 으로 스스로 복구)
판정: 5회 모두 감지·복구. 로그 kidnap.csv.
"""

import math

from amr_itest import cases, catalog, gz, metrics
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import Bool

CTX = Context(catalog.get(5))

# 순간 이동 목표 (x, y, yaw) — 서로 다른 통로의 자유 지점 (gen_warehouse_world.py 좌표계)
TARGETS = [(12.0, 6.0, math.pi / 2), (-12.0, -6.0, 0.0), (21.0, 0.0, math.pi),
           (-21.0, 13.0, -math.pi / 2), (0.0, 0.0, 0.0)]
DETECT_MAX_S = 5.0
RECOVER_MAX_S = 60.0
RECOVER_TOL_M = 0.10
HOLD_S = 3.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_perception=False)
                + [req.executable('amr_localization', 'kidnap_monitor_node',
                                  'localization/lost + 자가 복구'),
                   req.executable('nav2_amcl', 'amcl', 'AMCL 전역 재초기화')], 'kidnapped robot')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.system(use_localization=True, use_navigation=True, use_perception=False)
    return stack.launch_description(), {'stack': stack}


class TestKidnappedRobot(cases.ProbeCase):
    """납치 5회 → 감지·복구 시간."""

    CTX = CTX

    def _error(self, gt, est) -> float:
        g, e = gt.last(), est.last()
        if g is None or e is None:
            return math.inf
        a, b = metrics.sample_from_odom(g), metrics.sample_from_odom(e)
        return math.hypot(a.x - b.x, a.y - b.y)

    def _held(self, gt, est, hold: float, timeout: float) -> bool:
        state = {'since': None}

        def ok() -> bool:
            now = self.probe.now()
            if self._error(gt, est) > RECOVER_TOL_M:
                state['since'] = None
                return False
            state['since'] = state['since'] if state['since'] is not None else now
            return now - state['since'] >= hold
        return self.probe.wait_until(ok, timeout, 0.1)

    def test_10_kidnap(self) -> None:
        gt = self.probe.subscribe('ground_truth/odom', Odometry)
        est = self.probe.subscribe('odometry/filtered_map', Odometry)
        lost = self.probe.subscribe('localization/lost', Bool, 'latched')
        self.require_topic(est, 5, 300.0, 'odometry/filtered_map')
        self.assertTrue(self._held(gt, est, HOLD_S, self.timeout(120.0)),
                        '초기 위치 추정이 수렴하지 않음')
        world = self.settings.world.rsplit('.', 1)[0]
        rows, failed = [], []
        for trial, (x, y, yaw) in enumerate(TARGETS):
            before = metrics.sample_from_odom(gt.last())
            ok, out = gz.set_pose(world, self.settings.robot, x, y, yaw)
            self.assertTrue(ok, f'trial {trial}: set_pose 실패: {out}')
            t0 = self.probe.now()
            detected = self.probe.wait_until(
                lambda: lost.last() is not None and lost.last().data,
                self.timeout(DETECT_MAX_S * 10))
            t_detect = self.probe.now() - t0
            recovered = self._held(gt, est, HOLD_S, self.timeout(RECOVER_MAX_S * 10))
            t_recover = self.probe.now() - t0 - HOLD_S
            good = (detected and t_detect <= DETECT_MAX_S and recovered
                    and t_recover <= RECOVER_MAX_S)
            rows.append([trial, before.x, before.y, x, y, t_detect, t_recover,
                         self._error(gt, est), int(good)])
            if not good:
                failed.append(trial)
        self.ctx.record.write_csv('kidnap.csv', ['trial', 'from_x', 'from_y', 'to_x', 'to_y',
                                                 'detect_s', 'recover_s', 'final_error_m', 'ok'],
                                  rows)
        self.check('kidnap recoveries', len(TARGETS) - len(failed), len(TARGETS), not failed,
                   '', f'실패 시행 {failed}')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
