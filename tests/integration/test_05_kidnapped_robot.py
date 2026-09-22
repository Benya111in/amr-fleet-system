"""
시나리오 05: Kidnapped Robot 복구 (명세 4.3 "납치 후 제자리 회전 등으로 스스로 위치 복구").

스택(system 프로필): Gazebo + localization(amcl + scan_matcher + 이중 EKF + kidnap_monitor_node) +
navigation(behavior_server spin) + perception(safety_node = cmd_vel 의 유일한 발행자). 스폰 = 월드 원점 (명시).
준비: lifecycle(map_server·AMCL·Nav2) 활성, map ↔ 월드 항등 정합 확인(추정 = map, GT = 월드를 그대로 비교하는 근거),
IMU 바이어스 정지 표본, 제자리 한 바퀴(운용 중 납치 — 기동 후 한 번도 안 움직이면 AMCL 기준이 없다), 초기 수렴.
시행 5회: Gazebo set_pose 로 로봇을 다른 통로로 순간 이동(ign service /world/<w>/set_pose) →
  감지  순간 이동 뒤 받은 localization/lost 가 true (수신 시각 기준) ≤ 5 s
  복구  odometry/filtered_map 과 ground_truth/odom 의 위치 오차가 0.10 m 이하로 3 s 유지 ≤ 60 s
        (kidnap_monitor_node 가 전역 시드 탐색 + 회전 + set_pose 로 스스로 복구)
대기 상한은 판정 기준 + 여유 (감지 10 s, 복구 75 s) — 실패하는 시스템도 러너 상한 전에 측정값을 남긴다.
kidnap.csv 와 시행별 판정은 시행마다 갱신한다. 판정: 5회 모두 감지·복구.
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

SPAWN = (0.0, 0.0, 0.0)
# 순간 이동 목표 (x, y, yaw) — 서로 다른 통로의 자유 지점 (gen_warehouse_world.py 좌표계)
TARGETS = [(12.0, 6.0, math.pi / 2), (-12.0, -6.0, 0.0), (21.0, 0.0, math.pi),
           (-21.0, 13.0, -math.pi / 2), (0.0, 0.0, 0.0)]
DETECT_MAX_S = 5.0
RECOVER_MAX_S = 60.0
RECOVER_TOL_M = 0.10
HOLD_S = 3.0
DETECT_WAIT_S = DETECT_MAX_S + 5.0            # 판정 기준 + 여유
RECOVER_WAIT_S = RECOVER_MAX_S + HOLD_S + 12.0
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')
COLUMNS = ['trial', 'from_x', 'from_y', 'to_x', 'to_y', 'detect_s', 'recover_s',
           'final_error_m', 'ok']


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements()
                + [req.executable('amr_localization', 'kidnap_monitor_node',
                                  'localization/lost + 자가 복구'),
                   req.executable('nav2_amcl', 'amcl', 'AMCL')], 'kidnapped robot')
    stack = Stack(CTX, *CTX.select())
    # perception 포함: cmd_vel 의 유일한 발행자는 safety_node 다 (components.md §4.1). 빼면 복구 spin 이
    # cmd_vel_nav → velocity_profiler → cmd_vel_smoothed 에서 끊겨 로봇이 돌지 않는다
    stack.system(use_localization=True, use_navigation=True, use_perception=True, pose=SPAWN)
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

    def test_10_kidnap(self, stack) -> None:
        gt = self.probe.subscribe('ground_truth/odom', Odometry)
        est = self.probe.subscribe('odometry/filtered_map', Odometry)
        lost = self.probe.subscribe('localization/lost', Bool, 'latched', keep_messages=500)
        self.require_topic(est, 5, 300.0, 'odometry/filtered_map')
        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        self.wait_startup_still()
        self.warm_up_localization(stack.drive_topic, gt)     # 운용 중 납치 (추적 중인 위치 추정)
        converged = self._held(gt, est, HOLD_S, self.timeout(120.0))
        self.check('initial localization converged (error <= 0.10 m for 3 s)',
                   cases.fmt(self._error(gt, est)), RECOVER_TOL_M, converged, 'm')
        world = self.settings.world.rsplit('.', 1)[0]
        rows, failed = [], []
        for trial, (x, y, yaw) in enumerate(TARGETS):
            if not self.budget_for(DETECT_WAIT_S + RECOVER_WAIT_S + 15.0, f'trial {trial}'):
                failed += list(range(trial, len(TARGETS)))
                break
            before = metrics.sample_from_odom(gt.last())
            w0 = self.probe.wall()
            ok, out = gz.set_pose(world, self.settings.robot, x, y, yaw)
            self.assertTrue(ok, f'trial {trial}: set_pose 실패: {out}')
            t0 = self.probe.now()
            detected = self.probe.wait_until(
                lambda: any(m.data for _, m in lost.messages(w0)), self.timeout(DETECT_WAIT_S))
            t_detect = self.probe.now() - t0 if detected else math.nan
            recovered = self._held(gt, est, HOLD_S, self.timeout(RECOVER_WAIT_S))
            t_recover = self.probe.now() - t0 - HOLD_S if recovered else math.nan
            good = (detected and t_detect <= DETECT_MAX_S and recovered
                    and t_recover <= RECOVER_MAX_S)
            rows.append([trial, before.x, before.y, x, y, t_detect, t_recover,
                         self._error(gt, est), int(good)])
            self.ctx.record.write_csv('kidnap.csv', COLUMNS, rows)
            self.ctx.record.check(f'trial {trial} detect <= {DETECT_MAX_S} s, recover <= '
                                  f'{RECOVER_MAX_S} s', {'detect_s': cases.fmt(t_detect, 2),
                                                         'recover_s': cases.fmt(t_recover, 2)},
                                  {'detect_s': DETECT_MAX_S, 'recover_s': RECOVER_MAX_S}, good)
            if not good:
                failed.append(trial)
        self.check('kidnap recoveries', len(TARGETS) - len(failed), len(TARGETS), not failed,
                   '', f'실패 시행 {failed}')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
