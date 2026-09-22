"""
시나리오 11: BT 에러 복구 (명세 4.8 "3가지 이상 에러 상황 자동 복구").

스택(system 프로필): Gazebo + localization + navigation + perception + behavior
(task_executor_node BT: 대기-이동-인식-작업-복귀, 복구 서브트리 spin/backup/wait/clear costmap).
에러 주입 3종 — 작업(assign_task)을 주고 executor/phase 가 moving 이 된 뒤 주입한다.
  estop      E-stop 3 s (estop true → false → safety/reset_estop) → BT 가 IsEstopClear 로 멈췄다 재개
  blocked    경로 앞 2 m 에 1 × 3 m 박스 생성(ign create) → 계획/추종 실패 → 복구·재계획, 20 s 후 제거
  lost       로봇을 옆 통로로 순간 이동 → localization/lost → 재초기화·spin → 작업 재개
판정: 3종 모두 task_status 최종 COMPLETED, 주입 ~ 완료 ≤ 180 s. 로그 recovery.csv.
"""

import math

from amr_itest import actions, cases, catalog, gz, metrics
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

CTX = Context(catalog.get(11))

CASES = ('estop', 'blocked', 'lost')
# 작업 목표 (pickup → dropoff) — 통로 y = 0 을 따라 동서로 긴 이동
PICKUP = (15.0, 0.0, 0.0)
DROPOFF = (-15.0, 0.0, math.pi)
RECOVERY_MAX_S = 180.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_behavior=True)
                + [req.executable('amr_behavior', 'task_executor_node', 'BT 작업 실행기')],
                'BT recovery')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.system(use_localization=True, use_navigation=True, use_perception=True,
                 use_behavior=True)
    return stack.launch_description(), {'stack': stack}


class TestBtRecovery(cases.ProbeCase):
    """에러 3종 주입 → 작업 완료."""

    CTX = CTX

    def _inject(self, case: str, gt) -> None:
        world = self.settings.world.rsplit('.', 1)[0]
        if case == 'estop':
            self.probe.publish('estop', Bool(data=True), 'latched')
            self.probe.sleep_ros(3.0, self.timeout(60.0))
            self.probe.publish('estop', Bool(data=False), 'latched')
            res = self.probe.call(Trigger, 'safety/reset_estop', Trigger.Request(),
                                  self.timeout(5.0))
            self.assertTrue(res is not None and res.success, f'reset_estop 실패 {res}')
        elif case == 'blocked':
            s = metrics.sample_from_odom(gt.last())
            bx, by = s.x + 2.0 * math.cos(s.yaw), s.y + 2.0 * math.sin(s.yaw)
            ok, out = gz.spawn_box(world, 'itest_block', bx, by, 1.0, 3.0, 1.0)
            self.assertTrue(ok, f'장애물 생성 실패: {out}')
            self.probe.sleep_ros(20.0, self.timeout(600.0))
            gz.remove(world, 'itest_block')
        elif case == 'lost':
            s = metrics.sample_from_odom(gt.last())
            ok, out = gz.set_pose(world, self.settings.robot, s.x, s.y + 6.0, s.yaw)
            self.assertTrue(ok, f'set_pose 실패: {out}')

    def test_10_recovery(self) -> None:
        from amr_msgs.msg import Task
        from amr_msgs.srv import AssignTask
        gt = self.probe.subscribe('ground_truth/odom', Odometry)
        status = self.probe.subscribe('task_status', Task, keep_messages=500)
        phase = self.probe.subscribe('executor/phase', String, keep_messages=2000)
        self.require_topic(gt, 5, 300.0)
        self.assertTrue(self.probe.service_available(AssignTask, 'assign_task',
                                                     self.timeout(300.0)))
        rows, failed = [], []
        for i, case in enumerate(CASES):
            task = Task()
            task.task_id = f'itest_bt_{case}'
            task.robot_id = self.settings.robot
            task.priority = 100
            task.item_type = 'small'
            task.item_mass = 2.0
            task.pickup_pose = actions.pose_stamped(*PICKUP)
            task.dropoff_pose = actions.pose_stamped(*DROPOFF)
            task.header.stamp = self.probe.node.get_clock().now().to_msg()
            w0 = self.probe.wall()
            res = self.probe.call(AssignTask, 'assign_task', AssignTask.Request(task=task),
                                  self.timeout(10.0))
            self.assertTrue(res is not None and res.success, f'{case}: assign_task 거절 {res}')
            moving = self.probe.wait_until(
                lambda: any(m.data == 'moving' for _, m in phase.messages(w0)),
                self.timeout(120.0))
            self.assertTrue(moving, f'{case}: executor/phase 가 moving 이 되지 않음')
            t_inject = self.probe.now()
            self._inject(case, gt)

            def final(tid=task.task_id):
                for _, m in reversed(status.messages(w0)):
                    if m.task_id == tid and m.status in (Task.STATUS_COMPLETED,
                                                         Task.STATUS_FAILED):
                        return m.status
                return None
            self.probe.wait_until(lambda: final() is not None,
                                  self.timeout(RECOVERY_MAX_S * 10), 0.2)
            t_done = self.probe.now()
            phases = [m.data for _, m in phase.messages(w0)]
            compact = [p for j, p in enumerate(phases) if j == 0 or p != phases[j - 1]]
            ok = final() == Task.STATUS_COMPLETED and t_done - t_inject <= RECOVERY_MAX_S
            rows.append([case, t_inject, t_done, t_done - t_inject, final(), '>'.join(compact)])
            if not ok:
                failed.append(case)
        self.ctx.record.write_csv('recovery.csv', ['case', 'injected_at', 'recovered_at',
                                                   'recovery_s', 'final_status', 'phases'], rows)
        self.check('recovered error cases', len(CASES) - len(failed), len(CASES), not failed,
                   '', f'실패 {failed}')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
