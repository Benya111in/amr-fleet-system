"""
시나리오 12: 5대 동시 운용 교착 (명세 4.9 교착 탐지·해소, 4.10 "5대 운용 시 CPU 80 % 이하").

스택: amr_fleet/launch/multi_robot.launch.py (Gazebo 1개 + 로봇 5대 system 스택 + fleet_manager_node
+ traffic_manager_node, multi_robot.md). 작업 5건을 /fleet/assign_task 로 동시에 넣는다 — 그중 두
건은 좁은 통로(0, −10; 순폭 0.60 m)를 반대 방향으로 지나게 해 정면 대치를 만든다(sequences.md §3).
판정
  교착   /fleet/traffic_events 에 교착 탐지(ERROR "deadlock") ≥ 1 이고 탐지마다 해소(OK "resolved")
  작업   5건 모두 COMPLETED (/fleet/task_events), 전체 ≤ 20 분
  CPU    시스템 프로세스(--ros-args) CPU 합 평균 ≤ 80 % (procmon.ProcessCpuSampler — 공유 호스트의
         외부 부하 제외). 호스트 전체 사용률(amr_evaluation cpu_sampler 방식)은 참고로 함께 기록
로그 traffic.csv, cpu.csv.
"""

import math

from amr_itest import actions, cases, catalog, procmon
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
import pytest

CTX = Context(catalog.get(12))

ROBOTS = 5
CPU_MAX = 80.0
RUN_MAX_S = 1200.0
# (pickup, dropoff) — 첫 두 건은 좁은 통로를 반대 방향으로 통과
TASKS = [((0.0, -7.0, -math.pi / 2), (0.0, -13.0, -math.pi / 2)),
         ((0.0, -13.0, math.pi / 2), (0.0, -7.0, math.pi / 2)),
         ((15.0, 0.0, 0.0), (-15.0, 0.0, math.pi)),
         ((-15.0, 6.0, math.pi), (15.0, 6.0, 0.0)),
         ((15.0, -6.0, 0.0), (-15.0, -6.0, math.pi))]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_behavior=True)
                + [req.launch('amr_fleet', 'multi_robot.launch.py', '5대 기동 (multi_robot.md)'),
                   req.executable('amr_fleet', 'fleet_manager_node', '작업 할당'),
                   req.executable('amr_fleet', 'traffic_manager_node', '교착 탐지·해소')],
                'multi-robot deadlock')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.include('amr_fleet', 'multi_robot.launch.py',
                  {'num_robots': str(ROBOTS), 'use_sim_time': 'true', 'headless': 'true'})
    stack.components.update({'fleet': 'real', 'robots': str(ROBOTS)})
    return stack.launch_description(), {'stack': stack}


class TestMultiRobotDeadlock(cases.ProbeCase):
    """교착 탐지·해소 + 작업 완료 + CPU."""

    CTX = CTX

    def test_10_deadlock(self) -> None:
        from amr_msgs.msg import Task
        from amr_msgs.srv import AssignTask
        from diagnostic_msgs.msg import DiagnosticArray
        events = self.probe.subscribe('/fleet/task_events', Task, keep_messages=2000)
        traffic = self.probe.subscribe('/fleet/traffic_events', DiagnosticArray,
                                       keep_messages=2000)
        self.assertTrue(self.probe.service_available(AssignTask, '/fleet/assign_task',
                                                     self.timeout(600.0)),
                        '/fleet/assign_task 없음')
        ids = []
        for i, (pickup, dropoff) in enumerate(TASKS):
            task = Task()
            task.task_id = f'itest_mr_{i}'
            task.priority = 100 - i
            task.item_type = 'small'
            task.item_mass = 2.0
            task.pickup_pose = actions.pose_stamped(*pickup)
            task.dropoff_pose = actions.pose_stamped(*dropoff)
            res = self.probe.call(AssignTask, '/fleet/assign_task',
                                  AssignTask.Request(task=task), self.timeout(30.0))
            self.assertTrue(res is not None and res.success, f'작업 {i} 거절 {res}')
            ids.append(task.task_id)
        pids = list(procmon.ros_processes())
        own, host = procmon.ProcessCpuSampler(pids), procmon.CpuSampler()
        cpu_rows = []
        t0 = self.probe.now()

        def done() -> bool:
            final = {m.task_id: m.status for _, m in events.messages()}
            return all(final.get(t) in (Task.STATUS_COMPLETED, Task.STATUS_FAILED) for t in ids)

        while not done() and self.probe.now() - t0 < RUN_MAX_S:
            self.probe.sleep_ros(1.0, self.timeout(60.0))
            cpu_rows.append([self.probe.now(), own.sample(), host.sample()])
        final = {m.task_id: m.status for _, m in events.messages()}
        # DiagnosticStatus.level 은 byte (파이썬 bytes 1 개) — 정수로 바꿔 기록
        levels = [(t, s.level[0] if isinstance(s.level, bytes) else int(s.level), s.message)
                  for t, m in traffic.messages() for s in m.status]
        detected = [lv for lv in levels if 'deadlock' in lv[2].lower()]
        resolved = [lv for lv in levels if 'resolved' in lv[2].lower()]
        self.ctx.record.write_csv('traffic.csv', ['recv_time', 'level', 'message'],
                                  [list(lv) for lv in levels])
        self.ctx.record.write_csv('cpu.csv', ['time', 'system_cpu_percent', 'host_cpu_percent'],
                                  cpu_rows)
        own_mean = sum(r[1] for r in cpu_rows) / max(len(cpu_rows), 1)
        self.measure('cpu', {'system_mean': cases.fmt(own_mean, 1),
                             'host_mean': cases.fmt(sum(r[2] for r in cpu_rows)
                                                    / max(len(cpu_rows), 1), 1),
                             'processes': len(pids)})
        completed = sum(1 for t in ids if final.get(t) == Task.STATUS_COMPLETED)
        self.check('deadlocks detected', len(detected), '>= 1', len(detected) >= 1)
        self.check('deadlocks resolved', len(resolved), f'>= {len(detected)}',
                   len(resolved) >= len(detected) > 0)
        self.check('tasks completed', completed, len(ids), completed == len(ids))
        self.check('system CPU mean', own_mean, CPU_MAX, own_mean <= CPU_MAX, '%')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
