"""
시나리오 14: 4시간 연속 운용 (명세 4.10 "연속 4시간 운용 테스트에서 시스템 안정성 검증").

장시간 시나리오라 기본 실행에서 빠진다 (scripts/run_integration.sh --include-long 또는 14 를 지정).
기간은 ITEST_SOAK_HOURS (기본 4.0, 스모크 확인은 0.1 등으로 줄인다).
스택(system 프로필): Gazebo + localization + navigation + perception + behavior.
작업(assign_task)을 끝날 때마다 다시 넣으며(동서 통로 왕복) 60 s 마다 ROS 프로세스(--ros-args)의
RSS 를 잰다.
판정: 크래시 0 (시작 때 있던 프로세스가 사라지지 않음), 프로세스별 RSS 기울기 ≤ 5 MB/h
(첫 10 분 워밍업 제외 최소제곱), 작업 실패율 ≤ 2 %. 로그 memory.csv, soak.json.
"""

import json
import math
import os

from amr_itest import actions, cases, catalog, procmon
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
import pytest

CTX = Context(catalog.get(14))

HOURS = float(os.environ.get('ITEST_SOAK_HOURS', '4.0'))
SAMPLE_S = 60.0
WARMUP_S = 600.0
LEAK_MAX_MB_H = 5.0
FAIL_RATE_MAX = 0.02
ROUTE = [((15.0, 0.0, 0.0), (-15.0, 0.0, math.pi)), ((-15.0, 6.0, math.pi), (15.0, 6.0, 0.0))]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_behavior=True)
                + [req.executable('amr_behavior', 'task_executor_node', '작업 반복 투입')],
                'soak')
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.system(use_localization=True, use_navigation=True, use_perception=True,
                 use_behavior=True)
    return stack.launch_description(), {'stack': stack}


class TestSoak(cases.ProbeCase):
    """장시간 운용: 크래시·메모리 증가·작업 실패율."""

    CTX = CTX

    def test_10_soak(self) -> None:
        from amr_msgs.msg import Task
        from amr_msgs.srv import AssignTask
        status = self.probe.subscribe('task_status', Task, keep_messages=5000)
        self.assertTrue(self.probe.service_available(AssignTask, 'assign_task',
                                                     self.timeout(600.0)))
        self.probe.sleep_ros(30.0, self.timeout(600.0))
        procs = procmon.ros_processes()
        self.measure('processes', procs)
        start = self.probe.wall()
        mem_rows, n_tasks, current = [], 0, None
        next_sample = start
        while self.probe.wall() - start < HOURS * 3600.0:
            final = {m.task_id: m.status for _, m in status.messages()}
            if current is None or final.get(current) in (Task.STATUS_COMPLETED,
                                                         Task.STATUS_FAILED):
                pickup, dropoff = ROUTE[n_tasks % len(ROUTE)]
                task = Task(task_id=f'itest_soak_{n_tasks:05d}', robot_id=self.settings.robot,
                            priority=50, item_type='small', item_mass=2.0)
                task.pickup_pose = actions.pose_stamped(*pickup)
                task.dropoff_pose = actions.pose_stamped(*dropoff)
                res = self.probe.call(AssignTask, 'assign_task', AssignTask.Request(task=task),
                                      self.timeout(10.0))
                if res is not None and res.success:
                    current = task.task_id
                    n_tasks += 1
            if self.probe.wall() >= next_sample:
                t = self.probe.wall() - start
                for pid, name in procs.items():
                    mem_rows.append([t, name, pid, procmon.rss_mb(pid)])
                next_sample += SAMPLE_S
            self.probe.sleep_ros(1.0, self.timeout(60.0))
        self.ctx.record.write_csv('memory.csv', ['time', 'process', 'pid', 'rss_mb'], mem_rows)
        final = {m.task_id: m.status for _, m in status.messages()}
        failed_tasks = sum(1 for s in final.values() if s == Task.STATUS_FAILED)
        crashed = sorted({name for _, name, _, rss in mem_rows if rss is None})
        slopes = {}
        for pid, name in procs.items():
            pts = [(t, rss) for t, _, p, rss in mem_rows
                   if p == pid and rss is not None and t >= WARMUP_S]
            slopes[f'{name}[{pid}]'] = procmon.slope_per_hour(
                [p[0] for p in pts], [p[1] for p in pts])
        leaks = {k: v for k, v in slopes.items() if math.isfinite(v) and v > LEAK_MAX_MB_H}
        fail_rate = failed_tasks / max(n_tasks, 1)
        self.ctx.record.write_text('soak.json', json.dumps({
            'hours': HOURS, 'tasks': n_tasks, 'failed': failed_tasks, 'crashed': crashed,
            'rss_slope_mb_per_h': slopes}, ensure_ascii=False, indent=2))
        self.check('crashed processes', crashed, [], not crashed)
        self.check('memory growth > 5 MB/h', sorted(leaks), [], not leaks, '', str(leaks))
        self.check('task failure rate', fail_rate, FAIL_RATE_MAX, fail_rate <= FAIL_RATE_MAX)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
