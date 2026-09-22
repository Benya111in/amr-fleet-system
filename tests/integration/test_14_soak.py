"""
시나리오 14: 4시간 연속 운용 (명세 4.10 "연속 4시간 운용 테스트에서 시스템 안정성 검증").

장시간 시나리오라 기본 실행에서 빠진다 (scripts/run_integration.sh --include-long 또는 14 를 지정).
기간은 --soak-hours H (ITEST_SOAK_HOURS, 기본 4.0 — 스모크는 0.1 등). 러너 상한은 기간 + 30 분.
스택(system 프로필): Gazebo + localization + navigation + perception + behavior, 스폰 = 대기 자세 = (0, 0, 0).
작업(assign_task, 도크 staging 사이 왕복 — behavior.yaml)을 끝날 때마다 다시 넣고, 60 s 마다 시스템 프로세스
(ROS 노드 + Gazebo 서버)의 RSS 를 잰다. 작업 하나에 900 s 상한: 넘으면 실패(timeout)로 세고 다음 작업을 시도한다
(멈춘 실행기는 계속 busy 로 거절 → 완료 수가 늘지 않는다).
판정: 크래시 0 (시작 때 있던 프로세스가 사라지지 않음 + 실행 중 비정상 종료 0), 프로세스별 RSS 기울기 ≤ 5 MB/h
(첫 10 분 워밍업 제외 최소제곱), 작업 실패율(실패 + 시간 초과) ≤ 2 %, 완료 작업 ≥ 4 /h × 기간 (최소 1).
로그 memory.csv, tasks.csv (작업마다 갱신), soak.json.
"""

import json
import math
import os

from amr_itest import actions, cases, catalog, docks, procmon
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
import pytest
from std_msgs.msg import String

CTX = Context(catalog.get(14))

HOURS = float(os.environ.get('ITEST_SOAK_HOURS', '4.0'))
SAMPLE_S = 60.0
WARMUP_S = 600.0
LEAK_MAX_MB_H = 5.0
FAIL_RATE_MAX = 0.02
TASK_MAX_S = 900.0
MIN_TASKS_PER_H = 4.0
ROUTE = (('dock_1', 'dock_a'), ('dock_b', 'dock_2'))
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')
TASK_COLUMNS = ['task_id', 'assigned_at', 'ended_at', 'duration_s', 'status']


def min_completed(hours: float) -> int:
    """완료 작업 하한 = 4 /h × 기간 (내림, 최소 1)."""
    return max(1, int(math.floor(MIN_TASKS_PER_H * hours)))


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_behavior=True)
                + [req.executable('amr_behavior', 'task_executor_node', '작업 반복 투입'),
                   req.config('amr_behavior', 'behavior.yaml', '도크 표')], 'soak')
    stack = Stack(CTX, *CTX.select())
    stack.system(use_localization=True, use_navigation=True, use_perception=True,
                 use_behavior=True, pose=(0.0, 0.0, 0.0))
    return stack.launch_description(), {'stack': stack}


class TestSoak(cases.ProbeCase):
    """장시간 운용: 크래시·메모리 증가·작업 실패율·완료율."""

    CTX = CTX

    def test_10_soak(self) -> None:
        from amr_msgs.msg import Task
        from amr_msgs.srv import AssignTask
        status = self.probe.subscribe('task_status', Task, keep_messages=5000)
        phase = self.probe.subscribe('executor/phase', String, 'latched')
        self.assertTrue(self.probe.service_available(AssignTask, 'assign_task',
                                                     self.timeout(600.0)))
        self.wait_lifecycle_active(LIFECYCLE, 600.0)
        self.check_map_registration()
        self.wait_startup_still()
        table = docks.load_docks(req.config_file('amr_behavior', 'behavior.yaml'))
        procs = procmon.system_processes()
        self.measure('processes', procs)
        self.measure('soak_hours', HOURS)
        start = self.probe.wall()
        mem_rows, task_rows = [], []
        current, assigned_at, n_tasks, rejected = None, 0.0, 0, 0
        outcome = {}                # task_id → COMPLETED / FAILED / TIMEOUT
        next_sample = start

        def final(tid):
            for _, m in reversed(status.messages()):
                if m.task_id == tid and m.status in (Task.STATUS_COMPLETED, Task.STATUS_FAILED):
                    return 'COMPLETED' if m.status == Task.STATUS_COMPLETED else 'FAILED'
            return None

        while self.probe.wall() - start < HOURS * 3600.0 and self.time_left() > 60.0:
            now = self.probe.wall()
            if current is not None:
                end = final(current)
                if end is None and now - assigned_at > TASK_MAX_S:
                    end = 'TIMEOUT'
                if end is not None:
                    outcome[current] = end
                    task_rows.append([current, assigned_at - start, now - start,
                                      now - assigned_at, end])
                    self.ctx.record.write_csv('tasks.csv', TASK_COLUMNS, task_rows)
                    current = None
            idle = phase.last() is not None and phase.last().data.upper() == 'IDLE'
            if current is None and idle:
                pick, drop = ROUTE[n_tasks % len(ROUTE)]
                task = Task(task_id=f'itest_soak_{n_tasks:05d}', robot_id=self.settings.robot,
                            priority=50, item_type='small', item_mass=2.0)
                task.pickup_pose = actions.pose_stamped(*table[pick].staging)
                task.dropoff_pose = actions.pose_stamped(*table[drop].staging)
                task.header.stamp = self.probe.node.get_clock().now().to_msg()
                res = self.probe.call(AssignTask, 'assign_task', AssignTask.Request(task=task),
                                      self.timeout(10.0))
                if res is not None and res.success:
                    current, assigned_at = task.task_id, self.probe.wall()
                    n_tasks += 1
                else:
                    rejected += 1
            if now >= next_sample:
                t = now - start
                for pid, name in procs.items():
                    mem_rows.append([t, name, pid, procmon.rss_mb(pid)])
                self.ctx.record.write_csv('memory.csv', ['time', 'process', 'pid', 'rss_mb'],
                                          mem_rows)
                next_sample += SAMPLE_S
            self.probe.sleep_ros(1.0, self.timeout(60.0))
        elapsed_h = (self.probe.wall() - start) / 3600.0
        completed = sum(1 for v in outcome.values() if v == 'COMPLETED')
        failed_tasks = sum(1 for v in outcome.values() if v in ('FAILED', 'TIMEOUT'))
        crashed = sorted({name for _, name, _, rss in mem_rows if rss is None})
        slopes, growth, leaks, unjudged = {}, {}, {}, []
        for pid, name in procs.items():
            if name in crashed:
                continue                        # 크래시 판정이 따로 잡는다
            pts = [(t, rss) for t, _, p, rss in mem_rows
                   if p == pid and rss is not None and t >= WARMUP_S]
            key = f'{name}[{pid}]'
            state, slopes[key], growth[key] = procmon.leak_verdict(
                [p[0] for p in pts], [p[1] for p in pts], LEAK_MAX_MB_H)
            if state == 'leak':
                leaks[key] = round(slopes[key], 2)
            elif state == 'insufficient':
                unjudged.append(key)
        ended = max(len(outcome), 1)
        fail_rate = failed_tasks / ended
        need = min_completed(HOURS)
        self.ctx.record.write_text('soak.json', json.dumps({
            'hours_requested': HOURS, 'hours_run': round(elapsed_h, 3), 'tasks_assigned': n_tasks,
            'tasks_ended': len(outcome), 'completed': completed, 'failed_or_timeout': failed_tasks,
            'rejected_assign': rejected, 'crashed': crashed,
            'rss_slope_mb_per_h': slopes, 'rss_growth_mb_after_warmup': growth,
            'leak_unjudged': unjudged}, ensure_ascii=False, indent=2))
        self.measure('soak', {'hours_run': round(elapsed_h, 3), 'completed': completed,
                              'failed_or_timeout': failed_tasks, 'rejected_assign': rejected,
                              'leaks': leaks, 'leak_unjudged': len(unjudged)})
        failed = []
        for name, value, thr, ok, unit in (
                ('soak duration', round(elapsed_h, 3), HOURS, elapsed_h >= HOURS * 0.999, 'h'),
                ('crashed processes', crashed, [], not crashed, ''),
                (f'memory growth > {LEAK_MAX_MB_H} MB/h', sorted(leaks), [], not leaks, ''),
                (f'leak judged (>= {procmon.LEAK_MIN_POINTS} samples after warm-up)',
                 unjudged or 'all', [], not unjudged and bool(procs), ''),
                ('task failure rate (failed + timeout)', fail_rate, FAIL_RATE_MAX,
                 fail_rate <= FAIL_RATE_MAX, ''),
                ('completed tasks', completed, f'>= {need}', completed >= need, '')):
            self.ctx.record.check(name, cases.fmt(value), thr, bool(ok), unit)
            if not ok:
                failed.append(f'{name}: {cases.fmt(value)} {unit} (기준 {thr})')
        self.assertFalse(failed, '; '.join(failed))


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
