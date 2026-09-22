"""
시나리오 12: 5대 동시 운용 교착 (명세 4.9 교착 탐지·해소, 4.10 "5대 운용 시 CPU 80 % 이하").

스택: amr_bringup/launch/multi_robot.launch.py (Gazebo 1개 + 로봇 5대 전체 스택 + fleet_manager_node +
traffic_manager_node, multi_robot.md §9). 스폰은 fleet_spawn.yaml (대기 구역), 지도는 maps/warehouse.yaml
(명시).
로봇별 평가 로거(evaluation.launch.py)·대시보드는 CPU 측정을 부풀리므로 끄고 amr_evaluation cpu_sampler 하나만 띄운다.
준비: 5대의 lifecycle(localization·navigation) + 루트 map_server 활성, map ↔ 월드 항등 정합, /fleet/assign_task.

(a) 동시 작업: 도크 사이 작업(behavior.yaml staging — 적재 도크에 물품이 있어야 인식이 통과한다)을
    /fleet/assign_task 로 한꺼번에 넣는다 (ITEST_MR_TASKS, 기본 5). 같은 통로를 반대로 지나는 쌍이 있어
    교통 관리자가 예방(hold)·해소해야 한다.
    판정: 작업 모두 COMPLETED (작업당 600 s), 탐지된 교착(traffic/DEADLOCK)은 모두 해소
    (traffic/RESOLVED, UNRESOLVED 0)
    — 탐지가 없으면 교착이 생기지 않은 것 (명세 9장 "교착 없이 작업이 수행되는가"),
    CPU = 시스템 프로세스(ROS 노드 + Gazebo 서버) CPU 합 / 호스트 전체 코어 [%] 평균 ≤ 80 % (코어 수 환산도 기록).
(b) 강제 교착 (BLOCKED — amr_fleet 실제 배치 회귀
    test_warehouse_estop_holder_in_intersection_blocked_then_detour 와 같은 배치, 로봇 수를 줄이면
    마지막 두 대): IDLE 인 amr_05 를
    교차로 x_ab_4 가운데(3, 6)로 순간 이동해 E-stop(estop 버튼, 래치) — 몸체가 구역에 걸친 로봇은 토큰 보유자로 등록되고
    E-stop 이라 움직일 수 없다. amr_04 를 바로 서쪽 (0.8, 5) 로 옮기고 (배치 단계: 두 대 모두 옮긴 자세를
    actions.seed_pose 로 AMCL·map EKF 에 알려 주고 추정 오차 ≤ 0.10 m 를 확인) 실행기에 dock_b 작업을 준다
    (/amr_04/assign_task) —
    동쪽·북쪽 어느 경로든 x_ab_4 를 지나므로 토큰 대기 → BLOCKED 탐지 → 대체 경로(keepout) 해소가 기대 동작이다.
    작업은 실행기를 거쳐야 한다: 교통 관리자는 robot_state 가 주행(작업 중)인 로봇만 "서 있으면 안 되는" 로봇으로
    보고 wait-for 간선을 만든다 — 예전 판(Nav2 navigate_to_pose 직접, 좁은 통로 정면)은 실행기가 IDLE 이라
    두 로봇이 실제로 200 s 막혔는데도(Failed to make progress) 탐지 대상이 아니었다 (스모크 실측).
    판정: traffic/DEADLOCK ≥ 1 이고 탐지마다 traffic/RESOLVED, UNRESOLVED 0
    (해소 상한 t_deadlock_max 120 s + 여유). 끝나면 E-stop 해제 + safety/reset_estop.
로그 traffic.csv, tasks.csv, cpu.csv, forced.csv.
"""

import math
import os

from amr_itest import actions, cases, catalog, docks, gz, metrics, procmon
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import Bool, String

CTX = Context(catalog.get(12))

SPEC_ROBOTS = 5
# 명세 시행 구성은 5대. 줄이면(ITEST_MR_ROBOTS) '로봇 수' 판정이 실패로 남는다 — 기동이 불안정한 호스트에서
# 나머지 판정(작업·CPU·강제 교착)을 돌려 보기 위한 손잡이이지 통과 구성이 아니다.
ROBOTS = max(2, min(SPEC_ROBOTS, int(os.environ.get('ITEST_MR_ROBOTS', str(SPEC_ROBOTS)))))
NAMES = [f'amr_{i:02d}' for i in range(1, ROBOTS + 1)]
CPU_MAX = 80.0
TASK_MAX_S = 600.0
# (pickup, dropoff) — 앞 네 건은 같은 통로를 반대 방향으로 지나는 쌍
PLAN = [('dock_1', 'dock_a'), ('dock_a', 'dock_1'), ('dock_2', 'dock_b'), ('dock_b', 'dock_2'),
        ('dock_1', 'dock_b'), ('dock_a', 'dock_2'), ('dock_2', 'dock_a'), ('dock_b', 'dock_1'),
        ('dock_1', 'dock_b'), ('dock_a', 'dock_2')]
N_TASKS = max(1, min(len(PLAN), int(os.environ.get('ITEST_MR_TASKS', '5'))))
# 강제 교착 (BLOCKED): traffic_zones.yaml x_ab_4 = x 2..4, y 3.5..8.5 (통로 AB × 열 틈 4)
FORCED_BLOCKER = (NAMES[-1], (3.0, 6.0, 0.0))          # 구역 안 E-stop 보유자
FORCED_VICTIM = (NAMES[-2], (0.8, 5.0, 0.0))           # 구역 서쪽 입구 1.2 m 앞 (요청 거리 안)
FORCED_TASK = ('dock_b', 'dock_a')                     # 동쪽 도크 — 경로가 x_ab_4 를 지난다
FORCED = (FORCED_VICTIM, FORCED_BLOCKER)
FORCED_MAX_S = 180.0      # t_deadlock_max 120 s + 탐지·통과 여유
SETTLE_S = 4.0
SETTLE_TOL_M = 0.10
LIFECYCLE = ['/lifecycle_manager_map'] + [f'/{n}/lifecycle_manager_{s}' for n in NAMES
                                          for s in ('localization', 'navigation')]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require([req.executable('amr_fleet', 'fleet_manager_node', '작업 할당'),
                 req.executable('amr_fleet', 'traffic_manager_node', '교착 탐지·해소'),
                 req.config('amr_behavior', 'behavior.yaml', '도크 표')], 'multi-robot deadlock')
    stack = Stack(CTX, *CTX.select())
    stack.multi_robot(ROBOTS)
    # 명세 4.10 CPU 절차: amr_evaluation cpu_sampler (호스트 전체 · 프로세스 그룹 · RTF · load1) →
    # logs/itest/12_multi_robot_deadlock/cpu_*.csv (하네스 procmon 과 함께 기록)
    stack.eval_logger('cpu_sampler', {})
    stack.components.update({'tasks': str(N_TASKS)})
    return stack.launch_description(), {'stack': stack}


def traffic_rows(traffic):
    """/fleet/traffic_events → [(recv, level, event, robot, message)] (level 은 bytes 1 개)."""
    return [(t, s.level[0] if isinstance(s.level, bytes) else int(s.level), s.name,
             s.hardware_id, s.message) for t, m in traffic.messages() for s in m.status]


def deadlock_counts(rows, since: float = -1.0):
    """(탐지, 해소, 해소 실패) 수 (amr_fleet alerts.py: traffic/DEADLOCK · RESOLVED · UNRESOLVED)."""
    sel = [r for r in rows if r[0] >= since]
    return (sum(1 for r in sel if r[2] == 'traffic/DEADLOCK'),
            sum(1 for r in sel if r[2] == 'traffic/RESOLVED'),
            sum(1 for r in sel if r[2] == 'traffic/UNRESOLVED'))


class TestMultiRobotDeadlock(cases.ProbeCase):
    """동시 작업 + CPU, 강제 교착 탐지·해소."""

    CTX = CTX

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        from amr_msgs.msg import Task
        from diagnostic_msgs.msg import DiagnosticArray
        cls.Task = Task
        cls.events = cls.probe.subscribe('/fleet/task_events', Task, keep_messages=5000)
        cls.traffic = cls.probe.subscribe('/fleet/traffic_events', DiagnosticArray,
                                          keep_messages=5000)
        cls.phases = {n: cls.probe.subscribe(f'/{n}/executor/phase', String, 'latched',
                                             keep_messages=2000)
                      for n in NAMES}

    def _write_traffic(self) -> None:
        self.ctx.record.write_csv('traffic.csv', ['recv_time', 'level', 'event', 'robot',
                                                  'message'],
                                  [list(r) for r in traffic_rows(self.traffic)])

    def test_10_concurrent_tasks(self) -> None:
        from amr_msgs.srv import AssignTask
        self.assertTrue(self.probe.service_available(AssignTask, '/fleet/assign_task',
                                                     self.timeout(600.0)),
                        '/fleet/assign_task 없음')
        self.wait_lifecycle_active(LIFECYCLE, 600.0)
        self.check_map_registration()
        self.wait_startup_still()
        table = docks.load_docks(req.config_file('amr_behavior', 'behavior.yaml'))
        pids = procmon.system_processes()
        own, host = procmon.ProcessCpuSampler(list(pids)), procmon.CpuSampler()
        self.measure('system_processes', {'count': len(pids),
                                          'simulator': [p for p, n in pids.items()
                                                        if n == 'gazebo_server']})
        ids, t_assign = [], {}
        for i, (pick, drop) in enumerate(PLAN[:N_TASKS]):
            task = self.Task()
            task.task_id = f'itest_mr_{i}'
            task.priority = 100 - i
            task.item_type = 'small'
            task.item_mass = 2.0
            task.pickup_pose = actions.pose_stamped(*table[pick].staging)
            task.dropoff_pose = actions.pose_stamped(*table[drop].staging)
            res = self.probe.call(AssignTask, '/fleet/assign_task',
                                  AssignTask.Request(task=task), self.timeout(30.0))
            self.assertTrue(res is not None and res.success, f'작업 {i} 거절 {res}')
            ids.append(task.task_id)
            t_assign[task.task_id] = self.probe.now()
        w0 = self.probe.wall()
        run_max = TASK_MAX_S * math.ceil(N_TASKS / ROBOTS) + 60.0
        cpu_rows = []

        def final():
            return {m.task_id: m.status for _, m in self.events.messages()}

        def done() -> bool:
            f = final()
            return all(f.get(t) in (self.Task.STATUS_COMPLETED, self.Task.STATUS_FAILED)
                       for t in ids)

        while not done() and self.probe.wall() - w0 < self.timeout(run_max):
            if self.time_left() < 60.0 + FORCED_MAX_S + 300.0:
                self.ctx.record.note('러너 상한이 가까워 동시 작업 대기를 멈춘다')
                break
            self.probe.sleep_ros(1.0, self.timeout(60.0))
            own.update_pids(list(procmon.system_processes()))
            cpu_rows.append([self.probe.now(), own.sample(), host.sample(), own.cores_used()])
        f = final()
        rows = []
        for tid in ids:
            evs = [(t, m) for t, m in self.events.messages() if m.task_id == tid]
            rows.append([tid, evs[-1][1].robot_id if evs else '', f.get(tid, -1),
                         evs[0][0] if evs else math.nan, evs[-1][0] if evs else math.nan])
        self.ctx.record.write_csv('tasks.csv', ['task_id', 'robot_id', 'final_status',
                                                'first_event_wall', 'last_event_wall'], rows)
        self.ctx.record.write_csv('cpu.csv', ['time', 'system_cpu_percent', 'host_cpu_percent',
                                              'cores_used'], cpu_rows)
        self._write_traffic()
        n = max(len(cpu_rows), 1)
        own_mean = sum(r[1] for r in cpu_rows) / n
        self.measure('cpu', {'system_mean_percent': cases.fmt(own_mean, 1),
                             'system_max_percent': cases.fmt(max((r[1] for r in cpu_rows),
                                                                 default=math.nan), 1),
                             'cores_used_mean': cases.fmt(sum(r[3] for r in cpu_rows) / n, 2),
                             'host_cores': own.cores,
                             'host_mean_percent': cases.fmt(sum(r[2] for r in cpu_rows) / n, 1),
                             'samples': len(cpu_rows), 'loadavg_end': os.getloadavg()[0]})
        detected, resolved, unresolved = deadlock_counts(traffic_rows(self.traffic))
        completed = sum(1 for t in ids if f.get(t) == self.Task.STATUS_COMPLETED)
        self.measure('traffic', {'deadlocks': detected, 'resolved': resolved,
                                 'unresolved': unresolved})
        failed = []
        for name, value, thr, ok, unit in (
                ('robots', ROBOTS, SPEC_ROBOTS, ROBOTS == SPEC_ROBOTS, ''),
                ('tasks completed', completed, len(ids), completed == len(ids), ''),
                ('detected deadlocks resolved', {'detected': detected, 'resolved': resolved,
                                                 'unresolved': unresolved},
                 'resolved >= detected, unresolved 0',
                 resolved >= detected and unresolved == 0, ''),
                ('system CPU mean (ROS nodes + Gazebo, % of host cores)', own_mean, CPU_MAX,
                 bool(cpu_rows) and own_mean <= CPU_MAX, '%')):
            self.ctx.record.check(name, cases.fmt(value), thr, bool(ok), unit)
            if not ok:
                failed.append(f'{name}: {cases.fmt(value)} (기준 {thr})')
        self.assertFalse(failed, '; '.join(failed))

    def test_20_forced_deadlock(self) -> None:
        """교차로 안 E-stop 보유자 → 작업 중 로봇의 토큰 대기 → BLOCKED 탐지 → 대체 경로 해소."""
        from amr_msgs.srv import AssignTask
        from std_srvs.srv import Trigger
        self.assertTrue(self.budget_for(FORCED_MAX_S + 300.0, 'forced deadlock'),
                        '러너 상한 안에 강제 교착을 할 시간이 없다')
        self.wait_lifecycle_active([f'/{n}/lifecycle_manager_{s}' for n, _ in FORCED
                                    for s in ('localization', 'navigation')], 120.0,
                                   'forced robots lifecycle active')
        world = self.settings.world.rsplit('.', 1)[0]
        table = docks.load_docks(req.config_file('amr_behavior', 'behavior.yaml'))
        for name, _ in FORCED:
            idle = self.probe.wait_until(
                lambda name=name: self.phases[name].last() is not None
                and self.phases[name].last().data.upper() == 'IDLE', self.timeout(300.0))
            self.assertTrue(idle, f'{name}: IDLE 이 아니어서 강제 교착에 쓸 수 없다')
        lost = {n: self.probe.subscribe(f'/{n}/localization/lost', Bool, 'latched')
                for n, _ in FORCED}
        est = {n: self.probe.subscribe(f'/{n}/odometry/filtered_map', Odometry)
               for n, _ in FORCED}
        gts = {n: self.probe.subscribe(f'/{n}/ground_truth/odom', Odometry) for n, _ in FORCED}
        for name, start in FORCED:
            ok, out = gz.set_pose(world, name, *start)
            self.assertTrue(ok, f'{name}: set_pose 실패 {out}')
        # 배치 = 운영자가 로봇을 놓고 자세를 알려 주는 단계 (판정 대상 아님): 통로 AB 는 랙이 되풀이돼
        # kidnap 전역 재초기화가 옆 통로로 오수렴할 수 있다 (11 스모크 실측) → 옮긴 자세를 AMCL·map EKF 에 준다
        self.probe.sleep_ros(2.0, self.timeout(30.0))
        seeded = {name: actions.seed_pose(self.probe, name, *start, timeout=self.timeout(10.0))
                  for name, start in FORCED}
        self.measure('forced_placement', seeded)
        for name, _ in FORCED:                     # lost false + 추정 오차 ≤ 0.10 m 가 SETTLE_S 유지
            state = {'since': None}

            def settled(name=name, state=state) -> bool:
                m, g, e = lost[name].last(), gts[name].last(), est[name].last()
                if m is None or m.data or g is None or e is None:
                    state['since'] = None
                    return False
                a, b = metrics.sample_from_odom(g), metrics.sample_from_odom(e)
                if math.hypot(a.x - b.x, a.y - b.y) > SETTLE_TOL_M:
                    state['since'] = None
                    return False
                state['since'] = state['since'] or self.probe.now()
                return self.probe.now() - state['since'] >= SETTLE_S
            self.assertTrue(self.probe.wait_until(settled, self.timeout(120.0), 0.1),
                            f'{name}: 배치 뒤 위치 추정이 안정되지 않음 (lost 또는 오차 > '
                            f'{SETTLE_TOL_M} m)')
        blocker, victim = FORCED_BLOCKER[0], FORCED_VICTIM[0]
        estop = self.probe.subscribe(f'/{blocker}/safety/estop_active', Bool, 'latched')
        try:
            self.probe.publish(f'/{blocker}/estop', Bool(data=True), 'latched')
            self.assertTrue(self.probe.wait_until(
                lambda: estop.last() is not None and estop.last().data, self.timeout(30.0)),
                f'{blocker}: E-stop 이 걸리지 않음 (safety/estop_active)')
            task = self.Task()
            task.task_id = 'itest_mr_forced'
            task.robot_id = victim
            task.priority = 100
            task.item_type = 'small'
            task.item_mass = 2.0
            task.pickup_pose = actions.pose_stamped(*table[FORCED_TASK[0]].staging)
            task.dropoff_pose = actions.pose_stamped(*table[FORCED_TASK[1]].staging)
            task.header.stamp = self.probe.node.get_clock().now().to_msg()
            w0 = self.probe.wall()
            res = self.probe.call(AssignTask, f'/{victim}/assign_task',
                                  AssignTask.Request(task=task), self.timeout(30.0))
            self.assertTrue(res is not None and res.success, f'{victim}: 작업 거절 {res}')

            def resolved_all() -> bool:
                d, r, u = deadlock_counts(traffic_rows(self.traffic), w0)
                return d >= 1 and (r >= d or u > 0)
            self.probe.wait_until(resolved_all, self.timeout(FORCED_MAX_S), 0.5)
        finally:
            self.probe.publish(f'/{blocker}/estop', Bool(data=False), 'latched')
            self.probe.call(Trigger, f'/{blocker}/safety/reset_estop', Trigger.Request(),
                            self.timeout(10.0))
        detected, resolved, unresolved = deadlock_counts(traffic_rows(self.traffic), w0)
        rows = [[t, lv, ev, robot, msg] for t, lv, ev, robot, msg in traffic_rows(self.traffic)
                if t >= w0]
        self.ctx.record.write_csv('forced.csv', ['recv_time', 'level', 'event', 'robot',
                                                 'message'], rows)
        self._write_traffic()
        ends = {n: [round(v, 2) for v in (metrics.sample_from_odom(gts[n].last()).x,
                                          metrics.sample_from_odom(gts[n].last()).y)]
                for n in gts if gts[n].last() is not None}
        self.measure('forced', {'deadlocks': detected, 'resolved': resolved,
                                'unresolved': unresolved, 'end_positions': ends,
                                'victim_phases': [m.data for _, m in
                                                  self.phases[victim].messages(w0)][-20:],
                                'events': [f'{r[2]} {r[3]} {r[4]}'[:200] for r in rows][:40]})
        self.check('forced deadlock detected', detected, '>= 1', detected >= 1)
        self.check('forced deadlock resolved', {'resolved': resolved, 'unresolved': unresolved},
                   f'resolved >= {detected}, unresolved 0',
                   resolved >= detected and unresolved == 0)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
