"""
시나리오 13: 응답 시간 (명세 4.10 "명령 수신 ~ 로봇 반응 평균 200 ms 이내", 50회 이상).

측정 정의 — 판정은 설계 계약(sequences.md §1 성능 표: "assign_task 요청 시각 → 첫 cmd_vel ≠ 0,
평균 200 ms 이내")을 따른다.
  명령 시각  하네스가 fleet_manager_node 자리에서 AssignTask 요청의 task.header.stamp 를 발행 직전
            ROS 시각으로 찍는다 (fleet_manager_node 와 같은 규약). 같은 Task 를 itest/task_events 로도
            발행해 amr_evaluation response_time_logger 가 명령 시각으로 쓴다.
  반응      safety_node 출력 cmd_vel 의 첫 0 아닌 명령 (|v|·|ω| > 1e-3). 하네스는 wall 시각
            (명령 직전 wall → cmd_vel 수신 wall), response_time_logger 는 twist 모드(노드 시계)로 잰다.
  첫 움직임  참고 지표: ground_truth/odom 의 |v| ≥ 0.05 m/s 또는 |ω| ≥ 0.1 rad/s 인 첫 표본 스탬프.
            모든 명령에 대해 실제로 움직여야 한다(무응답 0 판정)만 보고, 시간은 기록만 한다 — 명세
            5장 저크 한계 2 m/s³ 의 S-커브는 정지에서 0.05 m/s 까지만 √(2·0.05/2) = 224 ms 가 걸려
            이 정의로는 200 ms 가 물리적으로 불가능하다 (실제 velocity_profiler_node 로 249 ms 측정).
경로: assign_task → task_executor_node → cmd_vel_nav → velocity_profiler_node → cmd_vel_smoothed →
safety_node → cmd_vel → 시뮬레이터. task_executor 는 component 프로필에서 대역이다 (실제 노드는
Nav2 가 필요 — ITEST_PROFILE=system 으로 같은 측정을 한다).
시행마다 로봇이 멈춘 뒤(GT 1 s 정지) 명령을 내므로 "이미 움직이는 중" 샘플은 없다.
"""

from amr_itest import actions, cases, catalog, evaluation, metrics
from amr_itest.scenario import Context
from amr_itest.stack import Stack
from geometry_msgs.msg import Twist
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

CTX = Context(catalog.get(13))

SAMPLES = 50
MEAN_MAX_MS = 200.0
STARTUP_WALL_S = 180.0
TASK_EVENTS = 'itest/task_events'
CMD_EPS = 1e-3             # [m/s, rad/s] "cmd_vel ≠ 0" 판정 (actions.is_zero 와 같은 허용치)
MOTION_V = 0.05            # [m/s] 참고 지표 "첫 움직임" (amr_evaluation 기본 motion_threshold)
MOTION_W = 0.1             # [rad/s]
EVENTS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.simulator()
    stack.description()
    stack.localization(ekf=False)
    stack.velocity_chain(profiler=True, safety=True)
    stack.task_executor(drive_speed=0.3, drive_time=0.6)
    stack.eval_logger('response_time_logger', {
        'cmd_topic': TASK_EVENTS, 'cmd_type': 'task', 'motion_topic': 'cmd_vel',
        'motion_type': 'twist', 'motion_threshold': CMD_EPS, 'angular_threshold': CMD_EPS,
        'robot_id': CTX.settings.robot, 'require_rest': True})
    return stack.launch_description(), {'stack': stack}


class TestResponseTime(cases.ProbeCase):
    """작업 명령 → 첫 움직임 지연 50회."""

    CTX = CTX

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        from amr_msgs.msg import Task
        cls.Task = Task
        cls.gt = cls.probe.subscribe('ground_truth/odom', Odometry, keep_messages=5000)
        cls.cmd = cls.probe.subscribe('cmd_vel', Twist, keep_messages=5000)
        cls.status = cls.probe.subscribe('task_status', Task, keep_messages=500)

    def _task(self, i: int):
        task = self.Task()
        task.task_id = f'itest_rt_{i:03d}'
        task.robot_id = self.settings.robot
        task.priority = 100
        task.item_type = 'small'
        task.item_mass = 2.0
        task.status = self.Task.STATUS_IN_PROGRESS
        task.pickup_pose.header.frame_id = task.dropoff_pose.header.frame_id = 'map'
        return task

    def test_10_response_time(self) -> None:
        """명령 → 첫 cmd_vel ≠ 0: 50회 평균 ≤ 200 ms, 무응답(cmd_vel·실제 움직임) 0."""
        from amr_msgs.srv import AssignTask
        self.ready_gate([('ground_truth/odom', Odometry)], STARTUP_WALL_S)
        self.require_topic(self.gt, 3, STARTUP_WALL_S)
        self.assertTrue(self.probe.service_available(AssignTask, 'assign_task',
                                                     self.timeout(STARTUP_WALL_S)),
                        'assign_task 서비스 없음 (task_executor_node)')
        events = self.probe.publisher(TASK_EVENTS, self.Task, EVENTS_QOS)
        rows, lat, onset_rows, onset, no_response = [], [], [], [], []
        for i in range(SAMPLES):
            self.assertTrue(actions.wait_rest(self.probe, self.gt, hold=1.0,
                                              timeout=self.timeout(30.0)),
                            f'trial {i}: 로봇이 정지하지 않음')
            req = AssignTask.Request()
            req.task = self._task(i)
            req.task.header.stamp = self.probe.node.get_clock().now().to_msg()
            req.task.pickup_pose.header.stamp = req.task.dropoff_pose.header.stamp = \
                req.task.header.stamp
            t_cmd = req.task.header.stamp.sec + req.task.header.stamp.nanosec * 1e-9
            w_cmd = self.probe.wall()
            events.publish(req.task)
            res = self.probe.call(AssignTask, 'assign_task', req, self.timeout(10.0))
            self.assertTrue(res is not None and res.success, f'trial {i}: assign_task 거절 {res}')
            react = actions.wait_first(self.probe, self.cmd, w_cmd,
                                       lambda m: not actions.is_zero(m, CMD_EPS),
                                       self.timeout(10.0))
            self.probe.wait_until(
                lambda: metrics.first_motion(self._track(t_cmd, w_cmd), t_cmd, MOTION_V,
                                             MOTION_W) is not None, self.timeout(10.0), 0.005)
            first = metrics.first_motion(self._track(t_cmd, w_cmd), t_cmd, MOTION_V, MOTION_W)
            if react is None or first is None:
                no_response.append(req.task.task_id)
                continue
            latency = (react[0] - w_cmd) * 1e3
            lat.append(latency)
            rows.append([w_cmd, react[0], latency, req.task.task_id])
            onset.append((first.t - t_cmd) * 1e3)
            onset_rows.append([t_cmd, first.t, onset[-1], req.task.task_id])
            done = self.probe.wait_until(lambda i=i: self._completed(f'itest_rt_{i:03d}'),
                                         self.timeout(20.0))
            self.assertTrue(done, f'trial {i}: task_status COMPLETED 없음')
        # 판정 지표(wall 시각)와 참고 지표(첫 움직임, ROS 시각)를 따로 남긴다
        self.ctx.record.write_csv('harness_response_time.csv', metrics.RESPONSE_COLUMNS, rows)
        self.ctx.record.write_csv('harness_motion_onset.csv', metrics.RESPONSE_COLUMNS,
                                  onset_rows)
        s = metrics.latency_summary(lat)
        self.measure('harness_latency_ms', s)
        # 참고: GT |v| ≥ 0.05 m/s 도달 (ROS 시각) — 판정 안 함 (모듈 docstring)
        self.measure('motion_onset_ms_reference', metrics.latency_summary(onset))
        self.check('no-response commands (cmd_vel and GT motion)', len(no_response), 0,
                   not no_response, '', str(no_response))
        self.check('samples', s['count'], f'>= {SAMPLES}', s['count'] >= SAMPLES)
        self.check('mean response time (harness, cmd→cmd_vel≠0)', s['mean'], MEAN_MAX_MS,
                   s['mean'] <= MEAN_MAX_MS, 'ms', f"max {s['max']} ms, p95 {s['p95']} ms")

    def _track(self, since: float, wall_since: float):
        """명령 이후 GT 표본 (수신 시각으로 먼저 거르고 스탬프로 다시 거른다)."""
        out = []
        for _, m in self.gt.messages(wall_since - 0.5):
            smp = metrics.sample_from_odom(m)
            if smp.t >= since:
                out.append(smp)
        return out

    def _completed(self, task_id: str) -> bool:
        return any(m.task_id == task_id and m.status == self.Task.STATUS_COMPLETED
                   for _, m in self.status.messages())


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX

    def test_evaluation_logger(self) -> None:
        """amr_evaluation response_time_logger CSV → analyze 판정 (설치돼 있을 때)."""
        if CTX.record.data['components'].get('evaluation') != 'real':
            self.skipTest('amr_evaluation 미설치 — 하네스 자체 계산(test_10)으로만 판정')
        result = evaluation.analyze(CTX.log_dir, response_ms=MEAN_MAX_MS)
        rows = evaluation.gated_rows(result, '응답 시간')
        CTX.record.measure('evaluation_response_rows', rows)
        CTX.record.measure('evaluation_warnings', result['warnings'])
        self.assertTrue(rows, f"response_time.csv 판정 행 없음: {result['warnings']}")
        r = rows[0]
        CTX.record.check(f"amr_evaluation mean response (n={r['count']})", cases.fmt(r['value']),
                         MEAN_MAX_MS, bool(r['passed']), 'ms')
        self.assertGreaterEqual(r['count'], SAMPLES, 'response_time.csv 샘플 부족')
        self.assertTrue(r['passed'], f"amr_evaluation 평균 {r['value']} ms > {MEAN_MAX_MS} ms")
