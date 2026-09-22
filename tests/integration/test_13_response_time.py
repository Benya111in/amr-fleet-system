"""
시나리오 13: 응답 시간 (명세 4.10 "명령 수신 ~ 로봇 반응 평균 200 ms 이내").

표준 절차 (명세 4.10 표): 작업 명령 발행 timestamp − 로봇 첫 움직임 timestamp, 50회 이상 평균/최대.

측정 정의 (명세 표)
  명령 시각  하네스가 플릿 자리에서 AssignTask 요청의 task.header.stamp 를 발행 직전 ROS 시각으로 찍는다.
            같은 Task(IN_PROGRESS)를 itest/task_events 로도 발행해 amr_evaluation response_time_logger 가
            명령 시각으로 쓴다 (cmd_stamp=header).
  첫 움직임  ground_truth/odom 에서 명령 뒤 처음으로 |v| ≥ 0.01 m/s 또는 |ω| ≥ 0.02 rad/s 인 표본의 스탬프 —
            "로봇이 움직였다" 의 판정 정의 (명세 5장 저크 2 m/s³ S-커브에서 0.01 m/s 까지 100 ms).
            참고 변형 |v| ≥ 0.05 m/s 또는 |ω| ≥ 0.1 rad/s 도 같은 시행에서 함께 기록한다 (판정 안 함).
  지연       판정 = 벽시계 (명령 직전 wall → 그 GT 표본 수신 wall; GT 브리지 전달 포함 — 상한 쪽).
            amr_evaluation analyze 의 기본(벽시계 환산 latency/rtf)과 같은 쪽이다: RTF < 1 이면 스탬프(sim) 차는
            소프트웨어 지연을 작게 보인다. 스탬프 차(latency_ms, 명세 로그 포맷)와 RTF 도 함께 남긴다.
구성
  system     명세 판정. system.launch.py 전체 단일 로봇 (localization · navigation · perception ·
             behavior): assign_task → task_executor_node BT → Nav2 navigate_to_pose → controller →
             cmd_vel_nav → velocity_profiler_node → cmd_vel_smoothed → safety_node → cmd_vel →
             Gazebo.
             스폰 = 대기 자세 = dock_1·dock_2 사이 (-24.5, 15.0, π) — 작업(dock_2 → dock_1,
             dock_1 → dock_2 번갈아, 도크 자세는 behavior.yaml)이 짧게 끝나고 매번 같은 대기 자세로
             돌아온다. 시행마다 executor/phase = IDLE 이고 GT 가 1 s 정지한 뒤 명령한다 (실행기는 IDLE
             에서만 수락, 이미 움직이는 중의 샘플 없음). 실행기에 작업 취소 API 가 없어 시행 하나 = 작업 하나.
  component  체인 회귀 (명세 판정 아님): 운동학 대역 + 실제 velocity_profiler·safety + 실행기 대역
             (assign_task 콜백에서 cmd_vel_nav 를 바로 낸다). 같은 GT 정의로 판정하고 판정 이름에 구성을 붙인다.
시행 수 ITEST_RT_SAMPLES (기본 50 — 줄이면 '샘플 ≥ 50' 판정이 실패한다).
로그 response_time.csv (amr_evaluation), harness_response_time.csv (시행마다 갱신).
"""

import math
import os

from amr_itest import actions, cases, catalog, docks, evaluation, metrics
from amr_itest import requirements as req
from amr_itest.scenario import Context, SYSTEM
from amr_itest.stack import Stack, system_requirements
from geometry_msgs.msg import Twist
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

CTX = Context(catalog.get(13))

SPEC_SAMPLES = 50
SAMPLES = max(1, int(os.environ.get('ITEST_RT_SAMPLES', str(SPEC_SAMPLES))))
MEAN_MAX_MS = 200.0
RTF_VALID = 0.9            # 이보다 느린 시뮬레이션에서의 벽시계 판정은 조건 미달로 표시 (result.json notes)
MOTION_V = 0.01            # [m/s] 판정 정의 "첫 움직임"
MOTION_W = 0.02            # [rad/s]
MOTION_V_REF = 0.05        # [m/s] 참고 변형 (amr_evaluation 기본 motion_threshold)
MOTION_W_REF = 0.1         # [rad/s]
REST_V, REST_W = 0.005, 0.01   # 명령 전 정지 판정 (판정 임계보다 작게)
CMD_EPS = 1e-3             # "cmd_vel ≠ 0" (sequences.md §1 설계 정의 — 참고)
STARTUP_WALL_S = 300.0
RESPONSE_WAIT_S = 10.0     # 명령 → 첫 움직임 대기 상한 (50배 여유, 넘으면 무응답)
TASK_MAX_S = 300.0         # 작업 하나 (주행 4 m · 인식 · 도킹 2회 · 적재/하역 · 복귀) 상한
SPAWN = (-24.5, 15.0, math.pi)     # 대기 자세 = 스폰 (dock_1/2 staging 에서 약 3.5 m, 자유 반경 2.5 m)
TASK_DOCKS = (('dock_2', 'dock_1'), ('dock_1', 'dock_2'))
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')
TASK_EVENTS = 'itest/task_events'
EVENTS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
COLUMNS = metrics.RESPONSE_COLUMNS + ['wall_ms', 'latency_05_ms', 'cmd_vel_ms', 'rtf',
                                      'task_status']


def label(profile: str) -> str:
    return '' if profile == SYSTEM else ' [component: executor stand-in]'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    profile = CTX.select_profile()
    if profile == SYSTEM:
        CTX.require(system_requirements(use_behavior=True)
                    + [req.executable('amr_behavior', 'task_executor_node', '작업 실행기'),
                       req.config('amr_behavior', 'behavior.yaml', '도크 표')],
                    'response time (system)')
    stack = Stack(CTX, CTX.select_backend(), profile)
    if profile == SYSTEM:
        stack.system(use_localization=True, use_navigation=True, use_perception=True,
                     use_behavior=True, pose=SPAWN)
    else:
        stack.simulator()
        stack.description()
        stack.localization(ekf=False)
        stack.velocity_chain(profiler=True, safety=True)
        stack.task_executor(drive_speed=0.3, drive_time=0.6)
    stack.eval_logger('response_time_logger', {
        'cmd_topic': TASK_EVENTS, 'cmd_type': 'task', 'cmd_stamp': 'header',
        'motion_topic': 'ground_truth/odom', 'motion_type': 'odom',
        'motion_threshold': MOTION_V, 'angular_threshold': MOTION_W,
        'robot_id': CTX.settings.robot, 'require_rest': True, 'target_samples': SPEC_SAMPLES})
    return stack.launch_description(), {'stack': stack}


class TestResponseTime(cases.ProbeCase):
    """작업 명령 → GT 첫 움직임 지연."""

    CTX = CTX

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        from amr_msgs.msg import Task
        cls.Task = Task
        cls.gt = cls.probe.subscribe('ground_truth/odom', Odometry, keep_messages=20000)
        cls.cmd = cls.probe.subscribe('cmd_vel', Twist, keep_messages=5000)
        cls.status = cls.probe.subscribe('task_status', Task, keep_messages=1000)
        cls.phase = cls.probe.subscribe('executor/phase', String, 'latched', keep_messages=5000)

    def _task(self, i: int, stamp, table):
        task = self.Task()
        task.task_id = f'itest_rt_{i:03d}'
        task.robot_id = self.settings.robot
        task.priority = 100
        task.item_type = 'small'
        task.item_mass = 2.0
        task.status = self.Task.STATUS_IN_PROGRESS
        task.header.stamp = stamp
        if table is not None:
            pick, drop = TASK_DOCKS[i % len(TASK_DOCKS)]
            task.pickup_pose = actions.pose_stamped(*table[pick].staging)
            task.dropoff_pose = actions.pose_stamped(*table[drop].staging)
        else:
            task.pickup_pose.header.frame_id = task.dropoff_pose.header.frame_id = 'map'
        task.pickup_pose.header.stamp = task.dropoff_pose.header.stamp = stamp
        return task

    def _idle(self) -> bool:
        msg = self.phase.last()
        return msg is not None and msg.data.upper() == 'IDLE'

    def _final(self, task_id: str):
        for _, m in reversed(self.status.messages()):
            if m.task_id == task_id and m.status in (self.Task.STATUS_COMPLETED,
                                                     self.Task.STATUS_FAILED):
                return m.status
        return None

    def _track(self, since: float, wall_since: float):
        """명령 이후 GT 표본 (수신 wall 시각, Sample) — 수신 시각으로 먼저 거르고 스탬프로 다시 거른다."""
        out = []
        for t, m in self.gt.messages(wall_since - 0.5):
            smp = metrics.sample_from_odom(m)
            if smp.t >= since:
                out.append((t, smp))
        return out

    def _onset(self, since: float, wall_since: float, v: float, w: float):
        for t, smp in self._track(since, wall_since):
            if abs(smp.v) >= v or abs(smp.w) >= w:
                return t, smp
        return None

    def test_10_response_time(self, stack) -> None:
        """명령 → GT 첫 움직임 (|v| ≥ 0.01 m/s ∨ |ω| ≥ 0.02 rad/s): 평균 ≤ 200 ms, 샘플 ≥ 50, 무응답 0."""
        from amr_msgs.srv import AssignTask
        system = stack.profile == SYSTEM
        self.ready_gate([('ground_truth/odom', Odometry)], STARTUP_WALL_S)
        self.require_topic(self.gt, 3, STARTUP_WALL_S)
        table = None
        if system:
            table = docks.load_docks(req.share_dir('amr_behavior') / 'config' / 'behavior.yaml')
            self.wait_lifecycle_active(LIFECYCLE, STARTUP_WALL_S)
            self.check_map_registration()
            self.wait_startup_still()
        self.assertTrue(self.probe.service_available(AssignTask, 'assign_task',
                                                     self.timeout(STARTUP_WALL_S)),
                        'assign_task 서비스 없음 (task_executor_node)')
        events = self.probe.publisher(TASK_EVENTS, self.Task, EVENTS_QOS)
        rows, lat, lat05, wall, cmdvel, no_response, rejected, statuses = \
            [], [], [], [], [], [], [], {}
        for i in range(SAMPLES):
            if not self.budget_for((TASK_MAX_S if system else 30.0) + 60.0, f'trial {i}'):
                break
            if system:
                idle = self.probe.wait_until(self._idle, self.timeout(TASK_MAX_S), 0.1)
                self.assertTrue(idle, f'trial {i}: executor/phase 가 IDLE 로 돌아오지 않음 '
                                      f'({self.phase.last().data if self.phase.last() else None})')
            rested = actions.wait_rest(self.probe, self.gt, hold=1.0, timeout=self.timeout(60.0),
                                       v_tol=REST_V, w_tol=REST_W)
            self.assertTrue(rested, f'trial {i}: 로봇이 정지하지 않음')
            stamp = self.probe.node.get_clock().now().to_msg()
            req_msg = AssignTask.Request(task=self._task(i, stamp, table))
            t_cmd = stamp.sec + stamp.nanosec * 1e-9
            w_cmd = self.probe.wall()
            events.publish(req_msg.task)
            res = self.probe.call(AssignTask, 'assign_task', req_msg, self.timeout(10.0))
            if res is None or not res.success:
                rejected.append(f'{req_msg.task.task_id}: {getattr(res, "message", None)}')
                continue
            tid = req_msg.task.task_id
            self.probe.wait_until(
                lambda: self._onset(t_cmd, w_cmd, MOTION_V_REF, MOTION_W_REF) is not None,
                self.timeout(RESPONSE_WAIT_S), 0.005)
            first = self._onset(t_cmd, w_cmd, MOTION_V, MOTION_W)
            first05 = self._onset(t_cmd, w_cmd, MOTION_V_REF, MOTION_W_REF)
            react = actions.first_after(self.cmd, w_cmd, lambda m: not actions.is_zero(m, CMD_EPS))
            if first is None:
                no_response.append(tid)
            else:
                lat.append((first[1].t - t_cmd) * 1e3)
                wall.append((first[0] - w_cmd) * 1e3)
                lat05.append((first05[1].t - t_cmd) * 1e3 if first05 else math.nan)
                cmdvel.append((react[0] - w_cmd) * 1e3 if react else math.nan)
            if system:
                self.probe.wait_until(lambda: self._final(tid) is not None,
                                      self.timeout(TASK_MAX_S), 0.2)
                statuses[tid] = self._final(tid)
            else:
                self.probe.wait_until(lambda: self._final(tid) is not None, self.timeout(20.0))
                statuses[tid] = self._final(tid)
            rtf = ((first[1].t - t_cmd) / max(first[0] - w_cmd, 1e-6)) if first else math.nan
            rows.append([t_cmd, first[1].t if first else math.nan,
                         lat[-1] if first else math.nan, tid,
                         wall[-1] if first else math.nan, lat05[-1] if first else math.nan,
                         cmdvel[-1] if first else math.nan, rtf, statuses[tid]])
            self.ctx.record.write_csv('harness_response_time.csv', COLUMNS, rows)
        s = metrics.latency_summary(wall)
        self.measure('response_wall_ms', {'definition': f'GT |v|>={MOTION_V} or |w|>={MOTION_W}, '
                                                        'wall clock (judged)', **s})
        self.measure('response_stamp_ms', {'definition': 'same, header stamps (sim time)',
                                           **metrics.latency_summary(lat)})
        self.measure('response_ms_v005', {'definition': f'GT |v|>={MOTION_V_REF} or '
                                                        f'|w|>={MOTION_W_REF}, stamps',
                                          **metrics.latency_summary(lat05)})
        self.measure('cmd_vel_nonzero_ms_reference', metrics.latency_summary(cmdvel))
        # 스탬프 지연 / 벽시계 지연 = 그 구간의 RTF. RTF < 0.9 (공유 호스트 과부하)면 물리 가속 구간이 벽시계로
        # 늘어나 벽시계 판정이 시스템보다 호스트를 잰다 — 실패해도 "조건 미달" 로 읽도록 남긴다
        rtfs = [r[7] for r in rows if math.isfinite(r[7])]
        rtf_mean = sum(rtfs) / len(rtfs) if rtfs else math.nan
        self.measure('rtf_mean', cases.fmt(rtf_mean, 3))
        if math.isfinite(rtf_mean) and rtf_mean < RTF_VALID:
            self.ctx.record.note(f'RTF 평균 {rtf_mean:.2f} < {RTF_VALID}: 벽시계 판정은 호스트 부하를 포함한다 '
                                 f'(스탬프 기준 평균 {metrics.latency_summary(lat)["mean"]} ms)')
        done = sum(1 for v in statuses.values() if v == self.Task.STATUS_COMPLETED)
        self.measure('tasks', {'assigned': len(statuses), 'completed': done,
                               'rejected': rejected, 'samples_requested': SAMPLES})
        tag = label(stack.profile)
        failed = []
        for name, value, thr, ok, unit in (
                ('assign_task accepted', len(rejected), 0, not rejected, ''),
                ('no-response commands (GT motion within 10 s)', len(no_response), 0,
                 not no_response, ''),
                ('samples', s['count'], f'>= {SPEC_SAMPLES}', s['count'] >= SPEC_SAMPLES, ''),
                (f'mean response time (cmd→GT first motion, wall){tag}', s['mean'], MEAN_MAX_MS,
                 s['count'] > 0 and s['mean'] <= MEAN_MAX_MS, 'ms')):
            self.ctx.record.check(name, cases.fmt(value), thr, bool(ok), unit)
            if not ok:
                failed.append(f'{name}: {cases.fmt(value)} {unit} (기준 {thr})')
        self.measure('max_response_ms', s['max'])
        self.assertFalse(failed, f"{'; '.join(failed)} — 평균 {s['mean']} / 최대 {s['max']} ms, "
                                 f'무응답 {no_response}, 거절 {rejected}')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX

    def test_evaluation_logger(self) -> None:
        """amr_evaluation response_time_logger CSV (GT odom 모드, 같은 정의) → analyze 판정."""
        if CTX.record.data['components'].get('evaluation') != 'real':
            self.skipTest('amr_evaluation 미설치 — 하네스 자체 계산(test_10)으로만 판정')
        result = evaluation.analyze(CTX.log_dir, response_ms=MEAN_MAX_MS)
        rows = evaluation.gated_rows(result, '응답 시간')
        CTX.record.measure('evaluation_response_rows', rows)
        CTX.record.measure('evaluation_warnings', result['warnings'])
        self.assertTrue(rows, f"response_time.csv 판정 행 없음: {result['warnings']}")
        r = rows[0]
        CTX.record.check(f"amr_evaluation mean response (n={r['count']}){label(CTX.profile)}",
                         cases.fmt(r['value']), MEAN_MAX_MS, bool(r['passed']), 'ms')
        self.assertGreaterEqual(r['count'], SPEC_SAMPLES, 'response_time.csv 샘플 부족')
        self.assertTrue(r['passed'], f"amr_evaluation 평균 {r['value']} ms > {MEAN_MAX_MS} ms")
