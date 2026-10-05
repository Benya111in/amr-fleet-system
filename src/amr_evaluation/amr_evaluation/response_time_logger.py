"""
response_time_logger: 명령 → 첫 움직임 지연 로거 (명세 4.10 "시스템 평균 응답 시간 200 ms").

측정 구간 기본값 = 명세 표의 "작업 명령 발행 timestamp → 로봇 첫 움직임 timestamp".
명령 원천 (cmd_type):
  task: cmd_topic (기본 /fleet/task_events, amr_msgs/Task) 에서 status 가 IN_PROGRESS 로 전이한 작업.
        명령 시각은 cmd_stamp 로 고른다:
          auto     (기본) pickup_pose.header.stamp 가 0 이 아니고 header.stamp 보다 이르면 그것
                   (= fleet_manager 가 assign_task 를 송신 지연 큐에 넣은 명령 시각, cmd_source=dispatch),
                   아니면 header.stamp (= IN_PROGRESS 전이 시각, cmd_source=event). 대체되면 한 번 경고:
                   전이 시각은 로봇 수락 뒤라 통신 지연·서비스 왕복이 빠져 지연을 적게 잰다.
          pickup   pickup_pose.header.stamp (0 이면 header.stamp 로 대체)
          dropoff  dropoff_pose.header.stamp = 접수 시각 (배치 창·큐 대기까지 포함, cmd_source=request)
          header   header.stamp (예전 정의, cmd_source=event)
        robot_id 가 비어 있지 않으면 그 로봇의 작업만 센다 ('/amr_01' 처럼 줘도 'amr_01' 로 맞춘다).
  pose: cmd_topic 의 geometry_msgs/PoseStamped 하나하나 (목표 자세 직접 발행 실험용, cmd_source=goal).
움직임 원천 (motion_type):
  odom:  motion_topic (nav_msgs/Odometry, 기본 ground_truth/odom) — 실제 움직임 (헤더 스탬프).
  twist: motion_topic 의 geometry_msgs/Twist (예: cmd_vel) — 첫 속도 명령 (수신 시각).
         sequences.md §1 의 "첫 cmd_vel ≠ 0" 정의로 잴 때 (motion_threshold 를 작게).
|v| ≥ motion_threshold 또는 |w| ≥ angular_threshold 인 첫 샘플이 응답이다. response_time.csv:
    [cmd_time, response_time, latency_ms] + cmd_id, cmd_source, rtf, latency_wall_ms
rtf 는 이 노드의 ROS 시각 대 단조 벽시계 비(최근 5 s), latency_wall_ms = latency_ms / rtf —
use_sim_time 이면 스탬프는 시뮬 시간이라 RTF < 1 일 때 벽시계 지연을 적게 보이므로 함께 남긴다.
명령과 움직임 스탬프의 시계 영역(시뮬 vs 벽시계)이 다르면 짝짓지 않고 경고한다 (clock_mismatch).
"""

import time

from amr_evaluation import clocks
from amr_evaluation import io
from amr_evaluation.logger_base import EvalLoggerNode
from amr_evaluation.metrics import ResponseTimeMatcher
from amr_evaluation.msg_adapters import odom_to_sample, stamp_of, twist_to_velocity
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException

IN_PROGRESS = 1   # amr_msgs/Task STATUS_IN_PROGRESS (메시지 상수와 동일, 실행 시 재확인)
CMD_STAMPS = ('auto', 'pickup', 'dropoff', 'header')

SOURCE_DISPATCH = 'dispatch'
SOURCE_EVENT = 'event'
SOURCE_REQUEST = 'request'
SOURCE_GOAL = 'goal'


def normalize_robot_id(value: str) -> str:
    """'/amr_01', 'amr_01/', '/fleet/amr_01' → 'amr_01' (fleet_adapter 의 strip('/') 규약과 같게)."""
    return value.strip().strip('/').split('/')[-1]


def task_command_time(msg, mode: str = 'auto', fallback: float = 0.0):
    """
    IN_PROGRESS Task 이벤트 → (명령 시각 [s], cmd_source). 규칙은 모듈 docstring 의 cmd_stamp.

    스탬프가 모두 0 이면 fallback (수신 시각).
    """
    header = stamp_of(msg.header, fallback)
    pickup = stamp_of(msg.pickup_pose.header)
    if mode == 'header':
        return header, SOURCE_EVENT
    if mode == 'dropoff':
        request = stamp_of(msg.dropoff_pose.header)
        return (request, SOURCE_REQUEST) if request > 0.0 else (header, SOURCE_EVENT)
    if mode == 'pickup':
        return (pickup, SOURCE_DISPATCH) if pickup > 0.0 else (header, SOURCE_EVENT)
    # auto: 명령 시각 필드가 전이 시각보다 앞설 때만 명령 시각으로 믿는다 (옛 fleet 는 같은 값을 싣는다)
    if 0.0 < pickup < header:
        return pickup, SOURCE_DISPATCH
    return header, SOURCE_EVENT


class ResponseTimeLogger(EvalLoggerNode):
    """작업 명령과 첫 움직임을 짝지어 지연을 기록하는 노드."""

    def __init__(self, **node_kwargs):
        super().__init__('response_time_logger', io.RESPONSE_FILE, io.RESPONSE_COLUMNS,
                         **node_kwargs)
        self.declare_parameter('cmd_topic', '/fleet/task_events')
        self.declare_parameter('cmd_type', 'task')          # 'task' | 'pose'
        self.declare_parameter('cmd_stamp', 'auto')         # auto | pickup | dropoff | header
        self.declare_parameter('motion_topic', 'ground_truth/odom')
        self.declare_parameter('motion_type', 'odom')       # 'odom' | 'twist'
        self.declare_parameter('motion_threshold', 0.05)    # [m/s]
        self.declare_parameter('angular_threshold', 0.1)    # [rad/s]
        self.declare_parameter('robot_id', '')              # task 모드: 이 로봇의 작업만
        self.declare_parameter('require_rest', True)        # 이미 움직이는 중의 명령은 제외
        self.declare_parameter('timeout', 10.0)             # [s] 무응답 판정
        self.declare_parameter('max_clock_skew', 3600.0)    # [s] 이보다 떨어지면 시계 불일치
        self.declare_parameter('rtf_window', 5.0)           # [s] RTF 추정 창 (벽시계)
        self.declare_parameter('target_samples', 50)        # 명세: 50 회 이상

        cmd_type = self.p_str('cmd_type')
        motion_type = self.p_str('motion_type')
        self.cmd_stamp = self.p_str('cmd_stamp')
        if cmd_type not in ('task', 'pose'):
            raise ValueError(f"cmd_type must be 'task' or 'pose', got '{cmd_type}'")
        if motion_type not in ('odom', 'twist'):
            raise ValueError(f"motion_type must be 'odom' or 'twist', got '{motion_type}'")
        if self.cmd_stamp not in CMD_STAMPS:
            raise ValueError(f'cmd_stamp must be one of {CMD_STAMPS}, got {self.cmd_stamp!r}')

        self.matcher = ResponseTimeMatcher(
            self.p_float('motion_threshold'), self.p_float('angular_threshold'),
            self.p_bool('require_rest'), self.p_float('timeout'),
            max_skew=self.p_float('max_clock_skew'))
        self.rtf = clocks.RtfEstimator(self.p_float('rtf_window'))
        self.robot_id = normalize_robot_id(self.p_str('robot_id'))
        self.target = int(self.get_parameter('target_samples').value)
        self._task_status = {}
        self._pose_seq = 0
        self._warned_fallback = False
        self._mismatch_warned = False
        self.commands = 0
        self.sources = {}
        self.in_progress = IN_PROGRESS
        self.open_writer()

        if cmd_type == 'task':
            from amr_msgs.msg import Task
            self.in_progress = int(Task.STATUS_IN_PROGRESS)
            self.create_subscription(Task, self.p_str('cmd_topic'), self.on_task,
                                     self.sub_qos())
        else:
            self.create_subscription(PoseStamped, self.p_str('cmd_topic'), self.on_goal_pose,
                                     self.sub_qos())
        if motion_type == 'odom':
            self.create_subscription(Odometry, self.p_str('motion_topic'), self.on_motion,
                                     self.sub_qos())
        else:
            self.create_subscription(Twist, self.p_str('motion_topic'), self.on_twist,
                                     self.sub_qos())
        self.get_logger().info(
            f"명령 {self.p_str('cmd_topic')} ({cmd_type}, cmd_stamp={self.cmd_stamp}) → "
            f"움직임 {self.p_str('motion_topic')} ({motion_type}), robot_id='{self.robot_id}'")

    def command(self, t: float, cmd_id: str, source: str) -> None:
        self.commands += 1
        self.sources[source] = self.sources.get(source, 0) + 1
        self.check_clock('command', t)
        before = self.matcher.clock_mismatch
        if not self.matcher.add_command(t, cmd_id, source):
            if self.matcher.clock_mismatch > before:
                self._warn_mismatch(t)
            else:
                self.get_logger().debug(f'명령 {cmd_id}: 이미 움직이는 중이라 제외')
        self._write(self.matcher.take_ready())

    def _warn_mismatch(self, cmd_t: float) -> None:
        if self._mismatch_warned:
            return
        self._mismatch_warned = True
        last = self.matcher.last_motion_time
        self.get_logger().warn(
            f'명령 스탬프 {cmd_t:.3f} s ({clocks.clock_domain(cmd_t)}) 와 움직임 스탬프 '
            f'{last if last is not None else float("nan"):.3f} s 가 같은 시계가 아니다 — '
            'fleet 와 로거의 use_sim_time 을 맞춘다 (짝짓지 않고 clock_mismatch 로 센다)')

    def on_task(self, msg) -> None:
        """Task 상태 전이 → IN_PROGRESS 진입 작업의 명령 시각."""
        if self.robot_id and normalize_robot_id(msg.robot_id) != self.robot_id:
            return
        prev = self._task_status.get(msg.task_id)
        self._task_status[msg.task_id] = int(msg.status)
        if int(msg.status) != self.in_progress or prev == self.in_progress:
            return
        t, source = task_command_time(msg, self.cmd_stamp, self.now_sec())
        if source == SOURCE_EVENT and self.cmd_stamp != 'header' and not self._warned_fallback:
            self._warned_fallback = True
            self.get_logger().warn(
                f'{msg.task_id}: 명령 시각(pickup_pose.header.stamp)이 없거나 전이 시각보다 이르지 '
                '않아 IN_PROGRESS 전이 시각(header.stamp)으로 잰다 — 할당 통신 지연·서비스 왕복이 빠진다')
        self.command(t, msg.task_id, source)

    def on_goal_pose(self, msg: PoseStamped) -> None:
        self._pose_seq += 1
        self.command(stamp_of(msg.header, self.now_sec()), f'goal_{self._pose_seq}', SOURCE_GOAL)

    def on_motion(self, msg: Odometry) -> None:
        s = odom_to_sample(msg, self.now_sec())
        self.check_clock('motion', s.t)
        self._motion(s.t, s.v, s.w)

    def on_twist(self, msg: Twist) -> None:
        """Twist 는 헤더가 없어 수신 시각을 응답 시각으로 쓴다."""
        v, w = twist_to_velocity(msg)
        self._motion(self.now_sec(), v, w)

    def _motion(self, t: float, v: float, w: float) -> None:
        self.rtf.add(self.now_sec(), time.monotonic())
        self._write(self.matcher.add_motion(t, v, w))

    def _write(self, rows) -> None:
        rtf = self.rtf.value()
        for row in rows:
            row.rtf = rtf
            row.latency_wall_ms = clocks.wall_latency_ms(row.latency_ms, rtf)
            self.write_row(row.as_list())

    def progress_text(self) -> str:
        m = self.matcher
        src = ', '.join(f'{k} {v}' for k, v in sorted(self.sources.items())) or '-'
        return (f'{self.rows} 샘플 (목표 {self.target}), 명령 {self.commands} ({src}), '
                f'대기 {m.pending}, 주행 중 제외 {m.skipped_moving}, 무응답 {m.timed_out}, '
                f'시계 불일치 {m.clock_mismatch}, RTF {self.rtf.value():.2f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ResponseTimeLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
