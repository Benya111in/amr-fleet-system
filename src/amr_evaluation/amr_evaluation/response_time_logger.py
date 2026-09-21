"""
response_time_logger: 명령 → 첫 움직임 지연 로거 (명세 4.10 "시스템 평균 응답 시간 200 ms").

명령 원천 (cmd_type):
  task: cmd_topic (기본 /fleet/task_events, amr_msgs/Task) 에서 status 가 IN_PROGRESS 로
        전이하는 순간. robot_id 파라미터가 비어 있지 않으면 그 로봇의 작업만 센다.
  pose: cmd_topic 의 geometry_msgs/PoseStamped 하나하나 (목표 자세 직접 발행 실험용).
움직임 원천 (motion_type):
  odom:  motion_topic (nav_msgs/Odometry, 기본 ground_truth/odom) — 실제 움직임 (헤더 스탬프).
  twist: motion_topic 의 geometry_msgs/Twist (예: cmd_vel) — 첫 속도 명령 (수신 시각).
         sequences.md §1 의 "첫 cmd_vel ≠ 0" 정의로 잴 때 (motion_threshold 를 작게).
|v| ≥ motion_threshold 또는 |w| ≥ angular_threshold 인 첫 샘플이 응답이다. response_time.csv:
    [cmd_time, response_time, latency_ms, cmd_id]
"""

from amr_evaluation import io
from amr_evaluation.logger_base import EvalLoggerNode
from amr_evaluation.metrics import ResponseTimeMatcher
from amr_evaluation.msg_adapters import odom_to_sample, stamp_of, twist_to_velocity
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException

IN_PROGRESS = 1   # amr_msgs/Task STATUS_IN_PROGRESS (메시지 상수와 동일, 실행 시 재확인)


class ResponseTimeLogger(EvalLoggerNode):
    """작업 명령과 첫 움직임을 짝지어 지연을 기록하는 노드."""

    def __init__(self, **node_kwargs):
        super().__init__('response_time_logger', io.RESPONSE_FILE, io.RESPONSE_COLUMNS,
                         **node_kwargs)
        self.declare_parameter('cmd_topic', '/fleet/task_events')
        self.declare_parameter('cmd_type', 'task')          # 'task' | 'pose'
        self.declare_parameter('motion_topic', 'ground_truth/odom')
        self.declare_parameter('motion_type', 'odom')       # 'odom' | 'twist'
        self.declare_parameter('motion_threshold', 0.05)    # [m/s]
        self.declare_parameter('angular_threshold', 0.1)    # [rad/s]
        self.declare_parameter('robot_id', '')              # task 모드: 이 로봇의 작업만
        self.declare_parameter('require_rest', True)        # 이미 움직이는 중의 명령은 제외
        self.declare_parameter('timeout', 10.0)             # [s] 무응답 판정
        self.declare_parameter('target_samples', 50)        # 명세: 50 회 이상

        cmd_type = self.p_str('cmd_type')
        motion_type = self.p_str('motion_type')
        if cmd_type not in ('task', 'pose'):
            raise ValueError(f"cmd_type must be 'task' or 'pose', got '{cmd_type}'")
        if motion_type not in ('odom', 'twist'):
            raise ValueError(f"motion_type must be 'odom' or 'twist', got '{motion_type}'")

        self.matcher = ResponseTimeMatcher(
            self.p_float('motion_threshold'), self.p_float('angular_threshold'),
            self.p_bool('require_rest'), self.p_float('timeout'))
        self.robot_id = self.p_str('robot_id')
        self.target = int(self.get_parameter('target_samples').value)
        self._task_status = {}
        self._pose_seq = 0
        self.commands = 0
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
            f"명령 {self.p_str('cmd_topic')} ({cmd_type}) → "
            f"움직임 {self.p_str('motion_topic')} ({motion_type})")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def command(self, t: float, cmd_id: str) -> None:
        self.commands += 1
        if not self.matcher.add_command(t, cmd_id):
            self.get_logger().debug(f'명령 {cmd_id}: 이미 움직이는 중이라 제외')

    def on_task(self, msg) -> None:
        """Task 상태 전이 → IN_PROGRESS 진입 시각을 명령 시각으로."""
        if self.robot_id and msg.robot_id != self.robot_id:
            return
        prev = self._task_status.get(msg.task_id)
        self._task_status[msg.task_id] = int(msg.status)
        if int(msg.status) == self.in_progress and prev != self.in_progress:
            self.command(stamp_of(msg.header, self.now_sec()), msg.task_id)

    def on_goal_pose(self, msg: PoseStamped) -> None:
        self._pose_seq += 1
        self.command(stamp_of(msg.header, self.now_sec()), f'goal_{self._pose_seq}')

    def on_motion(self, msg: Odometry) -> None:
        s = odom_to_sample(msg, self.now_sec())
        self._motion(s.t, s.v, s.w)

    def on_twist(self, msg: Twist) -> None:
        """Twist 는 헤더가 없어 수신 시각을 응답 시각으로 쓴다."""
        v, w = twist_to_velocity(msg)
        self._motion(self.now_sec(), v, w)

    def _motion(self, t: float, v: float, w: float) -> None:
        for row in self.matcher.add_motion(t, v, w):
            self.write_row(row.as_list())

    def progress_text(self) -> str:
        return (f'{self.rows} 샘플 (목표 {self.target}), 명령 {self.commands}, '
                f'대기 {self.matcher.pending}, 주행 중 제외 {self.matcher.skipped_moving}, '
                f'무응답 {self.matcher.timed_out}')


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
