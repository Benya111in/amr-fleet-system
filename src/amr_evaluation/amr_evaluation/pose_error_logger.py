"""
pose_error_logger: GT 대비 위치 추정 오차 로거 (명세 4.3 / 4.10).

Sub gt_topic (nav_msgs/Odometry, 기본 ground_truth/odom), est_topic (기본 odometry/filtered_map).
추정 샘플마다 GT 를 그 시각에 선형 보간(외삽 금지)해 위치·헤딩 오차를 구하고 GT 속도로
정지/직선/회전 구간을 붙여 pose_error.csv 에 쓴다:
    [timestamp, gt_x, gt_y, est_x, est_y, error, yaw_error, segment]
"""

from amr_evaluation import io
from amr_evaluation import segments
from amr_evaluation.logger_base import EvalLoggerNode
from amr_evaluation.metrics import PoseErrorPairer
from amr_evaluation.msg_adapters import odom_to_sample
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException


class PoseErrorLogger(EvalLoggerNode):
    """GT·추정 시간 정렬 + 오차 기록 노드."""

    def __init__(self, **node_kwargs):
        super().__init__('pose_error_logger', io.POSE_FILE, io.POSE_COLUMNS, **node_kwargs)
        self.declare_parameter('gt_topic', 'ground_truth/odom')
        self.declare_parameter('est_topic', 'odometry/filtered_map')
        self.declare_parameter('stop_linear_threshold', 0.02)     # [m/s]
        self.declare_parameter('stop_angular_threshold', 0.02)    # [rad/s]
        self.declare_parameter('straight_angular_threshold', 0.05)  # [rad/s]
        self.declare_parameter('gt_max_gap', 0.2)      # [s] 이보다 벌어진 GT 사이는 보간하지 않음
        self.declare_parameter('gt_buffer_sec', 5.0)   # [s] GT 보관 길이
        self.declare_parameter('max_wait', 1.0)        # [s] GT 가 따라오길 기다리는 상한
        self.declare_parameter('flush_rate', 20.0)     # [Hz] 대기 추정 처리 주기
        self.declare_parameter('target_samples', 100)  # 명세: 최소 100 시점

        self.pairer = PoseErrorPairer(
            thresholds=segments.MotionThresholds(
                self.p_float('stop_linear_threshold'),
                self.p_float('stop_angular_threshold'),
                self.p_float('straight_angular_threshold')),
            gt_max_age=self.p_float('gt_buffer_sec'),
            gt_max_gap=self.p_float('gt_max_gap'),
            max_wait=self.p_float('max_wait'))
        self.target = int(self.get_parameter('target_samples').value)
        self.open_writer()

        qos = self.sub_qos()
        self.create_subscription(Odometry, self.p_str('gt_topic'), self.on_ground_truth, qos)
        self.create_subscription(Odometry, self.p_str('est_topic'), self.on_estimate, qos)
        self.create_timer(1.0 / max(self.p_float('flush_rate'), 1.0), self.flush)
        self.get_logger().info(
            f"GT {self.p_str('gt_topic')} ↔ 추정 {self.p_str('est_topic')} 오차 기록")

    def on_ground_truth(self, msg: Odometry) -> None:
        sample = odom_to_sample(msg, self.now_sec())
        self.check_clock('ground_truth', sample.t)
        self.pairer.add_ground_truth(sample)

    def on_estimate(self, msg: Odometry) -> None:
        sample = odom_to_sample(msg, self.now_sec())
        self.check_clock('estimate', sample.t)
        self.pairer.add_estimate(sample)

    def flush(self) -> None:
        for row in self.pairer.flush():
            self.write_row(row.as_list())

    def progress_text(self) -> str:
        return (f'{self.rows} 샘플 (목표 {self.target}), 대기 {self.pairer.pending}, '
                f'폐기 {self.pairer.dropped}')

    def on_finish(self) -> None:
        self.flush()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseErrorLogger()
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
