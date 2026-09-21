"""
cte_logger: 경로 추종 Cross-Track Error 로거 (명세 4.5 / 4.10).

Sub path_topic (nav_msgs/Path, 기본 plan), gt_topic (nav_msgs/Odometry, 기본 ground_truth/odom).
GT 위치에서 최신 계획 경로(폴리라인)의 최근접 선분까지의 부호 있는 수직 거리를 구하고,
경로 국소 곡률로 직선/곡선 라벨을 붙여 cte.csv 에 쓴다:
    [timestamp, planned_x, planned_y, actual_x, actual_y, cte, segment]
GT 는 world, 경로는 map 프레임이다 — 두 프레임이 일치한다고 가정한다 (시뮬 시작 자세 = 원점).
"""

from typing import List, Optional

from amr_evaluation import io
from amr_evaluation import segments
from amr_evaluation.logger_base import EvalLoggerNode
from amr_evaluation.metrics import cross_track_error
from amr_evaluation.msg_adapters import odom_to_sample, path_to_points
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException


class CteLogger(EvalLoggerNode):
    """계획 경로 대비 GT 궤적의 CTE 기록 노드."""

    def __init__(self, **node_kwargs):
        super().__init__('cte_logger', io.CTE_FILE, io.CTE_COLUMNS, **node_kwargs)
        self.declare_parameter('path_topic', 'plan')
        self.declare_parameter('gt_topic', 'ground_truth/odom')
        self.declare_parameter('curvature_threshold', 0.1)  # [1/m] 초과 시 곡선 (R < 10 m)
        self.declare_parameter('curvature_window', 0.5)     # [m] 곡률 원적합 호길이 창
        self.declare_parameter('curve_margin', 0.5)         # [m] 곡선 앞뒤 천이대도 곡선으로
        self.declare_parameter('log_rate', 20.0)            # [Hz] GT 샘플 데시메이션
        self.declare_parameter('min_path_points', 2)

        self._points = np.zeros((0, 2))
        self._labels: List[str] = []
        self._last_t: Optional[float] = None
        self.paths_received = 0
        self.skipped_no_path = 0
        self.open_writer()

        self.create_subscription(Path, self.p_str('path_topic'), self.on_path, self.sub_qos(5))
        self.create_subscription(Odometry, self.p_str('gt_topic'), self.on_ground_truth,
                                 self.sub_qos())
        self.get_logger().info(
            f"경로 {self.p_str('path_topic')} 대비 GT {self.p_str('gt_topic')} CTE 기록")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def set_path(self, points: np.ndarray) -> None:
        """경로 갱신 + 정점 라벨 계산 (테스트에서 직접 호출 가능)."""
        self._points = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(self._points) < int(self.get_parameter('min_path_points').value):
            self._points = np.zeros((0, 2))
            self._labels = []
            return
        self._labels = segments.label_path_segments(
            self._points, self.p_float('curvature_threshold'),
            self.p_float('curvature_window'), self.p_float('curve_margin'))

    def on_path(self, msg: Path) -> None:
        self.paths_received += 1
        self.set_path(path_to_points(msg))

    def on_ground_truth(self, msg: Odometry) -> None:
        sample = odom_to_sample(msg, self.now_sec())
        if len(self._points) == 0:
            self.skipped_no_path += 1
            return
        rate = self.p_float('log_rate')
        if rate > 0.0 and self._last_t is not None and sample.t - self._last_t < 1.0 / rate:
            return
        res = cross_track_error(self._points, sample.x, sample.y)
        if res is None:
            return
        self._last_t = sample.t
        label = segments.segment_label_for_index(self._labels, res.segment_index)
        self.write_row([sample.t, res.planned_x, res.planned_y, sample.x, sample.y,
                        res.cte, label])

    def progress_text(self) -> str:
        return (f'{self.rows} 샘플, 경로 수신 {self.paths_received} 회, '
                f'경로 없어 건너뜀 {self.skipped_no_path}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CteLogger()
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
