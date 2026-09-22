"""
cte_logger: 경로 추종 Cross-Track Error 로거 (명세 4.5 / 4.10).

Sub path_topic (nav_msgs/Path, 기본 plan), gt_topic (nav_msgs/Odometry, 기본 ground_truth/odom),
    nav_status_topic (action_msgs/GoalStatusArray, 기본 navigate_to_pose/_action/status),
    phase_topic (std_msgs/String, 기본 executor/phase)
GT 위치에서 최신 계획 경로(폴리라인)의 최근접 선분까지의 부호 있는 수직 거리를 구하고,
평활한 경로의 국소 곡률로 직선/곡선 라벨을 붙여 cte.csv 에 쓴다:
    [timestamp, planned_x, planned_y, actual_x, actual_y, cte, segment]
로봇이 그 경로를 따라가는 중인 표본만 쓴다 — 주차·도킹·적재 중이거나 새 목표의 경로가 아직 안 온
(경로가 목표보다 오래된) 표본, 끝점 너머로 잘린 사영은 버린다. 규칙은 cte_gate 모듈 docstring.
GT 는 world, 경로는 map 프레임이다 — 두 프레임이 일치한다고 가정한다 (시뮬 시작 자세 = 원점).
"""

import math
from typing import List, Optional

from amr_evaluation import io
from amr_evaluation import segments
from amr_evaluation.cte_gate import GateConfig, goal_status_tuples, PlanGate
from amr_evaluation.logger_base import EvalLoggerNode
from amr_evaluation.metrics import cross_track_error
from amr_evaluation.msg_adapters import odom_to_sample, path_to_points, stamp_of
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

RESET_JUMP = 1.0   # [s] GT 시각이 이보다 크게 거꾸로 가면 시뮬 리셋으로 본다


class CteLogger(EvalLoggerNode):
    """계획 경로 대비 GT 궤적의 CTE 기록 노드."""

    def __init__(self, **node_kwargs):
        super().__init__('cte_logger', io.CTE_FILE, io.CTE_COLUMNS, **node_kwargs)
        self.declare_parameter('path_topic', 'plan')
        self.declare_parameter('gt_topic', 'ground_truth/odom')
        self.declare_parameter('nav_status_topic', 'navigate_to_pose/_action/status')  # '' 끔
        self.declare_parameter('phase_topic', 'executor/phase')                        # '' 끔
        self.declare_parameter('active_phases', ['moving'])
        self.declare_parameter('min_speed', 0.02)           # [m/s] 이 미만 + 아래 미만이면 정지
        self.declare_parameter('min_angular', 0.02)         # [rad/s]
        self.declare_parameter('plan_timeout', 3.0)         # [s] 경로 수신 후 이 시간이 지나면 폐기 (0 끔)
        self.declare_parameter('skip_endpoint', True)       # 끝점 너머로 잘린 사영(종방향 거리) 제외
        self.declare_parameter('curvature_threshold', 0.1)  # [1/m] 초과 시 곡선 (R < 10 m)
        self.declare_parameter('curvature_window', segments.DEFAULT_CURVATURE_WINDOW)   # [m]
        self.declare_parameter('curvature_smoothing', segments.DEFAULT_SMOOTHING)       # [m] σ
        self.declare_parameter('min_curve_turn_deg', math.degrees(segments.DEFAULT_MIN_TURN))
        self.declare_parameter('curve_margin', 0.5)         # [m] 곡선 앞뒤 천이대도 곡선으로
        self.declare_parameter('log_rate', 20.0)            # [Hz] GT 데시메이션
        self.declare_parameter('min_path_points', 2)

        self.gate = PlanGate(GateConfig(
            min_speed=self.p_float('min_speed'), min_angular=self.p_float('min_angular'),
            plan_timeout=self.p_float('plan_timeout'),
            active_phases=tuple(str(p).strip().lower()
                                for p in self.get_parameter('active_phases').value)))
        self._points = np.zeros((0, 2))
        self._labels: List[str] = []
        self._last_t: Optional[float] = None
        self.paths_received = 0
        self.resets = 0
        self.open_writer()

        self.create_subscription(Path, self.p_str('path_topic'), self.on_path, self.sub_qos(5))
        self.create_subscription(Odometry, self.p_str('gt_topic'), self.on_ground_truth,
                                 self.sub_qos())
        sources = []
        if self.p_str('nav_status_topic'):
            from action_msgs.msg import GoalStatusArray
            # 액션 서버는 상태를 reliable + transient_local(depth 1) 로 낸다 → 늦게 붙어도 최신 상태를 받는다
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
            self.create_subscription(GoalStatusArray, self.p_str('nav_status_topic'),
                                     self.on_nav_status, qos)
            sources.append(self.p_str('nav_status_topic'))
        if self.p_str('phase_topic'):
            self.create_subscription(String, self.p_str('phase_topic'), self.on_phase,
                                     self.sub_qos(10))
            sources.append(self.p_str('phase_topic'))
        self.get_logger().info(
            f"경로 {self.p_str('path_topic')} 대비 GT {self.p_str('gt_topic')} CTE 기록 "
            f"(주행 상태 원천: {', '.join(sources) or '없음 — 경로 만료·정지·끝점 규칙만'})")

    @property
    def skipped_no_path(self) -> int:
        return self.gate.skipped['no_plan']

    def set_path(self, points: np.ndarray) -> None:
        """경로 갱신 + 정점 라벨 계산 (테스트에서 직접 호출 가능)."""
        self._points = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(self._points) < int(self.get_parameter('min_path_points').value):
            self._points = np.zeros((0, 2))
            self._labels = []
            self.gate.clear_plan()
            return
        self._labels = segments.label_path_segments(
            self._points, self.p_float('curvature_threshold'),
            self.p_float('curvature_window'), self.p_float('curve_margin'),
            self.p_float('curvature_smoothing'), math.radians(self.p_float('min_curve_turn_deg')))

    def on_path(self, msg: Path) -> None:
        self.paths_received += 1
        stamp = stamp_of(msg.header)
        self.check_clock('path', stamp)
        self.gate.on_plan(stamp, self.now_sec())
        self.set_path(path_to_points(msg))

    def on_nav_status(self, msg) -> None:
        self.gate.on_nav_status(goal_status_tuples(msg.status_list))

    def on_phase(self, msg: String) -> None:
        self.gate.on_phase(msg.data, self.now_sec())

    def on_ground_truth(self, msg: Odometry) -> None:
        now = self.now_sec()
        sample = odom_to_sample(msg, now)
        self.check_clock('ground_truth', sample.t)
        if self._last_t is not None and sample.t < self._last_t - RESET_JUMP:
            # 시뮬 리셋: 이전 시간축의 경로·목표 기록은 무효
            self.resets += 1
            self.get_logger().warn(
                f'GT 시각이 {self._last_t:.3f} → {sample.t:.3f} s 로 되돌아감 — 경로를 비우고 다시 시작')
            self._last_t = None
            self.gate.reset()
            self._points = np.zeros((0, 2))
            self._labels = []
        rate = self.p_float('log_rate')
        if rate > 0.0 and self._last_t is not None and sample.t - self._last_t < 1.0 / rate:
            return
        self._last_t = sample.t
        if self.gate.check(now, sample.v, sample.w) is not None or len(self._points) == 0:
            return
        res = cross_track_error(self._points, sample.x, sample.y)
        if self.gate.check_projection(res.clamped and self.p_bool('skip_endpoint')) is not None:
            return
        label = segments.segment_label_for_index(self._labels, res.segment_index)
        self.write_row([sample.t, res.planned_x, res.planned_y, sample.x, sample.y,
                        res.cte, label])

    def progress_text(self) -> str:
        reset = f', 시뮬 리셋 {self.resets}' if self.resets else ''
        return (f'{self.rows} 샘플, 경로 수신 {self.paths_received} 회, '
                f'{self.gate.summary()}{reset}')


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
