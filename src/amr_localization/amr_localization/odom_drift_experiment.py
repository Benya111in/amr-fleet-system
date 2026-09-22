r"""
odom_drift_experiment: 휠 오도메트리 드리프트 측정 실험 노드 (명세 4.2).

    ros2 run amr_localization odom_drift_experiment --ros-args -r __ns:=/amr_01 \
        -p scenarios:="[straight, rotate, square_ccw, square_cw]" -p repetitions:=3

GT(ground_truth/odom) 되먹임으로 시나리오(drift_scenarios)를 주행하며 wheel_odom 샘플마다 GT 를 그 시각에
선형 보간해 <output_dir>/<scenario>_<rep>.csv 에 기록하고, 끝나면 drift_analysis 로 summary.md 를 쓴다.
각 기록 구간 시작 전에 wheel_odom/reset 을 불러 자세·공분산을 0 에서 시작한다 (NEES 검증).
cmd_vel 은 이 실험 동안 이 노드가 유일한 발행자여야 한다 (Nav2/safety 스택 없이 실행).
"""

import bisect
import collections
import csv
import math
import os
from pathlib import Path
import time
from typing import Deque, List, Optional

from amr_localization import drift_analysis
from amr_localization.drift_scenarios import build_scenario, MotionLimits, ScenarioRunner
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


def yaw_of(q) -> float:
    """쿼터니언 → yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def odom_pose(msg: Odometry):
    """(t, x, y, yaw)."""
    p = msg.pose.pose
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return t, p.position.x, p.position.y, yaw_of(p.orientation)


class OdomDriftExperiment(Node):
    """드리프트 실험 실행 노드."""

    def __init__(self, **kwargs) -> None:
        super().__init__('odom_drift_experiment', **kwargs)
        self.declare_parameter('scenarios', ['straight', 'rotate', 'square_ccw', 'square_cw'])
        self.declare_parameter('repetitions', 3)
        self.declare_parameter('straight_length', 10.0)       # [m]
        self.declare_parameter('rotation_angle', 2.0 * math.pi)  # [rad]
        self.declare_parameter('rect', [5.0, 4.0])             # [m] 직사각형 변
        self.declare_parameter('settle_time', 1.0)             # [s]
        self.declare_parameter('linear_speed', 0.5)            # [m/s]
        self.declare_parameter('angular_speed', 0.5)           # [rad/s]
        self.declare_parameter('linear_accel', 0.5)            # [m/s²]
        self.declare_parameter('angular_accel', 1.0)           # [rad/s²]
        self.declare_parameter('control_rate', 50.0)           # [Hz]
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('gt_topic', 'ground_truth/odom')
        self.declare_parameter('odom_topic', 'wheel_odom')
        self.declare_parameter('ekf_topic', 'odometry/filtered')  # '' = 기록 안 함
        self.declare_parameter('reset_service', 'wheel_odom/reset')
        # '' → $ROS_WS/logs/eval/odom_drift/run_<시각>
        self.declare_parameter('output_dir', '')
        self.declare_parameter('sigma_s', 0.01)                # 리포트 폐형 예측용
        self.declare_parameter('wheel_separation', 0.36)       # [m] 리포트 유효 파라미터 기준
        self.declare_parameter('wheel_radius', 0.0825)         # [m]
        self.declare_parameter('slip_reference_distance', 0.01)   # [m] 슬립 기준 굴림 ℓ_ref
        # [s] (벽시계) 첫 런 전에 wheel_odom/reset 서비스 발견을 기다리는 상한
        self.declare_parameter('service_wait', 15.0)

        g = self.get_parameter
        self.plan = []
        for rep in range(int(g('repetitions').value)):
            for name in g('scenarios').value:
                self.plan.append((name, rep))
        self.limits = MotionLimits(
            linear_speed=float(g('linear_speed').value),
            angular_speed=float(g('angular_speed').value),
            linear_accel=float(g('linear_accel').value),
            angular_accel=float(g('angular_accel').value))
        rect = [float(v) for v in g('rect').value]
        self.scenario_args = dict(straight_length=float(g('straight_length').value),
                                  rotation_angle=float(g('rotation_angle').value),
                                  rect=(rect[0], rect[1]),
                                  settle=float(g('settle_time').value))
        out = g('output_dir').value or os.path.join(
            os.environ.get('ROS_WS', '.'), 'logs', 'eval', 'odom_drift',
            time.strftime('run_%Y%m%d_%H%M%S'))
        self.output_dir = Path(out)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic').value, 10)
        self.gt: List[tuple] = []
        # GT 보다 먼저 도착한 wheel_odom 샘플 대기열: GT 가 그 시각을 지나면 보간해 기록한다
        # (외삽 금지 + 50 Hz GT 지연 때문에 즉시 처리하면 샘플 대부분이 버려진다)
        self.pending: Deque[tuple] = collections.deque(maxlen=2000)
        self.ekf: Optional[tuple] = None
        self.create_subscription(Odometry, g('gt_topic').value, self.on_gt, 50)
        self.create_subscription(Odometry, g('odom_topic').value, self.on_odom, 50)
        if g('ekf_topic').value:
            self.create_subscription(Odometry, g('ekf_topic').value, self.on_ekf, 50)
        self.reset_client = self.create_client(Trigger, g('reset_service').value)

        self.runner: Optional[ScenarioRunner] = None
        self.current = None
        self.writer = None
        self.file = None
        self.rows = 0
        self.waiting_reset = False
        self.reset_pending = False
        self.log_after = -math.inf   # 리셋 응답 이후 샘플만 기록 (전송 중이던 리셋 전 메시지 제외)
        self.finished = False
        self.service_deadline = time.monotonic() + float(g('service_wait').value)
        self.create_timer(1.0 / float(g('control_rate').value), self.on_control)
        self.get_logger().info(
            f'odom drift experiment: {len(self.plan)} runs → {self.output_dir}')

    # ------------------------------------------------------------ 입력
    def on_gt(self, msg: Odometry) -> None:
        self.gt.append(odom_pose(msg))
        if len(self.gt) > 1000:
            del self.gt[:500]
        self.flush_pending()

    def on_ekf(self, msg: Odometry) -> None:
        self.ekf = odom_pose(msg)

    def gt_at(self, t: float):
        """GT 를 시각 t 에 선형 보간 (범위 밖이면 None — 외삽하지 않는다)."""
        times = [s[0] for s in self.gt]
        i = bisect.bisect_left(times, t)
        if i == 0 or i >= len(times):
            if i < len(times) and abs(times[i] - t) < 1e-6:
                return self.gt[i]
            return None
        a, b = self.gt[i - 1], self.gt[i]
        u = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        yaw = a[3] + u * math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        return (t, a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), yaw)

    def on_odom(self, msg: Odometry) -> None:
        if (self.writer is None or self.runner is None or not self.runner.logging
                or self.waiting_reset or self.reset_pending):
            return
        t, x, y, yaw = odom_pose(msg)
        if t <= self.log_after:
            return
        c = msg.pose.covariance
        ekf = self.ekf if self.ekf is not None else (t, math.nan, math.nan, math.nan)
        self.pending.append((t, x, y, yaw, (c[0], c[7], c[35], c[1], c[5], c[11]), ekf))
        self.flush_pending()

    def flush_pending(self, final: bool = False) -> None:
        """GT 가 시각을 지난 대기 샘플을 보간해 기록. final 이면 남은 것은 버린다."""
        while self.pending and self.writer is not None:
            t, x, y, yaw, c, ekf = self.pending[0]
            if self.gt and t > self.gt[-1][0] and not final:
                return  # GT 가 아직 이 시각에 도달하지 않음
            self.pending.popleft()
            gt = self.gt_at(t)
            if gt is None:
                continue
            self.writer.writerow([f'{t:.4f}', f'{gt[1]:.6f}', f'{gt[2]:.6f}', f'{gt[3]:.6f}',
                                  f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}',
                                  *(f'{v:.9g}' for v in c),
                                  f'{ekf[1]:.6f}', f'{ekf[2]:.6f}', f'{ekf[3]:.6f}'])
            self.rows += 1
        if final:
            self.pending.clear()

    # ------------------------------------------------------------ 제어
    def on_control(self) -> None:
        if self.finished or not self.gt:
            return
        now = self.gt[-1][0]
        pose = self.gt[-1][1:]
        if self.runner is None:
            if (not self.reset_client.service_is_ready()
                    and time.monotonic() < self.service_deadline):
                return  # 기동 직후 서비스 발견 대기 (리셋 없이 시작하면 공분산 NEES 비교가 무의미)
            self.start_next()
            return
        self.request_reset_if_logging()
        if self.waiting_reset:
            self.cmd_pub.publish(Twist())
            return
        v, w = self.runner.step(now, pose)
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)
        if self.runner.done:
            self.close_run()
            self.runner = None

    def start_next(self) -> None:
        if not self.plan:
            self.finish()
            return
        name, rep = self.plan.pop(0)
        self.current = (name, rep)
        self.runner = ScenarioRunner(build_scenario(name, rep, **self.scenario_args), self.limits)
        path = self.output_dir / f'{name}_{rep}.csv'
        self.file = path.open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(drift_analysis.COLUMNS)
        self.rows = 0
        self.get_logger().info(f'run {name} #{rep} → {path.name}')
        # 기록 구간 앞 미기록 구간(방향 되돌리기)이 있으면 그 뒤에 리셋해야 하므로 첫 기록 구간에서 리셋
        self.reset_pending = True
        self.request_reset_if_logging()

    def request_reset_if_logging(self) -> None:
        if not self.reset_pending or self.runner is None or not self.runner.logging:
            return
        if not self.reset_client.service_is_ready():
            self.get_logger().warn('wheel_odom/reset 없음: 리셋 없이 기록 (상대 궤적으로 분석)')
            self.reset_pending = False
            return
        self.waiting_reset = True
        self.reset_pending = False
        self.reset_client.call_async(Trigger.Request()).add_done_callback(self.on_reset_done)

    def on_reset_done(self, _future) -> None:
        self.log_after = self.gt[-1][0] + 0.03 if self.gt else -math.inf
        self.waiting_reset = False

    def close_run(self) -> None:
        self.flush_pending(final=True)
        if self.file is not None:
            self.file.close()
            self.get_logger().info(
                f'run {self.current[0]} #{self.current[1]} done: {self.rows} samples')
        self.file = None
        self.writer = None

    def finish(self) -> None:
        self.finished = True
        self.cmd_pub.publish(Twist())
        g = self.get_parameter
        meta = {'sigma_s': float(g('sigma_s').value),
                'separation': float(g('wheel_separation').value),
                'wheel_radius': float(g('wheel_radius').value),
                'slip_reference_distance': float(g('slip_reference_distance').value)}
        runs = drift_analysis.analyze_directory(self.output_dir, meta)
        self.get_logger().info(f'{len(runs)} runs analysed → {self.output_dir}/summary.md')
        for line in (self.output_dir / 'summary.md').read_text(encoding='utf-8').splitlines():
            self.get_logger().info(line)
        raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomDriftExperiment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
