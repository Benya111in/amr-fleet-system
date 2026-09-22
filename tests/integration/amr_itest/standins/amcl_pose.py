"""
amcl_pose: amcl 대역 (components.md §5.2 — Pub amcl_pose, map 프레임 절대 위치).

실제 AMCL 은 Gazebo 스캔 + 저장된 지도(/map)가 있어야 돌아서, 운동학 백엔드에서는 이 대역이
ground_truth/odom 에 가우시안 노이즈를 더한 자세를 rate [Hz] 로 발행한다. 스탬프는 GT 표본
시각이고 발행은 latency [s] 뒤라서(AMCL 의 스캔 처리 지연) ekf_filter_node_map 의
smooth_lagged_data 경로도 함께 시험된다. 공분산은 노이즈 σ² 그대로 싣는다.
"""

from collections import deque
import math
from typing import Deque, Tuple

from amr_itest.standins.common import RELIABLE, run, StandinNode
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np


class AmclPose(StandinNode):
    """GT + 노이즈 → amcl_pose."""

    def __init__(self):
        super().__init__('amcl')
        self.rng = np.random.default_rng(int(self.param('seed', 13)))
        self.sigma_xy = self.param('sigma_xy', 0.02)
        self.sigma_yaw = self.param('sigma_yaw', 0.01)
        self.latency = self.param('latency', 0.1)
        self.period = 1.0 / self.param('rate', 2.0)
        self.frame_id = self.frame(str(self.param('global_frame', 'map')))
        self.last_gt = None
        self.queue: Deque[Tuple[float, PoseWithCovarianceStamped]] = deque()
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', RELIABLE)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, RELIABLE)
        self.create_timer(self.period, self.sample)
        self.create_timer(0.01, self.release)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, msg: Odometry) -> None:
        self.last_gt = msg

    def sample(self) -> None:
        gt = self.last_gt
        if gt is None:
            return
        q = gt.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        nx, ny = self.rng.normal(0.0, self.sigma_xy, 2) if self.sigma_xy > 0 else (0.0, 0.0)
        nyaw = self.rng.normal(0.0, self.sigma_yaw) if self.sigma_yaw > 0 else 0.0
        out = PoseWithCovarianceStamped()
        out.header.stamp = gt.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.pose.position.x = gt.pose.pose.position.x + float(nx)
        out.pose.pose.position.y = gt.pose.pose.position.y + float(ny)
        yaw += float(nyaw)
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = max(self.sigma_xy ** 2, 1e-6)
        cov[35] = max(self.sigma_yaw ** 2, 1e-6)
        out.pose.covariance = cov
        self.queue.append((self._now() + self.latency, out))

    def release(self) -> None:
        now = self._now()
        while self.queue and self.queue[0][0] <= now:
            self.pub.publish(self.queue.popleft()[1])


def main(args=None) -> None:
    run(AmclPose, args)


if __name__ == '__main__':
    main()
