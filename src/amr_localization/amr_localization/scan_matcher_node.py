"""
scan_matcher_node: 스캔 → 정적 지도 점-대-면 정합 → map EKF 자세 측정 (명세 4.3 정확도).

인터페이스 (docs/algorithms/slam.md §4.3)
  Sub  scan_filtered        sensor_msgs/LaserScan
  Sub  /map                 nav_msgs/OccupancyGrid (transient_local) — 면 점·법선을 맵당 1 회 만든다
  Sub  localization/lost    std_msgs/Bool (latched) — 위치 상실 중에는 측정을 내지 않는다
  TF   map → <prefix>base_footprint (스캔 시각) — 초기 추정 (map EKF 가 낸 자세)
  Pub  scan_match_pose      geometry_msgs/PoseWithCovarianceStamped (frame map, 스캔 시각)
                            → ekf_filter_node_map pose1
정합·공분산 식은 amr_localization.scan_registration (rclpy 비의존, pytest 대상).
EKF 쪽 이상치 거부는 pose1_rejection_threshold (Mahalanobis, config/ekf.yaml).
"""

import math
import time
from typing import Optional

from amr_localization import scan_registration as sr
from amr_localization.global_seed import base_beams
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import tf2_ros


def yaw_of(q) -> float:
    """쿼터니언 → yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanMatcherNode(Node):
    """스캔-지도 정합 측정 노드."""

    def __init__(self, **kwargs) -> None:
        super().__init__('scan_matcher_node', **kwargs)
        defaults = sr.RegistrationParams()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('scan_topic', 'scan_filtered')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', 'scan_match_pose')
        self.declare_parameter('lost_topic', 'localization/lost')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('lidar_offset', [0.15, 0.0, 0.0])    # [m, m, rad] base→lidar 2D
        self.declare_parameter('max_beams', 360)
        self.declare_parameter('max_rate', 10.0)                     # [Hz] 스캔 처리 상한
        self.declare_parameter('tf_timeout', 0.05)                   # [s]
        self.declare_parameter('max_tf_age', 0.5)                    # [s] 최근 TF 대체 허용
        self.declare_parameter('occupied_thresh', 65)
        self.declare_parameter('stats_log_period', 30.0)             # [s]
        self.params = sr.RegistrationParams(**{
            name: type(value)(self.get_parameter(name).value)
            for name, value in vars(defaults).items()})
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_offset = tuple(float(v) for v in self.get_parameter('lidar_offset').value)
        self.max_beams = int(self.get_parameter('max_beams').value)
        self.min_period = 1.0 / max(float(self.get_parameter('max_rate').value), 0.1)
        self.tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout').value))
        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)
        self.max_tf_age = float(self.get_parameter('max_tf_age').value)

        self.surface: Optional[sr.SurfaceMap] = None
        self.lost = False
        self.last_stamp = -math.inf
        self.stats = {'ok': 0, 'rejected': 0, 'no_tf': 0, 'latest_tf': 0, 'rms': 0.0,
                      'ratio': 0.0, 'correction': 0.0, 'ms': 0.0}
        self.reasons = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         self.get_parameter('output_topic').value, 10)
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                 self.on_map, latched)
        self.create_subscription(Bool, self.get_parameter('lost_topic').value, self.on_lost,
                                 latched)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        period = float(self.get_parameter('stats_log_period').value)
        if period > 0.0:
            self.create_timer(period, self.log_stats)

    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        if abs(yaw_of(info.origin.orientation)) > 1e-6:
            self.get_logger().warn('map origin yaw != 0 is ignored (AMCL/costmap convention)')
        t0 = time.monotonic()
        self.surface = sr.SurfaceMap.from_occupancy_grid(
            msg.data, info.width, info.height, info.resolution, info.origin.position.x,
            info.origin.position.y, self.occupied_thresh, params=self.params)
        self.get_logger().info(f'map surface: {len(self.surface)} points in '
                               f'{time.monotonic() - t0:.2f} s')

    def on_lost(self, msg: Bool) -> None:
        self.lost = bool(msg.data)

    def initial_pose(self, stamp) -> Optional[sr.Pose]:
        """
        정합 초기 추정 = map → base (map EKF TF). 없으면 None.

        스캔 시각 TF 가 아직 없으면(EKF TF 가 스캔보다 늦게 옴) 가장 최근 TF 를 쓴다: 초기 추정은 정합 분지
        (±0.25 m) 안이기만 하면 되고 결과 자세는 스캔 끝점이 정하므로 스캔 시각 자세다. 최근 TF 가 스캔과
        max_tf_age 이상 떨어져 있으면 건너뛴다.
        """
        target = Time.from_msg(stamp)
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, target,
                                                 self.tf_timeout)
        except tf2_ros.ExtrapolationException:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return None
            age = abs((target - Time.from_msg(tf.header.stamp)).nanoseconds) * 1e-9
            if age > self.max_tf_age:
                return None
            self.stats['latest_tf'] += 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            return None
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def on_scan(self, scan: LaserScan) -> None:
        if self.surface is None or self.lost:
            return
        t_scan = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        if t_scan - self.last_stamp < self.min_period * 0.9:
            return
        self.last_stamp = t_scan
        init = self.initial_pose(scan.header.stamp)
        if init is None:
            self.stats['no_tf'] += 1
            return
        t0 = time.monotonic()
        beams = base_beams(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max,
                           self.max_beams, self.lidar_offset)
        res = sr.register(self.surface, beams, init, self.params)
        self.stats['ms'] += (time.monotonic() - t0) * 1e3
        if not res.ok:
            self.stats['rejected'] += 1
            key = res.reason.split(' ')[0]
            self.reasons[key] = self.reasons.get(key, 0) + 1
            return
        self.stats['ok'] += 1
        self.stats['rms'] += res.rms_residual
        self.stats['ratio'] += res.inlier_ratio
        self.stats['correction'] += math.hypot(res.pose[0] - init[0], res.pose[1] - init[1])
        self.publish(scan.header.stamp, res)

    def publish(self, stamp, res: sr.RegistrationResult) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        x, y, yaw = res.pose
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
        cov = [0.0] * 36
        idx = (0, 1, 5)
        for i in range(3):
            for j in range(3):
                cov[idx[i] * 6 + idx[j]] = float(res.covariance[i, j])
        cov[2 * 6 + 2] = cov[3 * 6 + 3] = cov[4 * 6 + 4] = 1e3   # 평면 밖 (two_d_mode 에서 무시)
        msg.pose.covariance = cov
        self.pub.publish(msg)

    def log_stats(self) -> None:
        s = self.stats
        n = max(s['ok'], 1)
        m = max(s['ok'] + s['rejected'], 1)
        self.get_logger().info(
            f"scan match: {s['ok']} ok / {s['rejected']} rejected {self.reasons} / "
            f"{s['no_tf']} no-tf ({s['latest_tf']} latest-tf), rms {1e3 * s['rms'] / n:.1f} mm, "
            f"inlier {s['ratio'] / n:.2f}, "
            f"correction {1e3 * s['correction'] / n:.1f} mm, {s['ms'] / m:.1f} ms/scan")
        self.stats = {k: 0 if isinstance(v, int) else 0.0 for k, v in s.items()}
        self.reasons = {}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanMatcherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
