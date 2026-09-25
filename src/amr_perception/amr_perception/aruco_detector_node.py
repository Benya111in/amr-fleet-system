"""
aruco_detector_node — 도킹 ArUco 마커 자세 (components.md §3.4 / §5.4, 명세 4.8 도킹).

Sub  camera/image_raw (sensor_msgs/Image, sensor QoS depth 1), camera/camera_info (K, D)
Sub  perception/aruco/preferred_id (std_msgs/Int32, 여러 마커 중 우선할 id, -1 = 가장 가까운 것)
Pub  perception/dock_marker_pose (geometry_msgs/PoseStamped, frame = base_link, 미검출 시 미발행)
     perception/dock_marker_id (std_msgs/Int32, 같은 주기 — 발행한 마커의 id)
     perception/dock_marker_pose_cov (geometry_msgs/PoseWithCovarianceStamped, base_link, 재투영
       자코비안 기반 공분산 — 도킹 EKF 입력용 확장)
     perception/aruco_image (sensor_msgs/Image, publish_debug_image 일 때)
Srv  perception/aruco/enable (std_srvs/SetBool) — 도킹 중에만 켜려면 enabled:=false 로 띄우고 켠다

자세 규약: 위치 = 마커 중심, 자세 = 마커 모델 프레임 (x = 면 바깥 법선, z = 위; Gazebo dock_marker
모델과 같다). 정면으로 마주 보면 yaw = π. 계산은 amr_perception.aruco (ROS 비의존, pytest).
"""

from __future__ import annotations

from typing import List, Optional

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
import tf2_ros

from .aruco import ArucoPoseEstimator, MarkerDetection
from .transforms import matrix_to_quat, prefixed_frame, shutdown_tf_listener, \
    threaded_tf_listener, Transform, transform_from_msg


def select_marker(dets: List[MarkerDetection],
                  preferred: Optional[int]) -> Optional[MarkerDetection]:
    """선호 id 가 보이면 그것, 아니면 가장 가까운 마커."""
    if not dets:
        return None
    if preferred is not None and preferred >= 0:
        same = [d for d in dets if d.marker_id == preferred]
        if same:
            return min(same, key=lambda d: d.distance)
    return min(dets, key=lambda d: d.distance)


def to_base(det: MarkerDetection, tf_base: Transform):
    """광학 프레임 검출 → base 프레임 (위치, 회전 행렬, 6×6 공분산)."""
    p = tf_base.apply(det.translation)
    r = tf_base.rotation @ det.rotation
    cov = None
    if det.covariance is not None:
        big = np.zeros((6, 6))
        big[:3, :3] = tf_base.rotation
        big[3:, 3:] = tf_base.rotation
        cov = big @ det.covariance @ big.T
    return p, r, cov


class ArucoDetectorNode(Node):
    """camera/image_raw → ArUco → perception/dock_marker_pose (base_link)."""

    def __init__(self, **kwargs):
        super().__init__('aruco_detector_node', **kwargs)
        p = self.declare_parameter
        p('enabled', True)
        p('dictionary', 'DICT_4X4_50')
        p('marker_size', 0.18)
        p('marker_ids', [-1])
        p('preferred_id', -1)          # 시작값 (topics.preferred_id 로 바꿀 수 있다)
        p('min_side_px', 12.0)
        p('ambiguity_px', 1.0)
        p('use_upright_prior', True)
        p('frame_prefix', '')
        p('base_frame', 'base_link')
        p('tf_timeout', 0.1)
        p('publish_debug_image', False)
        p('topics.image', 'camera/image_raw')
        p('topics.camera_info', 'camera/camera_info')
        p('topics.pose', 'perception/dock_marker_pose')
        p('topics.id', 'perception/dock_marker_id')
        p('topics.pose_cov', 'perception/dock_marker_pose_cov')
        p('topics.preferred_id', 'perception/aruco/preferred_id')
        p('topics.debug_image', 'perception/aruco_image')
        p('topics.enable_service', 'perception/aruco/enable')
        gp = self.get_parameter

        self.enabled = bool(gp('enabled').value)
        self.estimator = ArucoPoseEstimator(
            str(gp('dictionary').value), float(gp('marker_size').value),
            float(gp('min_side_px').value), ambiguity_px=float(gp('ambiguity_px').value))
        ids = [int(i) for i in gp('marker_ids').value if int(i) >= 0]
        self.marker_ids = ids or None
        self.preferred = int(gp('preferred_id').value)
        self.use_upright = bool(gp('use_upright_prior').value)
        self.base_frame = prefixed_frame(str(gp('frame_prefix').value),
                                         str(gp('base_frame').value))
        self.tf_timeout = Duration(seconds=float(gp('tf_timeout').value))
        self.bridge = CvBridge()
        self.k: Optional[np.ndarray] = None
        self.d: Optional[np.ndarray] = None
        self.tf_cache = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = threaded_tf_listener(self.tf_buffer)   # 전용 스레드 (transforms.py)

        self.pose_pub = self.create_publisher(PoseStamped, gp('topics.pose').value, 10)
        self.id_pub = self.create_publisher(Int32, gp('topics.id').value, 10)
        self.cov_pub = self.create_publisher(PoseWithCovarianceStamped,
                                             gp('topics.pose_cov').value, 10)
        self.debug_pub = (self.create_publisher(Image, gp('topics.debug_image').value, 1)
                          if bool(gp('publish_debug_image').value) else None)
        sensor_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, gp('topics.camera_info').value, self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, gp('topics.image').value, self.on_image, sensor_qos)
        self.create_service(SetBool, gp('topics.enable_service').value, self.on_enable)
        # 여러 마커가 보일 때 어느 것을 낼지 실행 중에 바꾼다. 위치 표지 마커(기둥·랙)가 생기면서
        # 도크 앞에서도 랙 마커가 더 가까워 도크 마커가 안 나오는 일이 생겼다 (통합 10 실측:
        # search_timeout 으로 도킹 3 회 실패, 2 cm 이내 7/10). 도킹 서버가 원하는 id 를 알려준다.
        self.create_subscription(Int32, gp('topics.preferred_id').value, self.on_preferred, 10)
        self.get_logger().info(
            f'ArUco {gp("dictionary").value}, 마커 {self.estimator.marker_size:.3f} m, '
            f'출력 프레임 {self.base_frame}, {"켜짐" if self.enabled else "꺼짐"}')

    def on_enable(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self.enabled = bool(req.data)
        res.success = True
        res.message = 'enabled' if self.enabled else 'disabled'
        return res

    def on_info(self, msg: CameraInfo) -> None:
        self.k = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.d = np.asarray(msg.d, dtype=float) if len(msg.d) else None

    def destroy_node(self) -> bool:
        """TF 전용 스레드·내부 노드까지 정리한다."""
        shutdown_tf_listener(self.tf_listener)
        self.tf_listener = None
        return super().destroy_node()

    def base_from_optical(self, frame: str, stamp) -> Optional[Transform]:
        """카메라 장착은 고정 조인트 → 프레임별로 한 번 조회해 둔다."""
        if frame in self.tf_cache:
            return self.tf_cache[frame]
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, stamp, self.tf_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(f'TF {self.base_frame} <- {frame} 실패: {exc}',
                                   throttle_duration_sec=5.0)
            return None
        self.tf_cache[frame] = transform_from_msg(tf.transform)
        return self.tf_cache[frame]

    def process(self, msg: Image) -> Optional[MarkerDetection]:
        """이미지 한 장 → 선택된 마커 (광학 프레임) 와 발행. 미검출이면 None."""
        if not self.enabled or self.k is None:
            return None
        tf_base = self.base_from_optical(msg.header.frame_id, Time())
        if tf_base is None:
            return None
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        up = tf_base.rotation.T @ np.array([0.0, 0.0, 1.0]) if self.use_upright else None
        dets = self.estimator.detect(img, self.k, self.d, self.marker_ids, up)
        det = select_marker(dets, self.preferred)
        if self.debug_pub is not None:
            self.publish_debug(msg, img, dets)
        if det is None:
            return None
        p, r, cov = to_base(det, tf_base)
        q = matrix_to_quat(r)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in p)
        (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
         pose.pose.orientation.w) = (float(v) for v in q)
        self.pose_pub.publish(pose)
        self.id_pub.publish(Int32(data=int(det.marker_id)))
        pc = PoseWithCovarianceStamped()
        pc.header = pose.header
        pc.pose.pose = pose.pose
        if cov is not None:
            pc.pose.covariance = [float(v) for v in cov.ravel()]
        self.cov_pub.publish(pc)
        return det

    def on_preferred(self, msg: Int32) -> None:
        """우선 마커 id (-1 = 가장 가까운 마커). 도킹 서버가 세션 시작·종료에 낸다."""
        if int(msg.data) != self.preferred:
            self.get_logger().info(f'우선 마커 id {self.preferred} → {int(msg.data)}')
        self.preferred = int(msg.data)

    def publish_debug(self, msg: Image, gray: np.ndarray, dets: List[MarkerDetection]) -> None:
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for d in dets:
            cv2.polylines(vis, [d.corners.astype(np.int32)], True, (0, 255, 0), 2)
            rvec, _ = cv2.Rodrigues(d.rotation)
            cv2.drawFrameAxes(vis, self.k, self.d if self.d is not None else np.zeros(5), rvec,
                              d.translation, 0.5 * self.estimator.marker_size)
            cv2.putText(vis, f'id {d.marker_id} {d.distance:.2f} m', tuple(
                int(v) for v in d.corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out.header = msg.header
        self.debug_pub.publish(out)

    def on_image(self, msg: Image) -> None:
        self.process(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
