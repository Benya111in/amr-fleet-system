"""
object_localizer_node — 2D 검출 + 깊이 → Pinhole 역투영 → map 3D (components.md §3.4 / §5.4, 명세 4.6).

Sub  perception/detections_2d (vision_msgs/Detection2DArray) ┐ message_filters 근사 시각 동기
     camera/depth/image_raw (sensor_msgs/Image, 32FC1/16UC1) ┘ (깊이 15 Hz 가 트리거, slop 0.017 s)
     camera/camera_info (RGB K — bbox 픽셀 좌표계), camera/depth/camera_info (깊이 K, 없으면 RGB K)
     odometry/filtered_map (nav_msgs/Odometry, 선택) — 로봇 자세 공분산을 map 공분산에 더한다
Pub  perception/detected_objects (amr_msgs/DetectedObjectArray, pose_3d = map,
       distance = base_link 기준)
     perception/detections_3d (vision_msgs/Detection3DArray, 같은 결과 + 3×3 위치 공분산)
TF   target_frame(map) ← 깊이 optical 프레임 (이미지 스탬프), base_link ← optical (거리)

계산은 amr_perception.pinhole (ROS 비의존, pytest) 에 있다. map TF 가 없으면 fallback_frames 순서로
(odom → base_link) 내려가며, 사용한 프레임을 header.frame_id 에 적는다.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

from amr_msgs.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, \
    ObjectHypothesisWithPose

from .markers import CLASS_STYLE, DEFAULT_STYLE
from .pinhole import CameraIntrinsics, LocalizationParams, localize_bbox, OffsetModel, \
    pose_uncertainty_covariance
from .transforms import prefixed_frame, shutdown_tf_listener, threaded_tf_listener, Transform, \
    transform_from_msg

# 연구 브리프 §3.1 표: 광선 방향 표면→중심 오프셋 (μ_δ, σ_δ) [m]
DEFAULT_OFFSETS: Dict[str, Tuple[float, float]] = {
    'box_small': (0.136, 0.026),
    'box_medium': (0.250, 0.034),
    'box_large': (0.306, 0.039),
    'person': (0.12, 0.04),
    'sign': (0.02, 0.02),
    'forklift': (0.5, 0.15),
    'amr': (0.271, 0.052),
}


def depth_to_meters(msg: Image) -> Optional[np.ndarray]:
    """sensor_msgs/Image 깊이 → [m] float 배열 (16UC1 의 0 → NaN). 지원하지 않는 인코딩은 None."""
    if msg.encoding == '32FC1':
        dtype = np.dtype('<f4') if not msg.is_bigendian else np.dtype('>f4')
        row = msg.step // 4
        arr = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, row)
        return arr[:, :msg.width].astype(np.float64)
    if msg.encoding in ('16UC1', 'mono16'):
        dtype = np.dtype('<u2') if not msg.is_bigendian else np.dtype('>u2')
        row = msg.step // 2
        arr = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, row)
        out = arr[:, :msg.width].astype(np.float64) * 1e-3
        out[arr[:, :msg.width] == 0] = np.nan
        return out
    return None


def rgb_to_depth_pixel(u: float, v: float, rgb: CameraIntrinsics,
                       depth: CameraIntrinsics) -> Tuple[float, float]:
    """RGB 픽셀 → 깊이 픽셀 (두 광학 프레임이 같은 원점·축 — RGB-Depth 베이스라인 0 근사)."""
    return ((u - rgb.cx) * depth.fx / rgb.fx + depth.cx,
            (v - rgb.cy) * depth.fy / rgb.fy + depth.cy)


class ObjectLocalizerNode(Node):
    """detections_2d + depth → detected_objects (map)."""

    def __init__(self, **kwargs):
        super().__init__('object_localizer_node', **kwargs)
        p = self.declare_parameter
        p('frame_prefix', '')
        p('target_frame', 'map')
        p('fallback_frames', ['odom', 'base_link'])
        p('base_frame', 'base_link')
        p('sync_slop', 0.017)
        p('sync_queue', 10)
        p('tf_timeout', 0.05)
        p('roi_frac', 0.5)
        p('min_valid_fraction', 0.3)
        p('depth_camera.range_min', 0.20)
        p('depth_camera.range_max', 10.0)
        p('depth_camera.noise_base', 0.005)
        p('depth_camera.noise_quadratic_coeff', 0.002)
        p('add_quadratic_noise', True)
        p('iid_pixels', True)
        p('kappa', 0.05)
        p('sigma_skew_px', 0.0)
        p('mad_k', 3.0)
        p('seed', 0)
        p('use_pose_covariance', True)
        p('log_transforms', True)
        for name, (mu, sd) in DEFAULT_OFFSETS.items():
            p(f'camera_offset.{name}', [mu, sd])
        p('camera_offset.default', [0.0, 0.05])
        p('topics.detections', 'perception/detections_2d')
        p('topics.depth', 'camera/depth/image_raw')
        p('topics.camera_info', 'camera/camera_info')
        p('topics.depth_camera_info', 'camera/depth/camera_info')
        p('topics.odometry', 'odometry/filtered_map')
        p('topics.output', 'perception/detected_objects')
        p('topics.output_3d', 'perception/detections_3d')
        gp = self.get_parameter

        prefix = str(gp('frame_prefix').value)
        self.target_frame = prefixed_frame(prefix, str(gp('target_frame').value))
        self.fallback_frames = [prefixed_frame(prefix, f) for f in gp('fallback_frames').value]
        self.base_frame = prefixed_frame(prefix, str(gp('base_frame').value))
        self.tf_timeout = Duration(seconds=float(gp('tf_timeout').value))
        self.params = LocalizationParams(
            roi_frac=float(gp('roi_frac').value),
            min_valid_fraction=float(gp('min_valid_fraction').value),
            range_min=float(gp('depth_camera.range_min').value),
            range_max=float(gp('depth_camera.range_max').value),
            noise_base=float(gp('depth_camera.noise_base').value),
            noise_quadratic_coeff=float(gp('depth_camera.noise_quadratic_coeff').value),
            add_quadratic_noise=bool(gp('add_quadratic_noise').value),
            iid_pixels=bool(gp('iid_pixels').value),
            kappa=float(gp('kappa').value),
            sigma_skew_px=float(gp('sigma_skew_px').value),
            mad_k=float(gp('mad_k').value))
        table = {}
        for name in DEFAULT_OFFSETS:
            mu, sd = gp(f'camera_offset.{name}').value
            table[name] = (float(mu), float(sd))
        dmu, dsd = gp('camera_offset.default').value
        self.offsets = OffsetModel(table, (float(dmu), float(dsd)))
        seed = int(gp('seed').value)
        self.rng = np.random.default_rng(seed if seed > 0 else None)
        self.use_pose_covariance = bool(gp('use_pose_covariance').value)
        self.log_transforms = bool(gp('log_transforms').value)

        self.rgb_k: Optional[CameraIntrinsics] = None
        self.depth_k: Optional[CameraIntrinsics] = None
        self.pose_cov: Optional[np.ndarray] = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = threaded_tf_listener(self.tf_buffer)   # 전용 스레드 (transforms.py)

        self.pub = self.create_publisher(DetectedObjectArray, gp('topics.output').value, 10)
        self.pub_3d = self.create_publisher(Detection3DArray, gp('topics.output_3d').value, 10)
        self.create_subscription(CameraInfo, gp('topics.camera_info').value, self.on_rgb_info,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, gp('topics.depth_camera_info').value,
                                 self.on_depth_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, gp('topics.odometry').value, self.on_odometry, 10)
        self.det_sub = Subscriber(self, Detection2DArray, gp('topics.detections').value)
        self.depth_sub = Subscriber(self, Image, gp('topics.depth').value,
                                    qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.det_sub, self.depth_sub], int(gp('sync_queue').value),
            float(gp('sync_slop').value))
        self.sync.registerCallback(self.on_synced)
        self.get_logger().info(
            f'출력 프레임 {self.target_frame} (대체 {self.fallback_frames}), base {self.base_frame}, '
            f'ROI {self.params.roi_frac:.2f}, 깊이 잡음 k={self.params.noise_quadratic_coeff} '
            f'({"가산" if self.params.add_quadratic_noise else "끔"})')

    # ------------------------------------------------------------------ 입력
    def on_rgb_info(self, msg: CameraInfo) -> None:
        self.rgb_k = CameraIntrinsics.from_k(msg.k, msg.width, msg.height)

    def on_depth_info(self, msg: CameraInfo) -> None:
        self.depth_k = CameraIntrinsics.from_k(msg.k, msg.width, msg.height)

    def on_odometry(self, msg: Odometry) -> None:
        c = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        idx = [0, 1, 5]
        self.pose_cov = c[np.ix_(idx, idx)]

    def destroy_node(self) -> bool:
        """TF 전용 스레드·내부 노드까지 정리한다."""
        shutdown_tf_listener(self.tf_listener)
        self.tf_listener = None
        return super().destroy_node()

    def lookup(self, target: str, source: str, stamp) -> Optional[Transform]:
        try:
            tf = self.tf_buffer.lookup_transform(target, source, stamp, self.tf_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().debug(f'TF {target} <- {source} 실패: {exc}')
            return None
        return transform_from_msg(tf.transform)

    # ------------------------------------------------------------------ 처리
    def on_synced(self, dets: Detection2DArray, depth: Image) -> None:
        out = self.process(dets, depth)
        if out is not None:
            self.pub.publish(out[0])
            self.pub_3d.publish(out[1])

    def process(self, dets: Detection2DArray,
                depth: Image) -> Optional[Tuple[DetectedObjectArray, Detection3DArray]]:
        """동기된 한 쌍 → (DetectedObjectArray, Detection3DArray). 준비 안 됐으면 None."""
        if self.rgb_k is None and self.depth_k is None:
            self.get_logger().warn('camera_info 대기 중', throttle_duration_sec=5.0)
            return None
        rgb_k = self.rgb_k or self.depth_k
        depth_k = self.depth_k or self.rgb_k
        depth_m = depth_to_meters(depth)
        if depth_m is None:
            self.get_logger().warn(f'지원하지 않는 깊이 인코딩 {depth.encoding}',
                                   throttle_duration_sec=5.0)
            return None
        stamp = Time.from_msg(depth.header.stamp)
        optical = depth.header.frame_id
        frame, tf_target = None, None
        for cand in [self.target_frame] + self.fallback_frames:
            tf_target = self.lookup(cand, optical, stamp)
            if tf_target is not None:
                frame = cand
                break
        tf_base = self.lookup(self.base_frame, optical, stamp)
        if tf_target is None or tf_base is None:
            self.get_logger().warn(f'TF 없음 ({self.target_frame}/{self.base_frame} <- {optical})',
                                   throttle_duration_sec=5.0)
            return None
        if frame != self.target_frame:
            self.get_logger().warn(f'{self.target_frame} TF 없음 → {frame} 로 출력',
                                   throttle_duration_sec=5.0)
        robot_xy = None
        if self.use_pose_covariance and self.pose_cov is not None and frame == self.target_frame:
            tf_robot = self.lookup(frame, self.base_frame, stamp)
            robot_xy = None if tf_robot is None else tf_robot.translation[:2]

        out = DetectedObjectArray()
        out.header.stamp = depth.header.stamp
        out.header.frame_id = frame
        out3 = Detection3DArray()
        out3.header = out.header
        for det in dets.detections:
            obj = self.localize_one(det, depth_m, rgb_k, depth_k, tf_target, tf_base, robot_xy,
                                    out.header)
            if obj is None:
                continue
            out.objects.append(obj[0])
            out3.detections.append(obj[1])
        return out, out3

    def localize_one(self, det, depth_m: np.ndarray, rgb_k: CameraIntrinsics,
                     depth_k: CameraIntrinsics, tf_target: Transform, tf_base: Transform,
                     robot_xy, header):
        if not det.results:
            return None
        hyp = det.results[0].hypothesis
        class_name = str(hyp.class_id)
        cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
        du, dv = rgb_to_depth_pixel(cx, cy, rgb_k, depth_k)
        sx = depth_k.fx / rgb_k.fx
        sy = depth_k.fy / rgb_k.fy
        loc = localize_bbox(depth_m, (du, dv), (det.bbox.size_x * sx, det.bbox.size_y * sy),
                            class_name, depth_k, self.params, self.offsets, self.rng)
        if loc is None:
            return None
        p_target = tf_target.apply(loc.center)
        cov = tf_target.rotate_covariance(loc.covariance)
        if robot_xy is not None and self.pose_cov is not None:
            cov = cov + pose_uncertainty_covariance(p_target, robot_xy, self.pose_cov)
        p_base = tf_base.apply(loc.center)
        distance = float(np.linalg.norm(p_base))

        obj = DetectedObject()
        obj.header = header
        obj.class_name = class_name
        try:
            obj.class_id = max(int(det.id), 0)
        except ValueError:
            obj.class_id = 0
        obj.confidence = float(hyp.score)
        obj.bbox_x = int(max(cx - det.bbox.size_x / 2.0, 0.0))
        obj.bbox_y = int(max(cy - det.bbox.size_y / 2.0, 0.0))
        obj.bbox_width = int(max(det.bbox.size_x, 0.0))
        obj.bbox_height = int(max(det.bbox.size_y, 0.0))
        obj.pose_3d = PoseStamped()
        obj.pose_3d.header = header
        obj.pose_3d.pose.position.x = float(p_target[0])
        obj.pose_3d.pose.position.y = float(p_target[1])
        obj.pose_3d.pose.position.z = float(p_target[2])
        obj.pose_3d.pose.orientation.w = 1.0
        obj.distance = distance

        d3 = Detection3D()
        d3.header = header
        d3.id = str(obj.class_id)
        hyp3 = ObjectHypothesisWithPose()
        hyp3.hypothesis.class_id = class_name
        hyp3.hypothesis.score = float(hyp.score)
        hyp3.pose.pose = obj.pose_3d.pose
        c6 = np.zeros((6, 6))
        c6[:3, :3] = cov
        c6[3:, 3:] = np.eye(3) * math.pi ** 2  # 자세(방향)는 추정하지 않는다
        hyp3.pose.covariance = [float(v) for v in c6.ravel()]
        d3.results.append(hyp3)
        d3.bbox.center = obj.pose_3d.pose
        size = CLASS_STYLE.get(class_name, DEFAULT_STYLE)[1]
        d3.bbox.size.x, d3.bbox.size.y, d3.bbox.size.z = (float(v) for v in size)
        if self.log_transforms:
            self.get_logger().info(
                f'Transform 2D({cx:.0f}, {cy:.0f}) to 3D({header.frame_id} frame: '
                f'x={p_target[0]:.2f}, y={p_target[1]:.2f}, z={p_target[2]:.2f}), '
                f'{class_name} dist {distance:.2f} m, σ_xy '
                f'{math.sqrt(max(cov[0, 0] + cov[1, 1], 0.0)):.3f} m',
                throttle_duration_sec=2.0)
        return obj, d3


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
