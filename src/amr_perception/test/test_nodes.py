"""
ROS 노드 래퍼 테스트 (rclpy, 스핀 없이 콜백·처리 함수를 직접 호출).

TF 는 노드의 tf2 버퍼에 직접 넣는다. object_localizer 의 map 좌표와 aruco 의 base_link 자세를
해석해(Pinhole + 고정 외부 파라미터)와 비교한다.
"""

import math
import time

from amr_msgs.msg import DetectedObject, DetectedObjectArray
from amr_perception import aruco
from amr_perception.aruco_detector_node import ArucoDetectorNode, select_marker, to_base
from amr_perception.detection_marker_node import build_marker_array, DetectionMarkerNode
from amr_perception.object_localizer_node import depth_to_meters, ObjectLocalizerNode, \
    rgb_to_depth_pixel
from amr_perception.pinhole import back_project, CameraIntrinsics
from amr_perception.transforms import matrix_to_quat, rpy_to_matrix, Transform
from amr_perception.yolo_backend import Detection
from amr_perception.yolo_node import draw_detections, to_detection_array, YoloNode
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker

K = CameraIntrinsics.from_fov(640, 480, 1.518436)
STAMP = TimeMsg(sec=100, nanosec=0)
# base_link → camera_link (0.29, 0, 0.07) → optical rpy (-π/2, 0, -π/2)  (sensors.yaml)
T_BASE_OPT = Transform(rpy_to_matrix(-math.pi / 2, 0.0, -math.pi / 2), np.array([0.29, 0.0, 0.07]))
# map → base_link: (2, 1, 0.18), yaw 0.5
T_MAP_BASE = Transform(rpy_to_matrix(0.0, 0.0, 0.5), np.array([2.0, 1.0, 0.18]))


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _tf(parent, child, t: Transform, stamp=STAMP) -> TransformStamped:
    m = TransformStamped()
    m.header.frame_id = parent
    m.header.stamp = stamp
    m.child_frame_id = child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = \
        (float(v) for v in t.translation)
    q = matrix_to_quat(t.rotation)
    (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
     m.transform.rotation.w) = (float(v) for v in q)
    return m


def _info(frame='camera_optical_frame') -> CameraInfo:
    info = CameraInfo()
    info.header.frame_id = frame
    info.width, info.height = 640, 480
    info.k = [K.fx, 0.0, K.cx, 0.0, K.fy, K.cy, 0.0, 0.0, 1.0]
    return info


def _depth_msg(depth: np.ndarray, frame='camera_depth_optical_frame') -> Image:
    msg = CvBridge().cv2_to_imgmsg(depth.astype(np.float32), encoding='32FC1')
    msg.header.frame_id = frame
    msg.header.stamp = STAMP
    return msg


def _detections(cx, cy, w, h, cls='sign', score=0.8, cid='2') -> Detection2DArray:
    arr = Detection2DArray()
    arr.header.stamp = STAMP
    arr.header.frame_id = 'camera_optical_frame'
    d = Detection2D()
    d.id = cid
    d.bbox.center.position.x, d.bbox.center.position.y = cx, cy
    d.bbox.size_x, d.bbox.size_y = w, h
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = cls
    hyp.hypothesis.score = score
    d.results.append(hyp)
    arr.detections.append(d)
    arr.detections.append(Detection2D())      # 결과 없는 검출은 건너뛴다
    return arr


def test_object_localizer_matches_pinhole_and_tf():
    node = ObjectLocalizerNode(parameter_overrides=[
        Parameter('add_quadratic_noise', value=False), Parameter('log_transforms', value=True)])
    node.on_rgb_info(_info())
    node.on_depth_info(_info('camera_depth_optical_frame'))
    odom = Odometry()
    cov = np.zeros((6, 6))
    cov[0, 0] = cov[1, 1] = 0.0025
    cov[5, 5] = 1e-4
    odom.pose.covariance = [float(v) for v in cov.ravel()]
    node.on_odometry(odom)
    node.tf_buffer.set_transform(_tf('map', 'base_link', T_MAP_BASE), 'test')
    node.tf_buffer.set_transform_static(
        _tf('base_link', 'camera_depth_optical_frame', T_BASE_OPT), 'test')
    depth = np.full((480, 640), 8.0)
    depth[170:230, 360:440] = 3.0
    out, out3 = node.process(_detections(400.0, 200.0, 80.0, 60.0), _depth_msg(depth))
    assert out.header.frame_id == 'map' and len(out.objects) == 1
    obj = out.objects[0]
    surface = back_project(400.0, 200.0, 3.0, K)
    center = surface * (1.0 + 0.02 / np.linalg.norm(surface))      # sign 오프셋 0.02 m
    expect_map = T_MAP_BASE.apply(T_BASE_OPT.apply(center))
    p = obj.pose_3d.pose.position
    assert np.allclose([p.x, p.y, p.z], expect_map, atol=1e-6)
    assert obj.distance == pytest.approx(np.linalg.norm(T_BASE_OPT.apply(center)), abs=1e-6)
    assert (obj.class_name, obj.class_id) == ('sign', 2)
    assert obj.confidence == pytest.approx(0.8)
    assert (obj.bbox_x, obj.bbox_y, obj.bbox_width, obj.bbox_height) == (360, 170, 80, 60)
    c3 = np.array(out3.detections[0].results[0].pose.covariance).reshape(6, 6)
    assert np.all(np.linalg.eigvalsh(c3[:3, :3]) > 0.0)
    # 로봇 자세 공분산 (σ_xy 5 cm) 이 더해져 xy 분산 ≥ 0.0025
    assert c3[0, 0] >= 0.0025 and c3[1, 1] >= 0.0025


def test_object_localizer_fallback_and_invalid_inputs():
    node = ObjectLocalizerNode()
    depth = np.full((480, 640), 3.0)
    dets = _detections(320.0, 240.0, 50.0, 50.0, 'box', 0.9, 'x')
    assert node.process(dets, _depth_msg(depth)) is None          # camera_info 없음
    node.on_rgb_info(_info())
    assert node.process(dets, _depth_msg(depth)) is None          # TF 없음
    node.tf_buffer.set_transform_static(
        _tf('base_link', 'camera_depth_optical_frame', T_BASE_OPT), 'test')
    out, _ = node.process(dets, _depth_msg(depth))                # map/odom 없음 → base_link
    assert out.header.frame_id == 'base_link'
    assert out.objects[0].class_id == 0                           # id 파싱 실패 → 0
    assert out.objects[0].pose_3d.pose.position.x > 3.0           # 전방 3 m + 오프셋
    bad = _depth_msg(depth)
    bad.encoding = 'rgb8'
    assert node.process(dets, bad) is None
    node.on_synced(dets, _depth_msg(depth))                        # 발행 경로


def test_depth_decoding_and_pixel_mapping():
    d16 = np.array([[1500, 0], [250, 65535]], dtype=np.uint16)
    msg = CvBridge().cv2_to_imgmsg(d16, encoding='16UC1')
    m = depth_to_meters(msg)
    assert m[0, 0] == pytest.approx(1.5) and np.isnan(m[0, 1]) and m[1, 0] == pytest.approx(0.25)
    f32 = depth_to_meters(_depth_msg(np.full((2, 3), 2.5)))
    assert f32.shape == (2, 3) and np.all(f32 == 2.5)
    half = CameraIntrinsics(K.fx / 2, K.fy / 2, 159.5, 119.5)
    assert rgb_to_depth_pixel(K.cx, K.cy, K, half) == pytest.approx((159.5, 119.5))
    u, v = rgb_to_depth_pixel(K.cx + 100, K.cy - 50, K, half)
    assert (u, v) == pytest.approx((209.5, 94.5))


def test_detection_marker_node_builds_cube_and_text():
    arr = DetectedObjectArray()
    arr.header.frame_id = 'map'
    obj = DetectedObject()
    obj.class_name = 'box'
    obj.confidence = 0.92
    obj.distance = 1.5
    obj.pose_3d.header.frame_id = 'map'
    obj.pose_3d.pose.position.x = 12.5
    obj.pose_3d.pose.position.y = 3.2
    obj.pose_3d.pose.position.z = 0.2
    arr.objects.append(obj)
    m = build_marker_array(arr, 1.0, 0.25, capitalize=True)
    assert [x.action for x in m.markers] == [Marker.DELETEALL, Marker.ADD, Marker.ADD]
    cube, text = m.markers[1], m.markers[2]
    assert cube.type == Marker.CUBE and cube.color.r > 0.8
    assert (cube.pose.position.x, cube.pose.position.y) == (12.5, 3.2)
    assert text.type == Marker.TEXT_VIEW_FACING
    assert text.text == 'Class: Box, Conf: 0.92, Dist: 1.5m'
    assert text.pose.position.z > cube.pose.position.z
    node = DetectionMarkerNode()
    node.on_objects(arr)
    node.destroy_node()


class _FakeDetector:
    device = 'cpu'
    on_gpu = False

    def infer(self, bgr):
        return [Detection(0, 'box', 0.9, 100.0, 120.0, 40.0, 30.0, 28),
                Detection(1, 'person', 0.7, 300.0, 200.0, 60.0, 160.0, 0)]


def test_yolo_node_publishes_detection_array():
    node = YoloNode(detector=_FakeDetector(), parameter_overrides=[
        Parameter('publish_annotated', value=True), Parameter('stats_period', value=0.0),
        Parameter('async_inference', value=False)])
    img = CvBridge().cv2_to_imgmsg(np.zeros((480, 640, 3), np.uint8), encoding='bgr8')
    img.header.stamp = STAMP
    img.header.frame_id = 'camera_optical_frame'
    out = node.process(img)
    assert out.header.stamp == STAMP and len(out.detections) == 2
    d = out.detections[0]
    assert d.id == '0' and d.results[0].hypothesis.class_id == 'box'
    assert d.results[0].hypothesis.score == pytest.approx(0.9)
    assert (d.bbox.center.position.x, d.bbox.size_y) == (100.0, 30.0)
    node.on_image(img)          # 발행 + 통계 로그 경로
    arr = to_detection_array([], img.header)
    assert arr.detections == []
    vis = draw_detections(np.zeros((480, 640, 3), np.uint8), _FakeDetector().infer(None))
    assert vis.sum() > 0
    node.stop()
    node.destroy_node()


class _FailingDetector(_FakeDetector):
    def infer(self, bgr):
        raise RuntimeError('boom')


def test_yolo_node_async_worker():
    node = YoloNode(detector=_FakeDetector(), parameter_overrides=[
        Parameter('stats_period', value=1000.0), Parameter('async_inference', value=True)])
    img = CvBridge().cv2_to_imgmsg(np.zeros((480, 640, 3), np.uint8), encoding='bgr8')
    img.header.stamp = STAMP
    node.on_image(img)                      # 작업 스레드가 최신 1 장을 처리
    deadline = time.monotonic() + 5.0
    while node._count == 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert node._count == 1
    node.detector = _FailingDetector()      # 예외가 나도 작업 스레드는 살아 있다
    node.on_image(img)
    time.sleep(0.2)
    assert node._worker.is_alive()
    node.stop()
    assert not node._worker.is_alive()
    node.destroy_node()


def test_aruco_node_pose_in_base_link():
    node = ArucoDetectorNode()
    node.on_info(_info())
    node.tf_buffer.set_transform_static(
        _tf('base_link', 'camera_optical_frame', T_BASE_OPT), 'test')
    r, t = aruco.look_at_marker_pose(0.5, math.radians(10.0), 0.0, 0.05, 0.0)
    img = aruco.render_marker_image(2, 0.18, K.matrix(), r, t)
    msg = CvBridge().cv2_to_imgmsg(img, encoding='mono8')
    msg.header.frame_id = 'camera_optical_frame'
    msg.header.stamp = STAMP
    det = node.process(msg)
    assert det is not None and det.marker_id == 2
    p, rot, cov = to_base(det, T_BASE_OPT)
    assert np.allclose(p, T_BASE_OPT.apply(t), atol=0.005)
    # 마커 모델 x(면 바깥 법선) 는 로봇 쪽 → base 기준 yaw ≈ π - 10° (카메라 수직축 회전 = base z)
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    assert abs(math.remainder(yaw - (math.pi - math.radians(10.0)), 2 * math.pi)) < 0.02
    assert cov is not None and cov.shape == (6, 6)
    res = node.on_enable(SetBool.Request(data=False), SetBool.Response())
    assert res.success and node.process(msg) is None
    node.on_enable(SetBool.Request(data=True), SetBool.Response())
    blank = CvBridge().cv2_to_imgmsg(np.full((480, 640), 120, np.uint8), encoding='mono8')
    blank.header = msg.header
    assert node.process(blank) is None
    node.on_image(msg)
    node.destroy_node()


def test_aruco_node_debug_image_and_selection():
    node = ArucoDetectorNode(parameter_overrides=[Parameter('publish_debug_image', value=True)])
    node.on_info(_info())
    msg = CvBridge().cv2_to_imgmsg(np.full((480, 640), 120, np.uint8), encoding='mono8')
    msg.header.frame_id = 'camera_optical_frame'
    assert node.process(msg) is None                      # TF 없음
    node.tf_buffer.set_transform_static(
        _tf('base_link', 'camera_optical_frame', T_BASE_OPT), 'test')
    r, t = aruco.look_at_marker_pose(0.6)
    msg2 = CvBridge().cv2_to_imgmsg(aruco.render_marker_image(1, 0.18, K.matrix(), r, t),
                                    encoding='mono8')
    msg2.header.frame_id = 'camera_optical_frame'
    assert node.process(msg2) is not None
    a = aruco.MarkerDetection(1, np.zeros((4, 2)), np.eye(3), np.array([0, 0, 2.0]), 0.1, None, 20)
    b = aruco.MarkerDetection(4, np.zeros((4, 2)), np.eye(3), np.array([0, 0, 1.0]), 0.1, None, 40)
    assert select_marker([a, b], -1) is b                   # 가장 가까운 것
    assert select_marker([a, b], 1) is a                    # 선호 id
    assert select_marker([a, b], 7) is b
    assert select_marker([], 1) is None
    assert to_base(a, T_BASE_OPT)[2] is None
    node.destroy_node()
