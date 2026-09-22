"""
교차 패키지 계약 C3·C2 시험: 검출기 마커 자세를 도킹 서버가 받아들이고 예외 사각형을 보내는지.

amr_perception 의 ArUco 검출기가 만든 perception/dock_marker_pose 를 실제 docking_server_node 실행 파일이
관측으로 받아들이고(마커 모델 프레임 +x 법선), 도킹 중 safety/dock_exclusion 을 보내는지 본다.

마커 자세는 검출기와 같은 경로로 만든다: amr_perception.aruco.render_marker_image 로 알려진 자세의 마커를
렌더링 → ArucoPoseEstimator.detect (IPPE + 연직 사전정보) → aruco_detector_node.to_base (카메라 광학 →
base_link, config/sensors.yaml camera_link 장착) → matrix_to_quat. 리뷰 회귀: 서버 기본값이 OpenCV z 법선이라
이 자세의 z 축(연직)을 법선으로 읽어 모든 관측을 버렸다 (search 에 머묾) — 음성 대조로 z 를 함께 돌린다.
"""

import math
import os
import subprocess
import sys
import threading
import time
import uuid

import numpy as np
import pytest

# amr_perception 이 없는 단독 체크아웃에서는 이 모듈만 건너뛴다. pytest 8.3 은 모듈 수준
# importorskip 이 실제로 skip 을 내면 그 디렉터리의 나머지 시험까지 수집하지 않는다 (실측) —
# 그래서 import 를 직접 감싸고 pytestmark 로 건너뛴다.
try:
    from amr_perception import aruco
    from amr_perception import aruco_detector_node as detector
    from amr_perception import pinhole
    from amr_perception import transforms
except ImportError as exc:                       # pragma: no cover - 병합 트리에서는 항상 있다
    aruco = detector = transforms = pinhole = None
    _IMPORT_ERROR = str(exc)
else:
    _IMPORT_ERROR = ''

pytestmark = pytest.mark.skipif(aruco is None, reason=f'amr_perception 없음: {_IMPORT_ERROR}')

from amr_msgs.action import Dock  # noqa: E402
from geometry_msgs.msg import PolygonStamped, PoseStamped  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

EXE = os.environ.get('DOCKING_SERVER_EXE', '')
STANDOFF = 0.65
# config/sensors.yaml: camera_link (0.29, 0, 0.07) rpy 0 → optical: rpy (−π/2, 0, −π/2)
CAM_POS = np.array([0.29, 0.0, 0.07])
R_BASE_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# hfov 87° (amr_perception 이 없으면 이 모듈 전체가 skip 이므로 None)
K = pinhole.CameraIntrinsics.from_fov(640, 480, 1.518436).matrix() if pinhole else None


def marker_pose_from_detector(dist, lateral, yaw_offset_deg):
    """
    검출기 경로로 base_link 마커 자세를 만든다.

    로봇(base_link) 앞 dist [m] 판 면, 횡 lateral, 로봇 방위가 판 법선에서 yaw_offset 만큼 돈 자세의
    마커를 렌더링·검출해 aruco_detector_node 가 발행할 base_link 자세 (위치, 쿼터니언) 와 진값 판 중심·
    법선 방위를 돌려준다.
    """
    psi = math.radians(yaw_offset_deg)
    # 로봇 프레임: 판 중심 = R(−ψ)·(dist, lateral), 법선 방위 = π − ψ (로봇 쪽), 판 중심 높이 = 카메라 높이
    c, s = math.cos(-psi), math.sin(-psi)
    p_m = np.array([c * dist - s * lateral, s * dist + c * lateral, CAM_POS[2]])
    theta_n = math.pi - psi
    n = np.array([math.cos(theta_n), math.sin(theta_n), 0.0])
    z_up = np.array([0.0, 0.0, 1.0])
    r_base_model = np.column_stack([n, np.cross(z_up, n), z_up])
    r_opt_model = R_BASE_OPT.T @ r_base_model
    r_opt_cv = r_opt_model @ aruco.CV_FROM_MODEL.T
    t_opt = R_BASE_OPT.T @ (p_m - CAM_POS)
    img = aruco.render_marker_image(0, 0.18, K, r_opt_cv, t_opt, noise_std=2.0,
                                    rng=np.random.default_rng(3), supersample=3)
    est = aruco.ArucoPoseEstimator('DICT_4X4_50', 0.18)
    dets = est.detect(img, K, ids=[0], up_hint=R_BASE_OPT.T @ z_up)
    assert len(dets) == 1, '렌더링한 마커를 검출하지 못했다'
    p, r, _ = detector.to_base(dets[0], transforms.Transform(R_BASE_OPT, CAM_POS))
    q = transforms.matrix_to_quat(r)
    return p, q, p_m, theta_n


def test_detector_pose_matches_truth_and_uses_model_frame():
    """시험 입력 자체의 검증: 검출기 경로로 만든 자세가 진값과 1 cm / 1° 안이고 +x 가 법선이다."""
    p, q, p_m, theta_n = marker_pose_from_detector(1.40, 0.10, 8.0)
    assert np.linalg.norm(p[:2] - p_m[:2]) < 0.02
    r = transforms.quat_to_matrix(q)
    normal_yaw = math.atan2(r[1, 0], r[0, 0])
    assert abs(math.remainder(normal_yaw - theta_n, 2 * math.pi)) < math.radians(1.5)
    assert abs(r[2, 2]) > 0.99          # 모델 z = 위 → OpenCV 법선(z)으로 읽으면 수직이라 버려진다


class _Harness:
    """docking_server_node 프로세스 + 마커 발행 + dock 액션 클라이언트."""

    def __init__(self, axis):
        self.ns = f'/contract_{uuid.uuid4().hex[:8]}'
        args = [EXE, '--ros-args', '-r', f'__ns:={self.ns}',
                '-p', f'marker_normal_axis:={axis}', '-p', 'docks.ids:=[dock_1]',
                '-p', f'docks.dock_1.standoff:={STANDOFF}', '-p', 'attempt_timeout:=60.0']
        self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.node = rclpy.create_node(f'contract_client_{uuid.uuid4().hex[:6]}', namespace=self.ns)
        self.pub = self.node.create_publisher(PoseStamped, 'perception/dock_marker_pose', 10)
        self.client = ActionClient(self.node, Dock, 'dock')
        self.feedback = []
        self.exclusions = []
        self.node.create_subscription(
            PolygonStamped, 'safety/dock_exclusion',
            lambda m: self.exclusions.append((time.monotonic(), m)), 10)
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.node)
        self.stop = False
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while not self.stop:
            self.exec.spin_once(timeout_sec=0.05)

    def run(self, position, quat, seconds=2.5):
        assert self.client.wait_for_server(timeout_sec=20.0), 'dock 서버가 뜨지 않았다'
        goal = Dock.Goal()
        goal.dock_id = 'dock_1'
        goal.max_retries = 1
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda fb: self.feedback.append(fb.feedback))
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        pos = msg.pose.position
        pos.x, pos.y, pos.z = (float(v) for v in position)
        (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
         msg.pose.orientation.w) = (float(v) for v in quat)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pub.publish(msg)            # 검출기처럼 30 Hz
            time.sleep(1.0 / 30.0)
        handle = future.result() if future.done() else None
        if handle is not None:
            handle.cancel_goal_async()
        time.sleep(0.3)

    def close(self):
        self.stop = True
        self.thread.join(timeout=2.0)
        self.proc.terminate()
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:  # pragma: no cover
            self.proc.kill()
        self.node.destroy_node()


@pytest.fixture(scope='module')
def ros():
    if not EXE or not os.path.exists(EXE):
        pytest.skip('DOCKING_SERVER_EXE 가 없다 (CMake 가 설정)')
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.parametrize('dist,lateral,yaw_deg', [(1.40, 0.10, 8.0), (0.80, -0.05, -5.0)])
def test_docking_server_accepts_detector_pose(ros, dist, lateral, yaw_deg):
    position, quat, p_m, theta_n = marker_pose_from_detector(dist, lateral, yaw_deg)
    h = _Harness('x')
    try:
        h.run(position, quat)
    finally:
        h.close()
    phases = {fb.current_phase for fb in h.feedback}
    assert phases & {'approach', 'align', 'final'}, f'관측을 받지 못했다: {phases}'
    # 남은 거리 = 로봇 원점 ~ docked 자세 (판 면 앞 standoff) — 진값 기하와 3 cm 안
    target = p_m[:2] + STANDOFF * np.array([math.cos(theta_n), math.sin(theta_n)])
    expected = float(np.linalg.norm(target))
    remaining = [fb.distance_remaining for fb in h.feedback if fb.distance_remaining >= 0.0]
    assert remaining, '추정 오차가 보고되지 않았다'
    assert abs(remaining[0] - expected) < 0.03, (remaining[0], expected)
    # 계약 C2: 도킹 goal 동안 safety/dock_exclusion ≥ 10 Hz, base_link 프레임, 판 중심을 덮는다
    assert len(h.exclusions) >= 10
    stamps = [t for t, _ in h.exclusions]
    rate = (len(stamps) - 1) / max(stamps[-1] - stamps[0], 1e-6)
    assert rate >= 10.0, rate
    last = h.exclusions[-1][1]
    assert last.header.frame_id == 'base_link'
    xs = [pt.x for pt in last.polygon.points]
    assert min(xs) <= p_m[0] <= max(xs)


def test_opencv_z_axis_reading_discards_detector_poses(ros):
    """음성 대조 (리뷰 지적 재현): 같은 자세를 marker_normal_axis=z 로 읽으면 search 에 머문다."""
    position, quat, _, _ = marker_pose_from_detector(1.40, 0.10, 8.0)
    h = _Harness('z')
    try:
        h.run(position, quat, seconds=2.0)
    finally:
        h.close()
    phases = {fb.current_phase for fb in h.feedback}
    assert phases == {'search'}, phases


if __name__ == '__main__':  # pragma: no cover
    sys.exit(pytest.main([__file__, '-v']))
