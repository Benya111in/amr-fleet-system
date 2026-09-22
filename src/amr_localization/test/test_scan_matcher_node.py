"""scan_matcher_node 래퍼 테스트: 지도·TF·스캔 콜백 → scan_match_pose (공분산 배치, 위치 상실 중 정지)."""

import math

from amr_localization.scan_matcher_node import ScanMatcherNode
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from grid_sim import make_room, raycast
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rclpy
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

OFFSET = (0.15, 0.0, 0.0)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def stamp(t):
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def band_map(grid, res):
    """채운 격자 → SLAM 같은 띠 지도 (관측면 ±1 셀 점유, 물체 속 미지)."""
    occ = grid >= 65
    surface = occ & ndimage.binary_dilation(~occ, structure=np.ones((3, 3), bool))
    band = ndimage.binary_dilation(surface, structure=np.ones((3, 3), bool))
    data = np.where(band, 100, np.where(occ, -1, 0)).astype(np.int8)
    m = OccupancyGrid()
    m.info.width, m.info.height, m.info.resolution = grid.shape[1], grid.shape[0], res
    m.info.origin.orientation.w = 1.0
    m.data = data.ravel().tolist()
    return m


def scan_msg(t, grid, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    sensor = (pose[0] + c * OFFSET[0], pose[1] + s * OFFSET[0], pose[2])
    ranges, amin, inc = raycast(grid, 0.05, sensor, n_beams=720, noise=0.01, seed=2)
    m = LaserScan()
    m.header.stamp = stamp(t)
    m.angle_min, m.angle_increment = amin, inc
    m.range_min, m.range_max = 0.1, 25.0
    m.ranges = [float(r) for r in ranges]
    return m


def tf_msg(t, pose):
    m = TransformStamped()
    m.header.stamp = stamp(t)
    m.header.frame_id = 'map'
    m.child_frame_id = 'base_footprint'
    m.transform.translation.x, m.transform.translation.y = pose[0], pose[1]
    m.transform.rotation.z = math.sin(pose[2] / 2)
    m.transform.rotation.w = math.cos(pose[2] / 2)
    return m


def test_scan_matcher_publishes_registered_pose():
    grid, res = make_room()
    node = ScanMatcherNode()
    out = []
    node.pub.publish = out.append
    truth = (6.0, 6.0, 0.3)
    guess = (6.08, 5.95, 0.33)                             # map EKF 가 낸 초기 추정 (8 cm, 1.7° 오차)
    node.on_scan(scan_msg(10.0, grid, truth))              # 지도 전: 무시
    assert out == []
    node.on_map(band_map(grid, res))
    assert node.surface is not None and len(node.surface) > 500
    node.tf_buffer.set_transform(tf_msg(9.9, guess), 'test')
    node.tf_buffer.set_transform(tf_msg(10.1, guess), 'test')
    node.on_scan(scan_msg(10.0, grid, truth))
    assert len(out) == 1
    msg = out[0]
    assert msg.header.frame_id == 'map'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (10, 0)
    p = msg.pose.pose
    assert math.hypot(p.position.x - truth[0], p.position.y - truth[1]) < 0.02
    yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
    assert abs(yaw - truth[2]) < math.radians(0.5)
    cov = msg.pose.covariance
    assert cov[0] >= 0.005 ** 2 and cov[7] >= 0.005 ** 2 and cov[35] >= 0.002 ** 2
    assert cov[1] == cov[6] and cov[5] == cov[30]           # (x, y, yaw) 부분 대칭 배치
    # 최대 처리율: 같은 주기 안의 다음 스캔은 건너뛴다
    node.on_scan(scan_msg(10.05, grid, truth))
    assert len(out) == 1
    # 스캔 시각 TF 가 아직 없으면(EKF TF 지연) 가장 최근 TF 로 초기화
    node.on_scan(scan_msg(10.3, grid, truth))
    assert len(out) == 2 and node.stats['latest_tf'] >= 1
    # 최근 TF 가 너무 오래되면 건너뛴다
    node.on_scan(scan_msg(12.0, grid, truth))
    assert len(out) == 2 and node.stats['no_tf'] >= 1
    # 위치 상실 중에는 측정을 내지 않는다
    node.on_lost(Bool(data=True))
    node.tf_buffer.set_transform(tf_msg(12.5, guess), 'test')
    node.on_scan(scan_msg(12.5, grid, truth))
    assert len(out) == 2
    node.on_lost(Bool(data=False))
    # 다른 분지(납치 규모 초기 오차)는 거부
    node.tf_buffer.set_transform(tf_msg(13.0, (9.0, 3.0, 1.5)), 'test')
    node.on_scan(scan_msg(13.0, grid, truth))
    assert len(out) == 2 and node.stats['rejected'] >= 1
    node.log_stats()
    assert node.stats['ok'] == 0
    node.destroy_node()
