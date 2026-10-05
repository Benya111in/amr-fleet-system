"""rclpy 노드 래퍼 테스트: 콜백을 직접 불러 감지·복구 흐름과 드리프트 실험 기록을 확인한다."""

import csv
import math

from amr_localization.amcl_map_adapter import adapt, AmclMapAdapter, half_cell_offset
from amr_localization.kidnap_detector import State
from amr_localization.kidnap_monitor_node import KidnapMonitorNode, stamp_sec, yaw_of
from amr_localization.odom_drift_experiment import OdomDriftExperiment
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from grid_sim import make_room, raycast
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

OFFSET = (0.15, 0.0, 0.0)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def to_stamp(t):
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def odom_msg(t, pose, cov=None):
    m = Odometry()
    m.header.stamp = to_stamp(t)
    m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
    m.pose.pose.orientation.z = math.sin(pose[2] / 2)
    m.pose.pose.orientation.w = math.cos(pose[2] / 2)
    if cov is not None:
        m.pose.covariance = cov
    return m


def amcl_msg(t, pose, var_xy=0.005, var_yaw=0.002):
    m = PoseWithCovarianceStamped()
    m.header.stamp = to_stamp(t)
    m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
    m.pose.pose.orientation.z = math.sin(pose[2] / 2)
    m.pose.pose.orientation.w = math.cos(pose[2] / 2)
    cov = [0.0] * 36
    cov[0] = cov[7] = var_xy
    cov[35] = var_yaw
    m.pose.covariance = cov
    return m


def scan_msg(t, grid, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    sensor = (pose[0] + c * OFFSET[0], pose[1] + s * OFFSET[0], pose[2])
    ranges, amin, inc = raycast(grid, 0.05, sensor, n_beams=360, noise=0.01, seed=4)
    m = LaserScan()
    m.header.stamp = to_stamp(t)
    m.angle_min, m.angle_increment = amin, inc
    m.range_min, m.range_max = 0.1, 25.0
    m.ranges = [float(r) for r in ranges]
    return m


def map_msg(grid, res):
    m = OccupancyGrid()
    m.info.width, m.info.height, m.info.resolution = grid.shape[1], grid.shape[0], res
    m.info.origin.orientation.w = 1.0
    m.data = grid.ravel().tolist()
    return m


def test_amcl_map_adapter_shifts_origin_half_cell():
    grid, res = make_room()
    src = map_msg(grid, res)
    src.info.origin.position.x, src.info.origin.position.y = -14.1, -18.1
    assert half_cell_offset(0.05) == pytest.approx((0.025, 0.025))
    out = adapt(src)
    assert (out.info.origin.position.x, out.info.origin.position.y) == pytest.approx(
        (-14.075, -18.075))
    assert src.info.origin.position.x == pytest.approx(-14.1)      # 입력은 그대로
    assert out.info.width == src.info.width and len(out.data) == len(src.data)
    # AMCL 규약 셀 = floor((x − o')/s + ½) 가 OccupancyGrid 규약 셀 floor((x − o)/s) 와 일치
    for x in (-14.09, -14.0751, -14.051, -14.049, -12.99):   # 셀 경계는 부동소수 반올림 모호
        amcl_cell = math.floor((x - out.info.origin.position.x) / res + 0.5)
        assert amcl_cell == math.floor((x - src.info.origin.position.x) / res)
    node = AmclMapAdapter()
    try:
        sent = []
        node.pub.publish = sent.append
        rotated = map_msg(grid, res)
        rotated.info.origin.orientation.z = math.sin(0.25)
        rotated.info.origin.orientation.w = math.cos(0.25)
        node.on_map(src)
        node.on_map(rotated)                 # 원점 yaw ≠ 0: 경고 후에도 재발행
        assert node.maps == 2 and len(sent) == 2
        assert sent[0].info.origin.position.x == pytest.approx(-14.075)
    finally:
        node.destroy_node()


def test_helpers():
    assert stamp_sec(to_stamp(12.5)) == pytest.approx(12.5)
    m = odom_msg(0.0, (0.0, 0.0, 1.0))
    assert yaw_of(m.pose.pose.orientation) == pytest.approx(1.0)


def test_kidnap_monitor_detects_seeds_and_recovers(tmp_path):
    grid, res = make_room()
    log = tmp_path / 'events.csv'
    node = KidnapMonitorNode(parameter_overrides=[
        Parameter('match_window', value=1), Parameter('suspect_time', value=0.0),
        Parameter('converge_count', value=1), Parameter('seed_attempts', value=1),
        Parameter('fallback_cmd_vel_topic', value='test_cmd_vel'),
        Parameter('fallback_angular_speed', value=100.0),
        Parameter('lidar_offset', value=list(OFFSET)), Parameter('event_log', value=str(log))])
    try:
        cmds = []
        node.cmd_pub.publish = cmds.append
        now = stamp_sec(node.get_clock().now().to_msg()) - 5.0   # 스탬프는 과거 (tick 은 현재 시각)
        node.on_match_timer()                       # 맵 없음: 아무 일 없음
        node.on_map(map_msg(grid, res))
        home = (6.0, 6.0, 0.3)
        for k in range(20):
            node.on_odom(odom_msg(now + 0.05 * k, (0.0, 0.0, 0.0)))
        node.on_amcl(amcl_msg(now + 0.1, home))
        node.on_scan(scan_msg(now + 0.5, grid, home))
        node.on_match_timer()
        assert node.last_ratio > 0.9
        assert node.detector.state == State.TRACKING

        # 정지 중 납치: odom·AMCL 은 그대로, 스캔만 다른 곳
        kidnapped = (14.5, 5.5, 2.0)
        node.on_scan(scan_msg(now + 0.9, grid, kidnapped))
        node.on_match_timer()
        assert node.last_ratio < 0.5
        node.on_tick()
        assert node.detector.state == State.RECOVERING and node.detector.lost
        assert node.seeds, 'global seed search produced no hypotheses'
        best = node.seeds[0]
        assert math.hypot(best.x - kidnapped[0], best.y - kidnapped[1]) < 0.3
        assert node.fallback_until is not None     # spin 서버 없음 → 직접 회전
        node.fallback_until = 0.0
        node.on_tick()                              # 회전 끝 → 재시도(시드 소진 → AMCL 전역, 서버 없음)
        assert cmds and cmds[-1].angular.z == 0.0

        # AMCL 이 시드로 수렴 → 일치도 회복 → TRACKING
        t = stamp_sec(node.get_clock().now().to_msg())
        node.on_odom(odom_msg(t, (0.0, 0.0, 0.0)))
        node.on_amcl(amcl_msg(t, kidnapped))
        node.on_scan(scan_msg(t, grid, kidnapped))
        node.on_match_timer()
        assert node.detector.state == State.TRACKING and not node.detector.lost
        status = node.status_dict()
        assert status['recoveries'] == 1
        rows = list(csv.reader(log.open()))
        assert rows[0] == ['t', 'state', 'reason']
        assert any('recovered' in r[2] for r in rows[1:])
    finally:
        node.destroy_node()


def test_kidnap_monitor_without_spin_or_fallback():
    node = KidnapMonitorNode(parameter_overrides=[Parameter('seed_attempts', value=0)])
    try:
        node.reinitialize(1)          # AMCL 서비스 없음: 로그만
        node.start_spin(math.pi)      # spin 서버·fallback 없음 → 즉시 실패 처리
        node.cancel_spin()
        node.set_ekf_pose((0.0, 0.0, 0.0), 0.01, 0.01)   # 클라이언트 없음
        assert node.detector.state == State.TRACKING
    finally:
        node.destroy_node()


@pytest.mark.parametrize('odom_first', [False, True])
def test_odom_drift_experiment_records_runs(tmp_path, odom_first):
    # odom_first: wheel_odom 이 같은 시각 GT 보다 먼저 도착 (Gazebo 브리지 지연) → 대기열에서 기록
    out = tmp_path / 'drift'
    node = OdomDriftExperiment(parameter_overrides=[
        Parameter('scenarios', value=['straight', 'rotate']),
        Parameter('repetitions', value=1), Parameter('straight_length', value=0.5),
        Parameter('rotation_angle', value=1.0), Parameter('settle_time', value=0.1),
        Parameter('output_dir', value=str(out)), Parameter('service_wait', value=0.0)])
    try:
        cmds = []
        node.cmd_pub.publish = cmds.append
        x = y = th = 0.0
        dt = 0.02
        t = 100.0
        cov = [0.0] * 36
        finished = False
        for _ in range(3000):
            # odom: 1 % 거리 과대 + 작은 헤딩 드리프트
            cov[0] += 1e-7
            cov[7] += 1e-7
            cov[35] += 1e-8
            odom = odom_msg(t, (1.01 * x, 1.01 * y, th + 0.001 * t), list(cov))
            if odom_first:
                node.on_odom(odom)
            node.on_gt(odom_msg(t, (x, y, th)))
            try:
                node.on_control()
            except SystemExit:
                finished = True
                break
            v = cmds[-1].linear.x if cmds else 0.0
            w = cmds[-1].angular.z if cmds else 0.0
            node.on_ekf(odom_msg(t, (x, y, th)))
            if not odom_first:
                node.on_odom(odom)
            x += v * dt * math.cos(th + 0.5 * w * dt)
            y += v * dt * math.sin(th + 0.5 * w * dt)
            th += w * dt
            t += dt
        assert finished
        assert (out / 'straight_0.csv').exists() and (out / 'rotate_0.csv').exists()
        summary = (out / 'summary.md').read_text()
        assert '| straight | 1 |' in summary and '| rotate | 1 |' in summary
        rows = list(csv.DictReader((out / 'straight_0.csv').open()))
        assert len(rows) > 20
        # 기록 구간의 거의 모든 odom 샘플이 GT 와 짝지어져야 한다 (지연 시 버려지지 않음)
        span = float(rows[-1]['timestamp']) - float(rows[0]['timestamp'])
        assert len(rows) >= 0.9 * span / dt
        assert node.gt_at(-1.0) is None
    finally:
        node.destroy_node()
