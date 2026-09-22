"""
kinematic_sim 의 LiDAR 모델 = 실제 센서 기하·잡음.

리뷰: base_footprint 원점, z = 0, 1° 빔, 잡음 없음이라 코스트맵 높이 창 결함과 σ 0.03 통로 문제가
운동학 시험에서 가려졌다.

  - 런치: base_footprint → lidar_link = (0.15, 0, 0.20) (robot_params + sensors.yaml), σ 0.03, 720 빔
  - cast_scan: 광선 원점이 lidar_x 앞, 0.60 m 통로 벽 끝점 σ ≈ 0.03, 원판 장애물·range_min
"""
import importlib.util
import math
from pathlib import Path

from amr_navigation import warehouse_map as wm
import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
REPO_CONFIG = PKG.parents[1] / 'config'


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def sim():
    pytest.importorskip('rclpy')
    return _load('kinematic_sim', PKG / 'scripts' / 'kinematic_sim.py')


@pytest.fixture(scope='module')
def launch():
    return _load('kinematic_sim_launch', PKG / 'launch' / 'kinematic_sim.launch.py')


def test_launch_mounts_lidar_from_config(launch, tmp_path):
    x, z, sigma, beams = launch.lidar_mount(str(REPO_CONFIG))
    assert (x, z, sigma, beams) == pytest.approx((0.15, 0.20, 0.03, 720))
    # 설정이 없으면 재배치 후 기본값
    assert launch.lidar_mount(str(tmp_path)) == pytest.approx((0.15, 0.20, 0.03, 720))


def test_scan_origin_is_lidar_link(sim):
    grid = wm.build_warehouse(0.05)
    rng = np.random.default_rng(3)
    # 좁은 통로 중앙 (0, −10) 에서 남쪽(−y)을 보고: 빔 ±90° 는 양쪽 벽 x = ±0.30
    ang, r = sim.cast_scan(grid, 0.0, -10.0, -math.pi / 2, 0.15, 720, 12.0, 0.1, 0.0, rng)
    assert len(ang) == 720
    assert ang[1] - ang[0] == pytest.approx(math.radians(0.5))
    side = np.argmin(np.abs(ang - math.pi / 2))
    assert r[side] == pytest.approx(0.30, abs=0.03)   # 반 셀 행진 양자화
    # 전방 빔(0°): 통로 남쪽 끝 너머 → 원점이 0.15 앞이므로 base_footprint 기준보다 0.15 짧다
    fwd = np.argmin(np.abs(ang))
    _, r0 = sim.cast_scan(grid, 0.0, -10.0, -math.pi / 2, 0.0, 720, 12.0, 0.1, 0.0, rng)
    if math.isfinite(r0[fwd]) and math.isfinite(r[fwd]):
        assert r0[fwd] - r[fwd] == pytest.approx(0.15, abs=0.03)


def test_scan_noise_matches_sensor_sigma(sim):
    grid = wm.build_warehouse(0.05)
    rng = np.random.default_rng(5)
    errs = []
    for _ in range(40):
        ang, r = sim.cast_scan(grid, 0.0, -10.0, -math.pi / 2, 0.15, 720, 12.0, 0.1, 0.03, rng)
        _, clean = sim.cast_scan(grid, 0.0, -10.0, -math.pi / 2, 0.15, 720, 12.0, 0.1, 0.0,
                                 np.random.default_rng(0))
        ok = np.isfinite(r) & np.isfinite(clean)
        errs.extend((r[ok] - clean[ok]).tolist())
    assert np.std(errs) == pytest.approx(0.03, rel=0.1)
    assert abs(np.mean(errs)) < 0.003


def test_disc_obstacle_and_range_min(sim):
    rng = np.random.default_rng(1)
    # 지도 없음, 로봇 원점·동쪽, 1.15 m 앞(lidar 기준 1.0) 반경 0.25 원판 → 전방 빔 0.75
    ang, r = sim.cast_scan(None, 0.0, 0.0, 0.0, 0.15, 720, 12.0, 0.1, 0.0, rng,
                           [(1.15, 0.0, 0.0, 0.0, 0.25)])
    fwd = np.argmin(np.abs(ang))
    assert r[fwd] == pytest.approx(0.75, abs=1e-9)
    assert np.isinf(r[np.argmin(np.abs(ang - math.pi))])   # 뒤쪽은 미검출
    # range_min 안쪽 물체는 미검출
    _, r2 = sim.cast_scan(None, 0.0, 0.0, 0.0, 0.15, 720, 12.0, 0.1, 0.0, rng,
                          [(0.25, 0.0, 0.0, 0.0, 0.05)])
    assert np.isinf(r2[fwd])


def test_node_integrates_publishes_and_teleports(sim, tmp_path):
    # 노드 단위: 명령 → 서보(지연·1차) → 원호 적분, 50 Hz 상태·10 Hz 스캔·장애물 트랙 발행, initialpose 순간 이동
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    map_yaml = PKG.parents[1] / 'maps' / 'warehouse.yaml'
    args = ['--ros-args', '-p', 'moving_obstacles:=[2.0, 0.0, 0.5, 0.0, 0.25]',
            '-p', 'obstacle_period:=4.0', '-p', 'rate:=100.0']
    if map_yaml.is_file():
        args += ['-p', f'map_yaml:={map_yaml}']
    rclpy.init(args=args)
    try:
        node = sim.KinematicSim()
        got = {'odom': [], 'scan': [], 'obs': []}
        io = rclpy.create_node('kin_sim_io')
        from amr_msgs.msg import TrackedObstacleArray
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        io.create_subscription(Odometry, 'odometry/filtered', got['odom'].append, 50)
        io.create_subscription(LaserScan, 'scan_filtered', got['scan'].append, 10)
        io.create_subscription(TrackedObstacleArray, 'perception/tracked_obstacles',
                               got['obs'].append, 10)
        node.on_cmd(Twist())
        cmd = Twist()
        cmd.linear.x = 0.5
        node.on_cmd(cmd)
        for _ in range(100):             # 100 스텝 = 1 s (타이머 대신 직접 호출)
            node.on_tick()
        # 서보 지연 0.04 + 1차 0.08 + 가속 제한 1.0 m/s² → 1 s 뒤 속도 ≈ 0.5,
        # 이동 ≈ 0.5·(1 − 0.04 − 0.25 − 0.08) ≈ 0.32 m (램프·지연 몫)
        assert node.v_axis.value == pytest.approx(0.5, abs=0.01)
        assert 0.30 < node.x < 0.40
        assert node.y == pytest.approx(0.0, abs=1e-9)
        for _ in range(30):
            rclpy.spin_once(io, timeout_sec=0.01)
        assert got['odom'] and got['scan'] and got['obs']
        scan = got['scan'][-1]
        assert scan.header.frame_id == 'lidar_link' and len(scan.ranges) == 720
        o = got['obs'][-1].obstacles[0]
        assert o.is_dynamic and o.velocity.x == pytest.approx(0.5)
        # 왕복 장애물: 반주기(2 s) 뒤 속도 반전
        assert node.obstacle_state((2.0, 0.0, 0.5, 0.0, 0.25), 3.0)[2] == pytest.approx(-0.5)
        if node.grid is not None:
            assert node.footprint_clearance() >= 0.0
        # 순간 이동 (좁은 통로 한가운데, 남쪽) → 서보·명령 초기화
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = -10.0
        msg.pose.pose.orientation.z = -math.sqrt(0.5)
        msg.pose.pose.orientation.w = math.sqrt(0.5)
        node.on_pose(msg)
        assert (node.x, node.y) == (0.0, -10.0) and node.th == pytest.approx(-math.pi / 2)
        assert node.cmd == (0.0, 0.0) and node.v_axis.value == 0.0
        if node.grid is not None:
            # 0.60 m 통로 중앙: 풋프린트 측면 여유 ≈ 0.10 m (지도 벽 흩어짐 몫만큼 작다)
            assert 0.0 <= node.footprint_clearance() <= 0.12
        # 명령 시간 초과 → 정지 명령으로 적분
        node.cmd = (0.5, 0.0)
        node.last_cmd = node.get_clock().now() - rclpy.duration.Duration(seconds=1.0)
        for _ in range(10):
            node.on_tick()
        assert node.v_axis.value == pytest.approx(0.0, abs=1e-9)
        node.destroy_node()
        io.destroy_node()
    finally:
        rclpy.shutdown()
