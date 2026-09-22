"""
C++ 노드 래퍼 통합 테스트: 설치된 실행 파일을 띄워 토픽·서비스 계약을 확인한다.

알고리즘은 gtest 가 라이브러리 단위로 검증하므로 여기서는 래퍼가 파라미터·프레임 이름·QoS·메시지 필드를
계약대로 채우는지만 본다 (커버리지 빌드에서는 래퍼 코드도 이 테스트로 계측된다).
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_srvs.srv import Trigger

NS = f'/wrap_test_{os.getpid()}'


def executable(name):
    return os.path.join(get_package_prefix('amr_localization'), 'lib', 'amr_localization', name)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    node = Node('node_wrapper_test', namespace=NS)
    yield node
    node.destroy_node()
    rclpy.shutdown()


class Proc:
    """노드 실행 파일 (SIGINT 로 정상 종료 → gcov 기록)."""

    def __init__(self, name, params, log_dir: Path):
        args = [executable(name), '--ros-args', '-r', f'__ns:={NS}']
        for key, value in params.items():
            args += ['-p', f'{key}:={value}']
        self.log = (log_dir / f'{name}.log').open('w')
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:  # pragma: no cover - 비정상 종료 대비
                self.proc.kill()
        self.log.close()
        return self.proc.returncode


def spin_until(node, predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


def wait_for_subscriber(node, publisher, timeout=10.0):
    return spin_until(node, lambda: publisher.get_subscription_count() > 0, timeout)


def stamp_now(node):
    return node.get_clock().now().to_msg()


def test_wheel_odometry_node_contract(ros, tmp_path):
    proc = Proc('wheel_odometry_node', {'enable_slip_noise': 'false', 'frame_prefix': 'amr_09/',
                                        'publish_rate': 50.0, 'noise_seed': 7}, tmp_path)
    try:
        odoms, ticks = [], []
        ros.create_subscription(Odometry, 'wheel_odom', odoms.append, 50)
        ros.create_subscription(JointState, 'wheel_odom/ticks', ticks.append, 50)
        pub = ros.create_publisher(JointState, 'joint_states', 10)
        reset = ros.create_client(Trigger, 'wheel_odom/reset')
        assert wait_for_subscriber(ros, pub)
        # 이름이 없는 메시지·위치 누락 메시지는 무시 (경고만)
        pub.publish(JointState(name=['caster'], position=[0.0]))
        pub.publish(JointState(name=['left_wheel_joint', 'right_wheel_joint'], position=[0.0]))
        # 직진: 두 바퀴 5 rad/s (r = 0.0825 → 0.4125 m/s), ≈ 200 Hz 로 1.5 s. 각은 스탬프와 같은
        # 벽시계 경과 시간으로 계산한다 (루프 주기가 흔들려도 속도가 일정)
        omega = 5.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:
            msg = JointState(name=['amr_09/left_wheel_joint', 'amr_09/right_wheel_joint'])
            msg.header.stamp = stamp_now(ros)
            phi = omega * (time.monotonic() - t0)
            msg.position = [phi, phi]
            pub.publish(msg)
            rclpy.spin_once(ros, timeout_sec=0.005)
        assert spin_until(ros, lambda: len(odoms) > 20, 5.0)
        last = odoms[-1]
        assert last.header.frame_id == 'amr_09/odom'
        assert last.child_frame_id == 'amr_09/base_footprint'
        assert last.twist.twist.linear.x == pytest.approx(0.4125, rel=0.1)
        assert abs(last.pose.pose.position.y) < 1e-3
        assert last.pose.pose.position.x > 0.2
        assert last.pose.covariance[0] > 0.0 and last.twist.covariance[0] > 0.0
        assert last.twist.covariance[7] > 0.0 and last.twist.covariance[35] > 0.0
        assert ticks and ticks[-1].position[0] > 0.0
        # 자세 리셋 서비스
        assert reset.wait_for_service(timeout_sec=5.0)
        future = reset.call_async(Trigger.Request())
        assert spin_until(ros, future.done, 5.0) and future.result().success
    finally:
        assert proc.stop() == 0


def test_imu_filter_node_contract(ros, tmp_path):
    calib = tmp_path / 'calibration'
    proc = Proc('imu_filter_node', {'bias_estimation_time': 0.5, 'calibration_samples': 30,
                                    'calibration_dir': str(calib), 'frame_id': 'amr_09/imu_link'},
                tmp_path)
    try:
        outs = []
        ros.create_subscription(Imu, 'imu/data', outs.append, 50)
        pub = ros.create_publisher(Imu, 'imu/data_raw', 10)
        calibrate = ros.create_client(Trigger, 'imu/calibrate')
        assert wait_for_subscriber(ros, pub)
        future = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            msg = Imu()
            msg.header.stamp = stamp_now(ros)
            msg.header.frame_id = 'imu_link'
            msg.angular_velocity.z = 0.01               # 자이로 바이어스
            msg.linear_acceleration.z = 9.8             # 정지: +g
            msg.linear_acceleration.x = 0.05            # 가속도 바이어스
            pub.publish(msg)
            rclpy.spin_once(ros, timeout_sec=0.01)
            if future is None and time.monotonic() - t0 > 1.5:
                assert calibrate.wait_for_service(timeout_sec=5.0)
                future = calibrate.call_async(Trigger.Request())
        assert outs, 'imu/data 없음'
        last = outs[-1]
        assert last.header.frame_id == 'amr_09/imu_link'
        assert last.orientation_covariance[0] == -1.0
        assert abs(last.angular_velocity.z) < 1e-3     # 기동 바이어스 추정으로 제거
        assert abs(last.linear_acceleration.x) < 1e-2
        assert last.angular_velocity_covariance[0] > 0.0
        assert spin_until(ros, future.done, 10.0)
        result = future.result()
        assert result.success, result.message
        assert list(calib.glob('imu_bias_*.yaml')) and (calib / 'imu_bias.csv').exists()
    finally:
        assert proc.stop() == 0


def test_scan_filter_node_contract(ros, tmp_path):
    proc = Proc('scan_filter_node', {'stats_log_period': 0.5}, tmp_path)
    try:
        outs = []
        ros.create_subscription(LaserScan, 'scan_filtered', outs.append, 10)
        pub = ros.create_publisher(LaserScan, 'scan', 10)
        assert wait_for_subscriber(ros, pub)
        n = 720
        msg = LaserScan()
        msg.header.frame_id = 'amr_09/lidar_link'
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / n
        msg.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
        msg.range_min, msg.range_max = 0.1, 25.0
        ranges = [5.0] * n
        ranges[100] = 1.0                 # 고립 아웃라이어
        ranges[200] = 0.05                # range_min 미만
        ranges[300] = float('inf')        # 미검출
        msg.ranges = ranges
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:
            msg.header.stamp = stamp_now(ros)
            pub.publish(msg)
            rclpy.spin_once(ros, timeout_sec=0.1)
        assert outs, 'scan_filtered 없음'
        out = outs[-1]
        assert len(out.ranges) == n
        assert out.angle_min == pytest.approx(msg.angle_min)
        assert out.angle_increment == pytest.approx(msg.angle_increment)
        assert out.header.frame_id == msg.header.frame_id
        assert math.isnan(out.ranges[100])
        assert out.ranges[200] == -math.inf and out.ranges[300] == math.inf
        assert out.ranges[0] == pytest.approx(5.0)
    finally:
        assert proc.stop() == 0
