"""
kinematic_sim: Gazebo + ros_gz_bridge 대역 (시나리오 04/09/13 의 기본 백엔드, GPU 불필요).

로봇 1대의 브리지 토픽 계약(components.md §5.1)을 같은 이름·타입으로 흉내 낸다.
  Sub cmd_vel (Twist)            마지막 명령 유지 (Gazebo DiffDrive 와 같음), limits 가속 한계로 추종
  Pub ground_truth/odom  50 Hz   frame world, child <p>base_footprint, 노이즈 없음 (평가 전용)
      joint_states       50 Hz   <p>left_wheel_joint / <p>right_wheel_joint 위치·속도
      imu/data_raw      100 Hz   바이어스(시작 시 1회 추출) + 백색 노이즈 + 중력 (sensors.yaml imu)
      imu/data          100 Hz   publish_corrected_imu 일 때: 같은 표본에서 바이어스를 뺀 값
                                  (imu_filter_node 대역 — 실제 노드가 있으면 하네스가 끈다)
      scan (+scan_filtered) 10 Hz 720 빔 레이캐스트 + σ 노이즈 (sensors.yaml lidar)
      camera/camera_info 30 Hz, camera/depth/camera_info 15 Hz   (safety_node 생존 감시용)
  Sub itest/obstacles (std_msgs/String JSON, latched)
      {"world": {"circles": [[x, y, r]...], "boxes": [[xmin, ymin, xmax, ymax]...]},
       "robot": {"circles": [...], "boxes": [...]}}   robot = base_link 기준 (로봇과 같이 움직임)

모든 발행은 한 타이머(imu 주기)에서 실제 경과 시간 dt 로 적분한 상태를 쓰므로, 호스트 부하로
타이머가 늦어도 GT 스탬프와 자세가 서로 맞는다.
"""

import json
import math
from typing import List, Tuple

from amr_itest import kinematics as km
from amr_itest.standins.common import LATCHED, RELIABLE, run, StandinNode
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import CameraInfo, Imu, JointState, LaserScan
from std_msgs.msg import String

GRAVITY = 9.80665


class KinematicSim(StandinNode):
    """유니사이클 운동학 + 센서 합성."""

    def __init__(self):
        super().__init__('kinematic_sim')
        self.rng = np.random.default_rng(int(self.param('seed', 7)))
        self.limits = km.Limits(
            self.param('limits.max_linear_velocity', 2.0),
            self.param('limits.min_linear_velocity', -0.5),
            self.param('limits.max_linear_acceleration', 1.0),
            self.param('limits.max_angular_velocity', 1.5),
            self.param('limits.max_angular_acceleration', 2.0))
        self.wheel_r = self.param('robot.wheel_radius', 0.0825)
        self.wheel_sep = self.param('robot.wheel_separation', 0.36)
        self.state = km.State(self.param('initial_x', 0.0), self.param('initial_y', 0.0),
                              self.param('initial_yaw', 0.0))
        self.cmd = (0.0, 0.0)
        self.phi = [0.0, 0.0]
        self.noise = self.param('noise', True)

        # IMU 모델 (sensors.yaml imu)
        self.imu_rate = self.param('imu.update_rate', 100.0)
        self.acc_sigma = self.param('imu.accel_noise_stddev', 0.017) if self.noise else 0.0
        self.gyro_sigma = self.param('imu.gyro_noise_stddev', 0.0002) if self.noise else 0.0
        self.acc_bias = self._bias(self.param('imu.accel_bias_mean', 0.10),
                                   self.param('imu.accel_bias_stddev', 0.001))
        self.gyro_bias = self._bias(self.param('imu.gyro_bias_mean', 0.01),
                                    self.param('imu.gyro_bias_stddev', 0.0))
        self.imu_frame = self.frame(self.param('imu.frame_id', 'imu_link'))

        # LiDAR 모델 (sensors.yaml lidar)
        self.lidar_rate = self.param('lidar.update_rate', 10.0)
        self.angle_min = self.param('lidar.angle_min', -math.pi)
        self.angle_max = self.param('lidar.angle_max', math.pi - 2 * math.pi / 720)
        self.samples = self.param('lidar.samples', 720)
        self.range_min = self.param('lidar.range_min', 0.1)
        self.range_max = self.param('lidar.range_max', 25.0)
        self.range_sigma = self.param('lidar.noise_stddev', 0.03) if self.noise else 0.0
        self.lidar_ext = (self.param('lidar.extrinsic.x', 0.15),
                          self.param('lidar.extrinsic.y', 0.0),
                          self.param('lidar.extrinsic.yaw', 0.0))
        self.angles = km.beam_angles(self.angle_min, self.angle_max, self.samples)
        self.lidar_frame = self.frame(self.param('lidar.frame_id', 'lidar_link'))

        self.world_circles: List[Tuple[float, float, float]] = []
        self.world_boxes: List[Tuple[float, float, float, float]] = []
        self.robot_circles: List[Tuple[float, float, float]] = []
        self.robot_boxes: List[Tuple[float, float, float, float]] = []

        # 발행자 (브리지와 같은 reliable)
        self.pub_gt = self.create_publisher(Odometry, 'ground_truth/odom', RELIABLE)
        self.pub_js = self.create_publisher(JointState, 'joint_states', RELIABLE)
        # 원시 토픽 = sensors.yaml imu.topic (브리지와 같은 이름), 보정 토픽 = imu_filter_node 출력
        raw_topic = str(self.param('imu.topic', 'imu/data_raw'))
        filtered_topic = str(self.param('imu_filtered_topic', 'imu/data'))
        self.pub_imu_raw = None
        if raw_topic != filtered_topic:
            self.pub_imu_raw = self.create_publisher(Imu, raw_topic, RELIABLE)
        self.pub_imu = None
        if self.param('publish_corrected_imu', True) or self.pub_imu_raw is None:
            self.pub_imu = self.create_publisher(Imu, filtered_topic, RELIABLE)
        scan_topics = self.param('scan_topics', ['scan', 'scan_filtered'])
        self.pub_scans = [self.create_publisher(LaserScan, t, RELIABLE) for t in scan_topics]
        self.pub_rgb_info = self.create_publisher(
            CameraInfo, str(self.param('rgb_camera.info_topic', 'camera/camera_info')), RELIABLE)
        self.pub_depth_info = self.create_publisher(
            CameraInfo, str(self.param('depth_camera.info_topic', 'camera/depth/camera_info')),
            RELIABLE)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, RELIABLE)
        self.create_subscription(String, 'itest/obstacles', self.on_obstacles, LATCHED)

        self.odom_period = 1.0 / self.param('odom_rate', 50.0)
        self.scan_period = 1.0 / self.lidar_rate
        self.rgb_period = 1.0 / self.param('rgb_camera.update_rate', 30.0)
        self.depth_period = 1.0 / self.param('depth_camera.update_rate', 15.0)
        self.last = {'odom': -1e9, 'scan': -1e9, 'rgb': -1e9, 'depth': -1e9}
        self.t_prev = self._now()
        self.create_timer(1.0 / self.imu_rate, self.tick)
        self.get_logger().info(
            f'kinematic_sim: imu {self.imu_rate} Hz, scan {self.lidar_rate} Hz, '
            f'bias gyro {self.gyro_bias.round(4).tolist()} '
            f'accel {self.acc_bias.round(3).tolist()}')

    def _bias(self, mean: float, std: float) -> np.ndarray:
        """축별 상수 바이어스: |b| ~ N(mean, std), 부호 무작위 (sensors.yaml imu 주석의 모델)."""
        if not self.noise:
            return np.zeros(3)
        mag = np.abs(self.rng.normal(mean, std, 3)) if std > 0 else np.full(3, mean)
        return mag * self.rng.choice([-1.0, 1.0], 3)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp(self, t: float):
        from builtin_interfaces.msg import Time
        sec = int(math.floor(t))
        return Time(sec=sec, nanosec=int((t - sec) * 1e9))

    # --- 입력 ---
    def on_cmd(self, msg: Twist) -> None:
        self.cmd = (float(msg.linear.x), float(msg.angular.z))

    def on_obstacles(self, msg: String) -> None:
        try:
            spec = json.loads(msg.data or '{}')
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'itest/obstacles JSON 오류: {exc}')
            return
        world = spec.get('world', {})
        robot = spec.get('robot', {})
        self.world_circles = [tuple(map(float, c)) for c in world.get('circles', [])]
        self.world_boxes = [tuple(map(float, b)) for b in world.get('boxes', [])]
        self.robot_circles = [tuple(map(float, c)) for c in robot.get('circles', [])]
        self.robot_boxes = [tuple(map(float, b)) for b in robot.get('boxes', [])]
        self.get_logger().info(f'장애물 갱신: {spec}')

    # --- 주기 처리 ---
    def tick(self) -> None:
        t = self._now()
        dt = t - self.t_prev
        if dt <= 0.0:
            return
        dt = min(dt, 0.2)    # 부하로 타이머가 크게 밀려도 한 스텝을 과대 적분하지 않는다
        self.t_prev = t
        prev = self.state
        self.state, ax, ay = km.step(prev, self.cmd[0], self.cmd[1], dt, self.limits)
        wl, wr = km.wheel_rates(self.state.v, self.state.w, self.wheel_r, self.wheel_sep)
        wl0, wr0 = km.wheel_rates(prev.v, prev.w, self.wheel_r, self.wheel_sep)
        self.phi[0] += 0.5 * (wl + wl0) * dt
        self.phi[1] += 0.5 * (wr + wr0) * dt
        stamp = self._stamp(t)
        self.publish_imu(stamp, ax, ay)
        if t - self.last['odom'] >= self.odom_period * 0.95:
            self.last['odom'] = t
            self.publish_odom(stamp)
            self.publish_joints(stamp, wl, wr)
        if t - self.last['scan'] >= self.scan_period * 0.95:
            self.last['scan'] = t
            self.publish_scan(stamp)
        if t - self.last['rgb'] >= self.rgb_period * 0.95:
            self.last['rgb'] = t
            self.pub_rgb_info.publish(self._camera_info(stamp, 'rgb_camera'))
        if t - self.last['depth'] >= self.depth_period * 0.95:
            self.last['depth'] = t
            self.pub_depth_info.publish(self._camera_info(stamp, 'depth_camera'))

    def publish_odom(self, stamp) -> None:
        s = self.state
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self.param('gt_frame', 'world'))
        msg.child_frame_id = self.frame('base_footprint')
        msg.pose.pose.position.x = s.x
        msg.pose.pose.position.y = s.y
        qx, qy, qz, qw = km.yaw_quaternion(s.yaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = qx, qy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.twist.twist.linear.x = s.v
        msg.twist.twist.angular.z = s.w
        self.pub_gt.publish(msg)

    def publish_joints(self, stamp, wl: float, wr: float) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [self.frame('left_wheel_joint'), self.frame('right_wheel_joint')]
        msg.position = [self.phi[0], self.phi[1]]
        msg.velocity = [wl, wr]
        self.pub_js.publish(msg)

    def publish_imu(self, stamp, ax: float, ay: float) -> None:
        s = self.state
        acc_true = np.array([ax, ay, GRAVITY])
        gyro_true = np.array([0.0, 0.0, s.w])
        acc_noise = km.gaussian(self.rng, self.acc_sigma, 3)
        gyro_noise = km.gaussian(self.rng, self.gyro_sigma, 3)
        if self.pub_imu_raw is not None:
            self.pub_imu_raw.publish(self._imu_msg(stamp, acc_true + self.acc_bias + acc_noise,
                                                   gyro_true + self.gyro_bias + gyro_noise))
        if self.pub_imu is not None:
            self.pub_imu.publish(self._imu_msg(stamp, acc_true + acc_noise,
                                               gyro_true + gyro_noise))

    def _imu_msg(self, stamp, acc: np.ndarray, gyro: np.ndarray) -> Imu:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        qx, qy, qz, qw = km.yaw_quaternion(self.state.yaw)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = \
            qx, qy, qz, qw
        msg.orientation_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = \
            (float(v) for v in gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = \
            (float(v) for v in acc)
        g2 = max(self.gyro_sigma ** 2, 1e-8)
        a2 = max(self.acc_sigma ** 2, 1e-6)
        msg.angular_velocity_covariance = [g2, 0.0, 0.0, 0.0, g2, 0.0, 0.0, 0.0, g2]
        msg.linear_acceleration_covariance = [a2, 0.0, 0.0, 0.0, a2, 0.0, 0.0, 0.0, a2]
        return msg

    def publish_scan(self, stamp) -> None:
        s = self.state
        ex, ey, eyaw = self.lidar_ext
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        ox, oy = s.x + c * ex - sn * ey, s.y + sn * ex + c * ey
        circles = list(self.world_circles)
        boxes = list(self.world_boxes)
        for cx, cy, r in self.robot_circles:
            wx, wy = km.base_to_world(np.array([[cx, cy]]), s.x, s.y, s.yaw)[0]
            circles.append((wx, wy, r))
        for bx0, by0, bx1, by1 in self.robot_boxes:
            # 로봇 기준 박스는 로봇이 회전하지 않은 동안만 정확하다 (축 정렬 근사)
            corners = km.base_to_world(np.array([[bx0, by0], [bx1, by1]]), s.x, s.y, s.yaw)
            boxes.append((min(corners[:, 0]), min(corners[:, 1]),
                          max(corners[:, 0]), max(corners[:, 1])))
        ranges = km.raycast(ox, oy, s.yaw + eyaw, self.angles, circles, boxes, self.range_max)
        finite = np.isfinite(ranges)
        if self.range_sigma > 0.0 and np.any(finite):
            ranges[finite] += self.rng.normal(0.0, self.range_sigma, int(np.sum(finite)))
        ranges[finite & (ranges < self.range_min)] = self.range_min
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.lidar_frame
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max)
        msg.angle_increment = float((self.angle_max - self.angle_min) / max(self.samples - 1, 1))
        msg.scan_time = float(self.scan_period)
        msg.time_increment = 0.0
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = [float(r) for r in ranges]
        for pub in self.pub_scans:
            pub.publish(msg)

    def _camera_info(self, stamp, key: str) -> CameraInfo:
        w = int(self.param(f'{key}.width', 640))
        h = int(self.param(f'{key}.height', 480))
        hfov = self.param(f'{key}.hfov', 1.518436)
        fx = w / (2.0 * math.tan(hfov / 2.0))
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame(self.param(f'{key}.frame_id', 'camera_optical_frame'))
        msg.width, msg.height = w, h
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0] * 5
        msg.k = [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fx, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg


def main(args=None) -> None:
    run(KinematicSim, args)


if __name__ == '__main__':
    main()
