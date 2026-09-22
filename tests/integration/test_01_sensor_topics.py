"""
시나리오 01: 시뮬레이션 기동 및 센서 토픽 발행 (명세 4.1 센서 시스템 구성).

Gazebo 헤드리스(amr_simulation/warehouse.launch.py: 월드 + amr_description 스폰 + ros_gz 브리지)를
띄우고, 로봇 네임스페이스의 센서 토픽 주기를 header.stamp(sim time)로 잰다. 호스트 부하로 RTF < 1
이면 wall 주기는 낮아도 센서가 sim time 에서 규정 주기로 발행하는지가 명세 기준이다 (rates.py).

  주기    scan 10 · imu(원시) 100 · RGB 30 · depth 15 · camera_info 30/15 · joint_states 50 ·
          ground_truth/odom 50 Hz (sensors.yaml update_rate)
  규격    LiDAR 720 빔 / 0.5° / 360° / 25 m, 카메라 640×480 · 수평 FOV 87°(camera_info K 에서 역산)
  노이즈  정지 상태 LiDAR 빔별 σ ≈ 0.03 m, IMU 자이로·가속도 σ > 0 (명세 7장 노이즈 모델 적용)
이미지는 raw 구독 후 CDR 헤더만 읽는다 (cdr.py — 역직렬화 부하로 주기가 왜곡되지 않게).
"""

import math
from typing import List, NamedTuple

from amr_itest import cases, catalog, config, rates
from amr_itest.scenario import Context
from amr_itest.stack import Stack
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan

CTX = Context(catalog.get(1))

MEASURE_SIM_S = 5.0        # [s] sim time 측정 창
STARTUP_WALL_S = 240.0     # [s] Gazebo 기동 + 스폰 + 첫 센서 메시지 상한 (고부하 호스트 기준)


class Topic(NamedTuple):
    key: str
    topic: str
    msg_type: type
    nominal_hz: float
    raw: bool


def sensor_topics() -> List[Topic]:
    """sensors.yaml 에서 측정 대상 토픽과 규정 주기."""
    s = config.sensors()
    g = config.get
    enc = g(s, 'wheel_encoder.update_rate', 50.0)
    rgb = g(s, 'rgb_camera.update_rate', 30.0)
    depth = g(s, 'depth_camera.update_rate', 15.0)
    return [
        Topic('lidar', g(s, 'lidar.topic', 'scan'), LaserScan,
              g(s, 'lidar.update_rate', 10.0), False),
        Topic('imu', g(s, 'imu.topic', 'imu/data_raw'), Imu, g(s, 'imu.update_rate', 100.0),
              False),
        Topic('rgb_image', g(s, 'rgb_camera.topic', 'camera/image_raw'), Image, rgb, True),
        Topic('rgb_info', g(s, 'rgb_camera.info_topic', 'camera/camera_info'), CameraInfo, rgb,
              False),
        Topic('depth_image', g(s, 'depth_camera.topic', 'camera/depth/image_raw'), Image, depth,
              True),
        Topic('depth_info', g(s, 'depth_camera.info_topic', 'camera/depth/camera_info'),
              CameraInfo, depth, False),
        Topic('joint_states', 'joint_states', JointState, enc, False),
        Topic('ground_truth', 'ground_truth/odom', Odometry, enc, False),
    ]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.simulator()
    return stack.launch_description(), {'stack': stack}


class TestSensorTopics(cases.ProbeCase):
    """센서 토픽 주기·규격·노이즈."""

    CTX = CTX

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        cls.topics = sensor_topics()
        cls.recs = {}

    def test_05_ready(self) -> None:
        """기동 게이트 (launch_testing_ros WaitForTopics): GT·joint_states 브리지 흐름."""
        self.ready_gate([('ground_truth/odom', Odometry), ('joint_states', JointState)],
                        STARTUP_WALL_S)

    def test_10_rates(self) -> None:
        """모든 센서 토픽이 규정 주기(sim time)로 발행된다."""
        for t in self.topics:
            ok = self.probe.wait_for_publisher(t.topic, self.timeout(STARTUP_WALL_S))
            self.assertTrue(ok, f'{t.topic}: 발행자 없음 (Gazebo/브리지 기동 실패?)')
            qos = self.probe.matching_qos(t.topic)
            self.recs[t.key] = self.probe.subscribe(t.topic, t.msg_type, qos, raw=t.raw,
                                                    keep_messages=60)
        for t in self.topics:
            self.require_topic(self.recs[t.key], 3, STARTUP_WALL_S, t.topic)
        # 모든 토픽이 흐르기 시작한 뒤의 sim time 창에서 잰다
        wall0, sim0 = self.probe.wall(), self.probe.now()
        self.assertTrue(self.probe.sleep_ros(MEASURE_SIM_S, self.timeout(MEASURE_SIM_S * 60)),
                        'sim time 이 진행하지 않음 (/clock 확인)')
        wall1, sim1 = self.probe.wall(), self.probe.now()
        rtf = (sim1 - sim0) / max(wall1 - wall0, 1e-6)
        self.measure('rtf', round(rtf, 3))
        self.measure('measure_window', {'sim_s': round(sim1 - sim0, 2),
                                        'wall_s': round(wall1 - wall0, 2)})
        rows, failed = [], []
        for t in self.topics:
            rec = self.recs[t.key]
            st = rates.rate_stats(rates.in_window(rec.stamps(), sim0, sim1))
            wall = rates.rate_stats(rec.recv_times(wall0))
            ok = st.meets(t.nominal_hz)
            self.ctx.record.check(f'rate {t.topic}', round(st.mean_rate, 2),
                                  f'>= {0.9 * t.nominal_hz:.1f} (mean), '
                                  f'>= {0.95 * t.nominal_hz:.1f} (median)', ok, 'Hz')
            rows.append([t.topic, t.msg_type.__name__, t.nominal_hz, st.count, st.mean_rate,
                         st.median_rate, st.max_gap, wall.mean_rate, rec.frame_id or '',
                         int(ok)])
            if not ok:
                failed.append(f'{t.topic} {st.mean_rate:.1f}/{st.median_rate:.1f} Hz '
                              f'(규정 {t.nominal_hz})')
        self.ctx.record.write_csv('topic_rates.csv', [
            'topic', 'type', 'nominal_hz', 'count', 'mean_rate_hz', 'median_rate_hz',
            'max_gap_s', 'wall_rate_hz', 'frame_id', 'ok'], rows)
        self.measure('rates_hz', {r[0]: {'mean': round(r[4], 2), 'median': round(r[5], 2),
                                         'wall': round(r[7], 2)} for r in rows})
        self.assertFalse(failed, f'규정 주기 미달: {failed} (RTF {rtf:.2f})')

    def test_20_lidar_spec(self) -> None:
        """라이다: 720 빔 · 0.5° · 360° · 25 m, 정지 노이즈 σ ≈ 0.03 m."""
        s = config.sensors()
        topic = config.get(s, 'lidar.topic', 'scan')
        rec = self.probe.subscribe(topic, LaserScan, self.probe.matching_qos(topic))
        self.require_topic(rec, 20, STARTUP_WALL_S)
        msg = rec.last()
        span = msg.angle_max - msg.angle_min + msg.angle_increment
        self.check('lidar samples', len(msg.ranges), '>= 720', len(msg.ranges) >= 720)
        self.check('lidar resolution', math.degrees(msg.angle_increment), '<= 0.5',
                   math.degrees(msg.angle_increment) <= 0.5 + 1e-3, 'deg')
        self.check('lidar span', math.degrees(span), '360 +- 1',
                   abs(math.degrees(span) - 360) <= 1,
                   'deg')
        self.check('lidar range_max', msg.range_max, '>= 25', msg.range_max >= 25.0 - 1e-6, 'm')
        scans = [m.ranges for _, m in rec.messages()][-20:]
        sigma = rates.per_beam_noise(scans, msg.range_max)
        nominal = config.get(s, 'lidar.noise_stddev', 0.03)
        self.check('lidar noise sigma (stationary)', sigma, f'[{0.5 * nominal}, {2 * nominal}]',
                   math.isfinite(sigma) and 0.5 * nominal <= sigma <= 2.0 * nominal, 'm')

    def test_30_camera_spec(self) -> None:
        """RGB·depth 640×480 이상, 수평 FOV 87° (camera_info)."""
        s = config.sensors()
        for key in ('rgb_camera', 'depth_camera'):
            topic = config.get(s, f'{key}.info_topic', 'camera/camera_info')
            rec = self.probe.subscribe(topic, CameraInfo, self.probe.matching_qos(topic))
            self.require_topic(rec, 1, STARTUP_WALL_S)
            info = rec.last()
            self.check(f'{key} resolution', f'{info.width}x{info.height}', '>= 640x480',
                       info.width >= 640 and info.height >= 480)
            fx = info.k[0]
            hfov = math.degrees(2.0 * math.atan(info.width / (2.0 * fx))) if fx > 0 else math.nan
            self.check(f'{key} hfov', hfov, '87 +- 1', abs(hfov - 87.0) <= 1.0, 'deg')

    def test_40_imu_noise(self) -> None:
        """IMU 노이즈 모델 적용 (정지 상태 표본 표준편차 ≥ 0.5 σ_config)."""
        s = config.sensors()
        rec = self.recs.get('imu')
        self.assertIsNotNone(rec, 'test_10 에서 IMU 구독이 만들어지지 않음')
        msgs = [m for _, m in rec.messages()]
        self.assertGreaterEqual(len(msgs), 50, 'IMU 표본 부족')
        gyro_z = [m.angular_velocity.z for m in msgs]
        acc_x = [m.linear_acceleration.x for m in msgs]
        g_sigma = config.get(s, 'imu.gyro_noise_stddev', 0.0002)
        a_sigma = config.get(s, 'imu.accel_noise_stddev', 0.017)
        g_std, a_std = rates.sample_std(gyro_z), rates.sample_std(acc_x)
        self.measure('imu_gyro_z_mean', round(float(np.mean(gyro_z)), 6))
        self.check('imu gyro noise std', g_std, f'>= {0.5 * g_sigma}', g_std >= 0.5 * g_sigma,
                   'rad/s')
        self.check('imu accel noise std', a_std, f'>= {0.5 * a_sigma}', a_std >= 0.5 * a_sigma,
                   'm/s^2')

    def test_50_wheel_joints(self) -> None:
        """joint_states 에 양쪽 바퀴 관절 (휠 엔코더 입력)."""
        rec = self.recs.get('joint_states')
        self.assertIsNotNone(rec)
        names = set(rec.last().name)
        want = {self.settings.frame('left_wheel_joint'), self.settings.frame('right_wheel_joint')}
        self.check('wheel joints', sorted(names), sorted(want), want <= names)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
