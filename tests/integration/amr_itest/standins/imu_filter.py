"""
imu_filter: amr_localization/imu_filter_node 대역 — imu/data_raw → imu/data (components.md §3.2).

Gazebo 백엔드에서 실제 필터가 없을 때 EKF 가 imu/data 를 받을 수 있게 한다. 기동 직후
calibration_time [s] 동안(로봇 정지 가정) 자이로 평균을 바이어스로 잡아 이후 표본에서 뺀다.
가속도는 그대로 넘긴다 (EKF 가 imu0_remove_gravitational_acceleration 으로 중력을 뺀다).
실제 노드의 저역 통과·온라인 바이어스 추적은 하지 않는다 (대역 범위 밖).
"""

from typing import List

from amr_itest.standins.common import RELIABLE, run, SENSOR, StandinNode
import numpy as np
from sensor_msgs.msg import Imu


class ImuFilter(StandinNode):
    """정지 구간 자이로 바이어스 추정 + 보정."""

    def __init__(self):
        super().__init__('imu_filter_node')
        self.calib_time = self.param('calibration_time', 2.0)
        self.samples: List[np.ndarray] = []
        self.t0 = None
        self.bias = None
        self.pub = self.create_publisher(Imu, 'imu/data', RELIABLE)
        self.create_subscription(Imu, str(self.param('input_topic', 'imu/data_raw')),
                                 self.on_imu, SENSOR)

    def on_imu(self, msg: Imu) -> None:
        g = msg.angular_velocity
        gyro = np.array([g.x, g.y, g.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.bias is None:
            if self.t0 is None:
                self.t0 = t
            self.samples.append(gyro)
            if t - self.t0 < self.calib_time:
                return
            self.bias = np.mean(self.samples, axis=0)
            self.get_logger().info(f'자이로 바이어스 {self.bias.round(5).tolist()} rad/s '
                                   f'({len(self.samples)} 표본)')
        out = msg
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = \
            (float(v) for v in gyro - self.bias)
        self.pub.publish(out)


def main(args=None) -> None:
    run(ImuFilter, args)


if __name__ == '__main__':
    main()
