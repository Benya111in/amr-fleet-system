"""
wheel_odometry: amr_localization/wheel_odometry_node 대역 (components.md §5.2).

  Sub joint_states (<p>left_wheel_joint, <p>right_wheel_joint 위치)
  Pub wheel_odom (nav_msgs/Odometry) frame <p>odom, child <p>base_footprint, TF 는 발행하지 않음

바퀴 각을 엔코더 틱(sensors.yaml wheel_encoder.ticks_per_revolution)으로 양자화하고, 주기마다
바퀴 변위에 (1 + N(0, slip_noise_stddev)) 를 곱한 뒤 순기구학으로 적분한다. twist 공분산은
config/ekf.yaml 주석의 근거대로 σ² = slip² = 1e-4 (vx, vy, vyaw) 를 싣는다.
"""

import math
from typing import Optional

from amr_itest import kinematics as km
from amr_itest.standins.common import RELIABLE, run, StandinNode
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import JointState


class WheelOdometry(StandinNode):
    """joint_states → wheel_odom."""

    def __init__(self):
        super().__init__('wheel_odometry_node')
        self.rng = np.random.default_rng(int(self.param('seed', 11)))
        self.r = self.param('robot.wheel_radius', 0.0825)
        self.sep = self.param('robot.wheel_separation', 0.36)
        self.ticks = int(self.param('wheel_encoder.ticks_per_revolution', 4096))
        self.slip = self.param('wheel_encoder.slip_noise_stddev', 0.01)
        self.left = self.frame('left_wheel_joint')
        self.right = self.frame('right_wheel_joint')
        self.prev: Optional[tuple] = None
        self.x = self.y = self.yaw = 0.0
        self.var = max(self.slip ** 2, 1e-6)
        self.pub = self.create_publisher(Odometry, 'wheel_odom', RELIABLE)
        self.create_subscription(JointState, 'joint_states', self.on_joints, RELIABLE)

    def on_joints(self, msg: JointState) -> None:
        try:
            il, ir = msg.name.index(self.left), msg.name.index(self.right)
        except ValueError:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ql = km.quantize(msg.position[il], self.ticks)
        qr = km.quantize(msg.position[ir], self.ticks)
        if self.prev is None:
            self.prev = (t, ql, qr)
            return
        t0, l0, r0 = self.prev
        dt = t - t0
        if dt <= 0.0:
            return
        self.prev = (t, ql, qr)
        dl = (ql - l0) * (1.0 + float(km.gaussian(self.rng, self.slip)))
        dr = (qr - r0) * (1.0 + float(km.gaussian(self.rng, self.slip)))
        ds, dth = km.body_increment(dl, dr, self.r, self.sep)
        self.x, self.y, self.yaw = km.integrate_pose(self.x, self.y, self.yaw, ds, dth)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame('odom')
        out.child_frame_id = self.frame('base_footprint')
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        pose_cov = [0.0] * 36
        for i in (0, 7, 35):
            pose_cov[i] = 1e-3
        for i in (14, 21, 28):
            pose_cov[i] = 1e6         # 2D: z, roll, pitch 미관측
        out.pose.covariance = pose_cov
        out.twist.twist.linear.x = ds / dt
        out.twist.twist.angular.z = dth / dt
        twist_cov = [0.0] * 36
        for i in (0, 7, 35):
            twist_cov[i] = self.var
        for i in (14, 21, 28):
            twist_cov[i] = 1e6
        out.twist.covariance = twist_cov
        self.pub.publish(out)


def main(args=None) -> None:
    run(WheelOdometry, args)


if __name__ == '__main__':
    main()
