#!/usr/bin/env python3
"""
운동학 시뮬레이터 — Gazebo 없이 Nav2 체인(controller_server → velocity_profiler → cmd)을 폐루프로 시험한다.

차동구동 운동학(정확 원호 적분, 100 Hz) + DiffDrive 속도 서보 근사(순수 지연 + 1차 지연 + 가속 제한기)로
명령을 따라가고, 다음을 발행한다 (모두 상대 토픽, 프레임 접두어 frame_prefix):
  odometry/filtered   nav_msgs/Odometry      <p>odom → <p>base_footprint (EKF 출력 자리)
  /tf                 <p>odom → <p>base_footprint
  ground_truth/odom   nav_msgs/Odometry      map 프레임 참값 (amr_evaluation cte_logger 입력)
  scan_filtered       sensor_msgs/LaserScan  지도(map_yaml) + 이동 장애물 광선 투사, 10 Hz
  perception/tracked_obstacles  amr_msgs/TrackedObstacleArray  이동 장애물 참값 트랙 (DWA VO 시험)
  sim/footprint_clearance  std_msgs/Float64   풋프린트(0.60 x 0.40)와 지도 장애물 최소 거리 [m] (충돌 판정)
구독: cmd_topic (기본 cmd_vel), initialpose (자세 순간 이동, 속도 0).
map → <p>odom 은 항등 (런치의 static_transform_publisher): 위치추정 오차 없는 추종 성능만 본다.
"""
from collections import deque
import math
import sys

from amr_msgs.msg import TrackedObstacle, TrackedObstacleArray
from amr_navigation import warehouse_map
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def integrate_arc(x: float, y: float, th: float, v: float, w: float, dt: float):
    """(v, ω) 를 dt 동안 유지한 원호의 정확 적분."""
    if abs(w) < 1e-9:
        return x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th
    th1 = th + w * dt
    r = v / w
    return x + r * (math.sin(th1) - math.sin(th)), y - r * (math.cos(th1) - math.cos(th)), th1


class ServoAxis:
    """순수 지연 + 1차 지연 + 가속 제한 (DiffDrive 속도 서보 근사)."""

    def __init__(self, tau: float, delay: float, a_max: float, dt: float):
        self.tau = tau
        self.a_max = a_max
        self.dt = dt
        self.buf = deque([0.0] * max(0, int(round(delay / dt))))
        self.value = 0.0

    def step(self, cmd: float) -> float:
        if self.buf:
            self.buf.append(cmd)
            cmd = self.buf.popleft()
        alpha = 1.0 - math.exp(-self.dt / self.tau) if self.tau > 0 else 1.0
        dv = alpha * (cmd - self.value)
        lim = self.a_max * self.dt if self.a_max > 0 else float('inf')
        self.value += max(-lim, min(lim, dv))
        return self.value

    def reset(self):
        self.buf = deque([0.0] * len(self.buf))
        self.value = 0.0


class KinematicSim(Node):
    """운동학 시뮬레이터 노드."""

    def __init__(self):
        super().__init__('kinematic_sim')
        p = self.declare_parameter
        self.prefix = p('frame_prefix', '').value
        self.dt = 1.0 / p('rate', 100.0).value
        self.x = p('x0', 0.0).value
        self.y = p('y0', 0.0).value
        self.th = p('yaw0', 0.0).value
        tau = p('servo_time_constant', 0.08).value
        delay = p('servo_delay', 0.04).value
        self.v_axis = ServoAxis(tau, delay, p('max_linear_accel', 1.0).value, self.dt)
        self.w_axis = ServoAxis(tau, delay, p('max_angular_accel', 2.0).value, self.dt)
        self.odom_noise = p('odom_noise_v', 0.0).value
        self.beams = int(p('scan_beams', 360).value)
        self.scan_range = p('scan_range', 12.0).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.footprint = (p('footprint_length', 0.60).value, p('footprint_width', 0.40).value)
        mov = list(p('moving_obstacles', [0.0]).value)
        self.obs_period = p('obstacle_period', 12.0).value
        self.obstacles = [mov[i:i + 5] for i in range(0, len(mov) - 4, 5)]
        map_yaml = p('map_yaml', '').value
        self.grid = warehouse_map.load_map(map_yaml) if map_yaml else None
        self.rng = np.random.default_rng(1)
        self.cmd = (0.0, 0.0)
        self.last_cmd = self.get_clock().now()
        self.t = 0.0
        self.k = 0
        self.last_tick = None
        self.tf = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.pub_gt = self.create_publisher(Odometry, 'ground_truth/odom', 10)
        self.pub_scan = self.create_publisher(LaserScan, 'scan_filtered', 5)
        self.pub_obs = self.create_publisher(TrackedObstacleArray,
                                             'perception/tracked_obstacles', 5)
        self.pub_clear = self.create_publisher(Float64, 'sim/footprint_clearance', 10)
        self.create_subscription(Twist, p('cmd_topic', 'cmd_vel').value, self.on_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_pose, 10)
        self.create_timer(self.dt, self.on_tick)
        self.get_logger().info(
            f'kinematic_sim: {1.0 / self.dt:.0f} Hz, servo tau {tau} delay {delay}, '
            f'map {map_yaml or "(none)"}, {len(self.obstacles)} moving obstacles')

    def on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd = self.get_clock().now()

    def on_pose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.th = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.v_axis.reset()
        self.w_axis.reset()
        self.cmd = (0.0, 0.0)
        self.get_logger().info(f'teleport to ({self.x:.2f}, {self.y:.2f}, {self.th:.2f})')

    def obstacle_state(self, o, t):
        """왕복 등속 장애물: 반주기마다 속도 반전."""
        x0, y0, vx, vy, r = o
        half = 0.5 * self.obs_period
        ph = t % self.obs_period
        s = ph if ph < half else self.obs_period - ph
        sign = 1.0 if ph < half else -1.0
        return x0 + vx * s, y0 + vy * s, sign * vx, sign * vy, r

    def on_tick(self):
        now = self.get_clock().now()
        vc, wc = self.cmd
        if (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            vc, wc = 0.0, 0.0
        # 타이머 지연(부하)이 있어도 실제 경과 시간만큼 고정 스텝으로 적분 → 시뮬레이션 시각 = 벽시계
        elapsed = self.dt if self.last_tick is None else (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        n = max(1, min(20, int(round(elapsed / self.dt))))
        v = w = 0.0
        for _ in range(n):
            v = self.v_axis.step(vc)
            w = self.w_axis.step(wc)
            self.x, self.y, self.th = integrate_arc(self.x, self.y, self.th, v, w, self.dt)
            self.t += self.dt
        self.k += 1
        stamp = now.to_msg()
        steps_per = max(1, int(round(0.02 / self.dt)))
        if self.k % steps_per == 0:
            self.publish_state(stamp, v, w)
        if self.k % max(1, int(round(0.1 / self.dt))) == 0:
            self.publish_scan(stamp)
            self.publish_obstacles(stamp)

    def publish_state(self, stamp, v, w):
        qx, qy, qz, qw = yaw_to_quat(self.th)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.prefix + 'odom'
        odom.child_frame_id = self.prefix + 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        noise = self.rng.normal(0.0, self.odom_noise) if self.odom_noise > 0 else 0.0
        odom.twist.twist.linear.x = v + noise
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)
        gt = Odometry()
        gt.header.stamp = stamp
        gt.header.frame_id = 'map'
        gt.child_frame_id = self.prefix + 'base_footprint'
        gt.pose = odom.pose
        gt.twist.twist.linear.x = v
        gt.twist.twist.angular.z = w
        self.pub_gt.publish(gt)
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)
        if self.grid is not None:
            self.pub_clear.publish(Float64(data=self.footprint_clearance()))

    def footprint_clearance(self) -> float:
        """풋프린트 사각형과 반경 1 m 안 점유 셀 중심 사이 최소 거리 (안쪽이면 음수 0)."""
        g = self.grid
        cx, cy = g.world_to_cell(self.x, self.y)
        rad = int(math.ceil(1.0 / g.resolution))
        x0, x1 = max(0, cx - rad), min(g.width, cx + rad + 1)
        y0, y1 = max(0, cy - rad), min(g.height, cy + rad + 1)
        iy, ix = np.nonzero(g.occ[y0:y1, x0:x1])
        if len(ix) == 0:
            return 1.0
        px = g.origin[0] + (ix + x0 + 0.5) * g.resolution - self.x
        py = g.origin[1] + (iy + y0 + 0.5) * g.resolution - self.y
        c, s = math.cos(self.th), math.sin(self.th)
        lx = c * px + s * py
        ly = -s * px + c * py
        dx = np.maximum(np.abs(lx) - self.footprint[0] / 2, 0.0)
        dy = np.maximum(np.abs(ly) - self.footprint[1] / 2, 0.0)
        # 셀 반폭만큼 보정: 셀 중심이 아니라 셀 경계까지
        return float(max(0.0, np.min(np.hypot(dx, dy)) - 0.5 * g.resolution))

    def publish_scan(self, stamp):
        ang = np.linspace(-math.pi, math.pi, self.beams, endpoint=False)
        ranges = np.full(self.beams, np.inf)
        if self.grid is not None:
            g = self.grid
            step = 0.5 * g.resolution
            r = np.arange(step, self.scan_range, step)
            wa = self.th + ang
            px = self.x + np.cos(wa)[:, None] * r[None, :]
            py = self.y + np.sin(wa)[:, None] * r[None, :]
            ix = np.floor((px - g.origin[0]) / g.resolution).astype(int)
            iy = np.floor((py - g.origin[1]) / g.resolution).astype(int)
            inside = (ix >= 0) & (iy >= 0) & (ix < g.width) & (iy < g.height)
            hit = np.zeros_like(inside)
            hit[inside] = g.occ[iy[inside], ix[inside]]
            first = np.argmax(hit, axis=1)
            has = hit[np.arange(self.beams), first]
            ranges[has] = r[first[has]]
        for o in self.obstacles:
            ox, oy, _, _, orad = self.obstacle_state(o, self.t)
            dx, dy = ox - self.x, oy - self.y
            wa = self.th + ang
            b = dx * np.cos(wa) + dy * np.sin(wa)
            c = dx * dx + dy * dy - orad * orad
            disc = b * b - c
            ok = (disc >= 0) & (b > 0)
            t_hit = b - np.sqrt(np.where(ok, disc, 0.0))
            ranges = np.where(ok & (t_hit > 0) & (t_hit < ranges), t_hit, ranges)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.prefix + 'base_footprint'
        scan.angle_min = float(ang[0])
        scan.angle_max = float(ang[-1])
        scan.angle_increment = float(ang[1] - ang[0])
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = float(self.scan_range)
        scan.ranges = [float(v) if math.isfinite(v) else float('inf') for v in ranges]
        self.pub_scan.publish(scan)

    def publish_obstacles(self, stamp):
        if not self.obstacles:
            return
        arr = TrackedObstacleArray()
        arr.header.stamp = stamp
        arr.header.frame_id = 'map'
        for i, o in enumerate(self.obstacles):
            ox, oy, vx, vy, _ = self.obstacle_state(o, self.t)
            t = TrackedObstacle()
            t.header = arr.header
            t.track_id = i
            t.position.x = ox
            t.position.y = oy
            t.velocity.x = vx
            t.velocity.y = vy
            t.heading = math.atan2(vy, vx)
            t.confidence = 1.0
            t.is_dynamic = math.hypot(vx, vy) > 0.2
            t.time_to_collision = float('inf')
            arr.obstacles.append(t)
        self.pub_obs.publish(arr)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = KinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
