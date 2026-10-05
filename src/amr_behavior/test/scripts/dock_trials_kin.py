#!/usr/bin/env python3
"""
docking_server_node 폐루프 정밀도 시험 (fix wave: 마커 자세 = 검출기 규약, 마커 모델 프레임 +x 법선).

모드
  kinematic : 이 스크립트가 차동구동 운동학(100 Hz)으로 cmd_vel_nav 를 적분하고, 가상 마커 관측
              (base_link, 모델 프레임 +x 법선 = aruco_detector_node 규약, 노이즈·누락)을 30 Hz 로 발행한다.
              진값 = 적분 자세.
  gazebo    : Gazebo 로봇의 ground_truth/odom 으로 가상 마커를 만들고 (노이즈·누락), 진값도 그것으로 잰다.
              도킹 서버의 cmd_vel_nav 는 런치에서 cmd_vel 로 리맵한다. 시도마다 set_pose 로 초기화.

사용: python3 dock_trials_kin.py --mode kinematic --trials 15 --out log/bhv_trials/kin.json
"""

import argparse
import json
import math
import random
import subprocess
import threading
import time

from amr_msgs.action import Dock
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_marker(normal_yaw):
    """마커 모델 프레임 (+x = 판 법선, +z = 위): base_link 에서 R_z(normal_yaw)."""
    return (0.0, 0.0, math.sin(0.5 * normal_yaw), math.cos(0.5 * normal_yaw))   # x, y, z, w


class DockWorld(Node):
    """마커는 월드 (mx, my), 바깥 법선 방위 mn. 로봇 진값은 모드에 따라 적분 또는 GT."""

    def __init__(self, args):
        super().__init__('dock_world', namespace=args.ns)
        self.args = args
        self.rng = random.Random(args.seed)
        self.lock = threading.Lock()
        self.mx, self.my, self.mn = args.marker_x, args.marker_y, args.marker_normal
        self.pose = None                      # (x, y, yaw) 월드
        self.cmd = (0.0, 0.0)
        self.marker_pub = self.create_publisher(
            PoseStamped, 'perception/dock_marker_pose', qos_profile_sensor_data)
        if args.mode == 'kinematic':
            self.create_subscription(Twist, 'cmd_vel_nav', self._on_cmd, 10)
            self.create_timer(0.01, self._step)
        else:
            self.create_subscription(Odometry, 'ground_truth/odom', self._on_gt, 20)
        self.create_timer(1.0 / 30.0, self._observe)
        self.visible = True
        self.blackout = None                  # (시작, 끝) monotonic — 이 구간 마커 미검출 (가림 모사)

    def target(self):
        s = self.args.standoff
        return (self.mx + s * math.cos(self.mn), self.my + s * math.sin(self.mn),
                wrap(self.mn + math.pi))

    def _on_cmd(self, msg):
        with self.lock:
            self.cmd = (msg.linear.x, msg.angular.z)

    def _on_gt(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _step(self):
        dt = 0.01
        with self.lock:
            if self.pose is None:
                return
            x, y, yaw = self.pose
            v, w = self.cmd
            mid = yaw + 0.5 * w * dt
            self.pose = (x + v * dt * math.cos(mid), y + v * dt * math.sin(mid), yaw + w * dt)

    def _observe(self):
        with self.lock:
            pose = self.pose
        if pose is None or not self.visible:
            return
        if self.blackout and self.blackout[0] <= time.monotonic() < self.blackout[1]:
            return
        if self.rng.random() < self.args.dropout:
            return
        x, y, yaw = pose
        dx, dy = self.mx - x, self.my - y
        c, s = math.cos(yaw), math.sin(yaw)
        bx = c * dx + s * dy + self.rng.gauss(0.0, self.args.noise_pos)
        by = -s * dx + c * dy + self.rng.gauss(0.0, self.args.noise_pos)
        nyaw = wrap(self.mn - yaw + self.rng.gauss(0.0, math.radians(self.args.noise_yaw_deg)))
        bearing = abs(math.atan2(by, bx))
        if bx <= 0.0 or bearing > math.radians(43.5):   # 카메라 반화각 밖이면 미검출
            return
        msg = PoseStamped()
        msg.header.frame_id = self.args.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = bx, by, 0.07
        qx, qy, qz, qw = quat_marker(nyaw)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.marker_pub.publish(msg)

    def truth_error(self):
        with self.lock:
            pose = self.pose
        tx, ty, tyaw = self.target()
        return math.hypot(pose[0] - tx, pose[1] - ty), abs(wrap(pose[2] - tyaw))


def set_gz_pose(args, x, y, yaw):
    req = (f'name: "{args.ns}", position: {{x: {x}, y: {y}, z: 0.02}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    subprocess.run(['ign', 'service', '-s', f'/world/{args.world}/set_pose',
                    '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '3000', '--req', req], check=False, capture_output=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('kinematic', 'gazebo'), default='kinematic')
    ap.add_argument('--ns', default='amr_01')
    ap.add_argument('--trials', type=int, default=20)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--dock-id', default='dock_1')
    ap.add_argument('--standoff', type=float, default=0.65)
    ap.add_argument('--marker-x', type=float, default=-29.98)
    ap.add_argument('--marker-y', type=float, default=17.0)
    ap.add_argument('--marker-normal', type=float, default=0.0)
    ap.add_argument('--staging', type=float, default=1.69, help='마커 면 ~ 시작 위치 [m]')
    ap.add_argument('--lateral', type=float, default=0.2, help='시작 횡오차 균등 ±[m]')
    ap.add_argument('--heading-deg', type=float, default=15.0, help='시작 방위오차 균등 ±[deg]')
    ap.add_argument('--noise-pos', type=float, default=0.002)
    ap.add_argument('--noise-yaw-deg', type=float, default=0.5)
    ap.add_argument('--dropout', type=float, default=0.05)
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--world', default='warehouse')
    ap.add_argument('--max-retries', type=int, default=3)
    ap.add_argument('--blackout', type=float, nargs=2, default=None,
                    help='시도 시작 후 [a, b) s 동안 마커 가림 (재시도 경로 시험)')
    args = ap.parse_args()

    rclpy.init()
    world = DockWorld(args)
    client_node = Node('dock_trial_client', namespace=args.ns)
    client = ActionClient(client_node, Dock, 'dock')
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(world)
    ex.add_node(client_node)
    threading.Thread(target=ex.spin, daemon=True).start()
    if not client.wait_for_server(timeout_sec=30.0):
        raise SystemExit('dock 서버 없음')

    rng = random.Random(args.seed + 100)
    rows = []
    for i in range(args.trials):
        lat = rng.uniform(-args.lateral, args.lateral)
        dh = math.radians(rng.uniform(-args.heading_deg, args.heading_deg))
        # 시작 자세: 마커 법선 위 staging 거리 + 횡오차, 마커를 바라보는 방위 + 오차
        nx, ny = math.cos(args.marker_normal), math.sin(args.marker_normal)
        sx = args.marker_x + args.staging * nx - lat * ny
        sy = args.marker_y + args.staging * ny + lat * nx
        syaw = wrap(args.marker_normal + math.pi + dh)
        if args.mode == 'kinematic':
            with world.lock:
                world.pose = (sx, sy, syaw)
                world.cmd = (0.0, 0.0)
        else:
            set_gz_pose(args, sx, sy, syaw)
            time.sleep(1.0)
        goal = Dock.Goal()
        if args.blackout:
            now = time.monotonic()
            world.blackout = (now + args.blackout[0], now + args.blackout[1])
        goal.dock_id = args.dock_id
        goal.max_retries = args.max_retries
        t0 = time.monotonic()
        send = client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.01)
        handle = send.result()
        res_future = handle.get_result_async()
        while not res_future.done() and time.monotonic() - t0 < 200:
            time.sleep(0.02)
        dur = time.monotonic() - t0
        result = res_future.result().result if res_future.done() else None
        time.sleep(0.3)   # 정지 후 진값
        pe, ae = world.truth_error()
        row = {'trial': i, 'start_lateral_m': round(lat, 4),
               'start_heading_deg': round(math.degrees(dh), 2),
               'success': bool(result and result.success),
               'attempts': int(result.attempts_used) if result else None,
               'reported_pos_err_m': (round(float(result.final_position_error), 4)
                                      if result else None),
               'reported_ang_err_deg': round(math.degrees(result.final_angle_error), 3)
               if result else None,
               'true_pos_err_m': round(pe, 4), 'true_ang_err_deg': round(math.degrees(ae), 3),
               'duration_s': round(dur, 2)}
        rows.append(row)
        print(json.dumps(row), flush=True)
    ok = [r for r in rows if r['success']]
    within = [r for r in ok if r['true_pos_err_m'] <= 0.02 and r['true_ang_err_deg'] <= 1.0]
    summary = {
        'mode': args.mode, 'trials': len(rows), 'success': len(ok),
        'success_within_spec_truth': len(within),
        'true_pos_err_mean_m': round(sum(r['true_pos_err_m'] for r in ok) / len(ok), 4)
        if ok else None,
        'true_pos_err_max_m': max((r['true_pos_err_m'] for r in ok), default=None),
        'true_ang_err_mean_deg': round(sum(r['true_ang_err_deg'] for r in ok) / len(ok), 3)
        if ok else None,
        'true_ang_err_max_deg': max((r['true_ang_err_deg'] for r in ok), default=None),
        'attempts_hist': {str(a): sum(1 for r in rows if r['attempts'] == a) for a in (1, 2, 3)},
        'duration_mean_s': round(sum(r['duration_s'] for r in rows) / len(rows), 2),
        'args': vars(args)}
    print(json.dumps(summary), flush=True)
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump({'summary': summary, 'rows': rows}, f, indent=1)
    ex.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
