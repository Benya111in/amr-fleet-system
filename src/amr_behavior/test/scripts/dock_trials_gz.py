#!/usr/bin/env python3
"""
Gazebo 실제 체인 도킹 시험 (fix wave, amr_behavior).

체인: 렌더링 카메라 → aruco_detector_node(설치본) → perception/dock_marker_pose → docking_server_node(설치본)
      → cmd_vel_nav → velocity_profiler_node → cmd_vel_smoothed → safety_node → cmd_vel
      → DiffDrive.
      EKF + AMCL(glue0 지도, map = world) 로 Nav2 가 staging 까지 주행한다.
시도마다: Nav2 로 도크 staging → dock 액션(max_retries 3) → 1 s 정지 뒤 ground_truth/odom 으로 진값 오차
          → Nav2 backup 0.5 m 이탈 → 다음 도크. 도크 목록을 번갈아 돈다.
출력: JSON lines (시도별), 마지막 줄 요약.

    python3 dock_trials_gz.py --docks dock_1,dock_2 --trials 10 --out log/bhv_trials/inbound.jsonl
"""

import argparse
import importlib.util
import json
import math
import sys
import threading
import time

from amr_msgs.action import Dock
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt8
import yaml

# robot_params.yaml footprint 0.60 × 0.40, sensors.yaml lidar extrinsic x 0.15 (base_link 기준)
HALF_L, HALF_W, LIDAR_X = 0.30, 0.20, 0.15


def footprint_distance(x, y):
    dx = max(abs(x) - HALF_L, 0.0)
    dy = max(abs(y) - HALF_W, 0.0)
    return math.hypot(dx, dy)


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def marker_faces(world_gen):
    """월드 생성기 → 도크 id: (판 면 x, y, 바깥 법선 방위)."""
    spec = importlib.util.spec_from_file_location('gen_world', world_gen)
    w = importlib.util.module_from_spec(spec)
    argv = sys.argv
    sys.argv = ['gen']
    spec.loader.exec_module(w)
    sys.argv = argv
    faces = {}
    for name, (_x, y, kind, _m) in w.DOCKS.items():
        faces[name] = ((-w.HALF_X + 0.02, y, 0.0) if kind == 'inbound'
                       else (w.HALF_X - 0.02, y, math.pi))
    for name, (x, _m) in w.CHARGERS.items():
        faces[f'charger_{name}'] = (x, w.CHARGER_Y + w.STATION_FRONT + 0.02, math.pi / 2)
    return faces


class Trials(Node):
    def __init__(self, ns):
        # use_sim_time 을 쓰지 않는다: 부하가 큰 호스트에서 /clock(수백 Hz)을 파이썬이 따라가지 못해
        # 액션 응답이 늦어진다. 시뮬레이션 시각은 ground_truth/odom 의 stamp 로 잰다.
        super().__init__('bhv_dock_trials')
        self.ns = ns
        self.lock = threading.Lock()
        self.gt = None
        self.gt_stamp = None
        self.gt_hist = []
        self.amcl = None
        self.zone_max = 0
        self.zone_cur = 0
        self.zone_hist = {}
        self.estop_seen = False
        self.estop = False
        self.excl = []
        self.markers = 0
        self.cmd_nonzero = 0
        self.create_subscription(Odometry, f'/{ns}/ground_truth/odom', self._gt, 20)
        self.create_subscription(PoseWithCovarianceStamped, f'/{ns}/amcl_pose', self._amcl,
                                 LATCHED)
        self.create_subscription(UInt8, f'/{ns}/safety/zone', self._zone, 10)
        self.create_subscription(Bool, f'/{ns}/safety/estop_active', self._estop, LATCHED)
        self.create_subscription(PolygonStamped, f'/{ns}/safety/dock_exclusion', self._excl, 10)
        self.create_subscription(PoseStamped, f'/{ns}/perception/dock_marker_pose', self._marker,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, f'/{ns}/cmd_vel', self._cmd, 10)
        self.scan_min = []
        self.scan_on = False
        self.create_subscription(LaserScan, f'/{ns}/scan_filtered', self._scan,
                                 qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
        self.dock = ActionClient(self, Dock, f'/{ns}/dock')
        self.backup = ActionClient(self, BackUp, f'/{ns}/backup')

    def _gt(self, m):
        p = m.pose.pose
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.gt_stamp = t
            self.gt_hist.append((time.monotonic(), t))
            if len(self.gt_hist) > 3000:
                self.gt_hist = self.gt_hist[-1500:]

    def _amcl(self, m):
        p = m.pose.pose
        with self.lock:
            self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _zone(self, m):
        with self.lock:
            self.zone_cur = m.data
            self.zone_max = max(self.zone_max, m.data)
            self.zone_hist[m.data] = self.zone_hist.get(m.data, 0) + 1

    def _estop(self, m):
        with self.lock:
            self.estop = m.data
            self.estop_seen = self.estop_seen or m.data

    def _excl(self, m):
        with self.lock:
            self.excl.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)   # sim 시각

    def _marker(self, m):
        with self.lock:
            self.markers += 1

    def _scan(self, m):
        if not self.scan_on:
            return
        best = float('inf')
        a = m.angle_min
        for r in m.ranges:
            if m.range_min < r < m.range_max:
                x, y = LIDAR_X + r * math.cos(a), r * math.sin(a)
                if not (abs(x) < HALF_L - 0.02 and abs(y) < HALF_W - 0.02):
                    best = min(best, footprint_distance(x, y))
            a += m.angle_increment
        with self.lock:
            self.scan_min.append(best)

    def _cmd(self, m):
        if abs(m.linear.x) > 1e-4 or abs(m.angular.z) > 1e-4:
            with self.lock:
                self.cmd_nonzero += 1

    def reset_window(self):
        with self.lock:
            self.zone_max = self.zone_cur
            self.zone_hist = {}
            self.estop_seen = self.estop
            self.excl = []
            self.markers = 0
            self.cmd_nonzero = 0

    def rtf(self, window=20.0):
        with self.lock:
            c = [s for s in self.gt_hist if s[0] >= time.monotonic() - window]
        if len(c) < 2 or c[-1][0] - c[0][0] < 1.0:
            return None
        return (c[-1][1] - c[0][1]) / (c[-1][0] - c[0][0])

    def sim_now(self):
        with self.lock:
            return self.gt_stamp

    def sim_sleep(self, seconds):
        t0 = self.sim_now()
        while self.sim_now() - t0 < seconds:
            time.sleep(0.05)


def run_action(client, goal, timeout, feedback=None):
    """동기 실행: (결과 코드, 결과, 소요 wall s). 시간 초과면 취소."""
    t0 = time.monotonic()
    if not client.wait_for_server(timeout_sec=60.0):
        return 'no_server', None, 0.0
    fut = client.send_goal_async(goal, feedback_callback=feedback)
    while not fut.done():
        if time.monotonic() - t0 > 120.0:
            return 'no_response', None, time.monotonic() - t0
        time.sleep(0.02)
    handle = fut.result()
    if not handle.accepted:
        return 'rejected', None, time.monotonic() - t0
    res = handle.get_result_async()
    while not res.done():
        if time.monotonic() - t0 > timeout:
            handle.cancel_goal_async()
            return 'timeout', None, time.monotonic() - t0
        time.sleep(0.05)
    r = res.result()
    return {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(r.status, str(r.status)), r.result, \
        time.monotonic() - t0


def pose_msg(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='amr_01')
    ap.add_argument('--docks', required=True)
    ap.add_argument('--trials', type=int, default=10)
    ap.add_argument('--out', required=True)
    ap.add_argument('--config', default='/ros2_ws/src/amr_behavior/config/behavior.yaml')
    ap.add_argument('--world-gen',
                    default='/ros2_ws/src/amr_simulation/worlds/gen_warehouse_world.py')
    ap.add_argument('--nav-timeout', type=float, default=400.0)
    ap.add_argument('--dock-timeout', type=float, default=600.0)
    ap.add_argument('--undock', type=float, default=1.55)
    args = ap.parse_args()

    with open(args.config, encoding='utf-8') as f:
        docks = yaml.safe_load(f)['/**']['ros__parameters']['docks']
    faces = marker_faces(args.world_gen)
    ids = args.docks.split(',')

    rclpy.init()
    node = Trials(args.ns)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t_wait = time.monotonic()
    while node.gt is None and time.monotonic() - t_wait < 120:
        time.sleep(0.2)
    out = open(args.out, 'a', encoding='utf-8')
    rows = []
    for i in range(args.trials):
        dock_id = ids[i % len(ids)]
        sx, sy, syaw = docks[dock_id]['staging']
        standoff = docks[dock_id]['standoff']
        fx, fy, normal = faces[dock_id]
        tx, ty, tyaw = fx + standoff * math.cos(normal), fy + standoff * math.sin(normal), \
            wrap(normal + math.pi)
        row = {'trial': i, 'dock_id': dock_id, 'wall_start': time.strftime('%H:%M:%S')}
        node.reset_window()
        code, _, dt = run_action(node.nav, NavigateToPose.Goal(pose=pose_msg(sx, sy, syaw)),
                                 args.nav_timeout)
        row['nav'] = code
        row['nav_wall_s'] = round(dt, 1)
        with node.lock:
            gt = node.gt
            amcl = node.amcl
        # staging 도착 오차 (도크 프레임: 종·횡·방위)
        c, s = math.cos(syaw), math.sin(syaw)
        dx, dy = gt[0] - sx, gt[1] - sy
        row['staging_long_m'] = round(c * dx + s * dy, 3)
        row['staging_lat_m'] = round(-s * dx + c * dy, 3)
        row['staging_yaw_deg'] = round(math.degrees(wrap(gt[2] - syaw)), 2)
        if amcl:
            row['amcl_err_m'] = round(math.hypot(amcl[0] - gt[0], amcl[1] - gt[1]), 3)
            row['amcl_yaw_err_deg'] = round(math.degrees(wrap(amcl[2] - gt[2])), 2)
        row['nav_zone_max'] = node.zone_max
        row['nav_estop_seen'] = node.estop_seen
        node.reset_window()
        phases = []

        def fb(msg):
            ph = msg.feedback.current_phase
            if not phases or phases[-1] != ph:
                phases.append(ph)
        t_sim0 = node.sim_now()
        rtf0 = node.rtf()
        goal = Dock.Goal(dock_id=dock_id, approach_pose=pose_msg(sx, sy, syaw), max_retries=3)
        code, res, dt = run_action(node.dock, goal, args.dock_timeout, fb)
        dock_sim_s = node.sim_now() - t_sim0
        rtf1 = node.rtf(max(dt, 5.0))
        with node.lock:
            excl = list(node.excl)
            row['dock_markers'] = node.markers
            row['dock_zone_max'] = node.zone_max
            row['dock_zone_hist'] = dict(node.zone_hist)
            row['dock_estop_seen'] = node.estop_seen
            row['dock_cmd_nonzero'] = node.cmd_nonzero
        node.sim_sleep(1.0)
        with node.lock:
            gt = node.gt
        # docked 상태 LiDAR 최소 풋프린트 거리 (2 s sim): 0.30 m 이하 비율 — 도킹 예외가 필요한 이유
        node.scan_min = []
        node.scan_on = True
        node.sim_sleep(2.0)
        node.scan_on = False
        with node.lock:
            mins = [d for d in node.scan_min if math.isfinite(d)]
            row['docked_zone'] = node.zone_cur
            row['docked_estop'] = node.estop
        if mins:
            row['docked_scans'] = len(mins)
            row['docked_scan_min_mean_m'] = round(sum(mins) / len(mins), 3)
            row['docked_scan_min_min_m'] = round(min(mins), 3)
            row['docked_frac_le_030'] = round(sum(1 for d in mins if d <= 0.30) / len(mins), 3)
        row.update({
            'dock': code, 'success': bool(res and res.success),
            'attempts': int(res.attempts_used) if res else None,
            'reported_pos_mm': round(res.final_position_error * 1000, 1) if res else None,
            'reported_ang_deg': round(math.degrees(res.final_angle_error), 3) if res else None,
            'phases': phases, 'dock_wall_s': round(dt, 1), 'dock_sim_s': round(dock_sim_s, 1),
            'rtf': round(rtf1, 3) if rtf1 else (round(rtf0, 3) if rtf0 else None),
        })
        ex_, ey_ = gt[0] - tx, gt[1] - ty
        row['true_pos_mm'] = round(math.hypot(ex_, ey_) * 1000, 1)
        c, s = math.cos(tyaw), math.sin(tyaw)
        row['true_long_mm'] = round((c * ex_ + s * ey_) * 1000, 1)
        row['true_lat_mm'] = round((-s * ex_ + c * ey_) * 1000, 1)
        row['true_ang_deg'] = round(abs(math.degrees(wrap(gt[2] - tyaw))), 3)
        row['within_spec'] = row['success'] and row['true_pos_mm'] <= 20.0 and \
            row['true_ang_deg'] <= 1.0
        if len(excl) >= 2:
            gaps = [b - a for a, b in zip(excl, excl[1:])]
            row['excl_msgs'] = len(excl)
            row['excl_rate_hz_sim'] = round((len(excl) - 1) / max(excl[-1] - excl[0], 1e-6), 1)
            row['excl_max_gap_sim_s'] = round(max(gaps), 3)
        else:
            row['excl_msgs'] = len(excl)
        # 이탈 (Nav2 backup)
        node.reset_window()
        bgoal = BackUp.Goal()
        bgoal.target.x = args.undock
        bgoal.speed = 0.1
        bgoal.time_allowance.sec = 60
        code, _, dt = run_action(node.backup, bgoal, 200.0)
        row['undock'] = code
        row['undock_zone_max'] = node.zone_max
        row['undock_estop_seen'] = node.estop_seen
        row['wall_end'] = time.strftime('%H:%M:%S')
        rows.append(row)
        out.write(json.dumps(row) + '\n')
        out.flush()
        print(json.dumps(row), flush=True)
    ok = [r for r in rows if r['success']]
    summary = {
        'summary': True, 'docks': ids, 'trials': len(rows), 'success': len(ok),
        'within_spec_truth': sum(1 for r in rows if r['within_spec']),
        'true_pos_mm_mean': round(sum(r['true_pos_mm'] for r in ok) / len(ok), 1) if ok else None,
        'true_pos_mm_max': max((r['true_pos_mm'] for r in ok), default=None),
        'true_ang_deg_mean': (round(sum(r['true_ang_deg'] for r in ok) / len(ok), 3)
                              if ok else None),
        'true_ang_deg_max': max((r['true_ang_deg'] for r in ok), default=None),
        'attempts_hist': {str(a): sum(1 for r in rows if r['attempts'] == a) for a in (1, 2, 3)},
        'dock_sim_s_mean': (round(sum(r['dock_sim_s'] for r in rows) / len(rows), 1)
                            if rows else None),
        'estop_seen_any': any(r['dock_estop_seen'] for r in rows),
        'zone_stop_during_dock': sum(1 for r in rows if r['dock_zone_max'] >= 3),
        'rtf_values': [r['rtf'] for r in rows],
    }
    out.write(json.dumps(summary) + '\n')
    out.close()
    print(json.dumps(summary), flush=True)
    ex.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
