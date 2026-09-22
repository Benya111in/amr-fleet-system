#!/usr/bin/env python3
"""
obstacle_tracker_node 기능 시험기 — 합성 LaserScan 으로 이동 장애물을 만들고 추정값을 해석해와 비교한다.

로봇(TF odom→base_link)이 plan(+x 직선)을 따라 v_robot 으로 전진하고, 반경 r 의 원형 장애물이
(x0, y0) 에서 속도 (vx, vy) 로 움직인다. 10 Hz 로 scan_filtered(720 빔, σ = noise) + TF + odometry +
plan 을 발행하고 perception/tracked_obstacles 를 받아 다음을 잰다:
  속도 오차 |v̂| − |v|, 방향 오차, TTC 오차 (해석해: 같은 경로 추종 모델, 반경 합 R = r_robot + r),
  스캔 스탬프 → 수신 지연. 결과는 JSON (stdout, --out).

    ros2 run amr_perception tracker_scenario.py --duration 6 --out /tmp/tracker.json
기본 시나리오 = 명세 4.7 "1.0 m/s 동적 장애물": 로봇 1.0 m/s 로 +x, 장애물 (4, −4) 에서 +y 1.0 m/s
→ t = 4 s 에 (4, 0) 에서 정면 교차(충돌 경로).
"""

import argparse
import json
import math
import sys
import time

import numpy as np


def analytic_ttc(t, args, robot_speed, horizon=5.0, step=0.001):
    """해석 TTC: 로봇 (x_r0 + v t, 0), 장애물 p0 + v_o t, |d| <= R 인 첫 τ (없으면 inf)."""
    r = args.robot_radius + args.radius
    for tau in np.arange(0.0, horizon + step, step):
        tt = t + tau
        pr = np.array([robot_speed * tt, 0.0])
        po = np.array([args.x0 + args.vx * tt, args.y0 + args.vy * tt])
        if np.linalg.norm(pr - po) <= r:
            return tau
    return math.inf


def raycast(origin, yaw, center, radius, n=720, noise=0.03, rng=None, extra=()):
    """360° 스캔: 원 장애물 (center, radius) + 추가 원들. angle_min = −π, 증분 2π/n."""
    angles = -math.pi + np.arange(n) * (2.0 * math.pi / n)
    ranges = np.full(n, np.inf)
    circles = [(np.asarray(center), radius)] + list(extra)
    for i, a in enumerate(angles):
        d = np.array([math.cos(a + yaw), math.sin(a + yaw)])
        for c, rad in circles:
            oc = origin - c
            b = oc @ d
            disc = b * b - (oc @ oc - rad * rad)
            if disc < 0.0:
                continue
            t = -b - math.sqrt(disc)
            if t > 1e-6:
                ranges[i] = min(ranges[i], t)
    if rng is not None and noise > 0.0:
        finite = np.isfinite(ranges)
        ranges[finite] += rng.normal(0.0, noise, int(finite.sum()))
    return angles, ranges


def main(argv=None) -> int:  # pragma: no cover - ROS 통합 시험 (docs/algorithms/tracking.md §7)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=6.0)
    ap.add_argument('--robot-speed', type=float, default=1.0)
    ap.add_argument('--x0', type=float, default=4.0)
    ap.add_argument('--y0', type=float, default=-4.0)
    ap.add_argument('--vx', type=float, default=0.0)
    ap.add_argument('--vy', type=float, default=1.0)
    ap.add_argument('--radius', type=float, default=0.2)
    ap.add_argument('--robot-radius', type=float, default=0.361)
    ap.add_argument('--noise', type=float, default=0.03)
    ap.add_argument('--settle', type=float, default=1.5, help='[s] 이 시각 이후만 오차 집계')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)

    import rclpy
    from amr_msgs.msg import TrackedObstacleArray
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init()
    node = Node('tracker_scenario')
    tfb = TransformBroadcaster(node)
    stb = StaticTransformBroadcaster(node)
    scan_pub = node.create_publisher(LaserScan, 'scan_filtered', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, 'odometry/filtered_map', 10)
    plan_pub = node.create_publisher(Path, 'plan', 10)
    rng = np.random.default_rng(args.seed)
    rows = []
    sent = {}

    def tf(parent, child, x, y, z, yaw, stamp):
        m = TransformStamped()
        m.header.frame_id = parent
        m.header.stamp = stamp
        m.child_frame_id = child
        m.transform.translation.x, m.transform.translation.y = x, y
        m.transform.translation.z = z
        m.transform.rotation.z = math.sin(yaw / 2)
        m.transform.rotation.w = math.cos(yaw / 2)
        return m

    stb.sendTransform([tf('base_link', 'lidar_link', 0.15, 0.0, 0.20, 0.0, node.get_clock().now()
                          .to_msg())])
    t_start = None

    def on_tracks(msg):
        now = time.monotonic()
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key not in sent:
            return
        t, wall = sent[key]
        truth = np.array([args.x0 + args.vx * t, args.y0 + args.vy * t])
        best = None
        for o in msg.obstacles:
            d = math.hypot(o.position.x - truth[0], o.position.y - truth[1])
            if d < 0.6 and (best is None or d < best[0]):
                best = (d, o)
        row = {'t': round(t, 3), 'latency_ms': (now - wall) * 1e3, 'n_tracks': len(msg.obstacles)}
        if best is not None:
            o = best[1]
            speed = math.hypot(o.velocity.x, o.velocity.y)
            true_heading = math.atan2(args.vy, args.vx)
            row.update({
                'pos_err': best[0], 'speed': speed,
                'speed_err': speed - math.hypot(args.vx, args.vy),
                'heading_err_deg': math.degrees(math.remainder(o.heading - true_heading,
                                                               2 * math.pi)),
                'is_dynamic': bool(o.is_dynamic), 'confidence': float(o.confidence),
                'track_id': int(o.track_id), 'ttc': float(o.time_to_collision),
                'ttc_ref': analytic_ttc(t, args, args.robot_speed)})
        rows.append(row)

    node.create_subscription(TrackedObstacleArray, 'perception/tracked_obstacles', on_tracks, 10)

    def tick():
        nonlocal t_start
        if t_start is None:
            t_start = time.monotonic()
        t = time.monotonic() - t_start
        stamp = node.get_clock().now().to_msg()
        x_r = args.robot_speed * t
        tfb.sendTransform([tf('map', 'odom', 0.0, 0.0, 0.0, 0.0, stamp),
                           tf('odom', 'base_link', x_r, 0.0, 0.18, 0.0, stamp)])
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'map'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = x_r
        od.twist.twist.linear.x = args.robot_speed
        odom_pub.publish(od)
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'map'
        for x in np.arange(0.0, 20.0, 0.25):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        plan_pub.publish(path)
        obstacle = np.array([args.x0 + args.vx * t, args.y0 + args.vy * t])
        angles, ranges = raycast(np.array([x_r + 0.15, 0.0]), 0.0, obstacle, args.radius,
                                 noise=args.noise, rng=rng)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / 720
        scan.angle_max = scan.angle_min + 719 * scan.angle_increment
        scan.range_min = 0.1
        scan.range_max = 25.0
        scan.scan_time = 0.1
        scan.ranges = [float(r) for r in ranges]
        sent[(stamp.sec, stamp.nanosec)] = (t, time.monotonic())
        scan_pub.publish(scan)

    node.create_timer(0.1, tick)
    end = time.monotonic() + args.duration + 3.0   # 앞 3 s 는 발견·TF 대기 여유가 아니라 전체 상한
    t_wait = time.monotonic() + 2.0                # tracker 가 구독을 맺을 시간
    while time.monotonic() < t_wait:
        rclpy.spin_once(node, timeout_sec=0.05)
    while rclpy.ok() and time.monotonic() < end and (t_start is None or
                                                     time.monotonic() - t_start < args.duration):
        rclpy.spin_once(node, timeout_sec=0.01)
    t_flush = time.monotonic() + 0.5
    while time.monotonic() < t_flush:
        rclpy.spin_once(node, timeout_sec=0.01)
    node.destroy_node()
    rclpy.shutdown()

    matched = [r for r in rows if 'speed' in r]
    steady = [r for r in matched if r['t'] >= args.settle]
    ttc_rows = [r for r in steady if math.isfinite(r['ttc']) and math.isfinite(r['ttc_ref'])]
    first_dyn = next((r['t'] for r in matched if r['is_dynamic']), None)
    summary = {
        'scenario': vars(args), 'scans_received': len(rows), 'matched': len(matched),
        'steady_samples': len(steady),
        'first_confirmed_t': matched[0]['t'] if matched else None,
        'first_dynamic_t': first_dyn,
        'track_ids': sorted({r['track_id'] for r in matched}),
        'speed_err_mean': float(np.mean([r['speed_err'] for r in steady])) if steady else None,
        'speed_err_max_abs': float(np.max([abs(r['speed_err']) for r in steady]))
        if steady else None,
        'heading_err_max_abs_deg': float(np.max([abs(r['heading_err_deg']) for r in steady]))
        if steady else None,
        'pos_err_mean': float(np.mean([r['pos_err'] for r in steady])) if steady else None,
        'ttc_samples': len(ttc_rows),
        'ttc_err_max_abs': float(np.max([abs(r['ttc'] - r['ttc_ref']) for r in ttc_rows]))
        if ttc_rows else None,
        'ttc_err_mean': float(np.mean([r['ttc'] - r['ttc_ref'] for r in ttc_rows]))
        if ttc_rows else None,
        'latency_ms_p50': float(np.percentile([r['latency_ms'] for r in rows], 50))
        if rows else None,
        'latency_ms_p95': float(np.percentile([r['latency_ms'] for r in rows], 95))
        if rows else None,
        'rows': rows,
    }
    text = json.dumps(summary, indent=1)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
    print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}, indent=1))
    return 0 if matched else 1


if __name__ == '__main__':
    sys.exit(main())
