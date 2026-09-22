#!/usr/bin/env python3
"""
추적기 Gazebo 경로 시험기 (tracking.md §9.5, 하네스 tracker_gz.launch.py).

1) 관찰: (3.0, -4.6) 에서 남쪽을 보고 watch 초 동안 정지 — 횡단 작업자(worker_crossing, y = -7,
   1.0 m/s)가 2.4 m 앞을 지나간다 → 1.0 m/s 보행자 참 양성률.
2) 주행: 랙 사이 틈(x = 3)으로 북쪽 y = 0 (B-C 통로)까지, 동쪽 x = 15, 서쪽 x = -15 (speed m/s,
   지면 진실 선 추종, cmd_vel_smoothed → safety_node). 랙 면·기둥을 지나며 위치 추정 오차에서 생기는
   가짜 동적 트랙을 센다.
판정 (지면 진실 = /sim/dynamic_obstacles/tracks, 월드 = map)
  거짓 is_dynamic: 출력 위치가 모든 동적 장애물 참값에서 1.0 m 넘게 떨어진 is_dynamic 출력
  참 양성: 작업자가 로봇 전방 6 m 안일 때 0.5 m 안에 is_dynamic 트랙이 있는 표본 비율
  위치 추정 오차: odometry/filtered_map (EKF map) − ground_truth/odom
"""

import argparse
import json
import math
import os
import sys
import time

WAYPOINTS = [(3.0, -4.6), (3.0, 0.0), (15.0, 0.0), (-15.0, 0.0)]


def main(argv=None) -> int:  # pragma: no cover - Gazebo 통합 시험 (tracking.md §9.5)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--watch', type=float, default=60.0, help='[s wall] 관찰 단계')
    ap.add_argument('--speed', type=float, default=0.4)
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)

    import rclpy
    from amr_msgs.msg import TrackedObstacleArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = Node('tracker_gz_route', namespace='amr_01')
    cmd_pub = node.create_publisher(Twist, 'cmd_vel_smoothed', 10)
    st = {'gt': None, 'est': None, 'truth': [], 'sim': 0.0, 'phase': 'init'}
    loc_err = []
    outputs = []   # (phase, sim, robot, [(x, y, dyn, speed)], truth)

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_gt(msg):
        p = msg.pose.pose
        st['gt'] = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_est(msg):
        p = msg.pose.pose
        st['est'] = (p.position.x, p.position.y, yaw_of(p.orientation))
        if st['gt'] is not None:
            e = st['gt']
            dyaw = math.atan2(math.sin(st['est'][2] - e[2]), math.cos(st['est'][2] - e[2]))
            loc_err.append((math.hypot(st['est'][0] - e[0], st['est'][1] - e[1]), abs(dyaw)))

    def on_truth(msg):
        st['truth'] = [(o.position.x, o.position.y, math.hypot(o.velocity.x, o.velocity.y),
                        int(o.track_id)) for o in msg.obstacles]

    def on_tracks(msg):
        if st['gt'] is None or msg.header.frame_id != 'map':
            return
        tr = [(o.position.x, o.position.y, bool(o.is_dynamic),
               math.hypot(o.velocity.x, o.velocity.y)) for o in msg.obstacles]
        outputs.append((st['phase'], st['sim'], st['gt'], tr, list(st['truth'])))

    node.create_subscription(Odometry, 'ground_truth/odom', on_gt, qos_profile_sensor_data)
    node.create_subscription(Odometry, 'odometry/filtered_map', on_est, 10)
    node.create_subscription(TrackedObstacleArray, '/sim/dynamic_obstacles/tracks', on_truth, 10)
    node.create_subscription(TrackedObstacleArray, 'perception/tracked_obstacles', on_tracks, 10)
    node.create_subscription(Clock, '/clock',
                             lambda m: st.__setitem__('sim', m.clock.sec + 1e-9 * m.clock.nanosec),
                             qos_profile_sensor_data)

    def spin_for(sec, tick=None):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            if tick:
                tick()

    def cmd(v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        cmd_pub.publish(t)

    spin_for(5.0, lambda: cmd(0.0, 0.0))
    if st['gt'] is None:
        print('ground_truth 없음', file=sys.stderr)
        return 1
    sim0 = st['sim']
    wall0 = time.monotonic()
    st['phase'] = 'watch'
    spin_for(args.watch, lambda: cmd(0.0, 0.0))
    st['phase'] = 'drive'
    for (x0, y0), (x1, y1) in zip(WAYPOINTS[:-1], WAYPOINTS[1:]):
        heading = math.atan2(y1 - y0, x1 - x0)
        # 제자리 회전
        t0 = time.monotonic()
        while time.monotonic() - t0 < 60.0:
            e = math.atan2(math.sin(heading - st['gt'][2]), math.cos(heading - st['gt'][2]))
            if abs(e) < 0.03:
                break
            cmd(0.0, max(-0.6, min(0.6, 1.5 * e)))
            spin_for(0.02)
        # 선 추종
        length = math.hypot(x1 - x0, y1 - y0)
        dx, dy = (x1 - x0) / length, (y1 - y0) / length
        t0 = time.monotonic()
        v = 0.0
        while time.monotonic() - t0 < 300.0:
            gx, gy, gyaw = st['gt']
            along = dx * (gx - x0) + dy * (gy - y0)
            if along >= length - 0.05:
                break
            e_lat = dx * (gy - y0) - dy * (gx - x0)
            e_h = math.atan2(math.sin(gyaw - heading), math.cos(gyaw - heading))
            v = min(args.speed, v + 0.02, 0.8 * (length - along) + 0.05)
            cmd(v, max(-0.5, min(0.5, -1.5 * e_lat - 2.0 * e_h)))
            spin_for(0.02)
        spin_for(1.0, lambda: cmd(0.0, 0.0))
    st['phase'] = 'done'
    rtf = (st['sim'] - sim0) / max(time.monotonic() - wall0, 1e-6)

    # ---- 판정
    res = {'rtf': round(rtf, 3), 'load_end': os.getloadavg()[0],
           'sim_s': round(st['sim'] - sim0, 1)}
    for phase in ('watch', 'drive'):
        rows = [r for r in outputs if r[0] == phase]
        dyn = false_dyn = tracks = 0
        for _, _, robot, tr, truth in rows:
            tracks += len(tr)
            for x, y, is_dyn, _ in tr:
                if not is_dyn:
                    continue
                dyn += 1
                if all(math.hypot(x - tx, y - ty) > 1.0 for tx, ty, _, _ in truth):
                    false_dyn += 1
        res[phase] = {'scans': len(rows), 'tracks_per_scan': round(tracks / max(len(rows), 1), 2),
                      'is_dynamic_outputs': dyn, 'false_is_dynamic': false_dyn,
                      'false_rate': round(false_dyn / dyn, 4) if dyn else None,
                      'false_per_scan': round(false_dyn / max(len(rows), 1), 4)}
    # 참 양성: 관찰 단계, 1.0 m/s 작업자가 로봇 전방 6 m 안
    hits = total = 0
    for phase, _, robot, tr, truth in outputs:
        if phase != 'watch':
            continue
        c, s = math.cos(robot[2]), math.sin(robot[2])
        for tx, ty, speed, _ in truth:
            fx = c * (tx - robot[0]) + s * (ty - robot[1])
            if speed < 0.9 or fx < 0.3 or math.hypot(tx - robot[0], ty - robot[1]) > 6.0:
                continue
            total += 1
            if any(is_dyn and math.hypot(x - tx, y - ty) <= 0.5 for x, y, is_dyn, _ in tr):
                hits += 1
    res['walker_1mps'] = {'samples': total, 'detected_dynamic': hits,
                          'tp_rate': round(hits / total, 3) if total else None}
    if loc_err:
        pos = sorted(e for e, _ in loc_err)
        ang = sorted(a for _, a in loc_err)
        res['localization_error'] = {
            'n': len(pos), 'pos_median_m': round(pos[len(pos) // 2], 4),
            'pos_p95_m': round(pos[int(0.95 * (len(pos) - 1))], 4), 'pos_max_m': round(pos[-1], 4),
            'yaw_median_deg': round(math.degrees(ang[len(ang) // 2]), 3),
            'yaw_max_deg': round(math.degrees(ang[-1]), 3)}
    node.destroy_node()
    rclpy.shutdown()
    text = json.dumps(res, indent=1)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
