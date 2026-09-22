#!/usr/bin/env python3
"""
safety_node 기능 시험기 — 합성 스캔·센서 생존 신호·E-stop 을 넣고 cmd_vel 응답을 잰다.

단계 (모두 한 번에 순서대로)
  A 접근   : 전방 장애물이 2.0 m 에서 approach_speed 로 다가온다 (스캔 10 Hz). 존별 최대 출력 속도와
             "D <= 0.30 스캔 발행 → cmd_vel 0 수신" 지연을 잰다 (safety_node 50 Hz, 명세 0.3 m 즉시 정지).
             근접 정지는 zone 3 STOP 이고 estop_active 가 아니다 (계약 C1).
  B 센서   : 장애물을 치우고, 휠 엔코더 한 번 늦음(60 ms) → 정지 없음; IMU 발행 중단 → degraded(0.2 m/s)
             까지 지연; 휠 엔코더 끊김 → 정지 + estop_active 까지 지연; LiDAR 끊김 → 정지
             (지연 = safety.sensor_timeouts, 고장 = sensor_fault_periods / update_rate).
  C E-stop : estop=true (transient_local) → 정지; 입력이 true 인 동안 reset → 거절; estop=false → 여전히
             정지(래치); safety/reset_estop 호출 → 해제. 이어서 volatile 발행자로 같은 순서.
결과는 JSON (stdout, --out).

    ros2 run amr_perception safety_scenario.py --out /tmp/safety.json
"""

import argparse
import json
import math
import sys
import time


def main(argv=None) -> int:  # pragma: no cover - ROS 통합 시험 (docs/algorithms/tracking.md §7)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cmd-speed', type=float, default=1.0, help='[m/s] 입력 명령')
    ap.add_argument('--approach-speed', type=float, default=0.5, help='[m/s] 장애물 접근 속도')
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)

    import numpy as np
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile, \
        QoSReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Imu, LaserScan
    from std_msgs.msg import Bool, String, UInt8
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = Node('safety_scenario')
    latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    cmd_pub = node.create_publisher(Twist, 'cmd_vel_smoothed', 10)
    scan_pub = node.create_publisher(LaserScan, 'scan_filtered', qos_profile_sensor_data)
    wheel_pub = node.create_publisher(Odometry, 'wheel_odom', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
    rgb_pub = node.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
    depth_pub = node.create_publisher(CameraInfo, 'camera/depth/camera_info',
                                      qos_profile_sensor_data)
    estop_pub = node.create_publisher(Bool, 'estop', latched)
    estop_vol_pub = node.create_publisher(
        Bool, 'estop', QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                  durability=QoSDurabilityPolicy.VOLATILE))
    reset_cli = node.create_client(Trigger, 'safety/reset_estop')
    stb = StaticTransformBroadcaster(node)
    m = TransformStamped()
    m.header.frame_id = 'base_link'
    m.child_frame_id = 'lidar_link'
    m.header.stamp = node.get_clock().now().to_msg()
    m.transform.translation.x = 0.15
    m.transform.translation.z = 0.02   # sensors.yaml lidar extrinsic (지면 +0.20 m)
    m.transform.rotation.w = 1.0
    stb.sendTransform([m])

    state = {'obstacle': None, 'imu': True, 'lidar': True, 'wheel': True, 'cmd': args.cmd_speed}
    log = []          # (wall, linear)
    zone = {'name': '', 'level': -1, 'estop': None}
    events = []

    def on_cmd(msg):
        log.append((time.monotonic(), msg.linear.x, msg.angular.z))

    node.create_subscription(Twist, 'cmd_vel', on_cmd, 50)
    node.create_subscription(String, 'safety/zone_name',
                             lambda m: zone.__setitem__('name', m.data), latched)
    node.create_subscription(UInt8, 'safety/zone',
                             lambda m: zone.__setitem__('level', m.data), latched)
    node.create_subscription(Bool, 'safety/estop_active',
                             lambda m: zone.__setitem__('estop', m.data), latched)

    def scan_msg(front_dist):
        n = 720
        angles = -math.pi + np.arange(n) * (2.0 * math.pi / n)
        ranges = np.full(n, np.inf)
        if front_dist is not None:
            # base_link 전면 모서리(x = 0.30)에서 front_dist 떨어진 벽(폭 1 m) — LiDAR 는 x = 0.15
            x_wall = 0.30 + front_dist - 0.15
            for i, a in enumerate(angles):
                c = math.cos(a)
                if c > 1e-3:
                    r = x_wall / c
                    if abs(r * math.sin(a)) <= 0.5:
                        ranges[i] = r
        s = LaserScan()
        s.header.stamp = node.get_clock().now().to_msg()
        s.header.frame_id = 'lidar_link'
        s.angle_min = -math.pi
        s.angle_increment = 2.0 * math.pi / n
        s.angle_max = s.angle_min + (n - 1) * s.angle_increment
        s.range_min = 0.1
        s.range_max = 25.0
        s.ranges = [float(r) for r in ranges]
        return s

    def hb_fast():
        t = Twist()
        t.linear.x = state['cmd']
        cmd_pub.publish(t)
        if state['wheel']:
            wheel_pub.publish(Odometry())

    def hb_imu():
        if state['imu']:
            imu_pub.publish(Imu())

    def hb_cam():
        rgb_pub.publish(CameraInfo())

    def hb_depth():
        depth_pub.publish(CameraInfo())

    def hb_scan():
        if state['lidar']:
            scan_pub.publish(scan_msg(state['obstacle']))
            events.append(('scan', time.monotonic(), state['obstacle']))

    node.create_timer(0.02, hb_fast)
    node.create_timer(0.01, hb_imu)
    node.create_timer(1.0 / 30.0, hb_cam)
    node.create_timer(1.0 / 15.0, hb_depth)
    node.create_timer(0.1, hb_scan)

    def spin_for(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)

    def first_after(t0, pred, limit=3.0):
        for w, v, _ in log:
            if w >= t0 and pred(v):
                return (w - t0) * 1e3
        return None

    result = {}
    estop_pub.publish(Bool(data=False))
    spin_for(2.0)                                  # 연결·기동 유예
    # --- A 접근
    t0 = time.monotonic()
    log_start = len(log)
    d = 2.0
    while d > 0.1:
        state['obstacle'] = d
        spin_for(0.1)
        d = 2.0 - args.approach_speed * (time.monotonic() - t0)
    spin_for(0.5)
    scans = [(w, dd) for tag, w, dd in events if tag == 'scan' and w >= t0]
    stop_scan_wall = next((w for w, dd in scans if dd is not None and dd <= 0.30), None)

    def bucket(dd):
        if dd is None or dd > 1.0:
            return 'CLEAR (>1.0 m)'
        if dd > 0.5:
            return 'WARNING (0.5-1.0 m)'
        if dd > 0.3:
            return 'CRITICAL (0.3-0.5 m)'
        return 'STOP (<=0.3 m)'

    zones = {}
    for w, v, _ in log[log_start:]:
        # 출력 시각보다 60 ms 이상 앞서 발행된 마지막 스캔의 거리 (노드가 처리했을 스캔)
        prev = [dd for sw, dd in scans if sw <= w - 0.06]
        if prev:
            zones.setdefault(bucket(prev[-1]), []).append(v)
    result['A_approach'] = {
        'cmd_in': args.cmd_speed,
        'max_out_by_zone': {k: round(max(v), 3) for k, v in zones.items()},
        'samples_by_zone': {k: len(v) for k, v in zones.items()},
        'stop_latency_ms': first_after(stop_scan_wall, lambda v: v == 0.0)
        if stop_scan_wall else None,
        # D <= 0.30 스캔 발행 후 첫 0 명령 전까지 받은 0 이 아닌 cmd_vel 수 (50 Hz 주기 기준 "한 주기 이내")
        'nonzero_cmds_after_stop_scan': sum(
            1 for w, v, _ in log
            if stop_scan_wall and w >= stop_scan_wall and v != 0.0 and w < stop_scan_wall + (
                first_after(stop_scan_wall, lambda x: x == 0.0) or 0.0) * 1e-3),
        'estop_active_after_stop': zone['estop'], 'zone_after_stop': zone['name'],
        'zone_level_after_stop': zone['level'],
    }
    # --- B 센서 고장
    state['obstacle'] = None
    spin_for(1.5)                                  # 정지 래치 해제 (거리 > 0.5)
    released = [v for w, v, _ in log if w > time.monotonic() - 0.2]
    # 휠 엔코더 한 번 늦음 (60 ms = 3 주기) → 지연 경고만
    n0 = len(log)
    state['wheel'] = False
    spin_for(0.06)
    state['wheel'] = True
    spin_for(0.5)
    single_gap_zero = sum(1 for _, v, _ in log[n0:] if v == 0.0)
    single_gap_estop = zone['estop']
    state['imu'] = False
    t_imu = time.monotonic()
    spin_for(0.6)
    degraded_lat = first_after(t_imu, lambda v: 0.0 < v <= 0.2 + 1e-6)
    degraded_vals = [v for w, v, _ in log if w > time.monotonic() - 0.2]
    state['imu'] = True
    spin_for(0.6)
    recovered = [v for w, v, _ in log if w > time.monotonic() - 0.2]
    state['wheel'] = False
    t_wheel = time.monotonic()
    spin_for(0.6)
    wheel_lat = first_after(t_wheel, lambda v: v == 0.0)
    wheel_estop = zone['estop']
    state['wheel'] = True
    spin_for(0.6)
    state['lidar'] = False
    t_lidar = time.monotonic()
    spin_for(1.0)
    lidar_lat = first_after(t_lidar, lambda v: v == 0.0)
    lidar_estop = zone['estop']
    state['lidar'] = True
    spin_for(1.0)
    result['B_sensor_timeout'] = {
        'released_speed': round(max(released), 3) if released else None,
        'wheel_single_60ms_gap_zero_cmds': single_gap_zero,
        'wheel_single_60ms_gap_estop_active': single_gap_estop,
        'wheel_dropout_to_stop_ms': wheel_lat, 'wheel_dropout_estop_active': wheel_estop,
        'imu_timeout_to_degraded_ms': degraded_lat,
        'degraded_speed': round(max(degraded_vals), 3) if degraded_vals else None,
        'after_imu_recovery_speed': round(max(recovered), 3) if recovered else None,
        'lidar_timeout_to_stop_ms': lidar_lat, 'lidar_timeout_estop_active': lidar_estop,
    }
    # --- C E-stop 래치 (transient_local 발행자, 이어서 volatile 발행자)

    def reset():
        if not reset_cli.wait_for_service(timeout_sec=2.0):
            return None
        fut = reset_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        return bool(fut.result().success) if fut.result() else None

    for key, pub in (('C_estop', estop_pub), ('C_estop_volatile', estop_vol_pub)):
        pub.publish(Bool(data=True))
        t_e = time.monotonic()
        spin_for(0.5)
        estop_lat = first_after(t_e, lambda v: v == 0.0)
        rejected = reset()
        pub.publish(Bool(data=False))
        spin_for(0.5)
        still = [v for w, v, _ in log if w > time.monotonic() - 0.3]
        reset_ok = reset()
        spin_for(0.5)
        after = [v for w, v, _ in log if w > time.monotonic() - 0.2]
        result[key] = {
            'estop_to_stop_ms': estop_lat,
            'reset_while_pressed_success': rejected,
            'after_false_max_speed': round(max(still), 3) if still else None,
            'reset_success': reset_ok,
            'after_reset_speed': round(max(after), 3) if after else None,
            'estop_active_final': zone['estop'],
        }
    ts = [w for w, _, _ in log]
    result['cmd_vel_rate_hz'] = round((len(ts) - 1) / (ts[-1] - ts[0]), 1) if len(ts) > 1 \
        else None
    node.destroy_node()
    rclpy.shutdown()
    text = json.dumps(result, indent=1)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
