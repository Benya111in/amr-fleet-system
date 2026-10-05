#!/usr/bin/env python3
"""
안전 게이트 Gazebo 기능 시험기 (tracking.md §9.4, 하네스 safety_gz.launch.py).

시나리오 (--scenarios 로 고른다, 기본 전부)
  aisle   (a) 0.60 m 좁은 통로 중심선 주행 N 회 (1.0 m/s, 지면 진실 선 추종) — STOP 수, 속도 상한
  static  (b) 정지 장애물 접근: 사람 원기둥(LiDAR), 저상 상자 0.15 m·지게차 포크(깊이 점군만)
  moving  (b) 이동 장애물: 옆에서 경로로 들어와 멈추는 사람(0.5·1.0 m/s)·저상 상자(0.5 m/s)
  dock    (c) 도크 A 마커 판 앞 standoff(범퍼-판 0.35 m) 접근 — 예외 다각형 스텁(map, 10 Hz) 유무
  estop   (d) 1.0 m/s 주행 중 E-stop → cmd_vel 0 지연, 지면 진실 정지, 거절된 reset·false·reset
  sensor  (e) wheel_odom / imu 중계 끊김 주입 — 단발 간격(60~150 ms)과 실제 끊김
모든 거리는 지면 진실(ground_truth/odom, 이동체 odometry, 모델 치수)로 잰 풋프린트-물체 최단 거리다.
결과 JSON (--out). RTF 는 /clock 과 벽시계로 단계마다 잰다.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

FOOT_L, FOOT_W = 0.60, 0.40
LANE_Y = 14.0                      # 이동체 시험 lane (메인 통로)
FORK_LANE_Y = 17.8                 # 지게차 포크(한쪽 날) 중심선
FORK_TIP_X = 6.0                   # 포크 끝 x (safety_test_world.py FORKLIFT_POSE)
AISLE = dict(x=0.0, y_in=-8.0, y_out=-12.0, half=0.30)
DOCK = dict(face_x=29.98, y=17.0, staging_x=28.29, standoff=0.65)
PARK = {'obs_person': (-6.0, 19.0, 0.852), 'obs_box_low': (-7.0, 19.0, 0.077)}
SHAPES = {'obs_person': ('circle', 0.20), 'obs_box_low': ('box', 0.40, 0.40)}


# ---------------------------------------------------------------- 기하 (순수 함수, 테스트 대상)

def rect_corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for lx, ly in ((length / 2, width / 2), (-length / 2, width / 2),
                   (-length / 2, -width / 2), (length / 2, -width / 2)):
        out.append((x + c * lx - s * ly, y + s * lx + c * ly))
    return out


def _seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def _inside(p, poly):
    """볼록 다각형(반시계/시계 무관) 내부 판정."""
    sign = 0
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(cr) < 1e-12:
            continue
        s = 1 if cr > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return True


def poly_distance(a, b):
    """두 볼록 다각형 최단 거리 (겹치면 0)."""
    if any(_inside(p, b) for p in a) or any(_inside(p, a) for p in b):
        return 0.0
    best = math.inf
    for P, Q in ((a, b), (b, a)):
        for p in P:
            for i in range(len(Q)):
                best = min(best, _seg_dist(p, Q[i], Q[(i + 1) % len(Q)]))
    return best


def footprint_distance(robot, shape, pose):
    """로봇 풋프린트(지면 진실 자세) ↔ 물체 최단 거리. shape: ('circle', r) | ('box', l, w)."""
    fp = rect_corners(robot[0], robot[1], robot[2], FOOT_L, FOOT_W)
    if shape[0] == 'circle':
        c = (pose[0], pose[1])
        if _inside(c, fp):
            return 0.0
        d = min(_seg_dist(c, fp[i], fp[(i + 1) % 4]) for i in range(4))
        return max(0.0, d - shape[1])
    return poly_distance(fp, rect_corners(pose[0], pose[1], pose[2], shape[1], shape[2]))


def braking_tolerance(v, latency=0.15, decel=1.0, frame=0.1):
    """정지 판정 뒤 이동 거리 한도: 스캔 한 주기 + 반응 지연 동안 등속 + 감속 (보수적 a = 1.0)."""
    return v * (latency + frame) + v * v / (2.0 * decel)


def percentile(values, q):
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


# ---------------------------------------------------------------- ROS 시험기

def main(argv=None) -> int:  # pragma: no cover - Gazebo 통합 시험 (tracking.md §9.4)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenarios', default='aisle,static,moving,dock,estop,sensor')
    ap.add_argument('--aisle-runs', type=int, default=10)
    ap.add_argument('--aisle-speed', type=float, default=1.0)
    ap.add_argument('--repeat', type=int, default=1, help='static/moving 반복 배수')
    ap.add_argument('--world', default='warehouse')
    ap.add_argument('--ns', default='amr_01')
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Point32, PolygonStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile, \
        QoSReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Imu, LaserScan, PointCloud2
    from std_msgs.msg import Bool, UInt8
    from std_srvs.srv import Trigger

    rclpy.init()
    node = Node('safety_gz_trials', namespace=args.ns)
    latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    volatile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.VOLATILE)
    cmd_pub = node.create_publisher(Twist, 'cmd_vel_smoothed', 10)
    estop_pub = node.create_publisher(Bool, 'estop', latched)
    estop_vol_pub = node.create_publisher(Bool, 'estop', volatile)
    excl_pub = node.create_publisher(PolygonStamped, 'safety/dock_exclusion', 10)
    wheel_pub = node.create_publisher(Odometry, 'wheel_odom_gated', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, 'imu/data_gated', qos_profile_sensor_data)
    mover_pub = {m: node.create_publisher(Twist, f'/model/{m}/cmd_vel', 10) for m in PARK}
    reset_cli = node.create_client(Trigger, 'safety/reset_estop')

    st = {'robot': None, 'v': 0.0, 'w': 0.0, 'sim': 0.0, 'zone': -1, 'estop': None,
          'out': (0.0, 0.0), 'out_wall': 0.0, 'in': (0.0, 0.0), 'movers': {},
          'drop': {'wheel': False, 'imu': False}, 'diag': {}}
    gaps = {k: [] for k in ('scan_filtered', 'wheel_odom', 'imu/data', 'camera/camera_info',
                            'camera/depth/camera_info', 'camera/depth/points_filtered')}
    sim_gaps = {k: [] for k in gaps}   # safety_node 는 sim 시각으로 감시한다
    last_rx = {}
    last_rx_sim = {}
    trace = []   # (sim, wall, x, y, yaw, v_gt, in_v, out_v, zone, estop, movers)
    cmd_log = []  # (wall, out_v, out_w)
    sim_cmd = []  # (sim, out_v)
    st_hist = {'estop_seen': False}

    def on_gt(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        st['robot'] = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        st['v'] = msg.twist.twist.linear.x
        st['w'] = msg.twist.twist.angular.z
        trace.append((st['sim'], time.monotonic(), *st['robot'], st['v'], st['in'][0],
                      st['out'][0], st['zone'], st['estop'], dict(st['movers'])))

    def on_mover(name):
        def cb(msg):
            st['movers'][name] = (msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0)
        return cb

    def on_out(msg):
        st['out'] = (msg.linear.x, msg.angular.z)
        cmd_log.append((time.monotonic(), msg.linear.x, msg.angular.z))
        sim_cmd.append((st['sim'], msg.linear.x))
        if st['estop']:
            st_hist['estop_seen'] = True

    def stamp_gap(name):
        def cb(_msg):
            now = time.monotonic()
            if name in last_rx:
                gaps[name].append(now - last_rx[name])
                if st['sim'] >= last_rx_sim[name]:
                    sim_gaps[name].append(st['sim'] - last_rx_sim[name])
            last_rx[name] = now
            last_rx_sim[name] = st['sim']
        return cb

    relayed = {'wheel': [], 'imu': []}   # 중계한 sim 시각 (safety_node 가 받은 간격)

    def relay(kind, pub, name):
        def cb(msg):
            stamp_gap(name)(msg)
            if not st['drop'][kind]:
                pub.publish(msg)
                relayed[kind].append(st['sim'])
        return cb

    def on_diag(msg):
        for s in msg.status:
            if s.name.endswith('safety_node'):
                st['diag'] = {kv.key: kv.value for kv in s.values}

    node.create_subscription(Odometry, 'ground_truth/odom', on_gt, qos_profile_sensor_data)
    for m in PARK:
        node.create_subscription(Odometry, f'/model/{m}/odometry', on_mover(m), 10)
    node.create_subscription(Twist, 'cmd_vel', on_out, 50)
    node.create_subscription(UInt8, 'safety/zone', lambda m: st.__setitem__('zone', m.data),
                             latched)
    node.create_subscription(Bool, 'safety/estop_active',
                             lambda m: st.__setitem__('estop', m.data), latched)
    node.create_subscription(DiagnosticArray, 'diagnostics', on_diag, 10)
    node.create_subscription(Clock, '/clock',
                             lambda m: st.__setitem__('sim', m.clock.sec + 1e-9 * m.clock.nanosec),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, 'wheel_odom', relay('wheel', wheel_pub, 'wheel_odom'),
                             qos_profile_sensor_data)
    node.create_subscription(Imu, 'imu/data', relay('imu', imu_pub, 'imu/data'),
                             qos_profile_sensor_data)
    node.create_subscription(LaserScan, 'scan_filtered', stamp_gap('scan_filtered'),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, 'camera/camera_info', stamp_gap('camera/camera_info'),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, 'camera/depth/camera_info',
                             stamp_gap('camera/depth/camera_info'), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, 'camera/depth/points_filtered',
                             stamp_gap('camera/depth/points_filtered'), qos_profile_sensor_data)

    def spin_for(sec, tick=None):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)
            if tick:
                tick()

    def spin_sim(sec, tick=None, until=None):
        """Sim 시각으로 sec 초 스핀한다 (RTF < 1 이면 벽시계로 더 길다). until() 이 참이면 일찍 끝."""
        end = st['sim'] + sec
        while st['sim'] < end:
            rclpy.spin_once(node, timeout_sec=0.005)
            if tick:
                tick()
            if until is not None and until():
                return True
        return False

    def publish_cmd(v, w=0.0):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        cmd_pub.publish(t)
        st['in'] = (v, w)

    def set_pose(name, x, y, z, yaw=0.0):
        req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: '
               f'{{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        subprocess.run(['ign', 'service', '-s', f'/world/{args.world}/set_pose', '--reqtype',
                        'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout',
                        '3000', '--req', req], check=False, capture_output=True)

    def mover_vel(name, vx, vy):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        mover_pub[name].publish(t)

    def park_all():
        for m, (x, y, z) in PARK.items():
            mover_vel(m, 0.0, 0.0)
            set_pose(m, x, y, z)

    def place_robot(x, y, yaw):
        for _ in range(10):
            publish_cmd(0.0)
            spin_for(0.02)
        set_pose(args.ns, x, y, 0.01, yaw)
        spin_for(1.0, lambda: publish_cmd(0.0))

    class Driver:
        """지면 진실 선 추종 명령 (가속 1.0 m/s² 램프, 조향 = 가로·방향 오차 비례)."""

        def __init__(self, p0, heading, speed):
            self.p0, self.h, self.speed = p0, heading, speed
            self.v = 0.0
            self.t = st['sim']

        def tick(self, target=None):
            now = st['sim']
            dt = max(0.0, min(now - self.t, 0.1))
            self.t = now
            goal = self.speed if target is None else target
            step = 1.0 * dt
            self.v = min(goal, self.v + step) if goal >= self.v else max(goal, self.v - step)
            w = 0.0
            if st['robot'] is not None:
                x, y, yaw = st['robot']
                dx, dy = math.cos(self.h), math.sin(self.h)
                e_lat = dx * (y - self.p0[1]) - dy * (x - self.p0[0])
                e_h = math.atan2(math.sin(yaw - self.h), math.cos(yaw - self.h))
                w = max(-0.5, min(0.5, -1.5 * e_lat - 2.0 * e_h))
            publish_cmd(self.v, w if self.v > 0.02 else 0.0)

        def along(self):
            x, y, _ = st['robot']
            return math.cos(self.h) * (x - self.p0[0]) + math.sin(self.h) * (y - self.p0[1])

    def rtf_mark():
        return (st['sim'], time.monotonic())

    def rtf_since(mark):
        ds, dw = st['sim'] - mark[0], time.monotonic() - mark[1]
        return round(ds / dw, 3) if dw > 0 else None

    def loadavg():
        try:
            return os.getloadavg()[0]
        except OSError:
            return None

    def window(t0, t1):
        return [r for r in trace if t0 <= r[1] <= t1]

    results = {'conditions': {}}
    scen = set(args.scenarios.split(','))
    spin_for(3.0, lambda: publish_cmd(0.0))
    if st['robot'] is None:
        print('ground_truth/odom 없음 — 하네스가 떠 있는지 확인', file=sys.stderr)
        return 1
    estop_pub.publish(Bool(data=False))
    park_all()

    # ------------------------------------------------ (a) 좁은 통로
    if 'aisle' in scen:
        runs = []
        mark = rtf_mark()
        for i in range(args.aisle_runs):
            place_robot(AISLE['x'], -6.4, -math.pi / 2)
            drv = Driver((AISLE['x'], -6.4), -math.pi / 2, args.aisle_speed)
            t0 = time.monotonic()
            s0 = st['sim']
            stops = 0
            ticks = stale = 0
            last_zone = st['zone']
            while st['robot'][1] > -13.3 and st['sim'] - s0 < 30.0:
                drv.tick()
                spin_for(0.02)
                if st['zone'] == 3 and last_zone != 3:
                    stops += 1
                last_zone = st['zone']
                ticks += 1
                stale += st['diag'].get('depth_cloud_stale') == 'true'
            while abs(st['v']) > 0.02 and st['sim'] - s0 < 40.0:
                drv.tick(0.0)
                spin_for(0.02)
            t1 = time.monotonic()
            rows = window(t0, t1)
            inside = [r for r in rows if r[3] - FOOT_L / 2 < AISLE['y_in'] and
                      r[3] + FOOT_L / 2 > AISLE['y_out']]
            clear = []
            ratio = []
            for r in inside:
                corners = rect_corners(r[2], r[3], r[4], FOOT_L, FOOT_W)
                clear.append(AISLE['half'] - max(abs(cx - AISLE['x']) for cx, _ in corners))
                if r[6] > 0.1:
                    ratio.append(r[7] / r[6])
            runs.append({
                'stops': stops, 'zone_max': max((r[8] for r in rows), default=-1),
                'estop_seen': any(r[9] for r in rows),
                'time_in_aisle_s': round(inside[-1][0] - inside[0][0], 2) if inside else None,
                'min_clearance_m': round(min(clear), 3) if clear else None,
                'max_gt_speed': round(max((r[5] for r in inside), default=0.0), 3),
                'min_out_over_in': round(min(ratio), 3) if ratio else None,
                'capped_fraction': round(sum(1 for q in ratio if q < 0.99) / len(ratio), 3)
                if ratio else None,
                # 상한 원인 진단: 필수 깊이 점군이 늦어 전진 저속이던 시간 비율 (diagnostics)
                'depth_cloud_stale_fraction': round(stale / max(ticks, 1), 3)})
            print(f'[aisle {i}] {runs[-1]}', flush=True)
        results['aisle'] = {'runs': runs, 'total_stops': sum(r['stops'] for r in runs),
                            'rtf': rtf_since(mark), 'load': loadavg()}

    # ------------------------------------------------ (b) 정지·이동 장애물
    def obstacle_trial(kind, obstacle, speed, lateral, moving=None):
        """Lane 을 따라 speed 로 계속 명령한다 — safety_node 만이 멈춘다."""
        if obstacle == 'forklift':
            lane_y, start_x = FORK_LANE_Y + lateral, FORK_TIP_X - 5.0
            shape, opose = ('box', 1.2, 0.12), (FORK_TIP_X + 0.6, FORK_LANE_Y, 0.0)
        else:
            lane_y, start_x = LANE_Y + lateral, -6.0
            shape = SHAPES[obstacle]
            ox = start_x + 5.0
            if moving is None:
                set_pose(obstacle, ox, LANE_Y, PARK[obstacle][2])
            else:
                set_pose(obstacle, ox, LANE_Y + moving['from'], PARK[obstacle][2])
        place_robot(start_x, lane_y, 0.0)
        drv = Driver((start_x, lane_y), 0.0, speed)
        t0 = time.monotonic()
        s0 = st['sim']
        stop_t = None
        entered = False
        entry = None
        released_mover = False
        while st['sim'] - s0 < 30.0:
            drv.tick()
            spin_for(0.02)
            if moving is not None and obstacle in st['movers']:
                mx, my, _ = st['movers'][obstacle]
                d_now = mx - st['robot'][0]
                if not released_mover and d_now <= moving['trigger']:
                    mover_vel(obstacle, 0.0, -math.copysign(moving['speed'], moving['from']))
                    released_mover = True
                if released_mover and not entered and (my - LANE_Y) * moving['from'] <= 0.0:
                    mover_vel(obstacle, 0.0, 0.0)   # lane 중심에 들어와 멈춘다
                    entered = True
                    entry = (footprint_distance(st['robot'], shape, st['movers'][obstacle]),
                             st['v'])
            if st['zone'] == 3 and stop_t is None:
                stop_t = st['sim']
            if stop_t is not None and abs(st['v']) < 0.01 and st['sim'] - stop_t > 0.5:
                spin_sim(0.5, drv.tick)
                break
            if drv.along() > 7.0:
                break
        t1 = time.monotonic()
        rows = window(t0, t1)

        def pose_of(r):
            if obstacle == 'forklift':
                return opose
            movers = st['movers'] if r is None else r[10]
            return movers.get(obstacle, (math.nan, math.nan, 0.0))

        rows = [r for r in rows if obstacle == 'forklift' or obstacle in r[10]]
        dists = [footprint_distance((r[2], r[3], r[4]), shape, pose_of(r)) for r in rows]
        moving_d = [d for d, r in zip(dists, rows) if abs(r[5]) > 0.02]
        final = footprint_distance(st['robot'], shape, pose_of(None))
        v_max = max((r[5] for r in rows), default=0.0)
        rec = {'kind': kind, 'obstacle': obstacle, 'cmd_speed': speed, 'lateral': lateral,
               'stop_zone3': stop_t is not None, 'final_distance_m': round(final, 3),
               'min_distance_moving_m': round(min(moving_d), 3) if moving_d else None,
               'max_gt_speed': round(v_max, 3), 'estop_seen': any(r[9] for r in rows),
               'stop_causes': st['diag'].get('stop_causes', '-'),
               'tolerance_m': round(braking_tolerance(0.2), 3)}
        if moving is not None:
            rec['mover_speed'] = moving['speed']
            rec['entered'] = entered
            if entry is not None:
                # 경로에 들어온 순간의 지면 진실 간격과 로봇 속도 → 물리적 하한 간격 - d_brake(v)
                rec['entry_gap_m'] = round(entry[0], 3)
                rec['entry_robot_speed'] = round(entry[1], 3)
        if obstacle != 'forklift':
            mover_vel(obstacle, 0.0, 0.0)
        publish_cmd(0.0)
        park_all()
        print(f'[{kind}] {rec}', flush=True)
        return rec

    if 'static' in scen:
        mark = rtf_mark()
        trials = []
        for _ in range(args.repeat):
            for speed, lat in ((0.5, 0.0), (1.0, 0.0), (1.0, 0.1), (1.0, -0.1), (1.5, 0.0),
                               (1.0, 0.15)):
                trials.append(obstacle_trial('static', 'obs_person', speed, lat))
            for speed, lat in ((0.5, 0.0), (1.0, 0.0), (1.0, 0.1), (1.0, -0.1), (1.5, 0.0)):
                trials.append(obstacle_trial('static', 'obs_box_low', speed, lat))
            for speed, lat in ((0.5, 0.0), (1.0, 0.0), (1.0, 0.1), (1.0, -0.1)):
                trials.append(obstacle_trial('static', 'forklift', speed, lat))
        results['static'] = {'trials': trials, 'rtf': rtf_since(mark), 'load': loadavg()}

    if 'moving' in scen:
        mark = rtf_mark()
        trials = []
        for _ in range(args.repeat):
            for v_o, frm, trig in ((0.5, 1.5, 4.5), (1.0, 2.5, 4.0), (1.0, -2.5, 4.0),
                                   (1.0, 2.5, 3.2), (0.5, -1.5, 4.5), (1.0, 1.5, 2.6)):
                trials.append(obstacle_trial('moving', 'obs_person', 1.0, 0.0,
                                             {'speed': v_o, 'from': frm, 'trigger': trig}))
            for v_o, frm, trig in ((0.5, 1.0, 3.5), (0.5, -1.0, 3.0), (0.3, 0.8, 3.5),
                                   (0.5, 1.0, 2.6)):
                trials.append(obstacle_trial('moving', 'obs_box_low', 1.0, 0.0,
                                             {'speed': v_o, 'from': frm, 'trigger': trig}))
        results['moving'] = {'trials': trials, 'rtf': rtf_since(mark), 'load': loadavg()}

    # ------------------------------------------------ (c) 도킹 예외
    if 'dock' in scen:
        mark = rtf_mark()
        runs = []

        def polygon():
            msg = PolygonStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            for x, y in ((DOCK['face_x'] - 0.25, DOCK['y'] - 0.40),
                         (DOCK['face_x'] + 0.30, DOCK['y'] - 0.40),
                         (DOCK['face_x'] + 0.30, DOCK['y'] + 0.40),
                         (DOCK['face_x'] - 0.25, DOCK['y'] + 0.40)):
                msg.polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))
            return msg

        for with_poly in (True, True, True, True, True, False, False):
            place_robot(DOCK['staging_x'], DOCK['y'], 0.0)
            goal_x = DOCK['face_x'] - (DOCK['standoff'] - 0.0)
            drv = Driver((DOCK['staging_x'], DOCK['y']), 0.0, 0.15)
            s0 = st['sim']
            last_poly = -1.0
            stops = 0
            last_zone = st['zone']
            hold_until = None
            while st['sim'] - s0 < 60.0:
                now = st['sim']
                if with_poly and (now - last_poly >= 0.1 or now < last_poly):
                    excl_pub.publish(polygon())   # docking_server_node 계약: ≥ 10 Hz (sim)
                    last_poly = now
                remaining = goal_x - st['robot'][0]
                if hold_until is None:
                    drv.tick(max(0.0, min(0.15, 0.8 * remaining)) if remaining > 0.003 else 0.0)
                    if remaining <= 0.003 and abs(st['v']) < 0.005:
                        hold_until = now + 10.0   # 도킹 완료 후 10 s 유지 (잡음 STOP 확인)
                else:
                    publish_cmd(0.0)
                    if now >= hold_until:
                        break
                spin_for(0.02)
                if st['zone'] == 3 and last_zone != 3:
                    stops += 1
                last_zone = st['zone']
            gap = DOCK['face_x'] - (st['robot'][0] + FOOT_L / 2)
            runs.append({'exclusion_polygon': with_poly, 'stops': stops,
                         'bumper_to_plate_m': round(gap, 3),
                         'reached_standoff': hold_until is not None,
                         'zone_final': st['zone'],
                         'exclusion_distance_diag': st['diag'].get('exclusion_distance_m')})
            print(f'[dock] {runs[-1]}', flush=True)
            publish_cmd(0.0)
            spin_for(0.5)
        results['dock'] = {'runs': runs, 'rtf': rtf_since(mark), 'load': loadavg()}

    # ------------------------------------------------ (d) E-stop
    def call_reset():
        if not reset_cli.wait_for_service(timeout_sec=2.0):
            return None
        fut = reset_cli.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        return bool(fut.result().success) if fut.done() and fut.result() else None

    if 'estop' in scen:
        mark = rtf_mark()
        runs = []
        for i in range(6):
            pub = estop_pub if i % 2 == 0 else estop_vol_pub
            place_robot(-6.0, LANE_Y, 0.0)
            drv = Driver((-6.0, LANE_Y), 0.0, 1.0)
            spin_sim(6.0, drv.tick, until=lambda: st['v'] >= 0.97)
            spin_sim(0.3, drv.tick)
            x_press, v_press, sim_press = st['robot'][0], st['v'], st['sim']
            n_before = len(cmd_log)
            t_press = time.monotonic()
            pub.publish(Bool(data=True))
            t_zero = None
            t_gt = None
            sim_gt = None
            x_gt = None
            s_press = st['sim']
            while st['sim'] - s_press < 4.0:
                drv.tick()
                spin_for(0.005)
                if t_zero is None and any(c[1] == 0.0 for c in cmd_log[n_before:]):
                    t_zero = next(c[0] for c in cmd_log[n_before:] if c[1] == 0.0)
                if t_gt is None and abs(st['v']) < 0.01:
                    t_gt, sim_gt, x_gt = time.monotonic(), st['sim'], st['robot'][0]
                if t_gt is not None:
                    break
            nonzero_after = sum(1 for c in cmd_log[n_before:] if t_zero and c[0] > t_zero and
                                c[1] != 0.0)
            rej = call_reset()                      # 입력이 true 인 동안의 reset → 거절
            pub.publish(Bool(data=False))
            spin_sim(1.0, drv.tick)
            still_zero = all(c[1] == 0.0 for c in cmd_log[-20:])
            estop_after_false = st['estop']
            ok = call_reset()
            spin_sim(1.0, drv.tick)
            runs.append({
                'publisher': 'transient_local' if pub is estop_pub else 'volatile',
                'speed_at_press': round(v_press, 3),
                'press_to_cmd_zero_ms': round((t_zero - t_press) * 1e3, 2) if t_zero else None,
                'nonzero_cmds_after_zero': nonzero_after,
                'gt_stop_time_sim_s': round(sim_gt - sim_press, 3) if sim_gt else None,
                'gt_stop_distance_m': round(x_gt - x_press, 3) if t_gt else None,
                'gt_mean_decel': round(v_press / (sim_gt - sim_press), 3) if sim_gt else None,
                'reset_while_pressed_success': rej, 'still_stopped_after_false': still_zero,
                'estop_active_after_false': estop_after_false, 'reset_success': ok,
                'resumed_speed': round(st['v'], 3), 'estop_active_final': st['estop']})
            print(f'[estop] {runs[-1]}', flush=True)
            publish_cmd(0.0)
            spin_for(0.5)
        results['estop'] = {'runs': runs, 'rtf': rtf_since(mark), 'load': loadavg()}

    # ------------------------------------------------ (e) 센서 끊김 디바운스
    if 'sensor' in scen:
        mark = rtf_mark()
        place_robot(-6.0, LANE_Y, 0.0)
        out = {'single_gaps': [], 'dropouts': [], 'imu_dropouts': []}

        def hold(sec):
            spin_sim(sec, lambda: publish_cmd(0.4))   # sim 시각 — safety_node 도 sim 시각으로 감시

        hold(2.0)
        for gap in (0.06, 0.08, 0.10, 0.15):
            for _ in range(5):
                n0 = len(cmd_log)
                r0 = len(relayed['wheel'])
                st_hist['estop_seen'] = False
                st['drop']['wheel'] = True
                hold(gap)
                st['drop']['wheel'] = False
                hold(0.6)
                seg = cmd_log[n0:]
                rel = relayed['wheel'][max(r0 - 1, 0):]
                real_gap = max((b - a for a, b in zip(rel[:-1], rel[1:])), default=math.nan)
                out['single_gaps'].append({
                    'gap_s': gap, 'relayed_gap_sim_ms': round(1e3 * real_gap, 1),
                    'zero_cmds': sum(1 for c in seg if c[1] == 0.0),
                    'min_out': round(min((c[1] for c in seg), default=math.nan), 3),
                    'estop_active_seen': bool(st_hist['estop_seen'])})
        for kind, key in (('wheel', 'dropouts'), ('imu', 'imu_dropouts')):
            for _ in range(5):
                hold(1.0)
                st_hist['estop_seen'] = False
                n0 = len(sim_cmd)
                s_drop = st['sim']
                st['drop'][kind] = True
                hold(1.0)
                st['drop'][kind] = False
                s_back = st['sim']
                hold(1.0)
                seg = sim_cmd[n0:]
                pred = (lambda v: v == 0.0) if kind == 'wheel' else (lambda v: v <= 0.2 + 1e-6)
                first = next((c[0] for c in seg if c[0] >= s_drop and pred(c[1])), None)
                back = next((c[0] for c in seg if c[0] >= s_back and c[1] > 0.39), None)
                out[key].append({
                    'reaction_sim_ms': round((first - s_drop) * 1e3, 1) if first else None,
                    'estop_active_during': bool(st_hist['estop_seen']),
                    'recovery_sim_ms': round((back - s_back) * 1e3, 1) if back else None})
        out['rtf'] = rtf_since(mark)
        out['load'] = loadavg()
        results['sensor'] = out
        publish_cmd(0.0)

    def gap_stats(table):
        return {k: {'n': len(v), 'p50': round(1e3 * percentile(v, 50), 2) if v else None,
                    'p99': round(1e3 * percentile(v, 99), 2) if v else None,
                    'p999': round(1e3 * percentile(v, 99.9), 2) if v else None,
                    'max': round(1e3 * max(v), 2) if v else None}
                for k, v in table.items()}

    results['topic_gaps_ms'] = gap_stats(gaps)
    results['topic_gaps_sim_ms'] = gap_stats(sim_gaps)
    results['conditions'] = {'load_end': loadavg(), 'sim_end_s': round(st['sim'], 1)}
    node.destroy_node()
    rclpy.shutdown()
    text = json.dumps(results, indent=1)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
