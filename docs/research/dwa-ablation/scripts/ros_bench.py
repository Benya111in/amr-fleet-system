#!/usr/bin/env python3
"""
ROS 시험대 구동기: 같은 controller_server 에 올린 여러 제어기 팔로 같은 경로들을 FollowPath 로 돈다.

    ros2 launch amr_navigation kinematic_sim.launch.py robot_name:=amr_61 \
        map:=/runs/paths/composite.yaml params_file:=<ros_params_file 산출> use_ttc_bt:=false
    python3 ros_bench.py --paths /runs/paths/tuning.json --arms DWB_D00,dwa_full \
        --out /runs/raw/<tag>.jsonl --tag <tag> --ns /amr_61 --ros-args -r __ns:=/amr_61

시험대 (path_tracking.md §5.1 과 같다): 실제 Nav2 controller_server(팔마다 플러그인) →
velocity_profiler_node(50 Hz, 저크 2) → kinematic_sim(서보 지연 0.04 + 1차 0.08 s, 100 Hz,
lidar 720 빔 σ 0.03 → 지역 코스트맵 0.025 m). 모든 팔이 같은 하류 사슬·같은 목표 판정
(xy 0.10 m, yaw 0.05 rad)·같은 진행 검사(0.3 m / 20 s)를 지난다.
경로마다 팔 순서는 경로 번호만큼 돌려 순서 효과가 한쪽에 몰리지 않게 한다.
실행마다 JSON 한 줄 (chain_sim 과 같은 지표 이름 + ROS 전용: 부하, CPU, 벽시계).
CPU: controller_server 프로세스의 누적 CPU 시간(psutil) 증가분에서 정지 상태 10 s 로 잰 유휴
비율(코스트맵 갱신 등)을 빼고 명령 수로 나눈 근사 [ms/주기].
"""
import argparse
import json
import math
import os
import sys
import time

from action_msgs.msg import GoalStatus
from amr_navigation import path_metrics
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
import numpy as np
import psutil
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

LIMITS = path_metrics.ProfileLimits(desired_speed=1.0)


def make_pose(x, y, yaw, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(0.5 * yaw)
    p.pose.orientation.w = math.cos(0.5 * yaw)
    return p


def curve_labels(xy, threshold=0.1, margin=0.5):
    k = np.abs(path_metrics.discrete_curvature(xy, 0.25))
    s = path_metrics.cumulative_length(xy)
    curve = k > threshold
    out = np.zeros(len(xy), dtype=int)
    for i in range(len(xy)):
        near = np.abs(s - s[i]) <= margin
        out[i] = 1 if np.any(curve & near) else 0
    return out


def windowed_cte(xy, samples):
    """참값 표본 → (|CTE|, 정점 인덱스). 직전 투영 근처 창에서만 찾는다 (chain_sim 과 같다)."""
    hint = 0
    out = []
    n = len(xy)
    for x, y in samples:
        b = max(0, hint - 10)
        e = min(n - 1, hint + 80)
        a = xy[b:e]
        d = xy[b + 1:e + 1] - a
        ll = np.maximum((d ** 2).sum(axis=1), 1e-12)
        t = np.clip(((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / ll, 0.0, 1.0)
        px = a[:, 0] + t * d[:, 0]
        py = a[:, 1] + t * d[:, 1]
        dist = np.hypot(x - px, y - py)
        j = int(np.argmin(dist))
        hint = b + j
        seg = hint + (1 if t[j] > 0.5 else 0)
        out.append((float(dist[j]), min(seg, n - 1)))
    return out


class Bench(Node):
    """참값·명령·프로파일러·통계 기록과 FollowPath 호출."""

    def __init__(self):
        super().__init__('dwa_ablation_bench')
        self.gt = []
        self.prof = []
        self.cmd = []
        self.stats = []
        self.clear = []
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 100)
        self.create_subscription(Float64MultiArray, 'velocity_profiler/state', self.on_prof, 100)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_cmd, 100)
        self.create_subscription(Float64, 'sim/footprint_clearance', self.on_clear, 100)
        for topic in ('dwa/stats', 'pure_pursuit/stats'):
            self.create_subscription(Float64MultiArray, topic, self.on_stats, 100)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.follow = ActionClient(self, FollowPath, 'follow_path')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        st = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.gt.append((st, m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                        m.twist.twist.linear.x, m.twist.twist.angular.z))

    def on_prof(self, m):
        self.prof.append((self.now(), *m.data))

    def on_cmd(self, m):
        self.cmd.append((self.now(), m.linear.x, m.angular.z))

    def on_stats(self, m):
        if m.data:
            self.stats.append((self.now(), m.data[0]))

    def on_clear(self, m):
        self.clear.append((self.now(), m.data))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def teleport(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose = make_pose(x, y, yaw).pose
        for _ in range(3):
            self.pub_pose.publish(msg)
            self.spin_for(0.2)
        self.spin_for(2.0)


def find_controller(ns):
    """같은 컨테이너의 controller_server 중 이 네임스페이스 것 (psutil.Process)."""
    for p in psutil.process_iter(['cmdline']):
        cmd = ' '.join(p.info.get('cmdline') or [])
        if 'controller_server' in cmd and f'__ns:={ns}' in cmd:
            return p
    return None


def cpu_seconds(proc):
    if proc is None:
        return float('nan')
    try:
        t = proc.cpu_times()
        return t.user + t.system
    except psutil.Error:
        return float('nan')


def p99(a):
    return float(np.percentile(a, 99)) if len(a) else -1.0


def amax(a):
    return float(np.max(a)) if len(a) else -1.0


def run_one(ev, entry, arm, args, proc, idle_cpu_rate):
    pts = entry['points']
    xy = np.array([[p[0], p[1]] for p in pts])
    labels = curve_labels(xy)
    x0, y0, th0 = pts[0]
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = ev.get_clock().now().to_msg()
    path.poses = [make_pose(x, y, th) for x, y, th in pts]
    for p in path.poses:
        p.header.stamp = path.header.stamp
    ev.teleport(x0, y0, th0)
    for buf in (ev.gt, ev.prof, ev.cmd, ev.stats, ev.clear):
        buf.clear()
    t_pred = path_metrics.predict_travel_time(xy, LIMITS)
    timeout = args.timeout_scale * max(60.0, 3.0 * t_pred + 30.0)
    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = arm
    load0 = list(os.getloadavg())
    cpu0 = cpu_seconds(proc)
    wall0 = time.time()
    fut = ev.follow.send_goal_async(goal)
    rclpy.spin_until_future_complete(ev, fut, timeout_sec=10.0)
    handle = fut.result()
    status = GoalStatus.STATUS_ABORTED
    t_acc = ev.now()
    fail = 'none'
    if handle is None or not handle.accepted:
        fail = 'rejected'
    else:
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(ev, rf, timeout_sec=timeout)
        if rf.result() is None:
            handle.cancel_goal_async()
            ev.spin_for(1.0)
            status = GoalStatus.STATUS_UNKNOWN
            fail = 'timeout'
        else:
            status = rf.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                fail = 'aborted'
    t_res = ev.now()
    cpu1 = cpu_seconds(proc)
    n_cmd = sum(1 for c in ev.cmd if t_acc <= c[0] <= t_res)
    # 정지까지 기다린다 (최대 6 s): 참값 |v|,|ω| < 1e-3 가 0.5 s 이어지면 끝
    t_settle = -1.0
    end = time.monotonic() + 6.0
    while time.monotonic() < end:
        rclpy.spin_once(ev, timeout_sec=0.02)
        if ev.gt and abs(ev.gt[-1][4]) < 1e-3 and abs(ev.gt[-1][5]) < 1e-3:
            if t_settle < 0.0:
                t_settle = ev.now()
            elif ev.now() - t_settle > 0.5:
                break
        else:
            t_settle = -1.0
    reached = status == GoalStatus.STATUS_SUCCEEDED
    gt = np.array([g for g in ev.gt if g[0] >= t_acc - 0.05]) if ev.gt else np.zeros((0, 6))
    run_gt = gt[gt[:, 0] <= t_res] if len(gt) else gt
    cte = windowed_cte(xy, run_gt[:, 1:3]) if len(run_gt) else []
    seg = {0: [], 1: []}
    for e, i in cte:
        seg[int(labels[i])].append(e)
    gx, gy, gth = pts[-1]
    fin = gt[-1] if len(gt) else (0.0, x0, y0, th0, 0.0, 0.0)
    c, s = math.cos(gth), math.sin(gth)
    v90 = [g[0] for g in run_gt if g[4] >= 0.9]
    cmd = np.array([c_[1] for c_ in ev.cmd if t_acc <= c_[0] <= t_res])
    acc = np.diff(cmd) / 0.05 if len(cmd) > 1 else np.zeros(0)
    jerk = np.diff(acc) / 0.05 if len(acc) > 1 else np.zeros(0)
    prof = np.array([p for p in ev.prof if t_acc <= p[0] <= t_res + 3.0])
    pj = np.zeros(0)
    if len(prof) and prof.shape[1] > 12:
        rs = prof[:, 11] > 0.5
        keep = ~(rs | np.concatenate([[False], rs[:-1]]))
        pj = np.abs(prof[keep, 12])
    cyc = np.array([st[1] for st in ev.stats if t_acc <= st[0] <= t_res])
    clr = [cl[1] for cl in ev.clear]
    min_clear = float(min(clr)) if clr else -1.0
    collision = bool(clr) and min_clear <= 0.0
    dur = max(t_res - t_acc, 1e-6)
    cpu_cycle_ms = -1.0
    if n_cmd > 0 and math.isfinite(cpu0) and math.isfinite(cpu1):
        cpu_cycle_ms = 1000.0 * max(0.0, (cpu1 - cpu0) - idle_cpu_rate * dur) / n_cmd
    allv = seg[0] + seg[1]
    return {
        'testbed': 'ros', 'tag': args.tag, 'worker': args.worker, 'scene': entry['id'],
        'family': entry['family'], 'arm': arm, 'ctrl': arm, 'plant': 'chain',
        'seed': args.noise_seed, 'odom_noise_v': args.odom_noise_v,
        'odom_noise_w': args.odom_noise_w,
        'success': int(reached and not collision), 'reached': int(reached), 'fail': fail,
        'collision': int(collision), 'status': int(status),
        't_reach': (t_res - t_acc) if reached else -1.0, 't_end': t_res - t_acc,
        'length': float(path_metrics.path_length(xy)), 't_pred': float(t_pred),
        'cte_straight_mean': float(np.mean(seg[0])) if seg[0] else -1.0,
        'cte_straight_max': amax(seg[0]),
        'cte_curve_mean': float(np.mean(seg[1])) if seg[1] else -1.0,
        'cte_curve_max': amax(seg[1]),
        'cte_mean': float(np.mean(allv)) if allv else -1.0, 'cte_max': amax(allv),
        'stop_along': float((fin[1] - gx) * c + (fin[2] - gy) * s),
        'stop_lat': float(-(fin[1] - gx) * s + (fin[2] - gy) * c),
        'stop_err': float(math.hypot(fin[1] - gx, fin[2] - gy)),
        'settle': (t_settle - t_res) if (reached and t_settle > 0) else -1.0,
        'min_clear': min_clear, 't_v90': (v90[0] - t_acc) if v90 else -1.0,
        'cmd_acc_p99': p99(np.abs(acc)), 'cmd_acc_max': amax(np.abs(acc)),
        'cmd_jerk_p99': p99(np.abs(jerk)), 'cmd_jerk_max': amax(np.abs(jerk)),
        'prof_jerk_p99': p99(pj), 'prof_jerk_max': amax(pj),
        'cycle_us_mean': 1000.0 * float(np.mean(cyc)) if len(cyc) else -1.0,
        'cycle_us_p95': 1000.0 * float(np.percentile(cyc, 95)) if len(cyc) else -1.0,
        'cycle_us_max': 1000.0 * amax(cyc) if len(cyc) else -1.0,
        'cpu_ms_per_cycle': cpu_cycle_ms, 'n_cycles': n_cmd, 'cmd_rate_hz': n_cmd / dur,
        'load_start': load0, 'load_end': list(os.getloadavg()),
        'wall_start': wall0, 'wall_end': time.time(), 'n_gt': int(len(run_gt)),
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--paths', required=True, help='쉼표로 여러 JSON')
    ap.add_argument('--ids', default='', help='쉼표 목록 (비면 전부)')
    ap.add_argument('--arms', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--worker', default='w0')
    ap.add_argument('--ns', default='/amr_01')
    ap.add_argument('--timeout-scale', type=float, default=1.0)
    ap.add_argument('--noise-seed', type=int, default=0)
    ap.add_argument('--odom-noise-v', type=float, default=0.0)
    ap.add_argument('--odom-noise-w', type=float, default=0.0)
    ap.add_argument('--skip-done', action='store_true',
                    help='out 에 이미 있는 (scene, arm, seed) 는 건너뜀 (재개)')
    args, ros_args = ap.parse_known_args(argv)
    entries = []
    for f in args.paths.split(','):
        with open(f, encoding='utf-8') as fh:
            entries += json.load(fh)
    ids = [i for i in args.ids.split(',') if i]
    if ids:
        entries = [e for e in entries if e['id'] in ids]
    arms = [a for a in args.arms.split(',') if a]
    done = set()
    if args.skip_done and os.path.exists(args.out):
        with open(args.out, encoding='utf-8') as fh:
            for line in fh:
                d = json.loads(line)
                done.add((d['scene'], d['arm'], d.get('seed', 0)))
    rclpy.init(args=ros_args)
    ev = Bench()
    if not ev.follow.wait_for_server(timeout_sec=180.0):
        print('follow_path 서버 없음', file=sys.stderr)
        return 2
    ev.spin_for(3.0)
    proc = find_controller(args.ns)
    # 유휴 CPU 기준: 정지 상태 10 s (지역 코스트맵 갱신 등 제어와 무관한 몫)
    c0 = cpu_seconds(proc)
    t0 = time.monotonic()
    ev.spin_for(10.0)
    idle = (cpu_seconds(proc) - c0) / (time.monotonic() - t0)
    if not math.isfinite(idle):
        idle = 0.0
    with open(args.out, 'a', encoding='utf-8') as out:
        for k, e in enumerate(entries):
            order = arms[k % len(arms):] + arms[:k % len(arms)]
            for arm in order:
                if (e['id'], arm, args.noise_seed) in done:
                    continue
                r = run_one(ev, e, arm, args, proc, idle)
                r['idle_cpu_rate'] = idle
                out.write(json.dumps(r) + '\n')
                out.flush()
                ev.get_logger().info(
                    f"{e['id']} {arm}: ok={r['success']} fail={r['fail']} T={r['t_end']:.1f} "
                    f"cte={100 * r['cte_mean']:.1f}cm stop={100 * r['stop_along']:.1f}cm")
    ev.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
