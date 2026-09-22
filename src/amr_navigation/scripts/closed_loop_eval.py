#!/usr/bin/env python3
"""
폐루프 경로 추종·주행 시간 평가 (명세 4.4 예측 시간 오차 15 %, 4.5 CTE 직선 5 cm / 곡선 10 cm, 저크 한계).

    ros2 run amr_navigation closed_loop_eval.py --mode follow --controllers DWA,PurePursuit \
        --out <dir>
    ros2 run amr_navigation closed_loop_eval.py --mode navigate --controllers DWA --out <dir>

follow  : 고정 기준 경로(직선 20 m, U 턴 R 2 m)를 controller_server follow_path 로 직접 추종.
          참값(ground_truth/odom) CTE 를 직선/곡선 구간으로 나눠(곡률 > 0.1 1/m, 앞뒤 0.5 m 천이대 포함)
          평균·최대를 내고, 저크(velocity_profiler/state 기준 저크·참값 속도 2차 차분), 주행 시간 예측 오차,
          풋프린트 최소 여유(sim/footprint_clearance), 제어기 주기 계산 시간(dwa/stats·pure_pursuit/stats
          의 cycle_ms: 20 Hz 예산 50 ms 대비)을 기록한다.
navigate: bt_navigator navigate_to_pose 로 창고 목표 12 개(0.60 m 좁은 통로 포함)를 순회하며 첫 계획 경로의
          예측 주행 시간(path_metrics.predict_travel_time, 단순 L/v 도 함께)과 실제 시간을 비교한다.
출력: <out>/<run>.csv (명세 4.10 표 형식
      [timestamp, planned_x, planned_y, actual_x, actual_y, cte, segment]),
      <out>/summary.json, <out>/summary.md
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional

from action_msgs.msg import GoalStatus
from amr_navigation import path_metrics
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, Float64MultiArray, String

LIMITS = path_metrics.ProfileLimits(desired_speed=1.0)


def make_pose(x: float, y: float, yaw: float, frame: str = 'map') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(0.5 * yaw)
    p.pose.orientation.w = math.cos(0.5 * yaw)
    return p


def straight_path(x0: float, y0: float, yaw: float, length: float, step: float = 0.05):
    n = int(round(length / step))
    return [(x0 + i * step * math.cos(yaw), y0 + i * step * math.sin(yaw), yaw)
            for i in range(n + 1)]


def append_straight(pts, length, step=0.05):
    x, y, th = pts[-1]
    for i in range(1, int(round(length / step)) + 1):
        pts.append((x + i * step * math.cos(th), y + i * step * math.sin(th), th))


def append_arc(pts, radius, angle, step=0.05):
    x, y, th = pts[-1]
    sgn = 1.0 if angle >= 0 else -1.0
    n = int(round(abs(angle) * radius / step))
    for i in range(1, n + 1):
        a = th + sgn * i * step / radius
        pts.append((x + sgn * radius * (math.sin(a) - math.sin(th)),
                    y - sgn * radius * (math.cos(a) - math.cos(th)), a))


def reference_paths(layout: str = 'synthetic') -> Dict[str, list]:
    """
    기준 경로 4 종 (직선 / U 턴 R 2 m / S 자 R 3 m 물결 / 좁은 통로 관통).

    narrow: 월드·합성 지도 공통의 0.60 m 좁은 통로(중심 (0, −10), 길이 4 m)를 남쪽으로 관통하는 직선
            (명세 4.4 "로봇 폭 +20 cm 통로"). 전역 계획기는 우회가 6.5 m 이내면 넓은 통로를 택하므로
            (costmap.md §4.1) 통로 주행 자체는 경로를 고정해 따로 시험한다.

    synthetic: 운동학 시뮬레이터용 (합성 창고 지도의 빈 구역).
    gazebo   : Gazebo 창고 월드의 남측 공터 (y −13.5…−17.5) — 월드의 동적 actor(무작위 보행 격자 y ≥ −6,
               횡단 y = −7, 원 순환 (−24, 0) r 3.5)와 지게차(y = 15)의 경로를 피한 곳. 직선은 14 m.
    """
    if layout == 'gazebo':
        straight = straight_path(-16.0, -15.0, 0.0, 14.0)
        uturn = [(4.0, -13.5, 0.0)]            # 동쪽 3 m → R 2 우 180° (남쪽으로) → 서쪽 3 m
        s0 = (12.0, -15.0, 0.0)
        narrow = straight_path(0.0, -7.9, -math.pi / 2, 6.1)   # 횡단 actor(y = −7) 뒤에서 출발
    else:
        straight = straight_path(-10.0, 0.0, 0.0, 20.0)          # B/C 열 사이 5 m 통로
        uturn = [(-26.0, -4.0, math.pi / 2)]                      # 서측 공터: 직선 3 → R 2 우 180° → 직선 3
        s0 = (-18.0, -6.5, 0.0)            # 남측 통로
        narrow = straight_path(0.0, -6.0, -math.pi / 2, 8.5)
    append_straight(uturn, 3.0)
    append_arc(uturn, 2.0, -math.pi)
    append_straight(uturn, 3.0)
    scurve = [s0]                          # R 3 좌 30° → 우 60° → 좌 30° 물결 2 회 (곡률 부호 반전)
    for _ in range(2):
        append_arc(scurve, 3.0, math.pi / 6)
        append_arc(scurve, 3.0, -math.pi / 3)
        append_arc(scurve, 3.0, math.pi / 6)
    append_straight(scurve, 1.0)
    return {'straight': straight, 'uturn': uturn, 'scurve': scurve, 'narrow': narrow}


NAV_GOALS = [(-10.0, 0.0, 0.0), (10.0, 0.0, 0.0), (22.0, 6.0, math.pi / 2),
             (22.0, -8.0, -math.pi / 2), (5.0, -6.0, math.pi), (-15.0, -6.0, math.pi),
             (-24.0, 0.0, math.pi / 2), (-15.0, 6.0, 0.0), (15.0, 6.0, 0.0),
             (0.0, -6.5, -math.pi / 2), (0.0, -14.0, -math.pi / 2), (0.0, 0.0, math.pi)]


def segment_labels(xy: np.ndarray, threshold: float = 0.1, margin: float = 0.5) -> List[str]:
    k = np.abs(path_metrics.discrete_curvature(xy, 0.25))
    s = path_metrics.cumulative_length(xy)
    curve = k > threshold
    labels = []
    for i in range(len(xy)):
        near = np.abs(s - s[i]) <= margin
        labels.append('curve' if np.any(curve & near) else 'straight')
    return labels


class Evaluator(Node):
    """평가 노드: 참값·프로파일러 상태 기록, 액션 호출."""

    def __init__(self):
        super().__init__('closed_loop_eval')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gt: List[tuple] = []
        self.prof: List[tuple] = []
        self.clearance: List[float] = []
        self.plans: List[tuple] = []
        self.cycle: List[tuple] = []
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 50)
        self.create_subscription(Float64MultiArray, 'velocity_profiler/state', self.on_prof, 50)
        self.create_subscription(Float64, 'sim/footprint_clearance', self.on_clear, 50)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        for topic in ('dwa/stats', 'pure_pursuit/stats'):
            self.create_subscription(Float64MultiArray, topic, self.on_stats, 50)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.pub_ref = self.create_publisher(Path, 'eval/reference_path', latched)
        self.pub_ctrl = self.create_publisher(String, 'controller_selector', latched)
        self.follow = ActionClient(self, FollowPath, 'follow_path')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m: Odometry):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        # [수신 시각, x, y, yaw, v, ω, 발행 시각] — 저크는 수신 지터가 없는 발행 시각으로 미분한다
        self.gt.append((self.now(), m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                        m.twist.twist.linear.x, m.twist.twist.angular.z, stamp))

    def on_prof(self, m: Float64MultiArray):
        self.prof.append((self.now(), *m.data))

    def on_stats(self, m: Float64MultiArray):
        if m.data:
            self.cycle.append((self.now(), *m.data))

    def on_clear(self, m: Float64):
        self.clearance.append(m.data)

    def on_plan(self, m: Path):
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])
        self.plans.append((self.now(), xy))

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def teleport(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose = make_pose(x, y, yaw).pose
        for _ in range(3):
            self.pub_pose.publish(msg)
            self.spin_for(0.2)
        self.spin_for(2.0)   # 코스트맵 갱신·정지 대기

    def run_action(self, client: ActionClient, goal, timeout: float):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return GoalStatus.STATUS_ABORTED, self.now()
        t_acc = self.now()
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        if rf.result() is None:
            handle.cancel_goal_async()
            self.spin_for(1.0)
            return GoalStatus.STATUS_UNKNOWN, t_acc
        return rf.result().status, t_acc


def window(samples: List[tuple], t0: float, t1: float) -> np.ndarray:
    rows = [s for s in samples if t0 <= s[0] <= t1]
    return np.array(rows) if rows else np.zeros((0, 11))


def follow_run(ev: Evaluator, name: str, pts: list, controller: str, out: str) -> Dict:
    x0, y0, th0 = pts[0]
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = ev.get_clock().now().to_msg()
    path.poses = [make_pose(x, y, th) for x, y, th in pts]
    for p in path.poses:
        p.header.stamp = path.header.stamp
    # 기준 경로를 순간 이동보다 먼저 발행: 외부 CTE 로거(amr_evaluation cte_logger)가 이동 직후 표본을
    # 이전 경로와 비교하지 않게 한다
    ev.pub_ref.publish(path)
    ev.teleport(x0, y0, th0)
    ev.clearance.clear()
    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = controller
    t_send = ev.now()
    status, t_acc = ev.run_action(ev.follow, goal, timeout=180.0)
    t_end = ev.now()
    xy = np.array([[x, y] for x, y, _ in pts])
    labels = segment_labels(xy)
    gt = window(ev.gt, t_acc, t_end)
    rows = []
    cte = {'straight': [], 'curve': []}
    for t, x, y, *_ in gt:
        e = path_metrics.cross_track_error(xy, x, y)
        d2 = (xy[:, 0] - x) ** 2 + (xy[:, 1] - y) ** 2
        i = int(np.argmin(d2))
        seg = labels[i]
        cte[seg].append(abs(e))
        rows.append([t, xy[i, 0], xy[i, 1], x, y, e, seg])
    with open(os.path.join(out, f'{controller}_{name}.csv'), 'w', newline='',
              encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['timestamp', 'planned_x', 'planned_y', 'actual_x', 'actual_y', 'cte',
                    'segment'])
        w.writerows(rows)
    # 정지 후 최종 자세: 목표 도달 판정(xy 허용 오차) 뒤 남은 제동 거리 = 목표 통과량
    ev.spin_for(3.0)
    gx, gy, gth = pts[-1]
    fin = ev.gt[-1] if ev.gt else (0.0, gx, gy, gth)
    overshoot = (fin[1] - gx) * math.cos(gth) + (fin[2] - gy) * math.sin(gth)
    final_err = math.hypot(fin[1] - gx, fin[2] - gy)
    prof = window(ev.prof, t_acc, t_end + 3.0)
    ctrl = [c for c in ev.cycle if t_acc <= c[0] <= t_end]
    trace = os.path.join(out, f'{controller}_{name}_trace.csv')
    with open(trace, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['source', 't', 'values...'])
        w.writerows([['ctrl', *c] for c in ctrl])
        w.writerows([['prof', *p] for p in prof])
        w.writerows([['gt', *g] for g in window(ev.gt, t_acc, t_end + 3.0)])
    # 저크: 프로파일 기준(PID 전) / 최종 명령(PID 후, 노드 내부 dt) / 참값 속도(발행 시각, 0.2 s 평활)
    jerk_ref = float(np.max(np.abs(prof[:, 10]))) if len(prof) else float('nan')
    jerk_out = (np.abs(prof[:, 12]) if len(prof) and prof.shape[1] > 12 else np.zeros(0))
    jerk_gt = (path_metrics.jerk_from_velocity(gt[:, 6], gt[:, 4], smooth=10)
               if len(gt) > 20 else np.zeros(0))
    cyc = np.array([c[1] for c in ctrl])
    t_act = t_end - t_acc
    t_pred = path_metrics.predict_travel_time(xy, LIMITS)
    t_naive = path_metrics.path_length(xy) / LIMITS.desired_speed
    res = {
        'controller': controller, 'path': name, 'status': int(status),
        'succeeded': status == GoalStatus.STATUS_SUCCEEDED,
        'length_m': path_metrics.path_length(xy),
        'cte_straight_mean': float(np.mean(cte['straight'])) if cte['straight'] else None,
        'cte_straight_max': float(np.max(cte['straight'])) if cte['straight'] else None,
        'cte_curve_mean': float(np.mean(cte['curve'])) if cte['curve'] else None,
        'cte_curve_max': float(np.max(cte['curve'])) if cte['curve'] else None,
        'jerk_ref_max': jerk_ref,
        'jerk_out_max': float(np.max(jerk_out)) if len(jerk_out) else None,
        'jerk_out_p99': float(np.percentile(jerk_out, 99)) if len(jerk_out) else None,
        'jerk_gt_p99': float(np.percentile(np.abs(jerk_gt), 99)) if len(jerk_gt) else None,
        'goal_overshoot_m': overshoot, 'final_pos_err_m': final_err,
        't_actual': t_act, 't_pred': t_pred, 't_naive': t_naive,
        't_pred_err_pct': 100.0 * abs(t_act - t_pred) / max(t_act, 1e-6),
        't_naive_err_pct': 100.0 * abs(t_act - t_naive) / max(t_act, 1e-6),
        'min_footprint_clearance': float(min(ev.clearance)) if ev.clearance else None,
        'accept_latency_s': t_acc - t_send,
        'cycle_ms_mean': float(np.mean(cyc)) if len(cyc) else None,
        'cycle_ms_p95': float(np.percentile(cyc, 95)) if len(cyc) else None,
        'cycle_ms_max': float(np.max(cyc)) if len(cyc) else None,
    }
    ev.get_logger().info(json.dumps(res))
    return res


def plan_after(ev: Evaluator, t0: float, timeout: float = 5.0) -> Optional[np.ndarray]:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        for t, xy in ev.plans:
            if t >= t0 and len(xy) >= 2:
                return xy
        rclpy.spin_once(ev, timeout_sec=0.05)
    return None


def navigate_runs(ev: Evaluator, controller: str, out: str) -> List[Dict]:
    ev.pub_ctrl.publish(String(data=controller))
    results = []
    sx, sy, syaw = 0.0, 0.0, 0.0
    ev.teleport(sx, sy, syaw)
    for gx, gy, gyaw in NAV_GOALS:
        ev.clearance.clear()
        ev.plans.clear()
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(gx, gy, gyaw)
        t_send = ev.now()
        fut = ev.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(ev, fut, timeout_sec=10.0)
        handle = fut.result()
        xy = plan_after(ev, t_send)
        status = GoalStatus.STATUS_ABORTED
        t_end = ev.now()
        if handle is not None and handle.accepted:
            rf = handle.get_result_async()
            rclpy.spin_until_future_complete(ev, rf, timeout_sec=240.0)
            t_end = ev.now()
            status = rf.result().status if rf.result() is not None else GoalStatus.STATUS_UNKNOWN
        t_act = t_end - t_send
        r = {'controller': controller, 'goal': [gx, gy, gyaw], 'status': int(status),
             'succeeded': status == GoalStatus.STATUS_SUCCEEDED, 't_actual': t_act,
             'min_footprint_clearance': float(min(ev.clearance)) if ev.clearance else None}
        if xy is not None:
            head0 = math.atan2(xy[min(5, len(xy) - 1), 1] - xy[0, 1],
                               xy[min(5, len(xy) - 1), 0] - xy[0, 0])
            headn = math.atan2(xy[-1, 1] - xy[max(0, len(xy) - 6), 1],
                               xy[-1, 0] - xy[max(0, len(xy) - 6), 0])
            e0 = (head0 - syaw + math.pi) % (2 * math.pi) - math.pi
            en = (gyaw - headn + math.pi) % (2 * math.pi) - math.pi
            t_pred = path_metrics.predict_travel_time(xy, LIMITS, 0.0, e0, en)
            t_naive = path_metrics.path_length(xy) / LIMITS.desired_speed
            r.update({'length_m': path_metrics.path_length(xy), 't_pred': t_pred,
                      't_naive': t_naive,
                      't_pred_err_pct': 100.0 * abs(t_act - t_pred) / max(t_act, 1e-6),
                      't_naive_err_pct': 100.0 * abs(t_act - t_naive) / max(t_act, 1e-6)})
        ev.get_logger().info(json.dumps(r))
        results.append(r)
        if r['succeeded']:
            sx, sy, syaw = gx, gy, gyaw
        else:
            ev.teleport(gx, gy, gyaw)   # 실패 시 다음 시험을 위해 목표로 옮긴다
            sx, sy, syaw = gx, gy, gyaw
    return results


def to_markdown(results: List[Dict]) -> str:
    lines = []
    fr = [r for r in results if 'path' in r]
    if fr:
        lines += ['| controller | path | ok | CTE straight mean / max [cm] '
                  '| CTE curve mean / max [cm] '
                  '| jerk ref max / out p99 / GT p99 [m/s³] | T act / pred / naive [s] '
                  '| pred err [%] | overshoot / final err [cm] | min clearance [m] '
                  '| cycle mean / p95 / max [ms] |',
                  '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']

        def cm(v):
            return '-' if v is None else f'{100 * v:.1f}'

        def ms(v):
            return '-' if v is None else f'{v:.2f}'
        for r in fr:
            lines.append(
                f"| {r['controller']} | {r['path']} | {int(r['succeeded'])} "
                f"| {cm(r['cte_straight_mean'])} / {cm(r['cte_straight_max'])} "
                f"| {cm(r['cte_curve_mean'])} / {cm(r['cte_curve_max'])} "
                f"| {r['jerk_ref_max']:.2f} / {ms(r.get('jerk_out_p99'))} "
                f"/ {ms(r.get('jerk_gt_p99'))} "
                f"| {r['t_actual']:.1f} / {r['t_pred']:.1f} / {r['t_naive']:.1f} "
                f"| {r['t_pred_err_pct']:.1f} "
                f"| {cm(r.get('goal_overshoot_m'))} / {cm(r.get('final_pos_err_m'))} "
                f"| {r['min_footprint_clearance']} "
                f"| {ms(r.get('cycle_ms_mean'))} / {ms(r.get('cycle_ms_p95'))} "
                f"/ {ms(r.get('cycle_ms_max'))} |")
    nr = [r for r in results if 'goal' in r]
    if nr:
        ok = [r for r in nr if r['succeeded'] and 't_pred' in r]
        lines += ['', '| controller | goal | ok | length [m] | T act / pred / naive [s] '
                  '| pred err [%] | naive err [%] | min clearance [m] |',
                  '| --- | --- | --- | --- | --- | --- | --- | --- |']
        for r in nr:
            if 't_pred' in r:
                lines.append(
                    f"| {r['controller']} | ({r['goal'][0]:.1f}, {r['goal'][1]:.1f}) "
                    f"| {int(r['succeeded'])} | {r['length_m']:.1f} "
                    f"| {r['t_actual']:.1f} / {r['t_pred']:.1f} / {r['t_naive']:.1f} "
                    f"| {r['t_pred_err_pct']:.1f} | {r['t_naive_err_pct']:.1f} "
                    f"| {r['min_footprint_clearance']} |")
        if ok:
            errs = np.array([r['t_pred_err_pct'] for r in ok])
            lines.append(f'\n{len(ok)}/{len(nr)} succeeded; prediction error mean '
                         f'{errs.mean():.1f} %, max {errs.max():.1f} %')
    return '\n'.join(lines) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='closed-loop tracking / travel-time evaluation')
    ap.add_argument('--mode', choices=['follow', 'navigate', 'both'], default='follow')
    ap.add_argument('--controllers', default='DWA,PurePursuit')
    ap.add_argument('--paths', default='straight,uturn,scurve,narrow')
    ap.add_argument('--layout', choices=['synthetic', 'gazebo'], default='synthetic',
                    help='기준 경로 배치 (gazebo: 월드 동적 장애물 경로를 피한 남측 공터)')
    ap.add_argument('--out', required=True)
    args, ros_args = ap.parse_known_args(argv)
    os.makedirs(args.out, exist_ok=True)
    rclpy.init(args=ros_args)
    ev = Evaluator()
    for client in (ev.follow, ev.nav):
        if not client.wait_for_server(timeout_sec=60.0):
            print(f'action server {client._action_name} not available', file=sys.stderr)
            return 2
    ev.spin_for(2.0)
    results = []
    paths = reference_paths(args.layout)
    for ctrl in [c for c in args.controllers.split(',') if c]:
        if args.mode in ('follow', 'both'):
            for name in [p for p in args.paths.split(',') if p]:
                results.append(follow_run(ev, name, paths[name], ctrl, args.out))
        if args.mode in ('navigate', 'both'):
            results += navigate_runs(ev, ctrl, args.out)
    with open(os.path.join(args.out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=1)
    md = to_markdown(results)
    with open(os.path.join(args.out, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(md)
    print(md)
    ev.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
