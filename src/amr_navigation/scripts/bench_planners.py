#!/usr/bin/env python3
"""
전역 계획기 비교 벤치마크 (명세 4.4): AStar vs NavFn vs Smac2D, 같은 지도·같은 쌍.

    ros2 launch amr_navigation planner_benchmark.launch.py map:=<map.yaml>
    ros2 run amr_navigation bench_planners.py --map <map.yaml> --pairs <pairs.csv> --out <dir>

planner_server 의 compute_path_to_pose 액션(use_start=True)을 쌍·계획기마다 호출해
성공 여부, 서버 측 계획 시간(result.planning_time), 왕복 지연(요청→결과), 경로 길이,
최소 여유거리(지도 EDT, 로봇 중심 기준)를 bench_planners.csv 와 bench_planners.md 로 쓴다.
"""
import argparse
import csv
import math
import os
import sys
import time
from typing import Dict, List

from action_msgs.msg import GoalStatus
from amr_navigation import path_metrics, warehouse_map
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def make_pose(frame: str, x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(0.5 * yaw)
    p.pose.orientation.w = math.cos(0.5 * yaw)
    return p


def summarize(rows: List[Dict], planners: List[str]) -> str:
    """계획기별 요약 마크다운 표."""
    lines = ['| planner | success | plan time mean / p95 / max [ms] | round trip mean [ms] '
             '| length mean [m] | length vs AStar | min clearance mean / min [m] '
             '| mid-path clearance mean / min [m] | narrow-passage pairs |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    by_pair = {}
    for r in rows:
        by_pair.setdefault(r['idx'], {})[r['planner']] = r
    for pl in planners:
        rs = [r for r in rows if r['planner'] == pl]
        ok = [r for r in rs if r['success']]
        if not rs:
            continue
        t = np.array([r['plan_ms'] for r in ok]) if ok else np.zeros(1)
        rt = np.array([r['round_trip_ms'] for r in ok]) if ok else np.zeros(1)
        ln = np.array([r['length'] for r in ok]) if ok else np.zeros(1)
        cl = np.array([r['min_clearance'] for r in ok]) if ok else np.zeros(1)
        ratios = []
        for d in by_pair.values():
            if pl in d and 'AStar' in d and d[pl]['success'] and d['AStar']['success']:
                ratios.append(d[pl]['length'] / max(d['AStar']['length'], 1e-9))
        ratio = f'{np.mean(ratios):.3f}' if ratios else '-'
        mid = np.array([r['min_clearance_mid'] for r in ok
                        if not math.isnan(r['min_clearance_mid'])]) if ok else np.zeros(1)
        narrow = sum(1 for r in ok if r['min_clearance_mid'] < 0.35)
        lines.append(
            f'| {pl} | {len(ok)}/{len(rs)} ({100.0 * len(ok) / len(rs):.0f} %) '
            f'| {t.mean():.1f} / {np.percentile(t, 95):.1f} / {t.max():.1f} '
            f'| {rt.mean():.1f} | {ln.mean():.2f} | {ratio} '
            f'| {cl.mean():.3f} / {cl.min():.3f} | {mid.mean():.3f} / {mid.min():.3f} '
            f'| {narrow} |')
    return '\n'.join(lines) + '\n'


class Bench(Node):
    """compute_path_to_pose 클라이언트."""

    def __init__(self, action: str):
        super().__init__('planner_benchmark')
        self.client = ActionClient(self, ComputePathToPose, action)

    def plan(self, start: PoseStamped, goal: PoseStamped, planner: str, timeout: float):
        g = ComputePathToPose.Goal()
        g.start = start
        g.goal = goal
        g.planner_id = planner
        g.use_start = True
        t0 = time.monotonic()
        fut = self.client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None, (time.monotonic() - t0) * 1e3, GoalStatus.STATUS_UNKNOWN
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        rtt = (time.monotonic() - t0) * 1e3
        res = rf.result()
        if res is None:
            return None, rtt, GoalStatus.STATUS_UNKNOWN
        return res.result, rtt, res.status


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='A*/NavFn/Smac2D benchmark via planner_server')
    ap.add_argument('--map', required=True)
    ap.add_argument('--pairs', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--planners', default='AStar,NavFn,Smac')
    ap.add_argument('--action', default='compute_path_to_pose')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--warmup', type=int, default=2, help='계획기마다 버리는 첫 호출 수')
    args, ros_args = ap.parse_known_args(argv)
    planners = [p for p in args.planners.split(',') if p]
    grid = warehouse_map.load_map(args.map)
    dist = warehouse_map.distance_field(grid)
    with open(args.pairs, encoding='utf-8') as f:
        pairs = [[float(v) for v in row] for row in list(csv.reader(f))[1:] if row]

    rclpy.init(args=ros_args)
    node = Bench(args.action)
    if not node.client.wait_for_server(timeout_sec=30.0):
        print('compute_path_to_pose server not available', file=sys.stderr)
        return 2
    rows = []
    for pl in planners:
        for k in range(args.warmup):
            sx, sy, syaw, gx, gy, gyaw = pairs[k % len(pairs)]
            node.plan(make_pose(args.frame, sx, sy, syaw), make_pose(args.frame, gx, gy, gyaw),
                      pl, args.timeout)
        for i, (sx, sy, syaw, gx, gy, gyaw) in enumerate(pairs):
            res, rtt, status = node.plan(make_pose(args.frame, sx, sy, syaw),
                                         make_pose(args.frame, gx, gy, gyaw), pl, args.timeout)
            ok = (res is not None and status == GoalStatus.STATUS_SUCCEEDED
                  and len(res.path.poses) >= 2)
            xy = (np.array([[p.pose.position.x, p.pose.position.y] for p in res.path.poses])
                  if ok else np.zeros((0, 2)))
            plan_ms = (res.planning_time.sec * 1e3 + res.planning_time.nanosec * 1e-6
                       if res is not None else float('nan'))
            end_err = math.hypot(xy[-1, 0] - gx, xy[-1, 1] - gy) if ok else float('nan')
            # 끝점(여유 ≥ 0.45 m 로 뽑은 출발·목표) 영향을 뺀 경로 중간부 여유: 양 끝 1 m 제외
            mid = np.zeros((0, 2))
            if ok:
                s = path_metrics.cumulative_length(xy)
                mid = xy[(s >= 1.0) & (s <= s[-1] - 1.0)]
            rows.append({
                'planner': pl, 'idx': i, 'success': ok, 'plan_ms': plan_ms,
                'round_trip_ms': rtt, 'length': path_metrics.path_length(xy) if ok else 0.0,
                'min_clearance': (path_metrics.min_clearance(xy, dist, grid.resolution,
                                                             grid.origin) if ok else 0.0),
                'min_clearance_mid': (path_metrics.min_clearance(mid, dist, grid.resolution,
                                                                 grid.origin)
                                      if len(mid) else float('nan')),
                'poses': len(xy), 'goal_error': end_err})
            print(f'{pl:6s} #{i:02d} ok={int(ok)} plan {plan_ms:7.1f} ms rtt {rtt:7.1f} ms '
                  f"len {rows[-1]['length']:6.2f} clr {rows[-1]['min_clearance']:.3f}",
                  flush=True)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'bench_planners.csv'), 'w', newline='',
              encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    md = summarize(rows, planners)
    with open(os.path.join(args.out, 'bench_planners.md'), 'w', encoding='utf-8') as f:
        f.write(md)
    print(md)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
