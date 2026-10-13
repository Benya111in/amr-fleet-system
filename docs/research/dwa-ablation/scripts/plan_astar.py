#!/usr/bin/env python3
"""
held-out 창고 A* 경로: astar_pairs.json 의 쌍을 배포 planner_server(AStar, 전역 코스트맵)로 계획한다.

    python3 plan_astar.py --pairs /runs/paths/astar_pairs.json \
        --out /runs/paths/heldout_astar.json --n 20 --ros-args -r __ns:=/amr_61

시작 자세 = 경로 첫 접선, 끝 자세 = 마지막 접선으로 다시 쓴다 (출발·도착 제자리 회전 없이 추종만 본다).
경로 길이 < 5 m, 계획 실패는 버리고 다음 쌍을 쓴다 (쌍 순서 그대로 앞 n 개).
"""
import argparse
import json
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def pose(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(0.5 * yaw)
    p.pose.orientation.w = math.cos(0.5 * yaw)
    return p


def resample(xy, step=0.05):
    d = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(d)])
    keep = np.concatenate([[True], d > 1e-9])
    s, xy = s[keep], xy[keep]
    n = int(math.floor(s[-1] / step))
    ss = np.concatenate([np.arange(n + 1) * step, [s[-1]]]) if s[-1] - n * step > 1e-6 \
        else np.arange(n + 1) * step
    return np.stack([np.interp(ss, s, xy[:, 0]), np.interp(ss, s, xy[:, 1])], axis=1)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--pairs', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=20)
    args, ros_args = ap.parse_known_args(argv)
    with open(args.pairs, encoding='utf-8') as f:
        data = json.load(f)
    rclpy.init(args=ros_args)
    node = Node('dwa_ablation_astar')
    cli = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    if not cli.wait_for_server(timeout_sec=180.0):
        print('planner 없음', file=sys.stderr)
        return 2
    time.sleep(3.0)
    out = []
    log = []
    for pr in data['pairs']:
        if len(out) >= args.n:
            break
        (sx, sy), (gx, gy) = pr['start'], pr['goal']
        yaw = math.atan2(gy - sy, gx - sx)
        goal = ComputePathToPose.Goal()
        goal.goal = pose(gx, gy, yaw)
        goal.start = pose(sx, sy, yaw)
        goal.use_start = True
        goal.planner_id = 'AStar'
        fut = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        h = fut.result()
        if h is None or not h.accepted:
            log.append({'id': pr['id'], 'ok': False, 'why': 'rejected'})
            continue
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=30.0)
        res = rf.result()
        if res is None or not res.result.path.poses:
            log.append({'id': pr['id'], 'ok': False, 'why': 'no path'})
            continue
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in res.result.path.poses])
        xy = resample(xy)
        length = float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))
        if length < 5.0:
            log.append({'id': pr['id'], 'ok': False, 'why': f'short {length:.2f}'})
            continue
        th = np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0]))
        k5 = min(5, len(xy) - 1)
        th[0] = math.atan2(xy[k5, 1] - xy[0, 1], xy[k5, 0] - xy[0, 0])
        th[-1] = math.atan2(xy[-1, 1] - xy[-6, 1], xy[-1, 0] - xy[-6, 0])
        pts = [(float(x), float(y), float(t)) for (x, y), t in zip(xy, th)]
        out.append({'id': pr['id'], 'family': 'astar', 'points': pts,
                    'meta': {'start': pr['start'], 'goal': pr['goal'], 'length': length,
                             'planning_time_s': res.result.planning_time.sec +
                             1e-9 * res.result.planning_time.nanosec}})
        log.append({'id': pr['id'], 'ok': True, 'length': length})
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(out, f)
    print(json.dumps(log))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
