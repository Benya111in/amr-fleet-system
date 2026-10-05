#!/usr/bin/env python3
"""
RLController 통합 스모크 — controller_server 에 실린 플러그인이 명령을 내는지 확인한다.

    ros2 run amr_rl_controller rl_smoke.py --mode follow --controller-id RL --length 6 \
        --out /tmp/rl_smoke --ros-args -r __ns:=/amr_01
    ros2 run amr_rl_controller rl_smoke.py --mode navigate --goal 3.0,0.0,0.0 --out ...

follow  : 현재 자세(odometry/filtered)에서 앞으로 --length m 직선 경로를 만들어 follow_path 액션으로 보낸다
          (controller_id 지정, BT 를 거치지 않는다).
navigate: navigate_to_pose 액션 (BT 기본 제어기 — replace_dwa:=true 로 띄웠을 때 RL).
기록: cmd_vel_nav (controller_server 출력, navigation.launch.py 의 리맵), rl_controller/stats,
      sim/footprint_clearance (운동학 시뮬레이터가 있으면). 결과는 <out>/summary.json.
"""
import argparse
import json
import math
import os
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

SOURCES = ('policy', 'fallback_cell', 'dwa', 'align')
REASONS = ('none', 'state', 'policy_error', 'timeout', 'nonfinite')


def pct(v, p):
    """선형 보간 백분위 (빈 목록 = None)."""
    if not v:
        return None
    s = sorted(v)
    i = p / 100.0 * (len(s) - 1)
    lo = int(i)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (i - lo) * (s[hi] - s[lo])


class Smoke(Node):
    """명령·통계 기록기."""

    def __init__(self):
        super().__init__('rl_smoke')
        self.cmds = []
        self.stats = []
        self.clear = []
        self.odom = None
        self.create_subscription(Twist, 'cmd_vel_nav', self.cmds.append, 50)
        self.create_subscription(Float64MultiArray, 'rl_controller/stats',
                                 lambda m: self.stats.append(list(m.data)), 50)
        self.create_subscription(Float64, 'sim/footprint_clearance',
                                 lambda m: self.clear.append(m.data), 50)
        self.create_subscription(Odometry, 'odometry/filtered', self._odom, 10)

    def _odom(self, m):
        self.odom = m

    def pose_now(self):
        """현재 자세 (odom 프레임 x, y, yaw, frame)."""
        p = self.odom.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.position.x, p.position.y, yaw, self.odom.header.frame_id


def _pose(frame, x, y, yaw, stamp):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


def spin_until(node, pred, timeout):
    """pred() 가 참이거나 timeout [s] 까지 돈다."""
    t0 = time.monotonic()
    while rclpy.ok() and not pred() and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    return pred()


def main():
    """스모크 실행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('follow', 'navigate'), default='follow')
    ap.add_argument('--controller-id', default='RL')
    ap.add_argument('--length', type=float, default=6.0)
    ap.add_argument('--goal', default='')
    ap.add_argument('--frame', default='map', help='navigate 목표 프레임')
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--out', required=True)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Smoke()
    os.makedirs(args.out, exist_ok=True)
    summary = {'mode': args.mode, 'controller_id': args.controller_id}
    if not spin_until(node, lambda: node.odom is not None, 60.0):
        summary['error'] = 'odometry/filtered 없음'
    else:
        x, y, yaw, frame = node.pose_now()
        summary['start'] = [x, y, yaw, frame]
        stamp = node.get_clock().now().to_msg()
        if args.mode == 'follow':
            client = ActionClient(node, FollowPath, 'follow_path')
            goal = FollowPath.Goal()
            path = Path()
            path.header.frame_id = frame
            path.header.stamp = stamp
            n = int(args.length / 0.05)
            for k in range(n + 1):
                s = 0.05 * k
                path.poses.append(_pose(frame, x + s * math.cos(yaw), y + s * math.sin(yaw), yaw,
                                        stamp))
            goal.path = path
            goal.controller_id = args.controller_id
            goal.goal_checker_id = 'general_goal_checker'
            target = (path.poses[-1].pose.position.x, path.poses[-1].pose.position.y)
        else:
            client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
            gx, gy, gyaw = (float(v) for v in args.goal.split(','))
            goal = NavigateToPose.Goal()
            goal.pose = _pose(args.frame, gx, gy, gyaw, stamp)
            target = (gx, gy)
        summary['target'] = list(target)
        if not client.wait_for_server(timeout_sec=60.0):
            summary['error'] = '액션 서버 없음'
        else:
            t0 = time.monotonic()
            fut = client.send_goal_async(goal)
            spin_until(node, fut.done, 30.0)
            handle = fut.result() if fut.done() else None
            summary['accepted'] = bool(handle and handle.accepted)
            if handle and handle.accepted:
                res = handle.get_result_async()
                spin_until(node, res.done, args.timeout)
                summary['status'] = int(res.result().status) if res.done() else None
                summary['succeeded'] = summary['status'] == GoalStatus.STATUS_SUCCEEDED
                if not res.done():
                    handle.cancel_goal_async()
                    spin_until(node, lambda: False, 2.0)
            summary['elapsed_s'] = time.monotonic() - t0
            if node.odom is not None:
                x1, y1, _, _ = node.pose_now()
                summary['end'] = [x1, y1]
                summary['moved_m'] = math.hypot(x1 - x, y1 - y)
    v = [c.linear.x for c in node.cmds]
    summary['n_cmd'] = len(node.cmds)
    summary['n_cmd_moving'] = sum(1 for a in v if abs(a) > 1e-3)
    summary['max_v'] = max(v) if v else None
    dec = [s for s in node.stats if len(s) >= 24 and s[12] > 0.5]
    summary['n_stats'] = len(node.stats)
    summary['decisions'] = len(dec)
    summary['sources'] = {n: sum(1 for s in dec if int(s[4]) == i) for i, n in enumerate(SOURCES)}
    summary['dwa_reasons'] = {n: sum(1 for s in dec if int(s[4]) == 2 and int(s[5]) == i)
                              for i, n in enumerate(REASONS)}
    inf = [s[3] for s in dec if int(s[4]) in (0, 1)]
    cyc = [s[0] for s in node.stats]
    summary['infer_ms'] = {'p50': pct(inf, 50), 'p99': pct(inf, 99), 'max': pct(inf, 100)}
    summary['cycle_ms'] = {'p50': pct(cyc, 50), 'p99': pct(cyc, 99), 'max': pct(cyc, 100)}
    summary['min_footprint_clearance'] = min(node.clear) if node.clear else None
    with open(os.path.join(args.out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
