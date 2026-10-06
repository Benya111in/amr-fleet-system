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
      sim/footprint_clearance (운동학 시뮬레이터: 정적 지도까지, 1 m 안에 없으면 1.0),
      동적 장애물 여유 = ground_truth/odom 풋프린트 ↔ perception/tracked_obstacles 원(--obstacle-radius)
      최소 거리. 결과는 <out>/summary.json.
끝 자세 (A2): 'end' = odometry/filtered (odom 프레임), 'end_map' = TF map → --base-frame (위치추정),
      'end_gt' = ground_truth/odom (Gazebo 지면 진실, world = map 원점) — navigate 의 목표(map)까지 거리를
      'final_dist_map'·'final_dist_gt' 로, 목표 yaw 오차를 'final_yaw_err_map' 으로. 'elapsed_sim_s' 는 노드
      시계(use_sim_time 이면 /clock) 경과, 'rtf' = elapsed_sim_s / elapsed_s.
"""
import argparse
import json
import math
import os
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath, NavigateToPose
from amr_msgs.msg import TrackedObstacleArray
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from tf2_ros import Buffer, TransformListener

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

    def __init__(self, obstacle_radius):
        super().__init__('rl_smoke')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmds = []
        self.stats = []
        self.stats_t = []
        self.clear = []
        self.dyn_clear = []
        self.odom = None
        self.gt = None
        self.r_obs = obstacle_radius
        self.create_subscription(Twist, 'cmd_vel_nav', self._cmd, 50)
        self.create_subscription(Float64MultiArray, 'rl_controller/stats', self._stats, 50)
        self.create_subscription(Float64, 'sim/footprint_clearance',
                                 lambda m: self.clear.append(m.data), 50)
        self.create_subscription(Odometry, 'odometry/filtered', self._odom, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self._gt, 10)
        self.create_subscription(TrackedObstacleArray, 'perception/tracked_obstacles',
                                 self._tracks, 10)
        self.plan = None
        self.n_plans = 0
        self.create_subscription(Path, 'plan', self._plan, 10)
        self.cmd_t = []

    def _plan(self, m):
        self.plan = m
        self.n_plans += 1

    def _stats(self, m):
        self.stats.append(list(m.data))
        self.stats_t.append(time.monotonic())

    def _cmd(self, m):
        self.cmds.append(m)
        self.cmd_t.append(time.monotonic())

    def _odom(self, m):
        self.odom = m

    def _gt(self, m):
        self.gt = m

    def _tracks(self, m):
        if self.gt is None or not m.obstacles:
            return
        p = self.gt.pose.pose
        q = p.orientation
        th = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(th), math.sin(th)
        best = math.inf
        for o in m.obstacles:
            dx, dy = o.position.x - p.position.x, o.position.y - p.position.y
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            ex, ey = max(abs(lx) - 0.30, 0.0), max(abs(ly) - 0.20, 0.0)
            best = min(best, math.hypot(ex, ey) - self.r_obs)
        self.dyn_clear.append(best)

    def pose_map(self, frame='map', base='base_footprint'):
        """TF map → base 자세 (x, y, yaw) 또는 None."""
        try:
            t = self.tf_buffer.lookup_transform(frame, base, rclpy.time.Time())
        except Exception:  # noqa: BLE001 — TF 가 아직 없으면 None
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def pose_gt(self):
        """지면 진실 자세 (x, y, yaw) 또는 None."""
        if self.gt is None:
            return None
        p = self.gt.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.position.x, p.position.y, yaw

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
    ap.add_argument('--obstacle-radius', type=float, default=0.30)
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--out', required=True)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Smoke(args.obstacle_radius)
    os.makedirs(args.out, exist_ok=True)
    summary = {'mode': args.mode, 'controller_id': args.controller_id}
    if not spin_until(node, lambda: node.odom is not None, 60.0):
        summary['error'] = 'odometry/filtered 없음'
    else:
        x, y, yaw, frame = node.pose_now()
        summary['start'] = [x, y, yaw, frame]
        goal_yaw = None
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
            goal_yaw = gyaw
            goal = NavigateToPose.Goal()
            goal.pose = _pose(args.frame, gx, gy, gyaw, stamp)
            target = (gx, gy)
        summary['target'] = list(target)
        if not client.wait_for_server(timeout_sec=60.0):
            summary['error'] = '액션 서버 없음'
        else:
            t0 = time.monotonic()
            ts0 = node.get_clock().now()
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
            summary['elapsed_sim_s'] = (node.get_clock().now() - ts0).nanoseconds * 1e-9
            if summary['elapsed_s'] > 0:
                summary['rtf'] = summary['elapsed_sim_s'] / summary['elapsed_s']
            if node.odom is not None:
                x1, y1, _, _ = node.pose_now()
                summary['end'] = [x1, y1]
                summary['moved_m'] = math.hypot(x1 - x, y1 - y)
            spin_until(node, lambda: False, 0.5)   # 마지막 TF·지면 진실
            pm = node.pose_map(base=args.base_frame)
            pg = node.pose_gt()
            if node.plan is not None and node.plan.poses:
                e = node.plan.poses[-1].pose
                q = e.orientation
                summary['plan_end'] = [e.position.x, e.position.y, math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))]
                summary['plan_frame'] = node.plan.header.frame_id
                summary['plan_points'] = len(node.plan.poses)
                summary['n_plans'] = node.n_plans
            for key, pose in (('map', pm), ('gt', pg)):
                if args.mode != 'navigate':
                    break   # follow 의 목표는 odom 프레임 — 'end' 와 'target' 으로 본다
                if pose is None:
                    continue
                summary[f'end_{key}'] = list(pose)
                summary[f'final_dist_{key}'] = math.hypot(pose[0] - target[0],
                                                          pose[1] - target[1])
                if goal_yaw is not None:
                    summary[f'final_yaw_err_{key}'] = abs(
                        math.remainder(pose[2] - goal_yaw, 2.0 * math.pi))
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
    summary['min_dynamic_clearance'] = min(node.dyn_clear) if node.dyn_clear else None
    summary['n_valid_cells'] = {'p50': pct([s[8] for s in dec if s[8] >= 0], 50),
                                'min': pct([s[8] for s in dec if s[8] >= 0], 0)}
    summary['saturated_decisions'] = sum(1 for s in dec if s[9] > 0.5)
    # 계약 동적 창으로 자른 DWA 틱 누적 (stats 24, 리뷰 반영 뒤 판부터)
    last = node.stats[-1] if node.stats else []
    summary['dwa_limited'] = last[24] if len(last) >= 25 else None
    # 틱당 명령 변화 (stats 10 v, 11 ω). 수신 간격 0.5 s 초과(= 엔진 재동기)는 뺀다.
    # 'handover' = 직전 틱이 DWA 가 아니고 이번 틱이 DWA (정책·정렬 → DWA 전환)
    st = []
    for i in range(1, len(node.stats)):
        a, b = node.stats[i - 1], node.stats[i]
        if len(a) < 12 or len(b) < 12 or node.stats_t[i] - node.stats_t[i - 1] > 0.5:
            continue
        st.append((abs(b[10] - a[10]), abs(b[11] - a[11]), int(b[4]) == 2,
                   int(a[4]) != 2 and int(b[4]) == 2))
    hand = [x for x in st if x[3]]
    dwa = [x for x in st if x[2]]
    summary['tick_step'] = {
        'n': len(st), 'max_dv': max((x[0] for x in st), default=None),
        'max_dw': max((x[1] for x in st), default=None),
        'n_dwa': len(dwa), 'max_dv_dwa': max((x[0] for x in dwa), default=None),
        'max_dw_dwa': max((x[1] for x in dwa), default=None),
        'n_handover': len(hand), 'max_dv_handover': max((x[0] for x in hand), default=None),
        'max_dw_handover': max((x[1] for x in hand), default=None)}
    if args.mode == 'follow' and 'end' in summary:
        summary['final_dist_odom'] = math.hypot(summary['end'][0] - summary['target'][0],
                                                summary['end'][1] - summary['target'][1])
    # 원자료 시계열 (나중에 다시 분석할 수 있게): stats 행·명령, 수신 단조 시각 [s]
    t0s = node.stats_t[0] if node.stats_t else 0.0
    with open(os.path.join(args.out, 'series.json'), 'w', encoding='utf-8') as f:
        json.dump({'stats_fields': 'rl_controller.cpp kStatsFields 순서',
                   'stats_t': [t - t0s for t in node.stats_t], 'stats': node.stats,
                   'cmd_t': [t - t0s for t in node.cmd_t],
                   'cmd': [[c.linear.x, c.angular.z] for c in node.cmds]}, f)
    with open(os.path.join(args.out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
