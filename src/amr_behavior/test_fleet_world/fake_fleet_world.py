#!/usr/bin/env python3
"""
가짜 플릿 세계 — Nav2 · 도킹 · 인식 · 위치추정 없이 실제 task_executor_node 를 여러 대 돌리는 운동학 대역 (b4-wire).

    python3 fake_fleet_world.py --robots amr_01,amr_02 --poses="-22,0,0;-15,-9,1.57" \
        --out DIR --ros-args -p use_sim_time:=true

로봇마다 (네임스페이스 /<robot>):
- ActS navigate_to_pose (nav2_msgs/NavigateToPose): 현재 자세에서 goal 까지 직선 경로를 `plan`
  (nav_msgs/Path, map)으로 내고 V(1.0 m/s)로 따라간다. 취소되면 감속 ACC(1.0 m/s²)로 경로를 따라 미끄러져
  선 뒤 canceled.
  새 goal 은 진행 중인 goal 을 끝낸다 (preempt). 끝에서 goal 자세(방위 포함)로 놓는다.
- ActS spin · backup · wait (nav2_msgs): 각 0.5 s 안팎 (spin 은 방위만 바꾸고, backup 은 뒤로 거리만큼).
- ActS dock (amr_msgs/Dock): 1 s 뒤 성공 (분산 시험은 도크 없는 작업이라 거의 안 쓴다).
- SrvS global_costmap/clear_entirely_global_costmap · local_costmap/clear_entirely_local_costmap.
- Pub odometry/filtered_map (nav_msgs/Odometry, map, 20 Hz sim),
  perception/detected_objects (box 1.0 m, 5 Hz).
물리 · 시각은 노드 시계 (use_sim_time 이면 /clock). 실제 자세를 OUT/poses.jsonl (sim 10 Hz, 모든 로봇 한 줄)에
남긴다 — 시험이 이것으로 실제 몸체 기준 구역 동시 점유를 센다 (에이전트가 믿은 허가와 따로).
충돌 · 장애물 · 경로 계획은 흉내 내지 않는다 (직선). 시험 대역이라 패키지에 설치하지 않는다.
"""

import argparse
import json
import math
import sys
import threading
import time

from amr_msgs.action import Dock
from amr_msgs.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, NavigateToPose, Spin, Wait
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

V = 1.0          # [m/s] nav2_params max_vel_x
ACC = 1.0        # [m/s²] decel_lim_x


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Robot:
    """로봇 1 대의 운동학 상태 (lock 은 세계가 쥔다)."""

    def __init__(self, rid, x, y, yaw):
        self.rid = rid
        self.x, self.y, self.yaw = x, y, yaw
        self.v = 0.0
        self.path = None          # [(x, y)] 진행 중 경로
        self.s = 0.0
        self.cum = None
        self.goal_yaw = yaw
        self.gen = 0              # goal 세대 (preempt 판정)
        self.state = 'idle'       # idle · moving · braking · arrived · stopped
        self.nav_goals = 0
        self.cancels = 0


class FakeFleetWorld(Node):
    """여러 로봇의 운동학 대역."""

    def __init__(self, robots, poses, out):
        super().__init__('fake_fleet_world')
        self.lock = threading.Lock()
        self.robots = {r: Robot(r, *p) for r, p in zip(robots, poses)}
        self.out = out
        self._pose_log = open(f'{out}/poses.jsonl', 'a', encoding='utf-8') if out else None
        self._last_log = None
        self._last_phys = None
        self.cbg = ReentrantCallbackGroup()
        self.pubs = {}
        for r in robots:
            ns = f'/{r}'
            self.pubs[r] = (self.create_publisher(Odometry, f'{ns}/odometry/filtered_map', 10),
                            self.create_publisher(Path, f'{ns}/plan', 5),
                            self.create_publisher(DetectedObjectArray,
                                                  f'{ns}/perception/detected_objects',
                                                  qos_profile_sensor_data))
            ActionServer(self, NavigateToPose, f'{ns}/navigate_to_pose',
                         execute_callback=lambda h, r=r: self._nav(r, h),
                         goal_callback=lambda g: GoalResponse.ACCEPT,
                         cancel_callback=lambda h: CancelResponse.ACCEPT, callback_group=self.cbg)
            ActionServer(self, Spin, f'{ns}/spin',
                         execute_callback=lambda h, r=r: self._spin(r, h),
                         cancel_callback=lambda h: CancelResponse.ACCEPT, callback_group=self.cbg)
            ActionServer(self, BackUp, f'{ns}/backup',
                         execute_callback=lambda h, r=r: self._backup(r, h),
                         cancel_callback=lambda h: CancelResponse.ACCEPT, callback_group=self.cbg)
            ActionServer(self, Wait, f'{ns}/wait', execute_callback=lambda h: self._short(h, Wait),
                         cancel_callback=lambda h: CancelResponse.ACCEPT, callback_group=self.cbg)
            ActionServer(self, Dock, f'{ns}/dock', execute_callback=self._dock,
                         cancel_callback=lambda h: CancelResponse.ACCEPT, callback_group=self.cbg)
            for which in ('global', 'local'):
                self.create_service(
                    ClearEntireCostmap,
                    f'{ns}/{which}_costmap/clear_entirely_{which}_costmap',
                    lambda req, resp: resp, callback_group=self.cbg)
        self.create_timer(0.01, self._physics, callback_group=self.cbg,
                          clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.create_timer(0.05, self._publish_odom, callback_group=self.cbg)
        self.create_timer(0.2, self._publish_objects, callback_group=self.cbg)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 물리
    def _physics(self):
        now = self.sim()
        if now <= 0.0:
            return
        if self._last_phys is None:
            self._last_phys = now
            return
        dt = min(max(now - self._last_phys, 0.0), 0.1)
        self._last_phys = now
        with self.lock:
            for rb in self.robots.values():
                self._step(rb, dt)
            if self._pose_log is not None and (self._last_log is None
                                               or now - self._last_log >= 0.1):
                self._last_log = now
                self._pose_log.write(json.dumps(
                    {'t': round(now, 3), 'wall': round(time.time(), 3),
                     'p': {r: [round(b.x, 3), round(b.y, 3), round(b.yaw, 3), round(b.v, 3),
                               b.state] for r, b in self.robots.items()}}) + '\n')
                self._pose_log.flush()

    def _step(self, rb, dt):
        if rb.state not in ('moving', 'braking') or rb.path is None:
            rb.v = 0.0
            return
        if rb.state == 'moving':
            rb.v = min(V, rb.v + ACC * dt)
        else:
            rb.v = max(0.0, rb.v - ACC * dt)
        rb.s = min(rb.cum[-1], rb.s + rb.v * dt)
        rb.x, rb.y = self._at(rb, rb.s)
        if rb.s >= rb.cum[-1] - 1e-9:
            if rb.state == 'moving':
                rb.yaw = rb.goal_yaw
                rb.state = 'arrived'
            else:
                rb.state = 'stopped'
            rb.v = 0.0
        elif rb.state == 'braking' and rb.v <= 0.0:
            rb.state = 'stopped'

    @staticmethod
    def _at(rb, s):
        for k in range(1, len(rb.path)):
            if s <= rb.cum[k] or k == len(rb.path) - 1:
                a, b = rb.path[k - 1], rb.path[k]
                seg = rb.cum[k] - rb.cum[k - 1]
                f = 0.0 if seg <= 0 else min(1.0, max(0.0, (s - rb.cum[k - 1]) / seg))
                return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
        return rb.path[-1]

    # ------------------------------------------------------------------ 발행
    def _publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            snap = [(r, b.x, b.y, b.yaw, b.v) for r, b in self.robots.items()]
        for r, x, y, yaw, v in snap:
            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id = 'map'
            m.child_frame_id = f'{r}/base_link'
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z = math.sin(0.5 * yaw)
            m.pose.pose.orientation.w = math.cos(0.5 * yaw)
            m.twist.twist.linear.x = v
            self.pubs[r][0].publish(m)

    def _publish_objects(self):
        stamp = self.get_clock().now().to_msg()
        for r in self.robots:
            arr = DetectedObjectArray()
            arr.header.stamp = stamp
            o = DetectedObject()
            o.class_name = 'box'
            o.confidence = 0.9
            o.distance = 1.0
            arr.objects.append(o)
            self.pubs[r][2].publish(arr)

    def _plan(self, r, pts):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pubs[r][1].publish(msg)

    # ------------------------------------------------------------------ 액션
    def _nav(self, r, handle):
        g = handle.request.pose.pose
        gx, gy, gyaw = g.position.x, g.position.y, yaw_of(g.orientation)
        with self.lock:
            rb = self.robots[r]
            rb.gen += 1
            gen = rb.gen
            rb.nav_goals += 1
            d = math.hypot(gx - rb.x, gy - rb.y)
            n = max(1, int(math.ceil(d / 0.2)))
            pts = [(rb.x + (gx - rb.x) * k / n, rb.y + (gy - rb.y) * k / n) for k in range(n + 1)]
            rb.path, rb.s, rb.goal_yaw = pts, 0.0, gyaw
            rb.cum = [0.0]
            for k in range(1, len(pts)):
                rb.cum.append(rb.cum[-1] + math.hypot(pts[k][0] - pts[k - 1][0],
                                                      pts[k][1] - pts[k - 1][1]))
            if d > 1e-6:
                rb.yaw = math.atan2(gy - rb.y, gx - rb.x)
            rb.state = 'moving' if d > 1e-6 else 'arrived'
        self._plan(r, pts)
        while rclpy.ok():
            with self.lock:
                if rb.gen != gen:                       # 새 goal 이 덮었다
                    handle.abort()
                    return NavigateToPose.Result()
                st = rb.state
                if handle.is_cancel_requested and st == 'moving':
                    rb.state = 'braking'
                    rb.cancels += 1
                    st = 'braking'
            if st == 'arrived':
                handle.succeed()
                return NavigateToPose.Result()
            if st == 'stopped':
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        handle.abort()
        return NavigateToPose.Result()

    def _wait_sim(self, handle, seconds):
        end = self.sim() + seconds
        while rclpy.ok() and self.sim() < end:
            if handle.is_cancel_requested:
                return False
            time.sleep(0.02)
        return True

    def _spin(self, r, handle):
        ok = self._wait_sim(handle, 0.5)
        with self.lock:
            if ok:
                rb = self.robots[r]
                rb.yaw = math.atan2(math.sin(rb.yaw + handle.request.target_yaw),
                                    math.cos(rb.yaw + handle.request.target_yaw))
        (handle.succeed if ok else handle.canceled)()
        return Spin.Result()

    def _backup(self, r, handle):
        ok = self._wait_sim(handle, 0.5)
        with self.lock:
            if ok:
                rb = self.robots[r]
                d = abs(handle.request.target.x)
                rb.x -= d * math.cos(rb.yaw)
                rb.y -= d * math.sin(rb.yaw)
        (handle.succeed if ok else handle.canceled)()
        return BackUp.Result()

    def _short(self, handle, kind):
        ok = self._wait_sim(handle, 0.5)
        (handle.succeed if ok else handle.canceled)()
        return kind.Result()

    def _dock(self, handle):
        ok = self._wait_sim(handle, 1.0)
        res = Dock.Result()
        res.success = ok
        (handle.succeed if ok else handle.canceled)()
        return res


def main(argv=None):
    argv = list(sys.argv if argv is None else argv)
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', required=True)
    ap.add_argument('--poses', required=True, help='"x,y,yaw;x,y,yaw;..."')
    ap.add_argument('--out', default='')
    a = ap.parse_args(remove_ros_args(argv)[1:])
    robots = [r.strip() for r in a.robots.split(',') if r.strip()]
    poses = [tuple(float(v) for v in p.split(',')) for p in a.poses.split(';') if p.strip()]
    if len(robots) != len(poses):
        raise SystemExit('robots 와 poses 수가 다르다')
    rclpy.init(args=argv)
    node = FakeFleetWorld(robots, poses, a.out)
    ex = MultiThreadedExecutor(num_threads=32)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
