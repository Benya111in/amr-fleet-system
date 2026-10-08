#!/usr/bin/env python3
"""
정답(GT) 되먹임 경로 주행 노드 — 파일럿 bag 기록용 (경로 논리는 amr_slam3d/route.py).

    route_driver.py --route share/amr_slam3d/config/pilot_route.yaml --robot amr_01 \
        --log /out/route.jsonl

/<robot>/ground_truth/odom (월드 프레임, 50 Hz) 을 받아 /<robot>/cmd_vel 을 rate Hz 로 낸다 (시뮬레이션 시각 타이머).
perception(safety node) 를 띄우지 않은 실행에서만 쓴다 — cmd_vel 이 Gazebo DiffDrive 로 바로 간다.
단계가 바뀔 때마다 (sim 시각, 자세, 단계, 웨이포인트) 를 JSON 줄로 남긴다 → 정지·직진·회전 구간 분해에 쓴다.
끝나면 0 명령을 몇 번 더 내고 종료 코드 0. 벽시계 상한(--timeout-wall)을 넘으면 0 명령 뒤 종료 코드 2.

녹화 프로토콜 (보너스 2 R 단계, record_bag.sh — 기록 중에는 새 프로세스를 띄우지 않는다):
  --ready-file F --ready-after S : GT 를 받고 sim 시각으로 S 초 동안 0 명령을 낸 뒤 F 를 만든다 (이 노드의 기동 부하가
                                   기록 시작 전에 끝나게 — 파일럿 bag 의 9.48 s 정체는 이 노드 기동과 겹쳤다)
  --go-file F                    : F 가 생길 때까지 wait_go (0 명령). 생기면 hold_start 를 센다 (기록기 구독 확인 뒤)
  --done-file F                  : 경로가 끝나면 (hold_end 포함) F 를 만든다
  --quit-file F                  : 끝난 뒤에도 0 명령을 내며 F 가 생길 때까지 남는다 (기록기를 먼저 멈추고 이 노드를
                                   나중에 끝내 종료 부하가 bag 에 들어가지 않게)
파일 검사는 0.1 s(벽시계)마다 os.path.exists 하나다.
"""

import argparse
import json
import os
import sys
import time

from amr_slam3d.route import route_from_dict, RouteFollower
from amr_slam3d.se2 import yaw_from_quat
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml


class Driver(Node):
    """GT 자세 → RouteFollower → cmd_vel."""

    def __init__(self, spec, robot: str, rate: float, log_path: str, armed: bool = True):
        super().__init__('route_driver',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.follower = RouteFollower(spec, armed=armed)
        self.pose = None
        self.done = False
        self.phase = None
        self.t_first = None
        self.t_now = None
        self.log = open(log_path, 'w', encoding='utf-8') if log_path else None
        self.pub = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)
        self.create_subscription(Odometry, f'/{robot}/ground_truth/odom', self._on_gt, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'경로 {len(spec.waypoints)} 점, hold {spec.hold_start_s:g} s, '
                               f'v_max {spec.limits.v_max:g} m/s')

    def _on_gt(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def _tick(self) -> None:
        if self.pose is None or self.done:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t_first is None:
            self.t_first = t
        self.t_now = t
        cmd = self.follower.step(t, *self.pose)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(cmd.v), float(cmd.w)
        self.pub.publish(msg)
        if cmd.phase != self.phase:
            self.phase = cmd.phase
            rec = {'t': round(t, 4), 'phase': cmd.phase, 'index': cmd.index,
                   'x': round(self.pose[0], 4), 'y': round(self.pose[1], 4),
                   'yaw': round(self.pose[2], 5)}
            self.get_logger().info(json.dumps(rec, ensure_ascii=False))
            if self.log:
                self.log.write(json.dumps(rec) + '\n')
                self.log.flush()
        if cmd.done:
            self.done = True

    def stop(self, n: int = 5) -> None:
        for _ in range(n):
            self.pub.publish(Twist())
            time.sleep(0.05)
        if self.log:
            self.log.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--route', required=True)
    ap.add_argument('--robot', default='amr_01')
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--log', default='')
    ap.add_argument('--timeout-wall', type=float, default=7200.0)
    ap.add_argument('--ready-file', default='')
    ap.add_argument('--ready-after', type=float, default=10.0, help='[s, sim]')
    ap.add_argument('--go-file', default='')
    ap.add_argument('--done-file', default='')
    ap.add_argument('--quit-file', default='')
    ns, ros_args = ap.parse_known_args(argv)
    with open(ns.route, encoding='utf-8') as f:
        spec = route_from_dict((yaml.safe_load(f) or {})['route'])
    rclpy.init(args=ros_args)
    node = Driver(spec, ns.robot, ns.rate, ns.log, armed=not ns.go_file)
    deadline = time.monotonic() + ns.timeout_wall
    rc = 0
    ready = not ns.ready_file
    armed = not ns.go_file
    last_check = 0.0

    def touch(path: str, text: str) -> None:
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text + '\n')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline:
                node.get_logger().error('벽시계 상한 초과 — 정지')
                rc = 2
                break
            if time.monotonic() - last_check < 0.1:
                continue
            last_check = time.monotonic()
            if not ready and node.t_first is not None and \
                    node.t_now - node.t_first >= ns.ready_after:
                touch(ns.ready_file, json.dumps({'t_first': node.t_first, 't_ready': node.t_now}))
                ready = True
            if ready and not armed and os.path.exists(ns.go_file):
                node.follower.arm()
                armed = True
                node.get_logger().info(f'출발 신호 (sim {node.t_now})')
            if node.done:
                if ns.done_file and not os.path.exists(ns.done_file):
                    touch(ns.done_file, json.dumps({'t_done': node.t_now}))
                if not ns.quit_file or os.path.exists(ns.quit_file):
                    break
                node.pub.publish(Twist())
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
