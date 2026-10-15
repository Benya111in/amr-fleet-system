#!/usr/bin/env python3
"""
정답 자세 TF 중계 (b2-eval, 정답 지도 생성 전용).

ground_truth/odom → TF `<robot>/gt_odom → <robot>/base_footprint`.

    gt_odom_tf.py --pose 18.0,-16.0,1.5708 [--robot amr_01]

`rtabmap3d_gtdb` 행 (rtabmap.launch.py odom_mode:=gt_tf) 이 이 TF 를 오도메트리로 써서, bag 의 점군 장들을 **정답 자세**에
놓은 RTAB-Map DB 를 만든다 — E3(b) 의 "정답 지도 위 RTAB-Map 위치추정" 용 지도. SLAM 행이 아니다 (정답 입력).
gt_odom 원점 = 스폰 자세 (--pose) — 그래서 재생의 `map → slam_map` = 스폰 고정 TF 와 겹쳐 slam_map = gt_odom 이 된다.
값: 평면 (x, y, yaw) 만 — 이 월드의 바닥은 평평하고 RTAB-Map 은 Reg/Force3DoF 다. 스탬프 = 정답 메시지 스탬프 (sim).
"""

import argparse
import math
import sys

from amr_slam3d.se2 import compose, inverse, parse_pose, yaw_from_quat
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def relative(spawn, x, y, yaw):
    """스폰 기준 자세 inv(spawn) ⊕ (x, y, yaw)."""
    return compose(inverse(spawn), (x, y, yaw))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--pose', required=True, help="'x,y,yaw' (스폰 자세)")
    ap.add_argument('--robot', default='amr_01')
    ns, ros_args = ap.parse_known_args(argv)
    spawn = parse_pose(ns.pose)
    rclpy.init(args=ros_args)
    node = Node('gt_odom_tf', parameter_overrides=[Parameter('use_sim_time', value=True)])
    br = TransformBroadcaster(node)
    parent, child = f'{ns.robot}/gt_odom', f'{ns.robot}/base_footprint'
    count = {'n': 0}

    def on_odom(msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y, yaw = relative(spawn, p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        tr = TransformStamped()
        tr.header.stamp = msg.header.stamp
        tr.header.frame_id = parent
        tr.child_frame_id = child
        tr.transform.translation.x, tr.transform.translation.y = x, y
        tr.transform.rotation.z, tr.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        br.sendTransform(tr)
        count['n'] += 1

    node.create_subscription(Odometry, f'/{ns.robot}/ground_truth/odom', on_odom,
                             QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE))
    node.get_logger().info(f'{parent} → {child} from /{ns.robot}/ground_truth/odom, '
                           f'spawn={ns.pose}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'relayed {count["n"]}')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
