#!/usr/bin/env python3
"""
SLAM map 프레임을 월드에 고정하는 정적 TF — `<parent> → <child>` = 스폰 자세 (⊕ 선택적 TF).

    anchor_tf.py --parent map --child slam_map --pose 18.0,-16.0,1.5708
    anchor_tf.py --parent slam_map --child amr_01/liosam_odom --pose 0,0,0 \
        --via amr_01/base_footprint amr_01/lidar3d_link

SLAM 의 map(또는 오도메트리) 프레임은 출발 자세에서 시작한다. 평가 계약 "map = 월드" 를 위해 출발 자세를 정적 TF 로
준다 — 정답이 아니라 **스폰 자세**(현행 2D 스택의 AMCL 초기 자세와 같은 정보)를 쓴다.
--via a b 를 주면 /tf_static 에서 a→b 를 기다렸다가 pose ⊕ T(a→b) 를 낸다 (LIO-SAM 의 오도메트리 원점은
base_footprint 가 아니라 LiDAR 이므로 마스트 장착 자리만큼 옮긴다 — 수치를 하드코딩하지 않고 URDF 에서 읽는다).
"""

import argparse
import math
import sys

from amr_slam3d.se2 import parse_pose
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def qmul(a, b):
    """쿼터니언 곱 (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def compose_planar(pose, t, q):
    """평면 자세 (x, y, yaw) ⊕ 3D 변환 (t, q) → (t', q')."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    t2 = (x + c * t[0] - s * t[1], y + s * t[0] + c * t[1], t[2])
    qy = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return t2, qmul(qy, q)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--parent', required=True)
    ap.add_argument('--child', required=True)
    ap.add_argument('--pose', required=True, help="'x,y,yaw' (스폰 자세)")
    ap.add_argument('--via', nargs=2, metavar=('FROM', 'TO'), default=None)
    ns, ros_args = ap.parse_known_args(argv)
    pose = parse_pose(ns.pose)
    rclpy.init(args=ros_args)
    name = 'anchor_' + ns.child.replace('/', '_')
    node = Node(name, parameter_overrides=[Parameter('use_sim_time', value=True)])
    t, q = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    if ns.via:
        buf = Buffer()
        TransformListener(buf, node)
        src, dst = ns.via
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if buf.can_transform(src, dst, Time(), Duration(seconds=0.0)):
                tr = buf.lookup_transform(src, dst, Time()).transform
                t = (tr.translation.x, tr.translation.y, tr.translation.z)
                q = (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
                break
    tt, qq = compose_planar(pose, t, q)
    msg = TransformStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = ns.parent
    msg.child_frame_id = ns.child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = tt
    (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
     msg.transform.rotation.w) = qq
    br = StaticTransformBroadcaster(node)
    br.sendTransform(msg)
    node.get_logger().info(f'{ns.parent} → {ns.child}: t=({tt[0]:.4f}, {tt[1]:.4f}, {tt[2]:.4f}) '
                           f'q=({qq[0]:.5f}, {qq[1]:.5f}, {qq[2]:.5f}, {qq[3]:.5f}) '
                           f'pose={ns.pose} via={ns.via}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
