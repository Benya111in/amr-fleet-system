#!/usr/bin/env python3
"""
지면 진실 TF 중계 (시험 하네스 전용).

ground_truth/odom (frame world) → TF odom→<prefix>base_footprint, map→odom 항등 (계약 C4: map = world).
위치 추정 스택 없이 안전 게이트·점군·예외 다각형을 시험할 때 쓴다. 로봇을 set_pose 로 옮겨도 TF 가
따라간다.

    ros2 run ... gt_tf_relay.py --ros-args -r __ns:=/amr_01 -p frame_prefix:=''
"""

import sys


def main() -> int:  # pragma: no cover - Gazebo 시험 하네스 (tracking.md §9.4)
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init()
    node = Node('gt_tf_relay')
    prefix = node.declare_parameter('frame_prefix', '').value
    br = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    ident = TransformStamped()
    ident.header.frame_id = 'map'
    ident.child_frame_id = prefix + 'odom'
    ident.header.stamp = node.get_clock().now().to_msg()
    ident.transform.rotation.w = 1.0
    static.sendTransform([ident])

    def on_odom(msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = prefix + 'odom'
        t.child_frame_id = prefix + 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation
        br.sendTransform(t)

    node.create_subscription(Odometry, 'ground_truth/odom', on_odom, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):  # noqa: B902 - 런치 종료 시 조용히
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
