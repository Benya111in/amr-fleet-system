"""
static_tf: amr_description(robot_state_publisher) 대역 — config 의 extrinsic 으로 /tf_static 발행.

URDF 가 없는 워크스페이스에서 safety_node 등이 base_link ↔ 센서 변환을 찾을 수 있게 한다.
간선과 값은 amr_itest.tf_tree.expected_edges() 와 같은 출처(sensors.yaml, robot_params.yaml)다.
바퀴 링크는 고정 자세(회전 0)로 발행한다.
"""

from amr_itest import config
from amr_itest import tf_tree
from amr_itest.standins.common import run, StandinNode
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTf(StandinNode):
    """config → /tf_static."""

    def __init__(self):
        super().__init__('static_tf')
        prefix = str(self.param('frame_prefix', ''))
        edges = tf_tree.expected_edges(config.sensors(), config.robot_params(),
                                       config.ekf_params(), prefix)
        stamp = self.get_clock().now().to_msg()
        msgs = []
        for e in edges:
            if e.translation is None or e.parent.endswith('odom') or e.parent == 'map':
                continue
            tr = TransformStamped()
            tr.header.stamp = stamp
            tr.header.frame_id = e.parent
            tr.child_frame_id = e.child
            tr.transform.translation.x, tr.transform.translation.y, \
                tr.transform.translation.z = e.translation
            rot = e.rotation or (0.0, 0.0, 0.0, 1.0)
            tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, \
                tr.transform.rotation.w = rot
            msgs.append(tr)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(msgs)
        self.get_logger().info(f'/tf_static {len(msgs)} 간선 발행 (config extrinsic)')


def main(args=None) -> None:
    run(StaticTf, args)


if __name__ == '__main__':
    main()
