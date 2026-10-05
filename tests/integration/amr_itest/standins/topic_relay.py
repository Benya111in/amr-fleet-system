"""
topic_relay: 필터 노드 대역 — in_topic 을 그대로 out_topic 으로 넘긴다.

Gazebo 백엔드에서 amr_localization/scan_filter_node 가 없을 때 scan → scan_filtered 를 잇는다
(safety_node·AMCL 이 scan_filtered 를 구독하므로). 거리/각도/아웃라이어 필터는 하지 않는다.
  파라미터: msg_type (예: sensor_msgs/msg/LaserScan), in_topic, out_topic
"""

from amr_itest.standins.common import RELIABLE, run, SENSOR, StandinNode
from rosidl_runtime_py.utilities import get_message


class TopicRelay(StandinNode):
    """in_topic → out_topic 그대로 전달."""

    def __init__(self):
        super().__init__('topic_relay')
        msg_type = get_message(str(self.param('msg_type', 'sensor_msgs/msg/LaserScan')))
        self.pub = self.create_publisher(msg_type, str(self.param('out_topic', 'scan_filtered')),
                                         RELIABLE)
        self.create_subscription(msg_type, str(self.param('in_topic', 'scan')),
                                 self.pub.publish, SENSOR)


def main(args=None) -> None:
    run(TopicRelay, args)


if __name__ == '__main__':
    main()
