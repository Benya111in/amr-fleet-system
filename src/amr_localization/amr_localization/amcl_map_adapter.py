"""
amcl_map_adapter: nav2_amcl 의 반 셀 좌표 편향을 보정한 맵을 AMCL 전용 토픽으로 재발행한다.

nav2_amcl (humble 1.1.x, map/map.hpp) 은 월드 좌표 → 셀 변환을 반올림으로 한다.
    MAP_GXWX(x) = floor((x − o_x)/s + ½)          (convertMap 의 origin 보정을 풀어 쓴 식)
즉 AMCL 은 셀 i 가 [o + (i − ½)s, o + (i + ½)s) 를 덮는다고 보지만, nav_msgs/OccupancyGrid 규약은
[o + i·s, o + (i + 1)s) 이다. 그래서 AMCL 내부 지도(거리장 포함)는 모든 구조물이 (−s/2, −s/2)
만큼 옮겨진 것과 같고, 추정 자세도 같은 만큼 치우친다 (s = 0.05 m → 대각 3.5 cm, 정지 3 cm 목표와
같은 크기). 셀 → 월드 역변환 MAP_WXGX(i) = o + i·s 도 셀 모서리를 준다.
원점을 (+s/2, +s/2) 옮긴 사본을 AMCL 에만 주면 두 변환이 모두 규약과 일치한다:
    floor((x − o − s/2)/s + ½) = floor((x − o)/s),    o + s/2 + i·s = 셀 i 중심.
map_server 의 /map 과 다른 구독자(costmap, 추적기, kidnap_monitor)는 그대로 둔다.

인터페이스
  Sub  input_topic  (기본 /map)       nav_msgs/OccupancyGrid  transient_local
  Pub  output_topic (기본 map_amcl)   nav_msgs/OccupancyGrid  transient_local (AMCL map_topic)
"""

import copy
import math
from typing import Tuple

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def half_cell_offset(resolution: float) -> Tuple[float, float]:
    """
    AMCL 반올림 규약을 상쇄하는 원점 이동량 (+s/2, +s/2) [m] (map 프레임 축).

    AMCL(convertMap)은 원점 yaw 를 쓰지 않고 격자 축 = map 축으로 보므로 회전 없이 더한다.
    """
    h = 0.5 * resolution
    return h, h


def yaw_of(q) -> float:
    """쿼터니언 → yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def adapt(msg: OccupancyGrid) -> OccupancyGrid:
    """원점만 반 셀 옮긴 사본 (데이터·해상도·크기는 그대로)."""
    out = OccupancyGrid()
    out.header = msg.header
    out.info = copy.deepcopy(msg.info)     # 입력 메시지는 바꾸지 않는다
    out.data = msg.data
    dx, dy = half_cell_offset(msg.info.resolution)
    out.info.origin.position.x = msg.info.origin.position.x + dx
    out.info.origin.position.y = msg.info.origin.position.y + dy
    return out


class AmclMapAdapter(Node):
    """/map → map_amcl (원점 반 셀 보정) 재발행 노드."""

    def __init__(self, **kwargs) -> None:
        super().__init__('amcl_map_adapter', **kwargs)
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', 'map_amcl')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('output_topic').value, latched)
        self.create_subscription(OccupancyGrid, self.get_parameter('input_topic').value,
                                 self.on_map, latched)
        self.maps = 0

    def on_map(self, msg: OccupancyGrid) -> None:
        if abs(yaw_of(msg.info.origin.orientation)) > 1e-6:
            self.get_logger().warn('map origin yaw ≠ 0: AMCL 은 원점 회전을 무시한다 (맵을 축 정렬로 저장할 것)')
        out = adapt(msg)
        self.pub.publish(out)
        self.maps += 1
        self.get_logger().info(
            f'map {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m → '
            f'{self.pub.topic_name}: origin ({msg.info.origin.position.x:.3f}, '
            f'{msg.info.origin.position.y:.3f}) → ({out.info.origin.position.x:.3f}, '
            f'{out.info.origin.position.y:.3f}) (AMCL 반 셀 보정)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AmclMapAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
