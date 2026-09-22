"""
detection_marker_node — 인식 객체 RViz 마커 (components.md §3.4 / §5.4, 명세 4.6·8장).

Sub  perception/detected_objects (amr_msgs/DetectedObjectArray)
Pub  perception/markers (visualization_msgs/MarkerArray): 객체마다 CUBE(클래스 색·크기) +
     TEXT_VIEW_FACING "Class: box, Conf: 0.92, Dist: 1.5m". 메시지마다 DELETEALL 로 시작해
     사라진 객체의 마커가 남지 않게 하고, lifetime 으로 입력이 끊겨도 지워지게 한다.
"""

from __future__ import annotations

from amr_msgs.msg import DetectedObjectArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .markers import marker_spec


def build_marker_array(msg: DetectedObjectArray, lifetime: float, text_height: float,
                       capitalize: bool = False, ns: str = 'detections') -> MarkerArray:
    """검출 객체 배열 → MarkerArray (DELETEALL + 객체당 CUBE·TEXT)."""
    arr = MarkerArray()
    clear = Marker()
    clear.header = msg.header
    clear.ns = ns
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)
    life = Duration(seconds=lifetime).to_msg()
    for i, obj in enumerate(msg.objects):
        pose = obj.pose_3d.pose
        header = obj.pose_3d.header if obj.pose_3d.header.frame_id else msg.header
        spec = marker_spec(obj.class_name, obj.confidence, obj.distance, pose.position.z,
                           text_height=text_height, capitalize=capitalize)
        cube = Marker()
        cube.header = header
        cube.ns = ns
        cube.id = 2 * i
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = pose.position.x
        cube.pose.position.y = pose.position.y
        cube.pose.position.z = spec.cube_z
        cube.pose.orientation.w = 1.0
        cube.scale.x, cube.scale.y, cube.scale.z = spec.scale
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = spec.color
        cube.lifetime = life
        arr.markers.append(cube)
        text = Marker()
        text.header = header
        text.ns = ns
        text.id = 2 * i + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.position.x
        text.pose.position.y = pose.position.y
        text.pose.position.z = spec.text_z
        text.pose.orientation.w = 1.0
        text.scale.z = text_height
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = spec.text
        text.lifetime = life
        arr.markers.append(text)
    return arr


class DetectionMarkerNode(Node):
    """detected_objects → markers."""

    def __init__(self, **kwargs):
        super().__init__('detection_marker_node', **kwargs)
        self.declare_parameter('lifetime', 1.0)
        self.declare_parameter('text_height', 0.25)
        self.declare_parameter('capitalize_class', True)
        self.declare_parameter('topics.input', 'perception/detected_objects')
        self.declare_parameter('topics.markers', 'perception/markers')
        gp = self.get_parameter
        self.lifetime = float(gp('lifetime').value)
        self.text_height = float(gp('text_height').value)
        self.capitalize = bool(gp('capitalize_class').value)
        self.pub = self.create_publisher(MarkerArray, gp('topics.markers').value, 10)
        self.create_subscription(DetectedObjectArray, gp('topics.input').value, self.on_objects,
                                 10)

    def on_objects(self, msg: DetectedObjectArray) -> None:
        self.pub.publish(build_marker_array(msg, self.lifetime, self.text_height,
                                            self.capitalize))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
