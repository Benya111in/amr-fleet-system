"""
obstacle_truth_node — 동적 장애물 지면 진실 발행 (명세 4.7 추적기 평가·충돌 판정용, 시뮬레이션 전용).

보통 amr_simulation warehouse.launch.py 가 띄운다 (obstacle_truth:=true).

    ros2 run amr_simulation obstacle_truth_node.py --ros-args -p use_sim_time:=true
        --params-file <share/amr_simulation>/config/dynamic_obstacles.yaml

입력
    월드 SDF (world_file, 기본 share/amr_simulation/worlds/warehouse.sdf) — actor 궤적을 Gazebo 와 같은
    스플라인으로 sim time 에 보간한다 (amr_simulation/actor_trajectory.py).
    /sim/<model>/odom (nav_msgs/Odometry, gz OdometryPublisher 브리지, warehouse.launch.py) — 지게차·셔틀.
발행 (frame_id world = Gazebo 월드 원점 = map 원점과 같은 좌표, 시각 = sim time)
    /sim/dynamic_obstacles          visualization_msgs/MarkerArray   rate Hz
        장애물마다 마커 1개: ns = 이름, id = 고정 번호(0 부터, 이름 순서는 /sim/dynamic_obstacles/info),
        사람 = CYLINDER(지름 2·actor_radius), 차량 = CUBE(발자국 크기, 몸체 자세). pose = 발자국 중심. RViz 에 그대로 표시
    /sim/dynamic_obstacles/tracks   amr_msgs/TrackedObstacleArray    rate Hz
        추적기 출력과 같은 형식의 정답: track_id = 마커 id, position, velocity(월드), heading(이동 방향),
        confidence 1, is_dynamic true, time_to_collision inf
    /sim/dynamic_obstacles/info     std_msgs/String (JSON, transient_local, 1회)
        [{"id", "name", "kind": person|vehicle,
          "radius" | "footprint": [x_min, x_max, y_min, y_max], "height"}]
"""

import json
import math

import rclpy
from amr_msgs.msg import TrackedObstacle, TrackedObstacleArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from amr_simulation.dynamic_obstacles import (declare_obstacle_params,
                                              obstacle_set_from_params, yaw_from_quaternion)


def _stamp(t: float):
    from builtin_interfaces.msg import Time
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


class ObstacleTruthNode(Node):
    """actor·모델 지면 진실을 MarkerArray / TrackedObstacleArray 로 발행한다."""

    def __init__(self):
        super().__init__("obstacle_truth_node")
        p = declare_obstacle_params(self)
        rate = self.declare_parameter("rate", 50.0).value
        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.obs = obstacle_set_from_params(p)
        self.ids = {n: i for i, n in enumerate(self.obs.names())}
        for name in p["models"]:
            self.create_subscription(Odometry, p["model_odom_topic"].format(name=name),
                                     lambda m, n=name: self._on_odom(n, m), 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/sim/dynamic_obstacles", 10)
        self.pub_tracks = self.create_publisher(TrackedObstacleArray,
                                                "/sim/dynamic_obstacles/tracks", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        info = self.create_publisher(String, "/sim/dynamic_obstacles/info", latched)
        info.publish(String(data=json.dumps(self._info(), ensure_ascii=False)))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"actor {len(self.obs.actors)}개 + 모델 {len(self.obs.models)}개: "
            + ", ".join(f"{i}={n}" for n, i in self.ids.items()))

    def _info(self) -> list:
        out = [{"id": self.ids[a.name], "name": a.name, "kind": "person",
                "radius": self.obs.actor_radius, "height": self.obs.actor_height}
               for a in self.obs.actors]
        out += [{"id": self.ids[n], "name": n, "kind": "vehicle", "footprint": list(m.bounds),
                 "height": m.height} for n, m in self.obs.models.items()]
        return out

    def _on_odom(self, name: str, m: Odometry) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pp, tw = m.pose.pose, m.twist.twist
        self.obs.update_model(name, t, pp.position.x, pp.position.y,
                              yaw_from_quaternion(pp.orientation),
                              tw.linear.x, tw.linear.y, tw.angular.z)

    def _tick(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9
        stamp = _stamp(t)
        markers, tracks = MarkerArray(), TrackedObstacleArray()
        tracks.header.stamp, tracks.header.frame_id = stamp, self.frame_id
        for s in self.obs.states(t):
            mk = Marker()
            mk.header.stamp, mk.header.frame_id = stamp, self.frame_id
            mk.ns, mk.id, mk.action = s.name, self.ids[s.name], Marker.ADD
            cx, cy = s.x, s.y
            if s.kind == "person":
                mk.type = Marker.CYLINDER
                mk.scale.x = mk.scale.y = 2.0 * s.radius
                mk.color.r, mk.color.g, mk.color.b = 0.7, 1.0, 0.1
            else:
                x0, x1, y0, y1 = s.bounds
                ox, oy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
                cx += math.cos(s.yaw) * ox - math.sin(s.yaw) * oy
                cy += math.sin(s.yaw) * ox + math.cos(s.yaw) * oy
                mk.type = Marker.CUBE
                mk.scale.x, mk.scale.y = x1 - x0, y1 - y0
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.6, 0.1
            mk.scale.z = s.height
            mk.color.a = 0.6
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = cx, cy, 0.5 * s.height
            mk.pose.orientation.z, mk.pose.orientation.w = math.sin(s.yaw / 2), math.cos(s.yaw / 2)
            markers.markers.append(mk)

            tr = TrackedObstacle()
            tr.header = tracks.header
            tr.track_id = self.ids[s.name]
            tr.position.x, tr.position.y = s.x, s.y
            tr.velocity.x, tr.velocity.y = s.vx, s.vy
            tr.heading = float(s.heading)
            tr.confidence, tr.is_dynamic, tr.time_to_collision = 1.0, True, math.inf
            tracks.obstacles.append(tr)
        self.pub_markers.publish(markers)
        self.pub_tracks.publish(tracks)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
