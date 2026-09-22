"""
collision_monitor_node — 로봇 ↔ 동적 장애물 최소 거리와 접촉(충돌) 횟수 (명세 4.7 "30회 충돌 0건" 판정).

시뮬레이션 전용. 보통 amr_simulation warehouse.launch.py 가 띄운다 (collision_monitor:=true, monitor_robots).

    ros2 run amr_simulation collision_monitor_node.py --ros-args -p use_sim_time:=true
        -p robots:="[amr_01]" --params-file <share/amr_simulation>/config/dynamic_obstacles.yaml

판정 (결정)
    로봇 발자국 = config/robot_params.yaml footprint_length × footprint_width 직사각형
    (ground_truth/odom 자세).
    장애물 = 사람(원, actor_radius), 지게차·셔틀(직사각형 발자국), 다른 로봇(같은 직사각형 — robots 가 2대 이상일 때).
    부호 거리 < 0 (발자국이 겹침) = 접촉. 쌍마다 한 번 접촉하면 거리가 contact_release 이상으로 벌어질 때까지 같은 사건으로
    센다 (스치며 떨리는 경계에서 중복 계산 방지). actor 는 물리 충돌체가 없어 로봇이 통과하므로 이 기하 판정이 곧 충돌 판정이다.
    actor 는 로봇 오도메트리 시각에 SDF 궤적을 직접 보간하므로 시각 정렬 오차가 없다 (amr_simulation/dynamic_obstacles.py).
    적재물이 차체 밖으로 튀어나오는 경우(대형 박스 폭 0.50 > 차체 0.40)는 footprint_padding 으로 반영한다.
입력
    /<robot>/ground_truth/odom (nav_msgs/Odometry, 50 Hz), /sim/<model>/odom, 월드 SDF (actor 궤적)
발행
    /<robot>/collision_monitor/min_distance   std_msgs/Float64   오도메트리마다 (가장 가까운 장애물까지 부호 거리 [m])
    /<robot>/collision_monitor/contacts       std_msgs/UInt32    접촉 사건 누적 수 (변할 때 + 1 Hz)
    /sim/collision_monitor/events             std_msgs/String    접촉 사건마다 JSON
                                              {t, robot, obstacle, distance}
    /sim/collision_monitor/summary            std_msgs/String    1 Hz JSON {robot: {contacts,
                                              min_distance, current, nearest, per_obstacle_min}}
서비스
    /sim/collision_monitor/reset              std_srvs/Trigger   누적값 초기화 (시험 1회마다)
"""

import json
import math
import os

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String, UInt32
from std_srvs.srv import Trigger

from amr_simulation.dynamic_obstacles import (declare_obstacle_params,
                                              obstacle_set_from_params, yaw_from_quaternion)
from amr_simulation.footprint import Rect, rect_circle, rect_rect


def _default_config_dir() -> str:
    ros_ws = os.environ.get("ROS_WS", "")
    return os.path.join(ros_ws, "config") if ros_ws else "/ros2_ws/config"


class CollisionMonitorNode(Node):
    """로봇마다 최소 부호 거리와 접촉 사건을 센다."""

    def __init__(self):
        super().__init__("collision_monitor_node")
        p = declare_obstacle_params(self)
        self.robots = list(self.declare_parameter("robots", ["amr_01"]).value)
        config_dir = self.declare_parameter("config_dir", _default_config_dir()).value
        pad = self.declare_parameter("footprint_padding", 0.0).value
        self.release = self.declare_parameter("contact_release", 0.05).value
        with open(os.path.join(config_dir, "robot_params.yaml"), encoding="utf-8") as f:
            r = yaml.safe_load(f)["/**"]["ros__parameters"]["robot"]
        hl, hw = r["footprint_length"] / 2.0 + pad, r["footprint_width"] / 2.0 + pad
        self.bounds = (-hl, hl, -hw, hw)
        self.obs = obstacle_set_from_params(p)
        self.pose = {}                     # robot → (t, x, y, yaw)
        self.reset_stats()

        for name in p["models"]:
            self.create_subscription(Odometry, p["model_odom_topic"].format(name=name),
                                     lambda m, n=name: self._on_model(n, m), 10)
        self.pub_min, self.pub_cnt = {}, {}
        for rb in self.robots:
            base = f"/{rb}/collision_monitor"
            self.pub_min[rb] = self.create_publisher(Float64, f"{base}/min_distance", 10)
            self.pub_cnt[rb] = self.create_publisher(UInt32, f"{base}/contacts", 10)
            self.create_subscription(Odometry, f"/{rb}/ground_truth/odom",
                                     lambda m, n=rb: self._on_robot(n, m), 50)
        self.pub_events = self.create_publisher(String, "/sim/collision_monitor/events", 50)
        self.pub_summary = self.create_publisher(String, "/sim/collision_monitor/summary", 10)
        self.create_service(Trigger, "/sim/collision_monitor/reset", self._on_reset)
        self.create_timer(1.0, self._summary)
        self.get_logger().info(f"로봇 {self.robots}, 장애물 {len(self.obs.names())}개, "
                               f"발자국 {self.bounds}, 접촉 해제 {self.release} m")

    def reset_stats(self):
        self.contacts = {rb: 0 for rb in self.robots}
        self.in_contact = {}               # (robot, obstacle) → bool
        self.min_run = {rb: math.inf for rb in self.robots}
        self.per_obstacle_min = {rb: {} for rb in self.robots}
        self.last = {rb: (math.inf, "") for rb in self.robots}

    def _on_reset(self, _req, res):
        self.reset_stats()
        res.success, res.message = True, "reset"
        return res

    def _on_model(self, name: str, m: Odometry) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pp, tw = m.pose.pose, m.twist.twist
        self.obs.update_model(name, t, pp.position.x, pp.position.y,
                              yaw_from_quaternion(pp.orientation),
                              tw.linear.x, tw.linear.y, tw.angular.z)

    def _on_robot(self, rb: str, m: Odometry) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pp = m.pose.pose
        me = Rect(pp.position.x, pp.position.y, yaw_from_quaternion(pp.orientation), *self.bounds)
        self.pose[rb] = me
        dists = []
        for s in self.obs.states(t):
            if s.kind == "person":
                d = rect_circle(me, s.x, s.y, s.radius)
            else:
                d = rect_rect(me, s.rect())
            dists.append((d, s.name))
        for other, rect in self.pose.items():
            if other != rb:
                dists.append((rect_rect(me, rect), other))
        if not dists:
            return
        for d, name in dists:
            key = (rb, name)
            prev = self.per_obstacle_min[rb].get(name, math.inf)
            self.per_obstacle_min[rb][name] = min(prev, d)
            if d < 0.0 and not self.in_contact.get(key, False):
                self.in_contact[key] = True
                self.contacts[rb] += 1
                ev = {"t": round(t, 3), "robot": rb, "obstacle": name, "distance": round(d, 4)}
                self.pub_events.publish(String(data=json.dumps(ev)))
                self.pub_cnt[rb].publish(UInt32(data=self.contacts[rb]))
                self.get_logger().warn(f"접촉: {rb} ↔ {name} (부호 거리 {d:.3f} m, t={t:.2f})")
            elif d >= self.release:
                self.in_contact[key] = False
        dmin, nearest = min(dists)
        self.min_run[rb] = min(self.min_run[rb], dmin)
        self.last[rb] = (dmin, nearest)
        self.pub_min[rb].publish(Float64(data=float(dmin)))

    def _summary(self) -> None:
        out = {}
        for rb in self.robots:
            per = {k: round(v, 3) for k, v in sorted(self.per_obstacle_min[rb].items())}
            out[rb] = {"contacts": self.contacts[rb], "min_distance": round(self.min_run[rb], 3),
                       "current": round(self.last[rb][0], 3), "nearest": self.last[rb][1],
                       "per_obstacle_min": per}
            self.pub_cnt[rb].publish(UInt32(data=self.contacts[rb]))
        self.pub_summary.publish(String(data=json.dumps(out)))


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl-C / 런치 종료(SIGINT)
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
