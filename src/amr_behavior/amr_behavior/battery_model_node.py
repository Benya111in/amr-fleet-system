"""
battery_model_node — 시뮬레이션 배터리 (battery_model.BatteryModel 을 ROS 에 연결).

Sub  odometry/filtered (nav_msgs/Odometry)   속도 → 소비 전력
     payload/mass (std_msgs/Float32, latched) 적재 질량 → 주행 전력 (계약 C5)
     charging/enable (std_msgs/Bool, latched) task_executor_node 가 충전소에 도킹해 켠다
Pub  battery_state (sensor_msgs/BatteryState, publish_rate_hz)  percentage 0~1 (REP-147),
     task_executor_node 의 IsBatteryOk / 수락 정책과 fleet_adapter_node 가 구독한다

실기에서는 BMS 드라이버가 battery_state 를 내므로 이 노드를 띄우지 않는다
(behavior.launch.py start_battery_model:=false).
"""

from __future__ import annotations

import time

from amr_behavior.battery_model import BatteryModel, BatteryParams
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32

LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


class BatteryModelNode(Node):
    """소비 전력 적분 → battery_state."""

    def __init__(self, **kwargs):
        super().__init__('battery_model_node', **kwargs)
        defaults = BatteryParams()
        values = {}
        for name in BatteryParams.__dataclass_fields__:
            values[name] = float(self.declare_parameter(name, getattr(defaults, name)).value)
        self.model = BatteryModel(BatteryParams(**values))
        self.odom_timeout = float(self.declare_parameter('odom_timeout', 1.0).value)
        rate = float(self.declare_parameter('publish_rate_hz', 1.0).value)
        self._last_odom = None
        self._last_step = None
        self._pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.create_subscription(Odometry, 'odometry/filtered', self._on_odom, 10)
        self.create_subscription(Float32, 'payload/mass', self._on_mass, LATCHED_QOS)
        self.create_subscription(Bool, 'charging/enable', self._on_charging, LATCHED_QOS)
        self.create_timer(1.0 / max(rate, 0.1), self._on_timer)
        self.get_logger().info(
            f'배터리 모델: {self.model.params.capacity_wh:.0f} Wh, 시작 {self.model.percent:.1f} %, '
            f'방전 배율 {self.model.params.drain_scale:g}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        self.model.linear = float(msg.twist.twist.linear.x)
        self.model.angular = float(msg.twist.twist.angular.z)
        self._last_odom = time.monotonic()

    def _on_mass(self, msg: Float32) -> None:
        self.model.payload_mass = float(msg.data)

    def _on_charging(self, msg: Bool) -> None:
        if bool(msg.data) != self.model.charging_enabled:
            self.get_logger().info(
                f'charging/enable = {bool(msg.data)} ({self.model.percent:.1f} %)')
        self.model.charging_enabled = bool(msg.data)

    def _on_timer(self) -> None:
        now = self._now()
        if self._last_odom is not None and time.monotonic() - self._last_odom > self.odom_timeout:
            self.model.linear = 0.0        # 오도메트리가 끊기면 정지로 본다 (대기 전력만)
            self.model.angular = 0.0
        if self._last_step is not None:
            self.model.step(now - self._last_step)
        self._last_step = now
        self._pub.publish(self.to_msg())

    def to_msg(self) -> BatteryState:
        """현재 상태 → sensor_msgs/BatteryState."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(self.model.percent / 100.0)
        msg.voltage = float(self.model.voltage())
        msg.capacity = float(self.model.params.capacity_wh / self.model.params.nominal_voltage)
        msg.design_capacity = msg.capacity
        msg.charge = float(msg.capacity * msg.percentage)
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        if self.model.is_charging():
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self.model.percent >= 100.0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
