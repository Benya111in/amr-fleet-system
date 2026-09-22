"""배터리 모델(시뮬레이션 BMS 대역) 시험: 방전·충전 적분, 적재 질량, 정지 조건, ROS 노드 메시지."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amr_behavior.battery_model import BatteryModel, BatteryParams  # noqa: E402


def test_idle_and_drive_drain_match_the_power_model():
    m = BatteryModel(BatteryParams(initial_percent=100.0))
    m.step(3600.0)                                  # 대기 1 h: 30 W / 480 Wh = 6.25 %
    assert m.percent == pytest.approx(100.0 - 6.25)
    m.linear = 1.0                                   # 1 m/s 주행 1 h: 90 W → 18.75 %
    m.step(3600.0)
    assert m.percent == pytest.approx(100.0 - 6.25 - 18.75)
    m.payload_mass = 25.0                            # 대형 적재: +25 W
    assert m.power_draw() == pytest.approx(30.0 + 60.0 + 25.0)
    m.angular = 0.5
    assert m.power_draw() == pytest.approx(30.0 + 60.0 + 25.0 + 5.0)


def test_charges_only_when_enabled_and_stationary():
    m = BatteryModel(BatteryParams(initial_percent=15.0))
    m.charging_enabled = True
    m.linear = 0.3                                   # 움직이면 충전하지 않는다
    assert not m.is_charging()
    before = m.percent
    m.step(60.0)
    assert m.percent < before
    m.linear = 0.0
    assert m.is_charging()
    m.step(36.0 * 60.0)                              # 480 W − 대기 30 W 로 36 min → +56.25 %
    moving = (30.0 + 60.0 * 0.3) * 60.0 / (480.0 * 3600.0) * 100.0   # 0.3 m/s 로 1 min
    assert m.percent == pytest.approx(before - moving + 56.25, abs=0.01)
    m.step(10 * 3600.0)
    assert m.percent == 100.0                        # 상한
    assert not m.is_charging()


def test_drain_scale_and_invalid_steps():
    m = BatteryModel(BatteryParams(initial_percent=50.0, drain_scale=60.0))
    m.step(60.0)                                     # 1 min × 60 배 = 대기 1 h
    assert m.percent == pytest.approx(50.0 - 6.25)
    for dt in (0.0, -1.0, math.nan):
        assert m.step(dt) == pytest.approx(50.0 - 6.25)
    m.step(1e9)
    assert m.percent == 0.0                          # 하한
    assert m.voltage() == pytest.approx(24.0 * 0.85)
    assert BatteryModel(BatteryParams(initial_percent=150.0)).percent == 100.0


def test_node_publishes_battery_state():
    rclpy = pytest.importorskip('rclpy')
    from amr_behavior.battery_model_node import BatteryModelNode
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import BatteryState
    rclpy.init()
    try:
        node = BatteryModelNode(parameter_overrides=[
            Parameter('initial_percent', value=42.0), Parameter('drain_scale', value=2.0)])
        msg = node.to_msg()
        assert msg.percentage == pytest.approx(0.42)
        assert msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        node.model.charging_enabled = True
        assert node.to_msg().power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        node.model.percent = 100.0
        node.model.charging_enabled = False
        assert node.to_msg().power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL
        node._on_timer()                             # 첫 주기: 적분 없이 발행
        node._on_timer()
        node.destroy_node()
    finally:
        rclpy.shutdown()
