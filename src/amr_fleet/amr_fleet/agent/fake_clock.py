"""
시뮬레이터 없는 시험용 /clock 발행자 (python3 -m amr_fleet.agent.fake_clock).

sim 시각 = start_s + rtf × (벽시계 경과). 노드들이 use_sim_time 으로 같은 sim 축을 쓰게 한다 — CommEmulator 의
분할 · 단절 일정과 버스트 상태열이 sim 시각 0 근처에서 시작하도록 (벽시계 epoch 1.8e9 s 를 쓰면 버스트 경계를
0 부터 수억 개 만든다 — P0 리뷰 지적).
"""

from __future__ import annotations

import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock as ClockMsg


class FakeClock(Node):
    """벽시계에 비례하는 /clock."""

    def __init__(self, **kwargs):
        super().__init__('fake_clock', **kwargs)
        self.declare_parameter('start_s', 100.0)
        self.declare_parameter('rtf', 1.0)
        self.declare_parameter('rate_hz', 100.0)
        self._start = float(self.get_parameter('start_s').value)
        self._rtf = float(self.get_parameter('rtf').value)
        self._t0 = time.monotonic()
        # RELIABLE 발행자는 BEST_EFFORT · RELIABLE 구독자 모두와 맞는다 (rclcpp ClockQoS 는 BEST_EFFORT)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(ClockMsg, '/clock', qos)
        period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_timer(period, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def sim_now(self) -> float:
        """지금 sim 시각 [s]."""
        return self._start + self._rtf * (time.monotonic() - self._t0)

    def _tick(self) -> None:
        t = self.sim_now()
        msg = ClockMsg()
        msg.clock.sec = int(t)
        msg.clock.nanosec = int((t - int(t)) * 1e9)
        self._pub.publish(msg)


def main(args=None):
    """python3 -m amr_fleet.agent.fake_clock."""
    rclpy.init(args=args)
    node = FakeClock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass                                  # SIGINT · SIGTERM 로 컨텍스트가 먼저 닫힌 경우
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
