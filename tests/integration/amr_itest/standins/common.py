"""대역 노드 공통부: 파라미터 읽기(중첩 키), QoS, 실행 진입점."""

import sys
from typing import Any, Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                      reliability=ReliabilityPolicy.RELIABLE)
SENSOR = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                    reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class StandinNode(Node):
    """
    대역 노드 베이스.

    config/robot_params.yaml, config/sensors.yaml 을 --params-file 로 그대로 받는다 (실제 노드와
    같은 설정 경로). 선언하지 않은 중첩 키도 읽을 수 있게 overrides 자동 선언을 켠다.
    """

    def __init__(self, name: str):
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.get_logger().info('대역(stand-in) 노드 — 실제 노드가 머지되면 하네스가 자동 교체한다')

    def param(self, name: str, default: Any) -> Any:
        """파라미터 값 (없으면 default; 타입은 default 에 맞춘다)."""
        if not self.has_parameter(name):
            return default
        value = self.get_parameter(name).value
        if value is None:
            return default
        if isinstance(default, bool):
            return bool(value)
        if isinstance(default, float):
            return float(value)
        if isinstance(default, int) and not isinstance(value, float):
            return int(value)
        return value

    def now(self) -> float:
        """
        노드 시계 [s] — use_sim_time 이면 /clock.

        대역의 모든 시간 판정(명령 타임아웃·주행 시간·센서 신선도)은 이 시계로 한다. 고부하
        호스트에서 Gazebo RTF 가 0.1 아래로 떨어져도 sim time 기준 동작이 실제 노드와 같게.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def frame(self, name: str) -> str:
        """frame_prefix 를 붙인 프레임 이름 ('map' 제외)."""
        prefix = str(self.param('frame_prefix', ''))
        return name if name == 'map' or not prefix else prefix + name


def run(factory: Callable[[], Node], args=None) -> None:
    """노드 실행: rclpy 초기화 → spin → 정리 (SIGINT/외부 종료는 정상 종료로 취급)."""
    rclpy.init(args=args if args is not None else sys.argv)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
