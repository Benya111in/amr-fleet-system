"""
시험용 서버 프로세스 진입점 (launch 시험이 죽였다 살린다).

python3 -m amr_fleet.agent.manager_shim [--observe-s S] [--epoch E] [--leader-id ID] [--log-dir D]
                                        [--period-s P] [--no-manager] [--ros-args ...]

b4-wire 부터 하트비트 · 관찰자 복귀는 fleet_manager_node 자체에 있다 (heartbeat.enabled, amr_fleet/heartbeat.py,
p3.md §10). 이 진입점은 그 실제 노드를 하트비트를 켠 파라미터로 띄울 뿐이다 — 그래서 P3 launch 시험 ①–④b 가
실제 관리자 하트비트를 거친다. (P3 에서는 관리자 파일이 소유 밖이라 여기서 하트비트를 따로 냈다.)

--no-manager: 관리자 없이 하트비트만 내는 HeartbeatNode (옛 권위자 흉내 — launch ③). 이 경우에만 아래
HeartbeatNode 를 쓴다. 관찰자 규칙은 heartbeat.py 와 같다 (epoch = max(--epoch, 비컨 최댓값 + 1)).
"""

from __future__ import annotations

import argparse
import sys

from amr_fleet.heartbeat import ManagerHeartbeat
from amr_msgs.msg import AgentBeacon, FleetHeartbeat
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

HB_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
BEACON_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class HeartbeatNode(Node):
    """관리자 없는 하트비트 (옛 권위자 흉내, --no-manager). 규칙은 heartbeat.ManagerHeartbeat 그대로."""

    def __init__(self, leader_id: str, epoch: int, period_s: float, observe_s: float):
        super().__init__('fleet_heartbeat', namespace='/fleet')
        self.leader_id = leader_id
        self.hb = ManagerHeartbeat(max(1, int(epoch)), float(observe_s))
        self._pub = self.create_publisher(FleetHeartbeat, '/fleet/heartbeat', HB_QOS)
        self.create_subscription(AgentBeacon, '/fleet_p2p/beacon',
                                 lambda m: self.hb.on_beacon(int(m.fence_epoch)), BEACON_QOS)
        self.create_timer(float(period_s), self._tick)

    def _tick(self) -> None:
        beat = self.hb.tick(self.get_clock().now().nanoseconds * 1e-9)
        if beat is None:
            return
        msg = FleetHeartbeat()
        msg.stamp = self.get_clock().now().to_msg()
        msg.leader_id = self.leader_id
        msg.epoch = int(beat.epoch)
        msg.seq = int(beat.seq)
        msg.role = int(beat.role)
        self._pub.publish(msg)


def main(argv=None):
    """python3 -m amr_fleet.agent.manager_shim."""
    argv = list(sys.argv if argv is None else argv)
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--observe-s', type=float, default=0.0)
    ap.add_argument('--epoch', type=int, default=1)
    ap.add_argument('--leader-id', default='/fleet/fleet_manager_node')
    ap.add_argument('--log-dir', default='')
    ap.add_argument('--period-s', type=float, default=0.5)
    ap.add_argument('--no-manager', action='store_true',
                    help='하트비트만 (옛 권위자 흉내 등 — fleet_manager_node 를 띄우지 않는다)')
    args = ap.parse_args(remove_ros_args(argv)[1:])
    rclpy.init(args=argv)
    if args.no_manager:
        node = HeartbeatNode(args.leader_id, args.epoch, args.period_s, args.observe_s)
    else:
        from amr_fleet.fleet_manager_node import FleetManagerNode
        overrides = [Parameter('epoch', value=int(args.epoch)),
                     Parameter('leader_id', value=args.leader_id),
                     Parameter('heartbeat.enabled', value=True),
                     Parameter('heartbeat.period_s', value=float(args.period_s)),
                     Parameter('heartbeat.observe_s', value=float(args.observe_s))]
        if args.log_dir:
            overrides.append(Parameter('log_dir', value=args.log_dir))
        node = FleetManagerNode(parameter_overrides=overrides)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
