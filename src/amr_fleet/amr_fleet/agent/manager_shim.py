"""
시험용 서버 프로세스: 원본 fleet_manager_node + /fleet/heartbeat 발행자를 **한 프로세스**에 묶는다.

python3 -m amr_fleet.agent.manager_shim [--observe-s S] [--epoch E] [--leader-id ID] [--log-dir D]
                                        [--period-s P] [--ros-args ...]

왜: fleet_manager_node.py 는 이 과제의 소유 경로 밖이라 하트비트를 넣지 못했다 (p3.md §7 요청). 하트비트가
서버의 생존을 뜻하려면 서버와 같은 프로세스에서 나와야 한다 — 프로세스를 SIGKILL 하면 둘 다 멈춘다.
fleet_manager_node 는 수정 없이 import 해서 띄운다.

관찰자 복귀 (보고서 145–158행): --observe-s > 0 이면 먼저 관찰자(ROLE_OBSERVER)로 /fleet_p2p/beacon 을 그만큼
(sim 초) 듣고 로봇들이 본 최고 fence_epoch 를 모아 epoch = max(--epoch, 그 값 + 1) 로 정한 뒤에야
fleet_manager_node 를 그 epoch 로 띄우고 ACTIVE 하트비트를 낸다. 옛 epoch 로 명령하지 않게 하는 장치다.
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from amr_msgs.msg import AgentBeacon, FleetHeartbeat
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

ROLE_ACTIVE = FleetHeartbeat.ROLE_ACTIVE
ROLE_OBSERVER = FleetHeartbeat.ROLE_OBSERVER
HB_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
BEACON_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class HeartbeatNode(Node):
    """sim 주기 하트비트 + (관찰 중) 비컨에서 로봇들이 본 최고 epoch 수집."""

    def __init__(self, leader_id: str, epoch: int, period_s: float, observe_s: float):
        super().__init__('fleet_heartbeat', namespace='/fleet')
        self.leader_id = leader_id
        self.min_epoch = max(1, int(epoch))
        self.epoch = self.min_epoch
        self.role = ROLE_OBSERVER if observe_s > 0 else ROLE_ACTIVE
        self.observe_s = float(observe_s)
        self.max_seen = 0
        self.seq = 0
        self.started: Optional[float] = None
        self._pub = self.create_publisher(FleetHeartbeat, '/fleet/heartbeat', HB_QOS)
        self.create_subscription(AgentBeacon, '/fleet_p2p/beacon', self._on_beacon, BEACON_QOS)
        self.create_timer(float(period_s), self._tick)
        self.on_active = None

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_beacon(self, msg: AgentBeacon) -> None:
        self.max_seen = max(self.max_seen, int(msg.fence_epoch))

    def _tick(self) -> None:
        now = self._now()
        if now <= 0.0:
            return
        if self.started is None:
            self.started = now
        if self.role == ROLE_OBSERVER:
            self.epoch = max(self.min_epoch, self.max_seen + 1)
            if now - self.started >= self.observe_s:
                self.role = ROLE_ACTIVE      # seq 는 이어 간다 — 같은 epoch 안에서 되돌리면 감시가 무시한다
                self.get_logger().info(
                    f'관찰 끝: 로봇이 본 최고 epoch {self.max_seen} → epoch {self.epoch} 로 시작')
                if self.on_active is not None:
                    self.on_active(self.epoch)
        self.seq += 1
        msg = FleetHeartbeat()
        msg.stamp = self.get_clock().now().to_msg()
        msg.leader_id = self.leader_id
        msg.epoch = int(self.epoch)
        msg.seq = int(self.seq)
        msg.role = int(self.role)
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
    hb = HeartbeatNode(args.leader_id, args.epoch, args.period_s, args.observe_s)
    ex = SingleThreadedExecutor()
    ex.add_node(hb)
    holder = {}

    def start_manager(epoch: int) -> None:
        if args.no_manager or 'fm' in holder:
            return
        from amr_fleet.fleet_manager_node import FleetManagerNode
        overrides = [Parameter('epoch', value=int(epoch)),
                     Parameter('leader_id', value=args.leader_id)]
        if args.log_dir:
            overrides.append(Parameter('log_dir', value=args.log_dir))
        holder['fm'] = FleetManagerNode(parameter_overrides=overrides)
        ex.add_node(holder['fm'])

    hb.on_active = start_manager
    if hb.role == ROLE_ACTIVE:
        start_manager(hb.epoch)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        for n in list(holder.values()) + [hb]:
            n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
