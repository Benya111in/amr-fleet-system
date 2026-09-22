"""
cmd_relay: amr_navigation/velocity_profiler_node 대역 — cmd_vel_nav → cmd_vel_smoothed.

실제 노드는 사다리꼴/S-curve 프로파일·저크 제한·PID 를 하지만(components.md §5.3), 대역은
지연 없이 그대로 넘기고 50 Hz 로 마지막 명령을 재발행한다 (cmd_timeout 동안 새 명령이 없으면 0).
"""

from amr_itest.standins.common import RELIABLE, run, StandinNode
from geometry_msgs.msg import Twist


class CmdRelay(StandinNode):
    """cmd_vel_nav → cmd_vel_smoothed."""

    def __init__(self):
        super().__init__('velocity_profiler_node')
        self.timeout = self.param('cmd_timeout', 0.5)
        self.last = Twist()
        self.last_time = -1e9
        self.pub = self.create_publisher(Twist, 'cmd_vel_smoothed', RELIABLE)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_cmd, RELIABLE)
        self.create_timer(1.0 / self.param('rate', 50.0), self.republish)

    def on_cmd(self, msg: Twist) -> None:
        self.last = msg
        self.last_time = self.now()
        self.pub.publish(msg)

    def republish(self) -> None:
        if self.now() - self.last_time <= self.timeout:
            self.pub.publish(self.last)
        else:
            self.pub.publish(Twist())


def main(args=None) -> None:
    run(CmdRelay, args)


if __name__ == '__main__':
    main()
