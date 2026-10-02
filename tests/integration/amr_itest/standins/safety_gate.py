"""
safety_gate: amr_perception/safety_node 대역 (components.md §5.4 인터페이스 그대로).

  Sub cmd_vel_smoothed (Twist), scan_filtered (LaserScan, best-effort 호환)
      estop, /fleet/estop (Bool, transient_local — 대시보드 E-stop)
  Pub cmd_vel (Twist, 50 Hz + 상태 변화 즉시), safety/estop_active (Bool, latched),
      safety/zone (UInt8, latched: 0 CLEAR · 1 WARNING · 2 CRITICAL · 3 STOP)
  SrvS safety/reset_estop (std_srvs/Trigger) — 버튼 E-stop 래치 해제 (입력이 모두 false 일 때만)

거리 기준은 풋프린트 가장자리 (robot_params.yaml safety.distance_reference). 스캔 점을
라이다 extrinsic 으로 base_link 에 옮겨 차체 사각형까지의 최단 거리를 잰다.
  ≤ emergency_stop_distance (0.3 m)  → 정지, 여유가 release_distance(0.5 m) 를 넘으면 자동 해제
  ≤ critical_zone_distance (0.5 m)   → |v| ≤ critical_zone_max_speed (0.2 m/s)
  ≤ warning_zone_distance (1.0 m)    → |v| ≤ warning_zone_max_speed (0.5 m/s)
스캔이 sensor_timeouts.lidar 동안 없으면 정지. 실제 safety_node 의 TTC·센서별 타임아웃·저속 모드는
구현하지 않는다 (대역 범위 밖).
"""

import math

from amr_itest import kinematics as km
from amr_itest.standins.common import LATCHED, RELIABLE, run, SENSOR, StandinNode
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt8
from std_srvs.srv import Trigger

CLEAR, WARNING, CRITICAL, STOP = 0, 1, 2, 3


class SafetyGate(StandinNode):
    """존 감속 + E-stop 래치 게이트."""

    def __init__(self):
        super().__init__('safety_node')
        self.stop_d = self.param('safety.emergency_stop_distance', 0.30)
        self.crit_d = self.param('safety.critical_zone_distance', 0.50)
        self.warn_d = self.param('safety.warning_zone_distance', 1.00)
        self.release_d = self.param('release_distance', 0.50)
        self.crit_v = self.param('safety.critical_zone_max_speed', 0.2)
        self.warn_v = self.param('safety.warning_zone_max_speed', 0.5)
        self.scan_timeout = self.param('safety.sensor_timeouts.lidar', 0.3)
        self.cmd_timeout = self.param('cmd_timeout', 0.5)
        self.length = self.param('robot.footprint_length', 0.60)
        self.width = self.param('robot.footprint_width', 0.40)
        self.ext = (self.param('lidar.extrinsic.x', 0.15), self.param('lidar.extrinsic.y', 0.0),
                    self.param('lidar.extrinsic.yaw', 0.0))

        self.cmd = Twist()
        self.cmd_time = -math.inf
        self.scan_time = self.now()      # 기동 직후 유예 1 주기
        self.clearance = math.inf
        self.zone = CLEAR
        self.proximity_stop = False
        self.button = {'robot': False, 'fleet': False}
        self.button_latched = False

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', RELIABLE)
        self.pub_active = self.create_publisher(Bool, 'safety/estop_active', LATCHED)
        self.pub_zone = self.create_publisher(UInt8, 'safety/zone', LATCHED)
        self.create_subscription(Twist, 'cmd_vel_smoothed', self.on_cmd, RELIABLE)
        self.create_subscription(LaserScan, 'scan_filtered', self.on_scan, SENSOR)
        self.create_subscription(Bool, 'estop', lambda m: self.on_button('robot', m), LATCHED)
        self.create_subscription(Bool, '/fleet/estop', lambda m: self.on_button('fleet', m),
                                 LATCHED)
        self.create_service(Trigger, 'safety/reset_estop', self.on_reset)
        self.create_timer(1.0 / self.param('rate', 50.0), self.publish)
        self._last_active = None
        self._last_zone = None
        self.publish_state()

    # --- 입력 ---
    def on_cmd(self, msg: Twist) -> None:
        self.cmd = msg
        self.cmd_time = self.now()
        self.publish()

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_time = self.now()
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=float)
        ranges[(ranges < msg.range_min) | (ranges > msg.range_max)] = np.inf
        pts = km.scan_to_points(ranges, angles, self.ext)
        self.clearance = km.footprint_clearance(pts, self.length, self.width)
        before = self.effective_stop()
        if self.clearance <= self.stop_d:
            self.proximity_stop = True
        elif self.proximity_stop and self.clearance > self.release_d:
            self.proximity_stop = False
        if self.effective_stop() != before:
            self.publish()          # 상태가 바뀌면 타이머를 기다리지 않고 즉시 게이트

    def on_button(self, which: str, msg: Bool) -> None:
        self.button[which] = bool(msg.data)
        if msg.data:
            self.button_latched = True
        self.publish()

    def on_reset(self, request: Trigger.Request, response: Trigger.Response):
        if any(self.button.values()):
            response.success = False
            response.message = 'estop still asserted ' + str(
                [k for k, v in self.button.items() if v])
            return response
        self.button_latched = False
        response.success = True
        response.message = 'estop latch cleared'
        self.publish()
        return response

    # --- 판정 ---
    def scan_stale(self) -> bool:
        return self.now() - self.scan_time > self.scan_timeout

    def effective_stop(self) -> bool:
        return (self.proximity_stop or self.button_latched or any(self.button.values())
                or self.scan_stale())

    def current_zone(self) -> int:
        if self.proximity_stop or self.clearance <= self.stop_d:
            return STOP
        if self.clearance <= self.crit_d:
            return CRITICAL
        if self.clearance <= self.warn_d:
            return WARNING
        return CLEAR

    def publish_state(self) -> None:
        active = self.effective_stop()
        zone = STOP if active else self.current_zone()
        if active != self._last_active:
            self.pub_active.publish(Bool(data=active))
            self._last_active = active
        if zone != self._last_zone:
            self.pub_zone.publish(UInt8(data=zone))
            self._last_zone = zone

    def publish(self) -> None:
        out = Twist()
        fresh = self.now() - self.cmd_time <= self.cmd_timeout
        if not self.effective_stop() and fresh:
            zone = self.current_zone()
            cap = {CLEAR: math.inf, WARNING: self.warn_v, CRITICAL: self.crit_v}.get(zone, 0.0)
            out.linear.x = max(-cap, min(cap, self.cmd.linear.x))
            out.angular.z = self.cmd.angular.z
        self.pub_cmd.publish(out)
        self.publish_state()


def main(args=None) -> None:
    run(SafetyGate, args)


if __name__ == '__main__':
    main()
