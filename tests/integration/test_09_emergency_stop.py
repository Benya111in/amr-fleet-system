"""
시나리오 09: 긴급 정지 (명세 4.7 "안전 거리 0.3 m 이내 접근 시 즉시 정지", 4.9 E-stop).

스택: 운동학 대역(또는 Gazebo) + URDF TF + 휠 오도메트리(엔코더 생존 감시용) + 속도 명령 체인
cmd_vel_nav → velocity_profiler_node → cmd_vel_smoothed → safety_node → cmd_vel
(components.md §4.1).
safety_node·velocity_profiler_node 는 설치돼 있으면 실제 노드, 없으면 대역이다.

  (a) 근접 정지 20회: 0.3 m/s 주행 중 로봇 전방 0.2 m 에 장애물 주입(itest/obstacles) →
      "침범 스캔 수신 → 첫 cmd_vel = 0" 지연 ≤ 100 ms (주입 → 0 지연도 기록: 스캔 주기 포함)
  (b) E-stop 버튼 40회 (estop 20, /fleet/estop 20, 대시보드와 같은 transient_local Bool):
      발행 → 첫 cmd_vel = 0 지연 ≤ 100 ms, 해제(false) 후에도 래치 유지,
      safety/reset_estop(Trigger) 호출 후 주행 재개. 버튼마다 한 번 더: 눌린 동안의 reset 은
      거부되고 1 s 동안 정지 유지 (sequences.md §2 "버튼은 reset_estop 으로만 해제")
  (c) 벽 접근 폐루프: 0.5 m/s 로 벽을 향해 주행 → 존 감속 → 정지. 정지 명령 시점 GT 여유
      ≥ 0.25 m, 최종 여유 > 0 (충돌 0)
지연 판정은 최댓값, 호스트가 외부 부하 아래면 p95 (cases.latency_gate — 최댓값은 기록만).
장애물 주입·벽은 운동학 대역 전용 기능이라 Gazebo 백엔드에서는 (a)(c) 를 건너뛰고 (b) 만 한다.
"""

import math
from typing import List

from amr_itest import actions, cases, catalog, config, metrics
from amr_itest import kinematics as km
from amr_itest.scenario import Context, KINEMATIC
from amr_itest.stack import Stack
from geometry_msgs.msg import Twist
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

CTX = Context(catalog.get(9))

LATENCY_MAX_MS = 100.0
# 시행 수: 부하 판정 통계(p95)가 단일 이상치로 결정되지 않도록 20 (p95 = 정렬 19번째 부근)
PROXIMITY_TRIALS = 20
BUTTON_TRIALS = 20         # 버튼마다
BUTTON_TOPICS = ('estop', '/fleet/estop')   # 로봇별·전체 (feature/dashboard 와 같은 이름·QoS)
CRUISE = 0.3               # [m/s] (a)(b) 주행 속도
STARTUP_WALL_S = 180.0
OBSTACLE_GAP = 0.20        # [m] 주입 장애물 ~ 풋프린트 전면 여유 (< 0.3 m)


def scan_clearance(msg: LaserScan, ext, length: float, width: float) -> float:
    """safety_node 와 같은 기준(풋프린트 가장자리)으로 스캔의 최근접 여유 [m]."""
    angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
    ranges = np.asarray(msg.ranges, dtype=float)
    ranges[(ranges < msg.range_min) | (ranges > msg.range_max)] = np.inf
    return km.footprint_clearance(km.scan_to_points(ranges, angles, ext), length, width)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
    stack.simulator()
    stack.description()
    stack.localization(ekf=False)
    stack.velocity_chain(profiler=True, safety=True)
    return stack.launch_description(), {'stack': stack}


class TestEmergencyStop(cases.ProbeCase):
    """근접 정지 · E-stop 버튼 · 벽 접근."""

    CTX = CTX

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        rp = config.robot_params()
        s = config.sensors()
        cls.length = config.get(rp, 'robot.footprint_length', 0.60)
        cls.width = config.get(rp, 'robot.footprint_width', 0.40)
        cls.stop_d = config.get(rp, 'safety.emergency_stop_distance', 0.30)
        cls.ext = (config.get(s, 'lidar.extrinsic.x', 0.15),
                   config.get(s, 'lidar.extrinsic.y', 0.0),
                   config.get(s, 'lidar.extrinsic.yaw', 0.0))
        cls.cmd = cls.probe.subscribe('cmd_vel', Twist, keep_messages=20000)
        cls.gt = cls.probe.subscribe('ground_truth/odom', Odometry, keep_messages=20000)
        cls.scan = cls.probe.subscribe('scan_filtered', LaserScan, 'sensor', keep_messages=300)
        cls.active = cls.probe.subscribe('safety/estop_active', Bool, 'latched')
        cls.rows: List[list] = []
        cls.commander = None

    @classmethod
    def tearDownClass(cls) -> None:
        if cls.commander is not None:
            cls.commander.stop()
        super().tearDownClass()

    # --- 공통 ---
    def _obstacles(self, **kw) -> None:
        self.probe.publish('itest/obstacles', actions.obstacles_json(**kw), 'latched')

    def _cruise(self, stack, v: float = CRUISE) -> None:
        """속도 v 로 주행 명령을 계속 내고, 게이트를 통과한 cmd_vel 이 v 가 될 때까지 기다린다."""
        if self.commander is None:
            type(self).commander = actions.Commander(self.probe, stack.drive_topic).start()
        t0 = self.commander.set(v)
        hit = actions.wait_first(self.probe, self.cmd, t0,
                                 lambda m: m.linear.x >= 0.9 * v, self.timeout(10.0))
        self.assertIsNotNone(hit, f'cmd_vel 이 {v} m/s 로 열리지 않음 (safety 게이트가 닫힘?)')

    def _first_zero(self, since: float, timeout: float = 2.0):
        return actions.wait_first(self.probe, self.cmd, since, actions.is_zero,
                                  self.timeout(timeout))

    def _button_publishers(self) -> None:
        """
        대시보드처럼 버튼 발행자를 먼저 띄운다 (false, transient_local).

        safety_node 구독과 매칭된 뒤에 시행을 시작해, 첫 시행 지연에 발행자 디스커버리 시간이
        섞이지 않게 한다 (대시보드 발행자는 상시 떠 있다).
        """
        for topic in BUTTON_TOPICS:
            pub = self.probe.publisher(topic, Bool, 'latched')
            pub.publish(Bool(data=False))
            ok = self.probe.wait_until(lambda pub=pub: pub.get_subscription_count() >= 1,
                                       self.timeout(30.0))
            self.assertTrue(ok, f'{topic}: 구독자 없음 (safety_node 가 E-stop 을 구독하지 않음)')

    def _record(self, trial: int, kind: str, t_trigger: float, t_zero: float,
                clearance: float = math.nan) -> float:
        lat = (t_zero - t_trigger) * 1e3
        self.rows.append([trial, kind, t_trigger, t_zero, lat, clearance])
        return lat

    # --- 테스트 ---
    def test_00_ready(self, stack) -> None:
        """센서·게이트 준비 (GT 는 launch_testing_ros WaitForTopics, 스캔·cmd_vel 흐름)."""
        self.ready_gate([('ground_truth/odom', Odometry)], STARTUP_WALL_S)
        self.require_topic(self.scan, 3, STARTUP_WALL_S)
        self.require_topic(self.gt, 3, STARTUP_WALL_S)
        self._obstacles()
        self._cruise(stack)

    def test_10_proximity_stop(self, stack) -> None:
        """장애물 0.2 m 주입 → 침범 스캔 수신 후 100 ms 안에 cmd_vel = 0."""
        if stack.backend != KINEMATIC:
            self.skipTest('장애물 주입은 운동학 대역 전용 (Gazebo 는 시나리오 08 이 담당)')
        front = self.length / 2.0 + OBSTACLE_GAP
        box = (front, -0.15, front + 0.2, 0.15)
        scan_lat, inject_lat = [], []
        for trial in range(PROXIMITY_TRIALS):
            self._cruise(stack)
            t_inject = self.probe.wall()
            self._obstacles(robot_boxes=[box])
            hit_scan = actions.wait_first(
                self.probe, self.scan, t_inject,
                lambda m: scan_clearance(m, self.ext, self.length, self.width) <= self.stop_d,
                self.timeout(2.0))
            hit_zero = self._first_zero(t_inject)
            self.assertIsNotNone(hit_scan, f'trial {trial}: 침범 스캔이 오지 않음')
            self.assertIsNotNone(hit_zero, f'trial {trial}: cmd_vel 이 0 이 되지 않음')
            clearance = scan_clearance(hit_scan[1], self.ext, self.length, self.width)
            scan_lat.append(max(0.0, self._record(trial, 'proximity_scan', hit_scan[0],
                                                  hit_zero[0], clearance)))
            inject_lat.append(self._record(trial, 'proximity_inject', t_inject, hit_zero[0],
                                           clearance))
            # 정지 유지 확인 후 제거 → 자동 해제 (여유 > release 거리)
            self.assertTrue(self.probe.sleep_ros(0.3, self.timeout(10.0)))
            later = [m for _, m in self.cmd.messages(hit_zero[0] + 0.05)]
            self.assertTrue(later and all(actions.is_zero(m) for m in later),
                            f'trial {trial}: 장애물이 남아 있는데 주행 명령이 다시 나감')
            self._obstacles()
        s_scan = metrics.latency_summary(scan_lat)
        s_inject = metrics.latency_summary(inject_lat)
        self.measure('proximity_latency_scan_ms', s_scan)
        self.measure('proximity_latency_inject_ms', s_inject)
        stat, value, ok = cases.latency_gate(s_scan, LATENCY_MAX_MS)
        self.check(f'proximity stop latency {stat} (scan→cmd_vel=0)', value, LATENCY_MAX_MS, ok,
                   'ms', f'mean {s_scan["mean"]}, max {s_scan["max"]} ms, n={s_scan["count"]}')

    def _reset(self, what: str):
        res = self.probe.call(Trigger, 'safety/reset_estop', Trigger.Request(),
                              self.timeout(5.0))
        self.assertIsNotNone(res, f'{what}: safety/reset_estop 서비스 없음')
        return res

    def test_20_button_estop(self, stack) -> None:
        """
        E-stop 버튼(로봇별·전체) → 100 ms 안에 정지, 해제(false)만으로는 래치 유지, reset 후 재개.

        sequences.md §2: 대시보드 E-stop 버튼은 safety/reset_estop 서비스로만 해제한다.
        """
        self._obstacles()
        self._button_publishers()
        latencies = {}
        for topic in BUTTON_TOPICS:
            lat = []
            for trial in range(BUTTON_TRIALS):
                self._cruise(stack)
                t_press = self.probe.publish(topic, Bool(data=True), 'latched')
                hit = self._first_zero(t_press)
                self.assertIsNotNone(hit, f'{topic} trial {trial}: cmd_vel 이 0 이 되지 않음')
                lat.append(self._record(trial, f'button {topic}', t_press, hit[0]))
                t_release = self.probe.publish(topic, Bool(data=False), 'latched')
                self.assertTrue(self.probe.sleep_ros(0.3, self.timeout(10.0)))
                after = [m for _, m in self.cmd.messages(t_release + 0.05)]
                self.assertTrue(after and all(actions.is_zero(m) for m in after),
                                f'{topic} trial {trial}: 해제(false)만으로 래치가 풀림')
                res = self._reset(f'{topic} trial {trial}')
                self.assertTrue(res.success, f'{topic} trial {trial}: reset_estop 실패 {res}')
            latencies[topic] = metrics.latency_summary(lat)
            self._reset_while_pressed(stack, topic)
        self.measure('button_latency_ms', latencies)
        failed = []
        for topic, summary in latencies.items():
            stat, value, ok = cases.latency_gate(summary, LATENCY_MAX_MS)
            self.ctx.record.check(f'button {topic} latency {stat}', value, LATENCY_MAX_MS, ok,
                                  'ms')
            if not ok:
                failed.append(f'{topic} {stat} {value} ms')
        self.assertFalse(failed, f'E-stop 지연 기준 초과: {failed} — {latencies}')

    def _reset_while_pressed(self, stack, topic: str) -> None:
        """
        버튼이 눌린 동안의 reset 은 거부되고, 눌려 있는 동안 정지가 유지된다.

        해제(false) 직후 reset 이 토픽보다 먼저 도착하는 경우를 받아 주는 구현(reset 을 잠시
        보류했다가 false 가 오면 해제)도 계약 위반이 아니므로, 눌린 상태를 1 s 유지하는 동안만
        판정하고 끝은 false → reset 성공(또는 이미 해제)으로 정리한다.
        """
        self._cruise(stack)
        t_press = self.probe.publish(topic, Bool(data=True), 'latched')
        self.assertIsNotNone(self._first_zero(t_press), f'{topic}: cmd_vel 이 0 이 되지 않음')
        res = self._reset(f'{topic} pressed')
        t_reset = self.probe.wall()
        self.check(f'reset refused while {topic} asserted', res.success, False, not res.success)
        ok = self.probe.wait_until(
            lambda: self.active.last() is not None and self.active.last().data,
            self.timeout(2.0))
        self.check('safety/estop_active latched true', ok, True, ok)
        self.assertTrue(self.probe.sleep_ros(1.0, self.timeout(10.0)))
        held = [m for _, m in self.cmd.messages(t_reset)]
        stopped = bool(held) and all(actions.is_zero(m) for m in held)
        self.check(f'stopped while {topic} pressed after reset', stopped, True, stopped)
        self.probe.publish(topic, Bool(data=False), 'latched')
        ok = self.probe.wait_until(lambda: self._reset(f'{topic} release').success,
                                   self.timeout(3.0), 0.2)
        self.assertTrue(ok, f'{topic}: 해제 후 reset_estop 이 성공하지 않음')

    def test_30_wall_approach(self, stack) -> None:
        """0.5 m/s 로 벽 접근 → 0.3 m 부근에서 정지, 충돌 없음."""
        if stack.backend != KINEMATIC:
            self.skipTest('벽 주입은 운동학 대역 전용')
        self._obstacles()
        g = metrics.sample_from_odom(self.gt.last())
        self.assertLess(abs(g.yaw), 0.05, '직진 시험 전제: 로봇 방향이 +x 이어야 한다')
        front = g.x + self.length / 2.0
        wall = (front + 2.5, g.y - 1.0, front + 2.8, g.y + 1.0)
        self._obstacles(world_boxes=[wall])
        t0 = self.probe.wall()
        self.commander.set(0.5)

        def clearance(sample) -> float:
            return km.clearance_to_boxes(km.State(sample.x, sample.y, sample.yaw), [wall],
                                         self.length, self.width)

        stopped = actions.wait_rest(self.probe, self.gt, hold=1.0, timeout=self.timeout(30.0))
        self.commander.set(0.0)
        self.assertTrue(stopped, '로봇이 벽 앞에서 멈추지 않음')
        track = [(t, metrics.sample_from_odom(m)) for t, m in self.gt.messages(t0)]
        zero = actions.first_after(self.cmd, t0 + 0.5, actions.is_zero)
        self.assertIsNotNone(zero, '벽 접근 중 cmd_vel = 0 이 나오지 않음')
        at_stop = min(track, key=lambda ts: abs(ts[0] - zero[0]))[1]
        c_stop = clearance(at_stop)
        c_final = clearance(track[-1][1])
        c_min = min(clearance(s) for _, s in track)
        vmax = max(s.v for _, s in track)
        self.ctx.record.write_csv('approach.csv',
                                  ['recv_time', 'gt_x', 'gt_y', 'v', 'clearance_m'],
                                  [[t, s.x, s.y, s.v, clearance(s)] for t, s in track])
        self.measure('approach', {'clearance_at_stop_cmd_m': round(c_stop, 4),
                                  'final_clearance_m': round(c_final, 4),
                                  'min_clearance_m': round(c_min, 4), 'max_speed_mps': vmax})
        self._obstacles()
        self.check('clearance at stop command', c_stop, '>= 0.25', c_stop >= 0.25, 'm')
        self.check('no collision (min clearance)', c_min, '> 0', c_min > 0.0, 'm')

    def test_99_write_log(self) -> None:
        """estop_latency.csv 기록 (명세 4.10 응답 로그 포맷 확장)."""
        self.ctx.record.write_csv('estop_latency.csv', [
            'trial', 'kind', 't_trigger', 't_zero', 'latency_ms', 'clearance_m'], self.rows)
        self.assertTrue(self.rows, '기록된 시행이 없음')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
