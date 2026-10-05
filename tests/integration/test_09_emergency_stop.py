"""
시나리오 09: 긴급 정지 (명세 4.7 "안전 거리 0.3 m 이내 접근 시 즉시 정지", 4.9 E-stop).

스택: 운동학 대역(또는 Gazebo) + URDF TF + 휠 오도메트리(엔코더 생존 감시용) + 속도 명령 체인
cmd_vel_nav → velocity_profiler_node → cmd_vel_smoothed → safety_node → cmd_vel
(components.md §4.1). safety_node·velocity_profiler_node 는 설치돼 있으면 실제 노드, 없으면 대역이다.

  (a) 근접 정지 20회: 0.3 m/s 주행 중 로봇 전방 0.2 m 에 장애물 주입 → 첫 cmd_vel = 0.
      판정 두 가지 (sequences.md §2 "safety_node 50 Hz → 지연 ≤ 20 ms"):
        침범 스캔 수신 → 0   ≤ 20 ms            (safety 처리 지연)
        주입 → 0            ≤ 스캔 주기 + 20 ms  (명세 "즉시 정지" 의 끝-끝: 다음 스캔까지 기다림 포함)
      운동학: itest/obstacles 로 주입 (즉시). Gazebo: ign create 로 정적 상자 생성, 주입 시각 = 서비스 응답.
      프로브가 cmd_vel=0 을 스캔보다 먼저 받으면 음수 — 프로브 해상도이므로 그대로 둔다 (0 으로 자르지 않음).
  (b) E-stop 버튼 40회 (estop 20, /fleet/estop 20, 대시보드와 같은 transient_local Bool):
      발행 → 첫 cmd_vel = 0 ≤ 20 ms, 해제(false) 후에도 래치 유지, safety/reset_estop(Trigger) 뒤 주행 재개.
      버튼마다 한 번 더: 눌린 동안의 reset 은 거부되고 정지 유지, 이어서 false 를 보내도 reset 성공 전까지
      cmd_vel = 0 · safety/estop_active = true 유지 (components.md safety_node 계약: 거절된 reset 은 대기 상태를
      남기지 않는다 — 버튼은 reset_estop 으로만 해제, sequences.md §2)
  (c) 벽 접근 폐루프: 0.5 m/s 로 벽을 향해 주행 → 존 감속 → 정지. 정지 명령 시점 GT 여유 ≥ 0.25 m,
      최소 여유 > 0 (충돌 0). 벽 = 운동학 itest/obstacles, Gazebo 정적 상자.
지연 판정은 항상 최댓값 (부하 때문에 느슨하게 하지 않는다). 부하 중(loadavg > CPU/2) 초과면 그 묶음을 한 번 다시
재고 그 결과로 판정한다 — 두 번 모두 estop_latency.csv·result.json 에 남는다 (cases.retry_under_load).
"""

import math
from typing import List

from amr_itest import actions, cases, catalog, config, gz, metrics
from amr_itest import kinematics as km
from amr_itest.scenario import Context, GAZEBO, KINEMATIC
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

SAFETY_MAX_MS = 20.0       # sequences.md §2: safety_node 50 Hz → 지연 ≤ 20 ms
BUTTON_MAX_MS = 20.0       # 버튼 → 정지도 같은 safety_node 게이트
PROXIMITY_TRIALS = 20
BUTTON_TRIALS = 20         # 버튼마다
BUTTON_TOPICS = ('estop', '/fleet/estop')   # 로봇별·전체 (feature/dashboard 와 같은 이름·QoS)
CRUISE = 0.3               # [m/s] (a)(b) 주행 속도
STARTUP_WALL_S = 180.0
OBSTACLE_GAP = 0.20        # [m] 주입 장애물 ~ 풋프린트 전면 여유 (< 0.3 m)
RELEASE_HOLD_S = 1.5       # [s] false 뒤 래치 유지 확인 (예전 구현의 reset 보류 1 s 보다 길게)
BOX = (0.2, 0.3, 0.5)      # [m] 주입 상자 (전후, 좌우, 높이 — LiDAR 평면 0.20 m 을 덮는다)


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
    stack = Stack(CTX, *CTX.select())
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
        cls.length = config.get(rp, 'robot.footprint_length')
        cls.width = config.get(rp, 'robot.footprint_width')
        cls.stop_d = config.get(rp, 'safety.emergency_stop_distance')
        cls.ext = (config.get(s, 'lidar.extrinsic.x'), config.get(s, 'lidar.extrinsic.y'),
                   config.get(s, 'lidar.extrinsic.yaw'))
        cls.scan_period_ms = 1000.0 / float(config.get(s, 'lidar.update_rate'))
        cls.cmd = cls.probe.subscribe('cmd_vel', Twist, keep_messages=20000)
        cls.gt = cls.probe.subscribe('ground_truth/odom', Odometry, keep_messages=20000)
        cls.scan = cls.probe.subscribe('scan_filtered', LaserScan, 'sensor', keep_messages=300)
        cls.active = cls.probe.subscribe('safety/estop_active', Bool, 'latched')
        cls.rows: List[list] = []
        cls.commander = None
        cls.spawned: List[str] = []

    @classmethod
    def tearDownClass(cls) -> None:
        if cls.commander is not None:
            cls.commander.stop()
        super().tearDownClass()

    # --- 장애물 주입 (백엔드별) ---
    @property
    def world(self) -> str:
        return self.settings.world.rsplit('.', 1)[0]

    def _clear(self, stack) -> None:
        if stack.backend == KINEMATIC:
            self.probe.publish('itest/obstacles', actions.obstacles_json(), 'latched')
            return
        while self.spawned:
            gz.remove(self.world, self.spawned.pop())

    def _inject_front(self, stack, name: str) -> float:
        """
        로봇 전방 OBSTACLE_GAP 에 상자 → 주입 시각(wall).

        Gazebo 는 생성 요청 직전 시각이다 (상자가 언제 월드에 생기는지 알 수 없어 주입→0 은 ign 서비스 시간을
        포함한 상한이 된다 — 판정은 스캔→0 이 엄밀하다).
        """
        front = self.length / 2.0 + OBSTACLE_GAP
        if stack.backend == KINEMATIC:
            box = (front, -BOX[1] / 2, front + BOX[0], BOX[1] / 2)
            return self.probe.publish('itest/obstacles',
                                      actions.obstacles_json(robot_boxes=[box]), 'latched')
        g = metrics.sample_from_odom(self.gt.last())
        cx = g.x + math.cos(g.yaw) * (front + BOX[0] / 2)
        cy = g.y + math.sin(g.yaw) * (front + BOX[0] / 2)
        t = self.probe.wall()
        ok, out = gz.spawn_box(self.world, name, cx, cy, *BOX, yaw=g.yaw)
        self.assertTrue(ok, f'{name}: 상자 생성 실패 {out}')
        self.spawned.append(name)
        return t

    def _wall(self, stack, box) -> None:
        """월드 축 정렬 벽 (x0, y0, x1, y1)."""
        if stack.backend == KINEMATIC:
            self.probe.publish('itest/obstacles', actions.obstacles_json(world_boxes=[box]),
                               'latched')
            return
        ok, out = gz.spawn_box(self.world, 'itest_wall', (box[0] + box[2]) / 2,
                               (box[1] + box[3]) / 2, box[2] - box[0], box[3] - box[1], 1.0)
        self.assertTrue(ok, f'벽 생성 실패 {out}')
        self.spawned.append('itest_wall')

    # --- 공통 ---
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

    def _gate(self, name: str, attempts, limit_ms: float, ok: bool) -> None:
        """최댓값 판정 기록 (재측정했으면 시도마다 남기고 마지막 것으로 판정)."""
        self.measure(f'{name}_attempts', [{'max_ms': a['max'], 'mean_ms': a['mean'],
                                           'count': a['count'], 'under_load': loaded}
                                          for a, loaded in attempts])
        final = attempts[-1][0]
        self.ctx.record.check(f'{name} max', final['max'], limit_ms, ok, 'ms')

    # --- 테스트 ---
    def test_00_ready(self, stack) -> None:
        """센서·게이트 준비 (GT 는 launch_testing_ros WaitForTopics, 스캔·cmd_vel 흐름)."""
        self.ready_gate([('ground_truth/odom', Odometry)], STARTUP_WALL_S)
        self.require_topic(self.scan, 3, STARTUP_WALL_S)
        self.require_topic(self.gt, 3, STARTUP_WALL_S)
        self._clear(stack)
        self._cruise(stack)

    def _proximity_round(self, stack, attempt: int):
        scan_lat, inject_lat = [], []
        for trial in range(PROXIMITY_TRIALS):
            self._cruise(stack)
            t_inject = self._inject_front(stack, f'itest_prox_{attempt}_{trial}')
            hit_scan = actions.wait_first(
                self.probe, self.scan, t_inject,
                lambda m: scan_clearance(m, self.ext, self.length, self.width) <= self.stop_d,
                self.timeout(2.0))
            hit_zero = self._first_zero(t_inject)
            self.assertIsNotNone(hit_scan, f'trial {trial}: 침범 스캔이 오지 않음')
            self.assertIsNotNone(hit_zero, f'trial {trial}: cmd_vel 이 0 이 되지 않음')
            clearance = scan_clearance(hit_scan[1], self.ext, self.length, self.width)
            scan_lat.append(self._record(trial, f'proximity_scan#{attempt}', hit_scan[0],
                                         hit_zero[0], clearance))
            inject_lat.append(self._record(trial, f'proximity_inject#{attempt}', t_inject,
                                           hit_zero[0], clearance))
            # 정지 유지 확인 후 제거 → 자동 해제 (근접 정지는 E-stop 래치가 아니다, 계약 C1)
            self.assertTrue(self.probe.sleep_ros(0.3, self.timeout(10.0)))
            later = [m for _, m in self.cmd.messages(hit_zero[0] + 0.05)]
            self.assertTrue(later and all(actions.is_zero(m) for m in later),
                            f'trial {trial}: 장애물이 남아 있는데 주행 명령이 다시 나감')
            self._clear(stack)
        return metrics.latency_summary(scan_lat), metrics.latency_summary(inject_lat)

    def test_10_proximity_stop(self, stack) -> None:
        """장애물 0.2 m 주입 → 침범 스캔 수신 후 20 ms, 주입 후 스캔 주기 + 20 ms 안에 cmd_vel = 0."""
        inject_max = self.scan_period_ms + SAFETY_MAX_MS
        attempts = []
        for attempt in range(2):        # 부하 중 초과일 때만 한 번 더 (cases.retry_under_load 와 같은 규칙)
            s_scan, s_inject = self._proximity_round(stack, attempt)
            ok_scan = cases.latency_gate(s_scan, SAFETY_MAX_MS)[2]
            # Gazebo 의 주입 시각은 생성 요청 시각이라 ign 서비스 시간이 섞인다 → 기록만 (스캔→0 이 판정)
            ok_inject = (cases.latency_gate(s_inject, inject_max)[2]
                         if stack.backend == KINEMATIC else True)
            loaded = cases.host_under_load()
            attempts.append({'scan_ms': s_scan, 'inject_ms': s_inject, 'under_load': loaded})
            if (ok_scan and ok_inject) or not loaded:
                break
        self.measure('proximity_latency_scan_ms', s_scan)
        self.measure('proximity_latency_inject_ms', s_inject)
        self.measure('proximity_attempts', attempts)
        self.ctx.record.check('proximity stop latency max (scan→cmd_vel=0)', s_scan['max'],
                              SAFETY_MAX_MS, ok_scan, 'ms')
        if stack.backend == KINEMATIC:
            self.ctx.record.check('proximity stop latency max (inject→cmd_vel=0)',
                                  s_inject['max'], round(inject_max, 1), ok_inject, 'ms')
        self.assertTrue(ok_scan and ok_inject,
                        f'근접 정지 지연 초과: 스캔→0 {s_scan} / 주입→0 {s_inject} '
                        f'(기준 {SAFETY_MAX_MS} / {inject_max:.1f} ms, 시도 {len(attempts)})')

    def _reset(self, what: str):
        res = self.probe.call(Trigger, 'safety/reset_estop', Trigger.Request(),
                              self.timeout(5.0))
        self.assertIsNotNone(res, f'{what}: safety/reset_estop 서비스 없음')
        return res

    def _button_round(self, stack, topic: str, attempt: int):
        lat = []
        for trial in range(BUTTON_TRIALS):
            self._cruise(stack)
            t_press = self.probe.publish(topic, Bool(data=True), 'latched')
            hit = self._first_zero(t_press)
            self.assertIsNotNone(hit, f'{topic} trial {trial}: cmd_vel 이 0 이 되지 않음')
            lat.append(self._record(trial, f'button {topic}#{attempt}', t_press, hit[0]))
            t_release = self.probe.publish(topic, Bool(data=False), 'latched')
            self.assertTrue(self.probe.sleep_ros(0.3, self.timeout(10.0)))
            after = [m for _, m in self.cmd.messages(t_release + 0.05)]
            self.assertTrue(after and all(actions.is_zero(m) for m in after),
                            f'{topic} trial {trial}: 해제(false)만으로 래치가 풀림')
            res = self._reset(f'{topic} trial {trial}')
            self.assertTrue(res.success, f'{topic} trial {trial}: reset_estop 실패 {res}')
        return metrics.latency_summary(lat)

    def test_20_button_estop(self, stack) -> None:
        """
        E-stop 버튼(로봇별·전체) → 20 ms 안에 정지, 해제(false)만으로는 래치 유지, reset 후 재개.

        sequences.md §2: 대시보드 E-stop 버튼은 safety/reset_estop 서비스로만 해제한다.
        """
        self._clear(stack)
        self._button_publishers()
        latencies, failed = {}, []
        for topic in BUTTON_TOPICS:
            count = iter(range(10))
            summary, ok, attempts = cases.retry_under_load(
                lambda topic=topic, count=count: self._button_round(stack, topic, next(count)),
                BUTTON_MAX_MS)
            latencies[topic] = summary
            self._gate(f'button {topic} latency', attempts, BUTTON_MAX_MS, ok)
            if not ok:
                failed.append(f'{topic} max {summary["max"]} ms')
            self._reset_while_pressed(stack, topic)
        self.measure('button_latency_ms', latencies)
        self.assertFalse(failed, f'E-stop 지연 기준 초과: {failed} — {latencies}')

    def _reset_while_pressed(self, stack, topic: str) -> None:
        """
        눌린 동안의 reset 은 거부되고, 그 뒤 false 를 보내도 새 reset 이 성공하기 전까지 정지·래치 유지.

        거절된 reset 이 "보류" 되어 뒤이은 false 로 래치가 풀리면 계약 위반이다 (components.md: 거절된 reset 은
        대기 상태를 남기지 않는다, sequences.md §2: 버튼은 reset_estop 으로만 해제).
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
        self.assertTrue(self.probe.sleep_ros(0.3, self.timeout(10.0)))
        held = [m for _, m in self.cmd.messages(t_reset)]
        stopped = bool(held) and all(actions.is_zero(m) for m in held)
        self.check(f'stopped while {topic} pressed after refused reset', stopped, True, stopped)
        # 거절 직후 false: 예전 구현은 거절된 reset 을 1 s 보류했다가 false 가 오면 풀었다 — 그 창 안에서 보낸다
        t_release = self.probe.publish(topic, Bool(data=False), 'latched')
        self.assertTrue(self.probe.sleep_ros(RELEASE_HOLD_S, self.timeout(10.0)))
        after = [m for _, m in self.cmd.messages(t_release + 0.05)]
        still = (bool(after) and all(actions.is_zero(m) for m in after)
                 and self.active.last() is not None and self.active.last().data)
        self.check(f'{topic} released without reset: latch held (cmd_vel=0, estop_active)',
                   still, True, still)
        res = self._reset(f'{topic} release')
        self.check(f'reset after {topic} release succeeds', res.success, True, res.success)
        self._cruise(stack)

    def test_30_wall_approach(self, stack) -> None:
        """0.5 m/s 로 벽 접근 → 0.3 m 부근에서 정지, 충돌 없음."""
        self._clear(stack)
        if self.commander is not None:
            self.commander.set(0.0)
        self.assertTrue(actions.wait_rest(self.probe, self.gt, hold=0.5,
                                          timeout=self.timeout(30.0)), '벽 시험 전 정지 실패')
        g = metrics.sample_from_odom(self.gt.last())
        self.assertLess(abs(g.yaw), 0.05, '직진 시험 전제: 로봇 방향이 +x 이어야 한다')
        front = g.x + self.length / 2.0
        wall = (front + 2.5, g.y - 1.0, front + 2.8, g.y + 1.0)
        self._wall(stack, wall)
        if stack.backend == GAZEBO:     # 상자가 스캔에 나타날 때까지 (렌더링 1~2 주기)
            self.probe.sleep_ros(0.5, self.timeout(10.0))
        t0 = self.probe.wall()
        if self.commander is None:
            type(self).commander = actions.Commander(self.probe, stack.drive_topic).start()
        self.commander.set(0.5)

        def clearance(sample) -> float:
            return km.clearance_to_boxes(km.State(sample.x, sample.y, sample.yaw), [wall],
                                         self.length, self.width)

        self.probe.sleep_ros(1.0, self.timeout(10.0))
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
        self._clear(stack)
        self.check('clearance at stop command', c_stop, '>= 0.25', c_stop >= 0.25, 'm')
        self.check('no collision (min clearance)', c_min, '> 0', c_min > 0.0, 'm')

    def test_99_write_log(self) -> None:
        """estop_latency.csv 기록 (명세 4.10 응답 로그 포맷 확장, 음수 = 프로브 해상도)."""
        self.ctx.record.write_csv('estop_latency.csv', [
            'trial', 'kind', 't_trigger', 't_zero', 'latency_ms', 'clearance_m'], self.rows)
        self.assertTrue(self.rows, '기록된 시행이 없음')


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
