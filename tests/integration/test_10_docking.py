"""
시나리오 10: 정밀 도킹 (명세 4.8 "위치 오차 2 cm, 각도 오차 1° 이내", 도킹 3회 실패 시 에러 보고).

스택(system 프로필): Gazebo + localization + navigation + perception(ArUco 도킹 마커 검출) +
behavior(docking_server_node — amr_msgs/action/Dock). 스폰 = 월드 원점 (명시).
시행 10회 (dock_1 · dock_a 번갈아): 로봇을 도크 staging(behavior.yaml docks, ±0.2 m · ±10° 난수 오프셋)으로
순간 이동 → 위치 추정 안정(kidnap 감지 창 + localization/lost false 유지) → Dock 목표(dock_id,
approach_pose = staging, max_retries 3).
판정은 지면 진실: 도킹 완료 GT 자세 vs 기대 도킹 자세 = 월드 마커 판 면(마커 모델 +x = 바깥 법선, 계약 C3) +
behavior.yaml standoff, 방위 = 법선 반대 (docks.docked_pose). 위치 ≤ 0.02 m, 각도 ≤ 1°, 10/10. 서버의
자기 보고(final_position_error/angle_error)는 참고로 남긴다 — 마커 자세 추정이 치우치면 자기 보고는 합격해도
실제 자세는 어긋난다.
마커 가림 1회: dock_1 staging 에서 판 앞 1.1 m 에 상자(0.6 m)를 세워 마커를 가리고 Dock → success=false,
attempts_used = 3 (명세 "3회 실패 → 에러 보고"). 끝나면 상자를 치운다.
로그 docking.csv (시행마다 갱신).
"""

import math

from amr_itest import actions, cases, catalog, docks, gz, metrics, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from std_msgs.msg import Bool

CTX = Context(catalog.get(10))

TRIALS = 10
DOCKS = ('dock_1', 'dock_a')
POS_TOL = 0.02
ANG_TOL = math.radians(1.0)
MAX_RETRIES = 3
DOCK_TIMEOUT_S = 180.0
KIDNAP_DETECT_S = 6.0      # [s] kidnap_monitor 감지 창 (suspect 1 s + match 창 2.5 s + 여유)
SETTLE_S = 4.0             # [s] lost=false 유지 (복구 회전 cancel 포함)
RELOCALIZE_MAX_S = 120.0
OCCLUDER = (0.6, 0.6, 0.6)  # [m] 마커 가림 상자
OCCLUDER_FROM_PLATE = 1.1   # [m] 판 면 ~ 상자 중심
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')
COLUMNS = ['trial', 'dock_id', 'success', 'attempts', 'gt_pos_err_m', 'gt_ang_err_deg',
           'srv_pos_err_m', 'srv_ang_err_deg', 'time_s']


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_behavior=True)
                + [req.executable('amr_behavior', 'docking_server_node', 'Dock 액션 서버'),
                   req.config('amr_behavior', 'behavior.yaml', '도크 표')], 'docking')
    stack = Stack(CTX, *CTX.select())
    stack.system(use_localization=True, use_navigation=True, use_perception=True,
                 use_behavior=True, pose=(0.0, 0.0, 0.0))
    return stack.launch_description(), {'stack': stack}


class TestDocking(cases.ProbeCase):
    """도킹 10회 정밀도 (GT) + 마커 가림 재시도 소진."""

    CTX = CTX

    def _settled(self, lost) -> bool:
        """localization/lost 가 false 로 SETTLE_S (ROS 시각) 동안 이어질 때까지 (복구 회전 포함)."""
        state = {'since': None}

        def ok() -> bool:
            msg = lost.last()
            now = self.probe.now()
            if msg is None or msg.data:
                state['since'] = None
                return False
            state['since'] = now if state['since'] is None else state['since']
            return now - state['since'] >= SETTLE_S
        return self.probe.wait_until(ok, self.timeout(RELOCALIZE_MAX_S), 0.1)

    def _teleport(self, world: str, pose, lost, what: str) -> None:
        ok, out = gz.set_pose(world, self.settings.robot, *pose)
        self.assertTrue(ok, f'{what}: set_pose 실패: {out}')
        # 순간 이동은 위치 추정에는 납치다: kidnap_monitor_node 가 감지(≈3.4 s)하면 재초기화 + 제자리 회전
        # (behavior_server spin → cmd_vel_nav)을 하므로, 끝나기 전에 도킹을 시작하면 두 명령이 섞인다.
        self.assertTrue(self.probe.sleep_ros(KIDNAP_DETECT_S, self.timeout(120.0)))
        self.assertTrue(self._settled(lost), f'{what}: 순간 이동 뒤 위치 추정이 안정되지 않음')

    def _dock(self, dock, dock_id: str, staging, timeout: float):
        from amr_msgs.action import Dock
        goal = Dock.Goal()
        goal.dock_id = dock_id
        goal.approach_pose = actions.pose_stamped(*staging)
        goal.max_retries = MAX_RETRIES
        return dock.call(goal, self.timeout(timeout))[1]

    def test_10_docking(self, stack) -> None:
        from amr_msgs.action import Dock
        share = req.share_dir('amr_simulation')
        world_sdf = share / 'worlds' / self.settings.world
        world = self.settings.world.rsplit('.', 1)[0]
        table = docks.load_docks(req.config_file('amr_behavior', 'behavior.yaml'))
        gt = self.probe.subscribe('ground_truth/odom', Odometry)
        lost = self.probe.subscribe('localization/lost', Bool, 'latched')
        dock = actions.ActionCaller(self.probe, Dock, 'dock')
        self.assertTrue(dock.wait_server(self.timeout(300.0)), 'dock 액션 서버 없음')
        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        self.wait_startup_still()
        self.warm_up_localization(stack.drive_topic, gt)     # 첫 순간 이동을 kidnap_monitor 가 보게
        expected = {}
        for dock_id in DOCKS:
            marker = worldmap.include_pose(world_sdf, f'{dock_id}_marker')
            thick = worldmap.box_size(world_sdf, f'{dock_id}_marker')
            self.assertIsNotNone(marker, f'{dock_id}: 월드에 마커 모델 없음')
            expected[dock_id] = docks.docked_pose((marker[0], marker[1], marker[3]),
                                                  table[dock_id].standoff,
                                                  thick[0] if thick else 0.02)
        self.measure('expected_docked_pose', {k: [round(v, 4) for v in p]
                                              for k, p in expected.items()})
        rng = np.random.default_rng(self.settings.seed)
        rows, good = [], 0
        for trial in range(TRIALS):
            if not self.budget_for(DOCK_TIMEOUT_S + RELOCALIZE_MAX_S + 30.0, f'trial {trial}'):
                break
            dock_id = DOCKS[trial % len(DOCKS)]
            sx, sy, syaw = table[dock_id].staging
            ox, oy = rng.uniform(-0.2, 0.2, 2)
            oyaw = math.radians(rng.uniform(-10.0, 10.0))
            self._teleport(world, (sx + ox, sy + oy, syaw + oyaw), lost, f'trial {trial}')
            t0 = self.probe.now()
            result = self._dock(dock, dock_id, (sx, sy, syaw), DOCK_TIMEOUT_S)
            self.probe.sleep_ros(0.5, self.timeout(10.0))       # 정지 뒤 GT
            g = metrics.sample_from_odom(gt.last())
            pos_err, ang_err = docks.pose_error((g.x, g.y, g.yaw), expected[dock_id])
            success = (result is not None and result.success and pos_err <= POS_TOL
                       and ang_err <= ANG_TOL)
            good += int(success)
            rows.append([trial, dock_id, int(bool(result and result.success)),
                         result.attempts_used if result else -1, pos_err,
                         math.degrees(ang_err),
                         result.final_position_error if result else math.nan,
                         math.degrees(result.final_angle_error) if result else math.nan,
                         self.probe.now() - t0])
            self.ctx.record.write_csv('docking.csv', COLUMNS, rows)
        errs = [r for r in rows if r[2]]
        self.measure('gt_error', {
            'pos_max_m': cases.fmt(max((r[4] for r in errs), default=math.nan)),
            'ang_max_deg': cases.fmt(max((r[5] for r in errs), default=math.nan)),
            'srv_pos_max_m': cases.fmt(max((r[6] for r in errs), default=math.nan)),
            'srv_ang_max_deg': cases.fmt(max((abs(r[7]) for r in errs), default=math.nan))})
        self.check('docking within 2 cm / 1 deg (ground truth)', good, TRIALS, good == TRIALS)

    def test_20_marker_occluded(self) -> None:
        """마커를 가리면 재시도 3회를 모두 쓰고 실패를 보고한다 (명세 "3회 실패 → 에러 보고")."""
        from amr_msgs.action import Dock
        share = req.share_dir('amr_simulation')
        world_sdf = share / 'worlds' / self.settings.world
        world = self.settings.world.rsplit('.', 1)[0]
        table = docks.load_docks(req.config_file('amr_behavior', 'behavior.yaml'))
        lost = self.probe.subscribe('localization/lost', Bool, 'latched')
        dock = actions.ActionCaller(self.probe, Dock, 'dock')
        self.assertTrue(dock.wait_server(self.timeout(60.0)), 'dock 액션 서버 없음')
        self.assertTrue(self.budget_for(DOCK_TIMEOUT_S + RELOCALIZE_MAX_S + 30.0, 'occluded'),
                        '러너 상한 안에 가림 시험을 할 시간이 없다')
        dock_id = DOCKS[0]
        marker = worldmap.include_pose(world_sdf, f'{dock_id}_marker')
        nx, ny = math.cos(marker[3]), math.sin(marker[3])
        bx, by = marker[0] + OCCLUDER_FROM_PLATE * nx, marker[1] + OCCLUDER_FROM_PLATE * ny
        staging = table[dock_id].staging
        self._teleport(world, staging, lost, 'occluded')
        ok, out = gz.spawn_box(world, 'itest_occluder', bx, by, *OCCLUDER)
        self.assertTrue(ok, f'가림 상자 생성 실패: {out}')
        try:
            self.probe.sleep_ros(1.0, self.timeout(10.0))
            result = self._dock(dock, dock_id, staging, DOCK_TIMEOUT_S)
        finally:
            gz.remove(world, 'itest_occluder')
        self.measure('occluded', {'success': bool(result and result.success),
                                  'attempts_used': result.attempts_used if result else None})
        self.check('occluded marker: dock fails', bool(result and result.success), False,
                   result is not None and not result.success)
        self.check('occluded marker: attempts used', result.attempts_used if result else None,
                   MAX_RETRIES, result is not None and result.attempts_used == MAX_RETRIES)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
