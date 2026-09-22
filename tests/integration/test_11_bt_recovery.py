"""
시나리오 11: BT 에러 복구 (명세 4.8 "3가지 이상 에러 상황 자동 복구").

스택(system 프로필): Gazebo + localization + navigation + perception + behavior
(task_executor_node BT: 대기-이동-인식-작업-복귀, 복구 서브트리 RecoverNavigation/Perception/Docking).
스폰 = 대기 자세 = (−9, 0, π) (B-C 통로 교차로, 명시). 작업 = dock_2 적재 → dock_1 하역 (behavior.yaml staging).
에러 3종 — 하네스는 에러만 만들고 풀지 않는다(경로 차단은 작업 끝까지 유지, 위치 상실은 스스로 복구),
인식 실패만 복구 동작을 관측한 뒤 물품을 되돌린다 (복구 뒤 재시도가 성공할 수 있어야 복구를 판정할 수 있다).
  blocked     MOVING 중 현재 전역 경로 3 m 앞에 경로를 가로지르는 6 × 0.5 m 상자 (ign create). 복구 증거 =
              상자를 피해 가는 새 전역 경로(plan) 또는 RECOVERING 단계. 복구 시각 = 그 첫 증거
  lost        MOVING 중 로봇을 옆 통로 자유 지점으로 순간 이동 → localization/lost true → 스스로 재위치추정.
              복구 증거 = lost true 뒤 false + 추정 오차 ≤ 0.10 m. 복구 시각 = 그때
  perception  적재 도크에서 PERCEIVING 이 되면 도크 상자 2개를 치움(set_pose) → 인식 실패 → RecoverPerception
              (RECOVERING: 후진·회전) 관측 → 상자 원위치 → 재시도 인식 성공(DOCKING). 복구 시각 = DOCKING 전이
판정: 3종 모두 복구 증거가 주입 뒤 60 s 안에 나오고, 작업이 COMPLETED (작업 상한 600 s). executor/phase 전이는
recovery.csv phases 에 남는다. 순서는 lost → perception → blocked (차단 벽이 위치 추정까지 흔들 수 있어 마지막).
사례마다 실행기 IDLE + localization/lost false 를 기다린 뒤 작업을 준다 (lost 중에는 실행기가 'lost' 로 거절).
앞 사례가 위치 추정을 잃은 채 끝나면(IDLE 인데 lost 유지) 하네스가 사례 사이에 상태를 되돌린다 — 로봇을 대기
자세로 순간 이동 + initialpose·map EKF set_pose — 판정 창 밖의 초기화이며 행의 reset_before 에 남는다 (한 사례의 실패가
다음 사례를 '실행 불가'로 만들지 않게: 스모크 실측, lost 실패 뒤 나머지 2종이 실행되지 못했다).
한 사례의 전제가 무너지면(수락 거절 등) 그 사례를 실패로 기록하고 다음 사례로 넘어간다.
"""

import math

from amr_itest import actions, cases, catalog, config, docks, gz, metrics, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry, Path
import numpy as np
import pytest
from std_msgs.msg import Bool, String

CTX = Context(catalog.get(11))

# 흔적이 큰 주입을 뒤로: 경로 차단(6 m 벽)은 스캔을 크게 가려 위치 추정까지 흔들 수 있다 (스모크 실측)
CASES = ('lost', 'perception', 'blocked')
SPAWN = (-9.0, 0.0, math.pi)
PICKUP, DROPOFF = 'dock_2', 'dock_1'
RECOVERY_MAX_S = 60.0
TASK_MAX_S = 600.0
IDLE_WAIT_S = 300.0       # 앞 작업의 복귀(RETURNING) 가 끝나 IDLE 이 될 때까지 (실행기는 IDLE 에서만 수락)
RESET_WAIT_S = 60.0       # 사례 사이 초기화 뒤 lost false + 추정 오차 ≤ LOST_TOL_M 까지
BLOCK_AHEAD_M = 3.0
BLOCK = (0.5, 6.0, 1.0)   # [m] 경로 방향 두께, 가로 폭, 높이
LOST_TOL_M = 0.10
PARK = (-45.0, 45.0)      # 치운 물품을 둘 월드 밖 자리
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')
COLUMNS = ['case', 'injected_at', 'recovered_at', 'recovery_s', 'final_status', 'recovery_seen',
           'phases', 'reset_before']


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_behavior=True)
                + [req.executable('amr_behavior', 'task_executor_node', 'BT 작업 실행기'),
                   req.config('amr_behavior', 'behavior.yaml', '도크 표')], 'BT recovery')
    stack = Stack(CTX, *CTX.select())
    stack.system(use_localization=True, use_navigation=True, use_perception=True,
                 use_behavior=True, pose=SPAWN)
    return stack.launch_description(), {'stack': stack}


def point_along(path: np.ndarray, start_xy, distance: float):
    """경로에서 start 에 가장 가까운 점부터 distance [m] 앞의 (x, y, 진행 방위) (경로가 짧으면 끝점)."""
    if len(path) < 2:
        return None
    i = int(np.argmin(np.hypot(path[:, 0] - start_xy[0], path[:, 1] - start_xy[1])))
    acc = 0.0
    for j in range(i, len(path) - 1):
        seg = float(np.hypot(*(path[j + 1] - path[j])))
        if acc + seg >= distance and seg > 0:
            r = (distance - acc) / seg
            p = path[j] + r * (path[j + 1] - path[j])
            return float(p[0]), float(p[1]), math.atan2(*(path[j + 1] - path[j])[::-1])
        acc += seg
    d = path[-1] - path[-2]
    return float(path[-1, 0]), float(path[-1, 1]), math.atan2(d[1], d[0])


class TestBtRecovery(cases.ProbeCase):
    """에러 3종 주입 → 복구 증거 → 작업 완료."""

    CTX = CTX
    _reset_note = ''

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        from amr_msgs.msg import Task
        cls.Task = Task
        cls.gt = cls.probe.subscribe('ground_truth/odom', Odometry, keep_messages=5000)
        cls.est = cls.probe.subscribe('odometry/filtered_map', Odometry)
        cls.status = cls.probe.subscribe('task_status', Task, keep_messages=500)
        cls.phase = cls.probe.subscribe('executor/phase', String, 'latched', keep_messages=5000)
        cls.plan = cls.probe.subscribe('plan', Path, keep_messages=200)
        cls.lost = cls.probe.subscribe('localization/lost', Bool, 'latched', keep_messages=500)

    @property
    def world(self) -> str:
        return self.settings.world.rsplit('.', 1)[0]

    def _phase_seen(self, since: float, name: str) -> float:
        """since(wall) 이후 phase == name 을 처음 받은 wall 시각 (없으면 NaN)."""
        for t, m in self.phase.messages(since):
            if m.data.upper() == name:
                return t
        return math.nan

    def _final(self, tid: str, since: float):
        for _, m in reversed(self.status.messages(since)):
            if m.task_id == tid and m.status in (self.Task.STATUS_COMPLETED,
                                                 self.Task.STATUS_FAILED):
                return m.status
        return None

    def _est_error(self) -> float:
        g, e = self.gt.last(), self.est.last()
        if g is None or e is None:
            return math.inf
        a, b = metrics.sample_from_odom(g), metrics.sample_from_odom(e)
        return math.hypot(a.x - b.x, a.y - b.y)

    # --- 주입 ---
    def _inject_blocked(self, w_inject: float):
        s = metrics.sample_from_odom(self.gt.last())
        last = self.plan.last()
        path = (np.array([[p.pose.position.x, p.pose.position.y] for p in last.poses])
                if last is not None else np.zeros((0, 2)))
        at = point_along(path, (s.x, s.y), BLOCK_AHEAD_M) if len(path) else None
        if at is None:      # 경로가 없으면 진행 방향 3 m 앞
            at = (s.x + BLOCK_AHEAD_M * math.cos(s.yaw), s.y + BLOCK_AHEAD_M * math.sin(s.yaw),
                  s.yaw)
        bx, by, heading = at
        ok, out = gz.spawn_box(self.world, 'itest_block', bx, by, *BLOCK, yaw=heading)
        self.assertTrue(ok, f'장애물 생성 실패: {out}')
        self.measure('block', {'x': round(bx, 2), 'y': round(by, 2),
                               'yaw_deg': round(math.degrees(heading), 1)})
        box = worldmap.Rect(bx, by, heading, BLOCK[0], BLOCK[1])

        def evidence():
            t_rec = self._phase_seen(w_inject, 'RECOVERING')
            if math.isfinite(t_rec):
                return t_rec
            for t, m in self.plan.messages(w_inject + 0.5):
                pts = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])
                if len(pts) and not np.any(box.contains(pts[:, 0], pts[:, 1], 0.3)):
                    return t
            return None
        return evidence, lambda: gz.remove(self.world, 'itest_block')

    def _inject_lost(self, w_inject: float):
        s = metrics.sample_from_odom(self.gt.last())
        shapes = worldmap.footprints(req.share_dir('amr_simulation') / 'worlds'
                                     / self.settings.world,
                                     req.share_dir('amr_simulation') / 'models',
                                     config.scan_plane_height(), 'both')
        target = None
        for dx, dy in ((0.0, 6.0), (0.0, -6.0), (6.0, 0.0), (-6.0, 0.0)):
            x, y = s.x + dx, s.y + dy
            if not any(bool(sh.contains(np.array([x]), np.array([y]), 0.8)[0]) for sh in shapes) \
                    and abs(x) < 28.0 and abs(y) < 18.0:
                target = (x, y)
                break
        self.assertIsNotNone(target, '순간 이동할 자유 지점 없음')
        ok, out = gz.set_pose(self.world, self.settings.robot, target[0], target[1], s.yaw)
        self.assertTrue(ok, f'set_pose 실패: {out}')
        self.measure('lost_teleport', {'from': [round(s.x, 2), round(s.y, 2)],
                                       'to': [round(target[0], 2), round(target[1], 2)]})

        def evidence():
            msgs = self.lost.messages(w_inject)
            i = next((k for k, (_, m) in enumerate(msgs) if m.data), None)
            if i is None:
                return None
            back = next((t for t, m in msgs[i + 1:] if not m.data), None)
            if back is not None and self._est_error() <= LOST_TOL_M:
                return back
            return None
        return evidence, lambda: None

    def _inject_perception(self, w_inject: float, boxes):
        for name, _ in boxes:
            ok, out = gz.set_pose(self.world, name, PARK[0], PARK[1], 0.0, z=0.0)
            self.assertTrue(ok, f'{name} 치우기 실패: {out}')
        restored = {'done': False}

        def restore():
            if not restored['done']:
                for name, pose in boxes:
                    gz.set_pose(self.world, name, pose[0], pose[1], pose[3], z=pose[2] + 0.001)
                restored['done'] = True

        def evidence():
            t_rec = self._phase_seen(w_inject, 'RECOVERING')
            if not math.isfinite(t_rec):
                return None
            restore()                          # 복구 동작을 봤으니 물품을 되돌린다 → 재시도 성공 기대
            t_dock = self._phase_seen(t_rec, 'DOCKING')
            return t_dock if math.isfinite(t_dock) else None
        return evidence, restore

    def _wait_evidence(self, evidence, timeout: float):
        """evidence() 가 복구 시각(wall)을 돌려줄 때까지 (없으면 None)."""
        hit = {}

        def found() -> bool:
            t = evidence()
            if t is not None:
                hit['t'] = t
            return t is not None
        self.probe.wait_until(found, timeout, 0.2)
        return hit.get('t')

    # --- 시험 ---
    def test_10_recovery(self) -> None:
        from amr_msgs.srv import AssignTask
        table = docks.load_docks(req.config_file('amr_behavior', 'behavior.yaml'))
        world_sdf = req.share_dir('amr_simulation') / 'worlds' / self.settings.world
        boxes = [(n, worldmap.include_pose(world_sdf, n))
                 for n in (f'{PICKUP}_box_medium', f'{PICKUP}_box_large')]
        self.assertTrue(all(p is not None for _, p in boxes), f'월드에 {PICKUP} 물품 없음')
        self.require_topic(self.gt, 5, 300.0)
        self.assertTrue(self.probe.service_available(AssignTask, 'assign_task',
                                                     self.timeout(300.0)))
        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        self.wait_startup_still()
        rows, failed = [], []
        for case in CASES:
            if not self.budget_for(IDLE_WAIT_S + TASK_MAX_S, case):
                failed.append(f'{case}: 시간 부족')
                break
            try:
                row = self._run_case(case, table, boxes)
            except AssertionError as exc:      # 전제(IDLE·수락·주행 시작) 실패: 기록하고 다음 사례로
                row = [case, math.nan, math.nan, math.nan, f'not run: {exc}'[:160], 0, '',
                       self._reset_note]
            rows.append(row)
            self.ctx.record.write_csv('recovery.csv', COLUMNS, rows)
            ok = (row[4] == self.Task.STATUS_COMPLETED and row[5] == 1
                  and row[3] <= RECOVERY_MAX_S)
            self.ctx.record.check(f'{case}: recovered <= {RECOVERY_MAX_S} s and COMPLETED',
                                  {'recovery_s': cases.fmt(row[3], 1), 'final': row[4]},
                                  {'recovery_s': RECOVERY_MAX_S,
                                   'final': self.Task.STATUS_COMPLETED}, ok)
            if not ok:
                failed.append(case)
        self.check('recovered error cases', len(CASES) - len(failed), len(CASES), not failed,
                   '', f'실패 {failed}')

    def _idle(self) -> bool:
        phase = self.phase.last()
        return phase is not None and phase.data.upper() == 'IDLE'

    def _reset_state(self, case: str) -> str:
        """
        사례 사이 초기화: 대기 자세로 순간 이동 + initialpose·map EKF set_pose → lost false 대기.

        실행기가 IDLE 인데 앞 사례가 남긴 lost 가 풀리지 않을 때만 부른다 (판정 창 밖). 반환 = 기록 문자열.
        """
        ok, out = gz.set_pose(self.world, self.settings.robot, SPAWN[0], SPAWN[1], SPAWN[2])
        self.assertTrue(ok, f'{case}: 초기화 set_pose 실패: {out}')
        self.probe.sleep_ros(2.0, self.timeout(30.0))
        seeded = actions.seed_pose(self.probe, '', *SPAWN, timeout=self.timeout(10.0))
        back = self.probe.wait_until(lambda: self._ready() and self._est_error() <= LOST_TOL_M,
                                     self.timeout(RESET_WAIT_S), 0.2)
        note = (f'teleport+initialpose+ekf_set_pose({seeded["ekf_set_pose"]}) '
                f'({"ok" if back else "still lost"})')
        self.measure(f'{case}_reset', {'reason': 'previous case left localization lost',
                                       'result': note})
        return note

    def _ready(self) -> bool:
        """실행기 IDLE 이고 위치 추정이 lost 가 아님 (실행기는 lost 중 작업을 'lost' 로 거절한다)."""
        phase, lost = self.phase.last(), self.lost.last()
        return (phase is not None and phase.data.upper() == 'IDLE'
                and (lost is None or not lost.data))

    def _run_case(self, case: str, table, boxes) -> list:
        """사례 하나: 작업 → 주입 → 복구 증거 → 작업 종료. 반환 = recovery.csv 행."""
        from amr_msgs.srv import AssignTask
        self._reset_note = ''
        ready = self.probe.wait_until(self._ready, self.timeout(IDLE_WAIT_S), 0.2)
        lost = self.lost.last()
        if not ready and self._idle() and lost is not None and lost.data:
            self._reset_note = self._reset_state(case)
            ready = self._ready()
        self.assertTrue(ready, f'{case}: IDLE·위치 추정 정상으로 돌아오지 않음 '
                               f'(phase {getattr(self.phase.last(), "data", None)}, '
                               f'lost {getattr(self.lost.last(), "data", None)})')
        task = self.Task()
        task.task_id = f'itest_bt_{case}'
        task.robot_id = self.settings.robot
        task.priority = 100
        task.item_type = 'small'
        task.item_mass = 2.0
        task.pickup_pose = actions.pose_stamped(*table[PICKUP].staging)
        task.dropoff_pose = actions.pose_stamped(*table[DROPOFF].staging)
        task.header.stamp = self.probe.node.get_clock().now().to_msg()
        w0 = self.probe.wall()
        res = self.probe.call(AssignTask, 'assign_task', AssignTask.Request(task=task),
                              self.timeout(10.0))
        self.assertTrue(res is not None and res.success, f'{case}: assign_task 거절 {res}')
        trigger = 'PERCEIVING' if case == 'perception' else 'MOVING'
        started = self.probe.wait_until(
            lambda: math.isfinite(self._phase_seen(w0, trigger)), self.timeout(240.0))
        self.assertTrue(started, f'{case}: executor/phase 가 {trigger} 가 되지 않음')
        if case != 'perception':      # 방향이 정해지도록 1 m 이상 움직인 뒤
            p0 = metrics.sample_from_odom(self.gt.last())
            self.probe.wait_until(lambda: math.hypot(
                metrics.sample_from_odom(self.gt.last()).x - p0.x,
                metrics.sample_from_odom(self.gt.last()).y - p0.y) >= 1.0,
                self.timeout(60.0))
        w_inject, t_inject = self.probe.wall(), self.probe.now()
        if case == 'blocked':
            evidence, cleanup = self._inject_blocked(w_inject)
        elif case == 'lost':
            evidence, cleanup = self._inject_lost(w_inject)
        else:
            evidence, cleanup = self._inject_perception(w_inject, boxes)
        try:
            t_evidence = self._wait_evidence(evidence, self.timeout(RECOVERY_MAX_S + 30.0))
            recovery_s = (t_evidence - w_inject) if t_evidence is not None else math.nan
            self.probe.wait_until(lambda: self._final(task.task_id, w0) is not None,
                                  self.timeout(TASK_MAX_S), 0.5)
        finally:
            cleanup()
        final = self._final(task.task_id, w0)
        phases = [m.data for _, m in self.phase.messages(w0)]
        compact = [p for j, p in enumerate(phases) if j == 0 or p != phases[j - 1]]
        return [case, t_inject, t_inject + (recovery_s if t_evidence else math.nan),
                recovery_s, final, int(t_evidence is not None), '>'.join(compact),
                self._reset_note]


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
