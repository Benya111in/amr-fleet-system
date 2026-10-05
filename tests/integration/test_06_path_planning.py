"""
시나리오 06: 경로 계획 성공률 (명세 4.4 "임의 출발지-목적지 50쌍 성공률 98 % 이상", 재계획 500 ms).

스택(system 프로필): Gazebo + localization(지도 + AMCL + EKF) + navigation(planner_server 에 직접 구현한
A* 전역 플래너 플러그인). 스폰 = 월드 원점 (명시).
준비: lifecycle(map_server·AMCL·Nav2) 활성 — 활성화 도중 요청은 거절된다(이전 실행 50/50 거절) —, /map 수신과
map ↔ 월드 항등 정합(월드 좌표로 뽑은 쌍을 map 자세로 보내는 근거), planner_server 파라미터로 AStar 플러그인
확인 (planner_plugins 에 AStar, AStar.plugin = amr_navigation::AStarPlanner).
쌍 생성: 월드 SDF 구조물(visual ∪ collision, LiDAR 평면)에서 0.6 m 이상 떨어진 자유 지점을 고정 시드로
100 개 뽑아 50 쌍 (ITEST_SEED). compute_path_to_pose(use_start=true, planner_id=AStar)로 계획만 한다(주행 없음) —
planner_server 에 계획기가 여럿이면 Humble 은 빈 planner_id 를 거절한다. 같은 쌍을 NavFn·Smac 으로도
계획해 비교 열로 남긴다 (판정 안 함).
판정: AStar 성공(SUCCEEDED + 경로 점 ≥ 2 + 목표 0.1 m 이내 + 경로 점이 구조물 내부에 없음) ≥ 49/50,
계획 시간(result.planning_time) 최대 ≤ 500 ms (sequences.md §2 재계획 500 ms). 로그 plans.csv (행마다 갱신).
"""

import math
import os

from amr_itest import actions, cases, catalog, config, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
import numpy as np
import pytest

CTX = Context(catalog.get(6))

PAIRS = 50
SUCCESS_MIN = 0.98
PLAN_MAX_MS = 500.0
MARGIN = 0.6               # [m] 구조물에서 떨어진 거리 (로봇 외접원 0.36 + 여유)
BOUNDS = (-29.0, -19.0, 29.0, 19.0)   # 60 × 40 m 벽 안쪽
PLANNER_ID = os.environ.get('ITEST_PLANNER', 'AStar')     # nav2_params.yaml planner_plugins
PLANNER_PLUGIN = 'amr_navigation::AStarPlanner'            # 직접 구현 A*
COMPARE = ('NavFn', 'Smac')                                # 참고 비교 열
REQUEST_TIMEOUT_S = 5.0    # 요청 하나 (계획 한도 500 ms 의 10 배)
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')
COLUMNS = ['pair', 'planner', 'start_x', 'start_y', 'goal_x', 'goal_y', 'success', 'plan_ms',
           'length_m', 'straight_m', 'collision_free']


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements(use_perception=False) + [req.package('nav2_msgs')],
                'path planning')
    stack = Stack(CTX, *CTX.select())
    stack.system(use_localization=True, use_navigation=True, use_perception=False,
                 pose=(0.0, 0.0, 0.0))
    return stack.launch_description(), {'stack': stack}


def duration_ms(d) -> float:
    """builtin_interfaces/Duration → ms (없으면 NaN)."""
    if d is None:
        return math.nan
    return d.sec * 1e3 + d.nanosec * 1e-6


class TestPathPlanning(cases.ProbeCase):
    """50 쌍 계획 성공률 + 계획 시간."""

    CTX = CTX

    def _planner_params(self) -> dict:
        """planner_server 의 planner_plugins 와 <id>.plugin (rcl_interfaces GetParameters)."""
        from rcl_interfaces.srv import GetParameters
        res = self.probe.call(GetParameters, 'planner_server/get_parameters',
                              GetParameters.Request(names=['planner_plugins']),
                              self.timeout(10.0))
        plugins = list(res.values[0].string_array_value) if res and res.values else []
        out = {'planner_plugins': plugins}
        if plugins:
            res = self.probe.call(GetParameters, 'planner_server/get_parameters',
                                  GetParameters.Request(names=[f'{p}.plugin' for p in plugins]),
                                  self.timeout(10.0))
            if res is not None:
                out.update({p: v.string_value for p, v in zip(plugins, res.values)})
        return out

    def _plan(self, caller, planner: str, sx, sy, gx, gy, shapes):
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import ComputePathToPose
        goal = ComputePathToPose.Goal()
        goal.start = actions.pose_stamped(sx, sy)
        goal.goal = actions.pose_stamped(gx, gy)
        goal.use_start = True
        goal.planner_id = planner
        t0 = self.probe.wall()
        status, result = caller.call(goal, self.timeout(REQUEST_TIMEOUT_S))
        wall_ms = (self.probe.wall() - t0) * 1e3
        plan_ms = duration_ms(getattr(result, 'planning_time', None)) if result else math.nan
        path = (np.array([[p.pose.position.x, p.pose.position.y] for p in result.path.poses])
                if result is not None and result.path.poses else np.zeros((0, 2)))
        length = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))) \
            if len(path) > 1 else 0.0
        free = bool(len(path)) and not any(
            bool(np.any(s.contains(path[:, 0], path[:, 1]))) for s in shapes)
        reached = bool(len(path)) and math.hypot(path[-1, 0] - gx, path[-1, 1] - gy) <= 0.1
        success = status == GoalStatus.STATUS_SUCCEEDED and len(path) >= 2 and reached and free
        return success, (plan_ms if math.isfinite(plan_ms) else wall_ms), length, free

    def test_10_success_rate(self) -> None:
        from nav2_msgs.action import ComputePathToPose
        caller = actions.ActionCaller(self.probe, ComputePathToPose, 'compute_path_to_pose')
        self.assertTrue(caller.wait_server(self.timeout(300.0)), 'compute_path_to_pose 서버 없음')
        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        params = self._planner_params()
        self.measure('planner_server', params)
        self.check(f'planner {PLANNER_ID} is the own A* plugin', params.get(PLANNER_ID),
                   PLANNER_PLUGIN, params.get(PLANNER_ID) == PLANNER_PLUGIN)
        share = req.share_dir('amr_simulation')
        shapes = worldmap.footprints(share / 'worlds' / self.settings.world, share / 'models',
                                     config.scan_plane_height(), 'both')
        rng = np.random.default_rng(self.settings.seed)
        pts = worldmap.sample_free(shapes, BOUNDS, MARGIN, 2 * PAIRS, rng)
        self.assertEqual(len(pts), 2 * PAIRS, '자유 지점 표본 부족')
        compare = [p for p in COMPARE if p in params.get('planner_plugins', [])]
        rows, results = [], {p: [] for p in (PLANNER_ID,) + tuple(compare)}
        for i in range(PAIRS):
            (sx, sy), (gx, gy) = pts[2 * i], pts[2 * i + 1]
            if not self.budget_for(len(results) * REQUEST_TIMEOUT_S * self.settings.timeout_scale,
                                   f'pair {i}'):
                break
            for planner in results:
                ok, plan_ms, length, free = self._plan(caller, planner, sx, sy, gx, gy, shapes)
                results[planner].append((ok, plan_ms))
                rows.append([i, planner, sx, sy, gx, gy, int(ok), plan_ms, length,
                             math.hypot(gx - sx, gy - sy), int(free)])
            self.ctx.record.write_csv('plans.csv', COLUMNS, rows)
        summary = {}
        for planner, res in results.items():
            times = [t for ok, t in res if ok]
            summary[planner] = {
                'success': sum(ok for ok, _ in res), 'pairs': len(res),
                'plan_ms_mean': cases.fmt(float(np.mean(times)), 1) if times else None,
                'plan_ms_max': cases.fmt(float(np.max(times)), 1) if times else None}
        self.measure('planners', summary)
        mine = summary[PLANNER_ID]
        rate = mine['success'] / PAIRS
        failed = []
        for name, value, thr, ok, unit in (
                (f'planning success rate ({PLANNER_ID})', rate, SUCCESS_MIN, rate >= SUCCESS_MIN,
                 ''),
                (f'planning time max ({PLANNER_ID})', mine['plan_ms_max'], PLAN_MAX_MS,
                 mine['plan_ms_max'] is not None and mine['plan_ms_max'] <= PLAN_MAX_MS, 'ms')):
            self.ctx.record.check(name, cases.fmt(value), thr, bool(ok), unit)
            if not ok:
                failed.append(f'{name}: {cases.fmt(value)} {unit} (기준 {thr})')
        self.assertFalse(failed, '; '.join(failed) + f" — {mine['success']}/{PAIRS}")


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
