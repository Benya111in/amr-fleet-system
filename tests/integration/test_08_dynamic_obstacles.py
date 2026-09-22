"""
시나리오 08: 동적 장애물 회피 (명세 4.7 "30회 충돌 0건, 경로 이탈 1 m 이내, 원래 경로로 5 s 안에 복귀").

스택(system 프로필): Gazebo(동적 장애물 — actor 작업자들, 지게차 forklift_main, 셔틀 shuttle_amr) +
localization + navigation(DWA) + perception(obstacle_tracker_node 칼만 추적·TTC, safety_node).
amr_simulation 의 지면 진실 노드가 모든 동적 장애물을 판정한다 (warehouse.launch.py 가 띄운다):
  collision_monitor_node  로봇 발자국 ↔ 사람(원)·차량(사각형) 부호 거리, 접촉(< 0) 사건 (/sim/collision_monitor/*)
  obstacle_truth_node     장애물 위치·속도 (/sim/dynamic_obstacles/tracks, /info)
시행 30회 (ITEST_TRIALS, 줄이면 시행 수 판정이 실패): A(−4, −5) ↔ B(4, −9) 왕복 navigate_to_pose — 경로가 작업자
횡단선(worker_crossing, y = −7, 1.0 m/s)과 교차한다. 스폰 = A (명시). 시행마다 collision_monitor 를 reset.
  접촉     /sim/collision_monitor/events 중 이 로봇 사건 수 (사람·지게차·셔틀 모두)
  조우     시행의 최근접 동적 장애물 부호 거리 (summary per_obstacle_min) ≤ 2.0 m — 실제로 마주쳤는가
  TTC      GT 로봇 속도 · 장애물 지면 진실 속도로 등속 접촉 시간 최솟값 (기록)
  이탈     목표 직후 첫 plan(전역 경로) 대비 GT 수직 거리 최대
  복귀     이탈 > 0.3 m 구간마다 최대 이탈 시각 → 0.15 m 이하로 돌아온 시각
판정: 접촉 0, 최대 이탈 ≤ 1.0 m, 복귀 ≤ 5 s (모든 구간), 도달 ≥ 시행 − 1, 조우 ≥ 시행/6, 시행 ≥ 30.
로그 avoidance.csv (시행마다 갱신), contacts.csv (접촉마다 장애물·부호 거리·그 순간 GT 로봇 속도).
접촉은 로봇 속도로 나눠 기록한다 (> 0.05 m/s = 움직이며 부딪침, 이하 = 멈춘 로봇에 장애물이 닿음) — 판정은
그대로 '모든 접촉 0' 이고, 나눔은 원인 귀속(회피 계획 vs 정지 중 회피 동작 없음) 근거다.
"""

import json
import math
import os

from amr_itest import actions, cases, catalog, config, metrics, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry, Path
import numpy as np
import pytest
from std_msgs.msg import String
from std_srvs.srv import Trigger

CTX = Context(catalog.get(8))

SPEC_TRIALS = 30
TRIALS = max(1, int(os.environ.get('ITEST_TRIALS', str(SPEC_TRIALS))))
POINTS = [(-4.0, -5.0, -0.46), (4.0, -9.0, 2.68)]
DEVIATION_MAX = 1.0
RETURN_MAX_S = 5.0
DEV_OUT_M, DEV_BACK_M = 0.3, 0.15      # 이탈 구간 히스테리시스 (복귀 판정)
ENCOUNTER_M = 2.0                      # 조우 = 최근접 동적 장애물 부호 거리 ≤ 이 값
MIN_ENCOUNTERS = max(1, TRIALS // 6)
TRIAL_TIMEOUT_S = 90.0                 # 9 m 왕복 한 번 (0.5 m/s 18 s + 양보·재계획 여유)
TTC_PERIOD_S = 0.1                     # TTC 계산 간격 (GT 50 Hz 를 솎는다)
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization',
             'lifecycle_manager_navigation')
COLUMNS = ['trial', 'reached', 'time_s', 'contacts', 'min_distance_m', 'nearest', 'min_ttc_s',
           'max_deviation_m', 'episodes', 'return_s', 'contacts_robot_moving']
CONTACT_COLUMNS = ['trial', 'time', 'obstacle', 'distance_m', 'robot_speed_mps', 'robot_moving']


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    CTX.require(system_requirements()
                + [req.executable('amr_perception', 'obstacle_tracker_node', '동적 장애물 추적'),
                   req.executable('amr_simulation', 'collision_monitor_node.py', '접촉 판정'),
                   req.executable('amr_simulation', 'obstacle_truth_node.py', '장애물 지면 진실'),
                   req.package('nav2_msgs')], 'dynamic obstacle avoidance')
    stack = Stack(CTX, *CTX.select())
    stack.system(use_localization=True, use_navigation=True, use_perception=True,
                 pose=POINTS[0])
    return stack.launch_description(), {'stack': stack}


def obstacle_radii(info_json: str) -> dict:
    """/sim/dynamic_obstacles/info → {track_id: 외접 반지름 [m]} (사람 = radius, 차량 = 발자국 외접원)."""
    out = {}
    for o in json.loads(info_json or '[]'):
        if 'radius' in o:
            out[o['id']] = float(o['radius'])
        elif 'footprint' in o:
            x0, x1, y0, y1 = o['footprint']
            out[o['id']] = max(math.hypot(x, y) for x in (x0, x1) for y in (y0, y1))
    return out


class TestDynamicObstacles(cases.ProbeCase):
    """30회 왕복 — 접촉·조우·이탈·복귀·도달."""

    CTX = CTX

    def _summary(self, since: float):
        """리셋 뒤 collision_monitor summary 중 이 로봇 항목 (1 Hz — 다음 발행을 기다린다)."""
        ok = self.probe.wait_until(lambda: bool(self.summary.messages(since)),
                                   self.timeout(5.0), 0.05)
        if not ok:
            return None
        data = json.loads(self.summary.messages(since)[-1][1].data)
        return data.get(self.settings.robot)

    def _min_ttc(self, w0: float, radii: dict) -> float:
        best, last_t = math.inf, -math.inf
        tracks = self.tracks.messages(w0)
        if not tracks:
            return best
        times = np.array([t for t, _ in tracks])
        for t, m in self.gt.messages(w0):
            if t - last_t < TTC_PERIOD_S:
                continue
            last_t = t
            s = metrics.sample_from_odom(m)
            lin = m.twist.twist.linear          # child(base) 프레임 → 월드 (후진 부호 유지)
            c, sn = math.cos(s.yaw), math.sin(s.yaw)
            rvx, rvy = c * lin.x - sn * lin.y, sn * lin.x + c * lin.y
            k = int(np.clip(np.searchsorted(times, t), 0, len(tracks) - 1))
            for ob in tracks[k][1].obstacles:
                r = self.robot_r + radii.get(ob.track_id, 0.3)
                best = min(best, metrics.ttc_circle(ob.position.x - s.x, ob.position.y - s.y,
                                                    ob.velocity.x - rvx, ob.velocity.y - rvy, r))
        return best

    def test_10_trials(self) -> None:
        from action_msgs.msg import GoalStatus
        from amr_msgs.msg import TrackedObstacleArray
        from nav2_msgs.action import NavigateToPose
        rp = config.robot_params()
        type(self).robot_r = math.hypot(config.get(rp, 'robot.footprint_length') / 2.0,
                                        config.get(rp, 'robot.footprint_width') / 2.0)
        type(self).gt = self.probe.subscribe('ground_truth/odom', Odometry,
                                             keep_messages=20000)
        type(self).tracks = self.probe.subscribe('/sim/dynamic_obstacles/tracks',
                                                 TrackedObstacleArray, keep_messages=6000)
        type(self).summary = self.probe.subscribe('/sim/collision_monitor/summary', String,
                                                  keep_messages=200)
        events = self.probe.subscribe('/sim/collision_monitor/events', String,
                                      keep_messages=2000)
        info = self.probe.subscribe('/sim/dynamic_obstacles/info', String, 'latched')
        plan = self.probe.subscribe('plan', Path, keep_messages=50)
        nav = actions.ActionCaller(self.probe, NavigateToPose, 'navigate_to_pose')
        self.assertTrue(nav.wait_server(self.timeout(300.0)), 'navigate_to_pose 서버 없음')
        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        self.require_topic(info, 1, 60.0, '/sim/dynamic_obstacles/info (obstacle_truth_node)')
        self.require_topic(self.summary, 1, 60.0, '/sim/collision_monitor/summary')
        radii = obstacle_radii(info.last().data)
        self.measure('dynamic_obstacles', json.loads(info.last().data))
        self.wait_startup_still()
        rows, eps_all, contact_rows = [], [], []
        contacts = reached = encounters = moving_contacts = 0
        worst_dev, worst_return = 0.0, 0.0
        for trial in range(TRIALS):
            if not self.budget_for(TRIAL_TIMEOUT_S * self.settings.timeout_scale + 10.0,
                                   f'trial {trial}'):
                break
            x, y, yaw = POINTS[(trial + 1) % 2]
            res = self.probe.call(Trigger, '/sim/collision_monitor/reset', Trigger.Request(),
                                  self.timeout(5.0))
            self.assertTrue(res is not None and res.success, 'collision_monitor reset 실패')
            goal = NavigateToPose.Goal()
            goal.pose = actions.pose_stamped(x, y, yaw)
            w0, t0 = self.probe.wall(), self.probe.now()
            status, _ = nav.call(goal, self.timeout(TRIAL_TIMEOUT_S))
            ok = status == GoalStatus.STATUS_SUCCEEDED
            reached += int(ok)
            summ = self._summary(self.probe.wall()) or {}
            per = {k: v for k, v in summ.get('per_obstacle_min', {}).items()
                   if not k.startswith('amr_')}
            nearest = min(per, key=per.get) if per else ''
            dmin = per.get(nearest, math.inf)
            mine = [ev for ev in (json.loads(m.data) for _, m in events.messages(w0))
                    if ev.get('robot') == self.settings.robot]
            n_contacts = len(mine)
            contacts += n_contacts
            encounters += int(dmin <= ENCOUNTER_M)
            first_plan = next((m for _, m in plan.messages(w0) if m.poses), None)
            path = (np.array([[p.pose.position.x, p.pose.position.y] for p in first_plan.poses])
                    if first_plan is not None else np.zeros((0, 2)))
            track = [metrics.sample_from_odom(m) for _, m in self.gt.messages(w0)]
            speeds = metrics.speeds_at(track, [float(ev.get('t', math.nan)) for ev in mine])
            n_moving = sum(1 for v in speeds if v > metrics.CONTACT_MOVING_V)
            moving_contacts += n_moving
            contact_rows += [[trial, ev.get('t'), ev.get('obstacle'), ev.get('distance'), v,
                              int(v > metrics.CONTACT_MOVING_V) if math.isfinite(v) else '']
                             for ev, v in zip(mine, speeds)]
            if mine:
                self.ctx.record.write_csv('contacts.csv', CONTACT_COLUMNS, contact_rows)
            devs = [worldmap.polyline_distance(path, s.x, s.y) if len(path) else math.nan
                    for s in track]
            max_dev = max([d for d in devs if math.isfinite(d)], default=math.nan)
            eps = metrics.deviation_episodes([s.t for s in track], devs, DEV_OUT_M, DEV_BACK_M)
            t_last = track[-1].t if track else t0
            ret = max([e.return_time(t_last) for e in eps], default=0.0)
            eps_all += [e.returned and e.return_time(t_last) <= RETURN_MAX_S for e in eps]
            worst_dev = max(worst_dev, max_dev if math.isfinite(max_dev) else math.inf)
            worst_return = max(worst_return, ret if all(e.returned for e in eps) else math.inf)
            rows.append([trial, int(ok), self.probe.now() - t0, n_contacts, dmin, nearest,
                         self._min_ttc(w0, radii), max_dev, len(eps), ret, n_moving])
            self.ctx.record.write_csv('avoidance.csv', COLUMNS, rows)
        self.measure('summary', {'trials': len(rows), 'reached': reached,
                                 'contacts': contacts, 'contacts_robot_moving': moving_contacts,
                                 'encounters': encounters,
                                 'deviation_episodes': len(eps_all),
                                 'max_deviation_m': cases.fmt(worst_dev),
                                 'max_return_s': cases.fmt(worst_return, 2),
                                 'min_ttc_s': cases.fmt(min((r[6] for r in rows),
                                                            default=math.inf), 2)})
        failed = []
        for name, value, thr, passed, unit in (
                ('trials', len(rows), f'>= {SPEC_TRIALS}', len(rows) >= SPEC_TRIALS, ''),
                ('contacts (all dynamic obstacles)', contacts, 0, contacts == 0, ''),
                ('max path deviation', worst_dev, DEVIATION_MAX, worst_dev <= DEVIATION_MAX, 'm'),
                ('return to path', worst_return, RETURN_MAX_S,
                 all(eps_all) and worst_return <= RETURN_MAX_S, 's'),
                ('goals reached', reached, f'>= {len(rows) - 1}',
                 len(rows) > 0 and reached >= len(rows) - 1, ''),
                (f'encounters (nearest dynamic obstacle <= {ENCOUNTER_M} m)', encounters,
                 f'>= {MIN_ENCOUNTERS}', encounters >= MIN_ENCOUNTERS, '')):
            self.ctx.record.check(name, cases.fmt(value), thr, bool(passed), unit)
            if not passed:
                failed.append(f'{name}: {cases.fmt(value)} {unit} (기준 {thr})')
        self.assertFalse(failed, '; '.join(failed))


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
