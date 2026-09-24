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
contacts.csv 는 그 귀속에 필요한 기하도 같이 남긴다 (판정에는 쓰지 않는다):
  obstacle_speed_mps/obstacle_heading_deg  접촉 순간 장애물 지면 진실 속도·진행 방향
  lane_lateral_m/lane_along_m              장애물 진행축 기준 로봇 좌표 (가로: 왼쪽 +, 세로: 앞 +)
                                           → |가로| ≤ 두 반지름 합이면 로봇이 장애물 차선 안에 있었다
  yield_state/stop_distance_m              접촉 직전 제어 주기의 dwa/stats [11]·[12] (정지선 판단)
  cmd_v_mps / gate_in_v_mps / gate_out_v_mps
                                           계획기(dwa/stats [7]) → 속도 프로파일러(cmd_vel_smoothed)
                                           → 안전 게이트(cmd_vel) 로 이어지는 같은 시각의 명령 속도.
                                           셋이 0 이면 계획기가 멈춘 것, 앞은 음수인데 뒤가 0 이면
                                           그 단계가 후진 이탈을 막은 것이다
dwa/stats 에는 스탬프가 없어 지면 진실 오도메트리의 (수신 벽시계, sim 스탬프) 대응으로 맞춘다 — 제어
주기 한 번 정도의 오차가 있으므로 근거용이지 판정용이 아니다.
"""

import json
import math
import os

from amr_itest import actions, cases, catalog, config, metrics, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
from amr_itest.stack import Stack, system_requirements
from geometry_msgs.msg import Twist
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry, Path
import numpy as np
import pytest
from std_msgs.msg import Float64MultiArray, String
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
EPISODE_COLUMNS = ['trial', 't_start', 't_peak', 'peak_m', 't_end', 'return_s',
                   'stopped_frac', 'mean_v_mps', 'max_v_mps',
                   'yield_frac', 'cmd_v_mean', 'gate_out_v_mean', 'vo_rejected_mean']

COLUMNS = ['trial', 'reached', 'time_s', 'contacts', 'min_distance_m', 'nearest', 'min_ttc_s',
           'max_deviation_m', 'episodes', 'return_s', 'contacts_robot_moving',
           'episodes_open', 'open_peak_m', 'open_peak_t', 'dev_at_end_m']
CONTACT_COLUMNS = ['trial', 'time', 'obstacle', 'distance_m', 'robot_speed_mps', 'robot_moving',
                   'obstacle_speed_mps', 'obstacle_heading_deg', 'lane_lateral_m', 'lane_along_m',
                   'yield_state', 'stop_distance_m', 'cmd_v_mps',
                   'gate_in_v_mps', 'gate_out_v_mps']
ATTRIB_GAP_S = 0.3                     # 접촉 시각 ↔ 트랙·제어 표본 허용 시차 (넘으면 빈 칸)
YIELD_STATES = {0: 'clear', 1: 'yield', 2: 'committed'}   # dwa/stats [11]
DEFAULT_OBSTACLE_R = 0.3               # [m] /info 에 없는 장애물의 반지름 가정


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


def obstacle_ids(info_json: str) -> dict:
    """/sim/dynamic_obstacles/info → {모델 이름: track_id} (접촉 사건의 이름 → 지면 진실 트랙)."""
    return {o['name']: o['id'] for o in json.loads(info_json or '[]') if 'name' in o}


def _cmd_at(w_arr, msgs, w: float) -> float:
    """벽시계 w 직전에 발행된 Twist 의 선속도 (스탬프가 없어 수신 시각으로 맞춘다)."""
    if not len(w_arr):
        return math.nan
    j = int(np.searchsorted(w_arr, w)) - 1
    if j < 0 or w - w_arr[j] > ATTRIB_GAP_S:
        return math.nan
    return msgs[j][1].linear.x


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

    def _track_at(self, tr_t, tr, name: str, t: float):
        """접촉 시각 t 에 가장 가까운 지면 진실 트랙 표본에서 그 장애물 (없으면 None)."""
        if not len(tr_t) or not math.isfinite(t):
            return None
        k = int(np.clip(np.searchsorted(tr_t, t), 0, len(tr) - 1))
        if abs(tr_t[k] - t) > ATTRIB_GAP_S:
            return None
        tid = self.ids.get(name)
        return next((o for o in tr[k][1].obstacles if o.track_id == tid), None)

    def _attribution(self, w0: float, events: list, track: list) -> list:
        """접촉마다 [장애물 속도, 진행 방향, 차선 가로·세로, yield 상태, 정지선 거리] (CSV 근거)."""
        tr = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m)
              for _, m in self.tracks.messages(w0)]
        tr_t = np.array([t for t, _ in tr])
        gt = self.gt.messages(w0)
        wall = np.array([w for w, _ in gt])                       # 수신 벽시계
        sim = np.array([metrics.sample_from_odom(m).t for _, m in gt])   # 같은 표본의 sim 스탬프
        stats = self.stats.messages(w0)
        st_w = np.array([w for w, _ in stats])
        gi = self.gate_in.messages(w0)
        gi_w = np.array([w for w, _ in gi])
        go = self.gate_out.messages(w0)
        go_w = np.array([w for w, _ in go])
        rows = []
        for ev in events:
            t = float(ev.get('t', math.nan))
            row = [math.nan] * 4 + ['', math.nan, math.nan, math.nan, math.nan]
            ob = self._track_at(tr_t, tr, ev.get('obstacle'), t)
            if ob is not None:
                row[0] = math.hypot(ob.velocity.x, ob.velocity.y)
                row[1] = math.degrees(math.atan2(ob.velocity.y, ob.velocity.x))
                s = metrics.interpolate(track, t)
                if s is not None:
                    along, lat = metrics.lane_coords(
                        ob.position.x, ob.position.y, ob.velocity.x, ob.velocity.y, s.x, s.y)
                    row[2], row[3] = lat, along
            if len(st_w) and len(sim) > 1 and sim[0] <= t <= sim[-1]:   # 외삽 금지 (interp 는 자른다)
                w = float(np.interp(t, sim, wall))   # 접촉 시각 → 벽시계 (stats 에는 스탬프가 없다)
                j = int(np.searchsorted(st_w, w)) - 1                   # 접촉 직전 제어 주기
                d = list(stats[j][1].data) if 0 <= j < len(stats) else []
                if len(d) > 12 and w - st_w[j] <= ATTRIB_GAP_S:
                    row[4] = YIELD_STATES.get(int(d[11]), int(d[11]))
                    row[5] = d[12] if d[12] >= 0.0 else math.inf
                    row[6] = d[7]        # 그 주기에 계획기가 낸 속도 (음수 = 후진 이탈 명령)
                row[7] = _cmd_at(gi_w, gi, w)
                row[8] = _cmd_at(go_w, go, w)
            rows.append(row)
        return rows

    def _episode_row(self, ep, track, t_last: float, w0: float) -> list:
        """이탈 구간의 거동: 최대 이탈 → 복귀 사이 로봇이 멈춰 있었는지, 무엇이 세웠는지."""
        t_end = ep.t_end if ep.t_end is not None else t_last
        vs = [abs(s.v) for s in track if ep.t_peak <= s.t <= t_end]
        stopped = sum(1 for v in vs if v <= metrics.CONTACT_MOVING_V) / len(vs) if vs else math.nan
        return [ep.t_start, ep.t_peak, ep.peak, ep.t_end if ep.returned else math.nan,
                ep.return_time(t_last), stopped,
                sum(vs) / len(vs) if vs else math.nan,
                max(vs, default=math.nan)] + self._cause_stats(w0, ep.t_peak, t_end)

    def _cause_stats(self, w0: float, t0: float, t1: float) -> list:
        """구간 [t0, t1] 의 [양보 비율, 계획기 명령 평균, 게이트 출력 평균, VO 기각 평균]."""
        gt = self.gt.messages(w0)
        if len(gt) < 2:
            return [math.nan] * 4
        wall = np.array([w for w, _ in gt])
        sim = np.array([metrics.sample_from_odom(m).t for _, m in gt])
        if not (sim[0] <= t0 <= sim[-1]):        # 외삽 금지
            return [math.nan] * 4
        w_lo, w_hi = float(np.interp(t0, sim, wall)), float(np.interp(min(t1, sim[-1]), sim, wall))
        d = [list(m.data) for w, m in self.stats.messages(w0) if w_lo <= w <= w_hi]
        d = [x for x in d if len(x) > 12]
        gate = [m.linear.x for w, m in self.gate_out.messages(w0) if w_lo <= w <= w_hi]
        if not d:
            return [math.nan] * 4
        return [sum(1 for x in d if int(x[11]) != 0) / len(d),
                sum(x[7] for x in d) / len(d),
                sum(gate) / len(gate) if gate else math.nan,
                sum(x[4] for x in d) / len(d)]

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
        type(self).stats = self.probe.subscribe('dwa/stats', Float64MultiArray,
                                                keep_messages=4000)
        type(self).gate_in = self.probe.subscribe('cmd_vel_smoothed', Twist,
                                                  keep_messages=8000)
        type(self).gate_out = self.probe.subscribe('cmd_vel', Twist, keep_messages=8000)
        nav = actions.ActionCaller(self.probe, NavigateToPose, 'navigate_to_pose')
        self.assertTrue(nav.wait_server(self.timeout(300.0)), 'navigate_to_pose 서버 없음')
        self.wait_lifecycle_active(LIFECYCLE, 300.0)
        self.check_map_registration()
        self.require_topic(info, 1, 60.0, '/sim/dynamic_obstacles/info (obstacle_truth_node)')
        self.require_topic(self.summary, 1, 60.0, '/sim/collision_monitor/summary')
        radii = obstacle_radii(info.last().data)
        type(self).ids = obstacle_ids(info.last().data)
        self.measure('dynamic_obstacles', json.loads(info.last().data))
        self.wait_startup_still()
        rows, eps_all, contact_rows, ep_rows = [], [], [], []
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
            attrib = self._attribution(w0, mine, track) if mine else []
            contact_rows += [[trial, ev.get('t'), ev.get('obstacle'), ev.get('distance'), v,
                              int(v > metrics.CONTACT_MOVING_V) if math.isfinite(v) else ''] + a
                             for ev, v, a in zip(mine, speeds, attrib)]
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
            # 복귀하지 못한 구간이 있으면 그 자리(최대 이탈 크기·시각)와 시행 끝의 이탈을 남긴다 —
            # "5 s 초과" 와 "시행이 끝날 때까지 미복귀" 는 원인이 다르다 (후자는 목표 도착 시점에
            # 원래 경로에서 back_thr 밖인 경우가 많다)
            ep_rows += [[trial] + self._episode_row(e, track, t_last, w0) for e in eps]
            if eps:
                self.ctx.record.write_csv('episodes.csv', EPISODE_COLUMNS, ep_rows)
            open_eps = [e for e in eps if not e.returned]
            dev_end = next((d for d in reversed(devs) if math.isfinite(d)), math.nan)
            rows.append([trial, int(ok), self.probe.now() - t0, n_contacts, dmin, nearest,
                         self._min_ttc(w0, radii), max_dev, len(eps), ret, n_moving,
                         len(open_eps),
                         max((e.peak for e in open_eps), default=math.nan),
                         max((e.t_peak for e in open_eps), default=math.nan),
                         dev_end])
            self.ctx.record.write_csv('avoidance.csv', COLUMNS, rows)
        in_lane = sum(1 for r in contact_rows
                      if isinstance(r[8], float) and math.isfinite(r[8])
                      and abs(r[8]) <= self.robot_r + radii.get(self.ids.get(r[2]),
                                                                DEFAULT_OBSTACLE_R))
        self.measure('summary', {'trials': len(rows), 'reached': reached,
                                 'contacts': contacts, 'contacts_robot_moving': moving_contacts,
                                 'contacts_in_obstacle_lane': in_lane,
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
