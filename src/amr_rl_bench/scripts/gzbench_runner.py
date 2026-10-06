#!/usr/bin/env python3
"""
Gazebo 다회 에피소드 실행기 — 한 Gazebo 세션에서 시나리오를 연달아 돈다 (보너스 1 A5, 완료 정의 D4·D5).

    ros2 run amr_rl_bench gzbench_runner.py --scenarios <scenarios_v1.json> --select smoke \
        --controller rl --policy-id v12_s1 --out /runs/<tag> --ros-args -r __ns:=/amr_01 \
        -p use_sim_time:=true

에피소드마다 (README §2):
  1. 정답 자세로 순간 이동 (ign service /world/<world>/set_pose) — 지면 진실로 도착 확인 (0.05 m·0.05 rad, 3 회)
  2. 위치추정 초기화 = 운영자가 하듯 initialpose(정답 자세) + map EKF set_pose + AMCL 무이동 갱신 (amr_itest
     actions.seed_pose 와 같은 묶음). AMCL 을 그대로 쓰고, localization/lost 가 False 이고 추정(TF map→base)이
     정답과 0.10 m·0.05 rad 안으로 2 s 유지될 때까지 기다린다 (b1-plugin 다음 ⑦ — 첫 목표를 LOST 상태로 보내지 않는다)
  3. 코스트맵 비우기 (전역·지역 clear_entirely) 뒤 1 s
  4. navigate_to_pose (BT 기본 제어기 = 런치가 정한 DWA 또는 RL) → 종료: Nav2 종료 상태, 접촉(즉시 취소), 시간 상한
     min(120, 20 + 3L) s sim (취소), 감시견(sim 시각 정지 30 s 벽시계 → error)
  5. 정착(지면 진실 |v| < 0.01·|ω| < 0.02 또는 3 s) 뒤 최종 자세 → 지표(amr_rl_bench.metrics) → CSV·에피소드 원자료
점검 탐침 (--probe scan,step): 스캔 규약 표본·구동 계단 응답을 에피소드 앞에 한 번 (전이 점검 2·5).
"""

import argparse
import gzip
import json
import math
import os
import subprocess
import threading
import time

from amr_rl_bench import metrics as M
from amr_rl_bench import records as R
from amr_rl_bench import scenarios as S
from amr_rl_bench.world_geom import StaticWorld
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, Float64MultiArray, UInt8, UInt32
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    """쿼터니언 → yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def loadavg():
    """'1 5 15' loadavg."""
    try:
        return ' '.join(open('/proc/loadavg').read().split()[:3])
    except OSError:
        return 'n/a'


def ign_set_pose(world, model, x, y, yaw, z=0.02, timeout_ms=5000):
    """순간 이동: ign service set_pose (tests/integration/amr_itest/gz.py 와 같은 요청). 반환 (성공, 출력)."""
    req = (f'name: "{model}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    cmd = ['ign', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'ignition.msgs.Pose',
           '--reptype', 'ignition.msgs.Boolean', '--timeout', str(timeout_ms), '--req', req]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000 + 5,
                             check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    text = (out.stdout + out.stderr).strip()
    return out.returncode == 0 and 'data: true' in text, text


class Bench(Node):
    """기록기 + Nav2·위치추정·Gazebo 조작."""

    def __init__(self, args):
        super().__init__('gzbench_runner')
        self.args = args
        self.lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world = StaticWorld.from_sdf(args.world_sdf, args.models_dir)
        self.rec = None
        self.gt_last = None
        self.lost_last = None
        self.scan_last = None
        self.scan_raw_last = None
        self.cm_contacts_last = None
        sub = self.create_subscription
        sub(Odometry, 'ground_truth/odom', self._gt, 50)
        sub(Odometry, 'odometry/filtered', self._odom, 50)
        sub(Path, 'plan', self._plan, 10)
        sub(Twist, 'cmd_vel_nav', lambda m: self._cmd('cmd_nav', m, True), 50)
        sub(Twist, 'cmd_vel_smoothed', lambda m: self._cmd('cmd_in', m), 50)
        sub(Twist, 'cmd_vel', lambda m: self._cmd('cmd_out', m), 50)
        sub(Float64MultiArray, 'rl_controller/stats', self._stats, 50)
        sub(Float64MultiArray, 'velocity_profiler/state', self._prof, 50)
        sub(UInt8, 'safety/zone', self._zone, LATCHED)
        sub(Bool, 'localization/lost', self._lost, LATCHED)
        sub(Float64, 'collision_monitor/min_distance', self._cm_min, 50)
        sub(UInt32, 'collision_monitor/contacts', self._cm_cnt, 10)
        sub(LaserScan, 'scan_filtered', self._scan, qos_profile_sensor_data)
        sub(LaserScan, 'scan', self._scan_raw, qos_profile_sensor_data)
        sub(DiagnosticArray, 'diagnostics', self._diag, 20)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.clear_g = self.create_client(ClearEntireCostmap,
                                          'global_costmap/clear_entirely_global_costmap')
        self.clear_l = self.create_client(ClearEntireCostmap,
                                          'local_costmap/clear_entirely_local_costmap')
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')
        try:
            from robot_localization.srv import SetPose
            self.SetPose = SetPose
            self.ekf_set = self.create_client(SetPose, 'ekf_filter_node_map/set_pose')
        except ImportError:
            self.SetPose, self.ekf_set = None, None

    # ------------------------------------------------------------ 기록 콜백
    def now(self):
        """노드 시계 (sim) [s]."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _put(self, key, item):
        with self.lock:
            if self.rec is not None:
                self.rec[key].append(item)

    def _gt(self, m):
        p = m.pose.pose
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        row = [t, p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.linear.x,
               m.twist.twist.angular.z]
        self.gt_last = row
        self._put('gt', row)
        if self.rec is not None:
            self._put('clear_static', [t, self.world.footprint_clearance(row[1], row[2], row[3])])

    def _odom(self, m):
        # 제어기가 받는 측정 속도 (controller_server odom_topic = odometry/filtered, EKF) — 전이 점검 5 의 관측 사슬
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self._put('odom', [t, m.twist.twist.linear.x, m.twist.twist.angular.z])

    def _plan(self, m):
        pts = [[round(ps.pose.position.x, 4), round(ps.pose.position.y, 4)] for ps in m.poses]
        self._put('plans', [self.now(), pts])

    def _cmd(self, key, m, wall=False):
        row = [self.now(), time.monotonic(), m.linear.x, m.angular.z] if wall else \
            [self.now(), m.linear.x, m.angular.z]
        self._put(key, row)

    def _stats(self, m):
        self._put('stats', [self.now(), time.monotonic(), list(m.data)])

    def _prof(self, m):
        self._put('prof', [self.now(), list(m.data)])

    def _zone(self, m):
        self._put('zone', [self.now(), int(m.data)])
        self.zone_last = [self.now(), int(m.data)]

    def _lost(self, m):
        self.lost_last = bool(m.data)
        self._put('lost', [self.now(), bool(m.data)])

    def _cm_min(self, m):
        self._put('cm_min', [self.now(), float(m.data)])

    def _cm_cnt(self, m):
        self.cm_contacts_last = int(m.data)
        self._put('cm_contacts', [self.now(), int(m.data)])

    def _scan(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.scan_last = m
        self._put('scan_stamps', [self.now(), st])

    def _scan_raw(self, m):
        self.scan_raw_last = m

    def _diag(self, m):
        for s in m.status:
            if s.name.endswith('/safety_node'):
                kv = {v.key: v.value for v in s.values}
                self._put('diag', [self.now(), kv.get('zone'), kv.get('speed_limit_mps'),
                                   kv.get('proximity_stop'), kv.get('min_distance_m')])

    # ------------------------------------------------------------ 도우미
    def start_record(self):
        """새 기록 버퍼."""
        with self.lock:
            self.rec = {k: [] for k in ('gt', 'odom', 'clear_static', 'plans', 'cmd_nav',
                                        'cmd_in', 'cmd_out', 'stats', 'prof', 'zone', 'lost',
                                        'cm_min', 'cm_contacts', 'scan_stamps', 'diag', 'est')}
            # 래치 토픽은 구간 시작 전 마지막 값을 넣어 둔다
            if getattr(self, 'zone_last', None):
                self.rec['zone'].append(list(self.zone_last))
            if self.lost_last is not None:
                self.rec['lost'].append([self.now(), self.lost_last])
            if self.cm_contacts_last is not None:
                self.rec['cm_contacts'].append([self.now(), self.cm_contacts_last])

    def stop_record(self):
        """버퍼를 돌려주고 기록을 멈춘다."""
        with self.lock:
            r, self.rec = self.rec, None
        return r

    def est_pose(self):
        """TF map → base_footprint (x, y, yaw) 또는 None."""
        try:
            t = self.tf_buffer.lookup_transform(self.args.map_frame, self.args.base_frame,
                                                rclpy.time.Time())
        except Exception:  # noqa: BLE001 — TF 가 아직 없으면 None
            return None
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, yaw_of(q)

    def sleep_sim(self, dt, wall_cap=None):
        """노드 시계(sim)로 dt 쉰다 (벽시계 상한 wall_cap, 기본 dt/0.1 + 5). 반환: sim 이 흘렀나."""
        t0, w0 = self.now(), time.monotonic()
        cap = wall_cap if wall_cap is not None else dt / 0.1 + 5.0
        while self.now() - t0 < dt:
            if time.monotonic() - w0 > cap:
                return False
            time.sleep(0.01)
        return True

    def wait_until(self, pred, sim_timeout, wall_cap=None, poll=0.05):
        """pred() 가 참이 될 때까지 (sim 시각 상한). 반환 bool."""
        t0, w0 = self.now(), time.monotonic()
        cap = wall_cap if wall_cap is not None else sim_timeout / 0.1 + 10.0
        while self.now() - t0 < sim_timeout and time.monotonic() - w0 < cap:
            if pred():
                return True
            time.sleep(poll)
        return pred()

    def call(self, client, req, timeout=5.0):
        """서비스 호출 (벽시계 timeout). 결과 또는 None."""
        if client is None or not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        w0 = time.monotonic()
        while not fut.done() and time.monotonic() - w0 < timeout:
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def pose_cov(self, x, y, yaw, sigma_xy=0.05, sigma_yaw_deg=3.0):
        """초기 자세 메시지 initialpose (amr_itest actions.initial_pose 와 같은 공분산)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.args.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = sigma_xy ** 2
        msg.pose.covariance[35] = math.radians(sigma_yaw_deg) ** 2
        return msg

    def seed(self, x, y, yaw):
        """자세 알려 주기: initialpose ×3 + map EKF set_pose + AMCL 무이동 갱신 ×2 (actions.seed_pose)."""
        for _ in range(3):
            self.initialpose.publish(self.pose_cov(x, y, yaw))
            time.sleep(0.2)
        ok = False
        if self.SetPose is not None:
            req = self.SetPose.Request()
            req.pose = self.pose_cov(x, y, yaw)
            ok = self.call(self.ekf_set, req, 5.0) is not None
        n = sum(1 for _ in range(2) if self.call(self.nomotion, Empty.Request(), 2.0) is not None)
        return {'ekf_set_pose': ok, 'nomotion': n}

    def loc_error(self, x, y, yaw):
        """추정 대 (x, y, yaw) — (거리, |yaw 차|) 또는 (inf, inf)."""
        e = self.est_pose()
        if e is None:
            return math.inf, math.inf
        return math.hypot(e[0] - x, e[1] - y), abs(math.remainder(e[2] - yaw, 2 * math.pi))

    def stopped(self, v=0.01, w=0.02):
        """지면 진실로 멈췄나."""
        g = self.gt_last
        return g is not None and abs(g[4]) < v and abs(g[5]) < w

    # ------------------------------------------------------------ 에피소드
    def place(self, sc):
        """
        순간 이동 + 위치추정 초기화 + 코스트맵 비우기. 반환 (성공, 정보 dict).

        순간 이동 직후 바로(0.3 s sim) 자세를 알려 준다 — 납치 감시는 스캔 정합 거부가 창 1 s 넘게 쌓이고 의심이 1 s
        이어져야 LOST 를 선언하므로 그 전에 외부 자세 재설정(on_external_pose_reset)이 도착하게 한다. LOST 가 먼저
        나면 감시가 회전 복구를 돌려 시작 자세가 흐트러진다.
        """
        x, y, yaw = sc['start']
        info = {'teleport_tries': 0, 'seed_tries': 0}
        ok = False
        for k in range(3):
            info['teleport_tries'] = k + 1
            good_tp, out = ign_set_pose(self.args.world_name, self.args.robot, x, y, yaw)
            if not good_tp:
                info['teleport_out'] = out[-200:]
                time.sleep(1.0)
                continue
            self.sleep_sim(0.3)
            info['seed'] = self.seed(x, y, yaw)
            info['seed_tries'] += 1
            self.sleep_sim(0.5)
            g = self.gt_last
            if g is not None:
                info['teleport_err_m'] = math.hypot(g[1] - x, g[2] - y)
                info['teleport_err_yaw'] = abs(math.remainder(g[3] - yaw, 2 * math.pi))
                if info['teleport_err_m'] <= 0.05 and info['teleport_err_yaw'] <= 0.05:
                    ok = True
                    break
        if not ok:
            return False, dict(info, error='teleport')
        for k in range(2):
            hold = {'since': None}

            def good():
                d, a = self.loc_error(x, y, yaw)
                ok_now = (d <= 0.10 and a <= 0.05 and self.lost_last is not True and
                          self.stopped())
                now = self.now()
                if not ok_now:
                    hold['since'] = None
                    return False
                hold['since'] = hold['since'] if hold['since'] is not None else now
                return now - hold['since'] >= 2.0
            if self.wait_until(good, 30.0):
                break
            if k == 0:   # 한 번 더 알려 준다 (지면 진실 그대로 — 순간 이동은 이미 확인)
                info['seed_retry'] = self.seed(x, y, yaw)
                info['seed_tries'] += 1
        else:
            d, a = self.loc_error(x, y, yaw)
            return False, dict(info, error=f'localization (err {d:.3f} m {a:.3f} rad, '
                                           f'lost {self.lost_last})')
        info['loc_err_start_m'], info['loc_err_start_yaw'] = self.loc_error(x, y, yaw)
        info['lost_before_goal'] = self.lost_last
        for c in (self.clear_g, self.clear_l):
            self.call(c, ClearEntireCostmap.Request(), 5.0)
        self.sleep_sim(1.0)
        return True, info

    def send_goal(self, sc):
        """navigate_to_pose 보내기 (수락 확인 3 회). 반환 (handle|None, 시도 수)."""
        gx, gy, gyaw = sc['goal']
        for k in range(3):
            goal = NavigateToPose.Goal()
            ps = PoseStamped()
            ps.header.frame_id = self.args.map_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = float(gx), float(gy)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
            goal.pose = ps
            fut = self.nav.send_goal_async(goal)
            w0 = time.monotonic()
            while not fut.done() and time.monotonic() - w0 < 10.0:
                time.sleep(0.01)
            h = fut.result() if fut.done() else None
            if h is not None and h.accepted:
                return h, k + 1
            self.get_logger().warn(f'목표 수락 실패 (시도 {k + 1}) — 다시 보낸다')
            time.sleep(1.0)
        return None, 3

    def run_episode(self, sc, exec_index):
        """시나리오 하나 → (gzbench 행, 원자료 dict)."""
        a = self.args
        meta = {'run_tag': a.run_tag, 'exec_index': exec_index, 'scenario_id': sc['id'],
                'scenario_set': R.scenario_set_name(sc['set']), 'difficulty': sc['set'],
                'scenario_seed': sc['scenario_seed'], 'controller': a.controller,
                'arm': a.arm or R.ARM_OF.get(a.controller), 'policy_id': a.policy_id,
                'handover_radius': a.handover_radius, 'commit': a.commit,
                'rtf_target': a.rtf_target, 'path_ref_m': sc['path_length_m'],
                'time_limit_s': sc['time_limit_s'], 'goal_straight_m': sc.get('goal_straight_m'),
                'started_wall': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'load_start': loadavg()}
        w_start = time.monotonic()
        # 이전 에피소드가 남긴 움직임이 멈출 때까지 (최대 5 s sim)
        self.wait_until(self.stopped, 5.0)
        ok, info = self.place(sc)
        meta.update({'teleport_err_m': info.get('teleport_err_m'),
                     'seed_tries': info.get('seed_tries'),
                     'loc_err_start_m': info.get('loc_err_start_m'),
                     'loc_err_start_yaw': info.get('loc_err_start_yaw')})
        w_setup = time.monotonic()
        raw = {'scenario': sc, 'meta': meta, 'place': info}
        if not ok:
            meta.update({'setup_ok': False, 'setup_error': info.get('error'),
                         'wall_setup_s': w_setup - w_start, 'load_end': loadavg()})
            err = {'outcome_contract': 'error', 'outcome_system': 'error'}
            return R.gzbench_row(meta, err), raw
        self.start_record()
        handle, tries = self.send_goal(sc)
        meta['accept_tries'] = tries
        if handle is None:
            self.stop_record()
            meta.update({'setup_ok': False, 'setup_error': 'goal not accepted',
                         'wall_setup_s': w_setup - w_start, 'load_end': loadavg()})
            err = {'outcome_contract': 'error', 'outcome_system': 'error'}
            return R.gzbench_row(meta, err), raw
        t0, w0 = self.now(), time.monotonic()
        res_fut = handle.get_result_async()
        reason, status = '', None
        last_sim, last_sim_wall = t0, w0
        while True:
            time.sleep(0.02)
            now = self.now()
            if now > last_sim:
                last_sim, last_sim_wall = now, time.monotonic()
            if res_fut.done():
                status = res_fut.result().status
                break
            g = self.gt_last
            if g is not None and self.world.footprint_clearance(g[1], g[2], g[3]) < 0.0:
                reason = 'collision'
            elif now - t0 > sc['time_limit_s']:
                reason = 'timeout'
            elif time.monotonic() - last_sim_wall > 30.0:
                reason = 'watchdog'
            if reason:
                handle.cancel_goal_async()
                w1 = time.monotonic()
                while not res_fut.done() and time.monotonic() - w1 < 10.0:
                    time.sleep(0.02)
                status = res_fut.result().status if res_fut.done() else None
                break
            # 추정 자세 표본 (20 Hz 근처)
            e = self.est_pose()
            if e is not None:
                self._put('est', [now, e[0], e[1], e[2]])
        t_end, w_end = self.now(), time.monotonic()
        # 정착: 멈출 때까지 (최대 3 s sim) — 그동안도 기록한다 (최종 자세 진단)
        self.wait_until(self.stopped, 3.0)
        time.sleep(0.2)
        e = self.est_pose()
        g = self.gt_last
        rec = self.stop_record()
        ep = dict(rec)
        ep.update({'goal': sc['goal'], 't0': t0, 't_end': t_end, 't_limit': sc['time_limit_s'],
                   'nav2_status': status, 'cancel_reason': reason,
                   'final': {'gt': g[1:6] if g else None, 'est': list(e) if e else None},
                   'wall': {'t0': w0, 't_end': w_end}})
        try:
            mt = M.episode_metrics(ep)
        except Exception as exc:  # noqa: BLE001 — 지표 계산 실패도 행으로 남긴다
            mt = {'outcome_contract': 'error', 'outcome_system': 'error'}
            meta['note'] = f'metrics error: {exc!r}'[:200]
        meta.update({'setup_ok': True, 'cancel_reason': reason,
                     'wall_setup_s': w_setup - w_start, 'wall_nav_s': w_end - w0,
                     'wall_episode_s': time.monotonic() - w_start, 'load_end': loadavg()})
        raw.update({'episode': ep, 'metrics': mt})
        return R.gzbench_row(meta, mt), raw

    # ------------------------------------------------------------ 점검 탐침
    def probe_scan(self, out_dir, pose, n=20, name='probe_scan.json'):
        """전이 점검 2: scan_filtered·scan 표본 n 개와 그 시각 지면 진실 (분석은 rl/eval/gazebo)."""
        ok, info = self.place({'start': pose})
        rows = []
        last = None
        w0 = time.monotonic()
        while len(rows) < n and time.monotonic() - w0 < 120:
            m = self.scan_last
            if m is not None and m is not last:
                last = m
                st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                raw = self.scan_raw_last
                rows.append({'stamp': st, 'gt': self.gt_last, 'est': self.est_pose(),
                             'frame_id': m.header.frame_id, 'angle_min': m.angle_min,
                             'angle_max': m.angle_max, 'angle_increment': m.angle_increment,
                             'range_min': m.range_min, 'range_max': m.range_max,
                             'time_increment': m.time_increment, 'scan_time': m.scan_time,
                             'ranges': [None if not math.isfinite(r) else r for r in m.ranges],
                             'n_inf': sum(1 for r in m.ranges if math.isinf(r)),
                             'n_nan': sum(1 for r in m.ranges if math.isnan(r)),
                             'raw': None if raw is None else {
                                 'frame_id': raw.header.frame_id, 'n': len(raw.ranges),
                                 'angle_min': raw.angle_min,
                                 'angle_increment': raw.angle_increment,
                                 'ranges': [None if not math.isfinite(r) else r
                                            for r in raw.ranges]}})
            time.sleep(0.02)
        R.write_json(os.path.join(out_dir, name),
                     {'pose': pose, 'place_ok': ok, 'place': info, 'scans': rows})
        return ok and len(rows) == n

    def probe_step(self, out_dir, pose):
        """전이 점검 5: cmd_vel_nav 계단 (v 0.5 m/s 3 s, ω 1.0 rad/s 2 s) → 프로파일러·safety·지면 진실 기록."""
        ok, info = self.place({'start': pose})
        self.start_record()
        phases = [(1.0, 0.0, 0.0), (3.0, 0.5, 0.0), (2.5, 0.0, 0.0), (2.0, 0.0, 1.0),
                  (2.5, 0.0, 0.0)]
        marks = []
        for dur, v, w in phases:
            t_ph = self.now()
            marks.append([t_ph, v, w])
            while self.now() - t_ph < dur:
                msg = Twist()
                msg.linear.x, msg.angular.z = v, w
                self.cmd_pub.publish(msg)
                time.sleep(0.05)
        rec = self.stop_record()
        R.write_json(os.path.join(out_dir, 'probe_step.json'),
                     {'pose': pose, 'place_ok': ok, 'place': info, 'phases': marks,
                      'gt': rec['gt'], 'odom': rec['odom'], 'cmd_nav': rec['cmd_nav'],
                      'cmd_in': rec['cmd_in'],
                      'cmd_out': rec['cmd_out'], 'prof': rec['prof'],
                      'prof_fields': 'v_target w_target v_ref w_ref a_ref v_out w_out v_meas '
                                     'w_meas jerk resynced jerk_out a_out correction '
                                     'resync_count fresh'})
        return ok


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenarios', required=True)
    ap.add_argument('--select', default='smoke')
    ap.add_argument('--start', type=int, default=0)
    ap.add_argument('--count', type=int, default=None)
    ap.add_argument('--controller', choices=('dwa', 'rl', 'rl_handover'), required=True)
    ap.add_argument('--arm', default='')
    ap.add_argument('--policy-id', default='')
    ap.add_argument('--train-seed', type=int, default=None)
    ap.add_argument('--handover-radius', type=float, default=None)
    ap.add_argument('--rtf-target', type=float, default=1.0)
    ap.add_argument('--commit', default='unknown')
    ap.add_argument('--run-tag', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--world-sdf', required=True, help='정적 충돌 기하 (원본 warehouse.sdf)')
    ap.add_argument('--models-dir', required=True)
    ap.add_argument('--world-name', default='warehouse')
    ap.add_argument('--robot', default='amr_01')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--probe', default='', help='쉼표 목록: scan,step')
    ap.add_argument('--probe-scan-pose', default='0.0,0.0,0.0;-6.0,-7.0,2.0',
                    help='x,y,yaw[;x,y,yaw…] — 스캔 탐침 자세들 (첫째 = 랙 B-C 통로 중앙, 둘째 = 좌우가 다른 '
                         '자리 — 대칭 통로에서는 빔 순서 뒤집기 대조가 약하다, A5 흔들기 2)')
    ap.add_argument('--probe-step-pose', default='-15.9,-9.2,0.0',
                    help='x,y,yaw — 계단 응답 탐침 자세 (정적 상자와 10.8 m 떨어진 홀, 앞 5 m 넘게 빔)')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Bench(args)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    os.makedirs(os.path.join(args.out, 'episodes'), exist_ok=True)
    log = open(os.path.join(args.out, 'progress.log'), 'a', encoding='utf-8')

    def say(msg):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        print(line, flush=True)
        log.write(line + '\n')
        log.flush()
    d = S.load(args.scenarios)
    todo = S.select(d, args.select, args.start, args.count)
    say(f'시나리오 {len(todo)} 개 ({args.select}), 제어기 {args.controller}, 출력 {args.out}')
    if not node.wait_until(lambda: node.now() > 0 and node.gt_last is not None, 60.0, 120.0):
        say('sim 시계·지면 진실을 받지 못했다 — 끝')
    if not node.nav.wait_for_server(timeout_sec=120.0):
        say('navigate_to_pose 서버가 없다 — 끝')
        return
    if 'scan' in args.probe:
        for k, txt in enumerate(args.probe_scan_pose.split(';')):
            pose = [float(v) for v in txt.split(',')]
            name = 'probe_scan.json' if k == 0 else f'probe_scan_{k}.json'
            say(f'탐침 scan @ {pose}: {node.probe_scan(args.out, pose, name=name)}')
    if 'step' in args.probe:
        pose = [float(v) for v in args.probe_step_pose.split(',')]
        say(f'탐침 step @ {pose}: {node.probe_step(args.out, pose)}')
    seen = {}
    for k, sc in enumerate(todo):
        idx = args.start + k
        row, raw = node.run_episode(sc, idx)
        ep_index = seen.get(sc['scenario_seed'], 0)
        seen[sc['scenario_seed']] = ep_index + 1
        R.append_csv(os.path.join(args.out, 'gzbench.csv'), R.COLUMNS, row)
        R.append_csv(os.path.join(args.out, 'episodes_v1.csv'),
                     list(R.episode_v1_row(row).keys()),
                     R.episode_v1_row(row, ep_index, args.train_seed))
        with gzip.open(os.path.join(args.out, 'episodes', f'{idx:04d}_{sc["id"]}.json.gz'),
                       'wt', encoding='utf-8') as f:
            json.dump(raw, f, default=lambda o: None)
        say(f'[{idx}] {sc["id"]} ({sc["set"]}, L {sc["path_length_m"]:.1f}) → '
            f'{row.get("outcome_contract")} / {row.get("outcome_system")} '
            f'gt {row.get("final_dist_gt_m")} est {row.get("final_dist_est_m")} '
            f'nav {row.get("nav_time_s")} s rtf {row.get("rtf")} wall {row.get("wall_episode_s")}'
            f' setup_err {row.get("setup_error")}')
    say('끝')
    log.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
