"""
E2 단일 로봇 Gazebo 측위 시행 러너 (컨테이너 안, 스택이 이미 떠 있는 상태에서 돈다).

스택: amr_bringup system.launch.py (multi_robot 과 같은 구성 요소·파라미터, 로봇 amr_01, 프레임 접두어
amr_01/) — localization(amcl + 이중 EKF + kidnap_monitor + scan_matcher) + navigation(spin)
+ perception (safety_node = cmd_vel 유일 발행자). behavior·fleet 은 끈다 (e2_container.sh 머리말).

시행 하나 (trials.json 의 한 행):
  1. 더미 주차 → 로봇 순간 이동(set_pose) → 곧바로 참 자세 시드(initialpose ×3 + map EKF set_pose +
     AMCL 무이동 갱신, amr_itest.actions.seed_pose 와 같은 묶음) → 이웃 더미 k 대 배치.
  2. 안정: kidnap TRACKING · lost false · 추정 오차 < 0.15 m · |헤딩| < 5° 가 2 s 연속 (상한 40 s sim).
  3. 자연 단계 20 s (sim): LOST 여부와 inlier(kidnap_status.match_ratio) 기록.
  4. 강제 단계 (자연 LOST 가 없을 때): scan_match/accepted 에 false 를 100 Hz 로 넣어 운용 감지기의
     "scan registration rejected > 0.8" 경보를 일으킨다 → SUSPECT 1 s → _declare_lost → 실제 실패와
     같은 전역 재초기화 경로(가설 탐색 → 시드 #1 → 회전 → 여백 게이트 → 재초기화 #2…, 120 s 뒤 FAILED).
     P5 실측(launch.log)에서 이 경보 경로로도 LOST 가 났다. LOST 를 보면 바로 주입을 멈춘다.
  5. 판정: TRACKING 선언 시각(없으면 LOST + 120 s)의 odometry/filtered_map 과 ground_truth/odom 오차
     (e2_outcome.classify). 회복 시간, 재초기화 수, 시드·가설, 여백 값을 함께 남긴다.
시행이 끝날 때마다 trials/<id>_a<n>/ 와 trials.jsonl 에 바로 쓴다 (중간에 끊겨도 잃는 것은 진행 중 시행 하나).
이미 done 인 시행은 건너뛴다 (재개). 이상(시계 정지, 스택 미기동, 배치 실패, 안정 실패)은 그 시도를 invalid 로
남기고 종료 코드 10 으로 끝낸다 — 호스트 드라이버(e2_campaign.py)가 세션을 새로 띄운다.
"""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Dict, List, Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
from layopt.loc import e2_outcome as eo
from layopt.loc import gzcli
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

EXIT_DONE = 0          # 할 일 없음 / 세션 시행 수 채움
EXIT_ANOMALY = 10      # 세션을 새로 띄워야 한다
NS = 'amr_01'
N_DUMMIES = 4
PARK = [(60.0 + 2.0 * i, 60.0, 0.0) for i in range(N_DUMMIES)]   # LiDAR 25 m 밖
LOG_NODES = ('kidnap_monitor_node', 'amcl', 'behavior_server', 'scan_matcher_node')

P = {
    'settle_tol_m': 0.15, 'settle_tol_yaw': math.radians(5.0), 'settle_hold_s': 2.0,
    'settle_max_s': 40.0, 'natural_s': 20.0, 'inject_hz': 100.0, 'inject_max_s': 15.0,
    'window_s': eo.RECOVERY_WINDOW_S, 'post_s': 3.0, 'track_dt': 0.1,
    # 시계 감시: 느린 RTF 는 이상이 아니다(공유 부하 101 에서 RTF 0.17 실측 — 첫 캠페인 세션 3 개가
    # RTF 0.25 상한(wall_factor 4)에 걸려 멈췄다). 진짜 정지(stall_window_s 동안 sim 이 stall_min_sim_s 도
    # 안 감)만 이상으로 보고, 전체 상한은 wall_factor 배로 넉넉히 둔다.
    'wall_factor': 15.0, 'wall_min_s': 120.0, 'stall_window_s': 60.0, 'stall_min_sim_s': 0.5,
    # 부하 문턱: 시행 시작 전 1분 loadavg 가 이 아래로 내려오기를 기다린다 (상한 load_wait_max_s 뒤 그대로 진행)
    'load_max_1m': 64.0, 'load_wait_max_s': 1200.0, 'load_poll_s': 15.0,
}


def yaw_of(q) -> float:
    """쿼터니언 → yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp(s) -> float:
    """builtin_interfaces/Time → 초."""
    return s.sec + s.nanosec * 1e-9


def utc() -> str:
    """지금 UTC ISO."""
    return dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


class Anomaly(RuntimeError):
    """세션을 다시 띄워야 하는 이상 (시계 정지, 스택 미기동, 배치 실패 …)."""


class Recorder:
    """시행 하나의 원자료 (콜백 스레드가 넣고 주 스레드가 읽는다)."""

    def __init__(self) -> None:
        """빈 기록."""
        self.lock = threading.Lock()
        self.gt, self.est, self.amcl = eo.Track(), eo.Track(), eo.Track()
        self.rows: List[str] = []           # track.csv 행
        self._last = {'gt': -1e9, 'est': -1e9}
        self.status: List[Dict] = []
        self.logs: List[Dict] = []
        self.events: List = []              # (t, e2_outcome 사건)
        self.lost: List = []                # (t, bool)
        self.amcl_cov = eo.Track()          # (t, cov_xy, var_yaw, 0) — 판정 시각 AMCL 퍼짐
        self.scans: Dict[str, Dict] = {}

    def pose(self, src: str, t: float, x: float, y: float, yaw: float, extra: str = '') -> None:
        """궤적 표본 (gt·est 는 track_dt 간격으로 솎아 csv 에, Track 에는 전부)."""
        with self.lock:
            getattr(self, src).add(t, x, y, yaw)
            if src == 'amcl' or t - self._last.get(src, -1e9) >= P['track_dt']:
                self._last[src] = t
                self.rows.append(f'{t:.3f},{src},{x:.4f},{y:.4f},{yaw:.5f},{extra}')


class E2Node(Node):
    """E2 러너의 ROS 쪽: 구독(기록), 발행(시드·주입), 서비스(EKF set_pose·무이동 갱신)."""

    def __init__(self) -> None:
        """구독·발행·클라이언트."""
        super().__init__('layopt_e2_runner',
                         parameter_overrides=[rclpy.parameter.Parameter(
                             'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.rec: Optional[Recorder] = None
        self.last = {'gt': None, 'est': None, 'amcl': None, 'status': None, 'lost': None,
                     'scan': None}
        self.counts = {k: 0 for k in self.last}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        rel = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, f'/{NS}/ground_truth/odom', self.on_gt, 50)
        self.create_subscription(Odometry, f'/{NS}/odometry/filtered_map', self.on_est, 50)
        self.create_subscription(PoseWithCovarianceStamped, f'/{NS}/amcl_pose', self.on_amcl,
                                 latched)
        self.create_subscription(String, f'/{NS}/localization/kidnap_status', self.on_status, 50)
        self.create_subscription(Bool, f'/{NS}/localization/lost', self.on_lost, latched)
        self.create_subscription(LaserScan, f'/{NS}/scan_filtered', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.on_log, rel)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, f'/{NS}/initialpose', 10)
        self.reg_pub = self.create_publisher(Bool, f'/{NS}/scan_match/accepted', 50)
        from robot_localization.srv import SetPose
        from std_srvs.srv import Empty
        from lifecycle_msgs.srv import GetState
        self.SetPose, self.Empty, self.GetState = SetPose, Empty, GetState
        self.ekf_cli = self.create_client(SetPose, f'/{NS}/ekf_filter_node_map/set_pose')
        self.nomo_cli = self.create_client(Empty, f'/{NS}/request_nomotion_update')
        self.state_cli = {n: self.create_client(GetState, f'/{NS}/{n}/get_state')
                          for n in ('amcl', 'behavior_server')}
        self.injecting = threading.Event()
        threading.Thread(target=self._inject_loop, daemon=True).start()

    # ---------------------------------------------------------------- 시각
    def now(self) -> float:
        """sim 시각 [s]."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------------------------------------------------------- 콜백
    def on_gt(self, m: Odometry) -> None:
        """지면 진실 (월드 = map)."""
        p = m.pose.pose
        t = stamp(m.header.stamp)
        v = (t, (p.position.x, p.position.y, yaw_of(p.orientation)),
             abs(m.twist.twist.angular.z) + abs(m.twist.twist.linear.x))
        self.last['gt'] = v
        self.counts['gt'] += 1
        if self.rec:
            self.rec.pose('gt', t, *v[1])

    def on_est(self, m: Odometry) -> None:
        """운용 추정 (odometry/filtered_map)."""
        p = m.pose.pose
        t = stamp(m.header.stamp)
        self.last['est'] = (t, (p.position.x, p.position.y, yaw_of(p.orientation)))
        self.counts['est'] += 1
        if self.rec:
            self.rec.pose('est', t, *self.last['est'][1])

    def on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        """AMCL 자세 (공분산 trace 함께)."""
        p = m.pose.pose
        c = m.pose.covariance
        t = stamp(m.header.stamp)
        self.last['amcl'] = (t, (p.position.x, p.position.y, yaw_of(p.orientation)),
                             c[0] + c[7])
        self.counts['amcl'] += 1
        if self.rec:
            self.rec.pose('amcl', t, *self.last['amcl'][1], extra=f'{c[0] + c[7]:.5f}')
            with self.rec.lock:
                self.rec.amcl_cov.add(t, c[0] + c[7], c[35], 0.0)

    def on_status(self, m: String) -> None:
        """kidnap_status JSON (수신 sim 시각을 붙인다)."""
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        d['t'] = round(self.now(), 3)
        self.last['status'] = d
        self.counts['status'] += 1
        if self.rec:
            with self.rec.lock:
                self.rec.status.append(d)

    def on_lost(self, m: Bool) -> None:
        """localization/lost (래치)."""
        self.last['lost'] = (self.now(), bool(m.data))
        self.counts['lost'] += 1
        if self.rec:
            with self.rec.lock:
                self.rec.lost.append((round(self.now(), 3), bool(m.data)))

    def on_scan(self, m: LaserScan) -> None:
        """마지막 스캔 (자연 단계 시작·LOST 시각 스냅샷용)."""
        self.last['scan'] = m
        self.counts['scan'] += 1

    def on_log(self, m: Log) -> None:
        """/rosout 중 측위 관련 노드만 (kidnap_monitor 사건은 해석해 둔다)."""
        if not m.name.startswith(f'{NS}.') or not m.name.endswith(LOG_NODES):
            return
        if not self.rec:
            return
        # /rosout 의 stamp 는 rcutils 로깅 시각 = **벽시계**다 (use_sim_time 과 무관, 실측). 사건 시각은
        # 수신 sim 시각으로 쓰고 벽시계 stamp 는 따로 남긴다 (수신 지연은 ms 규모).
        t = self.now()
        with self.rec.lock:
            self.rec.logs.append({'t': round(t, 3), 'wall': round(stamp(m.stamp), 3),
                                  'node': m.name, 'level': int(m.level), 'msg': m.msg})
            if m.name.endswith('kidnap_monitor_node'):
                ev = eo.parse_monitor_line(m.msg)
                if ev:
                    self.rec.events.append((t, ev))

    # ---------------------------------------------------------------- 동작
    def _inject_loop(self) -> None:
        msg = Bool(data=False)
        period = 1.0 / P['inject_hz']
        while rclpy.ok():
            if self.injecting.wait(0.2):
                self.reg_pub.publish(msg)
                time.sleep(period)

    def seed(self, x: float, y: float, yaw: float) -> Dict:
        """참 자세를 AMCL(initialpose ×3)·map EKF(set_pose)·AMCL 무이동 갱신으로 준다."""
        def pose_msg():
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05 ** 2
            msg.pose.covariance[35] = math.radians(3.0) ** 2
            return msg
        for _ in range(3):
            self.init_pub.publish(pose_msg())
            time.sleep(0.2)
        ekf = False
        if self.ekf_cli.wait_for_service(timeout_sec=2.0):
            req = self.SetPose.Request()
            req.pose = pose_msg()
            ekf = self._call(self.ekf_cli, req, 5.0) is not None
        nomo = 0
        for _ in range(2):
            if self.nomo_cli.wait_for_service(timeout_sec=2.0) and self._call(
                    self.nomo_cli, self.Empty.Request(), 2.0) is not None:
                nomo += 1
        return {'initialpose': 3, 'ekf_set_pose': ekf, 'nomotion': nomo}

    def _call(self, cli, req, timeout: float):
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def lifecycle_active(self, name: str) -> bool:
        """/<ns>/<name>/get_state == active(3)."""
        cli = self.state_cli[name]
        if not cli.service_is_ready():
            return False
        res = self._call(cli, self.GetState.Request(), 3.0)
        return res is not None and int(res.current_state.id) == 3


class Runner:
    """시행 목록을 돌린다 (세션 하나)."""

    def __init__(self, node: E2Node, args) -> None:
        """시행 목록·실행 디렉터리·월드 이름."""
        self.node = node
        self.args = args
        self.run_dir = Path(args.run_dir)
        self.spec = json.loads(Path(args.trials).read_text(encoding='utf-8'))
        self.world = args.world_name
        self.session = args.session
        self.results = self.run_dir / 'trials.jsonl'
        self.attempts_log = self.run_dir / 'attempts.jsonl'
        self.dummy_mode = args.dummy_mode
        self.dummies_ready = False

    # ---------------------------------------------------------------- 상태
    def history(self) -> Dict[str, List[Dict]]:
        """trial_id → 기록된 시도들 (attempts.jsonl)."""
        out: Dict[str, List[Dict]] = {}
        if self.attempts_log.exists():
            for line in self.attempts_log.read_text(encoding='utf-8').splitlines():
                if line.strip():
                    r = json.loads(line)
                    out.setdefault(r['trial_id'], []).append(r)
        return out

    def pending(self) -> List[Dict]:
        """아직 done 이 아니고 시도 상한(max_attempts) 안인 시행 (order 순)."""
        hist = self.history()
        out = []
        for tr in sorted(self.spec['trials'], key=lambda r: r['order']):
            h = hist.get(tr['trial_id'], [])
            if any(r['status'] == 'done' for r in h):
                continue
            if sum(1 for r in h if r['status'] == 'invalid') >= self.args.max_attempts:
                continue
            out.append(tr)
        if self.args.only:
            want = set(self.args.only.split(','))
            out = [t for t in out if t['trial_id'] in want]
        return out

    # ---------------------------------------------------------------- 대기
    def wait_sim(self, cond, sim_s: float, what: str, poll: float = 0.05):
        """sim 시각으로 sim_s 동안 cond() 를 기다린다. 시계가 멈추면(벽시계 상한) Anomaly."""
        n = self.node
        t0, w0 = n.now(), time.time()
        cap = max(P['wall_min_s'], sim_s * P['wall_factor'])
        mark_w, mark_s = w0, t0          # 정지 감시 창의 시작 (벽시계, sim)
        while True:
            r = cond()
            if r:
                return r
            now_s, now_w = n.now(), time.time()
            if now_s - t0 >= sim_s:
                return None
            if now_w - mark_w >= P['stall_window_s']:
                if now_s - mark_s < P['stall_min_sim_s']:
                    raise Anomaly(f'clock_stalled during {what}: sim +{now_s - mark_s:.2f} s in '
                                  f'{now_w - mark_w:.0f} s wall')
                mark_w, mark_s = now_w, now_s
            if now_w - w0 > cap:
                raise Anomaly(f'too_slow during {what}: sim {now_s - t0:.1f}/{sim_s} s '
                              f'in {now_w - w0:.0f} s wall')
            time.sleep(poll)

    def wait_load(self) -> float:
        """1분 loadavg < load_max_1m 까지 (상한 load_wait_max_s). 반환 = 기다린 벽시계 초."""
        w0 = time.time()
        while (os.getloadavg()[0] >= P['load_max_1m']
               and time.time() - w0 < P['load_wait_max_s']):
            time.sleep(P['load_poll_s'])
        return round(time.time() - w0, 1)

    def wait_ready(self) -> Dict:
        """스택 준비: 시계·gt·est·kidnap_status·amcl/behavior_server active."""
        n = self.node
        w0 = time.time()
        stage = ''
        while time.time() - w0 < self.args.ready_timeout:
            if n.counts['gt'] < 10 or n.counts['est'] < 10:
                stage = 'topics gt/est'
            elif n.counts['status'] < 2:
                stage = 'kidnap_status'
            elif not n.lifecycle_active('amcl'):
                stage = 'amcl active'
            elif not n.lifecycle_active('behavior_server'):
                stage = 'behavior_server active'
            else:
                return {'ready_wall_s': round(time.time() - w0, 1)}
            time.sleep(1.0)
        raise Anomaly(f'bringup_timeout at {stage}')

    # ---------------------------------------------------------------- 더미
    def setup_dummies(self) -> None:
        """세션 시작: 더미 N_DUMMIES 개를 주차 자리에 만든다."""
        for i, (x, y, yaw) in enumerate(PARK):
            ok, out = gzcli.create_dummy(self.world, f'layopt_dummy_{i}', x, y, yaw)
            if not ok:
                raise Anomaly(f'create dummy {i} failed: {out[:200]}')
        self.dummies_ready = True

    def place_dummies(self, poses) -> None:
        """더미 i 를 poses[i] 로 (없으면 주차)."""
        for i in range(N_DUMMIES):
            name = f'layopt_dummy_{i}'
            x, y, yaw = poses[i] if i < len(poses) else PARK[i]
            if self.dummy_mode == 'move':
                ok, out = gzcli.set_pose(self.world, name, x, y, yaw, z=gzcli.DUMMY_Z)
            else:
                gzcli.remove(self.world, name)
                ok, out = gzcli.create_dummy(self.world, name, x, y, yaw)
            if not ok:
                raise Anomaly(f'place dummy {i} failed: {out[:200]}')

    # ---------------------------------------------------------------- 시행
    def gt_err(self):
        """최근 (est − gt) 위치·헤딩 오차."""
        g, e = self.node.last['gt'], self.node.last['est']
        if g is None or e is None:
            return math.inf, math.inf
        d, a = eo.pose_error(e[1], g[1])
        return d, a

    def stopped(self) -> bool:
        """지면 진실 속도 ≈ 0."""
        g = self.node.last['gt']
        return g is not None and g[2] < 0.01

    def run_trial(self, tr: Dict, attempt: int) -> Dict:
        """시행 하나. 반환 = trials.jsonl 한 줄 (status done | invalid)."""
        n = self.node
        load_wait = self.wait_load()
        w_start, s_start = time.time(), n.now()
        load0 = os.getloadavg()
        res: Dict = {'trial_id': tr['trial_id'], 'attempt': attempt, 'session': self.session,
                     'pose_id': tr['pose_id'], 'kind': tr['kind'], 'k': tr['k'],
                     'x': tr['x'], 'y': tr['y'], 'yaw': tr['yaw'], 'started': utc(),
                     'load_start': [round(v, 2) for v in load0], 'load_wait_s': load_wait}
        rec = Recorder()
        try:
            # 0. 앞 시행의 회전이 멈출 때까지
            self.wait_sim(self.stopped, 5.0, 'stop before teleport')
            n.injecting.clear()
            # 1. 더미 주차 → 순간 이동 → 즉시 시드 → 이웃 더미
            self.place_dummies([])
            ok, out = gzcli.set_pose(self.world, NS, tr['x'], tr['y'], tr['yaw'])
            if not ok:
                raise Anomaly(f'teleport failed: {out[:200]}')
            n.rec = rec
            t_place = n.now()
            res['seed'] = n.seed(tr['x'], tr['y'], tr['yaw'])
            self.place_dummies(tr['neighbors'][:tr['k']])
            res['place_s_sim'] = round(n.now() - t_place, 2)
            # 2. 안정
            hold = {'since': None}

            def settled():
                s, lost = n.last['status'], n.last['lost']
                d, a = self.gt_err()
                ok = (s is not None and s.get('state') == 'TRACKING' and not s.get('lost')
                      and (lost is None or not lost[1]) and d < P['settle_tol_m']
                      and a < P['settle_tol_yaw'])
                if not ok:
                    hold['since'] = None
                    return False
                hold['since'] = hold['since'] if hold['since'] is not None else n.now()
                return n.now() - hold['since'] >= P['settle_hold_s']
            if not self.wait_sim(settled, P['settle_max_s'], 'settle'):
                d, a = self.gt_err()
                raise Anomaly(f'settle_timeout: err {d:.2f} m {math.degrees(a):.1f} deg, '
                              f'status {n.last["status"]}')
            t_nat = n.now()
            res['settle_s_sim'] = round(t_nat - t_place, 2)
            rec.scans['natural_start'] = self.scan_snapshot()
            # 3. 자연 단계
            lost_flag = {'t': None}

            def lost_now():
                lo = n.last['lost']
                if lo is not None and lo[1] and lo[0] >= t_nat:
                    lost_flag['t'] = lo[0]
                    return True
                s = n.last['status']
                if (s is not None and s.get('state') in ('RECOVERING', 'FAILED')
                        and s['t'] >= t_nat):
                    lost_flag['t'] = s['t']
                    return True
                return False
            nat = self.wait_sim(lost_now, P['natural_s'], 'natural phase')
            with rec.lock:
                ratios = [s.get('match_ratio') for s in rec.status
                          if s['t'] >= t_nat and s.get('match_ratio') is not None]
            res['natural'] = {'lost': bool(nat), 't_lost': None if not nat else round(
                lost_flag['t'] - t_nat, 2), 'inlier_min': min(ratios) if ratios else None,
                'inlier_mean': round(sum(ratios) / len(ratios), 4) if ratios else None,
                'inlier_n': len(ratios)}
            # 4. 강제 단계
            if nat:
                res['trigger'] = 'natural'
                t_lost = lost_flag['t']
            else:
                res['trigger'] = 'forced_reg_reject'
                t_inj = n.now()
                n.injecting.set()
                got = self.wait_sim(lost_now, P['inject_max_s'], 'inject')
                n.injecting.clear()
                if not got:
                    raise Anomaly('forced LOST not declared within inject_max_s')
                t_lost = lost_flag['t']
                res['inject_s_sim'] = round(t_lost - t_inj, 2)
            # t_lost = localization/lost true 수신 sim 시각. 로그의 "[RECOVERING] LOST:" 줄은 쓰지 않는다 —
            # kidnap_monitor 는 사건 로그를 동작 실행(가설 탐색 0.2–0.3 s 포함) 뒤에 몰아 찍는다.
            res['t_lost_sim'] = round(t_lost, 3)
            rec.scans['lost'] = self.scan_snapshot()
            # 5. 복구 창
            decl = {'t': None}

            def declared():
                # TRACKING 선언 = localization/lost false 수신 또는 "[TRACKING] recovered" 로그 (먼저 온 것)
                with rec.lock:
                    rv = [t for t, e in rec.events if e['ev'] == 'recovered' and t > t_lost]
                    rv += [t for t, v in rec.lost if not v and t > t_lost]
                if rv:
                    decl['t'] = min(rv)
                    return True
                return False
            remain = t_lost + P['window_s'] - n.now()
            self.wait_sim(declared, max(remain, 0.0), 'recovery window')
            t_out = decl['t'] if decl['t'] is not None else t_lost + P['window_s']
            # 판정 시각 뒤 post_s 더 기록 (EKF set_pose 반영 확인, FAILED 기록)
            self.wait_sim(lambda: False, max(t_out + P['post_s'] - n.now(), 0.0), 'post')
            res.update(self.judge(rec, t_lost, t_out, decl['t']))
            res['status'] = 'done'
        except Anomaly as exc:
            n.injecting.clear()
            res['status'] = 'invalid'
            res['reason'] = str(exc)
        finally:
            n.rec = None
        res['ended'] = utc()
        res['wall_s'] = round(time.time() - w_start, 1)
        res['sim_s'] = round(n.now() - s_start, 1)
        res['rtf'] = round(res['sim_s'] / max(res['wall_s'], 1e-6), 3)
        res['load_end'] = [round(v, 2) for v in os.getloadavg()]
        self.write_trial(tr, attempt, res, rec)
        return res

    def scan_snapshot(self) -> Optional[Dict]:
        """마지막 scan_filtered (E0 대조용)."""
        m = self.node.last['scan']
        if m is None:
            return None
        return {'t': round(stamp(m.header.stamp), 3), 'angle_min': m.angle_min,
                'angle_increment': m.angle_increment, 'range_max': m.range_max,
                'ranges': [None if not math.isfinite(r) else round(float(r), 4)
                           for r in m.ranges]}

    def judge(self, rec: Recorder, t_lost: float, t_out: float, t_decl: Optional[float]) -> Dict:
        """판정 (prereg §3): t_out 의 filtered_map 대 gt 오차. 민감도: +1 s, AMCL."""
        with rec.lock:
            est = rec.est.at_or_after(t_out)
            est1 = rec.est.at_or_after(t_out + 1.0)
            amcl = rec.amcl.last_before(t_out + 0.05)   # 정지 중 AMCL 은 발행하지 않는다
            acov = rec.amcl_cov.last_before(t_out + 0.05)
            gt = rec.gt.nearest(t_out)
            gt1 = rec.gt.nearest(t_out + 1.0)
            events = list(rec.events)
            status = list(rec.status)
        err, yerr = eo.pose_error(est[1] if est else None, gt[1] if gt else None)
        err1, _ = eo.pose_error(est1[1] if est1 else None, gt1[1] if gt1 else None)
        erra, _ = eo.pose_error(amcl[1] if amcl else None, gt[1] if gt else None)
        summ = eo.summarize_recovery(events, t_lost, t_out)
        margins = [s['alias_margin'] for s in status
                   if t_lost <= s['t'] <= t_out and s.get('alias_margin') is not None]
        final_state = next((s.get('state') for s in reversed(status)), None)
        gtp = gt[1] if gt else None
        seeds_true = [eo.seed_is_true(s, gtp) for s in summ['seeds']] if gtp else []
        return {
            'declared': t_decl is not None,
            'recovery_s': None if t_decl is None else round(t_decl - t_lost, 2),
            't_out_rel': round(t_out - t_lost, 2),
            'err_m': None if err is None else round(err, 3),
            'err_yaw_deg': None if yerr is None else round(math.degrees(yerr), 1),
            'err_m_plus1s': None if err1 is None else round(err1, 3),
            'err_m_amcl': None if erra is None else round(erra, 3),
            'outcome': eo.classify(err),
            'outcome_plus1s': eo.classify(err1),
            'outcome_amcl': eo.classify(erra),
            'est_at': None if not est else [round(v, 3) for v in est[1]],
            'gt_at': None if not gt else [round(v, 3) for v in gt[1]],
            'n_reinit': summ['n_reinit'], 'n_seed': summ['n_seed'],
            'amcl_global': summ['amcl_global'], 'reseed_top1': summ['reseed_top1'],
            'seeds': summ['seeds'], 'seeds_true': seeds_true,
            'first_search': summ['searches'][0] if summ['searches'] else None,
            'gate_margins_n': len(summ['gate_margins']),
            'gate_margin_max': max(summ['gate_margins']) if summ['gate_margins'] else None,
            'gate_bypassed': summ['gate_bypassed'],
            'status_margins_n': len(margins),
            'status_margin_max': max(margins) if margins else None,
            'status_margin_median': (sorted(margins)[len(margins) // 2] if margins else None),
            'failed_reason': summ['failed'], 'lost_reason': summ['lost_reason'],
            'final_state': final_state,
            'est_source': 'odometry/filtered_map',
            # 판정 시각의 AMCL 퍼짐: 크면 "틀린 곳에 확신" 이 아니라 "아직 모름" (판정은 prereg 대로 오차만)
            'amcl_cov_xy_at': None if not acov else round(acov[1][0], 4),
            'amcl_var_yaw_at': None if not acov else round(acov[1][1], 4),
            'amcl_age_at': None if not acov else round(t_out - acov[0], 2),
        }

    def write_trial(self, tr: Dict, attempt: int, res: Dict, rec: Recorder) -> None:
        """trials/<id>_a<n>/ 원자료 + attempts.jsonl (+ done 이면 trials.jsonl) — 바로 fsync."""
        d = self.run_dir / 'trials' / f'{tr["trial_id"]}_a{attempt}'
        d.mkdir(parents=True, exist_ok=True)
        with rec.lock:
            (d / 'track.csv').write_text('t,src,x,y,yaw,extra\n' + '\n'.join(rec.rows) + '\n',
                                         encoding='utf-8')
            (d / 'status.jsonl').write_text(''.join(json.dumps(s) + '\n' for s in rec.status),
                                            encoding='utf-8')
            (d / 'rosout.jsonl').write_text(
                ''.join(json.dumps(x, ensure_ascii=False) + '\n' for x in rec.logs),
                encoding='utf-8')
            (d / 'lost.json').write_text(json.dumps(rec.lost), encoding='utf-8')
            (d / 'scans.json').write_text(json.dumps(rec.scans), encoding='utf-8')
        (d / 'trial.json').write_text(json.dumps({'trial': tr, 'result': res}, ensure_ascii=False,
                                                 indent=1), encoding='utf-8')
        line = json.dumps(res, ensure_ascii=False) + '\n'
        for path in ([self.attempts_log] + ([self.results] if res['status'] == 'done' else [])):
            with path.open('a', encoding='utf-8') as f:
                f.write(line)
                f.flush()
                os.fsync(f.fileno())

    # ---------------------------------------------------------------- 세션
    def run(self) -> int:
        """세션: 준비 → 더미 → 시행들 (상한 per_session). 이상이면 EXIT_ANOMALY."""
        todo = self.pending()
        sess = {'session': self.session, 'started': utc(), 'pending': len(todo),
                'load_start': [round(v, 2) for v in os.getloadavg()]}
        if not todo:
            sess['note'] = 'nothing pending'
            self.log_session(sess)
            return EXIT_DONE
        code = EXIT_DONE
        try:
            sess.update(self.wait_ready())
            self.setup_dummies()
            hist = self.history()
            for i, tr in enumerate(todo[:self.args.per_session]):
                attempt = len(hist.get(tr['trial_id'], [])) + 1
                w0, s0 = time.time(), self.node.now()
                r = self.run_trial(tr, attempt)
                rtf = (self.node.now() - s0) / max(time.time() - w0, 1e-6)
                print(f'[e2] {tr["trial_id"]} a{attempt} {r["status"]} '
                      f'{r.get("outcome", r.get("reason", ""))} err={r.get("err_m")} '
                      f'nat_lost={r.get("natural", {}).get("lost")} '
                      f'rec={r.get("recovery_s")} wall={r["wall_s"]} rtf={rtf:.2f}', flush=True)
                if r['status'] == 'invalid':
                    code = EXIT_ANOMALY
                    break
        except Anomaly as exc:
            sess['anomaly'] = str(exc)
            code = EXIT_ANOMALY
        sess['ended'] = utc()
        sess['exit'] = code
        sess['load_end'] = [round(v, 2) for v in os.getloadavg()]
        self.log_session(sess)
        return code

    def log_session(self, sess: Dict) -> None:
        """sessions.jsonl 에 한 줄."""
        with (self.run_dir / 'sessions.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps(sess, ensure_ascii=False) + '\n')


def main(argv=None) -> int:
    """컨테이너 진입 (e2_container.sh 가 부른다)."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', required=True)
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--world-name', required=True)
    ap.add_argument('--session', required=True)
    ap.add_argument('--per-session', type=int, default=20)
    ap.add_argument('--max-attempts', type=int, default=2)
    ap.add_argument('--ready-timeout', type=float, default=420.0)
    ap.add_argument('--dummy-mode', choices=('move', 'recreate'), default='move')
    ap.add_argument('--only', default='', help='쉼표 구분 trial_id (시험용)')
    a = ap.parse_args(argv)
    rclpy.init()
    node = E2Node()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    code = EXIT_ANOMALY
    try:
        code = Runner(node, a).run()
    finally:
        node.injecting.clear()
        sys.stdout.flush()
        sys.stderr.flush()
        # 결과는 시행마다 이미 fsync 했다. rclpy 정리 중 C++ 스레드 소멸자가 abort(134)를 낸 적이 있어
        # (dbg3: "terminate called without an active exception") 정리 없이 종료 코드를 그대로 낸다.
        os._exit(code)
    return code


if __name__ == '__main__':
    sys.exit(main())
