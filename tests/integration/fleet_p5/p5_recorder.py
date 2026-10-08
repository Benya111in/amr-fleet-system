#!/usr/bin/env python3
"""
보너스 4 P5 기록기 · 작업 소스 (Gazebo 5 대, 컨테이너 안) — docs/research/fleet-distributed/p5.md.

하네스(launch_testing) 대신 따로 도는 이유: 시나리오 12 하네스는 시험 스레드의 서비스 호출이 rclpy 클라이언트 안에서
멈추는 일이 잦았다 (b4-s12 2026-10-08: 12 갈래 중 7 갈래 정지, py-spy 스택 rclpy/client.py call_async). 이 노드는
단일 스레드 실행기 하나에서 타이머 · 콜백 · 비동기 서비스 호출을 모두 처리하고(다른 스레드가 클라이언트를 만지지 않는다),
UDP 전용 DDS 프로파일로 띄운다 (스택의 SHM 포트 고장이 기록기를 멈추지 않게 — 컨테이너 진입 스크립트가 정한다).

하는 일
- 준비 판정: 5 대 실행기 IDLE · 위치추정 lost 아님 · 추정 자세와 정답 자세 차 ≤ 0.30 m (3 s 유지) · Nav2/위치추정
  lifecycle_manager 활성 (is_active, 비동기) · 실행기 traffic/hold 첫 값 수신 (launch.log) · 분산 모드면 에이전트 5 대
  NORMAL + 서버 ALIVE, 중앙 모드면 /fleet/status 수신.
- 작업 게시: 계획 파일의 작업을 t_base + offset (sim) 에 /fleet/task_board 로 (RELIABLE · TRANSIENT_LOCAL · 256 —
  관리자와 에이전트가 같은 게시판을 본다). t_base = max(계획 t_start, 준비 시각).
- 기록 (모두 sim 시각 t, 벽시계 w): 정답 자세 gt.csv (로봇마다 ≥ 0.1 s 간격 = 10 Hz), 추정 자세 est.csv (0.2 s),
  events.jsonl (실행기 단계 · task_status · 관리자 task_events · 에이전트 상태 · 에이전트 사건 · 하트비트 · traffic/hold ·
  교통 사건 · plan 요약 · lost · 게시), p2p.csv (토픽별 초당 메시지 수 · 직렬화 바이트), clock.json (호스트 장애 주입기가
  읽는 현재 sim 시각 · t_base, 원자적 교체), meta.json, summary.json.
- 끝: t_base + horizon_s, 또는 (end_when_done) 게시한 작업이 모두 실행기 COMPLETED 이고 5 대가 5 s 동안 IDLE 이며
  sim ≥ t_base + min_end_s.

종료 코드: 0 정상 끝 · 3 준비 시간 초과 · 4 /clock 정지 (벽시계 clock_stall_s 동안 sim 이 안 감) · 2 인자 오류.
"""

import argparse
import json
import math
import os
import re
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from amr_msgs.msg import (AgentBeacon, AgentState, FleetHeartbeat, FleetStatus, FleetVoteBatch,
                          NegState, Task)
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
try:
    from robot_localization.srv import SetPose
except ImportError:          # pragma: no cover - robot_localization 없는 환경
    SetPose = None

ROBOTS = ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BOARD = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE100 = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
BEST = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
RE_HOLD_FIRST = re.compile(r'\[(amr_\d+)\.task_executor_node\]: traffic/hold: 첫 값 수신')
RE_HOLD_UNMANAGED = re.compile(r'\[(amr_\d+)\.task_executor_node\]: traffic/hold: 발행자 없음')
RE_WD_STUCK = re.compile(r'\[lifecycle_watchdog\]: \S+ 이 \w+ 에 멈춰 있다')
RE_WD_ALL = re.compile(r'\[lifecycle_watchdog\]: 감시 대상이 모두 active')
SEED_BAD_S = 15.0         # [sim s] lost 이거나 추정 오차 > 0.30 m 가 이만큼 이어지면 자세를 알려 준다 (준비 단계만)
SEED_GAP_S = 20.0         # [sim s] 같은 로봇 연속 알림 간격
SEED_MAX = 5              # 로봇당 상한


def watchdog_ok(all_active_pos: int, stuck_pos: int) -> bool:
    """lifecycle_watchdog 가 '모두 active' 를 마지막 '멈춰 있다' 경고 뒤에 냈나 (로그 안 위치로 비교, 없으면 -1)."""
    return all_active_pos >= 0 and all_active_pos > stuck_pos


def seed_due(now: float, bad_since, last_seed, n_seeds: int) -> bool:
    """준비 단계 자세 알림 규칙 (rclpy 비의존 — 시험용)."""
    if bad_since is None or n_seeds >= SEED_MAX:
        return False
    if now - bad_since < SEED_BAD_S:
        return False
    return last_seed is None or now - last_seed >= SEED_GAP_S


STATUS_NAME = {0: 'PENDING', 1: 'IN_PROGRESS', 2: 'COMPLETED', 3: 'FAILED'}


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_stamped(x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class Recorder(Node):
    """기록 · 준비 판정 · 작업 게시 (단일 스레드)."""

    def __init__(self, args, plan: dict):
        super().__init__('p5_recorder', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.args = args
        self.plan = plan
        self.robots = list(plan.get('robots', ROBOTS))
        self.mode = plan['mode']
        self.out = args.out
        os.makedirs(self.out, exist_ok=True)
        self.f_events = open(os.path.join(self.out, 'events.jsonl'), 'a', buffering=1,
                             encoding='utf-8')
        self.f_gt = open(os.path.join(self.out, 'gt.csv'), 'a', buffering=1, encoding='utf-8')
        self.f_est = open(os.path.join(self.out, 'est.csv'), 'a', buffering=1, encoding='utf-8')
        self.f_p2p = open(os.path.join(self.out, 'p2p.csv'), 'a', buffering=1, encoding='utf-8')
        if self.f_gt.tell() == 0:
            self.f_gt.write('t,robot,x,y,yaw,v\n')
        if self.f_est.tell() == 0:
            self.f_est.write('t,robot,x,y,yaw\n')
        if self.f_p2p.tell() == 0:
            self.f_p2p.write('t,w,topic,count,bytes\n')
        self.wall0 = time.time()
        self.state = 'WAIT_READY'
        self.t_ready = None
        self.t_base = None
        self.posted = set()
        self.notes = []
        # 로봇별 최신 값
        self.gt = {}
        self.gt_last_t = {}
        self.est = {}
        self.est_last_t = {}
        self.phase = {}
        self.phase_since = {}
        self.lost = {}
        self.agent = {}
        self.completed = {}          # task_id → (robot, t)
        self.status_seen = 0.0
        self.loc_ok_since = {}
        self.hold_first = set()
        self.hold_unmanaged = set()
        self.launch_log_pos = 0
        self.p2p = {}
        self.last_sim = 0.0
        self.last_sim_wall = time.time()
        self.idle_all_since = None
        # 준비: lifecycle_manager is_active
        self.managers = ['/lifecycle_manager_map'] + [
            f'/{r}/lifecycle_manager_{s}' for r in self.robots
            for s in ('localization', 'navigation')]
        self.mgr_active = {m: False for m in self.managers}
        self.mgr_client = {m: self.create_client(Trigger, f'{m}/is_active') for m in self.managers}
        self.mgr_pending = {}
        self.lifecycle_assumed = False
        self.lifecycle_watchdog = False
        self.wd_all_pos = -1
        self.wd_stuck_pos = -1
        # 준비 단계 자세 알림 (하네스 actions.seed_pose 와 같은 묶음: initialpose 3 회 + map EKF set_pose
        # + AMCL 무이동 갱신)
        self.bad_since = {}
        self.last_seed = {}
        self.n_seeds = {}
        self.pub_init = {r: self.create_publisher(PoseWithCovarianceStamped, f'/{r}/initialpose',
                                                  10) for r in self.robots}
        self.cli_ekf = ({r: self.create_client(SetPose, f'/{r}/ekf_filter_node_map/set_pose')
                         for r in self.robots} if SetPose is not None else {})
        self.cli_nomotion = {r: self.create_client(Empty, f'/{r}/request_nomotion_update')
                             for r in self.robots}
        # 구독
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/ground_truth/odom',
                                     lambda m, r=r: self._on_gt(r, m), BEST)
            self.create_subscription(Odometry, f'/{r}/odometry/filtered_map',
                                     lambda m, r=r: self._on_est(r, m), BEST)
            self.create_subscription(String, f'/{r}/executor/phase',
                                     lambda m, r=r: self._on_phase(r, m), LATCHED)
            self.create_subscription(Bool, f'/{r}/localization/lost',
                                     lambda m, r=r: self._on_lost(r, m), LATCHED)
            self.create_subscription(Task, f'/{r}/task_status',
                                     lambda m, r=r: self._on_task_status(r, m), RELIABLE100)
            self.create_subscription(Bool, f'/{r}/traffic/hold',
                                     lambda m, r=r: self._ev('hold', r, hold=bool(m.data)),
                                     LATCHED)
            self.create_subscription(Path, f'/{r}/plan',
                                     lambda m, r=r: self._on_plan(r, m), BEST)
            self.create_subscription(AgentState, f'/{r}/fleet_agent/state',
                                     lambda m, r=r: self._on_agent_state(r, m), LATCHED)
            self.create_subscription(String, f'/{r}/fleet_agent/events',
                                     lambda m, r=r: self._on_agent_event(r, m), RELIABLE100)
        self.create_subscription(Task, '/fleet/task_events', self._on_fleet_event, RELIABLE100)
        self.create_subscription(FleetStatus, '/fleet/status', self._on_fleet_status, 10)
        self.create_subscription(FleetHeartbeat, '/fleet/heartbeat', self._on_hb, BEST)
        self.create_subscription(DiagnosticArray, '/fleet/traffic_events', self._on_traffic,
                                 RELIABLE100)
        for topic, typ in (('/fleet_p2p/vote', FleetVoteBatch), ('/fleet_p2p/beacon', AgentBeacon),
                           ('/fleet_p2p/neg', NegState), ('/fleet/heartbeat', FleetHeartbeat)):
            self.create_subscription(typ, topic, lambda b, t=topic: self._on_raw(t, b), BEST,
                                     raw=True)
        self.pub_board = self.create_publisher(Task, '/fleet/task_board', BOARD)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self._on_tick, clock=steady)
        self.create_timer(1.0, self._flush_p2p, clock=steady)
        self._write_meta()
        self.get_logger().info(f'p5_recorder: mode={self.mode} robots={self.robots} '
                               f'out={self.out} t_start={plan["t_start"]} '
                               f'tasks={len(plan["tasks"])}')

    # ------------------------------------------------------------------ 공통
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ev(self, kind: str, robot: str = '', **kw) -> None:
        rec = {'t': round(self.now(), 3), 'w': round(time.time(), 3), 'k': kind}
        if robot:
            rec['r'] = robot
        rec.update(kw)
        self.f_events.write(json.dumps(rec, ensure_ascii=False, sort_keys=True,
                                       default=str) + '\n')

    def _write_meta(self) -> None:
        meta = {'plan': self.plan, 'args': vars(self.args), 'robots': self.robots,
                'mode': self.mode, 'started_wall': self.wall0,
                'env': {k: v for k, v in os.environ.items()
                        if k.startswith(('ROS_', 'RMW_', 'FASTRTPS', 'IGN_', 'AMR_FLEET', 'P5_'))}}
        for key, path in (('zones_file', self.args.zones), ('behavior_file', self.args.behavior),
                          ('traffic_file', self.args.traffic)):
            if path and os.path.isfile(path):
                with open(path, encoding='utf-8') as f:
                    meta[key.replace('_file', '_text')] = f.read()
                meta[key] = path
        self._atomic_json('meta.json', meta)

    def _atomic_json(self, name: str, obj) -> None:
        path = os.path.join(self.out, name)
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(obj, f, ensure_ascii=False, indent=1, default=str)
        os.replace(tmp, path)

    # ------------------------------------------------------------------ 콜백
    def _on_gt(self, r: str, m: Odometry) -> None:
        t = stamp_s(m.header.stamp)
        p = m.pose.pose
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.gt[r] = (t, p.position.x, p.position.y, yaw_of(p.orientation), v)
        if t - self.gt_last_t.get(r, -1e9) >= 0.1 - 1e-6:
            self.gt_last_t[r] = t
            self.f_gt.write(f'{t:.3f},{r},{p.position.x:.4f},{p.position.y:.4f},'
                            f'{yaw_of(p.orientation):.4f},{v:.3f}\n')

    def _on_est(self, r: str, m: Odometry) -> None:
        t = stamp_s(m.header.stamp)
        p = m.pose.pose
        self.est[r] = (t, p.position.x, p.position.y, yaw_of(p.orientation))
        if t - self.est_last_t.get(r, -1e9) >= 0.2 - 1e-6:
            self.est_last_t[r] = t
            self.f_est.write(f'{t:.3f},{r},{p.position.x:.4f},{p.position.y:.4f},'
                             f'{yaw_of(p.orientation):.4f}\n')

    def _on_phase(self, r: str, m: String) -> None:
        ph = str(m.data).upper()
        if self.phase.get(r) != ph:
            self.phase[r] = ph
            self.phase_since[r] = self.now()
            self._ev('phase', r, phase=ph)

    def _on_lost(self, r: str, m: Bool) -> None:
        if self.lost.get(r) != bool(m.data):
            self.lost[r] = bool(m.data)
            self._ev('lost', r, lost=bool(m.data))

    def _task_fields(self, m: Task) -> dict:
        return {'task_id': m.task_id, 'robot_id': m.robot_id, 'status': int(m.status),
                'status_name': STATUS_NAME.get(int(m.status), str(int(m.status))),
                'epoch': int(m.epoch), 'leader': m.leader_id, 'seq': int(m.seq),
                'stamp': stamp_s(m.header.stamp)}

    def _on_task_status(self, r: str, m: Task) -> None:
        f = self._task_fields(m)
        self._ev('task_status', r, **f)
        if int(m.status) == Task.STATUS_COMPLETED and m.task_id:
            self.completed.setdefault(m.task_id, (r, self.now()))

    def _on_fleet_event(self, m: Task) -> None:
        self._ev('fleet_event', '', **self._task_fields(m))

    def _on_fleet_status(self, m: FleetStatus) -> None:
        self.status_seen = time.time()

    def _on_hb(self, m: FleetHeartbeat) -> None:
        self._ev('hb', '', epoch=int(m.epoch), leader=m.leader_id, seq=int(m.seq),
                 role=int(m.role), stamp=stamp_s(m.stamp))

    def _on_traffic(self, m: DiagnosticArray) -> None:
        for s in m.status:
            lv = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
            self._ev('traffic', s.hardware_id, level=int(lv), name=s.name, msg=s.message[:300])

    def _on_plan(self, r: str, m: Path) -> None:
        n = len(m.poses)
        if n == 0:
            self._ev('plan', r, n=0)
            return
        a, b = m.poses[0].pose.position, m.poses[-1].pose.position
        self._ev('plan', r, n=n, x0=round(a.x, 3), y0=round(a.y, 3), x1=round(b.x, 3),
                 y1=round(b.y, 3), stamp=stamp_s(m.header.stamp))

    def _on_agent_state(self, r: str, m: AgentState) -> None:
        d = {'mode': int(m.mode), 'reason': m.mode_reason, 'server': int(m.server_status),
             'vote_epoch': int(m.vote_epoch), 'fence_epoch': int(m.fence_epoch),
             'fence_leader': m.fence_leader, 'held': sorted(m.held),
             'requested': sorted(m.requested), 'zone_phase': m.zone_phase,
             'denied': sorted(m.denied), 'task': m.current_task_id, 'task_phase': m.task_phase,
             'deadlock': sorted(m.deadlock_members), 'victim': m.deadlock_victim,
             'low_epoch_rejected': int(m.low_epoch_rejected), 'stamp': stamp_s(m.stamp)}
        prev = self.agent.get(r)
        self.agent[r] = d
        key = {k: v for k, v in d.items() if k != 'stamp'}
        if prev is None or {k: v for k, v in prev.items() if k != 'stamp'} != key:
            self._ev('agent_state', r, **d)

    def _on_agent_event(self, r: str, m: String) -> None:
        try:
            d = json.loads(m.data)
        except ValueError:
            d = {'raw': m.data[:500]}
        self._ev('agent_event', r, ev=d)

    def _on_raw(self, topic: str, data) -> None:
        c = self.p2p.setdefault(topic, [0, 0])
        c[0] += 1
        c[1] += len(data)

    def _flush_p2p(self) -> None:
        t, w = self.now(), time.time()
        for topic, (n, b) in sorted(self.p2p.items()):
            self.f_p2p.write(f'{t:.3f},{w:.3f},{topic},{n},{b}\n')
        self.p2p.clear()

    # ------------------------------------------------------------------ 준비 판정
    def _scan_launch_log(self) -> None:
        path = self.args.launch_log
        if not path or not os.path.isfile(path):
            return
        try:
            with open(path, 'rb') as f:
                f.seek(self.launch_log_pos)
                chunk = f.read()
                self.launch_log_pos += len(chunk)
        except OSError:
            return
        text = chunk.decode('utf-8', errors='replace')
        base = self.launch_log_pos - len(chunk)
        for mm in RE_WD_STUCK.finditer(text):
            self.wd_stuck_pos = base + mm.start()
        for mm in RE_WD_ALL.finditer(text):
            self.wd_all_pos = base + mm.start()
        for mm in RE_HOLD_FIRST.finditer(text):
            if mm.group(1) not in self.hold_first:
                self.hold_first.add(mm.group(1))
                self._ev('hold_first', mm.group(1))
        for mm in RE_HOLD_UNMANAGED.finditer(text):
            if mm.group(1) not in self.hold_unmanaged:
                self.hold_unmanaged.add(mm.group(1))
                self._ev('hold_unmanaged', mm.group(1))

    def _poll_lifecycle(self) -> None:
        for m in self.managers:
            if self.mgr_active[m]:
                continue
            pend = self.mgr_pending.get(m)
            if pend is not None:
                fut, t0 = pend
                if fut.done():
                    del self.mgr_pending[m]
                    try:
                        res = fut.result()
                        self.mgr_active[m] = bool(res is not None and res.success)
                    except Exception:  # noqa: B902 — 응답 실패는 다음 주기에 다시
                        pass
                elif time.time() - t0 > 15.0:
                    del self.mgr_pending[m]       # 응답 없음 — 버리고 다시 (같은 스레드라 잠금 경합 없음)
                    try:
                        self.mgr_client[m].remove_pending_request(fut)
                    except Exception:  # noqa: B902
                        pass
                continue
            c = self.mgr_client[m]
            if c.service_is_ready():
                self.mgr_pending[m] = (c.call_async(Trigger.Request()), time.time())

    def _ready_report(self) -> dict:
        now = self.now()
        rep = {}
        for r in self.robots:
            g, e = self.gt.get(r), self.est.get(r)
            err = math.hypot(g[1] - e[1], g[2] - e[2]) if g and e else math.inf
            ok_loc = err <= 0.30 and self.lost.get(r) is False
            if ok_loc:
                self.loc_ok_since.setdefault(r, now)
            else:
                self.loc_ok_since.pop(r, None)
            a = self.agent.get(r)
            rep[r] = {'phase': self.phase.get(r), 'lost': self.lost.get(r),
                      'loc_err': None if math.isinf(err) else round(err, 3),
                      'loc_ok_s': round(now - self.loc_ok_since[r], 1)
                      if r in self.loc_ok_since else 0.0,
                      'hold_first': r in self.hold_first,
                      'agent_mode': None if a is None else a['mode'],
                      'agent_server': None if a is None else a['server']}
        return rep

    def _is_ready(self, rep: dict) -> bool:
        for r, d in rep.items():
            if d['phase'] != 'IDLE' or d['lost'] is not False or d['loc_ok_s'] < 3.0:
                return False
            if not d['hold_first']:
                return False
            if self.mode == 'distributed' and (d['agent_mode'] != 0 or d['agent_server'] != 1):
                return False
        if self.mode == 'central' and time.time() - self.status_seen > 5.0:
            return False
        if not all(self.mgr_active.values()):
            if watchdog_ok(self.wd_all_pos, self.wd_stuck_pos):
                # 관리자 응답이 유실돼 is_active 는 false 지만 lifecycle_watchdog 가 관리 노드를 직접 올렸다 (하네스
                # cases.wait_lifecycle_active 의 wedge 폴백과 같은 근거)
                if not self.lifecycle_watchdog:
                    self.lifecycle_watchdog = True
                    self._ev('note', '', text='lifecycle_watchdog: 감시 대상이 모두 active — 응답 없는 관리자 '
                             + ','.join(m for m, ok in self.mgr_active.items() if not ok))
                return True
            if time.time() - self.wall0 < self.args.lifecycle_assume_s:
                return False
            if not self.lifecycle_assumed:
                self.lifecycle_assumed = True
                self.notes.append('lifecycle_assumed: 비활성 응답 ' + ','.join(
                    m for m, ok in self.mgr_active.items() if not ok))
                self._ev('note', '', text=self.notes[-1])
        return True

    def _maybe_seed(self, rep: dict, now: float) -> None:
        """
        준비 단계(작업 게시 전)에서만: 위치추정이 lost 이거나 추정 오차 > 0.30 m 가 SEED_BAD_S 이어지면 정답 자세를 알려 준다.

        로봇은 작업 전이라 대기 구역에 서 있다 — 운영자가 로봇 자세를 알려 주는 것과 같다 (하네스 actions.seed_pose, 판정 대상 아님).
        기동 때 lifecycle 응답 유실로 AMCL 이 늦게 활성되면 초기 자세를 놓치고, 대칭 창고에서 재위치추정이 별칭에 갇힌다 (파일럿
        p5_pilot_p1_distributed: amr_02 · 03 · 05 lost 반복, 오차 46–54 m). 사건 'seed' 로 남고
        summary.seeds 로 센다.
        """
        for r, d in rep.items():
            bad = d['lost'] is True or (d['loc_err'] is not None and d['loc_err'] > 0.30)
            if not bad:
                self.bad_since.pop(r, None)
                continue
            self.bad_since.setdefault(r, now)
            if not seed_due(now, self.bad_since.get(r), self.last_seed.get(r),
                            self.n_seeds.get(r, 0)):
                continue
            g = self.gt.get(r)
            if g is None:
                continue
            _, x, y, yaw, _ = g
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose = pose_stamped(x, y, yaw).pose
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05 ** 2
            msg.pose.covariance[35] = math.radians(3.0) ** 2
            for _ in range(3):
                self.pub_init[r].publish(msg)
            calls = []
            if r in self.cli_ekf and self.cli_ekf[r].service_is_ready():
                req = SetPose.Request()
                req.pose = msg
                self.cli_ekf[r].call_async(req)
                calls.append('ekf_set_pose')
            if self.cli_nomotion[r].service_is_ready():
                self.cli_nomotion[r].call_async(Empty.Request())
                calls.append('nomotion')
            self.last_seed[r] = now
            self.n_seeds[r] = self.n_seeds.get(r, 0) + 1
            self._ev('seed', r, x=round(x, 3), y=round(y, 3), yaw=round(yaw, 4),
                     lost=d['lost'], loc_err=d['loc_err'], calls=calls, n=self.n_seeds[r])

    # ------------------------------------------------------------------ 주기
    def _on_tick(self) -> None:
        now, wall = self.now(), time.time()
        if now > self.last_sim + 1e-6:
            self.last_sim, self.last_sim_wall = now, wall
        elif now > 0.0 and wall - self.last_sim_wall > self.args.clock_stall_s:
            self._finish('CLOCK_STALL', 4)
            return
        self._scan_launch_log()
        if self.state == 'WAIT_READY':
            self._poll_lifecycle()
            rep = self._ready_report()
            self._maybe_seed(rep, now)
            if int(wall) % 10 == 0:
                self._ev('ready_check', '', report=rep,
                         lifecycle={m: v for m, v in self.mgr_active.items() if not v})
            if self.args.skip_ready or self._is_ready(rep):
                self.t_ready = now
                lead = float(self.plan.get('lead_s', 0.0))
                self.t_base = max(float(self.plan['t_start']), now + lead)
                if self.t_base > float(self.plan['t_start']):
                    self.notes.append(f'LATE_START: 준비가 늦어 t_base {self.t_base:.1f} > t_start '
                                      f'{self.plan["t_start"]} (t_ready {now:.1f}, lead {lead:g})')
                self._ev('ready', '', t_ready=now, t_base=self.t_base, report=rep,
                         lifecycle_assumed=self.lifecycle_assumed,
                         lifecycle_watchdog=self.lifecycle_watchdog, seeds=self.n_seeds)
                self.state = 'RUN'
            elif wall - self.wall0 > self.args.ready_timeout_s:
                self._ev('ready_timeout', '', report=rep,
                         lifecycle={m: v for m, v in self.mgr_active.items() if not v})
                self._finish('READY_TIMEOUT', 3)
                return
        elif self.state == 'RUN':
            self._post_due(now)
            if self._should_end(now):
                self._finish('DONE', 0)
                return
        self._write_clock(now, wall)

    def _post_due(self, now: float) -> None:
        for task in self.plan['tasks']:
            tid = task['task_id']
            if tid in self.posted or now < self.t_base + float(task['offset_s']):
                continue
            msg = Task()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.task_id = tid
            msg.priority = int(task.get('priority', 100))
            msg.item_type = task.get('item_type', 'small')
            msg.item_mass = float(task.get('item_mass', 2.0))
            msg.pickup_pose = pose_stamped(*task['pickup'])
            msg.dropoff_pose = pose_stamped(*task['dropoff'])
            msg.status = Task.STATUS_PENDING
            msg.version = 1
            self.pub_board.publish(msg)
            self.posted.add(tid)
            self._ev('post', '', task_id=tid, pickup_dock=task.get('pickup_dock'),
                     dropoff_dock=task.get('dropoff_dock'), offset_s=task['offset_s'])

    def _should_end(self, now: float) -> bool:
        if now >= self.t_base + float(self.plan['horizon_s']):
            return True
        if not self.plan.get('end_when_done', True):
            return False
        if now < self.t_base + float(self.plan.get('min_end_s', 0.0)):
            return False
        ids = [t['task_id'] for t in self.plan['tasks']]
        if len(self.posted) < len(ids) or not all(t in self.completed for t in ids):
            self.idle_all_since = None
            return False
        if all(self.phase.get(r) == 'IDLE' for r in self.robots):
            if self.idle_all_since is None:
                self.idle_all_since = now
            return now - self.idle_all_since >= 5.0
        self.idle_all_since = None
        return False

    def _write_clock(self, now: float, wall: float) -> None:
        self._atomic_json('clock.json', {
            'sim': now, 'wall': wall, 'state': self.state, 't_ready': self.t_ready,
            't_base': self.t_base, 'posted': len(self.posted),
            'completed': len(self.completed)})

    def _finish(self, why: str, code: int) -> None:
        now = self.now()
        if why != 'DONE':
            # 진단: 이 참가자가 보는 그래프 (/clock 발행자 수 · 노드 이름) — 기동 실패 · /clock 정지의 원인 가르기용
            try:
                names = sorted(f'{ns.rstrip("/")}/{n}' for n, ns in
                               self.get_node_names_and_namespaces())
                self._atomic_json('graph_at_end.json', {
                    'why': why, 'sim': now, 'clock_publishers': self.count_publishers('/clock'),
                    'nodes': names, 'topics': len(self.get_topic_names_and_types())})
            except Exception as exc:  # noqa: B902 — 진단 실패는 끝내기를 막지 않는다
                self._ev('note', '', text=f'graph_at_end 실패: {exc}')
        self._flush_p2p()
        self._ev('end', '', why=why, code=code)
        summary = {'why': why, 'code': code, 'state': self.state, 't_ready': self.t_ready,
                   't_base': self.t_base, 't_end': now, 'wall_s': time.time() - self.wall0,
                   'posted': sorted(self.posted), 'completed': self.completed,
                   'notes': self.notes, 'lifecycle_assumed': self.lifecycle_assumed,
                   'lifecycle_watchdog': self.lifecycle_watchdog, 'seeds': self.n_seeds,
                   'hold_first': sorted(self.hold_first),
                   'hold_unmanaged': sorted(self.hold_unmanaged),
                   'lifecycle_inactive': [m for m, ok in self.mgr_active.items() if not ok]}
        self._atomic_json('summary.json', summary)
        self.state = 'END'
        self._write_clock(now, time.time())
        for f in (self.f_events, self.f_gt, self.f_est, self.f_p2p):
            f.flush()
        self.exit_code = code
        raise SystemExit(code)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--out', required=True)
    ap.add_argument('--plan', required=True, help='계획 JSON (p5_plan.py 가 만든다)')
    ap.add_argument('--launch-log', default='', help='스택 launch.log (실행기 hold 첫 값 줄)')
    ap.add_argument('--zones', default='')
    ap.add_argument('--behavior', default='')
    ap.add_argument('--traffic', default='')
    ap.add_argument('--ready-timeout-s', type=float, default=900.0, help='벽시계')
    ap.add_argument('--lifecycle-assume-s', type=float, default=600.0,
                    help='벽시계 — 이 뒤에는 is_active 응답 없는 관리자를 활성으로 본다 (기록)')
    ap.add_argument('--clock-stall-s', type=float, default=180.0, help='벽시계')
    ap.add_argument('--skip-ready', action='store_true',
                    help='파이프라인 시험 전용 (Gazebo 없음): 준비 판정을 건너뛴다')
    args = ap.parse_args(argv)
    with open(args.plan, encoding='utf-8') as f:
        plan = json.load(f)
    for key in ('mode', 't_start', 'horizon_s', 'tasks'):
        if key not in plan:
            print(f'계획에 {key} 가 없다', file=sys.stderr)
            return 2
    rclpy.init()
    node = Recorder(args, plan)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    code = 0
    try:
        ex.spin()
    except SystemExit as exc:
        code = int(exc.code or 0)
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
