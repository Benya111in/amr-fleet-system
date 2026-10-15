"""
fleet_agent 노드 — 로봇마다 하나 (보너스 4 P3). 순수 코어(agent/core.py)를 ROS 입출력에 잇기만 한다.

실행: ros2 run amr_fleet fleet_agent_node --ros-args -r __ns:=/amr_01 -p use_sim_time:=true ...
      (또는 python3 -m amr_fleet.agent.node — 같은 main)

토픽 · 서비스 (QoS 근거: amr_msgs 각 파일 머리, p3.md §2)
- Pub/Sub /fleet_p2p/vote    FleetVoteBatch  RELIABLE · KEEP_LAST 256
- Pub/Sub /fleet_p2p/beacon  AgentBeacon     BEST_EFFORT (발행 KEEP_LAST 1, 구독 10 — 다섯 발행자)
- Pub/Sub /fleet_p2p/neg     NegState        RELIABLE · KEEP_LAST 10 (DEGRADED 에서만 발행)
- Sub /fleet/heartbeat       FleetHeartbeat  BEST_EFFORT
- Sub /fleet/task_board      Task            RELIABLE · TRANSIENT_LOCAL · 256 (서버와 따로 오는 작업 경로)
- Sub odometry/filtered_map (Odometry), localization/lost (Bool, latched), battery_state,
      task_status (Task — 중앙 · 협상 작업이 끝나면 확정 전 잠금 요청을 놓는다),
      executor/phase (String, latched — 실제 실행기가 IDLE 일 때만 협상 입찰, 경로 게이트의 움직이는 단계),
      plan (nav_msgs/Path, path_gate.enabled 일 때 — 경로 게이트: 사슬 사전 요청 · 결정점 정지 · 지나온 구역 해제)
- Pub fleet_agent/state      AgentState      RELIABLE · TRANSIENT_LOCAL · 1 (바뀔 때 + 1 s 마다)
- Pub fleet_agent/events     String(JSON)    RELIABLE · 100 (사건 — 시험 · 로그), event_log 파일에도 JSONL
- Srv fleet_agent/zone_command (ZoneCommand) — BT 노드가 부른다
- Srv assign_task (AssignTask, proxy_assign_task=true 일 때) — 에포크 펜스 프록시. 받으면 executor_service
      (기본 executor/assign_task) 로 넘기고, 낮은 (epoch, leader_id) · NORMAL 아닌 모드면 실행기까지 가지 않는다.
      작업 잠정 잠금은 실행기가 받은 뒤에(시간 초과면 받았다고 보고) 시작한다 — 거부 · 실행기 없음이면 잡지 않는다.
      실행기의 서비스 이름을 executor/assign_task 로 바꾸는 리매핑은 behavior.launch.py fleet_mode:=distributed.
- SrvC executor_service (sim_execution=false) — DEGRADED 협상 낙찰을 실제 실행기에 넘긴다 (작업 내용은
      /fleet/task_board 게시물). 응답 · task_status 를 코어에 알린다 (on_negotiated_result · on_task_status).

통신 모의: 받는 쪽 EmulatedInbox (링크 송신자 → 나). 자기 앞 투표는 DDS 를 거치지 않고 곧바로 처리한다.
시각: 노드 시계 (use_sim_time 이면 /clock). /clock 을 받기 전(시각 0)에는 기동하지 않는다.
"""

from __future__ import annotations

import copy
import json
import math
import os
import pathlib
import time
from typing import Any, Dict, List, Optional

from amr_fleet.agent import codec
from amr_fleet.agent.core import (
    CH_TASK, CH_ZONE, CoreConfig, FleetAgentCore, GATE_ZONE_MAP, Outbox,
)
from amr_fleet.agent.modes import ModeConfig
from amr_fleet.agent.sim_exec import SimExecConfig
from amr_fleet.comm_emulator import FLEET, from_node_params
from amr_fleet.distributed.failure_detector import DetectorConfig
from amr_fleet.distributed.store import FileStore
from amr_fleet.distributed.vote_core import VoteConfig
from amr_fleet.robot_status import battery_percent
from amr_fleet.timer_queue import EmulatedInbox
from amr_msgs.msg import AgentBeacon, AgentState, FleetHeartbeat, FleetVoteBatch, NegState, Task
from amr_msgs.srv import AssignTask, ZoneCommand
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

VOTE_QOS = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE)
BEACON_PUB_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
BEACON_SUB_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
NEG_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
HB_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
BOARD_QOS = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

DEFAULT_MEMBERS = 'amr_01,amr_02,amr_03,amr_04,amr_05'
# 통신 모의 설정 파일의 환경 변수 대체값 (보너스 4 P5 — 로봇 스택 런치를 고치지 않고 다섯 에이전트에 같은 장애 일정을 준다).
# 파라미터 comm_model_file 이 비었을 때만 쓰고, 쓰면 모의를 켠다 (simulate_latency 파라미터와 무관 — 파일이 정한
# 링크 규칙 · 분할 · 단절이 효과를 내려면 모의가 켜져 있어야 한다). 기본(변수 없음)은 예전과 같다.
COMM_MODEL_ENV = 'AMR_FLEET_COMM_MODEL_FILE'
PARAMS: Dict[str, Any] = {
    'robot_id': '',                       # '' = 네임스페이스에서 (/amr_01 → amr_01)
    'members': DEFAULT_MEMBERS,           # 정적 멤버 (쉼표 문자열 또는 배열) — 정족수는 그 과반
    'store_dir': '',                      # '' = $ROS_WS/var/fleet_agent. 볼륨 경로여야 한다 (재기동에도 남게)
    'tick_hz': 10.0,
    'beacon_period_s': 0.5,
    'beacon_ttl_s': 2.0,
    'round_s': 0.2,
    'vote.tentative_s': 15.0,
    'vote.retry_s': 0.5,
    'vote.backoff_min_s': 0.5,
    'vote.backoff_max_s': 3.0,
    'task_hold_s': 60.0,
    'detector.period_s': 0.5,
    'detector.suspect_s': 1.5,
    'detector.dead_s': 5.0,
    'modes.resync_s': 30.0,
    'modes.isolate_s': 2.0,
    'modes.boot_grace_s': 3.0,
    'modes.unknown_dead_s': 5.0,
    'nominal_speed': 1.0,
    'release_gate': 'none',               # none | zone_map (zones_file 의 다각형으로 몸체가 밖인지 확인)
    'zones_file': '',                     # traffic_zones.yaml (release_gate=zone_map)
    'body_radius_m': 0.36,
    'accept_unfenced': True,
    'sim_execution': False,               # 가짜 실행기 (Gazebo · 실행기 없는 시험)
    'sim.pickup_s': 2.0,
    'sim.deliver_s': 3.0,
    'sim.commit_timeout_s': 30.0,
    'initial_pose': [math.nan, math.nan],  # 시험용 고정 자세 (odometry 가 없을 때)
    'simulate_latency': False,
    'comm_latency_ms': [0.0, 100.0],
    'drop_rate': 0.0,
    'seed': 1,
    'comm_model_file': '',
    'proxy_assign_task': False,
    'executor_service': 'executor/assign_task',
    'proxy_timeout_s': 3.0,
    'path_gate.enabled': False,           # 경로 게이트 (agent/path_gate.py) — 분산 모드에서 켠다
    'path_gate.prerequest_m': 1.0,        # 결정점보다 이만큼 먼저 요청 [m]
    'traffic_params_file': '',            # '' = share/.../config/traffic.yaml (요청 거리 · 사슬 · 반경)
    'nego.stale_lock_gc': True,           # 협상 표의 낡은 LOCKED 정리 (core.CoreConfig.stale_lock_gc)
    'event_log': '',                      # JSONL 사건 로그 경로 ('' = 파일 없음)
}


def _share_file(name: str) -> str:
    """share/amr_fleet/config/<name> (설치 공간), 없으면 소스 트리의 config/."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('amr_fleet'), 'config', name)
        if os.path.isfile(path):
            return path
    except Exception:  # noqa: B902 — 색인이 없으면 소스 트리
        pass
    return str(pathlib.Path(__file__).resolve().parents[2] / 'config' / name)


def comm_settings(simulate: bool, model_file: str, environ=None):
    """
    (모의 켬, 설정 파일 경로): 파라미터 comm_model_file 이 우선.

    파라미터가 비었고 환경 변수 COMM_MODEL_ENV 가 경로를 주면 (True, 그 경로) — 모의를 켠다. 둘 다 없으면 파라미터
    그대로 (simulate, '').
    """
    value = str(model_file or '').strip()
    if value:
        return bool(simulate), value
    env = os.environ if environ is None else environ
    path = str(env.get(COMM_MODEL_ENV, '') or '').strip()
    if path:
        return True, path
    return bool(simulate), ''


def parse_members(value: Any) -> List[str]:
    """멤버 파라미터: 쉼표 문자열 또는 문자열 배열."""
    if isinstance(value, str):
        items = [s.strip() for s in value.split(',')]
    else:
        items = [str(s).strip() for s in value]
    return [s for s in items if s]


class FleetAgentNode(Node):
    """로봇 1 대의 분산 플릿 에이전트."""

    def __init__(self, **kwargs):
        super().__init__('fleet_agent', **kwargs)
        for name, value in PARAMS.items():
            dyn = name in ('members', 'comm_latency_ms', 'initial_pose')
            self.declare_parameter(name, value, ParameterDescriptor(dynamic_typing=dyn))
        p = self._p
        self.me = p('robot_id') or self.get_namespace().strip('/').split('/')[-1]
        members = parse_members(p('members'))
        if self.me not in members:
            raise ValueError(f'robot_id {self.me!r} 가 members {members} 에 없다')
        vote = VoteConfig(tuple(members), tentative_s=float(p('vote.tentative_s')),
                          retry_s=float(p('vote.retry_s')),
                          backoff_min_s=float(p('vote.backoff_min_s')),
                          backoff_max_s=float(p('vote.backoff_max_s')))
        polygons = {}
        if p('zones_file'):
            from amr_fleet.traffic_zones import load_layout
            zones, _ = load_layout(str(p('zones_file')))
            polygons = {f'zone:{z.zone_id}': z.polygon for z in zones if z.polygon is not None}
        self._path_gate = None
        if bool(p('path_gate.enabled')):
            if not p('zones_file'):
                raise ValueError('path_gate.enabled 에는 zones_file 이 필요하다')
            from amr_fleet.agent.path_gate import load_path_gate
            traffic = str(p('traffic_params_file')) or _share_file('traffic.yaml')
            self._path_gate = load_path_gate(str(p('zones_file')), traffic,
                                             float(p('path_gate.prerequest_m')))
        self.cfg = CoreConfig(
            robot_id=self.me, members=tuple(members), zone_vote=vote, task_vote=vote,
            task_hold_s=float(p('task_hold_s')),
            detector=DetectorConfig(period_s=float(p('detector.period_s')),
                                    suspect_s=float(p('detector.suspect_s')),
                                    dead_s=float(p('detector.dead_s')),
                                    check_period_s=1.0 / float(p('tick_hz'))),
            modes=ModeConfig(resync_s=float(p('modes.resync_s')),
                             isolate_s=float(p('modes.isolate_s')),
                             boot_grace_s=float(p('modes.boot_grace_s')),
                             unknown_dead_s=float(p('modes.unknown_dead_s'))),
            beacon_period_s=float(p('beacon_period_s')), beacon_ttl_s=float(p('beacon_ttl_s')),
            round_s=float(p('round_s')), nominal_speed=float(p('nominal_speed')),
            release_gate=str(p('release_gate')), body_radius_m=float(p('body_radius_m')),
            accept_unfenced=bool(p('accept_unfenced')),
            stale_lock_gc=bool(p('nego.stale_lock_gc')),
            sim_exec=(SimExecConfig(pickup_s=float(p('sim.pickup_s')),
                                    deliver_s=float(p('sim.deliver_s')),
                                    commit_timeout_s=float(p('sim.commit_timeout_s')))
                      if bool(p('sim_execution')) else None))
        if self.cfg.release_gate == GATE_ZONE_MAP and not polygons:
            raise ValueError('release_gate=zone_map 에는 zones_file 이 필요하다')
        self._polygons = polygons
        root = p('store_dir') or os.path.join(os.environ.get('ROS_WS', '.'), 'var', 'fleet_agent')
        self._store_dir = pathlib.Path(root) / self.me
        simulate, model_file = comm_settings(bool(p('simulate_latency')),
                                             str(p('comm_model_file')))
        self._comm = from_node_params(simulate, p('comm_latency_ms'), float(p('drop_rate')),
                                      int(p('seed')), model_file)
        pose = list(p('initial_pose'))
        self._initial_pose = (None if len(pose) < 2 or any(math.isnan(float(v)) for v in pose[:2])
                              else (float(pose[0]), float(pose[1])))
        self._event_log = str(p('event_log'))
        self.core: Optional[FleetAgentCore] = None
        self._phase: Optional[str] = None
        self._last_state_key = None
        self._last_state_pub = -1.0
        self._pending_board: List[Task] = []

        # 받는 쪽 통신 모의 (토픽마다 큐, 같은 링크 난수열)
        self._in_vote = EmulatedInbox(self, self._comm, self.me, self._deliver_votes)
        self._in_beacon = EmulatedInbox(self, self._comm, self.me, self._deliver_beacon)
        self._in_neg = EmulatedInbox(self, self._comm, self.me, self._deliver_neg)
        self._in_hb = EmulatedInbox(self, self._comm, self.me, self._deliver_hb)

        self._pub_vote = self.create_publisher(FleetVoteBatch, '/fleet_p2p/vote', VOTE_QOS)
        self._pub_beacon = self.create_publisher(AgentBeacon, '/fleet_p2p/beacon', BEACON_PUB_QOS)
        self._pub_neg = self.create_publisher(NegState, '/fleet_p2p/neg', NEG_QOS)
        self._pub_state = self.create_publisher(AgentState, 'fleet_agent/state', LATCHED_QOS)
        self._pub_events = self.create_publisher(String, 'fleet_agent/events', EVENTS_QOS)
        self.create_subscription(FleetVoteBatch, '/fleet_p2p/vote', self._on_vote, VOTE_QOS)
        self.create_subscription(AgentBeacon, '/fleet_p2p/beacon', self._on_beacon,
                                 BEACON_SUB_QOS)
        self.create_subscription(NegState, '/fleet_p2p/neg', self._on_neg, NEG_QOS)
        self.create_subscription(FleetHeartbeat, '/fleet/heartbeat', self._on_hb, HB_QOS)
        self.create_subscription(Task, '/fleet/task_board', self._on_board, BOARD_QOS)
        self.create_subscription(Odometry, 'odometry/filtered_map', self._on_odom, 10)
        self.create_subscription(Bool, 'localization/lost', self._on_lost, LATCHED_QOS)
        self.create_subscription(BatteryState, 'battery_state', self._on_battery, 10)
        self.create_subscription(Task, 'task_status', self._on_task_status, 10)
        self.create_subscription(String, 'executor/phase', self._on_phase, LATCHED_QOS)
        if self._path_gate is not None:
            self.create_subscription(Path, 'plan', self._on_plan, 5)
        self.create_service(ZoneCommand, 'fleet_agent/zone_command', self._on_command)
        self._lost = False
        self._battery: Optional[float] = None
        self._exec_client = None
        # 프록시 서비스 · 실행기 클라이언트 · 시간 초과 타이머는 재진입 그룹에 둔다. 기본(상호 배제) 그룹에 두면
        # 응답을 기다리는 비동기 서비스 작업이 그룹을 쥔 채라 클라이언트 응답 · tick 타이머가 돌지 못한다
        # (launch_dev1 에서 amr_02 가 첫 프록시 호출 뒤 멈췄다).
        self._proxy_group = ReentrantCallbackGroup()
        if bool(p('proxy_assign_task')) or not bool(p('sim_execution')):
            self._exec_client = self.create_client(AssignTask, str(p('executor_service')),
                                                   callback_group=self._proxy_group)
        if bool(p('proxy_assign_task')):
            self.create_service(AssignTask, 'assign_task', self._on_assign,
                                callback_group=self._proxy_group)
        self._board_msgs: Dict[str, Task] = {}
        self._dispatching: Dict[str, Any] = {}     # 작업 → (보낸 벽시계, future) — 협상 낙찰 넘기기
        self._proxy_timeout_s = float(p('proxy_timeout_s'))
        self.create_timer(1.0 / float(p('tick_hz')), self._on_tick)
        self.get_logger().info(
            f'fleet_agent {self.me}: members={members}, store={self._store_dir}, '
            f'comm={self._comm.describe()}, sim_execution={p("sim_execution")}, '
            f'proxy={p("proxy_assign_task")}, release_gate={self.cfg.release_gate}, '
            f'path_gate={self._path_gate is not None}, stale_lock_gc={self.cfg.stale_lock_gc}')

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 기동 · 송신
    def _boot(self, now: float) -> None:
        d = self._store_dir
        d.mkdir(parents=True, exist_ok=True)
        self.core = FleetAgentCore.boot(
            self.cfg, FileStore(str(d / 'zone.json')), FileStore(str(d / 'task.json')),
            FileStore(str(d / 'fence.json')), now, inside_zones=None,
            zone_polygons=self._polygons, path_gate=self._path_gate)
        if self._phase is not None:
            self.core.set_executor_phase(self._phase)
        if self._initial_pose is not None:
            self.core.set_pose(self._initial_pose[0], self._initial_pose[1], 0.0)
        for t in self._pending_board:
            self.core.on_board(codec.task_to_board(t))
        self._pending_board.clear()
        self.get_logger().info(
            f'{self.me}: 기동 incarnation={self.core.zone.incarnation} '
            f'held={sorted(self.core.zone.authorized() | self.core.task.authorized())}')
        self._flush()

    def _flush(self) -> None:
        """코어 송신함을 비운다. 자기 앞 투표는 곧바로 다시 넣는다 (DDS 를 거치지 않음)."""
        core = self.core
        if core is None:
            return
        now = self._now()
        for _ in range(16):
            out: Outbox = core.drain()
            self._emit(out, now)
            for task_id in out.dispatch:
                self._dispatch_negotiated(task_id)
            mine = [(ch, v) for ch, v in out.votes if v.dst == self.me]
            others = [(ch, v) for ch, v in out.votes if v.dst != self.me]
            if others:
                self._pub_vote.publish(codec.votes_to_batch(self.me, others, now))
            if not mine:
                break
            for ch in (CH_ZONE, CH_TASK):
                msgs = [v for c, v in mine if c == ch]
                if msgs:
                    core.on_votes(ch, msgs, now)
        self._publish_state(now)

    def _emit(self, out: Outbox, now: float) -> None:
        if out.beacon is not None:
            self._pub_beacon.publish(codec.beacon_to_msg(out.beacon))
        if out.neg is not None:
            self._pub_neg.publish(codec.neg_to_msg(out.neg))
        for ev in out.events:
            ev = dict(ev, wall=round(Clock(clock_type=ClockType.SYSTEM_TIME).now().nanoseconds
                                     * 1e-9, 3))
            text = json.dumps(ev, ensure_ascii=False, sort_keys=True, default=str)
            self._pub_events.publish(String(data=text))
            if ev.get('event') in ('mode', 'fence_reject', 'pickup_start', 'failed',
                                   'deadlock', 'central_accept', 'central_rejected', 'boot'):
                self.get_logger().info(f'{self.me}: {text}')
            if self._event_log:
                try:
                    with open(self._event_log, 'a', encoding='utf-8') as f:
                        f.write(text + '\n')
                except OSError as exc:
                    self.get_logger().warn(f'사건 로그 쓰기 실패: {exc}')

    def _publish_state(self, now: float, force: bool = False) -> None:
        if self.core is None:
            return
        sv = self.core.state()
        key = sv.key()
        if force or key != self._last_state_key or now - self._last_state_pub >= 1.0:
            self._last_state_key = key
            self._last_state_pub = now
            self._pub_state.publish(codec.state_to_msg(sv, now))

    # ------------------------------------------------------------------ 주기
    def _on_tick(self) -> None:
        now = self._now()
        if now <= 0.0:
            return                              # /clock 전
        if self.core is None:
            self._boot(now)
            return
        self._check_dispatch_timeouts()
        self.core.tick(now)
        self._flush()

    # ------------------------------------------------------------------ 받기 (통신 모의 → 코어)
    def _on_vote(self, msg: FleetVoteBatch) -> None:
        if msg.src == self.me or self.core is None:
            return
        if not any(v.dst == self.me for v in msg.votes):
            return                              # 내 앞 메시지가 없는 묶음은 링크 모의도 건너뛴다
        self._in_vote.receive(msg.src, msg)

    def _deliver_votes(self, msg: FleetVoteBatch) -> None:
        if self.core is None:
            return
        try:
            items = codec.batch_for(msg, self.me)
        except ValueError as exc:
            self.get_logger().warn(f'{self.me}: 잘못된 투표 묶음 {exc}')
            return
        now = self._now()
        for ch in (CH_ZONE, CH_TASK):
            vs = [v for c, v in items if c == ch]
            if vs:
                self.core.on_votes(ch, vs, now)
        self._flush()

    def _on_beacon(self, msg: AgentBeacon) -> None:
        if msg.robot_id == self.me or self.core is None:
            return
        self._in_beacon.receive(msg.robot_id, msg)

    def _deliver_beacon(self, msg: AgentBeacon) -> None:
        if self.core is None:
            return
        try:
            b = codec.msg_to_beacon(msg)
        except (KeyError, ValueError) as exc:
            self.get_logger().warn(f'{self.me}: 잘못된 비컨 {exc}')
            return
        self.core.on_beacon(b, self._now())

    def _on_neg(self, msg: NegState) -> None:
        if msg.sender == self.me or self.core is None:
            return
        self._in_neg.receive(msg.sender, msg)

    def _deliver_neg(self, msg: NegState) -> None:
        if self.core is None:
            return
        self.core.on_neg(codec.msg_to_neg(msg), self._now())
        self._flush()

    def _on_hb(self, msg: FleetHeartbeat) -> None:
        if self.core is None:
            return
        self._in_hb.receive(FLEET, msg)

    def _deliver_hb(self, msg: FleetHeartbeat) -> None:
        if self.core is None:
            return
        self.core.on_heartbeat(int(msg.epoch), msg.leader_id, int(msg.seq), int(msg.role),
                               self._now())
        self._flush()

    def _on_board(self, msg: Task) -> None:
        if msg.status != Task.STATUS_PENDING or not msg.task_id:
            return
        self._board_msgs.setdefault(msg.task_id, msg)
        if self.core is None:
            self._pending_board.append(msg)
            return
        self.core.on_board(codec.task_to_board(msg))

    def _on_odom(self, msg: Odometry) -> None:
        if self.core is None:
            return
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pos = msg.pose.pose.position
        self.core.set_pose(pos.x, pos.y, yaw, healthy=not self._lost, battery=self._battery)

    def _on_lost(self, msg: Bool) -> None:
        self._lost = bool(msg.data)
        if self.core is not None and self.core.pose is not None:
            x, y, yaw = self.core.pose
            self.core.set_pose(x, y, yaw, healthy=not self._lost)

    def _on_task_status(self, msg: Task) -> None:
        if self.core is None or not msg.task_id:
            return
        self.core.on_task_status(msg.task_id, int(msg.status))
        self._flush()

    def _on_phase(self, msg: String) -> None:
        self._phase = str(msg.data)
        if self.core is not None:
            self.core.set_executor_phase(self._phase)

    def _on_plan(self, msg: Path) -> None:
        if self.core is None:
            return
        frame = msg.header.frame_id.lstrip('/')
        if frame not in ('', 'map'):
            self.get_logger().warn(f'{self.me}: plan 프레임 {frame!r} ≠ map — 무시',
                                   throttle_duration_sec=10.0)
            return
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.core.set_plan(pts if len(pts) >= 2 else None)

    # ------------------------------------------------------------------ 협상 낙찰 → 실제 실행기
    def _dispatch_negotiated(self, task_id: str) -> None:
        t = self._board_msgs.get(task_id)
        client = self._exec_client
        if t is None or client is None or not client.service_is_ready():
            why = 'no board task' if t is None else 'executor unavailable'
            self.core.on_negotiated_result(task_id, 'unavailable', why, now=self._now())
            return
        req = AssignTask.Request()
        req.task = copy.deepcopy(t)
        req.task.robot_id = self.me
        req.task.header.stamp = self.get_clock().now().to_msg()
        fut = client.call_async(req)
        self._dispatching[task_id] = (time.monotonic(), fut)
        fut.add_done_callback(lambda f, tid=task_id: self._on_dispatch_done(tid, f))

    def _on_dispatch_done(self, task_id: str, fut) -> None:
        entry = self._dispatching.get(task_id)
        if entry is None or entry[1] is not fut or self.core is None:
            return                                # 시간 초과로 이미 처리했다
        del self._dispatching[task_id]
        try:
            res = fut.result()
        except Exception as exc:  # noqa: B902 — 호출 실패는 종류를 가리지 않는다
            self.core.on_negotiated_result(task_id, 'unavailable', repr(exc), now=self._now())
        else:
            outcome = 'accepted' if res is not None and res.success else 'rejected'
            self.core.on_negotiated_result(task_id, outcome,
                                           '' if res is None else str(res.message),
                                           now=self._now())
        self._flush()

    def _check_dispatch_timeouts(self) -> None:
        now = time.monotonic()
        for task_id, (sent, fut) in list(self._dispatching.items()):
            if now - sent > self._proxy_timeout_s:
                del self._dispatching[task_id]
                fut.cancel()
                self.core.on_negotiated_result(task_id, 'timeout', 'executor timeout',
                                               now=self._now())

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery = battery_percent(msg.percentage, fallback=self._battery or 100.0)

    # ------------------------------------------------------------------ 서비스
    def _on_command(self, req: ZoneCommand.Request, resp: ZoneCommand.Response):
        if self.core is None:
            resp.accepted, resp.message = False, 'not booted'
            return resp
        try:
            ok, why = self.core.command(int(req.op), list(req.resources), int(req.priority),
                                        self._now())
        except Exception as exc:  # noqa: B902 — 서비스는 언제나 응답한다
            ok, why = False, f'error {exc!r}'
        resp.accepted, resp.message = bool(ok), str(why)
        self._flush()
        return resp

    async def _on_assign(self, req: AssignTask.Request, resp: AssignTask.Response):
        """
        에포크 펜스 프록시 — 거부한 명령은 실행기까지 가지 않는다.

        잠정 작업 잠금은 실행기가 받은 뒤에 시작한다 (core.on_central_result). 실행기가 거부하거나 없으면 아무것도
        잡지 않고, 시간 초과는 결과를 모르므로 받았다고 본다 (관리자도 robot_state 로 확인할 때까지 재할당을 미룬다).
        """
        resp.robot_id = self.me
        t = req.task
        if self.core is None:
            resp.success, resp.message = False, 'agent not booted'
            return resp
        ok, why = self.core.check_assignment(t.task_id, int(t.epoch), t.leader_id, self._now())
        self.core.note('assign_proxy', task_id=t.task_id, epoch=int(t.epoch),
                       leader=t.leader_id, accepted=bool(ok), reason=why)
        self._flush()
        if not ok:
            resp.success, resp.message = False, why
            return resp
        token = self.core.central_token(t.task_id)
        if not self._exec_client.service_is_ready():
            self._central_result(t.task_id, 'unavailable', token, 'executor unavailable')
            resp.success, resp.message = False, 'executor unavailable'
            return resp
        fut = self._exec_client.call_async(req)
        timer = self.create_timer(self._proxy_timeout_s, lambda: fut.cancel(),
                                  callback_group=self._proxy_group,
                                  clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:
            result = await fut
        finally:
            timer.cancel()
            self.destroy_timer(timer)
        if result is None:
            self._central_result(t.task_id, 'timeout', token, 'executor timeout')
            resp.success, resp.message = False, 'executor timeout'
            return resp
        self._central_result(t.task_id, 'accepted' if result.success else 'rejected', token,
                             str(result.message))
        return result

    def _central_result(self, task_id: str, outcome: str, token: int, reason: str) -> None:
        if self.core is None:
            return
        self.core.on_central_result(task_id, outcome, token, reason, now=self._now())
        self._flush()


def main(args=None):
    """python3 -m amr_fleet.agent.node."""
    rclpy.init(args=args)
    node = FleetAgentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass                                  # SIGINT · SIGTERM 로 컨텍스트가 먼저 닫힌 경우
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
